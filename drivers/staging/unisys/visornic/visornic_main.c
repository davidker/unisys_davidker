/* Copyright (c) 2012 - 2014 UNISYS CORPORATION
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 */

/* This driver lives in a sparlinux service partition, and registers to
 * receive director channels from the visorbus driver.  Currently, we don't need
 * any implementation here other than to accept the director channel.
 * The reason we need to accept the director channel is so the visorbus driver
 * will return successful DEVICE_CREATEs to CONTROL, which enables the partition
 * state to go RUNNING.
 */

#include "uniklog.h"
#include "diagnostics/appos_subsystems.h"
#include "timskmod.h"
#include "globals.h"
#include "visorbus.h"
#include "visorchannel.h"
#include "visorchipset.h"
#include "controlframework.h"
#include "iochannel.h"
#include "uisutils.h"

#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#define VISORNIC_STATS 0
#define VISORNIC_XMIT_TIMEOUT (5 * HZ)
#define VISORNIC_INFINITE_RESPONSE_WAIT 0
#define INTERRUPT_VECTOR_MASK 0x3F

/* MAX_BUF = 64 lines x 32 MAXVNIC x 80 characters
 *         = 163840 bytes ~ 40 pages
 */
#define MAX_BUF 163840

static spinlock_t dev_no_pool_lock;
static void *dev_no_pool;	/**< pool to grab device numbers from */

static int visornic_probe(struct visor_device *dev);
static void visornic_remove(struct visor_device *dev);
static int visornic_pause(struct visor_device *dev,
			  VISORBUS_STATE_COMPLETE_FUNC complete_func);
static int visornic_resume(struct visor_device *dev,
			   VISORBUS_STATE_COMPLETE_FUNC complete_func);

/* DEBUGFS declarations */
static ssize_t info_debugfs_read(struct file *file, char __user *buf,
				 size_t len, loff_t *offset);
static ssize_t enable_ints_write(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos);
static struct dentry *visornic_debugfs_dir;
static const struct file_operations debugfs_info_fops = {
	.read = info_debugfs_read,
};

static const struct file_operations debugfs_enable_ints_fops = {
	.write = enable_ints_write,
};

static struct workqueue_struct *visornic_serverdown_workqueue;
static struct workqueue_struct *visornic_timeout_reset_workqueue;

/* GUIDS for director channel type supported by this driver.  */
static struct visor_channeltype_descriptor visornic_channel_types[] = {
	/* Note that the only channel type we expect to be reported by the
	 * bus driver is the ULTRAVNIC channel.
	 */
	{ SPAR_VNIC_CHANNEL_PROTOCOL_UUID,
	  "ultravnic", 1, ULONG_MAX },
	{ NULL_UUID_LE, NULL, 0, 0 }
};

/* This is used to tell the visor bus driver which types of visor devices
 * we support, and what functions to call when a visor device that we support
 * is attached or removed.
 */
static struct visor_driver visornic_driver = {
	.name = MYDRVNAME,
	.version = VERSION,
	.vertag = NULL,
	.owner = THIS_MODULE,
	.channel_types = visornic_channel_types,
	.probe = visornic_probe,
	.remove = visornic_remove,
	.pause = visornic_pause,
	.resume = visornic_resume,
	.channel_interrupt = NULL,
};

/* This is the private data that we store for each device.
 * A pointer to this struct is kept in each "struct device", and can be
 * obtained using visor_get_drvdata(dev).
 */
/*
struct uisqueue_info {
	struct channel_header __iomem *chan;
	u64 packets_sent;
	u64 packets_received;
	u64 interrupts_sent;
	u64 interrupts_received;
	u64 max_not_empty_cnt;
	u64 total_wakeup_cnt;
	u64 non_empty_wakeup_cnt;
	struct {
		struct signal_queue_header reserved1;
		struct signal_queue_header reserved2;
	} safe_uis_queue;
};
*/

struct visor_thread_info {
	struct task_struct *task;
	struct completion has_stopped;
	int id;
};

/*
struct chaninfo {
	struct uisqueue_info *queueinfo;
	spinlock_t insertlock;
	struct visor_thread_info threadinfo;
};
*/

struct chanstat {
	unsigned long got_rcv;
	unsigned long got_enbdisack;
	unsigned long got_xmit_done;
	unsigned long xmit_fail;
	unsigned long sent_enbdis;
	unsigned long sent_promisc;
	unsigned long sent_post;
	unsigned long sent_xmit;
	unsigned long reject_count;
	unsigned long extra_rcvbufs_sent;
};

struct datachan {
	struct chaninfo chinfo;
	struct chanstat chstat;
};

struct visornic_devdata {
	int devno;
	int interrupt_vector;
	int thread_wait_ms;
	unsigned short enabled;		/* 0 disabled 1 enabled to receive */
	unsigned short enab_dis_acked;	/* NET_RCV_ENABLE/DISABLE acked by
					 * IOPART
					 */
	struct visor_device *dev;
	struct visorchipset_device_info *dev_chipset; /* IRQ Information */
	/* lock for dev */
	struct rw_semaphore lock_visor_dev;
	char name[99];
	struct list_head list_all;   /* < link within list_all_devices list */
	struct kref kref;
	struct net_device *netdev;
	struct net_device_stats net_stats;
	atomic_t interrupt_rcvd;
	wait_queue_head_t rsp_queue;
	struct sk_buff **rcvbuf;
	unsigned long long uniquenum;
	unsigned short old_flags;	/* flags as they were prior to
					 * set_multicast_list
					 */
	atomic_t usage;			/* count of users */
	int num_rcv_bufs;		/* indicates how many rcv buffers
					 * the vnic will post
					 */
	int num_rcv_bufs_could_not_alloc;
	atomic_t num_rcvbuf_in_iovm;
	unsigned long alloc_failed_in_if_needed_cnt;
	unsigned long alloc_failed_in_repost_rtn_cnt;
	int max_outstanding_net_xmits;   /* absolute max number of outstanding
					  * xmits - should never hit this */
	int upper_threshold_net_xmits;   /* high water mark for calling
					  * netif_stop_queue() */
	int lower_threshold_net_xmits;	 /* high water mark for calling
					    netif_wake_queue() */
	struct sk_buff_head xmitbufhead; /* xmitbufhead is the head of the
					  * xmit buffer list that have been
					  * sent to the IOPART end
					  */
	struct work_struct serverdown_completion;
	struct work_struct timeout_reset;
	struct uiscmdrsp *cmdrsp_rcv;	 /* cmdrsp_rcv is used for
					  * posting/unposting rcv buffers
					  */
	struct uiscmdrsp *xmit_cmdrsp;	 /* used to issue NET_XMIT - there is
					  * never more that one xmit in
					  * progress at a time
					  */
	bool server_down;		 /* IOPART is down */
	bool server_change_state;	 /* Processing SERVER_CHANGESTATE msg */
	struct dentry *eth_debugfs_dir;
	struct visor_thread_info threadinfo;
	unsigned long long interrupts_rcvd;
	unsigned long long interrupts_notme;
	unsigned long long interrupts_disabled;
	unsigned long long busy_cnt;
	spinlock_t insertlock; /* spinlock to put items into our queues */
	spinlock_t priv_lock; /* spinlock to access devdata structures */

	/* flow control counter */
	unsigned long long flow_control_upper_hits;
	unsigned long long flow_control_lower_hits;

	/* debug counters */
	ulong n_rcv0;			/* # rcvs of 0 buffers */
	ulong n_rcv1;			/* # rcvs of 1 buffers */
	ulong n_rcv2;			/* # rcvs of 2 buffers */
	ulong n_rcvx;			/* # rcvs of >2 buffers */
	ulong found_repost_rcvbuf_cnt;	/* #times we called repost_rcvbuf_cnt*/
	ulong repost_found_skb_cnt;	/* # times found the skb */
	ulong n_repost_deficit;		/* # times we couldn't find all of the
					   rcv buffers */
	ulong bad_rcv_buf;		/* # times we negleted to free the
					   rcv skb because we didn't know
					   where it came from */
	ulong n_rcv_packets_not_accepted;/* # bogs rcv packets */

	int queuefullmsg_logged;
	struct chanstat chstat;
};

#define VISORNICSOPENMAX 32
/* array of open devices maintained by open() and close() */
static struct net_device *num_visornic_open[VISORNICSOPENMAX];

/* unsigned int visor_copy_fragsinfo_from_skb(unsigned char *calling_ctx,
 *					      void *skb_in,
 *					      unsigned int firstfraglen,
 *					      unsigned int frags_max,
 *					      struct phys_info frags[])
 *
 *	calling_ctx - input -	a string that is displayed to show
 *				who calld this func
 *	void *skb_in - skb whose frag info we're copying type is hidden so we
 *		       don't need to include skbfuff in uisutils.h which is
 *		       included in non-networking code.
 *	unsigned int firstfraglen - input - length of first fragment in skb
 *	unsigned int frags_max - input - max len of frags array
 *	struct phys_info frags[] - output - frags array filled in on output
 *					    return value indicates number of
 *					    entries filled in frags
 */
unsigned int
visor_copy_fragsinfo_from_skb(unsigned char *calling_ctx, struct sk_buff *skb,
			      unsigned int firstfraglen, unsigned int frags_max,
			      struct phys_info frags[])
{
	unsigned int count = 0, ii, size, offset = 0, numfrags;

	numfrags = skb_shinfo(skb)->nr_frags;

	while (firstfraglen) {
		if (count == frags_max)
			return -1;

		frags[count].pi_pfn =
			page_to_pfn(virt_to_page(skb->data + offset));
		frags[count].pi_off =
			(unsigned long)(skb->data + offset) & PI_PAGE_MASK;
		size = min(firstfraglen,
			   (unsigned int)(PI_PAGE_SIZE - frags[count].pi_off));

		/* can take smallest of firstfraglen (what's left) OR
		 * bytes left in the page
		 */
		frags[count].pi_len = size;
		firstfraglen -= size;
		offset += size;
		count++;
	}
	if (!numfrags)
		goto dolist;

	if ((count + numfrags) > frags_max)
		return -1;

	for (ii = 0; ii < numfrags; ii++) {
		count = add_physinfo_entries(page_to_pfn(
				skb_frag_page(&skb_shinfo(skb)->frags[ii])),
					      skb_shinfo(skb)->frags[ii].
					      page_offset,
					      skb_shinfo(skb)->frags[ii].
					      size, count, frags_max, frags);
		if (!count)
			return -1;
	}

dolist: if (skb_shinfo(skb)->frag_list) {
		struct sk_buff *skbinlist;
		int c;

		for (skbinlist = skb_shinfo(skb)->frag_list; skbinlist;
		     skbinlist = skbinlist->next) {
			c = visor_copy_fragsinfo_from_skb("recursive",
							  skbinlist,
							  skbinlist->len -
							  skbinlist->data_len,
							  frags_max - count,
							  &frags[count]);
			if (c == -1)
				return -1;
			count += c;
		}
	}
	return count;
}

int visor_thread_start(struct visor_thread_info *thrinfo,
		       int (*threadfn)(void *),
		       void *thrcontext, char *name)
{
	/* used to stop the thread */
	init_completion(&thrinfo->has_stopped);
	thrinfo->task = kthread_run(threadfn, thrcontext, name);
	if (IS_ERR(thrinfo->task)) {
		thrinfo->id = 0;
		return 0;
	}
	thrinfo->id = thrinfo->task->pid;
	return 0;
}

void visor_thread_stop(struct visor_thread_info *thrinfo)
{
	if (!thrinfo->id)
		return;	/* thread not running */

	kthread_stop(thrinfo->task);
	/* give up if the thread has NOT died in 1 minute */
	if (wait_for_completion_timeout(&thrinfo->has_stopped, 60 * HZ))
		thrinfo->id = 0;
}

/** DebugFS code
 */
static ssize_t info_debugfs_read(struct file *file, char __user *buf,
				 size_t len, loff_t *offset)
{
	int i;
	ssize_t bytes_read = 0;
	int str_pos = 0;
	struct visornic_devdata *devdata;
	char *vbuf;

	if (len > MAX_BUF)
		len = MAX_BUF;
	vbuf = kzalloc(len, GFP_KERNEL);
	if (!vbuf)
		return -ENOMEM;

	/* for each vnic channel
	 * dump out channel specific data
	 */
	for (i = 0; i < VISORNICSOPENMAX; i++) {
		if (!num_visornic_open[i])
			continue;

		devdata = netdev_priv(num_visornic_open[i]);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     "Vnic i = %d\n", i);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     "netdev = %s (0x%p), MAC Addr %pM\n",
				     num_visornic_open[i]->name,
				     num_visornic_open[i],
				     num_visornic_open[i]->dev_addr);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     "VisorNic Dev Info = 0x%p\n", devdata);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " num_rcv_bufs = %d\n",
				     devdata->num_rcv_bufs);
		/* str_pos += scnprintf(vbuf + str_pos, len - str_pos,
		 *			" features = 0x%015llX\n",
		 *     visorchannel_read(devdata->dev->visorchannel
		 */
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " max_oustanding_next_xmits = %d\n",
				    devdata->max_outstanding_net_xmits);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " upper_threshold_net_xmits = %d\n",
				     devdata->upper_threshold_net_xmits);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " lower_threshold_net_xmits = %d\n",
				     devdata->lower_threshold_net_xmits);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " queuefullmsg_logged = %d\n",
				     devdata->queuefullmsg_logged);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " chstat.got_rcv = %lu\n",
				     devdata->chstat.got_rcv);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " chstat.got_enbdisack = %lu\n",
				     devdata->chstat.got_enbdisack);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " chstat.got_xmit_done = %lu\n",
				     devdata->chstat.got_xmit_done);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " chstat.xmit_fail = %lu\n",
				     devdata->chstat.xmit_fail);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " chstat.sent_enbdis = %lu\n",
				     devdata->chstat.sent_enbdis);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " chstat.sent_promisc = %lu\n",
				     devdata->chstat.sent_promisc);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " chstat.sent_post = %lu\n",
				     devdata->chstat.sent_post);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " chstat.sent_xmit = %lu\n",
				     devdata->chstat.sent_xmit);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " chstat.reject_count = %lu\n",
				     devdata->chstat.reject_count);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " chstat.extra_rcvbufs_sent = %lu\n",
				     devdata->chstat.extra_rcvbufs_sent);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " n_rcv0 = %lu\n", devdata->n_rcv0);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " n_rcv1 = %lu\n", devdata->n_rcv1);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " n_rcv2 = %lu\n", devdata->n_rcv2);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " n_rcvx = %lu\n", devdata->n_rcvx);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " num_rcvbuf_in_iovm = %d\n",
				     atomic_read(&devdata->num_rcvbuf_in_iovm));
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " alloc_failed_in_if_needed_cnt = %lu\n",
				     devdata->alloc_failed_in_if_needed_cnt);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " alloc_failed_in_repost_rtn_cnt = %lu\n",
				     devdata->alloc_failed_in_repost_rtn_cnt);
		/* str_pos += scnprintf(vbuf + str_pos, len - str_pos,
		 *		     " inner_loop_limit_reached_cnt = %lu\n",
		 *		     devdata->inner_loop_limit_reached_cnt);
		 */
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " found_repost_rcvbuf_cnt = %lu\n",
				     devdata->found_repost_rcvbuf_cnt);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " repost_found_skb_cnt = %lu\n",
				     devdata->repost_found_skb_cnt);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " n_repost_deficit = %lu\n",
				     devdata->n_repost_deficit);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " bad_rcv_buf = %lu\n",
				     devdata->bad_rcv_buf);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " n_rcv_packets_not_accepted = %lu\n",
				     devdata->n_rcv_packets_not_accepted);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " interrupts_rcvd = %llu\n",
				     devdata->interrupts_rcvd);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " interrupts_notme = %llu\n",
				     devdata->interrupts_notme);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " interrupts_disabled = %llu\n",
				     devdata->interrupts_disabled);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " busy_cnt = %llu\n",
				     devdata->busy_cnt);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " flow_control_upper_hits = %llu\n",
				     devdata->flow_control_upper_hits);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " flow_control_lower_hits = %llu\n",
				     devdata->flow_control_lower_hits);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " thread_wait_ms = %d\n",
				     devdata->thread_wait_ms);
		str_pos += scnprintf(vbuf + str_pos, len - str_pos,
				     " netif_queue = %s\n",
				     netif_queue_stopped(devdata->netdev) ?
				     "stopped" : "running");
	}
	bytes_read = simple_read_from_buffer(buf, len, offset, vbuf, str_pos);
	kfree(vbuf);
	return bytes_read;
}

static ssize_t enable_ints_write(struct file *file,
				 const char __user *buffer,
				 size_t count, loff_t *ppos)
{
	char buf[4];
	int i, new_value;
	struct visornic_devdata *devdata;

	if (count >= ARRAY_SIZE(buf))
		return -EINVAL;

	buf[count] = '\0';
	if (copy_from_user(buf, buffer, count)) {
		LOGERR("copy_from_user_failed.\n");
		return -EFAULT;
	}

	i = kstrtoint(buf, 10, &new_value);

	if (i != 0) {
		LOGERR("Failed to scan value for enable_ints, buf<<%.*s>>",
		       (int)count, buf);
		return -EFAULT;
	}

	/* set all counts to new_value usually 0 */
	for (i = 0; i < VISORNICSOPENMAX; i++) {
		if (num_visornic_open[i]) {
			devdata = num_visornic_open[i];
			/* TODO update features bit in channel */
		}
	}

	return count;
}

static void
visornic_serverdown_complete(struct work_struct *work)
{
	struct visornic_devdata *devdata;
	struct net_device *netdev;
	unsigned long flags;
	int i = 0, count = 0;

	devdata = container_of(work, struct visornic_devdata,
			       serverdown_completion);
	netdev = devdata->netdev;

	/* Stop using datachan */
	visor_thread_stop(&devdata->threadinfo);

	/* Inform Linux that the link is down */
	netif_carrier_off(netdev);
	netif_stop_queue(netdev);

	/* Free the skb for XMITs that haven't been serviced by the server
	 * We shouldn't have to inform Linux about these IOs because they
	 * are "lost in the ethernet"
	 */
	skb_queue_purge(&devdata->xmitbufhead);

	spin_lock_irqsave(&devdata->priv_lock, flags);
	/* free rcv buffers */
	for (i = 0; i < devdata->num_rcv_bufs; i++) {
		if (devdata->rcvbuf[i]) {
			kfree_skb(devdata->rcvbuf[i]);
			devdata->rcvbuf[i] = NULL;
			count++;
		}
	}
	atomic_set(&devdata->num_rcvbuf_in_iovm, 0);
	spin_unlock_irqrestore(&devdata->priv_lock, flags);

	devdata->server_down = true;
	devdata->server_change_state = false;
	visorchipset_device_pause_response(devdata->dev->chipset_bus_no,
					   devdata->dev->chipset_dev_no, 0);
}

static int
visornic_serverdown(struct visornic_devdata *devdata, u32 state)
{
	struct net_device *netdev = devdata->netdev;

	if (!devdata->server_down && !devdata->server_change_state) {
		devdata->server_change_state = true;
		queue_work(visornic_serverdown_workqueue,
			   &devdata->serverdown_completion);
	} else if (devdata->server_change_state) {
		return -1;
	}
	return 0;
}

/** List of all visornic_devdata structs,
  * linked via the list_all member
  */
static LIST_HEAD(list_all_devices);
static DEFINE_SPINLOCK(lock_all_devices);

static struct sk_buff *
alloc_rcv_buf(struct net_device *netdev)
{
	struct sk_buff *skb;
/*
 * NOTE: the first fragment in each rcv buffer is pointed to by rcvskb->data.
 * For now all rcv buffers will be RCVPOST_BUF_SIZE in length, so the firstfrag
 * is large enough to hold 1514.
 */
	DBGINF("netdev->name <<%s>>:  allocating skb len:%d\n", netdev->name,
	       RCVPOST_BUF_SIZE);
	skb = alloc_skb(RCVPOST_BUF_SIZE, GFP_ATOMIC | __GFP_NOWARN);
	if (!skb) {
		LOGVER("**** alloc_skb failed\n");
		return NULL;
	}
	skb->dev = netdev;
	skb->len = RCVPOST_BUF_SIZE;
	/* current value of mtu doesn't come into play here; large
	 * packets will just end up using multiple rcv buffers all of
	 * same size
	 */
	skb->data_len = 0;      /* dev_alloc_skb already zeroes it out.
				   for clarification. */
	return skb;
}

static inline void
post_skb(struct uiscmdrsp *cmdrsp,
	 struct visornic_devdata *devdata, struct sk_buff *skb)
{
	cmdrsp->net.buf = skb;
	cmdrsp->net.rcvpost.frag.pi_pfn = page_to_pfn(virt_to_page(skb->data));
	cmdrsp->net.rcvpost.frag.pi_off =
		(unsigned long)skb->data & PI_PAGE_MASK;
	cmdrsp->net.rcvpost.frag.pi_len = skb->len;
	cmdrsp->net.rcvpost.unique_num = devdata->uniquenum;

	if ((cmdrsp->net.rcvpost.frag.pi_off + skb->len) > PI_PAGE_SIZE) {
		LOGERRNAME(devdata->netdev,
			   "**** pi_off:0x%x pi_len:%d SPAN ACROSS A PAGE\n",
			   cmdrsp->net.rcvpost.frag.pi_off, skb->len);
	} else {
		cmdrsp->net.type = NET_RCV_POST;
		cmdrsp->cmdtype = CMD_NET_TYPE;
		visorchannel_signalinsert(devdata->dev->visorchannel,
					  IOCHAN_TO_IOPART,
					  cmdrsp);
		atomic_inc(&devdata->num_rcvbuf_in_iovm);
		devdata->chstat.sent_post++;
	}
}

static void
send_enbdis(struct net_device *netdev, int state,
	    struct visornic_devdata *devdata)
{
	devdata->cmdrsp_rcv->net.enbdis.enable = state;
	devdata->cmdrsp_rcv->net.enbdis.context = netdev;
	devdata->cmdrsp_rcv->net.type = NET_RCV_ENBDIS;
	devdata->cmdrsp_rcv->cmdtype = CMD_NET_TYPE;
	visorchannel_signalinsert(devdata->dev->visorchannel,
				  IOCHAN_TO_IOPART,
				  devdata->cmdrsp_rcv);
	devdata->chstat.sent_enbdis++;
}

static int
visornic_disable_with_timeout(struct net_device *netdev, const int timeout)
{
	struct visornic_devdata *devdata = netdev_priv(netdev);
	int i;
	unsigned long flags;
	int wait = 0;

	/* stop the transmit queue so nothing more can be transmitted */
	netif_stop_queue(netdev);

	/* send a msg telling the other end we are stopping incoming pkts */
	spin_lock_irqsave(&devdata->priv_lock, flags);
	devdata->enabled = 0;
	devdata->enab_dis_acked = 0; /* must wait for ack */
	spin_unlock_irqrestore(&devdata->priv_lock, flags);

	/* send disable and wait for ack -- don't hold lock when sending
	 * disable because if the queue is full, insert might sleep.
	 */
	send_enbdis(netdev, 0, devdata);

	/* wait for ack to arrive before we try to free rcv buffers
	 * NOTE: the other end automatically unposts the rcv buffers when
	 * when it gets a disable.
	 */
	spin_lock_irqsave(&devdata->priv_lock, flags);
	while ((timeout == VISORNIC_INFINITE_RESPONSE_WAIT) ||
	       (wait < timeout)) {
		if (devdata->n_rcv_packets_not_accepted)
			break;
		if (devdata->server_down || devdata->server_change_state) {
			spin_unlock_irqrestore(&devdata->priv_lock, flags);
			return -1;
		}
		set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&devdata->priv_lock, flags);
		wait += schedule_timeout(msecs_to_jiffies(10));
		spin_lock_irqsave(&devdata->priv_lock, flags);
	}
	/*
	 * Wait for usage to go to 1 (no other users) before freeing
	 * rcv buffers
	 */
	if (atomic_read(&devdata->usage) > 1) {
		while (1) {
			set_current_state(TASK_INTERRUPTIBLE);
			spin_unlock_irqrestore(&devdata->priv_lock, flags);
			schedule_timeout(msecs_to_jiffies(10));
			spin_lock_irqsave(&devdata->priv_lock, flags);
			if (atomic_read(&devdata->usage))
				break;
		}
	}
	/* we've set enabled to 0, so we can give up the lock. */
	spin_unlock_irqrestore(&devdata->priv_lock, flags);

	/*
	 * Free rcv buffers - other end has automatically unposed them on
	 * disable
	 */
	for (i = 0; i < devdata->num_rcv_bufs; i++) {
		if (devdata->rcvbuf[i]) {
			kfree_skb(devdata->rcvbuf[i]);
			devdata->rcvbuf[i] = NULL;
		}
	}

	/* remove references from array */
	for (i = 0; i < VISORNICSOPENMAX; i++)
		if (num_visornic_open[i] == netdev) {
			num_visornic_open[i] = NULL;
			break;
		}

	return 0;
}

static int
init_rcv_bufs(struct net_device *netdev, struct visornic_devdata *devdata)
{
	int i, count;

	/* allocate fixed number of receive buffers to post to uisnic
	 * post receive buffers after we've allocated a required amount
	 */
	for (i = 0; i < devdata->num_rcv_bufs; i++) {
		devdata->rcvbuf[i] = alloc_rcv_buf(netdev);
		if (!devdata->rcvbuf[i])
			break; /* if we failed to allocate one let us stop */
	}
	if (i == 0) /* couldn't even allocate one -- bail out */
		return -ENOMEM;
	count = i;

	/* Ensure we can alloc 2/3rd of the requeested number of buffers.
	 * 2/3 is an arbitrary choice; used also in ndis init.c
	 */
	if (count < ((2 * devdata->num_rcv_bufs) / 3)) {
		/* free receive buffers we did alloc and then bail out */
		for (i = 0; i < count; i++) {
			kfree_skb(devdata->rcvbuf[i]);
			devdata->rcvbuf[i] = NULL;
		}
		return -ENOMEM;
	}

	/* post receive buffers to receive incoming input - without holding
	 * lock - we've not enabled nor started the queue so there shouldn't
	 * be any rcv or xmit activity
	 */
	for (i = 0; i < count; i++)
		post_skb(devdata->cmdrsp_rcv, devdata, devdata->rcvbuf[i]);

	return 0;
}

/* Sends enable to IOVM, inits, and posts receive buffers to IOVM
 *
 * timeout is defined in msecs (timeout of 0 specifies infinite wait)
 */
static int
visornic_enable_with_timeout(struct net_device *netdev, const int timeout)
{
	int i;
	struct visornic_devdata *devdata = netdev_priv(netdev);
	unsigned long flags;
	int wait = 0;

	/* NOTE: the other end automatically unposts the rcv buffers when it
	 * gets a disable.
	 */
	i = init_rcv_bufs(netdev, devdata);
	if (i < 0)
		return i;

	spin_lock_irqsave(&devdata->priv_lock, flags);
	devdata->enabled = 1;

	/* now we're ready, let's send an ENB to uisnic but until we get
	 * an ACK back from uisnic, we'll drop the packets
	 */
	devdata->n_rcv_packets_not_accepted = 0;
	spin_unlock_irqrestore(&devdata->priv_lock, flags);

	/* send enable and wait for ack -- don't hold lock when sending enable
	 * because if the queue is full, insert might sleep.
	 */
	send_enbdis(netdev, 1, devdata);

	spin_lock_irqsave(&devdata->priv_lock, flags);
	while ((timeout == VISORNIC_INFINITE_RESPONSE_WAIT) ||
	       (wait < timeout)) {
		if (devdata->enab_dis_acked)
			break;
		if (devdata->server_down || devdata->server_change_state) {
			spin_unlock_irqrestore(&devdata->priv_lock, flags);
			return -1;
		}
		set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&devdata->priv_lock, flags);
		wait += schedule_timeout(msecs_to_jiffies(10));
		spin_lock_irqsave(&devdata->priv_lock, flags);
	}

	spin_unlock_irqrestore(&devdata->priv_lock, flags);

	if (!devdata->enab_dis_acked)
		return -1;

	/* find an open slot in the array to save off VisorNic references
	 * for debug
	 */
	for (i = 0; i < VISORNICSOPENMAX; i++) {
		if (!num_visornic_open[i]) {
			num_visornic_open[i] = netdev;
			break;
		}
	}

	return 0;
}

static void
visornic_timeout_reset(struct work_struct *work)
{
	struct visornic_devdata *devdata;
	struct net_device *netdev;
	int response = 0;

	devdata = container_of(work, struct visornic_devdata, timeout_reset);
	netdev = devdata->netdev;

	/* Transmit Timeouts are typically handled by resetting the
	 * device for our virtual NIC we will send a Disable and Enable
	 * to the IOVM. If it doesn't respond we will trigger a serverdown.
	 */
	netif_stop_queue(netdev);
	response = visornic_disable_with_timeout(netdev, 100);
	if (response)
		goto call_serverdown;

	response = visornic_enable_with_timeout(netdev, 100);
	if (response)
		goto call_serverdown;
	netif_wake_queue(netdev);

	return;

call_serverdown:
	visornic_serverdown(devdata, 0);
}

static int
visornic_open(struct net_device *netdev)
{
	visornic_enable_with_timeout(netdev, VISORNIC_INFINITE_RESPONSE_WAIT);

	/* start the interface's transmit queue, allowing it to accept
	 * packets for transmission
	 */
	netif_start_queue(netdev);

	return 0;
}

static int
visornic_close(struct net_device *netdev)
{
	netif_stop_queue(netdev);
	visornic_disable_with_timeout(netdev, VISORNIC_INFINITE_RESPONSE_WAIT);

	return 0;
}

/* This function is protected from concurrent calls by a spinlock xmit_lock
 * in the net_device struct, but as soon as the function returns it can be
 * called again.
 */
static int
visornic_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct visornic_devdata *devdata;
	int len, firstfraglen, padlen;
	struct uiscmdrsp *cmdrsp = NULL;
	unsigned long flags;

	devdata = netdev_priv(netdev);
	spin_lock_irqsave(&devdata->priv_lock, flags);

	if (netif_queue_stopped(netdev) || devdata->server_down ||
	    devdata->server_change_state) {
		spin_unlock_irqrestore(&devdata->priv_lock, flags);
		devdata->busy_cnt++;
		return NETDEV_TX_BUSY;
	}

	/* sk_buff struct is used to host network data throughout all the
	 * linux network subsystems
	 */
	len = skb->len;
	/*
	 * skb->len is the FULL length of data (including fragmentary portion)
	 * skb->data_len is the length of the fragment portion in frags
	 * skb->len - skb->data_len is size of the 1st fragment in skb->data
	 * calculate the length of the first fragment that skb->data is
	 * pointing to
	 */
	firstfraglen = skb->len - skb->data_len;
	if (firstfraglen < ETH_HEADER_SIZE) {
		spin_unlock_irqrestore(&devdata->priv_lock, flags);
		devdata->busy_cnt++;
		return NETDEV_TX_BUSY;
	}

	if ((len < ETH_MIN_PACKET_SIZE) &&
	    ((skb_end_pointer(skb) - skb->data) >= ETH_MIN_PACKET_SIZE)) {
		/* pad the packet out to minimum size */
		padlen = ETH_MIN_PACKET_SIZE - len;
		memset(&skb->data[len], 0, padlen);
		skb->tail += padlen;
		skb->len += padlen;
		len += padlen;
		firstfraglen += padlen;
	}

	cmdrsp = devdata->xmit_cmdrsp;
	/* clear cmdrsp */
	memset(cmdrsp, 0, SIZEOF_CMDRSP);
	cmdrsp->net.type = NET_XMIT;
	cmdrsp->cmdtype = CMD_NET_TYPE;

	/* save the pointer to skb -- we'll need it for completion */
	cmdrsp->net.buf = skb;

	if (((devdata->chstat.sent_xmit >= devdata->chstat.got_xmit_done) &&
	     (devdata->chstat.sent_xmit - devdata->chstat.got_xmit_done >=
	     devdata->max_outstanding_net_xmits)) ||
	     ((devdata->chstat.sent_xmit < devdata->chstat.got_xmit_done) &&
	     (ULONG_MAX - devdata->chstat.got_xmit_done +
	      devdata->chstat.sent_xmit >=
	      devdata->max_outstanding_net_xmits))) {
		/*
		 * too many NET_XMITs queued over to IOVM - need to wait
		 */
		devdata->chstat.reject_count++;
		if (!devdata->queuefullmsg_logged &&
		    ((devdata->chstat.reject_count & 0x3ff) == 1)) {
			devdata->queuefullmsg_logged = 1;
			LOGINFNAME(devdata->netdev, "**** REJECTING NEX_XMIT - rejected count=%ld chstat.sent_xmit=%lu chstat.got_xmit_done=%lu\n",
				   devdata->chstat.reject_count,
				   devdata->chstat.sent_xmit,
				   devdata->chstat.got_xmit_done);
		}
		netif_stop_queue(netdev);
		spin_unlock_irqrestore(&devdata->priv_lock, flags);
		devdata->busy_cnt++;
		return NETDEV_TX_BUSY;
	}
	if (devdata->queuefullmsg_logged)
		devdata->queuefullmsg_logged = 0;

	if (skb->ip_summed == CHECKSUM_UNNECESSARY) {
		cmdrsp->net.xmt.lincsum.valid = 1;
		cmdrsp->net.xmt.lincsum.protocol = skb->protocol;
		if (skb_transport_header(skb) > skb->data) {
			cmdrsp->net.xmt.lincsum.hrawoff =
				skb_transport_header(skb) - skb->data;
			cmdrsp->net.xmt.lincsum.hrawoff = 1;
		}
		if (skb_network_header(skb) > skb->data) {
			cmdrsp->net.xmt.lincsum.nhrawoff =
				skb_network_header(skb) - skb->data;
			cmdrsp->net.xmt.lincsum.nhrawoffv = 1;
		}
		cmdrsp->net.xmt.lincsum.csum = skb->csum;
	} else {
		cmdrsp->net.xmt.lincsum.valid = 0;
	}

	/* save off the length of the entire data packet */
	cmdrsp->net.xmt.len = len;

	/*
	 * copy ethernet header from first frag into ocmdrsp
	 * - everything else will be pass in frags & DMA'ed
	 */
	memcpy(cmdrsp->net.xmt.ethhdr, skb->data, ETH_HEADER_SIZE);
	/*
	 * copy frags info - from skb->data we need to only provide access
	 * beyond eth header
	 */
	cmdrsp->net.xmt.num_frags =
		visor_copy_fragsinfo_from_skb("visornic_xmit", skb,
					      firstfraglen, MAX_PHYS_INFO,
					      cmdrsp->net.xmt.frags);
	if (cmdrsp->net.xmt.num_frags == -1) {
		spin_unlock_irqrestore(&devdata->priv_lock, flags);
		devdata->busy_cnt++;
		return NETDEV_TX_BUSY;
	}

	if (!visorchannel_signalinsert(devdata->dev->visorchannel,
				       IOCHAN_TO_IOPART, cmdrsp)) {
		netif_stop_queue(netdev);
		spin_unlock_irqrestore(&devdata->priv_lock, flags);
		devdata->busy_cnt++;
		return NETDEV_TX_BUSY;
	}

	/* Track the skbs that have been sent to the IOVM for XMIT */
	skb_queue_head(&devdata->xmitbufhead, skb);

	/*
	 * set the last transmission start time
	 * linux doc says: Do not forget to update netdev->trans_start to
	 * jiffies after each new tx packet is given to the hardware.
	 */
	netdev->trans_start = jiffies;

	/* update xmt stats */
	devdata->net_stats.tx_packets++;
	devdata->net_stats.tx_bytes += skb->len;
	devdata->chstat.sent_xmit++;

	/*
	 * check to see if we have hit the high watermark for
	 * netif_stop_queue()
	 */
	if (((devdata->chstat.sent_xmit >= devdata->chstat.got_xmit_done) &&
	     (devdata->chstat.sent_xmit - devdata->chstat.got_xmit_done >=
	      devdata->upper_threshold_net_xmits)) ||
	    ((devdata->chstat.sent_xmit < devdata->chstat.got_xmit_done) &&
	     (ULONG_MAX - devdata->chstat.got_xmit_done +
	      devdata->chstat.sent_xmit >=
	      devdata->upper_threshold_net_xmits))) {
		/* too many NET_XMITs queued over to IOVM - need to wait */
		netif_stop_queue(netdev); /* calling stop queue - call
					   * netif_wake_queue() after lower
					   * threshold
					   */
		devdata->flow_control_upper_hits++;
	}
	spin_unlock_irqrestore(&devdata->priv_lock, flags);

	/* skb will be freed when we get back NET_XMIT_DONE */
	return NETDEV_TX_OK;
}

static struct net_device_stats *
visornic_get_stats(struct net_device *netdev)
{
	struct visornic_devdata *devdata = netdev_priv(netdev);

	return &devdata->net_stats;
}

static int
visornic_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	return -EOPNOTSUPP;
}

static int
visornic_change_mtu(struct net_device *netdev, int new_mtu)
{
	/* we cannot willy-nilly change the MTU, it has to come from
	 * CONTROLVM and all the vnics and pnics in a switch have to have
	 * the same MTU for everything to work.
	 */
	return -EINVAL;
}

static void
visornic_set_multi(struct net_device *netdev)
{
	struct uiscmdrsp *cmdrsp;
	struct visornic_devdata *devdata = netdev_priv(netdev);

	/* any filtering changes */
	if (devdata->old_flags != netdev->flags) {
		if ((netdev->flags & IFF_PROMISC) !=
		    (devdata->old_flags & IFF_PROMISC)) {
			cmdrsp = kmalloc(SIZEOF_CMDRSP, GFP_ATOMIC);
			if (!cmdrsp)
				return;
			cmdrsp->cmdtype = CMD_NET_TYPE;
			cmdrsp->net.type = NET_RCV_PROMISC;
			cmdrsp->net.enbdis.context = netdev;
			cmdrsp->net.enbdis.enable =
				(netdev->flags & IFF_PROMISC);
			visorchannel_signalinsert(devdata->dev->visorchannel,
						  IOCHAN_TO_IOPART,
						  cmdrsp);
			kfree(cmdrsp);
		}
		devdata->old_flags = netdev->flags;
	}
}

static void
visornic_xmit_timeout(struct net_device *netdev)
{
	struct visornic_devdata *devdata = netdev_priv(netdev);
	unsigned long flags;

	spin_lock_irqsave(&devdata->priv_lock, flags);
	/* Ensure that a ServerDown message hasn't been received */
	if (!devdata->enabled ||
	    (devdata->server_down && !devdata->server_change_state)) {
		spin_unlock_irqrestore(&devdata->priv_lock, flags);
		return;
	}
	spin_unlock_irqrestore(&devdata->priv_lock, flags);

	queue_work(visornic_timeout_reset_workqueue, &devdata->timeout_reset);
}

static inline int
repost_return(
	struct uiscmdrsp *cmdrsp,
	struct visornic_devdata *devdata,
	struct sk_buff *skb,
	struct net_device *netdev)
{
	struct net_pkt_rcv copy;
	int i = 0, cc, numreposted;
	int found_skb = 0;
	int status = 0;

	copy = cmdrsp->net.rcv;
	LOGVER("REPOST_RETURN: realloc rcv skbs to replace:%d rcvbufs\n",
	       copy.numrcvbufs);
	switch (copy.numrcvbufs) {
	case 0:
		devdata->n_rcv0++;
		break;
	case 1:
		devdata->n_rcv1++;
		break;
	case 2:
		devdata->n_rcv2++;
		break;
	default:
		devdata->n_rcvx++;
		break;
	}
	for (cc = 0, numreposted = 0; cc < copy.numrcvbufs; cc++) {
		for (i = 0; i < devdata->num_rcv_bufs; i++) {
			if (devdata->rcvbuf[i] != copy.rcvbuf[cc])
				continue;

			LOGVER("REPOST_RETURN: orphaning old rcvbuf[%d]:%p cc=%d",
			       i, devdata->rcvbuf[i], cc);
			if ((skb) && devdata->rcvbuf[i] == skb) {
				devdata->found_repost_rcvbuf_cnt++;
				found_skb = 1;
				devdata->repost_found_skb_cnt++;
			}
			devdata->rcvbuf[i] = alloc_rcv_buf(netdev);
			if (!devdata->rcvbuf[i]) {
				LOGVER("**** %s FAILED to reallocate new rcv buf - no REPOST, found_skb=%d, cc=%d, i=%d\n",
				       netdev->name, found_skb, cc, i);
				devdata->num_rcv_bufs_could_not_alloc++;
				devdata->alloc_failed_in_repost_rtn_cnt++;
				status = -1;
				break;
			}
			LOGVER("REPOST_RETURN: reposting new rcvbuf[%d]:%p\n",
			       i, devdata->rcvbuf[i]);
			post_skb(cmdrsp, devdata, devdata->rcvbuf[i]);
			numreposted++;
			break;
		}
	}
	LOGVER("REPOST_RETURN: num rcvbufs posted:%d\n", numreposted);
	if (numreposted != copy.numrcvbufs) {
		LOGVER("**** %s FAILED to repost all the rcv bufs; numreposted:%d rcv.numrcvbufs:%d\n",
		       netdev->name, numreposted, copy.numrcvbufs);
		devdata->n_repost_deficit++;
		status = -1;
	}
	if (skb) {
		if (found_skb) {
			LOGVER("REPOST_RETURN: skb is %p - freeing it", skb);
			kfree_skb(skb);
		} else {
			LOGERRNAME(devdata->netdev, "%s REPOST_RETURN: skb %p NOT found in rcvbuf list!!",
				   netdev->name, skb);
			status = -3;
			devdata->bad_rcv_buf++;
		}
	}
	atomic_dec(&devdata->usage);
	return status;
}

static void
visornic_rx(struct uiscmdrsp *cmdrsp)
{
	struct visornic_devdata *devdata;
	struct sk_buff *skb, *prev, *curr;
	struct net_device *netdev;
	int cc, currsize, off, status;
	struct ethhdr *eth;
	unsigned long flags;
#ifdef DEBUG
	struct phys_info testfrags[MAX_PHYS_INFO];
#endif

/*
 * post new rcv buf to the other end using the cmdrsp we have at hand
 * post it without holding lock - but we'll use the signal lock to synchronize
 * the queue insert the cmdrsp that contains the net.rcv is the one we are
 * using to repost, so copy the info we need from it.
 */
	skb = cmdrsp->net.buf;
	netdev = skb->dev;

	if (!netdev) {
		/* We must have previously downed this network device and
		 * this skb and device is no longer valid. This also means
		 * the skb reference was removed from devdata->rcvbuf so no
		 * need to search for it.
		 * All we can do is free the skb and return.
		 * Note: We crash if we try to log this here.
		 */
		kfree_skb(skb);
		return;
	}

	devdata = netdev_priv(netdev);

	spin_lock_irqsave(&devdata->priv_lock, flags);
	atomic_dec(&devdata->num_rcvbuf_in_iovm);

	/* update rcv stats - call it with priv_lock held */
	devdata->net_stats.rx_packets++;
	devdata->net_stats.rx_bytes == skb->len;

	atomic_inc(&devdata->usage);	/* don't want a close to happen before
					   we're done here */
	/*
	 * set length to how much was ACTUALLY received -
	 * NOTE: rcv_done_len includes actual length of data rcvd
	 * including ethhdr
	 */
	skb->len = cmdrsp->net.rcv.rcv_done_len;

	/* test enabled while holding lock */
	if (!(devdata->enabled && devdata->enab_dis_acked)) {
		/*
		 * don't process it unless we're in enable mode and until
		 * we've gotten an ACK saying the other end got our RCV enable
		 */
		LOGERRNAME(devdata->netdev,
			   "%s dropping packet - perhaps old\n", netdev->name);
		spin_unlock_irqrestore(&devdata->priv_lock, flags);
		if (repost_return(cmdrsp, devdata, skb, netdev) < 0)
			LOGERRNAME(devdata->netdev, "repost_return failed");
		return;
	}

	spin_unlock_irqrestore(&devdata->priv_lock, flags);

	/*
	 * when skb was allocated, skb->dev, skb->data, skb->len and
	 * skb->data_len were setup. AND, data has already put into the
	 * skb (both first frag and in frags pages)
	 * NOTE: firstfragslen is the amount of data in skb->data and that
	 * which is not in nr_frags or frag_list. This is now simply
	 * RCVPOST_BUF_SIZE. bump tail to show how much data is in
	 * firstfrag & set data_len to show rest see if we have to chain
	 * frag_list.
	 */
	if (skb->len > RCVPOST_BUF_SIZE) {	/* do PRECAUTIONARY check */
		if (cmdrsp->net.rcv.numrcvbufs < 2) {
			LOGERRNAME(devdata->netdev, "**** %s Something is wrong; rcv_done_len:%d > RCVPOST_BUF_SIZE:%d but numrcvbufs:%d < 2\n",
				   netdev->name, skb->len, RCVPOST_BUF_SIZE,
				   cmdrsp->net.rcv.numrcvbufs);
			if (repost_return(cmdrsp, devdata, skb, netdev) < 0)
				LOGERRNAME(devdata->netdev,
					   "repost_return failed");
			return;
		}
		/* length rcvd is greater than firstfrag in this skb rcv buf  */
		skb->tail += RCVPOST_BUF_SIZE;	/* amount in skb->data */
		skb->data_len = skb->len - RCVPOST_BUF_SIZE;	/* amount that
								   will be in
								   frag_list */
		DBGINF("len:%d data:%d\n", skb->len, skb->data_len);
	} else {
		/*
		 * data fits in this skb - no chaining - do PRECAUTIONARY check
		 */
		if (cmdrsp->net.rcv.numrcvbufs != 1) {	/* should be 1 */
			LOGERRNAME(devdata->netdev, "**** %s Something is wrong; rcv_done_len:%d <= RCVPOST_BUF_SIZE:%d but numrcvbufs:%d != 1\n",
				   netdev->name, skb->len, RCVPOST_BUF_SIZE,
				   cmdrsp->net.rcv.numrcvbufs);
			if (repost_return(cmdrsp, devdata, skb, netdev) < 0)
				LOGERRNAME(devdata->netdev,
					   "repost_return failed");
			return;
		}
		skb->tail += skb->len;
		skb->data_len = 0;	/* nothing rcvd in frag_list */
	}
	off = skb_tail_pointer(skb) - skb->data;
	/*
	 * amount we bumped tail by in the head skb
	 * it is used to calculate the size of each chained skb below
	 * it is also used to index into bufline to continue the copy
	 * (for chansocktwopc)
	 * if necessary chain the rcv skbs together.
	 * NOTE: index 0 has the same as cmdrsp->net.rcv.skb; we need to
	 * chain the rest to that one.
	 * - do PRECAUTIONARY check
	 */
	if (cmdrsp->net.rcv.rcvbuf[0] != skb) {
		LOGERRNAME(devdata->netdev, "**** %s Something is wrong; rcvbuf[0]:%p != skb:%p\n",
			   netdev->name, cmdrsp->net.rcv.rcvbuf[0], skb);
		if (repost_return(cmdrsp, devdata, skb, netdev) < 0)
			LOGERRNAME(devdata->netdev, "repost_return failed");
		return;
	}

	if (cmdrsp->net.rcv.numrcvbufs > 1) {
		/* chain the various rcv buffers into the skb's frag_list. */
		/* Note: off was initialized above  */
		for (cc = 1, prev = NULL;
		     cc < cmdrsp->net.rcv.numrcvbufs; cc++) {
			curr = (struct sk_buff *)cmdrsp->net.rcv.rcvbuf[cc];
			curr->next = NULL;
			DBGINF("chaining skb:%p data:%p to skb:%p data:%p\n",
			       curr, curr->data, skb, skb->data);
			if (!prev)	/* start of list- set head */
				skb_shinfo(skb)->frag_list = curr;
			else
				prev->next = curr;
			prev = curr;
			/*
			 * should we set skb->len and skb->data_len for each
			 * buffer being chained??? can't hurt!
			 */
			currsize =
			    min(skb->len - off,
				(unsigned int)RCVPOST_BUF_SIZE);
			curr->len = currsize;
			curr->tail += currsize;
			curr->data_len = 0;
			off += currsize;
		}
#ifdef DEBUG
		/* assert skb->len == off */
		if (skb->len != off) {
			LOGERRNAME(devdata->netdev, "%s something wrong; skb->len:%d != off:%d\n",
				   netdev->name, skb->len, off);
		}
		/* test code */
		cc = util_copy_fragsinfo_from_skb("rcvchaintest", skb,
						  RCVPOST_BUF_SIZE,
						  MAX_PHYS_INFO, testfrags);
		LOGINFNAME(devdata->netdev, "rcvchaintest returned:%d\n", cc);
		if (cc != cmdrsp->net.rcv.numrcvbufs) {
			LOGERRNAME(devdata->netdev, "**** %s Something wrong; rcvd chain length %d different from one we calculated %d\n",
				   netdev->name, cmdrsp->net.rcv.numrcvbufs,
				   cc);
		}
		for (i = 0; i < cc; i++) {
			LOGINFNAME(devdata->netdev, "test:RCVPOST_BUF_SIZE:%d[%d] pfn:%llu off:0x%x len:%d\n",
				   RCVPOST_BUF_SIZE, i, testfrags[i].pi_pfn,
				   testfrags[i].pi_off, testfrags[i].pi_len);
		}
#endif
	}

	/* set up packet's protocl type using ethernet header - this
	 * sets up skb->pkt_type & it also PULLS out the eth header
	 */
	skb->protocol = eth_type_trans(skb, netdev);

	eth = eth_hdr(skb);

	DBGINF("%d Src:%02x:%02x:%02x:%02x:%02x:%02x Dest:%02x:%02x:%02x:%02x:%02x:%02x proto:%x\n",
	       skb->pkt_type, eth->h_source[0], eth->h_source[1],
	       eth->h_source[2], eth->h_source[3], eth->h_source[4],
	       eth->h_source[5], eth->h_dest[0], eth->h_dest[1], eth->h_dest[2],
	       eth->h_dest[3], eth->h_dest[4], eth->h_dest[5], eth->h_proto);

	skb->csum = 0;
	skb->ip_summed = CHECKSUM_NONE;	/* trust me, the checksum has
					   been verified */

	do {
		if (netdev->flags & IFF_PROMISC) {
			DBGINF("IFF_PROMISC is set.\n");
			break;	/* accept all packets */
		}
		if (skb->pkt_type == PACKET_BROADCAST) {
			DBGINF("packet is broadcast.\n");
			if (netdev->flags & IFF_BROADCAST) {
				DBGINF("IFF_BROADCAST is set.\n");
				break;	/* accept all broadcast packets */
			}
		} else if (skb->pkt_type == PACKET_MULTICAST) {
			DBGINF("packet is multicast.\n");
			if (netdev->flags & IFF_ALLMULTI)
				DBGINF("IFF_ALLMULTI is set.\n");
			if ((netdev->flags & IFF_MULTICAST) &&
			    (netdev_mc_count(netdev))) {
				struct netdev_hw_addr *ha;
				int found_mc = 0;

				DBGINF("IFF_MULTICAST is set %d.\n",
				       netdev_mc_count(netdev));
				/*
				 * only accept multicast packets that we can
				 * find in our multicast address list
				 */
				netdev_for_each_mc_addr(ha, netdev) {
					if (memcmp
					    (eth->h_dest, ha->addr,
					     MAX_MACADDR_LEN) == 0) {
						DBGINF("multicast address is in our list at index:%i.\n", i);
						found_mc = 1;
						break;
					}
				}
				if (found_mc) {
					break;	/* accept packet, dest
						   matches a multicast
						   address */
				}
			}
		} else if (skb->pkt_type == PACKET_HOST) {
			DBGINF("packet is directed.\n");
			break;	/* accept packet, h_dest must match vnic
				   mac address */
		} else if (skb->pkt_type == PACKET_OTHERHOST) {
			/* something is not right */
			LOGERRNAME(devdata->netdev, "**** FAILED to deliver rcv packet to OS; name:%s Dest:%02x:%02x:%02x:%02x:%02x:%02x VNIC:%02x:%02x:%02x:%02x:%02x:%02x\n",
				   netdev->name, eth->h_dest[0], eth->h_dest[1],
				   eth->h_dest[2], eth->h_dest[3],
				   eth->h_dest[4], eth->h_dest[5],
				   netdev->dev_addr[0], netdev->dev_addr[1],
				   netdev->dev_addr[2], netdev->dev_addr[3],
				   netdev->dev_addr[4], netdev->dev_addr[5]);
		}
		/* drop packet - don't forward it up to OS */
		DBGINF("we cannot indicate this recv pkt! (netdev->flags:0x%04x, skb->pkt_type:0x%02x).\n",
		       netdev->flags, skb->pkt_type);
		devdata->n_rcv_packets_not_accepted++;
		if (repost_return(cmdrsp, devdata, skb, netdev) < 0)
			LOGERRNAME(devdata->netdev, "repost_return failed");
		return;
	} while (0);

	DBGINF("Calling netif_rx skb:%p head:%p end:%p data:%p tail:%p len:%d data_len:%d skb->nr_frags:%d\n",
	       skb, skb->head, skb->end, skb->data, skb->tail, skb->len,
	       skb->data_len, skb_shinfo(skb)->nr_frags);

	status = netif_rx(skb);
	if (status != NET_RX_SUCCESS)
		LOGWRNNAME(devdata->netdev, "status=%d\n", status);
	/*
	 * netif_rx returns various values, but "in practice most drivers
	 * ignore the return value
	 */

	skb = NULL;
	/*
	 * whether the packet got dropped or handled, the skb is freed by
	 * kernel code, so we shouldn't free it. but we should repost a
	 * new rcv buffer.
	 */
	if (repost_return(cmdrsp, devdata, skb, netdev) < 0)
		LOGVER("repost_return failed");
}

static struct visornic_devdata *
devdata_initialize(struct visornic_devdata *devdata, struct visor_device *dev)
{
	int devno = -1;

	if (!devdata) {
		ERRDRV("allocation of visornic_devdata failed\n");
		return NULL;
	}
	memset(devdata, '\0', sizeof(struct visornic_devdata));
	spin_lock(&dev_no_pool_lock);
	devno = find_first_zero_bit(dev_no_pool, MAXDEVICES);
	set_bit(devno, dev_no_pool);
	spin_unlock(&dev_no_pool_lock);
	if (devno == MAXDEVICES)
		devno = -1;
	if (devno < 0) {
		ERRDRV("unknown device\n");
		kfree(devdata);
		return NULL;
	}
	devdata->devno = devno;
	devdata->dev = dev;
	strncpy(devdata->name, dev_name(&dev->device), sizeof(devdata->name));
	init_rwsem(&devdata->lock_visor_dev);
	kref_init(&devdata->kref);
	spin_lock(&lock_all_devices);
	list_add_tail(&devdata->list_all, &list_all_devices);
	spin_unlock(&lock_all_devices);
	return devdata;
}

static void devdata_release(struct kref *mykref)
{
	struct visornic_devdata *devdata =
		container_of(mykref, struct visornic_devdata, kref);

	INFODRV("%s", __func__);
	spin_lock(&dev_no_pool_lock);
	clear_bit(devdata->devno, dev_no_pool);
	spin_unlock(&dev_no_pool_lock);
	spin_lock(&lock_all_devices);
	list_del(&devdata->list_all);
	spin_unlock(&lock_all_devices);
	kfree(devdata);
	INFODRV("%s finished", __func__);
}

static irqreturn_t
visornic_ISR(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static const struct net_device_ops visornic_dev_ops = {
	.ndo_open = visornic_open,
	.ndo_stop = visornic_close,
	.ndo_start_xmit = visornic_xmit,
	.ndo_get_stats = visornic_get_stats,
	.ndo_do_ioctl = visornic_ioctl,
	.ndo_change_mtu = visornic_change_mtu,
	.ndo_tx_timeout = visornic_xmit_timeout,
	.ndo_set_rx_mode = visornic_set_multi,
};

static void
send_rcv_posts_if_needed(struct visornic_devdata *devdata)
{
	int i;
	struct net_device *netdev;
	struct uiscmdrsp *cmdrsp = devdata->cmdrsp_rcv;
	int cur_num_rcv_bufs_to_alloc, rcv_bufs_allocated;

	/* don't do this until vnic is marked ready */
	if (!(devdata->enabled && devdata->enab_dis_acked))
		return;

	netdev = devdata->netdev;
	rcv_bufs_allocated = 0;
	/* this code is trying to prevent getting stuck here forever,
	 * but still retry it if you cant allocate them all this time.
	 */
	cur_num_rcv_bufs_to_alloc = devdata->num_rcv_bufs_could_not_alloc;
	while (cur_num_rcv_bufs_to_alloc > 0) {
		cur_num_rcv_bufs_to_alloc--;
		for (i = 0; i < devdata->num_rcv_bufs; i++) {
			if (devdata->rcvbuf[i])
				continue;
			devdata->rcvbuf[i] = alloc_rcv_buf(netdev);
			if (!devdata->rcvbuf[i]) {
				devdata->alloc_failed_in_if_needed_cnt++;
				break;
			}
			rcv_bufs_allocated++;
			post_skb(cmdrsp, devdata, devdata->rcvbuf[i]);
			devdata->chstat.extra_rcvbufs_sent++;
		}
	}
	devdata->num_rcv_bufs_could_not_alloc -= rcv_bufs_allocated;
	if (devdata->num_rcv_bufs_could_not_alloc > 0) {
		/*
		 * this path means you failed to alloc an skb in the
		 * normal path, and you are trying again later, and
		 * it still fails.
		 */
		LOGVER("attempted to recover buffers which could not be allocated and failed");
		LOGVER("rcv_bufs_allocated=%d, num_rcv_bufs_could_not_alloc=%d",
		       rcv_bufs_allocated,
		       devdata->num_rcv_bufs_could_not_alloc);
	}
}

static void
drain_queue(struct uiscmdrsp *cmdrsp, struct visornic_devdata *devdata)
{
	unsigned long flags;
	struct net_device *netdev;

	/* drain queue */
	while (1) {
		/* TODO: CLIENT ACQUIRE -- Don't really need this at the
		 * moment */
		if (!visorchannel_signalremove(devdata->dev->visorchannel,
					       IOCHAN_FROM_IOPART,
					       cmdrsp))
			break; /* queue empty */

		switch (cmdrsp->net.type) {
		case NET_RCV:
			DBGINF("Got NET_RCV\n");
			devdata->chstat.got_rcv++;
			/* process incoming packet */
			visornic_rx(cmdrsp);
			break;
		case NET_XMIT_DONE:
			DBGINF("Got NET_XMIT_DONE %p\n", cmdrsp->net.buf);
			spin_lock_irqsave(&devdata->priv_lock, flags);
			devdata->chstat.got_xmit_done++;
			if (cmdrsp->net.xmtdone.xmt_done_result) {
				LOGERRNAME(devdata->netdev,
					   "XMIT_DONE failure buf:%p\n",
					   cmdrsp->net.buf);
				devdata->chstat.xmit_fail++;
			}
			/* only call queue wake if we stopped it */
			netdev = ((struct sk_buff *)cmdrsp->net.buf)->dev;
			/* ASSERT netdev == vnicinfo->netdev; */
			if (netdev != devdata->netdev) {
				LOGERRNAME(devdata->netdev, "NET_XMIT_DONE something wrong; devdata->netdev:%p != cmdrsp->net.buf)->dev:%p\n",
					   devdata->netdev, netdev);
			} else if (netif_queue_stopped(netdev)) {
				/*
				 * check to see if we have crossed
				 * the lower watermark for
				 * netif_wake_queue()
				 */
				if (((devdata->chstat.sent_xmit >=
				    devdata->chstat.got_xmit_done) &&
				    (devdata->chstat.sent_xmit -
				    devdata->chstat.got_xmit_done <=
				    devdata->lower_threshold_net_xmits)) ||
				    ((devdata->chstat.sent_xmit <
				    devdata->chstat.got_xmit_done) &&
				    (ULONG_MAX - devdata->chstat.got_xmit_done
				    + devdata->chstat.sent_xmit <=
				    devdata->lower_threshold_net_xmits))) {
					/*
					 * enough NET_XMITs completed
					 * so can restart netif queue
					 */
					netif_wake_queue(netdev);
					devdata->flow_control_lower_hits++;
				}
			}
			skb_unlink(cmdrsp->net.buf, &devdata->xmitbufhead);
			spin_unlock_irqrestore(&devdata->priv_lock, flags);
			kfree_skb(cmdrsp->net.buf);
			break;
		case NET_RCV_ENBDIS_ACK:
			DBGINF("Got NET_RCV_ENBDIS_ACK on:%p\n",
			       (struct net_device *)
			       cmdrsp->net.enbdis.context);
			devdata->chstat.got_enbdisack++;
			netdev = (struct net_device *)
			cmdrsp->net.enbdis.context;
			spin_lock_irqsave(&devdata->priv_lock, flags);
			devdata->enab_dis_acked = 1;
			spin_unlock_irqrestore(&devdata->priv_lock, flags);

			if (devdata->server_down &&
			    devdata->server_change_state) {
				/* Inform Linux that the link is up */
				devdata->server_down = false;
				devdata->server_change_state = false;
				netif_wake_queue(netdev);
				netif_carrier_on(netdev);
			}
			break;
		case NET_CONNECT_STATUS:
			DBGINF("NET_CONNECT_STATUS, enable=:%d\n",
			       cmdrsp->net.enbdis.enable);
			netdev = devdata->netdev;
			if (cmdrsp->net.enbdis.enable == 1) {
				spin_lock_irqsave(&devdata->priv_lock, flags);
				devdata->enabled = cmdrsp->net.enbdis.enable;
				spin_unlock_irqrestore(&devdata->priv_lock,
						       flags);
				netif_wake_queue(netdev);
				netif_carrier_on(netdev);
			} else {
				netif_stop_queue(netdev);
				netif_carrier_off(netdev);
				spin_lock_irqsave(&devdata->priv_lock, flags);
				devdata->enabled = cmdrsp->net.enbdis.enable;
				spin_unlock_irqrestore(&devdata->priv_lock,
						       flags);
			}
			break;
		default:
			LOGERRNAME(devdata->netdev,
				   "Invalid net type:%d in cmdrsp\n",
				   cmdrsp->net.type);
			break;
		}
		/* cmdrsp is now available for reuse  */

		if (kthread_should_stop())
			break;
	}
}

static int
process_incoming_rsps(void *v)
{
	struct visornic_devdata *devdata = v;
	struct uiscmdrsp *cmdrsp = NULL;
	const int SZ = SIZEOF_CMDRSP;

	UIS_DAEMONIZE("vnic_incoming");
	cmdrsp = kmalloc(SZ, GFP_ATOMIC);
	if (!cmdrsp)
		complete_and_exit(&devdata->threadinfo.has_stopped, 0);

	while (1) {
		wait_event_interruptible_timeout(
			devdata->rsp_queue, (atomic_read(
					     &devdata->interrupt_rcvd) == 1),
				msecs_to_jiffies(devdata->thread_wait_ms));

		/*
		 * periodically check to see if there are any rcf bufs which
		 * need to get sent to the IOSP. This can only happen if
		 * we run out of memory when trying to allocate skbs.
		 */
		atomic_set(&devdata->interrupt_rcvd, 0);
		send_rcv_posts_if_needed(devdata);
		drain_queue(cmdrsp, devdata);
		// uisqueue_interlocked_or
		if (kthread_should_stop())
			break;
	}

	kfree(cmdrsp);
	complete_and_exit(&devdata->threadinfo.has_stopped, 0);
}

static int visornic_probe(struct visor_device *dev)
{
	struct visornic_devdata *devdata = NULL;
	struct net_device *netdev = NULL;
	int err;
	int rsp;
	int channel_offset = 0;
	irq_handler_t handler = visornic_ISR;
	struct channel_header __iomem *p_channel_header;
	struct signal_queue_header __iomem *pqhdr;
	u64 mask;
	u64 features;

	INFODRV("%s", __func__);
	netdev = alloc_etherdev(sizeof(struct visornic_devdata));
	if (!netdev) {
		LOGERR("***** FAILED to alloc etherdev\n");
		return -ENOMEM;
	}
	netdev->netdev_ops = &visornic_dev_ops;
	netdev->watchdog_timeo = VISORNIC_XMIT_TIMEOUT;

	/* Get MAC adddress from channel and read it into the device. */
	channel_offset = offsetof(struct spar_io_channel_protocol,
				  vnic.macaddr);
	visorbus_read_channel(dev, channel_offset, &netdev->dev_addr,
			      MAX_MACADDR_LEN);
	netdev->addr_len = MAX_MACADDR_LEN;
	netdev->dev.parent = &dev->device;

	devdata = devdata_initialize(netdev_priv(netdev), dev);
	if (!devdata)
		return -ENOMEM;

	devdata->interrupt_vector = -1;
	devdata->netdev = netdev;
	init_waitqueue_head(&devdata->rsp_queue);
	devdata->enabled = 0; /* not yet */
	atomic_set(&devdata->usage, 1);

	visorchipset_get_device_info(dev->chipset_bus_no,
				     dev->chipset_dev_no,
				     devdata->dev_chipset);

	/* Setup rcv bufs */
	channel_offset = offsetof(struct spar_io_channel_protocol,
				  vnic.num_rcv_bufs);
	visorbus_read_channel(dev, channel_offset, &devdata->num_rcv_bufs, 4);
	devdata->rcvbuf = kmalloc(sizeof(struct sk_buff *) *
				  devdata->num_rcv_bufs, GFP_ATOMIC);
	if (!devdata->rcvbuf) {
		free_netdev(netdev);
		return -ENOMEM;
	}
	/* set the net_xmit outstanding threshold */
	/* always leave two slots open but you should have 3 at a minimum */
	devdata->max_outstanding_net_xmits =
		max(3, ((devdata->num_rcv_bufs / 3) - 2));
	devdata->upper_threshold_net_xmits =
		max(2, devdata->max_outstanding_net_xmits - 1);
	devdata->lower_threshold_net_xmits =
		max(1, devdata->max_outstanding_net_xmits / 2);

	skb_queue_head_init(&devdata->xmitbufhead);

	/* create a cmdrsp we can use to post and unpost rcv buffers */
	devdata->cmdrsp_rcv = kmalloc(SIZEOF_CMDRSP, GFP_ATOMIC);
	if (!devdata->cmdrsp_rcv) {
		kfree(devdata->rcvbuf);
		free_netdev(netdev);
		return -ENOMEM;
	}
	INIT_WORK(&devdata->serverdown_completion,
		  visornic_serverdown_complete);
	INIT_WORK(&devdata->timeout_reset, visornic_timeout_reset);
	devdata->server_down = false;
	devdata->server_change_state = false;

	/*set the default mtu */
	channel_offset = offsetof(struct spar_io_channel_protocol,
				  vnic.mtu);
	visorbus_read_channel(dev, channel_offset, &netdev->mtu, 4);

	/* TODO: Setup Interrupt information */
	//devdata->intr = virtpcidev->intr;

	/* Let's start our threads to get responses */
	channel_offset = offsetof(struct spar_io_channel_protocol,
				  channel_header.features);
	visorbus_read_channel(dev, channel_offset, &features, 8);
	features |= ULTRA_IO_CHANNEL_IS_POLLING;
	visorbus_write_channel(dev, channel_offset, &features, 8);

	devdata->thread_wait_ms = 2;
	visor_thread_start(&devdata->threadinfo, process_incoming_rsps,
			   &devdata, "vnic_incoming");

	/* create debgug/sysfs directories */
	devdata->eth_debugfs_dir = debugfs_create_dir(netdev->name,
						      visornic_debugfs_dir);
	if (!devdata->eth_debugfs_dir) {
		visor_thread_stop(&devdata->threadinfo);
		kfree(devdata->cmdrsp_rcv);
		kfree(devdata->rcvbuf);
		free_netdev(netdev);
		return -ENOMEM;
	}

	err = register_netdev(netdev);
	if (err) {
		visor_thread_stop(&devdata->threadinfo);
		kfree(devdata->cmdrsp_rcv);
		kfree(devdata->rcvbuf);
		free_netdev(netdev);
		return err;
	}
	INFODRV("%s finished", __func__);
	return 0;
}

static void host_side_disappeared(struct visornic_devdata *devdata)
{
	down_write(&devdata->lock_visor_dev);
	sprintf(devdata->name, "<dev#%d-history>", devdata->devno);
	devdata->dev = NULL;   /* indicate device destroyed */
	up_write(&devdata->lock_visor_dev);
}

static void visornic_remove(struct visor_device *dev)
{
	struct visornic_devdata *devdata = visor_get_drvdata(dev);

	INFODRV("%s", __func__);
	if (!devdata) {
		ERRDRV("no devdata in %s", __func__);
		return;
	}
	visor_set_drvdata(dev, NULL);
	host_side_disappeared(devdata);
	kref_put(&devdata->kref, devdata_release);

	INFODRV("%s finished", __func__);
}

static int visornic_pause(struct visor_device *dev,
			  VISORBUS_STATE_COMPLETE_FUNC complete_func)
{
	INFODEV(dev_name(&dev->device), "paused");
	complete_func(dev, 0);
	return 0;
}

static int visornic_resume(struct visor_device *dev,
			   VISORBUS_STATE_COMPLETE_FUNC complete_func)
{
	INFODEV(dev_name(&dev->device), "resumed");
	complete_func(dev, 0);
	return 0;
}

static void visornic_cleanup_guts(void)
{
	visorbus_unregister_visor_driver(&visornic_driver);
	kfree(dev_no_pool);
	dev_no_pool = NULL;
}

static int visornic_init(void)
{
	INFODRV("driver version %s loaded", VERSION);

	/* DAK -- ASSERTS were here, RCVPOST_BUF_SIZE < 4K &
	   RCVPOST_BUF_SIZE < ETH_HEADER_SIZE.  We own these, why do we
	   need to assert?  No one is going to change the headers and if
	   they do oh well
	*/
	/* create workqueue for serverdown completion */
	visornic_serverdown_workqueue =
		create_singlethread_workqueue("visornic_serverdown");
	if (!visornic_serverdown_workqueue)
		return -1;

	/* creaet workqueue for tx timeout reset */
	visornic_timeout_reset_workqueue =
		create_singlethread_workqueue("visornic_timeout_reset");
	if (!visornic_timeout_reset_workqueue)
		return -1;

	visornic_debugfs_dir = debugfs_create_dir("visornic", NULL);
	debugfs_create_file("info", S_IRUSR, visornic_debugfs_dir, NULL,
			    &debugfs_info_fops);
	debugfs_create_file("enable_ints", S_IWUSR, visornic_debugfs_dir,
			    NULL, &debugfs_enable_ints_fops);

	spin_lock_init(&dev_no_pool_lock);
	dev_no_pool = kzalloc(BITS_TO_LONGS(MAXDEVICES), GFP_KERNEL);
	if (!dev_no_pool) {
		ERRDRV("Unable to create dev_no_pool");
		visornic_cleanup_guts();
		return -1;
	}
	visorbus_register_visor_driver(&visornic_driver);
	return 0;
}

static void visornic_cleanup(void)
{
	visornic_cleanup_guts();
	INFODRV("driver unloaded");
}

module_init(visornic_init);
module_exit(visornic_cleanup);

MODULE_AUTHOR("Unisys");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sPAR nic driver for sparlinux: ver "
		   VERSION);
MODULE_VERSION(VERSION);
