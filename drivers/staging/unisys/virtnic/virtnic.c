/* virtnic.c
 *
 * Copyright © 2010 - 2014 UNISYS CORPORATION
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

#define EXPORT_SYMTAB

#include <linux/kernel.h>
#ifdef CONFIG_MODVERSIONS
#include <config/modversions.h>
#endif

#include "uniklog.h"
#include "diagnostics/appos_subsystems.h"
#include "uisutils.h"
#include "uisthread.h"
#include "uisqueue.h"
#include "visorchipset.h"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/string.h>
#include <linux/tcp.h>
#include <linux/ip.h>
#include <linux/types.h>
#include <linux/uuid.h>
#include <linux/debugfs.h>

#include "virtpci.h"
#include "version.h"

/* this is shorter than using __FILE__ (full path name) in */
/* debug/info/error messages */
#define __MYFILE__ "virtnic.c"

/* turn off collecting of debug statistics */
#define VIRTNIC_STATS 0

 /* MAX_BUF = 64 lines x 32 MAXVHBA x 80 characters
 *         = 163840 bytes ~ 40 pages
 */
#define MAX_BUF 163840

/*
 * uisnic                   virtnic
 *         <---- xmit ---  virtnic_xmit(hard-start-xmit)
 *         <-- rcvpost --  open, virtnic_rx
 *	   <-- unpost ---  close
 *	   <-- enb/dis --  open, close
 *
 * open & close can't run at the same time as each other or rcv/xmit, but
 * virtnic_xmit and virtnic_rx could be running at the same time.
 * and all messages being sent to uisnic MUST be sent so if the queue is
 * full we have to retry, but we don't want to retry with a spinlock held.
 */

/*****************************************************/
/* Forward declarations                              */
/*****************************************************/
static int virtnic_probe(struct virtpci_dev *dev,
			 const struct pci_device_id *id);
static void virtnic_remove(struct virtpci_dev *dev);
static int virtnic_change_mtu(struct net_device *netdev, int new_mtu);
static int virtnic_close(struct net_device *netdev);
static struct net_device_stats *virtnic_get_stats(struct net_device *netdev);
static int virtnic_open(struct net_device *netdev);
static int virtnic_ioctl(struct net_device *netdev, struct ifreq *ifr,
			 int cmd);
static void virtnic_rx(struct uiscmdrsp *cmdrsp);
static int virtnic_xmit(struct sk_buff *skb, struct net_device *netdev);
static void virtnic_xmit_timeout(struct net_device *netdev);
static void virtnic_set_multi(struct net_device *netdev);
static int virtnic_serverdown(struct virtpci_dev *virtpcidev, u32 state);
static int virtnic_serverup(struct virtpci_dev *virtpcidev);
static void virtnic_serverdown_complete(struct work_struct *work);
static void virtnic_timeout_reset(struct work_struct *work);
static int process_incoming_rsps(void *);
static ssize_t info_debugfs_read(struct file *file, char __user *buf,
				 size_t len, loff_t *offset);
static ssize_t enable_ints_write(struct file *file,
				 const char __user *buffer,
				 size_t count, loff_t *ppos);

/*****************************************************/
/* Globals                                           */
/*****************************************************/

#define VIRTNIC_XMIT_TIMEOUT (5 * HZ)	/* Default timeout period in jiffies */
#define VIRTNIC_INFINITE_RESPONSE_WAIT 0
#define INTERRUPT_VECTOR_MASK 0x3F

static struct workqueue_struct *virtnic_serverdown_workqueue;
static struct workqueue_struct *virtnic_timeout_reset_workqueue;

static const struct pci_device_id virtnic_id_table[] = {
	{
	PCI_DEVICE(PCI_VENDOR_ID_UNISYS, PCI_DEVICE_ID_VIRTNIC)}, {
0},};
/* export virtnic_id_table */
MODULE_DEVICE_TABLE(pci, virtnic_id_table);

static struct virtpci_driver virtnic_driver = {
	.name = "uisvirtnic",
	.version = VERSION,
	.vertag = NULL,
	.id_table = virtnic_id_table,
	.probe = virtnic_probe,
	.remove = virtnic_remove,
	.suspend = virtnic_serverdown,
	.resume = virtnic_serverup
};

#define SEND_ENBDIS(ndev, state, cmdrsp, queue, insertlock, stats) { \
	DBGINF("sending rcv enb/dis netdev:%p state:%d\n", ndev, state); \
	cmdrsp->net.enbdis.enable = state; \
	cmdrsp->net.enbdis.context = ndev; \
	cmdrsp->net.type = NET_RCV_ENBDIS; \
	cmdrsp->cmdtype = CMD_NET_TYPE; \
	uisqueue_put_cmdrsp_with_lock_client(queue, cmdrsp, IOCHAN_TO_IOPART, \
					     (void *)insertlock, \
					     DONT_ISSUE_INTERRUPT, \
					     (uint64_t)NULL, \
					     OK_TO_WAIT, "vnic"); \
	stats.sent_enbdis++;\
}

struct chanstat {
	unsigned long got_rcv;	/* count of NET_RCV received */
	unsigned long got_enbdisack;	/* count of NET_RCV_ENBDIS_ACK rcvd */
	unsigned long got_xmit_done;	/* count of NET_XMIT_DONE received */
	unsigned long xmit_fail;	/* count of NET_XMIT_DONE failures */
	unsigned long sent_enbdis;	/* count of NET_RCV_ENBDIS sent */
	unsigned long sent_promisc;	/* count of NET_RCV_PROMISC sent */
	unsigned long sent_post;	/* count of NET_RCV_POST sent */
	unsigned long sent_xmit;	/* count of NET_XMIT sent */
	unsigned long reject_count;	/* count of NET_XMIT rejected because */
	/* of BUSY/queue full */
	unsigned long extra_rcvbufs_sent;
#if VIRTNIC_STATS
	unsigned long reject_jiffies_start;	/* jiffie count at start of
						   NET_XMIT rejects */
#endif /* VIRTNIC_STATS */
};

struct datachan {
	struct chaninfo chinfo;
	struct chanstat chstat;
};

struct virtnic_info {
	struct virtpci_dev *virtpcidev;
	struct net_device *netdev;
	struct net_device_stats net_stats;
	spinlock_t priv_lock; /* spinlock check for private lock */
	struct datachan datachan;
	struct sk_buff **rcvbuf;	/* rcvbuf is the array of rcv buffer */
	/* we post to */
	unsigned long long UniqueNum;

	/* the IOPART end */
	int num_rcv_bufs;	/* indicates how many receive buffers the
				   vnic will post */
	int num_rcv_bufs_could_not_alloc;
	atomic_t num_rcv_bufs_in_iovm;	/* indicates how many receive buffers
					   have actully been sent to the iovm */
	unsigned long inner_loop_limit_reached_cnt;
	unsigned long alloc_failed_in_if_needed_cnt;
	unsigned long alloc_failed_in_repost_return_cnt;

	struct sk_buff_head xmitbufhead;	/* xmitbufhead is the head of
						   the  xmit buffer list that
						   have been sent to the IOPART
						   end */
	int max_outstanding_net_xmits;	/* absolute max number of outstanding
					   xmits - should never hit this */
	int upper_threshold_net_xmits;	/* high water mark for calling
					   netif_stop_queue() */
	int lower_threshold_net_xmits;	/* high water mark for calling
					   netif_wake_queue() */
	uuid_le zoneGuid;		/* specifies the zone for the switch in
					   which this VNIC resides  */
	struct uiscmdrsp *cmdrsp_rcv;	/* cmdrsp_rcv is used for
					   posting/unposting rcv buffers */
	unsigned short enabled;	/* 0 disabled 1 enabled to receive */
	unsigned short enab_dis_acked;	/* NET_RCV_ENABLE/DISABLE acked by
					   uisnic */
	atomic_t usage;			/* count of users */
	unsigned short old_flags;	/* flags as they were prior to
					   set_multicast_list */
	struct uiscmdrsp *xmit_cmdrsp;	/* used to issue NET_XMIT -  there is
					   never more that one xmit in progress
					   at a time */
	struct dentry *eth_debugfs_dir;	/* this points to /proc/eth?
						   directory */
	struct dentry *zone_debugfs_entry;	/* this points to
						   /proc/virtnic/eth?/zone */
	/* file */
	struct dentry *clientstr_debugfs_entry;/* this points to
						  /proc/virtnic/eth?/clientstr
						  file  */
	struct irq_info intr;	/* use recvInterrupt info  to connect
					   to this to receive interrupts when
					   IOs complete */
	int interrupt_vector;
	int thread_wait_ms;
	int queuefullmsg_logged;	/* flag for throttling queue full */
	/* messages */
	/* some debug counters */
	ulong n_rcv0;			/* # rcvs of 0 buffers */
	ulong n_rcv1;			/* # rcvs of 1 buffer */
	ulong n_rcv2;			/* # rcvs of 2 buffers */
	ulong n_rcvx;			/* # rcvs of >2 buffers */
	ulong found_repost_rcvbuf_cnt;	/* #time we called repost_rcvbuf_cnt */
	ulong repost_found_skb_cnt;	/* # times found the skb */
	ulong n_repost_deficit;		/* # times we couldn't find all of the
					   rcv buffers */
	ulong bad_rcv_buf;		/* # times we neglected to
					     free the rcv skb because
					     we didn't know where it
					     came from */
	ulong n_rcv_packet_not_accepted;	/* # bogus recv packets */
	bool server_down;
	bool server_change_state;
	unsigned long long interrupts_rcvd;
	unsigned long long interrupts_notme;
	unsigned long long interrupts_disabled;
	unsigned long long busy_cnt;
	unsigned long long flow_control_upper_hits;
	unsigned long long flow_control_lower_hits;
	struct work_struct serverdown_completion;
	struct work_struct timeout_reset;
	uint64_t __iomem *flags_addr;
	atomic_t interrupt_rcvd;
	wait_queue_head_t rsp_queue;
};

struct virtnic_devices_open {
	struct net_device *netdev;
	struct virtnic_info *vnicinfo;
};

static ssize_t show_zone(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct net_device *net = to_net_dev(dev);
	struct virtnic_info *vnicinfo = netdev_priv(net);

	return scnprintf(buf, PAGE_SIZE, "%pUL\n", &vnicinfo->zoneGuid);
}

static ssize_t show_clientstr(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct net_device *net = to_net_dev(dev);
	struct virtnic_info *vnicinfo = netdev_priv(net);
	ULTRA_IO_CHANNEL_PROTOCOL *chan =
		(ULTRA_IO_CHANNEL_PROTOCOL *)vnicinfo->
		datachan.chinfo.queueinfo->chan;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			(char *)&chan->clientString);
}
static DEVICE_ATTR(clientstr, S_IRUGO, show_clientstr, NULL);
static DEVICE_ATTR(zone, S_IRUGO, show_zone, NULL);

#define VIRTNICSOPENMAX 32
/* array of open devices maintained by open() and close() */
static struct virtnic_devices_open num_virtnic_open[VIRTNICSOPENMAX];
static struct dentry *virtnic_debugfs_dir;

static const struct file_operations debugfs_info_fops = {
	.read = info_debugfs_read,
};

static const struct file_operations debugfs_enable_ints_fops = {
	.write = enable_ints_write,
};

/*****************************************************/
/* Probe Remove Functions                            */
/*****************************************************/
/* set up net.rcvpost struct in cmdrsp.
 * all rcv buf skb are allocated at RCVPOST_BUF_SIZE, so length is
 * RCVPOST_BUF_SIZE by default. and since RCVPOST_BUF_SIZE < 2048, one
 * phys_info struct can describe the rcv buf.
 */
static inline void
post_skb(struct uiscmdrsp *cmdrsp,
	 struct virtnic_info *vnicinfo, struct sk_buff *skb)
{
	cmdrsp->net.buf = skb;
	cmdrsp->net.rcvpost.frag.pi_pfn = page_to_pfn(virt_to_page(skb->data));
	cmdrsp->net.rcvpost.frag.pi_off =
		(unsigned long)skb->data & PI_PAGE_MASK;
	cmdrsp->net.rcvpost.frag.pi_len = skb->len;
	cmdrsp->net.rcvpost.UniqueNum = vnicinfo->UniqueNum;

	DBGINF("RCV_POST skb:%p pfn:%llu off:%x len:%d\n", skb,
	       cmdrsp->net.rcvpost.frag.pi_pfn,
	       cmdrsp->net.rcvpost.frag.pi_off,
	       cmdrsp->net.rcvpost.frag.pi_len);
	if ((cmdrsp->net.rcvpost.frag.pi_off + skb->len) > PI_PAGE_SIZE) {
		LOGERRNAME(vnicinfo->netdev,
			   "**** pi_off:0x%x pi_len:%d SPAN ACROSS A PAGE\n",
			   cmdrsp->net.rcvpost.frag.pi_off, skb->len);
	} else {
		cmdrsp->net.type = NET_RCV_POST;
		cmdrsp->cmdtype = CMD_NET_TYPE;
		uisqueue_put_cmdrsp_with_lock_client(vnicinfo->datachan.chinfo.
						     queueinfo, cmdrsp,
						     IOCHAN_TO_IOPART,
						     (void *)&vnicinfo->
						     datachan.chinfo.insertlock,
						     DONT_ISSUE_INTERRUPT,
						     (uint64_t)NULL,
						     OK_TO_WAIT,
						     "vnic");
		atomic_inc(&vnicinfo->num_rcv_bufs_in_iovm);
		vnicinfo->datachan.chstat.sent_post++;
	}
}

static irqreturn_t
virtnic_ISR(int irq, void *dev_id)
{
	struct virtnic_info *vnicinfo = (struct virtnic_info *)dev_id;

	struct channel_header __iomem *p_channel_header;

	struct signal_queue_header __iomem *pqhdr;
	uint64_t mask;
	unsigned long long rc1;

	if (vnicinfo == NULL)
		return IRQ_NONE;
	vnicinfo->interrupts_rcvd++;
	p_channel_header = vnicinfo->datachan.chinfo.queueinfo->chan;
	if (((readq(&p_channel_header->features) &
	      ULTRA_IO_IOVM_IS_OK_WITH_DRIVER_DISABLING_INTS) != 0) &&
	    ((readq(&p_channel_header->features) &
	      ULTRA_IO_DRIVER_DISABLES_INTS) != 0)) {
		/*
		 * should not enter this path because we setup without
		 * DRIVER_DISABLES_INTS.
		 */
		vnicinfo->interrupts_disabled++;
		mask = ~ULTRA_CHANNEL_ENABLE_INTS;
		rc1 = uisqueue_interlocked_and(vnicinfo->flags_addr, mask);
	}
	if (spar_signalqueue_empty(p_channel_header, IOCHAN_FROM_IOPART)) {
		vnicinfo->interrupts_notme++;
		return IRQ_NONE;
	}
	pqhdr = (struct signal_queue_header __iomem *)
		((char __iomem *)p_channel_header +
		 readq(&p_channel_header->ch_space_offset)) +
		IOCHAN_FROM_IOPART;
	writeq(readq(&pqhdr->num_irq_received) + 1,
	       &pqhdr->num_irq_received);
	atomic_set(&vnicinfo->interrupt_rcvd, 1);
	wake_up_interruptible(&vnicinfo->rsp_queue);
	return IRQ_HANDLED;
}

static const struct net_device_ops virtnic_dev_ops = {
	.ndo_open = virtnic_open,
	.ndo_stop = virtnic_close,
	.ndo_start_xmit = virtnic_xmit,
	.ndo_get_stats = virtnic_get_stats,
	.ndo_do_ioctl = virtnic_ioctl,
	.ndo_change_mtu = virtnic_change_mtu,
	.ndo_tx_timeout = virtnic_xmit_timeout,
	.ndo_set_rx_mode = virtnic_set_multi,
};

static int
virtnic_probe(struct virtpci_dev *virtpcidev, const struct pci_device_id *id)
{
	struct net_device *netdev = NULL;
	struct virtnic_info *vnicinfo;
	int err;
	int rsp;
	irq_handler_t handler = virtnic_ISR;
	struct channel_header __iomem *p_channel_header;
	struct signal_queue_header __iomem *pqhdr;
	uint64_t mask;

#define RETFAIL(res) {\
		kfree(vnicinfo->cmdrsp_rcv);  \
		kfree(vnicinfo->xmit_cmdrsp); \
		kfree(vnicinfo->rcvbuf);      \
		if (vnicinfo->interrupt_vector != -1)		\
			free_irq(vnicinfo->interrupt_vector, vnicinfo); \
		if (netdev)						\
			free_netdev(netdev);				\
		return res;						\
}

	DBGINF("virtpci_dev:%p\n", virtpcidev);
	DBGINF("virtpcidev busNo<<%d>>devNo<<%d>>",
	       virtpcidev->busNo, virtpcidev->deviceNo);
	netdev = alloc_etherdev(sizeof(struct virtnic_info));
	if (netdev == NULL) {
		LOGERR("**** FAILED to alloc etherdev\n");
		return -ENOMEM;
	}
	netdev->netdev_ops = &virtnic_dev_ops;
	netdev->watchdog_timeo = VIRTNIC_XMIT_TIMEOUT;

	memcpy(netdev->dev_addr, virtpcidev->net.mac_addr, MAX_MACADDR_LEN);
	netdev->addr_len = MAX_MACADDR_LEN;
	/* netdev->name should be ethx already */
	netdev->dev.parent = &virtpcidev->generic_dev;

	/* setup our private struct */
	vnicinfo = netdev_priv(netdev);
	memset(vnicinfo, 0, sizeof(struct virtnic_info));
	vnicinfo->interrupt_vector = -1;
	vnicinfo->netdev = netdev;
	vnicinfo->virtpcidev = virtpcidev;
	init_waitqueue_head(&vnicinfo->rsp_queue);
	spin_lock_init(&vnicinfo->priv_lock);
	vnicinfo->datachan.chinfo.queueinfo = &virtpcidev->queueinfo;
	spin_lock_init(&vnicinfo->datachan.chinfo.insertlock);
	vnicinfo->enabled = 0;	/* not yet */
	atomic_set(&vnicinfo->usage, 1);	/* starting val */
	vnicinfo->zoneGuid = virtpcidev->net.zoneGuid;
	vnicinfo->num_rcv_bufs = virtpcidev->net.num_rcv_bufs;
	LOGINFNAME(vnicinfo->netdev, "num_rcv_bufs =  %d\n",
		   vnicinfo->num_rcv_bufs);
	vnicinfo->rcvbuf = kmalloc(sizeof(struct sk_buff *) *
				   vnicinfo->num_rcv_bufs, GFP_ATOMIC);
	if (vnicinfo->rcvbuf == NULL) {
		LOGERRNAME(vnicinfo->netdev,
			   "**** FAILED to allocate memory for %d receive buffers.\n",
			   vnicinfo->num_rcv_bufs);
		RETFAIL(-ENOMEM);
	}
	memset(vnicinfo->rcvbuf, 0,
	       sizeof(struct sk_buff *) * vnicinfo->num_rcv_bufs);
	/* set the net_xmit outstanding threshold */
	vnicinfo->max_outstanding_net_xmits =
	    max(3, ((vnicinfo->num_rcv_bufs / 3) - 2));
	/* always leave two slots open but you should have 3 at a minumum */
	LOGINFNAME(vnicinfo->netdev, "max_outstanding_net_xmits =  %d\n",
		   vnicinfo->max_outstanding_net_xmits);
	vnicinfo->upper_threshold_net_xmits =
	    max(2, vnicinfo->max_outstanding_net_xmits - 1);
	LOGINFNAME(vnicinfo->netdev, "upper_threshold_net_xmits =  %d\n",
		   vnicinfo->upper_threshold_net_xmits);
	vnicinfo->lower_threshold_net_xmits =
	    max(1, vnicinfo->max_outstanding_net_xmits / 2);
	LOGINFNAME(vnicinfo->netdev, "lower_threshold_net_xmits =  %d\n",
		   vnicinfo->lower_threshold_net_xmits);
	skb_queue_head_init(&vnicinfo->xmitbufhead);

	/* create a cmdrsp we can use to post and unpost rcv buffers  */
	vnicinfo->cmdrsp_rcv = kmalloc(SIZEOF_CMDRSP, GFP_ATOMIC);
	if (vnicinfo->cmdrsp_rcv == NULL) {
		LOGERRNAME(vnicinfo->netdev,
			   "**** FAILED to allocate cmdrsp to use for posting rcv buffers\n");
		RETFAIL(-ENOMEM);
	}
	vnicinfo->xmit_cmdrsp = kmalloc(SIZEOF_CMDRSP, GFP_ATOMIC);
	if (vnicinfo->xmit_cmdrsp == NULL) {
		LOGERRNAME(vnicinfo->netdev,
			   "**** FAILED to allocate cmdrsp to use for xmits\n");
		RETFAIL(-ENOMEM);
	}
	INIT_WORK(&vnicinfo->serverdown_completion,
		  virtnic_serverdown_complete);
	INIT_WORK(&vnicinfo->timeout_reset, virtnic_timeout_reset);
	vnicinfo->server_down = false;
	vnicinfo->server_change_state = false;

	/* set the default mtu */
	netdev->mtu = virtpcidev->net.mtu;

	vnicinfo->intr = virtpcidev->intr;
	/* buffers will be allocated in open using mtu */

	/* save off netdev in virtpcidev  */
	virtpcidev->net.netdev = netdev;

	/* start thread that will receive responses */
	writeq(readq(&vnicinfo->datachan.chinfo.queueinfo->chan->features) |
	       ULTRA_IO_CHANNEL_IS_POLLING,
	       &vnicinfo->datachan.chinfo.queueinfo->chan->features);
	DBGINF("starting rsp thread queueinfo:%p threadinfo:%p\n",
	       vnicinfo->datachan.chinfo.queueinfo,
	       &vnicinfo->datachan.chinfo.threadinfo);
	p_channel_header = vnicinfo->datachan.chinfo.queueinfo->chan;
	pqhdr = (struct signal_queue_header __iomem *)
		((char __iomem *)p_channel_header +
		 readq(&p_channel_header->ch_space_offset)) +
	    IOCHAN_FROM_IOPART;
	vnicinfo->flags_addr = (__force uint64_t __iomem *)&pqhdr->features;
	vnicinfo->thread_wait_ms = 2;
	if (!uisthread_start(&vnicinfo->datachan.chinfo.threadinfo,
			     process_incoming_rsps, &vnicinfo->datachan,
			     "vnic_incoming")) {
		LOGERRNAME(vnicinfo->netdev, "**** FAILED to start thread\n");
		RETFAIL(-ENODEV);
	}

	/* register_netdev */
	LOGINFNAME(vnicinfo->netdev, "sendInterruptHandle=0x%16llX",
		   (unsigned long long)vnicinfo->intr.send_irq_handle);
	LOGINFNAME(vnicinfo->netdev, "recvInterruptHandle=0x%16llX",
		   (unsigned long long)vnicinfo->intr.recv_irq_handle);
	LOGINFNAME(vnicinfo->netdev, "recvInterruptVector=0x%8X",
		   vnicinfo->intr.recv_irq_vector);
	LOGINFNAME(vnicinfo->netdev, "recvInterruptShared=0x%2X",
		   vnicinfo->intr.recv_irq_shared);
	LOGINFNAME(vnicinfo->netdev, "netdev->name=%s", netdev->name);
	vnicinfo->interrupt_vector = vnicinfo->intr.recv_irq_handle &
	    INTERRUPT_VECTOR_MASK;
	netdev->irq = vnicinfo->interrupt_vector;
	err = register_netdev(netdev);
	if (err) {
		uisthread_stop(&vnicinfo->datachan.chinfo.threadinfo);
		RETFAIL(err);
	}

	/* create proc/ethx directory */
	vnicinfo->eth_debugfs_dir = debugfs_create_dir(netdev->name,
						       virtnic_debugfs_dir);
	if (!vnicinfo->eth_debugfs_dir) {
		LOGERRNAME(vnicinfo->netdev,
			   "****FAILED to create proc dir entry:%s\n",
			   netdev->name);
		uisthread_stop(&vnicinfo->datachan.chinfo.threadinfo);
		RETFAIL(-ENODEV);
	}

	if (device_create_file(&netdev->dev, &dev_attr_zone) < 0) {
		uisthread_stop(&vnicinfo->datachan.chinfo.threadinfo);
		RETFAIL(-ENODEV);
	}
	if (device_create_file(&netdev->dev, &dev_attr_clientstr) < 0) {
		device_remove_file(&netdev->dev, &dev_attr_zone);
		uisthread_stop(&vnicinfo->datachan.chinfo.threadinfo);
		RETFAIL(-ENODEV);
	}
	/* create proc/ethx directory  */
	rsp = request_irq(vnicinfo->interrupt_vector, handler, IRQF_SHARED,
			  netdev->name, vnicinfo);
	if (rsp != 0) {
		LOGERRNAME(vnicinfo->netdev,
			   "request_irq(%d) uislib_vnic_ISR request failed with rsp=%d\n",
			   vnicinfo->interrupt_vector, rsp);
		vnicinfo->interrupt_vector = -1;
	} else {
		uint64_t __iomem *features_addr =
		    &vnicinfo->datachan.chinfo.queueinfo->chan->features;
		LOGERRNAME(vnicinfo->netdev,
			   "request_irq(%d) uislib_vnic_ISR request succeeded\n",
			   vnicinfo->interrupt_vector);
		mask = ~(ULTRA_IO_CHANNEL_IS_POLLING |
			 ULTRA_IO_DRIVER_DISABLES_INTS |
			 ULTRA_IO_DRIVER_SUPPORTS_ENHANCED_RCVBUF_CHECKING);
		uisqueue_interlocked_and(features_addr, mask);
		mask = ULTRA_IO_DRIVER_ENABLES_INTS |
		    ULTRA_IO_DRIVER_SUPPORTS_ENHANCED_RCVBUF_CHECKING;
		uisqueue_interlocked_or(features_addr, mask);

		vnicinfo->thread_wait_ms = 2000;
	}

	LOGINFNAME(vnicinfo->netdev,
		   "Added VirtNic:%p %s insertlock:%p %02x:%02x:%02x:%02x:%02x:%02x\n",
		   netdev, netdev->name, &vnicinfo->datachan.chinfo.insertlock,
		   netdev->dev_addr[0], netdev->dev_addr[1],
		   netdev->dev_addr[2], netdev->dev_addr[3],
		   netdev->dev_addr[4], netdev->dev_addr[5]);
	return 0;
}

static void
virtnic_remove(struct virtpci_dev *virtpcidev)
{
	struct net_device *netdev = virtpcidev->net.netdev;
	struct virtnic_info *vnicinfo;

	vnicinfo = netdev_priv(netdev);

	LOGINFNAME(vnicinfo->netdev,
		   "virtpcidev:%p netdev:%p name:%s vnicinfo:%p\n",
		   virtpcidev, netdev, netdev->name, vnicinfo);
	LOGINFNAME(vnicinfo->netdev,
		   "virtpcidev busNo<<%d>>devNo<<%d>>",
		   virtpcidev->busNo, virtpcidev->deviceNo);
	/* REMOVE netdev */
	DBGINF("unregistering netdev\n");
	if (vnicinfo->interrupt_vector != -1)
		free_irq(vnicinfo->interrupt_vector, vnicinfo);
	unregister_netdev(netdev);
	/* this is going to call virtnic_close which will send out */
	/* disable don't take thread down until after that */
	uisthread_stop(&vnicinfo->datachan.chinfo.threadinfo);

	/* freeing of rcv bufs should have happened in close. */
	/* free cmdrsp we allocated for rcv post/unpost */
	kfree(vnicinfo->cmdrsp_rcv);
	kfree(vnicinfo->xmit_cmdrsp);

	/* delete proc file entries */
	device_remove_file(&netdev->dev, &dev_attr_zone);
	device_remove_file(&netdev->dev, &dev_attr_clientstr);

	debugfs_remove(vnicinfo->eth_debugfs_dir);
	LOGINFNAME(vnicinfo->netdev, "removed dentry %s\n",
		   netdev->name);

	kfree(vnicinfo->rcvbuf);
	free_netdev(netdev);

	LOGINF("virtnic removed\n");
}

/*****************************************************/
/* NIC statistics handling					         */
/*****************************************************/

/* update rcv stats - locking done by invoker */
#define UPD_RCV_STATS { \
	vnicinfo->net_stats.rx_packets++;  \
	vnicinfo->net_stats.rx_bytes += skb->len;  \
}

/* update xmt stats - locking done by invoker */
#define UPD_XMT_STATS { \
	vnicinfo->net_stats.tx_packets++;  \
	vnicinfo->net_stats.tx_bytes += skb->len;  \
}

static struct net_device_stats *
virtnic_get_stats(struct net_device *netdev)
{
	struct virtnic_info *vnicinfo = netdev_priv(netdev);

	/* take this opportunity to print out our internal stats */
	DBGINF
	    ("NET_RCV_ENBDIS sent: %ld     NET_RCV_ENBDIS_ACK received: %ld\n",
	     vnicinfo->datachan.chstat.sent_enbdis,
	     vnicinfo->datachan.chstat.got_enbdisack);

	DBGINF("NET_RCV received: %ld        NET_RCV_POST sent: %ld\n",
	       vnicinfo->datachan.chstat.got_rcv,
	       vnicinfo->datachan.chstat.sent_post);

	DBGINF("extra NET_RCV_POST sent: %ld\n",
	       vnicinfo->datachan.chstat.extra_rcvbufs_sent);

	DBGINF("NET_XMIT sent: %ld           NET_XMIT_DONE received: %ld\n",
	       vnicinfo->datachan.chstat.sent_xmit,
	       vnicinfo->datachan.chstat.got_xmit_done);

	DBGINF("XMIT failures: %ld           NET_RCV_PROMISC sent: %ld\n",
	       vnicinfo->datachan.chstat.xmit_fail,
	       vnicinfo->datachan.chstat.sent_promisc);

	DBGINF("XMIT reject/busy: %ld\n",
	       vnicinfo->datachan.chstat.reject_count);

	return &vnicinfo->net_stats;
}

/*****************************************************/
/* Local functions                                   */
/*****************************************************/

/*
 * This function allocates skb, skb->data for first fragment. If Mtu
 * size is > default, it allocates frags.
 */
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
	skb->data_len = 0;	/* dev_alloc_skb already zeroes it out.
				   for clarification. */
	return skb;
}

static int
init_rcv_bufs(struct net_device *netdev, struct virtnic_info *vnicinfo)
{
	int i, count;

	DBGINF("netdev->name <<%s>>", netdev->name);
	/*
	 * allocate fixed number of receive buffers to post to uisnic
	 * post receive buffers after we've allocated a required
	 * amount
	 */
	for (i = 0; i < vnicinfo->num_rcv_bufs; i++) {
		vnicinfo->rcvbuf[i] = alloc_rcv_buf(netdev);
		if (!vnicinfo->rcvbuf[i])
			break;	/* if we failed to allocate one let us stop */
	}
	if (i < vnicinfo->num_rcv_bufs) {
		LOGWRNNAME(vnicinfo->netdev,
			   "only allocated %d of %d receive buffers", i,
			   vnicinfo->num_rcv_bufs);
		if (i == 0) {
			/* couldn't even allocate one - bail out */
			LOGERRNAME(vnicinfo->netdev,
				   "**** FAILED to allocate any rcv buffers\n");
			return -ENOMEM;
		}
	}
	count = i;
	/* Ensure we can alloc 2/3rd of the requested number of
	 * buffers. 2/3 is an arbitraty choice; used also in ndis
	 * init.c.
	 */
	if (count < ((2 * vnicinfo->num_rcv_bufs) / 3)) {
		LOGERRNAME(vnicinfo->netdev,
			   "**** FAILED to allocate enough rcv bufs; allocated only:%d MAX_NET_RCV_BUFS:%d\n",
			   count, MAX_NET_RCV_BUFS);
		/* free receive buffers we did allocate and then bail out */
		for (i = 0; i < count; i++) {
			kfree_skb(vnicinfo->rcvbuf[i]);
			vnicinfo->rcvbuf[i] = NULL;
		}
		return -ENOMEM;
	}

	/* post receive buffers to receive incoming input - without holding */
	/* lock - we've not enabled nor started the queue so there shouldn't */
	/* be any rcv or xmit activity */
	for (i = 0; i < count; i++)
		post_skb(vnicinfo->cmdrsp_rcv, vnicinfo, vnicinfo->rcvbuf[i]);

	/* push through with what buffers we've got - unallocated ones will */
	/* be null */
	LOGINFNAME(vnicinfo->netdev, "Allocated & posted %d rcv buffers\n",
		   count);

	return 0;
}

/* Sends disable to IOVM and frees receive buffers that were posted to
 * IOVM (cleared by IOVM when disable is received)
 * returns 0 on success, negative number on failure
 *
 * timeout is defined in msecs (timeout of 0 specifies infinite wait)
 */
static int
virtnic_disable_with_timeout(struct net_device *netdev, const int timeout)
{
	struct virtnic_info *vnicinfo = netdev_priv(netdev);
	int i, count = 0;
	unsigned long flags;
	int wait = 0;

	LOGINFNAME(vnicinfo->netdev, "netdev->name <<%s>>", netdev->name);
	/* stop the transmit queue so nothing more can be transmitted */
	netif_stop_queue(netdev);

	/* send a msg telling the other end we are stopping incoming pkts */
	spin_lock_irqsave(&vnicinfo->priv_lock, flags);
	vnicinfo->enabled = 0;
	vnicinfo->enab_dis_acked = 0;	/* must wait for ack */
	spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);

	/* send disable and wait for ack - don't hold lock when
	 * sending disable because if the queue is full, insert might
	 * sleep.
	 */
	SEND_ENBDIS(netdev, 0, vnicinfo->cmdrsp_rcv,
		    vnicinfo->datachan.chinfo.queueinfo,
		    &vnicinfo->datachan.chinfo.insertlock,
		    vnicinfo->datachan.chstat);

	LOGINFNAME(vnicinfo->netdev,
		   "Waiting for ENBDIS ACK before freeing rcv buffers...\n");
	/* wait for ack to arrive before we try to free rcv buffers
	 * NOTE: the other end automatically unposts the rcv buffers
	 * when it gets a disable.
	 */
	while ((timeout == VIRTNIC_INFINITE_RESPONSE_WAIT) ||
	       (wait < timeout)) {
		spin_lock_irqsave(&vnicinfo->priv_lock, flags);
		if (vnicinfo->n_rcv_packet_not_accepted) {
			/* now we can continue with disable */
			break;
		} else if (vnicinfo->server_down ||
			vnicinfo->server_change_state) {
			LOGERRNAME(vnicinfo->netdev,
				   "IOVM is down so disable will not be acknowledged.  Stopping wait.\n");
			spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
			return -1;
		}
		set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
		wait += schedule_timeout(msecs_to_jiffies(10));
	}
	if (!vnicinfo->n_rcv_packet_not_accepted) {
		LOGERRNAME(vnicinfo->netdev,
			   "IOVM did not respond to Disable in allocated time (%d msecs).\n",
			   timeout);
		spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
		return -1;
	}
	LOGINFNAME(vnicinfo->netdev,
		   "Got ENBDIS ACK; now waiting for 0 usage count...\n");

	/*
	 * wait for usage to go to 1 (no other users) before freeing
	 * rcv buffers
	 */
	if (atomic_read(&vnicinfo->usage) > 1) {
		/* wait for usage count to be 1 */
		while (1) {
			set_current_state(TASK_INTERRUPTIBLE);
			spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
			schedule_timeout(msecs_to_jiffies(10));
			spin_lock_irqsave(&vnicinfo->priv_lock, flags);
			if (atomic_read(&vnicinfo->usage) == 1) {
				break;	/* go do work and only after
					   that give up lock */
			}
		}
	}
	/* we've set enabled to 0, so we can give up the lock. */
	spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
	LOGINFNAME(vnicinfo->netdev,
		   "Usage count is 0; freeing the rcv buffers now\n");

	/* free rcv buffers - other end has automatically unposted
	 * them on disable
	 */
	for (i = 0; i < vnicinfo->num_rcv_bufs; i++) {
		if (vnicinfo->rcvbuf[i]) {
			kfree_skb(vnicinfo->rcvbuf[i]);
			vnicinfo->rcvbuf[i] = NULL;
			count++;
		}
	}
	LOGINFNAME(vnicinfo->netdev, "Freed %d rcv bufs\n", count);

	/* remove references from debug array */
	for (i = 0; i < VIRTNICSOPENMAX; i++) {
		if (num_virtnic_open[i].netdev == netdev) {
			num_virtnic_open[i].netdev = NULL;
			num_virtnic_open[i].vnicinfo = NULL;
			break;
		}
	}

	return 0;
}

/* Wait indefinitely for IOVM to acknowledge disable request */
static int
virtnic_disable(struct net_device *netdev)
{
	return virtnic_disable_with_timeout(netdev,
					    VIRTNIC_INFINITE_RESPONSE_WAIT);
}

/* Sends enable to IOVM, inits, and  posts receive buffers to IOVM
 * returns 0 on success, negative number on failure
 *
 * timeout is defined in msecs (timeout of 0 specifies infinite wait)
 */
static int
virtnic_enable_with_timeout(struct net_device *netdev, const int timeout)
{
	int i;
	struct virtnic_info *vnicinfo = netdev_priv(netdev);
	unsigned long flags;
	int wait = 0;

	/* NOTE: the other end automatically unposts the rcv buffers when
	 * it gets a disable.
	 */
	i = init_rcv_bufs(netdev, vnicinfo);
	if (i < 0)
		return i;

	spin_lock_irqsave(&vnicinfo->priv_lock, flags);
	vnicinfo->enabled = 1;
	/* now we're ready, let's send an ENB to uisnic but until we
	 * get an ACK back from uisnic, we'll drop the packets
	 */
	vnicinfo->n_rcv_packet_not_accepted = 0;
	spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);

	/* send enable and wait for ack - don't hold lock when sending
	 * enable because if the queue is full, insert might sleep.
	 */
	SEND_ENBDIS(netdev, 1, vnicinfo->cmdrsp_rcv,
		    vnicinfo->datachan.chinfo.queueinfo,
		    &vnicinfo->datachan.chinfo.insertlock,
		    vnicinfo->datachan.chstat);

	LOGINFNAME(vnicinfo->netdev, "netdev->name <<%s>>", netdev->name);
	LOGINFNAME(vnicinfo->netdev,
		   "Waiting for ENBDIS ACK before starting device queue...\n");
	while ((timeout == VIRTNIC_INFINITE_RESPONSE_WAIT) ||
	       (wait < timeout)) {
		spin_lock_irqsave(&vnicinfo->priv_lock, flags);
		if (vnicinfo->enab_dis_acked) {
			/* now we can continue  */
			break;
		} else if (vnicinfo->server_down ||
			   vnicinfo->server_change_state) {
			/* IOVM is going down so don't wait for a response */
			LOGERRNAME(vnicinfo->netdev,
				   "IOVM is down so enable will not be acknowledged.  Stopping wait.\n");
			spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
			return -1;
		}
		set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
		wait += schedule_timeout(msecs_to_jiffies(10));
	}
	if (!vnicinfo->enab_dis_acked) {
		LOGERRNAME(vnicinfo->netdev,
			   "IOVM did not respond to Enable in allocated time (%d msecs).\n",
			   timeout);
		spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
		return -1;
	}
	spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
	LOGINFNAME(vnicinfo->netdev, "Got ENBDIS ACK\n");

	/* find an open slot in the array to save off VirtNic
	 * references for debug
	 */
	for (i = 0; i < VIRTNICSOPENMAX; i++) {
		if (num_virtnic_open[i].netdev == NULL) {
			num_virtnic_open[i].netdev = netdev;
			num_virtnic_open[i].vnicinfo = vnicinfo;
			break;
		}
	}
	if (i == VIRTNICSOPENMAX)
		LOGINFNAME(vnicinfo->netdev,
			   "No storage for debug ref for netdev = 0x%p vnicinfo = 0x%p\n",
			   netdev, vnicinfo);

	return 0;
}

/* Wait indefinitely for IOVM to acknowledge enable request */
static int
virtnic_enable(struct net_device *netdev)
{
	return virtnic_enable_with_timeout(netdev,
		VIRTNIC_INFINITE_RESPONSE_WAIT);
}

static void
send_rcv_posts_if_needed(struct virtnic_info *vnicinfo)
{
	int i;
	struct net_device *netdev;
	struct uiscmdrsp *cmdrsp = vnicinfo->cmdrsp_rcv;
	int cur_num_rcv_bufs_to_alloc, rcv_bufs_allocated;

	if (!(vnicinfo->enabled && vnicinfo->enab_dis_acked)) {
		/* dont do this until vnic is marked ready. */
		return;
	}
	netdev = vnicinfo->netdev;
	rcv_bufs_allocated = 0;
	/* this code is trying to prevent getting stuck here forever,
	 * but still retry it if you cant allocate them all this
	 * time.
	 */
	cur_num_rcv_bufs_to_alloc = vnicinfo->num_rcv_bufs_could_not_alloc;
	while (cur_num_rcv_bufs_to_alloc > 0) {
		cur_num_rcv_bufs_to_alloc--;
		for (i = 0; i < vnicinfo->num_rcv_bufs; i++) {
			if (vnicinfo->rcvbuf[i] != NULL)
				continue;
			vnicinfo->rcvbuf[i] = alloc_rcv_buf(netdev);
			if (!vnicinfo->rcvbuf[i]) {
				LOGVER("**** %s FAILED to allocate new rcv buf - no REPOST\n",
				       netdev->name);
				vnicinfo->
				    alloc_failed_in_if_needed_cnt++;
				break;
			} else {
				rcv_bufs_allocated++;
				post_skb(cmdrsp, vnicinfo,
					 vnicinfo->rcvbuf[i]);
				vnicinfo->datachan.chstat.
				    extra_rcvbufs_sent++;
			}
		}
	}
	vnicinfo->num_rcv_bufs_could_not_alloc -= rcv_bufs_allocated;
	if (vnicinfo->num_rcv_bufs_could_not_alloc > 0) {
		/*
		 * this path means you failed to alloc an skb in the
		 * normal path, and you are trying again later, and
		 * it still fails.
		 */
		LOGVER("attempted to recover buffers which could not be allocated and failed");
		LOGVER("rcv_bufs_allocated=%d, num_rcv_bufs_could_not_alloc=%d",
		       rcv_bufs_allocated,
		       vnicinfo->num_rcv_bufs_could_not_alloc);
	}
}

static void
drain_queue(struct datachan *dc, struct uiscmdrsp *cmdrsp,
	    struct virtnic_info *vnicinfo)
{
	unsigned long flags;
	int qrslt;
	struct net_device *netdev;

	/* drain queue */
	while (1) {
		spin_lock_irqsave(&dc->chinfo.insertlock, flags);
		if (!spar_channel_client_acquire_os(dc->chinfo.queueinfo->chan,
						    "vnic")) {
			spin_unlock_irqrestore(&dc->chinfo.insertlock,
					       flags);
			break;
		}
		qrslt = uisqueue_get_cmdrsp(dc->chinfo.queueinfo, cmdrsp,
					    IOCHAN_FROM_IOPART);
		spar_channel_client_release_os(dc->chinfo.queueinfo->chan,
					       "vnic");
		spin_unlock_irqrestore(&dc->chinfo.insertlock, flags);
		if (qrslt == 0)
			break;	/* queue empty */
		DBGINF("%p cmdrsp->net.type:%d\n",
		       &dc->chinfo.queueinfo, cmdrsp->net.type);
		switch (cmdrsp->net.type) {
		case NET_RCV:
			DBGINF("Got NET_RCV\n");
			dc->chstat.got_rcv++;
			/* process incoming packet */
			virtnic_rx(cmdrsp);
			break;
		case NET_XMIT_DONE:
			DBGINF("Got NET_XMIT_DONE %p\n", cmdrsp->net.buf);
			spin_lock_irqsave(&vnicinfo->priv_lock, flags);
			dc->chstat.got_xmit_done++;
			if (cmdrsp->net.xmtdone.xmt_done_result) {
				LOGERRNAME(vnicinfo->netdev,
					   "XMIT_DONE failure buf:%p\n",
					   cmdrsp->net.buf);
				dc->chstat.xmit_fail++;
			}
			/* only call queue wake if we stopped it */
			netdev = ((struct sk_buff *)cmdrsp->net.buf)->dev;
			/* ASSERT netdev == vnicinfo->netdev; */
			if (netdev != vnicinfo->netdev) {
				LOGERRNAME(vnicinfo->netdev, "NET_XMIT_DONE something wrong; vnicinfo->netdev:%p != cmdrsp->net.buf)->dev:%p\n",
					   vnicinfo->netdev, netdev);
			} else if (netif_queue_stopped(netdev)) {
				/*
				 * check to see if we have crossed
				 * the lower watermark for
				 * netif_wake_queue()
				 */
				if (((vnicinfo->datachan.chstat.sent_xmit >=
				    vnicinfo->datachan.chstat.got_xmit_done) &&
				    (vnicinfo->datachan.chstat.sent_xmit -
				    vnicinfo->datachan.chstat.got_xmit_done <=
				    vnicinfo->lower_threshold_net_xmits)) ||
				    ((vnicinfo->datachan.chstat.sent_xmit <
				    vnicinfo->datachan.chstat.got_xmit_done) &&
				    (ULONG_MAX -
				    vnicinfo->datachan.chstat.got_xmit_done
				    + vnicinfo->datachan.chstat.sent_xmit <=
				    vnicinfo->lower_threshold_net_xmits))) {
					/*
					 * enough NET_XMITs completed
					 * so can restart netif queue
					 */
					netif_wake_queue(netdev);
					vnicinfo->flow_control_lower_hits++;
				}
			}
			skb_unlink(cmdrsp->net.buf, &vnicinfo->xmitbufhead);
			spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
			kfree_skb(cmdrsp->net.buf);
			break;
		case NET_RCV_ENBDIS_ACK:
			DBGINF("Got NET_RCV_ENBDIS_ACK on:%p\n",
			       (struct net_device *)
			       cmdrsp->net.enbdis.context);
			dc->chstat.got_enbdisack++;
			netdev = (struct net_device *)
				cmdrsp->net.enbdis.context;
			spin_lock_irqsave(&vnicinfo->priv_lock, flags);
			vnicinfo->enab_dis_acked = 1;
			spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);

			if (vnicinfo->server_down &&
			    vnicinfo->server_change_state) {
				/* Inform Linux that the link is up */
				vnicinfo->server_down = false;
				vnicinfo->server_change_state = false;
				netif_wake_queue(netdev);
				netif_carrier_on(netdev);
			}
			break;
		case NET_CONNECT_STATUS:
			DBGINF("NET_CONNECT_STATUS, enable=:%d\n",
			       cmdrsp->net.enbdis.enable);
			netdev = vnicinfo->netdev;
			if (cmdrsp->net.enbdis.enable == 1) {
				spin_lock_irqsave(&vnicinfo->priv_lock, flags);
				vnicinfo->enabled = cmdrsp->net.enbdis.enable;
				spin_unlock_irqrestore(&vnicinfo->priv_lock,
						       flags);
				netif_wake_queue(netdev);
				netif_carrier_on(netdev);
			} else {
				netif_stop_queue(netdev);
				netif_carrier_off(netdev);
				spin_lock_irqsave(&vnicinfo->priv_lock, flags);
				vnicinfo->enabled = cmdrsp->net.enbdis.enable;
				spin_unlock_irqrestore(&vnicinfo->priv_lock,
						       flags);
			}
			break;
		default:
			LOGERRNAME(vnicinfo->netdev,
				   "Invalid net type:%d in cmdrsp\n",
				   cmdrsp->net.type);
			break;
		}
		/* cmdrsp is now available for reuse  */

		if (dc->chinfo.threadinfo.should_stop)
			break;
	}
}

static int
process_incoming_rsps(void *v)
{
	struct datachan *dc = v;
	struct uiscmdrsp *cmdrsp = NULL;
	const int SZ = SIZEOF_CMDRSP;
	struct virtnic_info *vnicinfo;
	struct channel_header __iomem *p_channel_header;
	struct signal_queue_header __iomem *pqhdr;
	uint64_t mask;
	unsigned long long rc1;

	UIS_DAEMONIZE("vnic_incoming");
	DBGINF("In process_incoming_rsps pid:%d queueinfo:%p threadinfo:%p\n",
	       current->pid, dc->chinfo.queueinfo, &dc->chinfo.threadinfo);
	/* alloc once and reuse */
	vnicinfo = container_of(dc, struct virtnic_info, datachan);
	cmdrsp = kmalloc(SZ, GFP_ATOMIC);
	if (cmdrsp == NULL) {
		LOGERRNAME(vnicinfo->netdev,
			   "**** FAILED to malloc - thread exiting\n");
		complete_and_exit(&dc->chinfo.threadinfo.has_stopped, 0);
	}
	p_channel_header = vnicinfo->datachan.chinfo.queueinfo->chan;
	pqhdr =
	       (struct signal_queue_header __iomem *)
	       ((char __iomem *)p_channel_header +
	       readq(&p_channel_header->ch_space_offset)) +
	       IOCHAN_FROM_IOPART;
	mask = ULTRA_CHANNEL_ENABLE_INTS;
	while (1) {
		wait_event_interruptible_timeout(
			vnicinfo->rsp_queue, (atomic_read
					      (&vnicinfo->interrupt_rcvd) == 1),
			msecs_to_jiffies(vnicinfo->thread_wait_ms));
		/*
		 * periodically check to see if there any rcv bufs which
		 * need to get sent to the iovm.   This can only happen if
		 * we run out of memory when trying to allocate skbs.
		 */
		atomic_set(&vnicinfo->interrupt_rcvd, 0);
		send_rcv_posts_if_needed(vnicinfo);
		drain_queue(dc, cmdrsp, vnicinfo);
		rc1 = uisqueue_interlocked_or((uint64_t __iomem *)
					     vnicinfo->flags_addr, mask);
		if (dc->chinfo.threadinfo.should_stop)
			break;
	}

	kfree(cmdrsp);
	DBGINF("In process_incoming_nic_rsp exiting\n");
	complete_and_exit(&dc->chinfo.threadinfo.has_stopped, 0);
}

/*****************************************************/
/* NIC support functions called external             */
/*****************************************************/

static int
virtnic_change_mtu(struct net_device *netdev, int new_mtu)
{
	LOGERRNAME(netdev, "netdev->name <<%s>>", netdev->name);
	LOGERRNAME(netdev, "**** FAILED: MTU cannot be changed at this end.\n");
	LOGERRNAME(netdev, "The same MTU is used for all the PNICs and VNICs in a switch.\n");
	LOGERRNAME(netdev, "Please change MTU from the Resource Partition\n");
	LOGERRNAME(netdev, "Current MTU is: %d\n", netdev->mtu);
	return -EINVAL;
	/*
	 * we cannot willy-nilly change the MTU; it has to come from
	 * CONTROL VM and all the vnics and pnics in a switch have to
	 * have the same MTU for everything to work.
	 */
}

/*
 * Called by kernel when ifconfig down is run.
 * Returns 0 on success, negative value on failure.
 */
static int
virtnic_close(struct net_device *netdev)
{
	/* this is called on ifconfig down but also if the device is
	 * being removed
	 */
	LOGINFNAME(netdev, "Closing %p name:%s\n", netdev, netdev->name);

	netif_stop_queue(netdev);
	virtnic_disable(netdev);

	LOGINFNAME(netdev, "Closed:%p\n", netdev);

	return 0;
}

static int
virtnic_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	return -EOPNOTSUPP;
}

/*
 * Called by kernel when ifconfig up is run.
 * Returns 0 on success, negative value on failure.
*/
static int
virtnic_open(struct net_device *netdev)
{
	struct virtnic_info *vnicinfo = netdev_priv(netdev);
	void *p = (__force void *)netdev->ip_ptr;

	LOGINFNAME(vnicinfo->netdev,
		   "Opening %p name:%s allocating:%d rcvbufs mtu:%d\n", netdev,
		   netdev->name, vnicinfo->num_rcv_bufs, netdev->mtu);

	virtnic_enable(netdev);
	/* start the interface's transmit queue, allowing it accept
	 * packets for transmission
	 */
	netif_start_queue(netdev);

	LOGINFNAME(vnicinfo->netdev,
		   "Opened %p netdev->ip_ptr:%p name:%s %02x:%02x:%02x:%02x:%02x:%02x\n",
		   netdev, netdev->ip_ptr, netdev->name, netdev->dev_addr[0],
		   netdev->dev_addr[1], netdev->dev_addr[2],
		   netdev->dev_addr[3], netdev->dev_addr[4],
		   netdev->dev_addr[5]);

	/*
	 * temporary code to see trap to catch if vnic inet addresses
	 * are getting trashed
	 */
	if (p != (__force void *)netdev->ip_ptr) {
		LOGERRNAME(vnicinfo->netdev, "***********FAILURE HAPPENED\n");
		LOGERRNAME(vnicinfo->netdev, "           Test to catch if vnic inet addresses are getting trashed.\n");
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(1000));
	}
	return 0;
}

static inline int
repost_return(
	struct uiscmdrsp *cmdrsp,
	struct virtnic_info *vnicinfo,
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
		vnicinfo->n_rcv0++;
		break;
	case 1:
		vnicinfo->n_rcv1++;
		break;
	case 2:
		vnicinfo->n_rcv2++;
		break;
	default:
		vnicinfo->n_rcvx++;
		break;
	}
	for (cc = 0, numreposted = 0; cc < copy.numrcvbufs; cc++) {
		for (i = 0; i < vnicinfo->num_rcv_bufs; i++) {
			if (vnicinfo->rcvbuf[i] != copy.rcvbuf[cc])
				continue;

			LOGVER("REPOST_RETURN: orphaning old rcvbuf[%d]:%p cc=%d",
			       i, vnicinfo->rcvbuf[i], cc);
			vnicinfo->found_repost_rcvbuf_cnt++;
			if ((skb) && vnicinfo->rcvbuf[i] == skb) {
				found_skb = 1;
				vnicinfo->repost_found_skb_cnt++;
			}
			vnicinfo->rcvbuf[i] = alloc_rcv_buf(netdev);
			if (!vnicinfo->rcvbuf[i]) {
				LOGVER("**** %s FAILED to reallocate new rcv buf - no REPOST, found_skb=%d, cc=%d, i=%d\n",
				       netdev->name, found_skb, cc, i);
				vnicinfo->num_rcv_bufs_could_not_alloc++;
				vnicinfo->alloc_failed_in_repost_return_cnt++;
				status = -1;
				break;
			}
			LOGVER("REPOST_RETURN: reposting new rcvbuf[%d]:%p\n",
			       i, vnicinfo->rcvbuf[i]);
			post_skb(cmdrsp, vnicinfo, vnicinfo->rcvbuf[i]);
			numreposted++;
			break;
		}
	}
	LOGVER("REPOST_RETURN: num rcvbufs posted:%d\n", numreposted);
	if (numreposted != copy.numrcvbufs) {
		LOGVER("**** %s FAILED to repost all the rcv bufs; numreposted:%d rcv.numrcvbufs:%d\n",
		       netdev->name, numreposted, copy.numrcvbufs);
		vnicinfo->n_repost_deficit++;
		status = -1;
	}
	if (skb) {
		if (found_skb) {
			LOGVER("REPOST_RETURN: skb is %p - freeing it", skb);
			kfree_skb(skb);
		} else {
			LOGERRNAME(vnicinfo->netdev, "%s REPOST_RETURN: skb %p NOT found in rcvbuf list!!",
				   netdev->name, skb);
			status = -3;
			vnicinfo->bad_rcv_buf++;
		}
	}
	atomic_dec(&vnicinfo->usage);
	return status;
}

static void
virtnic_rx(struct uiscmdrsp *cmdrsp)
{
	struct virtnic_info *vnicinfo;
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

	if (netdev)
		DBGINF("in virtnic_rx %p %s len:%d\n", netdev, netdev->name,
		       cmdrsp->net.rcv.rcv_done_len);
	else {
		/* We must have previously downed this network device and
		 * this skb and device is no longer valid. This also means
		 * the skb reference was removed from virtnic->rcvbuf so no
		 * need to search for it.
		 * All we can do is free the skb and return.
		 * Note: We crash if we try to log this here.
		 */
		kfree_skb(skb);
		return;
	}

	vnicinfo = netdev_priv(netdev);

	spin_lock_irqsave(&vnicinfo->priv_lock, flags);
	atomic_dec(&vnicinfo->num_rcv_bufs_in_iovm);

	/* update rcv stats - call it with priv_lock held */
	UPD_RCV_STATS;

	atomic_inc(&vnicinfo->usage);	/* don't want a close to happen before
					   we're done here */
	/*
	 * set length to how much was ACTUALLY received -
	 * NOTE: rcv_done_len includes actual length of data rcvd
	 * including ethhdr
	 */
	skb->len = cmdrsp->net.rcv.rcv_done_len;

	/* test enabled while holding lock */
	if (!(vnicinfo->enabled && vnicinfo->enab_dis_acked)) {
		/*
		 * don't process it unless we're in enable mode and until
		 * we've gotten an ACK saying the other end got our RCV enable
		 */
		LOGERRNAME(vnicinfo->netdev,
			   "%s dropping packet - perhaps old\n", netdev->name);
		spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
		if (repost_return(cmdrsp, vnicinfo, skb, netdev) < 0)
			LOGERRNAME(vnicinfo->netdev, "repost_return failed");
		return;
	}

	spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);

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
			LOGERRNAME(vnicinfo->netdev, "**** %s Something is wrong; rcv_done_len:%d > RCVPOST_BUF_SIZE:%d but numrcvbufs:%d < 2\n",
				   netdev->name, skb->len, RCVPOST_BUF_SIZE,
				   cmdrsp->net.rcv.numrcvbufs);
			if (repost_return(cmdrsp, vnicinfo, skb, netdev) < 0)
				LOGERRNAME(vnicinfo->netdev,
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
			LOGERRNAME(vnicinfo->netdev, "**** %s Something is wrong; rcv_done_len:%d <= RCVPOST_BUF_SIZE:%d but numrcvbufs:%d != 1\n",
				   netdev->name, skb->len, RCVPOST_BUF_SIZE,
				   cmdrsp->net.rcv.numrcvbufs);
			if (repost_return(cmdrsp, vnicinfo, skb, netdev) < 0)
				LOGERRNAME(vnicinfo->netdev,
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
		LOGERRNAME(vnicinfo->netdev, "**** %s Something is wrong; rcvbuf[0]:%p != skb:%p\n",
			   netdev->name, cmdrsp->net.rcv.rcvbuf[0], skb);
		if (repost_return(cmdrsp, vnicinfo, skb, netdev) < 0)
			LOGERRNAME(vnicinfo->netdev, "repost_return failed");
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
			if (prev == NULL)	/* start of list- set head */
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
			LOGERRNAME(vnicinfo->netdev, "%s something wrong; skb->len:%d != off:%d\n",
				   netdev->name, skb->len, off);
		}
		/* test code */
		cc = util_copy_fragsinfo_from_skb("rcvchaintest", skb,
						  RCVPOST_BUF_SIZE,
						  MAX_PHYS_INFO, testfrags);
		LOGINFNAME(vnicinfo->netdev, "rcvchaintest returned:%d\n", cc);
		if (cc != cmdrsp->net.rcv.numrcvbufs) {
			LOGERRNAME(vnicinfo->netdev, "**** %s Something wrong; rcvd chain length %d different from one we calculated %d\n",
				   netdev->name, cmdrsp->net.rcv.numrcvbufs,
				   cc);
		}
		for (i = 0; i < cc; i++) {
			LOGINFNAME(vnicinfo->netdev, "test:RCVPOST_BUF_SIZE:%d[%d] pfn:%llu off:0x%x len:%d\n",
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
			LOGERRNAME(vnicinfo->netdev, "**** FAILED to deliver rcv packet to OS; name:%s Dest:%02x:%02x:%02x:%02x:%02x:%02x VNIC:%02x:%02x:%02x:%02x:%02x:%02x\n",
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
		vnicinfo->n_rcv_packet_not_accepted++;
		if (repost_return(cmdrsp, vnicinfo, skb, netdev) < 0)
			LOGERRNAME(vnicinfo->netdev, "repost_return failed");
		return;
	} while (0);

	DBGINF("Calling netif_rx skb:%p head:%p end:%p data:%p tail:%p len:%d data_len:%d skb->nr_frags:%d\n",
	       skb, skb->head, skb->end, skb->data, skb->tail, skb->len,
	       skb->data_len, skb_shinfo(skb)->nr_frags);

	status = netif_rx(skb);
	if (status != NET_RX_SUCCESS)
		LOGWRNNAME(vnicinfo->netdev, "status=%d\n", status);
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
	if (repost_return(cmdrsp, vnicinfo, skb, netdev) < 0)
		LOGVER("repost_return failed");
	return;
}

/*
 * This function is protected from concurrent calls by a spinlock xmit_lock
 * in the  net_device struct, but as soon as the function returns it can be
 * called again.
 * Return 0, OK, !0 for error.
 */
static int
virtnic_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct virtnic_info *vnicinfo;
	int len, firstfraglen, padlen;
	struct uiscmdrsp *cmdrsp = NULL;
	unsigned long flags;
	int qrslt;

/* Note: NETDEV_TX_OK is 0, NETDEV_TX_BUSY is 1. */
#define BUSY { \
	spin_unlock_irqrestore(&vnicinfo->priv_lock, flags); \
	vnicinfo->busy_cnt++; \
	return NETDEV_TX_BUSY; \
}

/* return value NETDEV_TX_OK == 0 */
	DBGINF("got xmit for netdev:%p %s len:%d ip_summed:%d skb->data:%p data_len:%d skb->h.raw:%p maxdatalen:%d\n",
	       netdev, netdev->name, skb->len, skb->ip_summed, skb->data,
	       skb->data_len, skb->h.raw, skb->end - skb->data);

	vnicinfo = netdev_priv(netdev);
	spin_lock_irqsave(&vnicinfo->priv_lock, flags);
	/*Modified for Trac #2395 FIX TEL_CKS */
	if (netif_queue_stopped(netdev)) {
		LOGINFNAME(vnicinfo->netdev,
			   "Returning Busy because queue is stopped\n");
		BUSY;
	}
	if (vnicinfo->server_down || vnicinfo->server_change_state) {
		LOGINFNAME(vnicinfo->netdev, "Returning BUSY because server is down/changing state\n");
		BUSY;
	}
	/*
	 * sk_buff struct is used to host network data throughout all the
	 * Linux network subsystems
	 */
	len = skb->len;
	/*
	 * skb->len is the FULL length of data (including fragmentary portion)
	 * skb->data_len is the length of the fragment portion in frags
	 * skb->len - skb->data_len is the size of the 1st fragment in skb->data
	 * calculate the length of the first fragment that skb->data is
	 * pointing to
	 */
	firstfraglen = skb->len - skb->data_len;
	if (firstfraglen < ETH_HEADER_SIZE) {
		LOGERRNAME(vnicinfo->netdev, "first fragment in skb->data too small for ethernet header len:%d data_len:%d\n",
			   skb->len, skb->data_len);
		BUSY;		/* NOT LIKELY TO HAPPEN */
	}

	if ((len < ETH_MIN_PACKET_SIZE) &&
	    ((skb_end_pointer(skb) - skb->data) >= ETH_MIN_PACKET_SIZE)) {
		/* pad the packet out to minimum size */
		padlen = ETH_MIN_PACKET_SIZE - len;
		DBGINF("padding %d\n", padlen);
		memset(&skb->data[len], 0, padlen);
		skb->tail += padlen;
		skb->len += padlen;
		len += padlen;
		firstfraglen += padlen;
	}

	cmdrsp = vnicinfo->xmit_cmdrsp;
	/* clear cmdrsp */
	memset(cmdrsp, 0, SIZEOF_CMDRSP);
	cmdrsp->net.type = NET_XMIT;
	cmdrsp->cmdtype = CMD_NET_TYPE;

	/* save the pointer to skb - we'll need it for completion */
	cmdrsp->net.buf = skb;

	if (((vnicinfo->datachan.chstat.sent_xmit >=
	      vnicinfo->datachan.chstat.got_xmit_done) &&
	     (vnicinfo->datachan.chstat.sent_xmit -
	     vnicinfo->datachan.chstat.got_xmit_done >=
	     vnicinfo->max_outstanding_net_xmits)) ||
	    /* OR check wrap condition */
	    ((vnicinfo->datachan.chstat.sent_xmit <
	      vnicinfo->datachan.chstat.got_xmit_done) &&
	      (ULONG_MAX - vnicinfo->datachan.chstat.got_xmit_done +
	       vnicinfo->datachan.chstat.sent_xmit >=
	       vnicinfo->max_outstanding_net_xmits))
	    ) {
		/*
		 * too many NET_XMITs queued over to IOVM - need to wait
		 * Might need to remove the below message as these might be
		 * excessive under load.
		 */
		vnicinfo->datachan.chstat.reject_count++;
		if (!vnicinfo->queuefullmsg_logged &&
		    ((vnicinfo->datachan.chstat.reject_count & 0x3ff) ==
			1)) {
			vnicinfo->queuefullmsg_logged = 1;
#if VIRTNIC_STATS
			vnicinfo->datachan.chstat.reject_jiffies_start =
			    jiffies;
#endif
			LOGINFNAME(vnicinfo->netdev, "**** REJECTING NET_XMIT - rejected count=%ld chstat.sent_xmit=%lu chstat.got_xmit_done=%lu\n",
				   vnicinfo->datachan.chstat.reject_count,
				   vnicinfo->datachan.chstat.sent_xmit,
				   vnicinfo->datachan.chstat.got_xmit_done);
		}
		netif_stop_queue(netdev);	/* calling stop queue */
		BUSY;		/* return status that packet not accepted */
	} else if (vnicinfo->queuefullmsg_logged) {
#if VIRTNIC_STATS
		LOGINFNAME(vnicinfo->netdev, "**** NET_XMITs now working again - rejected count = %ld msec = %ld\n",
			   vnicinfo->datachan.chstat.reject_count,
			   ((long)jiffies -
			   (long)(vnicinfo->datachan.chstat.
				    reject_jiffies_start)) * 1000 / HZ);
#else
		LOGINFNAME(vnicinfo->netdev, "**** NET_XMITs now working again - rejected count = %ld\n",
			   vnicinfo->datachan.chstat.reject_count);
#endif
		/* queue is not blocked so reset the logging flag */
		vnicinfo->queuefullmsg_logged = 0;
	}

	if (skb->ip_summed == CHECKSUM_UNNECESSARY) {
		DBGINF("CHECKSUM_HW protocol:%x csum:%x tso_size:%x data:%p h.raw:%p nh.raw:%p\n",
		       skb->protocol, skb->csum, skb_shinfo(skb)->tso_size,
		       skb->data, skb->h.raw, skb->nh.raw);
		cmdrsp->net.xmt.lincsum.valid = 1;
		cmdrsp->net.xmt.lincsum.protocol = skb->protocol;
		if (skb_transport_header(skb) > skb->data) {
			cmdrsp->net.xmt.lincsum.hrawoff =
				skb_transport_header(skb) - skb->data;
			cmdrsp->net.xmt.lincsum.hrawoffv = 1;
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
	/* save off the length of the entire data packet  */
	 cmdrsp->net.xmt.len = len;	/* total data length */
	/*
	 * copy ethernet header from first frag into cmdrsp
	 * - everything else will be passed in frags & DMA'ed
	 */
	memcpy(cmdrsp->net.xmt.ethhdr, skb->data, ETH_HEADER_SIZE);
	/*
	 * copy frags info - from skb->data we need to only provide access
	 * beyond eth header
	 */
	cmdrsp->net.xmt.num_frags =
	    uisutil_copy_fragsinfo_from_skb("virtnic_xmit", skb, firstfraglen,
					    MAX_PHYS_INFO,
					    cmdrsp->net.xmt.frags);
	if (cmdrsp->net.xmt.num_frags == -1) {
		LOGERRNAME(vnicinfo->netdev, "**** FAILED to copy fragsinfo\n");
		BUSY;		/* WILL HAPPEN ONLY IF FRAG ARRAY WITH
				   MAX_PHYS_INFO ENTRIES IS NOT ENOUGH */
	}

	DBGINF("Forwarding packet cmdrsp:%p\n", cmdrsp);

	/*
	 * don't hold lock when forwarding xmit - if queue is full insert
	 * might sleep
	 */
	qrslt = uisqueue_put_cmdrsp_with_lock_client(
			vnicinfo->datachan.chinfo.queueinfo, cmdrsp,
			IOCHAN_TO_IOPART,
			(void *)&vnicinfo->datachan.chinfo.insertlock,
			DONT_ISSUE_INTERRUPT, (uint64_t)NULL,
			0 /* don't wait */ ,
			"vnic");
	if (!qrslt) {
		/* failed to queue xmit - return busy */
		LOGERRNAME(vnicinfo->netdev,
			   "**** FAILED to insert NET_XMIT\n");
		netif_stop_queue(netdev);	/* calling stop queue  */
		BUSY;		/* return status that packet not accepted */
	}
	/* Track the skbs that have been sent to the IOVM for XMIT */
	skb_queue_head(&vnicinfo->xmitbufhead, skb);

	/*
	 * set the last transmission start time
	 * linux docs says:  Do not forget to update netdev->trans_start to
	 * jiffies after each new tx packet is given to the hardware.
	 */
	netdev->trans_start = jiffies;	/* some code in Linux uses this. */

	/* update xmt stats */
	UPD_XMT_STATS;
	vnicinfo->datachan.chstat.sent_xmit++;

	/*
	 * check to see if we have hit the high watermark for
	 * netif_stop_queue()
	 */
	if (((vnicinfo->datachan.chstat.sent_xmit >=
	      vnicinfo->datachan.chstat.got_xmit_done) &&
	     (vnicinfo->datachan.chstat.sent_xmit -
	      vnicinfo->datachan.chstat.got_xmit_done >=
	      vnicinfo->upper_threshold_net_xmits)) ||
	    /* OR check wrap condition */
	    ((vnicinfo->datachan.chstat.sent_xmit <
	      vnicinfo->datachan.chstat.got_xmit_done) &&
	      (ULONG_MAX - vnicinfo->datachan.chstat.got_xmit_done +
	       vnicinfo->datachan.chstat.sent_xmit >=
	       vnicinfo->upper_threshold_net_xmits))
	   ) {
		/* too many NET_XMITs queued over to IOVM - need to wait */
		netif_stop_queue(netdev); /* calling stop queue - call
					     netif_wake_queue() after lower
					     threshold */
		vnicinfo->flow_control_upper_hits++;
	}

	spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);

	/* skb will be freed when we get back NET_XMIT_DONE */
	return NETDEV_TX_OK;
}

static void
virtnic_serverdown_complete(struct work_struct *work)
{
	struct virtnic_info *vnicinfo;
	struct net_device *netdev;
	struct virtpci_dev *virtpcidev;
	unsigned long flags;
	int i = 0, count = 0;

	vnicinfo =
	    container_of(work, struct virtnic_info, serverdown_completion);
	netdev = vnicinfo->netdev;
	virtpcidev = vnicinfo->virtpcidev;

	DBGINF("virtpcidev busNo<<%d>>devNo<<%d>>", virtpcidev->busNo,
	       virtpcidev->deviceNo);
	DBGINF("net_device name<<%s>>", netdev->name);
	/* Stop Using Datachan */
	uisthread_stop(&vnicinfo->datachan.chinfo.threadinfo);

	/* Inform Linux that the link is down */
	netif_carrier_off(netdev);
	netif_stop_queue(netdev);

	/*
	 * Free the skb for XMITs that haven't been serviced by the server
	 * We shouldn't have to inform Linux about these IOs because they
	 * are "lost in the ethernet"
	 */
	skb_queue_purge(&vnicinfo->xmitbufhead);

	spin_lock_irqsave(&vnicinfo->priv_lock, flags);
	/* free rcv buffers */
	for (i = 0; i < vnicinfo->num_rcv_bufs; i++) {
		if (vnicinfo->rcvbuf[i]) {
			kfree_skb(vnicinfo->rcvbuf[i]);
			vnicinfo->rcvbuf[i] = NULL;
			count++;
		}
	}
	atomic_set(&vnicinfo->num_rcv_bufs_in_iovm, 0);
	spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);

	LOGINFNAME(vnicinfo->netdev, "Closed:%p Freed %d rcv bufs\n", netdev,
		   count);

	vnicinfo->server_down = true;
	vnicinfo->server_change_state = false;
	visorchipset_device_pause_response(virtpcidev->busNo,
					   virtpcidev->deviceNo, 0);
}

/* As per VirtpciFunc returns 1 for success and 0 for failure */
static int
virtnic_serverdown(struct virtpci_dev *virtpcidev, u32 state)
{
	struct net_device *netdev = virtpcidev->net.netdev;
	struct virtnic_info *vnicinfo = netdev_priv(netdev);

	DBGINF("virtpcidev busNo<<%d>>devNo<<%d>>", virtpcidev->busNo,
	       virtpcidev->deviceNo);
	DBGINF("entering virtnic_serverdown");

	if (!vnicinfo->server_down && !vnicinfo->server_change_state) {
		vnicinfo->server_change_state = true;
		queue_work(virtnic_serverdown_workqueue,
			   &vnicinfo->serverdown_completion);
	} else if (vnicinfo->server_change_state) {
		LOGERRNAME(vnicinfo->netdev,
			   "Server already processing change state message.");
		return 0;
	} else
		LOGERRNAME(vnicinfo->netdev,
			   "Server already down, but another server down message received.");
	DBGINF("exiting virtnic_serverdown");
	return 1;
}

/* As per VirtpciFunc returns 1 for success and 0 for failure */
static int
virtnic_serverup(struct virtpci_dev *virtpcidev)
{
	struct net_device *netdev = virtpcidev->net.netdev;
	struct virtnic_info *vnicinfo = netdev_priv(netdev);
	unsigned long flags;

	DBGINF("entering virtnic_serverup");
	DBGINF("virtpcidev busNo<<%d>>devNo<<%d>>", virtpcidev->busNo,
	       virtpcidev->deviceNo);
	DBGINF("net_device name<<%s>>", netdev->name);
	if (vnicinfo->server_down && !vnicinfo->server_change_state) {
		vnicinfo->server_change_state = true;
		/*
		 * Must transition channel to ATTACHED state BEFORE we can
		 * start using the device again
		 */
		SPAR_CHANNEL_CLIENT_TRANSITION(vnicinfo->datachan.chinfo.
					       queueinfo->chan,
					       dev_name(&virtpcidev->
							generic_dev),
					       CHANNELCLI_ATTACHED, NULL);

		if (!uisthread_start(&vnicinfo->datachan.chinfo.threadinfo,
				     process_incoming_rsps,
				     &vnicinfo->datachan, "vnic_incoming")) {
			LOGERRNAME(vnicinfo->netdev,
				   "**** FAILED to start thread\n");
			return 0;
		}

		init_rcv_bufs(netdev, vnicinfo);

		spin_lock_irqsave(&vnicinfo->priv_lock, flags);
		vnicinfo->enabled = 1;
		/*
		 * now we're ready, let's send an ENB to uisnic
		 * but until we get an ACK back from uisnic, we'll drop
		 * the packets
		 */
		vnicinfo->enab_dis_acked = 0;
		spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);

		/*
		 * send enable and wait for ack - don't hold lock when
		 * sending enable because if the queue is full, insert
		 * might sleep.
		 */
		SEND_ENBDIS(netdev, 1, vnicinfo->cmdrsp_rcv,
			    vnicinfo->datachan.chinfo.queueinfo,
			    &vnicinfo->datachan.chinfo.insertlock,
			    vnicinfo->datachan.chstat);
	} else if (vnicinfo->server_change_state) {
		LOGERRNAME(vnicinfo->netdev,
			   "Server already processing change state message.");
		return 0;
	} else {
		DBGINF("Server up message recieved for server that was already up.");
	}
	DBGINF("exiting virtnic_serverup");
	return 1;
}

static void
virtnic_timeout_reset(struct work_struct *work)
{
	struct virtnic_info *vnicinfo;
	struct net_device *netdev;
	struct virtpci_dev *virtpcidev;
	int response = 0;

	vnicinfo = container_of(work, struct virtnic_info, timeout_reset);
	netdev = vnicinfo->netdev;

	DBGINF("net_device name<<%s>>", netdev->name);
	/* Transmit Timeouts are typically handled by resetting the
	 * device for our virtual NIC we will send a Disable and
	 * Enable to the IOVM.  If it doesn't respond we will trigger
	 * a serverdown
	 */
	DBGINF("Disabling connection to server.\n");
	netif_stop_queue(netdev);
	response = virtnic_disable_with_timeout(netdev, 100);
	if (response != 0)
		goto call_serverdown;

	DBGINF("Disable returned so reenable connection to server.\n");
	response = virtnic_enable_with_timeout(netdev, 100);
	if (response != 0)
		goto call_serverdown;
	netif_wake_queue(netdev);

	LOGWRNNAME(vnicinfo->netdev, "Virtual connection reset.\n");
	return;

call_serverdown:
	LOGERRNAME(vnicinfo->netdev,
		   "Disable/enabled Pair failed to return so start serverdown.\n");
	virtpcidev = vnicinfo->virtpcidev;
	virtnic_serverdown(virtpcidev, 0);
	return;
}

static void
virtnic_xmit_timeout(struct net_device *netdev)
{
	struct virtnic_info *vnicinfo = netdev_priv(netdev);
	unsigned long flags;

	LOGWRNNAME(vnicinfo->netdev,
		   "Transmit Timeout.  Resetting virtual connection.\n");
	LOGWRNNAME(vnicinfo->netdev, "net_device name<<%s>>", netdev->name);

	spin_lock_irqsave(&vnicinfo->priv_lock, flags);
	/* Ensure that a ServerDown message hasn't been received */
	if (!vnicinfo->enabled ||
	    (vnicinfo->server_down && !vnicinfo->server_change_state)) {
		spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);
		return;
	}
	spin_unlock_irqrestore(&vnicinfo->priv_lock, flags);

	queue_work(virtnic_timeout_reset_workqueue, &vnicinfo->timeout_reset);
}

static void
virtnic_set_multi(struct net_device *netdev)
{
	struct uiscmdrsp *cmdrsp;
	struct virtnic_info *vnicinfo = netdev_priv(netdev);

	DBGINF("net_device name<<%s>>", netdev->name);
	DBGINF("entering virtnic_set_multi\n");

	/* any filtering changes? */
	if (vnicinfo->old_flags != netdev->flags) {
		LOGINFNAME(vnicinfo->netdev,
			   "old filter = 0x%04x, new filter = 0x%04x.\n",
			   vnicinfo->old_flags, netdev->flags);
		if ((netdev->flags & IFF_PROMISC) !=
		    (vnicinfo->old_flags & IFF_PROMISC)) {
			LOGINFNAME(vnicinfo->netdev,
				   "we are %s promiscuous mode.\n",
				   (netdev->
				    flags & IFF_PROMISC) ? "entering" :
				   "exiting");
			cmdrsp = kmalloc(SIZEOF_CMDRSP, GFP_ATOMIC);
			if (cmdrsp == NULL) {
				LOGERRNAME(vnicinfo->netdev,
					   "**** FAILED to kmalloc cmdrsp.\n");
				return;
			}
			memset(cmdrsp, 0, SIZEOF_CMDRSP);
			cmdrsp->cmdtype = CMD_NET_TYPE;
			cmdrsp->net.type = NET_RCV_PROMISC;
			cmdrsp->net.enbdis.context = netdev;
			cmdrsp->net.enbdis.enable =
			    (netdev->flags & IFF_PROMISC);
			if (uisqueue_put_cmdrsp_with_lock_client
			    (vnicinfo->datachan.chinfo.queueinfo, cmdrsp,
			     IOCHAN_TO_IOPART,
			     (void *)&vnicinfo->datachan.chinfo.insertlock,
			     DONT_ISSUE_INTERRUPT, (uint64_t)NULL,
			     0 /* don't wait */ , "vnic")) {
				vnicinfo->datachan.chstat.sent_promisc++;
			} else
				LOGERRNAME(vnicinfo->netdev,
					   "**** FAILED to insert NET_RCV_PROMISC.\n");
			kfree(cmdrsp);
		}

		vnicinfo->old_flags = netdev->flags;
	}
	DBGINF("exiting virtnic_set_multi\n");
}

/*****************************************************/
/* debugfs filesystem functions			     */
/*****************************************************/

static ssize_t info_debugfs_read(struct file *file,
				 char __user *buf, size_t len, loff_t *offset)
{
	int i;
	ssize_t bytes_read = 0;
	int str_pos = 0;
	struct virtnic_info *vni;
	char *vbuf;

	if (len > MAX_BUF)
		len = MAX_BUF;
	vbuf = kzalloc(len, GFP_KERNEL);
	if (!vbuf)
		return -ENOMEM;

	/* for each vnic channel
	 * dump out channel specific data
	 */
	for (i = 0; i < VIRTNICSOPENMAX; i++) {
		if (num_virtnic_open[i].netdev == NULL)
			continue;

		vni = num_virtnic_open[i].vnicinfo;
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, "Vnic i = %d\n", i);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, "netdev = %s (0x%p), MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x\n",
			num_virtnic_open[i].netdev->name,
			num_virtnic_open[i].netdev,
			num_virtnic_open[i].netdev->dev_addr[0],
			num_virtnic_open[i].netdev->dev_addr[1],
			num_virtnic_open[i].netdev->dev_addr[2],
			num_virtnic_open[i].netdev->dev_addr[3],
			num_virtnic_open[i].netdev->dev_addr[4],
			num_virtnic_open[i].netdev->dev_addr[5]);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, "vnicinfo = 0x%p\n", vni);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " num_rcv_bufs = %d\n",
			vni->num_rcv_bufs);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " features = 0x%016llX\n",
			(uint64_t)readq(&vni->datachan.chinfo.queueinfo->chan->
				features));
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " max_outstanding_net_xmits = %d\n",
			vni->max_outstanding_net_xmits);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " upper_threshold_net_xmits = %d\n",
			vni->upper_threshold_net_xmits);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " lower_threshold_net_xmits = %d\n",
			vni->lower_threshold_net_xmits);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " queuefullmsg_logged = %d\n",
			vni->queuefullmsg_logged);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " queueinfo->packets_sent = %lld\n",
			vni->datachan.chinfo.queueinfo->packets_sent);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " queueinfo->packets_received = %lld\n",
			vni->datachan.chinfo.queueinfo->packets_received);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " chstat.got_rcv = %lu\n",
			vni->datachan.chstat.got_rcv);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " chstat.got_enbdisack = %lu\n",
			vni->datachan.chstat.got_enbdisack);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " chstat.got_xmit_done = %lu\n",
			vni->datachan.chstat.got_xmit_done);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " chstat.xmit_fail = %lu\n",
			vni->datachan.chstat.xmit_fail);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " chstat.sent_enbdis = %lu\n",
			vni->datachan.chstat.sent_enbdis);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " chstat.sent_promisc = %lu\n",
			vni->datachan.chstat.sent_promisc);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " chstat.sent_post = %lu\n",
			vni->datachan.chstat.sent_post);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " chstat.sent_xmit = %lu\n",
			vni->datachan.chstat.sent_xmit);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " chstat.reject_count = %lu\n",
			vni->datachan.chstat.reject_count);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " chstat.extra_rcvbufs_sent = %lu\n",
			vni->datachan.chstat.extra_rcvbufs_sent);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " n_rcv0 = %lu\n", vni->n_rcv0);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " n_rcv1 = %lu\n", vni->n_rcv1);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " n_rcv2 = %lu\n", vni->n_rcv2);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " n_rcvx = %lu\n", vni->n_rcvx);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " num_rcv_bufs_in_iovm = %d\n",
			atomic_read(&vni->num_rcv_bufs_in_iovm));
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " alloc_failed_in_if_needed_cnt = %lu\n",
			vni->alloc_failed_in_if_needed_cnt);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " alloc_failed_in_repost_return_cnt = %lu\n",
			vni->alloc_failed_in_repost_return_cnt);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " inner_loop_limit_reached_cnt = %lu\n",
			vni->inner_loop_limit_reached_cnt);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " found_repost_rcvbuf_cnt = %lu\n",
			vni->found_repost_rcvbuf_cnt);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " repost_found_skb_cnt = %lu\n",
			vni->repost_found_skb_cnt);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " n_repost_deficit = %lu\n",
			vni->n_repost_deficit);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " bad_rcv_buf = %lu\n",
			vni->bad_rcv_buf);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " n_rcv_packet_not_accepted = %lu\n",
			vni->n_rcv_packet_not_accepted);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " interrupts_rcvd = %llu\n",
			vni->interrupts_rcvd);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " interrupts_notme = %llu\n",
			vni->interrupts_notme);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " interrupts_disabled = %llu\n",
			vni->interrupts_disabled);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " busy_cnt = %llu\n",
			vni->busy_cnt);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " flow_control_upper_hits = %llu\n",
			vni->flow_control_upper_hits);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " flow_control_lower_hits = %llu\n",
			vni->flow_control_lower_hits);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " thread_wait_ms = %d\n",
			vni->thread_wait_ms);
		str_pos += scnprintf(vbuf + str_pos,
				len - str_pos, " netif_queue = %s\n",
			netif_queue_stopped(vni->netdev) ?
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
	struct virtnic_info *vnicinfo;
	uint64_t __iomem *features_addr;
	uint64_t mask;

	if (count >= ARRAY_SIZE(buf))
		return -EINVAL;

	buf[count] = '\0';
	if (copy_from_user(buf, buffer, count)) {
		LOGERR("copy_from_user failed.\n");
		return -EFAULT;
	}

	i = kstrtoint(buf, 10 , &new_value);

	if (i != 0) {
		LOGERR("Failed to scan value for enable_ints, buf<<%.*s>>",
		       (int)count, buf);
		return -EFAULT;
	}

	 /* set all counts to new_value usually 0 */
	for (i = 0; i < VIRTNICSOPENMAX; i++) {
		if (num_virtnic_open[i].vnicinfo != NULL) {
			vnicinfo = num_virtnic_open[i].vnicinfo;
			features_addr =
				&vnicinfo->datachan.chinfo.queueinfo->chan->
				features;
			if (new_value == 1) {
				mask =
				    ~(ULTRA_IO_CHANNEL_IS_POLLING |
				      ULTRA_IO_DRIVER_DISABLES_INTS);
				uisqueue_interlocked_and(features_addr, mask);
				mask = ULTRA_IO_DRIVER_ENABLES_INTS;
				uisqueue_interlocked_or(features_addr, mask);
				vnicinfo->thread_wait_ms = 2000;
			} else {
				mask =
					~(ULTRA_IO_DRIVER_ENABLES_INTS |
					ULTRA_IO_DRIVER_DISABLES_INTS);
				uisqueue_interlocked_and(features_addr, mask);
				mask = ULTRA_IO_CHANNEL_IS_POLLING;
				uisqueue_interlocked_or(features_addr, mask);
				vnicinfo->thread_wait_ms = 2;
			}
		}
}

return count;
}

/*****************************************************/
/* Module init & exit functions                      */
/*****************************************************/

static int __init
virtnic_mod_init(void)
{
	int error, i;

	LOGINF("entering virtnic_mod_init");
	/* ASSERT RCVPOST_BUF_SIZE < 4K */
	if (RCVPOST_BUF_SIZE > PI_PAGE_SIZE) {
		LOGERR("**** FAILED RCVPOST_BUF_SIZE:%d larger than a page\n",
		       RCVPOST_BUF_SIZE);
		return -1;
	}
	/* ASSERT RCVPOST_BUF_SIZE is big enough to hold eth header */
	if (RCVPOST_BUF_SIZE < ETH_HEADER_SIZE) {
		LOGERR("**** FAILED RCVPOST_BUF_SIZE:%d is < ETH_HEADER_SIZE:%d\n",
		       RCVPOST_BUF_SIZE, ETH_HEADER_SIZE);
		return -1;
	}

	/* clear out array */
	for (i = 0; i < VIRTNICSOPENMAX; i++) {
		num_virtnic_open[i].netdev = NULL;
		num_virtnic_open[i].vnicinfo = NULL;
	}
	/* create workqueue for serverdown completion */
	virtnic_serverdown_workqueue =
	    create_singlethread_workqueue("virtnic_serverdown");
	if (virtnic_serverdown_workqueue == NULL) {
		LOGERR("**** FAILED virtnic_serverdown_workqueue creation\n");
		return -1;
	}
	/* create workqueue for tx timeout reset  */
	virtnic_timeout_reset_workqueue =
	    create_singlethread_workqueue("virtnic_timeout_reset");
	if (virtnic_timeout_reset_workqueue == NULL) {
		LOGERR
		    ("**** FAILED virtnic_timeout_reset_workqueue creation\n");
		return -1;
	}
	virtnic_debugfs_dir = debugfs_create_dir("virtnic", NULL);
	debugfs_create_file("info", S_IRUSR, virtnic_debugfs_dir,
			    NULL, &debugfs_info_fops);
	debugfs_create_file("enable_ints", S_IWUSR,
			    virtnic_debugfs_dir, NULL,
			    &debugfs_enable_ints_fops);

	error = virtpci_register_driver(&virtnic_driver);
	if (error < 0) {
		LOGERR("**** FAILED to register driver %x\n", error);
		debugfs_remove_recursive(virtnic_debugfs_dir);
		return -1;
	}
	LOGINF("exiting virtnic_mod_init");
	return error;
}

static void __exit
virtnic_mod_exit(void)
{
	LOGINF("entering virtnic_mod_exit...\n");
	virtpci_unregister_driver(&virtnic_driver);
	/* unregister is going to call virtnic_remove for all devices */
	/* destroy serverdown completion workqueue */
	if (virtnic_serverdown_workqueue) {
		destroy_workqueue(virtnic_serverdown_workqueue);
		virtnic_serverdown_workqueue = NULL;
	}

	/* destroy timeout reset workqueue */
	if (virtnic_timeout_reset_workqueue) {
		destroy_workqueue(virtnic_timeout_reset_workqueue);
		virtnic_timeout_reset_workqueue = NULL;
	}

	debugfs_remove_recursive(virtnic_debugfs_dir);
	LOGINF("exiting virtnic_mod_exit...\n");
}

module_init(virtnic_mod_init);
module_exit(virtnic_mod_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Usha Srinivasan");
MODULE_ALIAS("uisvirtnic");
/* this is extracted during depmod and kept in modules.dep */
