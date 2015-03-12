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

#define VISORNIC_STATS 0
#define VISORNIC_XMIT_TIMEOUT (5 * HZ)
#define VISORNIC_INFINITE_RESPONSE_WAIT 0
#define INTERRUPT_VECTOR_MASK 0x3F

static spinlock_t dev_no_pool_lock;
static void *dev_no_pool;	/**< pool to grab device numbers from */

static int visornic_probe(struct visor_device *dev);
static void visornic_remove(struct visor_device *dev);
static int visornic_pause(struct visor_device *dev,
			  VISORBUS_STATE_COMPLETE_FUNC complete_func);
static int visornic_resume(struct visor_device *dev,
			   VISORBUS_STATE_COMPLETE_FUNC complete_func);
/** DEBUGFS declarations
 */
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

/**  GUIDS for director channel type supported by this driver.
*/
static struct visor_channeltype_descriptor visornic_channel_types[] = {
	/*  Note that the only channel type we expect to be reported by the
	 *  bus driver is the ULTRAVNIC channel.
	 */
	{ SPAR_VNIC_CHANNEL_PROTOCOL_UUID,
	  "ultravnic", 1, ULONG_MAX },
	{ NULL_UUID_LE, NULL, 0, 0 }
};

/** This is used to tell the visor bus driver which types of visor devices
 *  we support, and what functions to call when a visor device that we support
 *  is attached or removed.
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

/** This is the private data that we store for each device.
 *  A pointer to this struct is kept in each "struct device", and can be
 *  obtained using visor_get_drvdata(dev).
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
	struct completion has_stopped;  // REMOVE
	int should_stop; // REMOVE
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
					   IOPART */
	struct visor_device *dev;
	struct visorchipset_device_info *dev_chipset; /* IRQ Information */
	/** lock for dev */
	struct rw_semaphore lock_visor_dev;
	char name[99];
	struct list_head list_all;   /**< link within list_all_devices list */
	struct kref kref;
	struct net_device *netdev;
	atomic_t interrupt_rcvd;
	wait_queue_head_t rsp_queue;
	struct sk_buff **rcvbuf;
	unsigned long long uniquenum;

	int num_rcv_bufs;		 /* indicates how many rcv buffers
					    the vnic will post */
	int num_rcv_bufs_could_not_alloc;
	atomic_t num_rcv_bufs_in_iovm;
	unsigned long alloc_failed_in_if_needed_cnt;
	unsigned long alloc_failed_in_repost_return_cnt;
	int max_outstanding_net_xmits;   /* absolute max number of outstanding
					    xmits - should never hit this */
	int upper_threshold_net_xmits;   /* high water mark for calling
					    netif_stop_queue() */
	int lower_threshold_net_xmits;	 /* high water mark for calling
					    netif_wake_queue() */
	struct sk_buff_head xmitbufhead; /* xmitbufhead is the head of the
					    xmit buffer list that have been
					    sent to the IOPART end */
	struct work_struct serverdown_completion;
	struct work_struct timeout_reset;
	struct uiscmdrsp *cmdrsp_rcv;	 /* cmdrsp_rcv is used for
					    posting/unposting rcv buffers */
	bool server_down;		 /* IOPART is down */
	bool server_change_state;	 /* Processing SERVER_CHANGESTATE msg */
	struct dentry *eth_debugfs_dir;
	struct visor_thread_info threadinfo;
	unsigned long long interrupts_rcvd;
	unsigned long long interrupts_notme; 
	unsigned long long interrupts_disabled;
	unsigned long long busy_cnt;
	spinlock_t insertlock;
	spinlock_t priv_lock;
};

int visor_thread_start(struct visor_thread_info *thrinfo, 
		    int (*threadfn)(void *), 
		    void *thrcontext, char *name) 
{
	return 0;
}
void visor_thread_stop(struct visor_thread_info *thrinfo) 
{
}
/** DebugFS code
 */
static ssize_t info_debugfs_read(struct file *file, char __user *buf,
				 size_t len, loff_t *offset)
{
	/* DO NOTHING FOR NOW */
	return len;
}

static ssize_t enable_ints_write(struct file *file, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	/* DO NOTHING FOR NOW */
	return len;
}

static void
visornic_timeout_reset(struct work_struct *work)
{
	/* DO NOTHING FOR NOW */
}

static void
visornic_serverdown_complete(struct work_struct *work)
{
	/* DO NOTHING FOR NOW */
}

/** List of all visornic_devdata structs,
  * linked via the list_all member
  */
static LIST_HEAD(list_all_devices);
static DEFINE_SPINLOCK(lock_all_devices);

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
	/* .ndo_open = visornic_open,
	.ndo_close = visornic_close,
	.ndo_start_xmit = visornic_xmit,
	.ndo_get_stats = visornic_get_stats,
	.ndo_do_ioctl = visornic_ioctl,
	.ndo_change_mtu = visornic_change_mtu,
	.ndo_tx_timeout = visornic_xmit_timeout,
	.ndo_set_rx_mode = visornic_set_multi, */
};

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
                atomic_inc(&devdata->num_rcv_bufs_in_iovm);
                /* TODO vnicinfo->datachan.chstat.sent_post++;
		 */
        }
}

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
			} else {
				rcv_bufs_allocated++;
				post_skb(cmdrsp, devdata,
					 devdata->rcvbuf[i]);
				/* TODO: 
				 * Update datachan chanstat extra_rcvbufs_sent++
				 */
			}	
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
        int qrslt;
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
                	/* TODO:        dc->chstat.got_rcv++; */
                        /* process incoming packet */
                        //virtnic_rx(cmdrsp);
                        break;
                case NET_XMIT_DONE:
                        DBGINF("Got NET_XMIT_DONE %p\n", cmdrsp->net.buf);
                        spin_lock_irqsave(&devdata->priv_lock, flags);
			/* TODO: dc->chstat.got_xmit_done++; */
                        if (cmdrsp->net.xmtdone.xmt_done_result) {
                                LOGERRNAME(devdata->netdev,
                                           "XMIT_DONE failure buf:%p\n",
                                           cmdrsp->net.buf);
                                /* TODO: dc->chstat.xmit_fail++; */
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
				/* TODO: CLEAN UP CHANSTATS 
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
                                        *
                                         * enough NET_XMITs completed
                                         * so can restart netif queue
                                         *
                                        netif_wake_queue(netdev);
                                        vnicinfo->flow_control_lower_hits++;
                                }
				*/
                        }
                        skb_unlink(cmdrsp->net.buf, &devdata->xmitbufhead);
                        spin_unlock_irqrestore(&devdata->priv_lock, flags);
                        kfree_skb(cmdrsp->net.buf);
                        break;
                case NET_RCV_ENBDIS_ACK:
                        DBGINF("Got NET_RCV_ENBDIS_ACK on:%p\n",
                               (struct net_device *)
                               cmdrsp->net.enbdis.context);
			/* TODO: dc->chstat.got_enbdisack++; */
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

		/* TODO: 
		if (dc->chinfo.threadinfo.should_stop)
                        break;
		 */
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
		if (devdata->threadinfo.should_stop)
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
	visor_thread_start(&devdata->threadinfo, process_incoming_rsps, &devdata, "vnic_incoming");

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
