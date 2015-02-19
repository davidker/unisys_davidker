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
struct uisqueue_info { 
	struct channel_header __iomem *chan;
	/* channel containing queues in which commands & rsps are queued */
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

struct uisthread_info {
	struct task_struct *task;
	int id;
};

struct chaninfo {
	struct uisqueue_info *queueinfo;
	/* this specifies the queue structures for a channel */
	/* ALLOCATED BY THE OTHER END - WE JUST GET A POINTER TO THE MEMORY */
	spinlock_t insertlock;
	/* currently used only in visornic when sending data to uisnic */
	/* to synchronize the inserts into the signal queue */
	struct uisthread_info threadinfo;
	/* this specifies the thread structures used by the thread that */
	/* handels this channel */
};

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
#if VISORNIC_STATS
	unsigned long reject_jiffies_start; /* jiffie count at start of
					       NET_XMIT rejects */
#endif
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
	wait_queue_head_t rsp_queue;
	struct sk_buff **rcvbuf;
	int num_rcv_bufs;		 /* indicates how many rcv buffers
					    the vnic will post */
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
	struct uisthread_info threadinfo;
};

int uisthread_start(struct uisthread_info *thrinfo, 
		    int (*threadfn)(void *), 
		    void *thrcontext, char *name) 
{

}
void uisthread_stop(struct uisthread_info *thrinfo) 
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

	/* Let's start our threads to get responses */
	channel_offset = offsetof(struct spar_io_channel_protocol,
				  channel_header.features);
	visorbus_read_channel(dev, channel_offset, &features, 8);
	features |= ULTRA_IO_CHANNEL_IS_POLLING;
	visorbus_write_channel(dev, channel_offset, &features, 8);
	
	/* TODO: Get queue features flag and save it off */
	devdata->thread_wait_ms = 2;
	/* TODO: Start threads for process_incoming */

	/* create debgug/sysfs directories */ 
	devdata->eth_debugfs_dir = debugfs_create_dir(netdev->name, 
						      visornic_debugfs_dir);
	if (!devdata->eth_debugfs_dir) { 
		uisthread_stop(&devdata->threadinfo);
		kfree(devdata->cmdrsp_rcv);
		kfree(devdata->rcvbuf);
		free_netdev(netdev);
		return -ENOMEM;
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
