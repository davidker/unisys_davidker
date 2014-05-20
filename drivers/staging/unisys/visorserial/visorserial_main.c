/* visorserial_main.c
 *
 * Copyright � 2010 - 2013 UNISYS CORPORATION
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

#include "visorserial_private.h"
#include "easyproc.h"
#include "linuxserial.h"
#include "linuxconsole.h"
#include "uisutils.h"

static dev_t MajorDevSerial = -1;
				/**< indicates major num for serial devices */
static spinlock_t devnopool_lock;
static void *DevNoPool;	/**< pool to grab device numbers from */
static struct easyproc_driver_info Easyproc_driver_info;

static int visorserial_probe(struct visor_device *dev);
static void visorserial_remove(struct visor_device *dev);
static void visorserial_channel_interrupt(struct visor_device *dev);

#ifdef HAVE_UNLOCKED_IOCTL
static long visorserial_serial_ioctl(struct file *file,
				     unsigned int cmd, unsigned long arg);
#else
static int visorserial_serial_ioctl(struct inode *inode, struct file *file,
				    unsigned int cmd, unsigned long arg);
#endif
static int visorserial_serial_open(struct inode *inode, struct file *file);
static int visorserial_serial_release(struct inode *inode, struct file *file);
static ssize_t visorserial_serial_read(struct file *file,
				       char __user *buf,
				       size_t count, loff_t *ppos);
static ssize_t visorserial_serial_write(struct file *file,
					const char __user *buf,
					size_t count, loff_t *ppos);
static unsigned int visorserial_serial_poll(struct file *file,
					    poll_table *wait);

#define POLLJIFFIES     10

static void visorbus_enable_channel_interrupts(struct visor_device *dev);
static void visorbus_disable_channel_interrupts(struct visor_device *dev);

static int
simplebus_match(struct device *xdev, struct device_driver *xdrv)
{
	return 1;
}

/** This describes the TYPE of bus.
 *  (Don't confuse this with an INSTANCE of the bus.)
 */
static struct bus_type Simplebus_type = {
	.name = "visorconsole",
	.match = simplebus_match,
};

static struct workqueue_struct *Periodic_dev_workqueue;
static struct visor_device *StandaloneDevice;

static const struct file_operations visorserial_serial_fops = {
	.owner = THIS_MODULE,
	.open = visorserial_serial_open,
	.read = visorserial_serial_read,
	.write = visorserial_serial_write,
	.release = visorserial_serial_release,
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = visorserial_serial_ioctl,
#else
	.ioctl = visorserial_serial_ioctl,
#endif
	.poll = visorserial_serial_poll,
};

/** These are all the counters we maintain for each device.
 *  They will all be reported under /sys/bus/visorbus/devices/<devicename>.
 */
typedef struct {
	u64 hostBytesIn;   /**< bytes we have input from the host */
	u64 hostBytesOut;  /**< bytes we have output to the host */
	u64 umodeBytesIn;  /**< bytes we have input from user mode */
	u64 umodeBytesOut; /**< bytes we have output to user mode */
} DEVDATA_COUNTERS;

/** These are all the devdata properties we maintain for each device.
 *  They will all be reported under /sys/bus/visorbus/devices/<devicename>.
 */
typedef enum {
	prop_openFileCount,
	/* Add items above, but don't forget to modify
	 * register_devdata_attributes whenever you do...
	 */
	prop_DEVDATAMAX
} DEVDATA_PROPERTIES;

/** This is the private data that we store for each device.
 *  A pointer to this struct is kept in each "struct device", and can be
 *  obtained using visor_get_drvdata(dev).
 */
struct visorserial_devdata {
	int devno;
	struct visor_device *dev;
	/** lock for dev */
	struct rw_semaphore lockVisorDev;
	char name[99];
	struct list_head list_all;   /**< link within List_all_devices list */
	/** head of list of visorserial_filedata_serial structs, linked
	 *  via the list_all member */
	struct list_head list_files_serial;
	uint openFileCount;
	/** lock for list_files_serial */
	rwlock_t lock_files;
	/** lock for openFileCount */
	struct rw_semaphore lockOpenFileCount;
	DEVDATA_COUNTERS counter;
	struct device_attribute devdata_property[prop_DEVDATAMAX];
	struct kref kref;
	struct cdev cdev_serial;
	struct easyproc_device_info procinfo;
	int xmitqueue;
	int recvqueue;
	LINUXSERIAL *linuxserial;
};
/** List of all visorserial_devdata structs, linked via the list_all member */
static LIST_HEAD(List_all_devices);
static DEFINE_SPINLOCK(Lock_all_devices);

#define devdata_put(devdata, why)					\
	do {								\
		int refcount;						\
		kref_put(&devdata->kref, devdata_release);		\
		refcount = atomic_read(&devdata->kref.refcount);	\
		if (visorserial_debugref)				\
			VISORBUS_DEBUG_REFCOUNT_CHANGE			\
				(refcount+1, refcount, devdata, why);	\
	} while (0)
#define devdata_get(devdata, why)					\
	do {								\
		int refcount;						\
		kref_get(&devdata->kref);				\
		refcount = atomic_read(&devdata->kref.refcount);	\
		if (visorserial_debugref)				\
			VISORBUS_DEBUG_REFCOUNT_CHANGE			\
				(refcount-1, refcount, devdata, why);	\
	} while (0)

/** This is the private data that we store for each file descriptor that is
 *  opened to the "serial" character device.
 */
struct visorserial_filedata_serial {
	struct visorserial_devdata *devdata;
	/** link within devdata.list_files_serial list */
	struct list_head list_all;
	/** tasks queued here are waiting for read data */
	wait_queue_head_t waiting_readers;
	CHARQUEUE *data_from_host;
	unsigned char buf[NFILEWRITEBYTESTOBUFFER];
};

static void new_char_from_host(struct visorserial_devdata *devdata, U8 c);
static void new_char_to_host(void *context, U8 c);
static void serial_destroy_file(struct visorserial_filedata_serial *filedata);
static void host_side_disappeared(struct visorserial_devdata *devdata);
static void visorserial_show_device_info(struct seq_file *seq, void *p);
static void visorserial_show_driver_info(struct seq_file *seq);

/*  DEVICE attributes
 *
 *  define & implement display of device attributes under
 *  /sys/bus/visorbus/devices/<devicename>.
 *
 */

static ssize_t
devdata_property_show(struct device *ddev,
		      struct device_attribute *attr, char *buf)
{
	struct visorserial_devdata *devdata = dev_get_drvdata(ddev);
	ulong offset = (ulong) (attr) - (ulong) (devdata->devdata_property);
	ulong ix = offset / sizeof(struct device_attribute);
	if (ix >= prop_DEVDATAMAX) {
		pr_err("%s:%d trouble in paradise; ix=%lu\n",
		       __FILE__, __LINE__, ix);
		return 0;
	}
	switch (ix) {
	case prop_openFileCount:
		return sprintf(buf, "%u\n", devdata->openFileCount);
	default:
		pr_err("%s:%d trouble in paradise; ix=%lu\n",
		       __FILE__, __LINE__, ix);
		return 0;
	}
	return 0;
}

static int
register_devdata_attributes(struct visor_device *dev)
{
	int rc, i;
	struct visorserial_devdata *devdata = visor_get_drvdata(dev);
	struct device_attribute *pattr = devdata->devdata_property;

	pattr[prop_openFileCount].attr.name = "openFileCount";
	for (i = 0; i < prop_DEVDATAMAX; i++) {
		pattr[i].attr.mode = S_IRUGO;
		pattr[i].show = devdata_property_show;
		pattr[i].store = NULL;
		rc = device_create_file(&dev->device, &pattr[i]);
		if (rc < 0) {
			ERRDRV("device_create_file(&dev->device, &pattr[i]): (status=%d)\n", rc);
			goto Away;
		}
	}

	rc = 0;
Away:
	return rc;
}

static int
register_device_attributes(struct visor_device *dev)
{
	int rc;

	rc = register_devdata_attributes(dev);
	if (rc < 0) {
		ERRDRV("register_devdata_attributes(dev): (status=%d)\n", rc);
		goto Away;
	}
	rc = 0;
Away:
	return rc;
}

static int
unregister_devdata_attributes(struct visor_device *dev)
{
	int i;

	struct visorserial_devdata *devdata = visor_get_drvdata(dev);
	struct device_attribute *pattr = devdata->devdata_property;
	for (i = 0; i < prop_DEVDATAMAX; i++)
		device_remove_file(&dev->device, &pattr[i]);
	return 0;
}

static int
unregister_device_attributes(struct visor_device *dev)
{
	unregister_devdata_attributes(dev);
	return 0;
}

static struct visorserial_devdata *
devdata_create(struct visor_device *dev)
{
	void *rc = NULL;
	struct visorserial_devdata *devdata = NULL;
	int devno = -1;

	devdata = kmalloc(sizeof(struct visorserial_devdata),
			  GFP_KERNEL|__GFP_NORETRY);
	if (devdata == NULL) {
		ERRDRV("allocation of visorserial_devdata failed\n");
		goto Away;
	}
	memset(devdata, '\0', sizeof(struct visorserial_devdata));
	cdev_init(&devdata->cdev_serial, NULL);
	spin_lock(&devnopool_lock);
	devno = find_first_zero_bit(DevNoPool, MAXDEVICES);
	set_bit(devno, DevNoPool);
	spin_unlock(&devnopool_lock);
	if (devno == MAXDEVICES)
		devno = -1;
	if (devno < 0) {
		ERRDRV("attempt to create more than MAXDEVICES devices\n");
		goto Away;
	}

	devdata->devno = devno;
	devdata->dev = dev;
	dev_set_name(&devdata->dev->device, devdata->name);
	devdata->xmitqueue = 0;
	devdata->recvqueue = 1;
	if (visorserial_rxtxswap && ((devno % 2) == 0)) {
		devdata->xmitqueue = 1;
		devdata->recvqueue = 0;
	}

	cdev_init(&devdata->cdev_serial, &visorserial_serial_fops);
	devdata->cdev_serial.owner = THIS_MODULE;
	if (cdev_add(&devdata->cdev_serial,
		     MKDEV(MAJOR(MajorDevSerial), devdata->devno), 1) < 0) {
		ERRDRV("failed to create serial char device\n");
		goto Away;
	}

	rwlock_init(&devdata->lock_files);
	init_rwsem(&devdata->lockOpenFileCount);
	init_rwsem(&devdata->lockVisorDev);
	INIT_LIST_HEAD(&devdata->list_files_serial);
	kref_init(&devdata->kref);

	spin_lock(&Lock_all_devices);
	list_add_tail(&devdata->list_all, &List_all_devices);
	spin_unlock(&Lock_all_devices);

	if (visorserial_createttydevice)
		devdata->linuxserial = linuxserial_create(devno, devdata,
							  new_char_to_host);
	if (devdata->linuxserial != NULL)
		/* since we may have a tty attached, we will need to get
		 * interrupted when data comes in on the channel so we can
		 * deliver it to the tty
		 */
		visorbus_enable_channel_interrupts(devdata->dev);

	rc = devdata;
Away:
	if (rc == NULL) {
		if (devno >= 0)
		{
			spin_lock(&devnopool_lock);
			clear_bit(devno, DevNoPool);
			spin_unlock(&devnopool_lock);
		}
		if (devdata != NULL) {
			if (devdata->cdev_serial.ops != NULL)
				cdev_del(&devdata->cdev_serial);
			kfree(devdata);
		}
	}
	return rc;
}

static void
devdata_release(struct kref *mykref)
{
	struct visorserial_devdata *devdata = container_of(mykref,
							   struct
							   visorserial_devdata,
							   kref);
	INFODRV("%s", __func__);
	spin_lock(&devnopool_lock);
	clear_bit(devdata->devno, DevNoPool);
	spin_unlock(&devnopool_lock);
	spin_lock(&Lock_all_devices);
	list_del(&devdata->list_all);
	spin_unlock(&Lock_all_devices);
	cdev_del(&devdata->cdev_serial);
	if (devdata->linuxserial) {
		linuxserial_destroy(devdata->linuxserial);
		devdata->linuxserial = NULL;
		LOCKREADSEM(&devdata->lockVisorDev);
		if (devdata->dev != NULL)
			visorbus_disable_channel_interrupts(devdata->dev);
		UNLOCKREADSEM(&devdata->lockVisorDev);
	}
	kfree(devdata);
	INFODRV("%s finished", __func__);
}

static int
visorserial_probe(struct visor_device *dev)
{
	int rc = 0;
	struct visorserial_devdata *devdata = NULL;

	INFODRV("%s", __func__);

	devdata = devdata_create(dev);
	if (devdata == NULL) {
		rc = -1;
		goto Away;
	}
	if (ULTRA_CONSOLE_CHANNEL_OK_CLIENT
	    (visorchannel_get_header(dev->visorchannel), NULL) ||
	    ULTRA_CONSOLESERIAL_CHANNEL_OK_CLIENT
	    (visorchannel_get_header(dev->visorchannel), NULL))
		;
	else {
		rc = -1;
		ERRDRV("consoleserial channel cannot be used: (status=%d)\n",
		       rc);
		goto Away;
	}
	visor_set_drvdata(dev, devdata);
	if (register_device_attributes(dev) < 0) {
		rc = -1;
		ERRDRV("register_device_attributes failed: (status=%d)\n", rc);
		goto Away;
	}
	visor_easyproc_InitDevice(&Easyproc_driver_info,
				  &devdata->procinfo, devdata->devno, devdata);

Away:
	INFODRV("%s finished", __func__);
	if (rc < 0) {
		if (devdata != NULL)
			devdata_put(devdata, "existence");
	}
	return rc;
}

static void
visorserial_remove(struct visor_device *dev)
{
	struct visorserial_devdata *devdata = visor_get_drvdata(dev);
	INFODRV("%s", __func__);
	if (devdata == NULL) {
		ERRDRV("no devdata in %s", __func__);
		goto Away;
	}
	unregister_device_attributes(dev);
	visor_set_drvdata(dev, NULL);
	visor_easyproc_DeInitDevice(&Easyproc_driver_info,
				    &devdata->procinfo, devdata->devno);
	host_side_disappeared(devdata);
	devdata_put(devdata, "existence");

Away:
	INFODRV("%s finished", __func__);
}

static void
visorserial_channel_interrupt(struct visor_device *dev)
{
	U8 data;

	struct visorserial_devdata *devdata = visor_get_drvdata(dev);
	if (devdata == NULL) {
		ERRDRV("no devdata in %s", __func__);
		goto Away;
	}
	/* INFODRV("%s", __func__); */
	while (visorchannel_signalremove(dev->visorchannel,
					 devdata->recvqueue, &data)) {
		new_char_from_host(devdata, data);
		devdata->counter.hostBytesIn++;
	}

Away:
	;
}

static void
destroy_visor_device(struct visor_device *dev)
{
	char s[99];

	if (dev == NULL)
		return;
	if (dev->periodic_work != NULL) {
		visor_periodic_work_destroy(dev->periodic_work);
		dev->periodic_work = NULL;
	}
	if (dev->visorchannel != NULL) {
		INFODRV("Channel %s disconnected",
			visorchannel_id(dev->visorchannel, s));
		visorchannel_destroy(dev->visorchannel);
		dev->visorchannel = NULL;
	}
	if (dev)
		kfree(dev);
}

static void
simplebus_release_device(struct device *xdev)
{
	struct visor_device *dev = to_visor_device(xdev);
	INFODEV(dev_name(&dev->device), "child device destroyed");
	destroy_visor_device(dev);
}

static void
dev_periodic_work(void *xdev)
{
	struct visor_device *dev = (struct visor_device *) xdev;
	visorserial_channel_interrupt(dev);
	if (!visor_periodic_work_nextperiod(dev->periodic_work))
		put_visordev(dev, "delayed work", visorserial_debugref);
}

static void
dev_start_periodic_work(struct visor_device *dev)
{
	if (dev->being_removed)
		return;
	/* now up by at least 2 */
	get_visordev(dev, "delayed work", visorserial_debugref);
	if (!visor_periodic_work_start(dev->periodic_work))
		put_visordev(dev, "delayed work", visorserial_debugref);
}

static void
dev_stop_periodic_work(struct visor_device *dev)
{
	if (visor_periodic_work_stop(dev->periodic_work))
		put_visordev(dev, "delayed work", visorserial_debugref);
}

static void
visorbus_enable_channel_interrupts(struct visor_device *dev)
{
	dev_start_periodic_work(dev);
}

static void
visorbus_disable_channel_interrupts(struct visor_device *dev)
{
	dev_stop_periodic_work(dev);
}

static struct visor_device *
create_visor_device(U64 addr)
{
	struct visor_device *rc = NULL;
	VISORCHANNEL *visorchannel = NULL;
	struct visor_device *dev = NULL;
	BOOL gotten = FALSE;
	char s[99];
	uuid_le guid = ULTRA_CONSOLE_CHANNEL_PROTOCOL_GUID;

	/* prepare chan_hdr (abstraction to read/write channel memory) */
	visorchannel =
	    visorchannel_create(addr, 0 /*size in chan hdr */ , guid);
	if (visorchannel == NULL) {
		ERRDRV("channel addr = 0x%-16.16Lx", addr);
		ERRDRV("visorchannel_create failed\n");
		goto Away;
	}
	INFODRV("Channel %s discovered and connected",
		visorchannel_id(visorchannel, s));
	dev = kmalloc(sizeof(struct visor_device), GFP_KERNEL|__GFP_NORETRY);
	if (dev == NULL) {
		ERRDRV("failed to allocate visor_device\n");
		goto Away;
	}
	memset(dev, 0, sizeof(struct visor_device));
	dev->visorchannel = visorchannel;
	sema_init(&dev->visordriver_callback_lock, 1);	/* unlocked */
	dev->device.bus = &Simplebus_type;
	device_initialize(&dev->device);
	dev->device.release = simplebus_release_device;
	/* keep a reference just for us */
	get_visordev(dev, "create", visorserial_debugref);
	gotten = TRUE;
	dev->periodic_work = visor_periodic_work_create(POLLJIFFIES,
							Periodic_dev_workqueue,
							dev_periodic_work,
							dev,
							dev_name(&dev->device));
	if (dev->periodic_work == NULL) {
		ERRDRV("failed to create periodic_work\n");
		goto Away;
	}

	/* bus_id must be a unique name with respect to this bus TYPE
	 * (NOT bus instance).  That's why we need to include the bus
	 * number within the name.
	 */
	dev_set_name(&dev->device, "visorconsole");

	if (device_add(&dev->device) < 0) {
		ERRDRV("device_add failed\n");
		goto Away;
	}
	/* note: device_register is simply device_initialize + device_add */

	INFODEV(dev_name(&dev->device),
		"child device 0x%p created", &dev->device);

	rc = dev;
Away:
	if (rc == NULL) {
		if (gotten)
			put_visordev(dev, "create", visorserial_debugref);
		if (dev != NULL) {
			destroy_visor_device(dev);
			dev = NULL;
		}
	}
	return rc;
}

static void
visorserial_cleanup_guts(void)
{
	if (StandaloneDevice) {
		StandaloneDevice->being_removed = TRUE;
		/* ensure that the being_removed flag is set before
		 * continuing
		 */
		wmb();
		dev_stop_periodic_work(StandaloneDevice);
		lxcon_console_offline(StandaloneDevice);
		visorserial_remove(StandaloneDevice);
		put_visordev(StandaloneDevice, "create", visorserial_debugref);
		device_unregister(&StandaloneDevice->device);
		StandaloneDevice = NULL;
	}
	flush_workqueue(Periodic_dev_workqueue); /* better not be any work! */
	destroy_workqueue(Periodic_dev_workqueue);
	Periodic_dev_workqueue = NULL;
	bus_unregister(&Simplebus_type);
	visor_easyproc_DeInitDriver(&Easyproc_driver_info);
	if (MAJOR(MajorDevSerial) >= 0) {
		unregister_chrdev_region(MajorDevSerial, MAXDEVICES);
		MajorDevSerial = MKDEV(0, 0);
	}
	if (DevNoPool != NULL) {
		kfree(DevNoPool);
		DevNoPool = NULL;
	}
}

int __init
visorserial_init(void)
{
	int rc = -1;
	U64 visorserial_addr = 0;

	INFODRV("driver version %s loaded", VERSION);
	/* uintpool_test(); */
	INFODRV("Built with:");
	INFODRV("         STANDALONE_CLIENT yes");
	INFODRV("option - rxtxswap=%d", visorserial_rxtxswap);
	INFODRV("         createttydevice=%d",
		visorserial_createttydevice);
	INFODRV("         channeladdress=0x%lx",
		visorserial_channeladdress);
	INFODRV("         clearchannel=%d",
		visorserial_clearchannel);
	INFODRV("         debug=%d", visorserial_debug);
	INFODRV("         debugref=%d", visorserial_debugref);

	MajorDevSerial = MKDEV(0, 0);
	spin_lock_init(&devnopool_lock);
	DevNoPool = kzalloc(BITS_TO_LONGS(MAXDEVICES), GFP_KERNEL);
	if (DevNoPool == NULL) {
		ERRDRV("Unable to create DevNoPool");
		goto Away;
	}
	if (alloc_chrdev_region(&MajorDevSerial, 0, MAXDEVICES,
				MYDRVNAME "_serial") < 0) {
		ERRDRV("Unable to register char device %s",
		       MYDRVNAME "_serial");
		goto Away;
	}
	visor_easyproc_InitDriver(&Easyproc_driver_info,
				  MYDRVNAME,
				  visorserial_show_driver_info,
				  visorserial_show_device_info);
	rc = bus_register(&Simplebus_type);
	if (rc < 0) {
		ERRDRV("bus_register(&Simplebus_type): (status=%d)\n", rc);
		goto Away;
	}
	Periodic_dev_workqueue = create_singlethread_workqueue("visorconsole");
	if (Periodic_dev_workqueue == NULL) {
		rc = -ENOMEM;
		ERRDRV("cannot create dev workqueue: (status=%d)\n", rc);
		goto Away;
	}
	if (!visorserial_channeladdress) {
		INFODRV("channeladdress module/kernel parameter not specified so issuing vmcall");
		if (!VMCALL_SUCCESSFUL
		    (Issue_VMCALL_IO_VISORSERIAL_ADDR(&visorserial_addr))) {
			ERRDRV("channeladdress module/kernel parameter not specified and vmcall failed.");
			rc = -1;
			goto Away;
		}
		INFODRV("visorserial channel addr=%llx", visorserial_addr);
		visorserial_channeladdress = visorserial_addr;
	}
	StandaloneDevice =
	    create_visor_device(visorserial_channeladdress);
	if (StandaloneDevice == NULL) {
		ERRDRV("failed to initialize channel @ 0x%lx",
		       visorserial_channeladdress);
		rc = -1;
		goto Away;
	}
	if (visorserial_probe(StandaloneDevice) < 0) {
		ERRDRV("probe failed");
		rc = -1;
		goto Away;
	}
	if (visor_get_drvdata(StandaloneDevice) != NULL)
		lxcon_console_online(visor_get_drvdata(StandaloneDevice),
				     new_char_to_host);

	rc = 0;
Away:
	if (rc < 0)
		visorserial_cleanup_guts();
	return rc;
}

static void
visorserial_cleanup(void)
{
	visorserial_cleanup_guts();
	INFODRV("driver unloaded");
}

static struct visorserial_filedata_serial *
serial_create_file(struct visorserial_devdata *devdata)
{
	void *rc = NULL;
	struct visorserial_filedata_serial *filedata = NULL;
	filedata = kmalloc(sizeof(struct visorserial_filedata_serial),
			   GFP_KERNEL|__GFP_NORETRY);
	if (filedata == NULL) {
		rc = NULL;
		goto Away;
	}
	filedata->devdata = devdata;
	devdata_get(devdata, "create_file");
	filedata->data_from_host = NULL;
	init_waitqueue_head(&filedata->waiting_readers);
	filedata->data_from_host = visor_charqueue_create(NHOSTBYTESTOBUFFER);
	if (filedata->data_from_host == NULL) {
		rc = NULL;
		goto Away;
	}
	rc = filedata;
Away:
	if (rc == NULL) {
		if (filedata != NULL) {
			serial_destroy_file(filedata);
			filedata = NULL;
		}
	}
	return rc;
}

static void
serial_destroy_file(struct visorserial_filedata_serial *filedata)
{
	if (filedata->data_from_host != NULL) {
		visor_charqueue_destroy(filedata->data_from_host);
		filedata->data_from_host = NULL;
	}
	devdata_put(filedata->devdata, "create_file");
	kfree(filedata);
}

static void
serial_new_host_char(struct visorserial_filedata_serial *filedata, U8 c)
{
	visor_charqueue_enqueue(filedata->data_from_host, c);
	wake_up(&filedata->waiting_readers);
}

static void
new_char_from_host(struct visorserial_devdata *devdata, U8 c)
{
	struct list_head *listentry, *listtmp;
	read_lock(&devdata->lock_files);
	list_for_each_safe(listentry, listtmp, &devdata->list_files_serial) {
		struct visorserial_filedata_serial *filedata =
		    list_entry(listentry, struct visorserial_filedata_serial,
			       list_all);
		serial_new_host_char(filedata, c);
	}
	if (devdata->linuxserial != NULL)
		linuxserial_rx_char(devdata->linuxserial, c);
	read_unlock(&devdata->lock_files);
}

static void
new_char_to_host(void *context, U8 c)
{
	struct visorserial_devdata *devdata =
	    (struct visorserial_devdata *) (context);
	int done = 0;
	if (devdata->dev == NULL) {
		HUHDRV("dev is NULL in %s??", __func__);
		return;
	}
	while (!done) {
		if (visorchannel_signalinsert(devdata->dev->visorchannel,
					      devdata->xmitqueue, &c)) {
			devdata->counter.hostBytesOut++;
			done = 1;
		} else if (OK_TO_BLOCK_FOR_CONSOLE) {
			/* bug here; counter will show that we dropped
			* chars, when we actually didn't
			*/
			int i;
			for (i = 0; i < 100000; i++)
				cpu_relax();
		} else
			done = 1;
	}
}

static void
host_side_disappeared(struct visorserial_devdata *devdata)
{
	struct list_head *listentry, *listtmp;
	LOCKWRITESEM(&devdata->lockVisorDev);
	sprintf(devdata->name, "<dev#%d-history>", devdata->devno);
	devdata->dev = NULL;	/* indicate device destroyed */
	UNLOCKWRITESEM(&devdata->lockVisorDev);
	read_lock(&devdata->lock_files);
	list_for_each_safe(listentry, listtmp, &devdata->list_files_serial) {
		struct visorserial_filedata_serial *filedata =
		    list_entry(listentry, struct visorserial_filedata_serial,
			       list_all);
		wake_up(&filedata->waiting_readers);
	}
	read_unlock(&devdata->lock_files);
}

static BOOL
serial_ready_to_read(struct visorserial_filedata_serial *filedata)
{
	if (!visor_charqueue_is_empty(filedata->data_from_host))
		return TRUE;
	if (filedata->devdata->dev == NULL)	/* channel disappeared */
		return TRUE;
	return FALSE;
}

static void
firstFileOpened(struct visorserial_filedata_serial *filedata)
{
	struct visorserial_devdata *devdata = filedata->devdata;
	INFODEV(devdata->name, "lights on");
	if (devdata->linuxserial == NULL) {
		LOCKREADSEM(&devdata->lockVisorDev);
		if (devdata->dev != NULL)
			visorbus_enable_channel_interrupts(devdata->dev);
		UNLOCKREADSEM(&devdata->lockVisorDev);
	}
}

static void
lastFileClosed(struct visorserial_filedata_serial *filedata)
{
	struct visorserial_devdata *devdata = filedata->devdata;
	INFODEV(devdata->name, "lights off");
	if (devdata->linuxserial == NULL) {
		LOCKREADSEM(&devdata->lockVisorDev);
		if (devdata->dev != NULL)
			visorbus_disable_channel_interrupts(devdata->dev);
		UNLOCKREADSEM(&devdata->lockVisorDev);
	}
}

static int
visorserial_serial_open(struct inode *inode, struct file *file)
{
	struct visorserial_devdata *devdata = NULL;
	struct visorserial_filedata_serial *filedata = NULL;
	unsigned minor_number = iminor(inode);
	int rc = -ENODEV;

	list_for_each_entry(devdata, &List_all_devices, list_all) {
		if (devdata->devno == minor_number) {
			INFODEV(devdata->name,
				"%s minor=%d", __func__, minor_number);
			filedata = serial_create_file(devdata);
			if (filedata == NULL) {
				rc = -ENOMEM;
				ERRDRV("cannot alloc file data: (status=%d)\n",
				       rc);
				goto Away;
			}
			file->private_data = filedata;
			write_lock(&devdata->lock_files);
			list_add_tail(&filedata->list_all,
				      &devdata->list_files_serial);
			write_unlock(&devdata->lock_files);
			LOCKWRITESEM(&devdata->lockOpenFileCount);
			devdata->openFileCount++;
			if (devdata->openFileCount == 1)
				firstFileOpened(filedata);
			UNLOCKWRITESEM(&devdata->lockOpenFileCount);
			rc = 0;
			goto Away;
		}
	}
	rc = -ENODEV;
Away:
	if (rc < 0)
		ERRDRV("%s minor=%d failed", __func__, minor_number);
	return rc;
}

static int
visorserial_serial_release(struct inode *inode, struct file *file)
{
	int rc = -1;
	struct visorserial_filedata_serial *filedata =
	    (struct visorserial_filedata_serial *) (file->private_data);
	struct visorserial_devdata *devdata = NULL;
	if (filedata == NULL) {
		ERRDRV("unknown file: (status=%d)\n", rc);
		goto Away;
	}
	devdata = filedata->devdata;
	if (devdata == NULL) {
		ERRDRV("unknown device: (status=%d)\n", rc);
		goto Away;
	}

	INFODEV(devdata->name, "%s", __func__);
	LOCKWRITESEM(&devdata->lockOpenFileCount);
	if (devdata->openFileCount == 1)
		lastFileClosed(filedata);
	devdata->openFileCount--;
	UNLOCKWRITESEM(&devdata->lockOpenFileCount);
	write_lock(&devdata->lock_files);
	list_del(&filedata->list_all);
	write_unlock(&devdata->lock_files);
	serial_destroy_file(filedata);
	file->private_data = NULL;
	rc = 0;
Away:
	return rc;
}

#ifdef HAVE_UNLOCKED_IOCTL
static long
visorserial_serial_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int
visorserial_serial_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
#endif
{
	int rc = -1;
	struct visorserial_filedata_serial *filedata =
	    (struct visorserial_filedata_serial *) (file->private_data);
	struct visorserial_devdata *devdata = NULL;
	if (filedata == NULL) {
		ERRDRV("unknown file: (status=%d)\n", rc);
		goto Away;
	}
	devdata = filedata->devdata;
	if (devdata == NULL) {
		ERRDRV("unknown device: (status=%d)\n", rc);
		goto Away;
	}

	/* void __user *userptr = (void __user *)(arg); */
	INFODEV(devdata->name, "%s cmd=0x%x", __func__, cmd);
	rc = 0;
Away:
	return rc;
}

static ssize_t
visorserial_serial_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	int rc = -1, readchars = 0, mycount = count;
	struct visorserial_filedata_serial *filedata =
	    (struct visorserial_filedata_serial *) (file->private_data);
	struct visorserial_devdata *devdata = NULL;
	loff_t pos = *ppos;

	if(pos < 0)
		return -EINVAL;

	if(pos > 0 || !count)
		return 0;

	if (filedata == NULL) {
		ERRDRV("unknown file: (status=%d)\n", rc);
		goto Away;
	}
	devdata = filedata->devdata;
	if (devdata == NULL) {
		ERRDRV("unknown device: (status=%d)\n", rc);
		goto Away;
	}
	DEBUGDEV(devdata->name, "%s", __func__);
	if (file->f_flags & O_NONBLOCK)
		if (!serial_ready_to_read(filedata)) {
			rc = -EAGAIN;
			goto Away;
		}
	if (mycount > NFILEREADBYTESTOBUFFER)
		mycount = NFILEREADBYTESTOBUFFER;
	while (readchars <= 0) {
		wait_event_interruptible(filedata->waiting_readers,
					 serial_ready_to_read(filedata));
		if (signal_pending(current)) {
			rc = -EINTR;
			goto Away;
		}
		if (devdata->dev == NULL) {	/* channel disappeared */
			rc = 0;	/* end-of-file */
			goto Away;
		}
		readchars = visor_charqueue_dequeue_n(filedata->data_from_host,
						      filedata->buf, mycount);
	}
	if (copy_to_user(buf, filedata->buf, readchars)) {
		rc = -EFAULT;
		goto Away;
	}
	devdata->counter.umodeBytesOut += readchars;
	*ppos += readchars;
	rc = readchars;
Away:
	DEBUGDEV(devdata->name, "%s read %d of %d", __func__,
		 readchars, count);
	return rc;
}

static ssize_t
visorserial_serial_write(struct file *file,
			 const char __user *buf, size_t count, loff_t *ppos)
{
	int rc = -1, i = 0, writechars = 0;
	struct visorserial_filedata_serial *filedata =
	    (struct visorserial_filedata_serial *) (file->private_data);
	struct visorserial_devdata *devdata = NULL;
	if (filedata == NULL) {
		ERRDRV("unknown file: (status=%d)\n", rc);
		goto Away;
	}
	devdata = filedata->devdata;
	if (devdata == NULL) {
		ERRDRV("unknown device: (status=%d)\n", rc);
		goto Away;
	}
	DEBUGDEV(devdata->name, "%s", __func__);
	if (count > NFILEWRITEBYTESTOBUFFER)
		count = NFILEWRITEBYTESTOBUFFER;
	if (copy_from_user(filedata->buf, buf, count)) {
		rc = -EFAULT;
		goto Away;
	}
	devdata->counter.umodeBytesIn += count;
	LOCKREADSEM(&devdata->lockVisorDev);
	if (devdata->dev == NULL) {	/* host channel is gone */
		UNLOCKREADSEM(&devdata->lockVisorDev);
		rc = 0;	/* eof */
		goto Away;
	}

	for (i = 0; i < count; i++) {
		devdata->counter.hostBytesOut++;
		writechars++;
	}
	UNLOCKREADSEM(&devdata->lockVisorDev);

	rc = count;
Away:
	DEBUGDEV(devdata->name, "%s wrote %d of %d", __func__,
		 writechars, count);
	return rc;
}

static unsigned int
visorserial_serial_poll(struct file *file, poll_table *wait)
{
	int rc = -1;
	struct visorserial_filedata_serial *filedata =
	    (struct visorserial_filedata_serial *) (file->private_data);
	struct visorserial_devdata *devdata = NULL;
	if (filedata == NULL) {
		ERRDRV("unknown file: (status=%d)\n", rc);
		goto Away;
	}
	devdata = filedata->devdata;
	if (devdata == NULL) {
		ERRDRV("unknown device: (status=%d)\n", rc);
		goto Away;
	}

	poll_wait(file, &filedata->waiting_readers, wait);
	if (serial_ready_to_read(filedata)) {
		rc = POLLIN | POLLRDNORM;
		goto Away;
	}

	rc = 0;
Away:
	return rc;
}

static void
visorserial_show_device_info(struct seq_file *seq, void *p)
{
	struct visorserial_devdata *devdata =
	    (struct visorserial_devdata *) (p);
	seq_printf(seq, "devno=%d\n", devdata->devno);
	seq_printf(seq, "visorbus name = '%s'\n", devdata->name);
	seq_printf(seq, "hostBytesIn=%llu\n", devdata->counter.hostBytesIn);
	seq_printf(seq, "hostBytesOut=%llu\n", devdata->counter.hostBytesOut);
	seq_printf(seq, "umodeBytesIn=%llu\n", devdata->counter.umodeBytesIn);
	seq_printf(seq, "umodeBytesOut=%llu\n", devdata->counter.umodeBytesOut);
	if (devdata->dev == NULL || devdata->dev->visorchannel == NULL)
		return;
	visorchannel_debug(devdata->dev->visorchannel, 2, seq, 0);
	visorchannel_dump_section
	    (devdata->dev->visorchannel, "InData",
	     offsetof(ULTRA_CONSOLE_CHANNEL_PROTOCOL, InData),
	     sizeofmember(ULTRA_CONSOLE_CHANNEL_PROTOCOL, InData), seq);
	visorchannel_dump_section
	    (devdata->dev->visorchannel, "OutData",
	     offsetof(ULTRA_CONSOLE_CHANNEL_PROTOCOL, OutData),
	     sizeofmember(ULTRA_CONSOLE_CHANNEL_PROTOCOL, OutData), seq);
}

static void
visorserial_show_driver_info(struct seq_file *seq)
{
	char *p = lxcon_get_early_buffer();
	seq_printf(seq, "Version=%s\n", VERSION);
	seq_printf(seq, "Console_write_bytes=%lu\n",
		   visorserial_console_write_bytes);
	seq_printf(seq, "Console_dropped_bytes=%lu\n",
		   visorserial_console_dropped_bytes);
	seq_puts(seq, "    -*-*-*-*-*- begin console buffer -*-*-*-*-*-\n");
	if (p)
		while (*p)
			seq_printf(seq, "%c", *(p++));
	seq_puts(seq, "\n");
	seq_puts(seq, "    -*-*-*-*-*-  end  console buffer -*-*-*-*-*-\n");
}

module_param_named(rxtxswap, visorserial_rxtxswap, int, S_IRUGO);
MODULE_PARM_DESC(visorserial_rxtxswap,
		 "non-0 if you want even-numbered devices to have their "
		 "receive and transmit wires crossed");
int visorserial_rxtxswap = 1;

module_param_named(createttydevice, visorserial_createttydevice,
		   int, S_IRUGO);
MODULE_PARM_DESC(visorserial_createttydevice,
		 "non-0 if you want to create a tty device for each "
		 "visorserial device, suitable for logins and getty");
int visorserial_createttydevice = 1;

module_param_named(channeladdress, visorserial_channeladdress,
		   ulong, S_IRUGO);
MODULE_PARM_DESC(visorserial_channeladdress,
		 "if a specific console channel is to be used, and "
		 "there is no visor bus, specify the physical address "
		 "of the channel here");
ulong visorserial_channeladdress = 0;

module_param_named(clearchannel, visorserial_clearchannel,
		   int, S_IRUGO);
MODULE_PARM_DESC(visorserial_clearchannel,
		 "non-0 when you want to forcibly initialize the "
		 "channel to a known-good state before first using it");
int visorserial_clearchannel = 0;

module_param_named(debug, visorserial_debug, int, S_IRUGO);
MODULE_PARM_DESC(visorserial_debug, "1 to debug");
int visorserial_debug = 0;

module_param_named(debugref, visorserial_debugref, int, S_IRUGO);
MODULE_PARM_DESC(visorserial_debugref, "1 to debug reference counts");
int visorserial_debugref = 0;

module_init(visorserial_init);
module_exit(visorserial_cleanup);

MODULE_AUTHOR("Unisys");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Supervisor visorserial driver for service partition: ver "
		   VERSION);
MODULE_VERSION(VERSION);
