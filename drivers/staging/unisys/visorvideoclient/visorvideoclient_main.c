/* visorvideoclient_main.c
 *
 * Copyright (c) 2011 - 2014 UNISYS CORPORATION
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

/* This driver lives in a generic guest Linux partition, and registers to
 * receive video channels from the visorbus driver.  Currently, we don't need
 * any implementation here other than to accept the channels, because Linux
 * will successfully display video to the EFI framebuffer.  The reason we need
 * to accept the channels is so the visorbus driver will return successful
 * DEVICE_CREATEs to CONTROL, which enables the partition state to go
 * RUNNING.
 */

#include "uniklog.h"
#include "diagnostics/appos_subsystems.h"
#include "timskmod.h"
#include "globals.h"
#include "visorbus.h"
#include "visorchannel.h"
#include "consolevideochannel.h"
#include "consoleframebufferchannel.h"
#include "consoleframebuffermemorychannel.h"
#include "easyproc.h"

static spinlock_t devnopool_lock;
static void *dev_no_pool;	/**< pool to grab device numbers from */
static struct easyproc_driver_info easyproc_drv_info;

static int visorvideoclient_probe(struct visor_device *dev);
static void visorvideoclient_remove(struct visor_device *dev);
static int visorvideoclient_pause(struct visor_device *dev,
				  VISORBUS_STATE_COMPLETE_FUNC complete_func);
static int visorvideoclient_resume(struct visor_device *dev,
				   VISORBUS_STATE_COMPLETE_FUNC complete_func);
static void visorvideoclient_show_device_info(struct seq_file *seq, void *p);
static void visorvideoclient_show_driver_info(struct seq_file *seq);

/**  GUIDS for all channel types supported by this driver.
 */
static struct visor_channeltype_descriptor visorvideo_channel_types[] = {
	/*  Note that the only channel type we expect to be reported by the
	 *  bus driver is the CONSOLEVIDEO channel.  The other channel types
	 *  are simply contained within the CONSOLEVIDEO channel, and we
	 *  specify them here just so the bus driver knows what their
	 *  sizes are.  See visorconfb_probe().
	 */
	{ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL_GUID, "video",
	 CONSOLEVIDEO_CH_SIZE, CONSOLEVIDEO_CH_SIZE},
	{ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_GUID, "framebuffer",
	 CONSOLEFRAMEBUFFER_CH_SIZE, CONSOLEFRAMEBUFFER_CH_SIZE},
	{ULTRA_CONSOLELEGACYVIDEO_CHANNEL_PROTOCOL_GUID, "legacyvideo",
	 CONSOLELEGACYVIDEO_CH_SIZE, CONSOLELEGACYVIDEO_CH_SIZE},
	{ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL_GUID,
	 "framebuffermemory",
	 CONSOLEFRAMEBUFFERMEMORY_CH_SIZE, CONSOLEFRAMEBUFFERMEMORY_CH_SIZE},
	{NULL_UUID_LE, NULL, 0, 0}
};

/** This is used to tell the visor bus driver which types of visor devices
 *  we support, and what functions to call when a visor device that we support
 *  is attached or removed.
 */
static struct visor_driver visorvideoclient_driver = {
	.name = MYDRVNAME,
	.version = VERSION,
	.vertag = NULL,
	.owner = THIS_MODULE,
	.channel_types = visorvideo_channel_types,
	.probe = visorvideoclient_probe,
	.remove = visorvideoclient_remove,
	.pause = visorvideoclient_pause,
	.resume = visorvideoclient_resume,
	.channel_interrupt = NULL,
};

/** This is the private data that we store for each device.
 *  A pointer to this struct is kept in each "struct device", and can be
 *  obtained using visor_get_drvdata(dev).
 */
struct visorvideoclient_devdata {
	int devno;
	struct visor_device *dev;
	/** lock for dev */
	struct rw_semaphore lock_visor_dev;
	char name[99];
	struct list_head list_all;   /**< link within list_all_devices list */
	struct kref kref;
	struct easyproc_device_info procinfo;
};

/** List of all visorvideoclient_devdata structs,
  * linked via the list_all member */
static LIST_HEAD(list_all_devices);
static DEFINE_SPINLOCK(lock_all_devices);

#define devdata_put(devdata, why)					\
	do {								\
		int refcount;						\
		kref_put(&devdata->kref, devdata_release);		\
		refcount = atomic_read(&devdata->kref.refcount);	\
		if (visorvideoclient_debugref)				\
			VISORBUS_DEBUG_REFCOUNT_CHANGE			\
				(refcount+1, refcount, devdata, why);	\
	} while (0)

#define devdata_get(devdata, why)					\
	do {								\
		int refcount;						\
		kref_get(&devdata->kref);				\
		refcount = atomic_read(&devdata->kref.refcount);	\
		if (visorvideoclient_debugref)				\
			VISORBUS_DEBUG_REFCOUNT_CHANGE			\
				(refcount-1, refcount, devdata, why);	\
	} while (0)

static struct visorvideoclient_devdata *
devdata_create(struct visor_device *dev)
{
	void *rc = NULL;
	struct visorvideoclient_devdata *devdata = NULL;
	int devno = -1;

	devdata = kmalloc(sizeof(*devdata),
			  GFP_KERNEL|__GFP_NORETRY);
	if (devdata == NULL) {
		ERRDRV("allocation of visorvideoclient_devdata failed)\n");
		goto cleanups;
	}
	memset(devdata, '\0', sizeof(struct visorvideoclient_devdata));
	spin_lock(&devnopool_lock);
	devno = find_first_zero_bit(dev_no_pool, MAXDEVICES);
	set_bit(devno, dev_no_pool);
	spin_unlock(&devnopool_lock);
	if (devno == MAXDEVICES)
		devno = -1;
	if (devno < 0) {
		ERRDRV("attempt to create more than MAXDEVICES devices\n");
		goto cleanups;
	}
	devdata->devno = devno;
	devdata->dev = dev;
	strncpy(devdata->name, dev_name(&dev->device), sizeof(devdata->name));
	init_rwsem(&devdata->lock_visor_dev);
	kref_init(&devdata->kref);
	spin_lock(&lock_all_devices);
	list_add_tail(&devdata->list_all, &list_all_devices);
	spin_unlock(&lock_all_devices);
	rc = devdata;
cleanups:
	if (rc == NULL) {
		if (devno >= 0) {
			spin_lock(&devnopool_lock);
			clear_bit(devno, dev_no_pool);
			spin_unlock(&devnopool_lock);
		}
		if (devdata != NULL)
			kfree(devdata);
	}
	return rc;
}

static void
devdata_release(struct kref *mykref)
{
	struct visorvideoclient_devdata *devdata =
	    container_of(mykref, struct visorvideoclient_devdata, kref);

	INFODRV("%s", __func__);
	spin_lock(&devnopool_lock);
	clear_bit(devdata->devno, dev_no_pool);
	spin_unlock(&devnopool_lock);
	spin_lock(&lock_all_devices);
	list_del(&devdata->list_all);
	spin_unlock(&lock_all_devices);
	kfree(devdata);
	INFODRV("%s finished", __func__);
}

static int
visorvideoclient_probe(struct visor_device *dev)
{
	int rc;
	struct visorvideoclient_devdata *devdata = NULL;

	INFODRV("%s", __func__);
	devdata = devdata_create(dev);
	if (devdata == NULL) {
		rc = -1;
		goto cleanups;
	}
	visor_set_drvdata(dev, devdata);
	visor_easyproc_InitDevice(&easyproc_drv_info,
				  &devdata->procinfo, devdata->devno, devdata);
	rc = 0;

cleanups:
	INFODRV("%s finished", __func__);
	if (rc < 0) {
		if (devdata != NULL)
			devdata_put(devdata, "existence");
	}
	return rc;
}

static void
host_side_disappeared(struct visorvideoclient_devdata *devdata)
{
	down_write(&devdata->lock_visor_dev);
	sprintf(devdata->name, "<dev#%d-history>", devdata->devno);
	devdata->dev = NULL;	/* indicate device destroyed */
	up_write(&devdata->lock_visor_dev);
}

static void
visorvideoclient_remove(struct visor_device *dev)
{
	struct visorvideoclient_devdata *devdata = visor_get_drvdata(dev);

	INFODRV("%s", __func__);
	if (devdata == NULL) {
		ERRDRV("no devdata in %s", __func__);
		goto cleanups;
	}
	visor_set_drvdata(dev, NULL);
	visor_easyproc_DeInitDevice(&easyproc_drv_info,
				    &devdata->procinfo, devdata->devno);
	host_side_disappeared(devdata);
	devdata_put(devdata, "existence");
cleanups:
	INFODRV("%s finished", __func__);
}

static int
visorvideoclient_pause(struct visor_device *dev,
		       VISORBUS_STATE_COMPLETE_FUNC complete_func)
{
	INFODEV(dev_name(&dev->device), "paused");
	complete_func(dev, 0);
	return 0;
}

static int
visorvideoclient_resume(struct visor_device *dev,
			VISORBUS_STATE_COMPLETE_FUNC complete_func)
{
	INFODEV(dev_name(&dev->device), "resumed");
	complete_func(dev, 0);
	return 0;
}

static void
visorvideoclient_cleanup_guts(void)
{
	visorbus_unregister_visor_driver(&visorvideoclient_driver);
	visor_easyproc_DeInitDriver(&easyproc_drv_info);
	if (dev_no_pool != NULL) {
		kfree(dev_no_pool);
		dev_no_pool = NULL;
	}
}

static int
visorvideoclient_init(void)
{
	int rc;

	INFODRV("driver version %s loaded", VERSION);

	/* show module options */
	INFODRV("option - debug=%d", visorvideoclient_debug);
	INFODRV("         debugref=%d", visorvideoclient_debugref);

	spin_lock_init(&devnopool_lock);
	dev_no_pool = kzalloc(BITS_TO_LONGS(MAXDEVICES), GFP_KERNEL);
	if (dev_no_pool == NULL) {
		ERRDRV("Unable to create dev_no_pool");
		rc = -1;
		goto cleanups;
	}
	visor_easyproc_InitDriver(&easyproc_drv_info,
				  MYDRVNAME,
				  visorvideoclient_show_driver_info,
				  visorvideoclient_show_device_info);
	visorbus_register_visor_driver(&visorvideoclient_driver);
	rc = 0;

cleanups:
	if (rc < 0)
		visorvideoclient_cleanup_guts();
	return rc;
}

static void
visorvideoclient_cleanup(void)
{
	visorvideoclient_cleanup_guts();
	INFODRV("driver unloaded");
}

static void
visorvideoclient_show_device_info(struct seq_file *seq, void *p)
{
	struct visorvideoclient_devdata *devdata =
	    (struct visorvideoclient_devdata *)(p);
	seq_printf(seq, "devno=%d\n", devdata->devno);
	seq_printf(seq, "visorbus name = '%s'\n", devdata->name);
}

static void
visorvideoclient_show_driver_info(struct seq_file *seq)
{
	seq_printf(seq, "Version=%s\n", VERSION);
}

module_param_named(debug, visorvideoclient_debug, int, S_IRUGO);
MODULE_PARM_DESC(visorvideoclient_debug, "1 to debug");
int visorvideoclient_debug = 0;

module_param_named(debugref, visorvideoclient_debugref, int, S_IRUGO);
MODULE_PARM_DESC(visorvideoclient_debugref, "1 to debug reference counts");
int visorvideoclient_debugref = 0;

module_init(visorvideoclient_init);
module_exit(visorvideoclient_cleanup);

MODULE_AUTHOR("Unisys");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sPAR video driver for guest Linux: ver " VERSION);
MODULE_VERSION(VERSION);
