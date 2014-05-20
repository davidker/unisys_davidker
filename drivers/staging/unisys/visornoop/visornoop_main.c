/* Copyright © 2012 - 2013 UNISYS CORPORATION
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
#include "controlframework.h"
#include "channel_guid.h"
#include "easyproc.h"

static spinlock_t devnopool_lock;
static void *DevNoPool;	/**< pool to grab device numbers from */
static struct easyproc_driver_info Easyproc_driver_info;

static int visornoop_probe(struct visor_device *dev);
static void visornoop_remove(struct visor_device *dev);
static int visornoop_pause(struct visor_device *dev,
				  VISORBUS_STATE_COMPLETE_FUNC complete_func);
static int visornoop_resume(struct visor_device *dev,
				   VISORBUS_STATE_COMPLETE_FUNC complete_func);
static void visornoop_show_device_info(struct seq_file *seq, void *p);
static void visornoop_show_driver_info(struct seq_file *seq);

/**  GUIDS for director channel type supported by this driver.
*/
static struct visor_channeltype_descriptor visornoop_channel_types[] = {
	/*  Note that the only channel type we expect to be reported by the
	 *  bus driver is the CONTROLDIRECTOR channel.
	 */
	{ ULTRA_CONTROLDIRECTOR_CHANNEL_PROTOCOL_GUID,
	  "controldirector", 1, ULONG_MAX },
	{ NULL_UUID_LE, NULL, 0, 0 }
};

/** This is used to tell the visor bus driver which types of visor devices
 *  we support, and what functions to call when a visor device that we support
 *  is attached or removed.
 */
static struct visor_driver visornoop_driver = {
	.name = MYDRVNAME,
	.version = VERSION,
	.vertag = NULL,
	.build_date = __DATE__,
	.build_time = __TIME__,
	.owner = THIS_MODULE,
	.channel_types = visornoop_channel_types,
	.probe = visornoop_probe,
	.remove = visornoop_remove,
	.pause = visornoop_pause,
	.resume = visornoop_resume,
	.channel_interrupt = NULL,
};

/** This is the private data that we store for each device.
 *  A pointer to this struct is kept in each "struct device", and can be
 *  obtained using visor_get_drvdata(dev).
 */
struct visornoop_devdata {
	int devno;
	struct visor_device *dev;
	/** lock for dev */
	struct rw_semaphore lockVisorDev;
	char name[99];
	struct list_head list_all;   /**< link within List_all_devices list */
	struct kref kref;
	struct easyproc_device_info procinfo;
};

/** List of all visornoop_devdata structs,
  * linked via the list_all member
  */
static LIST_HEAD(List_all_devices);
static DEFINE_SPINLOCK(Lock_all_devices);

static struct visornoop_devdata *devdata_create(struct visor_device *dev)
{
	void *rc = NULL;
	struct visornoop_devdata *devdata = NULL;
	int devno = -1;

	devdata = kmalloc(sizeof(struct visornoop_devdata),
			  GFP_KERNEL|__GFP_NORETRY);
	if (devdata == NULL) {
		ERRDRV("allocation of visornoop_devdata failed\n");
		goto Away;
	}
	memset(devdata, '\0', sizeof(struct visornoop_devdata));
	spin_lock(&devnopool_lock);
	devno = find_first_zero_bit(DevNoPool, MAXDEVICES);
	set_bit(devno, DevNoPool);
	spin_unlock(&devnopool_lock);
	if (devno == MAXDEVICES)
		devno = -1;
	if (devno < 0) {
		ERRDRV("unknown device\n");
		goto Away;
	}
	devdata->devno = devno;
	devdata->dev = dev;
	strncpy(devdata->name, dev_name(&dev->device), sizeof(devdata->name));
	init_rwsem(&devdata->lockVisorDev);
	kref_init(&devdata->kref);
	spin_lock(&Lock_all_devices);
	list_add_tail(&devdata->list_all, &List_all_devices);
	spin_unlock(&Lock_all_devices);
	rc = devdata;
Away:
	if (rc == NULL) {
		if (devno >= 0)
		{
			spin_lock(&devnopool_lock);
			clear_bit(devno, DevNoPool);
			spin_unlock(&devnopool_lock);
		}
		if (devdata != NULL)
			kfree(devdata);
	}
	return rc;
}

static void devdata_release(struct kref *mykref)
{
	struct visornoop_devdata *devdata =
		container_of(mykref, struct visornoop_devdata, kref);

	INFODRV("%s", __func__);
	spin_lock(&devnopool_lock);
	clear_bit(devdata->devno, DevNoPool);
	spin_unlock(&devnopool_lock);
	spin_lock(&Lock_all_devices);
	list_del(&devdata->list_all);
	spin_unlock(&Lock_all_devices);
	kfree(devdata);
	INFODRV("%s finished", __func__);
}

static int visornoop_probe(struct visor_device *dev)
{
	int rc = 0;
	struct visornoop_devdata *devdata = NULL;

	INFODRV("%s", __func__);
	devdata = devdata_create(dev);
	if (devdata == NULL) {
		rc = -1;
		goto Away;
	}
	visor_set_drvdata(dev, devdata);
	visor_easyproc_InitDevice(&Easyproc_driver_info,
				  &devdata->procinfo, devdata->devno, devdata);
Away:
	INFODRV("%s finished", __func__);
	if (rc < 0) {
		if (devdata != NULL)
			kref_put(&devdata->kref, devdata_release);
	}
	return rc;
}

static void host_side_disappeared(struct visornoop_devdata *devdata)
{
	LOCKWRITESEM(&devdata->lockVisorDev);
	sprintf(devdata->name, "<dev#%d-history>", devdata->devno);
	devdata->dev = NULL;   /* indicate device destroyed */
	UNLOCKWRITESEM(&devdata->lockVisorDev);
}

static void visornoop_remove(struct visor_device *dev)
{
	struct visornoop_devdata *devdata = visor_get_drvdata(dev);

	INFODRV("%s", __func__);
	if (devdata == NULL) {
		ERRDRV("no devdata in %s", __func__);
		goto Away;
	}
	visor_set_drvdata(dev, NULL);
	visor_easyproc_DeInitDevice(&Easyproc_driver_info,
				    &devdata->procinfo, devdata->devno);
	host_side_disappeared(devdata);
	kref_put(&devdata->kref, devdata_release);
Away:
	INFODRV("%s finished", __func__);
}

static int visornoop_pause(struct visor_device *dev,
				  VISORBUS_STATE_COMPLETE_FUNC complete_func)
{
	INFODEV(dev_name(&dev->device), "paused");
	complete_func(dev, 0);
	return 0;
}

static int visornoop_resume(struct visor_device *dev,
				   VISORBUS_STATE_COMPLETE_FUNC complete_func)
{
	INFODEV(dev_name(&dev->device), "resumed");
	complete_func(dev, 0);
	return 0;
}

static void visornoop_cleanup_guts(void)
{
	visorbus_unregister_visor_driver(&visornoop_driver);
	visor_easyproc_DeInitDriver(&Easyproc_driver_info);
	if (DevNoPool != NULL) {
		kfree(DevNoPool);
		DevNoPool = NULL;
	}
}

static int visornoop_init(void)
{
	int rc = 0;

	INFODRV("driver version %s loaded", VERSION);

	spin_lock_init(&devnopool_lock);
	DevNoPool = kzalloc(BITS_TO_LONGS(MAXDEVICES), GFP_KERNEL);
	if (DevNoPool == NULL) {
		ERRDRV("Unable to create DevNoPool");
		rc = -1;
		goto Away;
	}
	visor_easyproc_InitDriver(&Easyproc_driver_info,
			    MYDRVNAME,
			    visornoop_show_driver_info,
			    visornoop_show_device_info);
	visorbus_register_visor_driver(&visornoop_driver);

Away:
	if (rc < 0)
		visornoop_cleanup_guts();
	return rc;
}

static void visornoop_cleanup(void)
{
	visornoop_cleanup_guts();
	INFODRV("driver unloaded");
}

static void visornoop_show_device_info(struct seq_file *seq, void *p)
{
	struct visornoop_devdata *devdata = (struct visornoop_devdata *)(p);
	seq_printf(seq, "devno=%d\n", devdata->devno);
	seq_printf(seq, "visorbus name = '%s'\n", devdata->name);
}

static void visornoop_show_driver_info(struct seq_file *seq)
{
	seq_printf(seq, "Version=%s\n", VERSION);
}

module_init(visornoop_init);
module_exit(visornoop_cleanup);

MODULE_AUTHOR("Unisys");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sPAR noop driver for sparlinux: ver "
		   VERSION);
MODULE_VERSION(VERSION);
