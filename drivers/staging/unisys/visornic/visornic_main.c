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
#include "controlframework.h"
#include "channel_guid.h"

static spinlock_t dev_no_pool_lock;
static void *dev_no_pool;	/**< pool to grab device numbers from */

static int visornic_probe(struct visor_device *dev);
static void visornic_remove(struct visor_device *dev);
static int visornic_pause(struct visor_device *dev,
			  VISORBUS_STATE_COMPLETE_FUNC complete_func);
static int visornic_resume(struct visor_device *dev,
			   VISORBUS_STATE_COMPLETE_FUNC complete_func);

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
struct visornic_devdata {
	int devno;
	struct visor_device *dev;
	/** lock for dev */
	struct rw_semaphore lock_visor_dev;
	char name[99];
	struct list_head list_all;   /**< link within list_all_devices list */
	struct kref kref;
};

/** List of all visornic_devdata structs,
  * linked via the list_all member
  */
static LIST_HEAD(list_all_devices);
static DEFINE_SPINLOCK(lock_all_devices);

static struct visornic_devdata *devdata_create(struct visor_device *dev)
{
	struct visornic_devdata *devdata = NULL;
	int devno = -1;

	devdata = kmalloc(sizeof(*devdata),
			  GFP_KERNEL|__GFP_NORETRY);
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

static int visornic_probe(struct visor_device *dev)
{
	struct visornic_devdata *devdata = NULL;

	INFODRV("%s", __func__);
	devdata = devdata_create(dev);
	if (!devdata)
		return -1;
	visor_set_drvdata(dev, devdata);
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
	return fifty;
}


module_init(visornic_init);
module_exit(visornic_cleanup);

MODULE_AUTHOR("Unisys");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sPAR nic driver for sparlinux: ver "
		   VERSION);
MODULE_VERSION(VERSION);
