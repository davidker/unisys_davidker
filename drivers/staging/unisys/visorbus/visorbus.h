/* visorbus.h
 *
 * Copyright (C) 2010 - 2013 UNISYS CORPORATION
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

/*
 *  This header file is to be included by other kernel mode components that
 *  implement a particular kind of visor_device.  Each of these other kernel
 *  mode components is called a visor device driver.  Refer to visortemplate
 *  for a minimal sample visor device driver.
 *
 *  There should be nothing in this file that is private to the visorbus
 *  bus implementation itself.
 *
 */

#ifndef __VISORBUS_H__
#define __VISORBUS_H__

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uuid.h>

#include "periodic_work.h"
#include "visorchannel.h"
#include "channel.h"

struct visor_driver;
struct visor_device;

typedef void (*VISORBUS_STATE_COMPLETE_FUNC) (struct visor_device *dev,
					      int status);

/** This struct describes a specific Supervisor channel, by providing its
 *  GUID, name, and sizes.
 */
struct visor_channeltype_descriptor {
	const uuid_le guid;
	const char *name;
	ulong min_size;
	ulong max_size;
};

/** Information provided by each visor driver when it registers with the
 *  visorbus driver.
 */
struct visor_driver {
	const char *name;
	const char *version;
	const char *vertag;
	const char *build_date;
	const char *build_time;
	struct module *owner;

	/** Types of channels handled by this driver, ending with 0 GUID.
	 *  Our specialized BUS.match() method knows about this list, and
	 *  uses it to determine whether this driver will in fact handle a
	 *  new device that it has detected.
	 */
	struct visor_channeltype_descriptor *channel_types;

	/** Called when a new device comes online, by our probe() function
	 *  specified by driver.probe() (triggered ultimately by some call
	 *  to driver_register() / bus_add_driver() / driver_attach()).
	 */
	int (*probe)(struct visor_device *dev);

	/** Called when a new device is removed, by our remove() function
	 *  specified by driver.remove() (triggered ultimately by some call
	 *  to device_release_driver()).
	 */
	void (*remove)(struct visor_device *dev);

	/** Called periodically, whenever there is a possibility that
	 *  "something interesting" may have happened to the channel state.
	 */
	void (*channel_interrupt)(struct visor_device *dev);

	/** Called to initiate a change of the device's state.  If the return
	 *  valu`e is < 0, there was an error and the state transition will NOT
	 *  occur.  If the return value is >= 0, then the state transition was
	 *  INITIATED successfully, and complete_func() will be called (or was
	 *  just called) with the final status when either the state transition
	 *  fails or completes successfully.
	 */
	int (*pause)(struct visor_device *dev,
		     VISORBUS_STATE_COMPLETE_FUNC complete_func);
	int (*resume)(struct visor_device *dev,
		      VISORBUS_STATE_COMPLETE_FUNC complete_func);

	/** These fields are for private use by the bus driver only. */
	struct device_driver driver;
	struct driver_attribute version_attr;
};

#define to_visor_driver(x) container_of(x, struct visor_driver, driver)

/** A device type for things "plugged" into the visorbus bus */

struct visor_device {
	/** visor driver can use the visorchannel member with the functions
	 *  defined in visorchannel.h to access the channel
	 */
	VISORCHANNEL *visorchannel;
	uuid_le channel_type_guid;
	u64 channel_bytes;

	/** These fields are for private use by the bus driver only.
	 *  A notable exception is that the visor driver can use
	 *  visor_get_drvdata() and visor_set_drvdata() to retrieve or stash
	 *  private visor driver specific data within the device member.
	 */
	struct device device;
	struct list_head list_all;
	struct periodic_work *periodic_work;
	BOOL being_removed;
	BOOL responded_to_device_create;
	struct kobject kobjchannel;	/* visorbus<x>/dev<y>/channel/ */
	struct kobject kobjdevmajorminor; /* visorbus<x>/dev<y>/devmajorminor/*/
	struct {
		int major, minor;
		void *attr;	/* private use by devmajorminor_attr.c you can
				   * change this constant to whatever you
				   * want; */
	} devnodes[5];
	/* the code will detect and behave appropriately) */
	struct semaphore visordriver_callback_lock;
	BOOL pausing;
	BOOL resuming;
	ulong chipset_bus_no;
	ulong chipset_dev_no;
};

#define to_visor_device(x) container_of(x, struct visor_device, device)

static inline void *
visor_get_drvdata(struct visor_device *dev)
{
	return dev_get_drvdata(&dev->device);
}

static inline void
visor_set_drvdata(struct visor_device *dev, void *data)
{
	dev_set_drvdata(&dev->device, data);
}

#ifndef STANDALONE_CLIENT
int visorbus_register_visor_driver(struct visor_driver *);
void visorbus_unregister_visor_driver(struct visor_driver *);
int visorbus_read_channel(struct visor_device *dev,
			  ulong offset, void *dest, ulong nbytes);
int visorbus_write_channel(struct visor_device *dev,
			   ulong offset, void *src, ulong nbytes);
int visorbus_clear_channel(struct visor_device *dev,
			   ulong offset, u8 ch, ulong nbytes);
int visorbus_registerdevnode(struct visor_device *dev,
			     const char *name, int major, int minor);
void visorbus_enable_channel_interrupts(struct visor_device *dev);
void visorbus_disable_channel_interrupts(struct visor_device *dev);
#endif

/* Reference counting interfaces */
#define VISORBUS_DEBUG_REFCOUNT_CHANGE(old, new, p, why)                \
	INFODRV("refcount:%d-->%d %p <<%s>>", old, new, p, why)

#define VISORBUS_DEBUG_REFCOUNT(count, p, why)                         \
	INFODRV("refcount:%d %p <<%s>>", count, p, why)

#define get_visordev(/*struct visor_device **/dev, /* char * */why, DBG) \
do {							     \
	int refcount;						     \
	get_device(&dev->device);				     \
	refcount = atomic_read(&dev->device.kobj.kref.refcount);     \
	if (DBG)				     \
		VISORBUS_DEBUG_REFCOUNT_CHANGE			     \
			(refcount-1, refcount, &dev->device, why);   \
} while (0)

#define put_visordev(/*struct visor_device **/dev, /* char * */why, DBG) \
do {							     \
	int refcount;						     \
	put_device(&dev->device);				     \
	refcount = atomic_read(&dev->device.kobj.kref.refcount);     \
	if (DBG)				     \
		VISORBUS_DEBUG_REFCOUNT_CHANGE			     \
			(refcount+1, refcount, &dev->device, why);   \
} while (0)

#define refcount_debug(/*struct visor_device **/dev, /* char * */why) \
do {							       \
	if (visorbus_debugref)				       \
		VISORBUS_DEBUG_REFCOUNT				       \
			(atomic_read(&dev->device.kobj.kref.refcount), \
			 &dev->device, why);                           \
} while (0)

#endif
