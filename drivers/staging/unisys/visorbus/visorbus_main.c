/* visorbus_main.c
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

#include <linux/uuid.h>

#include "visorbus_private.h"
#include "businst_attr.h"
#include "channel_attr.h"
#include "devmajorminor_attr.h"
#include "periodic_work.h"
#include "consolechannel.h"	/* for serialloopbacktest */
#include "vbuschannel.h"
#include "guestlinuxdebug.h"
#include "vbusdeviceinfo.h"
/* These forward declarations are required since our drivers are out-of-tree.
 * The structures referenced are kernel-private and are not in the headers, but
 * it is impossible to make a functioning bus driver without them.
 */
struct subsys_private {
	struct kset subsys;
	struct kset *devices_kset;

	struct kset *drivers_kset;
	struct klist klist_devices;
	struct klist klist_drivers;
	struct blocking_notifier_head bus_notifier;
	unsigned int drivers_autoprobe:1;
	struct bus_type *bus;

	struct list_head class_interfaces;
	struct kset glue_dirs;
	struct mutex class_mutex; /* ignore */
	struct class *class;
};

struct bus_type_private {
	struct kset subsys;
	struct kset *drivers_kset;
	struct kset *devices_kset;
	struct klist klist_devices;
	struct klist klist_drivers;
	struct blocking_notifier_head bus_notifier;
	unsigned int drivers_autoprobe:1;
	struct bus_type *bus;
};

#define CURRENT_FILE_PC VISOR_BUS_PC_visorbus_main_c
#define POLLJIFFIES_TESTWORK         100
#define POLLJIFFIES_NORMALCHANNEL     10

static int visorbus_uevent(struct device *xdev, struct kobj_uevent_env *env);
static int visorbus_match(struct device *xdev, struct device_driver *xdrv);
static void fix_vbus_dev_info(struct visor_device *visordev);

/** This describes the TYPE of bus.
 *  (Don't confuse this with an INSTANCE of the bus.)
 */
static struct bus_type visorbus_type = {
	.name = "visorbus",
	.match = visorbus_match,
	.uevent = visorbus_uevent,
};

static struct delayed_work periodic_work;

/* YES, we need 2 workqueues.
 * The reason is, workitems on the test queue may need to cancel
 * workitems on the other queue.  You will be in for trouble if you try to
 * do this with workitems queued on the same workqueue.
 */
static struct workqueue_struct *periodic_test_workqueue;
static struct workqueue_struct *periodic_dev_workqueue;
static long long bus_count;	/** number of bus instances */
static long long total_devices_created;
					/** ever-increasing */

static void chipset_bus_create(ulong bus_no);
static void chipset_bus_destroy(ulong bus_no);
static void chipset_device_create(ulong bus_no, ulong dev_no);
static void chipset_device_destroy(ulong bus_no, ulong dev_no);
static void chipset_device_pause(ulong bus_no, ulong dev_no);
static void chipset_device_resume(ulong bus_no, ulong dev_no);
static int chipset_get_channel_info(uuid_le type_guid, ulong *min_size,
				    ulong *max_size);

/** These functions are implemented herein, and are called by the chipset
 *  driver to notify us about specific events.
 */
static VISORCHIPSET_BUSDEV_NOTIFIERS chipset_notifiers = {
	.bus_create = chipset_bus_create,
	.bus_destroy = chipset_bus_destroy,
	.device_create = chipset_device_create,
	.device_destroy = chipset_device_destroy,
	.device_pause = chipset_device_pause,
	.device_resume = chipset_device_resume,
	.get_channel_info = chipset_get_channel_info,
};

/** These functions are implemented in the chipset driver, and we call them
 *  herein when we want to acknowledge a specific event.
 */
static VISORCHIPSET_BUSDEV_RESPONDERS chipset_responders;

/* filled in with info about parent chipset driver when we register with it */
static struct ultra_vbus_deviceinfo chipset_driverinfo;
/* filled in with info about this driver, wrt it servicing client busses */
static struct ultra_vbus_deviceinfo clientbus_driverinfo;

/** list of visorbus_devdata structs, linked via .list_all */
static LIST_HEAD(list_all_bus_instances);
static struct visorbus_devdata *devdata;	/* for testing ONLY */
/** list of visor_device structs, linked via .list_all */
static LIST_HEAD(list_all_device_instances);

static int
visorbus_uevent(struct device *xdev, struct kobj_uevent_env *env)
{
	if (add_uevent_var(env, "VERSION=%s", VERSION))
		return -ENOMEM;
	return 0;
}

/* This is called automatically upon adding a visor_device (device_add), or
 * adding a visor_driver (visorbus_register_visor_driver), and returns 1 iff the
 * provided driver can control the specified device.
 */
static int
visorbus_match(struct device *xdev, struct device_driver *xdrv)
{
	uuid_le channel_type;
	int rc = 0;
	int i;
	struct visor_device *dev;
	struct visor_driver *drv;

	dev = to_visor_device(xdev);
	drv = to_visor_driver(xdrv);
	channel_type = visorchannel_get_uuid(dev->visorchannel);
	if (visorbus_forcematch) {
		rc = 1;
		goto away;
	}
	if (visorbus_forcenomatch)
		goto away;

	if (drv->channel_types == NULL)
		goto away;
	for (i = 0;
	     (uuid_le_cmp(drv->channel_types[i].guid, NULL_UUID_LE) != 0) ||
	     (drv->channel_types[i].name == NULL);
	     i++)
		if (uuid_le_cmp(drv->channel_types[i].guid,
				channel_type) == 0) {
			rc = i + 1;
			goto away;
		}
away:
	return rc;
}

/** This is called when device_unregister() is called for the bus device
 *  instance, after all other tasks involved with destroying the device
 *  are complete.
 */
static void
visorbus_release_busdevice(struct device *xdev)
{
	struct visorbus_devdata *devdata = dev_get_drvdata(xdev);

	dev_set_drvdata(xdev, NULL);
	kfree(devdata);
	INFODEV(dev_name(xdev), "bus device destroyed - freeing up memory now");
	kfree(xdev);
}

/** This is called when device_unregister() is called for each child
 *  device instance.
 */
static void
visorbus_release_device(struct device *xdev)
{
	char s[99];
	struct visor_device *dev = to_visor_device(xdev);

	INFODEV(dev_name(xdev),
		"child device destroyed - freeing up memory now");
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
	kfree(dev);
}

static const struct sysfs_ops businst_sysfs_ops = {
	.show = businst_attr_show,
	.store = businst_attr_store,
};

static struct kobj_type businst_kobj_type = {
	.sysfs_ops = &businst_sysfs_ops
};

static struct kset businstances = { /* should actually be a member of
				     * bus_type */
};

/*  BUS type attributes
 *
 *  define & implement display of bus attributes under
 *  /sys/bus/visorbus.
 *
 */

static ssize_t
BUSTYPE_ATTR_version(struct bus_type *bus, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VERSION);
}

static struct bus_attribute bustype_attr_version =
__ATTR(version, S_IRUGO, BUSTYPE_ATTR_version, NULL);

static int
register_bustype_attributes(void)
{
	int rc = 0;

	rc = bus_create_file(&visorbus_type, &bustype_attr_version);
	if (rc < 0) {
		ERRDRV("bus_create_file(&Visorbus_type, &bustype_attr_version) failed: (status=%d)\n", rc);
		goto away;
	}
	/* Here we make up for the fact that bus_type does not yet have a
	 * member to keep track of multiple bus instances for a given bus
	 * type.  This is useful for stashing properties for each bus
	 * instance.
	 */
	kobject_set_name(&businstances.kobj, "busses");
	businstances.kobj.ktype = &businst_kobj_type;
	businstances.kobj.parent = &visorbus_type.p->subsys.kobj;
	rc = kset_register(&businstances);
	if (rc < 0) {
		ERRDRV("kset_register(&businstances) failed: (status=%d)\n",
		       rc);
		goto away;
	}
	rc = 0;
away:
	return rc;
}

static void
unregister_bustype_attributes(void)
{
	bus_remove_file(&visorbus_type, &bustype_attr_version);
	kset_unregister(&businstances);
}

/*  BUS instance attributes
 *
 *  define & implement display of bus attributes under
 *  /sys/bus/visorbus/busses/visorbus<n>.
 *
 *  This is a bit hoaky because the kernel does not yet have the infrastructure
 *  to separate bus INSTANCE attributes from bus TYPE attributes...
 *  so we roll our own.  See businst.c / businst.h.
 *
 */

static ssize_t businst_attr_partition_handle(struct visorbus_devdata *businst,
					     char *buf) {
	VISORCHIPSET_BUS_INFO bus_info;
	int len = 0;

	if (businst && visorchipset_get_bus_info(businst->devno, &bus_info))
		len = snprintf(buf, PAGE_SIZE,
			       "0x%Lx\n",
			       (unsigned long long)bus_info.partitionHandle);
	return len;
}

static ssize_t businst_attr_partition_guid(struct visorbus_devdata *businst,
					   char *buf) {
	VISORCHIPSET_BUS_INFO bus_info;
	int len = 0;

	if (businst && visorchipset_get_bus_info(businst->devno, &bus_info))
		len = snprintf(buf, PAGE_SIZE, "{%pUb}\n",
			       &bus_info.partitionGuid);
	return len;
}

static ssize_t businst_attr_partition_name(struct visorbus_devdata *businst,
					   char *buf) {
	VISORCHIPSET_BUS_INFO bus_info;
	int len = 0;

	if (businst &&
	    visorchipset_get_bus_info(businst->devno, &bus_info) &&
	    bus_info.name)
		len = snprintf(buf, PAGE_SIZE, "%s\n", bus_info.name);
	return len;
}

static ssize_t businst_attr_channel_addr(struct visorbus_devdata *businst,
					 char *buf) {
	VISORCHIPSET_BUS_INFO bus_info;
	int len = 0;

	if (businst && visorchipset_get_bus_info(businst->devno, &bus_info))
		len = snprintf(buf, PAGE_SIZE, "0x%Lx\n", (unsigned long long)
			       bus_info.chanInfo.channelAddr);
	return len;
}

static ssize_t businst_attr_nchannel_bytes(struct visorbus_devdata *businst,
					   char *buf) {
	VISORCHIPSET_BUS_INFO bus_info;
	int len = 0;

	if (businst && visorchipset_get_bus_info(businst->devno, &bus_info))
		len = snprintf(buf, PAGE_SIZE, "0x%Lx\n", (unsigned long long)
			       bus_info.chanInfo.nChannelBytes);
	return len;
}

static ssize_t businst_attr_channel_id(struct visorbus_devdata *businst,
				       char *buf) {
	int len = 0;

	if (businst && businst->chan) {
		visorchannel_id(businst->chan, buf);
		len = strlen(buf);
		buf[len++] = '\n';
	}
	return len;
}

static ssize_t businst_attr_client_bus_info(struct visorbus_devdata *businst,
					    char *buf) {
	VISORCHIPSET_BUS_INFO bus_info;
	int i, x, remain = PAGE_SIZE;
	ulong off;
	char *p = buf;
	u8 *partition_name;
	struct ultra_vbus_deviceinfo dev_info;

	partition_name = "";
	if (businst && businst->chan) {
		if (visorchipset_get_bus_info(businst->devno, &bus_info) &&
		    bus_info.name)
			partition_name = bus_info.name;
		x = snprintf(p, remain,
			     "Client device / client driver info for %s partition (vbus #%d):\n",
			     partition_name, businst->devno);
		p += x;
		remain -= x;
		x = visorchannel_read(businst->chan,
				      offsetof(struct
					       ultra_vbus_channel_protocol,
					       ChpInfo),
				      &dev_info, sizeof(dev_info));
		if (x >= 0) {
			x = vbuschannel_devinfo_to_string(&dev_info, p,
							  remain, -1);
			p += x;
			remain -= x;
		}
		x = visorchannel_read(businst->chan,
				      offsetof(struct
					       ultra_vbus_channel_protocol,
					       BusInfo),
				      &dev_info, sizeof(dev_info));
		if (x >= 0) {
			x = vbuschannel_devinfo_to_string(&dev_info, p,
							  remain, -1);
			p += x;
			remain -= x;
		}
		off = offsetof(struct ultra_vbus_channel_protocol, DevInfo);
		i = 0;
		while (off + sizeof(dev_info) <=
		       visorchannel_get_nbytes(businst->chan)) {
			x = visorchannel_read(businst->chan,
					      off, &dev_info, sizeof(dev_info));
			if (x >= 0) {
				x = vbuschannel_devinfo_to_string
				    (&dev_info, p, remain, i);
				p += x;
				remain -= x;
			}
			off += sizeof(dev_info);
			i++;
		}
	}
	return PAGE_SIZE - remain;
}

static struct businst_attribute ba_partition_handle =
	__ATTR(partition_handle, S_IRUGO, businst_attr_partition_handle, NULL);
static struct businst_attribute ba_partition_guid =
	__ATTR(partition_guid, S_IRUGO, businst_attr_partition_guid, NULL);
static struct businst_attribute ba_partition_name =
	__ATTR(partition_name, S_IRUGO, businst_attr_partition_name, NULL);
static struct businst_attribute ba_channel_addr =
	__ATTR(channel_addr, S_IRUGO, businst_attr_channel_addr, NULL);
static struct businst_attribute ba_nchannel_bytes =
	__ATTR(nchannel_bytes, S_IRUGO, businst_attr_nchannel_bytes, NULL);
static struct businst_attribute ba_channel_id =
	__ATTR(channel_id, S_IRUGO, businst_attr_channel_id, NULL);
static struct businst_attribute ba_client_bus_info =
	__ATTR(client_bus_info, S_IRUGO, businst_attr_client_bus_info, NULL);

static int
register_businst_attributes(struct visorbus_devdata *businst)
{
	int rc = 0;

	businst->kobj.kset = &businstances;	/* identify parent sysfs dir */
	rc = kobject_init_and_add(&businst->kobj, &businst_kobj_type,
				  NULL, "visorbus%d", businst->devno);
	if (rc < 0) {
		ERRDRV("kobject_init_and_add() failed: (status=%d)\n", rc);
		goto away;
	}

	rc = businst_create_file(businst, &ba_partition_handle);
	if (rc < 0) {
		ERRDRV("businst_create_file(ba_partition_handle) failed: (status=%d)\n",
		       rc);
		goto away;
	}
	rc = businst_create_file(businst, &ba_partition_guid);
	if (rc < 0) {
		ERRDRV("businst_create_file(ba_partition_guid) failed: (status=%d)\n",
		       rc);
		goto away;
	}
	rc = businst_create_file(businst, &ba_partition_name);
	if (rc < 0) {
		ERRDRV("businst_create_file(ba_partition_name) failed: (status=%d)\n",
		       rc);
		goto away;
	}
	rc = businst_create_file(businst, &ba_channel_addr);
	if (rc < 0) {
		ERRDRV("businst_create_file(ba_channel_addr) failed: (status=%d)\n",
		       rc);
		goto away;
	}
	rc = businst_create_file(businst, &ba_nchannel_bytes);
	if (rc < 0) {
		ERRDRV("businst_create_file(ba_nchannel_bytes) failed: (status=%d)\n",
		       rc);
		goto away;
	}
	rc = businst_create_file(businst, &ba_channel_id);
	if (rc < 0) {
		ERRDRV("businst_create_file(ba_channel_id) failed: (status=%d)\n",
		       rc);
		goto away;
	}
	rc = businst_create_file(businst, &ba_client_bus_info);
	if (rc < 0) {
		ERRDRV("businst_create_file(ba_client_bus_info) failed: (status=%d)\n",
		       rc);
		goto away;
	}

	kobject_uevent(&businst->kobj, KOBJ_ADD);

	rc = 0;
away:
	return rc;
}

static void
unregister_businst_attributes(struct visorbus_devdata *businst)
{
	businst_remove_file(businst, &ba_partition_handle);
	businst_remove_file(businst, &ba_partition_guid);
	businst_remove_file(businst, &ba_partition_name);
	businst_remove_file(businst, &ba_channel_addr);
	businst_remove_file(businst, &ba_nchannel_bytes);
	businst_remove_file(businst, &ba_channel_id);
	businst_remove_file(businst, &ba_client_bus_info);
	kobject_put(&businst->kobj);
}

/*  DRIVER attributes
 *
 *  define & implement display of driver attributes under
 *  /sys/bus/visorbus/drivers/<drivername>.
 *
 */

static ssize_t
DRIVER_ATTR_version(struct device_driver *xdrv, char *buf)
{
	struct visor_driver *drv = to_visor_driver(xdrv);

	return snprintf(buf, PAGE_SIZE, "%s\n", drv->version);
}

static int
register_driver_attributes(struct visor_driver *drv)
{
	int rc;
	struct driver_attribute version =
	    __ATTR(version, S_IRUGO, DRIVER_ATTR_version, NULL);
	drv->version_attr = version;
	rc = driver_create_file(&drv->driver, &drv->version_attr);
	if (rc < 0) {
		ERRDRV("driver_create_file(&drv->driver, &drv->version_attr) failed: (status=%d)\n",
		       rc);
		goto away;
	}
	rc = 0;
away:
	return rc;
}

static void
unregister_driver_attributes(struct visor_driver *drv)
{
	driver_remove_file(&drv->driver, &drv->version_attr);
}

/*  DEVICE attributes
 *
 *  define & implement display of device attributes under
 *  /sys/bus/visorbus/devices/<devicename>.
 *
 */

#define DEVATTR(nam, func) { \
	.attr = { .name = __stringify(nam), \
		  .mode = 0444, \
		  .owner = THIS_MODULE },	\
	.show = func, \
}

static struct device_attribute visor_device_attrs[] = {
	/* DEVATTR(channel_nbytes, DEVICE_ATTR_channel_nbytes), */
	__ATTR_NULL
};

static void
dev_periodic_work(void *xdev)
{
	struct visor_device *dev = (struct visor_device *)xdev;
	struct visor_driver *drv = to_visor_driver(dev->device.driver);

	down(&dev->visordriver_callback_lock);
	if (drv->channel_interrupt)
		drv->channel_interrupt(dev);
	up(&dev->visordriver_callback_lock);
	if (!visor_periodic_work_nextperiod(dev->periodic_work))
		put_visordev(dev, "delayed work", visorbus_debugref);
}

static void
dev_start_periodic_work(struct visor_device *dev)
{
	if (dev->being_removed)
		return;
	/* now up by at least 2 */
	get_visordev(dev, "delayed work", visorbus_debugref);
	if (!visor_periodic_work_start(dev->periodic_work))
		put_visordev(dev, "delayed work", visorbus_debugref);
}

static void
dev_stop_periodic_work(struct visor_device *dev)
{
	if (visor_periodic_work_stop(dev->periodic_work))
		put_visordev(dev, "delayed work", visorbus_debugref);
}

/** This is called automatically upon adding a visor_device (device_add), or
 *  adding a visor_driver (visorbus_register_visor_driver), but only after
 *  visorbus_match has returned 1 to indicate a successful match between
 *  driver and device.
 */
static int
visordriver_probe_device(struct device *xdev)
{
	int rc;
	struct visor_driver *drv;
	struct visor_device *dev;

	drv = to_visor_driver(xdev->driver);
	dev = to_visor_device(xdev);
	down(&dev->visordriver_callback_lock);
	dev->being_removed = FALSE;
	/*
	 * ensure that the dev->being_removed flag is cleared before
	 * we start the probe
	 */
	wmb();
	get_visordev(dev, "probe", visorbus_debugref);
	if (!drv->probe) {
		up(&dev->visordriver_callback_lock);
		ERRDEV(dev_name(&dev->device),
		       "driver did not specify probe func");
		rc = -1;
		goto away;
	}
	rc = drv->probe(dev);
	if (rc < 0) {
		ERRDRV("drv->probe(dev) failed: (status=%d)\n", rc);
		goto away;
	}
	fix_vbus_dev_info(dev);
	up(&dev->visordriver_callback_lock);
	rc = 0;
away:
	if (rc == 0) {
		INFODEV(dev_name(&dev->device), "child device probed");
		/* device ref count is now up by one (get_device) */
	} else {
		put_visordev(dev, "probe", visorbus_debugref);
		ERRDEV(dev_name(&dev->device), "child device probed failed");
	}
	/*  We could get here more than once if the child driver module is
	 *  unloaded and re-loaded while devices are present.  That's why we
	 *  need a flag to be sure that we only respond to the device_create
	 *  once.  We cannot respond to the device_create prior to here,
	 *  because until we call drv->probe() above, the channel has not been
	 *  initialized.
	 */
	if (!dev->responded_to_device_create) {
		dev->responded_to_device_create = TRUE;
		if (chipset_responders.device_create)
			(*chipset_responders.device_create)(dev->chipset_bus_no,
							    dev->chipset_dev_no,
							    rc);
	}
	return rc;
}

/** This is called when device_unregister() is called for each child device
 *  instance, to notify the appropriate visorbus_driver that the device is
 *  going away, and to decrease the reference count of the device.
 */
static int
visordriver_remove_device(struct device *xdev)
{
	int rc = 0;
	struct visor_device *dev;
	struct visor_driver *drv;

	dev = to_visor_device(xdev);
	drv = to_visor_driver(xdev->driver);
	down(&dev->visordriver_callback_lock);
	dev->being_removed = TRUE;
	/*
	 * ensure that the dev->being_removed flag is set before we start the
	 * actual removal
	 */
	wmb();
	if (drv) {
		INFODEV(dev_name(&dev->device),
			"detaching driver from child device");
		if (drv->remove)
			drv->remove(dev);
	} else
		INFODEV(dev_name(&dev->device),
			"no need to detach driver from child device");
	up(&dev->visordriver_callback_lock);
	dev_stop_periodic_work(dev);
	devmajorminor_remove_all_files(dev);

	put_visordev(dev, "probe", visorbus_debugref);

	return rc;
}

/** A particular type of visor driver calls this function to register
 *  the driver.  The caller MUST fill in the following fields within the
 *  #drv structure:
 *      name, version, owner, channel_types, probe, remove
 *
 *  Here's how the whole Linux bus / driver / device model works.
 *
 *  At system start-up, the visorbus kernel module is loaded, which registers
 *  visorbus_type as a bus type, using bus_register().
 *
 *  All kernel modules that support particular device types on a
 *  visorbus bus are loaded.  Each of these kernel modules calls
 *  visorbus_register_visor_driver() in their init functions, passing a
 *  visor_driver struct.  visorbus_register_visor_driver() in turn calls
 *  register_driver(&visor_driver.driver).  This .driver member is
 *  initialized with generic methods (like probe), whose sole responsibility
 *  is to act as a broker for the real methods, which are within the
 *  visor_driver struct.  (This is the way the subclass behavior is
 *  implemented, since visor_driver is essentially a subclass of the
 *  generic driver.)  Whenever a driver_register() happens, core bus code in
 *  the kernel does (see device_attach() in drivers/base/dd.c):
 *
 *      for each dev associated with the bus (the bus that driver is on) that
 *      does not yet have a driver
 *          if bus.match(dev,newdriver) == yes_matched  ** .match specified
 *                                                 ** during bus_register().
 *              newdriver.probe(dev)  ** for visor drivers, this will call
 *                    ** the generic driver.probe implemented in visorbus.c,
 *                    ** which in turn calls the probe specified within the
 *                    ** struct visor_driver (which was specified by the
 *                    ** actual device driver as part of
 *                    ** visorbus_register_visor_driver()).
 *
 *  The above dance also happens when a new device appears.
 *  So the question is, how are devices created within the system?
 *  Basically, just call device_add(dev).  See pci_bus_add_devices().
 *  pci_scan_device() shows an example of how to build a device struct.  It
 *  returns the newly-created struct to pci_scan_single_device(), who adds it
 *  to the list of devices at PCIBUS.devices.  That list of devices is what
 *  is traversed by pci_bus_add_devices().
 *
 */
int visorbus_register_visor_driver(struct visor_driver *drv)
{
	int rc = 0;

	INFODRV("child device driver %s loaded", drv->name);
	drv->driver.name = drv->name;
	drv->driver.bus = &visorbus_type;
	drv->driver.probe = visordriver_probe_device;
	drv->driver.remove = visordriver_remove_device;
	drv->driver.owner = drv->owner;

	/* driver_register does this:
	 *   bus_add_driver(drv)
	 *   ->if (drv.bus)  ** (bus_type) **
	 *       driver_attach(drv)
	 *         for each dev with bus type of drv.bus
	 *           if (!dev.drv)  ** no driver assigned yet **
	 *             if (bus.match(dev,drv))  [visorbus_match]
	 *               dev.drv = drv
	 *               if (!drv.probe(dev))   [visordriver_probe_device]
	 *                 dev.drv = NULL
	 */

	rc = driver_register(&drv->driver);
	if (rc < 0) {
		ERRDRV("driver_register(&drv->driver) failed: (status=%d)\n",
		       rc);
		goto away;
	}
	rc = register_driver_attributes(drv);
	if (rc < 0) {
		ERRDRV("register_driver_attributes(drv) failed: (status=%d)\n",
		       rc);
		goto away;
	}

away:

	if (rc)
		ERRDRV("visorbus_register_visor_driver failed");
	return rc;
}
EXPORT_SYMBOL_GPL(visorbus_register_visor_driver);

/** A particular type of visor driver calls this function to unregister
 *  the driver, i.e., within its module_exit function.
 */
void
visorbus_unregister_visor_driver(struct visor_driver *drv)
{
	INFODRV("child device driver %s unloaded", drv->name);
	unregister_driver_attributes(drv);
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(visorbus_unregister_visor_driver);

int
visorbus_read_channel(struct visor_device *dev, ulong offset, void *dest,
		      ulong nbytes)
{
	return visorchannel_read(dev->visorchannel, offset, dest, nbytes);
}
EXPORT_SYMBOL_GPL(visorbus_read_channel);

int
visorbus_write_channel(struct visor_device *dev, ulong offset, void *src,
		       ulong nbytes)
{
	return visorchannel_write(dev->visorchannel, offset, src, nbytes);
}
EXPORT_SYMBOL_GPL(visorbus_write_channel);

int
visorbus_clear_channel(struct visor_device *dev, ulong offset, u8 ch,
		       ulong nbytes)
{
	return visorchannel_clear(dev->visorchannel, offset, ch, nbytes);
}
EXPORT_SYMBOL_GPL(visorbus_clear_channel);

int
visorbus_registerdevnode(struct visor_device *dev,
			 const char *name, int major, int minor)
{
	return devmajorminor_create_file(dev, name, major, minor);
}
EXPORT_SYMBOL_GPL(visorbus_registerdevnode);

/** We don't really have a real interrupt, so for now we just call the
 *  interrupt function periodically...
 */
void
visorbus_enable_channel_interrupts(struct visor_device *dev)
{
	dev_start_periodic_work(dev);
}
EXPORT_SYMBOL_GPL(visorbus_enable_channel_interrupts);

void
visorbus_disable_channel_interrupts(struct visor_device *dev)
{
	dev_stop_periodic_work(dev);
}
EXPORT_SYMBOL_GPL(visorbus_disable_channel_interrupts);

/** This is how everything starts from the device end.
 *  This function is called when a channel first appears via a ControlVM
 *  message.  In response, this function allocates a visor_device to
 *  correspond to the new channel, and attempts to connect it the appropriate
 *  driver.  If the appropriate driver is found, the visor_driver.probe()
 *  function for that driver will be called, and will be passed the new
 *  visor_device that we just created.
 *
 *  It's ok if the appropriate driver is not yet loaded, because in that case
 *  the new device struct will just stick around in the bus' list of devices.
 *  When the appropriate driver calls visorbus_register_visor_driver(), the
 *  visor_driver.probe() for the new driver will be called with the new
 *  device.
 */
static int
create_visor_device(struct visorbus_devdata *devdata,
		    ulong chipset_bus_no, ulong chipset_dev_no,
		    VISORCHIPSET_CHANNEL_INFO chanInfo, u64 partition_handle)
{
	int rc = -1;
	VISORCHANNEL *visorchannel = NULL;
	struct visor_device *dev = NULL;
	BOOL gotten = FALSE, registered1 = FALSE, registered2 = FALSE;
	char s[99];

	POSTCODE_LINUX_4(DEVICE_CREATE_ENTRY_PC, chipset_dev_no, chipset_bus_no,
			 POSTCODE_SEVERITY_INFO);
	/* prepare chan_hdr (abstraction to read/write channel memory) */
	INFODRV("Channel discovered (addr=0x%-16.16llx, size=%llu)",
		(unsigned long long)chanInfo.channelAddr,
		(unsigned long long)chanInfo.nChannelBytes);
	visorchannel = visorchannel_create(chanInfo.channelAddr,
					   (ulong)chanInfo.nChannelBytes,
					   chanInfo.channelTypeGuid);
	if (visorchannel == NULL) {
		ERRDRV("channel addr = 0x%-16.16llx, size = %llu",
		       (unsigned long long)chanInfo.channelAddr,
		       (unsigned long long)chanInfo.nChannelBytes);

		ERRDRV("visorchannel_create failed: (status = -1)\n");
		POSTCODE_LINUX_3(DEVICE_CREATE_FAILURE_PC, chipset_dev_no,
				 DIAG_SEVERITY_ERR);
		goto away;
	}
	INFODRV("Channel %s connected (addr=0x%-16.16llx, size=%llu)",
		visorchannel_id(visorchannel, s),
		(unsigned long long)chanInfo.channelAddr,
		(unsigned long long)chanInfo.nChannelBytes);

	dev = kmalloc(sizeof(*dev), GFP_KERNEL|__GFP_NORETRY);
	if (dev == NULL) {
		ERRDRV("failed to allocate visor_device: (status = -1)\n");
		POSTCODE_LINUX_3(DEVICE_CREATE_FAILURE_PC, chipset_dev_no,
				 DIAG_SEVERITY_ERR);
		goto away;
	}

	memset(dev, 0, sizeof(struct visor_device));
	dev->visorchannel = visorchannel;
	dev->channel_type_guid = chanInfo.channelTypeGuid;
	dev->channel_bytes = chanInfo.nChannelBytes;
	dev->chipset_bus_no = chipset_bus_no;
	dev->chipset_dev_no = chipset_dev_no;
	dev->device.parent = devdata->dev;
	sema_init(&dev->visordriver_callback_lock, 1);	/* unlocked */
	dev->device.bus = &visorbus_type;
	device_initialize(&dev->device);
	dev->device.release = visorbus_release_device;
	/* keep a reference just for us (now 2) */
	get_visordev(dev, "create", visorbus_debugref);
	gotten = TRUE;
	dev->periodic_work =
		visor_periodic_work_create(POLLJIFFIES_NORMALCHANNEL,
					   periodic_dev_workqueue,
					   dev_periodic_work,
					   dev, dev_name(&dev->device));
	if (dev->periodic_work == NULL) {
		ERRDRV("failed to create periodic_work: (status = -1)\n");
		POSTCODE_LINUX_3(DEVICE_CREATE_FAILURE_PC, chipset_dev_no,
				 DIAG_SEVERITY_ERR);
		goto away;
	}

	/* bus_id must be a unique name with respect to this bus TYPE
	 * (NOT bus instance).  That's why we need to include the bus
	 * number within the name.
	 */
	dev_set_name(&dev->device, "vbus%lu:dev%lu",
		     chipset_bus_no, chipset_dev_no);

	/*  device_add does this:
	 *    bus_add_device(dev)
	 *    ->device_attach(dev)
	 *      ->for each driver drv registered on the bus that dev is on
	 *          if (dev.drv)  **  device already has a driver **
	 *            ** not sure we could ever get here... **
	 *          else
	 *            if (bus.match(dev,drv)) [visorbus_match]
	 *              dev.drv = drv
	 *              if (!drv.probe(dev))  [visordriver_probe_device]
	 *                dev.drv = NULL
	 *
	 *  Note that device_add does NOT fail if no driver failed to
	 *  claim the device.  The device will be linked onto
	 *  bus_type.klist_devices regardless (use bus_for_each_dev).
	 */
	rc = device_add(&dev->device);
	if (rc < 0) {
		ERRDRV("device_add(&dev->device) failed: (status = %d)\n", rc);
		POSTCODE_LINUX_3(DEVICE_ADD_PC, chipset_bus_no,
				 DIAG_SEVERITY_ERR);
		goto away;
	}

	/* note: device_register is simply device_initialize + device_add */
	refcount_debug(dev, "after device_add");

	rc = register_channel_attributes(dev);
	if (rc < 0) {
		ERRDRV("register_channel_attributes(dev) failed: (status = %d)\n",
		       rc);
		POSTCODE_LINUX_3(DEVICE_REGISTER_FAILURE_PC, chipset_dev_no,
				 DIAG_SEVERITY_ERR);
		goto away;
	}

	refcount_debug(dev, "after register_channel_attributes");
	registered1 = TRUE;

	rc = register_devmajorminor_attributes(dev);
	if (rc < 0) {
		ERRDRV("register_devmajorminor_attributes(dev) failed: (status = %d)\n",
		       rc);
		POSTCODE_LINUX_3(DEVICE_REGISTER_FAILURE_PC, chipset_dev_no,
				 DIAG_SEVERITY_ERR);
		goto away;
	}

	refcount_debug(dev, "after register_devmajorminor_attributes");
	registered2 = TRUE;

	INFODEV(dev_name(&dev->device),
		"child device 0x%p created", &dev->device);

	refcount_debug(dev, "device creation complete");
	rc = 0;

away:
	if (rc < 0) {
		if (registered2)
			unregister_devmajorminor_attributes(dev);
		if (registered1)
			unregister_channel_attributes(dev);
		if (gotten)
			put_visordev(dev, "create", visorbus_debugref);
		if (visorchannel != NULL) {
			INFODRV("Channel %s disconnected",
				visorchannel_id(visorchannel, s));
			visorchannel_destroy(visorchannel);
		}
		kfree(dev);
	} else {
		total_devices_created++;
		list_add_tail(&dev->list_all, &list_all_device_instances);
	}
	return rc;
}

static void
remove_visor_device(struct visor_device *dev)
{
	INFODRV("removing child device %s (0x%p)",
		dev_name(&dev->device), &dev->device);
	list_del(&dev->list_all);
	unregister_devmajorminor_attributes(dev);
	unregister_channel_attributes(dev);
	put_visordev(dev, "create", visorbus_debugref);
	refcount_debug(dev, "about to call device_unregister");
	device_unregister(&dev->device);
}

static struct visor_device *
find_visor_device_by_channel(HOSTADDRESS channel_physaddr)
{
	struct list_head *listentry, *listtmp;

	INFODRV("looking for dev with channel addr=0x%Lx",
		(unsigned long long)(channel_physaddr));

	list_for_each_safe(listentry, listtmp, &list_all_device_instances) {
		struct visor_device *dev = list_entry(listentry,
						      struct visor_device,
						      list_all);
		if (visorchannel_get_physaddr(dev->visorchannel) ==
		    channel_physaddr)
			return dev;
	}
	ERRDRV("dev with channel addr=0x%Lx not found",
	       (unsigned long long)(channel_physaddr));
	return NULL;
}

static int
init_vbus_channel(VISORCHANNEL *chan)
{
	int rc = -1;
	ulong allocated_bytes = visorchannel_get_nbytes(chan);
	struct ultra_vbus_channel_protocol *x =
		kmalloc(sizeof(struct ultra_vbus_channel_protocol),
			GFP_KERNEL|__GFP_NORETRY);

	POSTCODE_LINUX_3(VBUS_CHANNEL_ENTRY_PC, rc, POSTCODE_SEVERITY_INFO);

	if (x == NULL) {
		ERRDRV("%s failed malloc", __func__);
		POSTCODE_LINUX_2(MALLOC_FAILURE_PC, POSTCODE_SEVERITY_ERR);
		goto away;
	}
	if (visorchannel_clear(chan, 0, 0, allocated_bytes) < 0) {
		ERRDRV("%s clear failed", __func__);
		POSTCODE_LINUX_2(VBUS_CHANNEL_FAILURE_PC,
				 POSTCODE_SEVERITY_ERR);
		goto away;
	}
	if (visorchannel_read
	    (chan, 0, x, sizeof(struct ultra_vbus_channel_protocol)) < 0) {
		ERRDRV("%s chan read failed", __func__);
		POSTCODE_LINUX_2(VBUS_CHANNEL_FAILURE_PC,
				 POSTCODE_SEVERITY_ERR);
		goto away;
	}
	if (!ULTRA_VBUS_CHANNEL_OK_SERVER(allocated_bytes)) {
		ERRDRV("%s channel cannot be used", __func__);
		POSTCODE_LINUX_2(VBUS_CHANNEL_FAILURE_PC,
				 POSTCODE_SEVERITY_ERR);
		goto away;
	}

	if (visorchannel_write
	    (chan, 0, x, sizeof(struct ultra_vbus_channel_protocol)) < 0) {
		ERRDRV("%s chan write failed", __func__);
		POSTCODE_LINUX_3(VBUS_CHANNEL_FAILURE_PC, chan,
				 POSTCODE_SEVERITY_ERR);
		goto away;
	}

	POSTCODE_LINUX_3(VBUS_CHANNEL_EXIT_PC, chan, POSTCODE_SEVERITY_INFO);
	rc = 0;

away:
	if (x != NULL) {
		kfree(x);
		x = NULL;
	}
	return rc;
}

static int
get_vbus_header_info(VISORCHANNEL *chan, ULTRA_VBUS_HEADERINFO *hdr_info)
{
	int rc = -1;

	if (!SPAR_VBUS_CHANNEL_OK_CLIENT(visorchannel_get_header(chan),
					 NULL)) {
		ERRDRV("vbus channel cannot be used - visorchannel_get_header failed");
		goto away;
	}
	if (visorchannel_read
	    (chan, sizeof(struct channel_header), hdr_info,
	     sizeof(*hdr_info)) < 0) {
		ERRDRV("%s chan read failed", __func__);
		goto away;
	}
	if (hdr_info->structBytes < sizeof(ULTRA_VBUS_HEADERINFO)) {
		ERRDRV("vbus channel not used, because header too small (%d < %lu)",
		       hdr_info->structBytes, sizeof(ULTRA_VBUS_HEADERINFO));
		goto away;
	}
	if (hdr_info->deviceInfoStructBytes <
	    sizeof(struct ultra_vbus_deviceinfo)) {
		ERRDRV("vbus channel not used, because devinfo too small (%d < %lu)",
		       hdr_info->deviceInfoStructBytes,
		       sizeof(struct ultra_vbus_deviceinfo));
		goto away;
	}
	rc = 0;
away:
	return rc;
}

/* Write the contents of <info> to the struct
 * ultra_vbus_channel_protocol.ChpInfo. */

static int
write_vbus_chp_info(VISORCHANNEL *chan, ULTRA_VBUS_HEADERINFO *hdr_info,
		    struct ultra_vbus_deviceinfo *info)
{
	int off = sizeof(struct channel_header) + hdr_info->chpInfoByteOffset;
	int rc = -1;

	if (hdr_info->chpInfoByteOffset == 0) {
		ERRDRV("vbus channel not used, because chpInfoByteOffset == 0");
		goto away;
	}
	if (visorchannel_write(chan, off, info, sizeof(*info)) < 0) {
		ERRDRV("%s chan write of chpInfo to offset=%d", __func__,
		       off);
		goto away;
	}
	rc = 0;
away:
	return rc;
}

/* Write the contents of <info> to the struct
 * ultra_vbus_channel_protocol.BusInfo. */

static int
write_vbus_bus_info(VISORCHANNEL *chan, ULTRA_VBUS_HEADERINFO *hdr_info,
		    struct ultra_vbus_deviceinfo *info)
{
	int off = sizeof(struct channel_header) + hdr_info->busInfoByteOffset;
	int rc = -1;

	if (hdr_info->busInfoByteOffset == 0) {
		ERRDRV("vbus channel not used, because busInfoByteOffset == 0");
		goto away;
	}
	if (visorchannel_write(chan, off, info, sizeof(*info)) < 0) {
		ERRDRV("%s chan write of busInfo to offset=%d", __func__,
		       off);
		goto away;
	}
	rc = 0;
away:
	return rc;
}

/* Write the contents of <info> to the
 * struct ultra_vbus_channel_protocol.DevInfo[<devix>].
 */
static int
write_vbus_dev_info(VISORCHANNEL *chan, ULTRA_VBUS_HEADERINFO *hdr_info,
		    struct ultra_vbus_deviceinfo *info, int devix)
{
	int off =
	    (sizeof(struct channel_header) + hdr_info->devInfoByteOffset) +
	    (hdr_info->deviceInfoStructBytes * devix);
	int rc = -1;

	if (hdr_info->devInfoByteOffset == 0) {
		ERRDRV("vbus channel not used, because devInfoByteOffset == 0");
		goto away;
	}
	if (visorchannel_write(chan, off, info, sizeof(*info)) < 0) {
		ERRDRV("%s chan write of dev_info to offset=%d", __func__,
		       off);
		goto away;
	}
	rc = 0;
away:
	return rc;
}

/* For a child device just created on a client bus, fill in
 * information about the driver that is controlling this device into
 * the the appropriate slot within the vbus channel of the bus
 * instance.
 */
static void
fix_vbus_dev_info(struct visor_device *visordev)
{
	int i;
	VISORCHIPSET_BUS_INFO bus_info;
	struct visorbus_devdata *devdata = NULL;
	struct visor_driver *visordrv;
	int bus_no = visordev->chipset_bus_no;
	int dev_no = visordev->chipset_dev_no;
	struct ultra_vbus_deviceinfo dev_info;
	const char *chan_type_name = NULL;

	if (visordev->device.driver == NULL) {
		ERRDRV("%s no device driver for bus_no=%d dev_no=%d",
		       __func__, bus_no, dev_no);
		goto away;
	}
	visordrv = to_visor_driver(visordev->device.driver);
	if (!visorchipset_get_bus_info(bus_no, &bus_info)) {
		ERRDRV("%s visorchipset_get_bus_info for bus_no=%d failed",
		       __func__, bus_no);
		goto away;
	}
	devdata = (struct visorbus_devdata *)(bus_info.bus_driver_context);
	if (!devdata) {
		ERRDRV("%s bus_info.bus_driver_context is NULL for bus_no=%d",
		       __func__, bus_no);
		goto away;
	}
	if (!devdata->vbus_valid) {
		/* this error would have been blabbered earlier */
		goto away;
	}

	/* Within the list of device types (by GUID) that the driver
	 * says it supports, find out which one of those types matches
	 * the type of this device, so that we can include the device
	 * type name
	 */
	for (i = 0; visordrv->channel_types[i].name != NULL; i++) {
		if (STRUCTSEQUAL(visordrv->channel_types[i].guid,
				 visordev->channel_type_guid)) {
			chan_type_name = visordrv->channel_types[i].name;
			break;
		}
	}

	bus_device_info_init(&dev_info, chan_type_name,
			     visordrv->name, visordrv->version,
			     visordrv->vertag);
	write_vbus_dev_info(devdata->chan,
			    &devdata->vbus_hdr_info, &dev_info, dev_no);

	/* Re-write bus+chipset info, because it is possible that this
	* was previously written by our evil counterpart, virtpci.
	*/
	write_vbus_chp_info(devdata->chan, &devdata->vbus_hdr_info,
			    &chipset_driverinfo);
	write_vbus_bus_info(devdata->chan, &devdata->vbus_hdr_info,
			    &clientbus_driverinfo);

away:
	return;
}

/** Create a device instance for the visor bus itself.
 */
static struct visorbus_devdata *
create_bus_instance(int id)
{
	struct visorbus_devdata *rc = NULL;
	struct visorbus_devdata *devdata = NULL;
	struct device *dev;
	VISORCHIPSET_BUS_INFO bus_info;
	char s[99];

	POSTCODE_LINUX_2(BUS_CREATE_ENTRY_PC, POSTCODE_SEVERITY_INFO);
	dev = kmalloc(sizeof(*dev), GFP_KERNEL|__GFP_NORETRY);
	if (dev == NULL) {
		ERRDRV("allocation of device for bus #%d failed", id);
		POSTCODE_LINUX_2(MALLOC_FAILURE_PC, POSTCODE_SEVERITY_ERR);
		rc = NULL;
		goto away;
	}
	memset(dev, 0, sizeof(struct device));
	dev_set_name(dev, "visorbus%d", id);
	dev->release = visorbus_release_busdevice;
	if (device_register(dev) < 0) {
		ERRDRV("device_register for bus #%d failed", id);
		POSTCODE_LINUX_3(DEVICE_CREATE_FAILURE_PC, id,
				 POSTCODE_SEVERITY_ERR);
		rc = NULL;
		goto away;
	}
	INFODEV(dev_name(dev), "bus device created");

	devdata = kmalloc(sizeof(*devdata), GFP_KERNEL|__GFP_NORETRY);
	if (devdata == NULL) {
		ERRDEV(dev_name(dev), "allocation of visorbus_devdata failed");
		POSTCODE_LINUX_2(MALLOC_FAILURE_PC, POSTCODE_SEVERITY_ERR);
		rc = NULL;
		goto away;
	}
	memset(devdata, 0, sizeof(struct visorbus_devdata));
	devdata->devno = id;
	devdata->dev = dev;
	if ((visorchipset_get_bus_info(id, &bus_info)) &&
	    (bus_info.chanInfo.channelAddr > 0) &&
	    (bus_info.chanInfo.nChannelBytes > 0)) {
		HOSTADDRESS channelAddr = bus_info.chanInfo.channelAddr;
		ulong nChannelBytes = (ulong)bus_info.chanInfo.nChannelBytes;
		uuid_le channel_type_guid = bus_info.chanInfo.channelTypeGuid;

		devdata->chan = visorchannel_create(channelAddr,
						    nChannelBytes,
						    channel_type_guid);
		if (devdata->chan == NULL) {
			ERRDRV("bus channel addr = 0x%-16.16llx, size = %llu",
			       (unsigned long long)channelAddr,
			       (unsigned long long)nChannelBytes);
			ERRDRV("visorchannel_create failed");
			POSTCODE_LINUX_3(DEVICE_CREATE_FAILURE_PC, channelAddr,
					 POSTCODE_SEVERITY_ERR);
		} else {
			if (bus_info.flags.server) {
				INFODRV("Bus channel %s connected (server, addr=0x%-16.16llx, size=%llu)",
					visorchannel_id(devdata->chan, s),
					(unsigned long long)channelAddr,
					(unsigned long long)nChannelBytes);
				init_vbus_channel(devdata->chan);
			} else {
				INFODRV("Bus channel %s connected (client, addr=0x%-16.16llx, size=%llu)",
					visorchannel_id(devdata->chan, s),
					(unsigned long long)channelAddr,
					(unsigned long long)nChannelBytes);
				if (get_vbus_header_info(devdata->chan,
							 &devdata->
							 vbus_hdr_info) >= 0) {
					devdata->vbus_valid = TRUE;
					write_vbus_chp_info(devdata->chan,
							    &devdata->
							    vbus_hdr_info,
							    &chipset_driverinfo
							    );
					write_vbus_bus_info(devdata->chan,
							&devdata->
							vbus_hdr_info,
							&clientbus_driverinfo
							);
				}
			}
		}
	}
	register_businst_attributes(devdata);
	bus_count++;
	list_add_tail(&devdata->list_all, &list_all_bus_instances);
	if (id == 0)
			devdata = devdata;	/* for testing ONLY */
	dev_set_drvdata(dev, devdata);
	rc = devdata;
away:
	return rc;
}

/** Remove a device instance for the visor bus itself.
 */
static void
remove_bus_instance(struct visorbus_devdata *devdata)
{
	/* Note that this will result in the release method for
	 * devdata->dev being called, which will call
	 * visorbus_release_busdevice().  This has something to do with
	 * the put_device() done in device_unregister(), but I have never
	 * successfully been able to trace thru the code to see where/how
	 * release() gets called.  But I know it does.
	 */
	INFODRV("removing bus instance");
	unregister_businst_attributes(devdata);
	bus_count--;
	if (devdata->chan) {
		visorchannel_destroy(devdata->chan);
		devdata->chan = NULL;
	}
	list_del(&devdata->list_all);
	device_unregister(devdata->dev);
}

/** Create and register the one-and-only one instance of
 *  the visor bus type (visorbus_type).
 */
static int
create_bus_type(void)
{
	int rc = 0;

	visorbus_type.dev_attrs = visor_device_attrs;
	rc = bus_register(&visorbus_type);
	if (rc < 0) {
		ERRDRV("bus_register(&visorbus_type) failed: (status=%d)\n",
		       rc);
		goto away;
	}
	rc = register_bustype_attributes();
	if (rc < 0) {
		ERRDRV("register_bustype_attributes() failed: (status=%d)\n",
		       rc);
		goto away;
	}
	INFODRV("bus type registered %s", VERSION);
	rc = 0;
away:
	return rc;
}

/** Remove the one-and-only one instance of the visor bus type (visorbus_type).
 */
static void
remove_bus_type(void)
{
	unregister_bustype_attributes();
	bus_unregister(&visorbus_type);
	INFODRV("bus type unregistered %s", VERSION);
}

/** Remove all child visor bus device instances.
 */
static void
remove_all_visor_devices(void)
{
	struct list_head *listentry, *listtmp;

	INFODRV("removing all child devices:");

	list_for_each_safe(listentry, listtmp, &list_all_device_instances) {
		struct visor_device *dev = list_entry(listentry,
						      struct visor_device,
						      list_all);
		remove_visor_device(dev);
	}
}

static BOOL entered_testing_mode = FALSE;
static VISORCHIPSET_CHANNEL_INFO test_channel_infos[MAXDEVICETEST];
static ulong test_bus_nos[MAXDEVICETEST];
static ulong test_dev_nos[MAXDEVICETEST];

static void
chipset_bus_create(ulong bus_no)
{
	VISORCHIPSET_BUS_INFO bus_info;
	struct visorbus_devdata *devdata;
	int rc = -1;

	POSTCODE_LINUX_3(BUS_CREATE_ENTRY_PC, bus_no, POSTCODE_SEVERITY_INFO);
	if (!visorchipset_get_bus_info(bus_no, &bus_info))
		goto away;
	devdata = create_bus_instance(bus_no);
	if (!devdata)
		goto away;
	if (!visorchipset_set_bus_context(bus_no, devdata))
		goto away;
	POSTCODE_LINUX_3(BUS_CREATE_EXIT_PC, bus_no, POSTCODE_SEVERITY_INFO);
	rc = 0;
away:
	if (rc < 0) {
		ERRDRV("%s(%lu) failed", __func__, bus_no);
		POSTCODE_LINUX_3(BUS_CREATE_FAILURE_PC, bus_no,
				 POSTCODE_SEVERITY_ERR);
		return;
	}
	INFODRV("%s(%lu) successful", __func__, bus_no);
	POSTCODE_LINUX_3(CHIPSET_INIT_SUCCESS_PC, bus_no,
			 POSTCODE_SEVERITY_INFO);
	if (chipset_responders.bus_create)
		(*chipset_responders.bus_create) (bus_no, rc);
}

static void
chipset_bus_destroy(ulong bus_no)
{
	VISORCHIPSET_BUS_INFO bus_info;
	struct visorbus_devdata *devdata;
	int rc = -1;

	if (!visorchipset_get_bus_info(bus_no, &bus_info))
		goto away;
	devdata = (struct visorbus_devdata *)(bus_info.bus_driver_context);
	if (!devdata)
		goto away;
	remove_bus_instance(devdata);
	if (!visorchipset_set_bus_context(bus_no, NULL))
		goto away;
	rc = 0;
away:
	if (rc < 0) {
		ERRDRV("%s(%lu) failed", __func__, bus_no);
		return;
	}
	INFODRV("%s(%lu) successful", __func__, bus_no);
	if (chipset_responders.bus_destroy)
		(*chipset_responders.bus_destroy) (bus_no, rc);
}

static void
chipset_device_create(ulong bus_no, ulong dev_no)
{
	VISORCHIPSET_DEVICE_INFO dev_info;
	VISORCHIPSET_BUS_INFO bus_info;
	struct visorbus_devdata *devdata = NULL;
	int rc = -1;

	POSTCODE_LINUX_4(DEVICE_CREATE_ENTRY_PC, dev_no, bus_no,
			 POSTCODE_SEVERITY_INFO);

	if (entered_testing_mode)
		return;
	if (!visorchipset_get_device_info(bus_no, dev_no, &dev_info))
		goto away;
	if (!visorchipset_get_bus_info(bus_no, &bus_info))
		goto away;
	if (visorbus_devicetest)
		if (total_devices_created < MAXDEVICETEST) {
			test_channel_infos[total_devices_created] =
			    dev_info.chanInfo;
			test_bus_nos[total_devices_created] = bus_no;
			test_dev_nos[total_devices_created] = dev_no;
		}
	POSTCODE_LINUX_4(DEVICE_CREATE_EXIT_PC, dev_no, bus_no,
			 POSTCODE_SEVERITY_INFO);
	rc = 0;
away:
	if (rc < 0) {
		ERRDRV("%s(%lu,%lu) failed", __func__, bus_no, dev_no);
		POSTCODE_LINUX_4(DEVICE_CREATE_FAILURE_PC, dev_no, bus_no,
				 POSTCODE_SEVERITY_ERR);
		return;
	}
	INFODRV("%s(%lu,%lu) successful", __func__, bus_no, dev_no);
	devdata = (struct visorbus_devdata *)(bus_info.bus_driver_context);
	rc = create_visor_device(devdata, bus_no, dev_no,
				 dev_info.chanInfo, bus_info.partitionHandle);
	POSTCODE_LINUX_4(DEVICE_CREATE_SUCCESS_PC, dev_no, bus_no,
			 POSTCODE_SEVERITY_INFO);
	if (rc < 0)
		if (chipset_responders.device_create)
			(*chipset_responders.device_create)(bus_no, dev_no, rc);
}

static void
chipset_device_destroy(ulong bus_no, ulong dev_no)
{
	VISORCHIPSET_DEVICE_INFO dev_info;
	struct visor_device *dev;
	int rc = -1;

	if (entered_testing_mode)
		return;
	if (!visorchipset_get_device_info(bus_no, dev_no, &dev_info))
		goto away;
	dev = find_visor_device_by_channel(dev_info.chanInfo.channelAddr);
	if (!dev)
		goto away;
	rc = 0;
away:
	if (rc < 0) {
		ERRDRV("%s(%lu,%lu) failed", __func__, bus_no, dev_no);
		return;
	}
	INFODRV("%s(%lu,%lu) successful", __func__, bus_no, dev_no);
	if (chipset_responders.device_destroy)
		(*chipset_responders.device_destroy) (bus_no, dev_no, rc);
	remove_visor_device(dev);
}

/* This is the callback function specified for a function driver, to
 * be called when a pending "pause device" operation has been
 * completed.
 */
static void
pause_state_change_complete(struct visor_device *dev, int status)
{
	if (!dev->pausing) {
		ERRDEV(dev_name(&dev->device),
		       "%s, but not pausing! (rc=%d)", __func__, status);
		return;
	}
	INFODEV(dev_name(&dev->device),
		"transition running-->paused rc=%d", status);
	dev->pausing = FALSE;
	if (!chipset_responders.device_pause) {
		/* this can never happen! */
		HUHDEV(dev_name(&dev->device), "no pause complete function");
		return;
	}
	/* Notify the chipset driver that the pause is complete, which
	* will presumably want to send some sort of response to the
	* initiator. */
	(*chipset_responders.device_pause) (dev->chipset_bus_no,
					    dev->chipset_dev_no, status);
}

/* This is the callback function specified for a function driver, to
 * be called when a pending "resume device" operation has been
 * completed.
 */
static void
resume_state_change_complete(struct visor_device *dev, int status)
{
	if (!dev->resuming) {
		ERRDEV(dev_name(&dev->device),
		       "%s, but not resuming! (rc=%d)", __func__, status);
		return;
	}
	INFODEV(dev_name(&dev->device),
		"transition paused-->running rc=%d", status);
	dev->resuming = FALSE;
	if (!chipset_responders.device_resume) {
		/* this can never happen! */
		HUHDEV(dev_name(&dev->device), "no resume complete function");
		return;
	}
	/* Notify the chipset driver that the resume is complete,
	 * which will presumably want to send some sort of response to
	 * the initiator. */
	(*chipset_responders.device_resume) (dev->chipset_bus_no,
					     dev->chipset_dev_no, status);
}

/* Tell the subordinate function driver for a specific device to pause
 * or resume that device.  Result is returned asynchronously via a
 * callback function.
 */
static void
initiate_chipset_device_pause_resume(ulong bus_no, ulong dev_no, BOOL is_pause)
{
	VISORCHIPSET_DEVICE_INFO dev_info;
	struct visor_device *dev = NULL;
	int rc = -1, x;
	struct visor_driver *drv = NULL;
	void (*notify_func)(ulong bus_no, ulong dev_no, int response) = NULL;

	if (is_pause)
		notify_func = chipset_responders.device_pause;
	else
		notify_func = chipset_responders.device_resume;
	if (!notify_func) {
		HUHDRV("no chipset_responders notify function (this is WAY serious)");
		goto away;
	}
	if (!visorchipset_get_device_info(bus_no, dev_no, &dev_info)) {
		ERRDRV("visorchipset_get_device_info_failed");
		goto away;
	}
	dev = find_visor_device_by_channel(dev_info.chanInfo.channelAddr);
	if (!dev) {
		ERRDRV("device not found");
		goto away;
	}
	drv = to_visor_driver(dev->device.driver);
	if (!drv) {
		ERRDEV(dev_name(&dev->device), "driver not found");
		goto away;
	}
	if (dev->pausing || dev->resuming) {
		ERRDEV(dev_name(&dev->device), "already pausing or resuming");
		goto away;
	}
	/* Note that even though both drv->pause() and drv->resume
	 * specify a callback function, it is NOT necessary for us to
	 * increment our local module usage count.  Reason is, there
	 * is already a linkage dependency between child function
	 * drivers and visorbus, so it is already IMPOSSIBLE to unload
	 * visorbus while child function drivers are still running.
	 */
	if (is_pause) {
		if (!drv->pause) {
			ERRDEV(dev_name(&dev->device),
			       "visorbus cannot pause device, because function driver does not support pause");
			goto away;
		}
		dev->pausing = TRUE;
		x = drv->pause(dev, pause_state_change_complete);
	} else {
		/* This should be done at BUS resume time, but an
		 * existing problem prevents us from ever getting a bus
		 * resume...  This hack would fail to work should we
		 * ever have a bus that contains NO devices, since we
		 * would never even get here in that case. */
		fix_vbus_dev_info(dev);
		if (!drv->resume) {
			ERRDEV(dev_name(&dev->device),
			       "visorbus cannot resume device, because function driver does not support resume");
			goto away;
		}
		dev->resuming = TRUE;
		x = drv->resume(dev, resume_state_change_complete);
	}
	if (x < 0) {
		if (is_pause)
			dev->pausing = FALSE;
		else
			dev->resuming = FALSE;
		ERRDEV(dev_name(&dev->device),
		       "function driver pause/resume failed with rc=%d", x);
		goto away;
	}
	rc = 0;
away:
	if (rc < 0) {
		if (dev)
			ERRDEV(dev_name(&dev->device), "%s state change failed",
			       (is_pause) ? "pause" : "resume");
		else
			ERRDRV("%s(%s,%lu,%lu) failed",
			       __func__, (is_pause) ? "pause" : "resume",
			       bus_no, dev_no);
		if (notify_func)
			(*notify_func) (bus_no, dev_no, rc);
	}
}

static void
chipset_device_pause(ulong bus_no, ulong dev_no)
{
	initiate_chipset_device_pause_resume(bus_no, dev_no, TRUE);
}

static void
chipset_device_resume(ulong bus_no, ulong dev_no)
{
	initiate_chipset_device_pause_resume(bus_no, dev_no, FALSE);
}

struct channel_size_info {
	uuid_le guid;
	ulong min_size;
	ulong max_size;
};

static int
find_channel_size(struct device_driver *xdrv, void *xinfo)
{
	struct channel_size_info *info = (struct channel_size_info *)(xinfo);
	int i = 0;
	struct visor_driver *drv = to_visor_driver(xdrv);

	if (drv->channel_types == NULL)
		return 0;
	for (i = 0;
	     (uuid_le_cmp(drv->channel_types[i].guid, NULL_UUID_LE) != 0) ||
	     (drv->channel_types[i].name == NULL); i++)
		if (uuid_le_cmp(drv->channel_types[i].guid, info->guid) == 0) {
			info->min_size = drv->channel_types[i].min_size;
			info->max_size = drv->channel_types[i].max_size;
		}
	return 0;
}

static int
chipset_get_channel_info(uuid_le type_guid, ulong *min_size, ulong *max_size)
{
	struct channel_size_info info;
	int res = 0;

	memset(&info, 0, sizeof(info));
	info.guid = type_guid;
	res =
	    bus_for_each_drv(&visorbus_type, NULL, (void *)(&info),
			     find_channel_size);
	if (info.min_size == 0 && info.max_size == 0)
		return -1;
	*min_size = info.min_size;
	*max_size = info.max_size;
	return 0;
}

static void
periodic_test_work(struct work_struct *work)
{
	VISORCHIPSET_CHANNEL_INFO chanInfo;
	struct visor_device *dev;
	u64 current_interval;
	static BOOL create_phase = FALSE;
	static u64 last_interval;

	/* INFODRV("periodic work"); */
	current_interval = get_jiffies_64() / 8192;
	/* 32.5 seconds at HZ=250 */
	if (visorbus_devicetest && (current_interval > last_interval)) {
		last_interval = current_interval;
		INFODRV("devicetest interval #%llu",
			(unsigned long long)current_interval);
		if ((visorbus_devicetest > 0) &&
		    (total_devices_created >= visorbus_devicetest)) {
			int i;

			entered_testing_mode = TRUE;
			if (create_phase) {
				INFODRV("Adding devices for devicetest...");
				for (i = 0; i < visorbus_devicetest; i++) {
					create_visor_device
					    (devdata,
					     test_bus_nos[i],
					     test_dev_nos[i],
					     test_channel_infos[i], 0);
				}
				create_phase = FALSE;
			} else {
				INFODRV("Removing devices for devicetest...");
				for (i = 0; i < visorbus_devicetest; i++) {
					dev = find_visor_device_by_channel
					    (test_channel_infos[i].channelAddr);
					if (dev != NULL)
						remove_visor_device(dev);
				}
				create_phase = TRUE;
			}
		}
	}

	if (visorbus_serialloopbacktest && (!entered_testing_mode)) {
		entered_testing_mode = TRUE;
		memset(&chanInfo, 0, sizeof(chanInfo));
		chanInfo.channelAddr = SERIALLOOPBACKCHANADDR;
		create_visor_device(devdata, 0, 0, chanInfo, 0);
		create_visor_device(devdata, 0, 0, chanInfo, 0);
	}

	if (queue_delayed_work(periodic_test_workqueue,
			       &periodic_work, POLLJIFFIES_TESTWORK) < 0)
		ERRDRV("queue_delayed_work failed!");
}

static int __init
visorbus_init(void)
{
	int rc = 0;

	POSTCODE_LINUX_3(DRIVER_ENTRY_PC, rc, POSTCODE_SEVERITY_INFO);
	INFODRV("bus driver version %s loaded", VERSION);

	bus_device_info_init(&clientbus_driverinfo,
			     "clientbus", MYDRVNAME,
			     VERSION, NULL);

	/* process module options */

	INFODRV("option - debug=%d", visorbus_debug);
	INFODRV("option - forcematch=%d", visorbus_forcematch);
	INFODRV("option - forcenomatch=%d", visorbus_forcenomatch);
	INFODRV("option - devicetest=%d", visorbus_devicetest);
	if (visorbus_devicetest > MAXDEVICETEST) {
		visorbus_devicetest = MAXDEVICETEST;
		INFODRV("option - devicetest=%d (reduced to maximum)",
			visorbus_devicetest);
	}
	INFODRV("option - debugref=%d", visorbus_debugref);
	INFODRV("option - serialloopbacktest=%d",
		visorbus_serialloopbacktest);

	rc = create_bus_type();
	if (rc < 0) {
		ERRDRV("create_bus_type(): error (status=%d)\n", rc);
		POSTCODE_LINUX_2(BUS_CREATE_ENTRY_PC, DIAG_SEVERITY_ERR);
		goto away;
	}

	if (visorbus_serialloopbacktest || visorbus_devicetest) {
		INIT_DELAYED_WORK(&periodic_work, periodic_test_work);
		periodic_test_workqueue =
		    create_singlethread_workqueue("visorbus_test");
		if (periodic_test_workqueue == NULL) {
			ERRDRV("cannot create test workqueue: error (status=%d)\n",
			       -ENOMEM);
			POSTCODE_LINUX_2(BUS_CREATE_ENTRY_PC,
					 DIAG_SEVERITY_ERR);
			rc = -ENOMEM;
			goto away;
		}

		rc = queue_delayed_work(periodic_test_workqueue,
					&periodic_work,
					POLLJIFFIES_TESTWORK);
		if (rc < 0) {
			ERRDRV("queue_delayed_work(periodic_test_workqueue, &periodic_work, POLLJIFFIES_TESTWORK): error (status=%d)\n", rc);
			POSTCODE_LINUX_2(QUEUE_DELAYED_WORK_PC,
					 DIAG_SEVERITY_ERR);
			goto away;
		}
	}

	periodic_dev_workqueue = create_singlethread_workqueue("visorbus_dev");
	if (periodic_dev_workqueue == NULL) {
		ERRDRV("cannot create dev workqueue: error (status=%d)\n",
		       -ENOMEM);
		POSTCODE_LINUX_2(CREATE_WORKQUEUE_PC, DIAG_SEVERITY_ERR);
		rc = -ENOMEM;
		goto away;
	}

	/* This enables us to receive notifications when devices appear for
	 * which this service partition is to be a server for.
	 */
	visorchipset_register_busdev_server(&chipset_notifiers,
					    &chipset_responders,
					    &chipset_driverinfo);

	rc = 0;

away:
	if (rc) {
		ERRDRV("visorbus_init failed");
		POSTCODE_LINUX_3(CHIPSET_INIT_FAILURE_PC, rc,
				 POSTCODE_SEVERITY_ERR);
	}
	return rc;
}

static void
visorbus_exit(void)
{
	struct list_head *listentry, *listtmp;

	visorchipset_register_busdev_server(NULL, NULL, NULL);
	remove_all_visor_devices();

	flush_workqueue(periodic_dev_workqueue); /* better not be any work! */
	destroy_workqueue(periodic_dev_workqueue);
	periodic_dev_workqueue = NULL;

	if (periodic_test_workqueue) {
		cancel_delayed_work(&periodic_work);
		flush_workqueue(periodic_test_workqueue);
		destroy_workqueue(periodic_test_workqueue);
		periodic_test_workqueue = NULL;
	}

	list_for_each_safe(listentry, listtmp, &list_all_bus_instances) {
		struct visorbus_devdata *devdata = list_entry(listentry,
							      struct
							      visorbus_devdata,
							      list_all);
		remove_bus_instance(devdata);
	}
	remove_bus_type();
	INFODRV("bus driver unloaded");
}

module_param_named(debug, visorbus_debug, int, S_IRUGO);
MODULE_PARM_DESC(visorbus_debug, "1 to debug");
int visorbus_debug = 0;

module_param_named(forcematch, visorbus_forcematch, int, S_IRUGO);
MODULE_PARM_DESC(visorbus_forcematch,
		 "1 to force a successful dev <--> drv match");
int visorbus_forcematch = 0;

module_param_named(forcenomatch, visorbus_forcenomatch, int, S_IRUGO);
MODULE_PARM_DESC(visorbus_forcenomatch,
		 "1 to force an UNsuccessful dev <--> drv match");
int visorbus_forcenomatch = 0;

module_param_named(devicetest, visorbus_devicetest, int, S_IRUGO);
MODULE_PARM_DESC(visorbus_devicetest,
		 "non-0 to just test device creation and destruction");
int visorbus_devicetest = 0;

module_param_named(debugref, visorbus_debugref, int, S_IRUGO);
MODULE_PARM_DESC(visorbus_debugref, "1 to debug reference counting");
int visorbus_debugref = 0;

module_param_named(serialloopbacktest, visorbus_serialloopbacktest,
		   int, S_IRUGO);
MODULE_PARM_DESC(visorbus_serialloopbacktest,
		 "non-0 to just create 2 serial devices on the same channel");
int visorbus_serialloopbacktest = 0;

module_init(visorbus_init);
module_exit(visorbus_exit);

MODULE_AUTHOR("Unisys");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Supervisor bus driver for service partition: ver " VERSION);
MODULE_VERSION(VERSION);
