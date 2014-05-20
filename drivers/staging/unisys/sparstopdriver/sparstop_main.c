/* sparstop_main.c
 *
 * Copyright © 2010 - 2013 UNISYS CORPORATION
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

/* --- Compiletime options ---// */
/* Enable writing to /proc/sparstop/driver/diag to initiate tests */
#define INCLUDE_TEST_INTERFACE
#define ENABLE_RETURN_TRACE
/* Enable DEBUGDRV and DEBUGDEV messages */
/* (but you will still need "echo 8 >/proc/sys/kernel/printk" to see
 * on console)
 */
#define DEBUG

#include "sparstop_private.h"
#include "sparstop.h"
#include "easyproc.h"
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <linux/rtc.h>
#include <linux/version.h>

static int sparstop_major;

static dev_t MajorDev = -1;
				/**< indicates major num for devices */
static spinlock_t devnopool_lock;
static void *DevNoPool;	/**< pool to grab device numbers from */
static struct easyproc_driver_info Easyproc_driver_info;

static int
simplebus_uevent(struct device *xdev, struct kobj_uevent_env *env)
{
	if (add_uevent_var(env, "VERSION=%s", VERSION))
		return -ENOMEM;
	return 0;
}

static int
simplebus_match(struct device *xdev, struct device_driver *xdrv)
{
	return 1;
}

/** This describes the TYPE of bus.
 *  (Don't confuse this with an INSTANCE of the bus.)
 */
static struct bus_type Simplebus_type = {
	.name = SPARSTOP_DEVICEPREFIX,
	.match = simplebus_match,
	.uevent = simplebus_uevent,
};

static struct device *StandaloneDevice;

/** These are all the read-only devdata properties we maintain for each device.
 *  They will all be reported under /sys/devices/sparstop<n>.
 */
typedef enum {
	propro_devmajorminor,
	propro_state,
	propro_stateno,
	/* Add items above, but don't forget to modify
	 * register_ro_devdata_attributes whenever you do...
	 */
	propro_DEVDATAMAX
} DEVDATA_RO_PROPERTIES;

/*  These are all the read-write devdata properties we maintain
 *  flocal/source/sparstopdriver/sparstop.hor each device.  They will
 *  all be reported under /sys/devices/sparstop<n>/.
 */
typedef enum {
	proprw_inprogress,
	/* Add items above, but don't forget to modify
	 * register_rw_devdata_attributes whenever you do...
	 */
	proprw_DEVDATAMAX
} DEVDATA_RW_PROPERTIES;

typedef enum {
	state_none,		/* no stop requested, no stop data */
	state_requested,	/* kernel has requested usermode to stop */
	state_inProgress,	/* usermode stop has started */
	state_complete,		/* usermode stop completed */
	state_failed,		/* usermode stop app failed, data no good */
} SPARSTOP_STATE;

static char *
state_str(SPARSTOP_STATE state)
{
	switch (state) {
	case state_none:
		return "none";
	case state_requested:
		return "requested";
	case state_inProgress:
		return "inProgress";
	case state_complete:
		return "complete";
	case state_failed:
		return "failed";
	default:
		return "???";
	}
	return "";
}

#define TRANSITION_STATE(devdata, old_state, new_state) \
	transition_state_guts(devdata, old_state, new_state, \
			      PathName_Last_N_Nodes(__FILE__, 3), __LINE__)

/** This is the private data that we store for each kernelmode device.
 *  A pointer to this struct is kept in each "struct device", and can be
 *  obtained using dev_get_drvdata(dev).
 */
struct sparstop_devdata {
	int devno;
	struct device *dev;
	struct rw_semaphore lock_device; /** lock for dev */
	char name[99];
	struct list_head list_all;   /**< link within List_all_devices list */
	/** head of list of sparstop_filedata structs, linked
	 *  via the list_all member */
	struct list_head list_files;
	uint openFileCount;
	uint writeFileCount;
	rwlock_t lock_files; /** lock for list_files, openFileCount,
			       * writeFileCount */
	ulong umodeBytesIn;  /** \# bytes we have input from user mode */
	ulong umodeWriteCount;	/** \# writes we have seen from user mode */
	struct device_attribute devdata_ro_property[propro_DEVDATAMAX];
	struct device_attribute devdata_rw_property[proprw_DEVDATAMAX];
	struct kref kref; /** we can deallocate only when refcount drops to 0 */
	struct cdev cdev_stop; /** /dev/spar/sparstop */
	struct easyproc_device_info procinfo;

	/** callbacks to handle  interactions */
	SPARSTOP_COMPLETE_FUNC complete_func;
	void *complete_context;	/* context to pass to callbacks */
	rwlock_t lock_state;	/* not needed?? */
	SPARSTOP_STATE state;
};
/** List of all sparstop_devdata structs, linked via the list_all member */
static LIST_HEAD(List_all_devices);
static DEFINE_SPINLOCK(Lock_all_devices);

/** This is the private data that we store for each file descriptor that is
 *  opened to the diag character device.
 */
struct sparstop_filedata {
	struct sparstop_devdata *devdata;
	/** link within devdata.list_files list */
	struct list_head list_all;
};

static void sparstop_show_device_info(struct seq_file *seq, void *p);
static void sparstop_show_driver_info(struct seq_file *seq);
static BOOL transition_state_guts(struct sparstop_devdata *devdata,
				  SPARSTOP_STATE old_state,
				  SPARSTOP_STATE new_state, char *filename,
				  int lineno);
static void test_initiate(void);
static void remove_stop_device(struct device *dev);
static void sparstop_process_driver_diag_command(char *buf, size_t count,
						 loff_t *ppos);

/*  DEVICE attributes
 *
 *  define & implement display and storing of device attributes under
 *  /sys/devices/sparstop<n>/.
 *
 */
static ssize_t
devdata_ro_property_show(struct device *ddev,
			 struct device_attribute *attr, char *buf)
{
	struct sparstop_devdata *devdata = dev_get_drvdata(ddev);
	ulong offset = (ulong) (attr) - (ulong) (devdata->devdata_ro_property);
	ulong ix = offset / sizeof(struct device_attribute);
	if (ix >= propro_DEVDATAMAX) {
		dev_err(ddev, "%s:%d trouble in paradise; ix=%lu\n",
		       PathName_Last_N_Nodes(__FILE__, 3), __LINE__, ix);
		return 0;
	}
	switch (ix) {
	case propro_devmajorminor:
		return sprintf(buf, "%d:%d\n",
			       (int) MAJOR(MajorDev), devdata->devno);
	case propro_state:
		return sprintf(buf, "%s\n", state_str(devdata->state));
	case propro_stateno:
		return sprintf(buf, "%d\n", devdata->state);
	default:
		dev_err(ddev, "%s:%d trouble in paradise; ix=%lu\n",
		       PathName_Last_N_Nodes(__FILE__, 3), __LINE__, ix);
		return 0;
	}
	return 0;
}

static ssize_t
devdata_rw_property_show(struct device *ddev,
			 struct device_attribute *attr, char *buf)
{
	struct sparstop_devdata *devdata = dev_get_drvdata(ddev);
	ulong offset = (ulong) (attr) - (ulong) (devdata->devdata_rw_property);
	ulong ix = offset / sizeof(struct device_attribute);
	if (ix >= proprw_DEVDATAMAX) {
		dev_err(ddev, "%s:%d trouble in paradise; ix=%lu\n",
		       PathName_Last_N_Nodes(__FILE__, 3), __LINE__, ix);
		return 0;
	}
	switch (ix) {
	case proprw_inprogress:
		return sprintf(buf, "%d\n",
			       (devdata->state == state_inProgress));
	default:
		dev_err(ddev, "%s:%d trouble in paradise; ix=%lu\n",
		       PathName_Last_N_Nodes(__FILE__, 3), __LINE__, ix);
		return 0;
	}
	return 0;
}

static BOOL
get_ulong_from_buf(const char *buf, size_t count, ulong *answer)
{
	char s[99];
	ulong val;
	char *p = s;
	if (count >= sizeof(s))
		return FALSE;
	memcpy(s, buf, count);
	s[count] = '\0';
	if (kstrtoul(p, 0, &val) != 0)
		return FALSE;
	if (p != s) {
		if (*p == '\r')
			p++;
		if (*p == '\n')
			p++;
	}
	if ((p == s) || (*p != '\0'))
		return FALSE;
	*answer = val;
	return TRUE;
}

static ssize_t
devdata_rw_property_store(struct device *ddev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct sparstop_devdata *devdata = dev_get_drvdata(ddev);
	ulong offset = (ulong) (attr) - (ulong) (devdata->devdata_rw_property);
	ulong ix = offset / sizeof(struct device_attribute);
	ulong val = 0;

	if (ix >= proprw_DEVDATAMAX) {
		dev_err(ddev, "%s:%d trouble in paradise; ix=%lu\n",
		       PathName_Last_N_Nodes(__FILE__, 3), __LINE__, ix);
		return 0;
	}
	switch (ix) {
	case proprw_inprogress:
		if (!get_ulong_from_buf(buf, count, &val))
			ERRDEV(devdata->name,
			       "inprogress must be number (hex or dec)");
		LOCKWRITESEM(&devdata->lock_device);
		if (val == 1)
			TRANSITION_STATE(devdata, state_requested,
					 state_inProgress);
		else if (val == 3) {
			/* the stop script failed and detected that it
			 * failed.
			 */
			if (TRANSITION_STATE
			    (devdata, state_inProgress, state_failed)) {
				if (!devdata->complete_func)
					ERRDEV(devdata->name,
					       "no complete_func!");
				else
					devdata->complete_func(devdata->
							       complete_context,
							       -1);
			}
		} else if (val == 0) {
			/* the stop succeeded */
			if (TRANSITION_STATE
			    (devdata, state_inProgress, state_complete)) {
				if (!devdata->complete_func)
					ERRDEV(devdata->name,
					       "no complete_func!");
				else
					devdata->complete_func(devdata->
							       complete_context,
							       0);
			}
		} else {
		}
		UNLOCKWRITESEM(&devdata->lock_device);
		break;
	default:
		dev_err(ddev, "%s:%d trouble in paradise; ix=%lu\n",
		       PathName_Last_N_Nodes(__FILE__, 3), __LINE__, ix);
		return 0;
	}
	return strnlen(buf, count);
}

static int
register_ro_devdata_attributes(struct device *dev)
{
	int rc = 0, i = 0;
	struct sparstop_devdata *devdata = dev_get_drvdata(dev);
	struct device_attribute *pattr = devdata->devdata_ro_property;
	INFODRV("%s:Entered pattr=%p", __func__, pattr);

	pattr[propro_devmajorminor].attr.name = "devmajorminor";
	pattr[propro_state].attr.name = "state";
	pattr[propro_stateno].attr.name = "stateno";
	for (i = 0; i < propro_DEVDATAMAX; i++) {
		pattr[i].attr.mode = S_IRUGO;
		pattr[i].show = devdata_ro_property_show;
		pattr[i].store = NULL;
		rc = device_create_file(dev, &pattr[i]);
		if (rc < 0) {
			ERRDRV("device_create_file(dev, &pattr[i]): error (status=%d)\n",
			       rc);
			goto Away;
		}
	}

	rc = 0;
Away:
	return rc;
}

static int
register_rw_devdata_attributes(struct device *dev)
{
	int rc = 0, i = 0;
	struct sparstop_devdata *devdata = dev_get_drvdata(dev);
	struct device_attribute *pattr = devdata->devdata_rw_property;
	INFODRV("%s:Entered pattr=%p", __func__, pattr);

	pattr[proprw_inprogress].attr.name = "inprogress";
	for (i = 0; i < proprw_DEVDATAMAX; i++) {
		pattr[i].attr.mode = S_IRUGO | S_IWUGO;
		pattr[i].show = devdata_rw_property_show;
		pattr[i].store = devdata_rw_property_store;
		rc = device_create_file(dev, &pattr[i]);
		if (rc < 0) {
			ERRDRV("device_create_file(dev, &pattr[i]): error (status=%d)\n",
			       rc);
			goto Away;
		}
	}

	rc = 0;
Away:
	return rc;
}

static int
register_device_attributes(struct device *dev)
{
	int rc = 0;
	rc = register_ro_devdata_attributes(dev);
	if (rc < 0) {
		ERRDRV("register_ro_devdata_attributes(dev): error (status=%d)\n",
		       rc);
		goto Away;
	}
	rc = register_rw_devdata_attributes(dev);
	if (rc < 0) {
		ERRDRV("register_rw_devdata_attributes(dev): error (status=%d)\n",
		       rc);
		goto Away;
	}
	rc = 0;
Away:
	return rc;
}

static int
unregister_ro_devdata_attributes(struct device *dev)
{
	int rc = 0, i = 0;
	struct sparstop_devdata *devdata;
	struct device_attribute *pattr;
	INFODRV("%s:Entered ", __func__);
	devdata = dev_get_drvdata(dev);
	pattr = devdata->devdata_ro_property;
	for (i = 0; i < propro_DEVDATAMAX; i++)
		device_remove_file(dev, &pattr[i]);
	return rc;
}

static int
unregister_rw_devdata_attributes(struct device *dev)
{
	int rc = 0, i = 0;
	struct sparstop_devdata *devdata;
	struct device_attribute *pattr;
	INFODRV("%s:Entered ", __func__);
	devdata = dev_get_drvdata(dev);
	pattr = devdata->devdata_rw_property;
	for (i = 0; i < proprw_DEVDATAMAX; i++)
		device_remove_file(dev, &pattr[i]);
	return rc;
}

static int
unregister_device_attributes(struct device *dev)
{
	int rc = 0;
	INFODRV("%s:Entered ", __func__);
	unregister_ro_devdata_attributes(dev);
	unregister_rw_devdata_attributes(dev);
	INFODRV("%s:Leaving ", __func__);
	return rc;
}

static struct sparstop_devdata *
devdata_create(struct device *dev)
{
	void *rc = NULL;
	struct sparstop_devdata *devdata = NULL;

	devdata = kmalloc(sizeof(struct sparstop_devdata),
			  GFP_KERNEL|__GFP_NORETRY);
	if (devdata == NULL) {
		ERRDRV("allocation of sparstop_devdata failed: (status=0)\n");
		rc = NULL;
		goto Away;
	}
	memset(devdata, '\0', sizeof(struct sparstop_devdata));
	devdata->state = state_none;
	cdev_init(&devdata->cdev_stop, NULL);
	if (kstrtoint(dev_name(dev) + strlen(SPARSTOP_DEVICEPREFIX), 10,
		      &devdata->devno) != 0) {
		ERRDRV("failed to get devno: (status=0)\n");
		rc = NULL;
		goto Away;
	}

	devdata->dev = dev;
	strncpy(devdata->name, dev_name(devdata->dev), sizeof(devdata->name));

	devdata->cdev_stop.owner = THIS_MODULE;
	if (cdev_add(&devdata->cdev_stop,
		     MKDEV(MAJOR(MajorDev), devdata->devno), 1) < 0) {
		ERRDRV("failed to create char device: (status=0)\n");
		rc = NULL;
		goto Away;
	}
	rwlock_init(&devdata->lock_files);
	rwlock_init(&devdata->lock_state);
	init_rwsem(&devdata->lock_device);
	INIT_LIST_HEAD(&devdata->list_files);
	kref_init(&devdata->kref);	/* sets reference count to 1 */
	spin_lock(&Lock_all_devices);
	list_add_tail(&devdata->list_all, &List_all_devices);
	spin_unlock(&Lock_all_devices);

	rc = devdata;
Away:
	if (rc == NULL) {
		if (devdata != NULL) {
			if (devdata->cdev_stop.ops != NULL)
				cdev_del(&devdata->cdev_stop);
			kfree(devdata);
		}
	}
	return rc;
}

static void
devdata_release(struct kref *mykref)
{
	struct sparstop_devdata *devdata = container_of(mykref,
							struct sparstop_devdata,
							kref);
	INFODEV(devdata->name, "devdata deallocated (no more refs)");
	spin_lock(&Lock_all_devices);
	list_del(&devdata->list_all);
	spin_unlock(&Lock_all_devices);
	cdev_del(&devdata->cdev_stop);
	kfree(devdata);
}

static void
devdata_put(struct sparstop_devdata *devdata)
{
	INFODRV("%s:Entered ", __func__);
	kref_put(&devdata->kref, devdata_release);
}

static void
devdata_get(struct sparstop_devdata *devdata)
{
	kref_get(&devdata->kref);
}

static void
remove_stop_device(struct device *dev)
{
	struct sparstop_devdata *devdata = dev_get_drvdata(dev);
	INFODRV("%s:Entered ", __func__);
	if (devdata == NULL) {
		HUHDRV("no devdata in %s", __func__);
		goto Away;
	}
	unregister_device_attributes(dev);
	devdata_put(devdata);	/* 1 less reference to devdata */
	INFODRV("%s:About to do dev_set_drvdata", __func__);
	dev_set_drvdata(dev, NULL);
	INFODRV("%s:About to do easyproc_DeInitDevice ", __func__);
	visor_easyproc_DeInitDevice(&Easyproc_driver_info,
				    &devdata->procinfo, devdata->devno);

	/* Note that it is still possible to have files open to this device
	 * right now.  If this is the case, the reference counts for devdata
	 * will be up, preventing its deallocation below.
	 * The state of "devdata->dev == NULL" can be used to determine that we
	 * are in this state.
	 */
	sprintf(devdata->name, "<dev#%d-history>", devdata->devno);
	devdata->dev = NULL;
	INFODRV("%s:About to do devdata_put ", __func__);
	devdata_put(devdata);
	/* Undo kref_init(&devdata->kref) from devdata_create(): */
	INFODRV("%s:About to do put_device ", __func__);
	put_device(dev);	/* from add_stop_device */
	/* Undo device_initialize + device_add() from add_stop_device(): */
	INFODRV("%s:About to do device_unregister ", __func__);
	device_unregister(dev);	/* Here is where KOBJ_REMOVE hotplug happens */
	INFODRV("%s:Done ", __func__);

Away:
	INFODRV("%s finished", __func__);
}

static void
stop_device_release(struct device *dev)
{
	ulong ul;

	if (dev == NULL)
		return;
	INFODEV(dev_name(dev),
		"kernelmode stop device deallocated (no more refs)");
	if(kstrtoul(dev_name(dev) + strlen(SPARSTOP_DEVICEPREFIX), 10, &ul)) {
		ERRDEV(dev_name(dev), "invalid device name-not freeing");
		return;
	}
	spin_lock(&devnopool_lock);
	clear_bit(ul, DevNoPool);
	spin_unlock(&devnopool_lock);
	if (dev)
		kfree(dev);
}

static struct device *
add_stop_device(void)
{
	struct device *rc = NULL;
	struct device *dev = NULL;
	BOOL gotten = FALSE, registered = FALSE;
	int devno = -1;
	struct sparstop_devdata *devdata = NULL;
	BOOL devdata_bumped = FALSE;

	spin_lock(&devnopool_lock);
	devno = find_first_zero_bit(DevNoPool, MAXDEVICES);
	set_bit(devno, DevNoPool);
	spin_unlock(&devnopool_lock);
	if (devno == MAXDEVICES)
		devno = -1;
	if (devno < 0) {
		ERRDRV("attempt to create more than MAXDEVICES devices??: (status=0)\n");
		rc = NULL;
		goto Away;
	}

	dev = kmalloc(sizeof(struct device), GFP_KERNEL|__GFP_NORETRY);
	if (dev == NULL) {
		ERRDRV("failed to allocate device: (status=0)\n");
		rc = NULL;
		goto Away;
	}
	memset(dev, 0, sizeof(struct device));
	dev->bus = &Simplebus_type;
	device_initialize(dev);
	dev->release = stop_device_release;
	get_device(dev);	/* keep a reference just for us */
	gotten = TRUE;

	/* bus_id must be a unique name with respect to this bus TYPE
	 * (NOT bus instance).  That's why we need to include the bus
	 * number within the name.
	 */
	dev_set_name(dev, "%s%d", SPARSTOP_DEVICEPREFIX, devno);
	devdata = devdata_create(dev);
	if (devdata == NULL) {
		ERRDRV("failed to allocate devdata for device: (status=0)\n");
		rc = NULL;
		goto Away;
	}
	dev_set_drvdata(dev, devdata);
	/* We just gave a devdata reference to dev, so bump the in-user count.
	 * This protects against strange problems like devdata going away while
	 * we are processing a sysfs attribute or something...
	 */
	devdata_get(devdata);
	devdata_bumped = TRUE;
	visor_easyproc_InitDevice(&Easyproc_driver_info,
				  &devdata->procinfo, devdata->devno, devdata);

	/* This is where the KOBJ_ADD hotplug event happens */
	if (device_add(dev) < 0) {
		ERRDRV("device_add failed: (status=0)\n");
		rc = NULL;
		goto Away;
	}
	/* note: device_register is simply device_initialize + device_add */

	if (register_device_attributes(dev) < 0) {
		ERRDRV("register_device_attributes failed: (status=0)\n");
		rc = NULL;
		goto Away;
	}
	registered = TRUE;
	INFODEV(dev_name(dev), "kernelmode stop device 0x%p created", dev);
	rc = dev;
Away:
	if (rc == NULL) {
		if (registered)
			unregister_device_attributes(dev);
		if (devdata_bumped)
			devdata_put(devdata);
		if (gotten)
			put_device(dev);
		else if (devno >= 0)
		{
			spin_lock(&devnopool_lock);
			clear_bit(devno, DevNoPool);
			spin_unlock(&devnopool_lock);
		}
		if (devdata)
			devdata_put(devdata);
		if (dev)
			kfree(dev);
	}
	return rc;
}

static void
sparstop_cleanup_guts(void)
{
	if (StandaloneDevice) {
		remove_stop_device(StandaloneDevice);
		StandaloneDevice = NULL;
	}
	bus_unregister(&Simplebus_type);
	visor_easyproc_DeInitDriver(&Easyproc_driver_info);
	if (MAJOR(MajorDev) >= 0) {
		unregister_chrdev_region(MajorDev, MAXDEVICES);
		MajorDev = MKDEV(0, 0);
	}
	if (DevNoPool != NULL) {
		kfree(DevNoPool);
		DevNoPool = NULL;
	}
}

static BOOL
transition_state_guts(struct sparstop_devdata *devdata,
		      SPARSTOP_STATE old_state, SPARSTOP_STATE new_state,
		      char *filename, int lineno)
{
	if (devdata->state != old_state) {
		ERRDEV(devdata->name,
		       "Can't do state transition %s(%d)-->%s(%d) @%s:%d"
		       "(old state=%s(%d) invalid)", state_str(old_state),
		       old_state, state_str(new_state), new_state, filename,
		       lineno, state_str(devdata->state), devdata->state);
		return FALSE;
	}
	INFODEV(devdata->name, "state transition %s(%d)-->%s(%d) @%s:%d",
		state_str(old_state), old_state,
		state_str(new_state), new_state, filename, lineno);
	devdata->state = new_state;
	return TRUE;
}

static int __init
sparstop_init(void)
{
	int rc = -1;

	INFODRV("driver version %s loaded", VERSION);
	/* uintpool_test(); */
	INFODRV("Options:");
	INFODRV("         major=%d", sparstop_major);

	MajorDev = MKDEV(sparstop_major, 0);
	spin_lock_init(&devnopool_lock);
	DevNoPool = kzalloc(BITS_TO_LONGS(MAXDEVICES), GFP_KERNEL);
	if (DevNoPool == NULL) {
		ERRDRV("Unable to create DevNoPool");
		goto Away;
	}
	if (alloc_chrdev_region(&MajorDev, 0, MAXDEVICES, MYDRVNAME) < 0) {
		ERRDRV("Unable to register char device %s", MYDRVNAME);
		goto Away;
	}
	visor_easyproc_InitDriverEx(&Easyproc_driver_info,
				    MYDRVNAME,
				    sparstop_show_driver_info,
				    sparstop_show_device_info,
				    sparstop_process_driver_diag_command, NULL);
	rc = bus_register(&Simplebus_type);
	if (rc < 0) {
		ERRDRV("allocation of sparstop_devdata failed: (status=%d)\n",
		       rc);
		goto Away;
	}
	rc = 0;
Away:
	if (rc < 0)
		sparstop_cleanup_guts();
	return rc;
}

static void
sparstop_cleanup(void)
{
	sparstop_cleanup_guts();
	INFODRV("driver unloaded");
}

int
sp_stop(void *context, SPARSTOP_COMPLETE_FUNC complete_func)
{
	struct sparstop_devdata *devdata = NULL;
	int rc = -1;

	if (StandaloneDevice != NULL) {
		ERRDRV("%s StandaloneDevice already active, removing it",
		       __func__);
		remove_stop_device(StandaloneDevice);
		StandaloneDevice = NULL;
	}
	StandaloneDevice = add_stop_device();
	if (StandaloneDevice == NULL) {
		ERRDRV("%s failed to create sparstop device", __func__);
		goto Away;
	}
	devdata = dev_get_drvdata(StandaloneDevice);
	devdata->complete_func = complete_func;
	devdata->complete_context = context;
	TRANSITION_STATE(devdata, state_none, state_requested);
	rc = 0;
Away:
	return rc;
}
EXPORT_SYMBOL_GPL(sp_stop);

#ifdef INCLUDE_TEST_INTERFACE

static void
test_stop_complete_func(void *context, int status)
{
	INFODRV("%s called with context=%p, status=%d",
		__func__, context, status);
}

static void
test_initiate(void)
{
	INFODRV("%s sp_stop returned %d",
		__func__, sp_stop(NULL, &test_stop_complete_func));
}

#endif

void
test_remove_stop_device(void)
{
	if (StandaloneDevice) {
		remove_stop_device(StandaloneDevice);
		StandaloneDevice = NULL;
	}
}
EXPORT_SYMBOL_GPL(test_remove_stop_device);

/* This is called when you read /proc/sparstop/device/<n>/diag. */
static void
sparstop_show_device_info(struct seq_file *seq, void *p)
{
	struct sparstop_devdata *devdata = (struct sparstop_devdata *) (p);

	seq_printf(seq, "devno=%d\n", devdata->devno);
	seq_printf(seq, "bus name = '%s'\n", devdata->name);
	seq_printf(seq, "state = %s(%d)\n",
		   state_str(devdata->state), devdata->state);
}

/* This is called when you read /proc/sparstop/driver/diag. */
static void
sparstop_show_driver_info(struct seq_file *seq)
{
	seq_printf(seq, "Version=%s\n", VERSION);
}

/* This is called when you write to /proc/sparstop/driver/diag. */
static void
sparstop_process_driver_diag_command(char *buf, size_t count, loff_t *ppos)
{
#ifdef INCLUDE_TEST_INTERFACE
	char s[99];
	size_t i;
	if (count >= sizeof(s))
		return;
	for (i = 0; i < count; i++)
		if (buf[i] == '\n' || buf[i] == '\r')
			break;
		else
			s[i] = buf[i];
	s[i] = '\0';
	if (strcmp(s, "initiate") == 0)
		test_initiate();
	else {
		ERRDRV("%s unrecognized string %s", __func__, s);
		return;
	}
#endif
}

module_param_named(major, sparstop_major, int, S_IRUGO);
MODULE_PARM_DESC(sparstop_major, "major device number for the sparstop device");

module_init(sparstop_init);
module_exit(sparstop_cleanup);

MODULE_AUTHOR("Unisys");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sPAR sparstop driver for service partition: ver " VERSION);
MODULE_VERSION(VERSION);
