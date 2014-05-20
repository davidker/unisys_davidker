/* visorconinclient_main.c
 *
 * Copyright © 2011 - 2013 UNISYS CORPORATION
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
 * receive keyboard and mouse channels from the visorbus driver.  It reads
 * inputs from such channels, and delivers it to the Linux OS in the
 * standard way the Linux expects for input drivers.
 */

#include "uniklog.h"
#include "diagnostics/appos_subsystems.h"
#include "timskmod.h"
#include "globals.h"
#include "visorbus.h"
#include "visorchannel.h"
#include "keyboardchannel.h"
#include "mousechannel.h"
#include "easyproc.h"
#define KBDGUID UltraKeyboardChannelProtocolGuid
#define MOUGUID UltraMouseChannelProtocolGuid
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>

#define PIXELS_ACROSS_DEFAULT 800
#define PIXELS_DOWN_DEFAULT   600
#define SYSFS_VIRTUALSIZE "/sys/class/graphics/fb0/virtual_size"

static spinlock_t devnopool_lock;
static void *DevNoPool; /**< pool to grab device numbers from */
static struct easyproc_driver_info Easyproc_driver_info;

static int visorconinclient_probe(struct visor_device *dev);
static void visorconinclient_remove(struct visor_device *dev);
static void visorconinclient_channel_interrupt(struct visor_device *dev);
static int visorconinclient_pause(struct visor_device *dev,
				  VISORBUS_STATE_COMPLETE_FUNC complete_func);
static int visorconinclient_resume(struct visor_device *dev,
				   VISORBUS_STATE_COMPLETE_FUNC complete_func);
static struct input_dev *register_client_keyboard(void);
static struct input_dev *register_client_mouse(void);
static struct input_dev *register_client_wheel(void);
static void unregister_client_input(struct input_dev *visorinput_dev);
static void visorconinclient_show_device_info(struct seq_file *seq, void *p);
static void visorconinclient_show_driver_info(struct seq_file *seq);

/**  GUIDS for all channel types supported by this driver.
 */
static struct visor_channeltype_descriptor visorconinclient_channel_types[] = {
	{ULTRA_KEYBOARD_CHANNEL_PROTOCOL_GUID, "keyboard",
	 KEYBOARD_CH_SIZE, KEYBOARD_CH_SIZE},
	{ULTRA_MOUSE_CHANNEL_PROTOCOL_GUID, "mouse",
	 MOUSE_CH_SIZE, MOUSE_CH_SIZE},
	{ NULL_UUID_LE, NULL, 0, 0}
};

/** This is used to tell the visor bus driver which types of visor devices
 *  we support, and what functions to call when a visor device that we support
 *  is attached or removed.
 */
static struct visor_driver visorconinclient_driver = {
	.name = MYDRVNAME,
	.version = VERSION,
	.vertag = NULL,
	.build_date = __DATE__,
	.build_time = __TIME__,
	.owner = THIS_MODULE,
	.channel_types = visorconinclient_channel_types,
	.probe = visorconinclient_probe,
	.remove = visorconinclient_remove,
	.channel_interrupt = visorconinclient_channel_interrupt,
	.pause = visorconinclient_pause,
	.resume = visorconinclient_resume,
};

/** This is the private data that we store for each device.
 *  A pointer to this struct is kept in each "struct device", and can be
 *  obtained using visor_get_drvdata(dev).
 */
struct visorconinclient_devdata {
	int devno;
	struct visor_device *dev;
	/** lock for dev */
	struct rw_semaphore lockVisorDev;
	char name[99];
	struct list_head list_all;   /**< link within List_all_devices list */
	struct kref kref;
	struct easyproc_device_info procinfo;
	struct input_dev *visorinput_dev;
	struct input_dev *visorinput_dev2;
	BOOL supported_client_device;
	BOOL paused;
};
/** List of all visorconinclient_devdata structs,
  * linked via the list_all member */
static LIST_HEAD(List_all_devices);
static DEFINE_SPINLOCK(Lock_all_devices);

#define devdata_put(devdata, why)					\
	do {								\
		int refcount;						\
		kref_put(&devdata->kref, devdata_release);		\
		refcount = atomic_read(&devdata->kref.refcount);	\
		if (visorconinclient_debugref)				\
			VISORBUS_DEBUG_REFCOUNT_CHANGE			\
				(refcount+1, refcount, devdata, why);	\
	} while (0)

#define devdata_get(deevdata, why)					\
	do {								\
		int refcount;						\
		kref_get(&devdata->kref);				\
		refcount = atomic_read(&devdata->kref.refcount);	\
		if (visorconinclient_debugref)				\
			VISORBUS_DEBUG_REFCOUNT_CHANGE			\
				(refcount-1, refcount, devdata, why);	\
	} while (0)

/* Borrowed from drivers/input/keyboard/atakbd.c */
/* This maps 1-byte scancodes to keycodes. */
static unsigned char visorkbd_keycode[256] = {	/* American layout */
	[0] = KEY_GRAVE,
	[1] = KEY_ESC,
	[2] = KEY_1,
	[3] = KEY_2,
	[4] = KEY_3,
	[5] = KEY_4,
	[6] = KEY_5,
	[7] = KEY_6,
	[8] = KEY_7,
	[9] = KEY_8,
	[10] = KEY_9,
	[11] = KEY_0,
	[12] = KEY_MINUS,
	[13] = KEY_EQUAL,
	[14] = KEY_BACKSPACE,
	[15] = KEY_TAB,
	[16] = KEY_Q,
	[17] = KEY_W,
	[18] = KEY_E,
	[19] = KEY_R,
	[20] = KEY_T,
	[21] = KEY_Y,
	[22] = KEY_U,
	[23] = KEY_I,
	[24] = KEY_O,
	[25] = KEY_P,
	[26] = KEY_LEFTBRACE,
	[27] = KEY_RIGHTBRACE,
	[28] = KEY_ENTER,
	[29] = KEY_LEFTCTRL,
	[30] = KEY_A,
	[31] = KEY_S,
	[32] = KEY_D,
	[33] = KEY_F,
	[34] = KEY_G,
	[35] = KEY_H,
	[36] = KEY_J,
	[37] = KEY_K,
	[38] = KEY_L,
	[39] = KEY_SEMICOLON,
	[40] = KEY_APOSTROPHE,
	[41] = KEY_GRAVE,	/* FIXME, '#' */
	[42] = KEY_LEFTSHIFT,
	[43] = KEY_BACKSLASH,	/* FIXME, '~' */
	[44] = KEY_Z,
	[45] = KEY_X,
	[46] = KEY_C,
	[47] = KEY_V,
	[48] = KEY_B,
	[49] = KEY_N,
	[50] = KEY_M,
	[51] = KEY_COMMA,
	[52] = KEY_DOT,
	[53] = KEY_SLASH,
	[54] = KEY_RIGHTSHIFT,
	[55] = KEY_KPASTERISK,
	[56] = KEY_LEFTALT,
	[57] = KEY_SPACE,
	[58] = KEY_CAPSLOCK,
	[59] = KEY_F1,
	[60] = KEY_F2,
	[61] = KEY_F3,
	[62] = KEY_F4,
	[63] = KEY_F5,
	[64] = KEY_F6,
	[65] = KEY_F7,
	[66] = KEY_F8,
	[67] = KEY_F9,
	[68] = KEY_F10,
	[69] = KEY_NUMLOCK,
	[70] = KEY_SCROLLLOCK,
	[71] = KEY_KP7,
	[72] = KEY_KP8,
	[73] = KEY_KP9,
	[74] = KEY_KPMINUS,
	[75] = KEY_KP4,
	[76] = KEY_KP5,
	[77] = KEY_KP6,
	[78] = KEY_KPPLUS,
	[79] = KEY_KP1,
	[80] = KEY_KP2,
	[81] = KEY_KP3,
	[82] = KEY_KP0,
	[83] = KEY_KPDOT,
	[87] = KEY_F11,
	[88] = KEY_F12,
	[90] = KEY_KPLEFTPAREN,
	[91] = KEY_KPRIGHTPAREN,
	[92] = KEY_KPASTERISK,	/* FIXME */
	[93] = KEY_KPASTERISK,
	[94] = KEY_KPPLUS,
	[95] = KEY_HELP,
	[96] = KEY_KPENTER,
	[97] = KEY_RIGHTCTRL,
	[98] = KEY_KPSLASH,
	[99] = KEY_KPLEFTPAREN,
	[100] = KEY_KPRIGHTPAREN,
	[101] = KEY_KPSLASH,
	[102] = KEY_HOME,
	[103] = KEY_UP,
	[104] = KEY_PAGEUP,
	[105] = KEY_LEFT,
	[106] = KEY_RIGHT,
	[107] = KEY_END,
	[108] = KEY_DOWN,
	[109] = KEY_PAGEDOWN,
	[110] = KEY_INSERT,
	[111] = KEY_DELETE,
	[112] = KEY_MACRO,
	[113] = KEY_MUTE
};

/* This maps the <xx> in extended scancodes of the form "0xE0 <xx>" into */
/* keycodes. */
static unsigned char ext_keycode[256] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 0x00 */
	0, 0, 0, 0, 0, 0, 0, 0,	/* 0x10 */
	0, 0, 0, 0, KEY_KPENTER, KEY_RIGHTCTRL, 0, 0,	/* 0x18 */
	0, 0, 0, 0, 0, 0, 0, 0,	/* 0x20 */
	KEY_RIGHTALT, 0, 0, 0, 0, 0, 0, 0,	/* 0x28 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 0x30 */
	0, 0, 0, 0, 0, 0, 0, KEY_HOME,	/* 0x40 */
	KEY_UP, KEY_PAGEUP, 0, KEY_LEFT, 0, KEY_RIGHT, 0, KEY_END, /* 0x48 */
	KEY_DOWN, KEY_PAGEDOWN, KEY_INSERT, KEY_DELETE, 0, 0, 0, 0, /* 0x50 */
	0, 0, 0, 0, 0, 0, 0, 0,	/* 0x58 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 0x60 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 0x70 */
};

static struct visorconinclient_devdata *
devdata_create(struct visor_device *dev)
{
	void *rc = NULL;
	struct visorconinclient_devdata *devdata = NULL;
	int devno = -1;
	uuid_le guid;

	guid = visorchannel_get_uuid(dev->visorchannel);
	devdata = kmalloc(sizeof(struct visorconinclient_devdata),
			  GFP_KERNEL|__GFP_NORETRY);
	if (devdata == NULL) {
		ERRDRV("allocation of visorconinclient_devdata failed: (status=0)\n");
		goto Away;
	}
	memset(devdata, '\0', sizeof(struct visorconinclient_devdata));
	spin_lock(&devnopool_lock);
	devno = find_first_zero_bit(DevNoPool, MAXDEVICES);
	set_bit(devno, DevNoPool);
	spin_unlock(&devnopool_lock);
	if (devno == MAXDEVICES)
		devno = -1;
	if (devno < 0) {
		ERRDRV("attempt to create more than MAXDEVICES devices: (status=0)\n");
		goto Away;
	}

	devdata->devno = devno;
	devdata->dev = dev;
	strncpy(devdata->name, dev_name(&dev->device), sizeof(devdata->name));

	/* This is an input device in a client guest partition,
	 * so we need to create whatever gizmos are necessary to
	 * deliver our inputs to the guest OS. */
	if (memcmp(&guid, &KBDGUID, sizeof(guid)) == 0) {
		devdata->visorinput_dev = register_client_keyboard();
		if (devdata->visorinput_dev == NULL) {
			ERRDRV("failed to create client keyboard device: (status=0)\n");
			goto Away;
		}
		devdata->supported_client_device = TRUE;
	} else if (memcmp(&guid, &MOUGUID, sizeof(guid)) == 0) {
		devdata->visorinput_dev = register_client_mouse();
		if (devdata->visorinput_dev == NULL) {
			ERRDRV("failed to create client mouse device: (status=0)\n");
			goto Away;
		}
		devdata->visorinput_dev2 = register_client_wheel();
		if (devdata->visorinput_dev2 == NULL) {
			ERRDRV("failed to create client wheel device: (status=0)\n");
			goto Away;
		}
		devdata->supported_client_device = TRUE;
	}

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
		if (devdata != NULL) {
			if (devdata->visorinput_dev != NULL) {
				unregister_client_input
				    (devdata->visorinput_dev);
				devdata->visorinput_dev = NULL;
			}
			if (devdata->visorinput_dev2 != NULL) {
				unregister_client_input
				    (devdata->visorinput_dev2);
				devdata->visorinput_dev2 = NULL;
			}
			kfree(devdata);
		}
	}
	return rc;
}

static void
devdata_release(struct kref *mykref)
{
	struct visorconinclient_devdata *devdata =
	    container_of(mykref, struct visorconinclient_devdata, kref);
	INFODRV("%s", __func__);
	spin_lock(&devnopool_lock);
	clear_bit(devdata->devno, DevNoPool);
	spin_unlock(&devnopool_lock);
	spin_lock(&Lock_all_devices);
	list_del(&devdata->list_all);
	spin_unlock(&Lock_all_devices);
	INFODRV("%s finished", __func__);
}

static int
visorconinclient_probe(struct visor_device *dev)
{
	int rc = 0;
	struct visorconinclient_devdata *devdata = NULL;
	uuid_le guid;

	INFODRV("%s", __func__);

	devdata = devdata_create(dev);
	if (devdata == NULL) {
		rc = -1;
		goto Away;
	}
	visor_set_drvdata(dev, devdata);
	guid = visorchannel_get_uuid(dev->visorchannel);
	if (memcmp(&guid, &MOUGUID, sizeof(guid)) != 0
	    && memcmp(&guid, &KBDGUID, sizeof(guid)) != 0) {
		ERRDRV("unrecognized GUID: (status=-1)\n");
		rc = -1;
		goto Away;
	}

	visor_easyproc_InitDevice(&Easyproc_driver_info,
				  &devdata->procinfo, devdata->devno, devdata);
	if (devdata->supported_client_device)
		visorbus_enable_channel_interrupts(dev);

Away:
	INFODRV("%s finished", __func__);
	if (rc < 0) {
		if (devdata != NULL)
			devdata_put(devdata, "existence");
	}
	return rc;
}

static void
host_side_disappeared(struct visorconinclient_devdata *devdata)
{
	LOCKWRITESEM(&devdata->lockVisorDev);
	sprintf(devdata->name, "<dev#%d-history>", devdata->devno);
	devdata->dev = NULL;	/* indicate device destroyed */
	UNLOCKWRITESEM(&devdata->lockVisorDev);
}

static void
visorconinclient_remove(struct visor_device *dev)
{
	struct visorconinclient_devdata *devdata = visor_get_drvdata(dev);
	INFODRV("%s", __func__);
	if (devdata == NULL) {
		ERRDRV("no devdata in %s", __func__);
		goto Away;
	}
	visor_set_drvdata(dev, NULL);
	visor_easyproc_DeInitDevice(&Easyproc_driver_info,
				    &devdata->procinfo, devdata->devno);
	host_side_disappeared(devdata);
	unregister_client_input(devdata->visorinput_dev);
	devdata->visorinput_dev = NULL;
	unregister_client_input(devdata->visorinput_dev2);
	devdata->visorinput_dev2 = NULL;
	devdata_put(devdata, "existence");
Away:
	INFODRV("%s finished", __func__);
}

static void
visorconinclient_cleanup_guts(void)
{
	visorbus_unregister_visor_driver(&visorconinclient_driver);
	visor_easyproc_DeInitDriver(&Easyproc_driver_info);
	if (DevNoPool != NULL) {
		kfree(DevNoPool);
		DevNoPool = NULL;
	}
}

static void
unregister_client_input(struct input_dev *visorinput_dev)
{
	if (visorinput_dev != NULL) {
		input_unregister_device(visorinput_dev);
		INFODRV("unregistered client input device");
	}
}

/* register_client_keyboard() initializes and returns a Linux gizmo that we
 * can use to deliver keyboard inputs to Linux.  We of course do this when
 * we see keyboard inputs coming in on a keyboard channel.
 */
static struct input_dev *
register_client_keyboard(void)
{
	int i, error;
	struct input_dev *visorinput_dev = NULL;

	visorinput_dev = input_allocate_device();
	if (!visorinput_dev) {
		ERRDRV("input_allocate_device() failed");
		return NULL;
	}

	visorinput_dev->name = "visor Keyboard";
	visorinput_dev->phys = "visorkbd/input0";
	visorinput_dev->id.bustype = BUS_HOST;
	visorinput_dev->id.vendor = 0x0001;
	visorinput_dev->id.product = 0x0001;
	visorinput_dev->id.version = 0x0100;

	visorinput_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP)
	    | BIT_MASK(EV_LED);
	visorinput_dev->ledbit[0] = BIT_MASK(LED_CAPSL)
	    | BIT_MASK(LED_SCROLLL) | BIT_MASK(LED_NUML);
	visorinput_dev->keycode = visorkbd_keycode;
	visorinput_dev->keycodesize = sizeof(unsigned char);
	visorinput_dev->keycodemax = ARRAY_SIZE(visorkbd_keycode);

	for (i = 1; i < ARRAY_SIZE(visorkbd_keycode); i++)
		set_bit(visorkbd_keycode[i], visorinput_dev->keybit);

	error = input_register_device(visorinput_dev);
	if (error) {
		input_free_device(visorinput_dev);
		ERRDRV("input_register_device() failed");
		return NULL;
	}
	INFODRV("registered client input device");

	return visorinput_dev;
}

/* register_client_mouse() initializes and returns a Linux gizmo that we
 * can use to deliver mouse inputs to Linux.  We of course do this when
 * we see mouse inputs coming in on a mouse channel.
 *
 * Note that in order to use the mouse in X, it needs to be declared like
 * this in xorg.conf:
 *
 *    Section "InputDevice"
 *      Driver       "evdev"
 *      Identifier   "sPAR virtual mouse"
 *      Option       "Device" "/dev/input/event1"
 *      Option       "AccelerationProfile" "-1"
 *      Option       "AccelerationScheme" "none"
 *      Option       "CorePointer"
 *    EndSection
 *
 * Then reference that from your ServerLayout section, like this:
 *
 *    InputDevice  "sPAR virtual mouse" "CoreMouse"
 *
 * This command can be used to determine which /dev/input/event* device
 * is the sPAR mouse:
 *
 *    udevadm info --attribute-walk --name=/dev/input/event*
 */
static struct input_dev *
register_client_mouse(void)
{
	int error;
	struct input_dev *visorinput_dev = NULL;
	int xres, yres;
	struct fb_info *fb0;

	visorinput_dev = input_allocate_device();
	if (!visorinput_dev) {
		ERRDRV("input_allocate_device() failed");
		return NULL;
	}

	visorinput_dev->name = "visor Mouse";
	visorinput_dev->phys = "visormou/input0";
	visorinput_dev->id.bustype = BUS_HOST;
	visorinput_dev->id.vendor = 0x0001;
	visorinput_dev->id.product = 0x0001;
	visorinput_dev->id.version = 0x0100;

	visorinput_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	set_bit(BTN_LEFT, visorinput_dev->keybit);
	set_bit(BTN_RIGHT, visorinput_dev->keybit);
	set_bit(BTN_MIDDLE, visorinput_dev->keybit);

	if(registered_fb[0]) {
		fb0 = registered_fb[0];
		xres = fb0->var.xres_virtual;
		yres = fb0->var.yres_virtual;
	} else {
		xres = PIXELS_ACROSS_DEFAULT;
		yres = PIXELS_DOWN_DEFAULT;
	}
	input_set_abs_params(visorinput_dev, ABS_X, 0, xres, 0, 0);
	input_set_abs_params(visorinput_dev, ABS_Y, 0, yres, 0, 0);

	error = input_register_device(visorinput_dev);
	if (error) {
		input_free_device(visorinput_dev);
		ERRDRV("input_register_device() failed");
		return NULL;
	}
	INFODRV("registered client input device");

	/* Sending top-left and bottom-right positions is ABSOLUTELY
	* REQUIRED if we want X to move the mouse to the exact points
	* we tell it.  I have NO IDEA why.
	*/
	input_report_abs(visorinput_dev, ABS_X, 0);
	input_report_abs(visorinput_dev, ABS_Y, 0);
	input_sync(visorinput_dev);
	input_report_abs(visorinput_dev, ABS_X, xres - 1);
	input_report_abs(visorinput_dev, ABS_Y, yres - 1);
	input_sync(visorinput_dev);

	return visorinput_dev;
}

/* register_client_wheel() initializes and returns a Linux gizmo that we
 * can use to deliver mouse wheel inputs to Linux.  We of course do this when
 * we see wheel inputs coming in on a mouse channel.  It would be NICE to be
 * able to report wheel events on the mouse device, but X can't deal with a
 * single device that has both absolute (X and Y) and relative (wheel) axes.
 * That's why we create 2 devices for Linux: 1 mouse device and 1 wheel device.
 *
 * Note that in order to use the wheel in X, it needs to be declared like
 * this in xorg.conf:
 *
 *    Section "InputDevice"
 *      Driver       "evdev"
 *      Identifier   "sPAR virtual wheel"
 *      Option       "Device" "/dev/input/event4"
 *    EndSection
 *
 * Then reference that from your ServerLayout section, like this:
 *
 *    InputDevice  "sPAR virtual wheel"
 *
 * This command can be used to determine which /dev/input/event* device
 * is the sPAR wheel:
 *
 *    udevadm info --attribute-walk --name=/dev/input/event*
 */
static struct input_dev *
register_client_wheel(void)
{
	int error;
	struct input_dev *visorinput_dev = NULL;

	visorinput_dev = input_allocate_device();
	if (!visorinput_dev) {
		ERRDRV("input_allocate_device() failed");
		return NULL;
	}

	visorinput_dev->name = "visor Wheel";
	visorinput_dev->phys = "visorwhl/input0";
	visorinput_dev->id.bustype = BUS_HOST;
	visorinput_dev->id.vendor = 0x0001;
	visorinput_dev->id.product = 0x0001;
	visorinput_dev->id.version = 0x0100;

	/* We need to lie a little to prevent the evdev driver "Don't
	* know how to use device" error.  (evdev erroneously thinks
	* that a device without an X and Y axis is useless.)
	*/
	visorinput_dev->evbit[0] = BIT_MASK(EV_REL)
	    | /*lie */ BIT_MASK(EV_KEY)	/*lie */
	    ;
	visorinput_dev->relbit[0] = BIT_MASK(REL_WHEEL)
	    | /*lie */ BIT_MASK(REL_X)	/*lie */
	    | /*lie */ BIT_MASK(REL_Y)	/*lie */
	    ;
	set_bit(BTN_LEFT, visorinput_dev->keybit);	/*lie */
	set_bit(BTN_RIGHT, visorinput_dev->keybit);	/*lie */
	set_bit(BTN_MIDDLE, visorinput_dev->keybit);	/*lie */

	error = input_register_device(visorinput_dev);
	if (error) {
		input_free_device(visorinput_dev);
		ERRDRV("input_register_device() failed");
		return NULL;
	}
	INFODRV("registered client input device");
	return visorinput_dev;
}

static void
do_key(struct input_dev *inpt, int keycode, int down)
{
	input_report_key(inpt, keycode, down);
}

/* Make it so the current locking state of the locking key indicated by
 * <keycode> is as indicated by <desired_state> (1=locked, 0=unlocked).
 */
static void
handle_locking_key(struct input_dev *visorinput_dev,
		   int keycode, int desired_state)
{
	int led;
	char *sled;

	switch (keycode) {
	case KEY_CAPSLOCK:
		led = LED_CAPSL;
		sled = "CAP";
		break;
	case KEY_SCROLLLOCK:
		led = LED_SCROLLL;
		sled = "SCR";
		break;
	case KEY_NUMLOCK:
		led = LED_NUML;
		sled = "NUM";
		break;
	default:
		WARNDRV("invalid locking key %d", keycode);
		led = -1;
		break;
	}
	if (led >= 0) {
		int old_state = (test_bit(led, visorinput_dev->led) != 0);
		if (old_state != desired_state) {
			DEBUGDRV("LED %s change: %d-->%d",
				 sled, old_state, desired_state);
			do_key(visorinput_dev, keycode, 1);
			do_key(visorinput_dev, keycode, 0);
			input_sync(visorinput_dev);
			__change_bit(led, visorinput_dev->led);
		} else
			DEBUGDRV("LED %s UNCHANGED(%d)", sled, old_state);
	}
}

/* <scancode> is either a 1-byte scancode, or an extended 16-bit scancode
 * with 0xE0 in the low byte and the extended scancode value in the next
 * higher byte.
 */
static int
scancode_to_keycode(int scancode)
{
	int keycode;
	if (scancode > 0xff)
		keycode = ext_keycode[(scancode >> 8) & 0xff];
	else
		keycode = visorkbd_keycode[scancode];
	return keycode;
}

static int
calc_button(int x)
{
	switch (x) {
	case 1:
		return BTN_LEFT;
	case 2:
		return BTN_MIDDLE;
	case 3:
		return BTN_RIGHT;
	default:
		return -1;
	}
}

/* This is used only when this driver is active as an input driver in the
 * client guest partition.  It is called periodically so we can obtain inputs
 * from the channel, and deliver them to the guest OS.
 */
static void
visorconinclient_channel_interrupt(struct visor_device *dev)
{
	ULTRA_INPUTREPORT r;
	int scancode, keycode;
	struct input_dev *visorinput_dev;
	struct input_dev *visorinput_dev2;
	int xmotion, ymotion, zmotion, button;
	int i;
	BOOL locked = FALSE;

	struct visorconinclient_devdata *devdata = visor_get_drvdata(dev);
	if (devdata == NULL) {
		ERRDEV(dev_name(&dev->device), "no devdata in %s",
		       __func__);
		goto Away;
	}
	LOCKWRITESEM(&devdata->lockVisorDev);
	locked = TRUE;
	if (devdata->paused)
		goto Away;	/* don't touch device/channel when paused */
	visorinput_dev = devdata->visorinput_dev;
	if (visorinput_dev == NULL) {
		ERRDEV(dev_name(&dev->device), "no visorinput_dev in %s",
		       __func__);
		goto Away;
	}
	visorinput_dev2 = devdata->visorinput_dev2;
	while (visorchannel_signalremove(dev->visorchannel, 0, &r)) {
		scancode = r.activity.arg1;
		keycode = scancode_to_keycode(scancode);
		switch (r.activity.action) {
		case inputAction_keyDown:
			do_key(visorinput_dev, keycode, 1);
			input_sync(visorinput_dev);
			break;
		case inputAction_keyUp:
			do_key(visorinput_dev, keycode, 0);
			input_sync(visorinput_dev);
			break;
		case inputAction_keyDownUp:
			do_key(visorinput_dev, keycode, 1);
			do_key(visorinput_dev, keycode, 0);
			input_sync(visorinput_dev);
			break;
		case inputAction_setLockingKeyState:
			handle_locking_key(visorinput_dev, keycode,
					   r.activity.arg2);
			break;
		case inputAction_xyMotion:
			xmotion = r.activity.arg1;
			ymotion = r.activity.arg2;
			input_report_abs(visorinput_dev, ABS_X, xmotion);
			input_report_abs(visorinput_dev, ABS_Y, ymotion);
			input_sync(visorinput_dev);
			break;
		case inputAction_mouseButtonDown:
			button = calc_button(r.activity.arg1);
			if (button < 0)
				break;
			input_report_key(visorinput_dev, button, 1);
			input_sync(visorinput_dev);
			break;
		case inputAction_mouseButtonUp:
			button = calc_button(r.activity.arg1);
			if (button < 0)
				break;
			input_report_key(visorinput_dev, button, 0);
			input_sync(visorinput_dev);
			break;
		case inputAction_mouseButtonClick:
			button = calc_button(r.activity.arg1);
			if (button < 0)
				break;
			input_report_key(visorinput_dev, button, 1);

			input_sync(visorinput_dev);
			input_report_key(visorinput_dev, button, 0);
			input_sync(visorinput_dev);
			break;
		case inputAction_mouseButtonDclick:
			button = calc_button(r.activity.arg1);
			if (button < 0)
				break;
			for (i = 0; i < 2; i++) {
				input_report_key(visorinput_dev, button, 1);
				input_sync(visorinput_dev);
				input_report_key(visorinput_dev, button, 0);
				input_sync(visorinput_dev);
			}
			break;
		case inputAction_wheelRotateAway:
			if (visorinput_dev2 == NULL) {
				ERRDEV(dev_name(&dev->device),
				       "no visorinput_dev2 in %s",
				       __func__);
				goto Away;
			}
			zmotion = r.activity.arg1;
			input_report_rel(visorinput_dev2, REL_WHEEL, 1);
			input_sync(visorinput_dev2);
			break;
		case inputAction_wheelRotateToward:
			if (visorinput_dev2 == NULL) {
				ERRDEV(dev_name(&dev->device),
				       "no visorinput_dev2 in %s",
				       __func__);
				goto Away;
			}
			zmotion = r.activity.arg1;
			input_report_rel(visorinput_dev2, REL_WHEEL, -1);
			input_sync(visorinput_dev2);
			break;
		}
	}

Away:
	if (locked) {
		UNLOCKWRITESEM(&devdata->lockVisorDev);
		locked = FALSE;
	}
}

static int
visorconinclient_pause(struct visor_device *dev,
		       VISORBUS_STATE_COMPLETE_FUNC complete_func)
{
	BOOL locked = FALSE;
	int rc = -1;
	struct visorconinclient_devdata *devdata = visor_get_drvdata(dev);

	if (devdata == NULL) {
		ERRDEV(dev_name(&dev->device), "no devdata in %s",
		       __func__);
		goto Away;
	}
	LOCKWRITESEM(&devdata->lockVisorDev);
	locked = TRUE;
	if (devdata->paused) {
		ERRDEV(dev_name(&dev->device),
		       "already paused, so pause not necessary");
		goto Away;
	}
	/* SLEEP(5);  // test */
	devdata->paused = TRUE;
	INFODEV(dev_name(&dev->device), "paused");
	complete_func(dev, 0);
	rc = 0;
Away:
	if (locked) {
		UNLOCKWRITESEM(&devdata->lockVisorDev);
		locked = FALSE;
	}
	return rc;
}

static int
visorconinclient_resume(struct visor_device *dev,
			VISORBUS_STATE_COMPLETE_FUNC complete_func)
{
	BOOL locked = FALSE;
	int rc = -1;
	struct visorconinclient_devdata *devdata = visor_get_drvdata(dev);

	if (devdata == NULL) {
		ERRDEV(dev_name(&dev->device), "no devdata in %s",
		       __func__);
		goto Away;
	}
	LOCKWRITESEM(&devdata->lockVisorDev);
	locked = TRUE;
	if (!devdata->paused) {
		ERRDEV(dev_name(&dev->device),
		       "NOT paused, so resume not necessary");
		goto Away;
	}
	devdata->paused = FALSE;
	INFODEV(dev_name(&dev->device), "resumed");
	complete_func(dev, 0);
	rc = 0;
Away:
	if (locked) {
		UNLOCKWRITESEM(&devdata->lockVisorDev);
		locked = FALSE;
	}
	return rc;
}

static int
visorconinclient_init(void)
{
	int rc = 0;

	INFODRV("driver version %s loaded", VERSION);

	/* show module options */
	INFODRV("option - debug=%d", visorconinclient_debug);
	INFODRV("         debugref=%d", visorconinclient_debugref);

	spin_lock_init(&devnopool_lock);
	DevNoPool = kzalloc(BITS_TO_LONGS(MAXDEVICES), GFP_KERNEL);
	if (DevNoPool == NULL) {
		ERRDRV("Unable to create DevNoPool");
		rc = -1;
		goto Away;
	}
	visor_easyproc_InitDriver(&Easyproc_driver_info,
				  MYDRVNAME,
				  visorconinclient_show_driver_info,
				  visorconinclient_show_device_info);
	visorbus_register_visor_driver(&visorconinclient_driver);

Away:
	if (rc < 0)
		visorconinclient_cleanup_guts();
	return rc;
}

static void
visorconinclient_cleanup(void)
{
	visorconinclient_cleanup_guts();
	INFODRV("driver unloaded");
}

static void
visorconinclient_show_device_info(struct seq_file *seq, void *p)
{
	struct visorconinclient_devdata *devdata =
	    (struct visorconinclient_devdata *) (p);
	seq_printf(seq, "devno=%d\n", devdata->devno);
	seq_printf(seq, "visorbus name = '%s'\n", devdata->name);
}

static void
visorconinclient_show_driver_info(struct seq_file *seq)
{
	seq_printf(seq, "Version=%s\n", VERSION);
}

module_param_named(debug, visorconinclient_debug, int, S_IRUGO);
MODULE_PARM_DESC(visorconinclient_debug, "1 to debug");
int visorconinclient_debug = 0;

module_param_named(debugref, visorconinclient_debugref, int, S_IRUGO);
MODULE_PARM_DESC(visorconinclient_debugref, "1 to debug reference counts");
int visorconinclient_debugref = 0;

module_init(visorconinclient_init);
module_exit(visorconinclient_cleanup);

MODULE_AUTHOR("Unisys");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sPAR human input driver for guest Linux: ver " VERSION);
MODULE_VERSION(VERSION);
