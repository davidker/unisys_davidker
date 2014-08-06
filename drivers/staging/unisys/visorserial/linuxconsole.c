/* linuxconsole.c
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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/rwsem.h>
#include "visorserial_private.h"
#include "linuxconsole.h"

#define CONSOLE_TYPE_STRING "visor"
#define BUFFER_SIZE 4096

ulong visorserial_console_write_bytes = 0;
ulong visorserial_console_dropped_bytes = 0;

static void *Console_context;
static char Buffer[BUFFER_SIZE + 1];
static int Buffer_ix;
static void (*Transmit_char) (void *, u8);
static DECLARE_RWSEM(Console_lock);

void
lxcon_console_online(void *context, void (*transmit_char) (void *, u8))
{
	down_write(&Console_lock);
	if (context && transmit_char) {
		int i = 0;
		Console_context = context;
		Transmit_char = transmit_char;
		for (i = 0; i < Buffer_ix; i++)
			(*Transmit_char) (Console_context, Buffer[i]);
	}
	up_write(&Console_lock);
}

void
lxcon_console_offline(void *context)
{
	down_read(&Console_lock);
	Console_context = NULL;
	Transmit_char = NULL;
	up_read(&Console_lock);
}

char *
lxcon_get_early_buffer(void)
{
	Buffer[Buffer_ix] = '\0';
	return Buffer;
}

#ifndef MODULE			/* needed because of omission in linux/init.h */
static void
lxcon_console_write(struct console *co, const char *s, unsigned count)
{
	int i = 0;

	if (count == 0)
		return;
	down_write(&Console_lock);
	visorserial_console_write_bytes += count;
	if (Console_context == NULL) {
		if (Buffer_ix + count <= BUFFER_SIZE) {
			memcpy(Buffer + Buffer_ix, s, count);
			Buffer_ix += count;
		} else
			visorserial_console_dropped_bytes += count;
	} else {
		for (i = 0; i < count; i++)
			(*Transmit_char) (Console_context, s[i]);
	}
	up_write(&Console_lock);
}

static int __init
lxcon_console_setup(struct console *co, char *options)
{
	char *s = options;
	ulong channel_address = 0;
	int rc = 0;

	if (!options) {
		if (visorserial_channeladdress != 0)
			goto Away; /* channeladdress supplied on module load */
		pr_info("%s - channel address must be specified!\n",
		       __func__);
		rc = -ENODEV;
		goto Away;
	}
	pr_info("%s - options='%s'\n", __func__, options);
	if (kstrtoul(s, 0, &channel_address)) {
		pr_info("%s - channel address is NULL!\n", __func__);
		rc = -ENODEV;
		goto Away;
	}
	while (*s != '\0' && *s != ',')
		s++;
	if (*s) {
		pr_info("%s - extraneous console options ('%s')!\n",
		       __func__, s);
		rc = -ENODEV;
		goto Away;
	}
	visorserial_channeladdress = channel_address;

Away:
	if (rc >= 0) {
		pr_info("%s - using %s console @ 0x%lx\n",
		       __func__, CONSOLE_TYPE_STRING,
		       visorserial_channeladdress);
	} else {
		pr_info("%s console will NOT be used\n", CONSOLE_TYPE_STRING);
		return rc;
	}
	return 0;
}

static struct console lxcon_console = {
	.name = CONSOLE_TYPE_STRING,
	.write = lxcon_console_write,
	.device = uart_console_device,
	.setup = lxcon_console_setup,
	.flags = CON_PRINTBUFFER,
	/* Specify which "console=visor<x> to match from the kernel cmd line.
	 * Or specify -1 to match any "console=visor<x>".
	 * Note that "console=visor0" is the same as "console=visor".
	 */
	.index = -1,
	.data = &visorserial_lxser_reg,
};

static int __init
lxcon_console_init(void)
{
	pr_info("%s registering %s console\n",
	       __func__, CONSOLE_TYPE_STRING);
	memset(Buffer, 0, sizeof(Buffer));
	register_console(&lxcon_console);
	return 0;
}

console_initcall(lxcon_console_init);

#endif				/* #ifndef MODULE */
