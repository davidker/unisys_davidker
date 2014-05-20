/* visorserial_private.h
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

#include "timskmod.h"
#include "charqueue.h"
#include "visorbus.h"
#include "visorchannel.h"
#include "consolechannel.h"
#include "version.h"

#define MYDRVNAME "visorserialclient"

#define NHOSTBYTESTOBUFFER      10000	/* buffer size for bytes input from
					 * host */
#define NFILEREADBYTESTOBUFFER  10000	/* buffer size for user read data */
#define NFILEWRITEBYTESTOBUFFER 10000	/* buffer size for user write data */
#define MAXDEVICES     16384

#define OK_TO_BLOCK_FOR_CONSOLE 0	/* 1 = service partition will go into a
					 * wait loop if the console channel is
					 * full, to avoid dropping chars output
					 * to the console 0 = if console channel
					 * fills up, the service partition will
					 * NOT delay, but console output will be
					 * lost (i.e., dropped chars) */
extern int visorserial_rxtxswap;
extern int visorserial_createttydevice;
extern int visorserial_clearchannel;
extern int visorserial_debug;
extern int visorserial_debugref;
extern ulong visorserial_channeladdress;
extern struct uart_driver visorserial_lxser_reg;
extern ulong visorserial_console_write_bytes;
extern ulong visorserial_console_dropped_bytes;
int visorserial_init(void);
