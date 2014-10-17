/* linuxconsole.h
 *
 * Copyright (c) 2010 - 2014 UNISYS CORPORATION
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

#ifndef __LINUXCONSOLE_H__
#define __LINUXCONSOLE_H__

#include "timskmod.h"

void lxcon_console_online(void *context,
			  void (*transmit_char)(void *, u8));
void lxcon_console_offline(void *context);
char *lxcon_get_early_buffer(void);

#endif
