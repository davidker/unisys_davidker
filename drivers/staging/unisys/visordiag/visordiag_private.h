/* visordiag_private.h
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

#include "timskmod.h"
#include "visorbus.h"
#include "visorchannel.h"
#include "diagchannel.h"
#include "version.h"
#include "visordiag.h"
#include <linux/ctype.h>

#define MYDRVNAME "visordiag"

#define MAXDEVICES     16384
#define NFILEWRITEBYTESTOBUFFER 10000	/* buffer size for user write data */
#define SYSLOG_MAKE_PRI(facility, priority) (((facility) << 3) | (priority))

#define SYSLOG_GET_FACILITY(pri) (pri >> 3)

/*  These are syslog priority codes, defined in /usr/include/sys/syslog.h.
 *  The facility code is always encoded as the upper 29 bits of a 32-bit
 *  <pri> value.
 */
#define	SYSLOG_FAC_KERN	     0
#define	SYSLOG_FAC_USER      1
#define	SYSLOG_FAC_MAIL      2
#define	SYSLOG_FAC_DAEMON    3
#define	SYSLOG_FAC_AUTH      4
#define	SYSLOG_FAC_SYSLOG    5
#define	SYSLOG_FAC_LPR       6
#define	SYSLOG_FAC_NEWS      7
#define	SYSLOG_FAC_UUCP      8
#define	SYSLOG_FAC_CRON      9
#define	SYSLOG_FAC_AUTHPRIV 10
#define	SYSLOG_FAC_FTP      11
#define	SYSLOG_FAC_LOCAL0   16
#define	SYSLOG_FAC_LOCAL1   17
#define	SYSLOG_FAC_LOCAL2   18
#define	SYSLOG_FAC_LOCAL3   19
#define	SYSLOG_FAC_LOCAL4   20
#define	SYSLOG_FAC_LOCAL5   21
#define	SYSLOG_FAC_LOCAL6   22
#define	SYSLOG_FAC_LOCAL7   23
