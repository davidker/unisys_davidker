/* visordiag.h
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

#include "diagchannel.h"

#define VISORDIAG_MIN(a, b)     (((a) < (b)) ? (a) : (b))

#define VISORDIAG_MIN_SEVERITY_FOR_SUBSYS(filter, subsys) \
	(readb(&filter[subsys]) & SEVERITY_FILTER_MASK)

/* Don't let ANYONE filter out our LOG_INFO, LOG_WARNING, and LOG_ERR messages.
* Only LOG_DEBUG (DIAG_SEVERITY_VERBOSE) messages can be filtered out.
* NOTE: This is not currently used anywhere. */
#define VISORDIAG_MIN_SEVERITY_FOR_SUBSYS_ADJUSTED(filter, subsys) \
	VISORDIAG_MIN(VISORDIAG_MIN_SEVERITY_FOR_SUBSYS(filter, subsys), \
		      DIAG_SEVERITY_INFO)

/* Usermode mmap() info for visordiag device */
#define VISORDIAG_MMAP_CHANNEL_OFF    0
#define VISORDIAG_MMAP_CHANNEL_BYTES  sizeof(ULTRA_DIAG_CHANNEL_PROTOCOL)
#define VISORDIAG_DEVNAME  "visordiag"

/*  These are syslog priority codes, defined in /usr/include/sys/syslog.h.
 *  The priority code is always encoded as the low 3 bits of a <pri> value.
 */
#define	SYSLOG_PRI_EMERG   0
#define	SYSLOG_PRI_ALERT   1
#define	SYSLOG_PRI_CRIT    2
#define	SYSLOG_PRI_ERR     3
#define	SYSLOG_PRI_WARNING 4
#define	SYSLOG_PRI_NOTICE  5
#define	SYSLOG_PRI_INFO    6
#define	SYSLOG_PRI_DEBUG   7

#define SYSLOG_GET_PRIORITY(pri) (pri & 0x7)

static inline u32
pri_to_severity(int pri)
{
	switch (SYSLOG_GET_PRIORITY(pri)) {
	case SYSLOG_PRI_EMERG:
	case SYSLOG_PRI_ALERT:
	case SYSLOG_PRI_CRIT:
	case SYSLOG_PRI_ERR:
		return DIAG_SEVERITY_ERR;
	case SYSLOG_PRI_WARNING:
	case SYSLOG_PRI_NOTICE:
		return DIAG_SEVERITY_WARNING;
	case SYSLOG_PRI_INFO:
		return DIAG_SEVERITY_INFO;
	case SYSLOG_PRI_DEBUG:
		return DIAG_SEVERITY_VERBOSE;
	default:
		return DIAG_SEVERITY_INFO;
	}
	return DIAG_SEVERITY_INFO;
}

/* Grab a pointer to the subsystem severity filter array in the channel
 * header */
char __iomem *visordiag_get_severityfilter(void);
/* Return reference obtained previously via visordiag_get_severityfilter() */
void visordiag_release_severityfilter(char *filter);
