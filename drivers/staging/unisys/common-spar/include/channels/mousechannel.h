/* Copyright (C) 2010 - 2013 UNISYS CORPORATION
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

#ifndef __MOUSECHANNEL_H__
#define __MOUSECHANNEL_H__

#include <linux/types.h>
#include <linux/uuid.h>

#include "channel.h"
#include "ultrainputreport.h"

/* {ADDF07D4-94A9-46e2-81C3-61ABCDBDBD87} */
#define SPAR_MOUSE_CHANNEL_PROTOCOL_UUID  \
	UUID_LE(0xaddf07d4, 0x94a9, 0x46e2, \
		0x81, 0xc3, 0x61, 0xab, 0xcd, 0xbd, 0xbd, 0x87)
static const uuid_le spar_mouse_channel_protocol_uuid =
	SPAR_MOUSE_CHANNEL_PROTOCOL_UUID;
#define SPAR_MOUSE_CHANNEL_PROTOCOL_SIGNATURE ULTRA_CHANNEL_PROTOCOL_SIGNATURE

/* Must increment this whenever you insert or delete fields within this channel
* struct.  Also increment whenever you change the meaning of fields within this
* channel struct so as to break pre-existing software.  Note that you can
* usually add fields to the END of the channel struct withOUT needing to
* increment this. */
#define SPAR_MOUSE_CHANNEL_PROTOCOL_VERSIONID 1

#define MOUSE_MAXINPUTREPORTS 50

#pragma pack(push, 1)		/* both GCC and VC now allow this pragma */
struct spar_mouse_channel_protocol {
	struct channel_header header;	/* /< Generic Channel Protocol
						 * Header */
	u32 n_input_reports;	/* /< max # entries in <inputReport> */
	u32 filler1;
	struct signal_queue_header input_report_q;
	ULTRA_INPUTREPORT input_report[MOUSE_MAXINPUTREPORTS];
};

#define MOUSE_CH_SIZE COVER(sizeof(struct spar_mouse_channel_protocol), 4096)

static inline void
ULTRA_MOUSE_init_channel(struct spar_mouse_channel_protocol *x)
{
	memset(x, 0, sizeof(struct spar_mouse_channel_protocol));
	x->header.version_id = SPAR_MOUSE_CHANNEL_PROTOCOL_VERSIONID;
	x->header.signature = SPAR_MOUSE_CHANNEL_PROTOCOL_SIGNATURE;
	x->header.srv_state = CHANNELSRV_UNINITIALIZED;
	x->header.header_size = sizeof(x->header);
	x->header.size = MOUSE_CH_SIZE;
	x->header.chtype = spar_mouse_channel_protocol_uuid;
	x->header.zone_uuid = NULL_UUID_LE;
	spar_signal_init(x, input_report_q, input_report, ULTRA_INPUTREPORT,
			 0, 0);
	x->header.ch_space_offset =
	    offsetof(struct spar_mouse_channel_protocol, input_report_q);
	x->n_input_reports = MOUSE_MAXINPUTREPORTS;
}

#pragma pack(pop)

#endif
