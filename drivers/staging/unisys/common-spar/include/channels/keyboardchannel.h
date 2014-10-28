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

#ifndef __KEYBOARDCHANNEL_H__
#define __KEYBOARDCHANNEL_H__

#include <linux/uuid.h>

#include "channel.h"
#include "ultrainputreport.h"

/* {C73416D0-B0B8-44af-B304-9D2AE99F1B3D} */
#define SPAR_KEYBOARD_CHANNEL_PROTOCOL_UUID				\
	UUID_LE(0xc73416d0, 0xb0b8, 0x44af,				\
		0xb3, 0x4, 0x9d, 0x2a, 0xe9, 0x9f, 0x1b, 0x3d)
static const uuid_le spar_keyboard_channel_protocol_uuid =
	SPAR_KEYBOARD_CHANNEL_PROTOCOL_UUID;
#define SPAR_KEYBOARD_CHANNEL_PROTOCOL_SIGNATURE \
	ULTRA_CHANNEL_PROTOCOL_SIGNATURE

/* Must increment this whenever you insert or delete fields within this channel
* struct.  Also increment whenever you change the meaning of fields within this
* channel struct so as to break pre-existing software.  Note that you can
* usually add fields to the END of the channel struct withOUT needing to
* increment this. */
#define SPAR_KEYBOARD_CHANNEL_PROTOCOL_VERSIONID 1

#define KEYBOARD_MAXINPUTREPORTS 50

#pragma pack(push, 1)		/* both GCC and VC now allow this pragma */
struct spar_keyboard_channel_protocol {
	struct channel_header header;	/* /< Generic Channel Protocol
						 * Header */
	u32 n_input_reports;	/* /< max # entries in <inputReport> */
	struct {
		u32 yield_to_boot:1;/* This is a convenience for the EFI demo
				     * environment only.  This bit is set to 1
				     * by the client guest as a signal to to
				     * the EFI boot partition (who is NOT the
				     * server side of this channel, by the way)
				     * that it needs to do a "yield boot".
				     * This enables us to do a "yield boot" in
				     * environments where there are NO EFI
				     * vConsole clients where a "yield boot"
				     * can be done.
				     */
		/* remaining bits in this 32-bit word are available */
	} flags;
	struct signal_queue_header input_report_q;
	ULTRA_INPUTREPORT input_report[KEYBOARD_MAXINPUTREPORTS];
};

#define KEYBOARD_CH_SIZE COVER(sizeof(struct spar_keyboard_channel_protocol), \
			       4096)

static inline void
ULTRA_KEYBOARD_init_channel(struct spar_keyboard_channel_protocol *x)
{
	memset(x, 0, sizeof(struct spar_keyboard_channel_protocol));
	x->header.version_id = SPAR_KEYBOARD_CHANNEL_PROTOCOL_VERSIONID;
	x->header.signature = SPAR_KEYBOARD_CHANNEL_PROTOCOL_SIGNATURE;
	x->header.srv_state = CHANNELSRV_UNINITIALIZED;
	x->header.header_size = sizeof(x->header);
	x->header.size = KEYBOARD_CH_SIZE;
	x->header.chtype = spar_keyboard_channel_protocol_uuid;
	x->header.zone_uuid = NULL_UUID_LE;
	spar_signal_init(x, input_report_q, input_report, ULTRA_INPUTREPORT,
			 0, 0);
	x->header.ch_space_offset =
	    offsetof(struct spar_keyboard_channel_protocol, input_report_q);
	x->n_input_reports = KEYBOARD_MAXINPUTREPORTS;
}

#pragma pack(pop)

#endif
