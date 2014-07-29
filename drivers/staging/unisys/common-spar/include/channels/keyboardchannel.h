/* Copyright © 2010 - 2013 UNISYS CORPORATION
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

#include "commontypes.h"
#include "channel.h"
#include "ultrainputreport.h"

/* {C73416D0-B0B8-44af-B304-9D2AE99F1B3D} */
#define ULTRA_KEYBOARD_CHANNEL_PROTOCOL_GUID				\
	UUID_LE(0xc73416d0, 0xb0b8, 0x44af,				\
		0xb3, 0x4, 0x9d, 0x2a, 0xe9, 0x9f, 0x1b, 0x3d)
static const uuid_le UltraKeyboardChannelProtocolGuid =
	ULTRA_KEYBOARD_CHANNEL_PROTOCOL_GUID;
#define ULTRA_KEYBOARD_CHANNEL_PROTOCOL_SIGNATURE \
	ULTRA_CHANNEL_PROTOCOL_SIGNATURE

/* Must increment this whenever you insert or delete fields within this channel
* struct.  Also increment whenever you change the meaning of fields within this
* channel struct so as to break pre-existing software.  Note that you can
* usually add fields to the END of the channel struct withOUT needing to
* increment this. */
#define ULTRA_KEYBOARD_CHANNEL_PROTOCOL_VERSIONID 1

#define ULTRA_KEYBOARD_CHANNEL_OK_CLIENT(pChannel, logCtx)            \
	(ULTRA_check_channel_client(pChannel,                              \
				    UltraKeyboardChannelProtocolGuid,	\
				    "keyboard",				\
				    sizeof(ULTRA_KEYBOARD_CHANNEL_PROTOCOL), \
				    ULTRA_KEYBOARD_CHANNEL_PROTOCOL_VERSIONID, \
				    ULTRA_KEYBOARD_CHANNEL_PROTOCOL_SIGNATURE, \
				    __FILE__, __LINE__, logCtx))

#define ULTRA_KEYBOARD_CHANNEL_OK_SERVER(actualBytes, logCtx)         \
	(ULTRA_check_channel_server(UltraKeyboardChannelProtocolGuid, \
				    "keyboard",				\
				    sizeof(ULTRA_KEYBOARD_CHANNEL_PROTOCOL), \
				    actualBytes,			\
				    __FILE__, __LINE__, logCtx))

#define KEYBOARD_MAXINPUTREPORTS 50

#pragma pack(push, 1)		/* both GCC and VC now allow this pragma */
typedef struct _ULTRA_KEYBOARD_CHANNEL_PROTOCOL {
	ULTRA_CHANNEL_PROTOCOL ChannelHeader;	/* /< Generic Channel Protocol
						 * Header */
	u32 nInputReports;	/* /< max # entries in <inputReport> */
	struct {
		u32 yieldToBoot:1; /**< This is a convenience for the EFI demo
				    *   environment only.  This bit is set to 1
				    *   by the client guest as a signal to to
				    *   the EFI boot partition (who is NOT the
				    *   server side of this channel, by the way)
				    *   that it needs to do a "yield boot".
				    *   This enables us to do a "yield boot" in
				    *   environments where there are NO EFI
				    *   vConsole clients where a "yield boot"
				    *   can be done.
				    */
		/* remaining bits in this 32-bit word are available */
	} flags;
	SIGNAL_QUEUE_HEADER inputReportQ;
	ULTRA_INPUTREPORT inputReport[KEYBOARD_MAXINPUTREPORTS];
} ULTRA_KEYBOARD_CHANNEL_PROTOCOL;

#define KEYBOARD_CH_SIZE COVER(sizeof(ULTRA_KEYBOARD_CHANNEL_PROTOCOL), 4096)

static inline void
ULTRA_KEYBOARD_init_channel(ULTRA_KEYBOARD_CHANNEL_PROTOCOL *x)
{
	memset(x, 0, sizeof(ULTRA_KEYBOARD_CHANNEL_PROTOCOL));
	x->ChannelHeader.VersionId = ULTRA_KEYBOARD_CHANNEL_PROTOCOL_VERSIONID;
	x->ChannelHeader.Signature = ULTRA_KEYBOARD_CHANNEL_PROTOCOL_SIGNATURE;
	x->ChannelHeader.SrvState = CHANNELSRV_UNINITIALIZED;
	x->ChannelHeader.HeaderSize = sizeof(x->ChannelHeader);
	x->ChannelHeader.Size = KEYBOARD_CH_SIZE;
	x->ChannelHeader.Type = UltraKeyboardChannelProtocolGuid;
	x->ChannelHeader.ZoneGuid = NULL_UUID_LE;
	SignalInit(x, inputReportQ, inputReport, ULTRA_INPUTREPORT, 0, 0);
	x->ChannelHeader.oChannelSpace =
	    offsetof(ULTRA_KEYBOARD_CHANNEL_PROTOCOL, inputReportQ);
	x->nInputReports = KEYBOARD_MAXINPUTREPORTS;
}

#pragma pack(pop)

#endif
