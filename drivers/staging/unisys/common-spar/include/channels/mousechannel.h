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

#ifndef __MOUSECHANNEL_H__
#define __MOUSECHANNEL_H__

#include <linux/types.h>
#include <linux/uuid.h>

#include "channel.h"
#include "ultrainputreport.h"

/* {ADDF07D4-94A9-46e2-81C3-61ABCDBDBD87} */
#define ULTRA_MOUSE_CHANNEL_PROTOCOL_GUID  \
	UUID_LE(0xaddf07d4, 0x94a9, 0x46e2, \
		0x81, 0xc3, 0x61, 0xab, 0xcd, 0xbd, 0xbd, 0x87)
static const uuid_le UltraMouseChannelProtocolGuid =
	ULTRA_MOUSE_CHANNEL_PROTOCOL_GUID;
#define ULTRA_MOUSE_CHANNEL_PROTOCOL_SIGNATURE ULTRA_CHANNEL_PROTOCOL_SIGNATURE

/* Must increment this whenever you insert or delete fields within this channel
* struct.  Also increment whenever you change the meaning of fields within this
* channel struct so as to break pre-existing software.  Note that you can
* usually add fields to the END of the channel struct withOUT needing to
* increment this. */
#define ULTRA_MOUSE_CHANNEL_PROTOCOL_VERSIONID 1

#define ULTRA_MOUSE_CHANNEL_OK_CLIENT(pChannel, logCtx)			\
	(ULTRA_check_channel_client(pChannel,				\
				    UltraMouseChannelProtocolGuid,	\
				    "mouse",				\
				    sizeof(ULTRA_MOUSE_CHANNEL_PROTOCOL), \
				    ULTRA_MOUSE_CHANNEL_PROTOCOL_VERSIONID, \
				    ULTRA_MOUSE_CHANNEL_PROTOCOL_SIGNATURE, \
				    __FILE__, __LINE__, logCtx))

#define ULTRA_MOUSE_CHANNEL_OK_SERVER(actualBytes, logCtx)		\
	(ULTRA_check_channel_server(UltraMouseChannelProtocolGuid,	\
				    "mouse",				\
				    sizeof(ULTRA_MOUSE_CHANNEL_PROTOCOL), \
				    actualBytes,			\
				    __FILE__, __LINE__, logCtx))

#define MOUSE_MAXINPUTREPORTS 50

#pragma pack(push, 1)		/* both GCC and VC now allow this pragma */
typedef struct _ULTRA_MOUSE_CHANNEL_PROTOCOL {
	ULTRA_CHANNEL_PROTOCOL ChannelHeader;	/* /< Generic Channel Protocol
						 * Header */
	u32 nInputReports;	/* /< max # entries in <inputReport> */
	u32 filler1;
	SIGNAL_QUEUE_HEADER inputReportQ;
	ULTRA_INPUTREPORT inputReport[MOUSE_MAXINPUTREPORTS];
} ULTRA_MOUSE_CHANNEL_PROTOCOL;

#define MOUSE_CH_SIZE COVER(sizeof(ULTRA_MOUSE_CHANNEL_PROTOCOL), 4096)

static inline void
ULTRA_MOUSE_init_channel(ULTRA_MOUSE_CHANNEL_PROTOCOL *x)
{
	memset(x, 0, sizeof(ULTRA_MOUSE_CHANNEL_PROTOCOL));
	x->ChannelHeader.VersionId = ULTRA_MOUSE_CHANNEL_PROTOCOL_VERSIONID;
	x->ChannelHeader.Signature = ULTRA_MOUSE_CHANNEL_PROTOCOL_SIGNATURE;
	x->ChannelHeader.SrvState = CHANNELSRV_UNINITIALIZED;
	x->ChannelHeader.HeaderSize = sizeof(x->ChannelHeader);
	x->ChannelHeader.Size = MOUSE_CH_SIZE;
	x->ChannelHeader.Type = UltraMouseChannelProtocolGuid;
	x->ChannelHeader.ZoneGuid = NULL_UUID_LE;
	SignalInit(x, inputReportQ, inputReport, ULTRA_INPUTREPORT, 0, 0);
	x->ChannelHeader.oChannelSpace =
	    offsetof(ULTRA_MOUSE_CHANNEL_PROTOCOL, inputReportQ);
	x->nInputReports = MOUSE_MAXINPUTREPORTS;
}

#pragma pack(pop)

#endif
