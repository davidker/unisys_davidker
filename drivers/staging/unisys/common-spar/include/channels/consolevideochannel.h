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

#ifndef __CONSOLEVIDEO_H__
#define __CONSOLEVIDEO_H__

#include "commontypes.h"
#include "controlframework.h"
#include "consoleframebufferchannel.h"
#include "consoleframebuffermemorychannel.h"

/*  This is simply a composite wrapper for the real console video
 *  channels.  The reason we need 3 video channels is to enforce the
 *  desired memory protection for the 3 different environments
 *  involved. The reason we want a single composite is so that drivers
 *  can have a single channel (the composite) for each video device.
 *  All controlvm device messages are delivered using the GUID of this
 *  composite channel (ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL_GUID); the
 *  GUIDs for the containing channels are never specified in controlvm
 *  messages.
 */

/* {3CD6E705-D6A2-4aa5-AD5C-7B08889DFFE2} */

#define ULTRA_CONSOLELEGACYVIDEO_CHANNEL_PROTOCOL_GUID		\
	UUID_LE(0x3cd6e705, 0xd6a2, 0x4aa5,				\
		0xad, 0x5c, 0x7b, 0x8, 0x88, 0x9d, 0xff, 0xe2)


#define ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL_GUID \
	ULTRA_CONSOLELEGACYVIDEO_CHANNEL_PROTOCOL_GUID
static const uuid_le UltraConsoleVideoChannelProtocolGuid =
	ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL_GUID;

#define ULTRA_CONSOLEVIDEO_PRIMARY_CHANNEL_PROTOCOL_GUID	\
	UUID_LE(0xbae361b7, 0x820e, 0x4794,				\
		0x89, 0x2e, 0x23, 0xcc, 0x1b, 0xbc, 0xb7, 0x88)
static const uuid_le UltraConsoleVideoPrimaryChannelProtocolGuid =
	ULTRA_CONSOLEVIDEO_PRIMARY_CHANNEL_PROTOCOL_GUID;

#define ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL_SIGNATURE \
	ULTRA_CONSOLELEGACYVIDEO_CHANNEL_PROTOCOL_SIGNATURE

#define ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL_VERSIONID \
	ULTRA_CONSOLELEGACYVIDEO_CHANNEL_PROTOCOL_VERSIONID

#define ULTRA_CONSOLEVIDEO_CHANNEL_OK_CLIENT(pChannel, logCtx)		\
	(ULTRA_check_channel_client(pChannel, \
				UltraConsoleVideoPrimaryChannelProtocolGuid, \
				"consolevideo",			\
				\0, /* this composite channel has no header */ \
				ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL_VERSIONID, \
				ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL_SIGNATURE, \
				__FILE__, __LINE__, logCtx))
#define ULTRA_CONSOLEVIDEO_CHANNEL_OK_SERVER(actualBytes, logCtx)	\
	(ULTRA_check_channel_server(UltraConsoleVideoPrimaryChannelProtocolGuid, \
				  "consolevideo",			\
				  sizeof(ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL), \
				  actualBytes,			\
				  __FILE__, __LINE__, logCtx))
#pragma pack(push, 1)		/* both GCC and VC now allow this pragma */

/* size of the space reserved for legacy video */
#define CONSOLELEGACYVIDEO_CH_SIZE 395009
typedef struct _ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL {
	    /* space reserved for legacy video use: */
	u8 reserved[CONSOLELEGACYVIDEO_CH_SIZE];

	    /* Allowed access by host driver clients, and service partition: */
	u8 FrameBufferChannel[CONSOLEFRAMEBUFFER_CH_SIZE];

	    /* Allowed access by everyone: */
	u8 FrameBufferMemoryChannel[CONSOLEFRAMEBUFFERMEMORY_CH_SIZE];
} ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL;

/* Make it an even multiple of small pages: */
#define CONSOLEVIDEO_CH_SIZE \
	COVER(sizeof(ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL), ULTRA_MEMORY_PAGE_Ki)

static inline void
ULTRA_CONSOLEFRAMEBUFFER_init_offsets(CONSOLEFRAMEBUFFER_FIRMWAREVIDEODATA *fw)
{

	    /* This assumes the consoleframebuffer channel instance is
	     * contained within a composite consolevideo channel. */
	    ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL *x =
	    (ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL *) NULL;
	fw->frameBufferBytes = sizeof(x->FrameBuffer);
	fw->offsetToFrameBuffer =
	    offsetof(ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL,
		      FrameBufferMemoryChannel) +
	    offsetof(ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL,
		      FrameBuffer);
	fw->offsetToFrameBuffer -=
	    offsetof(ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL, FrameBufferChannel);

	/*structure starts on the second byte of reserved area */
	fw->offsetToLegacyVideo = offsetof(ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL,
					   reserved) + 1;
	fw->offsetToLegacyVideo -=
	    offsetof(ULTRA_CONSOLEVIDEO_CHANNEL_PROTOCOL, FrameBufferChannel);
}


#pragma pack(pop)

#endif
