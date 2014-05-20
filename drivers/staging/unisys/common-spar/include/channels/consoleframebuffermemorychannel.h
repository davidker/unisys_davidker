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

#ifndef __CONSOLEFRAMEBUFFERMEMORY_H__
#define __CONSOLEFRAMEBUFFERMEMORY_H__

#include "commontypes.h"
#include "channel.h"

/* {F2DB76C2-1C43-4f24-A5DA-A28EE66A7480} */
#define ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL_GUID	\
	{0xf2db76c2, 0x1c43, 0x4f24,					\
		{0xa5, 0xda, 0xa2, 0x8e, 0xe6, 0x6a, 0x74, 0x80}	\
	}

static const GUID UltraConsoleFrameBufferMemoryChannelProtocolGuid =
	ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL_GUID;

#define ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL_SIGNATURE \
	ULTRA_CHANNEL_PROTOCOL_SIGNATURE
/* Must increment this whenever you insert or delete fields within
* this channel struct.  Also increment whenever you change the meaning
* of fields within this channel struct so as to break pre-existing
* software.  Note that you can usually add fields to the END of the
* channel struct withOUT needing to increment this. */
#define ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL_VERSIONID 1

#define ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_OK_CLIENT(pChannel, logCtx) \
	(ULTRA_check_channel_client((pChannel), \
		UltraConsoleFramebufferMemoryChannelProtocolGuid, \
		"consoleframebuffermemory",		\
		sizeof(ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL), \
		ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL_VERSIONID, \
		ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL_SIGNATURE, \
		__FILE__, __LINE__, logCtx))
#define ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_OK_SERVER(actualBytes, logCtx) \
	(ULTRA_check_channel_server(UltraConsoleFramebufferMemoryChannelProtocolGuid, \
		"consoleframebuffermemory",		\
		sizeof(ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL), \
		actualBytes, __FILE__, __LINE__, logCtx))

#pragma pack(push, 1)		/* both GCC and VC now allow this pragma */
typedef struct _ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL {
	ULTRA_CHANNEL_PROTOCOL Header;
	U8 FrameBuffer[1024 * 1024 * 3]; /* frame buffer big enough
					  * for 1024x768x32 */
} ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL;

#define CONSOLEFRAMEBUFFERMEMORY_CH_SIZE \
	COVER(sizeof(ULTRA_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL), 65536)
#pragma pack(pop)

#endif
