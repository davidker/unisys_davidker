/* Copyright � 2010 - 2013 UNISYS CORPORATION
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

#include <linux/types.h>
#include <linux/uuid.h>

#include "channel.h"

/* {F2DB76C2-1C43-4f24-A5DA-A28EE66A7480} */
#define SPAR_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL_UUID	\
	UUID_LE(0xf2db76c2, 0x1c43, 0x4f24,				\
		0xa5, 0xda, 0xa2, 0x8e, 0xe6, 0x6a, 0x74, 0x80)

static const uuid_le spar_console_framebuffer_memory_channel_protocol_uuid =
	SPAR_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL_UUID;

#define SPAR_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL_SIGNATURE \
	SPAR_CHANNEL_PROTOCOL_SIGNATURE
/* Must increment this whenever you insert or delete fields within
* this channel struct.  Also increment whenever you change the meaning
* of fields within this channel struct so as to break pre-existing
* software.  Note that you can usually add fields to the END of the
* channel struct withOUT needing to increment this. */
#define SPAR_CONSOLEFRAMEBUFFERMEMORY_CHANNEL_PROTOCOL_VERSIONID 1

#pragma pack(push, 1)		/* both GCC and VC now allow this pragma */
struct spar_consoleframebuffermemory_channel_protocol {
	struct channel_header header;
	u8 framebuffer[1024 * 1024 * 3]; /* frame buffer big enough
					  * for 1024x768x32 */
};

#define CONSOLEFRAMEBUFFERMEMORY_CH_SIZE \
	COVER(sizeof(struct spar_consoleframebuffermemory_channel_protocol), \
	      65536)
#pragma pack(pop)

#endif
