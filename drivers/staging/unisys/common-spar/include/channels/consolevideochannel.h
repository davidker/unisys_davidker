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

#include <linux/types.h>
#include <linux/uuid.h>

#include "controlframework.h"
#include "consoleframebufferchannel.h"
#include "consoleframebuffermemorychannel.h"

/*  This is simply a composite wrapper for the real console video
 *  channels.  The reason we need 3 video channels is to enforce the
 *  desired memory protection for the 3 different environments
 *  involved. The reason we want a single composite is so that drivers
 *  can have a single channel (the composite) for each video device.
 *  All controlvm device messages are delivered using the UUID of this
 *  composite channel (SPAR_CONSOLEVIDEO_CHANNEL_PROTOCOL_UUID); the
 *  UUIDs for the containing channels are never specified in controlvm
 *  messages.
 */

/* {3CD6E705-D6A2-4aa5-AD5C-7B08889DFFE2} */

#define SPAR_CONSOLELEGACYVIDEO_CHANNEL_PROTOCOL_UUID		\
	UUID_LE(0x3cd6e705, 0xd6a2, 0x4aa5,				\
		0xad, 0x5c, 0x7b, 0x8, 0x88, 0x9d, 0xff, 0xe2)

#define SPAR_CONSOLEVIDEO_CHANNEL_PROTOCOL_UUID \
	SPAR_CONSOLELEGACYVIDEO_CHANNEL_PROTOCOL_UUID
static const uuid_le ultra_console_video_channel_protocol_uuid =
	SPAR_CONSOLEVIDEO_CHANNEL_PROTOCOL_UUID;

#define SPAR_CONSOLEVIDEO_PRIMARY_CHANNEL_PROTOCOL_UUID	\
	UUID_LE(0xbae361b7, 0x820e, 0x4794,				\
		0x89, 0x2e, 0x23, 0xcc, 0x1b, 0xbc, 0xb7, 0x88)
static const uuid_le ultra_console_video_primary_channel_protocol_uuid =
	SPAR_CONSOLEVIDEO_PRIMARY_CHANNEL_PROTOCOL_UUID;

#define SPAR_CONSOLEVIDEO_CHANNEL_PROTOCOL_SIGNATURE \
	SPAR_CONSOLELEGACYVIDEO_CHANNEL_PROTOCOL_SIGNATURE

#define SPAR_CONSOLEVIDEO_CHANNEL_PROTOCOL_VERSIONID \
	SPAR_CONSOLELEGACYVIDEO_CHANNEL_PROTOCOL_VERSIONID

#pragma pack(push, 1)		/* both GCC and VC now allow this pragma */

/* size of the space reserved for legacy video */
#define CONSOLELEGACYVIDEO_CH_SIZE 395009
struct spar_consolevideo_channel_protocol {
	    /* space reserved for legacy video use: */
	u8 reserved[CONSOLELEGACYVIDEO_CH_SIZE];

	    /* Allowed access by host driver clients, and service partition: */
	u8 framebuffer_channel[CONSOLEFRAMEBUFFER_CH_SIZE];

	    /* Allowed access by everyone: */
	u8 framebuffermemory_channel[CONSOLEFRAMEBUFFERMEMORY_CH_SIZE];
};

/* Make it an even multiple of small pages: */
#define CONSOLEVIDEO_CH_SIZE \
	COVER(sizeof(struct spar_consolevideo_channel_protocol), \
	      PAGE_SIZE)

static inline void ULTRA_CONSOLEFRAMEBUFFER_init_offsets(
		struct consoleframebuffer_firmwarevideodata *fw)
{
	/* This assumes the consoleframebuffer channel instance is
	 * contained within a composite consolevideo channel. */
	struct spar_consoleframebuffermemory_channel_protocol *x = NULL;

	fw->framebuffer_bytes = sizeof(x->framebuffer);
	fw->framebuffer_offset =
	    offsetof(struct spar_consolevideo_channel_protocol,
		     framebuffermemory_channel) +
	    offsetof(struct spar_consoleframebuffermemory_channel_protocol,
		     framebuffer);
	fw->framebuffer_offset -=
	    offsetof(struct spar_consolevideo_channel_protocol,
		     framebuffer_channel);

	/*structure starts on the second byte of reserved area */
	fw->legacy_video_offset =
		offsetof(struct spar_consolevideo_channel_protocol,
			 reserved) + 1;
	fw->legacy_video_offset -=
	    offsetof(struct spar_consolevideo_channel_protocol,
		     framebuffer_channel);
}

#pragma pack(pop)

#endif
