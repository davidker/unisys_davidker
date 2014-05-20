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

#ifndef __CONSOLECHANNEL_H__
#define __CONSOLECHANNEL_H__

#include <linux/uuid.h>

#include "commontypes.h"
#include "channel.h"

#define ULTRA_CONSOLE_CHANNEL_PROTOCOL_GUID  \
	UUID_LE(0x9bbc3671, 0x5aea, 0x44a8, \
		0xa9, 0xff, 0xab, 0x65, 0x8e, 0xdf, 0x83, 0x9c)
static const uuid_le UltraConsoleChannelProtocolGuid =
    ULTRA_CONSOLE_CHANNEL_PROTOCOL_GUID;

/* {BFE91F41-45E1-4ad8-8676-D46420810841} */
#define ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL_GUID  \
	UUID_LE(0xbfe91f41, 0x45e1, 0x4ad8, \
		0x86, 0x76, 0xd4, 0x64, 0x20, 0x81, 0x08, 0x41)
static const uuid_le UltraConsoleSerialChannelProtocolGuid =
    ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL_GUID;

#define ULTRA_CONSOLE_CHANNEL_PROTOCOL_SIGNATURE \
	ULTRA_CHANNEL_PROTOCOL_SIGNATURE
#define ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL_SIGNATURE \
	ULTRA_CHANNEL_PROTOCOL_SIGNATURE

/* Must increment this whenever you insert or delete fields within
* this channel struct.  Also increment whenever you change the meaning
* of fields within this channel struct so as to break pre-existing
* software.  Note that you can usually add fields to the END of the
* channel struct withOUT needing to increment this. */
#define ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL_VERSIONID 2
#define ULTRA_CONSOLE_CHANNEL_PROTOCOL_VERSIONID       2	/* deprecated */
#define ULTRA_CONSOLESERIAL_CHANNEL_OK_CLIENT(pChannel, logCtx)       \
	    ULTRA_check_channel_client                               \
			     (pChannel,                              \
			      UltraConsoleSerialChannelProtocolGuid, \
			      "consoleserial",                       \
			      sizeof(ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL),\
			      ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL_VERSIONID,\
			      ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL_SIGNATURE,\
			      __FILE__, __LINE__, logCtx               \
)
#define ULTRA_CONSOLESERIAL_CHANNEL_OK_SERVER(actualBytes, logCtx)    \
	    ULTRA_check_channel_server                               \
			     (UltraConsoleSerialChannelProtocolGuid, \
			      "consoleserial",                       \
			      sizeof(ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL),\
			      actualBytes,                           \
			      __FILE__, __LINE__, logCtx               \
)

#define ULTRA_CONSOLE_CHANNEL_OK_CLIENT(pChannel, logCtx) /*deprecated*/ \
	    ULTRA_check_channel_client                               \
			     (pChannel,                              \
			      UltraConsoleChannelProtocolGuid,       \
			      "console",                             \
			      sizeof(ULTRA_CONSOLE_CHANNEL_PROTOCOL),\
			      ULTRA_CONSOLE_CHANNEL_PROTOCOL_VERSIONID,\
			      ULTRA_CONSOLE_CHANNEL_PROTOCOL_SIGNATURE,\
			      __FILE__, __LINE__, logCtx               \
)
#define ULTRA_CONSOLE_CHANNEL_OK_SERVER(actualBytes, logCtx) /*deprecated*/ \
	    ULTRA_check_channel_server                               \
			     (UltraConsoleChannelProtocolGuid,       \
			      "console",                             \
			      sizeof(ULTRA_CONSOLE_CHANNEL_PROTOCOL),\
			      actualBytes,                           \
			      __FILE__, __LINE__, logCtx               \
)

/* INPUT/OUTPUT buffer size */
#define CONSOLE_IN_MAX_BUFFER_SIZE				512
#define CONSOLE_OUT_MAX_BUFFER_SIZE				3072
/* OUT 3072 for 1 page channel, 15360 for 4 page channel, additional
* pages provide larger OUT queue.  The
* GuestLinux/source/visorserial/visorserial_main.c uses the supplied
* channel size.  The MAX value is arbitrary but double the typical
* value needed to contain an entire normal AppOS boot sequence. */
#define CONSOLE_OUT_EXTRA_SMALL  (3*4096)
#define CONSOLE_OUT_EXTRA_MEDIUM (7*4096)
#define CONSOLE_OUT_EXTRA_MAX    (15*4096)

/* Defines for channel queues... */
#define CONSOLESERIAL_QUEUE_IN		0
#define CONSOLESERIAL_QUEUE_OUT		1

/* Other characteristics of the Console Channel */
typedef enum 
    { UART8250 = 0, UART16450 = 1, UART16550 = 2, UART16550A = 3 
} EFI_UART_TYPE;
 
/* Copied from Foundation/Efi/Protocol/SerialIo/SerialIo.h
* EFI_SERIAL_IO_MODE structure - copied here so AppOS (Diag) can make
* use of it in CONSOLE_CONTROL structure for db-all port...  */
typedef struct _ULTRA_SERIAL_IO_MODE  {
	U32 ControlMask;
	
	    /*  */
	    /* current Attributes */
	    /*  */
	U32 Timeout;
	U64 BaudRate;
	U32 ReceiveFifoDepth;
	U32 DataBits;
	U32 Parity;
	U32 StopBits;
} ULTRA_SERIAL_IO_MODE;
 typedef struct _CONSOLE_CONTROL  {
	EFI_UART_TYPE DeviceType;
	U8 IsActive;		/* /< If this channel active */
	U8 ControlChanged;	/* /< If SetControl is called */
	U8 AttributesChanged;	/* /< If SetAttributes is called */
	U8 YieldingToBoot;	/* /< This console channel has become inactive because it is yielding to the boot */
	U8 ReservedA;		/* /< This console channel wishes to be active */
	U8 GuestIsDead;		/* /< The guest is no longer running */
	U8 Reserved[2];		/* /< For alignment */
	U32 WaitCount;		/* /< Number of waits */
	ULTRA_SERIAL_IO_MODE Mode;	/* /< 32 bytes */
	/* If you increase the size of this structure, please also
	* check the value of SIZEOF_CONSOLE_CONTROL below, to be sure
	* it is still large enough. */
} CONSOLE_CONTROL, *LPCONSOLE_CONTROL;
 
#pragma pack(push, 1)		/* both GCC and VC now allow this pragma */
/* Defines for constants */
#define SIZEOF_CONSOLE_CONTROL 48+80 /*(sizeof(CONSOLE_CONTROL) + alignment) */

/*
 *       ULTRA_CONSOLE_CHANNEL_PROTOCOL definition:
 */
    typedef struct _ULTRA_CONSOLE_CHANNEL_PROTOCOL {
	ULTRA_CHANNEL_PROTOCOL ChannelHeader;	/* < Generic Channel
						 * Protocol Header */

	/* Control is only needed for the EFI-only demo environment */
	U8 Control[SIZEOF_CONSOLE_CONTROL];
	SIGNAL_QUEUE_HEADER InQ;
	SIGNAL_QUEUE_HEADER OutQ;
	U8 InData[CONSOLE_IN_MAX_BUFFER_SIZE];
	U8 OutData[CONSOLE_OUT_MAX_BUFFER_SIZE];
	/* OutData must remain last field of channel.  It can by
	 * expanded by allocating extra channel memory */
} ULTRA_CONSOLE_CHANNEL_PROTOCOL, ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL;

/* NOTE: ULTRA_CONSOLE_CHANNEL_PROTOCOL name has been deprecated - do
 * not use (use ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL instead)... */
#define CONSOLE_CH_SIZE COVER(sizeof(ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL), \
			      4096)

static inline void
ULTRA_CONSOLESERIAL_init_channel(ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL * x)
{
	memset(x, 0, sizeof (ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL));
	x->ChannelHeader.VersionId =
	    ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL_VERSIONID;
	x->ChannelHeader.Signature =
	    ULTRA_CONSOLESERIAL_CHANNEL_PROTOCOL_SIGNATURE;
	/* x->ChannelHeader.LegacyState = CHANNEL_ATTACHING; */
	x->ChannelHeader.SrvState = CHANNELSRV_READY;
	x->ChannelHeader.HeaderSize = sizeof (x->ChannelHeader);
	x->ChannelHeader.Size = CONSOLE_CH_SIZE;
	x->ChannelHeader.Type = UltraConsoleChannelProtocolGuid;
	/* x->ChannelHeader.Type = UltraConsoleSerialChannelProtocolGuid; */
	x->ChannelHeader.ZoneGuid = NULL_UUID_LE;
	SignalInit(x, InQ, InData, U8, 0, 0);
	SignalInit(x, OutQ, OutData, U8, 0, 0);
	x->ChannelHeader.oChannelSpace =
	    offsetof(ULTRA_CONSOLE_CHANNEL_PROTOCOL, InQ);
}

#pragma pack(pop)

#endif
