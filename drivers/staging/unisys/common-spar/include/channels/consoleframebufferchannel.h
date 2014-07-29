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

#ifndef __CONSOLEFRAMEBUFFER_H__
#define __CONSOLEFRAMEBUFFER_H__

#include <linux/uuid.h>

#include "commontypes.h"
#include "channel.h"
/* Needed for BOCHS_VIDEO_STATE */
#define BX_SUPPORT_VBE      1
#define BX_SUPPORT_CLGD54XX 0
typedef u8 Bit8u;
typedef u16 Bit16u;
typedef u32 bx_bool;
typedef u32 Bit32u;
typedef u64 Bit64u;


#ifdef EFIX64
/* TODO64: Can this be included from .h   */
typedef struct _VIDEO_POINTER_ATTRIBUTES {
	u64 Flags;
	u64 Width;
	u64 Height;
	u64 WidthInBytes;
	u64 Enable;
	u16 Column;
	u16 Row;
	u8 Pixels[1];
} VIDEO_POINTER_ATTRIBUTES, *PVIDEO_POINTER_ATTRIBUTES;

#endif	/*  */

/* {230A065A-39D8-4917-9F05-3ECC5CBF4A4F} */
#define ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_GUID     \
	UUID_LE(0x230a065a, 0x39d8, 0x4917, \
		0x9f, 0x5, 0x3e, 0xcc, 0x5c, 0xbf, 0x4a, 0x4f)
static const uuid_le UltraConsoleFramebufferChannelProtocolGuid =
	ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_GUID;

#define ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_SIGNATURE \
	ULTRA_CHANNEL_PROTOCOL_SIGNATURE
/* Must increment this whenever you insert or delete fields within
* this channel struct.  Also increment whenever you change the meaning
* of fields within this channel struct so as to break pre-existing
* software.  Note that you can usually add fields to the END of the
* channel struct withOUT needing to increment this. */
#define ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_VERSIONID 1

#define ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_OK_CLIENT(pChannel, logCtx)	\
	ULTRA_check_channel_client(pChannel, \
		UltraConsoleFramebufferChannelProtocolGuid, \
		"consoleframebuffer", \
		sizeof(ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL), \
		ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_VERSIONID, \
		ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_SIGNATURE, \
		__FILE__, __LINE__, logCtx)
#define ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_OK_SERVER(actualBytes, logCtx) \
	ULTRA_check_channel_server(UltraConsoleFramebufferChannelProtocolGuid, \
		"consoleframebuffer", \
		sizeof(ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL), \
		actualBytes, __FILE__, __LINE__, logCtx)
#pragma pack(push, 1)		/* both GCC and VC now allow this pragma */

#define CONSOLEFRAMEBUFFER_NPALETTEENTRIES          256
#define CONSOLEFRAMEBUFFER_MAXCURSORSHAPEBYTES    32768
#define CONSOLEFRAMEBUFFER_MAXIMAGECHANGESPERBATCH 1024
#define CONSOLEFRAMEBUFFER_FORCEFIXEDSIZESCANLINES    0	/* 1 for 4096-byte
							 * fixed-size scanlines
							 */

/** Indicates the time-of-day, so the host driver (client)
 *  can inform the firmware (server).
 *  @ingroup coretypes
 */
typedef struct CONSOLEFRAMEBUFFER_TimeOfDay  {
	u16 month;
	u16 day;
	u16 year;
	u16 hour;
	u16 minute;
	u16 second;
	u16 reserved1;		/* make it 64-bit friendly */
	u16 reserved2;
} CONSOLEFRAMEBUFFER_TIMEOFDAY;

/** Describes to the remote partition desktop client how the video
 *  data is compressed.
 *  @ingroup coretypes
 */
typedef enum  { compress_NONE = 0, /*< not compressed */
	    compress_RLE1 = 1, /*< compressed w/ sw run-length (1-byte units) */
	    compress_RLE2 = 2, /*< compressed w/ sw run-length (2-byte units) */
	    compress_RLE3 = 3, /*< compressed w/ sw run-length (3-byte units) */
	    compress_RLE4 = 4, /*< compressed w/ sw run-length (4-byte units) */
	    compress_LZW = 5 /*< compressed w/ hw LZW compression */
} CONSOLEFRAMEBUFFER_COMPRESSIONMODE;

/** Describes a single color in the palette by specifying the alpha,
 *  red, green, and blue (ARGB) color components.
 *  Important note:  the order of the fields within this structure is
 *  dependent upon the order of fields within the Windows
 *  VIDEO_CLUTDATA palette structure.
 *  @ingroup coretypes
 */
typedef struct CONSOLEFRAMEBUFFER_PaletteEntry  {
	u8 red;
	u8 green;
	u8 blue;
	u8 alpha;
} CONSOLEFRAMEBUFFER_PALETTEENTRY;

/** Identifies a rectangular region within the video frame buffer of video
 *  data that has changed.
 *  @ingroup coretypes
 */
typedef struct CONSOLEFRAMEBUFFER_ChangeRectangle  {
	u16 x;	   /**< x-coordinate of pixel at upper-left corner of rect */
	u16 y;	   /**< y-coordinate of pixel at upper-left corner of rect */
	u16 width;/**< width of rectangle  (in pixels) */
	u16 height;
		   /**< height of rectangle (in scanlines) */
} CONSOLEFRAMEBUFFER_CHANGERECTANGLE;

/** Specifies the colors in ARGB format for every color in the
 *  256-color palette; used when #CONSOLEFRAMEBUFFER_PIXELFMT is
 *  #pixelFmt_PALETTEIZED.
 *  @ingroup coretypes
 */
typedef struct CONSOLEFRAMEBUFFER_Palette  {
	CONSOLEFRAMEBUFFER_PALETTEENTRY paletteEntry
	    [CONSOLEFRAMEBUFFER_NPALETTEENTRIES];
} CONSOLEFRAMEBUFFER_PALETTE;

/** Describes the shape of the graphics cursor (aka mouse pointer) on the
 *  screen.
 *  @ingroup coretypes
 */
typedef struct _CONSOLEFRAMEBUFFER_GRAPHICSCURSOR  {
	u32 cursorDataBytes; /*< \#valid bytes in cursorData below */
	u8 graphicsCursorInUse;
			      /*< TRUE iff cursorData (below) is valid */
	u8 graphicsCursorVisible;
			      /*< TRUE iff cursorData (below) indicates
			       *   that the cursor is visible */
	u8 reserved[2];

    /** Bits describing the cursor shape.
     *  This is a data blob that needs to be passed to the remote client.
     */
	union CursorShapeData  {
		u8 cursorData[CONSOLEFRAMEBUFFER_MAXCURSORSHAPEBYTES];
	} u;
} CONSOLEFRAMEBUFFER_GRAPHICSCURSOR;

/* Describes the shape and location of the text cursor when in text mode.
 *  @ingroup coretypes
 */
typedef struct CONSOLEFRAMEBUFFER_TextCursor  {
	u8 textCursorInUse; /*< TRUE iff text cursor is visible */
	u8 characterCellHeight;
			     /*< height of each text char in (pixels) */
	u8 reserved[6];
	u32 row;	     /*< row number on the screen (0..49) */
	u32 column;	     /*< column number on the screen (0..131) */
	u32 startScanLine;   /*< start scan line defining text cursor shape
			      *   (pixel relative from char cell top)
			      *   (0..31)
			      */
	u32 endScanLine;    /*< end scan line defining text cursor shape
			      *   (pixel relative from char cell top)
			      *   (0..31)
			      */
	u8 reserved2[104];
} CONSOLEFRAMEBUFFER_TEXTCURSOR;

/* Specifies features that are supported by the firmware.
 *  The host driver may want to interrogate these.
 *  @ingroup coretypes
 */
typedef struct CONSOLEFRAMEBUFFER_FirmwareFeatures  {
	u32 res1024x768:1;	/* supports 1024x768 resolutions */
	u32 feature02:1;	/* available */
	u32 feature03:1;	/* available */
	u32 feature04:1;	/* available */
	u32 feature05:1;	/* available */
	u32 feature06:1;	/* available */
	u32 feature07:1;	/* available */
	u32 feature08:1;	/* available */
	u32 feature09:1;	/* available */
	u32 feature10:1;	/* available */
	u32 feature11:1;	/* available */
	u32 feature12:1;	/* available */
	u32 feature13:1;	/* available */
	u32 feature14:1;	/* available */
	u32 feature15:1;	/* available */
	u32 feature16:1;	/* available */
	u32 feature17:1;	/* available */
	u32 feature18:1;	/* available */
	u32 feature19:1;	/* available */
	u32 feature20:1;	/* available */
	u32 feature21:1;	/* available */
	u32 feature22:1;	/* available */
	u32 feature23:1;	/* available */
	u32 feature24:1;	/* available */
	u32 feature25:1;	/* available */
	u32 feature26:1;	/* available */
	u32 feature27:1;	/* available */
	u32 feature28:1;	/* available */
	u32 feature29:1;	/* available */
	u32 feature30:1;	/* available */
	u32 feature31:1;	/* available */
	u32 feature32:1;	/* available */
} CONSOLEFRAMEBUFFER_FIRMWAREFEATURES;

/* Specifies features that are supported by the host video driver.
 *  The firmware may want to interrogate these.
 *  @ingroup coretypes
 */
typedef struct CONSOLEFRAMEBUFFER_HostDriverFeatures  {
	u32 feature01:1;	/* available */
	u32 simpleRects:1;	 /*< host driver will indicate video
				   *   changes by inserting rectangles into
				   *   the RectsQ queue within the channel
				   *   (actual data is in Rects)
				   */
	u32 imageBatches:1;	 /*< host driver will indicate video
				   *   changes by inserting image batches into
				   *   the ImageBatchQ queue within the channel
				   *   (actual data is in ImageBatch)
				   */
	u32 crashDumpReqest:1;	 /*< hostRequestCode_crashDump supported */
	u32 rebootRequest:1;	 /*< hostRequestCode_reboot supported */
	u32 timeOfDay:1;	 /*< CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA
				   *   #timeOfDay is filled in */
	u32 refreshRequest:1;	 /*< hostRequestCode_refreshRegistry
				   *   supported */
	u32 res1024x768:1;	/* supports 1024x768 resolutions */
	u32 feature09:1;	/* available */
	u32 feature10:1;	/* available */
	u32 feature11:1;	/* available */
	u32 feature12:1;	/* available */
	u32 feature13:1;	/* available */
	u32 feature14:1;	/* available */
	u32 feature15:1;	/* available */
	u32 feature16:1;	/* available */
	u32 feature17:1;	/* available */
	u32 feature18:1;	/* available */
	u32 feature19:1;	/* available */
	u32 feature20:1;	/* available */
	u32 feature21:1;	/* available */
	u32 feature22:1;	/* available */
	u32 feature23:1;	/* available */
	u32 feature24:1;	/* available */
	u32 feature25:1;	/* available */
	u32 feature26:1;	/* available */
	u32 feature27:1;	/* available */
	u32 feature28:1;	/* available */
	u32 feature29:1;	/* available */
	u32 feature30:1;	/* available */
	u32 feature31:1;	/* available */
	u32 feature32:1;	/* available */
} CONSOLEFRAMEBUFFER_HOSTDRIVERFEATURES;

/* Identifies the format for each pixel in the video frame buffer.
 *  @ingroup coretypes
 */
typedef enum  { pixelFmt_NONE = 0, /*< text mode */
	    pixelFmt_PALETTEIZED = 1, /*< 8 bits/pixel  */
	    pixelFmt_GREYSCALE = 2, /*< 8 bits/pixel  */
	    pixelFmt_RGB565 = 3, /*< 16 bits/pixel */
	    pixelFmt_RGB555 = 4, /*< 16 bits/pixel */
	    pixelFmt_VGAMANGLE = 5, /*<  4 bits/pixel */
	    pixelFmt_RGB888 = 6, /*< 24 bits/pixel */
	    pixelFmt_ARGB8888 = 7, /*< 32 bits/pixel: alpha, red, green, blue */
	    pixelFmt_VGAUNMANGLE = 8 /*<  4 bits/pixel */
} CONSOLEFRAMEBUFFER_PIXELFMT;

/* Identifies primitive commands that the firmware can ask the host to
 *  perform.
 *  @ingroup coretypes
 */
typedef enum  { hostRequestCode_NONE = 0, /*< no command */
	    hostRequestCode_crashDump = 1, /*< crash dump system */
	    hostRequestCode_reboot = 2, /*< reboot system */
	    hostRequestCode_refreshRegistry =
	    3 /*< Refresh registry values */
} CONSOLEFRAMEBUFFER_HOSTREQUESTCODE;

/*---------------------------------------------------*
 *---  HOST DRIVER SHARED DATA (in frame buffer)  ---*
 *---------------------------------------------------*/

#define CONSOLEFRAMEBUFFER_HOSTDRIVERSIG1   0xFEED8086
#define CONSOLEFRAMEBUFFER_HOSTDRIVERSIG2   0x8086FEED

#define CONSOLEFRAMEBUFFER_MAXOUTSTANDINGIMAGECHANGEBATCHES    5
#define CONSOLEFRAMEBUFFER_MAXRECTS                         1024

/* Structure to describe a single batch of video changes that has happened
 *  in the host environment.
 *  @ingroup hostinterface
 */
typedef struct CONSOLEFRAMEBUFFER_ImageBatch  {

    /* A unique number identifying this particular batch.
     *  This number is incremented by the host driver after every batch.
     *  Note to host driver: to avoid possible synchronization problems,
     *  the host driver should set this value to 0 before
     *  changing it.  If the card then looks at this value while it is
     *  in the midst of getting written, the card may see a value
     *  that is too small, but never a value that is too large.
     */
	u64 seqNoBatch;
	u32 nRects;	/*< \# valid entries in the rect array below */
	u32 reserved1;	/*< filler to be 64-bit friendly */
	CONSOLEFRAMEBUFFER_CHANGERECTANGLE
	    rect[CONSOLEFRAMEBUFFER_MAXIMAGECHANGESPERBATCH];
} CONSOLEFRAMEBUFFER_IMAGEBATCH;

typedef struct CONSOLEFRAMEBUFFER_HostDriverVideoData_Header  {
	u32 sig1;	/*< #CONSOLEFRAMEBUFFER_HOSTDRIVERSIG1 */
	u16 compatibilityVersion;
	u8 version[18];/*< release info meaningful to user */

    /* features supported by host driver */
	CONSOLEFRAMEBUFFER_HOSTDRIVERFEATURES hostDriverFeatures;
	u8 reserved1[36];
			/*< leave more room for global data items,
			 *   and be 64-bit friendly
			 */
} CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA_HEADER;

typedef struct CONSOLEFRAMEBUFFER_HostDriverVideoData_VideoMode  {
	u16 textModeLines;
			/*<  0  = graphics mode,
			 *   25  = 25 lines,
			 *   50  = 50 lines
			 */
	u16 bytesPerScanLine;
			 /*< \# bytes in each scan line that need to
			 *   be looked at to display the screen image
			 */
	u16 bytesPerScanLineWithPad;
				/*< \# bytes occupied by each scan line
			 *   (this may be larger than
			 *   CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA
			 *   #bytesPerScanLine in modes that have extra
			 *   pad bytes at the end of each scan line)
			 */
	u16 nScanLines;/*< \# scan lines going down the screen */
	u16 pixelsAcross;
			/*< \# pixels on each scan line, or 0 */
	u8 bpp;	/*< 4, 8, 16, 24, 32 */
	u8 pixelFmt;	/*< PIXELFMT_xxx... */
	u8 reserved2[52];
			/*< initialized to 0 */
} CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA_VIDEOMODE;

typedef struct CONSOLEFRAMEBUFFER_HostDriverVideoData_SeqNo {
	u64 seqNoVideoMode;/*< incremented whenever video mode changes */
	u64 seqNoPalette;  /*< incremented whenever <palette> changes */
	u64 seqNoAbortBatch;
			    /*< incremented whenever the card should
			     *   just discard all remaining
			     *   batches and just refresh the image
			     */
	u64 seqNoReadyBatch;
			    /*  incremented whenever next batch ready
			     *   at CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA
			     *   #hostDriverImageChangesAddress
			     */
	u64 seqNoGraphicsCursor;
			    /*< incremented whenever graphicsCursor changes */
	u64 seqNoTextCursor;
			    /*< incremented whenever textCursor changes */
	u8 reserved5[80];  /*< leave room for more counters,
			     *   and be 64-bit friendly */
} CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA_SEQNO;

/* Structure describing the current host driver state.
 *  This structure lives in FPGAMEM
 *  @ #FPGAMEMBASEADDRESS+#FPGAMEMOFFSET_HOSTDRIVERVIDEODATA,
 *  and is always exactly 64 Kbytes in size.
 *  Note this structure ALWAYS lives in the card
 *  FPGAMEM, even if the video frame buffer is relocated
 *  into host memory.
 *  Changes to this structure are notified via the
 *  host driver by incrementing the appropriate <i>seqNoxxx</i>
 *  counter (depending upon what changed), then interrupting
 *  the card (if supported).
 *  The card can tell what was changed by comparing
 *  it's local values for the <i>seqNoxxx</i> counters to
 *  the one's contained within this structure.
 *  @ingroup hostinterface
 */
typedef struct CONSOLEFRAMEBUFFER_HostDriverVideoData  {
	CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA_HEADER header;
	CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA_VIDEOMODE videoMode;
	CONSOLEFRAMEBUFFER_PALETTE palette;
	CONSOLEFRAMEBUFFER_TEXTCURSOR textCursor;
	CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA_SEQNO seqNo;
	CONSOLEFRAMEBUFFER_GRAPHICSCURSOR graphicsCursor;

    /* host driver informs firmware about current time of day */
	CONSOLEFRAMEBUFFER_TIMEOFDAY timeOfDay;
	u8 reserved6[31324];
			  /*< force the structure to be exactly 64k bytes */
	u32 pendingBatch;
	u32 surfCount;
	u32 sig2;	    /*< CONSOLEFRAMEBUFFER_HOSTDRIVERSIG2 */
} CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA;

/*-----------------------------------------------*
 *---  FIRMWARE SHARED DATA (in FPGA memory)  ---*
 *-----------------------------------------------*/

#define CONSOLEFRAMEBUFFER_FIRMWARESIG1     0xFEED0520
#define CONSOLEFRAMEBUFFER_FIRMWARESIG2     0x0520FEED
typedef struct CONSOLEFRAMEBUFFER_FirmwareVideoData_Header  {
	u32 firmwareSig1;   /*< CONSOLEFRAMEBUFFER_FIRMWARESIG1 */
	u16 firmwareCompatibilityVersion;
	u8 firmwareVersion[18];
			     /*< release info meaningful to user */
	CONSOLEFRAMEBUFFER_FIRMWAREFEATURES firmwareFeatures;
	u8 reserved1[36];
} CONSOLEFRAMEBUFFER_FIRMWAREVIDEODATA_HEADER;

/* Structure describing the current firmware state.
 *  This structure lives in FPGAMEM
 *  @ #FPGAMEMBASEADDRESS+#FPGAMEMOFFSET_FIRMWAREVIDEODATA,
 *  and is always exactly 65536 bytes in size.
 *  @ingroup hostinterface
 */
typedef struct CONSOLEFRAMEBUFFER_FirmwareVideoData  {
	CONSOLEFRAMEBUFFER_FIRMWAREVIDEODATA_HEADER header;

	u32 hostRequestCode; /*< if non-0, command for host to perform
			      *   (hostRequestCode_xxx defined by
			      *   #CONSOLEFRAMEBUFFER_HOSTREQUESTCODE)
			      */
	u32 frameBufferBytes; /*< number of bytes of allocated video
			       *  framebuffer memory */
	s64 offsetToFrameBuffer; /* < offset from the beginning of the
				  *   consoleframebuffer channel to the
				  *   video framebuffer memory; note this
				  *   is a physical memory offset, not
				  *   virtual
				  */
	s64 offsetToLegacyVideo; /* < offset from the beginning of the
			      *   consoleframebuffer channel to the
			      *   BOCHS_VIDEO_STATE structure (if you
			      *   are a host driver running in
			      *   non-root mode, don't even THINK
			      *   about trying to access this memory);
			      *   note this is a physical memory
			      *   offset, not virtual
			      */
	u8 pad[65444];	     /* < force the structure to be exactly 64k */
	u32 firmwareSig2;   /*  #CONSOLEFRAMEBUFFER_FIRMWARESIG2 */
} CONSOLEFRAMEBUFFER_FIRMWAREVIDEODATA;
typedef struct _ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL {
	ULTRA_CHANNEL_PROTOCOL Header;	/* 128 bytes */
	CONSOLEFRAMEBUFFER_FIRMWAREVIDEODATA FirmwareVideoData;	/* 64k bytes */
	CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA HostDriverVideoData; /* 64k */
	SIGNAL_QUEUE_HEADER ImageBatchQ;	/* Signal Data in ImageBatch */
	SIGNAL_QUEUE_HEADER RectsQ;	/* Signal Data in Rects */
	 CONSOLEFRAMEBUFFER_IMAGEBATCH ImageBatch[CONSOLEFRAMEBUFFER_MAXOUTSTANDINGIMAGECHANGEBATCHES];
	CONSOLEFRAMEBUFFER_CHANGERECTANGLE Rects[CONSOLEFRAMEBUFFER_MAXRECTS];

	    /*  This field is in case we ever really want to add stuff
	     *  to this channel, but we do not want anyone else to
	     *  notice a change in channel size.
	     */
	 u8 reserved[65536];
} ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL;

#define CONSOLEFRAMEBUFFER_CH_SIZE COVER(sizeof(ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL), 65536)

#pragma pack(pop)

static inline void
ULTRA_CONSOLEFRAMEBUFFER_init_channel(ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL *x)
{
	int ofs1 = offsetof(ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL, ImageBatchQ);
	int ofs2 = offsetof(ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL, RectsQ);
	int firstq;

	/* Based on the field ordering, compute the appropriate
	 * value to assign to oChannelSpace.
	 */
	if (ofs1 > ofs2)
		firstq = ofs2;

	else
		firstq = ofs1;
	memset(x, 0, sizeof(x->Header));	/* YES, JUST the header! */
	x->Header.VersionId =
	    ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_VERSIONID;
	x->Header.Signature =
	    ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_SIGNATURE;
	x->Header.SrvState = CHANNELSRV_UNINITIALIZED;
	x->Header.HeaderSize = sizeof(x->Header);
	x->Header.Size = CONSOLEFRAMEBUFFER_CH_SIZE;
	x->Header.Type = UltraConsoleFramebufferChannelProtocolGuid;
	x->Header.ZoneGuid = NULL_UUID_LE;

	SignalInit(x, ImageBatchQ, ImageBatch,
		   CONSOLEFRAMEBUFFER_IMAGEBATCH, 0, 0);
	SignalInit(x, RectsQ, Rects, CONSOLEFRAMEBUFFER_CHANGERECTANGLE, 0, 0);
	x->Header.oChannelSpace = firstq;
}

static inline void
ULTRA_CONSOLEFRAMEBUFFER_set_graphics_mode_ex(ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL *x,
					      int pixelWidth,
					      int pixelHeight, int bpp)
{
	int bytesPerPixel;
	switch (bpp) {
	case 16:
		bytesPerPixel = 2;
		x->HostDriverVideoData.videoMode.pixelFmt = pixelFmt_RGB565;
		break;
	case 32:
		bytesPerPixel = 4;
		x->HostDriverVideoData.videoMode.pixelFmt = pixelFmt_ARGB8888;
		break;
	default:
		return;
		}
	x->HostDriverVideoData.videoMode.textModeLines = 0;
	x->HostDriverVideoData.videoMode.pixelsAcross = (u16) pixelWidth;
	x->HostDriverVideoData.videoMode.bytesPerScanLine =
	    pixelWidth * bytesPerPixel;

#if CONSOLEFRAMEBUFFER_FORCEFIXEDSIZESCANLINES
	    x->HostDriverVideoData.videoMode.bytesPerScanLineWithPad = 4096;

#else	/*  */
	    x->HostDriverVideoData.videoMode.bytesPerScanLineWithPad =
	    x->HostDriverVideoData.videoMode.bytesPerScanLine;

#endif	/*  */
	    x->HostDriverVideoData.videoMode.nScanLines = (u16) pixelHeight;
	x->HostDriverVideoData.videoMode.bpp = (u8) bpp;
	x->HostDriverVideoData.header.sig1 = CONSOLEFRAMEBUFFER_HOSTDRIVERSIG1;
	x->HostDriverVideoData.sig2 = CONSOLEFRAMEBUFFER_HOSTDRIVERSIG2;
}

static inline void
ULTRA_CONSOLEFRAMEBUFFER_set_graphics_mode(ULTRA_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL *x,
					   int pixelWidth,
					   int pixelHeight)
{
	ULTRA_CONSOLEFRAMEBUFFER_set_graphics_mode_ex(x, pixelWidth,
						       pixelHeight, 32);
}
#endif
