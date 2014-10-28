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

#include <linux/types.h>
#include <linux/uuid.h>

#include "channel.h"

/* {230A065A-39D8-4917-9F05-3ECC5CBF4A4F} */
#define SPAR_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_UUID     \
	UUID_LE(0x230a065a, 0x39d8, 0x4917, \
		0x9f, 0x5, 0x3e, 0xcc, 0x5c, 0xbf, 0x4a, 0x4f)
static const uuid_le ultra_console_framebuffer_channel_protocol_uuid =
	SPAR_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_UUID;

#define SPAR_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_SIGNATURE \
	ULTRA_CHANNEL_PROTOCOL_SIGNATURE
/* Must increment this whenever you insert or delete fields within
* this channel struct.  Also increment whenever you change the meaning
* of fields within this channel struct so as to break pre-existing
* software.  Note that you can usually add fields to the END of the
* channel struct withOUT needing to increment this. */
#define SPAR_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_VERSIONID 1

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
struct consoleframebuffer_timeofday  {
	u16 month;
	u16 day;
	u16 year;
	u16 hour;
	u16 minute;
	u16 second;
	u16 reserved1;		/* make it 64-bit friendly */
	u16 reserved2;
};

/** Describes to the remote partition desktop client how the video
 *  data is compressed.
 *  @ingroup coretypes
 */
enum consoleframebuffer_compressionmode {
	compress_NONE = 0, /*< not compressed */
	compress_RLE1 = 1, /*< compressed w/ sw run-length (1-byte units) */
	compress_RLE2 = 2, /*< compressed w/ sw run-length (2-byte units) */
	compress_RLE3 = 3, /*< compressed w/ sw run-length (3-byte units) */
	compress_RLE4 = 4, /*< compressed w/ sw run-length (4-byte units) */
	compress_LZW = 5 /*< compressed w/ hw LZW compression */
};

/** Describes a single color in the palette by specifying the alpha,
 *  red, green, and blue (ARGB) color components.
 *  Important note:  the order of the fields within this structure is
 *  dependent upon the order of fields within the Windows
 *  VIDEO_CLUTDATA palette structure.
 *  @ingroup coretypes
 */
struct consoleframebuffer_paletteentry  {
	u8 red;
	u8 green;
	u8 blue;
	u8 alpha;
};

/** Identifies a rectangular region within the video frame buffer of video
 *  data that has changed.
 *  @ingroup coretypes
 */
struct consoleframebuffer_changerect  {
	u16 x;	   /**< x-coordinate of pixel at upper-left corner of rect */
	u16 y;	   /**< y-coordinate of pixel at upper-left corner of rect */
	u16 width;/**< width of rectangle  (in pixels) */
	u16 height;
		   /**< height of rectangle (in scanlines) */
};

/** Specifies the colors in ARGB format for every color in the
 *  256-color palette; used when #CONSOLEFRAMEBUFFER_PIXELFMT is
 *  #pixelFmt_PALETTEIZED.
 *  @ingroup coretypes
 */
struct consoleframebuffer_palette  {
	struct consoleframebuffer_paletteentry palette_entry
	    [CONSOLEFRAMEBUFFER_NPALETTEENTRIES];
};

/** Describes the shape of the graphics cursor (aka mouse pointer) on the
 *  screen.
 *  @ingroup coretypes
 */
struct consoleframebuffer_graphicscursor  {
	u32 cursor_data_bytes; /*< \#valid bytes in cursorData below */
	u8 in_use;
			      /*< TRUE iff cursorData (below) is valid */
	u8 visible;
			      /*< TRUE iff cursorData (below) indicates
			       *   that the cursor is visible */
	u8 reserved[2];

    /** Bits describing the cursor shape.
     *  This is a data blob that needs to be passed to the remote client.
     */
	union cursor_shape_data  {
		u8 cursor_data[CONSOLEFRAMEBUFFER_MAXCURSORSHAPEBYTES];
	} u;
};

/* Describes the shape and location of the text cursor when in text mode.
 *  @ingroup coretypes
 */
struct consoleframebuffer_textcursor  {
	u8 in_use; /*< TRUE iff text cursor is visible */
	u8 cell_height;
			     /*< height of each text char in (pixels) */
	u8 reserved[6];
	u32 row;	     /*< row number on the screen (0..49) */
	u32 column;	     /*< column number on the screen (0..131) */
	u32 start_scanline;   /*< start scan line defining text cursor shape
			      *   (pixel relative from char cell top)
			      *   (0..31)
			      */
	u32 end_scanline;    /*< end scan line defining text cursor shape
			      *   (pixel relative from char cell top)
			      *   (0..31)
			      */
	u8 reserved2[104];
};

/* Specifies features that are supported by the firmware.
 *  The host driver may want to interrogate these.
 *  @ingroup coretypes
 */
struct consoleframebuffer_firmwarefeatures  {
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
};

/* Specifies features that are supported by the host video driver.
 *  The firmware may want to interrogate these.
 *  @ingroup coretypes
 */
struct consoleframebuffer_hostdriverfeatures {
	u32 feature01:1;	/* available */
	u32 simple_rects:1;	 /*< host driver will indicate video
				   *   changes by inserting rectangles into
				   *   the RectsQ queue within the channel
				   *   (actual data is in Rects)
				   */
	u32 image_batches:1;	 /*< host driver will indicate video
				   *   changes by inserting image batches into
				   *   the ImageBatchQ queue within the channel
				   *   (actual data is in ImageBatch)
				   */
	u32 crash_dump_request:1; /*< hostRequestCode_crashDump supported */
	u32 reboot_request:1;	 /*< hostRequestCode_reboot supported */
	u32 time_of_day:1;	 /*< CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA
				   *   #timeOfDay is filled in */
	u32 refresh_request:1;	 /*< hostRequestCode_refreshRegistry
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
};

/* Identifies the format for each pixel in the video frame buffer.
 *  @ingroup coretypes
 */
enum consoleframebuffer_pixelfmt {
	PIXELFMT_NONE = 0, /*< text mode */
	PIXELFMT_PALETTEIZED = 1, /*< 8 bits/pixel  */
	PIXELFMT_GREYSCALE = 2, /*< 8 bits/pixel  */
	PIXELFMT_RGB565 = 3, /*< 16 bits/pixel */
	PIXELFMT_RGB555 = 4, /*< 16 bits/pixel */
	PIXELFMT_VGAMANGLE = 5, /*<  4 bits/pixel */
	PIXELFMT_RGB888 = 6, /*< 24 bits/pixel */
	PIXELFMT_ARGB8888 = 7, /*< 32 bits/pixel: alpha, red, green, blue */
	PIXELFMT_VGAUNMANGLE = 8 /*<  4 bits/pixel */
};

/* Identifies primitive commands that the firmware can ask the host to
 *  perform.
 *  @ingroup coretypes
 */
enum consoleframebuffer_hostrrequestcode {
	HOSTREQUESTCODE_NONE = 0, /*< no command */
	HOSTREQUESTCODE_CRASHDUMP = 1, /*< crash dump system */
	HOSTREQUESTCODE_REBOOT = 2, /*< reboot system */
	HOSTREQUESTCODE_REFRESHREGISTRY = 3 /*< Refresh registry values */
};

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
struct consoleframebuffer_imagebatch {
    /* A unique number identifying this particular batch.
     *  This number is incremented by the host driver after every batch.
     *  Note to host driver: to avoid possible synchronization problems,
     *  the host driver should set this value to 0 before
     *  changing it.  If the card then looks at this value while it is
     *  in the midst of getting written, the card may see a value
     *  that is too small, but never a value that is too large.
     */
	u64 seq_no_batch;
	u32 n_rects;	/*< \# valid entries in the rect array below */
	u32 reserved1;	/*< filler to be 64-bit friendly */
	struct consoleframebuffer_changerect
	    rect[CONSOLEFRAMEBUFFER_MAXIMAGECHANGESPERBATCH];
};

struct consoleframebuffer_hostdrivervideodata_header  {
	u32 sig1;	/*< #CONSOLEFRAMEBUFFER_HOSTDRIVERSIG1 */
	u16 compat_version;
	u8 version[18];/*< release info meaningful to user */

    /* features supported by host driver */
	struct consoleframebuffer_hostdriverfeatures host_driver_features;
	u8 reserved1[36];
			/*< leave more room for global data items,
			 *   and be 64-bit friendly
			 */
};

struct consoleframebuffer_hostdrivervideodata_videomode  {
	u16 textmodelines;
			/*<  0  = graphics mode,
			 *   25  = 25 lines,
			 *   50  = 50 lines
			 */
	u16 bytesperscanline;
			 /*< \# bytes in each scan line that need to
			 *   be looked at to display the screen image
			 */
	u16 bytesperscanline_withpad;
				/*< \# bytes occupied by each scan line
			 *   (this may be larger than
			 *   CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA
			 *   #bytesPerScanLine in modes that have extra
			 *   pad bytes at the end of each scan line)
			 */
	u16 n_scanlines;/*< \# scan lines going down the screen */
	u16 pixels_across;
			/*< \# pixels on each scan line, or 0 */
	u8 bpp;	/*< 4, 8, 16, 24, 32 */
	u8 pixel_fmt;	/*< PIXELFMT_xxx... */
	u8 reserved2[52];
			/*< initialized to 0 */
} CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA_VIDEOMODE;

struct consoleframebuffer_hostdrivervideodata_seqno {
	u64 videomode;/*< incremented whenever video mode changes */
	u64 palette;  /*< incremented whenever <palette> changes */
	u64 abort_batch;
			    /*< incremented whenever the card should
			     *   just discard all remaining
			     *   batches and just refresh the image
			     */
	u64 ready_batch;
			    /*  incremented whenever next batch ready
			     *   at CONSOLEFRAMEBUFFER_HOSTDRIVERVIDEODATA
			     *   #hostDriverImageChangesAddress
			     */
	u64 graphics_cursor;
			    /*< incremented whenever graphicsCursor changes */
	u64 text_cursor;
			    /*< incremented whenever textCursor changes */
	u8 reserved5[80];  /*< leave room for more counters,
			     *   and be 64-bit friendly */
};

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
struct consoleframebuffer_hostdrivervideodata {
	struct consoleframebuffer_hostdrivervideodata_header header;
	struct consoleframebuffer_hostdrivervideodata_videomode video_mode;
	struct consoleframebuffer_palette palette;
	struct consoleframebuffer_textcursor text_cursor;
	struct consoleframebuffer_hostdrivervideodata_seqno seq_no;
	struct consoleframebuffer_graphicscursor graphics_cursor;

    /* host driver informs firmware about current time of day */
	struct consoleframebuffer_timeofday time_of_day;
	u8 reserved6[31324];
			  /*< force the structure to be exactly 64k bytes */
	u32 pending_batch;
	u32 surf_count;
	u32 sig2;	    /*< CONSOLEFRAMEBUFFER_HOSTDRIVERSIG2 */
};

/*-----------------------------------------------*
 *---  FIRMWARE SHARED DATA (in FPGA memory)  ---*
 *-----------------------------------------------*/

#define CONSOLEFRAMEBUFFER_FIRMWARESIG1     0xFEED0520
#define CONSOLEFRAMEBUFFER_FIRMWARESIG2     0x0520FEED
struct consoleframebuffer_firmwarevideodata_header  {
	u32 sig1;   /*< CONSOLEFRAMEBUFFER_FIRMWARESIG1 */
	u16 compatibility_version;
	u8 version[18];
			     /*< release info meaningful to user */
	struct consoleframebuffer_firmwarefeatures features;
	u8 reserved1[36];
};

/* Structure describing the current firmware state.
 *  This structure lives in FPGAMEM
 *  @ #FPGAMEMBASEADDRESS+#FPGAMEMOFFSET_FIRMWAREVIDEODATA,
 *  and is always exactly 65536 bytes in size.
 *  @ingroup hostinterface
 */
struct consoleframebuffer_firmwarevideodata  {
	struct consoleframebuffer_firmwarevideodata_header header;

	u32 host_request_code; /*< if non-0, command for host to perform
			      *   (hostRequestCode_xxx defined by
			      *   #CONSOLEFRAMEBUFFER_HOSTREQUESTCODE)
			      */
	u32 framebuffer_bytes; /*< number of bytes of allocated video
			       *  framebuffer memory */
	s64 framebuffer_offset; /* < offset from the beginning of the
				  *   consoleframebuffer channel to the
				  *   video framebuffer memory; note this
				  *   is a physical memory offset, not
				  *   virtual
				  */
	s64 legacy_video_offset; /* < offset from the beginning of the
			      *   consoleframebuffer channel to the
			      *   BOCHS_VIDEO_STATE structure (if you
			      *   are a host driver running in
			      *   non-root mode, don't even THINK
			      *   about trying to access this memory);
			      *   note this is a physical memory
			      *   offset, not virtual
			      */
	u8 pad[65444];	     /* < force the structure to be exactly 64k */
	u32 sig2;   /*  #CONSOLEFRAMEBUFFER_FIRMWARESIG2 */
};

struct spar_consoleframebuffer_channel_protocol {
	struct channel_header header;
	struct consoleframebuffer_firmwarevideodata firmwarevideodata;
	struct consoleframebuffer_hostdrivervideodata hostdrivervideodata;
	struct signal_queue_header image_batch_q;/* Signal Data in ImageBatch */
	struct signal_queue_header rects_q;	/* Signal Data in Rects */
	struct consoleframebuffer_imagebatch
	       image_batch[CONSOLEFRAMEBUFFER_MAXOUTSTANDINGIMAGECHANGEBATCHES];
	struct consoleframebuffer_changerect rects[CONSOLEFRAMEBUFFER_MAXRECTS];

	    /*  This field is in case we ever really want to add stuff
	     *  to this channel, but we do not want anyone else to
	     *  notice a change in channel size.
	     */
	 u8 reserved[65536];
};

#define CONSOLEFRAMEBUFFER_CH_SIZE \
	COVER(sizeof(struct spar_consoleframebuffer_channel_protocol), 65536)

#pragma pack(pop)

static inline void ULTRA_CONSOLEFRAMEBUFFER_init_channel(
		struct spar_consoleframebuffer_channel_protocol *x)
{
	int ofs1 = offsetof(struct spar_consoleframebuffer_channel_protocol,
			    image_batch_q);
	int ofs2 = offsetof(struct spar_consoleframebuffer_channel_protocol,
			    rects_q);
	int firstq;

	/* Based on the field ordering, compute the appropriate
	 * value to assign to oChannelSpace.
	 */
	if (ofs1 > ofs2)
		firstq = ofs2;

	else
		firstq = ofs1;
	memset(x, 0, sizeof(x->header));	/* YES, JUST the header! */
	x->header.version_id =
	    SPAR_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_VERSIONID;
	x->header.signature =
	    SPAR_CONSOLEFRAMEBUFFER_CHANNEL_PROTOCOL_SIGNATURE;
	x->header.srv_state = CHANNELSRV_UNINITIALIZED;
	x->header.header_size = sizeof(x->header);
	x->header.size = CONSOLEFRAMEBUFFER_CH_SIZE;
	x->header.chtype = ultra_console_framebuffer_channel_protocol_uuid;
	x->header.zone_uuid = NULL_UUID_LE;

	spar_signal_init(x, image_batch_q, image_batch,
			 struct consoleframebuffer_imagebatch, 0, 0);
	spar_signal_init(x, rects_q, rects,
			 struct consoleframebuffer_changerect, 0, 0);
	x->header.ch_space_offset = firstq;
}

static inline void ULTRA_CONSOLEFRAMEBUFFER_set_graphics_mode_ex(
		struct spar_consoleframebuffer_channel_protocol *x,
		int width, int height, int bpp)
{
	int bytesperpix;

	switch (bpp) {
	case 16:
		bytesperpix = 2;
		x->hostdrivervideodata.video_mode.pixel_fmt = PIXELFMT_RGB565;
		break;
	case 32:
		bytesperpix = 4;
		x->hostdrivervideodata.video_mode.pixel_fmt = PIXELFMT_ARGB8888;
		break;
	default:
		return;
		}
	x->hostdrivervideodata.video_mode.textmodelines = 0;
	x->hostdrivervideodata.video_mode.pixels_across = (u16)width;
	x->hostdrivervideodata.video_mode.bytesperscanline =
	    width * bytesperpix;

#if CONSOLEFRAMEBUFFER_FORCEFIXEDSIZESCANLINES
	    x->hostdrivervideodata.video_mode.bytesperscanline_withpad = 4096;

#else	/*  */
	    x->hostdrivervideodata.video_mode.bytesperscanline_withpad =
	    x->hostdrivervideodata.video_mode.bytesperscanline;

#endif	/*  */
	    x->hostdrivervideodata.video_mode.n_scanlines = (u16)height;
	x->hostdrivervideodata.video_mode.bpp = (u8)bpp;
	x->hostdrivervideodata.header.sig1 = CONSOLEFRAMEBUFFER_HOSTDRIVERSIG1;
	x->hostdrivervideodata.sig2 = CONSOLEFRAMEBUFFER_HOSTDRIVERSIG2;
}

static inline void ULTRA_CONSOLEFRAMEBUFFER_set_graphics_mode(
			struct spar_consoleframebuffer_channel_protocol *x,
			int width, int height)
{
	ULTRA_CONSOLEFRAMEBUFFER_set_graphics_mode_ex(x, width, height, 32);
}
#endif
