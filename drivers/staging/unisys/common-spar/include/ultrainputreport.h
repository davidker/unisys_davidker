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

#ifndef __ULTRAINPUTREPORT_H__
#define __ULTRAINPUTREPORT_H__

#include "commontypes.h"

#pragma pack(push, 1)		/* both GCC and VC now allow this pragma */
/* Identifies mouse and keyboard activity which is specified by the firmware to
 *  the host using the cmsimpleinput protocol.  @ingroup coretypes
 */
typedef enum {
	inputAction_none = 0,
	inputAction_xyMotion = 1,	/*< only motion; arg1=x, arg2=y */
	inputAction_mouseButtonDown = 2, /*< arg1: 1=left, 2=center, 3=right, 4,
					 *  5 */
	inputAction_mouseButtonUp = 3,	/*< arg1: 1=left, 2=center, 3=right, 4,
					 *  5 */
	inputAction_mouseButtonClick = 4, /*< arg1: 1=left, 2=center, 3=right,
					 *  4, 5 */
	inputAction_mouseButtonDclick = 5, /*< arg1: 1=left, 2=center, 3=right,
					    *  4, 5 */
	inputAction_wheelRotateAway = 6, /*< arg1: wheel rotation away from
					  *  user */
	inputAction_wheelRotateToward = 7, /*< arg1: wheel rotation toward
					    *  user */
	inputAction_setMaxXY = 8,	/*< set screen maxXY; arg1=x, arg2=y */
	inputAction_keyDown = 64,	/*< arg1: scancode, as follows:
					 * If arg1 <= 0xff, it's a 1-byte
					 * scancode and arg1 is that scancode.
					 * If arg1 > 0xff, it's a 2-byte
					 * scanecode, with the 1st byte in the
					 * low 8 bits, and the 2nd byte in the
					 * high 8 bits.  E.g., the right ALT key
					 * would appear as x'38e0'. */
	inputAction_keyUp = 65,		/*< arg1: scancode (in same format as
					 * inputAction_keyDown)
					 */
	inputAction_setLockingKeyState = 66,
					/*< arg1: scancode (in same format
					 *         as inputAction_keyDown);
					 *         MUST refer to one of the
					 *         locking keys, like capslock,
					 *         numlock, or scrolllock
					 *   arg2: 1 iff locking key should be
					 *         in the LOCKED position
					 *         (e.g., light is ON)
					 */
	inputAction_keyDownUp = 67,	/*< arg1: scancode (in same format
					 *         as inputAction_keyDown)
					 */
	inputAction_LAST
} ULTRA_INPUTACTION;

typedef struct {
	U16 action;
		 /*< see ULTRA_INPUTACTION */
	U16 arg1;
	U16 arg2;
	U16 arg3;
} ULTRA_INPUTACTIVITY;

typedef struct {
	U64 seqNo;
	ULTRA_INPUTACTIVITY activity;
} ULTRA_INPUTREPORT;

#pragma pack(pop)

#endif
