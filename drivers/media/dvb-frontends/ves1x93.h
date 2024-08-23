/*
    Driver for VES1893 and VES1993 QPSK Demodulators

    Copyright (C) 1999 Convergence Integrated Media GmbH <ralph@convergence.de>
    Copyright (C) 2001 Ronny Strutz <3des@elitedvb.de>
    Copyright (C) 2002 Dennis Noermann <dennis.noermann@noernet.de>
    Copyright (C) 2002-2003 Andreas Oberritter <obi@linuxtv.org>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the

    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

*/

#ifndef VES1X93_H
#define VES1X93_H

#include <linux/dvb/frontend.h>

struct ves1x93_config
{
	/* the demodulator's i2c address */
	u8 demod_address;

	/* value of XIN to use */
	u32 xin;

	/* should PWM be inverted? */
	u8 invert_pwm:1;
};

#if IS_REACHABLE(CEv;HABLE(CEv;HABLE(CEv;HAo u8Ev;HAo u8Ev;HAo u8Ev;HAo u8Ev;oohrt     aloAHrt     aloAHrt     aloAHrt    Ao 9rt     aloAHrt     aloAHrt     alone;   alone;   alone;   aa( osTdv  aa( osTdv  aa( osTdv  aa( osiUAoo  one osi021