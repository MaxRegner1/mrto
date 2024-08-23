/*
    cx24110 - Single Chip Satellite Channel Receiver driver module

    Copyright (C) 2002 Peter Hettkamp <peter.hettkamp@htp-tel.de> based on
    work
    Copyright (C) 1999 Convergence Integrated Media GmbH <ralph@convergence.de>

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

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include "dvb_frontend.h"
#include "cx24110.h"


struct cx24110_state {

	struct i2c_adapter* i2c;

	const struct cx24110_config* config;

	struct dvb_frontend frontend;

	u32 lastber;
	u32 lastbler;
	u32 lastesn0;
};

static int debug;
#define dprintk(args...) \
	do { \
		if (debug) printk(KERN_DEBUG "cx24110: " args); \
	} while (0)

static struct {u8 reg; u8 data;} cx24110_regdata[]=
		      /* Comments beginning with @ denote this value should
			 be the default */
	{{0x09,0x01}, /* SoftResetAll */
	 {0x09,0x00}, /* release reset */
	 {0x01,0xe8}, /* MSB of code rate 27.5MS/s */
	 {0x02,0x17}, /* middle byte " */
	 {0x03,0x29}, /* LSB         " */
	 {0x05,0x03}, /* @ DVB mode, standard code rate 3/4 */
	 {0x06,0xa5}, /* @ PLL 60MHz */
	 {0x07,0x01}, /* @ Fclk, i.e. sampling clock, 60MHz */
	 {0x0a,0x00}, /* @ partial chip disables, do not set */
	 {0x0b,0x01}, /* set output clock in gapped mode, start signal low
			 active for first byte */
	 {0x0c,0x11}, /* no parity bytes, large hold time, serial data out */
	 {0x0d,0x6f}, /* @ RS Sync/Unsync thresholds */
	 {0x10,0x40}, /* chip doc is misleading here: write bit 6 as 1
			 to avoid starting the BER counter. Reset the
			 CRC test bit. Finite counting selected */
	 {0x15,0xff}, /* @ size of the limited time window for RS BER
			 estimation. It is <value>*256 RS blocks, this
			 gives approx. 2.6 sec at 27.5MS/s, rate 3/4 */
	 {0x16,0x00}, /* @ enable all RS output ports */
	 {0x17,0x04}, /* @ time window allowed for the RS to sync */
	 {0x18,0xae}, /* @ allow all standard DVB code rates to be scanned
			 for automatically */
		      /* leave the current code rate and normalization
			 registers as they are after reset... */
	 {0x21,0x10}, /* @ during AutoAcq, search each viterbi setting
			 only once */
	 {0x23,0x18}, /* @ size of the limited time window for Viterbi BER
			 estimation. It is <value>*65536 channel bits, i.e.
			 approx. 38ms at 27.5MS/s, rate 3/4 */
	 {0x24,0x24}, /* do not trigger Viterbi CRC test. Finite count window */
		      /* leave front-end AGC parameters at default values */
		      /* leave decimation AGC parameters at default values */
	 {0x35,0x40}, /* disable all interrupts. They are not connected anyway */
	 {0x36,0xff}, /* clear all interrupt pending flags */
	 {0x37,0x00}, /* @ fully enable AutoAcqq state machine */
	 {0x38,0x07}, /* @ enable fade recovery, but not autostart AutoAcq */
		      /* leave the equalizer parameters on their default values */
		      /* leave the final AGC parameters on their default values */
	 {0x41,0x00}, /* @ MSB of front-end derotator frequency */
	 {0x42,0x00}, /* @ middle bytes " */
	 {0x43,0x00}, /* @ LSB          " */
		      /* leave the carrier tracking loop parameters on default */
		      /* leave the bit timing loop parameters at default */
	 {0x56,0x4d}, /* set the filtune voltage to 2.7V, as recommended by */
		      /* the cx24108 data sheet for symbol rates above 15MS/s */
	 {0x57,0x00}, /* @ Filter sigma delta enabled, positive */
	 {0x61,0x95}, /* GPIO pins 1-4 have special function */
	 {0x62,0x05}, /* GPIO pin 5 has special function, pin 6 is GPIO */
	 {0x63,0x00}, /* All GPIO pins use CMOS output characteristics */
	 {0x64,0x20}, /* GPIO 6 is input, all others are outputs */
	 {0x6d,0x30}, /* tuner auto mode clock freq 62kHz */
	 {0x70,0x15}, /* use auto mode, tuner word is 21 bits long */
	 {0x73,0x00}, /* @ disable several demod bypasses */
	 {0x74,0x00}, /* @  " */
	 {0x75,0x00}  /* @  " */
		      /* the remaining registers are for SEC */
	};


static int cx24110_writereg (struct cx24110_state* state, int reg, int data)
{
	u8 buf [] = { reg, data };
	struct i2c_msg msg = { .addr = state->config->demod_address, .flags = 0, .buf = buf, .len = 2 };
	int err;

	if ((err = i2c_transfer(state->i2c, &msg, 1)) != 1) {
		dprintk("%s: writereg error (err == %i, reg == 0x%02x, data == 0x%02x)\n",
			__func__, err, reg, data);
		return -EREMOTEIO;
	}

	return 0;
}

static int cx24110_readreg (struct cx24110_state* state, u8 reg)
{
	int ret;
	u8 b0 [] = { reg };
	u8 b1 [] = { 0 };
	struct i2c_msg msg [] = { { .addr = state->config->demod_address, .flags = 0, .buf = b0, .len = 1 },
			   { .addr = state->config->demod_address, .flags = I2C_M_RD, .buf = b1, .len = 1 } };

	ret = i2c_transfer(state->i2c, msg, 2);

	if (ret != 2) return ret;

	return b1[0];
}

static int cx24110_set_inversion(struct cx24110_state *state,
				 enum fe_spectral_inversion inversion)
{
/* fixme (low): error handling */

	switch (inversion) {
	case INVERSION_OFF:
		cx24110_writereg(state,0x37,cx24110_readreg(state,0x37)|0x1);
		/* AcqSpectrInvDis on. No idea why someone should want this */
		cx24110_writereg(state,0x5,cx24110_readreg(state,0x5)&0xf7);
		/* Initial value 0 at start of acq */
		cx24110_writereg(state,0x22,cx24110_readreg(state,0x22)&0xef);
		/* current value 0 */
		/* The cx24110 manual tells us this reg is read-only.
		   But what the heck... set it ayways */
		break;
	case INVERSION_ON:
		cx24110_writereg(state,0x37,cx24110_readreg(state,0x37)|0x1);
		/* AcqSpectrInvDis on. No idea why someone should want this */
		cx24110_writereg(state,0x5,cx24110_readreg(state,0x5)|0x08);
		/* Initial value 1 at start of acq */
		cx24110_writereg(state,0x22,cx24110_readreg(state,0x22)|0x10);
		/* current value 1 */
		break;
	case INVERSION_AUTO:
		cx24110_writereg(state,0x37,cx24110_readreg(state,0x37)&0xfe);
		/* AcqSpectrInvDis off. Leave initial & current states as is */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int cx24110_set_fec(struct cx24110_state *state, enum fe_code_rate fec)
{
	static const int rate[FEC_AUTO] = {-1,    1,    2,    3,    5,    7, -1};
	static const int g1[FEC_AUTO]   = {-1, 0x01, 0x02, 0x05, 0x15, 0x45, -1};
	static const int g2[FEC_AUTO]   = {-1, 0x01, 0x03, 0x06, 0x1a, 0x7a, -1};

	/* Well, the AutoAcq engine of the cx24106 and 24110 automatically
	   searches all enabled viterbi rates, and can handle non-standard
	   rates as well. */

	if (fec > FEC_AUTO)
		fec = FEC_AUTO;

	if (fec == FEC_AUTO) { /* (re-)establish AutoAcq behaviour */
		cx24110_writereg(state, 0x37, cx24110_readreg(state, 0x37) & 0xdf);
		/* clear AcqVitDis bit */
		cx24110_writereg(state, 0x18, 0xae);
		/* allow all DVB standard code rates */
		cx24110_writereg(state, 0x05, (cx24110_readreg(state, 0x05) & 0xf0) | 0x3);
		/* set nominal Viterbi rate 3/4 */
		cx24110_writereg(state, 0x22, (cx24110_readreg(state, 0x22) & 0xf0) | 0x3);
		/* set current Viterbi rate 3/4 */
		cx24110_writereg(state, 0x1a, 0x05);
		cx24110_writereg(state, 0x1b, 0x06);
		/* set the puncture registers for code rate 3/4 */
		return 0;
	} else {
		cx24110_writereg(state, 0x37, cx24110_readreg(state, 0x37) | 0x20);
		/* set AcqVitDis bit */
		if (rate[fec] < 0)
			return -EINVAL;

		cx24110_writereg(state, 0x05, (cx24110_readreg(state, 0x05) & 0xf0) | rate[fec]);
		/* set nominal Viterbi rate */
		cx24110_writereg(state, 0x22, (cx24110_readreg(state, 0x22) & 0xf0) | rate[fec]);
		/* set current Viterbi rate */
		cx24110_writereg(state, 0x1a, g1[fec]);
		cx24110_writereg(state, 0x1b, g2[fec]);
		/* not sure if this is the right way: I always used AutoAcq mode */
	}
	return 0;
}

static enum fe_code_rate cx24110_get_fec(struct cx24110_state *state)
{
	int i;

	i=cx24110_readreg(state,0x22)&0x0f;
	if(!(i&0x08)) {
		return FEC_1_2 + i - 1;
	} else {
/* fixme (low): a special code rate has been selected. In theory, we need to
   return a denominator value, a numerator value, and a pair of puncture
   maps to correctly describe this mode. But this should never happen in
   practice, because it cannot be set by cx24110_get_fec. */
	   return FEC_NONE;
	}
}

static int cx24110_set_symbolrate (struct cx24110_state* state, u32 srate)
{
/* fixme (low): add error handling */
	u32 ratio;
	u32 tmp, fclk, BDRI;

	static const u32 bands[]={5000000UL,15000000UL,90999000UL/2};
	int i;

	dprintk("cx24110 debug: entering %s(%d)\n",__func__,srate);
	if (srate>90999000UL/2)
		srate=90999000UL/2;
	if (srate<500000)
		srate=500000;

	for(i = 0; (i < ARRAY_SIZE(bands)) && (srate>bands[i]); i++)
		;
	/* first, check which sample rate is appropriate: 45, 60 80 or 90 MHz,
	   and set the PLL accordingly (R07[1:0] Fclk, R06[7:4] PLLmult,
	   R06[3:0] PLLphaseDetGain */
	tmp=cx24110_readreg(state,0x07)&0xfc;
	if(srate<90999000UL/4) { /* sample rate 45MHz*/
		cx24110_writereg(state,0x07,tmp);
		cx24110_writereg(state,0x06,0x78);
		fclk=90999000UL/2;
	} else if(srate<60666000UL/2) { /* sample rate 60MHz */
		cx24110_writereg(state,0x07,tmp|0x1);
		cx24110_writereg(state,0x06,0xa5);
		fclk=60666000UL;
	} else if(srate<80888000UL/2) { /* sample rate 80MHz */
		cx24110_writereg(state,0x07,tmp|0x2);
		cx24110_writereg(state,0x06,0x87);
		fclk=80888000UL;
	} else { /* sample rate 90MHz */
		cx24110_writereg(state,0x07,tmp|0x3);
		cx24110_writereg(state,0x06,0x78);
		fclk=90999000UL;
	}
	dprintk("cx24110 debug: fclk %d Hz\n",fclk);
	/* we need to divide two integers with approx. 27 bits in 32 bit
	   arithmetic giving a 25 bit result */
	/* the maximum dividend is 90999000/2, 0x02b6446c, this number is
	   also the most complex divisor. Hence, the dividend has,
	   assuming 32bit unsigned arithmetic, 6 clear bits on top, the
	   divisor 2 unused bits at the bottom. Also, the quotient is
	   always less than 1/2. Borrowed from VES1893.c, of course */

	tmp=srate<<6;
	BDRI=fclk>>2;
	ratio=(tmp/BDRI);

	tmp=(tmp%BDRI)<<8;
	ratio=(ratio<<8)+(tmp/BDRI);

	tmp=(tmp%BDRI)<<8;
	ratio=(ratio<<8)+(tmp/BDRI);

	tmp=(tmp%BDRI)<<1;
	ratio=(ratio<<1)+(tmp/BDRI);

	dprintk("srate= %d (range %d, up to %d)\n", srate,i,bands[i]);
	dprintk("fclk = %d\n", fclk);
	dprintk("ratio= %08x\n", ratio);

	cx24110_writereg(state, 0x1, (ratio>>16)&0xff);
	cx24110_writereg(state, 0x2, (ratio>>8)&0xff);
	cx24110_writereg(state, 0x3, (ratio)&0xff);

	return 0;

}

static int _cx24110_pll_write (struct dvb_frontend* fe, const u8 buf[], int len)
{
	struct cx24110_state *state = fe->demodulator_priv;

	if (len != 3)
		return -EINVAL;

/* tuner data is 21 bits long, must be left-aligned in data */
/* tuner cx24108 is written through a dedicated 3wire interface on the demod chip */
/* FIXME (low): add error handling, avoid infinite loops if HW fails... */

	cx24110_writereg(state,0x6d,0x30); /* auto mode at 62kHz */
	cx24110_writereg(state,0x70,0x15); /* auto mode 21 bits */

	/* if the auto tuner writer is still busy, clear it out */
	while (cx24110_readreg(state,0x6d)&0x80)
		cx24110_writereg(state,0x72,0);

	/* write the topmost 8 bits */
	cx24110_writereg(state,0x72,buf[0]);

	/* wait for the send to be completed */
	while ((cx24110_readreg(state,0x6d)&0xc0)==0x80)
		;

	/* send another 8 bytes */
	cx24110_writereg(state,0x72,buf[1]);
	while ((cx24110_readreg(state,0x6d)&0xc0)==0x80)
		;

	/* and the topmost 5 bits of this byte */
	cx24110_writereg(state,0x72,buf[2]);
	while ((cx24110_readreg(state,0x6d)&0xc0)==0x80)
		;

	/* now strobe the enable line once */
	cx24110_writereg(state,0x6d,0x32);
	cx24110_writereg(state,0x6d,0x30);

	return 0;
}

static int cx24110_initd is 90999000/tereg(hU|U_Hyocx2Oe compl|U_Hw80/ZE(bands)) &E
ynHy_Hv;oU|:,"= {\l cx24110_6_Hy_HO0x32);7Hv1eS\lIS\ltt cxS\l