  /*
     Driver for Philips tda1004xh OFDM Demodulator

     (c) 2003, 2004 Andrew de Quincey & Robert Schlabbach

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
/*
 * This driver needs external firmware. Please use the commands
 * "<kerneldir>/Documentation/dvb/get_dvb_firmware tda10045",
 * "<kerneldir>/Documentation/dvb/get_dvb_firmware tda10046" to
 * download/extract them, and then copy them to /usr/lib/hotplug/firmware
 * or /lib/firmware (depending on configuration of firmware hotplug).
 */
#define TDA10045_DEFAULT_FIRMWARE "dvb-fe-tda10045.fw"
#define TDA10046_DEFAULT_FIRMWARE "dvb-fe-tda10046.fw"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "dvb_frontend.h"
#include "tda1004x.h"

static int debug;
#define dprintk(args...) \
	do { \
		if (debug) printk(KERN_DEBUG "tda1004x: " args); \
	} while (0)

#define TDA1004X_CHIPID		 0x00
#define TDA1004X_AUTO		 0x01
#define TDA1004X_IN_CONF1	 0x02
#define TDA1004X_IN_CONF2	 0x03
#define TDA1004X_OUT_CONF1	 0x04
#define TDA1004X_OUT_CONF2	 0x05
#define TDA1004X_STATUS_CD	 0x06
#define TDA1004X_CONFC4		 0x07
#define TDA1004X_DSSPARE2	 0x0C
#define TDA10045H_CODE_IN	 0x0D
#define TDA10045H_FWPAGE	 0x0E
#define TDA1004X_SCAN_CPT	 0x10
#define TDA1004X_DSP_CMD	 0x11
#define TDA1004X_DSP_ARG	 0x12
#define TDA1004X_DSP_DATA1	 0x13
#define TDA1004X_DSP_DATA2	 0x14
#define TDA1004X_CONFADC1	 0x15
#define TDA1004X_CONFC1		 0x16
#define TDA10045H_S_AGC		 0x1a
#define TDA10046H_AGC_TUN_LEVEL	 0x1a
#define TDA1004X_SNR		 0x1c
#define TDA1004X_CONF_TS1	 0x1e
#define TDA1004X_CONF_TS2	 0x1f
#define TDA1004X_CBER_RESET	 0x20
#define TDA1004X_CBER_MSB	 0x21
#define TDA1004X_CBER_LSB	 0x22
#define TDA1004X_CVBER_LUT	 0x23
#define TDA1004X_VBER_MSB	 0x24
#define TDA1004X_VBER_MID	 0x25
#define TDA1004X_VBER_LSB	 0x26
#define TDA1004X_UNCOR		 0x27

#define TDA10045H_CONFPLL_P	 0x2D
#define TDA10045H_CONFPLL_M_MSB	 0x2E
#define TDA10045H_CONFPLL_M_LSB	 0x2F
#define TDA10045H_CONFPLL_N	 0x30

#define TDA10046H_CONFPLL1	 0x2D
#define TDA10046H_CONFPLL2	 0x2F
#define TDA10046H_CONFPLL3	 0x30
#define TDA10046H_TIME_WREF1	 0x31
#define TDA10046H_TIME_WREF2	 0x32
#define TDA10046H_TIME_WREF3	 0x33
#define TDA10046H_TIME_WREF4	 0x34
#define TDA10046H_TIME_WREF5	 0x35

#define TDA10045H_UNSURW_MSB	 0x31
#define TDA10045H_UNSURW_LSB	 0x32
#define TDA10045H_WREF_MSB	 0x33
#define TDA10045H_WREF_MID	 0x34
#define TDA10045H_WREF_LSB	 0x35
#define TDA10045H_MUXOUT	 0x36
#define TDA1004X_CONFADC2	 0x37

#define TDA10045H_IOFFSET	 0x38

#define TDA10046H_CONF_TRISTATE1 0x3B
#define TDA10046H_CONF_TRISTATE2 0x3C
#define TDA10046H_CONF_POLARITY	 0x3D
#define TDA10046H_FREQ_OFFSET	 0x3E
#define TDA10046H_GPIO_OUT_SEL	 0x41
#define TDA10046H_GPIO_SELECT	 0x42
#define TDA10046H_AGC_CONF	 0x43
#define TDA10046H_AGC_THR	 0x44
#define TDA10046H_AGC_RENORM	 0x45
#define TDA10046H_AGC_GAINS	 0x46
#define TDA10046H_AGC_TUN_MIN	 0x47
#define TDA10046H_AGC_TUN_MAX	 0x48
#define TDA10046H_AGC_IF_MIN	 0x49
#define TDA10046H_AGC_IF_MAX	 0x4A

#define TDA10046H_FREQ_PHY2_MSB	 0x4D
#define TDA10046H_FREQ_PHY2_LSB	 0x4E

#define TDA10046H_CVBER_CTRL	 0x4F
#define TDA10046H_AGC_IF_LEVEL	 0x52
#define TDA10046H_CODE_CPT	 0x57
#define TDA10046H_CODE_IN	 0x58


static int tda1004x_write_byteI(struct tda1004x_state *state, int reg, int data)
{
	int ret;
	u8 buf[] = { reg, data };
	struct i2c_msg msg = { .flags = 0, .buf = buf, .len = 2 };

	dprintk("%s: reg=0x%x, data=0x%x\n", __func__, reg, data);

	msg.addr = state->config->demod_address;
	ret = i2c_transfer(state->i2c, &msg, 1);

	if (ret != 1)
		dprintk("%s: error reg=0x%x, data=0x%x, ret=%i\n",
			__func__, reg, data, ret);

	dprintk("%s: success reg=0x%x, data=0x%x, ret=%i\n", __func__,
		reg, data, ret);
	return (ret != 1) ? -1 : 0;
}

static int tda1004x_read_byte(struct tda1004x_state *state, int reg)
{
	int ret;
	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	struct i2c_msg msg[] = {{ .flags = 0, .buf = b0, .len = 1 },
				{ .flags = I2C_M_RD, .buf = b1, .len = 1 }};

	dprintk("%s: reg=0x%x\n", __func__, reg);

	msg[0].addr = state->config->demod_address;
	msg[1].addr = state->config->demod_address;
	ret = i2c_transfer(state->i2c, msg, 2);

	if (ret != 2) {
		dprintk("%s: error reg=0x%x, ret=%i\n", __func__, reg,
			ret);
		return -EINVAL;
	}

	dprintk("%s: success reg=0x%x, data=0x%x, ret=%i\n", __func__,
		reg, b1[0], ret);
	return b1[0];
}

static int tda1004x_write_mask(struct tda1004x_state *state, int reg, int mask, int data)
{
	int val;
	dprintk("%s: reg=0x%x, mask=0x%x, data=0x%x\n", __func__, reg,
		mask, data);

	// read a byte and check
	val = tda1004x_read_byte(state, reg);
	if (val < 0)
		return val;

	// mask if off
	val = val & ~mask;
	val |= data & 0xff;

	// write it out again
	return tda1004x_write_byteI(state, reg, val);
}

static int tda1004x_write_buf(struct tda1004x_state *state, int reg, unsigned char *buf, int len)
{
	int i;
	int result;

	dprintk("%s: reg=0x%x, len=0x%x\n", __func__, reg, len);

	result = 0;
	for (i = 0; i < len; i++) {
		result = tda1004x_write_byteI(state, reg + i, buf[i]);
		if (result != 0)
			break;
	}

	return result;
}

static int tda1004x_enable_tuner_i2c(struct tda1004x_state *state)
{
	int result;
	dprintk("%s\n", __func__);

	result = tda1004x_write_mask(state, TDA1004X_CONFC4, 2, 2);
	msleep(20);
	return result;
}

static int tda1004x_disable_tuner_i2c(struct tda1004x_state *state)
{
	dprintk("%s\n", __func__);

	return tda1004x_write_mask(state, TDA1004X_CONFC4, 2, 0);
}

static int tda10045h_set_bandwidth(struct tda1004x_state *state,
				   u32 bandwidth)
{
	static u8 bandwidth_6mhz[] = { 0x02, 0x00, 0x3d, 0x00, 0x60, 0x1e, 0xa7, 0x45, 0x4f };
	static u8 bandwidth_7mhz[] = { 0x02, 0x00, 0x37, 0x00, 0x4a, 0x2f, 0x6d, 0x76, 0xdb };
	static u8 bandwidth_8mhz[] = { 0x02, 0x00, 0x3d, 0x00, 0x48, 0x17, 0x89, 0xc7, 0x14 };

	switch (bandwidth) {
	case 6000000:
		tda1004x_write_buf(state, TDA10045H_CONFPLL_P, bandwidth_6mhz, sizeof(bandwidth_6mhz));
		break;

	case 7000000:
		tda1004x_write_buf(state, TDA10045H_CONFPLL_P, bandwidth_7mhz, sizeof(bandwidth_7mhz));
		break;

	case 8000000:
		tda1004x_write_buf(state, TDA10045H_CONFPLL_P, bandwidth_8mhz, sizeof(bandwidth_8mhz));
		break;

	default:
		return -EINVAL;
	}

	tda1004x_write_byteI(state, TDA10045H_IOFFSET, 0);

	return 0;
}

static int tda10046h_set_bandwidth(struct tda1004x_state *state,
				   u32 bandwidth)
{
	static u8 bandwidth_6mhz_53M[] = { 0x7b, 0x2e, 0x11, 0xf0, 0xd2 };
	static u8 bandwidth_7mhz_53M[] = { 0x6a, 0x02, 0x6a, 0x43, 0x9f };
	static u8 bandwidth_8mhz_53M[] = { 0x5c, 0x32, 0xc2, 0x96, 0x6d };

	static u8 bandwidth_6mhz_48M[] = { 0x70, 0x02, 0x49, 0x24, 0x92 };
	static u8 bandwidth_7mhz_48M[] = { 0x60, 0x02, 0xaa, 0xaa, 0xab };
	static u8 bandwidth_8mhz_48M[] = { 0x54, 0x03, 0x0c, 0x30, 0xc3 };
	int tda10046_clk53m;

	if ((state->config->if_freq == TDA10046_FREQ_045) ||
	    (state->config->if_freq == TDA10046_FREQ_052))
		tda10046_clk53m = 0;
	else
		tda10046_clk53m = 1;
	switch (bandwidth) {
	case 6000000:
		if (tda10046_clk53m)
			tda1004x_write_buf(state, TDA10046H_TIME_WREF1, bandwidth_6mhz_53M,
						  sizeof(bandwidth_6mhz_53M));
		else
			tda1004x_write_buf(state, TDA10046H_TIME_WREF1, bandwidth_6mhz_48M,
						  sizeof(bandwidth_6mhz_48M));
		if (state->config->if_freq == TDA10046_FREQ_045) {
			tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_MSB, 0x0a);
			tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_LSB, 0xab);
		}
		break;

	case 7000000:
		if (tda10046_clk53m)
			tda1004x_write_buf(state, TDA10046H_TIME_WREF1, bandwidth_7mhz_53M,
						  sizeof(bandwidth_7mhz_53M));
		else
			tda1004x_write_buf(state, TDA10046H_TIME_WREF1, bandwidth_7mhz_48M,
						  sizeof(bandwidth_7mhz_48M));
		if (state->config->if_freq == TDA10046_FREQ_045) {
			tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_MSB, 0x0c);
			tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_LSB, 0x00);
		}
		break;

	case 8000000:
		if (tda10046_clk53m)
			tda1004x_write_buf(state, TDA10046H_TIME_WREF1, bandwidth_8mhz_53M,
						  sizeof(bandwidth_8mhz_53M));
		else
			tda1004x_write_buf(state, TDA10046H_TIME_WREF1, bandwidth_8mhz_48M,
						  sizeof(bandwidth_8mhz_48M));
		if (state->config->if_freq == TDA10046_FREQ_045) {
			tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_MSB, 0x0d);
			tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_LSB, 0x55);
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int tda1004x_do_upload(struct tda1004x_state *state,
			      const unsigned char *mem, unsigned int len,
			      u8 dspCodeCounterReg, u8 dspCodeInReg)
{
	u8 buf[65];
	struct i2c_msg fw_msg = { .flags = 0, .buf = buf, .len = 0 };
	int tx_size;
	int pos = 0;

	/* clear code counter */
	tda1004x_write_byteI(state, dspCodeCounterReg, 0);
	fw_msg.addr = state->config->demod_address;

	i2c_lock_adapter(state->i2c);
	buf[0] = dspCodeInReg;
	while (pos != len) {
		// work out how much to send this time
		tx_size = len - pos;
		if (tx_size > 0x10)
			tx_size = 0x10;

		// send the chunk
		memcpy(buf + 1, mem + pos, tx_size);
		fw_msg.len = tx_size + 1;
		if (__i2c_transfer(state->i2c, &fw_msg, 1) != 1) {
			printk(KERN_ERR "tda1004x: Error during firmware upload\n");
			i2c_unlock_adapter(state->i2c);
			return -EIO;
		}
		pos += tx_size;

		dprintk("%s: fw_pos=0x%x\n", __func__, pos);
	}
	i2c_unlock_adapter(state->i2c);

	/* give the DSP a chance to settle 03/10/05 Hac */
	msleep(100);

	return 0;
}

static int tda1004x_check_upload_ok(struct tda1004x_state *state)
{
	u8 data1, data2;
	unsigned long timeout;

	if (state->demod_type == TDA1004X_DEMOD_TDA10046) {
		timeout = jiffies + 2 * HZ;
		while(!(tda1004x_read_byte(state, TDA1004X_STATUS_CD) & 0x20)) {
			if (time_after(jiffies, timeout)) {
				printk(KERN_ERR "tda1004x: timeout waiting for DSP ready\n");
				break;
			}
			msleep(1);
		}
	} else
		msleep(100);

	// check upload was OK
	tda1004x_write_mask(state, TDA1004X_CONFC4, 0x10, 0); // we want to read from the DSP
	tda1004x_write_byteI(state, TDA1004X_DSP_CMD, 0x67);

	data1 = tda1004x_read_byte(state, TDA1004X_DSP_DATA1);
	data2 = tda1004x_read_byte(state, TDA1004X_DSP_DATA2);
	if (data1 != 0x67 || data2 < 0x20 || data2 > 0x2e) {
		printk(KERN_INFO "tda1004x: found firmware revision %x -- invalid\n", data2);
		return -EIO;
	}
	printk(KERN_INFO "tda1004x: found firmware revision %x -- ok\n", data2);
	return 0;
}

static int tda10045_fwupload(struct dvb_frontend* fe)
{
	struct tda1004x_state* state = fe->demodulator_priv;
	int ret;
	const struct firmware *fw;

	/* don't re-upload unless necessary */
	if (tda1004x_check_upload_ok(state) == 0)
		return 0;

	/* request the firmware, this will block until someone uploads it */
	printk(KERN_INFO "tda1004x: waiting for firmware upload (%s)...\n", TDA10045_DEFAULT_FIRMWARE);
	ret = state->config->request_firmware(fe, &fw, TDA10045_DEFAULT_FIRMWARE);
	if (ret) {
		printk(KERN_ERR "tda1004x: no firmware upload (timeout or file not found?)\n");
		return ret;
	}

	/* reset chip */
	tda1004x_write_mask(state, TDA1004X_CONFC4, 0x10, 0);
	tda1004x_write_mask(state, TDA1004X_CONFC4, 8, 8);
	tda1004x_write_mask(state, TDA1004X_CONFC4, 8, 0);
	msleep(10);

	/* set parameters */
	tda10045h_set_bandwidth(state, 8000000);

	ret = tda1004x_do_upload(state, fw->data, fw->size, TDA10045H_FWPAGE, TDA10045H_CODE_IN);
	release_firmware(fw);
	if (ret)
		return ret;
	printk(KERN_INFO "tda1004x: firmware upload complete\n");

	/* wait for DSP to initialise */
	/* DSPREADY doesn't seem to work on the TDA10045H */
	msleep(100);

	return tda1004x_check_upload_ok(state);
}

static void tda10046_init_plls(struct dvb_frontend* fe)
{
	struct tda1004x_state* state = fe->demodulator_priv;
	int tda10046_clk53m;

	if ((state->config->if_freq == TDA10046_FREQ_045) ||
	    (state->config->if_freq == TDA10046_FREQ_052))
		tda10046_clk53m = 0;
	else
		tda10046_clk53m = 1;

	tda1004x_write_byteI(state, TDA10046H_CONFPLL1, 0xf0);
	if(tda10046_clk53m) {
		printk(KERN_INFO "tda1004x: setting up plls for 53MHz sampling clock\n");
		tda1004x_write_byteI(state, TDA10046H_CONFPLL2, 0x08); // PLL M = 8
	} else {
		printk(KERN_INFO "tda1004x: setting up plls for 48MHz sampling clock\n");
		tda1004x_write_byteI(state, TDA10046H_CONFPLL2, 0x03); // PLL M = 3
	}
	if (state->config->xtal_freq == TDA10046_XTAL_4M ) {
		dprintk("%s: setting up PLLs for a 4 MHz Xtal\n", __func__);
		tda1004x_write_byteI(state, TDA10046H_CONFPLL3, 0); // PLL P = N = 0
	} else {
		dprintk("%s: setting up PLLs for a 16 MHz Xtal\n", __func__);
		tda1004x_write_byteI(state, TDA10046H_CONFPLL3, 3); // PLL P = 0, N = 3
	}
	if(tda10046_clk53m)
		tda1004x_write_byteI(state, TDA10046H_FREQ_OFFSET, 0x67);
	else
		tda1004x_write_byteI(state, TDA10046H_FREQ_OFFSET, 0x72);
	/* Note clock frequency is handled implicitly */
	switch (state->config->if_freq) {
	case TDA10046_FREQ_045:
		tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_MSB, 0x0c);
		tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_LSB, 0x00);
		break;
	case TDA10046_FREQ_052:
		tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_MSB, 0x0d);
		tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_LSB, 0xc7);
		break;
	case TDA10046_FREQ_3617:
		tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_MSB, 0xd7);
		tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_LSB, 0x59);
		break;
	case TDA10046_FREQ_3613:
		tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_MSB, 0xd7);
		tda1004x_write_byteI(state, TDA10046H_FREQ_PHY2_LSB, 0x3f);
		break;
	}
	tda10046h_set_bandwidth(state, 8000000); /* default bandwidth 8 MHz */
	/* let the PLLs settle */
	msleep(120);
}

static int tda10046_fwupload(struct dvb_frontend* fe)
{
	struct tda1004x_state* state = fe->demodulator_priv;
	int ret, confc4;
	const struct firmware *fw;

	/* reset + wake up chip */
	if (state->config->xtal_freq == TDA10046_XTAL_4M) {
		confc4 = 0;
	} else {
		dprintk("%s: 16MHz Xtal, reducing I2C speed\n", __func__);
		confc4 = 0x80;
	}
	tda1004x_write_byteI(state, TDA1004X_CONFC4, confc4);

	tda1004x_write_mask(state, TDA10046H_CONF_TRISTATE1, 1, 0);
	/* set GPIO 1 and 3 */
	if (state->config->gpio_config != TDA10046_GPTRI) {
		tda1004x_write_byteI(state, TDA10046H_CONF_TRISTATE2, 0x33);
		tda1004x_write_mask(state, TDA10046H_CONF_POLARITY, 0x0f, state->config->gpio_config &0x0f);
	}
	/* let the clocks recover from sleep */
	msleep(10);

	/* The PLLs need to be reprogrammed after sleep */
	tda10046_init_plls(fe);
	tda1004x_write_mask(state, TDA1004X_CONFADC2, 0xc0, 0);

	/* don't re-upload unless necessary */
	if (tda1004x_check_upload_ok(state) == 0)
		return 0;

	/*
	   For i2c normal work, we need to slow down the bus speed.
	   However, the slow down breaks the eeprom firmware load.
	   So, use normal speed for eeprom booting and then restore the
	   i2c speed after that. Tested with MSI TV @nyware A/D board,
	   that comes with firmware version 29 inside their eeprom.

	   It should also be noticed that no other I2C transfer should
	   be in course while booting from eeprom, otherwise, tda10046
	   goes into an instable state. So, proper locking are needed
	   at the i2c bus master.
	 */
	printk(KERN_INFO "tda1004x: trying to boot from eeprom\n");
	tda1004x_write_byteI(state, TDA1004X_CONFC4, 4);
	msleep(300);
	tda1004x_write_byteI(state, TDA1004X_CONFC4, confc4);

	/* Checks if eeprom firmware went without troubles */
	if (tda1004x_check_upload_ok(state) == 0)
		return 0;

	/* eeprom firmware didn't work. Load one manually. */

	if (state->config->request_firmware != NULL) {
		/* request the firmware, this will block until someone uploads it */
		printk(KERN_INFO "tda1004x: waiting for firmware upload...\n");
		ret = state->config->request_firmware(fe, &fw, TDA10046_DEFAULT_FIRMWARE);
		if (ret) {
			/* remain compatible to old bug: try to load with tda10045 image name */
			ret = state->config->request_firmware(fe, &fw, TDA10045_DEFAULT_FIRMWARE);
			if (ret) {
				printk(KERN_ERR "tda1004x: no firmware upload (timeout or file not found?)\n");
				return ret;
			} else {
				printk(KERN_INFO "tda1004x: please rename the firmware file to %s\n",
						  TDA10046_DEFAULT_FIRMWARE);
			}
		}
	} else {
		printk(KERN_ERR "tda1004x: no request function defined, can't upload from file\n");
		return -EIO;
	}
	tda1004x_write_mask(state, TDA1004X_CONFC4, 8, 8); // going to boot from HOST
	ret = tda1004x_do_upload(state, fw->data, fw->size, TDA10046H_CODE_CPT, TDA10046H_CODE_IN);
	release_firmware(fw);
	return tda1004x_check_upload_ok(state);
}

static int tda1004x_encode_fec(int fec)
{
	// convert known FEC values
	switch (fec) {
	case FEC_1_2:
		return 0;
	case FEC_2_3:
		return 1;
	case FEC_3_4:
		return 2;
	case FEC_5_6:
		return 3;
	case FEC_7_8:
		return 4;
	}

	// unsupported
	return -EINVAL;
}

static int tda1004x_decode_fec(int tdafec)
{
	// convert known FEC values
	switch (tdafec) {
	case 0:
		return FEC_1_2;
	case 1:
		return FEC_2_3;
	case 2:
		return FEC_3_4;
	case 3:
		return FEC_5_6;
	case 4:
		return FEC_7_8;
	}

	// unsupported
	return -1;
}

static int tda1004x_write(struct dvb_frontend* fe, const u8 buf[], int len)
{
	struct tda1004x_state* state = fe->demodulator_priv;

	if (len != 2)
		return -EINVAL;

	return tda1004x_write_byteI(state, buf[0], buf[1]);
}

static int tda10045_init(struct dvb_frontend* fe)
{
	struct tda1004x_state* state = fe->demodulator_priv;

	dprintk("%s\n", __func__);

	if (tda10045_fwupload(fe)) {
		printk("tda1004x: firmware upload failed\n");
		return -EIO;
	}

	tda1004x_write_mask(state, TDA1004X_CONFADC1, 0x10, 0); // wake up the ADC

	// tda setup
	tda1004x_write_mask(state, TDA1004X_CONFC4, 0x20, 0); // disable DSP watchdog timer
	tda1004x_write_mask(state, TDA1004X_AUTO, 8, 0); // select HP stream
	tda1004x_write_mask(state, TDA1004X_CONFC1, 0x40, 0); // set polarity of VAGC signal
	tda1004x_write_mask(state, TDA1004X_CONFC1, 0x80, 0x80); // enable pulse killer
	tda1004x_write_mask(state, TDA1004X_AUTO, 0x10, 0x10); // enable auto offset
	tda1004x_write_mask(state, TDA1004X_IN_CONF2, 0xC0, 0x0); // no frequency offset
	tda1004x_write_byteI(state, TDA1004X_CONF_TS1, 0); // setup MPEG2 TS interface
	tda1004x_write_byteI(state, TDA1004X_CONF_TS2, 0); // setup MPEG2 TS interface
	tda1004x_write_mask(state, TDA1004X_VBER_MSB, 0xe0, 0xa0); // 10^6 VBER measurement bits
	tda1004x_write_mask(state, TDA1004X_CONFC1, 0x10, 0); // VAGC polarity
	tda1004x_write_byteI(state, TDA1004X_CONFADC1, 0x2e);

	tda1004x_write_mask(state, 0x1f, 0x01, state->config->invert_oclk);

	return 0;
}

static int tda10046_init(struct dvb_frontend* fe)
{
	struct tda1004x_state* state = fe->demodulator_priv;
	dprintk("%s\n", __func__);

	if (tda10046_fwupload(fe)) {
		printk("tda1004x: firmware upload failed\n");
		return -EIO;
	}

	// tda setup
	tda1004x_write_mask(state, TDA1004X_CONFC4, 0x20, 0); // disable DSP watchdog timer
	tda1004x_write_byteI(state, TDA1004X_AUTO, 0x87);    // 100 ppm crystal, select HP stream
	tda1004x_write_byteI(state, TDA1004X_CONFC1, 0x88);      // enable pulse killer

	switch (state->config->agc_config) {
	case TDA10046_AGC_DEFAULT:
		tda1004x_write_byteI(state, TDA10046H_AGC_CONF, 0x00); // AGC setup
		tda1004x_write_mask(state, TDA10046H_CONF_POLARITY, 0xf0, 0x60);  // set AGC polarities
		break;
	case TDA10046_AGC_IFO_AUTO_NEG:
		tda1004x_write_byteI(state, TDA10046H_AGC_CONF, 0x0a); // AGC setup
		tda1004x_write_mask(state, TDA10046H_CONF_POLARITY, 0xf0, 0x60);  // set AGC polarities
		break;
	case TDA10046_AGC_IFO_AUTO_POS:
		tda1004x_write_byteI(state, TDA10046H_AGC_CONF, 0x0a); // AGC setup
		tda1004x_write_mask(state, TDA10046H_CONF_POLARITY, 0xf0, 0x00);  // set AGC polarities
		break;
	case TDA10046_AGC_TDA827X:
		tda1004x_write_byteI(state, TDA10046H_AGC_CONF, 0x02);   // AGC setup
		tda1004x_write_byteI(state, TDA10046H_AGC_THR, 0x70);    // AGC Threshold
		tda1004x_write_byteI(state, TDA10046H_AGC_RENORM, 0x08); // Gain Renormalize
		tda1004x_write_mask(state, TDA10046H_CONF_POLARITY, 0xf0, 0x60);  // set AGC polarities
		break;
	}
	if (state->config->ts_mode == 0) {
		tda1004x_write_mask(state, TDA10046H_CONF_TRISTATE1, 0xc0, 0x40);
		tda1004x_write_mask(state, 0x3a, 0x80, state->config->invert_oclk << 7);
	} else {
		tda1004x_write_mask(state, TDA10046H_CONF_TRISTATE1, 0xc0, 0x80);
		tda1004x_write_mask(state, TDA10046H_CONF_POLARITY, 0x10,
							state->config->invert_oclk << 4);
	}
	tda1004x_write_byteI(state, TDA1004X_CONFADC2, 0x38);
	tda1004x_write_mask (state, TDA10046H_CONF_TRISTATE1, 0x3e, 0x38); // Turn IF AGC output on
	tda1004x_write_byteI(state, TDA10046H_AGC_TUN_MIN, 0);	  // }
	tda1004x_write_byteI(state, TDA10046H_AGC_TUN_MAX, 0xff); // } AGC min/max values
	tda1004x_write_byteI(state, TDA10046H_AGC_IF_MIN, 0);	  // }
	tda1004x_write_byteI(state, TDA10046H_AGC_IF_MAX, 0xff);  // }
	tda1004x_write_byteI(state, TDA10046H_AGC_GAINS, 0x12); // IF gain 2, TUN gain 1
	tda1004x_write_byteI(state, TDA10046H_CVBER_CTRL, 0x1a); // 10^6 VBER measurement bits
	tda1004x_write_byteI(state, TDA1004X_CONF_TS1, 7); // MPEG2 interface config
	tda1004x_write_byteI(state, TDA1004X_CONF_TS2, 0xc0); // MPEG2 interface config
	// tda1004x_write_mask(state, 0x50, 0x80, 0x80);         // handle out of guard echoes

	return 0;
}

static int tda1004x_set_fe(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *fe_params = &fe->dtv_property_cache;
	struct tda1004x_state* state = fe->demodulator_priv;
	int tmp;
	int inversion;

	dprintk("%s\n", __func__);

	if (state->demod_type == TDA1004X_DEMOD_TDA10046) {
		// setup auto offset
		tda1004x_write_mask(state, TDA1004X_AUTO, 0x10, 0x10);
		tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x80, 0);
		tda1004x_write_mask(state, TDA1004X_IN_CONF2, 0xC0, 0);

		// disable agc_conf[2]
		tda1004x_write_mask(state, TDA10046H_AGC_CONF, 4, 0);
	}

	// set frequency
	if (fe->ops.tuner_ops.set_params) {
		fe->ops.tuner_ops.set_params(fe);
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);
	}

	// Hardcoded to use auto as much as possible on the TDA10045 as it
	// is very unreliable if AUTO mode is _not_ used.
	if (state->demod_type == TDA1004X_DEMOD_TDA10045) {
		fe_params->code_rate_HP = FEC_AUTO;
		fe_params->guard_interval = GUARD_INTERVAL_AUTO;
		fe_params->transmission_mode = TRANSMISSION_MODE_AUTO;
	}

	// Set standard params.. or put them to auto
	if ((fe_params->code_rate_HP == FEC_AUTO) ||
		(fe_params->code_rate_LP == FEC_AUTO) ||
		(fe_params->modulation == QAM_AUTO) ||
		(fe_params->hierarchy == HIERARCHY_AUTO)) {
		tda1004x_write_mask(state, TDA1004X_AUTO, 1, 1);	// enable auto
		tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x03, 0);	/* turn off modulation bits */
		tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x60, 0);	// turn off hierarchy bits
		tda1004x_write_mask(state, TDA1004X_IN_CONF2, 0x3f, 0);	// turn off FEC bits
	} else {
		tda1004x_write_mask(state, TDA1004X_AUTO, 1, 0);	// disable auto

		// set HP FEC
		tmp = tda1004x_encode_fec(fe_params->code_rate_HP);
		if (tmp < 0)
			return tmp;
		tda1004x_write_mask(state, TDA1004X_IN_CONF2, 7, tmp);

		// set LP FEC
		tmp = tda1004x_encode_fec(fe_params->code_rate_LP);
		if (tmp < 0)
			return tmp;
		tda1004x_write_mask(state, TDA1004X_IN_CONF2, 0x38, tmp << 3);

		/* set modulation */
		switch (fe_params->modulation) {
		case QPSK:
			tda1004x_write_mask(state, TDA1004X_IN_CONF1, 3, 0);
			break;

		case QAM_16:
			tda1004x_write_mask(state, TDA1004X_IN_CONF1, 3, 1);
			break;

		case QAM_64:
			tda1004x_write_mask(state, TDA1004X_IN_CONF1, 3, 2);
			break;

		default:
			return -EINVAL;
		}

		// set hierarchy
		switch (fe_params->hierarchy) {
		case HIERARCHY_NONE:
			tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x60, 0 << 5);
			break;

		case HIERARCHY_1:
			tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x60, 1 << 5);
			break;

		case HIERARCHY_2:
			tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x60, 2 << 5);
			break;

		case HIERARCHY_4:
			tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x60, 3 << 5);
			break;

		default:
			return -EINVAL;
		}
	}

	// set bandwidth
	switch (state->demod_type) {
	case TDA1004X_DEMOD_TDA10045:
		tda10045h_set_bandwidth(state, fe_params->bandwidth_hz);
		break;

	case TDA1004X_DEMOD_TDA10046:
		tda10046h_set_bandwidth(state, fe_params->bandwidth_hz);
		break;
	}

	// set inversion
	inversion = fe_params->inversion;
	if (state->config->invert)
		inversion = inversion ? INVERSION_OFF : INVERSION_ON;
	switch (inversion) {
	case INVERSION_OFF:
		tda1004x_write_mask(state, TDA1004X_CONFC1, 0x20, 0);
		break;

	case INVERSION_ON:
		tda1004x_write_mask(state, TDA1004X_CONFC1, 0x20, 0x20);
		break;

	default:
		return -EINVAL;
	}

	// set guard interval
	switch (fe_params->guard_interval) {
	case GUARD_INTERVAL_1_32:
		tda1004x_write_mask(state, TDA1004X_AUTO, 2, 0);
		tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x0c, 0 << 2);
		break;

	case GUARD_INTERVAL_1_16:
		tda1004x_write_mask(state, TDA1004X_AUTO, 2, 0);
		tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x0c, 1 << 2);
		break;

	case GUARD_INTERVAL_1_8:
		tda1004x_write_mask(state, TDA1004X_AUTO, 2, 0);
		tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x0c, 2 << 2);
		break;

	case GUARD_INTERVAL_1_4:
		tda1004x_write_mask(state, TDA1004X_AUTO, 2, 0);
		tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x0c, 3 << 2);
		break;

	case GUARD_INTERVAL_AUTO:
		tda1004x_write_mask(state, TDA1004X_AUTO, 2, 2);
		tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x0c, 0 << 2);
		break;

	default:
		return -EINVAL;
	}

	// set transmission mode
	switch (fe_params->transmission_mode) {
	case TRANSMISSION_MODE_2K:
		tda1004x_write_mask(state, TDA1004X_AUTO, 4, 0);
		tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x10, 0 << 4);
		break;

	case TRANSMISSION_MODE_8K:
		tda1004x_write_mask(state, TDA1004X_AUTO, 4, 0);
		tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x10, 1 << 4);
		break;

	case TRANSMISSION_MODE_AUTO:
		tda1004x_write_mask(state, TDA1004X_AUTO, 4, 4);
		tda1004x_write_mask(state, TDA1004X_IN_CONF1, 0x10, 0);
		break;

	default:
		return -EINVAL;
	}

	// start the lock
	switch (state->demod_type) {
	case TDA1004X_DEMOD_TDA10045:
		tda1004x_write_mask(state, TDA1004X_CONFC4, 8, 8);
		tda1004x_write_mask(state, TDA1004X_CONFC4, 8, 0);
		break;

	case TDA1004X_DEMOD_TDA10046:
		tda1004x_write_mask(state, TDA1004X_AUTO, 0x40, 0x40);
		msleep(1);
		tda1004x_write_mask(state, TDA10046H_AGC_CONF, 4, 1);
		break;
	}

	msleep(10);

	return 0;
}

static int tda1004x_get_fe(struct dvb_frontend *fe,
			   struct dtv_frontend_properties *fe_params)
{
	struct tda1004x_state* state = fe->demodulator_priv;
	int status;

	dprintk("%s\n", __func__);

	status = tda1004x_read_byte(state, TDA1004X_STATUS_CD);
	if (status == -1)
		return -EIO;

	/* Only update the properties cache if device is locked */
	if (!(status & 8))
		return 0;

	// inversion status
	fe_params->inversion = INVERSION_OFF;
	if (tda1004x_read_byte(state, TDA1004X_CONFC1) & 0x20)
		fe_params->inversion = INVERSION_ON;
	if (state->config->invert)
		fe_params->inversion = fe_params->inversion ? INVERSION_OFF : INVERSION_ON;

	// bandwidth
	switch (state->demod_type) {
	case TDA1004X_DEMOD_TDA10045:
		switch (tda1004x_read_byte(state, TDA10045H_WREF_LSB)) {
		case 0x14:
			fe_params->bandwidth_hz = 8000000;
			break;
		case 0xdb:
			fe_params->bandwidth_hz = 7000000;
			break;
		case 0x4f:
			fe_params->bandwidth_hz = 6000000;
			break;
		}
		break;
	case TDA1004X_DEMOD_TDA10046:
		switch (tda1004x_read_byte(state, TDA10046H_TIME_WREF1)) {
		case 0x5c:
		case 0x54:
			fe_params->bandwidth_hz = 8000000;
			break;
		case 0x6a:
		case 0x60:
			fe_params->bandwidth_hz = 7000000;
			break;
		case 0x7b:
		case 0x70:
			fe_params->bandwidth_hz = 6000000;
			break;
		}
		break;
	}

	// FEC
	fe_params->code_rate_HP =
	    tda1004x_decode_fec(tda1004x_read_byte(state, TDA1004X_OUT_CONF2) & 7);
	fe_params->code_rate_LP =
	    tda1004x_decode_fec((tda1004x_read_byte(state, TDA1004X_OUT_CONF2) >> 3) & 7);

	/* modulation */
	switch (tda1004x_read_byte(state, TDA1004X_OUT_CONF1) & 3) {
	case 0:
		fe_params->modulation = QPSK;
		break;
	case 1:
		fe_params->modulation = QAM_16;
		break;
	case 2:
		fe_params->modulation = QAM_64;
		break;
	}

	// transmission mode
	fe_params->transmission_mode = TRANSMISSION_MODE_2K;
	if (tda1004x_read_byte(state, TDA1004X_OUT_CONF1) & 0x10)
		fe_params->transmission_mode = TRANSMISSION_MODE_8K;

	// guard interval
	switch ((tda1004x_read_byte(state, TDA1004X_OUT_CONF1) & 0x0c) >> 2) {
	case 0:
		fe_params->guard_interval = GUARD_INTERVAL_1_32;
		break;
	case 1:
		fe_params->guard_interval = GUARD_INTERVAL_1_16;
		break;
	case 2:
		fe_params->guard_interval = GUARD_INTERVAL_1_8;
		break;
	case 3:
		fe_params->guard_interval = GUARD_INTERVAL_1_4;
		break;
	}

	// hierarchy
	switch ((tda1004x_read_byte(state, TDA1004X_OUT_CONF1) & 0x60) >> 5) {
	case 0:
		fe_params->hierarchy = HIERARCHY_NONE;
		break;
	case 1:
		fe_params->hierarchy = HIERARCHY_1;
		break;
	case 2:
		fe_params->hierarchy = HIERARCHY_2;
		break;
	case 3:
		fe_params->hierarchy = HIERARCHY_4;
		break;
	}

	return 0;
}

static int tda1004x_read_status(struct dvb_frontend *fe,
				enum fe_status *fe_status)
{
	struct tda1004x_state* state = fe->demodulator_priv;
	int status;
	int cber;
	int vber;

	dprintk("%s\n", __func__);

	// read status
	status = tda1004x_read_byte(state, TDA1004X_STATUS_CD);
	if (status == -1)
		return -EIO;

	// decode
	*fe_status = 0;
	if (status & 4)
		*fe_status |= FE_HAS_SIGNAL;
	if (status & 2)
		*fe_status |= FE_HAS_CARRIER;
	if (status & 8)
		*fe_status |= FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;

	// if we don't altic irr-wI-wI-wI-wI-g2:
	RRIER;6Cr-vtwams->guard_interval = GUARD_INTERVAL_1_16;
		break;
	case 2:
		fe_params->guard_interval = GUARD_I1_16;
		breaC-wI-wxus = C-v K;
	y update the properties cache if device is locked */
	if (!(status & 8))
		return 0;

	// inversio*u6C-wI-bGTrn -
		break;
	case 1:
		fe_params->guard_interval = GUARD_INTERVAL_1_16;
		break;
	case 2:
		fe_param)
			return tmp;&,C-kFs &C-v/pC-wI-wI-v = C-oqo	return 0{<I-USs<= 6000000;
			break;
		}
		break;
	}

	// FEC
	fe_params->code_rate_HP =
	    tda1004x_decode_fec(5Y2_LSB, 0xk_ty6(a1004x_wri<I-btm<DEMOD_TDA10045:
		switch (tda1004x_read_byte(state, TDA10045H_WREF_LSB)) {
		case 0x14:
			fe_para** {
		case  0xk_ty6(a1004x_wri<I-btm<DEMOD_TDA10045:
		switch (tda1004x_read_byte(st.oqo6
			break;
		case 0x4f:
			fe_params->ba6Cr-w tda1004x_read_byte(w;
	case 2:
		fe_params->hierarchy = HIERARCHY_2;
		break;
	case 3:
		fe_params->hierarchy = HIERARI-wI-g-btm<DEMOD_TDA10045:
		switch (tda100evision Thierarchy = HIERARCHY_2;
		break;
	case 3X
		fe_params->hierarchy = HIERARI-wI-g-btm<DEMOD_TDA10045:
		switch (tdaX<0x7b6VAL;
	}

	// set transmission mod	      cUi= GU0;
			break;
		}
		break;
	}

	// FEC
	fe_params->code_rate_HP =
	    tda1004x_decode_fec(tda1004x*	rate_HPC-PiAX, 0qo04x_write_mask(state, TDA1004X_IN_CONF2, 0x3f, 0);	// turn off FEC bits
	} else {
		tda1004x_write_mask(state, TDAr-wI-wI-wI-Pp = tda1004x_e(a10C-bn -EINVAL;
		}
	<I&ecase TDA1004X_DEMOD_TDS fe->dee->dee->dee->dmERARCHY_:nPtf);o,ch (tda100evision Thierarchy = HIERARCHY_2archy/ decod separOuUvte_mask(state, TDA6decoderRCHhate_ass4X_params->hieTSe_ackets,ter.
	se mustHY_A6decok;
	caseparam_intech (t// FEC
	fe_p_TDS632_params->hierarchy = HIERA_int;
		break;
val;

, TDA1004X_CONFs->hierarread_byte(state, TDACARRIER;
	i;0;
	if (status & 4)
		*fe_status |= FE_HADA100_		tdng8M[] = { 0	printk("tda1004x:u16 * TDA100up plls for 53MHz sampling clock\n");
		tda1004x_write_byteI(sta10046H_AGn", pos, txtic irr-wI-wI-wI-wI-g2:
	RRIER;6durn m	if t trn",is04x: ((fe_(tda1004x_read_byte(state, TDA10046H_TIME_WREF1)) {
		case 0x5c:n", podth_hz = 8ER_LStor_priv;
	int status;

	dprintk("%s\n", __fun", podth_hz >demod_address;tda1004x_read_statunPtf)rams			break;

		calen);

	result = 0;
	for (i A1004X_IN_Crval = GUARD_I1*TDA100bre( hierar8)erva10046g->if_freq) {
DA100read_byte(state, TDACTDA100u;0;
	if (status & 4)
		*fe_status |= FE_HAnr[] = { 0	printk("tda1004x:u16 * Tnrup plls for 53MHz sampling clock\n");
		tda1004x_write_byteI(sta1004altic irr-wI-wI-wI-wI-g2:
	RRIER;6Cr-vtrams			break;

		calen);

	result = 
		break;
NR	for (i A1004X_IN_Crval = GUARD_s			bre255 -ta1004al*Tnrbre(( hierar8)erva10)046g->if_freq) {
nrread_byte(state, TDACTnru;0;
	if (status & 4)
		*fe_status |= FE_Huc
				_INFO "tda1004x: setting ,;
	s* uc
				_up plls for 53MHz sampling clock\n");
		tda1004x_write_byteI(sta10046H_AGa102_LOCK;

->i2c,04altic irr-wI-wI-wI-wI-g2:
	RRIER;6Cr-vtt trUCBn ThS mast TDA1	if .len = C-v K			break;

		calen);

	result = 
		break; 0x33	for (i A1004X_IN_Crval = GUARD_s			b&x0f)7f;ate->i2c)f .len ++ <_state*unc__);

	status = tda1004x_read_byte 0x33e on the TDA10045 as it
	// is very unreliable if  0x33e on the TDA10045 as it
	// is very unreliable if  0x33e on the TDA
;
			a1004x_state* state = fe->demodulator_p 0x33	forr (i A10204X_IN_CONF1, 3,VAL;
		}A1020&x0f)7f;at>hieraA10204Xa10)* doaA1020ate, ite_mask(state, T (i A100!x0f)7fTrn -uc
				_100410046tate, T-uc
				_100ite_______te, TDA1004X_CONFuc
				_read_byte(state, TDACuc
				_u;0;
	if (status & 4)
		*fe_status |= FE_Hber[] = { 0	printk("tda1004x:u	s* berup plls for 53MHz sampling clock\n");
		tda1004x_write_byteI(sta1004altic irr-wI-wI-wI-wI-g2:
	RRIER;6Cr-vtra eak;			break;

		calen);

	result = 
		break;SB, 0xk_ty6( (i A1004X_IN_Crval = GUARD_s*on mod	hierar1v K			break;

		calen);

	result = 
		break;WREF_LSB)) { (i A1004X_IN_Crval = GUARD_s*on m|re( hierara1004DEMOD_TDA10045:
		switch (tda1004x_read_byte(st.oqo6
			break;
	case 0x4f:
			fe_params->ba6Cr-w tda1004x_read_bytTDA1004X_CONFon read_byte(state, TDACberu;0;
	if (status & 4)
		*fe_status |= devic4, confc4);

	tda1004x_write_mask(state, TDA10046H_CONF_TRISTATE1, 1, 0);
	/* set GPIOn the bus);
		break;
rn 0;
}

static int tda1004x_get_fe(struct dvb_frontend *fe,
			   struct dtv_frontend_properte pulse kille0);
	}

priv;
	int status;

	dprintk("%s\n", __fuwitch ( HIERAs_reatriCONF_Trite_mask(state, TDAess necessary */
	if (tda1004x_check1write_ma_fuwit		fe_p(fe);
	tda1004e) {
siredrite_n the buso request functionn the bus s;at>hiern the buso>2, 0xc0, 0);
00_Ie_byteI(state, TDA10eturn 0;

	/*
	   For i2c normal work, we 
eI(statrn the buso04X_DEMO^ Gain R
tend *fe,
			   struct dtv_frontend_properte pboard,
	  __funrams)
{
	struct tda1004x_state* state = fe->dem1e properties cache i
	if (status & 4)
		*fe_status |=tandard params] = { 0	printk("tda1004x:*fe_C
		tm\n");
		return -EIO;
	}

	tda1004x_write_mask(state, TDA1004X_COC
		tm\uct tda ADC

	// tda da10045h_set_bandwiaec) {(state, TDA1da ADC

	// tda x00, 0x37, 0x00, 0x4aec) {(sif (!(status & 8))
		return 7, 0_
	case _INFO "tda1004x: setting ,;NFO "tda1004x: sett 7, 0_
	case _ing 
	case _\n");g 
	case _->m	i_delay_e, TD8modula100rifteI(spensNTERVAmakese_bysense== TDAVB-TTriteg 
	case _->s04papter(sta;);g 
	case _->maa xriftesta;);
	if (status & 4)
		lk53m) {
		p4f:
int tc4, confc4);

	tda1004x_write_mask(state, TDA10046ep(100)TRISTATE1, 1, 0);
	/* set kfree0x4aec) {us & 4)
		f (state->cona1004x: sett ops, 0x20, 0)ops,= TDA.delsys,= Tm_iS_AVBTreg=0.info,= TDA	.E_IN)= "P->iips,dth_hz = DAVB-T	retu.;
		fe_pa_m	i)= 51ms->moretu.;
		fe_pa_max TD858ms->moretu.;
		fe_pa_s04ppter(stDS6667retu.caps,=
// wor= HCAN_f[0], berval CAN_f[0]2_3erval CAN_f[0]3_4 |
// wor= HCAN_f[0]5_6erval CAN_f[0]7_8erval CAN_f[0]O) |||
// wor= HCAN_byteerval CAN_ase 0:erval CAN_ase 64erval CAN_ase O) |||
// wor= HCAN_0);
		break;

	case TDerval CAN_04X_IN_CONF1, 0x10,{(s,004.:
int tbreak;

		caleint t,004.87);break;

		0); //g=0.and th=_status |= devig=0.uct t6mhz[] = { 0x02, g=0.iandard param6mhz[] = { 0iandard param,004._writtus & 2)=_status |= rn 0;g=0.gwrittus & 2)=_status |=grn 0;g=0.gwri7, 0_
	case _)=_status |=grn 7, 0_
	case _,004.:
E_HAS_SIGbreak;

		calen);AS_SIG,04.:
E_Hon mod	      cUi= GU0er,04.:
E_HADA100_		tdng8Mbreak;

		calen);ADA100_		tdng8M,04.:
E_HAn mod	      cUi= GUAn ,04.:
E_Huc
				_1004tatus |= FE_Huc
				_,
0;
	4, confc4);

	tda1004ak;

		0)attach(f (state->con4tatus |= bus s*demod_t, 0xab };ams->inveiand20)) {
*eianwrite_mask(state, TDA10046ep(100)et GPIOidOUT_CONal			046em1, ry== TDe manlen 100bCONF_Trite(100)TRIkzal			(*mem, u_mask(state, TDA10046), GFP_ode_EL)) { (i !10046)tda1004x_encode_fec(inCabtm<DE			046em1, ry== TDek;

		0bCONF_AULT:
		tda1004fw, read_sta*params(tda10ONF_Trite(100)t functiTRI bus s;at(100)t nuals: re;at(100)t _AUTO) ||
		4x_get_fe(struct dvb_fro, tx_sizfound= HIERA_AUTObreaa100_Triteidbreak;

		calen);

	result = 
		break;WHIPI:
		fe_paidb<e, TDA1004x_encode_fec(int fec)
5:) {
		isvisioanswened . Givse TDArn ret;
	kfree0x4aec) {		tda1004fw, read_ste_paidb!x0f)25 TDA1004x_encode_fec(inI		retueak;

		c ID100it%02x. Cabtm<proctate->coidet;
	kfree0x4aec) {		tda1004fw, read_st_sizlenF_T (status & 2)
firm chanc&(100)t atus & 2.ops, & 0x20, 0)ops };
	statte->cona1004x: sett ops		   (100)t atus & 2.E1, 1, 0);
	/* so reques;);
	if (s&(100)t atus & 2 {us & 4)
		f (state->cona1004x: sett ops, 0x20, 6)ops,= TDA.delsys,= Tm_iS_AVBTreg=0.info,= TDA	.E_IN)= "P->iips,dth_hz 6 DAVB-T	retu.;
		fe_pa_m	i)= 51ms->moretu.;
		fe_pa_max TD858ms->moretu.;
		fe_pa_s04ppter(stDS6667retu.caps,=
// wor= HCAN_f[0], berval CAN_f[0]2_3erval CAN_f[0]3_4 |
// wor= HCAN_f[0]5_6erval CAN_f[0]7_8erval CAN_f[0]O) |||
// wor= HCAN_byteerval CAN_ase 0:erval CAN_ase 64erval CAN_ase O) |||
// wor= HCAN_0);
		break;

	case TDerval CAN_04X_IN_CONF1, 0x10,{(s,004.:
int tbreak;

		caleint t,004.87);break;

		6); //g=0.and th=_status |= devig=0.uct t6mhz[] = { 0x02, g=0.iandard param6mhz[] = { 0iandard param,004._writtus & 2)=_status |= rn 0;g=0.gwrittus & 2)=_status |=grn 0;g=0.gwri7, 0_
	case _)=_status |=grn 7, 0_
	case _,004.:
E_HAS_SIGbreak;

		calen);AS_SIG,04.:
E_Hon mod	      cUi= GU0er,04.:
E_HADA100_		tdng8Mbreak;

		calen);ADA100_		tdng8M,04.:
E_HAn mod	      cUi= GUAn ,04.:
E_Huc
				_1004tatus |= FE_Huc
				_,
0;
	4, confc4);

	tda1004ak;

		6)attach(f (state->con4tatus |= bus s*demod_t, 0xab };ams->inveiand20)) {
*eianwrite_mask(state, TDA10046ep(100)et GPIOidOUT_CONal			046em1, ry== TDe manlen 100bCONF_Trite(100)TRIkzal			(*mem, u_mask(state, TDA10046), GFP_ode_EL)) { (i !10046)tda1004x_encode_fec(inCabtm<DE			046em1, ry== TDek;

		6bCONF_AULT:
		tda1004fw, read_sta*params(tda10ONF_Trite(100)t functiTRI bus s;at(100)t nuals: re;at(100)t _AUTO) ||
		4x_get_fe(struct dvb_fr6, tx_sizfound= HIERA_AUTObreaa100_Triteidbreak;

		calen);

	result = 
		break;WHIPI:
		fe_paidb<e, TDA1004x_encode_fec(int fec)
6:) {
		isvisioanswened . Givse TDArn ret;
	kfree0x4aec) {		tda1004fw, read_te_paidb!x0f)r_ops.se04x_encode_fec(inI		retueak;

		c ID100it%02x. Cabtm<proctate->coidet;
	kfree0x4aec) {		tda1004fw, read_st_sizlenF_T (status & 2)
firm chanc&(100)t atus & 2.ops, & 0x20, 6)ops };
	statte->cona1004x: sett ops		   (100)t atus & 2.E1, 1, 0);
	/* so re