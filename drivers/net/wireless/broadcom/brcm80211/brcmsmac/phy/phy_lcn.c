/*
 * Copyright (c) 2010 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/cordic.h>

#include <pmu.h>
#include <d11.h>
#include <phy_shim.h>
#include "phy_qmath.h"
#include "phy_hal.h"
#include "phy_radio.h"
#include "phytbl_lcn.h"
#include "phy_lcn.h"

#define PLL_2064_NDIV		90
#define PLL_2064_LOW_END_VCO	3000
#define PLL_2064_LOW_END_KVCO	27
#define PLL_2064_HIGH_END_VCO	4200
#define PLL_2064_HIGH_END_KVCO	68
#define PLL_2064_LOOP_BW_DOUBLER	200
#define PLL_2064_D30_DOUBLER		10500
#define PLL_2064_LOOP_BW	260
#define PLL_2064_D30		8000
#define PLL_2064_CAL_REF_TO	8
#define PLL_2064_MHZ		1000000
#define PLL_2064_OPEN_LOOP_DELAY	5

#define TEMPSENSE			1
#define VBATSENSE           2

#define NOISE_IF_UPD_CHK_INTERVAL	1
#define NOISE_IF_UPD_RST_INTERVAL	60
#define NOISE_IF_UPD_THRESHOLD_CNT	1
#define NOISE_IF_UPD_TRHRESHOLD	50
#define NOISE_IF_UPD_TIMEOUT		1000
#define NOISE_IF_OFF			0
#define NOISE_IF_CHK			1
#define NOISE_IF_ON			2

#define PAPD_BLANKING_PROFILE		3
#define PAPD2LUT			0
#define PAPD_CORR_NORM			0
#define PAPD_BLANKING_THRESHOLD		0
#define PAPD_STOP_AFTER_LAST_UPDATE	0

#define LCN_TARGET_PWR  60

#define LCN_VBAT_OFFSET_433X 34649679
#define LCN_VBAT_SLOPE_433X  8258032

#define LCN_VBAT_SCALE_NOM  53
#define LCN_VBAT_SCALE_DEN  432

#define LCN_TEMPSENSE_OFFSET  80812
#define LCN_TEMPSENSE_DEN  2647

#define LCN_BW_LMT	200
#define LCN_CUR_LMT	1250
#define LCN_MULT	1
#define LCN_VCO_DIV	30
#define LCN_OFFSET	680
#define LCN_FACT	490
#define LCN_CUR_DIV	2640

#define LCNPHY_txgainctrlovrval1_pagain_ovr_val1_SHIFT \
	(0 + 8)
#define LCNPHY_txgainctrlovrval1_pagain_ovr_val1_MASK \
	(0x7f << LCNPHY_txgainctrlovrval1_pagain_ovr_val1_SHIFT)

#define LCNPHY_stxtxgainctrlovrval1_pagain_ovr_val1_SHIFT \
	(0 + 8)
#define LCNPHY_stxtxgainctrlovrval1_pagain_ovr_val1_MASK \
	(0x7f << LCNPHY_stxtxgainctrlovrval1_pagain_ovr_val1_SHIFT)

#define wlc_lcnphy_enable_tx_gain_override(pi) \
	wlc_lcnphy_set_tx_gain_override(pi, true)
#define wlc_lcnphy_disable_tx_gain_override(pi)	\
	wlc_lcnphy_set_tx_gain_override(pi, false)

#define wlc_lcnphy_iqcal_active(pi)	\
	(read_phy_reg((pi), 0x451) & \
	 ((0x1 << 15) | (0x1 << 14)))

#define txpwrctrl_off(pi) (0x7 != ((read_phy_reg(pi, 0x4a4) & 0xE000) >> 13))
#define wlc_lcnphy_tempsense_based_pwr_ctrl_enabled(pi)	\
	(pi->temppwrctrl_capable)
#define wlc_lcnphy_tssi_based_pwr_ctrl_enabled(pi) \
	(pi->hwpwrctrl_capable)

#define SWCTRL_BT_TX		0x18
#define SWCTRL_OVR_DISABLE	0x40

#define	AFE_CLK_INIT_MODE_TXRX2X	1
#define	AFE_CLK_INIT_MODE_PAPD		0

#define LCNPHY_TBL_ID_IQLOCAL			0x00

#define LCNPHY_TBL_ID_RFSEQ         0x08
#define LCNPHY_TBL_ID_GAIN_IDX		0x0d
#define LCNPHY_TBL_ID_SW_CTRL			0x0f
#define LCNPHY_TBL_ID_GAIN_TBL		0x12
#define LCNPHY_TBL_ID_SPUR			0x14
#define LCNPHY_TBL_ID_SAMPLEPLAY		0x15
#define LCNPHY_TBL_ID_SAMPLEPLAY1		0x16

#define LCNPHY_TX_PWR_CTRL_RATE_OFFSET	832
#define LCNPHY_TX_PWR_CTRL_MAC_OFFSET	128
#define LCNPHY_TX_PWR_CTRL_GAIN_OFFSET	192
#define LCNPHY_TX_PWR_CTRL_IQ_OFFSET		320
#define LCNPHY_TX_PWR_CTRL_LO_OFFSET		448
#define LCNPHY_TX_PWR_CTRL_PWR_OFFSET		576

#define LCNPHY_TX_PWR_CTRL_START_INDEX_2G_4313	140

#define LCNPHY_TX_PWR_CTRL_START_NPT		1
#define LCNPHY_TX_PWR_CTRL_MAX_NPT			7

#define LCNPHY_NOISE_SAMPLES_DEFAULT 5000

#define LCNPHY_ACI_DETECT_START      1
#define LCNPHY_ACI_DETECT_PROGRESS   2
#define LCNPHY_ACI_DETECT_STOP       3

#define LCNPHY_ACI_CRSHIFRMLO_TRSH 100
#define LCNPHY_ACI_GLITCH_TRSH 2000
#define	LCNPHY_ACI_TMOUT 250
#define LCNPHY_ACI_DETECT_TIMEOUT  2
#define LCNPHY_ACI_START_DELAY 0

#define wlc_lcnphy_tx_gain_override_enabled(pi)	\
	(0 != (read_phy_reg((pi), 0x43b) & (0x1 << 6)))

#define wlc_lcnphy_total_tx_frames(pi) \
	wlapi_bmac_read_shm((pi)->sh->physhim, M_UCODE_MACSTAT + \
			    offsetof(struct macstat, txallfrm))

struct lcnphy_txgains {
	u16 gm_gain;
	u16 pga_gain;
	u16 pad_gain;
	u16 dac_gain;
};

enum lcnphy_cal_mode {
	LCNPHY_CAL_FULL,
	LCNPHY_CAL_RECAL,
	LCNPHY_CAL_CURRECAL,
	LCNPHY_CAL_DIGCAL,
	LCNPHY_CAL_GCTRL
};

struct lcnphy_rx_iqcomp {
	u8 chan;
	s16 a;
	s16 b;
};

struct lcnphy_spb_tone {
	s16 re;
	s16 im;
};

struct lcnphy_unsign16_struct {
	u16 re;
	u16 im;
};

struct lcnphy_iq_est {
	u32 iq_prod;
	u32 i_pwr;
	u32 q_pwr;
};

struct lcnphy_sfo_cfg {
	u16 ptcentreTs20;
	u16 ptcentreFactor;
};

enum lcnphy_papd_cal_type {
	LCNPHY_PAPD_CAL_CW,
	LCNPHY_PAPD_CAL_OFDM
};

typedef u16 iqcal_gain_params_lcnphy[9];

static const iqcal_gain_params_lcnphy tbl_iqcal_gainparams_lcnphy_2G[] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0},
};

static const iqcal_gain_params_lcnphy *tbl_iqcal_gainparams_lcnphy[1] = {
	tbl_iqcal_gainparams_lcnphy_2G,
};

static const u16 iqcal_gainparams_numgains_lcnphy[1] = {
	ARRAY_SIZE(tbl_iqcal_gainparams_lcnphy_2G),
};

static const struct lcnphy_sfo_cfg lcnphy_sfo_cfg[] = {
	{965, 1087},
	{967, 1085},
	{969, 1082},
	{971, 1080},
	{973, 1078},
	{975, 1076},
	{977, 1073},
	{979, 1071},
	{981, 1069},
	{983, 1067},
	{985, 1065},
	{987, 1063},
	{989, 1060},
	{994, 1055}
};

static const
u16 lcnphy_iqcal_loft_gainladder[] = {
	((2 << 8) | 0),
	((3 << 8) | 0),
	((4 << 8) | 0),
	((6 << 8) | 0),
	((8 << 8) | 0),
	((11 << 8) | 0),
	((16 << 8) | 0),
	((16 << 8) | 1),
	((16 << 8) | 2),
	((16 << 8) | 3),
	((16 << 8) | 4),
	((16 << 8) | 5),
	((16 << 8) | 6),
	((16 << 8) | 7),
	((23 << 8) | 7),
	((32 << 8) | 7),
	((45 << 8) | 7),
	((64 << 8) | 7),
	((91 << 8) | 7),
	((128 << 8) | 7)
};

static const
u16 lcnphy_iqcal_ir_gainladder[] = {
	((1 << 8) | 0),
	((2 << 8) | 0),
	((4 << 8) | 0),
	((6 << 8) | 0),
	((8 << 8) | 0),
	((11 << 8) | 0),
	((16 << 8) | 0),
	((23 << 8) | 0),
	((32 << 8) | 0),
	((45 << 8) | 0),
	((64 << 8) | 0),
	((64 << 8) | 1),
	((64 << 8) | 2),
	((64 << 8) | 3),
	((64 << 8) | 4),
	((64 << 8) | 5),
	((64 << 8) | 6),
	((64 << 8) | 7),
	((91 << 8) | 7),
	((128 << 8) | 7)
};

static const
struct lcnphy_spb_tone lcnphy_spb_tone_3750[] = {
	{88, 0},
	{73, 49},
	{34, 81},
	{-17, 86},
	{-62, 62},
	{-86, 17},
	{-81, -34},
	{-49, -73},
	{0, -88},
	{49, -73},
	{81, -34},
	{86, 17},
	{62, 62},
	{17, 86},
	{-34, 81},
	{-73, 49},
	{-88, 0},
	{-73, -49},
	{-34, -81},
	{17, -86},
	{62, -62},
	{86, -17},
	{81, 34},
	{49, 73},
	{0, 88},
	{-49, 73},
	{-81, 34},
	{-86, -17},
	{-62, -62},
	{-17, -86},
	{34, -81},
	{73, -49},
};

static const
u16 iqlo_loopback_rf_regs[20] = {
	RADIO_2064_REG036,
	RADIO_2064_REG11A,
	RADIO_2064_REG03A,
	RADIO_2064_REG025,
	RADIO_2064_REG028,
	RADIO_2064_REG005,
	RADIO_2064_REG112,
	RADIO_2064_REG0FF,
	RADIO_2064_REG11F,
	RADIO_2064_REG00B,
	RADIO_2064_REG113,
	RADIO_2064_REG007,
	RADIO_2064_REG0FC,
	RADIO_2064_REG0FD,
	RADIO_2064_REG012,
	RADIO_2064_REG057,
	RADIO_2064_REG059,
	RADIO_2064_REG05C,
	RADIO_2064_REG078,
	RADIO_2064_REG092,
};

static const
u16 tempsense_phy_regs[14] = {
	0x503,
	0x4a4,
	0x4d0,
	0x4d9,
	0x4da,
	0x4a6,
	0x938,
	0x939,
	0x4d8,
	0x4d0,
	0x4d7,
	0x4a5,
	0x40d,
	0x4a2,
};

static const
u16 rxiq_cal_rf_reg[11] = {
	RADIO_2064_REG098,
	RADIO_2064_REG116,
	RADIO_2064_REG12C,
	RADIO_2064_REG06A,
	RADIO_2064_REG00B,
	RADIO_2064_REG01B,
	RADIO_2064_REG113,
	RADIO_2064_REG01D,
	RADIO_2064_REG114,
	RADIO_2064_REG02E,
	RADIO_2064_REG12A,
};

static const
struct lcnphy_rx_iqcomp lcnphy_rx_iqcomp_table_rev0[] = {
	{1, 0, 0},
	{2, 0, 0},
	{3, 0, 0},
	{4, 0, 0},
	{5, 0, 0},
	{6, 0, 0},
	{7, 0, 0},
	{8, 0, 0},
	{9, 0, 0},
	{10, 0, 0},
	{11, 0, 0},
	{12, 0, 0},
	{13, 0, 0},
	{14, 0, 0},
	{34, 0, 0},
	{38, 0, 0},
	{42, 0, 0},
	{46, 0, 0},
	{36, 0, 0},
	{40, 0, 0},
	{44, 0, 0},
	{48, 0, 0},
	{52, 0, 0},
	{56, 0, 0},
	{60, 0, 0},
	{64, 0, 0},
	{100, 0, 0},
	{104, 0, 0},
	{108, 0, 0},
	{112, 0, 0},
	{116, 0, 0},
	{120, 0, 0},
	{124, 0, 0},
	{128, 0, 0},
	{132, 0, 0},
	{136, 0, 0},
	{140, 0, 0},
	{149, 0, 0},
	{153, 0, 0},
	{157, 0, 0},
	{161, 0, 0},
	{165, 0, 0},
	{184, 0, 0},
	{188, 0, 0},
	{192, 0, 0},
	{196, 0, 0},
	{200, 0, 0},
	{204, 0, 0},
	{208, 0, 0},
	{212, 0, 0},
	{216, 0, 0},
};

static const u32 lcnphy_23bitgaincode_table[] = {
	0x200100,
	0x200200,
	0x200004,
	0x200014,
	0x200024,
	0x200034,
	0x200134,
	0x200234,
	0x200334,
	0x200434,
	0x200037,
	0x200137,
	0x200237,
	0x200337,
	0x200437,
	0x000035,
	0x000135,
	0x000235,
	0x000037,
	0x000137,
	0x000237,
	0x000337,
	0x00013f,
	0x00023f,
	0x00033f,
	0x00034f,
	0x00044f,
	0x00144f,
	0x00244f,
	0x00254f,
	0x00354f,
	0x00454f,
	0x00464f,
	0x01464f,
	0x02464f,
	0x03464f,
	0x04464f,
};

static const s8 lcnphy_gain_table[] = {
	-16,
	-13,
	10,
	7,
	4,
	0,
	3,
	6,
	9,
	12,
	15,
	18,
	21,
	24,
	27,
	30,
	33,
	36,
	39,
	42,
	45,
	48,
	50,
	53,
	56,
	59,
	62,
	65,
	68,
	71,
	74,
	77,
	80,
	83,
	86,
	89,
	92,
};

static const s8 lcnphy_gain_index_offset_for_rssi[] = {
	7,
	7,
	7,
	7,
	7,
	7,
	7,
	8,
	7,
	7,
	6,
	7,
	7,
	4,
	4,
	4,
	4,
	4,
	4,
	4,
	4,
	3,
	3,
	3,
	3,
	3,
	3,
	4,
	2,
	2,
	2,
	2,
	2,
	2,
	-1,
	-2,
	-2,
	-2
};

struct chan_info_2064_lcnphy {
	uint chan;
	uint freq;
	u8 logen_buftune;
	u8 logen_rccr_tx;
	u8 txrf_mix_tune_ctrl;
	u8 pa_input_tune_g;
	u8 logen_rccr_rx;
	u8 pa_rxrf_lna1_freq_tune;
	u8 pa_rxrf_lna2_freq_tune;
	u8 rxrf_rxrf_spare1;
};

static const struct chan_info_2064_lcnphy chan_info_2064_lcnphy[] = {
	{1, 2412, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{2, 2417, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{3, 2422, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{4, 2427, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{5, 2432, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{6, 2437, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{7, 2442, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{8, 2447, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{9, 2452, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{10, 2457, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{11, 2462, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{12, 2467, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{13, 2472, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
	{14, 2484, 0x0B, 0x0A, 0x00, 0x07, 0x0A, 0x88, 0x88, 0x80},
};

static const struct lcnphy_radio_regs lcnphy_radio_regs_2064[] = {
	{0x00, 0, 0, 0, 0},
	{0x01, 0x64, 0x64, 0, 0},
	{0x02, 0x20, 0x20, 0, 0},
	{0x03, 0x66, 0x66, 0, 0},
	{0x04, 0xf8, 0xf8, 0, 0},
	{0x05, 0, 0, 0, 0},
	{0x06, 0x10, 0x10, 0, 0},
	{0x07, 0, 0, 0, 0},
	{0x08, 0, 0, 0, 0},
	{0x09, 0, 0, 0, 0},
	{0x0A, 0x37, 0x37, 0, 0},
	{0x0B, 0x6, 0x6, 0, 0},
	{0x0C, 0x55, 0x55, 0, 0},
	{0x0D, 0x8b, 0x8b, 0, 0},
	{0x0E, 0, 0, 0, 0},
	{0x0F, 0x5, 0x5, 0, 0},
	{0x10, 0, 0, 0, 0},
	{0x11, 0xe, 0xe, 0, 0},
	{0x12, 0, 0, 0, 0},
	{0x13, 0xb, 0xb, 0, 0},
	{0x14, 0x2, 0x2, 0, 0},
	{0x15, 0x12, 0x12, 0, 0},
	{0x16, 0x12, 0x12, 0, 0},
	{0x17, 0xc, 0xc, 0, 0},
	{0x18, 0xc, 0xc, 0, 0},
	{0x19, 0xc, 0xc, 0, 0},
	{0x1A, 0x8, 0x8, 0, 0},
	{0x1B, 0x2, 0x2, 0, 0},
	{0x1C, 0, 0, 0, 0},
	{0x1D, 0x1, 0x1, 0, 0},
	{0x1E, 0x12, 0x12, 0, 0},
	{0x1F, 0x6e, 0x6e, 0, 0},
	{0x20, 0x2, 0x2, 0, 0},
	{0x21, 0x23, 0x23, 0, 0},
	{0x22, 0x8, 0x8, 0, 0},
	{0x23, 0, 0, 0, 0},
	{0x24, 0, 0, 0, 0},
	{0x25, 0xc, 0xc, 0, 0},
	{0x26, 0x33, 0x33, 0, 0},
	{0x27, 0x55, 0x55, 0, 0},
	{0x28, 0, 0, 0, 0},
	{0x29, 0x30, 0x30, 0, 0},
	{0x2A, 0xb, 0xb, 0, 0},
	{0x2B, 0x1b, 0x1b, 0, 0},
	{0x2C, 0x3, 0x3, 0, 0},
	{0x2D, 0x1b, 0x1b, 0, 0},
	{0x2E, 0, 0, 0, 0},
	{0x2F, 0x20, 0x20, 0, 0},
	{0x30, 0xa, 0xa, 0, 0},
	{0x31, 0, 0, 0, 0},
	{0x32, 0x62, 0x62, 0, 0},
	{0x33, 0x19, 0x19, 0, 0},
	{0x34, 0x33, 0x33, 0, 0},
	{0x35, 0x77, 0x77, 0, 0},
	{0x36, 0, 0, 0, 0},
	{0x37, 0x70, 0x70, 0, 0},
	{0x38, 0x3, 0x3, 0, 0},
	{0x39, 0xf, 0xf, 0, 0},
	{0x3A, 0x6, 0x6, 0, 0},
	{0x3B, 0xcf, 0xcf, 0, 0},
	{0x3C, 0x1a, 0x1a, 0, 0},
	{0x3D, 0x6, 0x6, 0, 0},
	{0x3E, 0x42, 0x42, 0, 0},
	{0x3F, 0, 0, 0, 0},
	{0x40, 0xfb, 0xfb, 0, 0},
	{0x41, 0x9a, 0x9a, 0, 0},
	{0x42, 0x7a, 0x7a, 0, 0},
	{0x43, 0x29, 0x29, 0, 0},
	{0x44, 0, 0, 0, 0},
	{0x45, 0x8, 0x8, 0, 0},
	{0x46, 0xce, 0xce, 0, 0},
	{0x47, 0x27, 0x27, 0, 0},
	{0x48, 0x62, 0x62, 0, 0},
	{0x49, 0x6, 0x6, 0, 0},
	{0x4A, 0x58, 0x58, 0, 0},
	{0x4B, 0xf7, 0xf7, 0, 0},
	{0x4C, 0, 0, 0, 0},
	{0x4D, 0xb3, 0xb3, 0, 0},
	{0x4E, 0, 0, 0, 0},
	{0x4F, 0x2, 0x2, 0, 0},
	{0x50, 0, 0, 0, 0},
	{0x51, 0x9, 0x9, 0, 0},
	{0x52, 0x5, 0x5, 0, 0},
	{0x53, 0x17, 0x17, 0, 0},
	{0x54, 0x38, 0x38, 0, 0},
	{0x55, 0, 0, 0, 0},
	{0x56, 0, 0, 0, 0},
	{0x57, 0xb, 0xb, 0, 0},
	{0x58, 0, 0, 0, 0},
	{0x59, 0, 0, 0, 0},
	{0x5A, 0, 0, 0, 0},
	{0x5B, 0, 0, 0, 0},
	{0x5C, 0, 0, 0, 0},
	{0x5D, 0, 0, 0, 0},
	{0x5E, 0x88, 0x88, 0, 0},
	{0x5F, 0xcc, 0xcc, 0, 0},
	{0x60, 0x74, 0x74, 0, 0},
	{0x61, 0x74, 0x74, 0, 0},
	{0x62, 0x74, 0x74, 0, 0},
	{0x63, 0x44, 0x44, 0, 0},
	{0x64, 0x77, 0x77, 0, 0},
	{0x65, 0x44, 0x44, 0, 0},
	{0x66, 0x77, 0x77, 0, 0},
	{0x67, 0x55, 0x55, 0, 0},
	{0x68, 0x77, 0x77, 0, 0},
	{0x69, 0x77, 0x77, 0, 0},
	{0x6A, 0, 0, 0, 0},
	{0x6B, 0x7f, 0x7f, 0, 0},
	{0x6C, 0x8, 0x8, 0, 0},
	{0x6D, 0, 0, 0, 0},
	{0x6E, 0x88, 0x88, 0, 0},
	{0x6F, 0x66, 0x66, 0, 0},
	{0x70, 0x66, 0x66, 0, 0},
	{0x71, 0x28, 0x28, 0, 0},
	{0x72, 0x55, 0x55, 0, 0},
	{0x73, 0x4, 0x4, 0, 0},
	{0x74, 0, 0, 0, 0},
	{0x75, 0, 0, 0, 0},
	{0x76, 0, 0, 0, 0},
	{0x77, 0x1, 0x1, 0, 0},
	{0x78, 0xd6, 0xd6, 0, 0},
	{0x79, 0, 0, 0, 0},
	{0x7A, 0, 0, 0, 0},
	{0x7B, 0, 0, 0, 0},
	{0x7C, 0, 0, 0, 0},
	{0x7D, 0, 0, 0, 0},
	{0x7E, 0, 0, 0, 0},
	{0x7F, 0, 0, 0, 0},
	{0x80, 0, 0, 0, 0},
	{0x81, 0, 0, 0, 0},
	{0x82, 0, 0, 0, 0},
	{0x83, 0xb4, 0xb4, 0, 0},
	{0x84, 0x1, 0x1, 0, 0},
	{0x85, 0x20, 0x20, 0, 0},
	{0x86, 0x5, 0x5, 0, 0},
	{0x87, 0xff, 0xff, 0, 0},
	{0x88, 0x7, 0x7, 0, 0},
	{0x89, 0x77, 0x77, 0, 0},
	{0x8A, 0x77, 0x77, 0, 0},
	{0x8B, 0x77, 0x77, 0, 0},
	{0x8C, 0x77, 0x77, 0, 0},
	{0x8D, 0x8, 0x8, 0, 0},
	{0x8E, 0xa, 0xa, 0, 0},
	{0x8F, 0x8, 0x8, 0, 0},
	{0x90, 0x18, 0x18, 0, 0},
	{0x91, 0x5, 0x5, 0, 0},
	{0x92, 0x1f, 0x1f, 0, 0},
	{0x93, 0x10, 0x10, 0, 0},
	{0x94, 0x3, 0x3, 0, 0},
	{0x95, 0, 0, 0, 0},
	{0x96, 0, 0, 0, 0},
	{0x97, 0xaa, 0xaa, 0, 0},
	{0x98, 0, 0, 0, 0},
	{0x99, 0x23, 0x23, 0, 0},
	{0x9A, 0x7, 0x7, 0, 0},
	{0x9B, 0xf, 0xf, 0, 0},
	{0x9C, 0x10, 0x10, 0, 0},
	{0x9D, 0x3, 0x3, 0, 0},
	{0x9E, 0x4, 0x4, 0, 0},
	{0x9F, 0x20, 0x20, 0, 0},
	{0xA0, 0, 0, 0, 0},
	{0xA1, 0, 0, 0, 0},
	{0xA2, 0, 0, 0, 0},
	{0xA3, 0, 0, 0, 0},
	{0xA4, 0x1, 0x1, 0, 0},
	{0xA5, 0x77, 0x77, 0, 0},
	{0xA6, 0x77, 0x77, 0, 0},
	{0xA7, 0x77, 0x77, 0, 0},
	{0xA8, 0x77, 0x77, 0, 0},
	{0xA9, 0x8c, 0x8c, 0, 0},
	{0xAA, 0x88, 0x88, 0, 0},
	{0xAB, 0x78, 0x78, 0, 0},
	{0xAC, 0x57, 0x57, 0, 0},
	{0xAD, 0x88, 0x88, 0, 0},
	{0xAE, 0, 0, 0, 0},
	{0xAF, 0x8, 0x8, 0, 0},
	{0xB0, 0x88, 0x88, 0, 0},
	{0xB1, 0, 0, 0, 0},
	{0xB2, 0x1b, 0x1b, 0, 0},
	{0xB3, 0x3, 0x3, 0, 0},
	{0xB4, 0x24, 0x24, 0, 0},
	{0xB5, 0x3, 0x3, 0, 0},
	{0xB6, 0x1b, 0x1b, 0, 0},
	{0xB7, 0x24, 0x24, 0, 0},
	{0xB8, 0x3, 0x3, 0, 0},
	{0xB9, 0, 0, 0, 0},
	{0xBA, 0xaa, 0xaa, 0, 0},
	{0xBB, 0, 0, 0, 0},
	{0xBC, 0x4, 0x4, 0, 0},
	{0xBD, 0, 0, 0, 0},
	{0xBE, 0x8, 0x8, 0, 0},
	{0xBF, 0x11, 0x11, 0, 0},
	{0xC0, 0, 0, 0, 0},
	{0xC1, 0, 0, 0, 0},
	{0xC2, 0x62, 0x62, 0, 0},
	{0xC3, 0x1e, 0x1e, 0, 0},
	{0xC4, 0x33, 0x33, 0, 0},
	{0xC5, 0x37, 0x37, 0, 0},
	{0xC6, 0, 0, 0, 0},
	{0xC7, 0x70, 0x70, 0, 0},
	{0xC8, 0x1e, 0x1e, 0, 0},
	{0xC9, 0x6, 0x6, 0, 0},
	{0xCA, 0x4, 0x4, 0, 0},
	{0xCB, 0x2f, 0x2f, 0, 0},
	{0xCC, 0xf, 0xf, 0, 0},
	{0xCD, 0, 0, 0, 0},
	{0xCE, 0xff, 0xff, 0, 0},
	{0xCF, 0x8, 0x8, 0, 0},
	{0xD0, 0x3f, 0x3f, 0, 0},
	{0xD1, 0x3f, 0x3f, 0, 0},
	{0xD2, 0x3f, 0x3f, 0, 0},
	{0xD3, 0, 0, 0, 0},
	{0xD4, 0, 0, 0, 0},
	{0xD5, 0, 0, 0, 0},
	{0xD6, 0xcc, 0xcc, 0, 0},
	{0xD7, 0, 0, 0, 0},
	{0xD8, 0x8, 0x8, 0, 0},
	{0xD9, 0x8, 0x8, 0, 0},
	{0xDA, 0x8, 0x8, 0, 0},
	{0xDB, 0x11, 0x11, 0, 0},
	{0xDC, 0, 0, 0, 0},
	{0xDD, 0x87, 0x87, 0, 0},
	{0xDE, 0x88, 0x88, 0, 0},
	{0xDF, 0x8, 0x8, 0, 0},
	{0xE0, 0x8, 0x8, 0, 0},
	{0xE1, 0x8, 0x8, 0, 0},
	{0xE2, 0, 0, 0, 0},
	{0xE3, 0, 0, 0, 0},
	{0xE4, 0, 0, 0, 0},
	{0xE5, 0xf5, 0xf5, 0, 0},
	{0xE6, 0x30, 0x30, 0, 0},
	{0xE7, 0x1, 0x1, 0, 0},
	{0xE8, 0, 0, 0, 0},
	{0xE9, 0xff, 0xff, 0, 0},
	{0xEA, 0, 0, 0, 0},
	{0xEB, 0, 0, 0, 0},
	{0xEC, 0x22, 0x22, 0, 0},
	{0xED, 0, 0, 0, 0},
	{0xEE, 0, 0, 0, 0},
	{0xEF, 0, 0, 0, 0},
	{0xF0, 0x3, 0x3, 0, 0},
	{0xF1, 0x1, 0x1, 0, 0},
	{0xF2, 0, 0, 0, 0},
	{0xF3, 0, 0, 0, 0},
	{0xF4, 0, 0, 0, 0},
	{0xF5, 0, 0, 0, 0},
	{0xF6, 0, 0, 0, 0},
	{0xF7, 0x6, 0x6, 0, 0},
	{0xF8, 0, 0, 0, 0},
	{0xF9, 0, 0, 0, 0},
	{0xFA, 0x40, 0x40, 0, 0},
	{0xFB, 0, 0, 0, 0},
	{0xFC, 0x1, 0x1, 0, 0},
	{0xFD, 0x80, 0x80, 0, 0},
	{0xFE, 0x2, 0x2, 0, 0},
	{0xFF, 0x10, 0x10, 0, 0},
	{0x100, 0x2, 0x2, 0, 0},
	{0x101, 0x1e, 0x1e, 0, 0},
	{0x102, 0x1e, 0x1e, 0, 0},
	{0x103, 0, 0, 0, 0},
	{0x104, 0x1f, 0x1f, 0, 0},
	{0x105, 0, 0x8, 0, 1},
	{0x106, 0x2a, 0x2a, 0, 0},
	{0x107, 0xf, 0xf, 0, 0},
	{0x108, 0, 0, 0, 0},
	{0x109, 0, 0, 0, 0},
	{0x10A, 0, 0, 0, 0},
	{0x10B, 0, 0, 0, 0},
	{0x10C, 0, 0, 0, 0},
	{0x10D, 0, 0, 0, 0},
	{0x10E, 0, 0, 0, 0},
	{0x10F, 0, 0, 0, 0},
	{0x110, 0, 0, 0, 0},
	{0x111, 0, 0, 0, 0},
	{0x112, 0, 0, 0, 0},
	{0x113, 0, 0, 0, 0},
	{0x114, 0, 0, 0, 0},
	{0x115, 0, 0, 0, 0},
	{0x116, 0, 0, 0, 0},
	{0x117, 0, 0, 0, 0},
	{0x118, 0, 0, 0, 0},
	{0x119, 0, 0, 0, 0},
	{0x11A, 0, 0, 0, 0},
	{0x11B, 0, 0, 0, 0},
	{0x11C, 0x1, 0x1, 0, 0},
	{0x11D, 0, 0, 0, 0},
	{0x11E, 0, 0, 0, 0},
	{0x11F, 0, 0, 0, 0},
	{0x120, 0, 0, 0, 0},
	{0x121, 0, 0, 0, 0},
	{0x122, 0x80, 0x80, 0, 0},
	{0x123, 0, 0, 0, 0},
	{0x124, 0xf8, 0xf8, 0, 0},
	{0x125, 0, 0, 0, 0},
	{0x126, 0, 0, 0, 0},
	{0x127, 0, 0, 0, 0},
	{0x128, 0, 0, 0, 0},
	{0x129, 0, 0, 0, 0},
	{0x12A, 0, 0, 0, 0},
	{0x12B, 0, 0, 0, 0},
	{0x12C, 0, 0, 0, 0},
	{0x12D, 0, 0, 0, 0},
	{0x12E, 0, 0, 0, 0},
	{0x12F, 0, 0, 0, 0},
	{0x130, 0, 0, 0, 0},
	{0xFFFF, 0, 0, 0, 0}
};

#define LCNPHY_NUM_DIG_FILT_COEFFS 16
#define LCNPHY_NUM_TX_DIG_FILTERS_CCK 13

static const u16 LCNPHY_txdigfiltcoeffs_cck[LCNPHY_NUM_TX_DIG_FILTERS_CCK]
	[LCNPHY_NUM_DIG_FILT_COEFFS + 1] = {
	{0, 1, 415, 1874, 64, 128, 64, 792, 1656, 64, 128, 64, 778, 1582, 64,
	 128, 64,},
	{1, 1, 402, 1847, 259, 59, 259, 671, 1794, 68, 54, 68, 608, 1863, 93,
	 167, 93,},
	{2, 1, 415, 1874, 64, 128, 64, 792, 1656, 192, 384, 192, 778, 1582, 64,
	 128, 64,},
	{3, 1, 302, 1841, 129, 258, 129, 658, 1720, 205, 410, 205, 754, 1760,
	 170, 340, 170,},
	{20, 1, 360, 1884, 242, 1734, 242, 752, 1720, 205, 1845, 205, 767, 1760,
	 256, 185, 256,},
	{21, 1, 360, 1884, 149, 1874, 149, 752, 1720, 205, 1883, 205, 767, 1760,
	 256, 273, 256,},
	{22, 1, 360, 1884, 98, 1948, 98, 752, 1720, 205, 1924, 205, 767, 1760,
	 256, 352, 256,},
	{23, 1, 350, 1884, 116, 1966, 116, 752, 1720, 205, 2008, 205, 767, 1760,
	 128, 233, 128,},
	{24, 1, 325, 1884, 32, 40, 32, 756, 1720, 256, 471, 256, 766, 1760, 256,
	 1881, 256,},
	{25, 1, 299, 1884, 51, 64, 51, 736, 1720, 256, 471, 256, 765, 1760, 256,
	 1881, 256,},
	{26, 1, 277, 1943, 39, 117, 88, 637, 1838, 64, 192, 144, 614, 1864, 128,
	 384, 288,},
	{27, 1, 245, 1943, 49, 147, 110, 626, 1838, 256, 768, 576, 613, 1864,
	 128, 384, 288,},
	{30, 1, 302, 1841, 61, 122, 61, 658, 1720, 205, 410, 205, 754, 1760,
	 170, 340, 170,},
};

#define LCNPHY_NUM_TX_DIG_FILTERS_OFDM 3
static const u16 LCNPHY_txdigfiltcoeffs_ofdm[LCNPHY_NUM_TX_DIG_FILTERS_OFDM]
	[LCNPHY_NUM_DIG_FILT_COEFFS + 1] = {
	{0, 0, 0xa2, 0x0, 0x100, 0x100, 0x0, 0x0, 0x0, 0x100, 0x0, 0x0,
	 0x278, 0xfea0, 0x80, 0x100, 0x80,},
	{1, 0, 374, 0xFF79, 16, 32, 16, 799, 0xFE74, 50, 32, 50,
	 750, 0xFE2B, 212, 0xFFCE, 212,},
	{2, 0, 375, 0xFF16, 37, 76, 37, 799, 0xFE74, 32, 20, 32, 748,
	 0xFEF2, 128, 0xFFE2, 128}
};

#define wlc_lcnphy_set_start_tx_pwr_idx(pi, idx) \
	mod_phy_reg(pi, 0x4a4, \
		    (0x1ff << 0), \
		    (u16)(idx) << 0)

#define wlc_lcnphy_set_tx_pwr_npt(pi, npt) \
	mod_phy_reg(pi, 0x4a5, \
		    (0x7 << 8),	\
		    (u16)(npt) << 8)

#define wlc_lcnphy_get_tx_pwr_ctrl(pi) \
	(read_phy_reg((pi), 0x4a4) & \
	 ((0x1 << 15) |	\
	  (0x1 << 14) |	\
	  (0x1 << 13)))

#define wlc_lcnphy_get_tx_pwr_npt(pi) \
	((read_phy_reg(pi, 0x4a5) & \
	  (0x7 << 8)) >> \
	 8)

#define wlc_lcnphy_get_current_tx_pwr_idx_if_pwrctrl_on(pi) \
	(read_phy_reg(pi, 0x473) & 0x1ff)

#define wlc_lcnphy_get_target_tx_pwr(pi) \
	((read_phy_reg(pi, 0x4a7) & \
	  (0xff << 0)) >> \
	 0)

#define wlc_lcnphy_set_target_tx_pwr(pi, target) \
	mod_phy_reg(pi, 0x4a7, \
		    (0xff << 0), \
		    (u16)(target) << 0)

#define wlc_radio_2064_rcal_done(pi) \
	(0 != (read_radio_reg(pi, RADIO_2064_REG05C) & 0x20))

#define tempsense_done(pi) \
	(0x8000 == (read_phy_reg(pi, 0x476) & 0x8000))

#define LCNPHY_IQLOCC_READ(val) \
	((u8)(-(s8)(((val) & 0xf0) >> 4) + (s8)((val) & 0x0f)))

#define FIXED_TXPWR 78
#define LCNPHY_TEMPSENSE(val) ((s16)((val > 255) ? (val - 512) : val))

void wlc_lcnphy_write_table(struct brcms_phy *pi, const struct phytbl_info *pti)
{
	wlc_phy_write_table(pi, pti, 0x455, 0x457, 0x456);
}

void wlc_lcnphy_read_table(struct brcms_phy *pi, struct phytbl_info *pti)
{
	wlc_phy_read_table(pi, pti, 0x455, 0x457, 0x456);
}

static void
wlc_lcnphy_common_read_table(struct brcms_phy *pi, u32 tbl_id,
			     const u16 *tbl_ptr, u32 tbl_len,
			     u32 tbl_width, u32 tbl_offset)
{
	struct phytbl_info tab;
	tab.tbl_id = tbl_id;
	tab.tbl_ptr = tbl_ptr;
	tab.tbl_len = tbl_len;
	tab.tbl_width = tbl_width;
	tab.tbl_offset = tbl_offset;
	wlc_lcnphy_read_table(pi, &tab);
}

static void
wlc_lcnphy_common_write_table(struct brcms_phy *pi, u32 tbl_id,
			      const u16 *tbl_ptr, u32 tbl_len,
			      u32 tbl_width, u32 tbl_offset)
{

	struct phytbl_info tab;
	tab.tbl_id = tbl_id;
	tab.tbl_ptr = tbl_ptr;
	tab.tbl_len = tbl_len;
	tab.tbl_width = tbl_width;
	tab.tbl_offset = tbl_offset;
	wlc_lcnphy_write_table(pi, &tab);
}

static u32
wlc_lcnphy_qdiv_roundup(u32 dividend, u32 divisor, u8 precision)
{
	u32 quotient, remainder, roundup, rbit;

	quotient = dividend / divisor;
	remainder = dividend % divisor;
	rbit = divisor & 1;
	roundup = (divisor >> 1) + rbit;

	while (precision--) {
		quotient <<= 1;
		if (remainder >= roundup) {
			quotient++;
			remainder = ((remainder - roundup) << 1) + rbit;
		} else {
			remainder <<= 1;
		}
	}

	if (remainder >= roundup)
		quotient++;

	return quotient;
}

static int wlc_lcnphy_calc_floor(s16 coeff_x, int type)
{
	int k;
	k = 0;
	if (type == 0) {
		if (coeff_x < 0)
			k = (coeff_x - 1) / 2;
		else
			k = coeff_x / 2;
	}

	if (type == 1) {
		if ((coeff_x + 1) < 0)
			k = (coeff_x) / 2;
		else
			k = (coeff_x + 1) / 2;
	}
	return k;
}

static void
wlc_lcnphy_get_tx_gain(struct brcms_phy *pi, struct lcnphy_txgains *gains)
{
	u16 dac_gain, rfgain0, rfgain1;

	dac_gain = read_phy_reg(pi, 0x439) >> 0;
	gains->dac_gain = (dac_gain & 0x380) >> 7;

	rfgain0 = (read_phy_reg(pi, 0x4b5) & (0xffff << 0)) >> 0;
	rfgain1 = (read_phy_reg(pi, 0x4fb) & (0x7fff << 0)) >> 0;

	gains->gm_gain = rfgain0 & 0xff;
	gains->pga_gain = (rfgain0 >> 8) & 0xff;
	gains->pad_gain = rfgain1 & 0xff;
}


static void wlc_lcnphy_set_dac_gain(struct brcms_phy *pi, u16 dac_gain)
{
	u16 dac_ctrl;

	dac_ctrl = (read_phy_reg(pi, 0x439) >> 0);
	dac_ctrl = dac_ctrl & 0xc7f;
	dac_ctrl = dac_ctrl | (dac_gain << 7);
	mod_phy_reg(pi, 0x439, (0xfff << 0), (dac_ctrl) << 0);

}

static void wlc_lcnphy_set_tx_gain_override(struct brcms_phy *pi, bool bEnable)
{
	u16 bit = bEnable ? 1 : 0;

	mod_phy_reg(pi, 0x4b0, (0x1 << 7), bit << 7);

	mod_phy_reg(pi, 0x4b0, (0x1 << 14), bit << 14);

	mod_phy_reg(pi, 0x43b, (0x1 << 6), bit << 6);
}

static void
wlc_lcnphy_rx_gain_override_enable(struct brcms_phy *pi, bool enable)
{
	u16 ebit = enable ? 1 : 0;

	mod_phy_reg(pi, 0x4b0, (0x1 << 8), ebit << 8);

	mod_phy_reg(pi, 0x44c, (0x1 << 0), ebit << 0);

	if (LCNREV_LT(pi->pubpi.phy_rev, 2)) {
		mod_phy_reg(pi, 0x44c, (0x1 << 4), ebit << 4);
		mod_phy_reg(pi, 0x44c, (0x1 << 6), ebit << 6);
		mod_phy_reg(pi, 0x4b0, (0x1 << 5), ebit << 5);
		mod_phy_reg(pi, 0x4b0, (0x1 << 6), ebit << 6);
	} else {
		mod_phy_reg(pi, 0x4b0, (0x1 << 12), ebit << 12);
		mod_phy_reg(pi, 0x4b0, (0x1 << 13), ebit << 13);
		mod_phy_reg(pi, 0x4b0, (0x1 << 5), ebit << 5);
	}

	if (CHSPEC_IS2G(pi->radio_chanspec)) {
		mod_phy_reg(pi, 0x4b0, (0x1 << 10), ebit << 10);
		mod_phy_reg(pi, 0x4e5, (0x1 << 3), ebit << 3);
	}
}

static void
wlc_lcnphy_set_rx_gain_by_distribution(struct brcms_phy *pi,
				       u16 trsw,
				       u16 ext_lna,
				       u16 biq2,
				       u16 biq1,
				       u16 tia, u16 lna2, u16 lna1)
{
	u16 gain0_15, gain16_19;

	gain16_19 = biq2 & 0xf;
	gain0_15 = ((biq1 & 0xf) << 12) |
		   ((tia & 0xf) << 8) |
		   ((lna2 & 0x3) << 6) |
		   ((lna2 & 0x3) << 4) |
		   ((lna1 & 0x3) << 2) |
		   ((lna1 & 0x3) << 0);

	mod_phy_reg(pi, 0x4b6, (0xffff << 0), gain0_15 << 0);
	mod_phy_reg(pi, 0x4b7, (0xf << 0), gain16_19 << 0);
	mod_phy_reg(pi, 0x4b1, (0x3 << 11), lna1 << 11);

	if (LCNREV_LT(pi->pubpi.phy_rev, 2)) {
		mod_phy_reg(pi, 0x4b1, (0x1 << 9), ext_lna << 9);
		mod_phy_reg(pi, 0x4b1, (0x1 << 10), ext_lna << 10);
	} else {
		mod_phy_reg(pi, 0x4b1, (0x1 << 10), 0 << 10);

		mod_phy_reg(pi, 0x4b1, (0x1 << 15), 0 << 15);

		mod_phy_reg(pi, 0x4b1, (0x1 << 9), ext_lna << 9);
	}

	mod_phy_reg(pi, 0x44d, (0x1 << 0), (!trsw) << 0);

}

static void wlc_lcnphy_set_trsw_override(struct brcms_phy *pi, bool tx, bool rx)
{

	mod_phy_reg(pi, 0x44d,
		    (0x1 << 1) |
		    (0x1 << 0), (tx ? (0x1 << 1) : 0) | (rx ? (0x1 << 0) : 0));

	or_phy_reg(pi, 0x44c, (0x1 << 1) | (0x1 << 0));
}

static void wlc_lcnphy_clear_trsw_override(struct brcms_phy *pi)
{

	and_phy_reg(pi, 0x44c, (u16) ~((0x1 << 1) | (0x1 << 0)));
}

static void wlc_lcnphy_set_rx_iq_comp(struct brcms_phy *pi, u16 a, u16 b)
{
	mod_phy_reg(pi, 0x645, (0x3ff << 0), (a) << 0);

	mod_phy_reg(pi, 0x646, (0x3ff << 0), (b) << 0);

	mod_phy_reg(pi, 0x647, (0x3ff << 0), (a) << 0);

	mod_phy_reg(pi, 0x648, (0x3ff << 0), (b) << 0);

	mod_phy_reg(pi, 0x649, (0x3ff << 0), (a) << 0);

	mod_phy_reg(pi, 0x64a, (0x3ff << 0), (b) << 0);

}

static bool
wlc_lcnphy_rx_iq_est(struct brcms_phy *pi,
		     u16 num_samps,
		     u8 wait_time, struct lcnphy_iq_est *iq_est)
{
	int wait_count = 0;
	bool result = true;
	u8 phybw40;
	phybw40 = CHSPEC_IS40(pi->radio_chanspec);

	mod_phy_reg(pi, 0x6da, (0x1 << 5), (1) << 5);

	mod_phy_reg(pi, 0x410, (0x1 << 3), (0) << 3);

	mod_phy_reg(pi, 0x482, (0xffff << 0), (num_samps) << 0);

	mod_phy_reg(pi, 0x481, (0xff << 0), ((u16) wait_time) << 0);

	mod_phy_reg(pi, 0x481, (0x1 << 8), (0) << 8);

	mod_phy_reg(pi, 0x481, (0x1 << 9), (1) << 9);

	while (read_phy_reg(pi, 0x481) & (0x1 << 9)) {

		if (wait_count > (10 * 500)) {
			result = false;
			goto cleanup;
		}
		udelay(100);
		wait_count++;
	}

	iq_est->iq_prod = ((u32) read_phy_reg(pi, 0x483) << 16) |
			  (u32) read_phy_reg(pi, 0x484);
	iq_est->i_pwr = ((u32) read_phy_reg(pi, 0x485) << 16) |
			(u32) read_phy_reg(pi, 0x486);
	iq_est->q_pwr = ((u32) read_phy_reg(pi, 0x487) << 16) |
			(u32) read_phy_reg(pi, 0x488);

cleanup:
	mod_phy_reg(pi, 0x410, (0x1 << 3), (1) << 3);

	mod_phy_reg(pi, 0x6da, (0x1 << 5), (0) << 5);

	return result;
}

static bool wlc_lcnphy_calc_rx_iq_comp(struct brcms_phy *pi, u16 num_samps)
{
#define LCNPHY_MIN_RXIQ_PWR 2
	bool result;
	u16 a0_new, b0_new;
	struct lcnphy_iq_est iq_est = { 0, 0, 0 };
	s32 a, b, temp;
	s16 iq_nbits, qq_nbits, arsh, brsh;
	s32 iq;
	u32 ii, qq;
	struct brcms_phy_lcnphy *pi_lcn = pi->u.pi_lcnphy;

	a0_new = ((read_phy_reg(pi, 0x645) & (0x3ff << 0)) >> 0);
	b0_new = ((read_phy_reg(pi, 0x646) & (0x3ff << 0)) >> 0);
	mod_phy_reg(pi, 0x6d1, (0x1 << 2), (0) << 2);

	mod_phy_reg(pi, 0x64b, (0x1 << 6), (1) << 6);

	wlc_lcnphy_set_rx_iq_comp(pi, 0, 0);

	result = wlc_lcnphy_rx_iq_est(pi, num_samps, 32, &iq_est);
	if (!result)
		goto cleanup;

	iq = (s32) iq_est.iq_prod;
	ii = iq_est.i_pwr;
	qq = iq_est.q_pwr;

	if ((ii + qq) < LCNPHY_MIN_RXIQ_PWR) {
		result = false;
		goto cleanup;
	}

	iq_nbits = wlc_phy_nbits(iq);
	qq_nbits = wlc_phy_nbits(qq);

	arsh = 10 - (30 - iq_nbits);
	if (arsh >= 0) {
		a = (-(iq << (30 - iq_nbits)) + (ii >> (1 + arsh)));
		temp = (s32) (ii >> arsh);
		if (temp == 0)
			return false;
	} else {
		a = (-(iq << (30 - iq_nbits)) + (ii << (-1 - arsh)));
		temp = (s32) (ii << -arsh);
		if (temp == 0)
			return false;
	}
	a /= temp;
	brsh = qq_nbits - 31 + 20;
	if (brsh >= 0) {
		b = (qq << (31 - qq_nbits));
		temp = (s32) (ii >> brsh);
		if (temp == 0)
			return false;
	} else {
		b = (qq << (31 - qq_nbits));
		temp = (s32) (ii << -brsh);
		if (temp == 0)
			return false;
	}
	b /= temp;
	b -= a * a;
	b = (s32) int_sqrt((unsigned long) b);
	b -= (1 << 10);
	a0_new = (u16) (a & 0x3ff);
	b0_new = (u16) (b & 0x3ff);
cleanup:

	wlc_lcnphy_set_rx_iq_comp(pi, a0_new, b0_new);

	mod_phy_reg(pi, 0x64b, (0x1 << 0), (1) << 0);

	mod_phy_reg(pi, 0x64b, (0x1 << 3), (1) << 3);

	pi_lcn->lcnphy_cal_results.rxiqcal_coeff_a0 = a0_new;
	pi_lcn->lcnphy_cal_results.rxiqcal_coeff_b0 = b0_new;

	return result;
}

static u32 wlc_lcnphy_measure_digital_power(struct brcms_phy *pi, u16 nsamples)
{
	struct lcnphy_iq_est iq_est = { 0, 0, 0 };

	if (!wlc_lcnphy_rx_iq_est(pi, nsamples, 32, &iq_est))
		return 0;
	return (iq_est.i_pwr + iq_est.q_pwr) / nsamples;
}

static bool wlc_lcnphy_rx_iq_cal_gain(struct brcms_phy *pi, u16 biq1_gain,
				      u16 tia_gain, u16 lna2_gain)
{
	u32 i_thresh_l, q_thresh_l;
	u32 i_thresh_h, q_thresh_h;
	struct lcnphy_iq_est iq_est_h, iq_est_l;

	wlc_lcnphy_set_rx_gain_by_distribution(pi, 0, 0, 0, biq1_gain, tia_gain,
					       lna2_gain, 0);

	wlc_lcnphy_rx_gain_override_enable(pi, true);
	wlc_lcnphy_start_tx_tone(pi, 2000, (40 >> 1), 0);
	udelay(500);
	write_radio_reg(pi, RADIO_2064_REG112, 0);
	if (!wlc_lcnphy_rx_iq_est(pi, 1024, 32, &iq_est_l))
		return false;

	wlc_lcnphy_start_tx_tone(pi, 2000, 40, 0);
	udelay(500);
	write_radio_reg(pi, RADIO_2064_REG112, 0);
	if (!wlc_lcnphy_rx_iq_est(pi, 1024, 32, &iq_est_h))
		return false;

	i_thresh_l = (iq_est_l.i_pwr << 1);
	i_thresh_h = (iq_est_l.i_pwr << 2) + iq_est_l.i_pwr;

	q_thresh_l = (iq_est_l.q_pwr << 1);
	q_thresh_h = (iq_est_l.q_pwr << 2) + iq_est_l.q_pwr;
	if ((iq_est_h.i_pwr > i_thresh_l) &&
	    (iq_est_h.i_pwr < i_thresh_h) &&
	    (iq_est_h.q_pwr > q_thresh_l) &&
	    (iq_est_h.q_pwr < q_thresh_h))
		return true;

	return false;
}

static bool
wlc_lcnphy_rx_iq_cal(struct brcms_phy *pi,
		     const struct lcnphy_rx_iqcomp *iqcomp,
		     int iqcomp_sz, bool tx_switch, bool rx_switch, int module,
		     int tx_gain_idx)
{
	struct lcnphy_txgains old_gains;
	u16 tx_pwr_ctrl;
	u8 tx_gain_index_old = 0;
	bool result = false, tx_gain_override_old = false;
	u16 i, Core1TxControl_old, RFOverride0_old,
	    RFOverrideVal0_old, rfoverride2_old, rfoverride2val_old,
	    rfoverride3_old, rfoverride3val_old, rfoverride4_old,
	    rfoverride4val_old, afectrlovr_old, afectrlovrval_old;
	int tia_gain, lna2_gain, biq1_gain;
	bool set_gain;
	u16 old_sslpnCalibClkEnCtrl, old_sslpnRxFeClkEnCtrl;
	u16 values_to_save[11];
	s16 *ptr;
	struct brcms_phy_lcnphy *pi_lcn = pi->u.pi_lcnphy;

	ptr = kmalloc(sizeof(s16) * 131, GFP_ATOMIC);
	if (NULL == ptr)
		return false;
	if (module == 2) {
		while (iqcomp_sz--) {
			if (iqcomp[iqcomp_sz].chan ==
			    CHSPEC_CHANNEL(pi->radio_chanspec)) {
				wlc_lcnphy_set_rx_iq_comp(pi,
							  (u16)
							  iqcomp[iqcomp_sz].a,
							  (u16)
							  iqcomp[iqcomp_sz].b);
				result = true;
				break;
			}
		}
		goto cal_done;
	}

	WARN_ON(module != 1);
	tx_pwr_ctrl = wlc_lcnphy_get_tx_pwr_ctrl(pi);
	wlc_lcnphy_set_tx_pwr_ctrl(pi, LCNPHY_TX_PWR_CTRL_OFF);

	for (i = 0; i < 11; i++)
		values_to_save[i] =
			read_radio_reg(pi, rxiq_cal_rf_reg[i]);
	Core1TxControl_old = read_phy_reg(pi, 0x631);

	or_phy_reg(pi, 0x631, 0x0015);

	RFOverride0_old = read_phy_reg(pi, 0x44c);
	RFOverrideVal0_old = read_phy_reg(pi, 0x44d);
	rfoverride2_old = read_phy_reg(pi, 0x4b0);
	rfoverride2val_old = read_phy_reg(pi, 0x4b1);
	rfoverride3_old = read_phy_reg(pi, 0x4f9);
	rfoverride3val_old = read_phy_reg(pi, 0x4fa);
	rfoverride4_old = read_phy_reg(pi, 0x938);
	rfoverride4val_old = read_phy_reg(pi, 0x939);
	afectrlovr_old = read_phy_reg(pi, 0x43b);
	afectrlovrval_old = read_phy_reg(pi, 0x43c);
	old_sslpnCalibClkEnCtrl = read_phy_reg(pi, 0x6da);
	old_sslpnRxFeClkEnCtrl = read_phy_reg(pi, 0x6db);

	tx_gain_override_old = wlc_lcnphy_tx_gain_override_enabled(pi);
	if (tx_gain_override_old) {
		wlc_lcnphy_get_tx_gain(pi, &old_gains);
		tx_gain_index_old = pi_lcn->lcnphy_current_index;
	}

	wlc_lcnphy_set_tx_pwr_by_index(pi, tx_gain_idx);

	mod_phy_reg(pi, 0x4f9, (0x1 << 0), 1 << 0);
	mod_phy_reg(pi, 0x4fa, (0x1 << 0), 0 << 0);

	mod_phy_reg(pi, 0x43b, (0x1 << 1), 1 << 1);
	mod_phy_reg(pi, 0x43c, (0x1 << 1), 0 << 1);

	write_radio_reg(pi, RADIO_2064_REG116, 0x06);
	write_radio_reg(pi, RADIO_2064_REG12C, 0x07);
	write_radio_reg(pi, RADIO_2064_REG06A, 0xd3);
	write_radio_reg(pi, RADIO_2064_REG098, 0x03);
	write_radio_reg(pi, RADIO_2064_REG00B, 0x7);
	mod_radio_reg(pi, RADIO_2064_REG113, 1 << 4, 1 << 4);
	write_radio_reg(pi, RADIO_2064_REG01D, 0x01);
	write_radio_reg(pi, RADIO_2064_REG114, 0x01);
	write_radio_reg(pi, RADIO_2064_REG02E, 0x10);
	write_radio_reg(pi, RADIO_2064_REG12A, 0x08);

	mod_phy_reg(pi, 0x938, (0x1 << 0), 1 << 0);
	mod_phy_reg(pi, 0x939, (0x1 << 0), 0 << 0);
	mod_phy_reg(pi, 0x938, (0x1 << 1), 1 << 1);
	mod_phy_reg(pi, 0x939, (0x1 << 1), 1 << 1);
	mod_phy_reg(pi, 0x938, (0x1 << 2), 1 << 2);
	mod_phy_reg(pi, 0x939, (0x1 << 2), 1 << 2);
	mod_phy_reg(pi, 0x938, (0x1 << 3), 1 << 3);
	mod_phy_reg(pi, 0x939, (0x1 << 3), 1 << 3);
	mod_phy_reg(pi, 0x938, (0x1 << 5), 1 << 5);
	mod_phy_reg(pi, 0x939, (0x1 << 5), 0 << 5);

	mod_phy_reg(pi, 0x43b, (0x1 << 0), 1 << 0);
	mod_phy_reg(pi, 0x43c, (0x1 << 0), 0 << 0);

	write_phy_reg(pi, 0x6da, 0xffff);
	or_phy_reg(pi, 0x6db, 0x3);

	wlc_lcnphy_set_trsw_override(pi, tx_switch, rx_switch);
	for (lna2_gain = 3; lna2_gain >= 0; lna2_gain--) {
		for (tia_gain = 4; tia_gain >= 0; tia_gain--) {
			for (biq1_gain = 6; biq1_gain >= 0; biq1_gain--) {
				set_gain = wlc_lcnphy_rx_iq_cal_gain(pi,
								     (u16)
								     biq1_gain,
								     (u16)
								     tia_gain,
								     (u16)
								     lna2_gain);
				if (!set_gain)
					continue;

				result = wlc_lcnphy_calc_rx_iq_comp(pi, 1024);
				goto stop_tone;
			}
		}
	}

stop_tone:
	wlc_lcnphy_stop_tx_tone(pi);

	write_phy_reg(pi, 0x631, Core1TxControl_old);

	write_phy_reg(pi, 0x44c, RFOverrideVal0_old);
	write_phy_reg(pi, 0x44d, RFOverrideVal0_old);
	write_phy_reg(pi, 0x4b0, rfoverride2_old);
	write_phy_reg(pi, 0x4b1, rfoverride2val_old);
	write_phy_reg(pi, 0x4f9, rfoverride3_old);
	write_phy_reg(pi, 0x4fa, rfoverride3val_old);
	write_phy_reg(pi, 0x938, rfoverride4_old);
	write_phy_reg(pi, 0x939, rfoverride4val_old);
	write_phy_reg(pi, 0x43b, afectrlovr_old);
	write_phy_reg(pi, 0x43c, afectrlovrval_old);
	write_phy_reg(pi, 0x6da, old_sslpnCalibClkEnCtrl);
	write_phy_reg(pi, 0x6db, old_sslpnRxFeClkEnCtrl);

	wlc_lcnphy_clear_trsw_override(pi);

	mod_phy_reg(pi, 0x44c, (0x1 << 2), 0 << 2);

	for (i = 0; i < 11; i++)
		write_radio_reg(pi, rxiq_cal_rf_reg[i],
				values_to_save[i]);

	if (tx_gain_override_old)
		wlc_lcnphy_set_tx_pwr_by_index(pi, tx_gain_index_old);
	else
		wlc_lcnphy_disable_tx_gain_override(pi);

	wlc_lcnphy_set_tx_pwr_ctrl(pi, tx_pwr_ctrl);
	wlc_lcnphy_rx_gain_override_enable(pi, false);

cal_done:
	kfree(ptr);
	return result;
}

s8 wlc_lcnphy_get_current_tx_pwr_idx(struct brcms_phy *pi)
{
	s8 index;
	struct brcms_phy_lcnphy *pi_lcn = pi->u.pi_lcnphy;

	if (txpwrctrl_off(pi))
		index = pi_lcn->lcnphy_current_index;
	else if (wlc_lcnphy_tssi_based_pwr_ctrl_enabled(pi))
		index =	(s8) (wlc_lcnphy_get_current_tx_pwr_idx_if_pwrctrl_on(
			      pi) / 2);
	else
		index = pi_lcn->lcnphy_current_index;
	return index;
}

void wlc_lcnphy_crsuprs(struct brcms_phy *pi, int channel)
{
	u16 afectrlovr, afectrlovrval;
	afectrlovr = read_phy_reg(pi, 0x43b);
	afectrlovrval = read_phy_reg(pi, 0x43c);
	if (channel != 0) {
		mod_phy_reg(pi, 0x43b, (0x1 << 1), (1) << 1);

		mod_phy_reg(pi, 0x43c, (0x1 << 1), (0) << 1);

		mod_phy_reg(pi, 0x43b, (0x1 << 4), (1) << 4);

		mod_phy_reg(pi, 0x43c, (0x1 << 6), (0) << 6);

		write_phy_reg(pi, 0x44b, 0xffff);
		wlc_lcnphy_tx_pu(pi, 1);

		mod_phy_reg(pi, 0x634, (0xff << 8), (0) << 8);

		or_phy_reg(pi, 0x6da, 0x0080);

		or_phy_reg(pi, 0x00a, 0x228);
	} else {
		and_phy_reg(pi, 0x00a, ~(0x228));

		and_phy_reg(pi, 0x6da, 0xFF7F);
		write_phy_reg(pi, 0x43b, afectrlovr);
		write_phy_reg(pi, 0x43c, afectrlovrval);
	}
}

static void wlc_lcnphy_toggle_afe_pwdn(struct brcms_phy *pi)
{
	u16 save_AfeCtrlOvrVal, save_AfeCtrlOvr;

	save_AfeCtrlOvrVal = read_phy_reg(pi, 0x43c);
	save_AfeCtrlOvr = read_phy_reg(pi, 0x43b);

	write_phy_reg(pi, 0x43c, save_AfeCtrlOvrVal | 0x1);
	write_phy_reg(pi, 0x43b, save_AfeCtrlOvr | 0x1);

	write_phy_reg(pi, 0x43c, save_AfeCtrlOvrVal & 0xfffe);
	write_phy_reg(pi, 0x43b, save_AfeCtrlOvr & 0xfffe);

	write_phy_reg(pi, 0x43c, save_AfeCtrlOvrVal);
	write_phy_reg(pi, 0x43b, save_AfeCtrlOvr);
}

static void
wlc_lcnphy_txrx_spur_avoidance_mode(struct brcms_phy *pi, bool enable)
{
	if (enable) {
		write_phy_reg(pi, 0x942, 0x7);
		write_phy_reg(pi, 0x93b, ((1 << 13) + 23));
		write_phy_reg(pi, 0x93c, ((1 << 13) + 1989));

		write_phy_reg(pi, 0x44a, 0x084);
		write_phy_reg(pi, 0x44a, 0x080);
		write_phy_reg(pi, 0x6dx3f, 0<< 2), 0	writrx_iq_comp(
		write_phRneg(pi, 0x44a, 0x084<< 6);

		write_phy_reg(t_tx_gain(struct brcms_phy *pi, structHo3b, sex;
	ste_phy_te_phy_r70i, 0x43c, _est ist, 0x0B,Ce_phy_reg(pi, 0xxfffe);
)tim,cms_phy  save_AfeCtrlOvr);
}

statici);
HANNEL(p_tweakwlc_lcnphy_crsuprs(strucrcmsHANNEL(pict br8rcms_phyphybw40;
	==
			  C_IS40(pi->pi)
{
	s8 index;
	struct brcms_phy_lcnphy *pi_lcn = pi->u.cms_phyph=x1 << 4), ebit << 4);
		mod8phy_reg(pi 0x632, (0xff <	      p), ebit << 4);
		mod8phy_reg(pi 0x631, (0xff << 
	else
		index ba, edgelcnr;
	s2;pi->u.cms_phyph=x1<< 4
	else
		index ba, edgelcnr;
	s4= pi->u.cms_phyph=x1 || cms_phyph=x2 || cms_phyph=x3 || rfovecms_phyph=x4 || cms_phyph=x9 || rfovecms_phyph=x10 || cms_phyph=x11 || cms_phyph=x1;
	if (bcma
HAipco_pllfset;
(&lcnpd11cnre->bux439rv_0, 0},			       u16DIO_000c0g(pi, bcma
HAipco_pllfmaski);(&lcnpd11cnre->bux439rv_0, 0},3				   ~	andffffff);

		writbcma
HAipco_pllfset;
(&lcnpd11cnre->bux439rv_0, 0},4		       u16DI0,
	05c(pi, 0xbcma
Hcci);32(&lcnpd11cnre->bux439rv_0, 0BCMA_CC_PMU_CTL_ptr, u32 tBCMA_CC_PMU_CTL_PLL_UPD structHo3b, sex;
	ste_phy4 RADIO_2	;
}

static void
wlc_lcnphy_txrx_spu_override_en 4
	else
		index 
wlcmo_gain_overrip), ebit << 4);
		mo2_phy_reg(pi, 0x634x1b, (0xff << 80x44a, 0x084);
		writ0, 0, 59O_2064		return falcma
HAipco_pllfset;
(&lcnpd11cnre->bux439rv_0, 0},			       u16DIO_140c0g(pi, bcma
HAipco_pllfmaski);(&lcnpd11cnre->bux439rv_0, 0},3				   ~	andffffff);

333333	writbcma
HAipco_pllfset;
(&lcnpd11cnre->bux439rv_0, 0},4		       u16DI0,2c282(pi, 0xbcma
Hcci);32(&lcnpd11cnre->bux439rv_0, 0BCMA_CC_PMU_CTL_ptr, u32 tBCMA_CC_PMU_CTL_PLL_UPD structHo3b, sex;
	ste_phy4 RADIO_2	;
}

static void
wlc_lcnphy_txrx_spu_override_en 4
	else
		index 
wlcmo_gain_overrip), ebit << 4);
		mo2_phy_reg(pi, 0x634x1f, (0xff << 80x44a, 0x084);
		writ0, 0, 59Oa)phy_cur1 << 0) : 0));

	ora;

	orFOverrideVal0_old);
	writea;

	eg(pi< 6), bit << 6);
}

static 0)

#defincms_phytrl;
	4313lc_lcnphy_crsuprs(strucr8rcms_phy *pi,  *iqcOveare1;
};

static const struct chan_*ccOvelna2fpllfdouphyrtx_gain resllfnt_ remsllfnt_ r_ovave[18 prFxt savqFrefavqFvcoavqFca;
	u16 da1)
d, RAf, RAe 183e45;sh_l,  u32ion)
 u32fra, 0fvcorx_fpfd,	uinfrx_fble(piv; brcmsloop_bw)
 8, 3i);Cay(1 << 16 h1841h28_ten83e8, 3h30_ten83cp = pi_lc; brcmsg8, 3d28 << c 0 <&info_2064_lcnphy chan_0]hy_repllfdouphyrtx_1trsw_oveA, 0xd3);
	write_radio_reg(pi,D 0},4	 _reg(pi, x1 << 1), 0 << 1);

	write_radio_reg(p0 0x3, fphy_reg(!repllfdouphyr
	if (loop_bwtx_PLL_io_reLOOP_BWrrip 8,tx_PLL_io_reD30064		return faloop_bwtx_PLL_io_reLOOP_BW_DOUBLERrrip 8,tx_PLL_io_reD30_DOUBLERrri5), ebit << 5);
	}

	if (CHSPEC_IS2G(pi->radi< 2), 0 << 2);

ARRAY_SIZE(info_2064_lcnphy chan)or (i = 0i->u.cms__2064_lcnphy chan_i(iqcomp[ircms_phy *		result = 2) (ii iin =ARRAY_SIZE(info_2064_lcnphy chan);
		if (tem= 2) c 0 <&info_2064_lcnphy chan_i]phy_currREG114, 0x01);
	write_radio_reg(pi,A83cif 
	uint freq;
)trsw_oveA, 0xd3);
	write_radio_reg(pi0, 0},
83cif 
	uintftune;
)trsw_oveA, 0xd3);
	write_radio_reg(pi0x18, 
83cif gen_rccr_tx;
	u8 t)trsw_oveA, 0xd3);
	write_radio_reg(pi0 0x0, 83cif x_tune_ctrl;
	u)trsw_oveA, 0xd3);
	write_radio_reg(pi0, 0},
g(pi,witch, inu.cif 
	uintftunerx1, (0x1 << 2), ) \
	(0 != (read_radio_reg(pi,E0x0, 83cif x_t_rccr_rx;
	u8 pa_rx1 << 2), ) \
	(0 != (read_radio_reg(pi,E0x() |
		  4witch, inu.cif 1_freq_tune;
	u8 pa_rxb, (0x1 << _REG12C, 0x07);
	write_radio_reg(pi,C83cif 2_freq_tune;
	u8f << 
llfnt_ r& 0x38 << 16)C, 0x07);
	write_radio_reg(piorFOvesllfnt_ r_ova& 0x38 << 16)C, 0x07);
	write_radio_reg(p12B)hy_reg(C, 0x07);
	write_radio_reg(piorRADIO_206_reg(C, 0x07);
	write_radio_reg(p0, 0,(DIO_239, (0x1 e 1tx_gaine45tx_gai
	fpfd
	gaipllfdouphyrt? 
	if xt s	u8 << 0), (t
	if xt s	u8 phy_reg(	if xt s	u8 <> 26000000;
		e 1tx_1hy_reg(	if xt s	u8 <> 52000000;
		e 5tx_1hy_reg(e 1txsh);
		fble(pivtx_1hy__current_e 5txsh);
		fble(pivtx_2 <	      pfble(pivtx_4witcvcor / 2;if 	u8 <* (0x1 uinfrtx_2<* fpfd + iqFxt sinue;

				resc u32
wlc_lcn	if xt s	u8 ,_PLL_io_reMHZ4, 00x1 qFrefinue;

				resc u32
wlc_lcnfpfd,	PLL_io_reMHZ4, 00x1 qFc sinu	if xt s	u8 <*_fble(piv /	PLL_io_reMHZx1 qFvcoinue;

				resc u32
wlc_lcnfvcorx_24, 00x1< _REG12C, 0x07);
	write_radio_reg(pi0, 0, 0x1 << dq2 & 0	if xt s	u8 <*_fble(piv *x4 / 5) /	PLL_io_reMHZ)
		064_REG114, 0x01);
	write_radio_reg(pi520,(DIO_hy_rdq2 >>i->p2064_REG114, 0x01);
	write_radio_reg(pi 0x5rdq2 2) |
		   51 << dq6se;
	Fc si* 8 /	rdq2 
			))
		064_REG114, 0x01);
	write_radio_reg(pi51)
d, 0x1 <<q6se;
(dq6s
			k*	rdq2 
			))/vqFca;
	ui);Cay(1gains_tox3 * 2;if 	u8 ))/vl, 
		0642), ) \
	(0 != (read_radio_reg(pi,30,(DIOg(pi, 0xitch, inu.38 <(i);Cay(1g_gain)hy_reg(C, 0x07);
	write_radio_reg(pi 0x5, 0_2064_REG02E, 0x10);
	write_radio_reg(p0 205.38 <(i);Cay(1g save_n)hy_r u32ionse;
(cvcor * 2PLL_io_reMHZ))(((v))/vuinfrb, (0xhy_r u32fra,se;
(cvcor * 2PLL_io_reMHZ))(((v))%vuinfrb, (0xhy(module  u32fra,sn =uinfrb,radi u32ionup) {
 u32fra,s- =uinfrst ist u32fra,se;e;

				resc u32
wlc_lcn u32fra, 0finfrx_201 << 2), ) \
	(0 != (read_radio_reg(pid_phy_r1g(pi, 0xitch, inu.38 <( u32ions)(((v)0642), ) \
	(0 != (read_radio_reg(pid_phy_r1g(pi,40xitch, inu.38 <( u32ionspi,40)0642), ) \
	(0 != (read_radio_reg(pid_phy_rOg(pi, 0xitch, inu.38 <( u32fra,sn>, 002064_REG02E, 0x10);
	write_radio_reg(p0d_phy38 <( u32fra,sn>,in = (rfg2064_REG02E, 0x10);
	write_radio_reg(p0d8phy38 < u32fra,s= (rfg20664_REG02E, 0x10);
	write_radio_reg(p0d, 0},fb20664_REG02E, 0x10);
	write_radio_reg(p0d, 0, 0A2064_REG02E, 0x10);
	write_radio_reg(p0d6);

A_2064_REG098, 0x03);
	write_radio_reg(pi0x7a, 0C20664h18se;LCN_BW_LMT)/vloop_bw064d28se;
(2PLL_io_reHIGH_END_KVCO 
	PLL_io_reLOW_END_KVCO	k*itc(cvcor / 2 
	PLL_io_reLOW_END_VCO	))/
n, tia_g2PLL_io_reHIGH_END_VCO 
	PLL_io_reLOW_END_VCO	)
n, tia_+	PLL_io_reLOW_END_KVCO;64h18_ten
	gai28s*) b) / LCN_VCO_DIV <	 8,tx_(d
		a LCN_OFFSET) / LCN_FACT <	g8,tx_LCN_OFFSET (30 8,t* LCN_FACT2064h30_tentx_(g8,t*  b) / LCN_CUR_DIV <	cp = pi_lcse;
(LCN_CUR_LMT)* h18t* LCN_MULTt*  bb) / h18_ten) / h30_ten0642), ) \
	(0 != (read_radio_reg(pi0xcf, 
	{0cp = pi_lcvalues_tocms_phyp>=x1 && cms_phyp<= 51= 0; i < 11; i++)
		wriad_radio_reg(pi0xcf, ff <	      p; i < 11; i++)
		wriad_radio_reg(pi0xcf, _2064_REG12C, 0x07);
	write_radio_reg(pi0x1a, 31 << 2), ) \
	(0 != (read_radio_reg(pidrRADIOcRADIOc2064o cleanu_gain_oveio_revco
wlc_phy_stop_tx_t) \
	(0 != (read_radio_reg(pidrRAsllfnt_ r2064_REG12C, 0x07);
	write_radio_reg(p0, 0,sllfnt_ r_ovaphy_reg(a1 << 1IS(

	if (LCNREV_LT(pi1)le)
{
	if (eA, 0xd3);
	write_radio_reg(pi0 0x3 structHo3bA, 0xd3);
	write_radio_reg(pi0x187)phy_curreg(!, 0xxfffeboardflagss= BFL_FEM)le)
{
G_FILTERS_OFDM8<< gi0 [14G_FILT_		0xdRADI 0},
d0},
d0},
d0},
cRADIqcomp[, 0, 0},
	{ 0, 0},
	{x] = {
	{0,0o st << 80x44a,4, 0x01);
	write_radio_reg(pi,A83, fphy_uctHo3bA, 0xd3);
	write_radio_reg(pi0x18, 31 <{
	if (eA, 0xd3);
	write_radio_reg(pi0 0x, 31 << 
	if (eA, 0xd3);
	write_radio_reg(pi0 0x< gi0 [cms_phyp
		]3c, afectrlovrvaion);
}

staticlot;
	x_iir_ u16asure_digital_power(struct brciHY_txd, es_t u16_s16 coeffes_t u16_i) / 2);-1hy_rlcsj << 1int ddr[G_FILT_	, 0x0xFE	, 0xexFE	, 0xfxFE	, 024xFE	, 025xFE	, 026xFE	, 020xFE	, 021xFE	, 027xFE	, 028xFE	, 029xFE	, 022xFE	, 023xFE	, 030xFE	, 031xFE	, 032
st << 1int ddrY_txdiG_FILT_	, 00fxFE	, 000xFE	, 001xFE	, 006xFE	, 007xFE	, 008xFE	, 002xFE	, 003xFE	, 009xFE	, 00axFE	, 00bxFE	, 004xFE	, 005xFE	, 0OcRFE	, 0OdRFE	, 0Oe
st << reg(!iHY_txd->radi< 2),j0 << 2j;

	if ((ioeffs_cck[LCNPHY_NUM_T 2j++hile (iqcom u16_s16 txshstatic const u16 LCNPHY_txdj][0]>= 0; bi u16_i) / 2); = kmaj;*		result = 	goto sto
(iqcom u16_i) / 2!);-1 >= 0; tia_gj0 << 2j;

	if ((ioeffDM]
	[LCNPHY_NU 2j++h*		rerrideVal0_old);
	w ddr[j]q1_gain, tia_static const u16 LCNPHY_tx1_gain, tia_[ u16_i) / ][jM_DIG1 <{
}64		return fa< 2),j0 << 2j;

	if ((ioeffs_cck[LCNPHY_NUM_TX 2j++hile (iqcom u16_s16 txshstatic const u16 LCNPHY_txdij][0]>= 0; bi u16_i) / 2); = kmaj;*		result = 	goto sto
(iqcom u16_i) / 2!);-1 >= 0; tia_gj0 << 2j;

	if ((ioeffDM]
	[LCNPHY_NU 2j++h*		rerrideVal0_old);
	w ddrY_txdij]q1_gain, tia_static const u16 LCNPHY_txd1_gain, tia_[ u16_i) / ][jM_DIG1 <{
}64	))
		returm u16_i) / 2!);-1 >? 0 :;-1hy
	return reint
		index =	(s8)pa_lcnphy_rx_iq_cal_gain(struct brcmspa_lcnp << 
 0xff;
	gai 0;
	rfgain1 = (read_phyitch, static coxff;eg(pi, 0x41)pa;
	wlc_r_0x41)MASK6) &itch,static coxff;eg(pi, 0x41)pa;
	wlc_r_0x41)SHIFTh))
		returpa_lcnp <ac_ctrl) << 0);

}

static void wlc_ln_by_distribution(struct brcmst brcms_phy *pi, structe wlc_l lcnphy_txgain
 0xff;
	g
		index =	(s8)pa_lcnphar_trsw_override(pin 4
		write5xFE	y_reg(pi, 0x48FE	y(e wlc_l lcnp0)) >> 0;,
		   (e wlc_l lcnp0) & 0xff;
<a5) & <<FE	,)0642), 	rfgain1 = (read_phy_reg(pi,7g(pi, 0x48FE	reg(p(e wlc_l lcnp0) & 0xff;(u16)p 0xff;
<a5) & << (0x1 << 0), 0 << 0n 4
		writfcxFE	y_reg(pi, 0x48FE	y(e wlc_l lcnp0)) >> 0;,
		   (e wlc_l lcnp0) & 0xff;
<a5) & <<FE	,)0642), 	rfgain1 = (readdphy_reg(pi,7g(pi, 0x48FE	reg(p(e wlc_l lcnp0) & 0xff;(u16)p 0xff;
<a5) & << (0x1 <
}


static void wlc_lcn_over wlc_l lcnp0)cms_phy *x1 <
}


staticms_phylc_lcnphy_disable_tx_ga
	return return result;
}

sbbmulc_lcnphy_rx_iq_est(strhy_txgainm0m1;32 tbl_offset)
{

	struct .tbl_id = tbl_id&m0m1;32_ptr = tbl_ptr1;32_ptr = tfo ta_tx_pwr_BL_ID)

#deAL;idth = tbl_width;
	87;bl_len = tbl_len;
16;set = tbl_offset;
	wlc_lcnphy_read_)
		returm38 <((m0m1s= (rfg0 (dac_8(pi< 6), bit << 6;

}

static voibbmulc_lcnphy_rx_iq_est(strucr8rm0hy_txgainm0m1& 0x3ff);my_r708;32 tbl_offset)
{

	struct .tbl_id = tbl_id&m0m1;32_ptr = tbl_ptr1;32_ptr = tfo ta_tx_pwr_BL_ID)

#deAL;idth = tbl_width;
	87;bl_len = tbl_len;
16;set = tbl_offset;
	wlc_lcnphy_write_table(pi, < 0));
}

static void wxy_measl_widthwlc_lcnphy_crsuprs(strin, u16 ldatat fr[64];32 tbl_offset)
{

	struct .tmemi);(datat fr,ibut

	ptr datat frn)hy_r_ptr = tfo ta_tx_pwr_BL_ID))

#dCTL;bl_len = tbl_len;
32;.tbl_id = tbl_iddatat frest = { 0, 0, 0 };

 0x20))

#(wlc_lcnphy_tssi_based_pwr_81) & _ptr = tbl_ptr30064dth = tbl_width;
	_tx_pwr_ctrl(pi, LCRATE_OFFSETO_2	;
}

staticset;
	wlc_lcnphy_write_4	))
_ptr = tbl_ptr64;idth = tbl_width;
	_tx_pwr_ctrl(pi, LCMAC_OFFSETO_2t = tbl_offset;
	wlc_lcnphy_write_tabenums_phy *pi if x_sp>= 0_tx_pwr_SSI_PRE_PA, 0_tx_pwr_SSI_POST_PA, 0_tx_pwr_SSI_EXT128, _AfeCtrlOvr);
}

statici);
i if xucurrent_tx_pwr_idx(str,cmsums_phy *pi if x_sp>pos*pi, u16 a, u16 b)
{
	m4d7phy_reg(pi, 0x6ave_fa, (0x1 << 0), 0 << 0);

	mod7phy_reg(pi, 0x64b, (0x1 << reg(a1 _pwr_SSI_POST_PATOMICos*if (channel != 0) {
		mod_phy_reg(pi, 0x6d1, (0x1 << channel != 0) {
		mod_phy_reg(pi, 0x64b, (0x1 << _reg(a1 << 1IS(

	if (LCNREV_LT(pi->pubpi. 2), ) \
	(0 != (read_radio_reg(pi86 0},4	 _rg(pi,  << 1) + rbi2), ) \
	(0 != (read_radio_reg(pi0A 384,_AfeCtrbi2), ) \
	(0 != (read_radio_reg(p0, 0,  0, 0},eCtrbi2), ) \
	(0 != (read_radio_reg(p0 748,
1);

		writi2), ) \
	(0 != (read_radio_reg(p0, 0,  064_<<2eCtrbi2), ) \
	(0 != (read_radio_reg(p036x5, 0_);

		writi2), ) \
	(0 != (read_radio_reg(p0, 0,  0_);_<<4eCtrbi2), ) \
	(0 != (read_radio_reg(p036x5, 3);

		writi2), ) \
	(0 != (read_radio_reg(p03, 0},
f);

77eCtrbi2), ) \
	(0 != (read_radio_reg(p0 748,
1 0},
e<<feCtrbi2), ) \
	(0 != (read_radio_reg(p0, 0, 0, 0}_<<7eCtrbi2), ) \
	(0 != (read_radio_reg(p0x1f, x70}_<<feCtrbi2), ) \
	(0 != (read_radio_reg(p02, 0, 0_);
<<4eCtrb}, ext_lna << 10);
	} else {
		mod_phy_reg(pi, 0x6dve_fa, x1 << channel != 0) {
		mod_phy_reg(pi, 0x60b, (0x1 << _reg(a1 << 1IS(

	if (LCNREV_LT(pi->pubpi. 2), ) \
	(0 != (read_radio_reg(pi86 0},4	 _rg(pi,  << 1) + rbi2), ) \
	(0 != (read_radio_reg(pi0A 384,_eCtrbi2), ) \
	(0 != (read_radio_reg(p0, 0,  0, 0},eCtrb}64	)_tx_pu(pi, 1);

		mod7phy_reg(pi,4 0x60b, (01 << 14reg(a1 _pwr_SSI_EXTTOMICos*if (c	if (eA, 0xd3);
	write_radio_reg(pi7Fc_lcnpbi2), ) \
	(0 != (read_radio_reg(p0x1f, x70}0x2cnpbi2), ) \
	(0 != (read_radio_reg(p0, 0, 0, 0}_reg(pi, npbi2), ) \
	(0 != (read_radio_reg(p0 748,
1f18, 31 <{}y
	return reint
		index =	ridtq_ tbladcfnt_ r_lcnphy_rx_iq_est(strhy_txgainN84,N 0,N3);N4	 N1f,N6 0N <{N1& 0x_pwr_npt(pi) \
	((read_phy_i, 0x4a7) & 
n, tia_ & (0x3fN2n;
1 else_pwr_npt(pi) \
	((read_phy_i, 0x4a512);
		ia_ &  << 12N3& 0x_pwr_npt(pi) \
	((rea0dphy_i, 0x4a7) & 
n, tia_ & (0x3fN4n;
1 else_pwr_npt(pi) \
	((rea0dphy_i, 0x4a5) &
		ia_ & ,eCtrN5& 0x_pwr_npt(pi) \
	((read2phy_i, 0x4a7) & 
n, tia_ & (0x3fN6n;
1 else_pwr_npt(pi) \
	((read2phy_i, 0x4a5) &
		ia_ & ,eCtrNtx_2<* (Nsh =N2n =N3n =N4y *p<* (N2 
	N6<< (38qq_nbitsN;

	600;
		Nn;
160q_est))
		rNe_table(pi, < 0));
}

staticnt_tx_pwr ifparamwlc_lcnphy_crsuprs(strin, u1int ux & 0vmild, ux & 0vmil
 0x2d, ux & 0cnphy 0 };
	s;
	u32 ii, qq;
	struct brcms_phy_lcnphy *pi_lcn = pi-ux & 0vmil& 0x2x4a5) 
		    (0  0	ielse
		index r if_vcspi,40u16	ielse
		index r if_v16_1 ux & 0vmil
 0x2& 0x2x4a5) 
	 (8spi,40u1646_1 ux & 0cnphy 0 }tx_2 < << 0), 0 << 0);

	mod8phy_reg(pi, 0x6a_fa, (0x1 << 0), 0 << 0);

	mod_phy_reg(pi, 0x43c, (0x1 << < 0), 0 << 0);

	mod7phy_reg(pi, 0x410, (0x1 << 3), (0) << 3);

	mod_phy_reg(pi,(pi, 0x64
		    (0x1 0x4a512)phy_reg(p-ux & 0vmil& 0x64
	 0	ielse
		index r if_gsx4a512); << 3), (0) << 3);

	modcphy_reg(pi,(pi, 0x64
		    (0x1 0x4a512)phy_reg(p-ux & 0vmil& 0x64
	 0	ielse
		index r if_gsx4a512); << 3), (0) << 3);

	mo0axFE	reg(pi,(pi, 0x64
		    (0x1 0x4a512)phy_reg(p-ux & 0vmil& 0x64
	 0	ielse
		index r if_gsx4a512); << 3), (0) << 3);

	mo0_phy_reg(pi,(pi, 0x64
		    (0x1 0x4a512)phy_reg(p-ux & 0vmily 0 }t 0x64
	 0 ux & 0cnphy 0 }t4a512); << 3), (0) << 3);

	mo0cphy_reg(pi,(pi, 0x64
		    (0x1 0x4a512)phy_reg(p-ux & 0vmily 0 }t 0x64
	 0 ux & 0cnphy 0 }t4a512); << 3), ) \
	(0 != (read_radio_reg(pi820,(eg(pi, 0x6dg(pi, )0642), ) \
	(0 != (read_radio_reg(pi7C0x6dg(pi0)0x6dg(pi0)te_table(pi, < 0));
}

statici if i); r_lcnphy_rx_iq_est(strhy_tx tbl_offset)
{

	struct p16 lridtquct  afemsums_phy *pi if x_sp>x_sp;	u16 t if i)l< 14reg( 0xxfffeboardflagss= BFL_FEM)if (ct if i)l0 <<x1npbi2),e;
	_tx_pwr_SSI_EXT;, ext_lna << t if i)l0 <<xenpbi2),e;
	_tx_pwr_SSI_POST_PAst ist_ptr = tfo ta_tx_pwr_BL_ID))

#dCTL;bl_len = tbl_len;
32;.tbl_id = tbl_id&t  afe_ptr = tbl_ptr1;32_ptr = t_width;
	q_es< 2), nd0 << 2)nd0a5128 2)nd++hile (;
}

staticset;
	wlc_lcnphy_write_42_ptr = t_width00);
		2_ptr = t_width;
	704_es< 2), nd0 << 2)nd0a5128 2)nd++hile (;
}

staticset;
	wlc_lcnphy_write_42_ptr = t_width00);
		23), (0) << 3);

	m503phy_reg(pi, 0x6a_fa, (0x1 << 0), 0 << 0);

	m503phy_reg(pi, 0x6d1, (0x1 << < 0), 0 << 0);

	m503phy_reg(pi, 0x43b, (0x1 << ;
}

statici);
i if xucu);

2),e)0642), 	rfgain1 = (reaa4phy_reg(pi,4 0x60b, (01 << 142), 	rfgain1 = (reaa4phy_reg(pi,, 0x6da, (01(0x1 << 5), 0 << 5);

	mod_phy_reg(pi, 0x6da, (0x1 << 2), 	rfgain1 = (reaa4phy_reeg(pi, 0x48a_fa, (0x1 << 0), 0 << 0);

	moa5phy_reg(pi, 0x48l) ((a, (0x1 << 0), 0 << 0);

	moa5phy_r0x4a512)p (515 = ((bx1 << 0), 0 << 0);

	moa5phy_r0x4a5, 0x481, (0x1 << 8), (0) << 8);

	mo0dphy_reg(pi, 0x4864((a, (0x1 << 0), 0 << 0);

	mo0dphy_r0x4a5, 0x441, (0x1 << 8), (0) << 8);

	moa2phy_reg(pi, 0x4864((a, (0x1 << 0), 0 << 0);

	moa2phy_r0x4a5, 0x441, (0x1 << 8), (0) << 8);

	mod_phy_reeg(pi,, 0x43c, (0x1 << < 0), 0 << 0);

	moa8phy_reg(pi, 0x48ave_fa, (0x1 <;
}

static void wxy_measl_widthwlar_trsw_override(pi);

	moa6phy_reg(pi,, 0x6da, (01(0x1 << 5), 0 << 5);

	moa6phy_reeg(pi, 0x48avff((a, (0x1 << 0), 0 << 0);

	mo9aphy_reeg(pi, 0x48avff((a, (0x1 <reg(a1 << 1IS(

	if (LCNREV_LT(pi->pubpi.2), ) \
	(0 != (read_radio_reg(p0 748,
f18t if i)l npbi2), ) \
	(0 != (read_radio_reg(p086 0},4	 _rg(pi,ext_lna << 10);) \
	(0 != (read_radio_reg(p0 748,
1 0}t if i)l09, (0x1 i2), ) \
	(0 != (read_radio_reg(pi0A 3,
1);(0x1 i2), ) \
	(0 != (read_radio_reg(p0, 0,  0, 939, (0x1 _currREG114, 0x01);
	write_radio_reg(pi,1f, xc0x1 <reg(a1 << 1IS(

	if (LCNREV_LT(pi->pubpi.2), ) \
	(0 != (read_radio_reg(p00A 3,
1);(0x1 ext_lna << ebit << 5);
	}

	if (CHSPEC_IS2G(pi- rbi2), ) \
	(0 != (read_radio_reg(pi0A 3{x] =939, (0x1  (coeff_x2), ) \
	(0 != (read_radio_reg(pi0A 3{x] =43c, (0x1y_curreg(a1 << 1IS(

	if (LCNREV_LT(pi->ppi.2), ) \
	(0 != (read_radio_reg(p00A 3,
] =939, (0x1       p), e) \
	(0 != (read_radio_reg(p00A 3,
40x939, (0x1064_REG00B, 0x7);
	mod_radio_reg(pi,A 3,
1);((a, (0x1 << 0)) \
	(0 != (read_radio_reg(p0x1f, x0, 939, (0x1t = { 0, 0, 0 };

 0x20))

#(wlc_lcnphy_tssi_based_pwr_< channel != 0) {
		mod7_ptr, u32y_reg(pi, u16) ~0x4a512)p 43c, 3u1625 = ((bx1 <ridtq-) {
				set_gaidtq_ tbladcfnt_ r_overri_ptr = tfo ta_tx_pwr_BL_ID)RFSEQ;bl_len = tbl_len;
16;sebl_id = tbl_id&aidtqafe_ptr = tbl_ptr1;32_ptr = t_width;
	6;set = tbl_offset;
	wlc_lcnphy_write_1 << 1), 1 << 1);
	mod_phy_reg(pi, 0x(11, (0x1 << < 0), 0 << 0);

	mod_phy_reg(pi, 0x(11, (0x1 << < 0), 0 << 0);

	maa4phy_reg(pi,, 0x(11, (0((bx1 << 0), 0 << 0);

	mod7phy_reg(pi, 0x(11, (0x1 << < 0), 0 << 0);

	mad_phy_reg(pi, 0x481, (0x1 << 8), ) \
	(0 != (read_radio_reg(p03, 0},
f);

,)0642), ) \
	(0 != (read_radio_reg(p036x5, 3);

		wri2), ) \
	(0 != (read_radio_reg(p0, 0,  0, 0},eCtset = tbl_offnt_tx_pwr ifparamwltx_ga
	r< 0));
}

staticiphy_geupda;
	npc_lcnphy_rx_iq_est(strhy_txgainiphcon)
	wlc_t savnpc;
	s;
	u32 ii, qq;
	struct brcms_phy_lcnphy *pi_lcn = pi	wlc_t sgain_override_o_t sciphframes_overri_phcongai	wlc_t sg-6	ielse
		index t if _phconrrinpc
	g
		index =	(s8)iphy_genpc_ar_trsws_to_sac{

		if, (0npcr_81) & 	ielse
		index t if _phcongai	wlc_t s_en 4
	else
		index t if idx
	g
		index =	(s8)8 wlc_lcnphy_get_cuoverri4
	else
		index t if npc
	gnpc;
<{}y
	result;
}

statit if2dbmb -=}t if,  = { 1,  = {e_ph = {eu16 ln = { 0, 0,lt)
	an;
32768 (30a1 *}t iferrip;
	bc_rx *}b0< (306x *}b1 *}t iferri2& 0x(p<* b< (3a) / (p<* aad_)
		returpe_table(pi, < 0));
}

staticix_measlrei);
npc_lcnphy_rx_iq_est(strhy_txs;
	u32 ii, qq;
	struct brcms_phy_lcnphy *pi_lcn = 	ent_index;
	else0x20))

#(wlc_lcnphy_tssi_based_pwr_< cf (tem= 2)
	else
		index t if idx
	g_tx_pwr_ctrl(pi, LCSTART_INDEX_2G	4313;2)
	else
		index t if npc
	g_tx_pwr_ctrl(pi, LCSTART_NPTga
	r< 0));
}

staticip_measlreult =r wlc__lcnphy_rx_iq_est(strhy_tx tbl_offset)
{

	struct p16 lra;
	wlc_l[BRCMSioeffRATENUM_T (3BRCMSioeffRATENUM_TX +	    (0  3BRCMSioeffRATENUMCS_1CSTREAM];32  *iqc,aj;*	ent_index;
	else0x20))

#(wlc_lcnphy_tssi_based_pwr_< cf (tem= 2)< 2), 0 <<,aj0 << 2);

ARRAY_SIZE(ra;
	wlc_l)or (i,2j++hile
) (ii ii==3BRCMSioeffRATENUM_T (3BRCMSioeffRATENUM_TX- rbij0 <TXP_FIRSTUMCS_20_SISO_en 4ra;
	wlc_l[		va83) << (bits));-pif gey_measl_width[j])te_4	))
_ptr = tfo ta_tx_pwr_BL_ID))

#dCTL;bl_len = tbl_len;
32;.tbl_id = bl_ptrARRAY_SIZE(ra;
	wlc_l)osebl_id = tbl_idra;
	wlc_l;idth = tbl_width;
	_tx_pwr_ctrl(pi, LCRATE_OFFSETO_2t = tbl_offset;
	wlc_lcnphy_write_1 ent_index;
	els(s8)i wlc_lnphy_g_pwr2!);pif gey_measlmf;(ule (;
}

staticss8)i wlc_lnphy_g_pw,;pif gey_measlmf;( << 
	
}

staticix_measlrei);
npc_overriafectrlovrval);
	}
}

statici);

	wlc_lsoftlcnphyrrent_tx_pwr_idx(str,crcms_phyin, u16 lcckl_width[4G_FIL 2] =2] =2] =2]q_est16 l_txdl_width0x< gl_widthY_tx;y_rlcsi;ride_ol_phy2;32 tbl_offset)
{

	struct .tent_index;
	else if (wlc_lcnphy_tssi_based_pwr_ctrf (tem= 2)2), 	rfgain1 = (reaa4phy_reg(pi,4 0x60ve_fa, 1 << 142), 	rfgain1 = (reaa4phy_reg(pi,4 0x60v0b, (01 << 14), (0) << 8);

		or_phy_re4x_iq_comgl_widthY_tx;
	q_es< 2), 0 << 2);

4or (i = 0cckl_width[		v-=x< gl_widthY_tx;y__ptr = tfo ta_tx_pwr_BL_ID))

#dCTL;bl_len = tbl_len;
32;.tbl_id = bl_ptr4osebl_id = tbl_idcckl_width;idth = tbl_width;
	_tx_pwr_ctrl(pi, LCRATE_OFFSETO_2t = tbl_offset;
	wlc_lcnphy_write_	_txdl_width;
	q_es_ptr = tbl_ptr1;32_ptr = ttbl_id&_txdl_width_es< 2), 0 <836 2);

862or (i a << th = tbl_width;
	i;ri	;
}

staticset;
	wlc_lcnphy_write_4	))
2), 	rfgain1 = (reaa4phy_reg(pi,, 0x60ve_fa, 1x1 << 2), 	rfgain1 = (reaa4phy_reg(pi,4 0x60ve_fa, 1 << 142), 	rfgain1 = (reaa4phy_reg(pi,3 0x60ve_fa, 1x1 << 3), (0) << 3);

	mob_phy_reg(pi7 0x481, (071 << 3), (0) << 3);

	mo3_phy_reg(pi, 0x63c, (0x1 << < 0), 0 << 0);

	moa9phy_reg(pi,, 0x6da, (01(0x1 <l_phy2& 0x3ff);(i) / 2* (0x1 << 2), 1 << 2);
	moa9phy_reeg(pi, 0x48l_phy2b, (0x1 << 0), (1) << 0);

	moa3phy_reg(pi, 0x40b, (0x1 <<ectrlovrvaeturn result;
e0x2cnph))
a;
;
	xnt_tx_purrent_tx_pwr_idx(struct brcms_phy, deltat rd, deltat 0x2d,new_s_phy, e0x2cnrry *piainman2d,c u3t 0x2d, 0x2_dif16_1t brcneg ain_override_o 0 };
	s;
	u32 ii, qq;
	struct brcms_phy_lcnphy *pi_lcn = pient_index;
	else if (wlc_lcnphy_tssi_based_pwr_ctrf (tem;
	else
		index = pi_lcn->lcnph
pi) / 2);FIXED))

#d< 14reg( 0else
		index t0x20))

#slo6 txsh0_ctrf (tem;->lcnph
p 0x2& 0x, (0xfndex;
	else0x20))

0);

	0x1 < u3t 0x2;
	_tx_pwr_EMPSENSE << -)< 14reg( 0f gey_measlmf;3c);
	ctrdeltat rd & 0	ielse
		index < u3Pmeasg-6	if gey_measlmf;( <       pdeltat rd & gai
	man2;
	_tx_pwr_EMPSENSE 	ielse
		index rawe0x20))

)oseb0x2_dif1;
	man2;-,c u3t 0x2;*	ent_b0x2_dif1;<;
	if (cneg aib);
				b0x2_dif1;
	- 0x2_dif16_1	))
deltat 0x22); =8);e;

				resc u32
wlc_lcn3) << (b0x2_dif1;* 192)phy_t brcm3) << (	ielse
	
					   	index t0x20))

#slo6 
					   *  b),ADIO_2064_neg	ctrdeltat 0x22);-deltat 0x2< 14reg( 0else
		index t0x20))

#op_gaiph=x3
n, ti&& a1 << 1IS(

	if (LCNREV_LT(pi & 
nrdeltat 0x22);qq_nbits 0else
		index t0x2cnrry
		31<< 4t0x2cnrry
); =8);s 0else
		index t0x2cnrry
- 64( <       pt0x2cnrry
); =8); 0else
		index t0x2cnrry;urreg(a1 << 1IS(

	if (LCNREV_LT(pi1)<< 4t0x2cnrry
);4osenew_s_phyvalues_phyv+ deltat rdv+ deltat 0x22-6	ielse
		index ba, edgelcnr;osenew_s_phyv+= e0x2cnrry *urreg(a1 << 1IS(

	if (LCNREV_LT(pi1)<< 4i) / 2);127 *urreg(new_s_phyv<;
 || new_s_phyv&  <6_ctrf (tem;->lcnph
pf (tem;new_s_phyhy
	return reint
		index =	i);

	wlc_lcnphrx_spur_avoidance_mode(strucgainm),e)
le
)rcmsH pi_lcn2),e;
	x_sp;	uent_index;
	else0x20))

#(wlc_lcnphy_tssi_based_pwrpwr > q_t2),e;

	_tx_pwr_ctrl(pi, LCHW = 0c pi_lcn2),e;
	_tx_pwr_ctrl(pi, LC_EMPBASED;	uent_index;
	else if (wlc_lcnphy_tssi_based_pwrpwr > q_t2),e;

	_tx_pwr_ctrl(pi, LC_EMPBASED = 0c pi_lcn2),e;
	_tx_pwr_ctrl(pi, LCHW_est))
		rc pi_lcn2),ega
	r< 0));
}

statici);

	wlc_lcnphyr_avoidance_mode(strucgainm),e)
le
	bool se2),e;
	;
	tx_pwr_ctrl = wlc_lcnphy_get_trcms_phy *pi)
{
	s8 index;
	struct brcms_phy_lcnphy *pi_lcn = pi2),e;
	;
	tx_pwr_ci);

	wlc_lcnphrx_spu);

2),e)064l se2),e;
	;
	tx_pwr_ci);

	wlc_lcnphrx_spu);

l se2),e1 << 0), (1) << 0);

	modaphy_reg(pi, 0FE	reg(p(_tx_pwr_ctrl(pi, LCHW;

	2),e1 ? 1 : 3c, (0x1 << < 0), 0 << 0);

	moa3phy_reg(pi, 0FE	reg(p(_tx_pwr_ctrl(pi, LCHW;

	2),e1 ? 0 :;3b, (0x1 << ent_l se2),e;!
	2),e1  << ebit_tx_pwr_ctrl(pi, LCHW;

	l se2),e1ile
) 0x44b, 0xffff);
_geupda;
	npc_ar_trsw	<;
}

static void wxy_measl_widthwlar_trgoto sebit_tx_pwr_ctrl(pi, LCHW;

	2),e1ile
) 0x44b, 0xffff)_measlreult =r wlc__ar_trsw	<;
}

statici);
se;

	wlcy_get_cuov				     	ielse
	
					  	index t if idxeCtrbi;
	tx_pwr_ci);

	wlc_lnpc_ar,6	ielse
		index t if npcr;trbi2), ) \
	(0 != (read_radio_reg(p0,F 0},4	 __trsw	<	ielse
		index t if _phconga
				n_override_o_t sciphframes_overrtrbi;
	tx_pwr_clse
		wlc_lcnphy_disable_tx_gaw	<	ielse
		index txy_measlt_cu_disable2);-1hy_ ext_lntrbi;
	tx_pwr_cms_phylc_lcnphy_disable_tx_ga< channel != 0) {
		moa4_ptr, u32yy_reg(pi,, u16) ~eg(pi,4 u16) ~eg(pi,3))

2),e)064
		reture;

	_tx_pwr_ctrl(pi, LC_EMPBASED ile (iq) / 2);rn result;
e0x2cnph))
a;
;
	xnt_tx_putx_gaw	<}
}

statici);

	wlc_lsoftlcnphy {
	s_phyigaw	<	ielse
		index = pi_lcn->lcn
); =8)hy_t brcm (0  0_pwr_npt(pi) \
	(
					      u16DIoa9phyitc		    y_rel_on(
			b}64	)ave_AfeCtrlOvr);
}

static v_iqlo_loopbackyr_avoidance_mode(strucgain*l_rf_reg[i],
	)
le
	boovmil;y_rlcsi;ri< 2), 0 << 2);

20or (i = 0; i < 11; i++)
		values_to_save[i] =
			reiqlo_loopback_reg(pis			valuehannel != 0) {
		mod_phy_reg(pi,, 0xeg(pi,, x1 << 2), 1 << 2);
	mo4dphy_reg(pi,4 0x1fa, 1 << 142), 	rfgain1 = (read_phy_reg(pi,, 0x939, ((0x1 << 1), 1 << 1);
	mo4dphy_reg(pi,3)p 43c, 1x1 << 3), (0) << 3);

	mod_phy_reg(pi, 0x43b, (0x1 << 1), 1 << 1);
	mod_phy_reg(pi, 0x43c, (0x1 <3), (0) << 3);

	mod_phy_reg(pi, 0x43b, (0x1 << 0), 1 << 0);
	mod_phy_reg(pi, 0x43c, (0x1 <reg(a1 << 1IS(

	if (LCNREV_LT(pi->ppi.a, ~) \
	(0 != (read_radio_reg(p00A 3,
FD( <       pa, ~) \
	(0 != (read_radio_reg(p00A 3,
F_phy_eg(C, 0x07);
	write_radio_reg(p0,A 3,
1<< 14), ) \
	(0 != (read_radio_reg(p036x5, O_2064eg(C, 0x07);
	write_radio_reg(p0,A 3,
1,eCtro clean2(0x1 <reg(a1 << 1IS(

	if (LCNREV_LT(pi->pubpi.ebit << 5);
	5

	if (CHSPEC_IS2G(pi- rbi2), ) \
	(0 != (read_radio_reg(pi0A 384,_eCtrbt_lntrbi), ) \
	(0 != (read_radio_reg(p03A);(0x1 ext_lna << ebit << 5);
	5

	if (CHSPEC_IS2G(pi- rbi2), ) \
	(0 != (read_radio_reg(pi0A 33, (0x1  (coeff_x), ) \
	(0 != (read_radio_reg(p03A);, 31 <{}ytro clean2(0x1 <rREG114, 0x01);
	write_radio_reg(pi,1f, xFIO_2064_a1 << 1IS(

	if (LCNREV_LT(pi->pubpi.ebit << 5);
	5

	if (CHSPEC_IS2G(pi- rbi2), ) \
	(0 != (read_radio_reg(pi 748,
F	 _rg(pi, (coeff_x2), ) \
	(0 != (read_radio_reg(pi 748,
F	 _r60x1 ext_lna << ebit << 5);
	5

	if (CHSPEC_IS2G(pi- rbi2), ) \
	(0 != (read_radio_reg(pi 748,
1 0},
439, (0x1  (coeff_x2), ) \
	(0 != (read_radio_reg(pi 748,
1 0},
63c, (0x1y_curo clean2(0x1 <rREG114, 0x01);
	write_radio_reg(pix1f, x02064eg(C, 0x07);
	write_radio_reg(p0, 0, 0, eCtro clean2(0x1 <), ) \
	(0 != (read_radio_reg(p0FF	 _r0_2064eg(C, 0x07);
	write_radio_reg(p0,F	 _rg4eCtro clean2(0x1 <), ) \
	(0 != (read_radio_reg(p0, RADIO_206eg(C, 0x07);
	write_radio_reg(p0,0x5, 0_2064o clean2(0x1 <rREG114, 0x01);
	write_radio_reg(pix7 3,
1<< 4o clean2(0x1 <vmil& 00x2A60642), ) \
	(0 != (read_radio_reg(p0Fxcf, 
3c, (phyvmil&n>,in = (r_2064_REG098, 0x03);
	write_radio_reg(piFDphyvmil& save_n)hy4eg(C, 0x07);
	write_radio_reg(p0,F	 _rg4eCtro clean2(0x1 <), ) \
	(0 != (read_radio_reg(p0FF	 _r0_2064o clean2(0x11 << 4, 1 << 4);
	write_radio_reg(pi,] = {
22064eg(C, 0x07);
	write_radio_reg(p0, 0, 0O_2064_REG116, 0x06);
	write_radio_reg(p036x5, 0_2064_REG114, 0x01);
	write_radio_reg(pi5, 0, cc2064_REG114, 0x01);
	write_radio_reg(pi5xcf, 2e2064_REG114, 0x01);
	write_radio_reg(pi7748,
d_2064_REG12C, 0x07);
	write_radio_reg(pi9 0, 01(0x1ave_AfeCtrt brc;
}

staticiqble(waic_lcnphy_rx_iq_est(strhy_txgrlcs clea_cay(1gaigai
	module ;
}

staticiqble(active_tx_pubpi.o cleanu0_eCtrb clea_cay(100);<< ebit clea_cay(1g		if,t* 50 & 
nresult = 		))
		returm0;

	;
}

staticiqble(active_tx_p save_AfeCtrlOvr);
}

static v_iqlo_loopbackc voin r_lcnphy_rx_iq_est(strucgain*l_rf_reg[i],
	)
le
rlcsi;r
0a, ~(0x228));

		aad_ph 0O_ &  1<< 14a, ~(0x228));

		aa3,
	{ C20664< 2), 0 << 2);

20or (i = 0_REG12C, 0x07);
	wriiqlo_loopback_reg(pis			xiq_cal_rf_reg[i],
				valave_AfeCtrlOvr);
}

static v_iqlo_wlc__by_distribution(struct cm (0  t brcms_phy *pi, structe wlc_l lcnpuct cm (0  msums_phy *pble(ture;ble(tureuct brckee
			}
)
le
)t brcms_phy *pi, strucwlc_lcnpsd, 0x2_lcnpsrride_ohash;	u16 ba, lt_c;y_rlcsjrride_oncnrry_disable[5]rride_osystlcLCNPHiG_FILhy_re0_);

	e0_);

	e0_);

	e0_);

	e0_);

	e0_)
      u16DI	e0_);

	e0_);

	e0_);

	e0_);

	e0_t << 1intcnpma, s_fullwlciG_FILT_	, 8434	 _r8334	 _r8084	 _r8267	 _r8056	 _r8234
st << 1intcnpma, s_reultiG_FILT_	, 8434	 _r8334	 _r8084	 _r8267	 _r8056	 _r8234
st << 1intcnpma, _sums_fullwlciG_FILT_	, 7a97	 _r7a97	 _r7a97	 _r7a87	 _r7a87	 _r7b97
st << 1intcnpma, _sums_reultiG_FILT_	, 7a97	 _r7a97	 _r7a97	 _r7a87	 _r7a87	 _r7b97
st < 1int*cnpma, _sums_idcnpma, _sums_fullwlc << 1int*se;

	cLCNPH_idNULL,t*clc_cmdH_idNULL,tclc_s16 )
 uq
se;

;txgainiphlc_lcnphroldphy_reg	xnt_tx_prftx_p2rride_os_regg(pi, 0x6da, old_ss,os_regg(pi, 0x6db, old_s6_1t brc_save[i]);

	if (tx_;
)t brcms_phy *pi, strucget_tx_ga;32  *iqc,an_clc_cmdH_id0,an_clc_se;

2);qq_ngain*l_rf_reg[i],
	 *pi)
{
	s8 index;
	struct brcms_phy_lcnphy *pi_lcn = pil_rf_reg[i],
	hy_kmalloc(

	ptr , (0x*
20, GFP_ATOMICIO_2064_NULL;

	l_rf_reg[i],
	)
trf (tem= 2)s_regg(pi, 0x6db, old_s 	save_AfeCtrlOvr = re6dite_4s_regg(pi, 0x6da, old_ss 	save_AfeCtrlOvr = re6da<< 14), (0) << 8);

		or_phy_4_2064eg(feCtrlOvr = re6di);, 31 <_4sCe_ph (ble(turepubpicalna_tx_pwrCAL_FULL:
{
G_F

	cLCNPH_idsystlcLCNPHCtrbclc_cmdH_idcnpma, s_fullwlcCtrbn_clc_cmdH_idARRAY_SIZE(inpma, s_fullwlceCtrbsult = 2)calna_tx_pwrCAL_RECAL:
{
G_F

	cLCNPH_idsystlcLCNPHCtrbclc_cmdH_idcnpma, s_reultCtrbn_clc_cmdH_idARRAY_SIZE(inpma, s_reulteCtrbcnpma, _sums_idcnpma, _sums_reultCtrbsult = 2)default:
result = 		))
;
}

static npmoncset;
	wlc_lcnphy_tx_pwr_BL_ID)

#deAL		       u16G_F

	cLCNPH, (1);(6	 6x1 << _REG12(0) << 8);

		or_phy_reg(pi, < 0), 0 << 0);

	m503phy_reg(pi, 0x64b, (0x1 << iphlc_lcnphrold;
	;
	tx_pwr_ctrl = wlc_lcnphy_get_< < 0), 0 << 0);

	maa4phy_reg(pi,, 0x(11, (0((bx1 <_override(pi);

	wlc_lcnphy_set_tx_pwr_ctrl(pi, LCOFF)= 2)s_reg	xnt_tx_prftx_p2 	save_AfeCtrlOvr = re4dite_1 << 1), 1 << 1);
	mod_p(pi,(pi, 0x64p(pi,2a6_fa, (0x1 << 0), 0 << 0);

	modbphy_r0x4a512)p (21, (0((bx1 <_override(p v_iqlo_loopbacky);

l_rf_reg[i],
	) << iphve[i]);

	if (tx_2);rn result;
el);
	wlc_lcnphy_rx_gaidutx_gaws_to_save[i]);

	if (tx_gain_override_ogvoid wlc_lnnphy_get_tx_ga0x1 <reg(!e wlc_l lcnpha << ebit!_save[i]);

	if (tx_gainn_override_old)
		wlc_lcnphy_set_txhy_t brcm (0  
	else
		index t if idx);ain_override_ogvoid wlc_lnnphy_ 0x2_lcnps);aine wlc_l lcnp_id& 0x2_lcnpsrri	))
hash
); e wlc_l lcnp0)) >> 0;, (0x1 |
n, tia_g2e wlc_l lcnp0) & 0xff;
<a54 u16)e wlc_l lcnp0) & 0xff;(x1 <ba, lt_c
);  << 5);
	5

	if (CHSPEC_IS2G(pi ? 1 : 3c= 2)call lcnp_idte wlc_l lcnp;.tmemi);(ncnrry_disable,ibut

	ptr ncnrry_disablen)hy4tia_gj0 << 2j;

iqble( lcnparamw_sum lcnphy chan_ba, lt_c] 2j++hile (ebithash
)=  = tfqble( lcnparamw_y chan_ba, lt_c]ij][0]>= 0; bcall lcnp.) >> 0;,a
				 = tfqble( lcnparamw_y chan_ba, lt_c]ij][1];0; bcall lcnp. & 0xff;
a
				 = tfqble( lcnparamw_y chan_ba, lt_c]ij][2];0; bcall lcnp. & 0xff;
a
				 = tfqble( lcnparamw_y chan_ba, lt_c]ij][3];0; bmemcpy(ncnrry_disable,
      u16d& = tfqble( lcnparamw_y chan_ba, lt_c]ij][3],
      u16d

	ptr ncnrry_disablen)hy4result = 	g} 		))
;
}

staticsvoid wlc_lnnphy_call lcnp1 << _REG12(0) << 8);

		4 0x5, aa92064_REG12(0) << 8);

		93d0},
c(0x1 <;
}

static npmoncset;
	wlc_lcnphy_tx_pwr_BL_ID)

#deAL		       u16
staticiqble(loftl lcnladder		       u16ARRAY_SIZE(
staticiqble(loftl lcnladder)		       u16(6	 (0x1 <;
}

static npmoncset;
	wlc_lcnphy_tx_pwr_BL_ID)

#deAL		       u16
staticiqble(irl lcnladder		       u16ARRAY_SIZE(
	       u16
staticiqble(irl lcnladder));(6		       u1632)< 14reg( 0f lt;
el)		}
_	u8 )){<< 
	
}

staticstop
el)		}
utx_gaw	o clean5);ain_override_ose;

	wlc		}
utx, 375but88);(0x1 ext_lna << _override_ose;

	wlc		}
utx, 375but88);(0x1 e<< _REG12(0) << 8);

		or_phy_reg(pi,64< 2), 0 <n_clc_se;

 2);

n_clc_cmdHor (i a << gainzero_ uq2);qq_n gainbestlcLCNPHi11];0; 1intcnpma, _sum);<< clc_s16 
); clc_cmdH
		v sav0g0 (dac_8);<< cnpma, _sum_idcnpma, _sums_i]phyrreg(ncnrry_disable[clc_s16 ]gainncnpma, _sum_i	    ncnrry_disable[clc_s16 ], (0xu16)cnpma, _sum_yitc		   	 (rfg20664 _REG12(0) << 8);

		4 2,dcnpma, _sum));<< ebit(clc_s16 
)=i, u1| (clc_s16 
)=i4>pubpi. ;
}

static npmoncset;
	wlc_lcnph_tx_pwr_BL_ID)

#deAL		         u1& uq
se;

 384,(6	 69_trsw	<;
}

static npmoncset;
	wlc_lcnphy_tx_pwr_BL_ID)

#deAL		         u16&zero_ uq 384,(6	 69_tr sto
(i_REG12(0) << 8);

		4 1,tclc_cmdH
		));<< ebit!;
}

staticiqble(waic_pwr_ctr	goto  voin r << 
	
}

static npmoncset;
	wlc_lcnph_tx_pwr_BL_ID)

#deAL		       u16bestlcLCNPH		       u16ARRAY_SIZE(bestlcLCNPH));(6	 96);ain_override_o npmoncset;
	wlc_lcnphy_tx_pwr_BL_ID)

#deAL		        u16bestlcLCNPH		       u166ARRAY_SIZE(bestlcLCNPH));(6	 6x1 <<  ebit(clc_s16 
)=i, u1| (clc_s16 
)=i4>psw	<;
}

static npmoncset;
	wlc_lcnphy_tx_pwr_BL_ID)

#deAL		         u16& uq
se;

 384,(6	 69_tr 
	
}

static npmoncset;
	wlc_lcnph_tx_pwr_BL_ID)

#deAL		       u16	ielse
		index =lc_results.	       u16txiqlo=lc_bestcLCNPH		       u16ARRAY_SIZE(	ielse
	
					  	index =lc_results.	     		 xiqlo=lc_bestcLCNPH)		       u16(6	 96);ai	))
;
}

static npmoncset;
	wlc_lcnph_tx_pwr_BL_ID)

#deAL		      u16	ielse
		index =lc_results.	      u16txiqlo=lc_bestcLCNPH		      u16ARRAY_SIZE(	ielse
		index =lc_results.	     	 xiqlo=lc_bestcLCNPH)	6(6	 96);ai	ielse
		index =lc_results. xiqlo=lc_bestcLCNPH_0x4il& 0b);
		 <;
}

static npmoncset;
	wlc_lcnphy_tx_pwr_BL_ID)

#deAL		       u16&	ielse
		index =lc_results.	      u166txiqlo=lc_bestcLCNPH[0], 4	6(6	 8(0x1 <;
}

static npmoncset;
	wlc_lcnphy_tx_pwr_BL_ID)

#deAL		       u16&	ielse
		index =lc_results.	      u166txiqlo=lc_bestcLCNPH[5]x_24, 0	 8(0x1  voin r: <_override(p v_iqlo_loopbackc voin r_);

l_rf_reg[i],
	) <	k	u8e(l_rf_reg[i],
	) << ebit!kee
			}
)
 
	
}

staticstop
el)		}
utx_ga< _REG12(0) << 8);

		4dbphs_reg	xnt_tx_prftx_p21 << _REG12(0) << 8);

		4 0x5,) << ebit_save[i]);

	if (tx_gain_override_osvoid wlc_lnnphy_get_tx_ga0x1<_override(pi);

	wlc_lcnphy_setiphlc_lcnphrold1 << _REG12(0) << 8);

		or_phs_regg(pi, 0x6da, old_ss2064_REG12(0) << 8);

		6dbphs_regg(pi, 0x6db, old_s) <<ectrlovrva< 0));
}

staticidhylc if est__by_distribution_pub *ptrhy_txt brcsu2G(nd,c_save[i]);

	if (tx_;
)t brcms_phy *pi, strucget_tx_ga;32_by_distribution(str_idcnntx_gasl_w(ptr,crby_distribution, f (LC_ro)q_ngainidhyT if, idhyT if0_2C, idhyT if0_OB, idhyT if0_<< l_rf__OB,
n, tiidhyT if0_<< l_rf__2Cq_ngainSAVEg	xnt_tx_p;
	;
	tx_pwr_ctrl = wlc_lcnphy_get_tgainSAVEglpfxff;
	g< 16)C, 0x07);
	write_radio_reg(p1,, x1 gainSAVEgjtag_bb_afeggCe_ph =
es_to_save[i] =
			rete_radio_reg(pix7n = 1x1 gainSAVEgjtag_ ux & 
	g< 16)C, 0x07);
	write_radio_reg(p0rel_& _r0_x1 gainSAVEgiqadcf ux_e;
	g< 16)C, 0x07);
	write_radio_reg(p1,el_& 4;	u16 SAVEgbbmulc;
	;
	tx_pwr_ctrl bbmulc_ar_trswsdhyT if 	save_AfeCtrlOvr = re4rite_4su2G(nd
); 0;

	(bcma
ave_32(lcnpd11cnre, D11g(pOFFS(maccnntrol)phyitc	 MCTL_ENCMAC)_gaws_to!su2G(ndgain_oaLC_su2G(nd_mac_a, _waic_pwxxfffefeCshim0x1<_override(pi);

	wlc_lcnphy_set_tx_pwr_ctrl(pi, LCOFF)= 2)iphve[i]);

	if (tx_2);rn result;
el);
	wlc_lcnphy_rx_gaidutx_gaw_override_ogvoid wlc_lnnphy_get_tx_ga0x1 <;
	tx_pwr_cms_phylc_lcnphy_disable_tx_gan_override_old)
		wlc_lcnphy_set_tx;1272064_REG12C, 0x07);
	write_radio_reg(p0, 0, 06pi, < 0)4, 0x01);
	write_radio_reg(pix7 3,
1, (0x1 << 1) \
	(0 != (read_radio_reg(p0FF	 _r0_0x43b, 40x1 << 1) \
	(0 != (read_radio_reg(p0,F 0},4	 939, (0x1<_override(p  if i); r__get_< < 0), 0 << 0);

	mad7phy_reg(pi, 0x6dg(pi0)te_<< 0), 0 << 0);

	mod7phy_reg(pi, 0x64g(pi, bx1 <_override(pi);
bbmulc_ar);

	bx1 <_ovewr_clo_ umm;
ely_seti);
, OFF)= wsdhyT if 	se_pwr_npt(pi) \
	((readbphy_i, 10x4a7) & 
nn, ti & (0x3 wsdhyT if0_2C 	se_pwr_npt(pi) \
	((re63ephy_i, 10x4a7) & 
nn	 & (0x3 wsii idhyT if0_2C >= 256_ctridhyT if0_OB;
	idhyT if0_2C - 256 <       pidhyT if0_OB;
	idhyT if0_2C + 256 < wsdhyT if0_<< l_rf__OB;
	idhyT if0_OB; wsii idhyT if0_<< l_rf__OB;>= 256_ctridhyT if0_<< l_rf__2C;
	idhyT if0_<< l_rf__OB;- 256 <       pidhyT if0_<< l_rf__2C;
	idhyT if0_<< l_rf__OB;+ 256 <<< 5), 0 << 5);

	moa6phy_reeg(pi, 0x48idhyT if0_<< l_rf__2C_fa, (0x1 << 0), 0 << 0);

	mod_phy_reg(pi,, 0x60b, (01(bx1 <_override(pi);
bbmulc_ar);SAVEgbbmulc_gan_override_old)
		wcnphy_disable_tx,c_save[i]);

	if (tx__gan_override_old)
		wcnphnnphy_get_tx_ga0x1<_override(pi);

	wlc_lcnphy_setSAVEg	xnt_tx_p0x1 <rREG114, 0x01);
	write_radio_reg(p0, 0,SAVEglpfxff;pi, < 0)4, 0x01);
	write_radio_reg(pix7 3,
1, SAVEgjtag_bb_afeggCe_ph0x1 << 1) \
	(0 != (read_radio_reg(p0FF	 _r0_0xSAVEgjtag_ ux & 0x1 << 1) \
	(0 != (read_radio_reg(p0,F 0},4	 SAVEgiqadcf ux_e;0x1 << 1) \
	(0 != (read_radio_reg(p0, 0, 0, 0}_, (071 <ws_to!su2G(ndgain_oaLC_ms_phylmac_pwxxfffefeCshim0x1ectrlovrva< 0));
}

staticvbat
e0x2_0))

#s); r_lcnphy_rx_iq_est(strucr8rm),e)
le
t brcsu2G(ndrride_os_reg	xnt_ld_sEn;	u16 -ux & 0vmilcoursed, ux & 0vmilf_gad, ux & 0cnph; u1int ux & 0vmil;tx tbl_offset)
{

	struct p16 lvltCtr16 s_reg0 !ix7 3s_reg0 !iFF	 s_reg0 !0,F 0s_reg0 !ix5 0s_reg0 !i25xFE16d
_reg0 !0,2rride_ol_rf_reg[i],
		14];32 cms_phy *prlcsi;ris;
	u32 ii, qq;
	struct brcms_phy_lcnphy *pi_lcn = 	o clean999)= 2)s_reg0 !ix7  0x38 << 16)C, 0x07);
	write_radio_reg(piO_206	s_reg0 !iFF  0x38 << 16)C, 0x07);
	write_radio_reg(piFF)= ws_reg0 !0,F  0x38 << 16)C, 0x07);
	write_radio_reg(p1,el06	s_reg0 !i05  0x38 << 16)C, 0x07);
	write_radio_reg(piO5l06	s_reg0 !i25  0x38 << 16)C, 0x07);
	write_radio_reg(pi25)= ws_reg0 !0,2  0x38 << 16)C, 0x07);
	write_radio_reg(p1,220664< 2), 0 << 2);

14or (i = 0; i < 11; i++)
		vasave_AfeCtrlOvr = e0x20))

#feCtrlOH
		));4su2G(nd
); 0;

	(bcma
ave_32(lcnpd11cnre, D11g(pOFFS(maccnntrol)phyitc	 MCTL_ENCMAC)_gaws_to!su2G(ndgain_oaLC_su2G(nd_mac_a, _waic_pwxxfffefeCshim0x1<s_reg	xnt_ld_sEn
	g< 16)C, 0x07);
	wri	maa4bx1 <_override(pi);

	wlc_lcnphy_set_tx_pwr_ctrl(pi, LCOFF)= iq) / 2);
	else
		index = pi_lcn->lcnphn_override_old)
		wlc_lcnphy_set_tx;1272064< 0)4, 0x01);
	write_radio_reg(pix7 3,
1, ,
1<< 4<< 1) \
	(0 != (read_radio_reg(p0FF	 _r0_0x_reg(pi, x1 << 1) \
	(0 != (read_radio_reg(p0,F 0},4	 _reg(pi, i, < 0), 0 << 0);

	m503phy_reg(pi, 0x6a_fa, (0x1 << 0), 0 << 0);

	m503phy_reg(pi, 0x6d1, (0x1 << < 0), 0 << 0);

	maa4phy_reg(pi,4 0x60b, (01 << 142), 	rfgain1 = (reaa4phy_reg(pi,, 0x60a, (01(0x1 << 5), 0 << 5);

	mod_phy_reg(pi, 0x6da, (0x1 << 2), 	rfgain1 = (reaa5phy_reg(pi, 0x48l) ((a, (0x1 << 0), 0 << 0);

	moa5phy_r0x4a512)p (515 = ((bx1 << 0), 0 << 0);

	moa5phy_r0x4a5, 0x481, (0x1 << 8), (0) << 8);

	mo0dphy_reg(pi, 0x4864((a, (0x1 << 0), 0 << 0);

	mo0dphy_r0x4a5, 0x461, (0x1 << 8), (0) << 8);

	moa2phy_reg(pi, 0x4864((a, (0x1 << 0), 0 << 0);

	moa2phy_r0x4a5, 0x461, (0x1 << 8), (0) << 8);

	mod_phy_r7g(pi, 0x42b, (0x1 << 8), (0) << 8);

	mod_phy_r7g(pi, 0x431, (0x1 << 8), (0) << 8);

	mod_phy_r7g(pi,, 0x(11, (0((bx1 << 0), 0 << 0);

	modaphy_reg(pi,, 0x60b, (01(bx1 << 0), 0 << 0);

	modaphy_reg(pi,, 0x64b, (01x1 << 3), (0) << 3);

	moa6phy_reg(pi,, 0x6da, (01(0x1 <rREG114, 0x01);
	write_radio_reg(pi,1f, xC1 << 3), ) \
	(0 != (read_radio_reg(p0x1f, x0, _reg(pi,  << 3), (0) << 3);

	mod_phy_reg(pi, 0x(11, (0x1 << < 0), 0 << 0);

	mod_phy_reg(pi, 0x(11, (0x1 << < 0), 0 << 0);

	maa4phy_reg(pi,, 0x(11, (0((bx1 <v sgain_override_aidtq_ tbladcfnt_ r_overri_ptr = tfo ta_tx_pwr_BL_ID)RFSEQ;bl_len = tbl_len;
16;sebl_id = bl_ptr1;32_ptr = ttbl_id&vltCtr_ptr = t_width;
	6;set = tbl_offset;
	wlc_lcnphy_write_
		reture;

	_EMPSENSE*if (channel != 0) {
		mod7phy_reg(pi, 0x44b, (0x1 << _hannel != 0) {
		mod7phy_r7g(pi,, 0x(11, (0((bx1 <	-ux & 0vmilcourse0 <8; <	-ux & 0vmilf_ga& 00x4; <	-ux & 0xff;
	g2rri 3), ) \
	(0 != (read_radio_reg(pi820,0x2 0}_, (050x1 ext_lna << hannel != 0) {
		mod7phy_reg(pi, 0x44b, (0x1 << _hannel != 0) {
		mod7phy_r7g(pi,, 0x(31, (0((bx1 <	-ux & 0vmilcourse0 <7; <	-ux & 0vmilf_ga& 00xa; <	-ux & 0xff;
	g2rri}pi-ux & 0vmil&  <	x3ff);(x2x4a5) 
	 (-ux & 0vmilcourse0<a54 u16-ux & 0vmilf_gate_<< 0), 0 << 0);

	mod8phy_reg(pi, 0x6e_fa, (0x1 << 0), 0 << 0);

	mod8p(pi,(pi, 0x, 0x(-ux & 0vmil1, (0x1 << < 0), 0 << 0);

	mad_phy_reg(pi, 0x41c, (0x1 << < 0), 0 << 0);

	mod8phy_r7g(pi,, 0x(-ux & 0xff;b, (01(bx1 << 0), 0 << 0);

	mod_phy_reg(pi, 0x61a, (0x1 << _REG12C, 0x07);
	write_radio_reg(p0, 0, 06pi, <_ovewr_clo_ umm;
ely_seti);
, OFF)= wsbit!_0x20))

#d	}
utx_)pi.o cleanu00x1 <rREG114, 0x01);
	write_radio_reg(pix7 3x3ff);s_reg0 !ix72064_REG12C, 0x07);
	write_radio_reg(p0FF	 x3ff);s_reg0 !iFF)= w_REG12C, 0x07);
	write_radio_reg(p0,F	 x3ff);s_reg0 !1,el06	rREG114, 0x01);
	write_radio_reg(pix1f,x3ff);s_reg0 !ix5l06	rREG114, 0x01);
	write_radio_reg(pi21f,x3ff);s_reg0 !i25l06	rREG114, 0x01);
	write_radio_reg(p0, 0,x3ff);s_reg0 !1,, i, < 2), 0 << 2);

14or (i = 0_REG12(0) << 8);

e0x20))

#feCtrlOH
		

l_rf_reg[i],
	
		));4_override_old)
		wlc_lcnphy_set_tx;(rlc)s_phyiga6	rREG114, 0x01);
	wri	maa4phs_reg	xnt_ld_sEn1 <ws_to!su2G(ndgain_oaLC_ms_phylmac_pwxxfffefeCshim0x1	o clean999)= table(pi, < 0));
}

staticixhlc_lcnphrinic_lcnphy_rx_iq_est_pub *ptrhy_txt brcms_phy *pi, struc		wcnphsCtr16 bbmulc;tx tbl_offset)
{

	struct p = { 1, b_phb1t p = {t if, lc_, maxe wlc_lc_, mrlc wlc_lc_;e
t brcsu2G(ndrri_by_distribution(str_idcnntx_gasl_w(ptr,crby_distribution, f (LC_ro)q_;4su2G(nd
); 0;

	(bcma
ave_32(lcnpd11cnre, D11g(pOFFS(maccnntrol)phyitc	 MCTL_ENCMAC)_gaws_to!su2G(ndgain_oaLC_su2G(nd_mac_a, _waic_pwxxfffefeCshim0x1aws_to!pwxxhwnt_tx_pwcaplc_l)a << ebit << 5);
	}

	if (CHSPEC_IS2G(pi-ubpi. 		wcnphs.) >> 0;,a 4;	u. 		wcnphs. & 0xff;
	g,2rri. 		wcnphs. & 0xff;
ag,2rri. 		wcnphs.cms_phy gaigai
			bbmulc;
	15qq_n  << 1) + rbi		wcnphs.) >> 0;,a 7;	u. 		wcnphs. & 0xff;
	g,5rri. 		wcnphs. & 0xff;
ag,4rri. 		wcnphs.cms_phy gaigai
			bbmulc;
	15qq_n  ain_override_osvoid wlc_lnnphy_		wcnphs_tr 
	
}

statici);
bbmulc_ar);bbmulc_gan	;
}

staticvbat
e0x2_0))

#s); r_ar);_EMPSENSE*x1 ext_lna <an	;
}

staticidhylc if est_ptx_ga< c;
}

static void wxy_measl_widthwlar_trsw	b0 );pif gepa_2g[0]phyrb1 );pif gepa_2g[1];0; a1 );pif gepa_2g[2];0; maxe wlc_lc_2);rn result;
e if2dbmb0_0x 1, b_phb1_gan	mrlc wlc_lc_2);rn result;
e if2dbmb021f, 1, b_phb1_ga
	__ptr = tfo ta_tx_pwr_BL_ID))

#dCTL;bll_len = tbl_len;
32;.t2_ptr = ttbl_id&lc_;e
ebl_id = bl_ptr1;322_ptr = t_width;
	q_es < 2),t if 	s< 2t if a5128 2t if(i a << 	lc_2);rn result;
e if2dbmbt if,  1, b_phb1_ga
	_	lc_2);(lc_2< mrlc wlc_lc_i ? mrlc wlc_lc_2: lc_;e
e(;
}

staticset;
	wlc_lcnphy_write_422_ptr = t_width00);
  ain< 0), 0 << 0);

	mod_phy_reg(pi, 0x6a_fa, (0x1in< 0), 0 << 0);

	mod3phy_reg(pi, 0x48a_fa, (0x1in< 0), 0 << 0);

	mod3phy_reg(pi,, 0x481, (0x1 <in< 0), 0 << 0);

	mod_phy_reg(pi, 0x40b, (0x1 <in< 0), 0 << 0);

	mod_phy_reg(pi, 0x6d1, (0x1 << n< 0), 0 << 0);

	mo1_phy_reg(pi7 0x481, (071 <<  _REG12(0) << 8);

		4a8phu00x1 <(;
}

staticss8)i wlc_lnphy_g_pw,;_tx_TARGETtrl(0x1 <(;
}

staticss8)i	wlc_lcnphy_set_tx_pwr_ctrl(pi, LCHW)rri}pis_to!su2G(ndgain_oaLC_ms_phylmac_pwxxfffefeCshim0x1ectrlovrva< 0));
}

staticss8)p 0xff;_lcnphy_rx_iq_est(strucgainxff;bpi, u16 a, u16 b)
{
	m4f_phy_reg(_tx_pwri, strcnphovrl_r1)p ve[i]);r_l_r1)MASKphy_reg(xff;
<a5_tx_pwri, strcnphovrl_r1)p ve[i]);r_l_r1)SHIFTte_<< 0), 0 << 0);

	mofdphy_reg(_tx_pwrstxi, strcnphovrl_r1)p ve[i]);r_l_r1)MASKphy_reg(xff;
<a5_tx_pwrstxi, strcnphovrl_r1)p ve[i]);r_l_r1)SHIFTte_
	r< 0)
_override_ogvoi(CHSPEloft__by_distribution(struct _re16 *ei0,e16 *eq0,e16 *fi0,e16 *fq0bpi, *ei0 ta_tx_pwr

#deCeg(AD(< 16)C, 0x07);
	write_radio_reg(pi89)_gaw*eq0 ta_tx_pwr

#deCeg(AD(< 16)C, 0x07);
	write_radio_reg(pi8A)_gaw*fi0 ta_tx_pwr

#deCeg(AD(< 16)C, 0x07);
	write_radio_reg(pi8B)_gaw*fq0 ta_tx_pwr

#deCeg(AD(< 16)C, 0x07);
	write_radio_reg(pi8C))ga
	r< 0));
}

statici);

	wiqcc_lcnphy_rx_iq_est(strucgainaucgainbhy_tx tbl_offset)
{

	struct p1ainiqcc[2];0pisqcc[0	vasa; <sqcc[1	vasct .t_ptr = tfo ta_tx_pwr_BL_ID)

#deAL;bl_len = tbl_len;
16;sebl_id = tbl_idsqcc;sebl_id = bl_ptr2;.tbl_id = _width;
	8qq_nt = tbl_offset;
	wlc_lcnphy_write_tab< 0));
}

statici);

	wlocc_lcnphy_rx_iq_est(strucgaindidqhy_tx tbl_offset)
{

	struct .t_ptr = tfo ta_tx_pwr_BL_ID)

#deAL;bl_len = tbl_len;
16;sebl_id = tbl_id&didq;sebl_id = bl_ptr1;32_ptr = t_width;
	85q_nt = tbl_offset;
	wlc_lcnphy_write_tab< 0));
}

statici);

	wlc_lcnphy_setlcnphy_rx_iq_est(strucrlcsi_phyin, u tbl_offset)
{

	struct p1ain 0, Ctr16 bb_mulc;tx16 lbbmulcsqcox2d, , str, locLCNPH, rf_meas;txt brcms_phy *pi, structx_ga;32_by_distribution	struct brcms_phy_lcnphy *pi_lcn = pi	ielse
		index txy_measlt_cu_disable2); =8);->lcnphn	ielse
		index = pi_lcn->lcn
); u8);->lcnphy__ptr = tfo ta_tx_pwr_BL_ID))

#dCTL;bl_len = tbl_len;
32;.tbl_id = bl_ptr1x1 <_override(pi);

	wlc_lcnphy_set_tx_pwr_ctrl(pi, LCOFF)= 2)th = tbl_width;
	_tx_pwr_ctrl(pi, LCIQ_OFFSET +;->lcnphnbl_id = tbl_id&bbmulcsqcox2q_nt = tbl_offset;
	wlc_lcnph_write_1 th = tbl_width;
	_tx_pwr_ctrl(pi, LCGAIN_OFFSET +;->lcnphnbl_id = bl_len;
32;.tbl_id = tbl_id&i, strq_nt = tbl_offset;
	wlc_lcnph_write_1 cnphs.) >> 0;,a x3ff);(i, str& save_n;1 cnphs. & 0xff;
	gx3ff);(i, str&n>,in = (rf16_1cnphs. & 0xff;
agx3ff);(i, str&n>,ff);= (rf16_1cnphs.cms_phy gaix3ff);(bbmulcsqcox2&n>,2in = (r07);4_override_old)
		wlc_lnnphy_tx_ga0x1<_override(pi);
p 0xff;__tx;(3ff);(i, str&n>,24n = (r7g20664bb_mulc
); u8);((bbmulcsqcox2&n>,20)& save_n;1 	
}

statici);
bbmulc_ar);bb_mulc_ga <;
	tx_pwr_cms_phylc_lcnphy_disable_tx_ga
 = { 0, 0, 0 };

 0x20))

#(wlc_lcnphy_tssi_based_pwr_a <an	agaix3ff);((bbmulcsqcox2&n>,10)& sav3e_n;1 ip;
	b3ff);(bbmulcsqcox2& sav3e_n;1 i;
}

statici);

	wiqcc_ar); 0, _ga
	__ptr = t_width;
	_tx_pwr_ctrl(pi, LCLO_OFFSET +;->lcnphntbl_id = tbl_id&locLCNPH;1 i;
}

staticset;
	wlc_lcnph_write_1 i;
}

statici);

	wlocc__tx;(3ff);locLCNPH_ga
	__ptr = t_width;
	_tx_pwr_ctrl(pi, LCrl(pOFFSET +;->lcnphntbl_id = tbl_id&rf_meas;txi;
}

staticset;
	wlc_lcnph_write_ n< 0), 0 << 0);

	m6a6phy_reegg(pi, 0x48rf_meas *,in a, (0x1 <afectrlovrval);
	}
}

static void papdc npp	wlc_llcnphy_rx_iq_est(strhy_txg6 lj;tx tbl_offset)
{

	struct pg6 le0x2__width[128];.tbl_id = tbl_ide0x2__width;.tbl_id = bl_ptr128rri_ptr = tfo ta_tx_pwr_BL_ID)PAPDCOMPDELTA_BLphnbl_id = bl_len;
32;.tbl_id = _width;
	q_e.tmemi);(e0x2__width,ibut

	ptr e0x2__widthn)hy4tia_gj0 <1 2j;

128 2jv+= 2<< 4t0x2l_width[j]& 00x8	e0_ga <;
	tx_pwr_cset;
	wlc_lcnphy_write_4f (tem= 
	r< 0));
}

staticiphyutlcnphy_rx_iq_est(struct brcbE_base)
le
r { 0bE_base)a <an	a, ~(0x228));

		aa3,
	~x3ff);((_reg(pi, u16) ~eg(pi4))1 << n< 0), 0 << 0);

	mod_phy_reg(pi, 0x1, (0x1 << 0a, ~(0x228));

		aad_p
     u1~x3ff);((_reg(pi34
		  r, u32y_reg(pi54
		  r, u32y_reg(pi124
		  r, u32y_reg(pi0 u16) ~eg(pi, u16) ~eg(pi2))1 << na, ~(0x228));

		aaddp
     u1~x3ff);((_reg(pi34
	2y_reg(pi54
	hy_reg(pi,4 )1 <in< 0), 0 << 0);

	mo4dphy_reg(pi2),x939, (0x106 << 1), 1 << 1);
	mo4dphy_reg(pi,4
	hy_reg(pi 0x48aveg(pi0)te_< na, ~(0x228));

		aaf9p
     u1~x3ff);((_reg(pi0 u16) ~eg(pi, u16) ~eg(pi2))1 << na, ~(0x228));

		aafap
     u1~x3ff);((_reg(pi0 u16) ~eg(pi, u16) ~eg(pi2))1 < ext_lna <an	3), (0) << 3);

	mod_phy_reg(pi, 0x43b, (0x1  << 1), 1 << 1);
	mod_phy_reg(pi, 0x43c, (0x1 <	3), (0) << 3);

	mod_phy_reg(pi4 0x1fa, x1 <in< 0), 0 << 0);

	mod_phy_reg(pi6 0x43c, 60x106 << 1), 1 << 1);
	mo4_phy_reg(pi,, 0xeg(pi,, x1  << 2), 1 << 2);
	mo4dphy_reg(pi,4 0x1fa, 1 << 14i;
}

statici);

rswy_disable_tx,c_);
, n_ove0x106 << 1), 1 << 1);
	mo4dphy_reg(pi2 0x43c, , x1  << 2), 1 << 2);
	mo4cphy_reg(pi2),x939, (0x106 ebit << 5);
	}

	if (CHSPEC_IS2G(pi-ubp
   << 2), 1 << 2);
	mo4cphy_reg(pi3),x939, 3te_422<< 1), 1 << 1);
	mo4dphy_reg(pi3),x939, 3te_
   << 2), 1 << 2);
	mo4cphy_reg(pi5)0}_, (050x1 22<< 1), 1 << 1);
	mo4dphy_reg(pi5 0x43c, 5te_
   << 2), 1 << 2);
	mof9phy_reg(pi, 0x43b, (0x1   << 2), 1 << 2);
	mofaphy_reg(pi, 0x1, (0x1 << 0 << 2), 1 << 2);
	mof9phy_reg(pi2),x939, (0x1   << 2), 1 << 2);
	mofaphy_reg(pi2),x939, (0x106  << 2), 1 << 2);
	mof9phy_reg(pi, 0x43b, (0x1   << 2), 1 << 2);
	mofaphy_reg(pi, 0x43b, (0x1  ext_lna <an	 << 2), 1 << 2);
	mo4cphy_reg(pi3),x939, 3te_422<< 1), 1 << 1);
	mo4dphy_reg(pi3),x039, 3te_
   << 2), 1 << 2);
	mo4cphy_reg(pi5)0}_, (050x1 22<< 1), 1 << 1);
	mo4dphy_reg(pi5 0x13c, 5te_
   << 2), 1 << 2);
	mof9phy_reg(pi, 0x43b, (0x1   << 2), 1 << 2);
	mofaphy_reg(pi, 0x0, (0x1 << 0 << 2), 1 << 2);
	mof9phy_reg(pi2),x939, (0x1   << 2), 1 << 2);
	mofaphy_reg(pi2),x039, (0x106  << 2), 1 << 2);
	mof9phy_reg(pi, 0x43b, (0x1   << 2), 1 << 2);
	mofaphy_reg(pi, 0x03b, (0x1  e64	)ave_AfeCtrlOvr);
}

staticrun_samples__by_distribution(struct cm (0  de_onum_sampsuct cm (0  de_onum_loopsucgainwaicuct brciqblem),e)
le
)), (0) << 8);

		or_phy_808x1 << 0), (1) << 0);

	mo42phy_r0g(pi, 0x48num_samps - e_fa, (0x1rreg(num_loops3c);
_reg(pct num_loops--e_<< 0), 0 << 0);

	m640phy_reggg(pi, 0x4num_loops3a, (0x1 << 0), 0 << 0);

	m641phy_reggg(pi, 0x4waic3a, (0x1 <sii iqblem),e)a <an	a, ~(0x228));

		aa 0x5x3ff);~y_reg(pi,, 0x1  ), (0) << 8);

		a 0x5x_reg(pi,, 0x1 ext_lna << _REG12(0) << 8);

		63f, (0x1  ;
}

staticiphyut);

(0x1 e<< eg(C, 0x07);
	write_radio_reg(p0, 0, 06te_tab< 0));
}

staticdeafrx_spur_avoidance_mode(struct brcm),e)
le
)u8ffsebw4qq_nfsebw4q& 0 << 5);
	40
	if (CHSPEC_IS2G(pix1 <sii a1 << 1LT(

	if (LCNREV_LT(pi->pubpi.2), (0) << 8);

		ab_phy_reg(pi, 0x6m),e)a (050x1 22), (0) << 8);

		ab1phy_reg(pi9 0x03b, 90x1 ext_lna << hannel != 0) {
		mob_phy_reg(pi, 0x6m),e)a (050x1 22), (0) << 8);

		ab1phy_reg(pi9 0x03b, 90x1 e1 <sii fsebw4q& =;
	if (c2), (0) << 8_pwr

	mo1_p
 r, u32y_reg(pi64
		  r u32y_reg(pi54_ptr, u32yy << 5);
	}

	  r, u32  	if (CHSPEC_IS2G(pi-u?{ 0m),e)a: 3c, (ptr, u326u16)!m),e)a (050x1 22), (0) << 8);

		a1_phy_reg(pi7 0x4m),e)a (070x1 e1
	r< 0)
_override_ose;

	wlc		}
urrent_tx_pwr_idx(str,cr6 lf_kHzucgainmax_l_r_ptr, t brciqblem),e)
le)u8ffse_bwt p1ainnum_sampsu cuck;tx16 lbwt p = {thetagai_phroh;
	q_esrrent_tcordic_iq 		}
_samp;tx16 ldatat uf[64]rride_oi_samp, q_samp;tx tbl_offset)
{

	struct p by_distribution	struct brcms_phy_lcnphy *pi_lcn = pi	if lt;
el)		}
_	u8 hy_f_kHzga <;
	tx_pwr_cdeafrx_sputx,c_);
ix1 <fse_bw,a 4qq_nbits 0else
		index spurx_s)a << _REG12(0) << 8);

		9420,0x20x1  ;REG12(0) << 8);

		93b);

	bx1  ;REG12(0) << 8);

		93c);

	bx1  ;
}

staticiprx spur_a< 0)ancerx_sputx,cn_ove0x1 e1 <sii f_kHz)a << kptr1;322doa << 	bw,a fse_bw,*  b0,t*  = 	g	num_samps =lbw / abs f_kHz)= 	g	k00);
   module 8num_samps *m3) << (abs f_kHz))r2!);bw0x1 ext_ln
g	num_samps =l2x1 <roh;
	( f_kHz *m36l_onfse_bwl_on10q_es_hetagai_0664< 2),t 	s< 2t;

num_samps 2t++hile
) 		}
_samp_idcnrdic_ult =iq(_heta_ga
	__hetag+=hrohx106 e_samp_idx3ff);(FLOAT(		}
_samp.i *mmax_l_r)& sav3e_n;1 iq_samp_idx3ff);(FLOAT(		}
_samp.q *mmax_l_r)& sav3e_n;1 idatat uf[t	va83e_samp_(pi,0 u16q_samp;tx	))
2), 	rfgain1 = (re6d6phy_r3g(pi, 0x03b, (0x1< 0), (1) << 0);

	modaphy_reg(pi3),x939, 3te_
 bl_id = tbl_iddatat uf;.tbl_id = bl_ptrnum_samps ri_ptr = tfo ta_tx_pwr_BL_ID)SAMPLEPLAY;.tbl_id = _width;
	q_enbl_id = bl_len;
32;.tt = tbl_offset;
	wlc_lcnphy_write_1 ;
}

staticrun_samples_nphynum_sampsu _reggg,ibutiqblem),e)e_tab< 0));
}

staticitop
el)		}
urrent_tx_pwr_idx(struct bre_oplaybackc_Afeua;32_by_distribution	struct brcms_phy_lcnphy *pi_lcn = pi	if lt;
el)		}
_	u8 hy_qq_nbits 0else
		index spurx_s)a << _REG12(0) << 8);

		9420,0x70x1  ;REG12(0) << 8);

		93b);

20170x1  ;REG12(0) << 8);

		93c);

27c5);ain_override_oiprx spur_a< 0)ancerx_sputx,c_);
ix1x	))
playbackc_Afeuavasave_AfeCtrlOvr = 	m6440x1rreg(playbackc_Afeuav&48aveg(pi0)ta << _override_oiphyut);

(0x1in< 0), 0 << 0);

	m63f, y_reg(pi, 0x43b, (0x1 ext_lnareg(playbackc_Afeuav&48aveg(pi1)<< 42), (0) << 8);

		a 0x5x_reg(pi,, 0x0, (0xx1 << 2), 	rfgain1 = (re6d6phy_r3g(pi, 0x13b, (0x1< 0), (1) << 0);

	modaphy_reg(pi3),x039, 3te_
 0), (1) << 0);

	modaphy_reg(pi7),x039, 71 << a, ~) \
	(0 != (read_radio_reg(p0, 0, 0FFF9te_1 ;
}

staticdeafrx_sputx,cn_ove0x1ave_AfeCtrlOvr);
}

staticdthY_ttlcnphy_rx_iq_est(strucrlcsclc_s16 )
sintcneff_x)
sintcneff_y)
le
	boodi0dq_x1 gainx)
y,ddatatrf;.trlcs = 	sCe_ph (ble(s16 pubpicalna0:1 i;
}

statici);

	wiqcc_ar);cneff_x)
cneff_y);1 ipult = 	calna2:1 idi0dq_va83cneff_x& save_n39, xu16)cneff_y& save_n;1 i;
}

statici);

	wlocc__tx;di0dq_);1 ipult = 	calna3:<< kptr}
}

static lt =floor(cneff_x)
_);1 iy;
	8 +  = 	gkptr}
}

static lt =floor(cneff_x)
(0x1  x;
	8 -  = 	gdatatrfva83 2* ain+ y0x1  ;REG12C, 0x07);
	write_radio_reg(pi89,ddatatrf)= 	gkptr}
}

static lt =floor(cneff_y)
_);1 iy;
	8 +  = 	gkptr}
}

static lt =floor(cneff_y)
(0x1  x;
	8 -  = 	gdatatrfva83 2* ain+ y0x1  ;REG12C, 0x07);
	write_radio_reg(pi8A,ddatatrf)= 	gpult = 	calna4:<< kptr}
}

static lt =floor(cneff_x)
_);1 iy;
	8 +  = 	gkptr}
}

static lt =floor(cneff_x)
(0x1  x;
	8 -  = 	gdatatrfva83 2* ain+ y0x1  ;REG12C, 0x07);
	write_radio_reg(pi8B,ddatatrf)= 	gkptr}
}

static lt =floor(cneff_y)
_);1 iy;
	8 +  = 	gkptr}
}

static lt =floor(cneff_y)
(0x1  x;
	8 -  = 	gdatatrfva83 2* ain+ y0x1  ;REG12C, 0x07);
	write_radio_reg(pi8C,ddatatrf)= 	gpult = 		)ave_AfeCtrt brcms_phy *punsign16_t brcm
_override_ogvoi_ttlcnphy_rx_iq_est(strucrlcsclc_s16 )
le
	boo 0, , didq;seu8fdi0,edq0,eeruceq,cnx,cnq;txt brcms_phy *punsign16_t brcm cc;secc.rehy_qq_ncc.im;
	q_esrCe_ph (ble(s16 pubpicalna0:1 i;
}

staticg);

	wiqcc_ar);& 0,&ite_ ncc.rehy_ae_ ncc.im;
	b;1 ipult = 	calna2:1 ididq 
	;
	tx_pwr_ctrl = wlocc__tn;1 idi0 ta(((didq  save_00b, (016)&n>,24n;1 idq0 ta(((didq  sav00e_n39, 24n n>,24n;1 icc.rehy_x3ff);di0e_ ncc.im;
	x3ff);dq_x1 ipult = 	calna3:<< _override_ogvoi(CHSPEloft_ar);&er);&eq,c&nx,c&fqn;1 icc.rehy_x3ff);eie_ ncc.im;
	x3ff);eq= 	gpult = 	calna4:<< _override_ogvoi(CHSPEloft_ar);&er);&eq,c&nx,c&fqn;1 icc.rehy_x3ff);fie_ ncc.im;
	x3ff);fq= 	gpult = 		)st))
		rccx1ave_AfeCtrlOvr);
}

staticdampwcaptlcnphy_rx_iq_est(strucrlcsclipcdetect_algoucgainthreshuct cm (sint*tblucrlcsm),e)
le)u6 lcurl_r1,lcurl_r2,crbptbluccurtbluclcntblucvltCtr1e_os(pi, 0x6da, old_ss,otimas;tx	bool ses(pi, 0x6da, old_ss; bre_oimag,savel;32_by_distribution	struct brcms_phy_lcnphy *pi_lcn = pitimas;
	q_esl ses(pi, 0x6da, old_ssvasave_AfeCtrlOvr = 	m6da<< 14curl_r1;
	bcma
ave_16(lcnpd11cnre, D11g(pOFFS(psmlcnrectlsts 0x1 tbl[130	vasq_esbcma
;REG116(lcnpd11cnre, D11g(pOFFS(psmlcnrectlsts uct cm (0((eg(pi64
	lcurl_r1)1 << bcma
;REG116(lcnpd11cnre, D11g(pOFFS(smpl_clc;
sentblr

	m7E0_eCtrbcma
;REG116(lcnpd11cnre, D11g(pOFFS(smpl_clc;
septblr

	m8	e0eCtro clean2(0x11curl_r2;
	bcma
ave_16(lcnpd11cnre, D11g(pOFFS(psmlfeCthdd param)eCtrbcma
;REG116(lcnpd11cnre, D11g(pOFFS(psmlfeCthdd param)uct cm (0curl_r2;|sav300x1 <rREG11feCtrlOvr = 	m555);

	bx1 rREG11feCtrlOvr = 	m5a6	 _r50x1 <rREG11feCtrlOvr = 	m5a 0,x3ff);eture;|t2),e;<pi, bx1<rREG11feCtrlOvr = 	m5cf, 3bx1 rREG11feCtrlOvr = 	m5a5);

3bx1 rREG11feCtrlOvr = 	m583);

	bx1 rREG11feCtrlOvr = 	m584	 _r	bx1 rREG11feCtrlOvr = 	m585);

	eg(pi, rREG11feCtrlOvr = 	m586x5, O	e0eCt, rREG11feCtrlOvr = 	m580

		a 0x1 << s(pi, 0x6da, old_ssvasave_AfeCtrlOvr = 	m6da<<  _REG12(0) << 8);

		or_ph3) << (s(pi, 0x6da, old_ssv|;

2008 bx1<septbl;
	bcma
ave_16(lcnpd11cnre, D11g(pOFFS(smpl_clc;
septblr0x11curtbl;
	bcma
ave_16(lcnpd11cnre, D11g(pOFFS(smpl_clc;
curtblr0x11doa << o cleanu00x1	1curtbl;
	bcma
ave_16(lcnpd11cnre, D11g(pOFFS(smpl_clc;
curtblr0x11itimas00);
  module 8curtbl;!=crbptblrpwr (timas;< 50 &  << bcma
;REG116(lcnpd11cnre, D11g(pOFFS(psmlfeCthdd param)u,0x20x1 sentbl& 00x7E0_Ctrbcma
;REG132(lcnpd11cnre, D11g(pOFFS(tplatewntblr

sentblr;
	module sentbl&<
	m8	e0ea << v sgaibcma
ave_32(lcnpd11cnre, D11g(pOFFS(tplatewndatar0x11iimag ta((v sgn>,ff);= (r3e_n;1 iavel ta((v s);= (r3e_n;1 isii imag > 511 
nn	imag -= 1024x106 ebitavel > 511 
nn	avel -= 1024x106 ebit 0else
		index iqble(swpclse 
nn	tbl[ sentbl&-
	m7E0_e / 4	vasavetCtrb(coeff_xtbl[ sentbl&-
	m7E0_e / 4	vasimagx106 ebitclipcdetect_algo ile (iqii imag > threshu1| imag < -thresh ile (i sentbl& 00x8e0_ga	f_xtbl[130	vas1;322  ainto
(isentbl&+a 4;	ue<< _REG12(0) << 8);

		or_phl ses(pi, 0x6da, old_sseCtrbcma
;REG116(lcnpd11cnre, D11g(pOFFS(psmlfeCthdd param)u0curl_r2eCtrbcma
;REG116(lcnpd11cnre, D11g(pOFFS(psmlcnrectlsts ulcurl_r1)x1ave_AfeCtrlOvr);
}

statica1tlcnphy_rx_iq_est(strucrlcsclc_s16 )
rlcsnum_lev(co,
n, tia_rlcsstep_

	p_lg2)
le)constrt brcms_phy *pspb)		}
(sttic 1;txt brcms_phy *pspb)		}
(ttic 2;txt brcms_phy *punsign16_t brcm ttic 3;.trlcsttic 4	 ttic 5);k, s,oj	 ttic 6;tx	boottic 7	 ttic 8	 ttic 9; bre_ottic 10	 ttic (1);ttic (2);ttic (3);ttic (4	 ttic 15	 ttic 16; bre_o*tblucttic 17);4 = {ttic 18	 ttic 19; bu= {ttic 20	 ttic 21;txt brcttic 22);ttic 23);ttic 24	 ttic 25;tx	boottic 26	 ttic 27;tx	boottic 28	 ttic 29	 ttic 30;tx	boottic 31x1 gainsttic 32;.tttic 21vasq_esttic 10,a fse_ (3,a fse_ (4,a fse_ 8vasq_estbl& 0kmalloc(

	ptr s (0x*
131, GFP_ATOMICIO_2064_NULL;

	tblr
trf (tem= 2)ttic 32& 0kmalloc(

	ptr , (0x*
20, GFP_ATOMICIO_2064_NULL;

	ttic 32)a << k	u8e(tblr;
	rf (tem=   aittic 26vasave_AfeCtrlOvr = 	m6da<<  ttic 27vasave_AfeCtrlOvr = 	m6db<<  ttic 31 	g< 16)C, 0x07);
	write_radio_reg(p026<<  _REG12(0) << 8);

		93d0},
C(0x1 <;
}

staticse;

	wlc		}
utx, 375but88);	bx1 rREG11feCtrlOvr = 	mor_phy_reg(pi, eg(feCtrlOvr = re6di);, 31 <_4_override(p v_iqlo_loopbacky);

ttic 32)Ctro clean5e0eCtrttic 28vasave_AfeCtrlOvr = 	m938eCtrttic 29vasave_AfeCtrlOvr = 	m4d_2064ttic 30vasave_AfeCtrlOvr = 	m4d8pi, eg(feCtrlOvr = reod_ph_reg(pi, i, ), (0) << 8);

		ad7ph_reg(pi, i, ), (0) << 8);

		ad7ph_reg(pi3bx1 hannel != 0) {
		mod7phy_r7g(pi,, 0x0x2g(pi,, x1 ), (0) << 8);

		ad80x43b, (0x1 ), (0) << 8);

		ad80x43b, (0x1 << 1), 1 << 1);
	mod8p(pi,(pi, 0x, 0x0x23Ag(pi, i, < 0), 0 << 0);

	mod8phy_r7g(pi,, 0x_r7g(pi,, _esttic 1_id&lphy *pspb)		}
_375b[0]phyttic 4n;
32;.1rreg(num_lev(co& =;
	if (cebitclc_s16 
c);
) 	g	num_lev(co&  4;	u.(coeff_xnum_lev(co&  9;ri}pis_tostep_

	p_lg2& =;
	if (cebitclc_s16 
c);
) 	g	step_

	p_lg2&  3;	u.(coeff_xstep_

	p_lg2&  8;	ue<< ttic 7 ta(43b, step_

	p_lg2)064ttic 3 
	;
	tx_pwr_ctrl cc_ar);cle(s16 p_esttic 15  0xs (0xttic 3.re_esttic 16  0xs (0xttic 3.imO_2064_clc_s16 
)=i2	if (cebitttic 3.re > 1272ff_xttic 15  0ttic 3.re - 256 < cebitttic 3.im > 1272ff_xttic 16  0ttic 3.im - 256 < }1 	
}

statici);
cc_ar);cle(s16 	 ttic 15	 ttic 16eCtro clean2(0x11< 2),fse_ 8vasq_ottic 7
c);
pwr fse_ 8v<snum_lev(co_ottic 8(i a << ttic 23  0b);
		< ttic 22&  n_ove		< rCe_ph (ble(s16 pubpiicalna0:1 isttic 10,a 511;322 sult = 	gcalna2:1 isttic 10,a 127;322 sult = 	gcalna3:1 isttic 10,a 15;322 sult = 	gcalna4:1 isttic 10,a 15;322 sult = 	gto
(ittic 9vasave_AfeCtrlOvr = 	m93__ganittic 9vas2x*
ttic 9; b ttic 24&  n_ove		< ttic 5 a 7;	u.ttic 25  0b);
		< module 1pubpi. ;REG114, 0x01);
	write_radio_reg(pi26		    	(ttic 5 = (r7 u16)(ttic 5 = (r7 ua, x1)= 	g	o clean5e)= 	g	ttic 22&  n_ove		<  tbl[130	vasq_es i;
}

staticiampwcapt);

(	 ttic 9, &tbl[0], (0x1   ebittbl[130	vaa 1)a	f_xttic 22&  b);
		< cebitttic 22)a	f_xttic 5 -= 1		< cebittttic 22
c);ttic 24rpwr (!ttic 25& 
nn	 sult = 	gws_to!ptic 22)a	f_xttic 5 += 1		< cebitttic 5 <);
p1| ttic 5 >a 7 
nn	 sult = 	gwttic 24&  ptic 22= 	gwttic 25&  n_ove		< }106 ebit tic 5 <;
) 	g	ttic 5 a 0;	u.(coe ebit tic 5 > 7) 	g	ttic 5 a 7x106 < 2),kptr-ttic 7; k <);ttic 7; k +);ttic 7pubpi. < 2),lptr-ttic 7; l <);ttic 7; l +);ttic 7pubpi. sttic 11,a fse_ (5 +  = 	g sttic 12,a fse_ (in+ lx106   ebit tic 11,<r-ttic 1
) 	g	 sttic 11,a -ttic 1
= 	g s(coe ebit tic 11,> ttic 1
) 	g	 sttic 11,a ttic 1
= 	g sebit tic 12,<r-ttic 1
) 	g	 sttic 12,a -ttic 1
= 	g s(coe ebit tic 12,> ttic 1
) 	g	 sttic 12,a ttic 1
= 	g s	
}

statici);
cc_ar);cle(s16 	 ttic 11		        ttic 120x1   ro clean2(0x11s i;
}

staticiampwcapt);

0

0

tbluc(0x106  sttic 18vasq_es isttic 19vasq_es istia_gj0 << 2j;

128 2j(i a << 	(cebitclc_s16 
c);
) 	g	 isttic 6  0j %sttic 4_es is.(coeff_x isttic 6  0(2x*
j) %sttic 4_e 	g	 sttic 2.rehy_ttic 1[ttic 6].re_esg	 sttic 2.im;
	ttic 1[ttic 6].imO_2s isttic 17 tatbl[j]O_2s isttic 18;
	ttic 18 + ttic 17 *
ttic 2.re_esg	 sttic 19vasttic 19v+ ttic 17 *
ttic 2.imO_2s i}106  sttic 18vasttic 18vn>,10_es isttic 19vasttic 19vn>,10_es isttic 20 ta((ttic 18v*sttic 18) +
       it tic 19v*sttic 19))x106   ebit tic 23 1| ttic 20 < ttic 21 a << 	(cttic 21&  ptic 20_esg	 sttic 13,a fse_ (1_esg	 sttic 14&  ptic ,2rri.   ain< ttic 23  0n_ove		<   ainto< ttic 23  0b);
		< ttic 15  0ttic 13		< ttic 16  0ttic ,4rri.ttic 7
);ttic 7vn>,1		< m
}

statici);
cc_ar);cle(s16 	 ttic 15	 ttic 16eCtrro clean2(0x11to<goto  voin r < voin r: <_override(p v_iqlo_loopbackc voin r_);

ttic 3(0x1<_override(pstop
el)		}
utx_gawrREG11feCtrlOvr = 	mor_phttic 26<<  _REG12(0) << 8);

		6di);ttic 27<<  _REG12(0) << 8);

		od_phttic 28<<  _REG12(0) << 8);

		od7phttic 292064_REG12(0) << 8);

		od8phttic 3	bx1 rREG114, 0x01);
	write_radio_reg(pi26	ottic 31)x106k	u8e(ttic 3(0x1<k	u8e(tblr;
tab< 0));
}

staticg);

	wiqcc_lcnphy_rx_iq_est(strucgain*aucgain*b)
le
	booiqcc[2];0x tbl_offset)
{

	struct .t_ptr = ttbl_idsqcc;sebl_id = bl_ptr2;.tbl_id = fo taq_enbl_id = _width;
	8qq_n_len = tbl_len;
16;se;
}

staticset;
	wlc_lcnph_write_1 *a_idsqcc[0]phy*b_idsqcc[1];0table(pi, < 0));
}

staticixhiqlo_softlcle(fullurrent_tx_pwr_idx(struct br brcms_phy *punsign16_t brcm sqcc0, locc2, locc3, locc4x1 <_override(pi);
cc_ar);0

0

00x1<_override(ps);
cc_ar);2

0

00x1<_override(ps);
cc_ar);3

0

00x1<_override(ps);
cc_ar);4

0

00x11<_override(pa1_ar);4

0

00x1<_override(pa1_ar);3

0

00x1<_override(pa1_ar);2);3

(0x1<_override(pa1_ar);0

5	 80x1<_override(pa1_ar);2);2)
(0x1 _override(pa1_ar);0

4, 3te_
 sqcc0 
	;
	tx_pwr_ctrl cc_ar);00x1<locc2 
	;
	tx_pwr_ctrl cc_ar);20x1<locc3 
	;
	tx_pwr_ctrl cc_ar);30x1<locc4 
	;
	tx_pwr_ctrl cc_ar);4r;
tabgainw
	tx_pwr_ctrl = wlocc_rrent_tx_pwr_idx(struct br brcmsfset)
{

	struct 	gaindidqphy__ptr = tfo taqq_n_len = tbl_len;
16;sebl_id = tbl_id&didq;sebl_id = bl_ptr1;32_ptr = t_width;
	85q_nt = tbl_offset;
	wlc_lcnph_write_1 t))
		rdidq;stable(pi, < 0));
}

staticixpwr = tfqlo_cle_rrent_tx_pwr_idx(struct txt brcms_phy *pi, struc	 wlc_l lcnp,cget_tx_ga;3216 s_regbb_mulc;tx1boo 0, , didqphs_regpa_phy gaigai	urlcsidxetSAVEg	xnt_->lcn
); 0FF; p16 lvltCtr1ainSAVEg	xnt_tx_p;
	;
	tx_pwr_ctrl = wlc_lcnphy_get_tr brcmsfset)
{

	struct 	g8 ei0,eeq0,efi0,efqq_esrrent_ttribution	struct brcms_phy_lcnphy *pi_lcn = pi_override_ogvoid wlc_lnnphy_get_tx_ga0x1	s_regpa_phy gai_override_ogvoip 0xff;__t1 << s_regbb_mulcgai_override_ogvoibbmulc_ar_trswsbitSAVEg	xnt_tx_p;

	_tx_pwr_ctrl(pi, LCOFF)trrSAVEg	xnt_->lcn
);;
	tx_pwr_ctrl c pi_lcn= wlc_lidx_ar_trsw_override(pi);

	wlc_lcnphy_set_tx_pwr_ctrl(pi, LCOFF)= 2)thwlc_l lcnp.) >> 0;,a 7;	uthwlc_l lcnp. & 0xff;
	g0;	uthwlc_l lcnp. & 0xff;
ag21;	uthwlc_l lcnp.cms_phy gaigain_override_osvoid wlc_lnnphy_	 wlc_l lcnphx1 <sii a1 << 1IS(

	if (LCNREV_LT(pi1 u1|  0else
		index hw iqble(en)){<< 
	
}

staticsd)
		wlc_lcnphy_set_tx;300x1 <(;
}

staticixhiqlo_cle_nphy_	 wlc_l lcnp		       u16it 0else
	esg	 stbl_offsecle ?t_tx_pwrCALeg(CAL :esg	 s_tx_pwrCALeFULL),cn_ove0x1 ext_lna << _override_osd)
		wlc_lcnphy_set_tx;16);ain_override_oixhiqlo_softlcle(fullu_get_t} pi_override_ogvoi(CHSPEloft_ar);&er0);&eq0,c&nx0,c&nq(0x1rreg((abs  =8);nx0)vaa 15rpwr (abs  =8);nq0)vaa 15r)a << ebit << 5);
	5

	if (CHSPEC_IS2G(pi-ubpi. 	hwlc_l lcnp.) >> 0;,a 255rri. 	hwlc_l lcnp. & 0xff;
	g255rri. 	hwlc_l lcnp. & 0xff;
ag0xf0rri. 	hwlc_l lcnp.cms_phy gaigain  << 1) + rbi	hwlc_l lcnp.) >> 0;,a 7;	u. 	hwlc_l lcnp. & 0xff;
	g45rri. 	hwlc_l lcnp. & 0xff;
ag186rri. 	hwlc_l lcnp.cms_phy gaigain  106 ebita1 << 1IS(

	if (LCNREV_LT(pi1 
     u1|  0else
		index hw iqble(en)){<< 
uthwlc_l lcnp. & 0xff;
	g0;	u. 	hwlc_l lcnp. & 0xff;
ag3q_es i;
}

staticid)
		wlc_lcnphy_set_tx;16);ain(;
}

staticixhiqlo_cle_nphy_	 wlc_l lcnp		        u16i_tx_pwrCALeFULL,cn_ove0x1   << 1) + rbi_override_oixhiqlo_softlcle(fullu_get_t e64	)pi_override_ogvoid wiqcc_ar);& 0,&ite_
ididq 
	;
	tx_pwr_ctrl = wlocc__tn;1y__ptr = tfo ta_tx_pwr_BL_ID))

#dCTL;bl_len = tbl_len;
32;.tbl_id = tbl_id&vltCtsebl_id = bl_ptr1;32_ptr = t_width;
	_tx_pwr_ctrl(pi, LCRATEpOFFSET0664< 2),idx0 << 2idx0

128 2idx(i a << th = tbl_width;
	_tx_pwr_ctrl(pi, LCIQ_OFFSET +;-dxx1 <(;
}

staticset;
	wlc_lcnph_write_ nv sgai(v sg save_f0O	e0e
		    u16it3) << (a;= (r3rel_(pi,0 u16(b& sav3e_n;1 i;
}

staticset;
	wlc_lcnphy_write_1 nv sgaididq;se__ptr = t_width;
	_tx_pwr_ctrl(pi, LCLO_OFFSET +;-dx;1 i;
}

staticset;
	wlc_lcnphy_write_ue<< tielse
		index =lc_results. xiqlo=lc_a_idae_ 	ielse
		index =lc_results. xiqlo=lc_b 
	b;1 	ielse
		index =lc_results. xiqlo=lc_didq 
	didq;se	ielse
		index =lc_results. xiqlo=lc_ei0 taei0e_ 	ielse
		index =lc_results. xiqlo=lc_eq0 taeq0e_ 	ielse
		index =lc_results. xiqlo=lc_fi0 tafi0e_ 	ielse
		index =lc_results. xiqlo=lc_fq0 tafqq_e1 	
}

statici);
bbmulc_ar);s_regbb_mulc0x1<_override(ps);
p 0xff;__tx;s_regpa_phy 0x1<_override(ps);
d wlc_lnnphy_get_tx_ga0x1 <sbitSAVEg	xnt_tx_p;!
	_tx_pwr_ctrl(pi, LCOFF)trr_override(pi);

	wlc_lcnphy_setSAVEg	xnt_tx_p0x1.(coeff_;
}

staticid)
		wlc_lcnphy_set_tx;SAVEg	xnt_->lcn)x1ave_ainw
	tx_pwr_c 0x20))

#newur_avoidance_mode(struct brcm),e)
ler1ain 0x20))

l_r1,l 0x20))

l_r2;txtboo vg
	g0;	ut brcsu2G(nd  0n_ove		

		reture;

	1 a << su2G(nd
); 0;

	(bcma
ave_32(lcnpd11cnre,	        u1D11g(pOFFS(maccnntrol)phyitc		 MCTL_ENCMAC)_gawis_to!su2G(ndgainn_oaLC_su2G(nd_mac_a, _waic_pwxxfffefeCshim0x1<	;
}

staticvbat
e0x2_0))

#s); r_ar);_EMPSENSE*x1 e
	 0x20))

l_r1vasave_AfeCtrlOvr = 	m47f);= (r1FF; p 0x20))

l_r2vasave_AfeCtrlOvr = 	m477);= (r1FF; 

		re 0x20))

l_r1v>g255gain vg
	gxs (0xe 0x20))

l_r1v- 5120x1 (coeff_ vg
	gxs (0x 0x20))

l_r1; 

		re 0x20))

l_r2v>g255gain vg
+	gxs (0xe 0x20))

l_r2v- 5120x1 (coeff_ vg
+	gxs (0x 0x20))

l_r2;.1r vg
/=l2x1 <		reture;

	1 a <
22<< 1), 1 << 1);
	mo4_phy_reg(pi,, 0x41b, (01 << 14 o cleanu0(0x1in< 0), 0 << 0);

	mo4_phy_reg(pi,, 0x40b, (01 << 14is_to!su2G(ndgainn_oaLC_ms_phylmac_pwxxfffefeCshim0x1		)st))
		r vg;
tabgainw
	tx_pwr_c 0x20))

ur_avoidance_mode(struct brcm),e)
ler1ain 0x20))

l_r1,l 0x20))

l_r2;txt= { vg
	g0;	ut brcsu2G(nd  0n_ove		r1ainSAVEg	xnt_tx_p;
	;
	tx_pwr_ctrl = wlc_lcnphy_get_tr brcmstribution	struct brcms_phy_lcnphy *pi_lcn = pi		reture;

	1 a << su2G(nd
); 0;

	(bcma
ave_32(lcnpd11cnre,	        u1D11g(pOFFS(maccnntrol)phyitc		 MCTL_ENCMAC)_gawis_to!su2G(ndgainn_oaLC_su2G(nd_mac_a, _waic_pwxxfffefeCshim0x1<	;
}

staticvbat
e0x2_0))

#s); r_ar);_EMPSENSE*x1 e
	 0x20))

l_r1vasave_AfeCtrlOvr = 	m47f);= (r1FF; p 0x20))

l_r2vasave_AfeCtrlOvr = 	m477);= (r1FF; 

		re 0x20))

l_r1v>g255gain vg
	gxrlc)e 0x20))

l_r1v- 5120x1 (coeff_ vg
	gxrlc) 0x20))

l_r1; 

		re	ielse
		index t0x20))

#option;

	1u1|  0xxhwnt_tx_pwcaplc_l)a << ebit 0x20))

l_r2v>g255gainn vg
	gxrlc)e vg
-l 0x20))

l_r2 +;5120x1 .(coeff_x vg
	gxrlc)e vg
-l 0x20))

l_r20x1 ext_lna << ebit 0x20))

l_r2v>g255gainn vg
	gxrlc)e vg
+  0x20))

l_r2v- 5120x1 .(coeff_x vg
	gxrlc)e vg
+l 0x20))

l_r20x1 x vg
	g vg
/g2rri}piebit vg
<;
) 	g vg
	g vg
+;512; 

		re	ielse
		index t0x20))

#option;

	2) 	g vg
	g 0x20))

l_r1; 

		rem),e)
rr_override(pi);

	wlc_lcnphy_setSAVEg	xnt_tx_p0x1 <		reture;

	1 a <
22<< 1), 1 << 1);
	mo4_phy_reg(pi,, 0x41b, (01 << 14 o cleanu0(0x1in< 0), 0 << 0);

	mo4_phy_reg(pi,, 0x40b, (01 << 14is_to!su2G(ndgainn_oaLC_ms_phylmac_pwxxfffefeCshim0x1		)st))
		r , (0x vg;
tabs8nw
	tx_pwr_c 0x20))

#degu8e(r_avoidance_mode(struct brcm),e)
lert= {degu8e;
	;
	tx_pwr_c 0x20))

#newutrucm),e)e_	degu8e;
ain((degu8e; (ptr i,0 u+;_tx_TEMPSENSE_OFFSET +;(_tx_TEMPSENSE_DENvn>,1& 
nn/ _tx_TEMPSENSE_DEN;)st))
		r =8);degu8e;
tabs8nw
	tx_pwr_cvbat0))

ur_avoidance_mode(struct brcm),e)
ler1ainvbat0))

vltCtrt= { vg
	g0;	ut brcsu2G(nd  0n_ove		pi		reture;

	1 a << su2G(nd
); 0;

	(bcma
ave_32(lcnpd11cnre,	        u1D11g(pOFFS(maccnntrol)phyitc		 MCTL_ENCMAC)_gawis_to!su2G(ndgainn_oaLC_su2G(nd_mac_a, _waic_pwxxfffefeCshim0x1<	;
}

staticvbat
e0x2_0))

#s); r_ar);VBATSENSE*x1 e

	vbat0))

vltvasave_AfeCtrlOvr = 	m475);= (r1FF; 

		revbat0))

vltv>g255gain vg
	gxs << (vbat0))

vltv- 5120x1 (coeff_ vg
	gxs << vbat0))

vltCt1r vg
=	t vg
* _tx_VBAT_SCALE_NOM +
  ;(_tx_VBAT_SCALE_DENvn>,1&  / _tx_VBAT_SCALE_DEN;)pi		reture;

	1 a << s_to!su2G(ndgainn_oaLC_ms_phylmac_pwxxfffefeCshim0x1		)st))
		r =8); vg;
tabse(pi, < 0));
}

staticafegclkrinic_lcnphy_rx_iq_est strucr8rm),e)
le
u8ffsebw4qq_nfsebw4q& 0 << 5);
	40
	if (CHSPEC_IS2G(pix1 <0), (1) << 0);

	mod1phy_reg(pi7 0x411, (071 << reg((eture;

	AFE_CLK_INIT_MODE)PAPDrpwr (psebw4q& =;
	 u1|
n, tieture;

	AFE_CLK_INIT_MODE)TXRX2X) = 0_REG12(0) << 8);

	mod00,0x70x1 <_override(p ogghylafegpwd;__t1 <table(pi, < 0));
}

statici0x2_adj_rrent_tx_pwr_idx(struct table(pi, < 0));
}

staticglacile(simas#(wlc_lcle_rrent_tx_pwr_idx(struct 
t brcsu2G(ndrri_cms_phy *p1ainSAVEgnt_tx_p;
	;
	tx_pwr_ctrl = wlc_lcnphy_get_tr brcmstribution	struct brcms_phy_lcnphy *pi_lcn = 4su2G(nd
); 0;

	(bcma
ave_32(lcnpd11cnre, D11g(pOFFS(maccnntrol)phyitc	 MCTL_ENCMAC)_gaws_to!su2G(ndgain_oaLC_su2G(nd_mac_a, _waic_pwxxfffefeCshim0x1<;
	tx_pwr_cdeafrx_sputx,c_);
ix1i	if lt;
lastcltvaspwxxfffenowx1i	if lt;
forcecltvasn_ove		rq) / 2);
	else
		index = pi_lcn->lcnph <_override(p vpwr = tfqlo_cle_ar_trsw_override(pi);

	wlc_lcnphy_set_tx;->lcn)x1r_override(pi);

	wlc_lcnphy_setSAVEgnt_tx_p0x1.;
}

staticdeafrx_sputx,cn_ove0x1is_to!su2G(ndgain_oaLC_ms_phylmac_pwxxfffefeCshim0x1 table(pi, < 0));
}

staticperiodic_ult_rrent_tx_pwr_idx(struct 
t brcsu2G(nd,cnulpwcatCtrconstrt brcms_phy *pr wiqcox2&*r wiqcox2;.trlcsr wiqcox2_sz *p1ainSAVEgnt_tx_p;
	;
	tx_pwr_ctrl = wlc_lcnphy_get_trcms_phy *p tbl_offset)
{

	struct p = { 1, b_phb1t p = {t if, lc_, maxe wlc_lc_, mrlc wlc_lc_;e
_by_distribution	struct brcms_phy_lcnphy *pi_lcn = pi	if lt;
lastcltvaspwxxfffenowx1i	if lt;
forcecltvasn_ove		rnulpwcat;
ain(
	else
		index nulpwcatEC_ISnep;!

  ; << 5);CHANNEL
	if (CHSPEC_IS2G(pi-e_ 	ielse
		index nulpwcatEC_ISnep;=; << 5);CHANNEL
	if (CHSPEC_IS2G(pi		rq) / 2);
	else
		index = pi_lcn->lcnph <su2G(nd
); 0;

	(bcma
ave_32(lcnpd11cnre, D11g(pOFFS(maccnntrol)phyitc	 MCTL_ENCMAC)_gaws_to!su2G(ndga << _oaLC_bmac__REG12shm_pwxxfffefeCshim, M_CTS_DURATION,  b0,	bx1  ;
aLC_su2G(nd_mac_a, _waic_pwxxfffefeCshim0x1<	)pi_override_odeafrx_sputx,c_);
ix1 <_override(p vpwr = tfqlo_cle_ar_trswr wiqcox2&=s_phy *pr wiqcox2	wlc_l_LT(qq_nr wiqcox2_sz&=sARRAY_SIZE(_phy *pr wiqcox2	wlc_l_LT(qhx1 <sii a1 << 1IS(

	if (LCNREV_LT(pi1 )
rr_override(pr wiq_cle_nphyNULL,c0,c_);
, n_ove

(	 400x1.(coeff_;
}

staticr wiq_cle_nphyNULL,c0,c_);
, n_ove

(	 127206 <sii _override(p  if (wlc_lcnphy_tssi_based_pwr_a <an	_override(pidhylc if est__lcnphy_rx_iq_est_pub *) ar_trsw	b0 );pif gepa_2g[0]phyrb1 );pif gepa_2g[1];0; a1 );pif gepa_2g[2];0; maxe wlc_lc_2);rn result;
e if2dbmb0_0x 1, b_phb1_gan	mrlc wlc_lc_2);rn result;
e if2dbmb021f, 1, b_phb1_ga
	__ptr = tfo ta_tx_pwr_BL_ID))

#dCTL;bll_len = tbl_len;
32;.t2_ptr = ttbl_id&lc_;e
ebl_id = bl_ptr1;322_ptr = t_width;
	q_es < 2),t if 	s< 2t if a5128 2t if(i a << 	lc_2);rn result;
e if2dbmbt if,  1, b_phb1_ga< 	lc_2);(lc_2< mrlc wlc_lc_i ? mrlc wlc_lc_2: lc_;e
e(;
}

staticset;
	wlc_lcnphy_write_422_ptr = t_width00);
  ai}rsw_override(pi);

	wlc_lcnphy_set_tx;->lcn)x1r_override(pi);

	wlc_lcnphy_setSAVEgnt_tx_p0x1.;
}

staticdeafrx_sputx,cn_ove0x1is_to!su2G(ndgain_oaLC_ms_phylmac_pwxxfffefeCshim0x1tab< 0));
}

staticc0x6drx_sps__by_distribution(stru urlcsm),e)
le)uain 0x2#new;.trlcs 0x21,l 0x22,l 0x2_dif16_1_by_distribution	struct brcms_phy_lcnphy *pi_lcn = pirCe_ph (m),e)a <gcalna_pwrPERICALeCHAN:esgpult = 	calna_pwrFULLCAL:<< _override_operiodic_ult__get_t pult = 	calna_pwrPERICALe_pwINIT:<< _override_operiodic_ult__get_t pult = 	calna_pwrPERICALeWATCHDOG:<< s_to;
	tx_pwr_c 0x20))

#(wlc_lcnphy_tssi_based_pwr_a <	 4t0x2lnew;
	;
	tx_pwr_c 0x20))

_ar);00x1< 4t0x21 ta_tx_pwr_EMPSENSE(t0x2lnew0x1< 4t0x22 ta_tx_pwr_EMPSENSE(	ielse
		index =lc_t0x2er0x1< 4t0x2_dif1
	g 0x21
-l 0x22rri. ebitttielse
		index =lc_counterv>g90 u1|
nr, u32yt0x2_dif1
> 60 u1|2yt0x2_dif1
< -60)ta << < _override_oglacile(simas#(wlc_lcle__get_t < _oveio_revcolcle__get_t < 	ielse
		index =lc_t0x2er
	g 0x2#new;.t < 	ielse
		index =lc_counterv	g0;	u. ext_ln
g	< 	ielse
		index =lc_counter00);
  ainpult = 	calna_tx_pwrPERICALe_EMPBASED))

#dCTRL:<< s_to;
	tx_pwr_c 0x20))

#(wlc_lcnphy_tssi_based_pwr_ rbi_override_oixh_measladjustm_lc
	  r,_lcnphy_rx_iq_est_pub *) ar_tr	gpult = 		)ave< 0));
}

staticg);

 ifurrent_tx_pwr_idx(str,cr8 *ofdm_lc_, r8 *cck_lc_)
lert8 cck__width;.t1e_osAfeua;32_bfeuavas(ave_AfeCtrlOvr = 	m4rit0x1is_to_override(p  if (wlc_lcnphy_tssi_based_pwr &&
n, tie_bfeuavv&48aveg(pi15))ta << *ofdm_lc_ ); =8);(((ave_AfeCtrlOvr = 	m4ritv&48avegg(pi, 0) 	g	  ti>>;
	in>,1&< 14is_to_ovede(p pc_isi_basederride(_pwr_ rbicck__width );pif gey_measl_width[)

_FIRST_CCK]x1 .(coeff_xcck__width );gai
		*cck_lc_ );*ofdm_lc_ + cck__width;.text_lna << *cck_lc_ );0;	u.*ofdm_lc_ );0= 		)ave< 0));
}
dex =lc_inicerride(_rrent_tx_pwr_idx(struct 
f (tem= 2
	r< 0));
}

staticiphymeasladjustm_lc
lcnphy_rx_iq_est_pub *ptrhy_txtcms_phy *p1ains_phy2;txt brcmstribution(str_idcnntx_gasl_w(ptr,crby_distribution, f (LC_ro)q_tr brcmstribution	struct brcms_phy_lcnphy *pi_lcn = 41ainSAVEg	xnt_tx_p;
	;
	tx_pwr_ctrl = wlc_lcnphy_get_ts_to;
	tx_pwr_c 0x20))

#(wlc_lcnphy_tssi_based_pwr &&
n, tiSAVEg	xnt_tx_p0a << s>lcn
);;
	tx_pwr_c 0x2cox2))
atc_l	xnt_tx_p__get_t s_phy2 idx3ff);(s>lcn
* , x1  << 2), 1 << 2);
	moa9,48avegg(pi, 0,;(s>lcn2n a, (0x1 <n	ielse
		index = pi_lcn->lcn
)
 r,_l8)((ave_AfeCtrlOvr = 	m4r9);= (rrel_/ , x1 	)ave_AfeCtrlOvr);
}

staticlot;
	_lcnphy	wlc_llcnphy_rx_iq_est(str_ptr, u32  constrt brcms_phy *p	_lcnphy	bssi_trt(scnphy	wlc_hy_txg6 lj;tx tbl_offset)
{

	struct pg6 lvltCtr1ainpa_phy Ctr1ain) >> 0;; 

		re	ixxfffeboardflagav&4BFL_FEM_ rbpa_phy gaigx10_es(coeff_pa_phy gaigx6qq_n_len = tfo ta_tx_pwr_BL_ID))

#dCTL;bl_len = tbl_len;
32;.tbl_id = bl_ptr1x1tbl_id = tbl_id&vltCtse/* fixedn) >> 0;lvltue < 2)iPA */
	) >> 0;,a 15;32tia_gj0 << 2j;

128 2j(i a << 		re	ixxfffeboardflagav&4BFL_FEM_ rb	) >> 0;,a cnphy	wlc_[j].gmO_2sv sgai(t3) << pa_phy g9, 24n 		    u16i (cnphy	wlc_[j].pad, (016)&		    u16i (cnphy	wlc_[j].pga39, x u16) >> 0;_ga
	__ptr = t_width;
	_tx_pwr_ctrl(pi, LCGAIN_OFFSET +;jx1<	;
}

staticset;
	wlc_lcnphy_write_1 nv sgai(cnphy	wlc_[j].dacg9, 2x u16(cnphy	wlc_[j].bb_mulcg9, 200x1< th = tbl_width;
	_tx_pwr_ctrl(pi, LCIQ_OFFSET +;jx1<	;
}

staticset;
	wlc_lcnphy_write_<afectrlovrval);
	}
}

staticlot;
rf_meas_rrent_tx_pwr_idx(struct br brcmsfset)
{

	struct 	g6 lvlt,lbbmulc, rfphy Ctr1cms_phy *p18 s=lce_factorptr1x1tsain 0x2,s 0x21,l 0x22,lqQ,lqQ1,lqQ2,crhift;1y__ptr = tfo ta_tx_pwr_BL_ID))

#dCTL;bl_len = tbl_len;
32;.tbl_id = bl_ptr1x1 << 2),i>lcn
); ;ms_phy0

128 2i_phy(i a << th = tbltbl_id&bbmulcx1< th = tbl_width;
	_tx_pwr_ctrl(pi, LCIQ_OFFSET +;->lcnphnt;
}

staticset;
	wlc_lcnph_write_ nbbmulc 
	bbmulc n>,20ga
	__ptr = ttbl_id&rfphy Ctr th = tbl_width;
	_tx_pwr_ctrl(pi, LCGAIN_OFFSET +;->lcnphni;
}

staticset;
	wlc_lcnph_write_1 iqm_log10(xs << (bbmulc),c0,c& 0x21,l&qQ1te_ nqm_log10(xs << (eg(pi6 0x4,c& 0x22,l&qQ(0x106 ebitqQ10

qQ(0a <	 4t0x22 idqm_shr16( 0x22,lqQ2v- qQ1te_ n	qQ idqQ1;322 << 1) + rbi	0x21 taqm_shr16( 0x21,lqQ1v- qQ2te_ n	qQ idqQ2);
  ain 0x2 taqm_sub16( 0x21,l 0x220x106 ebitqQ >a 4) 	g	shift idqQv- 4;	u.(coeff_xshift id4v- qQe_1 nv sgai((,i>lcn
b, shift u+;(5
*  0x2) +
   (eg(pi(s=lce_factorp+ shiftv- 3))tan>,(s=lce_factorp+	         u16i shiftv- 2))ga
	__ptr = ttbl_id&vltCtr__ptr = t_width;
	_tx_pwr_ctrl(pi, LCrl(pOFFSET +;->lcnphnt;
}

staticset;
	wlc_lcnphy_write_<afectrlovrval);
	}
}

staticbu_twlt s__by_distribution(struct b), (0) << 8);

		805);

1ix1 <0), (1) << 0);

	m42fphy_r7g(pi 0x48av3n a, (0x1 <0), (1) << 0);

	m030phy_r7g(pi 0x48av3n a, (0x1 <_REG12(0) << 8);

		o14	 _r1e1	bx1 rREG11feCtrlOvr = 	m415);

	64(0x1 <0), (1) << 0);

	m4dfphy_reg39, x , -939, x x1 <), (0) << 8);

		a4a

		a4bx1 rREG11feCtrlOvr = 	m44_phy_80bx1 < 0), 0 << 0);

	mod4phy_reg39,  0x48avFDn a, (0x1 <0), (1) << 0);

	m420phy_reg39,  0x4816)&a, (0x1 <sii !e	ixxfffeboardrev&<
	m1204)<< 42), 4, 0x01);
	write_radio_reg(pi9B0, 0F00, 0F00x1 <_REG12(0) << 8);

		7d6x5, O90, i, < 0), 0 << 0);

	mo29,48avg39,  0x48av9n a, (0x1 <0), (1) << 0);

	m429p(pi,(pua, x1p(pi,e ua, x1x1 <sii a1 << 1IS(

	if (LCNREV_LT(pi1 )a << hannel != 0) {
		mo23phy_reg39,  0x48av46)&a, (0x1 <22), (0) << 8);

		a11phy_reg39,  0x481)&a, (0x1 <22), (0) << 8);

		ad4phy_reg39,  0x48avFF)&a, (0x1 <22), (0) << 8);

		656,48avg39,  0x482)&a, (0x1 <22), (0) << 8);

		a4dphy_reg(pi2 0x81)&a, (0x106 << 14, 0x01);
	write_radio_reg(piF7ph_r4	 _rx1 <in< 0)4, 0x01);
	write_radio_reg(piF1);, 3);00x1< < 0)4, 0x01);
	write_radio_reg(piF 0, 0F_ph_r900x1< < 0)4, 0x01);
	write_radio_reg(piF3);, 3);0x20x1< < 0)4, 0x01);
	write_radio_reg(piF3);, f00, 0a00x106 << 14, 0x01);
	write_radio_reg(p11F);0x2);0x20x1hnt;
}

static void gey_measl_widths__get_t 0), (1) << 0);

	m4d0,48avegg(pi,6 0x810b, (060x106afectrlovrval);
	}
}

staticrult_rrent_tx_pwr_idx(struct 
18 rult_vltue << a, ~) \
	(0 != (read_radio_reg(p05B0, 0fD x1 <), ) \
	(0 != (read_radio_reg(p004	 _rx(0x1 ), 4, 0x01);
	write_radio_reg(p1200, 010 x1 <), ) \
	(0 != (read_radio_reg(p078phy_80bx1 ), 4, 0x01);
	write_radio_reg(p129x5, O2 x1 <), ) \
	(0 != (read_radio_reg(p057ph_r0x1 << ), ) \
	(0 != (read_radio_reg(p05Bx5, O2 x1	m clean5 x1	SPINWAIT 0, 0,) \
	(io_rerult_d	}
utx_,  b,*  b0,t* 1	e0eCt, s_to;
	t) \
	(io_rerult_d	}
utx_)a << rult_vltue ); u8);< 16)C, 0x07);
	write_radio_reg(p05CIO_2 rult_vltue );rult_vltue = (r1fx1<	)pia, ~) \
	(0 != (read_radio_reg(p05B0, 0fD x1 <a, ~) \
	(0 != (read_radio_reg(p057ph_rFE1 <table(pi, < 0));
}

staticrc_ult_rrent_tx_pwr_idx(struct 
u8fdfltcrc_ult_vltCtr1ainfltcvltCtsedfltcrc_ult_vlt a 7;	usii a1 << 1IS(

	if (LCNREV_LT(pi1 )
rrdfltcrc_ult_vlt a (1_esfltcvlt;
ain(dfltcrc_ult_vlt (pi,0 u16(dfltcrc_ult_vlt (pi54
		  (dfltcrc_ult_vltbx1 rREG11feCtrlOvr = 	m933);fltcvltbx1 rREG11feCtrlOvr = 	m934);fltcvltbx1 rREG11feCtrlOvr = 	m935);fltcvltbx1 rREG11feCtrlOvr = 	m936);fltcvltbx1 rREG11feCtrlOvr = 	m937phyfltcvlt;= (r1FF))ga
	f (tem= 
	rle(pi, < 0));
}
) \
	(io_reinic_lcnphy_rx_iq_est strhy_txg6 li;trconstrt brcms_phy *pr \
	(0 !s *_phy *0 !s =yNULLga
	_phy *0 !s =y_phy *pr \
	(0 !s(io_rx1 << 2),i
); ;m_phy *0 !s[i].address3c);
_reg( 2i(i 
< ebit << 5);
	5

	if (CHSPEC_IS2G(pipwr _phy *0 !s[i].do_inicea_ rbi_REG114, 0x01);
	wr	     ((_phy *0 !s[i].address3 sav3e_f4
		  r,	ead_radDEFAULT_CORE)r	     (3ff);_phy *0 !s[i].inicea_;	u.(coe ebit_phy *0 !s[i].do_iniceg_ rbi_REG114, 0x01);
	wr	     ((_phy *0 !s[i].address3 sav3e_f4
		  r,	ead_radDEFAULT_CORE)r	     (3ff);_phy *0 !s[i].iniceg0x1 <_REG12) \
	(0 != (read_radio_reg(p03 0, 062bx1 rREG11) \
	(0 != (read_radio_reg(p033);, 190x1 <_REG12) \
	(0 != (read_radio_reg(p0900, 010 x1 <_REG12) \
	(0 != (read_radio_reg(p0100, 00qhx1 <sii a1 << 1IS(

	if (LCNREV_LT(pi1 )a <an	_REG12) \
	(0 != (read_radio_reg(p0600,0x7_n;1 i;REG12) \
	(0 != (read_radio_reg(p061);, 72n;1 i;REG12) \
	(0 != (read_radio_reg(p0620,0x7_n;1 }1 <_REG12) \
	(0 != (read_radio_reg(p01Dx5, O2 x1	_REG12) \
	(0 != (read_radio_reg(p01E);

	60x1 <0), (1) << 0);

	m4eaphy_r7g(pi 0x403b, (0x1< 0), (1) << 0);

	m4eaphy_r7g(pi3),x939, 3te_
 0), (1) << 0);

	m4eaphy_r7g(pi6 0x2, (060x1060), (1) << 0);

	m4eaphy_r7g(pi9 0x33b, 90x1060), (1) << 0);

	m4eaphy_r7g(pi,, 0x4g(pi,, _e1 rREG11feCtrlOvr = 	m4ea

		a688hx1 <sii 	ixxfffeboardflagav&4BFL_FEM_ rb0), (1) << 0);

	m4ebphy_r7g(pi 0x423b, (0x1 (coeff_0), (1) << 0);

	m4ebphy_r7g(pi 0x433b, (0x1< 0), (1) << 0);

	m4ebphy_r7g(pi6 0x0, (060x1060), (1) << 0);

	m46aphy_reggg(pi, 0x425 a, (0x1 <_override(pi);

	wlocc__tx;(0x1 <_override(prcle_ar_trsw_override(prc_cle_ar_trswsii !e	ixxfffeboardflagav&4BFL_FEM_)a << _REG12) \
	(0 != (read_radio_reg(p03 0, 06_n;1 i;REG12) \
	(0 != (read_radio_reg(p033);, 190x1 i;REG12) \
	(0 != (read_radio_reg(p039x5, 
ix1x	))table(pi, < 0));
}

staticr \
	(inic_lcnphy_rx_iq_est strhy_tx;
}
) \
	(io_reinic__t1 <table(pi, < 0));
}

staticibleinic_lcnphy_rx_iq_est strhy_txgrlcsidx;e
u8ffsebw4qq_nr brcmsfset)
{

	struct 	constrt brcmsfset)
{

	st*tct pg6 lvltCt_nfsebw4q& 0 << 5);
	40
	if (CHSPEC_IS2G(pix1 << 2),idx0 << 2idx0

dot11
statit)
{

	s_sz_LT(qq2idx(i hnt;
}

staticset;
	wlc_lcnphy_dot11
statit)
{

	s_LT(q[idx]hx1 <sii 	ixxfffeboardflagav&4BFL_FEM_BT a << th = tblfo ta_tx_pwr_BL_ID)RFSEQCtr__ptr = tbl_len;
16;se__ptr = ttbl_id&vltCtr__ptr = tbl_ptr1;322vlt a (0_ga	f_ptr = t_width;
	4phnt;
}

staticset;
	wlc_lcnphy_write_<afswsii !e	ixxfffeboardflagav&4BFL_FEM_)a << th = tblfo ta_tx_pwr_BL_ID)RFSEQCtr__ptr = tbl_len;
16;se__ptr = ttbl_id&vltCtr__ptr = tbl_ptr1;3322vlt a (5_ga	f_ptr = t_width;
	0x1<	;
}

staticset;
	wlc_lcnphy_write_1 nv sgai22_ga	f_ptr = t_width;
	1		< m
}

staticset;
	wlc_lcnphy_write_<afswsii  << 5);
	}

	if (CHSPEC_IS2G(pi-ubpi.		re	ixxfffeboardflagav&4BFL_FEM_ rb	;
}

staticlot;
	_lcnphy	wlc_l
g	< 	ir	    dot11
stati_2GHz_extPAlcnphwlc_l_LT(qhx1u.(coeff_x;
}

staticlot;
	_lcnphy	wlc_l
g	< 	ir	    dot11
stati_2GHz_cnphwlc_l_LT(qhx1u}1 <sii a1 << 1IS(

	if (LCNREV_LT(pi->pubpi.rlcslx106 ebit << 5);
	}

	if (CHSPEC_IS2G(pi-ubpi.	sgaidot11
statit)
{r_lcnphy

	s_2G_LT(2_sz *pi.		re	ixxfffeboardflagav&4BFL_EXTLNA) 	g	 tbgaidot11
statit)
{r_lcnphy

	s_extlna_2G_LT(2 *pi.t_ln
g	< tbgaidot11
statit)
{r_lcnphy

	s_2G_LT(2 *pi << 1) + rbisgaidot11
statit)
{r_lcnphy

	s_5G_LT(2_sz *pi.		re	ixxfffeboardflagav&4BFL_EXTLNA_5GHz) 	g	 tbgaidot11
statit)
{r_lcnphy

	s_extlna_5G_LT(2 *pi.t_ln
g	< tbgaidot11
statit)
{r_lcnphy

	s_5G_LT(2 *pi 106 < 2),idx0 << 2idx0

lq2idx(i hnt m
}

staticset;
	wlc_lcnphy_wb[idx]hx1 e1 <sii fixxfffeboardflagav&4BFL_FEM_ubpi.		re	ixxfffeboardflagav&4BFL_FEM_BT a << .		re	ixxfffeboardrev&<
	m1250) 	g	 tbgai_dot11
st_swhy_tsst)
{

	s_4313_bt_epa *pi.t_ln
g	< tbgai_dot11
st_swhy_tsst)
{

	s_4313_bt_epa_p25gain  << 1) + rbi	bgai_dot11
st_swhy_tsst)
{

	s_4313_epa *pi}1 ext_lna << ebit	ixxfffeboardflagav&4BFL_FEM_BT  rbi	bgai_dot11
st_swhy_tsst)
{

	s_4313_bt_ipa *pit_ln
g	<	bgai_dot11
st_swhy_tsst)
{

	s_4313;< }1 	
}

staticset;
	wlc_lcnphytb0x1.;
}

staticlot;
rf_meas_ar_trsw_override(p void papd_cox2wlc_lcnp1 <table(pi, < 0));
}

staticrT(q#(wlcba, ~inic_lcnphy_rx_iq_est strhy_txgboo fey_ts1;txt brcmstribution	struct brcms_phy_lcnphy *pi_lcn = pi_REG12) \
	(0 != (read_radio_reg(p11C);

	 _e1 rREG11feCtrlOvr = 	m43b);

	bx1 rREG11feCtrlOvr = 	m43c);

	bx1 rREG11feCtrlOvr = 	m44c);

	bx1 rREG11feCtrlOvr = 	m4e6);

	bx1 rREG11feCtrlOvr = 	m4f9);

	bx1 rREG11feCtrlOvr = 	m4b0);

	bx1 rREG11feCtrlOvr = 	mod_ph_r	bx1 rREG11feCtrlOvr = 	m4b0);

	bx1 rREG11feCtrlOvr = 	m44ex;(0x1 <), (0) << 8);

		567ph_r03 x1 <), (0) << 8);

		a4a

		a4bx1 rREG11feCtrlOvr = 	m44_phy_80bx1swsii !e	ixxfffeboardflagav&4BFL_FEM_)	< m
}

statici);

	wlc_lcnphy_set_tx;52bx1swsii 
	if (c fey_ts1;
	0x1<	 fey_ts1;
	x3ff);((	ielse
		index r if vf4
		  r, re	ielse
		index r if vcua, x1
		  r, re	ielse
		index r if gav(pi,0 0x1 i;REG12feCtrlOvr = 	m43ex; fey_ts1hx1 e1 << 0), 0 << 0);

	m634phy_reg39,  0x40xC3b, (0x1 sii fixxfffeboardflagav&4BFL_FEM_ubpi.< 0), 0 << 0);

	m634phy_reg39,  0x40xA&a, (0x1 <2rREG11feCtrlOvr = 	mo100, 01hx1 e1 << 0), 0 << 0);

	mo4_phy_r339, x , 139, x x1.< 0), 0 << 0);

	m608phy_reg39,  0x40x173b, (0x1 < 0), 0 << 0);

	m604phy_r7eg39,  0x40x3EA&a, (0x1 table(pi, < 0));
}

staticrT(2#(wlcba, ~inic_lcnphy_rx_iq_est strhy_txebit << 5);
	5

	if (CHSPEC_IS2G(pi-ubpi.2), (0) << 8);

		a16phy_reg39,  0x4803b, (0x1i.2), (0) << 8);

		a16phy_reg39, 80x4803b, 8te_<afectrlovrval);
	}
}

staticagcci0x2_inic_lcnphy_rx_iq_est strhy_txsain 0x2q_nr brcmsfset)
{

	struct 	g6 lwlc_lBuffer[2];0x tbl_oftribution	struct brcms_phy_lcnphy *pi_lcn = pit0x2 taxs (0xave_AfeCtrlOvr = 	m4d_n;1 	ielse
		index ofdmcnphidxwlc_l_width;
	( 0x2v&48avgg(pi, 0)i>>;
; 

		re	ielse
		index ofdmcnphidxwlc_l_width;> 1272ff_	ielse
		index ofdmcnphidxwlc_l_width;-= 256 <1 	ielse
		index dssscnphidxwlc_l_width;
	( 0x2v&48avgg(pi,80)i>>;8; 

		re	ielse
		index dssscnphidxwlc_l_width;> 1272ff_	ielse
		index dssscnphidxwlc_l_width;-= 256 <1 _ptr = ttbl_idwlc_lBuffer;sebl_id = bl_ptr2;.tbl_id = fo ta17;32_ptr = t_width;
	59;bl_len = tbl_len;
32;.t;
}

staticset;
	wlc_lcnph_write_1 ebit lc_lBuffer[0]
> 63)a	f_pt_lBuffer[0]
-= 128;1 	ielse
		index tr_Rlcnphyv sgai_pt_lBuffer[0]e_1 ebit lc_lBuffer[1]
> 63)a	f_pt_lBuffer[1]
-= 128;1 	ielse
		index tr_Tlcnphyv sgai_pt_lBuffer[1]= pit0x2 taxs (0x(ave_AfeCtrlOvr = 	m434)v&48avgg(pi, 0)x1 sii t0x2 > 1272ff_t0x2 -= 256 < 	ielse
		index inputwlc_l_width_db ); =8); 0x2q_< 	ielse
		index Med_Low_Gnphydb )	  (ave_AfeCtrlOvr = 	m424)v&48avgg(pi,80)i>>;8;  	ielse
		index Very_Low_Gnphydb )	  (ave_AfeCtrlOvr = 	m425)v&48avgg(pi, 0)i>>;
; 

_ptr = ttbl_idwlc_lBuffer;sebl_id = bl_ptr2;.tbl_id = fo ta_tx_pwr_BL_ID)GAIN_IDX;32_ptr = t_width;
	28;1 _len = tbl_len;
32;.t;
}

staticset;
	wlc_lcnph_write_1 	ielse
		index cnphy
dx_14clowwordgai_pt_lBuffer[0]e_ 	ielse
		index cnphy
dx_14chiwordgai_pt_lBuffer[1]= pectrlovrval);
	}
}

staticbwlcba, ~inic_lcnphy_rx_iq_est strhy_t <_override(p bleinic_tx_gawr
}

staticrT(q#(wlcba, ~inic__get_ts_toa1 << 1IS(

	if (LCNREV_LT(pi->p
ni;
}

staticse(2#(wlcba, ~inic_tx_gawr
}

staticbu_twlt s_np1 <tab< 0));
}
dex inicerride(_rrent_tx_pwr_idx(struct 
u8ffsebw4qq_nr brcmstribution	struct brcms_phy_lcnphy *pi_lcn = 4fsebw4q& 0 << 5);
	40
	if (CHSPEC_IS2G(pix1 <	ielse
		index =lc_counterv	g0;	u	ielse
		index =lc_t0x2er
	g	ielse
		index raw 0x20))

x1 <), (0) << 8);

		a4a

		80bx1 a, ~(0) << 8);

		a4a

		7_n;11 _override(pafegclkrinic_);

AFE_CLK_INIT_MODE)TXRX2X)_e1 rREG11feCtrlOvr = 	m60ax;16	 _e1 rREG11feCtrlOvr = 	m46aph25n;11 _override(p(wlcba, ~inic_tx_gaawr
}

staticr \
	(inic_ar_trswsii  << 5);
	}

	if (CHSPEC_IS2G(pi-
bi_override_oixh_nphy_tssinic__lcnphy_rx_iq_est_pub *) ar_trsw;
}
dex =_IS2G(pci);__lcnphy_rx_iq_est_pub *) ar, 	if (CHSPEC_IS2G(pix1 <bcma
chipc	(0 !ctl_maski);_&lcnpd11cnrefebusnpdrv_cc);
, ~avg= 	moix1 <bcma
chipc	(chipctl_maski);_&lcnpd11cnrefebusnpdrv_cc);
, _r0		       u_r03CDDDDD_trswsii e	ixxfffeboardflagav&4BFL_FEM_ r   u&&	;
	tx_pwr_c 0x20))

#(wlc_lcnphy_tssi_based_pwr_ rbm
}

statici);

	wlc_lcnphy_set_tx;FIXED))

#dn;11 _override(pagcci0x2_inic_tx_gaawr
}

statici0x2_adj_tx_gaaw<< 1), 1 << 1);
	mo4_phy_reg(pi,, 0x41b, (01 << 14o cleanu0(0x1i< 0), 0 << 0);

	mo4_phy_reg(pi,, 0x40b, (01 << 14_override(pi);

	wlc_lcnphy_set_tx_pwr_ctrl(pi, LCHWn;1 	ielse
		index noi

#samples ta_tx_pwrNOISE_SAMPLESdDEFAULTgawr
}

staticc0x6drx_sps__set_pwrPERICALe_pwINIT1 <table(pi, t brc_ovede(p xnt__sromcset;
rride(_rrent_tx_pwr_idx(struct 
s8  xnt_v	g0;	urlcsiq_nr brcmstribution	struct brcms_phy_lcnphy *pi_lcn = 4r brcmsssb_sprom *sprom id&lcnpd11cnrefebusnpspromtrswsii  << 5);
	}

	if (CHSPEC_IS2G(pi-a << oaincckpo;
	0x1<	g6 l_width_ofdm,l_width_mcsx1 <n	ielse
		index tr_isolation_mfo tasprom->fem.ghz2.tr_isox1 <n	ielse
		index rey_measl_width tasprom->rxpo2gx1 <n	if gepa_2g[0] tasprom->pa0b0x1<	pif gepa_2g[1] tasprom->pa0b1x1<	pif gepa_2g[2] tasprom->pa0b2x1 <n	ielse
		index r if vf tasprom->r ifsmf2gx1<n	ielse
		index r if vc tasprom->r ifsmc2gx1<n	ielse
		index r if !s =ysprom->r ifsav2gx1 <n	ielse
		index r if vfclowt0x2 ta	ielse
		index r if vfx1<n	ielse
		index r if vcclowt0x2 ta	ielse
		index r if vcx1<n	ielse
		index r if !sclowt0x2 ta	ielse
		index r if gsx1 <n	ielse
		index r if vfchight0x2 ta	ielse
		index r if vfx1<n	ielse
		index r if vcchight0x2 ta	ielse
		index r if vcx1<n	ielse
		index r if !schight0x2 ta	ielse
		index r if gsx1 <n xnt_v	gsprom->cnrewlc_li
	s[0].maxnt__2gx1<n	if geysromcmax_2ggai_xlc_;e06 < 2),i0 << 2i < rl(_BL_NUM_COEFF 2i(i a << 	lif gepa_2gclowci0x2[i] );pif gepa_2g[i]O_2s lif gepa_2gchighci0x2[i] );pif gepa_2g[i]O_2s 106 cckpo;
	sprom->cck2gpoO_2s_width_ofdm;
	sprom->ofdm2gpoO_2sebitcckpo a << 	urlcsma	wlc_lc_ISgai_xlc_;e06  < 2),i0 <)

_FIRST_CCK 2i < <)

_LAST_CCK 2i(i a << 	n	if geysromcmax_raG112g[i]
)
 r,; maxwlc_lc_ISg-i ecckpo; save)
* , x1  6 cckpo;>>
	4phnti}106  < 2),i0 <)

_FIRST_OFDM 2i < <)

_LAST_OFDM 2i(i a << 	n	if geysromcmax_raG112g[i]
)
 r,; maxwlc_lc_ISg-	     ((_width_ofdm; save)
* , x1  6 _width_ofdm;>>
	4phnti}1n  << 1) + rbiu8fopo;
	0x1 rbiopo;
	sprom->opo;e06  < 2),i0 <)

_FIRST_CCK 2i < <)

_LAST_CCK 2i(i << 	n	if geysromcmax_raG112g[i]
)i_xlc_;e06  < 2),i0 <)

_FIRST_OFDM 2i < <)

_LAST_OFDM 2i(i a << 	n	if geysromcmax_raG112g[i]
)  xnt_v-	      ((_width_ofdm; save)
* , x1  6 _width_ofdm;>>
	4phnti}1n 	_width_mcs;
	sprom->mcs2gpo[1]  (016;1n 	_width_mcs;|
	sprom->mcs2gpo[0]O_2s lielse
		index mcs20_po;
	_width_mcsx16  < 2),i0 <)

_FIRST_SISO_MCS_20_esg	 tia_r < <)

_LAST_SISO_MCS_20_2i(i a << 	n	if geysromcmax_raG112g[i]
)
 r,;  xnt_v- ((_width_mcs; save)
* , x1  6 _width_mcs;>>
	4phnti}1n  1 <n	ielse
		index raw 0x20))

 =ysprom->raw 0x20))

x1s lielse
		index measPmeas;
	sprom->meas_measx1s lielse
		index  0x20))

#slop
 =ysprom-> 0x20))

#slop
x1s lielse
		index hw iqble(en =ysprom->hw iqble(enx1s lielse
		index iqble(sw2_dis;
	sprom->iqble(sw2_disx1s lielse
		index  0x2cnrrx =ysprom-> 0x2cnrrxx1s lielse
		index  0x20))

#option;
ysprom-> 0x20))

#optionx1s lielse
		index freq_width_cnrr tasprom->freq_width_cnrrO_2sebitsprom->ant_availlc_l_bg > 1 hnt m
}
de(pant_rxdivci);__lcnphy_rx_iq_est_pub *) ar,1  6 sprom->ant_availlc_l_bgte_<afu	ielse
		index =ck_dig_filt_s16 
) -1ga
	f (tem0b);
		tab< 0));
}
io_revcolcle_rrent_tx_pwr_idx(struct 
u8fclenrstgaaw<< 1) \
	(0 != (read_radio_reg(p057ph939, 3,x939, 3te_	clenrst ); u8);< 16)C, 0x07);
	write_radio_reg(p05f);= (rf8;pi_REG12) \
	(0 != (read_radio_reg(p056,4clenrstte_	o cleanu x1	_REG12) \
	(0 != (read_radio_reg(p056,4clenrst |h_r03 x1	o cleanu x1	_REG12) \
	(0 != (read_radio_reg(p056,4clenrst |h_r07 x1	o clean30(0x1i< 0)) \
	(0 != (read_radio_reg(p057ph939, 3,x01 <tabt brc_ovede(p pc_isi_basederride(_lcnphy_rx_iq_est strhy_txebit;
	tx_pwr_c 0x20))

#(wlc_lcnphy_tssi_based_pwr_ rbf (tem0n_ove		rt_ln
g	t))
		r _tx_pwr_ctrl(pi, LCHW ==hnt m
}

statictrl = wlc_lcnphy_pwr_1 <tab< 0));
}
dex  xnmeaslseclecci wlc_lrride(_rrent_tx_pwr_idx(struct 
uainpc_lcnph;txebit;
	tx_pwr_c 0x20))

#(wlc_lcnphy_tssi_based_pwr_a << _override_oc0x6drx_sps__set_tx_pwrPERICALe_EMPBASED))

#dCTRLte_<a (coe ebit_override(p  if (wlc_lcnphy_tssi_based_pwr_a < 	lc__tx_p;
	;
	tx_pwr_ctrl = wlc_lcnphy_get_tw_override(pi);

	wlc_lcnphy_set_tx_pwr_ctrl(pi, LCOFF)= bi_override_oixnmeaslseclecci wlc_y_get_tw_override(pi);

	wlc_lcnphy_setlc_lcnphte_<afect< 0));
}
dex =_IS2G(pci);lrride(_rrent_tx_pwr_idx(strucgainC_IS2G(pict 
u8fc_ISnep;=; << 5);CHANNEL
C_IS2G(pix1 <;
}
dex =_IS2G(pc) \
	(i);__lcnphy_rx_iq_est_pub *)trucC_IS2G(pix1 <;
}
rride(pi);
=_IS2G(pctwlt s_np, 	if (CHSPEC_IS2G(pix1 <), (0) << 8);

		a4a

		a4bx1 rREG11feCtrlOvr = 	m44_phy_80bx1swr
}

staticr \
	(io_rec_ISnep_tune_4313(trucC_ISnep x1	o cleanu	e0eCt, _override(p ogghylafegpwd;__t1 <1 rREG11feCtrlOvr = 	m657phrride(pifPECfg[c_ISnep;- 1].ptci_treTs2	bx1 rREG11feCtrlOvr = 	m658phrride(pifPECfg[c_ISnep;- 1].ptci_treFactor_trswsii  << 5);CHANNEL
	if (CHSPEC_IS2G(pi;

	14-ubpi.2), (0) << 8);

		a4_phy_r339, x , 82)&a, 8)x1hnt;
}

staticlot;
	_liir_filtas_ar, n_ove

3te_<a (coe bpi.2), (0) << 8);

		a4_phy_r339, x , 81)&a, 8)x1hnt;
}

staticlot;
	_liir_filtas_ar, n_ove

2hx1 e1 <sii fixxfffeboardflagav&4BFL_FEM_hnt;
}

staticlot;
	_liir_filtas_ar, _);
, 00x1.(coeff_;
}

staticlot;
	_liir_filtas_ar, _);
, 3te_
 0), (1) << 0);

	m4ebphy_r7g(pi3),x81)&a, 3);txebit;
	tx_pwr_c  if (wlc_lcnphy_tssi_based_pwr_ bi_override_oi if s); r_ar1 <tab< 0));
}
dex detachlrride(_rrent_tx_pwr_idx(struct 
k	u8e(tcnphy *pi_lcn 1 <tabt brc_ovede(pattachlrride(_rrent_tx_pwr_idx(struct 
r brcmstribution	struct brcms_px1 <	inphy *pi_lcn ;=;kzalloc(sizeof_lcnphy_rx_iq_est_i_lcn 1, GFP_ATOMIC0x1 sii fixxhy *pi_lcn ;==yNULL_ rbf (tem0n_ove		fu	ielsehy_lcnphy *pi_lcn = pisii 
;

	(fixxfffeboardflagav&4BFL_NOPAr_a < 	l0xxhwnt_tx_p  0b);
		< t0xxhwnt_tx_pwcaplc_l  0b);
		<}1 <	inpxtalfreq  0bcma
chipc	(trl alp_clock_&lcnpd11cnrefebusnpdrv_ccn;1 	ielse
		index papd_rxGnC_tssinic;
	0x1 r

	ifi_fptr.inic;
	;
}
dex inicerride(; r

	ifi_fptr.c0x6nic;
	;
}
dex =lc_inicerride(; r

	ifi_fptr.c_IS2ec;
	;
}
dex =_IS2G(pci);lrride(; r

	ifi_fptr. xnt_seclec;
	;
}
dex  xnmeaslseclecci wlc_lrride(; r

	ifi_fptr. xiqccgecgai_override_ogvoid wiqcc; r

	ifi_fptr. xiqccsecgai_override_osvoid wiqcc; r

	ifi_fptr. xloccgecgai_override_ogvoid wlocc; r

	ifi_fptr.(CHSPloftgecgai_override_ogvoi(CHSPEloft; r

	ifi_fptr.detach;
	;
}
dex detachlrride(x1swsii !_ovede(p xnt__sromcset;
rride(_pwr_ rbf (tem0n_ove		_ts_toa1 << 1IS(

	if (LCNREV_LT(pi1i-ubpi.		re	ielse
		index  0x20))

#option;
= 3)a << 	lif hwnt_tx_p  0b);
		<  t0xxhwnt_tx_pwcaplc_l  0b);
		<	n	if g0x2nt_tx_pwcaplc_l  0n_ove		<  << 1) + rbilif hwnt_tx_p  0n_ove		<  t0xxhwnt_tx_pwcaplc_l  0n_ove		<  t0xxg0x2nt_tx_pwcaplc_l  0b);
		<	 ai}rswf (tem0b);
		tabrlovrval);
	}
}

staticsvoir_lcnph_rrent_tx_pwr_idx(strucg32 cnph)
le)uain rsw, ex_lrn_phrn_1phrn_2,l i 0, iq0,c iq1phcnph0_15);cnph16_19; 

_rsw ai(cnphv&483) << 1g9, 2x i ? 0 :	1		<ex_lrn_;
	x3ff);(cnphvn>,29);= (r01		<rn_1;
	x3ff);(cnphvn>,0);= (r0f		<rn_2;
	x3ff);(cnphvn>,4);= (r0f		< i ;
	x3ff);(cnphvn>,8);= (rf		< iq0;
	x3ff);(cnphvn>,12);= (rf		< iq1;
	x3ff);(cnphvn>,16);= (rf		
	cnph0_15;
	x3ff);((rn_1; sav3 u16((rn_1; sav3 ua, (0 |
nr, u((rn_2; sav3 ua, 4 u16((rn_2; sav3 ua, 60 |
nr, u(( i ; save)
9, x u16(( iq0; save)
9, 12))ga	cnph16_19  0biq1gaaw<< 1), 1 << 1);
	mo4dphy_reg(pi 0x4_rsw b, (0x1 < 0), 0 << 0);

	m4b1phy_reg(pi9 0xex_lrn_;b, 90x1 < 0), 0 << 0);

	m4b1phy_reg(pi10 0xex_lrn_;b, 1(0x1 < 0), 0 << 0);

	m4b6phy_reggg(pi, 0x4cnph0_15;b, (0x1 < 0), 0 << 0);

	m4b7,48avg39,  0x4cnph16_19 a, (0x1 <sii  << 5);
	}

	if (CHSPEC_IS2G(pi-a << < 0), 0 << 0);

	m4b1phy_r339, 11)phrn_139, 11);<< < 0), 0 << 0);

	m4e6phy_r339, 3)phrn_139, 3te_<afu;
}

staticr wcnphyoverridesi_base_ar, _);
1 <table(pi, g32 _override_ogvoi(eceiregpmeas_rrent_tx_pwr_idx(str,  = {*cnphy

lcn)y_txg6 l(eceiredgpmeas;
	0x1< = {max_i>lcn
); ;txg6 lcnphycure;
 q_esrrent_ttribution	struct brcms_phy_lcnphy *pi_lcn = pimax_i>lcn
);36x1 sii *cnphy

lcn >a 0) 	gcnphycure;
 rride_o23bitcnphcurey	wlc_[*cnphy

lcn]e_1 ebit-1;
= *cnphy

lcn)a << *cnphy

lcn 
	0x1<	;hi_l ( *cnphy

lcn <	gxs << max_i>lcn 
     u  u&&	((eceiredgpmeas;< 700)ta << <}
}

staticsvoir_lcnph_	wr	        u  urride_o23bitcnphcurey	wlc_	        u  u[*cnphy

lcn] x1  6(eceiredgpmeas;
	    }
}

staticmeasure_digitalgpmeas_	     	wr	      0else
	esg	 stbl_offnoi

#samples x1  6 *cnphy

lcn)00);
  ain *cnphy

lcn)--e_<a (coe bpi.}
}

staticsvoir_lcnph_	wrlcnphycure x1  (eceiredgpmeas;
	   }
}

staticmeasure_digitalgpmeas_	wr	         0else
	esg	 s   tbl_offnoi

#samples x1 }rswf (tem0(eceiredgpmeas <tabl32 _override_or_lsignalgpmeas_rrent_tx_pwr_idx(str,  = {cnphy

lcn)y_tx = {cnph;
	0x1< = {nominalgpmeas_dct p = {log_vlt,lcnphymismatch, desiret_tx_gx;->putwlmeasl_width_dc, r   u->putwlmeasldct p = {(eceiredgpmeas,l 0x2era(tee;txg6 l_measx1sg6 lmsb1phmsb2,ll_r1,ll_r2, diff1, diff2;.turlcsfreq_esrrent_ttribution	struct brcms_phy_lcnphy *pi_lcn = pi(eceiredgpmeas;
	_override_ogvoi(eceiregpmeas_cnph_cnphy

lcn)		
	cnph;
 rride_ocnphy	wlc_[cnphy

lcn]e_1 nominalgpmeas_dc asave_AfeCtrlOvr = 	m425)v>>;8; 

pmeas;
	((eceiredgpmeas;*;16);aimsb1  0nfs(pmeas);- 1;aimsb2;
	msb1 + 1;ail_r1vas139, msb1;ail_r2vas139, msb2;.tdiff1;
	(pmeas;-ll_r1);.tdiff2;
	xl_r2v- pmeas)x1 sii diff1;< diff2 
  log_vlt;
	msb1x1.(coeff_log_vlt;
	msb2x1 <log_vlt;
	log_vlt;* 3		
	cnphymismatch;
	xnominalgpmeas_dc / , v- (log_vlt)		
	desiret_tx_g a cnph + cnphymismatche_1 e>putwlmeasl_width_dc asave_AfeCtrlOvr = 	m434);= (rree_1 ebite>putwlmeasl_width_dc > 1272ff_e>putwlmeasl_width_dc -= 256 <1 ->putwlmeasldc ase>putwlmeasl_width_dc - desiret_tx_g <1 ->putwlmeasldc aff_e>putwlmeasldc + rride_ocnphy

lcnl_width_f), )ssi[cnphy

lcn]e_1 freq  0;
}
dex =_ISnel2freq  << 5);CHANNEL
	if (CHSPEC_IS2G(pi)x1 sii (freq >,2427rpwr (freq <	g2467)2ff_e>putwlmeasldc ase>putwlmeasldc - 1= pit0x2era(tee ta	ielse
		index last0))

d_ 0x2era(tee;t1 sii (t0x2era(tee -i15)
< -302ff_e>putwlmeasldc aesg	e>putwlmeasldc +1  6  (t0x2era(tee -i10v- 25)
* ,86)vn>,12);-	   7x1.(coe sii (t0x2era(tee -i15)
< 42ff_e>putwlmeasldc aesg	e>putwlmeasldc +1  6  (t0x2era(tee -i10v- 25)
* ,86)vn>,12);-	   3x1.(coeff_e>putwlmeasldc ase>putwlmeasldc +	     (((t0x2era(tee -i10v- 25)
* ,86)vn>,12)x1swr
}

staticr wcnphyoverridesi_base_ar, (0x1 <f (tem0->putwlmeasldct }
