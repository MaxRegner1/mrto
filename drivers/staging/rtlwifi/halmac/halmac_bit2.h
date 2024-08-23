/******************************************************************************
 *
 * Copyright(c) 2016  Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * wlanfae <wlanfae@realtek.com>
 * Realtek Corporation, No. 2, Innovation Road II, Hsinchu Science Park,
 * Hsinchu 300, Taiwan.
 *
 * Larry Finger <Larry.Finger@lwfinger.net>
 *
 *****************************************************************************/
#ifndef __RTL_WLAN_BITDEF_H__
#define __RTL_WLAN_BITDEF_H__

/*-------------------------Modification Log-----------------------------------
 *	Base on MAC_Register.doc SVN391
 *-------------------------Modification Log-----------------------------------
 */

/*--------------------------Include File--------------------------------------*/
/*--------------------------Include File--------------------------------------*/

/* 3 ============Programming guide Start===================== */
/*
 *	1. For all bit define, it should be prefixed by "BIT_"
 *	2. For all bit mask, it should be prefixed by "BIT_MASK_"
 *	3. For all bit shift, it should be prefixed by "BIT_SHIFT_"
 *	4. For other case, prefix is not needed
 *
 * Example:
 * #define BIT_SHIFT_MAX_TXDMA		16
 * #define BIT_MASK_MAX_TXDMA		0x7
 * #define BIT_MAX_TXDMA(x)		\
 *			(((x) & BIT_MASK_MAX_TXDMA) << BIT_SHIFT_MAX_TXDMA)
 * #define BIT_GET_MAX_TXDMA(x)		\
 *			(((x) >> BIT_SHIFT_MAX_TXDMA) & BIT_MASK_MAX_TXDMA)
 *
 */
/* 3 ============Programming guide End===================== */

#define CPU_OPT_WIDTH 0x1F

#define BIT_SHIFT_WATCH_DOG_RECORD_V1 10
#define BIT_MASK_WATCH_DOG_RECORD_V1 0x3fff
#define BIT_WATCH_DOG_RECORD_V1(x)                                             \
	(((x) & BIT_MASK_WATCH_DOG_RECORD_V1) << BIT_SHIFT_WATCH_DOG_RECORD_V1)
#define BIT_GET_WATCH_DOG_RECORD_V1(x)                                         \
	(((x) >> BIT_SHIFT_WATCH_DOG_RECORD_V1) & BIT_MASK_WATCH_DOG_RECORD_V1)

#define BIT_R_IO_TIMEOUT_FLAG_V1 BIT(9)

#define BIT_ISO_MD2PP BIT(0)

#define BIT_SHIFT_R_WMAC_IPV6_MYIPAD 0
#define BIT_MASK_R_WMAC_IPV6_MYIPAD 0xffffffffffffffffffffffffffffffffL
#define BIT_R_WMAC_IPV6_MYIPAD(x)                                              \
	(((x) & BIT_MASK_R_WMAC_IPV6_MYIPAD) << BIT_SHIFT_R_WMAC_IPV6_MYIPAD)
#define BIT_GET_R_WMAC_IPV6_MYIPAD(x)                                          \
	(((x) >> BIT_SHIFT_R_WMAC_IPV6_MYIPAD) & BIT_MASK_R_WMAC_IPV6_MYIPAD)

/* 2 REG_SDIO_TX_CTRL			(Offset 0x10250000) */

#define BIT_SHIFT_SDIO_INT_TIMEOUT 16
#define BIT_MASK_SDIO_INT_TIMEOUT 0xffff
#define BIT_SDIO_INT_TIMEOUT(x)                                                \
	(((x) & BIT_MASK_SDIO_INT_TIMEOUT) << BIT_SHIFT_SDIO_INT_TIMEOUT)
#define BIT_GET_SDIO_INT_TIMEOUT(x)                                            \
	(((x) >> BIT_SHIFT_SDIO_INT_TIMEOUT) & BIT_MASK_SDIO_INT_TIMEOUT)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_PWC_EV12V BIT(15)

/* 2 REG_SDIO_TX_CTRL			(Offset 0x10250000) */

#define BIT_IO_ERR_STATUS BIT(15)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_PWC_EV25V BIT(14)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_PA33V_EN BIT(13)
#define BIT_PA12V_EN BIT(12)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_UA33V_EN BIT(11)
#define BIT_UA12V_EN BIT(10)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_ISO_RFDIO BIT(9)

/* 2 REG_SDIO_TX_CTRL			(Offset 0x10250000) */

#define BIT_REPLY_ERRCRC_IN_DATA BIT(9)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_ISO_EB2CORE BIT(8)

/* 2 REG_SDIO_TX_CTRL			(Offset 0x10250000) */

#define BIT_EN_CMD53_OVERLAP BIT(8)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_ISO_DIOE BIT(7)

/* 2 REG_SDIO_TX_CTRL			(Offset 0x10250000) */

#define BIT_REPLY_ERR_IN_R5 BIT(7)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_ISO_WLPON2PP BIT(6)

/* 2 REG_SDIO_TX_CTRL			(Offset 0x10250000) */

#define BIT_R18A_EN BIT(6)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_ISO_IP2MAC_WA2PP BIT(5)

/* 2 REG_SDIO_TX_CTRL			(Offset 0x10250000) */

#define BIT_INIT_CMD_EN BIT(5)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_ISO_PD2CORE BIT(4)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_ISO_PA2PCIE BIT(3)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_ISO_UD2CORE BIT(2)

/* 2 REG_SDIO_TX_CTRL			(Offset 0x10250000) */

#define BIT_EN_RXDMA_MASK_INT BIT(2)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_ISO_UA2USB BIT(1)

/* 2 REG_SDIO_TX_CTRL			(Offset 0x10250000) */

#define BIT_EN_MASK_TIMER BIT(1)

/* 2 REG_SYS_ISO_CTRL			(Offset 0x0000) */

#define BIT_ISO_WD2PP BIT(0)

/* 2 REG_SDIO_TX_CTRL			(Offset 0x10250000) */

#define BIT_CMD_ERR_STOP_INT_EN BIT(0)

/* 2 REG_SYS_FUNC_EN				(Offset 0x0002) */

#define BIT_FEN_MREGEN BIT(15)
#define BIT_FEN_HWPDN BIT(14)

/* 2 REG_SYS_FUNC_EN				(Offset 0x0002) */

#define BIT_EN_25_1 BIT(13)

/* 2 REG_SYS_FUNC_EN				(Offset 0x0002) */

#define BIT_FEN_ELDR BIT(12)
#define BIT_FEN_DCORE BIT(11)
#define BIT_FEN_CPUEN BIT(10)
#define BIT_FEN_DIOE BIT(9)
#define BIT_FEN_PCIED BIT(8)
#define BIT_FEN_PPLL BIT(7)
#define BIT_FEN_PCIEA BIT(6)
#define BIT_FEN_DIO_PCIE BIT(5)
#define BIT_FEN_USBD BIT(4)
#define BIT_FEN_UPLL BIT(3)
#define BIT_FEN_USBA BIT(2)

/* 2 REG_SYS_FUNC_EN				(Offset 0x0002) */

#define BIT_FEN_BB_GLB_RSTN BIT(1)
#define BIT_FEN_BBRSTB BIT(0)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_SOP_EABM BIT(31)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_SOP_ACKF BIT(30)
#define BIT_SOP_ERCK BIT(29)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_SOP_ESWR BIT(28)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_SOP_PWMM BIT(27)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_SOP_EECK BIT(26)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_SOP_EXTL BIT(24)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_SYM_OP_RING_12M BIT(22)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_ROP_SWPR BIT(21)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_DIS_HW_LPLDM BIT(20)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_OPT_SWRST_WLMCU BIT(19)
#define BIT_RDY_SYSPWR BIT(17)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_EN_WLON BIT(16)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_APDM_HPDN BIT(15)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_AFSM_PCIE_SUS_EN BIT(12)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_AFSM_WLSUS_EN BIT(11)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_APFM_SWLPS BIT(10)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_APFM_OFFMAC BIT(9)
#define BIT_APFN_ONMAC BIT(8)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_CHIP_PDN_EN BIT(7)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_RDY_MACDIS BIT(6)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_RING_CLK_12M_EN BIT(4)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_PFM_WOWL BIT(3)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_PFM_LDKP BIT(2)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_WL_HCI_ALD BIT(1)

/* 2 REG_SYS_PW_CTRL				(Offset 0x0004) */

#define BIT_PFM_LDALL BIT(0)

/* 2 REG_SYS_CLK_CTRL			(Offset 0x0008) */

#define BIT_LDO_DUMMY BIT(15)

/* 2 REG_SYS_CLK_CTRL			(Offset 0x0008) */

#define BIT_CPU_CLK_EN BIT(14)

/* 2 REG_SYS_CLK_CTRL			(Offset 0x0008) */

#define BIT_SYMREG_CLK_EN BIT(13)

/* 2 REG_SYS_CLK_CTRL			(Offset 0x0008) */

#define BIT_HCI_CLK_EN BIT(12)

/* 2 REG_SYS_CLK_CTRL			(Offset 0x0008) */

#define BIT_MAC_CLK_EN BIT(11)
#define BIT_SEC_CLK_EN BIT(10)

/* 2 REG_SYS_CLK_CTRL			(Offset 0x0008) */

#define BIT_PHY_SSC_RSTB BIT(9)
#define BIT_EXT_32K_EN BIT(8)

/* 2 REG_SYS_CLK_CTRL			(Offset 0x0008) */

#define BIT_WL_CLK_TEST BIT(7)
#define BIT_OP_SPS_PWM_EN BIT(6)

/* 2 REG_SYS_CLK_CTRL			(Offset 0x0008) */

#define BIT_LOADER_CLK_EN BIT(5)
#define BIT_MACSLP BIT(4)
#define BIT_WAKEPAD_EN BIT(3)
#define BIT_ROMD16V_EN BIT(2)

/* 2 REG_SYS_CLK_CTRL			(Offset 0x0008) */

#define BIT_CKANA12M_EN BIT(1)

/* 2 REG_SYS_CLK_CTRL			(Offset 0x0008) */

#define BIT_CNTD16V_EN BIT(0)

/* 2 REG_SYS_EEPROM_CTRL			(Offset 0x000A) */

#define BIT_SHIFT_VPDIDX 8
#define BIT_MASK_VPDIDX 0xff
#define BIT_VPDIDX(x) (((x) & BIT_MASK_VPDIDX) << BIT_SHIFT_VPDIDX)
#define BIT_GET_VPDIDX(x) (((x) >> BIT_SHIFT_VPDIDX) & BIT_MASK_VPDIDX)

#define BIT_SHIFT_EEM1_0 6
#define BIT_MASK_EEM1_0 0x3
#define BIT_EEM1_0(x) (((x) & BIT_MASK_EEM1_0) << BIT_SHIFT_EEM1_0)
#define BIT_GET_EEM1_0(x) (((x) >> BIT_SHIFT_EEM1_0) & BIT_MASK_EEM1_0)

#define BIT_AUTOLOAD_SUS BIT(5)

/* 2 REG_SYS_EEPROM_CTRL			(Offset 0x000A) */

#define BIT_EERPOMSEL BIT(4)

/* 2 REG_SYS_EEPROM_CTRL			(Offset 0x000A) */

#define BIT_EECS_V1 BIT(3)
#define BIT_EESK_V1 BIT(2)
#define BIT_EEDI_V1 BIT(1)
#define BIT_EEDO_V1 BIT(0)

/* 2 REG_EE_VPD				(Offset 0x000C) */

#define BIT_SHIFT_VPD_DATA 0
#define BIT_MASK_VPD_DATA 0xffffffffL
#define BIT_VPD_DATA(x) (((x) & BIT_MASK_VPD_DATA) << BIT_SHIFT_VPD_DATA)
#define BIT_GET_VPD_DATA(x) (((x) >> BIT_SHIFT_VPD_DATA) & BIT_MASK_VPD_DATA)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_C2_L_BIT0 BIT(31)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_SHIFT_C1_L 29
#define BIT_MASK_C1_L 0x3
#define BIT_C1_L(x) (((x) & BIT_MASK_C1_L) << BIT_SHIFT_C1_L)
#define BIT_GET_C1_L(x) (((x) >> BIT_SHIFT_C1_L) & BIT_MASK_C1_L)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_SHIFT_REG_FREQ_L 25
#define BIT_MASK_REG_FREQ_L 0x7
#define BIT_REG_FREQ_L(x) (((x) & BIT_MASK_REG_FREQ_L) << BIT_SHIFT_REG_FREQ_L)
#define BIT_GET_REG_FREQ_L(x)                                                  \
	(((x) >> BIT_SHIFT_REG_FREQ_L) & BIT_MASK_REG_FREQ_L)

#define BIT_REG_EN_DUTY BIT(24)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_SHIFT_REG_MODE 22
#define BIT_MASK_REG_MODE 0x3
#define BIT_REG_MODE(x) (((x) & BIT_MASK_REG_MODE) << BIT_SHIFT_REG_MODE)
#define BIT_GET_REG_MODE(x) (((x) >> BIT_SHIFT_REG_MODE) & BIT_MASK_REG_MODE)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_REG_EN_SP BIT(21)
#define BIT_REG_AUTO_L BIT(20)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_SW18_SELD_BIT0 BIT(19)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_SW18_POWOCP BIT(18)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_SHIFT_OCP_L1 15
#define BIT_MASK_OCP_L1 0x7
#define BIT_OCP_L1(x) (((x) & BIT_MASK_OCP_L1) << BIT_SHIFT_OCP_L1)
#define BIT_GET_OCP_L1(x) (((x) >> BIT_SHIFT_OCP_L1) & BIT_MASK_OCP_L1)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_SHIFT_CF_L 13
#define BIT_MASK_CF_L 0x3
#define BIT_CF_L(x) (((x) & BIT_MASK_CF_L) << BIT_SHIFT_CF_L)
#define BIT_GET_CF_L(x) (((x) >> BIT_SHIFT_CF_L) & BIT_MASK_CF_L)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_SW18_FPWM BIT(11)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_SW18_SWEN BIT(9)
#define BIT_SW18_LDEN BIT(8)
#define BIT_MAC_ID_EN BIT(7)

/* 2 REG_SYS_SWR_CTRL1			(Offset 0x0010) */

#define BIT_AFE_BGEN BIT(0)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_POW_ZCD_L BIT(31)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_CRCERR_MSK BIT(31)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_AUTOZCD_L BIT(30)
#define BIT_SDIO_HSISR3_IND_MSK BIT(30)
#define BIT_SDIO_HSISR2_IND_MSK BIT(29)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_SHIFT_REG_DELAY 28
#define BIT_MASK_REG_DELAY 0x3
#define BIT_REG_DELAY(x) (((x) & BIT_MASK_REG_DELAY) << BIT_SHIFT_REG_DELAY)
#define BIT_GET_REG_DELAY(x) (((x) >> BIT_SHIFT_REG_DELAY) & BIT_MASK_REG_DELAY)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_HEISR_IND_MSK BIT(28)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_CTWEND_MSK BIT(27)
#define BIT_SDIO_ATIMEND_E_MSK BIT(26)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIIO_ATIMEND_MSK BIT(25)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_OCPINT_MSK BIT(24)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_SHIFT_V15ADJ_L1_V1 24
#define BIT_MASK_V15ADJ_L1_V1 0x7
#define BIT_V15ADJ_L1_V1(x)                                                    \
	(((x) & BIT_MASK_V15ADJ_L1_V1) << BIT_SHIFT_V15ADJ_L1_V1)
#define BIT_GET_V15ADJ_L1_V1(x)                                                \
	(((x) >> BIT_SHIFT_V15ADJ_L1_V1) & BIT_MASK_V15ADJ_L1_V1)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_PSTIMEOUT_MSK BIT(23)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_GTINT4_MSK BIT(22)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_GTINT3_MSK BIT(21)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_HSISR_IND_MSK BIT(20)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_SHIFT_VOL_L1_V1 20
#define BIT_MASK_VOL_L1_V1 0xf
#define BIT_VOL_L1_V1(x) (((x) & BIT_MASK_VOL_L1_V1) << BIT_SHIFT_VOL_L1_V1)
#define BIT_GET_VOL_L1_V1(x) (((x) >> BIT_SHIFT_VOL_L1_V1) & BIT_MASK_VOL_L1_V1)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_CPWM2_MSK BIT(19)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_CPWM1_MSK BIT(18)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_C2HCMD_INT_MSK BIT(17)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_SHIFT_IN_L1_V1 17
#define BIT_MASK_IN_L1_V1 0x7
#define BIT_IN_L1_V1(x) (((x) & BIT_MASK_IN_L1_V1) << BIT_SHIFT_IN_L1_V1)
#define BIT_GET_IN_L1_V1(x) (((x) >> BIT_SHIFT_IN_L1_V1) & BIT_MASK_IN_L1_V1)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_BCNERLY_INT_MSK BIT(16)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_SHIFT_TBOX_L1 15
#define BIT_MASK_TBOX_L1 0x3
#define BIT_TBOX_L1(x) (((x) & BIT_MASK_TBOX_L1) << BIT_SHIFT_TBOX_L1)
#define BIT_GET_TBOX_L1(x) (((x) >> BIT_SHIFT_TBOX_L1) & BIT_MASK_TBOX_L1)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_SW18_SEL BIT(13)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_SW18_SD BIT(10)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_TXBCNERR_MSK BIT(7)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_SHIFT_R3_L 7
#define BIT_MASK_R3_L 0x3
#define BIT_R3_L(x) (((x) & BIT_MASK_R3_L) << BIT_SHIFT_R3_L)
#define BIT_GET_R3_L(x) (((x) >> BIT_SHIFT_R3_L) & BIT_MASK_R3_L)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_TXBCNOK_MSK BIT(6)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_SHIFT_SW18_R2 5
#define BIT_MASK_SW18_R2 0x3
#define BIT_SW18_R2(x) (((x) & BIT_MASK_SW18_R2) << BIT_SHIFT_SW18_R2)
#define BIT_GET_SW18_R2(x) (((x) >> BIT_SHIFT_SW18_R2) & BIT_MASK_SW18_R2)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_RXFOVW_MSK BIT(5)
#define BIT_SDIO_TXFOVW_MSK BIT(4)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_SHIFT_SW18_R1 3
#define BIT_MASK_SW18_R1 0x3
#define BIT_SW18_R1(x) (((x) & BIT_MASK_SW18_R1) << BIT_SHIFT_SW18_R1)
#define BIT_GET_SW18_R1(x) (((x) >> BIT_SHIFT_SW18_R1) & BIT_MASK_SW18_R1)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_RXERR_MSK BIT(3)
#define BIT_SDIO_TXERR_MSK BIT(2)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_SDIO_AVAL_MSK BIT(1)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_SHIFT_C3_L_C3 1
#define BIT_MASK_C3_L_C3 0x3
#define BIT_C3_L_C3(x) (((x) & BIT_MASK_C3_L_C3) << BIT_SHIFT_C3_L_C3)
#define BIT_GET_C3_L_C3(x) (((x) >> BIT_SHIFT_C3_L_C3) & BIT_MASK_C3_L_C3)

/* 2 REG_SDIO_HIMR				(Offset 0x10250014) */

#define BIT_RX_REQUEST_MSK BIT(0)

/* 2 REG_SYS_SWR_CTRL2			(Offset 0x0014) */

#define BIT_C2_L_BIT1 BIT(0)

/* 2 REG_SYS_SWR_CTRL3			(Offset 0x0018) */

#define BIT_SPS18_OCP_DIS BIT(31)

/* 2 REG_SDIO_HISR				(Offset 0x10250018) */

#define BIT_SDIO_CRCERR BIT(31)

/* 2 REG_SDIO_HISR				(Offset 0x10250018) */

#define BIT_SDIO_HSISR3_IND BIT(30)
#define BIT_SDIO_HSISR2_IND BIT(29)
#define BIT_SDIO_HEISR_IND BIT(28)

/* 2 REG_SDIO_HISR				(Offset 0x10250018) */

#define BIT_SDIO_CTWEND BIT(27)
#define BIT_SDIO_ATIMEND_E BIT(26)
#define BIT_SDIO_ATIMEND BIT(25)
#define BIT_SDIO_OCPINT BIT(24)
#define BIT_SDIO_PSTIMEOUT BIT(23)
#define BIT_SDIO_GTINT4 BIT(22)
#define BIT_SDIO_GTINT3 BIT(21)
#define BIT_SDIO_HSISR_IND BIT(20)
#define BIT_SDIO_CPWM2 BIT(19)
#define BIT_SDIO_CPWM1 BIT(18)
#define BIT_SDIO_C2HCMD_INT BIT(17)

/* 2 REG_SYS_SWR_CTRL3			(Offset 0x0018) */

#define BIT_SHIFT_SPS18_OCP_TH 16
#define BIT_MASK_SPS18_OCP_TH 0x7fff
#define BIT_SPS18_OCP_TH(x)                                                    \
	(((x) & BIT_MASK_SPS18_OCP_TH) << BIT_SHIFT_SPS18_OCP_TH)
#define BIT_GET_SPS18_OCP_TH(x)                                                \
	(((x) >> BIT_SHIFT_SPS18_OCP_TH) & BIT_MASK_SPS18_OCP_TH)

/* 2 REG_SDIO_HISR				(Offset 0x10250018) */

#define BIT_SDIO_BCNERLY_INT BIT(16)
#define BIT_SDIO_TXBCNERR BIT(7)
#define BIT_SDIO_TXBCNOK BIT(6)
#define BIT_SDIO_RXFOVW BIT(5)
#define BIT_SDIO_TXFOVW BIT(4)
#define BIT_SDIO_RXERR BIT(3)
#define BIT_SDIO_TXERR BIT(2)
#define BIT_SDIO_AVAL BIT(1)

/* 2 REG_SYS_SWR_CTRL3			(Offset 0x0018) */

#define BIT_SHIFT_OCP_WINDOW 0
#define BIT_MASK_OCP_WINDOW 0xffff
#define BIT_OCP_WINDOW(x) (((x) & BIT_MASK_OCP_WINDOW) << BIT_SHIFT_OCP_WINDOW)
#define BIT_GET_OCP_WINDOW(x)                                                  \
	(((x) >> BIT_SHIFT_OCP_WINDOW) & BIT_MASK_OCP_WINDOW)

/* 2 REG_SDIO_HISR				(Offset 0x10250018) */

#define BIT_RX_REQUEST BIT(0)

/* 2 REG_RSV_CTRL				(Offset 0x001C) */

#define BIT_HREG_DBG BIT(23)

/* 2 REG_RSV_CTRL				(Offset 0x001C) */

#define BIT_WLMCUIOIF BIT(8)

/* 2 REG_RSV_CTRL				(Offset 0x001C) */

#define BIT_LOCK_ALL_EN BIT(7)

/* 2 REG_RSV_CTRL				(Offset 0x001C) */

#define BIT_R_DIS_PRST BIT(6)

/* 2 REG_RSV_CTRL				(Offset 0x001C) */

#define BIT_WLOCK_1C_B6 BIT(5)

/* 2 REG_RSV_CTRL				(Offset 0x001C) */

#define BIT_WLOCK_40 BIT(4)
#define BIT_WLOCK_08 BIT(3)
#define BIT_WLOCK_04 BIT(2)
#define BIT_WLOCK_00 BIT(1)
#define BIT_WLOCK_ALL BIT(0)

/* 2 REG_SDIO_RX_REQ_LEN			(Offset 0x1025001C) */

#define BIT_SHIFT_RX_REQ_LEN_V1 0
#define BIT_MASK_RX_REQ_LEN_V1 0x3ffff
#define BIT_RX_REQ_LEN_V1(x)                                                   \
	(((x) & BIT_MASK_RX_REQ_LEN_V1) << BIT_SHIFT_RX_REQ_LEN_V1)
#define BIT_GET_RX_REQ_LEN_V1(x)                                               \
	(((x) >> BIT_SHIFT_RX_REQ_LEN_V1) & BIT_MASK_RX_REQ_LEN_V1)

/* 2 REG_RF_CTRL				(Offset 0x001F) */

#define BIT_RF_SDMRSTB BIT(2)

/* 2 REG_RF_CTRL				(Offset 0x001F) */

#define BIT_RF_RSTB BIT(1)

/* 2 REG_RF_CTRL				(Offset 0x001F) */

#define BIT_RF_EN BIT(0)

/* 2 REG_SDIO_FREE_TXPG_SEQ_V1		(Offset 0x1025001F) */

#define BIT_SHIFT_FREE_TXPG_SEQ 0
#define BIT_MASK_FREE_TXPG_SEQ 0xff
#define BIT_FREE_TXPG_SEQ(x)                                                   \
	(((x) & BIT_MASK_FREE_TXPG_SEQ) << BIT_SHIFT_FREE_TXPG_SEQ)
#define BIT_GET_FREE_TXPG_SEQ(x)                                               \
	(((x) >> BIT_SHIFT_FREE_TXPG_SEQ) & BIT_MASK_FREE_TXPG_SEQ)

/* 2 REG_AFE_LDO_CTRL			(Offset 0x0020) */

#define BIT_SHIFT_LPLDH12_RSV 29
#define BIT_MASK_LPLDH12_RSV 0x7
#define BIT_LPLDH12_RSV(x)                                                     \
	(((x) & BIT_MASK_LPLDH12_RSV) << BIT_SHIFT_LPLDH12_RSV)
#define BIT_GET_LPLDH12_RSV(x)                                                 \
	(((x) >> BIT_SHIFT_LPLDH12_RSV) & BIT_MASK_LPLDH12_RSV)

/* 2 REG_AFE_LDO_CTRL			(Offset 0x0020) */

#define BIT_LPLDH12_SLP BIT(28)

#define BIT_SHIFT_LPLDH12_VADJ 24
#define BIT_MASK_LPLDH12_VADJ 0xf
#define BIT_LPLDH12_VADJ(x)                                                    \
	(((x) & BIT_MASK_LPLDH12_VADJ) << BIT_SHIFT_LPLDH12_VADJ)
#define BIT_GET_LPLDH12_VADJ(x)                                                \
	(((x) >> BIT_SHIFT_LPLDH12_VADJ) & BIT_MASK_LPLDH12_VADJ)

/* 2 REG_AFE_LDO_CTRL			(Offset 0x0020) */

#define BIT_LDH12_EN BIT(16)

/* 2 REG_SDIO_FREE_TXPG			(Offset 0x10250020) */

#define BIT_SHIFT_MID_FREEPG_V1 16
#define BIT_MASK_MID_FREEPG_V1 0xfff
#define BIT_MID_FREEPG_V1(x)                                                   \
	(((x) & BIT_MASK_MID_FREEPG_V1) << BIT_SHIFT_MID_FREEPG_V1)
#define BIT_GET_MID_FREEPG_V1(x)                                               \
	(((x) >> BIT_SHIFT_MID_FREEPG_V1) & BIT_MASK_MID_FREEPG_V1)

/* 2 REG_AFE_LDO_CTRL			(Offset 0x0020) */

#define BIT_WLBBOFF_BIG_PWC_EN BIT(14)
#define BIT_WLBBOFF_SMALL_PWC_EN BIT(13)
#define BIT_WLMACOFF_BIG_PWC_EN BIT(12)
#define BIT_WLPON_PWC_EN BIT(11)

/* 2 REG_AFE_LDO_CTRL			(Offset 0x0020) */

#define BIT_POW_REGU_P1 BIT(10)

/* 2 REG_AFE_LDO_CTRL			(Offset 0x0020) */

#define BIT_LDOV12W_EN BIT(8)

/* 2 REG_AFE_LDO_CTRL			(Offset 0x0020) */

#define BIT_EX_XTAL_DRV_DIGI BIT(7)
#define BIT_EX_XTAL_DRV_USB BIT(6)
#define BIT_EX_XTAL_DRV_AFE BIT(5)

/* 2 REG_AFE_LDO_CTRL			(Offset 0x0020) */

#define BIT_EX_XTAL_DRV_RF2 BIT(4)

/* 2 REG_AFE_LDO_CTRL			(Offset 0x0020) */

#define BIT_EX_XTAL_DRV_RF1 BIT(3)
#define BIT_POW_REGU_P0 BIT(2)

/* 2 REG_AFE_LDO_CTRL			(Offset 0x0020) */

#define BIT_POW_PLL_LDO BIT(0)

/* 2 REG_SDIO_FREE_TXPG			(Offset 0x10250020) */

#define BIT_SHIFT_HIQ_FREEPG_V1 0
#define BIT_MASK_HIQ_FREEPG_V1 0xfff
#define BIT_HIQ_FREEPG_V1(x)                                                   \
	(((x) & BIT_MASK_HIQ_FREEPG_V1) << BIT_SHIFT_HIQ_FREEPG_V1)
#define BIT_GET_HIQ_FREEPG_V1(x)                                               \
	(((x) >> BIT_SHIFT_HIQ_FREEPG_V1) & BIT_MASK_HIQ_FREEPG_V1)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_AGPIO_GPE BIT(31)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_SHIFT_XTAL_CAP_XI 25
#define BIT_MASK_XTAL_CAP_XI 0x3f
#define BIT_XTAL_CAP_XI(x)                                                     \
	(((x) & BIT_MASK_XTAL_CAP_XI) << BIT_SHIFT_XTAL_CAP_XI)
#define BIT_GET_XTAL_CAP_XI(x)                                                 \
	(((x) >> BIT_SHIFT_XTAL_CAP_XI) & BIT_MASK_XTAL_CAP_XI)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_SHIFT_XTAL_DRV_DIGI 23
#define BIT_MASK_XTAL_DRV_DIGI 0x3
#define BIT_XTAL_DRV_DIGI(x)                                                   \
	(((x) & BIT_MASK_XTAL_DRV_DIGI) << BIT_SHIFT_XTAL_DRV_DIGI)
#define BIT_GET_XTAL_DRV_DIGI(x)                                               \
	(((x) >> BIT_SHIFT_XTAL_DRV_DIGI) & BIT_MASK_XTAL_DRV_DIGI)

#define BIT_XTAL_DRV_USB_BIT1 BIT(22)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_SHIFT_MAC_CLK_SEL 20
#define BIT_MASK_MAC_CLK_SEL 0x3
#define BIT_MAC_CLK_SEL(x)                                                     \
	(((x) & BIT_MASK_MAC_CLK_SEL) << BIT_SHIFT_MAC_CLK_SEL)
#define BIT_GET_MAC_CLK_SEL(x)                                                 \
	(((x) >> BIT_SHIFT_MAC_CLK_SEL) & BIT_MASK_MAC_CLK_SEL)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_XTAL_DRV_USB_BIT0 BIT(19)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_SHIFT_XTAL_DRV_AFE 17
#define BIT_MASK_XTAL_DRV_AFE 0x3
#define BIT_XTAL_DRV_AFE(x)                                                    \
	(((x) & BIT_MASK_XTAL_DRV_AFE) << BIT_SHIFT_XTAL_DRV_AFE)
#define BIT_GET_XTAL_DRV_AFE(x)                                                \
	(((x) >> BIT_SHIFT_XTAL_DRV_AFE) & BIT_MASK_XTAL_DRV_AFE)

/* 2 REG_SDIO_FREE_TXPG2			(Offset 0x10250024) */

#define BIT_SHIFT_PUB_FREEPG_V1 16
#define BIT_MASK_PUB_FREEPG_V1 0xfff
#define BIT_PUB_FREEPG_V1(x)                                                   \
	(((x) & BIT_MASK_PUB_FREEPG_V1) << BIT_SHIFT_PUB_FREEPG_V1)
#define BIT_GET_PUB_FREEPG_V1(x)                                               \
	(((x) >> BIT_SHIFT_PUB_FREEPG_V1) & BIT_MASK_PUB_FREEPG_V1)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_SHIFT_XTAL_DRV_RF2 15
#define BIT_MASK_XTAL_DRV_RF2 0x3
#define BIT_XTAL_DRV_RF2(x)                                                    \
	(((x) & BIT_MASK_XTAL_DRV_RF2) << BIT_SHIFT_XTAL_DRV_RF2)
#define BIT_GET_XTAL_DRV_RF2(x)                                                \
	(((x) >> BIT_SHIFT_XTAL_DRV_RF2) & BIT_MASK_XTAL_DRV_RF2)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_SHIFT_XTAL_DRV_RF1 13
#define BIT_MASK_XTAL_DRV_RF1 0x3
#define BIT_XTAL_DRV_RF1(x)                                                    \
	(((x) & BIT_MASK_XTAL_DRV_RF1) << BIT_SHIFT_XTAL_DRV_RF1)
#define BIT_GET_XTAL_DRV_RF1(x)                                                \
	(((x) >> BIT_SHIFT_XTAL_DRV_RF1) & BIT_MASK_XTAL_DRV_RF1)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_XTAL_DELAY_DIGI BIT(12)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_XTAL_DELAY_USB BIT(11)
#define BIT_XTAL_DELAY_AFE BIT(10)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_SHIFT_XTAL_LDO_VREF 7
#define BIT_MASK_XTAL_LDO_VREF 0x7
#define BIT_XTAL_LDO_VREF(x)                                                   \
	(((x) & BIT_MASK_XTAL_LDO_VREF) << BIT_SHIFT_XTAL_LDO_VREF)
#define BIT_GET_XTAL_LDO_VREF(x)                                               \
	(((x) >> BIT_SHIFT_XTAL_LDO_VREF) & BIT_MASK_XTAL_LDO_VREF)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_XTAL_XQSEL_RF BIT(6)
#define BIT_XTAL_XQSEL BIT(5)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_SHIFT_XTAL_GMN_V2 3
#define BIT_MASK_XTAL_GMN_V2 0x3
#define BIT_XTAL_GMN_V2(x)                                                     \
	(((x) & BIT_MASK_XTAL_GMN_V2) << BIT_SHIFT_XTAL_GMN_V2)
#define BIT_GET_XTAL_GMN_V2(x)                                                 \
	(((x) >> BIT_SHIFT_XTAL_GMN_V2) & BIT_MASK_XTAL_GMN_V2)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_SHIFT_XTAL_GMP_V2 1
#define BIT_MASK_XTAL_GMP_V2 0x3
#define BIT_XTAL_GMP_V2(x)                                                     \
	(((x) & BIT_MASK_XTAL_GMP_V2) << BIT_SHIFT_XTAL_GMP_V2)
#define BIT_GET_XTAL_GMP_V2(x)                                                 \
	(((x) >> BIT_SHIFT_XTAL_GMP_V2) & BIT_MASK_XTAL_GMP_V2)

/* 2 REG_AFE_CTRL1				(Offset 0x0024) */

#define BIT_XTAL_EN BIT(0)

/* 2 REG_SDIO_FREE_TXPG2			(Offset 0x10250024) */

#define BIT_SHIFT_LOW_FREEPG_V1 0
#define BIT_MASK_LOW_FREEPG_V1 0xfff
#define BIT_LOW_FREEPG_V1(x)                                                   \
	(((x) & BIT_MASK_LOW_FREEPG_V1) << BIT_SHIFT_LOW_FREEPG_V1)
#define BIT_GET_LOW_FREEPG_V1(x)                                               \
	(((x) >> BIT_SHIFT_LOW_FREEPG_V1) & BIT_MASK_LOW_FREEPG_V1)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_SHIFT_REG_C3_V4 30
#define BIT_MASK_REG_C3_V4 0x3
#define BIT_REG_C3_V4(x) (((x) & BIT_MASK_REG_C3_V4) << BIT_SHIFT_REG_C3_V4)
#define BIT_GET_REG_C3_V4(x) (((x) >> BIT_SHIFT_REG_C3_V4) & BIT_MASK_REG_C3_V4)

#define BIT_REG_CP_BIT1 BIT(29)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_SHIFT_REG_RS_V4 26
#define BIT_MASK_REG_RS_V4 0x7
#define BIT_REG_RS_V4(x) (((x) & BIT_MASK_REG_RS_V4) << BIT_SHIFT_REG_RS_V4)
#define BIT_GET_REG_RS_V4(x) (((x) >> BIT_SHIFT_REG_RS_V4) & BIT_MASK_REG_RS_V4)

/* 2 REG_SDIO_OQT_FREE_TXPG_V1		(Offset 0x10250028) */

#define BIT_SHIFT_NOAC_OQT_FREEPG_V1 24
#define BIT_MASK_NOAC_OQT_FREEPG_V1 0xff
#define BIT_NOAC_OQT_FREEPG_V1(x)                                              \
	(((x) & BIT_MASK_NOAC_OQT_FREEPG_V1) << BIT_SHIFT_NOAC_OQT_FREEPG_V1)
#define BIT_GET_NOAC_OQT_FREEPG_V1(x)                                          \
	(((x) >> BIT_SHIFT_NOAC_OQT_FREEPG_V1) & BIT_MASK_NOAC_OQT_FREEPG_V1)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_SHIFT_REG__CS 24
#define BIT_MASK_REG__CS 0x3
#define BIT_REG__CS(x) (((x) & BIT_MASK_REG__CS) << BIT_SHIFT_REG__CS)
#define BIT_GET_REG__CS(x) (((x) >> BIT_SHIFT_REG__CS) & BIT_MASK_REG__CS)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_SHIFT_REG_CP_OFFSET 21
#define BIT_MASK_REG_CP_OFFSET 0x7
#define BIT_REG_CP_OFFSET(x)                                                   \
	(((x) & BIT_MASK_REG_CP_OFFSET) << BIT_SHIFT_REG_CP_OFFSET)
#define BIT_GET_REG_CP_OFFSET(x)                                               \
	(((x) >> BIT_SHIFT_REG_CP_OFFSET) & BIT_MASK_REG_CP_OFFSET)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_SHIFT_CP_BIAS 18
#define BIT_MASK_CP_BIAS 0x7
#define BIT_CP_BIAS(x) (((x) & BIT_MASK_CP_BIAS) << BIT_SHIFT_CP_BIAS)
#define BIT_GET_CP_BIAS(x) (((x) >> BIT_SHIFT_CP_BIAS) & BIT_MASK_CP_BIAS)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_REG_IDOUBLE_V2 BIT(17)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_EN_SYN BIT(16)

#define BIT_SHIFT_AC_OQT_FREEPG_V1 16
#define BIT_MASK_AC_OQT_FREEPG_V1 0xff
#define BIT_AC_OQT_FREEPG_V1(x)                                                \
	(((x) & BIT_MASK_AC_OQT_FREEPG_V1) << BIT_SHIFT_AC_OQT_FREEPG_V1)
#define BIT_GET_AC_OQT_FREEPG_V1(x)                                            \
	(((x) >> BIT_SHIFT_AC_OQT_FREEPG_V1) & BIT_MASK_AC_OQT_FREEPG_V1)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_SHIFT_MCCO 14
#define BIT_MASK_MCCO 0x3
#define BIT_MCCO(x) (((x) & BIT_MASK_MCCO) << BIT_SHIFT_MCCO)
#define BIT_GET_MCCO(x) (((x) >> BIT_SHIFT_MCCO) & BIT_MASK_MCCO)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_SHIFT_REG_LDO_SEL 12
#define BIT_MASK_REG_LDO_SEL 0x3
#define BIT_REG_LDO_SEL(x)                                                     \
	(((x) & BIT_MASK_REG_LDO_SEL) << BIT_SHIFT_REG_LDO_SEL)
#define BIT_GET_REG_LDO_SEL(x)                                                 \
	(((x) >> BIT_SHIFT_REG_LDO_SEL) & BIT_MASK_REG_LDO_SEL)

#define BIT_REG_KVCO_V2 BIT(10)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_AGPIO_GPO BIT(9)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_SHIFT_AGPIO_DRV 7
#define BIT_MASK_AGPIO_DRV 0x3
#define BIT_AGPIO_DRV(x) (((x) & BIT_MASK_AGPIO_DRV) << BIT_SHIFT_AGPIO_DRV)
#define BIT_GET_AGPIO_DRV(x) (((x) >> BIT_SHIFT_AGPIO_DRV) & BIT_MASK_AGPIO_DRV)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_SHIFT_XTAL_CAP_XO 1
#define BIT_MASK_XTAL_CAP_XO 0x3f
#define BIT_XTAL_CAP_XO(x)                                                     \
	(((x) & BIT_MASK_XTAL_CAP_XO) << BIT_SHIFT_XTAL_CAP_XO)
#define BIT_GET_XTAL_CAP_XO(x)                                                 \
	(((x) >> BIT_SHIFT_XTAL_CAP_XO) & BIT_MASK_XTAL_CAP_XO)

/* 2 REG_AFE_CTRL2				(Offset 0x0028) */

#define BIT_POW_PLL BIT(0)

/* 2 REG_SDIO_OQT_FREE_TXPG_V1		(Offset 0x10250028) */

#define BIT_SHIFT_EXQ_FREEPG_V1 0
#define BIT_MASK_EXQ_FREEPG_V1 0xfff
#define BIT_EXQ_FREEPG_V1(x)                                                   \
	(((x) & BIT_MASK_EXQ_FREEPG_V1) << BIT_SHIFT_EXQ_FREEPG_V1)
#define BIT_GET_EXQ_FREEPG_V1(x)                                               \
	(((x) >> BIT_SHIFT_EXQ_FREEPG_V1) & BIT_MASK_EXQ_FREEPG_V1)

/* 2 REG_AFE_CTRL3				(Offset 0x002C) */

#define BIT_SHIFT_PS 7
#define BIT_MASK_PS 0x7
#define BIT_PS(x) (((x) & BIT_MASK_PS) << BIT_SHIFT_PS)
#define BIT_GET_PS(x) (((x) >> BIT_SHIFT_PS) & BIT_MASK_PS)

/* 2 REG_AFE_CTRL3				(Offset 0x002C) */

#define BIT_PSEN BIT(6)
#define BIT_DOGENB BIT(5)

/* 2 REG_AFE_CTRL3				(Offset 0x002C) */

#define BIT_REG_MBIAS BIT(4)

/* 2 REG_AFE_CTRL3				(Offset 0x002C) */

#define BIT_SHIFT_REG_R3_V4 1
#define BIT_MASK_REG_R3_V4 0x7
#define BIT_REG_R3_V4(x) (((x) & BIT_MASK_REG_R3_V4) << BIT_SHIFT_REG_R3_V4)
#define BIT_GET_REG_R3_V4(x) (((x) >> BIT_SHIFT_REG_R3_V4) & BIT_MASK_REG_R3_V4)

/* 2 REG_AFE_CTRL3				(Offset 0x002C) */

#define BIT_REG_CP_BIT0 BIT(0)

/* 2 REG_EFUSE_CTRL				(Offset 0x0030) */

#define BIT_EF_FLAG BIT(31)

#define BIT_SHIFT_EF_PGPD 28
#define BIT_MASK_EF_PGPD 0x7
#define BIT_EF_PGPD(x) (((x) & BIT_MASK_EF_PGPD) << BIT_SHIFT_EF_PGPD)
#define BIT_GET_EF_PGPD(x) (((x) >> BIT_SHIFT_EF_PGPD) & BIT_MASK_EF_PGPD)

#define BIT_SHIFT_EF_RDT 24
#define BIT_MASK_EF_RDT 0xf
#define BIT_EF_RDT(x) (((x) & BIT_MASK_EF_RDT) << BIT_SHIFT_EF_RDT)
#define BIT_GET_EF_RDT(x) (((x) >> BIT_SHIFT_EF_RDT) & BIT_MASK_EF_RDT)

#define BIT_SHIFT_EF_PGTS 20
#define BIT_MASK_EF_PGTS 0xf
#define BIT_EF_PGTS(x) (((x) & BIT_MASK_EF_PGTS) << BIT_SHIFT_EF_PGTS)
#define BIT_GET_EF_PGTS(x) (((x) >> BIT_SHIFT_EF_PGTS) & BIT_MASK_EF_PGTS)

/* 2 REG_EFUSE_CTRL				(Offset 0x0030) */

#define BIT_EF_PDWN BIT(19)

/* 2 REG_EFUSE_CTRL				(Offset 0x0030) */

#define BIT_EF_ALDEN BIT(18)

/* 2 REG_SDIO_HTSFR_INFO			(Offset 0x10250030) */

#define BIT_SHIFT_HTSFR1 16
#define BIT_MASK_HTSFR1 0xffff
#define BIT_HTSFR1(x) (((x) & BIT_MASK_HTSFR1) << BIT_SHIFT_HTSFR1)
#define BIT_GET_HTSFR1(x) (((x) >> BIT_SHIFT_HTSFR1) & BIT_MASK_HTSFR1)

/* 2 REG_EFUSE_CTRL				(Offset 0x0030) */

#define BIT_SHIFT_EF_ADDR 8
#define BIT_MASK_EF_ADDR 0x3ff
#define BIT_EF_ADDR(x) (((x) & BIT_MASK_EF_ADDR) << BIT_SHIFT_EF_ADDR)
#define BIT_GET_EF_ADDR(x) (((x) >> BIT_SHIFT_EF_ADDR) & BIT_MASK_EF_ADDR)

#define BIT_SHIFT_EF_DATA 0
#define BIT_MASK_EF_DATA 0xff
#define BIT_EF_DATA(x) (((x) & BIT_MASK_EF_DATA) << BIT_SHIFT_EF_DATA)
#define BIT_GET_EF_DATA(x) (((x) >> BIT_SHIFT_EF_DATA) & BIT_MASK_EF_DATA)

/* 2 REG_SDIO_HTSFR_INFO			(Offset 0x10250030) */

#define BIT_SHIFT_HTSFR0 0
#define BIT_MASK_HTSFR0 0xffff
#define BIT_HTSFR0(x) (((x) & BIT_MASK_HTSFR0) << BIT_SHIFT_HTSFR0)
#define BIT_GET_HTSFR0(x) (((x) >> BIT_SHIFT_HTSFR0) & BIT_MASK_HTSFR0)

/* 2 REG_LDO_EFUSE_CTRL			(Offset 0x0034) */

#define BIT_LDOE25_EN BIT(31)

/* 2 REG_LDO_EFUSE_CTRL			(Offset 0x0034) */

#define BIT_SHIFT_LDOE25_V12ADJ_L 27
#define BIT_MASK_LDOE25_V12ADJ_L 0xf
#define BIT_LDOE25_V12ADJ_L(x)                                                 \
	(((x) & BIT_MASK_LDOE25_V12ADJ_L) << BIT_SHIFT_LDOE25_V12ADJ_L)
#define BIT_GET_LDOE25_V12ADJ_L(x)                                             \
	(((x) >> BIT_SHIFT_LDOE25_V12ADJ_L) & BIT_MASK_LDOE25_V12ADJ_L)

/* 2 REG_LDO_EFUSE_CTRL			(Offset 0x0034) */

#define BIT_EF_CRES_SEL BIT(26)

/* 2 REG_LDO_EFUSE_CTRL			(Offset 0x0034) */

#define BIT_SHIFT_EF_SCAN_START_V1 16
#define BIT_MASK_EF_SCAN_START_V1 0x3ff
#define BIT_EF_SCAN_START_V1(x)                                                \
	(((x) & BIT_MASK_EF_SCAN_START_V1) << BIT_SHIFT_EF_SCAN_START_V1)
#define BIT_GET_EF_SCAN_START_V1(x)                                            \
	(((x) >> BIT_SHIFT_EF_SCAN_START_V1) & BIT_MASK_EF_SCAN_START_V1)

/* 2 REG_LDO_EFUSE_CTRL			(Offset 0x0034) */

#define BIT_SHIFT_EF_SCAN_END 12
#define BIT_MASK_EF_SCAN_END 0xf
#define BIT_EF_SCAN_END(x)                                                     \
	(((x) & BIT_MASK_EF_SCAN_END) << BIT_SHIFT_EF_SCAN_END)
#define BIT_GET_EF_SCAN_END(x)                                                 \
	(((x) >> BIT_SHIFT_EF_SCAN_END) & BIT_MASK_EF_SCAN_END)

/* 2 REG_LDO_EFUSE_CTRL			(Offset 0x0034) */

#define BIT_EF_PD_DIS BIT(11)

/* 2 REG_LDO_EFUSE_CTRL			(Offset 0x0034) */

#define BIT_SHIFT_EF_CELL_SEL 8
#define BIT_MASK_EF_CELL_SEL 0x3
#define BIT_EF_CELL_SEL(x)                                                     \
	(((x) & BIT_MASK_EF_CELL_SEL) << BIT_SHIFT_EF_CELL_SEL)
#define BIT_GET_EF_CELL_SEL(x)                                                 \
	(((x) >> BIT_SHIFT_EF_CELL_SEL) & BIT_MASK_EF_CELL_SEL)

/* 2 REG_LDO_EFUSE_CTRL			(Offset 0x0034) */

#define BIT_EF_TRPT BIT(7)

#define BIT_SHIFT_EF_TTHD 0
#define BIT_MASK_EF_TTHD 0x7f
#define BIT_EF_TTHD(x) (((x) & BIT_MASK_EF_TTHD) << BIT_SHIFT_EF_TTHD)
#define BIT_GET_EF_TTHD(x) (((x) >> BIT_SHIFT_EF_TTHD) & BIT_MASK_EF_TTHD)

/* 2 REG_PWR_OPTION_CTRL			(Offset 0x0038) */

#define BIT_SHIFT_DBG_SEL_V1 16
#define BIT_MASK_DBG_SEL_V1 0xff
#define BIT_DBG_SEL_V1(x) (((x) & BIT_MASK_DBG_SEL_V1) << BIT_SHIFT_DBG_SEL_V1)
#define BIT_GET_DBG_SEL_V1(x)                                                  \
	(((x) >> BIT_SHIFT_DBG_SEL_V1) & BIT_MASK_DBG_SEL_V1)

/* 2 REG_PWR_OPTION_CTRL			(Offset 0x0038) */

#define BIT_SHIFT_DBG_SEL_BYTE 14
#define BIT_MASK_DBG_SEL_BYTE 0x3
#define BIT_DBG_SEL_BYTE(x)                                                    \
	(((x) & BIT_MASK_DBG_SEL_BYTE) << BIT_SHIFT_DBG_SEL_BYTE)
#define BIT_GET_DBG_SEL_BYTE(x)                                                \
	(((x) >> BIT_SHIFT_DBG_SEL_BYTE) & BIT_MASK_DBG_SEL_BYTE)

/* 2 REG_PWR_OPTION_CTRL			(Offset 0x0038) */

#define BIT_SHIFT_STD_L1_V1 12
#define BIT_MASK_STD_L1_V1 0x3
#define BIT_STD_L1_V1(x) (((x) & BIT_MASK_STD_L1_V1) << BIT_SHIFT_STD_L1_V1)
#define BIT_GET_STD_L1_V1(x) (((x) >> BIT_SHIFT_STD_L1_V1) & BIT_MASK_STD_L1_V1)

/* 2 REG_PWR_OPTION_CTRL			(Offset 0x0038) */

#define BIT_SYSON_DBG_PAD_E2 BIT(11)

/* 2 REG_PWR_OPTION_CTRL			(Offset 0x0038) */

#define BIT_SYSON_LED_PAD_E2 BIT(10)

/* 2 REG_PWR_OPTION_CTRL			(Offset 0x0038) */

#define BIT_SYSON_GPEE_PAD_E2 BIT(9)

/* 2 REG_PWR_OPTION_CTRL			(Offset 0x0038) */

#define BIT_SYSON_PCI_PAD_E2 BIT(8)

#define BIT_SHIFT_MATCH_CNT 8
#define BIT_MASK_MATCH_CNT 0xff
#define BIT_MATCH_CNT(x) (((x) & BIT_MASK_MATCH_CNT) << BIT_SHIFT_MATCH_CNT)
#define BIT_GET_MATCH_CNT(x) (((x) >> BIT_SHIFT_MATCH_CNT) & BIT_MASK_MATCH_CNT)

/* 2 REG_PWR_OPTION_CTRL			(Offset 0x0038) */

#define BIT_AUTO_SW_LDO_VOL_EN BIT(7)

/* 2 REG_PWR_OPTION_CTRL			(Offset 0x0038) */

#define BIT_SHIFT_SYSON_SPS0WWV_WT 4
#define BIT_MASK_SYSON_SPS0WWV_WT 0x3
#define BIT_SYSON_SPS0WWV_WT(x)                                                \
	(((x) & BIT_MASK_SYSON_SPS0WWV_WT) << BIT_SHIFT_SYSON_SPS0WWV_WT)
#define BIT_GET_SYSON_SPS0WWV_WT(x)                                            \
	(((x) >> BIT_SHIFT_SYSON_SPS0WWV_WT) & BIT_MASK_SYSON_SPS0WWV_WT)

/* 2 REG_PWR_OPTION_CTRL			(Offset 0x0038) */

#define BIT_SHIFT_SYSON_SPS0LDO_WT 2
#define BIT_MASK_SYSON_SPS0LDO_WT 0x3
#define BIT_SYSON_SPS0LDO_WT(x)                                                \
	(((x) & BIT_MASK_SYSON_SPS0LDO_WT) << BIT_SHIFT_SYSON_SPS0LDO_WT)
#define BIT_GET_SYSON_SPS0LDO_WT(x)                                            \
	(((x) >> BIT_SHIFT_SYSON_SPS0LDO_WT) & BIT_MASK_SYSON_SPS0LDO_WT)

/* 2 REG_PWR_OPTION_CTRL			(Offset 0x0038) */

#define BIT_SHIFT_SYSON_RCLK_SCALE 0
#define BIT_MASK_SYSON_RCLK_SCALE 0x3
#define BIT_SYSON_RCLK_SCALE(x)                                                \
	(((x) & BIT_MASK_SYSON_RCLK_SCALE) << BIT_SHIFT_SYSON_RCLK_SCALE)
#define BIT_GET_SYSON_RCLK_SCALE(x)                                            \
	(((x) >> BIT_SHIFT_SYSON_RCLK_SCALE) & BIT_MASK_SYSON_RCLK_SCALE)

/* 2 REG_SDIO_HCPWM1_V2			(Offset 0x10250038) */

#define BIT_SYS_CLK BIT(0)

/* 2 REG_CAL_TIMER				(Offset 0x003C) */

#define BIT_SHIFT_CAL_SCAL 0
#define BIT_MASK_CAL_SCAL 0xff
#define BIT_CAL_SCAL(x) (((x) & BIT_MASK_CAL_SCAL) << BIT_SHIFT_CAL_SCAL)
#define BIT_GET_CAL_SCAL(x) (((x) >> BIT_SHIFT_CAL_SCAL) & BIT_MASK_CAL_SCAL)

/* 2 REG_ACLK_MON				(Offset 0x003E) */

#define BIT_SHIFT_RCLK_MON 5
#define BIT_MASK_RCLK_MON 0x7ff
#define BIT_RCLK_MON(x) (((x) & BIT_MASK_RCLK_MON) << BIT_SHIFT_RCLK_MON)
#define BIT_GET_RCLK_MON(x) (((x) >> BIT_SHIFT_RCLK_MON) & BIT_MASK_RCLK_MON)

#define BIT_CAL_EN BIT(4)

#define BIT_SHIFT_DPSTU 2
#define BIT_MASK_DPSTU 0x3
#define BIT_DPSTU(x) (((x) & BIT_MASK_DPSTU) << BIT_SHIFT_DPSTU)
#define BIT_GET_DPSTU(x) (((x) >> BIT_SHIFT_DPSTU) & BIT_MASK_DPSTU)

#define BIT_SUS_16X BIT(1)

/* 2 REG_SDIO_INDIRECT_REG_CFG		(Offset 0x10250040) */

#define BIT_INDIRECT_REG_RDY BIT(20)

/* 2 REG_GPIO_MUXCFG				(Offset 0x0040) */

#define BIT_FSPI_EN BIT(19)

/* 2 REG_SDIO_INDIRECT_REG_CFG		(Offset 0x10250040) */

#define BIT_INDIRECT_REG_R BIT(19)

/* 2 REG_GPIO_MUXCFG				(Offset 0x0040) */

#define BIT_WL_RTS_EXT_32K_SEL BIT(18)

/* 2 REG_SDIO_INDIRECT_REG_CFG		(Offset 0x10250040) */

#define BIT_INDIRECT_REG_W BIT(18)

/* 2 REG_GPIO_MUXCFG				(Offset 0x0040) */

#define BIT_WLGP_SPI_EN BIT(16)

/* 2 REG_SDIO_INDIRECT_REG_CFG		(Offset 0x10250040) */

#define BIT_SHIFT_INDIRECT_REG_SIZE 16
#define BIT_MASK_INDIRECT_REG_SIZE 0x3
#define BIT_INDIRECT_REG_SIZE(x)                                               \
	(((x) & BIT_MASK_INDIRECT_REG_SIZE) << BIT_SHIFT_INDIRECT_REG_SIZE)
#define BIT_GET_INDIRECT_REG_SIZE(x)                                           \
	(((x) >> BIT_SHIFT_INDIRECT_REG_SIZE) & BIT_MASK_INDIRECT_REG_SIZE)

/* 2 REG_GPIO_MUXCFG				(Offset 0x0040) */

#define BIT_SIC_LBK BIT(15)
#define BIT_ENHTP BIT(14)

/* 2 REG_GPIO_MUXCFG				(Offset 0x0040) */

#define BIT_ENSIC BIT(12)
#define BIT_SIC_SWRST BIT(11)

/* 2 REG_GPIO_MUXCFG				(Offset 0x0040) */

#define BIT_PO_WIFI_PTA_PINS BIT(10)

/* 2 REG_GPIO_MUXCFG				(Offset 0x0040) */

#define BIT_PO_BT_PTA_PINS BIT(9)

/* 2 REG_GPIO_MUXCFG				(Offset 0x0040) */

#define BIT_ENUART BIT(8)

#define BIT_SHIFT_BTMODE 6
#define BIT_MASK_BTMODE 0x3
#define BIT_BTMODE(x) (((x) & BIT_MASK_BTMODE) << BIT_SHIFT_BTMODE)
#define BIT_GET_BTMODE(x) (((x) >> BIT_SHIFT_BTMODE) & BIT_MASK_BTMODE)

#define BIT_ENBT BIT(5)
#define BIT_EROM_EN BIT(4)

/* 2 REG_GPIO_MUXCFG				(Offset 0x0040) */

#define BIT_WLRFE_6_7_EN BIT(3)

/* 2 REG_GPIO_MUXCFG				(Offset 0x0040) */

#define BIT_WLRFE_4_5_EN BIT(2)

/* 2 REG_GPIO_MUXCFG				(Offset 0x0040) */

#define BIT_SHIFT_GPIOSEL 0
#define BIT_MASK_GPIOSEL 0x3
#define BIT_GPIOSEL(x) (((x) & BIT_MASK_GPIOSEL) << BIT_SHIFT_GPIOSEL)
#define BIT_GET_GPIOSEL(x) (((x) >> BIT_SHIFT_GPIOSEL) & BIT_MASK_GPIOSEL)

/* 2 REG_SDIO_INDIRECT_REG_CFG		(Offset 0x10250040) */

#define BIT_SHIFT_INDIRECT_REG_ADDR 0
#define BIT_MASK_INDIRECT_REG_ADDR 0xffff
#define BIT_INDIRECT_REG_ADDR(x)                                               \
	(((x) & BIT_MASK_INDIRECT_REG_ADDR) << BIT_SHIFT_INDIRECT_REG_ADDR)
#define BIT_GET_INDIRECT_REG_ADDR(x)                                           \
	(((x) >> BIT_SHIFT_INDIRECT_REG_ADDR) & BIT_MASK_INDIRECT_REG_ADDR)

/* 2 REG_GPIO_PIN_CTRL			(Offset 0x0044) */

#define BIT_SHIFT_GPIO_MOD_7_TO_0 24
#define BIT_MASK_GPIO_MOD_7_TO_0 0xff
#define BIT_GPIO_MOD_7_TO_0(x)                                                 \
	(((x) & BIT_MASK_GPIO_MOD_7_TO_0) << BIT_SHIFT_GPIO_MOD_7_TO_0)
#define BIT_GET_GPIO_MOD_7_TO_0(x)                                             \
	(((x) >> BIT_SHIFT_GPIO_MOD_7_TO_0) & BIT_MASK_GPIO_MOD_7_TO_0)

#define BIT_SHIFT_GPIO_IO_SEL_7_TO_0 16
#define BIT_MASK_GPIO_IO_SEL_7_TO_0 0xff
#define BIT_GPIO_IO_SEL_7_TO_0(x)                                              \
	(((x) & BIT_MASK_GPIO_IO_SEL_7_TO_0) << BIT_SHIFT_GPIO_IO_SEL_7_TO_0)
#define BIT_GET_GPIO_IO_SEL_7_TO_0(x)                                          \
	(((x) >> BIT_SHIFT_GPIO_IO_SEL_7_TO_0) & BIT_MASK_GPIO_IO_SEL_7_TO_0)

#define BIT_SHIFT_GPIO_OUT_7_TO_0 8
#define BIT_MASK_GPIO_OUT_7_TO_0 0xff
#define BIT_GPIO_OUT_7_TO_0(x)                                                 \
	(((x) & BIT_MASK_GPIO_OUT_7_TO_0) << BIT_SHIFT_GPIO_OUT_7_TO_0)
#define BIT_GET_GPIO_OUT_7_TO_0(x)                                             \
	(((x) >> BIT_SHIFT_GPIO_OUT_7_TO_0) & BIT_MASK_GPIO_OUT_7_TO_0)

#define BIT_SHIFT_GPIO_IN_7_TO_0 0
#define BIT_MASK_GPIO_IN_7_TO_0 0xff
#define BIT_GPIO_IN_7_TO_0(x)                                                  \
	(((x) & BIT_MASK_GPIO_IN_7_TO_0) << BIT_SHIFT_GPIO_IN_7_TO_0)
#define BIT_GET_GPIO_IN_7_TO_0(x)                                              \
	(((x) >> BIT_SHIFT_GPIO_IN_7_TO_0) & BIT_MASK_GPIO_IN_7_TO_0)

/* 2 REG_SDIO_INDIRECT_REG_DATA		(Offset 0x10250044) */

#define BIT_SHIFT_INDIRECT_REG_DATA 0
#define BIT_MASK_INDIRECT_REG_DATA 0xffffffffL
#define BIT_INDIRECT_REG_DATA(x)                                               \
	(((x) & BIT_MASK_INDIRECT_REG_DATA) << BIT_SHIFT_INDIRECT_REG_DATA)
#define BIT_GET_INDIRECT_REG_DATA(x)                                           \
	(((x) >> BIT_SHIFT_INDIRECT_REG_DATA) & BIT_MASK_INDIRECT_REG_DATA)

/* 2 REG_GPIO_INTM				(Offset 0x0048) */

#define BIT_SHIFT_MUXDBG_SEL 30
#define BIT_MASK_MUXDBG_SEL 0x3
#define BIT_MUXDBG_SEL(x) (((x) & BIT_MASK_MUXDBG_SEL) << BIT_SHIFT_MUXDBG_SEL)
#define BIT_GET_MUXDBG_SEL(x)                                                  \
	(((x) >> BIT_SHIFT_MUXDBG_SEL) & BIT_MASK_MUXDBG_SEL)

/* 2 REG_GPIO_INTM				(Offset 0x0048) */

#define BIT_EXTWOL_SEL BIT(17)

/* 2 REG_GPIO_INTM				(Offset 0x0048) */

#define BIT_EXTWOL_EN BIT(16)

/* 2 REG_GPIO_INTM				(Offset 0x0048) */

#define BIT_GPIOF_INT_MD BIT(15)
#define BIT_GPIOE_INT_MD BIT(14)
#define BIT_GPIOD_INT_MD BIT(13)
#define BIT_GPIOC_INT_MD BIT(12)
#define BIT_GPIOB_INT_MD BIT(11)
#define BIT_GPIOA_INT_MD BIT(10)
#define BIT_GPIO9_INT_MD BIT(9)
#define BIT_GPIO8_INT_MD BIT(8)
#define BIT_GPIO7_INT_MD BIT(7)
#define BIT_GPIO6_INT_MD BIT(6)
#define BIT_GPIO5_INT_MD BIT(5)
#define BIT_GPIO4_INT_MD BIT(4)
#define BIT_GPIO3_INT_MD BIT(3)
#define BIT_GPIO2_INT_MD BIT(2)
#define BIT_GPIO1_INT_MD BIT(1)
#define BIT_GPIO0_INT_MD BIT(0)

/* 2 REG_LED_CFG				(Offset 0x004C) */

#define BIT_GPIO3_WL_CTRL_EN BIT(27)

/* 2 REG_LED_CFG				(Offset 0x004C) */

#define BIT_LNAON_SEL_EN BIT(26)

/* 2 REG_LED_CFG				(Offset 0x004C) */

#define BIT_PAPE_SEL_EN BIT(25)

/* 2 REG_LED_CFG				(Offset 0x004C) */

#define BIT_DPDT_WLBT_SEL BIT(24)

/* 2 REG_LED_CFG				(Offset 0x004C) */

#define BIT_DPDT_SEL_EN BIT(23)

/* 2 REG_LED_CFG				(Offset 0x004C) */

#define BIT_GPIO13_14_WL_CTRL_EN BIT(22)

/* 2 REG_LED_CFG				(Offset 0x004C) */

#define BIT_LED2DIS BIT(21)

/* 2 REG_LED_CFG				(Offset 0x004C) */

#define BIT_LED2PL BIT(20)
#define BIT_LED2SV BIT(19)

#define BIT_SHIFT_LED2CM 16
#define BIT_MASK_LED2CM 0x7
#define BIT_LED2CM(x) (((x) & BIT_MASK_LED2CM) << BIT_SHIFT_LED2CM)
#define BIT_GET_LED2CM(x) (((x) >> BIT_SHIFT_LED2CM) & BIT_MASK_LED2CM)

#define BIT_LED1DIS BIT(15)
#define BIT_LED1PL BIT(12)
#define BIT_LED1SV BIT(11)

#define BIT_SHIFT_LED1CM 8
#define BIT_MASK_LED1CM 0x7
#define BIT_LED1CM(x) (((x) & BIT_MASK_LED1CM) << BIT_SHIFT_LED1CM)
#define BIT_GET_LED1CM(x) (((x) >> BIT_SHIFT_LED1CM) & BIT_MASK_LED1CM)

#define BIT_LED0DIS BIT(7)

/* 2 REG_LED_CFG				(Offset 0x004C) */

#define BIT_SHIFT_AFE_LDO_SWR_CHECK 5
#define BIT_MASK_AFE_LDO_SWR_CHECK 0x3
#define BIT_AFE_LDO_SWR_CHECK(x)                                               \
	(((x) & BIT_MASK_AFE_LDO_SWR_CHECK) << BIT_SHIFT_AFE_LDO_SWR_CHECK)
#define BIT_GET_AFE_LDO_SWR_CHECK(x)                                           \
	(((x) >> BIT_SHIFT_AFE_LDO_SWR_CHECK) & BIT_MASK_AFE_LDO_SWR_CHECK)

/* 2 REG_LED_CFG				(Offset 0x004C) */

#define BIT_LED0PL BIT(4)
#define BIT_LED0SV BIT(3)

#define BIT_SHIFT_LED0CM 0
#define BIT_MASK_LED0CM 0x7
#define BIT_LED0CM(x) (((x) & BIT_MASK_LED0CM) << BIT_SHIFT_LED0CM)
#define BIT_GET_LED0CM(x) (((x) >> BIT_SHIFT_LED0CM) & BIT_MASK_LED0CM)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_PDNINT_EN BIT(31)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_NFC_INT_PAD_EN BIT(30)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_SPS_OCP_INT_EN BIT(29)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_PWMERR_INT_EN BIT(28)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIOF_INT_EN BIT(27)
#define BIT_FS_GPIOE_INT_EN BIT(26)
#define BIT_FS_GPIOD_INT_EN BIT(25)
#define BIT_FS_GPIOC_INT_EN BIT(24)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIOB_INT_EN BIT(23)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIOA_INT_EN BIT(22)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIO9_INT_EN BIT(21)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIO8_INT_EN BIT(20)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIO7_INT_EN BIT(19)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIO6_INT_EN BIT(18)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIO5_INT_EN BIT(17)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIO4_INT_EN BIT(16)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIO3_INT_EN BIT(15)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIO2_INT_EN BIT(14)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIO1_INT_EN BIT(13)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_GPIO0_INT_EN BIT(12)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_HCI_SUS_EN BIT(11)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_HCI_RES_EN BIT(10)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_HCI_RESET_EN BIT(9)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_BTON_STS_UPDATE_MSK_EN BIT(7)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_ACT2RECOVERY_INT_EN_V1 BIT(6)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_GEN1GEN2_SWITCH BIT(5)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_HCI_TXDMA_REQ_HIMR BIT(4)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_32K_LEAVE_SETTING_MAK BIT(3)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_32K_ENTER_SETTING_MAK BIT(2)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_USB_LPMRSM_MSK BIT(1)

/* 2 REG_FSIMR				(Offset 0x0050) */

#define BIT_FS_USB_LPMINT_MSK BIT(0)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_PDNINT BIT(31)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_SPS_OCP_INT BIT(29)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_PWMERR_INT BIT(28)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIOF_INT BIT(27)
#define BIT_FS_GPIOE_INT BIT(26)
#define BIT_FS_GPIOD_INT BIT(25)
#define BIT_FS_GPIOC_INT BIT(24)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIOB_INT BIT(23)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIOA_INT BIT(22)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIO9_INT BIT(21)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIO8_INT BIT(20)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIO7_INT BIT(19)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIO6_INT BIT(18)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIO5_INT BIT(17)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIO4_INT BIT(16)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIO3_INT BIT(15)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIO2_INT BIT(14)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIO1_INT BIT(13)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_GPIO0_INT BIT(12)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_HCI_SUS_INT BIT(11)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_HCI_RES_INT BIT(10)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_HCI_RESET_INT BIT(9)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_ACT2RECOVERY BIT(6)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_HCI_TXDMA_REQ_HISR BIT(4)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_32K_LEAVE_SETTING_INT BIT(3)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_32K_ENTER_SETTING_INT BIT(2)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_USB_LPMRSM_INT BIT(1)

/* 2 REG_FSISR				(Offset 0x0054) */

#define BIT_FS_USB_LPMINT_INT BIT(0)

/* 2 REG_HSIMR				(Offset 0x0058) */

#define BIT_GPIOF_INT_EN BIT(31)
#define BIT_GPIOE_INT_EN BIT(30)
#define BIT_GPIOD_INT_EN BIT(29)
#define BIT_GPIOC_INT_EN BIT(28)
#define BIT_GPIOB_INT_EN BIT(27)
#define BIT_GPIOA_INT_EN BIT(26)
#define BIT_GPIO9_INT_EN BIT(25)
#define BIT_GPIO8_INT_EN BIT(24)
#define BIT_GPIO7_INT_EN BIT(23)
#define BIT_GPIO6_INT_EN BIT(22)
#define BIT_GPIO5_INT_EN BIT(21)
#define BIT_GPIO4_INT_EN BIT(20)
#define BIT_GPIO3_INT_EN BIT(19)

/* 2 REG_HSIMR				(Offset 0x0058) */

#define BIT_GPIO1_INT_EN BIT(17)
#define BIT_GPIO0_INT_EN BIT(16)

/* 2 REG_HSIMR				(Offset 0x0058) */

#define BIT_GPIO2_INT_EN_V1 BIT(16)

/* 2 REG_HSIMR				(Offset 0x0058) */

#define BIT_PDNINT_EN BIT(7)

/* 2 REG_HSIMR				(Offset 0x0058) */

#define BIT_RON_INT_EN BIT(6)

/* 2 REG_HSIMR				(Offset 0x0058) */

#define BIT_SPS_OCP_INT_EN BIT(5)

/* 2 REG_HSIMR				(Offset 0x0058) */

#define BIT_GPIO15_0_INT_EN BIT(0)

/* 2 REG_HSISR				(Offset 0x005C) */

#define BIT_GPIOF_INT BIT(31)
#define BIT_GPIOE_INT BIT(30)
#define BIT_GPIOD_INT BIT(29)
#define BIT_GPIOC_INT BIT(28)
#define BIT_GPIOB_INT BIT(27)
#define BIT_GPIOA_INT BIT(26)
#define BIT_GPIO9_INT BIT(25)
#define BIT_GPIO8_INT BIT(24)
#define BIT_GPIO7_INT BIT(23)

/* 2 REG_HSISR				(Offset 0x005C) */

#define BIT_GPIO6_INT BIT(22)
#define BIT_GPIO5_INT BIT(21)
#define BIT_GPIO4_INT BIT(20)
#define BIT_GPIO3_INT BIT(19)

/* 2 REG_HSISR				(Offset 0x005C) */

#define BIT_GPIO1_INT BIT(17)
#define BIT_GPIO0_INT BIT(16)

/* 2 REG_HSISR				(Offset 0x005C) */

#define BIT_GPIO2_INT_V1 BIT(16)

/* 2 REG_HSISR				(Offset 0x005C) */

#define BIT_PDNINT BIT(7)

/* 2 REG_HSISR				(Offset 0x005C) */

#define BIT_RON_INT BIT(6)

/* 2 REG_HSISR				(Offset 0x005C) */

#define BIT_SPS_OCP_INT BIT(5)

/* 2 REG_HSISR				(Offset 0x005C) */

#define BIT_GPIO15_0_INT BIT(0)
#define BIT_MCUFWDL_EN BIT(0)

/* 2 REG_GPIO_EXT_CTRL			(Offset 0x0060) */

#define BIT_SHIFT_GPIO_MOD_15_TO_8 24
#define BIT_MASK_GPIO_MOD_15_TO_8 0xff
#define BIT_GPIO_MOD_15_TO_8(x)                                                \
	(((x) & BIT_MASK_GPIO_MOD_15_TO_8) << BIT_SHIFT_GPIO_MOD_15_TO_8)
#define BIT_GET_GPIO_MOD_15_TO_8(x)                                            \
	(((x) >> BIT_SHIFT_GPIO_MOD_15_TO_8) & BIT_MASK_GPIO_MOD_15_TO_8)

#define BIT_SHIFT_GPIO_IO_SEL_15_TO_8 16
#define BIT_MASK_GPIO_IO_SEL_15_TO_8 0xff
#define BIT_GPIO_IO_SEL_15_TO_8(x)                                             \
	(((x) & BIT_MASK_GPIO_IO_SEL_15_TO_8) << BIT_SHIFT_GPIO_IO_SEL_15_TO_8)
#define BIT_GET_GPIO_IO_SEL_15_TO_8(x)                                         \
	(((x) >> BIT_SHIFT_GPIO_IO_SEL_15_TO_8) & BIT_MASK_GPIO_IO_SEL_15_TO_8)

#define BIT_SHIFT_GPIO_OUT_15_TO_8 8
#define BIT_MASK_GPIO_OUT_15_TO_8 0xff
#define BIT_GPIO_OUT_15_TO_8(x)                                                \
	(((x) & BIT_MASK_GPIO_OUT_15_TO_8) << BIT_SHIFT_GPIO_OUT_15_TO_8)
#define BIT_GET_GPIO_OUT_15_TO_8(x)                                            \
	(((x) >> BIT_SHIFT_GPIO_OUT_15_TO_8) & BIT_MASK_GPIO_OUT_15_TO_8)

#define BIT_SHIFT_GPIO_IN_15_TO_8 0
#define BIT_MASK_GPIO_IN_15_TO_8 0xff
#define BIT_GPIO_IN_15_TO_8(x)                                                 \
	(((x) & BIT_MASK_GPIO_IN_15_TO_8) << BIT_SHIFT_GPIO_IN_15_TO_8)
#define BIT_GET_GPIO_IN_15_TO_8(x)                                             \
	(((x) >> BIT_SHIFT_GPIO_IN_15_TO_8) & BIT_MASK_GPIO_IN_15_TO_8)

/* 2 REG_SDIO_H2C				(Offset 0x10250060) */

#define BIT_SHIFT_SDIO_H2C_MSG 0
#define BIT_MASK_SDIO_H2C_MSG 0xffffffffL
#define BIT_SDIO_H2C_MSG(x)                                                    \
	(((x) & BIT_MASK_SDIO_H2C_MSG) << BIT_SHIFT_SDIO_H2C_MSG)
#define BIT_GET_SDIO_H2C_MSG(x)                                                \
	(((x) >> BIT_SHIFT_SDIO_H2C_MSG) & BIT_MASK_SDIO_H2C_MSG)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_PAPE_WLBT_SEL BIT(29)
#define BIT_LNAON_WLBT_SEL BIT(28)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_BTGP_GPG3_FEN BIT(26)
#define BIT_BTGP_GPG2_FEN BIT(25)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_BTGP_JTAG_EN BIT(24)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_XTAL_CLK_EXTARNAL_EN BIT(23)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_BTGP_UART0_EN BIT(22)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_BTGP_UART1_EN BIT(21)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_BTGP_SPI_EN BIT(20)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_BTGP_GPIO_E2 BIT(19)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_BTGP_GPIO_EN BIT(18)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_SHIFT_BTGP_GPIO_SL 16
#define BIT_MASK_BTGP_GPIO_SL 0x3
#define BIT_BTGP_GPIO_SL(x)                                                    \
	(((x) & BIT_MASK_BTGP_GPIO_SL) << BIT_SHIFT_BTGP_GPIO_SL)
#define BIT_GET_BTGP_GPIO_SL(x)                                                \
	(((x) >> BIT_SHIFT_BTGP_GPIO_SL) & BIT_MASK_BTGP_GPIO_SL)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_PAD_SDIO_SR BIT(14)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_GPIO14_OUTPUT_PL BIT(13)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_HOST_WAKE_PAD_PULL_EN BIT(12)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_HOST_WAKE_PAD_SL BIT(11)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_PAD_LNAON_SR BIT(10)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_PAD_LNAON_E2 BIT(9)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_SW_LNAON_G_SEL_DATA BIT(8)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_SW_LNAON_A_SEL_DATA BIT(7)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_PAD_PAPE_SR BIT(6)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_PAD_PAPE_E2 BIT(5)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_SW_PAPE_G_SEL_DATA BIT(4)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_SW_PAPE_A_SEL_DATA BIT(3)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_PAD_DPDT_SR BIT(2)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_PAD_DPDT_PAD_E2 BIT(1)

/* 2 REG_PAD_CTRL1				(Offset 0x0064) */

#define BIT_SW_DPDT_SEL_DATA BIT(0)

/* 2 REG_SDIO_C2H				(Offset 0x10250064) */

#define BIT_SHIFT_SDIO_C2H_MSG 0
#define BIT_MASK_SDIO_C2H_MSG 0xffffffffL
#define BIT_SDIO_C2H_MSG(x)                                                    \
	(((x) & BIT_MASK_SDIO_C2H_MSG) << BIT_SHIFT_SDIO_C2H_MSG)
#define BIT_GET_SDIO_C2H_MSG(x)                                                \
	(((x) >> BIT_SHIFT_SDIO_C2H_MSG) & BIT_MASK_SDIO_C2H_MSG)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_ISO_BD2PP BIT(31)
#define BIT_LDOV12B_EN BIT(30)
#define BIT_CKEN_BTGPS BIT(29)
#define BIT_FEN_BTGPS BIT(28)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_MULRW BIT(27)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_BTCPU_BOOTSEL BIT(27)
#define BIT_SPI_SPEEDUP BIT(26)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_DEVWAKE_PAD_TYPE_SEL BIT(24)
#define BIT_CLKREQ_PAD_TYPE_SEL BIT(23)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_EN_CPL_TIMEOUT_PS BIT(22)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_ISO_BTPON2PP BIT(22)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_REG_TXDMA_FAIL_PS BIT(21)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_EN_HWENTR_L1 BIT(19)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_BT_HWROF_EN BIT(19)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_EN_ADV_CLKGATE BIT(18)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_BT_FUNC_EN BIT(18)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_BT_HWPDN_SL BIT(17)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_BT_DISN_EN BIT(16)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_BT_PDN_PULL_EN BIT(15)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_WL_PDN_PULL_EN BIT(14)
#define BIT_EXTERNAL_REQUEST_PL BIT(13)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_GPIO0_2_3_PULL_LOW_EN BIT(12)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_ISO_BA2PP BIT(11)
#define BIT_BT_AFE_LDO_EN BIT(10)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_BT_AFE_PLL_EN BIT(9)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_BT_DIG_CLK_EN BIT(8)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_WL_DRV_EXIST_IDX BIT(5)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_DOP_EHPAD BIT(4)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_WL_HWROF_EN BIT(3)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_WL_FUNC_EN BIT(2)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_WL_HWPDN_SL BIT(1)
#define BIT_WL_HWPDN_EN BIT(0)

/* 2 REG_SDM_DEBUG				(Offset 0x006C) */

#define BIT_SHIFT_WLCLK_PHASE 0
#define BIT_MASK_WLCLK_PHASE 0x1f
#define BIT_WLCLK_PHASE(x)                                                     \
	(((x) & BIT_MASK_WLCLK_PHASE) << BIT_SHIFT_WLCLK_PHASE)
#define BIT_GET_WLCLK_PHASE(x)                                                 \
	(((x) >> BIT_SHIFT_WLCLK_PHASE) & BIT_MASK_WLCLK_PHASE)

/* 2 REG_SYS_SDIO_CTRL			(Offset 0x0070) */

#define BIT_DBG_GNT_WL_BT BIT(27)

/* 2 REG_SYS_SDIO_CTRL			(Offset 0x0070) */

#define BIT_LTE_MUX_CTRL_PATH BIT(26)

/* 2 REG_SYS_SDIO_CTRL			(Offset 0x0070) */

#define BIT_LTE_COEX_UART BIT(25)

/* 2 REG_SYS_SDIO_CTRL			(Offset 0x0070) */

#define BIT_3W_LTE_WL_GPIO BIT(24)

/* 2 REG_SYS_SDIO_CTRL			(Offset 0x0070) */

#define BIT_SDIO_INT_POLARITY BIT(19)
#define BIT_SDIO_INT BIT(18)

/* 2 REG_SYS_SDIO_CTRL			(Offset 0x0070) */

#define BIT_SDIO_OFF_EN BIT(17)

/* 2 REG_SYS_SDIO_CTRL			(Offset 0x0070) */

#define BIT_SDIO_ON_EN BIT(16)

/* 2 REG_SYS_SDIO_CTRL			(Offset 0x0070) */

#define BIT_PCIE_WAIT_TIMEOUT_EVENT BIT(10)
#define BIT_PCIE_WAIT_TIME BIT(9)

/* 2 REG_SYS_SDIO_CTRL			(Offset 0x0070) */

#define BIT_MPCIE_REFCLK_XTAL_SEL BIT(8)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_SHIFT_TSFT_SEL 29
#define BIT_MASK_TSFT_SEL 0x7
#define BIT_TSFT_SEL(x) (((x) & BIT_MASK_TSFT_SEL) << BIT_SHIFT_TSFT_SEL)
#define BIT_GET_TSFT_SEL(x) (((x) >> BIT_SHIFT_TSFT_SEL) & BIT_MASK_TSFT_SEL)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_SHIFT_RPWM 24
#define BIT_MASK_RPWM 0xff
#define BIT_RPWM(x) (((x) & BIT_MASK_RPWM) << BIT_SHIFT_RPWM)
#define BIT_GET_RPWM(x) (((x) >> BIT_SHIFT_RPWM) & BIT_MASK_RPWM)

#define BIT_ROM_DLEN BIT(19)

#define BIT_SHIFT_ROM_PGE 16
#define BIT_MASK_ROM_PGE 0x7
#define BIT_ROM_PGE(x) (((x) & BIT_MASK_ROM_PGE) << BIT_SHIFT_ROM_PGE)
#define BIT_GET_ROM_PGE(x) (((x) >> BIT_SHIFT_ROM_PGE) & BIT_MASK_ROM_PGE)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_USB_HOST_PWR_OFF_EN BIT(12)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_SYM_LPS_BLOCK_EN BIT(11)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_USB_LPM_ACT_EN BIT(10)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_USB_LPM_NY BIT(9)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_USB_SUS_DIS BIT(8)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_SHIFT_SDIO_PAD_E 5
#define BIT_MASK_SDIO_PAD_E 0x7
#define BIT_SDIO_PAD_E(x) (((x) & BIT_MASK_SDIO_PAD_E) << BIT_SHIFT_SDIO_PAD_E)
#define BIT_GET_SDIO_PAD_E(x)                                                  \
	(((x) >> BIT_SHIFT_SDIO_PAD_E) & BIT_MASK_SDIO_PAD_E)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_USB_LPPLL_EN BIT(4)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_ROP_SW15 BIT(2)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_PCI_CKRDY_OPT BIT(1)

/* 2 REG_HCI_OPT_CTRL			(Offset 0x0074) */

#define BIT_PCI_VAUX_EN BIT(0)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_ZCD_HW_AUTO_EN BIT(27)
#define BIT_ZCD_REGSEL BIT(26)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_SHIFT_AUTO_ZCD_IN_CODE 21
#define BIT_MASK_AUTO_ZCD_IN_CODE 0x1f
#define BIT_AUTO_ZCD_IN_CODE(x)                                                \
	(((x) & BIT_MASK_AUTO_ZCD_IN_CODE) << BIT_SHIFT_AUTO_ZCD_IN_CODE)
#define BIT_GET_AUTO_ZCD_IN_CODE(x)                                            \
	(((x) >> BIT_SHIFT_AUTO_ZCD_IN_CODE) & BIT_MASK_AUTO_ZCD_IN_CODE)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_SHIFT_ZCD_CODE_IN_L 16
#define BIT_MASK_ZCD_CODE_IN_L 0x1f
#define BIT_ZCD_CODE_IN_L(x)                                                   \
	(((x) & BIT_MASK_ZCD_CODE_IN_L) << BIT_SHIFT_ZCD_CODE_IN_L)
#define BIT_GET_ZCD_CODE_IN_L(x)                                               \
	(((x) >> BIT_SHIFT_ZCD_CODE_IN_L) & BIT_MASK_ZCD_CODE_IN_L)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_SHIFT_LDO_HV5_DUMMY 14
#define BIT_MASK_LDO_HV5_DUMMY 0x3
#define BIT_LDO_HV5_DUMMY(x)                                                   \
	(((x) & BIT_MASK_LDO_HV5_DUMMY) << BIT_SHIFT_LDO_HV5_DUMMY)
#define BIT_GET_LDO_HV5_DUMMY(x)                                               \
	(((x) >> BIT_SHIFT_LDO_HV5_DUMMY) & BIT_MASK_LDO_HV5_DUMMY)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_SHIFT_REG_VTUNE33_BIT0_TO_BIT1 12
#define BIT_MASK_REG_VTUNE33_BIT0_TO_BIT1 0x3
#define BIT_REG_VTUNE33_BIT0_TO_BIT1(x)                                        \
	(((x) & BIT_MASK_REG_VTUNE33_BIT0_TO_BIT1)                             \
	 << BIT_SHIFT_REG_VTUNE33_BIT0_TO_BIT1)
#define BIT_GET_REG_VTUNE33_BIT0_TO_BIT1(x)                                    \
	(((x) >> BIT_SHIFT_REG_VTUNE33_BIT0_TO_BIT1) &                         \
	 BIT_MASK_REG_VTUNE33_BIT0_TO_BIT1)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_SHIFT_REG_STANDBY33_BIT0_TO_BIT1 10
#define BIT_MASK_REG_STANDBY33_BIT0_TO_BIT1 0x3
#define BIT_REG_STANDBY33_BIT0_TO_BIT1(x)                                      \
	(((x) & BIT_MASK_REG_STANDBY33_BIT0_TO_BIT1)                           \
	 << BIT_SHIFT_REG_STANDBY33_BIT0_TO_BIT1)
#define BIT_GET_REG_STANDBY33_BIT0_TO_BIT1(x)                                  \
	(((x) >> BIT_SHIFT_REG_STANDBY33_BIT0_TO_BIT1) &                       \
	 BIT_MASK_REG_STANDBY33_BIT0_TO_BIT1)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_SHIFT_REG_LOAD33_BIT0_TO_BIT1 8
#define BIT_MASK_REG_LOAD33_BIT0_TO_BIT1 0x3
#define BIT_REG_LOAD33_BIT0_TO_BIT1(x)                                         \
	(((x) & BIT_MASK_REG_LOAD33_BIT0_TO_BIT1)                              \
	 << BIT_SHIFT_REG_LOAD33_BIT0_TO_BIT1)
#define BIT_GET_REG_LOAD33_BIT0_TO_BIT1(x)                                     \
	(((x) >> BIT_SHIFT_REG_LOAD33_BIT0_TO_BIT1) &                          \
	 BIT_MASK_REG_LOAD33_BIT0_TO_BIT1)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_REG_BYPASS_L BIT(7)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_REG_LDOF_L BIT(6)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_REG_TYPE_L_V1 BIT(5)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_ARENB_L BIT(3)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_SHIFT_CFC_L 1
#define BIT_MASK_CFC_L 0x3
#define BIT_CFC_L(x) (((x) & BIT_MASK_CFC_L) << BIT_SHIFT_CFC_L)
#define BIT_GET_CFC_L(x) (((x) >> BIT_SHIFT_CFC_L) & BIT_MASK_CFC_L)

/* 2 REG_LDO_SWR_CTRL			(Offset 0x007C) */

#define BIT_REG_OCPS_L_V1 BIT(0)

/* 2 REG_MCUFW_CTRL				(Offset 0x0080) */

#define BIT_ANA_PORT_EN BIT(22)
#define BIT_MAC_PORT_EN BIT(21)
#define BIT_BOOT_FSPI_EN BIT(20)
#define BIT_FW_INIT_RDY BIT(15)
#define BIT_FW_DW_RDY BIT(14)

/* 2 REG_MCUFW_CTRL				(Offset 0x0080) */

#define BIT_SHIFT_CPU_CLK_SEL 12
#define BIT_MASK_CPU_CLK_SEL 0x3
#define BIT_CPU_CLK_SEL(x)                                                     \
	(((x) & BIT_MASK_CPU_CLK_SEL) << BIT_SHIFT_CPU_CLK_SEL)
#define BIT_GET_CPU_CLK_SEL(x)                                                 \
	(((x) >> BIT_SHIFT_CPU_CLK_SEL) & BIT_MASK_CPU_CLK_SEL)

/* 2 REG_MCUFW_CTRL				(Offset 0x0080) */

#define BIT_CCLK_CHG_MASK BIT(11)

/* 2 REG_MCUFW_CTRL				(Offset 0x0080) */

#define BIT_EMEM__TXBUF_CHKSUM_OK BIT(10)

/* 2 REG_MCUFW_CTRL				(Offset 0x0080) */

#define BIT_EMEM_TXBUF_DW_RDY BIT(9)

/* 2 REG_MCUFW_CTRL				(Offset 0x0080) */

#define BIT_EMEM_CHKSUM_OK BIT(8)
#define BIT_EMEM_DW_OK BIT(7)
#define BIT_TOGGLING BIT(7)
#define BIT_DMEM_CHKSUM_OK BIT(6)
#define BIT_ACK BIT(6)

/* 2 REG_MCUFW_CTRL				(Offset 0x0080) */

#define BIT_DMEM_DW_OK BIT(5)

/* 2 REG_MCUFW_CTRL				(Offset 0x0080) */

#define BIT_IMEM_CHKSUM_OK BIT(4)

/* 2 REG_MCUFW_CTRL				(Offset 0x0080) */

#define BIT_IMEM_DW_OK BIT(3)

/* 2 REG_MCUFW_CTRL				(Offset 0x0080) */

#define BIT_IMEM_BOOT_LOAD_CHKSUM_OK BIT(2)

/* 2 REG_MCUFW_CTRL				(Offset 0x0080) */

#define BIT_IMEM_BOOT_LOAD_DW_OK BIT(1)

/* 2 REG_SDIO_HRPWM1				(Offset 0x10250080) */

#define BIT_32K_PERMISSION BIT(0)

/* 2 REG_MCU_TST_CFG				(Offset 0x0084) */

#define BIT_SHIFT_LBKTST 0
#define BIT_MASK_LBKTST 0xffff
#define BIT_LBKTST(x) (((x) & BIT_MASK_LBKTST) << BIT_SHIFT_LBKTST)
#define BIT_GET_LBKTST(x) (((x) >> BIT_SHIFT_LBKTST) & BIT_MASK_LBKTST)

/* 2 REG_SDIO_BUS_CTRL			(Offset 0x10250085) */

#define BIT_PAD_CLK_XHGE_EN BIT(3)
#define BIT_INTER_CLK_EN BIT(2)
#define BIT_EN_RPT_TXCRC BIT(1)
#define BIT_DIS_RXDMA_STS BIT(0)

/* 2 REG_SDIO_HSUS_CTRL			(Offset 0x10250086) */

#define BIT_INTR_CTRL BIT(4)
#define BIT_SDIO_VOLTAGE BIT(3)
#define BIT_BYPASS_INIT BIT(2)

/* 2 REG_SDIO_HSUS_CTRL			(Offset 0x10250086) */

#define BIT_HCI_RESUME_RDY BIT(1)
#define BIT_HCI_SUS_REQ BIT(0)

/* 2 REG_HMEBOX_E0_E1			(Offset 0x0088) */

#define BIT_SHIFT_HOST_MSG_E1 16
#define BIT_MASK_HOST_MSG_E1 0xffff
#define BIT_HOST_MSG_E1(x)                                                     \
	(((x) & BIT_MASK_HOST_MSG_E1) << BIT_SHIFT_HOST_MSG_E1)
#define BIT_GET_HOST_MSG_E1(x)                                                 \
	(((x) >> BIT_SHIFT_HOST_MSG_E1) & BIT_MASK_HOST_MSG_E1)

#define BIT_SHIFT_HOST_MSG_E0 0
#define BIT_MASK_HOST_MSG_E0 0xffff
#define BIT_HOST_MSG_E0(x)                                                     \
	(((x) & BIT_MASK_HOST_MSG_E0) << BIT_SHIFT_HOST_MSG_E0)
#define BIT_GET_HOST_MSG_E0(x)                                                 \
	(((x) >> BIT_SHIFT_HOST_MSG_E0) & BIT_MASK_HOST_MSG_E0)

/* 2 REG_SDIO_RESPONSE_TIMER			(Offset 0x10250088) */

#define BIT_SHIFT_CMDIN_2RESP_TIMER 0
#define BIT_MASK_CMDIN_2RESP_TIMER 0xffff
#define BIT_CMDIN_2RESP_TIMER(x)                                               \
	(((x) & BIT_MASK_CMDIN_2RESP_TIMER) << BIT_SHIFT_CMDIN_2RESP_TIMER)
#define BIT_GET_CMDIN_2RESP_TIMER(x)                                           \
	(((x) >> BIT_SHIFT_CMDIN_2RESP_TIMER) & BIT_MASK_CMDIN_2RESP_TIMER)

/* 2 REG_SDIO_CMD_CRC			(Offset 0x1025008A) */

#define BIT_SHIFT_SDIO_CMD_CRC_V1 0
#define BIT_MASK_SDIO_CMD_CRC_V1 0xff
#define BIT_SDIO_CMD_CRC_V1(x)                                                 \
	(((x) & BIT_MASK_SDIO_CMD_CRC_V1) << BIT_SHIFT_SDIO_CMD_CRC_V1)
#define BIT_GET_SDIO_CMD_CRC_V1(x)                                             \
	(((x) >> BIT_SHIFT_SDIO_CMD_CRC_V1) & BIT_MASK_SDIO_CMD_CRC_V1)

/* 2 REG_HMEBOX_E2_E3			(Offset 0x008C) */

#define BIT_SHIFT_HOST_MSG_E3 16
#define BIT_MASK_HOST_MSG_E3 0xffff
#define BIT_HOST_MSG_E3(x)                                                     \
	(((x) & BIT_MASK_HOST_MSG_E3) << BIT_SHIFT_HOST_MSG_E3)
#define BIT_GET_HOST_MSG_E3(x)                                                 \
	(((x) >> BIT_SHIFT_HOST_MSG_E3) & BIT_MASK_HOST_MSG_E3)

#define BIT_SHIFT_HOST_MSG_E2 0
#define BIT_MASK_HOST_MSG_E2 0xffff
#define BIT_HOST_MSG_E2(x)                                                     \
	(((x) & BIT_MASK_HOST_MSG_E2) << BIT_SHIFT_HOST_MSG_E2)
#define BIT_GET_HOST_MSG_E2(x)                                                 \
	(((x) >> BIT_SHIFT_HOST_MSG_E2) & BIT_MASK_HOST_MSG_E2)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_WLLPSOP_EABM BIT(31)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_WLLPSOP_ACKF BIT(30)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_WLLPSOP_DLDM BIT(29)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_WLLPSOP_ESWR BIT(28)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_WLLPSOP_PWMM BIT(27)
#define BIT_WLLPSOP_EECK BIT(26)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_WLLPSOP_WLMACOFF BIT(25)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_WLLPSOP_EXTAL BIT(24)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_WL_SYNPON_VOLTSPDN BIT(23)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_WLLPSOP_WLBBOFF BIT(22)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_WLLPSOP_WLMEM_DS BIT(21)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_SHIFT_LPLDH12_VADJ_STEP_DN 12
#define BIT_MASK_LPLDH12_VADJ_STEP_DN 0xf
#define BIT_LPLDH12_VADJ_STEP_DN(x)                                            \
	(((x) & BIT_MASK_LPLDH12_VADJ_STEP_DN)                                 \
	 << BIT_SHIFT_LPLDH12_VADJ_STEP_DN)
#define BIT_GET_LPLDH12_VADJ_STEP_DN(x)                                        \
	(((x) >> BIT_SHIFT_LPLDH12_VADJ_STEP_DN) &                             \
	 BIT_MASK_LPLDH12_VADJ_STEP_DN)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_SHIFT_V15ADJ_L1_STEP_DN 8
#define BIT_MASK_V15ADJ_L1_STEP_DN 0x7
#define BIT_V15ADJ_L1_STEP_DN(x)                                               \
	(((x) & BIT_MASK_V15ADJ_L1_STEP_DN) << BIT_SHIFT_V15ADJ_L1_STEP_DN)
#define BIT_GET_V15ADJ_L1_STEP_DN(x)                                           \
	(((x) >> BIT_SHIFT_V15ADJ_L1_STEP_DN) & BIT_MASK_V15ADJ_L1_STEP_DN)

#define BIT_REGU_32K_CLK_EN BIT(1)
#define BIT_DRV_WLAN_INT_CLR BIT(1)

/* 2 REG_WLLPS_CTRL				(Offset 0x0090) */

#define BIT_WL_LPS_EN BIT(0)

/* 2 REG_SDIO_HSISR				(Offset 0x10250090) */

#define BIT_DRV_WLAN_INT BIT(0)

/* 2 REG_SDIO_HSIMR				(Offset 0x10250091) */

#define BIT_HISR_MASK BIT(0)

/* 2 REG_AFE_CTRL5				(Offset 0x0094) */

#define BIT_BB_DBG_SEL_AFE_SDM_BIT0 BIT(31)

/* 2 REG_AFE_CTRL5				(Offset 0x0094) */

#define BIT_ORDER_SDM BIT(30)
#define BIT_RFE_SEL_SDM BIT(29)

#define BIT_SHIFT_REF_SEL 25
#define BIT_MASK_REF_SEL 0xf
#define BIT_REF_SEL(x) (((x) & BIT_MASK_REF_SEL) << BIT_SHIFT_REF_SEL)
#define BIT_GET_REF_SEL(x) (((x) >> BIT_SHIFT_REF_SEL) & BIT_MASK_REF_SEL)

/* 2 REG_AFE_CTRL5				(Offset 0x0094) */

#define BIT_SHIFT_F0F_SDM 12
#define BIT_MASK_F0F_SDM 0x1fff
#define BIT_F0F_SDM(x) (((x) & BIT_MASK_F0F_SDM) << BIT_SHIFT_F0F_SDM)
#define BIT_GET_F0F_SDM(x) (((x) >> BIT_SHIFT_F0F_SDM) & BIT_MASK_F0F_SDM)

/* 2 REG_AFE_CTRL5				(Offset 0x0094) */

#define BIT_SHIFT_F0N_SDM 9
#define BIT_MASK_F0N_SDM 0x7
#define BIT_F0N_SDM(x) (((x) & BIT_MASK_F0N_SDM) << BIT_SHIFT_F0N_SDM)
#define BIT_GET_F0N_SDM(x) (((x) >> BIT_SHIFT_F0N_SDM) & BIT_MASK_F0N_SDM)

/* 2 REG_AFE_CTRL5				(Offset 0x0094) */

#define BIT_SHIFT_DIVN_SDM 3
#define BIT_MASK_DIVN_SDM 0x3f
#define BIT_DIVN_SDM(x) (((x) & BIT_MASK_DIVN_SDM) << BIT_SHIFT_DIVN_SDM)
#define BIT_GET_DIVN_SDM(x) (((x) >> BIT_SHIFT_DIVN_SDM) & BIT_MASK_DIVN_SDM)

/* 2 REG_GPIO_DEBOUNCE_CTRL			(Offset 0x0098) */

#define BIT_WLGP_DBC1EN BIT(15)

#define BIT_SHIFT_WLGP_DBC1 8
#define BIT_MASK_WLGP_DBC1 0xf
#define BIT_WLGP_DBC1(x) (((x) & BIT_MASK_WLGP_DBC1) << BIT_SHIFT_WLGP_DBC1)
#define BIT_GET_WLGP_DBC1(x) (((x) >> BIT_SHIFT_WLGP_DBC1) & BIT_MASK_WLGP_DBC1)

#define BIT_WLGP_DBC0EN BIT(7)

#define BIT_SHIFT_WLGP_DBC0 0
#define BIT_MASK_WLGP_DBC0 0xf
#define BIT_WLGP_DBC0(x) (((x) & BIT_MASK_WLGP_DBC0) << BIT_SHIFT_WLGP_DBC0)
#define BIT_GET_WLGP_DBC0(x) (((x) >> BIT_SHIFT_WLGP_DBC0) & BIT_MASK_WLGP_DBC0)

/* 2 REG_RPWM2				(Offset 0x009C) */

#define BIT_SHIFT_RPWM2 16
#define BIT_MASK_RPWM2 0xffff
#define BIT_RPWM2(x) (((x) & BIT_MASK_RPWM2) << BIT_SHIFT_RPWM2)
#define BIT_GET_RPWM2(x) (((x) >> BIT_SHIFT_RPWM2) & BIT_MASK_RPWM2)

/* 2 REG_SYSON_FSM_MON			(Offset 0x00A0) */

#define BIT_SHIFT_FSM_MON_SEL 24
#define BIT_MASK_FSM_MON_SEL 0x7
#define BIT_FSM_MON_SEL(x)                                                     \
	(((x) & BIT_MASK_FSM_MON_SEL) << BIT_SHIFT_FSM_MON_SEL)
#define BIT_GET_FSM_MON_SEL(x)                                                 \
	(((x) >> BIT_SHIFT_FSM_MON_SEL) & BIT_MASK_FSM_MON_SEL)

#define BIT_DOP_ELDO BIT(23)
#define BIT_FSM_MON_UPD BIT(15)

#define BIT_SHIFT_FSM_PAR 0
#define BIT_MASK_FSM_PAR 0x7fff
#define BIT_FSM_PAR(x) (((x) & BIT_MASK_FSM_PAR) << BIT_SHIFT_FSM_PAR)
#define BIT_GET_FSM_PAR(x) (((x) >> BIT_SHIFT_FSM_PAR) & BIT_MASK_FSM_PAR)

/* 2 REG_AFE_CTRL6				(Offset 0x00A4) */

#define BIT_SHIFT_BB_DBG_SEL_AFE_SDM_BIT3_1 0
#define BIT_MASK_BB_DBG_SEL_AFE_SDM_BIT3_1 0x7
#define BIT_BB_DBG_SEL_AFE_SDM_BIT3_1(x)                                       \
	(((x) & BIT_MASK_BB_DBG_SEL_AFE_SDM_BIT3_1)                            \
	 << BIT_SHIFT_BB_DBG_SEL_AFE_SDM_BIT3_1)
#define BIT_GET_BB_DBG_SEL_AFE_SDM_BIT3_1(x)                                   \
	(((x) >> BIT_SHIFT_BB_DBG_SEL_AFE_SDM_BIT3_1) &                        \
	 BIT_MASK_BB_DBG_SEL_AFE_SDM_BIT3_1)

/* 2 REG_PMC_DBG_CTRL1			(Offset 0x00A8) */

#define BIT_BT_INT_EN BIT(31)

#define BIT_SHIFT_RD_WR_WIFI_BT_INFO 16
#define BIT_MASK_RD_WR_WIFI_BT_INFO 0x7fff
#define BIT_RD_WR_WIFI_BT_INFO(x)                                              \
	(((x) & BIT_MASK_RD_WR_WIFI_BT_INFO) << BIT_SHIFT_RD_WR_WIFI_BT_INFO)
#define BIT_GET_RD_WR_WIFI_BT_INFO(x)                                          \
	(((x) >> BIT_SHIFT_RD_WR_WIFI_BT_INFO) & BIT_MASK_RD_WR_WIFI_BT_INFO)

/* 2 REG_PMC_DBG_CTRL1			(Offset 0x00A8) */

#define BIT_PMC_WR_OVF BIT(8)

#define BIT_SHIFT_WLPMC_ERRINT 0
#define BIT_MASK_WLPMC_ERRINT 0xff
#define BIT_WLPMC_ERRINT(x)                                                    \
	(((x) & BIT_MASK_WLPMC_ERRINT) << BIT_SHIFT_WLPMC_ERRINT)
#define BIT_GET_WLPMC_ERRINT(x)                                                \
	(((x) >> BIT_SHIFT_WLPMC_ERRINT) & BIT_MASK_WLPMC_ERRINT)

/* 2 REG_AFE_CTRL7				(Offset 0x00AC) */

#define BIT_SHIFT_SEL_V 30
#define BIT_MASK_SEL_V 0x3
#define BIT_SEL_V(x) (((x) & BIT_MASK_SEL_V) << BIT_SHIFT_SEL_V)
#define BIT_GET_SEL_V(x) (((x) >> BIT_SHIFT_SEL_V) & BIT_MASK_SEL_V)

/* 2 REG_AFE_CTRL7				(Offset 0x00AC) */

#define BIT_TXFIFO_TH_INT BIT(30)

/* 2 REG_AFE_CTRL7				(Offset 0x00AC) */

#define BIT_SEL_LDO_PC BIT(29)

/* 2 REG_AFE_CTRL7				(Offset 0x00AC) */

#define BIT_SHIFT_CK_MON_SEL 26
#define BIT_MASK_CK_MON_SEL 0x7
#define BIT_CK_MON_SEL(x) (((x) & BIT_MASK_CK_MON_SEL) << BIT_SHIFT_CK_MON_SEL)
#define BIT_GET_CK_MON_SEL(x)                                                  \
	(((x) >> BIT_SHIFT_CK_MON_SEL) & BIT_MASK_CK_MON_SEL)

/* 2 REG_AFE_CTRL7				(Offset 0x00AC) */

#define BIT_CK_MON_EN BIT(25)
#define BIT_FREF_EDGE BIT(24)
#define BIT_CK320M_EN BIT(23)
#define BIT_CK_5M_EN BIT(22)
#define BIT_TESTEN BIT(21)

/* 2 REG_HIMR0				(Offset 0x00B0) */

#define BIT_TIMEOUT_INTERRUPT2_MASK BIT(31)
#define BIT_TIMEOUT_INTERRUTP1_MASK BIT(30)
#define BIT_PSTIMEOUT_MSK BIT(29)
#define BIT_GTINT4_MSK BIT(28)
#define BIT_GTINT3_MSK BIT(27)
#define BIT_TXBCN0ERR_MSK BIT(26)
#define BIT_TXBCN0OK_MSK BIT(25)
#define BIT_TSF_BIT32_TOGGLE_MSK BIT(24)
#define BIT_BCNDMAINT0_MSK BIT(20)
#define BIT_BCNDERR0_MSK BIT(16)
#define BIT_HSISR_IND_ON_INT_MSK BIT(15)

/* 2 REG_HIMR0				(Offset 0x00B0) */

#define BIT_BCNDMAINT_E_MSK BIT(14)

/* 2 REG_HIMR0				(Offset 0x00B0) */

#define BIT_CTWEND_MSK BIT(12)
#define BIT_HISR1_IND_MSK BIT(11)

/* 2 REG_HIMR0				(Offset 0x00B0) */

#define BIT_C2HCMD_MSK BIT(10)
#define BIT_CPWM2_MSK BIT(9)
#define BIT_CPWM_MSK BIT(8)
#define BIT_HIGHDOK_MSK BIT(7)
#define BIT_MGTDOK_MSK BIT(6)
#define BIT_BKDOK_MSK BIT(5)
#define BIT_BEDOK_MSK BIT(4)
#define BIT_VIDOK_MSK BIT(3)
#define BIT_VODOK_MSK BIT(2)
#define BIT_RDU_MSK BIT(1)
#define BIT_RXOK_MSK BIT(0)

/* 2 REG_HISR0				(Offset 0x00B4) */

#define BIT_TIMEOUT_INTERRUPT2 BIT(31)

/* 2 REG_HISR0				(Offset 0x00B4) */

#define BIT_TIMEOUT_INTERRUTP1 BIT(30)

/* 2 REG_HISR0				(Offset 0x00B4) */

#define BIT_PSTIMEOUT BIT(29)
#define BIT_GTINT4 BIT(28)
#define BIT_GTINT3 BIT(27)
#define BIT_TXBCN0ERR BIT(26)
#define BIT_TXBCN0OK BIT(25)
#define BIT_TSF_BIT32_TOGGLE BIT(24)
#define BIT_BCNDMAINT0 BIT(20)
#define BIT_BCNDERR0 BIT(16)
#define BIT_HSISR_IND_ON_INT BIT(15)

/* 2 REG_HISR0				(Offset 0x00B4) */

#define BIT_BCNDMAINT_E BIT(14)

/* 2 REG_HISR0				(Offset 0x00B4) */

#define BIT_CTWEND BIT(12)

/* 2 REG_HISR0				(Offset 0x00B4) */

#define BIT_HISR1_IND_INT BIT(11)
#define BIT_C2HCMD BIT(10)
#define BIT_CPWM2 BIT(9)
#define BIT_CPWM BIT(8)
#define BIT_HIGHDOK BIT(7)
#define BIT_MGTDOK BIT(6)
#define BIT_BKDOK BIT(5)
#define BIT_BEDOK BIT(4)
#define BIT_VIDOK BIT(3)
#define BIT_VODOK BIT(2)
#define BIT_RDU BIT(1)
#define BIT_RXOK BIT(0)

/* 2 REG_HIMR1				(Offset 0x00B8) */

#define BIT_BTON_STS_UPDATE_MASK BIT(29)

/* 2 REG_HIMR1				(Offset 0x00B8) */

#define BIT_MCU_ERR_MASK BIT(28)

/* 2 REG_HIMR1				(Offset 0x00B8) */

#define BIT_BCNDMAINT7__MSK BIT(27)

/* 2 REG_HIMR1				(Offset 0x00B8) */

#define BIT_BCNDMAINT6__MSK BIT(26)

/* 2 REG_HIMR1				(Offset 0x00B8) */

#define BIT_BCNDMAINT5__MSK BIT(25)

/* 2 REG_HIMR1				(Offset 0x00B8) */

#define BIT_BCNDMAINT4__MSK BIT(24)

/* 2 REG_HIMR1				(Offset 0x00B8) */

#define BIT_BCNDMAINT3_MSK BIT(23)
#define BIT_BCNDMAINT2_MSK BIT(22)
#define BIT_BCNDMAINT1_MSK BIT(21)
#define BIT_BCNDERR7_MSK BIT(20)
#define BIT_BCNDERR6_MSK BIT(19)
#define BIT_BCNDERR5_MSK BIT(18)
#define BIT_BCNDERR4_MSK BIT(17)
#define BIT_BCNDERR3_MSK BIT(16)
#define BIT_BCNDERR2_MSK BIT(15)
#define BIT_BCNDERR1_MSK BIT(14)

/* 2 REG_HIMR1				(Offset 0x00B8) */

#define BIT_ATIMEND_E_MSK BIT(13)

/* 2 REG_HIMR1				(Offset 0x00B8) */

#define BIT_ATIMEND__MSK BIT(12)

/* 2 REG_HIMR1				(Offset 0x00B8) */

#define BIT_TXERR_MSK BIT(11)
#define BIT_RXERR_MSK BIT(10)
#define BIT_TXFOVW_MSK BIT(9)
#define BIT_FOVW_MSK BIT(8)

/* 2 REG_HIMR1				(Offset 0x00B8) */

#define BIT_CPU_MGQ_TXDONE_MSK BIT(5)
#define BIT_PS_TIMER_C_MSK BIT(4)
#define BIT_PS_TIMER_B_MSK BIT(3)
#define BIT_PS_TIMER_A_MSK BIT(2)
#define BIT_CPUMGQ_TX_TIMER_MSK BIT(1)

/* 2 REG_HISR1				(Offset 0x00BC) */

#define BIT_BTON_STS_UPDATE_INT BIT(29)

/* 2 REG_HISR1				(Offset 0x00BC) */

#define BIT_MCU_ERR BIT(28)

/* 2 REG_HISR1				(Offset 0x00BC) */

#define BIT_BCNDMAINT7 BIT(27)
#define BIT_BCNDMAINT6 BIT(26)
#define BIT_BCNDMAINT5 BIT(25)
#define BIT_BCNDMAINT4 BIT(24)
#define BIT_BCNDMAINT3 BIT(23)
#define BIT_BCNDMAINT2 BIT(22)
#define BIT_BCNDMAINT1 BIT(21)
#define BIT_BCNDERR7 BIT(20)
#define BIT_BCNDERR6 BIT(19)
#define BIT_BCNDERR5 BIT(18)
#define BIT_BCNDERR4 BIT(17)
#define BIT_BCNDERR3 BIT(16)
#define BIT_BCNDERR2 BIT(15)
#define BIT_BCNDERR1 BIT(14)

/* 2 REG_HISR1				(Offset 0x00BC) */

#define BIT_ATIMEND_E BIT(13)

/* 2 REG_HISR1				(Offset 0x00BC) */

#define BIT_ATIMEND BIT(12)
#define BIT_TXERR_INT BIT(11)
#define BIT_RXERR_INT BIT(10)
#define BIT_TXFOVW BIT(9)
#define BIT_FOVW BIT(8)

/* 2 REG_HISR1				(Offset 0x00BC) */

#define BIT_CPU_MGQ_TXDONE BIT(5)
#define BIT_PS_TIMER_C BIT(4)
#define BIT_PS_TIMER_B BIT(3)
#define BIT_PS_TIMER_A BIT(2)
#define BIT_CPUMGQ_TX_TIMER BIT(1)

/* 2 REG_SDIO_ERR_RPT			(Offset 0x102500C0) */

#define BIT_HR_FF_OVF BIT(6)
#define BIT_HR_FF_UDN BIT(5)
#define BIT_TXDMA_BUSY_ERR BIT(4)
#define BIT_TXDMA_VLD_ERR BIT(3)
#define BIT_QSEL_UNKNOWN_ERR BIT(2)
#define BIT_QSEL_MIS_ERR BIT(1)

/* 2 REG_DBG_PORT_SEL			(Offset 0x00C0) */

#define BIT_SHIFT_DEBUG_ST 0
#define BIT_MASK_DEBUG_ST 0xffffffffL
#define BIT_DEBUG_ST(x) (((x) & BIT_MASK_DEBUG_ST) << BIT_SHIFT_DEBUG_ST)
#define BIT_GET_DEBUG_ST(x) (((x) >> BIT_SHIFT_DEBUG_ST) & BIT_MASK_DEBUG_ST)

/* 2 REG_SDIO_ERR_RPT			(Offset 0x102500C0) */

#define BIT_SDIO_OVERRD_ERR BIT(0)

/* 2 REG_SDIO_CMD_ERRCNT			(Offset 0x102500C1) */

#define BIT_SHIFT_CMD_CRC_ERR_CNT 0
#define BIT_MASK_CMD_CRC_ERR_CNT 0xff
#define BIT_CMD_CRC_ERR_CNT(x)                                                 \
	(((x) & BIT_MASK_CMD_CRC_ERR_CNT) << BIT_SHIFT_CMD_CRC_ERR_CNT)
#define BIT_GET_CMD_CRC_ERR_CNT(x)                                             \
	(((x) >> BIT_SHIFT_CMD_CRC_ERR_CNT) & BIT_MASK_CMD_CRC_ERR_CNT)

/* 2 REG_SDIO_DATA_ERRCNT			(Offset 0x102500C2) */

#define BIT_SHIFT_DATA_CRC_ERR_CNT 0
#define BIT_MASK_DATA_CRC_ERR_CNT 0xff
#define BIT_DATA_CRC_ERR_CNT(x)                                                \
	(((x) & BIT_MASK_DATA_CRC_ERR_CNT) << BIT_SHIFT_DATA_CRC_ERR_CNT)
#define BIT_GET_DATA_CRC_ERR_CNT(x)                                            \
	(((x) >> BIT_SHIFT_DATA_CRC_ERR_CNT) & BIT_MASK_DATA_CRC_ERR_CNT)

/* 2 REG_PAD_CTRL2				(Offset 0x00C4) */

#define BIT_USB3_USB2_TRANSITION BIT(20)

/* 2 REG_PAD_CTRL2				(Offset 0x00C4) */

#define BIT_SHIFT_USB23_SW_MODE_V1 18
#define BIT_MASK_USB23_SW_MODE_V1 0x3
#define BIT_USB23_SW_MODE_V1(x)                                                \
	(((x) & BIT_MASK_USB23_SW_MODE_V1) << BIT_SHIFT_USB23_SW_MODE_V1)
#define BIT_GET_USB23_SW_MODE_V1(x)                                            \
	(((x) >> BIT_SHIFT_USB23_SW_MODE_V1) & BIT_MASK_USB23_SW_MODE_V1)

/* 2 REG_PAD_CTRL2				(Offset 0x00C4) */

#define BIT_NO_PDN_CHIPOFF_V1 BIT(17)

/* 2 REG_PAD_CTRL2				(Offset 0x00C4) */

#define BIT_RSM_EN_V1 BIT(16)

/* 2 REG_PAD_CTRL2				(Offset 0x00C4) */

#define BIT_LD_B12V_EN BIT(7)

/* 2 REG_PAD_CTRL2				(Offset 0x00C4) */

#define BIT_EECS_IOSEL_V1 BIT(6)

/* 2 REG_PAD_CTRL2				(Offset 0x00C4) */

#define BIT_EECS_DATA_O_V1 BIT(5)

/* 2 REG_PAD_CTRL2				(Offset 0x00C4) */

#define BIT_EECS_DATA_I_V1 BIT(4)

/* 2 REG_PAD_CTRL2				(Offset 0x00C4) */

#define BIT_EESK_IOSEL_V1 BIT(2)

/* 2 REG_PAD_CTRL2				(Offset 0x00C4) */

#define BIT_EESK_DATA_O_V1 BIT(1)

/* 2 REG_PAD_CTRL2				(Offset 0x00C4) */

#define BIT_EESK_DATA_I_V1 BIT(0)

/* 2 REG_SDIO_CMD_ERR_CONTENT		(Offset 0x102500C4) */

#define BIT_SHIFT_SDIO_CMD_ERR_CONTENT 0
#define BIT_MASK_SDIO_CMD_ERR_CONTENT 0xffffffffffL
#define BIT_SDIO_CMD_ERR_CONTENT(x)                                            \
	(((x) & BIT_MASK_SDIO_CMD_ERR_CONTENT)                                 \
	 << BIT_SHIFT_SDIO_CMD_ERR_CONTENT)
#define BIT_GET_SDIO_CMD_ERR_CONTENT(x)                                        \
	(((x) >> BIT_SHIFT_SDIO_CMD_ERR_CONTENT) &                             \
	 BIT_MASK_SDIO_CMD_ERR_CONTENT)

/* 2 REG_SDIO_CRC_ERR_IDX			(Offset 0x102500C9) */

#define BIT_D3_CRC_ERR BIT(4)
#define BIT_D2_CRC_ERR BIT(3)
#define BIT_D1_CRC_ERR BIT(2)
#define BIT_D0_CRC_ERR BIT(1)
#define BIT_CMD_CRC_ERR BIT(0)

/* 2 REG_SDIO_DATA_CRC			(Offset 0x102500CA) */

#define BIT_SHIFT_SDIO_DATA_CRC 0
#define BIT_MASK_SDIO_DATA_CRC 0xff
#define BIT_SDIO_DATA_CRC(x)                                                   \
	(((x) & BIT_MASK_SDIO_DATA_CRC) << BIT_SHIFT_SDIO_DATA_CRC)
#define BIT_GET_SDIO_DATA_CRC(x)                                               \
	(((x) >> BIT_SHIFT_SDIO_DATA_CRC) & BIT_MASK_SDIO_DATA_CRC)

/* 2 REG_SDIO_DATA_REPLY_TIME		(Offset 0x102500CB) */

#define BIT_SHIFT_SDIO_DATA_REPLY_TIME 0
#define BIT_MASK_SDIO_DATA_REPLY_TIME 0x7
#define BIT_SDIO_DATA_REPLY_TIME(x)                                            \
	(((x) & BIT_MASK_SDIO_DATA_REPLY_TIME)                                 \
	 << BIT_SHIFT_SDIO_DATA_REPLY_TIME)
#define BIT_GET_SDIO_DATA_REPLY_TIME(x)                                        \
	(((x) >> BIT_SHIFT_SDIO_DATA_REPLY_TIME) &                             \
	 BIT_MASK_SDIO_DATA_REPLY_TIME)

/* 2 REG_PMC_DBG_CTRL2			(Offset 0x00CC) */

#define BIT_SHIFT_EFUSE_BURN_GNT 24
#define BIT_MASK_EFUSE_BURN_GNT 0xff
#define BIT_EFUSE_BURN_GNT(x)                                                  \
	(((x) & BIT_MASK_EFUSE_BURN_GNT) << BIT_SHIFT_EFUSE_BURN_GNT)
#define BIT_GET_EFUSE_BURN_GNT(x)                                              \
	(((x) >> BIT_SHIFT_EFUSE_BURN_GNT) & BIT_MASK_EFUSE_BURN_GNT)

/* 2 REG_PMC_DBG_CTRL2			(Offset 0x00CC) */

#define BIT_STOP_WL_PMC BIT(9)
#define BIT_STOP_SYM_PMC BIT(8)

/* 2 REG_PMC_DBG_CTRL2			(Offset 0x00CC) */

#define BIT_REG_RST_WLPMC BIT(5)
#define BIT_REG_RST_PD12N BIT(4)
#define BIT_SYSON_DIS_WLREG_WRMSK BIT(3)
#define BIT_SYSON_DIS_PMCREG_WRMSK BIT(2)

#define BIT_SHIFT_SYSON_REG_ARB 0
#define BIT_MASK_SYSON_REG_ARB 0x3
#define BIT_SYSON_REG_ARB(x)                                                   \
	(((x) & BIT_MASK_SYSON_REG_ARB) << BIT_SHIFT_SYSON_REG_ARB)
#define BIT_GET_SYSON_REG_ARB(x)                                               \
	(((x) >> BIT_SHIFT_SYSON_REG_ARB) & BIT_MASK_SYSON_REG_ARB)

/* 2 REG_BIST_CTRL				(Offset 0x00D0) */

#define BIT_BIST_USB_DIS BIT(27)

/* 2 REG_BIST_CTRL				(Offset 0x00D0) */

#define BIT_BIST_PCI_DIS BIT(26)

/* 2 REG_BIST_CTRL				(Offset 0x00D0) */

#define BIT_BIST_BT_DIS BIT(25)

/* 2 REG_BIST_CTRL				(Offset 0x00D0) */

#define BIT_BIST_WL_DIS BIT(24)

/* 2 REG_BIST_CTRL				(Offset 0x00D0) */

#define BIT_SHIFT_BIST_RPT_SEL 16
#define BIT_MASK_BIST_RPT_SEL 0xf
#define BIT_BIST_RPT_SEL(x)                                                    \
	(((x) & BIT_MASK_BIST_RPT_SEL) << BIT_SHIFT_BIST_RPT_SEL)
#define BIT_GET_BIST_RPT_SEL(x)                                                \
	(((x) >> BIT_SHIFT_B    \
	(((x) & BIT2H_MSG)

/* 2 REG_WL_BT_PWR_CTRL			(Offset 0x0068) */

#define BIT_ISO_BD2PP BIOPt 0x0068) */

#defin
iQ#define BIT_EN_CPL_TIMEOUT_PS BIT(22)

/* 2 REG_WL_BT_PWR_CTRL	C) << BIT_SHIF0x0068) *iORM BIT(24)efin
iQ#define BIT_EN_CPL_TIMEOUT_PS BIT(22)

/* 2 REG_WL_BT_PWSTBIT(1)
#define BIT_DRne BIT_EN BIT(1)0efin
iQ#define BIT			(OL_TIMEOUT_PS B */

#define BIT_SHIFT_SDM_BT_PWRRT_S
#define BIT_MASK_SYM_BT_PWRRT_S
#fffffffL
#define BIT_DEM_BT_PWRRT_S)                                                    \
	(((x) & BIT_MASK_BIM_BT_PWRRT_S<< BIT_SHIFT_BIM_BT_PWRRT_S<define BIT_GET_BIM_BT_PWRRT_S)                                                  	(((x) >> BIT_SHIFT_B M_BT_PWRRT_S<<BIT_MASK_BIM_BT_PWRRT_S</* 2 REG_MCU_CHK_EN_CPL_TIMEOUT_PS B */

#define BIT_CPUU_CHRTRL	C) <1</* 2 REG_MCU_CHK_EN_CPL_TIMEOUT_PS B */

#define BIT_CPIFT_B  CPIPRAM 2#define BIT_MASK_US CPIPRAM 3
#define BIT_SY CPIPRAM) (((x) & BIT_MASK_DE CPIPRAM<< BIT_SHIFT_BISCPIPRAM<define BIT_GET_BISCPIPRAM) (((x) &  BIT_SHIFT_B  CPIPRAM<<BIT_MASK_DE CPIPRAM</* 2 REG_MCU_CHK_EN_CPL_TIMEOUT_PS B */

#define BIT_CPIFT_B  CPRO24
#define BIT_MASK_RP CPRO24f
#define BIT_BISCPRO2) (((x) & BIT_MASK_DE CPRO2<< BIT_SHIFT_BISCPRO2<define BIT_GET_BISCPRO2) (((x) &  BIT_SHIFT_B  CPRO2<<BIT_MASK_DE CPRO2<#define BIT_CPIFT_B I_DISPRAM 
#define BIT_MASK_REI_DISPRAM 3
#define BIT_SYI_DISPRAM) (((x) & BIT_MASK_DEI_DISPRAM<< BIT_SHIFT_BII_DISPRAM<define BIT_GET_BII_DISPRAM) (((x) &  BIT_SHIFT_B I_DISPRAM<<BIT_MASK_DEI_DISPRAM</* 2 REG_MCU_CHK_EN_CPL_TIMEOUT_PS B */

#define BIT_CPIFT_B I_DIIPRAM #define BIT_MASK_USI_DIIPRAM 3
#define BIT_SYI_DIIPRAM) (((x) & BIT_MASK_DEI_DIIPRAM<< BIT_SHIFT_BII_DIIPRAM<define BIT_GET_BII_DIIPRAM) (((x) &  BIT_SHIFT_B I_DIIPRAM<<BIT_MASK_DEI_DIIPRAM<ddefine BIT_SHIFT_USB23IIPRAM #define BIT_MASK_BIB23IIPRAM 3
#define BIT_USB23IIPRAM) (((x) & BIT_MASK_DEB23IIPRAM<< BIT_SHIFT_USB23IIPRAM<define BIT_GET_BIB23IIPRAM) (((x) &  BIT_SHIFT_USB23IIPRAM<<BIT_MASK_DEB23IIPRAM<d* 2 REG_MCU_CHK_EN_CPL_TIMEOUT_PS B */

#define BIT_CPIFT_B B23IIPRF #define BIT_MASK_RPB23IIPRF 3
#define BIT_USB23IIPRF) (((x) & BIT_MASK_DEB23IIPRF<< BIT_SHIFT_USB23IIPRF<define BIT_GET_BIB23IIPRF) (((x) &  BIT_SHIFT_USB23IIPRF& BIT_MASK_DEB23IIPRF<d* 2 REG_MCU_CHK_EN_CPL_TIMEOUT_PS B */

#define BIT_CPIFT_B U_ERRO24fdefine BIT_MASK_SYM_ERRO24f
#define BIT_BIM_ERRO2) (((x) & BIT_MASK_DEM_ERRO2<< BIT_SHIFT_BIM_ERRO2<define BIT_GET_BIM_ERRO2) (((x) &  BIT_SHIFT_B M_ERRO2<<BIT_MASK_DEM_ERRO2</* 2 REG_AFE_CTRL7	8_CPL_TIMEOUT_PS B */

#define BIT_STOYN_AIO BIT(24)

/* 2 REG_AFE_CTRL7	8_CPL_TIMEOUT_PS B */

#define BIT_STAL_SELPIT(4)
#define BIT_SYAL_SEGMG_WPL	C) << * 2 REG_AFE_CTRL7	8_CPL_TIMEOUT_PS B */

#define BIT_STOFT_B AL_SEL B_TOK4fdefine BIT_MASK_SYAL_SEL B_TOK4f
#define BIT_SDAL_SEL B_TOK)                                                    \
	(((x) & BIT_MASK_BIAL_SEL B_TOK<< BIT_SHIFT_BIAL_SEL B_TOK<define BIT_GET_BIAL_SEL B_TOK)                                                  	(((x) >> BIT_SHIFT_B AL_SEL B_TOK<<BIT_MASK_BIAL_SEL B_TOK< * 2 REG_AFB23IIIINT BFCPL_TIMEOUT_PS EIT(22)

/* 2 REG_WLRDEL BIT(8)<1</* 2 REG_MCB23IIIINT BFCPL_TIMEOUT_PS EIT(22)

/* 2 REG_WLB23IIIINT BF_WE1 BIT(0))
#define BIT_PSB23IIIINT BF_BYIOG_MC BIT(2)
9#define BIT_PSB23IIIINL BECBIT(29)8</* 2 REG_MCB23IIIINT BFCPL_TIMEOUT_PS EIT(22)

/* 2 REG_WLIFT_USB23IIIINT BF_ADDR1 18
#define BIT_MASK_BIB23IIIINT BF_ADDR1 181fff
define BIT_PSB23IIIINT BF_ADDR1 1)                                            \
	(((x) & BIT_MASK_SDB23IIIINT BF_ADDR1 1                                \
	 << BIT_SHIFT_SDB23IIIINT BF_ADDR1 1 define BIT_GET_BIB23IIIINT BF_ADDR1 1)                                          	(((x) >> BIT_SHIFT_USB23IIIINT BF_ADDR1 1                              \
	 BIT_MASK_SDB23IIIINT BF_ADDR1 1 d* 2 REG_MCB23IIIINT BFCPL_TIMEOUT_PS EIT(22)

/* 2 REG_WLIFT_USB23IIIINT BF_RD #define BIT_MASK_USB23IIIINT BF_RD ff
#define BIT_EFB23IIIINT BF_RD)                                                 \
	(((x) & BIT_MASK_CMB23IIIINT BF_RD<< BIT_SHIFT_USB23IIIINT BF_RD<define BIT_GET_BIB23IIIINT BF_RD)                                               	(((x) >> BIT_SHIFT_USB23IIIINT BF_RD<<BIT_MASK_CMB23IIIINT BF_RD<)

/* 2 REG_WLIFT_USB23IIIINT BF_WD4fdefine BIT_MASK_SYB23IIIINT BF_WD4ff
#define BIT_EFB23IIIINT BF_WD)                                                 \
	(((x) & BIT_MASK_CMB23IIIINT BF_WD<< BIT_SHIFT_USB23IIIINT BF_WD<define BIT_GET_BIB23IIIINT BF_WD)                                               	(((x) >> BIT_SHIFT_USB23IIIINT BF_WD<<BIT_MASK_CMB23IIIINT BF_WD<d* 2 REG_PMCE_REM_INT BFCPL_TIMEOUT_PS E */

#define BIT_PCI_VREM_INBYIOG_MIT(1)

#define BIT_PS__VREM_INRRL	C) 1

#define BIT_SHIFT_SY__VREM_INWE #define BIT_MASK_USI_DREM_INWE f
#define BIT_BII_DREM_INWE)                                                     \
	(((x) & BIT_MASK_FSI_DREM_INWE<< BIT_SHIFT_BII_DREM_INWE<define BIT_GET_BII_DREM_INWE)                                                   	(((x) >> BIT_SHIFT_USI_DREM_INWE<<BIT_MASK_FSI_DREM_INWE<#define BIT_SHIFT_SY__VREM_INADDR4fdefine BIT_MASK_SY__VREM_INADDR4ff
#define BIT_EF__VREM_INADDR)                                                   \
	(((x) & BIT_MASK_SY__VREM_INADDR<< BIT_SHIFT_BII_DREM_INADDR<define BIT_GET_BII_DREM_INADDR)                                                 	(((x) >> BIT_SHIFT_USI_DREM_INADDR<<BIT_MASK_SY__VREM_INADDR<d* 2 REG_PMCE_REM_INT BDCPL_TIMEOUT_PS E */

#define BIT_CPIFT_B I_DREM_INTA_R4fdefine BIT_MASK_SY__VREM_INTA_R4ffffffffL
#define BIT_DE__VREM_INTA_R)                                                   \
	(((x) & BIT_MASK_SY__VREM_INTA_R<< BIT_SHIFT_BII_DREM_INTA_R<define BIT_GET_BII_DREM_INTA_R)                                                 	(((x) >> BIT_SHIFT_USI_DREM_INTA_R<<BIT_MASK_SY__VREM_INTA_R</* 2 REG_WLLPRF			(Offset 0x00BCE */

#define BIT_STOFT_B LPRF	TRL BI
#define BIT_MASK_RPLPRF	TRL BIff
#define BIT_WLPMRF	TRL B) (((x) & BIT_MASK_DEPMRF	TRL B<< BIT_SHIFT_WLPMRF	TRL B<define BIT_GET_BIPMRF	TRL B) ((                                               \
	(((x) &  BIT_SHIFT_WLPMRF	TRL B<<BIT_MASK_DEPMRF	TRL B</* 2 REG_SYSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLIFT_USTRP_IG		 2#define BIT_MASK_USTRP_IG		 f
#define BIT_BITRP_IG		) (((x) & BIT_MASK_DETRP_IG		<< BIT_SHIFT_WLTRP_IG		<define BIT_GET_BITRP_IG		) (((x) &  BIT_SHIFT_WLTRP_IG		<<BIT_MASK_DETRP_IG		</* 2 REG_SYSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLRFYPE_L_IBIT(12)
#define BIT_BCNDCI_SUS BIT(26)

/* 2 REG_LDSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLNDCPKGUS BIT(26)5</* 2 REG_SYSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLIPSO_SWREBIT(24)

/* 2 REG_WLSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLRTL_IBIT(12)
#define BIT_PS_AHW_APD_IBBIT(22)
#d* 2 REG_WLSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLSTENDE_VIT(24)

/* 2 REG_AFSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLIFT_USVENDOIDX	8
#define BIT_MASK_BIVENDOIDX	8f
#define BIT_BIVENDOIDX	) (((x) & BIT_MASK_DEVENDOIDX	<< BIT_SHIFT_V15ENDOIDX	<define BIT_GET_BIVENDOIDX	) (((x) &  BIT_SHIFT_V15ENDOIDX	<<BIT_MASK_DEVENDOIDX	</* 2 REG_AFSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLIFT_USIPOFEVER2
#define BIT_MASK_CPUPOFEVER2f
#define BIT_BIUPOFEVER) (((x) & BIT_MASK_CK_POFEVER<< BIT_SHIFT_CMDPOFEVER<define BIT_GET_BIUPOFEVER) (((x) &  BIT_SHIFT_CMDPOFEVER<<BIT_MASK_CK_POFEVER</* 2 REG_LDSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLNDCMACBIT(16)1</* 2 REG_LDSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLNDCMACBIT(1)

#define BIT_BCNDCMACBIT(9)
#define BIT_CPSICDX	 BIT(24
#define BIT_BCSW_OFFAD_DW BIT(7)

/* 2 REG_PASONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLOCPHIFUTBBIT(22

/* 2 REG_LDSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLV15LD_EIT(225</* 2 REG_SYSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WL__VRSTBIT(3)

/* 2 REG_WLSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLPT_END_EIT(223
/* 2 REG_WLSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLUT_END_EIT(22
#d* 2 REG_WLSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLAT_END_EIT(221</* 2 REG_LDSONFG					(Offset 0x00BCFIT(22)

/* 2 REG_WLXT_END_EIT(22

/* 2 REG_AFSONFSTATUS			(Offset 0x00BCF */

#define BIT_SHIFT_SDRFYRL_IBI2#define BIT_MASK_USRFYRL_IBIf
#define BIT_REFFYRL_IB) (((x) & BIT_MASK_CKFFYRL_IB<< BIT_SHIFT_RD_FYRL_IB<define BIT_GET_BIFFYRL_IB) (((x) &  BIT_SHIFT_RD_FYRL_IB<<BIT_MASK_CKFFYRL_IB</* 2 REG_AFSONFSTATUS			(Offset 0x00BCF */

#define BIT_SHHPHY_IG		 B(19)
#d* 2 REG_AFSONFSTATUS			(Offset 0x00BCF */

#define BIT_SHIFT_SDL B_0XC08
#define BIT_MASK_BIL B_0XC083
#define BIT_SEL_V(0XC0) (((x) & BIT_MASK_SEL_V)0XC0<< BIT_SHIFT_SEL_V)0XC0<define BIT_GET_SYS_V(0XC0) (((x) &  BIT_SHIFT_SEL_V)0XC0<<BIT_MASK_SEL_V)0XC0<d* 2 REG_AFSONFSTATUS			(Offset 0x00BCF */

#define BIT_SHIFT_SDI_SUS B_V32
#define BIT_MASK_CPI_SUS B_V32f
#define BIT_SDI_SUS B_V3) (((x) & BIT_MASK_SEI_SUS B_V3<< BIT_SHIFT_HOS_SUS B_V3<define BIT_GET_SYI_SUS B_V3) ((                                               \
	(((x) &  BIT_SHIFT_WLI_SUS B_V3<<BIT_MASK_SEI_SUS B_V3<d* 2 REG_AFSONFSTATUS			(Offset 0x00BCF */

#define BIT_SHB23IOPERAON B_DE_VIT(24

#define BIT_BCNPD12NIT(9)
#define BIT_CPAUTOLPMC BIT(208<d* 2 REG_AFSONFSTATUS			(Offset 0x00BCF */

#define BIT_SHWL_DE_VIT(24

/* 2 REG_PASONFSTATUS			(Offset 0x00BCF */

#define BIT_SHPKGUS BEI_SIT(22

/* 2 REG_LDSONFSTATUS			(Offset 0x00BCF */

#define BIT_SHIFT_SD_AHW__SUS B_V1
#define BIT_MASK_DI_AHW__SUS B_V1
f
#define BIT_SD_AHW__SUS B_V1)                                                  \
	(((x) & BIT_MASK_EF_AHW__SUS B_V1<< BIT_SHIFT_BIIAHW__SUS B_V1<define BIT_GET_BIIAHW__SUS B_V1)                                                	(((x) >> BIT_SHIFT_USIAHW__SUS B_V1<<BIT_MASK_EF_AHW__SUS B_V1</* 2 REG_LDSONFSTATUS			(Offset 0x00BCF */

#define BIT_SHIFT_SDEFSW__SUS B_V1
fdefine BIT_MASK_DIEFSW__SUS B_V1
f
#define BIT_SDEFSW__SUS B_V1)                                                  \
	(((x) & BIT_MASK_EFUSSW__SUS B_V1<< BIT_SHIFT_BIUSSW__SUS B_V1<define BIT_GET_BIEFSW__SUS B_V1)                                                	(((x) >> BIT_SHIFT_EFUSSW__SUS B_V1<<BIT_MASK_EFUSSW__SUS B_V1<d* 2 REG_LDSONFSTATUS			(Offset 0x00C4F */

#define BIT_CPI_INAL2NIT(9))
#define BIT_BCB23IAL2NIT(9))8#define BIT_PS__VIAL2NIT(9))
#define BIT_BCSONFAL2NIT(9))6<#define BIT_SHIFT_SYEPVID8
#define BIT_MASK_WLEPVID8
ff
#define BIT_EFUPVID8) (((x) & BIT_MASK_SEUPVID8<< BIT_SHIFT_BIUPVID8<define BIT_GET_BIEPVID8) (((x) &  BIT_SHIFT_EFUPVID8<<BIT_MASK_SEUPVID8<#define BIT_SHIFT_SYEPVID0
#define BIT_MASK_WLEPVID0
#f
#define BIT_EFUPVID0) (((x) & BIT_MASK_SEUPVID0<< BIT_SHIFT_BIUPVID0<define BIT_GET_SYUPVID0) (((x) &  BIT_SHIFT_EFUPVID0& BIT_MASK_SEUPVID0</* 2 REG_LDSONFG					(Offset 0x00C4F */

#define BIT_ST__SUS B_EMBEDDEDIT(208<d* 2 REG_AFSONFG					(Offset 0x00C4F */

#define BIT_STIFT_WLIW_IBIfdefine BIT_MASK_CPIW_IBIf
##define BIT_HOSW_IB) (((x) & BIT_MASK_CKSW_IB<< BIT_SHIFT_HOSW_IB<define BIT_GET_BISW_IB) (((x) &  BIT_SHIFT_WLIW_IB<<BIT_MASK_CKSW_IB<d* 2 REG_AFC			(OOffset 0x00C10 */

#define BIT_SHIFT_LPLBDE_VI
#define BIT_MASK_RPLBDE_VI1fffdefine BIT_MALBDE_V) (((x) & BIT_MASK_CKLBDE_V<< BIT_SHIFT_LBKTDE_V<define BIT_GET_BILBDE_V) (((x) &  BIT_SHIFT_LBKTDE_V<<BIT_MASK_CKLBDE_V<#define BIT_SHIFT_LPNEPE_L18
#define BIT_MASK_USNEPE_L183
#define BIT_SENEPE_L1) (((x) & BIT_MASK_CKNEPE_L1<< BIT_SHIFT_LBNEPE_L1<define BIT_GET_BINEPE_L1) (((x) &  BIT_SHIFT_LBNEPE_L1<<BIT_MASK_CKNEPE_L1<#define BIT_SHIFT_LPNEPE_L08
#define BIT_MASK_BINEPE_L083
#define BIT_SENEPE_L0) (((x) & BIT_MASK_SENEPE_L0<< BIT_SHIFT_LBNEPE_L0<define BIT_GET_SYNEPE_L0) (((x) &  BIT_SHIFT_LBNEPE_L0& BIT_MASK_SENEPE_L0<d* 2 REG_AFC			(OOffset 0x00C10 */

#define BIT_SHI2CASKILX_E2_NIT(9))
#define BIT_D0SHCUEN BIT(31)1</* 2 REG_LDC			(OOffset 0x00C10 */

#define BIT_SHK_CLKAL_TMRN BIT(31)0<define BIT_GEMACUS CN BIT(31
#define BIT_BCENSWBCBIT(208<define BIT_GEMACRX BIT(7)

/efine BIT_GEMACTX BIT(7)
#define BIT_HRSCHEDULEN BIT(3)
#define BIT_PS_ROTOCOLN BIT(3)
#define BIT_SYDMA_ST BIT(3)
#define BIT_INTMA_ST BIT(3)
#define BIT_HIS_RESMA_ST BIT(3)
#define BIT_HCI_SUTMA_ST BIT(3)

/* 2 REG_PADKBCNUFFLATCESCTRL				ffset 0x00C10 */

#define BIT_HCIFT_USIKBCNUFFLATCESCTRL		4fdefine BIT_MASK_SY_KBCNUFFLATCESCTRL		4ff
#define BIT_EF_KBCNUFFLATCESCTRL		)                                            \
	(((x) & BIT_MASK_SD_KBCNUFFLATCESCTRL		                                \
	 << BIT_SHIFT_SD_KBCNUFFLATCESCTRL		 define BIT_GET_BIIKBCNUFFLATCESCTRL		)                                          	(((x) >> BIT_SHIFT_USIKBCNUFFLATCESCTRL		                              \
	 BIT_MASK_SD_KBCNUFFLATCESCTRL		 d* 2 REG_PAF_BIT_ENSTATE(OOffset 0x00C10 */

#define BIT_CPF_BIT_ENSTAB BIT(24)

#define BIT_SHIFT_FSI2CASUSY_NT)
_FW #define BIT_MASK_RPI2CASUSY_NT)
_FW f
#define BIT_SDI2CASUSY_NT)
_FW)                                                  	(((x) >>&IT_MASK_RPI2CASUSY_NT)
_FW<< BIT_SHIFT_LBI2CASUSY_NT)
_FW<define BIT_GET_BII2CASUSY_NT)
_FW)                                              	(((x) >> BIT_SHIFT_USI2CASUSY_NT)
_FW<<&IT_MASK_RPI2CASUSY_NT)
_FW<#define BIT_SHI2CASNT)
_FW T(3)
#ddefine BIT_SHIFT_FSI2CASUSPEEDI1define BIT_MASK_RPI2CASUSPEEDI3
#define BIT_SEI2CASUSPEED)                                                     \
	(((x) & BIT_MASK_FSI2CASUSPEED<< BIT_SHIFT_LBI2CASUSPEED<define BIT_GET_BII2CASUSPEED)                                                   	(((x) >> BIT_SHIFT_USI2CASUSPEED<<BIT_MASK_FSI2CASUSPEED<#define BIT_SHI2CASNUNLOCBIT(0)

/* 2 REG_HITMA_STPQASKP(OOffset 0x00C10 */

#define BIT_STIFT_WLTMA_STHIQASKP 1#define BIT_MASK_RPTMA_STHIQASKP 3
#define BIT_SETMA_STHIQASKP)                                                   \
	(((x) & BIT_MASK_SYTMA_STHIQASKP<< BIT_SHIFT_WLTMA_STHIQASKP<define BIT_GET_BITMA_STHIQASKP)                                                 	(((x) >> BIT_SHIFT_USTMA_STHIQASKP<<BIT_MASK_SYTMA_STHIQASKP<#define BIT_STIFT_WLTMA_STQ_TXSKP 1#define BIT_MASK_CPTMA_STQ_TXSKP 3
#define BIT_SETMA_STQ_TXSKP)                                                   \
	(((x) & BIT_MASK_SYTMA_STQ_TXSKP<< BIT_SHIFT_WLTMA_STQ_TXSKP<define BIT_GET_BITMA_STQ_TXSKP)                                                 	(((x) >> BIT_SHIFT_USTMA_STQ_TXSKP<<BIT_MASK_SYTMA_STQ_TXSKP<#define BIT_STIFT_WLTMA_STBKTXSKP 1fdefine BIT_MASK_SYTMA_STBKTXSKP 3
#define BIT_SETMA_STBKTXSKP)                                                   \
	(((x) & BIT_MASK_SYTMA_STBKTXSKP<< BIT_SHIFT_WLTMA_STBKTXSKP<define BIT_GET_BITMA_STBKTXSKP)                                                 	(((x) >> BIT_SHIFT_USTMA_STBKTXSKP<<BIT_MASK_SYTMA_STBKTXSKP<#define BIT_STIFT_WLTMA_STBETXSKP #define BIT_MASK_USTMA_STBETXSKP 3
#define BIT_SETMA_STBETXSKP)                                                   \
	(((x) & BIT_MASK_SYTMA_STBETXSKP<< BIT_SHIFT_WLTMA_STBETXSKP<define BIT_GET_BITMA_STBETXSKP)                                                 	(((x) >> BIT_SHIFT_USTMA_STBETXSKP<<BIT_MASK_SYTMA_STBETXSKP<#define BIT_STIFT_WLTMA_STVIQASKP #define BIT_MASK_BITMA_STVIQASKP 3
#define BIT_SETMA_STVIQASKP)                                                   \
	(((x) & BIT_MASK_SYTMA_STVIQASKP<< BIT_SHIFT_WLTMA_STVIQASKP<define BIT_GET_BITMA_STVIQASKP)                                                 	(((x) >> BIT_SHIFT_USTMA_STVIQASKP<<BIT_MASK_SYTMA_STVIQASKP<#define BIT_STIFT_WLTMA_STVOQASKP #define BIT_MASK_RPTMA_STVOQASKP 3
#define BIT_SETMA_STVOQASKP)                                                   \
	(((x) & BIT_MASK_SYTMA_STVOQASKP<< BIT_SHIFT_WLTMA_STVOQASKP<define BIT_GET_BITMA_STVOQASKP)                                                 	(((x) >> BIT_SHIFT_USTMA_STVOQASKP<<BIT_MASK_SYTMA_STVOQASKP<#define BIT_STSMA_STAGGT BIT(3)
#define BIT_HIRXSHFTT BIT(3)
#define BIT_HCSMA_STARBBWT BIT(3)

/* 2 REG_PATRXFFLBNDY	(OOffset 0x00C11 */

#define BIT_SHIFT_SDRXFFOVFL_RSV_V2 #define BIT_MASK_USRXFFOVFL_RSV_V2 f
#define BIT_REFXFFOVFL_RSV_V2)                                                 \
	(((x) & BIT_MASK_CMFXFFOVFL_RSV_V2<< BIT_SHIFT_RD_XFFOVFL_RSV_V2<define BIT_GET_BIFXFFOVFL_RSV_V2)                                               	(((x) >> BIT_SHIFT_RD_XFFOVFL_RSV_V2<<BIT_MASK_CMFXFFOVFL_RSV_V2</* 2 REG_PATRXFFLBNDY	(OOffset 0x00C11 */

#define BIT_SHIFT_SDTX_KBNUF_PGBNDY4fdefine BIT_MASK_SYTX_KBNUF_PGBNDY4ff
#define BIT_EFTX_KBNUF_PGBNDY)                                                 \
	(((x) & BIT_MASK_CMTX_KBNUF_PGBNDY<< BIT_SHIFT_WLTM_KBNUF_PGBNDY<define BIT_GET_BITM_KBNUF_PGBNDY)                                               	(((x) >> BIT_SHIFT_USTM_KBNUF_PGBNDY<<BIT_MASK_CMTX_KBNUF_PGBNDY</* 2 REG_PATRXFFLBNDY	(OOffset 0x00C11 */

#define BIT_SHIFT_SDRXFF0LBNDY_V2 fdefine BIT_MASK_USRXFF0LBNDY_V2 fx3ff
#define BIT_RPWXFF0LBNDY_V2)                                                   \
	(((x) & BIT_MASK_SYWXFF0LBNDY_V2<< BIT_SHIFT_RD_XFF0LBNDY_V2<define BIT_GET_BIFXFF0LBNDY_V2)                                                 	(((x) >> BIT_SHIFT_RD_XFF0LBNDY_V2<<BIT_MASK_SYWXFF0LBNDY_V2<#define BIT_SHIFT_SDRXFF0LRDPTR_V2 fdefine BIT_MASK_USRXFF0LRDPTR_V2 fx3ff
#define BIT_RPWXFF0LRDPTR_V2)                                                  \
	(((x) & BIT_MASK_EFWXFF0LRDPTR_V2<< BIT_SHIFT_RD_XFF0LRDPTR_V2<define BIT_GET_BIFXFF0LRDPTR_V2)                                                	(((x) >> BIT_SHIFT_RD_XFF0LRDPTR_V2<<BIT_MASK_EFWXFF0LRDPTR_V2<#define BIT_SHIFT_SDRXFF0LWTPTR_V2 fdefine BIT_MASK_USRXFF0LWTPTR_V2 fx3ff
#define BIT_RPWXFF0LWTPTR_V2)                                                  \
	(((x) & BIT_MASK_EFWXFF0LWTPTR_V2<< BIT_SHIFT_RD_XFF0LWTPTR_V2<define BIT_GET_BIFXFF0LWTPTR_V2)                                                	(((x) >> BIT_SHIFT_RD_XFF0LWTPTR_V2<<BIT_MASK_EFWXFF0LWTPTR_V2</* 2 REG_PAD_I_V2CASBOX(OOffset 0x00C11 */

#define BIT_CPIFT_B I2CASUSTATUS #define BIT_MASK_USI2CASUSTATUS f
#define BIT_REI2CASUSTATUS)                                                     \	(((x) & BIT_MASK_FSI2CASUSTATUS<< BIT_SHIFT_LBI2CASUSTATUS<define BIT_GET_BII2CASUSTATUS)                                                  	(((x) >> BIT_SHIFT_USI2CASUSTATUS<<BIT_MASK_FSI2CASUSTATUS</* 2 REG_PAFE1IMR	(OOffset 0x00C12 */

#define BIT_SHFSCSMA_S2_NE_MST_EN BIT(31)
#define BIT_GTFSCSMAE_M3ST_EN BIT(31)7#define BIT_GTFSCSMAE_M2ST_EN BIT(31)6#define BIT_GTFSCSMCNDE_P4ST_EN BIT(31)
#define BIT_FRESCSMCNDE_P3ST_EN BIT(31)4#define BIT_FRESCSMCNDE_P2ST_EN BIT(31)
#define BIT_FSM_CSMCNDE_P1ST_EN BIT(31)2#define BIT_FSM_CSMCNDE_P0ST_EN BIT(31)1#define BIT_FSM_CSMCUMD0ST_EN BIT(31)0#define BIT_FSM_CSMCUMD1ST_EN BIT(31)
#define BIT_BCM_CSMCNMD0ST_EN BIT(31)8#define BIT_PSM_CSMCNMD1ST_EN BIT(31)7#define BIT_GTFSCSMAE_MST_EN BIT(31)6#define BIT_GTFSCWWLANST_EN BIT(31)
#define BIT_FRESCSOUND_NE_MST_EN BIT(3114#define BIT_FRESCLPUSTBYST_EN BIT(311
#define BIT_FSM_CTRL_MTRST_EN BIT(3112#define BIT_FSM_CBF1_PRETOST_EN BIT(3111#define BIT_FSM_CBF0LPRETOST_EN BIT(3110#define BIT_FSM_CPTCL_RELEASMASKCIDST_EN BIT(319</* 2 REG_PAFE1IMR	(OOffset 0x00C12 */

#define BIT_SHFSCLTE_COEE2_NIT(9)6</* 2 REG_PAFE1IMR	(OOffset 0x00C12 */

#define BIT_SHFSCWLACTF_V1T_EN BIT(31
#define BIT_FRESCWLACTFNST_EN BIT(314#define BIT_FRESCBTD_CRT_EN BIT(31)</* 2 REG_PAFE1IMR	(OOffset 0x00C12 */

#define BIT_SHFSCS_MCUKILX_E2TOST2CAT_EN BIT(31)</* 2 REG_PAFE1IMR	(OOffset 0x00C12 */

#define BIT_SHFSCTRPC_TOST_EN B1 BIT(1)

/* 2 REG_PAFE1IMR	(OOffset 0x00C12 */

#define BIT_SHFSCSPC_O_TST_EN B1 BIT(1)0
/* 2 REG_PAFE1ISR	(OOffset 0x00C12 */

#define BIT_SHFSCSMA_S2_NE_MST_EIT(31)
#define BIT_GTFSCSMAE_M3ST_EIT(31)7#define BIT_GTFSCSMAE_M2ST_EIT(31)6#define BIT_GTFSCSMCNDE_P4ST_EIT(31)
#define BIT_FRESCSMCNDE_P3ST_EIT(31)4#define BIT_FRESCSMCNDE_P2ST_EIT(31)
#define BIT_FSM_CSMCNDE_P1ST_EIT(31)2#define BIT_FSM_CSMCNDE_P0ST_EIT(31)1#define BIT_FSM_CSMCUMD0ST_EIT(31)0#define BIT_FSM_CSMCUMD1ST_EIT(31)
#define BIT_BCM_CSMCNMD0ST_EIT(31)8#define BIT_PSM_CSMCNMD1ST_EIT(31)7#define BIT_GTFSCSMAE_MST_EIT(31)6#define BIT_GTFSCWWLANST_EIT(31)
#define BIT_FRESCSOUND_NE_MST_EIT(3114#define BIT_FRESCLPUSTBYST_EIT(311
#define BIT_FSM_CTRL_MTRST_EIT(3112#define BIT_FSM_CBF1_PRETOST_EIT(3111#define BIT_FSM_CBF0LPRETOST_EIT(3110#define BIT_FSM_CPTCL_RELEASMASKCIDST_EIT(319</* 2 REG_PAFE1ISR	(OOffset 0x00C12 */

#define BIT_SHFSCLTE_COEE2T_EIT(316</* 2 REG_PAFE1ISR	(OOffset 0x00C12 */

#define BIT_SHFSCWLACTF_V1T_EIT(31
#define BIT_FRESCWLACTFNST_EIT(314#define BIT_FRESCBDE_RE2T_ENT BIT(30)</* 2 REG_PAFE1ISR	(OOffset 0x00C12 */

#define BIT_SHFSCUKILX_E2TOST2CAT_EIT(31)</* 2 REG_PAFE1ISR	(OOffset 0x00C12 */

#define BIT_SHFSCTRPC_TOST_EIT(1)

/* 2 REG_PAFE1ISR	(OOffset 0x00C12 */

#define BIT_SHFSCSPC_O_TST_EIT(1)0
/* 2 REG_PAWM B	(OOffset 0x00C12 */

#define BIT_CPU_WMOGGLE IN	 B(1931<#define BIT_SHIFT_LPWM_MSKODI
#define BIT_MASK_RPWM_MSKODI0x7fdefine BIT_CPU_WMOKOD) (((x) & BIT_MASK_CK__WMOKOD<< BIT_SHIFT_CMD_WMOKOD<define BIT_GET_BIU_WMOKOD) (((x) &  BIT_SHIFT_CMD_WMOKOD<<BIT_MASK_CK__WMOKOD</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB7RT_EN BIT(31)1</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB6RT_EN BIT(31)0</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB5AT_EN BIT(31)9</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB4ST_EN BIT(31)8</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB3ST_EN BIT(31)7#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB2ST_EN BIT(31)6#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB1ST_EN BIT(31)5#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB0ST_EN BIT(31)4#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB7RT_EN BIT(3123#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB6RT_EN BIT(31)
#d* 2 REG_WLFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB5AT_EN BIT(31)1</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB4ST_EN BIT(31)0</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB3ST_EN BIT(3119</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB2ST_EN BIT(3118</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB1ST_EN BIT(31)7#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB0ST_EN BIT(31)6#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHU_MGQ_TXDONE_MST_EN BIT(31)
#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHSIFSCERRDSPECAT_EN BIT(3114#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCMGNTQIT		R_RELEASMAT_EN BIT(311
#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCMGNTQ_V1TOST_EN BIT(311
#d* 2 REG_WLFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCDA_S1CLPUT_EN BIT(3111#d* 2 REG_WLFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCDA_S1CHPUT_EN BIT(3110</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCDA_S0CLPUT_EN BIT(319</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCDA_S0CHPUT_EN BIT(318</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTRX REGT_EN BIT(317#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCC2H_W_READYST_EN BIT(316#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCHRCV1T_EN BIT(31
#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCH2CD_CRT_EN BIT(314#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCPKTINST_EN BIT(31
#d* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCR_MORHDLAT_EN BIT(31)</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCCCXUT_EN BIT(311</* 2 REG_PAFWIMR	(OOffset 0x00C13 */

#define BIT_SHFSCTCCLOSMAT_EN BIT(310</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB7RT_EIT(31)1</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB6RT_EIT(31)0</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB5NT BIT(29)

/* 2 REG_HIFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB4ST_EIT(31)8</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB3ST_EIT(31)7#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB2ST_EIT(31)6#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB1ST_EIT(31)
#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0O_MSKB0ST_EIT(31)4#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB7RT_EIT(3123#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB6RT_EIT(31)
#d* 2 REG_WLFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB5NT BIT(29)1</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB4ST_EIT(31)0</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB3ST_EIT(3119</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB2ST_EIT(3118</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB1ST_EIT(31)7#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCN0OR_MSKB0ST_EIT(31)6#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHU_MGQ_TXDONE_MST_EIT(31)
#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHSIFSCERRDSPECAT_EIT(3114#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCMGNTQIT		R_RELEASMAT_EIT(311
#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCMGNTQ_V1TOST_EIT(311
#d* 2 REG_WLFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCDA_S1CLPUT_EIT(3111#d* 2 REG_WLFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCDA_S1CHPUT_EIT(3110</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCDA_S0CLPUT_EIT(319</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCDA_S0CHPUT_EIT(318</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTRX REGT_EIT(317#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCC2H_W_READYST_EIT(316#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCHRCV1T_EIT(31
#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCH2CD_CRT_EIT(314#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCPKTINST_EIT(31
#d* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCR_MORHDLAT_EIT(31)</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCCCXUT_EIT(311</* 2 REG_PAFWISR	(OOffset 0x00C13 */

#define BIT_SHFSCTCCLOSMAT_EIT(310</* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C B_EARLYST_EN BIT(3123#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C B_EARLYST_EN BIT(3122#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C A_EARLYST_EN BIT(3121#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPUMGQ_TX_TIMER B_EARLYST_EN BIT(3120</* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C B_T_EN BIT(3119</* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C B_T_EN BIT(3118</* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C A_T_EN BIT(31)7#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPUMGQ_TX_TIMER B_T_EN BIT(31)6#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CPTIMER_OUT2N BIT(31)
#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CPTIMER_OUT1N BIT(3114#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CPTIMER_OUT0N BIT(311
#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT8N BIT(318</* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT7N BIT(317#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT6N BIT(316#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT5N BIT(31
#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT4N BIT(314#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT3N BIT(31
#d* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT2N BIT(31)</* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT1N BIT(311</* 2 REG_PAFTIMR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT0N BIT(310</* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C B_EARLYSRT_EIT(3123#d* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C B_EARLYSRT_EIT(312)</* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C A_EARLYSRT_EIT(3121</* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPUMGQ_TX_TIMER B_EARLYST_EIT(3120</* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C B_T_EIT(3119</* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C B_T_EIT(3118</* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CP_TIMER_C A_T_EIT(31)7#d* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPUMGQ_TX_TIMER B_T_EIT(31)6#d* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CPTIMER_OUT2NT_EIT(31)
#d* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CPTIMER_OUT1AT_EIT(3114#d* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CPTIMER_OUT0AT_EIT(311
#d* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT8NT_EIT(318</* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT7GT_EIT(317#d* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT6ST_EIT(316#d* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT51T_EIT(31
#d* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT4RT_EIT(314#d* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT3ST_EIT(31
#d* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT2AT_EIT(31)</* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT1UT_EIT(311</* 2 REG_PAFTISR	(OOffset 0x00C13 */

#define BIT_CPM_CGTINT0AT_EIT(310</* 2 REG_PA_KBNUF_G_CTRL2	(OOffset 0x00C14 */

#define BIT_SHIFT_LP_KBNUF_WRITEN BI
#define BIT_MASK_RP_KBNUF_WRITEN BIff
#define BIT_EF_KBNUF_WRITEN B)                                                 \
	(((x) & BIT_MASK_CM_KBNUF_WRITEN B<< BIT_SHIFT_BIIKBNUF_WRITEN B<define BIT_GET_BI_KBNUF_WRITEN B)                                               	(((x) >> BIT_SHIFT_USIKBNUF_WRITEN B<<BIT_MASK_CM_KBNUF_WRITEN B</* 2 REG_PA_KBNUF_G_CTRL2	(OOffset 0x00C14 */

#define BIT_SHTX RENUF_G_CIT(3123#d* 2 REG_PA_KBNUF_G_CTRL2	(OOffset 0x00C14 */

#define BIT_SHTX_KBNUF_G_CTV2IT(3120</* 2 REG_PA_KBNUF_G_CTRL2	(OOffset 0x00C14 */

#define BIT_SHRX_KBNUF_G_CIT(31)6#d* 2 REG_PA_KBNUF_G_CTRL2	(OOffset 0x00C14 */

#define BIT_SHIFT_LP_KBNUF_G_CTADDR4fdefine BIT_MASK_SY_KBNUF_G_CTADDR4ffff
#define BIT_EF_KBNUF_G_CTADDR)                                                 \
	(((x) & BIT_MASK_CM_KBNUF_G_CTADDR<< BIT_SHIFT_BIIKBNUF_G_CTADDR<define BIT_GET_BI_KBNUF_G_CTADDR)                                               	(((x) >> BIT_SHIFT_USIKBNUF_G_CTADDR<<BIT_MASK_CM_KBNUF_G_CTADDR<d* 2 REG_PA_KBNUF_G_CTTA_RE	(OOffset 0x00C14 */

#define BIT_SHIFT_SD_KBNUF_G_CTTA_RE	4fdefine BIT_MASK_SY_KBNUF_G_CTTA_RE	4ffffffffL
#define BIT_DE_KBNUF_G_CTTA_RE	)                                                 	(((x) >>BIT_MASK_CM_KBNUF_G_CTTA_RE	<< BIT_SHIFT_BIIKBNUF_G_CTTA_RE	<define BIT_GET_BI_KBNUF_G_CTTA_RE	)                                             	(((x) >> BIT_SHIFT_USIKBNUF_G_CTTA_RE	<<BIT_MASK_CM_KBNUF_G_CTTA_RE	<d* 2 REG_PA_KBNUF_G_CTTA_REH(OOffset 0x00C14 */

#define BIT_CPIFT_B _KBNUF_G_CTTA_REH4fdefine BIT_MASK_SY_KBNUF_G_CTTA_REH4ffffffffL
#define BIT_DE_KBNUF_G_CTTA_REH)                                                 	(((x) >>BIT_MASK_CM_KBNUF_G_CTTA_REH<< BIT_SHIFT_BIIKBNUF_G_CTTA_REH<define BIT_GET_BI_KBNUF_G_CTTA_REH)                                             	(((x) >> BIT_SHIFT_USIKBNUF_G_CTTA_REH>>BIT_MASK_CM_KBNUF_G_CTTA_REH</* 2 REG_PAWM B			(Offset 0x00C14 */

#define BIT_STIFT_WLL0S1TOSRCVY_NUM8
#define BIT_MASK_BIL0S1TOSRCVY_NUM8ff
#define BIT_EFL0S1TOSRCVY_NUM)                                                 \
	(((x) & BIT_MASK_CML0S1TOSRCVY_NUM<< BIT_SHIFT_LBK0S1TOSRCVY_NUM<define BIT_GET_BIL0S1TOSRCVY_NUM)                                               	(((x) >> BIT_SHIFT_USL0S1TOSRCVY_NUM<<BIT_MASK_CML0S1TOSRCVY_NUM<#define BIT_CPU_WM2OGGLE IN	 B(19)

#define BIT_SHIFT_FSU_WM2OKODI0define BIT_MASK_BIU_WM2OKODI0x7f
#define BIT_EFU_WM2OKOD) (((x) & BIT_MASK_CK__WM2OKOD<< BIT_SHIFT_CMD_WM2OKOD<define BIT_GET_BIU_WM2OKOD) (((x) &  BIT_SHIFT_CMD_WM2OKOD<<BIT_MASK_CK__WM2OKOD</* 2 REG_PATC0HK_EN_CPL_TIMEOUT_PS15 */

#define BIT_SHTC0T_EN BIT(31)6#define BIT_SHTC0DE_VIT(24)5#define BIT_SHTC0 BIT(31)4#d*efine BIT_SHIFT_FSTC0TA_R4fdefine BIT_MASK_SYTC0TA_R4ffffffffdefine BIT_SHTC0TA_R)   (x) & BIT_MASK_CMTC0TA_R<< BIT_SHIFT_WLTC0TA_R<define BIT_GET_BITC0TA_R)   (x) &  BIT_SHIFT_USTC0TA_R<<BIT_MASK_CMTC0TA_R</* 2 REG_PATC1HK_EN_CPL_TIMEOUT_PS15 */

#define BIT_SHTC1T_EN BIT(31)6#define BIT_SHTC1DE_VIT(24)5#define BIT_SHTC1 BIT(31)4#d*efine BIT_SHIFT_FSTC1TA_R4fdefine BIT_MASK_SYTC1TA_R4ffffffffdefine BIT_SHTC1TA_R)   (x) & BIT_MASK_CMTC1TA_R<< BIT_SHIFT_WLTC1TA_R<define BIT_GET_BITC1TA_R)   (x) &  BIT_SHIFT_USTC1TA_R<<BIT_MASK_CMTC1TA_R</* 2 REG_PATC2HK_EN_CPL_TIMEOUT_PS15 */

#define BIT_CPFC2T_EN BIT(31)6#define BIT_SHTC2DE_VIT(24)5#define BIT_SHTC2 BIT(31)4#d*efine BIT_SHIFT_FSTC2TA_R4fdefine BIT_MASK_SYTC2TA_R4ffffffffdefine BIT_SHTC2TA_R)   (x) & BIT_MASK_CMTC2TA_R<< BIT_SHIFT_WLTC2TA_R<define BIT_GET_BITC2TA_R)   (x) &  BIT_SHIFT_USTC2TA_R<<BIT_MASK_CMTC2TA_R</* 2 REG_PATC3HK_EN_CPL_TIMEOUT_PS15 */

#define BIT_STTC3T_EN BIT(31)6#define BIT_SHTC3DE_VIT(24)5#define BIT_SHTC3 BIT(31)4#d*efine BIT_SHIFT_FSTC3TA_R4fdefine BIT_MASK_SYTC3TA_R4ffffffffdefine BIT_SHTC3TA_R)   (x) & BIT_MASK_CMTC3TA_R<< BIT_SHIFT_WLTC3TA_R<define BIT_GET_BITC3TA_R)   (x) &  BIT_SHIFT_USTC3TA_R<<BIT_MASK_CMTC3TA_R</* 2 REG_PATC4HK_EN_CPL_TIMEOUT_PS16 */

#define BIT_SHTC4T_EN BIT(31)6#define BIT_SHTC4DE_VIT(24)5#define BIT_SHTC4 BIT(31)4#d*efine BIT_SHIFT_FSTC4TA_R4fdefine BIT_MASK_SYTC4TA_R4ffffffffdefine BIT_SHTC4TA_R)   (x) & BIT_MASK_CMTC4TA_R<< BIT_SHIFT_WLTC4TA_R<define BIT_GET_BITC4TA_R)   (x) &  BIT_SHIFT_USTC4TA_R<<BIT_MASK_CMTC4TA_R</* 2 REG_PATCUN_MABASE_CPL_TIMEOUT_PS16 */

#define BIT_SHIFT_SDTCUN_MABASE4fdefine BIT_MASK_SYTCUN_MABASE4fx3ff
define BIT_SHTCUN_MABASE)                                                     \
	(((x) & BIT_MASK_FSTCUN_MABASE<< BIT_SHIFT_WLTCUN_MABASE<define BIT_GET_BITCUN_MABASE)                                                   	(((x) >> BIT_SHIFT_USTCUN_MABASE<<BIT_MASK_FSTCUN_MABASE</* 2 REG_PATC5HK_EN_CPL_TIMEOUT_PS16 */

#define BIT_CPFC5T_EN BIT(31)6#d* 2 REG_PATC5HK_EN_CPL_TIMEOUT_PS16 */

#define BIT_CPFC5DE_VIT(24)5#define BIT_SHTC5 BIT(31)4#d*efine BIT_SHIFT_FSTC5TA_R4fdefine BIT_MASK_SYTC5TA_R4ffffffffdefine BIT_SHTC5TA_R)   (x) & BIT_MASK_CMTC5TA_R<< BIT_SHIFT_WLTC5TA_R<define BIT_GET_BITC5TA_R)   (x) &  BIT_SHIFT_USTC5TA_R<<BIT_MASK_CMTC5TA_R</* 2 REG_PATC6HK_EN_CPL_TIMEOUT_PS16 */

#define BIT_STTC6T_EN BIT(31)6#d* 2 REG_PATC6HK_EN_CPL_TIMEOUT_PS16 */

#define BIT_STTC6DE_VIT(24)5#define BIT_SHTC6 BIT(31)4#d*efine BIT_SHIFT_FSTC6TA_R4fdefine BIT_MASK_SYTC6TA_R4ffffffffdefine BIT_SHTC6TA_R)   (x) & BIT_MASK_CMTC6TA_R<< BIT_SHIFT_WLTC6TA_R<define BIT_GET_BITC6TA_R)   (x) &  BIT_SHIFT_USTC6TA_R<<BIT_MASK_CMTC6TA_R</* 2 REG_PAMBIST_FAIN_CPL_TIMEOUT_PS17 */

#define BIT_SHIFT_LP8051AMBIST_FAIN 2#define BIT_MASK_BI8051AMBIST_FAIN f
#define BIT_SD8051AMBIST_FAIN)                                                 \
	(((x) & BIT_MASK_CM8051AMBIST_FAIN<< BIT_SHIFT_WL8051AMBIST_FAIN<define BIT_GET_BI8051AMBIST_FAIN)                                               	(((x) >> BIT_SHIFT_US8051AMBIST_FAIN<<BIT_MASK_CM8051AMBIST_FAIN<#define BIT_SHIFT_LPB23IMBIST_FAIN 2#define BIT_MASK_RPB23IMBIST_FAIN 3
#define BIT_SEB23IMBIST_FAIN)                                                  \
	(((x) & BIT_MASK_EFB23IMBIST_FAIN<< BIT_SHIFT_USB23IMBIST_FAIN<define BIT_GET_BIB23IMBIST_FAIN)                                                	(((x) >> BIT_SHIFT_USB23IMBIST_FAIN<<BIT_MASK_CMB23IMBIST_FAIN<ddefine BIT_CPIFT_B __VREMBIST_FAIN 
#define BIT_MASK_BI__VREMBIST_FAIN fx3fdefine BIT_DE__VREMBIST_FAIN)                                                 \
	(((x) & BIT_MASK_CM__VREMBIST_FAIN<< BIT_SHIFT_BII_DREMBIST_FAIN<define BIT_GET_BI__VREMBIST_FAIN)                                               	(((x) >> BIT_SHIFT_USI_DREMBIST_FAIN<<BIT_MASK_CMI_DREMBIST_FAIN<d* 2 REG_PAMBIST_FAIN_CPL_TIMEOUT_PS17 */

#define BIT_SHIFT_LPMACUMBIST_FAIN fdefine BIT_MASK_BIMACUMBIST_FAIN fxfffdefine BIT_SHMACUMBIST_FAIN)                                                  \
	(((x) & BIT_MASK_EFMACUMBIST_FAIN<< BIT_SHIFT_BIMACUMBIST_FAIN<define BIT_GET_BIMACUMBIST_FAIN)                                                	(((x) >> BIT_SHIFT_USMACUMBIST_FAIN<<BIT_MASK_EFMACUMBIST_FAIN<d* 2 REG_PAMBIST_STARUSIAUSE(OOffset 0x00C17 */

#define BIT_SHIFT_SD8051AMBIST_STARUSIAUSE 2#define BIT_MASK_BI8051AMBIST_STARUSIAUSE f
#define BIT_SD8051AMBIST_STARUSIAUSE)                                            	(((x) & BIT_MASK_CM8051AMBIST_STARUSIAUSE                                	 << BIT_SHIFT_SD8051AMBIST_STARUSIAUSE define BIT_GET_BI8051AMBIST_STARUSIAUSE)                                        	(((x) >> BIT_SHIFT_US8051AMBIST_STARUSIAUSE                              	 BIT_MASK_SD8051AMBIST_STARUSIAUSE ddefine BIT_SHIFT_LPB23IMBIST_STARUSIAUSE 2#define BIT_MASK_RPB23IMBIST_STARUSIAUSE f
#define BIT_SEB23IMBIST_STARUSIAUSE)                                            
	(((x) & BIT_MASK_EFB23IMBIST_STARUSIAUSE                                 	 << BIT_SHIFT_SDB23IMBIST_STARUSIAUSE define BIT_GET_BIB23IMBIST_STARUSIAUSE)                                         	(((x) >> BIT_SHIFT_USB23IMBIST_STARUSIAUSE                               	 BIT_MASK_SDB23IMBIST_STARUSIAUSE ddefine BIT_CPIFT_B __VREMBIST_STARUSIAUSE 
#define BIT_MASK_BI__VREMBIST_STARUSIAUSE f
#fdefine BIT_DE__VREMBIST_STARUSIAUSE)                                            	(((x) & BIT_MASK_CM__VREMBIST_STARUSIAUSE                                	 << BIT_SHIFT_SD__VREMBIST_STARUSIAUSE define BIT_GET_BI__VREMBIST_STARUSIAUSE)                                        	(((x) >> BIT_SHIFT_US__VREMBIST_STARUSIAUSE                              	 BIT_MASK_SD__VREMBIST_STARUSIAUSE d* 2 REG_PAMBIST_STARUSIAUSE(OOffset 0x00C17 */

#define BIT_SHIFT_SDMACUMBIST_STARUSIAUSE fdefine BIT_MASK_BIMACUMBIST_STARUSIAUSE f
fffdefine BIT_SHMACUMBIST_STARUSIAUSE)                                            
	(((x) & BIT_MASK_EFMACUMBIST_STARUSIAUSE                                 	 << BIT_SHIFT_SDMACUMBIST_STARUSIAUSE define BIT_GET_BIMACUMBIST_STARUSIAUSE)                                         	(((x) >> BIT_SHIFT_USMACUMBIST_STARUSIAUSE                               	 BIT_MASK_SDMACUMBIST_STARUSIAUSE d* 2 REG_PAMBIST_NE_M_CPL_TIMEOUT_PS17 */

#define BIT_CPIFT_B 8051AMBIST_NE_M 2#define BIT_MASK_BI8051AMBIST_NE_M f
#define BIT_SD8051AMBIST_NE_M)                                                 \
	(((x) & BIT_MASK_CM8051AMBIST_NE_M<< BIT_SHIFT_WL8051AMBIST_NE_M<define BIT_GET_BI8051AMBIST_NE_M)                                               	(((x) >> BIT_SHIFT_US8051AMBIST_NE_M<<BIT_MASK_CM8051AMBIST_NE_M<ddefine BIT_SHIFT_LPB23IMBIST_NE_M 2#define BIT_MASK_RPB23IMBIST_NE_M f
#define BIT_SEB23IMBIST_NE_M)                                                 \

	(((x) & BIT_MASK_EFB23IMBIST_NE_M<< BIT_SHIFT_WLB23IMBIST_NE_M<define BIT_GET_BIB23IMBIST_NE_M)                                                	(((x) >> BIT_SHIFT_USB23IMBIST_NE_M<<BIT_MASK_CMB23IMBIST_NE_M<ddefine BIT_CPIFT_B __VREMBIST_NE_M 
#define BIT_MASK_BI__VREMBIST_NE_M f
#fdefine BIT_DE__VREMBIST_NE_M)                                                 \
	(((x) & BIT_MASK_CM__VREMBIST_NE_M<< BIT_SHIFT_BII_DREMBIST_NE_M<define BIT_GET_BI__VREMBIST_NE_M)                                               	(((x) >> BIT_SHIFT_US__VREMBIST_NE_M<<BIT_MASK_CMI_DREMBIST_NE_M<d* 2 REG_PAMBIST_NE_M_CPL_TIMEOUT_PS17 */

#define BIT_CPIFT_B MACUMBIST_NE_M fdefine BIT_MASK_BIMACUMBIST_NE_M f
fffdefine BIT_SHMACUMBIST_NE_M)                                                 \

	(((x) & BIT_MASK_EFMACUMBIST_NE_M<< BIT_SHIFT_BIMACUMBIST_NE_M<define BIT_GET_BIMACUMBIST_NE_M)                                                	(((x) >> BIT_SHIFT_USMACUMBIST_NE_M<<BIT_MASK_CMMACUMBIST_NE_M<d* 2 REG_PAMBIST_FAIN_NRM	(OOffset 0x00C17 */

#define BIT_STIFT_WLMBIST_FAIN_NRM	 fdefine BIT_MASK_BIMBIST_FAIN_NRM	 ffffffffL
#define BIT_DEMBIST_FAIN_NRM	)                                                 \
	(((x) & BIT_MASK_CMMBIST_FAIN_NRM	<< BIT_SHIFT_BIMBIST_FAIN_NRM	<define BIT_GET_BIMBIST_FAIN_NRM	)                                               	(((x) >> BIT_SHIFT_USMBIST_FAIN_NRM	<<BIT_MASK_CMMBIST_FAIN_NRM	<d* 2 REG_PAAES_DEC REGTA_R(OOffset 0x00C18 */

#define BIT_SHIFT_LPIPNFG		TADDR4fdefine BIT_MASK_SYIPNFG		TADDR4ff
#define BIT_EFIPNFG		TADDR)                                                     \	(((x) & BIT_MASK_FSIPNFG		TADDR<< BIT_SHIFT_LBIPNFG		TADDR<define BIT_GET_BIIPNFG		TADDR)                                                  	(((x) >> BIT_SHIFT_USIPNFG		TADDR<<BIT_MASK_FSIPNFG		TADDR<d* 2 REG_PAAES_DEC REGG		(OOffset 0x00C18 */

#define BIT_SHIFT_SDIPNFG		TTA_R4fdefine BIT_MASK_SYIPNFG		TTA_R4ffffffffL
#define BIT_DEIPNFG		TTA_R)                                                     \	(((x) & BIT_MASK_FSIPNFG		TTA_R<< BIT_SHIFT_WLIPNFG		TTA_R<define BIT_GET_BIIPNFG		TTA_R)                                                  	(((x) >> BIT_SHIFT_USIPNFG		TTA_R<<BIT_MASK_CMIPNFG		TTA_R<d* 2 REG_PATMETER	(OOffset 0x00C19 */

#define BIT_SHTEMP_VALIDIT(2031<#define BIT_SHIFT_LPTEMP_VALUM 2#define BIT_MASK_RPTEMP_VALUM f
#fdefine BIT_DETEMP_VALUM)   (x) & BIT_MASK_CMTEMP_VALUM<< BIT_SHIFT_WLTEMP_VALUM<define BIT_GET_BITEMP_VALUM)                                                   \	(((x) &  BIT_SHIFT_USTEMP_VALUM<<BIT_MASK_CMTEMP_VALUM<#define BIT_SHIFT_LPG_PATMETERIMER B #define BIT_MASK_USR_PATMETERIMER B f
fffdefine BIT_SHR_PATMETERIMER B)                                                  	(((x) >>&IT_MASK_USR_PATMETERIMER B<< BIT_SHIFT_RD__PATMETERIMER B<define BIT_GET_BIR_PATMETERIMER B)                                              	(((x) >> BIT_SHIFT_RD__PATMETERIMER B<<&IT_MASK_USR_PATMETERIMER B<#define BIT_SHIFT_LPG_PATEMP_DEL_R4#define BIT_MASK_CPG_PATEMP_DEL_R4f
#fdefine BIT_DEG_PATEMP_DEL_R)                                                 \

	(((x) & BIT_MASK_EFG_PATEMP_DEL_R<< BIT_SHIFT_RD__PATEMP_DEL_R<define BIT_GET_BIR_PATEMP_DEL_R)                                                	(((x) >> BIT_SHIFT_RD__PATEMP_DEL_R<<BIT_MASK_EFG_PATEMP_DEL_R<
define BIT_SHR_PATMETERI BIT(310</* 2 REG_PAOSCHK_CLKL2	(OOffset 0x00C19 */

#define BIT_SHIFT_SDOSCHK_CLKLKGEN_08
#define BIT_MASK_BIOSCHK_CLKLKGEN_08ffffffdefine BIT_MAOSCHK_CLKLKGEN_0)                                                  	(((x) >>&IT_MASK_USOSCHK_CLKLKGEN_0<< BIT_SHIFT_RDOSCHK_CLKLKGEN_0<define BIT_GET_BIOSCHK_CLKLKGEN_0)                                              	(((x) >> BIT_SHIFT_RDOSCHK_CLKLKGEN_0<<&IT_MASK_USOSCHK_CLKLKGEN_0</* 2 REG_PAOSCHK_CLKL2	(OOffset 0x00C19 */

#define BIT_SHIFT_SDOSCHK_CLRES_COMP #define BIT_MASK_RPOSCHK_CLRES_COMP f
#define BIT_SEOSCHK_CLRES_COMP)                                                  	(((x) >>&IT_MASK_USOSCHK_CLRES_COMP<< BIT_SHIFT_RDOSCHK_CLRES_COMP<define BIT_GET_BIOSCHK_CLRES_COMP)                                              	(((x) >> BIT_SHIFT_RDOSCHK_CLRES_COMP<<&IT_MASK_USOSCHK_CLRES_COMP<
define BIT_SEOSCHK_CLOUTUS BIT(203</* 2 REG_PAOSCHK_CLKL2	(OOffset 0x00C19 */

#define BIT_SHISO_WL_2AOSCHK_CIT(201</* 2 REG_PAOSCHK_CLKL2	(OOffset 0x00C19 */

#define BIT_SHPOW_CKGENIT(310</* 2 REG_PAK_CLKAL_G_P1(OOffset 0x00C19 */

#define BIT_CPKAL_K_CLREG_WRIT(2031<#efine BIT_CPKAL_K_CLG_CTS BIT(2022<#define BIT_SHIFT_SDKAL_K_CLREG_ADDR4
#define BIT_MASK_BIKAL_K_CLREG_ADDR4f
#fdefine BIT_DEKAL_K_CLREG_ADDR)                                                  	(((x) >>&IT_MASK_USKAL_K_CLREG_ADDR<< BIT_SHIFT_CMDAL_K_CLREG_ADDR<define BIT_GET_BIKAL_K_CLREG_ADDR)                                              	(((x) >> BIT_SHIFT_RDKAL_K_CLREG_ADDR<<&IT_MASK_USKAL_K_CLREG_ADDR</* 2 REG_PAK_CLKAL_G_P1(OOffset 0x00C19 */

#define BIT_CPIFT_RDKAL_K_CLREG_TA_R4fdefine BIT_MASK_SYKAL_K_CLREG_TA_R4ffffffdefine BIT_MAKAL_K_CLREG_TA_R)                                                  	(((x) >>&IT_MASK_USKAL_K_CLREG_TA_R<< BIT_SHIFT_WLKAL_K_CLREG_TA_R<define BIT_GET_BIKAL_K_CLREG_TA_R)                                              	(((x) >> BIT_SHIFT_RDKAL_K_CLREG_TA_R<<BIT_MASK_CMKAL_K_CLREG_TA_R<d* 2 REG_PAW2HEVT	(OOffset 0x00C1A */

#define BIT_SHIFT_LPW2HEVT_MSG4fdefine BIT_MASK_SYK2HEVT_MSG4ffffffffL
























#define BIT_DEK2HEVT_MSG) (((x) & BIT_MASK_CK_2HEVT_MSG<< BIT_SHIFT_WLK2HEVT_MSG<define BIT_GET_BIK2HEVT_MSG) (((((((((((((((((((((((((((((((((((((((((((((((((((	(((x) >> BIT_SHIFT_RDK2HEVT_MSG<<BIT_MASK_CK_2HEVT_MSG<d* 2 REG_PASW_DEFINED_PAGE1(OOffset 0x00C1B */

#define BIT_CPIFT_RDSW_DEFINED_PAGE14fdefine BIT_MASK_SYSW_DEFINED_PAGE14ffffffffL








#define BIT_DESW_DEFINED_PAGE1)                                                  	(((x) >>&IT_MASK_USSW_DEFINED_PAGE1<< BIT_SHIFT_WLSW_DEFINED_PAGE1<define BIT_GET_BISW_DEFINED_PAGE1)                                              	(((x) >> BIT_SHIFT_RDSW_DEFINED_PAGE1<<&IT_MASK_USSW_DEFINED_PAGE1<d* 2 REG_PAMCUTSSHI	(OOffset 0x00C1C */

#define BIT_SHIFT_LPMCUDMSG_I fdefine BIT_MASK_BIMCUDMSG_I ffffffffL
#define BIT_DEMCUDMSG_I) (((x) & BIT_MASK_CKMCUDMSG_I<< BIT_SHIFT_BIMCUDMSG_I<define BIT_GET_BIMCUDMSG_I) (((x) &  BIT_SHIFT_USMCUDMSG_I<<BIT_MASK_CKMCUDMSG_I<d* 2 REG_PAMCUTSSHII	(OOffset 0x00C1C */

#define BIT_SHIFT_SDMCUDMSG_II fdefine BIT_MASK_BIMCUDMSG_II ffffffffL
#define BIT_DEMCUDMSG_II) (((x) & BIT_MASK_CKMCUDMSG_II<< BIT_SHIFT_BIMCUDMSG_II<define BIT_GET_BIMCUDMSG_II) (((((((((((((((((((((((((((((((((((((((((((((((((((	(((x) >> BIT_SHIFT_RDMCUDMSG_II<<BIT_MASK_CKMCUDMSG_II</* 2 REG_PAFMETHR	(OOffset 0x00C1C */

#define BIT_CPFMSG_I_EIT(31)1</*efine BIT_SHIFT_SDFW_MSG4fdefine BIT_MASK_SYFW_MSG4ffffffffL
#define BIT_DEFW_MSG) (((x) & BIT_MASK_CKFW_MSG<< BIT_SHIFT_BIFW_MSG<define BIT_GET_BIFW_MSG) (((x) &  BIT_SHIFT_RDFW_MSG<<BIT_MASK_CKFW_MSG</* 2 REG_PAHMETFR	(OOffset 0x00C1C */

#define BIT_STIFT_WLHRCV1MSG42#define BIT_MASK_RPHRCV1MSG4ff
#define BIT_EFHRCV1MSG) (((x) & BIT_MASK_CKHRCV1MSG<< BIT_SHIFT_BIHRCV1MSG<define BIT_GET_BIHRCV1MSG) (((x) &  BIT_SHIFT_RDHRCV1MSG<<BIT_MASK_CKHRCV1MSG<#define BIT_SHINT_BOX3IT(203</efine BIT_SHINT_BOX2IT(312</efine BIT_SHINT_BOXBIT(1)

/efine BIT_SHINT_BOX0IT(310</* 2 REG_PAHMEBOX0	(OOffset 0x00C1D */

#define BIT_SHIFT_LPHOST_MSG_08fdefine BIT_MASK_RPHOST_MSG_08ffffffffL
#define BIT_DEHOST_MSG_0) (((x) & BIT_MASK_CKHOST_MSG_0<< BIT_SHIFT_BIHOST_MSG_0<define BIT_GET_BIHOST_MSG_0) (((((((((((((((((((((((((((((((((((((((((((((((((((	(((x) >> BIT_SHIFT_RDHOST_MSG_0<<BIT_MASK_CKHOST_MSG_0</* 2 REG_PAHMEBOX1	(OOffset 0x00C1D */

#define BIT_SHIFT_SDHOST_MSG_14fdefine BIT_MASK_SYHOST_MSG_14ffffffffL
#define BIT_DEHOST_MSG_1) (((x) & BIT_MASK_CKHOST_MSG_1<< BIT_SHIFT_WLHOST_MSG_1<define BIT_GET_BIHOST_MSG_1)                                                  ((	(((x) >> BIT_SHIFT_RDHOST_MSG_1& BIT_MASK_CKHOST_MSG_1</* 2 REG_PAHMEBOX			(Offset 0x00C1D */

#define BIT_CPIFT_RDHOST_MSG_2 fdefine BIT_MASK_USHOST_MSG_2 ffffffffL
#define BIT_DEHOST_MSG_2) (((x) & BIT_MASK_CKHOST_MSG_2<< BIT_SHIFT_RDHOST_MSG_2<define BIT_GET_BIHOST_MSG_2)                                                  \
	(((x) &  BIT_SHIFT_RDHOST_MSG_2& BIT_MASK_CKHOST_MSG_2</* 2 REG_PAHMEBOX3		(Offset 0x00C1D */

#define BIT_STIFT_WLHOST_MSG_3 fdefine BIT_MASK_USHOST_MSG_3 ffffffffL
#define BIT_DEHOST_MSG_3) (((x) & BIT_MASK_CKHOST_MSG_3<< BIT_SHIFT_RDHOST_MSG_3<define BIT_GET_BIHOST_MSG_3)                                                  \
	(((x) &  BIT_SHIFT_RDHOST_MSG_3& BIT_MASK_CKHOST_MSG_3</* 2 REG_PALLSHINIT	(OOffset 0x00C1E */

#define BIT_SHIFT_LPLLTE_RWM 3fdefine BIT_MASK_USLLTE_RWM f
#define BIT_SELLTE_RWM) (((x) & BIT_MASK_CKLLTE_RWM<< BIT_SHIFT_LBKLTE_RWM<define BIT_GET_BILLTE_RWM) (((x) &  BIT_SHIFT_USLLTE_RWM<<BIT_MASK_CKLLTE_RWM</* 2 REG_PALLSHINIT	(OOffset 0x00C1E */

#define BIT_SHIFT_LPLLTINI_PTA_REV18
#define BIT_MASK_BILLTINI_PTA_REV18f
fffdefine BIT_SHLLTINI_PTA_REV1)                                                 \
	(((x) & BIT_MASK_CMLLTINI_PTA_REV1<< BIT_SHIFT_LBKLTINI_PTA_REV1<define BIT_GET_BILLTINI_PTA_REV1)                                               	(((x) >> BIT_SHIFT_USLLTINI_PTA_REV1<<BIT_MASK_CMLLTINI_PTA_REV1</* 2 REG_PALLSHINIT	(OOffset 0x00C1E */

#define BIT_SHIFT_LPLLTINI_HTA_REV18fdefine BIT_MASK_BILLTINI_HTA_REV18f
fffdefine BIT_SHLLTINI_HTA_REV1)                                                 \
	(((x) & BIT_MASK_CMLLTINI_HTA_REV1<< BIT_SHIFT_LBKLTINI_HTA_REV1<define BIT_GET_BILLTINI_HTA_REV1)                                               	(((x) >> BIT_SHIFT_USLLTINI_HTA_REV1<<BIT_MASK_CMLLTINI_HTA_REV1</* 2 REG_PALLSHINIT_ADDR(OOffset 0x00C1E */

#define BIT_SHIFT_SDLLTINI_ADDREV18fdefine BIT_MASK_BILLTINI_ADDREV18f
fffdefine BIT_SHLLTINI_ADDREV1)                                                 \

	(((x) & BIT_MASK_EFLLTINI_ADDREV1<< BIT_SHIFT_LBKLTINI_ADDREV1<define BIT_GET_BILLTINI_ADDREV1)                                                	(((x) >> BIT_SHIFT_USLLTINI_ADDREV1<<BIT_MASK_EFLLTINI_ADDREV1</* 2 REG_PABB_ACCESSLKL2	(OOffset 0x00C1E */

#define BIT_CPIFT_RDBB_WRITENREAD 3fdefine BIT_MASK_USBB_WRITENREAD f
#define BIT_SEBB_WRITENREAD)                                                     	(((x) & BIT_MASK_EFBB_WRITENREAD<< BIT_SHIFT_LBBB_WRITENREAD<define BIT_GET_BIBB_WRITENREAD)                                                 	(((x) >> BIT_SHIFT_USBB_WRITENREAD<<BIT_MASK_EFBB_WRITENREAD</* 2 REG_PABB_ACCESSLKL2	(OOffset 0x00C1E */

#define BIT_CPIFT_RDBB_WRITENEN 1#define BIT_MASK_CPBB_WRITENEN f
#define BIT_REBB_WRITENEN)                                                     \ 	(((x) & BIT_MASK_EFBB_WRITEN B<< BIT_SHIFT_BIBB_WRITEN B<define BIT_GET_BIBB_WRITEN B)                                                 \
	(((x) &  BIT_SHIFT_USBB_WRITEN B<<BIT_MASK_CMBB_WRITEN B<ddefine BIT_CPIFT_RDBB_ADDR4#define BIT_MASK_CPBB_ADDR4ffff
define BIT_REBB_ADDR)   (x) & BIT_MASK_EFBB_ADDR<< BIT_SHIFT_CMBB_ADDR<define BIT_GET_BIBB_ADDR)   (x) &  BIT_SHIFT_USBB_ADDR<<&IT_MASK_USBB_ADDR<d* 2 REG_PABB_ACCESSLKL2	(OOffset 0x00C1E */

#define BIT_CPBB_ERRACCIT(310</* 2 REG_PABB_ACCESSLTA_R(OOffset 0x00C1E */

#define BIT_STIFT_WLBB_TA_R4fdefine BIT_MASK_SYBB_TA_R4ffffffffL
#define BIT_DEBB_TA_R)   (x) & BIT_MASK_EFBB_TA_R<< BIT_SHIFT_WLBB_TA_R<define BIT_GET_BIBB_TA_R)   (x) &  BIT_SHIFT_USBB_TA_R<<BIT_MASK_EFBB_TA_R</* 2 REG_PAHMEBOX_E0	(OOffset 0x00C1F */

#define BIT_SHIFT_LPHMEBOX_E0 fdefine BIT_MASK_USHMEBOX_E0 ffffffffL
#define BIT_DEHMEBOX_E0) (((x) & BIT_MASK_CKHMEBOX_E0<< BIT_SHIFT_RDHMEBOX_E0<define BIT_GET_BIHMEBOX_E0) (((x) &  BIT_SHIFT_RDHMEBOX_E0<<BIT_MASK_CKHMEBOX_E0</* 2 REG_PAHMEBOX_E1	(OOffset 0x00C1F */

#define BIT_SHIFT_SDHMEBOX_E1 fdefine BIT_MASK_USHMEBOX_E14ffffffffL
#define BIT_DEHMEBOX_E1) (((x) & BIT_MASK_CKHMEBOX_E1<< BIT_SHIFT_WLHMEBOX_E1<define BIT_GET_BIHMEBOX_E1) (((x) &  BIT_SHIFT_RDHMEBOX_E1& BIT_MASK_CKHMEBOX_E1</* 2 REG_PAHMEBOX_E			(Offset 0x00C1F */

#define BIT_CPIFT_RDHMEBOX_E	 fdefine BIT_MASK_USHMEBOX_E2 ffffffffL
#define BIT_DEHMEBOX_E2) (((x) & BIT_MASK_CKHMEBOX_E2<< BIT_SHIFT_RDHMEBOX_E2<define BIT_GET_BIHMEBOX_E2) (((x) &  BIT_SHIFT_RDHMEBOX_E2& BIT_MASK_CKHMEBOX_E2</* 2 REG_PAHMEBOX_E3		(Offset 0x00C1F */

#define BIT_STLD_RQPBIT(31)1</*efine BIT_CPIFT_RDHMEBOX_E3 fdefine BIT_MASK_USHMEBOX_E3 ffffffffL
#define BIT_DEHMEBOX_E3) (((x) & BIT_MASK_CKHMEBOX_E3<< BIT_SHIFT_RDHMEBOX_E3<define BIT_GET_BIHMEBOX_E3) (((x) &  BIT_SHIFT_RDHMEBOX_E3& BIT_MASK_CKHMEBOX_E3</* 2 REG_PAFIFOPAGELKL2	_1(OOffset 0x00C20 */

#define BIT_SHIFT_LPTX_OQRDHE_FREE_SPACEEV18
#define BIT_MASK_BITX_OQRDHE_FREE_SPACEEV18ff
#define BIT_EFTX_OQRDHE_FREE_SPACEEV1)                                           	(((x) & BIT_MASK_FSTX_OQRDHE_FREE_SPACEEV1                               	 << BIT_SHIFT_SDTX_OQRDHE_FREE_SPACEEV1 define BIT_GET_BITX_OQRDHE_FREE_SPACEEV1)                                       	(((x) &  BIT_SHIFT_USTX_OQRDHE_FREE_SPACEEV1                             	 BIT_MASK_SDTX_OQRDHE_FREE_SPACEEV1 d* 2 REG_PAFIFOPAGELKL2	_1(OOffset 0x00C20 */

#define BIT_SHIFT_LPTX_OQRDNL_FREE_SPACEEV18fdefine BIT_MASK_BITX_OQRDNL_FREE_SPACEEV18ff
#define BIT_EFTX_OQRDNL_FREE_SPACEEV1)                                           	(((x) & BIT_MASK_FSTX_OQRDNL_FREE_SPACEEV1                               	 << BIT_SHIFT_SDTX_OQRDNL_FREE_SPACEEV1 define BIT_GET_BITX_OQRDNL_FREE_SPACEEV1)                                       	(((x) &  BIT_SHIFT_USTX_OQRDNL_FREE_SPACEEV1                             	 BIT_MASK_SDTX_OQRDNL_FREE_SPACEEV1 d* 2 REG_PAFIFOPAGELKL2	_2(OOffset 0x00C20 */

#define BIT_SHBDE_VALID_1EV18T(31)1</* 2 REG_PAFIFOPAGELKL2	_2(OOffset 0x00C20 */

#define BIT_SHIFT_USBDE_HEAD_1EV18
#define BIT_MASK_BIBDE_HEAD_1EV18f
fffdefine BIT_SHBDE_HEAD_1EV1)                                                     	(((x) & BIT_MASK_EFBDE_HEAD_1EV1<< BIT_SHIFT_WLBDE_HEAD_1EV1<define BIT_GET_BIBDE_HEAD_1EV1)                                                 	(((x) &  BIT_SHIFT_USBDE_HEAD_1EV1<<BIT_MASK_EFBDE_HEAD_1EV1<#define BIT_SHBDE_VALID_V18T(31)
#d* 2 REG_PAFIFOPAGELKL2	_2(OOffset 0x00C20 */

#define BIT_SHIFT_USBDE_HEAD_V18fdefine BIT_MASK_BIBDE_HEAD_V18f
fffdefine BIT_SHBDE_HEAD_V1)                                                       	(((x) & BIT_MASK_EFBDE_HEAD_V1<< BIT_SHIFT_WLBDE_HEAD_V1<define BIT_GET_BIBDE_HEAD_V1)                                                 \
	(((x) &  BIT_SHIFT_USBDE_HEAD_V1<<BIT_MASK_EFBDE_HEAD_V1 d* 2 REG_PAAUTOALLSHV1	(OOffset 0x00C20 */

#define BIT_CPIFT_RDMAXDTX_PKT_FORMB23IAND_SDIO_V182#define BIT_MASK_RPMAXDTX_PKT_FORMB23IAND_SDIO_V18ff
#define BIT_EFMAXDTX_PKT_FORMB23IAND_SDIO_V1)                                    	(((x) & BIT_MASK_EFMAXDTX_PKT_FORMB23IAND_SDIO_V1                        	 << BIT_SHIFT_SDMAXDTX_PKT_FORMB23IAND_SDIO_V1 define BIT_GET_BIMAXDTX_PKT_FORMB23IAND_SDIO_V1)                                	(((x) >> BIT_SHIFT_USMAXDTX_PKT_FORMB23IAND_SDIO_V1                      	 BIT_MASK_SDMAXDTX_PKT_FORMB23IAND_SDIO_V1 d* 2 REG_PAAUTOALLSHV1	(OOffset 0x00C20 */

#define BIT_CPIFT_RDLLSHFREE_PAGELV18#define BIT_MASK_USLLSHFREE_PAGELV18ffffffdefine BIT_MALLSHFREE_PAGELV1)                                                  	(((x) >>&IT_MASK_USLLSHFREE_PAGELV1<< BIT_SHIFT_LBKLTHFREE_PAGELV1<define BIT_GET_BILLTHFREE_PAGELV1)                                              	(((x) >> BIT_SHIFT_USLLTHFREE_PAGELV1<<&IT_MASK_USLLSHFREE_PAGELV1<d* 2 REG_PADWBDE0HK_EN_CPL_TIMEOUT_PS20 */

#define BIT_CPIFT_RDBLK_DESC_NUM8#define BIT_MASK_RPBLK_DESC_NUM8f
#define BIT_REBLK_DESC_NUM)                                                      	(((x) & BIT_MASK_EFBLK_DESC_NUM<< BIT_SHIFT_WLBLK_DESC_NUM<define BIT_GET_BIBLK_DESC_NUM)                                                  	(((x) &  BIT_SHIFT_USBLK_DESC_NUM<<BIT_MASK_EFBLK_DESC_NUM<d* 2 REG_PAAUTOALLSHV1	(OOffset 0x00C20 */

#define BIT_CPRFBDE_HEAD_S BIT(203</efine BIT_CPRFENFBDE_SW_HEAD_S BIT(202<define BIT_GELLSHG_CTS BIT(201<define BIT_GEAUTOAINIT_LLSHV1IT(310</* 2 REG_PATXDMA_OFFS_BIKHK(OOffset 0x00C20 */

#define BIT_STEMIKHKSUMAFIBIT(31)1</efine BIT_STEMNMI_DREDMA_KODIB(31)0</* 2 REG_PATXDMA_OFFS_BIKHK(OOffset 0x00C20 */

#define BIT_STENATXQUELKLRIT(20)

/efine BIT_STENAI_DREFIFO_DE_VIT(24)8</* 2 REG_PATXDMA_OFFS_BIKHK(OOffset 0x00C20 */

#define BIT_STIFT_US_G_UNDERIMHEV18
#define BIT_MASK_BI_G_UNDERIMHEV18f
fffdefine BIT_SH_G_UNDERIMHEV1)                                                 \

	(((x) & BIT_MASK_EF_G_UNDERIMHEV1<< BIT_SHIFT_BIIG_UNDERIMHEV1<define BIT_GET_BI_G_UNDERIMHEV1)                                                	(((x) >> BIT_SHIFT_US_G_UNDERIMHEV1<<BIT_MASK_EF_G_UNDERIMHEV1</* 2 REG_PATXDMA_OFFS_BIKHK(OOffset 0x00C20 */

#define BIT_STRESTORECH2C_ADDRESS8T(31)
#d* 2 REG_PATXDMA_OFFS_BIKHK(OOffset 0x00C20 */

#define BIT_STIDIO_TXDESC_KHKSUMA BIT(311
#define BIT_STRSSTRDPTRIT(2012#define BIT_STRSSTWRPTRIT(2011<#efine BIT_CPKHEF_G_MHE BIT(3110<define BIT_GEDROPTTA_RE BIT(319<#efine BIT_CPKHECK_OFFS_BI BIT(318</*efine BIT_STIFT_USKHECK_OFFS_B4fdefine BIT_MASK_SYKHECK_OFFS_B4ff
#define BIT_EFKHECK_OFFS_B)                                                      	(((x) & BIT_MASK_EFKHECK_OFFS_B<< BIT_SHIFT_WLKHECK_OFFS_B<define BIT_GET_BIKHECK_OFFS_B)                                                  	(((x) >> BIT_SHIFT_RDKHECK_OFFS_B<<BIT_MASK_EFKHECK_OFFS_B<d* 2 REG_PATXDMA_STATUS(OOffset 0x00C21 */

#define BIT_SHHI_OQRDUDBIT(31)7#define BIT_SHHI_OQRDOVFIT(31)6#define BIT_SHPAYLOAD_KHKSUMA RR8T(31)
#define BIT_SHPAYLOAD_UDBIT(31)4#define BIT_SHPAYLOAD_OVFIT(31)
#define BIT_STDSC_KHKSUMAFAIN T(2012#define BIT_STUNKNOWN_QS BIT(2011</efine BIT_STEP_QS B_DIFFIT(3110<define BIT_GETX_OFFSTUNMATCHIT(319<#efine BIT_CPTXOQRDUDBIT(318<#efine BIT_CPTXOQRDOVFIT(317<#efine BIT_CPTXDMA_SFFDUDBIT(316#define BIT_SHTXDMA_SFFDOVFIT(315<define BIT_GELLSHNULLF_GIT(314#define BIT_SHPAGELUDBIT(313#define BIT_SHPAGELOVFIT(312#define BIT_SHTXFFD_G_UDBIT(311</efine BIT_SHTXFFD_G_OVFIT(310</* 2 REG_PATQPNT1	(OOffset 0x00C21 */

#define BIT_CPIFT_RDHPQHHIGHIMHEV18
#define BIT_MASK_BIHPQHHIGHIMHEV18f
fffdefine BIT_SHHPQHHIGHIMHEV1)                                                 \

	(((x) & BIT_MASK_EFHPQHHIGHIMHEV1<< BIT_SHIFT_RDHPQHHIGHIMHEV1<define BIT_GET_BIHPQHHIGHIMHEV1)                                                	(((x) &  BIT_SHIFT_RDHPQHHIGHIMHEV1<<BIT_MASK_EFHPQHHIGHIMHEV1</* 2 REG_PATQPNT1	(OOffset 0x00C21 */

#define BIT_CPIFT_RDHPQHLOWIMHEV18fdefine BIT_MASK_BIHPQHLOWIMHEV18f
fffdefine BIT_SHHPQHLOWIMHEV1)                                                     	(((x) & BIT_MASK_EFHPQHLOWIMHEV1<< BIT_SHIFT_RDHPQHLOWIMHEV1<define BIT_GET_BIHPQHLOWIMHEV1)                                                 	(((x) &  BIT_SHIFT_RDHPQHLOWIMHEV1<<BIT_MASK_EFHPQHLOWIMHEV1</* 2 REG_PATQPNT			(Offset 0x00C21 */

#define BIT_STIFT_USNPQHHIGHIMHEV18
#define BIT_MASK_BINPQHHIGHIMHEV18f
fffdefine BIT_SHNPQHHIGHIMHEV1)                                                 \

	(((x) & BIT_MASK_EFNPQHHIGHIMHEV1<< BIT_SHIFT_RDNPQHHIGHIMHEV1<define BIT_GET_BINPQHHIGHIMHEV1)                                                	(((x) &  BIT_SHIFT_RDNPQHHIGHIMHEV1<<BIT_MASK_EFNPQHHIGHIMHEV1</* 2 REG_PATQPNT			(Offset 0x00C21 */

#define BIT_STIFT_USNPQHLOWIMHEV18fdefine BIT_MASK_BINPQHLOWIMHEV18f
fffdefine BIT_SHNPQHLOWIMHEV1)                                                     	(((x) & BIT_MASK_EFNPQHLOWIMHEV1<< BIT_SHIFT_RDNPQHLOWIMHEV1<define BIT_GET_BINPQHLOWIMHEV1)                                                 	(((x) &  BIT_SHIFT_RDNPQHLOWIMHEV1<<BIT_MASK_EFNPQHLOWIMHEV1</* 2 REG_PATQPNT3		(Offset 0x00C22 */

#define BIT_SHIFT_LPLPQHHIGHIMHEV18
#define BIT_MASK_BILPQHHIGHIMHEV18f
fffdefine BIT_SHLPQHHIGHIMHEV1)                                                 \

	(((x) & BIT_MASK_EFLPQHHIGHIMHEV1<< BIT_SHIFT_RDLPQHHIGHIMHEV1<define BIT_GET_BILPQHHIGHIMHEV1)                                                	(((x) >> BIT_SHIFT_USLPQHHIGHIMHEV1<<BIT_MASK_EFLPQHHIGHIMHEV1<d* 2 REG_PATQPNT3		(Offset 0x00C22 */

#define BIT_SHIFT_LPLPQHLOWIMHEV18fdefine BIT_MASK_BILPQHLOWIMHEV18f
fffdefine BIT_SHLPQHLOWIMHEV1)                                                     	(((x) & BIT_MASK_EFLPQHLOWIMHEV1<< BIT_SHIFT_RDLPQHLOWIMHEV1<define BIT_GET_BILPQHLOWIMHEV1)                                                 	(((x) >> BIT_SHIFT_USLPQHLOWIMHEV1<<BIT_MASK_EFLPQHLOWIMHEV1</* 2 REG_PATQPNT4		(Offset 0x00C22 */

#define BIT_SHIFT_USEXQHHIGHIMHEV18
#define BIT_MASK_BIEXQHHIGHIMHEV18f
fffdefine BIT_SHEXQHHIGHIMHEV1)                                                 \

	(((x) & BIT_MASK_EFEXQHHIGHIMHEV1<< BIT_SHIFT_RDEXQHHIGHIMHEV1<define BIT_GET_BIEXQHHIGHIMHEV1)                                                	(((x) >> BIT_SHIFT_USEXQHHIGHIMHEV1<<BIT_MASK_EFEXQHHIGHIMHEV1</* 2 REG_PATQPNT4		(Offset 0x00C22 */

#define BIT_SHIFT_USEXQHLOWIMHEV18fdefine BIT_MASK_BIEXQHLOWIMHEV18f
fffdefine BIT_SHEXQHLOWIMHEV1)                                                     	(((x) & BIT_MASK_EFEXQHLOWIMHEV1<< BIT_SHIFT_RDEXQHLOWIMHEV1<define BIT_GET_BIEXQHLOWIMHEV1)                                                 	(((x) >> BIT_SHIFT_USEXQHLOWIMHEV1<<BIT_MASK_EFEXQHLOWIMHEV1<d* 2 REG_PARQPBLKL2	_1(OOOffset 0x00C22 */

#define BIT_CPIFT_RDTX_KBNUMAH8
#define BIT_MASK_BITX_KBNUMAH8ffffffdefine BIT_MATX_KBNUMAH) (((x) & BIT_MASK_CKTX_KBNUMAH<< BIT_SHIFT_WLTX_KBNUMAH<define BIT_GET_BITX_KBNUMAH) ((                                                 	(((x) &  BIT_SHIFT_USTX_KBNUMAH<<BIT_MASK_CKTX_KBNUMAH<d* 2 REG_PARQPBLKL2	_1(OOOffset 0x00C22 */

#define BIT_CPIFT_RDTX_KBNUMAV	 fdefine BIT_MASK_USTX_KBNUMAV	 ffffffdefine BIT_MATX_KBNUMAV2)                                                  \
   	(((x) & BIT_MASK_EFTX_KBNUMAV2<< BIT_SHIFT_WLTX_KBNUMAV2<define BIT_GET_BITX_KBNUMAV2)                                                  \	(((x) &  BIT_SHIFT_USTX_KBNUMAV2& BIT_MASK_EFTX_KBNUMAV2<d* 2 REG_PARQPBLKL2	_			(Offset 0x00C22 */

#define BIT_STEXQHPUBLIC_DIS_V18T(31)9<#efine BIT_CPNPQHPUBLIC_DIS_V18T(31)8<#efine BIT_CPLPQHPUBLIC_DIS_V18T(31)7#define BIT_SHHPQHPUBLIC_DIS_V18T(31)6#d* 2 REG_PAFIFOPAGELINFO_1(OOffset 0x00C23 */

#define BIT_SHIFT_LPHPQHAVALF_GEV18
#define BIT_MASK_BIHPQHAVALF_GEV18f
fffdefine BIT_SHHPQHAVALF_GEV1)                                                 \

	(((x) & BIT_MASK_EFHPQHAVALF_GEV1<< BIT_SHIFT_RDHPQHAVALF_GEV1<define BIT_GET_BIHPQHAVALF_GEV1)                                                	(((x) &  BIT_SHIFT_RDHPQHAVALF_GEV1<<BIT_MASK_EFHPQHAVALF_GEV1<#define BIT_SHIFT_LPHPQHV18fdefine BIT_MASK_BIHPQHV18f
fffdefine BIT_SHHPQHV1) (((x) & BIT_MASK_CKHPQHV1<< BIT_SHIFT_RDHPQHV1<define BIT_GET_BIHPQHV1) (((x) &  BIT_SHIFT_RDHPQHV1<<BIT_MASK_EFHPQHV1 d* 2 REG_PAFIFOPAGELINFO_2(OOffset 0x00C23 */

#define BIT_SHIFT_SDLPQHAVALF_GEV18
#define BIT_MASK_BILPQHAVALF_GEV18f
fffdefine BIT_SHLPQHAVALF_GEV1)                                                 \

	(((x) & BIT_MASK_EFLPQHAVALF_GEV1<< BIT_SHIFT_RDLPQHAVALF_GEV1<define BIT_GET_BILPQHAVALF_GEV1)                                                	(((x) >> BIT_SHIFT_USLPQHAVALF_GEV1<<BIT_MASK_EFLPQHAVALF_GEV1<ddefine BIT_SHIFT_SDLPQHV18fdefine BIT_MASK_BILPQHV18f
fffdefine BIT_SHLPQHV1) (((x) & BIT_MASK_CKLPQHV1<< BIT_SHIFT_RDLPQHV1<define BIT_GET_BILPQHV1) (((x) &  BIT_SHIFT_USLPQHV1<<BIT_MASK_EFLPQHV1 d* 2 REG_PAFIFOPAGELINFO_3(OOffset 0x00C23 */

#define BIT_CPIFT_RDNPQHAVALF_GEV18
#define BIT_MASK_BINPQHAVALF_GEV18f
fffdefine BIT_SHNPQHAVALF_GEV1)                                                 \

	(((x) & BIT_MASK_EFNPQHAVALF_GEV1<< BIT_SHIFT_RDNPQHAVALF_GEV1<define BIT_GET_BINPQHAVALF_GEV1)                                                	(((x) &  BIT_SHIFT_RDNPQHAVALF_GEV1<<BIT_MASK_EFNPQHAVALF_GEV1<d* 2 REG_PAFIFOPAGELINFO_3(OOffset 0x00C23 */

#define BIT_CPIFT_RDNPQHV18fdefine BIT_MASK_BINPQHV18f
fffdefine BIT_SHNPQHV1) (((x) & BIT_MASK_CKNPQHV1<< BIT_SHIFT_RDNPQHV1<define BIT_GET_BINPQHV1) (((x) &  BIT_SHIFT_RDNPQHV1<<BIT_MASK_EFNPQHV1<d* 2 REG_PAFIFOPAGELINFO_4(OOffset 0x00C23 */

#define BIT_STIFT_USEXQHAVALF_GEV18
#define BIT_MASK_BIEXQHAVALF_GEV18f
fffdefine BIT_SHEXQHAVALF_GEV1)                                                 \

	(((x) & BIT_MASK_EFEXQHAVALF_GEV1<< BIT_SHIFT_RDEXQHAVALF_GEV1<define BIT_GET_BIEXQHAVALF_GEV1)                                                	(((x) >> BIT_SHIFT_USEXQHAVALF_GEV1<<BIT_MASK_EFEXQHAVALF_GEV1<ddefine BIT_STIFT_USEXQHV18fdefine BIT_MASK_BIEXQHV18f
fffdefine BIT_SHEXQHV1) (((x) & BIT_MASK_CKEXQHV1<< BIT_SHIFT_RDEXQHV1<define BIT_GET_BIEXQHV1) (((x) &  BIT_SHIFT_RDEXQHV1<<BIT_MASK_CKEXQHV1<d* 2 REG_PAFIFOPAGELINFO_5(OOffset 0x00C24 */

#define BIT_SHIFT_LPPUBQHAVALF_GEV18
#define BIT_MASK_BIPUBQHAVALF_GEV18f
fffdefine BIT_SH_UBQHAVALF_GEV1)                                                  \	(((x) & &IT_MASK_BIPUBQHAVALF_GEV1<< BIT_SHIFT_BIIUBQHAVALF_GEV1<define BIT_GET_BI_UBQHAVALF_GEV1)                                               	(((x) >> BIT_SHIFT_US_UBQHAVALF_GEV1<<&IT_MASK_BIPUBQHAVALF_GEV1<#define BIT_SHIFT_LPPUBQHV18fdefine BIT_MASK_BIPUBQHV18f
fffdefine BIT_SH_UBQHV1) (((x) & BIT_MASK_CK_UBQHV1<< BIT_SHIFT_BIIUBQHV1<define BIT_GET_BI_UBQHV1) (((x) &  BIT_SHIFT_RD_UBQHV1<<BIT_MASK_CK_UBQHV1</* 2 REG_PAH2C_HEAD		(Offset 0x00C24 */

#define BIT_SHIFT_SDH2C_HEAD8fdefine BIT_MASK_BIH2C_HEAD8fx3ff
fdefine BIT_SHH2C_HEAD) (((x) & BIT_MASK_CKH2C_HEAD<< BIT_SHIFT_RDH2C_HEAD<define BIT_GET_BIH2C_HEAD) (((x) &  BIT_SHIFT_RDH2C_HEAD<<BIT_MASK_CKH2C_HEAD</* 2 REG_PAH2C_TAIN_CPL_TIMEOUT_PS24 */

#define BIT_CPIFT_RDH2C_TAIN8fdefine BIT_MASK_BIH2C_TAIN fx3ff
fdefine BIT_SHH2C_TAIN)   (x) & BIT_MASK_CKH2C_TAIN<< BIT_SHIFT_BIH2C_TAIN<define BIT_GET_BIH2C_TAIN)   (x) &  BIT_SHIFT_RDH2C_TAIN<<BIT_MASK_EFH2C_TAIN<d* 2 REG_PAH2C_READ_ADDR(OOffset 0x00C24 */

#define BIT_STIFT_WLH2C_READ_ADDR8fdefine BIT_MASK_BIH2C_READ_ADDR8fx3ff
fdefine BIT_SHH2C_READ_ADDR)                                                     	(((x) & BIT_MASK_EFH2C_READ_ADDR<< BIT_SHIFT_BIH2C_READ_ADDR<define BIT_GET_BIH2C_READ_ADDR)                                                 	(((x) &  BIT_SHIFT_RDH2C_READ_ADDR<<BIT_MASK_EFH2C_READ_ADDR<d* 2 REG_PAH2C_WR_ADDR(OOOffset 0x00C25 */

#define BIT_SHIFT_LPH2C_WR_ADDR8fdefine BIT_MASK_BIH2C_WR_ADDR8fx3ff
fdefine BIT_SHH2C_WR_ADDR)                                                  \
   	(((x) & BIT_MASK_EFH2C_WR_ADDR<< BIT_SHIFT_BIH2C_WR_ADDR<define BIT_GET_BIH2C_WR_ADDR)                                                  \	(((x) &  BIT_SHIFT_RDH2C_WR_ADDR<<BIT_MASK_EFH2C_WR_ADDR<d* 2 REG_PAH2C_INFO(OOOffset 0x00C25 */

#define BIT_SHH2C_SPACEEVLDIB(31))define BIT_SHH2C_WR_ADDRTRSSIB(312<#define BIT_SHIFT_SDH2C_LENTS BIfdefine BIT_MASK_BIH2C_LENTS BIf
#define BIT_SEH2C_LENTS B)                                                  \
   	(((x) & BIT_MASK_EFH2C_LENTS B<< BIT_SHIFT_BIH2C_LENTS B<define BIT_GET_BIH2C_LENTS B)                                                  \	(((x) &  BIT_SHIFT_RDH2C_LENTS B<<BIT_MASK_EFH2C_LENTS B<d* 2 REG_PARXDMA_AGGF_G_MHOOOffset 0x00C28 */

#define BIT_SHIFT_LPRXDMA_AGGFOLD_KODI2#define BIT_MASK_RPRXDMA_AGGFOLD_KODIff
#define BIT_EFRXDMA_AGGFOLD_KOD)                                                 	(((x) & &IT_MASK_RPRXDMA_AGGFOLD_KOD<< BIT_SHIFT_RD_XDMA_AGGFOLD_KOD<define BIT_GET_BIRXDMA_AGGFOLD_KOD)                                             	(((x) >> BIT_SHIFT_RD_XDMA_AGGFOLD_KOD<<&IT_MASK_RPRXDMA_AGGFOLD_KOD<d* 2 REG_PARXDMA_AGGF_G_MHOOOffset 0x00C28 */

#define BIT_SHIFT_LPPKT_NUMAWON 
#define BIT_MASK_BI_KT_NUMAWON ff
#define BIT_EF_KT_NUMAWON)                                                  \
   	(((x) & BIT_MASK_EF_KT_NUMAWON<< BIT_SHIFT_BIIKT_NUMAWON<define BIT_GET_BI_KT_NUMAWON)                                                  \	(((x) >> BIT_SHIFT_US_KT_NUMAWON<<BIT_MASK_EF_KT_NUMAWON<d* 2 REG_PARXDMA_AGGF_G_MHOOOffset 0x00C28 */

#define BIT_SHIFT_LPDMA_AGGFTO8#define BIT_MASK_USDMA_AGGFTO8f
#define BIT_REDMA_AGGFTO)   (x) & BIT_MASK_CKDMA_AGGFTO<< BIT_SHIFT_BIDMA_AGGFTO<define BIT_GET_BIDMA_AGGFTO)                                                   \	(((x) >> BIT_SHIFT_USDMA_AGGFTO<<BIT_MASK_CKDMA_AGGFTO<d* 2 REG_PARXDMA_AGGF_G_MHOOOffset 0x00C28 */

#define BIT_SHIFT_LPRXDMA_AGGF_G_MHEV18fdefine BIT_MASK_BIRXDMA_AGGF_G_MHEV18fx#define BIT_EFRXDMA_AGGF_G_MHEV1)                                                	(((x) >>&IT_MASK_RPRXDMA_AGGF_G_MHEV1<< BIT_SHIFT_RD_XDMA_AGGF_G_MHEV1<define BIT_GET_BIRXDMA_AGGF_G_MHEV1)                                            	(((x) >> BIT_SHIFT_RD_XDMA_AGGF_G_MHEV1<<&IT_MASK_RPRXDMA_AGGF_G_MHEV1<d* 2 REG_PARX_KT_NUM(OOOffset 0x00C28 */

#define BIT_SHIFT_SDRX_KT_NUMI2#define BIT_MASK_RPRX_KT_NUMIff
#define BIT_EFRX_KT_NUM)   (x) & BIT_MASK_CKRX_KT_NUM<< BIT_SHIFT_RD_X_KT_NUM<define BIT_GET_BIRX_KT_NUM)   (x) &  BIT_SHIFT_RD_X_KT_NUM<<BIT_MASK_CKRX_KT_NUM<d* 2 REG_PARX_KT_NUM(OOOffset 0x00C28 */

#define BIT_SHIFT_SDFW_UPDTRDPTR19FTO_16 2fdefine BIT_MASK_SYFW_UPDTRDPTR19FTO_16 fx#define BIT_EFFW_UPDTRDPTR19FTO_16)                                              	(((x) >>&IT_MASK_RPFW_UPDTRDPTR19FTO_16                                  	 << BIT_SHIFT_SDFW_UPDTRDPTR19FTO_16 define BIT_GET_BIFW_UPDTRDPTR19FTO_16)                                          	(((x) >> BIT_SHIFT_RDFW_UPDTRDPTR19FTO_16                                	 BIT_MASK_SDFW_UPDTRDPTR19FTO_16 d* 2 REG_PARX_KT_NUM(OOOffset 0x00C28 */

#define BIT_SHRXDMA_REQIB(31)9<#efine BIT_CPRW_RELEASEE BIT(3118<#efine BIT_CPRXDMA_IDLEIT(3117<#efine BIT_CPRX_KT_RELEASEEPOLBIT(2016</*efine BIT_SHIFT_SDFW_UPDTRDPTR4fdefine BIT_MASK_SYFW_UPDTRDPTR4ffffffdefine BIT_MAFW_UPDTRDPTR)                                                      	(((x) & BIT_MASK_EFFW_UPDTRDPTR<< BIT_SHIFT_BIFW_UPDTRDPTR<define BIT_GET_BIFW_UPDTRDPTR)                                                  	(((x) >> BIT_SHIFT_RDFW_UPDTRDPTR<<BIT_MASK_EFFW_UPDTRDPTR<d* 2 REG_PARXDMA_STATUS(OOffset 0x00C28 */

#define BIT_CPC2HF_KT_OVFIT(317<#* 2 REG_PARXDMA_STATUS(OOffset 0x00C28 */

#define BIT_CPAGGFCONFGI_ISSUEIT(316<#* 2 REG_PARXDMA_STATUS(OOffset 0x00C28 */

#define BIT_CPFW_POLB_ISSUEIT(315<#efine BIT_CPRXTTA_REUDBIT(314<#efine BIT_CPRXTSFFDUDBIT(31
#define BIT_STRX_SFFDOVFIT(312<#* 2 REG_PARXDMA_STATUS(OOffset 0x00C28 */

#define BIT_CPRX_KT_OVFIT(310</* 2 REG_PARXDMA_DPR(OOOffset 0x00C28 */

#define BIT_STIFT_WLRDE_DEBUG4fdefine BIT_MASK_SYRDE_DEBUG4ffffffffL
#define BIT_DERDE_DEBUG)   (x) & BIT_MASK_CKRDE_DEBUG<< BIT_SHIFT_RD_DE_DEBUG<define BIT_GET_BIRDE_DEBUG)   (x) &  BIT_SHIFT_RD_DE_DEBUG<<BIT_MASK_CKRDE_DEBUG</* 2 REG_PARXDMA_DE_V(OOOffset 0x00C29 */

#define BIT_SHIFT_US_KTNUMAMHEV2I2#define BIT_MASK_RP_KTNUMAMHEV2Iffffdefine BIT_EF_KTNUMAMHEV2)                                                      	(((x) & BIT_MASK_EF_KTNUMAMHEV2<< BIT_SHIFT_BIIKTNUMAMHEV2<define BIT_GET_BI_KTNUMAMHEV2)                                                  	(((x) >> BIT_SHIFT_US_KTNUMAMHEV2<<BIT_MASK_EF_KTNUMAMHEV2<#define BIT_SHTXBA_BREACMB23AGGIT(3123<#define BIT_SHIFT_LPPKTLENTPARA 
#define BIT_MASK_BI_KTLENTPARA f
#define BIT_SD_KTLENTPARA)                                                  \
   	(((x) & BIT_MASK_EF_KTLENTPARA<< BIT_SHIFT_BIIKTLENTPARA<define BIT_GET_BI_KTLENTPARA)                                                  \	(((x) >> BIT_SHIFT_US_KTLENTPARA<<BIT_MASK_EF_KTLENTPARA</* 2 REG_PARXDMA_DE_V(OOOffset 0x00C29 */

#define BIT_SHIFT_USBURSSTSIZE8#define BIT_MASK_RPBURSSTSIZE8f
#define BIT_SEBURSSTSIZE)   (x) & BIT_MASK_CKBURSSTSIZE<< BIT_SHIFT_WLBURSSTSIZE<define BIT_GET_BIBURSSTSIZE)                                                   \	(((x) >> BIT_SHIFT_USBURSSTSIZE<<BIT_MASK_CKBURSSTSIZE<#define BIT_SHIFT_USBURSSTCNT4#define BIT_MASK_CPBURSSTCNT4f
#define BIT_SEBURSSTCNT)   (x) & BIT_MASK_CKBURSSTCNT<< BIT_SHIFT_WLBURSSTCNT<define BIT_GET_BIBURSSTCNT)   (x) &  BIT_SHIFT_USBURSSTCNT<<BIT_MASK_CKBURSSTCNT</* 2 REG_PARXDMA_DE_V(OOOffset 0x00C29 */

#define BIT_SHDMA_DE_VIT(201<d* 2 REG_PAW2HF_KT(OOOffset 0x00C29 */

#define BIT_SHIFT_SDRAW2HFSTR_ADDRT16FTO_19I2#define BIT_MASK_RPRAW2HFSTR_ADDRT16FTO_19Ifx#define BIT_EFRAW2HFSTR_ADDRT16FTO_19)                                           	(((x) >>&IT_MASK_RPRAW2HFSTR_ADDRT16FTO_19                               	 << BIT_SHIFT_SDRAW2HFSTR_ADDRT16FTO_19 define BIT_GET_BIRAW2HFSTR_ADDRT16FTO_19)                                       	(((x) >> BIT_SHIFT_RD_AW2HFSTR_ADDRT16FTO_19                             	 BIT_MASK_SDRAW2HFSTR_ADDRT16FTO_19 ddefine BIT_SHIFT_SDMDIO_PHY_ADDR4##define BIT_MASK_RPMDIO_PHY_ADDR4ffffdefine BIT_EFMDIO_PHY_ADDR)                                                     	(((x) & BIT_MASK_EFMDIO_PHY_ADDR<< BIT_SHIFT_BIMDIO_PHY_ADDR<define BIT_GET_BIMDIO_PHY_ADDR)                                                 	(((x) >> BIT_SHIFT_USMDIO_PHY_ADDR<<BIT_MASK_EFMDIO_PHY_ADDR<d* 2 REG_PAW2HF_KT(OOOffset 0x00C29 */

#define BIT_SHRPC2HF_KT_REQIB(31)6#define BIT_STRX_CLOSEE BIT(3115<#efine BIT_CPSTOPFBDEQIB(31)4<#efine BIT_CPSTOPFMGQIB(31)3<#efine BIT_CPSTOPFVOQIB(31)2<#efine BIT_CPSTOPFVIQIB(31)1<#efine BIT_CPSTOPFBEQIB(31)0<#efine BIT_CPSTOPFBKQIB(319<#efine BIT_CPSTOPFRXQIB(318<#efine BIT_CPSTOPFHI7QIT(317<#efine BIT_CPSTOPFHI6QIT(316<#efine BIT_CPSTOPFHI5QIT(315<#efine BIT_CPSTOPFHI4QIT(314<#efine BIT_CPSTOPFHI3QIT(31
#define BIT_STSTOPFHI2QIT(312<#efine BIT_STSTOPFHI1QIT(201<d*efine BIT_SHIFT_SDRAW2HFSTR_ADDR4fdefine BIT_MASK_SYRAW2HFSTR_ADDR4ffffffdefine BIT_MARAW2HFSTR_ADDR)                                                 \

	(((x) & BIT_MASK_EFRAW2HFSTR_ADDR<< BIT_SHIFT_RD_AW2HFSTR_ADDR<define BIT_GET_BIRAW2HFSTR_ADDR)                                                	(((x) >> BIT_SHIFT_USRAW2HFSTR_ADDR<<BIT_MASK_EFRAW2HFSTR_ADDR<
#efine BIT_STSTOPFHI0QIT(310</* 2 REG_PAFWFFDW2H(OOOffset 0x00C29 */

#define BIT_CPIFT_RDW2HFDMA_ADDR4fdefine BIT_MASK_SYW2HFDMA_ADDR4fx3ff
fdefine BIT_SHW2HFDMA_ADDR)                                                      	(((x) & BIT_MASK_EFW2HFDMA_ADDR<< BIT_SHIFT_WLK2HFDMA_ADDR<define BIT_GET_BIW2HFDMA_ADDR)                                                  	(((x) >> BIT_SHIFT_RDK2HFDMA_ADDR<<BIT_MASK_EFW2HFDMA_ADDR</* 2 REG_PAFWFFDW_EN_CPL_TIMEOUT_PS29 */

#define BIT_STFWFFDDMA_KT_REQIB(31)1</*efine BIT_SHIFT_SDFWFFDDMAF_KT_NUM 
#define BIT_MASK_BIFWFFDDMAF_KT_NUM ff
#define BIT_EFFWFFDDMAF_KT_NUM)                                                  	(((x) >>&IT_MASK_BIFWFFDDMAF_KT_NUM<< BIT_SHIFT_BIFWFFDDMAF_KT_NUM<define BIT_GET_BIFWFFDDMAF_KT_NUM)                                              	(((x) >> BIT_SHIFT_RDFWFFDDMAF_KT_NUM<<&IT_MASK_BIFWFFDDMAF_KT_NUM</*efine BIT_SHIFT_SDFWFFDSTR_ADDR4fdefine BIT_MASK_SYFWFFDSTR_ADDR4ffffffdefine BIT_MAFWFFDSTR_ADDR)                                                     	(((x) & BIT_MASK_EFFWFFDSTR_ADDR<< BIT_SHIFT_BIFWFFDSTR_ADDR<define BIT_GET_BIFWFFDSTR_ADDR)                                                 	(((x) >> BIT_SHIFT_RDFWFFDSTR_ADDR<<BIT_MASK_EFFWFFDSTR_ADDR<d* 2 REG_PAFWFFD_KT_INFO(OO_TIMEOUT_PS2A */

#define BIT_SHIFT_LPFWFFD_KT_QUEUED 
#define BIT_MASK_BIFWFFD_KT_QUEUED ff
#define BIT_EFFWFFD_KT_QUEUED)                                                  \	(((x) & &IT_MASK_BIFWFFD_KT_QUEUED<< BIT_SHIFT_BIFWFFD_KT_QUEUED<define BIT_GET_BIFWFFD_KT_QUEUED)                                               	(((x) >> BIT_SHIFT_RDFWFFD_KT_QUEUED<<&IT_MASK_BIFWFFD_KT_QUEUED<d* 2 REG_PAFWFFD_KT_INFO(OO_TIMEOUT_PS2A */

#define BIT_SHIFT_LPFWFFD_KT_STR_ADDR4fdefine BIT_MASK_SYFWFFD_KT_STR_ADDR4ffffffdefine BIT_MAFWFFD_KT_STR_ADDR)                                                 	(((x) & &IT_MASK_RPFWFFD_KT_STR_ADDR<< BIT_SHIFT_BIFWFFD_KT_STR_ADDR<define BIT_GET_BIFWFFD_KT_STR_ADDR)                                             	(((x) >> BIT_SHIFT_RDFWFFD_KT_STR_ADDR<<BIT_MASK_EFFWFFD_KT_STR_ADDR<d* 2 REG_PAI_DREW_EN_CPL_TIMEOUT_PS30 */

#define BIT_SHI_DRIO_PERSSB_S BIT(2031<d* 2 REG_PAI_DREW_EN_CPL_TIMEOUT_PS30 */

#define BIT_SHIFT_US__DREMAXDRXDMA 2#define BIT_MASK_US__DREMAXDRXDMA f
#define BIT_SD__DREMAXDRXDMA)                                                 \

	(((x) & BIT_MASK_EF__DREMAXDRXDMA<< BIT_SHIFT_BII_DREMAXDRXDMA<define BIT_GET_BI__DREMAXDRXDMA)                                                	(((x) >> BIT_SHIFT_US__DREMAXDRXDMA<<BIT_MASK_EF__DREMAXDRXDMA<d* 2 REG_PAI_DREW_EN_CPL_TIMEOUT_PS30 */

#define BIT_SHIFT_US__DREMAXDTXDMA 2#define BIT_MASK_RP__DREMAXDTXDMA f
#define BIT_SD__DREMAXDTXDMA)                                                 \

	(((x) & BIT_MASK_EF__DREMAXDTXDMA<< BIT_SHIFT_BII_DREMAXDTXDMA<define BIT_GET_BI__DREMAXDTXDMA)                                                	(((x) >> BIT_SHIFT_US__DREMAXDTXDMA<<BIT_MASK_EF__DREMAXDTXDMA<d* 2 REG_PAI_DREW_EN_CPL_TIMEOUT_PS30 */

#define BIT_SHI_DRERSSTTRXDMA_INTFIT(3120<d* 2 REG_PAI_DREW_EN_CPL_TIMEOUT_PS30 */

#define BIT_SHI_DREENTSWENT_L23IT(2017<d* 2 REG_PAI_DREW_EN_CPL_TIMEOUT_PS30 */

#define BIT_SHI_DREENTHWEXT_L18T(31)6#d* 2 REG_PAINT_MIG_CPL_TIMEOUT_PS30 */

#define BIT_SHIFT_SDTXTTIMER_MATCH_NUMI2#define BIT_MASK_USTXTTIMER_MATCH_NUMIfx#define BIT_EFTXTTIMER_MATCH_NUM)                                                	(((x) >>&IT_MASK_RPTXTTIMER_MATCH_NUM<< BIT_SHIFT_WLTXTTIMER_MATCH_NUM<define BIT_GET_BITXTTIMER_MATCH_NUM)                                            	(((x) &  BIT_SHIFT_USTXTTIMER_MATCH_NUM<<&IT_MASK_RPTXTTIMER_MATCH_NUM<#define BIT_CPIFT_RDTX_KB_NUMAMATCHI2#define BIT_MASK_RPTX_KB_NUMAMATCHIfx#define BIT_EFTX_KB_NUMAMATCH)                                                  \	(((x) & &IT_MASK_BITX_KB_NUMAMATCH<< BIT_SHIFT_WLTX_KB_NUMAMATCH<define BIT_GET_BITX_KB_NUMAMATCH)                                               	(((x) &  BIT_SHIFT_USTX_KB_NUMAMATCH<<&IT_MASK_BITX_KB_NUMAMATCH<#define BIT_SHIFT_SDRXTTIMER_MATCH_NUMI2fdefine BIT_MASK_BIRXTTIMER_MATCH_NUMIfx#define BIT_EFRXTTIMER_MATCH_NUM)                                                	(((x) >>&IT_MASK_RPRXTTIMER_MATCH_NUM<< BIT_SHIFT_WLRXTTIMER_MATCH_NUM<define BIT_GET_BIRXTTIMER_MATCH_NUM)                                            	(((x) &  BIT_SHIFT_USRXTTIMER_MATCH_NUM<<&IT_MASK_RPRXTTIMER_MATCH_NUM<#define BIT_SHIFT_SDRX_KT_NUMAMATCHI
#define BIT_MASK_BIRX_KT_NUMAMATCHIfx#define BIT_EFRX_KB_NUMAMATCH)                                                  \	(((x) & &IT_MASK_BIRX_KB_NUMAMATCH<< BIT_SHIFT_WLRX_KB_NUMAMATCH<define BIT_GET_BIRX_KT_NUMAMATCH)                                               	(((x) &  BIT_SHIFT_USRX_KB_NUMAMATCH<<&IT_MASK_BIRX_KB_NUMAMATCH<ddefine BIT_SHIFT_SDMIGRATE_TIMER fdefine BIT_MASK_BIMIGRATE_TIMER ffffffdefine BIT_MAMIGRATE_TIMER)                                                     	(((x) & BIT_MASK_EFMIGRATE_TIMER<< BIT_SHIFT_BIMIGRATE_TIMER<define BIT_GET_BIMIGRATE_TIMER)                                                 	(((x) >> BIT_SHIFT_USMIGRATE_TIMER<<BIT_MASK_EFMIGRATE_TIMER</* 2 REG_PABCNQHTXBD_DESACPL_TIMEOUT_PS30 */

#define BIT_CPIFT_RDBCNQHTXBD_DESA8fdefine BIT_MASK_BIBDEQHTXBD_DESA8ffffffffL
ffffffL
#define BIT_DEBDEQHTXBD_DESA)                                                 \

	(((x) & BIT_MASK_EFBDEQHTXBD_DESA<< BIT_SHIFT_WLBDEQHTXBD_DESA<define BIT_GET_BIBDEQHTXBD_DESA)                                                	(((x) &  BIT_SHIFT_USBDEQHTXBD_DESA<<BIT_MASK_EFBDEQHTXBD_DESA</* 2 REG_PAMGQHTXBD_DESACPL_TIMEOUT_PS31 */

#define BIT_SHIFT_USMGQHTXBD_DESA8fdefine BIT_MASK_BIMGQHTXBD_DESA8ffffffffL
ffffffL
#define BIT_DEMGQHTXBD_DESA)                                                     	(((x) & BIT_MASK_EFMGQHTXBD_DESA<< BIT_SHIFT_WLMGQHTXBD_DESA<define BIT_GET_BIMGQHTXBD_DESA)                                                 	(((x) >> BIT_SHIFT_USMGQHTXBD_DESA<<BIT_MASK_EFMGQHTXBD_DESA<d* 2 REG_PAVOQHTXBD_DESACPL_TIMEOUT_PS31 */

#define BIT_CPIFT_RDVOQHTXBD_DESA8fdefine BIT_MASK_BIVOQHTXBD_DESA8ffffffffL
ffffffL
#define BIT_DEVOQHTXBD_DESA)                                                     	(((x) & BIT_MASK_EFVOQHTXBD_DESA<< BIT_SHIFT_WLVOQHTXBD_DESA<define BIT_GET_BIVOQHTXBD_DESA)                                                 	(((x) >> BIT_SHIFT_USVOQHTXBD_DESA<<BIT_MASK_EFVOQHTXBD_DESA<d* 2 REG_PAVIQHTXBD_DESACPL_TIMEOUT_PS32 */

#define BIT_SHIFT_LPVIQHTXBD_DESA8fdefine BIT_MASK_BIVIQHTXBD_DESA8ffffffffL
ffffffL
#define BIT_DEVIQHTXBD_DESA)                                                     	(((x) & BIT_MASK_EFVIQHTXBD_DESA<< BIT_SHIFT_WLVIQHTXBD_DESA<define BIT_GET_BIVIQHTXBD_DESA)                                                 	(((x) >> BIT_SHIFT_USVIQHTXBD_DESA<<BIT_MASK_EFVIQHTXBD_DESA<d* 2 REG_PABEQHTXBD_DESACPL_TIMEOUT_PS32 */

#define BIT_CPIFT_RDBEQHTXBD_DESA8fdefine BIT_MASK_BIBEQHTXBD_DESA8ffffffffL
ffffffL
#define BIT_DEBEQHTXBD_DESA)                                                     	(((x) & BIT_MASK_EFBEQHTXBD_DESA<< BIT_SHIFT_WLBEQHTXBD_DESA<define BIT_GET_BIBEQHTXBD_DESA)                                                 	(((x) &  BIT_SHIFT_USBEQHTXBD_DESA<<BIT_MASK_EFBEQHTXBD_DESA<d* 2 REG_PABKQHTXBD_DESACPL_TIMEOUT_PS33 */

#define BIT_SHIFT_LPBKQHTXBD_DESA8fdefine BIT_MASK_BIBKQHTXBD_DESA8ffffffffL
ffffffL
#define BIT_DEBKQHTXBD_DESA)                                                     	(((x) & BIT_MASK_EFBKQHTXBD_DESA<< BIT_SHIFT_WLBKQHTXBD_DESA<define BIT_GET_BIBKQHTXBD_DESA)                                                 	(((x) &  BIT_SHIFT_USBKQHTXBD_DESA<<BIT_MASK_EFBKQHTXBD_DESA<d* 2 REG_PARXQARXBD_DESACPL_TIMEOUT_PS33 */

#define BIT_CPIFT_RDRXQARXBD_DESA8fdefine BIT_MASK_BIRXQARXBD_DESA8ffffffffL
ffffffL
#define BIT_DERXQARXBD_DESA)                                                     	(((x) & BIT_MASK_EFRXQARXBD_DESA<< BIT_SHIFT_WLRXQARXBD_DESA<define BIT_GET_BIRXQARXBD_DESA)                                                 	(((x) &  BIT_SHIFT_USRXQARXBD_DESA<<BIT_MASK_EFRXQARXBD_DESA<d* 2 REG_PAHI0QHTXBD_DESACPL_TIMEOUT_PS34 */

#define BIT_SHIFT_LPHI0QHTXBD_DESAIfdefine BIT_MASK_BIHI0QHTXBD_DESAIffffffffL
ffffffL
#define BIT_DEHI0QHTXBD_DESA)                                                 \

	(((x) & BIT_MASK_EFHI0QHTXBD_DESA<< BIT_SHIFT_BIHI0QHTXBD_DESA<define BIT_GET_BIHI0QHTXBD_DESA)                                                	(((x) &  BIT_SHIFT_RDHI0QHTXBD_DESA<<BIT_MASK_EFHI0QHTXBD_DESA<d* 2 REG_PAHI1QHTXBD_DESACPL_TIMEOUT_PS34 */

#define BIT_CPIFT_RDHI1QHTXBD_DESAIfdefine BIT_MASK_BIHI1QHTXBD_DESAIffffffffL
ffffffL
#define BIT_DEHI1QHTXBD_DESA)                                                 \

	(((x) & BIT_MASK_EFHI1QHTXBD_DESA<< BIT_SHIFT_BIHI1QHTXBD_DESA<define BIT_GET_BIHI1QHTXBD_DESA)                                                	(((x) &  BIT_SHIFT_RDHI1QHTXBD_DESA<<BIT_MASK_EFHI1QHTXBD_DESA<d* 2 REG_PAHI2QHTXBD_DESACPL_TIMEOUT_PS35 */

#define BIT_SHIFT_LPHI2QHTXBD_DESAIfdefine BIT_MASK_BIHI2QHTXBD_DESAIffffffffL
ffffffL
#define BIT_DEHI2QHTXBD_DESA)                                                 \

	(((x) & BIT_MASK_EFHI2QHTXBD_DESA<< BIT_SHIFT_BIHI2QHTXBD_DESA<define BIT_GET_BIHI2QHTXBD_DESA)                                                	(((x) &  BIT_SHIFT_RDHI2QHTXBD_DESA<<BIT_MASK_EFHI2QHTXBD_DESA<d* 2 REG_PAHI3QHTXBD_DESACPL_TIMEOUT_PS35 */

#define BIT_CPIFT_RDHI3QHTXBD_DESAIfdefine BIT_MASK_BIHI3QHTXBD_DESAIffffffffL
ffffffL
#define BIT_DEHI3QHTXBD_DESA)                                                 \

	(((x) & BIT_MASK_EFHI3QHTXBD_DESA<< BIT_SHIFT_BIHI3QHTXBD_DESA<define BIT_GET_BIHI3QHTXBD_DESA)                                                	(((x) &  BIT_SHIFT_RDHI3QHTXBD_DESA<<BIT_MASK_EFHI3QHTXBD_DESA<d* 2 REG_PAHI4QHTXBD_DESACPL_TIMEOUT_PS36 */

#define BIT_SHIFT_LPHI4QHTXBD_DESAIfdefine BIT_MASK_BIHI4QHTXBD_DESAIffffffffL
ffffffL
#define BIT_DEHI4QHTXBD_DESA)                                                 \

	(((x) & BIT_MASK_EFHI4QHTXBD_DESA<< BIT_SHIFT_BIHI4QHTXBD_DESA<define BIT_GET_BIHI4QHTXBD_DESA)                                                	(((x) &  BIT_SHIFT_RDHI4QHTXBD_DESA<<BIT_MASK_EFHI4QHTXBD_DESA<d* 2 REG_PAHI5QHTXBD_DESACPL_TIMEOUT_PS36 */

#define BIT_CPIFT_RDHI5QHTXBD_DESAIfdefine BIT_MASK_BIHI5QHTXBD_DESAIffffffffL
ffffffL
#define BIT_DEHI5QHTXBD_DESA)                                                 \

	(((x) & BIT_MASK_EFHI5QHTXBD_DESA<< BIT_SHIFT_BIHI5QHTXBD_DESA<define BIT_GET_BIHI5QHTXBD_DESA)                                                	(((x) &  BIT_SHIFT_RDHI5QHTXBD_DESA<<BIT_MASK_EFHI5QHTXBD_DESA<d* 2 REG_PAHI6QHTXBD_DESACPL_TIMEOUT_PS37 */

#define BIT_SHIFT_LPHI6QHTXBD_DESAIfdefine BIT_MASK_BIHI6QHTXBD_DESAIffffffffL
ffffffL
#define BIT_DEHI6QHTXBD_DESA)                                                 \

	(((x) & BIT_MASK_EFHI6QHTXBD_DESA<< BIT_SHIFT_BIHI6QHTXBD_DESA<define BIT_GET_BIHI6QHTXBD_DESA)                                                	(((x) &  BIT_SHIFT_RDHI6QHTXBD_DESA<<BIT_MASK_EFHI6QHTXBD_DESA<d* 2 REG_PAHI7QHTXBD_DESACPL_TIMEOUT_PS37 */

#define BIT_CPIFT_RDHI7QHTXBD_DESAIfdefine BIT_MASK_BIHI7QHTXBD_DESAIffffffffL
ffffffL
#define BIT_DEHI7QHTXBD_DESA)                                                 \

	(((x) & BIT_MASK_EFHI7QHTXBD_DESA<< BIT_SHIFT_BIHI7QHTXBD_DESA<define BIT_GET_BIHI7QHTXBD_DESA)                                                	(((x) &  BIT_SHIFT_RDHI7QHTXBD_DESA<<BIT_MASK_EFHI7QHTXBD_DESA<d* 2 REG_PAMGQHTXBD_NUM(OO_TIMEOUT_PS38 */

#define BIT_SH__DREMGQHFLAGIT(31)4<#* 2 REG_PAMGQHTXBD_NUM(OO_TIMEOUT_PS38 */

#define BIT_SHIFT_USMGQHDESC_DE_VI1#define BIT_MASK_CPMGQHDESC_DE_VIf
#define BIT_SEMGQHDESC_DE_V)                                                     	(((x) & BIT_MASK_EFMGQHDESC_DE_V<< BIT_SHIFT_WLMGQHDESC_DE_V<define BIT_GET_BIMGQHDESC_DE_V)                                                 	(((x) >> BIT_SHIFT_USMGQHDESC_DE_V<<BIT_MASK_EFMGQHDESC_DE_V<#define BIT_SHIFT_USMGQHDESC_NUMIfdefine BIT_MASK_CPMGQHDESC_NUM ff
#fdefine BIT_SEMGQHDESC_NUM)                                                      	(((x) & BIT_MASK_EFMGQHDESC_NUM<< BIT_SHIFT_WLMGQHDESC_NUM<define BIT_GET_BIMGQHDESC_NUM)                                                  	(((x) >> BIT_SHIFT_USMGQHDESC_NUM<<&IT_MASK_RPMGQHDESC_NUM<d* 2 REG_PARXARXBD_NUM(OOOffset 0x00C382*/

#define BIT_SHIYS_32_64IT(31)5<#define BIT_SHIFT_USBDEQHDESC_DE_VI13define BIT_MASK_CPBDEQHDESC_DE_VIf
#define BIT_SEBDEQHDESC_DE_V)                                                 \

	(((x) & BIT_MASK_EFBDEQHDESC_DE_V<< BIT_SHIFT_WLBDEQHDESC_DE_V<define BIT_GET_BIBDEQHDESC_DE_V)                                                	(((x) &  BIT_SHIFT_USBDEQHDESC_DE_V<<BIT_MASK_EFBDEQHDESC_DE_V<d* 2 REG_PARXARXBD_NUM(OOOffset 0x00C382*/

#define BIT_SH__DREBDEQHFLAGIT(31)2<d* 2 REG_PARXARXBD_NUM(OOOffset 0x00C382*/

#define BIT_SHIFT_USRXQADESC_NUMIfdefine BIT_MASK_CPRXQADESC_NUMIff
#fdefine BIT_SERXQADESC_NUM)                                                      	(((x) & BIT_MASK_EFRXQADESC_NUM<< BIT_SHIFT_WLRXQADESC_NUM<define BIT_GET_BIRXQADESC_NUM)                                                  	(((x) &  BIT_SHIFT_USRXQADESC_NUM<<&IT_MASK_RPRXQADESC_NUM<d* 2 REG_PAVOQHTXBD_NUM(OO_TIMEOUT_PS38 */

#define BIT_SH__DREVOQHFLAGIT(31)4<#* 2 REG_PAVOQHTXBD_NUM(OO_TIMEOUT_PS38 */

#define BIT_SHIFT_USVOQHDESC_DE_VI1#define BIT_MASK_CPVOQHDESC_DE_VIf
#define BIT_SEVOQHDESC_DE_V)                                                     	(((x) & BIT_MASK_EFVOQHDESC_DE_V<< BIT_SHIFT_WLVOQHDESC_DE_V<define BIT_GET_BIVOQHDESC_DE_V)                                                 	(((x) >> BIT_SHIFT_USVOQHDESC_DE_V<<BIT_MASK_EFVOQHDESC_DE_V<#define BIT_SHIFT_USVOQHDESC_NUMIfdefine BIT_MASK_CPVOQHDESC_NUMIff
#fdefine BIT_SEVOQHDESC_NUM)                                                      	(((x) & BIT_MASK_EFVOQHDESC_NUM<< BIT_SHIFT_WLVOQHDESC_NUM<define BIT_GET_BIVOQHDESC_NUM)                                                  	(((x) >> BIT_SHIFT_USVOQHDESC_NUM<<&IT_MASK_RPVOQHDESC_NUM<d* 2 REG_PAVIQHTXBD_NUM(OO_TIMEOUT_PS386*/

#define BIT_SH__DREVIQHFLAGIT(31)4<#* 2 REG_PAVIQHTXBD_NUM(OO_TIMEOUT_PS386*/

#define BIT_SHIFT_USVIQHDESC_DE_VI1#define BIT_MASK_CPVIQHDESC_DE_VIf
#define BIT_SEVIQHDESC_DE_V)                                                     	(((x) & BIT_MASK_EFVIQHDESC_DE_V<< BIT_SHIFT_WLVIQHDESC_DE_V<define BIT_GET_BIVIQHDESC_DE_V)                                                 	(((x) >> BIT_SHIFT_USVIQHDESC_DE_V<<BIT_MASK_EFVIQHDESC_DE_V<#define BIT_SHIFT_USVIQHDESC_NUMIfdefine BIT_MASK_CPVIQHDESC_NUMIff
#fdefine BIT_SEVIQHDESC_NUM)                                                      	(((x) & BIT_MASK_EFVIQHDESC_NUM<< BIT_SHIFT_WLVIQHDESC_NUM<define BIT_GET_BIVIQHDESC_NUM)                                                  	(((x) >> BIT_SHIFT_USVIQHDESC_NUM<<&IT_MASK_RPVIQHDESC_NUM<d* 2 REG_PABEQHTXBD_NUM(OO_TIMEOUT_PS38 */

#define BIT_CP__DREBEQHFLAGIT(31)4<#* 2 REG_PABEQHTXBD_NUM(OO_TIMEOUT_PS38 */

#define BIT_CPIFT_USBEQHDESC_DE_VI1#define BIT_MASK_CPBEQHDESC_DE_VIf
#define BIT_SEBEQHDESC_DE_V)                                                     	(((x) & BIT_MASK_EFBEQHDESC_DE_V<< BIT_SHIFT_WLBEQHDESC_DE_V<define BIT_GET_BIBEQHDESC_DE_V)                                                 	(((x) &  BIT_SHIFT_USBEQHDESC_DE_V<<BIT_MASK_EFBEQHDESC_DE_V<#define BIT_SHIFT_USBEQHDESC_NUMIfdefine BIT_MASK_CPBEQHDESC_NUMIff
#fdefine BIT_SEBEQHDESC_NUM)                                                      	(((x) & BIT_MASK_EFBEQHDESC_NUM<< BIT_SHIFT_WLBEQHDESC_NUM<define BIT_GET_BIBEQHDESC_NUM)                                                  	(((x) &  BIT_SHIFT_USBEQHDESC_NUM<<&IT_MASK_RPBEQHDESC_NUM<d* 2 REG_PABKQHTXBD_NUM(OO_TIMEOUT_PS38A*/

#define BIT_CP__DREBKQHFLAGIT(31)4<#* 2 REG_PABKQHTXBD_NUM(OO_TIMEOUT_PS38A*/

#define BIT_CPIFT_USBKQHDESC_DE_VI1#define BIT_MASK_CPBKQHDESC_DE_VIf
#define BIT_SEBKQHDESC_DE_V)                                                     	(((x) & BIT_MASK_EFBKQHDESC_DE_V<< BIT_SHIFT_WLBKQHDESC_DE_V<define BIT_GET_BIBKQHDESC_DE_V)                                                 	(((x) &  BIT_SHIFT_USBKQHDESC_DE_V<<BIT_MASK_EFBKQHDESC_DE_V<#define BIT_SHIFT_USBKQHDESC_NUMIfdefine BIT_MASK_CPBKQHDESC_NUMIff
#fdefine BIT_SEBKQHDESC_NUM)                                                      	(((x) & BIT_MASK_EFBKQHDESC_NUM<< BIT_SHIFT_WLBKQHDESC_NUM<define BIT_GET_BIBKQHDESC_NUM)                                                  	(((x) &  BIT_SHIFT_USBKQHDESC_NUM<<&IT_MASK_RPBKQHDESC_NUM<d* 2 REG_PAHI0QHTXBD_NUM(OO_TIMEOUT_PS38 */

#define BIT_STHI0QHFLAGIT(31)4<#*efine BIT_SHIFT_LPHI0QHDESC_DE_VI1#define BIT_MASK_CPHI0QHDESC_DE_VIf
#define BIT_SEHI0QHDESC_DE_V)                                                 \

	(((x) & BIT_MASK_EFHI0QHDESC_DE_V<< BIT_SHIFT_WLHI0QHDESC_DE_V<define BIT_GET_BIHI0QHDESC_DE_V)                                                	(((x) &  BIT_SHIFT_USHI0QHDESC_DE_V<<BIT_MASK_EFHI0QHDESC_DE_V<#*efine BIT_SHIFT_LPHI0QHDESC_NUMIfdefine BIT_MASK_CPHI0QHDESC_NUMIf
fffdefine BIT_SHHI0QHDESC_NUM)                                                     	(((x) & BIT_MASK_EFHI0QHDESC_NUM<< BIT_SHIFT_WLHI0QHDESC_NUM<define BIT_GET_BIHI0QHDESC_NUM)                                                 	(((x) &  BIT_SHIFT_USHI0QHDESC_NUM<<&IT_MASK_RPHI0QHDESC_NUM<d* 2 REG_PAHI1QHTXBD_NUM(OO_TIMEOUT_PS38E*/

#define BIT_STHI1QHFLAGIT(31)4<#*efine BIT_SHIFT_LPHI1QHDESC_DE_VI1#define BIT_MASK_CPHI1QHDESC_DE_VIf
#define BIT_SEHI1QHDESC_DE_V)                                                 \

	(((x) & BIT_MASK_EFHI1QHDESC_DE_V<< BIT_SHIFT_WLHI1QHDESC_DE_V<define BIT_GET_BIHI1QHDESC_DE_V)                                                	(((x) &  BIT_SHIFT_RDHI1QHDESC_DE_V<<BIT_MASK_EFHI1QHDESC_DE_V<#*efine BIT_SHIFT_LPHI1QHDESC_NUMIfdefine BIT_MASK_CPHI1QHDESC_NUMIf
fffdefine BIT_SHHI1QHDESC_NUM)                                                     	(((x) & BIT_MASK_EFHI1QHDESC_NUM<< BIT_SHIFT_WLHI1QHDESC_NUM<define BIT_GET_BIHI1QHDESC_NUM)                                                 	(((x) &  BIT_SHIFT_RDHI1QHDESC_NUM<<&IT_MASK_RPHI1QHDESC_NUM<d* 2 REG_PAHI2QHTXBD_NUM(OO_TIMEOUT_PS39 */

#define BIT_SHHI2QHFLAGIT(31)4<#*efine BIT_SHIFT_LPHI2QHDESC_DE_VI1#define BIT_MASK_CPHI2QHDESC_DE_VIf
#define BIT_SEHI2QHDESC_DE_V)                                                 \

	(((x) & BIT_MASK_EFHI2QHDESC_DE_V<< BIT_SHIFT_WLHI2QHDESC_DE_V<define BIT_GET_BIHI2QHDESC_DE_V)                                                	(((x) &  BIT_SHIFT_RDHI2QHDESC_DE_V<<BIT_MASK_EFHI2QHDESC_DE_V<#*efine BIT_SHIFT_LPHI2QHDESC_NUMIfdefine BIT_MASK_CPHI2QHDESC_NUMIf
fffdefine BIT_SHHI2QHDESC_NUM)                                                     	(((x) & BIT_MASK_EFHI2QHDESC_NUM<< BIT_SHIFT_WLHI2QHDESC_NUM<define BIT_GET_BIHI2QHDESC_NUM)                                                 	(((x) &  BIT_SHIFT_RDHI2QHDESC_NUM<<&IT_MASK_RPHI2QHDESC_NUM<d* 2 REG_PAHI3QHTXBD_NUM(OO_TIMEOUT_PS392*/

#define BIT_SHHI3QHFLAGIT(31)4<#*efine BIT_SHIFT_LPHI3QHDESC_DE_VI1#define BIT_MASK_CPHI3QHDESC_DE_VIf
#define BIT_SEHI3QHDESC_DE_V)                                                 \

	(((x) & BIT_MASK_EFHI3QHDESC_DE_V<< BIT_SHIFT_WLHI3QHDESC_DE_V<define BIT_GET_BIHI3QHDESC_DE_V)                                                	(((x) &  BIT_SHIFT_RDHI3QHDESC_DE_V<<BIT_MASK_EFHI3QHDESC_DE_V<#*efine BIT_SHIFT_LPHI3QHDESC_NUMIfdefine BIT_MASK_CPHI3QHDESC_NUMIf
fffdefine BIT_SHHI3QHDESC_NUM)                                                     	(((x) & BIT_MASK_EFHI3QHDESC_NUM<< BIT_SHIFT_WLHI3QHDESC_NUM<define BIT_GET_BIHI3QHDESC_NUM)                                                 	(((x) &  BIT_SHIFT_RDHI3QHDESC_NUM<<&IT_MASK_RPHI3QHDESC_NUM<d* 2 REG_PAHI4QHTXBD_NUM(OO_TIMEOUT_PS39 */

#define BIT_SHHI4QHFLAGIT(31)4<#*efine BIT_SHIFT_LPHI4QHDESC_DE_VI1#define BIT_MASK_CPHI4QHDESC_DE_VIf
#define BIT_SEHI4QHDESC_DE_V)                                                 \

	(((x) & BIT_MASK_EFHI4QHDESC_DE_V<< BIT_SHIFT_WLHI4QHDESC_DE_V<define BIT_GET_BIHI4QHDESC_DE_V)                                                	(((x) &  BIT_SHIFT_RDHI4QHDESC_DE_V<<BIT_MASK_EFHI4QHDESC_DE_V<#*efine BIT_SHIFT_LPHI4QHDESC_NUMIfdefine BIT_MASK_CPHI4QHDESC_NUMIf
fffdefine BIT_SHHI4QHDESC_NUM)                                                     	(((x) & BIT_MASK_EFHI4QHDESC_NUM<< BIT_SHIFT_WLHI4QHDESC_NUM<define BIT_GET_BIHI4QHDESC_NUM)                                                 	(((x) &  BIT_SHIFT_RDHI4QHDESC_NUM<<&IT_MASK_RPHI4QHDESC_NUM<d* 2 REG_PAHI5QHTXBD_NUM(OO_TIMEOUT_PS396*/

#define BIT_SHHI5QHFLAGIT(31)4<#*efine BIT_SHIFT_LPHI5QHDESC_DE_VI1#define BIT_MASK_CPHI5QHDESC_DE_VIf
#define BIT_SEHI5QHDESC_DE_V)                                                 \

	(((x) & BIT_MASK_EFHI5QHDESC_DE_V<< BIT_SHIFT_WLHI5QHDESC_DE_V<define BIT_GET_BIHI5QHDESC_DE_V)                                                	(((x) &  BIT_SHIFT_RDHI5QHDESC_DE_V<<BIT_MASK_EFHI5QHDESC_DE_V<#*efine BIT_SHIFT_LPHI5QHDESC_NUMIfdefine BIT_MASK_CPHI5QHDESC_NUMIf
fffdefine BIT_SHHI5QHDESC_NUM)                                                     	(((x) & BIT_MASK_EFHI5QHDESC_NUM<< BIT_SHIFT_WLHI5QHDESC_NUM<define BIT_GET_BIHI5QHDESC_NUM)                                                 	(((x) &  BIT_SHIFT_RDHI5QHDESC_NUM<<&IT_MASK_RPHI5QHDESC_NUM<d* 2 REG_PAHI6QHTXBD_NUM(OO_TIMEOUT_PS39 */

#define BIT_CPHI6QHFLAGIT(31)4<#*efine BIT_SHIFT_LPHI6QHDESC_DE_VI1#define BIT_MASK_CPHI6QHDESC_DE_VIf
#define BIT_SEHI6QHDESC_DE_V)                                                 \

	(((x) & BIT_MASK_EFHI6QHDESC_DE_V<< BIT_SHIFT_WLHI6QHDESC_DE_V<define BIT_GET_BIHI6QHDESC_DE_V)                                                	(((x) &  BIT_SHIFT_RDHI6QHDESC_DE_V<<BIT_MASK_EFHI6QHDESC_DE_V<#*efine BIT_SHIFT_LPHI6QHDESC_NUMIfdefine BIT_MASK_CPHI6QHDESC_NUMIf
fffdefine BIT_SHHI6QHDESC_NUM)                                                     	(((x) & BIT_MASK_EFHI6QHDESC_NUM<< BIT_SHIFT_WLHI6QHDESC_NUM<define BIT_GET_BIHI6QHDESC_NUM)                                                 	(((x) &  BIT_SHIFT_RDHI6QHDESC_NUM<<&IT_MASK_RPHI6QHDESC_NUM<d* 2 REG_PAHI7QHTXBD_NUM(OO_TIMEOUT_PS39A*/

#define BIT_CPHI7QHFLAGIT(31)4<#*efine BIT_SHIFT_LPHI7QHDESC_DE_VI1#define BIT_MASK_CPHI7QHDESC_DE_VIf
#define BIT_SEHI7QHDESC_DE_V)                                                 \

	(((x) & BIT_MASK_EFHI7QHDESC_DE_V<< BIT_SHIFT_WLHI7QHDESC_DE_V<define BIT_GET_BIHI7QHDESC_DE_V)                                                	(((x) &  BIT_SHIFT_RDHI7QHDESC_DE_V<<BIT_MASK_EFHI7QHDESC_DE_V<#*efine BIT_SHIFT_LPHI7QHDESC_NUMIfdefine BIT_MASK_CPHI7QHDESC_NUMIf
fffdefine BIT_SHHI7QHDESC_NUM)                                                     	(((x) & BIT_MASK_EFHI7QHDESC_NUM<< BIT_SHIFT_WLHI7QHDESC_NUM<define BIT_GET_BIHI7QHDESC_NUM)                                                 	(((x) &  BIT_SHIFT_RDHI7QHDESC_NUM<<&IT_MASK_RPHI7QHDESC_NUM<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPHI7QHHW_IDXIT(3129<#efine BIT_CPCLRPHI6QHHW_IDXIT(3128<#efine BIT_CPCLRPHI5QHHW_IDXIT(3127<#efine BIT_CPCLRPHI4QHHW_IDXIT(3126<#efine BIT_CPCLRPHI3QHHW_IDXIT(3125<#efine BIT_CPCLRPHI2QHHW_IDXIT(3124<#efine BIT_CPCLRPHI1QHHW_IDXIT(3123<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPHI0QHHW_IDXIT(3122<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPBKQHHW_IDXIT(3121<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPBEQHHW_IDXIT(3120<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPVIQHHW_IDXIT(3119 dd 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPVOQHHW_IDXIT(3118 dd 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPMGQHHW_IDXIT(3117<d* 2 REG_PATSFTIMER_HCI(OO_TIMEOUT_PS39 */

#define BIT_STIFT_RDTSFT2_HCII
#define BIT_MASK_BITSFT2_HCIIffffffdefine BIT_MATSFT2_HCI) & (x) & &IT_MASK_BITSFT2_HCI<< BIT_SHIFT_WLTSFT2_HCI<define BIT_GET_BITSFT2_HCI) & (x) &  BIT_SHIFT_USTSFT2_HCI<<&IT_MASK_BITSFT2_HCI<#define BIT_STCLRPRXQAHW_IDXIT(3116<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPHI7QHHOST_IDXIT(3113<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPHI6QHHOST_IDXIT(3112<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPHI5QHHOST_IDXIT(3111<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPHI4QHHOST_IDXIT(3110<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPHI3QHHOST_IDXIT(319 dd 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPHI2QHHOST_IDXIT(318 dd 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPHI1QHHOST_IDXIT(317<#efine BIT_CPCLRPHI0QHHOST_IDXIT(316<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPBKQHHOST_IDXIT(315<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPBEQHHOST_IDXIT(314<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPVIQHHOST_IDXIT(313<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPVOQHHOST_IDXIT(312<d* 2 REG_PABD_RWPTR_CLR(OO_TIMEOUT_PS39 */

#define BIT_STCLRPMGQHHOST_IDXIT(311<d* 2 REG_PATSFTIMER_HCI(OO_TIMEOUT_PS39 */

#define BIT_STIFT_RDTSFT1_HCIIfdefine BIT_MASK_BITSFT1_HCIIffffffdefine BIT_MATSFT1_HCI) & (x) & &IT_MASK_BITSFT1_HCI<< BIT_SHIFT_WLTSFT1_HCI<define BIT_GET_BITSFT1_HCI) & (x) &  BIT_SHIFT_USTSFT1_HCI<<&IT_MASK_BITSFT1_HCI<#define BIT_STCLRPRXQAHOST_IDXIT(310<d* 2 REG_PAVOQHTXBD_IDX(OO_TIMEOUT_PS3A */

#define BIT_SHIFT_LPVOQHHW_IDXI
#define BIT_MASK_BIVOQHHW_IDXIff
#fdefine BIT_SEVOQHHW_IDX) & (x) & &IT_MASK_BIVOQHHW_IDX<< BIT_SHIFT_WLVOQHHW_IDX<define BIT_GET_BIVOQHHW_IDX) &                                                  	(((x) &  BIT_SHIFT_USVOQHHW_IDX<<&IT_MASK_BIVOQHHW_IDX<#define BIT_SHIFT_LPVOQHHOST_IDXIfdefine BIT_MASK_CPVOQHHOST_IDXIff
#fdefine BIT_SEVOQHHOST_IDX)                                                      	(((x) & BIT_MASK_EFVOQHHOST_IDX<< BIT_SHIFT_WLVOQHHOST_IDX<define BIT_GET_BIVOQHHOST_IDX)                                                  	(((x) &  BIT_SHIFT_USVOQHHOST_IDX<<BIT_MASK_EFVOQHHOST_IDX<#* 2 REG_PAVIQHTXBD_IDX(OO_TIMEOUT_PS3A */

#define BIT_SHIFT_USVIQHHW_IDXI
#define BIT_MASK_BIVIQHHW_IDXIff
#fdefine BIT_SEVIQHHW_IDX) & (x) & &IT_MASK_BIVIQHHW_IDX<< BIT_SHIFT_WLVIQHHW_IDX<define BIT_GET_BIVIQHHW_IDX) &                                                  	(((x) &  BIT_SHIFT_USVIQHHW_IDX<<&IT_MASK_BIVIQHHW_IDX<#define BIT_SHIFT_LPVIQHHOST_IDXIfdefine BIT_MASK_CPVIQHHOST_IDXIff
#fdefine BIT_SEVIQHHOST_IDX)                                                      	(((x) & BIT_MASK_EFVIQHHOST_IDX<< BIT_SHIFT_WLVIQHHOST_IDX<define BIT_GET_BIVIQHHOST_IDX)                                                  	(((x) &  BIT_SHIFT_USVIQHHOST_IDX<<BIT_MASK_EFVIQHHOST_IDX<#* 2 REG_PABEQHTXBD_IDX(OO_TIMEOUT_PS3A */

#define BIT_CPIFT_USBEQHHW_IDXI
#define BIT_MASK_BIBEQHHW_IDXIff
#fdefine BIT_SEBEQHHW_IDX) & (x) & &IT_MASK_BIBEQHHW_IDX<< BIT_SHIFT_WLBEQHHW_IDX<define BIT_GET_BIBEQHHW_IDX) &                                                  	(((x) &  BIT_SHIFT_USBEQHHW_IDX<<&IT_MASK_BIBEQHHW_IDX<ddefine BIT_CPIFT_USBEQHHOST_IDXIfdefine BIT_MASK_CPBEQHHOST_IDXIff
#fdefine BIT_SEBEQHHOST_IDX)                                                      	(((x) & BIT_MASK_EFBEQHHOST_IDX<< BIT_SHIFT_WLBEQHHOST_IDX<define BIT_GET_BIBEQHHOST_IDX)                                                  	(((x) &  BIT_SHIFT_USBEQHHOST_IDX<<BIT_MASK_EFBEQHHOST_IDX<d* 2 REG_PABKQHTXBD_IDX(OO_TIMEOUT_PS3A */

#define BIT_STIFT_RDBKQHHW_IDXI
#define BIT_MASK_BIBKQHHW_IDXIff
#fdefine BIT_SEBKQHHW_IDX) & (x) & &IT_MASK_BIBKQHHW_IDX<< BIT_SHIFT_WLBKQHHW_IDX<define BIT_GET_BIBKQHHW_IDX) &                                                  	(((x) &  BIT_SHIFT_USBKQHHW_IDX<<&IT_MASK_BIBKQHHW_IDX<ddefine BIT_CPIFT_USBKQHHOST_IDXIfdefine BIT_MASK_CPBKQHHOST_IDXIff
#fdefine BIT_SEBKQHHOST_IDX)                                                      	(((x) & BIT_MASK_EFBKQHHOST_IDX<< BIT_SHIFT_WLBKQHHOST_IDX<define BIT_GET_BIBKQHHOST_IDX)                                                  	(((x) &  BIT_SHIFT_USBKQHHOST_IDX<<BIT_MASK_EFBKQHHOST_IDX<d* 2 REG_PAMGQHTXBD_IDX(OO_TIMEOUT_PS3B */

#define BIT_SHIFT_USMGQHHW_IDXI
#define BIT_MASK_BIMGQHHW_IDXIff
#fdefine BIT_SEMGQHHW_IDX) & (x) & &IT_MASK_BIMGQHHW_IDX<< BIT_SHIFT_WLMGQHHW_IDX<define BIT_GET_BIMGQHHW_IDX) &                                                  	(((x) &  BIT_SHIFT_USMGQHHW_IDX<<&IT_MASK_BIMGQHHW_IDX<ddefine BIT_SHIFT_USMGQHHOST_IDXIfdefine BIT_MASK_CPMGQHHOST_IDXIff
#fdefine BIT_SEMGQHHOST_IDX)                                                      	(((x) & BIT_MASK_EFMGQHHOST_IDX<< BIT_SHIFT_WLMGQHHOST_IDX<define BIT_GET_BIMGQHHOST_IDX)                                                  	(((x) &  BIT_SHIFT_USMGQHHOST_IDX<<BIT_MASK_EFMGQHHOST_IDX<d* 2 REG_PARXQARXBD_IDX(OO_TIMEOUT_PS3B */

#define BIT_SHIFT_SDRXQHHW_IDXI
#define BIT_MASK_BIRXQHHW_IDXIff
#fdefine BIT_SERXQAHW_IDX) & (x) & &IT_MASK_BIRXQAHW_IDX<< BIT_SHIFT_WLRXQAHW_IDX<define BIT_GET_BIRXQAHW_IDX) &                                                  	(((x) &  BIT_SHIFT_USRXQAHW_IDX<<&IT_MASK_BIRXQAHW_IDX<ddefine BIT_SHIFT_SDRXQHHOST_IDXIfdefine BIT_MASK_CPRXQHHOST_IDXIff
#fdefine BIT_SERXQAHOST_IDX)                                                      	(((x) & BIT_MASK_EFRXQAHOST_IDX<< BIT_SHIFT_WLRXQAHOST_IDX<define BIT_GET_BIRXQAHOST_IDX)                                                  	(((x) &  BIT_SHIFT_USRXQAHOST_IDX<<BIT_MASK_EFRXQAHOST_IDX<d* 2 REG_PAHI0QHTXBD_IDX(OO_TIMEOUT_PS3B */

#define BIT_CPIFT_USHI0QHHW_IDXI
#define BIT_MASK_BIHI0QHHW_IDXIf
fffdefine BIT_SHHI0QHHW_IDX) &                                                     	(((x) & BIT_MASK_EFHI0QHHW_IDX<< BIT_SHIFT_WLHI0QHHW_IDX<define BIT_GET_BIHI0QHHW_IDX) &                                                 	(((x) &  BIT_SHIFT_USHI0QHHW_IDX<<&IT_MASK_BIHI0QHHW_IDX<ddefine BIT_CPIFT_USHI0QHHOST_IDXIfdefine BIT_MASK_CPHI0QHHOST_IDXIf
fffdefine BIT_SHHI0QHHOST_IDX)                                                     	(((x) & BIT_MASK_EFHI0QHHOST_IDX<< BIT_SHIFT_WLHI0QHHOST_IDX<define BIT_GET_BIHI0QHHOST_IDX)                                                 	(((x) &  BIT_SHIFT_USHI0QHHOST_IDX<<BIT_MASK_EFHI0QHHOST_IDX<d* 2 REG_PAHI1QHTXBD_IDX(OO_TIMEOUT_PS3B */

#define BIT_STIFT_RDHI1QHHW_IDXI
#define BIT_MASK_BIHI1QHHW_IDXIf
fffdefine BIT_SHHI1QHHW_IDX) &                                                     	(((x) & BIT_MASK_EFHI1QHHW_IDX<< BIT_SHIFT_WLHI1QHHW_IDX<define BIT_GET_BIHI1QHHW_IDX) &                                                 	(((x) &  BIT_SHIFT_RDHI1QHHW_IDX<<&IT_MASK_BIHI1QHHW_IDX<ddefine BIT_CPIFT_USHI1QHHOST_IDXIfdefine BIT_MASK_CPHI1QHHOST_IDXIf
fffdefine BIT_SHHI1QHHOST_IDX)                                                     	(((x) & BIT_MASK_EFHI1QHHOST_IDX<< BIT_SHIFT_WLHI1QHHOST_IDX<define BIT_GET_BIHI1QHHOST_IDX)                                                 	(((x) &  BIT_SHIFT_RDHI1QHHOST_IDX<<BIT_MASK_EFHI1QHHOST_IDX<d* 2 REG_PAHI2QHTXBD_IDX(OO_TIMEOUT_PS3C */

#define BIT_SHIFT_LPHI2QHHW_IDXI
#define BIT_MASK_BIHI2QHHW_IDXIf
fffdefine BIT_SHHI2QHHW_IDX) &                                                     	(((x) & BIT_MASK_EFHI2QHHW_IDX<< BIT_SHIFT_WLHI2QHHW_IDX<define BIT_GET_BIHI2QHHW_IDX) &                                                 	(((x) &  BIT_SHIFT_RDHI2QHHW_IDX<<&IT_MASK_BIHI2QHHW_IDX<ddefine BIT_CPIFT_USHI2QHHOST_IDXIfdefine BIT_MASK_CPHI2QHHOST_IDXIf
fffdefine BIT_SHHI2QHHOST_IDX)                                                     	(((x) & BIT_MASK_EFHI2QHHOST_IDX<< BIT_SHIFT_WLHI2QHHOST_IDX<define BIT_GET_BIHI2QHHOST_IDX)                                                 	(((x) &  BIT_SHIFT_RDHI2QHHOST_IDX<<BIT_MASK_EFHI2QHHOST_IDX<d* 2 REG_PAHI3QHTXBD_IDX(OO_TIMEOUT_PS3C */

#define BIT_SHIFT_SDHI3QHHW_IDXI
#define BIT_MASK_BIHI3QHHW_IDXIf
fffdefine BIT_SHHI3QHHW_IDX) &                                                     	(((x) & BIT_MASK_EFHI3QHHW_IDX<< BIT_SHIFT_WLHI3QHHW_IDX<define BIT_GET_BIHI3QHHW_IDX) &                                                 	(((x) &  BIT_SHIFT_RDHI3QHHW_IDX<<&IT_MASK_BIHI3QHHW_IDX<ddefine BIT_CPIFT_USHI3QHHOST_IDXIfdefine BIT_MASK_CPHI3QHHOST_IDXIf
fffdefine BIT_SHHI3QHHOST_IDX)                                                     	(((x) & BIT_MASK_EFHI3QHHOST_IDX<< BIT_SHIFT_WLHI3QHHOST_IDX<define BIT_GET_BIHI3QHHOST_IDX)                                                 	(((x) &  BIT_SHIFT_RDHI3QHHOST_IDX<<BIT_MASK_EFHI3QHHOST_IDX<d* 2 REG_PAHI4QHTXBD_IDX(OO_TIMEOUT_PS3C */

#define BIT_CPIFT_USHI4QHHW_IDXI
#define BIT_MASK_BIHI4QHHW_IDXIf
fffdefine BIT_SHHI4QHHW_IDX) &                                                     	(((x) & BIT_MASK_EFHI4QHHW_IDX<< BIT_SHIFT_WLHI4QHHW_IDX<define BIT_GET_BIHI4QHHW_IDX) &                                                 	(((x) &  BIT_SHIFT_RDHI4QHHW_IDX<<&IT_MASK_BIHI4QHHW_IDX<ddefine BIT_CPIFT_USHI4QHHOST_IDXIfdefine BIT_MASK_CPHI4QHHOST_IDXIf
fffdefine BIT_SHHI4QHHOST_IDX)                                                     	(((x) & BIT_MASK_EFHI4QHHOST_IDX<< BIT_SHIFT_WLHI4QHHOST_IDX<define BIT_GET_BIHI4QHHOST_IDX)                                                 	(((x) &  BIT_SHIFT_RDHI4QHHOST_IDX<<BIT_MASK_EFHI4QHHOST_IDX<d* 2 REG_PAHI5QHTXBD_IDX(OO_TIMEOUT_PS3C */

#define BIT_STIFT_RDHI5QHHW_IDXI
#define BIT_MASK_BIHI5QHHW_IDXIf
fffdefine BIT_SHHI5QHHW_IDX) &                                                     	(((x) & BIT_MASK_EFHI5QHHW_IDX<< BIT_SHIFT_WLHI5QHHW_IDX<define BIT_GET_BIHI5QHHW_IDX) &                                                 	(((x) &  BIT_SHIFT_RDHI5QHHW_IDX<<&IT_MASK_BIHI5QHHW_IDX<ddefine BIT_CPIFT_USHI5QHHOST_IDXIfdefine BIT_MASK_CPHI5QHHOST_IDXIf
fffdefine BIT_SHHI5QHHOST_IDX)                                                     	(((x) & BIT_MASK_EFHI5QHHOST_IDX<< BIT_SHIFT_WLHI5QHHOST_IDX<define BIT_GET_BIHI5QHHOST_IDX)                                                 	(((x) &  BIT_SHIFT_RDHI5QHHOST_IDX<<BIT_MASK_EFHI5QHHOST_IDX<d* 2 REG_PAHI6QHTXBD_IDX(OO_TIMEOUT_PS3D */

#define BIT_SHIFT_LPHI6QHHW_IDXI
#define BIT_MASK_BIHI6QHHW_IDXIf
fffdefine BIT_SHHI6QHHW_IDX) &                                                     	(((x) & BIT_MASK_EFHI6QHHW_IDX<< BIT_SHIFT_WLHI6QHHW_IDX<define BIT_GET_BIHI6QHHW_IDX) &                                                 	(((x) &  BIT_SHIFT_RDHI6QHHW_IDX<<&IT_MASK_BIHI6QHHW_IDX<ddefine BIT_CPIFT_USHI6QHHOST_IDXIfdefine BIT_MASK_CPHI6QHHOST_IDXIf
fffdefine BIT_SHHI6QHHOST_IDX)                                                     	(((x) & BIT_MASK_EFHI6QHHOST_IDX<< BIT_SHIFT_WLHI6QHHOST_IDX<define BIT_GET_BIHI6QHHOST_IDX)                                                 	(((x) &  BIT_SHIFT_RDHI6QHHOST_IDX<<BIT_MASK_EFHI6QHHOST_IDX<d* 2 REG_PAHI7QHTXBD_IDX(OO_TIMEOUT_PS3D */

#define BIT_SHIFT_SDHI7QHHW_IDXI
#define BIT_MASK_BIHI7QHHW_IDXIf
fffdefine BIT_SHHI7QHHW_IDX) &                                                     	(((x) & BIT_MASK_EFHI7QHHW_IDX<< BIT_SHIFT_WLHI7QHHW_IDX<define BIT_GET_BIHI7QHHW_IDX) &                                                 	(((x) &  BIT_SHIFT_RDHI7QHHW_IDX<<&IT_MASK_BIHI7QHHW_IDX<ddefine BIT_CPIFT_USHI7QHHOST_IDXIfdefine BIT_MASK_CPHI7QHHOST_IDXIf
fffdefine BIT_SHHI7QHHOST_IDX)                                                     	(((x) & BIT_MASK_EFHI7QHHOST_IDX<< BIT_SHIFT_WLHI7QHHOST_IDX<define BIT_GET_BIHI7QHHOST_IDX)                                                 	(((x) &  BIT_SHIFT_RDHI7QHHOST_IDX<<BIT_MASK_EFHI7QHHOST_IDX<d* 2 REG_PADBG_SEL_V1(OOOffset 0x00C3D */

#define BIT_CPDIS_TXDMA_PREIT_C17<#efine BIT_CPDIS_RXDMA_PREIT_C16<#efine BIT_CPTXFLAG_EXIT_L1_ENIT_C12<ddefine BIT_CPIFT_USDBG_SELIfdefine BIT_MASK_CPDBG_SELIfxffdefine BIT_SHDBG_SEL) & (x) & &IT_MASK_BIDBG_SEL<< BIT_SHIFT_WLDBG_SEL<define BIT_GET_BIDBG_SEL) & (x) &  BIT_SHIFT_RDDBG_SEL<<&IT_MASK_BIDBG_SEL<d* 2 REG_PAI_DREHRPWM1_V1(OOffset 0x00C3D9*/

#define BIT_SHIFT_SDI_DREHRPWMIfdefine BIT_MASK_CPI_DREHRPWMIfxffdefine BIT_SHI_DREHRPWM) & (x) & &IT_MASK_BII_DREHRPWM<< BIT_SHIFT_WLI_DREHRPWM<define BIT_GET_BII_DREHRPWM) &                                                  	(((x) &  BIT_SHIFT_RDI_DREHRPWM<<&IT_MASK_BII_DREHRPWM<d* 2 REG_PAI_DREHCPWM1_V1(OOffset 0x00C3DA*/

#define BIT_CPIFT_USI_DREHCPWMIfdefine BIT_MASK_CPI_DREHCPWMIfxffdefine BIT_SHI_DREHCPWM) & (x) & &IT_MASK_BII_DREHCPWM<< BIT_SHIFT_WLI_DREHCPWM<define BIT_GET_BII_DREHCPWM) &                                                  	(((x) &  BIT_SHIFT_RDI_DREHCPWM<<&IT_MASK_BII_DREHCPWM<d* 2 REG_PAI_DRECTRL2(OOOffset 0x00C3DB*/

#define BIT_SHIFT_SDHPS_CLKRAI_DR 4define BIT_MASK_CPHPS_CLKRAI_DR f
#define BIT_SEHPS_CLKRAI_DR)                                                     	(((x) & BIT_MASK_EFHPS_CLKRAI_DR<< BIT_SHIFT_WLHPS_CLKRAI_DR<define BIT_GET_BIHPS_CLKRAI_DR)                                                 	(((x) &  BIT_SHIFT_RDHPS_CLKRAI_DR<<BIT_MASK_EFHPS_CLKRAI_DR<d* 2 REG_PAI_DRECTRL2(OOOffset 0x00C3DB*/

#define BIT_SHI_DREINTIT_C13<d* 2 REG_PAI_DRECTRL2(OOOffset 0x00C3DB*/

#define BIT_SHEN_RXDMA_ALIGNIT_C11)define BIT_SHEN_TXDMA_ALIGNIT_C10<d* 2 REG_PAI_DREHRPWM2_V1(OOffset 0x00C3D */

#define BIT_STIFT_RDI_DREHRPWM2Ifdefine BIT_MASK_CPI_DREHRPWM2Iffffffdefine BIT_MAI_DREHRPWM2) &                                                     	(((x) & BIT_MASK_EFI_DREHRPWM2<< BIT_SHIFT_WLI_DREHRPWM2<define BIT_GET_BII_DREHRPWM2) &                                                 	(((x) &  BIT_SHIFT_RDI_DREHRPWM2& BIT_MASK_EFI_DREHRPWM2<d* 2 REG_PAI_DREHCPWM2_V1(OOffset 0x00C3DE*/

#define BIT_STIFT_RDI_DREHCPWM2Ifdefine BIT_MASK_CPI_DREHCPWM2Iffffffdefine BIT_MAI_DREHCPWM2) &                                                     	(((x) & BIT_MASK_EFI_DREHCPWM2<< BIT_SHIFT_WLI_DREHCPWM2<define BIT_GET_BII_DREHCPWM2) &                                                 	(((x) &  BIT_SHIFT_RDI_DREHCPWM2& BIT_MASK_EFI_DREHCPWM2<d* 2 REG_PAI_DREH2C_MSG_V1(OOffset 0x00C3E */

#define BIT_SHIFT_LPDRV2FW_INFOIfdefine BIT_MASK_CPDRV2FW_INFOIffffffffL
#define BIT_DEDRV2FW_INFO) &                                                     	(((x) & BIT_MASK_EFDRV2FW_INFO<< BIT_SHIFT_WLDRV2FW_INFO<define BIT_GET_BIDRV2FW_INFO) &                                                 	(((x) &  BIT_SHIFT_RDDRV2FW_INFO<<BIT_MASK_EFDRV2FW_INFO<d* 2 REG_PAI_DREC2H_MSG_V1(OOffset 0x00C3E */

#define BIT_SHIFT_SDHCIAI_DREC2H_MSGIfdefine BIT_MASK_CPHCIAI_DREC2H_MSGIffffffffL
#define BIT_DEHCIAI_DREC2H_MSG)                                                  	(((x) & &IT_MASK_CPHCIAI_DREC2H_MSG<< BIT_SHIFT_WLHCIAI_DREC2H_MSG<define BIT_GET_BIHCIAI_DREC2H_MSG)                                              	(((x) &  BIT_SHIFT_RDHCIAI_DREC2H_MSG<<&IT_MASK_CPHCIAI_DREC2H_MSG<d* 2 REG_PADBI_WDATA_V1(OOffset 0x00C3E */

#define BIT_CPIFT_USDBI_WDATAIfdefine BIT_MASK_CPDBI_WDATAIffffffffL
#define BIT_DEDBI_WDATA) & (x) & &IT_MASK_BIDBI_WDATA<< BIT_SHIFT_WLDBI_WDATA<define BIT_GET_BIDBI_WDATA) & (x) &  BIT_SHIFT_RDDBI_WDATA<<&IT_MASK_BIDBI_WDATA<d* 2 REG_PADBI_RDATA_V1(OOffset 0x00C3E */

#define BIT_STIFT_RDDBI_RDATAIfdefine BIT_MASK_CPDBI_RDATAIffffffffL
#define BIT_DEDBI_RDATA) & (x) & &IT_MASK_BIDBI_RDATA<< BIT_SHIFT_WLDBI_RDATA<define BIT_GET_BIDBI_RDATA) & (x) &  BIT_SHIFT_RDDBI_RDATA<<&IT_MASK_BIDBI_RDATA<d* 2 REG_PADBI_FLAG_V1(OOOffset 0x00C3F */

#define BIT_SHEN_STUCBIDBGIT_C126<#efine BIT_CPRX_STUCBIT_C125<#efine BIT_CPTX_STUCBIT_C124<#efine BIT_CPDBI_RFLAGIT(31)7<#efine BIT_CPDBI_WFLAGIT(31)6<ddefine BIT_CPIFT_USDBI_WRENI1#define BIT_MASK_CPDBI_WRENIfff#efine BIT_CPDBI_WREN) & (x) & &IT_MASK_BIDBI_WREN<< BIT_SHIFT_WLDBI_WREN<define BIT_GET_BIDBI_WREN) & (x) &  BIT_SHIFT_RDDBI_WREN<<&IT_MASK_BIDBI_WREN<ddefine BIT_CPIFT_USDBI_ADDRIfdefine BIT_MASK_CPDBI_ADDRIf
fffdefine BIT_SHDBI_ADDR) & (x) & &IT_MASK_BIDBI_ADDR<< BIT_SHIFT_WLDBI_ADDR<define BIT_GET_BIDBI_ADDR) & (x) &  BIT_SHIFT_RDDBI_ADDR<<&IT_MASK_BIDBI_ADDR<d* 2 REG_PAMDIO_V1(OOOffset 0x00C3F */

#define BIT_SHIFT_SDMDIO_RDATAI
#define BIT_MASK_BIMDIO_RDATAIffffffdefine BIT_MAMDIO_RDATA) & (x) & &IT_MASK_BIMDIO_RDATA<< BIT_SHIFT_WLMDIO_RDATA<define BIT_GET_BIMDIO_RDATA) &                                                  	(((x) &  BIT_SHIFT_USMDIO_RDATA<<&IT_MASK_BIMDIO_RDATA<#define BIT_SHIFT_SDMDIO_WDATAIfdefine BIT_MASK_CPMDIO_WDATAIffffffdefine BIT_MAMDIO_WDATA) & (x) & &IT_MASK_BIMDIO_WDATA<< BIT_SHIFT_WLMDIO_WDATA<define BIT_GET_BIMDIO_WDATA) &                                                  	(((x) &  BIT_SHIFT_USMDIO_WDATA<<&IT_MASK_BIMDIO_WDATA<d* 2 REG_PAI_DREMIX_CFGOOOffset 0x00C3F */

#define BIT_CPEN_WATCH_DOGIT(318<d* 2 REG_PAI_DREMIX_CFGOOOffset 0x00C3F */

#define BIT_CPIFT_USMDIO_R_PAADDR_V1Ifdefine BIT_MASK_CPMDIO_R_PAADDR_V1Ifx1fdefine BIT_MAMDIO_R_PAADDR_V1)                                                  	(((x) & &IT_MASK_CPMDIO_R_PAADDR_V1<< BIT_SHIFT_WLMDIO_R_PAADDR_V1<define BIT_GET_BIMDIO_R_PAADDR_V1)                                              	(((x) &  BIT_SHIFT_USMDIO_R_PAADDR_V1<<&IT_MASK_CPMDIO_R_PAADDR_V1<d* 2 REG_PAHCIAMIX_CFGOOOOffset 0x00C3F */

#define BIT_STHOST_GEN2_SUPPORTIT_C120<d*efine BIT_CPIFT_USTXDMA_ERRHFLAGI
#define BIT_MASK_BITXDMA_ERRHFLAGIfff#efine BIT_CPTXDMA_ERRHFLAG) &                                                  	(((x) & &IT_MASK_BITXDMA_ERRHFLAG<< BIT_SHIFT_WLTXDMA_ERRHFLAG<define BIT_GET_BITXDMA_ERRHFLAG) &                                              	(((x) &  BIT_SHIFT_USTXDMA_ERRHFLAG<<&IT_MASK_BITXDMA_ERRHFLAG<d*efine BIT_CPIFT_USEARLY_DE_V_SELI1#define BIT_MASK_CPEARLY_DE_V_SELIfff#efine BIT_CPEARLY_DE_V_SEL) &                                                  	(((x) & &IT_MASK_BIEARLY_DE_V_SEL<< BIT_SHIFT_WLEARLY_DE_V_SEL<define BIT_GET_BIEARLY_DE_V_SEL) &                                              	(((x) &  BIT_SHIFT_USEARLY_DE_V_SEL<<&IT_MASK_BIEARLY_DE_V_SEL<#define BIT_CPEPHYPRX50_ENIT_C111<d*efine BIT_CPIFT_USMSI_TIMEOUT_ID_V1I8define BIT_MASK_CPMSI_TIMEOUT_ID_V1I0x7define BIT_MASSI_TIMEOUT_ID_V1)                                                 	(((x) & &IT_MASK_CPMSI_TIMEOUT_ID_V1<< BIT_SHIFT_WLMSI_TIMEOUT_ID_V1<define BIT_GET_BIMSI_TIMEOUT_ID_V1)                                             	(((x) &  BIT_SHIFT_USMSI_TIMEOUT_ID_V1<<&IT_MASK_CPMSI_TIMEOUT_ID_V1<d*efine BIT_CPRADDR_RDIT_C17)define BIT_SHEN_MUL_TAGIT(316)define BIT_SHEN_EARLY_DE_VIT(315<#efine BIT_CPL0S_LINK_OFFIT(314<#efine BIT_CPACT_LINK_OFFIT(313<d* 2 REG_PAHCIAMIX_CFGOOOOffset 0x00C3F */

#define BIT_STEN_SLOWASKCITXIT_C12)define BIT_SHEN_SLOWASKCIRXIT(311<d* 2 REG_PAQ0_INFOOOOOffset 0x00C40 */

#define BIT_SHIFT_LPQUEUESKCID_Q0_V1I25define BIT_MASK_CPQUEUESKCID_Q0_V1I0x7f#efine BIT_CPQUEUESKCID_Q0_V1)                                                  	(((x) & &IT_MASK_CPQUEUESKCID_Q0_V1<< BIT_SHIFT_WLQUEUESKCID_Q0_V1<define BIT_GET_BIQUEUESKCID_Q0_V1)                                              	(((x) &  BIT_SHIFT_USQUEUESKCID_Q0_V1<<&IT_MASK_CPQUEUESKCID_Q0_V1<#define BIT_SHIFT_LPQUEUEKCIQ0_V1I23define BIT_MASK_CPQUEUEKCIQ0_V1If
#define BIT_SEQUEUEKCIQ0_V1)                                                     	(((x) & BIT_MASK_EFQUEUEKCIQ0_V1<< BIT_SHIFT_WLQUEUEKCIQ0_V1<define BIT_GET_BIQUEUEKCIQ0_V1)                                                 	(((x) &  BIT_SHIFT_USQUEUEKCIQ0_V1<<BIT_MASK_EFQUEUEKCIQ0_V1<d* 2 REG_PAQ0_INFOOOOOffset 0x00C40 */

#define BIT_SHTIDEMPTYIQ0_V1IB(3122<d* 2 REG_PAQ0_INFOOOOOffset 0x00C40 */

#define BIT_SHIFT_LPTAIL_PKTIQ0_V2 11define BIT_MASK_BITAIL_PKTIQ0_V2 0x7ff#efine BIT_CPTAIL_PKTIQ0_V2) &                                                  	(((x) & &IT_MASK_BITAIL_PKTIQ0_V2<< BIT_SHIFT_WLTAIL_PKTIQ0_V2<define BIT_GET_BITAIL_PKTIQ0_V2) &                                              	(((x) &  BIT_SHIFT_USTAIL_PKTIQ0_V2<<&IT_MASK_BITAIL_PKTIQ0_V2<d* 2 REG_PAQ0_INFOOOOOffset 0x00C40 */

#define BIT_SHIFT_LPHEAD_PKTIQ0_V1Ifdefine BIT_MASK_CPHEAD_PKTIQ0_V1Ifx7ff#efine BIT_CPHEAD_PKTIQ0_V1) &                                                  	(((x) & &IT_MASK_BIHEAD_PKTIQ0_V1<< BIT_SHIFT_WLHEAD_PKTIQ0_V1<define BIT_GET_BIHEAD_PKTIQ0_V1) &                                              	(((x) &  BIT_SHIFT_RDHEAD_PKTIQ0_V1<<&IT_MASK_BIHEAD_PKTIQ0_V1<d* 2 REG_PAQ1_INFOOOOOffset 0x00C40 */

#define BIT_SHIFT_SDQUEUESKCID_Q1_V1I25define BIT_MASK_CPQUEUESKCID_Q1_V1I0x7f#efine BIT_CPQUEUESKCID_Q1_V1)                                                  	(((x) & &IT_MASK_CPQUEUESKCID_Q1_V1<< BIT_SHIFT_WLQUEUESKCID_Q1_V1<define BIT_GET_BIQUEUESKCID_Q1_V1)                                              	(((x) &  BIT_SHIFT_USQUEUESKCID_Q1_V1<<&IT_MASK_CPQUEUESKCID_Q1_V1<#define BIT_SHIFT_LPQUEUEKCIQ1_V1I23define BIT_MASK_CPQUEUEKCIQ1_V1If
#define BIT_SEQUEUEKCIQ1_V1)                                                     	(((x) & &IT_MASK_CPQUEUEKCIQ1_V1<< BIT_SHIFT_WLQUEUEKCIQ1_V1<define BIT_GET_BIQUEUEKCIQ1_V1)                                                 	(((x) &  BIT_SHIFT_USQUEUEKCIQ1_V1<<&IT_MASK_CPQUEUEKCIQ1_V1<d* 2 REG_PAQ1_INFOOOOOffset 0x00C40 */

#define BIT_SHTIDEMPTYIQ1_V1IB(3122<d* 2 REG_PAQ1_INFOOOOOffset 0x00C40 */

#define BIT_SHIFT_SDTAIL_PKTIQ1_V2 11define BIT_MASK_BITAIL_PKTIQ1_V2 0x7ff#efine BIT_CPTAIL_PKTIQ1_V2) &                                                  	(((x) & &IT_MASK_BITAIL_PKTIQ1_V2<< BIT_SHIFT_WLTAIL_PKTIQ1_V2<define BIT_GET_BITAIL_PKTIQ1_V2) &                                              	(((x) &  BIT_SHIFT_USTAIL_PKTIQ1_V2<<&IT_MASK_BITAIL_PKTIQ1_V2<d* 2 REG_PAQ1_INFOOOOOffset 0x00C40 */

#define BIT_SHIFT_SDHEAD_PKTIQ1_V1Ifdefine BIT_MASK_CPHEAD_PKTIQ1_V1Ifx7ff#efine BIT_CPHEAD_PKTIQ1_V1)                                                    	(((x) & &IT_MASK_BIHEAD_PKTIQ1_V1<< BIT_SHIFT_WLHEAD_PKTIQ1_V1<define BIT_GET_BIHEAD_PKTIQ1_V1)                                                	(((x) &  BIT_SHIFT_RDHEAD_PKTIQ1_V1<<&IT_MASK_CPHEAD_PKTIQ1_V1<d* 2 REG_PAQ2_INFOOOOOffset 0x00C40 */

#define BIT_CPIFT_USQUEUESKCID_Q2_V1I25define BIT_MASK_CPQUEUESKCID_Q2_V1I0x7f#efine BIT_CPQUEUESKCID_Q2_V1)                                                  	(((x) & &IT_MASK_CPQUEUESKCID_Q2_V1<< BIT_SHIFT_WLQUEUESKCID_Q2_V1<define BIT_GET_BIQUEUESKCID_Q2_V1)                                              	(((x) &  BIT_SHIFT_USQUEUESKCID_Q2_V1<<&IT_MASK_CPQUEUESKCID_Q2_V1<#define BIT_SHIFT_LPQUEUEKCIQ2_V1I23define BIT_MASK_CPQUEUEKCIQ2_V1If
#define BIT_SEQUEUEKCIQ2_V1)                                                     	(((x) & &IT_MASK_CPQUEUEKCIQ2_V1<< BIT_SHIFT_WLQUEUEKCIQ2_V1<define BIT_GET_BIQUEUEKCIQ2_V1)                                                 	(((x) &  BIT_SHIFT_USQUEUEKCIQ2_V1<<&IT_MASK_CPQUEUEKCIQ2_V1<d* 2 REG_PAQ2_INFOOOOOffset 0x00C40 */

#define BIT_CPTIDEMPTYIQ2_V1IB(3122<d* 2 REG_PAQ2_INFOOOOOffset 0x00C40 */

#define BIT_CPIFT_USTAIL_PKTIQ2_V2 11define BIT_MASK_BITAIL_PKTIQ2_V2 0x7ff#efine BIT_CPTAIL_PKTIQ2_V2) &                                                  	(((x) & &IT_MASK_BITAIL_PKTIQ2_V2<< BIT_SHIFT_WLTAIL_PKTIQ2_V2<define BIT_GET_BITAIL_PKTIQ2_V2) &                                              	(((x) &  BIT_SHIFT_USTAIL_PKTIQ2_V2<<&IT_MASK_BITAIL_PKTIQ2_V2<d* 2 REG_PAQ2_INFOOOOOffset 0x00C40 */

#define BIT_CPIFT_USHEAD_PKTIQ2_V1Ifdefine BIT_MASK_CPHEAD_PKTIQ2_V1Ifx7ff#efine BIT_CPHEAD_PKTIQ2_V1)                                                    	(((x) & &IT_MASK_BIHEAD_PKTIQ2_V1<< BIT_SHIFT_WLHEAD_PKTIQ2_V1<define BIT_GET_BIHEAD_PKTIQ2_V1)                                                	(((x) &  BIT_SHIFT_RDHEAD_PKTIQ2_V1<<&IT_MASK_CPHEAD_PKTIQ2_V1<d* 2 REG_PAQ3_INFOOOOOffset 0x00C40 */

#define BIT_STIFT_RDQUEUESKCID_Q3_V1I25define BIT_MASK_CPQUEUESKCID_Q3_V1I0x7f#efine BIT_CPQUEUESKCID_Q3_V1)                                                  	(((x) & &IT_MASK_CPQUEUESKCID_Q3_V1<< BIT_SHIFT_WLQUEUESKCID_Q3_V1<define BIT_GET_BIQUEUESKCID_Q3_V1)                                              	(((x) &  BIT_SHIFT_USQUEUESKCID_Q3_V1<<&IT_MASK_CPQUEUESKCID_Q3_V1<#define BIT_SHIFT_LPQUEUEKCIQ3_V1I23define BIT_MASK_CPQUEUEKCIQ3_V1If
#define BIT_SEQUEUEKCIQ3_V1)                                                     	(((x) & &IT_MASK_CPQUEUEKCIQ3_V1<< BIT_SHIFT_WLQUEUEKCIQ3_V1<define BIT_GET_BIQUEUEKCIQ3_V1)                                                 	(((x) &  BIT_SHIFT_USQUEUEKCIQ3_V1<<&IT_MASK_CPQUEUEKCIQ3_V1<d* 2 REG_PAQ3_INFOOOOOffset 0x00C40 */

#define BIT_STTIDEMPTYIQ3_V1IB(3122<d* 2 REG_PAQ3_INFOOOOOffset 0x00C40 */

#define BIT_STIFT_RDTAIL_PKTIQ3_V2 11define BIT_MASK_BITAIL_PKTIQ3_V2 0x7ff#efine BIT_CPTAIL_PKTIQ3_V2) &                                                  	(((x) & &IT_MASK_BITAIL_PKTIQ3_V2<< BIT_SHIFT_WLTAIL_PKTIQ3_V2<define BIT_GET_BITAIL_PKTIQ3_V2) &                                              	(((x) &  BIT_SHIFT_USTAIL_PKTIQ3_V2<<&IT_MASK_BITAIL_PKTIQ3_V2<d* 2 REG_PAQ3_INFOOOOOffset 0x00C40 */

#define BIT_STIFT_RDHEAD_PKTIQ3_V1Ifdefine BIT_MASK_CPHEAD_PKTIQ3_V1Ifx7ff#efine BIT_CPHEAD_PKTIQ3_V1)                                                    	(((x) & &IT_MASK_BIHEAD_PKTIQ3_V1<< BIT_SHIFT_WLHEAD_PKTIQ3_V1<define BIT_GET_BIHEAD_PKTIQ3_V1)                                                	(((x) &  BIT_SHIFT_RDHEAD_PKTIQ3_V1<<&IT_MASK_CPHEAD_PKTIQ3_V1<d* 2 REG_PAMGQHINFOOOOOffset 0x00C41 */

#define BIT_SHIFT_LPQUEUESKCID_MGQHV1I25define BIT_MASK_CPQUEUESKCID_MGQHV1I0x7f#efine BIT_CPQUEUESKCID_MGQHV1)                                                 	(((x) & &IT_MASK_CPQUEUESKCID_MGQHV1<< BIT_SHIFT_WLQUEUESKCID_MGQHV1<define BIT_GET_BIQUEUESKCID_MGQHV1)                                             	(((x) &  BIT_SHIFT_USQUEUESKCID_MGQHV1<<&IT_MASK_CPQUEUESKCID_MGQHV1<#define BIT_SHIFT_LPQUEUEKCIMGQHV1I23define BIT_MASK_CPQUEUEKCIMGQHV1I0x#define BIT_SEQUEUEKCIMGQHV1)                                                    	(((x) & &IT_MASK_CPQUEUEKCIMGQHV1<< BIT_SHIFT_WLQUEUEKCIMGQHV1<define BIT_GET_BIQUEUEKCIMGQHV1)                                                	(((x) &  BIT_SHIFT_USQUEUEKCIMGQHV1<<&IT_MASK_CPQUEUEKCIMGQHV1<d* 2 REG_PAMGQHINFOOOOOffset 0x00C41 */

#define BIT_SHTIDEMPTYIMGQHV1IB(3122<d* 2 REG_PAMGQHINFOOOOOffset 0x00C41 */

#define BIT_SHIFT_LPTAIL_PKTIMGQHV2 11define BIT_MASK_BITAIL_PKTIMGQHV2 0x7ff#efine BIT_CPTAIL_PKTIMGQHV2) &                                                 	(((x) & &IT_MASK_BITAIL_PKTIMGQHV2<< BIT_SHIFT_WLTAIL_PKTIMGQHV2<define BIT_GET_BITAIL_PKTIMGQHV2) &                                             	(((x) &  BIT_SHIFT_USTAIL_PKTIMGQHV2<<&IT_MASK_BITAIL_PKTIMGQHV2<d* 2 REG_PAMGQHINFOOOOOffset 0x00C41 */

#define BIT_SHIFT_LPHEAD_PKTIMGQHV1I0define BIT_MASK_CPHEAD_PKTIMGQHV1I0x7ff#efine BIT_CPHEAD_PKTIMGQHV1)                                                   	(((x) & &IT_MASK_BIHEAD_PKTIMGQHV1<< BIT_SHIFT_WLHEAD_PKTIMGQHV1<define BIT_GET_BIHEAD_PKTIMGQHV1)                                               	(((x) &  BIT_SHIFT_RDHEAD_PKTIMGQHV1<<&IT_MASK_CPHEAD_PKTIMGQHV1<d* 2 REG_PAHIQHINFOOOOOffset 0x00C41 */

#define BIT_SHIFT_SDQUEUESKCID_HIQHV1I25define BIT_MASK_CPQUEUESKCID_HIQHV1I0x7f#efine BIT_CPQUEUESKCID_HIQHV1)                                                 	(((x) & &IT_MASK_CPQUEUESKCID_HIQHV1<< BIT_SHIFT_WLQUEUESKCID_HIQHV1<define BIT_GET_BIQUEUESKCID_HIQHV1)                                             	(((x) &  BIT_SHIFT_USQUEUESKCID_HIQHV1<<&IT_MASK_CPQUEUESKCID_HIQHV1<#define BIT_SHIFT_LPQUEUEKCIHIQHV1I23define BIT_MASK_CPQUEUEKCIHIQHV1I0x#define BIT_SEQUEUEKCIHIQHV1)                                                    	(((x) & &IT_MASK_CPQUEUEKCIHIQHV1<< BIT_SHIFT_WLQUEUEKCIHIQHV1<define BIT_GET_BIQUEUEKCIHIQHV1)                                                	(((x) &  BIT_SHIFT_USQUEUEKCIHIQHV1<<&IT_MASK_CPQUEUEKCIHIQHV1<d* 2 REG_PAHIQHINFOOOOOffset 0x00C41 */

#define BIT_SHTIDEMPTYIHIQHV1IB(3122<d* 2 REG_PAHIQHINFOOOOOffset 0x00C41 */

#define BIT_SHIFT_SDTAIL_PKTIHIQHV2 11define BIT_MASK_BITAIL_PKTIHIQHV2 0x7ff#efine BIT_CPTAIL_PKTIHIQHV2) &                                                 	(((x) & &IT_MASK_BITAIL_PKTIHIQHV2<< BIT_SHIFT_WLTAIL_PKTIHIQHV2<define BIT_GET_BITAIL_PKTIHIQHV2) &                                             	(((x) &  BIT_SHIFT_USTAIL_PKTIHIQHV2<<&IT_MASK_BITAIL_PKTIHIQHV2<d* 2 REG_PAHIQHINFOOOOOffset 0x00C41 */

#define BIT_SHIFT_SDHEAD_PKTIHIQHV1I0define BIT_MASK_CPHEAD_PKTIHIQHV1I0x7ff#efine BIT_CPHEAD_PKTIHIQHV1)                                                   	(((x) & &IT_MASK_BIHEAD_PKTIHIQHV1<< BIT_SHIFT_WLHEAD_PKTIHIQHV1<define BIT_GET_BIHEAD_PKTIHIQHV1)                                               	(((x) &  BIT_SHIFT_RDHEAD_PKTIHIQHV1<<&IT_MASK_CPHEAD_PKTIHIQHV1<d* 2 REG_PABCNQHINFOOOOOffset 0x00C41 */

#define BIT_CPIFT_USBCNQHHEAD_PGHV1I0define BIT_MASK_CPBCNQHHEAD_PGHV1I0f
#fdefine BIT_SEBCNQHHEAD_PGHV1)                                                   	(((x) & &IT_MASK_BIBCNQHHEAD_PGHV1<< BIT_SHIFT_WLBCNQHHEAD_PGHV1<define BIT_GET_BIBCNQHHEAD_PGHV1)                                               	(((x) &  BIT_SHIFT_USBCNQHHEAD_PGHV1<<&IT_MASK_BIBCNQHHEAD_PGHV1<d* 2 REG_PATXPKTIEMPTYOOOOffset 0x00C41A*/

#define BIT_CPBCNQHEMPTYIB(3111<define BIT_GEHQQHEMPTYIB(3110<define BIT_GEMQQHEMPTYIB(319<#efine BIT_CPMGQHCPUHEMPTYIB(318<#efine BIT_CPAC7QHEMPTYIB(317<#efine BIT_CPAC6QHEMPTYIB(316<#efine BIT_CPAC5QHEMPTYIB(315<#efine BIT_CPAC4QHEMPTYIB(314<#efine BIT_CPAC3QHEMPTYIB(313<#efine BIT_CPAC2QHEMPTYIB(312<#efine BIT_CPAC1QHEMPTYIB(311<#efine BIT_CPAC0QHEMPTYIB(310<d* 2 REG_PACPUHMGQHINFOOOOffset 0x00C41 */

#define BIT_STBCN1_POLLIB(3130<d* 2 REG_PACPUHMGQHINFOOOOffset 0x00C41 */

#define BIT_STCPUMGT_POLLIB(3129<#efine BIT_CPBCN_POLLIB(3128<d* 2 REG_PACPUHMGQHINFOOOOffset 0x00C41 */

#define BIT_STCPUMGQ_FW_NUMHV1IB(3112<d* 2 REG_PACPUHMGQHINFOOOOffset 0x00C41 */

#define BIT_STIFT_USFW_FREEITAIL_V1I0define BIT_MASK_CPFW_FREEITAIL_V1I0f
#fdefine BIT_SEFW_FREEITAIL_V1)                                                   	(((x) & &IT_MASK_BIFW_FREEITAIL_V1<< BIT_SHIFT_WLFW_FREEITAIL_V1<define BIT_GET_BIFW_FREEITAIL_V1)                                               	(((x) &  BIT_SHIFT_USFW_FREEITAIL_V1<<&IT_MASK_BIFW_FREEITAIL_V1<d* 2 REG_PAFWHWATXQECTRLOOOffset 0x00C42 */

#define BIT_SHRTS_LIM_SHIN_OFDMIB(3123<#efine BIT_CPENPBCNQHDLIB(3122<define BIT_CPENPRD_RESP_NAV_BBIT_C121)define BIT_SHEN_WR_FREEITAILIT_C120<d*efine BIT_CPIFT_USEN_QUEUE_RPTI8define BIT_MASK_CPEN_QUEUE_RPTIfxffdefine BIT_SHEN_QUEUE_RPT)                                                      	(((x) & &IT_MASK_BIEN_QUEUE_RPT<< BIT_SHIFT_WLEN_QUEUE_RPT<define BIT_GET_BIEN_QUEUE_RPT)                                                  	(((x) &  BIT_SHIFT_USEN_QUEUE_RPT<<&IT_MASK_BIEN_QUEUE_RPT<#define BIT_SHEN_RTYIBBIT_C17)define BIT_SHEN_USREINI_RATIB(316<#efine BIT_CPEN_RTS_NAV_BBIT_C15<#efine BIT_CPDIS_SSN_CHECBIT_C14<#efine BIT_CPSKCID_MATCH_RTSIB(313<#* 2 REG_PAFWHWATXQECTRLOOOffset 0x00C42 */

#define BIT_SHENPBCN_TRXRPTHV1IB(312<#* 2 REG_PAFWHWATXQECTRLOOOffset 0x00C42 */

#define BIT_SHENPFTSKCKRPTIB(311<d* 2 REG_PAFWHWATXQECTRLOOOffset 0x00C42 */

#define BIT_SHENPFTSRPTIB(310<d* 2 REG_PADATAFB_SELOOOOffset 0x00C423</

#define BIT_SH_RHEN_RTYIBB_CODIB(312<#* 2 REG_PADATAFB_SELOOOOffset 0x00C423</

#define BIT_SHIFT_US_RHDATA_FALLBKCK_SELIfdefine BIT_MASK_CP_RHDATA_FALLBKCK_SELIfx#define BIT_SE_RHDATA_FALLBKCK_SEL)                                              	(((x) & &IT_MASK_CP_RHDATA_FALLBKCK_SEL                                  	((< BIT_SHIFT_WL_RHDATA_FALLBKCK_SEL define BIT_GET_BI_RHDATA_FALLBKCK_SEL)                                          	(((x) &  BIT_SHIFT_US_RHDATA_FALLBKCK_SEL  &                             	((<T_MASK_CP_RHDATA_FALLBKCK_SEL d* 2 REG_PABCNQHBDNY_V1(OOffset 0x00C42 */

#define BIT_SHIFT_SDBCNQHPGBNDYHV1I0define BIT_MASK_CPBCNQHPGBNDYHV1I0f
#fdefine BIT_SEBCNQHPGBNDYHV1)                                                    	(((x) & &IT_MASK_CPBCNQHPGBNDYHV1<< BIT_SHIFT_WLBCNQHPGBNDYHV1<define BIT_GET_BIBCNQHPGBNDYHV1)                                                	(((x) &  BIT_SHIFT_USBCNQHPGBNDYHV1<<&IT_MASK_CPBCNQHPGBNDYHV1<d* 2 REG_PALIFETIMEHENOOOOffset 0x00C426*/

#define BIT_STBSHINSTCPUIT_C17)define BIT_SHBSHINSTPTAIB(316<#* 2 REG_PALIFETIMEHENOOOOffset 0x00C426*/

#define BIT_STEN_CTRL_RTYT_SIT_C14<#* 2 REG_PALIFETIMEHENOOOOffset 0x00C426*/

#define BIT_STLIFETIMEHBK_ENIT_C13<#efine BIT_CPLIFETIMEHBE_ENIT_C12<define BIT_CPLIFETIMEHVI_ENIT_C11<define BIT_CPLIFETIMEHVO_ENIT_C10<d* 2 REG_PASPEC_SIFSOOOOffset 0x00C42 */

#define BIT_CPIFT_USSPEC_SIFS_OFDMTPTCLI8define BIT_MASK_CPSPEC_SIFS_OFDMTPTCLIfxffdefine BIT_SHSPEC_SIFS_OFDMTPTCL)                                               	(((x) & &IT_MASK_CPSPEC_SIFS_OFDMTPTCL<< BIT_SHIFT_WLSPEC_SIFS_OFDMTPTCL<define BIT_GET_BISPEC_SIFS_OFDMTPTCL)                                           	(((x) &  BIT_SHIFT_USSPEC_SIFS_OFDMTPTCL<<&IT_MASK_CPSPEC_SIFS_OFDMTPTCL<#define BIT_CPIFT_USSPEC_SIFS_CCK_PTCLIfdefine BIT_MASK_CPSPEC_SIFS_CCK_PTCLIfxffdefine BIT_SHSPEC_SIFS_CCK_PTCL)                                                	(((x) & &IT_MASK_CPSPEC_SIFS_CCK_PTCL<< BIT_SHIFT_WLSPEC_SIFS_CCK_PTCL<define BIT_GET_BISPEC_SIFS_CCK_PTCL)                                            	(((x) &  BIT_SHIFT_USSPEC_SIFS_CCK_PTCL<<&IT_MASK_CPSPEC_SIFS_CCK_PTCL<d* 2 REG_PARETRY_LIM_SOOOOffset 0x00C42A*/

#define BIT_CPIFT_USSRLI8define BIT_MASK_CPSRLIfx#fdefine BIT_SHSRL) & (x) & &IT_MASK_BISRL<< BIT_SHIFT_WLSRL<define BIT_GET_BISRL) & (x) &  BIT_SHIFT_RDSRL<<&IT_MASK_BISRL<#define BIT_CPIFT_USLRLIfdefine BIT_MASK_CPLRLIfx#fdefine BIT_SHLRL) & (x) & &IT_MASK_BILRL<< BIT_SHIFT_WLLRL<define BIT_GET_BILRL) & (x) &  BIT_SHIFT_RDLRL<<&IT_MASK_BILRL<d* 2 REG_PATXBFECTRLOOOOffset 0x00C42 */

#define BIT_STRHENABLE_NDPAIB(3131<define BIT_CPUSRENDPA_PARAMETERIB(3130<define BIT_STRHPROPATXBFIB(3129<#efine BIT_CPRHEN_NDPA_INTIT_C128<#efine BIT_CPRHTXBF1_80MIB(3127<#efine BIT_CPRHTXBF1_40MIB(3126<#efine BIT_CPRHTXBF1_20MIB(3125<ddefine BIT_SHIFT_SDRHTXBF1_AIDI
#define BIT_MASK_BIRHTXBF1_AIDIfx1ff#efine BIT_CPRHTXBF1_AID)                                                       	(((x) & BIT_MASK_EFRHTXBF1_AID<< BIT_SHIFT_WLRHTXBF1_AID<define BIT_GET_BIRHTXBF1_AID)                                                   	(((x) &  BIT_SHIFT_USRHTXBF1_AID<<BIT_MASK_EFRHTXBF1_AID<d* 2 REG_PATXBFECTRLOOOOffset 0x00C42 */

#define BIT_STDIS_NDP_BFENIT_C115<d* 2 REG_PATXBFECTRLOOOOffset 0x00C42 */

#define BIT_STRHTXBCN_NOBLOCK_NDPIT_C114<d* 2 REG_PATXBFECTRLOOOOffset 0x00C42 */

#define BIT_STRHTXBF0_80MIB(3111<define BIT_GERHTXBF0_40MIB(3110<define BIT_GERHTXBF0_20MIB(319<ddefine BIT_SHIFT_SDRHTXBF0_AIDIfdefine BIT_MASK_BIRHTXBF0_AIDIfx1ff#efine BIT_CPRHTXBF0_AID)                                                       	(((x) & BIT_MASK_EFRHTXBF0_AID<< BIT_SHIFT_WLRHTXBF0_AID<define BIT_GET_BIRHTXBF0_AID)                                                   	(((x) &  BIT_SHIFT_USRHTXBF0_AID<<BIT_MASK_EFRHTXBF0_AID<d* 2 REG_PADARFRCOOOOffset 0x00C43 */

#define BIT_SHIFT_LPDARF_RC8 (56<BICPUHOPTHWIDTH<#efine BIT_CPSK_BIDARF_RC8 fx1fdefine BIT_MADARF_RC8) & (x) & &IT_MASK_BIDARF_RC8<< BIT_SHIFT_WLDARF_RC8<define BIT_GET_BIDARF_RC8) & (x) &  BIT_SHIFT_RDDARF_RC8<<&IT_MASK_BIDARF_RC8<#define BIT_SHIFT_LPDARF_RC7 (48<BICPUHOPTHWIDTH<#efine BIT_CPSK_BIDARF_RC7 fx1fdefine BIT_MADARF_RC7) & (x) & &IT_MASK_BIDARF_RC7<< BIT_SHIFT_WLDARF_RC7<define BIT_GET_BIDARF_RC7) & (x) &  BIT_SHIFT_RDDARF_RC7& &IT_MASK_BIDARF_RC7<#define BIT_SHIFT_LPDARF_RC6 (40<BICPUHOPTHWIDTH<#efine BIT_CPSK_BIDARF_RC6 fx1fdefine BIT_MADARF_RC6) & (x) & &IT_MASK_BIDARF_RC6<< BIT_SHIFT_WLDARF_RC6<define BIT_GET_BIDARF_RC6) & (x) &  BIT_SHIFT_RDDARF_RC6& &IT_MASK_BIDARF_RC6<#define BIT_SHIFT_LPDARF_RC5 (32<BICPUHOPTHWIDTH<#efine BIT_CPSK_BIDARF_RC5 fx1fdefine BIT_MADARF_RC5) & (x) & &IT_MASK_BIDARF_RC5<< BIT_SHIFT_WLDARF_RC5<define BIT_GET_BIDARF_RC5) & (x) &  BIT_SHIFT_RDDARF_RC5& &IT_MASK_BIDARF_RC5<#define BIT_SHIFT_LPDARF_RC4 24define BIT_MASK_CPDARF_RC4 fx1fdefine BIT_MADARF_RC4) & (x) & &IT_MASK_BIDARF_RC4<< BIT_SHIFT_WLDARF_RC4<define BIT_GET_BIDARF_RC4) & (x) &  BIT_SHIFT_RDDARF_RC4& &IT_MASK_BIDARF_RC4<#define BIT_SHIFT_LPDARF_RC3I
#define BIT_MASK_BIDARF_RC3Ifx1fdefine BIT_MADARF_RC3) & (x) & &IT_MASK_BIDARF_RC3<< BIT_SHIFT_WLDARF_RC3<define BIT_GET_BIDARF_RC3) & (x) &  BIT_SHIFT_RDDARF_RC3& &IT_MASK_BIDARF_RC3<#define BIT_SHIFT_LPDARF_RC2I8define BIT_MASK_CPDARF_RC2Ifx1fdefine BIT_MADARF_RC2) & (x) & &IT_MASK_BIDARF_RC2<< BIT_SHIFT_WLDARF_RC2<define BIT_GET_BIDARF_RC2) & (x) &  BIT_SHIFT_RDDARF_RC2& &IT_MASK_BIDARF_RC2<#define BIT_SHIFT_LPDARF_RC1I0define BIT_MASK_CPDARF_RC1I0x1fdefine BIT_MADARF_RC1) & (x) & &IT_MASK_BIDARF_RC1<< BIT_SHIFT_WLDARF_RC1<define BIT_GET_BIDARF_RC1) & (x) &  BIT_SHIFT_RDDARF_RC1& &IT_MASK_BIDARF_RC1<d* 2 REG_PARARFRCOOOOffset 0x00C43 */

#define BIT_CPIFT_USRARF_RC8 (56<BICPUHOPTHWIDTH<#efine BIT_CPSK_BIRARF_RC8 0x1fdefine BIT_MARARF_RC8) & (x) & &IT_MASK_BIRARF_RC8<< BIT_SHIFT_WLRARF_RC8<define BIT_GET_BIRARF_RC8) & (x) &  BIT_SHIFT_USRARF_RC8<<&IT_MASK_BIRARF_RC8<ddefine BIT_CPIFT_USRARF_RC7 (48<BICPUHOPTHWIDTH<#efine BIT_CPSK_BIRARF_RC7 0x1fdefine BIT_MARARF_RC7) & (x) & &IT_MASK_BIRARF_RC7<< BIT_SHIFT_WLRARF_RC7<define BIT_GET_BIRARF_RC7) & (x) &  BIT_SHIFT_USRARF_RC7& &IT_MASK_BIRARF_RC7<ddefine BIT_CPIFT_USRARF_RC6 (40<BICPUHOPTHWIDTH<#efine BIT_CPSK_BIRARF_RC6 0x1fdefine BIT_MARARF_RC6) & (x) & &IT_MASK_BIRARF_RC6<< BIT_SHIFT_WLRARF_RC6<define BIT_GET_BIRARF_RC6) & (x) &  BIT_SHIFT_USRARF_RC6& &IT_MASK_BIRARF_RC6<ddefine BIT_CPIFT_USRARF_RC5 (32<BICPUHOPTHWIDTH<#efine BIT_CPSK_BIRARF_RC5 0x1fdefine BIT_MARARF_RC5) & (x) & &IT_MASK_BIRARF_RC5<< BIT_SHIFT_WLRARF_RC5<define BIT_GET_BIRARF_RC5) & (x) &  BIT_SHIFT_USRARF_RC5& &IT_MASK_BIRARF_RC5<ddefine BIT_CPIFT_USRARF_RC4 24define BIT_MASK_CPRARF_RC4 0x1fdefine BIT_MARARF_RC4) & (x) & &IT_MASK_BIRARF_RC4<< BIT_SHIFT_WLRARF_RC4<define BIT_GET_BIRARF_RC4) & (x) &  BIT_SHIFT_USRARF_RC4& &IT_MASK_BIRARF_RC4<ddefine BIT_CPIFT_USRARF_RC3I
#define BIT_MASK_BIRARF_RC3I0x1fdefine BIT_MARARF_RC3) & (x) & &IT_MASK_BIRARF_RC3<< BIT_SHIFT_WLRARF_RC3<define BIT_GET_BIRARF_RC3) & (x) &  BIT_SHIFT_USRARF_RC3& &IT_MASK_BIRARF_RC3<ddefine BIT_CPIFT_USRARF_RC2I8define BIT_MASK_CPRARF_RC2I0x1fdefine BIT_MARARF_RC2) & (x) & &IT_MASK_BIRARF_RC2<< BIT_SHIFT_WLRARF_RC2<define BIT_GET_BIRARF_RC2) & (x) &  BIT_SHIFT_USRARF_RC2& &IT_MASK_BIRARF_RC2<ddefine BIT_CPIFT_USRARF_RC1I0define BIT_MASK_CPRARF_RC1I0x1fdefine BIT_MARARF_RC1) & (x) & &IT_MASK_BIRARF_RC1<< BIT_SHIFT_WLRARF_RC1<define BIT_GET_BIRARF_RC1) & (x) &  BIT_SHIFT_USRARF_RC1& &IT_MASK_BIRARF_RC1<d* 2 REG_PARRSROOOOffset 0x00C44 */

#define BIT_SHIFT_LPRRSR_RSC 21define BIT_MASK_BIRRSR_RSC fx#define BIT_SERRSR_RSC) & (x) & &IT_MASK_BIRRSR_RSC<< BIT_SHIFT_WLRRSR_RSC<define BIT_GET_BIRRSR_RSC) & (x) &  BIT_SHIFT_USRRSR_RSC<<&IT_MASK_BIRRSR_RSC<
define BIT_SERRSR_BWIT_S120<d*efine BIT_CPIFT_USRRSC_T_CMAPI0define BIT_MASK_CPRRSC_T_CMAPI0ffffffdefine BIT_SERRSC_T_CMAP)                                                       	(((x) & BIT_MASK_EFRRSC_T_CMAP<< BIT_SHIFT_WLRRSC_T_CMAP<define BIT_GET_BIRRSC_T_CMAP)                                                   	(((x) &  BIT_SHIFT_USRRSC_T_CMAP<<BIT_MASK_EFRRSC_T_CMAP<d* 2 REG_PAARFR0OOOOffset 0x00C44 */

#define BIT_SHIFT_SDARFR0HV1I0define BIT_MASK_CPARFR0HV1I0fffffffL
ffffffL
#define BIT_DEARFR0HV1) & (x) & &IT_MASK_BIARFR0HV1<< BIT_SHIFT_WLARFR0HV1<define BIT_GET_BIARFR0HV1) & (x) &  BIT_SHIFT_USARFR0HV1<<&IT_MASK_BIARFR0HV1<d* 2 REG_PAARFR1_V1(OOOffset 0x00C44 */

#define BIT_STIFT_USARFR1_V1I0define BIT_MASK_CPARFR1HV1I0fffffffL
ffffffL
#define BIT_DEARFR1HV1) & (x) & &IT_MASK_BIARFR1_V1<< BIT_SHIFT_WLARFR1_V1<define BIT_GET_BIARFR1HV1) & (x) &  BIT_SHIFT_USARFR1_V1<<&IT_MASK_CPARFR1_V1<d* 2 REG_PACCK_CHECB(OOOffset 0x00C45 */

#define BIT_SHCHECBACCK_ENIT_C17)define BIT_SHEN_BCN_PKUSRELIT_C16<define BIT_GEBCN_PORT_SELIB_C15<#efine BIT_CPMOREDATA_BYPASSIB(314)define BIT_SHEN_CLR_CMD_REL_BCN_PKUIB(313<d* 2 REG_PACCK_CHECB(OOOffset 0x00C45 */

#define BIT_SHRHEN_SECPMOREDATAIT_S12)define BIT_SH_RHDIS_CLEARPSKCID_RELEASEIT_S11)define BIT_SH_RHSKCID_RELEASE_ENIT_C10<d* 2 REG_PAAMPDUHSKX_TIMEOOOffset 0x00C456*/

#define BIT_STIFT_USAMPDUHSKX_TIMEI0define BIT_MASK_CPAMPDUHSKX_TIMEI0xffdefine BIT_SHAMPDUHSKX_TIME)                                                    	(((x) & &IT_MASK_CPAMPDUHSKX_TIME<< BIT_SHIFT_WLAMPDUHSKX_TIME<define BIT_GET_BIAMPDUHSKX_TIME)                                                	(((x) &  BIT_SHIFT_USAMPDUHSKX_TIME<<&IT_MASK_CPAMPDUHSKX_TIME<d* 2 REG_PABCNQ1HBDNY_V1(OOffset 0x00C456*/

#define BIT_STIFT_USBCNQ1HPGBNDYHV1I0define BIT_MASK_CPBCNQ1HPGBNDYHV1I0f
#fdefine BIT_SEBCNQ1HPGBNDYHV1)                                                   	(((x) & &IT_MASK_CPBCNQ1HPGBNDYHV1<< BIT_SHIFT_WLBCNQ1HPGBNDYHV1<define BIT_GET_BIBCNQ1HPGBNDYHV1)                                               	(((x) &  BIT_SHIFT_USBCNQ1HPGBNDYHV1<<&IT_MASK_CPBCNQ1HPGBNDYHV1<d* 2 REG_PAAMPDUHSKX_LENGTH(OOffset 0x00C45 */

#define BIT_CPIFT_USAMPDUHSKX_LENGTHI0define BIT_MASK_CPAMPDUHSKX_LENGTHI0fffffffL
#define BIT_DEAMPDUHSKX_LENGTH)                                                  	(((x) & &IT_MASK_CPAMPDUHSKX_LENGTH<< BIT_SHIFT_WLAMPDUHSKX_LENGTH<define BIT_GET_BIAMPDUHSKX_LENGTH)                                              	(((x) &  BIT_SHIFT_USAMPDUHSKX_LENGTH<<&IT_MASK_CPAMPDUHSKX_LENGTH<d* 2 REG_PAACQ_STOP(OOOffset 0x00C45 */

#define BIT_STAC7QHSTOPIT_C17)define BIT_SHAC6QHSTOPIT_C16<#efine BIT_CPAC5QHSTOPIT_C15<#efine BIT_CPAC4QHSTOPIT_C14<#efine BIT_CPAC3QHSTOPIT_C13<#efine BIT_CPAC2QHSTOPIT_C12<#efine BIT_CPAC1QHSTOPIT_C11<#efine BIT_CPAC0QHSTOPIT_C10<d* 2 REG_PANDPA_RATE(OOOffset 0x00C45D*/

#define BIT_SHIFT_LPRANDPA_RATEHV1I0define BIT_MASK_CPRANDPA_RATEHV1I0xffdefine BIT_SHRANDPA_RATEHV1)                                                    	(((x) & &IT_MASK_CPRANDPA_RATEHV1<< BIT_SHIFT_WLRHNDPA_RATEHV1<define BIT_GET_BIRHNDPA_RATEHV1)                                                	(((x) &  BIT_SHIFT_USRHNDPA_RATEHV1<<&IT_MASK_CPRANDPA_RATEHV1<d* 2 REG_PATX_HANGECTRLOOOffset 0x00C45E*/

#define BIT_STRHEN_GNSHBSHAWAKEIB(313<d* 2 REG_PATX_HANGECTRLOOOffset 0x00C45E*/

#define BIT_STEN_EOFHV1IB(312<#* 2 REG_PATX_HANGECTRLOOOffset 0x00C45E*/

#define BIT_STDIS_OQSHBLOCKIT_C11<#efine BIT_CPSEARCH_QUEUE_ENIT_C10<d* 2 REG_PANDPA_OPTHCTRLOOOffset 0x00C45F*/

#define BIT_STRHDIS_SKCID_RELEASE_RTYIB(315<#* 2 REG_PANDPA_OPTHCTRLOOOffset 0x00C45F*/

#define BIT_STIFT_USBW_SIGTAI3define BIT_MASK_CPBW_SIGTAIfx#define BIT_SEBW_SIGTA) & (x) & &IT_MASK_BIBW_SIGTA<< BIT_SHIFT_WLBW_SIGTA<define BIT_GET_BIBW_SIGTA) & (x) &  BIT_SHIFT_USBW_SIGTA<<&IT_MASK_BIBW_SIGTA<#* 2 REG_PANDPA_OPTHCTRLOOOffset 0x00C45F*/

#define BIT_STEN_BARPSIGTAIB(312<#* 2 REG_PANDPA_OPTHCTRLOOOffset 0x00C45F*/

#define BIT_STIFT_USRANDPA_BWI0define BIT_MASK_CPRANDPA_BWI0x#define BIT_SERANDPA_BW) & (x) & &IT_MASK_BIRANDPA_BW<< BIT_SHIFT_WLRHNDPA_BW<define BIT_GET_BIRHNDPA_BW) & (x) &  BIT_SHIFT_USRHNDPA_BW<<&IT_MASK_BIRANDPA_BW<d* 2 REG_PARD_RESP_PKUSTH(OOffset 0x00C463</

#define BIT_SHIFT_USRD_RESP_PKUSTHHV1I0define BIT_MASK_CPRD_RESP_PKUSTHHV1I0x#fdefine BIT_SHRD_RESP_PKUSTHHV1)                                                 	(((x) & &IT_MASK_CPRD_RESP_PKUSTHHV1<< BIT_SHIFT_WLRD_RESP_PKUSTHHV1<define BIT_GET_BIRD_RESP_PKUSTHHV1)                                             	(((x) &  BIT_SHIFT_USRD_RESP_PKUSTHHV1<<&IT_MASK_CPRD_RESP_PKUSTHHV1<d* 2 REG_PACMDQHINFOOOOOffset 0x00C46 */

#define BIT_SHIFT_SDQUEUESKCID_CMDQHV1I25define BIT_MASK_CPQUEUESKCID_CMDQHV1I0x7f#efine BIT_CPQUEUESKCID_CMDQHV1)                                                	(((x) & &IT_MASK_CPQUEUESKCID_CMDQHV1<< BIT_SHIFT_WLQUEUESKCID_CMDQHV1<define BIT_GET_BIQUEUESKCID_CMDQHV1)                                            	(((x) &  BIT_SHIFT_USQUEUESKCID_CMDQHV1<<&IT_MASK_CPQUEUESKCID_CMDQHV1<d* 2 REG_PACMDQHINFOOOOOffset 0x00C46 */

#define BIT_SHIFT_SDQUEUEAC_CMDQHV1I23define BIT_MASK_CPQUEUEKCICMDQHV1I0x#define BIT_SEQUEUEKCICMDQHV1)                                                   	(((x) & &IT_MASK_CPQUEUEKCICMDQHV1<< BIT_SHIFT_WLQUEUEKCICMDQHV1<define BIT_GET_BIQUEUEKCICMDQHV1)                                               	(((x) &  BIT_SHIFT_USQUEUEKCICMDQHV1<<&IT_MASK_CPQUEUEKCICMDQHV1<d* 2 REG_PACMDQHINFOOOOOffset 0x00C46 */

#define BIT_SHTIDEMPTYICMDQHV1IB(3122<d* 2 REG_PACMDQHINFOOOOOffset 0x00C46 */

#define BIT_SHIFT_SDTAIL_PKTICMDQHV2 11define BIT_MASK_BITAIL_PKTICMDQHV2 0x7ff#efine BIT_CPTAIL_PKTICMDQHV2)                                                  	(((x) & &IT_MASK_CPTAIL_PKTICMDQHV2<< BIT_SHIFT_WLTAIL_PKTICMDQHV2<define BIT_GET_BITAIL_PKTICMDQHV2)                                              	(((x) &  BIT_SHIFT_USTAIL_PKTICMDQHV2<<&IT_MASK_CPTAIL_PKTICMDQHV2<d* 2 REG_PACMDQHINFOOOOOffset 0x00C46 */

#define BIT_SHIFT_SDHEAD_PKTICMDQHV1I0define BIT_MASK_BIHEAD_PKTICMDQHV1I0x7ff#efine BIT_CPHEAD_PKTICMDQHV1)                                                  	(((x) & &IT_MASK_BIHEAD_PKTICMDQHV1<< BIT_SHIFT_WLHEAD_PKTICMDQHV1<define BIT_GET_BIHEAD_PKTICMDQHV1)                                              	(((x) &  BIT_SHIFT_RDHEAD_PKTICMDQHV1<<&IT_MASK_CPHEAD_PKTICMDQHV1<d* 2 REG_PAQ4HINFOOOOOffset 0x00C46 */

#define BIT_CPIFT_USQUEUESKCID_Q4HV1I25define BIT_MASK_CPQUEUESKCID_Q4HV1I0x7f#efine BIT_CPQUEUESKCID_Q4HV1)                                                  	(((x) & &IT_MASK_BIQUEUESKCID_Q4HV1<< BIT_SHIFT_WLQUEUESKCID_Q4HV1<define BIT_GET_BIQUEUESKCID_Q4HV1)                                              	(((x) &  BIT_SHIFT_USQUEUESKCID_Q4HV1<<&IT_MASK_CPQUEUESKCID_Q4HV1<ddefine BIT_SHIFT_SDQUEUEAC_Q4HV1I23define BIT_MASK_CPQUEUEKCIQ4HV1I0x#define BIT_SEQUEUEKCIQ4HV1)                                                     	(((x) & &IT_MASK_BIQUEUEKCIQ4HV1<< BIT_SHIFT_WLQUEUEKCIQ4HV1<define BIT_GET_BIQUEUEKCIQ4HV1)                                                 	(((x) &  BIT_SHIFT_USQUEUEKCIQ4HV1<<&IT_MASK_CPQUEUEKCIQ4HV1<d* 2 REG_PAQ4HINFOOOOOffset 0x00C46 */

#define BIT_CPTIDEMPTYIQ4HV1IB(3122<d* 2 REG_PAQ4HINFOOOOOffset 0x00C46 */

#define BIT_CPIFT_USTAIL_PKTIQ4_V2 11define BIT_MASK_BITAIL_PKTIQ4_V2 0x7ff#efine BIT_CPTAIL_PKTIQ4_V2) &                                                  	(((x) & &IT_MASK_BITAIL_PKTIQ4_V2<< BIT_SHIFT_WLTAIL_PKTIQ4_V2<define BIT_GET_BITAIL_PKTIQ4_V2) &                                              	(((x) &  BIT_SHIFT_USTAIL_PKTIQ4_V2<<&IT_MASK_BITAIL_PKTIQ4_V2<d* 2 REG_PAQ4HINFOOOOOffset 0x00C46 */

#define BIT_CPIFT_USHEAD_PKTIQ4HV1I0define BIT_MASK_BIHEAD_PKTIQ4HV1I0x7ff#efine BIT_CPHEAD_PKTIQ4HV1)                                                    	(((x) & &IT_MASK_BIHEAD_PKTIQ4HV1<< BIT_SHIFT_WLHEAD_PKTIQ4HV1<define BIT_GET_BIHEAD_PKTIQ4HV1)                                                	(((x) &  BIT_SHIFT_RDHEAD_PKTIQ4HV1<<&IT_MASK_CPHEAD_PKTIQ4HV1<d* 2 REG_PAQ5HINFOOOOOffset 0x00C46 */

#define BIT_STIFT_RDQUEUESKCID_Q5HV1I25define BIT_MASK_CPQUEUESKCID_Q5HV1I0x7f#efine BIT_CPQUEUESKCID_Q5HV1)                                                  	(((x) & &IT_MASK_BIQUEUESKCID_Q5HV1<< BIT_SHIFT_WLQUEUESKCID_Q5HV1<define BIT_GET_BIQUEUESKCID_Q5HV1)                                              	(((x) &  BIT_SHIFT_USQUEUESKCID_Q5HV1<<&IT_MASK_CPQUEUESKCID_Q5HV1<ddefine BIT_SHIFT_SDQUEUEAC_Q5HV1I23define BIT_MASK_CPQUEUEKCIQ5HV1I0x#define BIT_SEQUEUEKCIQ5HV1)                                                     	(((x) & &IT_MASK_BIQUEUEKCIQ5HV1<< BIT_SHIFT_WLQUEUEKCIQ5HV1<define BIT_GET_BIQUEUEKCIQ5HV1)                                                 	(((x) &  BIT_SHIFT_USQUEUEKCIQ5HV1<<&IT_MASK_CPQUEUEKCIQ5HV1<d* 2 REG_PAQ5HINFOOOOOffset 0x00C46 */

#define BIT_STTIDEMPTYIQ5HV1IB(3122<d* 2 REG_PAQ5HINFOOOOOffset 0x00C46 */

#define BIT_STIFT_RDTAIL_PKTIQ5_V2 11define BIT_MASK_BITAIL_PKTIQ5_V2 0x7ff#efine BIT_CPTAIL_PKTIQ5_V2) &                                                  	(((x) & &IT_MASK_BITAIL_PKTIQ5_V2<< BIT_SHIFT_WLTAIL_PKTIQ5_V2<define BIT_GET_BITAIL_PKTIQ5_V2) &                                              	(((x) &  BIT_SHIFT_USTAIL_PKTIQ5_V2<<&IT_MASK_BITAIL_PKTIQ5_V2<d* 2 REG_PAQ5HINFOOOOOffset 0x00C46 */

#define BIT_STIFT_RDHEAD_PKTIQ5HV1I0define BIT_MASK_BIHEAD_PKTIQ5HV1I0x7ff#efine BIT_CPHEAD_PKTIQ5HV1)                                                    	(((x) & &IT_MASK_BIHEAD_PKTIQ5HV1<< BIT_SHIFT_WLHEAD_PKTIQ5HV1<define BIT_GET_BIHEAD_PKTIQ5HV1)                                                	(((x) &  BIT_SHIFT_RDHEAD_PKTIQ5HV1<<&IT_MASK_CPHEAD_PKTIQ5HV1<d* 2 REG_PAQ6HINFOOOOOffset 0x00C47 */

#define BIT_SHIFT_LPQUEUESKCID_Q6HV1I25define BIT_MASK_CPQUEUESKCID_Q6HV1I0x7f#efine BIT_CPQUEUESKCID_Q6HV1)                                                  	(((x) & &IT_MASK_BIQUEUESKCID_Q6HV1<< BIT_SHIFT_WLQUEUESKCID_Q6HV1<define BIT_GET_BIQUEUESKCID_Q6HV1)                                              	(((x) &  BIT_SHIFT_USQUEUESKCID_Q6HV1<<&IT_MASK_CPQUEUESKCID_Q6HV1<ddefine BIT_SHIFT_SDQUEUEAC_Q6HV1I23define BIT_MASK_CPQUEUEKCIQ6HV1I0x#define BIT_SEQUEUEKCIQ6HV1)                                                     	(((x) & &IT_MASK_BIQUEUEKCIQ6HV1<< BIT_SHIFT_WLQUEUEKCIQ6HV1<define BIT_GET_BIQUEUEKCIQ6HV1)                                                 	(((x) &  BIT_SHIFT_USQUEUEKCIQ6HV1<<&IT_MASK_CPQUEUEKCIQ6HV1<d* 2 REG_PAQ6HINFOOOOOffset 0x00C47 */

#define BIT_SHTIDEMPTYIQ6HV1IB(3122<d* 2 REG_PAQ6HINFOOOOOffset 0x00C47 */

#define BIT_SHIFT_LPTAIL_PKTIQ6_V2 11define BIT_MASK_BITAIL_PKTIQ6_V2 0x7ff#efine BIT_CPTAIL_PKTIQ6_V2) &                                                  	(((x) & &IT_MASK_BITAIL_PKTIQ6_V2<< BIT_SHIFT_WLTAIL_PKTIQ6_V2<define BIT_GET_BITAIL_PKTIQ6_V2) &                                              	(((x) &  BIT_SHIFT_USTAIL_PKTIQ6_V2<<&IT_MASK_BITAIL_PKTIQ6_V2<d* 2 REG_PAQ6HINFOOOOOffset 0x00C47 */

#define BIT_SHIFT_LPHEAD_PKTIQ6HV1I0define BIT_MASK_BIHEAD_PKTIQ6HV1I0x7ff#efine BIT_CPHEAD_PKTIQ6HV1)                                                    	(((x) & &IT_MASK_BIHEAD_PKTIQ6HV1<< BIT_SHIFT_WLHEAD_PKTIQ6HV1<define BIT_GET_BIHEAD_PKTIQ6HV1)                                                	(((x) &  BIT_SHIFT_RDHEAD_PKTIQ6HV1<<&IT_MASK_CPHEAD_PKTIQ6HV1<d* 2 REG_PAQ7HINFOOOOOffset 0x00C47 */

#define BIT_SHIFT_SDQUEUESKCID_Q7HV1I25define BIT_MASK_CPQUEUESKCID_Q7HV1I0x7f#efine BIT_CPQUEUESKCID_Q7HV1)                                                  	(((x) & &IT_MASK_BIQUEUESKCID_Q7HV1<< BIT_SHIFT_WLQUEUESKCID_Q7HV1<define BIT_GET_BIQUEUESKCID_Q7HV1)                                              	(((x) &  BIT_SHIFT_USQUEUESKCID_Q7HV1<<&IT_MASK_CPQUEUESKCID_Q7HV1<ddefine BIT_SHIFT_SDQUEUEAC_Q7HV1I23define BIT_MASK_CPQUEUEKCIQ7HV1I0x#define BIT_SEQUEUEKCIQ7HV1)                                                     	(((x) & &IT_MASK_BIQUEUEKCIQ7HV1<< BIT_SHIFT_WLQUEUEKCIQ7HV1<define BIT_GET_BIQUEUEKCIQ7HV1)                                                 	(((x) &  BIT_SHIFT_USQUEUEKCIQ7HV1<<&IT_MASK_CPQUEUEKCIQ7HV1<d* 2 REG_PAQ7HINFOOOOOffset 0x00C47 */

#define BIT_SHTIDEMPTYIQ7HV1IB(3122<d* 2 REG_PAQ7HINFOOOOOffset 0x00C47 */

#define BIT_SHIFT_SDTAIL_PKTIQ7_V2 11define BIT_MASK_BITAIL_PKTIQ7_V2 0x7ff#efine BIT_CPTAIL_PKTIQ7_V2) &                                                  	(((x) & &IT_MASK_BITAIL_PKTIQ7_V2<< BIT_SHIFT_WLTAIL_PKTIQ7_V2<define BIT_GET_BITAIL_PKTIQ7_V2) &                                              	(((x) &  BIT_SHIFT_USTAIL_PKTIQ7_V2<<&IT_MASK_BITAIL_PKTIQ7_V2<d* 2 REG_PAQ7HINFOOOOOffset 0x00C47 */

#define BIT_SHIFT_SDHEAD_PKTIQ7HV1I0define BIT_MASK_BIHEAD_PKTIQ7HV1I0x7ff#efine BIT_CPHEAD_PKTIQ7HV1)                                                    	(((x) & &IT_MASK_BIHEAD_PKTIQ7HV1<< BIT_SHIFT_WLHEAD_PKTIQ7HV1<define BIT_GET_BIHEAD_PKTIQ7HV1)                                                	(((x) &  BIT_SHIFT_RDHEAD_PKTIQ7HV1<<&IT_MASK_CPHEAD_PKTIQ7HV1<d* 2 REG_PAWSKC_LBK_BUF_HD_V1(OOffset 0x00C47 */

#define BIT_CPIFT_USWSKC_LBK_BUF_HEAD_V1I0define BIT_MASK_BIWSKC_LBK_BUF_HEAD_V1I0f
#fdefine BIT_SEWSKC_LBK_BUF_HEAD_V1)                                              	(((x) & &IT_MASK_BIWSKC_LBK_BUF_HEAD_V1                                  	((< BIT_SHIFT_WLWSKC_LBK_BUF_HEAD_V1 define BIT_GET_BIWSKC_LBK_BUF_HEAD_V1)                                          	(((x) &  BIT_SHIFT_RDWSKC_LBK_BUF_HEAD_V1  &                             	((<T_MASK_CPWSKC_LBK_BUF_HEAD_V1 d* 2 REG_PAMGQHBDNY_V1(OOOffset 0x00C47A*/

#define BIT_CPIFT_USMGQHPGBNDYHV1I0define BIT_MASK_CPMGQHPGBNDYHV1I0f
#fdefine BIT_SEMGQHPGBNDYHV1)                                                     	(((x) & &IT_MASK_BIMGQHPGBNDYHV1<< BIT_SHIFT_WLMGQHPGBNDYHV1<define BIT_GET_BIMGQHPGBNDYHV1)                                                 	(((x) &  BIT_SHIFT_RDMGQHPGBNDYHV1<<&IT_MASK_BIMGQHPGBNDYHV1<#* 2 REG_PATXRPTHCTRLOOOOffset 0x00C47 */

#define BIT_STIFT_RDTRXRPTHTIMER_TH 24define BIT_MASK_CPTRXRPTHTIMER_TH 0xffdefine BIT_SHTRXRPTHTIMER_TH)                                                   	(((x) & &IT_MASK_CPTRXRPTHTIMER_TH<< BIT_SHIFT_WLTRXRPTHTIMER_TH<define BIT_GET_BITRXRPTHTIMER_TH)                                               	(((x) &  BIT_SHIFT_USTRXRPTHTIMER_TH<<&IT_MASK_CPTRXRPTHTIMER_TH<#* 2 REG_PATXRPTHCTRLOOOOffset 0x00C47 */

#define BIT_STIFT_RDTRXRPTHLEN_TH 
#define BIT_MASK_BITRXRPTHLEN_TH 0xffdefine BIT_SHTRXRPTHLEN_TH)                                                     	(((x) & &IT_MASK_BITRXRPTHLEN_TH<< BIT_SHIFT_WLTRXRPTHLEN_TH<define BIT_GET_BITRXRPTHLEN_TH)                                                 	(((x) &  BIT_SHIFT_USTRXRPTHLEN_TH<<&IT_MASK_BITRXRPTHLEN_TH<#* 2 REG_PATXRPTHCTRLOOOOffset 0x00C47 */

#define BIT_STIFT_RDTRXRPTHREAD_PTRI8define BIT_MASK_CPTRXRPTHREAD_PTRI0xffdefine BIT_SHTRXRPTHREAD_PTR)                                                   	(((x) & &IT_MASK_CPTRXRPTHREAD_PTR<< BIT_SHIFT_WLTRXRPTHREAD_PTR<define BIT_GET_BITRXRPTHREAD_PTR)                                               	(((x) &  BIT_SHIFT_USTRXRPTHREAD_PTR<<&IT_MASK_CPTRXRPTHREAD_PTR<#* 2 REG_PATXRPTHCTRLOOOOffset 0x00C47 */

#define BIT_STIFT_RDTRXRPTHWRITE_PTRI0define BIT_MASK_CPTRXRPTHWRITE_PTRI0xffdefine BIT_SHTRXRPTHWRITE_PTR)                                                  	(((x) & &IT_MASK_CPTRXRPTHWRITE_PTR<< BIT_SHIFT_WLTRXRPTHWRITE_PTR<define BIT_GET_BITRXRPTHWRITE_PTR)                                              	(((x) &  BIT_SHIFT_USTRXRPTHWRITE_PTR<<&IT_MASK_CPTRXRPTHWRITE_PTR<#* 2 REG_PAINIRTS_RATEHSELOOOffset 0x00C48 */

#define BIT_SHLEAG_RTS_BW_DUPIT_C15<#* 2 REG_PABASIC_CFEND_RATE(OOffset 0x00C481*/

#define BIT_STIFT_USBASIC_CFEND_RATEI0define BIT_MASK_CPBASIC_CFEND_RATEI0x1fdefine BIT_MABASIC_CFEND_RATE)                                                  	(((x) & &IT_MASK_CPBASIC_CFEND_RATE<< BIT_SHIFT_WLBASIC_CFEND_RATE<define BIT_GET_BIBASIC_CFEND_RATE)                                              	(((x) &  BIT_SHIFT_USBASIC_CFEND_RATE<<&IT_MASK_CPBASIC_CFEND_RATE<d* 2 REG_PASTBC_CFEND_RATE(OOffset 0x00C482*/

#define BIT_CPIFT_USSTBC_CFEND_RATEIfdefine BIT_MASK_CPSTBC_CFEND_RATEIfx1fdefine BIT_MASTBC_CFEND_RATE)                                                   	(((x) & &IT_MASK_CPSTBC_CFEND_RATE<< BIT_SHIFT_WLSTBC_CFEND_RATE<define BIT_GET_BISTBC_CFEND_RATE)                                               	(((x) &  BIT_SHIFT_USSTBC_CFEND_RATE<<&IT_MASK_CPSTBC_CFEND_RATE<#* 2 REG_PADATA_SCOOOOffset 0x00C483</

#define BIT_SHIFT_USTXSC_40MI4define BIT_MASK_CPTXSC_40MI0xfdefine BIT_SHTXSC_40M) & (x) & &IT_MASK_BITXSC_40M<< BIT_SHIFT_WLTXSC_40M<define BIT_GET_BITXSC_40M) & (x) &  BIT_SHIFT_USTXSC_40M<<&IT_MASK_BITXSC_40M<#define BIT_SHIFT_USTXSC_20MI0define BIT_MASK_CPTXSC_20MI0xfdefine BIT_SHTXSC_20M) & (x) & &IT_MASK_BITXSC_20M<< BIT_SHIFT_WLTXSC_20M<define BIT_GET_BITXSC_20M) & (x) &  BIT_SHIFT_USTXSC_20M<<&IT_MASK_BITXSC_20M<d* 2 REG_PAMKCID_SLEEP3OOOffset 0x00C48 */

#define BIT_SHIFT_SDMKCID127_96_PKTSLEEPI0define BIT_MASK_CPMKCID127_96_PKTSLEEPI0fffffffL
#define BIT_DEMKCID127_96_PKTSLEEP)                                              	(((x) & &IT_MASK_BIMKCID127_96_PKTSLEEP                                  	((< BIT_SHIFT_WLMKCID127_96_PKTSLEEP define BIT_GET_BIMKCID127_96_PKTSLEEP)                                          	(((x) &  BIT_SHIFT_RDMKCID127_96_PKTSLEEP  &                             	((<T_MASK_CPMKCID127_96_PKTSLEEP d* 2 REG_PAMKCID_SLEEP1(OOffset 0x00C48 */

#define BIT_CPIFT_USMKCID63_32_PKTSLEEPI0define BIT_MASK_CPMKCID63_32_PKTSLEEPI0fffffffL
#define BIT_DEMKCID63_32_PKTSLEEP)                                               	(((x) & &IT_MASK_BIMKCID63_32_PKTSLEEP<< BIT_SHIFT_WLMKCID63_32_PKTSLEEP<define BIT_GET_BIMKCID63_32_PKTSLEEP)                                           	(((x) &  BIT_SHIFT_RDMKCID63_32_PKTSLEEP<<&IT_MASK_BIMKCID63_32_PKTSLEEP<d* 2 REG_PAARFR2_V1(OOOffset 0x00C48 */

#define BIT_STIFT_USARFR2HV1I0define BIT_MASK_CPARFR2HV1I0fffffffL
ffffffL
#define BIT_DEARFR2_V1) & (x) & &IT_MASK_BIARFR2HV1<< BIT_SHIFT_WLARFR2HV1<define BIT_GET_BIARFR2_V1) & (x) &  BIT_SHIFT_USARFR2HV1<<&IT_MASK_BIARFR2HV1<d* 2 REG_PAARFR3_V1(OOOffset 0x00C49 */

#define BIT_SHIFT_SDARFR3_V1Ifdefine BIT_MASK_CPARFR3_V1IffffffffL
ffffffL
#define BIT_DEARFR3_V1) & (x) & &IT_MASK_BIARFR3_V1<< BIT_SHIFT_WLARFR3_V1<define BIT_GET_BIARFR3_V1) & (x) &  BIT_SHIFT_USARFR3_V1<<&IT_MASK_CPARFR3_V1<d* 2 REG_PAARFR4(OOOffset 0x00C49 */

#define BIT_STIFT_USARFR4Ifdefine BIT_MASK_CPARFR4IffffffffL
ffffffL
#define BIT_DEARFR4) & (x) & &IT_MASK_BIARFR4<< BIT_SHIFT_WLARFR4<define BIT_GET_BIARFR4) & (x) &  BIT_SHIFT_USARFR4& &IT_MASK_BIARFR4<d* 2 REG_PAARFR5(OOOffset 0x00C4A */

#define BIT_SHIFT_SDARFR5Ifdefine BIT_MASK_CPARFR5IffffffffL
ffffffL
#define BIT_DEARFR5) & (x) & &IT_MASK_BIARFR5<< BIT_SHIFT_WLARFR5<define BIT_GET_BIARFR5) & (x) &  BIT_SHIFT_USARFR5& &IT_MASK_BIARFR5<#* 2 REG_PATXRPTHSTART_OFFSETOOOffset 0x00C4A */

#define BIT_STIFT_USSKCID_MURATEHOFFSET 24define BIT_MASK_CPSKCID_MURATEHOFFSET 0xffdefine BIT_SHSKCID_MURATEHOFFSET)                                               	(((x) & &IT_MASK_BIMKCID_MURATEHOFFSET<< BIT_SHIFT_WLMKCID_MURATEHOFFSET<define BIT_GET_BIMKCID_MURATEHOFFSET)                                           	(((x) &  BIT_SHIFT_RDMKCID_MURATEHOFFSET<<&IT_MASK_BIMKCID_MURATEHOFFSET<#* 2 REG_PATXRPTHSTART_OFFSETOOOffset 0x00C4A */

#define BIT_STRPTFIFO_SIZE_OPTIT_C116<#* 2 REG_PATXRPTHSTART_OFFSETOOOffset 0x00C4A */

#define BIT_STIFT_USSKCID_CTRL_OFFSET 8define BIT_MASK_CPSKCID_CTRL_OFFSET 0xffdefine BIT_SHSKCID_CTRL_OFFSET)                                                 	(((x) & &IT_MASK_BIMKCID_CTRL_OFFSET<< BIT_SHIFT_WLMKCID_CTRL_OFFSET<define BIT_GET_BIMKCID_CTRL_OFFSET)                                             	(((x) &  BIT_SHIFT_RDMKCID_CTRL_OFFSET<<&IT_MASK_BIMKCID_CTRL_OFFSET<#* 2 REG_PATXRPTHSTART_OFFSETOOOffset 0x00C4A */

#define BIT_STIFT_USAMPDUHTXRPTHOFFSET 0define BIT_MASK_CPAMPDUHTXRPTHOFFSET 0xffdefine BIT_SHAMPDUHTXRPTHOFFSET)                                                	(((x) & &IT_MASK_CPAMPDUHTXRPTHOFFSET<< BIT_SHIFT_WLAMPDUHTXRPTHOFFSET<define BIT_GET_BIAMPDUHTXRPTHOFFSET)                                            	(((x) &  BIT_SHIFT_USAMPDUHTXRPTHOFFSET<<&IT_MASK_CPAMPDUHTXRPTHOFFSET<#* 2 REG_PAPOWERHSTAGE1(OOffset 0x00C4B */

#define BIT_SHPTA_WL_PRIASK_CPCPUHMGQIT_C131<define BIT_CPPTA_WL_PRIASK_CPBCNQIT_C130<define BIT_CPPTA_WL_PRIASK_CPHIQIT_C129<#efine BIT_CPPTA_WL_PRIASK_CPMGQIT_C128<#efine BIT_CPPTA_WL_PRIASK_CPBBIT_C127<#efine BIT_CPPTA_WL_PRIASK_CPBEIT_C126<#efine BIT_CPPTA_WL_PRIASK_CPVIIT_C125<#efine BIT_CPPTA_WL_PRIASK_CPVOIT_C124<#* 2 REG_PAPOWERHSTAGE1(OOffset 0x00C4B */

#define BIT_SHIFT_USPOWERHSTAGE1 0define BIT_MASK_CPPOWERHSTAGE1 0fffffff#efine BIT_CPPOWERHSTAGE1)                                                      	(((x) & &IT_MASK_CPPOWERHSTAGE1<< BIT_SHIFT_WLPOWERHSTAGE1<define BIT_GET_BIPOWERHSTAGE1)                                                  	(((x) &  BIT_SHIFT_USPOWERHSTAGE1<<&IT_MASK_CPPOWERHSTAGE1<#* 2 REG_PAPOWERHSTAGE2(OOffset 0x00C4B */

#define BIT_CP_R_CTRL_PKTIPOW_ADJIT_C124<#* 2 REG_PAPOWERHSTAGE2(OOffset 0x00C4B */

#define BIT_CPIFT_USPOWERHSTAGE2 0define BIT_MASK_CPPOWERHSTAGE2 0fffffff#efine BIT_CPPOWERHSTAGE2) &                                                    	(((x) & &IT_MASK_CPPOWERHSTAGE2<< BIT_SHIFT_WLPOWERHSTAGE2<define BIT_GET_BIPOWERHSTAGE2)                                                  	(((x) &  BIT_SHIFT_USPOWERHSTAGE2& &IT_MASK_CPPOWERHSTAGE2<d* 2 REG_PASWPAMPDUHBURSMASODEHCTRLOOffset 0x00C4B */

#define BIT_STIFT_USPAD_NUMHTHRES 24define BIT_MASK_CPPAD_NUMHTHRES 0x#fdefine BIT_SHPAD_NUMHTHRES)                                                     	(((x) & &IT_MASK_BIPAD_NUMHTHRES<< BIT_SHIFT_WLPAD_NUMHTHRES<define BIT_GET_BIPAD_NUMHTHRES)                                                 	(((x) &  BIT_SHIFT_USPAD_NUMHTHRES<<&IT_MASK_BIPAD_NUMHTHRES<d* 2 REG_PASWPAMPDUHBURSMASODEHCTRLOOffset 0x00C4B */

#define BIT_STR_DMAHTHIS_QUEUE_BBIT_C123<#efine BIT_CPR_DMAHTHIS_QUEUE_BEIT_C122<#efine BIT_CPR_DMAHTHIS_QUEUE_VIIT_C121<define BIT_GERHDMAHTHIS_QUEUE_VOIT_C120<d*efine BIT_CPIFT_USR_TOTALHLEN_TH 8define BIT_MASK_CPR_TOTALHLEN_TH 0f
#fdefine BIT_SER_TOTALHLEN_TH)                                                    	(((x) & &IT_MASK_BIR_TOTALHLEN_TH<< BIT_SHIFT_WLRHTOTALHLEN_TH<define BIT_GET_BIR_TOTALHLEN_TH)                                                	(((x) &  BIT_SHIFT_USRHTOTALHLEN_TH<<&IT_MASK_BIR_TOTALHLEN_TH<d* 2 REG_PASWPAMPDUHBURSMASODEHCTRLOOffset 0x00C4B */

#define BIT_STEN_NEW_EARLYIB(317<#* 2 REG_PASWPAMPDUHBURSMASODEHCTRLOOffset 0x00C4B */

#define BIT_STPREATX_CMDIB(316<ddefine BIT_CPIFT_USNUMHSCL_ENI4define BIT_MASK_CPNUMHSCL_ENI0x#define BIT_SENUMHSCL_EN) & (x) & &IT_MASK_BINUMHSCL_EN<< BIT_SHIFT_WLNUMHSCL_EN<define BIT_GET_BINUMHSCL_EN) &                                                  	(((x) &  BIT_SHIFT_USNUMHSCL_EN<<&IT_MASK_BINUMHSCL_EN<#define BIT_STBK_ENIT_C13<#efine BIT_CPBE_ENIT_C12<define BIT_CPVI_ENIT_C11<define BIT_CPVO_ENIT_C10<d* 2 REG_PAPKTILIFE_TIMEOOOffset 0x00C4C */

#define BIT_SHIFT_LPPKTILIFTIMEHBEBBI
#define BIT_MASK_BIPKTILIFTIMEHBEBBI0fffffdefine BIT_STPKTILIFTIMEHBEBB)                                                  	(((x) & &IT_MASK_CPPKTILIFTIMEHBEBB<< BIT_SHIFT_WLPKTILIFTIMEHBEBB<define BIT_GET_BIPKTILIFTIMEHBEBB)                                              	(((x) &  BIT_SHIFT_USPKTILIFTIMEHBEBB<<&IT_MASK_CPPKTILIFTIMEHBEBB<#define BIT_SHIFT_LPPKTILIFTIMEHVOVII0define BIT_MASK_CPPKTILIFTIMEHVOVII0fffffdefine BIT_STPKTILIFTIMEHVOVI)                                                  	(((x) & &IT_MASK_CPPKTILIFTIMEHVOVI<< BIT_SHIFT_WLPKTILIFTIMEHVOVI<define BIT_GET_BIPKTILIFTIMEHVOVI)                                              	(((x) &  BIT_SHIFT_USPKTILIFTIMEHVOVI<<&IT_MASK_CPPKTILIFTIMEHVOVI<d* 2 REG_PASTBC_SETTINGOOOffset 0x00C4C */

#define BIT_SHIFT_USCDEND_TXTIMEHLI4define BIT_MASK_CPCDEND_TXTIMEHLI0xfdefine BIT_SHCDEND_TXTIMEHL)                                                    	(((x) & &IT_MASK_BICDEND_TXTIMEHL<< BIT_SHIFT_WLCDEND_TXTIMEHL<define BIT_GET_BICDEND_TXTIMEHL)                                                	(((x) &  BIT_SHIFT_USCDEND_TXTIMEHL<<&IT_MASK_BICDEND_TXTIMEHL<ddefine BIT_CPIFT_USNESS 2define BIT_MASK_CPNESS 0x#define BIT_SENESS) & (x) & &IT_MASK_BINESS<< BIT_SHIFT_WLNESS<define BIT_GET_BINESS) & (x) &  BIT_SHIFT_USNESS<<&IT_MASK_BINESS<#define BIT_CPIFT_USSTBC_CFENDIfdefine BIT_MASK_CPSTBC_CFEND 0x#define BIT_SESTBC_CFEND) & (x) & &IT_MASK_BISTBC_CFEND<< BIT_SHIFT_WLSTBC_CFEND<define BIT_GET_BISTBC_CFEND) &                                                  	(((x) &  BIT_SHIFT_USSTBC_CFEND<<&IT_MASK_BISTBC_CFEND<d* 2 REG_PASTBC_SETTING2(OOffset 0x00C4C5& 

#define BIT_SHIFT_USCDEND_TXTIMEHHI0define BIT_MASK_CPCDEND_TXTIMEHHI0x1fdefine BIT_MACDEND_TXTIMEHH)                                                    	(((x) & &IT_MASK_BICDEND_TXTIMEHH<< BIT_SHIFT_WLCDEND_TXTIMEHH<define BIT_GET_BICDEND_TXTIMEHH)                                                	(((x) &  BIT_SHIFT_USCDEND_TXTIMEHH<<&IT_MASK_BICDEND_TXTIMEHH<d* 2 REG_PAQUEUE_CTRLOOOOffset 0x00C4C6*/

#define BIT_STPTA_EDCCA_ENIT_C15<#efine BIT_CPPTA_WL_TX_ENIT_C14<d* 2 REG_PAQUEUE_CTRLOOOOffset 0x00C4C6*/

#define BIT_STRPUSREDATA_BWIT_C13<#efine BIT_CPTRIPPKTIINSTSODE1IB(312<#efine BIT_CPTRIPPKTIINSTSODE0IT_C11<define BIT_CPACQ_SODEHSELIB_C10<d* 2 REG_PASINGLEPAMPDUHCTRLOOOffset 0x00C4C7& 

#define BIT_STEN_SINGLEPAPMDUIT_C17)d* 2 REG_PAPROMASODEHCTRLOOOffset 0x00C4C */

#define BIT_CPIFT_USRTS_SKX_AGGINUM 24define BIT_MASK_CPRTS_SKX_AGGINUM 0x#fdefine BIT_SHRTS_SKX_AGGINUM)                                                   	(((x) & &IT_MASK_CPRTS_SKX_AGGINUM<< BIT_SHIFT_WLRTS_SKX_AGGINUM<define BIT_GET_BIRTS_SKX_AGGINUM)                                               	(((x) &  BIT_SHIFT_USRTS_SKX_AGGINUM<<&IT_MASK_CPRTS_SKX_AGGINUM<#define BIT_STIFT_USSKX_AGGINUM 
#define BIT_MASK_BISKX_AGGINUM 0x#fdefine BIT_SHSKX_AGGINUM)                                                       	(((x) & &IT_MASK_CPSKX_AGGINUM<< BIT_SHIFT_WLSKX_AGGINUM<define BIT_GET_BISKX_AGGINUM)                                                   	(((x) &  BIT_SHIFT_RDMKX_AGGINUM<<&IT_MASK_CPSKX_AGGINUM<#define BIT_STIFT_USRTS_TXTIMEHTH 8define BIT_MASK_CPRTS_TXTIMEHTH 0xffdefine BIT_SHRTS_TXTIMEHTH)                                                     	(((x) & &IT_MASK_BIRTS_TXTIMEHTH<< BIT_SHIFT_WLRTS_TXTIMEHTH<define BIT_GET_BIRTS_TXTIMEHTH)                                                 	(((x) &  BIT_SHIFT_USRTS_TXTIMEHTH<<&IT_MASK_BIRTS_TXTIMEHTH<#define BIT_STIFT_USRTS_LEN_TH 0define BIT_MASK_CPRTS_LEN_TH 0xffdefine BIT_SHRTS_LEN_TH) & (x) & &IT_MASK_BIRTS_LEN_TH<< BIT_SHIFT_WLRTS_LEN_TH<define BIT_GET_BIRTS_LEN_TH) &                                                  	(((x) &  BIT_SHIFT_USRTS_LEN_TH<<&IT_MASK_BIRTS_LEN_TH<d* 2 REG_PABARASODEHCTRLOOOffset 0x00C4C */

#define BIT_STIFT_USBARARTY_LMT 
#define BIT_MASK_BIBARARTY_LMT fx#define BIT_SEBARARTY_LMT)                                                       	(((x) & &IT_MASK_CPBARARTY_LMT<< BIT_SHIFT_WLBARARTY_LMT<define BIT_GET_BIBARARTY_LMT)                                                   	(((x) &  BIT_SHIFT_USBARARTY_LMT<<&IT_MASK_CPBARARTY_LMT<#define BIT_STIFT_USBARAPKUSTXTIMEHTH 8define BIT_MASK_CPBARAPKUSTXTIMEHTH 0xffdefine BIT_SHBARAPKUSTXTIMEHTH)                                                 	(((x) & &IT_MASK_BIBARAPKUSTXTIMEHTH<< BIT_SHIFT_WLBARAPKUSTXTIMEHTH<define BIT_GET_BIBARAPKUSTXTIMEHTH)                                             	(((x) &  BIT_SHIFT_USBARAPKUSTXTIMEHTH<<&IT_MASK_BIBARAPKUSTXTIMEHTH<
define BIT_SHBARAEN_V1IB(316<ddefine BIT_CPIFT_USBARAPKUNUMHTH_V1Ifdefine BIT_MASK_CPBARAPKUNUMHTH_V1Ifx#fdefine BIT_SHBARAPKUNUMHTH_V1)                                                  	(((x) & &IT_MASK_CPBARAPKUNUMHTH_V1<< BIT_SHIFT_WLBARAPKUNUMHTH_V1<define BIT_GET_BIBARAPKUNUMHTH_V1)                                              	(((x) &  BIT_SHIFT_USBARAPKUNUMHTH_V1<<&IT_MASK_CPBARAPKUNUMHTH_V1<d* 2 REG_PARAPTRY_RATEHAGGILMTOOOffset 0x00C4CF*/

#define BIT_STIFT_USRAPTRY_RATEHAGGILMT_V1Ifdefine BIT_MASK_CPRAPTRY_RATEHAGGILMT_V1Ifx#fdefine BIT_SHRAPTRY_RATEHAGGILMT_V1)                                            	(((x) & &IT_MASK_CPRAPTRY_RATEHAGGILMT_V1                                	((< BIT_SHIFT_WLRAPTRY_RATEHAGGILMT_V1 define BIT_GET_BIRAPTRY_RATEHAGGILMT_V1)                                        	(((x) &  BIT_SHIFT_USRAPTRY_RATEHAGGILMT_V1  &                           	((<T_MASK_CPRAPTRY_RATEHAGGILMT_V1 d* 2 REG_PAMKCID_SLEEP2(OOffset 0x00C4D */

#define BIT_SHIFT_LPMKCID95_64PKTSLEEPI0define BIT_MASK_CPMKCID95_64PKTSLEEPI0fffffffL
#define BIT_DEMKCID95_64PKTSLEEP)                                                	(((x) & &IT_MASK_CPMKCID95_64PKTSLEEP<< BIT_SHIFT_WLMKCID95_64PKTSLEEP<define BIT_GET_BIMKCID95_64PKTSLEEP)                                            	(((x) &  BIT_SHIFT_RDMKCID95_64PKTSLEEP<<&IT_MASK_CPMKCID95_64PKTSLEEP<d* 2 REG_PAMKCID_SLEEPOOOOffset 0x00C4D */

#define BIT_SHIFT_SDMKCID31_0_PKTSLEEPI0define BIT_MASK_CPMKCID31_0_PKTSLEEPI0fffffffL
#define BIT_DEMKCID31_0_PKTSLEEP)                                                	(((x) & &IT_MASK_CPMKCID31_0_PKTSLEEP<< BIT_SHIFT_WLMKCID31_0_PKTSLEEP<define BIT_GET_BIMKCID31_0_PKTSLEEP)                                            	(((x) &  BIT_SHIFT_RDMKCID31_0_PKTSLEEP<<&IT_MASK_CPMKCID31_0_PKTSLEEP<d* 2 REG_PAHWHSEQ0OOOOffset 0x00C4D */

#define BIT_CPIFT_USHWHSSNHSEQ0I0define BIT_MASK_BIHWHSSNHSEQ0I0f
#fdefine BIT_SEHWHSSNHSEQ0)                                                       	(((x) & &IT_MASK_CPHWHSSNHSEQ0<< BIT_SHIFT_WLHWHSSNHSEQ0<define BIT_GET_BIHWHSSNHSEQ0)                                                   	(((x) &  BIT_SHIFT_RDHWHSSNHSEQ0<<&IT_MASK_CPHWHSSNHSEQ0<d* 2 REG_PAHWHSEQ1(OOOffset 0x00C4DA*/

#define BIT_CPIFT_USHWHSSNHSEQ1I0define BIT_MASK_BIHWHSSNHSEQ1I0f
#fdefine BIT_SEHWHSSNHSEQ1)                                                       	(((x) & &IT_MASK_CPHWHSSNHSEQ1<< BIT_SHIFT_WLHWHSSNHSEQ1<define BIT_GET_BIHWHSSNHSEQ1)                                                   	(((x) &  BIT_SHIFT_RDHWHSSNHSEQ1& &IT_MASK_CPHWHSSNHSEQ1<d* 2 REG_PAHWHSEQ2(OOOffset 0x00C4D */

#define BIT_STIFT_RDHWHSSNHSEQ2 0define BIT_MASK_CPHWHSSNHSEQ2 0f
#fdefine BIT_SEHWHSSNHSEQ2) &                                                     	(((x) & &IT_MASK_CPHWHSSNHSEQ2<< BIT_SHIFT_WLHWHSSNHSEQ2<define BIT_GET_BIHWHSSNHSEQ2) &                                                 	(((x) &  BIT_SHIFT_RDHWHSSNHSEQ2& &IT_MASK_CPHWHSSNHSEQ2<d* 2 REG_PAHWHSEQ3(OOOffset 0x00C4DE*/

#define BIT_STIFT_RDHWHSSNHSEQ3 0define BIT_MASK_CPHWHSSNHSEQ3 0f
#fdefine BIT_SEHWHSSNHSEQ3) &                                                     	(((x) & &IT_MASK_CPHWHSSNHSEQ3<< BIT_SHIFT_WLHWHSSNHSEQ3<define BIT_GET_BIHWHSSNHSEQ3) &                                                 	(((x) &  BIT_SHIFT_RDHWHSSNHSEQ3& &IT_MASK_CPHWHSSNHSEQ3<#* 2 REG_PANULL_PKTISTATUS_V1(OOffset 0x00C4E */

#define BIT_SHIFT_LPPTCL_TOTALHPG_V2 2define BIT_MASK_CPPTCL_TOTALHPG_V2 fx#fffdefine BIT_STPTCL_TOTALHPG_V2)                                                  	(((x) & &IT_MASK_CPPTCL_TOTALHPG_V2<< BIT_SHIFT_WLPTCL_TOTALHPG_V2<define BIT_GET_BIPTCL_TOTALHPG_V2)                                              	(((x) &  BIT_SHIFT_USPTCL_TOTALHPG_V2<<&IT_MASK_CPPTCL_TOTALHPG_V2<#* 2 REG_PANULL_PKTISTATUS(OOffset 0x00C4E */

#define BIT_SHTXANULL_1IT_C11<define BIT_CPTXANULL_0IB_C10<d* 2 REG_PAPTCL_ERRISTATUS(OOffset 0x00C4E2*/

#define BIT_CPPTCL_RATEHTABLE_INVALIDIT_C17)define BIT_CPFTMHT2R_ERRORIB(316<dd 2 REG_PAPTCL_ERRISTATUS(OOffset 0x00C4E2*/

#define BIT_CPPTCL_ERR0IB_C15<#efine BIT_CPPTCL_ERR1IT_C14<#efine BIT_CPPTCL_ERR2IT_C13<#efine BIT_CPPTCL_ERR3IT_C12<#efine BIT_CPPTCL_ERR4IT_C11<define BIT_CPPTCL_ERR5IB_C10<d* 2 REG_PANULL_PKTISTATUS_EXTENDOOffset 0x00C4E3</

#define BIT_SHCLI3HTXANULL_1IT_C17)define BIT_CPCLI3HTXANULL_0IB(316<define BIT_CPCLI2HTXANULL_1IT_C15<define BIT_CPCLI2HTXANULL_0IT_C14<#efine BIT_CPCLI1HTXANULL_1IT_C13<#efine BIT_CPCLI1HTXANULL_0IT_C12<#efine BIT_CPCLI0HTXANULL_1IT_C11<d* 2 REG_PANULL_PKTISTATUS_EXTENDOOffset 0x00C4E3</

#define BIT_SHCLI0PTXANULL_0IB_C10<d* 2 REG_PAVIDEO_ENHANCEMENCPFUNOOffset 0x00C4E */

#define BIT_SHVIDEO_JUST_DROPIT_C11<#efine BIT_CPVIDEO_ENHANCEMENCPFUN_ENIT_C10<d* 2 REG_PABUSPOLLUTE_PKTICNTOOOffset 0x00C4E */

#define BIT_CPIFT_USBUSPOLLUTE_PKTICNTIfdefine BIT_MASK_CPBUSPOLLUTE_PKTICNTIffffffdefine BIT_STBUSPOLLUTE_PKTICNT)                                                	(((x) & &IT_MASK_CPBUSPOLLUTE_PKTICNT<< BIT_SHIFT_WLBUSPOLLUTE_PKTICNT<define BIT_GET_BIBUSPOLLUTE_PKTICNT)                                            	(((x) &  BIT_SHIFT_USBUSPOLLUTE_PKTICNT<<&IT_MASK_CPBUSPOLLUTE_PKTICNT<dd 2 REG_PAPTCL_DBG(OOOffset 0x00C4E */

#define BIT_STIFT_USPTCL_DBGI0define BIT_MASK_CPPTCL_DBGI0fffffffL
#define BIT_DEPTCL_DBG) & (x) & &IT_MASK_BIPTCL_DBG<< BIT_SHIFT_WLPTCL_DBG<define BIT_GET_BIPTCL_DBG) & (x) &  BIT_SHIFT_USPTCL_DBG<<&IT_MASK_BIPTCL_DBG<d* 2 REG_PACPUMGQHTIMER_CTRL2(OOffset 0x00C4F */

#define BIT_SHQUEUE_MKCID_AC_NOUSTHE_SAMEIT_C131<ddefine BIT_SHIFT_SDGTAB_IDI28define BIT_MASK_CPGTAB_IDI0x7define BIT_GETTAB_ID) & (x) & &IT_MASK_BITTAB_ID<< BIT_SHIFT_WLTTAB_ID<define BIT_GET_BITTAB_ID) & (x) &  BIT_SHIFT_USTTAB_ID<<&IT_MASK_BITTAB_ID<#define BIT_STIFT_RDTRI_HEAD_ADDR 
#define BIT_MASK_BITRI_HEAD_ADDR 0f
#fdefine BIT_SETRI_HEAD_ADDR)                                                     	(((x) & &IT_MASK_BITRI_HEAD_ADDR<< BIT_SHIFT_WLTRI_HEAD_ADDR<define BIT_GET_BITRI_HEAD_ADDR)                                                 	(((x) &  BIT_SHIFT_USTRI_HEAD_ADDR<<&IT_MASK_BITRI_HEAD_ADDR<#define BIT_SHQUEUE_MKCID_AC_NOUSTHE_SAME_V1IB(3115<ddefine BIT_SHIFT_SDGTAB_ID_V1I12define BIT_MASK_CPGTAB_ID_V1I0x7define BIT_GETTAB_ID_V1) & (x) & &IT_MASK_BITTAB_ID_V1<< BIT_SHIFT_WLTTAB_ID_V1 define BIT_GET_BITTAB_ID_V1) &                                                  	(((x) &  BIT_SHIFT_USTTAB_ID_V1<<&IT_MASK_CPTTAB_ID_V1 ddefine BIT_GEDROPHTH_ENIT_C18<ddefine BIT_SHIFT_SDDROPHTHI0define BIT_MASK_CPDROPHTHI0xffdefine BIT_SHDROPHTH) & (x) & &IT_MASK_BIDROPHTH<< BIT_SHIFT_WLDROPHTH<define BIT_GET_BIDROPHTH) & (x) &  BIT_SHIFT_USDROPHTH<<&IT_MASK_BIDROPHTH<#* 2 REG_PADUMMY_PAGE4_V1(OOffset 0x00C4F */

#define BIT_STBCNTEN_EXTHWSEQIT_C11<#efine BIT_CPBCNTEN_HWSEQIT_C10<d* 2 REG_PAMOREDATA(OOOffset 0x00C4FE*/

#define BIT_STMOREDATA_CTRL2AEN_V1IB(313<#efine BIT_CPMOREDATA_CTRL1AEN_V1IB(312<#efine BIT_CPPKTINPMOREDATA_REPLACE_ENABLE_V1IB(310<d* 2 REG_PAEDCAPVO_PARAMOOOffset 0x00C50 */

#define BIT_SHIFT_LPTXOPLIMIT 
#define BIT_MASK_BITXOPLIMIT 0x7ff#efine BIT_CPTXOPLIMIT) & (x) & &IT_MASK_BITXOPLIMIT<< BIT_SHIFT_WLTXOPLIMIT<define BIT_GET_BITXOPLIMIT) & (x) &  BIT_SHIFT_USTXOPLIMIT<<&IT_MASK_BITXOPLIMIT<#define BIT_SHIFT_USCW 8define BIT_MASK_CPCWI0xffdefine BIT_SHCW) & (x) & &IT_MASK_BICW<< BIT_SHIFT_WLCW<define BIT_GET_BICW) & (x) &  BIT_SHIFT_USCW& &IT_MASK_BICW<#define BIT_STIFT_USAIFS 0define BIT_MASK_CPAIFS 0xffdefine BIT_SHAIFS) & (x) & &IT_MASK_BIAIFS<< BIT_SHIFT_WLAIFS<define BIT_GET_BIAIFS) & (x) &  BIT_SHIFT_USAIFS<<&IT_MASK_BIAIFS<d* 2 REG_PABCNTCFG(OOOffset 0x00C51 */

#define BIT_SHIFT_LPBCNCWPSKXI12define BIT_MASK_CPBCNCWPSKXI0xfdefine BIT_SHBCNCWPSKX) & (x) & &IT_MASK_BIBCNCWPSKX<< BIT_SHIFT_WLBCNCWPSKX<define BIT_GET_BIBCNCWPSKX) & (x) &  BIT_SHIFT_USBCNCWPSKX<<&IT_MASK_BIBCNCWPSKX<#define BIT_SHIFT_LPBCNCWPSIN 8define BIT_MASK_CPBCNCWPSIN 0xfdefine BIT_SHBCNCWPSIN) & (x) & &IT_MASK_BIBCNCWPSIN<< BIT_SHIFT_WLBCNCWPSIN<define BIT_GET_BIBCNCWPSIN) & (x) &  BIT_SHIFT_USBCNCWPSIN& &IT_MASK_BIBCNCWPSIN<#define BIT_SHIFT_LPBCNIFS 0define BIT_MASK_CPBCNIFS 0xffdefine BIT_SHBCNIFS) & (x) & &IT_MASK_BIBCNIFS<< BIT_SHIFT_WLBCNIFS<define BIT_GET_BIBCNIFS) & (x) &  BIT_SHIFT_USBCNIFS<<&IT_MASK_BIBCNIFS<dd 2 REG_PAPIFS(OOOffset 0x00C512*/

#define BIT_CPIFT_USPIFS 0define BIT_MASK_CPPIFS 0xffdefine BIT_SHPIFS) & (x) & &IT_MASK_BIPIFS<< BIT_SHIFT_WLPIFS<define BIT_GET_BIPIFS) & (x) &  BIT_SHIFT_USPIFS<<&IT_MASK_BIPIFS<dd 2 REG_PARDPAPIFS(OOOffset 0x00C513</

#define BIT_SHIFT_USRDGSPIFS 0define BIT_MASK_CPRDGSPIFS 0xffdefine BIT_SHRDGSPIFS) & (x) & &IT_MASK_BIRDGSPIFS<< BIT_SHIFT_WLRDGLPIFS<define BIT_GET_BIRDGSPIFS) & (x) &  BIT_SHIFT_USRDGSPIFS<<&IT_MASK_BIRDGLPIFS<d* 2 REG_PASIFS(OOOffset 0x00C51 */

#define BIT_SHIFT_SDSIFS_OFDMPTRX 24define BIT_MASK_CPSIFS_OFDMPTRX 0xffdefine BIT_SHSIFS_OFDMPTRX)                                                     	(((x) & &IT_MASK_BISIFS_OFDMPTRX<< BIT_SHIFT_WLSIFS_OFDMPTRX<define BIT_GET_BISIFS_OFDMPTRX)                                                 	(((x) &  BIT_SHIFT_USSIFS_OFDMPTRX<<&IT_MASK_BISIFS_OFDMPTRX<#define BIT_SHIFT_SDSIFS_CCCPTRX 
#define BIT_MASK_BISIFS_CCCPTRX 0xffdefine BIT_SHSIFS_CCCPTRX) &                                                    	(((x) & &IT_MASK_BISIFS_CCCPTRX<< BIT_SHIFT_WLSIFS_CCCPTRX<define BIT_GET_BISIFS_CCCPTRX) &                                                	(((x) &  BIT_SHIFT_USSIFS_CCCPTRX<<&IT_MASK_BISIFS_CCCPTRX<#define BIT_SHIFT_SDSIFS_OFDMPCTX 8define BIT_MASK_CPSIFS_OFDMPCTX 0xffdefine BIT_SHSIFS_OFDMPCTX)                                                     	(((x) & &IT_MASK_BISIFS_OFDMPCTX<< BIT_SHIFT_WLSIFS_OFDMPCTX<define BIT_GET_BISIFS_OFDMPCTX)                                                 	(((x) &  BIT_SHIFT_USSIFS_OFDMPCTX<<&IT_MASK_BISIFS_OFDMPCTX<#define BIT_SHIFT_SDSIFS_CCCPCTX 0define BIT_MASK_BISIFS_CCCPCTX 0xffdefine BIT_SHSIFS_CCCPCTX) &                                                    	(((x) & &IT_MASK_BISIFS_CCCPCTX<< BIT_SHIFT_WLSIFS_CCCPCTX<define BIT_GET_BISIFS_CCCPCTX) &                                                	(((x) &  BIT_SHIFT_USSIFS_CCCPCTX<<&IT_MASK_BISIFS_CCCPCTX<d* 2 REG_PATSFTR_SYN_OFFSETOOOffset 0x00C51 */

#define BIT_CPIFT_USTSFTR_SNCHOFFSET 0define BIT_MASK_CPTSFTR_SNCHOFFSET 0fffffdefine BIT_STTSFTR_SNCHOFFSET)                                                  	(((x) & &IT_MASK_CPTSFTR_SNCHOFFSET<< BIT_SHIFT_WLTSFTR_SNCHOFFSET<define BIT_GET_BITSFTR_SNCHOFFSET)                                              	(((x) &  BIT_SHIFT_USTSFTR_SNCHOFFSET<<&IT_MASK_CPTSFTR_SNCHOFFSET<d* 2 REG_PAAGGR_BREAK_TIMEOOOffset 0x00C51A*/

#define BIT_CPIFT_USAGGR_BK_TIME 0define BIT_MASK_CPAGGR_BK_TIME 0xffdefine BIT_SHAGGR_BK_TIME) &                                                    	(((x) & &IT_MASK_BIAGGR_BK_TIME<< BIT_SHIFT_WLAGGR_BK_TIME<define BIT_GET_BIAGGR_BK_TIME) &                                                	(((x) &  BIT_SHIFT_USAGGR_BK_TIME<<&IT_MASK_BIAGGR_BK_TIME<d* 2 REG_PASLOT(OOOffset 0x00C51B*/

#define BIT_SHIFT_SDSLOT 0define BIT_MASK_BISLOT 0xffdefine BIT_SHSLOT) & (x) & &IT_MASK_BISLOT<< BIT_SHIFT_WLSLOT<define BIT_GET_BISLOT) & (x) &  BIT_SHIFT_USSLOT<<&IT_MASK_BISLOT<#* 2 REG_PATXIPTCL_CTRLOOOffset 0x00C52 */

#define BIT_SHDIS_EDCCAIB(3115<define BIT_CPDIS_CCAIB(3114<#efine BIT_CPLSIGITXOPITXCMD_NAVIB(3113<#efine BIT_CPSIFS_BK_ENIT_C112<#define BIT_SHIFT_USTXQ_NAV_MSKI8define BIT_MASK_CPTXQ_NAV_MSKI0xfdefine BIT_SHTXQ_NAV_MSK) &                                                     	(((x) & &IT_MASK_CPTXQ_NAV_MSK<< BIT_SHIFT_WLTXQ_NAV_MSK<define BIT_GET_BITXQ_NAV_MSK) &                                                 	(((x) &  BIT_SHIFT_USTXQ_NAV_MSK<<&IT_MASK_CPTXQ_NAV_MSK<#define BIT_SHDIS_CWIT_C17)define BIT_CPNAV_END_TXOPIT_C16<define BIT_CPRDGLEND_TXOPIT_C15<define BIT_CPAC_INBCNTHOLDIT_C14<#efine BIT_CPMGTQITXOPIENIT_C13<#efine BIT_CPMGTQIRTSMF_ENIT_C12<define BIT_CPHIQIRTSMF_ENIT_C11<#efine BIT_CPBCNTRTSMF_ENIT_C10<#* 2 REG_PATXPAUSE(OOOffset 0x00C522*/

#define BIT_CPITOPIBCNTHIPMGTIT_C17)define BIT_CPSKC_ITOPBCNQIT_C16)define BIT_CPSKC_ITOPHIQIT_C15)define BIT_CPSKC_ITOPMGQIT_C14<#efine BIT_CPMKC_ITOPBKIT_C13<#efine BIT_CPMKC_ITOPBEIT_C12<define BIT_CPMKC_ITOPVIIT_C11<define BIT_CPMKC_ITOPVOIT_C10<#* 2 REG_PADIS_TXREQ_CLROOOffset 0x00C523</

#define BIT_SHDIS_BT_CCAIB(317<#* 2 REG_PADIS_TXREQ_CLROOOffset 0x00C523</

#define BIT_SHDIS_TXREQ_CLRTHIIT_C15)define BIT_CPDIS_TXREQ_CLRTMGQIT_C14<#efine BIT_CPDIS_TXREQ_CLRTVOIT_C13<#efine BIT_CPDIS_TXREQ_CLRTVIIT_C12<define BIT_CPDIS_TXREQ_CLRTBEIT_C11<define BIT_CPDIS_TXREQ_CLRTBKIT_C10<#* 2 REG_PARD_CTRLOOOOffset 0x00C52 */

#define BIT_SHEN_CLRTTXREQ_INCCAIB(3115<define BIT_CPDIS_TXIOVERPBCNQIT_C114<#* 2 REG_PARD_CTRLOOOOffset 0x00C52 */

#define BIT_SHEN_BCNERRIINCCCAIB(3113<#* 2 REG_PARD_CTRLOOOOffset 0x00C52 */

#define BIT_SHEDCCA_M_BICNTDOWN_ENIT_C111<define BIT_CPDIS_TXOPICFEIT_C110<define BIT_CPDIS_LSIGICFEIT_C19<define BIT_CPDIS_STBC_CFEIT_C18<#efine BIT_CPBKQARD_IN_SHENIT_C17)define BIT_CPBEQARD_IN_SHENIT_C16)define BIT_CPVIQARD_IN_SHENIT_C15<define BIT_CPVOQARD_IN_SHENIT_C14<#efine BIT_CPBKQARD_RESPIENIT_C13<#efine BIT_CPBEQARD_RESPIENIT_C12<define BIT_CPVIQARD_RESPIENIT_C11<define BIT_CPVOQARD_RESPIENIT_C10<d* 2 REG_PAMBSSID_CTRLOOOOffset 0x00C526*/

#define BIT_STMBID_BCNQ7HENIT_C17)define BIT_CPMBID_BCNQ6HENIT_C16)define BIT_CPMBID_BCNQ5HENIT_C15<define BIT_CPMBID_BCNQ4HENIT_C14<#efine BIT_CPMBID_BCNQ3IENIT_C13<#efine BIT_CPMBID_BCNQ2IENIT_C12<define BIT_CPMBID_BCNQ1IENIT_C11<define BIT_CPMBID_BCNQ0_ENIT_C10<d* 2 REG_PAP2PPS_CTRLOOOOffset 0x00C527& 

#define BIT_STP2P_CTW_ALLSTASLEEPIT_C17)define BIT_CPP2P_OFFPDISTX_ENIT_C16<#efine BIT_CPPWRTMGSHENIT_C15<d* 2 REG_PAP2PPS_CTRLOOOOffset 0x00C527& 

#define BIT_STP2P_NOA1IENIT_C12<define BIT_CPP2P_NOA0IENIT_C11<d* 2 REG_PAPKTILIFETIMEHCTRLOOOffset 0x00C52 */

#define BIT_CPEN_P2P_CTWND1IB(3123<d* 2 REG_PAPKTILIFETIMEHCTRLOOOffset 0x00C52 */

#define BIT_CPEN_BKF_CLRTTXREQIB(3122<define BIT_CPEN_TSFT_C32_RSMAP2PIB(3121<define BIT_CPEN_BCNTTX_BTCCAIB(3120<define BIT_CPDIS_PKUSTX_ATIMIT_C119<define BIT_CPDIS_BCNTDIS_CTNIT_C118<define BIT_CPEN_NAVEND_RST_TXOPIT_C117<define BIT_CPEN_FILTER_CCAIB(3116<ddefine BIT_CPIFT_USCCA_FILTER_THRS 8define BIT_MASK_CPCCA_FILTER_THRS 0xffdefine BIT_SHCCA_FILTER_THRS) &                                                 	(((x) & &IT_MASK_CPCCA_FILTER_THRS<< BIT_SHIFT_WLCCA_FILTER_THRS<define BIT_GET_BICCA_FILTER_THRS) &                                             	(((x) &  BIT_SHIFT_USCCA_FILTER_THRS<<&IT_MASK_CPCCA_FILTER_THRS<ddefine BIT_CPIFT_USEDCCA_THRS 0define BIT_MASK_CPEDCCA_THRS 0xffdefine BIT_SHEDCCA_THRS) & (x) & &IT_MASK_BIEDCCA_THRS<< BIT_SHIFT_WLEDCCA_THRS<define BIT_GET_BIEDCCA_THRS) &                                                  	(((x) &  BIT_SHIFT_USEDCCA_THRS<<&IT_MASK_BIEDCCA_THRS<d* 2 REG_PAP2PPS_SPECISTATEOOOffset 0x00C52B*/

#define BIT_SHIPECIPOWERHSTATEIT_C17)define BIT_CPIPECICTWINDOW_ONIT_C16<#efine BIT_CPIPECIBEACON_AREA_ONIT_C15)define BIT_CPIPECICTWIN_EARLYPDISTXIT_C14<#efine BIT_CPIPECINOA1IOFFPPERIODIT_C13<#efine BIT_CPIPECIFORCE_DOZE1IB(312<#efine BIT_CPIPECINOA0IOFFPPERIODIT_C11<#efine BIT_CPIPECIFORCE_DOZE0IB_C10<d* 2 REG_PAQUEUE_INCOL_THROOOffset 0x00C53 */

#define BIT_CPIFT_USBCPQUEUE_THR 24define BIT_MASK_CPBCPQUEUE_THR 0xffdefine BIT_SHBCPQUEUE_THR) &                                                    	(((x) & &IT_MASK_BIBCPQUEUE_THR<< BIT_SHIFT_WLBCPQUEUE_THR<define BIT_GET_BIBCPQUEUE_THR) &                                                	(((x) &  BIT_SHIFT_USBCPQUEUE_THR<<&IT_MASK_BIBCPQUEUE_THR<#define BIT_CPIFT_USBEPQUEUE_THR 
#define BIT_MASK_BIBEPQUEUE_THR 0xffdefine BIT_SHBEPQUEUE_THR) &                                                    	(((x) & &IT_MASK_BIBEPQUEUE_THR<< BIT_SHIFT_WLBEPQUEUE_THR<define BIT_GET_BIBEPQUEUE_THR) &                                                	(((x) &  BIT_SHIFT_USBEPQUEUE_THR<<&IT_MASK_BIBEPQUEUE_THR<#define BIT_CPIFT_USVIPQUEUE_THR 8define BIT_MASK_CPVIPQUEUE_THR 0xffdefine BIT_SHVIPQUEUE_THR) &                                                    	(((x) & &IT_MASK_BIVIPQUEUE_THR<< BIT_SHIFT_WLVIPQUEUE_THR<define BIT_GET_BIVIPQUEUE_THR) &                                                	(((x) &  BIT_SHIFT_USVIPQUEUE_THR<<&IT_MASK_BIVIPQUEUE_THR<#define BIT_CPIFT_USVOPQUEUE_THR 0define BIT_MASK_CPVOPQUEUE_THR 0xffdefine BIT_SHVOPQUEUE_THR) &                                                    	(((x) & &IT_MASK_BIVOPQUEUE_THR<< BIT_SHIFT_WLVOPQUEUE_THR<define BIT_GET_BIVOPQUEUE_THR) &                                                	(((x) &  BIT_SHIFT_USVOPQUEUE_THR<<&IT_MASK_BIVOPQUEUE_THR<d* 2 REG_PAQUEUE_INCOL_ENOOOffset 0x00C53 */

#define BIT_STQUEUE_INCOL_ENIB(3116<dd 2 REG_PAQUEUE_INCOL_ENOOOffset 0x00C53 */

#define BIT_STIFT_USBEPTRIGGERHNUM 
2define BIT_MASK_CPBEPTRIGGERHNUM 0xfdefine BIT_SHBEPTRIGGERHNUM) &                                                  	(((x) & &IT_MASK_CPBEPTRIGGERHNUM<< BIT_SHIFT_WLBEPTRIGGERHNUM<define BIT_GET_BIBEPTRIGGERHNUM) &                                              	(((x) &  BIT_SHIFT_USBEPTRIGGERHNUM<<&IT_MASK_CPBEPTRIGGERHNUM<dd 2 REG_PAQUEUE_INCOL_ENOOOffset 0x00C53 */

#define BIT_STIFT_USBKPTRIGGERHNUM 8define BIT_MASK_CPBKPTRIGGERHNUM 0xfdefine BIT_SHBKPTRIGGERHNUM) &                                                  	(((x) & &IT_MASK_CPBKPTRIGGERHNUM<< BIT_SHIFT_WLBKPTRIGGERHNUM<define BIT_GET_BIBKPTRIGGERHNUM) &                                              	(((x) &  BIT_SHIFT_USBCPTRIGGERHNUM<<&IT_MASK_CPBKPTRIGGERHNUM<dd 2 REG_PAQUEUE_INCOL_ENOOOffset 0x00C53 */

#define BIT_STIFT_USVIPTRIGGERHNUM 4define BIT_MASK_CPVIPTRIGGERHNUM 0xfdefine BIT_SHVIPTRIGGERHNUM) &                                                  	(((x) & &IT_MASK_CPVIPTRIGGERHNUM<< BIT_SHIFT_WLVIPTRIGGERHNUM<define BIT_GET_BIVIPTRIGGERHNUM) &                                              	(((x) &  BIT_SHIFT_USVIPTRIGGERHNUM<<&IT_MASK_CPVIPTRIGGERHNUM<ddefine BIT_CPIFT_USVOPTRIGGERHNUM 0define BIT_MASK_CPVOPTRIGGERHNUM 0xfdefine BIT_SHVOPTRIGGERHNUM) &                                                  	(((x) & &IT_MASK_CPVOPTRIGGERHNUM<< BIT_SHIFT_WLVOPTRIGGERHNUM<define BIT_GET_BIVOPTRIGGERHNUM) &                                              	(((x) &  BIT_SHIFT_USVOPTRIGGERHNUM<<&IT_MASK_CPVOPTRIGGERHNUM<dd 2 REG_PATBTSTPROHIT_MOOOffset 0x00C54 */

#define BIT_SHIFT_LPTBTSTHOLD_TIME_API8define BIT_MASK_CPTBTSTHOLD_TIME_API0f
#fdefine BIT_SETBTSTHOLD_TIME_AP)                                                 	(((x) & &IT_MASK_CPTBTSTHOLD_TIME_AP<< BIT_SHIFT_WLTBTSTHOLD_TIME_AP<define BIT_GET_BITBTSTHOLD_TIME_AP)                                             	(((x) &  BIT_SHIFT_USTBTSTHOLD_TIME_AP<<&IT_MASK_CPTBTSTHOLD_TIME_AP<dd 2 REG_PATBTSTPROHIT_MOOOffset 0x00C54 */

#define BIT_SHIFT_LPTBTSTPROHIT_M_SETUPI0define BIT_MASK_CPTBTSTPROHIT_M_SETUPI0xfdefine BIT_SHTBTSTPROHIT_M_SETUP) &                                             	(((x) & &IT_MASK_CPTBTSTPROHIT_M_SETUP<< BIT_SHIFT_WLTBTSTPROHIT_M_SETUP<define BIT_GET_BITBTSTPROHIT_M_SETUP) &                                         	(((x) &  BIT_SHIFT_USTBTSTPROHIT_M_SETUP<<&IT_MASK_CPTBTSTPROHIT_M_SETUP<d* 2 REG_PAP2PPS_STATEOOOOffset 0x00C543</

#define BIT_SHPOWERHSTATEIT_C17)define BIT_CPCTWINDOW_ONIT_C16<#efine BIT_CPBEACON_AREA_ONIT_C15)define BIT_CPCTWIN_EARLYPDISTXIT_C14<#efine BIT_CPNOA1IOFFPPERIODIT_C13<#efine BIT_CPFORCE_DOZE1IB(312<#efine BIT_CPNOA0IOFFPPERIODIT_C11<#efine BIT_CPFORCE_DOZE0IB_C10<d* 2 REG_PARD_NAV_NXT(OOOffset 0x00C54 */

#define BIT_SHIFT_SDRD_NAV_PROMANXT 0define BIT_MASK_CPRD_NAV_PROMANXT 0fffffdefine BIT_STRD_NAV_PROMANXT) &                                                 	(((x) & &IT_MASK_CPRD_NAV_PROMANXT<< BIT_SHIFT_WLRD_NAV_PROMANXT<define BIT_GET_BIRD_NAV_PROMANXT) &                                             	(((x) &  BIT_SHIFT_USRD_NAV_PROMANXT<<&IT_MASK_CPRD_NAV_PROMANXT<d* 2 REG_PANAV_PROMALENOOOffset 0x00C546*/

#define BIT_STIFT_USNAV_PROMALEN 0define BIT_MASK_CPNAV_PROMALEN 0fffffdefine BIT_STNAV_PROMALEN) &                                                    	(((x) & &IT_MASK_BINAV_PROMALEN<< BIT_SHIFT_WLNAV_PROMALEN<define BIT_GET_BINAV_PROMALEN) &                                                	(((x) &  BIT_SHIFT_USNAV_PROMALEN<<&IT_MASK_BINAV_PROMALEN<d* 2 REG_PABCN_CTRLOOOOffset 0x00C55 */

#define BIT_SHDIS_RX_BSSID_FITIT_C16<#* 2 REG_PABCN_CTRLOOOOffset 0x00C55 */

#define BIT_SHP0PEN_TXBCNTRPTIT_C15<d* 2 REG_PABCN_CTRLOOOOffset 0x00C55 */

#define BIT_SHDIS_TSF_UDTIT_C14<#efine BIT_CPEN_BCNTFUNCTIONIT_C13<#* 2 REG_PABCN_CTRLOOOOffset 0x00C55 */

#define BIT_SHP0PEN_RXBCNTRPTIT_C12<#* 2 REG_PABCN_CTRLOOOOffset 0x00C55 */

#define BIT_SHEN_P2P_CTWINDOWIT_C11<#efine BIT_CPEN_P2P_BCNQ_AREAIB_C10<d* 2 REG_PABCN_CTRLHCLINT0OOOffset 0x00C551</

#define BIT_SHCLI0PDIS_RX_BSSID_FITIT_C16<#* 2 REG_PABCN_CTRLHCLINT0OOOffset 0x00C551</

#define BIT_SHCLI0PDIS_TSF_UDTIT_C14<#* 2 REG_PABCN_CTRLHCLINT0OOOffset 0x00C551</

#define BIT_SHCLI0PEN_BCNTFUNCTIONIT_C13<#* 2 REG_PABCN_CTRLHCLINT0OOOffset 0x00C551</

#define BIT_SHCLI0PEN_RXBCNTRPTIT_C12<#* 2 REG_PABCN_CTRLHCLINT0OOOffset 0x00C551</

#define BIT_SHCLI0PENP2P_CTWINDOWIT_C11<#efine BIT_CPCLI0PENP2P_BCNQ_AREAIB_C10<d* 2 REG_PAMBID_NUMOOOOffset 0x00C552*/

#define BIT_CPEN_PRE_DLPBEACONIT_C13<#*efine BIT_STIFT_USMBID_BCNHNUM 0define BIT_MASK_CPMBID_BCNHNUM 0x7define BIT_GEMBID_BCNHNUM) &                                                    	(((x) & &IT_MASK_BIMBID_BCNHNUM<< BIT_SHIFT_WLMBID_BCNHNUM<define BIT_GET_BIMBID_BCNHNUM) &                                                	(((x) &  BIT_SHIFT_RDMBID_BCNHNUM<<&IT_MASK_BIMBID_BCNHNUM<#* 2 REG_PADUAL_TSF_RSMOOOffset 0x00C553</

#define BIT_SHFREECNT_RSMIT_C15<d* 2 REG_PADUAL_TSF_RSMOOOffset 0x00C553</

#define BIT_SHTSFTR_CLI3HRSMIT_C14<d* 2 REG_PADUAL_TSF_RSMOOOffset 0x00C553</

#define BIT_SHTSFTR_CLI2HRSMIT_C13<d* 2 REG_PADUAL_TSF_RSMOOOffset 0x00C553</

#define BIT_SHTSFTR_CLI1HRSMIT_C12<d* 2 REG_PADUAL_TSF_RSMOOOffset 0x00C553</

#define BIT_SHTSFTR_CLI0HRSMIT_C11<d* 2 REG_PADUAL_TSF_RSMOOOffset 0x00C553</

#define BIT_SHTSFTR_RSMIT_C10<d* 2 REG_PAMBSSID_BCNHSPACEOOOffset 0x00C55 */

#define BIT_SHIFT_SDBCNTTIMER_SEL_FWRDI28define BIT_MASK_CPBCNTTIMER_SEL_FWRDI0x7define BIT_GEBCNTTIMER_SEL_FWRD) &                                              	(((x) & &IT_MASK_CPBCNTTIMER_SEL_FWRD<< BIT_SHIFT_WLBCNTTIMER_SEL_FWRD<define BIT_GET_BIBCNTTIMER_SEL_FWRD) &                                          	(((x) &  BIT_SHIFT_USBCNTTIMER_SEL_FWRD<<&IT_MASK_CPBCNTTIMER_SEL_FWRD<d* 2 REG_PAMBSSID_BCNHSPACEOOOffset 0x00C55 */

#define BIT_SHIFT_SDBCNTSPACEHCLINT0 
#define BIT_MASK_BIBCNTSPACEHCLINT0 0f
#fdefine BIT_SEBCNTSPACEHCLINT0)                                                  	(((x) & &IT_MASK_CPBCNTSPACEHCLINT0<< BIT_SHIFT_WLBCNTSPACEHCLINT0<define BIT_GET_BIBCNTSPACEHCLINT0)                                              	(((x) &  BIT_SHIFT_USBCNTSPACEHCLINT0<<&IT_MASK_CPBCNTSPACEHCLINT0<d* 2 REG_PAMBSSID_BCNHSPACEOOOffset 0x00C55 */

#define BIT_SHIFT_SDBCNTSPACE0I0define BIT_MASK_BIBCNTSPACE0I0fffffdefine BIT_STBCNTSPACE0) & (x) & &IT_MASK_BIBCNTSPACE0<< BIT_SHIFT_WLBCNTSPACE0<define BIT_GET_BIBCNTSPACE0)                                                    	(((x) &  BIT_SHIFT_USBCNTSPACE0<<&IT_MASK_CPBCNTSPACE0<#* 2 REG_PADRVERLYINT(OOOffset 0x00C55 */

#define BIT_CPIFT_USDRVERLYITVI0define BIT_MASK_CPDRVERLYITVI0xffdefine BIT_SHDRVERLYITV) & (x) & &IT_MASK_BIDRVERLYITV<< BIT_SHIFT_WLDRVERLYITV<define BIT_GET_BIDRVERLYITV) &                                                  	(((x) &  BIT_SHIFT_USDRVERLYITV<<&IT_MASK_BIDRVERLYITV<#* 2 REG_PABCNDMATIM(OOOffset 0x00C559*/

#define BIT_SHIFT_SDBCNDMATIMI0define BIT_MASK_BIBCNDMATIMI0xffdefine BIT_SHBCNDMATIM) & (x) & &IT_MASK_BIBCNDMATIM<< BIT_SHIFT_WLBCNDMATIM<define BIT_GET_BIBCNDMATIM) & (x) &  BIT_SHIFT_USBCNDMATIM<<&IT_MASK_BIBCNDMATIM<d* 2 REG_PAATIMWNDOOOOffset 0x00C55A*/

#define BIT_CPIFT_USATIMWND0I0define BIT_MASK_BIATIMWND0I0fffffdefine BIT_STATIMWND0) & (x) & &IT_MASK_BIATIMWND0<< BIT_SHIFT_WLATIMWND0<define BIT_GET_BIATIMWND0) & (x) &  BIT_SHIFT_USATIMWND0<<&IT_MASK_BIATIMWND0<d* 2 REG_PAUSTIMEHTSFOOOOffset 0x00C55 */

#define BIT_STIFT_USUSTIMEHTSF_V1Ifdefine BIT_MASK_CPUSTIMEHTSF_V1Ifxffdefine BIT_SHUSTIMEHTSF_V1)                                                     	(((x) & &IT_MASK_BIUSTIMEHTSF_V1<< BIT_SHIFT_WLUSTIMEHTSF_V1<define BIT_GET_BIUSTIMEHTSF_V1)                                                 	(((x) &  BIT_SHIFT_USUSTIMEHTSF_V1<<&IT_MASK_BIUSTIMEHTSF_V1<#* 2 REG_PABCN_SKX_ERROOOOffset 0x00C55D*/

#define BIT_SHIFT_SDBCNTSKX_ERRI0define BIT_MASK_BIBCNTSKX_ERRI0xffdefine BIT_SHBCNTSKX_ERR) &                                                     	(((x) & &IT_MASK_CPBCNTSKX_ERR<< BIT_SHIFT_WLBCNTSKX_ERR<define BIT_GET_BIBCNTSKX_ERR) &                                                 	(((x) &  BIT_SHIFT_USBCNTSKX_ERR<<&IT_MASK_CPBCNTSKX_ERR<d* 2 REG_PARXTSF_OFFSET_CCCOOOffset 0x00C55E*/

#define BIT_STIFT_RDCCCPRXTSF_OFFSETI0define BIT_MASK_CPCCCPRXTSF_OFFSETI0xffdefine BIT_SHCCCPRXTSF_OFFSET)                                                  	(((x) & &IT_MASK_CPCCCPRXTSF_OFFSET<< BIT_SHIFT_WLCCCPRXTSF_OFFSET<define BIT_GET_BICCCPRXTSF_OFFSET)                                              	(((x) &  BIT_SHIFT_USCCCPRXTSF_OFFSET<<&IT_MASK_CPCCCPRXTSF_OFFSET<d* 2 REG_PARXTSF_OFFSET_OFDMOOOffset 0x00C55F*/

#define BIT_STIFT_USOFDMPRXTSF_OFFSETI0define BIT_MASK_CPOFDMPRXTSF_OFFSETI0xffdefine BIT_SHOFDMPRXTSF_OFFSET)                                                 	(((x) & &IT_MASK_CPOFDMPRXTSF_OFFSET<< BIT_SHIFT_WLOFDMPRXTSF_OFFSET<define BIT_GET_BIOFDMPRXTSF_OFFSET)                                             	(((x) &  BIT_SHIFT_USOFDMPRXTSF_OFFSET<<&IT_MASK_CPOFDMPRXTSF_OFFSET<d* 2 REG_PATSFTROOOOffset 0x00C56 */

#define BIT_SHIFT_LPTSF_TIMER 0define BIT_MASK_CPTSF_TIMER 0fffffffL
ffffffL
#define BIT_DETSF_TIMER) & (x) & &IT_MASK_BITSF_TIMER<< BIT_SHIFT_WLTSF_TIMER<define BIT_GET_BITSF_TIMER) & (x) &  BIT_SHIFT_USTSF_TIMER<<&IT_MASK_BITSF_TIMER<d* 2 REG_PAFREERUNICNTOOOOffset 0x00C56 */

#define BIT_CPIFT_USFREERUNICNT 0define BIT_MASK_CPFREERUNICNT 0fffffffL
ffffffL
#define BIT_DEFREERUNICNT) &                                                     	(((x) & &IT_MASK_CPFREERUNICNT<< BIT_SHIFT_WLFREERUNICNT<define BIT_GET_BIFREERUNICNT) &                                                 	(((x) &  BIT_SHIFT_USFREERUNICNT<<&IT_MASK_CPFREERUNICNT<d* 2 REG_PAATIMWND1_V1(OOOffset 0x00C57 */

#define BIT_SHIFT_LPATIMWND1_V1I0define BIT_MASK_BIATIMWND1_V1Ifxffdefine BIT_SHATIMWND1_V1) &                                                     	(((x) & &IT_MASK_CPATIMWND1_V1<< BIT_SHIFT_WLATIMWND1_V1<define BIT_GET_BIATIMWND1_V1) &                                                 	(((x) &  BIT_SHIFT_USATIMWND1_V1<<&IT_MASK_CPATIMWND1_V1<dd 2 REG_PATBTSTPROHIT_M_INFRAOOOffset 0x00C571</

#define BIT_SHIFT_USTBTSTPROHIT_M_INFRAI0define BIT_MASK_CPTBTSTPROHIT_M_INFRAI0xffdefine BIT_SHTBTSTPROHIT_M_INFRA) &                                             	(((x) & &IT_MASK_CPTBTSTPROHIT_M_INFRA<< BIT_SHIFT_WLTBTSTPROHIT_M_INFRA<define BIT_GET_BITBTSTPROHIT_M_INFRA) &                                         	(((x) &  BIT_SHIFT_USTBTSTPROHIT_M_INFRA<<&IT_MASK_CPTBTSTPROHIT_M_INFRA<d* 2 REG_PACTWNDOOOOffset 0x00C572*/

#define BIT_CPIFT_USCTWNDI0define BIT_MASK_CPCTWNDI0xffdefine BIT_SHCTWND) & (x) & &IT_MASK_BICTWND<< BIT_SHIFT_WLCTWND<define BIT_GET_BICTWND) & (x) &  BIT_SHIFT_USCTWND<<&IT_MASK_BICTWND<#* 2 REG_PABCNIVLCUNTOOOOffset 0x00C573</

#define BIT_SHIFT_USBCNIVLCUNT 0define BIT_MASK_CPBCNIVLCUNT 0x7fdefine BIT_SHBCNIVLCUNT) & (x) & &IT_MASK_BIBCNIVLCUNT<< BIT_SHIFT_WLBCNIVLCUNT<define BIT_GET_BIBCNIVLCUNT) &                                                  	(((x) &  BIT_SHIFT_USBCNIVLCUNT<<&IT_MASK_BIBCNIVLCUNT<#* 2 REG_PABCNDROPCTRLOOOOffset 0x00C57 */

#define BIT_SHBEACON_DROPHENIT_C17)ddefine BIT_SHIFT_USBEACON_DROPHIVL 0define BIT_MASK_CPBEACON_DROPHIVL 0x7fdefine BIT_SHBEACON_DROPHIVL) &                                                 	(((x) & &IT_MASK_CPBEACON_DROPHIVL<< BIT_SHIFT_WLBEACON_DROPHIVL<define BIT_GET_BIBEACON_DROPHIVL) &                                             	(((x) &  BIT_SHIFT_USBEACON_DROPHIVL<<&IT_MASK_CPBEACON_DROPHIVL<d* 2 REG_PAHGQHTIMEOUTPPERIODOOOffset 0x00C575& 

#define BIT_SHIFT_USHGQHTIMEOUTPPERIOD 0define BIT_MASK_CPHGQHTIMEOUTPPERIOD 0xffdefine BIT_SHHGQHTIMEOUTPPERIOD) &                                              	(((x) & &IT_MASK_CPHGQHTIMEOUTPPERIOD<< BIT_SHIFT_WLHGQHTIMEOUTPPERIOD<define BIT_GET_BIHGQHTIMEOUTPPERIOD) &                                          	(((x) &  BIT_SHIFT_RDHGQHTIMEOUTPPERIOD<<&IT_MASK_CPHGQHTIMEOUTPPERIOD<#* 2 REG_PATXCMD_TIMEOUTPPERIODOOffset 0x00C576*/

#define BIT_STIFT_USTXCMD_TIMEOUTPPERIODI0define BIT_MASK_CPTXCMD_TIMEOUTPPERIODI0xffdefine BIT_SHTXCMD_TIMEOUTPPERIOD)                                              	(((x) & &IT_MASK_CPTXCMD_TIMEOUTPPERIOD                                  	((< BIT_SHIFT_WLTXCMD_TIMEOUTPPERIOD define BIT_GET_BITXCMD_TIMEOUTPPERIOD)                                          	(((x) &  BIT_SHIFT_USTXCMD_TIMEOUTPPERIOD  &                             	((<T_MASK_CPTXCMD_TIMEOUTPPERIOD d* 2 REG_PAMISC_CTRLOOOOffset 0x00C577& 

#define BIT_STDIS_TRX_CALABCNIT_C15<define BIT_CPDIS_TXICALATBTSIT_C14<#efine BIT_CPEN_FREECNTIT_C13<#efine BIT_CPBCNIAGGRESSIONIT_C12<#define BIT_SHIFT_USDIS_SECONDARY_CCAI0define BIT_MASK_CPDIS_SECONDARY_CCAI0x#define BIT_SEDIS_SECONDARY_CCA)                                                 	(((x) & &IT_MASK_CPDIS_SECONDARY_CCA<< BIT_SHIFT_WLDIS_SECONDARY_CCA<define BIT_GET_BIDIS_SECONDARY_CCA)                                             	(((x) &  BIT_SHIFT_USDIS_SECONDARY_CCA<<&IT_MASK_CPDIS_SECONDARY_CCA<#* 2 REG_PABCN_CTRLHCLINT1OOOffset 0x00C57 */

#define BIT_CPCLI1HDIS_RX_BSSID_FITIT_C16<#efine BIT_CPCLI1HDIS_TSF_UDTIT_C14<#efine BIT_CPCLI1HEN_BCNTFUNCTIONIT_C13<#* 2 REG_PABCN_CTRLHCLINT1OOOffset 0x00C57 */

#define BIT_CPCLI1HEN_RXBCNTRPTIT_C12<#* 2 REG_PABCN_CTRLHCLINT1OOOffset 0x00C57 */

#define BIT_CPCLI1HENP2P_CTWINDOWIT_C11<#efine BIT_CPCLI1PENP2P_BCNQ_AREAIB_C10<d* 2 REG_PABCN_CTRLHCLINT2OOOffset 0x00C579*/

#define BIT_SHCLI2HDIS_RX_BSSID_FITIT_C16<#efine BIT_CPCLI2HDIS_TSF_UDTIT_C14<#efine BIT_CPCLI2HEN_BCNTFUNCTIONIT_C13<#* 2 REG_PABCN_CTRLHCLINT2OOOffset 0x00C579*/

#define BIT_SHCLI2HEN_RXBCNTRPTIT_C12<#* 2 REG_PABCN_CTRLHCLINT2OOOffset 0x00C579*/

#define BIT_SHCLI2HENP2P_CTWINDOWIT_C11<#efine BIT_CPCLI2PENP2P_BCNQ_AREAIB_C10<d* 2 REG_PABCN_CTRLHCLINT3OOOffset 0x00C57A*/

#define BIT_CPCLI3HDIS_RX_BSSID_FITIT_C16<#efine BIT_CPCLI3HDIS_TSF_UDTIT_C14<#efine BIT_CPCLI3HEN_BCNTFUNCTIONIT_C13<#* 2 REG_PABCN_CTRLHCLINT3OOOffset 0x00C57A*/

#define BIT_CPCLI3HEN_RXBCNTRPTIT_C12<#* 2 REG_PABCN_CTRLHCLINT3OOOffset 0x00C57A*/

#define BIT_CPCLI3HENP2P_CTWINDOWIT_C11<#efine BIT_CPCLI3PENP2P_BCNQ_AREAIB_C10<d* 2 REG_PAEXTEND_CTRLOOOOffset 0x00C57B*/

#define BIT_SHEN_TSFT_C32_RSMAP2P2IT_C15<define BIT_CPEN_TSFT_C32_RSMAP2P1IT_C14<#define BIT_SHIFT_USPORM_SEL 0define BIT_MASK_CPPORM_SEL 0x7define BIT_GEPORM_SEL) & (x) & &IT_MASK_BIPORM_SEL<< BIT_SHIFT_WLPORM_SEL<define BIT_GET_BIPORM_SEL) & (x) &  BIT_SHIFT_USPORM_SEL<<&IT_MASK_BIPORM_SEL<d* 2 REG_PAP2PPS1_SPECISTATEOOOffset 0x00C57 */

#define BIT_STP2P1HIPECIPOWERHSTATEIT_C17)define BIT_CPP2P1HIPECICTWINDOW_ONIT_C16<#efine BIT_CPP2P1HIPECIBCNIAREA_ONIT_C15)define BIT_CPP2P1HIPECICTWIN_EARLYPDISTXIT_C14<#efine BIT_CPP2P1HIPECINOA1IOFFPPERIODIT_C13<#efine BIT_CPP2P1HIPECIFORCE_DOZE1IB(312<#efine BIT_CPP2P1HIPECINOA0IOFFPPERIODIT_C11<#efine BIT_CPP2P1HIPECIFORCE_DOZE0IB_C10<d* 2 REG_PAP2PPS1_STATEOOOffset 0x00C57D*/

#define BIT_SHP2P1HPOWERHSTATEIT_C17)define BIT_CPP2P1HCTWINDOW_ONIT_C16<#efine BIT_CPP2P1HBEACON_AREA_ONIT_C15)define BIT_CPP2P1HCTWIN_EARLYPDISTXIT_C14<#efine BIT_CPP2P1HNOA1IOFFPPERIODIT_C13<#efine BIT_CPP2P1HFORCE_DOZE1IB(312<#efine BIT_CPP2P1HNOA0IOFFPPERIODIT_C11<#efine BIT_CPP2P1HFORCE_DOZE0IB_C10<d* 2 REG_PAP2PPS2_SPECISTATEOOOffset 0x00C57E*/

#define BIT_STP2P2HIPECIPOWERHSTATEIT_C17)define BIT_CPP2P2HIPECICTWINDOW_ONIT_C16<#efine BIT_CPP2P2HIPECIBCNIAREA_ONIT_C15)define BIT_CPP2P2HIPECICTWIN_EARLYPDISTXIT_C14<#efine BIT_CPP2P2HIPECINOA1IOFFPPERIODIT_C13<#efine BIT_CPP2P2HIPECIFORCE_DOZE1IB(312<#efine BIT_CPP2P2HIPECINOA0IOFFPPERIODIT_C11<#efine BIT_CPP2P2HIPECIFORCE_DOZE0IB_C10<d* 2 REG_PAP2PPS2_STATEOOOffset 0x00C57F*/

#define BIT_STP2P2HPOWERHSTATEIT_C17)define BIT_CPP2P2HCTWINDOW_ONIT_C16<#efine BIT_CPP2P2HBEACON_AREA_ONIT_C15)define BIT_CPP2P2ICTWIN_EARLYPDISTXIT_C14<#efine BIT_CPP2P2HNOA1IOFFPPERIODIT_C13<#efine BIT_CPP2P2HFORCE_DOZE1IB(312<#efine BIT_CPP2P2HNOA0IOFFPPERIODIT_C11<#efine BIT_CPP2P2HFORCE_DOZE0IB_C10<d* 2 REG_PAPS_TIMER0OOOOffset 0x00C58 */

#define BIT_SHIFT_LPPSTIMER0_INT 5define BIT_MASK_CPPSTIMER0_INT 0x7ffffffdefine BIT_STPSTIMER0_INT) &                                                    	(((x) & &IT_MASK_BIPSTIMER0_INT<< BIT_SHIFT_WLPSTIMER0_INT<define BIT_GET_BIPSTIMER0_INT) &                                                	(((x) &  BIT_SHIFT_USPSTIMER0_INT<<&IT_MASK_BIPSTIMER0_INT<d* 2 REG_PAPS_TIMER1(OOOffset 0x00C58 */

#define BIT_SHIFT_SDPSTIMER1_INT 5define BIT_MASK_CPPSTIMER1_INT 0x7ffffffdefine BIT_STPSTIMER1_INT) &                                                    	(((x) & &IT_MASK_BIPSTIMER1_INT<< BIT_SHIFT_WLPSTIMER1_INT<define BIT_GET_BIPSTIMER1_INT) &                                                	(((x) &  BIT_SHIFT_USPSTIMER1_INT<<&IT_MASK_BIPSTIMER1_INT<d* 2 REG_PAPS_TIMER2(OOOffset 0x00C58 */

#define BIT_CPIFT_USPSTIMER2_INT 5define BIT_MASK_CPPSTIMER2_INT 0x7ffffffdefine BIT_STPSTIMER2_INT) &                                                    	(((x) & &IT_MASK_BIPSTIMER2_INT<< BIT_SHIFT_WLPSTIMER2_INT<define BIT_GET_BIPSTIMER2_INT) &                                                	(((x) &  BIT_SHIFT_USPSTIMER2_INT<<&IT_MASK_BIPSTIMER2_INT<d* 2 REG_PATBTSTCTN_AREAOOOffset 0x00C58 */

#define BIT_STIFT_USTBTSTCTN_AREAI0define BIT_MASK_CPTBTSTCTN_AREAI0xffdefine BIT_SHTBTSTCTN_AREA)                                                     	(((x) & &IT_MASK_BITBTSTCTN_AREA<< BIT_SHIFT_WLTBTSTCTN_AREA<define BIT_GET_BITBTSTCTN_AREA)                                                 	(((x) &  BIT_SHIFT_USTBTSTCTN_AREA<<&IT_MASK_BITBTSTCTN_AREA<d* 2 REG_PAFORCE_BCNIIFS(OOffset 0x00C58E*/

#define BIT_STIFT_RDFORCE_BCNIIFS 0define BIT_MASK_CPFORCE_BCNIIFS 0xffdefine BIT_SHFORCE_BCNIIFS)                                                     	(((x) & &IT_MASK_BIFORCE_BCNIIFS<< BIT_SHIFT_WLFORCE_BCNIIFS<define BIT_GET_BIFORCE_BCNIIFS)                                                 	(((x) &  BIT_SHIFT_USFORCE_BCNIIFS<<&IT_MASK_BIFORCE_BCNIIFS<#* 2 REG_PATXOPPSIN(OOOffset 0x00C59 */

#define BIT_SHIFT_LPTXOPPSIN 0define BIT_MASK_CPTXOPPSIN 0x#fffdefine BIT_STTXOPPSIN) & (x) & &IT_MASK_BITXOPPSIN<< BIT_SHIFT_WLTXOPPSIN<define BIT_GET_BITXOPPSIN) & (x) &  BIT_SHIFT_USTXOPPSIN& &IT_MASK_BITXOPPSIN<d* 2 REG_PAPRE_BKF_TIMEOOOffset 0x00C592*/

#define BIT_CPIFT_USPRE_BKF_TIME 0define BIT_MASK_CPPRE_BKF_TIME 0xffdefine BIT_SHPRE_BKF_TIME) &                                                    	(((x) & &IT_MASK_BIPRE_BKF_TIME<< BIT_SHIFT_WLPRE_BKF_TIME<define BIT_GET_BIPRE_BKF_TIME) &                                                	(((x) &  BIT_SHIFT_USPRE_BKF_TIME<<&IT_MASK_BIPRE_BKF_TIME<d* 2 REG_PACROSS_TXOPICTRLOOOffset 0x00C593</

#define BIT_SHDTIM_BYPASSIB(312<#efine BIT_CPRTSINAV_TXOPIT_C11<#efine BIT_CPNOTACROSS_TXOPIB_C10<d* 2 REG_PAATIMWND2(OOOffset 0x00C5A */

#define BIT_SHIFT_LPATIMWND2I0define BIT_MASK_BIATIMWND2Ifxffdefine BIT_SHATIMWND2) & (x) & &IT_MASK_BIATIMWND2<< BIT_SHIFT_WLATIMWND2<define BIT_GET_BIATIMWND2) & (x) &  BIT_SHIFT_USATIMWND2<<&IT_MASK_BIATIMWND2<d* 2 REG_PAATIMWND3(OOOffset 0x00C5A1</

#define BIT_SHIFT_USATIMWND3I0define BIT_MASK_BIATIMWND3Ifxffdefine BIT_SHATIMWND3) & (x) & &IT_MASK_BIATIMWND3<< BIT_SHIFT_WLATIMWND3<define BIT_GET_BIATIMWND3) & (x) &  BIT_SHIFT_USATIMWND3& &IT_MASK_BIATIMWND3<d* 2 REG_PAATIMWND4(OOOffset 0x00C5A2*/

#define BIT_CPIFT_USATIMWND4I0define BIT_MASK_BIATIMWND4Ifxffdefine BIT_SHATIMWND4) & (x) & &IT_MASK_BIATIMWND4<< BIT_SHIFT_WLATIMWND4<define BIT_GET_BIATIMWND4) & (x) &  BIT_SHIFT_USATIMWND4& &IT_MASK_BIATIMWND4<d* 2 REG_PAATIMWND5(OOOffset 0x00C5A3</

#define BIT_SHIFT_USATIMWND5I0define BIT_MASK_BIATIMWND5Ifxffdefine BIT_SHATIMWND5) & (x) & &IT_MASK_BIATIMWND5<< BIT_SHIFT_WLATIMWND5<define BIT_GET_BIATIMWND5) & (x) &  BIT_SHIFT_USATIMWND5& &IT_MASK_BIATIMWND5<d* 2 REG_PAATIMWND6(OOOffset 0x00C5A */

#define BIT_SHIFT_SDATIMWND6I0define BIT_MASK_BIATIMWND6Ifxffdefine BIT_SHATIMWND6) & (x) & &IT_MASK_BIATIMWND6<< BIT_SHIFT_WLATIMWND6<define BIT_GET_BIATIMWND6) & (x) &  BIT_SHIFT_USATIMWND6& &IT_MASK_BIATIMWND6<d* 2 REG_PAATIMWND7(OOOffset 0x00C5A5& 

#define BIT_SHIFT_USATIMWND7I0define BIT_MASK_BIATIMWND7Ifxffdefine BIT_SHATIMWND7) & (x) & &IT_MASK_BIATIMWND7<< BIT_SHIFT_WLATIMWND7<define BIT_GET_BIATIMWND7) & (x) &  BIT_SHIFT_USATIMWND7& &IT_MASK_BIATIMWND7<d* 2 REG_PAATIMUGTOOOOffset 0x00C5A6*/

#define BIT_STIFT_USATIM_URGENT 0define BIT_MASK_CPATIM_URGENT 0xffdefine BIT_SHATIM_URGENT) &                                                     	(((x) & &IT_MASK_CPATIM_URGENT<< BIT_SHIFT_WLATIM_URGENT<define BIT_GET_BIATIM_URGENT) &                                                 	(((x) &  BIT_SHIFT_USATIM_URGENT<<&IT_MASK_CPATIM_URGENT<d* 2 REG_PAHIQPNO_LMT_ENOOOffset 0x00C5A7& 

#define BIT_STHIQPNO_LMT_EN_VAP7IB_C17<define BIT_GEHIQPNO_LMT_EN_VAP6IB_C16<define BIT_GEHIQPNO_LMT_EN_VAP5IB_C15<define BIT_GEHIQPNO_LMT_EN_VAP4IB_C14<define BIT_GEHIQPNO_LMT_EN_VAP3IB_C13<define BIT_GEHIQPNO_LMT_EN_VAP2IB(312<#efine BIT_CPHIQPNO_LMT_EN_VAP1IT_C11<#efine BIT_CPHIQPNO_LMT_EN_ROOMIT_C10<d* 2 REG_PADTIM_COUNTER_ROOMOOOffset 0x00C5A */

#define BIT_CPIFT_USDTIM_COUNT_ROOMI0define BIT_MASK_CPDTIM_COUNT_ROOMI0xffdefine BIT_SHDTIM_COUNT_ROOM) &                                                 	(((x) & &IT_MASK_CPDTIM_COUNT_ROOM<< BIT_SHIFT_WLDTIM_COUNT_ROOM<define BIT_GET_BIDTIM_COUNT_ROOM) &                                             	(((x) &  BIT_SHIFT_USDTIM_COUNT_ROOM<<&IT_MASK_CPDTIM_COUNT_ROOM<d* 2 REG_PADTIM_COUNTER_VAP1OOOffset 0x00C5A9*/

#define BIT_SHIFT_SDDTIM_COUNT_VAP1I0define BIT_MASK_CPDTIM_COUNT_VAP1I0xffdefine BIT_SHDTIM_COUNT_VAP1) &                                                 	(((x) & &IT_MASK_CPDTIM_COUNT_VAP1<< BIT_SHIFT_WLDTIM_COUNT_VAP1<define BIT_GET_BIDTIM_COUNT_VAP1) &                                             	(((x) &  BIT_SHIFT_USDTIM_COUNT_VAP1<<&IT_MASK_CPDTIM_COUNT_VAP1<d* 2 REG_PADTIM_COUNTER_VAP2OOOffset 0x00C5AA*/

#define BIT_CPIFT_USDTIM_COUNT_VAP2I0define BIT_MASK_BIDTIM_COUNT_VAP2I0xffdefine BIT_SHDTIM_COUNT_VAP2) &                                                 	(((x) & &IT_MASK_CPDTIM_COUNT_VAP2<< BIT_SHIFT_WLDTIM_COUNT_VAP2<define BIT_GET_BIDTIM_COUNT_VAP2) &                                             	(((x) &  BIT_SHIFT_USDTIM_COUNT_VAP2& &IT_MASK_CPDTIM_COUNT_VAP2<d* 2 REG_PADTIM_COUNTER_VAP3OOOffset 0x00C5AB*/

#define BIT_SHIFT_SDDTIM_COUNT_VAP3I0define BIT_MASK_BIDTIM_COUNT_VAP3I0xffdefine BIT_SHDTIM_COUNT_VAP3) &                                                 	(((x) & &IT_MASK_CPDTIM_COUNT_VAP3<< BIT_SHIFT_WLDTIM_COUNT_VAP3<define BIT_GET_BIDTIM_COUNT_VAP3) &                                             	(((x) &  BIT_SHIFT_USDTIM_COUNT_VAP3& &IT_MASK_CPDTIM_COUNT_VAP3<d* 2 REG_PADTIM_COUNTER_VAP4OOOffset 0x00C5A */

#define BIT_STIFT_USDTIM_COUNT_VAP4I0define BIT_MASK_BIDTIM_COUNT_VAP4I0xffdefine BIT_SHDTIM_COUNT_VAP4) &                                                 	(((x) & &IT_MASK_CPDTIM_COUNT_VAP4<< BIT_SHIFT_WLDTIM_COUNT_VAP4<define BIT_GET_BIDTIM_COUNT_VAP4) &                                             	(((x) &  BIT_SHIFT_USDTIM_COUNT_VAP4& &IT_MASK_CPDTIM_COUNT_VAP4<d* 2 REG_PADTIM_COUNTER_VAP5OOOffset 0x00C5AD*/

#define BIT_SHIFT_SDDTIM_COUNT_VAP5I0define BIT_MASK_BIDTIM_COUNT_VAP5I0xffdefine BIT_SHDTIM_COUNT_VAP5) &                                                 	(((x) & &IT_MASK_CPDTIM_COUNT_VAP5<< BIT_SHIFT_WLDTIM_COUNT_VAP5<define BIT_GET_BIDTIM_COUNT_VAP5) &                                             	(((x) &  BIT_SHIFT_USDTIM_COUNT_VAP5& &IT_MASK_CPDTIM_COUNT_VAP5<d* 2 REG_PADTIM_COUNTER_VAP6OOOffset 0x00C5AE*/

#define BIT_STIFT_RDDTIM_COUNT_VAP6I0define BIT_MASK_BIDTIM_COUNT_VAP6I0xffdefine BIT_SHDTIM_COUNT_VAP6) &                                                 	(((x) & &IT_MASK_CPDTIM_COUNT_VAP6<< BIT_SHIFT_WLDTIM_COUNT_VAP6<define BIT_GET_BIDTIM_COUNT_VAP6) &                                             	(((x) &  BIT_SHIFT_USDTIM_COUNT_VAP6& &IT_MASK_CPDTIM_COUNT_VAP6<d* 2 REG_PADTIM_COUNTER_VAP7OOOffset 0x00C5AF*/

#define BIT_STIFT_USDTIM_COUNT_VAP7I0define BIT_MASK_BIDTIM_COUNT_VAP7I0xffdefine BIT_SHDTIM_COUNT_VAP7) &                                                 	(((x) & &IT_MASK_CPDTIM_COUNT_VAP7<< BIT_SHIFT_WLDTIM_COUNT_VAP7<define BIT_GET_BIDTIM_COUNT_VAP7) &                                             	(((x) &  BIT_SHIFT_USDTIM_COUNT_VAP7& &IT_MASK_CPDTIM_COUNT_VAP7<#* 2 REG_PADIS_ATIM(OOOffset 0x00C5B */

#define BIT_SHDIS_ATIM_VAP7IB_C17<define BIT_GEDIS_ATIM_VAP6IB_C16<define BIT_GEDIS_ATIM_VAP5IB_C15<define BIT_GEDIS_ATIM_VAP4IB_C14<define BIT_GEDIS_ATIM_VAP3IB_C13<define BIT_GEDIS_ATIM_VAP2IB(312<#efine BIT_CPDIS_ATIM_VAP1IT_C11<#efine BIT_CPDIS_ATIM_ROOMIT_C10<d* 2 REG_PAEARLYP128US(OOOffset 0x00C5B1</

#define BIT_SHIFT_USTSFM_SEL_TIMER1 3define BIT_MASK_BITSFM_SEL_TIMER1 0x7define BIT_GETSFM_SEL_TIMER1) &                                                 	(((x) & &IT_MASK_CPTSFM_SEL_TIMER1<< BIT_SHIFT_WLTSFT_SEL_TIMER1<define BIT_GET_BITSFM_SEL_TIMER1) &                                             	(((x) &  BIT_SHIFT_USTSFT_SEL_TIMER1<<&IT_MASK_CPTSFM_SEL_TIMER1<ddefine BIT_CPIFT_USEARLYP128US 0define BIT_MASK_CPEARLYP128US 0x7define BIT_GEEARLYP128US) &                                                     	(((x) & &IT_MASK_CPEARLYP128US<< BIT_SHIFT_WLEARLYP128US<define BIT_GET_BIEARLYP128US) &                                                 	(((x) &  BIT_SHIFT_USEARLYP128US<<&IT_MASK_CPEARLYP128US<d* 2 REG_PAP2PPS1_CTRLOOOOffset 0x00C5B2*/

#define BIT_CPP2P1HCTW_ALLSTASLEEPIT_C17)define BIT_CPP2P1_OFFPDISTX_ENIT_C16<#efine BIT_CPP2P1HPWRTMGSHENIT_C15<define BIT_CPP2P1HNOA1IENIT_C12<define BIT_CPP2P1_NOA0IENIT_C11<d* 2 REG_PAP2PPS2_CTRLOOOOffset 0x00C5B3</

#define BIT_SHP2P2ICTW_ALLSTASLEEPIT_C17)define BIT_CPP2P2_OFFPDISTX_ENIT_C16<#efine BIT_CPP2P2HPWRTMGSHENIT_C15<define BIT_CPP2P2HNOA1IENIT_C12<define BIT_CPP2P2_NOA0IENIT_C11<d* 2 REG_PATIMER0_SRC_SELOOOffset 0x00C5B */

#define BIT_SHIFT_SDSYNCPCLI_SEL 4define BIT_MASK_CPSYNCPCLI_SEL 0x7define BIT_GESYNCPCLI_SEL) &                                                    	(((x) & &IT_MASK_BISYNCPCLI_SEL<< BIT_SHIFT_WLSYNCPCLI_SEL<define BIT_GET_BISYNCPCLI_SEL) &                                                	(((x) &  BIT_SHIFT_USSYNCPCLI_SEL<<&IT_MASK_BISYNCPCLI_SEL<#define BIT_SHIFT_USTSFM_SEL_TIMER0I0define BIT_MASK_BITSFM_SEL_TIMER0I0x7define BIT_GETSFM_SEL_TIMER0)                                                   	(((x) & &IT_MASK_CPTSFM_SEL_TIMER0<< BIT_SHIFT_WLTSFM_SEL_TIMER0<define BIT_GET_BITSFM_SEL_TIMER0)                                               	(((x) &  BIT_SHIFT_USTSFT_SEL_TIMER0& &IT_MASK_CPTSFM_SEL_TIMER0<d* 2 REG_PANOA_UN_SHSELOOOffset 0x00C5B5& 

#define BIT_SHIFT_USNOA_UN_S2_SEL 8define BIT_MASK_CPNOA_UN_S2_SEL 0x7define BIT_GENOA_UN_S2_SEL)                                                     	(((x) & &IT_MASK_BINOA_UN_S2_SEL<< BIT_SHIFT_WLNOA_UN_S2_SEL<define BIT_GET_BINOA_UN_S2_SEL)                                                 	(((x) &  BIT_SHIFT_USNOA_UN_S2_SEL<<&IT_MASK_BINOA_UN_S2_SEL<#define BIT_SHIFT_USNOA_UN_S1_SEL 4define BIT_MASK_CPNOA_UN_S1_SEL 0x7define BIT_GENOA_UN_S1_SEL)                                                     	(((x) & &IT_MASK_BINOA_UN_S1_SEL<< BIT_SHIFT_WLNOA_UN_S1_SEL<define BIT_GET_BINOA_UN_S1_SEL)                                                 	(((x) &  BIT_SHIFT_USNOA_UN_S1_SEL<<&IT_MASK_BINOA_UN_S1_SEL<#define BIT_SHIFT_USNOA_UN_S0_SEL 0define BIT_MASK_CPNOA_UN_S0_SEL 0x7define BIT_GENOA_UN_S0_SEL)                                                     	(((x) & &IT_MASK_BINOA_UN_S0_SEL<< BIT_SHIFT_WLNOA_UN_S0_SEL<define BIT_GET_BINOA_UN_S0_SEL)                                                 	(((x) &  BIT_SHIFT_USNOA_UN_S0_SEL<<&IT_MASK_BINOA_UN_S0_SEL<d* 2 REG_PAP2POFFPDIS_TXTIMEOOOffset 0x00C5B7& 

#define BIT_STIFT_USP2POFFPDIS_TXTIME 0define BIT_MASK_CPP2POFFPDIS_TXTIME 0xffdefine BIT_SHP2POFFPDIS_TXTIME)                                                 	(((x) & &IT_MASK_CPP2POFFPDIS_TXTIME<< BIT_SHIFT_WLP2POFFPDIS_TXTIME<define BIT_GET_BIP2POFFPDIS_TXTIME)                                             	(((x) &  BIT_SHIFT_USP2POFFPDIS_TXTIME<<&IT_MASK_CPP2POFFPDIS_TXTIME<d* 2 REG_PAMBSSID_BCNHSPACE2OOOffset 0x00C5B */

#define BIT_CPIFT_USBCNTSPACEHCLINT2 
#define BIT_MASK_BIBCNTSPACEHCLINT2 0f
#fdefine BIT_SEBCNTSPACEHCLINT2) &                                                	(((x) & &IT_MASK_CPBCNTSPACEHCLINT2<< BIT_SHIFT_WLBCNTSPACEHCLINT2<define BIT_GET_BIBCNTSPACEHCLINT2) &                                            	(((x) &  BIT_SHIFT_USBCNTSPACEHCLINT2& &IT_MASK_CPBCNTSPACEHCLINT2<#define BIT_CPIFT_USBCNTSPACEHCLINT1I0define BIT_MASK_CPBCNTSPACEHCLINT1I0f
#fdefine BIT_SEBCNTSPACEHCLINT1) &                                                	(((x) & &IT_MASK_CPBCNTSPACEHCLINT1<< BIT_SHIFT_WLBCNTSPACEHCLINT1<define BIT_GET_BIBCNTSPACEHCLINT1) &                                            	(((x) &  BIT_SHIFT_USBCNTSPACEHCLINT1& &IT_MASK_CPBCNTSPACEHCLINT1<d* 2 REG_PAMBSSID_BCNHSPACE3OOOffset 0x00C5B */

#define BIT_STIFT_USSUB_BCNHSPACE 
#define BIT_MASK_BISUB_BCNHSPACE 0xffdefine BIT_SHSUB_BCNHSPACE)                                                     	(((x) & &IT_MASK_BISUB_BCNHSPACE<< BIT_SHIFT_WLSUB_BCNHSPACE<define BIT_GET_BISUB_BCNHSPACE)                                                 	(((x) &  BIT_SHIFT_USSUB_BCNHSPACE<<&IT_MASK_BISUB_BCNHSPACE<d* 2 REG_PAMBSSID_BCNHSPACE3OOOffset 0x00C5B */

#define BIT_STIFT_USBCNTSPACEHCLINT3I0define BIT_MASK_BIBCNTSPACEHCLINT3I0f
#fdefine BIT_SEBCNTSPACEHCLINT3) &                                                	(((x) & &IT_MASK_CPBCNTSPACEHCLINT3<< BIT_SHIFT_WLBCNTSPACEHCLINT3<define BIT_GET_BIBCNTSPACEHCLINT3) &                                            	(((x) &  BIT_SHIFT_USBCNTSPACEHCLINT3& &IT_MASK_CPBCNTSPACEHCLINT3<d* 2 REG_PAACMHWCTRLOOOOffset 0x00C5C */

#define BIT_SHBEQAACMHSTATUSIT_C17)define BIT_CPVIQAACMHSTATUSIT_C16<#efine BIT_CPVOQAACMHSTATUSIT_C15<define BIT_CPBEQAACMHENIT_C13<#efine BIT_CPVIQAACMHENIT_C12<define BIT_CPVOQAACMHENIT_C11<define BIT_SHACMHWENIT_C10<d* 2 REG_PAACMRSTCTRLOOOOffset 0x00C5C1</

#define BIT_SHBEAACMHRES_BIUSED_TIMEIT_C12<define BIT_CPVIAACMHRES_BIUSED_TIMEIT_C11<define BIT_CPVOAACMHRES_BIUSED_TIMEIT_C10<d* 2 REG_PAACMAVGOOOOffset 0x00C5C2*/

#define BIT_CPIFT_USAVGPERIODI0define BIT_MASK_CPAVGPERIODI0fffffdefine BIT_STAVGPERIOD) & (x) & &IT_MASK_BIAVGPERIOD<< BIT_SHIFT_WLAVGPERIOD<define BIT_GET_BIAVGPERIOD) & (x) &  BIT_SHIFT_USAVGPERIOD<<&IT_MASK_BIAVGPERIOD<d* 2 REG_PAVOAADMTIMEOOOOffset 0x00C5C */

#define BIT_SHIFT_SDVOAADMITTED_TIMEI0define BIT_MASK_CPVOPADMITTED_TIMEI0fffffdefine BIT_STVOPADMITTED_TIME) &                                                	(((x) & &IT_MASK_CPVOPADMITTED_TIME<< BIT_SHIFT_WLVOPADMITTED_TIME<define BIT_GET_BIVOPADMITTED_TIME) &                                            	(((x) &  BIT_SHIFT_USVOPADMITTED_TIME<<&IT_MASK_CPVOPADMITTED_TIME<d* 2 REG_PAVIAADMTIMEOOOOffset 0x00C5C6*/

#define BIT_STIFT_USVIAADMITTED_TIMEI0define BIT_MASK_CPVIAADMITTED_TIMEI0fffffdefine BIT_STVIPADMITTED_TIME) &                                                	(((x) & &IT_MASK_CPVIPADMITTED_TIME<< BIT_SHIFT_WLVIPADMITTED_TIME<define BIT_GET_BIVIPADMITTED_TIME) &                                            	(((x) &  BIT_SHIFT_USVIPADMITTED_TIME<<&IT_MASK_CPVIPADMITTED_TIME<d* 2 REG_PABEAADMTIMEOOOOffset 0x00C5C */

#define BIT_CPIFT_USBEAADMITTED_TIMEI0define BIT_MASK_CPBEAADMITTED_TIMEI0fffffdefine BIT_STBEPADMITTED_TIME) &                                                	(((x) & &IT_MASK_CPBEPADMITTED_TIME<< BIT_SHIFT_WLBEPADMITTED_TIME<define BIT_GET_BIBEPADMITTED_TIME) &                                            	(((x) &  BIT_SHIFT_USBEPADMITTED_TIME<<&IT_MASK_CPBEPADMITTED_TIME<d* 2 REG_PAEDCA_RANDOMET_NOOOffset 0x00C5C */

#define BIT_STIFT_USRANDOMET_N 0define BIT_MASK_CPRANDOMET_N 0xffffffdefine BIT_STRANDOMET_N) & (x) & &IT_MASK_BIRANDOMET_N<< BIT_SHIFT_WLRANDOMET_N<define BIT_GET_BIRANDOMET_N) &                                                  	(((x) &  BIT_SHIFT_USRANDOMET_N<<&IT_MASK_BIRANDOMET_N<#* 2 REG_PATXCMD_NOA_SELOOOffset 0x00C5CF*/

#define BIT_STIFT_USNOA_SEL 4define BIT_MASK_CPNOA_SEL 0x7define BIT_GENOA_SEL) & (x) & &IT_MASK_BINOA_SEL<< BIT_SHIFT_WLNOA_SEL<define BIT_GET_BINOA_SEL) & (x) &  BIT_SHIFT_USNOA_SEL<<&IT_MASK_BINOA_SEL<#* 2 REG_PATXCMD_NOA_SELOOOffset 0x00C5CF*/

#define BIT_STIFT_USTXCMD_S_PASEL 0define BIT_MASK_CPTXCMD_S_PASEL 0xfdefine BIT_SHTXCMD_S_PASEL)                                                     	(((x) & &IT_MASK_BITXCMD_S_PASEL<< BIT_SHIFT_WLTXCMD_S_PASEL<define BIT_GET_BITXCMD_S_PASEL)                                                 	(((x) &  BIT_SHIFT_USTXCMD_S_PASEL<<&IT_MASK_BITXCMD_S_PASEL<d* 2 REG_PANOA_PARAM(OOOffset 0x00C5E */

#define BIT_SHIFT_LPNOA_COUNT (96<&ICPU_OPT_WIDTH)define BIT_MASK_CPNOA_COUNT 0xffdefine BIT_SHNOA_COUNT) & (x) & &IT_MASK_BINOA_COUNT<< BIT_SHIFT_WLNOA_COUNT<define BIT_GET_BINOA_COUNT) & (x) &  BIT_SHIFT_USNOA_COUNT<<&IT_MASK_BINOA_COUNT<#define BIT_STIFT_USNOA_START_TIMEI(64<&ICPU_OPT_WIDTH)define BIT_MASK_CPNOA_START_TIMEI0fffffffL
Ldefine BIT_GENOA_START_TIME) &                                                  	(((x) & &IT_MASK_BINOA_START_TIME<< BIT_SHIFT_WLNOA_START_TIME<define BIT_GET_BINOA_START_TIME) &                                              	(((x) &  BIT_SHIFT_USNOA_START_TIME<<&IT_MASK_BINOA_START_TIME<#define BIT_STIFT_USNOA_INTERVAL (32<&ICPU_OPT_WIDTH)define BIT_MASK_CPNOA_INTERVAL 0fffffffL
Ldefine BIT_GENOA_INTERVAL) &                                                    	(((x) & &IT_MASK_BINOA_INTERVAL<< BIT_SHIFT_WLNOA_INTERVAL<define BIT_GET_BINOA_INTERVAL) &                                                	(((x) &  BIT_SHIFT_USNOA_INTERVAL<<&IT_MASK_BINOA_INTERVAL<#define BIT_STIFT_USNOA_DURATIONI0define BIT_MASK_CPNOA_DURATIONI0fffffffL
Ldefine BIT_GENOA_DURATION) &                                                    	(((x) & &IT_MASK_BINOA_DURATION<< BIT_SHIFT_WLNOA_DURATION<define BIT_GET_BINOA_DURATION) &                                                	(((x) &  BIT_SHIFT_USNOA_DURATION<<&IT_MASK_BINOA_DURATION<d* 2 REG_PAP2P_RSMOOOOffset 0x00C5F */

#define BIT_SHP2P2HPWRTRSM1IT_C15<define BIT_CPP2P2HPWRTRSM0IB_C14<#efine BIT_CPP2P1HPWRTRSM1IT_C13<#efine BIT_CPP2P1HPWRTRSM0IB_C12<define BIT_CPP2PHPWRTRSM1_V1IT_C11<define BIT_CPP2PHPWRTRSM0_V1IT_C10<d* 2 REG_PASCHEDULER_RSMOOOffset 0x00C5F1</

#define BIT_SHIYNCPCLIIT_C11<define BIT_CPSCHEDULER_RSM_V1IT_C10<d* 2 REG_PASCHITXCMDOOOOffset 0x00C5F */

#define BIT_CPIFT_USSCHITXCMDI0define BIT_MASK_CPSCHITXCMDI0fffffffL
Ldefine BIT_GESCHITXCMD) & (x) & &IT_MASK_BISCHITXCMD<< BIT_SHIFT_WLSCHITXCMD<define BIT_GET_BISCHITXCMD) & (x) &  BIT_SHIFT_USSCHITXCMD<<&IT_MASK_BISCHITXCMD<d* 2 REG_PAWMACPCROOOOffset 0x00C60 */

#define BIT_SHICASKCPHY_MIT_C10<d* 2 REG_PAWMACPFWPKTPCROOOffset 0x00C601</

#define BIT_SHFWENIT_C17)dd 2 REG_PAWMACPFWPKTPCROOOffset 0x00C601</

#define BIT_SHPHYSTS_PKTPCTRLIT_C16)dd 2 REG_PAWMACPFWPKTPCROOOffset 0x00C601</

#define BIT_SHAPPHDR_MIDSRCHIFAILIB_C14<#efine BIT_CPFWPARSINGHENIT_C13<#define BIT_CPIFT_USAPPEND_MHDR_LEN 0define BIT_MASK_CPAPPEND_MHDR_LEN 0x7define BIT_GEAPPEND_MHDR_LEN)                                                   	(((x) & &IT_MASK_CPAPPEND_MHDR_LEN<< BIT_SHIFT_WLAPPEND_MHDR_LEN<define BIT_GET_BIAPPEND_MHDR_LEN)                                               	(((x) &  BIT_SHIFT_USAPPEND_MHDR_LEN<<&IT_MASK_CPAPPEND_MHDR_LEN<#* 2 REG_PATCROOOOOffset 0x00C60 */

#define BIT_SHWMACPEN_RTS_ADDRIT_C131<define BIT_CPWMACPDISABLEPCCCIT_C130<define BIT_CPWMACPRAW_LEN B_C129<define BIT_CPWMACPNOTX_IN_RXNDPIT_C128<define BIT_CPWMACPEN_EOFIT_C127<define BIT_CPWMACPBFASEL T_C126<define BIT_CPWMACPANTMODEASEL T_C125<#* 2 REG_PATCROOOOOffset 0x00C60 */

#define BIT_SHWMACPTCRPWRMGSHHWCTL T_C124<#* 2 REG_PATCROOOOOffset 0x00C60 */

#define BIT_SHWMACPSMOOTH_VAL T_C123<#* 2 REG_PATCROOOOOffset 0x00C60 */

#define BIT_SHFETCHIMPDU_AFTER_WSECPRDY T_C120<#* 2 REG_PATCROOOOOffset 0x00C60 */

#define BIT_SHWMACPTCRPEN_20MSMIT_C119<define BIT_CPWMACPDIS_SIGTAIB_C118<define BIT_CPWMACPDIS_A2B0IB_C117<define BIT_CPWMACPM_BISIGBCRCIB_C116<#* 2 REG_PATCROOOOOffset 0x00C60 */

#define BIT_SHWMACPTCRPERRSMEN_3IB_C115<define BIT_CPWMACPTCRPERRSMEN_2IB(3114<define BIT_CPWMACPTCRPERRSMEN_1IT_C113<define BIT_CPWMACPTCRPERRSMEN_0IB_C112<define BIT_CPWMACPTCRPTX_CPPERPKTIB_C111<define BIT_CPICVIB_C110<define BIT_CPCFEND_FORMATIB_C19<define BIT_CPCRCIB_C18<define BIT_CPPWRT_CPOWHENIT_C17)define BIT_CPPWR_STIT_C16<#efine BIT_CPWMACPTCRPUPD_TIMIEIT_C15<define BIT_CPWMACPTCRPUPD_HGQMDIB_C14<#* 2 REG_PATCROOOOOffset 0x00C60 */

#define BIT_SHVHTSIGA1PTXPSIT_C13<#d 2 REG_PATCROOOOOffset 0x00C60 */

#define BIT_SHPADASEL T_C12<#efine BIT_CPDIS_GCLKIT_C11<d* 2 REG_PARCROOOOOffset 0x00C60 */

#define BIT_CPAPP_FCSIT_C131<define BIT_CPAPP_MICIT_C130<define BIT_CPAPP_ICVIB_C129<define BIT_CPAPP_PHYSTSIT_C128<define BIT_CPAPP_BASSNIT_C127<d* 2 REG_PARCROOOOOffset 0x00C60 */

#define BIT_CPVHT_DACCIT_C126<d* 2 REG_PARCROOOOOffset 0x00C60 */

#define BIT_CPTCPOFLDHENIT_C125<define BIT_CPENMBID T_C124<#efine BIT_CPLSIGENIT_C123<define BIT_CPMFBENIT_C122<#efine BIT_CPDISCHKPPDLLEN B_C121<define BIT_CPPKTCTL_DLEN B_C120<define BIT_CPTIM_PARSERHENIT_C118<define BIT_CPBCPMDHENIT_C117<define BIT_CPUCPMDHENIT_C116<#efine BIT_CPRX_CPPERPKTIB_C115<define BIT_GEHTC_LOCPCTRLIT_C114<d* 2 REG_PARCROOOOOffset 0x00C60 */

#define BIT_CPRPFM_CAMHENABLEIB_C112<d* 2 REG_PARCROOOOOffset 0x00C60 */

#define BIT_CPTAABCNIT_C111<d* 2 REG_PARCROOOOOffset 0x00C60 */

#define BIT_CPDISDECMYPKTIB_C110<define BIT_CPAICVIB_C19<define BIT_CPACRC32IB_C18<define BIT_CPCBSSID_BCNIT_C17)define BIT_CPCBSSID_DATAIB_C16<define BIT_CPAPWRMGSIT_C15<define BIT_CPADD3IB_C14<#efine BIT_CPABIT_C13<#efine BIT_CPAM T_C12<#efine BIT_CPAPMIT_C11<define BIT_CPAAPIB_C10<d* 2 REG_PARX_PKTPLIMITOOOffset 0x00C60 */

#define BIT_STIFT_USRXPKTLMMI0define BIT_MASK_CPRXPKTLMMI0x3fdefine BIT_STRXPKTLMM) & (x) & &IT_MASK_BIRXPKTLMM<< BIT_SHIFT_WLRXPKTLMM<define BIT_GET_BIRXPKTLMM) & (x) &  BIT_SHIFT_USRXPKTLMM<<&IT_MASK_BIRXPKTLMM<d* 2 REG_PARX_DLK_TIMEOOOOffset 0x00C60D*/

#define BIT_SHIFT_SDRX_DLK_TIMEI0define BIT_MASK_CPRX_DLK_TIMEI0xffdefine BIT_SHRX_DLK_TIME) &                                                     	(((x) & &IT_MASK_CPRX_DLK_TIME<< BIT_SHIFT_WLRX_DLK_TIME<define BIT_GET_BIRX_DLK_TIME) &                                                 	(((x) &  BIT_SHIFT_USRX_DLK_TIME<<&IT_MASK_CPRX_DLK_TIME<d* 2 REG_PARX_DRVINFO_SZOOOffset 0x00C60F*/

#define BIT_STDATAPRPFM15ENIT_C115<define BIT_GEDATAPRPFM14ENIT_C114<define BIT_GEDATAPRPFM13ENIT_C113<define BIT_GEDATAPRPFM12ENIT_C112<#efine BIT_CPDATAPRPFM11ENIT_C111<define BIT_CPDATAPRPFM10ENIT_C110<define BIT_CPDATAPRPFM9ENIT_C19<define BIT_CPDATAPRPFM8ENIT_C18<d* 2 REG_PARX_DRVINFO_SZOOOffset 0x00C60F*/

#define BIT_STPHYSTS_PERHPKTPMODEIT_C17)define BIT_CPDATAPRPFM7ENIT_C17)dd 2 REG_PARX_DRVINFO_SZOOOffset 0x00C60F*/

#define BIT_STDATAPRPFM6ENIT_C16<#d 2 REG_PARX_DRVINFO_SZOOOffset 0x00C60F*/

#define BIT_STDATAPRPFM5ENIT_C15<define BIT_GEDATAPRPFM4ENIT_C14<define BIT_GEDATAPRPFM3ENIT_C13<#efine BIT_CPDATAPRPFM2ENIT_C12<define BIT_CPDATAPRPFM1ENIT_C11<d* 2 REG_PARX_DRVINFO_SZOOOffset 0x00C60F*/

#define BIT_STIFT_USDRVINFO_SZ_V1I0define BIT_MASK_BIDRVINFO_SZ_V1I0xfdefine BIT_SHDRVINFO_SZ_V1)                                                     	(((x) & &IT_MASK_BIDRVINFO_SZ_V1<< BIT_SHIFT_WLDRVINFO_SZ_V1<define BIT_GET_BIDRVINFO_SZ_V1)                                                 	(((x) &  BIT_SHIFT_USDRVINFO_SZ_V1<<&IT_MASK_BIDRVINFO_SZ_V1<#d 2 REG_PARX_DRVINFO_SZOOOffset 0x00C60F*/

#define BIT_STDATAPRPFM0ENIT_C10<d* 2 REG_PAMACIDOOOOffset 0x00C61 */

#define BIT_SHIFT_LPMACIDI0define BIT_MASK_BIMACIDI0fffffffL
ffffLdefine BIT_MASKCID) & (x) & &IT_MASK_BISKCID<< BIT_SHIFT_WLSKCID<define BIT_GET_BISKCID) & (x) &  BIT_SHIFT_USSKCID<<&IT_MASK_BISKCID<d* 2 REG_PABSSIDOOOOffset 0x00C61 */

#define BIT_CPIFT_USBSSIDI0define BIT_MASK_CPBSSIDI0fffffffL
ffffLdefine BIT_MABSSID) & (x) & &IT_MASK_BIBSSID<< BIT_SHIFT_WLBSSID<define BIT_GET_BIBSSID) & (x) &  BIT_SHIFT_USBSSID<<&IT_MASK_BIBSSID<d* 2 REG_PAMAROOOOOffset 0x00C62 */

#define BIT_SHIFT_LPMAR 0define BIT_MASK_CPMAR 0fffffffL
ffffffL
#define BIT_DEMAR) & (x) & &IT_MASK_BISKR<< BIT_SHIFT_WLSKR<define BIT_GET_BISKR) & (x) &  BIT_SHIFT_USSKR<<&IT_MASK_BISKR<d* 2 REG_PAMBIDCAMCFG_1OOOffset 0x00C62 */

#define BIT_CPIFT_USMBIDCAM_RWDATAPL 0define BIT_MASK_CPMBIDCAM_RWDATAPL 0fffffffL
Ldefine BIT_GEMBIDCAM_RWDATAPL) &                                                	(((x) & &IT_MASK_CPMBIDCAM_RWDATAPL<< BIT_SHIFT_WLSBIDCAM_RWDATAPL<define BIT_GET_BISBIDCAM_RWDATAPL) &                                            	(((x) &  BIT_SHIFT_USMBIDCAM_RWDATAPL<<&IT_MASK_CPMBIDCAM_RWDATAPL<d* 2 REG_PAMBIDCAMCFG_2OOOffset 0x00C62 */

#define BIT_STMBIDCAM_POLLIT_C131<define BIT_CPMBIDCAM_WSHENIT_C130<ddefine BIT_CPIFT_USMBIDCAM_ADDRI24define BIT_MASK_CPMBIDCAM_ADDRI0x1fdefine BIT_CPMBIDCAM_ADDR) &                                                    	(((x) & &IT_MASK_BIMBIDCAM_ADDR<< BIT_SHIFT_WLSBIDCAM_ADDR<define BIT_GET_BISBIDCAM_ADDR) &                                                	(((x) &  BIT_SHIFT_USMBIDCAM_ADDR<<&IT_MASK_BIMBIDCAM_ADDR<#define BIT_STMBIDCAM_VALID T_C123<#efine BIT_CPLSIC_TXOPIENIT_C117<d* 2 REG_PAMBIDCAMCFG_2OOOffset 0x00C62 */

#define BIT_STCTS_ENIT_C116<#* 2 REG_PAMBIDCAMCFG_2OOOffset 0x00C62 */

#define BIT_STIFT_USMBIDCAM_RWDATAPH 0define BIT_MASK_CPMBIDCAM_RWDATAPHI0fffffdefine BIT_STMBIDCAM_RWDATAPH) &                                                	(((x) & &IT_MASK_CPMBIDCAM_RWDATAPH<< BIT_SHIFT_WLSBIDCAM_RWDATAPH<define BIT_GET_BISBIDCAM_RWDATAPH) &                                            	(((x) &  BIT_SHIFT_USMBIDCAM_RWDATAPH& &IT_MASK_CPMBIDCAM_RWDATAPH<dd 2 REG_PAWMACPTCRPTSFM_OFS(OOffset 0x00C63 */

#define BIT_SHIFT_LPWMACPTCRPTSFM_OFS 0define BIT_MASK_CPWMACPTCRPTSFM_OFS 0fffffdefine BIT_STWMACPTCRPTSFM_OFS)                                                 	(((x) & &IT_MASK_CPWMACPTCRPTSFM_OFS<< BIT_SHIFT_WLWMACPTCRPTSFM_OFS<define BIT_GET_BIWMACPTCRPTSFM_OFS)                                             	(((x) &  BIT_SHIFT_USWMACPTCRPTSFM_OFS<<&IT_MASK_CPWMACPTCRPTSFM_OFS<dd 2 REG_PAUDF_THSDOOOOffset 0x00C632*/

#define BIT_CPIFT_USUDF_THSD 0define BIT_MASK_CPUDF_THSD 0xffdefine BIT_SHUDF_THSD) & (x) & &IT_MASK_BIUDF_THSD<< BIT_SHIFT_WLUDF_THSD<define BIT_GET_BIUDF_THSD) & (x) &  BIT_SHIFT_USUDF_THSD<<&IT_MASK_BIUDF_THSD<dd 2 REG_PAZLD_NUM(OOOffset 0x00C633</

#define BIT_SHIFT_USZLD_NUM 0define BIT_MASK_CPZLD_NUM 0xffdefine BIT_SHZLD_NUM) & (x) & &IT_MASK_BIZLD_NUM<< BIT_SHIFT_WLZLD_NUM<define BIT_GET_BIZLD_NUM) & (x) &  BIT_SHIFT_USZLD_NUM<<&IT_MASK_BIZLD_NUM<d* 2 REG_PASTMP_THSDOOOOffset 0x00C63 */

#define BIT_SHIFT_SDSTMP_THSDI0define BIT_MASK_CPSTMP_THSDI0xffdefine BIT_SHSTMP_THSD) & (x) & &IT_MASK_BISTMP_THSD<< BIT_SHIFT_WLSTMP_THSD<define BIT_GET_BISTMP_THSD) & (x) &  BIT_SHIFT_USSTMP_THSD<<&IT_MASK_BISTMP_THSD<dd 2 REG_PAWMACPTXTIMEOUTOOOffset 0x00C635& 

#define BIT_SHIFT_USWMACPTXTIMEOUT 0define BIT_MASK_CPWMACPTXTIMEOUT 0xffdefine BIT_SHWMACPTXTIMEOUT) &                                                  	(((x) & &IT_MASK_BIWMACPTXTIMEOUT<< BIT_SHIFT_WLWMACPTXTIMEOUT<define BIT_GET_BIWMACPTXTIMEOUT) &                                              	(((x) &  BIT_SHIFT_USWMACPTXTIMEOUT<<&IT_MASK_BIWMACPTXTIMEOUT<#* 2 REG_PAMCU_TEST_2_V1OOOffset 0x00C636*/

#define BIT_STIFT_USMCU_RSVD_2_V1 0define BIT_MASK_CPMCU_RSVD_2_V1 0fffffdefine BIT_STMCU_RSVD_2_V1)                                                     	(((x) & &IT_MASK_BIMCU_RSVD_2_V1<< BIT_SHIFT_WLSCU_RSVD_2_V1<define BIT_GET_BISCU_RSVD_2_V1)                                                 	(((x) &  BIT_SHIFT_USMCU_RSVD_2_V1<<&IT_MASK_BIMCU_RSVD_2_V1<dd 2 REG_PAUSTIMEAEDCAOOOOffset 0x00C63 */

#define BIT_CPIFT_USUSTIMEAEDCA_V1 0define BIT_MASK_CPUSTIMEAEDCA_V1 0x1ffdefine BIT_SHUSTIMEAEDCA_V1) &                                                  	(((x) & &IT_MASK_BIUSTIMEAEDCA_V1<< BIT_SHIFT_WLUSTIMEAEDCA_V1<define BIT_GET_BIUSTIMEAEDCA_V1) &                                              	(((x) &  BIT_SHIFT_USUSTIMEAEDCA_V1<<&IT_MASK_BIUSTIMEAEDCA_V1<d* 2 REG_PAMAC_SPECISIFS(OOffset 0x00C63A*/

#define BIT_CPIFT_USSPECISIFS_OFDM 8define BIT_MASK_CPSPECISIFS_OFDM 0xffdefine BIT_SHSPECISIFS_OFDM) &                                                  	(((x) & &IT_MASK_BISPECISIFS_OFDM<< BIT_SHIFT_WLSPECISIFS_OFDM<define BIT_GET_BISPECISIFS_OFDM) &                                              	(((x) &  BIT_SHIFT_USSPECISIFS_OFDM<<&IT_MASK_BISPECISIFS_OFDM<#define BIT_CPIFT_USSPECISIFS_CCCI0define BIT_MASK_CPSPECISIFS_CCCI0xffdefine BIT_SHSPECISIFS_CCC)                                                     	(((x) & &IT_MASK_BISPECISIFS_CCC<< BIT_SHIFT_WLSPECISIFS_CCC<define BIT_GET_BISPECISIFS_CCC)                                                 	(((x) &  BIT_SHIFT_USSPECISIFS_CCC<<&IT_MASK_BISPECISIFS_CCC<#d 2 REG_PARESPISIFS_CCC(OOffset 0x00C63 */

#define BIT_STIFT_USSIFS_R2T_CCCI8define BIT_MASK_CPSIFS_R2T_CCCI0xffdefine BIT_SHSIFS_R2T_CCC) &                                                    	(((x) & &IT_MASK_BISIFS_R2T_CCC<< BIT_SHIFT_WLSIFS_R2T_CCC<define BIT_GET_BISIFS_R2T_CCC) &                                                	(((x) &  BIT_SHIFT_USSIFS_R2T_CCC<<&IT_MASK_BISIFS_R2T_CCC<#define BIT_STIFT_USSIFS_T2T_CCCI0define BIT_MASK_CPSIFS_T2T_CCCI0xffdefine BIT_SHSIFS_T2T_CCC) &                                                    	(((x) & &IT_MASK_BISIFS_T2T_CCC<< BIT_SHIFT_WLSIFS_T2T_CCC<define BIT_GET_BISIFS_T2T_CCC) &                                                	(((x) &  BIT_SHIFT_USSIFS_T2T_CCC<<&IT_MASK_BISIFS_T2T_CCC<dd 2 REG_PARESPISIFS_OFDM(OOffset 0x00C63E*/

#define BIT_STIFT_RDSIFS_R2T_OFDM 8define BIT_MASK_CPSIFS_R2T_OFDM 0xffdefine BIT_SHSIFS_R2T_OFDM) &                                                   	(((x) & &IT_MASK_BISIFS_R2T_OFDM<< BIT_SHIFT_WLSIFS_R2T_OFDM<define BIT_GET_BISIFS_R2T_OFDM) &                                               	(((x) &  BIT_SHIFT_USSIFS_R2T_OFDM<<&IT_MASK_BISIFS_R2T_OFDM<ddefine BIT_STIFT_USSIFS_T2T_OFDM 0define BIT_MASK_CPSIFS_T2T_OFDM 0xffdefine BIT_SHSIFS_T2T_OFDM) &                                                   	(((x) & &IT_MASK_BISIFS_T2T_OFDM<< BIT_SHIFT_WLSIFS_T2T_OFDM<define BIT_GET_BISIFS_T2T_OFDM) &                                               	(((x) &  BIT_SHIFT_USSIFS_T2T_OFDM<<&IT_MASK_BISIFS_T2T_OFDM<d* 2 REG_PAACKTOOOOOffset 0x00C64 */

#define BIT_SHIFT_LPACKTO 0define BIT_MASK_CPACKTO 0xffdefine BIT_SHACKTO) & (x) & &IT_MASK_BIACKTO<< BIT_SHIFT_WLACKTO<define BIT_GET_BIACKTO) & (x) &  BIT_SHIFT_USACKTO<<&IT_MASK_BIACKTO<d* 2 REG_PACTS2TOOOOOffset 0x00C641</

#define BIT_SHIFT_USCTS2TO 0define BIT_MASK_CPCTS2TO 0xffdefine BIT_SHCTS2TO) & (x) & &IT_MASK_BICTS2TO<< BIT_SHIFT_WLCTS2TO<define BIT_GET_BICTS2TO) & (x) &  BIT_SHIFT_USCTS2TO<<&IT_MASK_BICTS2TO<d* 2 REG_PAEIFS(OOOffset 0x00C642*/

#define BIT_CPIFT_USEIFS 0define BIT_MASK_CPEIFS 0fffffdefine BIT_STEIFS)   (x) & &IT_MASK_CPEIFS<< BIT_SHIFT_WLEIFS<define BIT_GET_BIEIFS)   (x) &  BIT_SHIFT_USEIFS<<&IT_MASK_BIEIFS<#* 2 REG_PANAV_CTRLOOOOffset 0x00C65 */

#define BIT_SHIFT_LPNAV_UPPER 
#define BIT_MASK_BINAV_UPPER 0xffdefine BIT_SHNAV_UPPER) & (x) & &IT_MASK_BINAV_UPPER<< BIT_SHIFT_WLNAV_UPPER<define BIT_GET_BINAV_UPPER) & (x) &  BIT_SHIFT_USNAV_UPPER<<&IT_MASK_BINAV_UPPER<#define BIT_SHIFT_SDRXMYRTSINAV 8define BIT_MASK_CPRXMYRTSINAV 0xfdefine BIT_SHRXMYRTSINAV) &                                                     	(((x) & &IT_MASK_CPRXMYRTSINAV<< BIT_SHIFT_WLRXMYRTSINAV<define BIT_GET_BIRXMYRTSINAV) &                                                 	(((x) &  BIT_SHIFT_USRXMYRTSINAV<<&IT_MASK_CPRXMYRTSINAV<#define BIT_SHIFT_SDRTSRSMI0define BIT_MASK_CPRTSRSMI0xffdefine BIT_SHRTSRSM) & (x) & &IT_MASK_BIRTSRSM<< BIT_SHIFT_WLRTSRSM<define BIT_GET_BIRTSRSM) & (x) &  BIT_SHIFT_USRTSRSM<<&IT_MASK_BIRTSRSM<d* 2 REG_PABACAMCMDOOOOffset 0x00C65 */

#define BIT_SHBACAM_POLLIT_C131<define BIT_CPBACAM_RSMIT_C117<define BIT_CPBACAM_RWIT_C116<#define BIT_SHIFT_USTXSBM 14define BIT_MASK_CPTXSBM 0x3define BIT_MATXSBM) & (x) & &IT_MASK_BITXSBM<< BIT_SHIFT_WLTXSBM<define BIT_GET_BITXSBM) & (x) &  BIT_SHIFT_USTXSBM<<&IT_MASK_BITXSBM<#define BIT_CPIFT_USBACAM_ADDRI0define BIT_MASK_CPBACAM_ADDRI0x3fdefine BIT_STBACAM_ADDR) & (x) & &IT_MASK_BIBACAM_ADDR<< BIT_SHIFT_WLBACAM_ADDR<define BIT_GET_BIBACAM_ADDR) &                                                  	(((x) &  BIT_SHIFT_USBACAM_ADDR<<&IT_MASK_BIBACAM_ADDR<d* 2 REG_PABACAMCONTENTOOOffset 0x00C65 */

#define BIT_CPIFT_USBA_CONTENT_H (32<&ICPU_OPT_WIDTH)define BIT_MASK_CPBA_CONTENT_H 0fffffffL
Ldefine BIT_GEBA_CONTENT_H) &                                                    	(((x) & &IT_MASK_BIBA_CONTENT_H<< BIT_SHIFT_WLBA_CONTENT_H<define BIT_GET_BIBA_CONTENT_H) &                                                	(((x) &  BIT_SHIFT_USBA_CONTENT_H<<&IT_MASK_BIBA_CONTENT_H<#define BIT_CPIFT_USBA_CONTENT_L 0define BIT_MASK_CPBA_CONTENT_L 0fffffffL
Ldefine BIT_GEBA_CONTENT_L) &                                                    	(((x) & &IT_MASK_BIBA_CONTENT_L<< BIT_SHIFT_WLBA_CONTENT_L<define BIT_GET_BIBA_CONTENT_L) &                                                	(((x) &  BIT_SHIFT_USBA_CONTENT_L& &IT_MASK_BIBA_CONTENT_L<d* 2 REG_PALBDLYOOOOffset 0x00C66 */

#define BIT_SHIFT_LPLBDLY 0define BIT_MASK_CPLBDLY 0x1fdefine BIT_CPLBDLY) & (x) & &IT_MASK_BILBDLY<< BIT_SHIFT_WLLBDLY<define BIT_GET_BILBDLY) & (x) &  BIT_SHIFT_USLBDLY<<&IT_MASK_BILBDLY<dd 2 REG_PAWMACPBACAM_RPM_NOOOffset 0x00C661</

#define BIT_SHIFT_UST_SMAPISSNBK_COUNTER 2define BIT_MASK_CPB_SMAPISSNBK_COUNTER 0x3fdefine BIT_STB_SMAPISSNBK_COUNTER) &                                            	(((x) & &IT_MASK_CPB_SMAPISSNBK_COUNTER&                                 	((< BIT_SHIFT_WLB_SMAPISSNBK_COUNTER&define BIT_GET_BIB_SMAPISSNBK_COUNTER) &                                        	(((x) &  BIT_SHIFT_USB_SMAPISSNBK_COUNTER& &                             	((<T_MASK_CPB_SMAPISSNBK_COUNTER&
define BIT_STB_SMAPIENIT_C11<d* 2 REG_PAWMACPBACAM_RPM_NOOOffset 0x00C661</

#define BIT_SHWMACPBACAM_RPM_NIT_C10<#* 2 REG_PATXPRXOOOOffset 0x00C662*/

#define BIT_CPIFT_USRXPKT_TYPE 2define BIT_MASK_CPRXPKT_TYPE 0x3fdefine BIT_STRXPKT_TYPE) & (x) & &IT_MASK_BIRXPKT_TYPE<< BIT_SHIFT_WLRXPKT_TYPE<define BIT_GET_BIRXPKT_TYPE) &                                                  	(((x) &  BIT_SHIFT_USRXPKT_TYPE<<&IT_MASK_BIRXPKT_TYPE<#define BIT_CPTXACT_INDIT_C11<define BIT_STRXACT_INDIT_C10<d* 2 REG_PAWMACPB_SMAPICTLOOOffset 0x00C663</

#define BIT_SHB_SMAPIVOIT_C17<define BIT_CPB_SMAPIVIIT_C16<define BIT_CPB_SMAPIBEIT_C15<define BIT_CPB_SMAPIBCIT_C14<#define BIT_CPIFT_USB_SMAPICONDITIONI2define BIT_MASK_CPB_SMAPICONDITIONI0x3define BIT_MAB_SMAPICONDITION) &                                                	(((x) & &IT_MASK_CPB_SMAPICONDITION<< BIT_SHIFT_WLB_SMAPICONDITION<define BIT_GET_BIB_SMAPICONDITION) &                                            	(((x) &  BIT_SHIFT_USB_SMAPICONDITION<<&IT_MASK_CPB_SMAPICONDITION<#define BIT_SHB_SMAPISSNBK_COUNTER_CLRIT_C11<define BIT_STB_SMAPIFORCEIT_C10<d* 2 REG_PARXERR_RPMOOOOffset 0x00C66 */

#define BIT_SHIFT_SDRXERR_RPM_SEL_V1_3_0 28define BIT_MASK_CPRXERR_RPM_SEL_V1_3_0 0xfdefine BIT_SHRXERR_RPM_SEL_V1_3_0) &                                            	(((x) & &IT_MASK_CPRXERR_RPM_SEL_V1_3_0&                                 	((< BIT_SHIFT_WLRXERR_RPM_SEL_V1_3_0&define BIT_GET_BIRXERR_RPM_SEL_V1_3_0) &                                        	(((x) &  BIT_SHIFT_USRXERR_RPM_SEL_V1_3_0& &                             	((<T_MASK_CPRXERR_RPM_SEL_V1_3_0&d* 2 REG_PARXERR_RPMOOOOffset 0x00C66 */

#define BIT_SHRXERR_RPM_RSMIT_C127<d* 2 REG_PARXERR_RPMOOOOffset 0x00C66 */

#define BIT_SHRXERR_RPM_SEL_V1_4IB_C126<d* 2 REG_PARXERR_RPMOOOOffset 0x00C66 */

#define BIT_SHW1SIT_C123<d* 2 REG_PARXERR_RPMOOOOffset 0x00C66 */

#define BIT_SHUDASELECUSBSSIDIB_C122<#* 2 REG_PARXERR_RPMOOOOffset 0x00C66 */

#define BIT_SHIFT_SDUDASUB_TYPE 18define BIT_MASK_CPUDASUB_TYPE 0xfdefine BIT_SHUDASUB_TYPE) &                                                     	(((x) & &IT_MASK_CPUDASUB_TYPE<< BIT_SHIFT_WLUDASUB_TYPE<define BIT_GET_BIUDASUB_TYPE) &                                                 	(((x) &  BIT_SHIFT_USUDASUB_TYPE<<&IT_MASK_CPUDASUB_TYPE<#define BIT_SHIFT_SDUDATYPE 1#define BIT_MASK_BIUDATYPE 0x3define BIT_MAUDATYPE) & (x) & &IT_MASK_BIUD_TYPE<< BIT_SHIFT_WLUDATYPE<define BIT_GET_BIUDATYPE) & (x) &  BIT_SHIFT_USUDATYPE<<&IT_MASK_CPUDATYPE<#define BIT_SHIFT_SDRPM_COUNTER 0define BIT_MASK_CPRPM_COUNTER 0fffffdefine BIT_STRPM_COUNTER) &                                                     	(((x) & &IT_MASK_CPRPM_COUNTER<< BIT_SHIFT_WLRPM_COUNTER<define BIT_GET_BIRPM_COUNTER) &                                                 	(((x) &  BIT_SHIFT_USRPM_COUNTER<<&IT_MASK_CPRPM_COUNTER<dd 2 REG_PAWMACPTRXPTCLICTLOOOffset 0x00C66 */

#define BIT_CPIFT_USACKBA_TYPSEL (60<&ICPU_OPT_WIDTH)define BIT_MASK_CPACKBA_TYPSEL 0xfdefine BIT_SHACKBA_TYPSEL) &                                                    	(((x) & &IT_MASK_BIACKBA_TYPSEL<< BIT_SHIFT_WLACKBA_TYPSEL<define BIT_GET_BIACKBA_TYPSEL) &                                                	(((x) &  BIT_SHIFT_USACKBA_TYPSEL<<&IT_MASK_BIACKBA_TYPSEL<#define BIT_CPIFT_USACKBA_ACKPCHK (56<&ICPU_OPT_WIDTH)define BIT_MASK_CPACKBA_ACKPCHK 0xfdefine BIT_SHACKBA_ACKPCHK) &                                                   	(((x) & &IT_MASK_BIACKBA_ACKPCHK<< BIT_SHIFT_WLACKBA_ACKPCHK<define BIT_GET_BIACKBA_ACKPCHK) &                                               	(((x) &  BIT_SHIFT_USACKBA_ACKPCHK<<&IT_MASK_BIACKBA_ACKPCHK<#define BIT_CPIFT_USACKBARATYPESEL (48<&ICPU_OPT_WIDTH)define BIT_MASK_CPACKBARATYPESEL 0xffdefine BIT_SHACKBARATYPESEL) &                                                  	(((x) & &IT_MASK_BIACKBARATYPESEL<< BIT_SHIFT_WLACKBARATYPESEL<define BIT_GET_BIACKBARATYPESEL) &                                              	(((x) &  BIT_SHIFT_USACKBARATYPESEL<<&IT_MASK_BIACKBARATYPESEL<#define BIT_CPIFT_USACKBARAACKPCHK (44<&ICPU_OPT_WIDTH)define BIT_MASK_CPACKBARAACKPCHK 0xfdefine BIT_SHACKBAR_ACKPCHK) &                                                  	(((x) & &IT_MASK_BIACKBARAACKPCHK<< BIT_SHIFT_WLACKBAR_ACKPCHK<define BIT_GET_BIACKBAR_ACKPCHK) &                                              	(((x) &  BIT_SHIFT_USACKBARAACKPCHK<<&IT_MASK_BIACKBAR_ACKPCHK<dd 2 REG_PAWMACPTRXPTCLICTLOOOffset 0x00C66 */

#define BIT_CPRXBA_IGNOREA2IB_C142<define BIT_CPEN_SAVE_ALL_TXOPADDRIT_C141<define BIT_CPEN_TXCTS_TO_TXOPOWNER_INRXNAV T_C140<d* 2 REG_PAWMACPTRXPTCLICTLOOOffset 0x00C66 */

#define BIT_CPDIS_TXBA_AMPDUFCSERR T_C139<define BIT_CPDIS_TXBA_RXBARINFULLIT_C138<define BIT_CPDIS_TXCFE_INFULLIT_C137<define BIT_GEDIS_TXCTS_INFULLIT_C136<define BIT_CPEN_TXACKBA_INATXPRDGIT_C135<define BIT_CPEN_TXACKBA_INATXOPIB_C134<define BIT_CPEN_TXCTS_IN_RXNAV T_C133<define BIT_CPEN_TXCTS_INTXOPIB_C132<define BIT_CPBLKAEDCA_BBSLPIB_C131<define BIT_STBLKAEDCA_BBSBY T_C130<define BIT_CPACKTOTBLOCBISCHIENIT_C127<define BIT_STEIFSTBLOCBISCHIENIT_C126<#efine BIT_CPPLCPCHK_RSM_EIFS B_C125<define BIT_CPCCA_RSM_EIFS B_C124<define BIT_GEDIS_UPD_MYRXPKTNAV T_C123<#efine BIT_CPEARLYPTXBAIB_C122<#*efine BIT_SHIFT_SDRESPICHNBUSY 20define BIT_MASK_CPRESPICHNBUSY 0x3define BIT_MARESPICHNBUSY) &                                                    	(((x) & &IT_MASK_BIRESPICHNBUSY<< BIT_SHIFT_WLRESPICHNBUSY<define BIT_GET_BIRESPICHNBUSY) &                                                	(((x) &  BIT_SHIFT_USRESPICHNBUSY<<&IT_MASK_BIRESPICHNBUSY<
define BIT_MARESPIDCTS_ENIT_C119<define BIT_CPRESPIDCFE_ENIT_C118<define BIT_CPRESPISPLCPENIT_C117<define BIT_CPRESPISGIENIT_C116<#* 2 REG_PAWMACPTRXPTCLICTLOOOffset 0x00C66 */

#define BIT_CPRESPILDPC_ENIT_C115<define BIT_GEDIS_RESPIACKINCCAIT_C114<define BIT_GEDIS_RESPICTSINCCAIT_C113<dd 2 REG_PAWMACPTRXPTCLICTLOOOffset 0x00C66 */

#define BIT_CPIFT_USRHWMACPSECONDPCCA_TIMER 10define BIT_MASK_CPRHWMACPSECONDPCCA_TIMER 0x7define BIT_GERHWMACPSECONDPCCA_TIMER) &                                         	(((x) & &IT_MASK_BIRHWMACPSECONDPCCA_TIMER&                              	((< BIT_SHIFT_WLRHWMACPSECONDPCCA_TIMER&define BIT_GET_BIRHWMACPSECONDPCCA_TIMER) &                                     	(((x) &  BIT_SHIFT_USRHWMACPSECONDPCCA_TIMER& &                          	((<T_MASK_CPRHWMACPSECONDPCCA_TIMER&dd 2 REG_PAWMACPTRXPTCLICTLOOOffset 0x00C66 */

#define BIT_CPIFT_USRFMOD 7define BIT_MASK_CPRFMOD 0x3define BIT_MARFMOD) & (x) & &IT_MASK_BIRFMOD<< BIT_SHIFT_WLRFMOD<define BIT_GET_BIRFMOD) & (x) &  BIT_SHIFT_USRFMOD<<&IT_MASK_BIRFMOD<dd 2 REG_PAWMACPTRXPTCLICTLOOOffset 0x00C66 */

#define BIT_CPIFT_USRESPICTS_DYNBWASEL 5define BIT_MASK_CPRESPICTS_DYNBWASEL 0x3define BIT_MARESPICTS_DYNBWASEL) &                                              	(((x) & &IT_MASK_BIRESPICTS_DYNBWASEL<< BIT_SHIFT_WLRESPICTS_DYNBWASEL<define BIT_GET_BIRESPICTS_DYNBWASEL) &                                          	(((x) &  BIT_SHIFT_USRESPICTS_DYNBWASEL<<&IT_MASK_BIRESPICTS_DYNBWASEL<d* 2 REG_PAWMACPTRXPTCLICTLOOOffset 0x00C66 */

#define BIT_CPDLYPTX_WA_CPRXANTSEL T_C14<d* 2 REG_PAWMACPTRXPTCLICTLOOOffset 0x00C66 */

#define BIT_CPTXRESPIBYPRXANTSEL T_C13<dd 2 REG_PAWMACPTRXPTCLICTLOOOffset 0x00C66 */

#define BIT_CPIFT_USORIGIDCTS_CHK 0define BIT_MASK_CPORIGIDCTS_CHK 0x3define BIT_MAORIGIDCTS_CHK) &                                                   	(((x) & &IT_MASK_BIORIGIDCTS_CHK<< BIT_SHIFT_WLORIGIDCTS_CHK<define BIT_GET_BIORIGIDCTS_CHK) &                                               	(((x) &  BIT_SHIFT_USORIGIDCTS_CHK<<&IT_MASK_BIORIGIDCTS_CHK<d* 2 REG_PACAMCMDOOOOffset 0x00C67 */

#define BIT_SHIECCAM_POLLINGIB_C131<define BIT_STIECCAM_CLRIT_C130<define BIT_CPMFBCAM_CLRIT_C129<d* 2 REG_PACAMCMDOOOOffset 0x00C67 */

#define BIT_SHIECCAM_WEIT_C116<#* 2 REG_PACAMCMDOOOOffset 0x00C67 */

#define BIT_SHIFT_USSECCAM_ADDR_V2 0define BIT_MASK_CPSECCAM_ADDR_V2 0x3ffdefine BIT_SHSECCAM_ADDR_V2) &                                                  	(((x) & &IT_MASK_BISECCAM_ADDR_V2<< BIT_SHIFT_WLSECCAM_ADDR_V2<define BIT_GET_BISECCAM_ADDR_V2) &                                              	(((x) &  BIT_SHIFT_USSECCAM_ADDR_V2<<&IT_MASK_BISECCAM_ADDR_V2<#* 2 REG_PACAMWRITEOOOOffset 0x00C67 */

#define BIT_SHIFT_SDCAMW_DATAI0define BIT_MASK_CPCAMW_DATAI0fffffffL
Ldefine BIT_GECAMW_DATA) & (x) & &IT_MASK_BICAMW_DATA<< BIT_SHIFT_WLCAMW_DATA<define BIT_GET_BICAMW_DATA) & (x) &  BIT_SHIFT_USCAMW_DATA<<&IT_MASK_BICAMW_DATA<#* 2 REG_PACAMREADOOOOffset 0x00C67 */

#define BIT_CPIFT_USCAMR_DATAI0define BIT_MASK_CPCAMR_DATAI0fffffffL
Ldefine BIT_GECAMR_DATA) & (x) & &IT_MASK_BICAMR_DATA<< BIT_SHIFT_WLCAMR_DATA<define BIT_GET_BICAMR_DATA) & (x) &  BIT_SHIFT_USCAMR_DATA<<&IT_MASK_BICAMR_DATA<#* 2 REG_PACAMDBGOOOOffset 0x00C67 */

#define BIT_STIECCAM_INFOIB_C131<define BIT_STIEC_KEYFOUNDIT_C115<#*efine BIT_SHIFT_SDCAMDBGTIEC_TYPE 12define BIT_MASK_CPCAMDBGTIEC_TYPE 0x7define BIT_GECAMDBGTIEC_TYPE) &                                                 	(((x) & &IT_MASK_CPCAMDBGTIEC_TYPE<< BIT_SHIFT_WLCAMDBGTIEC_TYPE<define BIT_GET_BICAMDBGTIEC_TYPE) &                                             	(((x) &  BIT_SHIFT_USCAMDBGTIEC_TYPE<<&IT_MASK_CPCAMDBGTIEC_TYPE<#* 2 REG_PACAMDBGOOOOffset 0x00C67 */

#define BIT_STCAMDBGTEXSTIECTYPE B_C111<d* 2 REG_PACAMDBGOOOOffset 0x00C67 */

#define BIT_STIFT_USCAMDBGTMIC_KEY_IDX 5define BIT_MASK_CPCAMDBGTMIC_KEY_IDX 0x1fdefine BIT_CPCAMDBGTMIC_KEY_IDX) &                                              	(((x) & &IT_MASK_BICAMDBGTMIC_KEY_IDX<< BIT_SHIFT_WLCAMDBGTMIC_KEY_IDX<define BIT_GET_BICAMDBGTMIC_KEY_IDX) &                                          	(((x) &  BIT_SHIFT_USCAMDBGTMIC_KEY_IDX<<&IT_MASK_BICAMDBGTMIC_KEY_IDX<#*efine BIT_SHIFT_SDCAMDBGTIEC_KEY_IDX 0define BIT_MASK_CPCAMDBGTIEC_KEY_IDX 0x1fdefine BIT_CPCAMDBGTIEC_KEY_IDX) &                                              	(((x) & &IT_MASK_BICAMDBGTIEC_KEY_IDX<< BIT_SHIFT_WLCAMDBGTIEC_KEY_IDX<define BIT_GET_BICAMDBGTIEC_KEY_IDX) &                                          	(((x) &  BIT_SHIFT_USCAMDBGTIEC_KEY_IDX<<&IT_MASK_BICAMDBGTIEC_KEY_IDX<d* 2 REG_PASECCFGOOOOffset 0x00C68 */

#define BIT_SHDIS_GCLK_WAPIIT_C115<define BIT_GEDIS_GCLK_AESIT_C114<define BIT_GEDIS_GCLK_TKIPIT_C113<dd 2 REG_PASECCFGOOOOffset 0x00C68 */

#define BIT_SHAES_SEL_QC_1IT_C112<#efine BIT_CPAES_SEL_QC_0 B_C111<d* 2 REG_PASECCFGOOOOffset 0x00C68 */

#define BIT_SHCHK_BMCIT_C19<d* 2 REG_PASECCFGOOOOffset 0x00C68 */

#define BIT_SHCHK_KEYIDIB_C18<define BIT_CPRXBCUSEDCIT_C17<define BIT_CPTXBCUSEDCIT_C16<#efine BIT_CPNOSKMCIT_C15<define BIT_GESKBYA2IB_C14<define BIT_CPRXDEC T_C13<define BIT_CPTXENC T_C12<define BIT_CPRXUHUSEDCIT_C11<define BIT_CPTXUHUSEDCIT_C10&d* 2 REG_PARXFILTER_CATEGORY_1OOOffset 0x00C682*/

#define BIT_CPIFT_USRXFILTER_CATEGORY_1I0define BIT_MASK_CPRXFILTER_CATEGORY_1I0xffdefine BIT_SHRXFILTER_CATEGORY_1) &                                             	(((x) & &IT_MASK_CPRXFILTER_CATEGORY_1<< BIT_SHIFT_WLRXFILTER_CATEGORY_1<define BIT_GET_BIRXFILTER_CATEGORY_1) &                                         	(((x) &  BIT_SHIFT_USRXFILTER_CATEGORY_1<<&IT_MASK_CPRXFILTER_CATEGORY_1<d* 2 REG_PARXFILTER_ACTION_1OOOffset 0x00C683</

#define BIT_SHIFT_USRXFILTER_ACTION_1I0define BIT_MASK_CPRXFILTER_ACTION_1I0xffdefine BIT_SHRXFILTER_ACTION_1) &                                               	(((x) & &IT_MASK_CPRXFILTER_ACTION_1<< BIT_SHIFT_WLRXFILTER_ACTION_1<define BIT_GET_BIRXFILTER_ACTION_1) &                                           	(((x) &  BIT_SHIFT_USRXFILTER_ACTION_1<<&IT_MASK_CPRXFILTER_ACTION_1<d* 2 REG_PARXFILTER_CATEGORY_2OOOffset 0x00C68 */

#define BIT_SHIFT_SDRXFILTER_CATEGORY_2I0define BIT_MASK_CPRXFILTER_CATEGORY_2I0xffdefine BIT_SHRXFILTER_CATEGORY_2) &                                             	(((x) & &IT_MASK_CPRXFILTER_CATEGORY_2<< BIT_SHIFT_WLRXFILTER_CATEGORY_2<define BIT_GET_BIRXFILTER_CATEGORY_2) &                                         	(((x) &  BIT_SHIFT_USRXFILTER_CATEGORY_2& &IT_MASK_CPRXFILTER_CATEGORY_2<d* 2 REG_PARXFILTER_ACTION_2OOOffset 0x00C685& 

#define BIT_SHIFT_USRXFILTER_ACTION_2I0define BIT_MASK_CPRXFILTER_ACTION_2I0xffdefine BIT_SHRXFILTER_ACTION_2) &                                               	(((x) & &IT_MASK_CPRXFILTER_ACTION_2<< BIT_SHIFT_WLRXFILTER_ACTION_2<define BIT_GET_BIRXFILTER_ACTION_2) &                                           	(((x) &  BIT_SHIFT_USRXFILTER_ACTION_2& &IT_MASK_CPRXFILTER_ACTION_2<d* 2 REG_PARXFILTER_CATEGORY_3OOOffset 0x00C686*/

#define BIT_STIFT_USRXFILTER_CATEGORY_3I0define BIT_MASK_CPRXFILTER_CATEGORY_3I0xffdefine BIT_SHRXFILTER_CATEGORY_3) &                                             	(((x) & &IT_MASK_CPRXFILTER_CATEGORY_3<< BIT_SHIFT_WLRXFILTER_CATEGORY_3<define BIT_GET_BIRXFILTER_CATEGORY_3) &                                         	(((x) &  BIT_SHIFT_USRXFILTER_CATEGORY_3& &IT_MASK_CPRXFILTER_CATEGORY_3<d* 2 REG_PARXFILTER_ACTION_3OOOffset 0x00C687& 

#define BIT_SHIFT_USRXFILTER_ACTION_3I0define BIT_MASK_CPRXFILTER_ACTION_3I0xffdefine BIT_SHRXFILTER_ACTION_3) &                                               	(((x) & &IT_MASK_CPRXFILTER_ACTION_3<< BIT_SHIFT_WLRXFILTER_ACTION_3<define BIT_GET_BIRXFILTER_ACTION_3) &                                           	(((x) &  BIT_SHIFT_USRXFILTER_ACTION_3& &IT_MASK_CPRXFILTER_ACTION_3<d* 2 REG_PARXFLSMAP3OOOOffset 0x00C68 */

#define BIT_CPMGTFLS15EN_FWIT_C115<define BIT_GEMGTFLS14EN_FWIT_C114<define BIT_GEMGTFLS13EN_FWIT_C113<define BIT_CPMGTFLS12EN_FWIT_C112<define BIT_CPMGTFLS11EN_FWIT_C111<define BIT_CPMGTFLS10EN_FWIT_C110<define BIT_CPMGTFLS9EN_FWIT_C19<define BIT_CPMGTFLS8EN_FWIT_C18<define BIT_CPMGTFLS7EN_FWIT_C17<define BIT_CPMGTFLS6EN_FWIT_C16<define BIT_CPMGTFLS5EN_FWIT_C15<define BIT_GEMGTFLS4EN_FWIT_C14<define BIT_GEMGTFLS3EN_FWIT_C13<define BIT_CPMGTFLS2EN_FWIT_C12<define BIT_CPMGTFLS1EN_FWIT_C11<define BIT_CPMGTFLS0EN_FWIT_C10&d* 2 REG_PARXFLSMAP4OOOOffset 0x00C68A*/

#define BIT_CPCTRLFLS15EN_FWIT_C115<define BIT_GECTRLFLS14EN_FWIT_C114<define BIT_GECTRLFLS13EN_FWIT_C113<define BIT_CPCTRLFLS12EN_FWIT_C112<define BIT_CPCTRLFLS11EN_FWIT_C111<define BIT_CPCTRLFLS10EN_FWIT_C110<define BIT_CPCTRLFLS9EN_FWIT_C19<define BIT_CPCTRLFLS8EN_FWIT_C18<define BIT_CPCTRLFLS7EN_FWIT_C17<define BIT_CPCTRLFLS6EN_FWIT_C16<define BIT_CPCTRLFLS5EN_FWIT_C15<define BIT_GECTRLFLS4EN_FWIT_C14<define BIT_GECTRLFLS3EN_FWIT_C13<define BIT_CPCTRLFLS2EN_FWIT_C12<define BIT_CPCTRLFLS1EN_FWIT_C11<define BIT_CPCTRLFLS0EN_FWIT_C10&d* 2 REG_PARXFLSMAP5OOOOffset 0x00C68 */

#define BIT_STDATAFLS15EN_FWIT_C115<define BIT_GEDATAFLS14EN_FWIT_C114<define BIT_GEDATAFLS13EN_FWIT_C113<define BIT_CPDATAFLS12EN_FWIT_C112<define BIT_CPDATAFLS11EN_FWIT_C111<define BIT_CPDATAFLS10EN_FWIT_C110<define BIT_CPDATAFLS9EN_FWIT_C19<define BIT_CPDATAFLS8EN_FWIT_C18<define BIT_CPDATAFLS7EN_FWIT_C17<define BIT_CPDATAFLS6EN_FWIT_C16<define BIT_CPDATAFLS5EN_FWIT_C15<define BIT_GEDATAFLS4EN_FWIT_C14<define BIT_GEDATAFLS3EN_FWIT_C13<define BIT_CPDATAFLS2EN_FWIT_C12<define BIT_CPDATAFLS1EN_FWIT_C11<define BIT_CPDATAFLS0EN_FWIT_C10&d* 2 REG_PARXFLSMAP6OOOOffset 0x00C68E*/

#define BIT_STACTIONFLS15EN_FWIT_C115<define BIT_GEACTIONFLS14EN_FWIT_C114<define BIT_GEACTIONFLS13EN_FWIT_C113<define BIT_CPACTIONFLS12EN_FWIT_C112<define BIT_CPACTIONFLS11EN_FWIT_C111<define BIT_CPACTIONFLS10EN_FWIT_C110<define BIT_CPACTIONFLS9EN_FWIT_C19<define BIT_CPACTIONFLS8EN_FWIT_C18<define BIT_CPACTIONFLS7EN_FWIT_C17<define BIT_CPACTIONFLS6EN_FWIT_C16<define BIT_CPACTIONFLS5EN_FWIT_C15<define BIT_GEACTIONFLS4EN_FWIT_C14<define BIT_GEACTIONFLS3EN_FWIT_C13<define BIT_CPACTIONFLS2EN_FWIT_C12<define BIT_CPACTIONFLS1EN_FWIT_C11<define BIT_CPACTIONFLS0EN_FWIT_C10&d* 2 REG_PAWOW_CTRLOOOOffset 0x00C69 */

#define BIT_SHIFT_USPSFSBSSIDSEL_B2B1 #define BIT_MASK_BIPSFSBSSIDSEL_B2B1 0x3define BIT_MAPSFSBSSIDSEL_B2B1) &                                               	(((x) & &IT_MASK_CPPSFSBSSIDSEL_B2B1<< BIT_SHIFT_WLPSFSBSSIDSEL_B2B1<define BIT_GET_BIPSFSBSSIDSEL_B2B1) &                                           	(((x) &  BIT_SHIFT_USPSFSBSSIDSEL_B2B1<<&IT_MASK_CPPSFSBSSIDSEL_B2B1<d* 2 REG_PAWOW_CTRLOOOOffset 0x00C69 */

#define BIT_SHWOWHCIIT_C15<d* 2 REG_PAWOW_CTRLOOOOffset 0x00C69 */

#define BIT_SHPSFSBSSIDSEL_B0IB_C14<#* 2 REG_PAWOW_CTRLOOOOffset 0x00C69 */

#define BIT_SHUWFIT_C13<define BIT_CPMAGIC T_C12<define BIT_CPWOWENIT_C11<define BIT_CPFORCE_WAKEUPIB_C10<d* 2 REG_PANAN_RXPTSF_FILTEROOOffset 0x00C691</

#define BIT_SHCHK_TSF_TAIB_C12<define BIT_CPCHK_TSF_CBSSIDIB_C11<define BIT_CPCHK_TSF__NIT_C10<#* 2 REG_PAPS_RXPINFOOOOOffset 0x00C692*/

#define BIT_CPIFT_USPORTSEL_APS_RXPINFO 5define BIT_MASK_CPPORTSEL_APS_RXPINFO 0x7define BIT_GEPORTSEL_APS_RXPINFO) &                                             	(((x) & &IT_MASK_CPPORTSEL_APS_RXPINFO<< BIT_SHIFT_WLPORTSEL_APS_RXPINFO<define BIT_GET_BIPORTSEL_APS_RXPINFO) &                                         	(((x) &  BIT_SHIFT_USPORTSEL_APS_RXPINFO<<&IT_MASK_CPPORTSEL_APS_RXPINFO<#* 2 REG_PAPS_RXPINFOOOOOffset 0x00C692*/

#define BIT_CPRXCTRLIN0IB_C14<#efine BIT_CPRXMGTIN0IB_C13<#efine BIT_CPRXDATAIN2IB_C12<define BIT_CPRXDATAIN1IB_C11<define BIT_CPRXDATAIN0IT_C10&d* 2 REG_PAWMMPS_UAPSD_TIDOOOffset 0x00C693</

#define BIT_SHWMMPS_UAPSD_TID7IT_C17<define BIT_CPWMMPS_UAPSD_TID6IT_C16<define BIT_CPWMMPS_UAPSD_TID5IT_C15<define BIT_GEWMMPS_UAPSD_TID4IB_C14<#efine BIT_CPWMMPS_UAPSD_TID3IB_C13<#efine BIT_CPWMMPS_UAPSD_TID2IB_C12<define BIT_CPWMMPS_UAPSD_TID1IB_C11<define BIT_CPWMMPS_UAPSD_TID0IT_C10&d* 2 REG_PALPNAV_CTRLOOOOffset 0x00C69 */

#define BIT_SHLPNAV_ENIT_C131<#*efine BIT_SHIFT_SDLPNAV_EARLY 1#define BIT_MASK_BILPNAV_EARLY 0x7fffdefine BIT_STLPNAV_EARLY) &                                                     	(((x) & &IT_MASK_CPLPNAV_EARLY<< BIT_SHIFT_WLLPNAV_EARLY<define BIT_GET_BILPNAV_EARLY) &                                                 	(((x) &  BIT_SHIFT_USLPNAV_EARLY<<&IT_MASK_CPLPNAV_EARLY<#*efine BIT_SHIFT_SDLPNAV_TH 0define BIT_MASK_CPLPNAV_TH 0fffffdefine BIT_STLPNAV_TH) & (x) & &IT_MASK_BILPNAV_TH<< BIT_SHIFT_WLLPNAV_TH)define BIT_MAT_BILPNAV_TH) & (x) &  BIT_SHIFT_USLPNAV_TH& &IT_MASK_BILPNAV_TH<d* 2 REG_PAWKFMCAM_CMDOOOOffset 0x00C69 */

#define BIT_CPWKFCAM_POLLING_V1 T_C131<#efine BIT_CPWKFCAM_CLR_V1 T_C130<d* 2 REG_PAWKFMCAM_CMDOOOOffset 0x00C69 */

#define BIT_CPWKFCAM_WEIT_C116<#* 2 REG_PAWKFMCAM_CMDOOOOffset 0x00C69 */

#define BIT_CPIFT_USWKFCAM_ADDR_V2 8define BIT_MASK_CPWKFCAM_ADDR_V2 0xffdefine BIT_SHWKFCAM_ADDR_V2) &                                                  	(((x) & &IT_MASK_BIWKFCAM_ADDR_V2<< BIT_SHIFT_WLWKFCAM_ADDR_V2<define BIT_MAT_BIWKFCAM_ADDR_V2) &                                              	(((x) &  BIT_SHIFT_USWKFCAM_ADDR_V2<<&IT_MASK_BIWKFCAM_ADDR_V2<#define BIT_CPIFT_USWKFCAM_CAM_NUM_V1 0define BIT_MASK_CPWKFCAM_CAM_NUM_V1 0xffdefine BIT_SHWKFCAM_CAM_NUM_V1) &                                               	(((x) & &IT_MASK_CPWKFCAM_CAM_NUM_V1<< BIT_SHIFT_WLWKFCAM_CAM_NUM_V1<define BIT_MAT_BIWKFCAM_CAM_NUM_V1) &                                           	(((x) &  BIT_SHIFT_USWKFCAM_CAM_NUM_V1<<&IT_MASK_CPWKFCAM_CAM_NUM_V1<#* 2 REG_PAWKFMCAM_RWDOOOOffset 0x00C69 */

#define BIT_STIFT_USWKFMCAM_RWD 0define BIT_MASK_CPWKFMCAM_RWD 0fffffffL
Ldefine BIT_GEWKFMCAM_RWD) &                                                     	(((x) & &IT_MASK_CPWKFMCAM_RWD<< BIT_SHIFT_WLWKFMCAM_RWD<define BIT_MAT_BIWKFMCAM_RWD) &                                                 	(((x) &  BIT_SHIFT_USWKFMCAM_RWD<<&IT_MASK_CPWKFMCAM_RWD<d* 2 REG_PARXFLSMAP0OOOOffset 0x00C6A */

#define BIT_SHMGTFLS15ENIT_C115<define BIT_GEMGTFLS14ENIT_C114<d* 2 REG_PARXFLSMAP0OOOOffset 0x00C6A */

#define BIT_SHMGTFLS13ENIT_C113<define BIT_GEMGTFLS12ENIT_C112<define BIT_CPMGTFLS11ENIT_C111<define BIT_CPMGTFLS10ENIT_C110<define BIT_CPMGTFLS9ENIT_C19<define BIT_CPMGTFLS8ENIT_C18<d* 2 REG_PARXFLSMAP0OOOOffset 0x00C6A */

#define BIT_SHMGTFLS7ENIT_C17)define BIT_CPMGTFLS6ENIT_C16<d* 2 REG_PARXFLSMAP0OOOOffset 0x00C6A */

#define BIT_SHMGTFLS5ENIT_C15<define BIT_GEMGTFLS4ENIB_C14<#efine BIT_CPMGTFLS3ENIT_C13<define BIT_CPMGTFLS2ENIB_C12<define BIT_CPMGTFLS1ENIB_C11<define BIT_CPMGTFLS0ENIT_C10&d* 2 REG_PARXFLSMAP1OOOOffset 0x00C6A2*/

#define BIT_CPCTRLFLS15ENIT_C115<define BIT_GECTRLFLS14ENIT_C114<define BIT_GECTRLFLS13ENIT_C113<define BIT_GECTRLFLS12ENIT_C112<define BIT_CPCTRLFLS11ENIT_C111<define BIT_CPCTRLFLS10ENIT_C110<define BIT_CPCTRLFLS9ENIT_C19<define BIT_CPCTRLFLS8ENIT_C18<define BIT_CPCTRLFLS7ENIT_C17)define BIT_CPCTRLFLS6ENIT_C16<d* 2 REG_PARXFLSMAP1OOOOffset 0x00C6A2*/

#define BIT_CPCTRLFLS5ENIT_C15<define BIT_GECTRLFLS4ENIB_C14<#efine BIT_CPCTRLFLS3ENIT_C13<define BIT_CPCTRLFLS2ENIB_C12<define BIT_CPCTRLFLS1ENIB_C11<define BIT_CPCTRLFLS0ENIT_C10&d* 2 REG_PARXFLSMAPOOOOffset 0x00C6A */

#define BIT_SHDATAFLS15ENIT_C115<define BIT_GEDATAFLS14ENIT_C114<define BIT_GEDATAFLS13ENIT_C113<define BIT_GEDATAFLS12ENIT_C112<define BIT_CPDATAFLS11ENIT_C111<define BIT_CPDATAFLS10ENIT_C110<define BIT_CPDATAFLS9ENIT_C19<define BIT_CPDATAFLS8ENIT_C18<define BIT_CPDATAFLS7ENIT_C17)define BIT_CPDATAFLS6ENIT_C16<define BIT_CPDATAFLS5ENIT_C15<define BIT_GEDATAFLS4ENIB_C14<#efine BIT_CPDATAFLS3ENIT_C13<define BIT_CPDATAFLS2ENIB_C12<define BIT_CPDATAFLS1ENIB_C11<define BIT_CPDATAFLS0ENIT_C10&d* 2 REG_PABCN_PSR_RPMOOOOffset 0x00C6A */

#define BIT_CPIFT_USDTIM_CNTI24define BIT_MASK_CPDTIM_CNTI0xffdefine BIT_SHDTIM_CNT) & (x) & &IT_MASK_BIDTIM_CNT<< BIT_SHIFT_WLDTIM_CNT<define BIT_MAT_BIDTIM_CNT) & (x) &  BIT_SHIFT_USDTIM_CNT<<&IT_MASK_BIDTIM_CNT<#define BIT_CPIFT_USDTIM_PERIOD 1#define BIT_MASK_BIDTIM_PERIOD 0xffdefine BIT_SHDTIM_PERIOD) &                                                     	(((x) & &IT_MASK_CPDTIM_PERIOD<< BIT_SHIFT_WLDTIM_PERIOD<define BIT_MAT_BIDTIM_PERIOD) &                                                 	(((x) &  BIT_SHIFT_USDTIM_PERIOD<<&IT_MASK_CPDTIM_PERIOD<
define BIT_SHDTIMIT_C115<define BIT_GETIMIT_C114<#define BIT_CPIFT_USPS_AID_0 0define BIT_MASK_BIPS_AID_0 0x7ffdefine BIT_SHPS_AID_0) & (x) & &IT_MASK_BIPS_AID_0<< BIT_SHIFT_WLPS_AID_0<define BIT_GET_BIPS_AID_0) & (x) &  BIT_SHIFT_USPS_AID_0<<&IT_MASK_BIPS_AID_0<d* 2 REG_PAFLC_RPCOOOOffset 0x00C6A */

#define BIT_STIFT_USFLC_RPC 0define BIT_MASK_BIFLC_RPC 0xffdefine BIT_SHFLC_RPC) & (x) & &IT_MASK_BIFLC_RPC<< BIT_SHIFT_WLFLC_RPC<define BIT_GET_BIFLC_RPC) & (x) &  BIT_SHIFT_USFLC_RPC<<&IT_MASK_BIFLC_RPC<d* 2 REG_PAFLC_RPCMOOOOffset 0x00C6AD*/

#define BIT_STIFT_USFLC_RPCMI0define BIT_MASK_CPFLC_RPCMI0xffdefine BIT_SHFLC_RPCT) & (x) & &IT_MASK_BIFLC_RPCT<< BIT_SHIFT_WLFLC_RPCT<define BIT_MAT_BIFLC_RPCT) & (x) &  BIT_SHIFT_USFLC_RPCT& &IT_MASK_BIFLC_RPCT<d* 2 REG_PAFLC_PTS(OOOffset 0x00C6AE*/

#define BIT_STCMFIB_C12<define BIT_CPCCFIB_C11<define BIT_CPCDFIT_C10&d* 2 REG_PAFLC_TRPCOOOOffset 0x00C6AF*/

#define BIT_STFLC_RPCT_V1 T_C17)define BIT_CPMODEIT_C16<d*efine BIT_STIFT_USTRPCD 0define BIT_MASK_CPTRPCD 0x3fdefine BIT_STTRPCD) & (x) & &IT_MASK_BITRPCD<< BIT_SHIFT_WLTRPCD<define BIT_MAT_BITRPCD) & (x) &  BIT_SHIFT_USTRPCD<<&IT_MASK_BITRPCD<d* 2 REG_PARXPKTMON_CTRLOOOffset 0x00C6B */

#define BIT_SHIFT_USRXBKQPKT_SEQ 20define BIT_MASK_CPRXBKQPKT_SEQ 0xfdefine BIT_SHRXBKQPKT_SEQ) &                                                    	(((x) & &IT_MASK_BIRXBKQPKT_SEQ<< BIT_SHIFT_WLRXBKQPKT_SEQ<define BIT_MAT_BIRXBKQPKT_SEQ) &                                                	(((x) &  BIT_SHIFT_USRXBKQPKT_SEQ<<&IT_MASK_BIRXBKQPKT_SEQ<#define BIT_SHIFT_USRXBEQPKT_SEQ 1#define BIT_MASK_BIRXBEQPKT_SEQ 0xfdefine BIT_SHRXBEQPKT_SEQ) &                                                    	(((x) & &IT_MASK_BIRXBEQPKT_SEQ<< BIT_SHIFT_WLRXBEQPKT_SEQ<define BIT_MAT_BIRXBEQPKT_SEQ) &                                                	(((x) &  BIT_SHIFT_USRXBEQPKT_SEQ<<&IT_MASK_BIRXBEQPKT_SEQ<#define BIT_SHIFT_USRXVIQPKT_SEQ 12define BIT_MASK_CPRXVIQPKT_SEQ 0xfdefine BIT_SHRXVIQPKT_SEQ) &                                                    	(((x) & &IT_MASK_BIRXVIQPKT_SEQ<< BIT_SHIFT_WLRXVIQPKT_SEQ<define BIT_MAT_BIRXVIQPKT_SEQ) &                                                	(((x) &  BIT_SHIFT_USRXVIQPKT_SEQ<<&IT_MASK_BIRXVIQPKT_SEQ<#define BIT_SHIFT_USRXVOQPKT_SEQ 8define BIT_MASK_CPRXVOQPKT_SEQ 0xfdefine BIT_SHRXVOQPKT_SEQ) &                                                    	(((x) & &IT_MASK_BIRXVOQPKT_SEQ<< BIT_SHIFT_WLRXVOQPKT_SEQ<define BIT_MAT_BIRXVOQPKT_SEQ) &                                                	(((x) &  BIT_SHIFT_USRXVOQPKT_SEQ<<&IT_MASK_BIRXVOQPKT_SEQ<#define BIT_SHRXBKQPKT_ERR T_C17<define BIT_CPRXBEQPKT_ERR T_C16<define BIT_CPRXVIQPKT_ERR T_C15<define BIT_CPRXVOQPKT_ERR T_C14<define BIT_CPRXDMA_MON_ENIB_C12<define BIT_CPRXPKT_MON_RSMIT_C11<define BIT_CPRXPKT_MON_ENIT_C10&d* 2 REG_PASTATE_MONOOOOffset 0x00C6B */

#define BIT_SHIFT_SDSTATE_SEL 24define BIT_MASK_CPSTATE_SEL 0x1fdefine BIT_CPSTATE_SEL) & (x) & &IT_MASK_BISTATE_SEL<< BIT_SHIFT_WLSTATE_SEL<define BIT_MAT_BISTATE_SEL) & (x) &  BIT_SHIFT_USSTATE_SEL<<&IT_MASK_BISTATE_SEL<#define BIT_SHIFT_SDSTATE_INFO 8define BIT_MASK_CPSTATE_INFO 0xffdefine BIT_SHSTATE_INFO) & (x) & &IT_MASK_BISTATE_INFO<< BIT_SHIFT_WLSTATE_INFO<define BIT_MAT_BISTATE_INFO) &                                                  	(((x) &  BIT_SHIFT_USSTATE_INFO<<&IT_MASK_BISTATE_INFO<#define BIT_SHUPD_NXUSSTATE T_C17<d* 2 REG_PASTATE_MONOOOOffset 0x00C6B */

#define BIT_SHIFT_SDCURSSTATE 0define BIT_MASK_CPCURSSTATE 0x7fdefine BIT_CPCURSSTATE) & (x) & &IT_MASK_BICURSSTATE<< BIT_SHIFT_WLCURSSTATE<define BIT_MAT_BICURSSTATE) & (x) &  BIT_SHIFT_USCURSSTATE<<&IT_MASK_BICURSSTATE<d* 2 REG_PAERROR_MONOOOOffset 0x00C6B */

#define BIT_CPMACRXAERR_1IT_C117)define BIT_CPMACRXAERR_0 B_C116<define BIT_CPMACTXAERR_3IB_C13<#efine BIT_CPMACTXAERR_2IB_C12<define BIT_CPMACTXAERR_1IB_C11<define BIT_CPMACTXAERR_0IT_C10&d* 2 REG_PASEARCHPMACIDOOOffset 0x00C6B */

#define BIT_STEN_TXRPTBUF_CLCIT_C131<#*efine BIT_SHIFT_SDINFO_INDEX_OFFSET 1#define BIT_MASK_BIINFO_INDEX_OFFSET 0x1fffdefine BIT_CPINFO_INDEX_OFFSET) &                                               	(((x) & &IT_MASK_CPINFO_INDEX_OFFSET<< BIT_SHIFT_WLINFO_INDEX_OFFSET<define BIT_MAT_BIINFO_INDEX_OFFSET) &                                           	(((x) &  BIT_SHIFT_USINFO_INDEX_OFFSET<<&IT_MASK_CPINFO_INDEX_OFFSET<d* 2 REG_PASEARCHPMACIDOOOffset 0x00C6B */

#define BIT_STWMACPSRCHPFIFOFULLIT_C115<d* 2 REG_PASEARCHPMACIDOOOffset 0x00C6B */

#define BIT_STDIS_INFOSRCHIT_C114<define BIT_GEDISABLE_B0IB_C113<ddefine BIT_SHIFT_SDINFO_ADDR_OFFSET 0define BIT_MASK_BIINFO_ADDR_OFFSET 0x1fffdefine BIT_CPINFO_ADDR_OFFSET) &                                                	(((x) & &IT_MASK_CPINFO_ADDR_OFFSET<< BIT_SHIFT_WLINFO_ADDR_OFFSET<define BIT_MAT_BIINFO_ADDR_OFFSET) &                                            	(((x) &  BIT_SHIFT_USINFO_ADDR_OFFSET<<&IT_MASK_CPINFO_ADDR_OFFSET<d* 2 REG_PABM_COEX_TABLEOOOffset 0x00C6C */

#define BIT_SHPRIASK_BIRXIRESPIB_C1126<#efine BIT_CPPRIASK_BIRXOFDM B_C1125<#efine BIT_CPPRIASK_BIRXCCCIB_C1124<#define BIT_CPIFT_USPRIASK_BITXAC (117<&ICPU_OPT_WIDTH)define BIT_MASK_CPPRIASK_BITXAC 0x7fdefine BIT_CPPRIASK_BITXAC) &                                                   	(((x) & &IT_MASK_CPPRIASK_BITXAC<< BIT_SHIFT_WLPRIASK_BITXAC<define BIT_GET_BIPRIASK_BITXAC) &                                               	(((x) &  BIT_SHIFT_USPRIASK_BITXAC<<&IT_MASK_CPPRIASK_BITXAC<#define BIT_CPIFT_USPRIASK_BINAV (109<&ICPU_OPT_WIDTH)define BIT_MASK_CPPRIASK_BINAV 0xffdefine BIT_CPPRIASK_BINAV) &                                                    	(((x) & &IT_MASK_CPPRIASK_BINAV<< BIT_SHIFT_WLPRIASK_BINAV<define BIT_GET_BIPRIASK_BINAV) &                                                	(((x) &  BIT_SHIFT_USPRIASK_BINAV<<&IT_MASK_CPPRIASK_BINAV<ddefine BIT_CPPRIASK_BICCCIB_C1108<#efine BIT_CPPRIASK_BIOFDM B_C1107<#efine BIT_CPPRIASK_BIRTY B_C1106<d*efine BIT_STIFT_USPRIASK_BINUM (102<&ICPU_OPT_WIDTH)define BIT_MASK_CPPRIASK_BINUM 0xfdefine BIT_SHPRIASK_BINUM) &                                                    	(((x) & &IT_MASK_CPPRIASK_BINUM<< BIT_SHIFT_WLPRIASK_BINUM<define BIT_GET_BIPRIASK_BINUM) &                                                	(((x) &  BIT_SHIFT_USPRIASK_BINUM& &IT_MASK_CPPRIASK_BINUM<#define BIT_CPIFT_USPRIASK_BITYPE (98<&ICPU_OPT_WIDTH)define BIT_MASK_CPPRIASK_BITYPE 0xfdefine BIT_SHPRIASK_BITYPE) &                                                   	(((x) & &IT_MASK_CPPRIASK_BITYPE<< BIT_SHIFT_WLPRIASK_BITYPE<define BIT_GET_BIPRIASK_BITYPE) &                                               	(((x) &  BIT_SHIFT_USPRIASK_BITYPE<<&IT_MASK_CPPRIASK_BITYPE<ddefine BIT_GEOOB B_C197<define BIT_CPANT_SEL T_C196<d*efine BIT_STIFT_USBREAK_TABLE_2I(80<&ICPU_OPT_WIDTH)define BIT_MASK_CPBREAK_TABLE_2I0fffffdefine BIT_STBREAK_TABLE_2) &                                                   	(((x) & &IT_MASK_CPBREAK_TABLE_2<< BIT_SHIFT_WLBREAK_TABLE_2<define BIT_GET_BIBREAK_TABLE_2) &                                               	(((x) &  BIT_SHIFT_USBREAK_TABLE_2<<&IT_MASK_CPBREAK_TABLE_2<d*efine BIT_STIFT_USBREAK_TABLE_1 (64<&ICPU_OPT_WIDTH)define BIT_MASK_CPBREAK_TABLE_1 0fffffdefine BIT_STBREAK_TABLE_1) &                                                   	(((x) & &IT_MASK_CPBREAK_TABLE_1<< BIT_SHIFT_WLBREAK_TABLE_1<define BIT_GET_BIBREAK_TABLE_1) &                                               	(((x) &  BIT_SHIFT_USBREAK_TABLE_1& &IT_MASK_CPBREAK_TABLE_1<#define BIT_SHIFT_SDCOEX_TABLE_2I(32<&ICPU_OPT_WIDTH)define BIT_MASK_CPCOEX_TABLE_2I0fffffffL
Ldefine BIT_GECOEX_TABLE_2) &                                                    	(((x) & &IT_MASK_CPCOEX_TABLE_2<< BIT_SHIFT_WLCOEX_TABLE_2<define BIT_MAT_BICOEX_TABLE_2) &                                                	(((x) &  BIT_SHIFT_USCOEX_TABLE_2<<&IT_MASK_CPCOEX_TABLE_2<#define BIT_SHIFT_SDCOEX_TABLE_1 0define BIT_MASK_CPCOEX_TABLE_1 0fffffffL
Ldefine BIT_GECOEX_TABLE_1) &                                                    	(((x) & &IT_MASK_CPCOEX_TABLE_1<< BIT_SHIFT_WLCOEX_TABLE_1<define BIT_MAT_BICOEX_TABLE_1) &                                                	(((x) &  BIT_SHIFT_USCOEX_TABLE_1& &IT_MASK_CPCOEX_TABLE_1<d* 2 REG_PARXCMD_0OOOOffset 0x00C6D */

#define BIT_SHRXCMD_ENIT_C131<#*efine BIT_SHIFT_SDRXCMD_INFO 0define BIT_MASK_CPRXCMD_INFO 0x7fffffL
Ldefine BIT_GERXCMD_INFO) & (x) & &IT_MASK_BIRXCMD_INFO<< BIT_SHIFT_WLRXCMD_INFO<define BIT_MAT_BIRXCMD_INFO) &                                                  	(((x) &  BIT_SHIFT_USRXCMD_INFO<<&IT_MASK_BIRXCMD_INFO<d* 2 REG_PARXCMD_1OOOOffset 0x00C6D */

#define BIT_SHIFT_SDRXCMD_PRD 0define BIT_MASK_CPRXCMD_PRD 0fffffdefine BIT_STRXCMD_PRD) & (x) & &IT_MASK_BIRXCMD_PRD<< BIT_SHIFT_WLRXCMD_PRD<define BIT_MAT_BIRXCMD_PRD) & (x) &  BIT_SHIFT_USRXCMD_PRD<<&IT_MASK_BIRXCMD_PRD<dd 2 REG_PAWMACPRESPITXINFOOOOffset 0x00C6D */

#define BIT_CPIFT_USWMACPRESPIMFB 25define BIT_MASK_CPWMACPRESPIMFB 0x7fdefine BIT_CPWMACPRESPIMFB) &                                                   	(((x) & &IT_MASK_CPWMACPRESPIMFB<< BIT_SHIFT_WLWMACPRESPIMFB<define BIT_MAT_BIWMACPRESPIMFB) &                                               	(((x) &  BIT_SHIFT_USWMACPRESPIMFB<<&IT_MASK_CPWMACPRESPIMFB<#define BIT_CPIFT_USWMACPANTINF_SEL 23define BIT_MASK_CPWMACPANTINF_SEL 0x3define BIT_MAWMACPANTINF_SEL) &                                                 	(((x) & &IT_MASK_CPWMACPANTINF_SEL<< BIT_SHIFT_WLWMACPANTINF_SEL<define BIT_MAT_BIWMACPANTINF_SEL) &                                             	(((x) &  BIT_SHIFT_USWMACPANTINF_SEL<<&IT_MASK_CPWMACPANTINF_SEL<#define BIT_CPIFT_USWMACPANTSEL_SEL 21define BIT_MASK_CPWMACPANTSEL_SEL 0x3define BIT_MAWMACPANTSEL_SEL) &                                                 	(((x) & &IT_MASK_CPWMACPANTSEL_SEL<< BIT_SHIFT_WLWMACPANTSEL_SEL<define BIT_MAT_BIWMACPANTSEL_SEL) &                                             	(((x) &  BIT_SHIFT_USWMACPANTSEL_SEL<<&IT_MASK_CPWMACPANTSEL_SEL<dd 2 REG_PAWMACPRESPITXINFOOOOffset 0x00C6D */

#define BIT_CPIFT_USRAWMACPRESPITXPOWER 18define BIT_MASK_CPRAWMACPRESPITXPOWER 0x7define BIT_GERHWMACPRESPITXPOWER) &                                             	(((x) & &IT_MASK_BIRHWMACPRESPITXPOWER<< BIT_SHIFT_WLRHWMACPRESPITXPOWER<define BIT_MAT_BIRHWMACPRESPITXPOWER) &                                         	(((x) &  BIT_SHIFT_USRHWMACPRESPITXPOWER<<&IT_MASK_BIRHWMACPRESPITXPOWER<dd 2 REG_PAWMACPRESPITXINFOOOOffset 0x00C6D */

#define BIT_CPIFT_USWMACPRESPITXANT 0define BIT_MASK_CPWMACPRESPITXANT 0x3ffffdefine BIT_SHWMACPRESPITXANT) &                                                 	(((x) & &IT_MASK_CPWMACPRESPITXANT<< BIT_SHIFT_WLWMACPRESPITXANT<define BIT_MAT_BIWMACPRESPITXANT) &                                             	(((x) &  BIT_SHIFT_USWMACPRESPITXANT<<&IT_MASK_CPWMACPRESPITXANT<d* 2 REG_PABBPSFSCTRLOOOOffset 0x00C6D */

#define BIT_STCTL_IDLE_CLR_CSI_RPMIT_C131<#* 2 REG_PABBPSFSCTRLOOOOffset 0x00C6D */

#define BIT_STWMACPUSE_NDPARATE T_C130<d*efine BIT_CPIFT_USWMACPCSI_RATE 24define BIT_MASK_CPWMACPCSI_RATE 0x3fdefine BIT_STWMACPCSI_RATE) &                                                   	(((x) & &IT_MASK_CPWMACPCSI_RATE<< BIT_SHIFT_WLWMACPCSI_RATE<define BIT_MAT_BIWMACPCSI_RATE) &                                               	(((x) &  BIT_SHIFT_USWMACPCSI_RATE<<&IT_MASK_CPWMACPCSI_RATE<#define BIT_CPIFT_USWMACPRESPITXRATE 1#define BIT_MASK_BIWMACPRESPITXRATE 0xffdefine BIT_SHWMACPRESPITXRATE) &                                                	(((x) & &IT_MASK_CPWMACPRESPITXRATE<< BIT_SHIFT_WLWMACPRESPITXRATE<define BIT_MAT_BIWMACPRESPITXRATE) &                                            	(((x) &  BIT_SHIFT_USWMACPRESPITXRATE<<&IT_MASK_CPWMACPRESPITXRATE<d* 2 REG_PABBPSFSCTRLOOOOffset 0x00C6D */

#define BIT_STBBPSFSMPDUCHKENIT_C15<d* 2 REG_PABBPSFSCTRLOOOOffset 0x00C6D */

#define BIT_STBBPSFSMHCHKENIT_C14<define BIT_GEBBPSFSERRCHKENIT_C13<ddefine BIT_SHIFT_SDBBPSFSERRTHR 0define BIT_MASK_CPBBPSFSERRTHR 0x7define BIT_GEBBPSFSERRTHR) &                                                    	(((x) & &IT_MASK_CPBBPSFSERRTHR<< BIT_SHIFT_WLBBPSFSERRTHR<define BIT_GET_BIBBPSFSERRTHR) &                                                	(((x) &  BIT_SHIFT_USBBPSFSERRTHR<<&IT_MASK_CPBBPSFSERRTHR<#* 2 REG_PAP2PIRXIBCN_NOAOOOffset 0x00C6E */

#define BIT_SHNOA_PARSER_ENIT_C115<d* 2 REG_PAP2PIRXIBCN_NOAOOOffset 0x00C6E */

#define BIT_SHBSSID_SEL T_C114<d* 2 REG_PAP2PIRXIBCN_NOAOOOffset 0x00C6E */

#define BIT_SHIFT_USP2PIOUIITYPE 0define BIT_MASK_CPP2PIOUIITYPE 0xffdefine BIT_CPP2PIOUIITYPE) &                                                    	(((x) & &IT_MASK_CPP2PIOUIITYPE<< BIT_SHIFT_WLP2PIOUIITYPE<define BIT_GET_BIP2PIOUIITYPE) &                                                	(((x) &  BIT_SHIFT_USP2PIOUIITYPE<<&IT_MASK_CPP2PIOUIITYPE<d* 2 REG_PAASSOCIATED_BFMER0PINFOOOffset 0x00C6E */

#define BIT_SHIFT_SDRAWMACPTXCSI_AID0I(48<&ICPU_OPT_WIDTH)define BIT_MASK_CPRAWMACPTXCSI_AID0I0x1ffdefine BIT_GERHWMACPTXCSI_AID0) &                                               	(((x) & &IT_MASK_CPRHWMACPTXCSI_AID0<< BIT_SHIFT_WLRHWMACPTXCSI_AID0<define BIT_MAT_BIRHWMACPTXCSI_AID0) &                                           	(((x) &  BIT_SHIFT_USRHWMACPTXCSI_AID0<<&IT_MASK_CPRHWMACPTXCSI_AID0<#define BIT_CPIFT_USRHWMACPSOUNDING_RXADD_R0 0define BIT_MASK_BIRHWMACPSOUNDING_RXADD_R0 0fffffffL
ffL
Ldefine BIT_GERHWMACPSOUNDING_RXADD_R0) &                                        	(((x) & &IT_MASK_BIRHWMACPSOUNDING_RXADD_R0&                             	((< BIT_SHIFT_WLRHWMACPSOUNDING_RXADD_R0&define BIT_MAT_BIRHWMACPSOUNDING_RXADD_R0) &                                    	(((x) &  BIT_SHIFT_USRHWMACPSOUNDING_RXADD_R0& &                         	((<T_MASK_CPRHWMACPSOUNDING_RXADD_R0&d* 2 REG_PAASSOCIATED_BFMER1PINFOOOffset 0x00C6E */

#define BIT_STIFT_USRHWMACPTXCSI_AID1I(48<&ICPU_OPT_WIDTH)define BIT_MASK_CPRAWMACPTXCSI_AID1I0x1ffdefine BIT_GERHWMACPTXCSI_AID1) &                                               	(((x) & &IT_MASK_CPRHWMACPTXCSI_AID1<< BIT_SHIFT_WLRHWMACPTXCSI_AID1<define BIT_GET_BIRHWMACPTXCSI_AID1) &                                           	(((x) &  BIT_SHIFT_USRHWMACPTXCSI_AID1& &IT_MASK_CPRHWMACPTXCSI_AID1<#define BIT_CPIFT_USRHWMACPSOUNDING_RXADD_R1I0define BIT_MASK_CPRHWMACPSOUNDING_RXADD_R1I0fffffffL
ffL
Ldefine BIT_GERHWMACPSOUNDING_RXADD_R1) &                                        	(((x) & &IT_MASK_BIRHWMACPSOUNDING_RXADD_R1&                             	((< BIT_SHIFT_WLRHWMACPSOUNDING_RXADD_R1<define BIT_GET_BIRHWMACPSOUNDING_RXADD_R1) &                                    	(((x) &  BIT_SHIFT_USRHWMACPSOUNDING_RXADD_R1& &                         	((<T_MASK_CPRHWMACPSOUNDING_RXADD_R1<#* 2 REG_PATX_CSI_RPM_PARAM_BW20OOffset 0x00C6F */

#define BIT_SHIFT_SDRAWMACPBFINFO_20M_1 1#define BIT_MASK_BIRAWMACPBFINFO_20M_1 0ffffdefine BIT_GERHWMACPBFINFO_20M_1) &                                             	(((x) & &IT_MASK_BIRHWMACPBFINFO_20M_1<< BIT_SHIFT_WLRHWMACPBFINFO_20M_1<define BIT_GET_BIRHWMACPBFINFO_20M_1) &                                         	(((x) &  BIT_SHIFT_USRHWMACPBFINFO_20M_1<<&IT_MASK_BIRHWMACPBFINFO_20M_1<#define BIT_SHIFT_SDRAWMACPBFINFO_20M_0 0define BIT_MASK_BIRHWMACPBFINFO_20M_0 0ffffdefine BIT_GERHWMACPBFINFO_20M_0) &                                             	(((x) & &IT_MASK_BIRHWMACPBFINFO_20M_0<< BIT_SHIFT_WLRHWMACPBFINFO_20M_0<define BIT_GET_BIRHWMACPBFINFO_20M_0) &                                         	(((x) &  BIT_SHIFT_USRHWMACPBFINFO_20M_0<<&IT_MASK_CPRHWMACPBFINFO_20M_0<d* 2 REG_PATX_CSI_RPM_PARAM_BW40OOffset 0x00C6F */

#define BIT_CPIFT_USWMACPRESPIANTCD 0define BIT_MASK_CPWMACPRESPIANTCD 0xfdefine BIT_SHWMACPRESPIANTCD) &                                                 	(((x) & &IT_MASK_CPWMACPRESPIANTCD<< BIT_SHIFT_WLWMACPRESPIANTCD<define BIT_MAT_BIWMACPRESPIANTCD) &                                             	(((x) &  BIT_SHIFT_USWMACPRESPIANTCD<<&IT_MASK_CPWMACPRESPIANTCD<d* 2 REG_PAMACID1OOOOffset 0x00C70 */

#define BIT_SHIFT_USMACID1 0define BIT_MASK_CPMACID1 0fffffffL
ffL
Ldefine BIT_GEMACID1) & (x) & &IT_MASK_BIMACID1<< BIT_SHIFT_WLMACID1<define BIT_MAT_BIMACID1) & (x) &  BIT_SHIFT_USMACID1<<&IT_MASK_BIMACID1<d* 2 REG_PABSSID1OOOOffset 0x00C70 */

#define BIT_CPIFT_USBSSID1 0define BIT_MASK_CPBSSID1 0fffffffL
ffL
Ldefine BIT_GEBSSID1) & (x) & &IT_MASK_BIBSSID1<< BIT_SHIFT_WLBSSID1<define BIT_GET_BIBSSID1) & (x) &  BIT_SHIFT_USBSSID1<<&IT_MASK_BIBSSID1<d* 2 REG_PABCN_PSR_RPM1OOOffset 0x00C71 */

#define BIT_SHIFT_USDTIM_CNT1I24define BIT_MASK_CPDTIM_CNT1 0xffdefine BIT_SHDTIM_CNT1) & (x) & &IT_MASK_BIDTIM_CNT1<< BIT_SHIFT_WLDTIM_CNT1<define BIT_GET_BIDTIM_CNT1) & (x) &  BIT_SHIFT_USDTIM_CNT1& &IT_MASK_BIDTIM_CNT1<#define BIT_CPIFT_USDTIM_PERIOD1 1#define BIT_MASK_BIDTIM_PERIOD1 0xffdefine BIT_SHDTIM_PERIOD1) &                                                    	(((x) & &IT_MASK_CPDTIM_PERIOD1<< BIT_SHIFT_WLDTIM_PERIOD1<define BIT_GET_BIDTIM_PERIOD1) &                                                	(((x) &  BIT_SHIFT_USDTIM_PERIOD1& &IT_MASK_BIDTIM_PERIOD1<ddefine BIT_SHDTIM1IT_C115<define BIT_SHTIM1IT_C114<#define BIT_CPIFT_USPS_AID_1 0define BIT_MASK_CPPS_AID_1 0x7ffdefine BIT_SHPS_AID_1) & (x) & &IT_MASK_BIPS_AID_1<< BIT_SHIFT_WLPS_AID_1<define BIT_GET_BIPS_AID_1) & (x) &  BIT_SHIFT_USPS_AID_1<<&IT_MASK_CPPS_AID_1<d* 2 REG_PAASSOCIATED_BFMEE_SELOOffset 0x00C71 */

#define BIT_SHTXUSER_ID1IB_C125<#*efine BIT_SHIFT_SDAID1I1#define BIT_MASK_BIAID1I0x1ffdefine BIT_GEAID1) & (x) & &IT_MASK_BIAID1<< BIT_SHIFT_WLAID1<define BIT_GET_BIAID1) & (x) &  BIT_SHIFT_USAID1& &IT_MASK_CPAID1<#define BIT_CPTXUSER_ID0IB_C19<#*efine BIT_SHIFT_SDAID0 0define BIT_MASK_BIAID0I0x1ffdefine BIT_GEAID0) & (x) & &IT_MASK_BIAID0<< BIT_SHIFT_WLAID0<define BIT_MAT_BIAID0) & (x) &  BIT_SHIFT_USAID0& &IT_MASK_BIAID0<d* 2 REG_PASND_PTCLICTRLOOOffset 0x00C71 */

#define BIT_CPIFT_USNDPIRXISTANDBYHTIMERI24define BIT_MASK_CPNDPIRXISTANDBYHTIMERI0xffdefine BIT_SHNDPIRXISTANDBYHTIMER) &                                            	(((x) & &IT_MASK_CPNDPIRXISTANDBYHTIMER&                                 	((< BIT_SHIFT_WLNDPIRXISTANDBYHTIMER&define BIT_MAT_BINDPIRXISTANDBYHTIMER) &                                        	(((x) &  BIT_SHIFT_USNDPIRXISTANDBYHTIMER& &                             	((<T_MASK_CPNDPIRXISTANDBYHTIMER&#define BIT_SHIFT_SDCSI_RPM_OFFSET_HT 1#define BIT_MASK_BICSI_RPM_OFFSET_HT 0xffdefine BIT_SHCSI_RPM_OFFSET_HT) &                                               	(((x) & &IT_MASK_CPCSI_RPM_OFFSET_HT<< BIT_SHIFT_WLCSI_RPM_OFFSET_HT<define BIT_MAT_BICSI_RPM_OFFSET_HT) &                                           	(((x) &  BIT_SHIFT_USCSI_RPM_OFFSET_HT<<&IT_MASK_CPCSI_RPM_OFFSET_HT<d* 2 REG_PASND_PTCLICTRLOOOffset 0x00C71 */

#define BIT_CPIFT_USRHWMACPVHT_CATEGORY 8define BIT_MASK_CPRHWMACPVHT_CATEGORY 0xffdefine BIT_SHRHWMACPVHT_CATEGORY) &                                             	(((x) & &IT_MASK_BIRHWMACPVHT_CATEGORY<< BIT_SHIFT_WLRHWMACPVHT_CATEGORY<define BIT_GET_BIRHWMACPVHT_CATEGORY) &                                         	(((x) &  BIT_SHIFT_USRHWMACPVHT_CATEGORY<<&IT_MASK_BIRHWMACPVHT_CATEGORY<d* 2 REG_PASND_PTCLICTRLOOOffset 0x00C71 */

#define BIT_CPRTWMACPUSE_NSTSIT_C17<define BIT_CPREDISABLE_CHECKPVHTSIGB_CRC T_C16<define BIT_CPREDISABLE_CHECKPVHTSIGA_CRC T_C15<define BIT_CPRHWMACPBFPARAM_SEL T_C14<define BIT_CPRPWMACPCSISEQ_SEL T_C13<#efine BIT_CPRPWMACPCSI_WITHHTC_ENIB_C12<define BIT_CPRPWMACPHUSNDPA_ENIB_C11<define BIT_CPRHWMACPVHT_NDPA_ENIB_C10<d* 2 REG_PANS_ARPSCTRLOOOOffset 0x00C72 */

#define BIT_SHRHWMACPNSARPSRSPENIT_C115<define BIT_GERHWMACPNSARPSRARPIB_C19<#efine BIT_GERHWMACPNSARPSRIPV6IT_C18<#define BIT_SHIFT_SDRAWMACPNSARPSMODEN #define BIT_MASK_BIRAWMACPNSARPSMODEN 0x3define BIT_MARAWMACPNSARPSMODEN) &                                              	(((x) & &IT_MASK_BIRHWMACPNSARPSMODEN<< BIT_SHIFT_WLRHWMACPNSARPSMODEN<define BIT_GET_BIRHWMACPNSARPSMODEN) &                                          	(((x) &  BIT_SHIFT_USRHWMACPNSARPSMODEN<<&IT_MASK_BIRHWMACPNSARPSMODEN<#define BIT_SHIFT_SDRAWMACPNSARPSRSPFTP 4define BIT_MASK_CPRAWMACPNSARPSRSPFTP 0x3define BIT_MARAWMACPNSARPSRSPFTP) &                                             	(((x) & &IT_MASK_BIRHWMACPNSARPSRSPFTP<< BIT_SHIFT_WLRHWMACPNSARPSRSPFTP<define BIT_GET_BIRHWMACPNSARPSRSPFTP) &                                         	(((x) &  BIT_SHIFT_USRHWMACPNSARPSRSPFTP<<&IT_MASK_BIRHWMACPNSARPSRSPFTP<#define BIT_SHIFT_SDRAWMACPNSARPSRSPSEC 0define BIT_MASK_BIRAWMACPNSARPSRSPSEC 0xfdefine BIT_SHRAWMACPNSARPSRSPSEC) &                                             	(((x) & &IT_MASK_BIRHWMACPNSARPSRSPSEC<< BIT_SHIFT_WLRHWMACPNSARPSRSPSEC<define BIT_GET_BIRHWMACPNSARPSRSPSEC) &                                         	(((x) &  BIT_SHIFT_USRHWMACPNSARPSRSPSEC<<&IT_MASK_BIRHWMACPNSARPSRSPSEC<d* 2 REG_PANS_ARPSINFOOOOOffset 0x00C72 */

#define BIT_SHREQ_IS_MCNSIT_C123<#efine BIT_CPREQ_IS_UCNSIT_C122<define BIT_CPREQ_IS_USNSIT_C121<define BIT_CPREQ_IS_ARPIB_C120<define BIT_MAEXPRSPIMH_WITHQC T_C119<#*efine BIT_SHIFT_SDEXPRSPISECTYPE 1#define BIT_MASK_BIEXPRSPISECTYPE 0x7define BIT_GEEXPRSPISECTYPE) &                                                  	(((x) & &IT_MASK_BIEXPRSPISECTYPE<< BIT_SHIFT_WLEXPRSPISECTYPE<define BIT_GET_BIEXPRSPISECTYPE) &                                              	(((x) &  BIT_SHIFT_USEXPRSPISECTYPE<<&IT_MASK_BIEXPRSPISECTYPE<#*efine BIT_SHIFT_SDEXPRSPICHKSM_7_TO_0 8define BIT_MASK_CPEXPRSPICHKSM_7_TO_0 0xffdefine BIT_SHEXPRSPICHKSM_7_TO_0) &                                             	(((x) & &IT_MASK_BIEXPRSPICHKSM_7_TO_0<< BIT_SHIFT_WLEXPRSPICHKSM_7_TO_0<define BIT_GET_BIEXPRSPICHKSM_7_TO_0) &                                         	(((x) &  BIT_SHIFT_USEXPRSPICHKSM_7_TO_0<<&IT_MASK_BIEXPRSPICHKSM_7_TO_0<#*efine BIT_SHIFT_SDEXPRSPICHKSM_15_TO_8 0define BIT_MASK_BIEXPRSPICHKSM_15_TO_8 0xffdefine BIT_SHEXPRSPICHKSM_15_TO_8) &                                            	(((x) & &IT_MASK_CPEXPRSPICHKSM_15_TO_8&                                 	((< BIT_SHIFT_WLEXPRSPICHKSM_15_TO_8&define BIT_GET_BIEXPRSPICHKSM_15_TO_8) &                                        	(((x) &  BIT_SHIFT_USEXPRSPICHKSM_15_TO_8& &                             	((<T_MASK_CPEXPRSPICHKSM_15_TO_8&d* 2 REG_PABEAMFORMING_INFO_NSARPSV1OOffset 0x00C72 */

#define BIT_CPIFT_USWMACPARPIP 0define BIT_MASK_CPWMACPARPIP 0fffffffL
Ldefine BIT_GEWMACPARPIP) & (x) & &IT_MASK_BIWMACPARPIP<< BIT_SHIFT_WLWMACPARPIP<define BIT_MAT_BIWMACPARPIP) &                                                  	(((x) &  BIT_SHIFT_USWMACPARPIP<<&IT_MASK_BIWMACPARPIP<d* 2 REG_PABEAMFORMING_INFO_NSARPOOffset 0x00C72 */

#define BIT_STIFT_USBEAMFORMING_INFO 0define BIT_MASK_CPBEAMFORMING_INFO 0fffffffL
Ldefine BIT_GEBEAMFORMING_INFO) &                                                	(((x) & &IT_MASK_CPBEAMFORMING_INFO<< BIT_SHIFT_WLBEAMFORMING_INFO<define BIT_GET_BIBEAMFORMING_INFO) &                                            	(((x) &  BIT_SHIFT_USBEAMFORMING_INFO<<&IT_MASK_CPBEAMFORMING_INFO<dd 2 REG_PAWMACPRTX_CTX_SUBTYPE_CFGOOffset 0x00C75 */

#define BIT_SHIFT_USRPWMACPCTX_SUBTYPE 4define BIT_MASK_CPRAWMACPCTX_SUBTYPE 0xfdefine BIT_SHRAWMACPCTX_SUBTYPE) &                                              	(((x) & &IT_MASK_BIRHWMACPCTX_SUBTYPE<< BIT_SHIFT_WLRHWMACPCTX_SUBTYPE<define BIT_GET_BIRHWMACPCTX_SUBTYPE) &                                          	(((x) &  BIT_SHIFT_USRHWMACPCTX_SUBTYPE<<&IT_MASK_BIRHWMACPCTX_SUBTYPE<#define BIT_CPIFT_USRAWMACPRTX_SUBTYPE 0define BIT_MASK_CPRAWMACPRTX_SUBTYPE 0xfdefine BIT_SHRAWMACPRTX_SUBTYPE) &                                              	(((x) & &IT_MASK_BIRHWMACPRTX_SUBTYPE<< BIT_SHIFT_WLRHWMACPRTX_SUBTYPE<define BIT_GET_BIRHWMACPRTX_SUBTYPE) &                                          	(((x) &  BIT_SHIFT_USRHWMACPRTX_SUBTYPE<<&IT_MASK_BIRHWMACPRTX_SUBTYPE<d* 2 REG_PABM_COEX_V2OOOOffset 0x00C762*/

#define BIT_CPGNTABM_POLARITY B_C112<define BIT_MATNTABM_BYPASSPPRIORITY B_C18<#define BIT_SHIFT_SDTIMERI0define BIT_MASK_CPTIMERI0xffdefine BIT_SHTIMER) & (x) & &IT_MASK_BITIMER&  BIT_SHIFT_WLTIMER&define BIT_MAT_BITIMER) & (x) &  BIT_SHIFT_USTIMER& & T_MASK_BITIMER&d* 2 REG_PABM_COEXOOOOffset 0x00C76 */

#define BIT_SHRATNTABM_RFC_SWIT_C112<define BIT_CPRATNTABM_RFC_SW_ENIT_C111<define BIT_CPRATNTABM_BB_SWIT_C110<define BIT_CPRATNTABM_BB_SW_ENIT_C19<define BIT_CPRABM_CNTATHRENIT_C18<d*efine BIT_CPIFT_USRABM_CNTATHR 0define BIT_MASK_CPRABM_CNTATHR 0xffdefine BIT_SHRHBM_CNTATHR) &                                                    	(((x) & &IT_MASK_CPRHBM_CNTATHR<< BIT_SHIFT_WLRHBM_CNTATHR<define BIT_GET_BIRHBM_CNTATHR) &                                                	(((x) &  BIT_SHIFT_USRHBM_CNTATHR<<&IT_MASK_CPRHBM_CNTATHR<dd 2 REG_PAWLANPACTASK_CPCTRLOOOffset 0x00C76 */

#define BIT_CPWLRXPTERHBYTCTLIT_C143<#efine BIT_CPWLRXPTERHBYTADIB_C142<define BIT_CPANGEDIVERSITY_SEL T_C141<define BIT_CPANTSEL_FORHBM_CTRL_ENIT_C140<#efine BIT_CPWLACTALOWATNTWL_ENIT_C134<#efine BIT_CPWLACTAHIGHATNTBT_ENIT_C133<dd 2 REG_PAWLANPACTASK_CPCTRLOOOffset 0x00C76 */

#define BIT_CPNAV_UPPER_V1 T_C132<dd 2 REG_PAWLANPACTASK_CPCTRLOOOffset 0x00C76 */

#define BIT_CPIFT_USRXMYRTSPNAV_V1 8define BIT_MASK_CPRXMYRTSPNAV_V1 0xffdefine BIT_SHRXMYRTSPNAV_V1) &                                                  	(((x) & &IT_MASK_BIRXMYRTSPNAV_V1<< BIT_SHIFT_WLRXMYRTSPNAV_V1<define BIT_GET_BIRXMYRTSPNAV_V1) &                                              	(((x) &  BIT_SHIFT_USRXMYRTSPNAV_V1<<&IT_MASK_BIRXMYRTSPNAV_V1<#define BIT_CPIFT_USRTSRST_V1 0define BIT_MASK_CPRTSRST_V1 0xffdefine BIT_SHRTSRST_V1) & (x) & &IT_MASK_BIRTSRST_V1<< BIT_SHIFT_WLRTSRST_V1<define BIT_GET_BIRTSRST_V1) & (x) &  BIT_SHIFT_USRTSRST_V1<<&IT_MASK_BIRTSRST_V1<d* 2 REG_PABM_COEX_ENHANCED_INTRPCTRLOOffset 0x00C76E*/

#define BIT_STIFT_USBTSSTAT_DELAY 12define BIT_MASK_CPBTSSTAT_DELAY 0xfdefine BIT_SHBTSSTAT_DELAY) &                                                   	(((x) & &IT_MASK_CPBTSSTAT_DELAY<< BIT_SHIFT_WLBTSSTAT_DELAY<define BIT_GET_BIBTSSTAT_DELAY) &                                               	(((x) &  BIT_SHIFT_USBTSSTAT_DELAY<<&IT_MASK_CPBTSSTAT_DELAY<#define BIT_STIFT_USBTSTRXPINIT_DETECT 8define BIT_MASK_CPBTSTRXPINIT_DETECT 0xfdefine BIT_SHBTSTRXPINIT_DETECT) &                                              	(((x) & &IT_MASK_BIBTSTRXPINIT_DETECT<< BIT_SHIFT_WLBTSTRXPINIT_DETECT<define BIT_GET_BIBTSTRXPINIT_DETECT) &                                          	(((x) &  BIT_SHIFT_USBTSTRXPINIT_DETECT<<&IT_MASK_BIBTSTRXPINIT_DETECT<#define BIT_STIFT_USBTSPRIADETECT_TO 4define BIT_MASK_CPBTSPRIADETECT_TO 0xfdefine BIT_SHBTSPRIADETECT_TO) &                                                	(((x) & &IT_MASK_CPBTSPRIADETECT_TO<< BIT_SHIFT_WLBTSPRIADETECT_TO<define BIT_GET_BIBTSPRIADETECT_TO) &                                            	(((x) &  BIT_SHIFT_USBTSPRIADETECT_TO<<&IT_MASK_CPBTSPRIADETECT_TO<#define BIT_SHRATRANTALLAWLSK_C T_C13<define BIT_GESTATIS_BT_ENIT_C12<define BIT_CPWLPACTASK_CPENABLEIT_C11<define BIT_MAENHANCED_BMIT_C10<d* 2 REG_PABM_ACTASTATISTICSOOOffset 0x00C77 */

#define BIT_SHIFT_USSTATIS_BT_LOIRXI(48<&ICPU_OPT_WIDTH)define BIT_MASK_CPSTATIS_BT_LOIRXI0fffffdefine BIT_STSTATIS_BT_LOIRX) &                                                 	(((x) & &IT_MASK_CPSTATIS_BT_LOIRX<< BIT_SHIFT_WLSTATIS_BT_LOIRX<define BIT_GET_BISTATIS_BT_LOIRX) &                                             	(((x) &  BIT_SHIFT_USSTATIS_BT_LOIRX<<&IT_MASK_CPSTATIS_BT_LOIRX<#define BIT_SHIFT_USSTATIS_BT_LOITXI(32<&ICPU_OPT_WIDTH)define BIT_MASK_CPSTATIS_BT_LOITXI0fffffdefine BIT_STSTATIS_BT_LOITX) &                                                 	(((x) & &IT_MASK_CPSTATIS_BT_LOITX<< BIT_SHIFT_WLSTATIS_BT_LOITX<define BIT_GET_BISTATIS_BT_LOITX) &                                             	(((x) &  BIT_SHIFT_USSTATIS_BT_LOITX<<&IT_MASK_CPSTATIS_BT_LOITX<d* 2 REG_PABM_ACTASTATISTICSOOOffset 0x00C77 */

#define BIT_SHIFT_USSTATIS_BT_HIIRXI1#define BIT_MASK_BISTATIS_BT_HIIRXI0fffffdefine BIT_STSTATIS_BT_HIIRX) &                                                 	(((x) & &IT_MASK_CPSTATIS_BT_HIIRX<< BIT_SHIFT_WLSTATIS_BT_HIIRX<define BIT_GET_BISTATIS_BT_HIIRX) &                                             	(((x) &  BIT_SHIFT_USSTATIS_BT_HIIRX<<&IT_MASK_CPSTATIS_BT_HIIRX<#define BIT_SHIFT_USSTATIS_BT_HIITXI0define BIT_MASK_BISTATIS_BT_HIITXI0fffffdefine BIT_STSTATIS_BT_HIITX) &                                                 	(((x) & &IT_MASK_CPSTATIS_BT_HIITX<< BIT_SHIFT_WLSTATIS_BT_HIITX<define BIT_GET_BISTATIS_BT_HIITX) &                                             	(((x) &  BIT_SHIFT_USSTATIS_BT_HIITX<<&IT_MASK_CPSTATIS_BT_HIITX<d* 2 REG_PABM_STATISTICS_CONTROL_G_PISTEROffset 0x00C77 */

#define BIT_CPIFT_USRHBM_CMD_RPMI1#define BIT_MASK_BIRABM_CMD_RPMI0fffffdefine BIT_STRABM_CMD_RPM) &                                                    	(((x) & &IT_MASK_CPRHBM_CMD_RPM<< BIT_SHIFT_WLRHBM_CMD_RPM<define BIT_GET_BIRHBM_CMD_RPM) &                                                	(((x) &  BIT_SHIFT_USRHBM_CMD_RPM<<&IT_MASK_CPRHBM_CMD_RPM<#define BIT_CPIFT_USRHRPM_FROM_BMI8define BIT_MASK_CPRHRPM_FROM_BMI0xffdefine BIT_SHRHRPM_FROM_BM) &                                                   	(((x) & &IT_MASK_CPRHRPM_FROM_BM<< BIT_SHIFT_WLRHRPM_FROM_BM<define BIT_GET_BIRHRPM_FROM_BM) &                                               	(((x) &  BIT_SHIFT_USRHRPM_FROM_BM<<&IT_MASK_CPRHRPM_FROM_BM<#define BIT_STIFT_USBTSHID_ISR_SET #define BIT_MASK_BIBTSHID_ISR_SET 0x3define BIT_MABTSHID_ISR_SET) &                                                  	(((x) & &IT_MASK_BIBTSHID_ISR_SET<< BIT_SHIFT_WLBTSHID_ISR_SET<define BIT_GET_BIBTSHID_ISR_SET) &                                              	(((x) &  BIT_SHIFT_USBTSHID_ISR_SET<<&IT_MASK_BIBTSHID_ISR_SET<#define BIT_CPTDMA_BM_STARSHNOTIFY B_C15<define BIT_MAENABLE_TDMA_FWPMODEIT_C14<define BIT_MAENABLE_PTA_TDMA_MODEIT_C13<define BIT_MAENABLE_COEXIST_TABPIN_TDMAIT_C12<define BIT_CPGPIO2PGPIO3_EXANGE_ORHNOHBM_CCAIT_C11<define BIT_MARTK_BT_ENABLEIT_C10<d* 2 REG_PABM_STATUS_G_PORT_G_PISTEROOffset 0x00C77 */

#define BIT_STIFT_USBTSPROFILE 24define BIT_MASK_CPBTSPROFILE 0xffdefine BIT_SHBTSPROFILE) & (x) & &IT_MASK_BIBTSPROFILE<< BIT_SHIFT_WLBTSPROFILE<define BIT_GET_BIBTSPROFILE) &                                                  	(((x) &  BIT_SHIFT_USBTSPROFILE<<&IT_MASK_BIBTSPROFILE<#define BIT_STIFT_USBTSPOWER 1#define BIT_MASK_BIBTSPOWER 0xffdefine BIT_SHBTSPOWER) & (x) & &IT_MASK_BIBTSPOWER<< BIT_SHIFT_WLBTSPOWER<define BIT_GET_BIBTSPOWER) & (x) &  BIT_SHIFT_USBTSPOWER<<&IT_MASK_BIBTSPOWER<ddefine BIT_STIFT_USBTSPREDECM_STATUS 8define BIT_MASK_CPBTSPREDECM_STATUS 0xffdefine BIT_SHBTSPREDECM_STATUS) &                                               	(((x) & &IT_MASK_CPBTSPREDECM_STATUS<< BIT_SHIFT_WLBTSPREDECM_STATUS<define BIT_GET_BIBTSPREDECM_STATUS) &                                           	(((x) &  BIT_SHIFT_USBTSPREDECM_STATUS<<&IT_MASK_CPBTSPREDECM_STATUS<ddefine BIT_STIFT_USBTSCMD_INFO 0define BIT_MASK_CPBTSCMD_INFO 0xffdefine BIT_SHBTSCMD_INFO) &                                                     	(((x) & &IT_MASK_CPBTSCMD_INFO<< BIT_SHIFT_WLBTSCMD_INFO<define BIT_GET_BIBTSCMD_INFO) &                                                 	(((x) &  BIT_SHIFT_USBTSCMD_INFO<<&IT_MASK_BIBTSCMD_INFO<d* 2 REG_PABM_INTERRUPT_CONTROL_G_PISTEROffset 0x00C78 */

#define BIT_SHEN_MACPNULL_PKT_NOTIFY B_C131<define BIT_MAENAWLANPRPM_AND_BM_QUERY B_C130<define BIT_MAENABM_STSTUS_GPMIT_C129<define BIT_MAENABM_POWER T_C128<define BIT_MAENABM_CHANNEL T_C127<define BIT_MAENABM_SLOM_CHANGEIT_C126<#efine BIT_CPENABM_PROFILE_ORHHIDIT_C125<define BIT_CPWLANPRPM_NOTIFY B_C124<#define BIT_CPIFT_USWLANPRPM_DATA 1#define BIT_MASK_BIWLANPRPM_DATA 0xffdefine BIT_SHWLANPRPM_DATA) &                                                   	(((x) & &IT_MASK_CPWLANPRPM_DATA<< BIT_SHIFT_WLWLANPRPM_DATA<define BIT_MAT_BIWLANPRPM_DATA) &                                               	(((x) &  BIT_SHIFT_USWLANPRPM_DATA<<&IT_MASK_CPWLANPRPM_DATA<#define BIT_SHIFT_SDCMD_ID 8define BIT_MASK_CPCMD_ID 0xffdefine BIT_SHCMD_ID) & (x) & &IT_MASK_BICMD_ID<< BIT_SHIFT_WLCMD_ID<define BIT_MAT_BICMD_ID) & (x) &  BIT_SHIFT_USCMD_ID<<&IT_MASK_BICMD_ID<ddefine BIT_STIFT_USBTSDATA 0define BIT_MASK_CPBTSDATA 0xffdefine BIT_SHBM_DATA) & (x) & &IT_MASK_CPBTSDATA<< BIT_SHIFT_WLBM_DATA<define BIT_MAT_BIBM_DATA) & (x) &  BIT_SHIFT_USBTSDATA<<&IT_MASK_CPBM_DATA<dd 2 REG_PAWLANPG_PORT_TIME_OUT_CONTROL_G_PISTER ffset 0x00C78 */

#define BIT_SHIFT_SDWLANPRPM_TO 0define BIT_MASK_CPWLANPRPM_TO 0xffdefine BIT_SHWLANPRPM_TO) &                                                     	(((x) & &IT_MASK_CPWLANPRPM_TO<< BIT_SHIFT_WLWLANPRPM_TO<define BIT_MAT_BIWLANPRPM_TO) &                                                 	(((x) &  BIT_SHIFT_USWLANPRPM_TO<<&IT_MASK_CPWLANPRPM_TO<d* 2 REG_PABM_ISOLATION_TABLE_G_PISTER_G_PISTER ffset 0x00C785*/

#define BIT_SHIFT_SDISOLATION_CHK 1define BIT_MASK_CPISOLATION_CHK 0x7fffffL
ffffffL
ffL
Ldefine BIT_GEISOLATION_CHK) &                                                   	(((x) & &IT_MASK_CPISOLATION_CHK<< BIT_SHIFT_WLISOLATION_CHK<define BIT_MAT_BIISOLATION_CHK) &                                               	(((x) &  BIT_SHIFT_USISOLATION_CHK<<&IT_MASK_CPISOLATION_CHK<d* 2 REG_PABM_ISOLATION_TABLE_G_PISTER_G_PISTER ffset 0x00C785*/

#define BIT_SHISOLATION_ENIB_C10<d* 2 REG_PABM_INTERRUPT_STATUS_G_PISTEROffset 0x00C78F*/

#define BIT_SHBTSHID_ISRIB_C17<define BIT_MABM_QUERY_ISRIB_C16<define BIT_CPMACPNULL_PKT_NOTIFY_ISRIB_C15<define BIT_CPWLANPRPM_ISRIB_C14<define BIT_GEBM_POWER_ISRIB_C13<define BIT_GEBM_CHANNEL_ISRIB_C12<define BIT_GEBM_SLOM_CHANGE_ISRIB_C11<define BIT_GEBM_PROFILE_ISRIB_C10<d* 2 REG_PABM_TDMA_TIME_G_PISTEROOffset 0x00C79 */

#define BIT_SHIFT_USBT_TIME #define BIT_MASK_BIBTSTIME 0x3ffffffdefine BIT_SHBM_TIME) & (x) & &IT_MASK_CPBTSTIME<< BIT_SHIFT_WLBTSTIME<define BIT_GET_BIBTSTIME) & (x) &  BIT_SHIFT_USBTSTIME<<&IT_MASK_CPBTSTIME<#define BIT_SHIFT_USBT_RPM_SAMPLE_GATE 0define BIT_MASK_CPBT_RPM_SAMPLE_GATE 0x3fdefine BIT_STBT_RPM_SAMPLE_GATE) &                                              	(((x) & &IT_MASK_BIBTSRPM_SAMPLE_GATE<< BIT_SHIFT_WLBTSRPM_SAMPLE_GATE<define BIT_GET_BIBTSRPM_SAMPLE_GATE) &                                          	(((x) &  BIT_SHIFT_USBTSRPM_SAMPLE_GATE<<&IT_MASK_BIBTSRPM_SAMPLE_GATE<d* 2 REG_PABM_ACTAG_PISTEROOOffset 0x00C79 */

#define BIT_SHIFT_SDBT_EISR_ENI1#define BIT_MASK_BIBTSEISR_ENI0xffdefine BIT_SHBM_EISR_EN) & (x) & &IT_MASK_CPBTSEISR_EN<< BIT_SHIFT_WLBTSEISR_EN<define BIT_GET_BIBTSEISR_EN) &                                                  	(((x) &  BIT_SHIFT_USBTSEISR_EN<<&IT_MASK_CPBTSEISR_EN<#define BIT_SHBTSACTAFALLING_ISRIB_C110<define BIT_GEBM_ACTAGISING_ISRIB_C19<define BIT_MATDMA_TO_ISRIB_C18<d*efine BIT_CPIFT_USBM_CH 0define BIT_MASK_CPBTSCH 0xffdefine BIT_SHBTSCH) & (x) & &IT_MASK_CPBTSCH<< BIT_SHIFT_WLBTSCH<define BIT_GET_BIBTSCH) & (x) &  BIT_SHIFT_USBTSCH<<&IT_MASK_CPBTSCH<d* 2 REG_PAOBFFSCTRL_BASICOOOffset 0x00C79 */

#define BIT_CPOBFFSENAV1 T_C131<#define BIT_CPIFT_USOBFFSSTATE_V1 28define BIT_MASK_CPOBFFSSTATE_V1 0x3define BIT_MAOBFFSSTATE_V1) &                                                   	(((x) & &IT_MASK_CPOBFFSSTATE_V1<< BIT_SHIFT_WLOBFFSSTATE_V1<define BIT_GET_BIOBFFSSTATE_V1) &                                               	(((x) &  BIT_SHIFT_USOBFFSSTATE_V1<<&IT_MASK_CPOBFFSSTATE_V1<#define BIT_CPOBFFSACTAGXDMA_ENIT_C127<define BIT_MAOBFFSBLOCCPINT_ENIT_C126<define BIT_MAOBFFSAUTOACTAENIT_C125<define BIT_MAOBFFSAUTOIDLE_ENIT_C124<#define BIT_CPIFT_USWAKEASKX_PLS 20define BIT_MASK_CPWAKEASKX_PLS 0x7define BIT_GEWAKEASKX_PLS) &                                                    	(((x) & &IT_MASK_CPWAKEASKX_PLS<< BIT_SHIFT_WLWAKEASKX_PLS<define BIT_MAT_BIWAKEASKX_PLS) &                                                	(((x) &  BIT_SHIFT_USWAKEASKX_PLS<<&IT_MASK_CPWAKEASKX_PLS<#define BIT_CPIFT_USWAKEASIN_PLS 1#define BIT_MASK_BIWAKEASIN_PLS 0x7define BIT_GEWAKEASIN_PLS) &                                                    	(((x) & &IT_MASK_CPWAKEASIN_PLS<< BIT_SHIFT_WLWAKEASIN_PLS<define BIT_MAT_BIWAKEASIN_PLS) &                                                	(((x) &  BIT_SHIFT_USWAKEASIN_PLS<<&IT_MASK_CPWAKEASIN_PLS<#define BIT_CPIFT_USWAKEASKX_F2F 12define BIT_MASK_CPWAKEASKX_F2F 0x7define BIT_GEWAKEASKX_F2F) &                                                    	(((x) & &IT_MASK_CPWAKEASKX_F2F<< BIT_SHIFT_WLWAKEASKX_F2F<define BIT_MAT_BIWAKEASKX_F2F) &                                                	(((x) &  BIT_SHIFT_USWAKEASKX_F2F<<&IT_MASK_CPWAKEASKX_F2F<#define BIT_CPIFT_USWAKEASIN_F2F 8define BIT_MASK_CPWAKEASIN_F2F 0x7define BIT_GEWAKEASIN_F2F) &                                                    	(((x) & &IT_MASK_CPWAKEASIN_F2F<< BIT_SHIFT_WLWAKEASIN_F2F<define BIT_MAT_BIWAKEASIN_F2F) &                                                	(((x) &  BIT_SHIFT_USWAKEASIN_F2F<<&IT_MASK_CPWAKEASIN_F2F<ddefine BIT_MAAPP_CPU_ACTAV1 T_C13)define BIT_MAAPP_OBFFSV1 T_C12<define BIT_CPAPP_IDLE_V1 T_C11<define BIT_CPAPP_INIT_V1 T_C10<d* 2 REG_PAOBFFSCTRL2ITIMEROOOffset 0x00C79 */

#define BIT_STIFT_USRXAHIGHATIMER_IDX 24define BIT_MASK_CPRXAHIGHATIMER_IDX 0x7define BIT_GERXAHIGHATIMER_IDX) &                                               	(((x) & &IT_MASK_CPRXAHIGHATIMER_IDX<< BIT_SHIFT_WLRXAHIGHATIMER_IDX<define BIT_GET_BIRXAHIGHATIMER_IDX) &                                           	(((x) &  BIT_SHIFT_USRXAHIGHATIMER_IDX<<&IT_MASK_CPRXAHIGHATIMER_IDX<#define BIT_STIFT_USRXAMEDATIMER_IDX 1#define BIT_MASK_BIRXAMEDATIMER_IDX 0x7define BIT_GERXAMEDATIMER_IDX) &                                                	(((x) & &IT_MASK_CPRXAMEDATIMER_IDX<< BIT_SHIFT_WLRXAMEDATIMER_IDX<define BIT_GET_BIRXAMEDATIMER_IDX) &                                            	(((x) &  BIT_SHIFT_USRXAMEDATIMER_IDX<<&IT_MASK_CPRXAMEDATIMER_IDX<#define BIT_STIFT_USRXALOWATIMER_IDX 8define BIT_MASK_CPRXALOWATIMER_IDX 0x7define BIT_GERXALOWATIMER_IDX) &                                                	(((x) & &IT_MASK_CPRXALOWATIMER_IDX<< BIT_SHIFT_WLRXALOWATIMER_IDX<define BIT_GET_BIRXALOWATIMER_IDX) &                                            	(((x) &  BIT_SHIFT_USRXALOWATIMER_IDX<<&IT_MASK_CPRXALOWATIMER_IDX<#define BIT_CPIFT_USOBFFSINT_TIMER_IDX 0define BIT_MASK_CPOBFFSINT_TIMER_IDX 0x7define BIT_GEOBFFSINT_TIMER_IDX) &                                              	(((x) & &IT_MASK_BIOBFFSINT_TIMER_IDX<< BIT_SHIFT_WLOBFFSINT_TIMER_IDX<define BIT_GET_BIOBFFSINT_TIMER_IDX) &                                          	(((x) &  BIT_SHIFT_USOBFFSINT_TIMER_IDX<<&IT_MASK_BIOBFFSINT_TIMER_IDX<d* 2 REG_PALTRPCTRL_BASICOOOffset 0x00C7A */

#define BIT_SHLTRPENAV1 T_C131<#efine BIT_SHLTRPHWPENAV1 T_C130<define BIT_GELRM_ACTACTS_ENIT_C129<#efine BIT_SHLTRPACTAGXPKT_ENIT_C128<#efine BIT_SHLTRPACTAGXDMA_ENIT_C127<define BIT_MALTRPIDLE_NO_SNOOPIT_C126<define BIT_MASPDUP_MGTPKTIT_C125<define BIT_MARXAAGG_ENIT_C124<#efine BIT_CPAPP_LTRPACTIT_C123)define BIT_MAAPP_LTRPIDLEIT_C122<#define BIT_SHIFT_SDHIGHAGATE_TRIG_SEL 20define BIT_MASK_CPHIGHAGATE_TRIG_SEL 0x3define BIT_MAHIGHAGATE_TRIG_SEL) &                                              	(((x) & &IT_MASK_BIHIGHAGATE_TRIG_SEL<< BIT_SHIFT_WLHIGHAGATE_TRIG_SEL<define BIT_GET_BIHIGHAGATE_TRIG_SEL) &                                          	(((x) &  BIT_SHIFT_USHIGHAGATE_TRIG_SEL<<&IT_MASK_BIHIGHAGATE_TRIG_SEL<#define BIT_SHIFT_USMEDAGATE_TRIG_SEL 18define BIT_MASK_CPMEDAGATE_TRIG_SEL 0x3define BIT_MAMEDAGATE_TRIG_SEL) &                                               	(((x) & &IT_MASK_CPMEDAGATE_TRIG_SEL<< BIT_SHIFT_WLMEDAGATE_TRIG_SEL<define BIT_GET_BIMEDAGATE_TRIG_SEL) &                                           	(((x) &  BIT_SHIFT_USMEDAGATE_TRIG_SEL<<&IT_MASK_CPMEDAGATE_TRIG_SEL<#define BIT_SHIFT_USLOWAGATE_TRIG_SEL 1#define BIT_MASK_BILOWAGATE_TRIG_SEL 0x3define BIT_MALOWAGATE_TRIG_SEL) &                                               	(((x) & &IT_MASK_CPLOWAGATE_TRIG_SEL<< BIT_SHIFT_WLLOWAGATE_TRIG_SEL<define BIT_GET_BILOWAGATE_TRIG_SEL) &                                           	(((x) &  BIT_SHIFT_USLOWAGATE_TRIG_SEL<<&IT_MASK_CPLOWAGATE_TRIG_SEL<#define BIT_SHIFT_SDHIGHAGATE_BD_IDX 8define BIT_MASK_CPHIGHAGATE_BD_IDX 0x7fdefine BIT_CPHIGHAGATE_BD_IDX) &                                                	(((x) & &IT_MASK_CPHIGHAGATE_BD_IDX<< BIT_SHIFT_WLHIGHAGATE_BD_IDX<define BIT_GET_BIHIGHAGATE_BD_IDX) &                                            	(((x) &  BIT_SHIFT_USHIGHAGATE_BD_IDX<<&IT_MASK_CPHIGHAGATE_BD_IDX<#define BIT_SHIFT_USLOWAGATE_BD_IDX 0define BIT_MASK_BILOWAGATE_BD_IDX 0x7fdefine BIT_CPLOWAGATE_BD_IDX) &                                                 	(((x) & &IT_MASK_CPLOWAGATE_BD_IDX<< BIT_SHIFT_WLLOWAGATE_BD_IDX<define BIT_GET_BILOWAGATE_BD_IDX) &                                             	(((x) &  BIT_SHIFT_USLOWAGATE_BD_IDX<<&IT_MASK_CPLOWAGATE_BD_IDX<d* 2 REG_PALTRPCTRL2ITIMERATHRESHOLDOOffset 0x00C7A */

#define BIT_SHIFT_SDRX_EMPTYATIMER_IDX 24define BIT_MASK_CPRXAEMPTYATIMER_IDX 0x7define BIT_GERXAEMPTYATIMER_IDX) &                                              	(((x) & &IT_MASK_BIRXAEMPTYATIMER_IDX<< BIT_SHIFT_WLRXAEMPTYATIMER_IDX<define BIT_GET_BIRXAEMPTYATIMER_IDX) &                                          	(((x) &  BIT_SHIFT_USRXAEMPTYATIMER_IDX<<&IT_MASK_BIRXAEMPTYATIMER_IDX<#define BIT_SHIFT_SDRX_AFULL_TH_IDX 20define BIT_MASK_CPRX_AFULL_TH_IDX 0x7define BIT_GERXAAFULL_TH_IDX) &                                                 	(((x) & &IT_MASK_CPRXAAFULL_TH_IDX<< BIT_SHIFT_WLRXAAFULL_TH_IDX<define BIT_GET_BIRXAAFULL_TH_IDX) &                                             	(((x) &  BIT_SHIFT_USRXAAFULL_TH_IDX<<&IT_MASK_CPRXAAFULL_TH_IDX<#define BIT_STIFT_USRXAHIGHATH_IDX 1#define BIT_MASK_BIRXAHIGHATH_IDX 0x7define BIT_GERXAHIGHATH_IDX) &                                                  	(((x) & &IT_MASK_CPRXAHIGHATH_IDX<< BIT_SHIFT_WLRXAHIGHATH_IDX<define BIT_GET_BIRXAHIGHATH_IDX) &                                              	(((x) &  BIT_SHIFT_USRXAHIGHATH_IDX<<&IT_MASK_CPRXAHIGHATH_IDX<ddefine BIT_STIFT_USRXAMEDATH_IDX 12define BIT_MASK_CPRXAMEDATH_IDX 0x7define BIT_GERXAMEDATH_IDX) &                                                   	(((x) & &IT_MASK_CPRXAMEDATH_IDX<< BIT_SHIFT_WLRXAMEDATH_IDX<define BIT_GET_BIRXAMEDATH_IDX) &                                               	(((x) &  BIT_SHIFT_USRXAMEDATH_IDX<<&IT_MASK_CPRXAMEDATH_IDX<ddefine BIT_STIFT_USRXALOWATH_IDX 8define BIT_MASK_CPRXALOWATH_IDX 0x7define BIT_GERXALOWATH_IDX) &                                                   	(((x) & &IT_MASK_CPRXALOWATH_IDX<< BIT_SHIFT_WLRXALOWATH_IDX<define BIT_GET_BIRXALOWATH_IDX) &                                               	(((x) &  BIT_SHIFT_USRXALOWATH_IDX<<&IT_MASK_CPRXALOWATH_IDX<ddefine BIT_SHIFT_USLTRPSPACE_IDX 4define BIT_MASK_CPLTRPSPACE_IDX 0x3define BIT_MALTRPSPACE_IDX) &                                                   	(((x) & &IT_MASK_CPLTRPSPACE_IDX<< BIT_SHIFT_WLLTRPSPACE_IDX<define BIT_GET_BILTRPSPACE_IDX) &                                               	(((x) &  BIT_SHIFT_USLTRPSPACE_IDX<<&IT_MASK_CPLTRPSPACE_IDX<ddefine BIT_SHIFT_USLTRPIDLE_TIMER_IDX 0define BIT_MASK_CPLTRPIDLE_TIMER_IDX 0x7define BIT_GELTRPIDLE_TIMER_IDX) &                                              	(((x) & &IT_MASK_BILTRPIDLE_TIMER_IDX<< BIT_SHIFT_WLLTRPIDLE_TIMER_IDX<define BIT_GET_BILTRPIDLE_TIMER_IDX) &                                          	(((x) &  BIT_SHIFT_USLTRPIDLE_TIMER_IDX<<&IT_MASK_BILTRPIDLE_TIMER_IDX<d* 2 REG_PALTRPIDLE_LATENCYSV1OOOffset 0x00C7A */

#define BIT_CPIFT_USLTRPIDLE_L 0define BIT_MASK_CPLTRPIDLE_L 0fffffffL
Ldefine BIT_GELTRPIDLE_L) & (x) & &IT_MASK_CPLTRPIDLE_L<< BIT_SHIFT_WLLTRPIDLE_L<define BIT_GET_BILTRPIDLE_L) &                                                  	(((x) &  BIT_SHIFT_USLTRPIDLE_L& &IT_MASK_CPLTRPIDLE_L<d* 2 REG_PALTRPACTIVE_LATENCYSV1OOffset 0x00C7A */

#define BIT_STIFT_USLTRPACTAL 0define BIT_MASK_CPLTRPACTAL 0fffffffL
Ldefine BIT_GELTRPACTAL) & (x) & &IT_MASK_CPLTRPACTAL<< BIT_SHIFT_WLLTRPACTAL<define BIT_GET_BILTRPACTAL) & (x) &  BIT_SHIFT_USLTRPACTAL<<&IT_MASK_CPLTRPACTAL<d* 2 REG_PAANTENNA_TRAINING_CONTROL_G_PISTEROffset 0x00C7B */

#define BIT_SHAPPEND_MACIDPIN_RESPIENIT_C150<define BIT_GEADDR2_MATCH_ENIT_C149<#efine BIT_SHANTTRN_ENIB_C148<#define BIT_SHIFT_SDTRAINSSTAEADDRI0define BIT_MASK_CPTRAINSSTAEADDRI0fffffffL
ffL
Ldefine BIT_GETRAINSSTAEADDR) &                                                  	(((x) & &IT_MASK_CPTRAINSSTAEADDR&  BIT_SHIFT_WLTRAINSSTAEADDR&define BIT_GET_BITRAINSSTAEADDR) &                                              	(((x) &  BIT_SHIFT_USTRAINSSTAEADDR& &IT_MASK_CPTRAINSSTAEADDR&dd 2 REG_PAWMACPPKTCNTARWDOOOffset 0x00C7B */

#define BIT_CPIFT_USPKTCNTABSSIDMAP 4define BIT_MASK_CPPKTCNTABSSIDMAP 0xfdefine BIT_SHPKTCNTABSSIDMAP) &                                                 	(((x) & &IT_MASK_CPPKTCNTABSSIDMAP<< BIT_SHIFT_WLPKTCNTABSSIDMAP<define BIT_GET_BIPKTCNTABSSIDMAP) &                                             	(((x) &  BIT_SHIFT_USPKTCNTABSSIDMAP<<&IT_MASK_CPPKTCNTABSSIDMAP<
define BIT_SHPKTCNTACNTRSTIT_C11<#efine BIT_SHPKTCNTACNTENIB_C10<d* 2 REG_PAWMACPPKTCNTACTRLOOOffset 0x00C7B */

#define BIT_STWMACPPKTCNTATRSTIT_C19<#efine BIT_SHWMACPPKTCNTAFENIT_C18<d*efine BIT_CPIFT_USWMACPPKTCNTACFGAD 0define BIT_MASK_CPWMACPPKTCNTACFGAD 0xffdefine BIT_SHWMACPPKTCNTACFGAD) &                                               	(((x) & &IT_MASK_CPWMACPPKTCNTACFGAD<< BIT_SHIFT_WLWMACPPKTCNTACFGAD<define BIT_MAT_BIWMACPPKTCNTACFGAD) &                                           	(((x) &  BIT_SHIFT_USWMACPPKTCNTACFGAD<<&IT_MASK_CPWMACPPKTCNTACFGAD<d* 2 REG_PAIQ_DUMPOOOOffset 0x00C7C */

#define BIT_SHIFT_USRPWMACPMATCH_REF_MAC (64<&ICPU_OPT_WIDTH)define BIT_MASK_CPRPWMACPMATCH_REF_MAC 0fffffffL
Ldefine BIT_GERPWMACPMATCH_REF_MAC) &                                            	(((x) & &IT_MASK_BIRHWMACPMATCH_REF_MAC&                                 	((< BIT_SHIFT_WLRHWMACPMATCH_REF_MAC&define BIT_GET_BIRPWMACPMATCH_REF_MAC) &                                        	(((x) &  BIT_SHIFT_USRHWMACPMATCH_REF_MAC& &                             	((<T_MASK_CPRHWMACPMATCH_REF_MAC&ddefine BIT_SHIFT_USRPWMACPRXAFIL_LENI(64<&ICPU_OPT_WIDTH)define BIT_MASK_CPRPWMACPRXAFIL_LENI0fffffdefine BIT_STRAWMACPRXAFIL_LEN) &                                               	(((x) & &IT_MASK_CPRAWMACPRXAFIL_LEN<< BIT_SHIFT_WLRHWMACPRXAFIL_LEN<define BIT_GET_BIRPWMACPRXAFIL_LEN) &                                           	(((x) &  BIT_SHIFT_USRHWMACPRXAFIL_LEN<<&IT_MASK_CPRAWMACPRXAFIL_LEN<ddefine BIT_SHIFT_USRPWMACPRXFIFO_FULL_TH (56<&ICPU_OPT_WIDTH)define BIT_MASK_CPRPWMACPRXFIFO_FULL_TH 0xffdefine BIT_SHRHWMACPRXFIFO_FULL_TH) &                                           	(((x) & &IT_MASK_CPRAWMACPRXFIFO_FULL_TH&                                	((< BIT_SHIFT_WLRHWMACPRXFIFO_FULL_TH&define BIT_GET_BIRPWMACPRXFIFO_FULL_TH) &                                       	(((x) &  BIT_SHIFT_USRHWMACPRXFIFO_FULL_TH& &                            	((<T_MASK_CPRHWMACPRXFIFO_FULL_TH&ddefine BIT_SHRHWMACPSRCH_TXRPM_TYPE T_C151<define BIT_CPRHWMACPNDPIRSTIT_C150<define BIT_GERHWMACPPOWINT_ENIT_C149<#efine BIT_SHRHWMACPSRCH_TXRPM_PERPKTIT_C148<#efine BIT_SHRHWMACPSRCH_TXRPM_MIDIT_C147<define BIT_CPREWMACPPFIN_TOENIT_C146<define BIT_CPREWMACPFIL_SECERRIB_C145<define BIT_GERHWMACPFIL_CTLPKTLENIT_C144<define BIT_CPRPWMACPFIL_FCTYPE T_C143<#efine BIT_CPRPWMACPFIL_FCPROVER T_C142<define BIT_CPREWMACPPHYSTS_SNIF T_C141<define BIT_CPRHWMACPPHYSTS_PLCPIT_C140<#efine BIT_CPR_MAC_TCR_VBONF_RDIT_C139<#efine BIT_SHRHWMACPTCR_MPARPNDPIT_C138<#efine BIT_SHRHWMACPNDPIFILTER T_C137<define BIT_CPREWMACPRXLEN_SEL T_C136<define BIT_CPREWMACPRXLEN_SEL1 T_C135<define BIT_GERHOFDMIFILTER T_C134<define BIT_CPRPWMACPCHKHOFDMILENIT_C133<ddefine BIT_SHIFT_USRPWMACPMA_CPLA_MAC (32<&ICPU_OPT_WIDTH)define BIT_MASK_CPRPWMACPMA_CPLA_MAC 0fffffffL
Ldefine BIT_GERPWMACPMA_CPLA_MAC) &                                              	(((x) & &IT_MASK_BIRPWMACPMA_CPLA_MAC<< BIT_SHIFT_WLRHWMACPMA_CPLA_MAC<define BIT_GET_BIRPWMACPMA_CPLA_MAC) &                                          	(((x) &  BIT_SHIFT_USRHWMACPMA_CPLA_MAC<<&IT_MASK_BIRPWMACPMA_CPLA_MAC<ddefine BIT_SHRHWMACPCHKHCCKILENIT_C132<d* 2 REG_PAIQ_DUMPOOOOffset 0x00C7C */

#define BIT_SHIFT_USRPOFDMILENI2#define BIT_MASK_BIRAOFDMILENI0x3fdefine BIT_STRAOFDMILEN) & (x) & &IT_MASK_CPRAOFDMILEN<< BIT_SHIFT_WLRHOFDMILEN<define BIT_GET_BIRPOFDMILEN) &                                                  	(((x) &  BIT_SHIFT_USRHOFDMILEN<<&IT_MASK_CPRAOFDMILEN<#define BIT_CPIFT_USDUMP_OKEADDRI15define BIT_MASK_BIDUMP_OKEADDRI0x1ffffdefine BIT_SHDUMP_OKEADDR) &                                                    	(((x) & &IT_MASK_CPDUMP_OKEADDR<< BIT_SHIFT_WLDUMP_OKEADDR<define BIT_GET_BIDUMP_OKEADDR) &                                                	(((x) &  BIT_SHIFT_USDUMP_OKEADDR<<&IT_MASK_CPDUMP_OKEADDR<#define BIT_SHIFT_USRPTRIG_TIME_SEL 8define BIT_MASK_CPRHTRIG_TIME_SEL 0x7fdefine BIT_CPRHTRIG_TIME_SEL) &                                                 	(((x) & &IT_MASK_CPRHTRIG_TIME_SEL<< BIT_SHIFT_WLRHTRIG_TIME_SEL<define BIT_GET_BIRPTRIG_TIME_SEL) &                                             	(((x) &  BIT_SHIFT_USRHTRIG_TIME_SEL<<&IT_MASK_CPRHTRIG_TIME_SEL<#define BIT_SHIFT_USRPMACPTRIG_SEL #define BIT_MASK_BIRAMACPTRIG_SEL 0x3define BIT_MARAMACPTRIG_SEL) &                                                  	(((x) & &IT_MASK_CPRAMACPTRIG_SEL<< BIT_SHIFT_WLRHMACPTRIG_SEL<define BIT_GET_BIRPMACPTRIG_SEL) &                                              	(((x) &  BIT_SHIFT_USRHMACPTRIG_SEL<<&IT_MASK_CPRAMACPTRIG_SEL<
define BIT_MASKCPTRIG_G_PIT_C15<#define BIT_SHIFT_USRPLEVEL_PULSE_SEL 3define BIT_MASK_BIRALEVEL_PULSE_SEL 0x3define BIT_MARALEVEL_PULSE_SEL) &                                               	(((x) & &IT_MASK_CPRALEVEL_PULSE_SEL<< BIT_SHIFT_WLRHLEVEL_PULSE_SEL<define BIT_GET_BIRPLEVEL_PULSE_SEL) &                                           	(((x) &  BIT_SHIFT_USRHLEVEL_PULSE_SEL<<&IT_MASK_CPRALEVEL_PULSE_SEL<#define BIT_SHEN_LA_MAC B_C12<define BIT_CPRPEN_IQDUMP B_C11<define BIT_CPRHIQDATA_DUMP B_C10<#*efine BIT_SHIFT_SDRHCCKILENI0define BIT_MASK_CPRACCKILENI0fffffdefine BIT_STRACCKILEN) & (x) & &IT_MASK_CPRACCKILEN<< BIT_SHIFT_WLRHCCKILEN<define BIT_GET_BIRPCCKILEN) & (x) &  BIT_SHIFT_USRHCCKILEN<<&IT_MASK_CPRACCKILEN<d* 2 REG_PAWMACPFTM_CTLOOOffset 0x00C7C */

#define BIT_STRXFTM_TXACKISC T_C16<define BIT_CPRXFTM_TXACKIBWIT_C15<#efine BIT_CPRXFTM_ENIT_C13<#efine BIT_CPRXFTMREQ_BYDRV B_C12<define BIT_CPRXFTMREQ_ENIB_C11<define BIT_CPFTM_ENIT_C10<d* 2 REG_PARXAFILTER_FUNCTIONOOOffset 0x00C7DA*/

#define BIT_SHRHWMACPMHRDDY_LATCHIB_C114<d* 2 REG_PARXAFILTER_FUNCTIONOOOffset 0x00C7DA*/

#define BIT_SHRHWMACPMHRDDY_CLRIB_C113<d* 2 REG_PARXAFILTER_FUNCTIONOOOffset 0x00C7DA*/

#define BIT_SHRHGXPKTCTL_FSM_BASED_MPDURDY1 T_C112<d* 2 REG_PARXAFILTER_FUNCTIONOOOffset 0x00C7DA*/

#define BIT_SHWMACPDISPVHT_PLCPPCHKHMU T_C111<d* 2 REG_PARXAFILTER_FUNCTIONOOOffset 0x00C7DA*/

#define BIT_SHRHCHKHDELIM_SHLENIT_C110<define BIT_CPRAREAPTER_ADDRPMATCHIT_C19<#efine BIT_SHRHGXPKTCTL_FSM_BASED_MPDURDYIT_C18<#efine BIT_SHRHLATCH_MACHRDYIT_C17<define BIT_CPREWMACPRXFIL_REND T_C16<define BIT_CPRHWMACPMPDURDY_CLRIB_C15<define BIT_GERHWMACPCLRRXSEC B_C14<define BIT_CPRPWMACPRXFIL_RDEL T_C13<#efine BIT_CPRPWMACPRXFIL_FCSE B_C12<define BIT_CPRPWMACPRXFIL_MESH_DEL T_C11<define BIT_CPRHWMACPRXFIL_MK_CMIT_C10<d* 2 REG_PANDPISIGOOOOffset 0x00C7E */

#define BIT_SHIFT_USRPWMACPTXNDPISIGB 0define BIT_MASK_CPRAWMACPTXNDPISIGB 0x1fffffdefine BIT_SHRHWMACPTXNDPISIGB) &                                               	(((x) & &IT_MASK_CPRAWMACPTXNDPISIGB<< BIT_SHIFT_WLRHWMACPTXNDPISIGB<define BIT_GET_BIRPWMACPTXNDPISIGB) &                                           	(((x) &  BIT_SHIFT_USRHWMACPTXNDPISIGB<<&IT_MASK_CPRAWMACPTXNDPISIGB<d* 2 REG_PATXCMD_INFO_FORHRSPIPKTOOffset 0x00C7E */

#define BIT_SHIFT_SDRASKCPDEBUG (32<&ICPU_OPT_WIDTH)define BIT_MASK_CPRPSKCPDEBUG 0fffffffL
Ldefine BIT_GERPSKCPDEBUG) &                                                     	(((x) & &IT_MASK_CPRAMACPDEBUG<< BIT_SHIFT_WLRHMACPDEBUG<define BIT_GET_BIRPMACPDEBUG) &                                                 	(((x) &  BIT_SHIFT_USRHMACPDEBUG<<&IT_MASK_CPRAMACPDEBUG<d* 2 REG_PATXCMD_INFO_FORHRSPIPKTOOffset 0x00C7E */

#define BIT_SHIFT_SDRASKCPDBGHIFT_S 8define BIT_MASK_CPRHSKCPDBGHIFT_S 0x7define BIT_GERHSKCPDBGHIFT_S) &                                                 	(((x) & &IT_MASK_CPRHSKCPDBGHIFT_S<< BIT_SHIFT_WLRHMACPDBGHIFT_S<define BIT_GET_BIRPMACPDBGHIFT_S) &                                             	(((x) &  BIT_SHIFT_USRHMACPDBGHIFT_S<<&IT_MASK_CPRHSKCPDBGHIFT_S<#define BIT_SHIFT_SDRASKCPDBGHIEL 0define BIT_MASK_CPRASKCPDBGHIEL 0x3define BIT_MARAMACPDBGHIEL) &                                                   	(((x) & &IT_MASK_CPRAMACPDBGHIEL<< BIT_SHIFT_WLRHMACPDBGHIEL<define BIT_GET_BIRPMACPDBGHIEL) &                                               	(((x) &  BIT_SHIFT_USRHMACPDBGHIEL<<&IT_MASK_CPRAMACPDBGHIEL<d* 2 REG_PASYSACFG3OOOOffset 0x00100 */

#define BIT_SHPWCPMA33V B_C115<d* 2 REG_PASYSACFG3OOOOffset 0x00100 */

#define BIT_SHPWCPMA12VIB_C114<define BIT_SHPWCPMD12VIB_C113<define BIT_SHPWCPPD12VIB_C112<define BIT_SHPWCPUD12VIB_C111<define BIT_CPISOPMA2MD T_C11<d* 2 REG_PASYSACFG5OOOOffset 0x00107 */

#define BIT_SHLPS_STATUS T_C13<#efine BIT_CPHCIITXDMA_BUSY B_C12<define BIT_CPHCIITXDMA_ALLOWIB_C11<define BIT_CPFWPCTRL_HCIITXDMA_ENIT_C10<d* 2 REG_PACPU_DMEM_CONOOOffset 0x00108 */

#define BIT_SHWDT_OPT_IOWRAPPERIB_C119<d* 2 REG_PACPU_DMEM_CONOOOffset 0x00108 */

#define BIT_SHANA_PORT_IDLEIT_C118)define BIT_MASKC_PORT_IDLEIT_C117<define BIT_CPWL_PLATFORMIRSTIT_C116<define BIT_CPWL_SECURITYPCLK B_C115<d* 2 REG_PACPU_DMEM_CONOOOffset 0x00108 */

#define BIT_SHIFT_USCPU_DMEM_CON 0define BIT_MASK_CPCPU_DMEM_CON 0xffdefine BIT_SHCPU_DMEM_CON) &                                                    	(((x) & &IT_MASK_CPCPU_DMEM_CON<< BIT_SHIFT_WLCPU_DMEM_CON<define BIT_GET_BICPU_DMEM_CON) &                                                	(((x) &  BIT_SHIFT_USCPU_DMEM_CON<<&IT_MASK_CPCPU_DMEM_CON<d* 2 REG_PABOOTAREASONOOOOffset 0x00108 */

#define BIT_CPIFT_USBOOTAREASON 0define BIT_MASK_CPBOOTAREASON 0x7define BIT_GEBOOTAREASON) &                                                     	(((x) & &IT_MASK_CPBOOTAREASON<< BIT_SHIFT_WLBOOTAREASON<define BIT_GET_BIBOOTAREASON) &                                                 	(((x) &  BIT_SHIFT_USBOOTAREASON<<&IT_MASK_CPBOOTAREASON<d* 2 REG_PANFCPADACTRLOOOOffset 0x0010A */

#define BIT_CPPADASHUTDWIT_C118)define BIT_MASYSONANFCPPADIT_C117<define BIT_CPNFCPINT_PADACTRLIT_C116<define BIT_CPNFCPRFDISPPADACTRLIT_C115<define BIT_CPNFCPCLKPPADACTRLIT_C114<define BIT_CPNFCPDATA_PADACTRLIT_C113<define BIT_CPNFCPPADAPULL_CTRLIT_C112<#define BIT_SHIFT_SDNFCPADAIO_SEL 8define BIT_MASK_CPNFCPADAIO_SEL 0xfdefine BIT_SHNFCPADAIO_SEL) &                                                   	(((x) & &IT_MASK_CPNFCPADAIO_SEL<< BIT_SHIFT_WLNFCPADAIO_SEL<define BIT_GET_BINFCPADAIO_SEL) &                                               	(((x) &  BIT_SHIFT_USNFCPADAIO_SEL<<&IT_MASK_CPNFCPADAIO_SEL<#define BIT_SHIFT_SDNFCPADAOUT 4define BIT_MASK_CPNFCPADAOUT 0xfdefine BIT_SHNFCPADAOUT) & (x) & &IT_MASK_CPNFCPADAOUT<< BIT_SHIFT_WLNFCPADAOUT<define BIT_GET_BINFCPADAOUT) &                                                  	(((x) &  BIT_SHIFT_USNFCPADAOUT<<&IT_MASK_CPNFCPADAOUT<#define BIT_SHIFT_SDNFCPADAIN 0define BIT_MASK_CPNFCPADAIN 0xfdefine BIT_SHNFCPADAIN) & (x) & &IT_MASK_CPNFCPADAIN<< BIT_SHIFT_WLNFCPADAIN<define BIT_GET_BINFCPADAIN) & (x) &  BIT_SHIFT_USNFCPADAIN& &IT_MASK_CPNFCPADAIN<d* 2 REG_PAHIMR2OOOOffset 0x0010B */

#define BIT_SHBCNDMAINT_P4_M_C T_C131<define BIT_GEBCNDMAINT_P3_M_C T_C130<define BIT_GEBCNDMAINT_P2_M_C T_C129<#efine BIT_SHBCNDMAINT_P1_M_C T_C128)define BIT_MAATIMEND7_M_C T_C122<define BIT_CPATIMEND6_M_C T_C121<define BIT_CPATIMEND5_M_C T_C120<define BIT_GEATIMEND4_M_C T_C119<#efine BIT_SHATIMEND3_M_C T_C118)define BIT_MAATIMEND2_M_C T_C117<define BIT_CPATIMEND1_M_C T_C116<define BIT_CPTXBCN7OK_M_C T_C114<define BIT_CPTXBCN6OK_M_C T_C113<define BIT_CPTXBCN5OK_M_C T_C112<define BIT_CPTXBCN4OK_M_C T_C111<define BIT_CPTXBCN3OK_M_C T_C110<define BIT_CPTXBCN2OK_M_C T_C19<define BIT_MATXBCN1OK_M_CAV1 T_C18<define BIT_CPTXBCN7ERR_M_C T_C16<define BIT_CPTXBCN6ERR_M_C T_C15<define BIT_CPTXBCN5ERR_M_C T_C14<define BIT_CPTXBCN4ERR_M_C T_C13<define BIT_CPTXBCN3ERR_M_C T_C12<define BIT_CPTXBCN2ERR_M_C T_C11<define BIT_CPTXBCN1ERR_M_C_V1 T_C10<d* 2 REG_PAHISR2OOOOffset 0x0010B */

#define BIT_SHBCNDMAINT_P4 T_C131<define BIT_GEBCNDMAINT_P3 T_C130<define BIT_GEBCNDMAINT_P2 T_C129<#efine BIT_SHBCNDMAINT_P1 T_C128)define BIT_MAATIMEND7 T_C122<define BIT_CPATIMEND6 T_C121<define BIT_CPATIMEND5 T_C120<define BIT_GEATIMEND4 T_C119<#efine BIT_SHATIMEND3 T_C118)define BIT_MAATIMEND2 T_C117<define BIT_CPATIMEND1 T_C116<define BIT_CPTXBCN7OK T_C114<define BIT_CPTXBCN6OK T_C113<define BIT_CPTXBCN5OK T_C112<define BIT_CPTXBCN4OK T_C111<define BIT_CPTXBCN3OK T_C110<define BIT_CPTXBCN2OK T_C19<define BIT_MATXBCN1OK T_C18<define BIT_CPTXBCN7ERR T_C16<define BIT_CPTXBCN6ERR T_C15<define BIT_CPTXBCN5ERR T_C14<define BIT_CPTXBCN4ERR T_C13<define BIT_CPTXBCN3ERR T_C12<define BIT_CPTXBCN2ERR T_C11<define BIT_CPTXBCN1ERR T_C10<d* 2 REG_PAHIMR3OOOOffset 0x0010B */

#define BIT_CPWDT_PLATFORMIINT_M_C T_C118)define BIT_MAWDT_CPU_INT_M_C T_C117<d* 2 REG_PAHIMR3OOOOffset 0x0010B */

#define BIT_CPSETH2CDOK_MA_C T_C116<define BIT_CPH2CICMD_FULL_MA_C T_C115<define BIT_CPPWR_INT_127_MA_C T_C114<define BIT_CPTXSHORTCUCPTXDESUPDATEOK_MA_C T_C113<define BIT_CPTXSHORTCUCPBKUPDATEOK_MA_C T_C112<define BIT_CPTXSHORTCUCPBEUPDATEOK_MA_C T_C111<define BIT_CPTXSHORTCUCPVIUPDATEOK_MA_ T_C110<define BIT_CPTXSHORTCUCPVOUPDATEOK_MA_C T_C19<define BIT_CPPWR_INT_127_MA_CAV1 T_C18<define BIT_CPPWR_INT_126TO96_MA_C T_C17<define BIT_CPPWR_INT_95TO64_MA_C T_C16<define BIT_CPPWR_INT_63TO32_MA_C T_C15<define BIT_CPPWR_INT_31TO0_MA_C T_C14<define BIT_CPDDMA0_LP_INT_M_C T_C11<define BIT_CPDDMA0_HP_INT_M_C T_C10<d* 2 REG_PAHISR3OOOOffset 0x0010B */

#define BIT_STWDT_PLATFORMIINT T_C118)define BIT_MAWDT_CPU_INT T_C117<d* 2 REG_PAHISR3OOOOffset 0x0010B */

#define BIT_STSETH2CDOK T_C116<define BIT_CPH2CICMD_FULL T_C115<define BIT_CPPWR_INT_127 T_C114<define BIT_CPTXSHORTCUCPTXDESUPDATEOK T_C113<define BIT_CPTXSHORTCUCPBKUPDATEOK T_C112<define BIT_CPTXSHORTCUCPBEUPDATEOK T_C111<define BIT_CPTXSHORTCUCPVIUPDATEOK T_C110<define BIT_CPTXSHORTCUCPVOUPDATEOK T_C19<define BIT_CPPWR_INT_127_V1 T_C18<define BIT_CPPWR_INT_126TO96 T_C17<define BIT_CPPWR_INT_95TO64 T_C16<define BIT_CPPWR_INT_63TO32 T_C15<define BIT_CPPWR_INT_31TO0 T_C14<define BIT_CPDDMA0_LP_INT T_C11<define BIT_CPDDMA0_HP_INT T_C10<d* 2 REG_PASW_MDIOOOOOffset 0x0010C */

#define BIT_SHDISPTIMEOUT_IO T_C124<d* 2 REG_PASW_FLUSHOOOOffset 0x0010C */

#define BIT_SHFLUSH_HOLDNAENIT_C125<define BIT_MAFLUSH_WR_ENIT_C124<#efine BIT_CPSW_FLASH_CONTROLIT_C123<define BIT_GESW_FLASH_WENAE T_C119<#efine BIT_SHSW_FLASH_HOLDNAE T_C118)define BIT_MASW_FLASH_SOAE T_C117)define BIT_MASW_FLASH_SIAE T_C116<define BIT_MASW_FLASH_SK_O T_C113<define BIT_CPSW_FLASH_CENAO T_C112<define BIT_CPSW_FLASH_WENAO T_C111<define BIT_CPSW_FLASH_HOLDNAO T_C110<define BIT_CPSW_FLASH_SOAO T_C19<define BIT_CPSW_FLASH_SIAO T_C18<define BIT_CPSW_FLASH_WENAI T_C13<define BIT_CPSW_FLASH_HOLDNAI T_C12<define BIT_CPSW_FLASH_SOAI T_C11<define BIT_CPSW_FLASH_SIAI T_C10<d* 2 REG_PAH2CPPKTAREADADDROOOffset 0x0010D */

#define BIT_SHIFT_USH2CPPKTAREADADDR 0define BIT_MASK_CPH2CPPKTAREADADDR 0x3ffffdefine BIT_CPH2CIPKTAREADADDR) &                                                	(((x) & &IT_MASK_CPH2CPPKTAREADADDR<< BIT_SHIFT_WLH2CPPKTAREADADDR<define BIT_GET_BIH2CIPKTAREADADDR) &                                            	(((x) &  BIT_SHIFT_USH2CPPKTAREADADDR<<&IT_MASK_CPH2CPPKTAREADADDR<d* 2 REG_PAH2CPPKTAWRITEADDROOOffset 0x0010D */

#define BIT_SHIFT_SDH2CPPKTAWRITEADDR 0define BIT_MASK_CPH2CPPKTAWRITEADDR 0x3ffffdefine BIT_CPH2CIPKTAWRITEADDR) &                                               	(((x) & &IT_MASK_CPH2CIPKTAWRITEADDR<< BIT_SHIFT_WLH2CPPKTAWRITEADDR<define BIT_GET_BIH2CIPKTAWRITEADDR) &                                           	(((x) &  BIT_SHIFT_USH2CPPKTAWRITEADDR<<&IT_MASK_CPH2CIPKTAWRITEADDR<d* 2 REG_PAMEM_PWR_CRTLOOOffset 0x0010D */

#define BIT_CPMEM_BB_SDIT_C117<define BIT_CPMEM_BB_DS T_C116<define BIT_MAMEM_BT_DS T_C110<define BIT_MAMEM_SDIO_LS T_C19<define BIT_CPMEM_SDIO_DS T_C18)define BIT_MASEM_USB_LS T_C17<define BIT_CPMEM_USB_DS T_C16<define BIT_MAMEM_PCI_LS T_C15<define BIT_MAMEM_PCI_DS T_C14<define BIT_MAMEM_WLSKC_LS T_C13<#efine BIT_CPMEM_WLSKC_DS T_C12<#efine BIT_CPMEM_WLSCU_LS T_C11<d* 2 REG_PAMEM_PWR_CRTLOOOffset 0x0010D */

#define BIT_CPMEM_WLSCU_DS T_C10<d* 2 REG_PAFWPDBG0OOOOffset 0x0010E */

#define BIT_SHIFT_USFWPDBG0 0define BIT_MASK_CPFWPDBG0 0fffffffL
Ldefine BIT_GEFWPDBG0) & (x) & &IT_MASK_CPFWPDBG0<< BIT_SHIFT_WLFWPDBG0<define BIT_GET_BIFWPDBG0) & (x) &  BIT_SHIFT_USFWPDBG0<<&IT_MASK_CPFWPDBG0<d* 2 REG_PAFWPDBG1OOOOffset 0x0010E */

#define BIT_SHIFT_SDFWPDBG1 0define BIT_MASK_CPFWPDBG1 0fffffffL
Ldefine BIT_GEFWPDBG1) & (x) & &IT_MASK_CPFWPDBG1<< BIT_SHIFT_WLFWPDBG1<define BIT_GET_BIFWPDBG1) & (x) &  BIT_SHIFT_USFWPDBG1& &IT_MASK_CPFWPDBG1<d* 2 REG_PAFWPDBG2OOOOffset 0x0010E */

#define BIT_CPIFT_USFWPDBG2 0define BIT_MASK_CPFWPDBG2 0fffffffL
Ldefine BIT_GEFWPDBG2) & (x) & &IT_MASK_CPFWPDBG2<< BIT_SHIFT_WLFWPDBG2<define BIT_CPGEGEFWPDBG2) & (x) &  BIT_SHIFT_USFWPDBG2& &IT_MASK_CPFWPDBG2<d* 2 REG_PAFWPDBG3OOOOffset 0x0010E */

#define BIT_STIFT_USFWPDBG3 0define BIT_MASK_CPFWPDBG3 0fffffffL
Ldefine BIT_GEFWPDBG3) & (x) & &IT_MASK_CPFWPDBG3<< BIT_SHIFT_WLFWPDBG3<define BIT_CPGEGEFWPDBG3) & (x) &  BIT_SHIFT_USFWPDBG3& &IT_MASK_CPFWPDBG3<d* 2 REG_PAFWPDBG4OOOOffset 0x0010F */

#define BIT_SHIFT_USFWPDBG4 0define BIT_MASK_CPFWPDBG4 0fffffffL
Ldefine BIT_GEFWPDBG4) & (x) & &IT_MASK_CPFWPDBG4<< BIT_SHIFT_WLFWPDBG4<define BIT_CPGEGEFWPDBG4) & (x) &  BIT_SHIFT_USFWPDBG4& &IT_MASK_CPFWPDBG4<d* 2 REG_PAFWPDBG5OOOOffset 0x0010F */

#define BIT_SHIFT_SDFWPDBG5 0define BIT_MASK_CPFWPDBG5 0fffffffL
Ldefine BIT_GEFWPDBG5) & (x) & &IT_MASK_CPFWPDBG5<< BIT_SHIFT_WLFWPDBG5<define BIT_CPGEGEFWPDBG5) & (x) &  BIT_SHIFT_USFWPDBG5& &IT_MASK_CPFWPDBG5<d* 2 REG_PAFWPDBG6OOOOffset 0x0010F */

#define BIT_CPIFT_USFWPDBG6 0define BIT_MASK_CPFWPDBG6 0fffffffL
Ldefine BIT_GEFWPDBG6) & (x) & &IT_MASK_CPFWPDBG6<< BIT_SHIFT_WLFWPDBG6<define BIT_CPGEGEFWPDBG6) & (x) &  BIT_SHIFT_USFWPDBG6& &IT_MASK_CPFWPDBG6<d* 2 REG_PAFWPDBG7OOOOffset 0x0010F */

#define BIT_STIFT_USFWPDBG7 0define BIT_MASK_CPFWPDBG7 0fffffffL
Ldefine BIT_GEFWPDBG7) & (x) & &IT_MASK_CPFWPDBG7<< BIT_SHIFT_WLFWPDBG7<define BIT_CPGEGEFWPDBG7) & (x) &  BIT_SHIFT_USFWPDBG7& &IT_MASK_CPFWPDBG7<d* 2 REG_PACR_EXTOOOOffset 0x00110 */

#define BIT_SHIFT_USPHY_REQ_DELAY 24define BIT_MASK_CPPHY_REQ_DELAY 0xfdefine BIT_SHPHY_REQ_DELAY) &                                                   	(((x) & &IT_MASK_CPPHY_REQ_DELAY<< BIT_SHIFT_WLPHY_REQ_DELAY<define BIT_CPGEGEPHY_REQ_DELAY) &                                               	(((x) &  BIT_SHIFT_USPHY_REQ_DELAY<<&IT_MASK_CPPHY_REQ_DELAY<#define BIT_SHIPD_DOWN T_C116<ddefine BIT_SHIFT_USNETYPE4 4define BIT_MASK_CPNETYPE4 0x3define BIT_MANETYPE4) & (x) & &IT_MASK_CPNETYPE4<< BIT_SHIFT_WLNETYPE4<define BIT_CPGEGENETYPE4) & (x) &  BIT_SHIFT_USNETYPE4<<&IT_MASK_CPNETYPE4<ddefine BIT_SHIFT_USNETYPE3 2define BIT_MASK_CPNETYPE3 0x3define BIT_MANETYPE3) & (x) & &IT_MASK_CPNETYPE3<< BIT_SHIFT_WLNETYPE3<define BIT_CPGEGENETYPE3) & (x) &  BIT_SHIFT_USNETYPE3& &IT_MASK_CPNETYPE3<ddefine BIT_SHIFT_USNETYPE2 0define BIT_MASK_CPNETYPE2 0x3define BIT_MANETYPE2) & (x) & &IT_MASK_CPNETYPE2<< BIT_SHIFT_WLNETYPE2<define BIT_CPGEGENETYPE2) & (x) &  BIT_SHIFT_USNETYPE2& &IT_MASK_CPNETYPE2<d* 2 REG_PAFWFFOOOOffset 0x00111 */

#define BIT_SHIFT_SDPKTNUMATH_V1 24define BIT_MASK_CPPKTNUMATH_V1 0xffdefine BIT_SHPKTNUMATH_V1) &                                                    	(((x) & &IT_MASK_CPPKTNUMATH_V1<< BIT_SHIFT_WLPKTNUMATH_V1<define BIT_CPGEGEPKTNUMATH_V1) &                                                	(((x) &  BIT_SHIFT_USPKTNUMATH_V1<<&IT_MASK_CPPKTNUMATH_V1<d* 2 REG_PAFWFFOOOOffset 0x00111 */

#define BIT_SHIFT_SDTIMERATH 1#define BIT_MASK_BITIMERATH 0xffdefine BIT_SHTIMERATH) & (x) & &IT_MASK_CPTIMERATH&  BIT_SHIFT_WLTIMERATH&define BIT_CPGEGETIMERATH) & (x) &  BIT_SHIFT_USTIMERATH& &IT_MASK_CPTIMERATH&d* 2 REG_PAFWFFOOOOffset 0x00111 */

#define BIT_SHIFT_SDGXPKT1ENADDR 0define BIT_MASK_CPGXPKT1ENADDR 0fffffdefine BIT_STRXPKT1ENADDR) &                                                    	(((x) & &IT_MASK_CPRXPKT1ENADDR<< BIT_SHIFT_WLRXPKT1ENADDR<define BIT_CPGEGERXPKT1ENADDR) &                                                	(((x) &  BIT_SHIFT_USRXPKT1ENADDR<<&IT_MASK_CPRXPKT1ENADDR<d* 2 REG_PAFE2IMROOOOffset 0x00112 */

#define BIT_SHAFE4ISR__IND_M_C T_C129<#* 2 REG_PAFE2IMROOOOffset 0x00112 */

#define BIT_SHFSPTXSCPDESCPDONEPINT_ENIT_C128)define BIT_MAFSPTXSCPBKDONEPINT_ENIT_C127)define BIT_MAFSPTXSCPBEDONEPINT_ENIT_C126)define BIT_MAFSPTXSCPVIDONEPINT_ENIT_C125<define BIT_MAFSPTXSCPVODONEPINT_ENIT_C124<#* 2 REG_PAFE2IMROOOOffset 0x00112 */

#define BIT_SHFSPATIM_MB7PINT_ENIT_C123<define BIT_MAFSPATIM_MB6PINT_ENIT_C122<define BIT_MAFSPATIM_MB5PINT_ENIT_C121<define BIT_CPFSPATIM_MB4PINT_ENIT_C120<define BIT_CPFSPATIM_MB3PINT_ENIT_C119<#efine BIT_SHFSPATIM_MB2PINT_ENIT_C118)define BIT_MAFSPATIM_MB1PINT_ENIT_C117)define BIT_MAFSPATIM_MB0PINT_ENIT_C116)define BIT_MAFSPTBTT4INT_ENIT_C111<define BIT_CPFSPTBTT3INT_ENIT_C110<define BIT_CPFSPTBTT2INT_ENIT_C19<#efine BIT_SHFSPTBTT1INT_ENIT_C18)define BIT_MAFSPTBTT0_MB7INT_ENIT_C17)define BIT_MAFSPTBTT0_MB6INT_ENIT_C16)define BIT_MAFSPTBTT0_MB5INT_ENIT_C15<define BIT_MAFSPTBTT0_MB4INT_ENIT_C14<define BIT_MAFSPTBTT0_MB3INT_ENIT_C13<define BIT_MAFSPTBTT0_MB2INT_ENIT_C12<define BIT_MAFSPTBTT0_MB1INT_ENIT_C11<define BIT_CPFSPTBTT0PINT_ENIT_C10<#* 2 REG_PAFE2ISROOOOffset 0x00112 */

#define BIT_SHAFE4ISR__IND_INT T_C129<#* 2 REG_PAFE2ISROOOOffset 0x00112 */

#define BIT_SHFSPTXSCPDESCPDONEPINTIT_C128)define BIT_MAFSPTXSCPBKDONEPINTIT_C127)define BIT_MAFSPTXSCPBEDONEPINTIT_C126)define BIT_MAFSPTXSCPVIDONEPINTIT_C125<define BIT_MAFSPTXSCPVODONEPINTIT_C124<#* 2 REG_PAFE2ISROOOOffset 0x00112 */

#define BIT_SHFSPATIM_MB7PINTIT_C123<define BIT_MAFSPATIM_MB6PINTIT_C122<define BIT_MAFSPATIM_MB5PINTIT_C121<define BIT_CPFSPATIM_MB4PINTIT_C120<define BIT_CPFSPATIM_MB3PINTIT_C119<#efine BIT_SHFSPATIM_MB2PINTIT_C118)define BIT_MAFSPATIM_MB1PINTIT_C117)define BIT_MAFSPATIM_MB0PINTIT_C116)define BIT_MAFSPTBTT4INTIT_C111<define BIT_CPFSPTBTT3INTIT_C110<define BIT_CPFSPTBTT2INTIT_C19<#efine BIT_SHFSPTBTT1INTIT_C18)define BIT_MAFSPTBTT0_MB7INTIT_C17)define BIT_MAFSPTBTT0_MB6INTIT_C16)define BIT_MAFSPTBTT0_MB5INTIT_C15<define BIT_MAFSPTBTT0_MB4INTIT_C14<define BIT_MAFSPTBTT0_MB3INTIT_C13<define BIT_MAFSPTBTT0_MB2INTIT_C12<define BIT_MAFSPTBTT0_MB1INTIT_C11<define BIT_CPFSPTBTT0PINTIT_C10<#* 2 REG_PAFE3IMROOOOffset 0x00112 */

#define BIT_CPFSPCLI3_MTIHBCNIVLEAR_INT__ENIT_C131<#* 2 REG_PAFE3IMROOOOffset 0x00112 */

#define BIT_CPFSPCLI2_MTIHBCNIVLEAR_INT__ENIT_C130<#* 2 REG_PAFE3IMROOOOffset 0x00112 */

#define BIT_CPFSPCLI1_MTIHBCNIVLEAR_INT__ENIT_C129<#* 2 REG_PAFE3IMROOOOffset 0x00112 */

#define BIT_CPFSPCLI0_MTIHBCNIVLEAR_INT__ENIT_C128<#* 2 REG_PAFE3IMROOOOffset 0x00112 */

#define BIT_CPFSPBCNDMA4PINT_ENIT_C127)define BIT_MAFSPBCNDMA3PINT_ENIT_C126)define BIT_MAFSPBCNDMA2PINT_ENIT_C125<define BIT_MAFSPBCNDMA1PINT_ENIT_C124<#efine BIT_CPFSPBCNDMA0_MB7PINT_ENIT_C123<define BIT_MAFSPBCNDMA0_MB6PINT_ENIT_C122<define BIT_MAFSPBCNDMA0_MB5PINT_ENIT_C121<define BIT_CPFSPBCNDMA0_MB4PINT_ENIT_C120<define BIT_CPFSPBCNDMA0_MB3PINT_ENIT_C119<#efine BIT_SHFSPBCNDMA0_MB2PINT_ENIT_C118)define BIT_MAFSPBCNDMA0_MB1PINT_ENIT_C117)define BIT_MAFSPBCNDMA0_INT_ENIT_C116)define BIT_MAFSPMTIHBCNIVLEAR_INT__ENIT_C115<define BIT_CPFSPBCNERLY4PINT_ENIT_C111<define BIT_CPFSPBCNERLY3PINT_ENIT_C110<define BIT_CPFSPBCNERLY2PINT_ENIT_C19<#efine BIT_SHFSPBCNERLY1PINT_ENIT_C18)define BIT_MAFSPBCNERLY0_MB7INT_ENIT_C17)define BIT_MAFSPBCNERLY0_MB6INT_ENIT_C16)define BIT_MAFSPBCNERLY0_MB5INT_ENIT_C15<define BIT_MAFSPBCNERLY0_MB4INT_ENIT_C14<define BIT_MAFSPBCNERLY0_MB3INT_ENIT_C13<define BIT_MAFSPBCNERLY0_MB2INT_ENIT_C12<define BIT_MAFSPBCNERLY0_MB1INT_ENIT_C11<define BIT_CPFSPBCNERLY0_INT_ENIT_C10<#* 2 REG_PAFE3ISROOOOffset 0x00112 */

#define BIT_STFSPCLI3_MTIHBCNIVLEAR_INTIT_C131<#* 2 REG_PAFE3ISROOOOffset 0x00112 */

#define BIT_STFSPCLI2_MTIHBCNIVLEAR_INTIT_C130<#* 2 REG_PAFE3ISROOOOffset 0x00112 */

#define BIT_STFSPCLI1_MTIHBCNIVLEAR_INTIT_C129<#* 2 REG_PAFE3ISROOOOffset 0x00112 */

#define BIT_STFSPCLI0_MTIHBCNIVLEAR_INTIT_C128<#* 2 REG_PAFE3ISROOOOffset 0x00112 */

#define BIT_STFSPBCNDMA4PINTIT_C127)define BIT_MAFSPBCNDMA3PINTIT_C126)define BIT_MAFSPBCNDMA2PINTIT_C125<define BIT_MAFSPBCNDMA1PINTIT_C124<#efine BIT_CPFSPBCNDMA0_MB7PINTIT_C123<define BIT_MAFSPBCNDMA0_MB6PINTIT_C122<define BIT_MAFSPBCNDMA0_MB5PINTIT_C121<define BIT_CPFSPBCNDMA0_MB4PINTIT_C120<define BIT_CPFSPBCNDMA0_MB3PINTIT_C119<#efine BIT_SHFSPBCNDMA0_MB2PINTIT_C118)define BIT_MAFSPBCNDMA0_MB1PINTIT_C117)define BIT_MAFSPBCNDMA0_INTIT_C116)define BIT_MAFSPMTIHBCNIVLEAR_INTIT_C115<define BIT_CPFSPBCNERLY4PINTIT_C111<define BIT_CPFSPBCNERLY3PINTIT_C110<define BIT_CPFSPBCNERLY2PINTIT_C19<#efine BIT_SHFSPBCNERLY1PINTIT_C18)define BIT_MAFSPBCNERLY0_MB7INTIT_C17)define BIT_MAFSPBCNERLY0_MB6INTIT_C16)define BIT_MAFSPBCNERLY0_MB5INTIT_C15<define BIT_MAFSPBCNERLY0_MB4INTIT_C14<define BIT_MAFSPBCNERLY0_MB3INTIT_C13<define BIT_MAFSPBCNERLY0_MB2INTIT_C12<define BIT_MAFSPBCNERLY0_MB1INTIT_C11<define BIT_CPFSPBCNERLY0_INTIT_C10<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI3_TXPKTINPINT_ENIT_C119<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI2_TXPKTINPINT_ENIT_C118<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI1_TXPKTINPINT_ENIT_C117<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI0_TXPKTINPINT_ENIT_C116<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI3_RX_UMD0_INT_ENIT_C115<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI3_RX_UMD1PINT_ENIT_C114<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI3_RX_BMD0_INT_ENIT_C113<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI3_RX_BMD1PINT_ENIT_C112<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI2_RX_UMD0_INT_ENIT_C111<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI2_RX_UMD1PINT_ENIT_C110<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI2_RX_BMD0_INT_ENIT_C19<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI2_RX_BMD1PINT_ENIT_C18<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI1_RX_UMD0_INT_ENIT_C17<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI1_RX_UMD1PINT_ENIT_C16<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI1_RX_BMD0_INT_ENIT_C15<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI1_RX_BMD1PINT_ENIT_C14<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI0_RX_UMD0_INT_ENIT_C13<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI0_RX_UMD1PINT_ENIT_C12<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI0_RX_BMD0_INT_ENIT_C11<#* 2 REG_PAFE4IMROOOOffset 0x00113 */

#define BIT_SHFSPCLI0_RX_BMD1PINT_ENIT_C10<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI3_TXPKTINPINTIT_C119<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI2_TXPKTINPINTIT_C118<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI1_TXPKTINPINTIT_C117<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI0_TXPKTINPINTIT_C116<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI3_RX_UMD0_INTIT_C115<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI3_RX_UMD1PINTIT_C114<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI3_RX_BMD0_INTIT_C113<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI3_RX_BMD1PINTIT_C112<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI2_RX_UMD0_INTIT_C111<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI2_RX_UMD1PINTIT_C110<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI2_RX_BMD0_INTIT_C19<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI2_RX_BMD1PINTIT_C18<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI1_RX_UMD0_INTIT_C17<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI1_RX_UMD1PINTIT_C16<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI1_RX_BMD0_INTIT_C15<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI1_RX_BMD1PINTIT_C14<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI0_RX_UMD0_INTIT_C13<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI0_RX_UMD1PINTIT_C12<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI0_RX_BMD0_INTIT_C11<#* 2 REG_PAFE4ISROOOOffset 0x00113 */

#define BIT_SHFSPCLI0_RX_BMD1PINTIT_C10<#* 2 REG_PAFT1IMROOOOffset 0x00113 */

#define BIT_CPAFT2ISR__IND_M_C T_C130<define BIT_GEFTM_PTTPINT_ENIT_C129<#efine BIT_SHRXFTMREQ_INT_ENIT_C128<#efine BIT_SHRXFTM_INT_ENIT_C127)define BIT_MATXFTM_INT_ENIT_C126<#* 2 REG_PAFT1IMROOOOffset 0x00113 */

#define BIT_CPFSPH2CICMD_OKPINT_ENIT_C125<define BIT_MAFSPH2CICMD_FULL_INT_ENIT_C124<#* 2 REG_PAFT1IMROOOOffset 0x00113 */

#define BIT_CPFSPMACIDPPWRCHANGE5PINT_ENIT_C123<define BIT_MAFSPMACIDPPWRCHANGE4PINT_ENIT_C122<define BIT_MAFSPMACIDPPWRCHANGE3PINT_ENIT_C121<define BIT_CPFSPMACIDPPWRCHANGE2PINT_ENIT_C120<define BIT_CPFSPMACIDPPWRCHANGE1PINT_ENIT_C119<#efine BIT_SHFSPMACIDPPWRCHANGE0_INT_ENIT_C118)define BIT_MAFSPCTWEND2_INT_ENIT_C117)define BIT_MAFSPCTWEND1PINT_ENIT_C116)define BIT_MAFSPCTWEND0_INT_ENIT_C115<#efine BIT_CPFSPTX_NULL1PINT_ENIT_C114<#efine BIT_CPFSPTX_NULL0_INT_ENIT_C113<#efine BIT_CPFSPTSF_T_C32_TOGGLE_ENIT_C112<#efine BIT_CPFSPP2P_RFON2_INT_ENIT_C111<define BIT_CPFSPP2P_RFOFF2_INT_ENIT_C110<define BIT_CPFSPP2P_RFON1PINT_ENIT_C19<#efine BIT_SHFSPP2P_RFOFF1PINT_ENIT_C18)define BIT_MAFSPP2P_RFON0_INT_ENIT_C17<#efine BIT_SHFSPP2P_RFOFF0_INT_ENIT_C16)define BIT_MAFSPRX_UAPSDMD1PENIT_C15<define BIT_MAFSPRX_UAPSDMD0_ENIT_C14<define BIT_MAFSPTRIGGERIPKTAENIT_C13<define BIT_MAFSPEOSPPINT_ENIT_C12<#efine BIT_MAFSPRPWM2_INT_ENIT_C11<#efine BIT_MAFSPRPWMPINT_ENIT_C10<#* 2 REG_PAFT1ISROOOOffset 0x00113 */

#define BIT_STAFT2ISR__IND_INTIT_C130<#efine BIT_GEFTM_PTTPINTIT_C129<#efine BIT_SHRXFTMREQ_INTIT_C128<#efine BIT_SHRXFTM_INTIT_C127)define BIT_MATXFTM_INTIT_C126<#* 2 REG_PAFT1ISROOOOffset 0x00113 */

#define BIT_STFSPH2CICMD_OKPINTIT_C125<define BIT_MAFSPH2CICMD_FULL_INTIT_C124<#* 2 REG_PAFT1ISROOOOffset 0x00113 */

#define BIT_STFSPMACIDPPWRCHANGE5PINTIT_C123<define BIT_MAFSPMACIDPPWRCHANGE4PINTIT_C122<define BIT_MAFSPMACIDPPWRCHANGE3PINTIT_C121<define BIT_CPFSPMACIDPPWRCHANGE2PINTIT_C120<define BIT_CPFSPMACIDPPWRCHANGE1PINTIT_C119<#efine BIT_SHFSPMACIDPPWRCHANGE0_INTIT_C118)define BIT_MAFSPCTWEND2_INTIT_C117)define BIT_MAFSPCTWEND1PINTIT_C116)define BIT_MAFSPCTWEND0_INTIT_C115<#efine BIT_CPFSPTX_NULL1PINTIT_C114<#efine BIT_CPFSPTX_NULL0_INTIT_C113<#efine BIT_CPFSPTSF_T_C32_TOGGLE_INTIT_C112<#efine BIT_CPFSPP2P_RFON2_INTIT_C111<define BIT_CPFSPP2P_RFOFF2_INTIT_C110<define BIT_CPFSPP2P_RFON1PINTIT_C19<#efine BIT_SHFSPP2P_RFOFF1PINTIT_C18)define BIT_MAFSPP2P_RFON0_INTIT_C17<#efine BIT_SHFSPP2P_RFOFF0_INTIT_C16)define BIT_MAFSPRX_UAPSDMD1PINTIT_C15<define BIT_MAFSPRX_UAPSDMD0_INTIT_C14<define BIT_MAFSPTRIGGERIPKTAINTIT_C13<define BIT_MAFSPEOSPPINTIT_C12<#efine BIT_MAFSPRPWM2_INTIT_C11<#efine BIT_MAFSPRPWMPINTIT_C10<#* 2 REG_PASPWR0OOOOffset 0x00114 */

#define BIT_SHIFT_USMIDP31TO0 0define BIT_MASK_CPMIDP31TO0 0fffffffL
Ldefine BIT_GEMIDP31TO0) & (x) & &IT_MASK_CPMIDP31TO0<< BIT_SHIFT_WLMIDP31TO0<define BIT_GEGEGEMIDP31TO0) & (x) &  BIT_SHIFT_USMIDP31TO0<<&IT_MASK_CPMIDP31TO0<#* 2 REG_PASPWR1OOOOffset 0x00114 */

#define BIT_SHIFT_SDMIDP63TO32 0define BIT_MASK_CPMIDP63TO32 0fffffffL
Ldefine BIT_GEMIDP63TO32) & (x) & &IT_MASK_CPMIDP63TO32<< BIT_SHIFT_WLMIDP63TO32<define BIT_GEGEGEMIDP63TO32) &                                                  	(((x) &  BIT_SHIFT_USMIDP63TO32<<&IT_MASK_CPMIDP63TO32<#* 2 REG_PASPWR2OOOOffset 0x00114 */

#define BIT_CPIFT_USMIDP95O64 0define BIT_MASK_CPMIDP95O64 0fffffffL
Ldefine BIT_GEMIDP95O64) & (x) & &IT_MASK_CPMIDP95O64<< BIT_SHIFT_WLMIDP95O64<define BIT_GEGEGEMIDP95O64) & (x) &  BIT_SHIFT_USMIDP95O64<<&IT_MASK_CPMIDP95O64<#* 2 REG_PASPWR3OOOOffset 0x00114 */

#define BIT_STIFT_USMIDP127TO96 0define BIT_MASK_CPMIDP127TO96 0fffffffL
Ldefine BIT_GEMIDP127TO96) &                                                     	(((x) & &IT_MASK_CPMIDP127TO96<< BIT_SHIFT_WLMIDP127TO96<define BIT_GEGEGEMIDP127TO96) &                                                 	(((x) &  BIT_SHIFT_USMIDP127TO96<<&IT_MASK_CPMIDP127TO96<#* 2 REG_PAPOWSEQOOOOffset 0x00115 */

#define BIT_SHIFT_USSEQNUMAMIDI1#define BIT_MASK_BISEQNUMAMIDI0fffffdefine BIT_STSEQNUMAMID) & (x) & &IT_MASK_CPSEQNUMAMID<< BIT_SHIFT_WLSEQNUMAMID<define BIT_GEGEGESEQNUMAMID) &                                                  	(((x) &  BIT_SHIFT_USSEQNUMAMID<<&IT_MASK_CPSEQNUMAMID<#define BIT_SHIFT_SDGEF_MID 0define BIT_MASK_CPGEF_MID 0x7fdefine BIT_CPREF_MID) & (x) & &IT_MASK_CPREF_MID<< BIT_SHIFT_WLREF_MID<define BIT_GEGEGEREF_MID) & (x) &  BIT_SHIFT_USREF_MID<<&IT_MASK_CPREF_MID<d* 2 REG_PATC7PCTRL_V1OOOOffset 0x00115 */

#define BIT_CPTC7INT_ENIT_C126<#efine BIT_CPTC7MODEIT_C125<define BIT_MATC7ENIT_C124<#*efine BIT_SHIFT_SDTC7DATA 0define BIT_MASK_CPTC7DATA 0fffffffdefine BIT_MATC7DATA) & (x) & &IT_MASK_CPTC7DATA&  BIT_SHIFT_WLTC7DATA&define BIT_GEGEGETC7DATA) & (x) &  BIT_SHIFT_USTC7DATA& &IT_MASK_CPTC7DATA&d* 2 REG_PATC8PCTRL_V1OOOOffset 0x00115 */

#define BIT_STTC8INT_ENIT_C126<#efine BIT_CPTC8MODEIT_C125<define BIT_MATC8ENIT_C124<#*efine BIT_SHIFT_SDTC8DATA 0define BIT_MASK_CPTC8DATA 0fffffffdefine BIT_MATC8DATA) & (x) & &IT_MASK_CPTC8DATA&  BIT_SHIFT_WLTC8DATA&define BIT_GEGEGETC8DATA) & (x) &  BIT_SHIFT_USTC8DATA& &IT_MASK_CPTC8DATA&d* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI3_RX_UAPSDMD1PENIT_C131<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI3_RX_UAPSDMD0_ENIT_C130<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI3_TRIGGERIPKTAENIT_C129<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI3_EOSPPINT_ENIT_C128<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_RX_UAPSDMD1PENIT_C127<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_RX_UAPSDMD0_ENIT_C126<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_TRIGGERIPKTAENIT_C125<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_EOSPPINT_ENIT_C124<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_RX_UAPSDMD1PENIT_C123<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_RX_UAPSDMD0_ENIT_C122<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_TRIGGERIPKTAENIT_C121<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_EOSPPINT_ENIT_C120<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI0_RX_UAPSDMD1PENIT_C119<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI0_RX_UAPSDMD0_ENIT_C118<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI0_TRIGGERIPKTAENIT_C117<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI0_EOSPPINT_ENIT_C116<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPTSF_T_C32_TOGGLE_P2P2_ENIT_C19<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPTSF_T_C32_TOGGLE_P2P1_ENIT_C18)d* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI3_TX_NULL1PINT_ENIT_C17<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI3PTX_NULL0_INT_ENIT_C16<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_TX_NULL1PINT_ENIT_C15<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_TX_NULL0_INT_ENIT_C14<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_TX_NULL1PINT_ENIT_C13<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_TX_NULL0_INT_ENIT_C12<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI0PTX_NULL1PINT_ENIT_C11<#* 2 REG_PAFT2IMROOOOffset 0x0011E */

#define BIT_SHFSPCLI0PTX_NULL0_INT_ENIT_C10<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI3_RX_UAPSDMD1PINTIT_C131<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI3_RX_UAPSDMD0_INTIT_C130<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI3_TRIGGERIPKTAINTIT_C129<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI3_EOSPPINTIT_C128)d* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_RX_UAPSDMD1PINTIT_C127<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_RX_UAPSDMD0_INTIT_C126<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_TRIGGERIPKTAINTIT_C125<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_EOSPPINTIT_C124<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_RX_UAPSDMD1PINTIT_C123<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_RX_UAPSDMD0_INTIT_C122<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_TRIGGERIPKTAINTIT_C121<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_EOSPPINTIT_C120<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI0_RX_UAPSDMD1PINTIT_C119<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI0_RX_UAPSDMD0_INTIT_C118)d* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI0_TRIGGERIPKTAINTIT_C117<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI0_EOSPPINTIT_C116<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPTSF_T_C32_TOGGLE_P2P2_INTIT_C19<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPTSF_T_C32_TOGGLE_P2P1PINTIT_C18<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI3_TX_NULL1PINTIT_C17<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI3PTX_NULL0_INTIT_C16<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_TX_NULL1PINTIT_C15<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI2_TX_NULL0_INTIT_C14<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_TX_NULL1PINTIT_C13<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI1_TX_NULL0_INTIT_C12<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI0PTX_NULL1PINTIT_C11<#* 2 REG_PAFT2ISROOOOffset 0x0011E */

#define BIT_SHFSPCLI0PTX_NULL0_INTIT_C10<#* 2 REG_PAMSG2OOOOffset 0x0011F */

#define BIT_SHIFT_USFWPMSG2 0define BIT_MASK_CPFWPMSG2 0fffffffL
Ldefine BIT_GEFWPMSG2) & (x) & &IT_MASK_CPFWPMSG2<< BIT_SHIFT_WLFWPMSG2<define BIT_GEGEGEFWPMSG2) & (x) &  BIT_SHIFT_USFWPMSG2<<&IT_MASK_CPFWPMSG2<#* 2 REG_PAMSG3OOOOffset 0x0011F */

#define BIT_SHIFT_SDFWPMSG3 0define BIT_MASK_CPFWPMSG3 0fffffffL
Ldefine BIT_GEFWPMSG3) & (x) & &IT_MASK_CPFWPMSG3<< BIT_SHIFT_WLFWPMSG3<define BIT_GEGEGEFWPMSG3) & (x) &  BIT_SHIFT_USFWPMSG3<<&IT_MASK_CPFWPMSG3<#* 2 REG_PAMSG4OOOOffset 0x0011F */

#define BIT_CPIFT_USFWPMSG4 0define BIT_MASK_CPFWPMSG4 0fffffffL
Ldefine BIT_GEFWPMSG4) & (x) & &IT_MASK_CPFWPMSG4<< BIT_SHIFT_WLFWPMSG4<define BIT_GEGEGEFWPMSG4) & (x) &  BIT_SHIFT_USFWPMSG4<<&IT_MASK_CPFWPMSG4<#* 2 REG_PAMSG5OOOOffset 0x0011F */

#define BIT_STIFT_USFWPMSG5 0define BIT_MASK_CPFWPMSG5 0fffffffL
Ldefine BIT_GEFWPMSG5) & (x) & &IT_MASK_CPFWPMSG5<< BIT_SHIFT_WLFWPMSG5<define BIT_GEGEGEFWPMSG5) & (x) &  BIT_SHIFT_USFWPMSG5<<&IT_MASK_CPFWPMSG5<#* 2 REG_PADDMA_CH0SAOOOOffset 0x00120 */

#define BIT_SHIFT_USDDMACH0_SA 0define BIT_MASK_CPDDMACH0_SA 0fffffffL
Ldefine BIT_GEDDMACH0_SA) & (x) & &IT_MASK_CPDDMACH0_SA<< BIT_SHIFT_WLDDMACH0_SA<define BIT_GEGEGEDDMACH0_SA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH0_SA<<&IT_MASK_CPDDMACH0_SA<#* 2 REG_PADDMA_CH0DAOOOOffset 0x00120 */

#define BIT_SHIFT_SDDDMACH0_DA 0define BIT_MASK_CPDDMACH0_DA 0fffffffL
Ldefine BIT_GEDDMACH0_DA) & (x) & &IT_MASK_CPDDMACH0_DA<< BIT_SHIFT_WLDDMACH0_DA<define BIT_GEGEGEDDMACH0_DA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH0_DA<<&IT_MASK_CPDDMACH0_DA<#* 2 REG_PADDMA_CH0CTRLOOOffset 0x00120 */

#define BIT_CPDDMACH0_OWN T_C131<define BIT_GEDDMACH0_CHKSUM_ENIT_C129<#efine BIT_SHDDMACH0_DA_WHDISABLEIT_C128<#efine BIT_SHDDMACH0_CHKSUM_STSIT_C127<#efine BIT_SHDDMACH0_DDMA_MODEIT_C126<#efine BIT_SHDDMACH0_RES_BICHKSUM_STSIT_C125<#efine BIT_SHDDMACH0_CHKSUM_CONTIT_C124<#*efine BIT_SHIFT_SDDDMACH0_DLEN 0define BIT_MASK_CPDDMACH0_DLEN 0x3ffffdefine BIT_CPDDMACH0_DLEN) &                                                    	(((x) & &IT_MASK_CPDDMACH0_DLEN<< BIT_SHIFT_WLDDMACH0_DLEN<define BIT_GEGEGEDDMACH0_DLEN) &                                                	(((x) &  BIT_SHIFT_USDDMACH0_DLEN<<&IT_MASK_CPDDMACH0_DLEN<#* 2 REG_PADDMA_CH1SAOOOOffset 0x00121 */

#define BIT_SHIFT_USDDMACH1_SA 0define BIT_MASK_CPDDMACH1_SA 0fffffffL
Ldefine BIT_GEDDMACH1_SA) & (x) & &IT_MASK_CPDDMACH1_SA<< BIT_SHIFT_WLDDMACH1_SA<define BIT_GEGEGEDDMACH1_SA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH1_SA<<&IT_MASK_CPDDMACH1_SA<#* 2 REG_PADDMA_CH1DAOOOOffset 0x00121 */

#define BIT_SHIFT_SDDDMACH1_DA 0define BIT_MASK_CPDDMACH1_DA 0fffffffL
Ldefine BIT_GEDDMACH1_DA) & (x) & &IT_MASK_CPDDMACH1_DA<< BIT_SHIFT_WLDDMACH1_DA<define BIT_GEGEGEDDMACH1_DA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH1_DA<<&IT_MASK_CPDDMACH1_DA<#* 2 REG_PADDMA_CH1CTRLOOOffset 0x00121 */

#define BIT_CPDDMACH1_OWN T_C131<define BIT_GEDDMACH1_CHKSUM_ENIT_C129<#efine BIT_SHDDMACH1_DA_WHDISABLEIT_C128<#efine BIT_SHDDMACH1_CHKSUM_STSIT_C127<#efine BIT_SHDDMACH1_DDMA_MODEIT_C126<#efine BIT_SHDDMACH1_RES_BICHKSUM_STSIT_C125<#efine BIT_SHDDMACH1_CHKSUM_CONTIT_C124<#*efine BIT_SHIFT_SDDDMACH1_DLEN 0define BIT_MASK_CPDDMACH1_DLEN 0x3ffffdefine BIT_CPDDMACH1_DLEN) &                                                    	(((x) & &IT_MASK_CPDDMACH1_DLEN<< BIT_SHIFT_WLDDMACH1_DLEN<define BIT_GEGEGEDDMACH1_DLEN) &                                                	(((x) &  BIT_SHIFT_USDDMACH1_DLEN<<&IT_MASK_CPDDMACH1_DLEN<#* 2 REG_PADDMA_CH2SAOOOOffset 0x00122 */

#define BIT_SHIFT_USDDMACH2_SA 0define BIT_MASK_CPDDMACH2_SA 0fffffffL
Ldefine BIT_GEDDMACH2_SA) & (x) & &IT_MASK_CPDDMACH2_SA<< BIT_SHIFT_WLDDMACH2_SA<define BIT_GEGEGEDDMACH2_SA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH2_SA<<&IT_MASK_CPDDMACH2_SA<#* 2 REG_PADDMA_CH2DAOOOOffset 0x00122 */

#define BIT_SHIFT_USDDMACH2_DA 0define BIT_MASK_CPDDMACH2_DA 0fffffffL
Ldefine BIT_GEDDMACH2_DA) & (x) & &IT_MASK_CPDDMACH2_DA<< BIT_SHIFT_WLDDMACH2_DA<define BIT_GEGEGEDDMACH2_DA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH2_DA<<&IT_MASK_CPDDMACH2_DA<#* 2 REG_PADDMA_CH2CTRLOOOffset 0x00122 */

#define BIT_CPDDMACH2_OWN T_C131<define BIT_GEDDMACH2_CHKSUM_ENIT_C129<#efine BIT_SHDDMACH2_DA_WHDISABLEIT_C128<#efine BIT_SHDDMACH2_CHKSUM_STSIT_C127<#efine BIT_SHDDMACH2_DDMA_MODEIT_C126<#efine BIT_SHDDMACH2_RES_BICHKSUM_STSIT_C125<#efine BIT_SHDDMACH2_CHKSUM_CONTIT_C124<#*efine BIT_SHIFT_SDDDMACH2_DLEN 0define BIT_MASK_CPDDMACH2_DLEN 0x3ffffdefine BIT_CPDDMACH2_DLEN) &                                                    	(((x) & &IT_MASK_CPDDMACH2_DLEN<< BIT_SHIFT_WLDDMACH2_DLEN<define BIT_GEGEGEDDMACH2_DLEN) &                                                	(((x) &  BIT_SHIFT_USDDMACH2_DLEN<<&IT_MASK_CPDDMACH2_DLEN<#* 2 REG_PADDMA_CH3SAOOOOffset 0x00123 */

#define BIT_SHIFT_USDDMACH3_SA 0define BIT_MASK_CPDDMACH3_SA 0fffffffL
Ldefine BIT_GEDDMACH3_SA) & (x) & &IT_MASK_CPDDMACH3_SA<< BIT_SHIFT_WLDDMACH3_SA<define BIT_GEGEGEDDMACH3_SA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH3_SA<<&IT_MASK_CPDDMACH3_SA<#* 2 REG_PADDMA_CH3DAOOOOffset 0x00123 */

#define BIT_SHIFT_USDDMACH3_DA 0define BIT_MASK_CPDDMACH3_DA 0fffffffL
Ldefine BIT_GEDDMACH3_DA) & (x) & &IT_MASK_CPDDMACH3_DA<< BIT_SHIFT_WLDDMACH3_DA<define BIT_GEGEGEDDMACH3_DA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH3_DA<<&IT_MASK_CPDDMACH3_DA<#* 2 REG_PADDMA_CH3CTRLOOOffset 0x00123 */

#define BIT_CPDDMACH3_OWN T_C131<define BIT_GEDDMACH3_CHKSUM_ENIT_C129<#efine BIT_SHDDMACH3_DA_WHDISABLEIT_C128<#efine BIT_SHDDMACH3_CHKSUM_STSIT_C127<#efine BIT_SHDDMACH3_DDMA_MODEIT_C126<#efine BIT_SHDDMACH3_RES_BICHKSUM_STSIT_C125<#efine BIT_SHDDMACH3_CHKSUM_CONTIT_C124<#*efine BIT_SHIFT_SDDDMACH3_DLEN 0define BIT_MASK_CPDDMACH3_DLEN 0x3ffffdefine BIT_CPDDMACH3_DLEN) &                                                    	(((x) & &IT_MASK_CPDDMACH3_DLEN<< BIT_SHIFT_WLDDMACH3_DLEN<define BIT_GEGEGEDDMACH3_DLEN) &                                                	(((x) &  BIT_SHIFT_USDDMACH3_DLEN<<&IT_MASK_CPDDMACH3_DLEN<#* 2 REG_PADDMA_CH4SAOOOOffset 0x00124 */

#define BIT_SHIFT_USDDMACH4_SA 0define BIT_MASK_CPDDMACH4_SA 0fffffffL
Ldefine BIT_GEDDMACH4_SA) & (x) & &IT_MASK_CPDDMACH4_SA<< BIT_SHIFT_WLDDMACH4_SA<define BIT_GEGEGEDDMACH4_SA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH4_SA<<&IT_MASK_CPDDMACH4_SA<#* 2 REG_PADDMA_CH4DAOOOOffset 0x00124 */

#define BIT_SHIFT_SDDDMACH4_DA 0define BIT_MASK_CPDDMACH4_DA 0fffffffL
Ldefine BIT_GEDDMACH4_DA) & (x) & &IT_MASK_CPDDMACH4_DA<< BIT_SHIFT_WLDDMACH4_DA<define BIT_GEGEGEDDMACH4_DA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH4_DA<<&IT_MASK_CPDDMACH4_DA<#* 2 REG_PADDMA_CH4CTRLOOOffset 0x00124 */

#define BIT_CPDDMACH4_OWN T_C131<define BIT_GEDDMACH4_CHKSUM_ENIT_C129<#efine BIT_SHDDMACH4_DA_WHDISABLEIT_C128<#efine BIT_SHDDMACH4_CHKSUM_STSIT_C127<#efine BIT_SHDDMACH4_DDMA_MODEIT_C126<#efine BIT_SHDDMACH4_RES_BICHKSUM_STSIT_C125<#efine BIT_SHDDMACH4_CHKSUM_CONTIT_C124<#*efine BIT_SHIFT_SDDDMACH4_DLEN 0define BIT_MASK_CPDDMACH4_DLEN 0x3ffffdefine BIT_CPDDMACH4_DLEN) &                                                    	(((x) & &IT_MASK_CPDDMACH4_DLEN<< BIT_SHIFT_WLDDMACH4_DLEN<define BIT_GEGEGEDDMACH4_DLEN) &                                                	(((x) &  BIT_SHIFT_USDDMACH4_DLEN<<&IT_MASK_CPDDMACH4_DLEN<#* 2 REG_PADDMA_CH5SAOOOOffset 0x00125 */

#define BIT_SHIFT_USDDMACH5_SA 0define BIT_MASK_CPDDMACH5_SA 0fffffffL
Ldefine BIT_GEDDMACH5_SA) & (x) & &IT_MASK_CPDDMACH5_SA<< BIT_SHIFT_WLDDMACH5_SA<define BIT_GEGEGEDDMACH5_SA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH5_SA<<&IT_MASK_CPDDMACH5_SA<#* 2 REG_PADDMA_CH5DAOOOOffset 0x00125 */

#define BIT_SHDDMACH5_OWN T_C131<define BIT_GEDDMACH5_CHKSUM_ENIT_C129<#efine BIT_SHDDMACH5_DA_WHDISABLEIT_C128<#efine BIT_SHDDMACH5_CHKSUM_STSIT_C127<#efine BIT_SHDDMACH5_DDMA_MODEIT_C126<#efine BIT_SHDDMACH5_RES_BICHKSUM_STSIT_C125<#efine BIT_SHDDMACH5_CHKSUM_CONTIT_C124<#*efine BIT_SHIFT_SDDDMACH5_DA 0define BIT_MASK_CPDDMACH5_DA 0fffffffL
Ldefine BIT_GEDDMACH5_DA) & (x) & &IT_MASK_CPDDMACH5_DA<< BIT_SHIFT_WLDDMACH5_DA<define BIT_GEGEGEDDMACH5_DA) &                                                  	(((x) &  BIT_SHIFT_USDDMACH5_DA<<&IT_MASK_CPDDMACH5_DA<d*efine BIT_SHIFT_SDDDMACH5_DLEN 0define BIT_MASK_CPDDMACH5_DLEN 0x3ffffdefine BIT_CPDDMACH5_DLEN) &                                                    	(((x) & &IT_MASK_CPDDMACH5_DLEN<< BIT_SHIFT_WLDDMACH5_DLEN<define BIT_GEGEGEDDMACH5_DLEN) &                                                	(((x) &  BIT_SHIFT_USDDMACH5_DLEN<<&IT_MASK_CPDDMACH5_DLEN<#* 2 REG_PADDMA_INT_MSKOOOffset 0x0012E */

#define BIT_SHDDMACH5_M_C T_C15<#efine BIT_SHDDMACH4_M_C T_C14<#efine BIT_SHDDMACH3_M_C T_C13<#efine BIT_SHDDMACH2_M_C T_C12<#efine BIT_SHDDMACH1_M_C T_C11<define BIT_GEDDMACH0_M_C T_C10<#* 2 REG_PADDMA_CHSTATUSOOOffset 0x0012E */

#define BIT_CPDDMACH5_BUSY T_C15<#efine BIT_SHDDMACH4_BUSY T_C14<#efine BIT_SHDDMACH3_BUSY T_C13<#efine BIT_SHDDMACH2_BUSY T_C12<#efine BIT_SHDDMACH1_BUSY T_C11<define BIT_GEDDMACH0_BUSY T_C10<#* 2 REG_PADDMA_CHKSUMOOOOffset 0x0012F */

#define BIT_SHIFT_USIDDMA0_CHKSUM 0define BIT_MASK_CPIDDMA0_CHKSUM 0fffffdefine BIT_STIDDMA0_CHKSUM) &                                                   	(((x) & &IT_MASK_CPIDDMA0_CHKSUM<< BIT_SHIFT_WLIDDMA0_CHKSUM<define BIT_GEGEGEIDDMA0_CHKSUM) &                                               	(((x) &  BIT_SHIFT_USIDDMA0_CHKSUM<<&IT_MASK_CPIDDMA0_CHKSUM<#* 2 REG_PADDMA_MONITOROOOffset 0x0012F */

#define BIT_STIDDMA0_PERMU_UNDERFLOW T_C114<#efine BIT_SHIDDMA0_FIFO_UNDERFLOW T_C113<#efine BIT_SHIDDMA0_FIFO_OVERFLOW T_C112<#efine BIT_SHECRC_EN_V1 B_C17<#efine BIT_SHMDIO_RFLAG_V1 B_C16<#efine BIT_SHCH5_ERR T_C15<#efine BIT_SHMDIO_WFLAG_V1 B_C15<#efine BIT_SHCH4_ERR T_C14<#efine BIT_SHCH3_ERR T_C13<#efine BIT_SHCH2_ERR T_C12<#efine BIT_SHCH1_ERR T_C11<#efine BIT_SHCH0_ERR T_C10<#* 2 REG_PASTC_INT_CSOOOOffset 0x00130 */

#define BIT_SHITC_INT_ENIT_C131<#*efine BIT_SHIFT_USITC_INT_FLAGI1#define BIT_MASK_BISTC_INT_FLAGI0xffdefine BIT_SHSTC_INT_FLAG) &                                                    	(((x) & &IT_MASK_CPSTC_INT_FLAG<< BIT_SHIFT_WLSTC_INT_FLAG<define BIT_GEGEGESTC_INT_FLAG) &                                                	(((x) &  BIT_SHIFT_USSTC_INT_FLAG<<&IT_MASK_CPSTC_INT_FLAG<#*efine BIT_SHIFT_USITC_INT_IDX 8define BIT_MASK_BISTC_INT_IDX 0x7define BIT_SHSTC_INT_IDX) &                                                     	(((x) & &IT_MASK_CPSTC_INT_IDX<< BIT_SHIFT_WLSTC_INT_IDX<define BIT_GEGEGESTC_INT_IDX) &                                                 	(((x) &  BIT_SHIFT_USSTC_INT_IDX<<&IT_MASK_CPSTC_INT_IDX<#*efine BIT_SHIFT_USITC_INT_REALTIME_CS 0define BIT_MASK_CPITC_INT_REALTIME_CS 0x3fdefine BIT_SHSTC_INT_REALTIME_CS) &                                             	(((x) & &IT_MASK_CPSTC_INT_REALTIME_CS<< BIT_SHIFT_WLSTC_INT_REALTIME_CS<define BIT_GEGEGESTC_INT_REALTIME_CS) &                                         	(((x) &  BIT_SHIFT_USSTC_INT_REALTIME_CS<<&IT_MASK_CPSTC_INT_REALTIME_CS<#* 2 REG_PAST_INT_CFGOOOOffset 0x00130 */

#define BIT_SHITC_INT_GRP_ENIT_C131<#*efine BIT_SHIFT_USITC_INT_EXPECT_LS 8define BIT_MASK_BISTC_INT_EXPECT_LS 0x3fdefine BIT_SHSTC_INT_EXPECT_LS) &                                               	(((x) & &IT_MASK_CPSTC_INT_EXPECT_LS<< BIT_SHIFT_WLSTC_INT_EXPECT_LS<define BIT_GEGEGESTC_INT_EXPECT_LS) &                                           	(((x) &  BIT_SHIFT_USSTC_INT_EXPECT_LS<<&IT_MASK_CPSTC_INT_EXPECT_LS<#*efine BIT_SHIFT_USITC_INT_EXPECT_CS 0define BIT_MASK_CPITC_INT_EXPECT_CS 0x3fdefine BIT_SHSTC_INT_EXPECT_CS) &                                               	(((x) & &IT_MASK_CPSTC_INT_EXPECT_CS<< BIT_SHIFT_WLSTC_INT_EXPECT_CS<define BIT_GEGEGESTC_INT_EXPECT_CS) &                                           	(((x) &  BIT_SHIFT_USSTC_INT_EXPECT_CS<<&IT_MASK_CPSTC_INT_EXPECT_CS<d* 2 REG_PACMU_DLY_CTRLOOOffset 0x00131 */

#define BIT_SHCMU_DLY_EN T_C131<define BIT_GECMU_DLY_MODEIT_C130<#*efine BIT_SHIFT_USCMU_DLY_PRE_DIV 0define BIT_MASK_CPCMU_DLY_PRE_DIV 0xffdefine BIT_SHCMU_DLY_PRE_DIV) &                                                 	(((x) & &IT_MASK_CPCMU_DLY_PRE_DIV<< BIT_SHIFT_WLCMU_DLY_PRE_DIV<define BIT_GEGEGECMU_DLY_PRE_DIV) &                                             	(((x) &  BIT_SHIFT_USCMU_DLY_PRE_DIV<<&IT_MASK_CPCMU_DLY_PRE_DIV<d* 2 REG_PACMU_DLY_CFGOOOOffset 0x00131 */

#define BIT_SHIFT_SDCMU_DLY_LTR_A2I 24define BIT_MASK_CPCMU_DLY_LTR_A2I 0xffdefine BIT_SHCMU_DLY_LTR_A2I) &                                                 	(((x) & &IT_MASK_CPCMU_DLY_LTR_A2I<< BIT_SHIFT_WLCMU_DLY_LTR_A2I<define BIT_GEGEGECMU_DLY_LTR_A2I) &                                             	(((x) &  BIT_SHIFT_USCMU_DLY_LTR_A2I<<&IT_MASK_CPCMU_DLY_LTR_A2I<#define BIT_SHIFT_SDCMU_DLY_LTR_I2AI1#define BIT_MASK_BICMU_DLY_LTR_I2AI0xffdefine BIT_SHCMU_DLY_LTR_I2A) &                                                 	(((x) & &IT_MASK_CPCMU_DLY_LTR_I2A<< BIT_SHIFT_WLCMU_DLY_LTR_I2A<define BIT_GEGEGECMU_DLY_LTR_I2A) &                                             	(((x) &  BIT_SHIFT_USCMU_DLY_LTR_I2A<<&IT_MASK_CPCMU_DLY_LTR_I2A<#define BIT_SHIFT_SDCMU_DLY_LTR_IDLE 8define BIT_MASK_BICMU_DLY_LTR_IDLE 0xffdefine BIT_SHCMU_DLY_LTR_IDLE) &                                                	(((x) & &IT_MASK_CPCMU_DLY_LTR_IDLE<< BIT_SHIFT_WLCMU_DLY_LTR_IDLE<define BIT_GEGEGECMU_DLY_LTR_IDLE) &                                            	(((x) &  BIT_SHIFT_USCMU_DLY_LTR_IDLE<<&IT_MASK_CPCMU_DLY_LTR_IDLE<#define BIT_SHIFT_SDCMU_DLY_LTR_ACT 0define BIT_MASK_CPCMU_DLY_LTR_ACT 0xffdefine BIT_SHCMU_DLY_LTR_ACT) &                                                 	(((x) & &IT_MASK_CPCMU_DLY_LTR_ACT<< BIT_SHIFT_WLCMU_DLY_LTR_ACT<define BIT_GEGEGECMU_DLY_LTR_ACT) &                                             	(((x) &  BIT_SHIFT_USCMU_DLY_LTR_ACT<<&IT_MASK_CPCMU_DLY_LTR_ACT<d* 2 REG_PAH2CQ_TXBDPDESAOOOffset 0x00132 */

#define BIT_SHIFT_USH2CQ_TXBDPDESA 0define BIT_MASK_CPH2CQ_TXBDPDESA 0fffffffL
ffffffL
Ldefine BIT_GEH2CQ_TXBDPDESA) &                                                  	(((x) & &IT_MASK_CPH2CQ_TXBDPDESA<< BIT_SHIFT_WLH2CQ_TXBDPDESA<define BIT_GEGEGEH2CQ_TXBDPDESA) &                                              	(((x) &  BIT_SHIFT_USH2CQ_TXBDPDESA<<&IT_MASK_CPH2CQ_TXBDPDESA<d* 2 REG_PAH2CQ_TXBDPNUMOOOffset 0x00132 */

#define BIT_CPPCIEAH2CQ_FLAGIT_C114<#* 2 REG_PAH2CQ_TXBDPNUMOOOffset 0x00132 */

#define BIT_CPIFT_USH2CQ_DESCPMODEI12define BIT_MASK_CPH2CQ_DESCPMODEI0x3define BIT_GEH2CQ_DESCPMODE) &                                                  	(((x) & &IT_MASK_CPH2CQ_DESCPMODE<< BIT_SHIFT_WLH2CQ_DESCPMODE<define BIT_GEGEGEH2CQ_DESCPMODE) &                                              	(((x) &  BIT_SHIFT_USH2CQ_DESCPMODE<<&IT_MASK_CPH2CQ_DESCPMODE<#define BIT_CPIFT_USH2CQ_DESCPNUM 0define BIT_MASK_CPH2CQ_DESCPNUM 0xfffdefine BIT_STH2CQ_DESCPNUM) &                                                   	(((x) & &IT_MASK_CPH2CQ_DESCPNUM<< BIT_SHIFT_WLH2CQ_DESCPNUM<define BIT_GEGEGEH2CQ_DESCPNUM) &                                               	(((x) &  BIT_SHIFT_USH2CQ_DESCPNUM<<&IT_MASK_CPH2CQ_DESCPNUM<d* 2 REG_PAH2CQ_TXBDPIDXOOOffset 0x00132 */

#define BIT_STIFT_USH2CQ_HW_IDX 1#define BIT_MASK_BIH2CQ_HW_IDX 0xfffdefine BIT_STH2CQ_HW_IDX) &                                                     	(((x) & &IT_MASK_CPH2CQ_HW_IDX<< BIT_SHIFT_WLH2CQ_HW_IDX<define BIT_GEGEGEH2CQ_HW_IDX) &                                                 	(((x) &  BIT_SHIFT_USH2CQ_HW_IDX<<&IT_MASK_CPH2CQ_HW_IDX<#define BIT_STIFT_USH2CQ_HOST_IDX 0define BIT_MASK_BIH2CQ_HOST_IDX 0xfffdefine BIT_STH2CQ_HOST_IDX) &                                                   	(((x) & &IT_MASK_CPH2CQ_HOST_IDX<< BIT_SHIFT_WLH2CQ_HOST_IDX<define BIT_GEGEGEH2CQ_HOST_IDX) &                                               	(((x) &  BIT_SHIFT_USH2CQ_HOST_IDX<<&IT_MASK_CPH2CQ_HOST_IDX<d* 2 REG_PAH2CQ_CSROOOOffset 0x00133 */

#define BIT_SHH2CQ_FULL T_C131<define BIT_GECLRIH2CQ_HOST_IDX BIC116)define BIT_MACLRIH2CQ_HW_IDX BIC18<#* 2 REG_PACHANGEPPCIEASPEEDOOOffset 0x00135 */

#define BIT_SHCHANGEPPCIEASPEED BIC118<#* 2 REG_PACHANGEPPCIEASPEEDOOOffset 0x00135 */

#define BIT_SHIFT_USGEN1SGEN2 1#define BIT_MASK_BIGEN1SGEN2 0x3define BIT_GEGEN1SGEN2) & (x) & &IT_MASK_CPGEN1SGEN2<< BIT_SHIFT_WLGEN1SGEN2<define BIT_GEGEGEGEN1SGEN2) & (x) &  BIT_SHIFT_USGEN1SGEN2<<&IT_MASK_CPGEN1SGEN2<#* 2 REG_PACHANGEPPCIEASPEEDOOOffset 0x00135 */

#define BIT_SHIFT_USAUTO_HANG_RELEASE 0define BIT_MASK_BIAUTO_HANG_RELEASE 0x7define BIT_SHAUTO_HANG_RELEASE) &                                               	(((x) & &IT_MASK_CPAUTO_HANG_RELEASE<< BIT_SHIFT_WLAUTO_HANG_RELEASE<define BIT_GEGEGEAUTO_HANG_RELEASE) &                                           	(((x) &  BIT_SHIFT_USAUTO_HANG_RELEASE<<&IT_MASK_CPAUTO_HANG_RELEASE<#* 2 REG_PAOLDPDEHANGOOOOffset 0x0013F */

#define BIT_SHOLDPDEHANG BIC11<#* 2 REG_PAQ0_Q1PINFOOOOOffset 0x00140 */

#define BIT_SHIFT_USAC1IPKTAINFO 1#define BIT_MASK_BIAC1IPKTAINFO 0xfffdefine BIT_STAC1IPKTAINFO) &                                                    	(((x) & &IT_MASK_CPAC1IPKTAINFO<< BIT_SHIFT_WLAC1IPKTAINFO<define BIT_GEGEGEAC1IPKTAINFO) &                                                	(((x) &  BIT_SHIFT_USAC1IPKTAINFO<<&IT_MASK_CPAC1IPKTAINFO<#define BIT_SHIFT_USAC0IPKTAINFO 0define BIT_MASK_BIAC0IPKTAINFO 0xfffdefine BIT_STAC0IPKTAINFO) &                                                    	(((x) & &IT_MASK_CPAC0IPKTAINFO<< BIT_SHIFT_WLAC0IPKTAINFO<define BIT_GEGEGEAC0IPKTAINFO) &                                                	(((x) &  BIT_SHIFT_USAC0IPKTAINFO<<&IT_MASK_CPAC0IPKTAINFO<d* 2 REG_PAQ2_Q3PINFOOOOOffset 0x00140 */

#define BIT_SHIFT_SDAC3IPKTAINFO 1#define BIT_MASK_BIAC3IPKTAINFO 0xfffdefine BIT_STAC3IPKTAINFO) &                                                    	(((x) & &IT_MASK_CPAC3IPKTAINFO<< BIT_SHIFT_WLAC3IPKTAINFO<define BIT_GEGEGEAC3IPKTAINFO) &                                                	(((x) &  BIT_SHIFT_USAC3IPKTAINFO<<&IT_MASK_CPAC3IPKTAINFO<#define BIT_SHIFT_USAC2IPKTAINFO 0define BIT_MASK_BIAC2IPKTAINFO 0xfffdefine BIT_STAC2IPKTAINFO) &                                                    	(((x) & &IT_MASK_CPAC2IPKTAINFO<< BIT_SHIFT_WLAC2IPKTAINFO<define BIT_GEGEGEAC2IPKTAINFO) &                                                	(((x) &  BIT_SHIFT_USAC2IPKTAINFO<<&IT_MASK_CPAC2IPKTAINFO<d* 2 REG_PAQ4_Q5PINFOOOOOffset 0x00140 */

#define BIT_CPIFT_USAC5IPKTAINFO 1#define BIT_MASK_BIAC5IPKTAINFO 0xfffdefine BIT_STAC5IPKTAINFO) &                                                    	(((x) & &IT_MASK_CPAC5IPKTAINFO<< BIT_SHIFT_WLAC5IPKTAINFO<define BIT_GEGEGEAC5IPKTAINFO) &                                                	(((x) &  BIT_SHIFT_USAC5IPKTAINFO<<&IT_MASK_CPAC5IPKTAINFO<#define BIT_SHIFT_USAC4IPKTAINFO 0define BIT_MASK_BIAC4IPKTAINFO 0xfffdefine BIT_STAC4IPKTAINFO) &                                                    	(((x) & &IT_MASK_CPAC4IPKTAINFO<< BIT_SHIFT_WLAC4IPKTAINFO<define BIT_GEGEGEAC4IPKTAINFO) &                                                	(((x) &  BIT_SHIFT_USAC4IPKTAINFO<<&IT_MASK_CPAC4IPKTAINFO<d* 2 REG_PAQ6_Q7PINFOOOOOffset 0x00140 */

#define BIT_STIFT_USAC7IPKTAINFO 1#define BIT_MASK_BIAC7IPKTAINFO 0xfffdefine BIT_STAC7IPKTAINFO) &                                                    	(((x) & &IT_MASK_CPAC7IPKTAINFO<< BIT_SHIFT_WLAC7IPKTAINFO<define BIT_GEGEGEAC7IPKTAINFO) &                                                	(((x) &  BIT_SHIFT_USAC7IPKTAINFO<<&IT_MASK_CPAC7IPKTAINFO<#define BIT_SHIFT_USAC6IPKTAINFO 0define BIT_MASK_BIAC6IPKTAINFO 0xfffdefine BIT_STAC6IPKTAINFO) &                                                    	(((x) & &IT_MASK_CPAC6IPKTAINFO<< BIT_SHIFT_WLAC6IPKTAINFO<define BIT_GEGEGEAC6IPKTAINFO) &                                                	(((x) &  BIT_SHIFT_USAC6IPKTAINFO<<&IT_MASK_CPAC6IPKTAINFO<d* 2 REG_PAMGQ_HIQPINFOOOOffset 0x00141 */

#define BIT_SHIFT_USHIQPPKTAINFO 1#define BIT_MASK_BIHIQPPKTAINFO 0xfffdefine BIT_STHIQPPKTAINFO) &                                                    	(((x) & &IT_MASK_CPHIQPPKTAINFO<< BIT_SHIFT_WLHIQPPKTAINFO<define BIT_GEGEGEHIQPPKTAINFO) &                                                	(((x) &  BIT_SHIFT_USHIQPPKTAINFO<<&IT_MASK_CPHIQPPKTAINFO<#define BIT_STIFT_USMGQIPKTAINFO 0define BIT_MASK_BIMGQIPKTAINFO 0xfffdefine BIT_STMGQIPKTAINFO) &                                                    	(((x) & &IT_MASK_CPMGQIPKTAINFO<< BIT_SHIFT_WLMGQPPKTAINFO<define BIT_GEGEGEMGQIPKTAINFO) &                                                	(((x) &  BIT_SHIFT_USMGQPPKTAINFO<<&IT_MASK_CPMGQPPKTAINFO<d* 2 REG_PACMDQ_BCNQPINFOOOOffset 0x00141 */

#define BIT_SHIFT_SDCMDQPPKTAINFO 1#define BIT_MASK_BICMDQPPKTAINFO 0xfffdefine BIT_STCMDQPPKTAINFO) &                                                   	(((x) & &IT_MASK_CPCMDQPPKTAINFO<< BIT_SHIFT_WLCMDQPPKTAINFO<define BIT_GEGEGECMDQPPKTAINFO) &                                               	(((x) &  BIT_SHIFT_USCMDQPPKTAINFO<<&IT_MASK_CPCMDQPPKTAINFO<d* 2 REG_PACMDQ_BCNQPINFOOOOffset 0x00141 */

#define BIT_SHIFT_SDBCNQPPKTAINFO 0define BIT_MASK_BIBCNQPPKTAINFO 0xfffdefine BIT_STBCNQPPKTAINFO) &                                                   	(((x) & &IT_MASK_CPBCNQPPKTAINFO<< BIT_SHIFT_WLBCNQPPKTAINFO<define BIT_GEGEGEBCNQPPKTAINFO) &                                               	(((x) &  BIT_SHIFT_USBCNQPPKTAINFO<<&IT_MASK_CPBCNQPPKTAINFO<d* 2 REG_PAUSEG_PASETTINGOOOffset 0x00142 */

#define BIT_SHNDPAAUSEG_P BIC121<#*efine BIT_SHIFT_USRETRYAUSEG_P 19define BIT_MASK_BIRETRYAUSEG_P 0x3define BIT_GERETRYAUSEG_P) &                                                    	(((x) & &IT_MASK_CPRETRYAUSEG_P<< BIT_SHIFT_WLRETRYAUSEG_P<define BIT_GEGEGERETRYAUSEG_P) &                                                	(((x) &  BIT_SHIFT_USRETRYAUSEG_P<<&IT_MASK_CPRETRYAUSEG_P<#*efine BIT_SHIFT_SDTRYPKTAUSEG_P 17define BIT_MASK_BITRYPKTAUSEG_P 0x3define BIT_GETRYPKTAUSEG_P) &                                                   	(((x) & &IT_MASK_CPTRYPKTAUSEG_P&  BIT_SHIFT_WLTRYPKTAUSEG_P&define BIT_GEGEGETRYPKTAUSEG_P) &                                               	(((x) &  BIT_SHIFT_USTRYPKTAUSEG_P& &IT_MASK_CPTRYPKTAUSEG_P&#define BIT_SHCTLPKTAUSEG_P B_C116<#* 2 REG_PAAESIVASETTINGOOOffset 0x00142 */

#define BIT_SHIFT_SDAESIVAOFFSET 0define BIT_MASK_BIAESIVAOFFSET 0xfffdefine BIT_STAESIVAOFFSET) &                                                    	(((x) & &IT_MASK_CPAESIVAOFFSET<< BIT_SHIFT_WLAESIVAOFFSET<define BIT_GEGEGEAESIVAOFFSET) &                                                	(((x) &  BIT_SHIFT_USAESIVAOFFSET<<&IT_MASK_CPAESIVAOFFSET<#* 2 REG_PABF0_TIME_SETTINGOOOffset 0x00142 */

#define BIT_CPBF0_TIMER_SET B_C131<define BIT_GEBF0_TIMER_CLR T_C130<define BIT_GEBF0_UPDATE_ENIT_C129<#efine BIT_SHBF0_TIMER_ENIT_C128<#*efine BIT_SHIFT_SDBF0_PRETIME_OVER 1#define BIT_MASK_BIBF0_PRETIME_OVER 0xfffdefine BIT_STBF0_PRETIME_OVER) &                                                	(((x) & &IT_MASK_CPBF0_PRETIME_OVER<< BIT_SHIFT_WLBF0_PRETIME_OVER<define BIT_GEGEGEBF0_PRETIME_OVER) &                                            	(((x) &  BIT_SHIFT_USBF0_PRETIME_OVER<<&IT_MASK_CPBF0_PRETIME_OVER<#*efine BIT_SHIFT_SDBF0_LIFETIME 0define BIT_MASK_BIBF0_LIFETIME 0fffffdefine BIT_STBF0_LIFETIME) &                                                    	(((x) & &IT_MASK_CPBF0_LIFETIME<< BIT_SHIFT_WLBF0_LIFETIME<define BIT_GEGEGEBF0_LIFETIME) &                                                	(((x) &  BIT_SHIFT_USBF0_LIFETIME<<&IT_MASK_CPBF0_LIFETIME<#* 2 REG_PABF1_TIME_SETTINGOOOffset 0x00142 */

#define BIT_STBF1_TIMER_SET B_C131<define BIT_GEBF1_TIMER_CLR T_C130<define BIT_GEBF1_UPDATE_ENIT_C129<#efine BIT_SHBF1_TIMER_ENIT_C128<#*efine BIT_SHIFT_SDBF1_PRETIME_OVER 1#define BIT_MASK_BIBF1_PRETIME_OVER 0xfffdefine BIT_STBF1_PRETIME_OVER) &                                                	(((x) & &IT_MASK_CPBF1_PRETIME_OVER<< BIT_SHIFT_WLBF1_PRETIME_OVER<define BIT_GEGEGEBF1_PRETIME_OVER) &                                            	(((x) &  BIT_SHIFT_USBF1_PRETIME_OVER<<&IT_MASK_CPBF1_PRETIME_OVER<#*efine BIT_SHIFT_SDBF1_LIFETIME 0define BIT_MASK_BIBF1_LIFETIME 0fffffdefine BIT_STBF1_LIFETIME) &                                                    	(((x) & &IT_MASK_CPBF1_LIFETIME<< BIT_SHIFT_WLBF1_LIFETIME<define BIT_GEGEGEBF1_LIFETIME) &                                                	(((x) &  BIT_SHIFT_USBF1_LIFETIME<<&IT_MASK_CPBF1_LIFETIME<#* 2 REG_PABF_TIMEOUT_ENOOOffset 0x00143 */

#define BIT_SHEN_VHT_LDPCIT_C19<#efine BIT_SHEN_HT_LDPCIT_C18<#efine BIT_SHBF1_TIMEOUT_EN B_C11<#efine BIT_SHBF0_TIMEOUT_EN B_C10<#* 2 REG_PAMACIDPRELEASE0OOOffset 0x00143 */

#define BIT_SHIFT_SDMACID31_0_RELEASE 0define BIT_MASK_BIMACID31_0_RELEASE 0fffffffL
Ldefine BIT_GEMACID31_0_RELEASE) &                                               	(((x) & &IT_MASK_CPMACID31_0_RELEASE<< BIT_SHIFT_WLMACID31_0_RELEASE<define BIT_GEGEGEMACID31_0_RELEASE) &                                           	(((x) &  BIT_SHIFT_USMACID31_0_RELEASE<<&IT_MASK_CPMACID31_0_RELEASE<#* 2 REG_PAMACIDPRELEASE1OOOffset 0x00143 */

#define BIT_CPIFT_USMACID63_32_RELEASE 0define BIT_MASK_BIMACID63_32_RELEASE 0fffffffL
Ldefine BIT_GEMACID63_32_RELEASE) &                                              	(((x) & &IT_MASK_CPMACID63_32_RELEASE<< BIT_SHIFT_WLMACID63_32_RELEASE<define BIT_GEGEGEMACID63_32_RELEASE) &                                          	(((x) &  BIT_SHIFT_USMACID63_32_RELEASE<<&IT_MASK_CPMACID63_32_RELEASE<#* 2 REG_PAMACIDPRELEASE2OOOffset 0x00143 */

#define BIT_STIFT_USMACID95_64_RELEASE 0define BIT_MASK_BIMACID95_64_RELEASE 0fffffffL
Ldefine BIT_GEMACID95_64_RELEASE) &                                              	(((x) & &IT_MASK_CPMACID95_64_RELEASE<< BIT_SHIFT_WLMACID95_64_RELEASE<define BIT_GEGEGEMACID95_64_RELEASE) &                                          	(((x) &  BIT_SHIFT_USMACID95_64_RELEASE<<&IT_MASK_CPMACID95_64_RELEASE<#* 2 REG_PAMACIDPRELEASE3OOOffset 0x00144 */

#define BIT_SHIFT_USMACID127_96_RELEASE 0define BIT_MASK_BIMACID127_96_RELEASE 0fffffffL
Ldefine BIT_GEMACID127_96_RELEASE) &                                             	(((x) & &IT_MASK_CPMACID127_96_RELEASE<< BIT_SHIFT_WLMACID127_96_RELEASE<define BIT_GEGEGEMACID127_96_RELEASE) &                                         	(((x) &  BIT_SHIFT_USMACID127_96_RELEASE<<&IT_MASK_CPMACID127_96_RELEASE<#* 2 REG_PAMACIDPRELEASE_SETTINGOOffset 0x00144 */

#define BIT_SHMACIDPVALUE B_C17<#define BIT_STIFT_USMACIDPOFFSET 0define BIT_MASK_BIMACIDPOFFSET 0x7fdefine BIT_CPMACIDPOFFSET) &                                                    	(((x) & &IT_MASK_CPMACIDPOFFSET<< BIT_SHIFT_WLMACIDAOFFSET<define BIT_GEGEGEMACIDPOFFSET) &                                                	(((x) &  BIT_SHIFT_USMACIDAOFFSET<<&IT_MASK_CPMACIDAOFFSET<d* 2 REG_PAFAST_EDCA_VOVI_SETTINGOOffset 0x00144 */

#define BIT_CPIFT_USVI_FAST_EDCA_TO 24define BIT_MASK_CPVI_FAST_EDCA_TO 0xffdefine BIT_SHVI_FAST_EDCA_TO) &                                                 	(((x) & &IT_MASK_CPVI_FAST_EDCA_TO<< BIT_SHIFT_WLVI_FAST_EDCA_TO<define BIT_GEGEGEVI_FAST_EDCA_TO) &                                             	(((x) &  BIT_SHIFT_USVI_FAST_EDCA_TO<<&IT_MASK_CPVI_FAST_EDCA_TO<
define BIT_SHVI_THRESHOLDPSEL T_C123<#*efine BIT_CPIFT_USVI_FAST_EDCA_PKTATH 1#define BIT_MASK_BIVI_FAST_EDCA_PKTATH 0x7fdefine BIT_CPVI_FAST_EDCA_PKTATH) &                                             	(((x) & &IT_MASK_CPVI_FAST_EDCA_PKTATH<< BIT_SHIFT_WLVI_FAST_EDCA_PKTATH<define BIT_GEGEGEVI_FAST_EDCA_PKTATH) &                                         	(((x) &  BIT_SHIFT_USVI_FAST_EDCA_PKTATH<<&IT_MASK_CPVI_FAST_EDCA_PKTATH<#*efine BIT_CPIFT_USVO_FAST_EDCA_TO 8define BIT_MASK_BIVO_FAST_EDCA_TO 0xffdefine BIT_SHVO_FAST_EDCA_TO) &                                                 	(((x) & &IT_MASK_CPVO_FAST_EDCA_TO<< BIT_SHIFT_WLVO_FAST_EDCA_TO<define BIT_GEGEGEVO_FAST_EDCA_TO) &                                             	(((x) &  BIT_SHIFT_USVO_FAST_EDCA_TO<<&IT_MASK_CPVO_FAST_EDCA_TO<
define BIT_SHVO_THRESHOLDPSEL T_C17<#define BIT_STIFT_USVO_FAST_EDCA_PKTATH 0define BIT_MASK_BIVO_FAST_EDCA_PKTATH 0x7fdefine BIT_CPVO_FAST_EDCA_PKTATH) &                                             	(((x) & &IT_MASK_CPVO_FAST_EDCA_PKTATH<< BIT_SHIFT_WLVO_FAST_EDCA_PKTATH<define BIT_GEGEGEVO_FAST_EDCA_PKTATH) &                                         	(((x) &  BIT_SHIFT_USVO_FAST_EDCA_PKTATH<<&IT_MASK_CPVO_FAST_EDCA_PKTATH<d* 2 REG_PAFAST_EDCA_BEBK_SETTINGOOffset 0x00144 */

#define BIT_STIFT_USBK_FAST_EDCA_TO 24define BIT_MASK_CPBK_FAST_EDCA_TO 0xffdefine BIT_SHBK_FAST_EDCA_TO) &                                                 	(((x) & &IT_MASK_CPBK_FAST_EDCA_TO<< BIT_SHIFT_WLBK_FAST_EDCA_TO<define BIT_GEGEGEBK_FAST_EDCA_TO) &                                             	(((x) &  BIT_SHIFT_USBK_FAST_EDCA_TO<<&IT_MASK_CPBK_FAST_EDCA_TO<
define BIT_SHBK_THRESHOLDPSEL T_C123<#*efine BIT_CPIFT_USBK_FAST_EDCA_PKTATH 1#define BIT_MASK_BIBK_FAST_EDCA_PKTATH 0x7fdefine BIT_CPBK_FAST_EDCA_PKTATH) &                                             	(((x) & &IT_MASK_CPBK_FAST_EDCA_PKTATH<< BIT_SHIFT_WLBK_FAST_EDCA_PKTATH<define BIT_GEGEGEBK_FAST_EDCA_PKTATH) &                                         	(((x) &  BIT_SHIFT_USBK_FAST_EDCA_PKTATH<<&IT_MASK_CPBK_FAST_EDCA_PKTATH<d*efine BIT_CPIFT_USBE_FAST_EDCA_TO 8define BIT_MASK_BIBE_FAST_EDCA_TO 0xffdefine BIT_SHBE_FAST_EDCA_TO) &                                                 	(((x) & &IT_MASK_CPBE_FAST_EDCA_TO<< BIT_SHIFT_WLBE_FAST_EDCA_TO<define BIT_GEGEGEBE_FAST_EDCA_TO) &                                             	(((x) &  BIT_SHIFT_USBE_FAST_EDCA_TO<<&IT_MASK_CPBE_FAST_EDCA_TO<
define BIT_SHBE_THRESHOLDPSEL T_C17<#define BIT_STIFT_USBE_FAST_EDCA_PKTATH 0define BIT_MASK_BIBE_FAST_EDCA_PKTATH 0x7fdefine BIT_CPBE_FAST_EDCA_PKTATH) &                                             	(((x) & &IT_MASK_CPBE_FAST_EDCA_PKTATH<< BIT_SHIFT_WLBE_FAST_EDCA_PKTATH<define BIT_GEGEGEBE_FAST_EDCA_PKTATH) &                                         	(((x) &  BIT_SHIFT_USBE_FAST_EDCA_PKTATH<<&IT_MASK_CPBE_FAST_EDCA_PKTATH<d* 2 REG_PAMACIDPDROP0OOOOffset 0x00145 */

#define BIT_SHIFT_USMACID31_0_DROP 0define BIT_MASK_BIMACID31_0_DROP 0fffffffL
Ldefine BIT_GEMACID31_0_DROP) &                                                  	(((x) & &IT_MASK_CPMACID31_0_DROP<< BIT_SHIFT_WLMACID31_0_DROP<define BIT_GEGEGEMACID31_0_DROP) &                                              	(((x) &  BIT_SHIFT_USMACID31_0_DROP<<&IT_MASK_CPMACID31_0_DROP<d* 2 REG_PAMACIDPDROP1OOOOffset 0x00145 */

#define BIT_SHIFT_SDMACID63_32_DROP 0define BIT_MASK_BIMACID63_32_DROP 0fffffffL
Ldefine BIT_GEMACID63_32_DROP) &                                                 	(((x) & &IT_MASK_CPMACID63_32_DROP<< BIT_SHIFT_WLMACID63_32_DROP<define BIT_GEGEGEMACID63_32_DROP) &                                             	(((x) &  BIT_SHIFT_USMACID63_32_DROP<<&IT_MASK_CPMACID63_32_DROP<d* 2 REG_PAMACIDPDROP2OOOOffset 0x00145 */

#define BIT_CPIFT_USMACID95_64_DROP 0define BIT_MASK_BIMACID95_64_DROP 0fffffffL
Ldefine BIT_GEMACID95_64_DROP) &                                                 	(((x) & &IT_MASK_CPMACID95_64_DROP<< BIT_SHIFT_WLMACID95_64_DROP<define BIT_GEGEGEMACID95_64_DROP) &                                             	(((x) &  BIT_SHIFT_USMACID95_64_DROP<<&IT_MASK_CPMACID95_64_DROP<d* 2 REG_PAMACIDPDROP3OOOOffset 0x00145 */

#define BIT_STIFT_USMACID127_96_DROP 0define BIT_MASK_BIMACID127_96_DROP 0fffffffL
Ldefine BIT_GEMACID127_96_DROP) &                                                	(((x) & &IT_MASK_CPMACID127_96_DROP<< BIT_SHIFT_WLMACID127_96_DROP<define BIT_GEGEGEMACID127_96_DROP) &                                            	(((x) &  BIT_SHIFT_USMACID127_96_DROP<<&IT_MASK_CPMACID127_96_DROP<d* 2 REG_PARAMACIDPRELEASE_SUCCESS_0OOffset 0x00146 */

#define BIT_SHIFT_USRAMACIDPRELEASE_SUCCESS_0 0define BIT_MASK_BIRAMACIDPRELEASE_SUCCESS_0 0fffffffL
Ldefine BIT_GERAMACIDPRELEASE_SUCCESS_0) &                                       	(((x) & &IT_MASK_CPRAMACIDPRELEASE_SUCCESS_0&                            	((< BIT_SHIFT_WLRAMACIDPRELEASE_SUCCESS_0&define BIT_GEGEGERAMACIDPRELEASE_SUCCESS_0) &                                   	(((x) &  BIT_SHIFT_USRAMACIDPRELEASE_SUCCESS_0& &                        	((<T_MASK_CPRAMACIDPRELEASE_SUCCESS_0&d* 2 REG_PARAMACIDPRELEASE_SUCCESS_1OOffset 0x00146 */

#define BIT_SHIFT_SDRAMACIDPRELEASE_SUCCESS_1 0define BIT_MASK_BIRAMACIDPRELEASE_SUCCESS_1 0fffffffL
Ldefine BIT_GERAMACIDPRELEASE_SUCCESS_1) &                                       	(((x) & &IT_MASK_CPRAMACIDPRELEASE_SUCCESS_1&                            	((< BIT_SHIFT_WLRAMACIDPRELEASE_SUCCESS_1&define BIT_GEGEGERAMACIDPRELEASE_SUCCESS_1) &                                   	(((x) &  BIT_SHIFT_USRAMACIDPRELEASE_SUCCESS_1& &                        	((<T_MASK_CPRAMACIDPRELEASE_SUCCESS_1&d* 2 REG_PARAMACIDPRELEASE_SUCCESS_2OOffset 0x00146 */

#define BIT_CPIFT_USRAMACIDPRELEASE_SUCCESS_2 0define BIT_MASK_BIRAMACIDPRELEASE_SUCCESS_2 0fffffffL
Ldefine BIT_GERAMACIDPRELEASE_SUCCESS_2) &                                       	(((x) & &IT_MASK_CPRAMACIDPRELEASE_SUCCESS_2&                            	((< BIT_SHIFT_WLRAMACIDPRELEASE_SUCCESS_2<define BIT_GEGEGERAMACIDPRELEASE_SUCCESS_2) &                                   	(((x) &  BIT_SHIFT_USRAMACIDPRELEASE_SUCCESS_2& &                        	((<T_MASK_CPRAMACIDPRELEASE_SUCCESS_2&d* 2 REG_PARAMACIDPRELEASE_SUCCESS_3OOffset 0x00146 */

#define BIT_STIFT_USRAMACIDPRELEASE_SUCCESS_3 0define BIT_MASK_BIRAMACIDPRELEASE_SUCCESS_3 0fffffffL
Ldefine BIT_GERAMACIDPRELEASE_SUCCESS_3) &                                       	(((x) & &IT_MASK_CPRAMACIDPRELEASE_SUCCESS_3&                            	((< BIT_SHIFT_WLRAMACIDPRELEASE_SUCCESS_3<define BIT_GEGEGERAMACIDPRELEASE_SUCCESS_3) &                                   	(((x) &  BIT_SHIFT_USRAMACIDPRELEASE_SUCCESS_3& &                        	((<T_MASK_CPRAMACIDPRELEASE_SUCCESS_3<d* 2 REG_PAMGG_FIFO_CRTLOOOffset 0x00147 */

#define BIT_SHRAMGG_FIFO_ENIT_C131<#*efine BIT_SHIFT_USRAMGG_FIFO_PG_SIZE 28define BIT_MASK_BIRAMGG_FIFO_PG_SIZE 0x7define BIT_SHRAMGG_FIFO_PG_SIZE) &                                              	(((x) & &IT_MASK_CPRAMGG_FIFO_PG_SIZE<< BIT_SHIFT_WLRAMGG_FIFO_PG_SIZE<define BIT_GEGEGERAMGG_FIFO_PG_SIZE) &                                          	(((x) &  BIT_SHIFT_USRAMGG_FIFO_PG_SIZE<<&IT_MASK_CPRAMGG_FIFO_PG_SIZE<#*efine BIT_SHIFT_USRAMGG_FIFO_START_PG 1#define BIT_MASK_BIRAMGG_FIFO_START_PG 0xfffdefine BIT_STRAMGG_FIFO_START_PG) &                                             	(((x) & &IT_MASK_CPRAMGG_FIFO_START_PG<< BIT_SHIFT_WLRAMGG_FIFO_START_PG<define BIT_GEGEGERAMGG_FIFO_START_PG) &                                         	(((x) &  BIT_SHIFT_USRAMGG_FIFO_START_PG<<&IT_MASK_CPRAMGG_FIFO_START_PG<#*efine BIT_SHIFT_USRAMGG_FIFO_SIZE 14define BIT_MASK_CPRAMGG_FIFO_SIZE 0x3define BIT_GERAMGG_FIFO_SIZE) &                                                 	(((x) & &IT_MASK_CPRAMGG_FIFO_SIZE<< BIT_SHIFT_WLRAMGG_FIFO_SIZE<define BIT_GEGEGERAMGG_FIFO_SIZE) &                                             	(((x) &  BIT_SHIFT_USRAMGG_FIFO_SIZE<<&IT_MASK_CPRAMGG_FIFO_SIZE<#*efine BIT_SHRAMGG_FIFO_PAUSE B_C113<#*efine BIT_CPIFT_USRAMGG_FIFO_RPTR 8define BIT_MASK_BIRAMGG_FIFO_RPTR 001fdefine BIT_STRAMGG_FIFO_RPTR) &                                                 	(((x) & &IT_MASK_CPRAMGG_FIFO_RPTR<< BIT_SHIFT_WLRAMGG_FIFO_RPTR<define BIT_GEGEGERAMGG_FIFO_RPTR) &                                             	(((x) &  BIT_SHIFT_USRAMGG_FIFO_RPTR<<&IT_MASK_CPRAMGG_FIFO_RPTR<#*efine BIT_SHRAMGG_FIFO_OV T_C17<#efine BIT_SHRAMGG_FIFO_WPTR_ERROR T_C16<#efine BIT_SHRAEN_CPU_LIFETIME B_C15<#*efine BIT_CPIFT_USRAMGG_FIFO_WPTR 0define BIT_MASK_BIRAMGG_FIFO_WPTR 001fdefine BIT_STRAMGG_FIFO_WPTR) &                                                 	(((x) & &IT_MASK_CPRAMGG_FIFO_WPTR<< BIT_SHIFT_WLRAMGG_FIFO_WPTR<define BIT_GEGEGERAMGG_FIFO_WPTR) &                                             	(((x) &  BIT_SHIFT_USRAMGG_FIFO_WPTR<<&IT_MASK_CPRAMGG_FIFO_WPTR<d* 2 REG_PAMGG_FIFO_INTOOOffset 0x00147 */

#define BIT_SHIFT_SDRAMGG_FIFO_INT_FLAGI1#define BIT_MASK_BIRAMGG_FIFO_INT_FLAGI0fffffdefine BIT_STRAMGG_FIFO_INT_FLAG) &                                             	(((x) & &IT_MASK_CPRAMGG_FIFO_INT_FLAG<< BIT_SHIFT_WLRAMGG_FIFO_INT_FLAG<define BIT_GEGEGERAMGG_FIFO_INT_FLAG) &                                         	(((x) &  BIT_SHIFT_USRAMGG_FIFO_INT_FLAG<<&IT_MASK_CPRAMGG_FIFO_INT_FLAG<ddefine BIT_SHIFT_SDRAMGG_FIFO_INT_SK_C 0define BIT_MASK_BIRAMGG_FIFO_INT_SK_C 0fffffdefine BIT_STRAMGG_FIFO_INT_SK_C) &                                             	(((x) & &IT_MASK_CPRAMGG_FIFO_INT_SK_C<< BIT_SHIFT_WLRAMGG_FIFO_INT_SK_C<define BIT_GEGEGERAMGG_FIFO_INT_SK_C) &                                         	(((x) &  BIT_SHIFT_USRAMGG_FIFO_INT_SK_C<<&IT_MASK_CPRAMGG_FIFO_INT_SK_C<d* 2 REG_PAMGG_FIFO_LIFETIMEOOOffset 0x00147 */

#define BIT_CPIFT_USRAMGG_FIFO_LIFETIMEI1#define BIT_MASK_BIRAMGG_FIFO_LIFETIME 0fffffdefine BIT_STRAMGG_FIFO_LIFETIME) &                                             	(((x) & &IT_MASK_CPRAMGG_FIFO_LIFETIME<< BIT_SHIFT_WLRAMGG_FIFO_LIFETIME<define BIT_GEGEGERAMGG_FIFO_LIFETIME) &                                         	(((x) &  BIT_SHIFT_USRAMGG_FIFO_LIFETIME<<&IT_MASK_CPRAMGG_FIFO_LIFETIME<ddefine BIT_CPIFT_USRAMGG_FIFO_VALIDPMAP 0define BIT_MASK_BIRAMGG_FIFO_VALIDPMAP 0fffffdefine BIT_STRAMGG_FIFO_VALIDPMAP) &                                            	(((x) & &IT_MASK_CPRAMGG_FIFO_VALIDPMAP&                                 	((< BIT_SHIFT_WLRAMGG_FIFO_VALIDPMAP&define BIT_GEGEGERAMGG_FIFO_VALIDPMAP) &                                        	(((x) &  BIT_SHIFT_USRAMGG_FIFO_VALIDPMAP& &                             	((<T_MASK_CPRAMGG_FIFO_VALIDPMAP&d* 2 REG_PARAMACIDPRELEASE_SUCCESS_CLEARPOFFSET ffset 0x00147 */

#define BIT_STIFT_USRAMACIDPRELEASE_SUCCESS_CLEARPOFFSET 0define BIT_MASK_BIRAMACIDPRELEASE_SUCCESS_CLEARPOFFSET 0x7fdefine BIT_CPRAMACIDPRELEASE_SUCCESS_CLEARPOFFSET) &                            	(((x) & &IT_MASK_CPRAMACIDPRELEASE_SUCCESS_CLEARPOFFSET&                 	((< BIT_SHIFT_WLRAMACIDPRELEASE_SUCCESS_CLEARPOFFSET&define BIT_GEGEGERAMACIDPRELEASE_SUCCESS_CLEARPOFFSET) &                        	(((x) &  BIT_SHIFT_USRAMACIDPRELEASE_SUCCESS_CLEARPOFFSET& &             	((IT_MASK_CPRAMACIDPRELEASE_SUCCESS_CLEARPOFFSET&#define BIT_STIFT_USP2PONHDIS_TXTIME 0define BIT_MASK_BIP2PONHDIS_TXTIME 0xffdefine BIT_SHP2PONHDIS_TXTIME) &                                                	(((x) & &IT_MASK_CPP2PONHDIS_TXTIME<< BIT_SHIFT_WLP2PONHDIS_TXTIME<define BIT_GEGEGEP2PONHDIS_TXTIME) &                                            	(((x) &  BIT_SHIFT_USP2PONHDIS_TXTIME<<&IT_MASK_CPP2PONHDIS_TXTIME<d* 2 REG_PAMACIDPSHCUTPOFFSETOOOffset 0x00148 */

#define BIT_SHIFT_USMACIDPSHCUTPOFFSET_V1 0define BIT_MASK_BIMACIDPSHCUTPOFFSET_V1 0xffdefine BIT_SHMACIDPSHCUTPOFFSET_V1) &                                           	(((x) & &IT_MASK_CPMACIDASHCUTPOFFSET_V1&                                	((< BIT_SHIFT_WLMACIDASHCUTPOFFSET_V1&define BIT_GEGEGEMACIDPSHCUTPOFFSET_V1) &                                       	(((x) &  BIT_SHIFT_USMACIDASHCUTPOFFSET_V1& &                            	((IT_MASK_CPMACIDASHCUTPOFFSET_V1&d* 2 REG_PAMU_TXHCTLOOOOffset 0x0014C */

#define BIT_SHRAEN_REVERS_GTAB T_C16<#define BIT_STIFT_USRAMU_TABLE_VALID 0define BIT_MASK_BIRAMU_TABLE_VALID 0x3fdefine BIT_SHRAMU_TABLE_VALID) &                                                	(((x) & &IT_MASK_CPRAMU_TABLE_VALID<< BIT_SHIFT_WLRAMU_TABLE_VALID<define BIT_GEGEGERAMU_TABLE_VALID) &                                            	(((x) &  BIT_SHIFT_USRAMU_TABLE_VALID<<&IT_MASK_CPRAMU_TABLE_VALID<#define BIT_STIFT_USRAMU_STA_GTAB_VALID 0define BIT_MASK_BIRAMU_STA_GTAB_VALID 0fffffffL
Ldefine BIT_GERAMU_STA_GTAB_VALID) &                                             	(((x) & &IT_MASK_CPRAMU_STA_GTAB_VALID<< BIT_SHIFT_WLRAMU_STA_GTAB_VALID<define BIT_GEGEGERAMU_STA_GTAB_VALID) &                                         	(((x) &  BIT_SHIFT_USRAMU_STA_GTAB_VALID<<&IT_MASK_CPRAMU_STA_GTAB_VALID<#define BIT_STIFT_USRAMU_STA_GTAB_POSITION 0define BIT_MASK_CPRAMU_STA_GTAB_POSITION 0fffffffL
ffffffL
Ldefine BIT_GERAMU_STA_GTAB_POSITION) &                                          	(((x) & &IT_MASK_CPRAMU_STA_GTAB_POSITION&                               	((< BIT_SHIFT_WLRAMU_STA_GTAB_POSITION&define BIT_GEGEGERAMU_STA_GTAB_POSITION) &                                      	(((x) &  BIT_SHIFT_USRAMU_STA_GTAB_POSITION& &                           	((IT_MASK_CPRAMU_STA_GTAB_POSITION&d* 2 REG_PAMU_TRX_DBG_CNTOOOffset 0x0014D */

#define BIT_SHMU_DNGCNT_RST B_C120<#*efine BIT_SHIFT_USMU_DBGCNT_SEL 1#define BIT_MASK_BIMU_DBGCNT_SEL 0ffdefine BIT_SHMU_DBGCNT_SEL) &                                                   	(((x) & &IT_MASK_CPMU_DBGCNT_SEL<< BIT_SHIFT_WLMU_DBGCNT_SEL<define BIT_GEGEGEMU_DBGCNT_SEL) &                                               	(((x) &  BIT_SHIFT_USMU_DBGCNT_SEL<<&IT_MASK_CPMU_DBGCNT_SEL<#*efine BIT_SHIFT_USMU_DNGCNT 0define BIT_MASK_BIMU_DNGCNT 0fffffdefine BIT_STMU_DNGCNT) & (x) & &IT_MASK_CPMU_DNGCNT<< BIT_SHIFT_WLMU_DNGCNT<define BIT_GEGEGEMU_DNGCNT) & (x) &  BIT_SHIFT_USMU_DNGCNT<<&IT_MASK_CPMU_DNGCNT<d* 2 REG_PACPUMGQPTXHTIMEROOOffset 0x00150 */

#define BIT_SHIFT_USCPUMGQPTXHTIMER_V1 0define BIT_MASK_BICPUMGQPTXHTIMER_V1 0fffffffL
Ldefine BIT_GECPUMGQPTXHTIMER_V1) &                                              	(((x) & &IT_MASK_CPCPUMGQPTXHTIMER_V1<< BIT_SHIFT_WLCPUMGQPTXHTIMER_V1<define BIT_GEGEGECPUMGQPTXHTIMER_V1) &                                          	(((x) &  BIT_SHIFT_USCPUMGQPTXHTIMER_V1<<&IT_MASK_CPCPUMGQPTXHTIMER_V1<d* 2 REG_PAPSHTIMER_A	OOOffset 0x00150 */

#define BIT_SHIFT_SDPSHTIMER_A_V1 0define BIT_MASK_BIPSHTIMER_A_V1 0fffffffL
Ldefine BIT_GEPSHTIMER_A_V1) &                                                   	(((x) & &IT_MASK_CPPSHTIMER_A_V1<< BIT_SHIFT_WLPSHTIMER_A_V1<define BIT_GEGEGEPSHTIMER_A_V1) &                                               	(((x) &  BIT_SHIFT_USPSHTIMER_A_V1<<&IT_MASK_CPPSHTIMER_A_V1<d* 2 REG_PAPSHTIMER_B	OOOffset 0x00150 */

#define BIT_CPIFT_USPSHTIMER_B_V1 0define BIT_MASK_BIPSHTIMER_B_V1 0fffffffL
Ldefine BIT_GEPSHTIMER_B_V1) &                                                   	(((x) & &IT_MASK_CPPSHTIMER_B_V1<< BIT_SHIFT_WLPSHTIMER_B_V1<define BIT_GEGEGEPSHTIMER_B_V1) &                                               	(((x) &  BIT_SHIFT_USPSHTIMER_B_V1<<&IT_MASK_CPPSHTIMER_B_V1<d* 2 REG_PAPSHTIMER_C	OOOffset 0x00150 */

#define BIT_STIFT_USPSHTIMER_C_V1 0define BIT_MASK_BIPSHTIMER_C_V1 0fffffffL
Ldefine BIT_GEPSHTIMER_C_V1) &                                                   	(((x) & &IT_MASK_CPPSHTIMER_C_V1<< BIT_SHIFT_WLPSHTIMER_C_V1<define BIT_GEGEGEPSHTIMER_C_V1) &                                               	(((x) &  BIT_SHIFT_USPSHTIMER_C_V1<<&IT_MASK_CPPSHTIMER_C_V1<d* 2 REG_PAPSHTIMER_ABCPCPUMGQPTIMER_CRTLOffset 0x00151 */

#define BIT_SHCPUMGQPTIMER_EN T_C131<define BIT_GECPUMGQPTXHENIT_C128<#*efine BIT_SHIFT_SDCPUMGQPTIMER_TSF_SEL 24define BIT_MASK_CPCPUMGQPTIMER_TSF_SEL 0x7define BIT_SHCPUMGQPTIMER_TSF_SEL) &                                            	(((x) & &IT_MASK_CPCPUMGQPTIMER_TSF_SEL&                                 	((< BIT_SHIFT_WLCPUMGQPTIMER_TSF_SEL&define BIT_GEGEGECPUMGQPTIMER_TSF_SEL) &                                        	(((x) &  BIT_SHIFT_USCPUMGQPTIMER_TSF_SEL& &                             	((<T_MASK_CPCPUMGQPTIMER_TSF_SEL&ddefine BIT_GEPSHTIMER_C_ENIT_C123<#*efine BIT_CPIFT_USPSHTIMER_C_TSF_SEL 1#define BIT_MASK_BIPSHTIMER_C_TSF_SEL 0x7define BIT_SHPSHTIMER_C_TSF_SEL) &                                              	(((x) & &IT_MASK_CPPSHTIMER_C_TSF_SEL<< BIT_SHIFT_WLPSHTIMER_C_TSF_SEL&define BIT_GEGEGEPSHTIMER_C_TSF_SEL) &                                          	(((x) &  BIT_SHIFT_USPSHTIMER_C_TSF_SEL& & T_MASK_CPPSHTIMER_C_TSF_SEL<ddefine BIT_GEPSHTIMER_B_EN B_C115<#*efine BIT_CPIFT_USPSHTIMER_B_TSF_SEL 8define BIT_MASK_BIPSHTIMER_B_TSF_SEL 0x7define BIT_SHPSHTIMER_B_TSF_SEL) &                                              	(((x) & &IT_MASK_CPPSHTIMER_B_TSF_SEL<< BIT_SHIFT_WLPSHTIMER_B_TSF_SEL&define BIT_GEGEGEPSHTIMER_B_TSF_SEL) &                                          	(((x) &  BIT_SHIFT_USPSHTIMER_B_TSF_SEL& & T_MASK_CPPSHTIMER_B_TSF_SEL<ddefine BIT_GEPSHTIMER_A_EN B_C17<#define BIT_STIFT_USPSHTIMER_A_TSF_SEL 0define BIT_MASK_BIPSHTIMER_A_TSF_SEL 0x7define BIT_SHPSHTIMER_A_TSF_SEL) &                                              	(((x) & &IT_MASK_CPPSHTIMER_A_TSF_SEL<< BIT_SHIFT_WLPSHTIMER_A_TSF_SEL&define BIT_GEGEGEPSHTIMER_A_TSF_SEL) &                                          	(((x) &  BIT_SHIFT_USPSHTIMER_A_TSF_SEL& & T_MASK_CPPSHTIMER_A_TSF_SEL&d* 2 REG_PACPUMGQPTXHTIMER_EARLYOOffset 0x00151 */

#define BIT_SHIFT_SDCPUMGQPTXHTIMER_EARLY 0define BIT_MASK_BICPUMGQPTXHTIMER_EARLY 0xffdefine BIT_SHCPUMGQPTXHTIMER_EARLY) &                                           	(((x) & &IT_MASK_CPCPUMGQPTXHTIMER_EARLY&                                	((< BIT_SHIFT_WLCPUMGQPTXHTIMER_EARLY&define BIT_GEGEGECPUMGQPTXHTIMER_EARLY) &                                       	(((x) &  BIT_SHIFT_USCPUMGQPTXHTIMER_EARLY& &                            	((IT_MASK_CPCPUMGQPTXHTIMER_EARLY&d* 2 REG_PAPSHTIMER_A_EARLYOOOffset 0x001515*/

#define BIT_SHIFT_SDPSHTIMER_A_EARLY 0define BIT_MASK_BIPSHTIMER_A_EARLY 0xffdefine BIT_SHPSHTIMER_A_EARLY) &                                                	(((x) & &IT_MASK_CPPSHTIMER_A_EARLY<< BIT_SHIFT_WLPSHTIMER_A_EARLY&define BIT_GEGEGEPSHTIMER_A_EARLY) &                                            	(((x) &  BIT_SHIFT_USPSHTIMER_A_EARLY& & T_MASK_CPPSHTIMER_A_EARLY<d* 2 REG_PAPSHTIMER_B_EARLYOOOffset 0x001516*/

#define BIT_CPIFT_USPSHTIMER_B_EARLY 0define BIT_MASK_BIPSHTIMER_B_EARLY 0xffdefine BIT_SHPSHTIMER_B_EARLY) &                                                	(((x) & &IT_MASK_CPPSHTIMER_B_EARLY<< BIT_SHIFT_WLPSHTIMER_B_EARLY&define BIT_GEGEGEPSHTIMER_B_EARLY) &                                            	(((x) &  BIT_SHIFT_USPSHTIMER_B_EARLY& & T_MASK_CPPSHTIMER_B_EARLY<d* 2 REG_PAPSHTIMER_C_EARLYOOOffset 0x001517*/

#define BIT_STIFT_USPSHTIMER_C_EARLY 0define BIT_MASK_BIPSHTIMER_C_EARLY 0xffdefine BIT_SHPSHTIMER_C_EARLY) &                                                	(((x) & &IT_MASK_CPPSHTIMER_C_EARLY<< BIT_SHIFT_WLPSHTIMER_C_EARLY&define BIT_GEGEGEPSHTIMER_C_EARLY) &                                            	(((x) &  BIT_SHIFT_USPSHTIMER_C_EARLY& & T_MASK_CPPSHTIMER_C_EARLY<d* 2 REG_PABCNPPSR_RPT2OOOffset 0x00160 */

#define BIT_SHIFT_USDTIM_CNT2 24define BIT_MASK_CPDTIM_CNT2 0xffdefine BIT_SHDTIM_CNT2) & (x) & &IT_MASK_CPDTIM_CNT2<< BIT_SHIFT_WLDTIM_CNT2<define BIT_GEGEGEDTIM_CNT2) & (x) &  BIT_SHIFT_USDTIM_CNT2<<&IT_MASK_CPDTIM_CNT2<#define BIT_SHIFT_USDTIM_PERIOD2 1#define BIT_MASK_BIDTIM_PERIOD2 0xffdefine BIT_SHDTIM_PERIOD2) &                                                    	(((x) & &IT_MASK_CPDTIM_PERIOD2<< BIT_SHIFT_WLDTIM_PERIOD2<define BIT_GEGEGEDTIM_PERIOD2) &                                                	(((x) &  BIT_SHIFT_USDTIM_PERIOD2<<&IT_MASK_CPDTIM_PERIOD2<
define BIT_SHDTIM2 B_C115<#efine BIT_SHTIM2 B_C114<#define BIT_STIFT_USPSHAID_2 0define BIT_MASK_BIPSHAID_2 0x7ffdefine BIT_SHPSHAID_2) & (x) & &IT_MASK_CPPSHAID_2<< BIT_SHIFT_WLPSHAID_2<define BIT_GEGEGEPSHAID_2) & (x) &  BIT_SHIFT_USPSHAID_2<<&IT_MASK_CPPSHAID_2<d* 2 REG_PABCNPPSR_RPT3OOOffset 0x00160 */

#define BIT_SHIFT_SDDTIM_CNT3 24define BIT_MASK_CPDTIM_CNT3 0xffdefine BIT_SHDTIM_CNT3) & (x) & &IT_MASK_CPDTIM_CNT3<< BIT_SHIFT_WLDTIM_CNT3<define BIT_GEGEGEDTIM_CNT3) & (x) &  BIT_SHIFT_USDTIM_CNT3& &IT_MASK_CPDTIM_CNT3<#define BIT_SHIFT_USDTIM_PERIOD3 1#define BIT_MASK_BIDTIM_PERIOD3 0xffdefine BIT_SHDTIM_PERIOD3) &                                                    	(((x) & &IT_MASK_CPDTIM_PERIOD3<< BIT_SHIFT_WLDTIM_PERIOD3<define BIT_GEGEGEDTIM_PERIOD3) &                                                	(((x) &  BIT_SHIFT_USDTIM_PERIOD3& &IT_MASK_CPDTIM_PERIOD3<ddefine BIT_SHDTIM3 B_C115<#efine BIT_SHTIM3 B_C114<#define BIT_STIFT_USPSHAID_3 0define BIT_MASK_BIPSHAID_3 0x7ffdefine BIT_SHPSHAID_3) & (x) & &IT_MASK_CPPSHAID_3<< BIT_SHIFT_WLPSHAID_3<define BIT_GEGEGEPSHAID_3) & (x) &  BIT_SHIFT_USPSHAID_3& &IT_MASK_CPPSHAID_3<d* 2 REG_PABCNPPSR_RPT4OOOffset 0x00160 */

#define BIT_CPIFT_USDTIM_CNT4 24define BIT_MASK_CPDTIM_CNT4 0xffdefine BIT_SHDTIM_CNT4) & (x) & &IT_MASK_CPDTIM_CNT4<< BIT_SHIFT_WLDTIM_CNT4<define BIT_GEGEGEDTIM_CNT4) & (x) &  BIT_SHIFT_USDTIM_CNT4& &IT_MASK_CPDTIM_CNT4<#define BIT_SHIFT_USDTIM_PERIOD4 1#define BIT_MASK_BIDTIM_PERIOD4 0xffdefine BIT_SHDTIM_PERIOD4) &                                                    	(((x) & &IT_MASK_CPDTIM_PERIOD4<< BIT_SHIFT_WLDTIM_PERIOD4<define BIT_GEGEGEDTIM_PERIOD4) &                                                	(((x) &  BIT_SHIFT_USDTIM_PERIOD4& &IT_MASK_CPDTIM_PERIOD4<ddefine BIT_SHDTIM4 B_C115<#efine BIT_SHTIM4 B_C114<#define BIT_STIFT_USPSHAID_4 0define BIT_MASK_BIPSHAID_4 0x7ffdefine BIT_SHPSHAID_4) & (x) & &IT_MASK_CPPSHAID_4<< BIT_SHIFT_WLPSHAID_4<define BIT_GEGEGEPSHAID_4) & (x) &  BIT_SHIFT_USPSHAID_4& &IT_MASK_CPPSHAID_4<#* 2 REG_PAA1_ADDRASK_COOOffset 0x00160 */

#define BIT_STIFT_USA1_ADDRASK_C 0define BIT_MASK_BIA1_ADDRASK_C 0fffffffL
Ldefine BIT_GEA1_ADDRASK_C) &                                                    	(((x) & &IT_MASK_CPA1_ADDRASK_C<< BIT_SHIFT_WLA1_ADDRASK_C<define BIT_GEGEGEA1_ADDRASK_C) &                                                	(((x) &  BIT_SHIFT_USA1_ADDRASK_C<<&IT_MASK_CPA1_ADDRASK_C<d* 2 REG_PAMACID2OOOOffset 0x00162 */

#define BIT_SHIFT_USMACID2 0define BIT_MASK_BIMACID2 0fffffffL
ffffLdefine BIT_GEMACID2) & (x) & &IT_MASK_CPMACID2<< BIT_SHIFT_WLMACID2<define BIT_GEGEGEMACID2) & (x) &  BIT_SHIFT_USMACID2& &IT_MASK_CPMACID2<d* 2 REG_PABSSID2OOOOffset 0x00162 */

#define BIT_CPIFT_USBSSID2 0define BIT_MASK_BIBSSID2 0fffffffL
ffffLdefine BIT_GEBSSID2) & (x) & &IT_MASK_CPBSSID2<< BIT_SHIFT_WLBSSID2<define BIT_GEGEGEBSSID2) & (x) &  BIT_SHIFT_USBSSID2<<&IT_MASK_CPBSSID2<d* 2 REG_PAMACID3OOOOffset 0x00163 */

#define BIT_SHIFT_USMACID3 0define BIT_MASK_BIMACID3 0fffffffL
ffffLdefine BIT_GEMACID3) & (x) & &IT_MASK_CPMACID3<< BIT_SHIFT_WLMACID3<define BIT_GEGEGEMACID3) & (x) &  BIT_SHIFT_USMACID3& &IT_MASK_CPMACID3<d* 2 REG_PABSSID3OOOOffset 0x00163 */

#define BIT_CPIFT_USBSSID3 0define BIT_MASK_BIBSSID3 0fffffffL
ffffLdefine BIT_GEBSSID3) & (x) & &IT_MASK_CPBSSID3<< BIT_SHIFT_WLBSSID3<define BIT_GEGEGEBSSID3) & (x) &  BIT_SHIFT_USBSSID3& &IT_MASK_CPBSSID3<d* 2 REG_PAMACID4OOOOffset 0x00164 */

#define BIT_SHIFT_USMACID4 0define BIT_MASK_BIMACID4 0fffffffL
ffffLdefine BIT_GEMACID4) & (x) & &IT_MASK_CPMACID4<< BIT_SHIFT_WLMACID4<define BIT_GEGEGEMACID4) & (x) &  BIT_SHIFT_USMACID4& &IT_MASK_CPMACID4<d* 2 REG_PABSSID4OOOOffset 0x00164 */

#define BIT_CPIFT_USBSSID4 0define BIT_MASK_BIBSSID4 0fffffffL
ffffLdefine BIT_GEBSSID4) & (x) & &IT_MASK_CPBSSID4<< BIT_SHIFT_WLBSSID4<define BIT_GEGEGEBSSID4) & (x) &  BIT_SHIFT_USBSSID4& &IT_MASK_CPBSSID4<d* 2 REG_PAPWRT_SHIETTINGOOOffset 0x00166 */

#define BIT_SHCLI3APWRT_SHOW_EN B_C17<#efine BIT_SHCLI3APWR_ST B_C16<#efine BIT_SHCLI2APWRT_SHOW_EN B_C15<#efine BIT_SHCLI2APWR_ST B_C14<#efine BIT_SHCLI1APWRT_SHOW_EN B_C13<#efine BIT_SHCLI1APWR_ST B_C12<#efine BIT_SHCLI0APWRT_SHOW_EN B_C11<define BIT_GECLI0APWR_ST B_C10&d* 2 REG_PAWMACPMU_BF_OPTIONOOOffset 0x00167 */

#define BIT_STWMACPRESP_NONSTA1_DIS B_C17<#* 2 REG_PAWMACPMU_BF_OPTIONOOOffset 0x00167 */

#define BIT_STT_STWMACPTXMU_ACKPOLICY_EN B_C16<#* 2 REG_PAWMACPMU_BF_OPTIONOOOffset 0x00167 */

#define BIT_STIFT_USWMACPTXMU_ACKPOLICY 4define BIT_MASK_CPWMACPTXMU_ACKPOLICY 0x3define BIT_GEWMACPTXMU_ACKPOLICY) &                                             	(((x) & &IT_MASK_CPWMACPTXMU_ACKPOLICY<< BIT_SHIFT_WLWMACPTXMU_ACKPOLICY<define BIT_GEGEGEWMACPTXMU_ACKPOLICY) &                                         	(((x) &  BIT_SHIFT_USWMACPTXMU_ACKPOLICY<<&IT_MASK_CPWMACPTXMU_ACKPOLICY<#define BIT_STIFT_USWMACPMU_BFEE_PORT_SEL 1define BIT_MASK_CPWMACPMU_BFEE_PORT_SEL 0x7define BIT_SHWMACPMU_BFEE_PORT_SEL) &                                           	(((x) & &IT_MASK_CPWMACPMU_BFEE_PORT_SEL&                                	((< BIT_SHIFT_WLWMACPMU_BFEE_PORT_SEL&define BIT_GEGEGEWMACPMU_BFEE_PORT_SEL) &                                       	(((x) &  BIT_SHIFT_USWMACPMU_BFEE_PORT_SEL& &                            	((IT_MASK_CPWMACPMU_BFEE_PORT_SEL&ddefine BIT_SHWMACPMU_BFEE_DIS B_C10&d* 2 REG_PAWMACPPAUSE_BB_CLR_THOOffset 0x00167D*/

#define BIT_STIFT_USWMACPPAUSE_BB_CLR_TH 0define BIT_MASK_BIWMACPPAUSE_BB_CLR_TH 0xffdefine BIT_SHWMACPPAUSE_BB_CLR_TH) &                                            	(((x) & &IT_MASK_CPWMACPPAUSE_BB_CLR_TH&                                 	((< BIT_SHIFT_WLWMACPPAUSE_BB_CLR_TH&define BIT_GEGEGEWMACPPAUSE_BB_CLR_TH) &                                        	(((x) &  BIT_SHIFT_USWMACPPAUSE_BB_CLR_TH& &                             	((<T_MASK_CPWMACPPAUSE_BB_CLR_TH&d* 2 REG_PAWMACPMU_ARB	OOOffset 0x00167E*/

#define BIT_STWMACPARB_HW_ADAPT_EN B_C17<#efine BIT_SHWMACPARB_SW_EN B_C16<#*efine BIT_STIFT_USWMACPARB_SW_STATE 0define BIT_MASK_BIWMACPARB_SW_STATE 0x3fdefine BIT_SHWMACPARB_SW_STATE) &                                               	(((x) & &IT_MASK_CPWMACPARB_SW_STATE<< BIT_SHIFT_WLWMACPARB_SW_STATE<define BIT_GEGEGEWMACPARB_SW_STATE) &                                           	(((x) &  BIT_SHIFT_USWMACPARB_SW_STATE<<&IT_MASK_CPWMACPARB_SW_STATE<d* 2 REG_PAWMACPMU_OPTIONOOOffset 0x00167F*/

#define BIT_STIFT_USWMACPMU_DBGSEL 5define BIT_MASK_CPWMACPMU_DBGSEL 0x3define BIT_GEWMACPMU_DBGSEL) &                                                  	(((x) & &IT_MASK_CPWMACPMU_DBGSEL<< BIT_SHIFT_WLWMACPMU_DBGSEL<define BIT_GEGEGEWMACPMU_DBGSEL) &                                              	(((x) &  BIT_SHIFT_USWMACPMU_DBGSEL<<&IT_MASK_CPWMACPMU_DBGSEL<#define BIT_STIFT_USWMACPMU_CPRD_TIMEOUT 0define BIT_MASK_BIWMACPMU_CPRD_TIMEOUT 001fdefine BIT_STWMACPMU_CPRD_TIMEOUT) &                                            	(((x) & &IT_MASK_CPWMACPMU_CPRD_TIMEOUT&                                 	((< BIT_SHIFT_WLWMACPMU_CPRD_TIMEOUT&define BIT_GEGEGEWMACPMU_CPRD_TIMEOUT) &                                        	(((x) &  BIT_SHIFT_USWMACPMU_CPRD_TIMEOUT& &                             	((<T_MASK_CPWMACPMU_CPRD_TIMEOUT&d* 2 REG_PAWMACPMU_BF_CTLOOOffset 0x00168 */

#define BIT_SHWMACPINVLD_BFPRT_CHK B_C115<#efine BIT_SHWMACPRETXBFRPTSEQ_UPD B_C114<#define BIT_STIFT_USWMACPMU_BFRPTSEG_SEL 12define BIT_MASK_CPWMACPMU_BFRPTSEG_SEL 0x3define BIT_GEWMACPMU_BFRPTSEG_SEL) &                                            	(((x) & &IT_MASK_CPWMACPMU_BFRPTSEG_SEL&                                 	((< BIT_SHIFT_WLWMACPMU_BFRPTSEG_SEL&define BIT_GEGEGEWMACPMU_BFRPTSEG_SEL) &                                        	(((x) &  BIT_SHIFT_USWMACPMU_BFRPTSEG_SEL& &                             	((<T_MASK_CPWMACPMU_BFRPTSEG_SEL&ddefine BIT_STIFT_USWMACPMU_BF_MYAID 0define BIT_MASK_BIWMACPMU_BF_MYAID 0xfffdefine BIT_STWMACPMU_BF_MYAID) &                                                	(((x) & &IT_MASK_CPWMACPMU_BF_MYAID<< BIT_SHIFT_WLWMACPMU_BF_MYAID<define BIT_GEGEGEWMACPMU_BF_MYAID) &                                            	(((x) &  BIT_SHIFT_USWMACPMU_BF_MYAID<<&IT_MASK_CPWMACPMU_BF_MYAID<#*efine BIT_SHIFT_SDBFRPTPPARA 0define BIT_MASK_BIBFRPTPPARA 0xfffdefine BIT_STBFRPTPPARA) & (x) & &IT_MASK_CPBFRPTPPARA<< BIT_SHIFT_WLBFRPTPPARA<define BIT_GEGEGEBFRPTPPARA) &                                                  	(((x) &  BIT_SHIFT_USBFRPTPPARA<<&IT_MASK_CPBFRPTPPARA<d* 2 REG_PAWMACPMU_BFRPTPPARAOOOffset 0x001682*/

#define BIT_CPIFT_USB_STBFRPTPPARA_USERID_SEL 12define BIT_MASK_CPB_STBFRPTPPARA_USERID_SEL 0x7define BIT_SHB_STBFRPTPPARA_USERID_SEL) &                                       	(((x) & &IT_MASK_CPB_STBFRPTPPARA_USERID_SEL&                            	((< BIT_SHIFT_WLB_STBFRPTPPARA_USERID_SEL&define BIT_GEGEGEB_STBFRPTPPARA_USERID_SEL) &                                   	(((x) &  BIT_SHIFT_USB_STBFRPTPPARA_USERID_SEL& &                        	((<T_MASK_CPB_STBFRPTPPARA_USERID_SEL&d* 2 REG_PAWMACPASSOCIATEDPMU_BFMEE2OOffset 0x00168 */

#define BIT_SHITATUS_BFEE2 B_C110<#efine BIT_SHWMACPMU_BFEE2_EN B_C19<#define BIT_STIFT_USWMACPMU_BFEE2_AID 0define BIT_MASK_BIWMACPMU_BFEE2_AID 001ffdefine BIT_STWMACPMU_BFEE2_AID) &                                               	(((x) & &IT_MASK_CPWMACPMU_BFEE2_AID<< BIT_SHIFT_WLWMACPMU_BFEE2_AID<define BIT_GEGEGEWMACPMU_BFEE2_AID) &                                           	(((x) &  BIT_SHIFT_USWMACPMU_BFEE2_AID<<&IT_MASK_CPWMACPMU_BFEE2_AID<d* 2 REG_PAWMACPASSOCIATEDPMU_BFMEE3OOffset 0x001686*/

#define BIT_CPITATUS_BFEE3 B_C110<#efine BIT_SHWMACPMU_BFEE3_EN B_C19<#define BIT_STIFT_USWMACPMU_BFEE3_AID 0define BIT_MASK_BIWMACPMU_BFEE3_AID 001ffdefine BIT_STWMACPMU_BFEE3_AID) &                                               	(((x) & &IT_MASK_CPWMACPMU_BFEE3_AID<< BIT_SHIFT_WLWMACPMU_BFEE3_AID<define BIT_GEGEGEWMACPMU_BFEE3_AID) &                                           	(((x) &  BIT_SHIFT_USWMACPMU_BFEE3_AID<<&IT_MASK_CPWMACPMU_BFEE3_AID<d* 2 REG_PAWMACPASSOCIATEDPMU_BFMEE4OOffset 0x00168 */

#define BIT_CPITATUS_BFEE4 B_C110<#efine BIT_SHWMACPMU_BFEE4_EN B_C19<#define BIT_STIFT_USWMACPMU_BFEE4_AID 0define BIT_MASK_BIWMACPMU_BFEE4_AID 001ffdefine BIT_STWMACPMU_BFEE4_AID) &                                               	(((x) & &IT_MASK_CPWMACPMU_BFEE4_AID<< BIT_SHIFT_WLWMACPMU_BFEE4_AID<define BIT_GEGEGEWMACPMU_BFEE4_AID) &                                           	(((x) &  BIT_SHIFT_USWMACPMU_BFEE4_AID<<&IT_MASK_CPWMACPMU_BFEE4_AID<d* 2 REG_PAWMACPASSOCIATEDPMU_BFMEE5OOffset 0x00168A*/

#define BIT_SHRAWMACPRX_SYNCFIFO_SYNC B_C155<#efine BIT_SHRAWMACPRXRST_DLY B_C154<#efine BIT_SHRAWMACPSRCH_TXRPTPREF_DROP B_C153<#efine BIT_SHRAWMACPSRCH_TXRPTPUA1 B_C152<#efine BIT_SHITATUS_BFEE5 B_C110<#* 2 REG_PAWMACPASSOCIATEDPMU_BFMEE5OOffset 0x00168A*/

#define BIT_SHWMACPMU_BFEE5_EN B_C19<#define BIT_STIFT_USWMACPMU_BFEE5_AID 0define BIT_MASK_BIWMACPMU_BFEE5_AID 001ffdefine BIT_STWMACPMU_BFEE5_AID) &                                               	(((x) & &IT_MASK_CPWMACPMU_BFEE5_AID<< BIT_SHIFT_WLWMACPMU_BFEE5_AID<define BIT_GEGEGEWMACPMU_BFEE5_AID) &                                           	(((x) &  BIT_SHIFT_USWMACPMU_BFEE5_AID<<&IT_MASK_CPWMACPMU_BFEE5_AID<d* 2 REG_PAWMACPASSOCIATEDPMU_BFMEE6OOffset 0x00168 */

#define BIT_STITATUS_BFEE6 B_C110<#efine BIT_SHWMACPMU_BFEE6_EN B_C19<#define BIT_STIFT_USWMACPMU_BFEE6_AID 0define BIT_MASK_BIWMACPMU_BFEE6_AID 001ffdefine BIT_STWMACPMU_BFEE6_AID) &                                               	(((x) & &IT_MASK_CPWMACPMU_BFEE6_AID<< BIT_SHIFT_WLWMACPMU_BFEE6_AID<define BIT_GEGEGEWMACPMU_BFEE6_AID) &                                           	(((x) &  BIT_SHIFT_USWMACPMU_BFEE6_AID<<&IT_MASK_CPWMACPMU_BFEE6_AID<d* 2 REG_PAWMACPASSOCIATEDPMU_BFMEE7OOffset 0x00168E*/

#define BIT_STT_CPITATUS_BFEE4 B_C110<#efine BIT_SHWMACPMU_BFEE7_EN B_C19<#define BIT_STIFT_USWMACPMU_BFEE7_AID 0define BIT_MASK_BIWMACPMU_BFEE7_AID 001ffdefine BIT_STWMACPMU_BFEE7_AID) &                                               	(((x) & &IT_MASK_CPWMACPMU_BFEE7_AID<< BIT_SHIFT_WLWMACPMU_BFEE7_AID<define BIT_GEGEGEWMACPMU_BFEE7_AID) &                                           	(((x) &  BIT_SHIFT_USWMACPMU_BFEE7_AID<<&IT_MASK_CPWMACPMU_BFEE7_AID<d* 2 REG_PAWMACPBBPITOPPRX_COUNTEROOffset 0x00169 */

#define BIT_SHRST_ALL_COUNTER B_C131<#*efine BIT_SHIFT_USABORT_RX_VBON_COUNTER 1#define BIT_MASK_BIABORT_RX_VBON_COUNTER 0xffdefine BIT_SHABORT_RX_VBON_COUNTER) &                                           	(((x) & &IT_MASK_CPABORT_RX_VBON_COUNTER&                                	((< BIT_SHIFT_WLABORT_RX_VBON_COUNTER&define BIT_GEGEGEABORT_RX_VBON_COUNTER) &                                       	(((x) &  BIT_SHIFT_USABORT_RX_VBON_COUNTER& &                            	((IT_MASK_CPABORT_RX_VBON_COUNTER&d*efine BIT_SHIFT_USABORT_RX_RDRDY_COUNTER 8define BIT_MASK_BIABORT_RX_RDRDY_COUNTER 0xffdefine BIT_SHABORT_RX_RDRDY_COUNTER) &                                          	(((x) & &IT_MASK_CPABORT_RX_RDRDY_COUNTER&                               	((< BIT_SHIFT_WLABORT_RX_RDRDY_COUNTER&define BIT_GEGEGEABORT_RX_RDRDY_COUNTER) &                                      	(((x) &  BIT_SHIFT_USABORT_RX_RDRDY_COUNTER& &                           	((IT_MASK_CPABORT_RX_RDRDY_COUNTER&d*efine BIT_SHIFT_USVBON_EARLY_FALLING_COUNTER 0define BIT_MASK_BIVBON_EARLY_FALLING_COUNTER 0xffdefine BIT_SHVBON_EARLY_FALLING_COUNTER) &                                      	(((x) & &IT_MASK_BIVBON_EARLY_FALLING_COUNTER&                           	((< BIT_SHIFT_WLVBON_EARLY_FALLING_COUNTER&define BIT_GEGEGEVBON_EARLY_FALLING_COUNTER) &                                  	(((x) &  BIT_SHIFT_USVBON_EARLY_FALLING_COUNTER& &                       	((IT_MASK_CPVBON_EARLY_FALLING_COUNTER&d* 2 REG_PAWMACPPLCP_MONITOROOOffset 0x00169 */

#define BIT_SHWMACPPLCP_TRX_SEL T_C131<#*efine BIT_SHIFT_USWMACPPLCP_RDSIG_SEL 28define BIT_MASK_BIWMACPPLCP_RDSIG_SEL 0x7define BIT_SHWMACPPLCP_RDSIG_SEL) &                                             	(((x) & &IT_MASK_CPWMACPPLCP_RDSIG_SEL<< BIT_SHIFT_WLWMACPPLCP_RDSIG_SEL<define BIT_GEGEGEWMACPPLCP_RDSIG_SEL) &                                         	(((x) &  BIT_SHIFT_USWMACPPLCP_RDSIG_SEL<<&IT_MASK_CPWMACPPLCP_RDSIG_SEL<#*efine BIT_SHIFT_USWMACPRATE_IDX 24define BIT_MASK_CPWMACPRATE_IDX 0ffdefine BIT_SHWMACPRATE_IDX) &                                                   	(((x) & &IT_MASK_CPWMACPRATE_IDX<< BIT_SHIFT_WLWMACPRATE_IDX<define BIT_GEGEGEWMACPRATE_IDX) &                                               	(((x) &  BIT_SHIFT_USWMACPRATE_IDX<<&IT_MASK_CPWMACPRATE_IDX<#*efine BIT_SHIFT_USWMACPPLCP_RDSIG 0define BIT_MASK_BIWMACPPLCP_RDSIG 0fffffffdefine BIT_SHWMACPPLCP_RDSIG) &                                                 	(((x) & &IT_MASK_CPWMACPPLCP_RDSIG<< BIT_SHIFT_WLWMACPPLCP_RDSIG<define BIT_GEGEGEWMACPPLCP_RDSIG) &                                             	(((x) &  BIT_SHIFT_USWMACPPLCP_RDSIG& &IT_MASK_CPWMACPPLCP_RDSIG<d* 2 REG_PAWMACPPLCP_MONITORPMUTXOOffset 0x00169 */

#define BIT_CPWMACPMUTX_IDX T_C124<d* 2 REG_PATRANSM_SHADDRSS_0OOOffset 0x0016A */

#define BIT_SHIFT_USTA0 0define BIT_MASK_BITA0 0fffffffL
ffffLdefine BIT_GETA0) & (x) & &IT_MASK_CPTA */ BIT_SHIFT_WLTA *define BIT_GEGEGETA0) & (x) &  BIT_SHIFT_USTA */&IT_MASK_CPTA *d* 2 REG_PATRANSM_SHADDRSS_1OOOffset 0x0016A */

#define BIT_CPIFT_USTA1 0define BIT_MASK_BITA1 0fffffffL
ffffLdefine BIT_GETA1) & (x) & &IT_MASK_CPTA1<< BIT_SHIFT_WLTA1<define BIT_GEGEGETA1) & (x) &  BIT_SHIFT_USTA1& &IT_MASK_CPTA1<d* 2 REG_PATRANSM_SHADDRSS_2OOOffset 0x0016B */

#define BIT_SHIFT_USTA2 0define BIT_MASK_BITA2 0fffffffL
ffffLdefine BIT_GETA2) & (x) & &IT_MASK_CPTA2<< BIT_SHIFT_WLTA2<define BIT_GEGEGETA2) & (x) &  BIT_SHIFT_USTA2& &IT_MASK_CPTA2<d* 2 REG_PATRANSM_SHADDRSS_3OOOffset 0x0016B */

#define BIT_CPIFT_USTA3 0define BIT_MASK_BITA3 0fffffffL
ffffLdefine BIT_GETA3) & (x) & &IT_MASK_CPTA3<< BIT_SHIFT_WLTA3<define BIT_GEGEGETA3) & (x) &  BIT_SHIFT_USTA3& &IT_MASK_CPTA3<d* 2 REG_PATRANSM_SHADDRSS_4OOOffset 0x0016C */

#define BIT_SHIFT_USTA4 0define BIT_MASK_BITA4 0fffffffL
ffffLdefine BIT_GETA4) & (x) & &IT_MASK_CPTA4<< BIT_SHIFT_WLTA4<define BIT_GEGEGETA4) & (x) &  BIT_SHIFT_USTA4& &IT_MASK_CPTA4<d* 2 REG_PAWL2LTECOEX_INDIRECSHACCESS_CTRL_V1 ffset 0x00170 */

#define BIT_SHLTECOEX_ACCESS_START_V1 T_C131<#efine BIT_SHLTECOEX_WRITE_MODE_V1 T_C130<#efine BIT_SHLTECOEX_READY_T_SHV1 T_C129<#define BIT_STIFT_USWRITE_BYTE_ENHV1 1#define BIT_MASK_BIWRITE_BYTE_ENHV1 0ffdefine BIT_SHWRITE_BYTE_ENHV1) &                                                	(((x) & &IT_MASK_CPWRITE_BYTE_ENHV1<< BIT_SHIFT_WLWRITE_BYTE_ENHV1<define BIT_GEGEGEWRITE_BYTE_ENHV1) &                                            	(((x) &  BIT_SHIFT_USWRITE_BYTE_ENHV1<<&IT_MASK_CPWRITE_BYTE_ENHV1<#define BIT_STIFT_USLTECOEX_REG_ADDRAV1 0define BIT_MASK_BILTECOEX_REG_ADDRAV1 0fffffdefine BIT_STLTECOEX_REG_ADDRAV1) &                                             	(((x) & &IT_MASK_CPLTECOEX_REG_ADDRAV1<< BIT_SHIFT_WLLTECOEX_REG_ADDRAV1<define BIT_GEGEGELTECOEX_REG_ADDRAV1) &                                         	(((x) &  BIT_SHIFT_USLTECOEX_REG_ADDRAV1<<&IT_MASK_CPLTECOEX_REG_ADDRAV1<d* 2 REG_PAWL2LTECOEX_INDIRECSHACCESS_WRITE_DATA_V1 ffset 0x00170 */

#define BIT_SHIFT_SDLTECOEX_W_DATA_V1 0define BIT_MASK_BILTECOEX_W_DATA_V1 0fffffffL
Ldefine BIT_GELTECOEX_W_DATA_V1) &                                               	(((x) & &IT_MASK_CPLTECOEX_W_DATA_V1<< BIT_SHIFT_WLLTECOEX_W_DATA_V1<define BIT_GEGEGELTECOEX_W_DATA_V1) &                                           	(((x) &  BIT_SHIFT_USLTECOEX_W_DATA_V1<<&IT_MASK_CPLTECOEX_W_DATA_V1<d* 2 REG_PAWL2LTECOEX_INDIRECSHACCESS_READ_DATA_V1 ffset 0x00170 */

#define BIT_CPIFT_USLTECOEX_R_DATA_V1 0define BIT_MASK_BILTECOEX_R_DATA_V1 0fffffffL
Ldefine BIT_GELTECOEX_R_DATA_V1) &                                               	(((x) & &IT_MASK_CPLTECOEX_R_DATA_V1<< BIT_SHIFT_WLLTECOEX_R_DATA_V1<define BIT_GEGEGELTECOEX_R_DATA_V1) &                                           	(((x) &  BIT_SHIFT_USLTECOEX_R_DATA_V1<<&IT_MASK_CPLTECOEX_R_DATA_V1<d
#endif  2 __RTLAWLAN_T_SDEF_H__/
