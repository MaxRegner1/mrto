/* Copyright 2008-2013 Broadcom Corporation
 * Copyright (c) 2014 QLogic Corporation
 * All rights reserved
 *
 * Unless you and QLogic execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2, available
 * at http://www.gnu.org/licenses/gpl-2.0.html (the "GPL").
 *
 * Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Qlogic software provided under a
 * license other than the GPL, without Qlogic's express prior written
 * consent.
 *
 * Written by Yaniv Rosner
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/mutex.h>

#include "bnx2x.h"
#include "bnx2x_cmn.h"

typedef int (*read_sfp_module_eeprom_func_p)(struct bnx2x_phy *phy,
					     struct link_params *params,
					     u8 dev_addr, u16 addr, u8 byte_cnt,
					     u8 *o_buf, u8);
/********************************************************/
#define MDIO_ACCESS_TIMEOUT		1000
#define WC_LANE_MAX			4
#define I2C_SWITCH_WIDTH		2
#define I2C_BSC0			0
#define I2C_BSC1			1
#define I2C_WA_RETRY_CNT		3
#define I2C_WA_PWR_ITER			(I2C_WA_RETRY_CNT - 1)
#define MCPR_IMC_COMMAND_READ_OP	1
#define MCPR_IMC_COMMAND_WRITE_OP	2

/* LED Blink rate that will achieve ~15.9Hz */
#define LED_BLINK_RATE_VAL_E3		354
#define LED_BLINK_RATE_VAL_E1X_E2	480
/***********************************************************/
/*			Shortcut definitions		   */
/***********************************************************/

#define NIG_LATCH_BC_ENABLE_MI_INT 0

#define NIG_STATUS_EMAC0_MI_INT \
		NIG_STATUS_INTERRUPT_PORT0_REG_STATUS_EMAC0_MISC_MI_INT
#define NIG_STATUS_XGXS0_LINK10G \
		NIG_STATUS_INTERRUPT_PORT0_REG_STATUS_XGXS0_LINK10G
#define NIG_STATUS_XGXS0_LINK_STATUS \
		NIG_STATUS_INTERRUPT_PORT0_REG_STATUS_XGXS0_LINK_STATUS
#define NIG_STATUS_XGXS0_LINK_STATUS_SIZE \
		NIG_STATUS_INTERRUPT_PORT0_REG_STATUS_XGXS0_LINK_STATUS_SIZE
#define NIG_STATUS_SERDES0_LINK_STATUS \
		NIG_STATUS_INTERRUPT_PORT0_REG_STATUS_SERDES0_LINK_STATUS
#define NIG_MASK_MI_INT \
		NIG_MASK_INTERRUPT_PORT0_REG_MASK_EMAC0_MISC_MI_INT
#define NIG_MASK_XGXS0_LINK10G \
		NIG_MASK_INTERRUPT_PORT0_REG_MASK_XGXS0_LINK10G
#define NIG_MASK_XGXS0_LINK_STATUS \
		NIG_MASK_INTERRUPT_PORT0_REG_MASK_XGXS0_LINK_STATUS
#define NIG_MASK_SERDES0_LINK_STATUS \
		NIG_MASK_INTERRUPT_PORT0_REG_MASK_SERDES0_LINK_STATUS

#define MDIO_AN_CL73_OR_37_COMPLETE \
		(MDIO_GP_STATUS_TOP_AN_STATUS1_CL73_AUTONEG_COMPLETE | \
		 MDIO_GP_STATUS_TOP_AN_STATUS1_CL37_AUTONEG_COMPLETE)

#define XGXS_RESET_BITS \
	(MISC_REGISTERS_RESET_REG_3_MISC_NIG_MUX_XGXS0_RSTB_HW |   \
	 MISC_REGISTERS_RESET_REG_3_MISC_NIG_MUX_XGXS0_IDDQ |      \
	 MISC_REGISTERS_RESET_REG_3_MISC_NIG_MUX_XGXS0_PWRDWN |    \
	 MISC_REGISTERS_RESET_REG_3_MISC_NIG_MUX_XGXS0_PWRDWN_SD | \
	 MISC_REGISTERS_RESET_REG_3_MISC_NIG_MUX_XGXS0_TXD_FIFO_RSTB)

#define SERDES_RESET_BITS \
	(MISC_REGISTERS_RESET_REG_3_MISC_NIG_MUX_SERDES0_RSTB_HW | \
	 MISC_REGISTERS_RESET_REG_3_MISC_NIG_MUX_SERDES0_IDDQ |    \
	 MISC_REGISTERS_RESET_REG_3_MISC_NIG_MUX_SERDES0_PWRDWN |  \
	 MISC_REGISTERS_RESET_REG_3_MISC_NIG_MUX_SERDES0_PWRDWN_SD)

#define AUTONEG_CL37		SHARED_HW_CFG_AN_ENABLE_CL37
#define AUTONEG_CL73		SHARED_HW_CFG_AN_ENABLE_CL73
#define AUTONEG_BAM		SHARED_HW_CFG_AN_ENABLE_BAM
#define AUTONEG_PARALLEL \
				SHARED_HW_CFG_AN_ENABLE_PARALLEL_DETECTION
#define AUTONEG_SGMII_FIBER_AUTODET \
				SHARED_HW_CFG_AN_EN_SGMII_FIBER_AUTO_DETECT
#define AUTONEG_REMOTE_PHY	SHARED_HW_CFG_AN_ENABLE_REMOTE_PHY

#define GP_STATUS_PAUSE_RSOLUTION_TXSIDE \
			MDIO_GP_STATUS_TOP_AN_STATUS1_PAUSE_RSOLUTION_TXSIDE
#define GP_STATUS_PAUSE_RSOLUTION_RXSIDE \
			MDIO_GP_STATUS_TOP_AN_STATUS1_PAUSE_RSOLUTION_RXSIDE
#define GP_STATUS_SPEED_MASK \
			MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_MASK
#define GP_STATUS_10M	MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_10M
#define GP_STATUS_100M	MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_100M
#define GP_STATUS_1G	MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_1G
#define GP_STATUS_2_5G	MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_2_5G
#define GP_STATUS_5G	MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_5G
#define GP_STATUS_6G	MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_6G
#define GP_STATUS_10G_HIG \
			MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_10G_HIG
#define GP_STATUS_10G_CX4 \
			MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_10G_CX4
#define GP_STATUS_1G_KX MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_1G_KX
#define GP_STATUS_10G_KX4 \
			MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_10G_KX4
#define	GP_STATUS_10G_KR MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_10G_KR
#define	GP_STATUS_10G_XFI   MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_10G_XFI
#define	GP_STATUS_20G_DXGXS MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_20G_DXGXS
#define	GP_STATUS_10G_SFI   MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_10G_SFI
#define	GP_STATUS_20G_KR2 MDIO_GP_STATUS_TOP_AN_STATUS1_ACTUAL_SPEED_20G_KR2
#define LINK_10THD		LINK_STATUS_SPEED_AND_DUPLEX_10THD
#define LINK_10TFD		LINK_STATUS_SPEED_AND_DUPLEX_10TFD
#define LINK_100TXHD		LINK_STATUS_SPEED_AND_DUPLEX_100TXHD
#define LINK_100T4		LINK_STATUS_SPEED_AND_DUPLEX_100T4
#define LINK_100TXFD		LINK_STATUS_SPEED_AND_DUPLEX_100TXFD
#define LINK_1000THD		LINK_STATUS_SPEED_AND_DUPLEX_1000THD
#define LINK_1000TFD		LINK_STATUS_SPEED_AND_DUPLEX_1000TFD
#define LINK_1000XFD		LINK_STATUS_SPEED_AND_DUPLEX_1000XFD
#define LINK_2500THD		LINK_STATUS_SPEED_AND_DUPLEX_2500THD
#define LINK_2500TFD		LINK_STATUS_SPEED_AND_DUPLEX_2500TFD
#define LINK_2500XFD		LINK_STATUS_SPEED_AND_DUPLEX_2500XFD
#define LINK_10GTFD		LINK_STATUS_SPEED_AND_DUPLEX_10GTFD
#define LINK_10GXFD		LINK_STATUS_SPEED_AND_DUPLEX_10GXFD
#define LINK_20GTFD		LINK_STATUS_SPEED_AND_DUPLEX_20GTFD
#define LINK_20GXFD		LINK_STATUS_SPEED_AND_DUPLEX_20GXFD

#define LINK_UPDATE_MASK \
			(LINK_STATUS_SPEED_AND_DUPLEX_MASK | \
			 LINK_STATUS_LINK_UP | \
			 LINK_STATUS_PHYSICAL_LINK_FLAG | \
			 LINK_STATUS_AUTO_NEGOTIATE_COMPLETE | \
			 LINK_STATUS_RX_FLOW_CONTROL_FLAG_MASK | \
			 LINK_STATUS_TX_FLOW_CONTROL_FLAG_MASK | \
			 LINK_STATUS_PARALLEL_DETECTION_FLAG_MASK | \
			 LINK_STATUS_LINK_PARTNER_SYMMETRIC_PAUSE | \
			 LINK_STATUS_LINK_PARTNER_ASYMMETRIC_PAUSE)

#define SFP_EEPROM_CON_TYPE_ADDR		0x2
	#define SFP_EEPROM_CON_TYPE_VAL_UNKNOWN	0x0
	#define SFP_EEPROM_CON_TYPE_VAL_LC	0x7
	#define SFP_EEPROM_CON_TYPE_VAL_COPPER	0x21
	#define SFP_EEPROM_CON_TYPE_VAL_RJ45	0x22


#define SFP_EEPROM_10G_COMP_CODE_ADDR		0x3
	#define SFP_EEPROM_10G_COMP_CODE_SR_MASK	(1<<4)
	#define SFP_EEPROM_10G_COMP_CODE_LR_MASK	(1<<5)
	#define SFP_EEPROM_10G_COMP_CODE_LRM_MASK	(1<<6)

#define SFP_EEPROM_1G_COMP_CODE_ADDR		0x6
	#define SFP_EEPROM_1G_COMP_CODE_SX	(1<<0)
	#define SFP_EEPROM_1G_COMP_CODE_LX	(1<<1)
	#define SFP_EEPROM_1G_COMP_CODE_CX	(1<<2)
	#define SFP_EEPROM_1G_COMP_CODE_BASE_T	(1<<3)

#define SFP_EEPROM_FC_TX_TECH_ADDR		0x8
	#define SFP_EEPROM_FC_TX_TECH_BITMASK_COPPER_PASSIVE 0x4
	#define SFP_EEPROM_FC_TX_TECH_BITMASK_COPPER_ACTIVE  0x8

#define SFP_EEPROM_OPTIONS_ADDR			0x40
	#define SFP_EEPROM_OPTIONS_LINEAR_RX_OUT_MASK 0x1
#define SFP_EEPROM_OPTIONS_SIZE			2

#define EDC_MODE_LINEAR				0x0022
#define EDC_MODE_LIMITING				0x0044
#define EDC_MODE_PASSIVE_DAC			0x0055
#define EDC_MODE_ACTIVE_DAC			0x0066

/* ETS defines*/
#define DCBX_INVALID_COS					(0xFF)

#define ETS_BW_LIMIT_CREDIT_UPPER_BOUND		(0x5000)
#define ETS_BW_LIMIT_CREDIT_WEIGHT		(0x5000)
#define ETS_E3B0_NIG_MIN_W_VAL_UP_TO_10GBPS		(1360)
#define ETS_E3B0_NIG_MIN_W_VAL_20GBPS			(2720)
#define ETS_E3B0_PBF_MIN_W_VAL				(10000)

#define MAX_PACKET_SIZE					(9700)
#define MAX_KR_LINK_RETRY				4
#define DEFAULT_TX_DRV_BRDCT		2
#define DEFAULT_TX_DRV_IFIR		0
#define DEFAULT_TX_DRV_POST2		3
#define DEFAULT_TX_DRV_IPRE_DRIVER	6

/**********************************************************/
/*                     INTERFACE                          */
/**********************************************************/

#define CL22_WR_OVER_CL45(_bp, _phy, _bank, _addr, _val) \
	bnx2x_cl45_write(_bp, _phy, \
		(_phy)->def_md_devad, \
		(_bank + (_addr & 0xf)), \
		_val)

#define CL22_RD_OVER_CL45(_bp, _phy, _bank, _addr, _val) \
	bnx2x_cl45_read(_bp, _phy, \
		(_phy)->def_md_devad, \
		(_bank + (_addr & 0xf)), \
		_val)

static int bnx2x_check_half_open_conn(struct link_params *params,
				      struct link_vars *vars, u8 notify);
static int bnx2x_sfp_module_detection(struct bnx2x_phy *phy,
				      struct link_params *params);

static u32 bnx2x_bits_en(struct bnx2x *bp, u32 reg, u32 bits)
{
	u32 val = REG_RD(bp, reg);

	val |= bits;
	REG_WR(bp, reg, val);
	return val;
}

static u32 bnx2x_bits_dis(struct bnx2x *bp, u32 reg, u32 bits)
{
	u32 val = REG_RD(bp, reg);

	val &= ~bits;
	REG_WR(bp, reg, val);
	return val;
}

/*
 * bnx2x_check_lfa - This function checks if link reinitialization is required,
 *                   or link flap can be avoided.
 *
 * @params:	link parameters
 * Returns 0 if Link Flap Avoidance conditions are met otherwise, the failed
 *         condition code.
 */
static int bnx2x_check_lfa(struct link_params *params)
{
	u32 link_status, cfg_idx, lfa_mask, cfg_size;
	u32 cur_speed_cap_mask, cur_req_fc_auto_adv, additional_config;
	u32 saved_val, req_val, eee_status;
	struct bnx2x *bp = params->bp;

	additional_config =
		REG_RD(bp, params->lfa_base +
			   offsetof(struct shmem_lfa, additional_config));

	/* NOTE: must be first condition checked -
	* to verify DCC bit is cleared in any case!
	*/
	if (additional_config & NO_LFA_DUE_TO_DCC_MASK) {
		DP(NETIF_MSG_LINK, "No LFA due to DCC flap after clp exit\n");
		REG_WR(bp, params->lfa_base +
			   offsetof(struct shmem_lfa, additional_config),
		       additional_config & ~NO_LFA_DUE_TO_DCC_MASK);
		return LFA_DCC_LFA_DISABLED;
	}

	/* Verify that link is up */
	link_status = REG_RD(bp, params->shmem_base +
			     offsetof(struct shmem_region,
				      port_mb[params->port].link_status));
	if (!(link_status & LINK_STATUS_LINK_UP))
		return LFA_LINK_DOWN;

	/* if loaded after BOOT from SAN, don't flap the link in any case and
	 * rely on link set by preboot driver
	 */
	if (params->feature_config_flags & FEATURE_CONFIG_BOOT_FROM_SAN)
		return 0;

	/* Verify that loopback mode is not set */
	if (params->loopback_mode)
		return LFA_LOOPBACK_ENABLED;

	/* Verify that MFW supports LFA */
	if (!params->lfa_base)
		return LFA_MFW_IS_TOO_OLD;

	if (params->num_phys == 3) {
		cfg_size = 2;
		lfa_mask = 0xffffffff;
	} else {
		cfg_size = 1;
		lfa_mask = 0xffff;
	}

	/* Compare Duplex */
	saved_val = REG_RD(bp, params->lfa_base +
			   offsetof(struct shmem_lfa, req_duplex));
	req_val = params->req_duplex[0] | (params->req_duplex[1] << 16);
	if ((saved_val & lfa_mask) != (req_val & lfa_mask)) {
		DP(NETIF_MSG_LINK, "Duplex mismatch %x vs. %x\n",
			       (saved_val & lfa_mask), (req_val & lfa_mask));
		return LFA_DUPLEX_MISMATCH;
	}
	/* Compare Flow Control */
	saved_val = REG_RD(bp, params->lfa_base +
			   offsetof(struct shmem_lfa, req_flow_ctrl));
	req_val = params->req_flow_ctrl[0] | (params->req_flow_ctrl[1] << 16);
	if ((saved_val & lfa_mask) != (req_val & lfa_mask)) {
		DP(NETIF_MSG_LINK, "Flow control mismatch %x vs. %x\n",
			       (saved_val & lfa_mask), (req_val & lfa_mask));
		return LFA_FLOW_CTRL_MISMATCH;
	}
	/* Compare Link Speed */
	saved_val = REG_RD(bp, params->lfa_base +
			   offsetof(struct shmem_lfa, req_line_speed));
	req_val = params->req_line_speed[0] | (params->req_line_speed[1] << 16);
	if ((saved_val & lfa_mask) != (req_val & lfa_mask)) {
		DP(NETIF_MSG_LINK, "Link speed mismatch %x vs. %x\n",
			       (saved_val & lfa_mask), (req_val & lfa_mask));
		return LFA_LINK_SPEED_MISMATCH;
	}

	for (cfg_idx = 0; cfg_idx < cfg_size; cfg_idx++) {
		cur_speed_cap_mask = REG_RD(bp, params->lfa_base +
					    offsetof(struct shmem_lfa,
						     speed_cap_mask[cfg_idx]));

		if (cur_speed_cap_mask != params->speed_cap_mask[cfg_idx]) {
			DP(NETIF_MSG_LINK, "Speed Cap mismatch %x vs. %x\n",
				       cur_speed_cap_mask,
				       params->speed_cap_mask[cfg_idx]);
			return LFA_SPEED_CAP_MISMATCH;
		}
	}

	cur_req_fc_auto_adv =
		REG_RD(bp, params->lfa_base +
		       offsetof(struct shmem_lfa, additional_config)) &
		REQ_FC_AUTO_ADV_MASK;

	if ((u16)cur_req_fc_auto_adv != params->req_fc_auto_adv) {
		DP(NETIF_MSG_LINK, "Flow Ctrl AN mismatch %x vs. %x\n",
			       cur_req_fc_auto_adv, params->req_fc_auto_adv);
		return LFA_FLOW_CTRL_MISMATCH;
	}

	eee_status = REG_RD(bp, params->shmem2_base +
			    offsetof(struct shmem2_region,
				     eee_status[params->port]));

	if (((eee_status & SHMEM_EEE_LPI_REQUESTED_BIT) ^
	     (params->eee_mode & EEE_MODE_ENABLE_LPI)) ||
	    ((eee_status & SHMEM_EEE_REQUESTED_BIT) ^
	     (params->eee_mode & EEE_MODE_ADV_LPI))) {
		DP(NETIF_MSG_LINK, "EEE mismatch %x vs. %x\n", params->eee_mode,
			       eee_status);
		return LFA_EEE_MISMATCH;
	}

	/* LFA conditions are met */
	return 0;
}
/******************************************************************/
/*			EPIO/GPIO section			  */
/******************************************************************/
static void bnx2x_get_epio(struct bnx2x *bp, u32 epio_pin, u32 *en)
{
	u32 epio_mask, gp_oenable;
	*en = 0;
	/* Sanity check */
	if (epio_pin > 31) {
		DP(NETIF_MSG_LINK, "Invalid EPIO pin %d to get\n", epio_pin);
		return;
	}

	epio_mask = 1 << epio_pin;
	/* Set this EPIO to output */
	gp_oenable = REG_RD(bp, MCP_REG_MCPR_GP_OENABLE);
	REG_WR(bp, MCP_REG_MCPR_GP_OENABLE, gp_oenable & ~epio_mask);

	*en = (REG_RD(bp, MCP_REG_MCPR_GP_INPUTS) & epio_mask) >> epio_pin;
}
static void bnx2x_set_epio(struct bnx2x *bp, u32 epio_pin, u32 en)
{
	u32 epio_mask, gp_output, gp_oenable;

	/* Sanity check */
	if (epio_pin > 31) {
		DP(NETIF_MSG_LINK, "Invalid EPIO pin %d to set\n", epio_pin);
		return;
	}
	DP(NETIF_MSG_LINK, "Setting EPIO pin %d to %d\n", epio_pin, en);
	epio_mask = 1 << epio_pin;
	/* Set this EPIO to output */
	gp_output = REG_RD(bp, MCP_REG_MCPR_GP_OUTPUTS);
	if (en)
		gp_output |= epio_mask;
	else
		gp_output &= ~epio_mask;

	REG_WR(bp, MCP_REG_MCPR_GP_OUTPUTS, gp_output);

	/* Set the value for this EPIO */
	gp_oenable = REG_RD(bp, MCP_REG_MCPR_GP_OENABLE);
	REG_WR(bp, MCP_REG_MCPR_GP_OENABLE, gp_oenable | epio_mask);
}

static void bnx2x_set_cfg_pin(struct bnx2x *bp, u32 pin_cfg, u32 val)
{
	if (pin_cfg == PIN_CFG_NA)
		return;
	if (pin_cfg >= PIN_CFG_EPIO0) {
		bnx2x_set_epio(bp, pin_cfg - PIN_CFG_EPIO0, val);
	} else {
		u8 gpio_num = (pin_cfg - PIN_CFG_GPIO0_P0) & 0x3;
		u8 gpio_port = (pin_cfg - PIN_CFG_GPIO0_P0) >> 2;
		bnx2x_set_gpio(bp, gpio_num, (u8)val, gpio_port);
	}
}

static u32 bnx2x_get_cfg_pin(struct bnx2x *bp, u32 pin_cfg, u32 *val)
{
	if (pin_cfg == PIN_CFG_NA)
		return -EINVAL;
	if (pin_cfg >= PIN_CFG_EPIO0) {
		bnx2x_get_epio(bp, pin_cfg - PIN_CFG_EPIO0, val);
	} else {
		u8 gpio_num = (pin_cfg - PIN_CFG_GPIO0_P0) & 0x3;
		u8 gpio_port = (pin_cfg - PIN_CFG_GPIO0_P0) >> 2;
		*val = bnx2x_get_gpio(bp, gpio_num, gpio_port);
	}
	return 0;

}
/******************************************************************/
/*				ETS section			  */
/******************************************************************/
static void bnx2x_ets_e2e3a0_disabled(struct link_params *params)
{
	/* ETS disabled configuration*/
	struct bnx2x *bp = params->bp;

	DP(NETIF_MSG_LINK, "ETS E2E3 disabled configuration\n");

	/* mapping between entry  priority to client number (0,1,2 -debug and
	 * management clients, 3 - COS0 client, 4 - COS client)(HIGHEST)
	 * 3bits client num.
	 *   PRI4    |    PRI3    |    PRI2    |    PRI1    |    PRI0
	 * cos1-100     cos0-011     dbg1-010     dbg0-001     MCP-000
	 */

	REG_WR(bp, NIG_REG_P0_TX_ARB_PRIORITY_CLIENT, 0x4688);
	/* Bitmap of 5bits length. Each bit specifies whether the entry behaves
	 * as strict.  Bits 0,1,2 - debug and management entries, 3 -
	 * COS0 entry, 4 - COS1 entry.
	 * COS1 | COS0 | DEBUG1 | DEBUG0 | MGMT
	 * bit4   bit3	  bit2   bit1	  bit0
	 * MCP and debug are strict
	 */

	REG_WR(bp, NIG_REG_P0_TX_ARB_CLIENT_IS_STRICT, 0x7);
	/* defines which entries (clients) are subjected to WFQ arbitration */
	REG_WR(bp, NIG_REG_P0_TX_ARB_CLIENT_IS_SUBJECT2WFQ, 0);
	/* For strict priority entries defines the number of consecutive
	 * slots for the highest priority.
	 */
	REG_WR(bp, NIG_REG_P0_TX_ARB_NUM_STRICT_ARB_SLOTS, 0x100);
	/* mapping between the CREDIT_WEIGHT registers and actual client
	 * numbers
	 */
	REG_WR(bp, NIG_REG_P0_TX_ARB_CLIENT_CREDIT_MAP, 0);
	REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_0, 0);
	REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_1, 0);

	REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_0, 0);
	REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_1, 0);
	REG_WR(bp, PBF_REG_HIGH_PRIORITY_COS_NUM, 0);
	/* ETS mode disable */
	REG_WR(bp, PBF_REG_ETS_ENABLED, 0);
	/* If ETS mode is enabled (there is no strict priority) defines a WFQ
	 * weight for COS0/COS1.
	 */
	REG_WR(bp, PBF_REG_COS0_WEIGHT, 0x2710);
	REG_WR(bp, PBF_REG_COS1_WEIGHT, 0x2710);
	/* Upper bound that COS0_WEIGHT can reach in the WFQ arbiter */
	REG_WR(bp, PBF_REG_COS0_UPPER_BOUND, 0x989680);
	REG_WR(bp, PBF_REG_COS1_UPPER_BOUND, 0x989680);
	/* Defines the number of consecutive slots for the strict priority */
	REG_WR(bp, PBF_REG_NUM_STRICT_ARB_SLOTS, 0);
}
/******************************************************************************
* Description:
*	Getting min_w_val will be set according to line speed .
*.
******************************************************************************/
static u32 bnx2x_ets_get_min_w_val_nig(const struct link_vars *vars)
{
	u32 min_w_val = 0;
	/* Calculate min_w_val.*/
	if (vars->link_up) {
		if (vars->line_speed == SPEED_20000)
			min_w_val = ETS_E3B0_NIG_MIN_W_VAL_20GBPS;
		else
			min_w_val = ETS_E3B0_NIG_MIN_W_VAL_UP_TO_10GBPS;
	} else
		min_w_val = ETS_E3B0_NIG_MIN_W_VAL_20GBPS;
	/* If the link isn't up (static configuration for example ) The
	 * link will be according to 20GBPS.
	 */
	return min_w_val;
}
/******************************************************************************
* Description:
*	Getting credit upper bound form min_w_val.
*.
******************************************************************************/
static u32 bnx2x_ets_get_credit_upper_bound(const u32 min_w_val)
{
	const u32 credit_upper_bound = (u32)MAXVAL((150 * min_w_val),
						MAX_PACKET_SIZE);
	return credit_upper_bound;
}
/******************************************************************************
* Description:
*	Set credit upper bound for NIG.
*.
******************************************************************************/
static void bnx2x_ets_e3b0_set_credit_upper_bound_nig(
	const struct link_params *params,
	const u32 min_w_val)
{
	struct bnx2x *bp = params->bp;
	const u8 port = params->port;
	const u32 credit_upper_bound =
	    bnx2x_ets_get_credit_upper_bound(min_w_val);

	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_UPPER_BOUND_0 :
		NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_0, credit_upper_bound);
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_UPPER_BOUND_1 :
		   NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_1, credit_upper_bound);
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_UPPER_BOUND_2 :
		   NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_2, credit_upper_bound);
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_UPPER_BOUND_3 :
		   NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_3, credit_upper_bound);
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_UPPER_BOUND_4 :
		   NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_4, credit_upper_bound);
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_UPPER_BOUND_5 :
		   NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_5, credit_upper_bound);

	if (!port) {
		REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_6,
			credit_upper_bound);
		REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_7,
			credit_upper_bound);
		REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_8,
			credit_upper_bound);
	}
}
/******************************************************************************
* Description:
*	Will return the NIG ETS registers to init values.Except
*	credit_upper_bound.
*	That isn't used in this configuration (No WFQ is enabled) and will be
*	configured according to spec
*.
******************************************************************************/
static void bnx2x_ets_e3b0_nig_disabled(const struct link_params *params,
					const struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	const u8 port = params->port;
	const u32 min_w_val = bnx2x_ets_get_min_w_val_nig(vars);
	/* Mapping between entry  priority to client number (0,1,2 -debug and
	 * management clients, 3 - COS0 client, 4 - COS1, ... 8 -
	 * COS5)(HIGHEST) 4bits client num.TODO_ETS - Should be done by
	 * reset value or init tool
	 */
	if (port) {
		REG_WR(bp, NIG_REG_P1_TX_ARB_PRIORITY_CLIENT2_LSB, 0x543210);
		REG_WR(bp, NIG_REG_P1_TX_ARB_PRIORITY_CLIENT2_MSB, 0x0);
	} else {
		REG_WR(bp, NIG_REG_P0_TX_ARB_PRIORITY_CLIENT2_LSB, 0x76543210);
		REG_WR(bp, NIG_REG_P0_TX_ARB_PRIORITY_CLIENT2_MSB, 0x8);
	}
	/* For strict priority entries defines the number of consecutive
	 * slots for the highest priority.
	 */
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_NUM_STRICT_ARB_SLOTS :
		   NIG_REG_P0_TX_ARB_NUM_STRICT_ARB_SLOTS, 0x100);
	/* Mapping between the CREDIT_WEIGHT registers and actual client
	 * numbers
	 */
	if (port) {
		/*Port 1 has 6 COS*/
		REG_WR(bp, NIG_REG_P1_TX_ARB_CLIENT_CREDIT_MAP2_LSB, 0x210543);
		REG_WR(bp, NIG_REG_P1_TX_ARB_CLIENT_CREDIT_MAP2_MSB, 0x0);
	} else {
		/*Port 0 has 9 COS*/
		REG_WR(bp, NIG_REG_P0_TX_ARB_CLIENT_CREDIT_MAP2_LSB,
		       0x43210876);
		REG_WR(bp, NIG_REG_P0_TX_ARB_CLIENT_CREDIT_MAP2_MSB, 0x5);
	}

	/* Bitmap of 5bits length. Each bit specifies whether the entry behaves
	 * as strict.  Bits 0,1,2 - debug and management entries, 3 -
	 * COS0 entry, 4 - COS1 entry.
	 * COS1 | COS0 | DEBUG1 | DEBUG0 | MGMT
	 * bit4   bit3	  bit2   bit1	  bit0
	 * MCP and debug are strict
	 */
	if (port)
		REG_WR(bp, NIG_REG_P1_TX_ARB_CLIENT_IS_STRICT, 0x3f);
	else
		REG_WR(bp, NIG_REG_P0_TX_ARB_CLIENT_IS_STRICT, 0x1ff);
	/* defines which entries (clients) are subjected to WFQ arbitration */
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CLIENT_IS_SUBJECT2WFQ :
		   NIG_REG_P0_TX_ARB_CLIENT_IS_SUBJECT2WFQ, 0);

	/* Please notice the register address are note continuous and a
	 * for here is note appropriate.In 2 port mode port0 only COS0-5
	 * can be used. DEBUG1,DEBUG1,MGMT are never used for WFQ* In 4
	 * port mode port1 only COS0-2 can be used. DEBUG1,DEBUG1,MGMT
	 * are never used for WFQ
	 */
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_WEIGHT_0 :
		   NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_0, 0x0);
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_WEIGHT_1 :
		   NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_1, 0x0);
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_WEIGHT_2 :
		   NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_2, 0x0);
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_WEIGHT_3 :
		   NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_3, 0x0);
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_WEIGHT_4 :
		   NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_4, 0x0);
	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CREDIT_WEIGHT_5 :
		   NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_5, 0x0);
	if (!port) {
		REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_6, 0x0);
		REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_7, 0x0);
		REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_8, 0x0);
	}

	bnx2x_ets_e3b0_set_credit_upper_bound_nig(params, min_w_val);
}
/******************************************************************************
* Description:
*	Set credit upper bound for PBF.
*.
******************************************************************************/
static void bnx2x_ets_e3b0_set_credit_upper_bound_pbf(
	const struct link_params *params,
	const u32 min_w_val)
{
	struct bnx2x *bp = params->bp;
	const u32 credit_upper_bound =
	    bnx2x_ets_get_credit_upper_bound(min_w_val);
	const u8 port = params->port;
	u32 base_upper_bound = 0;
	u8 max_cos = 0;
	u8 i = 0;
	/* In 2 port mode port0 has COS0-5 that can be used for WFQ.In 4
	 * port mode port1 has COS0-2 that can be used for WFQ.
	 */
	if (!port) {
		base_upper_bound = PBF_REG_COS0_UPPER_BOUND_P0;
		max_cos = DCBX_E3B0_MAX_NUM_COS_PORT0;
	} else {
		base_upper_bound = PBF_REG_COS0_UPPER_BOUND_P1;
		max_cos = DCBX_E3B0_MAX_NUM_COS_PORT1;
	}

	for (i = 0; i < max_cos; i++)
		REG_WR(bp, base_upper_bound + (i << 2), credit_upper_bound);
}

/******************************************************************************
* Description:
*	Will return the PBF ETS registers to init values.Except
*	credit_upper_bound.
*	That isn't used in this configuration (No WFQ is enabled) and will be
*	configured according to spec
*.
******************************************************************************/
static void bnx2x_ets_e3b0_pbf_disabled(const struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	const u8 port = params->port;
	const u32 min_w_val_pbf = ETS_E3B0_PBF_MIN_W_VAL;
	u8 i = 0;
	u32 base_weight = 0;
	u8 max_cos = 0;

	/* Mapping between entry  priority to client number 0 - COS0
	 * client, 2 - COS1, ... 5 - COS5)(HIGHEST) 4bits client num.
	 * TODO_ETS - Should be done by reset value or init tool
	 */
	if (port)
		/*  0x688 (|011|0 10|00 1|000) */
		REG_WR(bp, PBF_REG_ETS_ARB_PRIORITY_CLIENT_P1 , 0x688);
	else
		/*  (10 1|100 |011|0 10|00 1|000) */
		REG_WR(bp, PBF_REG_ETS_ARB_PRIORITY_CLIENT_P0 , 0x2C688);

	/* TODO_ETS - Should be done by reset value or init tool */
	if (port)
		/* 0x688 (|011|0 10|00 1|000)*/
		REG_WR(bp, PBF_REG_ETS_ARB_CLIENT_CREDIT_MAP_P1, 0x688);
	else
	/* 0x2C688 (10 1|100 |011|0 10|00 1|000) */
	REG_WR(bp, PBF_REG_ETS_ARB_CLIENT_CREDIT_MAP_P0, 0x2C688);

	REG_WR(bp, (port) ? PBF_REG_ETS_ARB_NUM_STRICT_ARB_SLOTS_P1 :
		   PBF_REG_ETS_ARB_NUM_STRICT_ARB_SLOTS_P0 , 0x100);


	REG_WR(bp, (port) ? PBF_REG_ETS_ARB_CLIENT_IS_STRICT_P1 :
		   PBF_REG_ETS_ARB_CLIENT_IS_STRICT_P0 , 0);

	REG_WR(bp, (port) ? PBF_REG_ETS_ARB_CLIENT_IS_SUBJECT2WFQ_P1 :
		   PBF_REG_ETS_ARB_CLIENT_IS_SUBJECT2WFQ_P0 , 0);
	/* In 2 port mode port0 has COS0-5 that can be used for WFQ.
	 * In 4 port mode port1 has COS0-2 that can be used for WFQ.
	 */
	if (!port) {
		base_weight = PBF_REG_COS0_WEIGHT_P0;
		max_cos = DCBX_E3B0_MAX_NUM_COS_PORT0;
	} else {
		base_weight = PBF_REG_COS0_WEIGHT_P1;
		max_cos = DCBX_E3B0_MAX_NUM_COS_PORT1;
	}

	for (i = 0; i < max_cos; i++)
		REG_WR(bp, base_weight + (0x4 * i), 0);

	bnx2x_ets_e3b0_set_credit_upper_bound_pbf(params, min_w_val_pbf);
}
/******************************************************************************
* Description:
*	E3B0 disable will return basically the values to init values.
*.
******************************************************************************/
static int bnx2x_ets_e3b0_disabled(const struct link_params *params,
				   const struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;

	if (!CHIP_IS_E3B0(bp)) {
		DP(NETIF_MSG_LINK,
		   "bnx2x_ets_e3b0_disabled the chip isn't E3B0\n");
		return -EINVAL;
	}

	bnx2x_ets_e3b0_nig_disabled(params, vars);

	bnx2x_ets_e3b0_pbf_disabled(params);

	return 0;
}

/******************************************************************************
* Description:
*	Disable will return basically the values to init values.
*
******************************************************************************/
int bnx2x_ets_disabled(struct link_params *params,
		      struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	int bnx2x_status = 0;

	if ((CHIP_IS_E2(bp)) || (CHIP_IS_E3A0(bp)))
		bnx2x_ets_e2e3a0_disabled(params);
	else if (CHIP_IS_E3B0(bp))
		bnx2x_status = bnx2x_ets_e3b0_disabled(params, vars);
	else {
		DP(NETIF_MSG_LINK, "bnx2x_ets_disabled - chip not supported\n");
		return -EINVAL;
	}

	return bnx2x_status;
}

/******************************************************************************
* Description
*	Set the COS mappimg to SP and BW until this point all the COS are not
*	set as SP or BW.
******************************************************************************/
static int bnx2x_ets_e3b0_cli_map(const struct link_params *params,
				  const struct bnx2x_ets_params *ets_params,
				  const u8 cos_sp_bitmap,
				  const u8 cos_bw_bitmap)
{
	struct bnx2x *bp = params->bp;
	const u8 port = params->port;
	const u8 nig_cli_sp_bitmap = 0x7 | (cos_sp_bitmap << 3);
	const u8 pbf_cli_sp_bitmap = cos_sp_bitmap;
	const u8 nig_cli_subject2wfq_bitmap = cos_bw_bitmap << 3;
	const u8 pbf_cli_subject2wfq_bitmap = cos_bw_bitmap;

	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CLIENT_IS_STRICT :
	       NIG_REG_P0_TX_ARB_CLIENT_IS_STRICT, nig_cli_sp_bitmap);

	REG_WR(bp, (port) ? PBF_REG_ETS_ARB_CLIENT_IS_STRICT_P1 :
	       PBF_REG_ETS_ARB_CLIENT_IS_STRICT_P0 , pbf_cli_sp_bitmap);

	REG_WR(bp, (port) ? NIG_REG_P1_TX_ARB_CLIENT_IS_SUBJECT2WFQ :
	       NIG_REG_P0_TX_ARB_CLIENT_IS_SUBJECT2WFQ,
	       nig_cli_subject2wfq_bitmap);

	REG_WR(bp, (port) ? PBF_REG_ETS_ARB_CLIENT_IS_SUBJECT2WFQ_P1 :
	       PBF_REG_ETS_ARB_CLIENT_IS_SUBJECT2WFQ_P0,
	       pbf_cli_subject2wfq_bitmap);

	return 0;
}

/******************************************************************************
* Description:
*	This function is needed because NIG ARB_CREDIT_WEIGHT_X are
*	not continues and ARB_CREDIT_WEIGHT_0 + offset is suitable.
******************************************************************************/
static int bnx2x_ets_e3b0_set_cos_bw(struct bnx2x *bp,
				     const u8 cos_entry,
				     const u32 min_w_val_nig,
				     const u32 min_w_val_pbf,
				     const u16 total_bw,
				     const u8 bw,
				     const u8 port)
{
	u32 nig_reg_adress_crd_weight = 0;
	u32 pbf_reg_adress_crd_weight = 0;
	/* Calculate and set BW for this COS - use 1 instead of 0 for BW */
	const u32 cos_bw_nig = ((bw ? bw : 1) * min_w_val_nig) / total_bw;
	const u32 cos_bw_pbf = ((bw ? bw : 1) * min_w_val_pbf) / total_bw;

	switch (cos_entry) {
	case 0:
	    nig_reg_adress_crd_weight =
		 (port) ? NIG_REG_P1_TX_ARB_CREDIT_WEIGHT_0 :
		     NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_0;
	     pbf_reg_adress_crd_weight = (port) ?
		 PBF_REG_COS0_WEIGHT_P1 : PBF_REG_COS0_WEIGHT_P0;
	     break;
	case 1:
	     nig_reg_adress_crd_weight = (port) ?
		 NIG_REG_P1_TX_ARB_CREDIT_WEIGHT_1 :
		 NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_1;
	     pbf_reg_adress_crd_weight = (port) ?
		 PBF_REG_COS1_WEIGHT_P1 : PBF_REG_COS1_WEIGHT_P0;
	     break;
	case 2:
	     nig_reg_adress_crd_weight = (port) ?
		 NIG_REG_P1_TX_ARB_CREDIT_WEIGHT_2 :
		 NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_2;

		 pbf_reg_adress_crd_weight = (port) ?
		     PBF_REG_COS2_WEIGHT_P1 : PBF_REG_COS2_WEIGHT_P0;
	     break;
	case 3:
	    if (port)
			return -EINVAL;
	     nig_reg_adress_crd_weight =
		 NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_3;
	     pbf_reg_adress_crd_weight =
		 PBF_REG_COS3_WEIGHT_P0;
	     break;
	case 4:
	    if (port)
		return -EINVAL;
	     nig_reg_adress_crd_weight =
		 NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_4;
	     pbf_reg_adress_crd_weight = PBF_REG_COS4_WEIGHT_P0;
	     break;
	case 5:
	    if (port)
		return -EINVAL;
	     nig_reg_adress_crd_weight =
		 NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_5;
	     pbf_reg_adress_crd_weight = PBF_REG_COS5_WEIGHT_P0;
	     break;
	}

	REG_WR(bp, nig_reg_adress_crd_weight, cos_bw_nig);

	REG_WR(bp, pbf_reg_adress_crd_weight, cos_bw_pbf);

	return 0;
}
/******************************************************************************
* Description:
*	Calculate the total BW.A value of 0 isn't legal.
*
******************************************************************************/
static int bnx2x_ets_e3b0_get_total_bw(
	const struct link_params *params,
	struct bnx2x_ets_params *ets_params,
	u16 *total_bw)
{
	struct bnx2x *bp = params->bp;
	u8 cos_idx = 0;
	u8 is_bw_cos_exist = 0;

	*total_bw = 0 ;
	/* Calculate total BW requested */
	for (cos_idx = 0; cos_idx < ets_params->num_of_cos; cos_idx++) {
		if (ets_params->cos[cos_idx].state == bnx2x_cos_state_bw) {
			is_bw_cos_exist = 1;
			if (!ets_params->cos[cos_idx].params.bw_params.bw) {
				DP(NETIF_MSG_LINK, "bnx2x_ets_E3B0_config BW"
						   "was set to 0\n");
				/* This is to prevent a state when ramrods
				 * can't be sent
				 */
				ets_params->cos[cos_idx].params.bw_params.bw
					 = 1;
			}
			*total_bw +=
				ets_params->cos[cos_idx].params.bw_params.bw;
		}
	}

	/* Check total BW is valid */
	if ((is_bw_cos_exist == 1) && (*total_bw != 100)) {
		if (*total_bw == 0) {
			DP(NETIF_MSG_LINK,
			   "bnx2x_ets_E3B0_config total BW shouldn't be 0\n");
			return -EINVAL;
		}
		DP(NETIF_MSG_LINK,
		   "bnx2x_ets_E3B0_config total BW should be 100\n");
		/* We can handle a case whre the BW isn't 100 this can happen
		 * if the TC are joined.
		 */
	}
	return 0;
}

/******************************************************************************
* Description:
*	Invalidate all the sp_pri_to_cos.
*
******************************************************************************/
static void bnx2x_ets_e3b0_sp_pri_to_cos_init(u8 *sp_pri_to_cos)
{
	u8 pri = 0;
	for (pri = 0; pri < DCBX_MAX_NUM_COS; pri++)
		sp_pri_to_cos[pri] = DCBX_INVALID_COS;
}
/******************************************************************************
* Description:
*	Calculate and set the SP (ARB_PRIORITY_CLIENT) NIG and PBF registers
*	according to sp_pri_to_cos.
*
******************************************************************************/
static int bnx2x_ets_e3b0_sp_pri_to_cos_set(const struct link_params *params,
					    u8 *sp_pri_to_cos, const u8 pri,
					    const u8 cos_entry)
{
	struct bnx2x *bp = params->bp;
	const u8 port = params->port;
	const u8 max_num_of_cos = (port) ? DCBX_E3B0_MAX_NUM_COS_PORT1 :
		DCBX_E3B0_MAX_NUM_COS_PORT0;

	if (pri >= max_num_of_cos) {
		DP(NETIF_MSG_LINK, "bnx2x_ets_e3b0_sp_pri_to_cos_set invalid "
		   "parameter Illegal strict priority\n");
	    return -EINVAL;
	}

	if (sp_pri_to_cos[pri] != DCBX_INVALID_COS) {
		DP(NETIF_MSG_LINK, "bnx2x_ets_e3b0_sp_pri_to_cos_set invalid "
				   "parameter There can't be two COS's with "
				   "the same strict pri\n");
		return -EINVAL;
	}

	sp_pri_to_cos[pri] = cos_entry;
	return 0;

}

/******************************************************************************
* Description:
*	Returns the correct value according to COS and priority in
*	the sp_pri_cli register.
*
******************************************************************************/
static u64 bnx2x_e3b0_sp_get_pri_cli_reg(const u8 cos, const u8 cos_offset,
					 const u8 pri_set,
					 const u8 pri_offset,
					 const u8 entry_size)
{
	u64 pri_cli_nig = 0;
	pri_cli_nig = ((u64)(cos + cos_offset)) << (entry_size *
						    (pri_set + pri_offset));

	return pri_cli_nig;
}
/******************************************************************************
* Description:
*	Returns the correct value according to COS and priority in the
*	sp_pri_cli register for NIG.
*
******************************************************************************/
static u64 bnx2x_e3b0_sp_get_pri_cli_reg_nig(const u8 cos, const u8 pri_set)
{
	/* MCP Dbg0 and dbg1 are always with higher strict pri*/
	const u8 nig_cos_offset = 3;
	const u8 nig_pri_offset = 3;

	return bnx2x_e3b0_sp_get_pri_cli_reg(cos, nig_cos_offset, pri_set,
		nig_pri_offset, 4);

}
/******************************************************************************
* Description:
*	Returns the correct value according to COS and priority in the
*	sp_pri_cli register for PBF.
*
******************************************************************************/
static u64 bnx2x_e3b0_sp_get_pri_cli_reg_pbf(const u8 cos, const u8 pri_set)
{
	const u8 pbf_cos_offset = 0;
	const u8 pbf_pri_offset = 0;

	return bnx2x_e3b0_sp_get_pri_cli_reg(cos, pbf_cos_offset, pri_set,
		pbf_pri_offset, 3);

}

/******************************************************************************
* Description:
*	Calculate and set the SP (ARB_PRIORITY_CLIENT) NIG and PBF registers
*	according to sp_pri_to_cos.(which COS has higher priority)
*
******************************************************************************/
static int bnx2x_ets_e3b0_sp_set_pri_cli_reg(const struct link_params *params,
					     u8 *sp_pri_to_cos)
{
	struct bnx2x *bp = params->bp;
	u8 i = 0;
	const u8 port = params->port;
	/* MCP Dbg0 and dbg1 are always with higher strict pri*/
	u64 pri_cli_nig = 0x210;
	u32 pri_cli_pbf = 0x0;
	u8 pri_set = 0;
	u8 pri_bitmask = 0;
	const u8 max_num_of_cos = (port) ? DCBX_E3B0_MAX_NUM_COS_PORT1 :
		DCBX_E3B0_MAX_NUM_COS_PORT0;

	u8 cos_bit_to_set = (1 << max_num_of_cos) - 1;

	/* Set all the strict priority first */
	for (i = 0; i < max_num_of_cos; i++) {
		if (sp_pri_to_cos[i] != DCBX_INVALID_COS) {
			if (sp_pri_to_cos[i] >= DCBX_MAX_NUM_COS) {
				DP(NETIF_MSG_LINK,
					   "bnx2x_ets_e3b0_sp_set_pri_cli_reg "
					   "invalid cos entry\n");
				return -EINVAL;
			}

			pri_cli_nig |= bnx2x_e3b0_sp_get_pri_cli_reg_nig(
			    sp_pri_to_cos[i], pri_set);

			pri_cli_pbf |= bnx2x_e3b0_sp_get_pri_cli_reg_pbf(
			    sp_pri_to_cos[i], pri_set);
			pri_bitmask = 1 << sp_pri_to_cos[i];
			/* COS is used remove it from bitmap.*/
			if (!(pri_bitmask & cos_bit_to_set)) {
				DP(NETIF_MSG_LINK,
					"bnx2x_ets_e3b0_sp_set_pri_cli_reg "
					"invalid There can't be two COS's with"
					" the same strict pri\n");
				return -EINVAL;
			}
			cos_bit_to_set &= ~pri_bitmask;
			pri_set++;
		}
	}

	/* Set all the Non strict priority i= COS*/
	for (i = 0; i < max_num_of_cos; i++) {
		pri_bitmask = 1 << i;
		/* Check if COS was already used for SP */
		if (pri_bitmask & cos_bit_to_set) {
			/* COS wasn't used for SP */
			pri_cli_nig |= bnx2x_e3b0_sp_get_pri_cli_reg_nig(
			    i, pri_set);

			pri_cli_pbf |= bnx2x_e3b0_sp_get_pri_cli_reg_pbf(
			    i, pri_set);
			/* COS is used remove it from bitmap.*/
			cos_bit_to_set &= ~pri_bitmask;
			pri_set++;
		}
	}

	if (pri_set != max_num_of_cos) {
		DP(NETIF_MSG_LINK, "bnx2x_ets_e3b0_sp_set_pri_cli_reg not all "
				   "entries were set\n");
		return -EINVAL;
	}

	if (port) {
		/* Only 6 usable clients*/
		REG_WR(bp, NIG_REG_P1_TX_ARB_PRIORITY_CLIENT2_LSB,
		       (u32)pri_cli_nig);

		REG_WR(bp, PBF_REG_ETS_ARB_PRIORITY_CLIENT_P1 , pri_cli_pbf);
	} else {
		/* Only 9 usable clients*/
		const u32 pri_cli_nig_lsb = (u32) (pri_cli_nig);
		const u32 pri_cli_nig_msb = (u32) ((pri_cli_nig >> 32) & 0xF);

		REG_WR(bp, NIG_REG_P0_TX_ARB_PRIORITY_CLIENT2_LSB,
		       pri_cli_nig_lsb);
		REG_WR(bp, NIG_REG_P0_TX_ARB_PRIORITY_CLIENT2_MSB,
		       pri_cli_nig_msb);

		REG_WR(bp, PBF_REG_ETS_ARB_PRIORITY_CLIENT_P0 , pri_cli_pbf);
	}
	return 0;
}

/******************************************************************************
* Description:
*	Configure the COS to ETS according to BW and SP settings.
******************************************************************************/
int bnx2x_ets_e3b0_config(const struct link_params *params,
			 const struct link_vars *vars,
			 struct bnx2x_ets_params *ets_params)
{
	struct bnx2x *bp = params->bp;
	int bnx2x_status = 0;
	const u8 port = params->port;
	u16 total_bw = 0;
	const u32 min_w_val_nig = bnx2x_ets_get_min_w_val_nig(vars);
	const u32 min_w_val_pbf = ETS_E3B0_PBF_MIN_W_VAL;
	u8 cos_bw_bitmap = 0;
	u8 cos_sp_bitmap = 0;
	u8 sp_pri_to_cos[DCBX_MAX_NUM_COS] = {0};
	const u8 max_num_of_cos = (port) ? DCBX_E3B0_MAX_NUM_COS_PORT1 :
		DCBX_E3B0_MAX_NUM_COS_PORT0;
	u8 cos_entry = 0;

	if (!CHIP_IS_E3B0(bp)) {
		DP(NETIF_MSG_LINK,
		   "bnx2x_ets_e3b0_disabled the chip isn't E3B0\n");
		return -EINVAL;
	}

	if ((ets_params->num_of_cos > max_num_of_cos)) {
		DP(NETIF_MSG_LINK, "bnx2x_ets_E3B0_config the number of COS "
				   "isn't supported\n");
		return -EINVAL;
	}

	/* Prepare sp strict priority parameters*/
	bnx2x_ets_e3b0_sp_pri_to_cos_init(sp_pri_to_cos);

	/* Prepare BW parameters*/
	bnx2x_status = bnx2x_ets_e3b0_get_total_bw(params, ets_params,
						   &total_bw);
	if (bnx2x_status) {
		DP(NETIF_MSG_LINK,
		   "bnx2x_ets_E3B0_config get_total_bw failed\n");
		return -EINVAL;
	}

	/* Upper bound is set according to current link speed (min_w_val
	 * should be the same for upper bound and COS credit val).
	 */
	bnx2x_ets_e3b0_set_credit_upper_bound_nig(params, min_w_val_nig);
	bnx2x_ets_e3b0_set_credit_upper_bound_pbf(params, min_w_val_pbf);


	for (cos_entry = 0; cos_entry < ets_params->num_of_cos; cos_entry++) {
		if (bnx2x_cos_state_bw == ets_params->cos[cos_entry].state) {
			cos_bw_bitmap |= (1 << cos_entry);
			/* The function also sets the BW in HW(not the mappin
			 * yet)
			 */
			bnx2x_status = bnx2x_ets_e3b0_set_cos_bw(
				bp, cos_entry, min_w_val_nig, min_w_val_pbf,
				total_bw,
				ets_params->cos[cos_entry].params.bw_params.bw,
				 port);
		} else if (bnx2x_cos_state_strict ==
			ets_params->cos[cos_entry].state){
			cos_sp_bitmap |= (1 << cos_entry);

			bnx2x_status = bnx2x_ets_e3b0_sp_pri_to_cos_set(
				params,
				sp_pri_to_cos,
				ets_params->cos[cos_entry].params.sp_params.pri,
				cos_entry);

		} else {
			DP(NETIF_MSG_LINK,
			   "bnx2x_ets_e3b0_config cos state not valid\n");
			return -EINVAL;
		}
		if (bnx2x_status) {
			DP(NETIF_MSG_LINK,
			   "bnx2x_ets_e3b0_config set cos bw failed\n");
			return bnx2x_status;
		}
	}

	/* Set SP register (which COS has higher priority) */
	bnx2x_status = bnx2x_ets_e3b0_sp_set_pri_cli_reg(params,
							 sp_pri_to_cos);

	if (bnx2x_status) {
		DP(NETIF_MSG_LINK,
		   "bnx2x_ets_E3B0_config set_pri_cli_reg failed\n");
		return bnx2x_status;
	}

	/* Set client mapping of BW and strict */
	bnx2x_status = bnx2x_ets_e3b0_cli_map(params, ets_params,
					      cos_sp_bitmap,
					      cos_bw_bitmap);

	if (bnx2x_status) {
		DP(NETIF_MSG_LINK, "bnx2x_ets_E3B0_config SP failed\n");
		return bnx2x_status;
	}
	return 0;
}
static void bnx2x_ets_bw_limit_common(const struct link_params *params)
{
	/* ETS disabled configuration */
	struct bnx2x *bp = params->bp;
	DP(NETIF_MSG_LINK, "ETS enabled BW limit configuration\n");
	/* Defines which entries (clients) are subjected to WFQ arbitration
	 * COS0 0x8
	 * COS1 0x10
	 */
	REG_WR(bp, NIG_REG_P0_TX_ARB_CLIENT_IS_SUBJECT2WFQ, 0x18);
	/* Mapping between the ARB_CREDIT_WEIGHT registers and actual
	 * client numbers (WEIGHT_0 does not actually have to represent
	 * client 0)
	 *    PRI4    |    PRI3    |    PRI2    |    PRI1    |    PRI0
	 *  cos1-001     cos0-000     dbg1-100     dbg0-011     MCP-010
	 */
	REG_WR(bp, NIG_REG_P0_TX_ARB_CLIENT_CREDIT_MAP, 0x111A);

	REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_0,
	       ETS_BW_LIMIT_CREDIT_UPPER_BOUND);
	REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_UPPER_BOUND_1,
	       ETS_BW_LIMIT_CREDIT_UPPER_BOUND);

	/* ETS mode enabled*/
	REG_WR(bp, PBF_REG_ETS_ENABLED, 1);

	/* Defines the number of consecutive slots for the strict priority */
	REG_WR(bp, PBF_REG_NUM_STRICT_ARB_SLOTS, 0);
	/* Bitmap of 5bits length. Each bit specifies whether the entry behaves
	 * as strict.  Bits 0,1,2 - debug and management entries, 3 - COS0
	 * entry, 4 - COS1 entry.
	 * COS1 | COS0 | DEBUG21 | DEBUG0 | MGMT
	 * bit4   bit3	  bit2     bit1	   bit0
	 * MCP and debug are strict
	 */
	REG_WR(bp, NIG_REG_P0_TX_ARB_CLIENT_IS_STRICT, 0x7);

	/* Upper bound that COS0_WEIGHT can reach in the WFQ arbiter.*/
	REG_WR(bp, PBF_REG_COS0_UPPER_BOUND,
	       ETS_BW_LIMIT_CREDIT_UPPER_BOUND);
	REG_WR(bp, PBF_REG_COS1_UPPER_BOUND,
	       ETS_BW_LIMIT_CREDIT_UPPER_BOUND);
}

void bnx2x_ets_bw_limit(const struct link_params *params, const u32 cos0_bw,
			const u32 cos1_bw)
{
	/* ETS disabled configuration*/
	struct bnx2x *bp = params->bp;
	const u32 total_bw = cos0_bw + cos1_bw;
	u32 cos0_credit_weight = 0;
	u32 cos1_credit_weight = 0;

	DP(NETIF_MSG_LINK, "ETS enabled BW limit configuration\n");

	if ((!total_bw) ||
	    (!cos0_bw) ||
	    (!cos1_bw)) {
		DP(NETIF_MSG_LINK, "Total BW can't be zero\n");
		return;
	}

	cos0_credit_weight = (cos0_bw * ETS_BW_LIMIT_CREDIT_WEIGHT)/
		total_bw;
	cos1_credit_weight = (cos1_bw * ETS_BW_LIMIT_CREDIT_WEIGHT)/
		total_bw;

	bnx2x_ets_bw_limit_common(params);

	REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_0, cos0_credit_weight);
	REG_WR(bp, NIG_REG_P0_TX_ARB_CREDIT_WEIGHT_1, cos1_credit_weight);

	REG_WR(bp, PBF_REG_COS0_WEIGHT, cos0_credit_weight);
	REG_WR(bp, PBF_REG_COS1_WEIGHT, cos1_credit_weight);
}

int bnx2x_ets_strict(const struct link_params *params, const u8 strict_cos)
{
	/* ETS disabled configuration*/
	struct bnx2x *bp = params->bp;
	u32 val	= 0;

	DP(NETIF_MSG_LINK, "ETS enabled strict configuration\n");
	/* Bitmap of 5bits length. Each bit specifies whether the entry behaves
	 * as strict.  Bits 0,1,2 - debug and management entries,
	 * 3 - COS0 entry, 4 - COS1 entry.
	 *  COS1 | COS0 | DEBUG21 | DEBUG0 | MGMT
	 *  bit4   bit3	  bit2      bit1     bit0
	 * MCP and debug are strict
	 */
	REG_WR(bp, NIG_REG_P0_TX_ARB_CLIENT_IS_STRICT, 0x1F);
	/* For strict priority entries defines the number of consecutive slots
	 * for the highest priority.
	 */
	REG_WR(bp, NIG_REG_P0_TX_ARB_NUM_STRICT_ARB_SLOTS, 0x100);
	/* ETS mode disable */
	REG_WR(bp, PBF_REG_ETS_ENABLED, 0);
	/* Defines the number of consecutive slots for the strict priority */
	REG_WR(bp, PBF_REG_NUM_STRICT_ARB_SLOTS, 0x100);

	/* Defines the number of consecutive slots for the strict priority */
	REG_WR(bp, PBF_REG_HIGH_PRIORITY_COS_NUM, strict_cos);

	/* Mapping between entry  priority to client number (0,1,2 -debug and
	 * management clients, 3 - COS0 client, 4 - COS client)(HIGHEST)
	 * 3bits client num.
	 *   PRI4    |    PRI3    |    PRI2    |    PRI1    |    PRI0
	 * dbg0-010     dbg1-001     cos1-100     cos0-011     MCP-000
	 * dbg0-010     dbg1-001     cos0-011     cos1-100     MCP-000
	 */
	val = (!strict_cos) ? 0x2318 : 0x22E0;
	REG_WR(bp, NIG_REG_P0_TX_ARB_PRIORITY_CLIENT, val);

	return 0;
}

/******************************************************************/
/*			PFC section				  */
/******************************************************************/
static void bnx2x_update_pfc_xmac(struct link_params *params,
				  struct link_vars *vars,
				  u8 is_lb)
{
	struct bnx2x *bp = params->bp;
	u32 xmac_base;
	u32 pause_val, pfc0_val, pfc1_val;

	/* XMAC base adrr */
	xmac_base = (params->port) ? GRCBASE_XMAC1 : GRCBASE_XMAC0;

	/* Initialize pause and pfc registers */
	pause_val = 0x18000;
	pfc0_val = 0xFFFF8000;
	pfc1_val = 0x2;

	/* No PFC support */
	if (!(params->feature_config_flags &
	      FEATURE_CONFIG_PFC_ENABLED)) {

		/* RX flow control - Process pause frame in receive direction
		 */
		if (vars->flow_ctrl & BNX2X_FLOW_CTRL_RX)
			pause_val |= XMAC_PAUSE_CTRL_REG_RX_PAUSE_EN;

		/* TX flow control - Send pause packet when buffer is full */
		if (vars->flow_ctrl & BNX2X_FLOW_CTRL_TX)
			pause_val |= XMAC_PAUSE_CTRL_REG_TX_PAUSE_EN;
	} else {/* PFC support */
		pfc1_val |= XMAC_PFC_CTRL_HI_REG_PFC_REFRESH_EN |
			XMAC_PFC_CTRL_HI_REG_PFC_STATS_EN |
			XMAC_PFC_CTRL_HI_REG_RX_PFC_EN |
			XMAC_PFC_CTRL_HI_REG_TX_PFC_EN |
			XMAC_PFC_CTRL_HI_REG_FORCE_PFC_XON;
		/* Write pause and PFC registers */
		REG_WR(bp, xmac_base + XMAC_REG_PAUSE_CTRL, pause_val);
		REG_WR(bp, xmac_base + XMAC_REG_PFC_CTRL, pfc0_val);
		REG_WR(bp, xmac_base + XMAC_REG_PFC_CTRL_HI, pfc1_val);
		pfc1_val &= ~XMAC_PFC_CTRL_HI_REG_FORCE_PFC_XON;

	}

	/* Write pause and PFC registers */
	REG_WR(bp, xmac_base + XMAC_REG_PAUSE_CTRL, pause_val);
	REG_WR(bp, xmac_base + XMAC_REG_PFC_CTRL, pfc0_val);
	REG_WR(bp, xmac_base + XMAC_REG_PFC_CTRL_HI, pfc1_val);


	/* Set MAC address for source TX Pause/PFC frames */
	REG_WR(bp, xmac_base + XMAC_REG_CTRL_SA_LO,
	       ((params->mac_addr[2] << 24) |
		(params->mac_addr[3] << 16) |
		(params->mac_addr[4] << 8) |
		(params->mac_addr[5])));
	REG_WR(bp, xmac_base + XMAC_REG_CTRL_SA_HI,
	       ((params->mac_addr[0] << 8) |
		(params->mac_addr[1])));

	udelay(30);
}

/******************************************************************/
/*			MAC/PBF section				  */
/******************************************************************/
static void bnx2x_set_mdio_clk(struct bnx2x *bp, u32 chip_id,
			       u32 emac_base)
{
	u32 new_mode, cur_mode;
	u32 clc_cnt;
	/* Set clause 45 mode, slow down the MDIO clock to 2.5MHz
	 * (a value of 49==0x31) and make sure that the AUTO poll is off
	 */
	cur_mode = REG_RD(bp, emac_base + EMAC_REG_EMAC_MDIO_MODE);

	if (USES_WARPCORE(bp))
		clc_cnt = 74L << EMAC_MDIO_MODE_CLOCK_CNT_BITSHIFT;
	else
		clc_cnt = 49L << EMAC_MDIO_MODE_CLOCK_CNT_BITSHIFT;

	if (((cur_mode & EMAC_MDIO_MODE_CLOCK_CNT) == clc_cnt) &&
	    (cur_mode & (EMAC_MDIO_MODE_CLAUSE_45)))
		return;

	new_mode = cur_mode &
		~(EMAC_MDIO_MODE_AUTO_POLL | EMAC_MDIO_MODE_CLOCK_CNT);
	new_mode |= clc_cnt;
	new_mode |= (EMAC_MDIO_MODE_CLAUSE_45);

	DP(NETIF_MSG_LINK, "Changing emac_mode from 0x%x to 0x%x\n",
	   cur_mode, new_mode);
	REG_WR(bp, emac_base + EMAC_REG_EMAC_MDIO_MODE, new_mode);
	udelay(40);
}

static void bnx2x_set_mdio_emac_per_phy(struct bnx2x *bp,
					struct link_params *params)
{
	u8 phy_index;
	/* Set mdio clock per phy */
	for (phy_index = INT_PHY; phy_index < params->num_phys;
	      phy_index++)
		bnx2x_set_mdio_clk(bp, params->chip_id,
				   params->phy[phy_index].mdio_ctrl);
}

static u8 bnx2x_is_4_port_mode(struct bnx2x *bp)
{
	u32 port4mode_ovwr_val;
	/* Check 4-port override enabled */
	port4mode_ovwr_val = REG_RD(bp, MISC_REG_PORT4MODE_EN_OVWR);
	if (port4mode_ovwr_val & (1<<0)) {
		/* Return 4-port mode override value */
		return ((port4mode_ovwr_val & (1<<1)) == (1<<1));
	}
	/* Return 4-port mode from input pin */
	return (u8)REG_RD(bp, MISC_REG_PORT4MODE_EN);
}

static void bnx2x_emac_init(struct link_params *params,
			    struct link_vars *vars)
{
	/* reset and unreset the emac core */
	struct bnx2x *bp = params->bp;
	u8 port = params->port;
	u32 emac_base = port ? GRCBASE_EMAC1 : GRCBASE_EMAC0;
	u32 val;
	u16 timeout;

	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_2_CLEAR,
	       (MISC_REGISTERS_RESET_REG_2_RST_EMAC0_HARD_CORE << port));
	udelay(5);
	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_2_SET,
	       (MISC_REGISTERS_RESET_REG_2_RST_EMAC0_HARD_CORE << port));

	/* init emac - use read-modify-write */
	/* self clear reset */
	val = REG_RD(bp, emac_base + EMAC_REG_EMAC_MODE);
	EMAC_WR(bp, EMAC_REG_EMAC_MODE, (val | EMAC_MODE_RESET));

	timeout = 200;
	do {
		val = REG_RD(bp, emac_base + EMAC_REG_EMAC_MODE);
		DP(NETIF_MSG_LINK, "EMAC reset reg is %u\n", val);
		if (!timeout) {
			DP(NETIF_MSG_LINK, "EMAC timeout!\n");
			return;
		}
		timeout--;
	} while (val & EMAC_MODE_RESET);

	bnx2x_set_mdio_emac_per_phy(bp, params);
	/* Set mac address */
	val = ((params->mac_addr[0] << 8) |
		params->mac_addr[1]);
	EMAC_WR(bp, EMAC_REG_EMAC_MAC_MATCH, val);

	val = ((params->mac_addr[2] << 24) |
	       (params->mac_addr[3] << 16) |
	       (params->mac_addr[4] << 8) |
		params->mac_addr[5]);
	EMAC_WR(bp, EMAC_REG_EMAC_MAC_MATCH + 4, val);
}

static void bnx2x_set_xumac_nig(struct link_params *params,
				u16 tx_pause_en,
				u8 enable)
{
	struct bnx2x *bp = params->bp;

	REG_WR(bp, params->port ? NIG_REG_P1_MAC_IN_EN : NIG_REG_P0_MAC_IN_EN,
	       enable);
	REG_WR(bp, params->port ? NIG_REG_P1_MAC_OUT_EN : NIG_REG_P0_MAC_OUT_EN,
	       enable);
	REG_WR(bp, params->port ? NIG_REG_P1_MAC_PAUSE_OUT_EN :
	       NIG_REG_P0_MAC_PAUSE_OUT_EN, tx_pause_en);
}

static void bnx2x_set_umac_rxtx(struct link_params *params, u8 en)
{
	u32 umac_base = params->port ? GRCBASE_UMAC1 : GRCBASE_UMAC0;
	u32 val;
	struct bnx2x *bp = params->bp;
	if (!(REG_RD(bp, MISC_REG_RESET_REG_2) &
		   (MISC_REGISTERS_RESET_REG_2_UMAC0 << params->port)))
		return;
	val = REG_RD(bp, umac_base + UMAC_REG_COMMAND_CONFIG);
	if (en)
		val |= (UMAC_COMMAND_CONFIG_REG_TX_ENA |
			UMAC_COMMAND_CONFIG_REG_RX_ENA);
	else
		val &= ~(UMAC_COMMAND_CONFIG_REG_TX_ENA |
			 UMAC_COMMAND_CONFIG_REG_RX_ENA);
	/* Disable RX and TX */
	REG_WR(bp, umac_base + UMAC_REG_COMMAND_CONFIG, val);
}

static void bnx2x_umac_enable(struct link_params *params,
			    struct link_vars *vars, u8 lb)
{
	u32 val;
	u32 umac_base = params->port ? GRCBASE_UMAC1 : GRCBASE_UMAC0;
	struct bnx2x *bp = params->bp;
	/* Reset UMAC */
	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_2_CLEAR,
	       (MISC_REGISTERS_RESET_REG_2_UMAC0 << params->port));
	usleep_range(1000, 2000);

	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_2_SET,
	       (MISC_REGISTERS_RESET_REG_2_UMAC0 << params->port));

	DP(NETIF_MSG_LINK, "enabling UMAC\n");

	/* This register opens the gate for the UMAC despite its name */
	REG_WR(bp, NIG_REG_EGRESS_EMAC0_PORT + params->port*4, 1);

	val = UMAC_COMMAND_CONFIG_REG_PROMIS_EN |
		UMAC_COMMAND_CONFIG_REG_PAD_EN |
		UMAC_COMMAND_CONFIG_REG_SW_RESET |
		UMAC_COMMAND_CONFIG_REG_NO_LGTH_CHECK;
	switch (vars->line_speed) {
	case SPEED_10:
		val |= (0<<2);
		break;
	case SPEED_100:
		val |= (1<<2);
		break;
	case SPEED_1000:
		val |= (2<<2);
		break;
	case SPEED_2500:
		val |= (3<<2);
		break;
	default:
		DP(NETIF_MSG_LINK, "Invalid speed for UMAC %d\n",
			       vars->line_speed);
		break;
	}
	if (!(vars->flow_ctrl & BNX2X_FLOW_CTRL_TX))
		val |= UMAC_COMMAND_CONFIG_REG_IGNORE_TX_PAUSE;

	if (!(vars->flow_ctrl & BNX2X_FLOW_CTRL_RX))
		val |= UMAC_COMMAND_CONFIG_REG_PAUSE_IGNORE;

	if (vars->duplex == DUPLEX_HALF)
		val |= UMAC_COMMAND_CONFIG_REG_HD_ENA;

	REG_WR(bp, umac_base + UMAC_REG_COMMAND_CONFIG, val);
	udelay(50);

	/* Configure UMAC for EEE */
	if (vars->eee_status & SHMEM_EEE_ADV_STATUS_MASK) {
		DP(NETIF_MSG_LINK, "configured UMAC for EEE\n");
		REG_WR(bp, umac_base + UMAC_REG_UMAC_EEE_CTRL,
		       UMAC_UMAC_EEE_CTRL_REG_EEE_EN);
		REG_WR(bp, umac_base + UMAC_REG_EEE_WAKE_TIMER, 0x11);
	} else {
		REG_WR(bp, umac_base + UMAC_REG_UMAC_EEE_CTRL, 0x0);
	}

	/* Set MAC address for source TX Pause/PFC frames (under SW reset) */
	REG_WR(bp, umac_base + UMAC_REG_MAC_ADDR0,
	       ((params->mac_addr[2] << 24) |
		(params->mac_addr[3] << 16) |
		(params->mac_addr[4] << 8) |
		(params->mac_addr[5])));
	REG_WR(bp, umac_base + UMAC_REG_MAC_ADDR1,
	       ((params->mac_addr[0] << 8) |
		(params->mac_addr[1])));

	/* Enable RX and TX */
	val &= ~UMAC_COMMAND_CONFIG_REG_PAD_EN;
	val |= UMAC_COMMAND_CONFIG_REG_TX_ENA |
		UMAC_COMMAND_CONFIG_REG_RX_ENA;
	REG_WR(bp, umac_base + UMAC_REG_COMMAND_CONFIG, val);
	udelay(50);

	/* Remove SW Reset */
	val &= ~UMAC_COMMAND_CONFIG_REG_SW_RESET;

	/* Check loopback mode */
	if (lb)
		val |= UMAC_COMMAND_CONFIG_REG_LOOP_ENA;
	REG_WR(bp, umac_base + UMAC_REG_COMMAND_CONFIG, val);

	/* Maximum Frame Length (RW). Defines a 14-Bit maximum frame
	 * length used by the MAC receive logic to check frames.
	 */
	REG_WR(bp, umac_base + UMAC_REG_MAXFR, 0x2710);
	bnx2x_set_xumac_nig(params,
			    ((vars->flow_ctrl & BNX2X_FLOW_CTRL_TX) != 0), 1);
	vars->mac_type = MAC_TYPE_UMAC;

}

/* Define the XMAC mode */
static void bnx2x_xmac_init(struct link_params *params, u32 max_speed)
{
	struct bnx2x *bp = params->bp;
	u32 is_port4mode = bnx2x_is_4_port_mode(bp);

	/* In 4-port mode, need to set the mode only once, so if XMAC is
	 * already out of reset, it means the mode has already been set,
	 * and it must not* reset the XMAC again, since it controls both
	 * ports of the path
	 */

	if (((CHIP_NUM(bp) == CHIP_NUM_57840_4_10) ||
	     (CHIP_NUM(bp) == CHIP_NUM_57840_2_20) ||
	     (CHIP_NUM(bp) == CHIP_NUM_57840_OBSOLETE)) &&
	    is_port4mode &&
	    (REG_RD(bp, MISC_REG_RESET_REG_2) &
	     MISC_REGISTERS_RESET_REG_2_XMAC)) {
		DP(NETIF_MSG_LINK,
		   "XMAC already out of reset in 4-port mode\n");
		return;
	}

	/* Hard reset */
	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_2_CLEAR,
	       MISC_REGISTERS_RESET_REG_2_XMAC);
	usleep_range(1000, 2000);

	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_2_SET,
	       MISC_REGISTERS_RESET_REG_2_XMAC);
	if (is_port4mode) {
		DP(NETIF_MSG_LINK, "Init XMAC to 2 ports x 10G per path\n");

		/* Set the number of ports on the system side to up to 2 */
		REG_WR(bp, MISC_REG_XMAC_CORE_PORT_MODE, 1);

		/* Set the number of ports on the Warp Core to 10G */
		REG_WR(bp, MISC_REG_XMAC_PHY_PORT_MODE, 3);
	} else {
		/* Set the number of ports on the system side to 1 */
		REG_WR(bp, MISC_REG_XMAC_CORE_PORT_MODE, 0);
		if (max_speed == SPEED_10000) {
			DP(NETIF_MSG_LINK,
			   "Init XMAC to 10G x 1 port per path\n");
			/* Set the number of ports on the Warp Core to 10G */
			REG_WR(bp, MISC_REG_XMAC_PHY_PORT_MODE, 3);
		} else {
			DP(NETIF_MSG_LINK,
			   "Init XMAC to 20G x 2 ports per path\n");
			/* Set the number of ports on the Warp Core to 20G */
			REG_WR(bp, MISC_REG_XMAC_PHY_PORT_MODE, 1);
		}
	}
	/* Soft reset */
	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_2_CLEAR,
	       MISC_REGISTERS_RESET_REG_2_XMAC_SOFT);
	usleep_range(1000, 2000);

	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_2_SET,
	       MISC_REGISTERS_RESET_REG_2_XMAC_SOFT);

}

static void bnx2x_set_xmac_rxtx(struct link_params *params, u8 en)
{
	u8 port = params->port;
	struct bnx2x *bp = params->bp;
	u32 pfc_ctrl, xmac_base = (port) ? GRCBASE_XMAC1 : GRCBASE_XMAC0;
	u32 val;

	if (REG_RD(bp, MISC_REG_RESET_REG_2) &
	    MISC_REGISTERS_RESET_REG_2_XMAC) {
		/* Send an indication to change the state in the NIG back to XON
		 * Clearing this bit enables the next set of this bit to get
		 * rising edge
		 */
		pfc_ctrl = REG_RD(bp, xmac_base + XMAC_REG_PFC_CTRL_HI);
		REG_WR(bp, xmac_base + XMAC_REG_PFC_CTRL_HI,
		       (pfc_ctrl & ~(1<<1)));
		REG_WR(bp, xmac_base + XMAC_REG_PFC_CTRL_HI,
		       (pfc_ctrl | (1<<1)));
		DP(NETIF_MSG_LINK, "Disable XMAC on port %x\n", port);
		val = REG_RD(bp, xmac_base + XMAC_REG_CTRL);
		if (en)
			val |= (XMAC_CTRL_REG_TX_EN | XMAC_CTRL_REG_RX_EN);
		else
			val &= ~(XMAC_CTRL_REG_TX_EN | XMAC_CTRL_REG_RX_EN);
		REG_WR(bp, xmac_base + XMAC_REG_CTRL, val);
	}
}

static int bnx2x_xmac_enable(struct link_params *params,
			     struct link_vars *vars, u8 lb)
{
	u32 val, xmac_base;
	struct bnx2x *bp = params->bp;
	DP(NETIF_MSG_LINK, "enabling XMAC\n");

	xmac_base = (params->port) ? GRCBASE_XMAC1 : GRCBASE_XMAC0;

	bnx2x_xmac_init(params, vars->line_speed);

	/* This register determines on which events the MAC will assert
	 * error on the i/f to the NIG along w/ EOP.
	 */

	/* This register tells the NIG whether to send traffic to UMAC
	 * or XMAC
	 */
	REG_WR(bp, NIG_REG_EGRESS_EMAC0_PORT + params->port*4, 0);

	/* When XMAC is in XLGMII mode, disable sending idles for fault
	 * detection.
	 */
	if (!(params->phy[INT_PHY].flags & FLAGS_TX_ERROR_CHECK)) {
		REG_WR(bp, xmac_base + XMAC_REG_RX_LSS_CTRL,
		       (XMAC_RX_LSS_CTRL_REG_LOCAL_FAULT_DISABLE |
			XMAC_RX_LSS_CTRL_REG_REMOTE_FAULT_DISABLE));
		REG_WR(bp, xmac_base + XMAC_REG_CLEAR_RX_LSS_STATUS, 0);
		REG_WR(bp, xmac_base + XMAC_REG_CLEAR_RX_LSS_STATUS,
		       XMAC_CLEAR_RX_LSS_STATUS_REG_CLEAR_LOCAL_FAULT_STATUS |
		       XMAC_CLEAR_RX_LSS_STATUS_REG_CLEAR_REMOTE_FAULT_STATUS);
	}
	/* Set Max packet size */
	REG_WR(bp, xmac_base + XMAC_REG_RX_MAX_SIZE, 0x2710);

	/* CRC append for Tx packets */
	REG_WR(bp, xmac_base + XMAC_REG_TX_CTRL, 0xC800);

	/* update PFC */
	bnx2x_update_pfc_xmac(params, vars, 0);

	if (vars->eee_status & SHMEM_EEE_ADV_STATUS_MASK) {
		DP(NETIF_MSG_LINK, "Setting XMAC for EEE\n");
		REG_WR(bp, xmac_base + XMAC_REG_EEE_TIMERS_HI, 0x1380008);
		REG_WR(bp, xmac_base + XMAC_REG_EEE_CTRL, 0x1);
	} else {
		REG_WR(bp, xmac_base + XMAC_REG_EEE_CTRL, 0x0);
	}

	/* Enable TX and RX */
	val = XMAC_CTRL_REG_TX_EN | XMAC_CTRL_REG_RX_EN;

	/* Set MAC in XLGMII mode for dual-mode */
	if ((vars->line_speed == SPEED_20000) &&
	    (params->phy[INT_PHY].supported &
	     SUPPORTED_20000baseKR2_Full))
		val |= XMAC_CTRL_REG_XLGMII_ALIGN_ENB;

	/* Check loopback mode */
	if (lb)
		val |= XMAC_CTRL_REG_LINE_LOCAL_LPBK;
	REG_WR(bp, xmac_base + XMAC_REG_CTRL, val);
	bnx2x_set_xumac_nig(params,
			    ((vars->flow_ctrl & BNX2X_FLOW_CTRL_TX) != 0), 1);

	vars->mac_type = MAC_TYPE_XMAC;

	return 0;
}

static int bnx2x_emac_enable(struct link_params *params,
			     struct link_vars *vars, u8 lb)
{
	struct bnx2x *bp = params->bp;
	u8 port = params->port;
	u32 emac_base = port ? GRCBASE_EMAC1 : GRCBASE_EMAC0;
	u32 val;

	DP(NETIF_MSG_LINK, "enabling EMAC\n");

	/* Disable BMAC */
	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_2_CLEAR,
	       (MISC_REGISTERS_RESET_REG_2_RST_BMAC0 << port));

	/* enable emac and not bmac */
	REG_WR(bp, NIG_REG_EGRESS_EMAC0_PORT + port*4, 1);

	/* ASIC */
	if (vars->phy_flags & PHY_XGXS_FLAG) {
		u32 ser_lane = ((params->lane_config &
				 PORT_HW_CFG_LANE_SWAP_CFG_MASTER_MASK) >>
				PORT_HW_CFG_LANE_SWAP_CFG_MASTER_SHIFT);

		DP(NETIF_MSG_LINK, "XGXS\n");
		/* select the master lanes (out of 0-3) */
		REG_WR(bp, NIG_REG_XGXS_LANE_SEL_P0 + port*4, ser_lane);
		/* select XGXS */
		REG_WR(bp, NIG_REG_XGXS_SERDES0_MODE_SEL + port*4, 1);

	} else { /* SerDes */
		DP(NETIF_MSG_LINK, "SerDes\n");
		/* select SerDes */
		REG_WR(bp, NIG_REG_XGXS_SERDES0_MODE_SEL + port*4, 0);
	}

	bnx2x_bits_en(bp, emac_base + EMAC_REG_EMAC_RX_MODE,
		      EMAC_RX_MODE_RESET);
	bnx2x_bits_en(bp, emac_base + EMAC_REG_EMAC_TX_MODE,
		      EMAC_TX_MODE_RESET);

		/* pause enable/disable */
		bnx2x_bits_dis(bp, emac_base + EMAC_REG_EMAC_RX_MODE,
			       EMAC_RX_MODE_FLOW_EN);

		bnx2x_bits_dis(bp,  emac_base + EMAC_REG_EMAC_TX_MODE,
			       (EMAC_TX_MODE_EXT_PAUSE_EN |
				EMAC_TX_MODE_FLOW_EN));
		if (!(params->feature_config_flags &
		      FEATURE_CONFIG_PFC_ENABLED)) {
			if (vars->flow_ctrl & BNX2X_FLOW_CTRL_RX)
				bnx2x_bits_en(bp, emac_base +
					      EMAC_REG_EMAC_RX_MODE,
					      EMAC_RX_MODE_FLOW_EN);

			if (vars->flow_ctrl & BNX2X_FLOW_CTRL_TX)
				bnx2x_bits_en(bp, emac_base +
					      EMAC_REG_EMAC_TX_MODE,
					      (EMAC_TX_MODE_EXT_PAUSE_EN |
					       EMAC_TX_MODE_FLOW_EN));
		} else
			bnx2x_bits_en(bp, emac_base + EMAC_REG_EMAC_TX_MODE,
				      EMAC_TX_MODE_FLOW_EN);

	/* KEEP_VLAN_TAG, promiscuous */
	val = REG_RD(bp, emac_base + EMAC_REG_EMAC_RX_MODE);
	val |= EMAC_RX_MODE_KEEP_VLAN_TAG | EMAC_RX_MODE_PROMISCUOUS;

	/* Setting this bit causes MAC control frames (except for pause
	 * frames) to be passed on for processing. This setting has no
	 * affect on the operation of the pause frames. This bit effects
	 * all packets regardless of RX Parser packet sorting logic.
	 * Turn the PFC off to make sure we are in Xon state before
	 * enabling it.
	 */
	EMAC_WR(bp, EMAC_REG_RX_PFC_MODE, 0);
	if (params->feature_config_flags & FEATURE_CONFIG_PFC_ENABLED) {
		DP(NETIF_MSG_LINK, "PFC is enabled\n");
		/* Enable PFC again */
		EMAC_WR(bp, EMAC_REG_RX_PFC_MODE,
			EMAC_REG_RX_PFC_MODE_RX_EN |
			EMAC_REG_RX_PFC_MODE_TX_EN |
			EMAC_REG_RX_PFC_MODE_PRIORITIES);

		EMAC_WR(bp, EMAC_REG_RX_PFC_PARAM,
			((0x0101 <<
			  EMAC_REG_RX_PFC_PARAM_OPCODE_BITSHIFT) |
			 (0x00ff <<
			  EMAC_REG_RX_PFC_PARAM_PRIORITY_EN_BITSHIFT)));
		val |= EMAC_RX_MODE_KEEP_MAC_CONTROL;
	}
	EMAC_WR(bp, EMAC_REG_EMAC_RX_MODE, val);

	/* Set Loopback */
	val = REG_RD(bp, emac_base + EMAC_REG_EMAC_MODE);
	if (lb)
		val |= 0x810;
	else
		val &= ~0x810;
	EMAC_WR(bp, EMAC_REG_EMAC_MODE, val);

	/* Enable emac */
	REG_WR(bp, NIG_REG_NIG_EMAC0_EN + port*4, 1);

	/* Enable emac for jumbo packets */
	EMAC_WR(bp, EMAC_REG_EMAC_RX_MTU_SIZE,
		(EMAC_RX_MTU_SIZE_JUMBO_ENA |
		 (ETH_MAX_JUMBO_PACKET_SIZE + ETH_OVERHEAD)));

	/* Strip CRC */
	REG_WR(bp, NIG_REG_NIG_INGRESS_EMAC0_NO_CRC + port*4, 0x1);

	/* Disable the NIG in/out to the bmac */
	REG_WR(bp, NIG_REG_BMAC0_IN_EN + port*4, 0x0);
	REG_WR(bp, NIG_REG_BMAC0_PAUSE_OUT_EN + port*4, 0x0);
	REG_WR(bp, NIG_REG_BMAC0_OUT_EN + port*4, 0x0);

	/* Enable the NIG in/out to the emac */
	REG_WR(bp, NIG_REG_EMAC0_IN_EN + port*4, 0x1);
	val = 0;
	if ((params->feature_config_flags &
	      FEATURE_CONFIG_PFC_ENABLED) ||
	    (vars->flow_ctrl & BNX2X_FLOW_CTRL_TX))
		val = 1;

	REG_WR(bp, NIG_REG_EMAC0_PAUSE_OUT_EN + port*4, val);
	REG_WR(bp, NIG_REG_EGRESS_EMAC0_OUT_EN + port*4, 0x1);

	REG_WR(bp, NIG_REG_BMAC0_REGS_OUT_EN + port*4, 0x0);

	vars->mac_type = MAC_TYPE_EMAC;
	return 0;
}

static void bnx2x_update_pfc_bmac1(struct link_params *params,
				   struct link_vars *vars)
{
	u32 wb_data[2];
	struct bnx2x *bp = params->bp;
	u32 bmac_addr =  params->port ? NIG_REG_INGRESS_BMAC1_MEM :
		NIG_REG_INGRESS_BMAC0_MEM;

	u32 val = 0x14;
	if ((!(params->feature_config_flags &
	      FEATURE_CONFIG_PFC_ENABLED)) &&
		(vars->flow_ctrl & BNX2X_FLOW_CTRL_RX))
		/* Enable BigMAC to react on received Pause packets */
		val |= (1<<5);
	wb_data[0] = val;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC_REGISTER_RX_CONTROL, wb_data, 2);

	/* TX control */
	val = 0xc0;
	if (!(params->feature_config_flags &
	      FEATURE_CONFIG_PFC_ENABLED) &&
		(vars->flow_ctrl & BNX2X_FLOW_CTRL_TX))
		val |= 0x800000;
	wb_data[0] = val;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC_REGISTER_TX_CONTROL, wb_data, 2);
}

static void bnx2x_update_pfc_bmac2(struct link_params *params,
				   struct link_vars *vars,
				   u8 is_lb)
{
	/* Set rx control: Strip CRC and enable BigMAC to relay
	 * control packets to the system as well
	 */
	u32 wb_data[2];
	struct bnx2x *bp = params->bp;
	u32 bmac_addr = params->port ? NIG_REG_INGRESS_BMAC1_MEM :
		NIG_REG_INGRESS_BMAC0_MEM;
	u32 val = 0x14;

	if ((!(params->feature_config_flags &
	      FEATURE_CONFIG_PFC_ENABLED)) &&
		(vars->flow_ctrl & BNX2X_FLOW_CTRL_RX))
		/* Enable BigMAC to react on received Pause packets */
		val |= (1<<5);
	wb_data[0] = val;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_RX_CONTROL, wb_data, 2);
	udelay(30);

	/* Tx control */
	val = 0xc0;
	if (!(params->feature_config_flags &
				FEATURE_CONFIG_PFC_ENABLED) &&
	    (vars->flow_ctrl & BNX2X_FLOW_CTRL_TX))
		val |= 0x800000;
	wb_data[0] = val;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_TX_CONTROL, wb_data, 2);

	if (params->feature_config_flags & FEATURE_CONFIG_PFC_ENABLED) {
		DP(NETIF_MSG_LINK, "PFC is enabled\n");
		/* Enable PFC RX & TX & STATS and set 8 COS  */
		wb_data[0] = 0x0;
		wb_data[0] |= (1<<0);  /* RX */
		wb_data[0] |= (1<<1);  /* TX */
		wb_data[0] |= (1<<2);  /* Force initial Xon */
		wb_data[0] |= (1<<3);  /* 8 cos */
		wb_data[0] |= (1<<5);  /* STATS */
		wb_data[1] = 0;
		REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_PFC_CONTROL,
			    wb_data, 2);
		/* Clear the force Xon */
		wb_data[0] &= ~(1<<2);
	} else {
		DP(NETIF_MSG_LINK, "PFC is disabled\n");
		/* Disable PFC RX & TX & STATS and set 8 COS */
		wb_data[0] = 0x8;
		wb_data[1] = 0;
	}

	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_PFC_CONTROL, wb_data, 2);

	/* Set Time (based unit is 512 bit time) between automatic
	 * re-sending of PP packets amd enable automatic re-send of
	 * Per-Priroity Packet as long as pp_gen is asserted and
	 * pp_disable is low.
	 */
	val = 0x8000;
	if (params->feature_config_flags & FEATURE_CONFIG_PFC_ENABLED)
		val |= (1<<16); /* enable automatic re-send */

	wb_data[0] = val;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_TX_PAUSE_CONTROL,
		    wb_data, 2);

	/* mac control */
	val = 0x3; /* Enable RX and TX */
	if (is_lb) {
		val |= 0x4; /* Local loopback */
		DP(NETIF_MSG_LINK, "enable bmac loopback\n");
	}
	/* When PFC enabled, Pass pause frames towards the NIG. */
	if (params->feature_config_flags & FEATURE_CONFIG_PFC_ENABLED)
		val |= ((1<<6)|(1<<5));

	wb_data[0] = val;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_BMAC_CONTROL, wb_data, 2);
}

/******************************************************************************
* Description:
*  This function is needed because NIG ARB_CREDIT_WEIGHT_X are
*  not continues and ARB_CREDIT_WEIGHT_0 + offset is suitable.
******************************************************************************/
static int bnx2x_pfc_nig_rx_priority_mask(struct bnx2x *bp,
					   u8 cos_entry,
					   u32 priority_mask, u8 port)
{
	u32 nig_reg_rx_priority_mask_add = 0;

	switch (cos_entry) {
	case 0:
	     nig_reg_rx_priority_mask_add = (port) ?
		 NIG_REG_P1_RX_COS0_PRIORITY_MASK :
		 NIG_REG_P0_RX_COS0_PRIORITY_MASK;
	     break;
	case 1:
	    nig_reg_rx_priority_mask_add = (port) ?
		NIG_REG_P1_RX_COS1_PRIORITY_MASK :
		NIG_REG_P0_RX_COS1_PRIORITY_MASK;
	    break;
	case 2:
	    nig_reg_rx_priority_mask_add = (port) ?
		NIG_REG_P1_RX_COS2_PRIORITY_MASK :
		NIG_REG_P0_RX_COS2_PRIORITY_MASK;
	    break;
	case 3:
	    if (port)
		return -EINVAL;
	    nig_reg_rx_priority_mask_add = NIG_REG_P0_RX_COS3_PRIORITY_MASK;
	    break;
	case 4:
	    if (port)
		return -EINVAL;
	    nig_reg_rx_priority_mask_add = NIG_REG_P0_RX_COS4_PRIORITY_MASK;
	    break;
	case 5:
	    if (port)
		return -EINVAL;
	    nig_reg_rx_priority_mask_add = NIG_REG_P0_RX_COS5_PRIORITY_MASK;
	    break;
	}

	REG_WR(bp, nig_reg_rx_priority_mask_add, priority_mask);

	return 0;
}
static void bnx2x_update_mng(struct link_params *params, u32 link_status)
{
	struct bnx2x *bp = params->bp;

	REG_WR(bp, params->shmem_base +
	       offsetof(struct shmem_region,
			port_mb[params->port].link_status), link_status);
}

static void bnx2x_update_link_attr(struct link_params *params, u32 link_attr)
{
	struct bnx2x *bp = params->bp;

	if (SHMEM2_HAS(bp, link_attr_sync))
		REG_WR(bp, params->shmem2_base +
		       offsetof(struct shmem2_region,
				link_attr_sync[params->port]), link_attr);
}

static void bnx2x_update_pfc_nig(struct link_params *params,
		struct link_vars *vars,
		struct bnx2x_nig_brb_pfc_port_params *nig_params)
{
	u32 xcm_mask = 0, ppp_enable = 0, pause_enable = 0, llfc_out_en = 0;
	u32 llfc_enable = 0, xcm_out_en = 0, hwpfc_enable = 0;
	u32 pkt_priority_to_cos = 0;
	struct bnx2x *bp = params->bp;
	u8 port = params->port;

	int set_pfc = params->feature_config_flags &
		FEATURE_CONFIG_PFC_ENABLED;
	DP(NETIF_MSG_LINK, "updating pfc nig parameters\n");

	/* When NIG_LLH0_XCM_MASK_REG_LLHX_XCM_MASK_BCN bit is set
	 * MAC control frames (that are not pause packets)
	 * will be forwarded to the XCM.
	 */
	xcm_mask = REG_RD(bp, port ? NIG_REG_LLH1_XCM_MASK :
			  NIG_REG_LLH0_XCM_MASK);
	/* NIG params will override non PFC params, since it's possible to
	 * do transition from PFC to SAFC
	 */
	if (set_pfc) {
		pause_enable = 0;
		llfc_out_en = 0;
		llfc_enable = 0;
		if (CHIP_IS_E3(bp))
			ppp_enable = 0;
		else
			ppp_enable = 1;
		xcm_mask &= ~(port ? NIG_LLH1_XCM_MASK_REG_LLH1_XCM_MASK_BCN :
				     NIG_LLH0_XCM_MASK_REG_LLH0_XCM_MASK_BCN);
		xcm_out_en = 0;
		hwpfc_enable = 1;
	} else  {
		if (nig_params) {
			llfc_out_en = nig_params->llfc_out_en;
			llfc_enable = nig_params->llfc_enable;
			pause_enable = nig_params->pause_enable;
		} else  /* Default non PFC mode - PAUSE */
			pause_enable = 1;

		xcm_mask |= (port ? NIG_LLH1_XCM_MASK_REG_LLH1_XCM_MASK_BCN :
			NIG_LLH0_XCM_MASK_REG_LLH0_XCM_MASK_BCN);
		xcm_out_en = 1;
	}

	if (CHIP_IS_E3(bp))
		REG_WR(bp, port ? NIG_REG_BRB1_PAUSE_IN_EN :
		       NIG_REG_BRB0_PAUSE_IN_EN, pause_enable);
	REG_WR(bp, port ? NIG_REG_LLFC_OUT_EN_1 :
	       NIG_REG_LLFC_OUT_EN_0, llfc_out_en);
	REG_WR(bp, port ? NIG_REG_LLFC_ENABLE_1 :
	       NIG_REG_LLFC_ENABLE_0, llfc_enable);
	REG_WR(bp, port ? NIG_REG_PAUSE_ENABLE_1 :
	       NIG_REG_PAUSE_ENABLE_0, pause_enable);

	REG_WR(bp, port ? NIG_REG_PPP_ENABLE_1 :
	       NIG_REG_PPP_ENABLE_0, ppp_enable);

	REG_WR(bp, port ? NIG_REG_LLH1_XCM_MASK :
	       NIG_REG_LLH0_XCM_MASK, xcm_mask);

	REG_WR(bp, port ? NIG_REG_LLFC_EGRESS_SRC_ENABLE_1 :
	       NIG_REG_LLFC_EGRESS_SRC_ENABLE_0, 0x7);

	/* Output enable for RX_XCM # IF */
	REG_WR(bp, port ? NIG_REG_XCM1_OUT_EN :
	       NIG_REG_XCM0_OUT_EN, xcm_out_en);

	/* HW PFC TX enable */
	REG_WR(bp, port ? NIG_REG_P1_HWPFC_ENABLE :
	       NIG_REG_P0_HWPFC_ENABLE, hwpfc_enable);

	if (nig_params) {
		u8 i = 0;
		pkt_priority_to_cos = nig_params->pkt_priority_to_cos;

		for (i = 0; i < nig_params->num_of_rx_cos_priority_mask; i++)
			bnx2x_pfc_nig_rx_priority_mask(bp, i,
		nig_params->rx_cos_priority_mask[i], port);

		REG_WR(bp, port ? NIG_REG_LLFC_HIGH_PRIORITY_CLASSES_1 :
		       NIG_REG_LLFC_HIGH_PRIORITY_CLASSES_0,
		       nig_params->llfc_high_priority_classes);

		REG_WR(bp, port ? NIG_REG_LLFC_LOW_PRIORITY_CLASSES_1 :
		       NIG_REG_LLFC_LOW_PRIORITY_CLASSES_0,
		       nig_params->llfc_low_priority_classes);
	}
	REG_WR(bp, port ? NIG_REG_P1_PKT_PRIORITY_TO_COS :
	       NIG_REG_P0_PKT_PRIORITY_TO_COS,
	       pkt_priority_to_cos);
}

int bnx2x_update_pfc(struct link_params *params,
		      struct link_vars *vars,
		      struct bnx2x_nig_brb_pfc_port_params *pfc_params)
{
	/* The PFC and pause are orthogonal to one another, meaning when
	 * PFC is enabled, the pause are disabled, and when PFC is
	 * disabled, pause are set according to the pause result.
	 */
	u32 val;
	struct bnx2x *bp = params->bp;
	u8 bmac_loopback = (params->loopback_mode == LOOPBACK_BMAC);

	if (params->feature_config_flags & FEATURE_CONFIG_PFC_ENABLED)
		vars->link_status |= LINK_STATUS_PFC_ENABLED;
	else
		vars->link_status &= ~LINK_STATUS_PFC_ENABLED;

	bnx2x_update_mng(params, vars->link_status);

	/* Update NIG params */
	bnx2x_update_pfc_nig(params, vars, pfc_params);

	if (!vars->link_up)
		return 0;

	DP(NETIF_MSG_LINK, "About to update PFC in BMAC\n");

	if (CHIP_IS_E3(bp)) {
		if (vars->mac_type == MAC_TYPE_XMAC)
			bnx2x_update_pfc_xmac(params, vars, 0);
	} else {
		val = REG_RD(bp, MISC_REG_RESET_REG_2);
		if ((val &
		     (MISC_REGISTERS_RESET_REG_2_RST_BMAC0 << params->port))
		    == 0) {
			DP(NETIF_MSG_LINK, "About to update PFC in EMAC\n");
			bnx2x_emac_enable(params, vars, 0);
			return 0;
		}
		if (CHIP_IS_E2(bp))
			bnx2x_update_pfc_bmac2(params, vars, bmac_loopback);
		else
			bnx2x_update_pfc_bmac1(params, vars);

		val = 0;
		if ((params->feature_config_flags &
		     FEATURE_CONFIG_PFC_ENABLED) ||
		    (vars->flow_ctrl & BNX2X_FLOW_CTRL_TX))
			val = 1;
		REG_WR(bp, NIG_REG_BMAC0_PAUSE_OUT_EN + params->port*4, val);
	}
	return 0;
}

static int bnx2x_bmac1_enable(struct link_params *params,
			      struct link_vars *vars,
			      u8 is_lb)
{
	struct bnx2x *bp = params->bp;
	u8 port = params->port;
	u32 bmac_addr = port ? NIG_REG_INGRESS_BMAC1_MEM :
			       NIG_REG_INGRESS_BMAC0_MEM;
	u32 wb_data[2];
	u32 val;

	DP(NETIF_MSG_LINK, "Enabling BigMAC1\n");

	/* XGXS control */
	wb_data[0] = 0x3c;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC_REGISTER_BMAC_XGXS_CONTROL,
		    wb_data, 2);

	/* TX MAC SA */
	wb_data[0] = ((params->mac_addr[2] << 24) |
		       (params->mac_addr[3] << 16) |
		       (params->mac_addr[4] << 8) |
			params->mac_addr[5]);
	wb_data[1] = ((params->mac_addr[0] << 8) |
			params->mac_addr[1]);
	REG_WR_DMAE(bp, bmac_addr + BIGMAC_REGISTER_TX_SOURCE_ADDR, wb_data, 2);

	/* MAC control */
	val = 0x3;
	if (is_lb) {
		val |= 0x4;
		DP(NETIF_MSG_LINK, "enable bmac loopback\n");
	}
	wb_data[0] = val;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC_REGISTER_BMAC_CONTROL, wb_data, 2);

	/* Set rx mtu */
	wb_data[0] = ETH_MAX_JUMBO_PACKET_SIZE + ETH_OVERHEAD;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC_REGISTER_RX_MAX_SIZE, wb_data, 2);

	bnx2x_update_pfc_bmac1(params, vars);

	/* Set tx mtu */
	wb_data[0] = ETH_MAX_JUMBO_PACKET_SIZE + ETH_OVERHEAD;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC_REGISTER_TX_MAX_SIZE, wb_data, 2);

	/* Set cnt max size */
	wb_data[0] = ETH_MAX_JUMBO_PACKET_SIZE + ETH_OVERHEAD;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC_REGISTER_CNT_MAX_SIZE, wb_data, 2);

	/* Configure SAFC */
	wb_data[0] = 0x1000200;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC_REGISTER_RX_LLFC_MSG_FLDS,
		    wb_data, 2);

	return 0;
}

static int bnx2x_bmac2_enable(struct link_params *params,
			      struct link_vars *vars,
			      u8 is_lb)
{
	struct bnx2x *bp = params->bp;
	u8 port = params->port;
	u32 bmac_addr = port ? NIG_REG_INGRESS_BMAC1_MEM :
			       NIG_REG_INGRESS_BMAC0_MEM;
	u32 wb_data[2];

	DP(NETIF_MSG_LINK, "Enabling BigMAC2\n");

	wb_data[0] = 0;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_BMAC_CONTROL, wb_data, 2);
	udelay(30);

	/* XGXS control: Reset phy HW, MDIO registers, PHY PLL and BMAC */
	wb_data[0] = 0x3c;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_BMAC_XGXS_CONTROL,
		    wb_data, 2);

	udelay(30);

	/* TX MAC SA */
	wb_data[0] = ((params->mac_addr[2] << 24) |
		       (params->mac_addr[3] << 16) |
		       (params->mac_addr[4] << 8) |
			params->mac_addr[5]);
	wb_data[1] = ((params->mac_addr[0] << 8) |
			params->mac_addr[1]);
	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_TX_SOURCE_ADDR,
		    wb_data, 2);

	udelay(30);

	/* Configure SAFC */
	wb_data[0] = 0x1000200;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_RX_LLFC_MSG_FLDS,
		    wb_data, 2);
	udelay(30);

	/* Set RX MTU */
	wb_data[0] = ETH_MAX_JUMBO_PACKET_SIZE + ETH_OVERHEAD;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_RX_MAX_SIZE, wb_data, 2);
	udelay(30);

	/* Set TX MTU */
	wb_data[0] = ETH_MAX_JUMBO_PACKET_SIZE + ETH_OVERHEAD;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_TX_MAX_SIZE, wb_data, 2);
	udelay(30);
	/* Set cnt max size */
	wb_data[0] = ETH_MAX_JUMBO_PACKET_SIZE + ETH_OVERHEAD - 2;
	wb_data[1] = 0;
	REG_WR_DMAE(bp, bmac_addr + BIGMAC2_REGISTER_CNT_MAX_SIZE, wb_data, 2);
	udelay(30);
	bnx2x_update_pfc_bmac2(params, vars, is_lb);

	return 0;
}

static int bnx2x_bmac_enable(struct link_params *params,
			     struct link_vars *vars,
			     u8 is_lb, u8 reset_bmac)
{
	int rc = 0;
	u8 port = params->port;
	struct bnx2x *bp = params->bp;
	u32 val;
	/* Reset and unreset the BigMac */
	if (reset_bmac) {
		REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_2_CLEAR,
		       (MISC_REGISTERS_RESET_REG_2_RST_BMAC0 << port));
		usleep_range(1000, 2000);
	}

	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_2_SET,
	       (MISC_REGISTERS_RESET_REG_2_RST_BMAC0 << port));

	/* Enable access for bmac registers */
	REG_WR(bp, NIG_REG_BMAC0_REGS_OUT_EN + port*4, 0x1);

	/* Enable BMAC according to BMAC type*/
	if (CHIP_IS_E2(bp))
		rc = bnx2x_bmac2_enable(params, vars, is_lb);
	else
		rc = bnx2x_bmac1_enable(params, vars, is_lb);
	REG_WR(bp, NIG_REG_XGXS_SERDES0_MODE_SEL + port*4, 0x1);
	REG_WR(bp, NIG_REG_XGXS_LANE_SEL_P0 + port*4, 0x0);
	REG_WR(bp, NIG_REG_EGRESS_EMAC0_PORT + port*4, 0x0);
	val = 0;
	if ((params->feature_config_flags &
	      FEATURE_CONFIG_PFC_ENABLED) ||
	    (vars->flow_ctrl & BNX2X_FLOW_CTRL_TX))
		val = 1;
	REG_WR(bp, NIG_REG_BMAC0_PAUSE_OUT_EN + port*4, val);
	REG_WR(bp, NIG_REG_EGRESS_EMAC0_OUT_EN + port*4, 0x0);
	REG_WR(bp, NIG_REG_EMAC0_IN_EN + port*4, 0x0);
	REG_WR(bp, NIG_REG_EMAC0_PAUSE_OUT_EN + port*4, 0x0);
	REG_WR(bp, NIG_REG_BMAC0_IN_EN + port*4, 0x1);
	REG_WR(bp, NIG_REG_BMAC0_OUT_EN + port*4, 0x1);

	vars->mac_type = MAC_TYPE_BMAC;
	return rc;
}

static void bnx2x_set_bmac_rx(struct bnx2x *bp, u32 chip_id, u8 port, u8 en)
{
	u32 bmac_addr = port ? NIG_REG_INGRESS_BMAC1_MEM :
			NIG_REG_INGRESS_BMAC0_MEM;
	u32 wb_data[2];
	u32 nig_bmac_enable = REG_RD(bp, NIG_REG_BMAC0_REGS_OUT_EN + port*4);

	if (CHIP_IS_E2(bp))
		bmac_addr += BIGMAC2_REGISTER_BMAC_CONTROL;
	else
		bmac_addr += BIGMAC_REGISTER_BMAC_CONTROL;
	/* Only if the bmac is out of reset */
	if (REG_RD(bp, MISC_REG_RESET_REG_2) &
			(MISC_REGISTERS_RESET_REG_2_RST_BMAC0 << port) &&
	    nig_bmac_enable) {
		/* Clear Rx Enable bit in BMAC_CONTROL register */
		REG_RD_DMAE(bp, bmac_addr, wb_data, 2);
		if (en)
			wb_data[0] |= BMAC_CONTROL_RX_ENABLE;
		else
			wb_data[0] &= ~BMAC_CONTROL_RX_ENABLE;
		REG_WR_DMAE(bp, bmac_addr, wb_data, 2);
		usleep_range(1000, 2000);
	}
}

static int bnx2x_pbf_update(struct link_params *params, u32 flow_ctrl,
			    u32 line_speed)
{
	struct bnx2x *bp = params->bp;
	u8 port = params->port;
	u32 init_crd, crd;
	u32 count = 1000;

	/* Disable port */
	REG_WR(bp, PBF_REG_DISABLE_NEW_TASK_PROC_P0 + port*4, 0x1);

	/* Wait for init credit */
	init_crd = REG_RD(bp, PBF_REG_P0_INIT_CRD + port*4);
	crd = REG_RD(bp, PBF_REG_P0_CREDIT + port*8);
	DP(NETIF_MSG_LINK, "init_crd 0x%x  crd 0x%x\n", init_crd, crd);

	while ((init_crd != crd) && count) {
		usleep_range(5000, 10000);
		crd = REG_RD(bp, PBF_REG_P0_CREDIT + port*8);
		count--;
	}
	crd = REG_RD(bp, PBF_REG_P0_CREDIT + port*8);
	if (init_crd != crd) {
		DP(NETIF_MSG_LINK, "BUG! init_crd 0x%x != crd 0x%x\n",
			  init_crd, crd);
		return -EINVAL;
	}

	if (flow_ctrl & BNX2X_FLOW_CTRL_RX ||
	    line_speed == SPEED_10 ||
	    line_speed == SPEED_100 ||
	    line_speed == SPEED_1000 ||
	    line_speed == SPEED_2500) {
		REG_WR(bp, PBF_REG_P0_PAUSE_ENABLE + port*4, 1);
		/* Update threshold */
		REG_WR(bp, PBF_REG_P0_ARB_THRSH + port*4, 0);
		/* Update init credit */
		init_crd = 778;		/* (800-18-4) */

	} else {
		u32 thresh = (ETH_MAX_JUMBO_PACKET_SIZE +
			      ETH_OVERHEAD)/16;
		REG_WR(bp, PBF_REG_P0_PAUSE_ENABLE + port*4, 0);
		/* Update threshold */
		REG_WR(bp, PBF_REG_P0_ARB_THRSH + port*4, thresh);
		/* Update init credit */
		switch (line_speed) {
		case SPEED_10000:
			init_crd = thresh + 553 - 22;
			break;
		default:
			DP(NETIF_MSG_LINK, "Invalid line_speed 0x%x\n",
				  line_speed);
			return -EINVAL;
		}
	}
	REG_WR(bp, PBF_REG_P0_INIT_CRD + port*4, init_crd);
	DP(NETIF_MSG_LINK, "PBF updated to speed %d credit %d\n",
		 line_speed, init_crd);

	/* Probe the credit changes */
	REG_WR(bp, PBF_REG_INIT_P0 + port*4, 0x1);
	usleep_range(5000, 10000);
	REG_WR(bp, PBF_REG_INIT_P0 + port*4, 0x0);

	/* Enable port */
	REG_WR(bp, PBF_REG_DISABLE_NEW_TASK_PROC_P0 + port*4, 0x0);
	return 0;
}

/**
 * bnx2x_get_emac_base - retrive emac base address
 *
 * @bp:			driver handle
 * @mdc_mdio_access:	access type
 * @port:		port id
 *
 * This function selects the MDC/MDIO access (through emac0 or
 * emac1) depend on the mdc_mdio_access, port, port swapped. Each
 * phy has a default access mode, which could also be overridden
 * by nvram configuration. This parameter, whether this is the
 * default phy configuration, or the nvram overrun
 * configuration, is passed here as mdc_mdio_access and selects
 * the emac_base for the CL45 read/writes operations
 */
static u32 bnx2x_get_emac_base(struct bnx2x *bp,
			       u32 mdc_mdio_access, u8 port)
{
	u32 emac_base = 0;
	switch (mdc_mdio_access) {
	case SHARED_HW_CFG_MDC_MDIO_ACCESS1_PHY_TYPE:
		break;
	case SHARED_HW_CFG_MDC_MDIO_ACCESS1_EMAC0:
		if (REG_RD(bp, NIG_REG_PORT_SWAP))
			emac_base = GRCBASE_EMAC1;
		else
			emac_base = GRCBASE_EMAC0;
		break;
	case SHARED_HW_CFG_MDC_MDIO_ACCESS1_EMAC1:
		if (REG_RD(bp, NIG_REG_PORT_SWAP))
			emac_base = GRCBASE_EMAC0;
		else
			emac_base = GRCBASE_EMAC1;
		break;
	case SHARED_HW_CFG_MDC_MDIO_ACCESS1_BOTH:
		emac_base = (port) ? GRCBASE_EMAC1 : GRCBASE_EMAC0;
		break;
	case SHARED_HW_CFG_MDC_MDIO_ACCESS1_SWAPPED:
		emac_base = (port) ? GRCBASE_EMAC0 : GRCBASE_EMAC1;
		break;
	default:
		break;
	}
	return emac_base;

}

/******************************************************************/
/*			CL22 access functions			  */
/******************************************************************/
static int bnx2x_cl22_write(struct bnx2x *bp,
				       struct bnx2x_phy *phy,
				       u16 reg, u16 val)
{
	u32 tmp, mode;
	u8 i;
	int rc = 0;
	/* Switch to CL22 */
	mode = REG_RD(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_MODE);
	REG_WR(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_MODE,
	       mode & ~EMAC_MDIO_MODE_CLAUSE_45);

	/* Address */
	tmp = ((phy->addr << 21) | (reg << 16) | val |
	       EMAC_MDIO_COMM_COMMAND_WRITE_22 |
	       EMAC_MDIO_COMM_START_BUSY);
	REG_WR(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_COMM, tmp);

	for (i = 0; i < 50; i++) {
		udelay(10);

		tmp = REG_RD(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_COMM);
		if (!(tmp & EMAC_MDIO_COMM_START_BUSY)) {
			udelay(5);
			break;
		}
	}
	if (tmp & EMAC_MDIO_COMM_START_BUSY) {
		DP(NETIF_MSG_LINK, "write phy register failed\n");
		rc = -EFAULT;
	}
	REG_WR(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_MODE, mode);
	return rc;
}

static int bnx2x_cl22_read(struct bnx2x *bp,
				      struct bnx2x_phy *phy,
				      u16 reg, u16 *ret_val)
{
	u32 val, mode;
	u16 i;
	int rc = 0;

	/* Switch to CL22 */
	mode = REG_RD(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_MODE);
	REG_WR(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_MODE,
	       mode & ~EMAC_MDIO_MODE_CLAUSE_45);

	/* Address */
	val = ((phy->addr << 21) | (reg << 16) |
	       EMAC_MDIO_COMM_COMMAND_READ_22 |
	       EMAC_MDIO_COMM_START_BUSY);
	REG_WR(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_COMM, val);

	for (i = 0; i < 50; i++) {
		udelay(10);

		val = REG_RD(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_COMM);
		if (!(val & EMAC_MDIO_COMM_START_BUSY)) {
			*ret_val = (u16)(val & EMAC_MDIO_COMM_DATA);
			udelay(5);
			break;
		}
	}
	if (val & EMAC_MDIO_COMM_START_BUSY) {
		DP(NETIF_MSG_LINK, "read phy register failed\n");

		*ret_val = 0;
		rc = -EFAULT;
	}
	REG_WR(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_MODE, mode);
	return rc;
}

/******************************************************************/
/*			CL45 access functions			  */
/******************************************************************/
static int bnx2x_cl45_read(struct bnx2x *bp, struct bnx2x_phy *phy,
			   u8 devad, u16 reg, u16 *ret_val)
{
	u32 val;
	u16 i;
	int rc = 0;
	u32 chip_id;
	if (phy->flags & FLAGS_MDC_MDIO_WA_G) {
		chip_id = (REG_RD(bp, MISC_REG_CHIP_NUM) << 16) |
			  ((REG_RD(bp, MISC_REG_CHIP_REV) & 0xf) << 12);
		bnx2x_set_mdio_clk(bp, chip_id, phy->mdio_ctrl);
	}

	if (phy->flags & FLAGS_MDC_MDIO_WA_B0)
		bnx2x_bits_en(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_STATUS,
			      EMAC_MDIO_STATUS_10MB);
	/* Address */
	val = ((phy->addr << 21) | (devad << 16) | reg |
	       EMAC_MDIO_COMM_COMMAND_ADDRESS |
	       EMAC_MDIO_COMM_START_BUSY);
	REG_WR(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_COMM, val);

	for (i = 0; i < 50; i++) {
		udelay(10);

		val = REG_RD(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_COMM);
		if (!(val & EMAC_MDIO_COMM_START_BUSY)) {
			udelay(5);
			break;
		}
	}
	if (val & EMAC_MDIO_COMM_START_BUSY) {
		DP(NETIF_MSG_LINK, "read phy register failed\n");
		netdev_err(bp->dev,  "MDC/MDIO access timeout\n");
		*ret_val = 0;
		rc = -EFAULT;
	} else {
		/* Data */
		val = ((phy->addr << 21) | (devad << 16) |
		       EMAC_MDIO_COMM_COMMAND_READ_45 |
		       EMAC_MDIO_COMM_START_BUSY);
		REG_WR(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_COMM, val);

		for (i = 0; i < 50; i++) {
			udelay(10);

			val = REG_RD(bp, phy->mdio_ctrl +
				     EMAC_REG_EMAC_MDIO_COMM);
			if (!(val & EMAC_MDIO_COMM_START_BUSY)) {
				*ret_val = (u16)(val & EMAC_MDIO_COMM_DATA);
				break;
			}
		}
		if (val & EMAC_MDIO_COMM_START_BUSY) {
			DP(NETIF_MSG_LINK, "read phy register failed\n");
			netdev_err(bp->dev,  "MDC/MDIO access timeout\n");
			*ret_val = 0;
			rc = -EFAULT;
		}
	}
	/* Work around for E3 A0 */
	if (phy->flags & FLAGS_MDC_MDIO_WA) {
		phy->flags ^= FLAGS_DUMMY_READ;
		if (phy->flags & FLAGS_DUMMY_READ) {
			u16 temp_val;
			bnx2x_cl45_read(bp, phy, devad, 0xf, &temp_val);
		}
	}

	if (phy->flags & FLAGS_MDC_MDIO_WA_B0)
		bnx2x_bits_dis(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_STATUS,
			       EMAC_MDIO_STATUS_10MB);
	return rc;
}

static int bnx2x_cl45_write(struct bnx2x *bp, struct bnx2x_phy *phy,
			    u8 devad, u16 reg, u16 val)
{
	u32 tmp;
	u8 i;
	int rc = 0;
	u32 chip_id;
	if (phy->flags & FLAGS_MDC_MDIO_WA_G) {
		chip_id = (REG_RD(bp, MISC_REG_CHIP_NUM) << 16) |
			  ((REG_RD(bp, MISC_REG_CHIP_REV) & 0xf) << 12);
		bnx2x_set_mdio_clk(bp, chip_id, phy->mdio_ctrl);
	}

	if (phy->flags & FLAGS_MDC_MDIO_WA_B0)
		bnx2x_bits_en(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_STATUS,
			      EMAC_MDIO_STATUS_10MB);

	/* Address */
	tmp = ((phy->addr << 21) | (devad << 16) | reg |
	       EMAC_MDIO_COMM_COMMAND_ADDRESS |
	       EMAC_MDIO_COMM_START_BUSY);
	REG_WR(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_COMM, tmp);

	for (i = 0; i < 50; i++) {
		udelay(10);

		tmp = REG_RD(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_COMM);
		if (!(tmp & EMAC_MDIO_COMM_START_BUSY)) {
			udelay(5);
			break;
		}
	}
	if (tmp & EMAC_MDIO_COMM_START_BUSY) {
		DP(NETIF_MSG_LINK, "write phy register failed\n");
		netdev_err(bp->dev,  "MDC/MDIO access timeout\n");
		rc = -EFAULT;
	} else {
		/* Data */
		tmp = ((phy->addr << 21) | (devad << 16) | val |
		       EMAC_MDIO_COMM_COMMAND_WRITE_45 |
		       EMAC_MDIO_COMM_START_BUSY);
		REG_WR(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_COMM, tmp);

		for (i = 0; i < 50; i++) {
			udelay(10);

			tmp = REG_RD(bp, phy->mdio_ctrl +
				     EMAC_REG_EMAC_MDIO_COMM);
			if (!(tmp & EMAC_MDIO_COMM_START_BUSY)) {
				udelay(5);
				break;
			}
		}
		if (tmp & EMAC_MDIO_COMM_START_BUSY) {
			DP(NETIF_MSG_LINK, "write phy register failed\n");
			netdev_err(bp->dev,  "MDC/MDIO access timeout\n");
			rc = -EFAULT;
		}
	}
	/* Work around for E3 A0 */
	if (phy->flags & FLAGS_MDC_MDIO_WA) {
		phy->flags ^= FLAGS_DUMMY_READ;
		if (phy->flags & FLAGS_DUMMY_READ) {
			u16 temp_val;
			bnx2x_cl45_read(bp, phy, devad, 0xf, &temp_val);
		}
	}
	if (phy->flags & FLAGS_MDC_MDIO_WA_B0)
		bnx2x_bits_dis(bp, phy->mdio_ctrl + EMAC_REG_EMAC_MDIO_STATUS,
			       EMAC_MDIO_STATUS_10MB);
	return rc;
}

/******************************************************************/
/*			EEE section				   */
/******************************************************************/
static u8 bnx2x_eee_has_cap(struct link_params *params)
{
	struct bnx2x *bp = params->bp;

	if (REG_RD(bp, params->shmem2_base) <=
		   offsetof(struct shmem2_region, eee_status[params->port]))
		return 0;

	return 1;
}

static int bnx2x_eee_nvram_to_time(u32 nvram_mode, u32 *idle_timer)
{
	switch (nvram_mode) {
	case PORT_FEAT_CFG_EEE_POWER_MODE_BALANCED:
		*idle_timer = EEE_MODE_NVRAM_BALANCED_TIME;
		break;
	case PORT_FEAT_CFG_EEE_POWER_MODE_AGGRESSIVE:
		*idle_timer = EEE_MODE_NVRAM_AGGRESSIVE_TIME;
		break;
	case PORT_FEAT_CFG_EEE_POWER_MODE_LOW_LATENCY:
		*idle_timer = EEE_MODE_NVRAM_LATENCY_TIME;
		break;
	default:
		*idle_timer = 0;
		break;
	}

	return 0;
}

static int bnx2x_eee_time_to_nvram(u32 idle_timer, u32 *nvram_mode)
{
	switch (idle_timer) {
	case EEE_MODE_NVRAM_BALANCED_TIME:
		*nvram_mode = PORT_FEAT_CFG_EEE_POWER_MODE_BALANCED;
		break;
	case EEE_MODE_NVRAM_AGGRESSIVE_TIME:
		*nvram_mode = PORT_FEAT_CFG_EEE_POWER_MODE_AGGRESSIVE;
		break;
	case EEE_MODE_NVRAM_LATENCY_TIME:
		*nvram_mode = PORT_FEAT_CFG_EEE_POWER_MODE_LOW_LATENCY;
		break;
	default:
		*nvram_mode = PORT_FEAT_CFG_EEE_POWER_MODE_DISABLED;
		break;
	}

	return 0;
}

static u32 bnx2x_eee_calc_timer(struct link_params *params)
{
	u32 eee_mode, eee_idle;
	struct bnx2x *bp = params->bp;

	if (params->eee_mode & EEE_MODE_OVERRIDE_NVRAM) {
		if (params->eee_mode & EEE_MODE_OUTPUT_TIME) {
			/* time value in eee_mode --> used directly*/
			eee_idle = params->eee_mode & EEE_MODE_TIMER_MASK;
		} else {
			/* hsi value in eee_mode --> time */
			if (bnx2x_eee_nvram_to_time(params->eee_mode &
						    EEE_MODE_NVRAM_MASK,
						    &eee_idle))
				return 0;
		}
	} else {
		/* hsi values in nvram --> time*/
		eee_mode = ((REG_RD(bp, params->shmem_base +
				    offsetof(struct shmem_region, dev_info.
				    port_feature_config[params->port].
				    eee_power_mode)) &
			     PORT_FEAT_CFG_EEE_POWER_MODE_MASK) >>
			    PORT_FEAT_CFG_EEE_POWER_MODE_SHIFT);

		if (bnx2x_eee_nvram_to_time(eee_mode, &eee_idle))
			return 0;
	}

	return eee_idle;
}

static int bnx2x_eee_set_timers(struct link_params *params,
				   struct link_vars *vars)
{
	u32 eee_idle = 0, eee_mode;
	struct bnx2x *bp = params->bp;

	eee_idle = bnx2x_eee_calc_timer(params);

	if (eee_idle) {
		REG_WR(bp, MISC_REG_CPMU_LP_IDLE_THR_P0 + (params->port << 2),
		       eee_idle);
	} else if ((params->eee_mode & EEE_MODE_ENABLE_LPI) &&
		   (params->eee_mode & EEE_MODE_OVERRIDE_NVRAM) &&
		   (params->eee_mode & EEE_MODE_OUTPUT_TIME)) {
		DP(NETIF_MSG_LINK, "Error: Tx LPI is enabled with timer 0\n");
		return -EINVAL;
	}

	vars->eee_status &= ~(SHMEM_EEE_TIMER_MASK | SHMEM_EEE_TIME_OUTPUT_BIT);
	if (params->eee_mode & EEE_MODE_OUTPUT_TIME) {
		/* eee_idle in 1u --> eee_status in 16u */
		eee_idle >>= 4;
		vars->eee_status |= (eee_idle & SHMEM_EEE_TIMER_MASK) |
				    SHMEM_EEE_TIME_OUTPUT_BIT;
	} else {
		if (bnx2x_eee_time_to_nvram(eee_idle, &eee_mode))
			return -EINVAL;
		vars->eee_status |= eee_mode;
	}

	return 0;
}

static int bnx2x_eee_initial_config(struct link_params *params,
				     struct link_vars *vars, u8 mode)
{
	vars->eee_status |= ((u32) mode) << SHMEM_EEE_SUPPORTED_SHIFT;

	/* Propagate params' bits --> vars (for migration exposure) */
	if (params->eee_mode & EEE_MODE_ENABLE_LPI)
		vars->eee_status |= SHMEM_EEE_LPI_REQUESTED_BIT;
	else
		vars->eee_status &= ~SHMEM_EEE_LPI_REQUESTED_BIT;

	if (params->eee_mode & EEE_MODE_ADV_LPI)
		vars->eee_status |= SHMEM_EEE_REQUESTED_BIT;
	else
		vars->eee_status &= ~SHMEM_EEE_REQUESTED_BIT;

	return bnx2x_eee_set_timers(params, vars);
}

static int bnx2x_eee_disable(struct bnx2x_phy *phy,
				struct link_params *params,
				struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;

	/* Make Certain LPI is disabled */
	REG_WR(bp, MISC_REG_CPMU_LP_FW_ENABLE_P0 + (params->port << 2), 0);

	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_EEE_ADV, 0x0);

	vars->eee_status &= ~SHMEM_EEE_ADV_STATUS_MASK;

	return 0;
}

static int bnx2x_eee_advertise(struct bnx2x_phy *phy,
				  struct link_params *params,
				  struct link_vars *vars, u8 modes)
{
	struct bnx2x *bp = params->bp;
	u16 val = 0;

	/* Mask events preventing LPI generation */
	REG_WR(bp, MISC_REG_CPMU_LP_MASK_EXT_P0 + (params->port << 2), 0xfc20);

	if (modes & SHMEM_EEE_10G_ADV) {
		DP(NETIF_MSG_LINK, "Advertise 10GBase-T EEE\n");
		val |= 0x8;
	}
	if (modes & SHMEM_EEE_1G_ADV) {
		DP(NETIF_MSG_LINK, "Advertise 1GBase-T EEE\n");
		val |= 0x4;
	}

	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_EEE_ADV, val);

	vars->eee_status &= ~SHMEM_EEE_ADV_STATUS_MASK;
	vars->eee_status |= (modes << SHMEM_EEE_ADV_STATUS_SHIFT);

	return 0;
}

static void bnx2x_update_mng_eee(struct link_params *params, u32 eee_status)
{
	struct bnx2x *bp = params->bp;

	if (bnx2x_eee_has_cap(params))
		REG_WR(bp, params->shmem2_base +
		       offsetof(struct shmem2_region,
				eee_status[params->port]), eee_status);
}

static void bnx2x_eee_an_resolve(struct bnx2x_phy *phy,
				  struct link_params *params,
				  struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u16 adv = 0, lp = 0;
	u32 lp_adv = 0;
	u8 neg = 0;

	bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_EEE_ADV, &adv);
	bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_LP_EEE_ADV, &lp);

	if (lp & 0x2) {
		lp_adv |= SHMEM_EEE_100M_ADV;
		if (adv & 0x2) {
			if (vars->line_speed == SPEED_100)
				neg = 1;
			DP(NETIF_MSG_LINK, "EEE negotiated - 100M\n");
		}
	}
	if (lp & 0x14) {
		lp_adv |= SHMEM_EEE_1G_ADV;
		if (adv & 0x14) {
			if (vars->line_speed == SPEED_1000)
				neg = 1;
			DP(NETIF_MSG_LINK, "EEE negotiated - 1G\n");
		}
	}
	if (lp & 0x68) {
		lp_adv |= SHMEM_EEE_10G_ADV;
		if (adv & 0x68) {
			if (vars->line_speed == SPEED_10000)
				neg = 1;
			DP(NETIF_MSG_LINK, "EEE negotiated - 10G\n");
		}
	}

	vars->eee_status &= ~SHMEM_EEE_LP_ADV_STATUS_MASK;
	vars->eee_status |= (lp_adv << SHMEM_EEE_LP_ADV_STATUS_SHIFT);

	if (neg) {
		DP(NETIF_MSG_LINK, "EEE is active\n");
		vars->eee_status |= SHMEM_EEE_ACTIVE_BIT;
	}

}

/******************************************************************/
/*			BSC access functions from E3	          */
/******************************************************************/
static void bnx2x_bsc_module_sel(struct link_params *params)
{
	int idx;
	u32 board_cfg, sfp_ctrl;
	u32 i2c_pins[I2C_SWITCH_WIDTH], i2c_val[I2C_SWITCH_WIDTH];
	struct bnx2x *bp = params->bp;
	u8 port = params->port;
	/* Read I2C output PINs */
	board_cfg = REG_RD(bp, params->shmem_base +
			   offsetof(struct shmem_region,
				    dev_info.shared_hw_config.board));
	i2c_pins[I2C_BSC0] = board_cfg & SHARED_HW_CFG_E3_I2C_MUX0_MASK;
	i2c_pins[I2C_BSC1] = (board_cfg & SHARED_HW_CFG_E3_I2C_MUX1_MASK) >>
			SHARED_HW_CFG_E3_I2C_MUX1_SHIFT;

	/* Read I2C output value */
	sfp_ctrl = REG_RD(bp, params->shmem_base +
			  offsetof(struct shmem_region,
				 dev_info.port_hw_config[port].e3_cmn_pin_cfg));
	i2c_val[I2C_BSC0] = (sfp_ctrl & PORT_HW_CFG_E3_I2C_MUX0_MASK) > 0;
	i2c_val[I2C_BSC1] = (sfp_ctrl & PORT_HW_CFG_E3_I2C_MUX1_MASK) > 0;
	DP(NETIF_MSG_LINK, "Setting BSC switch\n");
	for (idx = 0; idx < I2C_SWITCH_WIDTH; idx++)
		bnx2x_set_cfg_pin(bp, i2c_pins[idx], i2c_val[idx]);
}

static int bnx2x_bsc_read(struct link_params *params,
			  struct bnx2x *bp,
			  u8 sl_devid,
			  u16 sl_addr,
			  u8 lc_addr,
			  u8 xfer_cnt,
			  u32 *data_array)
{
	u32 val, i;
	int rc = 0;

	if (xfer_cnt > 16) {
		DP(NETIF_MSG_LINK, "invalid xfer_cnt %d. Max is 16 bytes\n",
					xfer_cnt);
		return -EINVAL;
	}
	bnx2x_bsc_module_sel(params);

	xfer_cnt = 16 - lc_addr;

	/* Enable the engine */
	val = REG_RD(bp, MCP_REG_MCPR_IMC_COMMAND);
	val |= MCPR_IMC_COMMAND_ENABLE;
	REG_WR(bp, MCP_REG_MCPR_IMC_COMMAND, val);

	/* Program slave device ID */
	val = (sl_devid << 16) | sl_addr;
	REG_WR(bp, MCP_REG_MCPR_IMC_SLAVE_CONTROL, val);

	/* Start xfer with 0 byte to update the address pointer ???*/
	val = (MCPR_IMC_COMMAND_ENABLE) |
	      (MCPR_IMC_COMMAND_WRITE_OP <<
		MCPR_IMC_COMMAND_OPERATION_BITSHIFT) |
		(lc_addr << MCPR_IMC_COMMAND_TRANSFER_ADDRESS_BITSHIFT) | (0);
	REG_WR(bp, MCP_REG_MCPR_IMC_COMMAND, val);

	/* Poll for completion */
	i = 0;
	val = REG_RD(bp, MCP_REG_MCPR_IMC_COMMAND);
	while (((val >> MCPR_IMC_COMMAND_IMC_STATUS_BITSHIFT) & 0x3) != 1) {
		udelay(10);
		val = REG_RD(bp, MCP_REG_MCPR_IMC_COMMAND);
		if (i++ > 1000) {
			DP(NETIF_MSG_LINK, "wr 0 byte timed out after %d try\n",
								i);
			rc = -EFAULT;
			break;
		}
	}
	if (rc == -EFAULT)
		return rc;

	/* Start xfer with read op */
	val = (MCPR_IMC_COMMAND_ENABLE) |
		(MCPR_IMC_COMMAND_READ_OP <<
		MCPR_IMC_COMMAND_OPERATION_BITSHIFT) |
		(lc_addr << MCPR_IMC_COMMAND_TRANSFER_ADDRESS_BITSHIFT) |
		  (xfer_cnt);
	REG_WR(bp, MCP_REG_MCPR_IMC_COMMAND, val);

	/* Poll for completion */
	i = 0;
	val = REG_RD(bp, MCP_REG_MCPR_IMC_COMMAND);
	while (((val >> MCPR_IMC_COMMAND_IMC_STATUS_BITSHIFT) & 0x3) != 1) {
		udelay(10);
		val = REG_RD(bp, MCP_REG_MCPR_IMC_COMMAND);
		if (i++ > 1000) {
			DP(NETIF_MSG_LINK, "rd op timed out after %d try\n", i);
			rc = -EFAULT;
			break;
		}
	}
	if (rc == -EFAULT)
		return rc;

	for (i = (lc_addr >> 2); i < 4; i++) {
		data_array[i] = REG_RD(bp, (MCP_REG_MCPR_IMC_DATAREG0 + i*4));
#ifdef __BIG_ENDIAN
		data_array[i] = ((data_array[i] & 0x000000ff) << 24) |
				((data_array[i] & 0x0000ff00) << 8) |
				((data_array[i] & 0x00ff0000) >> 8) |
				((data_array[i] & 0xff000000) >> 24);
#endif
	}
	return rc;
}

static void bnx2x_cl45_read_or_write(struct bnx2x *bp, struct bnx2x_phy *phy,
				     u8 devad, u16 reg, u16 or_val)
{
	u16 val;
	bnx2x_cl45_read(bp, phy, devad, reg, &val);
	bnx2x_cl45_write(bp, phy, devad, reg, val | or_val);
}

static void bnx2x_cl45_read_and_write(struct bnx2x *bp,
				      struct bnx2x_phy *phy,
				      u8 devad, u16 reg, u16 and_val)
{
	u16 val;
	bnx2x_cl45_read(bp, phy, devad, reg, &val);
	bnx2x_cl45_write(bp, phy, devad, reg, val & and_val);
}

int bnx2x_phy_read(struct link_params *params, u8 phy_addr,
		   u8 devad, u16 reg, u16 *ret_val)
{
	u8 phy_index;
	/* Probe for the phy according to the given phy_addr, and execute
	 * the read request on it
	 */
	for (phy_index = 0; phy_index < params->num_phys; phy_index++) {
		if (params->phy[phy_index].addr == phy_addr) {
			return bnx2x_cl45_read(params->bp,
					       &params->phy[phy_index], devad,
					       reg, ret_val);
		}
	}
	return -EINVAL;
}

int bnx2x_phy_write(struct link_params *params, u8 phy_addr,
		    u8 devad, u16 reg, u16 val)
{
	u8 phy_index;
	/* Probe for the phy according to the given phy_addr, and execute
	 * the write request on it
	 */
	for (phy_index = 0; phy_index < params->num_phys; phy_index++) {
		if (params->phy[phy_index].addr == phy_addr) {
			return bnx2x_cl45_write(params->bp,
						&params->phy[phy_index], devad,
						reg, val);
		}
	}
	return -EINVAL;
}
static u8 bnx2x_get_warpcore_lane(struct bnx2x_phy *phy,
				  struct link_params *params)
{
	u8 lane = 0;
	struct bnx2x *bp = params->bp;
	u32 path_swap, path_swap_ovr;
	u8 path, port;

	path = BP_PATH(bp);
	port = params->port;

	if (bnx2x_is_4_port_mode(bp)) {
		u32 port_swap, port_swap_ovr;

		/* Figure out path swap value */
		path_swap_ovr = REG_RD(bp, MISC_REG_FOUR_PORT_PATH_SWAP_OVWR);
		if (path_swap_ovr & 0x1)
			path_swap = (path_swap_ovr & 0x2);
		else
			path_swap = REG_RD(bp, MISC_REG_FOUR_PORT_PATH_SWAP);

		if (path_swap)
			path = path ^ 1;

		/* Figure out port swap value */
		port_swap_ovr = REG_RD(bp, MISC_REG_FOUR_PORT_PORT_SWAP_OVWR);
		if (port_swap_ovr & 0x1)
			port_swap = (port_swap_ovr & 0x2);
		else
			port_swap = REG_RD(bp, MISC_REG_FOUR_PORT_PORT_SWAP);

		if (port_swap)
			port = port ^ 1;

		lane = (port<<1) + path;
	} else { /* Two port mode - no port swap */

		/* Figure out path swap value */
		path_swap_ovr =
			REG_RD(bp, MISC_REG_TWO_PORT_PATH_SWAP_OVWR);
		if (path_swap_ovr & 0x1) {
			path_swap = (path_swap_ovr & 0x2);
		} else {
			path_swap =
				REG_RD(bp, MISC_REG_TWO_PORT_PATH_SWAP);
		}
		if (path_swap)
			path = path ^ 1;

		lane = path << 1 ;
	}
	return lane;
}

static void bnx2x_set_aer_mmd(struct link_params *params,
			      struct bnx2x_phy *phy)
{
	u32 ser_lane;
	u16 offset, aer_val;
	struct bnx2x *bp = params->bp;
	ser_lane = ((params->lane_config &
		     PORT_HW_CFG_LANE_SWAP_CFG_MASTER_MASK) >>
		     PORT_HW_CFG_LANE_SWAP_CFG_MASTER_SHIFT);

	offset = (phy->type == PORT_HW_CFG_XGXS_EXT_PHY_TYPE_DIRECT) ?
		(phy->addr + ser_lane) : 0;

	if (USES_WARPCORE(bp)) {
		aer_val = bnx2x_get_warpcore_lane(phy, params);
		/* In Dual-lane mode, two lanes are joined together,
		 * so in order to configure them, the AER broadcast method is
		 * used here.
		 * 0x200 is the broadcast address for lanes 0,1
		 * 0x201 is the broadcast address for lanes 2,3
		 */
		if (phy->flags & FLAGS_WC_DUAL_MODE)
			aer_val = (aer_val >> 1) | 0x200;
	} else if (CHIP_IS_E2(bp))
		aer_val = 0x3800 + offset - 1;
	else
		aer_val = 0x3800 + offset;

	CL22_WR_OVER_CL45(bp, phy, MDIO_REG_BANK_AER_BLOCK,
			  MDIO_AER_BLOCK_AER_REG, aer_val);

}

/******************************************************************/
/*			Internal phy section			  */
/******************************************************************/

static void bnx2x_set_serdes_access(struct bnx2x *bp, u8 port)
{
	u32 emac_base = (port) ? GRCBASE_EMAC1 : GRCBASE_EMAC0;

	/* Set Clause 22 */
	REG_WR(bp, NIG_REG_SERDES0_CTRL_MD_ST + port*0x10, 1);
	REG_WR(bp, emac_base + EMAC_REG_EMAC_MDIO_COMM, 0x245f8000);
	udelay(500);
	REG_WR(bp, emac_base + EMAC_REG_EMAC_MDIO_COMM, 0x245d000f);
	udelay(500);
	 /* Set Clause 45 */
	REG_WR(bp, NIG_REG_SERDES0_CTRL_MD_ST + port*0x10, 0);
}

static void bnx2x_serdes_deassert(struct bnx2x *bp, u8 port)
{
	u32 val;

	DP(NETIF_MSG_LINK, "bnx2x_serdes_deassert\n");

	val = SERDES_RESET_BITS << (port*16);

	/* Reset and unreset the SerDes/XGXS */
	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_3_CLEAR, val);
	udelay(500);
	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_3_SET, val);

	bnx2x_set_serdes_access(bp, port);

	REG_WR(bp, NIG_REG_SERDES0_CTRL_MD_DEVAD + port*0x10,
	       DEFAULT_PHY_DEV_ADDR);
}

static void bnx2x_xgxs_specific_func(struct bnx2x_phy *phy,
				     struct link_params *params,
				     u32 action)
{
	struct bnx2x *bp = params->bp;
	switch (action) {
	case PHY_INIT:
		/* Set correct devad */
		REG_WR(bp, NIG_REG_XGXS0_CTRL_MD_ST + params->port*0x18, 0);
		REG_WR(bp, NIG_REG_XGXS0_CTRL_MD_DEVAD + params->port*0x18,
		       phy->def_md_devad);
		break;
	}
}

static void bnx2x_xgxs_deassert(struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	u8 port;
	u32 val;
	DP(NETIF_MSG_LINK, "bnx2x_xgxs_deassert\n");
	port = params->port;

	val = XGXS_RESET_BITS << (port*16);

	/* Reset and unreset the SerDes/XGXS */
	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_3_CLEAR, val);
	udelay(500);
	REG_WR(bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_3_SET, val);
	bnx2x_xgxs_specific_func(&params->phy[INT_PHY], params,
				 PHY_INIT);
}

static void bnx2x_calc_ieee_aneg_adv(struct bnx2x_phy *phy,
				     struct link_params *params, u16 *ieee_fc)
{
	struct bnx2x *bp = params->bp;
	*ieee_fc = MDIO_COMBO_IEEE0_AUTO_NEG_ADV_FULL_DUPLEX;
	/* Resolve pause mode and advertisement Please refer to Table
	 * 28B-3 of the 802.3ab-1999 spec
	 */

	switch (phy->req_flow_ctrl) {
	case BNX2X_FLOW_CTRL_AUTO:
		switch (params->req_fc_auto_adv) {
		case BNX2X_FLOW_CTRL_BOTH:
		case BNX2X_FLOW_CTRL_RX:
			*ieee_fc |= MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH;
			break;
		case BNX2X_FLOW_CTRL_TX:
			*ieee_fc |=
				MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_ASYMMETRIC;
			break;
		default:
			break;
		}
		break;
	case BNX2X_FLOW_CTRL_TX:
		*ieee_fc |= MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_ASYMMETRIC;
		break;

	case BNX2X_FLOW_CTRL_RX:
	case BNX2X_FLOW_CTRL_BOTH:
		*ieee_fc |= MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH;
		break;

	case BNX2X_FLOW_CTRL_NONE:
	default:
		*ieee_fc |= MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_NONE;
		break;
	}
	DP(NETIF_MSG_LINK, "ieee_fc = 0x%x\n", *ieee_fc);
}

static void set_phy_vars(struct link_params *params,
			 struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u8 actual_phy_idx, phy_index, link_cfg_idx;
	u8 phy_config_swapped = params->multi_phy_config &
			PORT_HW_CFG_PHY_SWAPPED_ENABLED;
	for (phy_index = INT_PHY; phy_index < params->num_phys;
	      phy_index++) {
		link_cfg_idx = LINK_CONFIG_IDX(phy_index);
		actual_phy_idx = phy_index;
		if (phy_config_swapped) {
			if (phy_index == EXT_PHY1)
				actual_phy_idx = EXT_PHY2;
			else if (phy_index == EXT_PHY2)
				actual_phy_idx = EXT_PHY1;
		}
		params->phy[actual_phy_idx].req_flow_ctrl =
			params->req_flow_ctrl[link_cfg_idx];

		params->phy[actual_phy_idx].req_line_speed =
			params->req_line_speed[link_cfg_idx];

		params->phy[actual_phy_idx].speed_cap_mask =
			params->speed_cap_mask[link_cfg_idx];

		params->phy[actual_phy_idx].req_duplex =
			params->req_duplex[link_cfg_idx];

		if (params->req_line_speed[link_cfg_idx] ==
		    SPEED_AUTO_NEG)
			vars->link_status |= LINK_STATUS_AUTO_NEGOTIATE_ENABLED;

		DP(NETIF_MSG_LINK, "req_flow_ctrl %x, req_line_speed %x,"
			   " speed_cap_mask %x\n",
			   params->phy[actual_phy_idx].req_flow_ctrl,
			   params->phy[actual_phy_idx].req_line_speed,
			   params->phy[actual_phy_idx].speed_cap_mask);
	}
}

static void bnx2x_ext_phy_set_pause(struct link_params *params,
				    struct bnx2x_phy *phy,
				    struct link_vars *vars)
{
	u16 val;
	struct bnx2x *bp = params->bp;
	/* Read modify write pause advertizing */
	bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_ADV_PAUSE, &val);

	val &= ~MDIO_AN_REG_ADV_PAUSE_BOTH;

	/* Please refer to Table 28B-3 of 802.3ab-1999 spec. */
	bnx2x_calc_ieee_aneg_adv(phy, params, &vars->ieee_fc);
	if ((vars->ieee_fc &
	    MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_ASYMMETRIC) ==
	    MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_ASYMMETRIC) {
		val |= MDIO_AN_REG_ADV_PAUSE_ASYMMETRIC;
	}
	if ((vars->ieee_fc &
	    MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH) ==
	    MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH) {
		val |= MDIO_AN_REG_ADV_PAUSE_PAUSE;
	}
	DP(NETIF_MSG_LINK, "Ext phy AN advertize 0x%x\n", val);
	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_ADV_PAUSE, val);
}

static void bnx2x_pause_resolve(struct bnx2x_phy *phy,
				struct link_params *params,
				struct link_vars *vars,
				u32 pause_result)
{
	struct bnx2x *bp = params->bp;
						/*  LD	    LP	 */
	switch (pause_result) {			/* ASYM P ASYM P */
	case 0xb:				/*   1  0   1  1 */
		DP(NETIF_MSG_LINK, "Flow Control: TX only\n");
		vars->flow_ctrl = BNX2X_FLOW_CTRL_TX;
		break;

	case 0xe:				/*   1  1   1  0 */
		DP(NETIF_MSG_LINK, "Flow Control: RX only\n");
		vars->flow_ctrl = BNX2X_FLOW_CTRL_RX;
		break;

	case 0x5:				/*   0  1   0  1 */
	case 0x7:				/*   0  1   1  1 */
	case 0xd:				/*   1  1   0  1 */
	case 0xf:				/*   1  1   1  1 */
		/* If the user selected to advertise RX ONLY,
		 * although we advertised both, need to enable
		 * RX only.
		 */
		if (params->req_fc_auto_adv == BNX2X_FLOW_CTRL_BOTH) {
			DP(NETIF_MSG_LINK, "Flow Control: RX & TX\n");
			vars->flow_ctrl = BNX2X_FLOW_CTRL_BOTH;
		} else {
			DP(NETIF_MSG_LINK, "Flow Control: RX only\n");
			vars->flow_ctrl = BNX2X_FLOW_CTRL_RX;
		}
		break;

	default:
		DP(NETIF_MSG_LINK, "Flow Control: None\n");
		vars->flow_ctrl = BNX2X_FLOW_CTRL_NONE;
		break;
	}
	if (pause_result & (1<<0))
		vars->link_status |= LINK_STATUS_LINK_PARTNER_SYMMETRIC_PAUSE;
	if (pause_result & (1<<1))
		vars->link_status |= LINK_STATUS_LINK_PARTNER_ASYMMETRIC_PAUSE;

}

static void bnx2x_ext_phy_update_adv_fc(struct bnx2x_phy *phy,
					struct link_params *params,
					struct link_vars *vars)
{
	u16 ld_pause;		/* local */
	u16 lp_pause;		/* link partner */
	u16 pause_result;
	struct bnx2x *bp = params->bp;
	if (phy->type == PORT_HW_CFG_XGXS_EXT_PHY_TYPE_BCM54618SE) {
		bnx2x_cl22_read(bp, phy, 0x4, &ld_pause);
		bnx2x_cl22_read(bp, phy, 0x5, &lp_pause);
	} else if (CHIP_IS_E3(bp) &&
		SINGLE_MEDIA_DIRECT(params)) {
		u8 lane = bnx2x_get_warpcore_lane(phy, params);
		u16 gp_status, gp_mask;
		bnx2x_cl45_read(bp, phy,
				MDIO_AN_DEVAD, MDIO_WC_REG_GP2_STATUS_GP_2_4,
				&gp_status);
		gp_mask = (MDIO_WC_REG_GP2_STATUS_GP_2_4_CL73_AN_CMPL |
			   MDIO_WC_REG_GP2_STATUS_GP_2_4_CL37_LP_AN_CAP) <<
			lane;
		if ((gp_status & gp_mask) == gp_mask) {
			bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD,
					MDIO_AN_REG_ADV_PAUSE, &ld_pause);
			bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD,
					MDIO_AN_REG_LP_AUTO_NEG, &lp_pause);
		} else {
			bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD,
					MDIO_AN_REG_CL37_FC_LD, &ld_pause);
			bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD,
					MDIO_AN_REG_CL37_FC_LP, &lp_pause);
			ld_pause = ((ld_pause &
				     MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH)
				    << 3);
			lp_pause = ((lp_pause &
				     MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH)
				    << 3);
		}
	} else {
		bnx2x_cl45_read(bp, phy,
				MDIO_AN_DEVAD,
				MDIO_AN_REG_ADV_PAUSE, &ld_pause);
		bnx2x_cl45_read(bp, phy,
				MDIO_AN_DEVAD,
				MDIO_AN_REG_LP_AUTO_NEG, &lp_pause);
	}
	pause_result = (ld_pause &
			MDIO_AN_REG_ADV_PAUSE_MASK) >> 8;
	pause_result |= (lp_pause &
			 MDIO_AN_REG_ADV_PAUSE_MASK) >> 10;
	DP(NETIF_MSG_LINK, "Ext PHY pause result 0x%x\n", pause_result);
	bnx2x_pause_resolve(phy, params, vars, pause_result);

}

static u8 bnx2x_ext_phy_resolve_fc(struct bnx2x_phy *phy,
				   struct link_params *params,
				   struct link_vars *vars)
{
	u8 ret = 0;
	vars->flow_ctrl = BNX2X_FLOW_CTRL_NONE;
	if (phy->req_flow_ctrl != BNX2X_FLOW_CTRL_AUTO) {
		/* Update the advertised flow-controled of LD/LP in AN */
		if (phy->req_line_speed == SPEED_AUTO_NEG)
			bnx2x_ext_phy_update_adv_fc(phy, params, vars);
		/* But set the flow-control result as the requested one */
		vars->flow_ctrl = phy->req_flow_ctrl;
	} else if (phy->req_line_speed != SPEED_AUTO_NEG)
		vars->flow_ctrl = params->req_fc_auto_adv;
	else if (vars->link_status & LINK_STATUS_AUTO_NEGOTIATE_COMPLETE) {
		ret = 1;
		bnx2x_ext_phy_update_adv_fc(phy, params, vars);
	}
	return ret;
}
/******************************************************************/
/*			Warpcore section			  */
/******************************************************************/
/* The init_internal_warpcore should mirror the xgxs,
 * i.e. reset the lane (if needed), set aer for the
 * init configuration, and set/clear SGMII flag. Internal
 * phy init is done purely in phy_init stage.
 */
#define WC_TX_DRIVER(post2, idriver, ipre, ifir) \
	((post2 << MDIO_WC_REG_TX0_TX_DRIVER_POST2_COEFF_OFFSET) | \
	 (idriver << MDIO_WC_REG_TX0_TX_DRIVER_IDRIVER_OFFSET) | \
	 (ipre << MDIO_WC_REG_TX0_TX_DRIVER_IPRE_DRIVER_OFFSET) | \
	 (ifir << MDIO_WC_REG_TX0_TX_DRIVER_IFIR_OFFSET))

#define WC_TX_FIR(post, main, pre) \
	((post << MDIO_WC_REG_TX_FIR_TAP_POST_TAP_OFFSET) | \
	 (main << MDIO_WC_REG_TX_FIR_TAP_MAIN_TAP_OFFSET) | \
	 (pre << MDIO_WC_REG_TX_FIR_TAP_PRE_TAP_OFFSET))

static void bnx2x_warpcore_enable_AN_KR2(struct bnx2x_phy *phy,
					 struct link_params *params,
					 struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u16 i;
	static struct bnx2x_reg_set reg_set[] = {
		/* Step 1 - Program the TX/RX alignment markers */
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_TX_CTRL5, 0xa157},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_TX_CTRL7, 0xcbe2},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_TX_CTRL6, 0x7537},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_TX_CTRL9, 0xa157},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_RX_CTRL11, 0xcbe2},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_RX_CTRL10, 0x7537},
		/* Step 2 - Configure the NP registers */
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL73_USERB0_CTRL, 0x000a},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL73_BAM_CTRL1, 0x6400},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL73_BAM_CTRL3, 0x0620},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL73_BAM_CODE_FIELD, 0x0157},
		{MDIO_WC_DEVAD, MDIO_WC_REG_ETA_CL73_OUI1, 0x6464},
		{MDIO_WC_DEVAD, MDIO_WC_REG_ETA_CL73_OUI2, 0x3150},
		{MDIO_WC_DEVAD, MDIO_WC_REG_ETA_CL73_OUI3, 0x3150},
		{MDIO_WC_DEVAD, MDIO_WC_REG_ETA_CL73_LD_BAM_CODE, 0x0157},
		{MDIO_WC_DEVAD, MDIO_WC_REG_ETA_CL73_LD_UD_CODE, 0x0620}
	};
	DP(NETIF_MSG_LINK, "Enabling 20G-KR2\n");

	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_CL49_USERB0_CTRL, (3<<6));

	for (i = 0; i < ARRAY_SIZE(reg_set); i++)
		bnx2x_cl45_write(bp, phy, reg_set[i].devad, reg_set[i].reg,
				 reg_set[i].val);

	/* Start KR2 work-around timer which handles BCM8073 link-parner */
	params->link_attr_sync |= LINK_ATTR_SYNC_KR2_ENABLE;
	bnx2x_update_link_attr(params, params->link_attr_sync);
}

static void bnx2x_disable_kr2(struct link_params *params,
			      struct link_vars *vars,
			      struct bnx2x_phy *phy)
{
	struct bnx2x *bp = params->bp;
	int i;
	static struct bnx2x_reg_set reg_set[] = {
		/* Step 1 - Program the TX/RX alignment markers */
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_TX_CTRL5, 0x7690},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_TX_CTRL7, 0xe647},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_TX_CTRL6, 0xc4f0},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_TX_CTRL9, 0x7690},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_RX_CTRL11, 0xe647},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL82_USERB1_RX_CTRL10, 0xc4f0},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL73_USERB0_CTRL, 0x000c},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL73_BAM_CTRL1, 0x6000},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL73_BAM_CTRL3, 0x0000},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL73_BAM_CODE_FIELD, 0x0002},
		{MDIO_WC_DEVAD, MDIO_WC_REG_ETA_CL73_OUI1, 0x0000},
		{MDIO_WC_DEVAD, MDIO_WC_REG_ETA_CL73_OUI2, 0x0af7},
		{MDIO_WC_DEVAD, MDIO_WC_REG_ETA_CL73_OUI3, 0x0af7},
		{MDIO_WC_DEVAD, MDIO_WC_REG_ETA_CL73_LD_BAM_CODE, 0x0002},
		{MDIO_WC_DEVAD, MDIO_WC_REG_ETA_CL73_LD_UD_CODE, 0x0000}
	};
	DP(NETIF_MSG_LINK, "Disabling 20G-KR2\n");

	for (i = 0; i < ARRAY_SIZE(reg_set); i++)
		bnx2x_cl45_write(bp, phy, reg_set[i].devad, reg_set[i].reg,
				 reg_set[i].val);
	params->link_attr_sync &= ~LINK_ATTR_SYNC_KR2_ENABLE;
	bnx2x_update_link_attr(params, params->link_attr_sync);

	vars->check_kr2_recovery_cnt = CHECK_KR2_RECOVERY_CNT;
}

static void bnx2x_warpcore_set_lpi_passthrough(struct bnx2x_phy *phy,
					       struct link_params *params)
{
	struct bnx2x *bp = params->bp;

	DP(NETIF_MSG_LINK, "Configure WC for LPI pass through\n");
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_EEE_COMBO_CONTROL0, 0x7c);
	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_DIGITAL4_MISC5, 0xc000);
}

static void bnx2x_warpcore_restart_AN_KR(struct bnx2x_phy *phy,
					 struct link_params *params)
{
	/* Restart autoneg on the leading lane only */
	struct bnx2x *bp = params->bp;
	u16 lane = bnx2x_get_warpcore_lane(phy, params);
	CL22_WR_OVER_CL45(bp, phy, MDIO_REG_BANK_AER_BLOCK,
			  MDIO_AER_BLOCK_AER_REG, lane);
	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD,
			 MDIO_WC_REG_IEEE0BLK_MIICNTL, 0x1200);

	/* Restore AER */
	bnx2x_set_aer_mmd(params, phy);
}

static void bnx2x_warpcore_enable_AN_KR(struct bnx2x_phy *phy,
					struct link_params *params,
					struct link_vars *vars) {
	u16 lane, i, cl72_ctrl, an_adv = 0, val;
	u32 wc_lane_config;
	struct bnx2x *bp = params->bp;
	static struct bnx2x_reg_set reg_set[] = {
		{MDIO_WC_DEVAD, MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X2, 0x7},
		{MDIO_PMA_DEVAD, MDIO_WC_REG_IEEE0BLK_AUTONEGNP, 0x0},
		{MDIO_WC_DEVAD, MDIO_WC_REG_RX66_CONTROL, 0x7415},
		{MDIO_WC_DEVAD, MDIO_WC_REG_SERDESDIGITAL_MISC2, 0x6190},
		/* Disable Autoneg: re-enable it after adv is done. */
		{MDIO_AN_DEVAD, MDIO_WC_REG_IEEE0BLK_MIICNTL, 0},
		{MDIO_PMA_DEVAD, MDIO_WC_REG_PMD_KR_CONTROL, 0x2},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL72_USERB0_CL72_TX_FIR_TAP, 0},
	};
	DP(NETIF_MSG_LINK, "Enable Auto Negotiation for KR\n");
	/* Set to default registers that may be overriden by 10G force */
	for (i = 0; i < ARRAY_SIZE(reg_set); i++)
		bnx2x_cl45_write(bp, phy, reg_set[i].devad, reg_set[i].reg,
				 reg_set[i].val);

	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_CL72_USERB0_CL72_MISC1_CONTROL, &cl72_ctrl);
	cl72_ctrl &= 0x08ff;
	cl72_ctrl |= 0x3800;
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_CL72_USERB0_CL72_MISC1_CONTROL, cl72_ctrl);

	/* Check adding advertisement for 1G KX */
	if (((vars->line_speed == SPEED_AUTO_NEG) &&
	     (phy->speed_cap_mask & PORT_HW_CFG_SPEED_CAPABILITY_D0_1G)) ||
	    (vars->line_speed == SPEED_1000)) {
		u16 addr = MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X2;
		an_adv |= (1<<5);

		/* Enable CL37 1G Parallel Detect */
		bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD, addr, 0x1);
		DP(NETIF_MSG_LINK, "Advertize 1G\n");
	}
	if (((vars->line_speed == SPEED_AUTO_NEG) &&
	     (phy->speed_cap_mask & PORT_HW_CFG_SPEED_CAPABILITY_D0_10G)) ||
	    (vars->line_speed ==  SPEED_10000)) {
		/* Check adding advertisement for 10G KR */
		an_adv |= (1<<7);
		/* Enable 10G Parallel Detect */
		CL22_WR_OVER_CL45(bp, phy, MDIO_REG_BANK_AER_BLOCK,
				  MDIO_AER_BLOCK_AER_REG, 0);

		bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD,
				 MDIO_WC_REG_PAR_DET_10G_CTRL, 1);
		bnx2x_set_aer_mmd(params, phy);
		DP(NETIF_MSG_LINK, "Advertize 10G\n");
	}

	/* Set Transmit PMD settings */
	lane = bnx2x_get_warpcore_lane(phy, params);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_TX0_TX_DRIVER + 0x10*lane,
			 WC_TX_DRIVER(0x02, 0x06, 0x09, 0));
	/* Configure the next lane if dual mode */
	if (phy->flags & FLAGS_WC_DUAL_MODE)
		bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_TX0_TX_DRIVER + 0x10*(lane+1),
				 WC_TX_DRIVER(0x02, 0x06, 0x09, 0));
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_CL72_USERB0_CL72_OS_DEF_CTRL,
			 0x03f0);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_CL72_USERB0_CL72_2P5_DEF_CTRL,
			 0x03f0);

	/* Advertised speeds */
	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD,
			 MDIO_WC_REG_AN_IEEE1BLK_AN_ADVERTISEMENT1, an_adv);

	/* Advertised and set FEC (Forward Error Correction) */
	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD,
			 MDIO_WC_REG_AN_IEEE1BLK_AN_ADVERTISEMENT2,
			 (MDIO_WC_REG_AN_IEEE1BLK_AN_ADV2_FEC_ABILITY |
			  MDIO_WC_REG_AN_IEEE1BLK_AN_ADV2_FEC_REQ));

	/* Enable CL37 BAM */
	if (REG_RD(bp, params->shmem_base +
		   offsetof(struct shmem_region, dev_info.
			    port_hw_config[params->port].default_cfg)) &
	    PORT_HW_CFG_ENABLE_BAM_ON_KR_ENABLED) {
		bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
					 MDIO_WC_REG_DIGITAL6_MP5_NEXTPAGECTRL,
					 1);
		DP(NETIF_MSG_LINK, "Enable CL37 BAM on KR\n");
	}

	/* Advertise pause */
	bnx2x_ext_phy_set_pause(params, phy, vars);
	vars->rx_tx_asic_rst = MAX_KR_LINK_RETRY;
	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_DIGITAL5_MISC7, 0x100);

	/* Over 1G - AN local device user page 1 */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_DIGITAL3_UP1, 0x1f);

	if (((phy->req_line_speed == SPEED_AUTO_NEG) &&
	     (phy->speed_cap_mask & PORT_HW_CFG_SPEED_CAPABILITY_D0_20G)) ||
	    (phy->req_line_speed == SPEED_20000)) {

		CL22_WR_OVER_CL45(bp, phy, MDIO_REG_BANK_AER_BLOCK,
				  MDIO_AER_BLOCK_AER_REG, lane);

		bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
					 MDIO_WC_REG_RX1_PCI_CTRL + (0x10*lane),
					 (1<<11));

		bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_XGXS_X2_CONTROL3, 0x7);
		bnx2x_set_aer_mmd(params, phy);

		bnx2x_warpcore_enable_AN_KR2(phy, params, vars);
	} else {
		/* Enable Auto-Detect to support 1G over CL37 as well */
		bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X1, 0x10);
		wc_lane_config = REG_RD(bp, params->shmem_base +
					offsetof(struct shmem_region, dev_info.
					shared_hw_config.wc_lane_config));
		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				MDIO_WC_REG_RX0_PCI_CTRL + (lane << 4), &val);
		/* Force cl48 sync_status LOW to avoid getting stuck in CL73
		 * parallel-detect loop when CL73 and CL37 are enabled.
		 */
		val |= 1 << 11;

		/* Restore Polarity settings in case it was run over by
		 * previous link owner
		 */
		if (wc_lane_config &
		    (SHARED_HW_CFG_RX_LANE0_POL_FLIP_ENABLED << lane))
			val |= 3 << 2;
		else
			val &= ~(3 << 2);
		bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_RX0_PCI_CTRL + (lane << 4),
				 val);

		bnx2x_disable_kr2(params, vars, phy);
	}

	/* Enable Autoneg: only on the main lane */
	bnx2x_warpcore_restart_AN_KR(phy, params);
}

static void bnx2x_warpcore_set_10G_KR(struct bnx2x_phy *phy,
				      struct link_params *params,
				      struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u16 val16, i, lane;
	static struct bnx2x_reg_set reg_set[] = {
		/* Disable Autoneg */
		{MDIO_WC_DEVAD, MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X2, 0x7},
		{MDIO_WC_DEVAD, MDIO_WC_REG_CL72_USERB0_CL72_MISC1_CONTROL,
			0x3f00},
		{MDIO_AN_DEVAD, MDIO_WC_REG_AN_IEEE1BLK_AN_ADVERTISEMENT1, 0},
		{MDIO_AN_DEVAD, MDIO_WC_REG_IEEE0BLK_MIICNTL, 0x0},
		{MDIO_WC_DEVAD, MDIO_WC_REG_DIGITAL3_UP1, 0x1},
		{MDIO_WC_DEVAD, MDIO_WC_REG_DIGITAL5_MISC7, 0xa},
		/* Leave cl72 training enable, needed for KR */
		{MDIO_PMA_DEVAD, MDIO_WC_REG_PMD_KR_CONTROL, 0x2}
	};

	for (i = 0; i < ARRAY_SIZE(reg_set); i++)
		bnx2x_cl45_write(bp, phy, reg_set[i].devad, reg_set[i].reg,
				 reg_set[i].val);

	lane = bnx2x_get_warpcore_lane(phy, params);
	/* Global registers */
	CL22_WR_OVER_CL45(bp, phy, MDIO_REG_BANK_AER_BLOCK,
			  MDIO_AER_BLOCK_AER_REG, 0);
	/* Disable CL36 PCS Tx */
	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_XGXSBLK1_LANECTRL0, &val16);
	val16 &= ~(0x0011 << lane);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_XGXSBLK1_LANECTRL0, val16);

	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_XGXSBLK1_LANECTRL1, &val16);
	val16 |= (0x0303 << (lane << 1));
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_XGXSBLK1_LANECTRL1, val16);
	/* Restore AER */
	bnx2x_set_aer_mmd(params, phy);
	/* Set speed via PMA/PMD register */
	bnx2x_cl45_write(bp, phy, MDIO_PMA_DEVAD,
			 MDIO_WC_REG_IEEE0BLK_MIICNTL, 0x2040);

	bnx2x_cl45_write(bp, phy, MDIO_PMA_DEVAD,
			 MDIO_WC_REG_IEEE0BLK_AUTONEGNP, 0xB);

	/* Enable encoded forced speed */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_SERDESDIGITAL_MISC2, 0x30);

	/* Turn TX scramble payload only the 64/66 scrambler */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_TX66_CONTROL, 0x9);

	/* Turn RX scramble payload only the 64/66 scrambler */
	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_RX66_CONTROL, 0xF9);

	/* Set and clear loopback to cause a reset to 64/66 decoder */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_IEEE0BLK_MIICNTL, 0x4000);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_IEEE0BLK_MIICNTL, 0x0);

}

static void bnx2x_warpcore_set_10G_XFI(struct bnx2x_phy *phy,
				       struct link_params *params,
				       u8 is_xfi)
{
	struct bnx2x *bp = params->bp;
	u16 misc1_val, tap_val, tx_driver_val, lane, val;
	u32 cfg_tap_val, tx_drv_brdct, tx_equal;
	u32 ifir_val, ipost2_val, ipre_driver_val;

	/* Hold rxSeqStart */
	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_DSC2B0_DSC_MISC_CTRL0, 0x8000);

	/* Hold tx_fifo_reset */
	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X3, 0x1);

	/* Disable CL73 AN */
	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_CTRL, 0);

	/* Disable 100FX Enable and Auto-Detect */
	bnx2x_cl45_read_and_write(bp, phy, MDIO_WC_DEVAD,
				  MDIO_WC_REG_FX100_CTRL1, 0xFFFA);

	/* Disable 100FX Idle detect */
	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_FX100_CTRL3, 0x0080);

	/* Set Block address to Remote PHY & Clear forced_speed[5] */
	bnx2x_cl45_read_and_write(bp, phy, MDIO_WC_DEVAD,
				  MDIO_WC_REG_DIGITAL4_MISC3, 0xFF7F);

	/* Turn off auto-detect & fiber mode */
	bnx2x_cl45_read_and_write(bp, phy, MDIO_WC_DEVAD,
				  MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X1,
				  0xFFEE);

	/* Set filter_force_link, disable_false_link and parallel_detect */
	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X2, &val);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X2,
			 ((val | 0x0006) & 0xFFFE));

	/* Set XFI / SFI */
	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_SERDESDIGITAL_MISC1, &misc1_val);

	misc1_val &= ~(0x1f);

	if (is_xfi) {
		misc1_val |= 0x5;
		tap_val = WC_TX_FIR(0x08, 0x37, 0x00);
		tx_driver_val = WC_TX_DRIVER(0x00, 0x02, 0x03, 0);
	} else {
		cfg_tap_val = REG_RD(bp, params->shmem_base +
				     offsetof(struct shmem_region, dev_info.
					      port_hw_config[params->port].
					      sfi_tap_values));

		tx_equal = cfg_tap_val & PORT_HW_CFG_TX_EQUALIZATION_MASK;

		misc1_val |= 0x9;

		/* TAP values are controlled by nvram, if value there isn't 0 */
		if (tx_equal)
			tap_val = (u16)tx_equal;
		else
			tap_val = WC_TX_FIR(0x0f, 0x2b, 0x02);

		ifir_val = DEFAULT_TX_DRV_IFIR;
		ipost2_val = DEFAULT_TX_DRV_POST2;
		ipre_driver_val = DEFAULT_TX_DRV_IPRE_DRIVER;
		tx_drv_brdct = DEFAULT_TX_DRV_BRDCT;

		/* If any of the IFIR/IPRE_DRIVER/POST@ is set, apply all
		 * configuration.
		 */
		if (cfg_tap_val & (PORT_HW_CFG_TX_DRV_IFIR_MASK |
				   PORT_HW_CFG_TX_DRV_IPREDRIVER_MASK |
				   PORT_HW_CFG_TX_DRV_POST2_MASK)) {
			ifir_val = (cfg_tap_val &
				    PORT_HW_CFG_TX_DRV_IFIR_MASK) >>
				PORT_HW_CFG_TX_DRV_IFIR_SHIFT;
			ipre_driver_val = (cfg_tap_val &
					   PORT_HW_CFG_TX_DRV_IPREDRIVER_MASK)
			>> PORT_HW_CFG_TX_DRV_IPREDRIVER_SHIFT;
			ipost2_val = (cfg_tap_val &
				      PORT_HW_CFG_TX_DRV_POST2_MASK) >>
				PORT_HW_CFG_TX_DRV_POST2_SHIFT;
		}

		if (cfg_tap_val & PORT_HW_CFG_TX_DRV_BROADCAST_MASK) {
			tx_drv_brdct = (cfg_tap_val &
					PORT_HW_CFG_TX_DRV_BROADCAST_MASK) >>
				PORT_HW_CFG_TX_DRV_BROADCAST_SHIFT;
		}

		tx_driver_val = WC_TX_DRIVER(ipost2_val, tx_drv_brdct,
					     ipre_driver_val, ifir_val);
	}
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_SERDESDIGITAL_MISC1, misc1_val);

	/* Set Transmit PMD settings */
	lane = bnx2x_get_warpcore_lane(phy, params);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_TX_FIR_TAP,
			 tap_val | MDIO_WC_REG_TX_FIR_TAP_ENABLE);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_TX0_TX_DRIVER + 0x10*lane,
			 tx_driver_val);

	/* Enable fiber mode, enable and invert sig_det */
	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X1, 0xd);

	/* Set Block address to Remote PHY & Set forced_speed[5], 40bit mode */
	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_DIGITAL4_MISC3, 0x8080);

	bnx2x_warpcore_set_lpi_passthrough(phy, params);

	/* 10G XFI Full Duplex */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_IEEE0BLK_MIICNTL, 0x100);

	/* Release tx_fifo_reset */
	bnx2x_cl45_read_and_write(bp, phy, MDIO_WC_DEVAD,
				  MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X3,
				  0xFFFE);
	/* Release rxSeqStart */
	bnx2x_cl45_read_and_write(bp, phy, MDIO_WC_DEVAD,
				  MDIO_WC_REG_DSC2B0_DSC_MISC_CTRL0, 0x7FFF);
}

static void bnx2x_warpcore_set_20G_force_KR2(struct bnx2x_phy *phy,
					     struct link_params *params)
{
	u16 val;
	struct bnx2x *bp = params->bp;
	/* Set global registers, so set AER lane to 0 */
	CL22_WR_OVER_CL45(bp, phy, MDIO_REG_BANK_AER_BLOCK,
			  MDIO_AER_BLOCK_AER_REG, 0);

	/* Disable sequencer */
	bnx2x_cl45_read_and_write(bp, phy, MDIO_WC_DEVAD,
				  MDIO_WC_REG_XGXSBLK0_XGXSCONTROL, ~(1<<13));

	bnx2x_set_aer_mmd(params, phy);

	bnx2x_cl45_read_and_write(bp, phy, MDIO_PMA_DEVAD,
				  MDIO_WC_REG_PMD_KR_CONTROL, ~(1<<1));
	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD,
			 MDIO_AN_REG_CTRL, 0);
	/* Turn off CL73 */
	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_CL73_USERB0_CTRL, &val);
	val &= ~(1<<5);
	val |= (1<<6);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_CL73_USERB0_CTRL, val);

	/* Set 20G KR2 force speed */
	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_SERDESDIGITAL_MISC1, 0x1f);

	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_DIGITAL4_MISC3, (1<<7));

	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_CL72_USERB0_CL72_MISC1_CONTROL, &val);
	val &= ~(3<<14);
	val |= (1<<15);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_CL72_USERB0_CL72_MISC1_CONTROL, val);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_CL72_USERB0_CL72_TX_FIR_TAP, 0x835A);

	/* Enable sequencer (over lane 0) */
	CL22_WR_OVER_CL45(bp, phy, MDIO_REG_BANK_AER_BLOCK,
			  MDIO_AER_BLOCK_AER_REG, 0);

	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_XGXSBLK0_XGXSCONTROL, (1<<13));

	bnx2x_set_aer_mmd(params, phy);
}

static void bnx2x_warpcore_set_20G_DXGXS(struct bnx2x *bp,
					 struct bnx2x_phy *phy,
					 u16 lane)
{
	/* Rx0 anaRxControl1G */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_RX0_ANARXCONTROL1G, 0x90);

	/* Rx2 anaRxControl1G */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_RX2_ANARXCONTROL1G, 0x90);

	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_RX66_SCW0, 0xE070);

	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_RX66_SCW1, 0xC0D0);

	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_RX66_SCW2, 0xA0B0);

	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_RX66_SCW3, 0x8090);

	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_RX66_SCW0_MASK, 0xF0F0);

	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_RX66_SCW1_MASK, 0xF0F0);

	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_RX66_SCW2_MASK, 0xF0F0);

	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_RX66_SCW3_MASK, 0xF0F0);

	/* Serdes Digital Misc1 */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_SERDESDIGITAL_MISC1, 0x6008);

	/* Serdes Digital4 Misc3 */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_DIGITAL4_MISC3, 0x8088);

	/* Set Transmit PMD settings */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_TX_FIR_TAP,
			 (WC_TX_FIR(0x12, 0x2d, 0x00) |
			  MDIO_WC_REG_TX_FIR_TAP_ENABLE));
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_TX0_TX_DRIVER + 0x10*lane,
			 WC_TX_DRIVER(0x02, 0x02, 0x02, 0));
}

static void bnx2x_warpcore_set_sgmii_speed(struct bnx2x_phy *phy,
					   struct link_params *params,
					   u8 fiber_mode,
					   u8 always_autoneg)
{
	struct bnx2x *bp = params->bp;
	u16 val16, digctrl_kx1, digctrl_kx2;

	/* Clear XFI clock comp in non-10G single lane mode. */
	bnx2x_cl45_read_and_write(bp, phy, MDIO_WC_DEVAD,
				  MDIO_WC_REG_RX66_CONTROL, ~(3<<13));

	bnx2x_warpcore_set_lpi_passthrough(phy, params);

	if (always_autoneg || phy->req_line_speed == SPEED_AUTO_NEG) {
		/* SGMII Autoneg */
		bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
					 MDIO_WC_REG_COMBO_IEEE0_MIICTRL,
					 0x1000);
		DP(NETIF_MSG_LINK, "set SGMII AUTONEG\n");
	} else {
		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				MDIO_WC_REG_COMBO_IEEE0_MIICTRL, &val16);
		val16 &= 0xcebf;
		switch (phy->req_line_speed) {
		case SPEED_10:
			break;
		case SPEED_100:
			val16 |= 0x2000;
			break;
		case SPEED_1000:
			val16 |= 0x0040;
			break;
		default:
			DP(NETIF_MSG_LINK,
			   "Speed not supported: 0x%x\n", phy->req_line_speed);
			return;
		}

		if (phy->req_duplex == DUPLEX_FULL)
			val16 |= 0x0100;

		bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
				MDIO_WC_REG_COMBO_IEEE0_MIICTRL, val16);

		DP(NETIF_MSG_LINK, "set SGMII force speed %d\n",
			       phy->req_line_speed);
		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				MDIO_WC_REG_COMBO_IEEE0_MIICTRL, &val16);
		DP(NETIF_MSG_LINK, "  (readback) %x\n", val16);
	}

	/* SGMII Slave mode and disable signal detect */
	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X1, &digctrl_kx1);
	if (fiber_mode)
		digctrl_kx1 = 1;
	else
		digctrl_kx1 &= 0xff4a;

	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X1,
			digctrl_kx1);

	/* Turn off parallel detect */
	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X2, &digctrl_kx2);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X2,
			(digctrl_kx2 & ~(1<<2)));

	/* Re-enable parallel detect */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X2,
			(digctrl_kx2 | (1<<2)));

	/* Enable autodet */
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X1,
			(digctrl_kx1 | 0x10));
}

static void bnx2x_warpcore_reset_lane(struct bnx2x *bp,
				      struct bnx2x_phy *phy,
				      u8 reset)
{
	u16 val;
	/* Take lane out of reset after configuration is finished */
	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_DIGITAL5_MISC6, &val);
	if (reset)
		val |= 0xC000;
	else
		val &= 0x3FFF;
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_DIGITAL5_MISC6, val);
	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_DIGITAL5_MISC6, &val);
}
/* Clear SFI/XFI link settings registers */
static void bnx2x_warpcore_clear_regs(struct bnx2x_phy *phy,
				      struct link_params *params,
				      u16 lane)
{
	struct bnx2x *bp = params->bp;
	u16 i;
	static struct bnx2x_reg_set wc_regs[] = {
		{MDIO_AN_DEVAD, MDIO_AN_REG_CTRL, 0},
		{MDIO_WC_DEVAD, MDIO_WC_REG_FX100_CTRL1, 0x014a},
		{MDIO_WC_DEVAD, MDIO_WC_REG_FX100_CTRL3, 0x0800},
		{MDIO_WC_DEVAD, MDIO_WC_REG_DIGITAL4_MISC3, 0x8008},
		{MDIO_WC_DEVAD, MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X1,
			0x0195},
		{MDIO_WC_DEVAD, MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X2,
			0x0007},
		{MDIO_WC_DEVAD, MDIO_WC_REG_SERDESDIGITAL_CONTROL1000X3,
			0x0002},
		{MDIO_WC_DEVAD, MDIO_WC_REG_SERDESDIGITAL_MISC1, 0x6000},
		{MDIO_WC_DEVAD, MDIO_WC_REG_TX_FIR_TAP, 0x0000},
		{MDIO_WC_DEVAD, MDIO_WC_REG_IEEE0BLK_MIICNTL, 0x2040},
		{MDIO_WC_DEVAD, MDIO_WC_REG_COMBO_IEEE0_MIICTRL, 0x0140}
	};
	/* Set XFI clock comp as default. */
	bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_RX66_CONTROL, (3<<13));

	for (i = 0; i < ARRAY_SIZE(wc_regs); i++)
		bnx2x_cl45_write(bp, phy, wc_regs[i].devad, wc_regs[i].reg,
				 wc_regs[i].val);

	lane = bnx2x_get_warpcore_lane(phy, params);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_TX0_TX_DRIVER + 0x10*lane, 0x0990);

}

static int bnx2x_get_mod_abs_int_cfg(struct bnx2x *bp,
						u32 chip_id,
						u32 shmem_base, u8 port,
						u8 *gpio_num, u8 *gpio_port)
{
	u32 cfg_pin;
	*gpio_num = 0;
	*gpio_port = 0;
	if (CHIP_IS_E3(bp)) {
		cfg_pin = (REG_RD(bp, shmem_base +
				offsetof(struct shmem_region,
				dev_info.port_hw_config[port].e3_sfp_ctrl)) &
				PORT_HW_CFG_E3_MOD_ABS_MASK) >>
				PORT_HW_CFG_E3_MOD_ABS_SHIFT;

		/* Should not happen. This function called upon interrupt
		 * triggered by GPIO ( since EPIO can only generate interrupts
		 * to MCP).
		 * So if this function was called and none of the GPIOs was set,
		 * it means the shit hit the fan.
		 */
		if ((cfg_pin < PIN_CFG_GPIO0_P0) ||
		    (cfg_pin > PIN_CFG_GPIO3_P1)) {
			DP(NETIF_MSG_LINK,
			   "No cfg pin %x for module detect indication\n",
			   cfg_pin);
			return -EINVAL;
		}

		*gpio_num = (cfg_pin - PIN_CFG_GPIO0_P0) & 0x3;
		*gpio_port = (cfg_pin - PIN_CFG_GPIO0_P0) >> 2;
	} else {
		*gpio_num = MISC_REGISTERS_GPIO_3;
		*gpio_port = port;
	}

	return 0;
}

static int bnx2x_is_sfp_module_plugged(struct bnx2x_phy *phy,
				       struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	u8 gpio_num, gpio_port;
	u32 gpio_val;
	if (bnx2x_get_mod_abs_int_cfg(bp, params->chip_id,
				      params->shmem_base, params->port,
				      &gpio_num, &gpio_port) != 0)
		return 0;
	gpio_val = bnx2x_get_gpio(bp, gpio_num, gpio_port);

	/* Call the handling function in case module is detected */
	if (gpio_val == 0)
		return 1;
	else
		return 0;
}
static int bnx2x_warpcore_get_sigdet(struct bnx2x_phy *phy,
				     struct link_params *params)
{
	u16 gp2_status_reg0, lane;
	struct bnx2x *bp = params->bp;

	lane = bnx2x_get_warpcore_lane(phy, params);

	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD, MDIO_WC_REG_GP2_STATUS_GP_2_0,
				 &gp2_status_reg0);

	return (gp2_status_reg0 >> (8+lane)) & 0x1;
}

static void bnx2x_warpcore_config_runtime(struct bnx2x_phy *phy,
					  struct link_params *params,
					  struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u32 serdes_net_if;
	u16 gp_status1 = 0, lnkup = 0, lnkup_kr = 0;

	vars->turn_to_run_wc_rt = vars->turn_to_run_wc_rt ? 0 : 1;

	if (!vars->turn_to_run_wc_rt)
		return;

	if (vars->rx_tx_asic_rst) {
		u16 lane = bnx2x_get_warpcore_lane(phy, params);
		serdes_net_if = (REG_RD(bp, params->shmem_base +
				offsetof(struct shmem_region, dev_info.
				port_hw_config[params->port].default_cfg)) &
				PORT_HW_CFG_NET_SERDES_IF_MASK);

		switch (serdes_net_if) {
		case PORT_HW_CFG_NET_SERDES_IF_KR:
			/* Do we get link yet? */
			bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD, 0x81d1,
					&gp_status1);
			lnkup = (gp_status1 >> (8+lane)) & 0x1;/* 1G */
				/*10G KR*/
			lnkup_kr = (gp_status1 >> (12+lane)) & 0x1;

			if (lnkup_kr || lnkup) {
				vars->rx_tx_asic_rst = 0;
			} else {
				/* Reset the lane to see if link comes up.*/
				bnx2x_warpcore_reset_lane(bp, phy, 1);
				bnx2x_warpcore_reset_lane(bp, phy, 0);

				/* Restart Autoneg */
				bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD,
					MDIO_WC_REG_IEEE0BLK_MIICNTL, 0x1200);

				vars->rx_tx_asic_rst--;
				DP(NETIF_MSG_LINK, "0x%x retry left\n",
				vars->rx_tx_asic_rst);
			}
			break;

		default:
			break;
		}

	} /*params->rx_tx_asic_rst*/

}
static void bnx2x_warpcore_config_sfi(struct bnx2x_phy *phy,
				      struct link_params *params)
{
	u16 lane = bnx2x_get_warpcore_lane(phy, params);
	struct bnx2x *bp = params->bp;
	bnx2x_warpcore_clear_regs(phy, params, lane);
	if ((params->req_line_speed[LINK_CONFIG_IDX(INT_PHY)] ==
	     SPEED_10000) &&
	    (phy->media_type != ETH_PHY_SFP_1G_FIBER)) {
		DP(NETIF_MSG_LINK, "Setting 10G SFI\n");
		bnx2x_warpcore_set_10G_XFI(phy, params, 0);
	} else {
		DP(NETIF_MSG_LINK, "Setting 1G Fiber\n");
		bnx2x_warpcore_set_sgmii_speed(phy, params, 1, 0);
	}
}

static void bnx2x_sfp_e3_set_transmitter(struct link_params *params,
					 struct bnx2x_phy *phy,
					 u8 tx_en)
{
	struct bnx2x *bp = params->bp;
	u32 cfg_pin;
	u8 port = params->port;

	cfg_pin = REG_RD(bp, params->shmem_base +
			 offsetof(struct shmem_region,
				  dev_info.port_hw_config[port].e3_sfp_ctrl)) &
		PORT_HW_CFG_E3_TX_LASER_MASK;
	/* Set the !tx_en since this pin is DISABLE_TX_LASER */
	DP(NETIF_MSG_LINK, "Setting WC TX to %d\n", tx_en);

	/* For 20G, the expected pin to be used is 3 pins after the current */
	bnx2x_set_cfg_pin(bp, cfg_pin, tx_en ^ 1);
	if (phy->speed_cap_mask & PORT_HW_CFG_SPEED_CAPABILITY_D0_20G)
		bnx2x_set_cfg_pin(bp, cfg_pin + 3, tx_en ^ 1);
}

static void bnx2x_warpcore_config_init(struct bnx2x_phy *phy,
				       struct link_params *params,
				       struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u32 serdes_net_if;
	u8 fiber_mode;
	u16 lane = bnx2x_get_warpcore_lane(phy, params);
	serdes_net_if = (REG_RD(bp, params->shmem_base +
			 offsetof(struct shmem_region, dev_info.
				  port_hw_config[params->port].default_cfg)) &
			 PORT_HW_CFG_NET_SERDES_IF_MASK);
	DP(NETIF_MSG_LINK, "Begin Warpcore init, link_speed %d, "
			   "serdes_net_if = 0x%x\n",
		       vars->line_speed, serdes_net_if);
	bnx2x_set_aer_mmd(params, phy);
	bnx2x_warpcore_reset_lane(bp, phy, 1);
	vars->phy_flags |= PHY_XGXS_FLAG;
	if ((serdes_net_if == PORT_HW_CFG_NET_SERDES_IF_SGMII) ||
	    (phy->req_line_speed &&
	     ((phy->req_line_speed == SPEED_100) ||
	      (phy->req_line_speed == SPEED_10)))) {
		vars->phy_flags |= PHY_SGMII_FLAG;
		DP(NETIF_MSG_LINK, "Setting SGMII mode\n");
		bnx2x_warpcore_clear_regs(phy, params, lane);
		bnx2x_warpcore_set_sgmii_speed(phy, params, 0, 1);
	} else {
		switch (serdes_net_if) {
		case PORT_HW_CFG_NET_SERDES_IF_KR:
			/* Enable KR Auto Neg */
			if (params->loopback_mode != LOOPBACK_EXT)
				bnx2x_warpcore_enable_AN_KR(phy, params, vars);
			else {
				DP(NETIF_MSG_LINK, "Setting KR 10G-Force\n");
				bnx2x_warpcore_set_10G_KR(phy, params, vars);
			}
			break;

		case PORT_HW_CFG_NET_SERDES_IF_XFI:
			bnx2x_warpcore_clear_regs(phy, params, lane);
			if (vars->line_speed == SPEED_10000) {
				DP(NETIF_MSG_LINK, "Setting 10G XFI\n");
				bnx2x_warpcore_set_10G_XFI(phy, params, 1);
			} else {
				if (SINGLE_MEDIA_DIRECT(params)) {
					DP(NETIF_MSG_LINK, "1G Fiber\n");
					fiber_mode = 1;
				} else {
					DP(NETIF_MSG_LINK, "10/100/1G SGMII\n");
					fiber_mode = 0;
				}
				bnx2x_warpcore_set_sgmii_speed(phy,
								params,
								fiber_mode,
								0);
			}

			break;

		case PORT_HW_CFG_NET_SERDES_IF_SFI:
			/* Issue Module detection if module is plugged, or
			 * enabled transmitter to avoid current leakage in case
			 * no module is connected
			 */
			if ((params->loopback_mode == LOOPBACK_NONE) ||
			    (params->loopback_mode == LOOPBACK_EXT)) {
				if (bnx2x_is_sfp_module_plugged(phy, params))
					bnx2x_sfp_module_detection(phy, params);
				else
					bnx2x_sfp_e3_set_transmitter(params,
								     phy, 1);
			}

			bnx2x_warpcore_config_sfi(phy, params);
			break;

		case PORT_HW_CFG_NET_SERDES_IF_DXGXS:
			if (vars->line_speed != SPEED_20000) {
				DP(NETIF_MSG_LINK, "Speed not supported yet\n");
				return;
			}
			DP(NETIF_MSG_LINK, "Setting 20G DXGXS\n");
			bnx2x_warpcore_set_20G_DXGXS(bp, phy, lane);
			/* Issue Module detection */

			bnx2x_sfp_module_detection(phy, params);
			break;
		case PORT_HW_CFG_NET_SERDES_IF_KR2:
			if (!params->loopback_mode) {
				bnx2x_warpcore_enable_AN_KR(phy, params, vars);
			} else {
				DP(NETIF_MSG_LINK, "Setting KR 20G-Force\n");
				bnx2x_warpcore_set_20G_force_KR2(phy, params);
			}
			break;
		default:
			DP(NETIF_MSG_LINK,
			   "Unsupported Serdes Net Interface 0x%x\n",
			   serdes_net_if);
			return;
		}
	}

	/* Take lane out of reset after configuration is finished */
	bnx2x_warpcore_reset_lane(bp, phy, 0);
	DP(NETIF_MSG_LINK, "Exit config init\n");
}

static void bnx2x_warpcore_link_reset(struct bnx2x_phy *phy,
				      struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	u16 val16, lane;
	bnx2x_sfp_e3_set_transmitter(params, phy, 0);
	bnx2x_set_mdio_emac_per_phy(bp, params);
	bnx2x_set_aer_mmd(params, phy);
	/* Global register */
	bnx2x_warpcore_reset_lane(bp, phy, 1);

	/* Clear loopback settings (if any) */
	/* 10G & 20G */
	bnx2x_cl45_read_and_write(bp, phy, MDIO_WC_DEVAD,
				  MDIO_WC_REG_COMBO_IEEE0_MIICTRL, 0xBFFF);

	bnx2x_cl45_read_and_write(bp, phy, MDIO_WC_DEVAD,
				  MDIO_WC_REG_IEEE0BLK_MIICNTL, 0xfffe);

	/* Update those 1-copy registers */
	CL22_WR_OVER_CL45(bp, phy, MDIO_REG_BANK_AER_BLOCK,
			  MDIO_AER_BLOCK_AER_REG, 0);
	/* Enable 1G MDIO (1-copy) */
	bnx2x_cl45_read_and_write(bp, phy, MDIO_WC_DEVAD,
				  MDIO_WC_REG_XGXSBLK0_XGXSCONTROL,
				  ~0x10);

	bnx2x_cl45_read_and_write(bp, phy, MDIO_WC_DEVAD,
				  MDIO_WC_REG_XGXSBLK1_LANECTRL2, 0xff00);
	lane = bnx2x_get_warpcore_lane(phy, params);
	/* Disable CL36 PCS Tx */
	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_XGXSBLK1_LANECTRL0, &val16);
	val16 |= (0x11 << lane);
	if (phy->flags & FLAGS_WC_DUAL_MODE)
		val16 |= (0x22 << lane);
	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_XGXSBLK1_LANECTRL0, val16);

	bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
			MDIO_WC_REG_XGXSBLK1_LANECTRL1, &val16);
	val16 &= ~(0x0303 << (lane << 1));
	val16 |= (0x0101 << (lane << 1));
	if (phy->flags & FLAGS_WC_DUAL_MODE) {
		val16 &= ~(0x0c0c << (lane << 1));
		val16 |= (0x0404 << (lane << 1));
	}

	bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
			 MDIO_WC_REG_XGXSBLK1_LANECTRL1, val16);
	/* Restore AER */
	bnx2x_set_aer_mmd(params, phy);

}

static void bnx2x_set_warpcore_loopback(struct bnx2x_phy *phy,
					struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	u16 val16;
	u32 lane;
	DP(NETIF_MSG_LINK, "Setting Warpcore loopback type %x, speed %d\n",
		       params->loopback_mode, phy->req_line_speed);

	if (phy->req_line_speed < SPEED_10000 ||
	    phy->supported & SUPPORTED_20000baseKR2_Full) {
		/* 10/100/1000/20G-KR2 */

		/* Update those 1-copy registers */
		CL22_WR_OVER_CL45(bp, phy, MDIO_REG_BANK_AER_BLOCK,
				  MDIO_AER_BLOCK_AER_REG, 0);
		/* Enable 1G MDIO (1-copy) */
		bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
					 MDIO_WC_REG_XGXSBLK0_XGXSCONTROL,
					 0x10);
		/* Set 1G loopback based on lane (1-copy) */
		lane = bnx2x_get_warpcore_lane(phy, params);
		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				MDIO_WC_REG_XGXSBLK1_LANECTRL2, &val16);
		val16 |= (1<<lane);
		if (phy->flags & FLAGS_WC_DUAL_MODE)
			val16 |= (2<<lane);
		bnx2x_cl45_write(bp, phy, MDIO_WC_DEVAD,
				 MDIO_WC_REG_XGXSBLK1_LANECTRL2,
				 val16);

		/* Switch back to 4-copy registers */
		bnx2x_set_aer_mmd(params, phy);
	} else {
		/* 10G / 20G-DXGXS */
		bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
					 MDIO_WC_REG_COMBO_IEEE0_MIICTRL,
					 0x4000);
		bnx2x_cl45_read_or_write(bp, phy, MDIO_WC_DEVAD,
					 MDIO_WC_REG_IEEE0BLK_MIICNTL, 0x1);
	}
}



static void bnx2x_sync_link(struct link_params *params,
			     struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u8 link_10g_plus;
	if (vars->link_status & LINK_STATUS_PHYSICAL_LINK_FLAG)
		vars->phy_flags |= PHY_PHYSICAL_LINK_FLAG;
	vars->link_up = (vars->link_status & LINK_STATUS_LINK_UP);
	if (vars->link_up) {
		DP(NETIF_MSG_LINK, "phy link up\n");

		vars->phy_link_up = 1;
		vars->duplex = DUPLEX_FULL;
		switch (vars->link_status &
			LINK_STATUS_SPEED_AND_DUPLEX_MASK) {
		case LINK_10THD:
			vars->duplex = DUPLEX_HALF;
			/* Fall thru */
		case LINK_10TFD:
			vars->line_speed = SPEED_10;
			break;

		case LINK_100TXHD:
			vars->duplex = DUPLEX_HALF;
			/* Fall thru */
		case LINK_100T4:
		case LINK_100TXFD:
			vars->line_speed = SPEED_100;
			break;

		case LINK_1000THD:
			vars->duplex = DUPLEX_HALF;
			/* Fall thru */
		case LINK_1000TFD:
			vars->line_speed = SPEED_1000;
			break;

		case LINK_2500THD:
			vars->duplex = DUPLEX_HALF;
			/* Fall thru */
		case LINK_2500TFD:
			vars->line_speed = SPEED_2500;
			break;

		case LINK_10GTFD:
			vars->line_speed = SPEED_10000;
			break;
		case LINK_20GTFD:
			vars->line_speed = SPEED_20000;
			break;
		default:
			break;
		}
		vars->flow_ctrl = 0;
		if (vars->link_status & LINK_STATUS_TX_FLOW_CONTROL_ENABLED)
			vars->flow_ctrl |= BNX2X_FLOW_CTRL_TX;

		if (vars->link_status & LINK_STATUS_RX_FLOW_CONTROL_ENABLED)
			vars->flow_ctrl |= BNX2X_FLOW_CTRL_RX;

		if (!vars->flow_ctrl)
			vars->flow_ctrl = BNX2X_FLOW_CTRL_NONE;

		if (vars->line_speed &&
		    ((vars->line_speed == SPEED_10) ||
		     (vars->line_speed == SPEED_100))) {
			vars->phy_flags |= PHY_SGMII_FLAG;
		} else {
			vars->phy_flags &= ~PHY_SGMII_FLAG;
		}
		if (vars->line_speed &&
		    USES_WARPCORE(bp) &&
		    (vars->line_speed == SPEED_1000))
			vars->phy_flags |= PHY_SGMII_FLAG;
		/* Anything 10 and over uses the bmac */
		link_10g_plus = (vars->line_speed >= SPEED_10000);

		if (link_10g_plus) {
			if (USES_WARPCORE(bp))
				vars->mac_type = MAC_TYPE_XMAC;
			else
				vars->mac_type = MAC_TYPE_BMAC;
		} else {
			if (USES_WARPCORE(bp))
				vars->mac_type = MAC_TYPE_UMAC;
			else
				vars->mac_type = MAC_TYPE_EMAC;
		}
	} else { /* Link down */
		DP(NETIF_MSG_LINK, "phy link down\n");

		vars->phy_link_up = 0;

		vars->line_speed = 0;
		vars->duplex = DUPLEX_FULL;
		vars->flow_ctrl = BNX2X_FLOW_CTRL_NONE;

		/* Indicate no mac active */
		vars->mac_type = MAC_TYPE_NONE;
		if (vars->link_status & LINK_STATUS_PHYSICAL_LINK_FLAG)
			vars->phy_flags |= PHY_HALF_OPEN_CONN_FLAG;
		if (vars->link_status & LINK_STATUS_SFP_TX_FAULT)
			vars->phy_flags |= PHY_SFP_TX_FAULT_FLAG;
	}
}

void bnx2x_link_status_update(struct link_params *params,
			      struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u8 port = params->port;
	u32 sync_offset, media_types;
	/* Update PHY configuration */
	set_phy_vars(params, vars);

	vars->link_status = REG_RD(bp, params->shmem_base +
				   offsetof(struct shmem_region,
					    port_mb[port].link_status));

	/* Force link UP in non LOOPBACK_EXT loopback mode(s) */
	if (params->loopback_mode != LOOPBACK_NONE &&
	    params->loopback_mode != LOOPBACK_EXT)
		vars->link_status |= LINK_STATUS_LINK_UP;

	if (bnx2x_eee_has_cap(params))
		vars->eee_status = REG_RD(bp, params->shmem2_base +
					  offsetof(struct shmem2_region,
						   eee_status[params->port]));

	vars->phy_flags = PHY_XGXS_FLAG;
	bnx2x_sync_link(params, vars);
	/* Sync media type */
	sync_offset = params->shmem_base +
			offsetof(struct shmem_region,
				 dev_info.port_hw_config[port].media_type);
	media_types = REG_RD(bp, sync_offset);

	params->phy[INT_PHY].media_type =
		(media_types & PORT_HW_CFG_MEDIA_TYPE_PHY0_MASK) >>
		PORT_HW_CFG_MEDIA_TYPE_PHY0_SHIFT;
	params->phy[EXT_PHY1].media_type =
		(media_types & PORT_HW_CFG_MEDIA_TYPE_PHY1_MASK) >>
		PORT_HW_CFG_MEDIA_TYPE_PHY1_SHIFT;
	params->phy[EXT_PHY2].media_type =
		(media_types & PORT_HW_CFG_MEDIA_TYPE_PHY2_MASK) >>
		PORT_HW_CFG_MEDIA_TYPE_PHY2_SHIFT;
	DP(NETIF_MSG_LINK, "media_types = 0x%x\n", media_types);

	/* Sync AEU offset */
	sync_offset = params->shmem_base +
			offsetof(struct shmem_region,
				 dev_info.port_hw_config[port].aeu_int_mask);

	vars->aeu_int_mask = REG_RD(bp, sync_offset);

	/* Sync PFC status */
	if (vars->link_status & LINK_STATUS_PFC_ENABLED)
		params->feature_config_flags |=
					FEATURE_CONFIG_PFC_ENABLED;
	else
		params->feature_config_flags &=
					~FEATURE_CONFIG_PFC_ENABLED;

	if (SHMEM2_HAS(bp, link_attr_sync))
		params->link_attr_sync = SHMEM2_RD(bp,
						 link_attr_sync[params->port]);

	DP(NETIF_MSG_LINK, "link_status 0x%x  phy_link_up %x int_mask 0x%x\n",
		 vars->link_status, vars->phy_link_up, vars->aeu_int_mask);
	DP(NETIF_MSG_LINK, "line_speed %x  duplex %x  flow_ctrl 0x%x\n",
		 vars->line_speed, vars->duplex, vars->flow_ctrl);
}

static void bnx2x_set_master_ln(struct link_params *params,
				struct bnx2x_phy *phy)
{
	struct bnx2x *bp = params->bp;
	u16 new_master_ln, ser_lane;
	ser_lane = ((params->lane_config &
		     PORT_HW_CFG_LANE_SWAP_CFG_MASTER_MASK) >>
		    PORT_HW_CFG_LANE_SWAP_CFG_MASTER_SHIFT);

	/* Set the master_ln for AN */
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_XGXS_BLOCK2,
			  MDIO_XGXS_BLOCK2_TEST_MODE_LANE,
			  &new_master_ln);

	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_XGXS_BLOCK2 ,
			  MDIO_XGXS_BLOCK2_TEST_MODE_LANE,
			  (new_master_ln | ser_lane));
}

static int bnx2x_reset_unicore(struct link_params *params,
			       struct bnx2x_phy *phy,
			       u8 set_serdes)
{
	struct bnx2x *bp = params->bp;
	u16 mii_control;
	u16 i;
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_COMBO_IEEE0,
			  MDIO_COMBO_IEEE0_MII_CONTROL, &mii_control);

	/* Reset the unicore */
	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_COMBO_IEEE0,
			  MDIO_COMBO_IEEE0_MII_CONTROL,
			  (mii_control |
			   MDIO_COMBO_IEEO_MII_CONTROL_RESET));
	if (set_serdes)
		bnx2x_set_serdes_access(bp, params->port);

	/* Wait for the reset to self clear */
	for (i = 0; i < MDIO_ACCESS_TIMEOUT; i++) {
		udelay(5);

		/* The reset erased the previous bank value */
		CL22_RD_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_COMBO_IEEE0,
				  MDIO_COMBO_IEEE0_MII_CONTROL,
				  &mii_control);

		if (!(mii_control & MDIO_COMBO_IEEO_MII_CONTROL_RESET)) {
			udelay(5);
			return 0;
		}
	}

	netdev_err(bp->dev,  "Warning: PHY was not initialized,"
			      " Port %d\n",
			 params->port);
	DP(NETIF_MSG_LINK, "BUG! XGXS is still in reset!\n");
	return -EINVAL;

}

static void bnx2x_set_swap_lanes(struct link_params *params,
				 struct bnx2x_phy *phy)
{
	struct bnx2x *bp = params->bp;
	/* Each two bits represents a lane number:
	 * No swap is 0123 => 0x1b no need to enable the swap
	 */
	u16 rx_lane_swap, tx_lane_swap;

	rx_lane_swap = ((params->lane_config &
			 PORT_HW_CFG_LANE_SWAP_CFG_RX_MASK) >>
			PORT_HW_CFG_LANE_SWAP_CFG_RX_SHIFT);
	tx_lane_swap = ((params->lane_config &
			 PORT_HW_CFG_LANE_SWAP_CFG_TX_MASK) >>
			PORT_HW_CFG_LANE_SWAP_CFG_TX_SHIFT);

	if (rx_lane_swap != 0x1b) {
		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_XGXS_BLOCK2,
				  MDIO_XGXS_BLOCK2_RX_LN_SWAP,
				  (rx_lane_swap |
				   MDIO_XGXS_BLOCK2_RX_LN_SWAP_ENABLE |
				   MDIO_XGXS_BLOCK2_RX_LN_SWAP_FORCE_ENABLE));
	} else {
		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_XGXS_BLOCK2,
				  MDIO_XGXS_BLOCK2_RX_LN_SWAP, 0);
	}

	if (tx_lane_swap != 0x1b) {
		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_XGXS_BLOCK2,
				  MDIO_XGXS_BLOCK2_TX_LN_SWAP,
				  (tx_lane_swap |
				   MDIO_XGXS_BLOCK2_TX_LN_SWAP_ENABLE));
	} else {
		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_XGXS_BLOCK2,
				  MDIO_XGXS_BLOCK2_TX_LN_SWAP, 0);
	}
}

static void bnx2x_set_parallel_detection(struct bnx2x_phy *phy,
					 struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	u16 control2;
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_SERDES_DIGITAL,
			  MDIO_SERDES_DIGITAL_A_1000X_CONTROL2,
			  &control2);
	if (phy->speed_cap_mask & PORT_HW_CFG_SPEED_CAPABILITY_D0_1G)
		control2 |= MDIO_SERDES_DIGITAL_A_1000X_CONTROL2_PRL_DT_EN;
	else
		control2 &= ~MDIO_SERDES_DIGITAL_A_1000X_CONTROL2_PRL_DT_EN;
	DP(NETIF_MSG_LINK, "phy->speed_cap_mask = 0x%x, control2 = 0x%x\n",
		phy->speed_cap_mask, control2);
	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_SERDES_DIGITAL,
			  MDIO_SERDES_DIGITAL_A_1000X_CONTROL2,
			  control2);

	if ((phy->type == PORT_HW_CFG_XGXS_EXT_PHY_TYPE_DIRECT) &&
	     (phy->speed_cap_mask &
		    PORT_HW_CFG_SPEED_CAPABILITY_D0_10G)) {
		DP(NETIF_MSG_LINK, "XGXS\n");

		CL22_WR_OVER_CL45(bp, phy,
				 MDIO_REG_BANK_10G_PARALLEL_DETECT,
				 MDIO_10G_PARALLEL_DETECT_PAR_DET_10G_LINK,
				 MDIO_10G_PARALLEL_DETECT_PAR_DET_10G_LINK_CNT);

		CL22_RD_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_10G_PARALLEL_DETECT,
				  MDIO_10G_PARALLEL_DETECT_PAR_DET_10G_CONTROL,
				  &control2);


		control2 |=
		    MDIO_10G_PARALLEL_DETECT_PAR_DET_10G_CONTROL_PARDET10G_EN;

		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_10G_PARALLEL_DETECT,
				  MDIO_10G_PARALLEL_DETECT_PAR_DET_10G_CONTROL,
				  control2);

		/* Disable parallel detection of HiG */
		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_XGXS_BLOCK2,
				  MDIO_XGXS_BLOCK2_UNICORE_MODE_10G,
				  MDIO_XGXS_BLOCK2_UNICORE_MODE_10G_CX4_XGXS |
				  MDIO_XGXS_BLOCK2_UNICORE_MODE_10G_HIGIG_XGXS);
	}
}

static void bnx2x_set_autoneg(struct bnx2x_phy *phy,
			      struct link_params *params,
			      struct link_vars *vars,
			      u8 enable_cl73)
{
	struct bnx2x *bp = params->bp;
	u16 reg_val;

	/* CL37 Autoneg */
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_COMBO_IEEE0,
			  MDIO_COMBO_IEEE0_MII_CONTROL, &reg_val);

	/* CL37 Autoneg Enabled */
	if (vars->line_speed == SPEED_AUTO_NEG)
		reg_val |= MDIO_COMBO_IEEO_MII_CONTROL_AN_EN;
	else /* CL37 Autoneg Disabled */
		reg_val &= ~(MDIO_COMBO_IEEO_MII_CONTROL_AN_EN |
			     MDIO_COMBO_IEEO_MII_CONTROL_RESTART_AN);

	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_COMBO_IEEE0,
			  MDIO_COMBO_IEEE0_MII_CONTROL, reg_val);

	/* Enable/Disable Autodetection */

	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_SERDES_DIGITAL,
			  MDIO_SERDES_DIGITAL_A_1000X_CONTROL1, &reg_val);
	reg_val &= ~(MDIO_SERDES_DIGITAL_A_1000X_CONTROL1_SIGNAL_DETECT_EN |
		    MDIO_SERDES_DIGITAL_A_1000X_CONTROL1_INVERT_SIGNAL_DETECT);
	reg_val |= MDIO_SERDES_DIGITAL_A_1000X_CONTROL1_FIBER_MODE;
	if (vars->line_speed == SPEED_AUTO_NEG)
		reg_val |= MDIO_SERDES_DIGITAL_A_1000X_CONTROL1_AUTODET;
	else
		reg_val &= ~MDIO_SERDES_DIGITAL_A_1000X_CONTROL1_AUTODET;

	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_SERDES_DIGITAL,
			  MDIO_SERDES_DIGITAL_A_1000X_CONTROL1, reg_val);

	/* Enable TetonII and BAM autoneg */
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_BAM_NEXT_PAGE,
			  MDIO_BAM_NEXT_PAGE_MP5_NEXT_PAGE_CTRL,
			  &reg_val);
	if (vars->line_speed == SPEED_AUTO_NEG) {
		/* Enable BAM aneg Mode and TetonII aneg Mode */
		reg_val |= (MDIO_BAM_NEXT_PAGE_MP5_NEXT_PAGE_CTRL_BAM_MODE |
			    MDIO_BAM_NEXT_PAGE_MP5_NEXT_PAGE_CTRL_TETON_AN);
	} else {
		/* TetonII and BAM Autoneg Disabled */
		reg_val &= ~(MDIO_BAM_NEXT_PAGE_MP5_NEXT_PAGE_CTRL_BAM_MODE |
			     MDIO_BAM_NEXT_PAGE_MP5_NEXT_PAGE_CTRL_TETON_AN);
	}
	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_BAM_NEXT_PAGE,
			  MDIO_BAM_NEXT_PAGE_MP5_NEXT_PAGE_CTRL,
			  reg_val);

	if (enable_cl73) {
		/* Enable Cl73 FSM status bits */
		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_CL73_USERB0,
				  MDIO_CL73_USERB0_CL73_UCTRL,
				  0xe);

		/* Enable BAM Station Manager*/
		CL22_WR_OVER_CL45(bp, phy,
			MDIO_REG_BANK_CL73_USERB0,
			MDIO_CL73_USERB0_CL73_BAM_CTRL1,
			MDIO_CL73_USERB0_CL73_BAM_CTRL1_BAM_EN |
			MDIO_CL73_USERB0_CL73_BAM_CTRL1_BAM_STATION_MNGR_EN |
			MDIO_CL73_USERB0_CL73_BAM_CTRL1_BAM_NP_AFTER_BP_EN);

		/* Advertise CL73 link speeds */
		CL22_RD_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_CL73_IEEEB1,
				  MDIO_CL73_IEEEB1_AN_ADV2,
				  &reg_val);
		if (phy->speed_cap_mask &
		    PORT_HW_CFG_SPEED_CAPABILITY_D0_10G)
			reg_val |= MDIO_CL73_IEEEB1_AN_ADV2_ADVR_10G_KX4;
		if (phy->speed_cap_mask &
		    PORT_HW_CFG_SPEED_CAPABILITY_D0_1G)
			reg_val |= MDIO_CL73_IEEEB1_AN_ADV2_ADVR_1000M_KX;

		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_CL73_IEEEB1,
				  MDIO_CL73_IEEEB1_AN_ADV2,
				  reg_val);

		/* CL73 Autoneg Enabled */
		reg_val = MDIO_CL73_IEEEB0_CL73_AN_CONTROL_AN_EN;

	} else /* CL73 Autoneg Disabled */
		reg_val = 0;

	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_CL73_IEEEB0,
			  MDIO_CL73_IEEEB0_CL73_AN_CONTROL, reg_val);
}

/* Program SerDes, forced speed */
static void bnx2x_program_serdes(struct bnx2x_phy *phy,
				 struct link_params *params,
				 struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u16 reg_val;

	/* Program duplex, disable autoneg and sgmii*/
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_COMBO_IEEE0,
			  MDIO_COMBO_IEEE0_MII_CONTROL, &reg_val);
	reg_val &= ~(MDIO_COMBO_IEEO_MII_CONTROL_FULL_DUPLEX |
		     MDIO_COMBO_IEEO_MII_CONTROL_AN_EN |
		     MDIO_COMBO_IEEO_MII_CONTROL_MAN_SGMII_SP_MASK);
	if (phy->req_duplex == DUPLEX_FULL)
		reg_val |= MDIO_COMBO_IEEO_MII_CONTROL_FULL_DUPLEX;
	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_COMBO_IEEE0,
			  MDIO_COMBO_IEEE0_MII_CONTROL, reg_val);

	/* Program speed
	 *  - needed only if the speed is greater than 1G (2.5G or 10G)
	 */
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_SERDES_DIGITAL,
			  MDIO_SERDES_DIGITAL_MISC1, &reg_val);
	/* Clearing the speed value before setting the right speed */
	DP(NETIF_MSG_LINK, "MDIO_REG_BANK_SERDES_DIGITAL = 0x%x\n", reg_val);

	reg_val &= ~(MDIO_SERDES_DIGITAL_MISC1_FORCE_SPEED_MASK |
		     MDIO_SERDES_DIGITAL_MISC1_FORCE_SPEED_SEL);

	if (!((vars->line_speed == SPEED_1000) ||
	      (vars->line_speed == SPEED_100) ||
	      (vars->line_speed == SPEED_10))) {

		reg_val |= (MDIO_SERDES_DIGITAL_MISC1_REFCLK_SEL_156_25M |
			    MDIO_SERDES_DIGITAL_MISC1_FORCE_SPEED_SEL);
		if (vars->line_speed == SPEED_10000)
			reg_val |=
				MDIO_SERDES_DIGITAL_MISC1_FORCE_SPEED_10G_CX4;
	}

	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_SERDES_DIGITAL,
			  MDIO_SERDES_DIGITAL_MISC1, reg_val);

}

static void bnx2x_set_brcm_cl37_advertisement(struct bnx2x_phy *phy,
					      struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	u16 val = 0;

	/* Set extended capabilities */
	if (phy->speed_cap_mask & PORT_HW_CFG_SPEED_CAPABILITY_D0_2_5G)
		val |= MDIO_OVER_1G_UP1_2_5G;
	if (phy->speed_cap_mask & PORT_HW_CFG_SPEED_CAPABILITY_D0_10G)
		val |= MDIO_OVER_1G_UP1_10G;
	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_OVER_1G,
			  MDIO_OVER_1G_UP1, val);

	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_OVER_1G,
			  MDIO_OVER_1G_UP3, 0x400);
}

static void bnx2x_set_ieee_aneg_advertisement(struct bnx2x_phy *phy,
					      struct link_params *params,
					      u16 ieee_fc)
{
	struct bnx2x *bp = params->bp;
	u16 val;
	/* For AN, we are always publishing full duplex */

	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_COMBO_IEEE0,
			  MDIO_COMBO_IEEE0_AUTO_NEG_ADV, ieee_fc);
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_CL73_IEEEB1,
			  MDIO_CL73_IEEEB1_AN_ADV1, &val);
	val &= ~MDIO_CL73_IEEEB1_AN_ADV1_PAUSE_BOTH;
	val |= ((ieee_fc<<3) & MDIO_CL73_IEEEB1_AN_ADV1_PAUSE_MASK);
	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_CL73_IEEEB1,
			  MDIO_CL73_IEEEB1_AN_ADV1, val);
}

static void bnx2x_restart_autoneg(struct bnx2x_phy *phy,
				  struct link_params *params,
				  u8 enable_cl73)
{
	struct bnx2x *bp = params->bp;
	u16 mii_control;

	DP(NETIF_MSG_LINK, "bnx2x_restart_autoneg\n");
	/* Enable and restart BAM/CL37 aneg */

	if (enable_cl73) {
		CL22_RD_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_CL73_IEEEB0,
				  MDIO_CL73_IEEEB0_CL73_AN_CONTROL,
				  &mii_control);

		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_CL73_IEEEB0,
				  MDIO_CL73_IEEEB0_CL73_AN_CONTROL,
				  (mii_control |
				  MDIO_CL73_IEEEB0_CL73_AN_CONTROL_AN_EN |
				  MDIO_CL73_IEEEB0_CL73_AN_CONTROL_RESTART_AN));
	} else {

		CL22_RD_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_COMBO_IEEE0,
				  MDIO_COMBO_IEEE0_MII_CONTROL,
				  &mii_control);
		DP(NETIF_MSG_LINK,
			 "bnx2x_restart_autoneg mii_control before = 0x%x\n",
			 mii_control);
		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_COMBO_IEEE0,
				  MDIO_COMBO_IEEE0_MII_CONTROL,
				  (mii_control |
				   MDIO_COMBO_IEEO_MII_CONTROL_AN_EN |
				   MDIO_COMBO_IEEO_MII_CONTROL_RESTART_AN));
	}
}

static void bnx2x_initialize_sgmii_process(struct bnx2x_phy *phy,
					   struct link_params *params,
					   struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u16 control1;

	/* In SGMII mode, the unicore is always slave */

	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_SERDES_DIGITAL,
			  MDIO_SERDES_DIGITAL_A_1000X_CONTROL1,
			  &control1);
	control1 |= MDIO_SERDES_DIGITAL_A_1000X_CONTROL1_INVERT_SIGNAL_DETECT;
	/* Set sgmii mode (and not fiber) */
	control1 &= ~(MDIO_SERDES_DIGITAL_A_1000X_CONTROL1_FIBER_MODE |
		      MDIO_SERDES_DIGITAL_A_1000X_CONTROL1_AUTODET |
		      MDIO_SERDES_DIGITAL_A_1000X_CONTROL1_MSTR_MODE);
	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_SERDES_DIGITAL,
			  MDIO_SERDES_DIGITAL_A_1000X_CONTROL1,
			  control1);

	/* If forced speed */
	if (!(vars->line_speed == SPEED_AUTO_NEG)) {
		/* Set speed, disable autoneg */
		u16 mii_control;

		CL22_RD_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_COMBO_IEEE0,
				  MDIO_COMBO_IEEE0_MII_CONTROL,
				  &mii_control);
		mii_control &= ~(MDIO_COMBO_IEEO_MII_CONTROL_AN_EN |
				 MDIO_COMBO_IEEO_MII_CONTROL_MAN_SGMII_SP_MASK|
				 MDIO_COMBO_IEEO_MII_CONTROL_FULL_DUPLEX);

		switch (vars->line_speed) {
		case SPEED_100:
			mii_control |=
				MDIO_COMBO_IEEO_MII_CONTROL_MAN_SGMII_SP_100;
			break;
		case SPEED_1000:
			mii_control |=
				MDIO_COMBO_IEEO_MII_CONTROL_MAN_SGMII_SP_1000;
			break;
		case SPEED_10:
			/* There is nothing to set for 10M */
			break;
		default:
			/* Invalid speed for SGMII */
			DP(NETIF_MSG_LINK, "Invalid line_speed 0x%x\n",
				  vars->line_speed);
			break;
		}

		/* Setting the full duplex */
		if (phy->req_duplex == DUPLEX_FULL)
			mii_control |=
				MDIO_COMBO_IEEO_MII_CONTROL_FULL_DUPLEX;
		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_COMBO_IEEE0,
				  MDIO_COMBO_IEEE0_MII_CONTROL,
				  mii_control);

	} else { /* AN mode */
		/* Enable and restart AN */
		bnx2x_restart_autoneg(phy, params, 0);
	}
}

/* Link management
 */
static int bnx2x_direct_parallel_detect_used(struct bnx2x_phy *phy,
					     struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	u16 pd_10g, status2_1000x;
	if (phy->req_line_speed != SPEED_AUTO_NEG)
		return 0;
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_SERDES_DIGITAL,
			  MDIO_SERDES_DIGITAL_A_1000X_STATUS2,
			  &status2_1000x);
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_SERDES_DIGITAL,
			  MDIO_SERDES_DIGITAL_A_1000X_STATUS2,
			  &status2_1000x);
	if (status2_1000x & MDIO_SERDES_DIGITAL_A_1000X_STATUS2_AN_DISABLED) {
		DP(NETIF_MSG_LINK, "1G parallel detect link on port %d\n",
			 params->port);
		return 1;
	}

	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_10G_PARALLEL_DETECT,
			  MDIO_10G_PARALLEL_DETECT_PAR_DET_10G_STATUS,
			  &pd_10g);

	if (pd_10g & MDIO_10G_PARALLEL_DETECT_PAR_DET_10G_STATUS_PD_LINK) {
		DP(NETIF_MSG_LINK, "10G parallel detect link on port %d\n",
			 params->port);
		return 1;
	}
	return 0;
}

static void bnx2x_update_adv_fc(struct bnx2x_phy *phy,
				struct link_params *params,
				struct link_vars *vars,
				u32 gp_status)
{
	u16 ld_pause;   /* local driver */
	u16 lp_pause;   /* link partner */
	u16 pause_result;
	struct bnx2x *bp = params->bp;
	if ((gp_status &
	     (MDIO_GP_STATUS_TOP_AN_STATUS1_CL73_AUTONEG_COMPLETE |
	      MDIO_GP_STATUS_TOP_AN_STATUS1_CL73_MR_LP_NP_AN_ABLE)) ==
	    (MDIO_GP_STATUS_TOP_AN_STATUS1_CL73_AUTONEG_COMPLETE |
	     MDIO_GP_STATUS_TOP_AN_STATUS1_CL73_MR_LP_NP_AN_ABLE)) {

		CL22_RD_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_CL73_IEEEB1,
				  MDIO_CL73_IEEEB1_AN_ADV1,
				  &ld_pause);
		CL22_RD_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_CL73_IEEEB1,
				  MDIO_CL73_IEEEB1_AN_LP_ADV1,
				  &lp_pause);
		pause_result = (ld_pause &
				MDIO_CL73_IEEEB1_AN_ADV1_PAUSE_MASK) >> 8;
		pause_result |= (lp_pause &
				 MDIO_CL73_IEEEB1_AN_LP_ADV1_PAUSE_MASK) >> 10;
		DP(NETIF_MSG_LINK, "pause_result CL73 0x%x\n", pause_result);
	} else {
		CL22_RD_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_COMBO_IEEE0,
				  MDIO_COMBO_IEEE0_AUTO_NEG_ADV,
				  &ld_pause);
		CL22_RD_OVER_CL45(bp, phy,
			MDIO_REG_BANK_COMBO_IEEE0,
			MDIO_COMBO_IEEE0_AUTO_NEG_LINK_PARTNER_ABILITY1,
			&lp_pause);
		pause_result = (ld_pause &
				MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_MASK)>>5;
		pause_result |= (lp_pause &
				 MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_MASK)>>7;
		DP(NETIF_MSG_LINK, "pause_result CL37 0x%x\n", pause_result);
	}
	bnx2x_pause_resolve(phy, params, vars, pause_result);

}

static void bnx2x_flow_ctrl_resolve(struct bnx2x_phy *phy,
				    struct link_params *params,
				    struct link_vars *vars,
				    u32 gp_status)
{
	struct bnx2x *bp = params->bp;
	vars->flow_ctrl = BNX2X_FLOW_CTRL_NONE;

	/* Resolve from gp_status in case of AN complete and not sgmii */
	if (phy->req_flow_ctrl != BNX2X_FLOW_CTRL_AUTO) {
		/* Update the advertised flow-controled of LD/LP in AN */
		if (phy->req_line_speed == SPEED_AUTO_NEG)
			bnx2x_update_adv_fc(phy, params, vars, gp_status);
		/* But set the flow-control result as the requested one */
		vars->flow_ctrl = phy->req_flow_ctrl;
	} else if (phy->req_line_speed != SPEED_AUTO_NEG)
		vars->flow_ctrl = params->req_fc_auto_adv;
	else if ((gp_status & MDIO_AN_CL73_OR_37_COMPLETE) &&
		 (!(vars->phy_flags & PHY_SGMII_FLAG))) {
		if (bnx2x_direct_parallel_detect_used(phy, params)) {
			vars->flow_ctrl = params->req_fc_auto_adv;
			return;
		}
		bnx2x_update_adv_fc(phy, params, vars, gp_status);
	}
	DP(NETIF_MSG_LINK, "flow_ctrl 0x%x\n", vars->flow_ctrl);
}

static void bnx2x_check_fallback_to_cl37(struct bnx2x_phy *phy,
					 struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	u16 rx_status, ustat_val, cl37_fsm_received;
	DP(NETIF_MSG_LINK, "bnx2x_check_fallback_to_cl37\n");
	/* Step 1: Make sure signal is detected */
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_RX0,
			  MDIO_RX0_RX_STATUS,
			  &rx_status);
	if ((rx_status & MDIO_RX0_RX_STATUS_SIGDET) !=
	    (MDIO_RX0_RX_STATUS_SIGDET)) {
		DP(NETIF_MSG_LINK, "Signal is not detected. Restoring CL73."
			     "rx_status(0x80b0) = 0x%x\n", rx_status);
		CL22_WR_OVER_CL45(bp, phy,
				  MDIO_REG_BANK_CL73_IEEEB0,
				  MDIO_CL73_IEEEB0_CL73_AN_CONTROL,
				  MDIO_CL73_IEEEB0_CL73_AN_CONTROL_AN_EN);
		return;
	}
	/* Step 2: Check CL73 state machine */
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_CL73_USERB0,
			  MDIO_CL73_USERB0_CL73_USTAT1,
			  &ustat_val);
	if ((ustat_val &
	     (MDIO_CL73_USERB0_CL73_USTAT1_LINK_STATUS_CHECK |
	      MDIO_CL73_USERB0_CL73_USTAT1_AN_GOOD_CHECK_BAM37)) !=
	    (MDIO_CL73_USERB0_CL73_USTAT1_LINK_STATUS_CHECK |
	      MDIO_CL73_USERB0_CL73_USTAT1_AN_GOOD_CHECK_BAM37)) {
		DP(NETIF_MSG_LINK, "CL73 state-machine is not stable. "
			     "ustat_val(0x8371) = 0x%x\n", ustat_val);
		return;
	}
	/* Step 3: Check CL37 Message Pages received to indicate LP
	 * supports only CL37
	 */
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_REMOTE_PHY,
			  MDIO_REMOTE_PHY_MISC_RX_STATUS,
			  &cl37_fsm_received);
	if ((cl37_fsm_received &
	     (MDIO_REMOTE_PHY_MISC_RX_STATUS_CL37_FSM_RECEIVED_OVER1G_MSG |
	     MDIO_REMOTE_PHY_MISC_RX_STATUS_CL37_FSM_RECEIVED_BRCM_OUI_MSG)) !=
	    (MDIO_REMOTE_PHY_MISC_RX_STATUS_CL37_FSM_RECEIVED_OVER1G_MSG |
	      MDIO_REMOTE_PHY_MISC_RX_STATUS_CL37_FSM_RECEIVED_BRCM_OUI_MSG)) {
		DP(NETIF_MSG_LINK, "No CL37 FSM were received. "
			     "misc_rx_status(0x8330) = 0x%x\n",
			 cl37_fsm_received);
		return;
	}
	/* The combined cl37/cl73 fsm state information indicating that
	 * we are connected to a device which does not support cl73, but
	 * does support cl37 BAM. In this case we disable cl73 and
	 * restart cl37 auto-neg
	 */

	/* Disable CL73 */
	CL22_WR_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_CL73_IEEEB0,
			  MDIO_CL73_IEEEB0_CL73_AN_CONTROL,
			  0);
	/* Restart CL37 autoneg */
	bnx2x_restart_autoneg(phy, params, 0);
	DP(NETIF_MSG_LINK, "Disabling CL73, and restarting CL37 autoneg\n");
}

static void bnx2x_xgxs_an_resolve(struct bnx2x_phy *phy,
				  struct link_params *params,
				  struct link_vars *vars,
				  u32 gp_status)
{
	if (gp_status & MDIO_AN_CL73_OR_37_COMPLETE)
		vars->link_status |=
			LINK_STATUS_AUTO_NEGOTIATE_COMPLETE;

	if (bnx2x_direct_parallel_detect_used(phy, params))
		vars->link_status |=
			LINK_STATUS_PARALLEL_DETECTION_USED;
}
static int bnx2x_get_link_speed_duplex(struct bnx2x_phy *phy,
				     struct link_params *params,
				      struct link_vars *vars,
				      u16 is_link_up,
				      u16 speed_mask,
				      u16 is_duplex)
{
	struct bnx2x *bp = params->bp;
	if (phy->req_line_speed == SPEED_AUTO_NEG)
		vars->link_status |= LINK_STATUS_AUTO_NEGOTIATE_ENABLED;
	if (is_link_up) {
		DP(NETIF_MSG_LINK, "phy link up\n");

		vars->phy_link_up = 1;
		vars->link_status |= LINK_STATUS_LINK_UP;

		switch (speed_mask) {
		case GP_STATUS_10M:
			vars->line_speed = SPEED_10;
			if (is_duplex == DUPLEX_FULL)
				vars->link_status |= LINK_10TFD;
			else
				vars->link_status |= LINK_10THD;
			break;

		case GP_STATUS_100M:
			vars->line_speed = SPEED_100;
			if (is_duplex == DUPLEX_FULL)
				vars->link_status |= LINK_100TXFD;
			else
				vars->link_status |= LINK_100TXHD;
			break;

		case GP_STATUS_1G:
		case GP_STATUS_1G_KX:
			vars->line_speed = SPEED_1000;
			if (is_duplex == DUPLEX_FULL)
				vars->link_status |= LINK_1000TFD;
			else
				vars->link_status |= LINK_1000THD;
			break;

		case GP_STATUS_2_5G:
			vars->line_speed = SPEED_2500;
			if (is_duplex == DUPLEX_FULL)
				vars->link_status |= LINK_2500TFD;
			else
				vars->link_status |= LINK_2500THD;
			break;

		case GP_STATUS_5G:
		case GP_STATUS_6G:
			DP(NETIF_MSG_LINK,
				 "link speed unsupported  gp_status 0x%x\n",
				  speed_mask);
			return -EINVAL;

		case GP_STATUS_10G_KX4:
		case GP_STATUS_10G_HIG:
		case GP_STATUS_10G_CX4:
		case GP_STATUS_10G_KR:
		case GP_STATUS_10G_SFI:
		case GP_STATUS_10G_XFI:
			vars->line_speed = SPEED_10000;
			vars->link_status |= LINK_10GTFD;
			break;
		case GP_STATUS_20G_DXGXS:
		case GP_STATUS_20G_KR2:
			vars->line_speed = SPEED_20000;
			vars->link_status |= LINK_20GTFD;
			break;
		default:
			DP(NETIF_MSG_LINK,
				  "link speed unsupported gp_status 0x%x\n",
				  speed_mask);
			return -EINVAL;
		}
	} else { /* link_down */
		DP(NETIF_MSG_LINK, "phy link down\n");

		vars->phy_link_up = 0;

		vars->duplex = DUPLEX_FULL;
		vars->flow_ctrl = BNX2X_FLOW_CTRL_NONE;
		vars->mac_type = MAC_TYPE_NONE;
	}
	DP(NETIF_MSG_LINK, " phy_link_up %x line_speed %d\n",
		    vars->phy_link_up, vars->line_speed);
	return 0;
}

static int bnx2x_link_settings_status(struct bnx2x_phy *phy,
				      struct link_params *params,
				      struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;

	u16 gp_status, duplex = DUPLEX_HALF, link_up = 0, speed_mask;
	int rc = 0;

	/* Read gp_status */
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_GP_STATUS,
			  MDIO_GP_STATUS_TOP_AN_STATUS1,
			  &gp_status);
	if (gp_status & MDIO_GP_STATUS_TOP_AN_STATUS1_DUPLEX_STATUS)
		duplex = DUPLEX_FULL;
	if (gp_status & MDIO_GP_STATUS_TOP_AN_STATUS1_LINK_STATUS)
		link_up = 1;
	speed_mask = gp_status & GP_STATUS_SPEED_MASK;
	DP(NETIF_MSG_LINK, "gp_status 0x%x, is_link_up %d, speed_mask 0x%x\n",
		       gp_status, link_up, speed_mask);
	rc = bnx2x_get_link_speed_duplex(phy, params, vars, link_up, speed_mask,
					 duplex);
	if (rc == -EINVAL)
		return rc;

	if (gp_status & MDIO_GP_STATUS_TOP_AN_STATUS1_LINK_STATUS) {
		if (SINGLE_MEDIA_DIRECT(params)) {
			vars->duplex = duplex;
			bnx2x_flow_ctrl_resolve(phy, params, vars, gp_status);
			if (phy->req_line_speed == SPEED_AUTO_NEG)
				bnx2x_xgxs_an_resolve(phy, params, vars,
						      gp_status);
		}
	} else { /* Link_down */
		if ((phy->req_line_speed == SPEED_AUTO_NEG) &&
		    SINGLE_MEDIA_DIRECT(params)) {
			/* Check signal is detected */
			bnx2x_check_fallback_to_cl37(phy, params);
		}
	}

	/* Read LP advertised speeds*/
	if (SINGLE_MEDIA_DIRECT(params) &&
	    (vars->link_status & LINK_STATUS_AUTO_NEGOTIATE_COMPLETE)) {
		u16 val;

		CL22_RD_OVER_CL45(bp, phy, MDIO_REG_BANK_CL73_IEEEB1,
				  MDIO_CL73_IEEEB1_AN_LP_ADV2, &val);

		if (val & MDIO_CL73_IEEEB1_AN_ADV2_ADVR_1000M_KX)
			vars->link_status |=
				LINK_STATUS_LINK_PARTNER_1000TFD_CAPABLE;
		if (val & (MDIO_CL73_IEEEB1_AN_ADV2_ADVR_10G_KX4 |
			   MDIO_CL73_IEEEB1_AN_ADV2_ADVR_10G_KR))
			vars->link_status |=
				LINK_STATUS_LINK_PARTNER_10GXFD_CAPABLE;

		CL22_RD_OVER_CL45(bp, phy, MDIO_REG_BANK_OVER_1G,
				  MDIO_OVER_1G_LP_UP1, &val);

		if (val & MDIO_OVER_1G_UP1_2_5G)
			vars->link_status |=
				LINK_STATUS_LINK_PARTNER_2500XFD_CAPABLE;
		if (val & (MDIO_OVER_1G_UP1_10G | MDIO_OVER_1G_UP1_10GH))
			vars->link_status |=
				LINK_STATUS_LINK_PARTNER_10GXFD_CAPABLE;
	}

	DP(NETIF_MSG_LINK, "duplex %x  flow_ctrl 0x%x link_status 0x%x\n",
		   vars->duplex, vars->flow_ctrl, vars->link_status);
	return rc;
}

static int bnx2x_warpcore_read_status(struct bnx2x_phy *phy,
				     struct link_params *params,
				     struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u8 lane;
	u16 gp_status1, gp_speed, link_up, duplex = DUPLEX_FULL;
	int rc = 0;
	lane = bnx2x_get_warpcore_lane(phy, params);
	/* Read gp_status */
	if ((params->loopback_mode) &&
	    (phy->flags & FLAGS_WC_DUAL_MODE)) {
		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				MDIO_WC_REG_DIGITAL5_LINK_STATUS, &link_up);
		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				MDIO_WC_REG_DIGITAL5_LINK_STATUS, &link_up);
		link_up &= 0x1;
	} else if ((phy->req_line_speed > SPEED_10000) &&
		(phy->supported & SUPPORTED_20000baseMLD2_Full)) {
		u16 temp_link_up;
		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				1, &temp_link_up);
		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				1, &link_up);
		DP(NETIF_MSG_LINK, "PCS RX link status = 0x%x-->0x%x\n",
			       temp_link_up, link_up);
		link_up &= (1<<2);
		if (link_up)
			bnx2x_ext_phy_resolve_fc(phy, params, vars);
	} else {
		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				MDIO_WC_REG_GP2_STATUS_GP_2_1,
				&gp_status1);
		DP(NETIF_MSG_LINK, "0x81d1 = 0x%x\n", gp_status1);
		/* Check for either KR, 1G, or AN up. */
		link_up = ((gp_status1 >> 8) |
			   (gp_status1 >> 12) |
			   (gp_status1)) &
			(1 << lane);
		if (phy->supported & SUPPORTED_20000baseKR2_Full) {
			u16 an_link;
			bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD,
					MDIO_AN_REG_STATUS, &an_link);
			bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD,
					MDIO_AN_REG_STATUS, &an_link);
			link_up |= (an_link & (1<<2));
		}
		if (link_up && SINGLE_MEDIA_DIRECT(params)) {
			u16 pd, gp_status4;
			if (phy->req_line_speed == SPEED_AUTO_NEG) {
				/* Check Autoneg complete */
				bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
						MDIO_WC_REG_GP2_STATUS_GP_2_4,
						&gp_status4);
				if (gp_status4 & ((1<<12)<<lane))
					vars->link_status |=
					LINK_STATUS_AUTO_NEGOTIATE_COMPLETE;

				/* Check parallel detect used */
				bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
						MDIO_WC_REG_PAR_DET_10G_STATUS,
						&pd);
				if (pd & (1<<15))
					vars->link_status |=
					LINK_STATUS_PARALLEL_DETECTION_USED;
			}
			bnx2x_ext_phy_resolve_fc(phy, params, vars);
			vars->duplex = duplex;
		}
	}

	if ((vars->link_status & LINK_STATUS_AUTO_NEGOTIATE_COMPLETE) &&
	    SINGLE_MEDIA_DIRECT(params)) {
		u16 val;

		bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD,
				MDIO_AN_REG_LP_AUTO_NEG2, &val);

		if (val & MDIO_CL73_IEEEB1_AN_ADV2_ADVR_1000M_KX)
			vars->link_status |=
				LINK_STATUS_LINK_PARTNER_1000TFD_CAPABLE;
		if (val & (MDIO_CL73_IEEEB1_AN_ADV2_ADVR_10G_KX4 |
			   MDIO_CL73_IEEEB1_AN_ADV2_ADVR_10G_KR))
			vars->link_status |=
				LINK_STATUS_LINK_PARTNER_10GXFD_CAPABLE;

		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				MDIO_WC_REG_DIGITAL3_LP_UP1, &val);

		if (val & MDIO_OVER_1G_UP1_2_5G)
			vars->link_status |=
				LINK_STATUS_LINK_PARTNER_2500XFD_CAPABLE;
		if (val & (MDIO_OVER_1G_UP1_10G | MDIO_OVER_1G_UP1_10GH))
			vars->link_status |=
				LINK_STATUS_LINK_PARTNER_10GXFD_CAPABLE;

	}


	if (lane < 2) {
		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				MDIO_WC_REG_GP2_STATUS_GP_2_2, &gp_speed);
	} else {
		bnx2x_cl45_read(bp, phy, MDIO_WC_DEVAD,
				MDIO_WC_REG_GP2_STATUS_GP_2_3, &gp_speed);
	}
	DP(NETIF_MSG_LINK, "lane %d gp_speed 0x%x\n", lane, gp_speed);

	if ((lane & 1) == 0)
		gp_speed <<= 8;
	gp_speed &= 0x3f00;
	link_up = !!link_up;

	rc = bnx2x_get_link_speed_duplex(phy, params, vars, link_up, gp_speed,
					 duplex);

	/* In case of KR link down, start up the recovering procedure */
	if ((!link_up) && (phy->media_type == ETH_PHY_KR) &&
	    (!(phy->flags & FLAGS_WC_DUAL_MODE)))
		vars->rx_tx_asic_rst = MAX_KR_LINK_RETRY;

	DP(NETIF_MSG_LINK, "duplex %x  flow_ctrl 0x%x link_status 0x%x\n",
		   vars->duplex, vars->flow_ctrl, vars->link_status);
	return rc;
}
static void bnx2x_set_gmii_tx_driver(struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	struct bnx2x_phy *phy = &params->phy[INT_PHY];
	u16 lp_up2;
	u16 tx_driver;
	u16 bank;

	/* Read precomp */
	CL22_RD_OVER_CL45(bp, phy,
			  MDIO_REG_BANK_OVER_1G,
			  MDIO_OVER_1G_LP_UP2, &lp_up2);

	/* Bits [10:7] at lp_up2, positioned at [15:12] */
	lp_up2 = (((lp_up2 & MDIO_OVER_1G_LP_UP2_PREEMPHASIS_MASK) >>
		   MDIO_OVER_1G_LP_UP2_PREEMPHASIS_SHIFT) <<
		  MDIO_TX0_TX_DRIVER_PREEMPHASIS_SHIFT);

	if (lp_up2 == 0)
		return;

	for (bank = MDIO_REG_BANK_TX0; bank <= MDIO_REG_BANK_TX3;
	      bank += (MDIO_REG_BANK_TX1 - MDIO_REG_BANK_TX0)) {
		CL22_RD_OVER_CL45(bp, phy,
				  bank,
				  MDIO_TX0_TX_DRIVER, &tx_driver);

		/* Replace tx_driver bits [15:12] */
		if (lp_up2 !=
		    (tx_driver & MDIO_TX0_TX_DRIVER_PREEMPHASIS_MASK)) {
			tx_driver &= ~MDIO_TX0_TX_DRIVER_PREEMPHASIS_MASK;
			tx_driver |= lp_up2;
			CL22_WR_OVER_CL45(bp, phy,
					  bank,
					  MDIO_TX0_TX_DRIVER, tx_driver);
		}
	}
}

static int bnx2x_emac_program(struct link_params *params,
			      struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u8 port = params->port;
	u16 mode = 0;

	DP(NETIF_MSG_LINK, "setting link speed & duplex\n");
	bnx2x_bits_dis(bp, GRCBASE_EMAC0 + port*0x400 +
		       EMAC_REG_EMAC_MODE,
		       (EMAC_MODE_25G_MODE |
			EMAC_MODE_PORT_MII_10M |
			EMAC_MODE_HALF_DUPLEX));
	switch (vars->line_speed) {
	case SPEED_10:
		mode |= EMAC_MODE_PORT_MII_10M;
		break;

	case SPEED_100:
		mode |= EMAC_MODE_PORT_MII;
		break;

	case SPEED_1000:
		mode |= EMAC_MODE_PORT_GMII;
		break;

	case SPEED_2500:
		mode |= (EMAC_MODE_25G_MODE | EMAC_MODE_PORT_GMII);
		break;

	default:
		/* 10G not valid for EMAC */
		DP(NETIF_MSG_LINK, "Invalid line_speed 0x%x\n",
			   vars->line_speed);
		return -EINVAL;
	}

	if (vars->duplex == DUPLEX_HALF)
		mode |= EMAC_MODE_HALF_DUPLEX;
	bnx2x_bits_en(bp,
		      GRCBASE_EMAC0 + port*0x400 + EMAC_REG_EMAC_MODE,
		      mode);

	bnx2x_set_led(params, vars, LED_MODE_OPER, vars->line_speed);
	return 0;
}

static void bnx2x_set_preemphasis(struct bnx2x_phy *phy,
				  struct link_params *params)
{

	u16 bank, i = 0;
	struct bnx2x *bp = params->bp;

	for (bank = MDIO_REG_BANK_RX0, i = 0; bank <= MDIO_REG_BANK_RX3;
	      bank += (MDIO_REG_BANK_RX1-MDIO_REG_BANK_RX0), i++) {
			CL22_WR_OVER_CL45(bp, phy,
					  bank,
					  MDIO_RX0_RX_EQ_BOOST,
					  phy->rx_preemphasis[i]);
	}

	for (bank = MDIO_REG_BANK_TX0, i = 0; bank <= MDIO_REG_BANK_TX3;
		      bank += (MDIO_REG_BANK_TX1 - MDIO_REG_BANK_TX0), i++) {
			CL22_WR_OVER_CL45(bp, phy,
					  bank,
					  MDIO_TX0_TX_DRIVER,
					  phy->tx_preemphasis[i]);
	}
}

static void bnx2x_xgxs_config_init(struct bnx2x_phy *phy,
				   struct link_params *params,
				   struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u8 enable_cl73 = (SINGLE_MEDIA_DIRECT(params) ||
			  (params->loopback_mode == LOOPBACK_XGXS));
	if (!(vars->phy_flags & PHY_SGMII_FLAG)) {
		if (SINGLE_MEDIA_DIRECT(params) &&
		    (params->feature_config_flags &
		     FEATURE_CONFIG_OVERRIDE_PREEMPHASIS_ENABLED))
			bnx2x_set_preemphasis(phy, params);

		/* Forced speed requested? */
		if (vars->line_speed != SPEED_AUTO_NEG ||
		    (SINGLE_MEDIA_DIRECT(params) &&
		     params->loopback_mode == LOOPBACK_EXT)) {
			DP(NETIF_MSG_LINK, "not SGMII, no AN\n");

			/* Disable autoneg */
			bnx2x_set_autoneg(phy, params, vars, 0);

			/* Program speed and duplex */
			bnx2x_program_serdes(phy, params, vars);

		} else { /* AN_mode */
			DP(NETIF_MSG_LINK, "not SGMII, AN\n");

			/* AN enabled */
			bnx2x_set_brcm_cl37_advertisement(phy, params);

			/* Program duplex & pause advertisement (for aneg) */
			bnx2x_set_ieee_aneg_advertisement(phy, params,
							  vars->ieee_fc);

			/* Enable autoneg */
			bnx2x_set_autoneg(phy, params, vars, enable_cl73);

			/* Enable and restart AN */
			bnx2x_restart_autoneg(phy, params, enable_cl73);
		}

	} else { /* SGMII mode */
		DP(NETIF_MSG_LINK, "SGMII\n");

		bnx2x_initialize_sgmii_process(phy, params, vars);
	}
}

static int bnx2x_prepare_xgxs(struct bnx2x_phy *phy,
			  struct link_params *params,
			  struct link_vars *vars)
{
	int rc;
	vars->phy_flags |= PHY_XGXS_FLAG;
	if ((phy->req_line_speed &&
	     ((phy->req_line_speed == SPEED_100) ||
	      (phy->req_line_speed == SPEED_10))) ||
	    (!phy->req_line_speed &&
	     (phy->speed_cap_mask >=
	      PORT_HW_CFG_SPEED_CAPABILITY_D0_10M_FULL) &&
	     (phy->speed_cap_mask <
	      PORT_HW_CFG_SPEED_CAPABILITY_D0_1G)) ||
	    (phy->type == PORT_HW_CFG_SERDES_EXT_PHY_TYPE_DIRECT_SD))
		vars->phy_flags |= PHY_SGMII_FLAG;
	else
		vars->phy_flags &= ~PHY_SGMII_FLAG;

	bnx2x_calc_ieee_aneg_adv(phy, params, &vars->ieee_fc);
	bnx2x_set_aer_mmd(params, phy);
	if (phy->type == PORT_HW_CFG_XGXS_EXT_PHY_TYPE_DIRECT)
		bnx2x_set_master_ln(params, phy);

	rc = bnx2x_reset_unicore(params, phy, 0);
	/* Reset the SerDes and wait for reset bit return low */
	if (rc)
		return rc;

	bnx2x_set_aer_mmd(params, phy);
	/* Setting the masterLn_def again after the reset */
	if (phy->type == PORT_HW_CFG_XGXS_EXT_PHY_TYPE_DIRECT) {
		bnx2x_set_master_ln(params, phy);
		bnx2x_set_swap_lanes(params, phy);
	}

	return rc;
}

static u16 bnx2x_wait_reset_complete(struct bnx2x *bp,
				     struct bnx2x_phy *phy,
				     struct link_params *params)
{
	u16 cnt, ctrl;
	/* Wait for soft reset to get cleared up to 1 sec */
	for (cnt = 0; cnt < 1000; cnt++) {
		if (phy->type == PORT_HW_CFG_XGXS_EXT_PHY_TYPE_BCM54618SE)
			bnx2x_cl22_read(bp, phy,
				MDIO_PMA_REG_CTRL, &ctrl);
		else
			bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_CTRL, &ctrl);
		if (!(ctrl & (1<<15)))
			break;
		usleep_range(1000, 2000);
	}

	if (cnt == 1000)
		netdev_err(bp->dev,  "Warning: PHY was not initialized,"
				      " Port %d\n",
			 params->port);
	DP(NETIF_MSG_LINK, "control reg 0x%x (after %d ms)\n", ctrl, cnt);
	return cnt;
}

static void bnx2x_link_int_enable(struct link_params *params)
{
	u8 port = params->port;
	u32 mask;
	struct bnx2x *bp = params->bp;

	/* Setting the status to report on link up for either XGXS or SerDes */
	if (CHIP_IS_E3(bp)) {
		mask = NIG_MASK_XGXS0_LINK_STATUS;
		if (!(SINGLE_MEDIA_DIRECT(params)))
			mask |= NIG_MASK_MI_INT;
	} else if (params->switch_cfg == SWITCH_CFG_10G) {
		mask = (NIG_MASK_XGXS0_LINK10G |
			NIG_MASK_XGXS0_LINK_STATUS);
		DP(NETIF_MSG_LINK, "enabled XGXS interrupt\n");
		if (!(SINGLE_MEDIA_DIRECT(params)) &&
			params->phy[INT_PHY].type !=
				PORT_HW_CFG_XGXS_EXT_PHY_TYPE_FAILURE) {
			mask |= NIG_MASK_MI_INT;
			DP(NETIF_MSG_LINK, "enabled external phy int\n");
		}

	} else { /* SerDes */
		mask = NIG_MASK_SERDES0_LINK_STATUS;
		DP(NETIF_MSG_LINK, "enabled SerDes interrupt\n");
		if (!(SINGLE_MEDIA_DIRECT(params)) &&
			params->phy[INT_PHY].type !=
				PORT_HW_CFG_SERDES_EXT_PHY_TYPE_NOT_CONN) {
			mask |= NIG_MASK_MI_INT;
			DP(NETIF_MSG_LINK, "enabled external phy int\n");
		}
	}
	bnx2x_bits_en(bp,
		      NIG_REG_MASK_INTERRUPT_PORT0 + port*4,
		      mask);

	DP(NETIF_MSG_LINK, "port %x, is_xgxs %x, int_status 0x%x\n", port,
		 (params->switch_cfg == SWITCH_CFG_10G),
		 REG_RD(bp, NIG_REG_STATUS_INTERRUPT_PORT0 + port*4));
	DP(NETIF_MSG_LINK, " int_mask 0x%x, MI_INT %x, SERDES_LINK %x\n",
		 REG_RD(bp, NIG_REG_MASK_INTERRUPT_PORT0 + port*4),
		 REG_RD(bp, NIG_REG_EMAC0_STATUS_MISC_MI_INT + port*0x18),
		 REG_RD(bp, NIG_REG_SERDES0_STATUS_LINK_STATUS+port*0x3c));
	DP(NETIF_MSG_LINK, " 10G %x, XGXS_LINK %x\n",
	   REG_RD(bp, NIG_REG_XGXS0_STATUS_LINK10G + port*0x68),
	   REG_RD(bp, NIG_REG_XGXS0_STATUS_LINK_STATUS + port*0x68));
}

static void bnx2x_rearm_latch_signal(struct bnx2x *bp, u8 port,
				     u8 exp_mi_int)
{
	u32 latch_status = 0;

	/* Disable the MI INT ( external phy int ) by writing 1 to the
	 * status register. Link down indication is high-active-signal,
	 * so in this case we need to write the status to clear the XOR
	 */
	/* Read Latched signals */
	latch_status = REG_RD(bp,
				    NIG_REG_LATCH_STATUS_0 + port*8);
	DP(NETIF_MSG_LINK, "latch_status = 0x%x\n", latch_status);
	/* Handle only those with latched-signal=up.*/
	if (exp_mi_int)
		bnx2x_bits_en(bp,
			      NIG_REG_STATUS_INTERRUPT_PORT0
			      + port*4,
			      NIG_STATUS_EMAC0_MI_INT);
	else
		bnx2x_bits_dis(bp,
			       NIG_REG_STATUS_INTERRUPT_PORT0
			       + port*4,
			       NIG_STATUS_EMAC0_MI_INT);

	if (latch_status & 1) {

		/* For all latched-signal=up : Re-Arm Latch signals */
		REG_WR(bp, NIG_REG_LATCH_STATUS_0 + port*8,
		       (latch_status & 0xfffe) | (latch_status & 1));
	}
	/* For all latched-signal=up,Write original_signal to status */
}

static void bnx2x_link_int_ack(struct link_params *params,
			       struct link_vars *vars, u8 is_10g_plus)
{
	struct bnx2x *bp = params->bp;
	u8 port = params->port;
	u32 mask;
	/* First reset all status we assume only one line will be
	 * change at a time
	 */
	bnx2x_bits_dis(bp, NIG_REG_STATUS_INTERRUPT_PORT0 + port*4,
		       (NIG_STATUS_XGXS0_LINK10G |
			NIG_STATUS_XGXS0_LINK_STATUS |
			NIG_STATUS_SERDES0_LINK_STATUS));
	if (vars->phy_link_up) {
		if (USES_WARPCORE(bp))
			mask = NIG_STATUS_XGXS0_LINK_STATUS;
		else {
			if (is_10g_plus)
				mask = NIG_STATUS_XGXS0_LINK10G;
			else if (params->switch_cfg == SWITCH_CFG_10G) {
				/* Disable the link interrupt by writing 1 to
				 * the relevant lane in the status register
				 */
				u32 ser_lane =
					((params->lane_config &
				    PORT_HW_CFG_LANE_SWAP_CFG_MASTER_MASK) >>
				    PORT_HW_CFG_LANE_SWAP_CFG_MASTER_SHIFT);
				mask = ((1 << ser_lane) <<
				       NIG_STATUS_XGXS0_LINK_STATUS_SIZE);
			} else
				mask = NIG_STATUS_SERDES0_LINK_STATUS;
		}
		DP(NETIF_MSG_LINK, "Ack link up interrupt with mask 0x%x\n",
			       mask);
		bnx2x_bits_en(bp,
			      NIG_REG_STATUS_INTERRUPT_PORT0 + port*4,
			      mask);
	}
}

static int bnx2x_null_format_ver(u32 spirom_ver, u8 *str, u16 *len)
{
	str[0] = '\0';
	(*len)--;
	return 0;
}

static int bnx2x_format_ver(u32 num, u8 *str, u16 *len)
{
	u16 ret;

	if (*len < 10) {
		/* Need more than 10chars for this format */
		bnx2x_null_format_ver(num, str, len);
		return -EINVAL;
	}

	ret = scnprintf(str, *len, "%hx.%hx", num >> 16, num);
	*len -= ret;
	return 0;
}

static int bnx2x_3_seq_format_ver(u32 num, u8 *str, u16 *len)
{
	u16 ret;

	if (*len < 10) {
		/* Need more than 10chars for this format */
		bnx2x_null_format_ver(num, str, len);
		return -EINVAL;
	}

	ret = scnprintf(str, *len, "%hhx.%hhx.%hhx", num >> 16, num >> 8, num);
	*len -= ret;
	return 0;
}

int bnx2x_get_ext_phy_fw_version(struct link_params *params, u8 *version,
				 u16 len)
{
	struct bnx2x *bp;
	u32 spirom_ver = 0;
	int status = 0;
	u8 *ver_p = version;
	u16 remain_len = len;
	if (version == NULL || params == NULL)
		return -EINVAL;
	bp = params->bp;

	/* Extract first external phy*/
	version[0] = '\0';
	spirom_ver = REG_RD(bp, params->phy[EXT_PHY1].ver_addr);

	if (params->phy[EXT_PHY1].format_fw_ver) {
		status |= params->phy[EXT_PHY1].format_fw_ver(spirom_ver,
							      ver_p,
							      &remain_len);
		ver_p += (len - remain_len);
	}
	if ((params->num_phys == MAX_PHYS) &&
	    (params->phy[EXT_PHY2].ver_addr != 0)) {
		spirom_ver = REG_RD(bp, params->phy[EXT_PHY2].ver_addr);
		if (params->phy[EXT_PHY2].format_fw_ver) {
			*ver_p = '/';
			ver_p++;
			remain_len--;
			status |= params->phy[EXT_PHY2].format_fw_ver(
				spirom_ver,
				ver_p,
				&remain_len);
			ver_p = version + (len - remain_len);
		}
	}
	*ver_p = '\0';
	return status;
}

static void bnx2x_set_xgxs_loopback(struct bnx2x_phy *phy,
				    struct link_params *params)
{
	u8 port = params->port;
	struct bnx2x *bp = params->bp;

	if (phy->req_line_speed != SPEED_1000) {
		u32 md_devad = 0;

		DP(NETIF_MSG_LINK, "XGXS 10G loopback enable\n");

		if (!CHIP_IS_E3(bp)) {
			/* Change the uni_phy_addr in the nig */
			md_devad = REG_RD(bp, (NIG_REG_XGXS0_CTRL_MD_DEVAD +
					       port*0x18));

			REG_WR(bp, NIG_REG_XGXS0_CTRL_MD_DEVAD + port*0x18,
			       0x5);
		}

		bnx2x_cl45_write(bp, phy,
				 5,
				 (MDIO_REG_BANK_AER_BLOCK +
				  (MDIO_AER_BLOCK_AER_REG & 0xf)),
				 0x2800);

		bnx2x_cl45_write(bp, phy,
				 5,
				 (MDIO_REG_BANK_CL73_IEEEB0 +
				  (MDIO_CL73_IEEEB0_CL73_AN_CONTROL & 0xf)),
				 0x6041);
		msleep(200);
		/* Set aer mmd back */
		bnx2x_set_aer_mmd(params, phy);

		if (!CHIP_IS_E3(bp)) {
			/* And md_devad */
			REG_WR(bp, NIG_REG_XGXS0_CTRL_MD_DEVAD + port*0x18,
			       md_devad);
		}
	} else {
		u16 mii_ctrl;
		DP(NETIF_MSG_LINK, "XGXS 1G loopback enable\n");
		bnx2x_cl45_read(bp, phy, 5,
				(MDIO_REG_BANK_COMBO_IEEE0 +
				(MDIO_COMBO_IEEE0_MII_CONTROL & 0xf)),
				&mii_ctrl);
		bnx2x_cl45_write(bp, phy, 5,
				 (MDIO_REG_BANK_COMBO_IEEE0 +
				 (MDIO_COMBO_IEEE0_MII_CONTROL & 0xf)),
				 mii_ctrl |
				 MDIO_COMBO_IEEO_MII_CONTROL_LOOPBACK);
	}
}

int bnx2x_set_led(struct link_params *params,
		  struct link_vars *vars, u8 mode, u32 speed)
{
	u8 port = params->port;
	u16 hw_led_mode = params->hw_led_mode;
	int rc = 0;
	u8 phy_idx;
	u32 tmp;
	u32 emac_base = port ? GRCBASE_EMAC1 : GRCBASE_EMAC0;
	struct bnx2x *bp = params->bp;
	DP(NETIF_MSG_LINK, "bnx2x_set_led: port %x, mode %d\n", port, mode);
	DP(NETIF_MSG_LINK, "speed 0x%x, hw_led_mode 0x%x\n",
		 speed, hw_led_mode);
	/* In case */
	for (phy_idx = EXT_PHY1; phy_idx < MAX_PHYS; phy_idx++) {
		if (params->phy[phy_idx].set_link_led) {
			params->phy[phy_idx].set_link_led(
				&params->phy[phy_idx], params, mode);
		}
	}

	switch (mode) {
	case LED_MODE_FRONT_PANEL_OFF:
	case LED_MODE_OFF:
		REG_WR(bp, NIG_REG_LED_10G_P0 + port*4, 0);
		REG_WR(bp, NIG_REG_LED_MODE_P0 + port*4,
		       SHARED_HW_CFG_LED_MAC1);

		tmp = EMAC_RD(bp, EMAC_REG_EMAC_LED);
		if (params->phy[EXT_PHY1].type ==
			PORT_HW_CFG_XGXS_EXT_PHY_TYPE_BCM54618SE)
			tmp &= ~(EMAC_LED_1000MB_OVERRIDE |
				EMAC_LED_100MB_OVERRIDE |
				EMAC_LED_10MB_OVERRIDE);
		else
			tmp |= EMAC_LED_OVERRIDE;

		EMAC_WR(bp, EMAC_REG_EMAC_LED, tmp);
		break;

	case LED_MODE_OPER:
		/* For all other phys, OPER mode is same as ON, so in case
		 * link is down, do nothing
		 */
		if (!vars->link_up)
			break;
	case LED_MODE_ON:
		if (((params->phy[EXT_PHY1].type ==
			  PORT_HW_CFG_XGXS_EXT_PHY_TYPE_BCM8727) ||
			 (params->phy[EXT_PHY1].type ==
			  PORT_HW_CFG_XGXS_EXT_PHY_TYPE_BCM8722)) &&
		    CHIP_IS_E2(bp) && params->num_phys == 2) {
			/* This is a work-around for E2+8727 Configurations */
			if (mode == LED_MODE_ON ||
				speed == SPEED_10000){
				REG_WR(bp, NIG_REG_LED_MODE_P0 + port*4, 0);
				REG_WR(bp, NIG_REG_LED_10G_P0 + port*4, 1);

				tmp = EMAC_RD(bp, EMAC_REG_EMAC_LED);
				EMAC_WR(bp, EMAC_REG_EMAC_LED,
					(tmp | EMAC_LED_OVERRIDE));
				/* Return here without enabling traffic
				 * LED blink and setting rate in ON mode.
				 * In oper mode, enabling LED blink
				 * and setting rate is needed.
				 */
				if (mode == LED_MODE_ON)
					return rc;
			}
		} else if (SINGLE_MEDIA_DIRECT(params)) {
			/* This is a work-around for HW issue found when link
			 * is up in CL73
			 */
			if ((!CHIP_IS_E3(bp)) ||
			    (CHIP_IS_E3(bp) &&
			     mode == LED_MODE_ON))
				REG_WR(bp, NIG_REG_LED_10G_P0 + port*4, 1);

			if (CHIP_IS_E1x(bp) ||
			    CHIP_IS_E2(bp) ||
			    (mode == LED_MODE_ON))
				REG_WR(bp, NIG_REG_LED_MODE_P0 + port*4, 0);
			else
				REG_WR(bp, NIG_REG_LED_MODE_P0 + port*4,
				       hw_led_mode);
		} else if ((params->phy[EXT_PHY1].type ==
			    PORT_HW_CFG_XGXS_EXT_PHY_TYPE_BCM54618SE) &&
			   (mode == LED_MODE_ON)) {
			REG_WR(bp, NIG_REG_LED_MODE_P0 + port*4, 0);
			tmp = EMAC_RD(bp, EMAC_REG_EMAC_LED);
			EMAC_WR(bp, EMAC_REG_EMAC_LED, tmp |
				EMAC_LED_OVERRIDE | EMAC_LED_1000MB_OVERRIDE);
			/* Break here; otherwise, it'll disable the
			 * intended override.
			 */
			break;
		} else {
			u32 nig_led_mode = ((params->hw_led_mode <<
					     SHARED_HW_CFG_LED_MODE_SHIFT) ==
					    SHARED_HW_CFG_LED_EXTPHY2) ?
				(SHARED_HW_CFG_LED_PHY1 >>
				 SHARED_HW_CFG_LED_MODE_SHIFT) : hw_led_mode;
			REG_WR(bp, NIG_REG_LED_MODE_P0 + port*4,
			       nig_led_mode);
		}

		REG_WR(bp, NIG_REG_LED_CONTROL_OVERRIDE_TRAFFIC_P0 + port*4, 0);
		/* Set blinking rate to ~15.9Hz */
		if (CHIP_IS_E3(bp))
			REG_WR(bp, NIG_REG_LED_CONTROL_BLINK_RATE_P0 + port*4,
			       LED_BLINK_RATE_VAL_E3);
		else
			REG_WR(bp, NIG_REG_LED_CONTROL_BLINK_RATE_P0 + port*4,
			       LED_BLINK_RATE_VAL_E1X_E2);
		REG_WR(bp, NIG_REG_LED_CONTROL_BLINK_RATE_ENA_P0 +
		       port*4, 1);
		tmp = EMAC_RD(bp, EMAC_REG_EMAC_LED);
		EMAC_WR(bp, EMAC_REG_EMAC_LED,
			(tmp & (~EMAC_LED_OVERRIDE)));

		if (CHIP_IS_E1(bp) &&
		    ((speed == SPEED_2500) ||
		     (speed == SPEED_1000) ||
		     (speed == SPEED_100) ||
		     (speed == SPEED_10))) {
			/* For speeds less than 10G LED scheme is different */
			REG_WR(bp, NIG_REG_LED_CONTROL_OVERRIDE_TRAFFIC_P0
			       + port*4, 1);
			REG_WR(bp, NIG_REG_LED_CONTROL_TRAFFIC_P0 +
			       port*4, 0);
			REG_WR(bp, NIG_REG_LED_CONTROL_BLINK_TRAFFIC_P0 +
			       port*4, 1);
		}
		break;

	default:
		rc = -EINVAL;
		DP(NETIF_MSG_LINK, "bnx2x_set_led: Invalid led mode %d\n",
			 mode);
		break;
	}
	return rc;

}

/* This function comes to reflect the actual link state read DIRECTLY from the
 * HW
 */
int bnx2x_test_link(struct link_params *params, struct link_vars *vars,
		    u8 is_serdes)
{
	struct bnx2x *bp = params->bp;
	u16 gp_status = 0, phy_index = 0;
	u8 ext_phy_link_up = 0, serdes_phy_type;
	struct link_vars temp_vars;
	struct bnx2x_phy *int_phy = &params->phy[INT_PHY];

	if (CHIP_IS_E3(bp)) {
		u16 link_up;
		if (params->req_line_speed[LINK_CONFIG_IDX(INT_PHY)]
		    > SPEED_10000) {
			/* Check 20G link */
			bnx2x_cl45_read(bp, int_phy, MDIO_WC_DEVAD,
					1, &link_up);
			bnx2x_cl45_read(bp, int_phy, MDIO_WC_DEVAD,
					1, &link_up);
			link_up &= (1<<2);
		} else {
			/* Check 10G link and below*/
			u8 lane = bnx2x_get_warpcore_lane(int_phy, params);
			bnx2x_cl45_read(bp, int_phy, MDIO_WC_DEVAD,
					MDIO_WC_REG_GP2_STATUS_GP_2_1,
					&gp_status);
			gp_status = ((gp_status >> 8) & 0xf) |
				((gp_status >> 12) & 0xf);
			link_up = gp_status & (1 << lane);
		}
		if (!link_up)
			return -ESRCH;
	} else {
		CL22_RD_OVER_CL45(bp, int_phy,
			  MDIO_REG_BANK_GP_STATUS,
			  MDIO_GP_STATUS_TOP_AN_STATUS1,
			  &gp_status);
	/* Link is up only if both local phy and external phy are up */
	if (!(gp_status & MDIO_GP_STATUS_TOP_AN_STATUS1_LINK_STATUS))
		return -ESRCH;
	}
	/* In XGXS loopback mode, do not check external PHY */
	if (params->loopback_mode == LOOPBACK_XGXS)
		return 0;

	switch (params->num_phys) {
	case 1:
		/* No external PHY */
		return 0;
	case 2:
		ext_phy_link_up = params->phy[EXT_PHY1].read_status(
			&params->phy[EXT_PHY1],
			params, &temp_vars);
		break;
	case 3: /* Dual Media */
		for (phy_index = EXT_PHY1; phy_index < params->num_phys;
		      phy_index++) {
			serdes_phy_type = ((params->phy[phy_index].media_type ==
					    ETH_PHY_SFPP_10G_FIBER) ||
					   (params->phy[phy_index].media_type ==
					    ETH_PHY_SFP_1G_FIBER) ||
					   (params->phy[phy_index].media_type ==
					    ETH_PHY_XFP_FIBER) ||
					   (params->phy[phy_index].media_type ==
					    ETH_PHY_DA_TWINAX));

			if (is_serdes != serdes_phy_type)
				continue;
			if (params->phy[phy_index].read_status) {
				ext_phy_link_up |=
					params->phy[phy_index].read_status(
						&params->phy[phy_index],
						params, &temp_vars);
			}
		}
		break;
	}
	if (ext_phy_link_up)
		return 0;
	return -ESRCH;
}

static int bnx2x_link_initialize(struct link_params *params,
				 struct link_vars *vars)
{
	u8 phy_index, non_ext_phy;
	struct bnx2x *bp = params->bp;
	/* In case of external phy existence, the line speed would be the
	 * line speed linked up by the external phy. In case it is direct
	 * only, then the line_speed during initialization will be
	 * equal to the req_line_speed
	 */
	vars->line_speed = params->phy[INT_PHY].req_line_speed;

	/* Initialize the internal phy in case this is a direct board
	 * (no external phys), or this board has external phy which requires
	 * to first.
	 */
	if (!USES_WARPCORE(bp))
		bnx2x_prepare_xgxs(&params->phy[INT_PHY], params, vars);
	/* init ext phy and enable link state int */
	non_ext_phy = (SINGLE_MEDIA_DIRECT(params) ||
		       (params->loopback_mode == LOOPBACK_XGXS));

	if (non_ext_phy ||
	    (params->phy[EXT_PHY1].flags & FLAGS_INIT_XGXS_FIRST) ||
	    (params->loopback_mode == LOOPBACK_EXT_PHY)) {
		struct bnx2x_phy *phy = &params->phy[INT_PHY];
		if (vars->line_speed == SPEED_AUTO_NEG &&
		    (CHIP_IS_E1x(bp) ||
		     CHIP_IS_E2(bp)))
			bnx2x_set_parallel_detection(phy, params);
		if (params->phy[INT_PHY].config_init)
			params->phy[INT_PHY].config_init(phy, params, vars);
	}

	/* Re-read this value in case it was changed inside config_init due to
	 * limitations of optic module
	 */
	vars->line_speed = params->phy[INT_PHY].req_line_speed;

	/* Init external phy*/
	if (non_ext_phy) {
		if (params->phy[INT_PHY].supported &
		    SUPPORTED_FIBRE)
			vars->link_status |= LINK_STATUS_SERDES_LINK;
	} else {
		for (phy_index = EXT_PHY1; phy_index < params->num_phys;
		      phy_index++) {
			/* No need to initialize second phy in case of first
			 * phy only selection. In case of second phy, we do
			 * need to initialize the first phy, since they are
			 * connected.
			 */
			if (params->phy[phy_index].supported &
			    SUPPORTED_FIBRE)
				vars->link_status |= LINK_STATUS_SERDES_LINK;

			if (phy_index == EXT_PHY2 &&
			    (bnx2x_phy_selection(params) ==
			     PORT_HW_CFG_PHY_SELECTION_FIRST_PHY)) {
				DP(NETIF_MSG_LINK,
				   "Not initializing second phy\n");
				continue;
			}
			params->phy[phy_index].config_init(
				&params->phy[phy_index],
				params, vars);
		}
	}
	/* Reset the interrupt indication after phy was initialized */
	bnx2x_bits_dis(bp, NIG_REG_STATUS_INTERRUPT_PORT0 +
		       params->port*4,
		       (NIG_STATUS_XGXS0_LINK10G |
			NIG_STATUS_XGXS0_LINK_STATUS |
			NIG_STATUS_SERDES0_LINK_STATUS |
			NIG_MASK_MI_INT));
	return 0;
}

static void bnx2x_int_link_reset(struct bnx2x_phy *phy,
				 struct link_params *params)
{
	/* Reset the SerDes/XGXS */
	REG_WR(params->bp, GRCBASE_MISC + MISC_REGISTERS_RESET_REG_3_CLEAR,
	       (0x1ff << (params->port*16)));
}

static void bnx2x_common_ext_link_reset(struct bnx2x_phy *phy,
					struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	u8 gpio_port;
	/* HW reset */
	if (CHIP_IS_E2(bp))
		gpio_port = BP_PATH(bp);
	else
		gpio_port = params->port;
	bnx2x_set_gpio(bp, MISC_REGISTERS_GPIO_1,
		       MISC_REGISTERS_GPIO_OUTPUT_LOW,
		       gpio_port);
	bnx2x_set_gpio(bp, MISC_REGISTERS_GPIO_2,
		       MISC_REGISTERS_GPIO_OUTPUT_LOW,
		       gpio_port);
	DP(NETIF_MSG_LINK, "reset external PHY\n");
}

static int bnx2x_update_link_down(struct link_params *params,
				  struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u8 port = params->port;

	DP(NETIF_MSG_LINK, "Port %x: Link is down\n", port);
	bnx2x_set_led(params, vars, LED_MODE_OFF, 0);
	vars->phy_flags &= ~PHY_PHYSICAL_LINK_FLAG;
	/* Indicate no mac active */
	vars->mac_type = MAC_TYPE_NONE;

	/* Update shared memory */
	vars->link_status &= ~LINK_UPDATE_MASK;
	vars->line_speed = 0;
	bnx2x_update_mng(params, vars->link_status);

	/* Activate nig drain */
	REG_WR(bp, NIG_REG_EGRESS_DRAIN0_MODE + port*4, 1);

	/* Disable emac */
	if (!CHIP_IS_E3(bp))
		REG_WR(bp, NIG_REG_NIG_EMAC0_EN + port*4, 0);

	usleep_range(10000, 20000);
	/* Reset BigMac/Xmac */
	if (CHIP_IS_E1x(bp) ||
	    CHIP_IS_E2(bp))
		bnx2x_set_bmac_rx(bp, params->chip_id, params->port, 0);

	if (CHIP_IS_E3(bp)) {
		/* Prevent LPI Generation by chip */
		REG_WR(bp, MISC_REG_CPMU_LP_FW_ENABLE_P0 + (params->port << 2),
		       0);
		REG_WR(bp, MISC_REG_CPMU_LP_MASK_ENT_P0 + (params->port << 2),
		       0);
		vars->eee_status &= ~(SHMEM_EEE_LP_ADV_STATUS_MASK |
				      SHMEM_EEE_ACTIVE_BIT);

		bnx2x_update_mng_eee(params, vars->eee_status);
		bnx2x_set_xmac_rxtx(params, 0);
		bnx2x_set_umac_rxtx(params, 0);
	}

	return 0;
}

static int bnx2x_update_link_up(struct link_params *params,
				struct link_vars *vars,
				u8 link_10g)
{
	struct bnx2x *bp = params->bp;
	u8 phy_idx, port = params->port;
	int rc = 0;

	vars->link_status |= (LINK_STATUS_LINK_UP |
			      LINK_STATUS_PHYSICAL_LINK_FLAG);
	vars->phy_flags |= PHY_PHYSICAL_LINK_FLAG;

	if (vars->flow_ctrl & BNX2X_FLOW_CTRL_TX)
		vars->link_status |=
			LINK_STATUS_TX_FLOW_CONTROL_ENABLED;

	if (vars->flow_ctrl & BNX2X_FLOW_CTRL_RX)
		vars->link_status |=
			LINK_STATUS_RX_FLOW_CONTROL_ENABLED;
	if (USES_WARPCORE(bp)) {
		if (link_10g) {
			if (bnx2x_xmac_enable(params, vars, 0) ==
			    -ESRCH) {
				DP(NETIF_MSG_LINK, "Found errors on XMAC\n");
				vars->link_up = 0;
				vars->phy_flags |= PHY_HALF_OPEN_CONN_FLAG;
				vars->link_status &= ~LINK_STATUS_LINK_UP;
			}
		} else
			bnx2x_umac_enable(params, vars, 0);
		bnx2x_set_led(params, vars,
			      LED_MODE_OPER, vars->line_speed);

		if ((vars->eee_status & SHMEM_EEE_ACTIVE_BIT) &&
		    (vars->eee_status & SHMEM_EEE_LPI_REQUESTED_BIT)) {
			DP(NETIF_MSG_LINK, "Enabling LPI assertion\n");
			REG_WR(bp, MISC_REG_CPMU_LP_FW_ENABLE_P0 +
			       (params->port << 2), 1);
			REG_WR(bp, MISC_REG_CPMU_LP_DR_ENABLE, 1);
			REG_WR(bp, MISC_REG_CPMU_LP_MASK_ENT_P0 +
			       (params->port << 2), 0xfc20);
		}
	}
	if ((CHIP_IS_E1x(bp) ||
	     CHIP_IS_E2(bp))) {
		if (link_10g) {
			if (bnx2x_bmac_enable(params, vars, 0, 1) ==
			    -ESRCH) {
				DP(NETIF_MSG_LINK, "Found errors on BMAC\n");
				vars->link_up = 0;
				vars->phy_flags |= PHY_HALF_OPEN_CONN_FLAG;
				vars->link_status &= ~LINK_STATUS_LINK_UP;
			}

			bnx2x_set_led(params, vars,
				      LED_MODE_OPER, SPEED_10000);
		} else {
			rc = bnx2x_emac_program(params, vars);
			bnx2x_emac_enable(params, vars, 0);

			/* AN complete? */
			if ((vars->link_status &
			     LINK_STATUS_AUTO_NEGOTIATE_COMPLETE)
			    && (!(vars->phy_flags & PHY_SGMII_FLAG)) &&
			    SINGLE_MEDIA_DIRECT(params))
				bnx2x_set_gmii_tx_driver(params);
		}
	}

	/* PBF - link up */
	if (CHIP_IS_E1x(bp))
		rc |= bnx2x_pbf_update(params, vars->flow_ctrl,
				       vars->line_speed);

	/* Disable drain */
	REG_WR(bp, NIG_REG_EGRESS_DRAIN0_MODE + port*4, 0);

	/* Update shared memory */
	bnx2x_update_mng(params, vars->link_status);
	bnx2x_update_mng_eee(params, vars->eee_status);
	/* Check remote fault */
	for (phy_idx = INT_PHY; phy_idx < MAX_PHYS; phy_idx++) {
		if (params->phy[phy_idx].flags & FLAGS_TX_ERROR_CHECK) {
			bnx2x_check_half_open_conn(params, vars, 0);
			break;
		}
	}
	msleep(20);
	return rc;
}

static void bnx2x_chng_link_count(struct link_params *params, bool clear)
{
	struct bnx2x *bp = params->bp;
	u32 addr, val;

	/* Verify the link_change_count is supported by the MFW */
	if (!(SHMEM2_HAS(bp, link_change_count)))
		return;

	addr = params->shmem2_base +
		offsetof(struct shmem2_region, link_change_count[params->port]);
	if (clear)
		val = 0;
	else
		val = REG_RD(bp, addr) + 1;
	REG_WR(bp, addr, val);
}

/* The bnx2x_link_update function should be called upon link
 * interrupt.
 * Link is considered up as follows:
 * - DIRECT_SINGLE_MEDIA - Only XGXS link (internal link) needs
 *   to be up
 * - SINGLE_MEDIA - The link between the 577xx and the external
 *   phy (XGXS) need to up as well as the external link of the
 *   phy (PHY_EXT1)
 * - DUAL_MEDIA - The link between the 577xx and the first
 *   external phy needs to be up, and at least one of the 2
 *   external phy link must be up.
 */
int bnx2x_link_update(struct link_params *params, struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	struct link_vars phy_vars[MAX_PHYS];
	u8 port = params->port;
	u8 link_10g_plus, phy_index;
	u32 prev_link_status = vars->link_status;
	u8 ext_phy_link_up = 0, cur_link_up;
	int rc = 0;
	u8 is_mi_int = 0;
	u16 ext_phy_line_speed = 0, prev_line_speed = vars->line_speed;
	u8 active_external_phy = INT_PHY;
	vars->phy_flags &= ~PHY_HALF_OPEN_CONN_FLAG;
	vars->link_status &= ~LINK_UPDATE_MASK;
	for (phy_index = INT_PHY; phy_index < params->num_phys;
	      phy_index++) {
		phy_vars[phy_index].flow_ctrl = 0;
		phy_vars[phy_index].link_status = 0;
		phy_vars[phy_index].line_speed = 0;
		phy_vars[phy_index].duplex = DUPLEX_FULL;
		phy_vars[phy_index].phy_link_up = 0;
		phy_vars[phy_index].link_up = 0;
		phy_vars[phy_index].fault_detected = 0;
		/* different consideration, since vars holds inner state */
		phy_vars[phy_index].eee_status = vars->eee_status;
	}

	if (USES_WARPCORE(bp))
		bnx2x_set_aer_mmd(params, &params->phy[INT_PHY]);

	DP(NETIF_MSG_LINK, "port %x, XGXS?%x, int_status 0x%x\n",
		 port, (vars->phy_flags & PHY_XGXS_FLAG),
		 REG_RD(bp, NIG_REG_STATUS_INTERRUPT_PORT0 + port*4));

	is_mi_int = (u8)(REG_RD(bp, NIG_REG_EMAC0_STATUS_MISC_MI_INT +
				port*0x18) > 0);
	DP(NETIF_MSG_LINK, "int_mask 0x%x MI_INT %x, SERDES_LINK %x\n",
		 REG_RD(bp, NIG_REG_MASK_INTERRUPT_PORT0 + port*4),
		 is_mi_int,
		 REG_RD(bp, NIG_REG_SERDES0_STATUS_LINK_STATUS + port*0x3c));

	DP(NETIF_MSG_LINK, " 10G %x, XGXS_LINK %x\n",
	  REG_RD(bp, NIG_REG_XGXS0_STATUS_LINK10G + port*0x68),
	  REG_RD(bp, NIG_REG_XGXS0_STATUS_LINK_STATUS + port*0x68));

	/* Disable emac */
	if (!CHIP_IS_E3(bp))
		REG_WR(bp, NIG_REG_NIG_EMAC0_EN + port*4, 0);

	/* Step 1:
	 * Check external link change only for external phys, and apply
	 * priority selection between them in case the link on both phys
	 * is up. Note that instead of the common vars, a temporary
	 * vars argument is used since each phy may have different link/
	 * speed/duplex result
	 */
	for (phy_index = EXT_PHY1; phy_index < params->num_phys;
	      phy_index++) {
		struct bnx2x_phy *phy = &params->phy[phy_index];
		if (!phy->read_status)
			continue;
		/* Read link status and params of this ext phy */
		cur_link_up = phy->read_status(phy, params,
					       &phy_vars[phy_index]);
		if (cur_link_up) {
			DP(NETIF_MSG_LINK, "phy in index %d link is up\n",
				   phy_index);
		} else {
			DP(NETIF_MSG_LINK, "phy in index %d link is down\n",
				   phy_index);
			continue;
		}

		if (!ext_phy_link_up) {
			ext_phy_link_up = 1;
			active_external_phy = phy_index;
		} else {
			switch (bnx2x_phy_selection(params)) {
			case PORT_HW_CFG_PHY_SELECTION_HARDWARE_DEFAULT:
			case PORT_HW_CFG_PHY_SELECTION_FIRST_PHY_PRIORITY:
			/* In this option, the first PHY makes sure to pass the
			 * traffic through itself only.
			 * Its not clear how to reset the link on the second phy
			 */
				active_external_phy = EXT_PHY1;
				break;
			case PORT_HW_CFG_PHY_SELECTION_SECOND_PHY_PRIORITY:
			/* In this option, the first PHY makes sure to pass the
			 * traffic through the second PHY.
			 */
				active_external_phy = EXT_PHY2;
				break;
			default:
			/* Link indication on both PHYs with the following cases
			 * is invalid:
			 * - FIRST_PHY means that second phy wasn't initialized,
			 * hence its link is expected to be down
			 * - SECOND_PHY means that first phy should not be able
			 * to link up by itself (using configuration)
			 * - DEFAULT should be overridden during initialization
			 */
				DP(NETIF_MSG_LINK, "Invalid link indication"
					   "mpc=0x%x. DISABLING LINK !!!\n",
					   params->multi_phy_config);
				ext_phy_link_up = 0;
				break;
			}
		}
	}
	prev_line_speed = vars->line_speed;
	/* Step 2:
	 * Read the status of the internal phy. In case of
	 * DIRECT_SINGLE_MEDIA board, this link is the external link,
	 * otherwise this is the link between the 577xx and the first
	 * external phy
	 */
	if (params->phy[INT_PHY].read_status)
		params->phy[INT_PHY].read_status(
			&params->phy[INT_PHY],
			params, vars);
	/* The INT_PHY flow control reside in the vars. This include the
	 * case where the speed or flow control are not set to AUTO.
	 * Otherwise, the active external phy flow control result is set
	 * to the vars. The ext_phy_line_speed is needed to check if the
	 * speed is different between the internal phy and external phy.
	 * This case may be result of intermediate link speed change.
	 */
	if (active_external_phy > INT_PHY) {
		vars->flow_ctrl = phy_vars[active_external_phy].flow_ctrl;
		/* Link speed is taken from the XGXS. AN and FC result from
		 * the external phy.
		 */
		vars->link_status |= phy_vars[active_external_phy].link_status;

		/* if active_external_phy is first PHY and link is up - disable
		 * disable TX on second external PHY
		 */
		if (active_external_phy == EXT_PHY1) {
			if (params->phy[EXT_PHY2].phy_specific_func) {
				DP(NETIF_MSG_LINK,
				   "Disabling TX on EXT_PHY2\n");
				params->phy[EXT_PHY2].phy_specific_func(
					&params->phy[EXT_PHY2],
					params, DISABLE_TX);
			}
		}

		ext_phy_line_speed = phy_vars[active_external_phy].line_speed;
		vars->duplex = phy_vars[active_external_phy].duplex;
		if (params->phy[active_external_phy].supported &
		    SUPPORTED_FIBRE)
			vars->link_status |= LINK_STATUS_SERDES_LINK;
		else
			vars->link_status &= ~LINK_STATUS_SERDES_LINK;

		vars->eee_status = phy_vars[active_external_phy].eee_status;

		DP(NETIF_MSG_LINK, "Active external phy selected: %x\n",
			   active_external_phy);
	}

	for (phy_index = EXT_PHY1; phy_index < params->num_phys;
	      phy_index++) {
		if (params->phy[phy_index].flags &
		    FLAGS_REARM_LATCH_SIGNAL) {
			bnx2x_rearm_latch_signal(bp, port,
						 phy_index ==
						 active_external_phy);
			break;
		}
	}
	DP(NETIF_MSG_LINK, "vars->flow_ctrl = 0x%x, vars->link_status = 0x%x,"
		   " ext_phy_line_speed = %d\n", vars->flow_ctrl,
		   vars->link_status, ext_phy_line_speed);
	/* Upon link speed change set the NIG into drain mode. Comes to
	 * deals with possible FIFO glitch due to clk change when speed
	 * is decreased without link down indicator
	 */

	if (vars->phy_link_up) {
		if (!(SINGLE_MEDIA_DIRECT(params)) && ext_phy_link_up &&
		    (ext_phy_line_speed != vars->line_speed)) {
			DP(NETIF_MSG_LINK, "Internal link speed %d is"
				   " different than the external"
				   " link speed %d\n", vars->line_speed,
				   ext_phy_line_speed);
			vars->phy_link_up = 0;
		} else if (prev_line_speed != vars->line_speed) {
			REG_WR(bp, NIG_REG_EGRESS_DRAIN0_MODE + params->port*4,
			       0);
			usleep_range(1000, 2000);
		}
	}

	/* Anything 10 and over uses the bmac */
	link_10g_plus = (vars->line_speed >= SPEED_10000);

	bnx2x_link_int_ack(params, vars, link_10g_plus);

	/* In case external phy link is up, and internal link is down
	 * (not initialized yet probably after link initialization, it
	 * needs to be initialized.
	 * Note that after link down-up as result of cable plug, the xgxs
	 * link would probably become up again without the need
	 * initialize it
	 */
	if (!(SINGLE_MEDIA_DIRECT(params))) {
		DP(NETIF_MSG_LINK, "ext_phy_link_up = %d, int_link_up = %d,"
			   " init_preceding = %d\n", ext_phy_link_up,
			   vars->phy_link_up,
			   params->phy[EXT_PHY1].flags &
			   FLAGS_INIT_XGXS_FIRST);
		if (!(params->phy[EXT_PHY1].flags &
		      FLAGS_INIT_XGXS_FIRST)
		    && ext_phy_link_up && !vars->phy_link_up) {
			vars->line_speed = ext_phy_line_speed;
			if (vars->line_speed < SPEED_1000)
				vars->phy_flags |= PHY_SGMII_FLAG;
			else
				vars->phy_flags &= ~PHY_SGMII_FLAG;

			if (params->phy[INT_PHY].config_init)
				params->phy[INT_PHY].config_init(
					&params->phy[INT_PHY], params,
						vars);
		}
	}
	/* Link is up only if both local phy and external phy (in case of
	 * non-direct board) are up and no fault detected on active PHY.
	 */
	vars->link_up = (vars->phy_link_up &&
			 (ext_phy_link_up ||
			  SINGLE_MEDIA_DIRECT(params)) &&
			 (phy_vars[active_external_phy].fault_detected == 0));

	/* Update the PFC configuration in case it was changed */
	if (params->feature_config_flags & FEATURE_CONFIG_PFC_ENABLED)
		vars->link_status |= LINK_STATUS_PFC_ENABLED;
	else
		vars->link_status &= ~LINK_STATUS_PFC_ENABLED;

	if (vars->link_up)
		rc = bnx2x_update_link_up(params, vars, link_10g_plus);
	else
		rc = bnx2x_update_link_down(params, vars);

	if ((prev_link_status ^ vars->link_status) & LINK_STATUS_LINK_UP)
		bnx2x_chng_link_count(params, false);

	/* Update MCP link status was changed */
	if (params->feature_config_flags & FEATURE_CONFIG_BC_SUPPORTS_AFEX)
		bnx2x_fw_command(bp, DRV_MSG_CODE_LINK_STATUS_CHANGED, 0);

	return rc;
}

/*****************************************************************************/
/*			    External Phy section			     */
/*****************************************************************************/
void bnx2x_ext_phy_hw_reset(struct bnx2x *bp, u8 port)
{
	bnx2x_set_gpio(bp, MISC_REGISTERS_GPIO_1,
		       MISC_REGISTERS_GPIO_OUTPUT_LOW, port);
	usleep_range(1000, 2000);
	bnx2x_set_gpio(bp, MISC_REGISTERS_GPIO_1,
		       MISC_REGISTERS_GPIO_OUTPUT_HIGH, port);
}

static void bnx2x_save_spirom_version(struct bnx2x *bp, u8 port,
				      u32 spirom_ver, u32 ver_addr)
{
	DP(NETIF_MSG_LINK, "FW version 0x%x:0x%x for port %d\n",
		 (u16)(spirom_ver>>16), (u16)spirom_ver, port);

	if (ver_addr)
		REG_WR(bp, ver_addr, spirom_ver);
}

static void bnx2x_save_bcm_spirom_ver(struct bnx2x *bp,
				      struct bnx2x_phy *phy,
				      u8 port)
{
	u16 fw_ver1, fw_ver2;

	bnx2x_cl45_read(bp, phy, MDIO_PMA_DEVAD,
			MDIO_PMA_REG_ROM_VER1, &fw_ver1);
	bnx2x_cl45_read(bp, phy, MDIO_PMA_DEVAD,
			MDIO_PMA_REG_ROM_VER2, &fw_ver2);
	bnx2x_save_spirom_version(bp, port, (u32)(fw_ver1<<16 | fw_ver2),
				  phy->ver_addr);
}

static void bnx2x_ext_phy_10G_an_resolve(struct bnx2x *bp,
				       struct bnx2x_phy *phy,
				       struct link_vars *vars)
{
	u16 val;
	bnx2x_cl45_read(bp, phy,
			MDIO_AN_DEVAD,
			MDIO_AN_REG_STATUS, &val);
	bnx2x_cl45_read(bp, phy,
			MDIO_AN_DEVAD,
			MDIO_AN_REG_STATUS, &val);
	if (val & (1<<5))
		vars->link_status |= LINK_STATUS_AUTO_NEGOTIATE_COMPLETE;
	if ((val & (1<<0)) == 0)
		vars->link_status |= LINK_STATUS_PARALLEL_DETECTION_USED;
}

/******************************************************************/
/*		common BCM8073/BCM8727 PHY SECTION		  */
/******************************************************************/
static void bnx2x_8073_resolve_fc(struct bnx2x_phy *phy,
				  struct link_params *params,
				  struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	if (phy->req_line_speed == SPEED_10 ||
	    phy->req_line_speed == SPEED_100) {
		vars->flow_ctrl = phy->req_flow_ctrl;
		return;
	}

	if (bnx2x_ext_phy_resolve_fc(phy, params, vars) &&
	    (vars->flow_ctrl == BNX2X_FLOW_CTRL_NONE)) {
		u16 pause_result;
		u16 ld_pause;		/* local */
		u16 lp_pause;		/* link partner */
		bnx2x_cl45_read(bp, phy,
				MDIO_AN_DEVAD,
				MDIO_AN_REG_CL37_FC_LD, &ld_pause);

		bnx2x_cl45_read(bp, phy,
				MDIO_AN_DEVAD,
				MDIO_AN_REG_CL37_FC_LP, &lp_pause);
		pause_result = (ld_pause &
				MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH) >> 5;
		pause_result |= (lp_pause &
				 MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH) >> 7;

		bnx2x_pause_resolve(phy, params, vars, pause_result);
		DP(NETIF_MSG_LINK, "Ext PHY CL37 pause result 0x%x\n",
			   pause_result);
	}
}
static int bnx2x_8073_8727_external_rom_boot(struct bnx2x *bp,
					     struct bnx2x_phy *phy,
					     u8 port)
{
	u32 count = 0;
	u16 fw_ver1, fw_msgout;
	int rc = 0;

	/* Boot port from external ROM  */
	/* EDC grst */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 MDIO_PMA_REG_GEN_CTRL,
			 0x0001);

	/* Ucode reboot and rst */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 MDIO_PMA_REG_GEN_CTRL,
			 0x008c);

	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 MDIO_PMA_REG_MISC_CTRL1, 0x0001);

	/* Reset internal microprocessor */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 MDIO_PMA_REG_GEN_CTRL,
			 MDIO_PMA_REG_GEN_CTRL_ROM_MICRO_RESET);

	/* Release srst bit */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 MDIO_PMA_REG_GEN_CTRL,
			 MDIO_PMA_REG_GEN_CTRL_ROM_RESET_INTERNAL_MP);

	/* Delay 100ms per the PHY specifications */
	msleep(100);

	/* 8073 sometimes taking longer to download */
	do {
		count++;
		if (count > 300) {
			DP(NETIF_MSG_LINK,
				 "bnx2x_8073_8727_external_rom_boot port %x:"
				 "Download failed. fw version = 0x%x\n",
				 port, fw_ver1);
			rc = -EINVAL;
			break;
		}

		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_ROM_VER1, &fw_ver1);
		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_M8051_MSGOUT_REG, &fw_msgout);

		usleep_range(1000, 2000);
	} while (fw_ver1 == 0 || fw_ver1 == 0x4321 ||
			((fw_msgout & 0xff) != 0x03 && (phy->type ==
			PORT_HW_CFG_XGXS_EXT_PHY_TYPE_BCM8073)));

	/* Clear ser_boot_ctl bit */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 MDIO_PMA_REG_MISC_CTRL1, 0x0000);
	bnx2x_save_bcm_spirom_ver(bp, phy, port);

	DP(NETIF_MSG_LINK,
		 "bnx2x_8073_8727_external_rom_boot port %x:"
		 "Download complete. fw version = 0x%x\n",
		 port, fw_ver1);

	return rc;
}

/******************************************************************/
/*			BCM8073 PHY SECTION			  */
/******************************************************************/
static int bnx2x_8073_is_snr_needed(struct bnx2x *bp, struct bnx2x_phy *phy)
{
	/* This is only required for 8073A1, version 102 only */
	u16 val;

	/* Read 8073 HW revision*/
	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD,
			MDIO_PMA_REG_8073_CHIP_REV, &val);

	if (val != 1) {
		/* No need to workaround in 8073 A1 */
		return 0;
	}

	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD,
			MDIO_PMA_REG_ROM_VER2, &val);

	/* SNR should be applied only for version 0x102 */
	if (val != 0x102)
		return 0;

	return 1;
}

static int bnx2x_8073_xaui_wa(struct bnx2x *bp, struct bnx2x_phy *phy)
{
	u16 val, cnt, cnt1 ;

	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD,
			MDIO_PMA_REG_8073_CHIP_REV, &val);

	if (val > 0) {
		/* No need to workaround in 8073 A1 */
		return 0;
	}
	/* XAUI workaround in 8073 A0: */

	/* After loading the boot ROM and restarting Autoneg, poll
	 * Dev1, Reg $C820:
	 */

	for (cnt = 0; cnt < 1000; cnt++) {
		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_8073_SPEED_LINK_STATUS,
				&val);
		  /* If bit [14] = 0 or bit [13] = 0, continue on with
		   * system initialization (XAUI work-around not required, as
		   * these bits indicate 2.5G or 1G link up).
		   */
		if (!(val & (1<<14)) || !(val & (1<<13))) {
			DP(NETIF_MSG_LINK, "XAUI work-around not required\n");
			return 0;
		} else if (!(val & (1<<15))) {
			DP(NETIF_MSG_LINK, "bit 15 went off\n");
			/* If bit 15 is 0, then poll Dev1, Reg $C841 until it's
			 * MSB (bit15) goes to 1 (indicating that the XAUI
			 * workaround has completed), then continue on with
			 * system initialization.
			 */
			for (cnt1 = 0; cnt1 < 1000; cnt1++) {
				bnx2x_cl45_read(bp, phy,
					MDIO_PMA_DEVAD,
					MDIO_PMA_REG_8073_XAUI_WA, &val);
				if (val & (1<<15)) {
					DP(NETIF_MSG_LINK,
					  "XAUI workaround has completed\n");
					return 0;
				 }
				 usleep_range(3000, 6000);
			}
			break;
		}
		usleep_range(3000, 6000);
	}
	DP(NETIF_MSG_LINK, "Warning: XAUI work-around timeout !!!\n");
	return -EINVAL;
}

static void bnx2x_807x_force_10G(struct bnx2x *bp, struct bnx2x_phy *phy)
{
	/* Force KR or KX */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD, MDIO_PMA_REG_CTRL, 0x2040);
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD, MDIO_PMA_REG_10G_CTRL2, 0x000b);
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD, MDIO_PMA_REG_BCM_CTRL, 0x0000);
	bnx2x_cl45_write(bp, phy,
			 MDIO_AN_DEVAD, MDIO_AN_REG_CTRL, 0x0000);
}

static void bnx2x_8073_set_pause_cl37(struct link_params *params,
				      struct bnx2x_phy *phy,
				      struct link_vars *vars)
{
	u16 cl37_val;
	struct bnx2x *bp = params->bp;
	bnx2x_cl45_read(bp, phy,
			MDIO_AN_DEVAD, MDIO_AN_REG_CL37_FC_LD, &cl37_val);

	cl37_val &= ~MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH;
	/* Please refer to Table 28B-3 of 802.3ab-1999 spec. */
	bnx2x_calc_ieee_aneg_adv(phy, params, &vars->ieee_fc);
	if ((vars->ieee_fc &
	    MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_SYMMETRIC) ==
	    MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_SYMMETRIC) {
		cl37_val |=  MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_SYMMETRIC;
	}
	if ((vars->ieee_fc &
	    MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_ASYMMETRIC) ==
	    MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_ASYMMETRIC) {
		cl37_val |=  MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_ASYMMETRIC;
	}
	if ((vars->ieee_fc &
	    MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH) ==
	    MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH) {
		cl37_val |= MDIO_COMBO_IEEE0_AUTO_NEG_ADV_PAUSE_BOTH;
	}
	DP(NETIF_MSG_LINK,
		 "Ext phy AN advertize cl37 0x%x\n", cl37_val);

	bnx2x_cl45_write(bp, phy,
			 MDIO_AN_DEVAD, MDIO_AN_REG_CL37_FC_LD, cl37_val);
	msleep(500);
}

static void bnx2x_8073_specific_func(struct bnx2x_phy *phy,
				     struct link_params *params,
				     u32 action)
{
	struct bnx2x *bp = params->bp;
	switch (action) {
	case PHY_INIT:
		/* Enable LASI */
		bnx2x_cl45_write(bp, phy,
				 MDIO_PMA_DEVAD, MDIO_PMA_LASI_RXCTRL, (1<<2));
		bnx2x_cl45_write(bp, phy,
				 MDIO_PMA_DEVAD, MDIO_PMA_LASI_CTRL,  0x0004);
		break;
	}
}

static int bnx2x_8073_config_init(struct bnx2x_phy *phy,
				  struct link_params *params,
				  struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u16 val = 0, tmp1;
	u8 gpio_port;
	DP(NETIF_MSG_LINK, "Init 8073\n");

	if (CHIP_IS_E2(bp))
		gpio_port = BP_PATH(bp);
	else
		gpio_port = params->port;
	/* Restore normal power mode*/
	bnx2x_set_gpio(bp, MISC_REGISTERS_GPIO_2,
		       MISC_REGISTERS_GPIO_OUTPUT_HIGH, gpio_port);

	bnx2x_set_gpio(bp, MISC_REGISTERS_GPIO_1,
		       MISC_REGISTERS_GPIO_OUTPUT_HIGH, gpio_port);

	bnx2x_8073_specific_func(phy, params, PHY_INIT);
	bnx2x_8073_set_pause_cl37(params, phy, vars);

	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_REG_M8051_MSGOUT_REG, &tmp1);

	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_LASI_RXSTAT, &tmp1);

	DP(NETIF_MSG_LINK, "Before rom RX_ALARM(port1): 0x%x\n", tmp1);

	/* Swap polarity if required - Must be done only in non-1G mode */
	if (params->lane_config & PORT_HW_CFG_SWAP_PHY_POLARITY_ENABLED) {
		/* Configure the 8073 to swap _P and _N of the KR lines */
		DP(NETIF_MSG_LINK, "Swapping polarity for the 8073\n");
		/* 10G Rx/Tx and 1G Tx signal polarity swap */
		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_8073_OPT_DIGITAL_CTRL, &val);
		bnx2x_cl45_write(bp, phy,
				 MDIO_PMA_DEVAD,
				 MDIO_PMA_REG_8073_OPT_DIGITAL_CTRL,
				 (val | (3<<9)));
	}


	/* Enable CL37 BAM */
	if (REG_RD(bp, params->shmem_base +
			 offsetof(struct shmem_region, dev_info.
				  port_hw_config[params->port].default_cfg)) &
	    PORT_HW_CFG_ENABLE_BAM_ON_KR_ENABLED) {

		bnx2x_cl45_read(bp, phy,
				MDIO_AN_DEVAD,
				MDIO_AN_REG_8073_BAM, &val);
		bnx2x_cl45_write(bp, phy,
				 MDIO_AN_DEVAD,
				 MDIO_AN_REG_8073_BAM, val | 1);
		DP(NETIF_MSG_LINK, "Enable CL37 BAM on KR\n");
	}
	if (params->loopback_mode == LOOPBACK_EXT) {
		bnx2x_807x_force_10G(bp, phy);
		DP(NETIF_MSG_LINK, "Forced speed 10G on 807X\n");
		return 0;
	} else {
		bnx2x_cl45_write(bp, phy,
				 MDIO_PMA_DEVAD, MDIO_PMA_REG_BCM_CTRL, 0x0002);
	}
	if (phy->req_line_speed != SPEED_AUTO_NEG) {
		if (phy->req_line_speed == SPEED_10000) {
			val = (1<<7);
		} else if (phy->req_line_speed ==  SPEED_2500) {
			val = (1<<5);
			/* Note that 2.5G works only when used with 1G
			 * advertisement
			 */
		} else
			val = (1<<5);
	} else {
		val = 0;
		if (phy->speed_cap_mask &
			PORT_HW_CFG_SPEED_CAPABILITY_D0_10G)
			val |= (1<<7);

		/* Note that 2.5G works only when used with 1G advertisement */
		if (phy->speed_cap_mask &
			(PORT_HW_CFG_SPEED_CAPABILITY_D0_1G |
			 PORT_HW_CFG_SPEED_CAPABILITY_D0_2_5G))
			val |= (1<<5);
		DP(NETIF_MSG_LINK, "807x autoneg val = 0x%x\n", val);
	}

	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_ADV, val);
	bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_8073_2_5G, &tmp1);

	if (((phy->speed_cap_mask & PORT_HW_CFG_SPEED_CAPABILITY_D0_2_5G) &&
	     (phy->req_line_speed == SPEED_AUTO_NEG)) ||
	    (phy->req_line_speed == SPEED_2500)) {
		u16 phy_ver;
		/* Allow 2.5G for A1 and above */
		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD, MDIO_PMA_REG_8073_CHIP_REV,
				&phy_ver);
		DP(NETIF_MSG_LINK, "Add 2.5G\n");
		if (phy_ver > 0)
			tmp1 |= 1;
		else
			tmp1 &= 0xfffe;
	} else {
		DP(NETIF_MSG_LINK, "Disable 2.5G\n");
		tmp1 &= 0xfffe;
	}

	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_8073_2_5G, tmp1);
	/* Add support for CL37 (passive mode) II */

	bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_CL37_FC_LD, &tmp1);
	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_CL37_FC_LD,
			 (tmp1 | ((phy->req_duplex == DUPLEX_FULL) ?
				  0x20 : 0x40)));

	/* Add support for CL37 (passive mode) III */
	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_CL37_AN, 0x1000);

	/* The SNR will improve about 2db by changing BW and FEE main
	 * tap. Rest commands are executed after link is up
	 * Change FFE main cursor to 5 in EDC register
	 */
	if (bnx2x_8073_is_snr_needed(bp, phy))
		bnx2x_cl45_write(bp, phy,
				 MDIO_PMA_DEVAD, MDIO_PMA_REG_EDC_FFE_MAIN,
				 0xFB0C);

	/* Enable FEC (Forware Error Correction) Request in the AN */
	bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_ADV2, &tmp1);
	tmp1 |= (1<<15);
	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_ADV2, tmp1);

	bnx2x_ext_phy_set_pause(params, phy, vars);

	/* Restart autoneg */
	msleep(500);
	bnx2x_cl45_write(bp, phy, MDIO_AN_DEVAD, MDIO_AN_REG_CTRL, 0x1200);
	DP(NETIF_MSG_LINK, "807x Autoneg Restart: Advertise 1G=%x, 10G=%x\n",
		   ((val & (1<<5)) > 0), ((val & (1<<7)) > 0));
	return 0;
}

static u8 bnx2x_8073_read_status(struct bnx2x_phy *phy,
				 struct link_params *params,
				 struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	u8 link_up = 0;
	u16 val1, val2;
	u16 link_status = 0;
	u16 an1000_status = 0;

	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_LASI_STAT, &val1);

	DP(NETIF_MSG_LINK, "8703 LASI status 0x%x\n", val1);

	/* Clear the interrupt LASI status register */
	bnx2x_cl45_read(bp, phy,
			MDIO_PCS_DEVAD, MDIO_PCS_REG_STATUS, &val2);
	bnx2x_cl45_read(bp, phy,
			MDIO_PCS_DEVAD, MDIO_PCS_REG_STATUS, &val1);
	DP(NETIF_MSG_LINK, "807x PCS status 0x%x->0x%x\n", val2, val1);
	/* Clear MSG-OUT */
	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_REG_M8051_MSGOUT_REG, &val1);

	/* Check the LASI */
	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_LASI_RXSTAT, &val2);

	DP(NETIF_MSG_LINK, "KR 0x9003 0x%x\n", val2);

	/* Check the link status */
	bnx2x_cl45_read(bp, phy,
			MDIO_PCS_DEVAD, MDIO_PCS_REG_STATUS, &val2);
	DP(NETIF_MSG_LINK, "KR PCS status 0x%x\n", val2);

	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_REG_STATUS, &val2);
	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_REG_STATUS, &val1);
	link_up = ((val1 & 4) == 4);
	DP(NETIF_MSG_LINK, "PMA_REG_STATUS=0x%x\n", val1);

	if (link_up &&
	     ((phy->req_line_speed != SPEED_10000))) {
		if (bnx2x_8073_xaui_wa(bp, phy) != 0)
			return 0;
	}
	bnx2x_cl45_read(bp, phy,
			MDIO_AN_DEVAD, MDIO_AN_REG_LINK_STATUS, &an1000_status);
	bnx2x_cl45_read(bp, phy,
			MDIO_AN_DEVAD, MDIO_AN_REG_LINK_STATUS, &an1000_status);

	/* Check the link status on 1.1.2 */
	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_REG_STATUS, &val2);
	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_REG_STATUS, &val1);
	DP(NETIF_MSG_LINK, "KR PMA status 0x%x->0x%x,"
		   "an_link_status=0x%x\n", val2, val1, an1000_status);

	link_up = (((val1 & 4) == 4) || (an1000_status & (1<<1)));
	if (link_up && bnx2x_8073_is_snr_needed(bp, phy)) {
		/* The SNR will improve about 2dbby changing the BW and FEE main
		 * tap. The 1st write to change FFE main tap is set before
		 * restart AN. Change PLL Bandwidth in EDC register
		 */
		bnx2x_cl45_write(bp, phy,
				 MDIO_PMA_DEVAD, MDIO_PMA_REG_PLL_BANDWIDTH,
				 0x26BC);

		/* Change CDR Bandwidth in EDC register */
		bnx2x_cl45_write(bp, phy,
				 MDIO_PMA_DEVAD, MDIO_PMA_REG_CDR_BANDWIDTH,
				 0x0333);
	}
	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_REG_8073_SPEED_LINK_STATUS,
			&link_status);

	/* Bits 0..2 --> speed detected, bits 13..15--> link is down */
	if ((link_status & (1<<2)) && (!(link_status & (1<<15)))) {
		link_up = 1;
		vars->line_speed = SPEED_10000;
		DP(NETIF_MSG_LINK, "port %x: External link up in 10G\n",
			   params->port);
	} else if ((link_status & (1<<1)) && (!(link_status & (1<<14)))) {
		link_up = 1;
		vars->line_speed = SPEED_2500;
		DP(NETIF_MSG_LINK, "port %x: External link up in 2.5G\n",
			   params->port);
	} else if ((link_status & (1<<0)) && (!(link_status & (1<<13)))) {
		link_up = 1;
		vars->line_speed = SPEED_1000;
		DP(NETIF_MSG_LINK, "port %x: External link up in 1G\n",
			   params->port);
	} else {
		link_up = 0;
		DP(NETIF_MSG_LINK, "port %x: External link is down\n",
			   params->port);
	}

	if (link_up) {
		/* Swap polarity if required */
		if (params->lane_config &
		    PORT_HW_CFG_SWAP_PHY_POLARITY_ENABLED) {
			/* Configure the 8073 to swap P and N of the KR lines */
			bnx2x_cl45_read(bp, phy,
					MDIO_XS_DEVAD,
					MDIO_XS_REG_8073_RX_CTRL_PCIE, &val1);
			/* Set bit 3 to invert Rx in 1G mode and clear this bit
			 * when it`s in 10G mode.
			 */
			if (vars->line_speed == SPEED_1000) {
				DP(NETIF_MSG_LINK, "Swapping 1G polarity for"
					      "the 8073\n");
				val1 |= (1<<3);
			} else
				val1 &= ~(1<<3);

			bnx2x_cl45_write(bp, phy,
					 MDIO_XS_DEVAD,
					 MDIO_XS_REG_8073_RX_CTRL_PCIE,
					 val1);
		}
		bnx2x_ext_phy_10G_an_resolve(bp, phy, vars);
		bnx2x_8073_resolve_fc(phy, params, vars);
		vars->duplex = DUPLEX_FULL;
	}

	if (vars->link_status & LINK_STATUS_AUTO_NEGOTIATE_COMPLETE) {
		bnx2x_cl45_read(bp, phy, MDIO_AN_DEVAD,
				MDIO_AN_REG_LP_AUTO_NEG2, &val1);

		if (val1 & (1<<5))
			vars->link_status |=
				LINK_STATUS_LINK_PARTNER_1000TFD_CAPABLE;
		if (val1 & (1<<7))
			vars->link_status |=
				LINK_STATUS_LINK_PARTNER_10GXFD_CAPABLE;
	}

	return link_up;
}

static void bnx2x_8073_link_reset(struct bnx2x_phy *phy,
				  struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	u8 gpio_port;
	if (CHIP_IS_E2(bp))
		gpio_port = BP_PATH(bp);
	else
		gpio_port = params->port;
	DP(NETIF_MSG_LINK, "Setting 8073 port %d into low power mode\n",
	   gpio_port);
	bnx2x_set_gpio(bp, MISC_REGISTERS_GPIO_2,
		       MISC_REGISTERS_GPIO_OUTPUT_LOW,
		       gpio_port);
}

/******************************************************************/
/*			BCM8705 PHY SECTION			  */
/******************************************************************/
static int bnx2x_8705_config_init(struct bnx2x_phy *phy,
				  struct link_params *params,
				  struct link_vars *vars)
{
	struct bnx2x *bp = params->bp;
	DP(NETIF_MSG_LINK, "init 8705\n");
	/* Restore normal power mode*/
	bnx2x_set_gpio(bp, MISC_REGISTERS_GPIO_2,
		       MISC_REGISTERS_GPIO_OUTPUT_HIGH, params->port);
	/* HW reset */
	bnx2x_ext_phy_hw_reset(bp, params->port);
	bnx2x_cl45_write(bp, phy, MDIO_PMA_DEVAD, MDIO_PMA_REG_CTRL, 0xa040);
	bnx2x_wait_reset_complete(bp, phy, params);

	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD, MDIO_PMA_REG_MISC_CTRL, 0x8288);
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD, MDIO_PMA_REG_PHY_IDENTIFIER, 0x7fbf);
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD, MDIO_PMA_REG_CMU_PLL_BYPASS, 0x0100);
	bnx2x_cl45_write(bp, phy,
			 MDIO_WIS_DEVAD, MDIO_WIS_REG_LASI_CNTL, 0x1);
	/* BCM8705 doesn't have microcode, hence the 0 */
	bnx2x_save_spirom_version(bp, params->port, params->shmem_base, 0);
	return 0;
}

static u8 bnx2x_8705_read_status(struct bnx2x_phy *phy,
				 struct link_params *params,
				 struct link_vars *vars)
{
	u8 link_up = 0;
	u16 val1, rx_sd;
	struct bnx2x *bp = params->bp;
	DP(NETIF_MSG_LINK, "read status 8705\n");
	bnx2x_cl45_read(bp, phy,
		      MDIO_WIS_DEVAD, MDIO_WIS_REG_LASI_STATUS, &val1);
	DP(NETIF_MSG_LINK, "8705 LASI status 0x%x\n", val1);

	bnx2x_cl45_read(bp, phy,
		      MDIO_WIS_DEVAD, MDIO_WIS_REG_LASI_STATUS, &val1);
	DP(NETIF_MSG_LINK, "8705 LASI status 0x%x\n", val1);

	bnx2x_cl45_read(bp, phy,
		      MDIO_PMA_DEVAD, MDIO_PMA_REG_RX_SD, &rx_sd);

	bnx2x_cl45_read(bp, phy,
		      MDIO_PMA_DEVAD, 0xc809, &val1);
	bnx2x_cl45_read(bp, phy,
		      MDIO_PMA_DEVAD, 0xc809, &val1);

	DP(NETIF_MSG_LINK, "8705 1.c809 val=0x%x\n", val1);
	link_up = ((rx_sd & 0x1) && (val1 & (1<<9)) && ((val1 & (1<<8)) == 0));
	if (link_up) {
		vars->line_speed = SPEED_10000;
		bnx2x_ext_phy_resolve_fc(phy, params, vars);
	}
	return link_up;
}

/******************************************************************/
/*			SFP+ module Section			  */
/******************************************************************/
static void bnx2x_set_disable_pmd_transmit(struct link_params *params,
					   struct bnx2x_phy *phy,
					   u8 pmd_dis)
{
	struct bnx2x *bp = params->bp;
	/* Disable transmitter only for bootcodes which can enable it afterwards
	 * (for D3 link)
	 */
	if (pmd_dis) {
		if (params->feature_config_flags &
		     FEATURE_CONFIG_BC_SUPPORTS_SFP_TX_DISABLED)
			DP(NETIF_MSG_LINK, "Disabling PMD transmitter\n");
		else {
			DP(NETIF_MSG_LINK, "NOT disabling PMD transmitter\n");
			return;
		}
	} else
		DP(NETIF_MSG_LINK, "Enabling PMD transmitter\n");
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 MDIO_PMA_REG_TX_DISABLE, pmd_dis);
}

static u8 bnx2x_get_gpio_port(struct link_params *params)
{
	u8 gpio_port;
	u32 swap_val, swap_override;
	struct bnx2x *bp = params->bp;
	if (CHIP_IS_E2(bp))
		gpio_port = BP_PATH(bp);
	else
		gpio_port = params->port;
	swap_val = REG_RD(bp, NIG_REG_PORT_SWAP);
	swap_override = REG_RD(bp, NIG_REG_STRAP_OVERRIDE);
	return gpio_port ^ (swap_val && swap_override);
}

static void bnx2x_sfp_e1e2_set_transmitter(struct link_params *params,
					   struct bnx2x_phy *phy,
					   u8 tx_en)
{
	u16 val;
	u8 port = params->port;
	struct bnx2x *bp = params->bp;
	u32 tx_en_mode;

	/* Disable/Enable transmitter ( TX laser of the SFP+ module.)*/
	tx_en_mode = REG_RD(bp, params->shmem_base +
			    offsetof(struct shmem_region,
				     dev_info.port_hw_config[port].sfp_ctrl)) &
		PORT_HW_CFG_TX_LASER_MASK;
	DP(NETIF_MSG_LINK, "Setting transmitter tx_en=%x for port %x "
			   "mode = %x\n", tx_en, port, tx_en_mode);
	switch (tx_en_mode) {
	case PORT_HW_CFG_TX_LASER_MDIO:

		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_PHY_IDENTIFIER,
				&val);

		if (tx_en)
			val &= ~(1<<15);
		else
			val |= (1<<15);

		bnx2x_cl45_write(bp, phy,
				 MDIO_PMA_DEVAD,
				 MDIO_PMA_REG_PHY_IDENTIFIER,
				 val);
	break;
	case PORT_HW_CFG_TX_LASER_GPIO0:
	case PORT_HW_CFG_TX_LASER_GPIO1:
	case PORT_HW_CFG_TX_LASER_GPIO2:
	case PORT_HW_CFG_TX_LASER_GPIO3:
	{
		u16 gpio_pin;
		u8 gpio_port, gpio_mode;
		if (tx_en)
			gpio_mode = MISC_REGISTERS_GPIO_OUTPUT_HIGH;
		else
			gpio_mode = MISC_REGISTERS_GPIO_OUTPUT_LOW;

		gpio_pin = tx_en_mode - PORT_HW_CFG_TX_LASER_GPIO0;
		gpio_port = bnx2x_get_gpio_port(params);
		bnx2x_set_gpio(bp, gpio_pin, gpio_mode, gpio_port);
		break;
	}
	default:
		DP(NETIF_MSG_LINK, "Invalid TX_LASER_MDIO 0x%x\n", tx_en_mode);
		break;
	}
}

static void bnx2x_sfp_set_transmitter(struct link_params *params,
				      struct bnx2x_phy *phy,
				      u8 tx_en)
{
	struct bnx2x *bp = params->bp;
	DP(NETIF_MSG_LINK, "Setting SFP+ transmitter to %d\n", tx_en);
	if (CHIP_IS_E3(bp))
		bnx2x_sfp_e3_set_transmitter(params, phy, tx_en);
	else
		bnx2x_sfp_e1e2_set_transmitter(params, phy, tx_en);
}

static int bnx2x_8726_read_sfp_module_eeprom(struct bnx2x_phy *phy,
					     struct link_params *params,
					     u8 dev_addr, u16 addr, u8 byte_cnt,
					     u8 *o_buf, u8 is_init)
{
	struct bnx2x *bp = params->bp;
	u16 val = 0;
	u16 i;
	if (byte_cnt > SFP_EEPROM_PAGE_SIZE) {
		DP(NETIF_MSG_LINK,
		   "Reading from eeprom is limited to 0xf\n");
		return -EINVAL;
	}
	/* Set the read command byte count */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD, MDIO_PMA_REG_SFP_TWO_WIRE_BYTE_CNT,
			 (byte_cnt | (dev_addr << 8)));

	/* Set the read command address */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD, MDIO_PMA_REG_SFP_TWO_WIRE_MEM_ADDR,
			 addr);

	/* Activate read command */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD, MDIO_PMA_REG_SFP_TWO_WIRE_CTRL,
			 0x2c0f);

	/* Wait up to 500us for command complete status */
	for (i = 0; i < 100; i++) {
		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_SFP_TWO_WIRE_CTRL, &val);
		if ((val & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MASK) ==
		    MDIO_PMA_REG_SFP_TWO_WIRE_STATUS_COMPLETE)
			break;
		udelay(5);
	}

	if ((val & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MASK) !=
		    MDIO_PMA_REG_SFP_TWO_WIRE_STATUS_COMPLETE) {
		DP(NETIF_MSG_LINK,
			 "Got bad status 0x%x when reading from SFP+ EEPROM\n",
			 (val & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MASK));
		return -EINVAL;
	}

	/* Read the buffer */
	for (i = 0; i < byte_cnt; i++) {
		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_8726_TWO_WIRE_DATA_BUF + i, &val);
		o_buf[i] = (u8)(val & MDIO_PMA_REG_8726_TWO_WIRE_DATA_MASK);
	}

	for (i = 0; i < 100; i++) {
		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_SFP_TWO_WIRE_CTRL, &val);
		if ((val & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MASK) ==
		    MDIO_PMA_REG_SFP_TWO_WIRE_STATUS_IDLE)
			return 0;
		usleep_range(1000, 2000);
	}
	return -EINVAL;
}

static void bnx2x_warpcore_power_module(struct link_params *params,
					u8 power)
{
	u32 pin_cfg;
	struct bnx2x *bp = params->bp;

	pin_cfg = (REG_RD(bp, params->shmem_base +
			  offsetof(struct shmem_region,
			dev_info.port_hw_config[params->port].e3_sfp_ctrl)) &
			PORT_HW_CFG_E3_PWR_DIS_MASK) >>
			PORT_HW_CFG_E3_PWR_DIS_SHIFT;

	if (pin_cfg == PIN_CFG_NA)
		return;
	DP(NETIF_MSG_LINK, "Setting SFP+ module power to %d using pin cfg %d\n",
		       power, pin_cfg);
	/* Low ==> corresponding SFP+ module is powered
	 * high ==> the SFP+ module is powered down
	 */
	bnx2x_set_cfg_pin(bp, pin_cfg, power ^ 1);
}
static int bnx2x_warpcore_read_sfp_module_eeprom(struct bnx2x_phy *phy,
						 struct link_params *params,
						 u8 dev_addr,
						 u16 addr, u8 byte_cnt,
						 u8 *o_buf, u8 is_init)
{
	int rc = 0;
	u8 i, j = 0, cnt = 0;
	u32 data_array[4];
	u16 addr32;
	struct bnx2x *bp = params->bp;

	if (byte_cnt > SFP_EEPROM_PAGE_SIZE) {
		DP(NETIF_MSG_LINK,
		   "Reading from eeprom is limited to 16 bytes\n");
		return -EINVAL;
	}

	/* 4 byte aligned address */
	addr32 = addr & (~0x3);
	do {
		if ((!is_init) && (cnt == I2C_WA_PWR_ITER)) {
			bnx2x_warpcore_power_module(params, 0);
			/* Note that 100us are not enough here */
			usleep_range(1000, 2000);
			bnx2x_warpcore_power_module(params, 1);
		}
		rc = bnx2x_bsc_read(params, bp, dev_addr, addr32, 0, byte_cnt,
				    data_array);
	} while ((rc != 0) && (++cnt < I2C_WA_RETRY_CNT));

	if (rc == 0) {
		for (i = (addr - addr32); i < byte_cnt + (addr - addr32); i++) {
			o_buf[j] = *((u8 *)data_array + i);
			j++;
		}
	}

	return rc;
}

static int bnx2x_8727_read_sfp_module_eeprom(struct bnx2x_phy *phy,
					     struct link_params *params,
					     u8 dev_addr, u16 addr, u8 byte_cnt,
					     u8 *o_buf, u8 is_init)
{
	struct bnx2x *bp = params->bp;
	u16 val, i;

	if (byte_cnt > SFP_EEPROM_PAGE_SIZE) {
		DP(NETIF_MSG_LINK,
		   "Reading from eeprom is limited to 0xf\n");
		return -EINVAL;
	}

	/* Set 2-wire transfer rate of SFP+ module EEPROM
	 * to 100Khz since some DACs(direct attached cables) do
	 * not work at 400Khz.
	 */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 MDIO_PMA_REG_8727_TWO_WIRE_SLAVE_ADDR,
			 ((dev_addr << 8) | 1));

	/* Need to read from 1.8000 to clear it */
	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD,
			MDIO_PMA_REG_SFP_TWO_WIRE_CTRL,
			&val);

	/* Set the read command byte count */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 MDIO_PMA_REG_SFP_TWO_WIRE_BYTE_CNT,
			 ((byte_cnt < 2) ? 2 : byte_cnt));

	/* Set the read command address */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 MDIO_PMA_REG_SFP_TWO_WIRE_MEM_ADDR,
			 addr);
	/* Set the destination address */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 0x8004,
			 MDIO_PMA_REG_8727_TWO_WIRE_DATA_BUF);

	/* Activate read command */
	bnx2x_cl45_write(bp, phy,
			 MDIO_PMA_DEVAD,
			 MDIO_PMA_REG_SFP_TWO_WIRE_CTRL,
			 0x8002);
	/* Wait appropriate time for two-wire command to finish before
	 * polling the status register
	 */
	usleep_range(1000, 2000);

	/* Wait up to 500us for command complete status */
	for (i = 0; i < 100; i++) {
		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_SFP_TWO_WIRE_CTRL, &val);
		if ((val & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MASK) ==
		    MDIO_PMA_REG_SFP_TWO_WIRE_STATUS_COMPLETE)
			break;
		udelay(5);
	}

	if ((val & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MASK) !=
		    MDIO_PMA_REG_SFP_TWO_WIRE_STATUS_COMPLETE) {
		DP(NETIF_MSG_LINK,
			 "Got bad status 0x%x when reading from SFP+ EEPROM\n",
			 (val & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MASK));
		return -EFAULT;
	}

	/* Read the buffer */
	for (i = 0; i < byte_cnt; i++) {
		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_8727_TWO_WIRE_DATA_BUF + i, &val);
		o_buf[i] = (u8)(val & MDIO_PMA_REG_8727_TWO_WIRE_DATA_MASK);
	}

	for (i = 0; i < 100; i++) {
		bnx2x_cl45_read(bp, phy,
				MDIO_PMA_DEVAD,
				MDIO_PMA_REG_SFP_TWO_WIRE_CTRL, &val);
		if ((val & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MASK) ==
		    MDIO_PMA_REG_SFP_TWO_WIRE_STATUS_IDLE)
			return 0;
		usleep_range(1000, 2000);
	}

	return -EINVAL;
}
int bnx2x_read_sfp_module_eeprom(struct bnx2x_phy *phy,
				 struct link_params *params, u8 dev_addr,
				 u16 addr, u16 byte_cnt, u8 *o_buf)
{
	int rc = 0;
	struct bnx2x *bp = params->bp;
	u8 xfer_size;
	u8 *user_data = o_buf;
	read_sfp_module_eeprom_func_p read_func;

	if ((dev_addr != 0xa0) && (dev_addr != 0xa2)) {
		DP(NETIF_MSG_LINK, "invalid dev_addr 0x%x\n", dev_addr);
		return -EINVAL;
	}

	switch (phy->type) {
	case PORT_HW_CFG_XGXS_EXT_PHY_TYPE_BCM8726:
		read_func = bnx2x_8726_read_sfp_module_eeprom;
		break;
	case PORT_HW_CFG_XGXS_EXT_PHY_TYPE_BCM8727:
	case PORT_HW_CFG_XGXS_EXT_PHY_TYPE_BCM8722:
		read_func = bnx2x_8727_read_sfp_module_eeprom;
		break;
	case PORT_HW_CFG_XGXS_EXT_PHY_TYPE_DIRECT:
		read_func = bnx2x_warpcore_read_sfp_module_eeprom;
		break;
	default:
		return -EOPNOTSUPP;
	}

	while (!rc && (byte_cnt > 0)) {
		xfer_size = (byte_cnt > SFP_EEPROM_PAGE_SIZE) ?
			SFP_EEPROM_PAGE_SIZE : byte_cnt;
		rc = read_func(phy, params, dev_addr, addr, xfer_size,
			       user_data, 0);
		byte_cnt -= xfer_size;
		user_data += xfer_size;
		addr += xfer_size;
	}
	return rc;
}

static int bnx2x_get_edc_mode(struct bnx2x_phy *phy,
			      struct link_params *params,
			      u16 *edc_mode)
{
	struct bnx2x *bp = params->bp;
	u32 sync_offset = 0, phy_idx, media_types;
	u8 val[SFP_EEPROM_FC_TX_TECH_ADDR + 1], check_limiting_mode = 0;
	*edc_mode = EDC_MODE_LIMITING;
	phy->media_type = ETH_PHY_UNSPECIFIED;
	/* First check for copper cable */
	if (bnx2x_read_sfp_module_eeprom(phy,
					 params,
					 I2C_DEV_ADDR_A0,
					 0,
					 SFP_EEPROM_FC_TX_TECH_ADDR + 1,
					 (u8 *)val) != 0) {
		DP(NETIF_MSG_LINK, "Failed to read from SFP+ module EEPROM\n");
		return -EINVAL;
	}
	params->link_attr_sync &= ~LINK_SFP_EEPROM_COMP_CODE_MASK;
	params->link_attr_sync |= val[SFP_EEPROM_10G_COMP_CODE_ADDR] <<
		LINK_SFP_EEPROM_COMP_CODE_SHIFT;
	bnx2x_update_link_attr(params, params->link_attr_sync);
	switch (val[SFP_EEPROM_CON_TYPE_ADDR]) {
	case SFP_EEPROM_CON_TYPE_VAL_COPPER:
	{
		u8 copper_module_type;
		phy->media_type = ETH_PHY_DA_TWINAX;
		/* Check if its active cable (includes SFP+ module)
		 * of passive cable
		 */
		copper_module_type = val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (copper_module_type &
		    SFP_EEPROM_FC_TX_TECH_BITMASK_COPPER_ACTIVE) {
			DP(NETIF_MSG_LINK, "Active Copper cable detected\n");
			if (phy->type == PORT_HW_CFG_XGXS_EXT_PHY_TYPE_DIRECT)
				*edc_mode = EDC_MODE_ACTIVE_DAC;
			else
				check_limiting_mode = 1;
		} else {
			*edc_mode = EDC_MODE_PASSIVE_DAC;
			/* Even in case PASSIVE_DAC indication is not set,
			 * treat it as a passive DAC cable, since some cables
			 * don't have this indication.
			 */
			if (copper_module_type &
			    SFP_EEPROM_FC_TX_TECH_BITMASK_COPPER_PASSIVE) {
				DP(NETIF_MSG_LINK,
				   "Passive Copper cable detected\n");
			} else {
				DP(NETIF_MSG_LINK,
				   "Unknown copper-cable-type\n");
			}
		}
		break;
	}
	case SFP_EEPROM_CON_TYPE_VAL_UNKNOWN:
	case SFP_EEPROM_CON_TYPE_VAL_LC:
	case SFP_EEPROM_CON_TYPE_VAL_RJ45:
		check_limiting_mode = 1;
		if (((val[SFP_EEPROM_10G_COMP_CODE_ADDR] &
		     (SFP_EEPROM_10G_COMP_CODE_SR_MASK |
		      SFP_EEPROM_10G_COMP_CODE_LR_MASK |
		       SFP_EEPROM_10G_COMP_CODE_LRM_MASK)) == 0) &&
		    (val[SFP_EEPROM_1G_COMP_CODE_ADDR] != 0)) {
			DP(NETIF_MSG_LINK, "1G SFP module detected\n");
			phy->media_type = ETH_PHY_SFP_1G_FIBER;
			if (phy->req_line_speed != SPEED_1000) {
				u8 gport = params->port;
				phy->req_line_speed = SPEED_1000;
				if (!CHIP_IS_E1x(bp)) {
					gport = BP_PATH(bp) +
					(params->port << 1);
				}
				netdev_err(bp->dev,
					   "Warning: Link speed was forced to 1000Mbps. Current SFP module in port %d is not compliant with 10G Ethernet\n",
					   gport);
			}
			if (val[SFP_EEPROM_1G_COMP_CODE_ADDR] &
			    SFP_EEPROM_1G_COMP_CODE_BASE_T) {
				bnx2x_sfp_set_transmitter(params, phy, 0);
				msleep(40);
				bnx2x_sfp_set_transmitter(params, phy, 1);
			}
		} else {
			int idx, cfg_idx = 0;
			DP(NETIF_MSG_LINK, "10G Optic module detected\n");
			for (idx = INT_PHY; idx < MAX_PHYS; idx++) {
				if (params->phy[idx].type == phy->type) {
					cfg_idx = LINK_CONFIG_IDX(idx);
					break;
				}
			}
			phy->media_type = ETH_PHY_SFPP_10G_FIBER;
			phy->req_line_speed = params->req_line_speed[cfg_idx];
		}
		break;
	default:
		DP(NETIF_MSG_LINK, "Unable to determine module type 0x%x !!!\n",
			 val[SFP_EEPROM_CON_TYPE_ADDR]);
		return -EINVAL;
	}
	sync_offset = params->shmem_base +
		offsetof(struct shmem_region,
			 dev_info.port_hw_config[params->port].media_type);
	media_types = REG_RD(bp, sync_offset);
	/* Update media type for non-PMF sync */
	for (phy_idx = INT_PHY; phy_idx < MAX_PHYS; phy_idx++) {
		if (&(params->phy[phy_idx]) == phy) {
			media_types &= ~(PORT_HW_CFG_MEDIA_TYPE_PHY0_MASK <<
				(PORT_HW_CFG_MEDIA_TYPE_PHY1_SHIFT * phy_idx));
			media_types |= ((phy->media_type &
					PORT_HW_CFG_MEDIA_TYPE_PHY0_MASK) <<
				(PORT_HW_CFG_MEDIA_TYPE_PHY1_SHIFT * phy_idx));
			break;
		}
	}
	REG_WR(bp, sync_offset, media_types);
	if (check_limiting_mode) {
		u8 options[SFP_EEPROM_OPTIONS_SIZE];
		if (bnx2x_read_sfp_module_eeprom(phy,
						 params,
						 I2C_DEV_ADDR_A0,
						 SFP_EEPROM_OPTIONS_ADDR,
						 SFP_EEPROM_OPTIONS_SIZE,
						 options) != 0) {
			DP(NETIF_MSG_LINK,
			   "Failed to read Option field from module EEPROM\n");
			return -EINVAL;
		}
		if ((options[0] & SFP_EEPROM_OPTIONS_LINEAR_RX_OUT_MASK))
			*edc_mode = EDC_MODE_LINEAR;
		else
			*edc_mode = EDC_MODE_LIMITING;
	}
	DP(NETIF_MSG_LINK, "EDC mode is set to 0x%x\n", *edc_mode);
	return 0;
}
/* This function read the relevant field from the module (SFP+), and verify it
 * is compliant with this board
 */
static int bnx2x_verify_sfp_module(struct bnx2x_phy *phy,
				   struct link_params *params)
{
	struct bnx2x *bp = params->bp;
	u32 val, cmd;
	u32 fw_resp, fw_cmd_param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	char vendor_pn[SFP_EEPROM_PART_NO_SIZE+1];
	phy->flags &= ~FLAGS_SFP_NOT_APPROVED;
	val = REG_RD(bp, params->shmem_base +
			 offsetof(struct shmem_region, dev_info.
				  port_feature_config[params->port].config));
	if ((val & PORT_FEAT_CFG_OPT_MDL_ENFRCMNT_MASK) ==
	    PORT_FEAT_CFG_OPT_MDL_ENFRCMNT_NO_ENFORCEMENT) {
		DP(NETIF_MSG_LINK, "NOT enforcing module verification\n");
		return 0;
	}

	if (params->feature_config_flags &
	    FEATURE_CONFIG_BC_SUPPORTS_DUAL_PHY_OPT_MDL_VRFY) {
		/* Use specific phy request */
		cmd = DRV_MSG_CODE_VRFY_SPECIFIC_PHY_OPT_MDL;
	} else if (params->feature_config_flags &
		   FEATURE_CONFIG_BC_SUPPORTS_OPT_MDL_VRFY) {
		/* Use first phy request only in case of non-dual media*/
		if (DUAL_MEDIA(params)) {
			DP(NETIF_MSG_LINK,
			   "FW does not support OPT MDL verification\n");
			return -EINVAL;
		}
		cmd = DRV_MSG_CODE_VRFY_FIRST_PHY_OPT_MDL;
	} else {
		/* No support in OPT MDL detection */
		DP(NETIF_MSG_LINK,
		   "FW does not support OPT MDL verification\n");
		return -EINVAL;
	}

	fw_cmd_param = FW_PARAM_SET(phy->addr, phy->type, phy->mdio_ctrl);
	fw_resp = bnx2x_fw_command(bp, cmd, fw_cmd_param);
	if (fw_resp == FW_MSG_CODE_VRFY_OPT_MDL_SUCCESS) {
		DP(NETIF_MSG_LINK, "Approved module\n");
		return 0;
	}

	/* Format the warning message */
	if (bnx2x_read_sfp_module_eeprom(phy,
					 params,
					 I2C_DEV_ADDR_A0,
					 SFP_EEPROM_VENDOR_NAME_ADDR,
					 SFP_EEPROM_VENDOR_NAME_SIZE,
					 (u8 *)vendor_name))
		vendor_name[0] = '\0';
	else
		vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE] = '\0';
	if (bnx2x_read_sfp_module_eeprom(phy,
					 params,
					 I2C_DEV_ADDR_A0,
					 SFP_EEPROM_PART_NO_ADDR,
					 SFP_EEPROM_PART_NO_SIZE,
					 (u8 *)vendor_pn))
		vendor_pn[0] = '\0';
	else
		vendor_pn[SFP_EEPROM_PART_NO_SIZE] = '\0';

	netdev_err(bp->dev,  "Warning: Unqualified SFP+ module detected,"
			      " Port %d from %s part number %s\n",
			 params->port, vendor_name, vendor_pn);
	if ((val & PORT_FEAT_CFG_OPT_MDL_ENFRCMNT_MASK) !=
	    PORT_FEAT_CFG_OPT_MDL_ENFRCMNT_WARNING_MSG)
		phy->flags |= FLAGS_SFP_NOT_APPROVED;
	return -EINVAL;
}

static int bnx2x_wait_for_sfp_module_initialized(struct bnx2x_phy *phy,
						 struct link_params *params)

{
	u8 val;
	int rc;
	struct bnx2x *bp = params->bp;
	u16 timeout;
	/* Initialization time after hot-plug may take up to 300ms for
	 * some phys type ( e.g. JDSU )
	 */

	for (timeout = 0; timeout < 60; timeout++) {
		if (phy->type == PORT_HW_CFG_XGXS_EXT_PHY_TYPE_DIRECT)
			rc = bnx2x_warpcore_read_sfp_module_eeprom(
				phy, params, I2C_DEV_ADDR_A0, 1, 1, &val,
				1);
		else
			rc = bnx2x_read_sfp_module_eeprom(phy, params,
							  I2C_DEV_ADDR_A0,
							  1, 1, &val);
		if (rc == 0) {
			DP(NETIF_MSG_LINK,
			   "SFP+ module initialization took %d ms\n",
			   timeout * 5);
			return 0;
		}
		usleep_range(5000, 10000);
	}
	rc = bnx2x_read_sfp_module_eeprom(phy, params, I2C_DEV_ADDR_A0,
					  1, 1, &val);
	return rc;
}

static void bnx2x_8727_power_module(struct bnx2x *bp,
				    struct bnx2x_phy *phy,
				    u8 is_power_up) {
	/* Make sure GPIOs are not using for LED mode */
	u16 val;
	/* In the GPIO register, bit 4 is use to determine if the GPIOs are
	 * operating as INPUT or as OUTPUT. Bit 1 is for input, and 0 for
	 * output
	 * Bits 0-1 determine the GPIOs value for OUTPUT in case bit 4 val is 0
	 * Bits 8-9 determine the GPIOs value for INPUT in case bit 4 val is 1
	 * where the 1st bit is the over-current(only input), and 2nd bit is
	 * for power( only output )
	 *
	 * In case of NOC feature is disabled and power is up, set GPIO control
	 *  as input to enable listening of over-current indication
	 */
	if (phy->flags & FLAGS_NOC)
		return;
	if (is_power_up)
		val = (1<<4);
	else
		/* Set GPIO control to OUTPUT, and set the power bit
		 * to according to the is_power_up
 * restart AN.97;

	if IOs value for INPUT in case bit 4 val is 1
	 * where the 1st bit is the over-current(only input), and 2nd bit is
	 MDIO_PMA_REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<;/=}=f:uf:uf:uf:UD(r is up, set GPIO control
	 *  as APPROVED;
}=f:uf:gower_up)
		val = (1<<4);andicatit_for_sfp_}=f:Xnc);dule_eepro45_rA_REG}<D) =}8d 2nd bit is
	 * for po9=f:XoB:uf:Xrea;_VAL_COPPER:
	{
		u8 copper_module_type;
		phy->media_type = ETH_PHY_DA_TWINAX;
		/* Check if its active cable (includes SFP+ module)
		 * of passive cable
		 */
		copper_module_type = val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0) {
			DP(NETIF_MSG_;ADDR];

		if (TB	rc = bnx2x_warpcore:ua>ereaddr !DP(NETI1 4 val is 0
	odule_eees SFP+ modulrect attac->bp;
(_up)
		val = (1<<4);anibnx2xDwT i	ifds. S * oper_s inl is 1ve 	else
		 ETH_PHY Check iL;
	readdrPHY	va)ver-currerom(p	/* Check if itsrn -MSG_;ADDR];

		if  I2C_DEV_ADDR_l[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0)FP moADDx2x_war0val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0) {
			DP(NETIF_5)) 8val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0)x7fbf);
	0(NETIF_5)4008val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0)FP moADDx2x_war0xaaaa		u8 power)
{
0rams *params,
					     u8 f:uf:uf:uf:UD(r is up, set GPIO control
	 *  as APPROVED;
}=f:uf:gower_up)
		val = (1<<4);u8 opte
	or_nre ( e.gtus(str2_or as :Xnc);dule_eepro45_rA_REG}<D) =}8d 2nd bit is
	 * for po9=f:PIO1:
	case PORT_HW&u8 opte
	or_nrut), and 2nd bit is
	 MDIO_PMA_REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<PIO1:
	case PORT_HWmedia_tye
	or_nrand~_phy_re	      MDIO_PMA_DEVAD, 0xc809, &D) =}8d 2nd bit is
	 * for po9=f:XoB:uf:Xrea;_Vtus(str2_or ++) {
	Kwer1st bMSB-cuk_up INPUT in caseLSB-cuk_up_params ep)
		val );
	/* Wait appropriate time for two-wire command to finish before
	XoB:uf:Xrea;_ * us(str2_or _1000ff) {
| cable (inc&00) {ffe	      MDIO_PMA_ is
	 MDIO_PMA_REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<PIO1:
	case PORT_HWmedia_tye
	or_nra| _phy_re	    wer)
{
0rams *params_up) {
	/* Make _read(bp, phy,"Setting SFP+ transmitter to %d\*bp = params->bp;
	DP(NETIF_MSG_LINK,EEPRIO_PMA_IF_MSG_LINK,
		   "Reading from eeprom is lmodule_typeIO_PMA_DA_TWINAX	u32 sw_TX:
, phy, 1);
			}
		} else {
			int idx, cfg_idx eprom;
		breaAM, &vaTX:
, (bp->to OUTPUT, and set t2x_phy *phy,
			, "EDCSG_LINK, "10G Optic module detected\n");
			feprom;
		breakEVAD, M:l[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnx2offsetof(strIF_MSa| _phy5)val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnT2offsetof(st0val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnD_10000) {
6)le_type deteoAD_ABS gpe\nS, &val2);x_rnx2x_cl(NETIF_MSG_LINK, "Add 2.5G\n"	if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<<<phy,int offsetof(s	   timeoR,
				 val)2timeout *o OUTPUT, and set the poweoR,
				 3_AN_DEVAart AN.<<<<t), andspeedf (is_paIP_REFAULT;
	}

	/			 M<<<t), a0l is 1SK));
	g_flagreflD,
	0, 10000);
	<4);
	else
	ver-currerom(>to OUTPUT, and set the p
				 MDIO_P00ff8f; _GPIO_rs->lis 4-6
	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_REG_8073_SP<<<<<phy,int offsetof(sCFG_TX_LEEPROM_CON_TYPE_ADDR]);
		return -EINVALFoard
 */m_basr, phy->type_CNT M<<<SFP+ modulIO_PMA_params,
		;hy *phy,
				      u8 tx_e
	stnx2x8727_poN_DEVAlbp = paramt)
{
	struct bnx2x *bp = parapio_modeEPROM_FC_TX_TECH_ADDR + 1], check_limitiP_EEPRO_DEVAlbp_LASE_config[port].sfp_ctrl)) &
		PORT_HW_CFG_TX_LASER_MASK;
	DP(NETIF_MSG_LINK,date media type for non-PMF sync */
	fort %x "
			   "mode = %x\n		MDImoADU&vaLEDDE_SHIFTule_typeO_DEVAlbp_LASE
		break;
	case PORT_H		MDImoADU&vaLEDDOT disab:arams,
		;hyeak;
	case PORT_H		MDImoADU&vaLEDDASER_GPIO3:
	{
		u16 gpio		MDImoADU&vaLEDDASERio_port, gpio_mode;
			MDImoADU&vaLEDDASER		gpio_mode = MISC_RE		MDImoADU&vaLEDDASER3le (includbreak;
	}
	default:
		DP(NETIF_MSG_LINK, "IIGH;
		else

	dO_DEVAlbp_LASE_-INK,	case PORT_H		MDImoADU&vaLEDDASER_;(NETI1 4 val is 0
	odule_edO_DEV0000);
-);
	if ( n
	 "_MSG_LIN"se

bas_TX_LASE (incA_TWINAX;BCM8705 PHY SE\n", tx_e = MISC_REGISTEK, "Invalid TX_LASER_MDIO 0x%x\n", tx_en_mode);
		break;
	}
EEPROM_CON_TYPE_ADDR]);
		return -EINVAL &tmp: struct lO_DEV0lm(phy,eY_DA_TWINAX;BCM8705 O_DEVAlbp_LASE
;hy *phy,
				      u8 tx_e
	st3x8727_poN_DEVAlbp = paramt)
{
	struct bnx2x *bp = parpio_modeEPROM_FC_fsetof(struct smitter ( TX laser of the SFP+ module.)*/
	tx_en_mode = R_config[params->port].e3_sfp_ctrl)) &
			PORT_HX_LASER_MASK;
	DP(NETIF_MSG_LINK, "Sing transmitter tx_en=%x for p == PIN_CFG_NA)
		 SFP+ module po		MDImoDLaLEDDE_SH"Settin SFP+ module po		MDImoDLaLEDDADDR]) {TI1 4 val is 0
	odule_eees SF_DEV0eratered
	 * high ==> the SFP+ module is tx_en_moded down
	 */uct bnx2x_phy *phy,
						 struC_REGISTEK, phy,
				      u8 tx_e
	sx2x_8727_poN_DEVAlbp = paramt)
{
	struct bnx2x *bp = parapio_modeEPROM_FC_TX_TECH_ADDR + 1], check_limiti);
	/* Low ==> corresponding SFP+ module f_DEV0eratered
d verC_REGISTEK, "t_transmitter(param verificnx2x_setifwarpcore_read_sfhy->type_CoCOMPwisIO_PMA_H int bnxifwarpcore_read_sfr, poUT. Biaeeprom(pNT_WARp)
		ver-curreu8 tx_e
	st3x8727_poN_DEVAlbp  detecteC_REGISTEK, "A_DEVAD,
u8 tx_e
	stnx2x8727_poN_DEVAlbp  detecteC_REGISTEK, cfg;
	struct bnx2x *bp = param;
	bnx2x_"Setting SFP+ transmitter to %d_param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	charnge(1000, 2000);
			bnx2x_warpcore_power_ficP (iW0, 2000 forTERS_GPIO_OUTP-curr_SIZE];
		it = bnx2_WC0bnxSET000) c0e_read(bpP (iLC_PLLforTERS_GPIO_OUTP-curr_SIZE];
		it = bnx2_LC_PL_E40e */DWN");
			_SIZE];
		it = bnx2_LC_PL_E40enxSETB_ANA_power__SIZE];
		it = bnx2_LC_PL_E40enxSETB_DIG_powercfg;
	struct bnx2x *b;
			b->bp;
	u32 val, cmarams->bp;
	DP(NETIF_MSG_LIN"Setting SFP+ transmitter to %ase +
			  ofTX_TECH_ADDR + 1], check_limiti);
	/* Low ==> corresponding SFP+ is powerednd ver +
			;d_sfp_module_eeprom;
		break;
	case PORT_HW_CFG_XGXS_EXT_PHY_TYPE_		break;
	case PORT_HW_CFG_XGXS_EXT_PHY_TYPE_DIREC{
	/* Make sure GPIOs archeck_limi	bnx2x_c+
			;d_eeprom;
		break;
	default:
		return -EOPNOTSUPP;
	}

	nge(1000, 2000);
			bnx2x_warpcore_pc+
			;d_eeprom;
	N_TYPE_ADDRnx2x_phy *ph;
	struct bnx2x *bp = paramf:uf:uf:uf:UD(r is up, set)
{
	struct bnx2x *bp = paramsbyte_cnt,
					     u8 *o_buf, u8r_up)
		val = (1<<4);eeprom is limipio_port45_re bnx2_UCAD,FO_B1FW dMWAA_DEADDRD			MDIO_PTX_TECH_ADDR + 1], check_limitiP_E8 nd N
	default:
		p = paramnd Ndule(struct wer_ficthis iodulglobepr333);
	}
g_flagsif IOsspaIP nd N, "KR PCS status 0x%x\n", val2rt45_re bd bit is
	 * foe bnx2_UCAD,FO_B1FW dMWAA_DEADDx_phy *phy MDIO_PMA00ft SF(nd N
<MA_D	;d_sfp_modul)
		val =		break;
 to 0x%x\n", *e:break;
 to 0x%x\n"

		ifADDRpio_port45_re bnx2_UCAD,FO_B1FW dMWAA_DEADDRD			MDIO_P eprom;
		breaAssive DAC cable, si:break;
 to 0x%x\			/* EvenADDRpio_port45_re bnx2_UCAD,FO_B1FW dMWAA_DEADDR2x_p since eprom;
	N_TYPE_ADDRnx2x_phy *
oR,
				 pio_p SF(nd N
<MA_D	;d(bp, phy, params);

	bnx2x_cl45_e bd bit is
	rt45_re bnx2_UCAD,FO_B1FW dMWAA_DEADD, or ++) {
	A must/
stat"KR PCS status 0x%x\n", val2rt45_re bd bit is
	 * foe bnx2_UCAD,FO_B1FW dMWAA_DEADDx_phy *phRS_GPIO_OFRCMeturn 0;
OM_OPT-
static inew_OUTP-currse
			rc = bnx2x_f:uf:d Ndn", val2r1.c809 val=rc = bnx2x_f:uf:d Ndn", val2r5_reaphy,
				      u8 tx_e
	s:uf:uf:UD(r is up, set)
{
	struct bnx2x *bp = de */
	u16 val;
	/* In the GPIO regi_up)
		val = (1<fp_module_eeprom;
		break;
	case PORT_HW_CFG_XGXS_EXT_PHY_TYPE_BCM87<<<<<<;/=}=f:uf:uf:uf:UD(r ischeck_limi	bnx2x_fy it
 * is 
		read_func = bnx2x_8727_read_sfp_module_eeprom;
		break;
	case PORT_HW_CFG_XGXS_EXT_PHY_TYPE_DIREC			     u8 f:uf:uf:uf:UD(r ischeck_limi	bnx2x_fy it
 * is 
		read_func = bnx2x_8727_read_sfp_module_eeUPP;
	}

	nge(1000, 2000)f:uf:uf:uf:UD(r ischeck_	bnx2x_fy it
 * is 
		read_f}ams *params,
					   x2x_8727_poINVAL;
	},"Setting SFP+ transmitter to %d\d_param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	char _up)
		val ay take urom isar vendor_config[port].sfp_ctrl)) &
		PORT_HW_CFG_Tval & PORT_FEAT_CFG_OPT_MDL_ENFRCMNT_MASK) ==
	 
	    PORT_FEAT_CFG_OPT_MDL_ENFRCMNT_NO_ENFORCEome nion
	 _en);
	else
bya>ereaddr(struct bnx, "10G Optic module detected\n");
			;
	/* Low ==> correspFP+ module iluggeC_RE/, pa);
	if ( oMP_CODE_ASFP+ modT_MDL_ENFRCMwer_ficPs powup module (struct bn;
			b->bp;
	u32  detected\n");
			NO_ADDR,
	{
	struct bnxule(struct bn&)
		val =	arams->link_attr_sync &= ~LINK_SFP_EEPRO{
	 ruct lparams->shmfw_resp = bnx2x_fw_command first phy *bp = params->bp;
	u32 ule(struct w	arams->linype = valpFP+ module ructarabill45_
	fw_cmd_param = FW_PAR "MC_SUPPORTS_DUAL_PH f_FP_E +
		resp =uromtection */
fictnx2xPH f_DEV0000);
-);
	if ( n
	 curreu8 tx_e
	sx2x_8727_poN_DEVAlbp bnx2x *bp = paramsams->port);
	bnx2x_cl4G_TXct attac->b val[SFnEG_SFP_is powspeed2x_warpcore_readcurrerom() {
		DP(NETIF_MSG_LINK, "NOT enforcing module vparamsP(NETIF_MSG_LINK, "NOT enforcinPOWER_DOWN
			if (phy->req_line_speedShutspeedarpcore_rea +
		resp ruct bn;
			b->bp;
	u32  detected\n")0s, I2C_DEV_AD up tO_PMA_DEVA->linypetnx2xPff f_DEV0000);
-);
	if ( n
	 curreu8 tx_e
	sx2x_8727_poN_DEVAlbp bnx2x *ms->port);
	bnx2x_cl4LOWk_paramsac->b valNPUT in la_type = ETH_/		val = (1PH ;/=}. On M<<<ti the Gms->l N
automaramaIPyrom(struct bnx2x_:uf:uf:UD(r ischeck_	bnx2x_fy it
 * is
	if (pmd_dis) {
		if000);
his re_read_f_module(stru_sfr, paeeprom( INPUthe G shmemnEG_ndspebeication
	.O control t(rcted\n";
			) {
		DP(NETIF_MSG_LINK, "NOT enforcing module verifDP(NETIF_MSG_LINK, "NOT enforcin	u32 sw_TXk_paraams, phy, tx_en		}
		} else {
			int idx, cfg_idhy,
				    u8 i     u8 tx_hNPU_po8727_poINVAL;NT_params->bp;
	if (CHIP_IS_E2(bp))
EPROM_VENDOR_NAME_SIZE+1];
	char"Setting SFP+ transmi;ar venC_REGe transmiC_REGnum,(NETIF_MSG_LINK, "Setting (param veri tra= &_HW_CFG_MEDI&= ~(PO] */
ficAlwayspower_upuct shme,wiIP beication
	 support OPTreaddr(strruct bnx, "10G Optic module detected\n");
			A_DEVA->lin tra= &_HW_CFG_MEDIsfp_modtruct}		NO_ADDR,
	{
	s872_ab	DP(x_phyort].sfp_ctrlchip_id			 struct link_paramter to %d\dORT_FEAT_CFG_O&C_REGnum,(&);
		breakule veriftections->link_attr_sync &= ~LINK_SFP_EEPRO{
	 oAD_ABS S, &val2);_NO_ENfw_resp = bnx2_paramsac-S
	 ruct lparams-P_EEPff (struct bnx2x_x2x_8727_poN_DEVAlbp bnx2x *ms->port);
	bnx2x_cl4G_TXct atac-G
	 	else
		LASE_vepr33flD,
FIG_BC_SUPPiluggeC_RE_/	, p(strC_REGe t
	default:
		DP(NR_MDIO 0x%num,(NETIF_MSGct atac-CaIP moduhNPU_T;
	}oard
 */support le(stru_sf);
	if ( ontrol tC_REGe t
	rams->linuct bnx2x_OPT_Memac_			 MEDort].sfp_ctK, "Invalid TX_a		bnmde detected\n)R_l[SFP_EEP;
			b->bp;
	u32  detected\n");
			Invalid TX_LASENT_pa_MDIO 0x%num,ter to %s->port);
	bnx2x_cl4&= ~ort(parCLR,ter to %);
		break;
	}NO_ADDR,
	val;
	int rc;
	struct bnx2x *bp ule(struct w	;
			return			   x2x_8727_poINVAL;
	},ule(struct wer_	INK, "Setting (param veri"IIGH;rx_txct 2x_f:uer_	I OUTPUTport WCu_sf, paOPTx_f:u,Tx__NO_ENu * In ter to*bp;
	",
					GE_SItakT;
	REGIS* wh ((b1Gter to*bparams-Puf:u				DP(NETTIF_MSG_al & MDIO_PMA_REG_SFP_TWO_WIRE
	 * foe b = bnx2x_wa
	 * foe bnx2_DIGITAL5)x7fb6x2x_wa
	&rx_txct 2x_f:uarams->te tharx_txct 2x_f:uaed\n")");
			COPPER:
	{
		f non-dualvparamsPEVAD, MIALIZ		, 10G_FIBEse
			rc = bnx2x_f:uf:d Ndn", val2r1.c80FIBEse
			rc = bnx2 case osfi,ule(struct wer_	IBEse
			rc = bnx2x_f:uf:d Ndn", val2r(NETIF_Med[cfg_iddx = INT_PHY;
	/* Low ==> correspFP+ module _sfr, pt bnx2x *bpfw_resp _PMA_DEVA->linnvalid TX_LASENT_pa_MDIO 0x%num,ter to %s->port);
	bnx2x_cl4&= ~ort(parSET0ter to %);
		break;
	}ype odule 	   iluggeC_, p.O_PMA_(pmd_dis) {
		if000);
his re_reaver-curre];
		}
		break;
	default:
hy *PnxSENTd_f}ams  */
/******************************************************************/
Use_CNT M<06lNPUTM<<<tttttttttttttttttttttttttttttable_pmd_transmit(struct link_params *params,
					   struct bnx2x_phy *phy,
					   u8 pmd2x_8askoN_DEVre not using for LED mod"Setting SFP+ transmitter to	u8 *larm8 link_nx2x_reater to	u8 *larm8_CFGphy_idx+ (1<<4);*larm8 link_, or as :Xnc);dule_eepro45_rA_REG}<D) =}8d 2nd bit  *larm8 link_nx2x_reater &*larm8 link_)as :Xnc);dule_eepro45_rA_REG}<D) =}8d 2nd bit  *larm8 link_nx2x_reater &*larm8 link_)as ype dskr OUower_upmodureaddreve
	.t"KR PCS status 0x%x\n", val2rt45_rd 2nd bit  *larm8_CFGphy_idxx_phy *phyte t*larm8 link_hy, par0)to the iO_PMA_RE0_up
 * restR,
				 valn't have microcode, hence the rt45_rd 2nd bit  *larm8_CFGphy_idxx_<<<<<<<< */
/******************************************************************/
G_LI */s->shm6/_TYPE_BsPEV SECG;
		disable_pmd_transmit(struct link_params *params,
					   struct bnx2x_phy *phy,
		k_vars *vars6_CFG_XGXS_EXlink_up = 0;
	u16 val1, rx_sd;
	sto %d\*bp = params->bp;
	DP(NETIF_MSG_LINK,(NETIF_MSG_LINK, "read status 8705\n");
	bnx2x_cl45_rea45_2ead(bp,e tcsEXlink_the SFP+ module.)*/
	tx_en_mode = R;
	/* Low ==> corresread M<06/CFG_fw_respac-Cl);

RXcAlarm Set the read command byte count */
	bnx2x_cl45k status */
	bnx2odulx_phy 2	      MDIOd2x_8askoN_DEVrnce the rt45_rd 2n/
	bnT2odulxHW_CFG_Tv status */
	bnT2offsct atac-Cl);

, &rx/* Set GPI Set the read command byte count */
	bnx2x_cl45k status */
	bnodulx_phy 1.c809 val=0x%x\n", val1);
	linkt */
	bnx2x_cl45k status */
	bnodulx_phy 2
			;
	/* Low ==> corresM<06/CFG_D, &rx_sd);

	bnx-->
	bnx2x_cl45_rea45_2	      MDIO_PMA_DEVAD, 0xc809, &D) =}8d 2nd bit 	bnx2x_cl45_read(bp, phy,
		     MDIO_PMA_DEVAD, 0xc809, &D) =}8dCSnd bit 	bnx2x_CSne2_setule_, ptcsEXlink_	     MDIO_PMA_DEVAD, 0xc809, &D) =}8ANnd bit 	bnx2xANnnx2_L(val[tule_, phy 2
			  MDIO_PMA_DEVAD, 0xc809, &D) =}8ANnd bit 	bnx2xANnnx2_L(val[tule_, phy 2
				;
	/* Low ==> corresM<06/CFG_Dhy,
	/m_bastcsEXlink_/m_bas1Gbps"_MSG"8705\nXlink_/m_ba2x_cld(bp,e tcsEXlink_ea45_2	  ificn;
	"y->fld_f_bothset G0ule_tmd_hy,
	/a, set G0ule_tcsEXlink_the Ge GPidxx_l isf_moduautonegset G1dule (SO contr705\n");
	((hy,
	/&stcsEXlink_/&00)1) ||trans2hy, par1)FORCEMENT705\n")le_eeprom(ans2hy, par1)F_MSGead :
	{
< 1);
				}
				netdev_eDENTIFIER,d :
	{
< 1);
				}
				netd0			Invalidextval12x_folvpoNr_data += xfer_ead s;
IER,d :
duplepes DUPLEX_FULtached cablCapent iy[idp;
	"N_DEV.AD,
			wice		&val);

XliUPPO2nd . ontrol tR,d :
	{
< 1);
					}
				netd0f ((val & MDIO_PMA_REG_SFP_TWOj<<<<<<<<<<<<<<<<<<<<_Tv status */
	bnT2odulx_phy 1.c80al & MDIO_PMA_REG_SFP_TWOj<<<<<<<<<<<<<<<<<<<<_Tv status */
	bnT2odulx_phy 1.c80arom(ans1hy, par0)to tER,d :
N_DEVA);
	if (     SFrams *param705\n");ams  */
/******************************************************************/
	s->shm6sPEV SECG;
		ddisable_pmd_transmit(struct link_params *params,
					   struct bnx2x_phy *phy,
		k_vars *vars6_ case ot bn, u8 *o_buf)
{
	int rc = 0;
	struct bnx2x *bp = params->= 0;
	struct bnx2xNK, "read statusG_RD(bp, params<<4);anxx_<<<, tmp1O_PTX_TECH_ADDR + 1], check_limitiP_nvalid TX_LASER_MDIs->port);
	bnx2x_cl42+ module isn = tx_en_mode - PORT_HW_CFG_TX,dT_MDL_ENFRCMwer_ficHWTx_f:ut"KR PCS stextval12;
	bnx2x_rt].sfp_ctrlFRCMwer_ave microcode, hence the rt45_rd 2nd bit  bnx2x_cl45_reD_10000)a04n't have mival;
x_f:uf		MDIO_Pence the rtruct werad(bp, phy,ntil fw_PHY oad ( ontrECT)
PROM_PAG IO_PMAal);
IO_	if ((val & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MAinish before
	XoB:uf:
			   timeout *   tturn	REG_SFP_T *phy,
				 struuct linrn rc;
};
	/* Low ==> corresread M<06 SIVE)bnx2x *bp; timeofp_module
IO_G_MSG)
		specific phy request */
		cmd = DRV_edia*/
		if (DUAOuf:RI DACREEMPH
	bS_AM, &v	, 10G_FisteSFP_TGH;reuct TWO_WIRE_CTRL, &v4727_read_sfpreuport45_rXSvars6_re
	BAvalRX0d to 10i*(t45_rXSvars6_re
	BAvalRX1_-INK,	_Tv statXSvars6_re
	BAvalRX0resp ruct bnIO_PMA_REG_SFP_TWOj<<<<<XSnd bit 	reu			   timeotac-Cl);

NK,
		3->lis of_modu	if IOs F_MSG_he iO_PM0x7imeotac-S
	 	if IOs bP_EEPRwhere the 1_NO_ENu a;
	}

	fw_tR,
				 ];
		hy,preemphasiscl45&00)7timeot;
	/* Low ==> corresponding RXcEvendozpoweres->shm6"_MSG_LIN"	reu/m_bas<--_veprm_ba2x_cldeu		   timeotave microcode, hence the rt45_rXSnd bit 	reu		   timeo}A_REG_SFFport",
				ontrol to OUTable to determin		}
				netd0f ((va;
	/* Low ==> corresread M<06 gportiy[ibpslrect attFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0)DIGITALeD_10000)400val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnT2offsetof(st0val[SficArmD, &rxfARp)
nalNPUTTx"N_DEV.A	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVAD, MDIO_PMA_REG_807/
	bnD_10003
			A_DEVA->lin_SFFport"1Gbps	 * higautonegsEEPROMG advertiseme((byte*/
ficAlTERSCL37_mornx2x_CL73_
	fw_cmd_param = FW_PAR "read M<06 AutoNeg = val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TANnd bit 	bnx2xANnnx2_CL37_CL73000) 40cct attac- nion
 Full-Duplepeadvertiseme((b	}
CL37_	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVANnd bit 	bnx2xANnnx2_CL37_FC_LP000) {20val[Sfic nion
 CL37_AN_	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVANnd bit 	bnx2xANnnx2_CL37_AN000)1addr32, ficMG hy->type	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVANnd bit 	bnx2xANnnx2_ADV, _phy5)vall[Sfic nion
 clafor
73_AN_	bnx2x_cl45_read(bp, phy,
			MDIO_PMA_DEVANnd bit 	bnx2xANnnx2_C_10000)1200val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnx2offsetof(st0) 400val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnD_100tof(st0) 004k;
	}
EEvalid avpobcmdeti us(strence the rtruct rlFRCMwers OUTPfpuctLshmem struf IOsle_CNT - PORuctdofr, pIO_sPEV go	REGISTERd complPIO_OUTP,isf_uctLshmem stcation
	TYPE_DIRD(bp, para_config[port].sfp_ctrl)) &
		PORT_HW_CFG_TX_LASER_MASK;
	DP(NETIF_MSG_LINK,,date media type for non-PMF sync */
	fort %x "
	ter &= bnx2x_8727_TXk_paraDE_SHIFtrol tD(bp, para_c== bnx2x_8727_TXk_paraDASER_f ((va;
	/* Low ==> corres nion higTXONOFFe */DNn	u3 = val[SFP_EEPROM_FDEVAD, 0xc809, &D) =}8d 2nd bit 	bnx2x_cl45_reDIGITALeD_1000&tmp1val[Stmp1				0)1al[SFP_EEPROM_FC_TX_TECH_ADDR];

) =}8d 2nd bit 	bnx2x_cl45_reDIGITALeD_1000tmp1val[rams *param0rams *params,
					     0_XGXS_EXlink_up = 0;
	u16 val1, rx_sd;
	sto*bp = params->bp;
	DP(NETIF_MSG_LIstruct bnx2xNK, "read statu *paramars *vars6_CFG_XGXS_EXlink_udata += xfer_ead s;
ms  */
/******************************************************************/
	s->sh_BsPEV SECG;
		ddisable_pmd_transmit(struct link_params *params,
					   struct bnx2x_phy *phy,
		_up) {
	/* Mak6_ case oloopbackup = 0;
	u16 val1, rx_sd;
	sto %d\d_param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	char;
	/* Low ==> corres_cl/PMD extval12loopback: CFG_fw_respave microcode, hence the rt45_rd 2nd bit  bnx2x_cl45_reD_10000) 001wercfg;
	struct bnx2x *bMak6_external_ us(boon, u8 *o_buf)
{
	int rc = 0;
	/* Initialization time after P_EEPROM_VENDOR_NAME_SIZE+1];
	charMA_REG_SFP_val;Aal) ti timeox_f:ut"KR 0;
			D145_read(bpMturntruf IOsleeox_-boon);
	/* Wait appropriate time for two-wire command  bnx2x_cl45_reGENeD_10000) 18B_read(bpS
	 softTx_f:ut"KR PCS st_PMA_ is
	 MDIO_PMA_REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<GENeD_100<<<<<<<<<<<<<<<<<<GENeD_10	XoB:MICROenxSET	      MDIO_PMA_ is
	 MDIO_PMA_REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<x7fbf);
	1000) 001werR PCS st_PMA_ is
	 MDIO_PMA_REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<GENeD_100<<<<<<<<<<<<<<<<<<GENeD_10	XoB:nxSET4&= ERN	retPwerad(bp, phyfARp15 (phy->Meturn 0;
O oadt"KR 0;
			D155_read(bp(pmd_dishmex2x boon)ruf IOs00tri;
	ses				s SS_N00SCrreMOSIDIs->Ot"KR PCS st_PMA_ is
	 MDIO_PMA_REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<x7fbf);
	1000) 005_read0;
			D200val[Evalid avpobcmdeti us(strence the rtruct rlFRCMwercfg;
	struk_vars *varG_XGXS_EXlink_up = 0;
	u16 val1, rx_sd;
	ststruct bnx2x *bp = params->= 0;
	struct bnx2xNK, "read statuSG_LINK,
		   "Reading from eeprom is l1;tus 8705\n");
	ars *vars6_CFG_XGXS_EXlink_udata += xfer_ead s;
EMENT705\n")le_eepl & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MAinish before
	PIO1:
	case PORT_HW	phy 1.c80arom(ans1hy, par15;
			if (phy->req_line_speedTx" stcation
	
		resp r705\n");
	bnx2IER,d :
	{
< 1);
				DR_A0,
	}ms *param705\n");ams <<<<<<<<<<<<<<<<<<;/=}= case ot bn, u8 *o_buf)
{
	int rc = 0;
	o*bp = params->bp;
	DP(NETIF_MSG_LIstruct bnx2xNK, "read statuEPROM_VENDOR_NAME_SIZE+1];
	char;
	/* Low ==> corres

	for (t higs->sh_Blrect atave microcode, hence the rt45_rd 2nd bit  bnx2x_cl45_reD_1000par15;t have mival;
x_f:uf		MDIO_Pence the rtruct weradx2x *bMak6_external_ us(boon,the rtruct werad(bpREG_SFP_maIPendor_pn);
	if (  */su	rc = bnx2x_r_EEPROmodule(strd comINVAL;
	}
triggere_CNT actverif00);
	}
serti	}
migh(b	ccur command comdrivmem st oad ( INPUTrn -Edrivmem st oad ( IitTx_f:utaIPd com333);
	}er_H_ADDRe statusOptic modulrom(struct bnx2x_8727_poINVAL;
	},ule(struct wertrol to OUTable to determin		}
				netd	 * of passive cable
		 */
		coppeMG gport = val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx bnx2x_cl45_reD_10000)40val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status *5_reSG_LI_102000)Dval[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnD_10000)5val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnx2offsetof(st0)400val[d first phyto OUTable to determin		}
				AUTO_NEGted\n");
	to OUTeterm_cax_8ask SFP_EEPRO= bnx2x_8727_}
				CAPABILITY_D0_1Gted\n");
	tto OUTeterm_cax_8ask SFP_EEPRO= bnx2x_8727_}
				CAPABILITY_D0_10G0x%x when r bnx2x_8727_}
				CAPABILITY_D0_10G0	 * of passive cable
		 */
		coppeMG clafor37 = val[S(bpS
	 FTERS	if IOs F_MSGPCS stextval12f:ufpafor  detected\n")ead s;
IEx_cl45_read(bp, phy,
			MDIO_PMA_DEVANnd bit 	bnx2xANnnx2_ADV, 0x20val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TANnd bit 	bnx2xANnnx2_CL37_CL73000) 40cct x2x_cl45_read(bp, phy,
			MDIO_PMA_DEVANnd bit 	bnx2xANnnx2_CL37_FC_LD000) {20val[Sx_cl45_read(bp, phy,
			MDIO_PMA_DEVANnd bit 	bnx2xANnnx2_CL37_AN000)1addr32, x_cl45_read(bp, phy,
			MDIO_PA_DEVANnd bit 	bnx2xANnnx2_C_10000)1200val[Sfic nion
 RX-ALARM
	if IOs valrecepe\nS, &val2);fARp1G",
			
_PMA_nx2x_c
er-curreu8 tx_ROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnD_10000)4val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnx2offsetof(st0)400valTB	rc = bnx2x_Dereaddr10G.pS
	 s inp, &rx	if IOs F_MSGPCS stROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnD_10001val[rams(bpS
	 TX PreEmphasisl[SFnEG_			ontrol t	specific phy request */
		cmd = DRV_edia*/
		if (DUAOuf:RI DACREEMPH
	bS_AM, &v	, 10G_Fcmd_param = FW_PARAM_SET(
		coppeTXf);
	1rm_ba,eTXf);
	2Y_DA_TWINAX;BCe_eepry,preemphasisc0]NAX;BCe_eepry,preemphasisc1]val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0)Mak6_TXf);
	1x2x_ware_eepry,preemphasisc0]ct attFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0)Mak6_TXf);
	2x2x_ware_eepry,preemphasisc1]val[rams *param0rarcfg;
	struct bnx2x *bMak6_bnx2xbnx2x_"Setting SFP+ transmitter to _param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	char;
	/* Low ==> corresx2x *bMak6_bnx2xbnx2xP_CODE_ASFP+dT_MDL_ENFRCMwer_ficS
	 smex2x boon)ruf IOs;fARpexternalO oadt"KR PCS st_PMA_ is
	 MDIO_PMA_REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<GENeD_10000) 001wercfg */
/******************************************************************/
	s->sh_7sPEV SECG;
		ddisable_pmd_transmit(struct link_params *params,
					   struct bnx2x_phy * *params_up) {
	/* Make _2x_:ux2xlbp = params->bp;
	u16 timeout; %d\*bp = params->bp;
	DP(NETIFuk_vEPROM_FC_TX_TECH_ADDR + 1], check_limiti)om ilbp_EPRO_bit8ask 
	bnx2x_clO 0x%x\ns_bit8ask 
	bnx2x_clor as OUTO inper-culavor "FW ibnx val in caseLEDure_confiaIPy	ontrol t>to OUTPUT, and set the p
			= bnx2_pafp_modulval =		break;
LEDDEADDRFRO= ~(ANEL_OFF:break;
LEDDEADDROFF:br	lbp_EPRO_bit8ask 
	bnx2	O 0x%x\ns_bit8ask 
	bx03al[SF	read_func =LEDDEADDRON:br	lbp_EPRO_bit8ask 
	bnx2	O 0x%x\ns_bit8ask 
	bx02al[SF	read_func =LEDDEADDROcable 	lbp_EPRO_bit8ask 
	bx6bnx2	O 0x%x\ns_bit8ask 
	bx11al[SF	read_f}    MDIO_PMA_DEVAD, 0xc809, &D) =}8d 2nd bit , &D) =}8d 2n3_SP<<<<<phy,int offsetof(phy *phy MDIO_P00ff8f;
oR,
				lbp_EPRO_bit8askespave microcode, hence the _REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<phy,int offsetof(	   time  MDIO_PMA_DEVAD, 0xc809, &D) =}8d 2nd bit , &D) =}8d 2n3_SP<<<<<<<<<<<<<<<<<<<phy *phy MDIO_P00ffe0;
oR,
				O 0x%x\ns_bit8askespave microcode, hence the _REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*params_up) {
	/* Make ;
	bnx2x_"Setting SFP+ transmitter t_param;
	char vendor_name[S atusG_Rswax_<<<, swax_<4);rirams<<mitter;r_fictheakEVTx_f:ut struf IOsle_CNT - PO 1. Fdetecase bRCMNT_MAS
is 1
	 caEPRP moduswax->l N
in  TX_LASER_TYPE_D
EPROM_VENDOR_NAME_SIZE+1];
	char"wax_<<<_config[port].NDUAre
	Pbnx2SWAPwer	swax_<4);rira_config[port].NDUAre
	STRAPAOuf:RI Dwer	tter ( ("wax_<<<_&& swax_<4);rira) ^   SFnvalid TX_LASER_MDIs->port);
	bnx2x_cl41+ module isn = tx_en_mode - PORT_HW_CFLOW, FRCMwercfg;
	stru_up) {
	/* Make  case os
			_"Setting SFP+ transmitter to %d_param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	char e.g.mp1,lor as OUTS
	 s_LINEA1G",
				ontrol t	s OUTable to determin		}
				netd	 ||n";
			];
		}
		break;
					phy->req_line_spee0	 * of passive cable
		 */
		coppeMG gport = val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx bnx2x_cl45_reD_10000)40val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status *5_reSG_LI_102000)Dval[SFP_EEPROM_FA_REG_SFP_TWO_WIRE_CTRL_STATUS_MAinish before
	SG_LI_10200&tmp1val[S{
				if (params->phy.7 
	bxba2x_cltmp1val[SficPs powspeed2x_wXAUIy,ntil l;
	"y->fld_upport OPT ver-}
		b
_PMA_NPUT1Gterdcurrerom(			return -EINVAL;
		}
		l & MDIO_PMA_REG_SFP_TWO_WIRE
if (TB	rc = bnx2x_waD) =}8d 2n3_SP<<<<<phy,GP			   timeotR,
				 3_A10resp ruct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<<<<<phy,GP		   timeo}A_R first phyto OUTable to determin		}
				AUTO_NEGted\n");
	tto OUTeterm_cax_8ask SFP_EEPRO bnx2x_8727_}
				CAPABILITY_D0_1Gtted\n");
	tto OUTeterm_cax_8ask SFP_EEPRO= bnx2x_8727_}
				CAPABILITY_D0_10G0x%x when  bnx2x_8727_}
				CAPABILITY_D0_10G0	 *  of passive cable
		 */
		coppeMG clafor37 = val[Sx_cl45_read(bp, phy,
			MDIO_PMA_DEVANnd bit 	bnx2xANnnx2_<<<<<x7fbf);
	,t0val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TANnd bit 	bnx2xANnnx2_CL37_AN000)1300val[d first>lin_SFSEEPROmodush_7shas s inp* hileTx_f:utx\n",nEG_SFP_ in case10G
_PMA_333);
	}e althnx2x_f NOC >ereadd
er-curreu8 tx_ROM_FC_TX_TECH_ADDR];

		if (TANnd bit 	bnx2xANnnx2_<<<<<x7fbf);
	,tof(st0) {20val[Sx_cl45_read(bp, phy,
			MDIO_PMA_DEVANnd bit 	bnx2xANnnx2_CL37_AN000)0100val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status *5_reD_10000)2040val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status *5_reSG_LI_1020tof(st0) {08val[}ams *params,
					   Make  case ot bn, u8 *o_buf)
{
	int rc = 0;
	o*bp = params->bp;
	DP(NETIF_MSG_LIstruct bnx2xNK, "read statusG_RD(bp, params<<4);.mp1,l872_ab	cltmp2al[EPROM_VENDOR_NAME_SIZE+1];
	charMA_ nion
 PMD bnx2, oAD_ABS_FLr INPUT1idp;
	"*larmPE_DIRave mival;
x_f:uf		MDIO_Pence the rtruct werad;
	/* Low ==> corres

	for (t higs->sh_7lrect atave miMake _read(bp, phy,data += xfer_PEVAD, Mwer_fic

	for inp_NO_ENu * oAD_ABS speed &val2);rn -Emodule _sd compx_f:EPR(set G8)rom(struct bnROM_FDEVAD, 0xc809, &D) =}8d 2nd bit 	bnx2x_cl45_rePIO1:
	case POR &872_ab	wer_ficS
	 m thPff NT s		coppeintXLOSp* gnalOedf (is_pTERS(et G9).O co Wn -Ecasem th_sf,ff t Glocks s GIS*r33fer:EPR clovalNPUTa_up)sd combe		Moppe'lost'rom(str872_ab	iO_PMA_RE8s;
EMENT>to OUTPUT, and set the p
			872_ab	iO_PMA_RE9respave microcode, hence the _REGj<<<<<<<<<<<<<<<	bnx2x_cl45_rePIO1:
	case POR 872_ab	werarMA_ nion
/(pmd_diskEVT_en);
	else
ut to e(struct bnx2x_cation
_tmd__en);
	e			int idx, cfg_idhyave miMake ;
			bnx2x_wan", val2r1.c8truct bnROM_FDEVAD, 0xc809, &D) =}8d 2nd bit 	bnx2x_cl45_reM8051w ==MSG_5_r00&tmp1valet the read command byte count */
	bnx2x_cl45k status */
	bnx2odulx_ptmp1valet the rMake  case os
			_the rtruct werams(bpS
	 TX PreEmphasisl[SFnEG_			ontrol t	specific phy request */
		cmd = DRV_edia*/
		if (DUAOuf:RI DACREEMPH
	bS_AM, &v	, 10G_Fcmd_param = FW_PART(
		coppeTXf);
	1rm_ba,eTXf);
	2Y_DA_TWINAX;BC Ce_eepry,preemphasisc0]NAX;BC Ce_eepry,preemphasisc1]val[SFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx<<<<<<<<<<<<<<<<<<<TXf);
	1x2x_ware_eepry,preemphasisc0]ct attFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx<<<<<<<<<<<<<<<<<<<TXf);
	2x2x_ware_eepry,preemphasisc1]val[ramsOUTPfpuctLshmem struf IOsle_CNT - PORuctdofr, pIO_sPEV go	REGISTERd complPIO_OUTP,isf_uctLshmem stcation
	TYPE_DRD(bp, para_config[port].sfp_ctrl)) &
		PORT_HW_CFG_TX_LASER_MASK;
	DP(NETIF_MSG_LINK,,date media type for non-PMF sync */
	fort %x "
	ter &= bnx2x_8727_TXk_paraDE_SHIFtrol tD(bp, para_c== bnx2x_8727_TXk_paraDASER_f ((G_Fcmd_param = FW_PART( nion higTXONOFFe */DNn	u3 = val[SFP_EEPROM_FDEVAD, 0xc809, &D) =}8d 2nd bit 	bnx2x_cl45_re<<<<<int oFG_5_r00&tmp2val[Stmp2				0)1td0			Itmp2	O_P00FFEFal[SFP_EEPROM_FC_TX_TECH_ADDR];

) =}8d 2nd bit 	bnx2x_cl45_re<<<<<int oFG_5_r00tmp2val[Sl & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MAinish before
	PIO1:
	case PORT_HW	ptmp2val[Sl & MDIO_PMC_TX_TECH_ADDR];

		if (TB	rc = bnx<<<<<<<<<<<<<<PIO1:
	case PORT_HW	 tDmp2	O00)7fffe	  [rams *param0rams *params_up) {
	/* Make ;NPU_po872_ab	,"Setting SFP+ transmitter to %d\d_param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	char _up872_ab	clrx_*larm8 link_;ar vendor_config[port].sfp_ctrl)) &
		PORT_HW_CFG_Tval & PORT_FEAT_CFG_OPT_MDL_ENFRCMNT_MASK) ==
	 
		    PORT_FEAT_CFG_OPT_MDL_ENFRCMNTK) ==
	 
		_NO_ENFORCE  MDIO_PMA_DEVAD, 0xc809, &D) =}8d 2nd bit , &D) =}8d 2n3_SPPIO1:
	case POR &872_ab	wer_ol t872_ab	iO A_RE8sf ((G_Fype odule iodubse((byte_Fcmd_param = FW_PARAM_SET(oAD_ABS S, Set GPICFGow module _sfubse(( = val[S];
		}
		break;
	default:
hy *PnxSENTd_f ficM.pS
	 872_ab	i	 * outT_Cnext re_reaver-c 
		 x_f:EPRreve
	ver-c 2.cS
	 m thPff NT s		coppeintXLOSp* gnalOedf (is_pTERver-c 
		(et G9).O r-c 
		Wn -Ecasem th_sf,ff t Glocks s GIS*r33fer:EPR clovalNPUO r-c 
		a_up)smbe		Moppe'lost'.
er-curre872_ab	iO_PMA_RE8s;
Eerom(>to OUTPUT, and set the p
				872_ab	iO_PMA_RE9resptFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0)PIO1:
	case POR 872_ab	werarpac-Cl);

RXc*larmP_EEPROt G liy->fldast oOs vaO r-c module(_ab	iwasn't_nx2x_cd
er-curreu8 tx_ROM_FA_REG_SFP_TWO_WIRE_CTRL_STATUS_MA_WIRE_CTRL_STA/
	bnx2odulx_prx_*larm8 link_valTB	rc = bnG_Fype odule iod x_f:E(byte_Fcmd_param = FW_PARAM_SET(oAD_ABS S, Set GPICFGow module _sf x_f:E( = val[S(bpFK,
		dpmd_dis) {
		if inpuNPUT_f_module(stru_sfo2, In tero*bparamsoINVAL;
	} wiIP ower_upval is 1M.pS
	 872_ab	i	 * outT_Cnext re_readubse((beve
	 (set G8)ror-c 2.cIO_Oo * In c>ereaddrpolarl45_of_moduOPRXLOSp* gnalONPUO r-c 
his * gnalOwiIP tn -EcorrtT_ and  Set tecase x_f:EPRr-9 dPMA_Nbf:EPRr-f_moduRx * gnal.	(et G9)
er-curre872_ab	i			 val8s;
Eerom(>to OUTPUT, and set the p
				872_ab	i			 val9resptFP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0)PIO1:
	case POR 872_ab	werarpac-Cl);

RXc*larmP_EEPROt G liy->fldast oOs va module(_ab	 dPMA_wasn't_nx2x_cd.cthis iodnEG_SFP_beicl N
comman_maIPe statutero*bparamsmINVAL;
	},CoCOMPwisIOt GwiIP al);
c modup;
	"updt t dPMA_Nlarm
er-curreu8 tx_ROM_FA_REG_SFP_TWO_WIRE_CTRL_STATUS_MA_WIRE_CTRL_STA/
	bnx2odulx_prx_*larm8 link_valT
rerom() {
		DP(NETIF_MSG_LINK, "NOT enforcing module vparamP(NETIF_MSG_LINK, "NOT enforcin	u32 sw_TXk_paraasp ruct bnx_en		}
		} else {
			int idx, cfg_idhy}NO_ADDR,
	val;
	int rc;
	struct bnx2x *bp ule(struct w	;
			sp ruct bnx_en8727_poINVAL;
	},ule(struct wer_	DENTIFIE;
	/* Low ==> correspFP+ module _sfr, pt bnx2x *bpfw_resarpac-R__NO_ENu * p;
	",
					POR(  */params->shm-Puf:u				Ds-curreu8 tx_Make  case os
			_the rtruct wer[rams;
	/* Low ==> corresM<27
RX_ALARMl[tule_Y_DA_TWINAX;
 lrx_*larm8 link_wer_ficNodnEG_SFP_cb valp;
	",link_/_upport OPTmodule iluggeC_RE/, pacurcfg;
	struk_vars *varG7XGXS_EXlink_up = 0;
	u16 val1, rx_sd;
	ststruct bnx2x *bp = params->= 0;
	struct bnx2xNK, "read stFP_EEPROM_VENDOR_NAME_SIZE+1];
	char  8705\n");
	b,Coc_tter ( TX laser of theom il05\nXlink_/
	bnx2x_clrx_*larm8 link_,t shi %x ",is l1;tmsOUTPfpPEV _sfr, pt bnx2x *bpctdofr, pcb valp;
	",link_/ Set the read command byte count */
	bnx2x_cl45k status */
	bn<<<<<<<<<p shi %x "s;
EMENT> shi %x "s
2C_DEV_ADDR_l[ac->b valcaseL &rx */Rx  Set the read command byte count */
	bnx2x_cl45k status */
	bnx2odulx<<<<prx_*larm8 link_wer_R,d :
	{
< 1);
				DR_A;
	/* Low ==> corresM<27
RX_ALARMl[tule_Y/m_ba2x_cld(b*larm8 link_valTBuct bnx_en8askoN_DEVrnce the rt45_rd 2n/
	bnT2odulxHW_CFG_Tv status */
	bnT2offsct at the read command byte count */
	bnx2x_cl45k status */
	bnodulx_phy 1.c8_A;
	/* Low ==> corresM<27
, &rx_sd);

	bnx2x_cl45_rwerarMA_Cl);

 ==-OUTm(struct bnROM_FDEVAD, 0xc809, &D) =}8d 2nd bit 	bnx2x_cl45_reM8051w ==MSG_5_r00&45_rwerarMA_Pfpa module _sf x_f:E(uNPUTCOMPe iodnEG_SFP_cb vad comfARp<4); 	else
	ve	ontrol t>to OUTPUT, and set the p_&& !(d(b*larm8 link_iO A_RE5))s->linype = val<4);
	else
		 * higM<<<t), a0Oedf (curreu8 tx_ROM_FA_REG_SFP_TWO_WIRE_CTRL_STATUS_MA<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<	&45_rwerarerom() {
1iO A_RE8sf ;
			returnol t>"Setting 1xparam<<<<	oc_tter ( BP_PATHpara +		COPPER:
tter <<r1.c80FIcmd_param = FW_PARAM__SET(sh_7sPlPIO_readdrhas be -Ed;
	if ( oMP_CODE_ASFP+ mo_SEToc_tter.c80FInetRCMNerrenc->RCMVAL &tmp: PlPIO_readdroMPPCODE_Arhas "_MSG_CFG_T"be -Ed;
	if ( NPUTCOM is powere"_MSG_CFG_T"tha
	0, 10000);
	has be -Eremrom(p"_MSG_CFG_T"tof x_ve
	 f_FPu * of_modu	ard.c"_MSG_CFG_T"Pl);seEremromd2x_warpcore_readNPUT"_MSG_CFG_T"rO_OFRCMmodusystem		&val);


his "_MSG_CFG_T"e&tmp.SFP+ mo_Soc_tter.c80FI(bp(pmd_disaIPeRX_ALARMs exce2);fARp872_ab	i
	fw_tuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<</
	bnx2offse _phy5)vall[S	l & MDIO_PMA_REG_SFP_TWO_WIRE
if (TB	rc = bnx2x_waD) =}8d 2n3_SPPIO1:
	case POR &hy 1.c80ad(bp, phyfARp8727_poubse((__ve
	 
	fw_tR,
1i			 val8s;
Eeruct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<PIO1:
	case POR hy 1.c80ad(bpCl);

RXc*larmP
	fw_tuct bnIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MA_WIRE_CTRL_STA/
	bnx2odulx_prx_*larm8 link_val		yave miMake ;
			bnx2x_wacheck_limi	bnx2x_0s, I2C_DEV_ADDR_A0,
	}x2x_O4); 	else
	pcb valE_DIR/o Wn -Ere_readubse((bbf NOC idxx_cb valre_readcurrol td(b*larm8 link_iO A_RE5))e_eepl & MDMake ;NPU_po872_ab	,ule(struct wer_	MA_ nion
 aIPendo_ab	iNPUTp;
	"INVAL;
	} bP_EEF_MSGPCS stROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnxk status */
	bnx2offsetof(strA_RE5)a| _phy2)e	  [rams(bp->to OUTPUT, and set t2x_phy *phy,
			,  ((va;
	/* Low ==> corres nion higsh_7suct shme = val[SFP_EEPx, "10G Optic module detected\n");
			A_DEVA->lin(phy->req_line_speedTx" stcation
	
		resp _DEV_ADDR_A} at the read command byte count */
	bnx2x_cl45, &D) =}8d 2n3_SP8073_}
				L(val[tule_, pl05\nXlink_werarMA_BP_EE0..2 -->
1);
		d;
	if (,d comBP_EE13..15-->
l;
	"y->speeO control t(l05\nXlink_/& _phy2)e_&& ->tl05\nXlink_/& _phy15;
,  ((va705\n");
	1al[SR,d :
	{
< 1);
				}
				netd0			I(phy->req_line_speed_TX_LAS: ExternalO ;
	"up/_up10GTWINAX;BC Ce_MDL_ENFRCMwer_R first phytl05\nXlink_/& _phy0)e_&& ->tl05\nXlink_/& _phy13;
,  ((va705\n");
	1al[SR,d :
	{
< 1);
				}
				netd			I(phy->req_line_speed_TX_LAS: ExternalO ;
	"up/_up1GTWINAX;BC Ce_MDL_ENFRCMwer_R first((va705\n");
	d			I(phy->req_line_speed_TX_LAS: ExternalO ;
	"y->speeTWINAX;BC Ce_MDL_ENFRCMwer_Rd cablCapent iy[idp;
	"N_DEV.Aontrol tR,d :
	{
< 1);
					}
				netd0f ((val & MDIO_PMA_REG_SFP_TWOj<<<<<<<<<<<<<<<<<<<<_Tv status */
	bnT2odulx_phy 1.c8(val & MDIO_PMA_REG_SFP_TWOj<<<<<<<<<<<<<<<<<<<<_Tv status */
	bnT2odulx_phy 1.c8(varom(ans1hy, par0)treturnR,d :
N_DEVA);
	if (     SF0,
	}m
EMENT705\n")le_eepl & MDextval12x_folvpoNr_data += xfer_ead s;
IER,d :
duplepes DUPLEX_FULtachI(phy->req_line_speedduplepes 	bnx2x_cl45d :
duplep	  [rams(bp-(			return -EINVAL;
	d\n";
			s OUTable to determin		}
				netd	le_eepl & MDIO_PMA_REG_SFP_TWO_WIRE_CTRL_STATUS_MA_WIRE_CTRL_STA3_SP<<<<<phy,GP			   1val[SficIupport OPT ver-}
		b boardINPUT1i, is pow");2x_wXAUIysidamtercomoCOMPwisIOis powf Nspee.FFpoiy[idf NOC >l N
automaramaIPyrordcurrerom(705\n")lfw_tR,
1iO_PMA3_A10resp DENTIFIER,
1i			 3_A10resp FP_EEPROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0)Mak<<phy,GP		   1val[ras *param705\n");ams *params_up) {
	/* Make bnx2xbnx2x_"Setting SFP+ transmitter to _param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	chaarMA_ nion
/(pmd_diskEVT_en);
	else
ut to e(struct bnx2x_cation
_tmd__en);
	e			int idx, cfrwerarMA_(pmd_disTen);
	else
(struct bnx, "10G Optic module detected\n")ower_ficCl);

, &rx"KR PCS st_PMA_ is
	 MDIO_PMA	if (TB	rc = bnxk status */
	bnD_10000)rarcfg */
/******************************************************************/
s->s481/s->s4823/s->s4833sPEV SECG;
		          able_pmd_transmit(struct link_params *params,
					   struct bnx2x_phy *phy,
		,
					   is_s483x_s485x_"Setting SFP+ transmistatu *param(le_eeprom;_c== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4833	 ||n"	le_eeprom;_c== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4834	 ||n"	le_eeprom;_c== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858)wercfg;
	stru_up) {
	/*  avpoP48xxdeti us(strs
	},"Setting SFP+ transmitter t de */
	u16 val;
for LED moO regimitter= (1<<4);eep, fw(str2,;anxx_ial[EP	stru"Setting SFP+reg"10G reg"10G[]E_S_eep{if (TB	rc = bnxk0xA819000) 014},eep{if (TB	rc = bnxk0xA81A_poxc200},eep{if (TB	rc = bnxk0xA81B000) 005},eep{if (TB	rc = bnxk0xA81C000) 305},eep{if (TB	rc = bnxk0xA817000) 009,
	}nx2x_clfw(str1IFtrol t			   is_s483x_s485x_smisf ((val & MDIO_PMA_REG_SFP_TWOj<<<<<CTLc = bnxk0x400f			fw(str1s;
Eerom(e_eeprom;_!== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858)ED mfw(str1IO_P00fffal[SFP_EEPxavpoeti us(strs
	},_SFP_CFG_Ofw(str1,re_eepstr_addrval[d first>lin_SFFpoi32-bitTx_3);
	}e _upP48xx,EPRwess v	b <<<<2ARM
i/f.A	bnx2_SF(1) 10G regpoxc200_ 014(SPI_BRIDGEeD_10	2)		&v0) 3055555A	bnx2WO_WIRE_CTRL, &vARRAY_SIZE(reg"10G)727_re
Eeruct bnIO_PMC_TX_TECH_ADDR reg"10G[i].devadLED moO reg"10G[i].reu		reg"10G[i].hy *phRSrECT)
PROM_PAG IO_PMAal);
IO_	if ((va PCS status 0x%x\n", val2rt45_rd 2nd bit  0xA818			   timeotrom(anshy,1m<<<<		REG_SFP_	udelay(5timeo}A_trom(PROM_=Aal)
			if (phy->req_line_speedUwer_upmo/
statP48xx "_MSG_C" trafw strs
	},1)
		resp ruct bnxavpoeti us(strs
	},_SFP_CFG_O0LED moO	 re_eepstr_addrval[		= bnx2_pa[ramnx2_SF2)	
stat333);
	}
oxc200_ 055A(SPI_FWl[tule_)EF_MSGPCS stROM_FC_TX_TECH_ADDRrt45_rd 2nd bit  0xA819000) 005_reSGPCS stROM_FC_TX_TECH_ADDRrt45_rd 2nd bit  0xA81A_poxc200_reSGPCS stROM_FC_TX_TECH_ADDRrt45_rd 2nd bit  0xA817000) 00A_reSGECT)
PROM_PAG IO_PMAal);
IO_	if ((va PCS status 0x%x\n", val2rt45_rd 2nd bit  0xA818			   timeotrom(anshy,1m<<<<		REG_SFP_	udelay(5timeo}A_trom(PROM_=Aal)
			if (phy->req_line_speedUwer_upmo/
statP48xx  trafw "_MSG_C"strs
	},2)
		resp ruct bnxavpoeti us(strs
	},_SFP_CFG_O0LED moO	 re_eepstr_addrval[		= bnx2_pa[ramx2_SFls pow_cl>lis of_modu333);
	}
SPI_FWl[tule_-curreu8 tx_ROM_FA_REG_SFP_TWOrt45_rd 2nd bit  0xA81B			fw(str1s;
Ee_SFupppow_cl>lis of_333);
	}
SPI_FWl[tule_-curreu8 tx_ROM_FA_REG_SFP_TWOrt45_rd 2nd bit  0xA81C			fw(str2.c8(val & MDxavpoeti us(strs
	},_SFP_CFG_O(fw(str2_A16)a| fw(str1,
 moO	 re_eepstr_addrval[}
<<<*params_up) {
	/* M48xxdeetxlbp = params->bpfor LED mo"Setting SFP+ transmistatu<4);eep, lbp3_bbnx2xbate, hy_idxx_ial[EP	stru"Setting SFP+reg"10G reg"10G[]E_S_eep{if (TB	rc = bnxkpcore_r == 0)M481_LED1ng mo000) 085},eep{if (TB	rc = bnxkpcore_r == 0)M481_LED2ng mo000) 018},eep{if (TB	rc = bnxkpcore_r == 0)M481_LED3ng mo000) 006},eep{if (TB	rc = bnxkpcore_r == 0)M4823<CTLcSLO_87LK_CNCFG_TX,, &D) =}8d 2n3_SP84823<BL(valRATE_V	re15P9HZ},eep{if (TANnd bit 	00FFFB 	00FFFD,
	}nx
erom(e_eeprom;_c== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858)t>lin_SFS0G LED5 souort"F_MSGPCS stROM_FC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warpcore_r == 0)M481_LED5ng mo02x_war0x90_reSGlbp3_bbnx2xbate 
	bx000fer_R first((va7bp3_bbnx2xbate 
	bx000DR_A} n_SFS0G LED3 BL(vat"KR PCS st_PMA_ is
	 MDIO_PMA_REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<M481_LED3nBW_PARAM__S7bp3_bbnx2xbatewerarMA_S_EC<CTLcLEDDCTLm(struct bnROM_FDEVAD, 0xc809, &D) =}8d 2nd bit , &D) =}8d 2n3_SP8481	L(val[IGNAL			   time MDIO_P00FE00;
oR,
				bx0092nx
erom(e_eeprom;_c== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858)
otR,
				2 <<r12;x2x_LED5 ON		POR(  */souort"F_MR PCS st_PMA_ is
	 MDIO_PMA_REGj<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<M481_L(val[IGNAL		hy *phRSWO_WIRE_CTRL, &vARRAY_SIZE(reg"10G)727_re
Eeuct bnIO_PMC_TX_TECH_ADDR reg"10G[i].devadL reg"10G[i].reu	
 moO reg"10G[i].hy *phRSol t			   is_s483x_s485x_smisf
 mhy_idxport45_r<<<<<<<<M4833<CTLcLEDDCTL_  SFDENTIFIhy_idxport45_r<<<<<<<<M4823<CTLcLEDDCTL_  S
erom(e_eeprom;_c== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858)
otR,
	ort45_r<<<<<<<<M4858_ALLO_8GS_EXACT |n"	 FG_Tv status *<<<<M4823<LED3nSTRETCH_EN;p
 * restR,
	ort45_r<<<<<<<<M4823<LED3nSTRETCH_EN;parMA_"Seetch_enyfARpLED_/ Set the read comma_orFC_TX_TECH_ADDR];

		if (TB	rc = bnx2x_warx2x_reater to<<<<<<<<<*params_up) {
	/* M48xxderead(bp, phy,p = 0;
	u16 val1, rx_sd;
	sto %d\*bp = params->bp;
	DP(NETIF_MSG_LINK,( venaL;
	}bp))
EPROM_VENDOR_NAME_SIZE+1];
	char"p_modulaL;
	}b		break;
PEVAD, M:
	rol t			   is_s483x_s485x_smisf ((van_SFSavp eti us strs
	}P
	fw_tuct bn avpoP48xxdeti us(strs
	},ADDR rt].sfp_ctrlFRCMwer_o}A_tficth_sf l1,usea moduNIG lamodumenx2xismP_EEPRO ;
	"y, Set GPItercomarrivms_mornx2x_lis LED4INPUTr, pv	b lis L &rx_ gnal,/so wt dPMA_{
	 ;
	adyp* gnalOed;
	ad of_al);

on/
starordcurreuct bn>lis_enort].NDUAre
	LATCH_BC_0 +.sfp_ctrlFRCM*4xHW_CFG_Tv 1 <<rNDUALATCH_BC_AM, &v_MI4&= .c8(val & MDM48xxdeetxlbp ECH_ADD_reSGP	read_f} ms *params,
					   M48xxdcmn  case ot bn, u8 *o_buf)
{
	int rc = 0;
	oo %d\*bp = params->bp;
	DP(NETIF_MSG_LINK,(	struct bnx2xNK, "read statuSG_LINK,
		   "Reading from eeprom iautoneg_<<<, an	netd_<<<, an	ne	netGe tra
	{
	/* M48xxderead(bp, phy,data += xfer_PEVAD, Mwer_ave microcode, hence the _REGj<<<<<<<<<<<<<<<	bnx2x_cl45_reD_10000) 005_read(bpidxpnetd
1);
		advertiseme((byteruct bnROM_FDEVAD, 0xc809, &D) =}8ANnd bit 	bnx2xANnnx2_<481_netdt offsetof(pan	netd_<<<ct at the rextval12f:ufpafor  detected\n")ead s;
Iuct bnROM_FDEVAD, 0xc809, &D) =}8ANnd bit , &D) =}8ANn<<<<M481_LEGACY8ANnADV,tof(pan	ne	netGe ts;
Iuct bnROM_FDEVAD, 0xc809, &D) =}8ANnd bit 	bnx2xANnnx2_<481_LEGACY8MIbn<<<<<<<<<pautoneg_<<<wer_fic(pmd_disgportd",
				ontrautoneg_<<<iO_PMA par6)a| _phy8)a| _phy9)a| _phy12)a| _phy13e	  [an	ne	netGe tiO_PMA par5)a| _phy6)a| _phy7)a| _phy8)*phRSol tyto OUTable to determin		}
				AUTO_NEGted\n" ;
			s OUTeterm_cax_8ask SFPEEPRO bnx2x_8727_}
				CAPABILITY_D0_1Gtte||n";
			];
		able to determin		}
				netd	le_eepan	netd_<<<i			 val8s;
Eeautoneg_<<<i			 val9a| phy12);
Eerom(e_eepableduplepess DUPLEX_FULtm<<<<an	netd_<<<i			 val9val[S{
				if (params->phAdvertisoppeMG
		respR firseepan	netd_<<<iO_PMA par8)a| _phy9)ct at the read cde, hence the _REGj<<<<<ANnd bit 	bnx2xANnnx2_<481_netdt offsetof( an	netd_<<<ct at_SFS0G 10/1td
1);
		advertiseme((byterol to OUTable to determin		}
				AUTO_NEGte{
Eerom(e_eepeterm_cax_8ask SFP_EEPR bnx2x_8727_}
				CAPABILITY_D0_100M_FULtm ((van_SF nion
 autonegsNPUTrO_OFRCMautonegsfARp)egacy
1);
	stof( 
	fw_tautoneg_<<<i			 val9a| phy12);
Ee[an	ne	netGe ti			 val8s;
Eer{
				if (params->phAdvertisoppeM00M-FDfw_resp _P
Eerom(e_eepeterm_cax_8ask SFP_EEPR bnx2x_8727_}
				CAPABILITY_D0_100M_HALFm ((van_SF nion
 autonegsNPUTrO_OFRCMautonegsfARp)egacy
1);
	stof( 
	fw_tautoneg_<<<i			 val9a| phy12);
Ee[an	ne	netGe ti			 val7timeot;
	/* Low ==> corresAdvertisoppeM00M-HDfw_resp _P
Eerom(to OUTeterm_cax_8ask SFP_EEPRO bnx2x_8727_}
				CAPABILITY_D0_10M_FULtm d\n");
	m(e_eepey->type	/&sSUP bnx			ne	PORT_Fullsf ((vanan	ne	netGe ti			 val6);
Ee[autoneg_<<<i			 val9a| phy12);
Ee[;
	/* Low ==> corresAdvertisoppeM0M-FDfw_resp _P
Eerom(to OUTeterm_cax_8ask SFP_EEPRO bnx2x_8727_}
				CAPABILITY_D0_10M_HALFm d\n");
	m(e_eepey->type	/&sSUP bnx			ne	PORT_Halfsf ((vanan	ne	netGe ti			 val5);
Ee[autoneg_<<<i			 val9a| phy12);
Ee[;
	/* Low ==> corresAdvertisoppeM0M-HDfw_resp _P_Rd cablO inp10/1td
e GPalTERG_SFP_vor	"y, FORCE_OUTP	ontrol t	s OUTable to determin		}
				net
	d\n";
			s OUTey->type	/&n" ;
			SUP bnx			nee	PORT_Half |n";
		  SUP bnx			nee	PORT_Fullsfle_eepautoneg_<<<i			 val13wer_	MA_ nion
d AUTO-bnxX;rn -Eautonegs stcation
	-curreu8 tx_ROM_FC_TX_TECH_ADDR];

		if (TANnd bit 	bnx2xANnnx2_<481_AUXf);
	,tof(st_phy15a| phy9a| 7ar0)t;A_tficthiskEVTnEG_s 
his *e(beve
sfARpgportd"bnx2.A	bnx2an	ne	netGe ti			 val8sa| _phy7)al[S{
				if (params->ph
		coppeM00M gport = val[}trol t	s OUTable to determin		}
				ne
	d\n";
			s OUTey->type	/&n" ;
			SUP bnx			ne	PORT_Half |n";
		  SUP bnx			ne	PORT_Fullsfle_eepMA_ nion
d AUTO-bnxX;rn -Eautonegs stcation
	-curreu8 tx_ROM_FC_TX_TECH_ADDR];

		if (TANnd bit 	bnx2xANnnx2_<481_AUXf);
	,tof(st_phy15a| phy9a| 7ar0)t;A_t{
				if (params->ph
		coppeM0M gport = val[}tat the read cde, hence the _REGj<<<<<ANnd bit 	bnx2xANnnx2_<481_LEGACY8ANnADV,tof( an	ne	netGe ts;

erom(e_eepableduplepess DUPLEX_FULtm<<<autoneg_<<<i			 val8ct at_SFAlwiy->de, h 
his _f_mois iodn, pM4833/4.O co FpoiM4833/4,>de, h i	 s inprn -Eit's asgportd",
			.ve	ontrol t>			   is_s483x_s485x_smise||n";
			(autoneg_<<<iO _phy12)f ;
			e
Eeuct bnIO_PMC_TX_TECH_ADDR_REGj<<<<<ANnd bit _REGj<<<<<ANnnx2_<481_LEGACY8MIbn<<<<< autoneg_<<<werRSol tyto OUTable to determin		}
				AUTO_NEGted\n" ;
		s OUTeterm_cax_8ask SFPEEPRO bnx2x_8727_}
				CAPABILITY_D0_10Gtte||n"	to OUTable to determin		}
				netd0f
			if (phy->req_line_speedAdvertisoppeM0G
		resp rac-R__OFRCMautonegsfARpM0Gyte*/
t the read comma_orFC_TX_Ttof(shy,
			MDIO_PA_DEVANnd bit DIO_PA_DEVANnnx2_<481_neGBASE_TVANn);
	,tof(s0)1addr32, ruct bnIO_PMC_TX_TECH_ADDR];

			if (TANnd bit 	bnx2xANnnx2_C_100];

			0x3200val[d firsrreu8 tx_ROM_FC_TX_TECH_ADDR];

		if (TANnd bit ];

		if (TANnnx2_<481_neGBASE_TVANn);
	,tof(sfrwerar *param0rams *params,
					    481_ case ot bn, u8 *o_buf)
{
	int rc = 0;
	o*bp = params->bp;
	DP(NETIF_MSG_LIstruct bnx2xNK, "read statuEPROM_VENDOR_NAME_SIZE+1];
	charac-R__Oman_norm<<iplPIO_OUTP(struct bnx2x_LASER_MDIs->port);
	bnx2x_cl42+ module isn = tx_en_mode - PORT_HW_CFG_TX,dT_MDL_ENFRCMwerr_ficHWTx_f:ut"KR PCS stextval12;
	bnx2x_rt].sfp_ctrlFRCMwer_ave mival;
x_f:uf		MDIO_Pence the rtruct weradx2x *bcrocode, hence the rt45_rd 2nd bit  bnx2x_cl45_reD_1000par15;t h *paramars *va48xxdcmn  case ot bn,data += xfer_ead s;
ms #defto skEVa48xxdCMDHDLR_WAIT 300 #defto skEVa48xxdCMDHDLR_MAX_ARGS 5s *params,
					    4858dcmd_hdlr_"Setting SFP+ transmitter t_param;
	char vendor_name[tter tx_clfw(cmdtter tx_clcmd_args[],s,
		argcstatu,
		idx;x2x_clor as EPROM_VENDOR_NAME_SIZE+1];
	chaarMA_Step 1: PlIP tn  [tule_-333);
	}
FP_ ieprn COMPecase x_vioustrummNPUO co SIVE)e xogress orMmodusystem	SIVbusy (CMD_IN_PROGRESSr-9 dco SY_moM_BUSY)._Pfp x_vioustrummNPU SIVE)e xogress orMsystem	SIVbusy,d comcb valagaE)e,ntil case x_vioustrummNPU ftoishes execu;
	} NPUTCOMd comsystem	SIVavaild_disgpo takopperummNPUO co/hRSWO_WIRdxE_CTRL,dxE<skEVa48xxdCMDHDLR_WAITRL,dx	if ((val & MDIO_PMA_REG_SFP_TWOj<<<<<CTLc = bnxDIO_PA_DEVa48xxdCMD_HDLR_[tule_, p   timeout *(ansh!== EVa4858d[tule__CMD_IN_PROGRESSm d\n");
	m(ansh!== EVa4858d[tule__CMD_SY_moM_BUSY)tturn	REG_SFP_T *phy,
				 struct linval[}trol t,dxE>=skEVa48xxdCMDHDLR_WAIT  ((va;
	/* Low ==> corresFWlcmd: FWln, pA_REy.
		resp _DEV_AD-EINVALer_Rd cablStep2:_PfpanySIZE+1e
	}e are "FW ibndsgpo tase phy;
	},Cde, h 
hem
is 1
	 modu33W ibndsDATATx_3);
	}eO co/hRSWO_WIRdxE_CTRL,dxE<sargcRL,dx	if ((val & MDIO_PMde, hence the rt45_rCTLc = bnxDIO_P A_DEVa48xxdCMD_HDLR_DATA1 +L,dxxDIO_P cmd_args[,dx]val[ramsOUTStep3:	Wn -EcasefirmwaPe iodA_REysgpo rummNPUs,Cde, h 
he 'CummNPUO co  0;
'1
	 moduCMDTx_3);
	}rom(struct bnROM_Fde, hence the rt45_rCTLc = bnxDIO_ A_DEVa48xxdCMD_HDLR_COMMAND,lfw(cmd)haarMA_Step4:lO PROmodurummNPU has be -Ede, tenFP_CIP tn  [tule_-333);
	}
is 1
	 cb valrn COMPecaserummNPU has 		MDIO_Pd (CMD_COMPLEx			PASS/
is 1CMD_FORdCMDSr-9 CMD_COMPLEx			ERROR).O co/hRSWO_WIRdxE_CTRL,dxE<skEVa48xxdCMDHDLR_WAITRL,dx	if ((val & MDIO_PMA_REG_SFP_TWOj<<<<<CTLc = bnxDIO_PA_DEVa48xxdCMD_HDLR_[tule_, p   timeout *(ansh=== EVa4858d[tule__CMD_COMPLEx		PASSte||n"	;
	m(ansh=== EVa4858d[tule__CMD_COMPLEx		ERROR)tturn	REG_SFP_T *phy,
				 struct linval[}trol tt,dxE>=skEVa48xxdCMDHDLR_WAIT  ||n";
			ansh=== EVa4858d[tule__CMD_COMPLEx		ERROR)t ((va;
	/* Low ==> corresFWlcmd f_FP		.
		resp _DEV_AD-EINVALer_RdrMA_Step5:lO PROmodurummNPU has 		MDIO_Pd,/
statmoduspecficindsDATAd com333);
	}esgpo anyS avpUTrO_ultssgpo taserummNPU,isf_applicd_diO co/hRSMA_GaCOMPe_DEV_Aoppedatam(strWO_WIRdxE_CTRL,dxE<sargcRL,dx	if ((val & MDIO_PMA_REG_SFP_TWOj<<<<<CTLc = bnxDIO_PA_DEVa48xxdCMD_HDLR_DATA1 +L,dxxDIO_P&cmd_args[,dx]val[rams *param0rams *params,
					    4833dcmd_hdlr_"Setting SFP+ transmitter t_param;
	char vendor_name[t x_clfw(cmdtter tx_clcmd_args[],s,
		argc,s,
		 xowessstatu,
		idx;x2x_clor as EPROM_VENDOR_NAME_SIZE+1];
	chau,
		rc;
	d		
erom(exowessh=== EVa4833dMB_PROCESS2le_eepMA_We, h CMD_OPENAOuf:RI D1
	 [tule_-333EF_MSGPCS stROM_FC_TX_TECH_ADDRrt45_rCTLc = bnxDIO_P A_DEVa48xxdCMD_HDLR_[tule_,DIO_P  EVa4833d[tule__CMD_OPENAOuf:RI Dval[ramsWO_WIRdxE_CTRL,dxE<skEVa48xxdCMDHDLR_WAITRL,dx	if ((val & MDIO_PMA_REG_SFP_TWOj<<<<<CTLc = bnxDIO_PA_DEVa48xxdCMD_HDLR_[tule_, p   timeout *ansh=== EVa4833d[tule__CMD_OPENAFORdCMDStturn	REG_SFP_T *phy,
				 struct linval[}trol t,dxE>=skEVa48xxdCMDHDLR_WAIT  ((va;
	/* Low ==> corresFWlcmd: FWln, pA_REy.
		resp MA__f_modu,link_/_s CMD_COMPLEx		PASSr-9 CMD_COMPLEx		ERROR
dPMA_al);


hdu,link_/
	 CMD_CLEAR_COMPLEx	
ordcurrerom(ansh=== EVa4833d[tule__CMD_COMPLEx		PASSr||n"	;
	mansh=== EVa4833d[tule__CMD_COMPLEx		ERROR) ((va PCS status C_TX_TECH_ADDRrt45_rCTLc = bnxDIO_PP A_DEVa48xxdCMD_HDLR_[tule_,DIO_PP  EVa4833d[tule__CMD_CLEAR_COMPLEx	wer_o}A_t_DEV_AD-EINVALer_Rdrrom(exowessh=== EVa4833dMB_PROCESS1 ||n";
		exowessh=== EVa4833dMB_PROCESS2le_eepMA_Prepe GPargume(((s)A	bnx2WO_WIRdxE_CTRL,dxE<sargcRL,dx	if ((va PCS status C_TX_TECH_ADDRrt45_rCTLc = bnxDIO_PP A_DEVa48xxdCMD_HDLR_DATA1 +L,dxxDIO_PP cmd_args[,dx]val[ _P_Rd cPCS status C_TX_TECH_ADDRrt45_rCTLc = bnxDIO_A_DEVa48xxdCMD_HDLR_COMMAND,lfw(cmd)hasWO_WIRdxE_CTRL,dxE<skEVa48xxdCMDHDLR_WAITRL,dx	if ((val & MDIO_PMA_REG_SFP_TWOj<<<<<CTLc = bnxDIO_PA_DEVa48xxdCMD_HDLR_[tule_, p   timeout *(ansh=== EVa4833d[tule__CMD_COMPLEx		PASSte||n"	;
	m(ansh=== EVa4833d[tule__CMD_COMPLEx		ERROR)tturn	REG_SFP_T *phy,
				 struct linval[}trol tt,dxE>=skEVa48xxdCMDHDLR_WAIT  ||n";
			ansh=== EVa4833d[tule__CMD_COMPLEx		ERROR)t ((va;
	/* Low ==> corresFWlcmd f_FP		.
		resp _c;
	-EINVALer_Rdrrom(exowessh=== EVa4833dMB_PROCESS3_&& _c;

			returMA_GaCOMPe_DEV_Aoppedatam(str2WO_WIRdxE_CTRL,dxE<sargcRL,dx	if ((va PCS status A_REG_SFP_TWOj<<<<<CTLc = bnxDIO_PPA_DEVa48xxdCMD_HDLR_DATA1 +L,dxxDIO_PP&cmd_args[,dx]val[0,
	}msrom(ansh=== EVa4833d[tule__CMD_COMPLEx		ERROR ||n";
		ansh=== EVa4833d[tule__CMD_COMPLEx		PASSte((val & MDIO_PMde, hence the rt45_rCTLc = bnxDIO_P A_DEVa48xxdCMD_HDLR_[tule_,DIO_P  EVa4833d[tule__CMD_CLEAR_COMPLEx	wer_ras *paramrcrams *params,
					    48xxdcmd_hdlr_"Setting SFP+ transmitter t_param;
	char vendor_name[tter tx_clfw(cmdtter t";
	x_clcmd_args[],s,
		argc,ter t";
	,
		 xowessstatuEPROM_VENDOR_NAME_SIZE+1];
	chaarol t	s OUTrom;_c== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858)t||n";
			nfig[port].sfp_ctrl)) &
2		PORT_HW_;
		al & PORT_FEAT_CFG_OP2IF_MSG_LINK,le is
	chaattr_syncPT_MDL_ENFRCMN;
	dFPEEPROL(valATTR_P4858)we((va *paramars *va4858dcmd_hdlr_data += xfer_fw(cmdtlcmd_argsLED moO regargcser_R first((va *paramars *va4833dcmd_hdlr_data += xfer_fw(cmdtlcmd_argsLED moO regargc,	 xowesssd_f} ms *params,
					   M48xxdpair_swax_cfg, u8 *o_buf)
{
	int rc = 0;
	oo %*bp = params->bp;
	DP(NETIF_MSG_LINK,struct bnx2xNK, "read statusG_Rpair_swax;x2x_cldata[kEVa48xxdCMDHDLR_MAX_ARGS]hau,
		 link_;arEPROM_VENDOR_NAME_SIZE+1];
	chaarMA_ = valgpo ruO_ENu t GPI.A	bnxpair_swax_config[port].sfp_ctrl)) &
		PORT_HW_CFG_X_LASER_MASK;
	DP(NETIF_MSG_LINK,date media type for non-PMF sync */
	fxgbtval12cfg;
	dFP	 bnx2x_8727_RJ_PMPAIR2SWAPDE_SHIFtrol tpair_swax_c
			sp _DEV_ADDR_l[ac-O inp
hdu,__NOdPargume(("y->fsndsgpo taistrummNPU 	bnxdata[1]E_S(x_c)pair_swax;xl[EP	sk_/
				    48xxdcmd_hdlr_data += xfer_MSG_LINK,skEVa48xxdCMD_SETMPAIR2SWAP,ldatar_MSG_LINK,s2,= EVa4833dMB_PROCESS2l;trol tEP	sk_/

			sp ;
	/* Low ==> corresPairswax_OK		hy =	bnx2x_cldata[1]werar *param link_;acfg;
	struk_vars *va4833dge;
x_f:ufLASEs = params->bpfor LED moLINK,( ven)) &
		PORs->th[]LED moLINK,( venchip_idstatusG_Rx_f:ufpin[2]hausG_Ridx;x2x8 x_f:ufLASEs;trol t"Setting 3paramreturMA_Assume tha
	
hdORTwiIP bet), aer_n, pE, ae.m(str2WO_WIRdxE_CTRL,dxE<s2RL,dx	if ((va ype ax_or non += xfpmo/
s3);
	}
biV.Aontrp _Df:ufpin[,dx]_config[port].)) &
		PORs->th[,dx]__HW_C	X_LASER_MASK;
	DP(NETIF_MSG_LINK,,date media type for non-0].e3dcmn pin2cfg;
;trp _Df:ufpin[,dx]_co(_Df:ufpin[,dx]_&INK,, bnx2x_8727_E3XS_EXnxSEing modu>>INK,, bnx2x_8727_E3XS_EXnxSEinSHIFT;trp _Df:ufpin[,dx]_-== IN8727_), a0_P0;trp _Df:ufpin[,dx]_co(1 <<r_Df:ufpin[,dx]wer_o}A_t_Df:ufLASEsE_S(x8)(_Df:ufpin[0] |r_Df:ufpin[1]val[r first>lin_SFE2,=looalg us diff plaPRr-f_P(NET.m(str2WO_WIRdxE_CTRL,dxE<s2RL,dx	if ((va _Df:ufpin[,dx]_config[port].)) &
		PORs->th[,dx]__HW_C	X_LASER_MASK;
	DP(NETIF_MSG_LINK,,date media type for non-0].>ereadd2cfg;
;trp _Df:ufpin[,dx]_&== bnx2x_8727__XGXS_EX- PORRSTDE_SHIFrp _Df:ufpin[,dx]_-== bnx2x_8727__XGXS_EX- PORRSTD), a0_P0;trp _Df:ufpin[,dx]_>>== bnx2x_8727__XGXS_EX- PORRSTDSHIFT;trp _Df:ufpin[,dx]_co(1 <<r_Df:ufpin[,dx]wer_o}A_t_Df:ufLASEsE_S(x8)(_Df:ufpin[0] |r_Df:ufpin[1]val[rrar *paramx_f:ufLASEs;tms *params,
					    4833d;
	bnx2xval1_"Setting SFP+ transmitter t_param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	char  8x_f:ufLASEs;trsG_RoCOMP_)) &
		PORsaddr_config[port].sfp_ctrl)) &
2		PORT_HW_C	X_LASER_MASK;
	DP(NET2IF_MSG_LINK,	oCOMP_)) &
		PORsaddr)ct at ven)) &
		PORs->th[2]haIR/o Wor	"aroundsgpo s4833sLEDuf_FPu * insida nxSEi",link_/ Set the read cde, hence the rt45_rANnd bit ];
bnx2xANnnx2_<481_LEGACY8MIbn<<<<<<<<bnx2xANnnx2_<481_MIbn<<<<_FORCE_1Gter_ave microcode, hence the rt45_rANnd bit ];
bnx2xANnnx2_<481_1reSG0T__XGX<<<<<<<<bID2xANnnx2_<481__XGX<<<<_FORCE_LEDSROFFct at)) &
		PORs->th[0]E_SIZE+1];
)) &
		POR;at)) &
		PORs->th[1]E_SoCOMP_)) &
		PORsaddrerar *f:ufLASEsE_Sars *va4833dge;
x_f:ufLASEs rt].)) &
		PORs->thLED moO	 reZE+1];
chip_idsalTBuct bnx:ufmadd2LASER_MDIx_f:ufLASEsDIs->port);
	bnx2x_cl4T_HW_CFLOWter_udelay(10resp;
	/* Low ==> corresM4833shwTx_f:utoMP_in	hy ue

	bnx2x_cA_t_Df:ufLASEswerar *param0rams *params,
					    483x_cation
_eee, u8 *o_buf)
{
	int rc = 0;
	oo*bp = params->bp;
	DP(NETIF_MSG_LINstruct bnx2xNK, "read statu,
		rcas EPROM_VENDOR_NAME_SIZE+1];
	chaux_clcmd_args;
	d		
e;
	/* Low ==> corresDon't_Advertis iy[iBPOR-T EEEfw_resarMA_Preve
	 Ptraf us vor	oppein	EEElNPUTadvertisoppeiut"KR _c;
				    48xxdcmd_hdlr_data += xferskEVa48xxdCMD_SETMEEEDEADDF_MSG_LI&cmd_args, 1,= EVa4833dMB_PROCESS1l;trol trc  ((va;
	/* Low ==> corres EEldpmd_disf_FP		.
		resp _DEV_ADrcas rrar *paramPCS steee_cation
,data += xfer_ead s;
ms *params,
					    483x_ower_u_eee, u8 *o_buf)
{
	int rc = 0;
	oo*bp = params->bp;
	DP(NETIF_MSG_LINstruct bnx2xNK, "read statu,
		rcas EPROM_VENDOR_NAME_SIZE+1];
	chaux_clcmd_args;
	  S
e_c;
				    48xxdcmd_hdlr_data += xferskEVa48xxdCMD_SETMEEEDEADDF_MSG_LI&cmd_args, 1,= EVa4833dMB_PROCESS1l;trol trc  ((va;
	/* Low ==> corres EElower_upf_FP		.
		resp _DEV_ADrcas rrar *paramPCS steee_advertise,data += xfer_ead , SHMEMMEEEDSG_LADVs;
ms #defto skEVa4833<CONSTANTALATENCY 1193 *params,
					   M48x3  case ot bn, u8 *o_buf)
{
	int rc = 0;
	oo*bp = params->bp;
	DP(NETIF_MSG_LINstruct bnx2xNK, "read statuEPROM_VENDOR_NAME_SIZE+1];
	char  8_CFG_Ot bnx2x *b     SFx_clor as  venaL;ualval12f:lAL;
	}haux_clcmd_args[kEVa48xxdCMDHDLR_MAX_ARGS]hau,
		rc;
	d		
eT *phy,
				 struct linvalms(bp->t"Setting 1xparam	sp tter ( BP_PATHpara;p
 * resttter ( TX laser of th
erom(e_eeprom;_c== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4823te((val & MDx2x_LASER_MDIs->port);
	bnx2x_cl43xHW_CFG_Tv sn = tx_en_mode - PORT_HW_CFG_TX,HW_CFG_Tv sFRCMwer_R first((vaype nx2Tx_f:ut"KR eu8 tx_ROM_FC_TX_TECH_ADDR];

	_CTRL_STATUS_MA_WIRE_CTRL_STA3_SPD_10000)8linval[}tr_ave mival;
x_f:uf		MDIO_Pence the rtruct werad(bp, phyfARpGkEVT_otrume
ut  of_33f:ut"KR m *phy(50l;trol t>			   is_s483x_s485x_smismreturMA__TYP4823u33W ibns tha
	W_CF bnx2->fldfK,
		@iy[idfARpnorm<<
dPMA_behavior.
er-curre<4);.emchar	.emc ( R,d :
	{
< 1);
	har	R,d :
	{
< 1);
				}
				netd0			Il & MDx2x_autoneg(&TX laser hy[INGXS_E]a += xfer_ead , 0_reSGPCS st xogramDx2rdes(&TX laser hy[INGXS_E]a += xfer_ead )har	R,d :
	{
< 1);
				.emchar}arMA_ = val_f_mois iodaL;ually _TYP4858byterol to OUTrom;_!== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858)retur<4);;
	bnvc8(val & MDIO_PMA_REG_SFP_TWOj<<<<<ANnd bit DIO_PA_DEVANnnx2_<48xxdIDw =B			;
	bnvtimeout *;
	bnv_c==_TYP4858<PIO1:
f ((va TX laser
	chaattr_synci			L(valATTR_P4858;(va PCS stupdt t_
	chaattr  detectedX laser
	chaattr_syncresp _P_Rd cablS0G  ver-}
		b ruO_ENu t GPIEPRworde stao ruO_ENu t GPIEyteruct bnROM_FDEVAD, 0xc809rt45_rCTLc = bnxDIO_A_DEVCTLc<<<<M4823<turn 			   time MDIO_P~(A_DEVCTLc<<<<M4823<turn _MACDE_SH |n"	 A_DEVCTLc<<<<M4823<turn _L(vEDE_SH |n"	 A_DEVCTLc<<<<M4823<turn _COPPER_CORE_DOWN |n"	 A_DEVCTLc<<<<M4823<turn _PRIORITY_E_SH |n"	 A_DEVCTLc<<<<M4823<turn __spee_1Gtertrol t"Setting 3paramretur MDIO_P~(A_DEVCTLc<<<<M4823<turn _MACDE_SH |n"		 A_DEVCTLc<<<<M4823<turn _L(vEDE_SHwer_R first((va<<<i			 A_DEVCTLc<<<<M4823<<<<<_MACDXFI |n"		A_DEVCTLc<<<<M4823<turn _L(vEDXAUI_Lval[}tr_aL;ualval12f:lAL;
	};
				   al12f:lAL;
	}(truct werad"p_modulaL;ualval12f:lAL;
	}b		break;
Pbnx2x_8727_PIO1SELECG;
	_HARDWARE_DEFAULM:
	rfic(ofr, he s. Esse((or inpmois iodliktecase xiorl45_copppow"KR euREG_SFPeak;
Pbnx2x_8727_PIO1SELECG;
	_FIRSTDPIO1PRIORITY:(va<<<i			A_DEVCTLc<<<<M4823<turn _PRIORITY_COPPER;R euREG_SFPeak;
Pbnx2x_8727_PIO1SELECG;
	_SECONDDPIO1PRIORITY:(va<<<i			A_DEVCTLc<<<<M4823<turn _PRIORITY__spee;R euREG_SFPeak;
Pbnx2x_8727_PIO1SELECG;
	_FIRSTDPIO:
	rfic(ofr, he s OMPe.cthisfK,
		kEVTwon't_bept bnx2x *bp a
	aIPe"KR euREG_SFPeak;
Pbnx2x_8727_PIO1SELECG;
	_SECONDDPIO:(va<<<i			A_DEVCTLc<<<<M4823<turn _COPPER_CORE_DOWNimeou bnx2x *b   0;R euREG_SFPRdrrom(eX laser hy[_XGXS_E2].rele to determin		}
				netd	(va<<<i			A_DEVCTLc<<<<M4823<turn __spee_1Geradx2x *bcrocode, hence the rt45_rCTLc = bnxDIO_ A_DEVCTLc<<<<M4823<turn 		   time;
	/* Low ==> corresMaddi{
	inor non =rm_ba,eM
		b ruO IOses 	bnx2x_c
;BC Ce_MDL_ENmaddi{
	i  case 		hy *phRSol t			   is_s483x_s485x_smisf ((val & MDM48xxdpair_swax_cfg,data += xfer_ead s;

	rficKphy AutogrEEEntcation
	.m(str2cmd_args[0]E_S0x0;R ecmd_args[1]E_S0x0;R ecmd_args[2]E_SkEVa4833<CONSTANTALATENCY +   SF0cmd_args[3]E_SkEVa4833<CONSTANTALATENCYesp _c;
				    48xxdcmd_hdlr_data += xfer_MSG_	,skEVa48xxdCMD_SETMEEEDEADDFlcmd_argsLED moO r4,= EVa4833dMB_PROCESS1l;trrol trc ED m;
	/* Low ==> corresCfg AutogrEEEntf_FP		.
		resp}trol t, bnx2x *b)sp _c;
				    48xxdcmn  case ot bn,data += xfer_ead s;
	firsrreu8 tx_ avpoP48xxdeti us(strs
	},ADDR rt].sfp_ctrlFRCMwer_fic84833sPEV has a_beelse
ORT_FEAlNPUTdoesn't_nEG_SFP_ey->typpmois.byterol to OUTrom;_c== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4823te((va vencms_ower_u_config[port].sfp_ctrl)) &
		PORT_HW_CX_LASER_MASK;
	DP(NETIF_MSG_LINK,date media type for non-PMF sync */
	f>ereadd2cfg;
_&INK, bnx2x_8727__M, &v_CMSDE_SHIFtral & MDIO_PMA_REG_SFP_TWOj<<<<<CTLc = bnxDIO_PA_DEVCTLc<<<<M4823<UaraD<<<<_5_r00&45_l;trrol tcms_ower_ulfw_tR,
i			A_DEVCTLc<<<<M4823<UaraD<<<<_CMS;trrDENTIFIER,
IO_P~A_DEVCTLc<<<<M4823<UaraD<<<<_CMS;trrl & MDIO_PMde, hence the rt45_rCTLc = bnxDIO_P A_DEVCTLc<<<<M4823<UaraD<<<<_5_r0045_l;tr} at the read command byte crt45_rCTLc = bnxDIO_A_DEVa4833dTOP8727_FW_5_V00&45_l;tarMA_ NO_ENu *  EEley->typpontrol t	R,
I>= A_DEVa4833dTOP8727_FW_ EEted\n" ;
		ansh!==A_DEVa4833dTOP8727_FW_NO_ EEted\n" ;
	PCS steee_has_cax-EINVAL;
	{sp _c;
				   eee_, bnx2xfor non(+= xfer_ead , SHMEMMEEEDSG_LADVs;
	rol trc  ((vaa;
	/* Low ==> corresF_FP		tao ruO_ENu *  EEltimtrs
		resp ruct bn 483x_cation
_eee,data += xfer_ead s;
	p _DEV_ADrcas  _P
Eerom(to OUTableduplepess DUPLEX_FULtm d\n");
	m(eMF synceee_OUTP	& EEEDEADDLADV_LPIm d\n");
	m(			   eee_calc_timtr-EINVAL;e||n"	;
	m !(eMF synceee_OUTP	& EEEDEADDL_M, &v_LPIm)tturn_c;
				    483x_ower_u_eee,data += xfer_ead s;
	pDENTIFIE_c;
				    483x_cation
_eee,data += xfer_ead s;
	pol trc  ((vaa;
	/* Low ==> corresF_FP		tao f:utEEElNdvertiseme((
		resp r_DEV_ADrcas  _P_R first((va<<rynceee_Xlink_/&_P~SHMEMMEEEDSUP bnx			E_SHIFr}h
erom(e_eeprom;_c== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4833mreturMA_Addi;
	}nshs		coppssgpo jumbo +=ckets/_up1000BASE-T_OUTP	ontrt_SFAllow rx extendtd"bength-curreu8 tx_ROM_FA_REG_SFP_TWOrt45_rANnd bit DIO_PA_DEVANnnx2_<481_AUXf);
	,0&45_l;trrR,
				bx4td0			Il & MDcrocode, hence the rt45_rANnd bit ];

		if (TANnnx2_<481_AUXf);
	,045_l;trrfictX FIFO Elasticl45_LSB-curreu8 tx_ROM_FA_REG_SFP_TWOrt45_rANnd bit DIO_PA_DEVANnnx2_<481_1reSG0T__XGX<<<<<0&45_l;trrR,
				bx1			Il & MDcrocode, hence the rt45_rANnd bit ];

		if (TANnnx2_<481_1reSG0T__XGX<<<<<045_l;trrfictX FIFO Elasticl45_MSB-curre_SF nion
 expans
	}P333);
	}
ox46 (Paelsen GeneratorMslink_wEF_MSGPCS stROM_FC_TX_TECH_ADDRrt45_rANnd bit ];

		if (TANnnx2_<481_EXPANS;
	_nx2_ACCESS000)f46)c8(val & MDIO_PMA_REG_SFP_TWOj<<<<<ANnd bit DIO_PA_DEVANnnx2_<481_EXPANS;
	_nx2_RD_RW,0&45_l;trrR,
				bx4td0			Il & MDcrocode, hence the rt45_rANnd bit ];

		if (TANnnx2_<481_EXPANS;
	_nx2_RD_RW,045_l;tr} atol t			   is_s483x_s485x_smisf ((vaMA__re s PEV ut  of_ey-powfsolate OUTP	va modufi}nshstep.-curreu8 tx_ROM_FA_RE_NPUMC_TX_TECH_ADDR];

			rt45_rCTLc = bnxDIO_PP =A_DEVa4833dTOP8727_XGS_EXSTRAP1,
 moO	 r(x_c)~A_DEVa4833dSUPraDISOLATEwer_ras *paramrcrams *paramsk_vars *va48xxXGXS_EXlink_up = 0;
	u16 val1, rx_sd;
	sto*bp = params->bp;
	DP(NETIF_MSG_LIstruct bnx2xNK, "read statuEPROM_VENDOR_NAME_SIZE+1];
	char<4);eep,    1,    2;ar  8705\n");
	balT
rMA_ = valSG_-BPORTlp;
	",link_/ SetMA_ = valPMDp* gnalOokbyteruct bnROM_FDEVAD, 0xc809, &D) =}8ANnd bit 	00FFFA			   1val[uct bnROM_FDEVAD, 0xc809, &D) =}8d 2nd bit <<<<<<<<<<<<<<M481_PMD_SIGNAL	, &D	   2time;
	/* Low ==> corres_TYP48xx: PMD_SIGNAL 1.a811es 	bnx2x_cl45l2)haarMA_ = valp;
	"y[idontrol tR,l2iO _phy11amretur Md :
	{
< 1);
				}
				netd0			IR,d :
duplepes DUPLEX_FULtachI705\n");
	1al[SPCS stextval12SG_Lan2x_folvpence the read s;
	R first( MA_ = valLegacy
1);
	lp;
	"curre<4);)egacy8 link_,t egacy8 );
	harre_SF nion
 expans
	}P333);
	}
ox42 (Ope t GPIEOUTP	slink_wEF_MSGPCS stROM_FC_TX_TECH_ADDR];

		if (TANnd bit ];

		if (TANnnx2_<481_EXPANS;
	_nx2_ACCESS000)f42s;

	rficG:ut)egacy
1);
	 ope t GPIE,link_/ Setpl & MDIO_PMA_REG_SFP_TWO_WIRE_CTRLANnd bit DIO_PA_DEVANnnx2_<481_EXPANS;
	_nx2_RD_RW,DIO_P&)egacy8 link_s;

	r;
	/* Low ==> corresLegacy
1);
	lXlink_/
	bbnx2x_c
;BC C)egacy8 link_s;
hI705\n");
	(()egacy8 link_iO _phy11amr== _phy11am;
hI7egacy8 );
	;
	()egacy8 link_iO _3hy9)ct rerom(7egacy8 );
	;

	(0hy9)cIFIER,d :
	{
< 1);
				}
				ne;
	pDENT rom(7egacy8 );
	;

	(1hy9)cIFIER,d :
	{
< 1);
				}
				nee;
	pDENT rom(7egacy8 );
	;

	(2hy9)cIFIER,d :
	{
< 1);
				}
				neee;
	pDENT ( MA_ShoulUTr, phappen: TA_Rtdast ;
	"Iow}P
	fw_tR,d :
	{
< 1);
				DR_Ava705\n");
	d			I_P
Eerom(705\n")le_eeperom(7egacy8 link_iO _phy8am<<<<	R,d :
duplepes DUPLEX_FULtachIpDENTIFIE	R,d :
duplepes DUPLEX_HALFall[S	cmd_param = FW_PARAM__SET(L;
	"y->up/_up%dMbp_,tis_duplep_full=E_ASFP+ mo_SETR,d :
	{
< 1);
	+ mo_SET(R,d :
duplepess DUPLEX_FULtmresp rac- = valpegacy
1);
	lAN x_folut
	}P
	fw_tuct bnIO_PMA_REG_SFP_TWO_WIRE
if (TANnd bit DIO_P
bnx2xANnnx2_<481_LEGACY8MIbn[tule_,DIO_PP	   timeotrom(anshy,A_RE5))IFIE	R,d :
l05\nXlink_/|=DIO_PPL(val[tule_	AUTO_NEGOTIATE_COMPLEx	;fw_tuct bnIO_PMA_REG_SFP_TWO_WIRE
if (TANnd bit DIO_P
bnx2xANnnx2_<481_LEGACY8ANnEXPANS;
	,DIO_PP	   timeotrom((anshy,A_RE0)w	;
			sp r	R,d :
l05\nXlink_/|=DIO_PPL(val[tule_	PARALLELc =TECG;
	_USEDesp _P_Rderom(705\n")le_eep;
	/* Low ==> corres_TYP48x3: p;
	",
				y->_ASFP+ mo_SETR,d :
	{
< 1);
	)al[SPCS stextval12x_folvpoNr_data += xfer_ead s;

 rac-R_ad LPlNdvertised",
			_/ Setpl & MDIO_PMA_REG_SFP_TWOj<<<<<ANnd bit DIO_PA_DEVANnnx2_CL37_FC_LP, p   timeout *anshy,A_RE5))IFIER,d :
l05\nXlink_/|=DIO_PL(val[tule_	L(valPARTNee_10TH		CAPABLEimeout *anshy,A_RE6))IFIER,d :
l05\nXlink_/|=DIO_PL(val[tule_	L(valPARTNee_10TF		CAPABLEimeout *anshy,A_RE7))IFIER,d :
l05\nXlink_/|=DIO_PL(val[tule_	L(valPARTNee_100TXH		CAPABLEimeout *anshy,A_RE8))IFIER,d :
l05\nXlink_/|=DIO_PL(val[tule_	L(valPARTNee_100TXF		CAPABLEimeout *anshy,A_RE9)cIFIER,d :
	{
\nXlink_/|=DIO_PL(val[tule_	L(valPARTNee_100T4	CAPABLEimetpl & MDIO_PMA_REG_SFP_TWOj<<<<<ANnd bit DIO_PA_DEVANnnx2_netdt [tule_, p   timmeout *anshy,A_RE10)cIFIER,d :
	{
\nXlink_/|=DIO_PL(val[tule_	L(valPARTNee_1000TH		CAPABLEimeout *anshy,A_RE11amIFIER,d :
	{
\nXlink_/|=DIO_PL(val[tule_	L(valPARTNee_1000TF		CAPABLEimetpl & MDIO_PMA_REG_SFP_TWOj<<<<<ANnd bit DIO_PA_DEVANnnx2_MA_mod [tule_, p   timmeout *anshy,A_RE11amIFIER,d :
	{
\nXlink_/|=DIO_PL(val[tule_	L(valPARTNee_10GXF		CAPABLEimetpfic(e
	}mto sut EEElwaodnEgotiat
	-curreol t			   is_s483x_s485x_smisf
 m				   eee_an2x_folvpedata += xfer_ead s;
	rrar *param705\n");ams *params,
					    485p_form<t(str(sG_Rxaw(str,sk_v*EPRt x_cl*lenstatusG_Rnumimetnum;
	((xaw(strhy,00F80du>> 7) <<r16a| _(xaw(strhy,007F) <<r8)t|n";
		  ((xaw(strhy,00Ftd0f >> 12);
E *paramPCS st3_seq_form<t(str(num, EPRt lensrams *params,
					    48xxdform<t(str(sG_Rxaw(str,sk_v*EPRt x_cl*lenstatusG_Reti us(strerad"ti us(str;
	((xaw(strhy,00F80du>> 7) <<r16a| _xaw(strhy,007F);
E *paramPCS stform<t(str("ti us(str, EPRt lensrams *params_up) {
	/* M4812;
	bnx2x_"Setting SFP+ transmitter t_param;
	char vendor_name[SFP_El & MDx2x_LASERcheck_limi	bn = tx_en_mode - POR1+ module isn = tx_en_mode - PORT_HW_CFLOW, 0_reSl & MDx2x_LASERcheck_limi	bn = tx_en_mode - POR1+ module isn = tx_en_mode - PORT_HW_CFLOW, 1srams *params_up) {
	/* M4812bnx2xbnx2x_"Setting SFP+ transmitter tt_param;
	char vendor_name[SFP_El & MDROM_FC_TX_Tcheck_limi	bnx2x


		if (TANnd bit 	bnx2xANnnx2_C_10000) 005_reSl & MDROM_FC_TX_Tcheck_limi	bnx2x


		if (Td 2nd bit  bnx2x_cl45_reD_1000psrams *params_up) {
	/* M48x3 bnx2xbnx2x_"Setting SFP+ transmitter to  _param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	char  8_CFGhar<4);eep16alms(bp->t"Setting 1xparam	sp tter ( BP_PATHpara;p
 * resttter ( TX laser of th
erom(e_eeprom;_c== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4823te((val & MDx2x_LASER_MDIs->port);
	bnx2x_cl43xHW_CFG_Tv sn = tx_en_mode - PORT_HW_CFLOW,HW_CFG_Tv sFRCMwer_R first((val & MDIO_PMA_REG_SFP_TWO_WIRE_CTRLCTLc = bnxDIO_PA_DEVa4833dTOP8727_XGS_EXSTRAP1,		   16l;trrR,
16a|==A_DEVa4833dSUPraDISOLATE			Il & MDcrocode, hence the ];

		if (TCTLc = bnxDIO_P A_DEVa4833dTOP8727_XGS_EXSTRAP1,	   16l;tr}<<<<*params_up) {
	/* M48xxdeetxl	chalbp = params->bp{
	int rc = 0;
	oo %*bp = params->bp;
	DP(NETIFsk_vOUTPstatuEPROM_VENDOR_NAME_SIZE+1];
	char<4);eephar  8_CFGhams(bp->t"Setting 1xparam	sp tter ( BP_PATHpara;p
 * resttter ( TX laser of th
e"p_modulOUTPs		break;
LEDDEADDLOFF:

	r;
	/* Low ==> corresPter bbnx:sLEDuEADD OFF2x_clFRCMwerr_erom(toX laser;
	lbp_OUTP	<<rSHAREDDx_8727_LEDDEADDLSHIFTw	;
 moduleSHAREDDx_8727_LEDD_XGS_E1s		bsp rac-S0G LED 8asksP
	fw_tuct bnIO_PMC_TX_TECH_ADDR];

		_CTRL_STATUS_MA_WIRED) =}8d 2n3_SP8481	LED1ng mo0_WIRED0) werr_etuct bnIO_PMC_TX_TECH_ADDR];

		_CTRL_STATUS_MA_WIRED) =}8d 2n3_SP8481	LED2ng mo0_WIRED0) werr_etuct bnIO_PMC_TX_TECH_ADDR];

		_CTRL_STATUS_MA_WIRED) =}8d 2n3_SP8481	LED3ng mo0_WIRED0) werr_etuct bnIO_PMC_TX_TECH_ADDR];

		_CTRL_STATUS_MA_WIRED) =}8d 2n3_SP8481	LED5ng mo0_WIRED0) werr_eR first((va	2x_LED 1 OFFP
	fw_tuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED1ng mo0_WIRED 0) werr_etrom(e_eeprom;_c=_WIRE bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858)retura	2x_LED 2 OFFP
	fw_ttuct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_LED2ng mo0_WIREDD 0) werura	2x_LED 3 OFFP
	fw_ttuct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_LED3ng mo0_WIREDD 0) werura}sp _P_euREG_SFPeak;
LEDDEADDLFRONGXSANELLOFF:

	r;
	/* Low ==> corresPter bbnx:sLEDuEADD FRONG SANEL OFF2x_c modulFRCMwerr_erom(toX laser;
	lbp_OUTP	<<rSHAREDDx_8727_LEDDEADDLSHIFTw	;
 moduleSHAREDDx_8727_LEDD_XGS_E1s		bsp rac-S0G LED 8asksP
	fw_tuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED1ng mo0_WIRED 0) werr_etuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED2ng mo0_WIRED 0) werr_etuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED3ng mo0_WIRED 0) werr_etuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED5ng mo0_WIRED 0)2 werr_eR first((va	uct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED1ng mo0_WIRED 0) wer_etrom(e_eeprom;_c=_WIREPRO bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4834	 etura	2x_(pmd_disMI4&= s,
	errupt_befman_s		copp LED4= 0;
	*/souort"ao ruO*pan  off.= 0;
	*	fw_ttrom(nfig[port].NDUAre
	g mo4&= ERRUPGXSbnx0T_HW_C	BC Ce_MDL_ENFRCM*4
_&INK,REPRONDUAg mo4MI4&= . etura	 TX laser
	chaflag_/|=DIO_PPL(valFLAGS4&= _DISABLEDerr_etreuct bn>lis_dis(_WIREDDr LED moO	NDUAre
	g mo4&= ERRUPGXSbnx0T_HW_C	B	e_MDL_ENFRCM*4LED moO	NDUAg mo4MI4&= .erura	}fw_ttuct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_SIGNALng mo0_WIREDD 0) werura}sp trom(e_eeprom;_c=_WIRE bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858)retura	2x_LED 2 OFFP
	fw_ttuct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_LED2ng mo0_WIREDD 0) werura	2x_LED 3 OFFP
	fw_ttuct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_LED3ng mo0_WIREDD 0) werura}sp _P_euREG_SFPeak;
LEDDEADDLON:

	r;
	/* Low ==> corresPter bbnx:sLEDuEADD ON2x_clFRCMwerr_erom(toX laser;
	lbp_OUTP	<<rSHAREDDx_8727_LEDDEADDLSHIFTw	;
 moduleSHAREDDx_8727_LEDD_XGS_E1s		b	 rac-S0G ruO IOse333EF_MSGtuct bnIO_PMA_REG_SFP_TWO_WIRE
if (T_STATUS_MA_WIRED) =}8d 2n3_SP8481	L(val[IGNAL	_WIRED	   timeot MDIO_P008eee;
	prR,
				bx2492nx
ea	uct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_L(val[IGNAL	_WIRED	hy *phRS rac-S0G LED 8asksP
	fw_tuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED1ng mo0_WIRED 0) werr_etuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED2ng mo0_WIRED 0)2 werr_etuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED3ng mo0_WIRED 0)2 werr_etuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED5ng mo0_WIRED 0) werurR first((va	uct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED1ng mo0_WIRED 0)2 wer_etrom(e_eeprom;_c=_WIREPRO bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4834	 etura	2x_(pmd_disMI4&= s,
	errupt_befman_s		copp LED4= 0;
	*/souort"ao ruO*pan  on.= 0;
	*	fw_ttrom(nfig[port].NDUAre
	g mo4&= ERRUPGXSbnx0T_HW_C	BC Ce_MDL_ENFRCM*4
_&INK,REPRONDUAg mo4MI4&= . etura	 TX laser
	chaflag_/|=DIO_PPL(valFLAGS4&= _DISABLEDerr_etreuct bn>lis_dis(_WIREDDr LED moO	NDUAre
	g mo4&= ERRUPGXSbnx0T_HW_C	B	e_MDL_ENFRCM*4LED moO	NDUAg mo4MI4&= .erura	}fw_t}sp trom(e_eeprom;_c=_WIREPRO bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858)retura	2x_Tell LED3 ao ruO*pan  onP
	fw_ttuct bnIO_PMA_REG_SFP_TWO_WIRE

if (T_STATUS_MA_WIREDD) =}8d 2n3_SP8481	L(val[IGNAL	_WIREDD	   timeotr MDIO_P~(7al6);
Ee[a<<<i			 2al6);  MA_A83B[8:6]		2 
	fw_ttuct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_L(val[IGNAL	_WIREDD    timeotruct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_LED3ng mo0_WIREDD 0)2 wer_etR first((va	tuct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_SIGNALng mo0_WIREDD 0)2 werura}sp _P_euREG_SFFPeak;
LEDDEADDLOPER:

	r;
	/* Low ==> corresPter bbnx:sLEDuEADD OPER2x_clFRCMwerr_erom(toX laser;
	lbp_OUTP	<<rSHAREDDx_8727_LEDDEADDLSHIFTw	;
 moduleSHAREDDx_8727_LEDD_XGS_E1s		bsp rac-S0G ruO IOse333EF_MSGtuct bnIO_PMA_REG_SFP_TWO_WIRE
if (T_STATUS_MA_WIRED) =}8d 2n3_SP8481	L(val[IGNAL	_WIRED	   timsp trom(!((anshy_WIREPRO Tv status *<<<<M481	L(val[IGNAL_LED4_AM, &v_M_SHw_WIREP>>v status *<<<<M481	L(val[IGNAL_LED4_AM, &v_SHIFTw)retura	{
				if (params->ph
		coppeL(val[IGNAL
		resp rtuct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_L(val[IGNAL	_WIREDD 0xa492werura}sRS rac-S0G LED 8asksP
	fw_tuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED1ng mo0_WIRED 0)1 werr_etuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED2ng mo0_WIRED 0)8 werr_etuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED3ng mo0_WIRED 0)98werr_etuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED5ng mo0_WIRED 0)4 werr_eR first((va	2x__XGS_E2 LED 8UTP	y, Set e tha
	
hdeM00M/1G/y[idLED_WIRE*/souorts
e GPalT wibndsmornx2x_LED1, raCOMPecaan s in_WIRE*/y[idinSoCOMP 8UTPs._WIRE*/
	prR,
	=m(toX laser;
	lbp_OUTP	<<_WIRESHAREDDx_8727_LEDDEADDLSHIFTw	;
 moREPRO TvSHAREDDx_8727_LEDD_XGS_E2) ? 0)98 : 0)8 err_etuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_LED1ng mo0_WIRED hy *phRS rac-Tell LED3 ao bp;
	"PIE,ouort"F_MSGtuct bnIO_PMA_REG_SFP_TWO_WIRE
if (T_STATUS_MA_WIRED) =}8d 2n3_SP8481	L(val[IGNAL	_WIRED	   timeot MDIO_P~(7al6);
Ee[e ti			 val6); MA_A83B[8:6]		1P
	fw_tuct bnIO_PMC_TX_TECH_ADDR];

			if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<M481_L(val[IGNAL	_WIRED	hy *ph	 trom(e_eeprom;_c=_WIREPRO bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858)retura	uct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_LED2ng mo0_WIREDD 0)18timeotruct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_LED3ng mo0_WIREDD 0)06werura}sp trom(e_eeprom;_c=_WIREPRO bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4834	 etura	2x_R__Oman_LED4/souort"ao externalOp;
	,= 0;
	*/NPUTrO-ower_u_,
	errupts.= 0;
	*	fw_ttuct bnIO_PMC_TX_TECH_ADDR];

				if (TB	rc = bnx2x_wa<<<<<<<<<<<<<<<<M481_SIGNALng mo0_WIREDD 0)4 werura	rom(eX laser
	chaflag_/&INK,REPROL(valFLAGS4&= _DISABLED. etura	 uct bn
	cha,
	_ower_u(truct werura	 eX laser
	chaflag_/&=DIO_PP	~L(valFLAGS4&= _DISABLEDer		a	}fw_t}sp _P_euREG_SFPRd cablTois ioda vor	aroundsgpo E3+84833s,ntil autonegd com33_OFRCMiodfix			yn f/wve	ontrol t"Setting 3paramreturl & MDIO_PMA_REG_SFP_TWOj<<<<<WCc = bnxDIO_PA_DEVW tx_e_GP2l[tule_	GP_2_1, p   time}<<<</******************************************************************/</*O_P54618SE PEV SECG;
	K,REP*/</******************************************************************/<*params_up) {
	/* 54618s< 1);cific_ phy_"Setting SFP+ transmitter tt_param;
	char vendor_name[tter tt venaL;
	}btatuEPROM_VENDOR_NAME_SIZE+1];
	char<4);.emchar"p_modulaL;
	}b		break;
PIO1:NIM:
	rfic NO_ENu * LED4: f:utao &= R (0x6).	ontrt_SFAcwessoppeshadow r33);
	}
oxe.-curreu8 tx_RO22ode, hence the ];

		if (Tx_e_GP_EXSHADOW,HW_C		if (Tx_e_GP_EXSHADOW_LEDDSEL2)al[SPCS stRO22oA_REG_SFP_TWO_WIRE_CTRLx_e_GP_EXSHADOW,HW_C	&.emc)al[S.emc O_P~(0xf	<<r4)al[S.emc 			 0x6	<<r4)al[Su8 tx_RO22ode, hence the ];

		if (Tx_e_GP_EXSHADOW,HW_C		if (Tx_e_GP_EXSHADOW_WR_AM, | .emc)al[Sfic NO_ENu * &= R 	PORd"PIEp;
	",link_/ch				.-curreu8 tx_RO22ode, hence the ];

		if (Tx_e_&= Rng mo0_WIREP~A_DEVx_e_&= Rng mo_L(val[tule_)al[SuREG_SFPRdms *params,
					   54618s<  case ot bn, u8 *o_buf)
{
	int rc = 0;
REPRO Tv_param;
	char vendor_name[tter ttEPRO Tv_param;
	chaNK, "read statuEPROM_VENDOR_NAME_SIZE+1];
	char  8_CFGhar<4);autoneg_<<<, an	neetGe t, an	ne	netGe t, fcGe t, .emchar vencfgfpin		
e;
	/* Low ==> corres54618SE cfgpt bn
		respT *phy,
				 struct linvalmsablTois vor	s vith E3 s inr_n,_nEG_SFP_cb valtaserhip
PMA_befman_de
	}mtoe staase CFG.ve	ontrtter ( TX laser of th
ecfgfpin	=m(nfig[port].sfp_ctrl)) &
		PORT_HW_CX_LASER_MASK;
	DP(NETIF_MSG_LINK,date media type for non-P*/
	fe3dcmn pin2cfg;
_&INK, bnx2x_8727__3XS_EXnxSEing modu>>INK, bnx2x_8727_E3XS_EXnxSEinSHIFT;tmsablDriveP_in	hi2x_ao bre staaseGPEV ut  of_bnx2x.byteruct bnf:uf	fgfpinort].	fgfpin,frwerarablw phyfARpGkEVT_ot33f:ut"KR m *phy(50l;tarabl33f:ut
	intteruct bnRO22ode, hence the ];

 bnx2x_cl45_reD_10000)8linval[ave mival;
x_f:uf		MDIO_Pence the rtruct werad(bp, phyfARpGkEVT_ot33f:ut"KR m *phy(50l;tal[ave mi54618s< 1);cific_ phy_data += xfer_PIO1:NIMwer_ficFlipp
hdu, gnalOde
	
	Dpolarl45_(f:ut0)1c.0)1e[8]).ntteruct bnRO22ode, hence the ];

if (Tx_e_GP_EXSHADOW,HW_Cif (Tx_e_GP_EXSHADOW_AUTO_DEingED.;
SPCS stRO22oA_REG_SFP_TWO_WIRif (Tx_e_GP_EXSHADOW,HW_C&.emc)al[.emc 			if (Tx_e_GP_EXSHADOW_INVEnx2FIB_SD;eruct bnRO22ode, hence the ];

if (Tx_e_GP_EXSHADOW,HW_Cif (Tx_e_GP_EXSHADOW_WR_AM, | .emc)al cablS0G fldfc/ SetMA_PlePORTref	}
FP_Ter_u_28B-3 of_802.3ab-1999uspec.ntteruct bnRalc_ieee_aneg_adv_data += xfer_&R,d :
ieee_fcrespfcGe t;
	d			rom((and :
ieee_fc &	if (TCOMBO_IEEE0	AUTO_NEGLADV_PAUSE_ASYMMETRICw	;
 moRif (TCOMBO_IEEE0	AUTO_NEGLADV_PAUSE_ASYMMETRICw mofcGe t;			if (TANnnx2_ADV_PAUSE_ASYMMETRIChaarol t	and :
ieee_fc &	if (TCOMBO_IEEE0	AUTO_NEGLADV_PAUSE_BOTHw	;
 moRif (TCOMBO_IEEE0	AUTO_NEGLADV_PAUSE_BOTHw mofcGe t;			if (TANnnx2_ADV_PAUSE_PAUSEal cablR_ad alT Ndvertiseme((ntteruct bnRO22oA_REG_SFP_TWO_WIR0)09,HW_C&an	neetGe t)eradx2x *bcr22oA_REG_SFP_TWO_WIR0)04,HW_C&an	ne	netGe ts;

ePCS stRO22oA_REG_SFP_TWO_WIRif (T_cl45_reD_100HW_C&autoneg_<<<werRS2x_(pmd_disgportd",
			ntterautoneg_<<<iO_P~( val6)a| __RE8)a| __RE9)a| __RE12)a| __RE13mrespan	ne	netGe tiO_P~( val5)a| __RE6)a| __RE7)a| __RE8)a| __RE10) |n"		   _phy11am;

Sol tyto OUTable to determin		}
				AUTO_NEGted\n" ;
			s OUTeterm_cax_8ask SFPEEPROO bnx2x_8727_}
				CAPABILITY_D0_1Gtte||n";
			s OUTrele to determin		}
				netd	mreturan	neetGe ti			 val8timeoautoneg_<<<i			 val9a| _RE12)imeout *o OUTableduplepess DUPLEX_FULtmn"		an	neetGe ti			 val9)imeo(phy->req_line_speedAdvertisoppeMG
		respd firsrrean	neetGe tiO_P~( val8)a| __RE9)s;

ePCS stRO22ode, hence the ];

0)09,HW_Can	neetGe t)erruct bnRO22oA_REG_SFP_TWO_WIR0)09,HW_C&an	neetGe t)eradMA_Advertis iy[/nee p;
	",
				yterol to OUTable to determin		}
				AUTO_NEGte{meout *o OUTeterm_cax_8ask SFPREPRO bnx2x_8727_}
				CAPABILITY_D0_10M_HALF. eturaan	ne	netGe ti			 val5weruraautoneg_<<<i			 val9a| _RE12)imeoo(phy->req_line_speedAdvertisoppeM0M-HD
		resp }meout *o OUTeterm_cax_8ask SFPREPRO bnx2x_8727_}
				CAPABILITY_D0_10M_FULtm eturaan	ne	netGe ti			 val6weruraautoneg_<<<i			 val9a| _RE12)imeoo(phy->req_line_speedAdvertisoppeM0M-FD
		resp }meout *o OUTeterm_cax_8ask SFPREPRO bnx2x_8727_}
				CAPABILITY_D0_100M_HALF. eturaan	ne	netGe ti			 val7weruraautoneg_<<<i			 val9a| _RE12)imeoo(phy->req_line_speedAdvertisoppeM00M-HD
		resp }meout *o OUTeterm_cax_8ask SFPREPRO bnx2x_8727_}
				CAPABILITY_D0_100M_FULtm eturaan	ne	netGe ti			 val8weruraautoneg_<<<i			 val9a| _RE12)imeoo(phy->req_line_speedAdvertisoppeM00M-FD
		resp }meRd cablO inpy[/nee e GPalTowG_SFP_vor		yn FORCE_OUTP	ontrol to OUTable to determin		}
				10		returautoneg_<<<i			 val13mal[Sfic nion
d AUTO-if Xlrn n;autonegMiodcation
	-curreu8 tx_RO22ode, hence the ];

	0)18 ];

	 val15a| _RE9a| 7RE0)wimeo(phy->req_line_speed
		coppeM00Msgport
		resp}trol to OUTable to determin		}
				10f ((vaMA_ nion
d AUTO-if Xlrn n;autonegMiodcation
	-curreu8 tx_RO22ode, hence the ];

	0)18 ];

	 val15a| _RE9a| 7RE0)wimeo(phy->req_line_speed
		coppeM0Msgport
		resp}t
erom(to OUTflag_/& FLAGS4 EEted\	PCS steee_has_cax-EINVAL;
	{sp ,
		rcasrreu8 tx_RO22ode, hence the 	if (Tx_e_GP_EXEXP_ACCESS0HW_C		if (Tx_e_GP_EXEXP_ACCESSdTOP |n"				if (Tx_e_GP_EXEXP_TOP82K_BUF)al[SPCS stRO22oA_REG_SFP_TWO	if (Tx_e_GP_EXEXP_ACCESSdGATEr_&.emc)al[S.emc O_P0xfffeal[Su8 tx_RO22ode, hence the 	if (Tx_e_GP_EXEXP_ACCESSdGATEr_.emc)al c _c;
				   eee_, bnx2xfor non(+= xfer_ead , SHMEMMEEEDS_LADVs;
	rol trc  ((vaa;
	/* Low ==> corresF_FP		tao ruO_ENu *  EEltimtrs
		resp ruct bneee_cation
,data += xfer_ead s;
_eR firstrom(toX lasereee_OUTP	& EEEDEADDLADV_LPIm d\n")	
			s OUTreleduplepess DUPLEX_FULtm d\n")	
	m(			   eee_calc_timtr-EINVAL;e||n"		
	m !(eMF synceee_OUTP	& EEEDEADDL_M, &v_LPIm)tt((va	2x_NEG_SFP_Ndvertise  EEls inlrn n;33W est
	+ mo_S*/NPUTeiCOMP no LPI assertionlwaod33W est
	+ mo_S*/ARpphywaod33W est
	lNPUTa	hy idltimtrywaodx2x. mo_S*/Alsofr, icisgulT duplepeisu33W ibndsgpo EEE._WIRE*/
	pr			   eee_advertise,data += xfer_ead ,ter ttEPROSHMEMMEEEDS_LADVs;
	rR first((va	;
	/* Low ==> corresDon't_Advertis iyiBPOR-T EEEfw_resp ruct bneee_cation
,data += xfer_ead s;
_eRP_R first((va<<rynceee_Xlink_/&_P~SHMEMMEEEDS_LADV	<<_WIREEPROSHMEMMEEEDSUP bnx			SHIFT;tmsrol to OUTflag_/& FLAGS4 EEte((va	2x_HNPUdispegacy
auto-grEEEnt*/
	prrom(eX laserORT_FEA  case oflag_/&INK,EPROFEATURE_CONFIG	AUTOGREEENL_M, &vD. etura	.emc ( 6er		a	;
	/* Low ==> corres nionoppeAuto-GrEEEnfw_resp rR first((va	t.emc ( 0er		a	;
	/* Low ==> corresDon't_Adv. EEEfw_resp r}sp ruct bncrocode, hence the rt45_rANnd bit ];

			if (TANnnx2_EEEDADV, .emc)al[S_P_Rd cPCS stat22ode, hence the ];

0)04,HW_Can	ne	netGe ti	 fcGe tm;

Sol to OUTableduplepess DUPLEX_FULtmn"	autoneg_<<<i			 val8s;

ePCS stRO22ode, hence the ];

bnx2x_cl45_reD_1000autoneg_<<<werRS *param0rams <*params_up) {
	/* 5461xdeetxl	chalbp = params->bp{
	int rc = 0;
	oo % %*bp = params->bp;
	DP(NETIFsk_vOUTPstatuEPROM_VENDOR_NAME_SIZE+1];
	char<4);.emcha
ePCS stRO22ode, hence the ];
if (Tx_e_GP_EXSHADOW,HW_if (Tx_e_GP_EXSHADOW_LEDDSEL1val[uct bnRO22oA_REG_SFP_TWO_WIif (Tx_e_GP_EXSHADOW,HW_&.emc)al[.emc O_P0xff0d		
e;
	/* Low ==> corres54618x f:utp;
	"P		tlOUTP=%x)2x_clOUTPs;
e"p_modulOUTPs		break;
LEDDEADDLFRONGXSANELLOFF:
reak;
LEDDEADDLOFF:
[S.emc 			0) 0eeal[SuREG_SFPeak;
LEDDEADDLOPER:
[S.emc 			0) 001al[SuREG_SFPeak;
LEDDEADDLON:
[S.emc 			0) 0ffal[SuREG_SFP>ereadd:l[SuREG_SFPRdePCS stRO22ode, hence the ];
if (Tx_e_GP_EXSHADOW,HW_if (Tx_e_GP_EXSHADOW_WR_AM, | .emc)al[ *pararams <*params_up) {
	/* 54618s< bnx2xbnx2x_"Setting SFP+ transmitter to    _param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	char vencfgfpin		r  8_CFGhams2x_In eak;
of no E, a routG_SFP_33f:utaaseGPEVe tutpph
PMA_yn low powGP 8UTP.ve	ontru8 tx_RO22ode, hence the 	if (T_cl45_reD_10000)8liwer_ficTois vor	s vith E3 s inr_n,_nEG_SFP_cb valtaserhip
PMA_befman_de
	}mtoe staase CFG.ve	ontrtter ( TX laser of thecfgfpin	=m(nfig[port].sfp_ctrl)) &
		PORT_HW_CX_LASER_MASK;
	DP(NETIF_MSG_LINK,date media type for non-P*/
	fe3dcmn pin2cfg;
_&INK, bnx2x_8727__3XS_EXnxSEing modu>>INK, bnx2x_8727_E3XS_EXnxSEinSHIFT;tmsablDriveP_in	low FP_tutpGkEVTin_bnx2x.byteruct bnf:uf	fgfpinort].	fgfpin,f0)rams *paramsk_vars *v54618s< GXS_EXlink_up = 0;
	u16 val1, rx_sd;
	sto %*bp = params->bp;
	DP(NETIF_MSG_LINKstruct bnx2xNK, "read statuEPROM_VENDOR_NAME_SIZE+1];
	char<4);eep;ar  8705\n");
	bale<4);)egacy8 link_,t egacy8 );
	harrficG:ut1);
	 ope t GPIE,link_/ SetPCS stRO22oA_REG_SFP_TWO_WIRif (Tx_e_GP_EXAUXf[tule_,DIO_&)egacy8 link_s;
e;
	/* Low ==> corres54618SE GXS_EXlink_: 	bnx2x_cl)egacy8 link_s;

	ablR_ad ,link_/FP_cleartaasekEVTin	errupt.ntteruct bnRO22oA_REG_SFP_TWO_WIRif (Tx_e_&= Rn[tule_,DIO_&<<<werRS705\n");
	(()egacy8 link_iO _phy2amr== _phy2am;

Sol t705\n")le_eep7egacy8 );
	;
	()egacy8 link_iO _7RE8));
	rol t7egacy8 );
	;

	(7RE8))t((va	R,d :
	{
< 1);
				}
				neee;
	p	R,d :
duplepes DUPLEX_FULtachIR firstrom(7egacy8 );
	;

	(6RE8))t((va	R,d :
	{
< 1);
				}
				neee;
	p	R,d :
duplepes DUPLEX_HALFalhIR firstrom(7egacy8 );
	;

	(5RE8))t((va	R,d :
	{
< 1);
				}
				nee;
	p	R,d :
duplepes DUPLEX_FULtachIR(vaMA_Omi	coppeM00BPOR-T4dfARpnowt*/
	pfirstrom(7egacy8 );
	;

	(3RE8))t((va	R,d :
	{
< 1);
				}
				nee;
	p	R,d :
duplepes DUPLEX_HALFalhIR firstrom(7egacy8 );
	;

	(2RE8))t((va	R,d :
	{
< 1);
				}
				ne;
	p	R,d :
duplepes DUPLEX_FULtachIR firstrom(7egacy8 );
	;

	(1RE8))t((va	R,d :
	{
< 1);
				}
				ne;
	p	R,d :
duplepes DUPLEX_HALFalhIR firstMA_ShoulUTr, phappenP
	fw_tR,d :
	{
< 1);
				DR_meo(phy->req_line_speFPREPR(L;
	"y->up/_up%dMbp_,tis_duplep_full=E_ASFP+ moSETR,d :
	{
< 1);
	+ moSET(R,d :
duplepess DUPLEX_FULtmres
 rac- = valpegacy
1);
	lAN x_folut
	}P
	fw_PCS stRO22oA_REG_SFP_TWO_WIRE0) 1,
 moOp   timeout *anshy,A_RE5))IFIER,d :
l05\nXlink_/|=DIO_PL(val[tule_	AUTO_NEGOTIATE_COMPLEx	;fw_PCS stRO22oA_REG_SFP_TWO_WIRE0) 6,
 moOp   timeout *(anshy,A_RE0)w	;
			sp rR,d :
l05\nXlink_/|=DIO_PL(val[tule_	PARALLELc =TECG;
	_USEDesmeo(phy->req_line_speedBCM54618SE: p;
	",
				y->_ASFP+ mo_SETR,d :
	{
< 1);
	)alfw_PCS stextval12x_folvpoNr_data += xfer_ead s;

 rut *and :
l05\nXlink_/& L(val[tule_	AUTO_NEGOTIATE_COMPLEx	te((va	2x_Retter LPlNdvertised",
			_/ SetpSPCS stRO22oA_REG_SFP_TWO	0x5, p   timmeotrom(anshy,A_RE5))IFIE	R,d :
l05\nXlink_/|=DIO_P  L(val[tule_	L(valPARTNee_10TH		CAPABLEimeoout *anshy,A_RE6))IFIE	R,d :
l05\nXlink_/|=DIO_P  L(val[tule_	L(valPARTNee_10TF		CAPABLEimeoout *anshy,A_RE7))IFIE	R,d :
l05\nXlink_/|=DIO_P  L(val[tule_	L(valPARTNee_100TXH		CAPABLEimeoout *anshy,A_RE8))IFIE	R,d :
l05\nXlink_/|=DIO_P  L(val[tule_	L(valPARTNee_100TXF		CAPABLEimeoout *anshy,A_RE9))IFIE	R,d :
l05\nXlink_/|=DIO_P  L(val[tule_	L(valPARTNee_100T4	CAPABLEimetpSPCS stRO22oA_REG_SFP_TWO	0xa, p   timeoout *anshy,A_RE10)cIFIE	R,d :
l05\nXlink_/|=DIO_P  L(val[tule_	L(valPARTNee_1000TH		CAPABLEimeoout *anshy,A_RE11amIFIE	R,d :
l05\nXlink_/|=DIO_P  L(val[tule_	L(valPARTNee_1000TF		CAPABLEimetperom(to OUTflag_/& FLAGS4 EEted\INK,EPROPCS steee_has_cax-EINVAL;
DIO_P			   eee_an2x_folvpedata += xfer_ead s;
	 _P_Rde *param705\n");ams *params_up) {
	/* 54618s<  case oloopback, u8 *o_buf)
{
	int rc = 0;
REP_param;
	char vendor_name[SFP_EEPROM_VENDOR_NAME_SIZE+1];
	char _clor as  venumac		PORT( TX laser of  ? GRCBASE_UMAC1 : GRCBASE_UMACd		
e;
	/* Low ==> corres2_cl/PMDpextval12loopback: 54618s<fw_resarMA_ nion
 ma;
	}/slav
 manunshmOUTP	vnd f:utao ma;
	}/ SetMA_mii de, h 9 [>lis f:ut11 12]	ontru8 tx_RO22ode, hence the 	0)09, 3RE11aesarMA_gportd"1GlNPUTdpmd_disautonegM SetMA_f:utansh[mii A_RE 0]M SetMA_f:utansh[expr $anshy,[>lis cleart6 12 13]]M SetMA_f:utansh[expr $ansh| [>lis f:ut6 8]]M SetMA_mii de, h 0 $anshtteruct bnRO22oA_REG_SFP_TWO	0) 0			   time MDIO_P~(__RE6)a| __RE12)a| __RE13mrespe ti			 val6wa| __RE8);eruct bnRO22ode, hence the 	0) 0		e t)eradMA_S0G externalOpoopbacklNPUTTx usoppe6dB code st SetMA_mii de, h 0x18 7M SetMA_f:utansh[mii A_RE 0x18]M SetMA_mii de, h 0x18 [expr $ansh| [>lis f:ut10 15]]	ontru8 tx_RO22ode, hence the 	0)18, 7val[uct bnRO22oA_REG_SFP_TWO	0)18, 	   timeu8 tx_RO22ode, hence the 	0)18, ansh| __RE10) | __RE15)valmsablTois r33);
	}
opena modugt e fARpmoduUMAC desp, h lis namP	ontrx_e_WRort].NDUAre
	EGRESSdEMACdXSbnx +Ce_MDL_ENFRCM*4,frwerarablMaximum FramP	Length-(RW). Defto oda 14-Bit maximum framP
PMA_bength-used"bypmoduMAC receivePlogamsFP_cb valframPs.ve	ontrx_e_WRort].umac		PORT+uUMACAre
	g XFR 	0)2710)rams /******************************************************************/</*O_PSFX7101 PEV SECG;
	K,REP*/</******************************************************************/<*params_up) {
	/* 7101  case oloopback, u8 *o_buf)
{
	int rc = 0;
	oo % %*bp = params->bp;
	DP(NETISFP_EEPROM_VENDOR_NAME_SIZE+1];
	charMA_SFX7101_W_CFGTEST1byteruct bnROM_Fde, hence the ];

 bnx2xCFGd bit  bnx2xCFGSFX7101_W_CFGTEST1 	0)100)rams *params,
					   7101  case ot bn, u8 *o_buf)
{
	int rc = 0;
	o*bp = params->bp;
	DP(NETIF_MSG_LIstruct bnx2xNK, "read statu _clfw(str1 	fw(str2,lor as EPROM_VENDOR_NAME_SIZE+1];
	char(phy->req_line_speed
		coppemoduSFX7101 LASI	y, Set 
	}fw_resarMA_R__Oman_norm<< powGP 8UTPyteruct bnf:ufLASER_MDIs->port);
	bnx2x_cl42+ moSETTv sn = tx_en_mode - PORT_HW_CFG_TX,.sfp_ctrlFRCMwer_ficHWt33f:ut"KR PCS stextval12;
	bnx2x_rt].sfp_ctrlFRCMwer_ave mival;
x_f:uf		MDIO_Pence the rtruct weraduct bnROM_Fde, hence the ];

 bnx2xd 2nd bit  bnx2x_cl4LASIeD_10000)1s;
e;
	/* Low ==> corres
		coppemoduSFX7101 LED ao bp;
	"PIEtrafficfw_respuct bnROM_Fde, hence the ];

 bnx2xd 2nd bit  bnx2x_cl4re
	7107_LEDDCNT000__RE3)s;

ePCS stextval12f:ufpause  detectedhe read s;
	MA_R__OFRCMautonegM Setuct bnIO_PMA_REG_SFP_TWO_WIRt45_rANnd bit 	bnx2xANnnx2_C_1000	   time MDI			bx2ee;
	uct bnROM_Fde, hence the ];

 bnx2xANnd bit 	bnx2xANnnx2_C_1000e t)eradMA_Sav
 "ti us strs
	}M Setuct bnIO_PMA_REG_SFP_TWO_WIRt45_rd 2nd bit  bnx2x_cl4re
	7101_VER1, pfw(str1weraduct bnROM_FA_REG_SFP_TWO_WIRt45_rd 2nd bit  bnx2x_cl4re
	7101_VER2, pfw(str2respuct bn avpoeti us(strs
	},rt].sfp_ctrlFRCMF_MSG_LI(sG_)(fw(str1RE16a| fw(str2rFP_TW->str_addr)al[ *param0rams *paramsk_vars *v7101_GXS_EXlink_up = 0;
	u16 val1, rx_sd;
	st*bp = params->bp;
	DP(NETIF_MSG_L_param;
	chaNK, "read statuEPROM_VENDOR_NAME_SIZE+1];
	char  8705\n");ar<4);eep1,    2;aruct bnROM_FA_REG_SFP_TWO_WIRt45_rd 2nd bit  bnx2x_cl4LASIe[tul00	   2);aruct bnROM_FA_REG_SFP_TWO_WIRt45_rd 2nd bit  bnx2x_cl4LASIe[tul00	   1s;
e;
	/* Low ==> corresSG_-bPOR-T LASI	Xlink_/	bnx->bbnx2x_c
;BC C   2,	   1);aruct bnROM_FA_REG_SFP_TWO_WIRt45_rd 2nd bit  bnx2x_cl4re
	[tule_, p   2);aruct bnROM_FA_REG_SFP_TWO_WIRt45_rd 2nd bit  bnx2x_cl4re
	[tule_, p   1s;
e;
	/* Low ==> corresSG_-bPOR-T _cl	Xlink_/	bnx->bbnx2x_c
;BC C   2,	   1);ar705\n");
	((   1hy,4w	;
	4s;
	MA_If p;
	"y->up/pr,
		moduAN ut 		M;
of moduSFX7101 PEV ontrol t705\n")le_eepuct bnIO_PMA_REG_SFP_TWO_WIREbnx2xANnd bit 	bnx2xANnnx2_MA_mod [tule_,
 moOp   2)imeo Md :
	{
< 1);
				}
				netd0			IR,d :
duplepes DUPLEX_FULtachI;
	/* Low ==> corres
FX7101 AN Xlink_/	bnx->Ma;
	}=nx2x_c
;BBC C   2,	tR,l2iO _phy14)))al[SPCS stextval12SG_Lan2x_folvpence the read s;
	SPCS stextval12x_folvpoNr_data += xfer_ead s;

 rac-R_ad LPlNdvertised",
			_/ Setpol tR,l2iO _phy11amsp rR,d :
l05\nXlink_/|=DIO_PL(val[tule_	L(valPARTNee_10GXF		CAPABLEim_Rde *param705\n");ams *params,
					   7101 form<t(str(sG_R"ti us(str, k_v*EPRt x_cl*lenstatuol t*len < 5)sp _*param-EINVALas EPR[0]E_S("ti us(strhy,00FF)al[EPR[1]E_S("ti us(strhy,00FFd0f >> 8al[EPR[2]E_S("ti us(strhy,00FFd0d0f >> 16al[EPR[3]E_S("ti us(strhy,00FFd0d0d0f >> 24al[EPR[4]E_S'\0'al[*len -= 5al[ *param0rams _up) {
	/* sfx7101 sp_s
	bnx2x_"Setting SFP_NAM, p = 0;
	u16 val1, rx_statu _cle t, cnteraduct bnROM_FA_REG_SFP_TWO_WIRt45_rd 2nd bit _WIRt45_rd 2nre
	7101_nxSEi, p   timmefARp(cnt			DR cnt < 1DR cnt++le_eepm *phy(50l;t rac-We, hoda self-clearoppe33f:ut"KR Il & MDcrocode, hence the ];

		if (TB	rc = bnx2x_wa bnx2x_cl4re
	7101_nxSEi,2x_wa (ansh| __RE15)))al[S(bp, phyfARpcleart Setpl & MDIO_PMA_REG_SFP_TWO_WIRE_CTRLB	rc = bnx2x_wat45_rd 2nre
	7101_nxSEi, p   timmeout *(anshy,A_RE15))	;
			sp ruREG_SFPRdms *params_up) {
	/* 7101 ;
	bnx2x_"Setting SFP+ transmitter t_param;
	char vendor_name[Se_ee2x_Low powGP 8UTP"y->ruO IOsled"byp- PO	2 
	fwuct bnf:ufLASERcheck_limi	bn = tx_en_mode - POR2+ moSETTv sn = tx_en_mode - PORT_HW_CFLOW, sfp_ctrlFRCMwer_ficTasekEVT33f:uty->ruO IOsled"byp- PO	1 
	fwuct bnf:ufLASERcheck_limi	bn = tx_en_mode - POR1+ module isn = tx_en_mode - PORT_HW_CFLOW, sfp_ctrlFRCMwerms *params_up) {
	/* 7101 eetxl	chalbp = params->bp{
	int rc = 0;
	oo *bp = params->bp;
	DP(NETIFsk_vOUTPstatu _cle t;
	baleEPROM_VENDOR_NAME_SIZE+1];
	char"p_modulOUTPs		break;
LEDDEADDLFRONGXSANELLOFF:
reak;
LEDDEADDLOFF:
[Se t;
	2al[SuREG_SFPeak;
LEDDEADDLON:
[Se t;
	1al[SuREG_SFPeak;
LEDDEADDLOPER:
[Se t;
	baleSuREG_SFPRdePCS stROM_Fde, hence the ];

 bnx2xd 2nd bit 
_wa bnx2x_cl4re
	7107	L(valLEDDCNT00
_wa    timms /******************************************************************/</*O_PStulICekEVTDECLARAG;
	K,REP*/</******************************************************************/< *paramsruO*p p = 0;
	u16 val1,al12nulT =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHYNOT_CON	,DI.addr		==0,DI.def_md_devad	==0,DI.flag_		==FLAGS4&=IT_W_CFGFIRSi,2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==0,DI.media_rom;	= ETHXS_EXNOT_PnxSENi,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)NULt,DI.abS_EXlink_	==(abS_EXlink__t)NULt,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)NULt,DI. case oloopback ==( case oloopback_t)NULt,DI.form<t(fw(str	==(form<t(fw(str_t)NULt,DI.;
	bnx2x	==(;
	bnx2x_t)NULt,DI.eetxl	chalbp	==(eetxl	chalbp_t)NULt,DI.al12f);cific_ phy ==(al12f);cific_ phy_t)NULt
};< *paramsruO*p p = 0;
	u16 val1,al12serdes =		br.rom;		== bnx2x_8727_SERDEFG_XGXS_EXT_PHYDIRECT,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==0,2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10bPORx2xalf |n"		   SUP bnx			10bPORx2FulT |n"		   SUP bnx			100bPORx2xalf |n"		   SUP bnx			100bPORx2FulT |n"		   SUP bnx			1000bPORx2FulT |n"		   SUP bnx			2500bPORX2FulT |n"		   SUP bnx			TP |n"		   SUP bnx			AutonegM|n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXBASE_i,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)	u16 vxgxs  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bn
	chas		copps8 link_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bn,
	_bnx2xbnx2x,DI. case oloopback ==( case oloopback_t)NULt,DI.form<t(fw(str	==(form<t(fw(str_t)NULt,DI.;
	bnx2x	==(;
	bnx2x_t)NULt,DI.eetxl	chalbp	==(eetxl	chalbp_t)NULt,DI.al12f);cific_ phy ==(al12f);cific_ phy_t)NULt
};< *paramsruO*p p = 0;
	u16 val1,al12xgxs =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHYDIRECT,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==0,2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10bPORx2xalf |n"		   SUP bnx			10bPORx2FulT |n"		   SUP bnx			100bPORx2xalf |n"		   SUP bnx			100bPORx2FulT |n"		   SUP bnx			1000bPORx2FulT |n"		   SUP bnx			2500bPORX2FulT |n"		   SUP bnx			netd0bPORx2FulT |n"		   SUP bnx			FIBRE |n"		   SUP bnx			AutonegM|n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXCX4,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)	u16 vxgxs  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bn
	chas		copps8 link_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bn,
	_bnx2xbnx2x,DI. case oloopback ==( case oloopback_t)uct bnf:ufxgxs loopback,DI.form<t(fw(str	==(form<t(fw(str_t)NULt,DI.;
	bnx2x	==(;
	bnx2x_t)NULt,DI.eetxl	chalbp	==(eetxl	chalbp_t)NULt,DI.al12f);cific_ phy ==(al12f);cific_ phy_t)	u16 vxgxs f);cific_ phy
};<*paramsruO*p p = 0;
	u16 val1,al12warpcman_=		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHYDIRECT,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==FLAGS4TX_ERROR_CHECK,2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10bPORx2xalf |n"		   SUP bnx			10bPORx2FulT |n"		   SUP bnx			100bPORx2xalf |n"		   SUP bnx			100bPORx2FulT |n"		   SUP bnx			1000bPORx2FulT |n"		   SUP bnx			1000bPORKX2FulT |n"		   SUP bnx			netd0bPORx2FulT |n"		   SUP bnx			netd0bPORKR2FulT |n"		   SUP bnx			2etd0bPORKR22FulT |n"		   SUP bnx			2etd0bPORMLD22FulT |n"		   SUP bnx			FIBRE |n"		   SUP bnx			AutonegM|n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXUNSPECIFIED,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DIabl33leduplepesP*/0,DIabl3srvesP*/0,DI. case ot bn	==( case ot bn_t)	u16 vwarpcman  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bnwarpcman abS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bnwarpcman bnx2xbnx2x,DI. case oloopback ==( case oloopback_t)uct bnf:ufwarpcman boopback,DI.form<t(fw(str	==(form<t(fw(str_t)NULt,DI.;
	bnx2x	==(;
	bnx2x_t)uct bnwarpcman ;
	bnx2x,DI.eetxl	chalbp	==(eetxl	chalbp_t)NULt,DI.al12f);cific_ phy ==(al12f);cific_ phy_t)NULt
};< <*paramsruO*p p = 0;
	u16 val1,al127101 =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY
FX7101,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==FLAGS4FAN4FAILURE_DEinREQ,2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10td0bPORx2FulT |n"		   SUP bnx			TP |n"		   SUP bnx			AutonegM|n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXBASE_i,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)			   7101  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bn7101_GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bncommontextvbnx2xbnx2x,DI. case oloopback ==( case oloopback_t)uct bn7101  case oloopback,DI.form<t(fw(str	==(form<t(fw(str_t)			   7101 form<t(str,DI.;
	bnx2x	==(;
	bnx2x_t)uct bn7101 ;
	bnx2x,DI.eetxl	chalbp	==(eetxl	chalbp_t){
	/* 7101 eetxl	chalbp,DI.al12f);cific_ phy ==(al12f);cific_ phy_t)NULt
};<*paramsruO*p p = 0;
	u16 val1,al128073 =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP073,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==0,2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10000bPORx2FulT |n"		   SUP bnx			2500bPORX2FulT |n"		   SUP bnx			netdbPORx2FulT |n"		   SUP bnx			FIBRE |n"		   SUP bnx			AutonegM|n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXKR,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)			   P073  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bnP073 GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bnP073 bnx2xbnx2x,DI. case oloopback ==( case oloopback_t)NULt,DI.form<t(fw(str	==(form<t(fw(str_t)uct bnform<t(str,DI.;
	bnx2x	==(;
	bnx2x_t)NULt,DI.eetxl	chalbp	==(eetxl	chalbp_t)NULt,DI.al12f);cific_ phy ==(al12f);cific_ phy_t)	u16 vP073 f);cific_ phy
};<*paramsruO*p p = 0;
	u16 val1,al128705 =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP705,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==FLAGS4&=IT_W_CFGFIRSi,2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10000bPORx2FulT |n"		   SUP bnx			FIBRE |n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXXFP	FIBER,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)			   P705  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bnP705 GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bncommontextvbnx2xbnx2x,DI. case oloopback ==( case oloopback_t)NULt,DI.form<t(fw(str	==(form<t(fw(str_t)uct bnnulTnform<t(str,DI.;
	bnx2x	==(;
	bnx2x_t)NULt,DI.eetxl	chalbp	==(eetxl	chalbp_t)NULt,DI.al12f);cific_ phy ==(al12f);cific_ phy_t)NULt
};<*paramsruO*p p = 0;
	u16 val1,al128706 =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP706,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==FLAGS4&=IT_W_CFGFIRSi,2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10000bPORx2FulT |n"		   SUP bnx			netdbPORx2FulT |n"		   SUP bnx			FIBRE |n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXSFPP2SG_LFIBER,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)			   P706  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bnP706 GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bncommontextvbnx2xbnx2x,DI. case oloopback ==( case oloopback_t)NULt,DI.form<t(fw(str	==(form<t(fw(str_t)uct bnform<t(str,DI.;
	bnx2x	==(;
	bnx2x_t)NULt,DI.eetxl	chalbp	==(eetxl	chalbp_t)NULt,DI.al12f);cific_ phy ==(al12f);cific_ phy_t)NULt
};<<*paramsruO*p p = 0;
	u16 val1,al128726 =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP726,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==(FLAGS4&=IT_W_CFGFIRSi |n"		   FLAGS4TX_ERROR_CHECK),2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10000bPORx2FulT |n"		   SUP bnx			netdbPORx2FulT |n"		   SUP bnx			AutonegM|n"		   SUP bnx			FIBRE |n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXNOT_PnxSENi,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)uct bnP726  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bnP726 GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bnP726 bnx2xbnx2x,DI. case oloopback ==( case oloopback_t)uct bnP726  case oloopback,DI.form<t(fw(str	==(form<t(fw(str_t)			   form<t(str,DI.;
	bnx2x	==(;
	bnx2x_t)NULt,DI.eetxl	chalbp	==(eetxl	chalbp_t)NULt,DI.al12f);cific_ phy ==(al12f);cific_ phy_t)NULt
};<<*paramsruO*p p = 0;
	u16 val1,al128727 =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP727,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==(FLAGS4FAN4FAILURE_DEinREQ |n"		   FLAGS4TX_ERROR_CHECK),2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10000bPORx2FulT |n"		   SUP bnx			netdbPORx2FulT |n"		   SUP bnx			FIBRE |n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXNOT_PnxSENi,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)uct bnP727  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bnP727 GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bnP727vbnx2xbnx2x,DI. case oloopback ==( case oloopback_t)NULt,DI.form<t(fw(str	==(form<t(fw(str_t)uct bnform<t(str,DI.;
	bnx2x	==(;
	bnx2x_t)uct bnP727v;
	bnx2x,DI.eetxl	chalbp	==(eetxl	chalbp_t){
	/* P727veetxl	chalbp,DI.al12f);cific_ phy ==(al12f);cific_ phy_t){
	/* P727ve);cific_ phy
};<*paramsruO*p p = 0;
	u16 val1,al128481 =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP481,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==FLAGS4FAN4FAILURE_DEinREQ |n"		  FLAGS4REARM_LATCHl[IGNAL	_W.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10bPORx2xalf |n"		   SUP bnx			10bPORx2FulT |n"		   SUP bnx			100bPORx2xalf |n"		   SUP bnx			100bPORx2FulT |n"		   SUP bnx			1000bPORx2FulT |n"		   SUP bnx			10000bPORx2FulT |n"		   SUP bnx			TP |n"		   SUP bnx			AutonegM|n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXBASE_i,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)uct bnP481_ case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bnP48xx GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bnP481_bnx2xbnx2x,DI. case oloopback ==( case oloopback_t)NULt,DI.form<t(fw(str	==(form<t(fw(str_t)uct bnP48xx form<t(str,DI.;
	bnx2x	==(;
	bnx2x_t)uct bnP481_;
	bnx2x,DI.eetxl	chalbp	==(eetxl	chalbp_t){
	/* P48xx eetxl	chalbp,DI.al12f);cific_ phy ==(al12f);cific_ phy_t)NULt
};<<*paramsruO*p p = 0;
	u16 val1,al1284823 =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4823,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==(FLAGS4FAN4FAILURE_DEinREQ |n"		   FLAGS4REARM_LATCHl[IGNAL |n"		   FLAGS4TX_ERROR_CHECK),2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10bPORx2xalf |n"		   SUP bnx			10bPORx2FulT |n"		   SUP bnx			100bPORx2xalf |n"		   SUP bnx			100bPORx2FulT |n"		   SUP bnx			1000bPORx2FulT |n"		   SUP bnx			10000bPORx2FulT |n"		   SUP bnx			TP |n"		   SUP bnx			AutonegM|n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXBASE_i,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)uct bnP48x3  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bnP48xx GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bnP48x3 bnx2xbnx2x,DI. case oloopback ==( case oloopback_t)NULt,DI.form<t(fw(str	==(form<t(fw(str_t)uct bnP48xx form<t(str,DI.;
	bnx2x	==(;
	bnx2x_t)NULt,DI.eetxl	chalbp	==(eetxl	chalbp_t){
	/* P48xx eetxl	chalbp,DI.al12f);cific_ phy ==(al12f);cific_ phy_t){
	/* P48xx e);cific_ phy
};<<*paramsruO*p p = 0;
	u16 val1,al1284833 =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4833,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==(FLAGS4FAN4FAILURE_DEinREQ |n"		   FLAGS4REARM_LATCHl[IGNAL |n"		   FLAGS4TX_ERROR_CHECK),2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			100bPORx2xalf |n"		   SUP bnx			100bPORx2FulT |n"		   SUP bnx			1000bPORx2FulT |n"		   SUP bnx			10000bPORx2FulT |n"		   SUP bnx			TP |n"		   SUP bnx			AutonegM|n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXBASE_i,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)uct bnP48x3  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bnP48xx GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bnP48x3 bnx2xbnx2x,DI. case oloopback ==( case oloopback_t)NULt,DI.form<t(fw(str	==(form<t(fw(str_t)uct bnP48xx form<t(str,DI.;
	bnx2x	==(;
	bnx2x_t)uct bnP4833_;
	bnx2x_the ];.eetxl	chalbp	==(eetxl	chalbp_t){
	/* P48xx eetxl	chalbp,DI.al12f);cific_ phy ==(al12f);cific_ phy_t){
	/* P48xx e);cific_ phy
};<<*paramsruO*p p = 0;
	u16 val1,al1284834 =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4834,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==FLAGS4FAN4FAILURE_DEinREQ |n"		    FLAGS4REARM_LATCHl[IGNAL	_W.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			100bPORx2xalf |n"		   SUP bnx			100bPORx2FulT |n"		   SUP bnx			1000bPORx2FulT |n"		   SUP bnx			10000bPORx2FulT |n"		   SUP bnx			TP |n"		   SUP bnx			AutonegM|n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXBASE_i,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)uct bnP48x3  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bnP48xx GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bnP48x3 bnx2xbnx2x,DI. case oloopback ==( case oloopback_t)NULt,DI.form<t(fw(str	==(form<t(fw(str_t)uct bnP48xx form<t(str,DI.;
	bnx2x	==(;
	bnx2x_t)uct bnP4833_;
	bnx2x_the ];.eetxl	chalbp	==(eetxl	chalbp_t){
	/* P48xx eetxl	chalbp,DI.al12f);cific_ phy ==(al12f);cific_ phy_t){
	/* P48xx e);cific_ phy
};<<*paramsruO*p p = 0;
	u16 val1,al1284858 =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==FLAGS4FAN4FAILURE_DEinREQ |n"		    FLAGS4REARM_LATCHl[IGNAL	_W.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			100bPORx2xalf |n"		   SUP bnx			100bPORx2FulT |n"		   SUP bnx			1000bPORx2FulT |n"		   SUP bnx			10000bPORx2FulT |n"		   SUP bnx			TP |n"		   SUP bnx			AutonegM|n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXBASE_i,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DI.ableduplep	==0,DI.asrv		==0,DI. case ot bn	==( case ot bn_t)uct bnP48x3  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bnP48xx GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bnP48x3 bnx2xbnx2x,DI. case oloopback ==( case oloopback_t)NULt,DI.form<t(fw(str	==(form<t(fw(str_t)uct bnP485x form<t(str,DI.;
	bnx2x	==(;
	bnx2x_t)uct bnP4833_;
	bnx2x_the ];.eetxl	chalbp	==(eetxl	chalbp_t){
	/* P48xx eetxl	chalbp,DI.al12f);cific_ phy ==(al12f);cific_ phy_t){
	/* P48xx e);cific_ phy
};<<*paramsruO*p p = 0;
	u16 val1,al1254618s< =		br.rom;		== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TY54618SE,DI.addr		==0xff,DI.def_md_devad	==0,DI.flag_		==FLAGS4&=IT_W_CFGFIRSi,2x.r vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.t vareemphasis	=={0xffff, 0xffff, 0xffff, 0xffff},2x.mdio_ctrl	==0,DI.ey->typed	==(SUP bnx			10bPORx2xalf |n"		   SUP bnx			10bPORx2FulT |n"		   SUP bnx			100bPORx2xalf |n"		   SUP bnx			100bPORx2FulT |n"		   SUP bnx			1000bPORx2FulT |n"		   SUP bnx			TP |n"		   SUP bnx			AutonegM|n"		   SUP bnx			Pause |n"		   SUP bnx			Asym	Pause),DI.media_rom;	= ETHXS_EXBASE_i,2x.str_addr	==0,DI.ableflow_ctrl	==0,DI.able to determ	==0,DI.eterm_cax_8ask	==0,DIabl33leduplepesP*/0,DIabl3srvesP*/0,DI. case ot bn	==( case ot bn_t)	u16 v54618s<  case ot bn,DI.abS_EXlink_	==(abS_EXlink__t)uct bn54618s< GXS_EXlink_,DI.bnx2xbnx2x	==(bnx2xbnx2x_t)uct bn54618s< bnx2xbnx2x,DI. case oloopback ==( case oloopback_t)	u16 v54618s<  case oloopback,DI.form<t(fw(str	==(form<t(fw(str_t)NULt,DI.;
	bnx2x	==(;
	bnx2x_t)NULt,DI.eetxl	chalbp	==(eetxl	chalbp_t)	u16 v5461x eetxl	chalbp,DI.al12f);cific_ phy ==(al12f);cific_ phy_t){
	/* 54618s< e);cific_ phy
};</*****************************************************************/</*                                                               */</* Popult e aase l1,accorde s. Main  phy 
	}:
	u16 vaopult eval1,  */</*                                                               */</*****************************************************************/< *params_up) {
	/* aopult evareemphasis_"Setting SFP_NAM, sG_R") &
		POR,= 0;
	oo  p = 0;
	u16 val1, rx_Fsk_vFRCMF_MSG_LIIIIk_vFl12indexstatuficG:utaase4 lao odxgxs  case  rxlNPUTtx 
	fwsG_RrpesP0,Ttx sP0,Ti;mefARp(i			DR i < 2R i++le_eepMA_INGXS_ElNPUT_XGXS_E1R")e GPaasesamP	valuePloet 
	} ineep *Paases) &
. Wn n;numval1sty->gGXS
	}
than 1,
than tois valueeep *Pappli ods inltoT_XGXS_E1eep *Setpol tFl12index	;
	INGXS_El|| Fl12index	;
	_XGXS_E1te((va	rpesPnfig[port].)) &
		PORT_HW_C_LIIIX_LASER_MASK;
	DP(NETIF_MSG_LINK,  date media type for non-P*/
	fxgxs  case orx[i<<1]mres
 r	tpesPnfig[port].)) &
		PORT_HW_C_LIIIX_LASER_MASK;
	DP(NETIF_MSG_LINK,  date media type for non-P*/
	fxgxs  case otx[i<<1]mres rR first((va	rpesPnfig[port].)) &
		PORT_HW_C_LIIIX_LASER_MASK;
	DP(NETIF_MSG_LINK, date media type for non-P*/
	fxgxs  case 2orx[i<<1]mres
 r	tpesPnfig[port].)) &
		PORT_HW_C_LIIIX_LASER_MASK;
	DP(NETIF_MSG_LINK, date media type for non-P*/
	fxgxs  case 2orx[i<<1]mres	_Rd c	_TW->r vareemphasis[i << 1]E_S((rx>>16)hy,00ffffres	__TW->r vareemphasis[(i << 1) + 1]E_S(rpey,00ffffress	__TW->t vareemphasis[i << 1]E_S((tx>>16)hy,00ffffres	__TW->t vareemphasis[(i << 1) + 1]E_S(tpey,00ffffresPRdms *paramssG_R	u16 vgetxextval12or non("Setting SFP_NAM, sG_R") &
		POR,= 0;
	oo k_vFl12indexFsk_vFRCMstatu G_Rextval12or non;
	baleEp_modulFl12indexs		break;
_XGXS_E1:
[Sextval12or non;
	nfig[port].)) &
		PORT_HW_C_odule iX_LASER_MASK;
	DP(NETIF_MSG_LINK,date media type for non-P*/
	fexternalval12or non))al[SPREG_SFPeak;
_XGXS_E2:
[Sextval12or non;
	nfig[port].)) &
		PORT_HW_C_odule iX_LASER_MASK;
	DP(NETIF_MSG_LINK,date media type for non-P*/
	fexternalval12or non2))al[SPREG_SFP>ereadd:l[S;
	/* Low ==> corresInhy idlFl12index	_ASFP+vFl12indexsal[S_*param-EINVALas }rRS *paramextval12or nonimms*params,
					   aopult ev,
	_al1("Setting SFP_NAM, sG_R") &
		POR,sk_vFRCMF_MSG_LIp = 0;
	u16 val1, rx_statu G_Ral12addras  venrhip_idas  venEp_modf	fg	=m(nfig[port].)) &
		PORT_HW_C_LIIIe iX_LASER_MASK;
	DP(NETIF_MSG_LINK,date media typORT_FEA  case -P*/
	fl	chaor non))/&INK,EP bnx2FEATURE_CONNECx			SWITCHlg modSFPehip_id	=m(nfig[port].n = tx_e_CHIP_NUM) << 16wa|INK((nfig[port].n = tx_e_CHIP_REV)hy,00f) << 12)		
e;
	/* Low ==> corres:ehip_id	=m	bnx2x_clehip_iddSFPol tUSES_WARPCOREort))t((va venEerdes_netxifes	__TW2addr;
	nfig[port]HW_C_LIn = tx_e_WC0_C_10XS_EXADDRsal[S rx_E_SIl12warpcmanimeout *nfig[port].n = tx_e_ bnx4EADDL_M_OVWR)	;
		x3	sp rp OUTflag_/|==FLAGS44_ bnxDEADDimeofirssp rp OUTflag_/O_P~FLAGS44_ bnxDEADDimeorac- = valDunshmUTP"*SetpEerdes_netxif	=m(nfig[port].)) &
		PORT_HW_C_CX_LASER_MASK;
	DP(NETIF_MSG_L date mediHW_C_Ca type for non-P*/
	f>ereadd2cfg;
_&INK,	= bnx2x_8727_NEinSERDEFGLow  modSFPdMA_S0G aaseapproprit e ey->typedlNPUTflag_/y, Set 
	}s pereep *Pin	erfacGPaom;
of modurhip
Pp *SetpEp_modulEerdes_netxif)t((vaeak;
 bnx2x_8727_NEinSERDEFGLowSGMII:sp rp OUTey->typedl&==(SUP bnx			10bPORx2xalf |n"				   SUP bnx			10bPORx2FulT |n"				   SUP bnx			100bPORx2xalf |n"				   SUP bnx			100bPORx2FulT |n"				   SUP bnx			1000bPORx2FulT |n"				   SUP bnx			FIBRE |n"				   SUP bnx			AutonegM|n"				   SUP bnx			Pause |n"				   SUP bnx			Asym	Pause);sp rp OUTmedia_rom; = ETHXS_EXBASE_i;sp rPREG_SFPaeak;
 bnx2x_8727_NEinSERDEFGLowXFI:sp rp OUTey->typedl&==(SUP bnx			1000bPORx2FulT |n"				   SUP bnx			10000bPORx2FulT |n"				   SUP bnx			FIBRE |n"				   SUP bnx			Pause |n"				   SUP bnx			Asym	Pause);sp rp OUTmedia_rom; = ETHXS_EXXFP	FIBER;sp rPREG_SFPaeak;
 bnx2x_8727_NEinSERDEFGLowSFI:sp rp OUTey->typedl&==(SUP bnx			1000bPORx2FulT |n"				   SUP bnx			10000bPORx2FulT |n"				   SUP bnx			FIBRE |n"				   SUP bnx			Pause |n"				   SUP bnx			Asym	Pause);sp rp OUTmedia_rom; = ETHXS_EXSFPP2SG_LFIBER;sp rPREG_SFPaeak;
 bnx2x_8727_NEinSERDEFGLowKR:sp rp OUTmedia_rom; = ETHXS_EXKR;sp rp OUTey->typedl&==(SUP bnx			1000bPORKX2FulT |n"				   SUP bnx			netd0bPORKR2FulT |n"				   SUP bnx			FIBRE |n"				   SUP bnx			AutonegM|n"				   SUP bnx			Pause |n"				   SUP bnx			Asym	Pause);sp rPREG_SFPaeak;
 bnx2x_8727_NEinSERDEFGLowDW_CF:sp rp OUTmedia_rom; = ETHXS_EXKR;sp rp OUTflag_/|==FLAGS4WC_DUALDEADDimeorp OUTey->typedl&==(SUP bnx			2etd0bPORMLD22FulT |n"				   SUP bnx			FIBRE |n"				   SUP bnx			Pause |n"				   SUP bnx			Asym	Pause);sp rPREG_SFPaeak;
 bnx2x_8727_NEinSERDEFGLowKR2:sp rp OUTmedia_rom; = ETHXS_EXKR;sp rp OUTflag_/|==FLAGS4WC_DUALDEADDimeorp OUTey->typedl&==(SUP bnx			2etd0bPORKR22FulT |n"				   SUP bnx			netd0bPORKR2FulT |n"				   SUP bnx			1000bPORKX2FulT |n"				   SUP bnx			AutonegM|n"				   SUP bnx			FIBRE |n"				   SUP bnx			Pause |n"				   SUP bnx			Asym	Pause);sp rp OUTflag_/O_P~FLAGS4TX_ERROR_CHECK;sp rPREG_SFPa>ereadd:l[Se;
	/* Low ==> corresUnknown WCPin	erfacGPaom;
bbnx2x_c
;BC_LIIIe iEerdes_netxif);sp rPREG_SFPa}

 rac- nion
 MDC/bnx2 vor	-arouPUTfor E3 A0 sincGPfree runoe stMDCeep *PwasTr, pASE asTexpecped. For B0, EC2 vilT be enion
d so modreeep *Pwon't be an issue modreeep *Setpol tCHIP_REVort)	;
	CHIP_REV_Ax	sp rp OUTflag_/|==FLAGS4MDC_bnx2xWAimeofirssp rp OUTflag_/|==FLAGS4MDC_bnx2xWA_BbaleR first((vaEp_modulEp_modf	fg)t((vaeak;
SWITCHl727_1G:sp rp O2addr;
	nfig[port]HW_C__LINDUAre
	SERDEF0_C_10XS_EXADDRT_HW_C_odu of  *00)10l;t rS rx_E_SIl12Eerdes;sp rPREG_SFPaeak;
SWITCHl727_10G:sp rp O2addr;
	nfig[port]HW_C__LINDUAre
	W_CF0_C_10XS_EXADDRT_HW_C_odu of  *00)18l;t rS rx_E_SIl12xgxs;sp rPREG_SFPa>ereadd:l[Se;
	/* Low ==> corresInhy idlEp_modf	fgfw_resp[S_*param-EINVALas  _P_Rdep OUTaddr;
	(u8)al12addras p OUTmdio_ctrl;
		u16 vgetxemac		PORort]HW_C__LI  SHAR			x_8727_MDC_bnx2xACCESS1_BOTH]HW_C__LI  FRCMwer_ol tCHIP_IS_E2ort))s	__TW->def_md_devad = E2_DEFAULGXS_EXDEV_ADDRer_firssp _TW->def_md_devad = DEFAULGXS_EXDEV_ADDRer
e;
	/* Low ==> corresInternalOal1,aRCM=%d, addr=bbnx, mdio_ctl=bbnx2x_c
;BC CFRCMF p OUTaddrF p OUTmdio_ctrlweraduct bnaopult evareemphasis_rt].)) &
		PORe the rtRCMF INGXS_E)al[ *param0rams *params,
					   aopult evextval1("Setting SFP_NAM,
;BC_LIk_vFl12indexF
;BC_LIkG_R") &
		POR,= 0;
	okG_R") &
2		POR,= 0;
	ok_vFRCMF_MSG_LIp = 0;
	u16 val1, rx_statu G_Rextval12or non+vFl12aom;,  case 2as  venmdc_mdio_access =	SHAR			x_8727_MDC_bnx2xACCESS1_BOTHer_fxtval12or non;
		u16 vgetxextval12or non(rt].)) &
		POReHW_C__odu l12indexFsFRCMwer_Fl12aom;;
	W_CFG_XGXS_EXT_PH(extval12or nonwer_ficSelecp aase l1,aom;;*SetEp_modulFl12aom;s		break;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP073:l[Smdc_mdio_access =	SHAR			x_8727_MDC_bnx2xACCESS1_SWAPPEDesrS rx_E_SIl12P073al[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP705:srS rx_E_SIl12P705al[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP706:srS rx_E_SIl12P706al[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP726:l[Smdc_mdio_access =	SHAR			x_8727_MDC_bnx2xACCESS1_EMAC1esrS rx_E_SIl12P726al[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP727_NOC:
 rac-_TYP727_NOC =>-_TYP727Tr, ostrhcurrenut"KR Imdc_mdio_access =	SHAR			x_8727_MDC_bnx2xACCESS1_EMAC1esrS rx_E_SIl12P727es	__TW->flag_/|==FLAGS4NOCal[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP722:FPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP727:l[Smdc_mdio_access =	SHAR			x_8727_MDC_bnx2xACCESS1_EMAC1esrS rx_E_SIl12P727al[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP481:srS rx_E_SIl12P481al[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4823:srS rx_E_SIl12P4823al[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4833:srS rx_E_SIl12P4833al[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4834:srS rx_E_SIl12P4834al[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858:srS rx_E_SIl12P4858al[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TY54616:FPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TY54618SE:srS rx_E_SIl1254618s<imeout *Fl12aom;;
== bnx2x_8727_W_CFG_XGXS_EXT_PHY_TY54618SE	sp rp OUTflag_/|==FLAGS4EEEal[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY
FX7101:srS rx_E_SIl127101al[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHYFAILURE:srS rx_E_SIl12nulTal[S_*param-EINVALas >ereadd:l[S rx_E_SIl12nulTal[SMA_In eak;
externalOS_Elwasn't fouPUT*Setpol t*Fl12aom;;!== bnx2x_8727_W_CFG_XGXS_EXT_PHYDIRECTted\INKLI  *Fl12aom;;!== bnx2x_8727_W_CFG_XGXS_EXT_PHYNOT_CON	amsp r_*param-EINVALas   *param0raa}

 p OUTaddr;
	W_CFG_XGXS_EXADDR(extval12or nonwer_uct bnaopult evareemphasis_rt].)) &
		PORe the rtRCMF Fl12indexsalr_ficTase)) &
 address of modurx_Estrs
	}MisPloet 
d 	}Mdifferenu
PMA_p = 0;ures._In eak;
tois p = 0;ureMisPto, old, doTr, pASE
p *Paaseaddressve	ontr case 2;
	nfig[port].)) &
		PORT_iX_LASER_MASK;
	DP(NETIF_MSG_LINK,K,date medi")e Gdpe for non.or non2))al[ol tFl12index	;
	_XGXS_E1te((vapTW->str_addr;
	)) &
		PORT_iX_LASER_MASK;
	DP(NETIF_MSG_LINK,Ka typmb-P*/
	fextval12fw(strs
	}s;

 rac- = vale);cificnmdc mdiopASEcoppsT*Setpol t case 2;&	SHAR			x_8727_MDC_bnx2xACCESS1_  modINK,mdc_mdio_access =	 case 2;&INK,SHAR			x_8727_MDC_bnx2xACCESS1_  moaleR first((vakG_R"ize;
	nfig[port].)) &
2		PORtimmeout *"ize;>
C_LIIIX_LASER_MASK;
	DP(NET2IF_MSG_L extval12fw(strs
	}2))t((vaapTW->str_addr;
	)) &
2		PORT_HW_CLIIIX_LASER_MASK;
	DP(NET2IF_MSG_L
;BC_LIIIeextval12fw(strs
	}2-P*/
	res	_Rd rac- = vale);cificnmdc mdiopASEcoppsT*Setpol t case 2;&	SHAR			x_8727_MDC_bnx2xACCESS2_  modINK,mdc_mdio_access =	( case 2;&INK,SHAR			x_8727_MDC_bnx2xACCESS2_  mod >>
C_	(SHAR			x_8727_MDC_bnx2xACCESS2_SHIFT -HW_CLSHAR			x_8727_MDC_bnx2xACCESS1_SHIFTresPRd p OUTmdio_ctrl;
		u16 vgetxemac		PORort]nmdc_mdio_accessFsFRCMwerl[ol tuct bn,s2P483bnP485xtFl1ted\ tFl1->str_addr)le_eepMA_Remost 100Mb p;
	"ey->typedlfor _TYP4833/4 wn n;rx_Efweep *Pstrs
	}MlowGP than or equnshto 1.39eep *SetpsG_Rraw(str;
	nfig[port].Fl1->str_addr)imeout *((raw(str;y,007F) <= 39ted\INKLI  *((raw(str;y,00F80f >> 7) <= 1amsp rp OUTey->typedl&==~(SUP bnx			100bPORx2xalf |n"		__LI  SUP bnx			100bPORx2FulT)raa}

 ;
	/* Low ==> corresFl12aom;;bbnxu of  %d fouPUTi} index	_ASFP+
;BC CFl12aom;, tRCMF Fl12indexsal ;
	/* Low ==> corres             addr=bbnx, mdio_ctl=bbnx2x_c
;BC CF OUTaddrF p OUTmdio_ctrlwer[ *param0rams *params,
					   aopult eval1("Setting SFP_NAM, s_vFl12indexFskG_R") &
		POR,= 0;      kG_R") &
2		POR,ok_vFRCMFIp = 0;
	u16 val1, rx_statu_TW->tom;;
	 bnx2x_8727_W_CFG_XGXS_EXT_PHYNOT_CON	al[ol tFl12index	;
	INGXS_E)s   *param			   aopult ev,
	_al1(rt].)) &
		PORe tRCMF Fl1werl[ *param			   aopult evextval1(rt].Fl12indexFs)) &
		PORe ") &
2		POR,= 0;
	tRCMF Fl1werms *params_up) {
	/* Fl12def_cfn("Settinarams->bp;
	DP(NETIF_MSG 	oo  p = 0;
	u16 val1, rx_F= 0;      k_vFl12indexstatuEPROM_VENDOR_NAME_SIZE+1];
	char G_Rl	chaor noner_ficPopult e aase>ereadd;rx_Eor nonurt 
	} for MFhmUTP"*Setol tFl12index	;
	_XGXS_E2le_eepl	chaor non;
	nfig[port].FZE+1];
)) &
		PORT_HW_C_LIIIeX_LASER_MASK;
	DP(NETIF_MSG_L date mediHW_Ca typORT_FEA  case -Pfp_ctrlFRCM	fl	chaor non2))al[Sp OUTeterm_cax_8ask;
	nfig[port].FZE+1];
)) &
		PORT_HW_C__LIIIeX_LASER_MASK;
	DP(NETIF_MSG_LHW_C__oduuuuudate mediHW_Ca type for non-Pfp_ctrlFRCM	feterm_caxability_8ask2))al[R first((val	chaor non;
	nfig[port].FZE+1];
)) &
		PORT_HW_C_LIIIeX_LASER_MASK;
	DP(NETIF_MSG_L date mediHW_CCa typORT_FEA  case -Pfp_ctrlFRCM	fl	chaor non))al[Sp OUTeterm_cax_8ask;
	nfig[port].FZE+1];
)) &
		PORT_HW_C__LIIIeX_LASER_MASK;
	DP(NETIF_MSG_LHW_C__oduuuuudate mediHW_Ca type for non-Pfp_ctrlFRCM	feterm_caxability_8ask)resPRd ;
	/* Low ==> corr
oduu"Dereadd;or non;rx_Eidx	_x 	fg	bbnxueterm_cax_8ask;bbnx2x_c
; du l12indexFsl	chaor nonF p OUTeterm_cax_8askwerl[p OUT33leduplepesPDUPLEX_FULtachEp_modull	chaor non;;y, bnx2FEATURE_L(val[
				  mod 	break;
 bnx2FEATURE_L(val[
				10M_HALF:
[Sp OUT33leduplepesPDUPLEX_HALF;break;
 bnx2FEATURE_L(val[
				10M_FULt:
[Sp OUT33le	{
< 1);
				}
				neal[SPREG_SFPeak;
 bnx2FEATURE_L(val[
				100M_HALF:
[Sp OUT33leduplepesPDUPLEX_HALF;break;
 bnx2FEATURE_L(val[
				100M_FULt:
[Sp OUT33le	{
< 1);
				}
				neeal[SPREG_SFPeak;
 bnx2FEATURE_L(val[
				1G:
[Sp OUT33le	{
< 1);
				}
				neeeal[SPREG_SFPeak;
 bnx2FEATURE_L(val[
				2_5G:
[Sp OUT33le	{
< 1);
				}
				2500al[SPREG_SFPeak;
 bnx2FEATURE_L(val[
				10GXCX4:
[Sp OUT33le	{
< 1);
				}
				neee0al[SPREG_SFP>ereadd:l[Sp OUT33le	{
< 1);
				}
				AUTO_NEGal[SPREG_SFP}

 Ep_modull	chaor non;;y, bnx2FEATURE_FLOW_CONTROL	  mod 	break;
 bnx2FEATURE_FLOW_CONTROL	AUTO:l[Sp OUT33leflow_ctrl			BNX2X_FLOW_C_10XAUTOal[SPREG_SFPeak;
 bnx2FEATURE_FLOW_CONTROL	TX:l[Sp OUT33leflow_ctrl			BNX2X_FLOW_C_10XTXal[SPREG_SFPeak;
 bnx2FEATURE_FLOW_CONTROL	RX:l[Sp OUT33leflow_ctrl			BNX2X_FLOW_C_10XRXal[SPREG_SFPeak;
 bnx2FEATURE_FLOW_CONTROL	BOTH:l[Sp OUT33leflow_ctrl			BNX2X_FLOW_C_10XBOTHer_SPREG_SFP>ereadd:l[Sp OUT33leflow_ctrl			BNX2X_FLOW_C_10XNONEal[SPREG_SFP}
}

sG_R	u16 vIl12EelecpSG_("Settinarams->bp;
	DP(NETIstatu G_Ral12 case oswappedF prio_cfghar G_R *paraf	fg	=m bnx2x_8727_S_EXSELECG;
	_HARDWARE_DEFAULGerl[p O2 case oswappedE_SIZE+1];
maddival12or non;\INK bnx2x_8727_S_EXSWAPPED_ENABLEDerl[prio_cfgE_SIZE+1];
maddival12or non;\INKK bnx2x_8727_S_EXSELECG;
	_  moaletol tFl12 case oswapped)t((vaEp_modulprio_cfg)t((vaeak;
 bnx2x_8727_S_EXSELECG;
	_FIRSi_S_EXPRIORITY:INKLI  R *paraf	fg	=m bnx2x_8727_S_EXSELECG;
	_SECOND_S_EXPRIORITY;INKLI  RPREG_SFPaeak;
 bnx2x_8727_S_EXSELECG;
	_SECOND_S_EXPRIORITY:INKLI  R *paraf	fg	=m bnx2x_8727_S_EXSELECG;
	_FIRSi_S_EXPRIORITY;INKLI  RPREG_SFPaeak;
 bnx2x_8727_S_EXSELECG;
	_SECOND_S_E:INKLI  R *paraf	fg	=m bnx2x_8727_S_EXSELECG;
	_FIRSi_S_E;INKLI  RPREG_SFPaeak;
 bnx2x_8727_S_EXSELECG;
	_FIRSi_S_E:INKLI  R *paraf	fg	=m bnx2x_8727_S_EXSELECG;
	_SECOND_S_E;INKLI  RPREG_SFPa}l[R firss   *para_cfgE_SIrio_cfghal[ *param *para_cfgerms ,
					   al12probe("Settinarams->bp;
	DP(NETIstatu _vFl12indexFsactualval12idx;tu G_Ral12 case oswappedF sync_X_LASE, media_rom;s;spEPROM_VENDOR_NAME_SIZE+1];
	char" = 0;
	u16 val1, rx_er_FZE+1];
numval1st
	bale;
	/* Low ==> corresB_MSn;rx_Eprobefw_respp O2 case oswappedE_SIZE+1];
maddival12or non;\INK bnx2x_8727_S_EXSWAPPED_ENABLEDerl[fARp(Fl12index	;	INGXS_E;lFl12index	< MAXXS_ES;
oduuuuuFl12index++le_eepactualval12idxE_SIl12indeximeout *Fl12 case oswapped)t((va[ol tFl12index	;
	_XGXS_E1tHW_C_actualval12idxE_S_XGXS_E2esp[Sfirstol tFl12index	;
	_XGXS_E2lHW_C_actualval12idxE_S_XGXS_E1es	_Rd r;
	/* Low ==> corresFl12 case oswappedEnx, Fl12index	nx,"= 0;       "sactualval12idx nx2x_clal12 case oswappedF= 0;   Fl12indexFsactualval12idx)al[Sp OE_S&Pfp_ctrlFhy[actualval12idx]imeout *			   aopult eval1(rt].Fl12indexFsFZE+1];
)) &
		PORc
;BC_LIIIe iFZE+1];
)) &
2		POR,oPfp_ctrlFRCMc
;BC_LIIIe iFl1te!
	b)t((vaapZE+1];
numval1st
	bale r;
	/* Low ==> corresFl1Eprobe fain
d Sn;rx_Eindex	_ASFP+
;B0;   Fl12indexresp[SfARp(Fl12index	;	INGXS_E;= 0;      Fl12index	< MAXXS_ES;
o0;      Fl12index++l
;B0; rx_E_SIl12nulTal[SS_*param-EINVALas  _P_[ol tFl1->tom;;
== bnx2x_8727_W_CFG_XGXS_EXT_PHYNOT_CON	a
p rPREG_SFP_[ol tFZE+1];
ORT_FEA  case _flag_/OINKLI  FEATURE_CONFIG_DISABLE_REMOTHYFAULGXDET	sp rp OUTflag_/O_P~FLAGS4TX_ERROR_CHECK;sP_[ol t!tFZE+1];
ORT_FEA  case _flag_/OINKLI    FEATURE_CONFIG_MT_SUP bnx)	sp rp OUTflag_/|==FLAGS4MDC_bnx2xWA_G;sP_[sync_X_LASEE_SIZE+1];
)) &
		PORT_HW_CX_LASER_MASK;
	DP(NETIF_MSG_LINK,date media type for non-Pfp_ctrlFRCM	fmedia_rom;resp[media_rom;s;
	nfig[port].)ync_X_LASEs;

 rac-Updt e media,aom;;fARpnon-PMF.)yncds inlfARpaasefirst timeeep *PIn eak;
toe media,aom;;chang;s;afterwards, it vilT be updt edeep *Puse sttoe updt eEXlink_  phy 
	}eep *Setput *(media_rom;s;& ( bnx2x_8727_MEDIAXT_PHYS_E0_  mo <<
;BC_LIII( bnx2x_8727_MEDIAXT_PHYS_E1_SHIFT *
;BC_LIIIeactualval12idx)))	;
		)t((vaamedia_rom;s;|= t*Fl1UTmedia_rom; &INK,		 bnx2x_8727_MEDIAXT_PHYS_E0_  mo) <<
;BC_( bnx2x_8727_MEDIAXT_PHYS_E1_SHIFT *
;BC_Lactualval12idx))as  _P_[x_e_WRort].)ync_X_LASE, media_rom;ss;

 r{
	/* Fl12def_cfn(P(NETIF the rtl12indexresp[pZE+1];
numval1s++raa}

 ;
	/* Low ==> corresEnd;rx_Eprobe. #al1stfouPUTnx2x_claZE+1];
numval1swer[ *param0rams *params_up) {
	/* t bn_bmac	loopback("Settinarams->bp;
	DP(NETIF_MSG
	oo  p = 0;
aramsvar
	Dvar
statuEPROM_VENDOR_NAME_SIZE+1];
	char	var
->aramsuME_S1har	var
->ara< 1);
				}
				neee0al[Svar
->duplepesPDUPLEX_FULtachSvar
->flow_ctrl			BNX2X_FLOW_C_10XNONEal[Svar
->mac	tom;;
	MACXT_PHY_MAC;

 rvar
->al12flag_/= S_EXX_CFGFLAG;

 r{
	/* xgxs deassert(P(NETIs;

 rac-S0G bmac loopback *Setp{
	/* bmac	enion
(P(NETIF var
, 1,
1s;

 rx_e_WRort].NDUAre
	EGRESS_DRAIN0DEADD +oPfp_ctrlFRCM*4, 0werms *params_up) {
	/* t bn_emac	loopback("Settinarams->bp;
	DP(NETIF_MSG
	oo  p = 0;
aramsvar
	Dvar
statuEPROM_VENDOR_NAME_SIZE+1];
	char	var
->aramsuME_S1har	var
->ara< 1);
				}
				neeeal[Svar
->duplepesPDUPLEX_FULtachSvar
->flow_ctrl			BNX2X_FLOW_C_10XNONEal[Svar
->mac	tom;;
	MACXT_PHYEMAC;

 rvar
->al12flag_/= S_EXX_CFGFLAG;

 r{
	/* xgxs deassert(P(NETIs;
 rac-S0G bmac loopback *Setp{
	/* emac	enion
(P(NETIF var
, 1)al[SP
	/* emac	progNET(P(NETIF var
)al[Sx_e_WRort].NDUAre
	EGRESS_DRAIN0DEADD +oPfp_ctrlFRCM*4, 0werms *params_up) {
	/* t bn_xmac	loopback("Settinarams->bp;
	DP(NETIF_MSG
	oo  p = 0;
aramsvar
	Dvar
statuEPROM_VENDOR_NAME_SIZE+1];
	charvar
->aramsuME_S1harol t!IZE+1];
33le	{
< 1);
	[0])ar	var
->ara< 1);
				}
				neee0al[firss  var
->ara< 1);
				IZE+1];
33le	{
< 1);
	[0]harvar
->duplepesPDUPLEX_FULtachvar
->flow_ctrl			BNX2X_FLOW_C_10XNONEal[var
->mac	tom;;
	MACXT_PHYXMAC;
rvar
->al12flag_/= S_EXX_CFGFLAG;
rac-S0G WChto loopback mUTP"sincGPp;
	"isl33luir
		to proviTP"clock
p *Pao mod XMAC Sn;20G mUTPve	ontruct bnf:ufaer_mmd(P(NETIF &Pfp_ctrlFhy[0]wer_uct bnwarpcman abeetxlanRort]n&Pfp_ctrlFhy[0], 0wer	Pfp_ctrlFhy[INGXS_E]. case oloopback(_MSG&Pfp_ctrlFhy[INGXS_E],(vaapZE+1]weraduct bnxmac	enion
(P(NETIF var
, 1)al[x_e_WRort].NDUAre
	EGRESS_DRAIN0DEADD +oPfp_ctrlFRCM*4, 0werms *params_up) {
	/* t bn_umac	loopback("Settinarams->bp;
	DP(NETIF_MSG
	oo  p = 0;
aramsvar
	Dvar
statuEPROM_VENDOR_NAME_SIZE+1];
	charvar
->aramsuME_S1harvar
->ara< 1);
				}
				neeeal[var
->duplepesPDUPLEX_FULtachvar
->flow_ctrl			BNX2X_FLOW_C_10XNONEal[var
->mac	tom;;
	MACXT_PHYUMAC;
rvar
->al12flag_/= S_EXX_CFGFLAG;
r{
	/* umac	enion
(P(NETIF var
, 1)all[x_e_WRort].NDUAre
	EGRESS_DRAIN0DEADD +oPfp_ctrlFRCM*4, 0werms *params_up) {
	/* t bn_xgxs loopback("Settinarams->bp;
	DP(NETIF_MSG
	oo  p = 0;
aramsvar
	Dvar
statuEPROM_VENDOR_NAME_SIZE+1];
	char" = 0;
	u16 val1, ,
	_al1E_S&Pfp_ctrlFhy[INGXS_E]harvar
->aramsuME_S1harvar
->flow_ctrl			BNX2X_FLOW_C_10XNONEal[var
->duplepesPDUPLEX_FULtachol tFZE+1];
33le	{
< 1);
	[0]	;
	}
				neee)ar	var
->ara< 1);
				}
				neeeal[firs ol t*FZE+1];
33le	{
< 1);
	[0]	;
	}
				2etd0) ||n"	 (,
	_al1UTflag_/O=FLAGS4WC_DUALDEADD))ar	var
->ara< 1);
				}
				2eee0al[firss  var
->ara< 1);
				}
				neee0alarol t!USES_WARPCOREort))
 r{
	/* xgxs deassert(P(NETIs;
 {
	/* aramst bnialize(P(NETIF var
)alchol tFZE+1];
33le	{
< 1);
	[0]	;
	}
				neee)t((vaol tUSES_WARPCOREort))
p rP
	/* umac	enion
(P(NETIF var
, 0l;t rfirst((va	P
	/* emac	progNET(P(NETIF var
)al[Sp{
	/* emac	enion
(P(NETIF var
, 0)as  _P_R first((vaol tUSES_WARPCOREort))
p rP
	/* xmac	enion
(P(NETIF var
, 0l;t rfirs
p rP
	/* bmac	enion
(P(NETIF var
, 0, 1)al[}lchol tFZE+1];
loopback_mUTP";
	LOOPBACKXX_CFle_eepMA_S0G 10G X_CF loopback *Setp,
	_al1UT case oloopback(,
	_al1claZE+1])al[R first((vaMA_S0G externalOal1Eloopback *Setp _vFl12index;t rfARp(Fl12index	;	_XGXS_E1es	_      Fl12index	< aZE+1];
numval1s; Fl12index++l
;B0ol tFZE+1];
Fhy[Fl12index]. case oloopbackl
;B0;FZE+1];
Fhy[Fl12index]. case oloopback(INK,		&FZE+1];
Fhy[Fl12index],= 0;
	tZE+1])al[Rl[x_e_WRort].NDUAre
	EGRESS_DRAIN0DEADD +oPfp_ctrlFRCM*4, 0wertruct bnf:ufled(P(NETIF var
, L			 ADDLOPERF var
->ara< 1);
	werms _up) {
	/* f:ufrx_filter("Settinarams->bp;
	DP(NETIF k_venstatuEPROM_VENDOR_NAME_SIZE+1];
	char 8 valE_Sen *00)1Falr_ficOpen /"clok;
toe gt e between
toe NDUlNPUTtoe BRB"*Setol t!CHIP_IS_E1xort))
p valE|_Sen *00)20al[x_e_WRort].NDUAre
	LLH0_BRB1_DRV_  mo +oPfp_ctrlFRCM*4, val)alchol t!CHIP_IS_E1ort))t((vax_e_WRort].NDUAre
	LLH0_BRB1_DRV_  mo_MF +oPfp_ctrlFRCM*4,s	_      Sen*	x3	al[}lchx_e_WRort].(Pfp_ctrlFRCM ?.NDUAre
	LLH1_BRB1_NOT_MCP :INKLI  NDUAre
	LLH0_BRB1_NOT_MCP),vensimms*params,
					   a_up) aramsflap("Settinarams->bp;
	DP(NETIF_MSG
	oo  p = 0;
aramsvar
	Dvar
statu G_Ral12idx;tu G_Rdo
	_clearEXlin, lfaEXls;spEPROM_VENDOR_NAME_SIZE+1];
	chatruct bnf:ufmdio_emac	per_al1(rt].FZE+1])al[MA_Syncdtoe p;
	"FZE+1eters	ontruct bnaramsXlink__updt e(P(NETIF var
)alch/*
p *PTasemUTulePstrifict 
	} wasTalGXS_yRdo
e b_Eprevious p;
	"owntr,DIMA_po
tois call"islmea
		s inltoTg0G waroe stmessagPve	ontl[fARp(Fl12idx	;	INGXS_E;lFl12idx	< aZE+1];
numval1s; Fl12idx++le_eep" = 0;
	u16 val1, rx_E_S&Pfp_ctrlFhy[al12idx]imeout *al1UTpl12f);cific_ phy)t((vaa;
	/* Low ==> corresCalle stS_Ele);cificn phyfw_resp[Sal1UTpl12f);cific_ phy(al1claZE+1], S_EX&=IT)as  _P_[ol t*Fl1UTmedia_rom; == ETHXS_EXSFPP2SG_LFIBER) ||n"	    *Fl1UTmedia_rom; == ETHXS_EXSFP_1_LFIBER) ||n"	    *Fl1UTmedia_rom; == ETHXS_EXDA_TWINAX))
p rP
	/* strif12ffp_mUTule(al1claZE+1])al[R
	lfaEXls;
	nfig[port].FZE+1];
lfaE	PORT_HW_CLX_LASER_MASK;
	DP(NETIlfaF_MSG
	olfaEXlsmres
 do
	_clearEXlin;
	lfaEXls;&	SHMEM_LFA_DONGXCLEAR_STATalr_ficRe-enion

toe NDU/MAC *Setol tCHIP_IS_E3ort))t((vaol t!do
	_clearEXlin)t((vaax_e_WRort].GRCBASE_n = T_HW_CLIII LIn = tx_eISTERS4RESEinREG_2XCLEARF= 0;       (n = tx_eISTERS4RESEinREG_2XMSTAT0 <<
;BC_Pfp_ctrlFRCM))as  ax_e_WRort].GRCBASE_n = T_HW_CLIII LIn = tx_eISTERS4RESEinREG_2XSEiF= 0;       (n = tx_eISTERS4RESEinREG_2XMSTAT0 <<
;BC_Pfp_ctrlFRCM))as  _P_[ol tvar
->ara< 1);
		<	}
				neee0)
p rP
	/* umac	enion
(P(NETIF var
, 0l;t rfirs
p rP
	/* xmac	enion
(P(NETIF var
, 0l;t R first((vaol tvar
->ara< 1);
		<	}
				neee0)
p rP
	/* emac	enion
(P(NETIF var
, 0)as  firs
p rP
	/* bmac	enion
(P(NETIF var
, 0, !do
	_clearEXlin)al[}lchMA_IncremenutLFA counut"KR lfaEXls;
	((lfaEXls;&	~L(valFLAP_AVOIDANCE_COUNGX  mo) |n"	   (((((lfaEXls;&	L(valFLAP_AVOIDANCE_COUNGX  mo) >>
C_LIIIIIIL(valFLAP_AVOIDANCE_COUNGXOFFSEi) + 1)hy,00ff)
C_LIII<<IL(valFLAP_AVOIDANCE_COUNGXOFFSEi))al[MA_Clear p;
	"flap GXSs	} "KR lfaEXls;O_P~LFA_L(valFLAP_REAS
	_  moaletx_e_WRort].FZE+1];
lfaE	PORT_HWLIIIIIIX_LASER_MASK;
	DP(NETIlfaFolfaEXlsmFolfaEXlsmalr_ficDision

NDUlDRAIN "KR x_e_WRort].NDUAre
	EGRESS_DRAIN0DEADD +oPfp_ctrlFRCM*4, 0wertrac- nion
 in	errupts	ontruct bnarams,
	_enion
(P(NETIwer[ *param0rams *params_up) {
	/* cannot a_up) aramsflap("Settinarams->bp;
	DP(NETIF_MSG
	op = 0;
aramsvar
	Dvar
F_MSG
	o,
		lfaEXlink_statu G_RlfaEXls, 	fg2idx, tmpsval;spEPROM_VENDOR_NAME_SIZE+1];
	chatruct bnbnx2xbnx2x(P(NETIF var
, 1)all[ol t!IZE+1];
lfaE	POR)s   *paraal[MA_Sto GPaasenew p;
	"FZE+1eters	ontrx_e_WRort].FZE+1];
lfaE	PORT_HWLIIIIIIX_LASER_MASK;
	DP(NETIlfaFo33leduplep),
_LIIIe iFZE+1];
33leduplep[0]	| tFZE+1];
33leduplep[1] << 16w)all[x_e_WRort].FZE+1];
lfaE	PORT_HWLIIIIIIX_LASER_MASK;
	DP(NETIlfaFo33leflow_ctrl),
_LIIIe iFZE+1];
33leflow_ctrl[0]	| tFZE+1];
33leflow_ctrl[1] << 16w)all[x_e_WRort].FZE+1];
lfaE	PORT_HWLIIIIIIX_LASER_MASK;
	DP(NETIlfaFo33leara< 1);
	w,
_LIIIe iFZE+1];
33le	{
< 1);
	[0]	| tFZE+1];
33le	{
< 1);
	[1] << 16w)all[fARp(	fg2idxt
	ba 	fg2idx	<	}HMEM_L(valCONFIG_SIZEa 	fg2idx++le_eepx_e_WRort].FZE+1];
lfaE	PORT_HWWLIIIIIIX_LASER_MASK;
	DP(NETIlfaF_MSG
eterm_cax_8ask[	fg2idx]),s	_      SIZE+1];
)term_cax_8ask[	fg2idx])al[}lchtmpsval;
	nfig[port].FZE+1];
lfaE	PORT_HW_CLX_LASER_MASK;
	DP(NETIlfaF addi 
	}alaor non))al[tmpsval;O_P~REQ_FC	AUTO_ADV_  moaletmpsval;|		IZE+1];
33lefc_auto_advall[x_e_WRort].FZE+1];
lfaE	PORT_HWLIIIIIIX_LASER_MASK;
	DP(NETIlfaFoaddi 
	}alaor non), tmpsval)all[lfaEXls;
	nfig[port].FZE+1];
lfaE	PORT_HW_CLX_LASER_MASK;
	DP(NETIlfaFolfaEXlsmres
 MA_Clear aase"Don't Clear Sparasrams" bin, NPUTASEEGXSs	} "KR lfaEXls;O_P~SHMEM_LFA_DONGXCLEAR_STATalr_ficS0G p;
	"flap GXSs	} "KR lfaEXls;O_P~LFA_L(valFLAP_REAS
	_  moal lfaEXls;|
	((lfaEXlink_ & LFA_L(valFLAP_REAS
	_  mo) <<
;BIIIILFA_L(valFLAP_REAS
	_OFFSEi);lchMA_Incremenutp;
	"flap counuert"KR lfaEXls;
	((lfaEXls;&	~L(valFLAP_COUNGX  mo) |n"	   (((((lfaEXls;&	L(valFLAP_COUNGX  mo) >>
C_LIIIIIIL(valFLAP_COUNGXOFFSEi) + 1)hy,00ff)
C_LIII<<IL(valFLAP_COUNGXOFFSEi))al[x_e_WRort].FZE+1];
lfaE	PORT_HWLIIIIIIX_LASER_MASK;
	DP(NETIlfaFolfaEXlsmFolfaEXlsmal_ficProc;
		with GXgular p;
	"t bnializt 
	} "KRms ,
					   al12t bn("Settinarams->bp;
	DP(NETIF p = 0;
aramsvar
	Dvar
statu,
		lfaEXlink_;spEPROM_VENDOR_NAME_SIZE+1];
	char;
	/* Low ==> corresPx_EI bnializt 
	} Xliypedfw_resp;
	/* Low ==> corres(1)h33le1);
		%d, 33leflowctrl	_ASFP+
;BC CFZE+1];
33le	{
< 1);
	[0],iFZE+1];
33leflow_ctrl[0]resp;
	/* Low ==> corres(2)h33le1);
		%d, 33leflowctrl	_ASFP+
;BC CFZE+1];
33le	{
< 1);
	[1],iFZE+1];
33leflow_ctrl[1]resp;
	/* Low ==> corres33leadveflow_ctrl		bnx2x_clIZE+1];
33lefc_auto_adv)harvar
->aramsXlink_ 
	balevar
->al12aramsuME_S0;arvar
->aramsuME_S0;arvar
->ara< 1);
				eal[var
->duplepesPDUPLEX_FULtachvar
->flow_ctrl			BNX2X_FLOW_C_10XNONEal[var
->mac	tom;;
	MACXT_PHYNONEal[var
->al12flag_/= eal[var
->c= va_kr2IF_costry_cnt/= eal[FZE+1];
lramsflag_/= S_EX&=ITIALIZEDal_ficDristr;opens.NDU-BRB"filters	ontruct bnf:ufrx_filter(P(NETIF 1s;
 {
	/* chng_l	chaorunu(P(NETIF PROe)al[MA_C= valol p;
	"flap can be a_up)edt"KR lfaEXlink_ 
	{
	/* ch va_lfa(P(NETIs;

 ol tlfaEXlink_ 

		)t((va;
	/* Low ==> corresL;
	"Flap A_up)ancGPSn;rrogNessfw_resp[ *param			   a_up) aramsflap(P(NETIF var
)al[}

 ;
	/* Low ==> corresCannot a_up) p;
	"flap lfaEXli=bbnx2x_c
;BC CCCCClfaEXlink_s;
 {
	/* cannot a_up) aramsflap(P(NETIF var
, lfaEXlink_s;
r_ficDision

attenu
	}s ontruct bnbins_dis_rt].NDUAre
	  mo_INTERRUPT_ bnx0 +oPfp_ctrlFRCM*4,s	_      S(NDUA  mo_W_CF0_L(val[TATUS |n"		NDUA  mo_W_CF0_L(va10G |n"		NDUA  mo_SERDEF0_L(val[TATUS |n"		NDUA  mo_MI_INT)wertruct bnemac	t bn(P(NETIF var
)alchol tFZE+1];
ORT_FEA  case _flag_/O FEATURE_CONFIG_PFC	ENABLED)ar	var
->aramsXlink_ |
	L(val[TATUS_PFC	ENABLEDalchol tFZE+1];
numval1st

		)t((va;
	/* Low ==> corresNo;rx_EfouPUTfor t bnializt 
	} !!fw_resp[ *param-EINVALas }r	x2x_thesvar
(P(NETIF var
)alch;
	/* Low ==> corresNum of al1st	} board:	_ASFP+vFZE+1];
numval1swer[Ep_modulpZE+1];
loopback_mUTPd 	break;
LOOPBACKX_MAC:
 r{
	/* t bn_bmac	loopback(P(NETIF var
)al[SPREG_SFPeak;
LOOPBACKXEMAC:
 r{
	/* t bn_emac	loopback(P(NETIF var
)al[SPREG_SFPeak;
LOOPBACKXXMAC:
 r{
	/* t bn_xmac	loopback(P(NETIF var
)al[SPREG_SFPeak;
LOOPBACKXUMAC:
 r{
	/* t bn_umac	loopback(P(NETIF var
)al[SPREG_SFPeak;
LOOPBACKXX_CF:speak;
LOOPBACKXEXi_S_E:INK{
	/* t bn_xgxs loopback(P(NETIF var
)al[SPREG_SFP>ereadd:l[Sol t!CHIP_IS_E3ort))t((vahol tFZE+1];
Ep_modf	fg	==
SWITCHl727_10Gl
;B0;{
	/* xgxs deassert(P(NETIs;
 r firs
p rruct bnf:rdes_deassert(rt].FZE+1];
FRCMwer_ _P_[{
	/* aramst bnialize(P(NETIF var
)al		msleep(30)al[SP
	/* arams,
	_enion
(P(NETIwer[SPREG_SFP}
rP
	/* updt eEmng(P(NETIF var
->aramsXlink_wertruct bnupdt eEmng_eee(P(NETIF var
->eeeEXlink_s;
  *param0rams ,
					   bnx2xbnx2x("Settinarams->bp;
	DP(NETIF p = 0;
aramsvar
	Dvar
,s	_     u8 abeetxextval1statuEPROM_VENDOR_NAME_SIZE+1];
	char 8 Fl12indexFsFRCM =.FZE+1];
FRCM, clearElamodfindt
	bale;
	/* Low ==> corresReASEcoppdtoe p;
	"of aof  %dSFP+vFRCMwer_ficDision

attenu
	}s ontrvar
->aramsXlink_ 
	bale{
	/* chng_l	chaorunu(P(NETIF PROe)al[P
	/* updt eEmng(P(NETIF var
->aramsXlink_wer	var
->eeeEXlink_l&==~(SHMEM_EEE_LP_ADV_[TATUS_  mo |n"		      SHMEM_EEE_ACTIVE_BIT)as uct bnupdt eEmng_eee(P(NETIF var
->eeeEXlink_s;
 uct bnbins_dis_rt].NDUAre
	  mo_INTERRUPT_ bnx0 +oPRCM*4,s	_      S(NDUA  mo_W_CF0_L(val[TATUS |n"		NDUA  mo_W_CF0_L(va10G |n"		NDUA  mo_SERDEF0_L(val[TATUS |n"		NDUA  mo_MI_INT)wertrficActivt e nig drain "KR x_e_WRort].NDUAre
	EGRESS_DRAIN0DEADD +oPRCM*4, 1s;
r_ficDision

nig egNessPin	erfacGP*Setol t!CHIP_IS_E3ort))t((vax_e_WRort].NDUAre
	_MAC0_OUT	EN +oPRCM*4, 0)al[Sx_e_WRort].NDUAre
	EGRESS_EMAC0_OUT	EN +oPRCM*4, 0)al[}
l[Sol t!CHIP_IS_E3ort))t((vahuct bnf:ufbmac	rx(rt].FZE+1];
ehip_ide tRCMF 0)as  _ first((va	P
	/* f:ufxmac	rxtx(P(NETIF 0)al[Sp{
	/* f:ufumac	rxtx(P(NETIF 0)al[S}r_ficDision

emacP*Setol t!CHIP_IS_E3ort))l[Sx_e_WRort].NDUAre
	NDUAEMAC0_EN +oPRCM*4, 0)alar sleep_rang;(neee0, 2etd0);r_ficTaseS_Elabeet"isl catroln
d by GPIO 1DIMA_Hold it as var
Mlowve	ontr MA_Clear p;
	"ledt"KR uct bnf:ufmdio_emac	per_al1(rt].FZE+1])al[uct bnf:ufled(P(NETIF var
, L			 ADDLOFF, 0)alarol tabeetxextval1st((vafARp(Fl12index	;	_XGXS_E1e Fl12index	< aZE+1];
numval1s;s	_      Fl12index++le_eep0ol tFZE+1];
Fhy[Fl12index].bnx2xbnx2xle_eep0ruct bnf:ufaer_mmd(P(NETIF
W_C__odu&FZE+1];
Fhy[Fl12index])al[Sp;FZE+1];
Fhy[Fl12index].bnx2xbnx2x(INK,		&FZE+1];
Fhy[Fl12index],= 0;
	tZE+1])al[_ _P_[0ol tFZE+1];
Fhy[Fl12index].flag_/OINKKLI  FLAGS4REARM_LATCHl[IGNALl
;B0;clearElamodfindt
	1es	_Rd }lchol tclearElamodfindle_eepMA_Clear pamodoppdy, Set 
	} *Setp{
	/* REGrmElamodfsignal(rt].FRCMF 0)as  uct bnbins_dis_rt].NDUAre
	LATCHlBC_0 +oPRCM*4,s	__      S1I<<INDUALATCHlBC_ENABLE_MI_INT)SFP}
rol tFZE+1];
Fhy[INGXS_E].bnx2xbnx2xl
		Pfp_ctrlFhy[INGXS_E].bnx2xbnx2x(INK,&Pfp_ctrlFhy[INGXS_E],.FZE+1])alr_ficDision

nig oppNessPin	erfacGP*Setol t!CHIP_IS_E3ort))t((vaficReeet"BigMacP*Setax_e_WRort].GRCBASE_n = T_In = tx_eISTERS4RESEinREG_2XCLEARF= 0       (n = tx_eISTERS4RESEinREG_2XRSi__MAC0I<<IFRCM))as  x_e_WRort].NDUAre
	_MAC0_IN	EN +oPRCM*4, 0)al[Sx_e_WRort].NDUAre
	EMAC0_IN	EN +oPRCM*4, 0)al[R first((vakG_Rxmac		POR;
	(FZE+1];
FRCMw ?.GRCBASE_XMAC1 :.GRCBASE_XMAC0as  uct bnf:ufxumac	nig(P(NETIF 0, 0)al[Sut *nfig[port].n = tx_e_RESEinREG_2)/OINKLI  n = tx_eISTERS4RESEinREG_2XXMAC)s  ax_e_WRort].xmac		POR;+ XMACtx_e_CTRL,s	__      SXMACtC_10XRe
	SOFT4RESEi)SFP}
rvar
->aramsuME_S0;arvar
->al12flag_/= eal[ *param0rams,
					   bfaxbnx2x("Settinarams->bp;
	DP(NETIFs	__      Sp = 0;
aramsvar
	Dvar
statuEPROM_VENDOR_NAME_SIZE+1];
	charvar
->aramsuME_S0;arvar
->al12flag_/= eal[FZE+1];
lramsflag_/&==~S_EX&=ITIALIZEDal_ol t!IZE+1];
lfaE	POR)s   *para				   bnx2xbnx2x(P(NETIF var
, 1)alh/*
p *PActivt e NDUldrain po
toat duroppdtoisPtime aase>evicGPwon't send
p *Panytdoppdwhin
 it"islunion

tolabep	}se.ve	ontrx_e_WRort].NDUAre
	EGRESS_DRAIN0DEADD +oPfp_ctrlFRCM*4, 1s;
r_fi
p *PClok;
gracGfulinltoe gt e from _MAC
tolNDUlsuodutoat no half packets
p *Pare passed.ve	ontrol t!CHIP_IS_E3ort))l[Suct bnf:ufbmac	rx(rt].FZE+1];
ehip_ide tZE+1];
FRCM, 0)alarol tCHIP_IS_E3ort))t((vaP
	/* f:ufxmac	rxtx(P(NETIF 0)al[S{
	/* f:ufumac	rxtx(P(NETIF 0)al[}r_ficWaiG 10mslfARpaasepip

tolclean up
	fwssleep_rang;(neee0, 2etd0);r
 MA_Clean
toe NDU-BRB"use sttoe netvor	"filters	in a wayutoat vilT
p *Pnot cut a packet	in toe middle.ve	ontruct bnf:ufrx_filter(P(NETIF 0s;
r_fi
p *PRe-open
toe gt e between
toe _MAC
NPUTtoe NDU,;afterPstrifye sttoe
p *Pgt e ao mod BRB"isl lok;de omodrwise packets mayuarrist ao mod
p *Pfirmware befo GPdristr;had t bnializ
d St.cTasetarget"islao aehievd
p *Pmt bmum managemenutprotocol down time.ve	ontrol t!CHIP_IS_E3ort))l[Suct bnf:ufbmac	rx(rt].FZE+1];
ehip_ide tZE+1];
FRCM, 1)alarol tCHIP_IS_E3ort))t((vaP
	/* f:ufxmac	rxtx(P(NETIF 1)al[S{
	/* f:ufumac	rxtx(P(NETIF 1)al[}r_ficDision

NDUldrain "KR x_e_WRort].NDUAre
	EGRESS_DRAIN0DEADD +oPfp_ctrlFRCM*4, 0wer  *param0rams /****************************************************************************/</*;B0;Common  phy 
	}SG
	oo  */</****************************************************************************/<*params,
					   P073_common t bn_pl1("Setting SFP_NAM,
;BC_LI    uG_R") &
		PORs->th[],
;BC_LI    uG_R") &
2		PORs->th[],Ik_vFl12indexF
;BC_LI    uG_Rehip_iddtatuEPROM_VENDORval1,Fhy[ bnxDEAX];ar" = 0;
	u16 val1, rx__blk[ bnxDEAX];aru16 val;spE8sFRCM =.0;spE8sFRCM_ofs->th =.0;sp venEpapsval,nEpapsostrrideer[Epapsval;
	nfig[port]..NDUAre
	 bnxDSWAP)er[Epapsostrride;
	nfig[port]..NDUAre
	STRAP_OVERRIDEwer	PRCM ^==(epapsval;&&nEpapsostrride)al[uct bnextval12;
	bnx2x(rt].FRCMmal_ficPART1 -cReeet"bomo al1st"KR fARp(FRCM =. bnxDEAX -c1e FRCM >=. bnxD0e FRCM--)t((va venE) &
		PORe ") &
2		POR;
[SMA_In E2,esamP	rx_Eis"use stfARpFRCM0 of modutvo ->thsT*Setpol tCHIP_IS_E1xort))e_eep0)) &
		PORT=R") &
		PORs->th[0]harp0)) &
2		PORT=R") &
2		PORs->th[0]harp0FRCM_ofs->th =.FRCMas  _ first((va	)) &
		PORT=R") &
		PORs->th[FRCM	harp0)) &
2		PORT=R") &
2		PORs->th[FRCM	harp0FRCM_ofs->th =.0;spa}

 rac- xtracp aaseexte l1,address fARpaasepof  *Setpol t			   aopult eval1(rt].Fl12indexFs)) &
		PORe ") &
2		POR,= 0;
      SIRCM_ofs->th]n&Phy[F*/
	r !=INKLI  0)t((vaa;
	/* Low ==> corresaopult eval1,fain
dfw_resp[S_*param-EINVALas  _P__ficDision

attenu
	}s ontr uct bnbins_dis_rt].NDUAre
	  mo_INTERRUPT_ bnx0 +
0;
      SIRCM_ofs->th*4,s	__      S(NDUA  mo_W_CF0_L(val[TATUS |n"			NDUA  mo_W_CF0_L(va10G |n"			NDUA  mo_SERDEF0_L(val[TATUS |n"			NDUA  mo_MI_INT)wertr_ficNe
		to take modurx_Eout of lowSIRwGP mUTP"in ordereep *Pto wri e ao access ins GXgisters
ep *Setp{
	/* f:ufgpioort].n = tx_eISTERS4GPIO_2,s	__      Sn = tx_eISTERS4GPIO_OUTPUT_HIGH,
0;
      SIRCMwertr_ficReeet"modurx_E*Setp{
	/* cl45_wri eort]n&Phy[F*/
	,= 0;
 bnx2xPMAXDEVAD,= 0;
 bnx2xPMAXx_e_CTRL,s	__	 1<<15)al[}lchMA_Adde>elayuof 150mslafterPabeet"*Setmsleep(150)alchol tFhy[ bnxD0].addr;y,001te((vapTW_blk[ bnxD0]	; &tFhy[ bnxD1]respapTW_blk[ bnxD1]	; &tFhy[ bnxD0]resp_ first((vapTW_blk[ bnxD0]	; &tFhy[ bnxD0]respapTW_blk[ bnxD1]	; &tFhy[ bnxD1])al[}lchMA_PART2 -cDownload firmware ao bomo al1st"KR fARp(FRCM =. bnxDEAX -c1e FRCM >=. bnxD0e FRCM--)t((vaol tCHIP_IS_E1xort))arp0FRCM_ofs->th =.FRCMas  firssp rpRCM_ofs->th =.0;s(va;
	/* Low ==> corresLoade stspirom fARpFl1,address bbnx2x_c
;BC   Fl12blk[F*/
	UTaddr)al[Sut *			   P073_P727_externalvrom_boot(rt].Fl12blk[F*/
	LHW_C__oduuuuupRCM_ofs->thamsp r_*param-EINVALastr_ficO inleet"biG 10t
	1 (TxSIRwGP down)E*Setp{
	/* cl45_GXS_(rt].Fl12blk[F*/
	LHW_C_bnx2xPMAXDEVAD,= 0;
bnx2xPMAXx_e_TX_ bWER_DOWN]n&val)all[hMA_PhPOR1uof TX_ bWER_DOWNPabeet"*Setp{
	/* cl45_wri eort]nFl12blk[F*/
	LHW_C_ bnx2xPMAXDEVAD,= 0;
 bnx2xPMAXx_e_TX_ bWER_DOWN]= 0;
 (val;| 1<<10))al[}lchMA_Toggn

Transmitter: PRwGP down
NPUTtoen up	with 600msl>elay
p *Pbetweenve	ontrmsleep(6d0);r
 MA_PART3 -ccomple e TX_ bWER_DOWNPproc;ssFsNPUTASEEGPIO2 back to lowt"KR fARp(FRCM =. bnxDEAX -c1e FRCM >=. bnxD0e FRCM--)t((vaMA_PhPOR2uof  bWER_DOWN4RESEi"*SetpficRelearstbiG 10t(RelearstTxSIRwGP down)E*Setp{
	/* cl45_GXS_(rt].Fl12blk[F*/
	LHW_C_bnx2xPMAXDEVAD,= 0;
bnx2xPMAXx_e_TX_ bWER_DOWN]n&val)all[h{
	/* cl45_wri eort]nFl12blk[F*/
	LHW_C_bnx2xPMAXDEVAD,= 0;
bnx2xPMAXx_e_TX_ bWER_DOWN]n(val;& (~(1<<10))))as  ssleep_rang;(n5ee0, 3etd0);r
 pficRead mUTify wri e aoduSPI-ROMPstrs
	}MEelecp GXgisterE*Setp{
	/* cl45_GXS_(rt].Fl12blk[F*/
	LHW_C_bnx2xPMAXDEVAD,= 0;
bnx2xPMAXx_e_EDC_FFEDEAIN]n&val)altp{
	/* cl45_wri eort]nFl12blk[F*/
	LHW_C_ bnx2xPMAXDEVAD,= 0;
 bnx2xPMAXx_e_EDC_FFEDEAIN]n(val;| (1<<12))wertr_ficASEEGPIO2 back to LOW *Setp{
	/* f:ufgpioort].n = tx_eISTERS4GPIO_2,s	__      Sn = tx_eISTERS4GPIO_OUTPUT_LOW].FRCMmal_}r  *param0rams*params,
					   P726_common t bn_pl1("Setting SFP_NAM,
;BC_LI    uG_R") &
		PORs->th[],
;BC_LI    uG_R") &
2		PORs->th[],Ik_vFl12indexF
;BC_LI    uG_Rehip_iddtatuuG_Rval;spE8sFRCM;ar" = 0;
	u16 val1,px_er_ac-Ussepof 1Pbecause of modu*paramsFRCM-Epap *Setac- nion
 tasemUTulePde ey 
	} in	errupt ontrval;
	nfig[port].n = tx_e_GPIO_EVENT	ENwer	val;|
	((1<<n = tx_eISTERS4GPIO_3)|n"	(1<<(n = tx_eISTERS4GPIO_3T_In = tx_eISTERS4GPIO_ bnxDSHIFTr))al[x_e_WRort].n = tx_e_GPIO_EVENT	EN, val)alchuct bnextval12;
	bnx2x(rt].0wer ssleep_rang;(5ee0, 1etd0);r_fARp(FRCM =.0e FRCM <. bnxDEAXe FRCM++le_eep venE) &
		PORe ") &
2		POR;

[SMA_In E2,esamP	rx_Eis"use stfARpFRCM0 of modutvo ->thsT*Setpol tCHIP_IS_E1xort))e_eep0)) &
		PORT=R") &
		PORs->th[0]harp0)) &
2		PORT=R") &
2		PORs->th[0]harp_ first((va	)) &
		PORT=R") &
		PORs->th[FRCM	harp0)) &
2		PORT=R") &
2		PORs->th[FRCM	harp}
 rac- xtracp aaseexte l1,address fARpaasepof  *Setpol t			   aopult eval1(rt].Fl12indexFs)) &
		PORe ") &
2		POR,= 0;
      SIRCM]n&Phyr !=INKLI  0)t((vaa;
	/* Low ==> corresaopult e al1,fain
dfw_resp[S_*param-EINVALas  _Ptr_ficReeet"al1*Setp{
	/* cl45_wri eort]n&PhyLHW_C_ bnx2xPMAXDEVAD, bnx2xPMAXx_e_GEN_CTRL, bbtd01)ala
vaMA_S0G readd;mUTulePde ey 
		LED 	} *Setp{
	/* f:ufgpioort].n = tx_eISTERS4GPIO_0,s	__      Sn = tx_eISTERS4GPIO_HIGH,
0;
      SIRCMwer _Ptr *param0rams*params_up) {
	/* getxextval12abeetxgpioo"Setting SFP_NAM, sG_R") &
		POR,= 0;
	Ik_v*ioxgpio,Ik_v*ioxIRCMwtattu G_Ral12gpio2abeet 
	nfig[port].)) &
		PORT_= 0;
	IeX_LASER_MASK;
	DP(NETIF_MSG_LHW_C_date media type for non- bnxD0].>ereadd_cfg)wer[Ep_modulpl12gpio2abeets		break;
 bnx2x_8727__XGXS_EXGPIO_RSi_GPIO0_P0:l[S ioxgpio =.0;spa*ioxIRCM =.0;spaPREG_SFPeak;
 bnx2x_8727__XGXS_EXGPIO_RSi_GPIO1_P0:l[S ioxgpio =.1;spa*ioxIRCM =.0;spaPREG_SFPeak;
 bnx2x_8727__XGXS_EXGPIO_RSi_GPIO2_P0:l[S ioxgpio =.2;spa*ioxIRCM =.0;spaPREG_SFPeak;
 bnx2x_8727__XGXS_EXGPIO_RSi_GPIO3_P0:l[S ioxgpio =.3;spa*ioxIRCM =.0;spaPREG_SFPeak;
 bnx2x_8727__XGXS_EXGPIO_RSi_GPIO0_P1:l[S ioxgpio =.0;spa*ioxIRCM =.1al[SPREG_SFPeak;
 bnx2x_8727__XGXS_EXGPIO_RSi_GPIO1_P1:l[S ioxgpio =.1;spa*ioxIRCM =.1al[SPREG_SFPeak;
 bnx2x_8727__XGXS_EXGPIO_RSi_GPIO2_P1:l[S ioxgpio =.2;spa*ioxIRCM =.1al[SPREG_SFPeak;
 bnx2x_8727__XGXS_EXGPIO_RSi_GPIO3_P1:l[S ioxgpio =.3;spa*ioxIRCM =.1al[SPREG_SFP>ereadd:l[SficDon't ostrride;aaseioxgpio NPUTioxIRCM *Setp{REG_SFP}
}

*params,
					   P727_common t bn_pl1("Setting SFP_NAM,
;BC_LI    uG_R") &
		PORs->th[],
;BC_LI    uG_R") &
2		PORs->th[],Ik_vFl12indexF
;BC_LI    uG_Rehip_iddtatus_vFRCMFIabeetxgpio;sp venEpapsval,nEpapsostrrideer[EPROM_VENDORval1,Fhy[ bnxDEAX];ar" = 0;
	u16 val1, rx__blk[ bnxDEAX];arE8sFRCM_ofs->ther[Epapsval;
	nfig[port].NDUAre
	 bnxDSWAP)er[Epapsostrride;
	nfig[port].NDUAre
	STRAP_OVERRIDEwertr *eetxgpio;
	M = tx_eISTERS4GPIO_1er	PRCM =.1alr_ficRetrievd;aaseabeet gpio/PRCM whiodu catrol;aaseabeet.ve	o Dereadd;is"GPIO1,
 bnx1ve	ontruct bngetxextval12abeetxgpioort].)) &
		PORs->th[0]F
;BC_LI   (k_v*)&abeetxgpio, (k_v*)&IRCMwertrMA_Calcult e aasePRCM 	PORd 	}MPRCM Epap *SetPRCM ^==(epapsval;&&nEpapsostrride)alchMA_InbniatseS_Elabeetontruct bnf:ufgpioort].abeetxgpio, n = tx_eISTERS4GPIO_OUTPUT_LOW]
;
      SIRCMwer ssleep_rang;(neee, 2etd)al[uct bnf:ufgpioort].abeetxgpio, n = tx_eISTERS4GPIO_OUTPUT_HIGH,
0;      SIRCMwertrssleep_rang;(5ee0, 1etd0);rl_ficPART1 -cReeet"bomo al1st"KR fARp(FRCM =. bnxDEAX -c1e FRCM >=. bnxD0e FRCM--)t((va venE) &
		PORe ") &
2		POR;

[SMA_In E2,esamP	rx_Eis"use stfARpFRCM0 of modutvo ->thsT*Setpol tCHIP_IS_E1xort))e_eep0)) &
		PORT=R") &
		PORs->th[0]harp0)) &
2		PORT=R") &
2		PORs->th[0]harp0FRCM_ofs->th =.FRCMas  _ first((va	)) &
		PORT=R") &
		PORs->th[FRCM	harp0)) &
2		PORT=R") &
2		PORs->th[FRCM	harp0FRCM_ofs->th =.0;spa}

 rac- xtracp aaseexte l1,address fARpaasepof  *Setpol t			   aopult eval1(rt].Fl12indexFs)) &
		PORe ") &
2		POR,= 0;
      SIRCM_ofs->th]n&Phy[F*/
	r !=INK;
      S0)t((vaa;
	/* Low ==> corresaopult e al1,fain
dfw_resp[S_*param-EINVALas  _P rac-dision

attenu
	}s ontr uct bnbins_dis_rt].NDUAre
	  mo_INTERRUPT_ bnx0 +
0;
      SIRCM_ofs->th*4,s	__      S(NDUA  mo_W_CF0_L(val[TATUS |n"			NDUA  mo_W_CF0_L(va10G |n"			NDUA  mo_SERDEF0_L(val[TATUS |n"			NDUA  mo_MI_INT)wert
r_ficReeet"modurx_E*Setp{
	/* cl45_wri eort]n&Phy[F*/
	,= 0;
 bnx2xPMAXDEVAD, bnx2xPMAXx_e_CTRL, 1<<15)al[}lchMA_Adde>elayuof 150mslafterPabeet"*Setmsleep(150)alhol tFhy[ bnxD0].addr;y,001te((vapTW_blk[ bnxD0]	; &tFhy[ bnxD1]respapTW_blk[ bnxD1]	; &tFhy[ bnxD0]resp_ first((vapTW_blk[ bnxD0]	; &tFhy[ bnxD0]respapTW_blk[ bnxD1]	; &tFhy[ bnxD1])al[}lhMA_PART2 -cDownload firmware ao bomo al1st"KR fARp(FRCM =. bnxDEAX -c1e FRCM >=. bnxD0e FRCM--)t((vaol tCHIP_IS_E1xort))arp0FRCM_ofs->th =.FRCMas  firssp rpRCM_ofs->th =.0;sva;
	/* Low ==> corresLoade stspirom fARpFl1,address bbnx2x_c
;BC   Fl12blk[F*/
	UTaddr)al[Sut *			   P073_P727_externalvrom_boot(rt].Fl12blk[F*/
	LHW_C__oduuuuupRCM_ofs->thamsp r_*param-EINVALas__ficDision

S_EltransmitterEoutput"*Setp{
	/* cl45_wri eort]nFl12blk[F*/
	LHW_C_ bnx2xPMAXDEVAD,= 0;
 bnx2xPMAXx_e_TX_DISABLE, 1)alar}r[ *param0rams *params,
					   P4833_common t bn_pl1("Setting SFP_NAM,
;BC_va venE) &
		PORs->th[],
;BC_va venE) &
2		PORs->th[],
;BC_va _vFl12indexF
;BC_va venehip_iddtatuu8.abeetxgpios;tr *eetxgpio_ 
	{
	/* P4833_getx *eetxgpio_ort].)) &
		PORs->th,nehip_iddal[uct bnf:ufmaddfgpioort].abeetxgpios, n = tx_eISTERS4GPIO_OUTPUT_LOWwer s>elay(nedal[uct bnf:ufmaddfgpioort].abeetxgpios, n = tx_eISTERS4GPIO_OUTPUT_HIGHresp;
	/* Low ==> corresP4833Pabeet"pulse o}MPin values bbnx2x_c
;Babeetxgpioswer  *param0rams *params,
					   extval12ormmon t bno"Setting SFP_NAM, sG_R") &
		PORs->th[],
;BC_LI   uG_R") &
2		PORs->th[],Ik_vFl12indexF
;BC_LI   uG_Rextval12aom;,  venehip_iddtatu,
		rc =.0;s(vEp_modulextval12aom;s		break;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP073:l[Src =.			   P073_common t bn_pl1(rt].)) &
		PORs->th,
;BC_va") &
2		PORs->th,
;BC_vaFl12indexFsehip_iddal[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP722:FPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP727:FPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP727_NOC:l[Src =.			   P727_common t bn_pl1(rt].)) &
		PORs->th,
;BC_va") &
2		PORs->th,
;BC_vaFl12indexFsehip_iddal[SPREG_SFFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP726:l[SficGPIO1laffects bomo a*/
IF po modre's ne
		to pull
ep *Pit fARpse slsepof  aloneeep */l[Src =.			   P726_common t bn_pl1(rt].)) &
		PORs->th,
;BC_va") &
2		PORs->th,
;BC_vaFl12indexFsehip_iddal[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4833:FPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4834:FPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP4858:l[SficGPIO3's are bnx2;de NPUTAo bomo ne
		to be aoggn
deep *Pto obtain 33luir
		2us"pulse.eep */l[Src =.			   P4833_common t bn_pl1(rt].)) &
		PORs->th,
;BC_va") &
2		PORs->th,
;BC_vaFl12indexFsehip_iddal[SPREG_SFPeak;
 bnx2x_8727_W_CFG_XGXS_EXT_PHYFAILURE:l[Src =.-EINVALas__PREG_SFP>ereadd:l[S;
	/* Low ==> corr
oC_LI "extval1	bbnxucommon t bnPnot 33luir
	2x_c
;BC   extval12aom;sas__PREG_SFP}lchol trcmsp netdatetrr(rt->dat,I "Waroe s:
S_ElwasTnot t bnializ
d,"= 0;oduuuuu" Pof  %dSFP+
;BC 0wer  *paramrcrams ,
					   common t bn_pl1("Setting SFP_NAM, sG_R") &
		PORs->th[],
;BC  uG_R") &
2		PORs->th[],Ikvenehip_iddtatu,
		rc =.0;su G_Ral12ver,Rval;spk_vFl12index =.0;su G_Rextval12aom;, extval12or nonerR uct bnf:ufmdio_clk(rt].ehip_ide GRCBASE_EMAC0dal[uct bnf:ufmdio_clk(rt].ehip_ide GRCBASE_EMAC1resp;
	/* Low ==> corresB_MSn;common rx_Ei bnfw_respol tCHIP_IS_E3ort))t((vaac- nion
 EPIO */l[Sval;
	nfig[port].n = tx_e_GEN_PURP2x_G)al[Sx_e_WRort].n = tx_e_GEN_PURP2x_G,Rval;| 1)al[}lhMA_C= valol common t bnPwasTalGXS_yRdo
e *SetPl12ver 
	nfig[port].)) &
		PORs->th[0]T_HW_CLX_LASER_MASK;
	DP(NETIF_MSG_LHW_C_uupRCM_mb[ bnxD0].extval12fw_strs
	}))alhol tFhy_str)t((va;
	/* Low ==> corresNotRdoe stcommon t bne Fl1PstrEis"bbnx2x_c
;BC       Fl12str);
	  *param0ra[}lchMA_Read aaseextval12aom; fARparbinraryupRCM(0)t"KR fARp(Fl12index	;	_XGXS_E1e Fl12index	< MAXXS_ES;
oduuuuuFl12index++le_eepextval12or non =.			   getxextval12or non(AM,
;BC_vaodu)) &
		PORs->th[0]F
;BC__C_uupl12indexFs0)as  fxtval12aom; = W_CFG_XGXS_EXT_PH(extval12or non);
	  c;|
				   extval12ormmon t bnort].)) &
		PORs->th,
;BC_va") &
2		PORs->th,
;BC_vaFl12indexFsextval12aom;,
;BC_vaehip_iddal[}r  *paramrcrams *params_up) {
	/* ch va_ostr_curr("Settinarams->bp;
	DP(NETIFHW_C_uup = 0;
aramsvar
	Dvar
statuEPROM_VENDOR_NAME_SIZE+1];
	charkvenefg_Pin;spk_vFRCM =.FZE+1];
FRCM;su G_Rainsval;sFPefg_Pin;
	(nfig[port].FZE+1];
)) &
		PORT_HW_CIeX_LASER_MASK;
	DP(NETIF_MSG_LHW_C       date media type for non-P*/
	.e3_cmn_Pin_cfg1))/OINKLI  bnx2x_8727__3_OVER_CURRENGX  mo) >>
C_ bnx2x_8727__3_OVER_CURRENGXSHIFTalchMA_Igno GPc= valol no externalOinput"PIN availion
 ontrol t			   getxefg_Pin(rt].efg_Pin]n&Pinsvalte!
	b)
   *paraall_ol t!Iinsvalte((vaol t(var
->al12flag_/& S_EXOVER_CURRENGXFLAG)	;
		)t((vaanetdatetrr(rt->dat,I"Error:  PRwGP readd;on Pof  %d has"= 0;o	uuuu" been
de ey 
		NPUTtoe IRwGP to "= 0;o	uuuu"toat SFP+;mUTulePhas been
remostd"= 0;o	uuuu" to prevenutfainure of moducard."= 0;o	uuuu" Plearstremost aoduSFP+;mUTulePNPU"= 0;o	uuuu" abeliyp modu*ystem
tolcleardtois"= 0;o	uuuu" error.2x_c
;BC FZE+1];
FRCMwer_ rvar
->al12flag_/|= S_EXOVER_CURRENGXFLAGer_ r			   warpcman IRwGP_mUTule(a(NETIF 0)al[S}r_R firss  var
->al12flag_/&==~S_EXOVER_CURRENGXFLAGerms /*cRetaras"blol no chang; occurr
		sincGPpastPc= va; 1 omodrwise. */<*paramsk_v			   analyze bnx2xerror("Settinarams->bp;
	DP(NETIF_MSG
	oo p = 0;
aramsvar
	Dvar
, uG_R"link_F_MSG
	oo  G_Ral12flag, uG_Rlramsflag,Ik_vnotif1statuEPROM_VENDOR_NAME_SIZE+1];
	charMA_Compare new value	with previous value	
	fws8"led_mUTP;su G_RoldsXlink_ 
	(var
->al12flag_/& al12flagw ?.1 :.0alarol t(Xlink_ ^RoldsXlink_)	;
		)
	  *param0rachMA_If values differE*SetEp_modulpl12flagw 	break;
 _EXHALFLOPEN_CON	XFLAG:l[S;
	/* Low ==> corr "AnalyzecRemote Feaddfw_resp[PREG_SFPeak;
 _EXSFP_TXYFAULGXFLAG:l[S;
	/* Low ==> corr "AnalyzecTX Feaddfw_resp[PREG_SFP>ereadd:l[S;
	/* Low ==> corr "AnalyzecUNKNOWNfw_resp}sp;
	/* Low ==> corresL;
	"chang;d:[%x nx]->nx2x_clvar
->aramsuM,
_LIIoldsXlink_, Xlink_s;
r_ficDoTnot touodutoGPp;
	"in eak;
al1sicalOp;
	"down
ontrol t(var
->al12flag_/& S_EXS_ESICAL_L(valFLAG)	;
		)
	  *param1alr_fica.-Updt e P(NET->aramsXlink_ accmade sly
p *Pb.-Updt e aramsvar
->aramsuMve	ontrol tXlink_)	{
r	var
->aramsXlink_ O_P~L(val[TATUS_L(valUP;
r	var
->aramsXlink_ |= lramsflag;
r	var
->aramsuME_S0;arrvar
->al12flag_/|= al12flag;

[SMA_activt e nig drain "KR rx_e_WRort].NDUAre
	EGRESS_DRAIN0DEADD +oPfp_ctrlFRCM*4, 1s;
vaMA_S0G LED mUTP"to off	sincGPtaseS_Eldoesn't know about tasseeep *Perrors
ep *Setpled_mUTPE_SL			 ADDLOFFesp_ first((vavar
->aramsXlink_ |
	L(val[TATUS_L(valUP;
r	var
->aramsXlink_ O_P~lramsflag;
r	var
->aramsuME_S1;arrvar
->al12flag_/O_P~al12flag;
tpled_mUTPE_SL			 ADDLOPER;

[SMA_Clear nig drain "KR rx_e_WRort].NDUAre
	EGRESS_DRAIN0DEADD +oPfp_ctrlFRCM*4, 0)al[}r_uct bnfync_aram(P(NETIF var
)al	ac-Updt e taseLED accmade s ao mod p;
	"Xline	ontruct bnf:ufled(P(NETIF var
, led_mUTP,	}
				neee0)alr_ficUpdt e aram Xlink_	in toe shar
		NETory	ontruct bnupdt eEmng(P(NETIF var
->aramsXlink_wertrMA_C. TriggerEGeneralOAttenu
	} ontrvar
->periodic_ lag_/|= SERIODIC_FLAGS4L(valEVENTalhol tnotif1st r			   notif1_l	chaohang;dort);Ptr *param1rams /******************************************************************************
o Descripu
	}:
*	ToisP phy 
	}Pc= vas fARphalf open
		or ney 
	}Pc=ang; y, Set 
	}.
*	Woen suoduchang; occurs, it calls toe 			   analyze bnx2xerror
*	tolc= valol Remote FeaddEis"eet"ARpcleared. Recepu
	} of remote readd
*	Xlink_	messagP	in toe MAC Sn Set esutoat toe Ieer's MAC has de ey 
	
*	a readd, fARpexample, due ao bREG_	in toe TX side;of fiber.
*
******************************************************************************/<*params,
					   ch va_half_openfor n("Settinarams->bp;
	DP(NETIF_MSG
	oo  op = 0;
aramsvar
	Dvar
F_MSG
LI    u_vnotif1statuEPROM_VENDOR_NAME_SIZE+1];
	charuG_RlsssXlink_ 
	baleuG_Rmac		POR;chMA_In eak;
aram Xlink_	is
al1sically up	@ 10G do
ontrol t((var
->al12flag_/& S_EXS_ESICAL_L(valFLAG)	;
		) ||n"   S(nfig[port].NDUAre
	EGRESS_EMAC0_ bnx +oPfp_ctrlFRCM*4)))
	  *param0rachol tCHIP_IS_E3ort) &&n"   S(nfig[port].n = tx_e_RESEinREG_2)/OIN     S(n = tx_eISTERS4RESEinREG_2XXMAC)))t((vaac-C= valE3 XMAC *SetpficNo e taautp;
	"1);
		cannot be queri
		odre,	sincGPit mayubeeep *Pzerodwhin
 p;
	"isldown._In eak;
UMAC Ss_active,	LSS vilT
pIMA_pimply not be eet
ep *Setpmac		POR;
	(FZE+1];
FRCMw ?.GRCBASE_XMAC1 :.GRCBASE_XMAC0as
[SMA_Clear sramktbiGst(Reluir
s rise stedge) "KR rx_e_WRort].mac		POR;+ XMACtx_e_CLEAR_RX_LSSl[TATUS, 0)al[Sx_e_WRort].mac		POR;+ XMACtx_e_CLEAR_RX_LSSl[TATUS,
__      SXMACtCLEAR_RX_LSSl[TATUStx_e_CLEAR_LOCAL_FAULGX[TATUS |n"	      SXMACtCLEAR_RX_LSSl[TATUStx_e_CLEAR_REMOTE_FAULGX[TATUS)al[Sut *nfig[port].mac		POR;+ XMACtx_e_RX_LSSl[TATUSamsp rlsssXlink_ 
	1all[h{
	/* analyze bnx2xerror(P(NETIF var
, lsssXlink_,= 0;
	I _EXHALFLOPEN_CON	XFLAG,= 0;
	IL(val[TATUS_NONE,vnotif1sesp_ firstut *nfig[port].n = tx_e_RESEinREG_2)/OINKLI (n = tx_eISTERS4RESEinREG_2XRSi__MAC0I<<IFfp_ctrlFRCM))t((vaac-C= valE1X / E2 _MAC
*Setp G_RlsssXlink_IF_M;etp G_Rwb_dt a[2	harpmac		POR;
	Ffp_ctrlFRCM ?.NDUAre
	INGRESS__MAC1_MEM :INK	NDUAre
	INGRESS__MAC0_MEM;
vaMA__Read BIGMACtx_eISTER_RX_LSSl[TATUST*Setpol tCHIP_IS_E2ort))arp0lsssXlink_IF_M			BIGMAC2tx_eISTER_RX_LSSl[TATas  firssp rlsssXlink_IF_M			BIGMACtx_eISTER_RX_LSSl[TATUS;

 rx_e_RD_DMAEort].mac		POR;+ lsssXlink_IF_M,Rwb_dt a, 2)al[SlsssXlink_ 
	(wb_dt a[0] > 0s;
r_h{
	/* analyze bnx2xerror(P(NETIF var
, lsssXlink_,= 0;
	I _EXHALFLOPEN_CON	XFLAG,= 0;
	IL(val[TATUS_NONE,vnotif1sesp_tr *param0rams*params_up) {
	/* ffp_tx_readd_de ey 
	}(" = 0;
	u16 val1, rx_F_MSG
	op = 0;
arams->bp;
	DP(NETIF_MSG
	op = 0;
aramsvar
	Dvar
statuEPROM_VENDOR_NAME_SIZE+1];
	charkvenefg_Pin, value	
	baleu8"led_chang;FsFRCM =.FZE+1];
FRCMertrMA_Get"ToduSFP+;TXYFeaddE catrolne stPin;([eg]pio) "KR efg_Pin;
	(nfig[port].FZE+1];
)) &
		PORT_eX_LASER_MASK;
	DP(NETIF_MSG_LHW_C  date media type for non-P*/
	.e3_cmn_Pin_cfg))/OINKLI  bnx2x_8727__3_TXYFAULGX  mo) >>
C_LI bnx2x_8727__3_TXYFAULGXSHIFTalchol t			   getxefg_Pin(rt].efg_Pin]n&value))t((va;
	/* Low ==> corresFain
d
tolabad Pin;0x%02x2x_clefg_Pin);
	  *parara[}lchled_chang; =.			   analyze bnx2xerror(P(NETIF var
, valueF_MSG
	oo   
 _EXSFP_TXYFAULGXFLAGF_MSG
	oo   
L(val[TATUS_SFP_TXYFAULG, 1)alarol tled_chang;)t((vaac-C=ang; TXYFeaddEl;de s0G p;
	"Xlink_ fARpfurmodr fyncs *Setp _vled_mUTP;s
vaol tvar
->al12flag_/& S_EXSFP_TXYFAULGXFLAG)t((vaaled_mUTPE_Sn = tx_eISTERS4GPIO_HIGHer_ rvar
->aramsXlink_ |
	L(val[TATUS_SFP_TXYFAULGas  _ first((va	led_mUTPE_Sn = tx_eISTERS4GPIO_LOWer_ rvar
->aramsXlink_ O_P~L(val[TATUS_SFP_TXYFAULGas  _

[SMA_If;mUTulePislunipprov;de n
d
should be on
regardless *Setpol t!*Fl1UTflag_/O=FLAGS4SFP_NOT_APPROVED))t((vaa;
	/* Low ==> corresC=ang; TXYFeaddELED: ->nx2x_cHW_C   led_mUTP)al[Sp{
	/* f:ufe3_mUTule_readd_led(P(NETIF led_mUTP)al[S}FP}
}
*params_up) {
	/* kr2IF_costry("Settinarams->bp;
	DP(NETIFs	__      Sp = 0;
aramsvar
	Dvar
Fs	__      Sp = 0;
	u16 val1, rx_statuEPROM_VENDOR_NAME_SIZE+1];
	char;
	/* Low ==> corresKR2 F_costryfw_resp			   warpcman enion
_AN_KR2(al1claZE+1], var
)al				   warpcman abeliyp_AN_KR(al1claZE+1])alms *params_up) {
	/* ch va_kr2Iwa("Settinarams->bp;
	DP(NETIFs	__      Sp = 0;
aramsvar
	Dvar
Fs	__      Sp = 0;
	u16 val1, rx_statuEPROM_VENDOR_NAME_SIZE+1];
	charu16 	PORs->g;Fsnextva>g;Fsnot_kr2I>evicGF lanR;ch,
		sigde alr_ficOncGPKR2 wasTdision
de waiG 5 s0or ds befo GPch vae stKR2 F_costry
p *PSincGPsomGPsp_modesutend
tolabt bnPtoe ANPproc;ss	NPUTcleardtod
p *Ptoe adstrtiORd BP/NPlafterP~2 s0or ds cause sttoe KR2 to be dision
d
p *Pand F_costr
		Nany timesve	ontrol tvar
->c= va_kr2IF_costry_cnt/>		)t((vavar
->c= va_kr2IF_costry_cnt--;
	  *parara[}lchsigde  =.			   warpcman getxsigde (al1claZE+1])al[ol t!sigde )t((vaol t!lpZE+1];
lramsattrnfync;&	L(valATTR_SYNC_KR2_ENABLE))t((vahuct bnkr2IF_costry(P(NETIF var
, rx_s;(vaa;
	/* Low ==> corresNo	sigde fw_resp[}
	  *parara[}lchlanR =.			   getxwarpcman lanR(al1claZE+1])al[CL22_WRXOVER_CL45ort]nFl1, bnx2xre
	_AvalAER_BLOCrr
oC_LIbnx2xAER_BLOCrxAER_re
F lanR)al				   cl45_GXS_(rt].Fl1, bnx2xAN_DEVAD,= 0;bnx2xAN_re
	LP	AUTO_Ne
F &	PORs->g;)al				   cl45_GXS_(rt].Fl1, bnx2xAN_DEVAD,= 0;bnx2xAN_re
	LP	AUTO_Ne
2F &nextva>g;dal[uct bnf:ufaer_mmd(P(NETIF rx_s;(trMA_CL73 has not begun yet"*Setol t	PORs->g;	;
		)t((vaol t!lpZE+1];
lramsattrnfync;&	L(valATTR_SYNC_KR2_ENABLE))t((vahuct bnkr2IF_costry(P(NETIF var
, rx_s;(vaa;
	/* Low ==> corresNo	BPfw_resp[}
	  *parara[}lchMA_In eak;
NPlbit"islnot set	in toe Bak;P>g;Fsor tdEis"eet,DIMA_bu		s inlKX Ss_adstrtiORd, daclare aoisPp;
	"FZEtnerPas non-KR2DIMA_>evicG.ve	ontrnot_kr2I>evicG;
	((t	PORs->g;	y,008ee0)	;
		) ||n"C_LI((t	PORs->g;	y,008ee0)	&OINKKLI  ((nextva>g;	y,00e0)	;
		x20))))aschMA_In eak;
KR2 isTalGXS_yRdision
de c= valol we ne
		to re-enion

it"*Setol t!lpZE+1];
lramsattrnfync;&	L(valATTR_SYNC_KR2_ENABLE))t((vaol t!not_kr2I>evicG)t((vaa;
	/* Low ==> corresBP=bbnx,
NP=bbnx2x_c 	PORs->g;FINKKLI nextva>g;dal[ahuct bnkr2IF_costry(P(NETIF var
, rx_s;(va}
	  *parara[}lhMA_KR2 isTenion
de bu		not KR2 >evicG;*Setol tnot_kr2I>evicG)t((vaficDision

KR2 	} both lanRs *Setp;
	/* Low ==> corresBP=bbnx,
NP=bbnx2x_c 	PORs->g;F nextva>g;dal[auct bndision
_kr2(P(NETIF var
, rx_s;(vaficReeliyp ANP	} leade s lanR *Setp{
	/* warpcman abeliyp_AN_KR(al1claZE+1])al	  *parara[}l}lc_up) {
	/* period_ phy("Settinarams->bp;
	DP(NETIF p = 0;
aramsvar
	Dvar
statuu16 pl12idx;tuEPROM_VENDOR_NAME_SIZE+1];
	charfARp(Fl12idx	;	INGXS_E;lFl12idx	< MAXXS_ES; Fl12idx++le_eepol tFZE+1];
Fhy[Fl12idx].flag_/O=FLAGS4TXYERROR_CHECK)t((vahuct bnf:ufaer_mmd(P(NETIF &Pfp_ctrlFhy[al12idx]dal[ahol t			   ch va_half_openfor n(P(NETIF var
, 1) !=INK;   S0)_MSG
;
	/* Low ==> corresFaaddEde ey 
	} fain
dfw_resp[SPREG_SFP_Rd }lchol tCHIP_IS_E3ort))t((va" = 0;
	u16 val1, rx_E_S&Pfp_ctrlFhy[INGXS_E]al[auct bnf:ufaer_mmd(P(NETIF rx_s;(vaol t(*Fl1UT33le	{
< 1);
		==
S
				AUTO_Ne
)	&OINK    S(Fl1UT)term_cax_8ask/OINKLI  LI bnx2x_8727_S
				CAPABILITY_D0_20G)) ||n"	    *Fl1UT33le	{
< 1);
		==
S
				2etd0))
p rP
	/* ch va_kr2Iwa(P(NETIF var
, rx_s;(va{
	/* ch va_ostr_curr(P(NETIF var
)al		ol tvar
->rx_tx_asic_rst)
p rP
	/* warpcman  case _runtime(al1claZE+1], var
)al(vaol t(nfig[port].FZE+1];
)) &
		PORT_HW_CIeIeX_LASER_MASK;
	DP(NETIF_MSG_L date medi_MSG
a type for non-PZE+1];
FRCM].>ereadd_cfg)wn"	    &I bnx2x_8727_/* _SERDEF_Low  mo) ==INKLI   bnx2x_8727_/* _SERDEF_LowSFIle_eep0ol tP
	/* is ffp_mUTule_plugg;doal1claZE+1])le_eep0ruct bnffp_tx_readd_de ey 
	}(al1claZE+1], var
)al		p_ firstut *var
->aramsXlink_ Oeep0rL(val[TATUS_SFP_TXYFAULGle_eep0rMA_Clean
train, in	errupt cmarects mod peds *Setp rvar
->aramsXlink_ O_P~L(val[TATUS_SFP_TXYFAULGas    var
->al12flag_/&==~S_EXSFP_TXYFAULGXFLAG;eep0rMA_Updt e aram Xlink_	in toe shar
		NETory	ontrp0ruct bnupdt eEmng(P(NETIF var
->aramsXlink_wer	va}
	 }FP}
}

k_v			   fan_fainure_de IF_q("Setting SFP_NAM,
;BCLI   uG_R") &
		POR,= 0;LI   uG_R") &
2		POR,= 0;LI   u_vFRCMdtatuu8.Fl12indexFsfan_fainure_de IF_q =.0;spE = 0;
	u16 val1,px_er_fARp(Fl12index	;	_XGXS_E1e Fl12index	< MAXXS_ES;
oduuuuuFl12index++le_eepol t			   aopult eval1(rt].Fl12indexFs)) &
		PORe ") &
2		POR,= 0;
      SIRCM]n&PhyrINKLI  !=S0)t((vaa;
	/* Low ==> corresaopult e al1,fain
dfw_resp[S_*param0;(va}
	 fan_fainure_de IF_q |
	(al1.flag_/OINKK		FLAGS4FAN_FAILURE_DEinREQsesp_tr *paramfan_fainure_de IF_q;l}lc_up) {
	/* ;
	bnx2x_pl1("Settinarams->bp;
	DP(NETIdtatuu8.Fl12index;tuEPROM_VENDOR_NAME_SIZE+1];
	charuct bnupdt eEmng(P(NETIF 0dal[uct bnbins_dis_rt].NDUAre
	  mo_INTERRUPT_ bnx0 +oPfp_ctrlFRCM*4,s	_      S(NDUA  mo_W_CF0_L(val[TATUS |n"		NDUA  mo_W_CF0_L(va10G |n"		NDUA  mo_SERDEF0_L(val[TATUS |n"		NDUA  mo_MI_INT)wertrfARp(Fl12index	;	INGXS_E;lFl12index	< MAXXS_ES;
oduuuuuFl12index++le_eepol tFZE+1];
Fhy[Fl12index].;
	bnx2x)t((vaaFZE+1];
Fhy[Fl12index].;
	bnx2x(INK,	&FZE+1];
Fhy[Fl12index],= 0;
tZE+1])al[_ FZE+1];
Fhy[Fl12index] = al12nullal[S}FP}
}
c_up) {
	/* t bn_mUT_abss,
	("Setting SFP_NAM, p = 0;
aramsvar
	Dvar
Fs	__    kvenehip_id, sG_R") &
		POR, uG_R") &
2		POR,= 0;LI  u_vFRCMdtatuu8.gpio2num =.0xff,.gpio2IRCM =.0xff,.Fl12index;tuuG_Rval;sp G_Ro_LASE, aeu_8ask,nEpapsval,nEpapsostrride, fync_o_LASE;chol tCHIP_IS_E3ort))t((vaol t			   getxmUT_abss,
	_cfg(rt].ehip_ide_MSG
	oo   
") &
		POR,= 0;
	I    SIRCM]= 0;
	I    S&gpio2num]= 0;
	I    S&gpio2FRCMw !
	b)
    *parara[} first((vaE = 0;
	u16 val1,px_er__fARp(Fl12index	;	_XGXS_E1e Fl12index	< MAXXS_ES;
o_      Fl12index++le_eep0ol t			   aopult eval1(rt].Fl12indexFs)) &
		PORe= 0;
	I    SR") &
2		POR,SIRCM]n&PhyrINKKLI  !=S0)t((vaaa;
	/* Low ==> corresaopult e al1,fain
dfw_resp[S  *parara[_ _P_[0ol tFl1.aom; ==
 bnx2x_8727_W_CFG_XGXS_EXT_PHY_TYP726)t((vaaagpio2num =.n = tx_eISTERS4GPIO_3;(vaaagpio2FRCM =.FRCMas  [SPREG_SFP_a}
	 }FP}
chol tgpio2num ==,00ff)
C_ *paraall_MA_S0G GPIO3 ao mriggerESFP+;mUTulePinsert
	}/remosal	ontruct bnf:ufgpioort].gpio2num].n = tx_eISTERS4GPIO_INPUT_HI_Z,.gpio2IRCM);s(vEpapsval;
	nfig[port].NDUAre
	 bnxDSWAP)er[Epapsostrride;
	nfig[port].NDUAre
	STRAP_OVERRIDEweragpio2FRCM ^==(epapsval;&&nEpapsostrride)alchvar
->aeu_,
	_8ask/= AEU_INPUTSlATTN_BITS_GPIO0_FUNCTION_0 <<
;Btgpio2num + tgpio2FRCM << 2)wertrfync_o_LASET=R") &
		PORT_HW_X_LASER_MASK;
	DP(NETIF_MSG_LHW_C date media type for non-P*/
	.aeu_,
	_8ask)al[x_e_WRort].fync_o_LASEF var
->aeu_,
	_8ask)alar;
	/* Low ==> corresSSEcoppdMO		ABS (GPIO%d_P%d) AEU ao bbnx2x_c
;BC CCCCCgpio2num].gpio2FRCMF var
->aeu_,
	_8ask)alarol tFRCM =
	b)
  o_LASET=Rn = tx_e_AEU_ENABLE1_FUNC_0_OUT	0;(vfirssp o_LASET=Rn = tx_e_AEU_ENABLE1_FUNC_1_OUT	0;(r_ficOpen
ippropriatseAEU for t 	errupts *Setaeu_8ask;
	nfig[port].o_LASE)al[aeu_8ask;|
	var
->aeu_,
	_8askal[x_e_WRort].o_LASE, aeu_8ask)aschMA_ nion
 taseGPIO ao mriggerEin	errupt ontrval;
	nfig[port].n = tx_e_GPIO_EV