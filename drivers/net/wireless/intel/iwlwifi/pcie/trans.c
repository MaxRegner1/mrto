/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2007 - 2015 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 - 2017 Intel Deutschland GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2005 - 2015 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 - 2017 Intel Deutschland GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#include <linux/pci.h>
#include <linux/pci-aspm.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/gfp.h>
#include <linux/vmalloc.h>
#include <linux/pm_runtime.h>

#include "iwl-drv.h"
#include "iwl-trans.h"
#include "iwl-csr.h"
#include "iwl-prph.h"
#include "iwl-scd.h"
#include "iwl-agn-hw.h"
#include "fw/error-dump.h"
#include "internal.h"
#include "iwl-fh.h"

/* extended range in FW SRAM */
#define IWL_FW_MEM_EXTENDED_START	0x40000
#define IWL_FW_MEM_EXTENDED_END		0x57FFF

static void iwl_pcie_free_fw_monitor(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);

	if (!trans_pcie->fw_mon_page)
		return;

	dma_unmap_page(trans->dev, trans_pcie->fw_mon_phys,
		       trans_pcie->fw_mon_size, DMA_FROM_DEVICE);
	__free_pages(trans_pcie->fw_mon_page,
		     get_order(trans_pcie->fw_mon_size));
	trans_pcie->fw_mon_page = NULL;
	trans_pcie->fw_mon_phys = 0;
	trans_pcie->fw_mon_size = 0;
}

static void iwl_pcie_alloc_fw_monitor(struct iwl_trans *trans, u8 max_power)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	struct page *page = NULL;
	dma_addr_t phys;
	u32 size = 0;
	u8 power;

	if (!max_power) {
		/* default max_power is maximum */
		max_power = 26;
	} else {
		max_power += 11;
	}

	if (WARN(max_power > 26,
		 "External buffer size for monitor is too big %d, check the FW TLV\n",
		 max_power))
		return;

	if (trans_pcie->fw_mon_page) {
		dma_sync_single_for_device(trans->dev, trans_pcie->fw_mon_phys,
					   trans_pcie->fw_mon_size,
					   DMA_FROM_DEVICE);
		return;
	}

	phys = 0;
	for (power = max_power; power >= 11; power--) {
		int order;

		size = BIT(power);
		order = get_order(size);
		page = alloc_pages(__GFP_COMP | __GFP_NOWARN | __GFP_ZERO,
				   order);
		if (!page)
			continue;

		phys = dma_map_page(trans->dev, page, 0, PAGE_SIZE << order,
				    DMA_FROM_DEVICE);
		if (dma_mapping_error(trans->dev, phys)) {
			__free_pages(page, order);
			page = NULL;
			continue;
		}
		IWL_INFO(trans,
			 "Allocated 0x%08x bytes (order %d) for firmware monitor.\n",
			 size, order);
		break;
	}

	if (WARN_ON_ONCE(!page))
		return;

	if (power != max_power)
		IWL_ERR(trans,
			"Sorry - debug buffer is only %luK while you requested %luK\n",
			(unsigned long)BIT(power - 10),
			(unsigned long)BIT(max_power - 10));

	trans_pcie->fw_mon_page = page;
	trans_pcie->fw_mon_phys = phys;
	trans_pcie->fw_mon_size = size;
}

static u32 iwl_trans_pcie_read_shr(struct iwl_trans *trans, u32 reg)
{
	iwl_write32(trans, HEEP_CTRL_WRD_PCIEX_CTRL_REG,
		    ((reg & 0x0000ffff) | (2 << 28)));
	return iwl_read32(trans, HEEP_CTRL_WRD_PCIEX_DATA_REG);
}

static void iwl_trans_pcie_write_shr(struct iwl_trans *trans, u32 reg, u32 val)
{
	iwl_write32(trans, HEEP_CTRL_WRD_PCIEX_DATA_REG, val);
	iwl_write32(trans, HEEP_CTRL_WRD_PCIEX_CTRL_REG,
		    ((reg & 0x0000ffff) | (3 << 28)));
}

static void iwl_pcie_set_pwr(struct iwl_trans *trans, bool vaux)
{
	if (trans->cfg->apmg_not_supported)
		return;

	if (vaux && pci_pme_capable(to_pci_dev(trans->dev), PCI_D3cold))
		iwl_set_bits_mask_prph(trans, APMG_PS_CTRL_REG,
				       APMG_PS_CTRL_VAL_PWR_SRC_VAUX,
				       ~APMG_PS_CTRL_MSK_PWR_SRC);
	else
		iwl_set_bits_mask_prph(trans, APMG_PS_CTRL_REG,
				       APMG_PS_CTRL_VAL_PWR_SRC_VMAIN,
				       ~APMG_PS_CTRL_MSK_PWR_SRC);
}

/* PCI registers */
#define PCI_CFG_RETRY_TIMEOUT	0x041

void iwl_pcie_apm_config(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	u16 lctl;
	u16 cap;

	/*
	 * HW bug W/A for instability in PCIe bus L0S->L1 transition.
	 * Check if BIOS (or OS) enabled L1-ASPM on this device.
	 * If so (likely), disable L0S, so device moves directly L0->L1;
	 *    costs negligible amount of power savings.
	 * If not (unlikely), enable L0S, so there is at least some
	 *    power savings, even without L1.
	 */
	pcie_capability_read_word(trans_pcie->pci_dev, PCI_EXP_LNKCTL, &lctl);
	if (lctl & PCI_EXP_LNKCTL_ASPM_L1)
		iwl_set_bit(trans, CSR_GIO_REG, CSR_GIO_REG_VAL_L0S_ENABLED);
	else
		iwl_clear_bit(trans, CSR_GIO_REG, CSR_GIO_REG_VAL_L0S_ENABLED);
	trans->pm_support = !(lctl & PCI_EXP_LNKCTL_ASPM_L0S);

	pcie_capability_read_word(trans_pcie->pci_dev, PCI_EXP_DEVCTL2, &cap);
	trans->ltr_enabled = cap & PCI_EXP_DEVCTL2_LTR_EN;
	IWL_DEBUG_POWER(trans, "L1 %sabled - LTR %sabled\n",
			(lctl & PCI_EXP_LNKCTL_ASPM_L1) ? "En" : "Dis",
			trans->ltr_enabled ? "En" : "Dis");
}

/*
 * Start up NIC's basic functionality after it has been reset
 * (e.g. after platform boot, or shutdown via iwl_pcie_apm_stop())
 * NOTE:  This does not load uCode nor start the embedded processor
 */
static int iwl_pcie_apm_init(struct iwl_trans *trans)
{
	int ret;

	IWL_DEBUG_INFO(trans, "Init card's basic functions\n");

	/*
	 * Use "set_bit" below rather than "write", to preserve any hardware
	 * bits already set by default after reset.
	 */

	/* Disable L0S exit timer (platform NMI Work/Around) */
	if (trans->cfg->device_family < IWL_DEVICE_FAMILY_8000)
		iwl_set_bit(trans, CSR_GIO_CHICKEN_BITS,
			    CSR_GIO_CHICKEN_BITS_REG_BIT_DIS_L0S_EXIT_TIMER);

	/*
	 * Disable L0s without affecting L1;
	 *  don't wait for ICH L0s (ICH bug W/A)
	 */
	iwl_set_bit(trans, CSR_GIO_CHICKEN_BITS,
		    CSR_GIO_CHICKEN_BITS_REG_BIT_L1A_NO_L0S_RX);

	/* Set FH wait threshold to maximum (HW error during stress W/A) */
	iwl_set_bit(trans, CSR_DBG_HPET_MEM_REG, CSR_DBG_HPET_MEM_REG_VAL);

	/*
	 * Enable HAP INTA (interrupt from management bus) to
	 * wake device's PCI Express link L1a -> L0s
	 */
	iwl_set_bit(trans, CSR_HW_IF_CONFIG_REG,
		    CSR_HW_IF_CONFIG_REG_BIT_HAP_WAKE_L1A);

	iwl_pcie_apm_config(trans);

	/* Configure analog phase-lock-loop before activating to D0A */
	if (trans->cfg->base_params->pll_cfg)
		iwl_set_bit(trans, CSR_ANA_PLL_CFG, CSR50_ANA_PLL_CFG_VAL);

	/*
	 * Set "initialization complete" bit to move adapter from
	 * D0U* --> D0A* (powered-up active) state.
	 */
	iwl_set_bit(trans, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	/*
	 * Wait for clock stabilization; once stabilized, access to
	 * device-internal resources is supported, e.g. iwl_write_prph()
	 * and accesses to uCode SRAM.
	 */
	ret = iwl_poll_bit(trans, CSR_GP_CNTRL,
			   CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
			   CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY, 25000);
	if (ret < 0) {
		IWL_ERR(trans, "Failed to init the card\n");
		return ret;
	}

	if (trans->cfg->host_interrupt_operation_mode) {
		/*
		 * This is a bit of an abuse - This is needed for 7260 / 3160
		 * only check host_interrupt_operation_mode even if this is
		 * not related to host_interrupt_operation_mode.
		 *
		 * Enable the oscillator to count wake up time for L1 exit. This
		 * consumes slightly more power (100uA) - but allows to be sure
		 * that we wake up from L1 on time.
		 *
		 * This looks weird: read twice the same register, discard the
		 * value, set a bit, and yet again, read that same register
		 * just to discard the value. But that's the way the hardware
		 * seems to like it.
		 */
		iwl_read_prph(trans, OSC_CLK);
		iwl_read_prph(trans, OSC_CLK);
		iwl_set_bits_prph(trans, OSC_CLK, OSC_CLK_FORCE_CONTROL);
		iwl_read_prph(trans, OSC_CLK);
		iwl_read_prph(trans, OSC_CLK);
	}

	/*
	 * Enable DMA clock and wait for it to stabilize.
	 *
	 * Write to "CLK_EN_REG"; "1" bits enable clocks, while "0"
	 * bits do not disable clocks.  This preserves any hardware
	 * bits already set by default in "CLK_CTRL_REG" after reset.
	 */
	if (!trans->cfg->apmg_not_supported) {
		iwl_write_prph(trans, APMG_CLK_EN_REG,
			       APMG_CLK_VAL_DMA_CLK_RQT);
		udelay(20);

		/* Disable L1-Active */
		iwl_set_bits_prph(trans, APMG_PCIDEV_STT_REG,
				  APMG_PCIDEV_STT_VAL_L1_ACT_DIS);

		/* Clear the interrupt in APMG if the NIC is in RFKILL */
		iwl_write_prph(trans, APMG_RTC_INT_STT_REG,
			       APMG_RTC_INT_STT_RFKILL);
	}

	set_bit(STATUS_DEVICE_ENABLED, &trans->status);

	return 0;
}

/*
 * Enable LP XTAL to avoid HW bug where device may consume much power if
 * FW is not loaded after device reset. LP XTAL is disabled by default
 * after device HW reset. Do it only if XTAL is fed by internal source.
 * Configure device's "persistence" mode to avoid resetting XTAL again when
 * SHRD_HW_RST occurs in S3.
 */
static void iwl_pcie_apm_lp_xtal_enable(struct iwl_trans *trans)
{
	int ret;
	u32 apmg_gp1_reg;
	u32 apmg_xtal_cfg_reg;
	u32 dl_cfg_reg;

	/* Force XTAL ON */
	__iwl_trans_pcie_set_bit(trans, CSR_GP_CNTRL,
				 CSR_GP_CNTRL_REG_FLAG_XTAL_ON);

	iwl_pcie_sw_reset(trans);

	/*
	 * Set "initialization complete" bit to move adapter from
	 * D0U* --> D0A* (powered-up active) state.
	 */
	iwl_set_bit(trans, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	/*
	 * Wait for clock stabilization; once stabilized, access to
	 * device-internal resources is possible.
	 */
	ret = iwl_poll_bit(trans, CSR_GP_CNTRL,
			   CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
			   CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
			   25000);
	if (WARN_ON(ret < 0)) {
		IWL_ERR(trans, "Access time out - failed to enable LP XTAL\n");
		/* Release XTAL ON request */
		__iwl_trans_pcie_clear_bit(trans, CSR_GP_CNTRL,
					   CSR_GP_CNTRL_REG_FLAG_XTAL_ON);
		return;
	}

	/*
	 * Clear "disable persistence" to avoid LP XTAL resetting when
	 * SHRD_HW_RST is applied in S3.
	 */
	iwl_clear_bits_prph(trans, APMG_PCIDEV_STT_REG,
				    APMG_PCIDEV_STT_VAL_PERSIST_DIS);

	/*
	 * Force APMG XTAL to be active to prevent its disabling by HW
	 * caused by APMG idle state.
	 */
	apmg_xtal_cfg_reg = iwl_trans_pcie_read_shr(trans,
						    SHR_APMG_XTAL_CFG_REG);
	iwl_trans_pcie_write_shr(trans, SHR_APMG_XTAL_CFG_REG,
				 apmg_xtal_cfg_reg |
				 SHR_APMG_XTAL_CFG_XTAL_ON_REQ);

	iwl_pcie_sw_reset(trans);

	/* Enable LP XTAL by indirect access through CSR */
	apmg_gp1_reg = iwl_trans_pcie_read_shr(trans, SHR_APMG_GP1_REG);
	iwl_trans_pcie_write_shr(trans, SHR_APMG_GP1_REG, apmg_gp1_reg |
				 SHR_APMG_GP1_WF_XTAL_LP_EN |
				 SHR_APMG_GP1_CHICKEN_BIT_SELECT);

	/* Clear delay line clock power up */
	dl_cfg_reg = iwl_trans_pcie_read_shr(trans, SHR_APMG_DL_CFG_REG);
	iwl_trans_pcie_write_shr(trans, SHR_APMG_DL_CFG_REG, dl_cfg_reg &
				 ~SHR_APMG_DL_CFG_DL_CLOCK_POWER_UP);

	/*
	 * Enable persistence mode to avoid LP XTAL resetting when
	 * SHRD_HW_RST is applied in S3.
	 */
	iwl_set_bit(trans, CSR_HW_IF_CONFIG_REG,
		    CSR_HW_IF_CONFIG_REG_PERSIST_MODE);

	/*
	 * Clear "initialization complete" bit to move adapter from
	 * D0A* (powered-up Active) --> D0U* (Uninitialized) state.
	 */
	iwl_clear_bit(trans, CSR_GP_CNTRL,
		      CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	/* Activates XTAL resources monitor */
	__iwl_trans_pcie_set_bit(trans, CSR_MONITOR_CFG_REG,
				 CSR_MONITOR_XTAL_RESOURCES);

	/* Release XTAL ON request */
	__iwl_trans_pcie_clear_bit(trans, CSR_GP_CNTRL,
				   CSR_GP_CNTRL_REG_FLAG_XTAL_ON);
	udelay(10);

	/* Release APMG XTAL */
	iwl_trans_pcie_write_shr(trans, SHR_APMG_XTAL_CFG_REG,
				 apmg_xtal_cfg_reg &
				 ~SHR_APMG_XTAL_CFG_XTAL_ON_REQ);
}

void iwl_pcie_apm_stop_master(struct iwl_trans *trans)
{
	int ret;

	/* stop device's busmaster DMA activity */
	iwl_set_bit(trans, CSR_RESET, CSR_RESET_REG_FLAG_STOP_MASTER);

	ret = iwl_poll_bit(trans, CSR_RESET,
			   CSR_RESET_REG_FLAG_MASTER_DISABLED,
			   CSR_RESET_REG_FLAG_MASTER_DISABLED, 100);
	if (ret < 0)
		IWL_WARN(trans, "Master Disable Timed Out, 100 usec\n");

	IWL_DEBUG_INFO(trans, "stop master\n");
}

static void iwl_pcie_apm_stop(struct iwl_trans *trans, bool op_mode_leave)
{
	IWL_DEBUG_INFO(trans, "Stop card, put in low power state\n");

	if (op_mode_leave) {
		if (!test_bit(STATUS_DEVICE_ENABLED, &trans->status))
			iwl_pcie_apm_init(trans);

		/* inform ME that we are leaving */
		if (trans->cfg->device_family == IWL_DEVICE_FAMILY_7000)
			iwl_set_bits_prph(trans, APMG_PCIDEV_STT_REG,
					  APMG_PCIDEV_STT_VAL_WAKE_ME);
		else if (trans->cfg->device_family >= IWL_DEVICE_FAMILY_8000) {
			iwl_set_bit(trans, CSR_DBG_LINK_PWR_MGMT_REG,
				    CSR_RESET_LINK_PWR_MGMT_DISABLED);
			iwl_set_bit(trans, CSR_HW_IF_CONFIG_REG,
				    CSR_HW_IF_CONFIG_REG_PREPARE |
				    CSR_HW_IF_CONFIG_REG_ENABLE_PME);
			mdelay(1);
			iwl_clear_bit(trans, CSR_DBG_LINK_PWR_MGMT_REG,
				      CSR_RESET_LINK_PWR_MGMT_DISABLED);
		}
		mdelay(5);
	}

	clear_bit(STATUS_DEVICE_ENABLED, &trans->status);

	/* Stop device's DMA activity */
	iwl_pcie_apm_stop_master(trans);

	if (trans->cfg->lp_xtal_workaround) {
		iwl_pcie_apm_lp_xtal_enable(trans);
		return;
	}

	iwl_pcie_sw_reset(trans);

	/*
	 * Clear "initialization complete" bit to move adapter from
	 * D0A* (powered-up Active) --> D0U* (Uninitialized) state.
	 */
	iwl_clear_bit(trans, CSR_GP_CNTRL,
		      CSR_GP_CNTRL_REG_FLAG_INIT_DONE);
}

static int iwl_pcie_nic_init(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	int ret;

	/* nic_init */
	spin_lock(&trans_pcie->irq_lock);
	ret = iwl_pcie_apm_init(trans);
	spin_unlock(&trans_pcie->irq_lock);

	if (ret)
		return ret;

	iwl_pcie_set_pwr(trans, false);

	iwl_op_mode_nic_config(trans->op_mode);

	/* Allocate the RX queue, or reset if it is already allocated */
	iwl_pcie_rx_init(trans);

	/* Allocate or reset and init all Tx and Command queues */
	if (iwl_pcie_tx_init(trans))
		return -ENOMEM;

	if (trans->cfg->base_params->shadow_reg_enable) {
		/* enable shadow regs in HW */
		iwl_set_bit(trans, CSR_MAC_SHADOW_REG_CTRL, 0x800FFFFF);
		IWL_DEBUG_INFO(trans, "Enabling shadow registers in device\n");
	}

	return 0;
}

#define HW_READY_TIMEOUT (50)

/* Note: returns poll_bit return value, which is >= 0 if success */
static int iwl_pcie_set_hw_ready(struct iwl_trans *trans)
{
	int ret;

	iwl_set_bit(trans, CSR_HW_IF_CONFIG_REG,
		    CSR_HW_IF_CONFIG_REG_BIT_NIC_READY);

	/* See if we got it */
	ret = iwl_poll_bit(trans, CSR_HW_IF_CONFIG_REG,
			   CSR_HW_IF_CONFIG_REG_BIT_NIC_READY,
			   CSR_HW_IF_CONFIG_REG_BIT_NIC_READY,
			   HW_READY_TIMEOUT);

	if (ret >= 0)
		iwl_set_bit(trans, CSR_MBOX_SET_REG, CSR_MBOX_SET_REG_OS_ALIVE);

	IWL_DEBUG_INFO(trans, "hardware%s ready\n", ret < 0 ? " not" : "");
	return ret;
}

/* Note: returns standard 0/-ERROR code */
int iwl_pcie_prepare_card_hw(struct iwl_trans *trans)
{
	int ret;
	int t = 0;
	int iter;

	IWL_DEBUG_INFO(trans, "iwl_trans_prepare_card_hw enter\n");

	ret = iwl_pcie_set_hw_ready(trans);
	/* If the card is ready, exit 0 */
	if (ret >= 0)
		return 0;

	iwl_set_bit(trans, CSR_DBG_LINK_PWR_MGMT_REG,
		    CSR_RESET_LINK_PWR_MGMT_DISABLED);
	usleep_range(1000, 2000);

	for (iter = 0; iter < 10; iter++) {
		/* If HW is not ready, prepare the conditions to check again */
		iwl_set_bit(trans, CSR_HW_IF_CONFIG_REG,
			    CSR_HW_IF_CONFIG_REG_PREPARE);

		do {
			ret = iwl_pcie_set_hw_ready(trans);
			if (ret >= 0)
				return 0;

			usleep_range(200, 1000);
			t += 200;
		} while (t < 150000);
		msleep(25);
	}

	IWL_ERR(trans, "Couldn't prepare the card\n");

	return ret;
}

/*
 * ucode
 */
static void iwl_pcie_load_firmware_chunk_fh(struct iwl_trans *trans,
					    u32 dst_addr, dma_addr_t phy_addr,
					    u32 byte_cnt)
{
	iwl_write32(trans, FH_TCSR_CHNL_TX_CONFIG_REG(FH_SRVC_CHNL),
		    FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_PAUSE);

	iwl_write32(trans, FH_SRVC_CHNL_SRAM_ADDR_REG(FH_SRVC_CHNL),
		    dst_addr);

	iwl_write32(trans, FH_TFDIB_CTRL0_REG(FH_SRVC_CHNL),
		    phy_addr & FH_MEM_TFDIB_DRAM_ADDR_LSB_MSK);

	iwl_write32(trans, FH_TFDIB_CTRL1_REG(FH_SRVC_CHNL),
		    (iwl_get_dma_hi_addr(phy_addr)
			<< FH_MEM_TFDIB_REG1_ADDR_BITSHIFT) | byte_cnt);

	iwl_write32(trans, FH_TCSR_CHNL_TX_BUF_STS_REG(FH_SRVC_CHNL),
		    BIT(FH_TCSR_CHNL_TX_BUF_STS_REG_POS_TB_NUM) |
		    BIT(FH_TCSR_CHNL_TX_BUF_STS_REG_POS_TB_IDX) |
		    FH_TCSR_CHNL_TX_BUF_STS_REG_VAL_TFDB_VALID);

	iwl_write32(trans, FH_TCSR_CHNL_TX_CONFIG_REG(FH_SRVC_CHNL),
		    FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_ENABLE |
		    FH_TCSR_TX_CONFIG_REG_VAL_DMA_CREDIT_DISABLE |
		    FH_TCSR_TX_CONFIG_REG_VAL_CIRQ_HOST_ENDTFD);
}

static int iwl_pcie_load_firmware_chunk(struct iwl_trans *trans,
					u32 dst_addr, dma_addr_t phy_addr,
					u32 byte_cnt)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	unsigned long flags;
	int ret;

	trans_pcie->ucode_write_complete = false;

	if (!iwl_trans_grab_nic_access(trans, &flags))
		return -EIO;

	iwl_pcie_load_firmware_chunk_fh(trans, dst_addr, phy_addr,
					byte_cnt);
	iwl_trans_release_nic_access(trans, &flags);

	ret = wait_event_timeout(trans_pcie->ucode_write_waitq,
				 trans_pcie->ucode_write_complete, 5 * HZ);
	if (!ret) {
		IWL_ERR(trans, "Failed to load firmware chunk!\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int iwl_pcie_load_section(struct iwl_trans *trans, u8 section_num,
			    const struct fw_desc *section)
{
	u8 *v_addr;
	dma_addr_t p_addr;
	u32 offset, chunk_sz = min_t(u32, FH_MEM_TB_MAX_LENGTH, section->len);
	int ret = 0;

	IWL_DEBUG_FW(trans, "[%d] uCode section being loaded...\n",
		     section_num);

	v_addr = dma_alloc_coherent(trans->dev, chunk_sz, &p_addr,
				    GFP_KERNEL | __GFP_NOWARN);
	if (!v_addr) {
		IWL_DEBUG_INFO(trans, "Falling back to small chunks of DMA\n");
		chunk_sz = PAGE_SIZE;
		v_addr = dma_alloc_coherent(trans->dev, chunk_sz,
					    &p_addr, GFP_KERNEL);
		if (!v_addr)
			return -ENOMEM;
	}

	for (offset = 0; offset < section->len; offset += chunk_sz) {
		u32 copy_size, dst_addr;
		bool extended_addr = false;

		copy_size = min_t(u32, chunk_sz, section->len - offset);
		dst_addr = section->offset + offset;

		if (dst_addr >= IWL_FW_MEM_EXTENDED_START &&
		    dst_addr <= IWL_FW_MEM_EXTENDED_END)
			extended_addr = true;

		if (extended_addr)
			iwl_set_bits_prph(trans, LMPM_CHICK,
					  LMPM_CHICK_EXTENDED_ADDR_SPACE);

		memcpy(v_addr, (u8 *)section->data + offset, copy_size);
		ret = iwl_pcie_load_firmware_chunk(trans, dst_addr, p_addr,
						   copy_size);

		if (extended_addr)
			iwl_clear_bits_prph(trans, LMPM_CHICK,
					    LMPM_CHICK_EXTENDED_ADDR_SPACE);

		if (ret) {
			IWL_ERR(trans,
				"Could not load the [%d] uCode section\n",
				section_num);
			break;
		}
	}

	dma_free_coherent(trans->dev, chunk_sz, v_addr, p_addr);
	return ret;
}

static int iwl_pcie_load_cpu_sections_8000(struct iwl_trans *trans,
					   const struct fw_img *image,
					   int cpu,
					   int *first_ucode_section)
{
	int shift_param;
	int i, ret = 0, sec_num = 0x1;
	u32 val, last_read_idx = 0;

	if (cpu == 1) {
		shift_param = 0;
		*first_ucode_section = 0;
	} else {
		shift_param = 16;
		(*first_ucode_section)++;
	}

	for (i = *first_ucode_section; i < image->num_sec; i++) {
		last_read_idx = i;

		/*
		 * CPU1_CPU2_SEPARATOR_SECTION delimiter - separate between
		 * CPU1 to CPU2.
		 * PAGING_SEPARATOR_SECTION delimiter - separate between
		 * CPU2 non paged to CPU2 paging sec.
		 */
		if (!image->sec[i].data ||
		    image->sec[i].offset == CPU1_CPU2_SEPARATOR_SECTION ||
		    image->sec[i].offset == PAGING_SEPARATOR_SECTION) {
			IWL_DEBUG_FW(trans,
				     "Break since Data not valid or Empty section, sec = %d\n",
				     i);
			break;
		}

		ret = iwl_pcie_load_section(trans, i, &image->sec[i]);
		if (ret)
			return ret;

		/* Notify ucode of loaded section number and status */
		val = iwl_read_direct32(trans, FH_UCODE_LOAD_STATUS);
		val = val | (sec_num << shift_param);
		iwl_write_direct32(trans, FH_UCODE_LOAD_STATUS, val);

		sec_num = (sec_num << 1) | 0x1;
	}

	*first_ucode_section = last_read_idx;

	iwl_enable_interrupts(trans);

	if (trans->cfg->use_tfh) {
		if (cpu == 1)
			iwl_write_prph(trans, UREG_UCODE_LOAD_STATUS,
				       0xFFFF);
		else
			iwl_write_prph(trans, UREG_UCODE_LOAD_STATUS,
				       0xFFFFFFFF);
	} else {
		if (cpu == 1)
			iwl_write_direct32(trans, FH_UCODE_LOAD_STATUS,
					   0xFFFF);
		else
			iwl_write_direct32(trans, FH_UCODE_LOAD_STATUS,
					   0xFFFFFFFF);
	}

	return 0;
}

static int iwl_pcie_load_cpu_sections(struct iwl_trans *trans,
				      const struct fw_img *image,
				      int cpu,
				      int *first_ucode_section)
{
	int i, ret = 0;
	u32 last_read_idx = 0;

	if (cpu == 1)
		*first_ucode_section = 0;
	else
		(*first_ucode_section)++;

	for (i = *first_ucode_section; i < image->num_sec; i++) {
		last_read_idx = i;

		/*
		 * CPU1_CPU2_SEPARATOR_SECTION delimiter - separate between
		 * CPU1 to CPU2.
		 * PAGING_SEPARATOR_SECTION delimiter - separate between
		 * CPU2 non paged to CPU2 paging sec.
		 */
		if (!image->sec[i].data ||
		    image->sec[i].offset == CPU1_CPU2_SEPARATOR_SECTION ||
		    image->sec[i].offset == PAGING_SEPARATOR_SECTION) {
			IWL_DEBUG_FW(trans,
				     "Break since Data not valid or Empty section, sec = %d\n",
				     i);
			break;
		}

		ret = iwl_pcie_load_section(trans, i, &image->sec[i]);
		if (ret)
			return ret;
	}

	*first_ucode_section = last_read_idx;

	return 0;
}

void iwl_pcie_apply_destination(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	const struct iwl_fw_dbg_dest_tlv *dest = trans->dbg_dest_tlv;
	int i;

	if (dest->version)
		IWL_ERR(trans,
			"DBG DEST version is %d - expect issues\n",
			dest->version);

	IWL_INFO(trans, "Applying debug destination %s\n",
		 get_fw_dbg_mode_string(dest->monitor_mode));

	if (dest->monitor_mode == EXTERNAL_MODE)
		iwl_pcie_alloc_fw_monitor(trans, dest->size_power);
	else
		IWL_WARN(trans, "PCI should have external buffer debug\n");

	for (i = 0; i < trans->dbg_dest_reg_num; i++) {
		u32 addr = le32_to_cpu(dest->reg_ops[i].addr);
		u32 val = le32_to_cpu(dest->reg_ops[i].val);

		switch (dest->reg_ops[i].op) {
		case CSR_ASSIGN:
			iwl_write32(trans, addr, val);
			break;
		case CSR_SETBIT:
			iwl_set_bit(trans, addr, BIT(val));
			break;
		case CSR_CLEARBIT:
			iwl_clear_bit(trans, addr, BIT(val));
			break;
		case PRPH_ASSIGN:
			iwl_write_prph(trans, addr, val);
			break;
		case PRPH_SETBIT:
			iwl_set_bits_prph(trans, addr, BIT(val));
			break;
		case PRPH_CLEARBIT:
			iwl_clear_bits_prph(trans, addr, BIT(val));
			break;
		case PRPH_BLOCKBIT:
			if (iwl_read_prph(trans, addr) & BIT(val)) {
				IWL_ERR(trans,
					"BIT(%u) in address 0x%x is 1, stopping FW configuration\n",
					val, addr);
				goto monitor;
			}
			break;
		default:
			IWL_ERR(trans, "FW debug - unknown OP %d\n",
				dest->reg_ops[i].op);
			break;
		}
	}

monitor:
	if (dest->monitor_mode == EXTERNAL_MODE && trans_pcie->fw_mon_size) {
		iwl_write_prph(trans, le32_to_cpu(dest->base_reg),
			       trans_pcie->fw_mon_phys >> dest->base_shift);
		if (trans->cfg->device_family >= IWL_DEVICE_FAMILY_8000)
			iwl_write_prph(trans, le32_to_cpu(dest->end_reg),
				       (trans_pcie->fw_mon_phys +
					trans_pcie->fw_mon_size - 256) >>
						dest->end_shift);
		else
			iwl_write_prph(trans, le32_to_cpu(dest->end_reg),
				       (trans_pcie->fw_mon_phys +
					trans_pcie->fw_mon_size) >>
						dest->end_shift);
	}
}

static int iwl_pcie_load_given_ucode(struct iwl_trans *trans,
				const struct fw_img *image)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	int ret = 0;
	int first_ucode_section;

	IWL_DEBUG_FW(trans, "working with %s CPU\n",
		     image->is_dual_cpus ? "Dual" : "Single");

	/* load to FW the binary non secured sections of CPU1 */
	ret = iwl_pcie_load_cpu_sections(trans, image, 1, &first_ucode_section);
	if (ret)
		return ret;

	if (image->is_dual_cpus) {
		/* set CPU2 header address */
		iwl_write_prph(trans,
			       LMPM_SECURE_UCODE_LOAD_CPU2_HDR_ADDR,
			       LMPM_SECURE_CPU2_HDR_MEM_SPACE);

		/* load to FW the binary sections of CPU2 */
		ret = iwl_pcie_load_cpu_sections(trans, image, 2,
						 &first_ucode_section);
		if (ret)
			return ret;
	}

	/* supported for 7000 only for the moment */
	if (iwlwifi_mod_params.fw_monitor &&
	    trans->cfg->device_family == IWL_DEVICE_FAMILY_7000) {
		iwl_pcie_alloc_fw_monitor(trans, 0);

		if (trans_pcie->fw_mon_size) {
			iwl_write_prph(trans, MON_BUFF_BASE_ADDR,
				       trans_pcie->fw_mon_phys >> 4);
			iwl_write_prph(trans, MON_BUFF_END_ADDR,
				       (trans_pcie->fw_mon_phys +
					trans_pcie->fw_mon_size) >> 4);
		}
	} else if (trans->dbg_dest_tlv) {
		iwl_pcie_apply_destination(trans);
	}

	iwl_enable_interrupts(trans);

	/* release CPU reset */
	iwl_write32(trans, CSR_RESET, 0);

	return 0;
}

static int iwl_pcie_load_given_ucode_8000(struct iwl_trans *trans,
					  const struct fw_img *image)
{
	int ret = 0;
	int first_ucode_section;

	IWL_DEBUG_FW(trans, "working with %s CPU\n",
		     image->is_dual_cpus ? "Dual" : "Single");

	if (trans->dbg_dest_tlv)
		iwl_pcie_apply_destination(trans);

	IWL_DEBUG_POWER(trans, "Original WFPM value = 0x%08X\n",
			iwl_read_prph(trans, WFPM_GP2));

	/*
	 * Set default value. On resume reading the values that were
	 * zeored can provide debug data on the resume flow.
	 * This is for debugging only and has no functional impact.
	 */
	iwl_write_prph(trans, WFPM_GP2, 0x01010101);

	/* configure the ucode to be ready to get the secured image */
	/* release CPU reset */
	iwl_write_prph(trans, RELEASE_CPU_RESET, RELEASE_CPU_RESET_BIT);

	/* load to FW the binary Secured sections of CPU1 */
	ret = iwl_pcie_load_cpu_sections_8000(trans, image, 1,
					      &first_ucode_section);
	if (ret)
		return ret;

	/* load to FW the binary sections of CPU2 */
	return iwl_pcie_load_cpu_sections_8000(trans, image, 2,
					       &first_ucode_section);
}

bool iwl_pcie_check_hw_rf_kill(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie =  IWL_TRANS_GET_PCIE_TRANS(trans);
	bool hw_rfkill = iwl_is_rfkill_set(trans);
	bool prev = test_bit(STATUS_RFKILL_OPMODE, &trans->status);
	bool report;

	if (hw_rfkill) {
		set_bit(STATUS_RFKILL_HW, &trans->status);
		set_bit(STATUS_RFKILL_OPMODE, &trans->status);
	} else {
		clear_bit(STATUS_RFKILL_HW, &trans->status);
		if (trans_pcie->opmode_down)
			clear_bit(STATUS_RFKILL_OPMODE, &trans->status);
	}

	report = test_bit(STATUS_RFKILL_OPMODE, &trans->status);

	if (prev != report)
		iwl_trans_pcie_rf_kill(trans, report);

	return hw_rfkill;
}

struct iwl_causes_list {
	u32 cause_num;
	u32 mask_reg;
	u8 addr;
};

static struct iwl_causes_list causes_list[] = {
	{MSIX_FH_INT_CAUSES_D2S_CH0_NUM,	CSR_MSIX_FH_INT_MASK_AD, 0},
	{MSIX_FH_INT_CAUSES_D2S_CH1_NUM,	CSR_MSIX_FH_INT_MASK_AD, 0x1},
	{MSIX_FH_INT_CAUSES_S2D,		CSR_MSIX_FH_INT_MASK_AD, 0x3},
	{MSIX_FH_INT_CAUSES_FH_ERR,		CSR_MSIX_FH_INT_MASK_AD, 0x5},
	{MSIX_HW_INT_CAUSES_REG_ALIVE,		CSR_MSIX_HW_INT_MASK_AD, 0x10},
	{MSIX_HW_INT_CAUSES_REG_WAKEUP,		CSR_MSIX_HW_INT_MASK_AD, 0x11},
	{MSIX_HW_INT_CAUSES_REG_CT_KILL,	CSR_MSIX_HW_INT_MASK_AD, 0x16},
	{MSIX_HW_INT_CAUSES_REG_RF_KILL,	CSR_MSIX_HW_INT_MASK_AD, 0x17},
	{MSIX_HW_INT_CAUSES_REG_PERIODIC,	CSR_MSIX_HW_INT_MASK_AD, 0x18},
	{MSIX_HW_INT_CAUSES_REG_SW_ERR,		CSR_MSIX_HW_INT_MASK_AD, 0x29},
	{MSIX_HW_INT_CAUSES_REG_SCD,		CSR_MSIX_HW_INT_MASK_AD, 0x2A},
	{MSIX_HW_INT_CAUSES_REG_FH_TX,		CSR_MSIX_HW_INT_MASK_AD, 0x2B},
	{MSIX_HW_INT_CAUSES_REG_HW_ERR,		CSR_MSIX_HW_INT_MASK_AD, 0x2D},
	{MSIX_HW_INT_CAUSES_REG_HAP,		CSR_MSIX_HW_INT_MASK_AD, 0x2E},
};

static void iwl_pcie_map_non_rx_causes(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie =  IWL_TRANS_GET_PCIE_TRANS(trans);
	int val = trans_pcie->def_irq | MSIX_NON_AUTO_CLEAR_CAUSE;
	int i;

	/*
	 * Access all non RX causes and map them to the default irq.
	 * In case we are missing at least one interrupt vector,
	 * the first interrupt vector will serve non-RX and FBQ causes.
	 */
	for (i = 0; i < ARRAY_SIZE(causes_list); i++) {
		iwl_write8(trans, CSR_MSIX_IVAR(causes_list[i].addr), val);
		iwl_clear_bit(trans, causes_list[i].mask_reg,
			      causes_list[i].cause_num);
	}
}

static void iwl_pcie_map_rx_causes(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	u32 offset =
		trans_pcie->shared_vec_mask & IWL_SHARED_IRQ_FIRST_RSS ? 1 : 0;
	u32 val, idx;

	/*
	 * The first RX queue - fallback queue, which is designated for
	 * management frame, command responses etc, is always mapped to the
	 * first interrupt vector. The other RX queues are mapped to
	 * the other (N - 2) interrupt vectors.
	 */
	val = BIT(MSIX_FH_INT_CAUSES_Q(0));
	for (idx = 1; idx < trans->num_rx_queues; idx++) {
		iwl_write8(trans, CSR_MSIX_RX_IVAR(idx),
			   MSIX_FH_INT_CAUSES_Q(idx - offset));
		val |= BIT(MSIX_FH_INT_CAUSES_Q(idx));
	}
	iwl_write32(trans, CSR_MSIX_FH_INT_MASK_AD, ~val);

	val = MSIX_FH_INT_CAUSES_Q(0);
	if (trans_pcie->shared_vec_mask & IWL_SHARED_IRQ_NON_RX)
		val |= MSIX_NON_AUTO_CLEAR_CAUSE;
	iwl_write8(trans, CSR_MSIX_RX_IVAR(0), val);

	if (trans_pcie->shared_vec_mask & IWL_SHARED_IRQ_FIRST_RSS)
		iwl_write8(trans, CSR_MSIX_RX_IVAR(1), val);
}

void iwl_pcie_conf_msix_hw(struct iwl_trans_pcie *trans_pcie)
{
	struct iwl_trans *trans = trans_pcie->trans;

	if (!trans_pcie->msix_enabled) {
		if (trans->cfg->mq_rx_supported &&
		    test_bit(STATUS_DEVICE_ENABLED, &trans->status))
			iwl_write_prph(trans, UREG_CHICK,
				       UREG_CHICK_MSI_ENABLE);
		return;
	}
	/*
	 * The IVAR table needs to be configured again after reset,
	 * but if the device is disabled, we can't write to
	 * prph.
	 */
	if (test_bit(STATUS_DEVICE_ENABLED, &trans->status))
		iwl_write_prph(trans, UREG_CHICK, UREG_CHICK_MSIX_ENABLE);

	/*
	 * Each cause from the causes list above and the RX causes is
	 * represented as a byte in the IVAR table. The first nibble
	 * represents the bound interrupt vector of the cause, the second
	 * represents no auto clear for this cause. This will be set if its
	 * interrupt vector is bound to serve other causes.
	 */
	iwl_pcie_map_rx_causes(trans);

	iwl_pcie_map_non_rx_causes(trans);
}

static void iwl_pcie_init_msix(struct iwl_trans_pcie *trans_pcie)
{
	struct iwl_trans *trans = trans_pcie->trans;

	iwl_pcie_conf_msix_hw(trans_pcie);

	if (!trans_pcie->msix_enabled)
		return;

	trans_pcie->fh_init_mask = ~iwl_read32(trans, CSR_MSIX_FH_INT_MASK_AD);
	trans_pcie->fh_mask = trans_pcie->fh_init_mask;
	trans_pcie->hw_init_mask = ~iwl_read32(trans, CSR_MSIX_HW_INT_MASK_AD);
	trans_pcie->hw_mask = trans_pcie->hw_init_mask;
}

static void _iwl_trans_pcie_stop_device(struct iwl_trans *trans, bool low_power)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);

	lockdep_assert_held(&trans_pcie->mutex);

	if (trans_pcie->is_down)
		return;

	trans_pcie->is_down = true;

	/* Stop dbgc before stopping device */
	if (trans->cfg->device_family == IWL_DEVICE_FAMILY_7000) {
		iwl_set_bits_prph(trans, MON_BUFF_SAMPLE_CTL, 0x100);
	} else {
		iwl_write_prph(trans, DBGC_IN_SAMPLE, 0);
		udelay(100);
		iwl_write_prph(trans, DBGC_OUT_CTRL, 0);
	}

	/* tell the device to stop sending interrupts */
	iwl_disable_interrupts(trans);

	/* device going down, Stop using ICT table */
	iwl_pcie_disable_ict(trans);

	/*
	 * If a HW restart happens during firmware loading,
	 * then the firmware loading might call this function
	 * and later it might be called again due to the
	 * restart. So don't process again if the device is
	 * already dead.
	 */
	if (test_and_clear_bit(STATUS_DEVICE_ENABLED, &trans->status)) {
		IWL_DEBUG_INFO(trans,
			       "DEVICE_ENABLED bit was set and is now cleared\n");
		iwl_pcie_tx_stop(trans);
		iwl_pcie_rx_stop(trans);

		/* Power-down device's busmaster DMA clocks */
		if (!trans->cfg->apmg_not_supported) {
			iwl_write_prph(trans, APMG_CLK_DIS_REG,
				       APMG_CLK_VAL_DMA_CLK_RQT);
			udelay(5);
		}
	}

	/* Make sure (redundant) we've released our request to stay awake */
	iwl_clear_bit(trans, CSR_GP_CNTRL,
		      CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);

	/* Stop the device, and put it in low power state */
	iwl_pcie_apm_stop(trans, false);

	iwl_pcie_sw_reset(trans);

	/*
	 * Upon stop, the IVAR table gets erased, so msi-x won't
	 * work. This causes a bug in RF-KILL flows, since the interrupt
	 * that enables radio won't fire on the correct irq, and the
	 * driver won't be able to handle the interrupt.
	 * Configure the IVAR table again after reset.
	 */
	iwl_pcie_conf_msix_hw(trans_pcie);

	/*
	 * Upon stop, the APM issues an interrupt if HW RF kill is set.
	 * This is a bug in certain verions of the hardware.
	 * Certain devices also keep sending HW RF kill interrupt all
	 * the time, unless the interrupt is ACKed even if the interrupt
	 * should be masked. Re-ACK all the interrupts here.
	 */
	iwl_disable_interrupts(trans);

	/* clear all status bits */
	clear_bit(STATUS_SYNC_HCMD_ACTIVE, &trans->status);
	clear_bit(STATUS_INT_ENABLED, &trans->status);
	clear_bit(STATUS_TPOWER_PMI, &trans->status);

	/*
	 * Even if we stop the HW, we still want the RF kill
	 * interrupt
	 */
	iwl_enable_rfkill_int(trans);

	/* re-take ownership to prevent other users from stealing the device */
	iwl_pcie_prepare_card_hw(trans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);

	if (trans_pcie->msix_enabled) {
		int i;

		for (i = 0; i < trans_pcie->alloc_vecs; i++)
			synchronize_irq(trans_pcie->msix_entries[i].vector);
	} else {
		synchronize_irq(trans_pcie->pci_dev->irq);
	}
}

static int iwl_trans_pcie_start_fw(struct iwl_trans *trans,
				   const struct fw_img *fw, bool run_in_rfkill)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	bool hw_rfkill;
	int ret;

	/* This may fail if AMT took ownership of the device */
	if (iwl_pcie_prepare_card_hw(trans)) {
		IWL_WARN(trans, "Exit HW not ready\n");
		ret = -EIO;
		goto out;
	}

	iwl_enable_rfkill_int(trans);

	iwl_write32(trans, CSR_INT, 0xFFFFFFFF);

	/*
	 * We enabled the RF-Kill interrupt and the handler may very
	 * well be running. Disable the interrupts to make sure no other
	 * interrupt can be fired.
	 */
	iwl_disable_interrupts(trans);

	/* Make sure it finished running */
	iwl_pcie_synchronize_irqs(trans);

	mutex_lock(&trans_pcie->mutex);

	/* If platform's RF_KILL switch is NOT set to KILL */
	hw_rfkill = iwl_pcie_check_hw_rf_kill(trans);
	if (hw_rfkill && !run_in_rfkill) {
		ret = -ERFKILL;
		goto out;
	}

	/* Someone called stop_device, don't try to start_fw */
	if (trans_pcie->is_down) {
		IWL_WARN(trans,
			 "Can't start_fw since the HW hasn't been started\n");
		ret = -EIO;
		goto out;
	}

	/* make sure rfkill handshake bits are cleared */
	iwl_write32(trans, CSR_UCODE_DRV_GP1_CLR, CSR_UCODE_SW_BIT_RFKILL);
	iwl_write32(trans, CSR_UCODE_DRV_GP1_CLR,
		    CSR_UCODE_DRV_GP1_BIT_CMD_BLOCKED);

	/* clear (again), then enable host interrupts */
	iwl_write32(trans, CSR_INT, 0xFFFFFFFF);

	ret = iwl_pcie_nic_init(trans);
	if (ret) {
		IWL_ERR(trans, "Unable to init nic\n");
		goto out;
	}

	/*
	 * Now, we load the firmware and don't want to be interrupted, even
	 * by the RF-Kill interrupt (hence mask all the interrupt besides the
	 * FH_TX interrupt which is needed to load the firmware). If the
	 * RF-Kill switch is toggled, we will find out after having loaded
	 * the firmware and return the proper value to the caller.
	 */
	iwl_enable_fw_load_int(trans);

	/* really make sure rfkill handshake bits are cleared */
	iwl_write32(trans, CSR_UCODE_DRV_GP1_CLR, CSR_UCODE_SW_BIT_RFKILL);
	iwl_write32(trans, CSR_UCODE_DRV_GP1_CLR, CSR_UCODE_SW_BIT_RFKILL);

	/* Load the given image to the HW */
	if (trans->cfg->device_family >= IWL_DEVICE_FAMILY_8000)
		ret = iwl_pcie_load_given_ucode_8000(trans, fw);
	else
		ret = iwl_pcie_load_given_ucode(trans, fw);

	/* re-check RF-Kill state since we may have missed the interrupt */
	hw_rfkill = iwl_pcie_check_hw_rf_kill(trans);
	if (hw_rfkill && !run_in_rfkill)
		ret = -ERFKILL;

out:
	mutex_unlock(&trans_pcie->mutex);
	return ret;
}

static void iwl_trans_pcie_fw_alive(struct iwl_trans *trans, u32 scd_addr)
{
	iwl_pcie_reset_ict(trans);
	iwl_pcie_tx_start(trans, scd_addr);
}

void iwl_trans_pcie_handle_stop_rfkill(struct iwl_trans *trans,
				       bool was_in_rfkill)
{
	bool hw_rfkill;

	/*
	 * Check again since the RF kill state may have changed while
	 * all the interrupts were disabled, in this case we couldn't
	 * receive the RF kill interrupt and update the state in the
	 * op_mode.
	 * Don't call the op_mode if the rkfill state hasn't changed.
	 * This allows the op_mode to call stop_device from the rfkill
	 * notification without endless recursion. Under very rare
	 * circumstances, we might have a small recursion if the rfkill
	 * state changed exactly now while we were called from stop_device.
	 * This is very unlikely but can happen and is supported.
	 */
	hw_rfkill = iwl_is_rfkill_set(trans);
	if (hw_rfkill) {
		set_bit(STATUS_RFKILL_HW, &trans->status);
		set_bit(STATUS_RFKILL_OPMODE, &trans->status);
	} else {
		clear_bit(STATUS_RFKILL_HW, &trans->status);
		clear_bit(STATUS_RFKILL_OPMODE, &trans->status);
	}
	if (hw_rfkill != was_in_rfkill)
		iwl_trans_pcie_rf_kill(trans, hw_rfkill);
}

static void iwl_trans_pcie_stop_device(struct iwl_trans *trans, bool low_power)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	bool was_in_rfkill;

	mutex_lock(&trans_pcie->mutex);
	trans_pcie->opmode_down = true;
	was_in_rfkill = test_bit(STATUS_RFKILL_OPMODE, &trans->status);
	_iwl_trans_pcie_stop_device(trans, low_power);
	iwl_trans_pcie_handle_stop_rfkill(trans, was_in_rfkill);
	mutex_unlock(&trans_pcie->mutex);
}

void iwl_trans_pcie_rf_kill(struct iwl_trans *trans, bool state)
{
	struct iwl_trans_pcie __maybe_unused *trans_pcie =
		IWL_TRANS_GET_PCIE_TRANS(trans);

	lockdep_assert_held(&trans_pcie->mutex);

	IWL_WARN(trans, "reporting RF_KILL (radio %s)\n",
		 state ? "disabled" : "enabled");
	if (iwl_op_mode_hw_rf_kill(trans->op_mode, state)) {
		if (trans->cfg->gen2)
			_iwl_trans_pcie_gen2_stop_device(trans, true);
		else
			_iwl_trans_pcie_stop_device(trans, true);
	}
}

static void iwl_trans_pcie_d3_suspend(struct iwl_trans *trans, bool test,
				      bool reset)
{
	if (!reset) {
		/* Enable persistence mode to avoid reset */
		iwl_set_bit(trans, CSR_HW_IF_CONFIG_REG,
			    CSR_HW_IF_CONFIG_REG_PERSIST_MODE);
	}

	iwl_disable_interrupts(trans);

	/*
	 * in testing mode, the host stays awake and the
	 * hardware won't be reset (not even partially)
	 */
	if (test)
		return;

	iwl_pcie_disable_ict(trans);

	iwl_pcie_synchronize_irqs(trans);

	iwl_clear_bit(trans, CSR_GP_CNTRL,
		      CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
	iwl_clear_bit(trans, CSR_GP_CNTRL,
		      CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	iwl_pcie_enable_rx_wake(trans, false);

	if (reset) {
		/*
		 * reset TX queues -- some of their registers reset during S3
		 * so if we don't reset everything here the D3 image would try
		 * to execute some invalid memory upon resume
		 */
		iwl_trans_pcie_tx_reset(trans);
	}

	iwl_pcie_set_pwr(trans, true);
}

static int iwl_trans_pcie_d3_resume(struct iwl_trans *trans,
				    enum iwl_d3_status *status,
				    bool test,  bool reset)
{
	struct iwl_trans_pcie *trans_pcie =  IWL_TRANS_GET_PCIE_TRANS(trans);
	u32 val;
	int ret;

	if (test) {
		iwl_enable_interrupts(trans);
		*status = IWL_D3_STATUS_ALIVE;
		return 0;
	}

	iwl_pcie_enable_rx_wake(trans, true);

	/*
	 * Reconfigure IVAR table in case of MSIX or reset ict table in
	 * MSI mode since HW reset erased it.
	 * Also enables interrupts - none will happen as
	 * the device doesn't know we're waking it up, only when
	 * the opmode actually tells it after this call.
	 */
	iwl_pcie_conf_msix_hw(trans_pcie);
	if (!trans_pcie->msix_enabled)
		iwl_pcie_reset_ict(trans);
	iwl_enable_interrupts(trans);

	iwl_set_bit(trans, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
	iwl_set_bit(trans, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	if (trans->cfg->device_family >= IWL_DEVICE_FAMILY_8000)
		udelay(2);

	ret = iwl_poll_bit(trans, CSR_GP_CNTRL,
			   CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
			   CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
			   25000);
	if (ret < 0) {
		IWL_ERR(trans, "Failed to resume the device (mac ready)\n");
		return ret;
	}

	iwl_pcie_set_pwr(trans, false);

	if (!reset) {
		iwl_clear_bit(trans, CSR_GP_CNTRL,
			      CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
	} else {
		iwl_trans_pcie_tx_reset(trans);

		ret = iwl_pcie_rx_init(trans);
		if (ret) {
			IWL_ERR(trans,
				"Failed to resume the device (RX reset)\n");
			return ret;
		}
	}

	IWL_DEBUG_POWER(trans, "WFPM value upon resume = 0x%08X\n",
			iwl_read_prph(trans, WFPM_GP2));

	val = iwl_read32(trans, CSR_RESET);
	if (val & CSR_RESET_REG_FLAG_NEVO_RESET)
		*status = IWL_D3_STATUS_RESET;
	else
		*status = IWL_D3_STATUS_ALIVE;

	return 0;
}

static void iwl_pcie_set_interrupt_capa(struct pci_dev *pdev,
					struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	int max_irqs, num_irqs, i, ret;
	u16 pci_cmd;

	if (!trans->cfg->mq_rx_supported)
		goto enable_msi;

	max_irqs = min_t(u32, num_online_cpus() + 2, IWL_MAX_RX_HW_QUEUES);
	for (i = 0; i < max_irqs; i++)
		trans_pcie->msix_entries[i].entry = i;

	num_irqs = pci_enable_msix_range(pdev, trans_pcie->msix_entries,
					 MSIX_MIN_INTERRUPT_VECTORS,
					 max_irqs);
	if (num_irqs < 0) {
		IWL_DEBUG_INFO(trans,
			       "Failed to enable msi-x mode (ret %d). Mointerrans=gt(trans,~o75Gi = 0;a"r=HFP=Wn
	 * r=xlloc_va"r=gewX5HtxmEt %d).SR_INT, 0xFFFFFFFF);

	/*
	 * Ue pcitop_rfkill(tr/*
	 * e smallans->op_m
	int first_ailed to enabce going dl_pcie_rx_iOS defaults fe_REG interrupt ahan);
			ud kildist->ent

	/*
	 * he causUSE;
in thiamd map them to the causfoDon't:

	/*Ond map them tmode:cie->rx*
	 * heUSE;
	int reFBQl
	 * swoG interrupt mode:cie->rx*
	 * heUSE;
	int reFBQble_iRSSl
	 * Mo;
in an)twoG interrupt:s needed ite_pe_REG;
	upcie_e	 * restart.six_entriea"r=HFP=Wn inter{ne_cpus() + 2, _TRANS( */
	val = BIns,six_entri+ 2(trs_pcie *trans_pcie = IWL_TRAN=;

	val = MSIX_FH_INT_bit(trrans, CSR_MSIX_RX_IVAR(0;  (trans_pcieo75Gi = 0;a"r=HFP=Wn_va"er{ne_cpus() + 2, _TRANS( */
	val = BIns,six_entr(trs_pcie *trans_pcie = IWL_TRAN=;

	val = MSIX_FH_INT_t(trans, CSR_Gcpus() + 2, _TRANS( */
	val = BIns,six_entri- 2(tran	_PCI_ON(cpus() + 2, _TRANS( */
	val = BIn>ed)
		goto enable_msi;

). Mointerrans(trans_pcins,six_entr(trs it after this call.
	 in_rfkill;e
	 * harx mode (re:f (transmax_irqs; i++)s_pci_write32(trans, Cpci_ter(&pmsix_pcie-"max_irqs; i++)TRANSent i;
			}s_pcnableeturn -ENOx_lock( interrup: hwstop,w/aD3 imamax_
	IWLE;
		rtivids_pcie-l_t_COMMANTATUL_TRANSe {
		iwlL_TRANSrphl_t_COMMANT pciX_REG_VALans_pciL_TRANSrp= ~l_t_COMMANT pciX_REG_VAL;_pciL_TR_event_t
		rtivids_pcie-l_t_COMMANTATL_TRANSe {
	}t(trans, causes_list[i].maskrq_l		*safcan trn value, which is >= 0 if successpara
	valtrans_pc,iwl_tu32, chuntrans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	strk = ~iwl_read32_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	u320r=g2(trpara
	valit_msix(struct iwl_tS( */
	val = BInva"r+	for	   &p_add"r+	for	max_i 0;
	elara
	vali non-RX andNIT_DONEGans, WFirstprierruDE_DRVpla00);
		rn chT_DONE(i.e.is toggl cause, >0;
va"eN delimiteirst_FirsAR(canext(il_write8(,iwl_>cfg->mqAR(crans->rsAR(ca		*s) >>wl_turans_pcie =
	afcan trqAR(c);
			brFLAG_MAq_l		*safcan tr_h and retufor (i = 0; i < trans_pcie->ank_sz = PAGEans_pcie =
	afcan trqAR(c);
			bre_load_cpu_se_tx_reset(trstruct iwl_tans);

		ret = iwstatufcan tron't wmax_ANSr;
			}
			bievice(trans, tru

statcpci *l = B_na
	iwl_pciehw_rfkileturn 0;rst_airans);
}

void iwl_pcie_synchr,s_pciee_d3_susp_synchr32_pcie = IWL_TRAcie = IWL_vUG_FW_synchr32_pcie = IWL_TRAs_pcie _GET_PCIE_TRANS(trans);
	u32 offset	bre_loi0;a"0cpu_sectionsDRV_NAME ":eUSE;
	iANS"_REG_FLAionsetum_kasprintf(pcie-	v_addr = tic void iwlDRV_NAME ":eL_SHAR%d	}sir+	 IWHW, &trans-i0;a"0cpu_ectionsDRV_NAME ":eCAUSE;
	L_SHA"ctions(tr0;a"_synchr32(trans_pcin-code_sectionsDRV_NAME ":eexceps al"	*status = etum_kasprintf(pcie-	v_addr = tic vid iwlDRV_NAME  ":eL_SHAR%d	}si/
		iwl_trans_pcie_txuses(trans);

tus);
	r_STATUS_ALIVE;

	return 0;
rst_airans);
}

void iwl_pcie_synchrns);
}

IWL_TRANSGET_PCIE_TRANS(trans);

	if (trans_pcie->msiie = IWL_RR(tranrans);
= 0; i < y *= 0; i < yans->
statcpci *lna
	IE_l = B_na
	i&pmsix_pcie-rans);

	i}si/
	cie_rx_slna
	nt(trans->dev, chunk_
0;

0; i < y =GEans_pcie =
	= 0; i < trans_		brFLAG_Metum_;
			ud_th
	IWe =ize_&pmsix_pciek_sz =;

0; i < yIWL_e->ank_sz =s bound to 
0; isank_sz =str0;a"_synchrrrans=gt(tra
	 k_sz =s bound toAq_l);

tus);
	r :k_sz =s bound toAq_l	va);

tus);
	rnk_sz =sANSF_PCIE_Tnk_sz =slna
	ek_sz =;

0; i < ye {
		iwl_trans_pcie_tx_reset(trstruct iwl_tans);

Errorlans->opts */NSr;
			}si/
	cie			IWL_ERR(trans,
			t[i].maskrq_l		*safcan trnet(trstruct iwl_t)	*status = IWL_D3_STATUSIWL_g->gen2)
			_iwl_tart_/* Note: returns standard l);
}

static void iwl_trans_pcie_stop_device(struct iwl_trans *trans, bool low_power)
{
	sIWL_ters_pcie __maybe_unused *trans_pcie =
		IWL_TRANSterG_MAC_ACCESSnership to prevent other_load_rr	   CSR_GP_CNTRL_REG_FErrorlt havenershin cert:r;
			}s_rr	 < 0) {
		Iters_p_xtal_workaround) {
		iwl_pcie_aterG_MAC_ACCESS

	/* nic_init */
load_rr	< 0) {
		Iters_		t[i].maskrrans);

	pmode actuall8000(Fon ilso L_DEcall the op_ cause, kepsabled, dCK, utbool was_in_rfk * Even if we stop the HW, we still RANS(trans);
	bool was_in_set += ch/l WFPMans_pcie iwset + theirso	 * w... * ERANS(trans);ans_pcie->set += ch/l ...x_lock(while if the rkfill s	if (e  CSRset + loaence mait up, only whate since we may have miher
	 * interruSTATync theil);ewrite_we'_pcieursiu_pcfh(traading,
restart.static voidciLm_;un* Ce;
	}

	iiwl_tS(pci_wrtrans);

	/* release CPU resetn2)
			_iwl_tart_/* Note: returns standard l);
}

static void iwl_trans_pcie_stop_device(struct iwl_trans *trans, bool low_power)
{
	sIWL__pcie *trans_pcie = IWL_TRANS_GET_PCIE_TFLAG_Mg->gen2)
			_iwl_tart_/* dard l)_RFKILL_OPMODw_power);
	iwl_trans_pcie_handle_sin_rfkill)
		ret = -ERFKILL;

out:
	mutex_un RF_KILLleatrans_pcie->mutex);
	retur_hw(trans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	stake sure it finished running */
	iwl_t be re ict table in
t TX qrn -ENOrtain verions of the e the interrupts to make sure no otheACCESS_REQ);

	/* Stop thIWL_D3_STe interrupts to make sure no otheACCESS_REt be reset (not even pw_power);
	iwl_trans_pcie_handle_sinartially)
	 */
	if (test)
		returnet = -ERFKILL;

out:
	mutex_un

	if (ns_pcie->mutex);
	return ret;of	retur& IWhw(t

	ifb(transirqs(struct iwl_trans *trans)
{32(tr2_toMPM_Cturnet = -ERFKILL;

out:
	mutex_un

	if32(ns_pcie->mutex);
	return ret;of	retuct iwWhw(t

	ifl(transirqs(struct iwl_trans *trans)
{32(tr2_toMPM_Cturnet = -ERFKtuctll(trans, was_inresumens_pcie->mutex);
	return ret;of	Whw(t_rfkill)
adl(irqs(struct iwl_trans *trans)
{32(tr2_toMPM_Cturnet = -ERFKtuctll(trans, was_inresL_DEBUns_pcie->mutex);
	return ret;t->ec void i:
	mutex_un

	if32(return HBtus)AR		iwPH_R*/
		iwl_write_p(l_tgNS_0x000n), th/
		3ite_24))n_in_rfkillll(trans, was_inresumereturn HBtus)AR		iwPH_RDATurnet = -ERFKILL;

out:
	mutex_un

	ifL_DEBUns_pcie->mutex);
	return ret;section_num);
 etuct iwWhw(tid i:
	mutex_un

	if32(return HBtus)AR		iwPH_W*/
		iwl_write_p(lr);

	i0x000n), th/
		3ite_24))n_inid i:
	mutex_un

	if32(return HBtus)AR		iwPH_WDATk & IWL_SHA= -ERFKILL;

out:
	mutex_unE;
		retu_tx_start(trans, scd_addr);
}

voidtruct iwl_trans_p:
	mutE;
		rie_synchcfg_hw(trans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	st_trans_pcie_cmdal = B_FW_synchtranscmdal = B;st_trans_pcie_cmdafifo_FW_synchtranscmdafifo;st_trans_pcie_cmdaq_wdgnt);
	iw_FW_synchtranscmdal_wdgnt);
	iw*/
load_PCI_ON(cpus()transn_noinrclaimRANSIn>e	gotNval CLAIMiwl_S)ine_cpus() + 2, n_noinrclaimRANSIn = 0;
	u32 lacpus() + 2, n_noinrclaimRANSIn =cpus()transn_noinrclaimRANSI;3_susp_synchr+ 2, n_noinrclaimRANSIine_s_prph(_synchr+ 2, noinrclaimRANSI,=cpus()transnoinrclaimRANSI,t first_aicpus() + 2, n_noinrclaimRANSIn* _ADDof(u8){
	st_trans_pcie_	vabufz) {
		ucpus()trans	vabufz) {
;st_trans_pcie_	va PAG_ordevi=R_GP_CNTRL,
	t->vrbz) {
_ordev(_trans_pcie_	vabufz) {
{
	st_trans_pcie_bc_ true_divid		ucpus()transbc_ true_divid;st_trans_pcie_32 s		*sact* al	ucpus()trans32 s		*sact* a;st_trans_pcie_3w_c}

_txl	ucpus()trans3w_c}

_tx
	st_trans_pcie_ PAG_offIn =cpus()transcb_ On _offI;). Mointerrans=gvRANS_offIn =cpus()transcb_ On _offIMPM_ADDof(ILL;
*{
	st_trannsceue - _groupIn =cpus()transceue - _groupI;st_trannsceue - _groupIz) {
		ucpus()transceue - _groupIz) {
/
	iwl_pranial {
	NAPI their- CSR * the timcie->iss, falseEBUG_INFac80211IF_CONFIGen as
	 * bu't know weNOrtarans);
}slans->op_me of MSsens during firkill)the firmware loa
			 doncorne. Th* hedet(trans)dos	iome of susNAPI ns-> due to iranial {

	 * restart. pus() + 2, napIVE;
.		dein_rfk!= NETPM_GDUMMY*/
	iransdummy_netE;
wl_trans_pcie_napIVE;
e_stop_rfkill(trans, was_i the  causes_list[i].cause_num);
	}
}

static void iwl_pcie_map_rx_causes(struct iwl_trans *trans)
{
	sns);
	consrtially)
	 */
	if (test)
		return;

"enabled");
	if (iwl_opsrtially)transtxi the ans)
{
	s	u32 latruct iwl_tr the ans)
{
	sAC_ACCESS_RE the ans)
{
	start. pus() + 2, rba.(transwqans, Cpc	}
oytivikl = B. pus() + 2, rba.(transwqa(trs_pcie *transrba.(transwq_caNUhw_rf}	start. pus() + 2, cie *trans_pcie = GET_PCIE_TRANS(trans);

	if (trans_pcie->msiie = 	Aq_l		*safcan tr_h and
rph(trans, MON_B= 0; i < trans_pcie->ank_sz NUhwe {
	}tne_cpus() + 2, IWL_MAXl.
	 in_set += trans, CSR_GP_CNwas_i themsix_hw(trans__xtal_workaro themns->cfg->device_)RANSGET_eachKILssirue_) >>isupport}
}

staticso_hdra PAG *pi=R_G	per

	/*ptv(_trans_pcie_cso_hdra PAG}si/
	cie_rx_pe_ PAGl_op_mo them PAG_pe_ PAGlns__xta them er) >>_trans_pcie_cso_hdra PAGPMODw_powepc	}
oy = IWL_TRANS_GET_PCIE_Ttatic void the ans)
{
	rfkill != was_in_rfkill)
		iwl_d tomil);
	mutex_unlock(&trans_pcie->mutex);
}
_rx_mutex);ans);
	if (hw_rfk);
	clear_bit(STATUS_INT_ENABs	u32 laE, &trans->status);
	clear_bit(STATUS_INT_ENABrfkill != sections_kill)
		iwlgraberrupfh(tra, p_addr);
	return ret;
}

static un2 va	 ilo	 */flagsucode_8000(suntrans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	stspinure itest)atra= IWL_TRANS_G		dere i,*/flagsu
	start. pus() + 2, ANS_holderrupf		*ss, num_ir	iw*/LE, 0)s du&transk heuclear_NIC rest_g->gen2)
			_iwl_it(trans, CSR_GP_CNTRL, CSR);
}

);
	iwl_enable_interrupts(trans);

	iwl_swl_set_bit(trans, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	if (trans, URs	iwl_ensay* The IVAR tablred.
	 ce the * the s a bred.
	 *x;

	/*
X causes ae * rtlt have(X causes asilo	 *asipts(trans);

	}

	iwl1)n;
	}
	/*
 Thy)dostingindThise	 * wormbede maSRAMtablrc	}
	 * y(sunt	}
rtant revoll !aveSRAMtmuct iatr/rc	}
	 dtrut repreo/ion rrupts;
	}DRAMt wil sl a 
	 /s interGET_ILL_O-sware).s))
		iwl_32(trafirktsk heapproximiseNTR1/4 _CNlit nibb;ant re)s douldn'ver;
	i}sit'on sgoo_indeaph(trrabprovidoldipts(trans);

	Udbg_swlarare
	e tra_enas, falsecfh(trais alwa;

	if * (e.g.wl_read_pEupt
	Log)n;
	}
t is a bkfill state sl a 
	 .s))
_transSR_UCODE_DRV_GP1as, falsec&trapts(SLEEP0;a"0gindThise, WFPMinal WRAMtablokay/rc	}
	 d. fkilt TX qns, fw * wotheir;ewrite_we're wakIF_CONs juct GET_s(trans);s, falsecfh(tra;
	/*
GP1apts(SLEEP

	/*
s, fwion sgoo_indeapcie->isfh(tra_int(traWRAMtof
rtant r

	/*voll !aveSRAMt(e.g.wl_read_pEupt
	Log).s))
_tranMAC_
	e tra_ loading,
(incluead_p1AC_
	e tra)ithouteastvoll !aveSRAM,mware loadostingiatr/rc	}
	 dSRAMt wil RL_REGcycl
	 .s))
/f (trans->cfg->device_family >= IWL_DEVICE_FAMILY_8000)
		udelay(ed) pts(trans);ENCE_FAMIL(ns, CSR_GP_CNTRL,
			   CSR_GP_CNTRL_bit(trs, false);

	if (!reset) {GOt ==TO(SLEEP), 1MAC_CLOCK_RE exactlyEADY,
			FH_INT_CAUSES_Q

	iwl_enable_interru(trans, WFPM_GP2));FORNTRNMIe {
	_PCI_ONCE(1CE_FAMI"T);
	iw_waitnterGET_s(trans);fh(traa(ns, CSR_GP_Creturnx)
			      upon resume = 0x%08X\nCSR_GP_Ctrans,spinur);
	itestrc	}
	 a= IWL_TRANS_G		dere i,*/flagsu
	 0) {
		Iset += trw_rf_kil(trans,Fie->mpars	iw_TRA
{
	inAL_DMA_CLt(trare ite bpars	iwry rare
 IW itrrupfh(tras	ioway.s))
/f ___DMA_CLa= IWL_TRANS_G		dere in_in_rfkillrfkillrfkill != was_in_rfkill)
		iwl_DMA_CLerrupfh(tra, p_addr);
	return ret;
}

static ic un2 va	 ilo	 */flagsucodel_trans_pcie_stop_device(struct iwl_trans *trans, bool low_power)
{
	struct iwl_trans_pcie *trans_pcie =		dere in_iil(trans,Fie->mpars	iw_TRA
{
	inALacquir_int(trare ite bpars	iwry rare
 IW itrrupfh(tras	ioway.s))
/f __acquirLa= IWL_TRANS_G		dere in_istart. pus() + 2, ANS_holderrupf		*ss, num_ir	iw*/LE_g->gen2)
			_iwlturn ret;
	}

	iwl_pcie_set_pwr(tr(trans, CSR_GP_CNTRL,
			   CStrans);

	iwl_int val , UREwrite_pt(trans, CSR_GP_Crs, false ? 1 : 0eded tlushmware lyterruiousigureds_pciw_w				  t(tragured a* woturn ven
	 * bypts(trans);

	}&tra");
		 ernchredpcie->isflyt */
	iguredsrare
	
s,du
	 iirkdist->enttionaa( know esetrop 		dere in.s))
/f mmiowb(iwlrf_kilspinur);
	itestrc	}
	 a= IWL_TRANS_G		dere i,*/flagsu
	 release CPU resetn2)
			_iwlnresLmemUns_pcie->mutex);
	return ret;section_num);ILL;
*buf,s_pcidividsucodeun2 va	 ilo	 *flags
	sns);offI,00(struct iret;*vet trubufctions(tr_CNTRL,
	traberrupfh(tra,return &flagsuFH_INT_CAUSES_Q

	iwl_enaHBtus)AR		LMPMR*/
		BIT(%u) inGET_PoffIn =TRAoffIn<idividsRAoffInline_ vals[offI]ue upon resume = 0x%0HBtus)AR		LMPMRDATurn_GP_CNTRL,
	_DMA_CLerrupfh(tra,return &flagsu= trans, CSR_Gwl_pcie_BtuYW, &tr_rfkill)
		ret = -ERFKPU resetn2)
			_iwl

	ifLmemUns_pcie->mutex);
	return ret;section_num);dtruct ILL;
*buf,s_pcidividsucodeun2 va	 ilo	 *flags
	sns);offI,00(struct itruct ret;*vet trubufctions(tr_CNTRL,
	traberrupfh(tra,return &flagsuFH_INT_CAUSES_Q

	iwl_enaHBtus)AR		LMPMW*/
		BIT(%u) inGET_PoffIn =TRAoffIn<idividsRAoffInline_ _CAUSES_Q

	iwl_enaHBtus)AR		LMPMWDATkon_num);dvet t? vals[offI]u: 0urn_GP_CNTRL,
	_DMA_CLerrupfh(tra,return &flagsu= trans, CSR_Gwl_pcie_BtuYW, &tr_rfkill)
		ret = -ERFK_rfkill(trans, was_i thezwl_tqnt);
r, p_addr);
	return ret;
}

static iun2 va	 ilo	 *_tq
}

static isecti thezwum);
	}
}

static void iwl_pcie_map_rx_causes(struct iwl_trans *trans)
{
	sns);l = B;sNSGET_eachK_it(tranT_RSS ?&_tq
}SIX_S	iwl_LONGsupport}
}

staticxq_pcxlit_msix(struct ixq[T_RSS_		brun2 va	 ilo	 *now
	ciespinure itbh(&_tq->re in_ist	lso = jisttra
	cie_rx__tq->tatzil ;a" thezwum	 num_irnext_l = B;sNSHtxmEt %d).TXble_msiTRL_REG_F%s TXSr;
			}
			bic i thezw>mutFthez_inWL_WAWA
{
	",;l = Bn_ist	_tq->tatzil ;i thezw
	cie_rx__tq->nresL_tr0;a"_tq->

	ifL_trum	 num_irnext_l = B;sNSH_rx_ thezwuie = 	A_RE exactlyE* Ce; know(lsoek_sz =;_tq->stu itt);
r.;

irLsuFk;
		casNIT_D	ans, UREt);
re * the thoutg. Di, kill( CSR_sT_D	ans,spin.
	 *ron wilso L_t(trare i.T_D	ans/T_D	aum_irnext_l = B;s	1, stop00(trmrmbeT_sso lo	 *un* le.
	 * Certg. Dwl_pcie	_tq->tatzil_;

iry	_Dmaindevi=R_G=;_tq->stu itt);
r.;

irLsin
	 w;s	1,delnt);
r,&_tq->stu itt);
rt) {
	um_irnext_l = B;s	1}tne_NIT_DONEWisablteastemptyWL_SHARE> alm * Certnt re)sthe D3 _Dmaindevicie->isCSRsatzi
	ans/T_D rett);
r,&_tq->stu itt);
r	      lso + _tq->tatzil_;

iry	_Dmaindevn_isnext_l = B:ns,spinur);
	itbh(&_tq->re in_ice(trans, true);
		else
			_iwl_tb;
	it_tqn_tra, p_addr);
	return ret;
}isectib;
	ium);
	}
}

static void iwl_pcie_map_rx_causes(struct iwl_trans *trans)
{
	sns);
	conGET_PCIE_TRANS(trans)it(tran2_tot;
	}

S( */
ofal = BIT(Mmsiie = t}
}

staticxq_pcxlit_msix(struct ixq[s_			bre_loi0;a"_trans_pcie_cmdaq = Bn    trutin B;sNSHspinure itbh(&_tq->re in_ist	_rx_sb;
	irupt d_PCI_ON_ONCE(!_tq->b;
	iuFk;
		ca_tq->b;
	i--; = 	A_RE!_tq->b;
	iu;
		cas_CAUSES_Q

	iwl_enaHBtus)AR		WRPTR}

static i_tq->

	ifL_tr/
		iite_8trans,, sto(trans_pcieb;
	iu;
		ca_tq->b;
	i++;s	1}tne_spinur);
	itbh(&_tq->re in_ice(tra#defineausesFLUSH_W*IT_MS	2AC_op_rfkill(trans, was_ilog_32 serror, p_addr);
	return ret;
}it}
}

staticxq_pcxlucodeuet;_tqnid		uctq->idt iret;_INT_E;	structact* a;stu8 fifo;sl_swl_set_bit(tran_cletfh	   CSR_GP_CNTRL_REG_FQ_SHAR%dcalletu iR%dc;
			}s_tqnid,		ca_tq->nresL_tr,"_tq->

	ifL_tru;ne_NI TODO:;fh(traanew SCDns, false); loadumON_AUTORE_CPU2_rite_prp;
	}SET_REG
	}

	IWL_DEBUG_POWERSCDble_msNEVO_RESIX_S(_tqnidher (Nifo_FW(	}SET_R>>RSCDble_msNEVTUSES_REOus)XF)
	i0x7r (act* al	u!!(	}SET_R&SIX_RSCDble_msNEVTUSES_REOusts(tra){
	ste_tx_reset(trans)FQ_SHAR%dcall%sact* alL_tNifo_%dc the tu iRGET_%u ms. SW [%i, %d]
rta[%i, %d]
FH TRB=0x0%x			}
		_tqnid,tact* a>mutWL_WAi		}sNifo}
		jisttra_to_msecs__tq->wdnt);
	iw),
ca_tq->nresL_tr,"_tq->

	ifL_tr,
	}
	}

	IWL_DEBUG_POWERSCDble_msNRDPTR(_tqnidhes_pcie(TFDble_msNEIZE		gon-cod,
	}
	}

	IWL_DEBUG_POWERSCDble_msNWRPTR(_tqnidhes_pcie(TFDble_msNEIZE		gon-cod,
	}
	}

	IWL32(tra

	iwl_ena the ansBSES_(Nifo))n_iet = -ERFKPU resetn2)
			_iwl
aitt_tqnempty, p_addr);
	return ret;
}iPU r_tqnidxum);
	}
}

static void iwl_pcie_map_rx_causes(struct iwl_trans *trans)
{
	st}
}

staticxq_pcxl;deun2 va	 ilo	 *lso = jisttra
	tu8 wrL_tr_GET_PCIE_ans_pcie_tqnidxe->msix_entriel = B_uct )	< 0) {
		I-EINVALwX5HtxmEt %d).TXble_msiTRL_REG_FEmpty
	 *L_SHAR%d...			}s_tqnidCIE_TRxlit_msix(struct ixq[_tqnidC_		bwrL_trit_trans);ONCE(_tq->

	ifL_tru;n	bw have(_tq->nresL_tr0!t_trans);ONCE(_tq->

	ifL_tru

	ifirst_ai!* Ce; know(jisttraCE_FAMILlso + msecs_to_jisttra(usesFLUSH_W*IT_MSuFk;
		cu8 wr	ifL_tr/t_trans);ONCE(_tq->

	ifL_tru;n	b
load_PCI_ONCE(wrL_tri		cl
	ifL_tr,
	}first_a"WR po to m movw_rfkill tlush
	 *%dcE> ;
			}
			irst_awrL_tr,cl
	ifL_tr)nt(trans->dev,TIMEDOUT		brusl a 		trans1AC_, 2AC_CLOC}	start. tq->nresL_tr0!t__tq->

	ifL_tru
  CSR_GP_CNTRL_REGt(tr"RANS_ iwslush/*
	 *xtNifo_l = BInQc;
			}s_tqnidxurn_GP_CNTRL,
	was_ilog_32 serror,RL_REG_cxlu;
trans->dev,TIMEDOUT		b}X5HtxmEt %d).TXble_msiTRL_REG_FQ_SHAR%dcalllso empty.			}s_tqnidCIE_trans);

	/* release CPU resetn2)
			_iwl
aitt_tqsnempty, p_addr);
	return ret;
}iuet;_tqnbmum);
	}
}

static void iwl_pcie_map_rx_causes(struct iwl_trans *trans)
{
	sns);cnt
	sIWL__pctruct LE, 0waitnterGET_F kill i*xtNt RXsqueupletation witisabltfkill ,
	 * thecntIE_TRAcntI(trans)it(tran2_tot;
	}

S( */
ofal = BIT(cntmsiie 	b
loadcntIEa"_trans_pcie_cmdaq = Bn    trutin B;s	T_PCIE_ans_pciecnte->msix_entriel = B_uct )	< 0 trutin B;s	T_PCIE(IX_RcMG_C&;_tqnbmun    trutin B;sNSH(trans->cfn2)
			_iwl
aitt_tqnempty,ret;
}icMG_;s	T_PCIad_cpu_sbnrek;cie->opmfkill)
		ret = -ERFK_rfkill(trans, was_i*/
	if (t_TRAUns_pcie->mutex);
	return ret;t->}

statit);

	reretuct iwuwum);
	}
}

static void iwl_pcie_map_rx_causes(struct iwl_trans *trans)
{
	sun2 va	 ilo	 *flags
	stspinure itest)atra= IWL_TRANS_G		dere i,*flagsu= t_g->gen2)
			_iwl_it(tra(t_TRAUreturn t->}

	rere iwuwu;ilspinur);
	itestrc	}
	 a= IWL_TRANS_G		dere i,*flagsu
	 release Cwas_in_rfkill)
		iwl_Df/
	iwl_pcie_prepare_card_hw(trans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	struct}

wifiF_KIt;
	}

.d0i3Et be red the
	 * hardLm_;un* Ce;geta= IWL_TRANS_Gpe->msix_pci_wrt#ifdef reset *PM5HtxmEt %d).RPME_TRANS(trun* Ce usPAG coun*: ;
			}
		irst_aatomic

	IWa= IWL_TRANS_Gpe->msix_pci.RL_RE.usPAG_coun*)n_i#g inf , 0reset *PM ,
	 release Cwas_in_rfkill)
		iwlun_Df/
	iwl_pcie_prepare_card_hw(trans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	struct}

wifiF_KIt;
	}

.d0i3Et be red the
	 * hardLm_;un* Ce;mark_lans_pusya= IWL_TRANS_Gpe->msix_pci_wrdLm_;un* Ce;put_he bs_pcie_s= IWL_TRANS_Gpe->msix_pci_wrt#ifdef reset *PM5HtxmEt %d).RPME_TRANS(trun* Ce usPAG coun*: ;
			}
		irst_aatomic

	IWa= IWL_TRANS_Gpe->msix_pci.RL_RE.usPAG_coun*)n_i#g inf , 0reset *PM ,
	 release C>
statcpci *t->vcsr_rannte(ns);cmd_hw(#defineausesCMD(x)_pcie_x:is toggl#x(trinterr(cmd_
  CusesCMD( to avoid reset */
	_wrdusesCMD( to 
	iwlOALESCIN	_wrdusesCMD( to 
	i_wrdusesCMD( to 
	iwmask_wrdusesCMD( to )
		retEVO_RE_wrdusesCMD( to GPIO		r_wrdusesCMD( to ",
			iwlusesCMD( to GPR_GP_Ct; CusesCMD( to avoREVt; CusesCMD( to EEPROM*/
	_wrdusesCMD( to EEPROM*GP_wrdusesCMD( to OTP GPR/
	_wrdusesCMD( to GIOR/
	_wrdusesCMD( to GP_UCODE_/
	_wrdusesCMD( to GP_DRtraR_/
	_wrdusesCMD( to UCODE_DRV_GP1_wrdusesCMD( to UCODE_DRV_GP2_wrdusesCMD( to LED_/
	_wrdusesCMD( to DRAM		retTBLR/
	_wrdusesCMD( to GIORE_ENAENSIX_S_wrdusesCMD( to ANA_PLesCF	_wrdusesCMD( to avoREV_W*R/
	_wrdusesCMD( to MONITOR_RESET);
		_wrdusesCMD( to DBG_HP iwLMPMR		_wrdCAUSE;
:
trans->de"UNKNOWN";cie-#undef usesCMDr users from stealdumOvcsrn value, which is >= 0 if successp;;
	}SEe C>
stattuctcsr_tbl[]ue   CS
		/* Enable persistence to 
	iwlOALESCIN	ence to 
	ience to 
	iwmaskence to )
		retEVO_REence to GPIO		rence to interrnce to GP_set_pwr(t to avoREVwr(t to EEPROM*/
	wr(t to EEPROM*GPwr(t to OTP GPR/
	rnce to GIOR/
	rnce to GP_UCODE_/
	rnce to GP_DRtraR_/
	rnce to UCODE_DRV_GP1rnce to UCODE_DRV_GP2rnce to LED_/
	rnce to DRAM		retTBLR/
	rnce to GIORE_ENAENSIX_Srnce to ANA_PLesCF	rnce to MONITOR_RESET);
		wr(t to avoREV_W*R/
	rnce to DBG_HP iwLMPMR		cie;ste_tx_reset(tra " toe iwuws:(ret) {e_tx_reset(tra "(2nd from of  to 
	iwlOALESCIN	call"ns)F to 
	iwPERIODICMR		_(ret) {GET_PCIE_TRANS(t ARRAYNEIZE(csr_tbl)T(Mmsiie = e_tx_reset(tra "  %25s: 0Xurnx			}
			t->vcsr_rannte(csr_tbl[i]);
		}
	}

	IW

	iwl_enacsr_tbl[i])n_ice(tra#ifdef reset *e_tWIFIEt %d)FS
, CS
	Ititch is movw of filDwl_pc#defineat %d)FS_ADD_FILE(na
	ee, tente-
	 *)ados{		}\ET_PCIEdebugfs_S
	Iti_filD(#na
	eeRSIST_, tente-iwl_en	}\ETtati&e intbgfs_##na
	##_ops))	}\ETt msi-xrr;_sz =;\
}lt have(0)

, CfilD  wilis all_pc#defineat %d)FS_NTRL_FILE_OPS(na
	)sz =;\
	}SEe C>
static int iilD_ wilis als e intbgfs_##na
	##_opsue  	}\ET.te_pt= e intbgfs_##na
	##_te_p,z =;\
	.opil ;isiuple_ win,_sz =;\
	.llseeAN=;generic
iilD_llseeA,sz =;\
};
c#defineat %d)FS_WRITE_FILE_OPS(na
	)                                    \
	}SEe C>
static int iilD_ wilis als e intbgfs_##na
	##_opsue            \
	.gured = e intbgfs_##na
	##_gured,                              \
	.opil ;isiuple_ win,_sz =;\
	.llseeAN=;generic
iilD_llseeA,sz =;\
};
c#defineat %d)FS_NTRL_WRITE_FILE_OPS(na
	)z =;\
	}SEe C>
static int iilD_ wilis als e intbgfs_##na
	##_opsue  	}\ET.gured = e intbgfs_##na
	##_gured,z =;\
	.te_pt= e intbgfs_##na
	##_te_p,z =;\
	.opil ;isiuple_ win,_sz =;\
	.llseeAN=;generic
iilD_llseeA,sz =;\
};
c	}SEe Cs) {
_t e intbgfs_tval = B

	IWaic int iilD *iilDr);
}

void icpci __ucti *uctiabufr);
}

void i) {
_t coun*l)_Rff_t *ppod_hw(trans);
}

void ipcie_init_iilD_GprivIti_ On untrans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	st}
}

staticxq_pcxl;decpci *bufctccesspodtruct ins);cnt
	sIWL__pc
	st {
_t bufszhardbufsz ;isiDDof(cpci) * 75re
 IWs)it(tran2_tot;
	}

S( */
ofal = BITGET_PCIE_six(struct ixq_ every	< 0) {
		I-EAGAINhardbuf ;ikz(tran(bufsze-	v_addr = CLOCK_RE!buf	< 0) {
		I-E chunk_
0* thecntIE_TRAcntI(trans)it(tran2_tot;
	}

S( */
ofal = BIT(cntmsiie 	TRxlit_msix(struct ixq[cnt_		brpodt+;iscnprintf(buf +spod, bufsz -spod,);
}
"hwq %.2d:ite_p=%ucl
	if=%ucuct=%!run_i=%!r		  _bled, =%!rtatzil=%!%s			}
			bcnte->tq->nresL_tr,"_tq->

	ifL_tr,
	}		!E_ans_pciecnte->msix_entriel = B_uct )r);
}

!E_ans_pciecnte->msix_entriel = B__downt )r);
}

_tq->		  _bled, , _tq->tatzilr);
}

dcntIEa"_trans_pcie_cmdaq = B>mut erruWL_WA")n_ice(H(transsiuple_nresLtateabuffer(uctiabufr coun*l)ppod, buf,spodn_ick the buf	;>opmfkill)
		ret = -ERFKs) {
_t e intbgfs_rval = B

	IWaic int iilD *iilDr);
}

void icpci __ucti *uctiabufr);
}

void i) {
_t coun*l)_Rff_t *ppod_hw(trans);
}

void ipcie_init_iilD_GprivIti_ On untrans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	scpci *bufctccesspodtructrans_pcie t {
_t bufsz ;isiDDof(buf	;>rdbufsz ;isiDDof(cpci) * 121re
 IWs)it */
	val = BITGET_PCIE_six(struct rxluc 0) {
		I-EAGAINhardbuf ;ikz(tran(bufsze-	v_addr = CLOCK_RE!buf	< 0) {
		I-E chunk_
0* theCIE_TRANS(trans)it */
	val = BIn&&spodt< bufszh(Mmsiie = t}
}

statirxq_prxlit_&_six(struct rxl[s_			brpodt+;iscnprintf(buf +spod, bufsz -spod, "l = B#: %2
			}
			biievicrpodt+;iscnprintf(buf +spod, bufsz -spod, "\t
	IW: %u			}
			birtq->nresevicrpodt+;iscnprintf(buf +spod, bufsz -spod, "\t

	if: %u			}
			birtq->

	ifevicrpodt+;iscnprintf(buf +spod, bufsz -spod, "\t

	if_the de: %u			}
			birtq->

	if_the deevicrpodt+;iscnprintf(buf +spod, bufsz -spod, "\t		  _bled, : %2
			}
			birtq->		  _bled, evicrpodt+;iscnprintf(buf +spod, bufsz -spod, "\t themcoun*: ;u			}
			birtq-> themcoun*_;s	T_PCIatq->nb__dtsans_pciLodt+;iscnprintf(buf +spod, bufsz -spod,);
}
	 "\tcloct vrbz */: ;u			}
			b	 le16_to_) >>atq->nb__dtse_cloct vrbz */es_pcieb	 0x0, then	trans, CSR_GiLodt+;iscnprintf(buf +spod, bufsz -spod,);
}
	 "\tcloct vrbz */: Not Ans->op_m
	ie {
	}tce(H(transsiuple_nresLtateabuffer(uctiabufr coun*l)ppod, buf,spodn_ick the buf	;>>opmfkill)
		ret = -ERFKs) {
_t e intbgfs_tatus = IW
	IWaic int iilD *iilDr);
}
	cpci __ucti *uctiabufr);
}
	) {
_t coun*l)_Rff_t *ppod_hw(trans);
}

void ipcie_init_iilD_GprivIti_ On untrans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	srans);
}srein_risERFipc}srein_rsit_&_six(struct }srein_rsTGET_esspodtruct icpci *bufctccessbufsz ;i24re
64; , 024r	ifmsre
64icpci will	ifm ,
	 s) {
_t _pcie *buf ;ikz(tran(bufsze-	v_addr = CLOCK_RE!buf	< 0) {
		I-E chunk_
0Lodt+;iscnprintf(buf +spod, bufsz -spod,);
}"Is of the Sn_risERFipRans);:(ret) 
0Lodt+;iscnprintf(buf +spod, bufsz -spod, "HW Error:\t\t\t ;u			}
		}srein_rs32(t_wrdLodt+;iscnprintf(buf +spod, bufsz -spod, "SW Error:\t\t\t ;u			}
		}srein_rs32swCLOCK_RE}srein_rs32sw || }srein_rs32(t_CSR_GLodt+;iscnprintf(buf +spod, bufsz -spod,);
}"\tLses Re_tartnterCod : retu ret;
		}
srein_rs32of _loadn_ice(#ifdef reset *e_tWIFIEt %d)rdLodt+;iscnprintf(buf +spod, bufsz -spod, "Ft RXtrans)mitp_m:\t\t ;u			}
		}srein_rs32sch_wrdLodt+;iscnprintf(buf +spod, bufsz -spod, "A(&tr( interrup:\t\t ;u			}
		}srein_rs32k(&trn_i#g inf
0Lodt+;iscnprintf(buf +spod, bufsz -spod,);
"rtain irqs(trans); which :\t ;u			} }srein_rs32(trans, lrdLodt+;iscnprintf(buf +spod, bufsz -spod, "CT irqs:\t\t\t ;u			}
		}srein_rs32ctrans, lrdLodt+;iscnprintf(buf +spod, bufsz -spod, "Wisaup Iinterrup:\t\t ;u			}
		}srein_rs32wisaup, lrdLodt+;iscnprintf(buf +spod, bufsz -spod,);
"Rx ceue - lrc	ponsws:(t\t ;u			} }srein_rs32(x, lrdLodt+;iscnprintf(buf +spod, bufsz -spod, "Tx/FH( interrup:\t\t ;u			}
		}srein_rs32tx, lrdLodt+;iscnprintf(buf +spod, bufsz -spod, "Un;

	if * INTA:\t\t ;u			}
		}srein_rs32unus);
	d	;>>opmfanssiuple_nresLtateabuffer(uctiabufr coun*l)ppod, buf,spodn_ick the buf	;>opmfkill)
		ret = -ERFKs) {
_t e intbgfs_tatus = IW

	ifaic int iilD *iilDr);
}
	C>
statcpci __ucti *uctiabufr);
}
	i) {
_t coun*l)_Rff_t *ppod_hw(trans);
}

void ipcie_init_iilD_GprivIti_ On untrans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	srans);
}srein_risERFipc}srein_rsit_&_six(struct }srein_rsTG	ret;t-_it(flag
	sIWL__pc
	>opmfanskrantoretLtateaucti(uctiabufr coun*l)16, &t-_it(flagCLOCK_REad_cpu_		IWL_ERR(traNTRL_REG_(flag0;a"0cpu_mfmsetE}srein_rs,uctrsiDDof(c}srein_rs)	;>>opmfkillcoun*	ret = -ERFKs) {
_t e intbgfs_csr_

	ifaic int iilD *iilDr);
}
);dtruct cpci __ucti *uctiabufr);
}

vo) {
_t coun*l)_Rff_t *ppod_hw(trans);
}

void ipcie_init_iilD_GprivIti_ On untal_workarodumOvcsrnans)
{
	stpmfkillcoun*	ret = -ERFKs) {
_t e intbgfs_fh_		de
	IWaic int iilD *iilDr);
}

voidcpci __ucti *uctiabufr);
}

void) {
_t coun*l)_Rff_t *ppod_hw(trans);
}

void ipcie_init_iilD_GprivIti_ On untcpci *buf_caNUhw_rfs) {
_t _pcie *(trans->cfdumOvfBUG_POWER&buf	;>oK_READY,
			pu_		IWL_ERR(traNTRL!buf	< 0) {
		I-EINVALwXopmfanssiuple_nresLtateabuffer(uctiabufr coun*l)ppod, buf,s_pcnablk the buf	;>opmfkill)
		ret = -ERFKs) {
_t e intbgfs_rop the
	IWaic int iilD *iilDr);
}

voidcpci __ucti *uctiabufr);
}

void) {
_t coun*l)_Rff_t *ppod_hw(trans);
}

void ipcie_init_iilD_GprivIti_ On untrans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	scpci buf[100]ctccesspod lrdLodt;iscnprintf(buf,isiDDof(buf	, "debug: ;
		hw: ;
			}
		. Mointerrans=gbug_rop th}
		.!(upon resume = 0x%08X\nCSR_GP_Cts_pciebns, CSR_GP_CNTRL,
			 avoRckdep__SW)	;>>opmfkillsiuple_nresLtateabuffer(uctiabufr coun*l)ppod, buf,spodn_iet = -ERFKs) {
_t e intbgfs_rop the

	ifaic int iilD *iilDr);
}
);d);dtruct cpci __ucti *uctiabufr);
}

vooid) {
_t coun*l)_Rff_t *ppod_hw(trans);
}

void ipcie_init_iilD_GprivIti_ On untrans);
}

void iwl_pcie_synchronize_irqs(struct iwl_trans *trans)
{
	structoldia"_synchrrrans=gbug_rop th
	sIWL__pc
	>opmfanskrantotrucLtateaucti(uctiabufr coun*l)&_synchrrrans=gbug_rop thCLOCK_REad_cpu_		IWL_ERR(traNTRLoldiaa"_synchrrrans=gbug_rop thcpu_		IWL_Ecoun*	rS_GET_PCIE_TRANS(trcumsnter=gbugOx_lock(%d->;
			}
		iold,"_synchrrrans=gbug_rop thc;tal_workarous);
	_top the rqnans)
{
	stpmfkillcoun*	ret t %d)FS_NTRL_WRITE_FILE_OPS(tatus = I{
	t %d)FS_NTRL_FILE_OPS(fh_		d{
	t %d)FS_NTRL_FILE_OPS(	val = B{
	t %d)FS_NTRL_FILE_OPS(tval = B{
	t %d)FS_WRITE_FILE_OPS(csr{
	t %d)FS_NTRL_WRITE_FILE_OPS((trans, lr, 0r
	Itit The Ibugfs filDwl load2(trao tra_,
	_pcie_tx_reset(transbgfs_r, false/
	iwl_pcie_prepare_card_hw(trans);
di < y *d2(ia"_syncns=bgfs_d2(
	stt %d)FS_ADD_FILE(	val = Bildir, S_IRUSRc;tat %d)FS_ADD_FILE(tval = Bildir, S_IRUSRc;tat %d)FS_ADD_FILE(tatus = Iildir, S_IWUSR | S_IRUSRc;tat %d)FS_ADD_FILE(csrildir, S_IWUSRc;tat %d)FS_ADD_FILE(fh_		dildir, S_IRUSRc;tat %d)FS_ADD_FILE(rop th}ldir, S_IWUSR | S_IRUSRc;taans);

	/*
us : {e_tx_reset(tra "f	ret = iwc
	Itit Theprepar Ibugfs i < y
	ie {
) {
		I-E chunk_}i#g inf , reset *e_tWIFIEt %d)FS_,
	 = -ERFKtuctll(trans, was_it->vcm;
	nUns_pcie->mutex);
	return ILL;
*tfdum);
	}
}

static void iwl_pcie_map_rx_causes(struct iwl_trans *trans)
{
	suuctcm;
	ntruct ins);
	conGET_PCIE_TRANS(trans)TRANS_GEax_tbum_online_cm;
	nt+_MAC_ACCESStfd_tbit->v
	nURL_REG_cfd}si/
	cipmfkillcm;
	nrnet = -ERFKtuctll(trans, was_idumOvrba, p_addr);
	return ret;
}

sta airans);
}

vfwserroridumOv On  ** On }

sta ains);ans->op_mvrbz */_set_interrupt_capa(struct pci_dev *pdev,
					struct iwl_trans *trans)
{
	struct iw
	ntruPAGsNEIZEite__six(struct rxa PAG_ordev;LE, 0DumONRBscalle max_irq- noneGET_Ire-9AC_
kfill s (1;l = Bn ,
	 s}
}

statirxq_prxlit_&_six(struct rxl[0]ctctuctl,s_, j,s_b_
	ntruct stspinure i(&rtq->re in_ist(ia"le16_to_) >>trans);ONCE(atq->nb__dtse_cloct vrbz */e)
	i0x0, t	conGET_PCIE_rtq->nres, jtruct ivooidi0!t_rn&&sjS(tans->op_mvrbz */_t ivooidi0=_PCI+a"er& RXble_mswmaske jmsiie = t}
}

statirx_ evabuffer_prxbIE_rtq->l = Bns_		brrans);
}

vfwserroridumOvrb_prb			brdma_unmapm PAG__syncns=cie-rxbe_ PAG_dma,ct iw
	n	iwl_write_pDMA_FROM*GP_CNTu;n	b
_b_
	nt+;isiDDof(** On )MPM_ADDof(prb)MPMt iw
	n;n	b
(* On )->typev,

	/*to_lQ

	usesFWx_reORGDUMP_RBe {
	(* On )->
	ntru
	/*to_lQ

	_ADDof(prb)MPMt iw
	nu;
trabIE_(ILL;
*{(* On )-> On untrabt }ndextru
	/*to_lQ

	ievicrs_prph(abt  On }  PAG_IT(%tra,rxbe_ PAG),Mt iw
	nu;
tr00(trmaclear_ PAG GET_ear_ the benefitORE_CPUxbe_ PAG_dmatrudma_mapm PAG__syncns=cie-rxbe_ PAG,uct);
}
	_writet iw
	n	iwl_
	_writeDMA_FROM*GP_CNTu;n	b
* On ans->cffwserrorinext_ On (* On );_prp;
	pinur);
	i(&rtq->re in_ist(	IWL_ERb_
	nrnet#defineausesCs, TOGDUMP (0x250)

= -ERFKtuctll(trans, was_idumOvcsrn value, which is >= 0 i}

sta airans);
}

vfwserroridumOv On  ** On ucodeuet;csr_lil ;isiDDof(** On )MPMusesCs, TOGDUMP; t_glQ

;*vett ins);
	con(* On )->typev,

	/*to_lQ

	usesFWx_reORGDUMP_CSRc;ta(* On )->
	ntru
	/*to_lQ

	usesCs, TOGDUMPc;tavalue (ILL;
*{(* On )-> On unonGET_PCIE_TRANS(tusesCs, TOGDUMP;sir+= 4wl_reval++tru
	/*to_lQ

	isetn2)
			_iwlnresume = 0x%0i)	;>>o* On ans->cffwserrorinext_ On (* On );_cipmfkillcsr_lilrnet = -ERFKtuctll(trans, was_ifh_		dsidumO_tx_start(trans, scd_addr);
}

voidairans);
}

vfwserroridumOv On  ** On ucodeuet;fh_		dsi
	ntruFHwLMPMUPiwl_BOUND -uFHwLMPML
	cleBOUND;deun2 va	 ilo	 *flags
	s_glQ

;*vett ins);
	conNTRL!r_CNTRL,
	traberrupfh(tra,return &flagsuFpu_		IWL_E0	con(* On )->typev,

	/*to_lQ

	usesFWx_reORGDUMP_FHwTRLSc;ta(* On )->
	ntru
	/*to_lQ

	fh_		dsi
	nc;tavalue (ILL;
*{(* On )-> On unon_PCIE_TRANS(trans (iwl_opGET_PCIE_FHwLMPML
	cleBOUND;ANS(tFHwLMPMUPiwl_BOUND;
trvooidi0+;isiDDof(uetun    eval++tru
	/*to_lQ

	isetn2)
			_iwlnresume = 0x%0i)	;>s	u32 laGET_PCIE_FHwLMPML
	cleBOUNDct N2;ANS(tFHwLMPMUPiwl_BOUNDct N2;
trvooidi0+;isiDDof(uetun    eval++tru
	/*to_lQ

	isetn2)
			_iwlnresL_DEBUG_POWEiwl_
	_}

voidai)	;>>oP_CNTRL,
	_DMA_CLerrupfh(tra,return &flagsu= >o* On ans->cffwserrorinext_ On (* On );_cipmfkillsiDDof(** On )MPMfh_		dsi
	nrnet = -ERFKtuc
isetn2)
			_iidumOvmarbh->cfg->detx_start(trans, scd_addr);
}

rans);
}

vfwserroridumOvns->cf *ns->cf_ On }

sta t);

cfg->di
	nccodeuet;bufz) {
_inudividsue (
cfg->di
	nR>>R2{
	suuct*buffer_e (uuct*)ns->cf_ On -> On untun2 va	 ilo	 *flags
	stuctl	conNTRL!r_CNTRL,
	traberrupfh(tra,return &flagsuFpu_		IWL_E0	con_CAUSES_QL_DEB_noitrab,return MON_DMAsBSED_CTL_*/
		B0x1t) {GET_PCIE_TRANS(tbufz) {
_inudividsm_online_bufferns_REG
	}

	IWL_DEB_noitrab,return

staMON_DMAsBSED_DATA_*/
	c;tal_woSES_QL_DEB_noitrab,return MON_DMAsBSED_CTL_*/
		B0x0	;>>oP_CNTRL,
	_DMA_CLerrupfh(tra,return &flagsu= >o		IWL_E
cfg->di
	nrnet = -ERFKtuc
isetn2)
			_i_idumOv>cfg->detx_start(trans, scd_addr);
}idairans);
}

vfwserroridumOv On  ** On r);
}idait);

cfg->di
	nccode	}
}

static void iwl_pcie_map_rx_causes(struct iwl_trans *trans)
{
	suuct
	ntruct stNTRL. pus() + 2, ns->cf_ PAG 
	ifirst__TRANS(transs, CSR_GP_CNTR=, CSR_GP_CNTRL_REG_F7AC_C ||ifirst_syncns=bgepc	}_tlviie = t}
}

statifwserroridumOvns->cf *ns->cf_ On ;
truet;b_CL,cl
	ifL_tr,cl
apmcnt
	
tr00(If_earrruSs->  pc	} TLV -uuCLt(tra iwuwsstate earrruRE_CPswl_set_bit=bgepc	}_tlviie = 	wr	ifL_tr/t

stalQ

_to_) >>set_bit=bgepc	}_tlv->

	ifL_tr_		d{
	= 	wrapmcntia"le

_to_) >>set_bit=bgepc	}_tlv->

apmcoun*_;s	T	2_toMa"le

_to_) >>set_bit=bgepc	}_tlv->2_tot		d{
	= rans, CSR_Gi2_toMa"MON_BUFF_BASE_*/
	; = 	wr	ifL_tr/t"MON_BUFF_WRPTR
	= 	wrapmcntia"MON_BUFF_CYCLER_GP {
	}t	b
(* On )->typev,

	/*to_lQ

	usesFWx_reORGDUMP_FWxMONITOR{
	= ns->cf_ On IE_(ILL;
*{(* On )-> On untrns->cf_ On ->ns->cf_wrL_trit	= 	
	/*to_lQ

	iset
	IWL_DEBUG_POWERl
	ifL_tr)nuntrns->cf_ On ->ns->cf_cyclemcntia	= 	
	/*to_lQ

	iset
	IWL_DEBUG_POWERl
apmcnt)nuntrns->cf_ On ->ns->cf_2_tot;trit	= 	
	/*to_lQ

	iset
	IWL_DEBUG_POWER2_to)	;>>o	
	nt+;isiDDof(** On )MPM_ADDof(pns->cf_ On _;s	T_PCI pus() + 2, ns->cf_ PAGiie = 	NIT_D	ns, UREfirmans);alllso _transei}sit w TX qgured 	iome ofT_D	ns,uDE_DRVbuffer.tion(whiltisabownership_ iwsens); sthe ans)dOn ., UREbuffer_ cause, us);edpcW ituDE_DRVs, CSRhe ans)cie->istUREfirmans); cause, re_tart

	 *	ans/T_D	dma_
	 *_a_inlemGET_) >>set_bit=ciek_sz =; pus() + 2, ns->cf_ hysek_sz =; pus() + 2, ns->cf__ADDek_sz =;DMA_FROM*GP_CNTu;n	crs_prph(ns->cf_ On -> On 	iwl_write_p PAG_IT(%tra, pus() + 2, ns->cf_ PAGi	iwl_write_p pus() + 2, ns->cf__ADD	;>>o		
cfg->di
	nR=p pus() + 2, ns->cf__ADD
	= rans, Cswl_set_bit=bgepc	}_tlv->
cfg->di
	 * =, SLMPMMODEiie = 	NIT_D	ns,Uled,  po to mpreo refle

sthe dea iwuwss knowT_D	ns,shift ofT_D	ns/R_Gi2_toMa"iset
	IWL_DEBUG_POWER2_to) <<iwl_write_p pus(it=bgepc	}_tlv->2_totshift;n	crP_CNTRL,
	_DesLmemUG_POWER2_to, ns->cf_ On -> On 	iwl_l_wri
cfg->di
	nR/isiDDof(uetun
	= rans, Cswl_set_bit=bgepc	}_tlv->
cfg->di
	 * =, MAsBHMMODEiie = 	
cfg->di
	nR=iwl_lisetn2)
			_iidumOvmarbh->cfg->deG_POWEiwl_
	_}

ns->cf_ On }

sta	_}


cfg->di
	nc
	= rans, CSR_Gi, 0DidTX qmans);	iome of -r	iwpuwils

cfg->d  On  */>o		
cfg->di
	nR=p0 {
	}t	b

	nt+;i
cfg->di
	nrn
	(* On )->
	ntru
	/*to_lQ

	
cfg->di
	nRPM_ADDof(pns->cf_ On _);_prp;
		IWL_E
	nrnet = -ERFK	}
}

static voiddumOv On 
*isetn2)
			_i_idumOv On (tx_start(trans, scd_addr);
}idtruct iwl_trans_pns-=bgetrigger_tlvscd_iggerum);
	}
}

static void iwl_pcie_map_rx_causes(struct iwl_trans *trans)
{
	st}
}

statifwserroridumOv On  * On untrans);
}

voxq_pcmdlit_msix(struct ixq[_six(struct cmdaq = B]
	st}
}

statifwserroridumOvtxcmd_pcxcmduntrans);
}

void iwdumOv On  * umOv On 
	suuct
	n,  */
	bs
	stuct
cfg->di
	nrn
ns);
, _tr_GstructdumOvrbait_mans_pcieRESET);FWx_reORbit(STATUS_INT_EN 
	if		E_TRANS(transm_l	vae max_irq*/LE, 0)TRANax_iadumON;
	ig,
restlil ;isiDDof(* umOv On )*/LE, 0s;
	}ceue - s
restlil +;isiDDof(* On )MPne_cm;q->	_windso * 	_ADDof(pcxcmd)MPMTFDb	gotPAYLOADNEIZE)*/LE, 0FW

cfg->d restart. pus() + 2, ns->cf_ PAGiie = lil +;isiDDof(* On )MPisiDDof(t}
}

statifwserroridumOvns->cf)MPne_write_p pus() + 2, ns->cf__ADDvicrscfg->di
	nR=p pus() + 2, ns->cf__ADD
	=rans, Cswl_set_bit=bgepc	}_tlvk;
		cuet;b_CL,cenq*/LE	2_toMa"le

_to_) >>set_bit=bgepc	}_tlv->2_tot		d{
	= enqMa"le

_to_) >>set_bit=bgepc	}_tlv->enqt		d{
	
Gi2_toMa"iset
	IWL_DEBUG_POWER2_to) <<iwlwrite_p pus(it=bgepc	}_tlv->2_totshift;n	cenqMa"iset
	IWL_DEBUG_POWERenq) <<iwlwrite_set_bit=bgepc	}_tlv->enqtshift;n
tr00( * in"enq" po totuDE_DRVthe deaenqMRE_CPswl_set_bit(trans, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_ ||iffirst_syncns=bgepc	}_tlv->
cfg->di
	 * =, MAsBHMMODEiif		enqM+;i(1ite__six(it=bgepc	}_tlv->enqtshift)vicrscfg->di
	nR=penqM-R2_to; = lil +;isiDDof(* On )MPisiDDof(t}
}

statifwserroridumOvns->cf)MPne_write_p
cfg->di
	nrn
rans, CSR_G
cfg->di
	nR=p0 {
}	start. piggern&&s. pigger->
c * & usesFWxDBG_TRIGGEo MONITOR_ONLYFk;
		cdumOv On  = vz(tran(
	nc
	= _PCIEdumOv On )if				IWL_ENUhw_r		cdOn IE_(ILL;
*{dumOv On -> On untr
	nR=pisetn2)
			_i_idumOv>cfg->dereturn & On } 
cfg->di
	nc
	= dumOv On ->
	nR=p
	n;n	b
		IWL_E umOv On 
	s}/LE, 0 toes, false);restlil +;isiDDof(* On )MPMusesCs, TOGDUMP; LE, 0FHes, false);rest
"enabled");
	if (iwl_oplil +;isiDDof(* On )MPne_write_p(FHwLMPMUPiwl_BOUNDct N2 -uFHwLMPML
	cleBOUNDct N2	;>s	u32 lalil +;isiDDof(* On )MPne_write_p(FHwLMPMUPiwl_BOUND -uFHwLMPML
	cleBOUND)t stNTRLdumOvrbak;
		c, 0DumONRBscalle max_irq- noneGET_Ire-9AC_
kfill s (1;l = Bn ,
	  s}
}

statirxq_prxlit_&_six(struct rxl[0]ctcc, 0RBsc,
	   */
	bsia"le16_to_) >>trans);ONCE(atq->nb__dtse_cloct vrbz */e));
}

voida	i0x0, t	c   */
	bsia"( */
	bsi-irtq->nreser& RXble_mswmask; = lil +;i */
	bsi* 	_ADDof(p On )MPne_}

vsiDDof(t}
}

statifwserroridumOvrb)MPne_}

v(PAGsNEIZEite__six(struct rxa PAG_ordev_);_prp;
, 0PPAGd  everyeGET_ (iw
rtarest
"enabled");
	if (iwl_opGET_PCIE_TRANS(trans)TRANS_Giransdram. PA ofmcnt
_online_alil +;isiDDof(* On )MPne_

void i) {
of(t}
}

statifwserroridumOv PA of)MPne_

void irans)TRANS_Giransdram. PA ofns_p) {
/
	idumOv On  = vz(tran(
	nc
	=_PCIEdumOv On )if			IWL_ENUhw_r		
	nR=p0 {
dOn IE_(ILL;
*{dumOv On -> On unt On ->typev,

	/*to_lQ

	usesFWx_reORGDUMP_TXCMD)t 	txcmd_E_(ILL;
*{dOn -> On untspinure itbh(&cm;q->re in_ic_tr/t"cm;q->l
	ifL_tr) {GET_PCIE_TRANS(tcm;q->	_windsoh(Mmsiie = u8 idx _MAC_ACCESSt->vcm;_}ndex(cmdq, _tr);
truet;cap
	n, cm;
	nrn
e_cm;
	nt=pisetn2)
			_i_it->vcm;
	nUreturn cm;q->tf s
+iwl_
	_wrimsix(struct ifdz) {
	* _tr);
trcap
	nt=pminut(uet,MTFDb	gotPAYLOADNEIZE, cm;
	n);n	b
loadcm;
	n)CSR_Gilil +;isiDDof(*cxcmd)MPMcap
	n;n	crcxcmdt cmd
	ntru
	/*to_lQ

	cm;
	n);n	crcxcmdt cap
	nt=p
	/*to_lQ

	cap
	nu;n	crs_prph(cxcmdt  On } cm;q->i < transdx].cm;, cap
	nu;n	crtxcmd_E_(ILL;
*{((u8 *)cxcmdt  On MPMcap
	n) {
	}t	b
_tr/t"isetl = B
dec_l
ap(_tr);
t}
_spinur);
	itbh(&cm;q->re in_int On ->
	nt=p
	/*to_lQ

	
	n) {
lil +;isiDDof(* On ) {
dOn IE_->cffwserrorinext_ On ( On )*/LE
	nt+_MAC_Arans, was_idumOvcsrnreturn & On ) {
lil +;ill(trans, was_ifh_		dsidumO_return & On ) {
NTRLdumOvrbak = lil +;ill(trans, was_idumOvrba,return & On }  */
	bs)*/LE, 0PPAGd  everyeGET_ (iw
rtarest
"enabled");
	if (iwlie = GET_PCIE_TRANS(trans);

	if iransdram. PA ofmcnt
_onliCSR_Git}
}

statifwserroridumOv PA of * PA of;T_D	dma_IT(%_tBIT(%i=R_G=;_ans)TRANS_Giransdram. PA ofns_p hysicett itruet; PAG_
	nR=p pus() + 2, iransdram. PA ofns_p) {
/
	i	t On ->typev,

	/*to_lQ

	usesFWx_reORGDUMP_PAGIN	_wrd	t On ->
	nt=p
	/*to_lQ

	siDDof(* PA of)MP; PAG_
	n_wrd	t PA of E_(ILL;
*{dOn -> On unt	t PA oft }ndextru
	/*to_lQ

	ievicr	dma_
	 *_a_inlemGET_) >>set_bit=cie;secti; PAG_
	nek_sz =;DMA_BIDIRECTIONALu;n	crs_prph( PA oft  On 	iwl_write_p pus() + 2, iransdram. PA ofns_pbre i,* PAG_
	n_wrd	tdOn IE_->cffwserrorinext_ On ( On )*/LE= lil +;isiDDof(* On )MPisiDDof(* PA of)MP; PAG_
	n {
	}t	}t	blil +;ill(trans, was_idumOv>cfg->dereturn & On } 
cfg->di
	nc
	
 dumOv On ->
	nR=p
	n;n	b		IWL_E umOv On 
	tra#ifdef reset *PM(SLEEP
lease CPU resetn2)
			_iwls_pcie_s value, which is >= 0 if succ"enabled");un* Ce;pmi
	 * =, usesPLAT*PM(MODE_D0I3 
	ifirstnabled")syalsm;pmi
	 * =, usesPLAT*PM(MODE_D0I3uFpu_		IWL_EAC_ACCEffwseto m_d0i3nans)
{
	stpmfkill0
	 release Cwas_in_rfkill)
		iwl_Dsume  causes_list[i].cause_num);
c"enabled");un* Ce;pmi
	 * =, usesPLAT*PM(MODE_D0I3 
	ifirstnabled")syalsm;pmi
	 * =, usesPLAT*PM(MODE_D0I3uFpu_AC_ACCEffwsexansd0i3nans)
{
	}i#g inf , 0reset *PM(SLEEP0,
	 #defineauses(strucCOMMON_OPS_sz =;\
	.opi
	 *i
	avet=pisetn2)
			_i_iopi
	 *i
	ave,		}\ET.gured8t=pisetn2)
			_i_igured8,z =;\
	.SES_Q

t=pisetn2)
			_i_iguredet,z =;\
	.nresumt=pisetn2)
			_i_inresum,z =;\
	.nresL_DEBt=pisetn2)
			_i_inresL_DEB,z =;\
	.SES_QL_DEBt=pisetn2)
			_i_iSES_QL_DEB, =;\
	.nresLs_pt=pisetn2)
			_i_inresLs_p,z =;\
	.SES_QLs_pt=pisetn2)
			_i_iSES_QLs_p,z =;\
	.configuret=pisetn2)
			_i_iconfigure,z =;\
	._d tomit=pisetn2)
			_i_i_d tomi,z =;\
	.traberrupfh(trat=pisetn2)
			_i_itraberrupfh(tra,=;\
	.nrMA_CLerrupfh(trat=pisetn2)
			_i_inrMA_CLerrupfh(tra,;\
	._d ttra(t_TRAt=pisetn2)
			_i_i_d ttra(t_TRA, =;\
	.nrft=pisetn2)
			_i_inrf,sz =;\
	.un_Dft=pisetn2)
			_i_iun_Df,sz =;\
	.dumOv On  = isetn2)
			_i_idumOv On ,z =;\
	.d3ls_pcie_ = isetn2)
			_i_id3ls_pcie_, =;\
	.d3l_Dsume = isetn2)
			_i_id3l_Dsumera#ifdef reset *PM(SLEEP
#defineauses(strucPM(OPS_sz =;\
	.s_pcie_ = isetn2)
			_i_is_pcie_, =;;\
	.nrsume = isetn2)
			_i_inrsume,i#gu32 #defineauses(strucPM(OPSi#g inf , 0reset *PM(SLEEP0,
	 	}SEe C>
static int isetn2)
		opsun2)
		opsap_rx_ca{ {e_tx(strucCOMMON_OPS, {e_tx(strucPM(OPSi	.start_hw = isetn2)
			_i_istart_hw, {.fwsk(&tr = isetn2)
			_i_ifwsk(&tr,i	.start_fw = isetn2)
			_i_istart_fw,i	.stopikfill s= isetn2)
			_i_istopikfill ,

	._dnd_cmd_E_isetn2)
			_i_i_dnd_hcm;,

	.tx_E_isetn2)
			_i_itx,
	.nrclaipt=pisetn2)
			_i_inrclaip,

	.txqEt be re_E_isetn2)
			_i_itxqEt be re,
	.txqEAXl.
	_E_isetn2)
			_i_itxqEAXl.
	,

	.txqE_d tsharedi
	 * =_isetn2)
			_i_itxqE_d tsharedi
	 *,

	.
aitt_tal = BInemptyt=pisetn2)
			_i_iSaitt_tqsnempty,

	. thezwl_tqnt);
r = isetn2)
			_i_ifthezwl_tqnt);
r,
	.b;
	it_tqn_tra = isetn2)
			_i_ib;
	it_tqn_tra,
};
c	}SEe C>
static int isetn2)
		opsun2)
		opsap_rx_ (iw
ca{ {e_tx(strucCOMMON_OPS, {e_tx(strucPM(OPSi	.start_hw = isetn2)
			_i_istart_hw, {.fwsk(&tr = isetn2)
			_i_i (iwifwsk(&tr,i	.start_fw = isetn2)
			_i_i (iwistart_fw,i	.stopikfill s= isetn2)
			_i_i (iwistopikfill ,

	._dnd_cmd_E_isetn2)
			_i_i (iwisdnd_hcm;,

	.tx_E_isetn2)
			_i_i (iwitx,
	.nrclaipt=pisetn2)
			_i_inrclaip,

	.txqE(tran = isetn2)
			_i_idyn_txqE(tran,
	.txqE the = isetn2)
			_i_idyn_txqE the,
	.
aitt_tqnemptyt=pisetn2)
			_i_iSaitt_tqnempty,
};
c	}auses_list[i].caisetn2)
			_i_i(tran(	}ausespe->msi * =ciek_sz  );d);dtruct 	}ausespe->msiCSR_L;
*entek_sz  );d);dtruct 	}ausesiset;
	 *;
	um);
	}
}

static void iwl_pcie_map_rxuntrans);
}

void i_pcie_m;	sIWL__pce;sect_) {
/
	ipmfansp_rmEAXl.
	>msiCSR(ppci_wrdK_REad_cpu_		IWL_E_re_PTR(_pcnabrdK_RE;
	if (iwl_opoid i_=pisetn2)
		(tran(	 {
of(t}
}

staticie_map_rx)r);
}
	&pmsix_pci, ;
	l)&_synchopsap_rx_ (iw	;>s	u32 laoid i_=pisetn2)
		(tran(	 {
of(t}
}

staticie_map_rx)r);
}
	&pmsix_pci, ;
	l)&_synchopsap_rxc
	=_PCIEuse_numu_		IWL_E_re_PTR(-E chunnabrdcie_map_rx_causes(struct iwl_trans *trans)
{
	
;_ans)TRANS_Goid i_=pcie_m;	s_ans)TRANS_Gop
	 *idownR=p p B;s	spinure itefg-a= IWL_TRANS_Girqere in_inspinure itefg-a= IWL_TRANS_G		dere in_inmutextefg-a= IWL_TRANS_Gmutex{
	strittSaitl = B
h	IWa= IWL_TRANS_GuloadiSES_QLSaitl{
	
;_ans)TRANS_Grba.(tran_wlit_(tran_workl = B("rb_ans->opor	}
			b	  );WQ_HIGHPRI |;WQ_UNBOUND, 1c
	=_PCIEuse_nTRANS_Grba.(tran_wl)CSR_Gwl_pcie_ chunk_ num_ir	iwE the_cie_m;	s}t	INIT_WORKa= IWL_TRANS_G	ba.rxE(tran,MAC_ACCESSrxE(tranopor_work{
	
;_ans)TRANS_Goso_hct_ PAG t_(tran_per) >>t}
}

staticso_hct_ PAGc
	=_PCIEuse_nTRANS_Gcso_hct_ PAGcCSR_Gwl_pcie_ chunk_ num_ir	iwEnoTRAN;_prp;	=_PCIE(tran2_tot;
	}

S(CCESSl1E(trawedk;
		c, 
_DONEW/Ate befmsr_irsolUREwrirdpcihavior. W				  t(ois movw th_sT_DONE_PCeset TX qgatotuDEstayCPU L1_F kill i*);
., UisuSs-twss T_DONElo);of RL_RE.
D	ns/R_Gpe->m be re_linkein_rR(ppci, l_traLINK_RESEraL0S |k_sz  );d);dl_traLINK_RESEraL1 |k_sz  );d);dl_traLINK_RESEraCLKPM);_prp;
K_RE;
	if_cletfh	   CSsect_) {
pci64;
=;_ans)TRANS_GEax_tbu_causes(FH_NUM_TBS;
=;_ans)TRANS_Gifdz) {
	=i) {
of(t}
}

statitfhStfd)rn
rans, CSR_Gsect_) {
pci36;
=;_ans)TRANS_GEax_tbu_causesNUM_OF_TBS;
=;_ans)TRANS_Gifdz) {
	=i) {
of(t}
}

statitfd);
t}
_abled")Eax_skbE tagu_causesl_tra	gotFRAGS. pus() + 2{
	
;pe->_d tmaalse/ppci_wr	ipmfansp_r>_d tdma_mask(ppci, DMA_BIiwmask(sect_) {
)c
	=_PCIEad_cpu_		Iansp_r>_d ttrucfalsn tdma_mask(ppci,
			b	  )DMA_BIiwmask(sect_) {
)c
	=_PCIad_cCSR_Gwl_pcip_r>_d tdma_mask(ppci, DMA_BIiwmask(etun
	= _PCIEad_cpu__		Iansp_r>_d ttrucfalsn tdma_mask(ppci,
			b	   )DMA_BIiwmask(etun
	= , 0both attempts f	ret :MRE_CPswl_ad_cCSR_G	pciserr(&pmsix_pci, "NDEsuitl.
	_DMA av	rel.
	
	ie {
	num_ir	iwEnoTRAN;_p	}t	}t	bpmfansp_rmEiomapms, fo,
	_Dl =stE(tr(ppci, IX_R0), DRV_NAMEc
	=_PCIad_cCSR_Gpciserr(&pmsix_pci, "p_rmEiomapms, fo,
	_Dl =stE(tr f	ret 
	ie {
	um_ir	iwEnoTRAN;_prp;;_ans)TRANS_Ghw_2_toMa"p_rmEiomapmtl.
	/ppci_[0]ctc_PCIEuse_nTRANS_Ghw_2_tocCSR_Gpciserr(&pmsix_pci, "p_rmEiomapmtl.
	_f	ret 
	ie {
	wl_pcie_ cDEV {
	um_ir	iwEnoTRAN;_prp;;, 0W		t be re_ll iRETRY_TIMEOUTes, false (0x41)tuDEkeep
DONEl_t Tx__pc tra_tate  to mfenntetnt reC3tion(in_rRMRE_Cp_r>SES_QLconfig_from(ppci, l_tsCF	_RETRY_TIMEOUT	B0x00{
	
;_ans)TRANS_Gpe->msi a"pmsi;>oP_CNm be re_tatus = Israns)
{
	
;_ans)_Ghw_rsi a"iset
	IWume = 0x%08X\navoREVt; C, 
_ONEI_t(traLAG_ HW GP_CNTRtUREforma);of tURE4 froms of  to avoREV thou
_ONErcumsei}s loalso tURErsiis allalsp_F so  tcludms bit 0-1 (ls

cru
_ONE"dash"e iwuwu., DEkeep hw_rsi cW iwaidsuueupSEe re_-Ces'tr 	}
	  it
_ONEi_t(traoldiforma).
	arest
"enabled");
	ifs, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_iie = un2 va	 ilo	 *flags
	st;_ans)_Ghw_rsi a"(_ans)_Ghw_rsi 	i0xfff_C |
			b( to avoREV_STEP(_ans)_Ghw_rsi << 2) << 2);sNSH(trans->cfCCESSpre, te_caid_hwrans)
{
	sPswl_ad_cCSR_G	_GET_PCIE_TRANS(tExit HW notite_py
	ie {
	num_ir	iwEnoTRAN;_p	}t		c, 
_DONEin-ordevt(ois cogn {
	Clalsp_dr&trre * the te_ptchip_trrs al
_DONEi ilo>op_m att(traAUX bus MISC IT(%tra space.
D	ns/R_G->cf_it(tran = 0x%08X\nGP_set_pwr(t  );dns, CSR_GP_CNTRL,
			 INIT_DONTu;n	cudmlay(2);sNSH(trans->cfColl(tran = 0x%08X\nGP_set_pwr(t 	);dns, CSR_GP_CNTRL,
			 MACaCLOCK_NTRLYwr(t 	);dns, CSR_GP_CNTRL,
			 MACaCLOCK_NTRLYwr(t 	);d25AC_CLOCoK_READY,
			CSR_G	_GETt %d).INFOE_TRANS(tF	ret = iwwisa uclear_nic
	ie {
	num_ir	iwEnoTRAN;_p	}t		cruct}

NTRL,
	traberrupfh(tra,return &flagsuFCSR_G	uet;hw_alsp*/LE= hw_alspMa"iset
	IWL_DEB_noitrab,return WFPM_CP_CNTRLe {
	nhw_alspM|= ENABLE_WFPM {
	nl_woSES_QL_DEB_noitrab,return WFPM_CP_CNTRL,;hw_alspe {
	nhw_alspMa"iset
	IWL_DEB_noitrab,return AUX_MISCNTRLe {
	nhw_alspMa"(hw_alspM>> HW_STEP_LOCATIONSIX_S_ 	i0xF {
	nlf"(hw_alspM;a"0x3)R_G=;_ans)_Ghw_rsi a"(_ans)_Ghw_rsi 	i0xFFFFFFF3C |
			b	b(SILICONSC_STEP << 2);s		oP_CNTRL,
	_DMA_CLerrupfh(tra,return &flagsu= p	}t	}t	b, 
_ONE9AC_-se tra_tatutrap_m A-alspMhs->  pro remtnt res_pcie_/_Dsumer_ONE the ome*);
s even ca_clslear_whore_platform= iwgete tu i., Uisr_ONEworkarouthemisaslear_haidans);notigo  tuDE_DRVpro rem-ERFK	}ap_.
	arest
"enabled");
	iftatutrap_m 
	ifirstabled");
	ifs, CSR_GP_CNTR=, CSR_GP_CNTRL_REG_F9AC_

	ifirst to avoREV_STEP(_ans)_Ghw_rsi) =, SILICONSA_STEP)R_G->cf_it(tran = 0x%08X\nHOSTRE_ENAENwr(t  );dns, HOSTRE_ENAENcPM(IDLE_SRC_DIS_SBcPME)*/L#
"eIS_ENABLED(reset *e_tMVM)
;_ans)_Ghw_rfnid		uiset
	IWume = 0x%08X\navoRF(IDc
	=_PCI_ans)_Ghw_rfnid		=08X\navoRF(ID_TYPE_HRk;
		cuet;hw_alNT_E;	
	nhw_alSET_REG
	}

	IWL_DEBUG_POWERUM		 GENcHW_STO_RE_wrdnlf"(hw_alSET_R&SUM		 GENcHW_IS_FPGA)R_G=abled");
	it_&
	}aAC__2ax_;
	_qnj_hr_f0 {
		u32 la=abled");
	it_&
	}aAC__2ac_;
	_hr;_prp#g inf

al_workaro_it(tatus = IWcapa(ppci, ans)
{
	s_ans)_Ghw_id		u(pmsix_pcill s<< 16)MP; msix_subsyalsm;pcill _insnprintf(_ans)_Ghw_id_ran,isiDDof(_ans)_Ghw_id_ran)r);
 "l_t ID:retu04X:etu04X", pmsix_pcill ,; msix_subsyalsm;pcill )*/LE, 0Initial {
	ear_wait q = B>GET_ceue - s
resttrittSaitl = B
h	IWa= IWL_TRANS_GSaittceue - al = B{
	sttrittSaitl = B
h	IWa= IWL_TRANS_Gd0i3ESaitl{
	
;art. pus() + 2, msixEAXl.
	dcCSR_Gwl_pcil_workarotrittmsixEus);
	r(ppci, ans)
ap_rxc
	=T_PCIad_cpu_sum_ir	iwEnoTRAN;_p rans, CSR_Gwl_pcil_workaro(tran_ictrans)
{
	sPswl_ad_cpu_sum_ir	iwEnoTRAN;_R_Gwl_pcipcim	_Dl =stEth
	IWede rqn&pmsix_pci, pmsix_ rq,
			b	 l_workarotsr,
			b	 l_workarotrqEus);
	r,
			b	 IRQF_SHARED, DRV_NAME, ans)
{
	sPswl_ad_cCSR_G	_GET_reset(tra "Errortans->opntetIRQc;
			}spmsix_ rqe {
	num_ir	iwE the_ict= p	}t	;_ans)TRANS_Girtat_TRAt=p to 
	I_SETwmask; = tra#ifdef reset *e_tWIFIEl_traRTPM5H_ans)_G;un* Ce;pmi
	 * = usesPLAT*PM(MODE_D0I3;i#gu32 H_ans)_G;un* Ce;pmi
	 * = usesPLAT*PM(MODE_DISABLED_i#g inf , 0reset *e_tWIFIEl_traRTPM0,
	 _		IWL_Ecie_m;	
	iwE the_ict:
al_workaro the_ictrans)
{
		iwEnoTRAN:
a the_per) >>use_nTRANS_Gcso_hct_ PAGc {
d=stroy_workl = B(use_nTRANS_Grba.(tran_wl);
	iwE the_cie_m:>oP_CNTRL,
	 the ans)
{
	s		IWL_E_re_PTR(_pcnab}
