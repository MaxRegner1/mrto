/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 * Copyright(c) 2015 - 2017 Intel Deutschland GmbH
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
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 * Copyright(c) 2015 - 2017 Intel Deutschland GmbH
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/etherdevice.h>

#include <net/mac80211.h>

#include "iwl-debug.h"
#include "mvm.h"
#include "iwl-modparams.h"
#include "fw/api/power.h"

#define POWER_KEEP_ALIVE_PERIOD_SEC    25

static
int iwl_mvm_beacon_filter_send_cmd(struct iwl_mvm *mvm,
				   struct iwl_beacon_filter_cmd *cmd,
				   u32 flags)
{
	IWL_DEBUG_POWER(mvm, "ba_enable_beacon_abort is: %d\n",
			le32_to_cpu(cmd->ba_enable_beacon_abort));
	IWL_DEBUG_POWER(mvm, "ba_escape_timer is: %d\n",
			le32_to_cpu(cmd->ba_escape_timer));
	IWL_DEBUG_POWER(mvm, "bf_debug_flag is: %d\n",
			le32_to_cpu(cmd->bf_debug_flag));
	IWL_DEBUG_POWER(mvm, "bf_enable_beacon_filter is: %d\n",
			le32_to_cpu(cmd->bf_enable_beacon_filter));
	IWL_DEBUG_POWER(mvm, "bf_energy_delta is: %d\n",
			le32_to_cpu(cmd->bf_energy_delta));
	IWL_DEBUG_POWER(mvm, "bf_escape_timer is: %d\n",
			le32_to_cpu(cmd->bf_escape_timer));
	IWL_DEBUG_POWER(mvm, "bf_roaming_energy_delta is: %d\n",
			le32_to_cpu(cmd->bf_roaming_energy_delta));
	IWL_DEBUG_POWER(mvm, "bf_roaming_state is: %d\n",
			le32_to_cpu(cmd->bf_roaming_state));
	IWL_DEBUG_POWER(mvm, "bf_temp_threshold is: %d\n",
			le32_to_cpu(cmd->bf_temp_threshold));
	IWL_DEBUG_POWER(mvm, "bf_temp_fast_filter is: %d\n",
			le32_to_cpu(cmd->bf_temp_fast_filter));
	IWL_DEBUG_POWER(mvm, "bf_temp_slow_filter is: %d\n",
			le32_to_cpu(cmd->bf_temp_slow_filter));

	return iwl_mvm_send_cmd_pdu(mvm, REPLY_BEACON_FILTERING_CMD, flags,
				    sizeof(struct iwl_beacon_filter_cmd), cmd);
}

static
void iwl_mvm_beacon_filter_set_cqm_params(struct iwl_mvm *mvm,
					  struct ieee80211_vif *vif,
					  struct iwl_beacon_filter_cmd *cmd,
					  bool d0i3)
{
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);

	if (vif->bss_conf.cqm_rssi_thold && !d0i3) {
		cmd->bf_energy_delta =
			cpu_to_le32(vif->bss_conf.cqm_rssi_hyst);
		/* fw uses an absolute value for this */
		cmd->bf_roaming_state =
			cpu_to_le32(-vif->bss_conf.cqm_rssi_thold);
	}
	cmd->ba_enable_beacon_abort = cpu_to_le32(mvmvif->bf_data.ba_enabled);
}

static void iwl_mvm_power_log(struct iwl_mvm *mvm,
			      struct iwl_mac_power_cmd *cmd)
{
	IWL_DEBUG_POWER(mvm,
			"Sending power table command on mac id 0x%X for power level %d, flags = 0x%X\n",
			cmd->id_and_color, iwlmvm_mod_params.power_scheme,
			le16_to_cpu(cmd->flags));
	IWL_DEBUG_POWER(mvm, "Keep alive = %u sec\n",
			le16_to_cpu(cmd->keep_alive_seconds));

	if (!(cmd->flags & cpu_to_le16(POWER_FLAGS_POWER_MANAGEMENT_ENA_MSK))) {
		IWL_DEBUG_POWER(mvm, "Disable power management\n");
		return;
	}

	IWL_DEBUG_POWER(mvm, "Rx timeout = %u usec\n",
			le32_to_cpu(cmd->rx_data_timeout));
	IWL_DEBUG_POWER(mvm, "Tx timeout = %u usec\n",
			le32_to_cpu(cmd->tx_data_timeout));
	if (cmd->flags & cpu_to_le16(POWER_FLAGS_SKIP_OVER_DTIM_MSK))
		IWL_DEBUG_POWER(mvm, "DTIM periods to skip = %u\n",
				cmd->skip_dtim_periods);
	if (cmd->flags & cpu_to_le16(POWER_FLAGS_LPRX_ENA_MSK))
		IWL_DEBUG_POWER(mvm, "LP RX RSSI threshold = %u\n",
				cmd->lprx_rssi_threshold);
	if (cmd->flags & cpu_to_le16(POWER_FLAGS_ADVANCE_PM_ENA_MSK)) {
		IWL_DEBUG_POWER(mvm, "uAPSD enabled\n");
		IWL_DEBUG_POWER(mvm, "Rx timeout (uAPSD) = %u usec\n",
				le32_to_cpu(cmd->rx_data_timeout_uapsd));
		IWL_DEBUG_POWER(mvm, "Tx timeout (uAPSD) = %u usec\n",
				le32_to_cpu(cmd->tx_data_timeout_uapsd));
		IWL_DEBUG_POWER(mvm, "QNDP TID = %d\n", cmd->qndp_tid);
		IWL_DEBUG_POWER(mvm, "ACs flags = 0x%x\n", cmd->uapsd_ac_flags);
		IWL_DEBUG_POWER(mvm, "Max SP = %d\n", cmd->uapsd_max_sp);
	}
}

static void iwl_mvm_power_configure_uapsd(struct iwl_mvm *mvm,
					  struct ieee80211_vif *vif,
					  struct iwl_mac_power_cmd *cmd)
{
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);
	enum ieee80211_ac_numbers ac;
	bool tid_found = false;

#ifdef CONFIG_IWLWIFI_DEBUGFS
	/* set advanced pm flag with no uapsd ACs to enable ps-poll */
	if (mvmvif->dbgfs_pm.use_ps_poll) {
		cmd->flags |= cpu_to_le16(POWER_FLAGS_ADVANCE_PM_ENA_MSK);
		return;
	}
#endif

	for (ac = IEEE80211_AC_VO; ac <= IEEE80211_AC_BK; ac++) {
		if (!mvmvif->queue_params[ac].uapsd)
			continue;

		if (mvm->fwrt.cur_fw_img != IWL_UCODE_WOWLAN)
			cmd->flags |=
				cpu_to_le16(POWER_FLAGS_ADVANCE_PM_ENA_MSK);

		cmd->uapsd_ac_flags |= BIT(ac);

		/* QNDP TID -softwa&rSi]i)
		cmdpsd_ac_flags |= BIT(ac);

		/*9/.