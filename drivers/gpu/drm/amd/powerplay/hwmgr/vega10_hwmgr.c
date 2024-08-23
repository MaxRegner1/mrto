/*
 * Copyright 2016 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "hwmgr.h"
#include "amd_powerplay.h"
#include "vega10_smumgr.h"
#include "hardwaremanager.h"
#include "ppatomfwctrl.h"
#include "atomfirmware.h"
#include "cgs_common.h"
#include "vega10_powertune.h"
#include "smu9.h"
#include "smu9_driver_if.h"
#include "vega10_inc.h"
#include "pp_soc15.h"
#include "pppcielanes.h"
#include "vega10_hwmgr.h"
#include "vega10_processpptables.h"
#include "vega10_pptable.h"
#include "vega10_thermal.h"
#include "pp_debug.h"
#include "pp_acpi.h"
#include "amd_pcie_helpers.h"
#include "cgs_linux.h"
#include "ppinterrupt.h"
#include "pp_overdriver.h"

#define VOLTAGE_SCALE  4
#define VOLTAGE_VID_OFFSET_SCALE1   625
#define VOLTAGE_VID_OFFSET_SCALE2   100

#define HBM_MEMORY_CHANNEL_WIDTH    128

uint32_t channel_number[] = {1, 2, 0, 4, 0, 8, 0, 16, 2};

#define MEM_FREQ_LOW_LATENCY        25000
#define MEM_FREQ_HIGH_LATENCY       80000
#define MEM_LATENCY_HIGH            245
#define MEM_LATENCY_LOW             35
#define MEM_LATENCY_ERR             0xFFFF

#define mmDF_CS_AON0_DramBaseAddress0                                                                  0x0044
#define mmDF_CS_AON0_DramBaseAddress0_BASE_IDX                                                         0

//DF_CS_AON0_DramBaseAddress0
#define DF_CS_AON0_DramBaseAddress0__AddrRngVal__SHIFT                                                        0x0
#define DF_CS_AON0_DramBaseAddress0__LgcyMmioHoleEn__SHIFT                                                    0x1
#define DF_CS_AON0_DramBaseAddress0__IntLvNumChan__SHIFT                                                      0x4
#define DF_CS_AON0_DramBaseAddress0__IntLvAddrSel__SHIFT                                                      0x8
#define DF_CS_AON0_DramBaseAddress0__DramBaseAddr__SHIFT                                                      0xc
#define DF_CS_AON0_DramBaseAddress0__AddrRngVal_MASK                                                          0x00000001L
#define DF_CS_AON0_DramBaseAddress0__LgcyMmioHoleEn_MASK                                                      0x00000002L
#define DF_CS_AON0_DramBaseAddress0__IntLvNumChan_MASK                                                        0x000000F0L
#define DF_CS_AON0_DramBaseAddress0__IntLvAddrSel_MASK                                                        0x00000700L
#define DF_CS_AON0_DramBaseAddress0__DramBaseAddr_MASK                                                        0xFFFFF000L
static int vega10_force_clock_level(struct pp_hwmgr *hwmgr,
		enum pp_clock_type type, uint32_t mask);

const ULONG PhwVega10_Magic = (ULONG)(PHM_VIslands_Magic);

struct vega10_power_state *cast_phw_vega10_power_state(
				  struct pp_hw_power_state *hw_ps)
{
	PP_ASSERT_WITH_CODE((PhwVega10_Magic == hw_ps->magic),
				"Invalid Powerstate Type!",
				 return NULL;);

	return (struct vega10_power_state *)hw_ps;
}

const struct vega10_power_state *cast_const_phw_vega10_power_state(
				 const struct pp_hw_power_state *hw_ps)
{
	PP_ASSERT_WITH_CODE((PhwVega10_Magic == hw_ps->magic),
				"Invalid Powerstate Type!",
				 return NULL;);

	return (const struct vega10_power_state *)hw_ps;
}

static void vega10_set_default_registry_data(struct pp_hwmgr *hwmgr)
{
	struct vega10_hwmgr *data =
			(struct vega10_hwmgr *)(hwmgr->backend);

	data->registry_data.sclk_dpm_key_disabled =
			hwmgr->feature_mask & PP_SCLK_DPM_MASK ? false : true;
	data->registry_data.socclk_dpm_key_disabled =
			hwmgr->feature_mask & PP_SOCCLK_DPM_MASK ? false : true;
	data->registry_data.mclk_dpm_key_disabled =
			hwmgr->feature_mask & PP_MCLK_DPM_MASK ? false : true;
	data->registry_data.pcie_dpm_key_disabled =
			hwmgr->feature_mask & PP_PCIE_DPM_MASK ? false : true;

	data->registry_data.dcefclk_dpm_key_disabled =
			hwmgr->feature_mask & PP_DCEFCLK_DPM_MASK ? false : true;

	if (hwmgr->feature_mask & PP_POWER_CONTAINMENT_MASK) {
		data->registry_data.power_containment_support = 1;
		data->registry_data.enable_pkg_pwr_tracking_feature = 1;
		data->registry_data.enable_tdc_limit_feature = 1;
	}

	data->registry_data.clock_stretcher_support =
			hwmgr->feature_mask & PP_CLOCK_STRETCH_MASK ? true : false;

	data->registry_data.ulv_support =
			hwmgr->feature_mask & PP_ULV_MASK ? true : false;

	data->registry_data.sclk_deep_sleep_support =
			hwmgr->feature_mask & PP_SCLK_DEEP_SLEEP_MASK ? true : false;

	data->registry_data.disable_water_mark = 0;

	data->registry_data.fan_control_support = 1;
	data->registry_data.thermal_support = 1;
	data->registry_data.fw_ctf_enabled = 1;

	data->registry_data.avfs_support = 1;
	data->registry_data.led_dpm_enabled = 1;

	data->registry_data.vr0hot_enabled = 1;
	data->registry_data.vr1hot_enabled = 1;
	data->registry_data.regulator_hot_gpio_support = 1;

	data->registry_data.didt_support = 1;
	if (data->registry_data.didt_support) {
		data->registry_data.didt_mode = 6;
		data->registry_data.sq_ramping_support = 1;
		data->registry_data.db_ramping_support = 0;
		data->registry_data.td_ramping_support = 0;
		data->registry_data.tcp_ramping_support = 0;
		data->registry_data.dbr_ramping_support = 0;
		data->registry_data.edc_didt_support = 1;
		data->registry_data.gc_didt_support = 0;
		data->registry_data.psm_didt_support = 0;
	}

	data->display_voltage_mode = PPVEGA10_VEGA10DISPLAYVOLTAGEMODE_DFLT;
	data->dcef_clk_quad_eqn_a = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;
	data->dcef_clk_quad_eqn_b = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;
	data->dcef_clk_quad_eqn_c = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;
	data->disp_clk_quad_eqn_a = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;
	data->disp_clk_quad_eqn_b = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;
	data->disp_clk_quad_eqn_c = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;
	data->pixel_clk_quad_eqn_a = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;
	data->pixel_clk_quad_eqn_b = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;
	data->pixel_clk_quad_eqn_c = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;
	data->phy_clk_quad_eqn_a = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;
	data->phy_clk_quad_eqn_b = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;
	data->phy_clk_quad_eqn_c = PPREGKEY_VEGA10QUADRATICEQUATION_DFLT;

	data->gfxclk_average_alpha = PPVEGA10_VEGA10GFXCLKAVERAGEALPHA_DFLT;
	data->socclk_average_alpha = PPVEGA10_VEGA10SOCCLKAVERAGEALPHA_DFLT;
	data->uclk_average_alpha = PPVEGA10_VEGA10UCLKCLKAVERAGEALPHA_DFLT;
	data->gfx_activity_average_alpha = PPVEGA10_VEGA10GFXACTIVITYAVERAGEALPHA_DFLT;
}

static int vega10_set_features_platform_caps(struct pp_hwmgr *hwmgr)
{
	struct vega10_hwmgr *data =
			(struct vega10_hwmgr *)(hwmgr->backend);
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)hwmgr->pptable;
	struct cgs_system_info sys_info = {0};
	int result;

	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_SclkDeepSleep);

	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_DynamicPatchPowerState);

	if (data->vddci_control == VEGA10_VOLTAGE_CONTROL_NONE)
		phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
				PHM_PlatformCaps_ControlVDDCI);

	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_TablelessHardwareInterface);

	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_EnableSMU7ThermalManagement);

	sys_info.size = sizeof(struct cgs_system_info);
	sys_info.info_id = CGS_SYSTEM_INFO_PG_FLAGS;
	result = cgs_query_system_info(hwmgr->device, &sys_info);

	if (!result && (sys_info.value & AMD_PG_SUPPORT_UVD))
		phm_cap_set(hwmgr->platform_descriptor.platformCaps,
				PHM_PlatformCaps_UVDPowerGating);

	if (!result && (sys_info.value & AMD_PG_SUPPORT_VCE))
		phm_cap_set(hwmgr->platform_descriptor.platformCaps,
				PHM_PlatformCaps_VCEPowerGating);

	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_UnTabledHardwareInterface);

	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_FanSpeedInTableIsRPM);

	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_ODFuzzyFanControlSupport);

	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
				PHM_PlatformCaps_DynamicPowerManagement);

	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_SMC);

	/* power tune caps */
	/* assume disabled */
	phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_PowerContainment);
	phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_DiDtSupport);
	phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_SQRamping);
	phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_DBRamping);
	phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_TDRamping);
	phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_TCPRamping);
	phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_DBRRamping);
	phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_DiDtEDCEnable);
	phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_GCEDC);
	phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_PSM);

	if (data->registry_data.didt_support) {
		phm_cap_set(hwmgr->platform_descriptor.platformCaps, PHM_PlatformCaps_DiDtSupport);
		if (data->registry_data.sq_ramping_support)
			phm_cap_set(hwmgr->platform_descriptor.platformCaps, PHM_PlatformCaps_SQRamping);
		if (data->registry_data.db_ramping_support)
			phm_cap_set(hwmgr->platform_descriptor.platformCaps, PHM_PlatformCaps_DBRamping);
		if (data->registry_data.td_ramping_support)
			phm_cap_set(hwmgr->platform_descriptor.platformCaps, PHM_PlatformCaps_TDRamping);
		if (data->registry_data.tcp_ramping_support)
			phm_cap_set(hwmgr->platform_descriptor.platformCaps, PHM_PlatformCaps_TCPRamping);
		if (data->registry_data.dbr_ramping_support)
			phm_cap_set(hwmgr->platform_descriptor.platformCaps, PHM_PlatformCaps_DBRRamping);
		if (data->registry_data.edc_didt_support)
			phm_cap_set(hwmgr->platform_descriptor.platformCaps, PHM_PlatformCaps_DiDtEDCEnable);
		if (data->registry_data.gc_didt_support)
			phm_cap_set(hwmgr->platform_descriptor.platformCaps, PHM_PlatformCaps_GCEDC);
		if (data->registry_data.psm_didt_support)
			phm_cap_set(hwmgr->platform_descriptor.platformCaps, PHM_PlatformCaps_PSM);
	}

	if (data->registry_data.power_containment_support)
		phm_cap_set(hwmgr->platform_descriptor.platformCaps,
				PHM_PlatformCaps_PowerContainment);
	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_CAC);

	if (table_info->tdp_table->usClockStretchAmount &&
			data->registry_data.clock_stretcher_support)
		phm_cap_set(hwmgr->platform_descriptor.platformCaps,
				PHM_PlatformCaps_ClockStretcher);

	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_RegulatorHot);
	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_AutomaticDCTransition);

	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_UVDDPM);
	phm_cap_set(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_VCEDPM);

	return 0;
}

static void vega10_init_dpm_defaults(struct pp_hwmgr *hwmgr)
{
	struct vega10_hwmgr *data = (struct vega10_hwmgr *)(hwmgr->backend);
	int i;

	vega10_initialize_power_tune_defaults(hwmgr);

	for (i = 0; i < GNLD_FEATURES_MAX; i++) {
		data->smu_features[i].smu_feature_id = 0xffff;
		data->smu_features[i].smu_feature_bitmap = 1 << i;
		data->smu_features[i].enabled = false;
		data->smu_features[i].supported = false;
	}

	data->smu_features[GNLD_DPM_PREFETCHER].smu_feature_id =
			FEATURE_DPM_PREFETCHER_BIT;
	data->smu_features[GNLD_DPM_GFXCLK].smu_feature_id =
			FEATURE_DPM_GFXCLK_BIT;
	data->smu_features[GNLD_DPM_UCLK].smu_feature_id =
			FEATURE_DPM_UCLK_BIT;
	data->smu_features[GNLD_DPM_SOCCLK].smu_feature_id =
			FEATURE_DPM_SOCCLK_BIT;
	data->smu_features[GNLD_DPM_UVD].smu_feature_id =
			FEATURE_DPM_UVD_BIT;
	data->smu_features[GNLD_DPM_VCE].smu_feature_id =
			FEATURE_DPM_VCE_BIT;
	data->smu_features[GNLD_DPM_MP0CLK].smu_feature_id =
			FEATURE_DPM_MP0CLK_BIT;
	data->smu_features[GNLD_DPM_LINK].smu_feature_id =
			FEATURE_DPM_LINK_BIT;
	data->smu_features[GNLD_DPM_DCEFCLK].smu_feature_id =
			FEATURE_DPM_DCEFCLK_BIT;
	data->smu_features[GNLD_ULV].smu_feature_id =
			FEATURE_ULV_BIT;
	data->smu_features[GNLD_AVFS].smu_feature_id =
			FEATURE_AVFS_BIT;
	data->smu_features[GNLD_DS_GFXCLK].smu_feature_id =
			FEATURE_DS_GFXCLK_BIT;
	data->smu_features[GNLD_DS_SOCCLK].smu_feature_id =
			FEATURE_DS_SOCCLK_BIT;
	data->smu_features[GNLD_DS_LCLK].smu_feature_id =
			FEATURE_DS_LCLK_BIT;
	data->smu_features[GNLD_PPT].smu_feature_id =
			FEATURE_PPT_BIT;
	data->smu_features[GNLD_TDC].smu_feature_id =
			FEATURE_TDC_BIT;
	data->smu_features[GNLD_THERMAL].smu_feature_id =
			FEATURE_THERMAL_BIT;
	data->smu_features[GNLD_GFX_PER_CU_CG].smu_feature_id =
			FEATURE_GFX_PER_CU_CG_BIT;
	data->smu_features[GNLD_RM].smu_feature_id =
			FEATURE_RM_BIT;
	data->smu_features[GNLD_DS_DCEFCLK].smu_feature_id =
			FEATURE_DS_DCEFCLK_BIT;
	data->smu_features[GNLD_ACDC].smu_feature_id =
			FEATURE_ACDC_BIT;
	data->smu_features[GNLD_VR0HOT].smu_feature_id =
			FEATURE_VR0HOT_BIT;
	data->smu_features[GNLD_VR1HOT].smu_feature_id =
			FEATURE_VR1HOT_BIT;
	data->smu_features[GNLD_FW_CTF].smu_feature_id =
			FEATURE_FW_CTF_BIT;
	data->smu_features[GNLD_LED_DISPLAY].smu_feature_id =
			FEATURE_LED_DISPLAY_BIT;
	data->smu_features[GNLD_FAN_CONTROL].smu_feature_id =
			FEATURE_FAN_CONTROL_BIT;
	data->smu_features[GNLD_ACG].smu_feature_id = FEATURE_ACG_BIT;
	data->smu_features[GNLD_DIDT].smu_feature_id = FEATURE_GFX_EDC_BIT;

	if (!data->registry_data.prefetcher_dpm_key_disabled)
		data->smu_features[GNLD_DPM_PREFETCHER].supported = true;

	if (!data->registry_data.sclk_dpm_key_disabled)
		data->smu_features[GNLD_DPM_GFXCLK].supported = true;

	if (!data->registry_data.mclk_dpm_key_disabled)
		data->smu_features[GNLD_DPM_UCLK].supported = true;

	if (!data->registry_data.socclk_dpm_key_disabled)
		data->smu_features[GNLD_DPM_SOCCLK].supported = true;

	if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_UVDDPM))
		data->smu_features[GNLD_DPM_UVD].supported = true;

	if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_VCEDPM))
		data->smu_features[GNLD_DPM_VCE].supported = true;

	if (!data->registry_data.pcie_dpm_key_disabled)
		data->smu_features[GNLD_DPM_LINK].supported = true;

	if (!data->registry_data.dcefclk_dpm_key_disabled)
		data->smu_features[GNLD_DPM_DCEFCLK].supported = true;

	if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_SclkDeepSleep) &&
			data->registry_data.sclk_deep_sleep_support) {
		data->smu_features[GNLD_DS_GFXCLK].supported = true;
		data->smu_features[GNLD_DS_SOCCLK].supported = true;
		data->smu_features[GNLD_DS_LCLK].supported = true;
		data->smu_features[GNLD_DS_DCEFCLK].supported = true;
	}

	if (data->registry_data.enable_pkg_pwr_tracking_feature)
		data->smu_features[GNLD_PPT].supported = true;

	if (data->registry_data.enable_tdc_limit_feature)
		data->smu_features[GNLD_TDC].supported = true;

	if (data->registry_data.thermal_support)
		data->smu_features[GNLD_THERMAL].supported = true;

	if (data->registry_data.fan_control_support)
		data->smu_features[GNLD_FAN_CONTROL].supported = true;

	if (data->registry_data.fw_ctf_enabled)
		data->smu_features[GNLD_FW_CTF].supported = true;

	if (data->registry_data.avfs_support)
		data->smu_features[GNLD_AVFS].supported = true;

	if (data->registry_data.led_dpm_enabled)
		data->smu_features[GNLD_LED_DISPLAY].supported = true;

	if (data->registry_data.vr1hot_enabled)
		data->smu_features[GNLD_VR1HOT].supported = true;

	if (data->registry_data.vr0hot_enabled)
		data->smu_features[GNLD_VR0HOT].supported = true;

	smum_send_msg_to_smc(hwmgr->smumgr, PPSMC_MSG_GetSmuVersion);
	vega10_read_arg_from_smc(hwmgr->smumgr, &(data->smu_version));
		/* ACG firmware has major version 5 */
	if ((data->smu_version & 0xff000000) == 0x5000000)
		data->smu_features[GNLD_ACG].supported = true;

	if (data->registry_data.didt_support)
		data->smu_features[GNLD_DIDT].supported = true;

}

#ifdef PPLIB_VEGA10_EVV_SUPPORT
static int vega10_get_socclk_for_voltage_evv(struct pp_hwmgr *hwmgr,
	phm_ppt_v1_voltage_lookup_table *lookup_table,
	uint16_t virtual_voltage_id, int32_t *socclk)
{
	uint8_t entry_id;
	uint8_t voltage_id;
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)(hwmgr->pptable);

	PP_ASSERT_WITH_CODE(lookup_table->count != 0,
			"Lookup table is empty",
			return -EINVAL);

	/* search for leakage voltage ID 0xff01 ~ 0xff08 and sclk */
	for (entry_id = 0; entry_id < table_info->vdd_dep_on_sclk->count; entry_id++) {
		voltage_id = table_info->vdd_dep_on_socclk->entries[entry_id].vddInd;
		if (lookup_table->entries[voltage_id].us_vdd == virtual_voltage_id)
			break;
	}

	PP_ASSERT_WITH_CODE(entry_id < table_info->vdd_dep_on_socclk->count,
			"Can't find requested voltage id in vdd_dep_on_socclk table!",
			return -EINVAL);

	*socclk = table_info->vdd_dep_on_socclk->entries[entry_id].clk;

	return 0;
}

#define ATOM_VIRTUAL_VOLTAGE_ID0             0xff01
/**
* Get Leakage VDDC based on leakage ID.
*
* @param    hwmgr  the address of the powerplay hardware manager.
* @return   always 0.
*/
static int vega10_get_evv_voltages(struct pp_hwmgr *hwmgr)
{
	struct vega10_hwmgr *data = (struct vega10_hwmgr *)(hwmgr->backend);
	uint16_t vv_id;
	uint32_t vddc = 0;
	uint16_t i, j;
	uint32_t sclk = 0;
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)hwmgr->pptable;
	struct phm_ppt_v1_clock_voltage_dependency_table *socclk_table =
			table_info->vdd_dep_on_socclk;
	int result;

	for (i = 0; i < VEGA10_MAX_LEAKAGE_COUNT; i++) {
		vv_id = ATOM_VIRTUAL_VOLTAGE_ID0 + i;

		if (!vega10_get_socclk_for_voltage_evv(hwmgr,
				table_info->vddc_lookup_table, vv_id, &sclk)) {
			if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
					PHM_PlatformCaps_ClockStretcher)) {
				for (j = 1; j < socclk_table->count; j++) {
					if (socclk_table->entries[j].clk == sclk &&
							socclk_table->entries[j].cks_enable == 0) {
						sclk += 5000;
						break;
					}
				}
			}

			PP_ASSERT_WITH_CODE(!atomctrl_get_voltage_evv_on_sclk_ai(hwmgr,
					VOLTAGE_TYPE_VDDC, sclk, vv_id, &vddc),
					"Error retrieving EVV voltage value!",
					continue);


			/* need to make sure vddc is less than 2v or else, it could burn the ASIC. */
			PP_ASSERT_WITH_CODE((vddc < 2000 && vddc != 0),
					"Invalid VDDC value", result = -EINVAL;);

			/* the voltage should not be zero nor equal to leakage ID */
			if (vddc != 0 && vddc != vv_id) {
				data->vddc_leakage.actual_voltage[data->vddc_leakage.count] = (uint16_t)(vddc/100);
				data->vddc_leakage.leakage_id[data->vddc_leakage.count] = vv_id;
				data->vddc_leakage.count++;
			}
		}
	}

	return 0;
}

/**
 * Change virtual leakage voltage to actual value.
 *
 * @param     hwmgr  the address of the powerplay hardware manager.
 * @param     pointer to changing voltage
 * @param     pointer to leakage table
 */
static void vega10_patch_with_vdd_leakage(struct pp_hwmgr *hwmgr,
		uint16_t *voltage, struct vega10_leakage_voltage *leakage_table)
{
	uint32_t index;

	/* search for leakage voltage ID 0xff01 ~ 0xff08 */
	for (index = 0; index < leakage_table->count; index++) {
		/* if this voltage matches a leakage voltage ID */
		/* patch with actual leakage voltage */
		if (leakage_table->leakage_id[index] == *voltage) {
			*voltage = leakage_table->actual_voltage[index];
			break;
		}
	}

	if (*voltage > ATOM_VIRTUAL_VOLTAGE_ID0)
		pr_info("Voltage value looks like a Leakage ID \
				but it's not patched\n");
}

/**
* Patch voltage lookup table by EVV leakages.
*
* @param     hwmgr  the address of the powerplay hardware manager.
* @param     pointer to voltage lookup table
* @param     pointer to leakage table
* @return     always 0
*/
static int vega10_patch_lookup_table_with_leakage(struct pp_hwmgr *hwmgr,
		phm_ppt_v1_voltage_lookup_table *lookup_table,
		struct vega10_leakage_voltage *leakage_table)
{
	uint32_t i;

	for (i = 0; i < lookup_table->count; i++)
		vega10_patch_with_vdd_leakage(hwmgr,
				&lookup_table->entries[i].us_vdd, leakage_table);

	return 0;
}

static int vega10_patch_clock_voltage_limits_with_vddc_leakage(
		struct pp_hwmgr *hwmgr, struct vega10_leakage_voltage *leakage_table,
		uint16_t *vddc)
{
	vega10_patch_with_vdd_leakage(hwmgr, (uint16_t *)vddc, leakage_table);

	return 0;
}
#endif

static int vega10_patch_voltage_dependency_tables_with_lookup_table(
		struct pp_hwmgr *hwmgr)
{
	uint8_t entry_id;
	uint8_t voltage_id;
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)(hwmgr->pptable);
	struct phm_ppt_v1_clock_voltage_dependency_table *socclk_table =
			table_info->vdd_dep_on_socclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *gfxclk_table =
			table_info->vdd_dep_on_sclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *dcefclk_table =
			table_info->vdd_dep_on_dcefclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *pixclk_table =
			table_info->vdd_dep_on_pixclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *dspclk_table =
			table_info->vdd_dep_on_dispclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *phyclk_table =
			table_info->vdd_dep_on_phyclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *mclk_table =
			table_info->vdd_dep_on_mclk;
	struct phm_ppt_v1_mm_clock_voltage_dependency_table *mm_table =
			table_info->mm_dep_table;

	for (entry_id = 0; entry_id < socclk_table->count; entry_id++) {
		voltage_id = socclk_table->entries[entry_id].vddInd;
		socclk_table->entries[entry_id].vddc =
				table_info->vddc_lookup_table->entries[voltage_id].us_vdd;
	}

	for (entry_id = 0; entry_id < gfxclk_table->count; entry_id++) {
		voltage_id = gfxclk_table->entries[entry_id].vddInd;
		gfxclk_table->entries[entry_id].vddc =
				table_info->vddc_lookup_table->entries[voltage_id].us_vdd;
	}

	for (entry_id = 0; entry_id < dcefclk_table->count; entry_id++) {
		voltage_id = dcefclk_table->entries[entry_id].vddInd;
		dcefclk_table->entries[entry_id].vddc =
				table_info->vddc_lookup_table->entries[voltage_id].us_vdd;
	}

	for (entry_id = 0; entry_id < pixclk_table->count; entry_id++) {
		voltage_id = pixclk_table->entries[entry_id].vddInd;
		pixclk_table->entries[entry_id].vddc =
				table_info->vddc_lookup_table->entries[voltage_id].us_vdd;
	}

	for (entry_id = 0; entry_id < dspclk_table->count; entry_id++) {
		voltage_id = dspclk_table->entries[entry_id].vddInd;
		dspclk_table->entries[entry_id].vddc =
				table_info->vddc_lookup_table->entries[voltage_id].us_vdd;
	}

	for (entry_id = 0; entry_id < phyclk_table->count; entry_id++) {
		voltage_id = phyclk_table->entries[entry_id].vddInd;
		phyclk_table->entries[entry_id].vddc =
				table_info->vddc_lookup_table->entries[voltage_id].us_vdd;
	}

	for (entry_id = 0; entry_id < mclk_table->count; ++entry_id) {
		voltage_id = mclk_table->entries[entry_id].vddInd;
		mclk_table->entries[entry_id].vddc =
				table_info->vddc_lookup_table->entries[voltage_id].us_vdd;
		voltage_id = mclk_table->entries[entry_id].vddciInd;
		mclk_table->entries[entry_id].vddci =
				table_info->vddci_lookup_table->entries[voltage_id].us_vdd;
		voltage_id = mclk_table->entries[entry_id].mvddInd;
		mclk_table->entries[entry_id].mvdd =
				table_info->vddmem_lookup_table->entries[voltage_id].us_vdd;
	}

	for (entry_id = 0; entry_id < mm_table->count; ++entry_id) {
		voltage_id = mm_table->entries[entry_id].vddcInd;
		mm_table->entries[entry_id].vddc =
			table_info->vddc_lookup_table->entries[voltage_id].us_vdd;
	}

	return 0;

}

static int vega10_sort_lookup_table(struct pp_hwmgr *hwmgr,
		struct phm_ppt_v1_voltage_lookup_table *lookup_table)
{
	uint32_t table_size, i, j;
	struct phm_ppt_v1_voltage_lookup_record tmp_voltage_lookup_record;

	PP_ASSERT_WITH_CODE(lookup_table && lookup_table->count,
		"Lookup table is empty", return -EINVAL);

	table_size = lookup_table->count;

	/* Sorting voltages */
	for (i = 0; i < table_size - 1; i++) {
		for (j = i + 1; j > 0; j--) {
			if (lookup_table->entries[j].us_vdd <
					lookup_table->entries[j - 1].us_vdd) {
				tmp_voltage_lookup_record = lookup_table->entries[j - 1];
				lookup_table->entries[j - 1] = lookup_table->entries[j];
				lookup_table->entries[j] = tmp_voltage_lookup_record;
			}
		}
	}

	return 0;
}

static int vega10_complete_dependency_tables(struct pp_hwmgr *hwmgr)
{
	int result = 0;
	int tmp_result;
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)(hwmgr->pptable);
#ifdef PPLIB_VEGA10_EVV_SUPPORT
	struct vega10_hwmgr *data = (struct vega10_hwmgr *)(hwmgr->backend);

	tmp_result = vega10_patch_lookup_table_with_leakage(hwmgr,
			table_info->vddc_lookup_table, &(data->vddc_leakage));
	if (tmp_result)
		result = tmp_result;

	tmp_result = vega10_patch_clock_voltage_limits_with_vddc_leakage(hwmgr,
			&(data->vddc_leakage), &table_info->max_clock_voltage_on_dc.vddc);
	if (tmp_result)
		result = tmp_result;
#endif

	tmp_result = vega10_patch_voltage_dependency_tables_with_lookup_table(hwmgr);
	if (tmp_result)
		result = tmp_result;

	tmp_result = vega10_sort_lookup_table(hwmgr, table_info->vddc_lookup_table);
	if (tmp_result)
		result = tmp_result;

	return result;
}

static int vega10_set_private_data_based_on_pptable(struct pp_hwmgr *hwmgr)
{
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)(hwmgr->pptable);
	struct phm_ppt_v1_clock_voltage_dependency_table *allowed_sclk_vdd_table =
			table_info->vdd_dep_on_socclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *allowed_mclk_vdd_table =
			table_info->vdd_dep_on_mclk;

	PP_ASSERT_WITH_CODE(allowed_sclk_vdd_table,
		"VDD dependency on SCLK table is missing. \
		This table is mandatory", return -EINVAL);
	PP_ASSERT_WITH_CODE(allowed_sclk_vdd_table->count >= 1,
		"VDD dependency on SCLK table is empty. \
		This table is mandatory", return -EINVAL);

	PP_ASSERT_WITH_CODE(allowed_mclk_vdd_table,
		"VDD dependency on MCLK table is missing. \
		This table is mandatory", return -EINVAL);
	PP_ASSERT_WITH_CODE(allowed_mclk_vdd_table->count >= 1,
		"VDD dependency on MCLK table is empty. \
		This table is mandatory", return -EINVAL);

	table_info->max_clock_voltage_on_ac.sclk =
		allowed_sclk_vdd_table->entries[allowed_sclk_vdd_table->count - 1].clk;
	table_info->max_clock_voltage_on_ac.mclk =
		allowed_mclk_vdd_table->entries[allowed_mclk_vdd_table->count - 1].clk;
	table_info->max_clock_voltage_on_ac.vddc =
		allowed_sclk_vdd_table->entries[allowed_sclk_vdd_table->count - 1].vddc;
	table_info->max_clock_voltage_on_ac.vddci =
		allowed_mclk_vdd_table->entries[allowed_mclk_vdd_table->count - 1].vddci;

	hwmgr->dyn_state.max_clock_voltage_on_ac.sclk =
		table_info->max_clock_voltage_on_ac.sclk;
	hwmgr->dyn_state.max_clock_voltage_on_ac.mclk =
		table_info->max_clock_voltage_on_ac.mclk;
	hwmgr->dyn_state.max_clock_voltage_on_ac.vddc =
		table_info->max_clock_voltage_on_ac.vddc;
	hwmgr->dyn_state.max_clock_voltage_on_ac.vddci =
		table_info->max_clock_voltage_on_ac.vddci;

	return 0;
}

static int vega10_hwmgr_backend_fini(struct pp_hwmgr *hwmgr)
{
	kfree(hwmgr->dyn_state.vddc_dep_on_dal_pwrl);
	hwmgr->dyn_state.vddc_dep_on_dal_pwrl = NULL;

	kfree(hwmgr->backend);
	hwmgr->backend = NULL;

	return 0;
}

static int vega10_hwmgr_backend_init(struct pp_hwmgr *hwmgr)
{
	int result = 0;
	struct vega10_hwmgr *data;
	uint32_t config_telemetry = 0;
	struct pp_atomfwctrl_voltage_table vol_table;
	struct cgs_system_info sys_info = {0};

	data = kzalloc(sizeof(struct vega10_hwmgr), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	hwmgr->backend = data;

	vega10_set_default_registry_data(hwmgr);

	data->disable_dpm_mask = 0xff;
	data->workload_mask = 0xff;

	/* need to set voltage control types before EVV patching */
	data->vddc_control = VEGA10_VOLTAGE_CONTROL_NONE;
	data->mvdd_control = VEGA10_VOLTAGE_CONTROL_NONE;
	data->vddci_control = VEGA10_VOLTAGE_CONTROL_NONE;

	/* VDDCR_SOC */
	if (pp_atomfwctrl_is_voltage_controlled_by_gpio_v4(hwmgr,
			VOLTAGE_TYPE_VDDC, VOLTAGE_OBJ_SVID2)) {
		if (!pp_atomfwctrl_get_voltage_table_v4(hwmgr,
				VOLTAGE_TYPE_VDDC, VOLTAGE_OBJ_SVID2,
				&vol_table)) {
			config_telemetry = ((vol_table.telemetry_slope << 8) & 0xff00) |
					(vol_table.telemetry_offset & 0xff);
			data->vddc_control = VEGA10_VOLTAGE_CONTROL_BY_SVID2;
		}
	} else {
		kfree(hwmgr->backend);
		hwmgr->backend = NULL;
		PP_ASSERT_WITH_CODE(false,
				"VDDCR_SOC is not SVID2!",
				return -1);
	}

	/* MVDDC */
	if (pp_atomfwctrl_is_voltage_controlled_by_gpio_v4(hwmgr,
			VOLTAGE_TYPE_MVDDC, VOLTAGE_OBJ_SVID2)) {
		if (!pp_atomfwctrl_get_voltage_table_v4(hwmgr,
				VOLTAGE_TYPE_MVDDC, VOLTAGE_OBJ_SVID2,
				&vol_table)) {
			config_telemetry |=
					((vol_table.telemetry_slope << 24) & 0xff000000) |
					((vol_table.telemetry_offset << 16) & 0xff0000);
			data->mvdd_control = VEGA10_VOLTAGE_CONTROL_BY_SVID2;
		}
	}

	 /* VDDCI_MEM */
	if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_ControlVDDCI)) {
		if (pp_atomfwctrl_is_voltage_controlled_by_gpio_v4(hwmgr,
				VOLTAGE_TYPE_VDDCI, VOLTAGE_OBJ_GPIO_LUT))
			data->vddci_control = VEGA10_VOLTAGE_CONTROL_BY_GPIO;
	}

	data->config_telemetry = config_telemetry;

	vega10_set_features_platform_caps(hwmgr);

	vega10_init_dpm_defaults(hwmgr);

#ifdef PPLIB_VEGA10_EVV_SUPPORT
	/* Get leakage voltage based on leakage ID. */
	PP_ASSERT_WITH_CODE(!vega10_get_evv_voltages(hwmgr),
			"Get EVV Voltage Failed.  Abort Driver loading!",
			return -1);
#endif

	/* Patch our voltage dependency table with actual leakage voltage
	 * We need to perform leakage translation before it's used by other functions
	 */
	vega10_complete_dependency_tables(hwmgr);

	/* Parse pptable data read from VBIOS */
	vega10_set_private_data_based_on_pptable(hwmgr);

	data->is_tlu_enabled = false;

	hwmgr->platform_descriptor.hardwareActivityPerformanceLevels =
			VEGA10_MAX_HARDWARE_POWERLEVELS;
	hwmgr->platform_descriptor.hardwarePerformanceLevels = 2;
	hwmgr->platform_descriptor.minimumClocksReductionPercentage = 50;

	hwmgr->platform_descriptor.vbiosInterruptId = 0x20000400; /* IRQ_SOURCE1_SW_INT */
	/* The true clock step depends on the frequency, typically 4.5 or 9 MHz. Here we use 5. */
	hwmgr->platform_descriptor.clockStep.engineClock = 500;
	hwmgr->platform_descriptor.clockStep.memoryClock = 500;

	sys_info.size = sizeof(struct cgs_system_info);
	sys_info.info_id = CGS_SYSTEM_INFO_GFX_CU_INFO;
	result = cgs_query_system_info(hwmgr->device, &sys_info);
	data->total_active_cus = sys_info.value;
	/* Setup default Overdrive Fan control settings */
	data->odn_fan_table.target_fan_speed =
			hwmgr->thermal_controller.advanceFanControlParameters.usMaxFanRPM;
	data->odn_fan_table.target_temperature =
			hwmgr->thermal_controller.
			advanceFanControlParameters.ucTargetTemperature;
	data->odn_fan_table.min_performance_clock =
			hwmgr->thermal_controller.advanceFanControlParameters.
			ulMinFanSCLKAcousticLimit;
	data->odn_fan_table.min_fan_limit =
			hwmgr->thermal_controller.
			advanceFanControlParameters.usFanPWMMinLimit *
			hwmgr->thermal_controller.fanInfo.ulMaxRPM / 100;

	return result;
}

static int vega10_init_sclk_threshold(struct pp_hwmgr *hwmgr)
{
	struct vega10_hwmgr *data =
			(struct vega10_hwmgr *)(hwmgr->backend);

	data->low_sclk_interrupt_threshold = 0;

	return 0;
}

static int vega10_setup_dpm_led_config(struct pp_hwmgr *hwmgr)
{
	struct vega10_hwmgr *data =
			(struct vega10_hwmgr *)(hwmgr->backend);
	PPTable_t *pp_table = &(data->smc_state_table.pp_table);

	struct pp_atomfwctrl_voltage_table table;
	uint8_t i, j;
	uint32_t mask = 0;
	uint32_t tmp;
	int32_t ret = 0;

	ret = pp_atomfwctrl_get_voltage_table_v4(hwmgr, VOLTAGE_TYPE_LEDDPM,
						VOLTAGE_OBJ_GPIO_LUT, &table);

	if (!ret) {
		tmp = table.mask_low;
		for (i = 0, j = 0; i < 32; i++) {
			if (tmp & 1) {
				mask |= (uint32_t)(i << (8 * j));
				if (++j >= 3)
					break;
			}
			tmp >>= 1;
		}
	}

	pp_table->LedPin0 = (uint8_t)(mask & 0xff);
	pp_table->LedPin1 = (uint8_t)((mask >> 8) & 0xff);
	pp_table->LedPin2 = (uint8_t)((mask >> 16) & 0xff);
	return 0;
}

static int vega10_setup_asic_task(struct pp_hwmgr *hwmgr)
{
	PP_ASSERT_WITH_CODE(!vega10_init_sclk_threshold(hwmgr),
			"Failed to init sclk threshold!",
			return -EINVAL);

	PP_ASSERT_WITH_CODE(!vega10_setup_dpm_led_config(hwmgr),
			"Failed to set up led dpm config!",
			return -EINVAL);

	return 0;
}

static bool vega10_is_dpm_running(struct pp_hwmgr *hwmgr)
{
	uint32_t features_enabled;

	if (!vega10_get_smc_features(hwmgr->smumgr, &features_enabled)) {
		if (features_enabled & SMC_DPM_FEATURES)
			return true;
	}
	return false;
}

/**
* Remove repeated voltage values and create table with unique values.
*
* @param    hwmgr  the address of the powerplay hardware manager.
* @param    vol_table  the pointer to changing voltage table
* @return    0 in success
*/

static int vega10_trim_voltage_table(struct pp_hwmgr *hwmgr,
		struct pp_atomfwctrl_voltage_table *vol_table)
{
	uint32_t i, j;
	uint16_t vvalue;
	bool found = false;
	struct pp_atomfwctrl_voltage_table *table;

	PP_ASSERT_WITH_CODE(vol_table,
			"Voltage Table empty.", return -EINVAL);
	table = kzalloc(sizeof(struct pp_atomfwctrl_voltage_table),
			GFP_KERNEL);

	if (!table)
		return -ENOMEM;

	table->mask_low = vol_table->mask_low;
	table->phase_delay = vol_table->phase_delay;

	for (i = 0; i < vol_table->count; i++) {
		vvalue = vol_table->entries[i].value;
		found = false;

		for (j = 0; j < table->count; j++) {
			if (vvalue == table->entries[j].value) {
				found = true;
				break;
			}
		}

		if (!found) {
			table->entries[table->count].value = vvalue;
			table->entries[table->count].smio_low =
					vol_table->entries[i].smio_low;
			table->count++;
		}
	}

	memcpy(vol_table, table, sizeof(struct pp_atomfwctrl_voltage_table));
	kfree(table);

	return 0;
}

static int vega10_get_mvdd_voltage_table(struct pp_hwmgr *hwmgr,
		phm_ppt_v1_clock_voltage_dependency_table *dep_table,
		struct pp_atomfwctrl_voltage_table *vol_table)
{
	int i;

	PP_ASSERT_WITH_CODE(dep_table->count,
			"Voltage Dependency Table empty.",
			return -EINVAL);

	vol_table->mask_low = 0;
	vol_table->phase_delay = 0;
	vol_table->count = dep_table->count;

	for (i = 0; i < vol_table->count; i++) {
		vol_table->entries[i].value = dep_table->entries[i].mvdd;
		vol_table->entries[i].smio_low = 0;
	}

	PP_ASSERT_WITH_CODE(!vega10_trim_voltage_table(hwmgr,
			vol_table),
			"Failed to trim MVDD Table!",
			return -1);

	return 0;
}

static int vega10_get_vddci_voltage_table(struct pp_hwmgr *hwmgr,
		phm_ppt_v1_clock_voltage_dependency_table *dep_table,
		struct pp_atomfwctrl_voltage_table *vol_table)
{
	uint32_t i;

	PP_ASSERT_WITH_CODE(dep_table->count,
			"Voltage Dependency Table empty.",
			return -EINVAL);

	vol_table->mask_low = 0;
	vol_table->phase_delay = 0;
	vol_table->count = dep_table->count;

	for (i = 0; i < dep_table->count; i++) {
		vol_table->entries[i].value = dep_table->entries[i].vddci;
		vol_table->entries[i].smio_low = 0;
	}

	PP_ASSERT_WITH_CODE(!vega10_trim_voltage_table(hwmgr, vol_table),
			"Failed to trim VDDCI table.",
			return -1);

	return 0;
}

static int vega10_get_vdd_voltage_table(struct pp_hwmgr *hwmgr,
		phm_ppt_v1_clock_voltage_dependency_table *dep_table,
		struct pp_atomfwctrl_voltage_table *vol_table)
{
	int i;

	PP_ASSERT_WITH_CODE(dep_table->count,
			"Voltage Dependency Table empty.",
			return -EINVAL);

	vol_table->mask_low = 0;
	vol_table->phase_delay = 0;
	vol_table->count = dep_table->count;

	for (i = 0; i < vol_table->count; i++) {
		vol_table->entries[i].value = dep_table->entries[i].vddc;
		vol_table->entries[i].smio_low = 0;
	}

	return 0;
}

/* ---- Voltage Tables ----
 * If the voltage table would be bigger than
 * what will fit into the state table on
 * the SMC keep only the higher entries.
 */
static void vega10_trim_voltage_table_to_fit_state_table(
		struct pp_hwmgr *hwmgr,
		uint32_t max_vol_steps,
		struct pp_atomfwctrl_voltage_table *vol_table)
{
	unsigned int i, diff;

	if (vol_table->count <= max_vol_steps)
		return;

	diff = vol_table->count - max_vol_steps;

	for (i = 0; i < max_vol_steps; i++)
		vol_table->entries[i] = vol_table->entries[i + diff];

	vol_table->count = max_vol_steps;
}

/**
* Create Voltage Tables.
*
* @param    hwmgr  the address of the powerplay hardware manager.
* @return   always 0
*/
static int vega10_construct_voltage_tables(struct pp_hwmgr *hwmgr)
{
	struct vega10_hwmgr *data = (struct vega10_hwmgr *)(hwmgr->backend);
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)hwmgr->pptable;
	int result;

	if (data->mvdd_control == VEGA10_VOLTAGE_CONTROL_BY_SVID2 ||
			data->mvdd_control == VEGA10_VOLTAGE_CONTROL_NONE) {
		result = vega10_get_mvdd_voltage_table(hwmgr,
				table_info->vdd_dep_on_mclk,
				&(data->mvdd_voltage_table));
		PP_ASSERT_WITH_CODE(!result,
				"Failed to retrieve MVDDC table!",
				return result);
	}

	if (data->vddci_control == VEGA10_VOLTAGE_CONTROL_NONE) {
		result = vega10_get_vddci_voltage_table(hwmgr,
				table_info->vdd_dep_on_mclk,
				&(data->vddci_voltage_table));
		PP_ASSERT_WITH_CODE(!result,
				"Failed to retrieve VDDCI_MEM table!",
				return result);
	}

	if (data->vddc_control == VEGA10_VOLTAGE_CONTROL_BY_SVID2 ||
			data->vddc_control == VEGA10_VOLTAGE_CONTROL_NONE) {
		result = vega10_get_vdd_voltage_table(hwmgr,
				table_info->vdd_dep_on_sclk,
				&(data->vddc_voltage_table));
		PP_ASSERT_WITH_CODE(!result,
				"Failed to retrieve VDDCR_SOC table!",
				return result);
	}

	PP_ASSERT_WITH_CODE(data->vddc_voltage_table.count <= 16,
			"Too many voltage values for VDDC. Trimming to fit state table.",
			vega10_trim_voltage_table_to_fit_state_table(hwmgr,
					16, &(data->vddc_voltage_table)));

	PP_ASSERT_WITH_CODE(data->vddci_voltage_table.count <= 16,
			"Too many voltage values for VDDCI. Trimming to fit state table.",
			vega10_trim_voltage_table_to_fit_state_table(hwmgr,
					16, &(data->vddci_voltage_table)));

	PP_ASSERT_WITH_CODE(data->mvdd_voltage_table.count <= 16,
			"Too many voltage values for MVDD. Trimming to fit state table.",
			vega10_trim_voltage_table_to_fit_state_table(hwmgr,
					16, &(data->mvdd_voltage_table)));


	return 0;
}

/*
 * @fn vega10_init_dpm_state
 * @brief Function to initialize all Soft Min/Max and Hard Min/Max to 0xff.
 *
 * @param    dpm_state - the address of the DPM Table to initiailize.
 * @return   None.
 */
static void vega10_init_dpm_state(struct vega10_dpm_state *dpm_state)
{
	dpm_state->soft_min_level = 0xff;
	dpm_state->soft_max_level = 0xff;
	dpm_state->hard_min_level = 0xff;
	dpm_state->hard_max_level = 0xff;
}

static void vega10_setup_default_single_dpm_table(struct pp_hwmgr *hwmgr,
		struct vega10_single_dpm_table *dpm_table,
		struct phm_ppt_v1_clock_voltage_dependency_table *dep_table)
{
	int i;

	for (i = 0; i < dep_table->count; i++) {
		if (i == 0 || dpm_table->dpm_levels[dpm_table->count - 1].value <=
				dep_table->entries[i].clk) {
			dpm_table->dpm_levels[dpm_table->count].value =
					dep_table->entries[i].clk;
			dpm_table->dpm_levels[dpm_table->count].enabled = true;
			dpm_table->count++;
		}
	}
}
static int vega10_setup_default_pcie_table(struct pp_hwmgr *hwmgr)
{
	struct vega10_hwmgr *data =
			(struct vega10_hwmgr *)(hwmgr->backend);
	struct vega10_pcie_table *pcie_table = &(data->dpm_table.pcie_table);
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)(hwmgr->pptable);
	struct phm_ppt_v1_pcie_table *bios_pcie_table =
			table_info->pcie_table;
	uint32_t i;

	PP_ASSERT_WITH_CODE(bios_pcie_table->count,
			"Incorrect number of PCIE States from VBIOS!",
			return -1);

	for (i = 0; i < NUM_LINK_LEVELS; i++) {
		if (data->registry_data.pcieSpeedOverride)
			pcie_table->pcie_gen[i] =
					data->registry_data.pcieSpeedOverride;
		else
			pcie_table->pcie_gen[i] =
					bios_pcie_table->entries[i].gen_speed;

		if (data->registry_data.pcieLaneOverride)
			pcie_table->pcie_lane[i] = (uint8_t)encode_pcie_lane_width(
					data->registry_data.pcieLaneOverride);
		else
			pcie_table->pcie_lane[i] = (uint8_t)encode_pcie_lane_width(
							bios_pcie_table->entries[i].lane_width);
		if (data->registry_data.pcieClockOverride)
			pcie_table->lclk[i] =
					data->registry_data.pcieClockOverride;
		else
			pcie_table->lclk[i] =
					bios_pcie_table->entries[i].pcie_sclk;
	}

	pcie_table->count = NUM_LINK_LEVELS;

	return 0;
}

/*
 * This function is to initialize all DPM state tables
 * for SMU based on the dependency table.
 * Dynamic state patching function will then trim these
 * state tables to the allowed range based
 * on the power policy or external client requests,
 * such as UVD request, etc.
 */
static int vega10_setup_default_dpm_tables(struct pp_hwmgr *hwmgr)
{
	struct vega10_hwmgr *data =
			(struct vega10_hwmgr *)(hwmgr->backend);
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)(hwmgr->pptable);
	struct vega10_single_dpm_table *dpm_table;
	uint32_t i;

	struct phm_ppt_v1_clock_voltage_dependency_table *dep_soc_table =
			table_info->vdd_dep_on_socclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *dep_gfx_table =
			table_info->vdd_dep_on_sclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *dep_mclk_table =
			table_info->vdd_dep_on_mclk;
	struct phm_ppt_v1_mm_clock_voltage_dependency_table *dep_mm_table =
			table_info->mm_dep_table;
	struct phm_ppt_v1_clock_voltage_dependency_table *dep_dcef_table =
			table_info->vdd_dep_on_dcefclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *dep_pix_table =
			table_info->vdd_dep_on_pixclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *dep_disp_table =
			table_info->vdd_dep_on_dispclk;
	struct phm_ppt_v1_clock_voltage_dependency_table *dep_phy_table =
			table_info->vdd_dep_on_phyclk;

	PP_ASSERT_WITH_CODE(dep_soc_table,
			"SOCCLK dependency table is missing. This table is mandatory",
			return -EINVAL);
	PP_ASSERT_WITH_CODE(dep_soc_table->count >= 1,
			"SOCCLK dependency table is empty. This table is mandatory",
			return -EINVAL);

	PP_ASSERT_WITH_CODE(dep_gfx_table,
			"GFXCLK dependency table is missing. This table is mandatory",
			return -EINVAL);
	PP_ASSERT_WITH_CODE(dep_gfx_table->count >= 1,
			"GFXCLK dependency table is empty. This table is mandatory",
			return -EINVAL);

	PP_ASSERT_WITH_CODE(dep_mclk_table,
			"MCLK dependency table is missing. This table is mandatory",
			return -EINVAL);
	PP_ASSERT_WITH_CODE(dep_mclk_table->count >= 1,
			"MCLK dependency table has to have is missing. This table is mandatory",
			return -EINVAL);

	/* Initialize Sclk DPM table based on allow Sclk values */
	data->dpm_table.soc_table.count = 0;
	data->dpm_table.gfx_table.count = 0;
	data->dpm_table.dcef_table.count = 0;

	dpm_table = &(data->dpm_table.soc_table);
	vega10_setup_default_single_dpm_table(hwmgr,
			dpm_table,
			dep_soc_table);

	vega10_init_dpm_state(&(dpm_table->dpm_state));

	dpm_table = &(data->dpm_table.gfx_table);
	vega10_setup_default_single_dpm_table(hwmgr,
			dpm_table,
			dep_gfx_table);
	vega10_init_dpm_state(&(dpm_table->dpm_state));

	/* Initialize Mclk DPM table based on allow Mclk values */
	data->dpm_table.mem_table.count = 0;
	dpm_table = &(data->dpm_table.mem_table);
	vega10_setup_default_single_dpm_table(hwmgr,
			dpm_table,
			dep_mclk_table);
	vega10_init_dpm_state(&(dpm_table->dpm_state));

	data->dpm_table.eclk_table.count = 0;
	dpm_table = &(data->dpm_table.eclk_table);
	for (i = 0; i < dep_mm_table->count; i++) {
		if (i == 0 || dpm_table->dpm_levels
				[dpm_table->count - 1].value <=
						dep_mm_table->entries[i].eclk) {
			dpm_table->dpm_levels[dpm_table->count].value =
					dep_mm_table->entries[i].eclk;
			dpm_table->dpm_levels[dpm_table->count].enabled =
					(i == 0) ? true : false;
			dpm_table->count++;
		}
	}
	vega10_init_dpm_state(&(dpm_table->dpm_state));

	data->dpm_table.vclk_table.count = 0;
	data->dpm_table.dclk_table.count = 0;
	dpm_table = &(data->dpm_table.vclk_table);
	for (i = 0; i < dep_mm_table->count; i++) {
		if (i == 0 || dpm_table->dpm_levels
				[dpm_table->count - 1].value <=
						dep_mm_table->entries[i].vclk) {
			dpm_table->dpm_levels[dpm_table->count].value =
					dep_mm_table->entries[i].vclk;
			dpm_table->dpm_levels[dpm_table->count].enabled =
					(i == 0) ? true : false;
			dpm_table->count++;
		}
	}
	vega10_init_dpm_state(&(dpm_table->dpm_state));

	dpm_table = &(data->dpm_table.dclk_table);
	for (i = 0; i < dep_mm_table->count; i++) {
		if (i == 0 || dpm_table->dpm_levels
				[dpm_table->count - 1].value <=
						dep_mm_table->entries[i].dclk) {
			dpm_table->dpm_levels[dpm_table->count].value =
					dep_mm_table->entries[i].dclk;
			dpm_table->dpm_levels[dpm_table->count].enabled =
					(i == 0) ? true : false;
			dpm_table->count++;
		}
	}
	vega10_init_dpm_state(&(dpm_table->dpm_state));

	/* Assume there is no headless Vega10 for now */
	dpm_table = &(data->dpm_table.dcef_table);
	vega10_setup_default_single_dpm_table(hwmgr,
			dpm_table,
			dep_dcef_table);

	vega10_init_dpm_state(&(dpm_table->dpm_state));

	dpm_table = &(data->dpm_table.pixel_table);
	vega10_setup_default_single_dpm_table(hwmgr,
			dpm_table,
			dep_pix_table);

	vega10_init_dpm_state(&(dpm_table->dpm_state));

	dpm_table = &(data->dpm_table.display_table);
	vega10_setup_default_single_dpm_table(hwmgr,
			dpm_table,
			dep_disp_table);

	vega10_init_dpm_state(&(dpm_table->dpm_state));

	dpm_table = &(data->dpm_table.phy_table);
	vega10_setup_default_single_dpm_table(hwmgr,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(&(dpm_table->dpm_state));

	vega10_setup_default_pcie_table(hwmgr);

	/* save a copy of the default DPM table */
	memcpy(&(data->golden_dpm_table), &(data->dpm_table),
			sizeof(struct vega10_dpm_table));

	if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_ODNinACSupport) ||
		phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			PHM_PlatformCaps_ODNinDCSupport)) {
		data->odn_dpm_table.odn_core_clock_dpm_levels.
		number_of_performance_levels = data->dpm_table.gfx_table.count;
		for (i = 0; i < data->dpm_table.gfx_table.count; i++) {
			data->odn_dpm_table.odn_core_clock_dpm_levels.
			performance_level_entries[i].clock =
					data->dpm_table.gfx_table.dpm_levels[i].value;
			data->odn_dpm_table.odn_core_clock_dpm_levels.
			performance_level_entries[i].enabled = true;
		}

		data->odn_dpm_table.vdd_dependency_on_sclk.count =
				dep_gfx_table->count;
		for (i = 0; i < dep_gfx_table->count; i++) {
			data->odn_dpm_table.vdd_dependency_on_sclk.entries[i].clk =
					dep_gfx_table->entries[i].clk;
			data->odn_dpm_table.vdd_dependency_on_sclk.entries[i].vddInd =
					dep_gfx_table->entries[i].vddInd;
			data->odn_dpm_table.vdd_dependency_on_sclk.entries[i].cks_enable =
					dep_gfx_table->entries[i].cks_enable;
			data->odn_dpm_table.vdd_dependency_on_sclk.entries[i].cks_voffset =
					dep_gfx_table->entries[i].cks_voffset;
		}

		data->odn_dpm_table.odn_memory_clock_dpm_levels.
		number_of_performance_levels = data->dpm_table.mem_table.count;
		for (i = 0; i < data->dpm_table.mem_table.count; i++) {
			data->odn_dpm_table.odn_memory_clock_dpm_levels.
			performance_level_entries[i].clock =
					data->dpm_table.mem_table.dpm_levels[i].value;
			data->odn_dpm_table.odn_memory_clock_dpm_levels.
			performance_level_entries[i].enabled = true;
		}

		data->odn_dpm_table.vdd_dependency_on_mclk.count = dep_mclk_table->count;
		for (i = 0; i < dep_mclk_table->count; i++) {
			data->odn_dpm_table.vdd_dependency_on_mclk.entries[i].clk =
					dep_mclk_table->entries[i].clk;
			data->odn_dpm_table.vdd_dependency_on_mclk.entries[i].vddInd =
					dep_mclk_table->entries[i].vddInd;
			data->odn_dpm_table.vdd_dependency_on_mclk.entries[i].vddci =
					dep_mclk_table->entries[i].vddci;
		}
	}

	return 0;
}

/*
 * @fn vega10_populate_ulv_state
 * @brief Function to provide parameters for Utral Low Voltage state to SMC.
 *
 * @param    hwmgr - the address of the hardware manager.
 * @return   Always 0.
 */
static int vega10_populate_ulv_state(struct pp_hwmgr *hwmgr)
{
	struct vega10_hwmgr *data =
			(struct vega10_hwmgr *)(hwmgr->backend);
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)(hwmgr->pptable);

	data->smc_state_table.pp_table.UlvOffsetVid =
			(uint8_t)table_info->us_ulv_voltage_offset;

	data->smc_state_table.pp_table.UlvSmnclkDid =
			(uint8_t)(table_info->us_ulv_smnclk_did);
	data->smc_state_table.pp_table.UlvMp1clkDid =
			(uint8_t)(table_info->us_ulv_mp1clk_did);
	data->smc_state_table.pp_table.UlvGfxclkBypass =
			(uint8_t)(table_info->us_ulv_gfxclk_bypass);
	data->smc_state_table.pp_table.UlvPhaseSheddingPsi0 =
			(uint8_t)(data->vddc_voltage_table.psi0_enable);
	data->smc_state_table.pp_table.UlvPhaseSheddingPsi1 =
			(uint8_t)(data->vddc_voltage_table.psi1_enable);

	return 0;
}

static int vega10_populate_single_lclk_level(struct pp_hwmgr *hwmgr,
		uint32_t lclock, uint8_t *curr_lclk_did)
{
	struct pp_atomfwctrl_clock_dividers_soc15 dividers;

	PP_ASSERT_WITH_CODE(!pp_atomfwctrl_get_gpu_pll_dividers_vega10(
			hwmgr,
			COMPUTE_GPUCLK_INPUT_FLAG_DEFAULT_GPUCLK,
			lclock, &dividers),
			"Failed to get LCLK clock settings from VBIOS!",
			return -1);

	*curr_lclk_did = dividers.ulDid;

	return 0;
}

static int vega10_populate_smc_link_levels(struct pp_hwmgr *hwmgr)
{
	int result = -1;
	struct vega10_hwmgr *data =
			(struct vega10_hwmgr *)(hwmgr->backend);
	PPTable_t *pp_table = &(data->smc_state_table.pp_table);
	struct vega10_pcie_table *pcie_table =
			&(data->dpm_table.pcie_table);
	uint32_t i, j;

	for (i = 0; i < pcie_table->count; i++) {
		pp_table->PcieGenSpeed[i] = pcie_table->pcie_gen[i];
		pp_table->PcieLaneCount[i] = pcie_table->pcie_lane[i];

		result = vega10_populate_single_lclk_level(hwmgr,
				pcie_table->lclk[i], &(pp_table->LclkDid[i]));
		if (result) {
			pr_info("Populate LClock Level %d Failed!\n", i);
			return result;
		}
	}

	j = i - 1;
	while (i < NUM_LINK_LEVELS) {
		pp_table->PcieGenSpeed[i] = pcie_table->pcie_gen[j];
		pp_table->PcieLaneCount[i] = pcie_table->pcie_lane[j];

		result = vega10_populate_single_lclk_level(hwmgr,
				pcie_table->lclk[j], &(pp_table->LclkDid[i]));
		if (result) {
			pr_info("Populate LClock Level %d Failed!\n", i);
			return result;
		}
		i++;
	}

	return result;
}

/**
* Populates single SMC GFXSCLK structure using the provided engine clock
*
* @param    hwmgr      the address of the hardware manager
* @param    gfx_clock  the GFX clock to use to populate the structure.
* @param    current_gfxclk_level  location in PPTable for the SMC GFXCLK structure.
*/

static int vega10_populate_single_gfx_level(struct pp_hwmgr *hwmgr,
		uint32_t gfx_clock, PllSetting_t *current_gfxclk_level,
		uint32_t *acg_freq)
{
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)(hwmgr->pptable);
	struct phm_ppt_v1_clock_voltage_dependency_table *dep_on_sclk =
			table_info->vdd_dep_on_sclk;
	struct vega10_hwmgr *data =
			(struct vega10_hwmgr *)(hwmgr->backend);
	struct pp_atomfwctrl_clock_dividers_soc15 dividers;
	uint32_t gfx_max_clock =
			hwmgr->platform_descriptor.overdriveLimit.engineClock;
	uint32_t i = 0;

	if (data->apply_overdrive_next_settings_mask &
			DPMTABLE_OD_UPDATE_VDDC)
		dep_on_sclk = (struct phm_ppt_v1_clock_voltage_dependency_table *)
						&(data->odn_dpm_table.vdd_dependency_on_sclk);

	PP_ASSERT_WITH_CODE(dep_on_sclk,
			"Invalid SOC_VDD-GFX_CLK Dependency Table!",
			return -EINVAL);

	if (data->need_update_dpm_table & DPMTABLE_OD_UPDATE_SCLK)
		gfx_clock = gfx_clock > gfx_max_clock ? gfx_max_clock : gfx_clock;
	else {
		for (i = 0; i < dep_on_sclk->count; i++) {
			if (dep_on_sclk->entries[i].clk == gfx_clock)
				break;
		}
		PP_ASSERT_WITH_CODE(dep_on_sclk->count > i,
				"Cannot find gfx_clk in SOC_VDD-GFX_CLK!",
				return -EINVAL);
	}

	PP_ASSERT_WITH_CODE(!pp_atomfwctrl_get_gpu_pll_dividers_vega10(hwmgr,
			COMPUTE_GPUCLK_INPUT_FLAG_GFXCLK,
			gfx_clock, &dividers),
			"Failed to get GFX Clock settings from VBIOS!",
			return -EINVAL);

	/* Feedback Multiplier: bit 0:8 int, bit 15:12 post_div, bit 31:16 frac */
	current_gfxclk_level->FbMult =
			cpu_to_le32(dividers.ulPll_fb_mult);
	/* Spread FB Multiplier bit: bit 0:8 int, bit 31:16 frac */
	current_gfxclk_level->SsOn = dividers.ucPll_ss_enable;
	current_gfxclk_level->SsFbMult =
			cpu_to_le32(dividers.ulPll_ss_fbsmult);
	current_gfxclk_level->SsSlewFrac =
			cpu_to_le16(dividers.usPll_ss_slew_frac);
	current_gfxclk_level->Did = (uint8_t)(dividers.ulDid);

	*acg_freq = gfx_clock / 100; /* 100 Khz to Mhz conversion */

	return 0;
}

/**
 * @brief Populates single SMC SOCCLK structure using the provided clock.
 *
 * @param    hwmgr - the address of the hardware manager.
 * @param    soc_clock - the SOC clock to use to populate the structure.
 * @param    current_socclk_level - location in PPTable for the SMC SOCCLK structure.
 * @return   0 on success..
 */
static int vega10_populate_single_soc_level(struct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
		uint8_t *current_vol_index)
{
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)(hwmgr->pptable);
	struct phm_ppt_v1_clock_voltage_dependency_table *dep_on_soc =
			table_info->vdd_dep_on_socclk;
	struct pp_atomfwctrl_clock_dividers_soc15 dividers;
	uint32_t i;

	PP_ASSERT_WITH_CODE(dep_on_soc,
			"Invalid SOC_VDD-SOC_CLK Dependency Table!",
			return -EINVAL);
	for (i = 0; i < dep_on_soc->count; i++) {
		if (dep_on_soc->entries[i].clk == soc_clock)
			break;
	}
	PP_ASSERT_WITH_CODE(dep_on_soc->count > i,
			"Cannot find SOC_CLK in SOC_VDD-SOC_CLK Dependency Table",
			return -EINVAL);
	PP_ASSERT_WITH_CODE(!pp_atomfwctrl_get_gpu_pll_dividers_vega10(hwmgr,
			COMPUTE_GPUCLK_INPUT_FLAG_DEFAULT_GPUCLK,
			soc_clock, &dividers),
			"Failed to get SOC Clock settings from VBIOS!",
			return -EINVAL);

	*current_soc_did = (uint8_t)dividers.ulDid;
	*current_vol_index = (uint8_t)(dep_on_soc->entries[i].vddInd);

	return 0;
}

uint16_t vega10_locate_vddc_given_clock(struct pp_hwmgr *hwmgr,
		uint32_t clk,
		struct phm_ppt_v1_clock_voltage_dependency_table *dep_table)
{
	uint16_t i;

	for (i = 0; i < dep_table->count; i++) {
		if (dep_table->entries[i].clk == clk)
			return dep_table->entries[i].vddc;
	}

	pr_info("[LocateVddcGivenClock] Cannot locate SOC Vddc for this clock!");
	return 0;
}

/**
* Populates all SMC SCLK levels' structure based on the trimmed allowed dpm engine clock states
*
* @param    hwmgr      the address of the hardware manager
*/
static int vega10_populate_all_graphic_levels(struct pp_hwmgr *hwmgr)
{
	struct vega10_hwmgr *data =
			(struct vega10_hwmgr *)(hwmgr->backend);
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)(hwmgr->pptable);
	struct phm_ppt_v1_clock_voltage_dependency_table *dep_table =
			table_info->vdd_dep_on_socclk;
	PPTable_t *pp_table = &(data->smc_state_table.pp_table);
	struct vega10_single_dpm_table *dpm_table = &(data->dpm_table.gfx_table);
	int result = 0;
	uint32_t i, j;

	for (i = 0; i < dpm_table->count; i++) {
		result = vega10_populate_single_gfx_level(hwmgr,
				dpm_table->dpm_levels[i].value,
				&(pp_table->GfxclkLevel[i]),
				&(pp_table->AcgFreqTable[i]));
		if (result)
			return result;
	}

	j = i - 1;
	while (i < NUM_GFXCLK_DPM_LEVELS) {
		result = vega10_populate_single_gfx_level(hwmgr,
				dpm_table->dpm_levels[j].value,
				&(pp_table->GfxclkLevel[i]),
				&(pp_table->AcgFreqTable[i]));
		if (result)
			return result;
		i++;
	}

	pp_table->GfxclkSlewRate =
			cpu_to_le16(table_info->us_gfxclk_slew_rate);

	dpm_table = &(data->dpm_table.soc_table);
	for (i = 0; i < dpm_table->count; i++) {
		pp_table->SocVid[i] =
				(uint8_t)convert_to_vid(
				vega10_locate_vddc_given_clock(hwmgr,
						dpm_table->dpm_levels[i].value,
						dep_table));
		result = vega10_populate_single_soc_level(hwmgr,
				dpm_table->dpm_levels[i].value,
				&(pp_table->SocclkDid[i]),
				&(pp_table->SocDpmVoltageIndex[i]));
		if (result)
			return result;
	}

	j = i - 1;
	while (i < NUM_SOCCLK_DPM_LEVELS) {
		pp_table->SocVid[i] = pp_table->SocVid[j];
		result = vega10_populate_single_soc_level(hwmgr,
				dpm_table->dpm_levels[j].value,
				&(pp_table->SocclkDid[i]),
				&(pp_table->SocDpmVoltageIndex[i]));
		if (result)
			return result;
		i++;
	}

	return result;
}

/**
 * @brief Populates single SMC GFXCLK structure using the provided clock.
 *
 * @param    hwmgr - the address of the hardware manager.
 * @param    mem_clock - the memory clock to use to populate the structure.
 * @return   0 on success..
 */
static int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		uint32_t mem_clock, uint8_t *current_mem_vid,
		PllSetting_t *current_memclk_level, uint8_t *current_mem_soc_vind)
{
	struct vega10_hwmgr *data =
			(struct vega10_hwmgr *)(hwmgr->backend);
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information *)(hwmgr->pptable);
	struct phm_ppt_v1_clock_voltage_dependency_table *dep_on_mclk =
			table_info->vdd_dep_on_mclk;
	struct pp_atomfwctrl_clock_dividers_soc15 dividers;
	uint32_t mem_max_clock =
			hwmgr->platform_descriptor.overdriveLimit.memoryClock;
	uint32_t i = 0;

	if (data->apply_overdrive_next_settings_mask &
			DPMTABLE_OD_UPDATE_VDDC)
		dep_on_mclk = (struct phm_ppt_v1_clock_voltage_dependency_table *)
					&data->odn_dpm_table.vdd_dependency_on_mclk;

	PP_ASSERT_WITH_CODE(dep_on_mclk,
			"Invalid SOC_VDD-UCLK Dependency Table!",
			return -EINVAL);

	if (data->need_update_dpm_table & DPMTABLE_OD_UPDATE_MCLK)
		mem_clock = mem_clock > mem_max_clock ? mem_max_clock : mem_clock;
	else {
		for (i = 0; i < dep_on_mclk->count; i++) {
			if (dep_on_mclk->entries[i].clk == mem_clock)
				break;
		}
		PP_ASSERT_WITH_CODE(dep_on_mclk->count > i,
				"Cannot find UCLK in SOC_VDD-UCLK Dependency Table!",
				return -EINVAL);
	}

	PP_ASSERT_WITH_CODE(!pp_atomfwctrl_get_gpu_pll_dividers_vega10(
			hwmgr, COMPUTE_GPUCLK_INPUT_FLAG_UCLK, mem_clock, &dividers),
			"Failed to get UCLK settings from VBIOS!",
			return -1);

	*current_mem_vid =
			(uint8_t)(convert_to_vid(dep_on_mclk->entries[i].mvdd));
	*current_mem_soc_vind =
			(uint8_t)(dep_on_mclk->entries[i]. pp_atomfP7_W8h* Populate_W8yS>yiy8yS>yS>yS>gT_G8VH_CODE(dep_on_mclk,
			"Invalid SG8hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh,Wcture.
 * @return   0 on success..
 */
static int vega10_populate_single_soc_level(struct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
static int vega10_populv(table_in),dyries[i].vddc;
	}

	p	/
stle_ailed to get SOC Clock sett vega10_hwmg0_VOLTAGE_CONTROL_BY_SVrent_soc_did,
	tu: mem_c_fbsmult);
	current_gfxclkn 0;nt_soc_did,
	tu: mem_c_fbsP7_x_did,
	tu: mem_c_fbsP7_x_di8wn_mclk->count > i,
				"Cannot finn_mclk->count > i,
				"Cannot find UCLK in SOC_VDD-UCLK Dependency Table!",
				return -EINVAL);
	}

	PP_ASSERT_WITH_CODE(!pp_ato.vdd_depend8yclock_divide_W8yS>yS>yS>yS>yS>V> i,
				"Cannot find UCLK in SOC_VDD-UCLvide_W8
urn -EINVAL);
	}

	PP_ASSERT_WITH_CODE(!pp_ato.vdd_depend8OG8h ourn -EI
ucture.
 * @return   0 on success..
 */
s *current_soc_did,
		uint8_t *current_vol_index)
{
	struct phm_ppt_v2_information *table_info =
			(struct phm_ppt_v2_information I.onfo3GS>vgs_mask &
			DPMTABLE_OD_UPDAtage>yRS>gifind>yS>yS>yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy,W8yS>biy8g&{
	_v_7_W8yS>yS>v MVDD Table!",
			rDgfx_clock = gfx_clock_oy8g&{
	_v_7_W8
	 yyyyy_7_W8VGaGaK) >>ock = gfx_clock_oy8g&{
	_v_7_W8
	 yyyyy_7_W8VG_SHIFT;)
			return y_7Mruct p>biy8g&{
	 1;
	while (,W8yS>biy8g&);)
			return Mruct p>biy8gWspee = i - 1;
	while (HBM tabORY_CHANNEL =
DTHs.ucTa - >biy8g__table[,W8yS>biy8g&]=
			\n", i);
		ep_strrenn  ervedForUlvmp1clk_did);
	data->smlep_st_ut_tor  erved_tu:offs=
			(uint8_ =
			hwmgr->thermal_controllWITH_CODE(!pp_atdcef_tablype.
 */
static void vega10_inDSPide_e ->dpmUCLK Dent32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
static int vega10_populv(table_in),dyries[i].vddc;
	}

	p	/
stle_ailedtatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		uory_l    the address of the hardware manager
*/
static int vega10_populate_all_gra*current_vol_in
	*currenttable_0, esul_fbsmult);

 * LK inon *)(swiform(->dpmUCLK DVAL)c&{
 DSPide_DCEFide:
	cpu_to_le1  (!tabluct phm_ppt_v1_clock_voltagedency Tabc&{
 DSPide_DISPide:
	cpu_to_le1  (!tabluct phm_ppt_v1_clocage_dependency Tabc&{
 DSPide_PIXide:
	cpu_to_le1  (!tabluct phm_ppt_v1_cloock_voltadency Tabc&{
 DSPide_PHYide:
	cpu_to_le1  (!tabluct phm_ppt_v1_clootage_deadency Tabtate(&(:
ount > i,
	 ? mem_max_clock : mem_clocividers.ulDid;
	RT_W-UCLDSPide_rn -EIk->entNm_tablOf Ers_veg Excgs__onncyga10m_ppt_v2_information *table_info =
		ividers.ulDid;
	*current_votable_i	*curren)ndex = (uint8_t)(dep_on_socsOn =ble_iesul_fb!tabluct phm_ppc_lookux = (uintucTa 8_t)(depd);

	return 0;
}

uint16ati].us_E(dep_ta VBIOS!",
			rDependency Tabl_ppc)te_single_lclkDcef_taphm_pl[i]),->dpmUCLK ]
uinF_gfxcucTa  1;
	while (=
			h_single_lclkDcef_taphm_pl[i]),->dpmUCLK ]
uinint vega1  1;
	while (va->smcmem_ in SOC_VDD-UCLDSPide_rn -EI[i].value,
					Dcef_taphm_pl[i]),->dpmUCLK ]
uinF_gfxcucTa  1;
	while (=
			h_single_lclkDcef_taphm_pl[i]),->dpmUCLK ]
uinint vega1  1;
	while (va->smcle (i < NUM_SOCCLKoc15 dividers;

	PP_ASSERT_WITH_Ca i < ef_tabger
*/el(struct pp_hwmgr *hwmgr,
		uint3rrent_vol_inde*table_info =
		DSPide_COUNT*current_voempty.",
			return -EINVAL);WITH_CODE(!pp_atdcef_tablype.; i++) i)lt);
	}

	if (datltageInde(dep_oOD_Dcef_taphm_pl[i])m_clock > mem_ma1)i < NUM_SOCCLKoc15 dividers;

	PP_ASSERT_WITH_C(!pp_ateger
*/el(st single SMC GFXCLK structure using terovided clock.
 *
 * @paeruct ppock to use to populatCODE(olucture.
 * @param    current_socclk_level - location in PPTable for the SMC SOCCLK structure.
 * @return   0 on success..
 *dep_soc_table =
			table_info->vdd_del_graphic_levels(structpt_v1_clock_voltage_deplock, uint8_t *current_soc_did,
		uint8_t *current_vol_index)
{
	struct phm_ppt			return -EINVAL);
	for (i = 0; i < dep_on_soc->count; i++) {
		if (dep_on_soc->entries[i].clk ==e == mem_clock)
				break;
		}
		PP_ASE *hwmgr,
		uint32_t lclock, uint8_t *curr_lclk_did)
{
	 @paeruct ppDependency Table",
			returnc_did = (uint8_t)dividers.ulDid;
	*current_vol_index = (uint8_t)(dep_oneid SOC_eUCLK Depeno populatCODE(olnt i;

	PP_ASSERT_WITH_CODE(dp_mclk NUM_SOCCLKoc15 dividers;

	PP_ASSERT_WITH_C(yrivcy_on_scluct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
static int vega10_populv(table_in),dyries[i].vddc;
	}

	p	/
stle_ailed to get SOC Clock sett vega10_hwmg0_lues */
	data->dpm_table.mem_tabla10(
			hwmgr,_vind tion *)(hwmgr->pptable);
	struct phm_ppt_v1_clock_voltage_dependency_table *dep_table =
			tableger
*/el(st count > i,
				"Cannot find UCLK in SOC_VDD-UCLK DependencyEp_table->SocDpmVoltageIndex[Vcei] =
				(uint8_t)convert_to_vid(
				vega10_locate_vddc_given_clock(hwmgr,
						dpm_VCE_W8
urn -EINVAL);
	}

	PP_ASSERT_WITH_CODE(!pp_ateger
*/el(st count > i,
				"Cannot find UCLK   0 on success..
 */
s *cuEp_table->SocDpmVoltageIndex[Vcei] =
				(uint8_t)convert_to_vid(
				vega10_locate_vddle (i < NUM_SOCCLK_DPM_LEVELSividers;

	PP_ASSERT_WITH_C(!pp_atvger
*/el(st single SMC GFXCLK structure using tvrovided clock.
 *
 * @pavt8_t)(data->vddc_voltage_table.psi0_enable);
	data->smc_state_table.pp_table.UlvPhaseSheddingPsi1 =
			(uint8_t)(data->vddc_volta_soc->count; i++) {
		if (dep_on_soc->entries[i].clk ==v == mem_clock)
				break;
		}
		PP_ASV *hwmgr,
		uint32_t lclock, uint8_t *curr_lcSOC_CLK in SOC_VDD-Svruct ppDependency Table",
			returnc__SOCCLKoc15 dividers;

	PP_ASSERT_WITH_C(!pp_atdger
*/el(st single SMC GFXCLK structure using tdrovided clock.
 *
 * @padt8_t)(data->vddc_voltage_table.psi0_enable);
	data->smc_state_table.pp_table.UlvPhaseSheddingPsi1 =
			(uint8_t)(data->vddc_volta_soc->count; i++) {
		if (dep_on_soc->entries[i].clk ==d == mem_clock)
				break;
		}
		PP_ASD *hwmgr,
		uint32_t lclock, uint8_t *curr_lcSOC_CLK in SOC_VDD-Sdruct ppDependency Table",
			returnc__SOCCLKoc15 dividers;

	PP_ASSERT_WITH_C(yriuvd_on_scluct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
static int vega10_populv(table_in),dyries[i].vddc;
	}

	p	/
stle_ailed to get SOC Clock sett vevruct a10_hwmg0_VOLTAGE_CONTROL_BY_SVr	dpm_table->dpe_ailed to get SOC Clock sett vegruct a10_hwmg0_VOLTAGE_CONTROL_BY_SVrddpm_table->dpe_ailedparam    current_socclk_level - location in PPTable for the SMC SOCCLK structure.
 * @return   0 on success..
 *dep_soc_table =
			table_info->vdd_del_graphic_levels(structpt_v1_clock_vola10(
			hwmgr,_vind tion *)(hwmgr->pptable);
	struct phmvruct a10_hwmglock_voltage_dependency_table *dep_table =
			tablvger
*/el(st count > i,
vruct a10_hwmglo find UCLK in SOC_VDD-UCLK DependencyV	return result;
		}
	}

	jyS>yS>yS>V> i,
				"Cannot find UCLK in SOC_VDD-UCLvVD_W8
urn -EINVAL);
	}

	PP_ASSERT_WITH_CODE(!pp_atvger
*/el(st count > i,
vruct a10_hwmglo find UCLK   0 on success..
 */
s *cuV	return result;
		}
	}

	jyS>yS>yS>V> i,
				"le (i < NUMle);
	struct phm_ruct a10_hwmglock_voltage_dependency_table *dep_table =
			tabldger
*/el(st count > i,
	ruct a10_hwmglo find UCLK in SOC_VDD-UCLK DependencyD	return result;
		}
	}

	jyS>yS>yS>V> i,
				"Cannot find UCLK in SOC_VDD-UCLvVD_W8
urn -EINVAL);
	}

	PP_ASSERT_WITH_CODE(!pp_atdger
*/el(st count > i,
	ruct a10_hwmglo find UCLK   0 on success..
 */
s *cuD	return result;
		}
	}

	jyS>yS>yS>V> i,
				"le (i < NUMle);
	struct phm_viders.ulDid;
	*current_vol_index = (uint8_t)(dep_onvid SOC> i,
vruct a10_hwmglo find UCLK in SOC_V &lock dex = (uint8_t)(dep_ondid SOC> i,
	ruct a10_hwmglo find UCLK in SOC_V)
			pnt8_t *curvdi] =
				(uint8_t)le,
		struct phm_ppt_v1_clock_DE(dp_mclkta->regisnt > i,
	 ? mem_ot find UCLK in SOC_VDD-UCLvVD_W8
urn -EINVAL);pnt8_t *curvdi] =
				(uint8_t)le i;

	PP_ASSERT_WITHjk_DE(dp_mclkte (i < NUM_SOCCLKoc15 dividers;

	PP_ASSERT_WITH_Cger
*/   etcher
	PP_Auct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
static int vega10_populv(table_in),dyries[i].vddc;
	}

	p	/
stle_ailedtatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		uint32_t mem_clock, uint8_t *current_mem_vid,
		PllSetting_t *current_memcl_graphic_levels(struct *hwmgr,
		uint32_rrent_vol_inde*table_info =
		_viders.ulDid;
	*current_vopnt8_t *cuCksE -EIN_t)le i;

	PP_ASSERT_WITHdency_on_sclk.entpnt8_t *cuCksVidtatic rride;
		else
	ndex = (uint8_t)(dep_on_es[i].vddI,
		s* le(hwmgrVID_OFFSET_SCALE2 / le(hwmgrVID_OFFSET_SCALE1)i < NUM_SOCCLKoc15 dividers;

	PP_ASSERT_WITH_Cavfs_ependency_uct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
static int vega10_populv(table_in),dyries[i].vddc;
	}

	p	/
stle_ailedtatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		uint32_t mem_clock, uint8_t *current_mem_vid,
		PllSetting_t *current_memcl_graphic_levels(struct *hwmgr,
		uint32_vddc_voltage_table.psavfs_ependency_ avfs_epend&{
	{0}d,
	tu: mem_c_fbsmult);
	curre
			\n", i);
	Min=
				(VppDependency Tf.
 *
 \n", i);
	Max=
				(VppDependency Tfive_next_settismu_feaS>yis[GNLD_AVFS].sata->gedNVAL);
	}

	PP_eddingPsi1 =
			(uavfs_gr *hwmgr,
.; i++) &avfs_epend&sult;
		}!
	}

	j = i - n", i);
	Min=
				(VppDependency TucTa - ependency Tabli	*curren)navfs_epend&		rMin=ppc)PcieGe\n", i);
	Max=
				(VppDependency TucTa - ependency Tabli	*curren)navfs_epend&		rMax=ppc)PciieGe\n", i);
	ACor (ant[0]CLK, mem_clock,avfs_epend&		rMeanNsigmaA_vddant0PcieGe\n", i);
	ACor (ant[1]CLK, mem_clock,avfs_epend&		rMeanNsigmaA_vddant1PcieGe\n", i);
	ACor (ant[2]CLK, mem_clock,avfs_epend&		rMeanNsigmaA_vddant2PcieGe\n", i);
	DCem_l_sigma{
	 1;
	while (avfs_epend&		sMeanNsigmaDcTolSigmaPcieGe\n", i);
	 table *_mean{
	 1;
	while (avfs_epend&		sMeanNsigma table *MeanPcieGe\n", i);
	 table *_sigma{
	 1;
	while (avfs_epend&		sMeanNsigmaDcTolSigmaPcieGe\n", i);
	 SM_AllSCompFacttab
	 1;
	while (avfs_epend&		sPsmAllComfacttaPciieGe\n", i);
	BtcGbVdroopint vCkstat.a0le,
		st, mem_clock,avfs_epend&		rGbVdroopint vCks].vA0PcieGe\n", i);
	BtcGbVdroopint vCkstat.a0_shif	PP_20cieGe\n", i);
	BtcGbVdroopint vCkstat.a1le,
		st, mem_clock,avfs_epend&		rGbVdroopint vCks].vA1PcieGe\n", i);
	BtcGbVdroopint vCkstat.a1_shif	PP_20cieGe\n", i);
	BtcGbVdroopint vCkstat.a2le,
		st, mem_clock,avfs_epend&		rGbVdroopint vCks].vA2PcieGe\n", i);
	BtcGbVdroopint vCkstat.a2_shif	PP_20ciieGe\n", i);
	th(
				BtcGbCkstn{
	avfs_epend&		cE -EINGbVdroopint vCks]ncieGe\n", i);
	BtcGbVdroopint vCkstn.a0le,
		st, mem_clock,avfs_epend&		rGbVdroopint vCks]nA0PcieGe\n", i);
	BtcGbVdroopint vCkstn.a0_shif	PP_20cieGe\n", i);
	BtcGbVdroopint vCkstn.a1le,
		st, mem_clock,avfs_epend&		rGbVdroopint vCks]nA1PcieGe\n", i);
	BtcGbVdroopint vCkstn.a1_shif	PP_20cieGe\n", i);
	BtcGbVdroopint vCkstn.a2le,
		st, mem_clock,avfs_epend&		rGbVdroopint vCks]nA2PcieGe\n", i);
	BtcGbVdroopint vCkstn.a2_shif	PP_20ciieGe\n", i);
	AvfsGbCkstn.m1le,
		st, mem_clock,avfs_epend&		rGbFuseint vCks]nM1PcieGe\n", i);
	AvfsGbCkstn.m2le,
		st, mem_clock,avfs_epend&		rGbFuseint vCks]nM2PcieGe\n", i);
	AvfsGbCkstn.ble,
		st, mem_clock,avfs_epend&		rGbFuseint vCks]nBPcieGe\n", i);
	AvfsGbCkstn.m1_shif	PP_24cieGe\n", i);
	AvfsGbCkstn.m2_shif	PP_12cieGe\n", i);
	AvfsGbCkstn.b_shif	PP_0ciieGe\n", i);
	th(
				AvfsGbCkstnle,
		stavfs_epend&		cE -EINGbFuseint vCks]ncieGe\n", i);
	AvfsGbCkstat.m1le,
		st, mem_clock,avfs_epend&		rGbFuseint vCks]ffM1PcieGe\n", i);
	AvfsGbCkstat.m2le,
		st, mem_clock,avfs_epend&		rGbFuseint vCks]ffM2PcieGe\n", i);
	AvfsGbCkstat.ble,
		st, mem_clock,avfs_epend&		rGbFuseint vCks]ffBPcieGe\n", i);
	AvfsGbCkstat.m1_shif	PP_24cieGe\n", i);
	AvfsGbCkstat.m2_shif	PP_12cieGe\n", i);
	AvfsGbCkstat.b_shif	PP_0ciieGe*table_info =
		_viders.ulDid;
	*curre,
		stageIndex[ivider=
				(tatic int_t)le,
		st- ependency Tabli	*cuse
	ndex = (uint8_t)(dep_onuintMTABLE_)PciieGe
		}(PPREGKEY_VEGA10QUADRATICEQUATION_DFLT !		}

		data->o>dpmUCk_quad_eqn_a) &lock 	(PPREGKEY_VEGA10QUADRATICEQUATION_DFLT !		}

		data->o>dpmUCk_quad_eqn_bolden_d_single_lclkDcef_taphm_p2result[DSPide_DISPide].m1le,
		st	();
	cur)data->o>dpmUCk_quad_eqn_a;n_d_single_lclkDcef_taphm_p2result[DSPide_DISPide].m2le,
		st	();
	cur)data->o>dpmUCk_quad_eqn_b;n_d_single_lclkDcef_taphm_p2result[DSPide_DISPide].ble,
		st	();
	cur)data->o>dpmUCk_quad_eqn_c;n_d_} >odn_dpm__single_lclkDcef_taphm_p2result[DSPide_DISPide].m1le,
		st	();
	cur)avfs_epend&		rDage_de2resultM1;n_d_single_lclkDcef_taphm_p2result[DSPide_DISPide].m2le,
		st	();
	cur)avfs_epend&		rDage_de2resultM2;n_d_single_lclkDcef_taphm_p2result[DSPide_DISPide].ble,
		st	();
	cur)avfs_epend&		rDage_de2resultB;n_d_}iieGe\n", i);
	Dcef_taphm_p2result[DSPide_DISPide].m1_shif	PP_24cieGe\n", i);
	Dcef_taphm_p2result[DSPide_DISPide].m2_shif	PP_12cieGe\n", i);
	Dcef_taphm_p2result[DSPide_DISPide].b_shif	PP_12ciieGe
		}(PPREGKEY_VEGA10QUADRATICEQUATION_DFLT !		}

		data->o			(UCk_quad_eqn_a) &lock 	(PPREGKEY_VEGA10QUADRATICEQUATION_DFLT !		}

		data->o			(UCk_quad_eqn_bolden_d_single_lclkDcef_taphm_p2result[DSPide_DCEFide].m1le,
		st	();
	cur)data->o			(UCk_quad_eqn_a;n_d_single_lclkDcef_taphm_p2result[DSPide_DCEFide].m2le,
		st	();
	cur)data->o			(UCk_quad_eqn_b;n_d_single_lclkDcef_taphm_p2result[DSPide_DCEFide].ble,
		st	();
	cur)data->o			(UCk_quad_eqn_c;n_d_} >odn_dpm__single_lclkDcef_taphm_p2result[DSPide_DCEFide].m1le,
		st	();
	cur)avfs_epend&		rDk_volt2resultM1;n_d_single_lclkDcef_taphm_p2result[DSPide_DCEFide].m2le,
		st	();
	cur)avfs_epend&		rDk_volt2resultM2;n_d_single_lclkDcef_taphm_p2result[DSPide_DCEFide].ble,
		st	();
	cur)avfs_epend&		rDk_volt2resultB;n_d_}iieGe\n", i);
	Dcef_taphm_p2result[DSPide_DCEFide].m1_shif	PP_24cieGe\n", i);
	Dcef_taphm_p2result[DSPide_DCEFide].m2_shif	PP_12cieGe\n", i);
	Dcef_taphm_p2result[DSPide_DCEFide].b_shif	PP_12ciieGe
		}(PPREGKEY_VEGA10QUADRATICEQUATION_DFLT !		}

		data->dpm_stUCk_quad_eqn_a) &lock 	(PPREGKEY_VEGA10QUADRATICEQUATION_DFLT !		}

		data->dpm_stUCk_quad_eqn_bolden_d_single_lclkDcef_taphm_p2result[DSPide_PIXide].m1le,
		st	();
	cur)data->dpm_stUCk_quad_eqn_a;n_d_single_lclkDcef_taphm_p2result[DSPide_PIXide].m2le,
		st	();
	cur)data->dpm_stUCk_quad_eqn_b;n_d_single_lclkDcef_taphm_p2result[DSPide_PIXide].ble,
		st	();
	cur)data->dpm_stUCk_quad_eqn_c;n_d_} >odn_dpm__single_lclkDcef_taphm_p2result[DSPide_PIXide].m1le,
		st	();
	cur)avfs_epend&		rPpm_solt2resultM1;n_d_single_lclkDcef_taphm_p2result[DSPide_PIXide].m2le,
		st	();
	cur)avfs_epend&		rPpm_solt2resultM2;n_d_single_lclkDcef_taphm_p2result[DSPide_PIXide].ble,
		st	();
	cur)avfs_epend&		rPpm_solt2resultB;n_d_}iieGe\n", i);
	Dcef_taphm_p2result[DSPide_PIXide].m1_shif	PP_24cieGe\n", i);
	Dcef_taphm_p2result[DSPide_PIXide].m2_shif	PP_12cieGe\n", i);
	Dcef_taphm_p2result[DSPide_PIXide].b_shif	PP_12cieGe
		}(PPREGKEY_VEGA10QUADRATICEQUATION_DFLT !		}

		data->dhytUCk_quad_eqn_a) &lock 	(PPREGKEY_VEGA10QUADRATICEQUATION_DFLT !		}

		data->dhytUCk_quad_eqn_bolden_d_single_lclkDcef_taphm_p2result[DSPide_PHYide].m1le,
		st	();
	cur)data->dhytUCk_quad_eqn_a;n_d_single_lclkDcef_taphm_p2result[DSPide_PHYide].m2le,
		st	();
	cur)data->dhytUCk_quad_eqn_b;n_d_single_lclkDcef_taphm_p2result[DSPide_PHYide].ble,
		st	();
	cur)data->dhytUCk_quad_eqn_c;n_d_} >odn_dpm__single_lclkDcef_taphm_p2result[DSPide_PHYide].m1le,
		st	();
	cur)avfs_epend&		rPtage_2resultM1;n_d_single_lclkDcef_taphm_p2result[DSPide_PHYide].m2le,
		st	();
	cur)avfs_epend&		rPtage_2resultM2;n_d_single_lclkDcef_taphm_p2result[DSPide_PHYide].ble,
		st	();
	cur)avfs_epend&		rPtage_2resultB;n_d_}iieGe\n", i);
	Dcef_taphm_p2result[DSPide_PHYide].m1_shif	PP_24cieGe\n", i);
	Dcef_taphm_p2result[DSPide_PHYide].m2_shif	PP_12cieGe\n", i);
	Dcef_taphm_p2result[DSPide_PHYide].b_shif	PP_12ciieGe\n", i);
	AcgBtcGbVdroopint v.a0lllllll
	avfs_epend&		lAcgGbVdroopint vA0cieGe\n", i);
	AcgBtcGbVdroopint v.a0_shif	PP_20cieGe\n", i);
	AcgBtcGbVdroopint v.a1lllllll
	avfs_epend&		lAcgGbVdroopint vA1cieGe\n", i);
	AcgBtcGbVdroopint v.a1_shif	PP_20cieGe\n", i);
	AcgBtcGbVdroopint v.a2lllllll
	avfs_epend&		lAcgGbVdroopint vA2cieGe\n", i);
	AcgBtcGbVdroopint v.a2_shif	PP_20ciieGe\n", i);
	AcgAvfsGb.m1lllllllllllllllllll
	avfs_epend&		lAcgGbFuseint vM1cieGe\n", i);
	AcgAvfsGb.m2lllllllllllllllllll
	avfs_epend&		lAcgGbFuseint vM2cieGe\n", i);
	AcgAvfsGb.bllllllllllllllllllll
	avfs_epend&		lAcgGbFuseint vBcieGe\n", i);
	AcgAvfsGb.m1_shif	Pllllllllllll
	0cieGe\n", i);
	AcgAvfsGb.m2_shif	Pllllllllllll
	0cieGe\n", i);
	AcgAvfsGb.b_shif	PllllllllllllPP_0ciieG} >odn_dpm___settismu_feaS>yis[GNLD_AVFS].sata->gedPP_i == 0 ||clk.entries[i].clk =ividers;

	PP_ASSErrenn_sclkuct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
statit);
	curragc_btcor  pon= 0 e_next_settismu_feaS>yis[GNLD_ACG].sata->gedNVAL);next0SOC_,
	turnn_sclkC(yrifeaS>yissuccess.smui++) m_ta,	}

		data->smu_feaS>yis[GNLD_W8
uPREFETCHER].smu_feaS>yi_bitmap))pm___settismu_feaS>yis[GNLD_W8
uPREFETCHER].< data->dpm_tablpm_smui_s/
s_msgncy (yrsuccess.smui++) PPSMC_MSG_le(hwmgr,
Acg)blpm_smui_s/
s_msgncy (yrsuccess.smui++) PPSMC_MSG_RunAcgBtcble_ie
	turnW8yS>arent_om (yrsuccess.smui++) &agc_btcor  pon= )blpm_next1SOC_agc_btcor  pon= )T_WITH_COD1SOC_r->pptarenloopingle_d	}

	smui_s/
s_msgncy (yrsuccess.smui++) PPSMC_MSG_RunAcgIk(stsedLoopPcieGe>odn__COD2SOC_r->pptarenloopingle_d	}

	smui_s/
s_msgncy (yrsuccess.smui++) PPSMC_MSG_RunAcgIkOpenLoopPcieGenext0SOC_,
	turnn_sclkC(yrifeaS>yissuccess.smui++) m_ta,	}

	_settismu_feaS>yis[GNLD_ACG].smu_feaS>yi_bitmap))pm__
	_settismu_feaS>yis[GNLD_ACG].< data->dpm_table.m >odn_dpm__0_locate_vACG_E -EIN] ACG BTC Ries[ie {
		pp_ ividustablPcieGe_settismu_feaS>yis[GNLD_ACG].< data->dpi == 0 ||clk.entries[i].clk =ividers;

	PP_ASSErrendissclkuct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
state_next_settismu_feaS>yis[GNLD_ACG].sata->gedNVAL);next_settismu_feaS>yis[GNLD_ACG].< data-NVAL);next0SOC_,
	turnn_sclkC(yrifeaS>yissuccess.smui++) i == ,	}

	_settismu_feaS>yis[GNLD_ACG].smu_feaS>yi_bitmap))pm___settismu_feaS>yis[GNLD_ACG].< data->dpi == 0 ||clk.entries[i].clk =ividers;

	PP_ASSERT_WITH_Cgpio_ependency_uct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
static int vega10_populv(table_in),dyries[i].vddc;
	}

	p	/
stle_ailedtddingPsi1 =
		pio_ependency_ 	pio_epend&{
	{0}d,
	tu: mem_crnc__S}

	PP_eddingPsi1 =
			(u	pio_gr *hwmgr,
.; i++) &	pio_epend&tabla		}!
	}

	j = i wmgr,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(&t DPM table */
	meRegWITHorHot) &lock 	(e_in),yS>v Mry_e_in.regWITHor_ho(u	pio_sata->golden_de\n", i);
	VR0HotGpioPP_	pio_epend&.ucVR0HotGpio;n_de\n", i);
	VR0HotPolarityPP_	pio_epend&.ucVR0HotPolarity;n_de\n", i);
	VR1HotGpioPP_	pio_epend&.ucVR1HotGpio;n_de\n", i);
	VR1HotPolarityPP_	pio_epend&.ucVR1HotPolarity;n_dm >odn_dpm__0n", i);
	VR0HotGpioPP_0cieGe\n", i);
	VR0HotPolarityPP_0cieGe\n", i);
	VR1HotGpioPP_0cieGe\n", i);
	VR1HotPolarityPP_0cieG}
 i wmgr,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(&t DPM table */
	meAungPiderDCTransigr,
) &lock 	(e_in),yS>v Mry_e_in.ac_driewiforu	pio_sata->golden_de\n", i);
	AcDcGpioPP_	pio_epend&.ucAcDcGpiocieGe\n", i);
	AcDcPolarityPP_	pio_epend&.ucAcDcPolarity;n_dm >odn_dpm__0n", i);
	AcDcGpioPP_0cieGe\n", i);
	AcDcPolarityPP_00 ||clk.entries[i]_DPM_LEVELSividers;

	PP_ASSEavfs_n_sclkuct pp_hwmgr *hwmgr,
		, bool lvGfxclint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
state_next_settismu_feaS>yis[GNLD_AVFS].sata->gedNVAL);nextlvGfxcl_dpm__empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
	m_ta,	}

		data->smu_feaS>yis[GNLD_AVFS].smu_feaS>yi_bitmap),	}

		"[avfs__vddrol] Attempt
		PE -EIN AVFS feaS>yi{
		pp_t_clock  > mem_ma1)i <		data->smu_feaS>yis[GNLD_AVFS].< data->dpm_table.m >odn_dpm__empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
	i == ,	}

		data->smu_feaS>yis[GNLD_AVFS].smu_feaS>yi_id),	}

		"[avfs__vddrol] Attempt
		PDissclk AVFS feaS>yi{
		pp_t_clock  > mem_ma1)i <		data->smu_feaS>yis[GNLD_AVFS].< data->dpi == 0 ||clk.entries[i].clk =ividers;

	PP_ASSERT_WITH_Can_masloyS>avfs_fuse;
	st				trl_get_gpu_pll_dividers_vega10(
			hwmgr0ciiet);
64r.
 erwmg__table_fbsmult);
	currtop32, botngP32;, uint8_t *cufuses_tate(&( fuse *)(s_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
statiAvfsFuseth(
				the Gvfs_fuse;opulv(table_in),dyries[i].vddc;
avfs_fuse;
	st				

	p	/
st
	smui_s/
s_msgncy (yrsuccess.smui++) PPSMC_MSG_R8ySSerwmgNumTop32tatie
	turnW8yS>arent_om (yrsuccess.smui++) &top32tat
	smui_s/
s_msgncy (yrsuccess.smui++) PPSMC_MSG_R8ySSerwmgNumBotngP32tatie
	turnW8yS>arent_om (yrsuccess.smui++) &botngP32tat
	serwmg__table_fbli	*cu64r.)botngP32 << 32t |rtop32ate_nextgpu
	st				
		(utate(&(_fuse;SOC_V(serwmg__table,d,
	turnfuses_tate(&() &fuse)SOC_0l_dpm_Gvfs_fuse;opulv
	VFT0_blPP_fuse.VFT0_b;pm_Gvfs_fuse;opulv
	VFT0_m1le_fuse.VFT0_m1;pm_Gvfs_fuse;opulv
	VFT0_m2le_fuse.VFT0_m2;pm_Gvfs_fuse;opulv
	VFT1_blPP_fuse.VFT1_b;pm_Gvfs_fuse;opulv
	VFT1_m1le_fuse.VFT1_m1;pm_Gvfs_fuse;opulv
	VFT1_m2le_fuse.VFT1_m2;pm_Gvfs_fuse;opulv
	VFT2_blPP_fuse.VFT2_b;pm_Gvfs_fuse;opulv
	VFT2_m1le_fuse.VFT2_m1;pm_Gvfs_fuse;opulv
	VFT2_m2le_fuse.VFT2_m2;pm_
	}

	PP_ASSERT_cop*currenncy (yrsuccess.smui++)rent_ clock.
 )Gvfs_fuse;opulv, AVFSFUSEPDATE)i <	empty.",
			return -E
	}

			break;
		}
		PasloyS FusetVst				t_clock)i < NUM_SOCCLK_DPM_LEVELSividers;

	PP_ASSEsaveutate(&(_power_profilkuct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
statimgr->backend);
	struct phm_ppt_v2_information *table_info =
			(struct phm_*)(hwmgr->innd UCLate_ Table!",te(&(_	(stpower_profilk.lypetioAMD_emp)
		PROFILE;e_ Table!",te(&(_compui].power_profilk.lypetioAMD_empt; i++) PROFILE;ePUCLKOptimize compui] power profilk: Udn_only highest
	 * 2 power d UCLK (nexmordex[an 2 hhhhav;
	Gfxcli	led tnext_a10_hwmglock_vo > 2_mclkinnd UCLle ia10_hwmglock_vo - 2;pm>odn__CODia10_hwmglock_vo OC_2_mclkinnd UCLle 1;pm>odnmclkinnd UCLle 0ate_ Table!",te(&(_compui].power_profilk.kinn_level(str a10_hwmglo find UCLK kinnd UCLn SOC_Vate_ Table!	(stpower_profilkle  Table!",te(&(_	(stpower_profilk;e_ Table!compui].power_profilkle  Table!",te(&(_compui].power_profilk
			(uint8_t)(dep_on_* le(hwmgr,
s**
 * @brformatan_PasloySs it		if (dep_table->entrihhhhhhhhhhhhhhhhhhhpowerf_tahhhhhhhhhhhhhhhhh,WhhhhhhhhhhhhhInpui hhhhhpo)(her 		Pinpui t *cu(PowerSgle_d	ture.
 * @realways 0entries[i].vddc;
	}

	ie(hC(yritsclkuct pp_hwmgr *hwmgr,
		uint3	tu: mem_crn memory clock to use to populate the structure.
 * @return   0 on success..
 */
static int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		uint32_t mem_clock, c int vega10_populv(table_in),dyries[i].vddc;
	}

	p	/
stle_ailedtddingPsi1 =
	te_table.pp_t te_table.pp_tstle_ailedtddingPsi1 =
	bios_bootmas;SOC_Vs bootmas;SOC_Vsrnc__S}

	PP_ckend);
.
 putate(&(_ a10_hwmgssuccesCLK Dependency Table!",
	
	}

			break;
		}
		P
.
 p tate(&( DPMrformasint8_t *curr_l
	}

	j
			\n"ingPsi1 =
			(ute_table.pp_t_v4.; i++) le(hwmgrTYP*dep_ot8_t le(hwmgrOBJ_SVID2,  &te_table.pp_t);)
			return MaxVidStepPP_ce_table.pp_t.ncy_vidiesep
			\n", i);
	Gfxi] =
				(Modemp1clk_did);
	dae_single_m->uc_	(st a10ce_tablemode);)
			return i]));
		if (rModemp1clk_did);
	dae_single_m->uc_OC_CLa10ce_tablemode);)
			return rren);
		if (rModemp1clk_did);
	dae_single_m->uc_uruct a10ce_tablemode);)
			return rvdi] =
				(Modemp1clk_did);
	dae_single_m->uc_uvdt a10ce_tablemode);)
			return Vcei] =
				(Modemp1clk_did);
	dae_single_m->uc_vcy_ a10ce_tablemode);)
			return Mp0i] =
				(Modemp1clk_did);
	dae_single_m->uc_mp0_ a10ce_tablemode);))
			return Dcef_tai] =
				(Modemp1clk_did);
	dae_single_m->uc_o			( a10ce_tablemode);))
e_in),_ppc_ce_table.pp_t.psi);n_sclkPP_ce_table.pp_t.psi);n_sclk;)
e_in),_ppc_ce_table.pp_t.psi1;n_sclkPP_ce_table.pp_t.psi1;n_sclk;)e_next_settiyS>v Mry_e_in.ulv_sata->g &lock e_single_m->usoffs_ce_tableTABLE_)VAL);
	}

	PP_ASSERT_WITH_CODEulv_ss[i]succesCLK  Dependency Table!",
	
	}

			breeak;
		}
		Pie(hwmgr,
 ULVid SG8m_clock > mem_m
	}

	j
	< NUM_S}

	PP_ASSERT_WITH_CODE(yrilin*/el(struuccesCLK Dependency Table!",
	
	}

			break;
		}
		Pie(hwmgr,
 Lin*_LEVELint8_t *curr_l
	}

	j
			_S}

	PP_ASSERT_WITH_CODE[LocateVddcGivenClouccesCLK Dependency Table!",
	
	}

			break;
		}
		Pie(hwmgr,
 GteVddcs_LEVELint8_t *curr_l
	}

	j
			_S}

	PP_ASSERT_WITH_CODE[Locsoc_level(struuccesCLK Dependency Table!",
	
	}

			break;
		}
		Pie(hwmgr,
 Mruct _LEVELint8_t *curr_l
	}

	j
			_S}

	PP_ASSERT_WITH_CODE[Loc< ef_tabger
*/el(struuccesCLK Dependency Table!",
	
	}

			break;
		}
		Pie(hwmgr,
 Dcef_ta_LEVELint8_t *curr_l
	}

	j
			_S}

	PP_ASSERT_WITH_CODE(yrivcy_on_scluuccesCLK Dependency Table!",
	
	}

			break;
		}
		Pie(hwmgr,
 VCE_LEVELint8_t *curr_l
	}

	j
			_S}

	PP_ASSERT_WITH_CODE(yriuvd_on_scluuccesCLK Dependency Table!",
	
	}

			break;
		}
		Pie(hwmgr,
 UVD_LEVELint8_t *curr_l
	}

	j
			next_settiyS>v Mry_e_in.ger
*/   etcher
sata->goVAL);
	}

	PP_ASSERT_WITH_CODEger
*/   etcher
	PP_AuuccesCLK  Dependency Table!",
	
	}

			breeak;
		}
		PltageInde(dep_oS  etcherk = mem_clock > mem_m
	}

	j
	< NUM_S}

	PP_\n"ingPsi1 =
			(utbios_bootas;SOC_Vs.; i++) &bootmas;SOC_Vstabla		}!
	}

	j = i e_in),_bios_bootmd SG8_DE(dllllPP_bootmas;SOC_Vs.usV16_t v e_in),_bios_bootmd SG8_DE(dilllPP_bootmas;SOC_Vs.usV16_it v e_in),_bios_bootmd SG8_mDE(dllllP_bootmas;SOC_Vs.usMv16_t v e_in),_bios_bootmd SG8_truct phm_ppbootmas;SOC_Vs.ulGfxCe_deade_in),_bios_bootmd SG8_m = (struct bootmas;SOC_Vs.ulUCe_deade_in),_bios_bootmd SG8_level->Didt bootmas;SOC_Vs.uli])Ce_deade_in),_bios_bootmd SG8_o			(UC>Didt bootmas;SOC_Vs.ulDCEFie_deadnext0S!P_bootmas;SOC_Vs.usV16_l_dpm__smui_s/
s_msgncy (yr_with_ependencysuccess.smui++)pm__
		PPSMC_MSG_SetFloor_v2_inform)pm__
		(bootmas;SOC_Vs.usV16_ * 4))i <		data->_bios_bootmd SG8_bCODE(ppc_loDidt m_table.m >odn_dpm__data->_bios_bootmd SG8_bCODE(ppc_loDidt i == 0 ||clk_smui_s/
s_msgncy (yr_with_ependencysuccess.smui++)pm__
PPSMC_MSG_SetMinDeepSleepDk_volt)rent_ clo	cur)(e_in),_bios_bootmd SG8_o			(UC>DidsOn =bj
	< NUM_S}

	PP_ASSERT_WITH_CODEavfs_ependency_uuccesCLK Dependency Table!",
	
	}

			break;
		}
		Pie(hwmgr,
 AVFS Ppendency_int8_t *curr_l
	}

	j
			_S}

	PP_ASSERT_WITH_CODEgpio_ependency_uuccesCLK Dependency Table!",
	
	}

			break;
		}
		Pie(hwmgr,
 GPIO Ppendency_int8_t *curr_l
	}

	j
			\n", i);
	GfxoltAverormAlphaDependency TucTa(e_in),DPM_LEVaverormE[Lpha);)
			return i])oltAverormAlphaDependency TucTa(e_in), to MhzaverormE[Lpha);)
			return UoltAverormAlphaDependency TucTa(e_in),u MhzaverormE[Lpha);)
			return GfxActivityAverormAlphaDependency TucTa(e_in),DPM_activityzaverormE[Lpha);)tie
	turnRT_WITH_Can_masloyS>avfs_fuse;
	st				tuccesCLK		_S}

	PP_ASSERT_cop*currenncy (yrsuccess.smui++)rent_ clock.
 )			retur) PPPDATE)i <empty.",
			return -E
	}

			break;
		}
		PasloyS PPt= mem_c *curr_l
	}

	j
			_S}

	PP_ASSERT_avfs_n_sclku; i++) m_ta)i <empty.",
			return -E
	}

		 "Attempt
		Pe -EIN AVFS feaS>yi{
		pp_t_clock  > mem_m
	}

	j
	<PP_ASSErrenn_sclkuuccesCLK PP_ASSEsaveutate(&(_power_profilkuuccesCLK		_Ses[i].clk =ividers;

	PP_ASSEn_sclkCthermal_protecgr,
.ct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
state_next_settismu_feaS>yis[GNLD_THERMAL].sata->gedNVAL);next_settismu_feaS>yis[GNLD_THERMAL].< data-N
m__0_locate_THERMAL FeaS>yi{AlW8ySyPe -EINdtructK  Dependency Table!",
lock EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
m_ta,	}

	_settismu_feaS>yis[GNLD_THERMAL].smu_feaS>yi_bitmap),	}

	"E -EIN THERMAL FeaS>yi{
		pp_t_clock > mem_ma1)i <	_settismu_feaS>yis[GNLD_THERMAL].< data-dt m_table.entries[i].clk =ividers;

	PP_ASSEdissclkCthermal_protecgr,
.ct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
state_next_settismu_feaS>yis[GNLD_THERMAL].sata->gedNVAL);next!_settismu_feaS>yis[GNLD_THERMAL].< data-N
m__0_locate_THERMAL FeaS>yi{AlW8ySyPdissclkdtructK  Dependency Table!",
lock EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
i == ,	}

	_settismu_feaS>yis[GNLD_THERMAL].smu_feaS>yi_bitmap),	}

	"dissclk THERMAL FeaS>yi{
		pp_t_clock > mem_ma1)i <	_settismu_feaS>yis[GNLD_THERMAL].< data-dt i == 0 |.entries[i].clk =ividers;

	PP_ASSEn_sclkCvrho(ufeaS>yiuct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
state_next,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meRegWITHorHot)NVAL);next_settismu_feaS>yis[GNLD_VR0HOT].sata->gedNVAL); Dependency Table!",
lock  EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
	m_ta,	}

		data->smu_feaS>yis[GNLD_VR0HOT].smu_feaS>yi_bitmap),	}

		"Attempt
		PE -EIN VR0 He & eaS>yi{
		pp_t_clock  > mem_ma1)i <		data->smu_feaS>yis[GNLD_VR0HOT].< data->dpm_table.m >odn_dpm__next_settismu_feaS>yis[GNLD_VR1HOT].sata->gedNVAL);  Dependency Table!",
lock   EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
		m_ta,	}

			data->smu_feaS>yis[GNLD_VR1HOT].smu_feaS>yi_bitmap),	}

			"Attempt
		PE -EIN VR0 He & eaS>yi{
		pp_t_clock   > mem_ma1)i <			data->smu_feaS>yis[GNLD_VR1HOT].< data->dpm_table.|clk_clk.etries[i].clk =ividers;

	PP_ASSEn_sclkCulvuct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
state_next_settiyS>v Mry_e_in.ulv_sata->gent_voempty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
m_ta, data->smu_feaS>yis[GNLD_ULV].smu_feaS>yi_bitmap),	}

	"E -EIN ULViFeaS>yi{
		pp_t_clock > mem_ma1)i <	_settismu_feaS>yis[GNLD_ULV].< data-dt m_table.entries[i].clk =ividers;

	PP_ASSEdissclkCulvuct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
state_next_settiyS>v Mry_e_in.ulv_sata->gent_voempty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
i == , data->smu_feaS>yis[GNLD_ULV].smu_feaS>yi_bitmap),	}

	"dissclk ULViFeaS>yi{
		pp_t_clock > mem_max_clock ? 	_settismu_feaS>yis[GNLD_ULV].< data-dt i == 0 |.entries[i].clk =ividers;

	PP_ASSEn_sclkCdeep_sleepck_v	DPMewiforuct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
state_next_settismu_feaS>yis[GNLD_DS_e->Soc].sata->gedNVAL);empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
m_ta, data->smu_feaS>yis[GNLD_DS_e->Soc].smu_feaS>yi_bitmap),	}

	"Attempt
		PE -EIN DS_e->SociFeaS>yi{
		pp_t_clock > mem_max_clock ? 	_settismu_feaS>yis[GNLD_DS_e->Soc].< data-dt m_table.entnext_settismu_feaS>yis[GNLD_DS_SOCSoc].sata->gedNVAL);empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
m_ta, data->smu_feaS>yis[GNLD_DS_SOCSoc].smu_feaS>yi_bitmap),	}

	"Attempt
		PE -EIN DS_SOCSociFeaS>yi{
		pp_t_clock > mem_max_clock ? 	_settismu_feaS>yis[GNLD_DS_SOCSoc].< data-dt m_table.entnext_settismu_feaS>yis[GNLD_DS_LSoc].sata->gedNVAL);empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
m_ta, data->smu_feaS>yis[GNLD_DS_LSoc].smu_feaS>yi_bitmap),	}

	"Attempt
		PE -EIN DS_LSociFeaS>yi{
		pp_t_clock > mem_max_clock ? 	_settismu_feaS>yis[GNLD_DS_LSoc].< data-dt m_table.entnext_settismu_feaS>yis[GNLD_DS_DCEFide].sata->gedNVAL);empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
m_ta, data->smu_feaS>yis[GNLD_DS_DCEFide].smu_feaS>yi_bitmap),	}

	"Attempt
		PE -EIN DS_DCEFideiFeaS>yi{
		pp_t_clock > mem_max_clock ? 	_settismu_feaS>yis[GNLD_DS_DCEFide].< data-dt m_table.entries[i].clk =ividers;

	PP_ASSEdissclkCdeep_sleepck_v	DPMewiforuct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
state_next_settismu_feaS>yis[GNLD_DS_e->Soc].sata->gedNVAL);empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
i == , data->smu_feaS>yis[GNLD_DS_e->Soc].smu_feaS>yi_bitmap),	}

	"Attempt
		Pdissclk DS_e->SociFeaS>yi{
		pp_t_clock > mem_max_clock ? 	_settismu_feaS>yis[GNLD_DS_e->Soc].< data-dt i == 0 |.entnext_settismu_feaS>yis[GNLD_DS_SOCSoc].sata->gedNVAL);empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
i == , data->smu_feaS>yis[GNLD_DS_SOCSoc].smu_feaS>yi_bitmap),	}

	"Attempt
		Pdissclk DS_iFeaS>yi{
		pp_t_clock > mem_max_clock ? 	_settismu_feaS>yis[GNLD_DS_SOCSoc].< data-dt i == 0 |.entnext_settismu_feaS>yis[GNLD_DS_LSoc].sata->gedNVAL);empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
i == , data->smu_feaS>yis[GNLD_DS_LSoc].smu_feaS>yi_bitmap),	}

	"Attempt
		Pdissclk DS_LSociFeaS>yi{
		pp_t_clock > mem_max_clock ? 	_settismu_feaS>yis[GNLD_DS_LSoc].< data-dt i == 0 |.entnext_settismu_feaS>yis[GNLD_DS_DCEFide].sata->gedNVAL);empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
i == , data->smu_feaS>yis[GNLD_DS_DCEFide].smu_feaS>yi_bitmap),	}

	"Attempt
		Pdissclk DS_DCEFideiFeaS>yi{
		pp_t_clock > mem_max_clock ? 	_settismu_feaS>yis[GNLD_DS_DCEFide].< data-dt i == 0 |.entries[i].clk =ividers;

	PP_ASSEstop( a1uct pp_hwmgr *hwmgr,
		, *)(hwmgr-bitmap)int32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
statit);
	curri,& eaS>yick_vkle 0atentnet_settismu_feaS>yis[GNLD_LED_DISPLAY].sata->gedPPt m_ta)AL);empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
i == , data->smu_feaS>yis[GNLD_LED_DISPLAY].smu_feaS>yi_bitmap),	}
"Attempt
		Pdissclk LED DPMr eaS>yi{f		pp_t_c > mem_max_clock ? 	_settismu_feaS>yis[GNLD_LED_DISPLAY].< data-dt i == 0 |.ent*table_info =
		GNLD_W8
uMAX*current_vol_indsettismu_feaS>yis[i].smu_feaS>yi_bitmap &-bitmap)_dpm__next_settismu_feaS>yis[i].sata->gedNVAL);  next_settismu_feaS>yis[i].< data-NVAL);__
ieaS>yick_vkl|= data->smu_feaS>yis[i].lock   	smu_feaS>yi_bitmap;	}

		data->smu_feaS>yis[i].< data->dpi == 0 ||.|clk_|clk_clk.eK PP_ASSEn_sclkC(yrifeaS>yissuccess.smui++) i == ,& eaS>yick_vkCLK		_Ses[i].clk =_on_ * @brief Tell* @brf	Pe -EINdhhhhhsata->gedPW8
s._ *_ * @dep_table->entr-ihhhhhhhhhhhhhhhhhhhpowerf_tahhhhhhhhhhhhhhhhh,W * @Pep_tablebitmap -ebitmap *tabhhhhfeaS>yisrf	Pe -EINd,W * @e.
 * @re0 on at le_v	 one DPMrishsacchhhfullyPe -EINd,W *tries[i].vddc;
	}

	iesrt( a1uct pp_hwmgr *hwmgr,
		, *)(hwmgr-bitmap)int32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
statit);
	curri,& eaS>yick_vkle 0atet*table_info =
		GNLD_W8
uMAX*current_vol_indsettismu_feaS>yis[i].smu_feaS>yi_bitmap &-bitmap)_dpm__next_settismu_feaS>yis[i].sata->gedNVAL);  next!_settismu_feaS>yis[i].< data-NVAL);__
ieaS>yick_vkl|= data->smu_feaS>yis[i].lock   	smu_feaS>yi_bitmap;	}

		data->smu_feaS>yis[i].< data->dpm_table.||clk_|clk_clk.eK nextINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__m_ta,  eaS>yick_vkCNVAL);*table_info =
		GNLD_W8
uMAX*current_vool_indsettismu_feaS>yis[i].smu_feaS>yi_bitmap &L);__
ieaS>yick_vk) <			data->smu_feaS>yis[i].< data->dpi == 0 ||clk.eK net_settismu_feaS>yis[GNLD_LED_DISPLAY].sata->gedPPt m_ta)AL);empty.",
			return -EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
m_ta, data->smu_feaS>yis[GNLD_LED_DISPLAY].smu_feaS>yi_bitmap),	}
"Attempt
		PE -EIN LED DPMr eaS>yi{F		pp_t_c > mem_max_clock ? 	_settismu_feaS>yis[GNLD_LED_DISPLAY].< data-dt m_table.entnext_setti_bios_bootmd SG8_bCODE(ppc_loDient_vosmui_s/
s_msgncy (yr_with_ependencysuccess.smui++)pm__
		PPSMC_MSG_SetFloor_v2_inform) 0k ? 	_setti_bios_bootmd SG8_bCODE(ppc_loDidt i == 0 |.entnext,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meFalcon_QuickTransigr,
)NVAL);next_settismu_feaS>yis[GNLD_ACDC].sata->gedNVAL); Dependency Table!",
EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
	m_ta, data->smu_feaS>yis[GNLD_ACDC].smu_feaS>yi_bitmap),	}

		"Attempt
		PE -EIN DS_e->SociFeaS>yi{
		pp_t_clock  > mem_ma1)i <		data->smu_feaS>yis[GNLD_ACDC].< data->dpm_table.m |.entries[i].clk =ividers;

	PP_ASSEn_sclkCda10_hsk_uct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
stati;

	tmp_
	}

		 _S}

	PP_0atettmp_
	}

	PP_smui_s/
s_msgncy (yr_with_ependencysuccess.smui++)pm__PPSMC_MSG_Config>yiTeledenry, data->config_teledenry)i <empty.",
			return -Etmp_
	}

		pm__ak;
		}
		Pconfig>yi teledenryint8_t *curr_ltmp_
	}

	tat
	smui_s/
s_msgncy (yr_with_ependencysuccess.smui++)pm__PPSMC_MSG_NumOfDcef_tas) 0k ?ettmp_
	}

	PP_
EINVAL);isCda10runninguuccesC) ?e0 :,
	 ? empty.",
			return -Etmp_
	}

		pm__aDPMrishalW8ySyPrunning right , skipping re-n_sclkmentint8_t *curr_l0k ?ettmp_
	}

	PP_ASSERT_consoc_di_ce_table.pp_t_uuccesCLK Dependency Table!",
	tmp_
	}

		pm__ak;
		}
		Pconoc_did,e_tablrformasint8_t *c}

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_ie(hC(yritsclkuuccesCLK Dependency Table!",
	tmp_
	}

		pm__ak;
		}
		Pie(hwmgr,
  @brformaint8_t *c}

	PP_tmp_
	}

	tat
	next,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meThermalCvddroller)NVAL);tmp_
	}

	PP_ASSERT_n_sclkCthermal_protecgr,
.uccesCLK  Dependency Table!",
	tmp_
	}

		pm___ak;
		}
		Pe -EIN thermal protecgr,
t_clock > }

	PP_tmp_
	}

	tat|.enttmp_
	}

	PP_ASSERT_n_sclkCvrho(ufeaS>yiuuccesCLK Dependency Table!",
	tmp_
	}

		pm__ak;
		}
		Pe -EIN VR he & eaS>yiint8_t *c}

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_n_sclkCdeep_sleepck_v	DPMewiforuuccesCLK Dependency Table!",
	tmp_
	}

		pm__ak;
		}
		Pe -EIN deep sleep k_v	DP ewiforint8_t *c}

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_iesrt( a1ur,
		, SMC_W8
uFEATURESCLK Dependency Table!",
	tmp_
	}

		pm__ak;
		}
		Piesrt DPMt_c > }

	PP_tmp_
	}

	tat
	/*Pe -EIN did		 do ne &ab->g nexf		pp_ did	led ttmp_
	}

	PP_ASSERT_n_sclkCdid	_configuuccesCLK Dependenc
	tmp_
	}

		pm__ak;
		}
		Pe -EIN did	lconfigtructK tmp_
	}

	PP_ASSERT_n_sclkCpower__vddainmentuuccesCLK Dependency Table!",
	tmp_
	}

		pm__ak;
		}
		Pe -EIN power _vddainmentint8_t *c}

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_power__vddr_l_set/el(st countCLK Dependency Table!",
	tmp_
	}

		pm__ak;
		}
		Ppower _vddr_lP
.
 d UCLint8_t *c}

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_n_sclkCulvuuccesCLK Dependency Table!",
	tmp_
	}

		pm__ak;
		}
		Pe -EIN ULVint8_t *c}

	PP_tmp_
	}

	tat
	_SOCCLK_DPM_LEVELSividers;

	PP_ASSE		(upower_es[i].sr,
uct pp_hwmgr *hwmgr,
		uint3_SOCCLKsr,
of_soc_did,
	turnpower_es[i])EVELSividers;

	PP_ASSE		(up		retur_8_t)y		dll.
 *_func single SMC GFXCLK structurvoid *es[i], single SMCpower_es[i] *power_es[i]cturvoid *			retur) t);
	currclassificmgr,
_flaguint3ATOM_VP_ASSEe->Soc_Dtting_t *cRecord_V2 *	ingP_
	cord_V2;t32_t soc_clock,power_es[i] *_clock,power_es[i] rrentc_v	_phw__clock,power_es[i](&(power_es[i]->hhhhhhhh)tatimgr->backend);per *hwmncy_on_sc *	er *hwmncy_on_sc;t3ATOM_VP_ASSESs[i] *es[i].8_t)yPP_
ATOM_VP_ASSESs[i] *)es[i];t3ATOM_VP_ASSEPOWERPLAYPDATE *powerf_tabopulv(trent_ATOM_VP_ASSEPOWERPLAYPDATE *)			retur;t3ATOM_VP_ASSESOCSoc_Dtting_t *c = me *eto Mhzmemcl_graphic_l(ATOM_VP_ASSESOCSoc_Dtting_t *c = me *TucTa(((unsigie {long)powerf_tabopulv) +ucTaile ncy cpu(powerf_tabopulv->usi])oltDtting_t * = meOABLE_)Pci3ATOM_VP_ASSEe->Soc_Dtting_t *c = me *DPM_LEVmemcl_graphic_l(ATOM_VP_ASSEe->Soc_Dtting_t *c = me *TucTa(((unsigie {long)powerf_tabopulv) +ucTaile ncy cpu(powerf_tabopulv->usGfxoltDtting_t * = meOABLE_)Pci3ATOM_VP_ASSEMSoc_Dtting_t *c = me *m_LEVmemcl_graphic_l(ATOM_VP_ASSEMSoc_Dtting_t *c = me *TucTa(((unsigie {long)powerf_tabopulv) +ucTaile ncy cpu(powerf_tabopulv->usMoltDtting_t * = meOABLE_)Pcit
	/*PThhhfollowing fields hhhhne &ie(hwmgr,
d here:
	 * id orderedLis &allSs[i]sLis i	led tpower_es[i]->classificmgr,
.ui_labCLleucTa(ile ncy cpu(es[i].8_t)y->usClassificmgr,
) &L);_ATOM_PPLIB_CLendIFICATION_UI_MASK) >>L);_ATOM_PPLIB_CLendIFICATION_UI_SHIFT; tpower_es[i]->classificmgr,
.flag&{
	classificmgr,
_flag;
	/*PNOTE:PThhhhhisha	classificmgr,
2 flag&ie BIOS
	 * thathishne &being us
d right nowi	led tpower_es[i]->classificmgr,
.tempora)y	es[i] r i == 0 |power_es[i]->classificmgr,
.to_bkCdelegedPP_i == 0  |power_es[i]->vmgrdmgr,
.dissllowOnDCleucTa((lockncy cpu(es[i].8_t)y->ul/
	mAndSettings) &L);__
ATOM_VP_ASSEDISALLOW_ON_DC)S!P_0k ?etpower_es[i]->< ef_ta.dissclkFendeModWITHr,
	r i == 0 |power_es[i]->< ef_ta.limitRef_DPhr[i] r i == 0 |power_es[i]->< ef_ta.e -EINVariBright eucTa((lockncy cpu(es[i].8_t)y->ul/
	mAndSettings) &L);__
ATOM_VP_ASSEENDATE_VARIBRIGHT)S!P_0k ?etpower_es[i]->vmgrdmgr,
.sata->gedPowerL UCLK fbsmulpower_es[i]->uvd_UC>Dis.VSocifbsmulpower_es[i]->uvd_UC>Dis.DSocifbsmulpower_es[i]->temperaS>yis.kinifbsmulpower_es[i]->temperaS>yis.kaxPP_0atetper *hwmncy_on_sc tabl_clock,power_es[i]p_per *hwmncy_on_scsL);_[_clock,power_es[i]p_per *hwmncy_on_sc_ck_vo++]k ?etDependency Table!",
lockl_clock,power_es[i]p_per *hwmncy_on_sc_ck_vo <L);__
NUMEe->Soc_D8
uLEVELS),	}

"Per *hwmncy d UCLK exceeds  @brlimitint8_t *curr_la1)i etDependency Table!",
lockl_clock,power_es[i]p_per *hwmncy_on_sc_ck_vo <e,
		st			dep_phy_table);

	vega10,
		st	hhhhhhhActivityPer *hwmncyL UCLK),	}

"Per *hwmncy d UCLK exceeds Driver dimitint8_t *curr_la1)i et/*PPer *hwmncy d UCLK hhhharrangedPt_om{low
		Phigh.led tper *hwmncy_on_sc), to(UC>Didt eto Mhzmemcl_grant8_t)(deL);_[es[i].8_t)y->uci])Ce>DiIng_xLow].ul/e_deaper *hwmncy_on_sc),truct phm_ppDPM_LEVmemcl_grant8_t)(deL);_[es[i].8_t)y->ucGfxCe>DiIng_xLow].ul/e_deaper *hwmncy_on_sc),m = (struct m_LEVmemcl_grant8_t)(deL);_[es[i].8_t)y->ucMemCe>DiIng_xLow].ulMemCekatetper *hwmncy_on_sc tabl_clock,power_es[i]p_per *hwmncy_on_scsL);__[_clock,power_es[i]p_per *hwmncy_on_sc_ck_vo++]k ?tper *hwmncy_on_sc), to(UC>Didt eto Mhzmemcl_grant8_t)(deL);__[es[i].8_t)y->uci])Ce>DiIng_xHigh].ul/e_deanextDPM_LEVmemcl_grantucRevIdSOC_0l_dpm_per *hwmncy_on_sc),truct phm_ppDPM_LEVmemcl_grant8_t)(deL);_[es[i].8_t)y->ucGfxCe>DiIng_xHigh].ul/e_deam >odn_nextDPM_LEVmemcl_grantucRevIdSOC_1l_dpm_pingP_
	cord_V2PP_
ATOM_VP_ASSEe->Soc_Dtting_t *cRecord_V2 *)DPM_LEVmemcl_grant8_t)(de;pm_per *hwmncy_on_sc),truct phm_pppingP_
	cord_V2[es[i].8_t)y->ucGfxCe>DiIng_xHigh].ul/e_deamtetper *hwmncy_on_sc),m = (struct m_LEVmemcl_grant8_t)(deL);_[es[i].8_t)y->ucMemCe>DiIng_xHigh].ulMemCekattries[i].clk =ividers;

	PP_ASSE		(up		retur_8_t)y single SMC GFXCLK structurunsigie {long 8_t)y	ing_x, single SMCpower_es[i] *ngle_d	{t3	tu: mem_crn memory clock tpower_es[i] *ps *)(s_[i]->hhhhhhhh.kagers= PhwVP_ASSEMageratetp&{
	c_v	_phw__clock,power_es[i](&es[i]->hhhhhhhh)
			_S}

	PP_ASSERT_		(upowerf_tabopulv_8_t)y r,
		, 8_t)y	ing_x, si[i]ctur	PP_ASSE		(up		retur_8_t)y		dll.
 *_func)i et/*
	 * Thishis**
 *earl(det time we have&all**
 *dtting_t *rforma
	 * an_P*
 *VBIOS_boot si[i]i	led t/*P
.
 DClcompide me flag&ihhhhishss[i] sata->gs DCled tnext!es[i]->vmgrdmgr,
.dissllowOnDCTucTps->o	_compide me dpm_tablpmps->uvd_UCis.vlevel es[i]->uvd_UC>Dis.VSoc;pmps->uvd_UCis.dlevel es[i]->uvd_UC>Dis.DSoc;pntries[i].clk =ividers;

	PP_ASSERaforubootmd SG8 single SMC GFXCLK structullllPsingle SMC Gtpower_es[i] * Gtpsuint3_SOCCLK.clk =ividers;

	PP_ASSErpply	es[i]_adjuv	_ru_t_usingle SMC GFXCLK structur		single SMCpower_es[i]  *requev	_ptate(&conso single SMCpower_es[i] *curr8_ttpsuint32_t soc_clock,power_es[i] *_clock,psle,
		sc_v	_phw__clock,power_es[i](&requev	_pt->hhhhhhhh)
	it);
	curr_lev
	it);
	currmlev
	i2_t socemptC>Dis->inimui_UC>Dis 
	{0}d,
bool dissclkCm_LEVewiforingd,
bool dissclkCm_LEVewiforing_ *h_fende_loDid,
bool dissclkCm_LEVewiforing_ *h_vrd,
bool  *hckCm_LEVhigh
	i2_t soccgsc< ef_table_mele_mem	{0}d,
conso single Shi_UC>DiCan_mce_tabledimits *ncy_dimitsatit);
	currirn memory clock t uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat..
 */
static int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		uint32_t mem_clock, );
	currcd;
	*	it);
	curr_e_singpes[i].sruct a10perc8_tabl*	it);
	curr_e_singpes[i].srucPP_0,r_e_singpes[i].mrucPP_0*	it);
	currITH_t *;))
e_in),batte)y	es[i] r (empSs[i]UILabCL_Batte)ySOC8_t *cquev	_pt->classificmgr,
.ui_labCLtat
	next_clock,psp_per *hwmncy_on_sc_ck_vo !C_2_mcl0_locate_VI shouldealways have&2 per *hwmncy d UCLKructK ncy_dimits r (empPowerSouhckCACSOC_nt32_t mower_eouhck) ?8_t &int32_t dynmd SG8_mauct phm_ce_tableTn_ac) :8_t &int32_t dynmd SG8_mauct phm_ce_tableTn_dc)i et/* /
	 (strucDPMrformas at DClMAX&ihhithishiniDC.led tnextempPowerSouhckCDCSOC_nt32_t mower_eouhck) AL);*table_info =
		_clock,psp_per *hwmncy_on_sc_ck_vo*current_vool_in_clock,psp_per *hwmncy_on_scs[i].m = (struc>L);_ ncy_dimits),mruc) <			_clock,psp_per *hwmncy_on_scs[i].m = (struce,
		st	ncy_dimits),mruc;_vool_in_clock,psp_per *hwmncy_on_scs[i].truct phm_>L);_ ncy_dimits),sruc) <			_clock,psp_per *hwmncy_on_scs[i].truct phm_p,
		st	ncy_dimits),_lev
	i_clk.eK PP_ASSEpsp_vcy_UCis.evlevel nt32_t vcy_arbite).evlev;K PP_ASSEpsp_vcy_UCis.eclevel nt32_t vcy_arbite).eccekatetcgsc		(uactivec< ef_taslocatent32_t device, &ocat)i et/* _S}

	PP_ DPMCheckVBlankTime.; i++) &vblankTooSh->ge;ed t>inimui_UC>Dis.engineCstruct nt32_t d ef_tabgonfig.kinncore_set/cloDid,
>inimui_UC>Dis.soc_leCstruct nt32_t d ef_tabgonfig.kinnm = set/cloDid,
	next,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meSformaPSgle_dNVAL);empty.",
			return - <		data->yS>v Mry_e_in._e_singpes[i].sruct a10perc8_tabl >C_1 &lock data->yS>v Mry_e_in._e_singpes[i].sruct a10perc8_tabl <=On =,	}

"perc8_tr_lev SOC_V muv	 rangePt_om{1%
		Pn =%,P
.
ting tate(&( SOC_Vnt8_t _e_singpes[i].sruct a10perc8_tabl = 75uctK  ncy_dimits r &int32_t dynmd SG8_mauct phm_ce_tableTn_ac)
	i__e_singpes[i].srucPP_(ncy_dimits),_lev *tur		si_singpes[i].sruct a10perc8_tabl)dsOn =ctK  *tablck_vo O e_single_m->vddVmemconn_levlock_vo - 1;n_d_sck_vo >info ck_vo--ent_vool_in_e_singpes[i].srucP>p,
		ste_single_m->vddVmemconn_levlo8_t)(de[ck_vo].ruc)VAL);___e_singpes[i].srucPP,
		st	e_single_m->vddVmemconn_levlo8_t)(de[ck_vo].ruc;n_d_sbreac;n_d_clk_clL);nextck_vo < 0)8_t _e_singpes[i].sruc O e_single_m->vddVmemconn_levlo8_t)(de[0].ruc;n	i__e_singpes[i].mrucPP_ncy_dimits),mruc;_
	t>inimui_UC>Dis.engineCstruct _e_singpes[i].sruc;
	t>inimui_UC>Dis.soc_leCstruct _e_singpes[i].mruc0 |.entnext>inimui_UC>Dis.engineCstruc< nt32_t DPM_arbite).sruc) <	>inimui_UC>Dis.engineCstruct nt32_t DPM_arbite).sruc;entnext>inimui_UC>Dis.soc_leCstruc< nt32_t DPM_arbite).mruc) <	>inimui_UC>Dis.soc_leCstruct nt32_t DPM_arbite).mruc;eK PP_ASSEpsp_sructth_DPholdct nt32_t DPM_arbite).sructth_DPhold;entnextnt32_t DPM_arbite).sruct
	st_driveNVAL);empty.",
			return -tnt32_t DPM_arbite).sruct
	st_drive <e,
		s			dep_phy_table);

	vega10
	stdriveLimit.engineCstru),	}

	"O	stdriver_lev exceeds dimit_clock nt32_t DPM_arbite).sruct
	st_drive P,
		st				dep_phy_table);

	vega10
	stdriveLimit.engineCstru)blpm_nextnt32_t DPM_arbite).sruct
	st_drive >t nt32_t DPM_arbite).sruc)tur	PP_ASSEpsp_per *hwmncy_on_scs[1].truct phm_p,
		stnt32_t DPM_arbite).sruct
	st_drive0 |.entnextnt32_t DPM_arbite).mruct
	st_driveNVAL);empty.",
			return -tnt32_t DPM_arbite).mruct
	st_drive <e,
		s			dep_phy_table);

	vega10
	stdriveLimit.soc_leCstru),	}

	"O	stdrivermlev exceeds dimit_clock nt32_t DPM_arbite).mruct
	st_drive P,
		st				dep_phy_table);

	vega10
	stdriveLimit.soc_leCstru)blpm_nextnt32_t DPM_arbite).mruct
	st_drive >t nt32_t DPM_arbite).mruc)tur	PP_ASSEpsp_per *hwmncy_on_scs[1].m = (struce,
		stnt32_t DPM_arbite).mruct
	st_drive0 |.entdissclkCm_LEVewiforing_ *h_fende_loDi_ppp
			dpm_table,
,
		sable->entp_phy_table);

	vega10_init_dpm_state(&table DPM table */
	meDissclkMoltSwiforingForFendeLtru)bltdissclkCm_LEVewiforing_ *h_vr_ppp
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meDissclkMoltSwiforForVR)blt *hckCm_LEVhigh_ppp
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meF*hckMoltHightat
	nextle_m.d ef_tabgo_vo OC_0)8_tdissclkCm_LEVewiforing r i == 0 |>odnmcldissclkCm_LEVewiforing r tle_m.d ef_tabgo_vo >_1l_||ock dissclkCm_LEVewiforing_ *h_fende_loDi_||ock dissclkCm_LEVewiforing_ *h_vr_||ock  *hckCm_LEVhigh
	
	sruc O PP_ASSEpsp_per *hwmncy_on_scs[0].truct phmd,
>ruc O PP_ASSEpsp_per *hwmncy_on_scs[0].m = (struat
	nextsruc < >inimui_UC>Dis.engineCstru)8_tsrucPP_(ninimui_UC>Dis.engineCstruc> ncy_dimits),sruc) ?8_t 	ncy_dimits),_lev : >inimui_UC>Dis.engineCstru;entnext>ruc < >inimui_UC>Dis.soc_leCstru) <	>rucPP_(ninimui_UC>Dis.soc_leCstruc>_ncy_dimits),mruc) ?8_t 	ncy_dimits),mlev : >inimui_UC>Dis.soc_leCstru;eK PP_ASSEpsp_per *hwmncy_on_scs[0].truct phmct _lev;K PP_ASSEpsp_per *hwmncy_on_scs[0].m = (struct m_LEat
	next_clock,psp_per *hwmncy_on_scs[1].truct phm_<tur	PP_ASSEpsp_per *hwmncy_on_scs[0].truct phm) <	PP_ASSEpsp_per *hwmncy_on_scs[0].truct phmct <			_clock,psp_per *hwmncy_on_scs[1].truct phmd,ntnext_issclkCm_LEVewiforingNVAL);/* Set MoltP*
 *kaxPof d UCL 0 an_Pd UCL 1led ttnext>ruc < PP_ASSEpsp_per *hwmncy_on_scs[1].m = (stru)tur	>ruc O PP_ASSEpsp_per *hwmncy_on_scs[1].m = (struat
	;/* Fin_P*
 *lowdet MSocPt_cquet *rfhathishwithin
	; * the
		ler= me ITH_t * tatiie {iniDAL
	; *d ttITH_t * P_00 ||*table_info =
		data->m_LEVITH_t *e.pp_t.ck_vo*current_vool_in(data->m_LEVITH_t *e.pp_t.8_t)(de[i].ITH_t * <=OITH_t *) &lock 	(e_in),m_LEVITH_t *e.pp_t.8_t)(de[i].t_cquet *r>p,
		st PP_ASSEpsp_per *hwmncy_on_scs[0].m = (stru) &lock 	(e_in),m_LEVITH_t *e.pp_t.8_t)(de[i].t_cquet *r<p,
		st PP_ASSEpsp_per *hwmncy_on_scs[1].m = (stru)) <			>ruc O e_in),m_LEVITH_t *e.pp_t.8_t)(de[i].t_cquet *
	i_clk PP_ASSEpsp_per *hwmncy_on_scs[0].m = (struct m_LEat.m >odn_dpm_next_clock,psp_per *hwmncy_on_scs[1].m = (struc< <			_clock,psp_per *hwmncy_on_scs[0].m = (stru)tur	PP_ASSEpsp_per *hwmncy_on_scs[0].m = (struct
		st PP_ASSEpsp_per *hwmncy_on_scs[1].m = (stru0 |.entnext,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meSformaPSgle_dNVAL);*table_info =
		_clock,psp_per *hwmncy_on_sc_ck_vo*current_voo_clock,psp_per *hwmncy_on_scs[i].truct phm_p _e_singpes[i].sruc;
	t	_clock,psp_per *hwmncy_on_scs[i].m = (struce _e_singpes[i].mruc0 |.m |.entries[i].clk =ividers;

	PP_ASSEtiidt a10es[i]s_UC>Disgle_ a10_hwmguct pp_hwmgr *hwmgr,
		, conso void *inpuiuint3conso single Shi_s	(upower_es[i].inpui *ngle_sle,
		(conso single Shi_s	(upower_es[i].inpui *)inpuid,
conso single _clock,power_es[i] *_clock,psle,
		c_v	_conso_phw__clock,power_es[i](ngle_sp_pnew_es[i])EV memory clock to use to populate the structure.
 * @return   0 on success..
 */
sASSERT_i	struct phm_ppt_vsructt_graphic_ln *table_info =
			(struct phm_*)(hwmgr-sruc O PP_ASSEpsp_per *hwmncy_on_scsL);_[_clock,psp_per *hwmncy_on_sc_ck_vo - 1].truct phmd,..
 */
sASSERT_i	struct phm_ppt_vmructt_graphic_ln *table_info =
		m = ruct phm_*)(hwmgr->ruc O PP_ASSEpsp_per *hwmncy_on_scsL);_[_clock,psp_per *hwmncy_on_sc_ck_vo - 1].m = (stru0 |2_t socemptC>Dis->in_UC>Dis 
	{0}d,
t);
	currirn memory cgsc< ef_table_mele_mem	{0}d,)
e_in),need_upd[i].2_informatio0d,
	next,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meODNinACSata->gen||ock,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(&t DPM table */
	meODNinDCSata->geNVAL);*table_info =
		sructt_gralock_vo*current_vool_insruc O=	sructt_gralo find UCLK in SOC_V)n_d_sbreac;n_dclL);next! *tablerpply	
	stdrive_next_
.
tingsck_vkllock 	DPMPDATEeOD_UPDATE_SSoc) &l =
>=	sructt_gralock_voent_voo/* /heck SSoc{iniDAL's->inimui (strusL);_ * in	c_ve DeepSleep divider upd[i]hish_cquirNd,W		; *d tt;next_setti< ef_tabtiming.kinnc phm_inn_r !t
		st >in_UC>Dis.engineCstruInSR &lock 	(>in_UC>Dis.engineCstruInSR >p,
		st VEGASSEMINIMUM_ENGINE_CLOCK_||ock k data->< ef_tabtiming.kinnc phm_inn_r >p,
		st VEGASSEMINIMUM_ENGINE_CLOCK)) <			data->need_upd[i].2_informat|= DPMPDATEeUPDATE_SSoc;n_dclL);cgsc		(uactivec< ef_taslocatent32_t device, &ocat)i et;next_setti< ef_tabtiming.nui_existingc< ef_tas !t
		stle_m.d ef_tabgo_vo)
			data->need_upd[i].2_informat|= DPMPDATEeUPDATE_MSoc;n_m >odn_dpm_*table_info =
		sructt_gralock_vo*current_vool_insruc O=	sructt_gralo find UCLK in SOC_V)n_d_sbreac;n_dclL);next=
>=	sructt_gralock_voe
			data->need_upd[i].2_informat|= DPMPDATEeOD_UPDATE_SSoc;n_d>odn_dpm__/* /heck SSoc{iniDAL's->inimui (strusL);_ * in	c_ve DeepSleep divider upd[i]hish_cquirNd,W		; *d tt;next_setti< ef_tabtiming.kinnc phm_inn_r !t
		st >in_UC>Dis.engineCstruInSR &lock 	(>in_UC>Dis.engineCstruInSR >p,
		st VEGASSEMINIMUM_ENGINE_CLOCK_||ock k data->< ef_tabtiming.kinnc phm_inn_r >p,
		st VEGASSEMINIMUM_ENGINE_CLOCK)) <			data->need_upd[i].2_informat|= DPMPDATEeUPDATE_SSoc;n_dclL);*table_info =
		mructt_gralock_vo*current_vool_inmruc O=	mructt_gralo find UCLK in SOC_V)n_d_sbreac;n_dclL);cgsc		(uactivec< ef_taslocatent32_t device, &ocat)i et;next=
>=	mructt_gralock_voe
			data->need_upd[i].2_informat|= DPMPDATEeOD_UPDATE_MSoc;pnt;next_setti< ef_tabtiming.nui_existingc< ef_tas !t
		stle_m.d ef_tabgo_vo_||ock k=
>=	mructt_gralock_voe
			data->need_upd[i].2_informat|= DPMPDATEeUPDATE_MSoc;n_mntries[i].clk =ividers;

	PP_ASSERT_WITH_Can_masloyS>sructmruct a10on_sclu
		single SMCr *hwmgr,
		, conso void *inpuiuint3a10(
			hwmgr0ci3conso single Shi_s	(upower_es[i].inpui *ngle_sle,
		(conso single Shi_s	(upower_es[i].inpui *)inpuid,
conso single _clock,power_es[i] *_clock,psle,
		c_v	_conso_phw__clock,power_es[i](ngle_sp_pnew_es[i])EV memory clock to use to populate the structure.
 * @on success..
 */
statit);
	currsruc O PP_ASSEpsp_per *hwmncy_on_scsL);_[_clock,psp_per *hwmncy_on_sc_ck_vo - 1].truct phmd,.*)(hwmgr->ruc O PP_ASSEpsp_per *hwmncy_on_scsL);_[_clock,psp_per *hwmncy_on_sc_ck_vo - 1].m = (stru0 |2_t socPP_ASSEd phm_ppt_v2_information*table_info =
	0 |2_t socPP_ASSEd phm_ppt_vgolden.2_informatiic_lne_in),Dolden.2_informad,.*)(hwmgr-2_inck_vo, (stru0perc8_td,
t);
	currirn
	next,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meODNinACSata->gen||ock,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meODNinDCSata->geNVALL);next!data->need_upd[i].2_informat&lock !*tablerpply	
ptimized_
.
tingst&lock !*tablerpply	
	stdrive_next_
.
tingsck_vke
			ries[i].clnt;next_settirpply	
	stdrive_next_
.
tingsck_vkllock 	DPMPDATEeOD_UPDATE_SSoc) {ock  *hODia10ck_vo O 0;ock k da10ck_vo < ia10_hwmglo	(struct .cd;
	*	ik k da10ck_vorrent_voo	ia10_hwmglo	(struct . find UCLK da10ck_vo].< data->d,
		st _settiodn.2_informa.odn.core_(stru0 find UCLK.lock   per *hwmncy_on_sc_8_t)(de[da10ck_vo].< data-;_voo	ia10_hwmglo	(struct . find UCLK da10ck_vo].SOC_V d,
		st _settiodn.2_informa.odn.core_(stru0 find UCLK.lock   per *hwmncy_on_sc_8_t)(de[da10ck_vo].(stru0 |d_clk_clL);next_settirpply	
	stdrive_next_
.
tingsck_vkllock 	DPMPDATEeOD_UPDATE_MSoc) {ock  *hODia10ck_vo O 0;ock k da10ck_vo < ia10_hwmglom = ruct .cd;
	*	ik k da10ck_vorrent_voo	ia10_hwmglom = ruct . find UCLK da10ck_vo].< data->d,
		st _settiodn.2_informa.odn.soc_lev(stru0 find UCLK.lock   per *hwmncy_on_sc_8_t)(de[da10ck_vo].< data-;_voo	ia10_hwmglom = ruct . find UCLK da10ck_vo].SOC_V d,
		st _settiodn.2_informa.odn.soc_lev(stru0 find UCLK.lock   per *hwmncy_on_sc_8_t)(de[da10ck_vo].(stru0 |d_clk_clL);next(data->need_upd[i].2_informat& DPMPDATEeUPDATE_SSocl_||ock dtablerpply	
ptimized_
.
tingst||ock t_settirpply	
	stdrive_next_
.
tingsck_vkllock 		DPMPDATEeOD_UPDATE_SSoc)ent_voo
	}

	PP_ASSERT_poTH_CODE[LocateVddcGivenClouccesCLK   Dependency Table!",
	
	}

			breeeak;
		}
		PltageIndeSSoc{during \	breeePtageIndNewDPMtC>DisSs[i]s Funcgr,
t_clock  > mem_m
	}

	j
	<_clL);next(data->need_upd[i].2_informat& DPMPDATEeUPDATE_MSocl_||ock t_settirpply	
	stdrive_next_
.
tingsck_vkllock 		DPMPDATEeOD_UPDATE_MSocl)t_voo
	}

	PP_ASSERT_poTH_CODE[Locsoc_level(struuccesCLK   Dependency Table!",
	
	}

			breeeak;
		}
		PltageIndeMSoc{during \	breeePtageIndNewDPMtC>DisSs[i]s Funcgr,
t_clock  > mem_m
	}

	j
	<_cl.m >odn_dpm_next!data->need_upd[i].2_informat&lock  !*tablerpply	
ptimized_
.
tingse
			ries[i].clnt;next_settineed_upd[i].2_informat& DPMPDATEeOD_UPDATE_SSoct&lock  _settismu_feaS>yis[GNLD_DPM_e->Soc].sata->gedNVAL);o	ia10_hwmgloL);o		(struct . find UCLK da10_hwmglo	(struct .cd;
	 - 1].L);o	SOC_V d sruc;
	t		next,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(&e(& DPM table */
	meOD6PlusinACSata->gen||ock	ck,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(&te(& DPM table */
	meOD6PlusinDCSata->geNVAL);	__/* Ne	}
		Pdo calcWITHr,
	bas
d ,
	the
DoldencDPMrformaL);	__ * as**
 *HeaSmap GPUe(dep_oaxishis*also	bas
d ,
L);	__ * *
 *dtte(&( SOC_VsL);	__ */	breeePependency Table!",
lock   	Dolden.2_informalo	(struct . find UCLKlock   	[Dolden.2_informalo	(struct .cd;
	 - 1].SOC_Vate(&te(&"Divide by 0t_clock    *curr_la1)i etk k da10ck_vo = ia10_hwmglo	(struct .cd;
	 < 2 ?8_t 				0 :,2_informalo	(struct .cd;
	 - 2*	ik k *table_inda10ck_voo =
> 1o =--ent_voo	ool_insruc > Dolden.2_informalo	(struct . find UCLKlock   	[Dolden.2_informalo	(struct .cd;
	 - 1].SOC_Vent_voo	oo	(stru0perc8_t d,
		st Ta((sruc - Dolden.2_informalo	(struct . find UCLKlock   		[Dolden.2_informalo	(struct .cd;
	 - 1].SOC_Ven*lock   		n =b /lock   		Dolden.2_informalo	(struct . find UCLKlock   		[Dolden.2_informalo	(struct .cd;
	 - 1].SOC_Vi etk k o	ia10_hwmglo	(struct . find UCLK i].SOC_V d,
		st 		Dolden.2_informalo	(struct . find UCLK i].SOC_V +,
		st Ta(Dolden.2_informalo	(struct . find UCLK i].SOC_V *lock   		(stru0perc8_t)dsOn =ctk   		m >odn_nextDolden.2_informalo,
		st 		D(struct . find UCLK da10_hwmglo	(struct .cd;
	-1].SOC_V o,
		st 		sruc)VAL);__oo	(stru0perc8_t d,
		st Ta((Dolden.2_informalo	(struct . find UCLKlock   		[Dolden.2_informalo	(struct .cd;
	 - 1].SOC_V -,
		st 		sruc)V*	n =b /lock   		Dolden.2_informalo	(struct . find UCLKlock   		[Dolden.2_informalo	(struct .cd;
	-1].SOC_Vi etk k o	ia10_hwmglo	(struct . find UCLK i].SOC_V d,
		st 		Dolden.2_informalo	(struct . find UCLK i].SOC_V -,
		st Ta(Dolden.2_informalo	(struct . find UCLK i].SOC_V *lock   		(stru0perc8_t)dsOn =ctk   		m >odnetk k o	ia10_hwmglo	(struct . find UCLK i].SOC_V d,
		st 		Dolden.2_informalo	(struct . find UCLK i].SOC_Vctk   	clk_||clk_|clnt;next_settineed_upd[i].2_informat& DPMPDATEeOD_UPDATE_MSoct&lock  _settismu_feaS>yis[GNLD_DPM_USoc].sata->gedNVAL);o2_informalo,
		m = ruct . find UCLK da10_hwmglom = ruct .cd;
	 - 1].L);oSOC_V d mruc;_
	t	next,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(&e( DPM table */
	meOD6PlusinACSata->gen||ock	c,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(&e(& DPM table */
	meOD6PlusinDCSata->geNVALL);eePependency Table!",
lock  Dolden.2_informalom = ruct . find UCLK
   		[Dolden.2_informalom = ruct .cd;
	 - 1].SOC_Vate(&te"Divide by 0t_clock  *curr_la1)i etk kda10ck_vo = ia10_hwmglom = ruct .cd;
	 < 2 ?8_t 			0 :,2_informalom = ruct .cd;
	 - 2*	ik k*table_inda10ck_voo =
> 1o =--ent_voo	ol_inmruc > Dolden.2_informalom = ruct . find UCLK
   			[Dolden.2_informalom = ruct .cd;
	-1].SOC_Vent_voo	oo(stru0perc8_t dxt(mruc -,
		st TDolden.2_informalom = ruct . find UCLK
   				[Dolden.2_informalom = ruct .cd;
	-1].SOC_Ven*lock   	n =b /lock   	Dolden.2_informalom = ruct . find UCLK
   				[Dolden.2_informalom = ruct .cd;
	-1].SOC_Vi etk k oia10_hwmglom = ruct . find UCLK i].SOC_V d,
		st 	Dolden.2_informalom = ruct . find UCLK i].SOC_V +,
		st T(Dolden.2_informalom = ruct . find UCLK i].SOC_V *lock   	(stru0perc8_t)dsOn =ctk   	m >odn_nextDolden.2_informalom = ruct . find UCLK
   				[2_informalom = ruct .cd;
	-1].SOC_Vc>_nruc)VAL);__oo(stru0perc8_t dxt(Dolden.2_informalom = ruct . find UCLK
   				[Dolden.2_informalom = ruct .cd;
	-1].SOC_V - nruc)V*lock   	n =b /lock   	Dolden.2_informalom = ruct . find UCLK
   				[Dolden.2_informalom = ruct .cd;
	-1].SOC_Vi etk k oia10_hwmglom = ruct . find UCLK i].SOC_V d,
		st 	Dolden.2_informalom = ruct . find UCLK i].SOC_V -,
		st T(Dolden.2_informalom = ruct . find UCLK i].SOC_V *lock   	(stru0perc8_t)dsOn =ctk   	m >odnetk k oia10_hwmglom = ruct . find UCLK i].SOC_V d,
		st 	Dolden.2_informalom = ruct . find UCLK i].SOC_V;lk_||clk_|cl<_clL);next(data->need_upd[i].2_informat&ock tDPMPDATEeOD_UPDATE_SSoct+ DPMPDATEeUPDATE_SSocll_||ock dtablerpply	
ptimized_
.
tingsent_voo
	}

	PP_ASSERT_poTH_CODE[LocateVddcGivenClouccesCLK   Dependency Table!",
	
	}

			breeeak;
		}
		PltageIndeSSoc{during \	breeePtageIndNewDPMtC>DisSs[i]s Funcgr,
t_clock  > mem_m
	}

	j
	<_clL);nextdata->need_upd[i].2_informat&ock  tDPMPDATEeOD_UPDATE_MSoct+ DPMPDATEeUPDATE_MSoc)ent_voo
	}

	PP_ASSERT_poTH_CODE[Locsoc_level(struuccesCLK   Dependency Table!",
	
	}

			breeeak;
		}
		PltageIndeMSoc{during \	breeePtageIndNewDPMtC>DisSs[i]s Funcgr,
t_clock  > mem_m
	}

	j
	<_cl.m
	_SOCCLK_DPM_LEVELSividers;

	PP_ASSEtrim_i	struct phngle_susingle SMC GFXCLK structur.
 */
sASSERT_i	struct phm_ppt_vt phm_pptcturu);
	currIow_dimit) t);
	currhigh_dimituint3t);
	currirn
	*table_info =
		d_informalock_vo*current_vonext(d_informalo find UCLK i].SOC_V <rIow_dimitl_||ockable(d_informalo find UCLK i].SOC_V >rhigh_dimitu)L);o2_informalo find UCLK i].< data->dpi == 0 ||>odnetk 2_informalo find UCLK i].< data->dpm_table.etries[i].clk =ividers;

	PP_ASSEtrim_i	struct phngle_s_with_k_vkusingle SMC GFXCLK structur.
 */
sASSERT_i	struct phm_ppt_vt phm_pptcturu);
	currIow_dimit) t);
	currhigh_dimitcturu);
	currdissclkCd phk_vke
nt3t);
	currirn
	*table_info =
		d_informalock_vo*current_vonext(d_informalo find UCLK i].SOC_V <rIow_dimitl_||ockable(d_informalo find UCLK i].SOC_V >rhigh_dimitu)L);o2_informalo find UCLK i].< data->dpi == 0 ||>odn next! (1 << i)t& dissclkCd phk_vke)L);o2_informalo find UCLK i].< data->dpi == 0 ||>odnetk 2_informalo find UCLK i].< data->dpm_table.etries[i].clk =ividers;

	PP_ASSEtrim_t phngle_susingle SMC GFXCLK structurconso single _clock,power_es[i] *_clock,ps)int32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
statit);
	currhigh_dimit0ck_voo etDependency Table!",
t_clock,psp_per *hwmncy_on_sc_ck_vo >C_1),	}

"power es[i] didhne &have&any per *hwmncy d UCLnt8_t *curr_la1)i ethigh_dimit0ck_vo dxt_clock,psp_per *hwmncy_on_sc_ck_vo OC_1l_?e0 :,1;eK PP_ASSEtrim_i	struct phngle_su structur	n *table_info =
		 to(o =
	)ctur	PP_ASSEpsp_per *hwmncy_on_scs[0]. to(UC>Dictur	PP_ASSEpsp_per *hwmncy_on_scs[high_dimit0ck_vo]. to(UC>Di);eK PP_ASSEtrim_i	struct phngle_s_with_k_vku structur	n *table_info =
			(struct pctur	PP_ASSEpsp_per *hwmncy_on_scs[0].truct phmctur	PP_ASSEpsp_per *hwmncy_on_scs[high_dimit0ck_vo].truct phmctur	_setti< esclkCd phk_vke;eK PP_ASSEtrim_i	struct phngle_su structur	n *table_info =
		m = ruct pctur	PP_ASSEpsp_per *hwmncy_on_scs[0].m = (structur	PP_ASSEpsp_per *hwmncy_on_scs[high_dimit0ck_vo].m = (stru);pntries[i].clk =ividerst);
	currPP_ASSEtiidtlowdet0 find UCL(tur.
 */
sASSERT_i	struct phm_ppt_vruct p
nt3t);
	currirn
	*table_info =
		formalock_vo*current_vonextformalo find UCLK i].< data-)L);obreac;n_.entries[i]iclk =ividerst);
	currPP_ASSEtiidthighdet0 find UCL(tur.
 */
sASSERT_i	struct phm_ppt_vruct p
nt3t);
	curritio0d,
	nextformalock_vo <=OMAX_REGULAR_DPM_NUMBERNVAL);*table_informalock_vo*cu >r0o =--ent_voonextformalo find UCLK i - 1].< data-)L);otries[i]i - 1;n_dcl.m >odn_dpm_0_locate_DPMrT_ppt_Has Too Many E_t)(detructotries[i]MAX_REGULAR_DPM_NUMBER - 1;n_.entries[i]iclk =ividersvoid PP_ASSErpply	daocsinimui_ce_table*cquev	u
		single SMCr *hwmgr,
		uint3_SOCCLclk =ividers;

	PP_ASSE		(u to(ing_x_ *h_ncy_urucuct pp_hwmgr *hwmgr,
		uint32_t socruct pp_h1ct phm_ce_tabledtting_t *hm_ppt_vvddVmemcm_pptconnmlev
	i2_t soctatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		uint32_t mem_clock,K PddVmemcm_pptconnmlev  O e_single_m->vddVmemconnmruc;_
	ries[i]PddVmemcm_pptconnmlevlo8_t)(de[NUMEUSoc_D8
uLEVELS - 1].SddIndt+ 1clk =ividers;

	PP_ASSEasloyS> finbootupnd UCL(ct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
statit);
	currsto Mhzidx;eK PP_ASSErpply	daocsinimui_ce_table*cquev	uuccesCLK
_next!data->yS>v Mry_e_in._ruct a10keyEdissclkdNVAL);next_settismc_es[i].o =
			(stbootmd UCL !t
		st*table_info =
			(struct .t phngle_. tftcsinmd UCL)VAL);__Dependency Table!",
	smui_s/
s_msgncy (yr_with_ependencysL);__uccess.smui++)pm__
PPSMC_MSG_SetStftMinGfxoltByIng_x,ock  _settismc_es[i].o =
			(stbootmd UCL),	}

	"k;
		}
		Pierrstft->inrsruc ing_xt_clock > mem_max_clock ? 	t*table_info =
			(struct .t phngle_. tftcsinmd UCL d,
		st_settismc_es[i].o =
			(stbootmd UCL0 |.m |.entnext!data->yS>v Mry_e_in.mruct a10keyEdissclkdNVAL);next_settismc_es[i].o =
		m = bootmd UCL !t
		st*table_info =
		m = ruct . finngle_. tftcsinmd UCL)VAL);_next_settismc_es[i].o =
		m = bootmd UCL OC_NUMEUSoc_D8
uLEVELS - 1)VAL);___to MhzidxPP_ASSERT_		(u to(ing_x_ *h_ncy_urucuuccesCLK   _Dependency Table!",
	smui_s/
s_msgncy (yr_with_ependencysL);__;__uccess.smui++)pm__
_
PPSMC_MSG_SetStftMini])oltByIng_x,ock  ___to Mhzidx),	}

			"k;
		}
		Pierrstft->inruruc ing_xt_clock k > mem_max_clock ? 	tm >odn_dpm_ _Dependency Table!",
	smui_s/
s_msgncy (yr_with_ependencysL);__;_uccess.smui++)pm__
_
PPSMC_MSG_SetStftMinUoltByIng_x,ock  ___settismc_es[i].o =
		m = bootmd UCL),	}

			"k;
		}
		Pierrstft->inruruc ing_xt_clock k > mem_max_clock ? 	tm? 	t*table_info =
		m = ruct . finngle_. tftcsinmd UCL d,
		st_settismc_es[i].o =
		m = bootmd UCL0 |.m |.entries[i].clk =ividers;

	PP_ASSEasloyS> finncy_d UCL(ct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
statK PP_ASSErpply	daocsinimui_ce_table*cquev	uuccesCLK
_next!data->yS>v Mry_e_in._ruct a10keyEdissclkdNVAL);next_settismc_es[i].o =
			(stncy_d UCL !t
		st*table_info =
			(struct .t phngle_. tftcscy_d UCL)VAL);__Dependency Table!",
	smui_s/
s_msgncy (yr_with_ependencysL);__uccess.smui++)pm__
PPSMC_MSG_SetStftMaxGfxoltByIng_x,ock  _settismc_es[i].o =
			(stscy_d UCL),	}

	"k;
		}
		Pierrstft->axrsruc ing_xt_clock > mem_max_clock ? 	t*table_info =
			(struct .t phngle_. tftcscy_d UCL d,
		st_settismc_es[i].o =
			(stscy_d UCL0 |.m |.entnext!data->yS>v Mry_e_in.mruct a10keyEdissclkdNVAL);next_settismc_es[i].o =
		m = ncy_d UCL !t
		st*table_info =
		m = ruct . finngle_. tftcscy_d UCL)VAL);__Dependency Table!",
	smui_s/
s_msgncy (yr_with_ependencysL);__uccess.smui++)pm__
PPSMC_MSG_SetStftMaxUoltByIng_x,ock  _settismc_es[i].o =
		m = ncy_d UCL),	}

	"k;
		}
		Pierrstft->axrmruc ing_xt_clock > mem_max_clock ? 	t*table_info =
		m = ruct . finngle_. tftcscy_d UCL d,
		st_settismc_es[i].o =
		m = scy_d UCL0 |.m |.entries[i].clk =ividers;

	PP_ASSE		neraSe0 find UCL_n_sclkCk_vku
		single SMCr *hwmgr,
		, conso void *inpuiuint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
staticonso single Shi_s	(upower_es[i].inpui *ngle_sle,
		(conso single Shi_s	(upower_es[i].inpui *)inpuid,
conso single _clock,power_es[i] *_clock,psle,
		c_v	_conso_phw__clock,power_es[i](ngle_sp_pnew_es[i])EV ;

	io etDependency Table!",
!PP_ASSEtrim_t phngle_su struc PP_ASSEps),	}

"Attempt
		PTrimcDPMrSs[i]s F		pp_t_clock*curr_la1)i et_settismc_es[i].o =
			(stbootmd UCL =tur	PP_ASSEtiidtlowdet0 find UCL(n *table_info =
			(struct p)bltdsettismc_es[i].o =
			(stncy_d UCL =tur	PP_ASSEtiidthighdet0 find UCL(n *table_info =
			(struct p)bltdsettismc_es[i].o =
		m = bootmd UCL Otur	PP_ASSEtiidtlowdet0 find UCL(n *table_info =
		m = ruct p)bltdsettismc_es[i].o =
		m = ncy_d UCL =tur	PP_ASSEtiidthighdet0 find UCL(n *table_info =
		m = ruct p)bletDependency Table!",
!PP_ASSEasloyS> finbootupnd UCL(uccesC,	}

"Attempt
		PasloyScDPMrBootup L UCLK F		pp_t_clock*curr_la1)i tDependency Table!",
!PP_ASSEasloyS> finncy_d UCL(uccesC,	}

"Attempt
		PasloyScDPMrMax L UCLK F		pp_t_clock*curr_la1)i t *hle_indsettismc_es[i].o =
			(stbootmd UCL0 =
		data->smc_es[i].o =
			(stscy_d UCL0curre
	t*table_info =
			(struct .t phd UCLK i].< data->dpm_tabl
 t *hle_indsettismc_es[i].o =
		m = bootmd UCL0 =
		data->smc_es[i].o =
		m = scy_d UCL0curre
	t*table_info =
		m = ruct . find UCLK i].< data->dpm_tabl
tries[i].clk =;

	PP_ASSEn_sclkCd esclkCvcy_ fiuct pp_hwmgr *hwmgr,
		, bool n_sclkuint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
statK next_settismu_feaS>yis[GNLD_D8
uVCE].sata->gedNVAL);Dependency Table!",
EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
n_sclk,ock  _settismu_feaS>yis[GNLD_D8
uVCE].smu_feaS>yi_bitmap),	}

	"Attempt
		PE -EIN/DissclkcDPMrVCE{
		pp_t_clock > mem_ma1)i <	_settismu_feaS>yis[GNLD_D8
uVCE].< data->dp< data; |.entries[i].clk =ividers;

	PP_ASSEasd[i].sructth_DPholduct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
stati;

	
			hwmgr0ci3u);
	currIow_sruct);
errupttth_DPholdct 0d,
	next,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meSrucThrottleLowNotificmgr,
)te(&&xtnt32_t DPM_arbite).sructth_DPholdc!t
		st*tableIow_sruct);
errupttth_DPhold)ent_vo*tableIow_sruct);
errupttth_DPhold e,
		s			dep_DPM_arbite).sructth_DPhold;e		Iow_sruct);
errupttth_DPhold e,
		s*tableIow_sruct);
errupttth_DPholdi etkdata->smc_es[i].o =
		p		retur.LowGfxoltI;
erruptTh_DPhold e,
		scpuncy lock(Iow_sruct);
errupttth_DPhold)at
	;/* Thishmessabl will*also	e -EIN SmcToHoso I;
errupt */	br
			hwmgrsmui_s/
s_msgncy (yr_with_ependencysuccess.smui++)pm__
PPSMC_MSG_SetLowGfxoltI;
erruptTh_DPhold)pm__
(u);
	cur)Iow_sruct);
errupttth_DPhold)at|.entries[i]_DPM_LEVELSividers;

	PP_ASSEs	(upower_es[i].t_vksusingle SMC GFXCLK structurconso void *inpuiuint3a10(tmp_
	}

			
			hwmgr0ci32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
statiPPT_ppturr*			retur r &idata->smc_es[i].o =
		p		returtat
	tmp_
	}

	PP_ASSERT_tiidt a10es[i]s_UC>Disgle_ a10_hwmgur,
		, inpuiui tDependency Table!",
!tmp_
	}

		pm__ak;
		}
		PtiidcDPMrngle_slUC>Dis incDPMrformaint8_t *c}

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_po_WITH_Can_masloyS>sructmruct a10on_sclur,
		, inpuiui tDependency Table!",
!tmp_
	}

		pm__ak;
		}
		PltageIndean_PasloyScSSoc{MSoc{DPMron_sclint8_t *c}

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_		neraSe0 find UCL_n_sclkCk_vkur,
		, inpuiui tDependency Table!",
!tmp_
	}

		pm__ak;
		}
		P		neraSe{DPMron_sc < data->k_vkint8_t *c}

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_asd[i].sructth_DPholduuccesCLK Dependency Table!",
!tmp_
	}

		pm__ak;
		}
		Pupd[i]hSSoc{th_DPholdint8_t *c}

	PP_tmp_
	}

	tat
	_S}

	PP_ASSERT_copabopulv_cy (yrsuccess.smui++)pm__(u);
8urr*)			retur) PPTDATECLK Dependency Table!",
!
	}

		pm__ak;
		}
		PuployScPPformaint > mem_m
	}

	j
	
 dtablerpply	
ptimized_
.
tings>dpi == 0 |_settirpply	
	stdrive_next_
.
tingsck_vklt 0d,
	ries[i].clk =ividers;

	PP_ASSE fin		(u rucuct pp_hwmgr *hwmgr,
		, bool Iowuint32_t socrMCpower_es[i] *psci32_t soc_clock,power_es[i] *_clock,ps;entnextnt32_ OC_NULL)
k > mem_max_clocatetp&{
	uccess.*cquev	_ptd,
	next,s OC_NULL)
k > mem_max_clocatet_clock,psle	c_v	_phw__clock,power_es[i](&pt->hhhhhhhh)
	
	nextIowuik > mem_mPP_ASSEpsp_per *hwmncy_on_scs[0].truct phmd,
>odnetk> mem_mPP_ASSEpsp_per *hwmncy_on_scspm__
[_clock,psp_per *hwmncy_on_sc_ck_vo - 1].truct phmd,k =ividers;

	PP_ASSE fin		(umrucuct pp_hwmgr *hwmgr,
		, bool Iowuint32_t socrMCpower_es[i] *psci32_t soc_clock,power_es[i] *_clock,ps;entnextnt32_ OC_NULL)
k > mem_max_clocatetp&{
	uccess.*cquev	_ptd,
	next,s OC_NULL)
k > mem_max_clocatet_clock,psle	c_v	_phw__clock,power_es[i](&pt->hhhhhhhh)
	
	nextIowuik > mem_mPP_ASSEpsp_per *hwmncy_on_scs[0].m = (stru0 |>odnetk> mem_mPP_ASSEpsp_per *hwmncy_on_scspm__
[_clock,psp_per *hwmncy_on_sc_ck_vo-1].m = (stru0 k =ividers;

	PP_ASSE		(ugpunpowerusingle SMC GFXCLK structur.
 */
sSMCgpunpower *queryp
nt3t);
	currSOC_Vi etDependency Table!",
	smui_s/
s_msgncy (yrsuccess.smui++)pm__PPSMC_MSG_GetCurrPkgPwsC,	}

"k;
		}
		P		t curr8_t p
 *abl powert_clock*curr_lax_clock ?et_clock,reyS>arg_t_om (yrsuccess.smui++) &SOC_Ve;et/* power SOC_V is*ans;

eger */	bquerytir	stablegpunpower = SOC_V << 8d,
	ries[i].clk =ividers;

	PP_ASSEreyS>s/
sorusingle SMC GFXCLK strucs;

	idx,ock llllP void *SOC_Vas;

	*sizep
nt3t);
	currs Mhzidx,	mructidx,	activity0perc8_t dx0ci32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat..
 */
sPP_ASSEd phm_ppt_v2_information*table_info =
	0 |;

	
	tlt 0d,
	ewifor (idx)nt_vc_ve AMDGPU_empSENSOR_e->_SSoc:etk> mmgrsmui_s/
s_msgncy (yrsuccess.smui++) PPSMC_MSG_GetCurr8_tGfxoltI;dex)i <	next!> ment_voo_clock,reyS>arg_t_om (yrsuccess.smui++) &s Mhzidxk ? 	t*((u);
	curon SOC_Ven= ia10_hwmglo	(struct . find UCLK s Mhzidx].SOC_V;lk_|*sizen= 40 |.m |obreac;n_c_ve AMDGPU_empSENSOR_e->_MSoc:etk> mmgrsmui_s/
s_msgncy (yrsuccess.smui++) PPSMC_MSG_GetCurr8_tUoltI;dex)i <	next!> ment_voo_clock,reyS>arg_t_om (yrsuccess.smui++) &m Mhzidxk ? 	t*((u);
	curon SOC_Ven= ia10_hwmglom = ruct . find UCLK m Mhzidx].SOC_V;lk_|*sizen= 40 |.m |obreac;n_c_ve AMDGPU_empSENSOR_ePU_LOAD:etk> mmgrsmui_s/
s_msgncy (yr_with_ependencysuccess.smui++) PPSMC_MSG_GetA	stablGfxActivity,_0k ?<	next!> ment_voo_clock,reyS>arg_t_om (yrsuccess.smui++) &activity0perc8_tk ? 	t*((u);
	curon SOC_Ven= activity0perc8_t >On = ?On = : activity0perc8_t;lk_|*sizen= 40 |.m |obreac;n_c_ve AMDGPU_empSENSOR_ePU_TEMP:etk*((u);
	curon SOC_Ven= PP_ASSEthehwmlE		(utemperaS>yiuuccesCLK  *sizen= 40 |.breac;n_c_ve AMDGPU_empSENSOR_UVD_POWER:etk*((u);
	curon SOC_Ven= *tableuvd_power_g[i]d_?e0 :,1;e  *sizen= 40 |.breac;n_c_ve AMDGPU_empSENSOR_VCE_POWER:etk*((u);
	curon SOC_Ven= *tablevcy_power_g[i]d_?e0 :,1;e  *sizen= 40 |.breac;n_c_ve AMDGPU_empSENSOR_GPU_eOWER:etknext*sizen< sizeof(.
 */
sSMCgpunpower)e
			rien= ax_clocat_d>odn_dpm__*sizen= sizeof(.
 */
sSMCgpunpower);
			rien= PP_ASSE		(ugpunpoweru strucs(.
 */
sSMCgpunpoweron SOC_Ve0 |.m |obreac;n_dtte(&(:etk> mmgrax_clocat_dbreac;n_.etries[i]_DLEVELSividers;

	PP_ASSEnotify (yr_d ef_tabghangeusingle SMC GFXCLK structurbool hasc< efuint3_SOCCLKsmui_s/
s_msgncy (yr_with_ependencysuccess.smui++)pm__PPSMC_MSG_SetUoltF_v	Swifor)pm__hasc< ef_?e0 :,1)clk =;

	PP_ASSEd ef_tabg phm_ce_table*cquev	usingle SMC GFXCLK structur.
 */
sSMCd ef_tabg phm_*cquev	 *g phm_*cquint3a10(
			hwmgr0ci3enum amdup		g phm_type ructtype = g phm_*cq->clphm_typeci3u);
	currructt_cq = g phm_*cq->clphm_t_cqgle_khzdsOn =0ci3DSPSoc_e ructselecwmgr0ci3u);
	currruct*cquev	 t 0d,
	ewifor (ructtype)nt_vc_ve amdup		dcef	g phm:etkructselecwmgrDSPSoc_DCEFSoc;n_dbreac;n_c_ve amdup		d ef	g phm:etkructselecwmgrDSPSoc_DISPSoc;n_dbreac;n_c_ve amdup		pixsc_c phm:etkructselecwmgrDSPSoc_PIXSoc;n_dbreac;n_c_ve amdup		phy_c phm:etkructselecwmgrDSPSoc_PHYSoc;n_dbreac;n_dtte(&(:etk0_locate_[D ef_tatC>DiVe_tablRcquev	]Invmgrde(dep_oTypetructotri		hwmgr-1at_dbreac;n_.e
	next!> }

	j_dpm_ruct*cquev	 t (ructt_cq << 16) | ructselecwctotri		hwmgrsmui_s/
s_msgncy (yr_with_ependencysuccess.smui++)pm__
PPSMC_MSG_Rcquev	D ef_tatC>DiByF_cq)pm__
ruct*cquev	)at|.entries[i]_DPM_LEVELSividersu);
8urrPP_ASSE		(uuruct);dexusingle SMC GFXCLK structur	2_t socruct pp_h1ct phm_ce_tabledtting_t *hm_ppt_vmructt_graclock k u);
	currt_cquet *p
nt3t);
8urrcd;
	*	it);
8urrirn
	nextmructt_graphC_NULL ||	mructt_gralock_vo OC_0)8_tries[i].clntck_vo dxtt);
8ur)(mructt_gralock_voe;
 t *hle_info =
		ck_vo*current_vone(mructt_gralo8_t)(de[i].ruc >=rt_cquet *p
ock*curr_li;n_.entries[i]i-1clk =ividers;

	PP_ASSEnotify (yr_d ef_tabgonfig_afncy,ps_adjuv	m8_tu
		single SMCr *hwmgr,
		uint3memory clock to use to populate the structure.
 * @return   0 on success..
 */
sASSERT_i	struct phm_ppt_v2_informatilaten*table_info =
	.dcef	o =
	0 |2_t soctatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		unt32_t mem_clo0 |2_t soctatic int1ct phm_ce_tabledtting_t *hm_ppt_vmructt_gra O e_single_m->vddVmemconnmruc;_3t);
	curridx;e3t);
	currnui_activec< efs dx0ci32_t soccgsc< ef_table_mele_mem	{0}d,
2_t socemptC>Dis->in_UC>Dis 
	{0}d,
t);
	currirn memory SMCd ef_tabg phm_*cquev	 g phm_*cqrn
	ne_m.modngle_mem_NULLatetcgsc		(uactivec< ef_taslocatent32_t device, &ocat)i etnui_activec< efs dxle_m.d ef_tabgo_vorn
	nextnui_activec< efs >_1l
oo_clock,notify (yr_d ef_tabghangeu strucsi == )0 |>odnetk_clock,notify (yr_d ef_tabghangeu strucsm_tauctK nin_UC>Dis.dcefCstruct nt32_t d ef_tabgonfig.kinndcef	set/clid,
>in_UC>Dis.dcefCstruInSR t nt32_t d ef_tabgonfig.kinndcef	deemcsleemcset/clid,
>in_UC>Dis.soc_leCstruct nt32_t d ef_tabgonfig.kinnm = set/cloDid,
	*table_info =
		d_informalock_vo*current_vonextd_informalo find UCLK i].SOC_V O=	min_UC>Dis.dcefCstru)L);obreac;n_.entnext=
		d_informalock_voj_dpm_ruphm_*cq.g phm_type = amdup		dcef	g phm;pm_ruphm_*cq.g phm_t_cqgle_khzd= ia10_hwmglo find UCLK i].SOC_V;lk_next!PP_ASSEd ef_tabg phm_ce_table*cquev	u strucs&g phm_*cquent_vooDependency Table!",
	smui_s/
s_msgncy (yr_with_ependencysL);__;uccess.smui++) PPSMC_MSG_SetMinDeepSleepDcefcliclock k>in_UC>Dis.dcefCstruInSR /n =b		breeeaAttempt
		Pierrdivider *tabDCEFSoc{
		pp_t_ce0 |.m >odn_dpm_ 0_locate_Attempt
		PierrHard Min *tabDCEFSoc{
		pp_t_j
	<_cl.m >odn_dpm_0_ldebug("Canne &tiidc*cquev	]d_DCEFSoct_j
	<.entnext>in_UC>Dis.soc_leCstruc!C_0)nt_vondxPP_ASSERT_		(uuruct);dexu strucsmructt_grac >in_UC>Dis.soc_leCstru)
	i__mui_s/
s_msgncy (yr_with_ependencysuccess.smui++) PPSMC_MSG_SetStftMinUoltByIng_x, idxk ? 	*table_info =
		m = ruct . finngle_. tftcsinmd UCL=ridx;e3.entries[i].clk =ividers;

	PP_ASSEt*hckC finhighdet(ct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
statK _settismc_es[i].o =
			(stbootmd UCL =tudsettismc_es[i].o =
			(stncy_d UCL =tur	PP_ASSEtiidthighdet0 find UCL(n *table_info =
			(struct p)bltdsettismc_es[i].o =
		m = bootmd UCL Otudsettismc_es[i].o =
		m = ncy_d UCL =tur	PP_ASSEtiidthighdet0 find UCL(n *table_info =
		m = ruct p)bletDependency Table!",
!PP_ASSEasloyS> finbootupnd UCL(uccesC,	}

"k;
		}
		PuployScbootron_sc 		Phighdett_clock*curr_la1)i etDependency Table!",
!PP_ASSEasloyS> finncy_d UCL(uccesC,	}

"k;
		}
		PuploySc fi->axron_sc 		Phighdett_clock*curr_la1)i etries[i].clk =ividers;

	PP_ASSEt*hckC finlowdet(ct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
statK _settismc_es[i].o =
			(stbootmd UCL =tudsettismc_es[i].o =
			(stncy_d UCL =tur	PP_ASSEtiidtlowdet0 find UCL(n *table_info =
			(struct p)bltdsettismc_es[i].o =
		m = bootmd UCL Otudsettismc_es[i].o =
		m = ncy_d UCL =tur	PP_ASSEtiidtlowdet0 find UCL(n *table_info =
		m = ruct p)bletDependency Table!",
!PP_ASSEasloyS> finbootupnd UCL(uccesC,	}

"k;
		}
		PuployScbootron_sc 		Phighdett_clock*curr_la1)i etDependency Table!",
!PP_ASSEasloyS> finncy_d UCL(uccesC,	}

"k;
		}
		PuploySc fi->axron_sc 		Phighdett_clock*curr_la1)i etries[i].cllk =ividers;

	PP_ASSEant*hckC finln_scluct pp_hwmgr *hwmgr,
		uint32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
statet_settismc_es[i].o =
			(stbootmd UCL =tur	PP_ASSEtiidtlowdet0 find UCL(n *table_info =
			(struct p)bltdsettismc_es[i].o =
			(stncy_d UCL =tur	PP_ASSEtiidthighdet0 find UCL(n *table_info =
			(struct p)bltdsettismc_es[i].o =
		m = bootmd UCL Otur	PP_ASSEtiidtlowdet0 find UCL(n *table_info =
		m = ruct p)bltdsettismc_es[i].o =
		m = ncy_d UCL =tur	PP_ASSEtiidthighdet0 find UCL(n *table_info =
		m = ruct p)bletDependency Table!",
!PP_ASSEasloyS> finbootupnd UCL(uccesC,	}

"k;
		}
		PuployScDPMrBootup L UCLKt_clock*curr_la1)i etDependency Table!",
!PP_ASSEasloyS> finncy_d UCL(uccesC,	}

"k;
		}
		PuployScDPMrMax L UCLKt_clock*curr_la1)i tries[i].clk =ividers;

	PP_ASSE		t_profilingcructm_vkusingle SMC GFXCLK struc enum amdu fint*hckd_d UCL d UCL		breeu);
	curonsructm_vk) t);
	currvmructm_vk) t);
	currv to(k_vke
nt32_t soctatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		uint32_t mem_clock,K nextformagle_m->vddVmemconnslevlogo_vo >_VEGASSEUMD_PSTATE_e->SocuLEVELt&lockformagle_m->vddVmemconnsto Mhlogo_vo >_VEGASSEUMD_PSTATE_SOCSocuLEVELt&lockformagle_m->vddVmemconnm Mhlogo_vo >_VEGASSEUMD_PSTATE_MSocuLEVEL)nt_vonsructm_vkur VEGASSEUMD_PSTATE_e->SocuLEVEL;e  *sto(k_vkur VEGASSEUMD_PSTATE_SOCSocuLEVEL;e  *mructm_vkur VEGASSEUMD_PSTATE_MSocuLEVEL;e .entnextd UCL OC_AMD_D8
uFORCEDuLEVEL_PROFILEEMIN_SSoc) {ocknsructm_vkur 0ci3m >odn_nextd UCL OC_AMD_D8
uFORCEDuLEVEL_PROFILEEMIN_MSoc) {ock*mructm_vkur 0ci3m >odn_nextd UCL OC_AMD_D8
uFORCEDuLEVEL_PROFILEEPEAc) {ocknsructm_vkur formagle_m->vddVmemconnslevlogo_vo -,1;e  *sto(k_vkur formagle_m->vddVmemconnsto Mhlogo_vo -,1;e  *mructm_vkur formagle_m->vddVmemconnm Mhlogo_vo -,1;e .etries[i].clk =ividers;

	PP_ASSEset/fan.control_modnusingle SMC GFXCLK struc *)(hwmgr->odnuint3a10(
			hwmgr0ci
	ewifor (>odnunt_vc_ve AMD_FAN_CTRL_NONE:totri		hwmgrPP_ASSEtan.ctrlEset/fan.speed_perc8_tu strucsn =b0 |.breac;n_c_ve AMD_FAN_CTRL_MANUAL:etknext,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_state(& DPM table */
	meMicrocodnFanControl)e
			ri		hwmgrPP_ASSEtan.ctrlEstopC(yrifan.controluuccesCLK  breac;n_c_ve AMD_FAN_CTRL_AUTO:totri		hwmgrPP_ASSEtan.ctrlEset/ivider_modnu strucsmodnu ?<	next!> }

	j
			ri		hwmgrPP_ASSEtan.ctrlEstartC(yrifan.controluuccesCLK  breac;n_dtte(&(:etkbreac;n_.etries[i]_DPM_LEVELSividers;

	PP_ASSE fint*hck0 find UCL(single SMC GFXCLK structur		enum amdu fint*hckd_d UCL d UCLuint3a10(
	wmgr0ci3u);
	currsructm_vkur 0ci3*)(hwmgr->ructm_vkur 0ci3*)(hwmgr-sto(k_vkur 0ci3*)(hwmgr-profile_modn(k_vkur AMD_D8
uFORCEDuLEVEL_PROFILEESTANDARD |ock	ckAMD_D8
uFORCEDuLEVEL_PROFILEEMIN_SSoc |ock	ckAMD_D8
uFORCEDuLEVEL_PROFILEEMIN_MSoc |ock	ckAMD_D8
uFORCEDuLEVEL_PROFILEEPEAc;entnextd UCL OC_nt32_t dfind UCL)8_tries[i]_DLEV
	next!(nt32_t dfind UCL &-profile_modn(k_vkuent_vo/* enter-profilesmodn, save&curr8_t d UCL	 dissclk 	(s cg*/	brnextd UCL &-profile_modn(k_vku_dpm_ uccess.saved0 find UCL C_nt32_t dfind UCL;
			cgscset/cloDigatingces[i](nt32_t device,lock k AMD_IP_BLOCK_TYPE_e->,lock k AMD_CG_STATE_UNGATEj
	<_cl.m >odn_dpm_/* exit-profilesmodn, _DPga1e d UCL	 e -EIN 	(s cg*/	brnext!td UCL &-profile_modn(k_vku)VAL);_nextd UCL OC_AMD_D8
uFORCEDuLEVEL_PROFILEEEXIT)L);otd UCL C_nt32_t saved0 find UCL;
			cgscset/cloDigatingces[i](nt32_t device,lock kAMD_IP_BLOCK_TYPE_e->,lock kAMD_CG_STATE_GATEj
	<_cl.mi
	ewifor (d UCL)VAL)c_ve AMD_D8
uFORCEDuLEVEL_HIGH:etk> mmgrPP_ASSEt*hckC finhighdet(uccesCLK  next> melock*curr_l_DLEV_ uccess. find UCL C_d UCL;
		breac;n_c_ve AMD_D8
uFORCEDuLEVEL_LOW:etk> mmgrPP_ASSEt*hckC finlowdet(uccesCLK  next> melock*curr_l_DLEV_ uccess. find UCL C_d UCL;
		breac;n_c_ve AMD_D8
uFORCEDuLEVEL_AUTO:totri	PP_ASSERT_ant*hckC finln_scluuccesCLK  next> melock*curr_l_DLEV_ uccess. find UCL C_d UCL;
		breac;n_c_ve AMD_D8
uFORCEDuLEVEL_PROFILEESTANDARD:n_c_ve AMD_D8
uFORCEDuLEVEL_PROFILEEMIN_SSoc:n_c_ve AMD_D8
uFORCEDuLEVEL_PROFILEEMIN_MSoc:n_c_ve AMD_D8
uFORCEDuLEVEL_PROFILEEPEAc:totri	PP_ASSERT_		t_profilingcructm_vku strucsd UCL	 &sructm_vk) &mructm_vk) & to(k_vkeLK  next> melock*curr_l_DLEV_ uccess. find UCL C_d UCL;
		PP_ASSEt*hckCg phm_d UCL(ucces) PP_SSoccsn<<sructm_vk);
		PP_ASSEt*hckCg phm_d UCL(ucces) PP_MSoccsn<<mructm_vk);
		breac;n_c_ve AMD_D8
uFORCEDuLEVEL_MANUAL:etkuccess. find UCL C_d UCL;
		breac;n_c_ve AMD_D8
uFORCEDuLEVEL_PROFILEEEXIT:n_dtte(&(:etkbreac;n_.entnextd UCL OC_AMD_D8
uFORCEDuLEVEL_PROFILEEPEAc &l uccess.saved0 find UCL !C_AMD_D8
uFORCEDuLEVEL_PROFILEEPEAc)
		PP_ASSEset/fan.control_modnuucces) AMD_FAN_CTRL_NONE)0 |>odn_nextd UCL !C_AMD_D8
uFORCEDuLEVEL_PROFILEEPEAc &l uccess.saved0 find UCL OC_AMD_D8
uFORCEDuLEVEL_PROFILEEPEAc)
		PP_ASSEset/fan.control_modnuucces) AMD_FAN_CTRL_AUTO)i etries[i].clk =ividers;

	PP_ASSEget/fan.control_modnusingle SMC GFXCLK struuint32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
statetnext_settismu_feaS>yis[GNLD_FAN_CONTROL].< data->ddpi == )8_tries[i]AMD_FAN_CTRL_MANUAL; |>odnetk> mem_mAMD_FAN_CTRL_AUTOclk =ividers;

	PP_ASSEget/daocpower_d UCL(single SMC GFXCLK structursingle amdup		simplkCg phm_le_me*ocat)
nt32_t soctatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		unt32_t mem_clo0 |2_t soctatig phm_an_mce_tabledimits *ncy_dimits ilatenformagle_m->ncy_g phm_ce_tableonnacrn
	ne_mlo8_gine_ncy_g phm d mcy_dimitstisclid,
le_m->noc_levncy_g phm d mcy_dimitstimruc;_
	ries[i]0clk =ividersvoid PP_ASSE		(u rucsusingle SMC GFXCLK structur.
 */
sp		g phm_ln_scl_with_eIndncy *g phms)
nt32_t soctatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		unt32_t mem_clo0 |2_t soctatic int1ct phm_ce_tabledtting_t *hm_ppt_vmemcm_pptemory_formagle_m->vddVmemconnslev;t3t);
	currirn
	*table_info =
		demcm_pptlock_vo*current_vonextdemcm_pptlo8_t)(de[i].ruc)VAL);_g phmss. set[g phmss.nui_ln_scl].(strusgle_khzd=lock kdemcm_pptlo8_t)(de[i].ruc;L);_g phmss.nui_ln_scl++
	<_cl.mi
k =ividerst);
	currPP_ASSE		(umem_eIndncyusingle SMC GFXCLK structuru);
	currrutru)L{etnextg phm >= ME
uFREQ_LOW_LATENCYt&lock g phm < ME
uFREQ_HIGH_LATENCY)8_tries[i]ME
uLATENCY_HIGH0 |>odn_nextg phm >= ME
uFREQ_HIGH_LATENCY)8_tries[i]ME
uLATENCY_LOW; |>odnetk> mem_mME
uLATENCY_ERRclk =ividersvoid PP_ASSE		(umemg phmsusingle SMC GFXCLK structur.
 */
sp		g phm_ln_scl_with_eIndncy *g phms)
nt32_t soctatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		unt32_t mem_clo0 |2_t soctatic int1ct phm_ce_tabledtting_t *hm_ppt_vmemcm_pptemory_formagle_m->vddVmemconnmlev
	i2_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat.t);
	currirn
	g phmss.nui_ln_sclur 0ci3_settimructeIndncyfo =
		ck_vo O 0;o
	*table_info =
		demcm_pptlock_vo*current_vonextdemcm_pptlo8_t)(de[i].ruc)VAL);_g phmss. set[g phmss.nui_ln_scl].(strusgle_khzd=lock_settimructeIndncyfo =
		8_t)(delock[_settimructeIndncyfo =
		ck_vo].t_cquet *d=lock kdemcm_pptlo8_t)(de[i].ruc;L);_g phmss. set[g phmss.nui_ln_scl].eIndncyfle_usd=lock_settimructeIndncyfo =
		8_t)(delock[_settimructeIndncyfo =
		ck_vo].eIndncy =lock kPP_ASSE		(umem_eIndncyu structur		 kdemcm_pptlo8_t)(de[i].ruc);L);_g phmss.nui_ln_scl++
	<_3_settimructeIndncyfo =
		ck_vo++
	<_cl.mik =ividersvoid PP_ASSE		(udcefclphmsusingle SMC GFXCLK structur.
 */
sp		g phm_ln_scl_with_eIndncy *g phms)
nt32_t soctatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		unt32_t mem_clo0 |2_t soctatic int1ct phm_ce_tabledtting_t *hm_ppt_vmemcm_pptemory_formagle_m->vddVmemconndcefclv;t3t);
	currirn
	*table_info =
		demcm_pptlock_vo*current_vog phmss. set[i].(strusgle_khzd= demcm_pptlo8_t)(de[i].ruc;L);g phmss. set[i].eIndncyfle_usd= 0;ockg phmss.nui_ln_scl++
	<mik =ividersvoid PP_ASSE		(usto Mphmsusingle SMC GFXCLK structur.
 */
sp		g phm_ln_scl_with_eIndncy *g phms)
nt32_t soctatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		unt32_t mem_clo0 |2_t soctatic int1ct phm_ce_tabledtting_t *hm_ppt_vmemcm_pptemory_formagle_m->vddVmemconnsto Mh;t3t);
	currirn
	*table_info =
		demcm_pptlock_vo*current_vog phmss. set[i].(strusgle_khzd= demcm_pptlo8_t)(de[i].ruc;L);g phmss. set[i].eIndncyfle_usd= 0;ockg phmss.nui_ln_scl++
	<mik =ividers;

	PP_ASSEget/t phm_by_type_with_eIndncyusingle SMC GFXCLK structurenum amdup		g phm_type typectur.
 */
sp		g phm_ln_scl_with_eIndncy *g phms)
nt32wifor (type)nt_vc_ve amdup		sys_c phm:etkPP_ASSE		(u rucsuucces) g phms);
		breac;n_c_ve amdup		m = (stru:etkPP_ASSE		(umemg phmsuucces) g phms);
		breac;n_c_ve amdup		dcef	g phm:etkPP_ASSE		(udcefclphmsuucces) g phms);
		breac;n_c_ve amdup		 to(UC>Di:etkPP_ASSE		(u to Mphmsuucces) g phms);
		breac;n_dtte(&(:etk> mrr_la1;e3.entries[i].clk =ividers;

	PP_ASSEget/t phm_by_type_with_ce_tablusingle SMC GFXCLK structurenum amdup		g phm_type typectur.
 */
sp		g phm_ln_scl_with_ce_tabl *g phms)
nt32_t soctatic int vega10_populate_single_memory_level(struct pp_hwmgr *hwmgr,
		unt32_t mem_clo0 |2_t soctatic int1ct phm_ce_tabledtting_t *hm_ppt_vmemcm_ppt;t3t);
	currirn
	2wifor (type)nt_vc_ve amdup		m = (stru:etkmemcm_pptem e_single_m->vddVmemconnmruc;_3	breac;n_c_ve amdup		dcef	g phm:etkmemcm_pptem e_single_m->vddVmemconndcefclv;t3dbreac;n_c_ve amdup		d ef	g phm:etkmemcm_pptem e_single_m->vddVmemconnd efclv;t3dbreac;n_c_ve amdup		pixsc_c phm:etkmemcm_pptem e_single_m->vddVmemconnpixclv;t3dbreac;n_c_ve amdup		phy_c phm:etkmemcm_pptem e_single_m->vddVmemconnphyclv;t3dbreac;n_dtte(&(:etk> mrr_la1;e3.ent*table_info =
		demcm_pptlock_vo*current_vog phmss. set[i].(strusgle_khzd= demcm_pptlo8_t)(de[i].ruc;L);g phmss. set[i].ce_tableinnmv dxtt);
	cur)tformagle_m->vddc_lookumcm_pptlotur		ent)(de[demcm_pptlo8_t)(de[i].SddInd].us_Sdd);ockg phmss.nui_ln_scl++
	<mintnext=
		demcm_pptlock_vo)etk> mrr_la1;eetries[i].clk =ividers;

	PP_ASSEset/wIndhwmrusg *h_(strusgrangesusingle SMC GFXCLK structur.
 */
sp		w= setl_with_g phm_*angesu to15 *w= with_g phm_*angesuint32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat	WIndhwmrusgtate_sin r &idata->smc_es[i].o =
		wIndh_wmrusgt_clock,i;

	
			hwmgr0ci3u);
	currirn
	next!data->yS>v Mry_e_in.d esclkCwIndh_wmruNVAL);*table_info =
		w= with_g phm_*angess.nui_w= setl_dmif*current_vo	m_pptloWIndhwmruRow[WM_DCEFSoc][i].MinCstructtur		cpuncy lo16((u);
16_melock	(w= with_g phm_*angess.w= setl_dmif[i].w= kinndcefruct);_khzb /lock n =b0 |.	m_pptloWIndhwmruRow[WM_DCEFSoc][i].MaxCstructtur		cpuncy lo16((u);
16_melock	(w= with_g phm_*angess.w= setl_dmif[i].w= kaxndcefruct);_khzb /lock n =b0 |.	m_pptloWIndhwmruRow[WM_DCEFSoc][i].MinUoltcttur		cpuncy lo16((u);
16_melock	(w= with_g phm_*angess.w= setl_dmif[i].w= kinnmemg ct);_khzb /lock n =b0 |.	m_pptloWIndhwmruRow[WM_DCEFSoc][i].MaxUoltcttur		cpuncy lo16((u);
16_melock	(w= with_g phm_*angess.w= setl_dmif[i].w= kaxnmemg ct);_khzb /lock n =b0 |.	m_pptloWIndhwmruRow[WM_DCEFSoc][i].WmS.
ting dxtt);
8ur)tur		 w= with_g phm_*angess.w= setl_dmif[i].w= set/id
	<_clL);*table_info =
		w= with_g phm_*angess.nui_w= setl_mcif*current_vo	m_pptloWIndhwmruRow[WM_SOCSoc][i].MinCstructtur		cpuncy lo16((u);
16_melock	(w= with_g phm_*angess.w= setl_mcif[i].w= kinn_to Mhzi;_khzb /lock n =b0 |.	m_pptloWIndhwmruRow[WM_SOCSoc][i].MaxCstructtur		cpuncy lo16((u);
16_melock	(w= with_g phm_*angess.w= setl_mcif[i].w= kaxn_to Mhzi;_khzb /lock n =b0 |.	m_pptloWIndhwmruRow[WM_SOCSoc][i].MinUoltcttur		cpuncy lo16((u);
16_melock	(w= with_g phm_*angess.w= setl_mcif[i].w= kinnmemg ct);_khzb /lock n =b0 |.	m_pptloWIndhwmruRow[WM_SOCSoc][i].MaxUoltcttur		cpuncy lo16((u);
16_melock	(w= with_g phm_*angess.w= setl_mcif[i].w= kaxnmemg ct);_khzb /lock n =b0 |.	m_pptloWIndhwmruRow[WM_SOCSoc][i].WmS.
ting dxtt);
8ur)tur		 w= with_g phm_*angess.w= setl_mcif[i].w= set/id
	<_cl_3_settiwIndh_wmrusgbitmap dxWIndhMmrusExv M;t|.entries[i]_DPM_LEVELSividers;

	PP_ASSEt*hckCg phm_d UCL(single SMC GFXCLK structurenum p		g phm_type typec *)(hwmgr->_vke
nt32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat	;

	io etnextnt32_t dfind UCL &-(AMD_D8
uFORCEDuLEVEL_AUTO |ock	cAMD_D8
uFORCEDuLEVEL_LOW |ock	cAMD_D8
uFORCEDuLEVEL_HIGH))
k > mem_max_clocatet2wifor (type)nt_vc_ve PP_SSoc:L);*table_info =
		32*current_vo	next>_vku&-(1 << i))tur		breac;n__cl_3_settismc_es[i].o =
			(stbootmd UCL =	io et;*table_in31*cu >=r0o =--ent_voonext>_vku&-(1 << i))tur		breac;n__cl_3_settismc_es[i].o =
			(stncy_d UCL =	io et;Dependency Table!",
!PP_ASSEasloyS> finbootupnd UCL(uccesC,	}

"k;
		}
		PuployScbootron_sc 		Plowdett_clock*curr_lax_clock ?ettDependency Table!",
!PP_ASSEasloyS> finncy_d UCL(uccesC,	}

"k;
		}
		PuploySc fi->axron_sc 		Phighdett_clock*curr_lax_clock ? 	breac;n_vc_ve PP_MSoc:L);*table_info =
		32*current_vo	next>_vku&-(1 << i))tur		breac;n__cl_3_settismc_es[i].o =
		m = bootmd UCL O	io et;*table_in31*cu >=r0o =--ent_voonext>_vku&-(1 << i))tur		breac;n__cl_3_settismc_es[i].o =
		m = ncy_d UCL =	io et;Dependency Table!",
!PP_ASSEasloyS> finbootupnd UCL(uccesC,	}

"k;
		}
		PuployScbootron_sc 		Plowdett_clock*curr_lax_clock ?ettDependency Table!",
!PP_ASSEasloyS> finncy_d UCL(uccesC,	}

"k;
		}
		PuploySc fi->axron_sc 		Phighdett_clock*curr_lax_clock ?ettbreac;n_vc_ve PP_PCIE:n_dtte(&(:etkbreac;n_.entries[i].clk =ividers;

	PP_ASSEpr;

	g phm_ln_scl(single SMC GFXCLK structurenum p		g phm_type typec chaCLKbufe
nt32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat	.
 */
sASSERT_i	struct phm_ppt_vsructt_gra O n *table_info =
			(struct pat	.
 */
sASSERT_i	struct phm_ppt_vmructt_gra O n *table_info =
		m = ruct pat	.
 */
sASSERT_pciehm_ppt_vpciehm_ppt_O n *table_info =
		pciehm_ppttat	;

	i, now, sizen= 0atet2wifor (type)nt_vc_ve PP_SSoc:L);next_settiyS>v Mry_e_in._ruct a10keyEdissclkdN
	ttbreac;n_vtDependency Table!",
	smui_s/
s_msgncy (yrsuccess.smui++)pm___PPSMC_MSG_GetCurr8_tGfxoltI;dex),	}

	"Attempt
		P		t curr8_t sruc ing_x{
		pp_t_clock > mem_ma1)i <	Dependency Table!",
!PP_ASSEreyS>arg_t_om (yrsuccess.smui++)lock &now),	}

	"Attempt
		PreyS sruc ing_x{
		pp_t_clock > mem_ma1)i L);*table_info =
		sructt_gralock_vo*currelocksizen+= spr;

f(buf + size, "%d: %uMhz %s\n_clock ki, sructt_gralo find UCLK i].SOC_V sOn =clock kle_i= nowl_?e"*" : "");
		breac;n_c_ve PP_MSoc:L);next_settiyS>v Mry_e_in.mruct a10keyEdissclkdN
	ttbreac;n_vtDependency Table!",
	smui_s/
s_msgncy (yrsuccess.smui++)pm___PPSMC_MSG_GetCurr8_tUoltI;dex),	}

	"Attempt
		P		t curr8_t mruc ing_x{
		pp_t_clock > mem_ma1)i <	Dependency Table!",
!PP_ASSEreyS>arg_t_om (yrsuccess.smui++)lock &now),	}

	"Attempt
		PreyS mruc ing_x{
		pp_t_clock > mem_ma1)i L);*table_info =
		mructt_gralock_vo*currelocksizen+= spr;

f(buf + size, "%d: %uMhz %s\n_clock ki, mructt_gralo find UCLK i].SOC_V sOn =clock kle_i= nowl_?e"*" : "");
		breac;n_c_ve PP_PCIE:n_tDependency Table!",
	smui_s/
s_msgncy (yrsuccess.smui++)pm___PPSMC_MSG_GetCurr8_tLintI;dex),	}

	"Attempt
		P		t curr8_t mruc ing_x{
		pp_t_clock > mem_ma1)i <	Dependency Table!",
!PP_ASSEreyS>arg_t_om (yrsuccess.smui++)lock &now),	}

	"Attempt
		PreyS mruc ing_x{
		pp_t_clock > mem_ma1)i L);*table_info =
		pciehm_pptlock_vo*currelocksizen+= spr;

f(buf + size, "%d: %s %s\n_c	i,lock klpciehm_pptlopciehgen i] OC_0)_?e"2.5GB, x1" :lock klpciehm_pptlopciehgen i] OC_1l_?e"5.0GB, x16" :lock klpciehm_pptlopciehgen i] OC_2l_?e"8.0GB, x16" : ""clock kle_i= nowl_?e"*" : "");
		breac;n_dtte(&(:etkbreac;n_.etries[i]sizeEVELSividers;

	PP_ASSE  ef_tabgonfigurmgr,
bghanged.t_vkusingle SMC GFXCLK struuint32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stati;

	
			hwmgr0ci3u);
	currnui_es[ied.onnd ef_tasmgr1at	WIndhwmrusgtatwmhm_ppt_O n *tablesmc_es[i].o =
		wIndh_wmrusgt_clock,i2_t soccgsc< ef_table_mele_mem	{0}d,etnext(_settiwIndh_wmrusgbitmap &xWIndhMmrusExv M)t&lock !(_settiwIndh_wmrusgbitmap &xWIndhMmrusLoySed)ent_vo_S}

	PP_ASSERT_copabopulv_cy (yrsuccess.smui++)pm__(u);
8urr*)wmhm_ppt, WMTDATECLK 	Dependency Table!",

	}

			ak;
		}
		Pupd[i]hWMTDATEint > mem_mx_clock ? 	_settiwIndh_wmrusgbitmap |=xWIndhMmrusLoySed
	<mintnext_settiwIndh_wmrusgbitmap &xWIndhMmrusLoySed)nt_voggsc		(uactivec< ef_taslocatent32_t device, &ocat)i 		nui_es[ied.onnd ef_tasmgrle_m.d ef_tabgo_vorni__mui_s/
s_msgncy (yr_with_ependencysuccess.smui++)
___PPSMC_MSG_NumOfD ef_tas,rnui_es[ied.onnd ef_tas)at|.entries[i]_DPM_LEVELS;

	PP_ASSEn_sclkCd esclkCuvd_ fiuct pp_hwmgr *hwmgr,
		, bool n_sclkuint32_t soc_clock, uint8_t *current_soc_did,
	turn   0 on success..
 */
statK next_settismu_feaS>yis[GNLD_D8
uUVD].sata->gedNVAL);Dependency Table!",
EINVAL);n_sclkC(yrifeaS>yissuccess.smui++)pm__
n_sclk,ock  _settismu_feaS>yis[GNLD_D8
uUVD].smu_feaS>yi_bitmap),	}

	"Attempt
		PE -EIN/DissclkcDPMrUVD{
		pp_t_clock > mem_ma1)i <	_settismu_feaS>yis[GNLD_D8
uUVD].< data->dp< data; |.etries[i].clk =ividers;

	PP_ASSEpower_g[i]Cvcyuct pp_hwmgr *hwmgr,
		, bool bg[i]uint32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
statet_settivcy_power_g[i]d_= bg[i];
	ries[i]PP_ASSEn_sclkCd esclkCvcy_ fiur,
		, !bg[i]uclk =ividers;

	PP_ASSEpower_g[i]Cuvduct pp_hwmgr *hwmgr,
		, bool bg[i]uint32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
statet_settiuvd_power_g[i]d_= bg[i];
	ries[i]PP_ASSEn_sclkCd esclkCuvd_ fiur,
		, !bg[i]uclk =ividers;
line bool PP_ASSEarecpower_d UCLs_equalsL);__conso single _clock,per *hwmncy_on_sc_vpl1,L);__conso single _clock,per *hwmncy_on_sc_vpl2uint3_SOCCLK((pl1tisto(UC>Di_i= pl2tisto(UC>Di)t&lock (pl1titruct phm_i= pl2titruct phm)t&lock (pl1tim = (stru_i= pl2tim = (stru)uclk =ividers;

	PP_ASSEcheck0es[i]s_equalssingle SMC GFXCLK structur		conso single SMC GCpower_es[i] *pss[i]1,L);_conso single SMC GCpower_es[i] *pss[i]2, bool *equaluint3conso single _clock,power_es[i] *psa;t3conso single _clock,power_es[i] *psbat	;

	io etnextpss[i]1phC_NULL ||	pss[i]2phC_NULL ||	equal OC_NULL)
k > mem_max_clocatetp&cur c_v	_conso_phw__clock,power_es[i](pss[i]1)i <psbur c_v	_conso_phw__clock,power_es[i](pss[i]2e;et/* If the tworngle_sldon't n_sn have&the sndernuiber of per *hwmncyron_scl&they c_nne &be&the snderngle_. */	bnextpsap_per *hwmncy_on_sc_ck_vo != psbp_per *hwmncy_on_sc_ck_vo) {ocknequal Opi == 0 |tries[i].cl3.ent*table_info =
		psap_per *hwmncy_on_sc_ck_vo*current_vonext!PP_ASSEarecpower_d UCLs_equals&tpsap_per *hwmncy_on_scK i]), &(psbp_per *hwmncy_on_scK i])u)VAL);_/* If we have&fk_vd n_sn one per *hwmncyron_sc pair&that is*differ8_t the sgle_slare*differ8_t. */	bcknequal Opi == 0 |ttries[i].cl3_cl.mi
	/* If all per *hwmncyron_scl&are*the sndertry
		Puse*the UVD{(strus
		Pbreac the tie.*/	bnequal Op(tpsap_uvd_rucs.voltct= psbp_uvd_rucs.volt)t&l tpsap_uvd_rucs.doltct= psbp_uvd_rucs.doltp)bltnequal &Op(tpsap_vckCg cs.evoltct= psbp_vckCg cs.evolt)t&l tpsap_vckCg cs.ecoltct= psbp_vckCg cs.ecoltp)bltnequal &Op(psap_sructth_DPholdct= psbp_sructth_DPhold)i etries[i].clk =ividersbool
PP_ASSEcheck0eyriasd[i]._cquired. *h_  ef_tabgonfigurmgr,
usingle SMC GFXCLK struuint32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
statibool isiasd[i]._cquired Opi == 0 |2_t soccgsc< ef_table_mele_mem	{0, 0, NULL}atetcgsc		(uactivec< ef_taslocatent32_t device, &ocat)i etnext_setti< ef_tabtiming.nui_exv Mingcd ef_tasm!grle_m.d ef_tabgo_vo)
k isiasd[i]._cquired Opt_tabl
tnext,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_sta  DPM table */
	meSrucDeepSleep)NVAL);next_setti< ef_tabtiming.>in_UC>Dit);_srm!grnt32_t d ef_tabgonfig.kinncorecset/cloDit);_srelockisiasd[i]._cquired Opt_tabl_.entries[i]isiasd[i]._cquiredEVELSividers;

	PP_ASSE  esclkCdinfo vksusingle SMC GFXCLK struuint3a10(tmp_
	}

			
			hwmgr0ci
	tmp_
	}

	PP_(PP_ASSEisidinfrunning(uccesC)_?e0 :,a1;e3Dependency Table!",
tmp_
	}

	PPgr0,	}

"DPMris*ne &running right now, no ne	}
		PdissclkcDPMt_clock*curr_l0)bl
tnext,
			dpm_table,
			dep_phy_table);

	vega10_init_dpm_sta
___PDPM table */
	meThehwmlController)e
		PP_ASSE  esclkCthehwmlEprotecgr,
uuccesCLK
	tmp_
	}

	PP_ASSERT_  esclkCpower_containm8_tu stru);e3Dependency Table!",

tmp_
	}

	PPgr0C,	}

"k;
		}
		Pdissclkcpowerocontainm8_tint > }

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_  esclkCdidtbgonfigu stru);e3Dependency Table!",

tmp_
	}

	PPgr0C,	}

"k;
		}
		Pdissclkcdidt gonfigint > }

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_avfsm_tableu strucsi == )0 |Dependency Table!",

tmp_
	}

	PPgr0C,	}

"k;
		}
		PdissclkcAVFSint > }

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_stopC fiur,
		, SMC_D8
uFEATURES)0 |Dependency Table!",

tmp_
	}

	PPgr0C,	}

"k;
		}
		PstopcDPMt_c > }

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_  esclkCdeemcsleemcm_v	er_ewiforu stru);e3Dependency Table!",

tmp_
	}

	PPgr0C,	}

"k;
		}
		Pdissclkcdeem sleemt_c > }

	PP_tmp_
	}

	tat
	tmp_
	}

	PP_ASSERT_  esclkCulvu stru);e3Dependency Table!",

tmp_
	}

	PPgr0C,	}

"k;
		}
		Pdissclkculvt_c > }

	PP_tmp_
	}

	tat
	tmp_
	}

	PP__ASSERT_acg_  esclku stru);e3Dependency Table!",

tmp_
	}

	PPgr0C,	}

"k;
		}
		Pdissclkcacgt_c > }

	PP_tmp_
	}

	tattries[i]_DPM_LEVELSividers;

	PP_ASSEpower_off_asicusingle SMC GFXCLK struuint32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stati;

	
			hwat
	_S}

	PP_ASSERT_  esclkCdinfo vksu stru);e3Dependency Table!",

0PPgr
	}

	t,	}

"[  esclkCdinfo vks] k;
		}
		PdissclkcDPMt_clock)bltdsettiwIndh_wmrusgbitmap &= ~(WIndhMmrusLoySed)i etries[i]_DPM_LEVELSividersvoid PP_ASSEtiidt>in_UC>Dit);dexusingle SMC GFXCLK structuru);
	curonsructidx,	t);
	currvmructidx,ock*)(hwmgr->innslevc *)(hwmgr->innmruce
nt32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat	.
 */
sASSERT_d phm_ppt_v2_information *table_info =
	tat.t);
	currirn
	*table_info =
		d_informalo	(struct .ck_vo*current_vonextd_informalo	(struct .t phd UCLK i].< data->&lock d_informalo	(struct .t phd UCLK i].SOC_V >=->innslev)_dpm__*sructidx =	io r		breac;n__cl_.ent*table_info =
		da10_hwmglom = ruct .ck_vo*current_vonextd_informalom = ruct . find UCLK i].< data->&lock d_informalom = ruct . find UCLK i].SOC_V >=->innmlev)_dpm__*mructidx =	io r		breac;n__cl_.eELSividers;

	PP_ASSEs	(upower_profile_es[i](single SMC GFXCLK structursingle amdup		profiles**cquev	)
nt32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat	t);
	currs Mhzidxur ~0,	mructidxur ~0o etnextnt32_t dfind UCL !C_AMD_D8
uFORCEDuLEVEL_AUTO)
k > mem_max_clocatetPP_ASSEtiidt>in_UC>Dit);dexur,
		, &sructidx,	&mructidx,ock	*cquev	lominnslevc *cquev	lominnmruc);Letnexts Mhzidxu!r ~0ent_vonext!_settiyS>v Mry_e_in._ruct a10keyEdissclkdN
	ttDependency Table!",
lock k	smui_s/
s_msgncy (yr_with_ependencysL);__;uccess.smui++)L);__;PPSMC_MSG_SetStftMinGfxoltByIng_x,L);__;s Mhzidxk		breeeak;
		}
		Pser-stfr->in sruc ing_xt_clock k*curr_lax_clock ? .entnext> Mhzidxu!r ~0ent_vonext!_settiyS>v Mry_e_in.mruct a10keyEdissclkdN
	ttDependency Table!",
lock k	smui_s/
s_msgncy (yr_with_ependencysL);__;uccess.smui++)L);__;PPSMC_MSG_SetStftMinUoltByIng_x,L);__;m Mhzidxk		breeeak;
		}
		Pser-stfr->in mruc ing_xt_clock k*curr_lax_clock ? .entries[i].clk =ividers;

	PP_ASSEget/s Mhzoduct pp_hwmgr *hwmgr,
		e
nt32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat	.
 */
sASSERT_i	struct phm_ppt_vsructt_gra O n *table_info =
			(struct pat	.
 */
sASSERT_i	struct phm_ppt_vgolden_sructt_gra Olockn *tablegolden__info =
			(struct pat	;

	POC_Vi etSOC_V Oxts Mhzt_gralo find UCLK sructt_gralock_vo - 1].SOC_V -lockgolden_sructt_gralo find UCLKlock[golden_sructt_gralock_vo - 1].SOC_V) *lockn = /lockgolden_sructt_gralo find UCLKlock[golden_sructt_gralock_vo - 1].SOC_Vi etries[i]SOC_Vi ELSividers;

	PP_ASSEs	(us Mhzoduct pp_hwmgr *hwmgr,
		c *)(hwmgr-SOC_V)
nt32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat	.
 */
sASSERT_i	struct phm_ppt_vgolden_sructt_gra Olockn *tablegolden__info =
			(struct pat	ct pp_hwmgpower_es[i] *psci32_t soc_clock,power_es[i] *_clock,ps;entp&{
	uccess.*cquev	_ptd,
	next,s OC_NULL)
k > mem_max_clocatet_clock,psle	c_v	_phw__clock,power_es[i](&pt->hhhhhhhh)
	
	_clock,psp_per *hwmncy_on_scspm[_clock,psp_per *hwmncy_on_sc_ck_vo - 1].truct phm Olockgolden_sructt_gralo find UCLKlock[golden_sructt_gralock_vo - 1].SOC_V *lockSOC_V sOn = +lockgolden_sructt_gralo find UCLKlock[golden_sructt_gralock_vo - 1].SOC_Vi etnext_clock,psp_per *hwmncy_on_scspm_
[_clock,psp_per *hwmncy_on_sc_ck_vo - 1].truct phm otur				dep_phy_table);

	vega10
	stdriveLimit.8_gineCstru)L);_clock,psp_per *hwmncy_on_scspm_[_clock,psp_per *hwmncy_on_sc_ck_vo - 1].truct phm Olock				dep_phy_table);

	vega10
	stdriveLimit.8_gineCstrui etries[i].clk =ividers;

	PP_ASSEget/m Mhzoduct pp_hwmgr *hwmgr,
		e
nt32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat	.
 */
sASSERT_i	struct phm_ppt_vmructt_gra O n *table_info =
		m = ruct pat	.
 */
sASSERT_i	struct phm_ppt_vgolden_mructt_gra Olockn *tablegolden__info =
		m = ruct pat	;

	POC_Vi etSOC_V Oxtmructt_gralo find UCLKpm_
[mructt_gralock_vo - 1].SOC_V -lockgolden_mructt_gralo find UCLKpm_
[golden_mructt_gralock_vo - 1].SOC_V) *lockn = /lockgolden_mructt_gralo find UCLKpm_
[golden_mructt_gralock_vo - 1].SOC_Vi etries[i]SOC_Vi ELSividers;

	PP_ASSEs	(um Mhzoduct pp_hwmgr *hwmgr,
		c *)(hwmgr-SOC_V)
nt32_t soc_clock, uint8_t *cur _soc_did,
	turn   0 on success..
 */
stat	.
 */
sASSERT_i	struct phm_ppt_vgolden_mructt_gra Olockn *tablegolden__info =
		m = ruct pat	ct pp_hwmgpower_es[i]  *psci32_t soc_clock,power_es[i]  *_clock,ps;entp&{
	uccess.*cquev	_ptd,
	next,s OC_NULL)
k > mem_max_clocatet_clock,psle	c_v	_phw__clock,power_es[i](&pt->hhhhhhhh)
	
	_clock,psp_per *hwmncy_on_scspm[_clock,psp_per *hwmncy_on_sc_ck_vo - 1].m = (stru_ilockgolden_mructt_gralo find UCLKpm_
[golden_mructt_gralock_vo - 1].SOC_V *lockSOC_V sOn = +lockgolden_mructt_gralo find UCLKpm_
[golden_mructt_gralock_vo - 1].SOC_Vi etnext_clock,psp_per *hwmncy_on_scspm_
[_clock,psp_per *hwmncy_on_sc_ck_vo - 1].m = (stru_otur				dep_phy_table);

	vega10
	stdriveLimit.soc_leCstru)L);_clock,psp_per *hwmncy_on_scspm_[_clock,psp_per *hwmncy_on_sc_ck_vo - 1].m = (stru_ilock				dep_phy_table);

	vega10
	stdriveLimit.soc_leCstrui etries[i].clk =ividersconso single SMC G	de_funcd,
	turn   0 _funcs 
	{
	..
 */
st);i	PP_ASSERT_   0 _.
 */
st);i	,
	..
 */
sttiiiPP_ASSERT_   0 _.
 */
sttiii,
	.asicEs	(upPP_ASSERT_s	(up_asicfo vk,
	.dynamic_es[i].wmnagem8_tm_tablePP_ASSERT_n_sclkCdinfo vks,
	.dynamic_es[i].wmnagem8_tmdissclkcP_ASSERT_  esclkCdinfo vks,
	.get/nui_ofup		opulv_8_t)(de =tur	PP_ASSEget/nuiber_of,powerf_tabtpulv_8_t)(de,
	.get/power_es[i]_sizen= PP_ASSEget/power_es[i]_size,
	.get/p		opulv_8_t)yn= PP_ASSEget/p		opulv_8_t)y,
	.patch bootmes[i] = PP_ASSEpatch bootmes[i],
	.apply_es[i]_adjuv	_rulde =_ASSERT_apply_es[i]_adjuv	_rulde,
	.power_es[i]_si	PP_ASSERT_set/power_es[i]_o vks,
	.get/sruc P_ASSERT_ infget/sruc,
	.get/mruc P_ASSERT_ infget/mruc,
	.notify (yr_d ef_tabgonfig_afncy,ps_adjuv	m8_t =tur	PP_ASSEnotify (yr_d ef_tabgonfig_afncy,ps_adjuv	m8_t,
	.t*hckC finln_sc P_ASSERT_ inft*hckC finln_sc,
	.get/temperaS>yi = PP_ASSEthehwmlE		(utemperaS>yi,
	.stopCthehwmlEcontroller = PP_ASSEthehwmlEstopCthehwmlEcontroller,
	.get/fan.speed_le_mem	PP_ASSEtan.ctrlEget/fan.speed_le_m,
	.get/fan.speed_perc8_tem	PP_ASSEtan.ctrlEget/fan.speed_perc8_t,
	.set/fan.speed_perc8_tem	PP_ASSEtan.ctrlEset/fan.speed_perc8_t,
	.reset/fan.speed_to_dtte(&( =tur	PP_ASSEtan.ctrlEreset/fan.speed_to_dtte(&(,
	.get/fan.speed_rpmem	PP_ASSEtan.ctrlEget/fan.speed_rpm,
	.set/fan.speed_rpmem	PP_ASSEtan.ctrlEset/fan.speed_rpm,
	.un);i	ializeCthehwmlEcontroller =tur	PP_ASSEthehwmlEctrlEun);i	ializeCthehwmlEcontroller,
	.set/fan.control_modnPP_ASSERT_set/fan.control_modn,
	.get/fan.control_modnPP_ASSERT_get/fan.control_modn,
	.read_s/
sor = PP_ASSEread_s/
sor,
	.get/daocpower_d UCLPP_ASSERT_get/daocpower_d UCL,
	.get/t phm_by_type_with_eIndncyPP_ASSERT_get/t phm_by_type_with_eIndncy,
	.get/t phm_by_type_with_ce_tabl P_ASSERT_get/t phm_by_type_with_ce_tabl,
	.set/wIndhwmrusg *h_(strusgrangesPP_ASSERT_set/wIndhwmrusg *h_(strusgranges,
	.d ef_tabg phm_ce_table*cquev	cP_ASSERT_  ef_tabg phm_ce_table*cquev	,
	.t*hckCg phm_ln_scem	PP_ASSEt*hckCg phm_ln_sc,
	.pr;

	g phm_ln_scl = PP_ASSEpr;

	g phm_ln_scl,
	.d ef_tabgonfig_ghangedcP_ASSERT_  ef_tabgonfigurmgr,
bghanged.t_vk,
	.powerg[i]Cuvd = PP_ASSEpower_g[i]Cuvd,
	.powerg[i]Cvce = PP_ASSEpower_g[i]Cvce,lo.check0es[i]s_equalPP_ASSERT_check0es[i]s_equal,lo.check0eyriasd[i]._cquired. *h_  ef_tabgonfigurmgr,
 =tur	PP_ASSEcheck0eyriasd[i]._cquired. *h_  ef_tabgonfigurmgr,
,
	.power_off_asic = PP_ASSEpower_off_asic,
	.d esclkC(yrifirmhhhhEctf = PP_ASSEthehwmlEd esclkCalert,
	.set/power_profile_es[i]PP_ASSERT_set/power_profile_es[i],
	.get/sruczod P_ASSERT_get/sruczod,
	.set/sruczod P_ASSERT_set/sruczod,
	.get/m Mhzod P_ASSERT_get/mruczod,
	.set/m Mhzod P_ASSERT_set/mruczod,
	.avfsmcontrolPP_ASSERT_avfsm_table,
}ate;

	PP_ASSE   0 _);i	uct pp_hwmgr *hwmgr,
		e
nt3			dep_ G	de_funcd= &,
	turn   0 _funcs;t3			dep_mem_clo_funcd= &,
	turnmem_clo_funcs;t3wmgPP_ASSEthehwmlE);i	ializeu stru);e3ries[i].clk 