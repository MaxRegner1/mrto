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
#include <linux/firmware.h>
#include <drm/drmP.h>
#include "amdgpu.h"
#include "amdgpu_gfx.h"
#include "soc15.h"
#include "soc15d.h"

#include "vega10/soc15ip.h"
#include "vega10/GC/gc_9_0_offset.h"
#include "vega10/GC/gc_9_0_sh_mask.h"
#include "vega10/vega10_enum.h"
#include "vega10/HDP/hdp_4_0_offset.h"

#include "soc15_common.h"
#include "clearstate_gfx9.h"
#include "v9_structs.h"

#define GFX9_NUM_GFX_RINGS     1
#define GFX9_MEC_HPD_SIZE 2048
#define RLCG_UCODE_LOADING_START_ADDRESS 0x00002000L
#define RLC_SAVE_RESTORE_ADDR_STARTING_OFFSET 0x00000000L
#define GFX9_RLC_FORMAT_DIRECT_REG_LIST_LENGTH 34

#define mmPWR_MISC_CNTL_STATUS					0x0183
#define mmPWR_MISC_CNTL_STATUS_BASE_IDX				0
#define PWR_MISC_CNTL_STATUS__PWR_GFX_RLC_CGPG_EN__SHIFT	0x0
#define PWR_MISC_CNTL_STATUS__PWR_GFXOFF_STATUS__SHIFT		0x1
#define PWR_MISC_CNTL_STATUS__PWR_GFX_RLC_CGPG_EN_MASK		0x00000001L
#define PWR_MISC_CNTL_STATUS__PWR_GFXOFF_STATUS_MASK		0x00000006L

MODULE_FIRMWARE("amdgpu/vega10_ce.bin");
MODULE_FIRMWARE("amdgpu/vega10_pfp.bin");
MODULE_FIRMWARE("amdgpu/vega10_me.bin");
MODULE_FIRMWARE("amdgpu/vega10_mec.bin");
MODULE_FIRMWARE("amdgpu/vega10_mec2.bin");
MODULE_FIRMWARE("amdgpu/vega10_rlc.bin");

MODULE_FIRMWARE("amdgpu/raven_ce.bin");
MODULE_FIRMWARE("amdgpu/raven_pfp.bin");
MODULE_FIRMWARE("amdgpu/raven_me.bin");
MODULE_FIRMWARE("amdgpu/raven_mec.bin");
MODULE_FIRMWARE("amdgpu/raven_mec2.bin");
MODULE_FIRMWARE("amdgpu/raven_rlc.bin");

static const struct amdgpu_gds_reg_offset amdgpu_gds_reg_offset[] =
{
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID0_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID0_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID0), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID0)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID1_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID1_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID1), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID1)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID2_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID2_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID2), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID2)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID3_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID3_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID3), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID3)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID4_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID4_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID4), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID4)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID5_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID5_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID5), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID5)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID6_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID6_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID6), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID6)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID7_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID7_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID7), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID7)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID8_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID8_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID8), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID8)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID9_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID9_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID9), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID9)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID10_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID10_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID10), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID10)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID11_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID11_SIZE),
	       	SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID11), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID11)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID12_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID12_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID12), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID12)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID13_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID13_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID13), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID13)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID14_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID14_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID14), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID14)},
	{SOC15_REG_OFFSET(GC, 0, mmGDS_VMID15_BASE), SOC15_REG_OFFSET(GC, 0, mmGDS_VMID15_SIZE),
		SOC15_REG_OFFSET(GC, 0, mmGDS_GWS_VMID15), SOC15_REG_OFFSET(GC, 0, mmGDS_OA_VMID15)}
};

static const u32 golden_settings_gc_9_0[] =
{
	SOC15_REG_OFFSET(GC, 0, mmCPC_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmCPF_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmCPG_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmDB_DEBUG2), 0xf00fffff, 0x00000420,
	SOC15_REG_OFFSET(GC, 0, mmGB_GPU_ID), 0x0000000f, 0x00000000,
	SOC15_REG_OFFSET(GC, 0, mmIA_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmPA_SC_BINNER_EVENT_CNTL_3), 0x00000003, 0x82400024,
	SOC15_REG_OFFSET(GC, 0, mmPA_SC_ENHANCE), 0x3fffffff, 0x00000001,
	SOC15_REG_OFFSET(GC, 0, mmPA_SC_LINE_STIPPLE_STATE), 0x0000ff0f, 0x00000000,
	SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_UTCL1_CNTL_0), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_UTCL1_CNTL_1), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_UTCL1_CNTL_2), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmRLC_PREWALKER_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmRLC_SPM_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmSH_MEM_CONFIG), 0x00001000, 0x00001000,
	SOC15_REG_OFFSET(GC, 0, mmSPI_CONFIG_CNTL_1), 0x0000000f, 0x01000107,
	SOC15_REG_OFFSET(GC, 0, mmSQC_CONFIG), 0x03000000, 0x020a2000,
	SOC15_REG_OFFSET(GC, 0, mmTA_CNTL_AUX), 0xfffffeef, 0x010b0000,
	SOC15_REG_OFFSET(GC, 0, mmTCP_CHAN_STEER_HI), 0xffffffff, 0x4a2c0e68,
	SOC15_REG_OFFSET(GC, 0, mmTCP_CHAN_STEER_LO), 0xffffffff, 0xb5d3f197,
	SOC15_REG_OFFSET(GC, 0, mmVGT_CACHE_INVALIDATION), 0x3fff3af3, 0x19200000,
	SOC15_REG_OFFSET(GC, 0, mmVGT_GS_MAX_WAVE_ID), 0x00000fff, 0x000003ff,
	SOC15_REG_OFFSET(GC, 0, mmWD_UTCL1_CNTL), 0x08000000, 0x08000080
};

static const u32 golden_settings_gc_9_0_vg10[] =
{
	SOC15_REG_OFFSET(GC, 0, mmCB_HW_CONTROL), 0x0000f000, 0x00012107,
	SOC15_REG_OFFSET(GC, 0, mmCB_HW_CONTROL_3), 0x30000000, 0x10000000,
	SOC15_REG_OFFSET(GC, 0, mmGB_ADDR_CONFIG), 0xffff77ff, 0x2a114042,
	SOC15_REG_OFFSET(GC, 0, mmGB_ADDR_CONFIG_READ), 0xffff77ff, 0x2a114042,
	SOC15_REG_OFFSET(GC, 0, mmPA_SC_ENHANCE_1), 0x00008000, 0x00048000,
	SOC15_REG_OFFSET(GC, 0, mmRMI_UTCL1_CNTL2), 0x00030000, 0x00020000,
	SOC15_REG_OFFSET(GC, 0, mmTD_CNTL), 0x00001800, 0x00000800
};

static const u32 golden_settings_gc_9_1[] =
{
	SOC15_REG_OFFSET(GC, 0, mmCB_HW_CONTROL), 0xfffdf3cf, 0x00014104,
	SOC15_REG_OFFSET(GC, 0, mmCPC_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmCPF_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmCPG_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmDB_DEBUG2), 0xf00fffff, 0x00000420,
	SOC15_REG_OFFSET(GC, 0, mmGB_GPU_ID), 0x0000000f, 0x00000000,
	SOC15_REG_OFFSET(GC, 0, mmIA_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmPA_SC_BINNER_EVENT_CNTL_3), 0x00000003, 0x82400024,
	SOC15_REG_OFFSET(GC, 0, mmPA_SC_ENHANCE), 0x3fffffff, 0x00000001,
	SOC15_REG_OFFSET(GC, 0, mmPA_SC_LINE_STIPPLE_STATE), 0x0000ff0f, 0x00000000,
	SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_UTCL1_CNTL_0), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_UTCL1_CNTL_1), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_UTCL1_CNTL_2), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmRLC_PREWALKER_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmRLC_SPM_UTCL1_CNTL), 0x08000000, 0x08000080,
	SOC15_REG_OFFSET(GC, 0, mmTA_CNTL_AUX), 0xfffffeef, 0x010b0000,
	SOC15_REG_OFFSET(GC, 0, mmTCP_CHAN_STEER_HI), 0xffffffff, 0x00000000,
	SOC15_REG_OFFSET(GC, 0, mmTCP_CHAN_STEER_LO), 0xffffffff, 0x00003120,
	SOC15_REG_OFFSET(GC, 0, mmVGT_CACHE_INVALIDATION), 0x3fff3af3, 0x19200000,
	SOC15_REG_OFFSET(GC, 0, mmVGT_GS_MAX_WAVE_ID), 0x00000fff, 0x000000ff,
	SOC15_REG_OFFSET(GC, 0, mmWD_UTCL1_CNTL), 0x08000000, 0x08000080
};

static const u32 golden_settings_gc_9_1_rv1[] =
{
	SOC15_REG_OFFSET(GC, 0, mmCB_HW_CONTROL_3), 0x30000000, 0x10000000,
	SOC15_REG_OFFSET(GC, 0, mmGB_ADDR_CONFIG), 0xffff77ff, 0x24000042,
	SOC15_REG_OFFSET(GC, 0, mmGB_ADDR_CONFIG_READ), 0xffff77ff, 0x24000042,
	SOC15_REG_OFFSET(GC, 0, mmPA_SC_ENHANCE_1), 0xffffffff, 0x04048000,
	SOC15_REG_OFFSET(GC, 0, mmPA_SC_MODE_CNTL_1), 0x06000000, 0x06000000,
	SOC15_REG_OFFSET(GC, 0, mmRMI_UTCL1_CNTL2), 0x00030000, 0x00020000,
	SOC15_REG_OFFSET(GC, 0, mmTD_CNTL), 0x01bd9f33, 0x00000800
};

#define VEGA10_GB_ADDR_CONFIG_GOLDEN 0x2a114042
#define RAVEN_GB_ADDR_CONFIG_GOLDEN 0x24000042

static void gfx_v9_0_set_ring_funcs(struct amdgpu_device *adev);
static void gfx_v9_0_set_irq_funcs(struct amdgpu_device *adev);
static void gfx_v9_0_set_gds_init(struct amdgpu_device *adev);
static void gfx_v9_0_set_rlc_funcs(struct amdgpu_device *adev);
static int gfx_v9_0_get_cu_info(struct amdgpu_device *adev,
                                 struct amdgpu_cu_info *cu_info);
static uint64_t gfx_v9_0_get_gpu_clock_counter(struct amdgpu_device *adev);
static void gfx_v9_0_select_se_sh(struct amdgpu_device *adev, u32 se_num, u32 sh_num, u32 instance);
static void gfx_v9_0_ring_emit_de_meta(struct amdgpu_ring *ring);

static void gfx_v9_0_init_golden_registers(struct amdgpu_device *adev)
{
	switch (adev->asic_type) {
	case CHIP_VEGA10:
		amdgpu_program_register_sequence(adev,
						 golden_settings_gc_9_0,
						 (const u32)ARRAY_SIZE(golden_settings_gc_9_0));
		amdgpu_program_register_sequence(adev,
						 golden_settings_gc_9_0_vg10,
						 (const u32)ARRAY_SIZE(golden_settings_gc_9_0_vg10));
		break;
	case CHIP_RAVEN:
		amdgpu_program_register_sequence(adev,
						 golden_settings_gc_9_1,
						 (const u32)ARRAY_SIZE(golden_settings_gc_9_1));
		amdgpu_program_register_sequence(adev,
						 golden_settings_gc_9_1_rv1,
						 (const u32)ARRAY_SIZE(golden_settings_gc_9_1_rv1));
		break;
	default:
		break;
	}
}

static void gfx_v9_0_scratch_init(struct amdgpu_device *adev)
{
	adev->gfx.scratch.num_reg = 8;
	adev->gfx.scratch.reg_base = SOC15_REG_OFFSET(GC, 0, mmSCRATCH_REG0);
	adev->gfx.scratch.free_mask = (1u << adev->gfx.scratch.num_reg) - 1;
}

static void gfx_v9_0_write_data_to_reg(struct amdgpu_ring *ring, int eng_sel,
				       bool wc, uint32_t reg, uint32_t val)
{
	amdgpu_ring_write(ring, PACKET3(PACKET3_WRITE_DATA, 3));
	amdgpu_ring_write(ring, WRITE_DATA_ENGINE_SEL(eng_sel) |
				WRITE_DATA_DST_SEL(0) |
				(wc ? WR_CONFIRM : 0));
	amdgpu_ring_write(ring, reg);
	amdgpu_ring_write(ring, 0);
	amdgpu_ring_write(ring, val);
}

static void gfx_v9_0_wait_reg_mem(struct amdgpu_ring *ring, int eng_sel,
				  int mem_space, int opt, uint32_t addr0,
				  uint32_t addr1, uint32_t ref, uint32_t mask,
				  uint32_t inv)
{
	amdgpu_ring_write(ring, PACKET3(PACKET3_WAIT_REG_MEM, 5));
	amdgpu_ring_write(ring,
				 /* memory (1) or register (0) */
				 (WAIT_REG_MEM_MEM_SPACE(mem_space) |
				 WAIT_REG_MEM_OPERATION(opt) | /* wait */
				 WAIT_REG_MEM_FUNCTION(3) |  /* equal */
				 WAIT_REG_MEM_ENGINE(eng_sel)));

	if (mem_space)
		BUG_ON(addr0 & 0x3); /* Dword align */
	amdgpu_ring_write(ring, addr0);
	amdgpu_ring_write(ring, addr1);
	amdgpu_ring_write(ring, ref);
	amdgpu_ring_write(ring, mask);
	amdgpu_ring_write(ring, inv); /* poll interval */
}

static int gfx_v9_0_ring_test_ring(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;
	uint32_t scratch;
	uint32_t tmp = 0;
	unsigned i;
	int r;

	r = amdgpu_gfx_scratch_get(adev, &scratch);
	if (r) {
		DRM_ERROR("amdgpu: cp failed to get scratch reg (%d).\n", r);
		return r;
	}
	WREG32(scratch, 0xCAFEDEAD);
	r = amdgpu_ring_alloc(ring, 3);
	if (r) {
		DRM_ERROR("amdgpu: cp failed to lock ring %d (%d).\n",
			  ring->idx, r);
		amdgpu_gfx_scratch_free(adev, scratch);
		return r;
	}
	amdgpu_ring_write(ring, PACKET3(PACKET3_SET_UCONFIG_REG, 1));
	amdgpu_ring_write(ring, (scratch - PACKET3_SET_UCONFIG_REG_START));
	amdgpu_ring_write(ring, 0xDEADBEEF);
	amdgpu_ring_commit(ring);

	for (i = 0; i < adev->usec_timeout; i++) {
		tmp = RREG32(scratch);
		if (tmp == 0xDEADBEEF)
			break;
		DRM_UDELAY(1);
	}
	if (i < adev->usec_timeout) {
		DRM_INFO("ring test on %d succeeded in %d usecs\n",
			 ring->idx, i);
	} else {
		DRM_ERROR("amdgpu: ring %d test failed (scratch(0x%04X)=0x%08X)\n",
			  ring->idx, scratch, tmp);
		r = -EINVAL;
	}
	amdgpu_gfx_scratch_free(adev, scratch);
	return r;
}

static int gfx_v9_0_ring_test_ib(struct amdgpu_ring *ring, long timeout)
{
        struct amdgpu_device *adev = ring->adev;
        struct amdgpu_ib ib;
        struct dma_fence *f = NULL;
        uint32_t scratch;
        uint32_t tmp = 0;
        long r;

        r = amdgpu_gfx_scratch_get(adev, &scratch);
        if (r) {
                DRM_ERROR("amdgpu: failed to get scratch reg (%ld).\n", r);
                return r;
        }
        WREG32(scratch, 0xCAFEDEAD);
        memset(&ib, 0, sizeof(ib));
        r = amdgpu_ib_get(adev, NULL, 256, &ib);
        if (r) {
                DRM_ERROR("amdgpu: failed to get ib (%ld).\n", r);
                goto err1;
        }
        ib.ptr[0] = PACKET3(PACKET3_SET_UCONFIG_REG, 1);
        ib.ptr[1] = ((scratch - PACKET3_SET_UCONFIG_REG_START));
        ib.ptr[2] = 0xDEADBEEF;
        ib.length_dw = 3;

        r = amdgpu_ib_schedule(ring, 1, &ib, NULL, &f);
        if (r)
                goto err2;

        r = dma_fence_wait_timeout(f, false, timeout);
        if (r == 0) {
                DRM_ERROR("amdgpu: IB test timed out.\n");
                r = -ETIMEDOUT;
                goto err2;
        } else if (r < 0) {
                DRM_ERROR("amdgpu: fence wait failed (%ld).\n", r);
                goto err2;
        }
        tmp = RREG32(scratch);
        if (tmp == 0xDEADBEEF) {
                DRM_INFO("ib test on ring %d succeeded\n", ring->idx);
                r = 0;
        } else {
                DRM_ERROR("amdgpu: ib test failed (scratch(0x%04X)=0x%08X)\n",
                          scratch, tmp);
                r = -EINVAL;
        }
err2:
        amdgpu_ib_free(adev, &ib, NULL);
        dma_fence_put(f);
err1:
        amdgpu_gfx_scratch_free(adev, scratch);
        return r;
}

static int gfx_v9_0_init_microcode(struct amdgpu_device *adev)
{
	const char *chip_name;
	char fw_name[30];
	int err;
	struct amdgpu_firmware_info *info = NULL;
	const struct common_firmware_header *header = NULL;
	const struct gfx_firmware_header_v1_0 *cp_hdr;
	const struct rlc_firmware_header_v2_0 *rlc_hdr;
	unsigned int *tmp = NULL;
	unsigned int i = 0;

	DRM_DEBUG("\n");

	switch (adev->asic_type) {
	case CHIP_VEGA10:
		chip_name = "vega10";
		break;
	case CHIP_RAVEN:
		chip_name = "raven";
		break;
	default:
		BUG();
	}

	snprintf(fw_name, sizeof(fw_name), "amdgpu/%s_pfp.bin", chip_name);
	err = request_firmware(&adev->gfx.pfp_fw, fw_name, adev->dev);
	if (err)
		goto out;
	err = amdgpu_ucode_validate(adev->gfx.pfp_fw);
	if (err)
		goto out;
	cp_hdr = (const struct gfx_firmware_header_v1_0 *)adev->gfx.pfp_fw->data;
	adev->gfx.pfp_fw_version = le32_to_cpu(cp_hdr->header.ucode_version);
	adev->gfx.pfp_feature_version = le32_to_cpu(cp_hdr->ucode_feature_version);

	snprintf(fw_name, sizeof(fw_name), "amdgpu/%s_me.bin", chip_name);
	err = request_firmware(&adev->gfx.me_fw, fw_name, adev->dev);
	if (err)
		goto out;
	err = amdgpu_ucode_validate(adev->gfx.me_fw);
	if (err)
		goto out;
	cp_hdr = (const struct gfx_firmware_header_v1_0 *)adev->gfx.me_fw->data;
	adev->gfx.me_fw_version = le32_to_cpu(cp_hdr->header.ucode_version);
	adev->gfx.me_feature_version = le32_to_cpu(cp_hdr->ucode_feature_version);

	snprintf(fw_name, sizeof(fw_name), "amdgpu/%s_ce.bin", chip_name);
	err = request_firmware(&adev->gfx.ce_fw, fw_name, adev->dev);
	if (err)
		goto out;
	err = amdgpu_ucode_validate(adev->gfx.ce_fw);
	if (err)
		goto out;
	cp_hdr = (const struct gfx_firmware_header_v1_0 *)adev->gfx.ce_fw->data;
	adev->gfx.ce_fw_version = le32_to_cpu(cp_hdr->header.ucode_version);
	adev->gfx.ce_feature_version = le32_to_cpu(cp_hdr->ucode_feature_version);

	snprintf(fw_name, sizeof(fw_name), "amdgpu/%s_rlc.bin", chip_name);
	err = request_firmware(&adev->gfx.rlc_fw, fw_name, adev->dev);
	if (err)
		goto out;
	err = amdgpu_ucode_validate(adev->gfx.rlc_fw);
	rlc_hdr = (const struct rlc_firmware_header_v2_0 *)adev->gfx.rlc_fw->data;
	adev->gfx.rlc_fw_version = le32_to_cpu(rlc_hdr->header.ucode_version);
	adev->gfx.rlc_feature_version = le32_to_cpu(rlc_hdr->ucode_feature_version);
	adev->gfx.rlc.save_and_restore_offset =
			le32_to_cpu(rlc_hdr->save_and_restore_offset);
	adev->gfx.rlc.clear_state_descriptor_offset =
			le32_to_cpu(rlc_hdr->clear_state_descriptor_offset);
	adev->gfx.rlc.avail_scratch_ram_locations =
			le32_to_cpu(rlc_hdr->avail_scratch_ram_locations);
	adev->gfx.rlc.reg_restore_list_size =
			le32_to_cpu(rlc_hdr->reg_restore_list_size);
	adev->gfx.rlc.reg_list_format_start =
			le32_to_cpu(rlc_hdr->reg_list_format_start);
	adev->gfx.rlc.reg_list_format_separate_start =
			le32_to_cpu(rlc_hdr->reg_list_format_separate_start);
	adev->gfx.rlc.starting_offsets_start =
			le32_to_cpu(rlc_hdr->starting_offsets_start);
	adev->gfx.rlc.reg_list_format_size_bytes =
			le32_to_cpu(rlc_hdr->reg_list_format_size_bytes);
	adev->gfx.rlc.reg_list_size_bytes =
			le32_to_cpu(rlc_hdr->reg_list_size_bytes);
	adev->gfx.rlc.register_list_format =
			kmalloc(adev->gfx.rlc.reg_list_format_size_bytes +
				adev->gfx.rlc.reg_list_size_bytes, GFP_KERNEL);
	if (!adev->gfx.rlc.register_list_format) {
		err = -ENOMEM;
		goto out;
	}

	tmp = (unsigned int *)((uintptr_t)rlc_hdr +
			le32_to_cpu(rlc_hdr->reg_list_format_array_offset_bytes));
	for (i = 0 ; i < (rlc_hdr->reg_list_format_size_bytes >> 2); i++)
		adev->gfx.rlc.register_list_format[i] =	le32_to_cpu(tmp[i]);

	adev->gfx.rlc.register_restore = adev->gfx.rlc.register_list_format + i;

	tmp = (unsigned int *)((uintptr_t)rlc_hdr +
			le32_to_cpu(rlc_hdr->reg_list_array_offset_bytes));
	for (i = 0 ; i < (rlc_hdr->reg_list_size_bytes >> 2); i++)
		adev->gfx.rlc.register_restore[i] = le32_to_cpu(tmp[i]);

	snprintf(fw_name, sizeof(fw_name), "amdgpu/%s_mec.bin", chip_name);
	err = request_firmware(&adev->gfx.mec_fw, fw_name, adev->dev);
	if (err)
		goto out;
	err = amdgpu_ucode_validate(adev->gfx.mec_fw);
	if (err)
		goto out;
	cp_hdr = (const struct gfx_firmware_header_v1_0 *)adev->gfx.mec_fw->data;
	adev->gfx.mec_fw_version = le32_to_cpu(cp_hdr->header.ucode_version);
	adev->gfx.mec_feature_version = le32_to_cpu(cp_hdr->ucode_feature_version);


	snprintf(fw_name, sizeof(fw_name), "amdgpu/%s_mec2.bin", chip_name);
	err = request_firmware(&adev->gfx.mec2_fw, fw_name, adev->dev);
	if (!err) {
		err = amdgpu_ucode_validate(adev->gfx.mec2_fw);
		if (err)
			goto out;
		cp_hdr = (const struct gfx_firmware_header_v1_0 *)
		adev->gfx.mec2_fw->data;
		adev->gfx.mec2_fw_version =
		le32_to_cpu(cp_hdr->header.ucode_version);
		adev->gfx.mec2_feature_version =
		le32_to_cpu(cp_hdr->ucode_feature_version);
	} else {
		err = 0;
		adev->gfx.mec2_fw = NULL;
	}

	if (adev->firmware.load_type == AMDGPU_FW_LOAD_PSP) {
		info = &adev->firmware.ucode[AMDGPU_UCODE_ID_CP_PFP];
		info->ucode_id = AMDGPU_UCODE_ID_CP_PFP;
		info->fw = adev->gfx.pfp_fw;
		header = (const struct common_firmware_header *)info->fw->data;
		adev->firmware.fw_size +=
			ALIGN(le32_to_cpu(header->ucode_size_bytes), PAGE_SIZE);

		info = &adev->firmware.ucode[AMDGPU_UCODE_ID_CP_ME];
		info->ucode_id = AMDGPU_UCODE_ID_CP_ME;
		info->fw = adev->gfx.me_fw;
		header = (const struct common_firmware_header *)info->fw->data;
		adev->firmware.fw_size +=
			ALIGN(le32_to_cpu(header->ucode_size_bytes), PAGE_SIZE);

		info = &adev->firmware.ucode[AMDGPU_UCODE_ID_CP_CE];
		info->ucode_id = AMDGPU_UCODE_ID_CP_CE;
		info->fw = adev->gfx.ce_fw;
		header = (const struct common_firmware_header *)info->fw->data;
		adev->firmware.fw_size +=
			ALIGN(le32_to_cpu(header->ucode_size_bytes), PAGE_SIZE);

		info = &adev->firmware.ucode[AMDGPU_UCODE_ID_RLC_G];
		info->ucode_id = AMDGPU_UCODE_ID_RLC_G;
		info->fw = adev->gfx.rlc_fw;
		header = (const struct common_firmware_header *)info->fw->data;
		adev->firmware.fw_size +=
			ALIGN(le32_to_cpu(header->ucode_size_bytes), PAGE_SIZE);

		info = &adev->firmware.ucode[AMDGPU_UCODE_ID_CP_MEC1];
		info->ucode_id = AMDGPU_UCODE_ID_CP_MEC1;
		info->fw = adev->gfx.mec_fw;
		header = (const struct common_firmware_header *)info->fw->data;
		cp_hdr = (const struct gfx_firmware_header_v1_0 *)info->fw->data;
		adev->firmware.fw_size +=
			ALIGN(le32_to_cpu(header->ucode_size_bytes) - le32_to_cpu(cp_hdr->jt_size) * 4, PAGE_SIZE);

		info = &adev->firmware.ucode[AMDGPU_UCODE_ID_CP_MEC1_JT];
		info->ucode_id = AMDGPU_UCODE_ID_CP_MEC1_JT;
		info->fw = adev->gfx.mec_fw;
		adev->firmware.fw_size +=
			ALIGN(le32_to_cpu(cp_hdr->jt_size) * 4, PAGE_SIZE);

		if (adev->gfx.mec2_fw) {
			info = &adev->firmware.ucode[AMDGPU_UCODE_ID_CP_MEC2];
			info->ucode_id = AMDGPU_UCODE_ID_CP_MEC2;
			info->fw = adev->gfx.mec2_fw;
			header = (const struct common_firmware_header *)info->fw->data;
			cp_hdr = (const struct gfx_firmware_header_v1_0 *)info->fw->data;
			adev->firmware.fw_size +=
				ALIGN(le32_to_cpu(header->ucode_size_bytes) - le32_to_cpu(cp_hdr->jt_size) * 4, PAGE_SIZE);
			info = &adev->firmware.ucode[AMDGPU_UCODE_ID_CP_MEC2_JT];
			info->ucode_id = AMDGPU_UCODE_ID_CP_MEC2_JT;
			info->fw = adev->gfx.mec2_fw;
			adev->firmware.fw_size +=
				ALIGN(le32_to_cpu(cp_hdr->jt_size) * 4, PAGE_SIZE);
		}

	}

out:
	if (err) {
		dev_err(adev->dev,
			"gfx9: Failed to load firmware \"%s\"\n",
			fw_name);
		release_firmware(adev->gfx.pfp_fw);
		adev->gfx.pfp_fw = NULL;
		release_firmware(adev->gfx.me_fw);
		adev->gfx.me_fw = NULL;
		release_firmware(adev->gfx.ce_fw);
		adev->gfx.ce_fw = NULL;
		release_firmware(adev->gfx.rlc_fw);
		adev->gfx.rlc_fw = NULL;
		release_firmware(adev->gfx.mec_fw);
		adev->gfx.mec_fw = NULL;
		release_firmware(adev->gfx.mec2_fw);
		adev->gfx.mec2_fw = NULL;
	}
	return err;
}

static u32 gfx_v9_0_get_csb_size(struct amdgpu_device *adev)
{
	u32 count = 0;
	const struct cs_section_def *sect = NULL;
	const struct cs_extent_def *ext = NULL;

	/* begin clear state */
	count += 2;
	/* context control state */
	count += 3;

	for (sect = gfx9_cs_data; sect->section != NULL; ++sect) {
		for (ext = sect->section; ext->extent != NULL; ++ext) {
			if (sect->id == SECT_CONTEXT)
				count += 2 + ext->reg_count;
			else
				return 0;
		}
	}

	/* end clear state */
	count += 2;
	/* clear state */
	count += 2;

	return count;
}

static void gfx_v9_0_get_csb_buffer(struct amdgpu_device *adev,
				    volatile u32 *buffer)
{
	u32 count = 0, i;
	const struct cs_section_def *sect = NULL;
	const struct cs_extent_def *ext = NULL;

	if (adev->gfx.rlc.cs_data == NULL)
		return;
	if (buffer == NULL)
		return;

	buffer[count++] = cpu_to_le32(PACKET3(PACKET3_PREAMBLE_CNTL, 0));
	buffer[count++] = cpu_to_le32(PACKET3_PREAMBLE_BEGIN_CLEAR_STATE);

	buffer[count++] = cpu_to_le32(PACKET3(PACKET3_CONTEXT_CONTROL, 1));
	buffer[count++] = cpu_to_le32(0x80000000);
	buffer[count++] = cpu_to_le32(0x80000000);

	for (sect = adev->gfx.rlc.cs_data; sect->section != NULL; ++sect) {
		for (ext = sect->section; ext->extent != NULL; ++ext) {
			if (sect->id == SECT_CONTEXT) {
				buffer[count++] =
					cpu_to_le32(PACKET3(PACKET3_SET_CONTEXT_REG, ext->reg_count));
				buffer[count++] = cpu_to_le32(ext->reg_index -
						PACKET3_SET_CONTEXT_REG_START);
				for (i = 0; i < ext->reg_count; i++)
					buffer[count++] = cpu_to_le32(ext->extent[i]);
			} else {
				return;
			}
		}
	}

	buffer[count++] = cpu_to_le32(PACKET3(PACKET3_PREAMBLE_CNTL, 0));
	buffer[count++] = cpu_to_le32(PACKET3_PREAMBLE_END_CLEAR_STATE);

	buffer[count++] = cpu_to_le32(PACKET3(PACKET3_CLEAR_STATE, 0));
	buffer[count++] = cpu_to_le32(0);
}

static void gfx_v9_0_init_lbpw(struct amdgpu_device *adev)
{
	uint32_t data;

	/* set mmRLC_LB_THR_CONFIG_1/2/3/4 */
	WREG32_SOC15(GC, 0, mmRLC_LB_THR_CONFIG_1, 0x0000007F);
	WREG32_SOC15(GC, 0, mmRLC_LB_THR_CONFIG_2, 0x0333A5A7);
	WREG32_SOC15(GC, 0, mmRLC_LB_THR_CONFIG_3, 0x00000077);
	WREG32_SOC15(GC, 0, mmRLC_LB_THR_CONFIG_4, (0x30 | 0x40 << 8 | 0x02FA << 16));

	/* set mmRLC_LB_CNTR_INIT = 0x0000_0000 */
	WREG32_SOC15(GC, 0, mmRLC_LB_CNTR_INIT, 0x00000000);

	/* set mmRLC_LB_CNTR_MAX = 0x0000_0500 */
	WREG32_SOC15(GC, 0, mmRLC_LB_CNTR_MAX, 0x00000500);

	mutex_lock(&adev->grbm_idx_mutex);
	/* set mmRLC_LB_INIT_CU_MASK thru broadcast mode to enable all SE/SH*/
	gfx_v9_0_select_se_sh(adev, 0xffffffff, 0xffffffff, 0xffffffff);
	WREG32_SOC15(GC, 0, mmRLC_LB_INIT_CU_MASK, 0xffffffff);

	/* set mmRLC_LB_PARAMS = 0x003F_1006 */
	data = REG_SET_FIELD(0, RLC_LB_PARAMS, FIFO_SAMPLES, 0x0003);
	data |= REG_SET_FIELD(data, RLC_LB_PARAMS, PG_IDLE_SAMPLES, 0x0010);
	data |= REG_SET_FIELD(data, RLC_LB_PARAMS, PG_IDLE_SAMPLE_INTERVAL, 0x033F);
	WREG32_SOC15(GC, 0, mmRLC_LB_PARAMS, data);

	/* set mmRLC_GPM_GENERAL_7[31-16] = 0x00C0 */
	data = RREG32_SOC15(GC, 0, mmRLC_GPM_GENERAL_7);
	data &= 0x0000FFFF;
	data |= 0x00C00000;
	WREG32_SOC15(GC, 0, mmRLC_GPM_GENERAL_7, data);

	/* set RLC_LB_ALWAYS_ACTIVE_CU_MASK = 0xFFF */
	WREG32_SOC15(GC, 0, mmRLC_LB_ALWAYS_ACTIVE_CU_MASK, 0xFFF);

	/* set RLC_LB_CNTL = 0x8000_0095, 31 bit is reserved,
	 * but used for RLC_LB_CNTL configuration */
	data = RLC_LB_CNTL__LB_CNT_SPIM_ACTIVE_MASK;
	data |= REG_SET_FIELD(data, RLC_LB_CNTL, CU_MASK_USED_OFF_HYST, 0x09);
	data |= REG_SET_FIELD(data, RLC_LB_CNTL, RESERVED, 0x80000);
	WREG32_SOC15(GC, 0, mmRLC_LB_CNTL, data);
	mutex_unlock(&adev->grbm_idx_mutex);
}

static void gfx_v9_0_enable_lbpw(struct amdgpu_device *adev, bool enable)
{
	WREG32_FIELD15(GC, 0, RLC_LB_CNTL, LOAD_BALANCE_ENABLE, enable ? 1 : 0);
}

static void rv_init_cp_jump_table(struct amdgpu_device *adev)
{
	const __le32 *fw_data;
	volatile u32 *dst_ptr;
	int me, i, max_me = 5;
	u32 bo_offset = 0;
	u32 table_offset, table_size;

	/* write the cp table buffer */
	dst_ptr = adev->gfx.rlc.cp_table_ptr;
	for (me = 0; me < max_me; me++) {
		if (me == 0) {
			const struct gfx_firmware_header_v1_0 *hdr =
				(const struct gfx_firmware_header_v1_0 *)adev->gfx.ce_fw->data;
			fw_data = (const __le32 *)
				(adev->gfx.ce_fw->data +
				 le32_to_cpu(hdr->header.ucode_array_offset_bytes));
			table_offset = le32_to_cpu(hdr->jt_offset);
			table_size = le32_to_cpu(hdr->jt_size);
		} else if (me == 1) {
			const struct gfx_firmware_header_v1_0 *hdr =
				(const struct gfx_firmware_header_v1_0 *)adev->gfx.pfp_fw->data;
			fw_data = (const __le32 *)
				(adev->gfx.pfp_fw->data +
				 le32_to_cpu(hdr->header.ucode_array_offset_bytes));
			table_offset = le32_to_cpu(hdr->jt_offset);
			table_size = le32_to_cpu(hdr->jt_size);
		} else if (me == 2) {
			const struct gfx_firmware_header_v1_0 *hdr =
				(const struct gfx_firmware_header_v1_0 *)adev->gfx.me_fw->data;
			fw_data = (const __le32 *)
				(adev->gfx.me_fw->data +
				 le32_to_cpu(hdr->header.ucode_array_offset_bytes));
			table_offset = le32_to_cpu(hdr->jt_offset);
			table_size = le32_to_cpu(hdr->jt_size);
		} else if (me == 3) {
			const struct gfx_firmware_header_v1_0 *hdr =
				(const struct gfx_firmware_header_v1_0 *)adev->gfx.mec_fw->data;
			fw_data = (const __le32 *)
				(adev->gfx.mec_fw->data +
				 le32_to_cpu(hdr->header.ucode_array_offset_bytes));
			table_offset = le32_to_cpu(hdr->jt_offset);
			table_size = le32_to_cpu(hdr->jt_size);
		} else  if (me == 4) {
			const struct gfx_firmware_header_v1_0 *hdr =
				(const struct gfx_firmware_header_v1_0 *)adev->gfx.mec2_fw->data;
			fw_data = (const __le32 *)
				(adev->gfx.mec2_fw->data +
				 le32_to_cpu(hdr->header.ucode_array_offset_bytes));
			table_offset = le32_to_cpu(hdr->jt_offset);
			table_size = le32_to_cpu(hdr->jt_size);
		}

		for (i = 0; i < table_size; i ++) {
			dst_ptr[bo_offset + i] =
				cpu_to_le32(le32_to_cpu(fw_data[table_offset + i]));
		}

		bo_offset += table_size;
	}
}

static void gfx_v9_0_rlc_fini(struct amdgpu_device *adev)
{
	/* clear state block */
	amdgpu_bo_free_kernel(&adev->gfx.rlc.clear_state_obj,
			&adev->gfx.rlc.clear_state_gpu_addr,
			(void **)&adev->gfx.rlc.cs_ptr);

	/* jump table block */
	amdgpu_bo_free_kernel(&adev->gfx.rlc.cp_table_obj,
			&adev->gfx.rlc.cp_table_gpu_addr,
			(void **)&adev->gfx.rlc.cp_table_ptr);
}

static int gfx_v9_0_rlc_init(struct amdgpu_device *adev)
{
	volatile u32 *dst_ptr;
	u32 dws;
	const struct cs_section_def *cs_data;
	int r;

	adev->gfx.rlc.cs_data = gfx9_cs_data;

	cs_data = adev->gfx.rlc.cs_data;

	if (cs_data) {
		/* clear state block */
		adev->gfx.rlc.clear_state_size = dws = gfx_v9_0_get_csb_size(adev);
		r = amdgpu_bo_create_reserved(adev, dws * 4, PAGE_SIZE,
					      AMDGPU_GEM_DOMAIN_VRAM,
					      &adev->gfx.rlc.clear_state_obj,
					      &adev->gfx.rlc.clear_state_gpu_addr,
					      (void **)&adev->gfx.rlc.cs_ptr);
		if (r) {
			dev_err(adev->dev, "(%d) failed to create rlc csb bo\n",
				r);
			gfx_v9_0_rlc_fini(adev);
			return r;
		}
		/* set up the cs buffer */
		dst_ptr = adev->gfx.rlc.cs_ptr;
		gfx_v9_0_get_csb_buffer(adev, dst_ptr);
		amdgpu_bo_kunmap(adev->gfx.rlc.clear_state_obj);
		amdgpu_bo_unreserve(adev->gfx.rlc.clear_state_obj);
	}

	if (adev->asic_type == CHIP_RAVEN) {
		/* TODO: double check the cp_table_size for RV */
		adev->gfx.rlc.cp_table_size = ALIGN(96 * 5 * 4, 2048) + (64 * 1024); /* JT + GDS */
		r = amdgpu_bo_create_reserved(adev, adev->gfx.rlc.cp_table_size,
					      PAGE_SIZE, AMDGPU_GEM_DOMAIN_VRAM,
					      &adev->gfx.rlc.cp_table_obj,
					      &adev->gfx.rlc.cp_table_gpu_addr,
					      (void **)&adev->gfx.rlc.cp_table_ptr);
		if (r) {
			dev_err(adev->dev,
				"(%d) failed to create cp table bo\n", r);
			gfx_v9_0_rlc_fini(adev);
			return r;
		}

		rv_init_cp_jump_table(adev);
		amdgpu_bo_kunmap(adev->gfx.rlc.cp_table_obj);
		amdgpu_bo_unreserve(adev->gfx.rlc.cp_table_obj);

		gfx_v9_0_init_lbpw(adev);
	}

	return 0;
}

static void gfx_v9_0_mec_fini(struct amdgpu_device *adev)
{
	amdgpu_bo_free_kernel(&adev->gfx.mec.hpd_eop_obj, NULL, NULL);
	amdgpu_bo_free_kernel(&adev->gfx.mec.mec_fw_obj, NULL, NULL);
}

static int gfx_v9_0_mec_init(struct amdgpu_device *adev)
{
	int r;
	u32 *hpd;
	const __le32 *fw_data;
	unsigned fw_size;
	u32 *fw;
	size_t mec_hpd_size;

	const struct gfx_firmware_header_v1_0 *mec_hdr;

	bitmap_zero(adev->gfx.mec.queue_bitmap, AMDGPU_MAX_COMPUTE_QUEUES);

	/* take ownership of the relevant compute queues */
	amdgpu_gfx_compute_queue_acquire(adev);
	mec_hpd_size = adev->gfx.num_compute_rings * GFX9_MEC_HPD_SIZE;

	r = amdgpu_bo_create_reserved(adev, mec_hpd_size, PAGE_SIZE,
				      AMDGPU_GEM_DOMAIN_GTT,
				      &adev->gfx.mec.hpd_eop_obj,
				      &adev->gfx.mec.hpd_eop_gpu_addr,
				      (void **)&hpd);
	if (r) {
		dev_warn(adev->dev, "(%d) create HDP EOP bo failed\n", r);
		gfx_v9_0_mec_fini(adev);
		return r;
	}

	memset(hpd, 0, adev->gfx.mec.hpd_eop_obj->tbo.mem.size);

	amdgpu_bo_kunmap(adev->gfx.mec.hpd_eop_obj);
	amdgpu_bo_unreserve(adev->gfx.mec.hpd_eop_obj);

	mec_hdr = (const struct gfx_firmware_header_v1_0 *)adev->gfx.mec_fw->data;

	fw_data = (const __le32 *)
		(adev->gfx.mec_fw->data +
		 le32_to_cpu(mec_hdr->header.ucode_array_offset_bytes));
	fw_size = le32_to_cpu(mec_hdr->header.ucode_size_bytes) / 4;

	r = amdgpu_bo_create_reserved(adev, mec_hdr->header.ucode_size_bytes,
				      PAGE_SIZE, AMDGPU_GEM_DOMAIN_GTT,
				      &adev->gfx.mec.mec_fw_obj,
				      &adev->gfx.mec.mec_fw_gpu_addr,
				      (void **)&fw);
	if (r) {
		dev_warn(adev->dev, "(%d) create mec firmware bo failed\n", r);
		gfx_v9_0_mec_fini(adev);
		return r;
	}

	memcpy(fw, fw_data, fw_size);

	amdgpu_bo_kunmap(adev->gfx.mec.mec_fw_obj);
	amdgpu_bo_unreserve(adev->gfx.mec.mec_fw_obj);

	return 0;
}

static uint32_t wave_read_ind(struct amdgpu_device *adev, uint32_t simd, uint32_t wave, uint32_t address)
{
	WREG32_SOC15(GC, 0, mmSQ_IND_INDEX,
		(wave << SQ_IND_INDEX__WAVE_ID__SHIFT) |
		(simd << SQ_IND_INDEX__SIMD_ID__SHIFT) |
		(address << SQ_IND_INDEX__INDEX__SHIFT) |
		(SQ_IND_INDEX__FORCE_READ_MASK));
	return RREG32_SOC15(GC, 0, mmSQ_IND_DATA);
}

static void wave_read_regs(struct amdgpu_device *adev, uint32_t simd,
			   uint32_t wave, uint32_t thread,
			   uint32_t regno, uint32_t num, uint32_t *out)
{
	WREG32_SOC15(GC, 0, mmSQ_IND_INDEX,
		(wave << SQ_IND_INDEX__WAVE_ID__SHIFT) |
		(simd << SQ_IND_INDEX__SIMD_ID__SHIFT) |
		(regno << SQ_IND_INDEX__INDEX__SHIFT) |
		(thread << SQ_IND_INDEX__THREAD_ID__SHIFT) |
		(SQ_IND_INDEX__FORCE_READ_MASK) |
		(SQ_IND_INDEX__AUTO_INCR_MASK));
	while (num--)
		*(out++) = RREG32_SOC15(GC, 0, mmSQ_IND_DATA);
}

static void gfx_v9_0_read_wave_data(struct amdgpu_device *adev, uint32_t simd, uint32_t wave, uint32_t *dst, int *no_fields)
{
	/* type 1 wave data */
	dst[(*no_fields)++] = 1;
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_STATUS);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_PC_LO);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_PC_HI);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_EXEC_LO);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_EXEC_HI);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_HW_ID);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_INST_DW0);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_INST_DW1);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_GPR_ALLOC);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_LDS_ALLOC);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_TRAPSTS);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_IB_STS);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_IB_DBG0);
	dst[(*no_fields)++] = wave_read_ind(adev, simd, wave, ixSQ_WAVE_M0);
}

static void gfx_v9_0_read_wave_sgprs(struct amdgpu_device *adev, uint32_t simd,
				     uint32_t wave, uint32_t start,
				     uint32_t size, uint32_t *dst)
{
	wave_read_regs(
		adev, simd, wave, 0,
		start + SQIND_WAVE_SGPRS_OFFSET, size, dst);
}


static const struct amdgpu_gfx_funcs gfx_v9_0_gfx_funcs = {
	.get_gpu_clock_counter = &gfx_v9_0_get_gpu_clock_counter,
	.select_se_sh = &gfx_v9_0_select_se_sh,
	.read_wave_data = &gfx_v9_0_read_wave_data,
	.read_wave_sgprs = &gfx_v9_0_read_wave_sgprs,
};

static void gfx_v9_0_gpu_early_init(struct amdgpu_device *adev)
{
	u32 gb_addr_config;

	adev->gfx.funcs = &gfx_v9_0_gfx_funcs;

	switch (adev->asic_type) {
	case CHIP_VEGA10:
		adev->gfx.config.max_hw_contexts = 8;
		adev->gfx.config.sc_prim_fifo_size_frontend = 0x20;
		adev->gfx.config.sc_prim_fifo_size_backend = 0x100;
		adev->gfx.config.sc_hiz_tile_fifo_size = 0x30;
		adev->gfx.config.sc_earlyz_tile_fifo_size = 0x4C0;
		gb_addr_config = VEGA10_GB_ADDR_CONFIG_GOLDEN;
		break;
	case CHIP_RAVEN:
		adev->gfx.config.max_hw_contexts = 8;
		adev->gfx.config.sc_prim_fifo_size_frontend = 0x20;
		adev->gfx.config.sc_prim_fifo_size_backend = 0x100;
		adev->gfx.config.sc_hiz_tile_fifo_size = 0x30;
		adev->gfx.config.sc_earlyz_tile_fifo_size = 0x4C0;
		gb_addr_config = RAVEN_GB_ADDR_CONFIG_GOLDEN;
		break;
	default:
		BUG();
		break;
	}

	adev->gfx.config.gb_addr_config = gb_addr_config;

	adev->gfx.config.gb_addr_config_fields.num_pipes = 1 <<
			REG_GET_FIELD(
					adev->gfx.config.gb_addr_config,
					GB_ADDR_CONFIG,
					NUM_PIPES);

	adev->gfx.config.max_tile_pipes =
		adev->gfx.config.gb_addr_config_fields.num_pipes;

	adev->gfx.config.gb_addr_config_fields.num_banks = 1 <<
			REG_GET_FIELD(
					adev->gfx.config.gb_addr_config,
					GB_ADDR_CONFIG,
					NUM_BANKS);
	adev->gfx.config.gb_addr_config_fields.max_compress_frags = 1 <<
			REG_GET_FIELD(
					adev->gfx.config.gb_addr_config,
					GB_ADDR_CONFIG,
					MAX_COMPRESSED_FRAGS);
	adev->gfx.config.gb_addr_config_fields.num_rb_per_se = 1 <<
			REG_GET_FIELD(
					adev->gfx.config.gb_addr_config,
					GB_ADDR_CONFIG,
					NUM_RB_PER_SE);
	adev->gfx.config.gb_addr_config_fields.num_se = 1 <<
			REG_GET_FIELD(
					adev->gfx.config.gb_addr_config,
					GB_ADDR_CONFIG,
					NUM_SHADER_ENGINES);
	adev->gfx.config.gb_addr_config_fields.pipe_interleave_size = 1 << (8 +
			REG_GET_FIELD(
					adev->gfx.config.gb_addr_config,
					GB_ADDR_CONFIG,
					PIPE_INTERLEAVE_SIZE));
}

static int gfx_v9_0_ngg_create_buf(struct amdgpu_device *adev,
				   struct amdgpu_ngg_buf *ngg_buf,
				   int size_se,
				   int default_size_se)
{
	int r;

	if (size_se < 0) {
		dev_err(adev->dev, "Buffer size is invalid: %d\n", size_se);
		return -EINVAL;
	}
	size_se = size_se ? size_se : default_size_se;

	ngg_buf->size = size_se * adev->gfx.config.max_shader_engines;
	r = amdgpu_bo_create_kernel(adev, ngg_buf->size,
				    PAGE_SIZE, AMDGPU_GEM_DOMAIN_VRAM,
				    &ngg_buf->bo,
				    &ngg_buf->gpu_addr,
				    NULL);
	if (r) {
		dev_err(adev->dev, "(%d) failed to create NGG buffer\n", r);
		return r;
	}
	ngg_buf->bo_size = amdgpu_bo_size(ngg_buf->bo);

	return r;
}

static int gfx_v9_0_ngg_fini(struct amdgpu_device *adev)
{
	int i;

	for (i = 0; i < NGG_BUF_MAX; i++)
		amdgpu_bo_free_kernel(&adev->gfx.ngg.buf[i].bo,
				      &adev->gfx.ngg.buf[i].gpu_addr,
				      NULL);

	memset(&adev->gfx.ngg.buf[0], 0,
			sizeof(struct amdgpu_ngg_buf) * NGG_BUF_MAX);

	adev->gfx.ngg.init = false;

	return 0;
}

static int gfx_v9_0_ngg_init(struct amdgpu_device *adev)
{
	int r;

	if (!amdgpu_ngg || adev->gfx.ngg.init == true)
		return 0;

	/* GDS reserve memory: 64 bytes alignment */
	adev->gfx.ngg.gds_reserve_size = ALIGN(5 * 4, 0x40);
	adev->gds.mem.total_size -= adev->gfx.ngg.gds_reserve_size;
	adev->gds.mem.gfx_partition_size -= adev->gfx.ngg.gds_reserve_size;
	adev->gfx.ngg.gds_reserve_addr = amdgpu_gds_reg_offset[0].mem_base;
	adev->gfx.ngg.gds_reserve_addr += adev->gds.mem.gfx_partition_size;

	/* Primitive Buffer */
	r = gfx_v9_0_ngg_create_buf(adev, &adev->gfx.ngg.buf[NGG_PRIM],
				    amdgpu_prim_buf_per_se,
				    64 * 1024);
	if (r) {
		dev_err(adev->dev, "Failed to create Primitive Buffer\n");
		goto err;
	}

	/* Position Buffer */
	r = gfx_v9_0_ngg_create_buf(adev, &adev->gfx.ngg.buf[NGG_POS],
				    amdgpu_pos_buf_per_se,
				    256 * 1024);
	if (r) {
		dev_err(adev->dev, "Failed to create Position Buffer\n");
		goto err;
	}

	/* Control Sideband */
	r = gfx_v9_0_ngg_create_buf(adev, &adev->gfx.ngg.buf[NGG_CNTL],
				    amdgpu_cntl_sb_buf_per_se,
				    256);
	if (r) {
		dev_err(adev->dev, "Failed to create Control Sideband Buffer\n");
		goto err;
	}

	/* Parameter Cache, not created by default */
	if (amdgpu_param_buf_per_se <= 0)
		goto out;

	r = gfx_v9_0_ngg_create_buf(adev, &adev->gfx.ngg.buf[NGG_PARAM],
				    amdgpu_param_buf_per_se,
				    512 * 1024);
	if (r) {
		dev_err(adev->dev, "Failed to create Parameter Cache\n");
		goto err;
	}

out:
	adev->gfx.ngg.init = true;
	return 0;
err:
	gfx_v9_0_ngg_fini(adev);
	return r;
}

static int gfx_v9_0_ngg_en(struct amdgpu_device *adev)
{
	struct amdgpu_ring *ring = &adev->gfx.gfx_ring[0];
	int r;
	u32 data;
	u32 size;
	u32 base;

	if (!amdgpu_ngg)
		return 0;

	/* Program buffer size */
	data = 0;
	size = adev->gfx.ngg.buf[NGG_PRIM].size / 256;
	data = REG_SET_FIELD(data, WD_BUF_RESOURCE_1, INDEX_BUF_SIZE, size);

	size = adev->gfx.ngg.buf[NGG_POS].size / 256;
	data = REG_SET_FIELD(data, WD_BUF_RESOURCE_1, POS_BUF_SIZE, size);

	WREG32_SOC15(GC, 0, mmWD_BUF_RESOURCE_1, data);

	data = 0;
	size = adev->gfx.ngg.buf[NGG_CNTL].size / 256;
	data = REG_SET_FIELD(data, WD_BUF_RESOURCE_2, CNTL_SB_BUF_SIZE, size);

	size = adev->gfx.ngg.buf[NGG_PARAM].size / 1024;
	data = REG_SET_FIELD(data, WD_BUF_RESOURCE_2, PARAM_BUF_SIZE, size);

	WREG32_SOC15(GC, 0, mmWD_BUF_RESOURCE_2, data);

	/* Program buffer base address */
	base = lower_32_bits(adev->gfx.ngg.buf[NGG_PRIM].gpu_addr);
	data = REG_SET_FIELD(0, WD_INDEX_BUF_BASE, BASE, base);
	WREG32_SOC15(GC, 0, mmWD_INDEX_BUF_BASE, data);

	base = upper_32_bits(adev->gfx.ngg.buf[NGG_PRIM].gpu_addr);
	data = REG_SET_FIELD(0, WD_INDEX_BUF_BASE_HI, BASE_HI, base);
	WREG32_SOC15(GC, 0, mmWD_INDEX_BUF_BASE_HI, data);

	base = lower_32_bits(adev->gfx.ngg.buf[NGG_POS].gpu_addr);
	data = REG_SET_FIELD(0, WD_POS_BUF_BASE, BASE, base);
	WREG32_SOC15(GC, 0, mmWD_POS_BUF_BASE, data);

	base = upper_32_bits(adev->gfx.ngg.buf[NGG_POS].gpu_addr);
	data = REG_SET_FIELD(0, WD_POS_BUF_BASE_HI, BASE_HI, base);
	WREG32_SOC15(GC, 0, mmWD_POS_BUF_BASE_HI, data);

	base = lower_32_bits(adev->gfx.ngg.buf[NGG_CNTL].gpu_addr);
	data = REG_SET_FIELD(0, WD_CNTL_SB_BUF_BASE, BASE, base);
	WREG32_SOC15(GC, 0, mmWD_CNTL_SB_BUF_BASE, data);

	base = upper_32_bits(adev->gfx.ngg.buf[NGG_CNTL].gpu_addr);
	data = REG_SET_FIELD(0, WD_CNTL_SB_BUF_BASE_HI, BASE_HI, base);
	WREG32_SOC15(GC, 0, mmWD_CNTL_SB_BUF_BASE_HI, data);

	/* Clear GDS reserved memory */
	r = amdgpu_ring_alloc(ring, 17);
	if (r) {
		DRM_ERROR("amdgpu: NGG failed to lock ring %d (%d).\n",
			  ring->idx, r);
		return r;
	}

	gfx_v9_0_write_data_to_reg(ring, 0, false,
				   amdgpu_gds_reg_offset[0].mem_size,
			           (adev->gds.mem.total_size +
				    adev->gfx.ngg.gds_reserve_size) >>
				   AMDGPU_GDS_SHIFT);

	amdgpu_ring_write(ring, PACKET3(PACKET3_DMA_DATA, 5));
	amdgpu_ring_write(ring, (PACKET3_DMA_DATA_CP_SYNC |
				PACKET3_DMA_DATA_SRC_SEL(2)));
	amdgpu_ring_write(ring, 0);
	amdgpu_ring_write(ring, 0);
	amdgpu_ring_write(ring, adev->gfx.ngg.gds_reserve_addr);
	amdgpu_ring_write(ring, 0);
	amdgpu_ring_write(ring, adev->gfx.ngg.gds_reserve_size);


	gfx_v9_0_write_data_to_reg(ring, 0, false,
				   amdgpu_gds_reg_offset[0].mem_size, 0);

	amdgpu_ring_commit(ring);

	return 0;
}

static int gfx_v9_0_compute_ring_init(struct amdgpu_device *adev, int ring_id,
				      int mec, int pipe, int queue)
{
	int r;
	unsigned irq_type;
	struct amdgpu_ring *ring = &adev->gfx.compute_ring[ring_id];

	ring = &adev->gfx.compute_ring[ring_id];

	/* mec0 is me1 */
	ring->me = mec + 1;
	ring->pipe = pipe;
	ring->queue = queue;

	ring->ring_obj = NULL;
	ring->use_doorbell = true;
	ring->doorbell_index = (AMDGPU_DOORBELL_MEC_RING0 + ring_id) << 1;
	ring->eop_gpu_addr = adev->gfx.mec.hpd_eop_gpu_addr
				+ (ring_id * GFX9_MEC_HPD_SIZE);
	sprintf(ring->name, "comp_%d.%d.%d", ring->me, ring->pipe, ring->queue);

	irq_type = AMDGPU_CP_IRQ_COMPUTE_MEC1_PIPE0_EOP
		+ ((ring->me - 1) * adev->gfx.mec.num_pipe_per_mec)
		+ ring->pipe;

	/* type-2 packets are deprecated on MEC, use type-3 instead */
	r = amdgpu_ring_init(adev, ring, 1024,
			     &adev->gfx.eop_irq, irq_type);
	if (r)
		return r;


	return 0;
}

static int gfx_v9_0_sw_init(void *handle)
{
	int i, j, k, r, ring_id;
	struct amdgpu_ring *ring;
	struct amdgpu_kiq *kiq;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	switch (adev->asic_type) {
	case CHIP_VEGA10:
	case CHIP_RAVEN:
		adev->gfx.mec.num_mec = 2;
		break;
	default:
		adev->gfx.mec.num_mec = 1;
		break;
	}

	adev->gfx.mec.num_pipe_per_mec = 4;
	adev->gfx.mec.num_queue_per_pipe = 8;

	/* KIQ event */
	r = amdgpu_irq_add_id(adev, AMDGPU_IH_CLIENTID_GRBM_CP, 178, &adev->gfx.kiq.irq);
	if (r)
		return r;

	/* EOP Event */
	r = amdgpu_irq_add_id(adev, AMDGPU_IH_CLIENTID_GRBM_CP, 181, &adev->gfx.eop_irq);
	if (r)
		return r;

	/* Privileged reg */
	r = amdgpu_irq_add_id(adev, AMDGPU_IH_CLIENTID_GRBM_CP, 184,
			      &adev->gfx.priv_reg_irq);
	if (r)
		return r;

	/* Privileged inst */
	r = amdgpu_irq_add_id(adev, AMDGPU_IH_CLIENTID_GRBM_CP, 185,
			      &adev->gfx.priv_inst_irq);
	if (r)
		return r;

	adev->gfx.gfx_current_status = AMDGPU_GFX_NORMAL_MODE;

	gfx_v9_0_scratch_init(adev);

	r = gfx_v9_0_init_microcode(adev);
	if (r) {
		DRM_ERROR("Failed to load gfx firmware!\n");
		return r;
	}

	r = gfx_v9_0_rlc_init(adev);
	if (r) {
		DRM_ERROR("Failed to init rlc BOs!\n");
		return r;
	}

	r = gfx_v9_0_mec_init(adev);
	if (r) {
		DRM_ERROR("Failed to init MEC BOs!\n");
		return r;
	}

	/* set up the gfx ring */
	for (i = 0; i < adev->gfx.num_gfx_rings; i++) {
		ring = &adev->gfx.gfx_ring[i];
		ring->ring_obj = NULL;
		sprintf(ring->name, "gfx");
		ring->use_doorbell = true;
		ring->doorbell_index = AMDGPU_DOORBELL64_GFX_RING0 << 1;
		r = amdgpu_ring_init(adev, ring, 1024,
				     &adev->gfx.eop_irq, AMDGPU_CP_IRQ_GFX_EOP);
		if (r)
			return r;
	}

	/* set up the compute queues - allocate horizontally across pipes */
	ring_id = 0;
	for (i = 0; i < adev->gfx.mec.num_mec; ++i) {
		for (j = 0; j < adev->gfx.mec.num_queue_per_pipe; j++) {
			for (k = 0; k < adev->gfx.mec.num_pipe_per_mec; k++) {
				if (!amdgpu_gfx_is_mec_queue_enabled(adev, i, k, j))
					continue;

				r = gfx_v9_0_compute_ring_init(adev,
							       ring_id,
							       i, k, j);
				if (r)
					return r;

				ring_id++;
			}
		}
	}

	r = amdgpu_gfx_kiq_init(adev, GFX9_MEC_HPD_SIZE);
	if (r) {
		DRM_ERROR("Failed to init KIQ BOs!\n");
		return r;
	}

	kiq = &adev->gfx.kiq;
	r = amdgpu_gfx_kiq_init_ring(adev, &kiq->ring, &kiq->irq);
	if (r)
		return r;

	/* create MQD for all compute queues as wel as KIQ for SRIOV case */
	r = amdgpu_gfx_compute_mqd_sw_init(adev, sizeof(struct v9_mqd));
	if (r)
		return r;

	/* reserve GDS, GWS and OA resource for gfx */
	r = amdgpu_bo_create_kernel(adev, adev->gds.mem.gfx_partition_size,
				    PAGE_SIZE, AMDGPU_GEM_DOMAIN_GDS,
				    &adev->gds.gds_gfx_bo, NULL, NULL);
	if (r)
		return r;

	r = amdgpu_bo_create_kernel(adev, adev->gds.gws.gfx_partition_size,
				    PAGE_SIZE, AMDGPU_GEM_DOMAIN_GWS,
				    &adev->gds.gws_gfx_bo, NULL, NULL);
	if (r)
		return r;

	r = amdgpu_bo_create_kernel(adev, adev->gds.oa.gfx_partition_size,
				    PAGE_SIZE, AMDGPU_GEM_DOMAIN_OA,
				    &adev->gds.oa_gfx_bo, NULL, NULL);
	if (r)
		return r;

	adev->gfx.ce_ram_size = 0x8000;

	gfx_v9_0_gpu_early_init(adev);

	r = gfx_v9_0_ngg_init(adev);
	if (r)
		return r;

	return 0;
}


static int gfx_v9_0_sw_fini(void *handle)
{
	int i;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	amdgpu_bo_free_kernel(&adev->gds.oa_gfx_bo, NULL, NULL);
	amdgpu_bo_free_kernel(&adev->gds.gws_gfx_bo, NULL, NULL);
	amdgpu_bo_free_kernel(&adev->gds.gds_gfx_bo, NULL, NULL);

	for (i = 0; i < adev->gfx.num_gfx_rings; i++)
		amdgpu_ring_fini(&adev->gfx.gfx_ring[i]);
	for (i = 0; i < adev->gfx.num_compute_rings; i++)
		amdgpu_ring_fini(&adev->gfx.compute_ring[i]);

	amdgpu_gfx_compute_mqd_sw_fini(adev);
	amdgpu_gfx_kiq_free_ring(&adev->gfx.kiq.ring, &adev->gfx.kiq.irq);
	amdgpu_gfx_kiq_fini(adev);

	gfx_v9_0_mec_fini(adev);
	gfx_v9_0_ngg_fini(adev);

	return 0;
}


static void gfx_v9_0_tiling_mode_table_init(struct amdgpu_device *adev)
{
	/* TODO */
}

static void gfx_v9_0_select_se_sh(struct amdgpu_device *adev, u32 se_num, u32 sh_num, u32 instance)
{
	u32 data;

	if (instance == 0xffffffff)
		data = REG_SET_FIELD(0, GRBM_GFX_INDEX, INSTANCE_BROADCAST_WRITES, 1);
	else
		data = REG_SET_FIELD(0, GRBM_GFX_INDEX, INSTANCE_INDEX, instance);

	if (se_num == 0xffffffff)
		data = REG_SET_FIELD(data, GRBM_GFX_INDEX, SE_BROADCAST_WRITES, 1);
	else
		data = REG_SET_FIELD(data, GRBM_GFX_INDEX, SE_INDEX, se_num);

	if (sh_num == 0xffffffff)
		data = REG_SET_FIELD(data, GRBM_GFX_INDEX, SH_BROADCAST_WRITES, 1);
	else
		data = REG_SET_FIELD(data, GRBM_GFX_INDEX, SH_INDEX, sh_num);

	WREG32_SOC15(GC, 0, mmGRBM_GFX_INDEX, data);
}

static u32 gfx_v9_0_get_rb_active_bitmap(struct amdgpu_device *adev)
{
	u32 data, mask;

	data = RREG32_SOC15(GC, 0, mmCC_RB_BACKEND_DISABLE);
	data |= RREG32_SOC15(GC, 0, mmGC_USER_RB_BACKEND_DISABLE);

	data &= CC_RB_BACKEND_DISABLE__BACKEND_DISABLE_MASK;
	data >>= GC_USER_RB_BACKEND_DISABLE__BACKEND_DISABLE__SHIFT;

	mask = amdgpu_gfx_create_bitmask(adev->gfx.config.max_backends_per_se /
					 adev->gfx.config.max_sh_per_se);

	return (~data) & mask;
}

static void gfx_v9_0_setup_rb(struct amdgpu_device *adev)
{
	int i, j;
	u32 data;
	u32 active_rbs = 0;
	u32 rb_bitmap_width_per_sh = adev->gfx.config.max_backends_per_se /
					adev->gfx.config.max_sh_per_se;

	mutex_lock(&adev->grbm_idx_mutex);
	for (i = 0; i < adev->gfx.config.max_shader_engines; i++) {
		for (j = 0; j < adev->gfx.config.max_sh_per_se; j++) {
			gfx_v9_0_select_se_sh(adev, i, j, 0xffffffff);
			data = gfx_v9_0_get_rb_active_bitmap(adev);
			active_rbs |= data << ((i * adev->gfx.config.max_sh_per_se + j) *
					       rb_bitmap_width_per_sh);
		}
	}
	gfx_v9_0_select_se_sh(adev, 0xffffffff, 0xffffffff, 0xffffffff);
	mutex_unlock(&adev->grbm_idx_mutex);

	adev->gfx.config.backend_enable_mask = active_rbs;
	adev->gfx.config.num_rbs = hweight32(active_rbs);
}

#define DEFAULT_SH_MEM_BASES	(0x6000)
#define FIRST_COMPUTE_VMID	(8)
#define LAST_COMPUTE_VMID	(16)
static void gfx_v9_0_init_compute_vmid(struct amdgpu_device *adev)
{
	int i;
	uint32_t sh_mem_config;
	uint32_t sh_mem_bases;

	/*
	 * Configure apertures:
	 * LDS:         0x60000000'00000000 - 0x60000001'00000000 (4GB)
	 * Scratch:     0x60000001'00000000 - 0x60000002'00000000 (4GB)
	 * GPUVM:       0x60010000'00000000 - 0x60020000'00000000 (1TB)
	 */
	sh_mem_bases = DEFAULT_SH_MEM_BASES | (DEFAULT_SH_MEM_BASES << 16);

	sh_mem_config = SH_MEM_ADDRESS_MODE_64 |
			SH_MEM_ALIGNMENT_MODE_UNALIGNED <<
			SH_MEM_CONFIG__ALIGNMENT_MODE__SHIFT;

	mutex_lock(&adev->srbm_mutex);
	for (i = FIRST_COMPUTE_VMID; i < LAST_COMPUTE_VMID; i++) {
		soc15_grbm_select(adev, 0, 0, 0, i);
		/* CP and shaders */
		WREG32_SOC15(GC, 0, mmSH_MEM_CONFIG, sh_mem_config);
		WREG32_SOC15(GC, 0, mmSH_MEM_BASES, sh_mem_bases);
	}
	soc15_grbm_select(adev, 0, 0, 0, 0);
	mutex_unlock(&adev->srbm_mutex);
}

static void gfx_v9_0_gpu_init(struct amdgpu_device *adev)
{
	u32 tmp;
	int i;

	WREG32_FIELD15(GC, 0, GRBM_CNTL, READ_TIMEOUT, 0xff);

	gfx_v9_0_tiling_mode_table_init(adev);

	gfx_v9_0_setup_rb(adev);
	gfx_v9_0_get_cu_info(adev, &adev->gfx.cu_info);

	/* XXX SH_MEM regs */
	/* where to put LDS, scratch, GPUVM in FSA64 space */
	mutex_lock(&adev->srbm_mutex);
	for (i = 0; i < 16; i++) {
		soc15_grbm_select(adev, 0, 0, 0, i);
		/* CP and shaders */
		tmp = 0;
		tmp = REG_SET_FIELD(tmp, SH_MEM_CONFIG, ALIGNMENT_MODE,
				    SH_MEM_ALIGNMENT_MODE_UNALIGNED);
		WREG32_SOC15(GC, 0, mmSH_MEM_CONFIG, tmp);
		WREG32_SOC15(GC, 0, mmSH_MEM_BASES, 0);
	}
	soc15_grbm_select(adev, 0, 0, 0, 0);

	mutex_unlock(&adev->srbm_mutex);

	gfx_v9_0_init_compute_vmid(adev);
}

static void gfx_v9_0_wait_for_rlc_serdes(struct amdgpu_device *adev)
{
	u32 i, j, k;
	u32 mask;

	mutex_lock(&adev->grbm_idx_mutex);
	for (i = 0; i < adev->gfx.config.max_shader_engines; i++) {
		for (j = 0; j < adev->gfx.config.max_sh_per_se; j++) {
			gfx_v9_0_select_se_sh(adev, i, j, 0xffffffff);
			for (k = 0; k < adev->usec_timeout; k++) {
				if (RREG32_SOC15(GC, 0, mmRLC_SERDES_CU_MASTER_BUSY) == 0)
					break;
				udelay(1);
			}
		}
	}
	gfx_v9_0_select_se_sh(adev, 0xffffffff, 0xffffffff, 0xffffffff);
	mutex_unlock(&adev->grbm_idx_mutex);

	mask = RLC_SERDES_NONCU_MASTER_BUSY__SE_MASTER_BUSY_MASK |
		RLC_SERDES_NONCU_MASTER_BUSY__GC_MASTER_BUSY_MASK |
		RLC_SERDES_NONCU_MASTER_BUSY__TC0_MASTER_BUSY_MASK |
		RLC_SERDES_NONCU_MASTER_BUSY__TC1_MASTER_BUSY_MASK;
	for (k = 0; k < adev->usec_timeout; k++) {
		if ((RREG32_SOC15(GC, 0, mmRLC_SERDES_NONCU_MASTER_BUSY) & mask) == 0)
			break;
		udelay(1);
	}
}

static void gfx_v9_0_enable_gui_idle_interrupt(struct amdgpu_device *adev,
					       bool enable)
{
	u32 tmp = RREG32_SOC15(GC, 0, mmCP_INT_CNTL_RING0);

	tmp = REG_SET_FIELD(tmp, CP_INT_CNTL_RING0, CNTX_BUSY_INT_ENABLE, enable ? 1 : 0);
	tmp = REG_SET_FIELD(tmp, CP_INT_CNTL_RING0, CNTX_EMPTY_INT_ENABLE, enable ? 1 : 0);
	tmp = REG_SET_FIELD(tmp, CP_INT_CNTL_RING0, CMP_BUSY_INT_ENABLE, enable ? 1 : 0);
	tmp = REG_SET_FIELD(tmp, CP_INT_CNTL_RING0, GFX_IDLE_INT_ENABLE, enable ? 1 : 0);

	WREG32_SOC15(GC, 0, mmCP_INT_CNTL_RING0, tmp);
}

static void gfx_v9_0_init_csb(struct amdgpu_device *adev)
{
	/* csib */
	WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_CSIB_ADDR_HI),
			adev->gfx.rlc.clear_state_gpu_addr >> 32);
	WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_CSIB_ADDR_LO),
			adev->gfx.rlc.clear_state_gpu_addr & 0xfffffffc);
	WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_CSIB_LENGTH),
			adev->gfx.rlc.clear_state_size);
}

static void gfx_v9_0_parse_ind_reg_list(int *register_list_format,
				int indirect_offset,
				int list_size,
				int *unique_indirect_regs,
				int *unique_indirect_reg_count,
				int max_indirect_reg_count,
				int *indirect_start_offsets,
				int *indirect_start_offsets_count,
				int max_indirect_start_offsets_count)
{
	int idx;
	bool new_entry = true;

	for (; indirect_offset < list_size; indirect_offset++) {

		if (new_entry) {
			new_entry = false;
			indirect_start_offsets[*indirect_start_offsets_count] = indirect_offset;
			*indirect_start_offsets_count = *indirect_start_offsets_count + 1;
			BUG_ON(*indirect_start_offsets_count >= max_indirect_start_offsets_count);
		}

		if (register_list_format[indirect_offset] == 0xFFFFFFFF) {
			new_entry = true;
			continue;
		}

		indirect_offset += 2;

		/* look for the matching indice */
		for (idx = 0; idx < *unique_indirect_reg_count; idx++) {
			if (unique_indirect_regs[idx] ==
				register_list_format[indirect_offset])
				break;
		}

		if (idx >= *unique_indirect_reg_count) {
			unique_indirect_regs[*unique_indirect_reg_count] =
				register_list_format[indirect_offset];
			idx = *unique_indirect_reg_count;
			*unique_indirect_reg_count = *unique_indirect_reg_count + 1;
			BUG_ON(*unique_indirect_reg_count >= max_indirect_reg_count);
		}

		register_list_format[indirect_offset] = idx;
	}
}

static int gfx_v9_0_init_rlc_save_restore_list(struct amdgpu_device *adev)
{
	int unique_indirect_regs[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	int unique_indirect_reg_count = 0;

	int indirect_start_offsets[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	int indirect_start_offsets_count = 0;

	int list_size = 0;
	int i = 0;
	u32 tmp = 0;

	u32 *register_list_format =
		kmalloc(adev->gfx.rlc.reg_list_format_size_bytes, GFP_KERNEL);
	if (!register_list_format)
		return -ENOMEM;
	memcpy(register_list_format, adev->gfx.rlc.register_list_format,
		adev->gfx.rlc.reg_list_format_size_bytes);

	/* setup unique_indirect_regs array and indirect_start_offsets array */
	gfx_v9_0_parse_ind_reg_list(register_list_format,
				GFX9_RLC_FORMAT_DIRECT_REG_LIST_LENGTH,
				adev->gfx.rlc.reg_list_format_size_bytes >> 2,
				unique_indirect_regs,
				&unique_indirect_reg_count,
				sizeof(unique_indirect_regs)/sizeof(int),
				indirect_start_offsets,
				&indirect_start_offsets_count,
				sizeof(indirect_start_offsets)/sizeof(int));

	/* enable auto inc in case it is disabled */
	tmp = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_SRM_CNTL));
	tmp |= RLC_SRM_CNTL__AUTO_INCR_ADDR_MASK;
	WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_SRM_CNTL), tmp);

	/* write register_restore table to offset 0x0 using RLC_SRM_ARAM_ADDR/DATA */
	WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_SRM_ARAM_ADDR),
		RLC_SAVE_RESTORE_ADDR_STARTING_OFFSET);
	for (i = 0; i < adev->gfx.rlc.reg_list_size_bytes >> 2; i++)
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_SRM_ARAM_DATA),
			adev->gfx.rlc.register_restore[i]);

	/* load direct register */
	WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_SRM_ARAM_ADDR), 0);
	for (i = 0; i < adev->gfx.rlc.reg_list_size_bytes >> 2; i++)
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_SRM_ARAM_DATA),
			adev->gfx.rlc.register_restore[i]);

	/* load indirect register */
	WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_SCRATCH_ADDR),
		adev->gfx.rlc.reg_list_format_start);
	for (i = 0; i < adev->gfx.rlc.reg_list_format_size_bytes >> 2; i++)
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_SCRATCH_DATA),
			register_list_format[i]);

	/* set save/restore list size */
	list_size = adev->gfx.rlc.reg_list_size_bytes >> 2;
	list_size = list_size >> 1;
	WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_SCRATCH_ADDR),
		adev->gfx.rlc.reg_restore_list_size);
	WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_SCRATCH_DATA), list_size);

	/* write the starting offsets to RLC scratch ram */
	WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_SCRATCH_ADDR),
		adev->gfx.rlc.starting_offsets_start);
	for (i = 0; i < sizeof(indirect_start_offsets)/sizeof(int); i++)
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_GPM_SCRATCH_DATA),
			indirect_start_offsets[i]);

	/* load unique indirect regs*/
	for (i = 0; i < sizeof(unique_indirect_regs)/sizeof(int); i++) {
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_SRM_INDEX_CNTL_ADDR_0) + i,
			unique_indirect_regs[i] & 0x3FFFF);
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_SRM_INDEX_CNTL_DATA_0) + i,
			unique_indirect_regs[i] >> 20);
	}

	kfree(register_list_format);
	return 0;
}

static void gfx_v9_0_enable_save_restore_machine(struct amdgpu_device *adev)
{
	u32 tmp = 0;

	tmp = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_SRM_CNTL));
	tmp |= RLC_SRM_CNTL__SRM_ENABLE_MASK;
	WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_SRM_CNTL), tmp);
}

static void pwr_10_0_gfxip_control_over_cgpg(struct amdgpu_device *adev,
					     bool enable)
{
	uint32_t data = 0;
	uint32_t default_data = 0;

	default_data = data = RREG32(SOC15_REG_OFFSET(PWR, 0, mmPWR_MISC_CNTL_STATUS));
	if (enable == true) {
		/* enable GFXIP control over CGPG */
		data |= PWR_MISC_CNTL_STATUS__PWR_GFX_RLC_CGPG_EN_MASK;
		if(default_data != data)
			WREG32(SOC15_REG_OFFSET(PWR, 0, mmPWR_MISC_CNTL_STATUS), data);

		/* update status */
		data &= ~PWR_MISC_CNTL_STATUS__PWR_GFXOFF_STATUS_MASK;
		data |= (2 << PWR_MISC_CNTL_STATUS__PWR_GFXOFF_STATUS__SHIFT);
		if(default_data != data)
			WREG32(SOC15_REG_OFFSET(PWR, 0, mmPWR_MISC_CNTL_STATUS), data);
	} else {
		/* restore GFXIP control over GCPG */
		data &= ~PWR_MISC_CNTL_STATUS__PWR_GFX_RLC_CGPG_EN_MASK;
		if(default_data != data)
			WREG32(SOC15_REG_OFFSET(PWR, 0, mmPWR_MISC_CNTL_STATUS), data);
	}
}

static void gfx_v9_0_init_gfx_power_gating(struct amdgpu_device *adev)
{
	uint32_t data = 0;

	if (adev->pg_flags & (AMD_PG_SUPPORT_GFX_PG |
			      AMD_PG_SUPPORT_GFX_SMG |
			      AMD_PG_SUPPORT_GFX_DMG)) {
		/* init IDLE_POLL_COUNT = 60 */
		data = RREG32(SOC15_REG_OFFSET(GC, 0, mmCP_RB_WPTR_POLL_CNTL));
		data &= ~CP_RB_WPTR_POLL_CNTL__IDLE_POLL_COUNT_MASK;
		data |= (0x60 << CP_RB_WPTR_POLL_CNTL__IDLE_POLL_COUNT__SHIFT);
		WREG32(SOC15_REG_OFFSET(GC, 0, mmCP_RB_WPTR_POLL_CNTL), data);

		/* init RLC PG Delay */
		data = 0;
		data |= (0x10 << RLC_PG_DELAY__POWER_UP_DELAY__SHIFT);
		data |= (0x10 << RLC_PG_DELAY__POWER_DOWN_DELAY__SHIFT);
		data |= (0x10 << RLC_PG_DELAY__CMD_PROPAGATE_DELAY__SHIFT);
		data |= (0x40 << RLC_PG_DELAY__MEM_SLEEP_DELAY__SHIFT);
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_DELAY), data);

		data = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_DELAY_2));
		data &= ~RLC_PG_DELAY_2__SERDES_CMD_DELAY_MASK;
		data |= (0x4 << RLC_PG_DELAY_2__SERDES_CMD_DELAY__SHIFT);
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_DELAY_2), data);

		data = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_DELAY_3));
		data &= ~RLC_PG_DELAY_3__CGCG_ACTIVE_BEFORE_CGPG_MASK;
		data |= (0xff << RLC_PG_DELAY_3__CGCG_ACTIVE_BEFORE_CGPG__SHIFT);
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_DELAY_3), data);

		data = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_AUTO_PG_CTRL));
		data &= ~RLC_AUTO_PG_CTRL__GRBM_REG_SAVE_GFX_IDLE_THRESHOLD_MASK;

		/* program GRBM_REG_SAVE_GFX_IDLE_THRESHOLD to 0x55f0 */
		data |= (0x55f0 << RLC_AUTO_PG_CTRL__GRBM_REG_SAVE_GFX_IDLE_THRESHOLD__SHIFT);
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_AUTO_PG_CTRL), data);

		pwr_10_0_gfxip_control_over_cgpg(adev, true);
	}
}

static void gfx_v9_0_enable_sck_slow_down_on_power_up(struct amdgpu_device *adev,
						bool enable)
{
	uint32_t data = 0;
	uint32_t default_data = 0;

	default_data = data = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL));

	if (enable == true) {
		data |= RLC_PG_CNTL__SMU_CLK_SLOWDOWN_ON_PU_ENABLE_MASK;
		if (default_data != data)
			WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL), data);
	} else {
		data &= ~RLC_PG_CNTL__SMU_CLK_SLOWDOWN_ON_PU_ENABLE_MASK;
		if(default_data != data)
			WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL), data);
	}
}

static void gfx_v9_0_enable_sck_slow_down_on_power_down(struct amdgpu_device *adev,
						bool enable)
{
	uint32_t data = 0;
	uint32_t default_data = 0;

	default_data = data = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL));

	if (enable == true) {
		data |= RLC_PG_CNTL__SMU_CLK_SLOWDOWN_ON_PD_ENABLE_MASK;
		if(default_data != data)
			WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL), data);
	} else {
		data &= ~RLC_PG_CNTL__SMU_CLK_SLOWDOWN_ON_PD_ENABLE_MASK;
		if(default_data != data)
			WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL), data);
	}
}

static void gfx_v9_0_enable_cp_power_gating(struct amdgpu_device *adev,
					bool enable)
{
	uint32_t data = 0;
	uint32_t default_data = 0;

	default_data = data = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL));

	if (enable == true) {
		data &= ~RLC_PG_CNTL__CP_PG_DISABLE_MASK;
		if(default_data != data)
			WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL), data);
	} else {
		data |= RLC_PG_CNTL__CP_PG_DISABLE_MASK;
		if(default_data != data)
			WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL), data);
	}
}

static void gfx_v9_0_enable_gfx_cg_power_gating(struct amdgpu_device *adev,
						bool enable)
{
	uint32_t data, default_data;

	default_data = data = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL));
	if (enable == true)
		data |= RLC_PG_CNTL__GFX_POWER_GATING_ENABLE_MASK;
	else
		data &= ~RLC_PG_CNTL__GFX_POWER_GATING_ENABLE_MASK;
	if(default_data != data)
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL), data);
}

static void gfx_v9_0_enable_gfx_pipeline_powergating(struct amdgpu_device *adev,
						bool enable)
{
	uint32_t data, default_data;

	default_data = data = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL));
	if (enable == true)
		data |= RLC_PG_CNTL__GFX_PIPELINE_PG_ENABLE_MASK;
	else
		data &= ~RLC_PG_CNTL__GFX_PIPELINE_PG_ENABLE_MASK;
	if(default_data != data)
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL), data);

	if (!enable)
		/* read any GFX register to wake up GFX */
		data = RREG32(SOC15_REG_OFFSET(GC, 0, mmDB_RENDER_CONTROL));
}

static void gfx_v9_0_enable_gfx_static_mg_power_gating(struct amdgpu_device *adev,
						       bool enable)
{
	uint32_t data, default_data;

	default_data = data = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL));
	if (enable == true)
		data |= RLC_PG_CNTL__STATIC_PER_CU_PG_ENABLE_MASK;
	else
		data &= ~RLC_PG_CNTL__STATIC_PER_CU_PG_ENABLE_MASK;
	if(default_data != data)
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL), data);
}

static void gfx_v9_0_enable_gfx_dynamic_mg_power_gating(struct amdgpu_device *adev,
						bool enable)
{
	uint32_t data, default_data;

	default_data = data = RREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL));
	if (enable == true)
		data |= RLC_PG_CNTL__DYN_PER_CU_PG_ENABLE_MASK;
	else
		data &= ~RLC_PG_CNTL__DYN_PER_CU_PG_ENABLE_MASK;
	if(default_data != data)
		WREG32(SOC15_REG_OFFSET(GC, 0, mmRLC_PG_CNTL), data);
}

static void gfx_v9_0_init_pg(struct amdgpu_device *adev)
{
	if (adev->pg_flags & (AMD_PG_SUPPORT_GFX_PG |
			      AMD_PG_SUPPORT_GFX_SMG |
			      AMD_PG_SUPPORT_GFX_DMG |
			      AMD_PG_SUPPORT_CP |
			      AMD_PG_SUPPORT_GDS |
			      AMD_PG_SUPPORT_RLC_SMU_HS)) {
		gfx_v9_0_init_csb(adev);
		gfx_v9_0_init_rlc_save_restore_list(adev);
		gfx_v9_0_enable_save_restore_machine(adev);

		if (adev->asic_type == CHIP_RAVEN) {
			WREG32(mmRLC_JUMP_TABLE_RESTORE,
				adev->gfx.rlc.cp_table_gpu_addr >> 8);
			gfx_v9_0_init_gfx_power_gating(adev);

			if (adev->pg_flags & AMD_PG_SUPPORT_RLC_SMU_HS) {
				gfx_v9_0_enable_sck_slow_down_on_power_up(adev, true);
				gfx_v9_0_enable_sck_slow_down_on_power_down(adev, true);
			} else {
				gfx_v9_0_enable_sck_slow_down_on_power_up(adev, false);
				gfx_v9_0_enable_sck_slow_down_on_power_down(adev, false);
			}

			if (adev->pg_flags & AMD_PG_SUPPORT_CP)
				gfx_v9_0_enable_cp_power_gating(adev, true);
			else
				gfx_v9_0_enable_cp_power_gating(adev, false);
		}
	}
}

void gfx_v9_0_rlc_stop(struct amdgpu_device *adev)
{
	u32 tmp = RREG32_SOC15(GC, 0, mmRLC_CNTL);

	tmp = REG_SET_FIELD(tmp, RLC_CNTL, RLC_ENABLE_F32, 0);
	WREG32_SOC15(GC, 0, mmRLC_CNTL, tmp);

	gfx_v9_0_enable_gui_idle_interrupt(adev, false);

	gfx_v9_0_wait_for_rlc_serdes(adev);
}

static void gfx_v9_0_rlc_reset(struct amdgpu_device *adev)
{
	WREG32_FIELD15(GC, 0, GRBM_SOFT_RESET, SOFT_RESET_RLC, 1);
	udelay(50);
	WREG32_FIELD15(GC, 0, GRBM_SOFT_RESET, SOFT_RESET_RLC, 0);
	udelay(50);
}

static void gfx_v9_0_rlc_start(struct amdgpu_device *adev)
{
#ifdef AMDGPU_RLC_DEBUG_RETRY
	u32 rlc_ucode_ver;
#endif

	WREG32_FIELD15(GC, 0, RLC_CNTL, RLC_ENABLE_F32, 1);
	udelay(50);

	/* carrizo do enable cp interrupt after cp inited */
	if (!(adev->flags & AMD_IS_APU)) {
		gfx_v9_0_enable_gui_idle_interrupt(adev, true);
		udelay(50);
	}

#ifdef AMDGPU_RLC_DEBUG_RETRY
	/* RLC_GPM_GENERAL_6 : RLC Ucode version */
	rlc_ucode_ver = RREG32_SOC15(GC, 0, mmRLC_GPM_GENERAL_6);
	if(rlc_ucode_ver == 0x108) {
		DRM_INFO("Using rlc debug ucode. mmRLC_GPM_GENERAL_6 ==0x08%x / fw_ver == %i \n",
				rlc_ucode_ver, adev->gfx.rlc_fw_version);
		/* RLC_GPM_TIMER_INT_3 : Timer interval in RefCLK cycles,
		 * default is 0x9C4 to create a 100us interval */
		WREG32_SOC15(GC, 0, mmRLC_GPM_TIMER_INT_3, 0x9C4);
		/* RLC_GPM_GENERAL_12 : Minimum gap between wptr and rptr
		 * to disable the page fault retry interrupts, default is
		 * 0x100 (256) */
		WREG32_SOC15(GC, 0, mmRLC_GPM_GENERAL_12, 0x100);
	}
#endif
}

static int gfx_v9_0_rlc_load_microcode(struct amdgpu_device *adev)
{
	const struct rlc_firmware_header_v2_0 *hdr;
	const __le32 *fw_data;
	unsigned i, fw_size;

	if (!adev->gfx.rlc_fw)
		return -EINVAL;

	hdr = (const struct rlc_firmware_header_v2_0 *)adev->gfx.rlc_fw->data;
	amdgpu_ucode_print_rlc_hdr(&hdr->header);

	fw_data = (const __le32 *)(adev->gfx.rlc_fw->data +
			   le32_to_cpu(hdr->header.ucode_array_offset_bytes));
	fw_size = le32_to_cpu(hdr->header.ucode_size_bytes) / 4;

	WREG32_SOC15(GC, 0, mmRLC_GPM_UCODE_ADDR,
			RLCG_UCODE_LOADING_START_ADDRESS);
	for (i = 0; i < fw_size; i++)
		WREG32_SOC15(GC, 0, mmRLC_GPM_UCODE_DATA, le32_to_cpup(fw_data++));
	WREG32_SOC15(GC, 0, mmRLC_GPM_UCODE_ADDR, adev->gfx.rlc_fw_version);

	return 0;
}

static int gfx_v9_0_rlc_resume(struct amdgpu_device *adev)
{
	int r;

	if (amdgpu_sriov_vf(adev))
		return 0;

	gfx_v9_0_rlc_stop(adev);

	/* disable CG */
	WREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL, 0);

	/* disable PG */
	WREG32_SOC15(GC, 0, mmRLC_PG_CNTL, 0);

	gfx_v9_0_rlc_reset(adev);

	gfx_v9_0_init_pg(adev);

	if (adev->firmware.load_type != AMDGPU_FW_LOAD_PSP) {
		/* legacy rlc firmware loading */
		r = gfx_v9_0_rlc_load_microcode(adev);
		if (r)
			return r;
	}

	if (adev->asic_type == CHIP_RAVEN) {
		if (amdgpu_lbpw != 0)
			gfx_v9_0_enable_lbpw(adev, true);
		else
			gfx_v9_0_enable_lbpw(adev, false);
	}

	gfx_v9_0_rlc_start(adev);

	return 0;
}

static void gfx_v9_0_cp_gfx_enable(struct amdgpu_device *adev, bool enable)
{
	int i;
	u32 tmp = RREG32_SOC15(GC, 0, mmCP_ME_CNTL);

	tmp = REG_SET_FIELD(tmp, CP_ME_CNTL, ME_HALT, enable ? 0 : 1);
	tmp = REG_SET_FIELD(tmp, CP_ME_CNTL, PFP_HALT, enable ? 0 : 1);
	tmp = REG_SET_FIELD(tmp, CP_ME_CNTL, CE_HALT, enable ? 0 : 1);
	if (!enable) {
		for (i = 0; i < adev->gfx.num_gfx_rings; i++)
			adev->gfx.gfx_ring[i].ready = false;
	}
	WREG32_SOC15(GC, 0, mmCP_ME_CNTL, tmp);
	udelay(50);
}

static int gfx_v9_0_cp_gfx_load_microcode(struct amdgpu_device *adev)
{
	const struct gfx_firmware_header_v1_0 *pfp_hdr;
	const struct gfx_firmware_header_v1_0 *ce_hdr;
	const struct gfx_firmware_header_v1_0 *me_hdr;
	const __le32 *fw_data;
	unsigned i, fw_size;

	if (!adev->gfx.me_fw || !adev->gfx.pfp_fw || !adev->gfx.ce_fw)
		return -EINVAL;

	pfp_hdr = (const struct gfx_firmware_header_v1_0 *)
		adev->gfx.pfp_fw->data;
	ce_hdr = (const struct gfx_firmware_header_v1_0 *)
		adev->gfx.ce_fw->data;
	me_hdr = (const struct gfx_firmware_header_v1_0 *)
		adev->gfx.me_fw->data;

	amdgpu_ucode_print_gfx_hdr(&pfp_hdr->header);
	amdgpu_ucode_print_gfx_hdr(&ce_hdr->header);
	amdgpu_ucode_print_gfx_hdr(&me_hdr->header);

	gfx_v9_0_cp_gfx_enable(adev, false);

	/* PFP */
	fw_data = (const __le32 *)
		(adev->gfx.pfp_fw->data +
		 le32_to_cpu(pfp_hdr->header.ucode_array_offset_bytes));
	fw_size = le32_to_cpu(pfp_hdr->header.ucode_size_bytes) / 4;
	WREG32_SOC15(GC, 0, mmCP_PFP_UCODE_ADDR, 0);
	for (i = 0; i < fw_size; i++)
		WREG32_SOC15(GC, 0, mmCP_PFP_UCODE_DATA, le32_to_cpup(fw_data++));
	WREG32_SOC15(GC, 0, mmCP_PFP_UCODE_ADDR, adev->gfx.pfp_fw_version);

	/* CE */
	fw_data = (const __le32 *)
		(adev->gfx.ce_fw->data +
		 le32_to_cpu(ce_hdr->header.ucode_array_offset_bytes));
	fw_size = le32_to_cpu(ce_hdr->header.ucode_size_bytes) / 4;
	WREG32_SOC15(GC, 0, mmCP_CE_UCODE_ADDR, 0);
	for (i = 0; i < fw_size; i++)
		WREG32_SOC15(GC, 0, mmCP_CE_UCODE_DATA, le32_to_cpup(fw_data++));
	WREG32_SOC15(GC, 0, mmCP_CE_UCODE_ADDR, adev->gfx.ce_fw_version);

	/* ME */
	fw_data = (const __le32 *)
		(adev->gfx.me_fw->data +
		 le32_to_cpu(me_hdr->header.ucode_array_offset_bytes));
	fw_size = le32_to_cpu(me_hdr->header.ucode_size_bytes) / 4;
	WREG32_SOC15(GC, 0, mmCP_ME_RAM_WADDR, 0);
	for (i = 0; i < fw_size; i++)
		WREG32_SOC15(GC, 0, mmCP_ME_RAM_DATA, le32_to_cpup(fw_data++));
	WREG32_SOC15(GC, 0, mmCP_ME_RAM_WADDR, adev->gfx.me_fw_version);

	return 0;
}

static int gfx_v9_0_cp_gfx_start(struct amdgpu_device *adev)
{
	struct amdgpu_ring *ring = &adev->gfx.gfx_ring[0];
	const struct cs_section_def *sect = NULL;
	const struct cs_extent_def *ext = NULL;
	int r, i, tmp;

	/* init the CP */
	WREG32_SOC15(GC, 0, mmCP_MAX_CONTEXT, adev->gfx.config.max_hw_contexts - 1);
	WREG32_SOC15(GC, 0, mmCP_DEVICE_ID, 1);

	gfx_v9_0_cp_gfx_enable(adev, true);

	r = amdgpu_ring_alloc(ring, gfx_v9_0_get_csb_size(adev) + 4 + 3);
	if (r) {
		DRM_ERROR("amdgpu: cp failed to lock ring (%d).\n", r);
		return r;
	}

	amdgpu_ring_write(ring, PACKET3(PACKET3_PREAMBLE_CNTL, 0));
	amdgpu_ring_write(ring, PACKET3_PREAMBLE_BEGIN_CLEAR_STATE);

	amdgpu_ring_write(ring, PACKET3(PACKET3_CONTEXT_CONTROL, 1));
	amdgpu_ring_write(ring, 0x80000000);
	amdgpu_ring_write(ring, 0x80000000);

	for (sect = gfx9_cs_data; sect->section != NULL; ++sect) {
		for (ext = sect->section; ext->extent != NULL; ++ext) {
			if (sect->id == SECT_CONTEXT) {
				amdgpu_ring_write(ring,
				       PACKET3(PACKET3_SET_CONTEXT_REG,
					       ext->reg_count));
				amdgpu_ring_write(ring,
				       ext->reg_index - PACKET3_SET_CONTEXT_REG_START);
				for (i = 0; i < ext->reg_count; i++)
					amdgpu_ring_write(ring, ext->extent[i]);
			}
		}
	}

	amdgpu_ring_write(ring, PACKET3(PACKET3_PREAMBLE_CNTL, 0));
	amdgpu_ring_write(ring, PACKET3_PREAMBLE_END_CLEAR_STATE);

	amdgpu_ring_write(ring, PACKET3(PACKET3_CLEAR_STATE, 0));
	amdgpu_ring_write(ring, 0);

	amdgpu_ring_write(ring, PACKET3(PACKET3_SET_BASE, 2));
	amdgpu_ring_write(ring, PACKET3_BASE_INDEX(CE_PARTITION_BASE));
	amdgpu_ring_write(ring, 0x8000);
	amdgpu_ring_write(ring, 0x8000);

	amdgpu_ring_write(ring, PACKET3(PACKET3_SET_UCONFIG_REG,1));
	tmp = (PACKET3_SET_UCONFIG_REG_INDEX_TYPE |
		(SOC15_REG_OFFSET(GC, 0, mmVGT_INDEX_TYPE) - PACKET3_SET_UCONFIG_REG_START));
	amdgpu_ring_write(ring, tmp);
	amdgpu_ring_write(ring, 0);

	amdgpu_ring_commit(ring);

	return 0;
}

static int gfx_v9_0_cp_gfx_resume(struct amdgpu_device *adev)
{
	struct amdgpu_ring *ring;
	u32 tmp;
	u32 rb_bufsz;
	u64 rb_addr, rptr_addr, wptr_gpu_addr;

	/* Set the write pointer delay */
	WREG32_SOC15(GC, 0, mmCP_RB_WPTR_DELAY, 0);

	/* set the RB to use vmid 0 */
	WREG32_SOC15(GC, 0, mmCP_RB_VMID, 0);

	/* Set ring buffer size */
	ring = &adev->gfx.gfx_ring[0];
	rb_bufsz = order_base_2(ring->ring_size / 8);
	tmp = REG_SET_FIELD(0, CP_RB0_CNTL, RB_BUFSZ, rb_bufsz);
	tmp = REG_SET_FIELD(tmp, CP_RB0_CNTL, RB_BLKSZ, rb_bufsz - 2);
#ifdef __BIG_ENDIAN
	tmp = REG_SET_FIELD(tmp, CP_RB0_CNTL, BUF_SWAP, 1);
#endif
	WREG32_SOC15(GC, 0, mmCP_RB0_CNTL, tmp);

	/* Initialize the ring buffer's write pointers */
	ring->wptr = 0;
	WREG32_SOC15(GC, 0, mmCP_RB0_WPTR, lower_32_bits(ring->wptr));
	WREG32_SOC15(GC, 0, mmCP_RB0_WPTR_HI, upper_32_bits(ring->wptr));

	/* set the wb address wether it's enabled or not */
	rptr_addr = adev->wb.gpu_addr + (ring->rptr_offs * 4);
	WREG32_SOC15(GC, 0, mmCP_RB0_RPTR_ADDR, lower_32_bits(rptr_addr));
	WREG32_SOC15(GC, 0, mmCP_RB0_RPTR_ADDR_HI, upper_32_bits(rptr_addr) & CP_RB_RPTR_ADDR_HI__RB_RPTR_ADDR_HI_MASK);

	wptr_gpu_addr = adev->wb.gpu_addr + (ring->wptr_offs * 4);
	WREG32_SOC15(GC, 0, mmCP_RB_WPTR_POLL_ADDR_LO, lower_32_bits(wptr_gpu_addr));
	WREG32_SOC15(GC, 0, mmCP_RB_WPTR_POLL_ADDR_HI, upper_32_bits(wptr_gpu_addr));

	mdelay(1);
	WREG32_SOC15(GC, 0, mmCP_RB0_CNTL, tmp);

	rb_addr = ring->gpu_addr >> 8;
	WREG32_SOC15(GC, 0, mmCP_RB0_BASE, rb_addr);
	WREG32_SOC15(GC, 0, mmCP_RB0_BASE_HI, upper_32_bits(rb_addr));

	tmp = RREG32_SOC15(GC, 0, mmCP_RB_DOORBELL_CONTROL);
	if (ring->use_doorbell) {
		tmp = REG_SET_FIELD(tmp, CP_RB_DOORBELL_CONTROL,
				    DOORBELL_OFFSET, ring->doorbell_index);
		tmp = REG_SET_FIELD(tmp, CP_RB_DOORBELL_CONTROL,
				    DOORBELL_EN, 1);
	} else {
		tmp = REG_SET_FIELD(tmp, CP_RB_DOORBELL_CONTROL, DOORBELL_EN, 0);
	}
	WREG32_SOC15(GC, 0, mmCP_RB_DOORBELL_CONTROL, tmp);

	tmp = REG_SET_FIELD(0, CP_RB_DOORBELL_RANGE_LOWER,
			DOORBELL_RANGE_LOWER, ring->doorbell_index);
	WREG32_SOC15(GC, 0, mmCP_RB_DOORBELL_RANGE_LOWER, tmp);

	WREG32_SOC15(GC, 0, mmCP_RB_DOORBELL_RANGE_UPPER,
		       CP_RB_DOORBELL_RANGE_UPPER__DOORBELL_RANGE_UPPER_MASK);


	/* start the ring */
	gfx_v9_0_cp_gfx_start(adev);
	ring->ready = true;

	return 0;
}

static void gfx_v9_0_cp_compute_enable(struct amdgpu_device *adev, bool enable)
{
	int i;

	if (enable) {
		WREG32_SOC15(GC, 0, mmCP_MEC_CNTL, 0);
	} else {
		WREG32_SOC15(GC, 0, mmCP_MEC_CNTL,
			(CP_MEC_CNTL__MEC_ME1_HALT_MASK | CP_MEC_CNTL__MEC_ME2_HALT_MASK));
		for (i = 0; i < adev->gfx.num_compute_rings; i++)
			adev->gfx.compute_ring[i].ready = false;
		adev->gfx.kiq.ring.ready = false;
	}
	udelay(50);
}

static int gfx_v9_0_cp_compute_load_microcode(struct amdgpu_device *adev)
{
	const struct gfx_firmware_header_v1_0 *mec_hdr;
	const __le32 *fw_data;
	unsigned i;
	u32 tmp;

	if (!adev->gfx.mec_fw)
		return -EINVAL;

	gfx_v9_0_cp_compute_enable(adev, false);

	mec_hdr = (const struct gfx_firmware_header_v1_0 *)adev->gfx.mec_fw->data;
	amdgpu_ucode_print_gfx_hdr(&mec_hdr->header);

	fw_data = (const __le32 *)
		(adev->gfx.mec_fw->data +
		 le32_to_cpu(mec_hdr->header.ucode_array_offset_bytes));
	tmp = 0;
	tmp = REG_SET_FIELD(tmp, CP_CPC_IC_BASE_CNTL, VMID, 0);
	tmp = REG_SET_FIELD(tmp, CP_CPC_IC_BASE_CNTL, CACHE_POLICY, 0);
	WREG32_SOC15(GC, 0, mmCP_CPC_IC_BASE_CNTL, tmp);

	WREG32_SOC15(GC, 0, mmCP_CPC_IC_BASE_LO,
		adev->gfx.mec.mec_fw_gpu_addr & 0xFFFFF000);
	WREG32_SOC15(GC, 0, mmCP_CPC_IC_BASE_HI,
		upper_32_bits(adev->gfx.mec.mec_fw_gpu_addr));

	/* MEC1 */
	WREG32_SOC15(GC, 0, mmCP_MEC_ME1_UCODE_ADDR,
			 mec_hdr->jt_offset);
	for (i = 0; i < mec_hdr->jt_size; i++)
		WREG32_SOC15(GC, 0, mmCP_MEC_ME1_UCODE_DATA,
			le32_to_cpup(fw_data + mec_hdr->jt_offset + i));

	WREG32_SOC15(GC, 0, mmCP_MEC_ME1_UCODE_ADDR,
			adev->gfx.mec_fw_version);
	/* Todo : Loading MEC2 firmware is only necessary if MEC2 should run different microcode than MEC1. */

	return 0;
}

/* KIQ functions */
static void gfx_v9_0_kiq_setting(struct amdgpu_ring *ring)
{
	uint32_t tmp;
	struct amdgpu_device *adev = ring->adev;

	/* tell RLC which is KIQ queue */
	tmp = RREG32_SOC15(GC, 0, mmRLC_CP_SCHEDULERS);
	tmp &= 0xffffff00;
	tmp |= (ring->me << 5) | (ring->pipe << 3) | (ring->queue);
	WREG32_SOC15(GC, 0, mmRLC_CP_SCHEDULERS, tmp);
	tmp |= 0x80;
	WREG32_SOC15(GC, 0, mmRLC_CP_SCHEDULERS, tmp);
}

static int gfx_v9_0_kiq_kcq_enable(struct amdgpu_device *adev)
{
	struct amdgpu_ring *kiq_ring = &adev->gfx.kiq.ring;
	uint32_t scratch, tmp = 0;
	uint64_t queue_mask = 0;
	int r, i;

	for (i = 0; i < AMDGPU_MAX_COMPUTE_QUEUES; ++i) {
		if (!test_bit(i, adev->gfx.mec.queue_bitmap))
			continue;

		/* This situation may be hit in the future if a new HW
		 * generation exposes more than 64 queues. If so, the
		 * definition of queue_mask needs updating */
		if (WARN_ON(i >= (sizeof(queue_mask)*8))) {
			DRM_ERROR("Invalid KCQ enabled: %d\n", i);
			break;
		}

		queue_mask |= (1ull << i);
	}

	r = amdgpu_gfx_scratch_get(adev, &scratch);
	if (r) {
		DRM_ERROR("Failed to get scratch reg (%d).\n", r);
		return r;
	}
	WREG32(scratch, 0xCAFEDEAD);

	r = amdgpu_ring_alloc(kiq_ring, (7 * adev->gfx.num_compute_rings) + 11);
	if (r) {
		DRM_ERROR("Failed to lock KIQ (%d).\n", r);
		amdgpu_gfx_scratch_free(adev, scratch);
		return r;
	}

	/* set resources */
	amdgpu_ring_write(kiq_ring, PACKET3(PACKET3_SET_RESOURCES, 6));
	amdgpu_ring_write(kiq_ring, PACKET3_SET_RESOURCES_VMID_MASK(0) |
			  PACKET3_SET_RESOURCES_QUEUE_TYPE(0));	/* vmid_mask:0 queue_type:0 (KIQ) */
	amdgpu_ring_write(kiq_ring, lower_32_bits(queue_mask));	/* queue mask lo */
	amdgpu_ring_write(kiq_ring, upper_32_bits(queue_mask));	/* queue mask hi */
	amdgpu_ring_write(kiq_ring, 0);	/* gws mask lo */
	amdgpu_ring_write(kiq_ring, 0);	/* gws mask hi */
	amdgpu_ring_write(kiq_ring, 0);	/* oac mask */
	amdgpu_ring_write(kiq_ring, 0);	/* gds heap base:0, gds heap size:0 */
	for (i = 0; i < adev->gfx.num_compute_rings; i++) {
		struct amdgpu_ring *ring = &adev->gfx.compute_ring[i];
		uint64_t mqd_addr = amdgpu_bo_gpu_offset(ring->mqd_obj);
		uint64_t wptr_addr = adev->wb.gpu_addr + (ring->wptr_offs * 4);

		amdgpu_ring_write(kiq_ring, PACKET3(PACKET3_MAP_QUEUES, 5));
		/* Q_sel:0, vmid:0, vidmem: 1, engine:0, num_Q:1*/
		amdgpu_ring_write(kiq_ring, /* Q_sel: 0, vmid: 0, engine: 0, num_Q: 1 */
				  PACKET3_MAP_QUEUES_QUEUE_SEL(0) | /* Queue_Sel */
				  PACKET3_MAP_QUEUES_VMID(0) | /* VMID */
				  PACKET3_MAP_QUEUES_QUEUE(ring->queue) |
				  PACKET3_MAP_QUEUES_PIPE(ring->pipe) |
				  PACKET3_MAP_QUEUES_ME((ring->me == 1 ? 0 : 1)) |
				  PACKET3_MAP_QUEUES_QUEUE_TYPE(0) | /*queue_type: normal compute queue */
				  PACKET3_MAP_QUEUES_ALLOC_FORMAT(1) | /* alloc format: all_on_one_pipe */
				  PACKET3_MAP_QUEUES_ENGINE_SEL(0) | /* engine_sel: compute */
				  PACKET3_MAP_QUEUES_NUM_QUEUES(1)); /* num_queues: must be 1 */
		amdgpu_ring_write(kiq_ring, PACKET3_MAP_QUEUES_DOORBELL_OFFSET(ring->doorbell_index));
		amdgpu_ring_write(kiq_ring, lower_32_bits(mqd_addr));
		amdgpu_ring_write(kiq_ring, upper_32_bits(mqd_addr));
		amdgpu_ring_write(kiq_ring, lower_32_bits(wptr_addr));
		amdgpu_ring_write(kiq_ring, upper_32_bits(wptr_addr));
	}
	/* write to scratch for completion */
	amdgpu_ring_write(kiq_ring, PACKET3(PACKET3_SET_UCONFIG_REG, 1));
	amdgpu_ring_write(kiq_ring, (scratch - PACKET3_SET_UCONFIG_REG_START));
	amdgpu_ring_write(kiq_ring, 0xDEADBEEF);
	amdgpu_ring_commit(kiq_ring);

	for (i = 0; i < adev->usec_timeout; i++) {
		tmp = RREG32(scratch);
		if (tmp == 0xDEADBEEF)
			break;
		DRM_UDELAY(1);
	}
	if (i >= adev->usec_timeout) {
		DRM_ERROR("KCQ enable failed (scratch(0x%04X)=0x%08X)\n",
			  scratch, tmp);
		r = -EINVAL;
	}
	amdgpu_gfx_scratch_free(adev, scratch);

	return r;
}

static int gfx_v9_0_mqd_init(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;
	struct v9_mqd *mqd = ring->mqd_ptr;
	uint64_t hqd_gpu_addr, wb_gpu_addr, eop_base_addr;
	uint32_t tmp;

	mqd->header = 0xC0310800;
	mqd->compute_pipelinestat_enable = 0x00000001;
	mqd->compute_static_thread_mgmt_se0 = 0xffffffff;
	mqd->compute_static_thread_mgmt_se1 = 0xffffffff;
	mqd->compute_static_thread_mgmt_se2 = 0xffffffff;
	mqd->compute_static_thread_mgmt_se3 = 0xffffffff;
	mqd->compute_misc_reserved = 0x00000003;

	eop_base_addr = ring->eop_gpu_addr >> 8;
	mqd->cp_hqd_eop_base_addr_lo = eop_base_addr;
	mqd->cp_hqd_eop_base_addr_hi = upper_32_bits(eop_base_addr);

	/* set the EOP size, register value is 2^(EOP_SIZE+1) dwords */
	tmp = RREG32_SOC15(GC, 0, mmCP_HQD_EOP_CONTROL);
	tmp = REG_SET_FIELD(tmp, CP_HQD_EOP_CONTROL, EOP_SIZE,
			(order_base_2(GFX9_MEC_HPD_SIZE / 4) - 1));

	mqd->cp_hqd_eop_control = tmp;

	/* enable doorbell? */
	tmp = RREG32_SOC15(GC, 0, mmCP_HQD_PQ_DOORBELL_CONTROL);

	if (ring->use_doorbell) {
		tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_DOORBELL_CONTROL,
				    DOORBELL_OFFSET, ring->doorbell_index);
		tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_DOORBELL_CONTROL,
				    DOORBELL_EN, 1);
		tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_DOORBELL_CONTROL,
				    DOORBELL_SOURCE, 0);
		tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_DOORBELL_CONTROL,
				    DOORBELL_HIT, 0);
	}
	else
		tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_DOORBELL_CONTROL,
					 DOORBELL_EN, 0);

	mqd->cp_hqd_pq_doorbell_control = tmp;

	/* disable the queue if it's active */
	ring->wptr = 0;
	mqd->cp_hqd_dequeue_request = 0;
	mqd->cp_hqd_pq_rptr = 0;
	mqd->cp_hqd_pq_wptr_lo = 0;
	mqd->cp_hqd_pq_wptr_hi = 0;

	/* set the pointer to the MQD */
	mqd->cp_mqd_base_addr_lo = ring->mqd_gpu_addr & 0xfffffffc;
	mqd->cp_mqd_base_addr_hi = upper_32_bits(ring->mqd_gpu_addr);

	/* set MQD vmid to 0 */
	tmp = RREG32_SOC15(GC, 0, mmCP_MQD_CONTROL);
	tmp = REG_SET_FIELD(tmp, CP_MQD_CONTROL, VMID, 0);
	mqd->cp_mqd_control = tmp;

	/* set the pointer to the HQD, this is similar CP_RB0_BASE/_HI */
	hqd_gpu_addr = ring->gpu_addr >> 8;
	mqd->cp_hqd_pq_base_lo = hqd_gpu_addr;
	mqd->cp_hqd_pq_base_hi = upper_32_bits(hqd_gpu_addr);

	/* set up the HQD, this is similar to CP_RB0_CNTL */
	tmp = RREG32_SOC15(GC, 0, mmCP_HQD_PQ_CONTROL);
	tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_CONTROL, QUEUE_SIZE,
			    (order_base_2(ring->ring_size / 4) - 1));
	tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_CONTROL, RPTR_BLOCK_SIZE,
			((order_base_2(AMDGPU_GPU_PAGE_SIZE / 4) - 1) << 8));
#ifdef __BIG_ENDIAN
	tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_CONTROL, ENDIAN_SWAP, 1);
#endif
	tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_CONTROL, UNORD_DISPATCH, 0);
	tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_CONTROL, ROQ_PQ_IB_FLIP, 0);
	tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_CONTROL, PRIV_STATE, 1);
	tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_CONTROL, KMD_QUEUE, 1);
	mqd->cp_hqd_pq_control = tmp;

	/* set the wb address whether it's enabled or not */
	wb_gpu_addr = adev->wb.gpu_addr + (ring->rptr_offs * 4);
	mqd->cp_hqd_pq_rptr_report_addr_lo = wb_gpu_addr & 0xfffffffc;
	mqd->cp_hqd_pq_rptr_report_addr_hi =
		upper_32_bits(wb_gpu_addr) & 0xffff;

	/* only used if CP_PQ_WPTR_POLL_CNTL.CP_PQ_WPTR_POLL_CNTL__EN_MASK=1 */
	wb_gpu_addr = adev->wb.gpu_addr + (ring->wptr_offs * 4);
	mqd->cp_hqd_pq_wptr_poll_addr_lo = wb_gpu_addr & 0xfffffffc;
	mqd->cp_hqd_pq_wptr_poll_addr_hi = upper_32_bits(wb_gpu_addr) & 0xffff;

	tmp = 0;
	/* enable the doorbell if requested */
	if (ring->use_doorbell) {
		tmp = RREG32_SOC15(GC, 0, mmCP_HQD_PQ_DOORBELL_CONTROL);
		tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_DOORBELL_CONTROL,
				DOORBELL_OFFSET, ring->doorbell_index);

		tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_DOORBELL_CONTROL,
					 DOORBELL_EN, 1);
		tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_DOORBELL_CONTROL,
					 DOORBELL_SOURCE, 0);
		tmp = REG_SET_FIELD(tmp, CP_HQD_PQ_DOORBELL_CONTROL,
					 DOORBELL_HIT, 0);
	}

	mqd->cp_hqd_pq_doorbell_control = tmp;

	/* reset read and write pointers, similar to CP_RB0_WPTR/_RPTR */
	ring->wptr = 0;
	mqd->cp_hqd_pq_rptr = RREG32_SOC15(GC, 0, mmCP_HQD_PQ_RPTR);

	/* set the vmid for the queue */
	mqd->cp_hqd_vmid = 0;

	tmp = RREG32_SOC15(GC, 0, mmCP_HQD_PERSISTENT_STATE);
	tmp = REG_SET_FIELD(tmp, CP_HQD_PERSISTENT_STATE, PRELOAD_SIZE, 0x53);
	mqd->cp_hqd_persistent_state = tmp;

	/* set MIN_IB_AVAIL_SIZE */
	tmp = RREG32_SOC15(GC, 0, mmCP_HQD_IB_CONTROL);
	tmp = REG_SET_FIELD(tmp, CP_HQD_IB_CONTROL, MIN_IB_AVAIL_SIZE, 3);
	mqd->cp_hqd_ib_control = tmp;

	/* activate the queue */
	mqd->cp_hqd_active = 1;

	return 0;
}

static int gfx_v9_0_kiq_init_register(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;
	struct v9_mqd *mqd = ring->mqd_ptr;
	int j;

	/* disable wptr polling */
	WREG32_FIELD15(GC, 0, CP_PQ_WPTR_POLL_CNTL, EN, 0);

	WREG32_SOC15(GC, 0, mmCP_HQD_EOP_BASE_ADDR,
	       mqd->cp_hqd_eop_base_addr_lo);
	WREG32_SOC15(GC, 0, mmCP_HQD_EOP_BASE_ADDR_HI,
	       mqd->cp_hqd_eop_base_addr_hi);

	/* set the EOP size, register value is 2^(EOP_SIZE+1) dwords */
	WREG32_SOC15(GC, 0, mmCP_HQD_EOP_CONTROL,
	       mqd->cp_hqd_eop_control);

	/* enable doorbell? */
	WREG32_SOC15(GC, 0, mmCP_HQD_PQ_DOORBELL_CONTROL,
	       mqd->cp_hqd_pq_doorbell_control);

	/* disable the queue if it's active */
	if (RREG32_SOC15(GC, 0, mmCP_HQD_ACTIVE) & 1) {
		WREG32_SOC15(GC, 0, mmCP_HQD_DEQUEUE_REQUEST, 1);
		for (j = 0; j < adev->usec_timeout; j++) {
			if (!(RREG32_SOC15(GC, 0, mmCP_HQD_ACTIVE) & 1))
				break;
			udelay(1);
		}
		WREG32_SOC15(GC, 0, mmCP_HQD_DEQUEUE_REQUEST,
		       mqd->cp_hqd_dequeue_request);
		WREG32_SOC15(GC, 0, mmCP_HQD_PQ_RPTR,
		       mqd->cp_hqd_pq_rptr);
		WREG32_SOC15(GC, 0, mmCP_HQD_PQ_WPTR_LO,
		       mqd->cp_hqd_pq_wptr_lo);
		WREG32_SOC15(GC, 0, mmCP_HQD_PQ_WPTR_HI,
		       mqd->cp_hqd_pq_wptr_hi);
	}

	/* set the pointer to the MQD */
	WREG32_SOC15(GC, 0, mmCP_MQD_BASE_ADDR,
	       mqd->cp_mqd_base_addr_lo);
	WREG32_SOC15(GC, 0, mmCP_MQD_BASE_ADDR_HI,
	       mqd->cp_mqd_base_addr_hi);

	/* set MQD vmid to 0 */
	WREG32_SOC15(GC, 0, mmCP_MQD_CONTROL,
	       mqd->cp_mqd_control);

	/* set the pointer to the HQD, this is similar CP_RB0_BASE/_HI */
	WREG32_SOC15(GC, 0, mmCP_HQD_PQ_BASE,
	       mqd->cp_hqd_pq_base_lo);
	WREG32_SOC15(GC, 0, mmCP_HQD_PQ_BASE_HI,
	       mqd->cp_hqd_pq_base_hi);

	/* set up the HQD, this is similar to CP_RB0_CNTL */
	WREG32_SOC15(GC, 0, mmCP_HQD_PQ_CONTROL,
	       mqd->cp_hqd_pq_control);

	/* set the wb address whether it's enabled or not */
	WREG32_SOC15(GC, 0, mmCP_HQD_PQ_RPTR_REPORT_ADDR,
				mqd->cp_hqd_pq_rptr_report_addr_lo);
	WREG32_SOC15(GC, 0, mmCP_HQD_PQ_RPTR_REPORT_ADDR_HI,
				mqd->cp_hqd_pq_rptr_report_addr_hi);

	/* only used if CP_PQ_WPTR_POLL_CNTL.CP_PQ_WPTR_POLL_CNTL__EN_MASK=1 */
	WREG32_SOC15(GC, 0, mmCP_HQD_PQ_WPTR_POLL_ADDR,
	       mqd->cp_hqd_pq_wptr_poll_addr_lo);
	WREG32_SOC15(GC, 0, mmCP_HQD_PQ_WPTR_POLL_ADDR_HI,
	       mqd->cp_hqd_pq_wptr_poll_addr_hi);

	/* enable the doorbell if requested */
	if (ring->use_doorbell) {
		WREG32_SOC15(GC, 0, mmCP_MEC_DOORBELL_RANGE_LOWER,
					(AMDGPU_DOORBELL64_KIQ *2) << 2);
		WREG32_SOC15(GC, 0, mmCP_MEC_DOORBELL_RANGE_UPPER,
					(AMDGPU_DOORBELL64_USERQUEUE_END * 2) << 2);
	}

	WREG32_SOC15(GC, 0, mmCP_HQD_PQ_DOORBELL_CONTROL,
	       mqd->cp_hqd_pq_doorbell_control);

	/* reset read and write pointers, similar to CP_RB0_WPTR/_RPTR */
	WREG32_SOC15(GC, 0, mmCP_HQD_PQ_WPTR_LO,
	       mqd->cp_hqd_pq_wptr_lo);
	WREG32_SOC15(GC, 0, mmCP_HQD_PQ_WPTR_HI,
	       mqd->cp_hqd_pq_wptr_hi);

	/* set the vmid for the queue */
	WREG32_SOC15(GC, 0, mmCP_HQD_VMID, mqd->cp_hqd_vmid);

	WREG32_SOC15(GC, 0, mmCP_HQD_PERSISTENT_STATE,
	       mqd->cp_hqd_persistent_state);

	/* activate the queue */
	WREG32_SOC15(GC, 0, mmCP_HQD_ACTIVE,
	       mqd->cp_hqd_active);

	if (ring->use_doorbell)
		WREG32_FIELD15(GC, 0, CP_PQ_STATUS, DOORBELL_ENABLE, 1);

	return 0;
}

static int gfx_v9_0_kiq_init_queue(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;
	struct v9_mqd *mqd = ring->mqd_ptr;
	int mqd_idx = AMDGPU_MAX_COMPUTE_RINGS;

	gfx_v9_0_kiq_setting(ring);

	if (adev->gfx.in_reset) { /* for GPU_RESET case */
		/* reset MQD to a clean status */
		if (adev->gfx.mec.mqd_backup[mqd_idx])
			memcpy(mqd, adev->gfx.mec.mqd_backup[mqd_idx], sizeof(*mqd));

		/* reset ring buffer */
		ring->wptr = 0;
		amdgpu_ring_clear_ring(ring);

		mutex_lock(&adev->srbm_mutex);
		soc15_grbm_select(adev, ring->me, ring->pipe, ring->queue, 0);
		gfx_v9_0_kiq_init_register(ring);
		soc15_grbm_select(adev, 0, 0, 0, 0);
		mutex_unlock(&adev->srbm_mutex);
	} else {
		memset((void *)mqd, 0, sizeof(*mqd));
		mutex_lock(&adev->srbm_mutex);
		soc15_grbm_select(adev, ring->me, ring->pipe, ring->queue, 0);
		gfx_v9_0_mqd_init(ring);
		gfx_v9_0_kiq_init_register(ring);
		soc15_grbm_select(adev, 0, 0, 0, 0);
		mutex_unlock(&adev->srbm_mutex);

		if (adev->gfx.mec.mqd_backup[mqd_idx])
			memcpy(adev->gfx.mec.mqd_backup[mqd_idx], mqd, sizeof(*mqd));
	}

	return 0;
}

static int gfx_v9_0_kcq_init_queue(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;
	struct v9_mqd *mqd = ring->mqd_ptr;
	int mqd_idx = ring - &adev->gfx.compute_ring[0];

	if (!adev->gfx.in_reset && !adev->gfx.in_suspend) {
		memset((void *)mqd, 0, sizeof(*mqd));
		mutex_lock(&adev->srbm_mutex);
		soc15_grbm_select(adev, ring->me, ring->pipe, ring->queue, 0);
		gfx_v9_0_mqd_init(ring);
		soc15_grbm_select(adev, 0, 0, 0, 0);
		mutex_unlock(&adev->srbm_mutex);

		if (adev->gfx.mec.mqd_backup[mqd_idx])
			memcpy(adev->gfx.mec.mqd_backup[mqd_idx], mqd, sizeof(*mqd));
	} else if (adev->gfx.in_reset) { /* for GPU_RESET case */
		/* reset MQD to a clean status */
		if (adev->gfx.mec.mqd_backup[mqd_idx])
			memcpy(mqd, adev->gfx.mec.mqd_backup[mqd_idx], sizeof(*mqd));

		/* reset ring buffer */
		ring->wptr = 0;
		amdgpu_ring_clear_ring(ring);
	} else {
		amdgpu_ring_clear_ring(ring);
	}

	return 0;
}

static int gfx_v9_0_kiq_resume(struct amdgpu_device *adev)
{
	struct amdgpu_ring *ring = NULL;
	int r = 0, i;

	gfx_v9_0_cp_compute_enable(adev, true);

	ring = &adev->gfx.kiq.ring;

	r = amdgpu_bo_reserve(ring->mqd_obj, false);
	if (unlikely(r != 0))
		goto done;

	r = amdgpu_bo_kmap(ring->mqd_obj, (void **)&ring->mqd_ptr);
	if (!r) {
		r = gfx_v9_0_kiq_init_queue(ring);
		amdgpu_bo_kunmap(ring->mqd_obj);
		ring->mqd_ptr = NULL;
	}
	amdgpu_bo_unreserve(ring->mqd_obj);
	if (r)
		goto done;

	for (i = 0; i < adev->gfx.num_compute_rings; i++) {
		ring = &adev->gfx.compute_ring[i];

		r = amdgpu_bo_reserve(ring->mqd_obj, false);
		if (unlikely(r != 0))
			goto done;
		r = amdgpu_bo_kmap(ring->mqd_obj, (void **)&ring->mqd_ptr);
		if (!r) {
			r = gfx_v9_0_kcq_init_queue(ring);
			amdgpu_bo_kunmap(ring->mqd_obj);
			ring->mqd_ptr = NULL;
		}
		amdgpu_bo_unreserve(ring->mqd_obj);
		if (r)
			goto done;
	}

	r = gfx_v9_0_kiq_kcq_enable(adev);
done:
	return r;
}

static int gfx_v9_0_cp_resume(struct amdgpu_device *adev)
{
	int r, i;
	struct amdgpu_ring *ring;

	if (!(adev->flags & AMD_IS_APU))
		gfx_v9_0_enable_gui_idle_interrupt(adev, false);

	if (adev->firmware.load_type != AMDGPU_FW_LOAD_PSP) {
		/* legacy firmware loading */
		r = gfx_v9_0_cp_gfx_load_microcode(adev);
		if (r)
			return r;

		r = gfx_v9_0_cp_compute_load_microcode(adev);
		if (r)
			return r;
	}

	r = gfx_v9_0_cp_gfx_resume(adev);
	if (r)
		return r;

	r = gfx_v9_0_kiq_resume(adev);
	if (r)
		return r;

	ring = &adev->gfx.gfx_ring[0];
	r = amdgpu_ring_test_ring(ring);
	if (r) {
		ring->ready = false;
		return r;
	}

	ring = &adev->gfx.kiq.ring;
	ring->ready = true;
	r = amdgpu_ring_test_ring(ring);
	if (r)
		ring->ready = false;

	for (i = 0; i < adev->gfx.num_compute_rings; i++) {
		ring = &adev->gfx.compute_ring[i];

		ring->ready = true;
		r = amdgpu_ring_test_ring(ring);
		if (r)
			ring->ready = false;
	}

	gfx_v9_0_enable_gui_idle_interrupt(adev, true);

	return 0;
}

static void gfx_v9_0_cp_enable(struct amdgpu_device *adev, bool enable)
{
	gfx_v9_0_cp_gfx_enable(adev, enable);
	gfx_v9_0_cp_compute_enable(adev, enable);
}

static int gfx_v9_0_hw_init(void *handle)
{
	int r;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	gfx_v9_0_init_golden_registers(adev);

	gfx_v9_0_gpu_init(adev);

	r = gfx_v9_0_rlc_resume(adev);
	if (r)
		return r;

	r = gfx_v9_0_cp_resume(adev);
	if (r)
		return r;

	r = gfx_v9_0_ngg_en(adev);
	if (r)
		return r;

	return r;
}

static int gfx_v9_0_hw_fini(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	amdgpu_irq_put(adev, &adev->gfx.priv_reg_irq, 0);
	amdgpu_irq_put(adev, &adev->gfx.priv_inst_irq, 0);
	if (amdgpu_sriov_vf(adev)) {
		gfx_v9_0_cp_gfx_enable(adev, false);
		/* must disable polling for SRIOV when hw finished, otherwise
		 * CPC engine may still keep fetching WB address which is already
		 * invalid after sw finished and trigger DMAR reading error in
		 * hypervisor side.
		 */
		WREG32_FIELD15(GC, 0, CP_PQ_WPTR_POLL_CNTL, EN, 0);
		return 0;
	}
	gfx_v9_0_cp_enable(adev, false);
	gfx_v9_0_rlc_stop(adev);

	return 0;
}

static int gfx_v9_0_suspend(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	adev->gfx.in_suspend = true;
	return gfx_v9_0_hw_fini(adev);
}

static int gfx_v9_0_resume(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;
	int r;

	r = gfx_v9_0_hw_init(adev);
	adev->gfx.in_suspend = false;
	return r;
}

static bool gfx_v9_0_is_idle(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	if (REG_GET_FIELD(RREG32_SOC15(GC, 0, mmGRBM_STATUS),
				GRBM_STATUS, GUI_ACTIVE))
		return false;
	else
		return true;
}

static int gfx_v9_0_wait_for_idle(void *handle)
{
	unsigned i;
	u32 tmp;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	for (i = 0; i < adev->usec_timeout; i++) {
		/* read MC_STATUS */
		tmp = RREG32_SOC15(GC, 0, mmGRBM_STATUS) &
			GRBM_STATUS__GUI_ACTIVE_MASK;

		if (!REG_GET_FIELD(tmp, GRBM_STATUS, GUI_ACTIVE))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

static int gfx_v9_0_soft_reset(void *handle)
{
	u32 grbm_soft_reset = 0;
	u32 tmp;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	/* GRBM_STATUS */
	tmp = RREG32_SOC15(GC, 0, mmGRBM_STATUS);
	if (tmp & (GRBM_STATUS__PA_BUSY_MASK | GRBM_STATUS__SC_BUSY_MASK |
		   GRBM_STATUS__BCI_BUSY_MASK | GRBM_STATUS__SX_BUSY_MASK |
		   GRBM_STATUS__TA_BUSY_MASK | GRBM_STATUS__VGT_BUSY_MASK |
		   GRBM_STATUS__DB_BUSY_MASK | GRBM_STATUS__CB_BUSY_MASK |
		   GRBM_STATUS__GDS_BUSY_MASK | GRBM_STATUS__SPI_BUSY_MASK |
		   GRBM_STATUS__IA_BUSY_MASK | GRBM_STATUS__IA_BUSY_NO_DMA_MASK)) {
		grbm_soft_reset = REG_SET_FIELD(grbm_soft_reset,
						GRBM_SOFT_RESET, SOFT_RESET_CP, 1);
		grbm_soft_reset = REG_SET_FIELD(grbm_soft_reset,
						GRBM_SOFT_RESET, SOFT_RESET_GFX, 1);
	}

	if (tmp & (GRBM_STATUS__CP_BUSY_MASK | GRBM_STATUS__CP_COHERENCY_BUSY_MASK)) {
		grbm_soft_reset = REG_SET_FIELD(grbm_soft_reset,
						GRBM_SOFT_RESET, SOFT_RESET_CP, 1);
	}

	/* GRBM_STATUS2 */
	tmp = RREG32_SOC15(GC, 0, mmGRBM_STATUS2);
	if (REG_GET_FIELD(tmp, GRBM_STATUS2, RLC_BUSY))
		grbm_soft_reset = REG_SET_FIELD(grbm_soft_reset,
						GRBM_SOFT_RESET, SOFT_RESET_RLC, 1);


	if (grbm_soft_reset) {
		/* stop the rlc */
		gfx_v9_0_rlc_stop(adev);

		/* Disable GFX parsing/prefetching */
		gfx_v9_0_cp_gfx_enable(adev, false);

		/* Disable MEC parsing/prefetching */
		gfx_v9_0_cp_compute_enable(adev, false);

		if (grbm_soft_reset) {
			tmp = RREG32_SOC15(GC, 0, mmGRBM_SOFT_RESET);
			tmp |= grbm_soft_reset;
			dev_info(adev->dev, "GRBM_SOFT_RESET=0x%08X\n", tmp);
			WREG32_SOC15(GC, 0, mmGRBM_SOFT_RESET, tmp);
			tmp = RREG32_SOC15(GC, 0, mmGRBM_SOFT_RESET);

			udelay(50);

			tmp &= ~grbm_soft_reset;
			WREG32_SOC15(GC, 0, mmGRBM_SOFT_RESET, tmp);
			tmp = RREG32_SOC15(GC, 0, mmGRBM_SOFT_RESET);
		}

		/* Wait a little for things to settle down */
		udelay(50);
	}
	return 0;
}

static uint64_t gfx_v9_0_get_gpu_clock_counter(struct amdgpu_device *adev)
{
	uint64_t clock;

	mutex_lock(&adev->gfx.gpu_clock_mutex);
	WREG32_SOC15(GC, 0, mmRLC_CAPTURE_GPU_CLOCK_COUNT, 1);
	clock = (uint64_t)RREG32_SOC15(GC, 0, mmRLC_GPU_CLOCK_COUNT_LSB) |
		((uint64_t)RREG32_SOC15(GC, 0, mmRLC_GPU_CLOCK_COUNT_MSB) << 32ULL);
	mutex_unlock(&adev->gfx.gpu_clock_mutex);
	return clock;
}

static void gfx_v9_0_ring_emit_gds_switch(struct amdgpu_ring *ring,
					  uint32_t vmid,
					  uint32_t gds_base, uint32_t gds_size,
					  uint32_t gws_base, uint32_t gws_size,
					  uint32_t oa_base, uint32_t oa_size)
{
	gds_base = gds_base >> AMDGPU_GDS_SHIFT;
	gds_size = gds_size >> AMDGPU_GDS_SHIFT;

	gws_base = gws_base >> AMDGPU_GWS_SHIFT;
	gws_size = gws_size >> AMDGPU_GWS_SHIFT;

	oa_base = oa_base >> AMDGPU_OA_SHIFT;
	oa_size = oa_size >> AMDGPU_OA_SHIFT;

	/* GDS Base */
	gfx_v9_0_write_data_to_reg(ring, 0, false,
				   amdgpu_gds_reg_offset[vmid].mem_base,
				   gds_base);

	/* GDS Size */
	gfx_v9_0_write_data_to_reg(ring, 0, false,
				   amdgpu_gds_reg_offset[vmid].mem_size,
				   gds_size);

	/* GWS */
	gfx_v9_0_write_data_to_reg(ring, 0, false,
				   amdgpu_gds_reg_offset[vmid].gws,
				   gws_size << GDS_GWS_VMID0__SIZE__SHIFT | gws_base);

	/* OA */
	gfx_v9_0_write_data_to_reg(ring, 0, false,
				   amdgpu_gds_reg_offset[vmid].oa,
				   (1 << (oa_size + oa_base)) - (1 << oa_base));
}

static int gfx_v9_0_early_init(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	adev->gfx.num_gfx_rings = GFX9_NUM_GFX_RINGS;
	adev->gfx.num_compute_rings = AMDGPU_MAX_COMPUTE_RINGS;
	gfx_v9_0_set_ring_funcs(adev);
	gfx_v9_0_set_irq_funcs(adev);
	gfx_v9_0_set_gds_init(adev);
	gfx_v9_0_set_rlc_funcs(adev);

	return 0;
}

static int gfx_v9_0_late_init(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;
	int r;

	r = amdgpu_irq_get(adev, &adev->gfx.priv_reg_irq, 0);
	if (r)
		return r;

	r = amdgpu_irq_get(adev, &adev->gfx.priv_inst_irq, 0);
	if (r)
		return r;

	return 0;
}

static void gfx_v9_0_enter_rlc_safe_mode(struct amdgpu_device *adev)
{
	uint32_t rlc_setting, data;
	unsigned i;

	if (adev->gfx.rlc.in_safe_mode)
		return;

	/* if RLC is not enabled, do nothing */
	rlc_setting = RREG32_SOC15(GC, 0, mmRLC_CNTL);
	if (!(rlc_setting & RLC_CNTL__RLC_ENABLE_F32_MASK))
		return;

	if (adev->cg_flags &
	    (AMD_CG_SUPPORT_GFX_CGCG | AMD_CG_SUPPORT_GFX_MGCG |
	     AMD_CG_SUPPORT_GFX_3D_CGCG)) {
		data = RLC_SAFE_MODE__CMD_MASK;
		data |= (1 << RLC_SAFE_MODE__MESSAGE__SHIFT);
		WREG32_SOC15(GC, 0, mmRLC_SAFE_MODE, data);

		/* wait for RLC_SAFE_MODE */
		for (i = 0; i < adev->usec_timeout; i++) {
			if (!REG_GET_FIELD(RREG32_SOC15(GC, 0, mmRLC_SAFE_MODE), RLC_SAFE_MODE, CMD))
				break;
			udelay(1);
		}
		adev->gfx.rlc.in_safe_mode = true;
	}
}

static void gfx_v9_0_exit_rlc_safe_mode(struct amdgpu_device *adev)
{
	uint32_t rlc_setting, data;

	if (!adev->gfx.rlc.in_safe_mode)
		return;

	/* if RLC is not enabled, do nothing */
	rlc_setting = RREG32_SOC15(GC, 0, mmRLC_CNTL);
	if (!(rlc_setting & RLC_CNTL__RLC_ENABLE_F32_MASK))
		return;

	if (adev->cg_flags &
	    (AMD_CG_SUPPORT_GFX_CGCG | AMD_CG_SUPPORT_GFX_MGCG)) {
		/*
		 * Try to exit safe mode only if it is already in safe
		 * mode.
		 */
		data = RLC_SAFE_MODE__CMD_MASK;
		WREG32_SOC15(GC, 0, mmRLC_SAFE_MODE, data);
		adev->gfx.rlc.in_safe_mode = false;
	}
}

static void gfx_v9_0_update_gfx_cg_power_gating(struct amdgpu_device *adev,
						bool enable)
{
	/* TODO: double check if we need to perform under safe mdoe */
	/* gfx_v9_0_enter_rlc_safe_mode(adev); */

	if ((adev->pg_flags & AMD_PG_SUPPORT_GFX_PG) && enable) {
		gfx_v9_0_enable_gfx_cg_power_gating(adev, true);
		if (adev->pg_flags & AMD_PG_SUPPORT_GFX_PIPELINE)
			gfx_v9_0_enable_gfx_pipeline_powergating(adev, true);
	} else {
		gfx_v9_0_enable_gfx_cg_power_gating(adev, false);
		gfx_v9_0_enable_gfx_pipeline_powergating(adev, false);
	}

	/* gfx_v9_0_exit_rlc_safe_mode(adev); */
}

static void gfx_v9_0_update_gfx_mg_power_gating(struct amdgpu_device *adev,
						bool enable)
{
	/* TODO: double check if we need to perform under safe mode */
	/* gfx_v9_0_enter_rlc_safe_mode(adev); */

	if ((adev->pg_flags & AMD_PG_SUPPORT_GFX_SMG) && enable)
		gfx_v9_0_enable_gfx_static_mg_power_gating(adev, true);
	else
		gfx_v9_0_enable_gfx_static_mg_power_gating(adev, false);

	if ((adev->pg_flags & AMD_PG_SUPPORT_GFX_DMG) && enable)
		gfx_v9_0_enable_gfx_dynamic_mg_power_gating(adev, true);
	else
		gfx_v9_0_enable_gfx_dynamic_mg_power_gating(adev, false);

	/* gfx_v9_0_exit_rlc_safe_mode(adev); */
}

static void gfx_v9_0_update_medium_grain_clock_gating(struct amdgpu_device *adev,
						      bool enable)
{
	uint32_t data, def;

	/* It is disabled by HW by default */
	if (enable && (adev->cg_flags & AMD_CG_SUPPORT_GFX_MGCG)) {
		/* 1 - RLC_CGTT_MGCG_OVERRIDE */
		def = data = RREG32_SOC15(GC, 0, mmRLC_CGTT_MGCG_OVERRIDE);
		data &= ~(RLC_CGTT_MGCG_OVERRIDE__CPF_CGTT_SCLK_OVERRIDE_MASK |
			  RLC_CGTT_MGCG_OVERRIDE__GRBM_CGTT_SCLK_OVERRIDE_MASK |
			  RLC_CGTT_MGCG_OVERRIDE__GFXIP_MGCG_OVERRIDE_MASK |
			  RLC_CGTT_MGCG_OVERRIDE__GFXIP_MGLS_OVERRIDE_MASK);

		/* only for Vega10 & Raven1 */
		data |= RLC_CGTT_MGCG_OVERRIDE__RLC_CGTT_SCLK_OVERRIDE_MASK;

		if (def != data)
			WREG32_SOC15(GC, 0, mmRLC_CGTT_MGCG_OVERRIDE, data);

		/* MGLS is a global flag to control all MGLS in GFX */
		if (adev->cg_flags & AMD_CG_SUPPORT_GFX_MGLS) {
			/* 2 - RLC memory Light sleep */
			if (adev->cg_flags & AMD_CG_SUPPORT_GFX_RLC_LS) {
				def = data = RREG32_SOC15(GC, 0, mmRLC_MEM_SLP_CNTL);
				data |= RLC_MEM_SLP_CNTL__RLC_MEM_LS_EN_MASK;
				if (def != data)
					WREG32_SOC15(GC, 0, mmRLC_MEM_SLP_CNTL, data);
			}
			/* 3 - CP memory Light sleep */
			if (adev->cg_flags & AMD_CG_SUPPORT_GFX_CP_LS) {
				def = data = RREG32_SOC15(GC, 0, mmCP_MEM_SLP_CNTL);
				data |= CP_MEM_SLP_CNTL__CP_MEM_LS_EN_MASK;
				if (def != data)
					WREG32_SOC15(GC, 0, mmCP_MEM_SLP_CNTL, data);
			}
		}
	} else {
		/* 1 - MGCG_OVERRIDE */
		def = data = RREG32_SOC15(GC, 0, mmRLC_CGTT_MGCG_OVERRIDE);
		data |= (RLC_CGTT_MGCG_OVERRIDE__CPF_CGTT_SCLK_OVERRIDE_MASK |
			 RLC_CGTT_MGCG_OVERRIDE__RLC_CGTT_SCLK_OVERRIDE_MASK |
			 RLC_CGTT_MGCG_OVERRIDE__GRBM_CGTT_SCLK_OVERRIDE_MASK |
			 RLC_CGTT_MGCG_OVERRIDE__GFXIP_MGCG_OVERRIDE_MASK |
			 RLC_CGTT_MGCG_OVERRIDE__GFXIP_MGLS_OVERRIDE_MASK);
		if (def != data)
			WREG32_SOC15(GC, 0, mmRLC_CGTT_MGCG_OVERRIDE, data);

		/* 2 - disable MGLS in RLC */
		data = RREG32_SOC15(GC, 0, mmRLC_MEM_SLP_CNTL);
		if (data & RLC_MEM_SLP_CNTL__RLC_MEM_LS_EN_MASK) {
			data &= ~RLC_MEM_SLP_CNTL__RLC_MEM_LS_EN_MASK;
			WREG32_SOC15(GC, 0, mmRLC_MEM_SLP_CNTL, data);
		}

		/* 3 - disable MGLS in CP */
		data = RREG32_SOC15(GC, 0, mmCP_MEM_SLP_CNTL);
		if (data & CP_MEM_SLP_CNTL__CP_MEM_LS_EN_MASK) {
			data &= ~CP_MEM_SLP_CNTL__CP_MEM_LS_EN_MASK;
			WREG32_SOC15(GC, 0, mmCP_MEM_SLP_CNTL, data);
		}
	}
}

static void gfx_v9_0_update_3d_clock_gating(struct amdgpu_device *adev,
					   bool enable)
{
	uint32_t data, def;

	adev->gfx.rlc.funcs->enter_safe_mode(adev);

	/* Enable 3D CGCG/CGLS */
	if (enable && (adev->cg_flags & AMD_CG_SUPPORT_GFX_3D_CGCG)) {
		/* write cmd to clear cgcg/cgls ov */
		def = data = RREG32_SOC15(GC, 0, mmRLC_CGTT_MGCG_OVERRIDE);
		/* unset CGCG override */
		data &= ~RLC_CGTT_MGCG_OVERRIDE__GFXIP_GFX3D_CG_OVERRIDE_MASK;
		/* update CGCG and CGLS override bits */
		if (def != data)
			WREG32_SOC15(GC, 0, mmRLC_CGTT_MGCG_OVERRIDE, data);
		/* enable 3Dcgcg FSM(0x0020003f) */
		def = RREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL_3D);
		data = (0x2000 << RLC_CGCG_CGLS_CTRL_3D__CGCG_GFX_IDLE_THRESHOLD__SHIFT) |
			RLC_CGCG_CGLS_CTRL_3D__CGCG_EN_MASK;
		if (adev->cg_flags & AMD_CG_SUPPORT_GFX_3D_CGLS)
			data |= (0x000F << RLC_CGCG_CGLS_CTRL_3D__CGLS_REP_COMPANSAT_DELAY__SHIFT) |
				RLC_CGCG_CGLS_CTRL_3D__CGLS_EN_MASK;
		if (def != data)
			WREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL_3D, data);

		/* set IDLE_POLL_COUNT(0x00900100) */
		def = RREG32_SOC15(GC, 0, mmCP_RB_WPTR_POLL_CNTL);
		data = (0x0100 << CP_RB_WPTR_POLL_CNTL__POLL_FREQUENCY__SHIFT) |
			(0x0090 << CP_RB_WPTR_POLL_CNTL__IDLE_POLL_COUNT__SHIFT);
		if (def != data)
			WREG32_SOC15(GC, 0, mmCP_RB_WPTR_POLL_CNTL, data);
	} else {
		/* Disable CGCG/CGLS */
		def = data = RREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL_3D);
		/* disable cgcg, cgls should be disabled */
		data &= ~(RLC_CGCG_CGLS_CTRL_3D__CGCG_EN_MASK |
			  RLC_CGCG_CGLS_CTRL_3D__CGLS_EN_MASK);
		/* disable cgcg and cgls in FSM */
		if (def != data)
			WREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL_3D, data);
	}

	adev->gfx.rlc.funcs->exit_safe_mode(adev);
}

static void gfx_v9_0_update_coarse_grain_clock_gating(struct amdgpu_device *adev,
						      bool enable)
{
	uint32_t def, data;

	adev->gfx.rlc.funcs->enter_safe_mode(adev);

	if (enable && (adev->cg_flags & AMD_CG_SUPPORT_GFX_CGCG)) {
		def = data = RREG32_SOC15(GC, 0, mmRLC_CGTT_MGCG_OVERRIDE);
		/* unset CGCG override */
		data &= ~RLC_CGTT_MGCG_OVERRIDE__GFXIP_CGCG_OVERRIDE_MASK;
		if (adev->cg_flags & AMD_CG_SUPPORT_GFX_CGLS)
			data &= ~RLC_CGTT_MGCG_OVERRIDE__GFXIP_CGLS_OVERRIDE_MASK;
		else
			data |= RLC_CGTT_MGCG_OVERRIDE__GFXIP_CGLS_OVERRIDE_MASK;
		/* update CGCG and CGLS override bits */
		if (def != data)
			WREG32_SOC15(GC, 0, mmRLC_CGTT_MGCG_OVERRIDE, data);

		/* enable cgcg FSM(0x0020003F) */
		def = RREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL);
		data = (0x2000 << RLC_CGCG_CGLS_CTRL__CGCG_GFX_IDLE_THRESHOLD__SHIFT) |
			RLC_CGCG_CGLS_CTRL__CGCG_EN_MASK;
		if (adev->cg_flags & AMD_CG_SUPPORT_GFX_CGLS)
			data |= (0x000F << RLC_CGCG_CGLS_CTRL__CGLS_REP_COMPANSAT_DELAY__SHIFT) |
				RLC_CGCG_CGLS_CTRL__CGLS_EN_MASK;
		if (def != data)
			WREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL, data);

		/* set IDLE_POLL_COUNT(0x00900100) */
		def = RREG32_SOC15(GC, 0, mmCP_RB_WPTR_POLL_CNTL);
		data = (0x0100 << CP_RB_WPTR_POLL_CNTL__POLL_FREQUENCY__SHIFT) |
			(0x0090 << CP_RB_WPTR_POLL_CNTL__IDLE_POLL_COUNT__SHIFT);
		if (def != data)
			WREG32_SOC15(GC, 0, mmCP_RB_WPTR_POLL_CNTL, data);
	} else {
		def = data = RREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL);
		/* reset CGCG/CGLS bits */
		data &= ~(RLC_CGCG_CGLS_CTRL__CGCG_EN_MASK | RLC_CGCG_CGLS_CTRL__CGLS_EN_MASK);
		/* disable cgcg and cgls in FSM */
		if (def != data)
			WREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL, data);
	}

	adev->gfx.rlc.funcs->exit_safe_mode(adev);
}

static int gfx_v9_0_update_gfx_clock_gating(struct amdgpu_device *adev,
					    bool enable)
{
	if (enable) {
		/* CGCG/CGLS should be enabled after MGCG/MGLS
		 * ===  MGCG + MGLS ===
		 */
		gfx_v9_0_update_medium_grain_clock_gating(adev, enable);
		/* ===  CGCG /CGLS for GFX 3D Only === */
		gfx_v9_0_update_3d_clock_gating(adev, enable);
		/* ===  CGCG + CGLS === */
		gfx_v9_0_update_coarse_grain_clock_gating(adev, enable);
	} else {
		/* CGCG/CGLS should be disabled before MGCG/MGLS
		 * ===  CGCG + CGLS ===
		 */
		gfx_v9_0_update_coarse_grain_clock_gating(adev, enable);
		/* ===  CGCG /CGLS for GFX 3D Only === */
		gfx_v9_0_update_3d_clock_gating(adev, enable);
		/* ===  MGCG + MGLS === */
		gfx_v9_0_update_medium_grain_clock_gating(adev, enable);
	}
	return 0;
}

static const struct amdgpu_rlc_funcs gfx_v9_0_rlc_funcs = {
	.enter_safe_mode = gfx_v9_0_enter_rlc_safe_mode,
	.exit_safe_mode = gfx_v9_0_exit_rlc_safe_mode
};

static int gfx_v9_0_set_powergating_state(void *handle,
					  enum amd_powergating_state state)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;
	bool enable = (state == AMD_PG_STATE_GATE) ? true : false;

	switch (adev->asic_type) {
	case CHIP_RAVEN:
		if (adev->pg_flags & AMD_PG_SUPPORT_RLC_SMU_HS) {
			gfx_v9_0_enable_sck_slow_down_on_power_up(adev, true);
			gfx_v9_0_enable_sck_slow_down_on_power_down(adev, true);
		} else {
			gfx_v9_0_enable_sck_slow_down_on_power_up(adev, false);
			gfx_v9_0_enable_sck_slow_down_on_power_down(adev, false);
		}

		if (adev->pg_flags & AMD_PG_SUPPORT_CP)
			gfx_v9_0_enable_cp_power_gating(adev, true);
		else
			gfx_v9_0_enable_cp_power_gating(adev, false);

		/* update gfx cgpg state */
		gfx_v9_0_update_gfx_cg_power_gating(adev, enable);

		/* update mgcg state */
		gfx_v9_0_update_gfx_mg_power_gating(adev, enable);
		break;
	default:
		break;
	}

	return 0;
}

static int gfx_v9_0_set_clockgating_state(void *handle,
					  enum amd_clockgating_state state)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	if (amdgpu_sriov_vf(adev))
		return 0;

	switch (adev->asic_type) {
	case CHIP_VEGA10:
	case CHIP_RAVEN:
		gfx_v9_0_update_gfx_clock_gating(adev,
						 state == AMD_CG_STATE_GATE ? true : false);
		break;
	default:
		break;
	}
	return 0;
}

static void gfx_v9_0_get_clockgating_state(void *handle, u32 *flags)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;
	int data;

	if (amdgpu_sriov_vf(adev))
		*flags = 0;

	/* AMD_CG_SUPPORT_GFX_MGCG */
	data = RREG32_SOC15(GC, 0, mmRLC_CGTT_MGCG_OVERRIDE);
	if (!(data & RLC_CGTT_MGCG_OVERRIDE__GFXIP_MGCG_OVERRIDE_MASK))
		*flags |= AMD_CG_SUPPORT_GFX_MGCG;

	/* AMD_CG_SUPPORT_GFX_CGCG */
	data = RREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL);
	if (data & RLC_CGCG_CGLS_CTRL__CGCG_EN_MASK)
		*flags |= AMD_CG_SUPPORT_GFX_CGCG;

	/* AMD_CG_SUPPORT_GFX_CGLS */
	if (data & RLC_CGCG_CGLS_CTRL__CGLS_EN_MASK)
		*flags |= AMD_CG_SUPPORT_GFX_CGLS;

	/* AMD_CG_SUPPORT_GFX_RLC_LS */
	data = RREG32_SOC15(GC, 0, mmRLC_MEM_SLP_CNTL);
	if (data & RLC_MEM_SLP_CNTL__RLC_MEM_LS_EN_MASK)
		*flags |= AMD_CG_SUPPORT_GFX_RLC_LS | AMD_CG_SUPPORT_GFX_MGLS;

	/* AMD_CG_SUPPORT_GFX_CP_LS */
	data = RREG32_SOC15(GC, 0, mmCP_MEM_SLP_CNTL);
	if (data & CP_MEM_SLP_CNTL__CP_MEM_LS_EN_MASK)
		*flags |= AMD_CG_SUPPORT_GFX_CP_LS | AMD_CG_SUPPORT_GFX_MGLS;

	/* AMD_CG_SUPPORT_GFX_3D_CGCG */
	data = RREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL_3D);
	if (data & RLC_CGCG_CGLS_CTRL_3D__CGCG_EN_MASK)
		*flags |= AMD_CG_SUPPORT_GFX_3D_CGCG;

	/* AMD_CG_SUPPORT_GFX_3D_CGLS */
	if (data & RLC_CGCG_CGLS_CTRL_3D__CGLS_EN_MASK)
		*flags |= AMD_CG_SUPPORT_GFX_3D_CGLS;
}

static u64 gfx_v9_0_ring_get_rptr_gfx(struct amdgpu_ring *ring)
{
	return ring->adev->wb.wb[ring->rptr_offs]; /* gfx9 is 32bit rptr*/
}

static u64 gfx_v9_0_ring_get_wptr_gfx(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;
	u64 wptr;

	/* XXX check if swapping is necessary on BE */
	if (ring->use_doorbell) {
		wptr = atomic64_read((atomic64_t *)&adev->wb.wb[ring->wptr_offs]);
	} else {
		wptr = RREG32_SOC15(GC, 0, mmCP_RB0_WPTR);
		wptr += (u64)RREG32_SOC15(GC, 0, mmCP_RB0_WPTR_HI) << 32;
	}

	return wptr;
}

static void gfx_v9_0_ring_set_wptr_gfx(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;

	if (ring->use_doorbell) {
		/* XXX check if swapping is necessary on BE */
		atomic64_set((atomic64_t*)&adev->wb.wb[ring->wptr_offs], ring->wptr);
		WDOORBELL64(ring->doorbell_index, ring->wptr);
	} else {
		WREG32_SOC15(GC, 0, mmCP_RB0_WPTR, lower_32_bits(ring->wptr));
		WREG32_SOC15(GC, 0, mmCP_RB0_WPTR_HI, upper_32_bits(ring->wptr));
	}
}

static void gfx_v9_0_ring_emit_hdp_flush(struct amdgpu_ring *ring)
{
	u32 ref_and_mask, reg_mem_engine;
	struct nbio_hdp_flush_reg *nbio_hf_reg;

	if (ring->adev->asic_type == CHIP_VEGA10)
		nbio_hf_reg = &nbio_v6_1_hdp_flush_reg;

	if (ring->funcs->type == AMDGPU_RING_TYPE_COMPUTE) {
		switch (ring->me) {
		case 1:
			ref_and_mask = nbio_hf_reg->ref_and_mask_cp2 << ring->pipe;
			break;
		case 2:
			ref_and_mask = nbio_hf_reg->ref_and_mask_cp6 << ring->pipe;
			break;
		default:
			return;
		}
		reg_mem_engine = 0;
	} else {
		ref_and_mask = nbio_hf_reg->ref_and_mask_cp0;
		reg_mem_engine = 1; /* pfp */
	}

	gfx_v9_0_wait_reg_mem(ring, reg_mem_engine, 0, 1,
			      nbio_hf_reg->hdp_flush_req_offset,
			      nbio_hf_reg->hdp_flush_done_offset,
			      ref_and_mask, ref_and_mask, 0x20);
}

static void gfx_v9_0_ring_emit_hdp_invalidate(struct amdgpu_ring *ring)
{
	gfx_v9_0_write_data_to_reg(ring, 0, true,
				   SOC15_REG_OFFSET(HDP, 0, mmHDP_DEBUG0), 1);
}

static void gfx_v9_0_ring_emit_ib_gfx(struct amdgpu_ring *ring,
                                      struct amdgpu_ib *ib,
                                      unsigned vm_id, bool ctx_switch)
{
	u32 header, control = 0;

	if (ib->flags & AMDGPU_IB_FLAG_CE)
		header = PACKET3(PACKET3_INDIRECT_BUFFER_CONST, 2);
	else
		header = PACKET3(PACKET3_INDIRECT_BUFFER, 2);

	control |= ib->length_dw | (vm_id << 24);

	if (amdgpu_sriov_vf(
	data = Rvg       struct amdgpu_ib *ib_MASK |
			 RLC_CGTT_MGCG_OVERRIDE__RLC_CGTT_SCLK_OVERRIDE_MASK |
			 RLC_CGTT_MGCG_OVERRIDE__GRBM_CGTT_SCLK_OVERRIDE_MASK |
			 RLC_CGTT_MGCG_OVERRIDE__GFXIP_MGCG_OVERRIDE_MASK |
			 RLC_CGTT_MGCG_OVERRIDE__GFXIP_MGLS_OVERRIDE_MASK);
		if (def != data)
			W0, mmRLC_CGTT_MGCG_OVERRIDE);
	if (!(data & RLC_CGTT_MGCG_OVERRIDEM
WU00 << CP_RB_WPTR_POLL_CNTL_cs[LE_POLL_COUNT(0x00900100) */
		def = RREG32_SOC15(GC, 0, mmCP_RB_WPTR_POLL_CNTL);
		data = (0x0100PZs_POLL_CNTL);
		data = (0x0100PZs_POLL	if (data & RLC_CGCG_CGLS_CTRL_3D__CGLS_EN_MASK)
		*flags |= AMD_CG_SUPPORT_GFX_3D_CGLS;
}

staticSUPPORT_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_	_CGCG */
	data = RREG32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRL);
	if (data & RLC_CGCG_CGLS_CTRL__CGCG_EN_MASK)
		*flagsRWbi_SUPPORT_GFX_MGLS;

	/* AMD_CG_SUPPORT_GFX_3D_CGCG */
	data = RREG_MASK)
		N_MASK)
		*flags |= AMD_CG_SUPPORT_GFX_CP_LS | AMD_CG_SUPPORT_GFX_CG_S	MD_CG_SUPPORT_GFX_RLC_LS */
	data = RREG32_SOC15(GC, 0, mmRLC_MEM_SLP_CNTL);
	if (data & RLC_MEM_SLP_CNTL__RLC_MEMRWh_CGLS_CTRL_	/* NTL__RLC_MvPPORT_GFX_3D_CGCG */
	data = RREG__
GFX_3D_CGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCCGCC	v      

		if (adev->pg_flags & AMD_PG_SUPPORT_CP)
			gfx_v9_0_enable_cp_power_gating(adev, true);
		elsecoarET, ring	e,
				   SOC1f (adev->pg_)	GRBM_STATUCCGCCGCCGCCGCCGCC gfx_v9_0_ringf (adev->pg_)	GRBM_STATUCCGCCGCCGCCGCCGCC ring buff)
		*flags |= AMD_CG_SUPPORT_GFX_3D_C;

		/*;
}

staticSUPPORT_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GF_GGF_GF_GF_GF_GF_GF_GF_GF_	_CGCG */
	data = RREG32_SOC15(GC, 0, mm, mmRLC_CGCG_CGLS_CTRL);
	if (data & RL, mm, mmRCGCTRL__CGCG_Wh_CGLS_CTRL_	/*VALIDUPP_CG_S	MD_CG_SUPPORT_GFX_RLC_LS *, mm, mmM_STATUCCGCCGCCGCCGCCGCCs |= AMD_CG_SUPPORT_GFX_CP_LS | AMD	GRB  

		if (adev->pg_flags & AMD_PG_SUPPORT_CP)
, mm, mmM_STATUCCGCCGCCGCCGCCGCwer_gating(adev, trueata = RREG32_SOC15(GC, 0, mm, mm
		elsecoarET, rinata = RREG32_SOC15(GC, 0, mm, mme,
				   SOC1f (adev->pg_)	GR, mm, mmM_STATUCCGCCGCCGCCGCCGCC gfx_v9_0_ringf (adev->pg_)	GR, mm, mmM_STATUCCGCCGCCGCCGCCGCC ring buff)
		*flags |= AMD_CG_SUPPORT_GFX_3fenc*;
}

staticSUPPORT_GF_GF_Gype =>pg_ase_2(ringype =seq,mRLC_CGCG_/
	data = RL);
GCCGC64 (ri= CGCG */
	data =FENCEREG__
64BIT:
	case iing-eli= CGCG */
	data =FENCEREG__
INnt r;

	RELEbellMEM - T_MGC ca,
	ringBM_SCGCGP)
			gfx_v9_0_enable_cp_poCs |= AMD_CG_SUPPRELEbellMEM, 6)	GRBM_STATUCCGCCGCCGCCGCCGCCREG32TCL1
	}

ONev,r GFX 3 EG32TC
	}

ONev,r GFX 3 EG32TC
WB
	}

ONev,r GFX 3 EG32TC
MD
	}

ONev,r GFX 3 EV;
		else(CACHEREGUSH_AND
INV_T32_V;
	)r GFX 3 EV;
		ORTEX(5))	GRBM_STATUCCGCCGCCGCCGCCGCCDATA_SEL(GCCGC64 (ri? 2 : thi|_WhT_SEL(iing-eli? 2 : 0)T) |
		
	}

ex_utimeout;ow_down_onQG_SUPPORT_the qu64 (riGCCGC,_PG_SU
	}

PORT_the quGTT_SgBM_Sif (ri5(GC,e,
 (discaSUP5(GC,hig & 	d_mask = GCCGC64 (r_3D_  

		i>pg_flags7K)
		N_MASK  

		i>pg_flags3	GRBM_STATUCCGCCGCCGCCGCCGCCe,
				   SOC1>pg_)	GRBM_STATUCCGCCGCCGCCGCCGCC gfx_v9_0_ring>pg_)	GRBM_STATUCCGCCGCCGCCGCCGCCe,
				   SOC1seq)	GRBM_STATUCCGCCGCCGCCGCCGCC gfx_v9_0_ringseq)	GRBM_STATUCCGCCGCCGCCGCCGCCUNT(0x00900100) */
		def = RREG32_SOC1ERRIDE);
syncRB_WPTR_POLL_CNTL);
		data = (CGCGreg |
	=ACKET3_INDIRECT_BUFFER_CONST, 2);
	else
GFX	GRBDLE_POLL_seqne, 0, 1,fenc*_drv.syncg-eq;				   gds_ELOAD_S 0, 1,fenc*_drv.dev->pg_;C_CGTT_MGCG_OVERRIDE__GFXIP_MGLreg |

		iS_CU00 << CP_e,
				   SOC1>pg_)CC gfx_v9_0_ring>pg_)CU00 << CP_seq,mSOC15(GC,f,ate =0x00900100) */
		def = RREG32_SOC1RT_                      struct aMEM_SLPRLC_CGCG_CGLS_CT		   gds_pd->pg_)ait_reg_mem(ring, vmhuF_GhuF_IB_
	u32 headervmhuF[KET3_INDIRECTvmhuF]; (CGCGreg |
	=ACKET3_INDIRECT_BUFFER_CONST, 2);
	else
GFX	GRBDLE_POLL_reqne, 0, 1,ating_sart.sart_NDIRECT
		n, 0, mmCP_T_MGORT_GF	GRBDLC_CGCG_e_gati 0, 1,RT_GnvIDE_;C_Cpd->pg_
	/* if RLsart_
		nRT_pde15(GC, 0, m,_pd->pg_);_Cpd->pg_
|R_CONST, PTE*VALID;C_CGTT_MGCG_OPOLL_CNTL);
		data = (0reg |

	00PZs_POLL	ifhuF->	if0_ptb DOORBELCGC+m
		*_CGLS_)s_POLL	ife,
				   SOC1pd->pg_));C_CGTT_MGCG_OPOLL_CNTL);
		data = (0reg |

	00PZs_POLL	ifhuF->	if0_ptb DOORBhiCGC+m
		*_CGLS_)s_POLL	if gfx_v9_0_ringpd->pg_));C_CGTT_MGCG_OPOLL_CNTL);
		data = (0reg |

	00PZs_POLL	ifhuF->RT_GnvIDE_S__PqC+me_MGLS_qT) |
			dev->pg_fex_u, 0, mmCP_		if (mplet	if (r)
		returnVERRIDE__GFXIP_MGL_compute_huF->RT_GnvIDE_S_ack +U00 << CP_e_MGL_cowe neCGLS_CTwe neCGLS_CT_COUNT(|
			;

		/* doesn't have PFPd_mask = reg |
< 32ULL);
ync PFPd	ifMEt amdgpu_deGCG_mmCP_M
		STATUS */PFPdipe;rn 0;
}		gfx_v9_0_enable_cp_poCs |= AMD_CG_SUPPPFP_SYN, mm, 0)T) 
}		gfx_v9_0_enable_cp_poC0x0x(struct amdgpu_p_flush(struct amdgpu_ring ;

		/*;
}

staticSUPPORT_GF_GF_reg_mem_engine;
	struct nbio_hdp_flush_reg *nbio_hf_reg;haSU
	gfx

	if (ring-> adev->asic_type == CHIP_VEGA10)
		nbio_h;

		/*;
}

staticSUPPORT_GF_GF_reg_mPE_COMPUTE) {
		switch (ring->me) {
		case 1:
			ref_and_mask = nbio_hf_reg->ref_anmask_cp2 << ring->pipe;
			break;
		casne;
	struct nbio_hdp_flusk = nbio_hf_r	N_MASK  
(ase)) - (1 hf_reg->ref_and_mask_cp0;
		reg_mem_engine =;

		/*;
}

staticSUPPORT_GF_GF_reg_mreg_mem(ring, reg_mem_engine, 0, 1,
			    {
		switch (ring->me) {
		case 1:
			ref_and_mask = nbio_hf_reg->ref_and_masset,
			      ref_and_mask, ref_and_mask, 0x20);
}

static void gfx_v9_0_ring_emit_hdp_invalidate(struct amdgpu_ring *ring)
{
{ASK  
(as C_CGTT_Sring_emi method s gfor;
	sonf_reg;n,
 		 RLC)
		*flags |= AMD_CG_SUPPORT_GFX_3fenc*urn ;
}

staticSUPPORT_GF_GF_Gype =>pg_ase_2(	ype =seq,mRLC_CGCG_CGCG/
	data = 			d enableallocCP_	if (ripg_fea,
_seqnHQD_PQ_WPTR		 R  

		iCGCG */
	data =FENCEREG__
64BITT) |
			dGC, 0fenc*_seqnOORBELL"_PQ_"GP)
			gfx_v9_0_enable_cp_poCs |= AMD_CG_SUPPWRITE*DATA, 3)	GRBM_STATUCCGCCGCCGCCGCCGCCRWRITE*DATAev,GINE_SEL(0)r GFX 3 WRITE*DATAeDST_SEL(5hi|_W= RREFIRM)	GRBM_STATUCCGCCGCCGCCGCCGCCe,
				   SOC1>pg_)	GRBM_STATUCCGCCGCCGCCGCCGCC gfx_v9_0_ring>pg_)	GRBM_STATUCCGCCGCCGCCGCCGCCe,
				   SOC1seq)	GRask = CGCG */
	data =FENCEREG__
INn< 32ULL);
ring-QD vmidOORBC, 0, mINnn 0;
}		gfx_v9_0_enable_cp_poCs |= AMD_CG_SUPPWRITE*DATA, 3)	GRBBM_STATUCCGCCGCCGCCGCCGCCRWRITE*DATAev,GINE_SEL(0)r GFX 33 WRITE*DATAeDST_SEL(0hi|_W= RREFIRM)	GRBBM_STATUCCGCCGCCGCCGCCGCC (data & RLC_CGCGBUG0), 1);
C_I
		soc1US)T) 
}		gfx_v9_0_enable_cp_poC0T) 
}		gfx_v9_0_enable_cp_poC0xdgpuuuuuas C_Csrc_GFX	ca178 		 RLC)
		*flags |= AMD_CG_SPORT_GFX_3sb;
}

staticSUPPORT_GF_GF_reg_m		gfx_v9_0_enable_cp_poCs |= AMD_CG_SUPPSWITCHCP_LS | A0)	GRBM_STATUCCGCCGCCGCCGCCGCCUNT(0x00900100) */
		def = RREG32_SOC1cCGCCGCC
}

staticSUPPORT_GF_GF_reg_mre00100bo_reservecCGib= RREG3cCGpay	ret}

	0};				   gds_csa->pg_;C(CGCGcnt_CG_SntCGCG_ve(rincCGpay	ret)evic2)C+m4 - nd_mcsa->pg_for RLC_SAVA				 RVEDtting - n	*_4096;GCCGCCGCCGCCGCCGCCGCCGCCGCCs |= AMD_CG_SUPPWRITE*DATA, Snt)	GRBM_STATUCCGCCGCCGCCGCCGCCRWRITE*DATAev,GINE_SEL(2)r GFX 3 WRITE*DATAeDST_SEL(8)r GFX 3 WR RREFIRM)r GFX 3 WRITE*DATAeCACHERPOLICY(0)	GRBM_STATUCCGCCGCCGCCGCCGCCe,
				   SOC1csa->pg_f+ RIDEM
rinbo_reserveFX_CGCGC_ (0x01cCGpay	ret))	GRBM_STATUCCGCCGCCGCCGCCGCC gfx_v9_0_ringcsa->pg_f+ RIDEM
rinbo_reserveFX_CGCGC_ (0x01cCGpay	ret))	GRBM_STATUCCGCCGCCGC_multipleCGCCGCCRfx_v9_0&cCGpay	retserve(rincCGpay	ret)evic2)T(0x00900100) */
		def = RREG32_SOC1dCGCCGCC
}

staticSUPPORT_GF_GF_reg_mre00100bo_reservedCGib= RREG3dCGpay	ret}

	0};				   gds_csa->pg_,ev)
{>pg_;C(CGCGcnt_CG_Ssa->pg_for RLC_SAVA				 RVEDtting - n	*_4096;G	v)
{>pg_forcsa->pg_f+ 4096;G	dCGpay	ret.gfx.prmdgp->pg_lofore,
				   SOC1v)
{>pg_);G	dCGpay	ret.gfx.prmdgp->pg_hX_PG gfx_v9_0_ringv)
{>pg_);GG_SntCGCG_ve(rindCGpay	ret)evic2)C+m4 - nd_mGCCGCCGCCGCCGCCGCCGCCGCCs |= AMD_CG_SUPPWRITE*DATA, Snt)	GRBM_STATUCCGCCGCCGCCGCCGCCRWRITE*DATAev,GINE_SEL(1)r GFX 3 WRITE*DATAeDST_SEL(8)r GFX 3 WR RREFIRM)r GFX 3 WRITE*DATAeCACHERPOLICY(0)	GRBM_STATUCCGCCGCCGCCGCCGCCe,
				   SOC1csa->pg_f+ RIDEM
rinbo_reserveFX_CGCGC_ (0x01dCGpay	ret))	GRBM_STATUCCGCCGCCGCCGCCGCC gfx_v9_0_ringcsa->pg_f+ RIDEM
rinbo_reserveFX_CGCGC_ (0x01dCGpay	ret))	GRBM_STATUCCGCCGCCGC_multipleCGCCGCCRfx_v9_0&dCGpay	retserve(rindCGpay	ret)evic2)T(0x00900100) */
		def =REG32_SOC1cntxcntl;
}

staticSUPPORT_GF_GF_GypLE_POLL_/
	data = DLE_POLL_Cw2CG_EN_MASK)
ta = RREG32_SOC15(GC, 0, mm
			/* 2 - RLREG32_SOC1cCGCCGCC_GF_rN_MACw2C|G_Ex8gpuuuuus C_Csrin	retuCHIP_RAamdgpu_deGth	caprmdagfx

	j	u32NOPsd_mask = CGCG */
	data =HAVE_CTXPSWITCH< 32ULL);
rin	retuIDE_MAt rifi{
	/	retuIDE_MAtu rifi{
 0;
}
w2C|G_Ex8gp1;2ULL);
rin	retucs_ = PACs
 0;
}
w2C|G_Ex
		 uuuus2ULL);
rin	retufx_v rinexng->pip
	/	retuI is = PACs
0_enable 0;
}
w2C|G_Ex		 u2update_coars	retuce_ramringpipem;
	st, enntCG_CGLS_k = CONST, PREAMeck IB PRESENnn&_/
	data	
}
w2C|G_Ex		 uuuuus2U		gfx_v9_0_ena(i = 0	retuce_ramringth	ca	caBELLfiru32fx_vgpipem;
	st, enntCG
		}

althoughaBELgfx

	nif (deexn CKET3(Phme)enspower_gatik = CONST, PREAMeck IB PRESENn_FIRSnn&_/
	data	
}
w2C|G_Ex		 uuuuus2U	GCCGCCGCCGCCGCCGCCGCCGCCGCCs |= AMD_CG_SUPPRRETEXTPRRETROLN_MA	GRBM_STATUCCGCCGCCGCCGCCGCC
w2	GRBM_STATUCCGCCGCCGCCGCCGCCUNT(0x00900100RLC_CGCG_MD_CG_SUPPORT_GFX_3Dx_v9 (dtuCxecRB_WPTR_POLL_CNTL);
		data = (RLC_CGCG_r = oaGCCGCCGCCGCCGCCGCCGCCGCCs |= AMD_CG_SUPPRRED_EXEC, 3)	GRBM_STATUCCGCCGCCGCCGCCGCCe,
				   SOC15_REG_ (dtuCxe_dev->pg_)	GRBM_STATUCCGCCGCCGCCGCCGCC gfx_v9_0_ring5_REG_ (dtuCxe_dev->pg_)	GRBM_STATUCCGCCGCCGCCGCCGCCuas C_CdiscaSUPfollowL);
DWsring* (dtuCxec_dev->pg_==0e_gfx_C15(Gmdgpu_ringn&_mdgpu_bufLE_POGRBM_STATUCCGCCGCCGCCGCCGCCux55aa55aaas C_CpaT3(Pdummy value rlc.re_gfx_C1engin = o0x00900100) */
		def = RREG32_SOC1EaT3(9 (dtuCxecRB_WPTR_POLL_CNTL);
		dat,mRLC_CGCG_RIDEM
a = (RLC_CGCG_curGRB  

		iRIDEM
 >_mdgpu_bufLE_PO	GRB  

		idp_flusp_f[RIDEM
]ring_x55aa55aaasGG_Sur	=ACKET3_Iringn&_mdgpu_bufLE_POCGCG1RBELL64 amdgpuSur	>_RIDEM
a)) {
		gfx_p_f[RIDEM
]r=_curGCGRIDEM
f_r	N_MASK
		gfx_p_f[RIDEM
]r=_idp_flusp_f_CG_S>>2)GCGRIDEM
f+ curGR0x00900100) */
		def = RREG32_SOC1tmzRB_WPTR_POLL_CNTL);
		dat,mcase 090rtreg_m		gfx_v9_0_enable_cp_poCs |= AMD_CG_SUPPFRAMEPRRETROLN_0)	GRBM_STATUCCGCCGCCGCCGCCGCCFRAMEPRMD(090rt ? 0 : thas C_Cfram_t gdM_SLP_CNTL__RLC_MEM_LS_EN_MAREG32_SOC1r	dat
}

staticSUPPORT_GF_GF_GypLE_POLL_	dareg_mreg_mem(ring, reg_mem_engine, 0, 1,
			    GCCGCCGCCGCCGCCGCCGCCGCCs |= AMD_CG_SUPPRRPY*DATA, 4)	GRBM_STATUCCGCCGCCGCCGCCGCCu |	C_Csrc:ng-QD vmi_CNTL)	(5e ne8)r  gfx_st:(GC, 0, _CNTL)	(we ne20)	GOC15(GC, 0,rifirmGP)
			gfx_v9_0_enable_cp_poC	darGRBM_STATUCCGCCGCCGCCGCCGCCUNT(BM_STATUCCGCCGCCGCCGCCGCCe,
				   SOC1>pef_and_dev->pg_f+U00 gating_virt.IDE_val}

st	*_4)	GRBM_STATUCCGCCGCCGCCGCCGCC gfx_v9_0_ring>pef_and_dev->pg_f+U00 gating_virt.IDE_val}

st	*_4)	GRP_CNTL__RLC_MEM_LS_EN_MAREG32_SOC1w	dat
}

staticSUPPORT_GF_GF_GypLE_POLL_	das_POLL	i *adev = (alreg_m		gfx_v9_0_enable_cp_poCs |= AMD_CG_SUPPWRITE*DATA, 3)	GRBM_STATUCCGCCGCCGCCGCCGCCRwe ne16has C_Cnifinc_ELOADP)
			gfx_v9_0_enable_cp_poC	darGRBM_STATUCCGCCGCCGCCGCCGCCUNT(BM_STATUCCGCCGCCGCCGCCGCC(alrGRP_CNTL__RLC_MEM_LS_EN_MAC, 0,itchopr in
		 * D_CG_SU32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRGFX_RLC__GF_Gin
		 * D_CG_S32_SOC15(GC,KET3(PA2_SOC1RT_GFX_MG	data = RQ & CP_MDISheck:_GFX_MG	data = RQ & CP_M check:
		if (!REG_GET_FIELD(tmp, GI
		 *ade2);
_CU00 << CP_ 0, m & CM GI
		 checkCU00 << CP_ NTL);
	if (dta = RQ & CP_M check ? 1 : 0);S_EN_MASK)
		*flags |= AMD_CG_SUP_CNTL__RLC_MEM_LS_EN_MAC, 0uint32_t opr in
		 * D_CG_SU32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRRRRRCGCGme,RCGCGERRI_CGLS_CTRRRRRGFX_RLC__GF_Gin
		 * D_CG_S32_SOC15(GCRCGCmec_iingcntl,Cmec_iingcntl PACKET3		
	}

LC__GFC ring bsenableBELLfiru32MEC. That't; iygth	caNDIRtionenabl
	}

ta & RcaBELLable)
{
of Gin
		 * sd].mem_bs specif_RLMEC. A= 0amdgp
	}

ERRIs' Gin
		 * sd	gfxEM
fby
LC_kfd. 	d_maBELL64m;
	if1r = PACKET3(PAERRIDIRECT_BUFF0IDE_Mmec_iingcntl PACne, (data & RLC_CGCGBUG0), 1);
_ME1*/
	i0GI
		 *ad)_sriov_vf(
	data = 1IDE_Mmec_iingcntl PACne, (data & RLC_CGCGBUG0), 1);
_ME1*/
	i1GI
		 *ad)_sriov_vf(
	data = 2IDE_Mmec_iingcntl PACne, (data & RLC_CGCGBUG0), 1);
_ME1*/
	i2GI
		 *ad)_sriov_vf(
	data = 3IDE_Mmec_iingcntl PACne, (data & RLC_CGCGBUG0), 1);
_ME1*/
	i3GI
		 *ad)_sriov_vf(
	daK_OVERRIDE_MDRM__CGLS("TATUS */ERRI %dDGPU_ERRID;DE_MASK |
			 RLC		gfx_v9_0_DRM__CGLS("TATUS */mI %dDGPU_mID;DE_ASK |
			}
GC,KET3(PA2_SOC1RT_GFX_MG	data = RQ & CP_MDISheck:_GMmec_iingcntlk_cp6 << (mec_iingcntl PACD;DE_mec_iingcntlk_cp, 0, mmRLC_GPmec_iingcntl,C;
_ME1*/
	i0GI
		 *advice *)hand 0, m & CM GI
		 checkC 0);S_EW6 << (mec_iingcntl PAC,Cmec_iingcntl);S_EN_MASK)
FX_MG	data = RQ & CP_M check:
		mec_iingcntlk_cp6 << (mec_iingcntl PACD;DE_mec_iingcntlk_cp, 0, mmRLC_GPmec_iingcntl,C;
_ME1*/
	i0GI
		 *advice *)hand 0, m & CM GI
		 checkC 1);S_EW6 << (mec_iingcntl PAC,Cmec_iingcntl);S_EN_MASK)
		*flags |= AMD_CG_SUP_CNTL__RL
{
	struct amdgpu_(!adev->*flagD_CG_SU32_SOC15(GC, 0, mmRLC_CGCG_CGLS_C CP_ NTGF_GF_GF_GF_Grq_src *source_CGLS_C CP_ RLC_CGCG__BUF_CGLS_C CP_ GFX_RLC__GF_Gin
		 * D_CG_S32_SOC15(GC,KET3(PA2_SOC1RT_GFX_MG	data = RQ & CP_MDISheck:_GFX_MG	data = RQ & CP_M check:
		if (!REG_GET_FIELD(tmp, GI
		 *ade2);
_CU00 << CP_ PRIVa & RI
		 checkCU00 << CP_ NTL);
	if (dta = RQ & CP_M check ? 1 : 0);S_EN_MASK)
		*flags |= AMD_CG_SU_CTRL__CGLS_EN_MASK)
		*flags |= AMD_CGing */
	rl*flagD_CG_SU32_SOC15(GC, 0, mmRLC_CGCG_CGLS_C CP_  NTGF_GF_GF_GF_Grq_src *source_CGLS_C CP_  RLC_CGCG__BUF_CGLS_C CP_  GFX_RLC__GF_Gin
		 * D_CG_S32_SOC15(GC,KET3(PA2_SOC1RT_GFX_MG	data = RQ & CP_MDISheck:_GFX_MG	data = RQ & CP_M check:
		if (!REG_GET_FIELD(tmp, GI
		 *ade2);
_CU00 << CP_ PRIVaINSTRRI
		 checkCU00 << CP_ NTL);
	if (dta = RQ & CP_M check ? 1 : 0);S_		*flags |= AMD_CG_SU_CTRL__CGLS_EN_MASK)
		*flags |= AMD_CG opr in
		 * D_CG_SU32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CP_  NTGF_GF_GF_GF_Grq_src *src_CGLS_CP_  RLC_CGCG__BUF_CGLS_C CP_GFX_RLC__GF_Gin
		 * D_CG_S32_SOC15(GC,KET3(PASUPPORT_GFX_MG (dta =, GIRQ d gfEOPP_LS */
	dataC, 0,itchopr in
		 * D_CG_SU0, m,_2_SOC1;S_EN_MASK)
FX_MG	data =, GIRQ ODE */
	MEC1*/
	i0GEOPP_LS */
	dataC, 0uint32_t opr in
		 * D_CG_SU0, m,_	iS_C_2_SOC1;S_EN_MASK)
FX_MG	data =, GIRQ ODE */
	MEC1*/
	i1GEOPP_LS */
	dataC, 0uint32_t opr in
		 * D_CG_SU0, m,_	iS1C_2_SOC1;S_EN_MASK)
FX_MG	data =, GIRQ ODE */
	MEC1*/
	i2GEOPP_LS */
	dataC, 0uint32_t opr in
		 * D_CG_SU0, m,_	iS2C_2_SOC1;S_EN_MASK)
FX_MG	data =, GIRQ ODE */
	MEC1*/
	i3GEOPP_LS */
	dataC, 0uint32_t opr in
		 * D_CG_SU0, m,_	iS3C_2_SOC1;S_EN_MASK)
FX_MG	data =, GIRQ ODE */
	MEC2*/
	i0GEOPP_LS */
	dataC, 0uint32_t opr in
		 * D_CG_SU0, m,_2iS_C_2_SOC1;S_EN_MASK)
FX_MG	data =, GIRQ ODE */
	MEC2*/
	i1GEOPP_LS */
	dataC, 0uint32_t opr in
		 * D_CG_SU0, m,_2iS1C_2_SOC1;S_EN_MASK)
FX_MG	data =, GIRQ ODE */
	MEC2*/
	i2GEOPP_LS */
	dataC, 0uint32_t opr in
		 * D_CG_SU0, m,_2iS2C_2_SOC1;S_EN_MASK)
FX_MG	data =, GIRQ ODE */
	MEC2*/
	i3GEOPP_LS */
	dataC, 0uint32_t opr in
		 * D_CG_SU0, m,_2iS3C_2_SOC1;S_EN_MASK)
		*flags |= AMD_CG_SUPPORT_GFX_CP_LS | AMD|
	     AMD_CGopr r ;
}

staticSUPP, mmRLC_CGCG_CGLSP_  NTGF_GF_GF_GF_Grq_src *source_CGLSP_  NTGF_GF_GF_GF_GvIDEt0, _DEt0,a = (CGCGi;			8CmeLS_CTERRILS_CTqueuILS_;_mreg_mem(ring, ORT_GF_GF_KET3DRM__CGLS("IH:_SUPEOPDGP1;S_meLS_r=_iDEt0,lusp_f_S_rlags0c)evic2;S_ERRILS_r=_iDEt0,lusp_f_S_rlags03)evicus2UqueuILS_r=_iDEt0,lusp_f_S_rlags70)evic4RT_GFX_RLC_LmeLS_ORT_GFX_MG0IDE_(ring, fenc*upro 1:
			   (1 << (;
		WRE[0]1;S_EN_MASK)
FX_MG1:_GFX_MG2IDE_RT_GFX_PG) && enable) g, data);

		/* wait _enable_gfx_ORT_G=led, do noth;

		/* wait[i];DE_M/* Per-queuI Gin
		 * x

	s gfor;
	sRT_Guint090rtRT_Gfrom VI.CGLSP_* ThI Gin
		 * xcanenablrue : false/_power_doper/ERRI /
	reaG_RIoper/queuI.CGLSP_*CNTL);
		ACKET3_IN
	ifmeLS_ORLC_MCKET3_ERRI 	ifERRILS_ORLC_MCKET3_queuI 	ifqueuILS_LINE)
	(ring, fenc*upro 1:
	_GF_rN_	 RLC_ AMD_CG_SUPPORT_GFX_CP_LS | AMD|
	     AMD_C (!adev->gfxU32_SOC15(GC, 0, mmRLC_CGCG_CGLS_ NTGF_GF_GF_GF_Grq_src *source_CGLS_ NTGF_GF_GF_GF_GvIDEt0, _DEt0,a = (DRM_ERROR("Illegalng-QD vmidac 1:
mode;

mlc_sNTGeamDGP1;S_schedule_wor				   (1, ena_wor	ase)) - (1 X_CP_LS | AMD|
	     AMD_C (!ad/
	rlc_sU32_SOC15(GC, 0, mmRLC_CGCG_CGLS_  NTGF_GF_GF_GF_Grq_src *source_CGLS_  NTGF_GF_GF_GF_GvIDEt0, _DEt0,a = (DRM_ERROR("Illegaln/
	rGF_Gioneode;

mlc_sNTGeamDGP1;S_schedule_wor				   (1, ena_wor	ase)) - (1 X_CP_LS | AMD|
	     AMD_Ckiqf (!REin
		 * D_CG_SU32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CP_  NTGF_GF_GF_GF_Grq_src *src_CGLS_CP_  RLC_CGCG_|
	 _BUF_CGLS_C CP_GFX_RLC__GF_Gin
		 * D_CG_S32_SOC15(GC *adev = tm

	0argM
f_rreg_mem(ring, ORT_GF_GF_G=le AMD_CG_SUPkiq._GF_rN_MAk = nbio_hm;
	if1rCGL0argM
ne, (data & RLC_CGCGBUG0), 1);
_ME1*/
	i0GI
		 *ad)_sr	N_MASK0argM
ne, (data & RLC_CGCGBUG0), 1);
_ME2*/
	i0GI
		 *ad)_sr0argM
n+=CG_OVERRIDE__GC,KET3(PASUPPORT_GFX_MG (dta =, GKIQGIRQ DRIVER0se);
		bNTL);
	if (dta = RQ & CP_MDISheckle_gfx__0_write_data_to_reg(ring, 0;
C_I
		 *ad)_srio_0_writ, 0, mmRLC_GPtm

	;
C_I
		 *ad_CGLS_CTRGENERIC2GI
		 checkC 0);S_EP, 0, mmHDP_DEBUG0), 1);
C_I
		 *ad_*/
	gfxgfx__0_write_dat(0argM
)_srio_0_writ, 0, mmRLC_GPtm

	;
_ME2*/
	i0GI
		 *ad_CGLS_CTRGENERIC2GI
		 checkC 0);S_EP, 0, m(0argM
_*/
	gfx *)handle;
	in_0_write_data_to_reg(ring, 0;
C_I
		 *ad)_srio_0_writ, 0, mmRLC_GPtm

	;
C_I
		 *ad_CGLS_CTRGENERIC2GI
		 checkC 1);S_EP, 0, mmHDP_DEBUG0), 1);
C_I
		 *ad_*/
	gfxgfx__0_write_dat(0argM
)_srio_0_writ, 0, mmRLC_GPtm

	;
_ME2*/
	i0GI
		 *ad_CGLS_CTRGENERIC2GI
		 checkC 1);S_EP, 0, m(0argM
_*/
	gfx *)S_EN_MASK)
		*flags |=  
(as C_CkiquGTT_Sg gfor;RGENERIC2GI
	;n,
 		 R_ AMD_CG_SUPPORT_GFX_CP_LS | AMD|
	     AMD_Ckiqf r ;
}

staticSUPP, mmRLC_CGCG_CGLSP_  NTGF_GF_GF_GF_Grq_src *source_CGLSP_  NTGF_GF_GF_GF_GvIDEt0, _DEt0,a = (	8CmeLS_CTERRILS_CTqueuILS_;_mreg_mem(ring, ORT_GF_GF_G=le AMD_CG_SUPkiq._GF_rN_MAmeLS_r=_iDEt0,lusp_f_S_rlags0c)evic2;S_ERRILS_r=_iDEt0,lusp_f_S_rlags03)evicus2UqueuILS_r=_iDEt0,lusp_f_S_rlags70)evic4RT3DRM__CGLS("IH:_SUCRGENERIC2GI
	,Cme:%_CTERRI:%_CTqueuI:%dDGPU
LSP_ meLS_CTERRILS_CTqueuILS_)    GCCGCCGfenc*upro 1:
	_GF_rN_	ate_gfx_mg_power_gating(adev, enable_ipeak;
	default:
	ipeak;
	d

	retnam;
	 "efault:
"state_SUPPORT_,
					  enum_SUPPORT_statrlc.in_sa,
					  enurlc.in_sastatswin_sa,
					  enuswin_sastatswifn_s,
					  enuswifn_sstathwin_sa,
					  enuhwin_sastathwifn_s,
					  enuhwifn_sstatsusp gdM
					  enususp gdstat, eume,
					  enu, eumestatisLS_le,
					  enuisLS_lestatnVERRRT_LS_le,
					  enunVERRRT_LS_lestatsofRRIDsea,
					  enusofRRIDseastats_MGLS;

	/* AMD_CG_S,
					  enus_MGLS;

	/* AMD_CG_Sstats_MG
	if (amdgpu_sriov
					  enus_MG
	if (amdgpu_sriostatX_MGLS;

	/* AMD_CG_Sv
					  enuX_MGLS;

	/* AMD_CG_S,state state)ing(adev, enable);
		iusec_timM_LS_EN_MAREG32c_tim0,itd

	ret_BUFFE_CONST, 2);
	else
GFXstatPORT_RIDE_MASSOC1statnoplags |= AMD_CG_SUPPNOGLS_x3FFF)statsugfor;_64 (r_ptr	d

00PZs_P.vmhuF_E_CONST, GFXHUBstatX_MGr_cp2 <ush(struct amdgpu_ring *ristatX_MGw_cp2 <ush(struct amdgpu_wing *ristats_MGw_cp2 <ush(struct amdspu_wing *ristatGFX_3fram_tCG_S2 <C_CtotalT_S242 maximumring16 IBs
 0;
}5 + <C_CRRED_EXEC
 0;
}7 + <C_C/
	if (e_SYN,
 0;
}24 + C_CVMREGUSH
 0;
}8 + <C_CFENCESCLK_OMREGUSH
 0;
}20 + C_CGDS CKET3(P 0;
}4 + C_C			  RLSWITCHCP_LS | 
0 << CP_ BELLfiru32RRED_EXEC
ju0_wOORBELLplaRLCj	u3CGLSP_  (!oidOORB_bs 			  RLSWITCHCP_LS | 
 0;
}5 + C_CRRED_EXEC
 0;
}7 +	 C_	_3D_T_MGC  0;
}4 +	 C_	VGT_T_MGC  0;
}14 + C_	C
	META  0;
}31 + C_	D
	META  0;
}3 + C_CRNTXTR_HI
 0;
}5 + C_C_3D_INVL
 0;
}8 + 8 + C_CFENCESx2
 0;
}2, C_CSWITCHCP_LS |  0;
.GFX_3D_CCG_S2 	4, RLC_MEM_SLP_PORT_GFX_3D_CGLS  0;
.GFX_3D_2 <ush(struct amdGFX_3D_CGLSstatGFX_3fenc*_ <ush(struct amdGFX_3fenc*statGFX_3ERRIDE);
sync_ <ush(struct amdGFX_3ERRIDE);
syncstatGFX_3RT_     _ <ush(struct amdGFX_3RT_     statGFX_3v)
{CKET3(P <ush(struct amdGFX_3v)
{CKET3(statGFX_3_CGTT_MGCP <ush(struct amdGFX_3_CGTT_MGCstatGFX_3_CGT, 0, mmCP_	 <ush(struct amdGFX_3_CGT, 0, mmCP_statte	rl_GF_G=lush(struct amdte	rl_GF_statte	rlD_2 <ush(struct amdte	rlD_statinser;_noplagble);
		iuseinser;_nopstatpadlD_2 <ble);
		iusegeneric_padlD_statGFX_3CKET3(_buffep2 <ush(strt amdGFX_3s_statGFX_3cntxcntl2 <ush(strt amdGFX_3cntxcntlstatin_v9 (dtuCxec2 <ush(struct amdGFX_3Dn_v9 (dtuCxecstatpaT3(9 (dtuCxec_ <ush(struct amdGFX_3EaT3(9 (dtuCxecstatGFX_3tmz_ <ush(struct amdGFX_3tmz,state state)ing(adev, enable);
		iusec_timM_LS_EN_MAREG32c_tim0;

		/* 

	ret_BUFFE_CONST, 2);
	else
ODE */
statPORT_RIDE_MASSOC1statnoplags |= AMD_CG_SUPPNOGLS_x3FFF)statsugfor;_64 (r_ptr	d

00PZs_P.vmhuF_E_CONST, GFXHUBstatX_MGr_cp2 <ush(struct amdgpu_ring ;

		/*statX_MGw_cp2 <ush(struct amdgpu_wing ;

		/*stats_MGw_cp2 <ush(struct amdspu_wing ;

		/*statGFX_3fram_tCG_S2 ;
}20 + C_Cush(struct amdGFX_3v)
{CKET3(
 0;
}7 + C_Cush(struct amdGFX_3_CGTT_MGCP 0;
}5 + C_Cush(struct amdGFX_3_CGT, 0, mmCP_
 0;
}7 + C_Cush(struct amdGFX_3ERRIDE);
sync_ 0;
}24 + C_Cush(struct amdGFX_3RT_     
 0;
}8 + 8 + 8, RLC_MEM_SLP_PORT_GFX_3fenc*_x3SCLK_user0fenc*, vm0fenc*_ 0;
.GFX_3D_CCG_S2 	4, RLC_MEM_SLP_PORT_GFX_3D_C;

		/*  0;
.GFX_3D_2 <ush(struct amdGFX_3D_C;

		/*statGFX_3fenc*_ <ush(struct amdGFX_3fenc*statGFX_3ERRIDE);
sync_ <ush(struct amdGFX_3ERRIDE);
syncstatGFX_3RT_     _ <ush(struct amdGFX_3RT_     statGFX_3v)
{CKET3(P <ush(struct amdGFX_3v)
{CKET3(statGFX_3_CGTT_MGCP <ush(struct amdGFX_3_CGTT_MGCstatGFX_3_CGT, 0, mmCP_	 <ush(struct amdGFX_3_CGT, 0, mmCP_statte	rl_GF_G=lush(struct amdte	rl_GF_statte	rlD_2 <ush(struct amdte	rlD_statinser;_noplagble);
		iuseinser;_nopstatpadlD_2 <ble);
		iusegeneric_padlD_sttate state)ing(adev, enable);
		iusec_timM_LS_EN_MAREG32c_tim0kiqu

	ret_BUFFE_CONST, 2);
	else
KIQstatPORT_RIDE_MASSOC1statnoplags |= AMD_CG_SUPPNOGLS_x3FFF)statsugfor;_64 (r_ptr	d

00PZs_P.vmhuF_E_CONST, GFXHUBstatX_MGr_cp2 <ush(struct amdgpu_ring ;

		/*statX_MGw_cp2 <ush(struct amdgpu_wing ;

		/*stats_MGw_cp2 <ush(struct amdspu_wing ;

		/*statGFX_3fram_tCG_S2 ;
}20 + C_Cush(struct amdGFX_3v)
{CKET3(
 0;
}7 + C_Cush(struct amdGFX_3_CGTT_MGCP 0;
}5 + C_Cush(struct amdGFX_3_CGT, 0, mmCP_
 0;
}7 + C_Cush(struct amdGFX_3ERRIDE);
sync_ 0;
}24 + C_Cush(struct amdGFX_3RT_     
 0;
}8 + 8 + 8, RLC_MEM_SLP_PORT_GFX_3fenc*urn _x3SCLK_user0fenc*, vm0fenc*_ 0;
.GFX_3D_CCG_S2 	4, RLC_MEM_SLP_PORT_GFX_3D_C;

		/*  0;
.GFX_3D_2 <ush(struct amdGFX_3D_C;

		/*statGFX_3fenc*_ <ush(struct amdGFX_3fenc*urn statte	rl_GF_G=lush(struct amdte	rl_GF_statte	rlD_2 <ush(struct amdte	rlD_statinser;_noplagble);
		iuseinser;_nopstatpadlD_2 <ble);
		iusegeneric_padlD_statGFX_3rPACne,_LS_EN_MAREG32_SOC1r	dastatGFX_3wPACne,_LS_EN_MAREG32_SOC1w	dasttate state)C_MEM_LS_EN_MAC, 0REG32c_tim;
}

staticSUPP, mmRLC_CGCGa = (CGCGi;	rgating_statkiq._GF_.ak;
	d

&_LS_EN_MAREG32c_tim0kiq;	rgRT_GFX_PG) && enable) g, data)(;
		WRE _enabl
			   (1 << (;
		WRE[i].ak;
	d

&_LS_EN_MAREG32c_tim0_LS;	rgRT_GFX_PG) && enable) g, data);

		/* wait _enabl
			   (1 << ;

		/* wait[i].ak;
	d

&_LS_EN_MAREG32c_tim0;

		/*mg_power_gating(adev, enable);
	Grq_srcec_timM_LS_EN_MAkiqf r eak;
	d

	retsea,
					  enukiqf (!REin
		 * D_CG_Sstatpro 1:
,
					  enukiqf r sttate state)ing(adev, enable);
	Grq_srcec_timM_LS_EN_MAGopr r eak;
	d

	retsea,
					  enuD_CG opr in
		 * D_CG_Sstatpro 1:
,
					  enuGopr r sttate state)ing(adev, enable);
	Grq_srcec_timM_LS_EN_MA (!adev->gfxeak;
	d

	retsea,
					  enuD_CG_(!adev->*flagD_CG_Sstatpro 1:
,
					  enu (!adev->gfxsttate state)ing(adev, enable);
	Grq_srcec_timM_LS_EN_MA (!ad/
	rlc_seak;
	d

	retsea,
					  enuD_CG_(!ad/
	rl*flagD_CG_Sstatpro 1:
,
					  enu (!ad/
	rlc_ssttate state)C_MEM_LS_EN_MAC, 0c_seak;
	;
}

staticSUPP, mmRLC_CGCGa = (	   (1 << Gopr r data)_BUFs_E_CONST, , GIRQ LAST:
		   (1 << Gopr r dak;
	d

&_LS_EN_MAGopr r eak;
	;	rgating_stat (!adev->gfxdata)_BUFs_E_1;rgating_stat (!adev->gfxdak;
	d

&_LS_EN_MA (!adev->gfxeak;
	;	rgating_stat (!ad/
	rlc_sdata)_BUFs_E_1;rgating_stat (!ad/
	rlc_sdak;
	d

&_LS_EN_MA (!ad/
	rlc_seak;
	;	rgating_statkiq. r data)_BUFs_E_CONST, , GKIQGIRQ LAST:
		   (1 << kiq. r dak;
	d

&_LS_EN_MAkiqf r eak;
	GRP_CNTL__RLC_MEM_LS_EN_MAC, 0	break;
	;
}

staticSUPP, mmRLC_CGCGa = (FX_RLC_LS | AMD_CG_SUPPORT_GFX_MGLS;

	/* AMD_CG_SUPPORT_GFX_CP_LSating_state(void *hd

&_LS_EN_MARbreak;
	;S_EN_MASK)
		*flags |= AMD_CG_SUP_CNTL__RLC_MEM_LS_EN_MAC, 0v)
{Dn_v;
}

staticSUPP, mmRLC_CGCGa = (RLCn_sa,asciev)
Cn_foDP)
		ting_sds.mem.totaltCG_S2 <te_data_to_reg(ring, 0GDS_VMID0tting);
		ting_sds.gws.totaltCG_S2 <64;
		ting_sds.oa.totaltCG_S2 <16N_MASK)
tting_sds.mem.totaltCG_S2  <64 * 1024nd_massting_sds.mem._LS_p0rtRGiontCG_S2 <4096;G	ssting_sds.mem.im0p0rtRGiontCG_S2 <4096;GG	ssting_sds.gws._LS_p0rtRGiontCG_S2 <4;G	ssting_sds.gws.im0p0rtRGiontCG_S2 <4;GG	ssting_sds.oa._LS_p0rtRGiontCG_S2 <4;G	ssting_sds.oa.im0p0rtRGiontCG_S2 <1;rg)handle;
	isting_sds.mem._LS_p0rtRGiontCG_S2 <1024;G	ssting_sds.mem.im0p0rtRGiontCG_S2 <1024;GG	ssting_sds.gws._LS_p0rtRGiontCG_S2 <16;G	ssting_sds.gws.im0p0rtRGiontCG_S2 <16;GG	ssting_sds.oa._LS_p0rtRGiontCG_S2 <4;G	ssting_sds.oa.im0p0rtRGiontCG_S2 <4CG_SUP_CNTL__RLC_MEM_LS_EN_MAC, 0user_cF_Giactive0_rimapU32_SOC15(GC, 0, mmRLC_CGCG_CGLS_CTRRCGC_rimap15(GCRCGC_CGLS_CTRL_3!_rimap15E_ASK |
		t*)&adev-_rimape neGC_USER_SHADER_ARRAY RREFIG__IN	}

VE_CUS /CGLS ;
}

staticGC_USER_SHADER_ARRAY RREFIG__IN	}

VE_CUS ate_3d
P, 0, mmHDP_DEBUG0), 1)GC_USER_SHADER_ARRAY RREFIG9_0_set_p0x00900100RCGC				  enuX_MGLu_active0_rimapU32_SOC15(GC, 0, mmRLC_CGCG15(GCRCGC_CGL, 1_POGRv9_0_ring_set_wptr_gfx(struct amC_GC_SHADER_ARRAY RREFIG);
}

sta| <te_data_to_reg(ring, 0GC_USER_SHADER_ARRAY RREFIG)GRv9_0_riticmC_GC_SHADER_ARRAY RREFIG__IN	}

VE_CUS ate_3d9_0_ri>>icmC_GC_SHADER_ARRAY RREFIG__IN	}

VE_CUS /CGLS ;

	IDE_MAS* if RLs			cAMDte0_rimask(	   (1 << ;
ifi{.maxGLu_fx_vsh)GRv9ate_gfx(~0_set & 1_POGRP_LS | AMD|
	     AMD_CX_MGLu_n_foU32_SOC15(GC, 0, mmRLC_CGCG_CGLS_ NTGF_GF_GF_GF_Lu_n_fo *Lu_n_foa = (CGCGi, j, kCC ruin
	, active0Lu_atabep2 <us2URCGCm_POLL_rimap, ao__rimap, ao_Lu_IDE_MASS; (RLC_CGCG__power__IDE_s[4 * 2]S_CTRL_3!engin|| !Lu_n_foa D_CG_SUPP-EINVAL    GCCGCCG_LS_p0r_regpower__cu(_power__IDE_s, 4 AMD_CG_m	/*x_S;

			   (1 rbGLS_x_m	/*x);
}RT_GFX_PG) && enable) g, d;
ifi{.maxGshC	v IDE_MAS _enable_gfxRT_GFj_PG) &j enable) g, d;
ifi{.ma