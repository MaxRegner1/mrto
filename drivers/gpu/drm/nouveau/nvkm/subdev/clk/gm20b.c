/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
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
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <subdev/clk.h>
#include <subdev/volt.h>
#include <subdev/timer.h>
#include <core/device.h>
#include <core/tegra.h>

#include "priv.h"
#include "gk20a.h"

#define GPCPLL_CFG_SYNC_MODE	BIT(2)

#define BYPASSCTRL_SYS	(SYS_GPCPLL_CFG_BASE + 0x340)
#define BYPASSCTRL_SYS_GPCPLL_SHIFT	0
#define BYPASSCTRL_SYS_GPCPLL_WIDTH	1

#define GPCPLL_CFG2_SDM_DIN_SHIFT	0
#define GPCPLL_CFG2_SDM_DIN_WIDTH	8
#define GPCPLL_CFG2_SDM_DIN_MASK	\
	(MASK(GPCPLL_CFG2_SDM_DIN_WIDTH) << GPCPLL_CFG2_SDM_DIN_SHIFT)
#define GPCPLL_CFG2_SDM_DIN_NEW_SHIFT	8
#define GPCPLL_CFG2_SDM_DIN_NEW_WIDTH	15
#define GPCPLL_CFG2_SDM_DIN_NEW_MASK	\
	(MASK(GPCPLL_CFG2_SDM_DIN_NEW_WIDTH) << GPCPLL_CFG2_SDM_DIN_NEW_SHIFT)
#define GPCPLL_CFG2_SETUP2_SHIFT	16
#define GPCPLL_CFG2_PLL_STEPA_SHIFT	24

#define GPCPLL_DVFS0	(SYS_GPCPLL_CFG_BASE + 0x10)
#define GPCPLL_DVFS0_DFS_COEFF_SHIFT	0
#define GPCPLL_DVFS0_DFS_COEFF_WIDTH	7
#define GPCPLL_DVFS0_DFS_COEFF_MASK	\
	(MASK(GPCPLL_DVFS0_DFS_COEFF_WIDTH) << GPCPLL_DVFS0_DFS_COEFF_SHIFT)
#define GPCPLL_DVFS0_DFS_DET_MAX_SHIFT	8
#define GPCPLL_DVFS0_DFS_DET_MAX_WIDTH	7
#define GPCPLL_DVFS0_DFS_DET_MAX_MASK	\
	(MASK(GPCPLL_DVFS0_DFS_DET_MAX_WIDTH) << GPCPLL_DVFS0_DFS_DET_MAX_SHIFT)

#define GPCPLL_DVFS1		(SYS_GPCPLL_CFG_BASE + 0x14)
#define GPCPLL_DVFS1_DFS_EXT_DET_SHIFT		0
#define GPCPLL_DVFS1_DFS_EXT_DET_WIDTH		7
#define GPCPLL_DVFS1_DFS_EXT_STRB_SHIFT		7
#define GPCPLL_DVFS1_DFS_EXT_STRB_WIDTH		1
#define GPCPLL_DVFS1_DFS_EXT_CAL_SHIFT		8
#define GPCPLL_DVFS1_DFS_EXT_CAL_WIDTH		7
#define GPCPLL_DVFS1_DFS_EXT_SEL_SHIFT		15
#define GPCPLL_DVFS1_DFS_EXT_SEL_WIDTH		1
#define GPCPLL_DVFS1_DFS_CTRL_SHIFT		16
#define GPCPLL_DVFS1_DFS_CTRL_WIDTH		12
#define GPCPLL_DVFS1_EN_SDM_SHIFT		28
#define GPCPLL_DVFS1_EN_SDM_WIDTH		1
#define GPCPLL_DVFS1_EN_SDM_BIT			BIT(28)
#define GPCPLL_DVFS1_EN_DFS_SHIFT		29
#define GPCPLL_DVFS1_EN_DFS_WIDTH		1
#define GPCPLL_DVFS1_EN_DFS_BIT			BIT(29)
#define GPCPLL_DVFS1_EN_DFS_CAL_SHIFT		30
#define GPCPLL_DVFS1_EN_DFS_CAL_WIDTH		1
#define GPCPLL_DVFS1_EN_DFS_CAL_BIT		BIT(30)
#define GPCPLL_DVFS1_DFS_CAL_DONE_SHIFT		31
#define GPCPLL_DVFS1_DFS_CAL_DONE_WIDTH		1
#define GPCPLL_DVFS1_DFS_CAL_DONE_BIT		BIT(31)

#define GPC_BCAST_GPCPLL_DVFS2	(GPC_BCAST_GPCPLL_CFG_BASE + 0x20)
#define GPC_BCAST_GPCPLL_DVFS2_DFS_EXT_STROBE_BIT	BIT(16)

#define GPCPLL_CFG3_PLL_DFS_TESTOUT_SHIFT	24
#define GPCPLL_CFG3_PLL_DFS_TESTOUT_WIDTH	7

#define DFS_DET_RANGE	6	/* -2^6 ... 2^6-1 */
#define SDM_DIN_RANGE	12	/* -2^12 ... 2^12-1 */

struct gm20b_clk_dvfs_params {
	s32 coeff_slope;
	s32 coeff_offs;
	u32 vco_ctrl;
};

static const struct gm20b_clk_dvfs_params gm20b_dvfs_params = {
	.coeff_slope = -165230,
	.coeff_offs = 214007,
	.vco_ctrl = 0x7 << 3,
};

/*
 * base.n is now the *integer* part of the N factor.
 * sdm_din contains n's decimal part.
 */
struct gm20b_pll {
	struct gk20a_pll base;
	u32 sdm_din;
};

struct gm20b_clk_dvfs {
	u32 dfs_coeff;
	s32 dfs_det_max;
	s32 dfs_ext_cal;
};

struct gm20b_clk {
	/* currently applied parameters */
	struct gk20a_clk base;
	struct gm20b_clk_dvfs dvfs;
	u32 uv;

	/* new parameters to apply */
	struct gk20a_pll new_pll;
	struct gm20b_clk_dvfs new_dvfs;
	u32 new_uv;

	const struct gm20b_clk_dvfs_params *dvfs_params;

	/* fused parameters */
	s32 uvdet_slope;
	s32 uvdet_offs;

	/* safe frequency we can use at minimum voltage */
	u32 safe_fmax_vmin;
};
#define gm20b_clk(p) container_of((gk20a_clk(p)), struct gm20b_clk, base)

static u32 pl_to_div(u32 pl)
{
	return pl;
}

static u32 div_to_pl(u32 div)
{
	return div;
}

static const struct gk20a_clk_pllg_params gm20b_pllg_params = {
	.min_vco = 1300000, .max_vco = 2600000,
	.min_u = 12000, .max_u = 38400,
	.min_m = 1, .max_m = 255,
	.min_n = 8, .max_n = 255,
	.min_pl = 1, .max_pl = 31,
};

static void
gm20b_pllg_read_mnp(struct gm20b_clk *clk, struct gm20b_pll *pll)
{
	struct nvkm_subdev *subdev = &clk->base.base.subdev;
	struct nvkm_device *device = subdev->device;
	u32 val;

	gk20a_pllg_read_mnp(&clk->base, &pll->base);
	val = nvkm_rd32(device, GPCPLL_CFG2);
	pll->sdm_din = (val >> GPCPLL_CFG2_SDM_DIN_SHIFT) &
		       MASK(GPCPLL_CFG2_SDM_DIN_WIDTH);
}

static void
gm20b_pllg_write_mnp(struct gm20b_clk *clk, const struct gm20b_pll *pll)
{
	struct nvkm_device *device = clk->base.base.subdev.device;

	nvkm_mask(device, GPCPLL_CFG2, GPCPLL_CFG2_SDM_DIN_MASK,
		  pll->sdm_din << GPCPLL_CFG2_SDM_DIN_SHIFT);
	gk20a_pllg_write_mnp(&clk->base, &pll->base);
}

/*
 * Determine DFS_COEFF for the requested voltage. Always select external
 * calibration override equal to the voltage, and set maximum detection
 * limit "0" (to make sure that PLL output remains under F/V curve when
 * voltage increases).
 */
static void
gm20b_dvfs_calc_det_coeff(struct gm20b_clk *clk, s32 uv,
			  struct gm20b_clk_dvfs *dvfs)
{
	struct nvkm_subdev *subdev = &clk->base.base.subdev;
	const struct gm20b_clk_dvfs_params *p = clk->dvfs_params;
	u32 coeff;
	/* Work with mv as uv would likely trigger an overflow */
	s32 mv = DIV_ROUND_CLOSEST(uv, 1000);

	/* coeff = slope * voltage + offset */
	coeff = DIV_ROUND_CLOSEST(mv * p->coeff_slope, 1000) + p->coeff_offs;
	coeff = DIV_ROUND_CLOSEST(coeff, 1000);
	dvfs->dfs_coeff = min_t(u32, coeff, MASK(GPCPLL_DVFS0_DFS_COEFF_WIDTH));

	dvfs->dfs_ext_cal = DIV_ROUND_CLOSEST(uv - clk->uvdet_offs,
					     clk->uvdet_slope);
	/* should never happen */
	if (abs(dvfs->dfs_ext_cal) >= BIT(DFS_DET_RANGE))
		nvkm_error(subdev, "dfs_ext_cal overflow!\n");

	dvfs->dfs_det_max = 0;

	nvkm_debug(subdev, "%s uv: %d coeff: %x, ext_cal: %d, det_max: %d\n",
		   __func__, uv, dvfs->dfs_coeff, dvfs->dfs_ext_cal,
		   dvfs->dfs_det_max);
}

/*
 * Solve equation for integer and fractional part of the effective NDIV:
 *
 * n_eff = n_int + 1/2 + (SDM_DIN / 2^(SDM_DIN_RANGE + 1)) +
 *         (DVFS_COEFF * DVFS_DET_DELTA) / 2^DFS_DET_RANGE
 *
 * The SDM_DIN LSB is finally shifted out, since it is not accessible by sw.
 */
static void
gm20b_dvfs_calc_ndiv(struct gm20b_clk *clk, u32 n_eff, u32 *n_int, u32 *sdm_din)
{
	struct nvkm_subdev *subdev = &clk->base.base.subdev;
	const struct gk20a_clk_pllg_params *p = clk->base.params;
	u32 n;
	s32 det_delta;
	u32 rem, rem_range;

	/* calculate current ext_cal and subtract previous one */
	det_delta = DIV_ROUND_CLOSEST(((s32)clk->uv) - clk->uvdet_offs,
				      clk->uvdet_slope);
	det_delta -= clk->dvfs.dfs_ext_cal;
	det_delta = min(det_delta, clk->dvfs.dfs_det_max);
	det_delta *= clk->dvfs.dfs_coeff;

	/* integer part of n */
	n = (n_eff << DFS_DET_RANGE) - det_delta;
	/* should never happen! */
	if (n <= 0) {
		nvkm_error(subdev, "ndiv <= 0 - setting to 1...\n");
		n = 1 << DFS_DET_RANGE;
	}
	if (n >> DFS_DET_RANGE > p->max_n) {
		nvkm_error(subdev, "ndiv > max_n - setting to max_n...\n");
		n = p->max_n << DFS_DET_RANGE;
	}
	*n_int = n >> DFS_DET_RANGE;

	/* fractional part of n */
	rem = ((u32)n) & MASK(DFS_DET_RANGE);
	rem_range = SDM_DIN_RANGE + 1 - DFS_DET_RANGE;
	/* subtract 2^SDM_DIN_RANGE to account for the 1/2 of the equation */
	rem = (rem << rem_range) - BIT(SDM_DIN_RANGE);
	/* lose 8 LSB and clip - sdm_din only keeps the most significant byte */
	*sdm_din = (rem >> BITS_PER_BYTE) & MASK(GPCPLL_CFG2_SDM_DIN_WIDTH);

	nvkm_debug(subdev, "%s n_eff: %d, n_int: %d, sdm_din: %d\n", __func__,
		   n_eff, *n_int, *sdm_din);
}

static int
gm20b_pllg_slide(struct gm20b_clk *clk, u32 n)
{
	struct nvkm_subdev *subdev = &clk->base.base.subdev;
	struct nvkm_device *device = subdev->device;
	struct gm20b_pll pll;
	u32 n_int, sdm_din;
	int ret = 0;

	/* calculate the new n_int/sdm_din for this n/uv */
	gm20b_dvfs_calc_ndiv(clk, n, &n_int, &sdm_din);

	/* get old coefficients */
	gm20b_pllg_read_mnp(clk, &pll);
	/* do nothing if NDIV is the same */
	if (n_int == pll.base.n && sdm_din == pll.sdm_din)
		return 0;

	/* pll slowdown mode */
	nvkm_mask(device, GPCPLL_NDIV_SLOWDOWN,
		BIT(GPCPLL_NDIV_SLOWDOWN_SLOWDOWN_USING_PLL_SHIFT),
		BIT(GPCPLL_NDIV_SLOWDOWN_SLOWDOWN_USING_PLL_SHIFT));

	/* new ndiv ready for ramp */
	/* in DVFS mode SDM is uPLL_SHIFT),
		BIT(GPCPLL_NDIV_SLOWDOWN_SLOWDOWN_US?HO<bT/<V000DvrNDIV_