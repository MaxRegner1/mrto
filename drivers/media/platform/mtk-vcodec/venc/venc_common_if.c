/*
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Longfei Wang <longfei.wang@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "../mtk_vcodec_drv.h"
#include "../mtk_vcodec_util.h"
#include "../mtk_vcodec_enc.h"
#include "../venc_drv_base.h"
#include "../venc_ipi_msg.h"
#include "../venc_vcu_if.h"
#include "mtk_vcodec_enc_pm.h"
#include "mtk_vcodec_intr.h"
#include "mtk_vcu.h"



static unsigned int venc_h265_get_profile(struct venc_inst *inst,
	unsigned int profile)
{
	switch (profile) {
	case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN:
		return 2;
	case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10:
		return 4;
	case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE:
		return 8;
	default:
		mtk_vcodec_debug(inst, "unsupported profile %d", profile);
		return 0;
	}
}

static unsigned int venc_h265_get_level(struct venc_inst *inst,
	unsigned int level, unsigned int tier)
{
	switch (level) {
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 0 : 1;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_2:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 2 : 3;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 4 : 5;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_3:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 6 : 7;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 8 : 9;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_4:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 10 : 11;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 12 : 13;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_5:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 14 : 15;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 16 : 17;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 18 : 19;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_6:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 20 : 21;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_6_1:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 22 : 23;
	case V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2:
		return (tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ? 24 : 25;
	default:
		mtk_vcodec_debug(inst, "unsupported level %d", level);
		return 26;
	}
}

static unsigned int venc_mpeg4_get_profile(struct venc_inst *inst,
	unsigned int profile)
{
	switch (profile) {
	case V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE:
		return 0;
	case V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_SIMPLE:
		return 1;
	case V4L2_MPEG_VIDEO_MPEG4_PROFILE_CORE:
		return 2;
	case V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE_SCALABLE:
		return 3;
	case V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_CODING_EFFICIENCY:
		return 4;
	default:
		mtk_vcodec_debug(inst, "unsupported mpeg4 profile %d", profile);
		return 100;
	}
}

static unsigned int venc_mpeg4_get_level(struct venc_inst *inst,
	unsigned int level)
{
	switch (level) {
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_0:
		return 0;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_0B:
		return 1;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_1:
		return 2;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_2:
		return 3;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_3:
		return 4;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_3B:
		return 5;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_4:
		return 6;
	case V4L2_MPEG_VIDEO_MPEG4_LEVEL_5:
		return 7;
	default:
		mtk_vcodec_debug(inst, "unsupported mpeg4 level %d", level);
		return 4;
	}
}

static int venc_encode_header(struct venc_inst *inst,
	struct mtk_vcodec_mem *bs_buf,
	unsigned int *bs_size)
{
	int ret = 0;

	mtk_vcodec_debug_enter(inst);
	if (bs_buf == NULL)
		inst->vsi->venc.venc_bs_va = 0;
	else
		inst->vsi->venc.venc_bs_va = (u64)(uintptr_t)bs_buf;

	inst->vsi->venc.venc_fb_va = 0;

	mtk_vcodec_debug(inst, "vsi venc_bs_va 0x%llx",
			 inst->vsi->venc.venc_bs_va);

	ret = vcu_enc_encode(&inst->vcu_inst, VENC_BS_MODE_SEQ_HDR, NULL,
						 bs_buf, bs_size);

	return ret;
}

static int venc_encode_frame(struct venc_inst *inst,
	struct venc_frm_buf *frm_buf,
	struct mtk_vcodec_mem *bs_buf,
	unsigned int *bs_size)
{
	int ret = 0;
	unsigned int fm_fourcc = inst->ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc;
	unsigned int bs_fourcc = inst->ctx->q_data[MTK_Q_DATA_