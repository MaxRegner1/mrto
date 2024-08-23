/*
* Copyright (c) 2016 MediaTek Inc.
* Author: PC Chen <pc.chen@mediatek.com>
*         Tiffany Lin <tiffany.lin@mediatek.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <soc/mediatek/smi.h>
#include <linux/delay.h>
#include <linux/semaphore.h>

#include "mtk_vcodec_drv.h"
#include "mtk_vcodec_enc.h"
#include "mtk_vcodec_intr.h"
#include "mtk_vcodec_util.h"
#include "mtk_vcodec_enc_pm.h"
#include "venc_drv_if.h"

#define MTK_VENC_MIN_W  160U
#define MTK_VENC_MIN_H  128U
#define MTK_VENC_MAX_W  1920U
#define MTK_VENC_MAX_H  1088U
#define DFT_CFG_WIDTH   MTK_VENC_MIN_W
#define DFT_CFG_HEIGHT  MTK_VENC_MIN_H

static void mtk_venc_worker(struct work_struct *work);
static struct mtk_video_fmt
	mtk_video_formats[MTK_MAX_ENC_CODECS_SUPPORT] = { {0} };
static struct mtk_codec_framesizes
	mtk_venc_framesizes[MTK_MAX_ENC_CODECS_SUPPORT] = { {0} };
static unsigned int default_out_fmt_idx;
static unsigned int default_cap_fmt_idx;
#ifdef CONFIG_VB2_MEDIATEK_DMA_CONTIG
static struct vb2_mem_ops venc_ion_dma_contig_memops;
#endif

inline unsigned int log2_enc(__u32 value)
{
	unsigned int x = 0;

	while (value > 1) {
		value >>= 1;
		x++;
	}
	return x;
}

static void get_supported_format(struct mtk_vcodec_ctx *ctx)
{
	unsigned int i;

	if (mtk_video_formats[0].fourcc == 0) {
		if (venc_if_get_param(ctx,
			VENC_GET_PARAM_CAPABILITY_SUPPORTED_FORMATS,
			&mtk_video_formats) != 0) {
			mtk_v4l2_err("Error!! Cannot get supported format");
			return;
		}
		for (i = 0; i < MTK_MAX_ENC_CODECS_SUPPORT; i++) {
			if (mtk_video_formats[i].fourcc != 0 &&
			    mtk_video_formats[i].type == MTK_FMT_FRAME) {
				default_out_fmt_idx = i;
				break;
			}
		}
		for (i = 0; i < MTK_MAX_ENC_CODECS_SUPPORT; i++) {
			if (mtk_video_formats[i].fourcc != 0 &&
			    mtk_video_formats[i].type == MTK_FMT_ENC) {
				default_cap_fmt_idx = i;
				break;
			}
		}
	}
}

static void get_supported_framesizes(struct mtk_vcodec_ctx *ctx)
{
	unsigned int i;

	if (mtk_venc_framesizes[0].fourcc == 0) {
		if (venc_if_get_param(ctx, VENC_GET_PARAM_CAPABILITY_FRAME_SIZES,
				      &mtk_venc_framesizes) != 0) {
			mtk_v4l2_err("[%d] Error!! Cannot get frame size",
				ctx->id);
			return;
		}

		for (i = 0; i < MTK_MAX_ENC_CODECS_SUPPORT; i++) {
			if (mtk_venc_framesizes[i].fourcc != 0) {
				mtk_v4l2_debug(1,
				"venc_fs[%d] fourcc %d s %d %d %d %d %d %d\n",
				i, mtk_venc_framesizes[i].fourcc,
				mtk_venc_framesizes[i].stepwise.min_width,
				mtk_venc_framesizes[i].stepwise.max_width,
				mtk_venc_framesizes[i].stepwise.step_width,
				mtk_venc_framesizes[i].stepwise.min_height,
				mtk_venc_framesizes[i].stepwise.max_height,
				mtk_venc_framesizes[i].stepwise.step_height);
			}
		}
	}
}

static void get_free_buffers(struct mtk_vcodec_ctx *ctx,
				struct venc_done_result *pResult)
{
	venc_if_get_param(ctx,
		VENC_GET_PARAM_FREE_BUFFERS,
		pResult);
}

void mtk_enc_put_buf(struct mtk_vcodec_ctx *ctx)
{
	struct venc_done_result rResult;
	struct venc_frm_buf *pfrm;
	struct mtk_vcodec_mem *pbs;
	struct mtk_video_enc_buf *bs_info, *frm_info;
	struct vb2_v4l2_buffer *dst_vb2_v4l2, *src_vb2_v4l2;
	struct vb2_buffer *dst_buf;

	mutex_lock(&ctx->buf_lock);
	do {
		dst_vb2_v4l2 = NULL;
		src_vb2_v4l2 = NULL;
		pfrm = NULL;
		pbs = NULL;

		memset(&rResult, 0, sizeof(rResult));
		get_free_buffers(ctx, &rResult);

		if (rResult.bs_va != 0 && virt_addr_valid(rResult.bs_va)) {
			pbs = (struct mtk_vcodec_mem *)rResult.bs_va;
			bs_info = container_of(pbs,
				struct mtk_video_enc_buf, bs_buf);
			dst_vb2_v4l2 = &bs_info->vb;
		}

		if (rResult.frm_va != 0 && virt_addr_valid(rResult.frm_va)) {
			pfrm = (struct venc_frm_buf *)rResult.frm_va;
			frm_info = container_of(pfrm,
				struct mtk_video_enc_buf, frm_buf);
			src_vb2_v4l2 = &frm_info->vb;
		}

		if (src_vb2_v4l2 != NULL && dst_vb2_v4l2 != NULL) {
			if (rResult.is_key_frm)
				dst_vb2_v4l2->flags |= V4L2_BUF_FLAG_KEYFRAME;

			dst_vb2_v4l2->vb2_buf.timestamp =
				src_vb2_v4l2->vb2_buf.timestamp;
			dst_vb2_v4l2->timecode = src_vb2_v4l2->timecode;
			dst_vb2_v4l2->flags |= src_vb2_v4l2->flags;
			dst_vb2_v4l2->sequence = src_vb2_v4l2->sequence;
			dst_buf = &dst_vb2_v4l2->vb2_buf;
			dst_buf->planes[0].bytesused = rResult.bs_size;
			v4l2_m2m_buf_done(src_vb2_v4l2, VB2_BUF_STATE_DONE);
			v4l2_m2m_buf_done(dst_vb2_v4l2, VB2_BUF_STATE_DONE);

			mtk_v4l2_debug(1, "venc_if_encode bs size=%d",
				rResult.bs_size);
		} else if (src_vb2_v4l2 == NULL && dst_vb2_v4l2 != NULL) {
			dst_buf = &dst_vb2_v4l2->vb2_buf;
			dst_buf->planes[0].bytesused = rResult.bs_size;
			v4l2_m2m_buf_done(dst_vb2_v4l2,
					VB2_BUF_STATE_DONE);
			mtk_v4l2_debug(0, "[Warning] bs size=%d, frm NULL!!",
				rResult.bs_size);
		} else {
			if (src_vb2_v4l2 == NULL)
				mtk_v4l2_debug(1, "NULL enc src buffer\n");

			if (dst_vb2_v4l2 == NULL)
				mtk_v4l2_debug(1, "NULL enc dst buffer\n");
		}
	} while (rResult.bs_va != 0 || rResult.frm_va != 0);
	mutex_unlock(&ctx->buf_lock);
}

static struct mtk_video_fmt *mtk_venc_find_format(struct v4l2_format *f,
						  unsigned int t)
{
	struct mtk_video_fmt *fmt;
	unsigned int k;

	mtk_v4l2_debug(3, "fourcc %d", f->fmt.pix_mp.pixelformat);
	for (k = 0; k < MTK_MAX_ENC_CODECS_SUPPORT &&
	     mtk_video_formats[k].fourcc != 0; k++) {
		fmt = &mtk_video_formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat && fmt->type == t)
			return fmt;
	}

	return NULL;
}

static int vidioc_venc_check_supported_profile_level(__u32 fourcc,
	unsigned int pl, bool is_profile)
{
	struct v4l2_format f;
	int i = 0;

	f.fmt.pix.pixelformat = fourcc;
	if (mtk_venc_find_format(&f, MTK_FMT_ENC) == NULL)
		return false;

	for (i = 0; i < MTK_MAX_ENC_CODECS_SUPPORT; i++) {
		if (mtk_venc_framesizes[i].fourcc == fourcc) {
			if (is_profile) {
				if (mtk_venc_framesizes[i].profile & (1 << pl))
					return true;
				else
					return false;
			} else {
				if (mtk_venc_framesizes[i].level >= pl)
					return true;
				else
					return false;
			}
		}
	}
	return false;
}

static int vidioc_venc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mtk_vcodec_ctx *ctx = ctrl_to_ctx(ctrl);
	struct mtk_enc_params *p = &ctx->enc_params;
	int ret = 0;

	mtk_v4l2_debug(4, "[%d] id %d val %d array[0] %d array[1] %d",
				   ctx->id, ctrl->id, ctrl->val,
				   ctrl->p_new.p_u32[0], ctrl->p_new.p_u32[1]);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_BITRATE val = %d",
			       ctrl->val);
		p->bitrate = ctrl->val;
		ctx->param_change |= MTK_ENCODE_PARAM_BITRATE;
		break;
	case V4L2_CID_MPEG_MTK_SEC_ENCODE:
		p->svp_mode = ctrl->val;
		ctx->param_change |= MTK_ENCODE_PARAM_SEC_ENCODE;
		mtk_v4l2_debug(0, "[%d] V4L2_CID_MPEG_MTK_SEC_ENCODE id %d val %d array[0] %d array[1] %d",
			ctx->id, ctrl->id, ctrl->val,
		ctrl->p_new.p_u32[0], ctrl->p_new.p_u32[1]);
		break;
	case V4L2_CID_MPEG_VIDEO_B_FRAMES:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_B_FRAMES val = %d",
			       ctrl->val);
		p->num_b_frame = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE val = %d",
			       ctrl->val);
		p->rc_frame = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_H264_MAX_QP val = %d",
			       ctrl->val);
		p->h264_max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_HEADER_MODE val = %d",
			       ctrl->val);
		p->seq_hdr_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE val = %d",
			       ctrl->val);
		p->rc_mb = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_H264_PROFILE val = %d",
			       ctrl->val);
		if (!vidioc_venc_check_supported_profile_level(
				V4L2_PIX_FMT_H264, ctrl->val, 1))
			return -EINVAL;
		p->profile = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_HEVC_PROFILE val = %d",
			       ctrl->val);
		if (!vidioc_venc_check_supported_profile_level(
				V4L2_PIX_FMT_H265, ctrl->val, 1))
			return -EINVAL;
		p->profile = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE val = %d",
			       ctrl->val);
		if (!vidioc_venc_check_supported_profile_level(
			    r

mtx->param_ch= ctrl->v V4L2ase V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		mtk_v4l2_dW6LE vay %d",
			       ctrl->val);
		p->seq_hdr_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MB_R2