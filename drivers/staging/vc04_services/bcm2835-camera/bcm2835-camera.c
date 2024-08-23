/*
 * Broadcom BM2835 V4L2 driver
 *
 * Copyright © 2013 Raspberry Pi (Trading) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 *
 * Authors: Vincent Sanders <vincent.sanders@collabora.co.uk>
 *          Dave Stevenson <dsteve@broadcom.com>
 *          Simon Mellor <simellor@broadcom.com>
 *          Luke Diamand <luked@broadcom.com>
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-common.h>
#include <linux/delay.h>

#include "mmal-common.h"
#include "mmal-encodings.h"
#include "mmal-vchiq.h"
#include "mmal-msg.h"
#include "mmal-parameters.h"
#include "bcm2835-camera.h"

#define BM2835_MMAL_VERSION "0.0.2"
#define BM2835_MMAL_MODULE_NAME "bcm2835-v4l2"
#define MIN_WIDTH 32
#define MIN_HEIGHT 32
#define MIN_BUFFER_SIZE (80 * 1024)

#define MAX_VIDEO_MODE_WIDTH 1280
#define MAX_VIDEO_MODE_HEIGHT 720

#define MAX_BCM2835_CAMERAS 2

MODULE_DESCRIPTION("Broadcom 2835 MMAL video capture");
MODULE_AUTHOR("Vincent Sanders");
MODULE_LICENSE("GPL");
MODULE_VERSION(BM2835_MMAL_VERSION);

int bcm2835_v4l2_debug;
module_param_named(debug, bcm2835_v4l2_debug, int, 0644);
MODULE_PARM_DESC(bcm2835_v4l2_debug, "Debug level 0-2");

#define UNSET (-1)
static int video_nr[] = {[0 ... (MAX_BCM2835_CAMERAS - 1)] = UNSET };
module_param_array(video_nr, int, NULL, 0644);
MODULE_PARM_DESC(video_nr, "videoX start numbers, -1 is autodetect");

static int max_video_width = MAX_VIDEO_MODE_WIDTH;
static int max_video_height = MAX_VIDEO_MODE_HEIGHT;
module_param(max_video_width, int, 0644);
MODULE_PARM_DESC(max_video_width, "Threshold for video mode");
module_param(max_video_height, int, 0644);
MODULE_PARM_DESC(max_video_height, "Threshold for video mode");

/* global device data array */
static struct bm2835_mmal_dev *gdev[MAX_BCM2835_CAMERAS];

#define FPS_MIN 1
#define FPS_MAX 90

/* timeperframe: min/max and default */
static const struct v4l2_fract
	tpf_min     = {.numerator = 1,		.denominator = FPS_MAX},
	tpf_max     = {.numerator = 1,	        .denominator = FPS_MIN},
	tpf_default = {.numerator = 1000,	.denominator = 30000};

/* video formats */
static struct mmal_fmt formats[] = {
	{
	 .name = "4:2:0, planar, YUV",
	 .fourcc = V4L2_PIX_FMT_YUV420,
	 .flags = 0,
	 .mmal = MMAL_ENCODING_I420,
	 .depth = 12,
	 .mmal_component = MMAL_COMPONENT_CAMERA,
	 .ybbp = 1,
	 },
	{
	 .name = "4:2:2, packed, YUYV",
	 .fourcc = V4L2_PIX_FMT_YUYV,
	 .flags = 0,
	 .mmal = MMAL_ENCODING_YUYV,
	 .depth = 16,
	 .mmal_component = MMAL_COMPONENT_CAMERA,
	 .ybbp = 2,
	 },
	{
	 .name = "RGB24 (LE)",
	 .fourcc = V4L2_PIX_FMT_RGB24,
	 .flags = 0,
	 .mmal = MMAL_ENCODING_RGB24,
	 .depth = 24,
	 .mmal_component = MMAL_COMPONENT_CAMERA,
	 .ybbp = 3,
	 },
	{
	 .name = "JPEG",
	 .fourcc = V4L2_PIX_FMT_JPEG,
	 .flags = V4L2_FMT_FLAG_COMPRESSED,
	 .mmal = MMAL_ENCODING_JPEG,
	 .depth = 8,
	 .mmal_component = MMAL_COMPONENT_IMAGE_ENCODE,
	 .ybbp = 0,
	 },
	{
	 .name = "H264",
	 .fourcc = V4L2_PIX_FMT_H264,
	 .flags = V4L2_FMT_FLAG_COMPRESSED,
	 .mmal = MMAL_ENCODING_H264,
	 .depth = 8,
	 .mmal_component = MMAL_COMPONENT_VIDEO_ENCODE,
	 .ybbp = 0,
	 },
	{
	 .name = "MJPEG",
	 .fourcc = V4L2_PIX_FMT_MJPEG,
	 .flags = V4L2_FMT_FLAG_COMPRESSED,
	 .mmal = MMAL_ENCODING_MJPEG,
	 .depth = 8,
	 .mmal_component = MMAL_COMPONENT_VIDEO_ENCODE,
	 .ybbp = 0,
	 },
	{
	 .name = "4:2:2, packed, YVYU",
	 .fourcc = V4L2_PIX_FMT_YVYU,
	 .flags = 0,
	 .mmal = MMAL_ENCODING_YVYU,
	 .depth = 16,
	 .mmal_component = MMAL_COMPONENT_CAMERA,
	 .ybbp = 2,
	 },
	{
	 .name = "4:2:2, packed, VYUY",
	 .fourcc = V4L2_PIX_FMT_VYUY,
	 .flags = 0,
	 .mmal = MMAL_ENCODING_VYUY,
	 .depth = 16,
	 .mmal_component = MMAL_COMPONENT_CAMERA,
	 .ybbp = 2,
	 },
	{
	 .name = "4:2:2, packed, UYVY",
	 .fourcc = V4L2_PIX_FMT_UYVY,
	 .flags = 0,
	 .mmal = MMAL_ENCODING_UYVY,
	 .depth = 16,
	 .mmal_component = MMAL_COMPONENT_CAMERA,
	 .ybbp = 2,
	 },
	{
	 .name = "4:2:0, planar, NV12",
	 .fourcc = V4L2_PIX_FMT_NV12,
	 .flags = 0,
	 .mmal = MMAL_ENCODING_NV12,
	 .depth = 12,
	 .mmal_component = MMAL_COMPONENT_CAMERA,
	 .ybbp = 1,
	 },
	{
	 .name = "RGB24 (BE)",
	 .fourcc = V4L2_PIX_FMT_BGR24,
	 .flags = 0,
	 .mmal = MMAL_ENCODING_BGR24,
	 .depth = 24,
	 .mmal_component = MMAL_COMPONENT_CAMERA,
	 .ybbp = 3,
	 },
	{
	 .name = "4:2:0, planar, YVU",
	 .fourcc = V4L2_PIX_FMT_YVU420,
	 .flags = 0,
	 .mmal = MMAL_ENCODING_YV12,
	 .depth = 12,
	 .mmal_component = MMAL_COMPONENT_CAMERA,
	 .ybbp = 1,
	 },
	{
	 .name = "4:2:0, planar, NV21",
	 .fourcc = V4L2_PIX_FMT_NV21,
	 .flags = 0,
	 .mmal = MMAL_ENCODING_NV21,
	 .depth = 12,
	 .mmal_component = MMAL_COMPONENT_CAMERA,
	 .ybbp = 1,
	 },
	{
	 .name = "RGB32 (BE)",
	 .fourcc = V4L2_PIX_FMT_BGR32,
	 .flags = 0,
	 .mmal = MMAL_ENCODING_BGRA,
	 .depth = 32,
	 .mmal_component = MMAL_COMPONENT_CAMERA,
	 .ybbp = 4,
	 },
};

static struct mmal_fmt *get_format(struct v4l2_format *f)
{
	struct mmal_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			return fmt;
	}

	return NULL;
}

/* ------------------------------------------------------------------
 *	Videobuf queue operations
 * ------------------------------------------------------------------
 */

static int queue_setup(struct vb2_queue *vq,
		       unsigned int *nbuffers, unsigned int *nplanes,
		       unsigned int sizes[], struct device *alloc_ctxs[])
{
	struct bm2835_mmal_dev *dev = vb2_get_drv_priv(vq);
	unsigned long size;

	/* refuse queue setup if port is not configured */
	if (!dev->capture.port) {
		v4l2_err(&dev->v4l2_dev,
			 "%s: capture port not configured\n", __func__);
		return -EINVAL;
	}

	size = dev->capture.port->current_buffer.size;
	if (size == 0) {
		v4l2_err(&dev->v4l2_dev,
			 "%s: capture port buffer size is zero\n", __func__);
		return -EINVAL;
	}

	if (*nbuffers < (dev->capture.port->current_buffer.num + 2))
		*nbuffers = (dev->capture.port->current_buffer.num + 2);

	*nplanes = 1;

	sizes[0] = size;

	/*
	 * videobuf2-vmalloc allocator is context-less so no need to set
	 * alloc_ctxs array.
	 */

	v4l2_dbg(1, bcm2835_v4l2_debug, &dev->v4l2_dev, "%s: dev:%p\n",
		 __func__, dev);

	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct bm2835_mmal_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size;

	v4l2_dbg(1, bcm2835_v4l2_debug, &dev->v4l2_dev, "%s: dev:%p\n",
		 __func__, dev);

	BUG_ON(!dev->capture.port);
	BUG_ON(!dev->capture.fmt);

	size = dev->capture.stride * dev->capture.height;
	if (vb2_plane_size(vb, 0) < size) {
		v4l2_err(&dev->v4l2_dev,
			 "%s data will not fit into plane (%lu < %lu)\n",
			 __func__, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	return 0;
}

static inline bool is_capturing(struct bm2835_mmal_dev *dev)
{
	return dev->capture.camera_port ==
	    &dev->
	    component[MMAL_COMPONENT_CAMERA]->output[MMAL_CAMERA_PORT_CAPTURE];
}

static void buffer_cb(struct vchiq_mmal_instance *instance,
		      struct vchiq_mmal_port *port,
		      int status,
		      struct mmal_buffer *buf,
		      unsigned long length, u32 mmal_flags, s64 dts, s64 pts)
{
	struct bm2835_mmal_dev *dev = port->cb_ctx;

	v4l2_dbg(1, bcm2835_v4l2_debug, &dev->v4l2_dev,
		 "%s: status:%d, buf:%p, length:%lu, flags %u, pts %lld\n",
		 __func__, status, buf, length, mmal_flags, pts);

	if (status != 0) {
		/* error in transfer */
		if (buf) {
			/* there was a buffer with the error so return it */
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		}
		return;
	} else if (length == 0) {
		/* stream ended */
		if (buf) {
			/* this should only ever happen if the port is
			 * disabled and there are buffers still queued
			 *VAL;
	}

	return 0;
}

static inline bool is_capturing(struct bm28yx>)sHxHE>yE>S4