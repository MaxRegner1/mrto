/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _PRIMARY_DISPLAY_H_
#define _PRIMARY_DISPLAY_H_

#include "ddp_hal.h"
#include "ddp_manager.h"
#include <linux/types.h>
#include "disp_session.h"
#include "disp_lcm.h"
#include "disp_helper.h"
#ifdef MTK_FB_MMDVFS_SUPPORT
#include <linux/pm_qos.h>
#endif


#ifdef MTK_FB_MMDVFS_SUPPORT
extern struct pm_qos_request primary_display_qos_request;
extern struct pm_qos_request primary_display_emi_opp_request;
extern struct pm_qos_request primary_display_mm_freq_request;
#endif

enum DISP_PRIMARY_PATH_MODE {
	DIRECT_LINK_MODE,
	DECOUPLE_MODE,
	SINGLE_LAYER_MODE,
	DEBUG_RDMA1_DSI0_MODE
};
#define UINT8 unsigned char
#define UINT32 unsigned int

#ifndef TRUE
	#define TRUE	(1)
#endif

#ifndef FALSE
	#define FALSE	(0)
#endif

#define ALIGN_TO(x, n)	(((x) + ((n) - 1)) & ~((n) - 1))

#define ASSERT_LAYER    (DDP_OVL_LAYER_MUN-1)
extern unsigned int FB_LAYER;	/* default LCD layer */
#define DISP_DEFAULT_UI_LAYER_ID (DDP_OVL_LAYER_MUN-1)
#define DISP_CHANGED_UI_LAYER_ID (DDP_OVL_LAYER_MUN-2)

#define pgc	_get_context()

extern unsigned int ap_fps_changed;
extern unsigned int arr_fps_backup;
extern unsigned int arr_fps_enable;
extern unsigned int round_corner_offset_enable;

extern bool g_force_cfg;
extern unsigned int g_force_cfg_id;

struct DISP_LAYER_INFO {
	unsigned int id;
	unsigned int curr_en;
	unsigned int next_en;
	unsigned int hw_en;
	int curr_idx;
	int next_idx;
	int hw_idx;
	int curr_identity;
	int next_identity;
	int hw_identity;
	int curr_conn_type;
	int next_conn_type;
	int hw_conn_type;
};

enum DISP_STATUS {
	DISP_STATUS_OK = 0,
	DISP_STATUS_NOT_IMPLEMENTED,
	DISP_STATUS_ALREADY_SET,
	DISP_STATUS_ERROR,
};

#if 0
enum DISP_STATE {
	DISP_STATE_IDLE = 0,
	DISP_STATE_BUSY,
};

enum DISP_OP_STATE {
	DISP_OP_PRE = 0,
	DISP_OP_NORMAL,
	DISP_OP_POST,
};
#endif

enum DISP_POWER_STATE {
	DISP_ALIVE = 0xf0,
	DISP_SLEPT,
	DISP_BLANK
};

enum DISP_FRM_SEQ_STATE {
	FRM_CONFIG = 0,
	FRM_TRIGGER,
	FRM_START,
	FRM_END
};

#if 0
enum DISPLAY_HAL_IOCTL {
	DISPLAY_HAL_IOCTL_SET_CMDQ = 0xff00,
	DISPLAY_HAL_IOCTL_ENABLE_CMDQ,
	DISPLAY_HAL_IOCTL_DUMP,
	DISPLAY_HAL_IOCTL_PATTERN,
};
#endif

struct primary_disp_input_config {
	unsigned int layer;
	unsigned int layer_en;
	unsigned int buffer_source;
	unsigned int fmt;
	unsigned long addr;
	unsigned long addr_sub_u;
	unsigned long addr_sub_v;
	unsigned long vaddr;
	unsigned int src_x;
	unsigned int src_y;
	unsigned int src_w;
	unsigned int src_h;
	unsigned int src_pitch;
	unsigned int dst_x;
	unsigned int dst_y;
	unsigned int dst_w;
	unsigned int dst_h;	/* clip region */
	unsigned int keyEn;
	unsigned int key;
	unsigned int aen;
	unsigned char alpha;

	unsigned int sur_aen;
	unsigned int src_alpha;
	unsigned int dst_alpha;
	unsigned int isTdshp;
	unsigned int isDirty;

	unsigned int buff_idx;
	unsigned int identity;
	unsigned int connected_type;
	enum DISP_BUFFER_TYPE security;
	unsigned int dirty;
	unsigned int yuv_range;
};

struct disp_mem_output_config {
	enum UNIFIED_COLOR_FMT fmt;
	unsigned long addr;
	unsigned long addr_sub_u;
	unsigned long addr_sub_v;
	unsigned long vaddr;
	unsigned int x;
	unsigned int y;
	unsigned int w;
	unsigned int h;
	unsigned int pitch;
	unsigned int pitchUV;

	unsigned int buff_idx;
	unsigned int interface_idx;
	enum DISP_BUFFER_TYPE security;
	unsigned int dirty;
	int mode;

	/* night light setting */
	struct disp_ccorr_config m_ccorr_config;
};

#define DISP_INTERNAL_BUFFER_COUNT 3

struct disp_internal_buffer_info {
	struct list_head list;
	struct ion_handle *handle;
	struct sync_fence *pfence;
	void *va;
	uint32_t fence_id;
	uint32_t mva;
	uint32_t size;
	uint32_t output_fence_id;
	uint32_t interface_fence_id;
	unsigned long long timestamp;
};

struct disp_frm_seq_info {
	unsigned int mva;
	unsigned int max_offset;
	unsigned int seq;
	enum DISP_FRM_SEQ_STATE state;
};

struct OPT_BACKUP {
	enum DISP_HELhlwkpunsx}xhYxhYwnsigned int mva;M