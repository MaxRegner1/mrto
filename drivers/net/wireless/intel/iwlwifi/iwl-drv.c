/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2007 - 2014 Intel Corporation. All rights reserved.
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
 * Copyright(c) 2005 - 2014 Intel Corporation. All rights reserved.
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
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/vmalloc.h>

#include "iwl-drv.h"
#include "iwl-csr.h"
#include "iwl-debug.h"
#include "iwl-trans.h"
#include "iwl-op-mode.h"
#include "iwl-agn-hw.h"
#include "fw/img.h"
#include "iwl-config.h"
#include "iwl-modparams.h"

/******************************************************************************
 *
 * module boiler plate
 *
 ******************************************************************************/

#define DRV_DESCRIPTION	"Intel(R) Wireless WiFi driver for Linux"
MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_AUTHOR(DRV_COPYRIGHT " " DRV_AUTHOR);
MODULE_LICENSE("GPL");

#ifdef CONFIG_IWLWIFI_DEBUGFS
static struct dentry *iwl_dbgfs_root;
#endif

/**
 * struct iwl_drv - drv common data
 * @list: list of drv structures using this opmode
 * @fw: the iwl_fw structure
 * @op_mode: the running op_mode
 * @trans: transport layer
 * @dev: for debug prints only
 * @fw_index: firmware revision to try loading
 * @firmware_name: composite filename of ucode file to load
 * @request_firmware_complete: the firmware has been obtained from user space
 */
struct iwl_drv {
	struct list_head list;
	struct iwl_fw fw;

	struct iwl_op_mode *op_mode;
	struct iwl_trans *trans;
	struct device *dev;

	int fw_index;                   /* firmware we're trying to load */
	char firmware_name[64];         /* name of firmware file to load */

	struct completion request_firmware_complete;

#ifdef CONFIG_IWLWIFI_DEBUGFS
	struct dentry *dbgfs_drv;
	struct dentry *dbgfs_trans;
	struct dentry *dbgfs_op_mode;
#endif
};

enum {
	DVM_OP_MODE,
	MVM_OP_MODE,
};

/* Protects the table contents, i.e. the ops pointer & drv list */
static struct mutex iwlwifi_opmode_table_mtx;
static struct iwlwifi_opmode_table {
	const char *name;			/* name: iwldvm, iwlmvm, etc */
	const struct iwl_op_mode_ops *ops;	/* pointer to op_mode ops */
	struct list_head drv;		/* list of devices using this op_mode */
} iwlwifi_opmode_table[] = {		/* ops set when driver is initialized */
	[DVM_OP_MODE] = { .name = "iwldvm", .ops = NULL },
	[MVM_OP_MODE] = { .name = "iwlmvm", .ops = NULL },
};

#define IWL_DEFAULT_SCAN_CHANNELS 40

/*
 * struct fw_sec: Just for the image parsing process.
 * For the fw storage we are using struct fw_desc.
 */
struct fw_sec {
	const void *data;		/* the sec data */
	size_t size;			/* section size */
	u32 offset;			/* offset of writing in the device */
};

static void iwl_free_fw_desc(struct iwl_drv *drv, struct fw_desc *desc)
{
	vfree(desc->data);
	desc->data = NULL;
	desc->len = 0;
}

static void iwl_free_fw_img(struct iwl_drv *drv, struct fw_img *img)
{
	int i;
	for (i = 0; i < img->num_sec; i++)
		iwl_free_fw_desc(drv, &img->sec[i]);
	kfree(img->sec);
}

static void iwl_dealloc_ucode(struct iwl_drv *drv)
{
	int i;

	kfree(drv->fw.dbg_dest_tlv);
	for (i = 0; i < ARRAY_SIZE(drv->fw.dbg_conf_tlv); i++)
		kfree(drv->fw.dbg_conf_tlv[i]);
	for (i = 0; i < ARRAY_SIZE(drv->fw.dbg_trigger_tlv); i++)
		kfree(drv->fw.dbg_trigger_tlv[i]);
	kfree(drv->fw.dbg_mem_tlv);

	for (i = 0; i < IWL_UCODE_TYPE_MAX; i++)
		iwl_free_fw_img(drv, drv->fw.img + i);
}

static int iwl_alloc_fw_desc(struct iwl_drv *drv, struct fw_desc *desc,
			     struct fw_sec *sec)
{
	void *data;

	desc->data = NULL;

	if (!sec || !sec->size)
		return -EINVAL;

	data = vmalloc(sec->size);
	if (!data)
		return -ENOMEM;

	desc->len = sec->size;
	desc->offset = sec->offset;
	memcpy(data, sec->data, desc->len);
	desc->data = data;

	return 0;
}

static void iwl_req_fw_callback(const struct firmware *ucode_raw,
				void *context);

static int iwl_request_firmware(struct iwl_drv *drv, bool first)
{
	const struct iwl_cfg *cfg = drv->trans->cfg;
	char tag[8];
	const char *fw_pre_name;

	if (drv->trans->cfg->device_family == IWL_DEVICE_FAMILY_9000 &&
	    (CSR_HW_REV_STEP(drv->trans->hw_rev) == SILICON_B_STEP ||
	     CSR_HW_REV_STEP(drv->trans->hw_rev) == SILICON_C_STEP))
		fw_pre_name = cfg->fw_name_pre_b_or_c_step;
	else if (drv->trans->cfg->integrated &&
		 CSR_HW_RFID_STEP(drv->trans->hw_rf_id) == SILICON_B_STEP &&
		 cfg->fw_name_pre_rf_next_step)
		fw_pre_name = cfg->fw_name_pre_rf_next_step;
	else
		fw_pre_name = cfg->fw_name_pre;

	if (first) {
		drv->fw_index = cfg->ucode_api_max;
		sprintf(tag, "%d", drv->fw_index);
	} else {
		drv->fw_index--;
		sprintf(tag, "%d", drv->fw_index);
	}

	if (drv->fw_index < cfg->ucode_api_min) {
		IWL_ERR(drv, "no suitable firmware found!\n");

		if (cfg->ucode_api_min == cfg->ucode_api_max) {
			IWL_ERR(drv, "%s%d is required\n", fw_pre_name,
				cfg->ucode_api_max);
		} else {
			IWL_ERR(drv, "minimum version required: %s%d\n",
				fw_pre_name,
				cfg->ucode_api_min);
			IWL_ERR(drv, "maximum version supported: %s%d\n",
				fw_pre_name,
				cfg->ucode_api_max);
		}

		IWL_ERR(drv,
			"check git://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git\n");
		return -ENOENT;
	}

	snprintf(drv->firmware_name, sizeof(drv->firmware_name), "%s%s.ucode",
		 fw_pre_name, tag);

	IWL_DEBUG_INFO(drv, "attempting to load firmware '%s'\n",
		       drv->firmware_name);

	return request_firmware_nowait(THIS_MODULE, 1, drv->firmware_name,
				       drv->trans->dev,
				       GFP_KERNEL, drv, iwl_req_fw_callback);
}

struct fw_img_parsing {
	struct fw_sec *sec;
	int sec_counter;
};

/*
 * struct fw_sec_parsing: to extract fw section and it's offset from tlv
 */
struct fw_sec_parsing {
	__le32 offset;
	const u8 data[];
} __packed;

/**
 * struct iwl_tlv_calib_data - parse the default calib data from TLV
 *
 * @ucode_type: the uCode to which the following default calib relates.
 * @calib: default calibrations.
 */
struct iwl_tlv_calib_data {
	__le32 ucode_type;
	struct iwl_tlv_calib_ctrl calib;
} __packed;

struct iwl_firmware_pieces {
	struct fw_img_parsing img[IWL_UCODE_TYPE_MAX];

	u32 init_evtlog_ptr, init_evtlog_size, init_errlog_ptr;
	u32 inst_evtlog_ptr, inst_evtlog_size, inst_errlog_ptr;

	/* FW debug data parsed for driver usage */
	struct iwl_fw_dbg_dest_tlv *dbg_dest_tlv;
	struct iwl_fw_dbg_conf_tlv *dbg_conf_tlv[FW_DBG_CONF_MAX];
	size_t dbg_conf_tlv_len[FW_DBG_CONF_MAX];
	struct iwl_fw_dbg_trigger_tlv *dbg_trigger_tlv[FW_DBG_TRIGGER_MAX];
	size_t dbg_trigger_tlv_len[FW_DBG_TRIGGER_MAX];
	struct iwl_fw_dbg_mem_seg_tlv *dbg_mem_tlv;
	size_t n_dbg_mem_tlv;
};

/*
 * These functions are just to extract uCode section data from the pieces
 * structure.
 */
static struct fw_sec *get_sec(struct iwl_firmware_pieces *pieces,
			      enum iwl_ucode_type type,
			      int  sec)
{
	return &pieces->img[type].sec[sec];
}

static void alloc_sec_data(struct iwl_firmware_pieces *pieces,
			   enum iwl_ucode_type type,
			   int sec)
{
	struct fw_img_parsing *img = &pieces->img[type];
	struct fw_sec *sec_memory;
	int size = sec + 1;
	size_t alloc_size = sizeof(*img->sec) * size;

	if (img->sec && img->sec_counter >= size)
		return;

	sec_memory = krealloc(img->sec, alloc_size, GFP_KERNEL);
	if (!sec_memory)
		return;

	img->sec = sec_memory;
	img->sec_counter = size;
}

static void set_sec_data(struct iwl_firmware_pieces *pieces,
			 enum iwl_ucode_type type,
			 int sec,
			 const void *data)
{
	alloc_sec_data(pieces, type, sec);

	pieces->img[type].sec[sec].data = data;
}

static void set_sec_size(struct iwl_firmware_pieces *pieces,
			 enum iwl_ucode_type type,
			 int sec,
			 size_t size)
{
	alloc_sec_data(pieces, type, sec);

	pieces->img[type].sec[sec].size = size;
}

static size_t get_sec_size(struct iwl_firmware_pieces *pieces,
			   enum iwl_ucode_type type,
			   int sec)
{
	return pieces->img[type].sec[sec].size;
}

static void set_sec_offset(struct iwl_firmware_pieces *pieces,
			   enum iwl_ucode_type type,
			   int sec,
			   u32 offset)
{
	alloc_sec_data(pieces, type, sec);

	pieces->img[type].sec[sec].offset = offset;
}

static int iwl_store_cscheme(struct iwl_fw *fw, const u8 *data, const u32 len)
{
	int i, j;
	struct iwl_fw_cscheme_list *l = (struct iwl_fw_cscheme_list *)data;
	struct iwl_fw_cipher_scheme *fwcs;

	if (len < sizeof(*l) ||
	    len < sizeof(l->size) + l->size * sizeof(l->cs[0]))
		return -EINVAL;

	for (i = 0, j = 0; i < IWL_UCODE_MAX_CS && i < l->size; i++) {
		fwcs = &l->cs[j];

		/* we skip schemes with zero cipher suite selector */
		if (!fwcs->cipher)
			continue;

		fw->cs[j++] = *fwcs;
	}

	return 0;
}

static void iwl_store_gscan_capa(struct iwl_fw *fw, const u8 *data,
				 const u32 len)
{
	struct iwl_fw_gscan_capabilities *fw_capa = (void *)data;
	struct iwl_gscan_capabilities *capa = &fw->gscan_capa;

	capa->max_scan_cache_size = le32_to_cpu(fw_capa->max_scan_cache_size);
	capa->max_scan_buckets = le32_to_cpu(fw_capa->max_scan_buckets);
	capa->max_ap_cache_per_scan =
		le32_to_cpu(fw_capa->max_ap_cache_per_scan);
	capa->max_rssi_sample_size = le32_to_cpu(fw_capa->max_rssi_sample_size);
	capa->max_scan_reporting_threshold =
		le32_to_cpu(fw_capa->max_scan_reporting_threshold);
	capa->max_hotlist_aps = le32_to_cpu(fw_capa->max_hotlist_aps);
	capa->max_significant_change_aps =
		le32_to_cpu(fw_capa->max_significant_change_aps);
	capa->max_bssid_history_entries =
		le32_to_cpu(fw_capa->max_bssid_history_entries);
	capa->max_hotlist_ssids = le32_to_cpu(fw_capa->max_hotlist_ssids);
	capa->max_number_epno_networks =
		le32_to_cpu(fw_capa->max_number_epno_networks);
	capa->max_number_epno_networks_by_ssid =
		le32_to_cpu(fw_capa->max_number_epno_networks_by_ssid);
	capa->max_number_of_white_listed_ssid =
		le32_to_cpu(fw_capa->max_number_of_white_listed_ssid);
	capa->max_number_of_black_listed_ssid =
		le32_to_cpu(fw_capa->max_number_of_black_listed_ssid);
}

/*
 * Gets uCode section from tlv.
 */
static int iwl_store_ucode_sec(struct iwl_firmware_pieces *pieces,
			       const void *data, enum iwl_ucode_type type,
			       int size)
{
	struct fw_img_parsing *img;
	struct fw_sec *sec;
	struct fw_sec_parsing *sec_parse;
	size_t alloc_size;

	if (WARN_ON(!pieces || !data || type >= IWL_UCODE_TYPE_MAX))
		return -1;

	sec_parse = (struct fw_sec_parsing *)data;

	img = &pieces->img[type];

	alloc_size = sizeof(*img->sec) * (img->sec_counter + 1);
	sec = krealloc(img->sec, alloc_size, GFP_KERNEL);
	if (!sec)
		return -ENOMEM;
	img->sec = sec;

	sec = &img->sec[img->sec_counter];

	sec->offset = le32_to_cpu(sec_parse->offset);
	sec->data = sec_parse->data;
	sec->size = size - sizeof(sec_parse->offset);

	++img->sec_counter;

	return 0;
}

static int iwl_set_default_calib(struct iwl_drv *drv, const u8 *data)
{
	struct iwl_tlv_calib_data *def_calib =
					(struct iwl_tlv_calib_data *)data;
	u32 ucode_type = le32_to_cpu(def_calib->ucode_type);
	if (ucode_type >= IWL_UCODE_TYPE_MAX) {
		IWL_ERR(drv, "Wrong ucode_type %u for default calibration.\n",
			ucode_type);
		return -EINVAL;
	}
	drv->fw.default_calib[ucode_type].flow_trigger =
		def_calib->calib.flow_trigger;
	drv->fw.default_calib[ucode_type].event_trigger =
		def_calib->calib.event_trigger;

	return 0;
}

static void iwl_set_ucode_api_flags(struct iwl_drv *drv, const u8 *data,
				    struct iwl_ucode_capabilities *capa)
{
	const struct iwl_ucode_api *ucode_api = (void *)data;
	u32 api_index = le32_to_cpu(ucode_api->api_index);
	u32 api_flags = le32_to_cpu(ucode_api->api_flags);
	int i;

	if (api_index >= DIV_ROUND_UP(NUM_IWL_UCODE_TLV_API, 32)) {
		IWL_WARN(drv,
			 "api flags index %d larger than supported by driver\n",
			 api_index);
		return;
	}

	for (i = 0; i < 32; i++) {
		if (api_flags & BIT(i))
			__set_bit(i + 32 * api_index, capa->_api);
	}
}

static void iwl_set_ucode_capabilities(struct iwl_drv *drv, const u8 *data,
				       struct iwl_ucode_capabilities *capa)
{
	const struct iwl_ucode_capa *ucode_capa = (void *)data;
	u32 api_index = le32_to_cpu(ucode_capa->api_index);
	u32 api_flags = le32_to_cpu(ucode_capa->api_capa);
	int i;

	if (api_index >= DIV_ROUND_UP(NUM_IWL_UCODE_TLV_CAPA, 32)) {
		IWL_WARN(drv,
			 "capa flags index %d larger than supported by driver\n",
			 api_index);
		return;
	}

	for (i = 0; i < 32; i++) {
		if (api_flags & BIT(i))
			__set_bit(i + 32 * api_index, capa->_capa);
	}
}

static int iwl_parse_v1_v2_firmware(struct iwl_drv *drv,
				    const struct firmware *ucode_raw,
				    struct iwl_firmware_pieces *pieces)
{
	struct iwl_ucode_header *ucode = (void *)ucode_raw->data;
	u32 api_ver, hdr_size, build;
	char buildstr[25];
	const u8 *src;

	drv->fw.ucode_ver = le32_to_cpu(ucode->ver);
	api_ver = IWL_UCODE_API(drv->fw.ucode_ver);

	switch (api_ver) {
	default:
		hdr_size = 28;
		if (ucode_raw->size < hdr_size) {
			IWL_ERR(drv, "File size too small!\n");
			return -EINVAL;
		}
		build = le32_to_cpu(ucode->u.v2.build);
		set_sec_size(pieces, IWL_UCODE_REGULAR, IWL_UCODE_SECTION_INST,
			     le32_to_cpu(ucode->u.v2.inst_size));
		set_sec_size(pieces, IWL_UCODE_REGULAR, IWL_UCODE_SECTION_DATA,
			     le32_to_cpu(ucode->u.v2.data_size));
		set_sec_size(pieces, IWL_UCODE_INIT, IWL_UCODE_SECTION_INST,
			     le32_to_cpu(ucode->u.v2.init_size));
		set_sec_size(pieces, IWL_UCODE_INIT, IWL_UCODE_SECTION_DATA,
			     le32_to_cpu(ucode->u.v2.init_data_size));
		src = ucode->u.v2.data;
		break;
	case 0:
	case 1:
	case 2:
		hdr_size = 24;
		if (ucode_raw->size < hdr_size) {
			IWL_ERR(drv, "File size too small!\n");
			return -EINVAL;
		}
		build = 0;
		set_sec_size(pieces, IWL_UCODE_REGULAR, IWL_UCODE_SECTION_INST,
			     le32_to_cpu(ucode->u.v1.inst_size));
		set_sec_size(pieces, IWL_UCODE_REGULAR, IWL_UCODE_SECTION_DATA,
			     le32_to_cpu(ucode->u.v1.data_size));
		set_sec_size(pieces, IWL_UCODE_INIT, IWL_UCODE_SECTION_INST,
			     le32_to_cpu(ucode->u.v1.init_size));
		set_sec_size(pieces, IWL_UCODE_INIT, IWL_UCODE_SECTION_DATA,
			     le32_to_cpu(ucode->u.v1.init_data_size));
		src = ucode->u.v1.data;
		break;
	}

	if (build)
		sprintf(buildstr, " build %u", build);
	else
		buildstr[0] = '\0';

	snprintf(drv->fw.fw_version,
		 sizeof(drv->fw.fw_version),
		 "%u.%u.%u.%u%s",
		 IWL_UCODE_MAJOR(drv->fw.ucode_ver),
		 IWL_UCODE_MINOR(drv->fw.ucode_ver),
		 IWL_UCODE_API(drv->fw.ucode_ver),
		 IWL_UCODE_SERIAL(drv->fw.ucode_ver),
		 buildstr);

	/* Verify size of file vs. image size info in file's header */

	if (ucode_raw->size != hdr_size +
	    get_sec_size(pieces, IWL_UCODE_REGULAR, IWL_UCODE_SECTION_INST) +
	    get_sec_size(pieces, IWL_UCODE_REGULAR, IWL_UCODE_SECTION_DATA) +
	    get_sec_size(pieces, IWL_UCODE_INIT, IWL_UCODE_SECTION_INST) +
	    get_sec_size(pieces, IWL_UCODE_INIT, IWL_UCODE_SECTION_DATA)) {

		IWL_ERR(drv,
			"uCode file size %d does not match expected size\n",
			(int)ucode_raw->size);
		return -EINVAL;
	}


	set_sec_data(pieces, IWL_UCODE_REGULAR, IWL_UCODE_SECTION_INST, src);
	src += get_sec_size(pieces, IWL_UCODE_REGULAR, IWL_UCODE_SECTION_INST);
	set_sec_offset(pieces, IWL_UCODE_REGULAR, IWL_UCODE_SECTION_INST,
		       ib_hdr_\iAnst s {
		i/>vR>P ;VDE_SECTION_INST);
	set_sec_offset(pieces, IWL_UCODE_REGULA@&c/>g