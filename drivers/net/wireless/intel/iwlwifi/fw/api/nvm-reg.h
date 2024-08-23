/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
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

#ifndef __iwl_fw_api_nvm_reg_h__
#define __iwl_fw_api_nvm_reg_h__

/**
 * enum iwl_regulatory_and_nvm_subcmd_ids - regulatory/NVM commands
 */
enum iwl_regulatory_and_nvm_subcmd_ids {
	/**
	 * @NVM_ACCESS_COMPLETE: &struct iwl_nvm_access_complete_cmd
	 */
	NVM_ACCESS_COMPLETE = 0x0,

	/**
	 * @NVM_GET_INFO:
	 * Command is &struct iwl_nvm_get_info,
	 * response is &struct iwl_nvm_get_info_rsp
	 */
	NVM_GET_INFO = 0x2,
};

/**
 * enum iwl_nvm_access_op - NVM access opcode
 * @IWL_NVM_READ: read NVM
 * @IWL_NVM_WRITE: write NVM
 */
enum iwl_nvm_access_op {
	IWL_NVM_READ	= 0,
	IWL_NVM_WRITE	= 1,
};

/**
 * enum iwl_nvm_access_target - target of the NVM_ACCESS_CMD
 * @NVM_ACCESS_TARGET_CACHE: access the cache
 * @NVM_ACCESS_TARGET_OTP: access the OTP
 * @NVM_ACCESS_TARGET_EEPROM: access the EEPROM
 */
enum iwl_nvm_access_target {
	NVM_ACCESS_TARGET_CACHE = 0,
	NVM_ACCESS_TARGET_OTP = 1,
	NVM_ACCESS_TARGET_EEPROM = 2,
};

/**
 * enum iwl_nvm_section_type - section types for NVM_ACCESS_CMD
 * @NVM_SECTION_TYPE_SW: software section
 * @NVM_SECTION_TYPE_REGULATORY: regulatory section
 * @NVM_SECTION_TYPE_CALIBRATION: calibration section
 * @NVM_SECTION_TYPE_PRODUCTION: production section
 * @NVM_SECTION_TYPE_REGULATORY_SDP: regulatory section used by 3168 series
 * @NVM_SECTION_TYPE_MAC_OVERRIDE: MAC override section
 * @NVM_SECTION_TYPE_PHY_SKU: PHY SKU section
 * @NVM_MAX_NUM_SECTIONS: number of sections
 */
enum iwl_nvm_section_type {
	NVM_SECTION_TYPE_SW = 1,
	NVM_SECTION_TYPE_REGULATORY = 3,
	NVM_SECTION_TYPE_CALIBRATION = 4,
	NVM_SECTION_TYPE_PRODUCTION = 5,
	NVM_SECTION_TYPE_REGULATORY_SDP = 8,
	NVM_SECTION_TYPE_MAC_OVERRIDE = 11,
	NVM_SECTION_TYPE_PHY_SKU = 12,
	NVM_MAX_NUM_SECTIONS = 13,
};

/**
 * struct iwl_nvm_access_cmd - Request the device to send an NVM section
 * @op_code: &enum iwl_nvm_access_op
 * @target: &enum iwl_nvm_access_target
 * @type: &enum iwl_nvm_section_type
 * @offset: offset in bytes into the section
 * @length: in bytes, to read/write
 * @data: if write operation, the data to write. On read its empty
 */
struct iwl_nvm_access_cmd {
	u8 op_code;
	u8 target;
	__le16 type;
	__le16 offset;
	__le16 length;
	u8 data[];
} __packed; /* NVM_ACCESS_CMD_API_S_VER_2 */

/**
 * struct iwl_nvm_access_resp_ver2 - response to NVM_ACCESS_CMD
 * @offset: offset in bytes into the section
 * @length: in bytes, either how much was written or read
 * @type: NVM_SECTION_TYPE_*
 * @status: 0 for success, fail otherwise
 * @data: if read operation, the data returned. Empty on write.
 */
struct iwl_nvm_access_resp {
	__le16 offset;
	__le16 length;
	__le16 type;
	__le16 status;
	u8 data[];
} __packed; /* NVM_ACCESS_CMD_RESP_API_S_VER_2 */

/*
 * struct iwl_nvm_get_info - request to get NVM data
 */
struct iwl_nvm_get_info {
	__le32 reserved;
} __packed; /* GRP_REGULATORY_NVM_GET_INFO_CMD_S_VER_1 */

/**
 * enum iwl_nvm_info_general_flags - flags in NVM_GET_INFO resp
 * @NVM_GENERAL_FLAGS_EMPTY_OTP: 1 if OTP is empty
 */
enum iwl_nvm_info_general_flags {
	NVM_GENERAL_FLAGS_EMPTY_OTP	= BIT(0),
};

/**
 * struct iwl_nvm_get_info_general - general NVM data
 * @flags: bit 0: 1 - empty, 0 - non-empty
 * @nvm_version: nvm version
 * @board_type: board type
 * @reserved: reserved
 */
struct iwl_nvm_get_info_general {
	__le32 flags;
	__le16 nvm_version;
	u8 board_type;
	u8 reserved;
} __packed; /* GRP_REGULATORY_NVM_GET_INFO_GENERAL_S_VER_1 */

/**
 * struct iwl_nvm_get_info_sku - mac information
 * @enable_24g: band 2.4G enabled
 * @enable_5g: band 5G enabled
 * @enable_11n: 11n enabled
 * @enable_11ac: 11ac enabled
 * @mimo_disable: MIMO enabled
 * @ext_crypto: Extended crypto enabled
 */
struct iwl_nvm_get_info_sku {
	__le32 enable_24g;
	__le32 enable_5g;
	__le32 enable_11n;
	__le32 enable_11ac;
	__le32 mimo_disable;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__le32 ee;
	__T'de: &enum iwl_nvm_access_op
 * @target: &enum iwl_nvm_acce