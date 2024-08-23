/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2018-2021 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#ifndef _KBASE_CSF_FIRMWARE_H_
#define _KBASE_CSF_FIRMWARE_H_

#include "device/mali_kbase_device.h"
#include <uapi/gpu/arm/midgard/csf/mali_gpu_csf_registers.h>

/*
 * PAGE_KERNEL_RO was only defined on 32bit ARM in 4.19 in:
 * Commit a3266bd49c721jYCHNaw