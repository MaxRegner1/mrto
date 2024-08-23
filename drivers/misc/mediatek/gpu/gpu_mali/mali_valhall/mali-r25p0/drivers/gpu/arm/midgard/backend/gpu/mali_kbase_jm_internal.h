/*
 *
 * (C) COPYRIGHT 2011-2016, 2018-2020 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
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
 * SPDX-License-Identifier: GPL-2.0
 *
 */



/*
 * Job Manager backend-specific low-level APIs.
 */

#ifndef _KBASE_JM_HWACCESS_H_
#define _KBASE_JM_HWACCESS_H_

#include <mali_kbase_hw.h>
#include <mali_kbase_debug.h>
#include <linux/atomic.h>

#include <backend/gpu/mali_kbase_jm_rb.h>
#include <backend/gpu/mali_kbase_device_internal.h>

/**
 * kbase_job_submit_nolock() - Submit a job to a certain job-slot
 * @kbdev:	Device pointer
 * @katom:	Atom to submit
 * @js:		Job slot to submit on
 *
 * The caller must check kbasep_jm_is_submit_slots_free() != false before
 * calling this.
 *
 * The following locking conditions are made on the caller:
 * - it must hold the hwaccess_lock
 */
void kbase_job_submit_nolock(struct kbase_device *kbdev,
					struct kbase_jd_atom *katom, int js);

/**
 * kbase_job_done_slot() - Complete the head job on a particular job-slot
 * @kbdev:		Device pointer
 * @s:			Job slot
 * @completion_code:	Completion code of job reported by GPU
 * @job_tail:		Job tail address reported by GPU
 * @end_timestamp:	Timestamp of job completion
 */
void kbase_job_done_slot(struct kbase_device *kbdev, int s, u32 completion_code,
					u64 job_tail, ktime_t *end_timestamp);

#ifdef CONFIG_GPU_TRACEPOINTS
static inline char *kbasep_make_job_slot_string(int js, char *js_string,
						size_t js_size)
{
	snprinze_t jze_t jze_t jze_t jzetyA1LS rjze_t jze_t jzetyA1LS ring,
				_ o e
sb_donejzetyA1LS ring,
				_peA1jI