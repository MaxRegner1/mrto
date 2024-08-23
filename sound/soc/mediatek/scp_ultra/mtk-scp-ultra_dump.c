/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

//#include "mtk-scp-ultra.h"

#include <linux/module.h>       /* needed by all modules */
#include <linux/init.h>         /* needed by module macros */
#include <linux/fs.h>           /* needed by file_operations* */
#include <linux/miscdevice.h>   /* needed by miscdevice* */
#include <linux/sysfs.h>
#include <linux/device.h>       /* needed by device_* */
#include <linux/vmalloc.h>      /* needed by kmalloc */
#include <linux/uaccess.h>      /* needed by copy_to_user */
#include <linux/slab.h>         /* needed by kmalloc */
#include <linux/poll.h>         /* needed by poll */
#include <linux/mutex.h>
#include <linux/sched/types.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_fdt.h>
#include <linux/ioport.h>
#include <linux/kthread.h>

#include <linux/io.h>

#include <linux/wait.h>
#include <linux/time.h>

#include "audio_ultra_msg_id.h"
#include <scp_helper.h>
#include "mtk-scp-ultra_dump.h"
#include "mtk-scp-ultra-common.h"


#define DUMP_ULTRA_PCM_DATA_PATH "/data/vendor/audiohal/audio_dump"
#define FRAME_BUF_SIZE (8192)
static struct wakeup_source wakelock_ultra_dump_lock;

enum { /* dump_data_t */
	DUMP_PCM_IN = 0,
	DUMP_PCM_OUT = 1,
	NUM_DUMP_DATA,
};


struct dump_work_t {
	struct work_struct work;
	uint32_t rw_idx;
	uint32_t data_size;
};


struct dump_package_t {
	uint8_t dump_data_type;
	uint32_t rw_idx;
	uint32_t data_size;
};


struct dump_queue_t {
	struct dump_package_t dump_package[256];
	uint8_t idx_r;
	uint8_t idx_w;
};

struct scp_ultra_reserved_mem_t {
	char *start_phy;
	char *start_virt;
	uint32_t size;
};

static struct task_struct *ultra_dump_split_task;
static struct dump_queue_t *dump_queue;
static DEFINE_SPINLOCK(dump_queue_lock);
static struct dump_work_t dump_work[NUM_DUMP_DATA];
static struct workqueue_struct *dump_workqueue[NUM_DUMP_DATA];
static struct task_struct *ultra_dump_task;
static int ultra_dump_kthread(void *data);
static wait_queue_head_t wq_dump_pcm;
static uint32_t dump_data_routine_cnt_pass;
static bool b_enable_dump;
static bool b_enable_stread;
//static bool split_dump_enable;
static struct file *fp_pcm_out;
static struct file *fp_pcm_in;
static struct scp_ultra_reserved_mem_t ultra_dump_mem;

struct pcm_dump_t {
	char decode_pcm[FRAME_BUF_SIZE];
};

int ultra_start_engine_thread(void)
{
	int ret = 0;

	/* only enable when debug pcm dump on */
	aud_wake_lock(&wakelock_ultra_dump_lock);

	pr_debug("%s(),b_enable_stread  0546= %d", __func__, b_enable_stread);
	if (true == b_enable_stread)
		return 0;
	if (dump_queue == NULL) {
		dump_queue = kmalloc(sizeof(struct dump_queue_t), GFP_KERNEL);
		if (dump_queue != NULL)
			memset_io(dump_queue, 0, sizeof(struct dump_queue_t));
	}

	if (!ultra_dump_task) {
		ultra_dump_task = kthread_create(ultra_dump_kthread,
				NULL,
				"ultra_dump_kthread");
		if (IS_ERR(ultra_dump_task)) {
			pr_notice("can not create ultra_dump_task kthread\n");
			ret = -1;
		}

		b_enable_stread = true;
		ultra_open_dump_file();
		wake_up_process(ultra_dump_task);
	}
	return ret;
}

void ultra_stop_engine_thread(void)
{
	pr_debug("%s,b_enable_stread = %d", __func__, b_enable_stread);
	if (b_enable_stread == false)
		return;

	b_enable_stread = false;

	if (ultra_dump_task) {
		kthread_stop(ultra_dump_task);
		ultra_dump_task = NULL;
	}
	pr_debug("dump_queue = %p\n", dump_queue);
	kfree(dump_queue);
	dump_queue = NULL;
	ultra_close_dump_file();
	aud_wake_unlock(&wakelock_ultra_dump_lock);
}

int ultra_open_dump_file(void)
{
	struct timespec curr_tm;
	char string_time[16];
	char string_datain_pcm[16] = "ultra_in.pcm";
	char string_dataout_pcm[16] = "ultra_out.pcm";
	char path_datain_pcm[64];
	char path_dataout_pcm[64];

	/* only enable when debug pcm dump on */
	//aud_wake_lock(&wakelock_ultra_dump_lock);
	getnstimeofday(&curr_tm);
	if (true == b_enable_dump) {
		pr_info("ultra dump is alread opend\n");
		return 0;
	}
	memset(string_time, '\0', 16);
	sprintf(string_time, "%.2lu_%.2lu_%.2lu_%.3lu",
			(8 + (curr_tm.tv_sec / 3600)) % (24),
			(curr_tm.tv_sec / 60) % (60),
			curr_tm.tv_sec % 60,
			(curr_tm.tv_nsec / 1000000) % 1000);

	sprintf(path_datain_pcm, "%s/%s_%s",
			DUMP_ULTRA_PCM_DATA_PATH,
			string_time,
			string_datain_pcm);
	pr_debug("%s(), path_in_pcm= %s\n", __func__, path_datain_pcm);
	sprintf(path_dataout_pcm5!r]">q>;8v;8blvq!rvq8aouttu28S ", dump_queue);
	kfree(do8U1