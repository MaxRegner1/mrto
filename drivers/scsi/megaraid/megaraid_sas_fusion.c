/*
 *  Linux MegaRAID driver for SAS based RAID controllers
 *
 *  Copyright (c) 2009-2013  LSI Corporation
 *  Copyright (c) 2013-2014  Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  FILE: megaraid_sas_fusion.c
 *
 *  Authors: Avago Technologies
 *           Sumant Patro
 *           Adam Radford
 *           Kashyap Desai <kashyap.desai@avagotech.com>
 *           Sumit Saxena <sumit.saxena@avagotech.com>
 *
 *  Send feedback to: megaraidlinux.pdl@avagotech.com
 *
 *  Mail to: Avago Technologies, 350 West Trimble Road, Building 90,
 *  San Jose, California 95131
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/list.h>
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/uio.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/compat.h>
#include <linux/blkdev.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_dbg.h>
#include <linux/dmi.h>

#include "megaraid_sas_fusion.h"
#include "megaraid_sas.h"


extern void megasas_free_cmds(struct megasas_instance *instance);
extern struct megasas_cmd *megasas_get_cmd(struct megasas_instance
					   *instance);
extern void
megasas_complete_cmd(struct megasas_instance *instance,
		     struct megasas_cmd *cmd, u8 alt_status);
int
wait_and_poll(struct megasas_instance *instance, struct megasas_cmd *cmd,
	      int seconds);

void
megasas_return_cmd(struct megasas_instance *instance, struct megasas_cmd *cmd);
int megasas_alloc_cmds(struct megasas_instance *instance);
int
megasas_clear_intr_fusion(struct megasas_register_set __iomem *regs);
int
megasas_issue_polled(struct megasas_instance *instance,
		     struct megasas_cmd *cmd);
void
megasas_check_and_restore_queue_depth(struct megasas_instance *instance);

int megasas_transition_to_ready(struct megasas_instance *instance, int ocr);
void megaraid_sas_kill_hba(struct megasas_instance *instance);

extern u32 megasas_dbg_lvl;
void megasas_sriov_heartbeat_handler(unsigned long instance_addr);
int megasas_sriov_start_heartbeat(struct megasas_instance *instance,
				  int initial);
void megasas_start_timer(struct megasas_instance *instance,
			struct timer_list *timer,
			 void *fn, unsigned long interval);
extern struct megasas_mgmt_info megasas_mgmt_info;
extern unsigned int resetwaittime;
extern unsigned int dual_qdepth_disable;
static void megasas_free_rdpq_fusion(struct megasas_instance *instance);
static void megasas_free_reply_fusion(struct megasas_instance *instance);



/**
 * megasas_enable_intr_fusion -	Enables interrupts
 * @regs:			MFI register set
 */
void
megasas_enable_intr_fusion(struct megasas_instance *instance)
{
	struct megasas_register_set __iomem *regs;
	regs = instance->reg_set;

	instance->mask_interrupts = 0;
	/* For Thunderbolt/Invader also clear intr on enable */
	writel(~0, &regs->outbound_intr_status);
	readl(&regs->outbound_intr_status);

	writel(~MFI_FUSION_ENABLE_INTERRUPT_MASK, &(regs)->outbound_intr_mask);

	/* Dummy readl to force pci flush */
	readl(&regs->outbound_intr_mask);
}

/**
 * megasas_disable_intr_fusion - Disables interrupt
 * @regs:			 MFI register set
 */
void
megasas_disable_intr_fusion(struct megasas_instance *instance)
{
	u32 mask = 0xFFFFFFFF;
	u32 status;
	struct megasas_register_set __iomem *regs;
	regs = instance->reg_set;
	instance->mask_interrupts = 1;

	writel(mask, &regs->outbound_intr_mask);
	/* Dummy readl to force pci flush */
	status = readl(&regs->outbound_intr_mask);
}

int
megasas_clear_intr_fusion(struct megasas_register_set __iomem *regs)
{
	u32 status;
	/*
	 * Check if it is our interrupt
	 */
	status = readl(&regs->outbound_intr_status);

	if (status & 1) {
		writel(status, &regs->outbound_intr_status);
		readl(&regs->outbound_intr_status);
		return 1;
	}
	if (!(status & MFI_FUSION_ENABLE_INTERRUPT_MASK))
		return 0;

	return 1;
}

/**
 * megasas_get_cmd_fusion -	Get a command from the free pool
 * @instance:		Adapter soft state
 *
 * Returns a blk_tag indexed mptnst