/*
 *  Linux MegaRAID driver for SAS based RAID controllers
 *
 *  Copyright (c) 2003-2013  LSI Corporation
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
 *  Authors: Avago Technologies
 *           Sreenivas Bagalkote
 *           Sumant Patro
 *           Bo Yang
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
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/unaligned.h>
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
#include <scsi/scsi_tcq.h>
#include "megaraid_sas_fusion.h"
#include "megaraid_sas.h"

/*
 * Number of sectors per IO command
 * Will be set in megasas_init_mfi if user does not provide
 */
static unsigned int max_sectors;
module_param_named(max_sectors, max_sectors, int, 0);
MODULE_PARM_DESC(max_sectors,
	"Maximum number of sectors per IO command");

static int msix_disable;
module_param(msix_disable, int, S_IRUGO);
MODULE_PARM_DESC(msix_disable, "Disable MSI-X interrupt handling. Default: 0");

static unsigned int msix_vectors;
module_param(msix_vectors, int, S_IRUGO);
MODULE_PARM_DESC(msix_vectors, "MSI-X max vector count. Default: Set by FW");

static int allow_vf_ioctls;
module_param(allow_vf_ioctls, int, S_IRUGO);
MODULE_PARM_DESC(allow_vf_ioctls, "Allow ioctls in SR-IOV VF mode. Default: 0");

static unsigned int throttlequeuedepth = MEGASAS_THROTTLE_QUEUE_DEPTH;
module_param(throttlequeuedepth, int, S_IRUGO);
MODULE_PARM_DESC(throttlequeuedepth,
	"Adapter queue depth when throttled due to I/O timeout. Default: 16");

unsigned int resetwaittime = MEGASAS_RESET_WAIT_TIME;
module_param(resetwaittime, int, S_IRUGO);
MODULE_PARM_DESC(resetwaittime, "Wait time in seconds after I/O timeout "
		 "before resetting adapter. Default: 180");

int smp_affinity_enable = 1;
module_param(smp_affinity_enable, int, S_IRUGO);
MODULE_PARM_DESC(smp_affinity_enable, "SMP affinity feature enable/disbale Default: enable(1)");

int rdpq_enable = 1;
module_param(rdpq_enable, int, S_IRUGO);
MODULE_PARM_DESC(rdpq_enable, " Allocate reply queue in chunks for large queue depth enable/disable Default: disable(0)");

unsigned int dual_qdepth_disable;
module_param(dual_qdepth_disable, int, S_IRUGO);
MODULE_PARM_DESC(dual_qdepth_disable, "Disable dual queue depth feature. Default: 0");

unsigned int scmd_timeout = MEGASAS_DEFAULT_CMD_TIMEOUT;
module_param(scmd_timeout, int, S_IRUGO);
MODULE_PARM_DESC(scmd_timeout, "scsi command timeout (10-90s), default 90s. See megasas_reset_timer.");

MODULE_LICENSE("GPL");
MODULE_VERSION(MEGASAS_VERSION);
MODULE_AUTHOR("megaraidlinux.pdl@avagotech.com");
MODULE_DESCRIPTION("Avago MegaRAID SAS Driver");

int megasas_transition_to_ready(struct megasas_instance *instance, int ocr);
static int megasas_get_pd_list(struct megasas_instance *instance);
static int megasas_ld_list_query(struct megasas_instance *instance,
				 u8 query_type);
static int megasas_issue_init_mfi(struct megasas_instance *instance);
static int megasas_register_aen(struct megasas_instance *instance,
				u32 seq_num, u32 class_locale_word);
static void megasas_get_pd_info(struct megasas_instance *instance,
				struct scsi_device *sdev);
static int megasas_get_target_prop(struct megasas_instance *instance,
				   struct scsi_device *sdev);
/*
 * PCI ID table for all supported controllers
 */
static struct pci_device_id megasas_pci_table[] = {

	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_SAS1064R)},
	/* xscale IOP */
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_SAS1078R)},
	/* ppc IOP */
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_SAS1078DE)},
	/* ppc IOP */
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_SAS1078GEN2)},
	/* gen2*/
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_SAS0079GEN2)},
	/* gen2*/
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_SAS0073SKINNY)},
	/* skinny*/
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_SAS0071SKINNY)},
	/* skinny*/
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_VERDE_ZCR)},
	/* xscale IOP, vega */
	{PCI_DEVICE(PCI_VENDOR_ID_DELL, PCI_DEVICE_ID_DELL_PERC5)},
	/* xscale IOP */
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_FUSION)},
	/* Fusion */
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_PLASMA)},
	/* Plasma */
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_INVADER)},
	/* Invader */
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_FURY)},
	/* Fury */
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_INTRUDER)},
	/* Intruder */
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_INTRUDER_24)},
	/* Intruder 24 port*/
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_CUTLASS_52)},
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_CUTLASS_53)},
	/* VENTURA */
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_VENTURA)},
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_HARPOON)},
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_TOMCAT)},
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_VENTURA_4PORT)},
	{PCI_DEVICE(PCI_VENDOR_ID_LSI_LOGIC, PCI_DEVICE_ID_LSI_CRUSADER_4PORT)},
	{}
};

MODULE_DEVICE_TABLE(pci, megasas_pci_table);

static int megasas_mgmt_majorno;
struct megasas_mgmt_info megasas_mgmt_info;
static struct fasync_struct *megasas_async_queue;
static DEFINE_MUTEX(megasas_async_queue_mutex);

static int megasas_poll_wait_aen;
static DECLARE_WAIT_QUEUE_HEAD(megasas_poll_wait);
static u32 support_poll_for_event;
u32 megasas_dbg_lvl;
static u32 support_device_change;

/* define lock for aen poll */
spinlock_t poll_aen_lock;

void
megasas_complete_cmd(struct megasas_instance *instance, struct megasas_cmd *cmd,
		     u8 alt_status);
static u32
megasas_read_fw_status_reg_gen2(struct megasas_register_set __iomem *regs);
static int
megasas_adp_reset_gen2(struct megasas_instance *instance,
		       struct megasas_register_set __iomem *reg_set);
static irqreturn_t megasas_isr(int irq, void *devp);
static u32
megasas_init_adapter_mfi(struct megasas_instance *instance);
u32
megasas_build_and_issue_cmd(struct megasas_instance *instance,
			    struct scsi_cmnd *scmd);
static void megasas_complete_cmd_dpc(unsigned long instance_addr);
int
wait_and_poll(struct megasas_instance *instance, struct megasas_cmd *cmd,
	int seconds);
void megasas_fusion_ocr_wq(struct work_struct *work);
static int megasas_get_ld_vf_affiliation(struct megasas_instance *instance,
					 int initial);

void
megasas_issue_dcmd(struct megasas_instance *instance, struct megasas_cmd *cmd)
{
	instance->instancet->fire_cmd(instance,
		cmd->frame_phys_addr, 0, instance->reg_set);
	return;
}

/**
 * megasas_get_cmd -	Get a command from the free pool
 * @instance:		Adapter soft state
 *
 * Returns a free command from the pool
 */
struct megasas_cmd *megasas_get_cmd(struct megasas_instance
						  *instance)
{
	unsigned long flags;
	struct megasas_cmd *cmd = NULL;

	spin_lock_irqsave(&instance->mfi_pool_lock, flags);

	if (!list_empty(&instance->cmd_pool)) {
		cmd = list_entry((&instance->cmd_pool)->next,
				 struct megasas_cmd, list);
		list_del_init(&cmd->list);
	} else {
		dev_err(&instance->pdev->dev, "Command pool empty!\n");
	}

	spin_unlock_irqrestore(&instance->mfi_pool_lock, flags);
	return cmd;
}

/**
 * megasas_return_cmd -	Return a cmd to free command pool
 * @instance:		Adapter soft state
 * @cmd:		Command packet to be returned to free command pool
 */
void
megasas_return_cmd(struct megasas_instance *instance, struct megasas_cmd *cmd)
{
	unsigned long flags;
	u32 blk_tags;
	struct megasas_cmd_fusion *cmd_fusion;
	struct fusion_context *fusion = instance->ctrl_context;

	/* This flag is used only for fusion adapter.
	 * Wait for Interrupt for Polled mode DCMD
	 */
	if (cmd->flags & DRV_DCMD_POLLED_MODE)
		return;

	spin_lock_irqsave(&instance->mfi_pool_lock, flags);

	if (fusion) {
		blk_tags = instance->max_scsi_cmds + cmd->index;
		cmd_fusion = fusion->cmd_list[blk_tags];
		megasas_return_cmd_fusion(instance, cmd_fusion);
	}
	cmd->scmd = NULL;
	cmd->frame_count = 0;
	cmd->flags = 0;
	memset(cmd->frame, 0, instance->mfi_frame_size);
	cmd->frame->io.context = cpu_to_le32(cmd->index);
	if (!fusion && reset_devices)
		cmd->frame->hdr.cmd = MFI_CMD_INVALID;
	list_add(&cmd->list, (&instance->cmd_pool)->next);

	spin_unlock_irqrestore(&instance->mfi_pool_lock, flags);

}

static const char *
format_timestamp(uint32_t timestamp)
{
	static char buffer[32];

	if ((timestamp & 0xff000000) == 0xff000000)
		snprintf(buffer, sizeof(buffer), "boot + %us", timestamp &
		0x00ffffff);
	else
		snprintf(buffer, sizeof(buffer), "%us", timestamp);
	return buffer;
}

static const char *
format_class(int8_t class)
{
	static char buffer[6];

	switch (class) {
	case MFI_EVT_CLASS_DEBUG:
		return "debug";
	case MFI_EVT_CLASS_PROGRESS:
		return "progress";
	case MFI_EVT_CLASS_INFO:
		return "info";
	case MFI_EVT_CLASS_WARNING:
		return "WARN";
	case MFI_EVT_CLASS_CRITICAL:
		return "CRIT";
	case MFI_EVT_CLASS_FATAL:
		return "FATAL";
	case MFI_EVT_CLASS_DEAD:
		return "DEAD";
	default:
		snprintf(buffer, sizeof(buffer), "%d", class);
		return buffer;
	}
}

/**
  * megasas_decode_evt: Decode FW AEN event and print critical event
  * for information.
  * @instance:			Adapter soft state
  */
static void
megasas_decode_evt(struct megasas_instance *instance)
{
	struct megasas_evt_detail *evt_detail = instance->evt_detail;
	union megasas_evt_class_locale class_locale;
	class_locale.word = le32_to_cpu(evt_detail->cl.word);

	if (class_locale.members.class >= MFI_EVT_CLASS_CRITICAL)
		dev_info(&instance->pdev->dev, "%d (%s/0x%04x/%s) - %s\n",
			le32_to_cpu(evt_detail->seq_num),
			format_timestamp(le32_to_cpu(evt_detail->time_stamp)),
			(class_locale.members.locale),
			format_class(class_locale.members.class),
			evt_detail->description);
}

/**
*	The following functions are defined for xscale
*	(deviceid : 1064R, PERC5) controllers
*/

/**
 * megasas_enable_intr_xscale -	Enables interrupts
 * @regs:			MFI register set
 */
static inline void
megasas_enable_intr_xscale(struct megasas_instance *instance)
{
	struct megasas_register_set __iomem *regs;

	regs = instance->reg_set;
	writel(0, &(regs)->outbound_intr_mask);

	/* Dummy readl to force pci flush */
	readl(&regs->outbound_intr_mask);
}

/**
 * megasas_disable_intr_xscale -Disables interrupt
 * @regs:			MFI register set
 */
static inline void
megasas_disable_intr_xscale(struct megasas_instance *instance)
{
	struct megasas_register_set __iomem *regs;
	u32 mask = 0x1f;

	regs = instance->reg_set;
	writel(mask, &regs->outbound_intr_mask);
	/* Dummy readl to force pci flush */
	readl(&regs->outbound_intr_mask);
}

/**
 * megasas_read_fw_status_reg_xscale - returns the current FW status value
 * @regs:			MFI register set
 */
static u32
megasas_read_fw_status_reg_xscale(struct megasas_register_set __iomem * regs)
{
	return readl(&(regs)->outbound_msg_0);
}
/**
 * megasas_clear_interrupt_xscale -	Check & clear interrupt
 * @regs:				MFI register set
 */
static int
megasas_clear_intr_xscale(struct megasas_register_set __iomem * regs)
{
	u32 status;
	u32 mfiStatus = 0;

	/*
	 * Check if it is our interrupt
	 */
	status = readl(&regs->outbound_intr_status);

	if (status & MFI_OB_INTR_STATUS_MASK)
		mfiStatus = MFI_INTR_FLAG_REPLY_MESSAGE;
	if (status & MFI_XSCALE_OMR0_CHANGE_INTERRUPT)
		mfiStatus |= MFI_INTR_FLAG_FIRMWARE_STATE_CHANGE;

	/*
	 * Clear the interrupt by writing back the same value
	 */
	if (mfiStatus)
		writel(status, &regs->outbound_intr_status);

	/* Dummy readl to force pci flush */
	readl(&regs->outbound_intr_status);

	return mfiStatus;
}

/**
 * megasas_fire_cmd_xscale -	Sends command to the FW
 * @frame_phys_addr :		Physical address of cmd
 * @frame_count :		Number of frames for the command
 * @regs :			MFI register set
 */
static inline void
megasas_fire_cmd_xscale(struct megasas_instance *instance,
		dma_addr_t frame_phys_addr,
		u32 frame_count,
		struct megasas_register_set __iomem *regs)
{
	unsigned long flags;

	spin_lock_irqsave(&instance->hba_lock, flags);
	writel((frame_phys_addr >> 3)|(frame_count),
	       &(regs)->inbound_queue_port);
	spin_unlock_irqrestore(&instance->hba_lock, flags);
}

/**
 * megasas_adp_reset_xscale -  For controller reset
 * @regs:                              MFI register set
 */
static int
megasas_adp_reset_xscale(struct megasas_instance *instance,
	struct megasas_register_set __iomem *regs)
{
	u32 i;
	u32 pcidata;

	writel(MFI_ADP_RESET, &regs->inbound_doorbell);

	for (i = 0; i < 3; i++)
		msleep(1000); /* sleep for 3 secs */
	pcidata  = 0;
	pci_read_config_dword(instance->pdev, MFI_1068_PCSR_OFFSET, &pcidata);
	dev_notice(&instance->pdev->dev, "pcidata = %x\n", pcidata);
	if (pcidata & 0x2) {
		dev_notice(&instance->pdev->dev, "mfi 1068 offset read=%x\n", pcidata);
		pcidata &= ~0x2;
		pci_write_config_dword(instance->pdev,
				MFI_1068_PCSR_OFFSET, pcidata);

		for (i = 0; i < 2; i++)
			msleep(1000); /* need to wait 2 secs again */

		pcidata  = 0;
		pci_read_config_dword(instance->pdev,
				MFI_1068_FW_HANDSHAKE_OFFSET, &pcidata);
		dev_notice(&instance->pdev->dev, "1068 offset handshake read=%x\n", pcidata);
		if ((pcidata & 0xffff0000) == MFI_1068_FW_READY) {
			dev_notice(&instance->pdev->dev, "1068 offset pcidt=%x\n", pcidata);
			pcidata = 0;
			pci_write_config_dword(instance->pdev,
				MFI_1068_FW_HANDSHAKE_OFFSET, pcidata);
		}
	}
	return 0;
}

/**
 * megasas_check_reset_xscale -	For controller reset check
 * @regs:				MFI register set
 */
static int
megasas_check_reset_xscale(struct megasas_instance *instance,
		struct megasas_register_set __iomem *regs)
{
	if ((atomic_read(&instance->adprecovery) != MEGASAS_HBA_OPERATIONAL) &&
	    (le32_to_cpu(*instance->consumer) ==
		MEGASAS_ADPRESET_INPROG_SIGN))
		return 1;
	return 0;
}

static struct megasas_instance_template megasas_instance_template_xscale = {

	.fire_cmd = megasas_fire_cmd_xscale,
	.enable_intr = megasas_enable_intr_xscale,
	.disable_intr = megasas_disable_intr_xscale,
	.clear_intr = megasas_clear_intr_xscale,
	.read_fw_status_reg = megasas_read_fw_status_reg_xscale,
	.adp_reset = megasas_adp_reset_xscale,
	.check_reset = megasas_check_reset_xscale,
	.service_isr = megasas_isr,
	.tasklet = megasas_complete_cmd_dpc,
	.init_adapter = megasas_init_adapter_mfi,
	.build_and_issue_cmd = megasas_build_and_issue_cmd,
	.issue_dcmd = megasas_issue_dcmd,
};

/**
*	This is the end of set of functions & definitions specific
*	to xscale (deviceid : 1064R, PERC5) controllers
*/

/**
*	The following functions are defined for ppc (deviceid : 0x60)
*	controllers
*/

/**
 * megasas_enable_intr_ppc -	Enables interrupts
 * @regs:			MFI register set
 */
static inline void
megasas_enable_intr_ppc(struct megasas_instance *instance)
{
	struct megasas_register_set __iomem *regs;

	regs = instance->reg_set;
	writel(0xFFFFFFFF, &(regs)->outbound_doorbell_clear);

	writel(~0x80000000, &(regs)->outbound_intr_mask);

	/* Dummy readl to force pci flush */
	readl(&regs->outbound_intr_mask);
}

/**
 * megasas_disable_intr_ppc -	Disable interrupt
 * @regs:			MFI register set
 */
static inline void
megasas_disable_intr_ppc(struct megasas_instance *instance)
{
	struct megasas_register_set __iomem *regs;
	u32 mask = 0xFFFFFFFF;

	regs = instance->reg_set;
	writel(mask, &regs->outbound_intr_mask);
	/* Dummy readl to force pci flush */
	readl(&regs->outbound_intr_mask);
}

/**
 * megasas_read_fw_status_reg_ppc - returns the current FW status value
 * @regs:			MFI register set
 */
static u32
megasas_read_fw_status_reg_ppc(struct megasas_register_set __iomem * regs)
{
	return readl(&(regs)->outbound_scratch_pad);
}

/**
 * megasas_clear_interrupt_ppc -	Check & clear interrupt
 * @regs:				MFI register set
 */
static int
megasas_clear_intr_ppc(struct megasas_register_set __iomem * regs)
{
	u32 status, mfiStatus = 0;

	/*
	 * Check if it is our interrupt
	 */
	status = readl(&regs->outbound_intr_status);

	if (status & MFI_REPLY_1078_MESSAGE_INTERRUPT)
		mfiStatus = MFI_INTR_FLAG_REPLY_MESSAGE;

	if (status & MFI_G2_OUTBOUND_DOORBELL_CHANGE_INTERRUPT)
		mfiStatus |= MFI_INTR_FLAG_FIRMWARE_STATE_CHANGE;

	/*
	 * Clear the interrupt by writing back the same value
	 */
	writel(status, &regs->outbound_doorbell_clear);

	/* Dummy readl to force pci flush */
	readl(&regs->outbound_doorbell_clear);

	return mfiStatus;
}

/**
 * megasas_fire_cmd_ppc -	Sends command to the FW
 * @frame_phys_addr :		Physical address of cmd
 * @frame_count :		Number of frames for the command
 * @regs :			MFI register set
 */
static inline void
megasas_fire_cmd_ppc(struct megasas_instance *instance,
		dma_addr_t frame_phys_addr,
		u32 frame_count,
		struct megasas_register_set __iomem *regs)
{
	unsigned long flags;

	spin_lock_irqsave(&instance->hba_lock, flags);
	writel((frame_phys_addr | (frame_count<<1))|1,
			&(regs)->inbound_queue_port);
	spin_unlock_irqrestore(&instance->hba_lock, flags);
}

/**
 * megasas_check_reset_ppc -	For controller reset check
 * @regs:				MFI register set
 */
static int
megasas_check_reset_ppc(struct megasas_instance *instance,
			struct megasas_register_set __iomem *regs)
{
	if (atomic_read(&instance->adprecovery) != MEGASAS_HBA_OPERATIONAL)
		return 1;

	return 0;
}

static struct megasas_instance_template megasas_instance_template_ppc = {

	.fire_cmd = megasas_fire_cmd_ppc,
	.enable_intr = megasas_enable_intr_ppc,
	.disable_intr = megasas_disable_intr_ppc,
	.clear_intr = megasas_clear_intr_ppc,
	.read_fw_status_reg = megasas_read_fw_status_reg_ppc,
	.adp_reset = megasas_adp_reset_xscale,
	.check_reset = megasas_check_reset_ppc,
	.service_isr = megasas_isr,
	.tasklet = megasas_complete_cmd_dpc,
	.init_adapter = megasas_init_adapter_mfi,
	.build_and_issue_cmd = megasas_build_and_issue_cmd,
	.issue_dcmd = megasas_issue_dcmd,
};

/**
 * megasas_enable_intr_skinny -	Enables interrupts
 * @regs:			MFI register set
 */
static inline void
megasas_enable_intr_skinny(struct megasas_instance *instance)
{
	struct megasas_register_set __iomem *regs;

	regs = instance->reg_set;
	writel(0xFFFFFFFF, &(regs)->outbound_intr_mask);

	writel(~MFI_SKINNY_ENABLE_INTERRUPT_MASK, &(regs)->outbound_intr_mask);

	/* Dummy readl to force pci flush */
	readl(&regs->outbound_intr_mask);
}

/**
 * megasas_disable_intr_skinny -	Disables interrupt
 * @regs:			MFI register set
 */
static inline void
megasas_disable_intr_skinny(struct megasas_instance *instance)
{
	struct megasas_register_set __iomem *regs;
	u32 mask = 0xFFFFFFFF;

	regs = instance->reg_set;
	writel(mask, &regs->outbound_intr_mask);
	/* Dummy readl to force pci flush */
	readl(&regs->outbound_intr_mask);
}

/**
 * megasas_read_fw_status_reg_skinny - returns the current FW status value
 * @regs:			MFI register set
 */
static u32
megasas_read_fw_status_reg_skinny(struct megasas_register_set __iomem *regs)
{
	return readl(&(regs)->outbound_scratch_pad);
}

/**
 * megasas_clear_interrupt_skinny -	Check & clear interrupt
 * @regs:				MFI register set
 */
static int
megasas_clear_intr_skinny(struct megasas_register_set __iomem *regs)
{
	u32 status;
	u32 mfiStatus = 0;

	/*
	 * Check if it is our interrupt
	 */
	status = readl(&regs->outbound_intr_status);

	if (!(status & MFI_SKINNY_ENABLE_INTERRUPT_MASK)) {
		return 0;
	}

	/*
	 * Check if it is our interrupt
	 */
	if ((megasas_read_fw_status_reg_skinny(regs) & MFI_STATE_MASK) ==
	    MFI_STATE_FAULT) {
		mfiStatus = MFI_INTR_FLAG_FIRMWARE_STATE_CHANGE;
	} else
		mfiStatus = MFI_INTR_FLAG_REPLY_MESSAGE;

	/*
	 * Clear the interrupt by writing back the same value
	 */
	writel(status, &regs->outbound_intr_status);

	/*
	 * dummy read to flush PCI
	 */
	readl(&regs->outbound_intr_status);

	return mfiStatus;
}

/**
 * megasas_fire_cmd_skinny -	Sends command to the FW
 * @frame_phys_addr :		Physical address of cmd
 * @frame_count :		Number of frames for the command
 * @regs :			MFI register set
 */
static inline void
megasas_fire_cmd_skinny(struct megasas_instance *instance,
			dma_addr_t frame_phys_addr,
			u32 frame_count,
			struct megasas_register_set __iomem *regs)
{
	unsigned long flags;

	spin_lock_irqsave(&instance->hba_lock, flags);
	writel(upper_32_bits(frame_phys_addr),
	       &(regs)->inbound_high_queue_port);
	writel((lower_32_bits(frame_phys_addr) | (frame_count<<1))|1,
	       &(regs)->inbound_low_queue_port);
	mmiowb();
	spin_unlock_irqrestore(&instance->hba_lock, flags);
}

/**
 * megasas_check_reset_skinny -	For controller reset check
 * @regs:				MFI register set
 */
static int
megasas_check_reset_skinny(struct megasas_instance *instance,
				struct megasas_register_set __iomem *regs)
{
	if (atomic_read(&instance->adprecovery) != MEGASAS_HBA_OPERATIONAL)
		return 1;

	return 0;
}

static struct megasas_instance_template megasas_instance_template_skinny = {

	.fire_cmd = megasas_fire_cmd_skinny,
	.enable_intr = megasas_enable_intr_skinny,
	.disable_intr = megasas_disable_intr_skinny,
	.clear_intr = megasas_clear_intr_skinny,
	.read_fw_status_reg = megasas_read_fw_status_reg_skinny,
	.adp_reset = megasas_adp_reset_gen2,
	.check_reset = megasas_check_reset_skinny,
	.service_isr = megasas_isr,
	.tasklet = megasas_complete_cmd_dpc,
	.init_adapter = megasas_init_adapter_mfi,
	.build_and_issue_cmd = megasas_build_and_issue_cmd,
	.issue_dcmd = megasas_issue_dcmd,
};


/**
*	The following functions are defined for gen2 (deviceid : 0x78 0x79)
*	controllers
*/

/**
 * megasas_enable_intr_gen2 -  Enables interrupts
 * @regs:                      MFI register set
 */
static inline void
megasas_enable_intr_gen2(struct megasas_instance *instance)
{
	struct megasas_register_set __iomem *regs;

	regs = instance->reg_set;
	writel(0xFFFFFFFF, &(regs)->outbound_doorbell_clear);

	/* write ~0x00000005 (4 & 1) to the intr mask*/
	writel(~MFI_GEN2_ENABLE_INTERRUPT_MASK, &(regs)->outbound_intr_mask);

	/* Dummy readl to force pci flush */
	readl(&regs->outbound_intr_mask);
}

/**
 * megasas_disable_intr_gen2 - Disables interrupt
 * @regs:                      MFI register set
 */
static inline void
megasas_disable_intr_gen2(struct megasas_instance *instance)
{
	struct megasas_register_set __iomem *regs;
	u32 mask = 0xFFFFFFFF;

	regs = instance->reg_set;
	writel(mask, &regs->outbound_intr_mask);
	/* Dummy readl to force pci flush */
	readl(&regs->outbound_intr_mask);
}

/**
 * megasas_read_fw_status_reg_gen2 - returns the current FW status value
 * @regs:                      MFI register set
 */
static u32
megasas_read_fw_status_reg_gen2(struct megasas_register_set __iomem *regs)
{
	return readl(&(regs)->outbound_scratch_pad);
}

/**
 * megasas_clear_interrupt_gen2 -      Check & clear interrupt
 * @regs:                              MFI register set
 */
static int
megasas_clear_intr_gen2(struct megasas_register_set __iomem *regs)
{
	u32 status;
	u32 mfiStatus = 0;

	/*
	 * Check if it is our interrupt
	 */
	status = readl(&regs->outbound_intr_status);

	if (status & MFI_INTR_FLAG_REPLY_MESSAGE) {
		mfiStatus = MFI_INTR_FLAG_REPLY_MESSAGE;
	}
	if (status & MFI_G2_OUTBOUND_DOORBELL_CHANGE_INTERRUPT) {
		mfiStatus |= MFI_INTR_FLAG_FIRMWARE_STATE_CHANGE;
	}

	/*
	 * Clear the interrupt by writing back the same value
	 */
	if (mfiStatus)
		writel(status, &regs->outbound_doorbell_clear);

	/* Dummy readl to force pci flush */
	readl(&regs->outbound_intr_status);

	return mfiStatus;
}
/**
 * megasas_fire_cmd_gen2 -     Sends command to the FW
 * @frame_phys_addr :          Physical address of cmd
 * @frame_count :              Number of frames for the command
 * @regs :                     MFI register set
 */
static inline void
megasas_fire_cmd_gen2(struct megasas_instance *instance,
			dma_addr_t frame_phys_addr,
			u32 frame_count,
			struct megasas_register_set __iomem *regs)
{
	unsigned long flags;

	spin_lock_irqsave(&instance->hba_lock, flags);
	writel((frame_phys_addr | (frame_count<<1))|1,
			&(regs)->inbound_queue_port);
	spin_unlock_irqrestore(&instance->hba_lock, flags);
}

/**
 * megasas_adp_reset_gen2 -	For controller reset
 * @regs:				MFI register set
 */
static int
megasas_adp_reset_gen2(struct megasas_instance *instance,
			struct megasas_register_set __iomem *reg_set)
{
	u32 retry = 0 ;
	u32 HostDiag;
	u32 __iomem *seq_offset = &reg_set->seq_offset;
	u32 __iomem *hostdiag_offset = &reg_set->host_diag;

	if (instance->instancet == &megasas_instance_template_skinny) {
		seq_offset = &reg_set->fusion_seq_offset;
		hostdiag_offset = &reg_set->fusion_host_diag;
	}

	writel(0, seq_offset);
	writel(4, seq_offset);
	writel(0xb, seq_offset);
	writel(2, seq_offset);
	writel(7, seq_offset);
	writel(0xd, seq_offset);

	msleep(1000);

	HostDiag = (u32)readl(hostdiag_offset);

	while (!(HostDiag & DIAG_WRITE_ENABLE)) {
		msleep(100);
		HostDiag = (u32)readl(hostdiag_offset);
		dev_notice(&instance->pdev->dev, "RESETGEN2: retry=%x, hostdiag=%x\n",
					retry, HostDiag);

		if (retry++ >= 100)
			return 1;

	}

	dev_notice(&instance->pdev->dev, "ADP_RESET_GEN2: HostDiag=%x\n", HostDiag);

	writel((HostDiag | DIAG_RESET_ADAPTER), hostdiag_offset);

	ssleep(10);

	HostDiag = (u32)readl(hostdiag_offset);
	while (HostDiag & DIAG_RESET_ADAPTER) {
		msleep(100);
		HostDiag = (u32)readl(hostdiag_offset);
		dev_notice(&instance->pdev->dev, "RESET_GEN2: retry=%x, hostdiag=%x\n",
				retry, HostDiag);

		if (retry++ >= 1000)
			return 1;

	}
	return 0;
}

/**
 * megasas_check_reset_gen2 -	For controller reset check
 * @regs:				MFI register set
 */
static int
megasas_check_reset_gen2(struct megasas_instance *instance,
		struct megasas_register_set __iomem *regs)
{
	if (atomic_read(&instance->adprecovery) != MEGASAS_HBA_OPERATIONAL)
		return 1;

	return 0;
}

static struct megasas_instance_template megasas_instance_template_gen2 = {

	.fire_cmd = megasas_fire_cmd_gen2,
	.enable_intr = megasas_enable_intr_gen2,
	.disable_intr = megasas_disable_intr_gen2,
	.clear_intr = megasas_clear_intr_gen2,
	.read_fw_status_reg = megasas_read_fw_status_reg_gen2,
	.adp_reset = megasas_adp_reset_gen2,
	.check_reset = megasas_check_reset_gen2,
	.service_isr = megasas_isr,
	.tasklet = megasas_complete_cmd_dpc,
	.init_adapter = megasas_init_adapter_mfi,
	.build_and_issue_cmd = megasas_build_and_issue_cmd,
	.issue_dcmd = megasas_issue_dcmd,
};

/**
*	This is the end of set of functions & definitions
*       specific to gen2 (deviceid : 0x78, 0x79) controllers
*/

/*
 * Template added for TB (Fusion)
 */
extern struct megasas_instance_template megasas_instance_template_fusion;

/**
 * megasas_issue_polled -	Issues a polling command
 * @instance:			Adapter soft state
 * @cmd:			Command packet to be issued
 *
 * For polling, MFI requires the cmd_status to be set to MFI_STAT_INVALID_STATUS before posting.
 */
int
megasas_issue_polled(struct megasas_instance *instance, struct megasas_cmd *cmd)
{
	struct megasas_header *frame_hdr = &cmd->frame->hdr;

	frame_hdr->cmd_status = MFI_STAT_INVALID_STATUS;
	frame_hdr->flags |= cpu_to_le16(MFI_FRAME_DONT_POST_IN_REPLY_QUEUE);

	if (atomic_read(&instance->adprecovery) == MEGASAS_HW_CRITICAL_ERROR) {
		dev_err(&instance->pdev->dev, "Failed from %s %d\n",
			__func__, __LINE__);
		return DCMD_NOT_FIRED;
	}

	instance->instancet->issue_dcmd(instance, cmd);

	return wait_and_poll(instance, cmd, instance->requestorId ?
			MEGASAS_ROUTINE_WAIT_TIME_VF : MFI_IO_TIMEOUT_SECS);
}

/**
 * megasas_issue_blocked_cmd -	Synchronous wrapper around regular FW cmds
 * @instance:			Adapter soft state
 * @cmd:			Command to be issued
 * @timeout:			Timeout in seconds
 *
 * This function waits on an event for the command to be returned from ISR.
 * Max wait time is MEGASAS_INTERNAL_CMD_WAIT_TIME secs
 * Used to issue ioctl commands.
 */
int
megasas_issue_blocked_cmd(struct megasas_instance *instance,
			  struct megasas_cmd *cmd, int timeout)
{
	int ret = 0;
	cmd->cmd_status_drv = MFI_STAT_INVALID_STATUS;

	if (atomic_read(&instance->adprecovery) == MEGASAS_HW_CRITICAL_ERROR) {
		dev_err(&instance->pdev->dev, "Failed from %s %d\n",
			__func__, __LINE__);
		return DCMD_NOT_FIRED;
	}

	instance->instancet->issue_dcmd(instance, cmd);

	if (timeout) {
		ret = wait_event_timeout(instance->int_cmd_wait_q,
				cmd->cmd_status_drv != MFI_STAT_INVALID_STATUS, timeout * HZ);
		if (!ret) {
			dev_err(&instance->pdev->dev, "Failed from %s %d DCMD Timed out\n",
				__func__, __LINE__);
			return DCMD_TIMEOUT;
		}
	} else
		wait_event(instance->int_cmd_wait_q,
				cmd->cmd_status_drv != MFI_STAT_INVALID_STATUS);

	return (cmd->cmd_status_drv == MFI_STAT_OK) ?
		DCMD_SUCCESS : DCMD_FAILED;
}

/**
 * megasas_issue_blocked_abort_cmd -	Aborts previously issued cmd
 * @instance:				Adapter soft state
 * @cmd_to_abort:			Previously issued cmd to be aborted
 * @timeout:				Timeout in seconds
 *
 * MFI firmware can abort previously issued AEN comamnd (automatic event
 * notification). The megasas_issue_blocked_abort_cmd() issues such abort
 * cmd and waits for return status.
 * Max wait time is MEGASAS_INTERNAL_CMD_WAIT_TIME secs
 */
static int
megasas_issue_blocked_abort_cmd(struct megasas_instance *instance,
				struct megasas_cmd *cmd_to_abort, int timeout)
{
	struct megasas_cmd *cmd;
	struct megasas_abort_frame *abort_fr;
	int ret = 0;

	cmd = megasas_get_cmd(instance);

	if (!cmd)
		return -1;

	abort_fr = &cmd->frame->abort;

	/*
	 * Prepare and issue the abort frame
	 */
	abort_fr->cmd = MFI_CMD_ABORT;
	abort_fr->cmd_status = MFI_STAT_INVALID_STATUS;
	abort_fr->flags = cpu_to_le16(0);
	abort_fr->abort_context = cpu_to_le32(cmd_to_abort->index);
	abort_fr->abort_mfi_phys_addr_lo =
		cpu_to_le32(lower_32_bits(cmd_to_abort->frame_phys_addr));
	abort_fr->abort_mfi_phys_addr_hi =
		cpu_to_le32(upper_32_bits(cmd_to_abort->frame_phys_addr));

	cmd->sync_cmd = 1;
	cmd->cmd_status_drv = MFI_STAT_INVALID_STATUS;

	if (atomic_read(&instance->adprecovery) == MEGASAS_HW_CRITICAL_ERROR) {
		dev_err(&instance->pdev->dev, "Failed from %s %d\n",
			__func__, __LINE__);
		return DCMD_NOT_FIRED;
	}

	instance->instancet->issue_dcmd(instance, cmd);

	if (timeout) {
		ret = wait_event_timeout(instance->abort_cmd_wait_q,
				cmd->cmd_status_drv != MFI_STAT_INVALID_STATUS, timeout * HZ);
		if (!ret) {
			dev_err(&instance->pdev->dev, "Failed from %s %d Abort Timed out\n",
				__func__, __LINE__);
			return DCMD_TIMEOUT;
		}
	} else
		wait_event(instance->abort_cmd_wait_q,
				cmd->cmd_status_drv != MFI_STAT_INVALID_STATUS);

	cmd->sync_cmd = 0;

	megasas_return_cmd(instance, cmd);
	return (cmd->cmd_status_drv == MFI_STAT_OK) ?
		DCMD_SUCCESS : DCMD_FAILED;
}

/**
 * megasas_make_sgl32 -	Prepares 32-bit SGL
 * @instance:		Adapter soft state
 * @scp:		SCSI command from the mid-layer
 * @mfi_sgl:		SGL to be filled in
 *
 * If successful, this function returns the number of SG elements. Otherwise,
 * it returnes -1.
 */
static int
megasas_make_sgl32(struct megasas_instance *instance, struct scsi_cmnd *scp,
		   union megasas_sgl *mfi_sgl)
{
	int i;
	int sge_count;
	struct scatterlist *os_sgl;

	sge_count = scsi_dma_map(scp);
	BUG_ON(sge_count < 0);

	if (sge_count) {
		scsi_for_each_sg(scp, os_sgl, sge_count, i) {
			mfi_sgl->sge32[i].length = cpu_to_le32(sg_dma_len(os_sgl));
			mfi_sgl->sge32[i].phys_addr = cpu_to_le32(sg_dma_address(os_sgl));
		}
	}
	return sge_count;
}

/**
 * megasas_make_sgl64 -	Prepares 64-bit SGL
 * @instance:		Adapter soft state
 * @scp:		SCSI command from the mid-layer
 * @mfi_sgl:		SGL to be filled in
 *
 * If successful, this function returns the number of SG elements. Otherwise,
 * it returnes -1.
 */
static int
megasas_make_sgl64(struct megasas_instance *instance, struct scsi_cmnd *scp,
		   union megasas_sgl *mfi_sgl)
{
	int i;
	int sge_count;
	struct scatterlist *os_sgl;

	sge_count = scsi_dma_map(scp);
	BUG_ON(sge_count < 0);

	if (sge_count) {
		scsi_for_each_sg(scp, os_sgl, sge_count, i) {
			mfi_sgl->sge64[i].length = cpu_to_le32(sg_dma_len(os_sgl));
			mfi_sgl->sge64[i].phys_addr = cpu_to_le64(sg_dma_address(os_sgl));
		}
	}
	return sge_count;
}

/**
 * megasas_make_sgl_skinny - Prepares IEEE SGL
 * @instance:           Adapter soft state
 * @scp:                SCSI command from the mid-layer
 * @mfi_sgl:            SGL to be filled in
 *
 * If successful, this function returns the number of SG elements. Otherwise,
 * it returnes -1.
 */
static int
megasas_make_sgl_skinny(struct megasas_instance *instance,
		struct scsi_cmnd *scp, union megasas_sgl *mfi_sgl)
{
	int i;
	int sge_count;
	struct scatterlist *os_sgl;

	sge_count = scsi_dma_map(scp);

	if (sge_count) {
		scsi_for_each_sg(scp, os_sgl, sge_count, i) {
			mfi_sgl->sge_skinny[i].length =
				cpu_to_le32(sg_dma_len(os_sgl));
			mfi_sgl->sge_skinny[i].phys_addr =
				cpu_to_le64(sg_dma_address(os_sgl));
			mfi_sgl->sge_skinny[i].flag = cpu_to_le32(0);
		}
	}
	return sge_count;
}

 /**
 * megasas_get_frame_count - Computes the number of frames
 * @frame_type		: type of frame- io or pthru frame
 * @sge_count		: number of sg elements
 *
 * Returns the number of frames required for numnber of sge's (sge_count)
 */

static u32 megasas_get_frame_count(struct megasas_instance *instance,
			u8 sge_count, u8 frame_type)
{
	int num_cnt;
	int sge_bytes;
	u32 sge_sz;
	u32 frame_count = 0;

	sge_sz = (IS_DMA64) ? sizeof(struct megasas_sge64) :
	    sizeof(struct megasas_sge32);

	if (instance->flag_ieee) {
		sge_sz = sizeof(struct megasas_sge_skinny);
	}

	/*
	 * Main frame can contain 2 SGEs for 64-bit SGLs and
	 * 3 SGEs for 32-bit SGLs for ldio &
	 * 1 SGEs for 64-bit SGLs and
	 * 2 SGEs for 32-bit SGLs for pthru frame
	 */
	if (unlikely(frame_type == PTHRU_FRAME)) {
		if (instance->flag_ieee == 1) {
			num_cnt = sge_count - 1;
		} else if (IS_DMA64)
			num_cnt = sge_count - 1;
		else
			num_cnt = sge_count - 2;
	} else {
		if (instance->flag_ieee == 1) {
			num_cnt = sge_count - 1;
		} else if (IS_DMA64)
			num_cnt = sge_count - 2;
		else
			num_cnt = sge_count - 3;
	}

	if (num_cnt > 0) {
		sge_bytes = sge_sz * num_cnt;

		frame_count = (sge_bytes / MEGAMFI_FRAME_SIZE) +
		    ((sge_bytes % MEGAMFI_FRAME_SIZE) ? 1 : 0) ;
	}
	/* Main frame */
	frame_count += 1;

	if (frame_count > 7)
		frame_count = 8;
	return frame_count;
}

/**
 * megasas_build_dcdb -	Prepares a direct cdb (DCDB) command
 * @instance:		Adapter soft state
 * @scp:		SCSI command
 * @cmd:		Command to be prepared in
 *
 * This function prepares CDB commands. These are typcially pass-through
 * commands to the devices.
 */
static int
megasas_build_dcdb(struct megasas_instance *instance, struct scsi_cmnd *scp,
		   struct megasas_cmd *cmd)
{
	u32 is_logical;
	u32 device_id;
	u16 flags = 0;
	struct megasas_pthru_frame *pthru;

	is_logical = MEGASAS_IS_LOGICAL(scp->device);
	device_id = MEGASAS_DEV_INDEX(scp);
	pthru = (struct megasas_pthru_frame *)cmd->frame;

	if (scp->sc_data_direction == PCI_DMA_TODEVICE)
		flags = MFI_FRAME_DIR_WRITE;
	else if (scp->sc_data_direction == PCI_DMA_FROMDEVICE)
		flags = MFI_FRAME_DIR_READ;
	else if (scp->sc_data_direction == PCI_DMA_NONE)
		flags = MFI_FRAME_DIR_NONE;

	if (instance->flag_ieee == 1) {
		flags |= MFI_FRAME_IEEE;
	}

	/*
	 * Prepare the DCDB frame
	 */
	pthru->cmd = (is_logical) ? MFI_CMD_LD_SCSI_IO : MFI_CMD_PD_SCSI_IO;
	pthru->cmd_status = 0x0;
	pthru->scsi_status = 0x0;
	pthru->target_id = device_id;
	pthru->lun = scp->device->lun;
	pthru->cdb_len = scp->cmd_len;
	pthru->timeout = 0;
	pthru->pad_0 = 0;
	pthru->flags = cpu_to_le16(flags);
	pthru->data_xfer_len = cpu_to_le32(scsi_bufflen(scp));

	memcpy(pthru->cdb, scp->cmnd, scp->cmd_len);

	/*
	 * If the command is for the tape device, set the
	 * pthru timeout to the os layer timeout value.
	 */
	if (scp->device->type == TYPE_TAPE) {
		if ((scp->request->timeout / HZ) > 0xFFFF)
			pthru->timeout = cpu_to_le16(0xFFFF);
		else
			pthru->timeout = cpu_to_le16(scp->request->timeout / HZ);
	}

	/*
	 * Construct SGL
	 */
	if (instance->flag_ieee == 1) {
		pthru->flags |= cpu_to_le16(MFI_FRAME_SGL64);
		pthru->sge_count = megasas_make_sgl_skinny(instance, scp,
						      &pthru->sgl);
	} else if (IS_DMA64) {
		pthru->flags |= cpu_to_le16(MFI_FRAME_SGL64);
		pthru->sge_count = megasas_make_sgl64(instance, scp,
						      &pthru->sgl);
	} else
		pthru->sge_count = megasas_make_sgl32(instance, scp,
						      &pthru->sgl);

	if (pthru->sge_count > instance->max_num_sge) {
		dev_err(&instance->pdev->dev, "DCDB too many SGE NUM=%x\n",
			pthru->sge_count);
		return 0;
	}

	/*
	 * Sense info specific
	 */
	pthru->sense_len = SCSI_SENSE_BUFFERSIZE;
	pthru->sense_buf_phys_addr_hi =
		cpu_to_le32(upper_32_bits(cmd->sense_phys_addr));
	pthru->sense_buf_phys_addr_lo =
		cpu_to_le32(lower_32_bits(cmd->sense_phys_addr));

	/*
	 * Compute the total number of frames this command consumes. FW uses
	 * this number to pull sufficient number of frames from host memory.
	 */
	cmd->frame_count = megasas_get_frame_count(instance, pthru->sge_count,
							PTHRU_FRAME);

	return cmd->frame_count;
}

/**
 * megasas_build_ldio -	Prepares IOs to logical devices
 * @instance:		Adapter soft state
 * @scp:		SCSI command
 * @cmd:		Command to be prepared
 *
 * Frames (and accompanying SGLs) for regular SCSI IOs use this function.
 */
static int
megasas_build_ldio(struct megasas_instance *instance, struct scsi_cmnd *scp,
		   struct megasas_cmd *cmd)
{
	u32 device_id;
	u8 sc = scp->cmnd[0];
	u16 flags = 0;
	struct megasas_io_frame *ldio;

	device_id = MEGASAS_DEV_INDEX(scp);
	ldio = (struct megasas_io_frame *)cmd->frame;

	if (scp->sc_data_direction == PCI_DMA_TODEVICE)
		flags = MFI_FRAME_DIR_WRITE;
	else if (scp->sc_data_direction == PCI_DMA_FROMDEVICE)
		flags = MFI_FRAME_DIR_READ;

	if (instance->flag_ieee == 1) {
		flags |= MFI_FRAME_IEEE;
	}

	/*
	 * Prepare the Logical IO frame: 2nd bit is zero for all read cmds
	 */
	ldio->cmd = (sc & 0x02) ? MFI_CMD_LD_WRITE : MFI_CMD_LD_READ;
	ldio->cmd_status = 0x0;
	ldio->scsi_status = 0x0;
	ldio->target_id = device_id;
	ldio->timeout = 0;
	ldio->reserved_0 = 0;
	ldio->pad_0 = 0;
	ldio->flags = cpu_to_le16(flags);
	ldio->start_lba_hi = 0;
	ldio->access_byte = (scp->cmd_len != 6) ? scp->cmnd[1] : 0;

	/*
	 * 6-byte READ(0x08) or WRITE(0x0A) cdb
	 */
	if (scp->cmd_len == 6) {
		ldio->lba_count = cpu_to_le32((u32) scp->cmnd[4]);
		ldio->start_lba_lo = cpu_to_le32(((u32) scp->cmnd[1] << 16) |
						 ((u32) scp->cmnd[2] << 8) |
						 (u32) scp->cmnd[3]);

		ldio->start_lba_lo &= cpu_to_le32(0x1FFFFF);
	}

	/*
	 * 10-byte READ(0x28) or WRITE(0x2A) cdb
	 */
	else if (scp->cmd_len == 10) {
		ldio->lba_count = cpu_to_le32((u32) scp->cmnd[8] |
					      ((u32) scp->cmnd[7] << 8));
		ldio->start_lba_lo = cpu_to_le32(((u32) scp->cmnd[2] << 24) |
						 ((u32) scp->cmnd[3] << 16) |
						 ((u32) scp->cmnd[4] << 8) |
						 (u32) scp->cmnd[5]);
	}

	/*
	 * 12-byte READ(0xA8) or WRITE(0xAA) cdb
	 */
	else if (scp->cmd_len == 12) {
		ldio->lba_count = cpu_to_le32(((u32) scp->cmnd[6] << 24) |
					      ((u32) scp->cmnd[7] << 16) |
					      ((u32) scp->cmnd[8] << 8) |
					      (u32) scp->cmnd[9]);

		ldio->start_lba_lo = cpu_to_le32(((u32) scp->cmnd[2] << 24) |
						 ((u32) scp->cmnd[3] << 16) |
						 ((u32) scp->cmnd[4] << 8) |
						 (u32) scp->cmnd[5]);
	}

	/*
	 * 16-byte READ(0x88) or WRITE(0x8A) cdb
	 */
	else if (scp->cmd_len == 16) {
		ldio->lba_count = cpu_to_le32(((u32) scp->cmnd[10] << 24) |
					      ((u32) scp->cmnd[11] << 16) |
					      ((u32) scp->cmnd[12] << 8) |
					      (u32) scp->cmnd[13]);

		ldio->start_lba_lo = cpu_to_le32(((u32) scp->cmnd[6] << 24) |
						 ((u32) scp->cmnd[7] << 16) |
						 ((u32) scp->cmnd[8] << 8) |
						 (u32) scp->cmnd[9]);

		ldio->start_lba_hi = cpu_to_le32(((u32) scp->cmnd[2] << 24) |
						 ((u32) scp->cmnd[3] << 16) |
						 ((u32) scp->cmnd[4] << 8) |
						 (u32) scp->cmnd[5]);

	}

	/*
	 * Construct SGL
	 */
	if (instance->flag_ieee) {
		ldio->flags |= cpu_to_le16(MFI_FRAME_SGL64);
		ldio->sge_count = megasas_make_sgl_skinny(instance, scp,
					      &ldio->sgl);
	} else if (IS_DMA64) {
		ldio->flags |= cpu_to_le16(MFI_FRAME_SGL64);
		ldio->sge_count = megasas_make_sgl64(instance, scp, &ldio->sgl);
	} else
		ldio->sge_count = megasas_make_sgl32(instance, scp, &ldio->sgl);

	if (ldio->sge_count > instance->max_num_sge) {
		dev_err(&instance->pdev->dev, "build_ld_io: sge_count = %x\n",
			ldio->sge_count);
		return 0;
	}

	/*
	 * Sense info specific
	 */
	ldio->sense_len = SCSI_SENSE_BUFFERSIZE;
	ldio->sense_buf_phys_addr_hi = 0;
	ldio->sense_buf_phys_addr_lo = cpu_to_le32(cmd->sense_phys_addr);

	/*
	 * Compute the total number of frames this command consumes. FW uses
	 * this number to pull sufficient number of frames from host memory.
	 */
	cmd->frame_count = megasas_get_frame_count(instance,
			ldio->sge_count, IO_FRAME);

	return cmd->frame_count;
}

/**
 * megasas_cmd_type -		Checks if the cmd is for logical drive/sysPD
 *				and whether it's RW or non RW
 * @scmd:			SCSI command
 *
 */
inline int megasas_cmd_type(struct scsi_cmnd *cmd)
{
	int ret;

	switch (cmd->cmnd[0]) {
	case READ_10:
	case WRITE_10:
	case READ_12:
	case WRITE_12:
	case READ_6:
	case WRITE_6:
	case READ_16:
	case WRITE_16:
		ret = (MEGASAS_IS_LOGICAL(cmd->device)) ?
			READ_WRITE_LDIO : READ_WRITE_SYSPDIO;
		break;
	default:
		ret = (MEGASAS_IS_LOGICAL(cmd->device)) ?
			NON_READ_WRITE_LDIO : NON_READ_WRITE_SYSPDIO;
	}
	return ret;
}

 /**
 * megasas_dump_pending_frames -	Dumps the frame address of all pending cmds
 *					in FW
 * @instance:				Adapter soft state
 */
static inline void
megasas_dump_pending_frames(struct megasas_instance *instance)
{
	struct megasas_cmd *cmd;
	int i,n;
	union megasas_sgl *mfi_sgl;
	struct megasas_io_frame *ldio;
	struct megasas_pthru_frame *pthru;
	u32 sgcount;
	u16 max_cmd = instance->max_fw_cmds;

	dev_err(&instance->pdev->dev, "[%d]: Dumping Frame Phys Address of all pending cmds in FW\n",instance->host->host_no);
	dev_err(&instance->pdev->dev, "[%d]: Total OS Pending cmds : %d\n",instance->host->host_no,atomic_read(&instance->fw_outstanding));
	if (IS_DMA64)
		dev_err(&instance->pdev->dev, "[%d]: 64 bit SGLs were sent to FW\n",instance->host->host_no);
	else
		dev_err(&instance->pdev->dev, "[%d]: 32 bit SGLs were sent to FW\n",instance->host->host_no);

	dev_err(&instance->pdev->dev, "[%d]: Pending OS cmds in FW : \n",instance->host->host_no);
	for (i = 0; i < max_cmd; i++) {
		cmd = instance->cmd_list[i];
		if (!cmd->scmd)
			continue;
		dev_err(&instance->pdev->dev, "[%d]: Frame addr :0x%08lx : ",instance->host->host_no,(unsigned long)cmd->frame_phys_addr);
		if (megasas_cmd_type(cmd->scmd) == READ_WRITE_LDIO) {
			ldio = (struct megasas_io_frame *)cmd->frame;
			mfi_sgl = &ldio->sgl;
			sgcount = ldio->sge_count;
			dev_err(&instance->pdev->dev, "[%d]: frame count : 0x%x, Cmd : 0x%x, Tgt id : 0x%x,"
			" lba lo : 0x%x, lba_hi : 0x%x, sense_buf addr : 0x%x,sge count : 0x%x\n",
			instance->host->host_no, cmd->frame_count, ldio->cmd, ldio->target_id,
			le32_to_cpu(ldio->start_lba_lo), le32_to_cpu(ldio->start_lba_hi),
			le32_to_cpu(ldio->sense_buf_phys_addr_lo), sgcount);
		} else {
			pthru = (struct megasas_pthru_frame *) cmd->frame;
			mfi_sgl = &pthru->sgl;
			sgcount = pthru->sge_count;
			dev_err(&instance->pdev->dev, "[%d]: frame count : 0x%x, Cmd : 0x%x, Tgt id : 0x%x, "
			"lun : 0x%x, cdb_len : 0x%x, data xfer len : 0x%x, sense_buf addr : 0x%x,sge count : 0x%x\n",
			instance->host->host_no, cmd->frame_count, pthru->cmd, pthru->target_id,
			pthru->lun, pthru->cdb_len, le32_to_cpu(pthru->data_xfer_len),
			le32_to_cpu(pthru->sense_buf_phys_addr_lo), sgcount);
		}
		if (megasas_dbg_lvl & MEGASAS_DBG_LVL) {
			for (n = 0; n < sgcount; n++) {
				if (IS_DMA64)
					dev_err(&instance->pdev->dev, "sgl len : 0x%x, sgl addr : 0x%llx\n",
						le32_to_cpu(mfi_sgl->sge64[n].length),
						le64_to_cpu(mfi_sgl->sge64[n].phys_addr));
				else
					dev_err(&instance->pdev->dev, "sgl len : 0x%x, sgl addr : 0x%x\n",
						le32_to_cpu(mfi_sgl->sge32[n].length),
						le32_to_cpu(mfi_sgl->sge32[n].phys_addr));
			}
		}
	} /*for max_cmd*/
	dev_err(&instance->pdev->dev, "[%d]: Pending Internal cmds in FW : \n",instance->host->host_no);
	for (i = 0; i < max_cmd; i++) {

		cmd = instance->cmd_list[i];

		if (cmd->sync_cmd == 1)
			dev_err(&instance->pdev->dev, "0x%08lx : ", (unsigned long)cmd->frame_phys_addr);
	}
	dev_err(&instance->pdev->dev, "[%d]: Dumping Done\n\n",instance->host->host_no);
}

u32
megasas_build_and_issue_cmd(struct megasas_instance *instance,
			    struct scsi_cmnd *scmd)
{
	struct megasas_cmd *cmd;
	u32 frame_count;

	cmd = megasas_get_cmd(instance);
	if (!cmd)
		return SCSI_MLQUEUE_HOST_BUSY;

	/*
	 * Logical drive command
	 */
	if (megasas_cmd_type(scmd) == READ_WRITE_LDIO)
		frame_count = megasas_build_ldio(instance, scmd, cmd);
	else
		frame_count = megasas_build_dcdb(instance, scmd, cmd);

	if (!frame_count)
		goto out_return_cmd;

	cmd->scmd = scmd;
	scmd->SCp.ptr = (char *)cmd;

	/*
	 * Issue the command to the FW
	 */
	atomic_inc(&instance->fw_outstanding);

	instance->instancet->fire_cmd(instance, cmd->frame_phys_addr,
				cmd->frame_count-1, instance->reg_set);

	return 0;
out_return_cmd:
	megasas_return_cmd(instance, cmd);
	return SCSI_MLQUEUE_HOST_BUSY;
}


/**
 * megasas_queue_command -	Queue entry point
 * @scmd:			SCSI command to be queued
 * @done:			Callback entry point
 */
static int
megasas_queue_command(struct Scsi_Host *shost, struct scsi_cmnd *scmd)
{
	struct megasas_instance *instance;
	struct MR_PRIV_DEVICE *mr_device_priv_data;

	instance = (struct megasas_instance *)
	    scmd->device->host->hostdata;

	if (instance->unload == 1) {
		scmd->result = DID_NO_CONNECT << 16;
		scmd->scsi_done(scmd);
		return 0;
	}

	if (instance->issuepend_done == 0)
		return SCSI_MLQUEUE_HOST_BUSY;


	/* Check for an mpio path and adjust behavior */
	if (atomic_read(&instance->adprecovery) == MEGASAS_ADPRESET_SM_INFAULT) {
		if (megasas_check_mpio_paths(instance, scmd) ==
		    (DID_REQUEUE << 16)) {
			return SCSI_MLQUEUE_HOST_BUSY;
		} else {
			scmd->result = DID_NO_CONNECT << 16;
			scmd->scsi_done(scmd);
			return 0;
		}
	}

	if (atomic_read(&instance->adprecovery) == MEGASAS_HW_CRITICAL_ERROR) {
		scmd->result = DID_NO_CONNECT << 16;
		scmd->scsi_done(scmd);
		return 0;
	}

	mr_device_priv_data = scmd->device->hostdata;
	if (!mr_device_priv_data) {
		scmd->result = DID_NO_CONNECT << 16;
		scmd->scsi_done(scmd);
		return 0;
	}

	if (atomic_read(&instance->adprecovery) != MEGASAS_HBA_OPERATIONAL)
		return SCSI_MLQUEUE_HOST_BUSY;

	if (mr_device_priv_data->tm_busy)
		return SCSI_MLQUEUE_DEVICE_BUSY;


	scmd->result = 0;

	if (MEGASAS_IS_LOGICAL(scmd->device) &&
	    (scmd->device->id >= instance->fw_supported_vd_count ||
		scmd->device->lun)) {
		scmd->result = DID_BAD_TARGET << 16;
		goto out_done;
	}

	if ((scmd->cmnd[0] == SYNCHRONIZE_CACHE) &&
	    MEGASAS_IS_LOGICAL(scmd->device) &&
	    (!instance->fw_sync_cache_support)) {
		scmd->result = DID_OK << 16;
		goto out_done;
	}

	return instance->instancet->build_and_issue_cmd(instance, scmd);

 out_done:
	scmd->scsi_done(scmd);
	return 0;
}

static struct megasas_instance *megasas_lookup_instance(u16 host_no)
{
	int i;

	for (i = 0; i < megasas_mgmt_info.max_index; i++) {

		if ((megasas_mgmt_info.instance[i]) &&
		    (megasas_mgmt_info.instance[i]->host->host_no == host_no))
			return megasas_mgmt_info.instance[i];
	}

	return NULL;
}

/*
* megasas_set_dynamic_target_properties -
* Device property set by driver may not be static and it is required to be
* updated after OCR
*
* set tm_capable.
* set dma alignment (only for eedp protection enable vd).
*
* @sdev: OS provided scsi device
*
* Returns void
*/
void megasas_set_dynamic_target_properties(struct scsi_device *sdev)
{
	u16 pd_index = 0, ld;
	u32 device_id;
	struct megasas_instance *instance;
	struct fusion_context *fusion;
	struct MR_PRIV_DEVICE *mr_device_priv_data;
	struct MR_PD_CFG_SEQ_NUM_SYNC *pd_sync;
	struct MR_LD_RAID *raid;
	struct MR_DRV_RAID_MAP_ALL *local_map_ptr;

	instance = megasas_lookup_instance(sdev->host->host_no);
	fusion = instance->ctrl_context;
	mr_device_priv_data = sdev->hostdata;

	if (!fusion || !mr_device_priv_data)
		return;

	if (MEGASAS_IS_LOGICAL(sdev)) {
		device_id = ((sdev->channel % 2) * MEGASAS_MAX_DEV_PER_CHANNEL)
					+ sdev->id;
		local_map_ptr = fusion->ld_drv_map[(instance->map_id & 1)];
		ld = MR_TargetIdToLdGet(device_id, local_map_ptr);
		if (ld >= instance->fw_supported_vd_count)
			return;
		raid = MR_LdRaidGet(ld, local_map_ptr);

		if (raid->capability.ldPiMode == MR_PROT_INFO_TYPE_CONTROLLER)
		blk_queue_update_dma_alignment(sdev->request_queue, 0x7);

		mr_device_priv_data->is_tm_capable =
			raid->capability.tmCapable;
	} else if (instance->use_seqnum_jbod_fp) {
		pd_index = (sdev->channel * MEGASAS_MAX_DEV_PER_CHANNEL) +
			sdev->id;
		pd_sync = (void *)fusion->pd_seq_sync
				[(instance->pd_seq_map_id - 1) & 1];
		mr_device_priv_data->is_tm_capable =
			pd_sync->seq[pd_index].capability.tmCapable;
	}
}

/*
 * megasas_set_nvme_device_properties -
 * set nomerges=2
 * set virtual page boundary = 4K (current mr_nvme_pg_size is 4K).
 * set maximum io transfer = MDTS of NVME device provided by MR firmware.
 *
 * MR firmware provides value in KB. Caller of this function converts
 * kb into bytes.
 *
 * e.a MDTS=5 means 2^5 * nvme page size. (In case of 4K page size,
 * MR firmware provides value 128 as (32 * 4K) = 128K.
 *
 * @sdev:				scsi device
 * @max_io_size:				maximum io transfer size
 *
 */
static inline void
megasas_set_nvme_device_properties(struct scsi_device *sdev, u32 max_io_size)
{
	struct megasas_instance *instance;
	u32 mr_nvme_pg_size;

	instance = (struct megasas_instance *)sdev->host->hostdata;
	mr_nvme_pg_size = max_t(u32, instance->nvme_page_size,
				MR_DEFAULT_NVME_PAGE_SIZE);

	blk_queue_max_hw_sectors(sdev->request_queue, (max_io_size / 512));

	queue_flag_set_unlocked(QUEUE_FLAG_NOMERGES, sdev->request_queue);
	blk_queue_virt_boundary(sdev->request_queue, mr_nvme_pg_size - 1);
}


/*
 * megasas_set_static_target_properties -
 * Device property set by driver are static and it is not required to be
 * updated after OCR.
 *
 * set io timeout
 * set device queue depth
 * set nvme device properties. see - megasas_set_nvme_device_properties
 *
 * @sdev:				scsi device
 * @is_target_prop			true, if fw provided target properties.
 */
static void megasas_set_static_target_properties(struct scsi_device *sdev,
						 bool is_target_prop)
{
	u16	target_index = 0;
	u8 interface_type;
	u32 device_qd = MEGASAS_DEFAULT_CMD_PER_LUN;
	u32 max_io_size_kb = MR_DEFAULT_NVME_MDTS_KB;
	u32 tgt_device_qd;
	struct megasas_instance *instance;
	struct MR_PRIV_DEVICE *mr_device_priv_data;

	instance = megasas_lookup_instance(sdev->host->host_no);
	mr_device_priv_data = sdev->hostdata;
	interface_type  = mr_device_priv_data->interface_type;

	/*
	 * The RAID firmware may require extended timeouts.
	 */
	blk_queue_rq_timeout(sdev->request_queue, scmd_timeout * HZ);

	target_index = (sdev->channel * MEGASAS_MAX_DEV_PER_CHANNEL) + sdev->id;

	switch (interface_type) {
	case SAS_PD:
		device_qd = MEGASAS_SAS_QD;
		break;
	case SATA_PD:
		device_qd = MEGASAS_SATA_QD;
		break;
	case NVME_PD:
		device_qd = MEGASAS_NVME_QD;
		break;
	}

	if (is_target_prop) {
		tgt_device_qd = le32_to_cpu(instance->tgt_prop->device_qdepth);
		if (tgt_device_qd &&
		    (tgt_device_qd <= instance->host->can_queue))
			device_qd = tgt_device_qd;

		/* max_io_size_kb will be set to non zero for
		 * nvme based vd and syspd.
		 */
		max_io_size_kb = le32_to_cpu(instance->tgt_prop->max_io_size_kb);
	}

	if (instance->nvme_page_size && max_io_size_kb)
		megasas_set_nvme_device_properties(sdev, (max_io_size_kb << 10));

	scsi_change_queue_depth(sdev, device_qd);

}


static int megasas_slave_configure(struct scsi_device *sdev)
{
	u16 pd_index = 0;
	struct megasas_instance *instance;
	int ret_target_prop = DCMD_FAILED;
	bool is_target_prop = false;

	instance = megasas_lookup_instance(sdev->host->host_no);
	if (instance->pd_list_not_supported) {
		if (!MEGASAS_IS_LOGICAL(sdev) && sdev->type == TYPE_DISK) {
			pd_index = (sdev->channel * MEGASAS_MAX_DEV_PER_CHANNEL) +
				sdev->id;
			if (instance->pd_list[pd_index].driveState !=
				MR_PD_STATE_SYSTEM)
				return -ENXIO;
		}
	}

	mutex_lock(&instance->hba_mutex);
	/* Send DCMD to Firmware and cache the information */
	if ((instance->pd_info) && !MEGASAS_IS_LOGICAL(sdev))
		megasas_get_pd_info(instance, sdev);

	/* Some ventura firmware may not have instance->nvme_page_size set.
	 * Do not send MR_DCMD_DRV_GET_TARGET_PROP
	 */
	if ((instance->tgt_prop) && (instance->nvme_page_size))
		ret_target_prop = megasas_get_target_prop(instance, sdev);

	is_target_prop = (ret_target_prop == DCMD_SUCCESS) ? true : false;
	megasas_set_static_target_properties(sdev, is_target_prop);

	mutex_unlock(&instance->hba_mutex);

	/* This sdev property may change post OCR */
	megasas_set_dynamic_target_properties(sdev);

	return 0;
}

static int megasas_slave_alloc(struct scsi_device *sdev)
{
	u16 pd_index = 0;
	struct megasas_instance *instance ;
	struct MR_PRIV_DEVICE *mr_device_priv_data;

	instance = megasas_lookup_instance(sdev->host->host_no);
	if (!MEGASAS_IS_LOGICAL(sdev)) {
		/*
		 * Open the OS scan to the SYSTEM PD
		 */
		pd_index =
			(sdev->channel * MEGASAS_MAX_DEV_PER_CHANNEL) +
			sdev->id;
		if ((instance->pd_list_not_supported ||
			instance->pd_list[pd_index].driveState ==
			MR_PD_STATE_SYSTEM)) {
			goto scan_target;
		}
		return -ENXIO;
	}

scan_target:
	mr_device_priv_data = kzalloc(sizeof(*mr_device_priv_data),
					GFP_KERNEL);
	if (!mr_device_priv_data)
		return -ENOMEM;
	sdev->hostdata = mr_device_priv_data;

	atomic_set(&mr_device_priv_data->r1_ldio_hint,
		   instance->r1_ldio_hint_default);
	return 0;
}

static void megasas_slave_destroy(struct scsi_device *sdev)
{
	kfree(sdev->hostdata);
	sdev->hostdata = NULL;
}

/*
* megasas_complete_outstanding_ioctls - Complete outstanding ioctls after a
*                                       kill adapter
* @instance:				Adapter soft state
*
*/
static void megasas_complete_outstanding_ioctls(struct megasas_instance *instance)
{
	int i;
	struct megasas_cmd *cmd_mfi;
	struct megasas_cmd_fusion *cmd_fusion;
	struct fusion_context *fusion = instance->ctrl_context;

	/* Find all outstanding ioctls */
	if (fusion) {
		for (i = 0; i < instance->max_fw_cmds; i++) {
			cmd_fusion = fusion->cmd_list[i];
			if (cmd_fusion->sync_cmd_idx != (u32)ULONG_MAX) {
				cmd_mfi = instance->cmd_list[cmd_fusion->sync_cmd_idx];
				if (cmd_mfi->sync_cmd &&
				    (cmd_mfi->frame->hdr.cmd != MFI_CMD_ABORT)) {
					cmd_mfi->frame->hdr.cmd_status =
							MFI_STAT_WRONG_STATE;
					megasas_complete_cmd(instance,
							     cmd_mfi, DID_OK);
				}
			}
		}
	} else {
		for (i = 0; i < instance->max_fw_cmds; i++) {
			cmd_mfi = instance->cmd_list[i];
			if (cmd_mfi->sync_cmd && cmd_mfi->frame->hdr.cmd !=
				MFI_CMD_ABORT)
				megasas_complete_cmd(instance, cmd_mfi, DID_OK);
		}
	}
}


void megaraid_sas_kill_hba(struct megasas_instance *instance)
{
	/* Set critical error to block I/O & ioctls in case caller didn't */
	atomic_set(&instance->adprecovery, MEGASAS_HW_CRITICAL_ERROR);
	/* Wait 1 second to ensure IO or ioctls in build have posted */
	msleep(1000);
	if ((instance->pdev->device == PCI_DEVICE_ID_LSI_SAS0073SKINNY) ||
		(instance->pdev->device == PCI_DEVICE_ID_LSI_SAS0071SKINNY) ||
		(instance->adapter_type != MFI_SERIES)) {
		writel(MFI_STOP_ADP, &instance->reg_set->doorbell);
		/* Flush */
		readl(&instance->reg_set->doorbell);
		if (instance->requestorId && instance->peerIsPresent)
			memset(instance->ld_ids, 0xff, MEGASAS_MAX_LD_IDS);
	} else {
		writel(MFI_STOP_ADP,
			&instance->reg_set->inbound_doorbell);
	}
	/* Complete outstanding ioctls when adapter is killed */
	megasas_complete_outstanding_ioctls(instance);
}

 /**
  * megasas_check_and_restore_queue_depth - Check if queue depth needs to be
  *					restored to max value
  * @instance:			Adapter soft state
  *
  */
void
megasas_check_and_restore_queue_depth(struct megasas_instance *instance)
{
	unsigned long flags;

	if (instance->flag & MEGASAS_FW_BUSY
	    && time_after(jiffies, instance->last_time + 5 * HZ)
	    && atomic_read(&instance->fw_outstanding) <
	    instance->throttlequeuedepth + 1) {

		spin_lock_irqsave(instance->host->host_lock, flags);
		instance->flag &= ~MEGASAS_FW_BUSY;

		instance->host->can_queue = instance->cur_can_queue;
		spin_unlock_irqrestore(instance->host->host_lock, flags);
	}
}

/**
 * megasas_complete_cmd_dpc	 -	Returns FW's controller structure
 * @instance_addr:			Address of adapter soft state
 *
 * Tasklet to complete cmds
 */
static void megasas_complete_cmd_dpc(unsigned long instance_addr)
{
	u32 producer;
	u32 consumer;
	u32 context;
	struct megasas_cmd *cmd;
	struct megasas_instance *instance =
				(struct megasas_instance *)instance_addr;
	unsigned long flags;

	/* If we have already declared adapter dead, donot complete cmds */
	if (atomic_read(&instance->adprecovery) == MEGASAS_HW_CRITICAL_ERROR)
		return;

	spin_lock_irqsave(&instance->completion_lock, flags);

	producer = le32_to_cpu(*instance->producer);
	consumer = le32_to_cpu(*instance->consumer);

	while (consumer != producer) {
		context = le32_to_cpu(instance->reply_queue[consumer]);
		if (context >= instance->max_fw_cmds) {
			dev_err(&instance->pdev->dev, "Unexpected context value %x\n",
				context);
			BUG();
		}

		cmd = instance->cmd_list[context];

		megasas_complete_cmd(instance, cmd, DID_OK);

		consumer++;
		if (consumer == (instance->max_fw_cmds + 1)) {
			consumer = 0;
		}
	}

	*instance->consumer = cpu_to_le32(producer);

	spin_unlock_irqrestore(&instance->completion_lock, flags);

	/*
	 * Check if we can restore can_queue
	 */
	megasas_check_and_restore_queue_depth(instance);
}

/**
 * megasas_start_timer - Initializes a timer object
 * @instance:		Adapter soft state
 * @timer:		timer object to be initialized
 * @fn:			timer function
 * @interval:		time interval between timer function call
 *
 */
void megasas_start_timer(struct megasas_instance *instance,
			struct timer_list *timer,
			void *fn, unsigned long interval)
{
	init_timer(timer);
	timer->expires = jiffies + interval;
	timer->data = (unsigned long)instance;
	timer->function = fn;
	add_timer(timer);
}

static void
megasas_internal_reset_defer_cmds(struct megasas_instance *instance);

static void
process_fw_state_change_wq(struct work_struct *work);

void megasas_do_ocr(struct megasas_instance *instance)
{
	if ((instance->pdev->device == PCI_DEVICE_ID_LSI_SAS1064R) ||
	(instance->pdev->device == PCI_DEVICE_ID_DELL_PERC5) ||
	(instance->pdev->device == PCI_DEVICE_ID_LSI_VERDE_ZCR)) {
		*instance->consumer = cpu_to_le32(MEGASAS_ADPRESET_INPROG_SIGN);
	}
	instance->instancet->disable_intr(instance);
	atomic_set(&instance->adprecovery, MEGASAS_ADPRESET_SM_INFAULT);
	instance->issuepend_done = 0;

	atomic_set(&instance->fw_outstanding, 0);
	megasas_internal_reset_defer_cmds(instance);
	process_fw_state_change_wq(&instance->work_init);
}

static int megasas_get_ld_vf_affiliation_111(struct megasas_instance *instance,
					    int initial)
{
	struct megasas_cmd *cmd;
	struct megasas_dcmd_frame *dcmd;
	struct MR_LD_VF_AFFILIATION_111 *new_affiliation_111 = NULL;
	dma_addr_t new_affiliation_111_h;
	int ld, retval = 0;
	u8 thisVf;

	cmd = megasas_get_cmd(instance);

	if (!cmd) {
		dev_printk(KERN_DEBUG, &instance->pdev->dev, "megasas_get_ld_vf_affiliation_111:"
		       "Failed to get cmd for scsi%d\n",
			instance->host->host_no);
		return -ENOMEM;
	}

	dcmd = &cmd->frame->dcmd;

	if (!instance->vf_affiliation_111) {
		dev_warn(&instance->pdev->dev, "SR-IOV: Couldn't get LD/VF "
		       "affiliation for scsi%d\n", instance->host->host_no);
		megasas_return_cmd(instance, cmd);
		return -ENOMEM;
	}

	if (initial)
			memset(instance->vf_affiliation_111, 0,
			       sizeof(struct MR_LD_VF_AFFILIATION_111));
	else {
		new_affiliation_111 =
			pci_alloc_consistent(instance->pdev,
					     sizeof(struct MR_LD_VF_AFFILIATION_111),
					     &new_affiliation_111_h);
		if (!new_affiliation_111) {
			dev_printk(KERN_DEBUG, &instance->pdev->dev, "SR-IOV: Couldn't allocate "
			       "memory for new affiliation for scsi%d\n",
			       instance->host->host_no);
			megasas_return_cmd(instance, cmd);
			return -ENOMEM;
		}
		memset(new_affiliation_111, 0,
		       sizeof(struct MR_LD_VF_AFFILIATION_111));
	}

	memset(dcmd->mbox.b, 0, MFI_MBOX_SIZE);

	dcmd->cmd = MFI_CMD_DCMD;
	dcmd->cmd_status = MFI_STAT_INVALID_STATUS;
	dcmd->sge_count = 1;
	dcmd->flags = cpu_to_le16(MFI_FRAME_DIR_BOTH);
	dcmd->timeout = 0;
	dcmd->pad_0 = 0;
	dcmd->data_xfer_len =
		cpu_to_le32(sizeof(struct MR_LD_VF_AFFILIATION_111));
	dcmd->opcode = cpu_to_le32(MR_DCMD_LD_VF_MAP_GET_ALL_LDS_111);

	if (initial)
		dcmd->sgl.sge32[0].phys_addr =
			cpu_to_le32(instance->vf_affiliation_111_h);
	else
		dcmd->sgl.sge32[0].phys_addr =
			cpu_to_le32(new_affiliation_111_h);

	dcmd->sgl.sge32[0].length = cpu_to_le32(
		sizeof(struct MR_LD_VF_AFFILIATION_111));

	dev_warn(&instance->pdev->dev, "SR-IOV: Getting LD/VF affiliation for "
	       "scsi%d\n", instance->host->host_no);

	if (megasas_issue_blocked_cmd(instance, cmd, 0) != DCMD_SUCCESS) {
		dev_warn(&instance->pdev->dev, "SR-IOV: LD/VF affiliation DCMD"
		       " failed with status 0x%x for scsi%d\n",
		       dcmd->cmd_status, instance->host->host_no);
		retval = 1; /* Do a scan if we couldn't get affiliation */
		goto out;
	}

	if (!initial) {
		thisVf = new_affiliation_111->thisVf;
		for (ld = 0 ; ld < new_affiliation_111->vdCount; ld++)
			if (instance->vf_affiliation_111->map[ld].policy[thisVf] !=
			    new_affiliation_111->map[ld].policy[thisVf]) {
				dev_warn(&instance->pdev->dev, "SR-IOV: "
				       "Got new LD/VF affiliation for scsi%d\n",
				       instance->host->host_no);
				memcpy(instance->vf_affiliation_111,
				       new_affiliation_111,
				       sizeof(struct MR_LD_VF_AFFILIATION_111));
				retval = 1;
				goto out;
			}
	}
out:
	if (new_affiliation_111) {
		pci_free_consistent(instance->pdev,
				    sizeof(struct MR_LD_VF_AFFILIATION_111),
				    new_affiliation_111,
				    new_affiliation_111_h);
	}

	megasas_return_cmd(instance, cmd);

	return retval;
}

static int megasas_get_ld_vf_affiliation_12(struct megasas_instance *instance,
					    int initial)
{
	struct megasas_cmd *cmd;
	struct megasas_dcmd_frame *dcmd;
	struct MR_LD_VF_AFFILIATION *new_affiliation = NULL;
	struct MR_LD_VF_MAP *newmap = NULL, *savedmap = NULL;
	dma_addr_t new_affiliation_h;
	int i, j, retval = 0, found = 0, doscan = 0;
	u8 thisVf;

	cmd = megasas_get_cmd(instance);

	if (!cmd) {
		dev_printk(KERN_DEBUG, &instance->pdev->dev, "megasas_get_ld_vf_affiliation12: "
		       "Failed to get cmd for scsi%d\n",
		       instance->host->host_no);
		return -ENOMEM;
	}

	dcmd = &cmd->frame->dcmd;

	if (!instance->vf_affiliation) {
		dev_warn(&instance->pdev->dev, "SR-IOV: Couldn't get LD/VF "
		       "affiliation for scsi%d\n", instance->host->host_no);
		megasas_return_cmd(instance, cmd);
		return -ENOMEM;
	}

	if (initial)
		memset(instance->vf_affiliation, 0, (MAX_LOGICAL_DRIVES + 1) *
		       sizeof(struct MR_LD_VF_AFFILIATION));
	else {
		new_affiliation =
			pci_alloc_consistent(instance->pdev,
					     (MAX_LOGICAL_DRIVES + 1) *
					     sizeof(struct MR_LD_VF_AFFILIATION),
					     &new_affiliation_h);
		if (!new_affiliation) {
			dev_printk(KERN_DEBUG, &instance->pdev->dev, "SR-IOV: Couldn't allocate "
			       "memory for new affiliation for scsi%d\n",
			       instance->host->host_no);
			megasas_return_cmd(instance, cmd);
			return -ENOMEM;
		}
		memset(new_affiliation, 0, (MAX_LOGICAL_DRIVES + 1) *
		       sizeof(struct MR_LD_VF_AFFILIATION));
	}

	memset(dcmd->mbox.b, 0, MFI_MBOX_SIZE);

	dcmd->cmd = MFI_CMD_DCMD;
	dcmd->cmd_status = MFI_STAT_INVALID_STATUS;
	dcmd->sge_count = 1;
	dcmd->flags = cpu_to_le16(MFI_FRAME_DIR_BOTH);
	dcmd->timeout = 0;
	dcmd->pad_0 = 0;
	dcmd->data_xfer_len = cpu_to_le32((MAX_LOGICAL_DRIVES + 1) *
		sizeof(struct MR_LD_VF_AFFILIATION));
	dcmd->opcode = cpu_to_le32(MR_DCMD_LD_VF_MAP_GET_ALL_LDS);

	if (initial)
		dcmd->sgl.sge32[0].phys_addr =
			cpu_to_le32(instance->vf_affiliation_h);
	else
		dcmd->sgl.sge32[0].phys_addr =
			cpu_to_le32(new_affiliation_h);

	dcmd->sgl.sge32[0].length = cpu_to_le32((MAX_LOGICAL_DRIVES + 1) *
		sizeof(struct MR_LD_VF_AFFILIATION));

	dev_warn(&instance->pdev->dev, "SR-IOV: Getting LD/VF affiliation for "
	       "scsi%d\n", instance->host->host_no);


	if (megasas_issue_blocked_cmd(instance, cmd, 0) != DCMD_SUCCESS) {
		dev_warn(&instance->pdev->dev, "SR-IOV: LD/VF affiliation DCMD"
		       " failed with status 0x%x for scsi%d\n",
		       dcmd->cmd_status, instance->host->host_no);
		retval = 1; /* Do a scan if we couldn't get affiliation */
		goto out;
	}

	if (!initial) {
		if (!new_affiliation->ldCount) {
			dev_warn(&instance->pdev->dev, "SR-IOV: Got new LD/VF "
			       "affiliation for passive path for scsi%d\n",
			       instance->host->host_no);
			retval = 1;
			goto out;
		}
		newmap = new_affiliation->map;
		savedmap = instance->vf_affiliation->map;
		thisVf = new_affiliation->thisVf;
		for (i = 0 ; i < new_affiliation->ldCount; i++) {
			found = 0;
			for (j = 0; j < instance->vf_affiliation->ldCount;
			     j++) {
				if (newmap->ref.targetId ==
				    savedmap->ref.targetId) {
					found = 1;
					if (newmap->policy[thisVf] !=
					    savedmap->policy[thisVf]) {
						doscan = 1;
						goto out;
					}
				}
				savedmap = (struct MR_LD_VF_MAP *)
					((unsigned char *)savedmap +
					 savedmap->size);
			}
			if (!found && newmap->policy[thisVf] !=
			    MR_LD_ACCESS_HIDDEN) {
				doscan = 1;
				goto out;
			}
			newmap = (struct MR_LD_VF_MAP *)
				((unsigned char *)newmap + newmap->size);
		}

		newmap = new_affiliation->map;
		savedmap = instance->vf_affiliation->map;

		for (i = 0 ; i < instance->vf_affiliation->ldCount; i++) {
			found = 0;
			for (j = 0 ; j < new_affiliation->ldCount; j++) {
				if (savedmap->ref.targetId ==
				    newmap->ref.targetId) {
					found = 1;
					if (savedmap->policy[thisVf] !=
					    newmap->policy[thisVf]) {
						doscan = 1;
						goto out;
					}
				}
				newmap = (struct MR_LD_VF_MAP *)
					((unsigned char *)newmap +
					 newmap->size);
			}
			if (!found && savedmap->policy[thisVf] !=
			    MR_LD_ACCESS_HIDDEN) {
				doscan = 1;
				goto out;
			}
			savedmap = (struct MR_LD_VF_MAP *)
				((unsigned char *)savedmap +
				 savedmap->size);
		}
	}
out:
	if (doscan) {
		dev_warn(&instance->pdev->dev, "SR-IOV: Got new LD/VF "
		       "affiliation for scsi%d\n", instance->host->host_no);
		memcpy(instance->vf_affiliation, new_affiliation,
		       new_affiliation->size);
		retval = 1;
	}

	if (new_affiliation)
		pci_free_consistent(instance->pdev,
				    (MAX_LOGICAL_DRIVES + 1) *
				    sizeof(struct MR_LD_VF_AFFILIATION),
				    new_affiliation, new_affiliation_h);
	megasas_return_cmd(instance, cmd);

	return retval;
}

/* This function will get the current SR-IOV LD/VF affiliation */
static int megasas_get_ld_vf_affiliation(struct megasas_instance *instance,
	int initial)
{
	int retval;

	if (instance->PlasmaFW111)
		retval = megasas_get_ld_vf_affiliation_111(instance, initial);
	else
		retval = megasas_get_ld_vf_affiliation_12(instance, initial);
	return retval;
}

/* This function will tell FW to start the SR-IOV heartbeat */
int megasas_sriov_start_heartbeat(struct megasas_instance *instance,
					 int initial)
{
	struct megasas_cmd *cmd;
	struct megasas_dcmd_frame *dcmd;
	int retval = 0;

	cmd = megasas_get_cmd(instance);

	if (!cmd) {
		dev_printk(KERN_DEBUG, &instance->pdev->dev, "megasas_sriov_start_heartbeat: "
		       "Failed to get cmd for scsi%d\n",
		       instance->host->host_no);
		return -ENOMEM;
	}

	dcmd = &cmd->frame->dcmd;

	if (initial) {
		instance->hb_host_mem =
			pci_zalloc_consistent(instance->pdev,
					      sizeof(struct MR_CTRL_HB_HOST_MEM),
					      &instance->hb_host_mem_h);
		if (!instance->hb_host_mem) {
			dev_printk(KERN_DEBUG, &instance->pdev->dev, "SR-IOV: Couldn't allocate"
			       " memory for heartbeat host memory for scsi%d\n",
			       instance->host->host_no);
			retval = -ENOMEM;
			goto out;
		}
	}

	memset(dcmd->mbox.b, 0, MFI_MBOX_SIZE);

	dcmd->mbox.s[0] = cpu_to_le16(sizeof(struct MR_CTRL_HB_HOST_MEM));
	dcmd->cmd = MFI_CMD_DCMD;
	dcmd->cmd_status = MFI_STAT_INVALID_STATUS;
	dcmd->sge_count = 1;
	dcmd->flags = cpu_to_le16(MFI_FRAME_DIR_BOTH);
	dcmd->timeout = 0;
	dcmd->pad_0 = 0;
	dcmd->data_xfer_len = cpu_to_le32(sizeof(struct MR_CTRL_HB_HOST_MEM));
	dcmd->opcode = cpu_to_le32(MR_DCMD_CTRL_SHARED_HOST_MEM_ALLOC);
	dcmd->sgl.sge32[0].phys_addr = cpu_to_le32(instance->hb_host_mem_h);
	dcmd->sgl.sge32[0].length = cpu_to_le32(sizeof(struct MR_CTRL_HB_HOST_MEM));

	dev_warn(&instance->pdev->dev, "SR-IOV: Starting heartbeat for scsi%d\n",
	       instance->host->host_no);

	if ((instance->adapter_type != MFI_SERIES) &&
	    !instance->mask_interrupts)
		retval = megasas_issue_blocked_cmd(instance, cmd,
			MEGASAS_ROUTINE_WAIT_TIME_VF);
	else
		retval = megasas_issue_polled(instance, cmd);

	if (retval) {
		dev_warn(&instance->pdev->dev, "SR-IOV: MR_DCMD_CTRL_SHARED_HOST"
			"_MEM_ALLOC DCMD %s for scsi%d\n",
			(dcmd->cmd_status == MFI_STAT_INVALID_STATUS) ?
			"timed out" : "failed", instance->host->host_no);
		retval = 1;
	}

out:
	megasas_return_cmd(instance, cmd);

	return retval;
}

/* Handler for SR-IOV heartbeat */
void megasas_sriov_heartbeat_handler(unsigned long instance_addr)
{
	struct megasas_instance *instance =
		(struct megasas_instance *)instance_addr;

	if (instance->hb_host_mem->HB.fwCounter !=
	    instance->hb_host_mem->HB.driverCounter) {
		instance->hb_host_mem->HB.driverCounter =
			instance->hb_host_mem->HB.fwCounter;
		mod_timer(&instance->sriov_heartbeat_timer,
			  jiffies + MEGASAS_SRIOV_HEARTBEAT_INTERVAL_VF);
	} else {
		dev_warn(&instance->pdev->dev, "SR-IOV: Heartbeat never "
		       "completed for scsi%d\n", instance->host->host_no);
		schedule_work(&instance->work_init);
	}
}

/**
 * megasas_wait_for_outstanding -	Wait for all outstanding cmds
 * @instance:				Adapter soft state
 *
 * This function waits for up to MEGASAS_RESET_WAIT_TIME seconds for FW to
 * complete all its outstanding commands. Returns error if one or more IOs
 * are pending after this time period. It also marks the controller dead.
 */
static int megasas_wait_for_outstanding(struct megasas_instance *instance)
{
	int i, sl, outstanding;
	u32 reset_index;
	u32 wait_time = MEGASAS_RESET_WAIT_TIME;
	unsigned long flags;
	struct list_head clist_local;
	struct megasas_cmd *reset_cmd;
	u32 fw_state;

	if (atomic_read(&instance->adprecovery) == MEGASAS_HW_CRITICAL_ERROR) {
		dev_info(&instance->pdev->dev, "%s:%d HBA is killed.\n",
		__func__, __LINE__);
		return FAILED;
	}

	if (atomic_read(&instance->adprecovery) != MEGASAS_HBA_OPERATIONAL) {

		INIT_LIST_HEAD(&clist_local);
		spin_lock_irqsave(&instance->hba_lock, flags);
		list_splice_init(&instance->internal_reset_pending_q,
				&clist_local);
		spin_unlock_irqrestore(&instance->hba_lock, flags);

		dev_notice(&instance->pdev->dev, "HBA reset wait ...\n");
		for (i = 0; i < wait_time; i++) {
			msleep(1000);
			if (atomic_read(&instance->adprecovery) == MEGASAS_HBA_OPERATIONAL)
				break;
		}

		if (atomic_read(&instance->adprecovery) != MEGASAS_HBA_OPERATIONAL) {
			dev_notice(&instance->pdev->dev, "reset: Stopping HBA.\n");
			atomic_set(&instance->adprecovery, MEGASAS_HW_CRITICAL_ERROR);
			return FAILED;
		}

		reset_index = 0;
		while (!list_empty(&clist_local)) {
			reset_cmd = list_entry((&clist_local)->next,
						struct megasas_cmd, list);
			list_del_init(&reset_cmd->list);
			if (reset_cmd->scmd) {
				reset_cmd->scmd->result = DID_REQUEUE << 16;
				dev_notice(&instance->pdev->dev, "%d:%p reset [%02x]\n",
					reset_index, reset_cmd,
					reset_cmd->scmd->cmnd[0]);

				reset_cmd->scmd->scsi_done(reset_cmd->scmd);
				megasas_return_cmd(instance, reset_cmd);
			} else if (reset_cmd->sync_cmd) {
				dev_notice(&instance->pdev->dev, "%p synch cmds"
						"reset queue\n",
						reset_cmd);

				reset_cmd->cmd_status_drv = MFI_STAT_INVALID_STATUS;
				instance->instancet->fire_cmd(instance,
						reset_cmd->frame_phys_addr,
						0, instance->reg_set);
			} else {
				dev_notice(&instance->pdev->dev, "%p unexpected"
					"cmds lst\n",
					reset_cmd);
			}
			reset_index++;
		}

		return SUCCESS;
	}

	for (i = 0; i < resetwaittime; i++) {
		outstanding = atomic_read(&instance->fw_outstanding);

		if (!outstanding)
			break;

		if (!(i % MEGASAS_RESET_NOTICE_INTERVAL)) {
			dev_notice(&instance->pdev->dev, "[%2d]waiting for %d "
			       "commands to complete\n",i,outstanding);
			/*
			 * Call cmd completion routine. Cmd to be
			 * be completed directly without depending on isr.
			 */
			megasas_complete_cmd_dpc((unsigned long)instance);
		}

		msleep(1000);
	}

	i = 0;
	outstanding = atomic_read(&instance->fw_outstanding);
	fw_state = instance->instancet->read_fw_status_reg(instance->reg_set) & MFI_STATE_MASK;

	if ((!outstanding && (fw_state == MFI_STATE_OPERATIONAL)))
		goto no_outstanding;

	if (instance->disableOnlineCtrlReset)
		goto kill_hba_and_failed;
	do {
		if ((fw_state == MFI_STATE_FAULT) || atomic_read(&instance->fw_outstanding)) {
			dev_info(&instance->pdev->dev,
				"%s:%d waiting_for_outstanding: before issue OCR. FW state = 0x%x, oustanding 0x%x\n",
				__func__, __LINE__, fw_state, atomic_read(&instance->fw_outstanding));
			if (i == 3)
				goto kill_hba_and_failed;
			megasas_do_ocr(instance);

			if (atomic_read(&instance->adprecovery) == MEGASAS_HW_CRITICAL_ERROR) {
				dev_info(&instance->pdev->dev, "%s:%d OCR failed and HBA is killed.\n",
				__func__, __LINE__);
				return FAILED;
			}
			dev_info(&instance->pdev->dev, "%s:%d waiting_for_outstanding: after issue OCR.\n",
				__func__, __LINE__);

			for (sl = 0; sl < 10; sl++)
				msleep(500);

			outstanding = atomic_read(&instance->fw_outstanding);

			fw_state = instance->instancet->read_fw_status_reg(instance->reg_set) & MFI_STATE_MASK;
			if ((!outstanding && (fw_state == MFI_STATE_OPERATIONAL)))
				goto no_outstanding;
		}
		i++;
	} while (i <= 3);

no_outstanding:

	dev_info(&instance->pdev->dev, "%s:%d no more pending commands remain after reset handling.\n",
		__func__, __LINE__);
	return SUCCESS;

kill_hba_and_failed:

	/* Reset not supported, kill adapter */
	dev_info(&instance->pdev->dev, "%s:%d killing adapter scsi%d"
		" disableOnlineCtrlReset %d fw_outstanding %d \n",
		__func__, __LINE__, instance->host->host_no, instance->disableOnlineCtrlReset,
		atomic_read(&instance->fw_outstanding));
	megasas_dump_pending_frames(instance);
	megaraid_sas_kill_hba(instance);

	return FAILED;
}

/**
 * megasas_generic_reset -	Generic reset routine
 * @scmd:			Mid-layer SCSI command
 *
 * This routine implements a generic reset handler for device, bus and host
 * reset requests. Device, bus and host specific reset handlers can use this
 * function after they do their specific tasks.
 */
static int megasas_generic_reset(struct scsi_cmnd *scmd)
{
	int ret_val;
	struct megasas_instance *instance;

	instance = (struct megasas_instance *)scmd->device->host->hostdata;

	scmd_printk(KERN_NOTICE, scmd, "megasas: RESET cmd=%x retries=%x\n",
		 scmd->cmnd[0], scmd->retries);

	if (atomic_read(&instance->adprecovery) == MEGASAS_HW_CRITICAL_ERROR) {
		dev_err(&instance->pdev->dev, "cannot recover from previous reset failures\n");
		return FAILED;
	}

	ret_val = megasas_wait_for_outstanding(instance);
	if (ret_val == SUCCESS)
		dev_notice(&instance->pdev->dev, "reset successful\n");
	else
		dev_err(&instance->pdev->dev, "failed to do reset\n");

	return ret_val;
}

/**
 * megasas_reset_timer - quiesce the adapter if required
 * @scmd:		scsi cmnd
 *
 * Sets the FW busy flag and reduces the host->can_queue if the
 * cmd has not been completed within the timeout period.
 */
static enum
blk_eh_timer_return megasas_reset_timer(struct scsi_cmnd *scmd)
{
	struct megasas_instance *instance;
	unsigned long flags;

	if (time_after(jiffies, scmd->jiffies_at_alloc +
				(scmd_timeout * 2) * HZ)) {
		return BLK_EH_NOT_HANDLED;
	}

	instance = (struct megasas_instance *)scmd->device->host->hostdata;
	if (!(instance->flag & MEGASAS_FW_BUSY)) {
		/* FW is busy, throttle IO */
		spin_lock_irqsave(instance->host->host_lock, flags);

		instance->host->can_queue = instance->throttlequeuedepth;
		instance->last_time = jiffies;
		instance->flag |= MEGASAS_FW_BUSY;

		spin_unlock_irqrestore(instance->host->host_lock, flags);
	}
	return BLK_EH_RESET_TIMER;
}

/**
 * megasas_dump_frame -	This function will dump MPT/MFI frame
 */
static inline void
megasas_dump_frame(void *mpi_request, int sz)
{
	int i;
	__le32 *mfp = (__le32 *)mpi_request;

	printk(KERN_INFO "IO request frame:\n\t");
	for (i = 0; i < sz / sizeof(__le32); i++) {
		if (i && ((i % 8) == 0))
			printk("\n\t");
		printk("%08x ", le32_to_cpu(mfp[i]));
	}
	printk("\n");
}

/**
 * megasas_reset_bus_host -	Bus & host reset handler entry point
 */
static int megasas_reset_bus_host(struct scsi_cmnd *scmd)
{
	int ret;
	struct megasas_instance *instance;

	instance = (struct megasas_instance *)scmd->device->host->hostdata;

	scmd_printk(KERN_INFO, scmd,
		"Controller reset is requested due to IO timeout\n"
		"SCSI command pointer: (%p)\t SCSI host state: %d\t"
		" SCSI host busy: %d\t FW outstanding: %d\n",
		scmd, scmd->device->host->shost_state,
		atomic_read((atomic_t *)&scmd->device->host->host_busy),
		atomic_read(&instance->fw_outstanding));

	/*
	 * First wait for all commands to complete
	 */
	if (instance->adapter_type == MFI_SERIES) {
		ret = megasas_generic_reset(scmd);
	} else {
		struct megasas_cmd_fusion *cmd;
		cmd = (struct megasas_cmd_fusion *)scmd->SCp.ptr;
		if (cmd)
			megasas_dump_frame(cmd->io_request,
				MEGA_MPI2_RAID_DEFAULT_IO_FRAME_SIZE);
		ret = megasas_reset_fusion(scmd->device->host,
				SCSIIO_TIMEOUT_OCR);
	}

	return ret;
}

/**
 * megasas_task_abort - Issues task abort request to firmware
 *			(supported only for fusion adapters)
 * @scmd:		SCSI command pointer
 */
static int megasas_task_abort(struct scsi_cmnd *scmd)
{
	int ret;
	struct megasas_instance *instance;

	instance = (struct megasas_instance *)scmd->device->host->hostdata;

	if (instance->adapter_type != MFI_SERIES)
		ret = megasas_task_abort_fusion(scmd);
	else {
		sdev_printk(KERN_NOTICE, scmd->device, "TASK ABORT not supported\n");
		ret = FAILED;
	}

	return ret;
}

/**
 * megasas_reset_target:  Issues target reset request to firmware
 *                        (supported only for fusion adapters)
 * @scmd:                 SCSI command pointer
 */
static int megasas_reset_target(struct scsi_cmnd *scmd)
{
	int ret;
	struct megasas_instance *instance;

	instance = (struct megasas_instance *)scmd->device->host->hostdata;

	if (instance->adapter_type != MFI_SERIES)
		ret = megasas_reset_target_fusion(scmd);
	else {
		sdev_printk(KERN_NOTICE, scmd->device, "TARGET RESET not supported\n");
		ret = FAILED;
	}

	return ret;
}

/**
 * megasas_bios_param - Returns disk geometry for a disk
 * @sdev:		device handle
 * @bdev:		block device
 * @capacity:		drive capacity
 * @geom:		geometry parameters
 */
static int
megasas_bios_param(struct scsi_device *sdev, struct block_device *bdev,
		 sector_t capacity, int geom[])
{
	int heads;
	int sectors;
	sector_t cylinders;
	unsigned long tmp;

	/* Default heads (64) & sectors (32) */
	heads = 64;
	sectors = 32;

	tmp = heads * sectors;
	cylinders = capacity;

	sector_div(cylinders, tmp);

	/*
	 * Handle extended translation size for logical drives > 1Gb
	 */

	if (capacity >= 0x200000) {
		heads = 255;
		sectors = 63;
		tmp = heads*sectors;
		cylinders = capacity;
		sector_div(cylinders, tmp);
	}

	geom[0] = heads;
	geom[1] = sectors;
	geom[2] = cylinders;

	return 0;
}

static void megasas_aen_polling(struct work_struct *work);

/**
 * megasas_service_aen -	Processes an event notification
 * @instance:			Adapter soft state
 * @cmd:			AEN command completed by the ISR
 *
 * For AEN, driver sends a command down to FW that is held by the FW till an
 * event occurs. When an event of interest occurs, FW completes the command
 * that it was previously holding.
 *
 * This routines sends SIGIO signal to processes that have registered with the
 * driver for AEN.
 */
static void
megasas_service_aen(struct megasas_instance *instance, struct megasas_cmd *cmd)
{
	unsigned long flags;

	/*
	 * Don't signal app if it is just an aborted previously registered aen
	 */
	if ((!cmd->abort_aen) && (instance->unload == 0)) {
		spin_lock_irqsave(&poll_aen_lock, flags);
		megasas_poll_wait_aen = 1;
		spin_unlock_irqrestore(&poll_aen_lock, flags);
		wake_up(&megasas_poll_wait);
		kill_fasync(&megasas_async_queue, SIGIO, POLL_IN);
	}
	else
		cmd->abort_aen = 0;

	instance->aen_cmd = NULL;

	megasas_return_cmd(instance, cmd);

	if ((instance->unload == 0) &&
		((instance->issuepend_done == 1))) {
		struct megasas_aen_event *ev;

		ev = kzalloc(sizeof(*ev), GFP_ATOMIC);
		if (!ev) {
			dev_err(&instance->pdev->dev, "megasas_service_aen: out of memory\n");
		} else {
			ev->instance = instance;
			instance->ev = ev;
			INIT_DELAYED_WORK(&ev->hotplug_work,
					  megasas_aen_polling);
			schedule_delayed_work(&ev->hotplug_work, 0);
		}
	}
}

static ssize_t
megasas_fw_crash_buffer_store(struct device *cdev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct Scsi_Host *shost = class_to_shost(cdev);
	struct megasas_instance *instance =
		(struct megasas_instance *) shost->hostdata;
	int val = 0;
	unsigned long flags;

	if (kstrtoint(buf, 0, &val) != 0)
		return -EINVAL;

	spin_lock_irqsave(&instance->crashdump_lock, flags);
	instance->fw_crash_buffer_offset = val;
	spin_unlock_irqrestore(&instance->crashdump_lock, flags);
	return strlen(buf);
}

static ssize_t
megasas_fw_crash_buffer_show(struct device *cdev,
	struct device_attribute *attr, char *buf)
{
	struct Scsi_Host *shost = class_to_shost(cdev);
	struct megasas_instance *instance =
		(struct megasas_instance *) shost->hostdata;
	u32 size;
	unsigned long buff_addr;
	unsigned long dmachunk = CRASH_DMA_BUF_SIZE;
	unsigned long chunk_left_bytes;
	unsigned long src_addr;
	unsigned long flags;
	u32 buff_offset;

	spin_lock_irqsave(&instance->crashdump_lock, flags);
	buff_offset = instance->fw_crash_buffer_offset;
	if (!instance->crash_dump_buf &&
		!((instance->fw_crash_state == AVAILABLE) ||
		(instance->fw_crash_state == COPYING))) {
		dev_err(&instance->pdev->dev,
			"Firmware crash dump is not available\n");
		spin_unlock_irqrestore(&instance->crashdump_lock, flags);
		return -EINVAL;
	}

	buff_addr = (unsigned long) buf;

	if (buff_offset > (instance->fw_crash_buffer_size * dmachunk)) {
		dev_err(&instance->pdev->dev,
			"Firmware crash dump offset is out of range\n");
		spin_unlock_irqrestore(&instance->crashdump_lock, flags);
		return 0;
	}

	size = (instance->fw_crash_buffer_size * dmachunk) - buff_offset;
	chunk_left_bytes = dmachunk - (buff_offset % dmachunk);
	size = (size > chunk_left_bytes) ? chunk_left_bytes : size;
	size = (size >= PAGE_SIZE) ? (PAGE_SIZE - 1) : size;

	src_addr = (unsigned long)instance->crash_buf[buff_offset / dmachunk] +
		(buff_offset % dmachunk);
	memcpy(buf, (void *)src_addr, size);
	spin_unlock_irqrestore(&instance->crashdump_lock, flags);

	return size;
}

static ssize_t
megasas_fw_crash_buffer_size_show(struct device *cdev,
	struct device_attribute *attr, char *buf)
{
	struct Scsi_Host *shost = class_to_shost(cdev);
	struct megasas_instance *instance =
		(struct megasas_instance *) shost->hostdata;

	return snprintf(buf, PAGE_SIZE, "%ld\n", (unsigned long)
		((instance->fw_crash_buffer_size) * 1024 * 1024)/PAGE_SIZE);
}

static ssize_t
megasas_fw_crash_state_store(struct device *cdev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct Scsi_Host *shost = class_to_shost(cdev);
	struct megasas_instance *instance =
		(struct megasas_instance *) shost->hostdata;
	int val = 0;
	unsigned long flags;

	if (kstrtoint(buf, 0, &val) != 0)
		return -EINVAL;

	if ((val <= AVAILABLE || val > COPY_ERROR)) {
		dev_err(&instance->pdev->dev, "application updates invalid "
			"firmware crash state\n");
		return -EINVAL;
	}

	instance->fw_crash_state = val;

	if ((val == COPIED) || (val == COPY_ERROR)) {
		spin_lock_irqsave(&instance->crashdump_lock, flags);
		megasas_free_host_crash_buffer(instance);
		spin_unlock_irqrestore(&instance->crashdump_lock, flags);
		if (val == COPY_ERROR)
			dev_info(&instance->pdev->dev, "application failed to "
				"copy Firmware crash dump\n");
		else
			dev_info(&instance->pdev->dev, "Firmware crash dump "
				"copied successfully\n");
	}
	return strlen(buf);
}

static ssize_t
megasas_fw_crash_state_show(struct device *cdev,
	struct device_attribute *attr, char *buf)
{
	struct Scsi_Host *shost = class_to_shost(cdev);
	struct megasas_instance *instance =
		(struct megasas_instance *) shost->hostdata;

	return snprintf(buf, PAGE_SIZE, "%d\n", instance->fw_crash_state);
}

static ssize_t
megasas_page_size_show(struct device *cdev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%ld\n", (unsigned long)PAGE_SIZE - 1);
}

static ssize_t
megasas_ldio_outstanding_show(struct device *cdev, struct device_attribute *attr,
	char *buf)
{
	struct Scsi_Host *shost = class_to_shost(cdev);
	struct megasas_instance *instance = (struct megasas_instance *)shost->hostdata;

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&instance->ldio_outstanding));
}

static DEVICE_ATTR(fw_crash_buffer, S_IRUGO | S_IWUSR,
	megasas_fw_crash_buffer_show, megasas_fw_crash_buffer_store);
static DEVICE_ATTR(fw_crash_buffer_size, S_IRUGO,
	megasas_fw_crash_buffer_size_show, NULL);
static DEVICE_ATTR(fw_crash_state, S_IRUGO | S_IWUSR,
	megasas_fw_crash_state_show, megasas_fw_crash_state_store);
static DEVICE_ATTR(page_size, S_IRUGO,
	megasas_page_size_show, NULL);
static DEVICE_ATTR(ldio_outstanding, S_IRUGO,
	megasas_ldio_outstanding_show, NULL);

struct device_attribute *megaraid_host_attrs[] = {
	&dev_attr_fw_crash_buffer_size,
	&dev_attr_fw_crash_buffer,
	&dev_attr_fw_crash_state,
	&dev_attr_page_size,
	&dev_attr_ldio_outstanding,
	NULL,
};

/*
 * Scsi host template for megaraid_sas driver
 */
static struct scsi_host_template megasas_template = {

	.module = THIS_MODULE,
	.name = "Avago SAS based MegaRAID driver",
	.proc_name = "megaraid_sas",
	.slave_configure = megasas_slave_configure,
	.slave_alloc = megasas_slave_alloc,
	.slave_destroy = megasas_slave_destroy,
	.queuecommand = megasas_queue_command,
	.eh_target_reset_handler = megasas_reset_target,
	.eh_abort_handler = megasas_task_abort,
	.eh_host_reset_handler = megasas_reset_bus_host,
	.eh_timed_out = megasas_reset_timer,
	.shost_attrs = megaraid_host_attrs,
	.bios_param = megasas_bios_param,
	.use_clustering = ENABLE_CLUSTERING,
	.change_queue_depth = scsi_change_queue_depth,
	.no_write_same = 1,
};

/**
 * megasas_complete_int_cmd -	Completes an internal command
 * @instance:			Adapter soft state
 * @cmd:			Command to be completed
 *
 * The megasas_issue_blocked_cmd() function waits for a command to complete
 * after it issues a command. This function wakes up that waiting routine by
 * calling wake_up() on the wait queue.
 */
static void
megasas_complete_int_cmd(struct megasas_instance *instance,
			 struct megasas_cmd *cmd)
{
	cmd->cmd_status_drv = cmd->frame->io.cmd_status;
	wake_up(&instance->int_cmd_wait_q);
}

/**
 * megasas_complete_abort -	Completes aborting a command
 * @instance:			Adapter soft state
 * @cmd:			Cmd that was issued to abort another cmd
 *
 * The megasas_issue_blocked_abort_cmd() function waits on abort_cmd_wait_q
 * after it issues an abort on a previously issued command. This function
 * wakes up all functions waiting on the same wait queue.
 */
static void
megasas_complete_abort(struct megasas_instance *instance,
		       struct megasas_cmd *cmd)
{
	if (cmd->sync_cmd) {
		cmd->sync_cmd = 0;
		cmd->cmd_status_drv = 0;
		wake_up(&instance->abort_cmd_wait_q);
	}
}

/**
 * megasas_complete_cmd -	Completes a command
 * @instance:			Adapter soft state
 * @cmd:			Command to be completed
 * @alt_status:			If non-zero, use this value as status to
 *				SCSI mid-layer instead of the value returned
 *				by the FW. This should be used if caller wants
 *				an alternate status (as in the case of aborted
 *				commands)
 */
void
megasas_complete_cmd(struct megasas_instance *instance, struct megasas_cmd *cmd,
		     u8 alt_status)
{
	int exception = 0;
	struct megasas_header *hdr = &cmd->frame->hdr;
	unsigned long flags;
	struct fusion_context *fusion = instance->ctrl_context;
	u32 opcode, status;

	/* flag for the retry reset */
	cmd->retry_for_fw_reset = 0;

	if (cmd->scmd)
		cmd->scmd->SCp.ptr = NULL;

	switch (hdr->cmd) {
	case MFI_CMD_INVALID:
		/* Some older 1068 controller FW may keep a pended
		   MR_DCMD_CTRL_EVENT_GET_INFO left over from the main kernel
		   when booting the kdump kernel.  Ignore this command to
		   prevent a kernel panic on shutdown of the kdump kernel. */
		dev_warn(&instance->pdev->dev, "MFI_CMD_INVALID command "
		       "completed\n");
		dev_warn(&instance->pdev->dev, "If you have a controller "
		       "other than PERC5, please upgrade your firmware\n");
		break;
	case MFI_CMD_PD_SCSI_IO:
	case MFI_CMD_LD_SCSI_IO:

		/*
		 * MFI_CMD_PD_SCSI_IO and MFI_CMD_LD_SCSI_IO could have been
		 * issued either through an IO path or an IOCTL path. If it
		 * was via IOCTL, we will send it to internal completion.
		 */
		if (cmd->sync_cmd) {
			cmd->sync_cmd = 0;
			megasas_complete_int_cmd(instance, cmd);
			break;
		}

	case MFI_CMD_LD_READ:
	case MFI_CMD_LD_WRITE:

		if (alt_status) {
			cmd->scmd->result = alt_status << 16;
			exception = 1;
		}

		if (exception) {

			atomic_dec(&instance->fw_outstanding);

			scsi_dma_unmap(cmd->scmd);
			cmd->scmd->scsi_done(cmd->scmd);
			megasas_return_cmd(instance, cmd);

			break;
		}

		switch (hdr->cmd_status) {

		case MFI_STAT_OK:
			cmd->scmd->result = DID_OK << 16;
			break;

		case MFI_STAT_SCSI_IO_FAILED:
		case MFI_STAT_LD_INIT_IN_PROGRESS:
			cmd->scmd->result =
			    (DID_ERROR << 16) | hdr->scsi_status;
			break;

		case MFI_STAT_SCSI_DONE_WITH_ERROR:

			cmd->scmd->result = (DID_OK << 16) | hdr->scsi_status;

			if (hdr->scsi_status == SAM_STAT_CHECK_CONDITION) {
				memset(cmd->scmd->sense_buffer, 0,
				       SCSI_SENSE_BUFFERSIZE);
				memcpy(cmd->scmd->sense_buffer, cmd->sense,
				       hdr->sense_len);

				cmd->scmd->result |= DRIVER_SENSE << 24;
			}

			break;

		case MFI_STAT_LD_OFFLINE:
		case MFI_STAT_DEVICE_NOT_FOUND:
			cmd->scmd->result = DID_BAD_TARGET << 16;
			break;

		default:
			dev_printk(KERN_DEBUG, &instance->pdev->dev, "MFI FW status %#x\n",
			       hdr->cmd_status);
			cmd->scmd->result = DID_ERROR << 16;
			break;
		}

		atomic_dec(&instance->fw_outstanding);

		scsi_dma_unmap(cmd->scmd);
		cmd->scmd->scsi_done(cmd->scmd);
		megasas_return_cmd(instance, cmd);

		break;

	case MFI_CMD_SMP:
	case MFI_CMD_STP:
	case MFI_CMD_DCMD:
		opcode = le32_to_cpu(cmd->frame->dcmd.opcode);
		/* Check for LD map update */
		if ((opcode == MR_DCMD_LD_MAP_GET_INFO)
			&& (cmd->frame->dcmd.mbox.b[1] == 1)) {
			fusion->fast_path_io = 0;
			spin_lock_irqsave(instance->host->host_lock, flags);
			instance->map_update_cmd = NULL;
			if (cmd->frame->hdr.cmd_status != 0) {
				if (cmd->frame->hdr.cmd_status !=
				    MFI_STAT_NOT_FOUND)
					dev_warn(&instance->pdev->dev, "map syncfailed, status = 0x%x\n",
					       cmd->frame->hdr.cmd_status);
				else {
					megasas_return_cmd(instance, cmd);
					spin_unlock_irqrestore(
						instance->host->host_lock,
						flags);
					break;
				}
			} else
				instance->map_id++;
			megasas_return_cmd(instance, cmd);

			/*
			 * Set fast path IO to ZERO.
			 * Validate Map will set proper value.
			 * Meanwhile all IOs will go as LD IO.
			 */
			if (MR_ValidateMapInfo(instance))
				fusion->fast_path_io = 1;
			else
				fusion->fast_path_io = 0;
			megasas_sync_map_info(instance);
			spin_unlock_irqrestore(instance->host->host_lock,
					       flags);
			break;
		}
		if (opcode == MR_DCMD_CTRL_EVENT_GET_INFO ||
		    opcode == MR_DCMD_CTRL_EVENT_GET) {
			spin_lock_irqsave(&poll_aen_lock, flags);
			megasas_poll_wait_aen = 0;
			spin_unlock_irqrestore(&poll_aen_lock, flags);
		}

		/* FW has an updated PD sequence */
		if ((opcode == MR_DCMD_SYSTEM_PD_MAP_GET_INFO) &&
			(cmd->frame->dcmd.mbox.b[0] == 1)) {

			spin_lock_irqsave(instance->host->host_lock, flags);
			status = cmd->frame->hdr.cmd_status;
			instance->jbod_seq_cmd = NULL;
			megasas_return_cmd(instance, cmd);

			if (status == MFI_STAT_OK) {
				instance->pd_seq_map_id++;
				/* Re-register a pd sync seq num cmd */
				if (megasas_sync_pd_seq_num(instance, true))
					instance->use_seqnum_jbod_fp = false;
			} else
				instance->use_seqnum_jbod_fp = false;

			spin_unlock_irqrestore(instance->host->host_lock, flags);
			break;
		}

		/*
		 * See if got an event notification
		 */
		if (opcode == MR_DCMD_CTRL_EVENT_WAIT)
			megasas_service_aen(instance, cmd);
		else
			megasas_complete_int_cmd(instance, cmd);

		break;

	case MFI_CMD_ABORT:
		/*
		 * Cmd issued to abort another cmd returned
		 */
		megasas_complete_abort(instance, cmd);
		break;

	default:
		dev_info(&instance->pdev->dev, "Unknown command completed! [0x%X]\n",
		       hdr->cmd);
		break;
	}
}

/**
 * megasas_issue_pending_cmds_again -	issue all pending cmds
 *					in FW again because of the fw reset
 * @instance:				Adapter soft state
 */
static inline void
megasas_issue_pending_cmds_again(struct megasas_instance *instance)
{
	struct megasas_cmd *cmd;
	struct list_head clist_local;
	union megasas_evt_class_locale class_locale;
	unsigned long flags;
	u32 seq_num;

	INIT_LIST_HEAD(&clist_local);
	spin_lock_irqsave(&instance->hba_lock, flags);
	list_splice_init(&instance->internal_reset_pending_q, &clist_local);
	spin_unlock_irqrestore(&instance->hba_lock, flags);

	while (!list_empty(&clist_local)) {
		cmd = list_entry((&clist_local)->next,
					struct megasas_cmd, list);
		list_del_init(&cmd->list);

		if (cmd->sync_cmd || cmd->scmd) {
			dev_notice(&instance->pdev->dev, "command %p, %p:%d"
				"detected to be pending while HBA reset\n",
					cmd, cmd->scmd, cmd->sync_cmd);

			cmd->retry_for_fw_reset++;

			if (cmd->retry_for_fw_reset == 3) {
				dev_notice(&instance->pdev->dev, "cmd %p, %p:%d"
					"was tried multiple times during reset."
					"Shutting down the HBA\n",
					cmd, cmd->scmd, cmd->sync_cmd);
				instance->instancet->disable_intr(instance);
				atomic_set(&instance->fw_reset_no_pci_access, 1);
				megaraid_sas_kill_hba(instance);
				return;
			}
		}

		if (cmd->sync_cmd == 1) {
			if (cmd->scmd) {
				dev_notice(&instance->pdev->dev, "unexpected"
					"cmd attached to internal command!\n");
			}
			dev_notice(&instance->pdev->dev, "%p synchronous cmd"
						"on the internal reset queue,"
						"issue it again.\n", cmd);
			cmd->cmd_status_drv = MFI_STAT_INVALID_STATUS;
			instance->instancet->fire_cmd(instance,
							cmd->frame_phys_addr,
							0, instance->reg_set);
		} else if (cmd->scmd) {
			dev_notice(&instance->pdev->dev, "%p scsi cmd [%02x]"
			"detected on the internal queue, issue again.\n",
			cmd, cmd->scmd->cmnd[0]);

			atomic_inc(&instance->fw_outstanding);
			instance->instancet->fire_cmd(instance,
					cmd->frame_phys_addr,
					cmd->frame_count-1, instance->reg_set);
		} else {
			dev_notice(&instance->pdev->dev, "%p unexpected cmd on the"
				"internal reset defer list while re-issue!!\n",
				cmd);
		}
	}

	if (instance->aen_cmd) {
		dev_notice(&instance->pdev->dev, "aen_cmd in def process\n");
		megasas_return_cmd(instance, instance->aen_cmd);

		instance->aen_cmd = NULL;
	}

	/*
	 * Initiate AEN (Asynchronous Event Notification)
	 */
	seq_num = instance->last_seq_num;
	class_locale.members.reserved = 0;
	class_locale.members.locale = MR_EVT_LOCALE_ALL;
	class_locale.members.class = MR_EVT_CLASS_DEBUG;

	megasas_register_aen(instance, seq_num, class_locale.word);
}

/**
 * Move the internal reset pending commands to a deferred queue.
 *
 * We move the commands pending at internal reset time to a
 * pending queue. This queue would be flushed after successful
 * completion of the internal reset sequence. if the internal reset
 * did not complete in time, the kernel reset handler would flush
 * these commands.
 **/
static void
megasas_internal_reset_defer_cmds(struct megasas_instance *instance)
{
	struct megasas_cmd *cmd;
	int i;
	u16 max_cmd = instance->max_fw_cmds;
	u32 defer_index;
	unsigned long flags;

	defer_index = 0;
	spin_lock_irqsave(&instance->mfi_pool_lock, flags);
	for (i = 0; i < max_cmd; i++) {
		cmd = instance->cmd_list[i];
		if (cmd->sync_cmd == 1 || cmd->scmd) {
			dev_notice(&instance->pdev->dev, "moving cmd[%d]:%p:%d:%p"
					"on the defer queue as internal\n",
				defer_index, cmd, cmd->sync_cmd, cmd->scmd);

			if (!list_empty(&cmd->list)) {
				dev_notice(&instance->pdev->dev, "ERROR while"
					" moving this cmd:%p, %d %p, it was"
					"discovered on some list?\n",
					cmd, cmd->sync_cmd, cmd->scmd);

				list_del_init(&cmd->list);
			}
			defer_index++;
			list_add_tail(&cmd->list,
				&instance->internal_reset_pending_q);
		}
	}
	spin_unlock_irqrestore(&instance->mfi_pool_lock, flags);
}


static void
process_fw_state_change_wq(struct work_struct *work)
{
	struct megasas_instance *instance =
		container_of(work, struct megasas_instance, work_init);
	u32 wait;
	unsigned long flags;

    if (atomic_read(&instance->adprecovery) != MEGASAS_ADPRESET_SM_INFAULT) {
		dev_notice(&instance->pdev->dev, "error, recovery st %x\n",
				atomic_read(&instance->adprecovery));
		return ;
	}

	if (atomic_read(&instance->adprecovery) == MEGASAS_ADPRESET_SM_INFAULT) {
		dev_notice(&instance->pdev->dev, "FW detected to be in fault"
					"state, restarting it...\n");

		instance->instancet->disable_intr(instance);
		atomic_set(&instance->fw_outstanding, 0);

		atomic_set(&instance->fw_reset_no_pci_access, 1);
		instance->instancet->adp_reset(instance, instance->reg_set);
		atomic_set(&instance->fw_reset_no_pci_access, 0);

		dev_notice(&instance->pdev->dev, "FW restarted successfully,"
					"initiating next stage...\n");

		dev_notice(&instance->pdev->dev, "HBA recovery state machine,"
					"state 2 starting...\n");

		/* waiting for about 20 second before start the second init */
		for (wait = 0; wait < 30; wait++) {
			msleep(1000);
		}

		if (megasas_transition_to_ready(instance, 1)) {
			dev_notice(&instance->pdev->dev, "adapter not ready\n");

			atomic_set(&instance->fw_reset_no_pci_access, 1);
			megaraid_sas_kill_hba(instance);
			return ;
		}

		if ((instance->pdev->device == PCI_DEVICE_ID_LSI_SAS1064R) ||
			(instance->pdev->device == PCI_DEVICE_ID_DELL_PERC5) ||
			(instance->pdev->device == PCI_DEVICE_ID_LSI_VERDE_ZCR)
			) {
			*instance->consumer = *instance->producer;
		} else {
			*instance->consumer = 0;
			*instance->producer = 0;
		}

		megasas_issue_init_mfi(instance);

		spin_lock_irqsave(&instance->hba_lock, flags);
		atomic_set(&instance->adprecovery, MEGASAS_HBA_OPERATIONAL);
		spin_unlock_irqrestore(&instance->hba_lock, flags);
		instance->instancet->enable_intr(instance);

		megasas_issue_pending_cmds_again(instance);
		instance->issuepend_done = 1;
	}
}

/**
 * megasas_deplete_reply_queue -	Processes all completed commands
 * @instance:				Adapter soft state
 * @alt_status:				Alternate status to be returned to
 *					SCSI mid-layer instead of the status
 *					returned by the FW
 * Note: this must be called with hba lock held
 */
static int
megasas_deplete_reply_queue(struct megasas_instance *instance,
					u8 alt_status)
{
	u32 mfiStatus;
	u32 fw_state;

	if ((mfiStatus = instance->instancet->check_reset(instance,
					instance->reg_set)) == 1) {
		return IRQ_HANDLED;
	}

	if ((mfiStatus = instance->instancet->clear_intr(
						instance->reg_set)
						) == 0) {
		/* Hardware may not set outbound_intr_status in MSI-X mode */
		if (!instance->msix_vectors)
			return IRQ_NONE;
	}

	instance->mfiStatus = mfiStatus;

	if ((mfiStatus & MFI_INTR_FLAG_FIRMWARE_STATE_CHANGE)) {
		fw_state = instance->instancet->read_fw_status_reg(
				instance->reg_set) & MFI_STATE_MASK;

		if (fw_state != MFI_STATE_FAULT) {
			dev_notice(&instance->pdev->dev, "fw state:%x\n",
						fw_state);
		}

		if ((fw_state == MFI_STATE_FAULT) &&
				(instance->disableOnlineCtrlReset == 0)) {
			dev_notice(&instance->pdev->dev, "wait adp restart\n");

			if ((instance->pdev->device ==
					PCI_DEVICE_ID_LSI_SAS1064R) ||
				(instance->pdev->device ==
					PCI_DEVICE_ID_DELL_PERC5) ||
				(instance->pdev->device ==
					PCI_DEVICE_ID_LSI_VERDE_ZCR)) {

				*instance->consumer =
					cpu_to_le32(MEGASAS_ADPRESET_INPROG_SIGN);
			}


			instance->instancet->disable_intr(instance);
			atomic_set(&instance->adprecovery, MEGASAS_ADPRESET_SM_INFAULT);
			instance->issuepend_done = 0;

			atomic_set(&instance->fw_outstanding, 0);
			megasas_internal_reset_defer_cmds(instance);

			dev_notice(&instance->pdev->dev, "fwState=%x, stage:%d\n",
					fw_state, atomic_read(&instance->adprecovery));

			schedule_work(&instance->work_init);
			return IRQ_HANDLED;

		} else {
			dev_notice(&instance->pdev->dev, "fwstate:%x, dis_OCR=%x\n",
				fw_state, instance->disableOnlineCtrlReset);
		}
	}

	tasklet_schedule(&instance->isr_tasklet);
	return IRQ_HANDLED;
}
/**
 * megasas_isr - isr entry point
 */
static irqreturn_t megasas_isr(int irq, void *devp)
{
	struct megasas_irq_context *irq_context = devp;
	struct megasas_instance *instance = irq_context->instance;
	unsigned long flags;
	irqreturn_t rc;

	if (atomic_read(&instance->fw_reset_no_pci_access))
		return IRQ_HANDLED;

	spin_lock_irqsave(&instance->hba_lock, flags);
	rc = megasas_deplete_reply_queue(instance, DID_OK);
	spin_unlock_irqrestore(&instance->hba_lock, flags);

	return rc;
}

/**
 * megasas_transition_to_ready -	Move the FW to READY state
 * @instance:				Adapter soft state
 *
 * During the initialization, FW passes can potentially be in any one of
 * several possible states. If the FW in operational, waiting-for-handshake
 * states, driver must take steps to bring it to ready state. Otherwise, it
 * has to wait for the ready state.
 */
int
megasas_transition_to_ready(struct megasas_instance *instance, int ocr)
{
	int i;
	u8 max_wait;
	u32 fw_state;
	u32 cur_state;
	u32 abs_state, curr_abs_state;

	abs_state = instance->instancet->read_fw_status_reg(instance->reg_set);
	fw_state = abs_state & MFI_STATE_MASK;

	if (fw_state != MFI_STATE_READY)
		dev_info(&instance->pdev->dev, "Waiting for FW to come to ready"
		       " state\n");

	while (fw_state != MFI_STATE_READY) {

		switch (fw_state) {

		case MFI_STATE_FAULT:
			dev_printk(KERN_DEBUG, &instance->pdev->dev, "FW in FAULT state!!\n");
			if (ocr) {
				max_wait = MEGASAS_RESET_WAIT_TIME;
				cur_state = MFI_STATE_FAULT;
				break;
			} else
				return -ENODEV;

		case MFI_STATE_WAIT_HANDSHAKE:
			/*
			 * Set the CLR bit in inbound doorbell
			 */
			if ((instance->pdev->device ==
				PCI_DEVICE_ID_LSI_SAS0073SKINNY) ||
				(instance->pdev->device ==
				 PCI_DEVICE_ID_LSI_SAS0071SKINNY) ||
				(instance->adapter_type != MFI_SERIES))
				writel(
				  MFI_INIT_CLEAR_HANDSHAKE|MFI_INIT_HOTPLUG,
				  &instance->reg_set->doorbell);
			else
				writel(
				    MFI_INIT_CLEAR_HANDSHAKE|MFI_INIT_HOTPLUG,
					&instance->reg_set->inbound_doorbell);

			max_wait = MEGASAS_RESET_WAIT_TIME;
			cur_state = MFI_STATE_WAIT_HANDSHAKE;
			break;

		case MFI_STATE_BOOT_MESSAGE_PENDING:
			if ((instance->pdev->device ==
			     PCI_DEVICE_ID_LSI_SAS0073SKINNY) ||
				(instance->pdev->device ==
				 PCI_DEVICE_ID_LSI_SAS0071SKINNY) ||
				(instance->adapter_type != MFI_SERIES))
				writel(MFI_INIT_HOTPLUG,
				       &instance->reg_set->doorbell);
			else
				writel(MFI_INIT_HOTPLUG,
					&instance->reg_set->inbound_doorbell);

			max_wait = MEGASAS_RESET_WAIT_TIME;
			cur_state = MFI_STATE_BOOT_MESSAGE_PENDING;
			break;

		case MFI_STATE_OPERATIONAL:
			/*
			 * Bring it to READY state; assuming max wait 10 secs
			 */
			instance->instancet->disable_intr(instance);
			if ((instance->pdev->device ==
				PCI_DEVICE_ID_LSI_SAS0073SKINNY) ||
				(instance->pdev->device ==
				PCI_DEVICE_ID_LSI_SAS0071SKINNY)  ||
				(instance->adapter_type != MFI_SERIES)) {
				writel(MFI_RESET_FLAGS,
					&instance->reg_set->doorbell);

				if (instance->adapter_type != MFI_SERIES) {
					for (i = 0; i < (10 * 1000); i += 20) {
						if (readl(
							    &instance->
							    reg_set->
							    doorbell) & 1)
							msleep(20);
						else
							break;
					}
				}
			} else
				writel(MFI_RESET_FLAGS,
					&instance->reg_set->inbound_doorbell);

			max_wait = MEGASAS_RESET_WAIT_TIME;
			cur_state = MFI_STATE_OPERATIONAL;
			break;

		case MFI_STATE_UNDEFINED:
			/*
			 * This state should not last for more than 2 seconds
			 */
			max_wait = MEGASAS_RESET_WAIT_TIME;
			cur_state = MFI_STATE_UNDEFINED;
			break;

		case MFI_STATE_BB_INIT:
			max_wait = MEGASAS_RESET_WAIT_TIME;
			cur_state = MFI_STATE_BB_INIT;
			break;

		case MFI_STATE_FW_INIT:
			max_wait = MEGASAS_RESET_WAIT_TIME;
			cur_state = MFI_STATE_FW_INIT;
			break;

		case MFI_STATE_FW_INIT_2:
			max_wait = MEGASAS_RESET_WAIT_TIME;
			cur_state = MFI_STATE_FW_INIT_2;
			break;

		case MFI_STATE_DEVICE_SCAN:
			max_wait = MEGASAS_RESET_WAIT_TIME;
			cur_state = MFI_STATE_DEVICE_SCAN;
			break;

		case MFI_STATE_FLUSH_CACHE:
			max_wait = MEGASAS_RESET_WAIT_TIME;
			cur_state = MFI_STATE_FLUSH_CACHE;
			break;

		default:
			dev_printk(KERN_DEBUG, &instance->pdev->dev, "Unknown state 0x%x\n",
			       fw_state);
			return -ENODEV;
		}

		/*
		 * The cur_state should not last for more than max_wait secs
		 */
		for (i = 0; i < max_wait * 50; i++) {
			curr_abs_state = instance->instancet->
				read_fw_status_reg(instance->reg_set);

			if (abs_state == curr_abs_state) {
				msleep(20);
			} else
				break;
		}

		/*
		 * Return error if fw_state hasn't changed after max_wait
		 */
		if (curr_abs_state == abs_state) {
			dev_printk(KERN_DEBUG, &instance->pdev->dev, "FW state [%d] hasn't changed "
			       "in %d secs\n", fw_state, max_wait);
			return -ENODEV;
		}

		abs_state = curr_abs_state;
		fw_state = curr_abs_state & MFI_STATE_MASK;
	}
	dev_info(&instance->pdev->dev, "FW now in Ready state\n");

	return 0;
}

/**
 * megasas_teardown_frame_pool -	Destroy the cmd frame DMA pool
 * @instance:				Adapter soft state
 */
static void megasas_teardown_frame_pool(struct megasas_instance *instance)
{
	int i;
	u16 max_cmd = instance->max_mfi_cmds;
	struct megasas_cmd *cmd;

	if (!instance->frame_dma_pool)
		return;

	/*
	 * Return all frames to pool
	 */
	for (i = 0; i < max_cmd; i++) {

		cmd = instance->cmd_list[i];

		if (cmd->frame)
			dma_pool_free(instance->frame_dma_pool, cmd->frame,
				      cmd->frame_phys_addr);

		if (cmd->sense)
			dma_pool_free(instance->sense_dma_pool, cmd->sense,
				      cmd->sense_phys_addr);
	}

	/*
	 * Now destroy the pool itself
	 */
	dma_pool_destroy(instance->frame_dma_pool);
	dma_pool_destroy(instance->sense_dma_pool);

	instance->frame_dma_pool = NULL;
	instance->sense_dma_pool = NULL;
}

/**
 * megasas_create_frame_pool -	Creates DMA pool for cmd frames
 * @instance:			Adapter soft state
 *
 * Each command packet has an embedded DMA memory buffer that is used for
 * filling MFI frame and the SG list that immediately follows the frame. This
 * function creates those DMA memory buffers for each command packet by using
 * PCI pool facility.
 */
static int megasas_create_frame_pool(struct megasas_instance *instance)
{
	int i;
	u16 max_cmd;
	u32 sge_sz;
	u32 frame_count;
	struct megasas_cmd *cmd;

	max_cmd = instance->max_mfi_cmds;

	/*
	 * Size of our frame is 64 bytes for MFI frame, followed by max SG
	 * elements and finally SCSI_SENSE_BUFFERSIZE bytes for sense buffer
	 */
	sge_sz = (IS_DMA64) ? sizeof(struct megasas_sge64) :
	    sizeof(struct megasas_sge32);

	if (instance->flag_ieee)
		sge_sz = sizeof(struct megasas_sge_skinny);

	/*
	 * For MFI controllers.
	 * max_num_sge = 60
	 * max_sge_sz  = 16 byte (sizeof megasas_sge_skinny)
	 * Total 960 byte (15 MFI frame of 64 byte)
	 *
	 * Fusion adapter require only 3 extra frame.
	 * max_num_sge = 16 (defined as MAX_IOCTL_SGE)
	 * max_sge_sz  = 12 byte (sizeof  megasas_sge64)
	 * Total 192 byte (3 MFI frame of 64 byte)
	 */
	frame_count = (instance->adapter_type == MFI_SERIES) ?
			(15 + 1) : (3 + 1);
	instance->mfi_frame_size = MEGAMFI_FRAME_SIZE * frame_count;
	/*
	 * Use DMA pool facility provided by PCI layer
	 */
	instance->frame_dma_pool = dma_pool_create("megasas frame pool",
					&instance->pdev->dev,
					instance->mfi_frame_size, 256, 0);

	if (!instance->frame_dma_pool) {
		dev_printk(KERN_DEBUG, &instance->pdev->dev, "failed to setup frame pool\n");
		return -ENOMEM;
	}

	instance->sense_dma_pool = dma_pool_create("megasas sense pool",
						   &instance->pdev->dev, 128,
						   4, 0);

	if (!instance->sense_dma_pool) {
		dev_printk(KERN_DEBUG, &instance->pdev->dev, "failed to setup sense pool\n");

		dma_pool_destroy(instance->frame_dma_pool);
		instance->frame_dma_pool = NULL;

		return -ENOMEM;
	}

	/*
	 * Allocate and attach a frame to each of the commands in cmd_list.
	 * By making cmd->index as the context instead of the &cmd, we can
	 * always use 32bit context regardless of the architecture
	 */
	for (i = 0; i < max_cmd; i++) {

		cmd = instance->cmd_list[i];

		cmd->frame = dma_pool_alloc(instance->frame_dma_pool,
					    GFP_KERNEL, &cmd->frame_phys_addr);

		cmd->sense = dma_pool_alloc(instance->sense_dma_pool,
					    GFP_KERNEL, &cmd->sense_phys_addr);

		/*
		 * megasas_teardown_frame_pool() takes care of freeing
		 * whatever has been allocated
		 */
		if (!cmd->frame || !cmd->sense) {
			dev_printk(KERN_DEBUG, &instance->pdev->dev, "dma_pool_alloc failed\n");
			megasas_teardown_frame_pool(instance);
			return -ENOMEM;
		}

		memset(cmd->frame, 0, instance->mfi_frame_size);
		cmd->frame->io.context = cpu_to_le32(cmd->index);
		cmd->frame->io.pad_0 = 0;
		if ((instance->adapter_type == MFI_SERIES) && reset_devices)
			cmd->frame->hdr.cmd = MFI_CMD_INVALID;
	}

	return 0;
}

/**
 * megasas_free_cmds -	Free all the cmds in the free cmd pool
 * @instance:		Adapter soft state
 */
void megasas_free_cmds(struct megasas_instance *instance)
{
	int i;

	/* First free the MFI frame pool */
	megasas_teardown_frame_pool(instance);

	/* Free all the commands in the cmd_list */
	for (i = 0; i < instance->max_mfi_cmds; i++)

		kfree(instance->cmd_list[i]);

	/* Free the cmd_list buffer itself */
	kfree(instance->cmd_list);
	instance->cmd_list = NULL;

	INIT_LIST_HEAD(&instance->cmd_pool);
}

/**
 * megasas_alloc_cmds -	Allocates the command packets
 * @instance:		Adapter soft state
 *
 * Each command that is issued to the FW, whether IO commands from the OS or
 * internal commands like IOCTLs, are wrapped in local data structure called
 * megasas_cmd. The frame embedded in this megasas_cmd is actually issued to
 * the FW.
 *
 * Each frame has a 32-bit field called context (tag). This context is used
 * to get back the megasas_cmd from the frame when a frame gets completed in
 * the ISR. Typically the address of the megasas_cmd itself would be used as
 * the context. But we wanted to keep the differences between 32 and 64 bit
 * systems to the mininum. We always use 32 bit integers for the context. In
 * this driver, the 32 bit values are the indices into an array cmd_list.
 * This array is used only to look up the megasas_cmd given the context. The
 * free commands themselves are maintained in a linked list called cmd_pool.
 */
int megasas_alloc_cmds(struct megasas_instance *instance)
{
	int i;
	int j;
	u16 max_cmd;
	struct megasas_cmd *cmd;
	struct fusion_context *fusion;

	fusion = instance->ctrl_context;
	max_cmd = instance->max_mfi_cmds;

	/*
	 * instance->cmd_list is an array of struct megasas_cmd pointers.
	 * Allocate the dynamic array first and then allocate individual
	 * commands.
	 */
	instance->cmd_list = kcalloc(max_cmd, sizeof(struct megasas_cmd*), GFP_KERNEL);

	if (!instance->cmd_list) {
		dev_printk(KERN_DEBUG, &instance->pdev->dev, "out of memory\n");
		return -ENOMEM;
	}

	memset(instance->cmd_list, 0, sizeof(struct megasas_cmd *) *max_cmd);

	for (i = 0; i < max_cmd; i++) {
		instance->cmd_list[i] = kmalloc(sizeof(struct megasas_cmd),
						GFP_KERNEL);

		if (!instance->cmd_list[i]) {

			for (j = 0; j < i; j++)
				kfree(instance->cmd_list[j]);

			kfree(instance->cmd_list);
			instance->cmd_list = NULL;

			return -ENOMEM;
		}
	}

	for (i = 0; i < max_cmd; i++) {
		cmd = instance->cmd_list[i];
		memset(cmd, 0, sizeof(struct megasas_cmd));
		cmd->index = i;
		cmd->scmd = NULL;
		cmd->instance = instance;

		list_add_tail(&cmd->list, &instance->cmd_pool);
	}

	/*
	 * Create a frame pool and assign one frame to each cmd
	 */
	if (megasas_create_frame_pool(instance)) {
		dev_printk(KERN_DEBUG, &instance->pdev->dev, "Error creating frame DMA pool\n");
		megasas_free_cmds(instance);
		return -ENOMEM;
	}

	return 0;
}

/*
 * dcmd_timeout_ocr_possible -	Check if OCR is possible based on Driver/FW state.
 * @instance:				Adapter soft state
 *
 * Return 0 for only Fusion adapter, if driver load/unload is not in progress
 * or FW is not under OCR.
 */
inline int
dcmd_timeout_ocr_possible(struct megasas_instance *instance) {

	if (instance->adapter_type == MFI_SERIES)
		return KILL_ADAPTER;
	else if (instance->unload ||
			test_bit(MEGASAS_FUSION_OCR_NOT_POSSIBLE,
				 &instance->reset_flags))
		return IGNORE_TIMEOUT;
	else
		return INITIATE_OCR;
}

static void
megasas_get_pd_info(struct megasas_instance *instance, struct scsi_device *sdev)
{
	int ret;
	struct megasas_cmd *cmd;
	struct megasas_dcmd_frame *dcmd;

	struct MR_PRIV_DEVICE *mr_device_priv_data;
	u16 device_id = 0;

	device_id = (sdev->channel * MEGASAS_MAX_DEV_PER_CHANNEL) + sdev->id;
	cmd = megasas_get_cmd(instance);

	if (!cmd) {
		dev_err(&instance->pdev->dev, "Failed to get cmd %s\n", __func__);
		return;
	}

	dcmd = &cmd->frame->dcmd;

	memset(instance->pd_info, 0, sizeof(*instance->pd_info));
	memset(dcmd->mbox.b, 0, MFI_MBOX_SIZE);

	dcmd->mbox.s[0] = cpu_to_le16(device_id);
	dcmd->cmd = MFI_CMD_DCMD;
	dcmd->cmd_status = 0xFF;
	dcmd->sge_count = 1;
	dcmd->flags = cpu_to_le16(MFI_FRAME_DIR_READ);
	dcmd->timeout = 0;
	dcmd->pad_0 = 0;
	dcmd->data_xfer_len = cpu_to_le32(sizeof(struct MR_PD_INFO));
	dcmd->opcode = cpu_to_le32(MR_DCMD_PD_GET_INFO);
	dcmd->sgl.sge32[0].phys_addr = cpu_to_le32(instance->pd_info_h);
	dcmd->sgl.sge32[0].length = cpu_to_le32(sizeof(struct MR_PD_INFO));

	if ((instance->adapter_type != MFI_SERIES) &&
	    !instance->mask_interrupts)
		ret = megasas_issue_blocked_cmd(instance, cmd, MFI_IO_TIMEOUT_SECS);
	else
		ret = megasas_issue_polled(instance, cmd);

	switch (ret) {
	case DCMD_SUCCESS:
		mr_device_priv_data = sdev->hostdata;
		le16_to_cpus((u16 *)&instance->pd_info->state.ddf.pdType);
		mr_device_priv_data->interface_type =
				instance->pd_info->state.ddf.pdType.intf;
		break;

	case DCMD_TIMEOUT:

		switch (dcmd_timeout_ocr_possible(instance)) {
		case INITIATE_OCR:
			cmd->flags |= DRV_DCMD_SKIP_REFIRE;
			megasas_reset_fusion(instance->host,
				MFI_IO_TIMEOUT_OCR);
			break;
		case KILL_ADAPTER:
			megaraid_sas_kill_hba(instance);
			break;
		case IGNORE_TIMEOUT:
			dev_info(&instance->pdev->dev, "Ignore DCMD timeout: %s %d\n",
				__func__, __LINE__);
			break;
		}

		break;
	}

	if (ret != DCMD_TIMEOUT)
		megasas_return_cmd(instance, cmd);

	return;
}
/*
 * megasas_get_pd_list_info -	Returns FW's pd_list structure
 * @instance:				Adapter soft state
 * @pd_list:				pd_list structure
 *
 * Issues an internal command (DCMD) to get the FW's controller PD
 * list structure.  This information is mainly used to find out SYSTEM
 * supported by the FW.
 */
static int
megasas_get_pd_list(struct megasas_instance *instance)
{
	int ret = 0, pd_index = 0;
	struct megasas_cmd *cmd;
	struct megasas_dcmd_frame *dcmd;
	struct MR_PD_LIST *ci;
	struct MR_PD_ADDRESS *pd_addr;
	dma_addr_t ci_h = 0;

	if (instance->pd_list_not_supported) {
		dev_info(&instance->pdev->dev, "MR_DCMD_PD_LIST_QUERY "
		"not supported by firmware\n");
		return ret;
	}

	cmd = megasas_get_cmd(instance);

	if (!cmd) {
		dev_printk(KERN_DEBUG, &instance->pdev->dev, "(get_pd_list): Failed to get cmd\n");
		return -ENOMEM;
	}

	dcmd = &cmd->frame->dcmd;

	ci = pci_alloc_consistent(instance->pdev,
		  MEGASAS_MAX_PD * sizeof(struct MR_PD_LIST), &ci_h);

	if (!ci) {
		dev_printk(KERN_DEBUG, &instance->pdev->dev, "Failed to alloc mem for pd_list\n");
		megasas_return_cmd(instance, cmd);
		return -ENOMEM;
	}

	memset(ci, 0, sizeof(*ci));
	memset(dcmd->mbox.b, 0, MFI_MBOX_SIZE);

	dcmd->mbox.b[0] = MR_PD_QUERY_TYPE_EXPOSED_TO_HOST;
	dcmd->mbox.b[1] = 0;
	dcmd->cmd = MFI_CMD_DCMD;
	dcmd->cmd_status = MFI_STAT_INVALID_STATUS;
	dcmd->sge_count = 1;
	dcmd->flags = cpu_to_le16(MFI_FRAME_DIR_READ);
	dcmd->timeout = 0;
	dcmd->pad_0 = 0;
	dcmd->data_xfer_len = cpu_to_le32(MEGASAS_MAX_PD * sizeof(struct MR_PD_LIST));
	dcmd->opcode = cpu_to_le32(MR_DCMD_PD_LIST_QUERY);
	dcmd->sgl.sge32[0].phys_addr = cpu_to_le32(ci_h);
	dcmd->sgl.sge32[0].length = cpu_to_le32(MEGASAS_MAX_PD * sizeof(struct MR_PD_LIST));

	if ((instance->adapter_type != MFI_SERIES) &&
	    !instance->mask_interrupts)
		ret = megasas_issue_blocked_cmd(instance, cmd,
			MFI_IO_TIMEOUT_SECS);
	else
		ret = megasas_issue_polled(instance, cmd);

	switch (ret) {
	case DCMD_FAILED:
		dev_info(&instance->pdev->dev, "MR_DCMD_PD_LIST_QUERY "
			"failed/not supported by firmware\n");

		if (instance->adapter_type != MFI_SERIES)
			megaraid_sas_kill_hba(instance);
		else
			instance->pd_list_not_supported = 1;
		break;
	case DCMD_TIMEOUT:

		switch (dcmd_timeout_ocr_possible(instance)) {
		case INITIATE_OCR:
			cmd->flags |= DRV_DCMD_SKIP_REFIRE;
			/*
			 * DCMD failed from AEN path.
			 * AEN path already hold reset_mutex to avoid PCI access
			 * while OCR is in progress.
			 */
			mutex_unlock(&instance->reset_mutex);
			megasas_reset_fusion(instance->host,
						MFI_IO_TIMEOUT_OCR);
			mutex_lock(&instance->reset_mutex);
			break;
		case KILL_ADAPTER:
			megaraid_sas_kill_hba(instance);
			break;
		case IGNORE_TIMEOUT:
			dev_info(&instance->pdev->dev, "Ignore DCMD timeout: %s %d \n",
				__func__, __LINE__);
			break;
		}

		break;

	case DCMD_SUCCESS:
		pd_addr = ci->addr;

		if ((le32_to_cpu(ci->count) >
			(MEGASAS_MAX_PD_CHANNELS * MEGASAS_MAX_DEV_PER_CHANNEL)))
			break;

		memset(instance->local_pd_list, 0,
				MEGASAS_MAX_PD * sizeof(struct megasas_pd_list));

		for (pd_index = 0; pd_index < le32_to_cpu(ci->count); pd_index++) {
			instance->local_pd_list[le16_to_cpu(pd_addr->deviceId)].tid	=
					le16_to_cpu(pd_addr->deviceId);
			instance->local_pd_list[le16_to_cpu(pd_addr->deviceId)].driveType	=
					pd_addr->scsiDevType;
			instance->local_pd_list[le16_to_cpu(pd_addr->deviceId)].driveState	=
					MR_PD_STATE_SYSTEM;
			pd_addr++;
		}

		memcpy(instance->pd_list, instance->local_pd_list,
			sizeof(instance->pd_list));
		break;

	}

	pci_free_consistent(instance->pdev,
				MEGASAS_MAX_PD * sizeof(struct MR_PD_LIST),
				ci, ci_h);

	if (ret != DCMD_TIMEOUT)
		megasas_return_cmd(instance, cmd);

	return ret;
}

/*
 * megasas_get_ld_list_info -	Returns FW's ld_list structure
 * @instance:				Adapter soft state
 * @ld_list:				ld_list structure
 *
 * Issues an internal command (DCMD) to get the FW's controller PD
 * list structure.  This information is mainly used to find out SYSTEM
 * supported by the FW.
 */
static int
megasas_get_ld_list(struct megasas_instance *instance)
{
	int ret = 0, ld_index = 0, ids = 0;
	struct megasas_cmd *cmd;
	struct megasas_dcmd_frame *dcmd;
	struct MR_LD_LIST *ci;
	dma_addr_t ci_h = 0;
	u32 ld_count;

	cmd = megasas_get_cmd(instance);

	if (!cmd) {
		dev_printk(KERN_DEBUG, &instance->pdev->dev, "megasas_get_ld_list: Failed to get cmd\n");
		return -ENOMEM;
	}

	dcmd = &cmd->frame->dcmd;

	ci = pci_alloc_consistent(instance->pdev,
				sizeof(struct MR_LD_LIST),
				&ci_h);

	if (!ci) {
		dev_printk(KERN_DEBUG, &instance->pdev->dev, "Failed to alloc mem in get_ld_list\n");
		megasas_return_cmd(instance, cmd);
		return -ENOMEM;
	}

	memset(ci, 0, sizeof(*ci));
	memset(dcmd->mbox.b, 0, MFI_MBOX_SIZE);

	if (instance->supportmax256vd)
		dcmd->mbox.b[0] = 1;
	dcmd->cmd = MFI_CMD_DCMD;
	dcmd->cmd_status = MFI_STAT_INVALID_STATUS;
	dcmd->sge_count = 1;
	dcmd->flags = cpu_to_le16(MFI_FRAME_DIR_READ);
	dcmd->timeout = 0;
	dcmd->data_xfer_len = cpu_to_le32(sizeof(struct MR_LD_LIST));
	dcmd->opcode = cpu_to_le32(MR_DCMD_LD_GET_LIST);
	dcmd->sgl.sge32[0].phys_addr = cpu_to_le32(ci_h);
	dcmd->sgl.sge32[0].length = cpu_to_le32(sizeof(struct MR_LD_LIST));
	dcmd->pad_0  = 0;

	if ((instance->adapter_type != MFI_SERIES) &&
	    !instance->mask_interrupts)
		ret = megasas_issue_blocked_cmd(instance, cmd,
			MFI_IO_TIMEOUT_SECS);
	else
		ret = megasas_issue_polled(instance, cmd);

	ld_count = le32_to_cpu(ci->ldCount);

	switch (ret) {
	case DCMD_FAILED:
		megaraid_sas_kill_hba(instance);
		break;
	case DCMD_TIMEOUT:

		switch (dcmd_timeout_ocr_possible(instance)) {
		case INITIATE_OCR:
			cmd->flags |= DRV_DCMD_SKIP_REFIRE;
			/*
			 * DCMD failed from AEN path.
			 * AEN path already hold reset_mutex to avoid PCI access
			 * while OCR is in progress.
			 */
			mutex_unlock(&instance->reset_mutex);
			megasas_reset_fusion(instance->host,
						MFI_IO_TIMEOUT_OCR);
			mutex_lock(&instance->reset_mutex);
			break;
		case KILL_ADAPTER:
			megaraid_sas_kill_hba(instance);
			break;
		case IGNORE_TIMEOUT:
			dev_info(&instance->pdev->dev, "Ignore DCMD timeout: %s %d\n",
				__func__, __LINE__);
			break;
		}

		break;

	case DCMD_SUCCESS:
		if (ld_count > instance->fw_supported_vd_count)
			break;

		memset(instance->ld_ids, 0xff, MAX_LOGICAL_DRIVES_EXT);

		for (ld_index = 0; ld_index < ld_count; ld_index++) {
			if (ci->ldList[ld_index].state != 0) {
				ids = ci->ldList[ld_index].ref.targetId;
				instance->ld_ids[ids] = ci->ldList[ld_index].ref.targetId;
			}
		}

		break;
	}

	info(&instance->pdev->dev, "Ignore DCMD ti_ance-> (ci->ldList[laC	break;
	}

	info(&instance->pdev->dev, "Ignore DCMD ti_ance-> (ci->ldList[laC	break;
	}

	info(&instance->pdev->dev, "Ignore DCMD ti_ance-> (ci->ldList[laC	break;
	}

	info(&instance->pdev->dev, "Ignore DCMD ti_ance-> (ci->ldList[laC	break;
	}

	info(&insta_Hreak;
	}

	info(&insta_Hreak;
	}

	info(&insta_Hreak;
	}

	info(&insta_Hreak;
	}

	info(&insta_Hreak;
	}

	info(&insta_Hreak;
	}

	info(&insta_Hre|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>cv");

		if (nareak;
	}

	info(&instance->pdev->dev, "Ignore DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore DCMD ti_anceHv,|6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>c<v, "FW state [%d] hasn't changed "
			       "in %d secs\n", fw_state, max_wait);
			return -ENODEV;
		}

		abs_state = curr_abs_state;
		fw_stateHnsta_Hre|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>cv");

		if (nareak;
	}

	info(&instance->pdev->dev, "Ignore DCMD ti_anceHv, "lT, "Ignore DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore (DCMD) to get the FW's c\d(s used>w(KERNC)DCe DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore (DCMD) to get the FW's c\d(s used>w(KERNC)DCe DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore (DCMD) to get the FW's c\d(s used>w(KERNC)DCe DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore (DCMD) to get the FWF>FWF>FWF>FWF>FWF>FWF>FWF>FWF>FWF>FWF>FWF>FWF>FWF>FWF>FWF>c<OUT:
			dev_info(&instance->pdev->dev, "Ignore DCMD timeout: %s %d\n",
				__func__, __LINE__);
			break;
		}

		break;

	case DCMD_SUCCESS:
		if  getsF>yce-C	int i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignore (DCMD) to get the FW's c\d(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignore (DCMD) to get the FW's c\d(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignore (DCMD) to get the FW's c\d(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignore (DCMD) to get the FW's c\|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>cs&insCisf cpu_ii-___LINEbreak;
		}

		|8pu_ii-___LIF>yce-C	int i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignor3 (DCMD) to get the FW's c\d(s used>w(KERNC)Dfd i;
	u8 max_A cgHf>|Hf>|Hf>cs&insCisf cpu_ii-___LINEb9>/DReturn 0 for s c\d(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	i;
	u8 max_A cgHf>|Hf>|Hf>cs&insCisf cpu_iie>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>c {
			dev_notice(&9BlFW's c\|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|HfvOF>v .f>		fw_state, instance->disableOnlineCtrlReset);
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>linc
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>linc
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>linc
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>linc
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>lineCtrlR IGNORE_TIMEOUHrlR IGNORE_TIMEOUHrlR IGNORE_TIMEOUHrlR IGNORE_TIMEOUHrlR IGNORE_TIMEOUHrlR IGNORE_TIMEOUHrlR IGNORE_TIMEOUHrlR IGNORE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIMR IGN|6OF>wss
			 *cS)CMEOUHrlR IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIMR IGN|6OF>wss
			 *cS)CMEOUHrlR IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIMR IGN|6OF>wss
			 *cS)CMEOUHrlR IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE_TIvF>hHe3bp6rn -ENODEV;
		}

		abs_sWF>FWF>FWF>FW5l;
		}
	}
");
		 IGNORE_TExRE_nly VD
f (!instanc;
		: C max_wait's(!instanc
he pool itself
	 */
	dm	abs_sWF>FWF>FWF>FW5lpdev->dev, "ERROR while"
					" moving this cmax_cmd; i++) {
		instancore Dte iura_mape DMA 0nce->cmd_list[i] = kmalloc(sizeof(stru/instance->ce, stframe_dma_p	if (!i dummySI_SAS10_SERIES)
!>cmd_l*
 * megasas_c;
	u32 fw_state;
	nceHv, turn;[i] = kmalloc(ss_get_pu_ii-_Ostate\n"s3.state;
MaxExRLDstru/inBelhys_sas_cie\n"); e->pdCtrlR_cmd *cfu an aFW enh= km92 b_SERIES)
[i] = kmalloc(ss_get_n DrlUHr> a_pooc;
	u32 fw_state;
	nceHv, tu the F;
	u32 fw_drvTIMEOUdRE_TIMEOUHrl =ORE_TIMEOUdRE_TIinstanceEOUHrl>pd_list, instance->local_pd_; F;
	u32 fw_drvTIMEOUdRE_TpMEOUHrl =ORE_TIMEOUdREPTIinstanceEOUHrl>pd_list, instance->local_pd_; F;ait;
	u32 fw_state;
	nceHv, "e frame to eachE_TIMEOUdRE_TIMEOUHrl =ORci->ldList[laC	break;;frame to eachE_TIMEOUdRE_TpMEOUHrl =ORci-PHYSList[lEAD);Ignoance->hba_lme to eachE_TIMEOUdRE_TIMEOUHrl =ORci->ldList[laC	br;frame to eachE_TIMEOUdRE_TpMEOUHrl =ORci-PHYSList[lEAD);Ignoa
ol, cmd->frame,
				      cmd->f
TIME>		fw_st_LIN\IMEOUR IGNOR;
	u32 fw_state;
	nceHv, t? "ExRE_nly VD(240 VD)E>		fw_s"nsta	"L, "cy(64 VD).f>		fw_s"k_interrupts)
		retn DrHrlR map_cmdshba_lte iura_mape DMA pts)
		retn DrHrlR map_cmd *EOUHrlR R_MIN inPe FW'; /in64kASK;
	}cmd_lallurre i_mape DMA te iura_mape D;;
	}cmd_lal			GFape DMA te iura_mape D;;
ance->hba_l}cmd_lal		}GFape DMA UHrlR IGNORE_TIMEOFW_RAMR inP) +EOUHrlget the FW's c\d(s usSPAN inP) *EOUHrlUHrlR IGNORE_TIMEOUdRE_TIMEOUHrltat1megasa}cmd_lalnewGFape DMA UHrlR IGNORE_TIMEOFW_RAMR inPeak;
	}

	icmd_lal			GFape DMAE_TIMax(}cmd_lal		}GFape D, }cmd_lalnewGFape D>reset_mutex);
			brstate;
	nceHv, "Ign	}cmd_lallurre i_mape DMA }cmd_lalnewGFape D			break;
		}cmd_lallurre i_mape DMA }cmd_lal		}GFape Dgnoa
u/inirress_loiv cmd _DEValf
	ap_EADY) {

Valf
	ap led cm);
	sued tocmd_laldrvTmape DMA HrlR IGNORE_TIMEOineCRAMR inPeALLmegasas_cmd itself wo to grame_dma_	if (!cmd) {
		dev_pr, max_wait)eHv,|6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OFd] hasn't changed "
			       "in %d secs\n", fw_state, max_wait) -ENODEV;
nce->cmd_abs_state = curr_abs_state;
		fw_stateH(struaximumse 3ate!!ck tsta
->cmd_listHf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hfnce->pdev->dev, loc(ss_ge to alloc mem in get_ld_list\n");
		megasas_returnre DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore (DCMD) to get the F_anceHv, oc(ss_gee->pdev->dev, _anceHv, oc(ss_gee->oc(ss_geDCe DCMD ti_anceHv, "IgnOUHoc(ss_geest[i] = kmalloc(ss_geDC		dcmd->mbox.b[0] = 1;
	dcmd->cmd = MFI_CMD_DCMD;
	dcmd->cmd_status = MFI_STAT_INVALID_STATUS;
	d the FW's c\d(aer soft ssed>w(KERNC)DCe DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore (DCMD) to get the FWF>FWF>FWF>FWF>FWF>FWF>FWF>FWF>FWF>F_anceHv, oc(ss_ge_PD * sizeof(struct MR_PD_LIST));
	dcmd->opcode = cpu_to_le32(MR_DCMD_PD_LIST_QUERY);
	dcmd->sgl oc(_abs_reak;
		}

		break;

	case DCMD_SUCCESS:
		if  getsF>yce-C	int i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignore (DCMD) to get the FW's c\d(s used(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignore (DCMD) to get the FW's c\d(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignore	le16_to_cpus((u16 *)&instance->pd_info->state.ddf.pdType);
_anceHv, oc(ss_ge_|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>cs&insCisf cpu_CTRL->pd_info->state.ddf.pdType.intf;
		break;

	case DCMD_Twait;
	u32 fw_state;
	nceHv, "Ignor3 (DCMD) to get the FW's c\_anceHv, oc(ss_ge_|Hf>|Hf>|Hcmd(instance, c>|Hf>cs&insCisf cpu_ii-___LINEb9>/DReturn 0 for s c\d(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	i;
	u8 max_A cgHf>|Hf>|Hf>cs&insCisf cpu_iie>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>c {
			dev_notice(&9re DCMD timeout SY oc(ss_genceHv,et the FW's c\_anceHv, oc(ss_ge_|Hf>u/inSave{
		dev_stframe_dma__abs_state = cn
>lincCPU 			ito_lsrs.
	tat.

		}
	}
IMEOUHrlHE_		}
e D*)&loc(ss_get_pr staties.OnOffPr staties|Hf>uIMEOUHrlHE_		}
e D*)&loc(ss_get_pu_ii-_Ostate\n"s2|Hf>uIMEOUHrlHE_		}
e D*)&loc(ss_get_pu_ii-_Ostate\n"s3|Hf>uIM		break;
		}

		breloc(ss_get_pu_ii-__ state\n"s4) * mega		}
	}
">|Hflthe _TExR VD_abs_.

		} F}
9HIce->neCt
	dcE_TIlurre i.f>		fw_stWF>FW5l.

		} F}
9H}
9HneCt
	t 20 sev->df>		fw_stpr staties*
	 * Rl.

		} md->notimd _>		fw_stupgraf>|NTR_
	nce->cmd{
	boot.

		}
	}
	 */
	dm	abs_sWF>FWF>FWF>FW5lpHrlHE_TIMEOUHs used>w(Ktat_seq>pdejboorepMAE_TIloc(ss_get_pu_ii-_Ostate\n"s3.tatSeqNumJbooFPEOUHs used>w(Kf>|Hf>|_rr_as_steHvjbooMAE_TIloc(ss_get_pu_ii-__ state\n"s4.state;
TpMEmaperlHE_T_id * mega megaslways usframe_dma__ad_aMRa;
	MRaoid me used>w(KiOF>mk;

(loc(ss_get_nmd = DEBUG,? 0 :instancHf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>frame_dma___LIN\IMEOU(%dMB)R IGNOREe used>w(KiOF>mk;? "aMR"nsstanIGNORE) to get theloc(ss_get_nmd = DEBUGX_PD * ;
	u32 fw_d&instance->fw_outstandiE_TIloc(ss_get_pr staties.OnOffPr staties.d&instance->fw_outstanEOUHs used>w(Kfelureejboorstate;
MAE_TIloc(ss_get_pu_ii-_Ostate\n"s3.state;
SelurityonJBODtancHf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf "nce->f C max_wait tstan(rlR \IMEOUR IGNOR ;
	u32 fw_d&instance->fw_outstand? "D&instad"nsstEnnstad"stancHf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf "Selure JBODHf>|Hf>|\IMEOUR IGNOR ;
	u32 fw_felureejboorstate;
M? "Yes"nsstNo"egasas_instancReset);
		}
	}
9Hf>linc
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>linc
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf FW's controller PD
 * list structure.  This information is mainly used to find out SYSTEM
 * supported by the FW.
 */
static int
megasas_get_pd_list(struct megasas_instance *instance)
{
	int ret = 0, pd_index = 0;
	struct megasas_cmd *cmd;
	struct megasas_dOUHrlR IGce);
			break;
		case IGNORE_TIMEOUT:
			dev_info(&instance->pdOUHrlR IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TI_anceHv, oc(ss_ge_P);
		s
			 *cS)CMEOUHHrlHE_TIMEOUHrlR IGNORE_TIMEOUHrlHE__TIvF>hHe3bp6rn -ENODEV;
		}

	rollcrash_dump_pNORmed aSe>cmds_cmd *cmd;crash dumptancestance-stance-
		fw		fw_s6OFd] h6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6OF>6crash_stafree(i		-	tellICE_ID_F>hHeON/OFF;crash dumptfeaturn -HrlR R_CRASHyte)_TU->oOFF;to_ -HrlR R_CRASHyte)_TU->oONnce,6OF>6_OCR;
}
trucI_SAS10_non-zerotruc);
	EV;
nce-asn't changed "
			       "in %d secstandpNORmetters.
	 crash dumptfeaturn
nce-t scsi|NTllIseif (_cmd *cmd;crash dumptancestance
	if tandcmd(_ID_FellICEdynamiatADY) {

state;
s crash dumptfeaturn
->cmd_indexNTllIbeIseitsas_dciWaiticrash dumptfeaturn_ad_f>|Hf>|Hf>|Hf>|Hf>|Hf>|t of memory\n");
rollcrash_dump_pNORmetancet->read_fw_status_reg(
				instau8 crash_stafree(i		megasas_returnre DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore (DCMD) to ge = cpu_to_le32(MR_DCMD_PD_GET_INFO);
	dcmd->sgl.sge32[0].phys_addr = cpu_to_le32(instance->pd_iaer soft ssed>w(KERNC)DCe DCMD ti_ance= cpu_to_le32(sizeof(struct MR_PD_INF "Ignore (DCMD) to get the FW's c>|Hf>|Hcmd(instancecrash_stafree(i;\d(s used(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignore (DCMD) to get the FW's c\d(s used>w(KERNC)Dfd i;
	u8 max_wa->pd	u32 fw_state;
	nceHv, "Ignore	le16_to_cpus((u16 *)&instance->pd_info->stateCRASHyancyte)_ FW's c>|Hf>|Hf>|Hf>|Hf>|Hf>cs&insCisf cpu_CTRL-*/
	CRASHyaUMP_PAu8 S->state.ddf.pdType.intf;
		break;

	case DCMD_T[i] = kmallrash_dump_t;
	u32 fw_state;
	nceHv, "Ignor3 (DCMD) to gCRASHyancyte)_ FW's c>|Hf>cs&insCisf cpu_ii-___LINEb9>/DReturn 0 for s c\d(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	i;
	u8 max_A cgHf>|Hf>|Hf>cs&insCisf cpu_iie>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOrlR IGNO=E_TIMEOUdRE_TIMgl.sgnc
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>linc
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf FW's controller PD
 * list structure.  TThis information is mainly used to find out SYSTEM
 * supported by the FW.
 */
static int
megasas_get_pd_list(struct megasas_instance *instance)
{
	int ret = 0, pd_index = 0;
	struct megasas_cmd *cmd;
	struct megasas_dOUHrlR IGc */
statiD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignore DCMD ti_anceHv, "Ignor>vOF>vte !GFP_d aIe != MFIe conteCEdynao keep the differences between 32 andasn't cmax_c;
	nce->fmdf>|Hf>|Hf>|Hf>|Hf>|Hf>|H>vOF>vte !GFP_ to alloc mem in get_ld_list\n");
		meg_) to  izeof(struDCMD ti_anceHv, "Ignore DCMD ti_anceHv, "te !Ge (DCMDte !Ge (DC DCMD ti_anceHv, "te !G	int ss_gee-te !qss_geDCe DCMD ti_ante !Ge (DC_hDCe DCMD ti_ante !qwitch (L);

		if (Prepw_stante !);

	if Not individ !);

	i)
				_list	int _abs_	if ( -ENODEV;
	ommands themselSGLft state
 tance)
ree(in_sz  = 1.nsta	if (mic ads the-UHrld_lwe do
	/*ne
 tanylSGLf-lwe megaSGL'd_fpad_las	if (	int _abs_)eHv,|6OF>6ance->frWexNTllIHf>|pd_iaen pr	       "belhyy of j	swi KILL_d
 */
stat= NULL;
= cpu_to_le32(MR_DCMD_PD_GET_INFO);
e !Ge (DCM_lenD ti_anceHv, "te !Ge (DCMD)e32(sizeof;);
e !qss_geestenD ti_anceHv, "te !G	int ss_gee- max((unnstainstong)te !Ge (DCM+ a_pFO);
e !Ge (DCHv, "e32(sizeof 0;
}

/**;);
e !qss_geHv, "
e !Ge (DCHv,+ a_ ge =ance->cmd
e !Ge (DCEOUHeof(strucRE_TIMEO !Ge (DCMD) tonstance->pdev->de)trucRE_TIMEO !qinstance->adapternD ti_anceHv, "te !G	int ss_ge_|Hf>
e !Ge (DCEOUHeof(s, "eHeof(str);
e !qss_geTIMEplyG	int seitries*

	case DCMD_T[i] = kmal			Gfwe usedpool) {
	 !qss_geTIMEplyG	int s] =;
Tp;
}

/**_lo*

	case DCMD_T[i] = kmalMEplyG	int sh)tr);
e !qss_geTIproduca_	ifdexTp;
}

/**_lo*

	case DCMD_T[i] = kmalproduca_	hl) {
	 !qss_geTIIMEOuma_	ifdexTp;
}

/**_lo*

	case DCMD_T[i] = kmalIMEOuma_	hpFO);
e !Ge (DCsed(s used>w(KERore tha
e !Ge (DCsed(sx_wait;
	u32 fw_state;
	nceHv, "Igno
e !Ge (DCse	int ss_ge_newGp;
}

/**_lo*
get_case DCMD_Tasasr_32__);s(
e !qss_geHv_|Hf>
e !Ge (DCEO	int ss_ge_newGp;
}

/**_hi*
get_case DCMD_T>|Hsr_32__);s(
e !qss_geHv_|Hff>
e !Ge (DCEO *)&instance->pd_info->state.ddf.pdType);
_anceHv,te !G	int ss_ge_|Hf;

		if (d&instaindividtr"bef pd_ree				ndivid !);

	i)istCEdNULL;

			return
			retul(MFI_RESET_FLAGS,
					&in;

		if (asn't	ndivid !);

	i)ineCF>vOF mHf>dNULL;
IES)
		return>vOF>vOF>vOF>vOF>vOF>vOF>vOsgl.sge32[0].phys_addr = cpu_to_le32(instance->id !);

	dcmd->mbox.b}

		break;

	case DCMD_SUCCESS:
		ifgo
		fnstGfweid !int i;
	uti_anceHv, "Ignore DCMD ti_anceHv, "IgnoreIgnOfnstGfweid !:
ERNC)DCe Dte;
	mbox.b, 0, Mu32
_anceHv,te !Gpu_ii-__FP_ to alloc mem in get_ld_list\n");
		megto alloc mem inregUdREr
rol	stiocmd-*	Destroncore DeHeof(syte)
	 *
	MEplyG	yte)

ERNestroest[i] = kmal	Destronc;

		if (Geen ariot;
 state\n"aldpNORmetters.}
9H_wait;
regUdRErdNULL;

			return			Gfwe used\n");

	return 0;
}

/*megasas_teardown_fr	Destroy & 0x00FFpollereate_fraducaH(struax_f>|Hf>|Hf> used|Hf1
->cmd_i_listeEOu_abs_seH(state_fMEplyG	yte (1urr_abs_sta(struax_d(s miatADY) {

mayIseifpool =doedeviceexce
 tuax_d(ssbs_seH(stICE_can_f>|Hf>|dNULL;

			return			Gfwe used\n");

	retu			Gfwe use-1;;

			return			GFP_KERNEd\n");

	retu			Gfwe use;;

			return			G>pdev->dev(");

	return 0;
}

/*megasas_teardown_fr	Destroy & 0xFF0:
		 >		break0x10;
	 */
	instance->,
				me *dcmdsNORE_TIMEOI_STATateTw(KEScmd_list led\n"re
	}
9rvDMA64), the dpoDY) {
's(!id "
			inde = NULL;

ce->reg_set->inbound_doorbell 		max_wait = MEGASAS_RESET_WAIT_TIME;
>reg_set->inbound_doorbell 		max_wait = MEGASAS_RESFI_STATE"e frame to each
		red = ERNEd\nT[i] = kmal			Gfwe used-	breRE_TIMEOI_STATateTw(KES
		ifseDCMte !phys_addr = ioctlstrmNORE_TIMEOI_STATateTw(KES)gnoance->hba_lme to each
		red = ERNEd\nT[i] = kmal			Gfwe used-	breRE_TIMEOteTw(KES
		ifseDCMte !phys_addr = ioctlstrmNO;
		}

		mis iname (KES
);ure
	 */
	for (iurr_canG	int d\n");

	retu			Ged = ERNE;gasas_instance *in) {

md;cd_list led\ERIES)
		return		return -E	}
9Hf>lin	ifgo
		fnstG		return -nc;

		if (tance->cmzeof(st64),MEply(	int . L "Ignomd;MEply(	int tate;
		if (be _one_urr_abs_sta(struaximumsance->cmd_stdlHf>|Hf>|Hf;

	dcmd.6ance->frNot : We->cCE_c;
	int ssance->cm, e->nlac ssanrressondntk(KHeof(e->fr allocato look circula),MEply(	int . Took circula),	int _aelvefnsrly	if (mstruct produca_-IMEOuma_(	int . u16 de */
sroduca_ (md;cd_	int jce->cmd_list )sas_sge64DY) {

i&instanceOuma_= NULL;
=Heof(syteMA HrlR IGu_FRAM	MEplyG	yte =DeHeof(syte->cT[i] = kmal			Gfwe usedpool) 	 */
	for (iMEplyG	int nore (DCMD) to get the FWF>FWF>FWF>FWF>FWF>F timeouMEplyG	yte>FWF>F timeou&*/
	for (iMEplyG	int sizeof(struc*/
	for (iMEplyG	int MR_PD_LIST));
	dcmd->opcode = cpu_to_le32(MR_DCMD_PDOcmd->s= sizeot64),MEply(	int ->mbox.bgo
		fnstGMEplyG	int ;struct MR_f>|Hf>|H>vOF>vte !GFP_ 	}
9Hf>lin	ifgo
		fnstGfweid !inct MR_f>|Hf>|Hev, loc(ss_ge 	}
9Hf>lineCtrle32[0].phys_addr = cpu_to_le32((%d): C fusic\d(srame_dma__abs__IO_TIMinsts.}
9Hruct megan");

	retuuniqF>vt>host,md *cmd;
	struct megasago
		fnstGfweid !int i;
HrlR IGNORE_TIMEOUdR;
	/*_to_cpuHrlR IGNORE_TIMEOUdR;
	/*_tE;
>reg_set->irn 0;
}

/*megasas_teardown_fr	Destroy &
ak0x04000000)cmd->daUT:
orbphys_addr = cpu_to_le32(_anceHv,te !GFP_: E_TIMEOUdR;
	/*=%dIGNOR ;
	u32 fw_E_TIMEOUdR;
	/*k_interrupts)
		retE_TIMEOUdR;
	/*kframe to eachEunt;
	/*nce, c>|"IgnoreIgnOfnstGfweid !:
lR IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOUMEplyG	yte>FWF>meou*/
	for (iMEplyG	int ,u*/
	for (iMEplyG	int sizeofnstGMEplyG	int :;
	uti_ancturn IGNORE_TIMEOUT;

fnstG		return -:
ERNC)DCe16rn -ENODEV;
		}

	rolup_irqv,toatru ->vOFgUdREr l, "cy Dfd i;
	u8sdev)
{
	int ret;
	struct megasas_cmd *cmd;
Doeviceennsta Dfd i;
	u,sas_dc>senseISRs* free cROCR;
}
trucI_SAS10Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hrolup_irqv,toatru to alloc mem in get_ld_list\n");
		megto allo IGNIMEloc ev c>|c evd\n");

	retuc ev c

			return
rqsizeof(sceHv get_ld_l not under O

			return
rqsizeof(sceHvMSIxIX_PD * sizerlR IG	in

	irq( IGN
rqsvector(TIMEOU0)GNOREe used>w(Kin 0;
}

/*
9rve, cmsr, IRQF_SHAREDf>|Hf>c mem i" = cpu_to_le3
rqsizeof(sceHineCtrle32[0].phys_addr = cpu_to_le3FWF>F(instance->OFgUdREr IRQs.}
9Hruct megasas_cmd *cmd;
	struct megasaRNC)DCe 1gnoa
u_alloc_cmds -	Allocates the rolup_irqv,msix ->vOFgUdREr MSI-x Dfd i;
	u8sdev)
{
	int ret;
	struct megasas_cmd *c)
{sT))obret;
	t scsi|))obr e->pd *cmd;
Doeviceennsta Dfd i;
	u,sas_dc>senseISRs* free cROCR;
}
trucI_SAS10Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hrolup_irqv,msix>cv");

		if (nareak;
	}

	info(&instan{sT))obrlist, 0, s,structo allo IGNIMEloc ev c>|c evd\n");

	retuc ev c
u/inT(stMSI-x ed to
 * the FW.
 *
 * Each frasixsvectorsign one frame to each
rqsizeof(sciHv get_ld_l not under O
ame to each
rqsizeof(sciHvMSIxIX_PD * i;set_mutIG	in

	irq( IGN
rqsvector(TIMEOUi)GNOREe used>w(Kin 0;
}

/*
9rve, cmsr, 032(_anceHvgasas_ cpu_to_le3
rqsizeof(sciHineCtrlle32[0].phys_addr = cpu_to_le3FWF>F(instance->OFgUdREr IRQs.
 *vectorct .megan"egasasr creating frame DMA pool\n"turn irq( IGN
rqsvector(TIMEOUjce)) {
	= cpu_to_le3
rqsizeof(scstanc	mega	ROC(st
rq>OFgUdREr 64), t_APIC}
	}
9 * Each frasixsvectors * sizeet_mutesT))obrl

	info IGNORE_T
rqsvectorsIMEOUHrlHE_TIMEanc	meu_alloc_f>|Hf>|Hrolup_irqv,toatru 
static int
meance->hba_lsaRNC)DCe 1gnof>|Hf>|Hoa
u_alloc_cmds -	Alocates the EL, &cm_irqv->vunOFgUdREr Dfd i;
	u8sdev)
{
	int ret;
	struct megasas_cmd *c)_allocet;
	FI_MB he pool itself
tes the EL, &cm_irqv_DEV_PER_CHANNEL) + sdev->id;
	cmd = megalike IOCerrupts)
		retnsixsvectorsool\o
 * the FW.
 *
 * Each frasixsvectorsign one fra"turn irq( IGN
rqsvector(MEOUHrlHE_TIMEOUi)GNORE	= cpu_to_le3
rqsizeof(scitanc	m}vOF>vOF>vturn irq( IGN
rqsvector(MEOUHrlHE_TIMEOU0)GNORE= cpu_to_le3
rqsizeof(sceHimds -	Allocates the rolup_jboor	ap -	>sensejbooM	ap o
 *FPc>sqG>pdba_= ev)
{
	int ret;
	struct megasas_cmd *c)
{sT))obret;
	t scsi|))obr e->pd *cmd;
ROCR;
}
trucI_SAS10Hf>|Hfself
tes the rolup_jboor	apENOMEM;
	}

	memset(instance->cmd_list, 0, sizethis cmax_cmd; i++) {
		instaist[i] = kmalloc(sizeof(stru *
	pd_seqGFape Dgn>|c _seqGFape DMA HrlR IGNORE_TIMEOPTIiFG_SEQ_NUM_SYNC) +EOUse INITIATE_OCR:
			ciFG_SEQ)->cTRci-PHYSList[lEAD);Itat1megazerlR IGLL;

	INIT_s(str	instaiIME;
![i] = kmalloc(ss_get_pu_ii-_Ostate\n"s3.tatSeqNumJbooFP -ENOMEM;
	}

	memset(ci, 0, sizeof(>|Hf>JbooM	ap  devicef>|Hf>|Hf>ruct megasas_md *cmd;
	struct megasa");

	retuuat_seq>pdejboorepMA	fn>vOgasaRNC)DC;struct MR_ocmd_lalc _seqGsyncceHiasago
		skipG		retCreate a frame pool JBOD inPS_COUNTign one fraocmd_lalc _seqGsyncc
	if  DCMDMD) to he->mtsas_phys_addr = cpu_to_le32c _seqGFape Dasas_ ocmd_lalc _seqGp;
}c
	d = instance->c	IES)
!>cmd_lalc _seqGsyncc
	neCtrlle32[0].phys_addr = cpu_to_le3FWF>F(instance->aance->cmzeof(st6}
9Hruct megasas_cmd *cmd;
	struct megasaCerrupell 1l

	info DCMORE_TIMhe->mtphys_addr = cpu_to_le3FWF>F|c _seqGFape D, }cmd_lalc _seqGsyncceH3FWF>F|ocmd_lalc _seqGp;
}c0tanc	meaocmd_lalc _seqGsyncctancen progre>|Hf>a");

	retuuat_seq>pdejboorepMA	fn>vOgasaaRNC)DC;st OCR isskipG		ret:f(structes the rync_c _seqG>pdre DCMD ti_fn>vO for s	ctes the rync_c _seqG>pdre DCMD ti_TE_lin	if");

	retuuat_seq>pdejboorepMA	TE_l>vOF>vOF>v");

	retuuat_seq>pdejboorepMA	fn>vOgaox.b, 0, MFI_Mates the rolup_MEplyG	apENOMEM;
	}

	memset(instance->cmd_list, cm);	dcmd->ccpuERNC

	/sHrlRunnstains 0, 	int ,ucpuCreate a 	int nore p	int n*
 * Each frasixsvectorsig	int  one fraERNC
ore (D
rqsev, affid !y(MEOUHrlHE_TIMEOU	int M>c	IES)
!ERNCool\ngo
		fnll_all	}

	inf_ype t thelpu, ERNCool\n*/
	for (iMEplyG	ap[lpuC)Dfd it ;stru* megasas_fnll_all:
	inf_ype tet);
		}t thelpun	if");

	retuMEplyG	ap[lpuC)Dflpu %
 * Each frasixsvectorsiD ti_anceHv, "Ignor>e !Gewd aIe != MFIe conteCEdynao keep the differences between 32 and>cmd_i_lihv_prin  *cmte = te aie != MFI				 w		fw_s6OF/
f>|Hf>|Hf>|v, "Ignor>e !GewENOMEM;
	}

	memset(instance->cmd_list, *
				Geectors_1;t, *
				Geectors_2i_TmpGeectors, Esixsennsta;t, *
	scatee tead_2i_scatee tead_3i_scatee tead_4AM	MEsour, ce IN_t>ce, 

/**;);to alloc mem inregUdREr
rol	stiocmd-*	Destronco->dev, _anceHv, oc(ss_gee->oc(ss_gencen progrunnstainstong>cerReset;t, 0, s,st,d, sp, E_TEsixsto get tnre DCMD tiIOV_111ancovPtrizethis cmax_cmd; i++) {
		instance->cmd_list[i] = kmalloc(sizeof(strru/insw_stree(inzeof(stcer ed tcerReset
ore (Dselect_cersIMEOUHrlHE_TIME,iIORESOURCE_MEM) O

			returncer =	fw_s_ree(i__);
&cerReset, BITSe->loLONG)izerlR e (DIG	in

	selectednregUcm)IMEOUHrlHE_TIME,i1<<
			returncere)) {
	=(_anceHv: LSI"nstance->unload ||
			test_bit(MEGASAS_FUSION_OCR_NOTIOnzeof(stregUcm busy!sed>w(KERNC)DCe DBUSY;structce, 

/**
ore (DMEsour, ce =;
(MEOUHrlHE_TIMEOUi			returncer) O

			returnRNestroest[oreFapence-e->(ce, 

/**, 8192zeof(struc*/
	for (iMEestroy ance->unload ||
			test_bit(MEGASAS_FUSION_OCR_NOTinstance->	ap IOnzeo->mbox.bgo
		fnstG[oreFap;ITIATE_Oestroest[i] = kmal	Destronc;
_mutex);
			break;
		case KILL_ADAPTER:
			me used>w(Kin 0;
}

to_l	}

	memset(inst_templLL_ADinstancoce->hba_lnc
		}
	reg_set->inbound_doorbneCtrlReset		max_wait = MEGASAS_1078
 * sReset		max_wait = MEGASAS_1078DE:NOREe used>w(Kin 0;
}

to_l	}

	memset(inst_templLL_Appcnt
megasas_get_pd_l		max_wait = MEGASAS_1078GEN2 * sReset		max_wait = MEGASAS_0079GEN2 * sEe used>w(Kin 0;
}

to_l	}

	memset(inst_templLL_Agen2nt
megasas_get_pd_l		max_wait = MEGASAS_RESET_WAIT * sReset		max_wait = MEGASAS_0071T_WAIT * sEe used>w(Kin 0;
}

to_l	}

	memset(inst_templLL_A,
				nt
megasas_get_pd_l		max_wait = MEGASAS_1064
 * sReset		max_wait = MDELLe->lC5 c\|Hffault * sEe used>w(Kin 0;
}

to_l	}

	memset(inst_templLL_Axscalsues an internalE_TIMEOUT:
			dev_info(&instas_dOUHrlR IGc nct MR_f>|Hf>|Hate!!ie\n"fo->eset)re DCMD ti_0nstanceatomicstrophys_addr = E_TIGLL;
no_e (DCSAS10,instance used>w(Kin 0;
}

/*adpTIGLL;sas_pe DCMD ti_*/
	for (iMEestroy;nceatomicstrophys_addr = E_TIGLL;
no_e (DCSAS10,i0stancHf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>_DEVee =;
edcI_SAS10fullst6}
9Hru!megasas_md *cmd;) * megawait
				 e aab
	nc30 eeco  "bef pd_rOC(s}
	}
ssleep(30>reset_mutf>|Hf>|Hate!!ie\n"fo->eset)re DCMD ti_0nsol\ngo
		fnstGMEet)free(i;\d}c;
_mutex);
			break;
		case Kll VENTURAeturn 0 fba_lncatee tead_3MAE_TIMEetllR IGNORE_TIMEestro->
	nb
	_s_ncatee tead_3stance used>w(Kn DrHrlR map_cmd = ((ncatee tead_3M>		bre R_MAXCRAMR inPe->deoOFF*/
	SHIFTy &
ake R_MAXCRAMR inPe->deoMASK);ure
	 /in megasas_MSI-X_ad_f>|Hf>|Hf>
		}
	ire Det);ree(i ed tEsixsennstadev(");

	return 0;
}

/*megasas_teardown_fr	Destroy &
 timeout:0x4000000)M>	:0x1aizerlR Esixsennstad&& !EsixsFI_RESEne framett
rq_d(s used		maIRQ_MSIXresetscatee tead_2sedMEetlsas_phys_addr = MEestro->
	nb
	_s_ncatee tead_2|Hf>u/in megasuax_MSI-X_vectors oid meMR_ocmd_lfo(&insta_Hex);
			break;
		case Kll THUNDERBOLTeturn 0 fba_l>u/inTh_id =bolt Series}
	}
99 * Each frasixsvectors * (ncatee tead_2)) {
	&  R_MAXCREPLYINEbUESoOFF*/

	dc1nc	meao_TEsixsto get t * Each frasixsvectorsiD
meance->hb /inInvad = series
state;
s rr_abs_sta8tMSI-x vectors}
	}
99 * Each frasixsvectors * ((ncatee tead_2)) {
	&  R_MAXCREPLYINEbUESoEXToOFF*/

)) {
	>	: R_MAXCREPLYINEbUESoEXToOFF*/
	SHIFTy dc1nc	meaerrupts)
		retnsixsvectors > 16
)) {
	pts)
		retnsixscombinnfo(&TE_l>vc	meaerrurdpqsennsta
)) {
	pts)
		retiowndpq * (ncatee tead_2 &  R_RDPQ_MODEoOFF*/

	?)) {
				1 : sizeetao_TEsixsto get t * Each frasixsvectorsiD
meu/inSave{1-15,MEply(et)ante_PD (_cmd *c
		 */
snzeof(sNORE	=inIn_PD 0_aelvReset);sav	}
	}
9HMEe offLL;sas_	=inMPI2CREPLYIPOST->mas_INDEXoOFF*/
sas_	=i
	}
99E_TIME spo(&ind, sp <: R_MAXCMSIXCREG_ARRAYnd, sp one fra"if");

	retuMEplyGet)a_uctu	ifdexT(_cm[, sp]turn;
}s_p *
	stiocmd-*)rn;
}s_p(u8	stiocmd-*)
			returnRNestroe+EOUHrl	MPI2CSUPCREPLYIPOST->mas_INDEXoOFF*/
sas_			+IME spo*:0x10)anc	mea}gre>|Hf>a"lR Esixsvectorsool\99 * Each frasixsvectors * min Esixsvectors,)) {
	pts)
		retnsixsvectorsorlR Ince->h/ince->e *dcmds }
	}
9 * Each frasixsvectors * 1Hf>u/inDo
	/*boys usaance->				rr_abMSI-X_vectors s_stak;
	aoid me used>w(Kasixsvectors * min e used>w(Kasixsvectorse)) {
	=====(unnstains 0,)>pdeoce->fak;
		)M>c	IES)
smp affid !ysennsta
)) {
rq_d(s us|ed		maIRQ_AFFore Y>c	IEnore (DCMD) t
rqsvectorsIMEOUHrlHE_TIME,ine)) {
	==e used>w(Kasixsvectorset
rq_d(s uM>c	IES)
i > 0)	}
9 * Each frasixsvectors * i;setreak;
		case IGNORasixsvectors * size};

		if (MSI-X_ht)ante_PD 0 led cmm = te aCMD>e *dcmd.	if (at ledtate;te aCMD>MPT>ce, stifferen = NULL;

ce-pts)
		retnsixscombinnfne frame to eachMEplyGet)a_uctu	ifdexT(_cm[0]turn;
}}
e D*)p(u8	*)
			returnRNestroe+EOUHrMPI2CSUPCREPLYIPOST->mas_INDEXoOFF*/
)gnoance->hba_lme to eachMEplyGet)a_uctu	ifdexT(_cm[0]turn;
}
e D*)p(u8	*)
			returnRNestroe+EOUHMPI2CREPLYIPOST->mas_INDEXoOFF*/
);\d}c;
_mut!pts)
		retnsixsvectorsohba_lmnore (DCMD) t
rqsvectorsIMEOUHrlHE_TIME,ineinei		maIRQ_LEGACYM>c	IES)
i < 0)	}
9go
		fnstG[e !Gpu_ii-_;nt i;
	uti_ancrolup_MEplyG	apE
static intol, cmd->frame,
				      cmd->f
TIME>		fw_ststate;
s rsix\IME(%d)", E_TEsixsto ge;
	u3 cmd->frame,
				      cmd->f
TIMlurre i.Esix/oce->fak;
	\IME(%d/%d)R IGNOR;
	u32 fw_asixsvectorset(unnstains 0,)>pdeoce->fak;
		)M>c	3 cmd->frame,
				      cmd->f
TIMRDPQ mHf>\IME(%s)megan");

	retuiowndpq ? "ennstad"nsstd&instad"intoltaskl_T_ie !phys_addr = is	caaskl_Tan");

	retuin 0;
}

/*aaskl_Ta
;
}
nnstainstong)tetatic intol[i] = kmalloc(ss_genorkzCMD) get the FW's c\_anceHv, oc(ss_ge_e)) {
= instance->c	ES)
[i] = kmalloc(ss_geKll n prn	ifgo
		fnstG[e !Gpu_ii-_;n;

		if (Belhysw_stWFfaultr allo_, __L, "cy F

	dcmd.6anc_non->cmd_lice, stframe_dma_pdNULL;

			returnE_TIMEOUdRE_TIMEOUHrl =ORci->ldList[laC	br;fr
			returnE_TIMEOUdRE_TpMEOUHrl =ORci-PHYSList[lEAD);Igno/ (Geen state\n"aldpNORmsetv->dd(s u,Iseif id !)d(s mo(srame_dma__LL;

ce-pts)
		retin 0;
}

/*[e !Gpu_ii-_ 	}
9Hf>lin	ifgo
		fnstG[e !Gpu_ii-_;n;
_mutex);
			break;
		case Kll VENTURAeturn 0 fba_lncatee tead_4MAE_TIMEetllR IGNORE_TIMEestro->
	nb
	_s_ncatee tead_4M>c	IES)
(ncatee tead_4M&  R_NVME_PAGEe->deoMASK) >=
ake R_DEFAULTeNVME_PAGEe-HIFTy;
		case IGNORnvof 0ag ce INturn;
}}1 << (ncatee tead_4M&  R_NVME_PAGEe->deoMASK)>resetHf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf "NVME 0ag  HrlR\IME(%d)megan");

	retunvof 0ag ce IN);\d}c;
_mutcase IGNORasixsvectors ?)) f>|Hf>|Hrolup_irqv,msix>e DCMD ti_1) :)) f>|Hf>|Hrolup_irqv,toatru 
static in	ifgo
		fnstG[e !Gpu_ii-_;n;
_ts)
		retin 0;
}

/*ennstaT_FLAGS,
					&in;
instance *instance)
{
	int ret = ;
	npu_ii-_ do
nce->resetes the rolup_jboor	apES,
					&in;
/**_, __passthrough	if (mio_, MD)w				 *cmte = NTllI\n", fw_PD IGN|= NULL;
ORE_TIMEOUHrlHE_Treak;

	}

	pcOUT:

		switch (dcmd_timeout_ocr
				MEGASAS_MAX_->c	ES)
f>|Hf>|Hev, plReset);
		}
	}
 < 0)eCtrle32[0].phys_addr = cpu_to_le32(fnstance->pd_ih (.length = cpgo
		fnstGneCtrlRplReset;nt i;
	u8 maxys_addr = IMEOUHrlR IGNORE_TIMEOUdRE_TIMEOUHrno/ (outeam	t 20 se = cn != MFIate = LL;

ce-pts)
		reteak;
		case Kll VENTURAeturn 0 fba_locmd_lalouteam_t 20 s_bytrlturn;
kzCMD) get the FW's c\ usSTREAM_DETECT-*)rn;
*ORci->ldList[laC	break;f>|Hf= instance->c	IES)
!>cmd_lalouteam_t 20 s_bytrlneCtrlle32[0].phys_addr = cpu_to_le3FWF>F(unnstade->aance->cmouteam	t 20 se = , __p {

md;LDsgth = cppgo
		fnstGneCtrlRplReset;nt>|Hf>te a frame pool Rci->ldList[laC	break;; ++ine fra"tcmd_lalouteam_t 20 s_bytrlc
	ifFWF>FkmCMD) get the FW's c\ usSTREAM_DETECT_e)) {
= instance->c		IES)
!>cmd_lalouteam_t 20 s_bytrlc
	neCtrllle32[0].phys_addr = cpu_to_le3FWF>FF(unnstade->aance->cmouteam	t 20 s>|HfLD\n "anc	meao creating frame DM++j)rn;
}skORE_(>cmd_lalouteam_t 20 s_bytrlcstanc	meskORE_(>cmd_lalouteam_t 20 s_bytrlanc	meaocmd_lalouteam_t 20 s_bytrltuen progre>pgo
		fnstGneCtrlRplReset;nt>>|Hf>a>cmd_lalouteam_t 20 s_bytrlc
	ORaru__);r	apgre>p=  R_STREAM_BITMAPrlR IGc nct MR_f>|Hf>|Hf>|Hf>|Hf>|Hf>e DCMD tisas_	=\d(s usNEbre_TYPEeakPOSED_TO->masin	ifgo
		fnstGneCtrlRplReset;ngasas_instomputaH(struax_aancwedcIectors pEr IO: Tstanceme_dma__abs__mseltw_	if (limi
s  = uax_fectors.-t scsi|ate;
	 mega(strucn mumsmd;(stega(wo.6ance->fr1 << outip cez_ops.ucn * max_fectors pEr outip6ance->frNot bs_seHolde)
ree	dcmds (ameFW csi|30)edid
	/*MEpe;
Mabs_state =	if (mo(salculate				Geectors_1. Soa(str>pdba_ E_nly up selzerotalway = NULL;
TmpGeectors * sizeHoc(ss_geest[i] = kmalloc(ss_geDC					Geectors_1 * (1 << loc(ss_get_outip cez_ops.ucn) *EOU) to get theloc(ss_get_n		GeutipEGAa_	io)truc		Geectors_2 _TIMEOUHrlHE_TIoc(ss_get_n		GIG	in

	s IN);\;
TmpGeectors * ucn_t}
e ,				Geectors_1,				Geectors_2intol[i] = kmalpeerIsPIGLLrl =OIoc(ss_get_cludREr.peerIsPIGLLrl;fr
			returnpassiv c=OIoc(ss_get_cludREr.passiv ;
d out SYSTEM
 * sucludRErId,OIoc(ss_get_cludRErId,Oasas_get_ld_list(cludRErId)) O

			returnUnevenSpanState;
MAE_Tloc(ss_get_pu_ii-_Ostate\n"s2.state;
UnevenSpans>c	ES)
[i] = kmalUnevenSpanState;
 fba_lnhis cmax_cmd; i++) {
		instaist[i] = kmalloc(sizeof(stru	ES)
d(sV MFbs_sMapI_ge 	}
9Hf>linHf>a>cmd_lalfa

	neCt	io * 1Hf>ureak;
		}cmd_lalfa

	neCt	io * 0pdOUHrsta_Hroc(ss_get_uctu	ifi-_face.SRIOVne frame to eachME	in

orIdc=OIoc(ss_get_iov.ME	in

orId>c	IES)
ieg_set->inbound_doorbell 		max_wait = MEGASPLASMAfo(&insta_H!loc(ss_get_pu_ii-_Ostate\n"s2.aloiv Passiv nHf>ameou*/
	for (iPlasmaFW111ace, c>|ncHf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf "SR-IOV: E>		fw_st_LINMEOUR IGNOR meou*/
	for (iPlasmaFW111a? "1.11"nsstnew->rese	IES)
ieg_set->iPlasmaFW111fo(&insmeou*ovPtrestenD ti_aIOV_111an)rn;
}((unnstainscher e)Hoc(ss_gee+aIOV_111oOFF*/
);\dF>meou*/
	for (iME	in

orIdc=O*ovPtr(iME	in

orId;nt>>|Hf>}setHf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf "SRIOV: VF ME	in

orIdct megasas_*/
	for (iME	in

orId);ure
	 */
	for (iurash_dump_E_TIMEOUdRMAE_Tloc(ss_get_pu_ii-_Ostate\n"s3.state;
CrashDump;	 */
	for (iurash_dump_drvTIMEOUdR_tE;
>reg_set->iurash_dump_E_TIMEOUdRMor s	reg_set->iurash_dump_buf->c	ES)
[i] = kmallrash_dump_drvTIMEOUdRMEOUHrlHE_TIrollcrash_dump_pNORmete DCMD tisas_ R_CRASHyte)_TU->oOFF>resece->hba_lmS)
[i] = kmallrash_dump_buf-sas_ IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOrn;
}CRASHyancyte)_ FW',ol\99 * Each frlrash_dump_buf,ol\99 * Each frlrash_dump_hstance used>w(Klrash_dump_buftuen progrnce= c cmd->frame,
				      cmd->f
TIM IG id\|\IME(0x%04x)/(0x%04x)/(0x%04x)/(0x%04x)megasas) to get theloc(ss_get_ IG.vendo_	id)asas) to get theloc(ss_get_ IG._doorb	id)asas) to get theloc(ss_get_ IG.sub_vendo_	id)asas) to get theloc(ss_get_ IG.sub__doorb	id)M>c	3 cmd->frame,
				      cmd->f (unevenspan_f>|Hf>|	MEOUR IGNOR;
	u32 fw_UnevenSpanState;
M? "yes"nsstno"egas3 cmd->frame,
				      cmd->f (E>		fw_stcrash dump	MEOUR IGNOR;
	u32 fw_urash_dump_drvTIMEOUdR_? "yes"nsstno"egas3 cmd->frame,
				      cmd->f (jbooMrync>	ap		MEOUR IGNOR;
	u32 fw_uat_seq>pdejboorepM? "yes"nsstno"ega
	 */
	for (i			Geectors_Aa_	ME	d\n");

	retu			G>pdev->d*sas_			SGEyte)FER_ FW' / 512>c	ES)
TmpGeectors &&cT[i] = kmal			Geectors_Aa_	ME	d>_TmpGeectorsin	if");

	retu			Geectors_Aa_	ME	d\nTmpGeectors;
	 /in megas.
 *v MFb throttled it depth mHfutadpNORmette LL;

ce-throttled it depth or s		throttled it depth <\n");

	retu			Ged = ERNEn	if");

	retuthrottled it depth = throttled it depth>vOF>vOF>v");

	retuthrottled it depth =EOUHrME_TIMEOTHROTTLEINEbUEax_PTH c>|Hf>csIGLL;wait
	m n*
1_TIME;====(IGLL;wait
	m n>ORE_TIMEORE*/
	WAITruct ) max_wLL;wait
	m n=ORE_TIMEORE*/
	WAITruct  c>|Hf>css
9Hf>lineCtn*
10_TIM ss
9Hf>lineCtn>ORE_TIMEODEFAULTeIMEOUdRE_TIM)a_lnc9Hf>lineCtn=ORE_TIMEODEFAULTeIMEOUdRE_TI;
	 /inLaunch SR-IOV heartbeseH(ima__LL;

ce-pts)
		retME	in

orId)hba_lmS)
ctes the rr*ovs] =;
Theartbese>e DCMD ti_1)-sas_tes the r =;
T(ima_>e DCMD tisas_		meou&*/
	for (irr*ovsheartbeseT(ima_isas_		meoutes the rr*ovsheartbeseT_stdlH_isas_		meouRE_TIMEOIRIOV_HEARTBEstateTERVAL_VF);setreak;
		case IGNORskipGheartbeseT(ima___dl * 1Hf>}c>|"IgnoreIgnOfnstGneCtrlRplReset:;
_ts)
		retin 0;
}

/*FI_RESET_FLAGS,
					&in_tes the EL, &cm_irqv_S,
					&infnstG[e !Gpu_ii-_:OCerrupts)
		retnsixsvectorsool\ IGNORE_T
rqsvectorsIMEOUHrlHE_TIMEanc	case IGNORasixsvectors * sizfnstGMEet)free(i:OCkORE_([i] = kmalloc(ss_geanc	case IGNOR>oc(ss_gencen progrioun	apES,
					(iMEestroy;n
fnstG[oreFap:lR IGNreleaat_selectednregUcm)IMEOUHrlHE_TIME,i1<<
			returncer&in;
inst0].phys_addr = cpu_to_le32(instanc6}
9Hruct megasasmd *cmd;
	struct megasRNC)DCe Dte;
	mbox._anceHv, "Ignorreleaat_FP_d aReverse conteCE cn != MFIate = ev)
{
	int ret;
ifferences between 32e pool itself
	 */
	dmreleaat_FP_ENOMEM;
	}

	memset(instance->cmd_list, *
	MEplyG	yte =DHrlR IGu_FR *T[i] = kmal			GFP_KERNEdpool) 	 *ce-pts)
		retMEplyG	int M
s_ IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOUMEplyG	yte>FWF>meou*/
	for (iMEplyG	int ,u*/
	for (iMEplyG	int sizeo;
	uti_ancturn IGNORE_TIMEOUT;

rioun	apES,
					(iMEestroy;n
R IGNreleaat_selectednregUcm)IMEOUHrlHE_TIME,i1<<
			returncer&inasas_cmd itself wo to seqG>pdd aGetsflthe _Teventc>squensta>pdba_s ev)
{
	int ret;
ifferences between 32 @eliet;
FW eventclogc>squensta>pdba_sMabs_state =	32 andFW printrinelvelogcofaCMD>eventcato a_non-volth	}
	w_sa. U|Hsrfltya_sMwe;
		andusuallst6w_stateH(strlthe _T>squensta>pdba_smd;(st>eventc,;(st>>sqa>pdba_sat
f (mio_boot etc. ToeyMwe;
	 "MEet"aCMD>(st>eventc"belhyH(strlthe _T>sqa>pdba_
f (by >vOF				a dir0 s>fw)d(s in %d .nsta(mio_,u an aeventc"(beyo  "lthe _T>sq
f (>pdba_),;(styMwe;
	 subrr*bade->f>li(arynchronot;
eventcT:
oficate =)sas_
f (waits.
 *(st>eventc"t__ms|HsnHf>|Hf>|Hf>|Hf>|Hf>|Hf>|H to seqG>pdtancet->read_fw_status_reg(
				insta>meouancet->read_fw_evCtrogss_gee-eli		megto alloc mem in "Ignore DCMD ti_anceHv, "Ignore (DCMD) to get the F_anceHv,evCtrogss_gee-elss_geDCe DCMD ti_anelss_geHv, "IgnOUH cpu_to_le32(MR_DCMD_PD_GET_INFO);
	dcmd->sgl.sgRNC)DCe DCMD ti_anceHv, "Ignore DCMD ti_anceHv,	e(ss_gencee (DCMD) to get the FWF>FWF>FWF>FWF>FWF>FWFFFFFFmd_timeout_ocr
				MEGevCtrogss_ge)>FWF>FWFFFFFF&elss_geHvNFO);
	dcmelss_gene fraE

		break;

	case DCMD_SUCCESS:
		if  getsF>yce-C	int i;
	u8 maxe(ss_genct;
	u32 fw_elss_gen)trucRE_TIM "Ignore (DCMD) to get the FW's c\d(s used(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u0x_cpus((u16MD) to get the FW's c\d(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignore	le16_to_cpus((u16 *)&instance->pd_info->state.ddf.pdType);
_anceHv,evCtrogss_ge)	u32 fw_stf>|Hf>|Hf>|Hf>cs&insCisf cpu_CTRL-EVENT->pd_info->state.ddf.pdType.intf;
		break;

	case DCMD_Telss_geHvNFOu32 fw_state;
	nceHv, "Ignor3 (DCMD) to get the FW's c\_anceHv,evCtrogss_ge)	u3ct MR_f>|Hf>|H_state;
	i;
	u8 max_A cgHf>|Hf>|Hf>cs&insCisf cpu_iie ==EOUce(&9re DCMDne frasas__instopHf>|Hf *)& _allHf>|o(salma_p	stance-_	=i
	}
elialnewn

	seqG>pdd=nelss_gealnewn

	seqG>pd;	}
elialolde

	seqG>pdd=nelss_gealolde

	seqG>pd;	}
elialclear	seqG>pdd=nelss_gealclear	seqG>pd;	}
elialshutdown	seqG>pdd=nelss_gealshutdown	seqG>pd;	}
elialboot	seqG>pdd=nelss_gealboot	seqG>pd;Gc */
statiinst0].phys_addr = cpu_to_le32(indexfnstancIO_TIM6}
9Hrumeganmd *cmd;) * m IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEOUHrlR IGNORE_TI_anceHv,evCtrogss_ge)>FWF>FFFFe(ss_gencelss_geHvNFO);	uti_anceHv, "Ignore DCMD ti_anceHv, "IgnoreIgnox._anceHv, "IgnorregUdREr
ae-> aRegUdRErrs.
	 arynchronot;
eventcT:
oficate = ev)
{
	int ret;
ifferences between 32 @seqG>pdet;
Tstar =;
				>squensta>pdba_6OF>6class_ */
sret;Classsmd;(st>event 32 and>cmd_ *cmte = subrcr*bars.
	 f>li.
	 eventc"beyo  "(st>@seqG>pd.(at ME	in

s ev)|o(badT:
ofiins fsas_sas_dciW;(st>event ledmd;(se K6class_ */
srf>|Hf>|Hf>|Hf>|Hf>|Hf>|HregUdREr
ae->cv");

		if (nareak;
	}

	info(&inst*
	seqG>pdsta>meoust*
	class_ */
sr_word		megasas_re_val get the F_anceHv, "Ignore DCMD ti_anceHv, "Ignore (DCMD) to geune = _anceHv,evCtclass_ */
srIlurr
ae- geune = _anceHv,evCtclass_ */
srIprnstae- g;

		if (aW;(st_stan f>lipendntk(vReset);(ae-, "I),;cmegasas_(state_fclass_ */
srImd;(satipendntk(f>limd_abcludiv cmd (str>ew	if (t>liME	in

lwe lurre i_dchave.(aW;it le,;(stnlwe do
	/*have	if (mo(dotanythntk.(an oys usworde,;whicmecsi|eventc"the lurre i	if (t>liME	in

lad_f>brcr*b				nenchave(vReset);bee= subrcr*bad	if (mo.6ance->fraW;(stHold, "Igad_UT:
	_abcludiv ,;(stnlwe have(e->abf>|dNUL;(satimd_list,s.
	t	a class_ */
srI(satiad_f>|ersemd->sboysdNUL;oldsas_slurre i.as_sre-isn't	noconteCEdNULL;
Ilurr
ae-.wordor3 lass_ */
sr_word;n;
_mutex);
			bree-, "I)= meg	prnstae-.wordorFWF>IMEOUHrlHE_Tex);
			bree-, "IDCMD ti_anceH.re (Dw[1]>reset_mut(lurr
ae-.cREba_s. lassol R>csEVT_CLASStest_b_TIME;
====(lurr
ae-.cREba_s. lasso> R>csEVT_CLASStes;
	neCtrlle32[Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hf	 "ruct tateHmd;M * R3 lassct tseif by s|Hlicate =megasas_c md *cmd;
	struct m, lurr
ae-.cREba_s. lass);\dF>"IgnoreIgn	t i;
asas__insA3 lasscwhost>e>pdd allo_ad_fmadma__ad_abcludiv cmd adms__inshigs us alloc.(aW;a PROGRCMD (= -1)(wasIprnsiot;ly	ite_fMEgUdREred,;(stnlar>ewfMEgUdRate\n" ME	in

si.
	 higs u	ite_f lassedeve
 tvicebeIseitsistCE. ToeyMw_stautotatesalmy	ite_fabcluded.

		}	ite_fL*/
srI>pdba_sMdo
	/*havecI_Sh hierarchy. ToeyMw_stbit	apgre>fr alloc-_	=i
	}
_mut(prnstae-.cREba_s. lassol= lurr
ae-.cREba_s. lass) or s	 c\d(t(prnstae-.cREba_s. */
srI& lurr
ae-.cREba_s. */
sr) ^ta>meous lurr
ae-.cREba_s. */
sr)neCtrllsas__if (Presiot;ly >vOF
 tevent MEgUdRate\n" abcludess__if (lurre i.ME	in

f Noth				neMdo.s__if /\dF>"IgnoreIgn	t nce->hba_lslurr
ae-.cREba_s. */
srs|edprnstae-.cREba_s. */
srrese	IES)
prnstae-.cREba_s. lassol lurr
ae-.cREba_s. lass)sas_clurr
ae-.cREba_s. lassoedprnstae-.cREba_s. lassrese	IEx);
			bree-, "IDCabf>|
ae->(&instas_re_val t;
	u32 fw_state;
	i;
	uabf>|
gnore DCMD tisas_			
	==e used>w(Ksas_			
	==ee-, "I, 30>resetOrlR IGN_valneCtrllle32[));
	dcmd->opcode = cpu_to_le32(MR_DCMD_PD_LIST_QUERYbf>|cIO_TIFWFFFFFF"prnsiot;(t>limd_listgth = cpp "Ignore DC_val gef>|Hf>|Hoa
OUH cpu_to_le32(MR_DCMD_PD_GET_INFO);
	dcmd->s	if  getsF>yce-C	in= cpu_to_le32(sizeof(struct MR_PD_INFcpu_to_le3evCtWF>FW5nce->adapternD ti_anceHv, "evCtWF>FW5_|Hf;

		if (Prepw_stindexf
	 aen MEgUdRate\n" NULL;
ORE_TIM "Ignore (DCMD) to get the FW's c\d(s used(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u0x_cpus((u16MD) to get the FW's c\d(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignore	le16_to_cpus((u16 *)&instance->pd_info->state.ddf.pdType);
_anceHv,evCtWF>FW5_|Hf2 fw_stf>|Hf>|Hf>|Hf>cs&insCisf cpu_CTRL-EVENT-WAIT|Hf2 fw_stre (Dw[0]>pd_info->state.eqG>pdanc	case IGNORla

	seqG>pdd=nseqG>pd;Gc fw_stre (Dw[1]>pd_info->statelurr
ae-.word->state.ddf.pdType.intf;
		break;

	case DCMD_Tcpu_to_le3evCtWF>FW5HvNFOu32 fw_state;
	nceHv, "Ignor3 (DCMD) to get the FW's c\_anceHv,evCtWF>FW5_|Hf;
_mutex);
			bree-, "IKILLn prne fraE

		break;

	case DCMD_SUCCESS:
		if  getsF0;ure
	 /i	if (St pd_rOfe->mct	noconte "IKtate;e->OFgUdREr .
	 f>l. We->ca=	if (s|Hlicate =(wantc"us;e->OFgUdREr .
	 f>l,lwe have(e->abf>| lookate_fcmd.as_sre-OFgUdREr NTR_lar>ewfEVENT LOCALE_f>|HliHf>|Hf>|atis|HdNULL;

			returnee-, "IK "e32in;

		if (asn't	ndivaen MEgUdRate\n");

	idNULL;

			return
			retul(M_statedgnore DCMD ti_anceHv, "IgnoreIgnox._a_to_le32(MR_DrlHE_T_pr s - Se>c_indexNTR_lbelhyHWF>FW5l 
		fw		fw_s. 32 and>cmd_indexNTllIfe		}
fewfpr staties*md;LD/e->cmd{PDtWFfinnf
	} md-isfTAR>pd_DEV_PROPERTIES. ek.(Qint nDepth toDTSr allo. 32 andindexseif by DY) {
scwhenecsi|>ewfrlHE_T_aelvddte;e->ndivOS. 32 andnceH.f>|Hf>|||||||||- isf cpu_DEV_>pd_TAR>pd_PROP andnceH.cmd(instan|||||- indexi_listbHf;

te;te aLDa;
	e->cmd{PD= ev)))))))))))))))))))))))6_toe->cmd{PD, 1aceLD= ev)nceH.cmd(is[1]>|||||- TlHE_TID;te aLD/e->cmd{PD= ev)nceH.v->dIN|||||||||- Poifi-_;e->OFgnoreisfTAR>pd_DEV_PROPERTIES.6OFd] h6OF>6OF>6OF>ifferences between 32 @sCMDOF>OSfpr vidte;ed = _doorb *cmd;
ROCR;
s}
trucI_SAS10_non-zerotruc);
	EV;
nceHf>|Hf>|Hf>|Hf>|Hf>|H to rlHE_T_pr stancet->read_fw_status_reg(
				insta>CMD ti_aed = _doorbe*sCMD		megasas_re get the F_anceHv, "Ignore DCMD ti_anceHv, "Ignore (DCMD) to geu16frlHE_TIdc=O((s(MR_Dchannel % FR *ORE_TIMEOUdREDEV_PER_CHANnce-e+EOUHs(MR_Dio ge = cpu_to_le32(MR_DCMD_PD_GET_INFO);
	dcmd->sgl.sge32[0].phys_addr = cpu_to_le3O_TIMinstance->pd_i= cprumeganmd *cmd;) *sgRNC)DCe DCMD ti_anceHv, "Ignore DCMD ti_anceHv,MR_PD_INFcpu_to_le3tgT_pr snct;
	u32 fw_cpu_to_le3tgT_pr sn)trucRE_TIM "Ignore (DCMD) to get the FW's c>|Hf>|Hcmd(instanceRE_TIMEOtS->ldList(s(MRs c\d(s usecmd(is[1]>ed>w(KERNC)DfdrlHE_TId);\d(s used(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u0xpolles((u16MD) to get the FW's c\d(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	nceHv, "Ignore	le16_to_cpus((u16 *)&instance->p
et_case DCMD_Te INITIATE_OCR:
	TAR>pd_PROPERTIES_|Hf2 fw_stf>|Hf>|Hf>|Hf>cs&insCisf cpu_ineC>pd_TAR>pd_PROP->state.ddf.pdType.intf;
		break;

et_case DCMD_Tcpu_to_le3tgT_pr sHvNFOu32 fw_state;
	nceHv, "Ignor
et_case DCMD_Te INITIATE_OCR:
	TAR>pd_PROPERTIES_|Hf
|Hf>cs&insCisf cpu_ii-___LINEb9>/DReturn 0 for s c\d(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u32 fw_state;
	i;
	u8 max_A cgHf>sas_			Hf>|Hf>cs&insCisf cpu_iie;vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOF>vOnc
		}
	rroy ancResetTIMEOUdRE_TI:)) nc
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf>linc
		}
	}
9Hf>lineCtrlReset);
		}
	}
9Hf FW's controller PD
 * list structure.  TT|||||his information is mainly used to find out SYSTEM
 * supported by the FW.
 */
static int
megasas_get_pd_list(struct megasas_instance *instance)
{
	int ret>|Hf	 " 0, pd_index = 0;
	struct megasas_c md *cmd;
	struct mint
megasas_get|Hf>gasas_g
|Hffault * sE

		break;

	case DCMD_SUCCESS:
		iHrsta_HOF>v!E_TIMEOre DCMDn.sge32[0].phys_addr = cpu_to_le3O_TIMRNC)DCe6}
9Hruct >OFgnore allo_t megasas_md *cmd;
	struct mOUMEteHv, "Ignore DCMD ti_anceHv, "Ignorr =;
Tae-> aSubrcr*barse->f>lidue				DY) {

loadx = 0d] h6OF>6OF>6OF>ifferences between 32/f>|Hf>|Hf>|v, "Ignorr =;
Tae- to alloc mem in get_ld_list\n");
		megto alloc mem inevCtrogss_geeeli geune = _anceHv,evCtclass_ */
srIllass_ */
srnc;

		if (Geen(strlthe _T>squensta>pdba_s6}
9HCEdNULL;
cRE_TIM&elinct;
	u32 fweli		u3ct MR_f>|Hf>|H to seqG>pdtDCMD_SUCCE&eli		
saRNC)DCe 1gnlereate_fragUdREr f>liNTR_lFW_, __Lthe _T>squensta>pdba_splt;
1 NULL;
=lass_ */
sr.cREba_s.	}
9rvDMA* sizeHlass_ */
sr.cREba_s. */
srI=  R_EVT_LOCALE_AprogrHlass_ */
sr.cREba_s. lassoed R_EVT_CLASStest_bHv, "Ignoref>|Hf>|HregUdREr
ae->x_A cgHf>sas_IMEOUHrlHE_Teli.newn

	seqG>pdy dc1>sas_Hlass_ */
sr.word->s ti_anceHv, "Ignor>o_attmman-	Attmmae coni_lDY) {

e->SCSI mid-ltya_d] h6OF>6OF>6OF>ifferences between 32/f>|Hf>|Hf>|v, "Ignor>o_attmma to alloc mem in get_ld_list\n");
		megto alloSd = Ht)an*ht)an\n");

	retuht)agnlereate_fExOUdR_pNORmettersME	i

te;by SCSI mid-ltya_dNULL;
ht)atuuniqF>vt>n\n");

	retuuniqF>vt>;;
ht)atucanG	int d\n");

	retu			Ged = ERNE;gaht)atuoni_vt>n\n");

	retu[e !Gt>;;
ht)atusg rl		}_cmd = ");

	retu			G>pdev->_interrupts)
		retE_TIMEOUdR;
	/*kframe to each			Geectors_Aa_	ME	d\nRE_TIMEOUdREu_iTOREOtEEE;ngasas_instmegasas_(st mHfutadpNORmette  allo_, __			Geectors_can_be megdled\ERIES)
			Geectors_&&_			Geectors_<n");

	retu			Geectors_Aa_	ME	kframe to each			Geectors_Aa_	ME	d\n			Geectors;sece->hba_lmS)
			Geectorsfo(&insta_H->reg_set->inbound_doorbellsas_c		max_wait = MEGASAS_1078GEN2_TIME;
;
>reg_set->inbound_doorbellsas_c		max_wait = MEGASAS_0079GEN2)) or s	;
>			Geectors_<\nRE_TIMEOUdREu_iTORE)neCtrllame to each			Geectors_Aa_	ME	d\n			Geectors;se	t nce->hba_ls3 cmd->frame,
				      cmd->f (			Geectors_ate;
	 b n>O0IO_TIF"as_s<\nt >( __< 1MB_, __GEN2anceme_dma_)megasas_c");

	retu			Geectors_Aa_	ME	k gef>|Hf>|Hoa
OUht)atu			Geectors_\n");

	retu			Geectors_Aa_	ME	;;
ht)atuc9HfAa_	lunn=ORE_TIMEODEFAULTeIMEO->loLUN;OUht)atu			Gchannel \nRE_TIMEOUdRECHANnceItat1;OUht)atu			Gt>n\nRE_TIMEOUdREDEV_PER_CHANnce;OUht)atu			Glunn=ORE_TIMEORci->UN;OUht)atu			Gc9Hfce->pd16;ngasas_insN:
ofya(strucd-ltya_aab
	nc(str>ewanceme_dma_led\ERIES)
ed = rea_uctu(uctur= cpu_to_le32(MR_DCMDineCtrle32[0].phys_addr = cpu_to_le3FWF>D_LIST_QUERYdd_ht)an6}
9Hruct megasas_md *cmd;
	struct megasaRNC)DCe DCMDEVHf>}c>|"IgnoreIgn}
f>|Hf>|Hf>|Hf>|Hf>|Hrol_ DCMERNC to allo IGNIMEloc ev		meg
		if (tan ourtframe_dma_pMw_stcapnstadof pEr.
	t				64-bits= sled\ERIES)
tS-= s64)hba_lmS)
e (Dsel_ DCMERNC TIMEOUancytIToMASK(64))v!E_0)= meg	lmS)
e (Dsel_ DCMERNC TIMEOUancytIToMASK(32))v!E_0)sas_cgo
		fnstGsel_ DCMERNC_get|Hfance->hba_lmS)
e (Dsel_ DCMERNC TIMEOUancytIToMASK(32))v!E_0)sas_go
		fnstGsel_ DCMERNC_ge};

		if (EEOu_abs_seHCMD> *)& to allumds w_staance->cf id 32-bit	if (zeof(s= NULL;

ce-e (Dsel_IMEOUdRE_T_ DCMERNC TIMEOUancytIToMASK(32))v!E_0)e frasanT(st32bits= s ERNC.as_s32tbit CMEOUdRE_T  DC ERNC.i
	}
_mut!e (Dsel_ DCMERNC TIMEOUancytIToMASK(32))sas_&& !e (Dsel_IMEOUdRE_T_ DCMERNC TIMEOUancytIToMASK(32)))a_ls3 cmd->fra    cmd->f (semd32bits= s ERNCIO_TIF"as_s32tbit IMEOUdRE_T ERNCgth = cpreak;
		go
		fnstGsel_ DCMERNC_ge};>|"IgnoreIgnOfnstGsel_ DCMERNC:
ERNC)DCe16rn -ENODEV;
		}

	rol_pu_ii-___LINE aSemdpu_ii-_ _LIN
nce_			S>|Hf>|Hf> rame_dma_pMcan_be dividte;innce_			4 e->cgories- >e>pdd R_SYSTEM
_TYPEe fce_			UHrMDReturn 0>pd1,fce_			UHrTHUNDERBOLTeturn 0>pd2,fce_			UHrte;
DER_ urn 0>pd3,fce_			UHrVENTURAeturn 0>pd4,fce_			UH}; ev)
{
	int ret;
ifferences between 32 _allocet;
FI_MB he pool itice->faFI_Mates the rol_pu_ii-___LIN to alloc mem in get_ld_list\n");
		megtc
		}
	reg_set->inbound_doorbneCtr_pd_l		max_wait = MEGASVENTURA:tr_pd_l		max_wait = MEGASHARPOON:tr_pd_l		max_wait = MEGASTOMCAT:tr_pd_l		max_wait = MEGASVENTURAe4PORT:tr_pd_l		max_wait = MEGASCRUS
DER_4PORT:tr

			returneak;
		case Kl VENTURAeturn 0;Hf>gasas_gr_pd_l		max_wait = MEGASFUSION:tr_pd_l		max_wait = MEGASPLASMA:tr

			returneak;
		case Kl THUNDERBOLTeturn 0;Hf>gasas_gr_pd_l		max_wait = MEGASte;
DER:tr_pd_l		max_wait = MEGASINTRUDER:tr_pd_l		max_wait = MEGASINTRUDER_24:tr_pd_l		max_wait = MEGASCUTLASSt52:tr_pd_l		max_wait = MEGASCUTLASSt53:tr_pd_l		max_wait = MEGASFURY:tr

			returneak;
		case Kl te;
DER_ urn 0;Hf>gasas_grHffault h/inFe aCMD>oys uss>|Hf>|Hf> rame_dma_pMoid me used>w(Keak;
		case Kl MDReturn 0;Hf>gasas_gr}n}
f>|Hf>|Hf>e->faf>|v, "IgnorCMD) tFP_KEoc(szeoENOMEM;
	}

	memset(instance->cmd_list, 0g_set->inroduca_ cee (DCMD) to get the FWF>FWF>FWF>FWF>DHrlR IGu_FR>FWF>F tim& 0g_set->inroduca_HvNFOucase IGNOR>MEOuma_(cee (DCMD) to get the FWF>FWF>FWF>FWF>DHrlR IGu_FR>FWF>F tim& 0g_set->i>MEOuma_sizeof(struc*/
	for (inroduca_ (str 0g_set->i>MEOuma_neCtrle32[0].phys_addr = cpu_to_le3FWF>D_LIST_QUERYance->cmzeof(st64),nroduca_,anceOuma_sed>w(KERNC)DCe 1Hf>}c>|* 0g_set->inroduca_ cesize*case IGNOR>MEOuma_(cesize"IgnoreIgnox._anceHv, "IgnorCMD) tooc(szeon-	Aance->cmHsrfnceme_dma__zeof(st64),c pd_ *)&fce_			to allumds whicm w_stvice cmm = across MDRfce_			e *dcmds as_s	instaiafferen = ce_			Fe ace->ce, stafferen ,RYance->cmnroduca_ as_
f _			HMEOuma_(stances.nsta(	instaiafferen ,RYance->c
f _			zeof(st64),	instaiizeof(s= ev)
{
	int ret;
ifferences between 32 _allocet;
0t64),re DCMD 32/f>|Hf>|Hf>|v, "IgnorCMD) tooc(szeoENOMEM;
	}

	memset(instance->cmd_list, 0g_set->iMEplyG	apnorkzCMD) get the unnstains 0,)f (>rlHE_EOUHrO_TIFWFFFFF= instance->c	ES)
c*/
	for (iMEplyG	aps	if  getsF>yce-C	in= tc
		}
	reg_set->ieak;
		case neCtr_pd_lMDReturn 0:a_lmS)
	 "IgnorCMD) tFP_KEoc(szeoE	}
9Hf>lin;
		go
		fnst;Hf>gasas_gr_pd_lVENTURAeturn 0:tr_pd_lTHUNDERBOLTeturn 0:tr_pd_lte;
DER_ urn 0:a_lmS)
	 "IgnorCMD) tax_cmd; i++) {E	}
9Hf>lin;
		go
		fnst;Hf>gasas_gr};>|"IgnoreIgn	fnst:OCkORE_([i] = kmalMEplyG	aps;t, 0g_set->iMEplyG	apnorn progr  getsF>yce-C	inn -ENODEV;
		}

	ORE_TIoc(szeon-	FRE_,	instaiizeof(st64),	instaie *dcmds as_
f _			nroduca_,anceOuma_(stancest64),ce->e *dcmds6OFd] h6OF>6OF>6 ->v
ifferences beOF>6OF>66OFd] e pool itice->faFI_Mates the ORE_TIoc(szeoENOMEM;
	}

	memset(instance->cmd_list,kORE_([i] = kmalMEplyG	aps;t, ce-pts)
		reteak;
		case Kll /DReturn 0 fba_lmS)
 0g_set->inroduca_-sas_ IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEODHrlR IGu_FR>FWF>F meou*/
	for (inroduca_,FWF>F meou*/
	for (inroduca__hstanceS)
[i] = kmallMEOuma_nsas_ IGNORE_TIMEOUdRE_TIMEOUHrlHE_TIMEODHrlR IGu_FR>FWF>F meou*/
	for (ilMEOuma_>FWF>F meou*/
	for (ilMEOuma__hstanance->hba_ltes the ORE_Tax_cmd; i++) {E	}
9Hf>li_gr}n}
f_anceHv, "Ignor))obr_on6 ->		m hotplug>e>C(stpoifid] h6pCMDOF>		m _doorbeto allumdd] h6Od:s_c		m OUHdof s>|Hf>|Hf>hotplugg stafferen 32/f>|Hf>|Hf>|v, "Ignor))obr_on6 to allo IGNIMEloc ev>FWF>FFFF  cm);	dcmd->c IGNIMEorb	idancd		megasas_val,(et);egto alloSd = Ht)an*ht)a DCMD ti_anceHv, "set(instance->cmd_ geu16fnceme_d, "IgnOUga	ROsemdMSI-X_anc(strkdump kernel LL;

ce-IGLL;

	INIT_ fba_let)(cee (Dfw_s_capnsil !y(TIMEOD		maCAP = MMSIXstanceS)
et)neCtrll IGNreaMEOUnfig_word(TIMEODet)(+D		maMSIXCFLAGSre.  TT|||||&nceme_d->c		IES)
nceme_d,&D		maMSIXCFLAGS_ENABLEneCtrllle32[d->fra    cmd->f (IGLL;
				MSI-Xgth = cpp  IGNwriteEOUnfig_word(TIMEOFWF>F timmmmmet)(+D		maMSIXCFLAGSre.  TT>meous lceme_d,&e.  TT>meous ~		maMSIXCFLAGS_ENABLEn gef>|Hf>|Hoa
OU
		if (P	m pMEpp			:eennsta _doorbetemdbus ERNrenntk(vs_s DC ERNC NULL;
rval t; IGNennstaTIMEorb	zeoEp(MRs c\d
ce-IvalneCtrl"Ignore val ge}* m IGNLL;
ERNrenEp(MRs c\d
ce-f>|Hf>|Hrol_ DCMERNC p(MRsn	ifgo
		fnstGsel_ DCMERNC_gOUht)a_toed = uctu	CMD) gl	}

	memtemplLL_>FWF>FFFF FFmd_timeout_ocr
				MEG	}
9Hf>lineof(strucuctuy ance->unload ||
			test_bit(    cmd->f (sd = uctu	CMD) xfnstangth = cpgo
		fnstGCMD) t
e->cmd_ gee
	 */
	for estenD ti_anceHv, "set(instan)ht)atuht)a *)&;MR_PD_INFcpu_to_lnct;
	u32 fw_cpu_to_l_|Hf2atomicstrophys_addr = E_TIGLL;
no_e (DCSAS10,i0stanMEOUHrlHE_TIME t; IMEresetes the rol_pu_ii-___LIN PD_GET_INFO);
	dc, "IgnorCMD) tooc(szeoE	}
9Hf>lin;
	go
		fnstGCMD) t DCMsta;
	 /in rash dump fea an arel->cf id != MFste\n"LL;

			returndrvTsta	ifdex(cesize
			returndrvTsta	CMD) xcesize
			returnurash_dump_E_TIMEOUdRMAesize
			returnurash_dump_appTIMEOUdRMAesize
			returnE_Turash_tweenMAeUNAVAILABLE DCMp		_ */k_ie !phys_addr = urashdump_ */k)ize
			returnurash_dump_buftuen progsetes the OF>v_wait
ae->(&size
			returnElag;
	/*>(&size
			returnME t;n progri);

	retu[vOF>pend_do
nt the Fatomicstrophys_addr = adpMEcoveryNORE_TIMEOHBA_OPERATIONAL)ogri);

	retu[v_imr, "IgnOUcpu_to_le3evCtWF>FW5(cee (DCMD) to get the FTIMEOFWF>F timmmmd_timeout_ocFWF>F t	eoutes the evCtWF>FW5_OFWF>F timmm&cpu_to_le3evCtWF>FW5sizeof(struc*/
	for (ievCtWF>FW5_ ance->unload ||
			test_bit(    cmd->f (_LIST_QUERYance->cmzeof(st64),IO_TWFFFFFF"event WF>FW5(to allumdgth = cpgo
		fnstGCMD) t DCMsta;
d}c;
_mut!IGLL;

	INIT_ fba_l*/
	for (ie->cmdss_geHbuftuee (DzCMD) to get the FTIMEOFWF>F e INITIATE_OCR:
	ineCSYSTEM_info-OFWF>F &*/
	for (ie->cmdss_geHhstanceS)
!*/
	for (ie->cmdss_geHbuf)a_ls3 cmd->frame,
				      cmd->f (Ca
	/*aance->cmo->cmd{s_geestancece->resean internalE_Ts_gencee (DCMD) to get the FTIMEOFWF>e INITIATE_OCR:
			cinfo-O= cpu_to_le32(ss_geHvNFO);(struc*/
	for (in(ss_ge)a_ls3 cm0].phys_addr = cpu_to_le32(instancUERYancemzeot64),n(ss_gece->resean internaltgT_pr sncee (DCMD) to get the FTIMEOFWF>e INITIATE_OCR:
	TAR>pd_PROPERTIES_O= cpu_to_le3tgT_pr sHvNFO);(struc*/
	for (itgT_pr sna_ls3 cm0].phys_addr = cpu_to_le32(instancUERYancemzeot64),tgT_pr sce->resean internalurash_dump_buftuee (DCMD) to get the FTIMEOFWF>F tCRASHyancyte)_ FW',ol\99F &*/
	for (ilrash_dump_hstancetruc*/
	for (ilrash_dump_buf-sas_3 cm0].ph    cmd->f (Ca
	/*aance->cmF

	dcmdcIO_TIF"crash dumps= s stancece->reoa
OU
		if (Ie != MFIe	 */ks as_s	int pdNULL;
);
	_IGN|_HEADphys_addr = u9HfAo_d->c	);
	_IGN|_HEADphys_addr = Dfd inalTIGLL;
pendntk_q>reseatomicstrophys_addr = E_Tout_adddntk,0intol[iit
wait	int sieaMphys_addr = DfdGc9Hfwait
q)ogri)it
wait	int sieaMphys_addr = abf>|
gnofwait
q)ogDCMp		_ */k_ie !phys_addr = FP_KAo_d_ */k)izeMp		_ */k_ie !phys_addr = hba_ */k)izeMp		_ */k_ie !phys_addr = outeam_ */k)izeMp		_ */k_ie !phys_addr = complee\n"f */k)izsetu+) _ie !phys_addr = IGLL;
tu+) )trucu+) _ie !phys_addr = hba_tu+) )trOU
		if (Ie != MFIe	P	m rel->cf as_smisc_pNORmetterdNULL;

			returnht)a_toht)a DC");

	retuuniqF>vt> t; IME->bus->>pdba_s<< 8 |     cmd->fnogri);

	retu[e !Gt>n=ORE_TIMEODEFAULTe);
	_IDize
			returnuoc(ss_gencen progf
|Hf>cs&insCisf cnbound_doorbell 		max_wait = MEGASAS_RESET_WAIT_TIME;
s&insCisf cnbound_doorbell 		max_wait = MEGASAS_RES1T_WAITin	if");

	retuElag;
	/*>(&1ogsetes the dbg_lvl>(&size
			returnElag>(&size
			returnunloadx the F
			returnla

	
	m n=Osize
			returndI_RESEOce->fCoc(ROsemd the F
			returnUnevenSpanState;
MA"IgnOUcmutex);
			break;
		case KILL_ADAPTER:
	fba_l);
	_WORKphys_addr = work_ie !,ates the Ox_cmd;lResw	k gef);
	_WORKphys_addr = lrash_ie !,ates the Ox_cmd;lrash_dump_w	k ge */
stati);
	_WORKphys_addr = work_ie !,apr AS10sas_teareGchange_w	k gOU
		if (Ie != MFIe	ce->F

	dcmdled\ERIES)
	ceHv, "se !Gfw 
static in	ifgo
		fnstG[e !GFP_) 	 *ce-pts)
		retME	in

orId)hba_lmS)
pts)
		retPlasmaFW111fo(&inspts)
		retvf affilite\n"_111a= cpp  IGNCMD) to get the FTIMEO e INITIATE_OCR:
	LD_VF_AFFoLIATION_111_OFWF>F timmm= cpu_to_le3vf affilite\n"_111_hstanccetruc*/
	for (ivf affilite\n"_111)trllle32[dcmnph    cmd->f (Ca
	/*aance->cmIO_TIFWFFFFFF"zeof(st64),VF affilite\n" stancece->reot nce->hba_ls*/
	for (ivf affilite\n"a= cpp  IGNCMD) to get the FTIMEOFWF>F timmm=(Rci->ldList[laC	brdpoold*sas_			FF FFmd_timeout_ocr:
	LD_VF_AFFoLIATION_OFWF>F timmm= cpu_to_le3vf affilite\n"_hstanccetruc*/
	for (ivf affilite\n")trllle32[dcmnph    cmd->f (Ca
	/*aance->cmIO_TIFWFFFFFF"zeof(st64),VF affilite\n" stancece->reot ure
	 /i	if (St pd_set(instain	P	m es btween ed\ERIe (Dsel_ rv *)&(TIMEOUi			retuk gOU
		if (Addconi_lnceme_dma__e->	ceHv, "mgmtss_gento allumd esI(satiat	if (can_be exHf>|Hf>e->	anageme i.a|Hlicate =sdNULL;
cReHv, "mgmtss_ge.to ge++;;
cReHv, "mgmtss_ge.i			retu[cReHv, "mgmtss_ge.			Gtfdex]_\n");

	re;;
cReHv, "mgmtss_ge.			Gtfdex++;;lereate_fragUdREr NTR_lSCSI mid-ltya_dNULL;
ES)
	ceHv, "so_attmma 
static in	ifgo
		fnstG[o_attmmagnOUcpu_to_le3unloadx tsizereate_fTriggEr SCSI 
		scan_ourtDY) {sdNULL;
sd = scan_uctu(uctuk gOU
		if (Ie != >cmf>li(Arynchronot;
Event N:
oficate =)dNULL;
ES)
	ceHv, "s =;
Tae- 
static in ance->unload ||
			test_bit(    cmd->f (s =;
vaen fnstangth = cpgo
		fnstGs =;
Tae-reoa
OU
	(Geenlurre i.SR-IOV LD/VF affilite\n" LL;

ce-pts)
		retME	in

orId)a_ltes the neCtrlRvf affilite\n">e DCMD ti_1)Hv, "IgnoreIgn
fnstGs =;
Tae-:
fnstG[o_attmma:;
cReHv, "mgmtss_ge.to ge--;;
cReHv, "mgmtss_ge.			Gtfdex--;;
cReHv, "mgmtss_ge.i			retu[cReHv, "mgmtss_ge.			Gtfdex]_\nn progsei);

	retu[e;

	re
/*FI_RESET_FLAGS,
					&in_tes the EL, &cm_irqv_S,
					&inOUcmutex);
			break;
		case KILL_ADAPTER:
	
 sE

		breakleaat_er PD
 * list se;vOF>vOF>v	 */
	dmreleaat_FP_E* list se;vOerrupts)
		retnsixsvectorsool\ IGNORE_T
rqsvectorsIMEOUHrlHE_TIMEancfnstG[e !GFP_:
fnstGCMD) t DCMsta:OCerrupts)
		retevCtWF>FW5_
s_ IGNORE_TIMEOUdRE_TITIMEOUHrlR IGNORE_TI_anceHv,evCtWF>FW5_OFWF>Fmeou*/
	for (ievCtWF>FW5nFWF>Fmeou*/
	for (ievCtWF>FW5sizeof(stru*/
	for (in(ss_ge)a_l IGNORE_TIMEOUdRE_TITIMEOUHrlR IGNORE_TI:
			cinfo-OFWF>F */
	for (in(ss_geOFWF>F */
	for (in(ss_geHvNFOuctru*/
	for (itgT_pr sna_l IGNORE_TIMEOUdRE_TITIMEOUHrlR IGNORE_TI:
	TAR>pd_PROPERTIES_OFWF>F */
	for (itgT_pr snFWF>F */
	for (itgT_pr sHvNFOutes the ORE_TIoc(szeoE* list se;vOsd = uctu	puu(uctuk gfnstGCMD) t
e->cmd_:OfnstGsel_ DCMERNC:
E IGNII_RESET_doorbEp(MRs c\dRNC)DCe DCMDEVHf}
f_anceHv, "Ignorflush_cmmae> aRe	in

siFW 
		flushaCMD>i
s cmmaes ev)
{
	int ret;
ifferences between 32e pool itself
	 */
	dmflush_cmmae to alloc mem in get_ld_list\n");
		megto alloc mem in "Ignore DCMD ti_anceHv, "Ignore (DCMD) to gOuctruatomicsreaMphys_addr = adpMEcovery)Kll /E_TIMEOHW_CRITList[ERRORs	if  gets ge = cpu_to_le32(MR_DCMD_PD_GET_INFO);
	dcmd->s	if  gets ge v, "Ignore DCMD ti_anceHv,MR_PD_INF "Ignore (DCMD) to get the FW's c\d(s used(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u0x_cpus((u16MD) to get t_cpus((u16d(s used>w(KERNC)Dfd i;
	u8 max_waNON's c>|Hf>|Hate;
	nceHv, "Ignore	le16_to_cpus((u16 *)&instance->pd_cpus((u16f>|Hf>|Hf>|Hf>cs&insCisf cpu_CTRL-CACHE_FLUSHs c>|Hf>|Hcmd(instanceRR_FLUSH_CTRL-CACHE | RR_FLUSH_DISK-CACHEu3ct MR_f>|Hf>|H_state;
	i;
	u8 max_A cgHf>|Hf>|Hf>cs&insCisf cpu_iieFWF>!E_TIMEOre DCMDneCtrle32[0].phys_addr = cpu_to_le3FWF>DRNC)DCe6}
9Hruct meganmd *cmd;
	struct megasaRNC)DC;nt i;
	uti_ancak;

	case DCMD_SUCCESS:
		 ti_anceHv, "Ignorrhutdown	nceme_dma__-	InMD ti_siFW 
		rhutdownconte ceme_dma_lev)
{
	int ret;
>ifferences between 32 @f>|Hf>et;
>Shutdown/Hib inaen 32e pool itself
	 */
	dmrhutdown	nceme_dma_tancet->read_fw_status_reg(
				insta>C	, *
	f>|Hf>		megto alloc mem in "Ignore DCMD ti_anceHv, "Ignore (DCMD) to gOuctruatomicsreaMphys_addr = adpMEcovery)Kll /E_TIMEOHW_CRITList[ERRORs	if  gets ge = cpu_to_le32(MR_DCMD_PD_GET_INFO);
	dcmd->s	if  gets ge _mutex);
			bree-, "I)F>v	 */
	dm_state;
	i;
	uabf>|
gnore DCMD tisas_ex);
			bree-, "I|Hf>cs&insCisf cpu_iie;vOerrupts)
		retnap_updareGc"I)F>v	 */
	dm_state;
	i;
	uabf>|
gnore DCMD tisas_ex);
			brnap_updareGc"I|Hf>cs&insCisf cpu_iie;vOerrupts)
		retjboorseqGc"I)F>v	 */
	dm_state;
	i;
	uabf>|
gnore DCMD tisas_ex);
			brjboorseqGc"I|Hf>cs&insCisf cpu_iie;ve v, "Ignore DCMD ti_anceHv,MR_PD_INF "Ignore (DCMD) to get the FW's c\d(s used(s used>w(KERNC)Dfd i;
	u8 max_wait;
	u0x_cpus((u16MD) to get t_cpus((u16d(s used>w(KERNC)Dfd i;
	u8 max_waNON's c>|Hf>|Hate;
	nceHv, "Ignore	le16_to_cpus((u16 *)&instance->pd_cpus((u16f>|Hf>|Hf>|Hf>cs&insCf>|Hf>	u3ct MR_f>|Hf>|H_state;
	i;
	u8 max_A cgHf>|Hf>|Hf>cs&insCisf cpu_iieFWF>!E_TIMEOre DCMDneCtrle32[0].phys_addr = cpu_to_le3FWF>DRNC)DCe6}
9Hruct meganmd *cmd;
	struct megasaRNC)DC;nt i;
	uti_ancak;

	case DCMD_SUCCESS:
		 ti#ifdef CONFIG_PMi_anceHv, "Ignorruspend_-	DY) {

ruspend_e>C(stpoifid] h6pCMDOF>		m _doorbeto allumdd] h6tweenOF>		m powEr ou >cm
		ruspend_r
	ninrf>|Hf>|Hf>|Hf>|Hf>|Hf>|Hruspend to allo IGNIMEloc ev> pmszessag cbetween		megto alloSd = Ht)an*ht)a DCMD ti_anceHv, "set(instance->cmd_ g	 */
	for est IGNgel_ rv *)&(TIMEegasht)an\n");

	retuht)agne
			returnunloadx the OU
	(Shutdown SR-IOV heartbeseH(ima__LL;

ce-pts)
		retME	in

orId_&&_!*/
	for (iekipGheartbeseT(ima___dln.sge3lT(ima__ryncphys_addr = or*ovsheartbeseT(ima_NFO);	uti_ancflush_cmmae S,
					&in_tes the rhutdown	nceme_dma_tDCMD_SUCCEisf cpu_HIBERN}
	}SHUTDOWNk gOU
	(canceD>(st>e3layedsworasas_(sissworasstTllIins	int _LL;

ce-pts)
		retMElILLn prne fraMD ti_anceHv, "ee-,event *ME t;pts)
		retMEgasacanceD_e3layed_work_ryncphu_tohotplug_workstance			returnME t;n progr i;
tRNClel_e FWphys_addr = Ds	caRNClel) * m IGNsel_ rv *)&(ys_addr = cpu_OUi			retuk gei);

	retu[e;

	re
/*FI_RESET_FLAGS,
					&inn_tes the EL, &cm_irqv_S,
					&inOUcmutex);
			brnsixsvectorsool\ IGNORE_T
rqsvectorsIMEOUHrlHE_TIMEanc m IGNsave_teare(TIMEegas IGNII_RESET_doorbEp(MRs c\d IGNsel_powEr_teare(TIME,t IGNchoose_teare(TIME,ttween	eHv, "IgnoreIgnox._anceHv, "IgnorreOuma-FFFFFFDY) {

reOuma_e>C(stpoifid] h6pCMDO)))))))))))))))		m _doorbeto allumdd] Hf>|Hf>|Hf>|Hf>|Hf>|HreOuma to allo IGNIMEloc ev		megasas_val;egto alloSd = Ht)an*ht)a DCMD ti_anceHv, "set(instance->cmd_ geasas
rqsd(s used		maIRQ_LE_TCY g	 */
	for est IGNgel_ rv *)&(TIMEegasht)an\n");

	retuht)agne IGNsel_powEr_teare(TIME,t		max0egas IGNennstaTwake(TIME,t		max0,i0stan IGNre

ore_teare(TIMEegaOU
		if (P	m pMEpp			:eennsta _doorbetemdbus ERNrenntk(vs_s DC ERNC NULL;
rval t; IGNennstaTIMEorb	zeoEp(MRs c\d
ce-IvalneCtrl3 cm0].ph    cmd->f (Ennsta _doorbefnstangth = cp"Ignore val ge}* m IGNLL;
ERNrenEp(MRs c\d
ce-f>|Hf>|Hrol_ DCMERNC p(MRsn	ifgo
		fnstGsel_ DCMERNC_gOU
		if (Ie != MFIe	ce->F

	dcmdled\ERseatomicstrophys_addr = E_Tout_adddntk, 0egaOU
		if (We exH0 s>onteCE ou >cm
		be it;
YdNULL;
ES)
	ceHv, "transie\n"_>cseset)Fcpu_to_lnctsn	ifgo
		fnstGMEet)free(i gOU
	(NowfME-ennsta MSI-X_LL;

ce-pts)
		retnsixsvectorsohba_lmrqsd(s used		maIRQ_MSIXtancetrusmp_aff[e !yNennstaeFWF>mrqsd(s us|ed		maIRQ_AFFoNITY ge}*
rval t; IGNCMD) t
rqsvectorsIMEOUHrlHE_TIME,c1>sas_FWFFFFpts)
		retnsixsvectors ?sas_FWFFFFpts)
		retnsixsvectors :c1>s
rqsd(s ue;vOerrurval <_0)sasgo
		fnstGMEennstaTnsixresetes the rolup_MEplyG	ap_S,
					&inOUcmutex);
			break;
		case KILL_ADAPTER:
	e fraE

		breaksel_MEplyGEL,c/
static int
mES)
	ceHv, "soc"se !Gfr PD
 * list sefo(&instes the ORE_TIGNORE_TIMEOUT;
instes the ORE_TIGNO_er PD
 * list se;vOifgo
		fnstG[e !GFP_) f>|Hf>mS)
ctes the gL;
ERpmd->fr
static in	ifetes the rync
ERpmd->fr
static i;anance->hba_l* 0g_set->inroduca_ cesizee*case IGNOR>MEOuma_(cesizet MR_f>|Hf>|H_state[e !GFP_E	}
9Hf>lin;
		go
		fnstG[e !GFP_) f nct MR_f>|Hf>|HMR_DCoc(ss_geE	}
9Hf>liv!E_TIMEOre DCMDn.sggo
		fnstG[e !GFP_) 	 tRNClel_ie !phys_addr = Ds	caRNClel,u*/
	for (i[e;

	re
/*aRNClel,O_TWFFFF unnstainslong)S,
					&inOUcmutex);
			brnsixsvectors ?sas_tes the rolup_irqvTnsixFcpu_to_lncts :sas_tes the rolup_irqvTioapicE	}
9Hf>lin;
	go
		fnstG[e !GFP_) 	 ga	RO-launch SR-IOV heartbeseH(ima__LL;

ce-pts)
		retME	in

orId)hba_lmS)
ctes the rr*ovs] =;
Theartbese>e DCMD ti_0)-sas_tes the r =;
T(ima_>e DCMD tisas_		meou&*/
	for (irr*ovsheartbeseT(ima_isas_		meoutes the rr*ovsheartbeseT_stdlH_isas_		meouRE_TIMEOIRIOV_HEARTBEstateTERVAL_VF);setreakhba_ls*/
	for (iskipGheartbeseT(ima___dl * 1Hf>ifgo
		fnstG[e !GFP_) f>|Hfe
	 */
	for (i[e;

	re
/*ennstaT_FLAGS,
					&in_tes the rolup_jboor	ap_S,
					&inUcpu_to_le3unloadx tsizOU
		if (Ie != >cmf>li(Arynchronot;
Event N:
oficate =)dNULL;
ES)
	ceHv, "s =;
Tae- 
static intrle32[0].phys_addr = cpu_to_le3 "S =;
vf>li.nstangth = , "IgnoreIgn
fnstG[e !GFP_:
Cerrupts)
		retevCtWF>FW5_
s_ IGNORE_TIMEOUdRE_TITIMEOUHrlR IGNORE_TI_anceHv,evCtWF>FW5_OFWF>F*/
	for (ievCtWF>FW5nFWF>F*/
	for (ievCtWF>FW5sizeof(stru*/
	for (in(ss_ge)a_l IGNORE_TIMEOUdRE_TITIMEOUHrlR IGNORE_TI:
			cinfo-OFWF>F */
	for (in(ss_geOFWF>F */
	for (in(ss_geHvNFOuctru*/
	for (itgT_pr sna_l IGNORE_TIMEOUdRE_TITIMEOUHrlR IGNORE_TI:
	TAR>pd_PROPERTIES_OFWF>F */
	for (itgT_pr snFWF>F */
	for (itgT_pr sHvNFOOutes the ORE_TIoc(szeoE* list se;vOsd = uctu	puu(uctuk gOfnstGsel_ DCMERNC:
fnstGMEet)free(i:OfnstGMEennstaTnsix:

E IGNII_RESET_doorbEp(MRs c\dRNC)DCe DCMDEVHf}
#F>vOF#WFfinnv, "Ignorruspend	n prF#WFfinnv, "IgnorreOuma	n prF#endnf
f>|Hf>|Hf>e->faf>|Hf>|Hf>|Hwait
for_pu_ii-__ostate\n"alENOMEM;
	}

	memset(instance->cmd_list, 0t(wait_
	m n=ORE_TIMEORE*/
	WAITruct f (2 geasas
 gOuctruatomicsreaMphys_addr = adpMEcovery)Kll /E_TIMEOHW_CRITList[ERRORs	if  getsthe OU64),(ix tsi i <_wait_
	m i i++)hba_lmS)
atomicsreaMphys_addr = adpMEcovery)	ll /E_TIMEOHBA_OPERATIONAL)FWF>gasas_g
|lmS)
c(ix%ORE_TIMEORE*/
	NOTait =eTERVAL))a_ls3 cmT:
orbEhys_addr = cpu_to_le3 "waitntk(64),c eme_dma__aksel 
		fwnishce->reseamsleep(1000)) f nct MR_atomicsreaMphys_addr = adpMEcovery)K!l /E_TIMEOHBA_OPERATIONAL)eCtrl3 cmd->frame,
				      cmd->f (%sH(ima tateHwh	}
	waitntk(64),HBA;e->OFcover.megasas_md *cmd; = cp"Ignore1Hf>}c>|"IgnoreIgnox._anceHv, "IgnorWF>Fch_on6 ->		m hot"un"plug>e>C(stpoifid] h6pCMDOF>		m _doorbeto allumdd] e pool itself
	 */
	dmWF>Fch_on6 to allo IGNIMEloc ev		megasasi;egto alloSd = Ht)an*ht)a DCMD ti_anceHv, "set(instance->cmd_ geMD ti_aax_cmd; i++) { *ax_cmd;t, *
	porseqGERpmsz g	 */
	for est IGNgel_ rv *)&(TIMEegasht)an\n");

	retuht)agne	instai\n");

	retuIoc(s i++) {e OU
	(Shutdown SR-IOV heartbeseH(ima__LL;

ce-pts)
		retME	in

orId_&&_!*/
	for (iekipGheartbeseT(ima___dln.sge3lT(ima__ryncphys_addr = or*ovsheartbeseT(ima_NFO);errupts)
		retE_Turash_tweenM!AeUNAVAILABLE)F>v	 */
	dmORE_Tuctu	urash_stanceE* list se;vOsd = remove_uctu(");

	retuht)a&inUcpu_to_le3unloadx t1 c\d
ce-f>|Hf>|Hwait
for_pu_ii-__ostate\n"alE	}
9Hf>lin;
	go
		ekipGfinntk"IgnosFO);	uti_ancflush_cmmae S,
					&in_tes the rhutdown	nceme_dma_tDCMD_SUCCEisf cpu_CTRL-SHUTDOWNk gOekipGfinntk"Ignos:OU
	(canceD>(st>e3layedsworasas_(sissworasstTllIins	int LL;

ce-pts)
		retMElILLn prne fraMD ti_anceHv, "ee-,event *ME t;pts)
		retMEgasacanceD_e3layed_work_ryncphu_tohotplug_workstance			returnME t;n progr i;

	(canceD>CMD>waitseventc"LL;
wake_up_aFWphys_addr = DfdGc9Hfwait
q)og;
tRNClel_e FWphys_addr = Ds	caRNClel) * mreate_fTake>(st>*/
	for eoff>(st>*/
	for earrayf Notabs_seHwexNTllIT:
ate_fdecreme i.(stru		Gtfdex.(We le| lookearrayebeIspars earraydNULL;
64),(ix tsi i <_cReHv, "mgmtss_ge.			Gtfdexi i++)hba_lmS)
cReHv, "mgmtss_ge.i			retu[i]Kll 	}
9Hf>liv(&instes the mgmtss_ge.to ge--;;
nstes the mgmtss_ge.i			retu[i]Klnn progsemegasas_get|Hfe
	 */
	for (i[e;

	re
/*FI_RESET_FLAGS,
					&inn_tes the EL, &cm_irqv_S,
					&inOUcmutex);
			brnsixsvectorsool\ IGNORE_T
rqsvectorsIMEOUHrlHE_TIMEanc m ce-pts)
		reteak;
		case Kll VENTURAeturn 0iv(&in64),(ix tsi i <_Rci->ldList[laC	br_EXT; ++i)a_lskORE_(	insta= outeam_WF>ect_by_ld[i]stanckORE_(	insta= outeam_WF>ect_by_ldstanc	insta= outeam_WF>ect_by_ld t;n progr i;OUcmutex);
			break;
		case KILL_ADAPTER:
	e fraE

		breakleaat_er PD
 * list se;vOl\ orseqGERpmsz =UHrlR IGNORE_TI:
			cCFGAPTQ_NUM_SYNC-e+EOUH	Te INITIATE_OCR:
			cCFGAPTQld*sas_		(Rci-PHYSList[l_waitItat1)stanc	4),(ix tsi i <_2 i i++)hba_lUcmut	insta= lor	ap[i]strllleDCMORE_TIMst_sntphys_addr = cpu_to_le3FWF>_		me	insta= 			GFRpmsz3FWF>_		me	insta= lor	ap[i]3FWF>_		me	insta= lor	ap_;
		[i]stancUcmut	insta= lordrvT	ap[i]seCtrllammutes_vmCMD) treakt	insta= lordrvT	ap[i]sstrlll	vORE_(	insta= lordrvT	ap[i]stancUpreak;
		nc	RE_Tpages((ulong)	insta= lordrvT	ap[i]OFWF>F timm	insta= drvT	apTpagesstancU i;
aUcmut	insta=  orseqGrync[i]strllleDCMORE_TIMst_sntphys_addr = cpu_to_le3FWF>_	 orseqGERpmsz3FWF>_		insta=  orseqGrync[i]3FWF>_		insta=  orseqG;
		[i]stanc}anance->hba_ltes the releaat_FP_E* list se;vO i;
kORE_([i] = kmalCoc(ss_geanc m ce-pts)
		retevCtWF>FW5_
s_ IGNORE_TIMEOUdRE_TITIMEOUHrlR IGNORE_TI_anceHv,evCtWF>FW5_OFWF>F*/
	for (ievCtWF>FW5nu*/
	for (ievCtWF>FW5sizeo(stru*/
	for (in(ss_ge)a_l IGNORE_TIMEOUdRE_TITIMEOUHrlR IGNORE_TI:
			cinfo-OFWF>F */
	for (in(ss_geOFWF>F */
	for (in(ss_geHvNFOuctru*/
	for (itgT_pr sna_l IGNORE_TIMEOUdRE_TITIMEOUHrlR IGNORE_TI:
	TAR>pd_PROPERTIES_OFWF>F */
	for (itgT_pr snFWF>F */
	for (itgT_pr sHvNFOuctru*/
	for (ivf affilite\n")trl IGNORE_TIMEOUdRE_TITIMEOU(Rci->ldList[laC	brdpoold*sas_	F FFmd_timeout_ocr:
	LD_VF_AFFoLIATION_OFWF>FFFFFpts)
		retvf affilite\n"OFWF>FFFFFpts)
		retvf affilite\n"sizeof(stru*/
	for (ivf affilite\n"_111)trl IGNORE_TIMEOUdRE_TITIMEOsas_	F FFmd_timeout_ocr:
	LD_VF_AFFoLIATION_111_OFWF>FFFFFpts)
		retvf affilite\n"s111OFWF>FFFFFpts)
		retvf affilite\n"s111sizeof(stru*/
	for (ihbTuctu	memna_l IGNORE_TIMEOUdRE_TITIMEOUHrlR IGNORE_TI:
	CTRL-HB_HOST_MEM_OFWF>FFFFFpts)
		rethbTuctu	memOFWF>FFFFFpts)
		rethbTuctu	memsizeof(stru*/
	for (ilrash_dump_buf-sas IGNORE_TIMEOUdRE_TITIMEOUCRASHyancyte)_ FW',ol\9meou*/
	for (ilrash_dump_buf,u*/
	for (ilrash_dump_izeof(stru*/
	for (ie->cmdss_geHbuf)a_l IGNORE_TIMEOUdRE_TITIMEOUHrlR IGNORE_TI:
	ineCSYSTEM_info-OFWF>Fmeou*/
	for (ie->cmdss_geHbuf,u*/
	for (ie->cmdss_geHvNFOOutes the ORE_TIoc(szeoE* list se;vvOsd = uctu	puu(uctuk gOE IGNII_RESET_doorbEp(MRs c ti_anceHv, "IgnorrhutdownE aShutdownEe>C(stpoifid] h6_doorbOF>Gener it_doorbeto allumdd] e pool itself
	 */
	dmrhutdown to allo IGNIMEloc ev		megMD ti_anceHv, "set(instance->cmd_est IGNgel_ rv *)&(TIMEeganUcpu_to_le3unloadx t1 c\d
ce-f>|Hf>|Hwait
for_pu_ii-__ostate\n"alE	}
9Hf>lin;
	go
		ekipGfinntk"IgnosFO);	uti_ancflush_cmmae S,
					&in_tes the rhutdown	nceme_dma_tDCMD_SUCCEisf cpu_CTRL-SHUTDOWNk gOekipGfinntk"Ignos:OU*/
	for (i[e;

	re
/*FI_RESET_FLAGS,
					&in_tes the EL, &cm_irqv_S,
					&inOUcmutex);
			brnsixsvectorsool\ IGNORE_T
rqsvectorsIMEOUHrlHE_TIMEanc ti_anceHv, "IgnormgmtsostnE acharIT:de "ostn"Ee>C(stpoifid] /f>|Hf>|Hf>|v, "Ignormgmtsostn to alloiT:de *iT:de,	dcmd->cf	}
	*f	}
p		meg
		if (tanowsas_dcthost>us{
scwTR_ladminsrightsdNULL;
ES)
!capnsta(CAP SYS_ADMIN		
saRNC)DCe EA DCMHv, "IgnoreIgnox._anceHv, "Ignormgmtsfaryncn-	AryncnT:
ofiir MEgUdRate\n");
om.a|Hlicate =sd32 and>cmd_ *cmte = reac"the lCMDntk(pr AS10cUERYFDY) {

globals	int . We->ca=	andevent occun ,RSIGIOxNTllIbeIseitsistCMD>pr AS10es_anc(smd_	int .d] /f>|Hf>|Hf>|v, "Ignormgmtsfarync(f>|vfd,	dcmd->cf	}
	*f	}
p,Hf>|v,Hf>		megasas_cizsetu+) _
	i;gl	}

	memaryncG	int _tu+) )trOUr xcefarync_helsta(fd,	f	}
p,Hm:de,	l	}

	memaryncG	int )izsetu+) _unloi;gl	}

	memaryncG	int _tu+) )trOUerrurc >E_0)e frasansta(sae !y;cmegaswstnlwe E_T_aoctl LL;
	f	}
p>inrivareG *)& =	f	}
p= cp"Ignore0;ure
	 load ||
			test_bF"ze

	me:efarync_helstai.nstan [%d]meganrcs c\dRNC)DCe_cizox._anceHv, "IgnormgmtsOF>vtat charIT:de "OF>v"Ee>C(stpoifid] ] /f>|Hf>|Hunnstains 0,v, "IgnormgmtsOF>v(dcmd->cf	}
	*f	}
ODetll rl		} *wait		megunnstains 0,v,RNC_geunnstainslong d(s u gOE F>v_wait(f	}
ODl	}

	mem F>v_wait,>waite;vOsp		_ */k_irqvave(& F>v_ee-, */k, d(s ue;vOerrutes the OF>v_wait
ae-)F>v	RNC.= (POLLIN | POLLRDNORMe;vOF>vOF>v	RNC.= 0;setes the OF>v_wait
ae->(&sizesp		_unloi;_irqre

ore(& F>v_ee-, */k, d(s ue;vO"IgnorefRNC_gn -ENODEV;
		}

	rol_lrash_dump_pNORm "soctl:
f _	Se>c_CRASHyaUMP_MODE_index stCMD> rame_dma_pd] h6gno:rMDRimd_list);

	idd\ERs>|Hf>|Hf>|v, "Ignorrol_lrash_dump_pNORm "soctlGNORE_TI_anceHv, "Ignore		megMD ti_anceHv, "set(instan */
st
e->cmd_ geasasi, erro_(cesizeasasurash_ttate;
 ge =rash_ttate;
  "e32DCMD ti_anceH.re (Dw[0]e OU64),(ix tsi i <_cReHv, "mgmtss_ge.			Gtfdexi i++)hba_l */
st
e->cmd_pu_to_le32(mgmtss_ge.i			retu[i]izet MR_ */
st
e->cmd_p&&  */
st
e->cmd_(ilrash_dump_drvTttate;
fo(&insta_H-atomicsreaMph */
st
e->cmd_(iadpMEcovery)KllFWF>F/E_TIMEOHBA_OPERATIONAL)eor s	;
!, "Ignorrol_lrash_dump_pNORm _ */
st
e->cmd_nFWF>F =rash_ttate;
)neCtrlla */
st
e->cmd_(ilrash_dump_appTIMEOUdRMAFWF>F =rash_ttate;
tancUp3 cmd->fra */
st
e->cmd_(icpu_to_le3FWF>_	"A|Hlicate =(fw		fw_ssurashmIO_TIF	"dumpsm:deetemdI_SAS10gth = cpp erro_(cesizeot nce->hba_lsp3 cmd->fra */
st
e->cmd_(icpu_to_le3FWF>_	"A|Hlicate =(fw		fw_ssurashmIO_TIF	"dumpsm:deetemdfnstangth = cpp erro_(ce-1tancU iet|Hfe
O"Ignoreerro_gnox._anceHv, "Ignormgmtsfw"soctl_-	Istats>	anageme i.soctl0cUERFW ev)
{
	int ret;
ifferences between 32 @argpet;
Us{
's_aoctl packeid] /f>|Hf>|Hf>|
, "Ignormgmtsfw"soctltancet->read_fw_status_reg(
				insta>FFF FFmncet->read_fw_socpackei __us{
32 us{
_socsta>FFF FFmncet->read_fw_socpackei *soc		megMD ti_anceHv, "e;
	n *kern"e;
	n DCMD ti_anceHv, " "Ignore DCself
*kstan_arr[Rci-IOCTL-SGE]izeeDCMreak_mdbufT_stdlH(cesizeasaserro_(ces,si;egself
*seise t;n progreDCMreak_mdseiseT_stdlH_geunnstainslong *seise_ptr;t, *
	f>|Hf>v,MR_PD_INFkstan_arrnct;
	u32 fwkstan_arr_|Hf;
_muteoc16MD) to get> Rci-IOCTL-SGEn ance->unload ||
			test_bit(ys_addr = cpu_to_le3 "SGE to get[%d]t> ru		 limit [%d]megata>FFF FF eoc16MD) to ge, Rci-IOCTL-SGEn;
saRNC)DCe Ete;
rogr i;
= cpu_to_le32(MR_DCMD_PD_GET_INFO;
	dcmd->sgl.sge32[load ||
			test_bit(ys_addr = cpu_to_le3 "instance->pd_iai= cppackeised>w(KERNC)DCe DCMD ti_anceH
		if (Us{
's_IOCTLppackei has 2);

	is)
			imum). Copdcthost>two	if (;

	is)ad o ourtfmd's_;

	is."e32DCMD ti'siizeof(stNTllIpd_	if (overwritttnlwstnlwe copdc;
om.us{
's_;

	is."Soetemds_seH allo	if (slon6 sepNORtelydNULL;
cRmcpy(e32DCMD ti, eoc16MD ti.raw, 2)*ORE_T i;
	u8 ma FW's c>e32DCMD ti_ahdr.izeof(stpd_info->statel32DCtfdexs c>e32DCMD ti_ahdr.	le16_to_cpue32DCMD ti_ahdr.d(s us&ed>w(KERNC)Dfd~d i;
	u8 matEEE |sas_		meou ||his 	u8 ma GL64 |sas_		meou ||his 	u8 ma ENSE64))cpuf>|Hf>|HfIMEOUHrlHE_Te32DCMD ti_anceH.f>|Hf>	u3ct MR_f>|Hf>|H=Eisf cpu_CTRL-SHUTDOWNkhba_lmS)
cReHv, "MR_DCoc(ss_geE	}
9Hf>liv!E_TIMEOre DCMDnv(&instes the ak;

	case DCMD_SUCCESS:
		iKERNC)DCe 1Hf>t|Hfe
	 *MR_f>|Hf>|H=Eisf aC	bR_ uT_APP_CRASHaUMP_MODEnv(&inerro_(ce, "Ignorrol_lrash_dump_pNORm "soctlGSS:
		iKtes the ak;

	case DCMD_SUCCESS:
		iK"Ignoreerro_gnanceH
		if (Tstru	nageme i.sfd ifa_rebetwe->ca|Hlicate =s(vs_sthe fw.us{sdNUL	ce->;

	is."E.g, RAID OUnfiguate\n")change ,RLDfpr staty)change dNUL	etc w_staccomplisae conrough diancee i.ktfdHdof ce->;

	is."Tstate_fdY) {

need0cUERcw_stas_dcab
	ncsubstitutntk(us{
3stancestwTR_ate_fkernel stancestanc GLs."Tst  */
e\n")of  GLxi_lREbaddte;instheate_fto alloiocpackei itself= NULL;
kern"e;
	nestenD ti_anceHv, "e;
	n *) s c\d( unnstainslong)e32DCMD ti + eoc16MDl_off) * mreate_fsta(emmanus{
3stanceCESreatsta mirro_(stance(vs_scopdcindNULL;
64),(ix tsi i <_eoc16MD) to gei i++)hba_lmS)
!eoc16MDl[i].*ovsle-)F>v	HMEtntu>v,MR	kstan_arr[i]KlneDCMrMD) to st_sntphys_addr = cpu_to_le3FWF>_		meF eoc16MDl[i].*ovsle-OFWF>F timmm&bufT_stdlH,F= instance->c	lmS)
!kstan_arr[i])hba_ls3 cmload ||
			test_bit(ys_addr = cpu_to_le3 "instance->Yancem"
_		meou ||"kernel  GLxstance(64),IOCTLgth = cpperro_(ce-DCMD ti_a
	go
		
	nHf>t|Hfrasata>F (We do
	/*change>(st>eDCMo st_sntMERNC;
	ota>F ( IGNCMD) to get the tas_dc"Ignorsd32bitsreakS10esta>F L;
	kern"e;
	n[i].;
		break;
d_info->statebufT_stdlHstanckern"e;
	n[i]., "Ignord_info->stateeoc16MDl[i].*ovsle-);Hfrasata>F (We Sreatsdta kernel stance,c pkS1poddntkce->theat	32 us{
 stance.(Nowfcopdcinc;
om.(st>us{
 stanceta>F L;
	ES)
ncpy_;
om_us{
(kstan_arr[i], eoc16MDl[i].*ovsce, OFWF>FmeoGu_FR eeoc16MDl[i].*ovsle-))neCtrllerro_(ce-DFAULTi_a
	go
		
	nHf>t|Hfe
	 *MR_eoc16Meise_le-)e fraMeise t;eDCMrMD) to st_sntphys_addr = cpu_to_le3 eoc16Meise_le-re.  TT|||||&seiseT_stdlH,F= instance->c	lmS)
!seiseneCtrllerro_(ce-DCMD ti_a
	go
		
	nHf>t|Hfraseise_ptrMAFWF(unnstainslong *)d( unnstainslong)e32DCMD ti + eoc16Meise_off) *		*seise_ptrMAd_info->stateseiseT_stdlH)gnanceH
		if (Seen(strrync_= cpElag>esI(sati(strISR knowstviceUERcompleeec(smd	if (cmdce->thelSCSI mid-ltya_dNULL;
((u16Mync_= cp the F
MR_f>|Hf>|H_state;
	i;
	u8 max_A cgHf>|Hf>|H0)Kll TIMEONOT_	}
	DneCtrlR(u16Mync_= cp tsizeoe32[0].phys_addr = cpu_to_le3FWF>DRNC)DCe-st_SYe6}
9Hruct >f>|Hf>|0x%x ;
	u8 max_wait;_drv|0x%xmegasas_md *cmd;
	struct mOUf>|Hf>,	;
	u8 max_wait;_drv>w(KERNC)DCe Dt_SYogr i;
= c16Mync_= cp tsizOUcmutex);
			brunloadx = 1)eCtrl3 cmd->frame,
				      cmd->f (DY) {

unloadxistancpr gkS10m"
_		"do
	/*submit  *)& e->Y|Hlicate =gth = cpgo
		
	nHf>};

		if (copdc
	nc(strkernel stancest
		us{
3stancesdNULL;
64),(ix tsi i <_eoc16MD) to gei i++)hba_lmS)
ncpy_o->us{
(eoc16MDl[i].*ovsce, O kstan_arr[i],FWF>Fmeoc16MDl[i].*ovsle-))eCtrllerro_(ce-DFAULTi_a
	go
		
	nHf>t|Hfe
	 
		if (copdc
	nc(strseisedNULL;
ES)
eoc16Meise_le-)e frasata>F (seise_ptrMpoifisce->thel */
e\n")(satihas (st>us{
ta>F (seise(stance(veakS10ta>F L;
	seise_ptrMAd(unnstainslong *)d( unnstainslong)eoc16MD ti.rawe+EOUH	eoc16Meise_off) *a_lmS)
ncpy_o->us{
((self
__us{
32)( unnstainslong)FWF>FmMR_Dunalstain((unnstainslong *)seise_ptr)-OFWF>Fmseise3 eoc16Meise_le-))hba_ls3 cm0].phys_addr = cpu_to_le32(instancUERcopdc
	nc(		us{
3IO_TIF	"seise( *)&gth = cpperro_(ce-DFAULTi_a
	go
		
	nHf>t|Hfe
	 
		if (copdc(strswait;
|Hf>sc"Ignorte;by the fwdNULL;
ES)
ncpy_o->us{
(&us{
_soc16MD ti.hdr.imax_wait;,ol\9m&e32DCMD ti_ahdr.imax_wait;,DHrlR IGu8)in ance->unload ||
			test_bit(ys_addr = cpu_to_le32(Erro_(ncpyntkc
	ncimax_wait;gth = cprrro_(ce-DFAULTi_ae
	
	n:
CerruseiseneCtrleDCMORE_TIMst_sntphys_addr = cpu_to_le3 eoc16Meise_le-re.  TF FFmeise3 seiseT_stdlH)gnanceH64),(ix tsi i <_eoc16MD) to gei i++)hba_lmS)
kstan_arr[i])hba_ls3DCMORE_TIMst_sntphys_addr = cpu_to_le3FWF>_	 fIMEOUHrlHE_Tkern"e;
	n[i]., "IgnR>FWF>F mekstan_arr[i],FWF>F	 fIMEOUHrlHE_Tkern"e;
	n[i].;
		break)stanc	kstan_arr[i]Klnn progrt|Hfe
	 tes the ak;

	case DCMD_SUCCESS:
		i"Ignoreerro_gnox.>|Hf>|Hf>|v, "IgnormgmtssoctlGfw dcmd->cf	}
	*f	}
ODunnstainslong arg		megMD ti_anceHv, "socpackei __us{
32us{
_socMAFW c\d(MD ti_anceHv, "socpackei __us{
32)arg DCMD ti_anceHv, "socpackei *soc DCMD ti_anceHv, "set(instance->cmd_ geasaserro_ geasasi_geunnstainslong d(s u g, *
	wait_
	m n=ORE_TIMEORE*/
	WAITruct izOUc) xcememdup>us{
(us{
_socs
	u32 fw_coc)NFO;
	dcIS[ERR(coc)N(KERNC)DCePTR[ERR(coc) g	 */
	for estnceHv, "lookup_i/
	for 
eoc16uctu	noNFO;
	dcm	}
9Hf>liv(&inerro_(ce-DCMDEVHf>	go
		
	n_kORE_T
oc DC i;

	(AdjusT_aoctl waits
	m n64),VF m:deeLL;

ce-pts)
		retME	in

orId)a_lwait_
	m n=ORE_TIMEOROUTuct WAITruct _VFe OU
	(B
	i;.soctl0cancVF m:deeLL;

ce-pts)
		retME	in

orId_&&_!rMD)wRvf soctl0iv(&inerro_(ce-DCMDEVHf>	go
		
	n_kORE_T
oc DC i;
ctruatomicsreaMphys_addr = adpMEcovery)Kll /E_TIMEOHW_CRITList[ERRORseCtrle32[0].phys_addr = cpu_to_le3 "C eme_dma__anccriaserro_gth = cprrro_(ce-DCMDEVHf>	go
		
	n_kORE_T
oc DC i;
ctruex);
			brunloadx = 1)eCtrlrrro_(ce-DCMDEVHf>	go
		
	n_kORE_T
oc DC i;
ctrudown	sfd iruptista(hys_addr = DoctlGsem))eCtrlrrro_(ce-DRE*TARTSYSHf>	go
		
	n_kORE_T
oc DC i;
64),(ix tsi i <_wait_
	m i i++)hba;
	sp		_ */k_irqvave(&pts)
		rethba, */k, d(s ue;vO
ctruatomicsreaMphys_addr = adpMEcovery)Kll /E_TIMEOHBA_OPERATIONAL)eCtrlesp		_unloi;_irqre

ore(&pts)
		rethba, */k, d(s ue;vO
egasas_get|Hfesp		_unloi;_irqre

ore(&pts)
		rethba, */k, d(s ue;vc	lmS)
!(ix%ORE_TIMEORE*/
	NOTait =eTERVAL))hba_ls3 cmT:
orbEhys_addr = cpu_to_le3 "waitntkIO_TIF"64),c eme_dma__aksel 
		fwnishce->re>t|Hframsleep(1000)) f nctsp		_ */k_irqvave(&pts)
		rethba, */k, d(s ue;vO MR_atomicsreaMphys_addr = adpMEcovery)K!l /E_TIMEOHBA_OPERATIONAL)eCtrlsp		_unloi;_irqre

ore(&pts)
		rethba, */k, d(s ue;vc	le32[0].phys_addr = cpu_to_le3 "(ima tateHwh	}
	waitntk(64),HBA;e->OFcovergth = cprrro_(ce-DCMDEVHf>	go
		
	n_upHf>};
sp		_unloi;_irqre

ore(&pts)
		rethba, */k, d(s ue;vc	erro_(ce, "Ignormgmtsfw"soctltDCMD_SUCCEus{
_socs
coc) g
	n_up:
Cup(hys_addr = DoctlGsem);
	
	n_kORE_T
oc:OCkORE_([oc) gi"Ignoreerro_gnox.>|Hf>|Hf>|v, "IgnormgmtssoctlGatn to allof	}
	*f	}
ODunnstainslong arg		megMD ti_anceHv, "set(instance->cmd_ geMD ti_anceHv, "ee- ae-reoasaserro_ geasasi_geunnstainslong d(s u g, *
	wait_
	m n=ORE_TIMEORE*/
	WAITruct izOUcmut		}
>inrivareG *)& !=	f	}
 fba_leoad ||
			test_bF"ze

	me:efarync_helstaiwastviceIO_TWFFFFFF"lCMDinsfirsised>w(KERNC)DCe Dte;
rogr i;
ES)
ncpy_;
om_us{
(&ae-, (self
__us{
32)args
	u32 fwae-))n(KERNC)DCe DFAULTi_	 */
	for estnceHv, "lookup_i/
	for 
ae-.uctu	noNFOO;
	dcm	}
9Hf>li(KERNC)DCe DCMDEVHf;
ctruatomicsreaMphys_addr = adpMEcovery)Kll /E_TIMEOHW_CRITList[ERRORseCtrlRNC)DCe DCMDEVHfC i;
ctruex);
			brunloadx = 1)eCtrlRNC)DCe DCMDEVHfC i;
64),(ix tsi i <_wait_
	m i i++)hba;
	sp		_ */k_irqvave(&pts)
		rethba, */k, d(s ue;vO
ctruatomicsreaMphys_addr = adpMEcovery)Kll /E_TIMEOHBA_OPERATIONAL)eCtrlesp		_unloi;_irqre

ore(&pts)
		rethba, */k,FWF>F td(s ue;vO
egasas_get|HHfesp		_unloi;_irqre

ore(&pts)
		rethba, */k, d(s ue;vc	lmS)
!(ix%ORE_TIMEORE*/
	NOTait =eTERVAL))hba_ls3 cmT:
orbEhys_addr = cpu_to_le3 "waitntk(64)IO_TIF"c eme_dma__aksel 
		fwnishce->re>t|Hframsleep(1000)) f nctsp		_ */k_irqvave(&pts)
		rethba, */k, d(s ue;vO MR_atomicsreaMphys_addr = adpMEcovery)K!l /E_TIMEOHBA_OPERATIONAL)eCtrlsp		_unloi;_irqre

ore(&pts)
		rethba, */k, d(s ue;v	le32[0].phys_addr = cpu_to_le3 "(ima tateHwh	}
	waitntk(64),HBA;e->OFcovergth = cpRNC)DCe DCMDEVHfC iesp		_unloi;_irqre

ore(&pts)
		rethba, */k, d(s ue;vc	tu+) _
	i;glpts)
		retMELL;
tu+) )truerro_(ce, "IgnorragUdRErTae- 
static , ae-.seqGnumOFWF>FFFFF ae-.clas "lolCMe_word&in_tu+) _unloi;glpts)
		retMELL;
tu+) )tru"Ignoreerro_gnox._anceHv, "Ignormgmtssoctl_-	charIT:de soctl_e>C(stpoifid] /f>|Hf>|Hlong
, "Ignormgmtssoctl to allof	}
	*f	}
ODunnstainsasasuf>|Hunnstainslong arg		megMc
		}
	d->sgl.s_pd_lME_TIMEOIOC_	}
MWARE: cpRNC)DCe, "IgnormgmtssoctlGfw f	}
ODarg	 ge =pd_lME_TIMEOIOC_>pd_AEN: cpRNC)DCe, "IgnormgmtssoctlGae- f	}
ODarg	 g>}c>|"Ignore DCMTTY g ti#ifdef CONFIG_COMPAT.>|Hf>|Hf>|v, "IgnormgmtscompatssoctlGfw dcmd->cf	}
	*f	}
ODunnstainslong arg		megMD ti_acompatsnceHv, "socpackei __us{
32csocMAFW c\d(MD ti_acompatsnceHv, "socpackei __us{
32)arg DCMD ti_anceHv, "socpackei __us{
32socMAFW c\dcompatsrMD) tus{
_space(md_timeout_ocr
				MEG	ocpackei)NFO;
sasi_geasaserro_(cescpueompatsuptk_mdptr;t, *
	 */
stMeise_off;t, *
	 */
stMeise_led;t, *
	us{
_seise_off;t;
ES)
nlear>us{
(eocs
	u32 fw_coc)Nn(KERNC)DCe DFAULTi_	 *S)
ncpy_		_us{
(&eoc16uctu	no, &ceoc16uctu	no, HrlR IGu16)_TIME; c\dcopy_		_us{
(&eoc16MDl_off, &ceoc16MDl_off, HrlR IGu_FR_TIME; c\dcopy_		_us{
(&eoc16Meise_off, &ceoc16Meise_off, HrlR IGu_FR_TIME; c\dcopy_		_us{
(&eoc16Meise_le-r &ceoc16Meise_le-r HrlR IGu_FR_TIME; c\dcopy_		_us{
(eoc16MD ti.raw, ceoc16MD ti.raw, 128_TIME; c\dcopy_		_us{
(&eoc16MD) to ge, &ceoc16MD) to ge, HrlR IGu_FR_n(KERNC)DCe DFAULTi_	 
		if (Tstrseise_ptrMis	us{e;ins, "Ignormgmtsfw"soctl_as_dcwstn
>F (seise_ce->istvicenull;
	o pMEpw_stthe 64bits alloDunda_dNULc(strs ti coddne\n".dNULL;
ES)
MR_Dus{
( */
stMeise_offit(yoc16Meise_off)TIME;
MR_Dus{
( */
stMeise_le-r &eoc16Meise_le-)eIME;
MR_Dus{
(us{
_seise_off, &ceoc16Meise_offNn(KERNC)DCe DFAULTi_	 *S)
 */
stMeise_offK!l us{
_seise_off)(KERNC)DCe Dte;
rog	 *S)
 */
stMeise_le-)e fraself
__us{
32*seise_soc"ptrMAFWF	(self
__us{
322)( u832)( unnstainslong)&eoc16MD ti.raw) +	 */
stMeise_off = cpeompatsuptk_md*seise_csoc"ptrMAFWF	(eompatsuptk_md*)(  unnstainslong)&ceoc16MD ti.raw) +	us{
_seise_off);vO
ctruMR_Dus{
(ptk3 seiseTcsoc"ptr)eIME;
 c\dpu_Dus{
(eompatsptr(ptk)3 seiseTsoc"ptr))FWF>RNC)DCe DFAULTi_C i;
64),(ix tsi i <_Rci-IOCTL-SGEi i++)hba_lmS)
MR_Dus{
(ptk3 &ceoc16MDl[i].*ovsce, )eIME;
 c\dpu_Dus{
(eompatsptr(ptk)3 &eoc16MDl[i].*ovsce, )eIME;
 c\dcopy_		_us{
(&eoc16MDl[i].*ovsle-OFWF>F &ceoc16MDl[i].*ovsle-r HrlR IGeompatsHrlR_t)))FWF>RNC)DCe DFAULTi_C i;
erro_(ce, "IgnormgmtssoctlGfw f	}
OD unnstainslong)eoc)i_	 *S)
ncpy_		_us{
(&ceoc16MD ti.hdr.imax_wait;,ol\9m&eoc16MD ti.hdr.imax_wait;,DHrlR IGu8)in anceeoad ||
			test_bF"ze

	me:eerro_(ncpy_		_us{
cimax_wait;gth = cpRNC)DCe DFAULTi_C ii"Ignoreerro_gnox.>|Hf>|Hlong
, "Ignormgmtscompatssoctl to allof	}
	*f	}
ODunnstainsasasuf>|ol\9mHunnstainslong arg		megMc
		}
	d->sgl.s_pd_lME_TIMEOIOC_	}
MWARE32: cpRNC)DCe, "IgnormgmtscompatssoctlGfw f	}
ODarg	 g>=pd_lME_TIMEOIOC_>pd_AEN: cpRNC)DCe, "IgnormgmtssoctlGae- f	}
ODarg	 g>}c>|"Ignore DCMTTY g t#endnf
fENODEVF	}
	ostate\n"snto allumd 64),u	nageme i.sfd ifa_rd] /f>|Hf>|Ho get	dcmd->cf	}
_ostate\n"sn, "Ignormgmtsfopusedl.s.owna_(ceTHIS_MODUL',ol.ostnEce, "Ignormgmtsostn,ol.faryncnce, "Ignormgmtsfarync,ol.unloi;ed"soctl_ce, "Ignormgmtssoctl,ol.OF>vtce, "IgnormgmtsOF>v,i#ifdef CONFIG_COMPAT.	.compatssoctltce, "Ignormgmtscompatssoctl,t#endnf
	.llseeC.= noop_llseeC,
};
fENODEV		m hotplug>IMEOUdRMMEgUdRate\n")to allumdd] e pool itto allo IGNIY) {

tes the OIGNIY) {

=hba;
.nam n=O"ze

raiax_asgasa.iaxrl		} =
tes the OIGNrl		},ol.Orob} =
tes the Orob}_on6,ol.remove =
tes the WF>Fch_on6,ol.ruspend_ce, "Ignorruspend,ol.reOuma_ce, "IgnorraOuma,ol.rhutdownEce, "Ignorrhutdown,
};
fENODEVSysfsfdY) {

attribu+)sd] e pool ittHrlR_t  {
_cmd;show dcmd->cIMEorb	dY) {

*d>|HcharI*buf)a{>|"Ignoresneoad f(buf,udcmle-(ME_TIMEOVERSION_ +	2f (%smegasas_ME_TIMEOVERSION_ g tpool it aC	bR_ATTR_RO( {
_cmdk gOeool ittHrlR_t releaat_dareGshow dcmd->cIMEorb	dY) {

*d>|HcharI*buf)a{>|"Ignoresneoad f(buf,udcmle-(ME_TIMEORELDATE_ +	2f (%smegasasME_TIMEORELDATE_ g tpool it aC	bR_ATTR_RO(releaat_darek gOeool ittHrlR_t IMEOUdR OF>v_for_eventGshow dcmd->cIMEorb	dY) {

*d>|HcharI*buf)a{>|"Ignoreseoad f(buf,u"%umeganIMEOUdR OF>v_for_event_ g tpool it aC	bR_ATTR_RO(IMEOUdR OF>v_for_event_ gOeool ittHrlR_t IMEOUdR IMEorb	change_show dcmd->cIMEorb	dY) {

*d>|HcharI*buf)a{>|"Ignoreseoad f(buf,u"%umeganIMEOUdR IMEorb	change_ g tpool it aC	bR_ATTR_RO(IMEOUdR IMEorb	change_ gOeool ittHrlR_t dbg_lvl_show dcmd->cIMEorb	dY) {

*d>|HcharI*buf)a{>|"Ignoreseoad f(buf,u"%umegantes the Wbg_lvl)gnox.>|Hf>|HtHrlR_t dbg_lvl_s
ore(dcmd->cIMEorb	dY) {

*d>|Hc get	charI*buf,ol\9meou HrlR_t to ge		megasas_etval t;to gei_	 *S)
sscanf(buf,u"%u"ODl	}

	memWbg_lvl) <_1n anceeoad ||
			tERRF"ze

	me:eto ldtvicesel Wbg_lvlgth = cpRNCval t; Dte;
rogr idRNC)DCe_NCval g tpool it aC	bR_ATTR_RW(Wbg_lvl)gnf>|Hf>|Hf>e->faself
	 */
	dmremove_sd = _doorbEdcmd->csd = _doorbd*s ev		megM->unload ||
			tinfoanI_le3 "SCSI _doorbdisc"Imovength = csd = remove__doorbEdIMEegassd = _doorb	puu(sIMEanc tipool itself
, "Ignoree-,OF>vingEdcmd->cwork_rcmd->c*works	megMD ti_anceHv, "ee-,event *ME t
v	HMEta->fr_of(work,Fmncet->read_fw_ee-,event, hotplug_work.workstanMD ti_anceHv, "set(instance->cmd_estu_toce->cmd_ geun\n")_anceHv,evCtclas "lolCMe clas "lolCMetanMD ti_aoSd = Ht)an*ht)a DCMD ti_acsd = _doorbd*s evhe Fu16meou n(ss_dex(cescpuu16	l(ss_dex(cescpuasasmeF e, j, doscan_cescpuu32 seqGnumO	wait_
	m n=ORE_TIMEORE*/
	WAITruct izoasaserro_ geu83 Ignorremd tTIMEOre DCMDFOO;
	dcm	}
9Hf>li anceeoad ||
			tERRF"invalidu*/
	for !gth = cpkORE_(ev>w(KERNC)DC DC i;

	(AdjusT_event work	int _threaM waits
	m n64),VF m:deeLL;

ce-pts)
		retME	in

orId)a_lwait_
	m n=ORE_TIMEOROUTuct WAITruct _VFe OU
	(Do
	/*runconteevent work	int _threaM 
ceOCRdisc"unnntk(LL;
cu+) _
	i;glpts)
		retMELL;
tu+) )tr
ce			returnME t;n progrht)an\n");

	retuht)agne
ce-pts)
		retevCtWF>FW5_hba_ltes the de|Hf>,evC_S,
					&inOUgMc
		}
	IMEOUHrlHE_Tpts)
		retevCtWF>FW5OR>Mdsefo(&
v	Hpd_lMR_EVT			cinSERTED:
v	Hpd_lMR_EVT			cREMOVED:
v		Ignorremd tcReHv, "MR_Dn(slisC_S,
					&in	_lmS)
Ignorremd E_TIMEOre DCMDn.sg		Ioscan_ceSCAN			cCHANNEL;vO
egasas_g
v	Hpd_lMR_EVT	LD_OFFruct:
v	Hpd_lMR_EVT	CFGACLEARED:
v	Hpd_lMR_EVT	LD_DELETED:
v	Hpd_lMR_EVT	LD_CREATED:
v	;
	dcm	}
9Hf>letME	in

orId_||sas_	-pts)
		retME	in

orId_&&_cReHv, "MR_DrlRvf affilite\n">e DCMD ti_0)))FWF>	Ignorremd tcReHv, "l(slisCG	inrytDCMD_SUCCEisfLD_QUERY_TYPE_EXPOSED_TO_HOST&inOUglmS)
Ignorremd E_TIMEOre DCMDn.sg		Ioscan_ceSCAN	V	cCHANNEL;vvO
egasas_g
v	Hpd_lMR_EVT	CTRL-HOST_BUS_SCAN	REQUESTED:
v	Hpd_lMR_EVT	FOREIGN	CFGAIMPORTED:
v	Hpd_lMR_EVT	LD_ST}
	}CHANGE:
v		Ignorremd tcReHv, "MR_Dn(slisC_S,
					&inOUglmS)
Ignorremd!E_TIMEOre DCMDn.sg
egasas_g
v	;
	dcm	}
9Hf>letME	in

orId_||sas_	-pts)
		retME	in

orId_&&_cReHv, "MR_DrlRvf affilite\n">e DCMD ti_0)))FWF>	Ignorremd tcReHv, "l(slisCG	inrytDCMD_SUCCEisfLD_QUERY_TYPE_EXPOSED_TO_HOST&inOUglmS)
Ignorremd!E_TIMEOre DCMDn.sg
egasas_g
v	;Ioscan_ceSCAN	V	cCHANNEL |eSCAN			cCHANNEL;vO
e3 cmd->frame,
				      cmd->f (scannntk(64),sd =%d...megasas_	");

	retuht)a16uctu	noNFO;
egasas_g
v	Hpd_lMR_EVT	CTRL-PROP}CHANGED:
v	;	Ignorremd tcReHv, "MR_DCoc(ss_geE	}
9Hf>li;.sg
egasas_g
e3 fault:
v	;Ioscan_ce0;vO
egasas_get|Hf nce->hba_le32[0].phys_addr = cpu_to_le3 "invaliduevCtWF>FW5!gth = cptu+) _unloi;glpts)
		retMELL;
tu+) )trupkORE_(ev>w(KERNC)DC DC i;
tu+) _unloi;glpts)
		retMELL;
tu+) )tr;
ctrudoscan_&eSCAN			cCHANNELiv(&in64),(ix tsi i <_RE_TIMEORci-P	cCHANNELSi i++)hba_lU64),(jx tsi j <_RE_TIMEORci-DEV_PbR_CHANNEL; j++)hba_lU	n(ss_dex(cei*RE_TIMEORci-DEV_PbR_CHANNEL +	j;.sg
es evh =UHd = _doorb	lookup(uctu, e, j, 0i;.sg
estru*/
	for (in(slisC[n(ss_dex].dY) {SweenMllFWF>Fas_M
			cST}
	}SYSTEM)hba_lU	lmS)
!s evhstrlll	ssd = add__doorbEuctu, e, j, 0i;.sg
epreak;
		ncssd = _doorb	puu(sIME1i;.sg
e nce->hba_lsp *S)
s evhstrlll	s	 */
	dmremove_sd = _doorbEdIME1i;.sg
e 
	cU iet|Hfe
;
ctrudoscan_&eSCAN	V	cCHANNELiv(&in64),(ix tsi i <_RE_TIMEORci-L	cCHANNELSi i++)hba_lU64),(jx tsi j <_RE_TIMEORci-DEV_PbR_CHANNEL; j++)hba_lU	l(ss_dex(ce(ix*_RE_TIMEORci-DEV_PbR_CHANNEL) +	j;.sg
es evh =UHd = _doorb	lookup(uctu, RE_TIMEORci-P	cCHANNELS + e, j, 0i;.sg
estru*/
	for (il(ssds[l(ss_dex]d!E_0xff)Tba_lU	lmS)
!s evhstrlll	ssd = add__doorbEuctu, RE_TIMEORci-P	cCHANNELS + e, j, 0i;.sg
epreak;
		ncssd = _doorb	puu(sIME1i;.sg
e nce->hba_lsp *S)
s evhstrlll	s	 */
	dmremove_sd = _doorbEdIME1i;.sg
e 
	cU iet|Hfe
;
ctrudgnorremd E_TIMEOre DCMDn.sgseqGnum|HfIMEOUHrlHE_Tpts)
		retevCtWF>FW5ORseqGnum) +	he Freak;
	seqGnum|Hf*/
	for (ilatu	seqGnum) 	 ga	ROgUdRErvf>liwTR_lFW(64),leenet	dE	inmd_enumbErvplus 1ULL;
(las "lolCMe.mREbars.reOerven(cescpuelas "lolCMe.mREbars.lolCMe =lMR_EVT	LOCALE_Aprogrelas "lolCMe.mREbars.elas  =lMR_EVT	CLASStest_b ge _mutex);
			bree-, "IlILLn prne frakORE_(ev>w(KERNC)DC DC i;
tu+) _
	i;glpts)
		retMELL;
tu+) )truerro_(ce, "IgnorragUdRErTae- 
static , seqGnumOFWF>Frelas "lolCMe.word&in__muterro_ntrle32[0].phys_addr = cpu_to_le3FWF>"ragUdREr ae-i.nstan erro_(%xmega erro_)izsetu+) _unloi;glpts)
		retMELL;
tu+) )trukORE_(ev>w(ox._anceHv, "Ignor[e ! - DY) {

loadxe>C(stpoifid] /f>|Hf>|Hf>|v_r[e ! , "Ignor[e !(self		megasas_val;e	 
		if (Boot{e;inskdumpskernel, m[e mFIe	mREo(stfooteoad s byate_fdI_RESntk(6ew(6ealumdsdNULL;
ES)
MELL;
_doorb0iv(&innsixsvectors * 1Hf>irdpqNennstap tsizeoeual_qdepthNII_RESE * 1Hf>e
	 
		if (Annoumd_edY) {

 {
_cmd(vs_soont__an64)mte\n"dNULL;
prss_geE"ze

	me:e%smega ME_TIMEOVERSION_ gctsp		_ */k_ie !ph F>v_ee-, */k_ gctsMEOUdR OF>v_for_event * n DCMMEOUdR IMEorb	changex t1 c\d_PD_INF&, "Ignormgmtss_geOct;
	u32 fw, "Ignormgmtss_ge)) * mreate_fROgUdRErvcharacRErv_doorbdT:de NULL;
rval t;ragUdRErTchr_do(t;
"ze

raiax_asssoctl"ODl	}

	memmgmtsfopu)trOUerrurval <_0)fba_leoad ||
			test_bF"ze

	me:efastancUERostnE_doorbdT:degth = cp"Ignore val ge}* m	}

	memmgmtsmajorno t;rval;e	 
		if (ROgUdRErvourselves asV		m hotplug>module NULL;
rval t; IGNregUdRErTdY) {
F&, "IgnorOIGNIY) {
)trOUerrurval)fba_leoad ||
			test_bF"ze

	me:e		m hotplug>MEgUdRate\n");astancgth = cpgo
		errrOIGdrv g>}c>|"val t;IY) {
_Sreats_f	}
F&, "IgnorOIGNIY) {
.dY) {rOFWF>FFF&IY) {
_attr_ {
_cmdk gUerrurval) cpgo
		errrdcf_attr_ {
;c>|"val t;IY) {
_Sreats_f	}
F&, "IgnorOIGNIY) {
.dY) {rOFWF>FFF&IY) {
_attr_releaat_darek gUerrurval) cpgo
		errrdcf_reD_ee(i gOU"val t;IY) {
_Sreats_f	}
F&, "IgnorOIGNIY) {
.dY) {rOFWF>F&IY) {
_attr_IMEOUdR OF>v_for_event_ gUerrurval) cpgo
		errrdcf_IMEOUdR OF>v_for_event;c>|"val t;IY) {
_Sreats_f	}
F&, "IgnorOIGNIY) {
.dY) {rOFWF>FFF&IY) {
_attr_Wbg_lvl)gnUerrurval) cpgo
		errrdcf_Wbg_lvl;OU"val t;IY) {
_Sreats_f	}
F&, "IgnorOIGNIY) {
.dY) {rOFWF>F&IY) {
_attr_IMEOUdR IMEorb	change_ gUerrurval) cpgo
		errrdcf_IMEOUdR IMEorb	change c\dRNC)DCe_val;e	errrdcf_IMEOUdR IMEorb	change:
vIY) {
_remove_f	}
F&, "IgnorOIGNIY) {
.dY) {rOFWF> FF&IY) {
_attr_Wbg_lvl)gnerrrdcf_Wbg_lvl:
vIY) {
_remove_f	}
F&, "IgnorOIGNIY) {
.dY) {rOFWF>&IY) {
_attr_IMEOUdR OF>v_for_event_ gerrrdcf_IMEOUdR OF>v_for_event:
vIY) {
_remove_f	}
F&, "IgnorOIGNIY) {
.dY) {rOFWF> FF&IY) {
_attr_releaat_darek gerrrdcf_reD_ee(i:
vIY) {
_remove_f	}
F&, "IgnorOIGNIY) {
.dY) {rOF&IY) {
_attr_ {
_cmdk gerrrdcf_attr_ {
:OE IGNunregUdRErTdY) {
F&, "IgnorOIGNIY) {
)trerrrOIGdrv:
CunragUdRErTchr_do(	}

	memmgmtsmajorno;
"ze

raiax_asssoctl")tru"Ignorerval g t._anceHv, "Ignorex ! - DY) {

unloadxe>C(stpoifid] /f>|Hf>|Hself
__ex ! , "Ignorex !(self		megIY) {
_remove_f	}
F&, "IgnorOIGNI