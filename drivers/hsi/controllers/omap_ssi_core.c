/* OMAP SSI driver.
 *
 * Copyright (C) 2010 Nokia Corporation. All rights reserved.
 * Copyright (C) 2014 Sebastian Reichel <sre@kernel.org>
 *
 * Contact: Carlos Chinea <carlos.chinea@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/compiler.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/of_platform.h>
#include <linux/hsi/hsi.h>
#include <linux/idr.h>

#include "omap_ssi_regs.h"
#include "omap_ssi.h"

/* For automatically allocated device IDs */
static DEFINE_IDA(platform_omap_ssi_ida);

#ifdef CONFIG_DEBUG_FS
static int ssi_debug_show(struct seq_file *m, void *p __maybe_unused)
{
	struct hsi_controller *ssi = m->private;
	struct omap_ssi_controller *omap_ssi = hsi_controller_drvdata(ssi);
	void __iomem *sys = omap_ssi->sys;

	pm_runtime_get_sync(ssi->device.parent);
	seq_printf(m, "REVISION\t: 0x%08x\n",  readl(sys + SSI_REVISION_REG));
	seq_printf(m, "SYSCONFIG\t: 0x%08x\n", readl(sys + SSI_SYSCONFIG_REG));
	seq_printf(m, "SYSSTATUS\t: 0x%08x\n", readl(sys + SSI_SYSSTATUS_REG));
	pm_runtime_put(ssi->device.parent);

	return 0;
}

static int ssi_debug_gdd_show(struct seq_file *m, void *p __maybe_unused)
{
	struct hsi_controller *ssi = m->private;
	struct omap_ssi_controller *omap_ssi = hsi_controller_drvdata(ssi);
	void __iomem *gdd = omap_ssi->gdd;
	void __iomem *sys = omap_ssi->sys;
	int lch;

	pm_runtime_get_sync(ssi->device.parent);

	seq_printf(m, "GDD_MPU_STATUS\t: 0x%08x\n",
		readl(sys + SSI_GDD_MPU_IRQ_STATUS_REG));
	seq_printf(m, "GDD_MPU_ENABLE\t: 0x%08x\n\n",
		readl(sys + SSI_GDD_MPU_IRQ_ENABLE_REG));
	seq_printf(m, "HW_ID\t\t: 0x%08x\n",
				readl(gdd + SSI_GDD_HW_ID_REG));
	seq_printf(m, "PPORT_ID\t: 0x%08x\n",
				readl(gdd + SSI_GDD_PPORT_ID_REG));
	seq_printf(m, "MPORT_ID\t: 0x%08x\n",
				readl(gdd + SSI_GDD_MPORT_ID_REG));
	seq_printf(m, "TEST\t\t: 0x%08x\n",
				readl(gdd + SSI_GDD_TEST_REG));
	seq_printf(m, "GCR\t\t: 0x%08x\n",
				readl(gdd + SSI_GDD_GCR_REG));

	for (lch = 0; lch < SSI_MAX_GDD_LCH; lch++) {
		seq_printf(m, "\nGDD LCH %d\n=========\n", lch);
		seq_printf(m, "CSDP\t\t: 0x%04x\n",
				readw(gdd + SSI_GDD_CSDP_REG(lch)));
		seq_printf(m, "CCR\t\t: 0x%04x\n",
				readw(gdd + SSI_GDD_CCR_REG(lch)));
		seq_printf(m, "CICR\t\t: 0x%04x\n",
				readw(gdd + SSI_GDD_CICR_REG(lch)));
		seq_printf(m, "CSR\t\t: 0x%04x\n",
				readw(gdd + SSI_GDD_CSR_REG(lch)));
		seq_printf(m, "CSSA\t\t: 0x%08x\n",
				readl(gdd + SSI_GDD_CSSA_REG(lch)));
		seq_printf(m, "CDSA\t\t: 0x%08x\n",
				readl(gdd + SSI_GDD_CDSA_REG(lch)));
		seq_printf(m, "CEN\t\t: 0x%04x\n",
				readw(gdd + SSI_GDD_CEN_REG(lch)));
		seq_printf(m, "CSAC\t\t: 0x%04x\n",
				readw(gdd + SSI_GDD_CSAC_REG(lch)));
		seq_printf(m, "CDAC\t\t: 0x%04x\n",
				readw(gdd + SSI_GDD_CDAC_REG(lch)));
		seq_printf(m, "CLNK_CTRL\t: 0x%04x\n",
				readw(gdd + SSI_GDD_CLNK_CTRL_REG(lch)));
	}

	pm_runtime_put(ssi->device.parent);

	return 0;
}

static int ssi_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, ssi_debug_show, inode->i_private);
}

static int ssi_gdd_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, ssi_debug_gdd_show, inode->i_private);
}

static const struct file_operations ssi_regs_fops = {
	.open		= ssi_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations ssi_gdd_regs_fops = {
	.open		= ssi_gdd_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int ssi_debug_add_ctrl(struct hsi_controller *ssi)
{
	struct omap_ssi_controller *omap_ssi = hsi_controller_drvdata(ssi);
	struct dentry *dir;

	/* SSI controller */
	omap_ssi->dir = debugfs_create_dir(dev_name(&ssi->device), NULL);
	if (!omap_ssi->dir)
		return -ENOMEM;

	debugfs_create_file("regs", S_IRUGO, omap_ssi->dir, ssi,
								&ssi_regs_fops);
	/* SSI GDD (DMA) */
	dir = debugfs_create_dir("gdd", omap_ssi->dir);
	if (!dir)
		goto rback;
	debugfs_create_file("regs", S_IRUGO, dir, ssi, &ssi_gdd_regs_fops);

	return 0;
rback:
	debugfs_remove_recursive(omap_ssi->dir);

	return -ENOMEM;
}

static void ssi_debug_remove_ctrl(struct hsi_controller *ssi)
{
	struct omap_ssi_controller *omap_ssi = hsi_controller_drvdata(ssi);

	debugfs_remove_recursive(omap_ssi->dir);
}
#endif /* CONFIG_DEBUG_FS */

/*
 * FIXME: Horrible HACK needed until we remove the useless wakeline test
 * in the CMT. To be removed !!!!
 */
void ssi_waketest(struct hsi_client *cl, unsigned int enable)
{
	struct hsi_port *port = hsi_get_port(cl);
	struct omap_ssi_port *omap_port = hsi_port_drvdata(port);
	struct hsi_controller *ssi = to_hsi_controller(port->device.parent);
	struct omap_ssi_controller *omap_ssi = hsi_controller_drvdata(ssi);

	omap_port->wktest = !!enable;
	if (omap_port->wktest) {
		pm_runtime_get_sync(ssi->device.parent);
		writel_relaxed(SSI_WAKE(0),
				omap_ssi->sys + SSI_SET_WAKE_REG(port->num));
	} else {
		writel_relaxed(SSI_WAKE(0),
				omap_ssi->sys +	SSI_CLEAR_WAKE_REG(port->num));
		pm_runtime_put(ssi->device.parent);
	}
}
EXPORT_SYMBOL_GPL(ssi_waketest);

static void ssi_gdd_complete(struct hsi_controller *ssi, unsigned int lch)
{
	struct omap_ssi_controller *omap_ssi = hsi_controller_drvdata(ssi);
	struct hsi_msg *msg = omap_ssi->gdd_trn[lch].msg;
	struct hsi_port *port = to_hsi_port(msg->cl->device.parent);
	struct omap_ssi_port *omap_port = hsi_port_drvdata(port);
	unsigned int dir;
	u32 csr;
	u32 val;

	spin_lock(&omap_ssi->lock);

	val = readl(omap_ssi->sys + SSI_GDD_MPU_IRQ_ENABLE_REG);
	val &= ~SSI_GDD_LCH(lch);
	writel_relaxed(val, omap_ssi->sys + SSI_GDD_MPU_IRQ_ENABLE_REG);

	if (msg->ttype == HSI_MSG_READ) {
		dir = DMA_FROM_DEVICE;
		val = SSI_DATAAVAILABLE(msg->channel);
		pm_runtime_put(omap_port->pdev);
	} else {
		dir = DMA_TO_DEVICE;
		val = SSI_DATAACCEPT(msg->channel);
		/* Keep clocks reference for write pio event */
	}
	dma_unmap_sg(&ssi->device, msg->sgt.sgl, msg->sgt.nents, dir);
	csr = readw(omap_ssi->gdd + SSI_GDD_CSR_REG(lch));
	omap_ssi->gdd_trn[lch].msg = NULL; /* release GDD lch */
	dev_dbg(&port->device, "DMA completed ch %d ttype %d\n",
				msg->channel, msg->ttype);
	spin_unlock(&omap_ssi->lock);
	if (csr & SSI_CSR_TOUR) { /* Timeout error */
		msg->status = HSI_STATUS_ERROR;
		msg->actual_len = 0;
		spin_lock(&omap_port->lock);
		list_del(&msg->link); /* Dequeue msg */
		spin_unlock(&omap_port->lock);

		list_add_tail(&msg->link, &omap_port->errqueue);
		schedule_delayed_work(&omap_port->errqueue_work, 0);
		return;
	}
	spin_lock(&omap_port->lock);
	val |= readl(omap_ssi->sys + SSI_MPU_ENABLE_REG(port->num, 0));
	writel_relaxed(val, omap_ssi->sys + SSI_MPU_ENABLE_REG(port->num, 0));
	spin_unlock(&omap_port->lock);

	msg->status = HSI_STATUS_COMPLETED;
	msg->actual_len = sg_dma_len(msg->sgt.sgl);
}

static void ssi_gdd_tasklet(unsigned long dev)
{
	struct hsi_controller *ssi = (struct hsi_controller *)dev;
	struct omap_ssi_controller *omap_ssi = hsi_controller_drvdata(ssi);
	void __iomem *sys = omap_ssi->sys;
	unsigned int lch;
	u32 status_reg;

	pm_runtime_get(ssi->device.parent);

	if (!pm_runtime_active(ssi->device.parent)) {
		dev_warn(ssi->device.parent, "ssi_gdd_tasklet called without runtime PM!\n");
		pm_runtime_put(ssi->device.parent);
		return;
	}

	status_reg = readl(sys + SSI_GDD_MPU_IRQ_STATUS_REG);
	for (lch = 0; lch < SSI_MAX_GDD_LCH; lch++) {
		if (status_reg & SSI_GDD_LCH(lch))
			ssi_gdd_complete(ssi, lch);
	}
	writel_relaxed(status_reg, sys + SSI_GDD_MPU_IRQ_STATUS_REG);
	status_reg = readl(sys + SSI_GDD_MPU_IRQ_STATUS_REG);

	pm_runtime_put(ssi->device.parent);

	if (status_reg)
		tasklet_hi_schedule(&omap_ssi->gdd_tasklet);
	else
		enable_irq(omap_ssi->gdd_irq);

}

static irqreturn_t ssi_gdd_isr(int irq, void *ssi)
{
	struct omap_ssi_controller *omap_ssi = hsi_controller_drvdata(ssi);

	tasklet_hi_schedule(&omap_ssi->gdd_tasklet);
	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static unsigned long ssi_get_clk_rate(struct hsi_controller *ssi)
{
	struct omap_ssi_controller *omap_ssi = hsi_controller_drvdata(ssi);
	unsigned long rate = clk_get_rate(omap_ssi->fck);
	return rate;
}

static int ssi_clk_event(struct notifier_block *nb, unsigned long event,
								void *data)
{
	struct omap_ssi_controller *omap_ssi = container_of(nb,
					struct omap_ssi_controller, fck_nb);
	struct hsi_controller *ssi = to_hsi_controller(omap_ssi->dev);
	struct clk_notifier_data *clk_data = data;
	struct omap_ssi_port *omap_port;
	int i;

	switch (event) {
	case PRE_RATE_CHANGE:
		dev_dbg(&ssi->device, "pre rate change\n");

		for (i = 0; i < ssi->num_ports; i++) {
			omap_port = omap_ssi->port[i];

			if (!omap_port)
				continue;

			/* Workaround for SWBREAK + CAwake down race in CMT */
			disable_irq(omap_port->wake_irq);

			/* stop all ssi communication */
			pinctrl_pm_select_idle_state(omap_port->pdev);
			udelay(1); /* wait for racing frames */
		}

		break;
	case ABORT_RATE_CHANGE:
		dev_dbg(&ssi->device, "abort rate change\n");
		/* Fall through */
	case POST_RATE_CHANGE:
		dev_dbg(&ssi->device, "post rate change (%lu -> %lu)\n",
			clk_data->old_rate, clk_data->new_rate);
		omap_ssi->fck_rate = DIV_ROUND_CLOSEST(clk_data->new_rate, 1000); /* KHz */

		for (i = 0; i < ssi->num_ports; i++) {
			omap_port = omap_ssi->port[i];

			if (!omap_port)
				continue;

			omap_ssi_port_update_fclk(ssi, omap_port);

			/* resume ssi communication */
			pinctrl_pm_select_default_state(omap_port->pdev);
			enable_irq(omap_port->wake_irq);
		}

		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static int ssi_get_iomem(struct platform_device *pd,
		const char *name, void __iomem **pbase, dma_addr_t *phy)
{
	struct resource *mem;
	void __iomem *base;
	struct hsi_controller *ssi = platform_get_drvdata(pd);

	mem = platform_get_resource_byname(pd, IORESOURCE_MEM, name);
	base = devm_ioremap_resource(&ssi->device, mem);
	if (IS_ERR(base))
		return PTR_ERR(base);

	*pbase = base;

	if (phy)
		*phy = mem->start;

	return 0;
}

static int ssi_add_controller(struct hsi_controller *ssi,
						struct platform_device *pd)
{
	struct omap_ssi_controller *omap_ssi;
	int err