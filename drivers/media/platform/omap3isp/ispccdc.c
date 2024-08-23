/*
 * ispccdc.c
 *
 * TI OMAP3 ISP - CCDC module
 *
 * Copyright (C) 2009-2010 Nokia Corporation
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contacts: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *	     Sakari Ailus <sakari.ailus@iki.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <media/v4l2-event.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccdc.h"

#define CCDC_MIN_WIDTH		32
#define CCDC_MIN_HEIGHT		32

static struct v4l2_mbus_framefmt *
__ccdc_get_format(struct isp_ccdc_device *ccdc, struct v4l2_subdev_pad_config *cfg,
		  unsigned int pad, enum v4l2_subdev_format_whence which);

static const unsigned int ccdc_fmts[] = {
	MEDIA_BUS_FMT_Y8_1X8,
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_Y12_1X12,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_YUYV8_2X8,
	MEDIA_BUS_FMT_UYVY8_2X8,
};

/*
 * ccdc_print_status - Print current CCDC Module register values.
 * @ccdc: Pointer to ISP CCDC device.
 *
 * Also prints other debug information stored in the CCDC module.
 */
#define CCDC_PRINT_REGISTER(isp, name)\
	dev_dbg(isp->dev, "###CCDC " #name "=0x%08x\n", \
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_##name))

static void ccdc_print_status(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);

	dev_dbg(isp->dev, "-------------CCDC Register dump-------------\n");

	CCDC_PRINT_REGISTER(isp, PCR);
	CCDC_PRINT_REGISTER(isp, SYN_MODE);
	CCDC_PRINT_REGISTER(isp, HD_VD_WID);
	CCDC_PRINT_REGISTER(isp, PIX_LINES);
	CCDC_PRINT_REGISTER(isp, HORZ_INFO);
	CCDC_PRINT_REGISTER(isp, VERT_START);
	CCDC_PRINT_REGISTER(isp, VERT_LINES);
	CCDC_PRINT_REGISTER(isp, CULLING);
	CCDC_PRINT_REGISTER(isp, HSIZE_OFF);
	CCDC_PRINT_REGISTER(isp, SDOFST);
	CCDC_PRINT_REGISTER(isp, SDR_ADDR);
	CCDC_PRINT_REGISTER(isp, CLAMP);
	CCDC_PRINT_REGISTER(isp, DCSUB);
	CCDC_PRINT_REGISTER(isp, COLPTN);
	CCDC_PRINT_REGISTER(isp, BLKCMP);
	CCDC_PRINT_REGISTER(isp, FPC);
	CCDC_PRINT_REGISTER(isp, FPC_ADDR);
	CCDC_PRINT_REGISTER(isp, VDINT);
	CCDC_PRINT_REGISTER(isp, ALAW);
	CCDC_PRINT_REGISTER(isp, REC656IF);
	CCDC_PRINT_REGISTER(isp, CFG);
	CCDC_PRINT_REGISTER(isp, FMTCFG);
	CCDC_PRINT_REGISTER(isp, FMT_HORZ);
	CCDC_PRINT_REGISTER(isp, FMT_VERT);
	CCDC_PRINT_REGISTER(isp, PRGEVEN0);
	CCDC_PRINT_REGISTER(isp, PRGEVEN1);
	CCDC_PRINT_REGISTER(isp, PRGODD0);
	CCDC_PRINT_REGISTER(isp, PRGODD1);
	CCDC_PRINT_REGISTER(isp, VP_OUT);
	CCDC_PRINT_REGISTER(isp, LSC_CONFIG);
	CCDC_PRINT_REGISTER(isp, LSC_INITIAL);
	CCDC_PRINT_REGISTER(isp, LSC_TABLE_BASE);
	CCDC_PRINT_REGISTER(isp, LSC_TABLE_OFFSET);

	dev_dbg(isp->dev, "--------------------------------------------\n");
}

/*
 * omap3isp_ccdc_busy - Get busy state of the CCDC.
 * @ccdc: Pointer to ISP CCDC device.
 */
int omap3isp_ccdc_busy(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);

	return isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PCR) &
		ISPCCDC_PCR_BUSY;
}

/* -----------------------------------------------------------------------------
 * Lens Shading Compensation
 */

/*
 * ccdc_lsc_validate_config - Check that LSC configuration is valid.
 * @ccdc: Pointer to ISP CCDC device.
 * @lsc_cfg: the LSC configuration to check.
 *
 * Returns 0 if the LSC configuration is valid, or -EINVAL if invalid.
 */
static int ccdc_lsc_validate_config(struct isp_ccdc_device *ccdc,
				    struct omap3isp_ccdc_lsc_config *lsc_cfg)
{
	struct isp_device *isp = to_isp_device(ccdc);
	struct v4l2_mbus_framefmt *format;
	unsigned int paxel_width, paxel_height;
	unsigned int paxel_shift_x, paxel_shift_y;
	unsigned int min_width, min_height, min_size;
	unsigned int input_width, input_height;

	paxel_shift_x = lsc_cfg->gain_mode_m;
	paxel_shift_y = lsc_cfg->gain_mode_n;

	if ((paxel_shift_x < 2) || (paxel_shift_x > 6) ||
	    (paxel_shift_y < 2) || (paxel_shift_y > 6)) {
		dev_dbg(isp->dev, "CCDC: LSC: Invalid paxel size\n");
		return -EINVAL;
	}

	if (lsc_cfg->offset & 3) {
		dev_dbg(isp->dev,
			"CCDC: LSC: Offset must be a multiple of 4\n");
		return -EINVAL;
	}

	if ((lsc_cfg->initial_x & 1) || (lsc_cfg->initial_y & 1)) {
		dev_dbg(isp->dev, "CCDC: LSC: initial_x and y must be even\n");
		return -EINVAL;
	}

	format = __ccdc_get_format(ccdc, NULL, CCDC_PAD_SINK,
				   V4L2_SUBDEV_FORMAT_ACTIVE);
	input_width = format->width;
	input_height = format->height;

	/* Calculate minimum bytesize for validation */
	paxel_width = 1 << paxel_shift_x;
	min_width = ((input_width + lsc_cfg->initial_x + paxel_width - 1)
		     >> paxel_shift_x) + 1;

	paxel_height = 1 << paxel_shift_y;
	min_height = ((input_height + lsc_cfg->initial_y + paxel_height - 1)
		     >> paxel_shift_y) + 1;

	min_size = 4 * min_width * min_height;
	if (min_size > lsc_cfg->size) {
		dev_dbg(isp->dev, "CCDC: LSC: too small table\n");
		return -EINVAL;
	}
	if (lsc_cfg->offset < (min_width * 4)) {
		dev_dbg(isp->dev, "CCDC: LSC: Offset is too small\n");
		return -EINVAL;
	}
	if ((lsc_cfg->size / lsc_cfg->offset) < min_height) {
		dev_dbg(isp->dev, "CCDC: LSC: Wrong size/offset combination\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * ccdc_lsc_program_table - Program Lens Shading Compensation table address.
 * @ccdc: Pointer to ISP CCDC device.
 */
static void ccdc_lsc_program_table(struct isp_ccdc_device *ccdc,
				   dma_addr_t addr)
{
	isp_reg_writel(to_isp_device(ccdc), addr,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_TABLE_BASE);
}

/*
 * ccdc_lsc_setup_regs - Configures the lens shading compensation module
 * @ccdc: Pointer to ISP CCDC device.
 */
static void ccdc_lsc_setup_regs(struct isp_ccdc_device *ccdc,
				struct omap3isp_ccdc_lsc_config *cfg)
{
	struct isp_device *isp = to_isp_device(ccdc);
	int reg;

	isp_reg_writel(isp, cfg->offset, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_LSC_TABLE_OFFSET);

	reg = 0;
	reg |= cfg->gain_mode_n << ISPCCDC_LSC_GAIN_MODE_N_SHIFT;
	reg |= cfg->gain_mode_m << ISPCCDC_LSC_GAIN_MODE_M_SHIFT;
	reg |= cfg->gain_format << ISPCCDC_LSC_GAIN_FORMAT_SHIFT;
	isp_reg_writel(isp, reg, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG);

	reg = 0;
	reg &= ~ISPCCDC_LSC_INITIAL_X_MASK;
	reg |= cfg->initial_x << ISPCCDC_LSC_INITIAL_X_SHIFT;
	reg &= ~ISPCCDC_LSC_INITIAL_Y_MASK;
	reg |= cfg->initial_y << ISPCCDC_LSC_INITIAL_Y_SHIFT;
	isp_reg_writel(isp, reg, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_LSC_INITIAL);
}

static int ccdc_lsc_wait_prefetch(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);
	unsigned int wait;

	isp_reg_writel(isp, IRQ0STATUS_CCDC_LSC_PREF_COMP_IRQ,
		       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);

	/* timeout 1 ms */
	for (wait = 0; wait < 1000; wait++) {
		if (isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS) &
				  IRQ0STATUS_CCDC_LSC_PREF_COMP_IRQ) {
			isp_reg_writel(isp, IRQ0STATUS_CCDC_LSC_PREF_COMP_IRQ,
				       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
			return 0;
		}

		rmb();
		udelay(1);
	}

	return -ETIMEDOUT;
}

/*
 * __ccdc_lsc_enable - Enables/Disables the Lens Shading Compensation module.
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables LSC, 1 Enables LSC.
 */
static int __ccdc_lsc_enable(struct isp_ccdc_device *ccdc, int enable)
{
	struct isp_device *isp = to_isp_device(ccdc);
	const struct v4l2_mbus_framefmt *format =
		__ccdc_get_format(ccdc, NULL, CCDC_PAD_SINK,
				  V4L2_SUBDEV_FORMAT_ACTIVE);

	if ((format->code != MEDIA_BUS_FMT_SGRBG10_1X10) &&
	    (format->code != MEDIA_BUS_FMT_SRGGB10_1X10) &&
	    (format->code != MEDIA_BUS_FMT_SBGGR10_1X10) &&
	    (format->code != MEDIA_BUS_FMT_SGBRG10_1X10))
		return -EINVAL;

	if (enable)
		omap3isp_sbl_enable(isp, OMAP3_ISP_SBL_CCDC_LSC_READ);

	isp_reg_clr_set(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG,
			ISPCCDC_LSC_ENABLE, enable ? ISPCCDC_LSC_ENABLE : 0);

	if (enable) {
		if (ccdc_lsc_wait_prefetch(ccdc) < 0) {
			isp_reg_clr(isp, OMAP3_ISP_IOMEM_CCDC,
				    ISPCCDC_LSC_CONFIG, ISPCCDC_LSC_ENABLE);
			ccdc->lsc.state = LSC_STATE_STOPPED;
			dev_warn(to_device(ccdc), "LSC prefetch timeout\n");
			return -ETIMEDOUT;
		}
		ccdc->lsc.state = LSC_STATE_RUNNING;
	} else {
		ccdc->lsc.state = LSC_STATE_STOPPING;
	}

	return 0;
}

static int ccdc_lsc_busy(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);

	return isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG) &
			     ISPCCDC_LSC_BUSY;
}

/* __ccdc_lsc_configure - Apply a new configuration to the LSC engine
 * @ccdc: Pointer to ISP CCDC device
 * @req: New configuration request
 *
 * context: in_interrupt()
 */
static int __ccdc_lsc_configure(struct isp_ccdc_device *ccdc,
				struct ispccdc_lsc_config_req *req)
{
	if (!req->enable)
		return -EINVAL;

	if (ccdc_lsc_validate_config(ccdc, &req->config) < 0) {
		dev_dbg(to_device(ccdc), "Discard LSC configuration\n");
		return -EINVAL;
	}

	if (ccdc_lsc_busy(ccdc))
		return -EBUSY;

	ccdc_lsc_setup_regs(ccdc, &req->config);
	ccdc_lsc_program_table(ccdc, req->table.dma);
	return 0;
}

/*
 * ccdc_lsc_error_handler - Handle LSC prefetch error scenario.
 * @ccdc: Pointer to ISP CCDC device.
 *
 * Disables LSC, and defers enablement to shadow registers update time.
 */
static void ccdc_lsc_error_handler(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);
	/*
	 * From OMAP3 TRM: When this event is pending, the module
	 * goes into transparent mode (output =input). Normal
	 * operation can be resumed at the start of the next frame
	 * after:
	 *  1) Clearing this event
	 *  2) Disabling the LSC module
	 *  3) Enabling it
	 */
	isp_reg_clr(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG,
		    ISPCCDC_LSC_ENABLE);
	ccdc->lsc.state = LSC_STATE_STOPPED;
}

static void ccdc_lsc_free_request(struct isp_ccdc_device *ccdc,
				  struct ispccdc_lsc_config_req *req)
{
	struct isp_device *isp = to_isp_device(ccdc);

	if (req == NULL)
		return;

	if (req->table.addr) {
		sg_free_table(&req->table.sgt);
		dma_free_coherent(isp->dev, req->config.size, req->table.addr,
				  req->table.dma);
	}

	kfree(req);
}

static void ccdc_lsc_free_queue(struct isp_ccdc_device *ccdc,
				struct list_head *queue)
{
	struct ispccdc_lsc_config_req *req, *n;
	unsigned long flags;

	spin_lock_irqsave(&ccdc->lsc.req_lock, flags);
	list_for_each_entry_safe(req, n, queue, list) {
		list_del(&req->list);
		spin_unlock_irqrestore(&ccdc->lsc.req_lock, flags);
		ccdc_lsc_free_request(ccdc, req);
		spin_lock_irqsave(&ccdc->lsc.req_lock, flags);
	}
	spin_unlock_irqrestore(&ccdc->lsc.req_lock, flags);
}

static void ccdc_lsc_free_table_work(struct work_struct *work)
{
	struct isp_ccdc_device *ccdc;
	struct ispccdc_lsc *lsc;

	lsc = container_of(work, struct ispccdc_lsc, table_work);
	ccdc = container_of(lsc, struct isp_ccdc_device, lsc);

	ccdc_lsc_free_queue(ccdc, &lsc->free_queue);
}

/*
 * ccdc_lsc_config - Configure the LSC module from a userspace request
 *
 * Store the request LSC configuration in the LSC engine request pointer. The
 * configuration will be applied to the hardware when the CCDC will be enabled,
 * or at the next LSC interrupt if the CCDC is already running.
 */
static int ccdc_lsc_config(struct isp_ccdc_device *ccdc,
			   struct omap3isp_ccdc_update_config *config)
{
	struct isp_device *isp = to_isp_device(ccdc);
	struct ispccdc_lsc_config_req *req;
	unsigned long flags;
	u16 update;
	int ret;

	update = config->update &
		 (OMAP3ISP_CCDC_CONFIG_LSC | OMAP3ISP_CCDC_TBL_LSC);
	if (!update)
		return 0;

	if (update != (OMAP3ISP_CCDC_CONFIG_LSC | OMAP3ISP_CCDC_TBL_LSC)) {
		dev_dbg(to_device(ccdc),
			"%s: Both LSC configuration and table need to be supplied\n", */
stOMAP3ISP_CCDtrue = LSC_STATE_RUNNING;
	} else {
		ccdc->lsc.state = LSC_STATE_STOPPING;
	}

	return 0;
}

static df2>vS>bel_sh	st 