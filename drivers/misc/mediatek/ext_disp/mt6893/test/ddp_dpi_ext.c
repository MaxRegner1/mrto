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

#include "dpi_dvt_test.h"

#if defined(RDMA_DPI_PATH_SUPPORT) || defined(DPI_DVT_TEST_SUPPORT)

#ifdef BUILD_UBOOT
#include <asm/arch/disp_drv_platform.h>
#else
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include "cmdq_record.h"
#include <disp_drv_log.h>
#endif
#include <debug.h>
#include <mt-plat/sync_write.h>
#include <linux/types.h>
#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#endif
/* #include <mach/irqs.h> */
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#include "mtkfb.h"
#include "ddp_drv.h"
#include "ddp_hal.h"
#include "ddp_manager.h"
#include "ddp_dpi_reg.h"
#include "ddp_reg.h"
#include "ddp_log.h"

#include "dpi_dvt_test.h"
#include "ddp_dpi_ext.h"

#include <linux/of.h>
#include <linux/of_irq.h>
/*#include "mach/eint.h"*/

/* #ifdef DPI_EXT_INREG32 */
/* #undef DPI_EXT_INREG32 */
#define DPI_EXT_INREG32(x)          (__raw_readl((unsigned long *)(x)))
/* #endif */

#define DPI_EXT_OUTREG32(cmdq, addr, val) \
	{\
		mt_reg_sync_writel(val, addr); \
	}

#define DPI_EXT_LOG_PRINT(fmt, arg...)  \
	{\
		pr_debug(fmt, ##arg); \
	}

/***************************DPI DVT Case Start********************************/
int configInterlaceMode(unsigned int resolution)
{
	/*Enable Interlace mode */
	struct DPI_REG_CNTL ctr = DPI_REG->CNTL;
	/*Set LODD,LEVEN Vsize */
	struct DPI_REG_SIZE size = DPI_REG->SIZE = DPI_REG->CNtH<H