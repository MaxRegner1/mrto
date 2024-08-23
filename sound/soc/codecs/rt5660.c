/*
 * rt5660.c  --  RT5660 ALSA SoC audio codec driver
 *
 * Copyright 2016 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/acpi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "rl6231.h"
#include "rt5660.h"

#define RT5660_DEVICE_ID 0x6338

#define RT5660_PR_RANGE_BASE (0xff + 1)
#define RT5660_PR_SPACING 0x100

#define RT5660_PR_BASE (RT5660_PR_RANGE_BASE + (0 * RT5660_PR_SPACING))

static const struct regmap_range_cfg rt5660_ranges[] = {
	{ .name = "PR", .range_min = RT5660_PR_BASE,
	  .range_max = RT5660_PR_BASE + 0xf3,
	  .selector_reg = RT5660_PRIV_INDEX,
	  .selector_mask = 0xff,
	  .selector_shift = 0x0,
	  .window_start = RT5660_PRIV_DATA,
	  .window_len = 0x1, },
};

static const struct reg_sequence rt5660_patch[] = {
	{ RT5660_ALC_PGA_C&Es