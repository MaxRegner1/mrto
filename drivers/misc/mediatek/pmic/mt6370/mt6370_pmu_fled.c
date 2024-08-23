/*
 *  Copyright (C) 2017 MediaTek Inc.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include "../../flashlight/richtek/rtfled.h"

#include "inc/mt6370_pmu.h"
#include "inc/mt6370_pmu_fled.h"
#include "inc/mt6370_pmu_charger.h"

#define MT6370_PMU_FLED_DRV_VERSION	"1.0.3_MTK"

static DEFINE_MUTEX(fled_lock);

static u8 mt6370_fled_inited;
static u8 mt6370_global_mode = FLASHLIGHT_MODE_OFF;

static u8 mt6370_fled_on;

enum {
	MT6370_FLED1 = 0,
	MT6370_FLED2 = 1,
};

struct mt6370_pmu_fled_data {
	struct rt_fled_dev base;
	struct mt6370_pmu_chip *chip;
	struct device *dev;
	struct platform_device *mt_flash_dev;
	int id;
	unsigned char suspend:1;
	unsigned char fled_ctrl:2; /* fled1, fled2, both */
	unsigned char fled_ctrl_reg;
	unsigned char fled_tor_cur_reg;
	unsigned char fled_strb_cur_reg;
	unsigned char fled_strb_to_reg;
	unsigned char fled_cs_mask;
};

static const char *flashlight_mode_str[FLASHLIGHT_MODE_MAX] = {
	"off", "torch", "flash", "mixed",
	"dual flash", "dual torch", "dual off",
};

static irqreturn_t mt6370_pmu_fled_strbpin_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_fled_torpin_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_fled_tx_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_fled_lvf_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_fled_data *info = data;

	dev_notice(info->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_fled2_short_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_fled_data *info = data;

	dev_notice(info->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_fled1_short_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_fled_data *info = data;

	dev_notice(info->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_fled2_strb_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_fled1_strb_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_fled2_strb_to_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_fled_data *fi = (struct mt6370_pmu_fled_data *)data;

	dev_info(fi->dev, "%s occurred\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_fled1_strb_to_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_fled_data *fi = (struct mt6370_pmu_fled_data *)data;

	dev_info(fi->dev, "%s occurred\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_fled2_tor_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_fled1_tor_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static struct mt6370_pmu_irq_desc mt6370_fled_irq_desc[] = {
	MT6370_PMU_IRQDESC(fled_strbpin),
	MT6370_PMU_IRQDESC(fled_torpin),
	MT6370_PMU_IRQDESC(fled_tx),
	MT6370_PMU_IRQDESC(fled_lvf),
	MT6370_PMU_IRQDESC(fled2_short),
	MT6370_PMU_IRQDESC(fled1_short),
	MT6370_PMU_IRQDESC(fled2_strb),
	MT6370_PMU_IRQDESC(fled1_strb),
	MT6370_PMU_IRQDESC(fled2_strb_to),
	MT6370_PMU_IRQDESC(fled1_strb_to),
	MT6370_PMU_IRQDESC(fled2_tor),
	MT6370_PMU_IRQDESC(fled1_tor),
};

static void mt6370_pmu_fled_irq_register(struct platform_device *pdev)
{
	struct resource *res;
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(mt6370_fled_irq_desc); i++) {
		if (!mt6370_fled_irq_desc[i].name)
			continue;
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
				mt6370_fled_irq_desc[i].name);
		if (!res)
			continue;
		ret = devm_request_threaded_irq(&pdev->dev, res->start, NULL,
				mt6370_fled_irq_desc[i].irq_handler,
				IRQF_TRIGGER_FALLING,
				mt6370_fled_irq_desc[i].name,
				platform_get_drvdata(pdev));
		if (ret < 0) {
			dev_err(&pdev->dev, "request %s irq fail\n", res->name);
			continue;
		}
		mt6370_fled_irq_desc[i].irq = res->start;
	}
}

static inline int mt6370_fled_parse_dt(struct device *dev,
				struct mt6370_pmu_fled_data *fi)
{
	struct device_node *np = dev->of_node;
	int ret = 0;
	u32 val = 0;
	unsigned char regval;

	pr_info("%s start\n", __func__);
	if (!np) {
		pr_err("%s cannot mt6370 fled dts node\n", __func__);
		return -ENODEV;
	}

#if 0
	ret = of_property_read_u32(np, "fled_enable", &val);
	if (ret < 0) {
		pr_err("%s default enable fled%d\n", __func__, fi->id+1);
	} else {
		if (val) {
			pr_info("%s enable fled%d\n", __func__, fi->id+1);
			mt6370_pmu_reg_set_bit(fi->chip,
				MT6370_PMU_REG_FLEDEN, fi->fled_cs_mask);
		} else {
			pr_info("%s disable fled%d\n", __func__, fi->id+1);
			mt6370_pmu_reg_clr_bit(fi->chip,
				MT6370_PMU_REG_FLEDEN, fi->fled_cs_mask);
		}
	}
#endif

	ret = of_property_read_u32(np, "torch_cur", &val);
	if (ret < 0)
		pr_err("%s use default torch cur\n", __func__);
	else {
		pr_info("%s use torch cur %d\n", __func__, val);
		regval = (val > 400000) ? 30 : (val - 25000)/12500;
		mt6370_pmu_reg_update_bits(fi->chip,
				fi->fled_tor_cur_reg,
				MT6370_FLED_TORCHCUR_MASK,
				regval << MT6370_FLED_TORCHCUR_SHIFT);
	}

	ret = of_property_read_u32(np, "strobe_cur", &val);
	if (ret < 0)
		pr_err("%s use default strobe cur\n", __func__);
	else {
		pr_info("%s use strobe cur %d\n", __func__, val);
		regval = (val > 1500000) ? 112 : (val - 100000)/12500;
		mt6370_pmu_reg_update_bits(fi->chip,
				fi->fled_strb_cur_reg,
				MT6370_FLED_STROBECUR_MASK,
				regval << MT6370_FLED_STROBECUR_SHIFT);
	}

	ret = of_property_read_u32(np, "strobe_timeout", &val);
	if (ret < 0)
		pr_err("%s use default strobe timeout\n", __func__);
	else {
		pr_info("%s use strobe timeout %d\n", __func__, val);
		regval = (val > 2432) ? 74 : (val - 64)/32;
		mt6370_pmu_reg_update_bits(fi->chip,
				MT6370_PMU_REG_FLEDSTRBCTRL,
				MT6370_FLED_STROBE_TIMEOUT_MASK,
				regval << MT6370_FLED_STROBE_TIMEOUT_SHIFT);
	}
	return 0;
}

static struct flashlight_properties mt6370_fled_props = {
	.type = FLASHLIGHT_TYPE_LED,
	.torch_brightness = 0,
	.torch_max_brightness = 30, /* 00000 ~ 11110 */
	.strobe_brightness = 0,
	.strobe_max_brightness = 255, /* 0000000 ~ 1111111 */
	.strobe_delay = 0,
	.strobe_timeout = 0,
	.alias_name = "mt6370-fled",
};

static int mt6370_fled_reg_init(struct mt6370_pmu_fled_data *info)
{
	/* TBD */
	return 0;
}

static int mt6370_fled_init(struct rt_fled_dev *info)
{
	struct mt6370_pmu_fled_data *fi = (struct mt6370_pmu_fled_data *)info;
	int ret = 0;

	ret = mt6370_fled_reg_init(fi);
	if (ret < 0)
		dev_err(fi->dev, "init mt6370 fled register fail\n");
	return ret;
}

static int mt6370_fled_suspend(struct rt_fled_dev *info, pm_message_t state)
{
	struct mt6370_pmu_fled_data *fi = (struct mt6370_pmu_fled_data *)info;

	fi->suspend = 1;
	return 0;
}

static int mt6370_fled_resume(struct rt_fled_dev *info)
{
	struct mt6370_pmu_fled_data *fi = (struct mt6370_pmu_fled_data *)info;

	fi->suspend = 0;
	return 0;
}

static int mt6370_fled_set_mode(struct rt_fled_dev *info,
					enum flashlight_mode mode)
{
	struct mt6370_pmu_fled_data *fi = (struct mt6370_pmu_fled_data *)info;
	int ret = 0;
	u8 val, mask;
	bool hz_en = false, cfo_en = true;

	switch (mode) {
	case FLASHLIGHT_MODE_FLASH:
	case FLASHLIGHT_MODE_DUAL_FLASH:
		ret = mt6370_pmu_reg_test_bit(fi->chip, MT6370_PMU_REG_CHGCTRL1,
				MT6370_SHIFT_HZ_EN, &hz_en);
		if (ret >= 0 && hz_en) {
			dev_err(fi->dev, "%s WARNING\n", __func__);
			dev_err(fi->dev, "%s set %s mode with HZ=1\n",
					 __func__, flashlight_mode_str[mode]);
		}

		ret = mt6370_pmu_reg_test_bit(fi->chip, MT6370_PMU_REG_CHGCTRL2,
				MT6370_SHIFT_CFO_EN, &cfo_en);
		if (ret >= 0 && !cfo_en) {
			dev_err(fi->dev, "%s WARNING\n", __func__);
			dev_err(fi->dev, "%s set %s mode with CFO=0\n",
					 __func__, flashlight_mode_str[mode]);
		}
		break;
	default:
		break;
	}

	mutex_lock(&fled_lock);
	switch (mode) {
	case FLASHLIGHT_MODE_TORCH:
		if (mt6370_global_mode == FLASHLIGHT_MODE_FLASH)
			break;
		ret |= mt6370_pmu_reg_clr_bit(fi->chip,
			MT6370_PMU_REG_FLEDEN, MT6370_STROBE_EN_MASK);
		udelay(500);
		ret |= mt6370_pmu_reg_set_bit(fi->chip, MT6370_PMU_REG_FLEDEN,
				fi->id == MT6370_FLED1 ? 0x02 : 0x01);
		ret |= mt6370_pmu_reg_set_bit(fi->chip,
				MT6370_PMU_REG_FLEDEN, MT6370_TORCH_EN_MASK);
		udelay(500);
		dev_info(fi->dev, "set to torch mode with 500 us delay\n");
		mt6370_global_mode = mode;
		if (fi->id == MT6370_FLED1)
			mt6370_fled_on |= 1 << MT6370_FLED1;
		if (fi->id == MT6370_FLED2)
			mt6370_fled_on |= 1 << MT6370_FLED2;
		break;
	case FLASHLIGHT_MODE_FLASH:
		ret = mt6370_pmu_reg_clr_bit(fi->chip,
			MT6370_PMU_REG_FLEDEN, MT6370_STROBE_EN_MASK);
		udelay(400);
		ret |= mt6370_pmu_reg_set_bit(fi->chip, MT6370_PMU_REG_FLEDEN,
			fi->id == MT6370_FLED1 ? 0x02 : 0x01);
		ret |= mt6370_pmu_reg_set_bit(fi->chip,
			MT6370_PMU_REG_FLEDEN, MT6370_STROBE_EN_MASK);
		mdelay(5);
		dev_info(fi->dev, "set to flash mode with 400/4500 us delay\n");
		mt6370_global_mode = mode;
		if (fi->id == MT6370_FLED1)
			mt6370_fled_on |= 1 << MT6370_FLED1;
		if (fi->id == MT6370_FLED2)
			mt6370_fled_on |= 1 << MT6370_FLED2;
		break;
	case FLASHLIGHT_MODE_OFF:
		ret = mt6370_pmu_reg_clr_bit(fi->chip,
				MT6370_PMU_REG_FLEDEN,
				fi->id == MT6370_FLED1 ? 0x02 : 0x01);
		dev_info(fi->dev, "set to off mode\n");
		if (fi->id == MT6370_FLED1)
			mt6370_fled_on &= ~(1 << MT6370_FLED1);
		if (fi->id == MT6370_FLED2)
			mt6370_fled_on &= ~(1 << MT6370_FLED2);
		if (mt6370_fled_on == 0)
			mt6370_global_mode = mode;
		break;
	case FLASHLIGHT_MODE_DUAL_FLASH:
		if (fi->id == MT6370_FLED2)
			goto out;
		/* strobe off */
		ret = mt6370_pmu_reg_clr_bit(fi->chip, MT6370_PMU_REG_FLEDEN,
					     MT6370_STROBE_EN_MASK);
		if (ret < 0)
			break;
		udelay(400);
		/* fled en/strobe on */
		val = BIT(MT6370_FLED1) | BIT(MT6370_FLED2) |
			MT6370_STROBE_EN_MASK;
		mask = val;
		ret = mt6370_pmu_reg_update_bits(fi->chip,
						 MT6370_PMU_REG_FLEDEN,
						 mask, val);
		if (ret < 0)
			break;
		mt6370_global_mode = mode;
		mt6370_fled_on |= (BIT(MT6370_FLED1) | BIT(MT6370_FLED2));
		break;
	case FLASHLIGHT_MODE_DUAL_TORCH:
		if (fi->id == MT6370_FLED2)
			goto out;
		if (mt6370_global_mode == FLASHLIGHT_MODE_FLASH ||
		    mt6370_global_mode == FLASHLIGHT_MODE_DUAL_FLASH)
			goto out;
		/* Fled en/Strobe off/Torch on */
		ret = mt6370_pmu_reg_clr_bit(fi->chip, MT6370_PMU_REG_FLEDEN,
					     MT6370_STROBE_EN_MASK);
		if (ret < 0)
			break;
		udelay(500);
		val = BIT(MT6370_FLED1) | BIT(MT6370_FLED2) |
			MT6370_TORCH_EN_MASK;
		ret = mt6370_pmu_reg_set_bit(fi->chip,
					     MT6370_PMU_REG_FLEDEN, val);
		if (ret < 0)
			break;
		udelay(500);
		mt6370_global_mode = mode;
		mt6370_fled_on |= (BIT(MT6370_FLED1) | BIT(MT6370_FLED2));
		break;
	case FLASHLIGHT_MODE_DUAL_OFF:
		if (fi->id == MT6370_FLED2)
			goto out;
		ret = mt6370_pmu_reg_clr_bit(fi->chip, MT6370_PMU_REG_FLEDEN,
					 BIT(MT6370_FLED1) | BIT(MT6370_FLED2));
		if (ret < 0)
			break;
		mt6370_fled_on = 0;
		mt6370_global_mode = FLASHLIGHT_MODE_OFF;
		break;
	default:
		mutex_unlock(&fled_lock);
		return -EINVAL;
	}
	if (ret < 0)
		dev_info(fi->dev, "%s set %s mode fail\n", __func__,
			 flashlight_mode_str[mode]);
	else
		dev_info(fi->dev, "%s set %s\n", __func__,
			 flashlight_mode_str[mode]);
out:
	mutex_unlock(&fled_lock);
	return ret;
}

static int mt6370_fled_get_mode(struct rt_fled_dev *info)
{
	struct mt6370_pmu_fled_data *fi = (struct mt6370_pmu_fled_data *)info;
	int ret;

	if (fi->id == MT6370_FLED2) {
		pr_err("%s FLED2 not support get mode\n", __func__);
		return 0;
	}

	ret = mt6370_pmu_reg_read(fi->chip, MT6370_PMU_REG_FLEDEN);
	if (ret < 0)
		return -EINVAL;

	if (ret & MT6370_STROBE_EN_MASK)
		return FLASHLIGHT_MODE_FLASH;
	else if (ret & MT6370_TORCH_EN_MASK)
		return FLASHLIGHT_MODE_TORCH;
	else
		return FLASHLIGHT_MODE_OFF;
}

static int mt6370_fled_strobe(struct rt_fled_dev *info)
{
	return mt6370_fled_set_mode(info, FLASHLIGHT_MODE_FLASH);
}

static int mt6370_fled_torch_current_list(
			struct rt_fled_dev *info, int selector)
{

	struct mt6370_pmu_fled_data *fi = (struct mt6370_pmu_fled_data *)info;

	return (selector > fi->base.init_props->torch_max_brightness) ?
		-EINVAL : 25000 + selector * 12500;
}

static int mt6370_fled_strobe_current_list(struct rt_fled_dev *info,
							int selector)
{
	struct mt6370_pmu_fled_data *fi = (struct mt6370_pmu_fled_data *)info;

	if (selector > fi->base.init_props->strobe_max_brightness)
		return -EINVAL;
	if (selector < 128)
		return 50000 + selector * 12500;
	else
		return 25000 + (selector - 128) * 6250;
}

static unsigned int mt6370_timeout_level_list[] = {
	50000, 75000, 100000, 125000, 150000, 175000, I-list[] = {
	50000, 75000, 100000, 125000, 1ot6370_timeout_level_list[] = {
	50000, 75000, 100000, 125000, 150000, 175000, I-list[] = {
	50000, 75000, 100000, 125000, 1ot6370_timeout_level_list[] = {
	50000, 75000, 100000, 125000, 1500000, 75000, 100000, 125000, 1500000, 75000, 100000, 125000, 1500000, 75000, 100000, 125000, 1500000, 75000, 100000, 125000, 1500000utvnc__);
		return 0;
	}

	ret = mt6370_pmu_reg_read(fi->chip, MT6370_PMU_REG_FLEDEN);
	if (ret < 0)
		return -EINVALI<vI<bnve.init_props->strobe_max_brightness)
		return -EINVAL;
	if (selector < 128)
		return 50000 + selector * 12500;
	el		 -&evt6370_global_mode == FLASHLIGHT_MODE_DUAL_FLASH)
			goto out;
		/* Fled en/Strobe off/Torch on */
		ret = mt6370_p-: