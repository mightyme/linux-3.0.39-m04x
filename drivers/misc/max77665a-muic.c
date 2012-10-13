/*
 * max77665a-muic.c - MUIC driver for the Maxim 77665
 *
 *  Copyright (C) 2012 Meizu Technology Co.Ltd
 *  <jgmai@meizu.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/mfd/max77665.h>
#include <linux/mfd/max77665-private.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>

#define DEV_NAME	"max77665-muic"

enum {
	ADC_GND			= 0x00,
	ADC_REMOVE		= 0x10,
	ADC_OPEN		= 0x1f
};

struct max77665_muic_info {
	struct device		*dev;
	struct max77665_dev	*max77665;
	struct i2c_client	*muic;
	int mhl_insert;
	int host_insert;
	struct regulator *reverse;
	struct delayed_work dwork;
};

static struct max77665_muic_info *g_info;

static int init_max77665_muic(struct max77665_muic_info *info)
{
	int ret;
	struct i2c_client *client = info->muic;
	u8 val, msk;

	val = (0x1 << COMN1SW_SHIFT) | (0x1 << COMP2SW_SHIFT) |
		(0 << MICEN_SHIFT) | (0 << IDBEN_SHIFT);

	msk = COMN1SW_MASK | COMP2SW_MASK | MICEN_MASK | IDBEN_MASK;

	ret = max77665_update_reg(client, MAX77665_MUIC_REG_CTRL1, val, msk);

	return 0;
}

inline static void echi_pm_runtime(int onoff)
{
#ifndef CONFIG_MX_RECOVERY_KERNEL
	pr_info("@@@ %s %d\n", __func__, onoff);
	if(onoff)
		pm_runtime_put_sync(&s5p_device_ehci.dev);
	else
		pm_runtime_get_sync(&s5p_device_ehci.dev);
#endif
}

inline static int max77665a_set_usbid(struct max77665_muic_info *info, int value)
{
	int ret;
	struct i2c_client *client = info->muic;
	u8 val, msk;

	val = (0x1 << COMN1SW_SHIFT) | (0x1 << COMP2SW_SHIFT) |
		(0 << MICEN_SHIFT) | (!!value << IDBEN_SHIFT);

	msk = COMN1SW_MASK | COMP2SW_MASK | MICEN_MASK | IDBEN_MASK;

	ret = max77665_update_reg(client, MAX77665_MUIC_REG_CTRL1, val, msk);

	return 0;
}

static irqreturn_t max77665_muic_isr(int irq, void *dev_id)
{
	struct max77665_muic_info *info = dev_id;
	u8 adc, adclow ,adcerr, adc1k;
	u8 status1;
	int insert = true;

	max77665_read_reg(info->muic, MAX77665_MUIC_REG_STATUS1, &status1);
	adc = status1 & STATUS1_ADC_MASK;
	adclow = !!(status1 & STATUS1_ADCLOW_MASK);
	adcerr = !!(status1 & STATUS1_ADCERR_MASK);
	adc1k = !!(status1 & STATUS1_ADC1K_MASK);

	pr_info("adc 0x%02x, adclow %d, adcerr %d, adc1k %d\n",
			adc, adclow, adcerr, adc1k);

	if(adc1k) {
		info->mhl_insert = true;
		schedule_delayed_work(&info->dwork, 0);
	} else if (adc == ADC_GND) {
		if(!info->host_insert) {
			pr_info("otg connect\n");

			echi_pm_runtime(false);
			if(!regulator_is_enabled(info->reverse))
				regulator_enable(info->reverse);
			info->host_insert = true;
		}
	} else if (adc == ADC_REMOVE || adc == ADC_OPEN) {
		if (info->host_insert) {
			pr_info("otg disconnect\n");

			echi_pm_runtime(true);
			if(regulator_is_enabled(info->reverse))
				regulator_disable(info->reverse);

			info->host_insert = false;
		}

	}
	return IRQ_HANDLED;
}

void check_mhl_connect(void)
{
	struct max77665_muic_info *info = g_info;
	u8  adc1k;
	u8 status1;

	max77665_read_reg(info->muic, MAX77665_MUIC_REG_STATUS1, &status1);
	adc1k = !!(status1 & STATUS1_ADC1K_MASK);

	if(!adc1k) {
		pr_info("adc1k not set\n");
		info->mhl_insert = false;
		schedule_delayed_work(&info->dwork, 0);
	}
}

static void muic_mhl_work(struct work_struct *work)
{
	struct max77665_muic_info *info =
		container_of(work, struct max77665_muic_info, dwork.work);
	if (info->mhl_insert) {
		pr_info("mhl connect\n");

		max77665a_set_usbid(info, true);
	} else {
		pr_info("mhl disconnect\n");

		max77665a_set_usbid(info, false);
	}
}

static int __devinit max77665_muic_probe(struct platform_device *pdev)
{
	struct max77665_dev *max77665 = dev_get_drvdata(pdev->dev.parent);
	struct max77665_muic_info *info;
	int ret = 0;
	int irq;

	pr_info("func:%s\n", __func__);

	info = kzalloc(sizeof(struct max77665_muic_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "%s: failed to allocate info\n", __func__);
		ret = -ENOMEM;
		goto err_return;
	}

	g_info = info;

	info->dev = &pdev->dev;
	info->max77665 = max77665;
	info->muic = max77665->muic;

	info->reverse = regulator_get(NULL, "reverse");
	if (IS_ERR(info->reverse)) {
		dev_err(&pdev->dev, "Failed to get reverse regulator\n");
		goto fail0;
	}

	platform_set_drvdata(pdev, info);

	ret = init_max77665_muic(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to initialize MUIC:%d\n", ret);
		goto fail1;
	}

	INIT_DELAYED_WORK(&info->dwork, muic_mhl_work);

	irq = max77665->irq_base + MAX77665_MUIC_IRQ_INT1_ADC1K;
	ret = request_threaded_irq(irq, 0, max77665_muic_isr,
			0, "max77665_adc1k", info);

	irq = max77665->irq_base + MAX77665_MUIC_IRQ_INT1_ADCLOW;
	ret = request_threaded_irq(irq, 0, max77665_muic_isr,
			0, "max77665_adclow", info);

	return 0;

fail1:
	regulator_put(info->reverse);
fail0:
	platform_set_drvdata(pdev, NULL);
	kfree(info);
err_return:
	return ret;
}

static int __devexit max77665_muic_remove(struct platform_device *pdev)
{
	struct max77665_muic_info *info = platform_get_drvdata(pdev);

	if (info) {
		dev_info(info->dev, "func:%s\n", __func__);
		platform_set_drvdata(pdev, NULL);
		kfree(info);
	}
	return 0;
}

void max77665_muic_shutdown(struct device *dev)
{
	return ;
}

static struct platform_driver max77665_muic_driver = {
	.driver		= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
		.shutdown = max77665_muic_shutdown,
	},
	.probe		= max77665_muic_probe,
	.remove		= __devexit_p(max77665_muic_remove),
};

static int __init max77665_muic_init(void)
{
	pr_info("func:%s\n", __func__);
	return platform_driver_register(&max77665_muic_driver);
}
module_init(max77665_muic_init);

static void __exit max77665_muic_exit(void)
{
	pr_info("func:%s\n", __func__);
	platform_driver_unregister(&max77665_muic_driver);
}
module_exit(max77665_muic_exit);

MODULE_DESCRIPTION("Maxim MAX77665 MUIC driver");
MODULE_AUTHOR("<sukdong.kim@samsung.com>");
MODULE_LICENSE("GPL");
