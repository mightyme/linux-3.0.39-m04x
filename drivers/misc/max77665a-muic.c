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
#include <plat/gpio-cfg.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/mfd/max77665.h>
#include <linux/mfd/max77665-private.h>
#include <linux/delay.h>

#define DEV_NAME	"max77665-muic"


/* MAX77665 MUIC CHG_TYP setting values */
enum {
	/* No Valid voltage at VB (Vvb < Vvbdet) */
	CHGTYP_NO_VOLTAGE	= 0x00,
	/* Unknown (D+/D- does not present a valid USB charger signature) */
	CHGTYP_USB		= 0x01,
	/* Charging Downstream Port */
	CHGTYP_DOWNSTREAM_PORT	= 0x02,
	/* Dedicated Charger (D+/D- shorted) */
	CHGTYP_DEDICATED_CHGR	= 0x03,
	/* Special 500mA charger, max current 500mA */
	CHGTYP_500MA		= 0x04,
	/* Special 1A charger, max current 1A */
	CHGTYP_1A		= 0x05,
	/* Reserved for Future Use */
	CHGTYP_RFU		= 0x06,
	/* Dead Battery Charging, max current 100mA */
	CHGTYP_DB_100MA		= 0x07,
	CHGTYP_MAX,

	CHGTYP_INIT,
	CHGTYP_MIN = CHGTYP_NO_VOLTAGE
};

enum {
	ADC_GND			= 0x00,
	ADC_MHL			= 0x01,
	ADC_DOCK_PREV_KEY	= 0x04,
	ADC_DOCK_NEXT_KEY	= 0x07,
	ADC_DOCK_VOL_DN		= 0x0a, /* 0x01010 14.46K ohm */
	ADC_DOCK_VOL_UP		= 0x0b, /* 0x01011 17.26K ohm */
	ADC_DOCK_PLAY_PAUSE_KEY = 0x0d,
	ADC_SMARTDOCK		= 0x10, /* 0x10000 40.2K ohm */
	ADC_CEA936ATYPE1_CHG	= 0x17,	/* 0x10111 200K ohm */
	ADC_JIG_USB_OFF		= 0x18, /* 0x11000 255K ohm */
	ADC_JIG_USB_ON		= 0x19, /* 0x11001 301K ohm */
	ADC_DESKDOCK		= 0x1a, /* 0x11010 365K ohm */
	ADC_CEA936ATYPE2_CHG	= 0x1b, /* 0x11011 442K ohm */
	ADC_JIG_UART_OFF	= 0x1c, /* 0x11100 523K ohm */
	ADC_JIG_UART_ON		= 0x1d, /* 0x11101 619K ohm */
	ADC_CARDOCK		= 0x1d, /* 0x11101 619K ohm */
	ADC_OPEN		= 0x1f
};

enum {
	DOCK_KEY_NONE			= 0,
	DOCK_KEY_VOL_UP_PRESSED,
	DOCK_KEY_VOL_UP_RELEASED,
	DOCK_KEY_VOL_DOWN_PRESSED,
	DOCK_KEY_VOL_DOWN_RELEASED,
	DOCK_KEY_PREV_PRESSED,
	DOCK_KEY_PREV_RELEASED,
	DOCK_KEY_PLAY_PAUSE_PRESSED,
	DOCK_KEY_PLAY_PAUSE_RELEASED,
	DOCK_KEY_NEXT_PRESSED,
	DOCK_KEY_NEXT_RELEASED,
};

struct max77665_muic_info {
	struct device		*dev;
	struct max77665_dev	*max77665;
	struct i2c_client	*muic;
};

static int init_max77665_muic(struct max77665_muic_info *info)
{
	int ret;
	struct i2c_client *client = info->muic;
	u8 val, msk;

	val = (0x1 << COMN1SW_SHIFT) | (0x1 << COMP2SW_SHIFT) |
		(0 << MICEN_SHIFT) | (1 << IDBEN_SHIFT);

	msk = COMN1SW_MASK | COMP2SW_MASK | MICEN_MASK | IDBEN_MASK;

	ret = max77665_update_reg(client, MAX77665_MUIC_REG_CTRL1, val, msk);

	return 0;
}

static int __devinit max77665_muic_probe(struct platform_device *pdev)
{
	struct max77665_dev *max77665 = dev_get_drvdata(pdev->dev.parent);
	struct max77665_muic_info *info;
	int ret = 0;

	pr_info("func:%s\n", __func__);

	info = kzalloc(sizeof(struct max77665_muic_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "%s: failed to allocate info\n", __func__);
		ret = -ENOMEM;
		goto err_return;
	}

	info->dev = &pdev->dev;
	info->max77665 = max77665;
	info->muic = max77665->muic;

	platform_set_drvdata(pdev, info);

	ret = init_max77665_muic(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to initialize MUIC:%d\n", ret);
		goto fail;
	}

	return 0;

 fail:
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
