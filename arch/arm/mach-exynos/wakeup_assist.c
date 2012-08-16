/*
 * Wakeup assist driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/err.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/regs-pmu.h>
#include <plat/pm.h>

#define DEV_NAME "wakeup_assist"

static int wakeup_assist_keycode[] = { KEY_HOME, KEY_HOME,KEY_VOLUMEDOWN,KEY_VOLUMEUP};

static int __devinit wakeup_assist_probe(struct platform_device *pdev)
{
	int error;
	struct input_dev *input_dev;

	input_dev = input_allocate_device();

	if (!input_dev)
		return -ENOMEM;

	input_dev->name = DEV_NAME;
	input_dev->id.bustype = BUS_HOST;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY);

	input_set_capability(input_dev, EV_MSC, MSC_SCAN);

	input_dev->keycode = wakeup_assist_keycode;
	input_dev->keycodesize = sizeof(wakeup_assist_keycode[0]);
	input_dev->keycodemax = ARRAY_SIZE(wakeup_assist_keycode);

	__set_bit(wakeup_assist_keycode[0], input_dev->keybit);
	__set_bit(wakeup_assist_keycode[1], input_dev->keybit);
	__set_bit(wakeup_assist_keycode[2], input_dev->keybit);
	__set_bit(wakeup_assist_keycode[3], input_dev->keybit);
	__clear_bit(KEY_RESERVED, input_dev->keybit);

	error = input_register_device(input_dev);
	if (error) {
		input_free_device(input_dev);
		return error;
	}

	platform_set_drvdata(pdev, input_dev);

	return 0;
}

static int __devexit wakeup_assist_remove(struct platform_device *pdev)
{
	struct input_dev *input_dev = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	input_unregister_device(input_dev);
	input_free_device(input_dev);

	return 0;
}

#if defined(CONFIG_MX_SERIAL_TYPE) || defined(CONFIG_MX2_SERIAL_TYPE)
static unsigned long mx_wakeup_type;
unsigned long mx_get_wakeup_type(void)
{
	return mx_wakeup_type;
}

unsigned long mx_set_wakeup_type(unsigned long mask)
{
	return mask ? (mx_wakeup_type |= mask) :
				  (mx_wakeup_type = mask);
}

void mx_show_wakeup_name(void)
{
	int len;
	char wakeup_str[256];

	memset(wakeup_str, 0, 256);

	len = sprintf(wakeup_str, "%s", "MX Wakeup By:");

	if(mx_get_wakeup_type() & MX_USB_WAKE)
		len +=  sprintf(wakeup_str+len, " USB_WAKE");
	if(mx_get_wakeup_type() & MX_LOWBAT_WAKE)
		len +=  sprintf(wakeup_str+len, " LOWBAT_WAKE");
	if(mx_get_wakeup_type() & MX_CHARG_WAKE)
		len +=  sprintf(wakeup_str+len, " CHARG_WAKE");
	if(mx_get_wakeup_type() & MX_WIFI_WAKE)
		len +=  sprintf(wakeup_str+len, " WIFI_WAKE");
	if(mx_get_wakeup_type() & MX_BLUETOOTH_WAKE)
		len +=  sprintf(wakeup_str+len, " BLUETOOTH_WAKE");
	if(mx_get_wakeup_type() & MX_MODEM_RST_WAKE)
		len +=  sprintf(wakeup_str+len, " MODEM_RST_WAKE");
	if(mx_get_wakeup_type() & MX_MODEM_WAKE)
		len +=  sprintf(wakeup_str+len, " MODEM_WAKE");
	if(mx_get_wakeup_type() & MX_KEY_POWER_WAKE)
		len +=  sprintf(wakeup_str+len, " KEY_POWER_WAKE");
	if(mx_get_wakeup_type() & MX_KEY_HOME_WAKE)
		len +=  sprintf(wakeup_str+len, " KEY_HOME_WAKE");
	if(mx_get_wakeup_type() & MX_USB_HOST_WAKE)
		len +=  sprintf(wakeup_str+len, " MX_USB_HOST_WAKE");
	if(mx_get_wakeup_type() & MX_ALARM_WAKE)
		len +=  sprintf(wakeup_str+len, " ALARM_WAKE");
	if(mx_get_wakeup_type() & MX_TICK_WAKE)
		len +=  sprintf(wakeup_str+len, " TICK_WAKE");
	if(mx_get_wakeup_type() & MX_I2S_WAKE)
		len +=  sprintf(wakeup_str+len, " I2S_WAKE");
	if(mx_get_wakeup_type() & MX_SYSTIMER_WAKE)
		len +=  sprintf(wakeup_str+len, " SYSTIMER_WAKE");
	if(mx_get_wakeup_type() & MX_MINUS_KEY_WAKE)
		len +=  sprintf(wakeup_str+len, " VOLUMEDOWN_KEY_WAKE");
	if(mx_get_wakeup_type() & MX_PLUS_KEY_WAKE)
		len +=  sprintf(wakeup_str+len, " VOLUMEUP_KEY_WAKE");
	if(!mx_get_wakeup_type())
		len +=  sprintf(wakeup_str+len, " UNKNOW_WAKE");
	pr_info("%s\n", wakeup_str);
}

static inline int wakeup_assist_done(struct input_dev *input_dev)
{
	unsigned long mx_wake_typed = mx_get_wakeup_type();
	unsigned long allow_wake_type = MX_USB_WAKE |
								   MX_LOWBAT_WAKE |
								   MX_IR_WAKE |
								   MX_KEY_HOME_WAKE |
								   MX_KEY_POWER_WAKE ;

	if (mx_wake_typed & allow_wake_type) {
		input_report_key(input_dev, wakeup_assist_keycode[0], 1);
		input_sync(input_dev);
		input_report_key(input_dev, wakeup_assist_keycode[0], 0);
		input_sync(input_dev);
	} else if (mx_wake_typed & MX_MINUS_KEY_WAKE) {
		input_report_key(input_dev, wakeup_assist_keycode[2], 1);
		input_sync(input_dev);
		input_report_key(input_dev, wakeup_assist_keycode[2], 0);
		input_sync(input_dev);
	} else if (mx_wake_typed & MX_PLUS_KEY_WAKE) {
		input_report_key(input_dev, wakeup_assist_keycode[3], 1);
		input_sync(input_dev);
		input_report_key(input_dev, wakeup_assist_keycode[3], 0);
		input_sync(input_dev);
	}

	mx_show_wakeup_name();
	mx_set_wakeup_type(0);
	return 0;
}
#else
static inline int wakeup_assist_done(struct input_dev *input_dev)
{
	if (readl(S5P_WAKEUP_STAT) & 0x1) {
		input_report_key(input_dev, wakeup_assist_keycode[0], 1);
		input_sync(input_dev);
		input_report_key(input_dev, wakeup_assist_keycode[0], 0);
		input_sync(input_dev);
	}
	return 0;
}
#endif

static int wakeup_assist_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct input_dev *input_dev = platform_get_drvdata(pdev);

	return wakeup_assist_done(input_dev);
}

static const struct dev_pm_ops wakeup_assist_pm_ops = {
	.resume		= wakeup_assist_resume,
};

static struct platform_driver wakeup_assist_driver = {
	.probe		= wakeup_assist_probe,
	.remove		= __devexit_p(wakeup_assist_remove),
	.driver		= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &wakeup_assist_pm_ops,
	},
};

static int __init wakeup_assist_init(void)
{
	return platform_driver_register(&wakeup_assist_driver);
}
module_init(wakeup_assist_init);

static void __exit wakeup_assist_exit(void)
{
	platform_driver_unregister(&wakeup_assist_driver);
}
module_exit(wakeup_assist_exit);

MODULE_DESCRIPTION("Wakeup assist driver");
MODULE_AUTHOR("Eunki Kim <eunki_kim@samsung.com>");
MODULE_LICENSE("GPL");
