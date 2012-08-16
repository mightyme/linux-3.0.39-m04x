/*
 * mx_tvout.c - tyout driver helper for mx2 board
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author:  lvcha qiu   <lvcha@meizu.com>
 *
 * This program is not provided / owned by Maxim Integrated Products.
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
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/notifier.h>
#include <linux/mhl.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <linux/err.h>

#include <mach/irqs.h>
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-m040.h>

#include <plat/pd.h>
#include <plat/devs.h>
#include <plat/tvout.h>

#ifdef CONFIG_MHL_DRIVER
static int mx2_mhl_power_on(struct mhl_platform_data *pdata, int enable)
{
	struct regulator *vdd12mhl_regulator;
	struct regulator *vdd12mhl_gpio_regulator;
	int ret;

	pdata->mhl_logic_regulator = regulator_get(NULL, "vdd_ldo26");
	if (!pdata->mhl_logic_regulator) {
		MHLPRINTK("regulator_get failed");
		return -1;
	}
	vdd12mhl_regulator = regulator_get(NULL, "vdd_ldo20");
	if (!vdd12mhl_regulator) {
		MHLPRINTK("regulator_get failed");
		regulator_put(pdata->mhl_logic_regulator);
		return -1;
	}
	vdd12mhl_gpio_regulator = regulator_get(NULL, "MHL_1.2V");
	if (!vdd12mhl_gpio_regulator) {
		regulator_put(pdata->mhl_logic_regulator);
		regulator_put(vdd12mhl_regulator);
		MHLPRINTK("regulator_get failed");
		return -1;
	}

	if (enable) {
		ret = regulator_enable(pdata->mhl_logic_regulator);
		ret = regulator_enable(vdd12mhl_regulator);
		ret = regulator_enable(vdd12mhl_gpio_regulator);
	} else {
		ret = regulator_disable(pdata->mhl_logic_regulator);
		ret = regulator_disable(vdd12mhl_regulator);
		ret = regulator_disable(vdd12mhl_gpio_regulator);
	}
	regulator_put(pdata->mhl_logic_regulator);
	regulator_put(vdd12mhl_regulator);
	regulator_put(vdd12mhl_gpio_regulator);

	if (ret < 0) {
		MHLPRINTK("regulator_%sable failed\n", enable ? "en" : "dis");
		return ret;
	}
	MHLPRINTK("regulator_%sable\n", enable ? "en" : "dis");
	return 0;
}

static int mx2_mhl_reset(struct mhl_platform_data *pdata)
{
	int err;

	/* mhl wake */
	err = gpio_request(pdata->mhl_wake_pin, NULL);
	if (err) {
		MHLPRINTK("gpio_request failed\n");
		return -1;
	}
	s3c_gpio_cfgpin(pdata->mhl_wake_pin, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(pdata->mhl_wake_pin, S3C_GPIO_PULL_NONE);
	gpio_direction_output(pdata->mhl_wake_pin, 1);
	gpio_free(pdata->mhl_wake_pin);
	mdelay(10);

	/* mhl reset */
	err = gpio_request(pdata->mhl_reset_pin, NULL);
	if (err) {
		MHLPRINTK("gpio_request failed\n");
		return -1;
	}
	s3c_gpio_cfgpin(pdata->mhl_reset_pin, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(pdata->mhl_reset_pin, S3C_GPIO_PULL_NONE);
	gpio_direction_output(pdata->mhl_reset_pin, 0);
	mdelay(5);
	gpio_direction_output(pdata->mhl_reset_pin, 1);
	gpio_free(pdata->mhl_reset_pin);

	return 0;
}

static struct mhl_platform_data mx2_mhl_platdata = {
	.mhl_wake_pin = M040_MHL_WAKE,
	.mhl_reset_pin = M040_MHL_RST,
	.mhl_irq_pin  = M040_MHL_IRQ,
	.eint  = IRQ_EINT(23),
	.mhl_usb_irq_pin  = M040_MHL_USB_IRQ,
	.mhl_power_on = mx2_mhl_power_on,
	.reset = mx2_mhl_reset,
};
#endif
static struct i2c_board_info __initdata i2c_devs8[] = {
#ifdef CONFIG_MHL_DRIVER
	{I2C_BOARD_INFO("mhl_page0", (0x76 >> 1)),},
	{I2C_BOARD_INFO("mhl_page1", (0x7E >> 1)),},
	{I2C_BOARD_INFO("mhl_page2", (0x96 >> 1)),},
	{I2C_BOARD_INFO("mhl_cbus", (0xCC >> 1)),
	.platform_data = &mx2_mhl_platdata,},
#endif
};

#if defined(CONFIG_VIDEO_TVOUT)
static int mx2_tvout_enable_power( int on)
{
	int ret = 0;
	struct regulator *hdmi_18_v = NULL;
	struct regulator *hdmi_10_v = NULL;

	printk("%s:[%s]\n", __func__, on?"ON":"OFF");

	hdmi_10_v = regulator_get(NULL, "HDMI_1.0V");
	if (!hdmi_10_v) {
		pr_err("regulator_get failed\n");
		goto ret_on_err0;
	}
	hdmi_18_v = regulator_get(NULL, "vdd_ldo19");
	if (!hdmi_18_v) {
		pr_err("regulator_get failed\n");
		goto ret_on_err1;
	}

	if(on){
		ret = regulator_enable(hdmi_10_v);
		if (ret != 0) {
			pr_err("regulator_enable failed.\n");
			goto ret_on_err2;
		}
		ret = regulator_enable(hdmi_18_v);
		if (ret != 0) {
			pr_err("regulator_enable failed.\n");
			goto ret_on_err2;
		}
	}else{
		ret = regulator_disable(hdmi_10_v);
		if (ret != 0) {
			pr_err("regulator_disable failed.\n");
			goto ret_on_err2;

		}
		ret = regulator_disable(hdmi_18_v);
		if (ret != 0) {
			pr_err("regulator_disable failed.\n");
			goto ret_on_err2;
		}
	}
ret_on_err2:
	regulator_put(hdmi_10_v);
ret_on_err1:
	regulator_put(hdmi_18_v);
ret_on_err0:
	return ret;
}
static struct s5p_platform_tvout __initdata mx2_tvout_data={
	.enable_power = mx2_tvout_enable_power,
};
static struct s5p_platform_hpd hdmi_hpd_data __initdata = {

};
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif
static struct i2c_board_info __initdata i2c_devs13[] = {
#ifdef CONFIG_VIDEO_TVOUT
	{
		I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),
	},
#endif
};
static int  __init mx2_init_tvout(void)
{
	/* mhl */
#ifdef CONFIG_MHL_DRIVER
	i2c_register_board_info(8, i2c_devs8, ARRAY_SIZE(i2c_devs8));
#endif

#if defined(CONFIG_VIDEO_TVOUT)
	/* hdmi ddc */
	i2c_register_board_info(13, i2c_devs13, ARRAY_SIZE(i2c_devs13));

	s5p_device_tvout.dev.parent = &exynos4_device_pd[PD_TV].dev;
	s5p_tvout_set_platdata(&mx2_tvout_data);
	s5p_hdmi_hpd_set_platdata(&hdmi_hpd_data);
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
#endif
	return 0;
}

arch_initcall(mx2_init_tvout);

MODULE_DESCRIPTION("mx2 tvout driver helper");
MODULE_AUTHOR("lvcha qiu <lvcha@meizu.com>");
MODULE_LICENSE("GPLV2");

