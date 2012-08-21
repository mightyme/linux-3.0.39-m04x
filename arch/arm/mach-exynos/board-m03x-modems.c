/* arch/arm/mach-exynos/board-m03x-modems.c
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/platform_data/modem.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-m03x.h>
#include <mach/modem.h>
#include <asm/mach-types.h>
#include <mach/gpio-m030.h>

static struct modem_io_t m03x_io_devices[] = {
	[0] = {
		.name = "ttyACM0",
		.id = 0x0,
		.format  = IPC_RAW,
		.io_type = IODEV_TTY,
		.links   = LINKTYPE(LINKDEV_HSIC),
	},
	[1] = {
		.name = "ttyACM1",
		.id = 0x1,
		.format = IPC_RAW,
		.io_type = IODEV_TTY,
		.links = LINKTYPE(LINKDEV_HSIC),
	},
	[2] = {
		.name = "ttyACM2",
		.id = 0x2,
		.format = IPC_RAW,
		.io_type = IODEV_TTY,
		.links = LINKTYPE(LINKDEV_HSIC),
	},
	[3] = {
		.name = "ttyACM3",
		.id = 0x3,
		.format = IPC_RAW,
		.io_type = IODEV_TTY,
		.links = LINKTYPE(LINKDEV_HSIC),
	},
};

static struct resource umts_modem_res[] = {
	[0]     =  {
		.name   =  "link_pm_hostwake",
		.start  =  IRQ_EINT16_31,
		.end    =  IRQ_EINT16_31,
		.flags  =  IORESOURCE_IRQ,
	},
};

static void xmm_gpio_revers_bias_restore(void);

static struct modemlink_pm_data modem_link_pm_data = {
	.name = "link_pm",
	.gpio_link_enable    = 0,                    
	.gpio_link_hostwake  = GPIO_HOST_WAKEUP, 
	.gpio_link_slavewake = GPIO_SLAVE_WAKEUP,    
};

static struct modem_data umts_modem_data;

static struct modem_data umts_modem_data_m030 = {
	.name                     = "xmm6260",
	.gpio_cp_on               = GPIO_MODEM_ON,
	.gpio_reset_req_n         = GPIO_MODEM_RST_FULL,
	.gpio_cp_reset            = GPIO_MODEM_RST,
	.gpio_host_active         = GPIO_HOST_ACTIVE,
	.gpio_cp_reset_int        = GPIO_MODEM_RESET_INT,
	.gpio_cp_dump_int         = GPIO_MODEM_DUMP_INT,
	.gpio_cp_warm_reset       = 0,
	.gpio_sim_detect          = M030_INT16_SIMDETECT,
	.modem_type               = IMC_XMM6260,
	.link_types               = LINKTYPE(LINKDEV_HSIC),
	.modem_net                = UMTS_NETWORK,
	.use_handover             = false,
	.num_iodevs               = ARRAY_SIZE(m03x_io_devices),
	.iodevs                   = m03x_io_devices,
	.link_pm_data             = &modem_link_pm_data,
	.gpio_revers_bias_clear   = NULL,
	.gpio_revers_bias_restore = xmm_gpio_revers_bias_restore,
};

static struct modem_data umts_modem_data_m03x = {
	.name                     = "xmm6260",
	.gpio_cp_on               = GPIO_MODEM_ON,
	.gpio_reset_req_n         = GPIO_MODEM_RST_FULL,
	.gpio_cp_reset            = GPIO_MODEM_RST,
	.gpio_host_active         = GPIO_HOST_ACTIVE,
	.gpio_cp_reset_int        = GPIO_MODEM_RESET_INT,
	.gpio_cp_dump_int         = GPIO_MODEM_DUMP_INT,
	.gpio_cp_warm_reset       = 0,
	.gpio_sim_detect          = 0,
	.modem_type               = IMC_XMM6260,
	.link_types               = LINKTYPE(LINKDEV_HSIC),
	.modem_net                = UMTS_NETWORK,
	.use_handover             = false,
	.num_iodevs               = ARRAY_SIZE(m03x_io_devices),
	.iodevs                   = m03x_io_devices,
	.link_pm_data             = &modem_link_pm_data,
	.gpio_revers_bias_clear   = NULL,
	.gpio_revers_bias_restore = xmm_gpio_revers_bias_restore,
};

static void xmm_gpio_revers_bias_restore(void)
{
	unsigned gpio_link_hostwake = modem_link_pm_data.gpio_link_hostwake;
	unsigned gpio_sim_detect = umts_modem_data.gpio_sim_detect;

	gpio_direction_output(gpio_link_hostwake, 0);
	s3c_gpio_cfgpin(gpio_link_hostwake, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(gpio_link_hostwake, S3C_GPIO_PULL_NONE);
	irq_set_irq_type(gpio_to_irq(gpio_link_hostwake),
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	enable_irq_wake(gpio_to_irq(gpio_link_hostwake));

	if (umts_modem_data.gpio_sim_detect) {
		gpio_direction_output(gpio_sim_detect, 0);
		s3c_gpio_cfgpin(gpio_sim_detect, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_sim_detect, S3C_GPIO_PULL_NONE);
		irq_set_irq_type(gpio_to_irq(gpio_sim_detect),
				IRQ_TYPE_EDGE_BOTH);
		enable_irq_wake(gpio_to_irq(gpio_sim_detect));
	}
}

/* HSIC specific function */
void set_slave_wake(void)
{
	if (gpio_get_value(modem_link_pm_data.gpio_link_hostwake)) {
		pr_info("[MODEM_IF]Slave Wake\n");
		if (gpio_get_value(modem_link_pm_data.gpio_link_slavewake)) {
			gpio_direction_output(
			modem_link_pm_data.gpio_link_slavewake, 0);
			mdelay(10);
		}
		gpio_direction_output(modem_link_pm_data.gpio_link_slavewake, 1);
	}
}

void modem_set_active_state(int state)
{
	int err = 0;
	
	mif_debug("%s state=>:%d\n", __func__, state);
	err = gpio_request(umts_modem_data.gpio_host_active, "PDA_ACTIVE");
	if (err)
		pr_err("%s fail to request gpio %s:%d\n", __func__,
				"PDA_ACTIVE", err);
	else {
		gpio_direction_output(umts_modem_data.gpio_host_active, state);
		gpio_free(umts_modem_data.gpio_host_active);
	}
}

static struct platform_device umts_modem_m030 = {
	.name = "modem_ifx_6260",
	.id = -1,
	.num_resources = ARRAY_SIZE(umts_modem_res),
	.resource = umts_modem_res,
	.dev = {
		.platform_data = &umts_modem_data_m030,
	},
};

static struct platform_device umts_modem_m03x = {
	.name = "modem_ifx_6260",
	.id = -1,
	.num_resources = ARRAY_SIZE(umts_modem_res),
	.resource = umts_modem_res,
	.dev = {
		.platform_data = &umts_modem_data_m03x,
	},
};

static void umts_modem_cfg_gpio(void)
{
	int err = 0;

	unsigned gpio_reset_req_n = umts_modem_data.gpio_reset_req_n;
	unsigned gpio_cp_on = umts_modem_data.gpio_cp_on;
	unsigned gpio_cp_rst = umts_modem_data.gpio_cp_reset;
	unsigned gpio_host_active = umts_modem_data.gpio_host_active;
	unsigned gpio_cp_reset_int = umts_modem_data.gpio_cp_reset_int;
	unsigned gpio_sim_detect = umts_modem_data.gpio_sim_detect;

	if (gpio_reset_req_n) {
		err = gpio_request(gpio_reset_req_n, "RESET_REQ_N");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "RESET_REQ_N", err);
		}
		gpio_direction_output(gpio_reset_req_n, 0);
	}

	if (gpio_cp_on) {
		err = gpio_request(gpio_cp_on, "CP_ON");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "CP_ON", err);
		}
		gpio_direction_output(gpio_cp_on, 0);
	}

	if (gpio_cp_rst) {
		err = gpio_request(gpio_cp_rst, "CP_RST");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "CP_RST", err);
		}
		gpio_direction_output(gpio_cp_rst, 0);
		s3c_gpio_setpull(gpio_cp_rst, S3C_GPIO_PULL_UP);
	}

	if (gpio_cp_reset_int) {
		err = gpio_request(gpio_cp_reset_int, "GPIO_CP_RESET_INT");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "GPIO_CP_RESET_INT", err);
		}
		s3c_gpio_setpull(gpio_cp_reset_int, S3C_GPIO_PULL_NONE);
                gpio_direction_input(gpio_cp_reset_int);
	}

	if (gpio_host_active) {
		err = gpio_request(gpio_host_active, "HOST_ACTIVE");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "HOST_ACTIVE", err);
		}
		gpio_direction_output(gpio_host_active, 0);
		gpio_free(gpio_host_active);
	}

	if (gpio_sim_detect) {
		err = gpio_request(gpio_sim_detect, "SIM_DETECT");
		if (err)
			printk(KERN_ERR "fail to request gpio %s: %d\n",
				"SIM_DETECT", err);
		s3c_gpio_cfgpin(gpio_sim_detect, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_sim_detect, S3C_GPIO_PULL_NONE);
		irq_set_irq_type(gpio_to_irq(gpio_sim_detect),
							IRQ_TYPE_EDGE_BOTH);
	}

	printk(KERN_INFO "umts_modem_cfg_gpio done\n");
}

static void modem_link_pm_config_gpio(void)
{
	int err = 0;

	unsigned gpio_link_enable = modem_link_pm_data.gpio_link_enable;
	unsigned gpio_link_hostwake = modem_link_pm_data.gpio_link_hostwake;
	unsigned gpio_link_slavewake = modem_link_pm_data.gpio_link_slavewake;

	if (gpio_link_enable) {
		err = gpio_request(gpio_link_enable, "LINK_EN");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "LINK_EN", err);
		}
		gpio_direction_output(gpio_link_enable, 0);
		gpio_free(gpio_link_enable);
	}

	if (gpio_link_hostwake) {
		err = gpio_request(gpio_link_hostwake, "HOSTWAKE");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "HOSTWAKE", err);
		}
		gpio_direction_input(gpio_link_hostwake);
		s3c_gpio_cfgpin(gpio_link_hostwake, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_link_hostwake, S3C_GPIO_PULL_NONE);
	}

	if (gpio_link_slavewake) {
		err = gpio_request(gpio_link_slavewake, "SLAVEWAKE");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "SLAVEWAKE", err);
		}
		gpio_direction_output(gpio_link_slavewake, 0);
		s3c_gpio_setpull(gpio_link_slavewake, S3C_GPIO_PULL_NONE);
	}

	if (gpio_link_hostwake)
		irq_set_irq_type(gpio_to_irq(gpio_link_hostwake),
				IRQ_TYPE_EDGE_BOTH);

	printk(KERN_INFO "modem_link_pm_config_gpio done\n");
}

static int __init init_modem(void)
{
	int ret;

	if(machine_is_m030())
		umts_modem_data = umts_modem_data_m030;
	else
		umts_modem_data = umts_modem_data_m03x;

	umts_modem_cfg_gpio();
	modem_link_pm_config_gpio();

	if(machine_is_m030())
		ret = platform_device_register(&umts_modem_m030);
	else 
		ret = platform_device_register(&umts_modem_m03x);
	if (ret < 0)
		return ret;

	pr_info("[MODEM_IF] init_modem device.\n");

	return ret;
}
late_initcall(init_modem);
