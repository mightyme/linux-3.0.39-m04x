/* linux/arch/arm/mach-exynos/mach-m040.c
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *		http://www.meizu.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/input.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max77686.h>
#include <linux/mfd/max77665.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/bu26507.h>
#include <linux/power_supply.h>
#include <linux/memblock.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/scatterlist.h>
#include <linux/gpio_keys.h>
#include <linux/pwm_backlight.h>
#include <linux/bq27541.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#ifdef CONFIG_AUDIENCE_ES305B
#include <linux/es305b_soc.h>
#endif
#include <linux/mhl.h>
#include <linux/devfreq-exynos4.h>
#include <linux/platform_data/exynos4_tmu.h>
#include <linux/platform_data/exynos4_ppmu.h>
#include <linux/mx_lis3dh.h>
#ifdef CONFIG_SENSORS_L3G4200D
#include <linux/mx_l3g4200d.h>
#endif
#ifdef CONFIG_SENSORS_L3GD20
#include <linux/m040_l3gd20.h>
#endif
#include <linux/mx_spdif_platform.h>
#include <linux/exynos-cpufreq.h>
#include <linux/led-lm3530.h>
#ifdef CONFIG_SWITCH_GPIO
#include <linux/switch.h>
#endif

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/map.h>
#include <mach/gpio-m040.h>
#include <mach/dwmci.h>
#include <mach/dev-sysmmu.h>
#include <mach/dev-ppmu.h>
#include <mach/dev.h>
#include <mach/system.h>
#include <mach/regs-clock.h>
#include <mach/usb-detect.h>
#include <mach/i2c-m040.h>
#ifdef CONFIG_BT
#include <mach/mx-rfkill.h>
#endif
#ifdef CONFIG_XMM6260_MODEM
#include <mach/modem.h>
#endif
#include <plat/regs-serial.h>
#include <plat/exynos4.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/pd.h>
#include <plat/iic.h>
#include <plat/gpio-cfg.h>
#include <plat/s5p-mfc.h>
#include <plat/usbgadget.h>
#include <plat/hwmon.h>
#include <plat/ehci.h>
#include <plat/tvout.h>
#include <plat/backlight.h>
#include <plat/sysmmu.h>
#include <plat/fimg2d.h>
#include <plat/jpeg.h>
#include <plat/sdhci.h>
#include <plat/mshci.h>
#ifdef CONFIG_BATTERY_MEIZU
#include <mach/meizu_battery.h>
#endif
#include <mach/m040_regulator.h>
#include <mach/m040_regulator_fixed.h>
#ifdef CONFIG_MFD_MXQM
#include <linux/mx_qm.h>
#endif


extern void __init m040_reserve_mem(void);

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define M040_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define M040_ULCON_DEFAULT	S3C2410_LCON_CS8

#define M040_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg __initdata m040_uartcfgs[] = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= M040_UCON_DEFAULT,
		.ulcon		= M040_ULCON_DEFAULT,
		.ufcon		= M040_UFCON_DEFAULT,
#if defined(CONFIG_BT)
		.wake_peer	= bt_uart_wake_peer,
#endif
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= M040_UCON_DEFAULT,
		.ulcon		= M040_ULCON_DEFAULT,
		.ufcon		= M040_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags			= 0,
		.ucon		= M040_UCON_DEFAULT,
		.ulcon		= M040_ULCON_DEFAULT,
		.ufcon		= M040_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= M040_UCON_DEFAULT,
		.ulcon		= M040_ULCON_DEFAULT,
		.ufcon		= M040_UFCON_DEFAULT,
	},
};

#ifdef CONFIG_BT
static struct platform_device m040_bt = {
	.name = "bcm4330_bluetooth",
	.id = -1,
};
#endif

#ifdef CONFIG_S3C_DEV_HWMON
static struct s3c_hwmon_pdata __initdata m040_hwmon_pdata = {
	/* Reference voltage (1.8V) */
	.in[0] = &(struct s3c_hwmon_chcfg) {
		.name	= "m03x: vbat",
		.mult		= 1800,
		.div		= 2048,
	},
	.in[1] = &(struct s3c_hwmon_chcfg) {
		.name	= "m03x: thermal",
		.mult		= 1800,
		.div		= 2048,
	},
	.in[2] = &(struct s3c_hwmon_chcfg) {
		.name	= "m03x: vdd_arm",
		.mult		= 1800,
		.div		= 4096,
	},
	.in[3] = &(struct s3c_hwmon_chcfg) {
		.name	= "m03x: mic",
		.mult		= 1800,
		.div		= 4096,
	},
	.in[4] = NULL,
};
#endif

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
static void m040_dwmci_cfg_gpio(int width)
{
	static int pre_width = -1;

	unsigned int gpio;

	if (pre_width == width)
		return;

	for (gpio = EXYNOS4_GPK0(0); gpio < EXYNOS4_GPK0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	}

	switch (width) {
	case 8:
		for (gpio = EXYNOS4_GPK1(3); gpio <= EXYNOS4_GPK1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(4));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
	case 4:
		for (gpio = EXYNOS4_GPK0(3); gpio <= EXYNOS4_GPK0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
		break;
	case 1:
		gpio = EXYNOS4_GPK0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	default:
		break;
	}

	pre_width = width;
}

static int m040_dwmci_get_bus_wd(u32 slot_id)
{
	if (0 == slot_id)
		return 8;
	return 4;
}

static struct dw_mci_board __initdata m040_dwmci_pdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION |
					    DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps				= MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR |
					   MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio			= m040_dwmci_cfg_gpio,
	.get_bus_wd		= m040_dwmci_get_bus_wd,
};
#endif

#ifdef CONFIG_SND_SOC_M040_WM8958
static struct platform_device m040_audio_device = {
	.name = "m040-audio",
	.id = -1,
};
#endif

#ifdef CONFIG_SND_SAMSUNG_SPDIF
static void m040_spdif_output_enable(int bEn)
{
	unsigned long flags =  bEn ? GPIOF_OUT_INIT_LOW :
					   GPIOF_OUT_INIT_HIGH;
	gpio_request_one(M040_SPDIF_OUT, flags, "spdif output");
	gpio_free(M040_SPDIF_OUT);
}

static struct mx_spdif_platform_data m040_spdif_data={
	.spdif_output_enable = m040_spdif_output_enable,
};

static struct platform_device m040_spdif_device={
	.name = "m040-spdif",
	.id = -1,
	.dev = {
		.platform_data = &m040_spdif_data,
	},
};
#endif

#if defined(CONFIG_MFD_WM8994)
static struct regulator_consumer_supply wm8958_avdd1_supply =
	REGULATOR_SUPPLY("AVDD1", "0-001a");

static struct regulator_consumer_supply wm8958_dcvdd_supply =
	REGULATOR_SUPPLY("DCVDD", "0-001a");

static struct regulator_init_data wm8958_ldo1_data = {
	.constraints	= {
		.name		= "AVDD1",
		.min_uV		= 2400000,
		.max_uV		= 3100000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8958_avdd1_supply,
};

static struct regulator_init_data wm8958_ldo2_data = {
	.constraints	= {
		.name		= "DCVDD",
		.min_uV		= 1000000,
		.max_uV		= 1300000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8958_dcvdd_supply,
};

#if defined(CONFIG_SWITCH_GPIO)
static struct gpio_switch_platform_data m040_earphone_pd = {
	.name = "h2w",
	.gpio = M040_HDETEC_IRQ,
	.active_level = 0,
	.adc_channel = 3,
};
static struct platform_device m040_switch_gpio = {
	.name = "switch-gpio",
	.dev = {
		.platform_data = &m040_earphone_pd,
	},
	.id = -1,
};
#endif

static struct wm8994_pdata wm8958_platform_data = {
	/* configure gpio1 function: 0x0001(Logic level input/output) */
	.gpio_defaults[0] = 0x0001,
	/* If the i2s0 and i2s2 is enabled simultaneously */
	.gpio_defaults[7] = 0x8100, /* GPIO8  DACDAT3 in */
	.gpio_defaults[8] = 0x0100, /* GPIO9  ADCDAT3 out */
	.gpio_defaults[9] = 0x0100, /* GPIO10 LRCLK3  out */
	.gpio_defaults[10] = 0x0100,/* GPIO11 BCLK3   out */
	.ldo[0] = {M040_CODEC_LDO1_EN, NULL, &wm8958_ldo1_data},
	.ldo[1] = {M040_CODEC_LDO2_EN, NULL, &wm8958_ldo2_data},
};
#endif


static struct i2c_board_info __initdata i2c_devs0[] = {
#if defined(CONFIG_MFD_WM8994)
	{
		I2C_BOARD_INFO("wm8958", (0x34 >> 1)),
		.platform_data	= &wm8958_platform_data,
	},
#endif

};
static struct i2c_board_info __initdata i2c_devs1[] = {
#if defined(CONFIG_REGULATOR_MAX77686)
	{
		I2C_BOARD_INFO("max77686", (0x12 >> 1)),
		.platform_data	= &m040_max77686_info,
	},
#endif
};

#if defined(CONFIG_MFD_MAX77665)
#ifdef CONFIG_USB_GADGET
static int m040_usb_attach(bool enable)
{
	struct usb_gadget *gadget =
		platform_get_drvdata(&s3c_device_usbgadget);
	int ret = -1;

	if (gadget) {
		pr_info("m040_usb_attach %s\n", enable?"enable":"disable");
		if (enable)
			ret = usb_gadget_vbus_connect(gadget);
		else
			ret = usb_gadget_vbus_disconnect(gadget);
	}
	return ret;
}
#endif

#if defined(CONFIG_AUDIENCE_ES305B)
/* es305b */
static struct es305b_platform_data __initdata es305b_pd = {
	.gpio_es305b_wake	= M040_NOISE_CANCELLER_WAKE,
	.gpio_es305b_reset	= M040_NOISE_CANCELLER_RST,
};
#endif
static struct i2c_board_info __initdata i2c_devs4[] = {
#if defined(CONFIG_AUDIENCE_ES305B)
	{
		I2C_BOARD_INFO(ES305B_I2C_NAME, ES305B_I2S_SLAVE_ADDRESS),
		.platform_data	= &es305b_pd,
	},
#endif
};

#if defined(CONFIG_LEDS_LM3530)
static struct lm3530_platform_data lm3530_pd = {
	.mode = LM3530_BL_MODE_MANUAL,
	.als_input_mode = LM3530_INPUT_AVRG,
	.max_current = 0x4,
	.pwm_pol_hi = true,
	.brt_val = 0xff,
	.brt_ramp_law = true,
};
#endif
static struct i2c_board_info __initdata i2c_devs6[] = {
	{
#if defined(CONFIG_LEDS_LM3530)
		I2C_BOARD_INFO("lm3530-led", (0x39)),
		.platform_data = &lm3530_pd,
		.irq = M040_TOUCH_IRQ,
#endif
	},
};

/* I2C8 */
static struct i2c_gpio_platform_data gpio_i2c8_data  = {
	.sda_pin		= M040_SDA_MHL,
	.scl_pin		= M040_SCL_MHL,
	.udelay			= 5,
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

struct platform_device m040_device_gpio_i2c8 = {
	.name	= "i2c-gpio",
	.id	= 8,
	.dev	= {
		.platform_data = &gpio_i2c8_data
	},
};

/*st lis3dh accelerometer sensor*/
static struct i2c_gpio_platform_data gpio_i2c9_data = {
	.sda_pin = M040_SDA_GS,
	.scl_pin = M040_SCL_GS,
	.udelay = 2,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device m040_device_gpio_i2c9 = {
	.name = "i2c-gpio",
	.id = 9,
	.dev.platform_data = &gpio_i2c9_data,
};

#ifdef CONFIG_SENSORS_LIS3DH
static struct lis3dh_acc_platform_data lis3dh_platdata = {
	.poll_interval = 200,   /*default poll delay 200 ms*/
	.min_interval = 10,

	.g_range = LIS3DH_ACC_G_2G,

	.axis_map_x = 1,  /*x=-x, y=-y, z=z*/
	.axis_map_y = 0,
	.axis_map_z = 2,

	.negate_x = 1,
	.negate_y = 1,
	.negate_z = 1,
};
#endif

static struct i2c_board_info __initdata i2c_devs9[]  = {
#ifdef CONFIG_SENSORS_LIS3DH
	{
		I2C_BOARD_INFO("lis3dh", 0x19),
		.platform_data = &lis3dh_platdata,
	},
#endif
};

/*akm8975C compass sensor*/
static struct i2c_gpio_platform_data gpio_i2c10_data = {
	.sda_pin = M040_SDA_CP,
	.scl_pin = M040_SCL_CP,
	.udelay = 5,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device m040_device_gpio_i2c10 = {
	.name = "i2c-gpio",
	.id = 10,
	.dev.platform_data = &gpio_i2c10_data,
};

static struct i2c_board_info __initdata i2c_devs10[]  = {
#ifdef CONFIG_SENSORS_AKM8963N
	{I2C_BOARD_INFO("akm8963", 0x0c)},
#endif
};

/*st l3g4200d gyroscope sensor*/
static struct i2c_gpio_platform_data gpio_i2c11_data = {
	.sda_pin = M040_SDA_GY,
	.scl_pin = M040_SCL_GY,
	.udelay = 2,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device m040_device_gpio_i2c11 = {
	.name = "i2c-gpio",
	.id = 11,
	.dev.platform_data = &gpio_i2c11_data,
};

#ifdef CONFIG_SENSORS_L3GD20
static struct l3g4200d_platform_data l3gd20_platdata = {
	.fs_range = L3G4200D_FS_2000DPS,
	.axis_map_x = 1, 
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
};
#endif

#ifdef CONFIG_SENSORS_L3G4200D
static struct l3g4200d_platform_data l3g4200d_platdata = {
	.fs_range = L3G4200D_FS_2000DPS,

	.axis_map_x = 0,  /*x=-x, y=-y, z=-z*/
	.axis_map_y = 1,
	.axis_map_z = 2,

	.negate_x = 1,
	.negate_y = 1,
	.negate_z = 1,
};
#endif

static struct i2c_board_info __initdata i2c_devs11[]  = {
#ifdef CONFIG_SENSORS_L3G4200D
	{
		I2C_BOARD_INFO("l3g4200d", 0x69),
		.platform_data = &l3g4200d_platdata,
	},
#endif
#ifdef CONFIG_SENSORS_L3GD20
	{
		I2C_BOARD_INFO("l3gd20", 0x6b),
		.platform_data = &l3gd20_platdata,
	},
#endif
};

#if defined(CONFIG_BATTERY_MEIZU)
static struct meizu_bat_platform_data m040_bat_pdata = {
	.fuel_gauge_name	= "fuelgauge",
	.charger_name		= "charger",
};

static struct platform_device m040_battery_device = {
	.name = "meizu-battery",
	.id = -1,
	.dev.platform_data = &m040_bat_pdata,
};
#endif

/*sharp gp2ap020a00f sensor*/
static struct i2c_gpio_platform_data gpio_i2c12_data = {
	.sda_pin = M040_SDA_IR,
	.scl_pin = M040_SCL_IR,
	.udelay = 2,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device m040_device_gpio_i2c12 = {
	.name = "i2c-gpio",
	.id = 12,
	.dev.platform_data = &gpio_i2c12_data,
};

static struct i2c_board_info __initdata i2c_devs12[] = {
#ifdef CONFIG_SENSORS_GP2AP020A00F
	{I2C_BOARD_INFO("gp2ap020a00f", 0x39),
	.irq = M040_IR_IRQ,},
#endif
};
/*gpio i2c 13*/
static struct i2c_gpio_platform_data gpio_i2c13_data = {
	.sda_pin = M040_SDA_HPD,
	.scl_pin = M040_SCL_HPD,
	.udelay = 2,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device m040_device_gpio_i2c13 = {
	.name = "i2c-gpio",
	.id = 13,
	.dev.platform_data = &gpio_i2c13_data,
};

/*gpio i2c 14*/
static struct i2c_gpio_platform_data gpio_i2c14_data = {
	.sda_pin = M040_SDA_CHG,
	.scl_pin = M040_SCL_CHG,
	.udelay = 2,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device m040_device_gpio_i2c14 = {
	.name = "i2c-gpio",
	.id = 14,
	.dev.platform_data = &gpio_i2c14_data,
};

#ifdef CONFIG_MOTOR_DRV_MAX77665
static struct max77665_haptic_platform_data max77665_haptic_pdata = {
	.pwm_channel_id = 1,
	.pwm_period = 38022,
	.pwm_duty = 37641,
	.type = MAX77665_HAPTIC_LRA,
	.mode = MAX77665_EXTERNAL_MODE,
	.pwm_divisor = MAX77665_PWM_DIVISOR_128
};
#endif

static struct max77665_platform_data __initdata m040_max77665_info = {
	.irq_base = IRQ_BOARD_START + 20,
	.wakeup = true,
	.name = "max77665-charger",

	/* charger */
	.supply = "vinchg1",
	.fast_charge_timer = MAX77665_FCHGTIME_4H,
	.charging_restart_thresold = MAX77665_CHG_RSTRT_150MV,
	.top_off_current_thresold = MAX77665_CHG_TO_ITH_100MA,
	.top_off_timer = MAX77665_CHG_TO_TIME_70MIN,
	.charger_termination_voltage = MAX77665_CHG_CV_PRM_4300MV,
	.fast_charge_current = 866,		/* 0mA ~ 2100mA */
	.chgin_ilim_usb = 460,			/* 60mA ~ 2580mA */
	.chgin_ilim_ac = 900,
#ifdef CONFIG_USB_GADGET
	.charger_pin = M040_CHARGER_IRQ,
	.usb_attach = m040_usb_attach,
#endif

	/* regulator */
	.regulators = max77665_regulators,
	.num_regulators = ARRAY_SIZE(max77665_regulators),

	/* haptic */
#ifdef CONFIG_MOTOR_DRV_MAX77665
	.haptic_pdata = &max77665_haptic_pdata,
#endif
};
#endif

static struct i2c_board_info __initdata i2c_devs14[] = {
#if defined(CONFIG_MFD_MAX77665)
	{
		I2C_BOARD_INFO("max77665", (0xcc >> 1)),
		.platform_data	= &m040_max77665_info,
		.irq = IRQ_EINT(3),
	},
#endif
};

/*gpio i2c 15*/
static struct i2c_gpio_platform_data gpio_i2c15_data = {
	.sda_pin = M040_SDA_FUEL0,
	.scl_pin = M040_SCL_FUEL0,
	.udelay = 5,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device m040_device_gpio_i2c15 = {
	.name = "i2c-gpio",
	.id = 15,
	.dev.platform_data = &gpio_i2c15_data,
};

#if defined(CONFIG_BATTERY_BQ27541_I2C)
static struct bq27541_platform_data __initdata m040_bq27541_info = {
	.wakeup = true,
	.name = "m03x-fuelgauge",
	.low_bat_gpio = M040_BAT_LOW_IRQ,
};
#endif

static struct i2c_board_info __initdata i2c_devs15[] = {
#if defined(CONFIG_BATTERY_BQ27541_I2C)
	{
		I2C_BOARD_INFO("bq27541", (0xaa >> 1)),
		.platform_data	= &m040_bq27541_info,
	},
#endif
};
#if defined(CONFIG_MFD_MXQM)
/*
struct mx_qm_platform_data {
	unsigned int gpio_wake;
	unsigned int gpio_reset;
	unsigned int gpio_irq;
};*/

/*mxqm*/
static struct mx_qm_platform_data __initdata mxqm_pd = {
	.gpio_wake 	= M040_TOUCHPAD_WAKE,
	.gpio_reset 	= M040_TOUCHPAD_RESET,
	.gpio_irq = M040_TOUCHPAD_INT,
};
#endif

/*gpio i2c 16*/
static struct i2c_gpio_platform_data gpio_i2c16_data = {
	.sda_pin = M040_SDA_TOUCHPAD,
	.scl_pin = M040_SCL_TOUCHPAD,
	.udelay = 4,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device m040_device_gpio_i2c16 = {
	.name = "i2c-gpio",
	.id = 16,
	.dev.platform_data = &gpio_i2c16_data,
};


static struct i2c_board_info __initdata i2c_devs16[] = {
#if defined(CONFIG_MFD_MXQM)
	{
		I2C_BOARD_INFO("mx_qm", 0x43),
		.platform_data	= &mxqm_pd,
		.irq = IRQ_EINT(8),
	},
#endif
};


/*gpio i2c 17*/
static struct i2c_gpio_platform_data gpio_i2c17_data = {
	.sda_pin = M040_SDA_CAMERA1,
	.scl_pin = M040_SCL_CAMERA1,
	.udelay = 2,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device m040_device_gpio_i2c17 = {
	.name = "i2c-gpio",
	.id = 17,
	.dev.platform_data = &gpio_i2c17_data,
};

#if defined(CONFIG_KEYBOARD_GPIO)
static struct gpio_keys_button m040_gpio_keys_tables[] = {
	{
		.code		= KEY_POWER,
		.gpio			= M040_POWER_KEY,
		.desc			= "gpio-keys: KEY_POWER",
		.type			= EV_KEY,
		.active_low	= 1,
		.wakeup		= 1,
		.debounce_interval	= 1,
	}, {
		.code		= KEY_HOME,
		.gpio			= M040_HOME_KEY,
		.desc			= "gpio-keys: KEY_HOME",
		.type			= EV_KEY,
		.active_low	= 1,
		.wakeup		= 1,
		.debounce_interval	= 1,
	}, {
		.code		= KEY_VOLUMEDOWN,
		.gpio			= M040_VOLUMEDOWN_KEY,
		.desc			= "gpio-keys: KEY_VOLUMEDOWN",
		.type			= EV_KEY,
		.active_low	= 1,
		.debounce_interval	= 1,
	}, {
		.code		= KEY_VOLUMEUP,
		.gpio			= M040_VOLUMEUP_KEY,
		.desc			= "gpio-keys: KEY_VOLUMEDOWN",
		.type			= EV_KEY,
		.active_low	= 1,
		.debounce_interval	= 1,
	},
};

static struct gpio_keys_platform_data m040_gpio_keys_data = {
	.buttons		= m040_gpio_keys_tables,
	.nbuttons		= ARRAY_SIZE(m040_gpio_keys_tables),
};

static struct platform_device m040_gpio_keys = {
	.name			= "gpio-keys",
	.dev			= {
		.platform_data	= &m040_gpio_keys_data,
	},
};
#endif

static struct usb_detect_platform_data usb_detect_data = {
	.usb_vbus_gpio = M040_INT_USB,
	.usb_host_gpio = M040_INT_USB_MHL,
	.usb_dock_gpio = M040_DOCK_IRQ,
};

static struct platform_device usb_detect_device = {
	.name	 = "usb_detect",
	.id = -1,
	.dev	 = {
		.platform_data    = &usb_detect_data,
	},
	.num_resources	= 0,
	.resource 	= NULL,
};

#ifdef CONFIG_BRCM_GPS
static struct platform_device brcm_gps = {
	.name = "brcm-gps",
	.id = -1,
};
#endif

#ifdef CONFIG_WAKEUP_ASSIST
static struct platform_device wakeup_assist_device = {
	.name   = "wakeup_assist",
};
#endif

#ifdef CONFIG_XMM6260_MODEM
static struct modem_platform_data xmm6260_data = {
	.name = "xmm6260",
	.gpio_phone_on = M040_GPIO_MODEM_ON,
	.gpio_cp_reset = M040_GPIO_MODEM_RST,
	.gpio_pmu_reset = M040_GPIO_MODEM_RST_FULL,
	.gpio_ipc_slave_wakeup = M040_GPIO_SLAVE_WAKEUP,
	.gpio_ipc_host_wakeup = M040_GPIO_HOST_WAKEUP,
	.gpio_suspend_request = M040_GPIO_REQUEST_SUSPEND,
	.gpio_active_state = M040_GPIO_HOST_ACTIVE,
	.gpio_cp_dump_int = M040_GPIO_MODEM_DUMP_INT,
	.gpio_cp_reset_int = M040_GPIO_MODEM_RESET_INT,
	.gpio_sim_detect_int = 0,
	.wakeup = 1,
};

static struct platform_device xmm6260_modem = {
	.name = "xmm_modem",
	.id = -1,

	.dev = {
		.platform_data = &xmm6260_data,
	},
};
#endif

#ifdef CONFIG_EXYNOS4_DEV_PPMU
/* image, peril, mfc_l, g3d */
static struct exynos4_ppmu_pd __initdata ppmu_pd_dmc_l = {
	.name = "ppmu_dmc_l",
	.count[0] = 0xfff80000U,
	.event[0] = RW_BUSY,
	.event[1] = RD_LATENCY,
	.event[2] = WR_LATENCY,
	.event[3] = RW_DATA,
};

/* lcd, fsys, cam, perir, mfc_r */
static struct exynos4_ppmu_pd __initdata ppmu_pd_dmc_r = {
	.name = "ppmu_dmc_r",
	.count[0] = 0xfff80000U,
	.event[0] = RW_BUSY,
	.event[1] = RD_REQ,
	.event[2] = WR_REQ,
	.event[3] = RW_DATA,
};

static struct exynos4_ppmu_pd __initdata ppmu_pd_cpu = {
	.name = "ppmu_cpu",
	.count[0] = 0xf0000000U,
	.event[0] = RW_BUSY,
	.event[1] = RD_REQ,
	.event[2] = WR_REQ,
	.event[3] = RW_DATA,
};
#endif

#ifdef CONFIG_BUSFREQ_OPP
static struct device_domain busfreq;
static struct platform_device exynos4_busfreq = {
	.id = -1,
	.name = "exynos-busfreq",
};
#endif

#ifdef CONFIG_CPU_FREQ
static struct exynos_cpufreq_platdata __initdata m040_cpufreq_pd = {
	.regulator = "vdd_arm",
	.gpio_dvfs = true,
	.polling_ms = 50,	//50mS
};
#endif
static struct platform_device __initdata *m040_devices[]  = {
#ifdef CONFIG_WAKEUP_ASSIST
	&wakeup_assist_device,
#endif
	&exynos4_device_pd[PD_MFC],
	&exynos4_device_pd[PD_G3D],
	&exynos4_device_pd[PD_LCD0],
	&exynos4_device_pd[PD_CAM],
	&exynos4_device_pd[PD_TV],
	&exynos4_device_pd[PD_GPS],
	&exynos4_device_pd[PD_GPS_ALIVE],
	&exynos4_device_pd[PD_ISP],

	&s3c_device_i2c0,
	&s3c_device_i2c1,
//	&s3c_device_i2c3,
	&s3c_device_i2c4,
	&s3c_device_i2c5,
	&s3c_device_i2c6,
	&s3c_device_i2c7,
	&m040_device_gpio_i2c8,
	&m040_device_gpio_i2c9,
	&m040_device_gpio_i2c10,
	&m040_device_gpio_i2c11,
	&m040_device_gpio_i2c12,
	&m040_device_gpio_i2c13,
	&m040_device_gpio_i2c14,
	&m040_device_gpio_i2c15,
	&m040_device_gpio_i2c16,
	&m040_device_gpio_i2c17,
#if defined(CONFIG_KEYBOARD_GPIO)
	&m040_gpio_keys,
#endif

#if defined(CONFIG_REGULATOR_FIXED_VOLTAGE)
	&m040_wm8958_fixed_voltage0,
	&m040_wm8958_fixed_voltage1,
	&m040_wm8958_fixed_voltage2,
	&m040_lcd_fixed_voltage0,
	&m040_lcd_fixed_voltage1,
	&m040_lcd_fixed_voltage2,
	&m040_fixed_voltage1,
	&m040_fixed_voltage2,
	&m040_fixed_voltage3,
	&m040_hdmi_fixed_voltage,
	&m040_mhl_fixed_voltage,
	&m040_isp_fixed_voltage,
	&m040_backlight_fixed_voltage,

#endif

#if defined(CONFIG_FB_S5P)
	&s3c_device_fb,
#endif

#if defined(CONFIG_FB_S5P_MIPI_DSIM)
	&s5p_device_mipi_dsim0,
#endif

#if defined(CONFIG_EXYNOS4_DEV_DWMCI)
	&exynos_device_dwmci,
#endif

#ifdef CONFIG_EXYNOS4_DEV_MSHC
	&s3c_device_mshci,
#endif

#if defined(CONFIG_SND_SAMSUNG_I2S)
	&exynos_device_i2s0,
	&samsung_asoc_idma,
#endif
#ifdef CONFIG_SND_SAMSUNG_PCM
	&exynos_device_pcm1,
	&samsung_asoc_dma,
#endif
#ifdef CONFIG_SND_SAMSUNG_SPDIF
	&exynos_device_spdif,
#endif
#ifdef CONFIG_SND_SOC_SAMSUNG_SMDK_SPDIF
	&m040_spdif_device,
#endif
#ifdef CONFIG_SND_SOC_M040_WM8958
	&m040_audio_device,
#endif
#ifdef CONFIG_VIDEO_TVOUT
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif
#if defined(CONFIG_VIDEO_FIMC)
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
	&s3c_device_fimc3,
#endif

#if defined(CONFIG_VIDEO_MFC5X)
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
#endif

#ifdef CONFIG_VIDEO_FIMC_MIPI
	&s3c_device_csis0,
#endif

#ifdef CONFIG_S3C2410_WATCHDOG
	&s3c_device_wdt,
#endif

#if defined(CONFIG_S3C_DEV_RTC)
	&s3c_device_rtc,
#endif

#ifdef CONFIG_XMM6260_MODEM
	&xmm6260_modem,
#endif
#if defined(CONFIG_USB_GADGET)
	&s3c_device_usbgadget,
#endif

#if defined(CONFIG_VIDEO_FIMG2D)
	&s5p_device_fimg2d,
	&SYSMMU_PLATDEV(g2d_acp),
#endif
	&usb_detect_device,
	&s3c_device_adc,
#ifdef CONFIG_S3C_DEV_HWMON
	&s3c_device_hwmon,
#endif

#ifdef CONFIG_BRCM_GPS
	&brcm_gps,
#endif

#if defined(CONFIG_VIDEO_JPEG_V2X) || defined(CONFIG_VIDEO_JPEG_M03X)
	&s5p_device_jpeg,
#endif
#ifdef CONFIG_SWITCH_GPIO
	&m040_switch_gpio,
#endif

#ifdef CONFIG_BT
	&m040_bt,
#endif

#ifdef CONFIG_CPU_FREQ
	&exynos_device_cpufreq,
#endif

#ifdef CONFIG_EXYNOS4_DEV_PPMU
	&PPMU_PLATDEV(dmc_l),
	&PPMU_PLATDEV(dmc_r),
	&PPMU_PLATDEV(cpu),
#endif

#ifdef CONFIG_EXYNOS4_BUS_DEVFREQ
	&exynos_device_busfreq,
#endif
#ifdef CONFIG_BUSFREQ_OPP
	&exynos4_busfreq,
#endif

#ifdef CONFIG_SENSORS_EXYNOS4_TMU
	&exynos4_device_tmu,
#endif

#ifdef CONFIG_HAVE_PWM
	&s3c_device_timer[1],	/* for vibrator */
#endif

#ifdef CONFIG_VIDEO_ROTATOR
	&s5p_device_rotator,
#endif

#if defined(CONFIG_BATTERY_MEIZU)
	&m040_battery_device,
#endif
};

static void __init m040_gpio_irq_init(void)
{
	unsigned gpio;
	int ret;

	gpio = M040_CAMERA_ISP_IRQ;
	ret = gpio_request(gpio, "M6MO_INT");
	ret = s5p_register_gpio_interrupt(gpio);
	gpio_free(gpio);
}

#if defined(CONFIG_SAMSUNG_DEV_BACKLIGHT)
/* LCD Backlight data */
static struct samsung_bl_gpio_info m040_bl_gpio_info = {
	.no = EXYNOS4_GPD0(0),
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data m040_bl_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 180,
	.pwm_period_ns  = 78770,
};
#endif

#ifdef CONFIG_VIDEO_FIMG2D
static struct fimg2d_platdata __initdata fimg2d_data = {
	.hw_ver = 0x41,
	.parent_clkname = "mout_g2d0",
	.clkname = "sclk_fimg2d",
	.gate_clkname = "g2d_acp",
	.clkrate = 201 * 1000000,	/* 200 Mhz */
};
#endif

static void __init m040_sysmmu_init(void)
{
#if defined(CONFIG_VIDEO_FIMG2D)
	sysmmu_set_owner(&SYSMMU_PLATDEV(g2d_acp).dev, &s5p_device_fimg2d.dev);
#endif
}

static void __init m040_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(m040_uartcfgs, ARRAY_SIZE(m040_uartcfgs));
#if defined(CONFIG_S5P_MEM_CMA)
	m040_reserve_mem();
#endif
}

static void __init m040_machine_init(void)
{
	m040_gpio_irq_init();
	m040_sysmmu_init();

	/* init i2c part */
	s3c_i2c0_set_platdata(&m040_default_i2c0_data);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
	/* max77668 */
	s3c_i2c1_set_platdata(&m040_default_i2c1_data);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	/* es305b */
#ifdef CONFIG_AUDIENCE_ES305B
	s3c_i2c4_set_platdata(&m040_default_i2c4_data);
	i2c_register_board_info(4, i2c_devs4, ARRAY_SIZE(i2c_devs4));
#endif
	s3c_i2c6_set_platdata(&m040_default_i2c6_data);
	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));
	/* lis3dh */
	i2c_register_board_info(9, i2c_devs9, ARRAY_SIZE(i2c_devs9));
	/* akm8975 */
	i2c_register_board_info(10, i2c_devs10, ARRAY_SIZE(i2c_devs10));
	/* l3g4200d */
#if defined(CONFIG_SENSORS_L3G4200D) || defined(CONFIG_SENSORS_L3GD20) || defined(CONFIG_SENSORS_GP2AP020A00F)
	i2c_register_board_info(11, i2c_devs11, ARRAY_SIZE(i2c_devs11));
#endif
	/* gp2ap */
	i2c_register_board_info(12, i2c_devs12, ARRAY_SIZE(i2c_devs12));
	/* max77665 */
	i2c_register_board_info(14, i2c_devs14, ARRAY_SIZE(i2c_devs14));
	/*Ti fuelguage  bq27541 */
	i2c_register_board_info(15, i2c_devs15, ARRAY_SIZE(i2c_devs15));
	/*MX qm */
	i2c_register_board_info(16, i2c_devs16, ARRAY_SIZE(i2c_devs16));

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos_dwmci_set_platdata(&m040_dwmci_pdata);
#endif
#ifdef CONFIG_EXYNOS4_DEV_MSHC
#endif

#ifdef CONFIG_VIDEO_FIMG2D
	s5p_fimg2d_set_platdata(&fimg2d_data);
#endif

#ifdef CONFIG_VIDEO_MFC5X
	exynos4_mfc_setup_clock(&s5p_device_mfc.dev, 200 * MHZ);
#ifndef CONFIG_PM_GENERIC_DOMAINS
	s5p_device_mfc.dev.parent = &exynos4_device_pd[PD_MFC].dev;
#endif
#endif

#if defined(CONFIG_USB_GADGET)
	s5p_usbgadget_set_platdata(NULL);
#endif

#if defined(CONFIG_SAMSUNG_DEV_BACKLIGHT)
	samsung_bl_set(&m040_bl_gpio_info, &m040_bl_data);
#endif

#ifdef CONFIG_S3C_DEV_HWMON
	s3c_hwmon_set_platdata(&m040_hwmon_pdata);
#endif

#if defined(CONFIG_VIDEO_JPEG_V2X) || defined(CONFIG_VIDEO_JPEG_M03X)
	exynos4_jpeg_setup_clock(&s5p_device_jpeg.dev, 160000000);
#ifndef CONFIG_PM_GENERIC_DOMAINS
	s5p_device_jpeg.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif

#ifdef CONFIG_CPU_FREQ
	exynos_cpufreq_set_platdata(&m040_cpufreq_pd);
#endif

#ifdef CONFIG_EXYNOS4_DEV_PPMU
	exynos_ppmu_set_pd(&ppmu_pd_dmc_l, &PPMU_PLATDEV(dmc_l));
	exynos_ppmu_set_pd(&ppmu_pd_dmc_r, &PPMU_PLATDEV(dmc_r));
	exynos_ppmu_set_pd(&ppmu_pd_cpu, &PPMU_PLATDEV(cpu));
#endif

#ifdef CONFIG_EXYNOS4_BUS_DEVFREQ
	exynos_busfreq_set_platdata();
#endif

#ifdef CONFIG_BUSFREQ_OPP
	dev_add(&busfreq, &exynos4_busfreq.dev);
#endif

#ifdef CONFIG_SENSORS_EXYNOS4_TMU
	exynos4_tmu_set_platdata();
#endif

	platform_add_devices(m040_devices, ARRAY_SIZE(m040_devices));
}

MACHINE_START(M040, "MX2")

	/* Maintainer: WenbinWu <wenbinwu@meizu.com> */
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= m040_map_io,
	.init_machine	= m040_machine_init,
	.timer		= &exynos4_timer,
MACHINE_END
