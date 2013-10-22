/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/memory.h>
#include <linux/mutex.h>

#include <plat/gpio-cfg.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/devs.h>

#ifdef CONFIG_INPUT_LSM330DLC
#include <linux/input/lsm330dlc.h>
#endif
#ifdef CONFIG_SENSORS_AK8963
#include <linux/input/akm8963.h>
#endif

#ifdef CONFIG_INPUT_LSM330DLC
struct lsm330dlc_acc_platform_data accelerometer_pdata = {
	.poll_interval = 10,   /*default poll delay 10 ms*/
	.min_interval = 1,

	.fs_range = LSM330DLC_ACC_G_2G,

	.axis_map_x = 1,  /*x=-x, y=-y, z=z*/
	.axis_map_y = 0,
	.axis_map_z = 2,

	.negate_x = 1,
	.negate_y = 1,
	.negate_z = 1,
};

struct lsm330dlc_gyr_platform_data gyroscope_pdata = {
	.poll_interval = 10,
	.fs_range = LSM330DLC_GYR_FS_2000DPS,
	.min_interval = 1,

	.axis_map_x = 0,  /*x=-x, y=-y, z=-z*/
	.axis_map_y = 1,
	.axis_map_z = 2,

	.negate_x = 1,
	.negate_y = 1,
	.negate_z = 1,
};
#endif


#ifdef CONFIG_INPUT_LSM330DLC
/*accelerometer and gyroscope*/
static struct i2c_board_info __initdata i2c_devs_accelerometer[]  = {
	{
		I2C_BOARD_INFO(LSM330DLC_ACC_DEV_NAME, 0x19),
		.platform_data = &accelerometer_pdata,
	},
};
static struct i2c_board_info __initdata i2c_devs_gyroscope[]  = {
	{
		I2C_BOARD_INFO(LSM330DLC_GYR_DEV_NAME, 0x6b ),
		.platform_data = &gyroscope_pdata,
	},
};
/*end of accelerometer and gyroscope*/
#endif

#ifdef CONFIG_SENSOR_GP2AP
static struct i2c_board_info __initdata i2c_devs_gp2ap[] = {
	{
		I2C_BOARD_INFO("gp2ap030a00f", 0x39),
		//.irq = MEIZU_IR_IRQ,
	}

};
#endif

#ifdef CONFIG_SENSORS_AK8963
static struct akm8963_platform_data akm_platform_data_8963 = {
	.gpio_DRDY = 0,
	.gpio_RSTN = 0,
	.layout = 3,
};
static struct i2c_board_info __initdata i2c_devs_akm[] = {
	{
		I2C_BOARD_INFO("akm8963", 0x0c),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &akm_platform_data_8963,
	}
};
#endif

#ifdef CONFIG_SENSOR_GP2AP
static void __init senor_gp2ap_rt_init_res(void) {
	i2c_devs_gp2ap[0].irq = MEIZU_IR_IRQ;
}
#endif

void __init mx2_sensors_init(void)
{
	/*for accelerometer and gyroscope*/
#ifdef CONFIG_INPUT_LSM330DLC
	pr_info("new sensor device lsm330dlc registered\n");
	i2c_register_board_info(9, i2c_devs_accelerometer, ARRAY_SIZE(i2c_devs_accelerometer));
	i2c_register_board_info(11, i2c_devs_gyroscope, ARRAY_SIZE(i2c_devs_gyroscope));
#endif

#ifdef CONFIG_SENSOR_GP2AP
	senor_gp2ap_rt_init_res();
	i2c_register_board_info(15, i2c_devs_gp2ap, ARRAY_SIZE(i2c_devs_gp2ap));
#endif

#ifdef CONFIG_SENSORS_AK8963
	pr_info("new sensor device ak8963 registered\n");
	i2c_register_board_info(10, i2c_devs_akm, ARRAY_SIZE(i2c_devs_akm));
#endif
}
