/* mach/touch_booster.h
 *
 * Copyright (C) 2012 ZhuHai MEIZU Technology Co., Ltd.
 *	  http://www.meizu.com
 *
 * Meizu touch booster interface
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __H_TOUCH_BOOSTER__
#define __H_TOUCH_BOOSTER__

#include <linux/sysdev.h>
#include <linux/input.h>

#define MAX_BOOST_DEVICE		3
#define DEFUALT_BOOST_FREETIME		200000	/*400ms*/
#define DEFUALT_BOOST_4210_BUSFREQ	267000
#define DEFUALT_BOOST_4412_BUSFREQ	267200
#define DEFUALT_LAUNCH_BOOST_CPUFREQ    1200000	/* 1.2GHz */
#define DEFUALT_FIRST_BOOST_CPUFREQ	800000	/* 800Mhz */
#define DEFUALT_SECOND_BOOST_CPUFREQ	600000	/* 600Mhz */
#define TABLE_SIZE			3

struct tb_private_info {
	struct class tb_class;
#ifdef CONFIG_BUSFREQ_OPP
	struct device *bus_dev;
	struct device dev;
#endif
	unsigned int down_time;
	unsigned int boost_cpufreq_level;
	unsigned int lock_busfreq;
	unsigned int boost_debug;

	struct input_handle handle[MAX_BOOST_DEVICE];
	struct workqueue_struct *wq;
	struct work_struct boost_work;
	atomic_t boost_lock;

	ktime_t key_time;
};

extern unsigned int app_into;
#endif
