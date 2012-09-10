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

struct tb_private_data {
	struct sysdev_class tb_class;
#ifdef CONFIG_BUSFREQ_OPP
	struct device *bus_dev;
	struct device dev;
#endif
	unsigned int down_time;
	unsigned int lock_cpufreq;
	unsigned int lock_busfreq;
};

extern void start_touch_boost(void);

#endif

