/* arch/arm/mach-exynos/include/mach/modem.h
 *
 * Copyright (c) 2011 Meizu Technology Co., Ltd.
 *		http://www.meizu.com/
 *
 * Based on arch/arm/mach-s5p6442/include/mach/io.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MODEM_H__
#define __MODEM_H__

enum MODEM_EVENT_TYPE {
	MODEM_EVENT_POWEROFF,
	MODEM_EVENT_RESET,
	MODEM_EVENT_CRASH,
	MODEM_EVENT_DUMP,
	MODEM_EVENT_CONN,
	MODEM_EVENT_DISCONN,
	MODEM_EVENT_SIM,
	MODEM_EVENT_BOOT_INIT,
};

extern void modem_set_active_state(int state);
extern void modem_notify_event(int type);

#endif
