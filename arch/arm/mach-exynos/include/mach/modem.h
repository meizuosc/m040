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

extern int modem_is_on(void);
extern int modem_is_host_wakeup(void);
extern int modem_set_slave_wakeup(int val);
extern void modem_notify_event(int type);
extern void modem_set_active_state(int state);
#endif
