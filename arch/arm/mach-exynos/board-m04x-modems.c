/**
 * arch/arm/mach-exynos/board-m04x-modems.c
 *
 * Copyright (C) 2010 Samsung Electronics.
 * Copyright (C) 2012 Zhuhai Meizu Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/mach-types.h>

#ifdef CONFIG_MACH_M040
extern int m040_modem_device_init(void);
#endif
#ifdef CONFIG_MACH_M041
extern int m041_modem_device_init(void);
#endif

static int __init modem_device_init(void)
{
#ifdef CONFIG_MACH_M040
	if(machine_is_m040())
		return m040_modem_device_init();
#endif
#ifdef CONFIG_MACH_M041
	if(machine_is_m041())
		return m041_modem_device_init();
#endif
	return 0;
}

late_initcall(modem_device_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("WenBin Wu<wenbinwu@meizu.com>");
MODULE_DESCRIPTION("Meizu TD Modem Interface Driver");