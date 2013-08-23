/* linux/arch/arm/mach-exynos/mx2_reboot.c
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *
 * Author: lvcha qiu <lvcha@meizu.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 * Revision History
 *
 * Inital code : Apr 16 , 2012 : lvcha@meizu.com
 *
*/

#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/sched.h>

#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>

#include <mach/system.h>
#include <mach/regs-pmu.h>
#include <mach/gpio.h>

#define REBOOT_MODE_CHARGE		0x0
#define REBOOT_MODE_WIPE			0x1
#define REBOOT_MODE_UPGRADE		0x2
#define CUSTOM_MASK			0xFF

static void mx2_reboot_internal(const char *cmd)
{
	unsigned long custom_val;

	local_irq_disable();

	if(cmd) {
		if (strstr(cmd, "charge"))
			__raw_writel(REBOOT_MODE_CHARGE, S5P_INFORM4);
		else if (strstr(cmd, "wipe"))
			__raw_writel(REBOOT_MODE_WIPE, S5P_INFORM4);
		else if (strstr(cmd, "upgrade"))
			__raw_writel(REBOOT_MODE_UPGRADE, S5P_INFORM4);
		else if (strstr(cmd, "custom")) {
			if (!strict_strtoul(cmd+7, 10, &custom_val)) {
				__raw_writel(custom_val & CUSTOM_MASK,
								S5P_INFORM7);
				/* notify uboot reboot to recovery */
				__raw_writel(REBOOT_MODE_UPGRADE, S5P_INFORM4);
			}
			/* error cmd reboot to android */
		}
	}

	flush_cache_all();
	outer_flush_all();
	arch_reset(0, 0);

	pr_emerg("%s: waiting for reboot\n", __func__);
	while (1);
}

static void mx2_disable_inand(void)
{
	__raw_writel(0x100, S5P_VA_GPIO2 + 0x40);
	__raw_writel(0, S5P_VA_GPIO2 + 0x44);
	__raw_writel(0, S5P_VA_GPIO2 + 0x48);
	__raw_writel(0, S5P_VA_GPIO2 + 0x60);
	__raw_writel(0, S5P_VA_GPIO2 + 0x64);
	__raw_writel(0, S5P_VA_GPIO2 + 0x68);
}

static inline int is_cable_insert(void)
{
	int gpio;

	gpio = EXYNOS4_GPX0(2);
	
	return gpio_get_value(gpio);
}

static void mx2_power_off(void)
{
	struct task_struct *task = get_current();
	char task_com[TASK_COMM_LEN];

	pr_emerg("func:%s, process is:%d:%s\n", __func__, task->pid, get_task_comm(task_com, task));
	if (task->parent) {
		task = task->parent;
		pr_emerg("func:%s, parent:%d:%s\n", __func__, task->pid, get_task_comm(task_com, task));
		if (task->parent) {
			task = task->parent;
			pr_emerg("func:%s, pparent:%d:%s\n", __func__, task->pid, get_task_comm(task_com, task));
		}
	}
	mx2_disable_inand();
	if(is_cable_insert()){ /* 1. Check reboot charging */
		mx2_reboot_internal("charge");
	}else{	/* 2. Power off */
		int regs;
		regs = __raw_readl(S5P_PS_HOLD_CONTROL);
		/* dead loop to avoid sometimes auto restart*/
		while(1) {
			pr_emerg("%s: waiting for power off\n", __func__);
			__raw_writel(regs & 0xFFFFFEFF, S5P_PS_HOLD_CONTROL);
		}
	}
}

static void mx2_reboot(char str, const char *cmd)
{
	struct task_struct *task = get_current();
	char task_com[TASK_COMM_LEN];
	
	pr_emerg("%s (%d, %s)\n", __func__, str, cmd ? cmd : "(null)");
	pr_emerg("func:%s, process is:%d:%s\n", __func__, task->pid, get_task_comm(task_com, task));
	if (task->parent) {
		task = task->parent;
		pr_emerg("func:%s, parent:%d:%s\n", __func__, task->pid, get_task_comm(task_com, task));
		if (task->parent) {
			task = task->parent;
			pr_emerg("func:%s, pparent:%d:%s\n", __func__, task->pid, get_task_comm(task_com, task));
		}
	}

	mx2_disable_inand();
	mx2_reboot_internal(cmd);
}

static int __init mx2_reboot_init(void)
{
	/* to support system shut down */
	pm_power_off = mx2_power_off;
	arm_pm_restart = mx2_reboot;

	return 0;
}

arch_initcall_sync(mx2_reboot_init);

