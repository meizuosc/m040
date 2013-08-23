/*
 * Reboot test case for Built-in autotest framework
 *
 * Copyright (C) 2012 Meizu Co,. Ltd.
 *
 * Author: Wu Zhangjin <falcon@meizu.com> or <wuzhangjin@gmail.com>
 * Update: Sun Apr 29 12:45:37 CST 2012
 */

#define pr_fmt(fmt) "AUTOTEST_REBOOT: " fmt

#include <linux/autotest/core.h>
#include <linux/wakelock.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/bootmode.h>

#ifdef CONFIG_ANDROID_RAM_CONSOLE
extern void set_reboot_test(void);
#else
#define set_reboot_test()	do { } while (0)
#endif

static struct wake_lock reboot_wake_lock;
struct task_struct *reboot_task;
unsigned int  __read_mostly sysctl_reboot_test;
unsigned long __read_mostly sysctl_reboot_count;
unsigned long __read_mostly sysctl_reboot_cycle_secs = CONFIG_AUTOTEST_REBOOT_CYCLE;

int proc_doreboot_test(struct ctl_table *table, int write,
				  void __user *buffer,
				  size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);

	if (ret || !write)
		goto out;

	set_reboot_test();

	pr_info("Start reboot test, reboot_count = %ld times, reboot_cycle = %ld seconds\n",
		sysctl_reboot_count, sysctl_reboot_cycle_secs);

	wake_up_process(reboot_task);

 out:
	return ret;
}

static unsigned long timeout_jiffies(unsigned long timeout)
{
	/* timeout of 0 will disable the watchdog */
	return timeout ? timeout * HZ : MAX_SCHEDULE_TIMEOUT;
}

static int reboot_thread(void *data)
{
	unsigned int reboot_cycle;

	reboot_cycle = sysctl_reboot_test ? get_random_secs(1, sysctl_reboot_cycle_secs) + 1 : 0;
	if (reboot_cycle) {
		pr_info("%s: Enter into reboot_thread\n", __func__);

		pr_info("%s: Sleep %d seconds for reboot\n", __func__, reboot_cycle);
	}

	while (schedule_timeout_interruptible(timeout_jiffies(reboot_cycle)))
		reboot_cycle = sysctl_reboot_test ? get_random_secs(1, sysctl_reboot_cycle_secs) + 1 : 0;

	pr_info("%s: Sync all changes to disk\n", __func__);
	/* Sync the changes from cache to disk */
	sys_sync();

#ifdef CONFIG_AUTOTEST_SUSPEND
	pr_info("%s: Stop suspend just before sending out restart command\n", __func__);
	wake_lock(&reboot_wake_lock);
#endif

	pr_info("%s: Send out the restart command ...\n", __func__);
	kernel_restart(NULL);

	return 0;
}

void start_reboot_thread(void)
{
	wake_lock_init(&reboot_wake_lock, WAKE_LOCK_SUSPEND, "autotest_reboot");

#ifndef CONFIG_AUTOTEST_SUSPEND
	/* Stop suspend when doing reboot test */
	wake_lock(&reboot_wake_lock);
#endif

	pr_info("%s: Start reboot test thread\n", __func__);
	reboot_task = kthread_run(reboot_thread, NULL, "reboottest/daemon");
	if (IS_ERR(reboot_task))
		pr_err("%s: Fail to create reboot_thread\n", __func__);
}
