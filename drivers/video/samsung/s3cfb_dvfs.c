/* linux/drivers/video/samsung/s3cfb-main.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Core file for Samsung Display Controller (FIMD) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/pm_runtime.h>
#include <plat/clock.h>
#include <plat/media.h>
#include <mach/media.h>
#include "s3cfb.h"

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif

#ifdef CONFIG_BUSFREQ_OPP
#include <mach/dev.h>
#include <plat/cpu.h>
#endif

struct s3cfb_dvfs_info
{
	struct s3cfb_global *fbdev;
#ifdef CONFIG_HAS_WAKELOCK
	struct early_suspend	early_suspend;
#endif

	struct workqueue_struct *fimd_dvfs;
	struct delayed_work  dvfs_work;
	struct notifier_block dvfs_nb;
	u32 src_clk;
	int dynamic_freq_disable;
};
static struct s3cfb_dvfs_info *fbdvfs;

#include <asm/mach-types.h>
static ssize_t s3cfb_sysfs_show_dynamic_freq_disable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
    return sprintf(buf, "%d\n", fbdvfs->dynamic_freq_disable);
}
static ssize_t s3cfb_sysfs_store_dynamic_freq_disable(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	unsigned long value;
	
	sscanf(buf, "%lu", &value);
	fbdvfs->dynamic_freq_disable = !!value;

	return count;
}
static DEVICE_ATTR(dynamic_freq_disable, S_IRUGO | S_IWUSR, s3cfb_sysfs_show_dynamic_freq_disable, s3cfb_sysfs_store_dynamic_freq_disable);
static void set_lcd_freq(struct work_struct *work)
{
	struct s3cfb_dvfs_info *dvfsdev =
		 list_entry(work, struct s3cfb_dvfs_info, dvfs_work.work);
	struct s3cfb_global *fbdev = dvfsdev->fbdev;
#ifdef CONFIG_EXYNOS_DEV_PD
	struct platform_device *pdev = to_platform_device(fbdev->dev);

	if (fbdev->system_state == POWER_OFF)
		return;
	pm_runtime_get_sync(&pdev->dev);
#endif
	s3cfb_frame_adjust(fbdev, FB_DYNAMIC_LOW_FREQ);
#ifdef CONFIG_EXYNOS_DEV_PD
	pm_runtime_put(&pdev->dev);
#endif
}

static int lcd_dvfs_notify(struct notifier_block *nb,
			  unsigned long action, void *data)
{
	switch (action) {
	case FB_EVENT_MODE_PAN:
	{
		struct s3cfb_dvfs_info *dvfsdev =
			list_entry(nb, struct s3cfb_dvfs_info, dvfs_nb);
		struct s3cfb_global *fbdev = dvfsdev->fbdev;

		if(!dvfsdev->dynamic_freq_disable){
#ifdef CONFIG_EXYNOS_DEV_PD
			if (fbdev->system_state == POWER_OFF)
				return NOTIFY_OK;
			pm_runtime_get_sync(fbdev->dev);
#endif

			if (delayed_work_pending(&dvfsdev->dvfs_work))
				cancel_delayed_work(&dvfsdev->dvfs_work);
			queue_delayed_work(dvfsdev->fimd_dvfs, &dvfsdev->dvfs_work, HZ);
			s3cfb_frame_adjust(fbdev, FB_DYNAMIC_HIGH_FREQ);
#ifdef CONFIG_EXYNOS_DEV_PD
			pm_runtime_put(fbdev->dev);
#endif
		}
		break;
	}
	default:
		break;
	}

	return NOTIFY_OK;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void s3cfb_dvfs_early_suspend(struct early_suspend *h)
{
	struct s3cfb_dvfs_info *info = container_of(h, struct s3cfb_dvfs_info, early_suspend);

	if (!machine_is_m030()) {
		flush_delayed_work_sync(&info->dvfs_work);
	}

	return;
}

static void s3cfb_dvfs_late_resume(struct early_suspend *h)
{
	struct s3cfb_dvfs_info *info = container_of(h, struct s3cfb_dvfs_info, early_suspend);

	dev_dbg(info->fbdev->dev, "wake up from suspend\n");

	return;
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

int s3cfb_dvfs_init(struct s3cfb_global *fbdev)
{
	int ret;
	struct s3cfb_dvfs_info *s3cfb_dvfs;

	s3cfb_dvfs = kzalloc(sizeof(struct s3cfb_dvfs_info), GFP_KERNEL);
	if (!s3cfb_dvfs) {
		dev_err(fbdev->dev, "failed to allocate for global fbfimd structure\n");
		return -1;
	}

	if (!machine_is_m030()) {
		INIT_DELAYED_WORK(&s3cfb_dvfs->dvfs_work, set_lcd_freq);
		s3cfb_dvfs->fimd_dvfs = create_singlethread_workqueue("lcd_fresh");
		s3cfb_dvfs->dvfs_nb.notifier_call = lcd_dvfs_notify;
		fb_register_client(&s3cfb_dvfs->dvfs_nb);
	}

	ret = device_create_file(fbdev->dev, &dev_attr_dynamic_freq_disable);
	if (ret < 0)
		dev_err(fbdev->dev, "failed to add sysfs entries\n");

#ifdef CONFIG_HAS_WAKELOCK
#ifdef CONFIG_HAS_EARLYSUSPEND
		s3cfb_dvfs->early_suspend.suspend = s3cfb_dvfs_early_suspend;
		s3cfb_dvfs->early_suspend.resume = s3cfb_dvfs_late_resume;
		s3cfb_dvfs->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB-15;

		register_early_suspend(&s3cfb_dvfs->early_suspend);
#endif
#endif
	s3cfb_dvfs->fbdev = fbdev;
	s3cfb_dvfs->dynamic_freq_disable = 1;
	fbdvfs = s3cfb_dvfs;
	return 0;
}
void s3cfb_dvfs_remove(struct s3cfb_global *fbdev)
{
	if (!machine_is_m030()) {
		destroy_workqueue(fbdvfs->fimd_dvfs);
		fb_unregister_client(&fbdvfs->dvfs_nb);
	}

#ifdef CONFIG_HAS_WAKELOCK
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&fbdvfs->early_suspend);
#endif
#endif
	kfree(fbdvfs);
}
