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

#ifdef CONFIG_FB_DYNAMIC_FREQ
int s3cfb_dvfs_init(struct s3cfb_global *fbdev);
int s3cfb_dvfs_remove(struct s3cfb_global *fbdev);
#endif

struct s3cfb_fimd_desc	*fbfimd;

#ifdef CONFIG_FB_S5P_VSYNC_THREAD
static ssize_t s3cfb_sysfs_vsync_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct s3cfb_global *fbdev[1];
	fbdev[0] = fbfimd->fbdev[0];

	if(fbdev[0]->vsync_debug)
		pr_info("%llu\n", ktime_to_ns(fbdev[0] ->vsync_info.timestamp));
	return scnprintf(buf, PAGE_SIZE, "%llu\n",
			ktime_to_ns(fbdev[0] ->vsync_info.timestamp));
}

static DEVICE_ATTR(vsync, S_IRUGO, s3cfb_sysfs_vsync_show, NULL);

static ssize_t s3cfb_sysfs_show_vsync_debug(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct s3cfb_global *fbdev[1];
	fbdev[0] = fbfimd->fbdev[0];
	
    return sprintf(buf, "%d\n", fbdev[0]->vsync_debug);
}
static ssize_t s3cfb_sysfs_store_vsync_debug(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	unsigned long value;
	struct s3cfb_global *fbdev[1];
	fbdev[0] = fbfimd->fbdev[0];
	
	sscanf(buf, "%lu", &value);
	fbdev[0]->vsync_debug = !!value;

	return count;
}
static DEVICE_ATTR(vsync_debug, S_IRUGO | S_IWUSR, s3cfb_sysfs_show_vsync_debug, s3cfb_sysfs_store_vsync_debug);

static ssize_t s3cfb_sysfs_show_vsync_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct s3cfb_global *fbdev[1];
	fbdev[0] = fbfimd->fbdev[0];
	
    return sprintf(buf, "%d\n", fbdev[0]->vsync_enable);
}
static ssize_t s3cfb_sysfs_store_vsync_enable(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	unsigned long value;
	struct s3cfb_global *fbdev[1];
	fbdev[0] = fbfimd->fbdev[0];
	
	sscanf(buf, "%lu", &value);
	fbdev[0]->vsync_enable = !!value;

	return count;
}
static DEVICE_ATTR(vsync_enable, S_IRUGO | S_IWUSR, s3cfb_sysfs_show_vsync_enable, s3cfb_sysfs_store_vsync_enable);

static void s3cfb_activate_vsync(struct s3cfb_global *fbdev)
{
	int prev_refcount;

	mutex_lock(&fbdev->vsync_info.irq_lock);
	prev_refcount = fbdev->vsync_info.irq_refcount++;
	if (!prev_refcount) {
		fbdev->vsync_info.active = true;
		s3cfb_set_global_interrupt(fbdev, 1);
		s3cfb_set_vsync_interrupt(fbdev, 1);
		if(fbdev->vsync_debug)
			dev_info(fbdev->dev, "%s", __func__);
	}

	mutex_unlock(&fbdev->vsync_info.irq_lock);
}

static void s3cfb_deactivate_vsync(struct s3cfb_global *fbdev)
{
	int new_refcount;

	mutex_lock(&fbdev->vsync_info.irq_lock);

	new_refcount = --fbdev->vsync_info.irq_refcount;
	WARN_ON(new_refcount < 0);
	if (!new_refcount) {
		fbdev->vsync_info.active = false;
		s3cfb_set_global_interrupt(fbdev, 0);
		s3cfb_set_vsync_interrupt(fbdev, 0);
		if(fbdev->vsync_debug)
			dev_info(fbdev->dev, "%s", __func__);
	}

	mutex_unlock(&fbdev->vsync_info.irq_lock);
}

int s3cfb_set_vsync_int(struct fb_info *info, bool active)
{
	struct s3cfb_global *fbdev = fbfimd->fbdev[0];

	if (active)
		s3cfb_activate_vsync(fbdev);
	else
		s3cfb_deactivate_vsync(fbdev);

	return 0;
}

/**
 * s3cfb_wait_for_vsync() - sleep until next VSYNC interrupt or timeout
 * @sfb: main hardware state
 * @timeout: timeout in msecs, or 0 to wait indefinitely.
 */
int s3cfb_wait_for_vsync(struct s3cfb_global *fbdev)
{
	ktime_t timestamp;
	int ret;
	u32 timeout = HZ/10;

	timestamp = fbdev->vsync_info.timestamp;
	ret = wait_event_interruptible_timeout(fbdev->vsync_info.wait,
					!ktime_equal(timestamp,
					fbdev->vsync_info.timestamp),
					msecs_to_jiffies(timeout));
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}

static int s3cfb_wait_for_vsync_thread(void *data)
{
	struct s3cfb_global *fbdev = data;

	while (!kthread_should_stop()) {
		ktime_t timestamp = fbdev->vsync_info.timestamp;
		int ret = wait_event_interruptible_timeout(
						fbdev->vsync_info.wait,
						!ktime_equal(timestamp,
						fbdev->vsync_info.timestamp) &&
						fbdev->vsync_info.active,
						msecs_to_jiffies(VSYNC_TIMEOUT_MSEC));

		if (ret > 0 && fbdev->vsync_enable) {
			sysfs_notify(&fbdev->dev->kobj, NULL, "vsync");
		}
	}

	return 0;
}
#endif
static irqreturn_t s3cfb_irq_frame(int irq, void *dev_id)
{
	struct s3cfb_global *fbdev[2];
	fbdev[0] = fbfimd->fbdev[0];

	spin_lock(&fbdev[0]->vsync_slock);	

	if (fbdev[0]->regs != 0)
		s3cfb_clear_interrupt(fbdev[0]);

#if defined(CONFIG_FB_S5P_VSYNC_THREAD)
	fbdev[0]->vsync_info.timestamp = ktime_get();
	wake_up_interruptible_all(&fbdev[0]->vsync_info.wait);
#endif

	fbdev[0]->wq_count++;
	wake_up(&fbdev[0]->wq);

	spin_unlock(&fbdev[0]->vsync_slock);
	return IRQ_HANDLED;
}

#ifdef CONFIG_FB_S5P_TRACE_UNDERRUN
static irqreturn_t s3cfb_irq_fifo(int irq, void *dev_id)
{
	struct s3cfb_global *fbdev[2];
	fbdev[0] = fbfimd->fbdev[0];

	if (fbdev[0]->regs != 0)
		s3cfb_clear_interrupt(fbdev[0]);
	pr_info("\n\nfifo underrun++++++++++\n");
	return IRQ_HANDLED;
}
#endif

int s3cfb_register_framebuffer(struct s3cfb_global *fbdev)
{
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);
	int ret, i, j;

	/* on registering framebuffer, framebuffer of default window is registered at first. */
	for (i = pdata->default_win; i < pdata->nr_wins + pdata->default_win; i++) {
		j = i % pdata->nr_wins;
		ret = register_framebuffer(fbdev->fb[j]);
		if (ret) {
			dev_err(fbdev->dev, "failed to register	\
				framebuffer device\n");
			goto err;
		}
#ifndef CONFIG_FRAMEBUFFER_CONSOLE
		if (j == pdata->default_win) {
			s3cfb_check_var_window(fbdev, &fbdev->fb[j]->var,
					fbdev->fb[j]);
			s3cfb_set_par_window(fbdev, fbdev->fb[j]);
			s3cfb_draw_logo(fbdev->fb[j]);
		}
#endif
	}
	return 0;

err:
	while (--i >= pdata->default_win) {
		j = i % pdata->nr_wins;
		unregister_framebuffer(fbdev->fb[j]);
	}
	return -EINVAL;
}

static int s3cfb_sysfs_show_win_power(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s3c_platform_fb *pdata = to_fb_plat(dev);
	struct s3cfb_window *win;
	char temp[16];
	int i;
	struct s3cfb_global *fbdev[1];
	fbdev[0] = fbfimd->fbdev[0];

	for (i = 0; i < pdata->nr_wins; i++) {
		win = fbdev[0]->fb[i]->par;
		sprintf(temp, "[fb%d] %s\n", i, win->enabled ? "on" : "off");
		strcat(buf, temp);
	}

	return strlen(buf);
}

static int s3cfb_sysfs_store_win_power(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct s3c_platform_fb *pdata = to_fb_plat(dev);
	char temp[4] = { 0, };
	const char *p = buf;
	int id, to;
	struct s3cfb_global *fbdev[1];
	fbdev[0] = fbfimd->fbdev[0];

	while (*p != '\0') {
		if (!isspace(*p))
			strncat(temp, p, 1);
		p++;
	}

	if (strlen(temp) != 2)
		return -EINVAL;

	id = simple_strtoul(temp, NULL, 10) / 10;
	to = simple_strtoul(temp, NULL, 10) % 10;

	if (id < 0 || id > pdata->nr_wins)
		return -EINVAL;

	if (to != 0 && to != 1)
		return -EINVAL;

	if (to == 0)
		s3cfb_disable_window(fbdev[0], id);
	else
		s3cfb_enable_window(fbdev[0], id);

	return len;
}

static DEVICE_ATTR(win_power, 0644,
	s3cfb_sysfs_show_win_power, s3cfb_sysfs_store_win_power);

static int s3cfb_probe(struct platform_device *pdev)
{
	struct s3c_platform_fb *pdata = NULL;
	struct resource *res = NULL;
	struct s3cfb_global *fbdev[2];
	int ret = 0;
	int i = 0;

#ifdef CONFIG_EXYNOS_DEV_PD
	/* to use the runtime PM helper functions */
	pm_runtime_enable(&pdev->dev);
	/* enable the power domain */
	pm_runtime_get_sync(&pdev->dev);
#endif
	fbfimd = kzalloc(sizeof(struct s3cfb_fimd_desc), GFP_KERNEL);
	if (!fbfimd) {
		dev_err(&pdev->dev, "failed to allocate for global fbfimd structure\n");
		goto err0;
	}

	if (FIMD_MAX == 2)
		fbfimd->dual = 1;
	else
		fbfimd->dual = 0;

	for (i = 0; i < FIMD_MAX; i++) {
		/* global structure */
		fbfimd->fbdev[i] = kzalloc(sizeof(struct s3cfb_global), GFP_KERNEL);
		fbdev[i] = fbfimd->fbdev[i];
		if (!fbdev[i]) {
			dev_err(fbdev[i]->dev, "failed to allocate for	\
				global fb structure fimd[%d]!\n", i);
			goto err0;
		}

		/* platform_data*/
		pdata = to_fb_plat(&pdev->dev);

		fbdev[i]->dev = &pdev->dev;
#ifndef CONFIG_FB_S5P_MIPI_DSIM	
		s3cfb_set_lcd_info(fbdev[i]);
#else
		fbdev[i]->lcd = (struct s3cfb_lcd *)pdata->lcd[i];
#endif

		if (pdata->cfg_gpio)
			pdata->cfg_gpio(pdev);

		if (pdata->clk_on)
			pdata->clk_on(pdev, &fbdev[i]->clock);

		/* io memory */
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			dev_err(fbdev[i]->dev,
				"failed to get io memory region\n");
			ret = -EINVAL;
			goto err1;
		}
		res = request_mem_region(res->start,
					resource_size(res), pdev->name);
		if (!res) {
			dev_err(fbdev[i]->dev,
				"failed to request io memory region\n");
			ret = -EINVAL;
			goto err1;
		}
		fbdev[i]->regs = ioremap(res->start, resource_size(res));
		if (!fbdev[i]->regs) {
			dev_err(fbdev[i]->dev, "failed to remap io region\n");
			ret = -EINVAL;
			goto err1;
		}
		spin_lock_init(&fbdev[i]->vsync_slock);
		/* irq */
		fbdev[i]->irq = platform_get_irq(pdev, 0);
		if (request_irq(fbdev[i]->irq, s3cfb_irq_frame, IRQF_SHARED,
				pdev->name, fbdev[i])) {
			dev_err(fbdev[i]->dev, "request_irq failed\n");
			ret = -EINVAL;
			goto err2;
		}

#ifdef CONFIG_FB_S5P_TRACE_UNDERRUN
		if (request_irq(platform_get_irq(pdev, 1), s3cfb_irq_fifo,
				IRQF_DISABLED, pdev->name, fbdev[i])) {
			dev_err(fbdev[i]->dev, "request_irq failed\n");
			ret = -EINVAL;
			goto err2;
		}

		s3cfb_set_fifo_interrupt(fbdev[i], 1);
		dev_info(fbdev[i]->dev, "fifo underrun trace\n");
#endif
		/* hw setting */
		s3cfb_init_global(fbdev[i]);

		fbdev[i]->system_state = POWER_ON;

		/* alloc fb_info */
		if (s3cfb_alloc_framebuffer(fbdev[i], i)) {
			dev_err(fbdev[i]->dev, "alloc error fimd[%d]\n", i);
			goto err3;
		}

		/* register fb_info */
		if (s3cfb_register_framebuffer(fbdev[i])) {
			dev_err(fbdev[i]->dev, "register error fimd[%d]\n", i);
			goto err3;
		}

		/* enable display */
		s3cfb_set_clock(fbdev[i]);
		s3cfb_enable_window(fbdev[0], pdata->default_win);

		s3cfb_update_power_state(fbdev[i], pdata->default_win,
					FB_BLANK_UNBLANK);
		s3cfb_display_on(fbdev[i]);

#ifdef CONFIG_BUSFREQ_OPP
		/* To lock bus frequency in OPP mode */
		fbdev[i]->bus_dev = dev_get("exynos-busfreq");
#endif

#ifdef CONFIG_HAS_WAKELOCK
#ifdef CONFIG_HAS_EARLYSUSPEND
		fbdev[i]->early_suspend.suspend = s3cfb_early_suspend;
		fbdev[i]->early_suspend.resume = s3cfb_late_resume;
		fbdev[i]->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;

		register_early_suspend(&fbdev[i]->early_suspend);
#endif
#endif

#if defined(CONFIG_FB_S5P_VSYNC_THREAD)
		init_waitqueue_head(&fbdev[i]->vsync_info.wait);

		/* Create vsync thread */
		mutex_init(&fbdev[i]->vsync_info.irq_lock);

		fbdev[i]->vsync_info.thread = kthread_run(
						s3cfb_wait_for_vsync_thread,
						fbdev[i], "s3c-fb-vsync");
		if (fbdev[i]->vsync_info.thread == ERR_PTR(-ENOMEM)) {
			dev_err(fbdev[i]->dev, "failed to run vsync thread\n");
			fbdev[i]->vsync_info.thread = NULL;
		}
		fbdev[i]->vsync_enable = 1;
#endif
	}
#ifdef CONFIG_FB_S5P_LCD_INIT
	/* panel control */
	if (pdata->backlight_on)
		pdata->backlight_on(pdev);

	if (pdata->lcd_on)
		pdata->lcd_on(pdev);
#endif
#ifdef CONFIG_FB_DYNAMIC_FREQ
	s3cfb_dvfs_init(fbfimd->fbdev[0]);
#endif
#if defined(CONFIG_FB_S5P_VSYNC_THREAD)
	ret = device_create_file(&(pdev->dev), &dev_attr_vsync);
	if (ret < 0) 
		dev_err(fbdev[0]->dev, "failed to create vsync file\n");
	ret = device_create_file(&(pdev->dev), &dev_attr_vsync_debug);
	if (ret < 0)
		dev_err(fbdev[0]->dev, "failed to add sysfs entries\n");
	ret = device_create_file(&(pdev->dev), &dev_attr_vsync_enable);
	if (ret < 0)
		dev_err(fbdev[0]->dev, "failed to add sysfs entries\n");
#endif
	ret = device_create_file(&(pdev->dev), &dev_attr_win_power);
	if (ret < 0)
		dev_err(fbdev[0]->dev, "failed to add sysfs entries\n");

	dev_info(fbdev[0]->dev, "registered successfully\n");
	return 0;

err3:
	for (i = 0; i < FIMD_MAX; i++)
		free_irq(fbdev[i]->irq, fbdev[i]);
err2:
	for (i = 0; i < FIMD_MAX; i++)
		iounmap(fbdev[i]->regs);
err1:
	for (i = 0; i < FIMD_MAX; i++)
		pdata->clk_off(pdev, &fbdev[i]->clock);
err0:
	return ret;
}

static int s3cfb_remove(struct platform_device *pdev)
{
	struct s3c_platform_fb *pdata = to_fb_plat(&pdev->dev);
	struct s3cfb_window *win;
	struct fb_info *fb;
	struct s3cfb_global *fbdev[2];
	int i;
	int j;
#if defined(CONFIG_FB_S5P_VSYNC_THREAD)
	device_remove_file(&(pdev->dev), &dev_attr_vsync_enable);
	device_remove_file(&(pdev->dev), &dev_attr_vsync_debug);
	device_remove_file(&(pdev->dev), &dev_attr_vsync);
#endif
#ifdef CONFIG_FB_DYNAMIC_FREQ
	s3cfb_dvfs_remove(fbfimd->fbdev[0]);
#endif
	for (i = 0; i < FIMD_MAX; i++) {
		fbdev[i] = fbfimd->fbdev[i];

#ifdef CONFIG_HAS_WAKELOCK
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&fbdev[i]->early_suspend);
#endif
#endif
		free_irq(fbdev[i]->irq, fbdev[i]);
		iounmap(fbdev[i]->regs);
		pdata->clk_off(pdev, &fbdev[i]->clock);

		for (j = 0; j < pdata->nr_wins; j++) {
			fb = fbdev[i]->fb[j];

			/* free if exists */
			if (fb) {
				win = fb->par;
				if (win->id == pdata->default_win)
					s3cfb_unmap_default_video_memory(fbdev[i], fb);
				else
					s3cfb_unmap_video_memory(fbdev[i], fb);

				s3cfb_set_buffer_address(fbdev[i], j);
				framebuffer_release(fb);
			}
		}
#if defined(CONFIG_FB_S5P_VSYNC_THREAD)
		if (fbdev[i]->vsync_info.thread)
			kthread_stop(fbdev[i]->vsync_info.thread);
#endif
		kfree(fbdev[i]->fb);
		kfree(fbdev[i]);
	}
#ifdef CONFIG_EXYNOS_DEV_PD
	/* disable the power domain */
	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
#endif
	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_FB_S5P_AMS369FG06
extern void ams369fg06_ldi_init(void);
extern void ams369fg06_ldi_enable(void);
extern void ams369fg06_ldi_disable(void);
extern void ams369fg06_gpio_cfg(void);
#elif defined(CONFIG_FB_S5P_LMS501KF03)
extern void lms501kf03_ldi_disable(void);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
void s3cfb_early_suspend(struct early_suspend *h)
{
	struct s3cfb_global *info = container_of(h, struct s3cfb_global, early_suspend);
	struct s3c_platform_fb *pdata = to_fb_plat(info->dev);
	struct platform_device *pdev = to_platform_device(info->dev);
	struct s3cfb_global *fbdev[2];
	int i;

	info->system_state = POWER_OFF;

	for (i = 0; i < FIMD_MAX; i++) {
		fbdev[i] = fbfimd->fbdev[i];

		if (pdata->backlight_off)
			pdata->backlight_off(pdev);

#ifdef CONFIG_FB_S5P_AMS369FG06
		ams369fg06_ldi_disable();
#elif defined(CONFIG_FB_S5P_LMS501KF03)
		lms501kf03_ldi_disable();
#endif

		s3cfb_display_off(fbdev[i]);
		if (pdata->clk_off)
			pdata->clk_off(pdev, &fbdev[i]->clock);
	}
#ifdef CONFIG_EXYNOS_DEV_PD
	/* disable the power domain */
	printk(KERN_DEBUG "s3cfb - disable power domain\n");
	pm_runtime_put_sync(&pdev->dev);
#endif
	return;
}

void s3cfb_late_resume(struct early_suspend *h)
{
	struct s3cfb_global *info = container_of(h, struct s3cfb_global, early_suspend);
	struct s3c_platform_fb *pdata = to_fb_plat(info->dev);
	struct fb_info *fb;
	struct s3cfb_window *win;
	struct s3cfb_global *fbdev[2];
	int i, j;
	struct platform_device *pdev = to_platform_device(info->dev);


	dev_dbg(info->dev, "wake up from suspend\n");

#ifdef CONFIG_EXYNOS_DEV_PD
	/* enable the power domain */
	printk(KERN_DEBUG "s3cfb - enable power domain\n");
	pm_runtime_get_sync(&pdev->dev);
#endif

	info->system_state = POWER_ON;

	for (i = 0; i < FIMD_MAX; i++) {
		fbdev[i] = fbfimd->fbdev[i];
		if (pdata->cfg_gpio)
			pdata->cfg_gpio(pdev);

		if (pdata->backlight_on)
			pdata->backlight_on(pdev);

		if (pdata->lcd_on)
			pdata->lcd_on(pdev);

#if defined(CONFIG_FB_S5P_DUMMYLCD)
		max8698_ldo_enable_direct(MAX8698_LDO4);
#endif

		if (info->lcd->init_ldi)
			fbdev[i]->lcd->init_ldi();
		else
			dev_dbg(info->dev, "no init_ldi\n");

		if (pdata->clk_on)
			pdata->clk_on(pdev, &fbdev[i]->clock);

		s3cfb_init_global(fbdev[i]);
		s3cfb_set_clock(fbdev[i]);
		s3cfb_display_on(fbdev[i]);

		for (j = 0; j < pdata->nr_wins; j++) {
			fb = fbdev[i]->fb[j];
			win = fb->par;

			s3cfb_set_par(fb);
			s3cfb_set_buffer_address(fbdev[i], win->id);
			if ((win->path == DATA_PATH_DMA) && (win->enabled)) {
				s3cfb_enable_window(fbdev[i], win->id);
			}
		}

#ifdef CONFIG_FB_S5P_AMS369FG06
		ams369fg06_gpio_cfg();
		ams369fg06_ldi_init();
		ams369fg06_ldi_enable();
#endif

		if (pdata->backlight_on)
			pdata->backlight_on(pdev);
#if defined(CONFIG_FB_S5P_VSYNC_THREAD)
		mutex_lock(&fbdev[i]->vsync_info.irq_lock);
		if (fbdev[i]->vsync_info.irq_refcount) {
			s3cfb_set_global_interrupt(fbdev[i], 1);
			s3cfb_set_vsync_interrupt(fbdev[i], 1);
		}
		mutex_unlock(&fbdev[i]->vsync_info.irq_lock);
#endif
	}

	info->system_state = POWER_ON;

	return;
}
#else /* else !CONFIG_HAS_EARLYSUSPEND */

int s3cfb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct s3c_platform_fb *pdata = to_fb_plat(&pdev->dev);
	struct s3cfb_global *fbdev[2];
	int i;

	for (i = 0; i < FIMD_MAX; i++) {
		fbdev[i] = fbfimd->fbdev[i];

		if (atomic_read(&fbdev[i]->enabled_win) > 0) {
			/* lcd_off and backlight_off isn't needed. */
			if (fbdev[i]->lcd->deinit_ldi)
				fbdev[i]->lcd->deinit_ldi();

			s3cfb_display_off(fbdev[i]);
			pdata->clk_off(pdev, &fbdev[i]->clock);
		}

	}

	return 0;
}

int s3cfb_resume(struct platform_device *pdev)
{
	struct s3c_platform_fb *pdata = to_fb_plat(&pdev->dev);
	struct fb_info *fb;
	struct s3cfb_window *win;
	struct s3cfb_global *fbdev[2];
	int i;
	int j;

	for (i = 0; i < FIMD_MAX; i++) {
		fbdev[i] = fbfimd->fbdev[i];
		dev_dbg(fbdev[i]->dev, "wake up from suspend fimd[%d]\n", i);

		if (pdata->cfg_gpio)
			pdata->cfg_gpio(pdev);

		if (pdata->backlight_on)
			pdata->backlight_on(pdev);
		if (pdata->lcd_on)
			pdata->lcd_on(pdev);
		if (fbdev[i]->lcd->init_ldi)
			fbdev[i]->lcd->init_ldi();

		if (pdata->backlight_off)
			pdata->backlight_off(pdev);
		if (pdata->lcd_off)
			pdata->lcd_off(pdev);
		if (fbdev[i]->lcd->deinit_ldi)
			fbdev[i]->lcd->deinit_ldi();

		if (atomic_read(&fbdev[i]->enabled_win) > 0) {
			pdata->clk_on(pdev, &fbdev[i]->clock);
			s3cfb_init_global(fbdev[i]);
			s3cfb_set_clock(fbdev[i]);

			for (j = 0; j < pdata->nr_wins; j++) {
				fb = fbdev[i]->fb[j];
				win = fb->par;
				if (win->owner == DMA_MEM_FIMD) {
					s3cfb_set_win_params(fbdev[i], win->id);
					if (win->enabled) {
						if (win->power_state == FB_BLANK_NORMAL)
							s3cfb_win_map_on(fbdev[i], win->id, 0x0);

						s3cfb_enable_window(fbdev[i], win->id);
					}
				}
			}

			s3cfb_display_on(fbdev[i]);

			if (pdata->backlight_on)
				pdata->backlight_on(pdev);

			if (pdata->lcd_on)
				pdata->lcd_on(pdev);

			if (fbdev[i]->lcd->init_ldi)
				fbdev[i]->lcd->init_ldi();
		}
	}

	return 0;
}
#endif
#else
#define s3cfb_suspend NULL
#define s3cfb_resume NULL
#endif

#ifdef CONFIG_EXYNOS_DEV_PD
static int s3cfb_runtime_suspend(struct device *dev)
{
	return 0;
}

static int s3cfb_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops s3cfb_pm_ops = {
	.runtime_suspend = s3cfb_runtime_suspend,
	.runtime_resume = s3cfb_runtime_resume,
};
#endif

static struct platform_driver s3cfb_driver = {
	.probe		= s3cfb_probe,
	.remove		= s3cfb_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= s3cfb_suspend,
	.resume		= s3cfb_resume,
#endif
	.driver		= {
		.name	= S3CFB_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_EXYNOS_DEV_PD
		.pm	= &s3cfb_pm_ops,
#endif
	},
};

struct fb_ops s3cfb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= s3cfb_open,
	.fb_release	= s3cfb_release,
	.fb_check_var	= s3cfb_check_var,
	.fb_set_par	= s3cfb_set_par,
	.fb_setcolreg	= s3cfb_setcolreg,
	.fb_blank	= s3cfb_blank,
	.fb_pan_display	= s3cfb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_cursor	= s3cfb_cursor,
	.fb_ioctl	= s3cfb_ioctl,
};

static int s3cfb_register(void)
{
	return platform_driver_register(&s3cfb_driver);
}

static void s3cfb_unregister(void)
{
	platform_driver_unregister(&s3cfb_driver);
}

arch_initcall_sync(s3cfb_register);
module_exit(s3cfb_unregister);

MODULE_AUTHOR("Jingoo Han <jg1.han@samsung.com>");
MODULE_AUTHOR("Jonghun, Han <jonghun.han@samsung.com>");
MODULE_AUTHOR("Jinsung, Yang <jsgood.yang@samsung.com>");
MODULE_DESCRIPTION("Samsung Display Controller (FIMD) driver");
MODULE_LICENSE("GPL");
