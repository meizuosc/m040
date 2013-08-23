/*
 * charge-detect.c
 *
 * Copyright (c) 2011 WenbinWu	<wenbinwu@meizu.com>
 *
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <linux/sched.h>
#include <linux/regulator/machine.h>
#include <linux/pm_runtime.h>
#include <asm/mach-types.h>
#include <mach/usb-detect.h>
#include <mach/gpio-m032.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>

#define USB_VBUS_INSERT_LEVEL 1
#define USB_HOST_INSERT_LEVEL 0
#define USB_DOCK_INSERT_LEVEL 0

struct usb_detect_info {
	int usb_vbus_gpio;
	int usb_host_gpio;
	int usb_dock_gpio;

	struct delayed_work usb_work;
	struct regulator *reverse;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock usb_detect_lock;
#endif
};

static struct usb_detect_info *g_ud_info = NULL;
static struct blocking_notifier_head usb_notifier_list = BLOCKING_NOTIFIER_INIT(usb_notifier_list);

#ifdef CONFIG_HAS_WAKELOCK
static void usb_detect_wake_lock_initial(struct usb_detect_info *ud_info)
{
	wake_lock_init(&ud_info->usb_detect_lock, WAKE_LOCK_SUSPEND, "usb-detect");
}

static void usb_detect_wake_lock_destroy(struct usb_detect_info *ud_info)
{
	wake_lock_destroy(&ud_info->usb_detect_lock);
}

static void usb_detect_wake_lock_timeout(struct usb_detect_info *ud_info, int timeout)
{
	wake_lock_timeout(&ud_info->usb_detect_lock, timeout);
}

#else
static void usb_detect_wake_lock_initial(struct usb_detect_info *ud_info){}
static void usb_detect_wake_lock_destroy(struct usb_detect_info *ud_info){}
static void usb_detect_wake_lock_timeout(struct usb_detect_info *ud_info , int timeout){}
#endif

#define HOST_SELECT			1
#define DEVICES_SELECT		0
int mx_usb_host_select(int on)
{
	gpio_set_value(USB_SELECT, on);
	pr_info("%s: USB_SELECT=%d\n", __func__, gpio_get_value(USB_SELECT));
	return 0;
}

static irqreturn_t usb_detect_irq_handler(int irq, void *dev_id)
{
	struct usb_detect_info *ud_info = (struct usb_detect_info *)dev_id;

	usb_detect_wake_lock_timeout(ud_info, HZ*3);
	if (!delayed_work_pending(&ud_info->usb_work))
		schedule_delayed_work(&ud_info->usb_work, msecs_to_jiffies(500));
	return IRQ_HANDLED;
}

static void m030_usb_detect_work(struct work_struct *work)
{
	struct usb_detect_info *ud_info = container_of(work, struct usb_detect_info, usb_work.work);
	int val;
	int vbus;
	int usbid;
	static int last_vbus = 0;
	static int last_usbid = 0;

	vbus = gpio_get_value(ud_info->usb_vbus_gpio) == USB_VBUS_INSERT_LEVEL;
	usbid = gpio_get_value(ud_info->usb_host_gpio) == USB_HOST_INSERT_LEVEL;

	pr_info("vbus(%d), usbid(%d)\n", vbus, usbid);
	pr_info("last_vbus(%d), last_usbid(%d)\n", last_vbus, last_usbid);

	if(usbid != last_usbid) {
		last_usbid = usbid;
		val = usbid ? USB_HOST_INSERT : USB_HOST_REMOVE;
		pr_info("=======usb id %d\n", val);
		blocking_notifier_call_chain(&usb_notifier_list, val, NULL);
	}

	if(vbus != last_vbus) {
		last_vbus = vbus;
		val = vbus ? USB_VBUS_INSERT : USB_VBUS_REMOVE;
		blocking_notifier_call_chain(&usb_notifier_list, val, NULL);
	}
}
inline static void echi_pm_runtime(int onoff)
{
	if(onoff)
		pm_runtime_put_sync(&s5p_device_ehci.dev);
	else
		pm_runtime_get_sync(&s5p_device_ehci.dev);
}

static void mx_usb_detect_work(struct work_struct *work)
{
	struct usb_detect_info *ud_info = container_of(work, struct usb_detect_info, usb_work.work);
	int val;
	int vbus;
	int usbid;
	int dock;
	static int last_vbus = 0;
	static int last_usbid = 0;
	static int last_dock = 0;

	vbus = gpio_get_value(ud_info->usb_vbus_gpio) == USB_VBUS_INSERT_LEVEL;
	dock = gpio_get_value(ud_info->usb_dock_gpio) == USB_DOCK_INSERT_LEVEL;

	pr_info("vbus(%d), dock(%d)\n", vbus, dock);
	pr_info("last_vbus(%d), last_dock(%d)\n", last_vbus, last_dock);

	if(dock != last_dock) {
		last_dock = dock;
		val = dock ? USB_DOCK_INSERT : USB_DOCK_REMOVE;
		blocking_notifier_call_chain(&usb_notifier_list, val, NULL);
		if(dock) {
			pr_info("dock insert\n");
			free_irq(gpio_to_irq(ud_info->usb_host_gpio), ud_info);
			mx_usb_host_select(HOST_SELECT);
			s3c_gpio_cfgpin(ud_info->usb_host_gpio, S3C_GPIO_OUTPUT);
			s3c_gpio_setpull(ud_info->usb_host_gpio, S3C_GPIO_PULL_NONE);
			gpio_set_value(ud_info->usb_host_gpio, 0);
			echi_pm_runtime(false);
		} else {
			int error;
			s3c_gpio_setpull(ud_info->usb_host_gpio, S3C_GPIO_PULL_UP);
			error = request_irq(gpio_to_irq(ud_info->usb_host_gpio),
					usb_detect_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					"gpio_usb_host_insert_int", ud_info);
			if(error) {
				pr_info("request usb_host irq error\n");
				last_dock = true;
				schedule_delayed_work(&ud_info->usb_work, msecs_to_jiffies(500));
			} else {
				mx_usb_host_select(DEVICES_SELECT);
				echi_pm_runtime(true);
			}
		}
	}

	if(!dock) {
		usbid = gpio_get_value(ud_info->usb_host_gpio) == USB_HOST_INSERT_LEVEL;
		pr_info("usbid(%d), last_usbid(%d)\n", usbid, last_usbid);
		if(vbus && usbid) {
			pr_info("vbus in, dismiss the usbid in.\n");
		} else {
			if(usbid != last_usbid) {
				last_usbid = usbid;
				val = usbid ? USB_HOST_INSERT : USB_HOST_REMOVE;
				blocking_notifier_call_chain(&usb_notifier_list, val, NULL);
				if(usbid) {
					echi_pm_runtime(false);
					if(!regulator_is_enabled(ud_info->reverse))
						regulator_enable(ud_info->reverse);
				} else{
					if(regulator_is_enabled(ud_info->reverse))
						regulator_disable(ud_info->reverse);
					echi_pm_runtime(true);
				}
			}
		}
	}

	if(vbus != last_vbus) {
		last_vbus = vbus;
		val = vbus ? USB_VBUS_INSERT : USB_VBUS_REMOVE;
		blocking_notifier_call_chain(&usb_notifier_list, val, NULL);
	}
}

static int __devinit usb_detect_probe(struct platform_device *pdev)
{
	struct usb_detect_platform_data *pdata;
	struct device *dev = &pdev->dev;
	int error = -EINVAL;

	pdata = pdev->dev.platform_data;
	if(pdata == NULL) {
		dev_err(dev, "Failed to get platform data\n");
		return -ENOMEM;
	}

	g_ud_info = kzalloc(sizeof(struct usb_detect_info), GFP_KERNEL);
	if (!g_ud_info) {
		dev_err(dev, "Failed to allocate device\n");
		return -ENOMEM;
	}

	g_ud_info->usb_vbus_gpio = pdata->usb_vbus_gpio;
	g_ud_info->usb_host_gpio = pdata->usb_host_gpio;
	g_ud_info->usb_dock_gpio = pdata->usb_dock_gpio;

	usb_detect_wake_lock_initial(g_ud_info);

	if(machine_is_m030()) {
		INIT_DELAYED_WORK(&g_ud_info->usb_work, m030_usb_detect_work);
	} else {
		INIT_DELAYED_WORK(&g_ud_info->usb_work, mx_usb_detect_work);

		g_ud_info->reverse = regulator_get(NULL, "reverse");
		if (IS_ERR(g_ud_info->reverse)) {
			dev_err(dev, "Failed to get reverse regulator\n");
			goto fail0;
		}
	}

	platform_set_drvdata(pdev, g_ud_info);

	error = request_irq(gpio_to_irq(g_ud_info->usb_vbus_gpio),
			usb_detect_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"gpio_usb_vbus_insert_int", g_ud_info);
	if (error) {
		dev_err(dev, "Active Failed to allocate usb_vbus interrupt\n");
		goto fail1;
	}

	error = request_irq(gpio_to_irq(g_ud_info->usb_host_gpio),
			usb_detect_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"gpio_usb_host_insert_int", g_ud_info);
	if (error) {
		dev_err(dev, "Active Failed to allocate usb_host interrupt\n");
		goto fail2;
	}

	if(!machine_is_m030()) {
		error = request_irq(gpio_to_irq(g_ud_info->usb_dock_gpio),
				usb_detect_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"gpio_usb_dock_insert_int", g_ud_info);
		if (error) {
			dev_err(dev, "Active Failed to allocate dock_detect interrupt\n");
			goto fail3;
		}
	}

	schedule_delayed_work(&g_ud_info->usb_work, msecs_to_jiffies(500));

	return 0;

fail3:
	free_irq(gpio_to_irq(g_ud_info->usb_host_gpio), g_ud_info);
fail2:
	free_irq(gpio_to_irq(g_ud_info->usb_vbus_gpio), g_ud_info);
fail1:
	regulator_put(g_ud_info->reverse);
fail0:
	usb_detect_wake_lock_destroy(g_ud_info);
	kfree(g_ud_info);
	return error;
}

static int __devexit usb_detect_remove(struct platform_device *pdev)
{
	struct usb_detect_info *ud_info = platform_get_drvdata(pdev);

	free_irq(gpio_to_irq(ud_info->usb_host_gpio), ud_info);
	free_irq(gpio_to_irq(ud_info->usb_vbus_gpio), ud_info);
	if(!machine_is_m030()) {
		free_irq(gpio_to_irq(ud_info->usb_dock_gpio), ud_info);
		cancel_delayed_work_sync(&ud_info->usb_work);
		if(regulator_is_enabled(ud_info->reverse))
			regulator_disable(ud_info->reverse);
		regulator_put(ud_info->reverse);
	}
	usb_detect_wake_lock_destroy(ud_info);
	platform_set_drvdata(pdev, NULL);
	kfree(ud_info);
	return 0;
}

static void usb_detect_shutdown(struct platform_device *pdev)
{
	struct usb_detect_info *ud_info = platform_get_drvdata(pdev);
	if(!machine_is_m030()) {
		if(regulator_is_enabled(ud_info->reverse))
			regulator_disable(ud_info->reverse);
		regulator_put(ud_info->reverse);
	}
}

int register_mx_usb_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&usb_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(register_mx_usb_notifier);

int unregister_mx_usb_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&usb_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(unregister_mx_usb_notifier);

int mx_is_usb_vbus_insert(void)
{
	if(g_ud_info)
		return (gpio_get_value(g_ud_info->usb_vbus_gpio) == USB_VBUS_INSERT_LEVEL) ? 1 : 0;
	else
		return 0;
}
EXPORT_SYMBOL_GPL(mx_is_usb_vbus_insert);

int mx_is_usb_host_insert(void)
{
	if(g_ud_info)
		return (gpio_get_value(g_ud_info->usb_host_gpio) == USB_HOST_INSERT_LEVEL) ? 1 : 0;
	else
		return 0;
}
EXPORT_SYMBOL_GPL(mx_is_usb_host_insert);

int mx_is_usb_dock_insert(void)
{
	if(g_ud_info)
		return (gpio_get_value(g_ud_info->usb_dock_gpio) == USB_DOCK_INSERT_LEVEL) ? 1 : 0;
	else
		return 0;
}
EXPORT_SYMBOL_GPL(mx_is_usb_dock_insert);

#ifdef CONFIG_PM
static int usb_detect_suspend(struct device *dev)
{
	disable_irq(gpio_to_irq(g_ud_info->usb_host_gpio));
	disable_irq(gpio_to_irq(g_ud_info->usb_vbus_gpio));
	return 0;
}

static int usb_detect_resume(struct device *dev)
{
	enable_irq(gpio_to_irq(g_ud_info->usb_vbus_gpio));
	enable_irq(gpio_to_irq(g_ud_info->usb_host_gpio));
	return 0;
}

static const struct dev_pm_ops usb_detect_pm_ops = {
	.suspend	= usb_detect_suspend,
	.resume		= usb_detect_resume,
};
#endif

static struct platform_driver usb_detect_driver = {
	.probe		= usb_detect_probe,
	.remove		= __devexit_p(usb_detect_remove),
	.shutdown	= usb_detect_shutdown,
	.driver		= {
		.name	= "usb_detect",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &usb_detect_pm_ops,
#endif
	},
};

static int __init usb_detect_init(void)
{
	platform_driver_register(&usb_detect_driver);
	return 0;
}

static void __exit usb_detect_exit(void)
{
	platform_driver_unregister(&usb_detect_driver);
}

late_initcall(usb_detect_init);
module_exit(usb_detect_exit);
