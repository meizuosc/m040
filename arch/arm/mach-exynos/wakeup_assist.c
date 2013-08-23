/*
 * Wakeup assist driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/err.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/regs-pmu.h>
#include <plat/pm.h>

#define DEV_NAME "wakeup_assist"

static int wakeup_assist_keycode[] = {
 	KEY_POWER,
 	KEY_HOME,
	KEY_VOLUMEDOWN,
	KEY_VOLUMEUP,
 	KEY_WAKEUP,
};

#if defined(CONFIG_MX_SERIAL_TYPE) || defined(CONFIG_MX2_SERIAL_TYPE)

#define WAKE_STR_LEN 256
static unsigned long mx_wakeup_type;
static u64 total_wakeup;

struct wakeup_assist_des assist_des[] = {
	{MX_UNKNOW_WAKE,		KEY_RESERVED,		"MX_UNKNOW_WAKE"},
	{MX_USB_WAKE,			KEY_RESERVED,		"MX_USB_WAKE"},
	{MX_LOWBAT_WAKE,		KEY_WAKEUP,			"MX_LOWBAT_WAKE"},
	{MX_KEY_POWER_WAKE,		KEY_POWER,			"MX_KEY_POWER_WAKE"},
	{MX_KEY_HOME_WAKE,		KEY_HOME,			"MX_KEY_HOME_WAKE"},
	{MX_MODEM_WAKE,			KEY_RESERVED,		"MX_MODEM_WAKE"},
	{MX_BLUETOOTH_WAKE,		KEY_RESERVED,		"MX_BLUETOOTH_WAKE"},
	{MX_MODEM_RST_WAKE,		KEY_RESERVED,		"MX_MODEM_RST_WAKE"},
	{MX_CHARG_WAKE,			KEY_RESERVED,		"MX_CHARG_WAKE"},
	{MX_ALARM_WAKE,			KEY_RESERVED,		"MX_ALARM_WAKE"},
	{MX_TICK_WAKE,			KEY_RESERVED,		"MX_TICK_WAKE"},
	{MX_I2S_WAKE,			KEY_RESERVED,		"MX_I2S_WAKE"},
	{MX_SYSTIMER_WAKE,		KEY_RESERVED,		"MX_SYSTIMER_WAKE"},
	{MX_WIFI_WAKE,			KEY_RESERVED,		"MX_WIFI_WAKE"},
	{MX_IR_WAKE,			KEY_RESERVED,		"MX_IR_WAKE"},
	{MX_USB_HOST_WAKE,		KEY_RESERVED,		"MX_USB_HOST_WAKE"},
	{MX_MINUS_KEY_WAKE,		KEY_VOLUMEDOWN,		"MX_MINUS_KEY_WAKE"},
	{MX_PLUS_KEY_WAKE,		KEY_VOLUMEUP,		"MX_PLUS_KEY_WAKE"},
	{MX_MIC_WAKE,			KEY_RESERVED,		"MX_MIC_WAKE"},
};

static unsigned long mx_get_wakeup_type(void)
{
	return mx_wakeup_type;
}

unsigned long mx_set_wakeup_type(unsigned long mask)
{
	int i = 0, index = 0;
	unsigned long tmp = mask;
	int size = ARRAY_SIZE(assist_des);
	
	for(i = 1; i < size; i++) {
		if ((tmp >> (i-1)) & 0x1) {
			index = i;
			break;
		}
	}	
	
	assist_des[index].wakeup_count++;
	
	return mask ? (mx_wakeup_type |= mask) :
				  (mx_wakeup_type = mask);
}

static ssize_t wakeup_stats_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int i = 0, len = 0;
	int size = ARRAY_SIZE(assist_des);
	
	len = sprintf(buf, "Total wakeup count is: %llu\n", total_wakeup);
	len += sprintf(buf + len, "Wakeup sources\t\tCount\n");
	len += sprintf(buf + len, "------------------------------\n");

	for (i = 0; i < size; i++)
		len += sprintf(buf + len, "%-20s:\t%llu\n",
			assist_des[i].name, assist_des[i].wakeup_count);

	return len;
}

static DEVICE_ATTR(wakeup_stats, 0444, wakeup_stats_show, NULL);

static void mx_show_wakeup_name(void)
{
	int i = 0, len;
	char wakeup_str[WAKE_STR_LEN];
	int size = ARRAY_SIZE(assist_des);
	unsigned long type = mx_get_wakeup_type();

	total_wakeup++;
		
	memset(wakeup_str, 0, WAKE_STR_LEN);
	len = sprintf(wakeup_str, "%s", "System wake up by ---> ");

	if (!type) {
		len += sprintf(wakeup_str + len, "[%s] ",
					assist_des[0].name);
		goto out;
	}

	for (i = 1; i < size; i++) {
		if (type & assist_des[i].type)
			len += sprintf(wakeup_str + len, "[%s] ", 
						assist_des[i].name); 
	}
out:	
	pr_info("%s\n", wakeup_str);
}

static inline int wakeup_assist_done(struct input_dev *input_dev)
{
	unsigned long mx_wake_typed = mx_get_wakeup_type();
	int i, size = ARRAY_SIZE(assist_des);

	for (i = 0; i < size; i++){
		if(mx_wake_typed & assist_des[i].type && assist_des[i].keycode != KEY_RESERVED)
		{
			input_report_key(input_dev, assist_des[i].keycode, 1);
			input_sync(input_dev);
			input_report_key(input_dev, assist_des[i].keycode, 0);
			input_sync(input_dev);
			break;
		}
	}

	mx_show_wakeup_name();
	mx_set_wakeup_type(0);
	return 0;
}
#else
static inline int wakeup_assist_done(struct input_dev *input_dev)
{
	if (readl(S5P_WAKEUP_STAT) & 0x1) {
		input_report_key(input_dev, wakeup_assist_keycode[0], 1);
		input_sync(input_dev);
		input_report_key(input_dev, wakeup_assist_keycode[0], 0);
		input_sync(input_dev);
	}
	return 0;
}
#endif

static int __devinit wakeup_assist_probe(struct platform_device *pdev)
{
	int error;
	struct input_dev *input_dev;

	input_dev = input_allocate_device();

	if (!input_dev)
		return -ENOMEM;

	input_dev->name = DEV_NAME;
	input_dev->id.bustype = BUS_HOST;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY);

	input_set_capability(input_dev, EV_MSC, MSC_SCAN);

	input_dev->keycode = wakeup_assist_keycode;
	input_dev->keycodesize = sizeof(wakeup_assist_keycode[0]);
	input_dev->keycodemax = ARRAY_SIZE(wakeup_assist_keycode);

	__set_bit(wakeup_assist_keycode[0], input_dev->keybit);
	__set_bit(wakeup_assist_keycode[1], input_dev->keybit);
	__set_bit(wakeup_assist_keycode[2], input_dev->keybit);
	__set_bit(wakeup_assist_keycode[3], input_dev->keybit);
	__set_bit(wakeup_assist_keycode[4], input_dev->keybit);
	__clear_bit(KEY_RESERVED, input_dev->keybit);

	error = input_register_device(input_dev);
	if (error)  
		goto err_reg;

#if defined(CONFIG_MX_SERIAL_TYPE) || defined(CONFIG_MX2_SERIAL_TYPE)
	error = device_create_file(&pdev->dev, &dev_attr_wakeup_stats);
	if(error)
		goto err_sys;
#endif
	platform_set_drvdata(pdev, input_dev);	
	return 0;

err_sys:
	input_free_device(input_dev);
err_reg:
	return error;
}

static int __devexit wakeup_assist_remove(struct platform_device *pdev)
{
	struct input_dev *input_dev = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	input_unregister_device(input_dev);
	input_free_device(input_dev);
#if defined(CONFIG_MX_SERIAL_TYPE) || defined(CONFIG_MX2_SERIAL_TYPE)
	device_remove_file(&pdev->dev, &dev_attr_wakeup_stats);
#endif

	return 0;
}

static int wakeup_assist_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct input_dev *input_dev = platform_get_drvdata(pdev);

	return wakeup_assist_done(input_dev);
}

static const struct dev_pm_ops wakeup_assist_pm_ops = {
	.resume		= wakeup_assist_resume,
};

static struct platform_driver wakeup_assist_driver = {
	.probe		= wakeup_assist_probe,
	.remove		= __devexit_p(wakeup_assist_remove),
	.driver		= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &wakeup_assist_pm_ops,
	},
};

static int __init wakeup_assist_init(void)
{
	return platform_driver_register(&wakeup_assist_driver);
}
module_init(wakeup_assist_init);

static void __exit wakeup_assist_exit(void)
{
	platform_driver_unregister(&wakeup_assist_driver);
}
module_exit(wakeup_assist_exit);

MODULE_DESCRIPTION("Wakeup assist driver");
MODULE_AUTHOR("Eunki Kim <eunki_kim@samsung.com>");
MODULE_LICENSE("GPL");
