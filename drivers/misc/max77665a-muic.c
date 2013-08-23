/*
 * max77665a-muic.c - MUIC driver for the Maxim 77665
 *
 *  Copyright (C) 2012 Meizu Technology Co.Ltd
 *  <jgmai@meizu.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/mfd/max77665.h>
#include <linux/mfd/max77665-private.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <mach/gpio-m040.h>

#define DEV_NAME	"max77665-muic"

enum {
	ADC_GND			= 0x00,
	ADC_REMOVE		= 0x10,
	ADC_OPEN		= 0x1f
};

enum {
	ADC_TYPE_NONE	= 0,
	ADC_TYPE_MHL,
	ADC_TYPE_DOCK,
	ADC_TYPE_OTG,
	ADC_TYPE_DISCONNECT,
};

#define USB_DOCK_INSERT_LEVEL 0

struct max77665_muic_info {
	struct device		*dev;
	struct max77665_dev	*max77665;
	struct i2c_client	*muic;

	int adc_type;
	int suspended;

	int mhl_insert;
	int host_insert;
	struct regulator *reverse;
	struct delayed_work dwork;

	int usb_select_gpio;
	int dock_irq_gpio;
	int dock_output_gpio;
};

static struct max77665_muic_info *g_info;

static int init_max77665_muic(struct max77665_muic_info *info)
{
	int ret;
	struct i2c_client *client = info->muic;
	u8 val, msk;

	val = (0x1 << COMN1SW_SHIFT) | (0x1 << COMP2SW_SHIFT) |
		(0 << MICEN_SHIFT) | (0 << IDBEN_SHIFT);

	msk = COMN1SW_MASK | COMP2SW_MASK | MICEN_MASK | IDBEN_MASK;

	ret = max77665_update_reg(client, MAX77665_MUIC_REG_CTRL1, val, msk);
	if (ret) {
		pr_err("[MUIC]: can not update reg_ctrl1\n");
		return ret;
	}

	val = msk = 0;
	val = (0 << ADCEN_SHIFT) | (0 << LOWPWR_SHIFT);
	msk = ADCEN_MASK | LOWPWR_SHIFT;
	ret = max77665_update_reg(client, MAX77665_MUIC_REG_CTRL2, val, msk);
	if (ret) {
		pr_err("[MUIC]: can not disable adc\n");
		return ret;
	}

	val = msk = 0;
	val = (1 << ADCEN_SHIFT) | (1 << LOWPWR_SHIFT);
	msk = ADCEN_MASK | LOWPWR_SHIFT;
	ret = max77665_update_reg(client, MAX77665_MUIC_REG_CTRL2, val, msk);
	if (ret) {
		pr_err("[MUIC]: can not enable adc\n");
		return ret;
	}

	val = msk = 0;
	val = (0x2 << ADCDBSET_SHIFT); /*set adc debounce time to 25ms*/
	msk = ADCDBSET_MASK;
	ret = max77665_update_reg(client, MAX77665_MUIC_REG_CTRL3, val, msk);
	if (ret) {
		pr_err("[MUIC]: can not set adc debounce time\n");
		return ret;
	}

	return ret;
}

inline static void echi_pm_runtime(int onoff)
{
	pr_info("@@@ %s %d\n", __func__, onoff);
	if(onoff)
		pm_runtime_put_sync(&s5p_device_ehci.dev);
	else
		pm_runtime_get_sync(&s5p_device_ehci.dev);
}
inline static int max77665a_is_dock_detect(struct max77665_muic_info *info)
{
	return (gpio_get_value(info->dock_irq_gpio) == USB_DOCK_INSERT_LEVEL)?1:0;
}

int mx_is_usb_dock_insert(void)
{
	struct max77665_muic_info *info = g_info;
	
	if(info){
		return (max77665a_is_dock_detect(info) || info->mhl_insert);
	}
	return 0;
}

inline static void max77665a_select_usb_dock(struct max77665_muic_info *info)
{
	gpio_set_value(info->usb_select_gpio, 1);
}

inline static void max77665a_select_usb_device(struct max77665_muic_info *info)
{
	gpio_set_value(info->usb_select_gpio, 0);
}

inline static int max77665a_set_usbid(struct max77665_muic_info *info, int value)
{
	int ret;
	struct i2c_client *client = info->muic;
	u8 val, msk;

	val = (0x1 << COMN1SW_SHIFT) | (0x1 << COMP2SW_SHIFT) |
		(0 << MICEN_SHIFT) | (!!value << IDBEN_SHIFT);

	msk = COMN1SW_MASK | COMP2SW_MASK | MICEN_MASK | IDBEN_MASK;

	ret = max77665_update_reg(client, MAX77665_MUIC_REG_CTRL1, val, msk);

	return 0;
}
static int max77665_muic_get_type(struct max77665_muic_info *info)
{
	u8 adc, adclow ,adcerr, adc1k, dock;
	u8 status1;
	int type =ADC_TYPE_NONE;
	
	max77665_read_reg(info->muic, MAX77665_MUIC_REG_STATUS1, &status1);
	adc = status1 & STATUS1_ADC_MASK;
	adclow = !!(status1 & STATUS1_ADCLOW_MASK);
	adcerr = !!(status1 & STATUS1_ADCERR_MASK);
	adc1k = !!(status1 & STATUS1_ADC1K_MASK);
	dock = max77665a_is_dock_detect(info);
	
	pr_info("adc 0x%02x, adclow %d, adcerr %d, adc1k %d, dock=%d\n",
			adc, adclow, adcerr, adc1k, dock);
	
	if(adc1k) {
		type = ADC_TYPE_MHL;
	} else if (dock) {
		type= ADC_TYPE_DOCK;
	} else if (adc == ADC_GND) {
		type = ADC_TYPE_OTG;
	} else if ((adc == ADC_OPEN) && !dock) {
		type = ADC_TYPE_DISCONNECT;
	}
	return type;
}
static void max77665_muic_enable_host(struct max77665_muic_info *info)
{
	if(!info->host_insert) {
		pr_info("otg connect\n");
		switch(info->adc_type){
		case ADC_TYPE_MHL:
		case ADC_TYPE_DOCK:
			echi_pm_runtime(false);
			max77665a_select_usb_dock(info);
			break;
		case ADC_TYPE_OTG:
			echi_pm_runtime(false);
			if(!regulator_is_enabled(info->reverse))
					regulator_enable(info->reverse);
			break;
		}
		info->host_insert = true;
	}
}

static void max77665_muic_disable_host(struct max77665_muic_info *info)
{
	if (info->host_insert) {
		pr_info("otg disconnect\n");
		max77665a_select_usb_device(info);

		echi_pm_runtime(true);

		if(regulator_is_enabled(info->reverse))
			regulator_disable(info->reverse);
		info->host_insert = false;
	}
}

static irqreturn_t max77665_muic_isr(int irq, void *dev_id)
{
	struct max77665_muic_info *info = dev_id;

	info->adc_type = max77665_muic_get_type(info);

	switch(info->adc_type){
	case ADC_TYPE_MHL:
		info->mhl_insert = true;
		max77665_muic_enable_host(info);
		schedule_delayed_work(&info->dwork, 0);
		break;
	case ADC_TYPE_DOCK:
	case ADC_TYPE_OTG:
		max77665_muic_enable_host(info);
		break;
	case ADC_TYPE_DISCONNECT:
		max77665_muic_disable_host(info);
		break;
	}
	return IRQ_HANDLED;
}

void check_mhl_connect(void)
{
	struct max77665_muic_info *info = g_info;
	u8  adc1k;
	u8 status1;

	max77665_read_reg(info->muic, MAX77665_MUIC_REG_STATUS1, &status1);
	adc1k = !!(status1 & STATUS1_ADC1K_MASK);

	if(!adc1k) {
		pr_info("adc1k not set\n");
		info->mhl_insert = false;
	} else {
		pr_info("adc1k is set\n");
		info->mhl_insert = true;
	}
	if (delayed_work_pending(&info->dwork))
		cancel_delayed_work(&info->dwork);
	schedule_delayed_work(&info->dwork, 0);
}

#ifdef CONFIG_MHL_DRIVER
extern void mhl_connect(int on);
#else
inline static void mhl_connect(int on){}
#endif

static void muic_mhl_work(struct work_struct *work)
{
	struct max77665_muic_info *info =
		container_of(work, struct max77665_muic_info, dwork.work);
	if (info->mhl_insert) {
		pr_info("mhl connect\n");

		max77665a_set_usbid(info, true);
		mhl_connect(true);
	} else {
		pr_info("mhl disconnect\n");

		mhl_connect(false);
		max77665a_set_usbid(info, false);
	}
}

static ssize_t debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max77665_muic_info *info = g_info;
	unsigned long debug = simple_strtoul(buf, NULL, 10);
	if(debug == 0) {
		gpio_set_value(M040_USB_SELECT, 0);
	} else {
		int ret;
		u8 val, msk;

		val = (0x1 << COMN1SW_SHIFT) | (0x1 << COMP2SW_SHIFT);

		msk = COMN1SW_MASK | COMP2SW_MASK;

		ret = max77665_update_reg(info->muic, MAX77665_MUIC_REG_CTRL1, val, msk);
	}
	return count;
}

static ssize_t debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77665_muic_info *info = g_info;
	int usb_gpio = gpio_get_value(M040_USB_SELECT);
	int usb_dm, usb_dp;
	u8 val;

	max77665_read_reg(info->muic, MAX77665_MUIC_REG_CTRL1, &val);
	usb_dm = !!(val & COMN1SW_MASK);
	usb_dp = !!(val & COMP2SW_MASK);

	return sprintf(buf, "usb_gpio %d usb_dm %d usb_dp %d\n", 
			usb_gpio, usb_dm, usb_dp);
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR|S_IWGRP,
		   				debug_show, debug_store);

static int __devinit max77665_muic_probe(struct platform_device *pdev)
{
	struct max77665_dev *max77665 = dev_get_drvdata(pdev->dev.parent);
	struct max77665_platform_data *pdata = dev_get_platdata(max77665->dev);
	struct max77665_muic_platform_data *muic_pdata = pdata->muic_pdata;
	struct max77665_muic_info *info;
	int ret = 0;
	int irq;

	pr_info("func:%s\n", __func__);

	info = kzalloc(sizeof(struct max77665_muic_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "%s: failed to allocate info\n", __func__);
		ret = -ENOMEM;
		goto err_return;
	}

	g_info = info;

	info->dev = &pdev->dev;
	info->max77665 = max77665;
	info->muic = max77665->muic;

	info->usb_select_gpio = muic_pdata->usb_select_gpio;
	info->dock_irq_gpio= muic_pdata->dock_irq_gpio;
	info->dock_output_gpio= muic_pdata->dock_output_gpio;

	info->reverse = regulator_get(NULL, "reverse");
	if (IS_ERR(info->reverse)) {
		dev_err(&pdev->dev, "Failed to get reverse regulator\n");
		goto fail0;
	}

	platform_set_drvdata(pdev, info);

	ret = init_max77665_muic(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to initialize MUIC:%d\n", ret);
		goto fail1;
	}
	INIT_DELAYED_WORK(&info->dwork, muic_mhl_work);

	irq = gpio_to_irq(info->dock_irq_gpio);
	ret = request_threaded_irq(irq, 0, max77665_muic_isr,
			IRQF_TRIGGER_RISING |IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "max77665_dock", info);

	irq = max77665->irq_base + MAX77665_MUIC_IRQ_INT1_ADC1K;
	ret = request_threaded_irq(irq, 0, max77665_muic_isr,
			0, "max77665_adc1k", info);

	irq = max77665->irq_base + MAX77665_MUIC_IRQ_INT1_ADCERR;
	ret = request_threaded_irq(irq, 0, max77665_muic_isr,
			0, "max77665_adcerr", info);

	irq = max77665->irq_base + MAX77665_MUIC_IRQ_INT1_ADCLOW;
	ret = request_threaded_irq(irq, 0, max77665_muic_isr,
			0, "max77665_adclow", info);

	irq = max77665->irq_base + MAX77665_MUIC_IRQ_INT1_ADC;
	ret = request_threaded_irq(irq, 0, max77665_muic_isr,
			0, "max77665_adc", info);

	/* create sysfs attributes */
	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_debug.attr);
	if (ret < 0) {
		pr_debug("sysfs_create_group failed\n");
	}

	return 0;

fail1:
	regulator_put(info->reverse);
fail0:
	platform_set_drvdata(pdev, NULL);
	kfree(info);
err_return:
	return ret;
}

static int __devexit max77665_muic_remove(struct platform_device *pdev)
{
	struct max77665_muic_info *info = platform_get_drvdata(pdev);

	if (info) {
		dev_info(info->dev, "func:%s\n", __func__);
		platform_set_drvdata(pdev, NULL);
		kfree(info);
	}
	return 0;
}

void max77665_muic_shutdown(struct device *dev)
{
	return ;
}

#ifdef CONFIG_PM
static int max77665_muic_suspend(struct device *dev)
{
	struct max77665_muic_info *info = dev_get_drvdata(dev);
	/*usb d+/d- switch to usb dock host*/
	max77665a_select_usb_dock(info);
	info->suspended = true;
	return 0;
}

static int max77665_muic_resume(struct device *dev)
{
	struct max77665_muic_info *info = dev_get_drvdata(dev);
	/*usb d+/d- switch to usb device*/
	max77665a_select_usb_device(info);
	info->suspended = false;
	return 0;
}

static const struct dev_pm_ops max77665_pm_ops = {
	.suspend        = max77665_muic_suspend,
	.resume		= max77665_muic_resume,
};
#endif

static struct platform_driver max77665_muic_driver = {
	.driver		= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
		.shutdown = max77665_muic_shutdown,
#ifdef CONFIG_PM
		.pm       =  &max77665_pm_ops,
#endif
	},
	.probe		= max77665_muic_probe,
	.remove		= __devexit_p(max77665_muic_remove),
};

static int __init max77665_muic_init(void)
{
	pr_info("func:%s\n", __func__);
	return platform_driver_register(&max77665_muic_driver);
}
module_init(max77665_muic_init);

static void __exit max77665_muic_exit(void)
{
	pr_info("func:%s\n", __func__);
	platform_driver_unregister(&max77665_muic_driver);
}
module_exit(max77665_muic_exit);

MODULE_DESCRIPTION("Maxim MAX77665 MUIC driver");
MODULE_AUTHOR("<sukdong.kim@samsung.com>");
MODULE_LICENSE("GPL");
