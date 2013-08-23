/*
 * mx_tvout.c - tyout driver helper for mx2 board
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author:  lvcha qiu   <lvcha@meizu.com>
 *
 * This program is not provided / owned by Maxim Integrated Products.
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
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/notifier.h>
#include <linux/mhl.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/gpio.h>

#include <mach/irqs.h>
#include <mach/gpio-m040.h>
#include <mach/i2c-m040.h>

#include <plat/gpio-cfg.h>
#include <plat/pd.h>
#include <plat/devs.h>
#include <plat/tvout.h>

#ifdef CONFIG_MHL_DRIVER
static int mx2_mhl_power_on(struct mhl_platform_data *pdata, int enable)
{
	struct regulator_bulk_data supplies[3] = {
		{"vdd_ldo26", },
		{"vdd_ldo20", },
		{"MHL_1.2V",},
	};
	int num_consumers = ARRAY_SIZE(supplies);
	int ret;

	ret = regulator_bulk_get(NULL, num_consumers, supplies);
	if (ret) {
		MHLPRINTK("regulator_bulk_get failed!\n");
		return ret;
	}

	if (enable) {
		ret = regulator_bulk_enable(num_consumers, supplies);
	} else {
		ret = regulator_bulk_disable(num_consumers, supplies);
	}

	if (ret < 0)
		MHLPRINTK("regulator_%sable failed\n", enable ? "en" : "dis");
	else 
		MHLPRINTK("regulator_%sable\n", enable ? "en" : "dis");

	regulator_bulk_free(num_consumers, supplies);
	return ret;
}

static int mx2_mhl_reset(struct mhl_platform_data *pdata)
{
	int err;

	/* mhl wake */
	
	err = gpio_request(pdata->mhl_wake_pin, NULL);
	if (err) {
		MHLPRINTK("gpio_request failed\n");
		return -1;
	}
	s3c_gpio_cfgpin(pdata->mhl_wake_pin, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(pdata->mhl_wake_pin, S3C_GPIO_PULL_NONE);
	gpio_direction_output(pdata->mhl_wake_pin, 1);
	gpio_free(pdata->mhl_wake_pin);
	mdelay(10);

	/* mhl reset */
	err = gpio_request(pdata->mhl_reset_pin, NULL);
	if (err) {
		MHLPRINTK("gpio_request failed\n");
		return -1;
	}
	s3c_gpio_cfgpin(pdata->mhl_reset_pin, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(pdata->mhl_reset_pin, S3C_GPIO_PULL_NONE);
	gpio_direction_output(pdata->mhl_reset_pin, 0);
	mdelay(5);
	gpio_direction_output(pdata->mhl_reset_pin, 1);
	gpio_free(pdata->mhl_reset_pin);

	return 0;
}

static struct mhl_platform_data mx2_mhl_platdata = {
	.mhl_wake_pin = M040_MHL_WAKE,
	.mhl_reset_pin = M040_MHL_RST,
	.mhl_irq_pin  = M040_MHL_IRQ,
	.eint  = IRQ_EINT(23),
	.mhl_usb_irq_pin  = M040_MHL_USB_IRQ,
	.hw_onoff = mx2_mhl_power_on,
	.mhl_power_on = mx2_mhl_power_on,
	.hw_reset = mx2_mhl_reset,
};
/*
static struct sii9234_platform_data sii9234_pdata = {
	.init = sii9234_cfg_gpio,
#if defined(CONFIG_SAMSUNG_USE_11PIN_CONNECTOR) || \
		defined(CONFIG_MACH_P4NOTE)
	.mhl_sel = NULL,
#else
	.mhl_sel = mhl_usb_switch_control,
#endif
	.hw_onoff = sii9234_power_onoff,
	.hw_reset = sii9234_reset,
	.enable_vbus = NULL,
#if defined(__MHL_NEW_CBUS_MSC_CMD__)
	.vbus_present = NULL,
#else
	.vbus_present = NULL,
#endif

#ifdef CONFIG_EXTCON
	.extcon_name = "max77693-muic",
#endif
};
*/
#endif
static struct i2c_board_info i2c_devs8[] = {
#ifdef CONFIG_MHL_DRIVER	
	{I2C_BOARD_INFO("sii9244_tpi",  (0x7E >> 1)),
	.platform_data = &mx2_mhl_platdata,},
	{I2C_BOARD_INFO("sii9244_hdmi_rx", (0x96 >> 1)),
	.platform_data = &mx2_mhl_platdata,},
	{I2C_BOARD_INFO("sii9244_cbus",    (0xCC >> 1)),
	.platform_data = &mx2_mhl_platdata,},
	{I2C_BOARD_INFO("sii9244_mhl_tx",  (0x76 >> 1)),
	.platform_data = &mx2_mhl_platdata,},
#endif
};


#if defined(CONFIG_VIDEO_TVOUT)
static int mx2_tvout_enable_power( int on)
{
	struct regulator_bulk_data supplies[2] = {
		{"HDMI_1.0V", },
		{"vdd_ldo19", },
	};
	int num_consumers = ARRAY_SIZE(supplies);
	int ret = 0;

	pr_info("%s:[%s]\n", __func__, on?"ON":"OFF");

	ret = regulator_bulk_get(NULL, num_consumers, supplies);
	if (ret) {
		pr_err("%s, regulator_bulk_get failed!\n", __func__);
		return ret;
	}

	if (on)
		ret = regulator_bulk_enable(num_consumers, supplies);
	else
		ret = regulator_bulk_disable(num_consumers, supplies);

	if (ret != 0) {
		pr_err("regulator_%sable failed.\n", on ? "en" : "dis");
	}

	regulator_bulk_free(num_consumers, supplies);

	return ret;
}
static struct s5p_platform_tvout __initdata mx2_tvout_data={
	.enable_power = mx2_tvout_enable_power,
};
static struct s5p_platform_hpd hdmi_hpd_data __initdata = {

};
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif
static struct i2c_board_info __initdata i2c_devs13[] = {
#ifdef CONFIG_VIDEO_TVOUT
	{
		I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),
	},
#endif
};
static struct platform_device __initdata *m040_tvout_devices[]  = {
	&m040_device_gpio_i2c8,
	&m040_device_gpio_i2c13,
#ifdef CONFIG_VIDEO_TVOUT
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif
};
static int  __init mx2_init_tvout(void)
{
	/* mhl */
#ifdef CONFIG_MHL_DRIVER
	i2c_register_board_info(8, i2c_devs8, ARRAY_SIZE(i2c_devs8));
#endif

#if defined(CONFIG_VIDEO_TVOUT)
	/* hdmi ddc */
	i2c_register_board_info(13, i2c_devs13, ARRAY_SIZE(i2c_devs13));

	s5p_device_tvout.dev.parent = &exynos4_device_pd[PD_TV].dev;
	s5p_tvout_set_platdata(&mx2_tvout_data);
	s5p_hdmi_hpd_set_platdata(&hdmi_hpd_data);
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
#endif
	if(platform_add_devices(m040_tvout_devices, ARRAY_SIZE(m040_tvout_devices))){
		pr_err("%s: register tvout device fail\n", __func__);
	}
	return 0;
}

arch_initcall(mx2_init_tvout);

MODULE_DESCRIPTION("mx2 tvout driver helper");
MODULE_AUTHOR("lvcha qiu <lvcha@meizu.com>");
MODULE_LICENSE("GPLV2");

