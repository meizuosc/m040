/*
 * linux/arch/arm/mach-exynos/mx2_factory.c
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author: 	lvcha qiu	<lvcha@meizu.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/gpio.h>

#include <asm/mach-types.h>

#include <mach/gpio-common.h>
#include <mach/gpio-m040.h>

static int is_factory_test_mode(int type)
{
	int gpio1, gpio2, gpio3;
	int ret = 0;

 	gpio1 = M040_GPIO_FACTORY_MODE;
	gpio2 = M040_VOLUMEUP_KEY;
	gpio3 = M040_VOLUMEDOWN_KEY;
	
	switch(type) {
	case MX_FACTORY_TEST_BT:
		if(!gpio_get_value(gpio1) && !gpio_get_value(gpio2))
			ret = 1;
		break;
	case MX_FACTORY_TEST_CAMERA:
		if(!gpio_get_value(gpio1) && !gpio_get_value(gpio3))
			ret = 1;
		break;
	default:
		if(!gpio_get_value(gpio1))
			ret = 1;
		break;
	}

	return ret;

}

static int set_factory_test_led(int on){
	int gpio_value = on ? GPIOF_OUT_INIT_HIGH: GPIOF_OUT_INIT_LOW;
	int gpio, ret;


 	gpio = M040_GPIO_TEST_LED;

	ret = gpio_request_one(gpio, gpio_value, "mx2_test_led");
	if (ret)
		return ret;

	gpio_free(gpio);

	return 0;
}
static int  __init mx2_init_factory(void)
{
	mx_is_factory_test_mode = is_factory_test_mode;
	mx_set_factory_test_led = set_factory_test_led;
	return 0;
}

arch_initcall(mx2_init_factory);

