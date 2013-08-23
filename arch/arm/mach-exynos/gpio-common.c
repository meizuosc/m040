/*
 * linux/arch/arm/mach-exynos/gpio-m30x.c
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author: 	lvcha qiu	<lvcha@meizu.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <asm/mach-types.h>
#include <mach/map.h>
#include <mach/gpio-common.h>

int (*mx_is_factory_test_mode)(int type);
int (*mx_set_factory_test_led)(int on);

int mx_config_gpio_table(const struct gpio_info_table *gpio_table, int array_size)
{
	struct gpio_info_table gpio;
	unsigned int i;

	for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i];

		/* Off part */
		if(gpio.pin <= EXYNOS4_GPIO_END) {
			if(gpio.type == S3C_GPIO_RESERVED) {
				s3c_gpio_setpull(gpio.pin, gpio.pull);
			} else if (gpio.type == S3C_GPIO_INPUT) {
				s3c_gpio_setpull(gpio.pin, gpio.pull);
				s3c_gpio_cfgpin(gpio.pin, gpio.type);
			} else if (gpio.type == S3C_GPIO_OUTPUT) {
				s3c_gpio_setpull(gpio.pin, gpio.pull);
				gpio_set_value(gpio.pin, !!gpio.data);
				s3c_gpio_cfgpin(gpio.pin, S3C_GPIO_OUTPUT);
				s5p_gpio_set_drvstr(gpio.pin,  gpio.drv);
			}
		}
	}
	return 0;
}

inline void mx_set_sleep_pin(unsigned int pin, s5p_gpio_pd_cfg_t conpdn, s5p_gpio_pd_pull_t pudpdn)
{
	s5p_gpio_set_pd_cfg(pin, conpdn);
	s5p_gpio_set_pd_pull(pin, pudpdn);
}