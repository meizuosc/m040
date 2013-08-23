/* linux/arch/arm/mach-exynos/i2c-m040.c
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author: 	wbwu	<wenbinwu@meizu.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
 
 #ifndef __I2C_M040_H
 #define __I2C_M040_H
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/map.h>

#include <plat/regs-iic.h>
#include <plat/iic.h>
 /*audio codec*/
extern struct s3c2410_platform_i2c m040_default_i2c0_data;
 /*max77686 pmic*/
extern struct s3c2410_platform_i2c m040_default_i2c1_data;
extern struct s3c2410_platform_i2c m040_default_i2c2_data;
extern struct s3c2410_platform_i2c m040_default_i2c3_data;
/*audio noise*/
extern struct s3c2410_platform_i2c m040_default_i2c4_data;
/*camera isp*/
extern struct s3c2410_platform_i2c m040_default_i2c5_data;
/*lcd backlight*/
extern struct s3c2410_platform_i2c m040_default_i2c6_data;
/*touch ic*/
extern struct s3c2410_platform_i2c m040_default_i2c7_data;

extern struct platform_device m040_device_gpio_i2c8;
extern struct platform_device m040_device_gpio_i2c9;
extern struct platform_device m040_device_gpio_i2c10;
extern struct platform_device m040_device_gpio_i2c11;
extern struct platform_device m040_device_gpio_i2c12;
extern struct platform_device m040_device_gpio_i2c13;
extern struct platform_device m040_device_gpio_i2c14;
extern struct platform_device m040_device_gpio_i2c15;
extern struct platform_device m040_device_gpio_i2c16;
extern struct platform_device m040_device_gpio_i2c17;
extern struct platform_device m041_device_gpio_i2c16;
#endif //__I2C_M040_H