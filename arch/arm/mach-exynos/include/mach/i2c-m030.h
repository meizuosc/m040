/* linux/arch/arm/mach-exynos/i2c-m040.c
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author: 	wbwu	<wenbinwu@meizu.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
 
 #ifndef __I2C_M030_H
 #define __I2C_M030_H
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/map.h>

#include <plat/regs-iic.h>
#include <plat/iic.h>
extern struct s3c2410_platform_i2c m030_default_i2c0_data;
extern struct s3c2410_platform_i2c m030_default_i2c1_data;
extern struct s3c2410_platform_i2c m030_default_i2c2_data;
extern struct s3c2410_platform_i2c m030_default_i2c3_data;
extern struct s3c2410_platform_i2c m030_default_i2c4_data;
extern struct s3c2410_platform_i2c m030_default_i2c5_data;
extern struct s3c2410_platform_i2c m030_default_i2c6_data;
extern struct s3c2410_platform_i2c m030_default_i2c7_data;
#endif //__I2C_M030_H