/* linux/arch/arm/mach-exynos/i2c-m032.c
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author: 	wbwu	<wenbinwu@meizu.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <mach/i2c-m032.h>

 /*audio codec*/
struct s3c2410_platform_i2c m032_default_i2c0_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 0,
};
 /*max77686 pmic*/
struct s3c2410_platform_i2c m032_default_i2c1_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 1,
};

struct s3c2410_platform_i2c m032_default_i2c2_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 2,
};
struct s3c2410_platform_i2c m032_default_i2c3_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 3,
};
/*audio noise*/
struct s3c2410_platform_i2c m032_default_i2c4_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 4,
};
/*camera isp*/
struct s3c2410_platform_i2c m032_default_i2c5_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 5,
};
/*lcd backlight*/
struct s3c2410_platform_i2c m032_default_i2c6_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 6,
};
/*touch ic*/
struct s3c2410_platform_i2c m032_default_i2c7_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 7,
};