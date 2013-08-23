/* linux/arch/arm/mach-exynos/i2c-m040.c
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author: 	wbwu	<wenbinwu@meizu.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
 #include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/gpio.h>
#include <plat/iic.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>

#include <mach/gpio-m040.h>
#include <mach/i2c-m040.h>

 /*audio codec*/
struct s3c2410_platform_i2c m040_default_i2c0_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 0,
};
 /*max77686 pmic*/
struct s3c2410_platform_i2c m040_default_i2c1_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 1,
};

struct s3c2410_platform_i2c m040_default_i2c2_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 2,
};
struct s3c2410_platform_i2c m040_default_i2c3_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 3,
};
/*audio noise*/
struct s3c2410_platform_i2c m040_default_i2c4_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 4,
};
/*camera isp*/
struct s3c2410_platform_i2c m040_default_i2c5_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 5,
};
/*lcd backlight*/
struct s3c2410_platform_i2c m040_default_i2c6_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 6,
};
/*touch ic*/
struct s3c2410_platform_i2c m040_default_i2c7_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 400*1000,
	.sda_delay	= 100,
	.bus_num 	= 7,
};

/*GPIO I2C setting*/

/* gpio I2C8: MHL */
static struct i2c_gpio_platform_data gpio_i2c8_data  = {
	.sda_pin		= M040_SDA_MHL,
	.scl_pin		= M040_SCL_MHL,
	.udelay			= 5,
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

struct platform_device m040_device_gpio_i2c8 = {
	.name	= "i2c-gpio",
	.id	= 8,
	.dev	= {
		.platform_data = &gpio_i2c8_data
	},
};

/*gpio i2c 9: st lis3dh accelerometer sensor*/
static struct i2c_gpio_platform_data gpio_i2c9_data = {
	.sda_pin = M040_SDA_GS,
	.scl_pin = M040_SCL_GS,
	.udelay = 5,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

struct platform_device m040_device_gpio_i2c9 = {
	.name = "i2c-gpio",
	.id = 9,
	.dev.platform_data = &gpio_i2c9_data,
};
/*gpio i2c 10:akm8975C compass sensor*/
static struct i2c_gpio_platform_data gpio_i2c10_data = {
	.sda_pin = M040_SDA_CP,
	.scl_pin = M040_SCL_CP,
	.udelay = 5,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

struct platform_device m040_device_gpio_i2c10 = {
	.name = "i2c-gpio",
	.id = 10,
	.dev.platform_data = &gpio_i2c10_data,
};
/*gpio i2c 11:st l3g4200d gyroscope sensor*/
static struct i2c_gpio_platform_data gpio_i2c11_data = {
	.sda_pin = M040_SDA_GY,
	.scl_pin = M040_SCL_GY,
	.udelay = 5,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

struct platform_device m040_device_gpio_i2c11 = {
	.name = "i2c-gpio",
	.id = 11,
	.dev.platform_data = &gpio_i2c11_data,
};

/*gpio i2c 12: sharp gp2ap020a00f sensor*/
static struct i2c_gpio_platform_data gpio_i2c12_data = {
	.sda_pin = M040_SDA_IR,
	.scl_pin = M040_SCL_IR,
	.udelay = 5,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

struct platform_device m040_device_gpio_i2c12 = {
	.name = "i2c-gpio",
	.id = 12,
	.dev.platform_data = &gpio_i2c12_data,
};

/*gpio i2c 13: HPD*/
static struct i2c_gpio_platform_data gpio_i2c13_data = {
	.sda_pin = M040_SDA_HPD,
	.scl_pin = M040_SCL_HPD,
	.udelay = 2,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

struct platform_device m040_device_gpio_i2c13 = {
	.name = "i2c-gpio",
	.id = 13,
	.dev.platform_data = &gpio_i2c13_data,
};

/*gpio i2c 14: CHARGER*/
static struct i2c_gpio_platform_data gpio_i2c14_data = {
	.sda_pin = M040_SDA_CHG,
	.scl_pin = M040_SCL_CHG,
	.udelay = 2,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

struct platform_device m040_device_gpio_i2c14 = {
	.name = "i2c-gpio",
	.id = 14,
	.dev.platform_data = &gpio_i2c14_data,
};

/*gpio i2c 15: FULEGAUGE*/
static struct i2c_gpio_platform_data gpio_i2c15_data = {
	.sda_pin = M040_SDA_FUEL0,
	.scl_pin = M040_SCL_FUEL0,
	.udelay = 10,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

struct platform_device m040_device_gpio_i2c15 = {
	.name = "i2c-gpio",
	.id = 15,
	.dev.platform_data = &gpio_i2c15_data,
};
/*gpio i2c 16: TOUCH PAD*/
static struct i2c_gpio_platform_data m040_gpio_i2c16_data = {
	.sda_pin = M040_SDA_TOUCHPAD,
	.scl_pin = M040_SCL_TOUCHPAD,
	.udelay = 5,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

struct platform_device m040_device_gpio_i2c16 = {
	.name = "i2c-gpio",
	.id = 16,
	.dev.platform_data = &m040_gpio_i2c16_data,
};
/*gpio i2c 17: EARPHONE*/
static struct i2c_gpio_platform_data gpio_i2c17_data = {
	.sda_pin = M040_SDA_EARPHONE,
	.scl_pin = M040_SCL_EARPHONE,
	.udelay = 2,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};
struct platform_device m040_device_gpio_i2c17 = {
	.name = "i2c-gpio",
	.id = 17,
	.dev.platform_data = &gpio_i2c17_data,
};
/*gpio i2c 18: M041 TOUCH PAD*/
static struct i2c_gpio_platform_data m041_gpio_i2c16_data = {
	.sda_pin = M041_SDA_TOUCHPAD,
	.scl_pin = M041_SCL_TOUCHPAD,
	.udelay = 5,   /*the scl frequency is (500 / udelay) kHz*/
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

struct platform_device m041_device_gpio_i2c16 = {
	.name = "i2c-gpio",
	.id = 16,
	.dev.platform_data = &m041_gpio_i2c16_data,
};

