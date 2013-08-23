/*
 * mx2_ts.c - touch pannel driver helper for m040 board
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
 
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/gpio.h>
#ifdef CONFIG_RMI4_I2C
#include <linux/rmi.h>
#endif

#include <asm/mach-types.h>
#include <plat/devs.h>
#include <plat/iic.h>
#include <mach/i2c-m040.h>
#include <mach/gpio-m040.h>

#if defined(CONFIG_TOUCHSCREEN_MXT224S)
#include <linux/i2c/atmel_mxts_ts.h>
#endif

#ifdef CONFIG_RMI4_I2C

#define RMI4_ADDR	(0x20)

struct syna_gpio_data {
	u16 gpio_number;
	char* gpio_name;
};


static int synaptics_touchpad_gpio_setup(void *gpio_data, bool configure)
{
	int retval=0;
	struct syna_gpio_data *data = gpio_data;

	if (configure) {
		retval = gpio_request(data->gpio_number, "rmi4_attn");
		if (retval) {
			pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			return retval;
		}

		retval = gpio_direction_input(data->gpio_number);
		if (retval) {
			pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			gpio_free(data->gpio_number);
		}
	} else {
		pr_warn("%s: No way to deconfigure gpio %d.",
		       __func__, data->gpio_number);
	}

	return retval;
}


static struct syna_gpio_data rmi4_gpiodata = {
	.gpio_number = M040_TOUCH_IRQ,
	.gpio_name = "sdmmc2_clk.gpio_191",
};


static struct rmi_device_platform_data rmi4_platformdata = {
	.driver_name     = "rmi_generic",       
	.sensor_name     = "m040 touch",
	.attn_gpio       = M040_TOUCH_IRQ,      
	.vbus_gpio       = M040_USB_IRQ,     
	.reset_gpio       = M040_TOUCH_RESET,     
	.attn_polarity   = RMI_ATTN_ACTIVE_LOW, 
	.level_triggered = true,         
	.gpio_data = &rmi4_gpiodata,
	.gpio_config = synaptics_touchpad_gpio_setup,
};
#endif


#if defined(CONFIG_TOUCHSCREEN_MXT224S)
/* Initial register values recommended from chip vendor */
static const u8 mxt224_config_data[] = {
       /* T37 Object, 130 */
       /*0x11, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
       0x00, 0x00, */
       /* T38 Object, 8 */
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	/* T7 Object, 4 */
	0x14, 0x0A, 0x32, 0x03,
	/* T8 Object, 10 */
	0x1B, 0x00, 0x14, 0x14, 0x00, 0x00, 0x02, 0x19, 0x00, 0x00,
	/* T9 Object, 36 */
	0x8B, 0x01, 0x01, 0x13, 0x0B, 0x00, 0x40, 0x32, 0x02, 0x05,
	0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xFF, 0x04,
	0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0F,0x00, 0x00, 0x01, 0x00,
	/* T15 Object, 11 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00,
	/* T18 Object, 2  */
	0x00, 0x00,
	/* T19 Object, 6 */
	0x00, 0x00, 0x00, 0x0C, 0x00, 0x00,
	/* T23 Object, 15 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00,
	/* T25 Object, 15 */
	0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00,
	/* T40 Object, 5 */
	0x00, 0x00, 0x00, 0x00, 0x00,
	/* T42 Object , 10*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* T46 Object, 10 */
	0x00, 0x00, 0x10, 0x20, 0x00, 0x00, 0x03, 0x00, 0x00, 0x01,
	/* T47 Object , 13*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00,
	/* T55 Object , 6*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* T56 Object, 42*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00,
	/* T57 Object, 3 */
	0x00, 0x00, 0x00,
	/* T61 Object, 5 */
       0x00, 0x00, 0x00, 0x00, 0x00, 
       /* T62 Object, 54 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
       0x00, 0x00, 0x00, 0x00,
};

static const struct mxt_config_info mxt224_config_info = {
	.config = mxt224_config_data, 
	.config_length = ARRAY_SIZE(mxt224_config_data),
	.family_id = MXT224S_ID,
	.variant_id = 26,
	.version = 0x10,
	.build = 0xAA,
	.bootldr_id = MXT_BOOTLOADER_ID_224,
	
	/* Points to the firmware name to be upgraded to */
	.fw_name = "mxtouch.fw",
};

static struct mxt_platform_data mxt224_platform_data = {
	.config_array = &mxt224_config_info, 
	.config_array_size = sizeof(mxt224_config_info),
	
	/* touch panel's minimum and maximum coordinates */
	.panel_minx = 0,
	.panel_maxx = 800,
	.panel_miny = 0,
	.panel_maxy = 1280,
	
	/* display's minimum and maximum coordinates */
	.disp_minx = 0,
	.disp_maxx = 800,
	.disp_miny = 0,
	.disp_maxy = 1280,
		
	.irqflags	= IRQF_TRIGGER_FALLING,
	.reset_gpio = M040_TOUCH_RESET,
	.irq_gpio = M040_TOUCH_IRQ,
};
#endif

static struct i2c_board_info __initdata i2c_devs7[] = {
#ifdef	CONFIG_RMI4_I2C      
	{
		I2C_BOARD_INFO("rmi_i2c", RMI4_ADDR),
		.platform_data = &rmi4_platformdata,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_MXT224S)
	{
		I2C_BOARD_INFO("mXT224", 0x4B),
		.platform_data	= &mxt224_platform_data,
		.irq = IRQ_EINT(1),
	},
#endif
};

static struct platform_device __initdata *m040_tsp_devices[]  = {
	&s3c_device_i2c7,
};
static int  __init mx2_init_ts(void)
{
	/* touch pannel */
	s3c_i2c7_set_platdata(&m040_default_i2c7_data);
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));
	if(platform_add_devices(m040_tsp_devices, ARRAY_SIZE(m040_tsp_devices))){
		pr_err("%s: register touchscreen device fail\n", __func__);
	}
	return 0;
}

arch_initcall(mx2_init_ts);

MODULE_DESCRIPTION("m040 touch pannel driver helper");
MODULE_AUTHOR("lvcha qiu <lvcha@meizu.com>");
MODULE_LICENSE("GPLV2");
