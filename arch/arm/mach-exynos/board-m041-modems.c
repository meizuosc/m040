/**
 * arch/arm/mach-exynos/board-m041-modems.c
 *
 * Copyright (C) 2010 Samsung Electronics.
 * Copyright (C) 2013 Zhuhai Meizu Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/platform_data/modem.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/spi/spi.h>
#include <asm/mach-types.h>
#include <plat/devs.h>
#include <plat/s3c64xx-spi.h>
#include <mach/spi-clocks.h>

#include <mach/map.h>
#include <mach/gpio-common.h>
#include <mach/gpio-m040.h>
#include <mach/mx2_cs8803g_spi.h>

struct modem_gpio_info_table{
	unsigned char *name;
	unsigned int export;
	unsigned int pin;
	unsigned int type;
	unsigned int data;
	unsigned int pull;
	unsigned int drv;
};

static const struct modem_gpio_info_table m041_modem_gpio_table[] = {

	{"AP_REDY", 	1, M041_GPIO_AP_RDY, 			S3C_GPIO_OUTPUT, 	GPIO_SETPIN_HI, 	GPIO_PULL_NONE, 		GPIO_DRVSTR_LV1},/*M041 AP READY*/
	{"AP_POWER", 	1, M041_GPIO_MODEM_POWER_ON, 	S3C_GPIO_OUTPUT, 	GPIO_SETPIN_LOW, 	GPIO_PULL_NONE, 		GPIO_DRVSTR_LV1},/*M041 AP POWER ON*/
	{"AP_RTS", 		1, M041_GPIO_AP_RTS, 			S3C_GPIO_OUTPUT, 	GPIO_SETPIN_LOW, 	GPIO_PULL_NONE, 		GPIO_DRVSTR_LV1},/*M041 AP RTS*/	
	{"AP_RESEND", 	1, M041_GPIO_AP_RESEND, 		S3C_GPIO_OUTPUT, 	GPIO_SETPIN_LOW, 	GPIO_PULL_NONE, 		GPIO_DRVSTR_LV1},/*M041 AP RESEND*/


	{"MDM_RESEND", 	1, M041_GPIO_MODEM_RESEND, 		S3C_GPIO_INPUT, 		GPIO_SETPIN_NONE, 	GPIO_PULL_DOWN, 	GPIO_DRVSTR_LV1},/*M041 Modem RESEND*/
	{"MDM_RDY", 	1, M041_GPIO_MODEM_RDY, 		S3C_GPIO_INPUT, 		GPIO_SETPIN_NONE, 	GPIO_PULL_UP, 		GPIO_DRVSTR_LV1},/*M041 Modem RDY*/
	{"MDM_RTS", 	1, M041_GPIO_MODEM_RTS, 		S3C_GPIO_INPUT, 		GPIO_SETPIN_NONE, 	GPIO_PULL_UP, 		GPIO_DRVSTR_LV1},/*M041 Modem RTS*/
	{"MDM_ALIVE", 	1, M041_GPIO_MODEM_ALIVE, 		S3C_GPIO_INPUT, 		GPIO_SETPIN_NONE, 	GPIO_PULL_DOWN,		GPIO_DRVSTR_LV1},/*M041 Modem Alive*/

#ifndef CONFIG_MX2_SC8803G_TEST
	{"AP_TO_MDM2",	1, M041_GPIO_AP_TO_MODEM2, 		S3C_GPIO_OUTPUT, 	GPIO_SETPIN_HI, 	GPIO_PULL_NONE, 	GPIO_DRVSTR_LV1},/*M041 AP TO MDM2*/
	{"AP_TO_MDM1", 	1, M041_GPIO_AP_TO_MODEM1, 		S3C_GPIO_OUTPUT, 	GPIO_SETPIN_HI, 	GPIO_PULL_UP, 	GPIO_DRVSTR_LV1},/*M041 AP TO MDM1*/
	{"MDM_TO_AP1", 	1, M041_GPIO_MODEM_TO_AP1, 		S3C_GPIO_INPUT, 	GPIO_SETPIN_NONE, 	GPIO_PULL_DOWN, 	GPIO_DRVSTR_LV1},/*M041 MDM TO AP1*/
	{"MDM_TO_AP2", 	1, M041_GPIO_MODEM_TO_AP2, 		S3C_GPIO_INPUT, 	GPIO_SETPIN_NONE, 	GPIO_PULL_DOWN,		GPIO_DRVSTR_LV1},/*M041 MDM TO AP2*/
#endif	

};

static int m041_modem_gpio_config(const struct modem_gpio_info_table *gpio_table, int array_size)
{
	int ret = 0;
	struct modem_gpio_info_table gpio;
	unsigned int i;

	for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i];

		ret = gpio_request(gpio.pin, gpio.name);
		if (ret < 0)
			pr_err( "Unable to allocate GPIO%d (%s)", gpio.pin,gpio.name);
		ret += gpio_export(gpio.pin, gpio.export);
		if (ret)
			pr_err( "Unable to configure GPIO%d (%s)", gpio.pin,gpio.name);

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
	return ret;
}

static struct spt_modem_platform_data default_mdm_data={
#ifdef CONFIG_MX2_SC8803G_TEST
	.pwr_on = M041_GPIO_MODEM_POWER_ON,

	.srdy = M041_GPIO_MODEM_RDY,
	.mrdy = M041_GPIO_AP_RDY,
	.srts = M041_GPIO_MODEM_RTS,
	.mrts = M041_GPIO_AP_RTS,
	.srsd = M041_GPIO_MODEM_RESEND,
	.mrsd = M041_GPIO_AP_RESEND,
	.salive = M041_GPIO_MODEM_ALIVE,
#else
	.pwr_on = M041_GPIO_MODEM_POWER_ON,

	.srdy = M041_GPIO_MODEM_RDY,
	.mrdy = M041_GPIO_AP_RDY,
	.srts = M041_GPIO_MODEM_RTS,
	.mrts = M041_GPIO_AP_RTS,
	.srsd = M041_GPIO_MODEM_RESEND,
	.mrsd = M041_GPIO_AP_RESEND,
	.salive = M041_GPIO_MODEM_ALIVE,

	.s2m1 = M041_GPIO_MODEM_TO_AP1,
	.s2m2 = M041_GPIO_MODEM_TO_AP2,
	.m2s1 = M041_GPIO_AP_TO_MODEM1,
	.m2s2 = M041_GPIO_AP_TO_MODEM2,
#endif
	.modem_type = SPT_MODEM_SC8803G,
	.mode		= SPI_MODE_3,
	.bit_per_word	= 32,
	.max_hz = 12*1000*1000,
	.use_dma = 1,
};

#ifdef CONFIG_MX2_SC8803G_CONTROL
static struct platform_device m041_mdm_device = {
	.name		  = "sc880xg-modem",
	.id		  = -1,
	.dev = {
		.platform_data = &default_mdm_data,
	},
};
#endif

#ifdef CONFIG_MX2_SC8803G
#ifdef CONFIG_MX2_SC8803G_TEST
static struct s3c64xx_spi_csinfo spi0_csi[] = {
	[0] = {
		.line = EXYNOS4_GPB(1),
		.set_level = gpio_set_value,
		.fb_delay = 0x2,
	},
};

static struct spi_board_info __initdata spi0_board_info[] = {
	{
		.modalias = "sc880xg-spi",
		.platform_data = &default_mdm_data,
		.max_speed_hz = 12*1000*1000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
		.controller_data = &spi0_csi[0],
	}
};
#else//CONFIG_MX2_SC8803G_TEST
static struct s3c64xx_spi_csinfo spi1_csi[] = {
	[0] = {
		.line = EXYNOS4_GPB(5),
		.set_level = gpio_set_value,
		.fb_delay = 0x1,
	},
};

static struct spi_board_info __initdata spi1_board_info[] = {
	{
		.modalias = "sc880xg-spi",
		.platform_data = &default_mdm_data,
		.max_speed_hz = 12*1000*1000,
		.bus_num = 1,
		.chip_select = 0,
		.mode = SPI_MODE_1,
		.controller_data = &spi1_csi[0],
	}
};
#endif//CONFIG_MX2_SC8803G_TEST
#endif//CONFIG_SPI_S3C64XX

static struct platform_device __initdata *m041_modem_devices[] = {
#ifdef CONFIG_MX2_SC8803G_CONTROL
	&m041_mdm_device,
#endif
#ifdef CONFIG_MX2_SC8803G
#ifdef CONFIG_MX2_SC8803G_TEST
	&exynos_device_spi0,
#else
	&exynos_device_spi1,
#endif
#endif
};

int m041_modem_device_init(void)
{
	int ret = 0;

	pr_info("[MODEM_IF] init_modem for td\n");
	m041_modem_gpio_config(m041_modem_gpio_table, ARRAY_SIZE(m041_modem_gpio_table));

#ifdef CONFIG_MX2_SC8803G
#ifdef CONFIG_MX2_SC8803G_TEST
	exynos_spi_set_info(0, EXYNOS_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi0_csi));
	spi_register_board_info(spi0_board_info, ARRAY_SIZE(spi0_board_info));
#else
	exynos_spi_set_info(1, EXYNOS_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi1_csi));
	spi_register_board_info(spi1_board_info, ARRAY_SIZE(spi1_board_info));
#endif
#endif
	ret = platform_add_devices(m041_modem_devices, ARRAY_SIZE(m041_modem_devices));

	pr_info("[MODEM_IF] init_modem device over, ret=%d.\n", ret);

	return ret;
}

