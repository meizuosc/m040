/*
 * mx2_audio.c - audio driver helper for m040 board
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
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
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/mx_spdif_platform.h>
#include <linux/mfd/wm8994/pdata.h>
#ifdef CONFIG_AUDIENCE_ES305B
#include <linux/es305b_soc.h>
#endif

#include <asm/mach-types.h>

#include <plat/iic.h>
#include <plat/devs.h>

#include <mach/dev.h>
#include <mach/i2c-m040.h>
#include <mach/gpio-m040.h>

#if defined(CONFIG_MFD_WM8994)
static struct regulator_consumer_supply wm8958_avdd1_supply =
	REGULATOR_SUPPLY("AVDD1", "0-001a");

static struct regulator_consumer_supply wm8958_dcvdd_supply =
	REGULATOR_SUPPLY("DCVDD", "0-001a");

static struct regulator_init_data wm8958_ldo1_data = {
	.constraints	= {
		.name		= "AVDD1",
		.min_uV		= 2400000,
		.max_uV		= 3100000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8958_avdd1_supply,
};

static struct regulator_init_data wm8958_ldo2_data = {
	.constraints	= {
		.name		= "DCVDD",
		.min_uV		= 1000000,
		.max_uV		= 1300000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8958_dcvdd_supply,
};

struct wm8958_mbc_cfg m040_mbc_cfgs[] = {
	{
		.name = "playback spk normal",
		.coeff_regs[0] = 0x0028,
		.coeff_regs[1] = 0x1851,
		.coeff_regs[2] = 0x0032,
		.coeff_regs[3] = 0xF52D,
		.coeff_regs[4] = 0x0065,
		.coeff_regs[5] = 0xAC8C,
		.coeff_regs[6] = 0x006B,
		.coeff_regs[7] = 0xE087,
		.coeff_regs[8] = 0x0072,
		.coeff_regs[9] = 0x1483,
		.coeff_regs[10] = 0x0072,
		.coeff_regs[11] = 0x1483,
		.coeff_regs[12] = 0x0045,
		.coeff_regs[13] = 0x156D,
		.coeff_regs[14] = 0x000A,
		.coeff_regs[15] = 0x2ADB,
		.coeff_regs[16] = 0x0045,
		.coeff_regs[17] = 0x85B9,
		.coeff_regs[18] = 0x000C,
		.coeff_regs[19] = 0xCCCD,
		.coeff_regs[20] = 0x0000,
		.coeff_regs[21] = 0x0800,
		.coeff_regs[22] = 0x003F,
		.coeff_regs[23] = 0x8BD8,
		.coeff_regs[24] = 0x0032,
		.coeff_regs[25] = 0xF52D,
		.coeff_regs[26] = 0x0065,
		.coeff_regs[27] = 0xAC8C,
		.coeff_regs[28] = 0x006B,
		.coeff_regs[29] = 0xE087,
		.coeff_regs[30] = 0x0072,
		.coeff_regs[31] = 0x1483,
		.coeff_regs[32] = 0x0072,
		.coeff_regs[33] = 0x1483,
		.coeff_regs[34] = 0x0043,
		.coeff_regs[35] = 0x3525,
		.coeff_regs[36] = 0x0006,
		.coeff_regs[37] = 0x6A4A,
		.coeff_regs[38] = 0x0043,
		.coeff_regs[39] = 0x6079,
		.coeff_regs[40] = 0x0020,
		.coeff_regs[41] = 0x0000,
		.coeff_regs[42] = 0x0000,
		.coeff_regs[43] = 0xFF97,
		.coeff_regs[44] = 0x005A,
		.coeff_regs[45] = 0x9DF8,
		.coeff_regs[46] = 0x005A,
		.coeff_regs[47] = 0x7EFA,
		.cutoff_regs[0] = 0x00A7,
		.cutoff_regs[1] = 0x0D1C,
		.cutoff_regs[2] = 0x0059,
		.cutoff_regs[3] = 0xFE0F,
		.cutoff_regs[4] = 0x0099,
		.cutoff_regs[5] = 0xD861,
		.cutoff_regs[6] = 0x00A7,
		.cutoff_regs[7] = 0x0D1C,
		.cutoff_regs[8] = 0x0059,
		.cutoff_regs[9] = 0xFE0F,
		.cutoff_regs[10] = 0x0099,
		.cutoff_regs[11] = 0xD861,
		.cutoff_regs[12] = 0x002B,
		.cutoff_regs[13] = 0x3769,
		.cutoff_regs[14] = 0x0017,
		.cutoff_regs[15] = 0xB53B,
		.cutoff_regs[16] = 0x0008,
		.cutoff_regs[17] = 0xE7A2,
		.cutoff_regs[18] = 0x0086,
		.cutoff_regs[19] = 0x3A99,
	},
};

static struct wm8994_pdata wm8958_platform_data = {
	/* configure gpio1 function: 0x0001(Logic level input/output) */
	.gpio_defaults[0] = 0x0001,
	/* If the i2s0 and i2s2 is enabled simultaneously */
	.gpio_defaults[7] = 0x8100, /* GPIO8  DACDAT3 in */
	.gpio_defaults[8] = 0x0100, /* GPIO9  ADCDAT3 out */
	.gpio_defaults[9] = 0x0100, /* GPIO10 LRCLK3  out */
	.gpio_defaults[10] = 0x0100,/* GPIO11 BCLK3   out */
	.ldo[0] = {M040_CODEC_LDO1_EN, NULL, &wm8958_ldo1_data},
	.ldo[1] = {M040_CODEC_LDO2_EN, NULL, &wm8958_ldo2_data},
	.num_mbc_cfgs = 1,
	.mbc_cfgs = &m040_mbc_cfgs[0],
};
#endif

static struct i2c_board_info __initdata i2c_devs0[] = {
#if defined(CONFIG_MFD_WM8994)
	{
		I2C_BOARD_INFO("wm8958", (0x34 >> 1)),
		.platform_data	= &wm8958_platform_data,
	},
#endif

};

#ifdef CONFIG_SND_SOC_MX_WM8958
static struct platform_device m040_audio_device = {
	.name = "mx-audio",
	.id = -1,
};
#endif

#if defined(CONFIG_AUDIENCE_ES305B)
/* es305b */
static struct es305b_platform_data __initdata es305b_pd = {
	.gpio_es305b_wake	= M040_NOISE_CANCELLER_WAKE,
	.gpio_es305b_reset	= M040_NOISE_CANCELLER_RST,
};
#endif
static struct i2c_board_info __initdata i2c_devs4[] = {
#if defined(CONFIG_AUDIENCE_ES305B)
	{
		I2C_BOARD_INFO(ES305B_I2C_NAME, ES305B_I2S_SLAVE_ADDRESS),
		.platform_data	= &es305b_pd,
	},
#endif
};

#ifdef CONFIG_SND_SAMSUNG_SPDIF
static void m040_spdif_output_enable(int bEn)
{
	unsigned long flags =  bEn ? GPIOF_OUT_INIT_LOW :
					   GPIOF_OUT_INIT_HIGH;
	gpio_request_one(M040_SPDIF_OUT, flags, "spdif output");
	gpio_free(M040_SPDIF_OUT);
}

static struct mx_spdif_platform_data m040_spdif_data={
	.spdif_output_enable = m040_spdif_output_enable,
};

static struct platform_device m040_spdif_device={
	.name = "mx-spdif",
	.id = -1,
	.dev = {
		.platform_data = &m040_spdif_data,
	},
};
#endif
static struct platform_device __initdata *m040_audio_devices[]  = {
	&s3c_device_i2c0,
#ifndef CONFIG_MX2_SC8803G_TEST
	&s3c_device_i2c4,
#endif
#if defined(CONFIG_SND_SAMSUNG_I2S)
	&exynos_device_i2s0,
	&samsung_asoc_idma,
#endif
#ifdef CONFIG_SND_SAMSUNG_PCM
	&exynos_device_pcm1,
	&samsung_asoc_dma,
#endif
#ifdef CONFIG_SND_SOC_MX_WM8958
	&m040_audio_device,
#endif		
#ifdef CONFIG_SND_SAMSUNG_SPDIF
	&exynos_device_spdif,
#endif
#ifdef CONFIG_SND_SOC_SAMSUNG_SMDK_SPDIF
	&m040_spdif_device,
#endif
};
static int  __init mx2_init_audio(void)
{
	/* wm8958 */
	s3c_i2c0_set_platdata(&m040_default_i2c0_data);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

	/* es305b */
#ifdef CONFIG_AUDIENCE_ES305B
#ifndef CONFIG_MX2_SC8803G_TEST
	s3c_i2c4_set_platdata(&m040_default_i2c4_data);
	i2c_register_board_info(4, i2c_devs4, ARRAY_SIZE(i2c_devs4));
#endif
#endif

	if(platform_add_devices(m040_audio_devices, ARRAY_SIZE(m040_audio_devices))){
		pr_err("%s: register audio device fail\n", __func__);
	}
	return 0;
}

arch_initcall(mx2_init_audio);

