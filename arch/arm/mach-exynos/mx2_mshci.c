/*
 * mx2_mshci.c - lcd driver helper for mx board
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

#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/scatterlist.h>

#include <mach/dev.h>
#include <mach/dwmci.h>
#include <mach/gpio.h>

#include <plat/devs.h>
#include <plat/gpio-cfg.h>
#include <plat/mshci.h>
#include <plat/cpu.h>

#ifdef CONFIG_EXYNOS4_DEV_MSHC
static struct s3c_mshci_platdata __initdata mx2_mshc_pdata = {
	.cd_type		= S3C_MSHCI_CD_PERMANENT,
	.fifo_depth	= 0x80,
	.max_width	= 8,
#if defined(CONFIG_EXYNOS4_MSHC_DDR)
	.host_caps	= MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23 |
				   MMC_CAP_1_8V_DDR |MMC_CAP_UHS_DDR50,
	.host_caps2	= MMC_CAP2_PACKED_CMD,
#else
	.host_caps	= MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
#endif
};

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
static void m040_dwmci_cfg_gpio(int width)
{
	static int pre_width = -1;

	unsigned int gpio;

	if (pre_width == width)
		return;

	for (gpio = EXYNOS4_GPK0(0); gpio < EXYNOS4_GPK0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	}

	switch (width) {
	case 8:
		for (gpio = EXYNOS4_GPK1(3); gpio <= EXYNOS4_GPK1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(4));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
	case 4:
		for (gpio = EXYNOS4_GPK0(3); gpio <= EXYNOS4_GPK0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
		break;
	case 1:
		gpio = EXYNOS4_GPK0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	default:
		break;
	}

	pre_width = width;
}

static int m040_dwmci_get_bus_wd(u32 slot_id)
{
	if (0 == slot_id)
		return 8;
	return 4;
}

static struct dw_mci_board __initdata m040_dwmci_pdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION |
					    DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps				= MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR |
					   MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio			= m040_dwmci_cfg_gpio,
	.get_bus_wd		= m040_dwmci_get_bus_wd,
};
#endif

static struct platform_device __initdata *m040_emmc_devices[]  = {
#if defined(CONFIG_EXYNOS4_DEV_DWMCI)
	&exynos_device_dwmci,
#endif
#ifdef CONFIG_EXYNOS4_DEV_MSHC
	&s3c_device_mshci,
#endif
};

static int  __init mx2_init_mshci(void)
{
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos_dwmci_set_platdata(&m040_dwmci_pdata);
#endif
#ifdef CONFIG_EXYNOS4_DEV_MSHC
	s3c_mshci_set_platdata(&mx2_mshc_pdata);
#endif
	if(platform_add_devices(m040_emmc_devices, ARRAY_SIZE(m040_emmc_devices))){
		pr_err("%s: register audio device fail\n", __func__);
	}
	return 0;
}

arch_initcall(mx2_init_mshci);
#endif
