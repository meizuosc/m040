/*
 * mx2_fb.c - lcd driver helper for m040 board
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *
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
#include <linux/regulator/machine.h>
#include <linux/lcd.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>

#include <mach/map.h>
#include <mach/gpio-m040.h>

#include <plat/mipi_dsim.h>
#include <plat/devs.h>
#include <plat/pd.h>

#include <../../../drivers/video/samsung/s3cfb.h>
#include <../../../drivers/video/samsung/meizu_lcd.h>

#if defined(CONFIG_FB_S5P_MIPI_DSIM)
/* fb helper*/
static struct s3cfb_lcd mx2_mipi_lcd = {
	.width = 800,
	.height = 1280,
	.p_width = 56,
	.p_height = 93,
	.bpp = 24,

	.freq = 60,

	.timing = {
		.h_fp = 0x1c,
		.h_bp = 0x1a,
		.h_sw = 0x5,
		.v_fp = 0xC,
		.v_bp = 0x6,
		.v_sw = 0x2,
		.cmd_allow_len = 0xf,
	},

	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 0,
		.inv_vsync = 0,
		.inv_vden = 0,
	},

};

static void mx2_fb_cfg_gpio(struct platform_device *pdev)
{
	u32 reg;

	/* Set FIMD0 bypass */
	reg = __raw_readl(S3C_VA_SYS + 0x0210);
	reg |= (1<<1);
	__raw_writel(reg, S3C_VA_SYS + 0x0210);
}

static struct s3c_platform_fb __initdata mx2_fb_pd = {
	.hw_ver		= 0x70,
	.nr_wins	= 5,
	.default_win	= CONFIG_FB_S5P_DEFAULT_WINDOW,
	.swap		= FB_SWAP_HWORD | FB_SWAP_WORD,
	.lcd[0]		= &mx2_mipi_lcd,
	.cfg_gpio	= mx2_fb_cfg_gpio,
};

/* mipi dsi helper */
static struct mipi_dsim_config mx2_dsi_config = {
	.manual_flush = false,
	.eot_disable = false,
	.auto_vertical_cnt = false,
	.hse = true,
	.hfp = false,
	.hbp = false,
	.hsa = false,
	.e_interface = DSIM_VIDEO,
	.e_virtual_ch = DSIM_VIRTUAL_CH_0,
	.e_pixel_format = DSIM_24BPP_888,
	.e_burst_mode = DSIM_NON_BURST_SYNC_EVENT,	/*for exynos4x12*/
	.e_no_data_lane = DSIM_DATA_LANE_4,
	.e_byte_clk = DSIM_PLL_OUT_DIV8,

	.p = 3,
	.m = 110,
	.s = 1,

	.pll_stable_time = 500,
	.esc_clk = 30 * 1000000,	/* escape clk : 10MHz */
	.stop_holding_cnt = 0x07ff,
	.bta_timeout = 0xff,		/* bta timeout 0 ~ 0xff */
	.rx_timeout = 0xffff,		/* lp rx timeout 0 ~ 0xffff */
	.e_lane_swap = DSIM_NO_CHANGE,
};

static int mx2_mipi_power(struct platform_device *pdev, unsigned int enable)
{
	int ret;
	struct regulator_bulk_data supplies[2];
	int num_consumers = ARRAY_SIZE(supplies);

	supplies[0].supply = "vdd_ldo8";
	supplies[1].supply = "vdd_ldo10";

	ret = regulator_bulk_get(&pdev->dev, num_consumers, supplies);
	if (ret) {
		dev_err(&pdev->dev, "regulator_bulk_get failed\n");
		return ret;
	}

	if (enable)
		ret = regulator_bulk_enable(num_consumers, supplies);
	else
		ret = regulator_bulk_disable(num_consumers, supplies);

	if (ret) {
		dev_err(&pdev->dev, "regulator_bulk_%sable failed\n",
				enable ? "en" : "dis");
		return ret;
	}

	regulator_bulk_free(num_consumers, supplies);

	return 0;
}

static struct s5p_platform_mipi_dsim __initdata mx2_dsi_pd = {
	.lcd_panel_name	= "lcd_panel",
	.phy_enable	= s5p_dsim_phy_enable,
	.mipi_power	= mx2_mipi_power,
	.dsim_config	= &mx2_dsi_config,
	.lcd_panel_info	= &mx2_mipi_lcd,

	/*
	 * the stable time of needing to write data on SFR
	 * when the mipi mode becomes LP mode.
	 */
	.delay_for_stabilization = 10,
};
#endif

#if defined(CONFIG_FB_MX2_MIPI_LCD)
/* mipi lcd helper */
static int lcd_panel_reset_level(struct lcd_device *ld, int level)
{
	int ret;
	unsigned int gpio = M040_LCD_RST;

	ret = gpio_request_one(gpio, GPIOF_OUT_INIT_LOW	, "LCD_RST");
	if (ret) {
		dev_err(&ld->dev, "gpio_request failed\n");
		return ret;
	}
	gpio_set_value(gpio, level);
	gpio_free(gpio);
	return 0;
}

static int lcd_panel_reset(struct lcd_device *ld)
{
	return lcd_panel_reset_level(ld, 0);
}
static int lcd_panel_power_vddio(struct lcd_device *ld, int enable)
{
	struct regulator *vddio = regulator_get(&ld->dev, "vdd_ldo13");
	int ret = 0;

	ret = enable ? regulator_enable(vddio) :
		regulator_disable(vddio);
	if (ret)
		dev_err(&ld->dev, "%s vddio failure!\n", enable ? "enable" : "disable");

	regulator_put(vddio);
	return ret;
}
static int lcd_panel_power_vci(struct lcd_device *ld, int enable)
{
	struct regulator *vci = regulator_get(&ld->dev, "LCD_2V8");
	int ret = 0;

	ret = enable ? regulator_enable(vci) :
		regulator_disable(vci);
	if (ret)
		dev_err(&ld->dev, "%s vci failure!\n", enable ? "enable" : "disable");

	regulator_put(vci);
	return ret;
}

static int lcd_panel_power_avdd(struct lcd_device *ld, int enable)
{
	struct regulator *lcd_5v = regulator_get(&ld->dev, "LCD_5V");
	int ret = 0;

	ret = enable ? regulator_enable(lcd_5v) :
		regulator_disable(lcd_5v);
	if (ret)
		dev_err(&ld->dev, "%s lcd5v failure!\n", enable ? "enable" : "disable");

	regulator_put(lcd_5v);
	return ret;
}
static int lcd_panel_power_avee(struct lcd_device *ld, int enable)
{
	struct regulator *lcd_n5v = regulator_get(&ld->dev, "LCD_N5V");
	int ret = 0;

	ret = enable ? regulator_enable(lcd_n5v) :
		regulator_disable(lcd_n5v);
	if (ret)
		dev_err(&ld->dev, "%s lcdn5v failure!\n", enable ? "enable" : "disable");

	regulator_put(lcd_n5v);
	return ret;
}

static int lcd_panel_power(struct lcd_device *ld, int enable, int panel)
{
	struct lcd_panel_info *lcd =
		container_of(&ld, struct lcd_panel_info, ld);

	if (enable) {
		if (panel == 0) {
			lcd_panel_reset_level(ld, 0);
			lcd_panel_power_vci(ld, enable);
			mdelay(1);
			lcd_panel_power_vddio(ld, enable);
			mdelay(1);
			lcd_panel_reset_level(ld, 1);
			mdelay(5);
			lcd_panel_power_avdd(ld, enable);
			mdelay(1);
			lcd_panel_power_avee(ld, enable);
			mdelay(1);
			msleep(40);
		} else {
			lcd_panel_reset_level(ld, 0);
			lcd_panel_power_vddio(ld, enable);
			mdelay(1);
			lcd_panel_power_vci(ld, enable);
			lcd_panel_power_avdd(ld, enable);
			mdelay(1);
			lcd_panel_power_avee(ld, enable);
			mdelay(10);
			lcd_panel_reset_level(ld, 1);
			mdelay(10);
		}
	} else {
		if (panel == 0) {
			lcd_panel_power_avee(ld, enable);
			mdelay(1);
			lcd_panel_power_avdd(ld, enable);
			mdelay(1);
			lcd_panel_reset_level(ld, 0);
			mdelay(1);
			lcd_panel_power_vddio(ld, enable);
			mdelay(100);
			lcd_panel_power_vci(ld, enable);
		} else {
			lcd_panel_power_avee(ld, enable);
			lcd_panel_power_avdd(ld, enable);
			lcd_panel_power_vci(ld, enable);
			lcd_panel_power_vddio(ld, enable);
			lcd_panel_reset_level(ld, 0);
		}
	}
	return 0;
}

/* ls040b3sx01 panel. */
static struct lcd_platform_data lcd_panel_pd = {
	.reset			= lcd_panel_reset,
	.power_on		= lcd_panel_power,
};

static struct mipi_dsim_lcd_device mx2_mipi_lcd_device = {
	.name		= "lcd_panel",
	.id		= -1,
	.bus_id		= 0,
	.platform_data	= (void *)&lcd_panel_pd,
};
#endif

static int  __init mx2_init_fb(void)
{
#if defined(CONFIG_FB_S5P)
	s3cfb_set_platdata(&mx2_fb_pd);
#ifndef CONFIG_PM_GENERIC_DOMAINS
	s3c_device_fb.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#endif

#if defined(CONFIG_FB_S5P_MIPI_DSIM)
	s5p_dsim_set_platdata(&mx2_dsi_pd, 0);
#ifndef CONFIG_PM_GENERIC_DOMAINS
	s5p_device_mipi_dsim0.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#endif
#if defined(CONFIG_FB_MX2_MIPI_LCD)
	mx2_mipi_lcd_device.irq = gpio_to_irq(M040_LCD_TE);

	s5p_mipi_dsi_register_lcd_device(&mx2_mipi_lcd_device);
#endif
	return 0;
}

arch_initcall(mx2_init_fb);
