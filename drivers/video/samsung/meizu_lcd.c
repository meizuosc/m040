#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/lcd.h>
#include <linux/backlight.h>

#include <video/mipi_display.h>
#include <plat/mipi_dsim.h>
#include "meizu_lcd.h"

#define CHECK_PANEL_RET(func) do {\
	int ret = func;\
	if (ret) {\
		msleep(120);\
		ret = func;\
		if (ret) {\
		pr_err("#LCD WRITE ERROR: line %d\n", __LINE__);\
		return ret;}}\
} while(0);

#define CHECK_LCD_NULL() do {\
	if (g_lcd_info == NULL)\
		return 0;\
}while(0);

struct sharp_ce_mode {
	const char *mode;
	int mode_val;
};

static struct sharp_ce_mode lcd_mode_map[] = {
	{"lowlow", 0},
	{"lowmed", 1},
	{"lowhigh", 2},
	{"medlow", 3},
	{"medmed", 4},
	{"medhigh", 5},
	{"highlow", 6},
	{"highmed", 7},
	{"highhigh", 8},
	{"satlow", 9},
	{"satmed", 10},
	{"sathigh", 11},
	{"ceoff", 12},
};
static int lcd_id[ID_CODEMAX] = {0,};
module_param_array(lcd_id, int, NULL, S_IRUGO | S_IWUSR | S_IWGRP);

static struct lcd_panel_info *g_lcd_info;
static int write_to_lcd(struct lcd_panel_info *lcd,
		const struct lcd_param *param)
{
	int i = 0, ret = 0;

	do {
		ret = param[i].size ?
			write_data(lcd, param[i].type,  param[i].param, param[i].size, BTA_TIMEOUT) :
			write_cmd(lcd, param[i].type, param[i].param[0], param[i].param[1], BTA_TIMEOUT);
		if (param[i].delay && param[i].delay > 20)
			msleep(param[i].delay);
		else if (param[i].delay && param[i].delay <= 20)
			usleep_range(param[i].delay*1000, param[i].delay*1000);
	} while (!ret && param[++i].size != -1);

	return ret;
}

static int lcd_panel_sharp_init_code(struct lcd_panel_info *lcd)
{
	pr_info("LCD ID Code3 %d\n", lcd_id[ID_CODE3]);
	switch (lcd_id[ID_CODE3]) {
		case 0:
			return write_to_lcd(lcd, sharp_init_seq_0_4);
		break;
		case 1:
			return write_to_lcd(lcd, sharp_init_seq_0_7);
		break;
		default:
			pr_info("ID Code(%d)! use default gamma settings.\n", lcd_id[ID_CODE3]);
			return 0;
		break;
	}
}

static int lcd_panel_sharp_sleep_in(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_slpin_seq);
}

static int lcd_panel_sharp_sleep_out(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_slpout_seq);
}

static int lcd_panel_sharp_display_on(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_dspon_seq); 
}

static int lcd_panel_sharp_display_off(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_dspoff_seq);
}
static int lcd_panel_jdi_gamma_seq(struct lcd_panel_info *lcd)
{
	int ret = 0;
	
	ret = write_to_lcd(lcd, jdi_gamma_cmd2page0);
	ret = write_to_lcd(lcd, jdi_gamma_cmd2page1);
	return ret;
}

static int lcd_panel_jdi_sleep_in(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, jdi_slpin_seq);
}

static int lcd_panel_jdi_sleep_out(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, jdi_slpout_seq);
}

static int lcd_panel_jdi_display_on(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, jdi_dspon_seq); 
}

static int lcd_panel_jdi_display_off(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, jdi_dspoff_seq);
}
#ifdef LCD_TEST
static int lcd_panel_hsync_out(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_hsync_out_seq);
}
static int lcd_panel_vsync_out(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_vsync_out_seq);
}
static int lcd_panel_set_brightness(struct lcd_panel_info *lcd, int brt)
{
	struct lcd_param sharp_brightness[] = {
		DCS_SHORT_PARAM(0x51, brt),
		LCD_PARAM_DEF_END,
	};

	return write_to_lcd(lcd, sharp_brightness);
}
#endif
static int lcd_panel_sharp_cabc_switch(struct lcd_panel_info *lcd, int enable)
{
	if (enable)
		return write_to_lcd(lcd, sharp_cabc_seq_on);
	else
		return write_to_lcd(lcd, sharp_cabc_seq_off);
}
static int lcd_panel_sharp_cabc_seq_prepare(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_cabc_seq);
}

static int lcd_panel_sharp_cabc_gradient(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_cabc_gradient);
}

static int lcd_panel_sharp_set_ce_mode(struct lcd_panel_info *lcd)
{
	if (lcd->id_code[ID_CODE2] == 0x00) return 0;
	switch (lcd->ce_mode) {
	case 0xff:
		return 0;
	case 0:
		return write_to_lcd(lcd, sharp_sat_low_lit_low);
	break;
	case 1:
		return write_to_lcd(lcd, sharp_sat_low_lit_med);
	break;
	case 2:
		return write_to_lcd(lcd, sharp_sat_low_lit_high);
	break;
	case 3:
		return write_to_lcd(lcd, sharp_sat_med_lit_low);
	break;
	case 4:
		return write_to_lcd(lcd, sharp_sat_med_lit_med);
	break;
	case 5:
		return write_to_lcd(lcd, sharp_sat_med_lit_high);
	break;
	case 6:
		return write_to_lcd(lcd, sharp_sat_high_lit_low);
	break;
	case 7:
		return write_to_lcd(lcd, sharp_sat_high_lit_low);
	break;
	case 8:
		return write_to_lcd(lcd, sharp_sat_high_lit_high);
	break;
	case 9:
		return write_to_lcd(lcd, sharp_sat_low);
	break;
	case 10:
		return write_to_lcd(lcd, sharp_sat_med);
	break;
	case 11:
		return write_to_lcd(lcd, sharp_sat_high);
	break;
	case 12:
		return write_to_lcd(lcd, sharp_ce_off);
	default:
		return 0;
	break;
	}
}

static int lcd_read_id(struct mipi_dsim_lcd_device *mipi_dev)
{
	struct lcd_panel_info *lcd = dev_get_drvdata(&mipi_dev->dev);
	int i = 0;
	unsigned int id_code = 0;

	for (i = 0; i < ID_CODEMAX; i++) {
		write_to_lcd(lcd, sharp_unlock); /*set password for ROnly*/
		set_packet_size(lcd, 1); /* set return data size*/
		id_code = read_data(lcd, 0xda + i); /*read ID Code reg 0xda*/
		if (id_code > 0xff)
			lcd->id_code[i] = 0x18;
		else
			lcd->id_code[i] = id_code;
		lcd_id[i] = lcd->id_code[i];
		pr_info("id code 0X%x value 0x%x\n", 0xDA+i, lcd_id[i]);
	}
	if ((lcd->id_code[ID_CODE1] & 0x10))
		lcd->panel_manu = 1;
	else
		lcd->panel_manu = 0;
	return lcd->panel_manu;
}

static int lcd_init(struct mipi_dsim_lcd_device *mipi_dev)
{
	struct lcd_panel_info *lcd = dev_get_drvdata(&mipi_dev->dev);
	
	if ((lcd->id_code[ID_CODE1] & 0x10)) {
		/*JDI init*/
		CHECK_PANEL_RET(lcd_panel_jdi_sleep_out(lcd));
		CHECK_PANEL_RET(lcd_panel_jdi_gamma_seq(lcd));
		CHECK_PANEL_RET(lcd_panel_jdi_display_on(lcd));
	} else {
		/*Sharp init*/
		CHECK_PANEL_RET(lcd_panel_sharp_sleep_out(lcd));
		CHECK_PANEL_RET(lcd_panel_sharp_init_code(lcd));
		CHECK_PANEL_RET(lcd_panel_sharp_display_on(lcd));
		CHECK_PANEL_RET(lcd_panel_sharp_cabc_seq_prepare(lcd));
		CHECK_PANEL_RET(lcd_panel_sharp_cabc_switch(lcd, lcd->cabc_en));
		CHECK_PANEL_RET(lcd_panel_sharp_cabc_gradient(lcd));
		CHECK_PANEL_RET(lcd_panel_sharp_set_ce_mode(lcd));
	} 
	
	return 0;
}

static int lcd_remove(struct mipi_dsim_lcd_device *mipi_dev)
{
	struct lcd_panel_info *lcd = NULL;
	struct lcd_platform_data	*ddi_pd;

	lcd = (struct lcd_panel_info *)dev_get_drvdata(&mipi_dev->dev);
	ddi_pd = lcd->ddi_pd;

	if (ddi_pd->power_on)
		ddi_pd->power_on(lcd->ld, false, lcd->panel_manu);

	kfree(lcd);

	return dev_set_drvdata(&mipi_dev->dev, NULL);
}
int lcd_cabc_opr(unsigned int brightness, unsigned int enable)
{
	CHECK_LCD_NULL(); 
	if ((g_lcd_info->id_code[ID_CODE1] & 0x10)) {
		pr_info("JDI panel.");
		return -1;
	} else {
		pr_info("Sharp panel.");
		CHECK_PANEL_RET(lcd_panel_sharp_cabc_switch(g_lcd_info, enable));
	}
	g_lcd_info->cabc_en = enable;
	return 0;
}
EXPORT_SYMBOL_GPL(lcd_cabc_opr);

#ifdef LCD_TEST
int lcd_cabc_set_brightness(unsigned int brightness)
{
	CHECK_LCD_NULL(); 
	CHECK_PANEL_RET(lcd_panel_set_brightness(g_lcd_info , brightness));
	return 0;
}
EXPORT_SYMBOL_GPL(lcd_cabc_set_brightness);

static ssize_t lcd_sync_enable(struct device *dev, struct device_attribute
					*attr, const char *buf, size_t size)
{
	struct lcd_panel_info *lcd = dev_get_drvdata(dev);
	int num;

	sscanf(buf, "%d", &num);
	pr_info("cabc %s\n", num == 0 ? "vsync out" : "hsync out");

	if (num == 0) {
		CHECK_PANEL_RET(lcd_panel_vsync_out(lcd));
	} else {
		CHECK_PANEL_RET(lcd_panel_hsync_out(lcd));
	}

	return sizeof(num);
}
static DEVICE_ATTR(sync, 0644, NULL, lcd_sync_enable);

static ssize_t lcd_set_brt(struct device *dev, struct device_attribute
					*attr, const char *buf, size_t size)
{
	struct lcd_panel_info *lcd = dev_get_drvdata(dev);
	int brt;

	sscanf(buf, "%d", &brt);
	pr_info("brightness set %d\n", brt);
	CHECK_PANEL_RET(lcd_panel_set_brightness(lcd, brt));

	return sizeof(brt);
}
static DEVICE_ATTR(brt, 0644, NULL, lcd_set_brt);

static ssize_t lcd_set_ce(struct device *dev, struct device_attribute
					*attr, const char *buf, size_t size)
{
	struct lcd_panel_info *lcd = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(lcd_mode_map); i++) {
		if (sysfs_streq(buf, lcd_mode_map[i].mode)) {
			lcd->ce_mode = lcd_mode_map[i].mode_val;
			break;
		} else {
			lcd->ce_mode = 0xff;
		}
	}
	if ((g_lcd_info->id_code[ID_CODE1] & 0x10)) {
		pr_info("JDI panel.");
	} else {
		CHECK_PANEL_RET(lcd_panel_sharp_set_ce_mode(lcd));
	}
	pr_info("set mode %d name %s\n", lcd->ce_mode, buf);
	pr_info("please reset the LCD\n");

	return size;
}
static ssize_t lcd_get_ce(struct device *dev, struct device_attribute
					*attr, char *buf)
{
	struct lcd_panel_info *lcd = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(lcd_mode_map); i++) {
		if (lcd->ce_mode == lcd_mode_map[i].mode_val) {
			return sprintf(buf, "the ce mode is %s\n", lcd_mode_map[i].mode);
		}
	}
	return sprintf(buf, "you don't set ce mode\n");
}

static DEVICE_ATTR(ce, 0644, lcd_get_ce, lcd_set_ce);

#endif

static int lcd_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct lcd_panel_info *lcd = NULL;
	int err = 0;

	lcd = kzalloc(sizeof(struct lcd_panel_info), GFP_KERNEL);
	if (!lcd) {
		dev_err(&dsim_dev->dev, "failed to allocate ls044k3sx01 structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->ddi_pd = (struct lcd_platform_data *)dsim_dev->platform_data;

	if (IS_ERR_OR_NULL(lcd->ddi_pd))
		pr_err("%s: ddi_pd is NULL\n", __func__);

	lcd->dev = &dsim_dev->dev;

	dev_set_drvdata(&dsim_dev->dev, lcd);

	lcd->ld = lcd_device_register("lcd_panel", lcd->dev, lcd,
				NULL);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		goto err_dev_register;
	}

	lcd->state = LCD_DISPLAY_POWER_OFF;
	lcd->ce_mode = 0xff;
	lcd->cabc_en = false;

#ifdef LCD_TEST
	err = device_create_file(lcd->dev, &dev_attr_sync);
	if (err < 0) {
		dev_err(lcd->dev, "Failed to create attr file cabc %d!\n", err);
	}

	err = device_create_file(lcd->dev, &dev_attr_brt);
	if (err < 0) {
		dev_err(lcd->dev, "Failed to create attr file cabc %d!\n", err);
	}

	err = device_create_file(lcd->dev, &dev_attr_ce);
	if (err < 0) {
		dev_err(lcd->dev, "Failed to create attr file cabc %d!\n", err);
	}
#endif

	g_lcd_info = lcd;

	pr_info("lcd_probe finish\n");
	return err;

err_dev_register:
	kfree(lcd);
	return -1;
}
static void lcd_shutdown(struct mipi_dsim_lcd_device *mipi_dev)
{
	struct lcd_panel_info *lcd = dev_get_drvdata(&mipi_dev->dev);

	if (lcd->state == LCD_DISPLAY_POWER_OFF)
		return;
	lcd->state = LCD_DISPLAY_POWER_OFF;

	/* lcd power off */
	if (lcd->ddi_pd->power_on)
		lcd->ddi_pd->power_on(lcd->ld, false, lcd->panel_manu);
}

static int lcd_suspend(struct mipi_dsim_lcd_device *mipi_dev)
{
	struct lcd_panel_info *lcd = dev_get_drvdata(&mipi_dev->dev);

	if (lcd->state == LCD_DISPLAY_POWER_OFF)
		return 0;

	if ((g_lcd_info->id_code[ID_CODE1] & 0x10)) {
		CHECK_PANEL_RET(lcd_panel_jdi_display_off(lcd));
		CHECK_PANEL_RET(lcd_panel_jdi_sleep_in(lcd));
	} else {
		CHECK_PANEL_RET(lcd_panel_sharp_display_off(lcd));
		CHECK_PANEL_RET(lcd_panel_sharp_sleep_in(lcd));
	}
	lcd->state = LCD_DISPLAY_SLEEP_IN;

	return 0;
}

static int lcd_resume(struct mipi_dsim_lcd_device *mipi_dev)
{
	struct lcd_panel_info *lcd = dev_get_drvdata(&mipi_dev->dev);

	/* lcd power on */
	if (lcd->ddi_pd->power_on)
		lcd->ddi_pd->power_on(lcd->ld, true, lcd->panel_manu);

	lcd->state = LCD_DISPLAY_NORMAL;
	pr_debug("%s: lcd->state = %d\n", __func__, lcd->state);

	return 0;
}

static struct mipi_dsim_lcd_driver lcd_mipi_driver = {
	.name	= "lcd_panel",
	.id		= -1,
	.probe	= lcd_probe,
	.init_lcd	= lcd_init,
	.suspend	= lcd_suspend,
	.resume	= lcd_resume,
	.shutdown = lcd_shutdown,
	.remove	= lcd_remove,
	.read_id = lcd_read_id,
};

static int __init lcd_panel_init(void)
{
	return s5p_mipi_dsi_register_lcd_driver(&lcd_mipi_driver);
}

arch_initcall(lcd_panel_init);
