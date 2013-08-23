#ifndef __GPIO_COMMON_H
#define __GPIO_COMMON_H
#include <sound/soc.h>
#include <linux/gpio.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>

#define MX_FACTORY_TEST_BT				0
#define MX_FACTORY_TEST_CAMERA			1
#define MX_FACTORY_TEST_ALL				2


#define GPIO_SETPIN_LOW 	0
#define GPIO_SETPIN_HI   	1
#define GPIO_SETPIN_NONE	2

#define GPIO_PULL_NONE	S3C_GPIO_PULL_NONE
#define GPIO_PULL_DOWN	S3C_GPIO_PULL_DOWN
#define GPIO_PULL_UP		S3C_GPIO_PULL_UP

#define GPIO_DRVSTR_LV1 	S5P_GPIO_DRVSTR_LV1
#define GPIO_DRVSTR_LV2	S5P_GPIO_DRVSTR_LV2
#define GPIO_DRVSTR_LV3	S5P_GPIO_DRVSTR_LV3
#define GPIO_DRVSTR_LV4 	S5P_GPIO_DRVSTR_LV4

#define GPIO_SLP_OUT0       ((__force s3c_gpio_pull_t)0x00)
#define GPIO_SLP_OUT1       ((__force s3c_gpio_pull_t)0x01)
#define GPIO_SLP_INPUT      ((__force s3c_gpio_pull_t)0x02)
#define GPIO_SLP_PREV       ((__force s3c_gpio_pull_t)0x03)

struct gpio_info_table{
	unsigned int pin;
	unsigned int type;
	unsigned int data;
	unsigned int pull;
	unsigned int drv;
};

struct gpio_slp_info{
	unsigned char *name;
	unsigned int pin;
	unsigned int type;
	unsigned int pull;
};


extern void (*exynos4_sleep_gpio_set)(void);
extern  int (*mx_is_factory_test_mode)(int type);
extern  int (*mx_set_factory_test_led)(int on);
#ifdef	CONFIG_WM8958_ALWAYS_ON_SUSPEND	
extern int wm8994_isalwayson(struct snd_soc_codec *codec);	
#endif
#ifdef CONFIG_SND_SOC_WM8994
extern int wm8994_get_playback_path(struct snd_soc_codec *codec);
#endif
int mx_config_gpio_table(const struct gpio_info_table *gpio_table, int array_size);
void mx_set_sleep_pin(unsigned int pin, s5p_gpio_pd_cfg_t conpdn, s5p_gpio_pd_pull_t pudpdn);
#endif //__GPIO_COMMON_H
