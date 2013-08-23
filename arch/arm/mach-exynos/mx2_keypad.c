#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#ifdef CONFIG_MFD_MXQM
#include <linux/mx_qm.h>
#endif
#include <asm/mach-types.h>
#include <mach/map.h>
#include <mach/gpio-m040.h>
#include <mach/i2c-m040.h>

#if defined(CONFIG_MFD_MXQM)
/*mxqm*/
static struct mx_qm_platform_data __initdata mxqm_pd = {
	.gpio_wake 	= M040_TOUCHPAD_WAKE,
	.gpio_reset 	= M040_TOUCHPAD_RESET,
	.gpio_irq 	= M040_TOUCHPAD_INT,
};
#endif

static struct i2c_board_info __initdata i2c_devs_touchpad[] = {
#if defined(CONFIG_MFD_MXQM)
	{
		I2C_BOARD_INFO("mx_qm", 0x43),
		.platform_data	= &mxqm_pd,
		.irq = IRQ_EINT(8),
	},
#endif
};

static struct platform_device __initdata *m040_keypad_devices[]  = {
	&m040_device_gpio_i2c16,
};
static struct platform_device __initdata *m041_keypad_devices[]  = {
	&m041_device_gpio_i2c16,
};
static int  __init mx2_init_keypad(void)
{
#ifdef CONFIG_MX2_SC8803G_TEST
	i2c_register_board_info(16, i2c_devs_touchpad, ARRAY_SIZE(i2c_devs_touchpad));
	if(platform_add_devices(m040_keypad_devices, ARRAY_SIZE(m040_keypad_devices)))
		pr_err("%s: register touchpad device fail\n", __func__);
#else
#ifdef CONFIG_MACH_M040
	/*MX qm */
	if(machine_is_m040()){
		i2c_register_board_info(16, i2c_devs_touchpad, ARRAY_SIZE(i2c_devs_touchpad));
		if(platform_add_devices(m040_keypad_devices, ARRAY_SIZE(m040_keypad_devices)))
			pr_err("%s: register touchpad device fail\n", __func__);
	}
#endif
#ifdef CONFIG_MACH_M041
	if(machine_is_m041()){
		i2c_register_board_info(16, i2c_devs_touchpad, ARRAY_SIZE(i2c_devs_touchpad));
		if(platform_add_devices(m041_keypad_devices, ARRAY_SIZE(m041_keypad_devices)))
			pr_err("%s: register touchpad device fail\n", __func__);
	}
#endif
#endif
	return 0;
}

arch_initcall(mx2_init_keypad);

