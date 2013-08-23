/**
 * arch/arm/mach-exynos/board-m03x-modems.c
 *
 * Copyright (C) 2010 Samsung Electronics.
 * Copyright (C) 2012 Zhuhai Meizu Inc.
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
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/platform_data/modem.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <asm/mach-types.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-m040.h>
#include <mach/modem.h>

static struct modem_io_t m040_io_devices[] = {
	[0] = {
		.id      = 0x0,                    
		.name    = "ttyACM0",              
		.links   = LINKTYPE(LINKDEV_HSIC), 
		.format  = IPC_RAW,                
		.io_type = IODEV_TTY,              
	},
	[1] = {
		.id      = 0x1,                    
		.name    = "rmnet0",
		.links   = LINKTYPE(LINKDEV_HSIC), 
		.format  = IPC_RAW,                
		.io_type = IODEV_NET,
	},
	[2] = {
		.id      = 0x2,                    
		.name    = "ttyACM1",
		.links   = LINKTYPE(LINKDEV_HSIC), 
		.format  = IPC_RAW,                
		.io_type = IODEV_TTY,
	},
	[3] = {
		.id      = 0x3,                    
		.name    = "ttyACM2",
		.links   = LINKTYPE(LINKDEV_HSIC), 
		.format  = IPC_RAW,                
		.io_type = IODEV_TTY,
	},
};

static struct resource umts_modem_res[] = {
	[0] = {
		.name  = "link_pm_hostwake",
		.start = IRQ_EINT16_31,
		.end   = IRQ_EINT16_31,
		.flags = IORESOURCE_IRQ,
	},
};

static struct modemlink_pm_data modem_link_pm_data = {
	.name = "link_pm",
	.gpio_link_enable    = 0,
	.gpio_hostwake  = M040_GPIO_HOST_WAKEUP,
	.gpio_slavewake = M040_GPIO_SLAVE_WAKEUP,
};

static struct modem_data umts_modem_data;

static struct modem_data umts_modem_data_m040 = {
	.name                     = "xmm6260",
	.gpio_cp_on               = M040_GPIO_MODEM_ON,
	.gpio_reset_req_n         = M040_GPIO_MODEM_RST_FULL,
	.gpio_cp_reset            = M040_GPIO_MODEM_RST,
	.gpio_host_active         = M040_GPIO_HOST_ACTIVE,
	.gpio_cp_reset_int        = M040_GPIO_MODEM_RESET_INT,
	.gpio_cp_dump_int         = M040_GPIO_MODEM_DUMP_INT,
	.gpio_sim_detect          = 0,                      
	.gpio_hostwake            = M040_GPIO_HOST_WAKEUP,  
	.gpio_slavewake           = M040_GPIO_SLAVE_WAKEUP, 
	.modem_type               = IMC_XMM6260,
	.link_types               = LINKTYPE(LINKDEV_HSIC),
	.modem_net                = UMTS_NETWORK,
	.num_iodevs               = ARRAY_SIZE(m040_io_devices),
	.iodevs                   = m040_io_devices,
	.link_pm_data             = &modem_link_pm_data,
	.gpio_revers_bias_clear   = NULL,
	.gpio_revers_bias_restore = NULL,
};

void modem_set_active_state(int state)
{
	gpio_direction_output(umts_modem_data.gpio_host_active, state);
}

static struct platform_device umts_modem_m040 = {
	.name = "modem_ifx_6260",
	.id = -1,
	.num_resources = ARRAY_SIZE(umts_modem_res),
	.resource = umts_modem_res,
	.dev = {
		.platform_data = &umts_modem_data_m040,
	},
};

static void umts_modem_cfg_gpio(void)
{
	int err = 0;
	unsigned gpio_cp_on        = umts_modem_data.gpio_cp_on;
	unsigned gpio_cp_rst       = umts_modem_data.gpio_cp_reset;
	unsigned gpio_host_active  = umts_modem_data.gpio_host_active;
	unsigned gpio_sim_detect   = umts_modem_data.gpio_sim_detect;
	unsigned gpio_reset_req_n  = umts_modem_data.gpio_reset_req_n;
	unsigned gpio_cp_reset_int = umts_modem_data.gpio_cp_reset_int;

	if (gpio_reset_req_n) {
		err = gpio_request(gpio_reset_req_n, "RESET_REQ_N");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "RESET_REQ_N", err);
		}
		gpio_direction_output(gpio_reset_req_n, 0);
		s3c_gpio_setpull(gpio_reset_req_n, S3C_GPIO_PULL_NONE);
	}

	if (gpio_cp_on) {
		err = gpio_request(gpio_cp_on, "CP_ON");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "CP_ON", err);
		}
		gpio_direction_output(gpio_cp_on, 0);
		s3c_gpio_setpull(gpio_cp_on, S3C_GPIO_PULL_NONE);
	}

	if (gpio_cp_rst) {
		err = gpio_request(gpio_cp_rst, "CP_RST");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "CP_RST", err);
		}
		gpio_direction_output(gpio_cp_rst, 0);
		s3c_gpio_setpull(gpio_cp_rst, S3C_GPIO_PULL_NONE);
	}

	if (gpio_cp_reset_int) {
		err = gpio_request(gpio_cp_reset_int, "GPIO_CP_RESET_INT");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "GPIO_CP_RESET_INT", err);
		}
		s3c_gpio_setpull(gpio_cp_reset_int, S3C_GPIO_PULL_UP);
	}

	if (gpio_host_active) {
		err = gpio_request(gpio_host_active, "HOST_ACTIVE");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "HOST_ACTIVE", err);
		}
		gpio_direction_output(gpio_host_active, 0);
	}

	if (gpio_sim_detect) {
		err = gpio_request(gpio_sim_detect, "SIM_DETECT");
		if (err)
			printk(KERN_ERR "fail to request gpio %s: %d\n",
				"SIM_DETECT", err);
		s3c_gpio_cfgpin(gpio_sim_detect, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_sim_detect, S3C_GPIO_PULL_NONE);
		irq_set_irq_type(gpio_to_irq(gpio_sim_detect),
							IRQ_TYPE_EDGE_BOTH);
	}

	printk(KERN_INFO "umts_modem_cfg_gpio done\n");
}

static void modem_hsic_pm_config_gpio(void)
{
	int err = 0;
	unsigned gpio_link_enable    = modem_link_pm_data.gpio_link_enable;
	unsigned gpio_hostwake  = modem_link_pm_data.gpio_hostwake;
	unsigned gpio_slavewake = modem_link_pm_data.gpio_slavewake;

	if (gpio_link_enable) {
		err = gpio_request(gpio_link_enable, "LINK_EN");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "LINK_EN", err);
		}
		gpio_direction_output(gpio_link_enable, 0);
		gpio_free(gpio_link_enable);
	}

	if (gpio_hostwake) {
		err = gpio_request(gpio_hostwake, "HOSTWAKE");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "HOSTWAKE", err);
		}
		gpio_direction_input(gpio_hostwake);
		s3c_gpio_cfgpin(gpio_hostwake, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_hostwake, S3C_GPIO_PULL_NONE);
	}

	if (gpio_slavewake) {
		err = gpio_request(gpio_slavewake, "SLAVEWAKE");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "SLAVEWAKE", err);
		}
		gpio_direction_output(gpio_slavewake, 0);
		s3c_gpio_setpull(gpio_slavewake, S3C_GPIO_PULL_NONE);
	}

	if (gpio_hostwake)
		irq_set_irq_type(gpio_to_irq(gpio_hostwake),
				IRQ_TYPE_EDGE_BOTH);

	printk(KERN_INFO "modem_hsic_pm_config_gpio done\n");
}

int m040_modem_device_init(void)
{
	int ret = 0;

	pr_info("[MODEM_IF] init_modem for wm\n");
	umts_modem_data = umts_modem_data_m040;

	umts_modem_cfg_gpio();
	modem_hsic_pm_config_gpio();

	ret = platform_device_register(&umts_modem_m040);

	pr_info("[MODEM_IF] init_modem device over, ret=%d.\n", ret);

	return ret;

}
