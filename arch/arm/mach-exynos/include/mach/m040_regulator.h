/* linux/arch/arm/mach-exynos/m040_regulator.c
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *		http://www.meizu.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#if defined(CONFIG_REGULATOR_MAX77686)
/* max77686 */
static struct regulator_consumer_supply __initdata max77686_buck1 =
	REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply __initdata max77686_buck2[] = {
	REGULATOR_SUPPLY("vdd_arm", "exynos4212-cpufreq"),
	REGULATOR_SUPPLY("vdd_arm", "exynos4412-cpufreq"),
};

static struct regulator_consumer_supply __initdata max77686_buck3 =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply __initdata max77686_buck4 =
	REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_consumer_supply __initdata max77686_buck5 =
	REGULATOR_SUPPLY("vdd_mem", NULL);

static struct regulator_consumer_supply __initdata max77686_buck6 =
	REGULATOR_SUPPLY("vdd_1.35v", NULL);

static struct regulator_consumer_supply __initdata max77686_buck7 =
	REGULATOR_SUPPLY("vdd_2.00v", NULL);

static struct regulator_consumer_supply __initdata max77686_buck8 =
	REGULATOR_SUPPLY("vdd_2.80v", NULL);

/*isp core 1.2v, it's control by fixed regulator "cam_isp_1.2v" */
static struct regulator_consumer_supply __initdata max77686_buck9 =
	REGULATOR_SUPPLY("cam_isp", NULL);

/* alive 1.0v */
static struct regulator_consumer_supply __initdata max77686_ldo1 =
	REGULATOR_SUPPLY("vdd_ldo1", NULL);

/* memoff 1.2v */
static struct regulator_consumer_supply __initdata max77686_ldo2 =
	REGULATOR_SUPPLY("vdd_ldo2", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo3 =
	REGULATOR_SUPPLY("vdd_ldo3", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo4 =
	REGULATOR_SUPPLY("vmmc", "dw_mmc");

/*isp io 1.8v*/
static struct regulator_consumer_supply __initdata max77686_ldo5 =
	REGULATOR_SUPPLY("cam_isp_1.8v", NULL);

/* mpll 1.0v */
static struct regulator_consumer_supply __initdata max77686_ldo6 =
	REGULATOR_SUPPLY("vdd_ldo6", NULL);

/* vpll 1.0v */
static struct regulator_consumer_supply __initdata max77686_ldo7 =
	REGULATOR_SUPPLY("vdd_ldo7", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo8 = /*mipi 1.0v */
	REGULATOR_SUPPLY("vdd_ldo8", "s5p-mipi-dsim.0");

/* front camera core 1.5v */
static struct regulator_consumer_supply __initdata max77686_ldo9 =
	REGULATOR_SUPPLY("cam_front_1.5v", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo10 = /*mipi 1.8v */
	REGULATOR_SUPPLY("vdd_ldo10", "s5p-mipi-dsim.0");

static struct regulator_consumer_supply __initdata max77686_ldo11 =
	REGULATOR_SUPPLY("vdd_ldo11", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo12[] = {
	REGULATOR_SUPPLY("vdd_ldo12", "s3c-usbgadget"),
	REGULATOR_SUPPLY("vdd_ldo12", "s5p-ehci"),
};

static struct regulator_consumer_supply __initdata max77686_ldo13 = /*lcd 1.8v */
	REGULATOR_SUPPLY("vdd_ldo13", "lcd_panel");

static struct regulator_consumer_supply __initdata max77686_ldo14 =
	REGULATOR_SUPPLY("vdd_ldo14", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo15[] = {
	REGULATOR_SUPPLY("vdd_ldo15", "s3c-usbgadget"),
	REGULATOR_SUPPLY("vdd_ldo15", "s5p-ehci"),
};

static struct regulator_consumer_supply __initdata max77686_ldo16[] ={
	REGULATOR_SUPPLY("vdd_ldo16", "s3c-usbgadget"),
	REGULATOR_SUPPLY("vdd_ldo16", "s5p-ehci"),
};

/*back camera core 1.2v*/
static struct regulator_consumer_supply __initdata max77686_ldo17 = 
	REGULATOR_SUPPLY("cam_back_1.2v", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo18 =
	REGULATOR_SUPPLY("vdd_ldo18", NULL);

/* hdmi 1.8v */
static struct regulator_consumer_supply __initdata max77686_ldo19 =
	REGULATOR_SUPPLY("vdd_ldo19", NULL);

/* mhl 1.2v */
static struct regulator_consumer_supply __initdata max77686_ldo20 =
	REGULATOR_SUPPLY("vdd_ldo20", NULL);

/* front camera analog 2.8v*/
static struct regulator_consumer_supply __initdata max77686_ldo21 =
	REGULATOR_SUPPLY("cam_front_2.8v", NULL);

/* iNAND IO 2.8v */
static struct regulator_consumer_supply __initdata max77686_ldo22 =
	REGULATOR_SUPPLY("inand_io2.8v", NULL);

/* back camera sensor anolog 2.7v*/
static struct regulator_consumer_supply __initdata max77686_ldo23 =
	REGULATOR_SUPPLY("cam_back_2.7v", NULL);

/* back camera af 2.7v */
static struct regulator_consumer_supply __initdata max77686_ldo24 =
	REGULATOR_SUPPLY("cam_back_af_2.7v", NULL);

static struct regulator_consumer_supply __initdata max77686_ldo25 =
	REGULATOR_SUPPLY("vdd_ldo25", NULL);

/* MHL 3.0v */
static struct regulator_consumer_supply __initdata max77686_ldo26 =
	REGULATOR_SUPPLY("vdd_ldo26", NULL);

static struct regulator_consumer_supply max77686_en32khcp =
	REGULATOR_SUPPLY("lpo-in", "bcm47511");

static struct regulator_consumer_supply max77686_enp32khz =
	REGULATOR_SUPPLY("lpo", NULL);

static struct regulator_init_data __initdata max77686_buck1_data = {
	.constraints = {
		.name 		= "vdd_mif range",
		.min_uV 	= 850000,
		.max_uV 	= 1100000,
		.always_on 	= true,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_buck1,
};

static struct regulator_init_data __initdata max77686_buck2_data = {
	.constraints = {
		.name 		= "vdd_arm range",
		.min_uV 	= 600000,
		.max_uV 	= 1400000,
		.always_on 	= true,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.disabled 	= true,
		},
	},
	.num_consumer_supplies 	= 2,
	.consumer_supplies 	= max77686_buck2,
};

static struct regulator_init_data __initdata max77686_buck3_data = {
	.constraints = {
		.name 		= "vdd_int range",
		.min_uV 	= 850000,
		.max_uV 	= 1100000,
		.always_on 	= true,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.disabled 	= true,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_buck3,
};

static struct regulator_init_data __initdata max77686_buck4_data = {
	.constraints = {
		.name 		= "vdd_g3d range",
		.min_uV 	= 850000,
		.max_uV 	= 1200000,
		.boot_on 	= true,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.state_mem = {
			.disabled 	= true,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_buck4,
};

static struct regulator_init_data __initdata max77686_buck5_data = {
	.constraints = {
		.name 		= "vdd_mem_on",
		.min_uV 	= 1200000,
		.max_uV 	= 1200000,
		.boot_on 	= true,
		.always_on 	= true,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_buck5,
};

static struct regulator_init_data __initdata max77686_buck6_data = {
	.constraints = {
		.name 		= "vdd_1.35v",
		.min_uV 	= 1350000,
		.max_uV 	= 1350000,
		.boot_on 	= true,
		.always_on 	= true,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_buck6,
};

static struct regulator_init_data __initdata max77686_buck7_data = {
	.constraints = {
		.name 		= "vdd_2.00v",
		.min_uV 	= 2000000,
		.max_uV 	= 2000000,
		.boot_on 	= true,
		.always_on 	= true,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_buck7,
};

static struct regulator_init_data __initdata max77686_buck8_data = {
	.constraints = {
		.name 		= "vdd_2.80v",
		.min_uV 	= 2800000,
		.max_uV 	= 2800000,
		.boot_on 	= true,
		.always_on 	= true,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_buck8,
};

static struct regulator_init_data __initdata max77686_buck9_data = {
	.constraints	= {
		.name		= "CAM_ISP_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.boot_on 	= true,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_buck9,
};

static struct regulator_init_data __initdata max77686_ldo1_data = {
	.constraints = {
		.name 		= "vdd_ldo1 range",
		.min_uV 	= 1000000,
		.max_uV 	= 1000000,
		.apply_uV	= true,
		.always_on	= true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_ldo1,
};

static struct regulator_init_data __initdata max77686_ldo2_data = {
	.constraints = {
		.name 		= "vdd_ldo2 range",
		.min_uV 	= 1200000,
		.max_uV 	= 1200000,
		.apply_uV	= true,
		.always_on 	= true,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_ldo2,
};

static struct regulator_init_data __initdata max77686_ldo3_data = {
	.constraints = {
		.name 		= "vdd_ldo3 range",
		.min_uV 	= 1800000,
		.max_uV 	= 1800000,
		.apply_uV	= true,
		.always_on	= true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_ldo3,
};

static struct regulator_init_data __initdata max77686_ldo4_data = {
	.constraints = {
		.name 		= "vdd_ldo4 range",
		.min_uV 	= 2800000,
		.max_uV 	= 2800000,
		.apply_uV	= true,
		.always_on	= true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_ldo4,
};

static struct regulator_init_data __initdata max77686_ldo5_data = {
	.constraints = {
		.name 		= "CAM_ISP_1.8V",
		.min_uV 	= 1800000,
		.max_uV 	= 1800000,
		.apply_uV 	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled 	= true,
		}
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies	= &max77686_ldo5,
};

static struct regulator_init_data __initdata max77686_ldo6_data = {
	.constraints = {
		.name 		= "vdd_ldo6 range",
		.min_uV 	= 1000000,
		.max_uV 	= 1000000,
		.apply_uV	= true,
		.always_on 	= true,
		.state_mem = {
			.disabled 	= true,
		}
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_ldo6,
};

static struct regulator_init_data __initdata max77686_ldo7_data = {
	.constraints = {
		.name 		= "vdd_ldo7 range",
		.min_uV 	= 1000000,
		.max_uV 	= 1000000,
		.apply_uV	= true,
		.always_on 	= true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_ldo7,
};

static struct regulator_init_data __initdata max77686_ldo8_data = {
	.constraints	= {
		.name		= "vdd_ldo8 range",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= true,
		.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo8,
};

static struct regulator_init_data __initdata max77686_ldo9_data = {
	.constraints = {
		.name		= "CAM_FRONT_1.5V",
		.min_uV 	= 1500000,
		.max_uV 	= 1500000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		}
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_ldo9,
};

static struct regulator_init_data __initdata max77686_ldo10_data = {
	.constraints	= {
		.name		= "vdd_ldo10 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= true,
		.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo10,
};

static struct regulator_init_data __initdata max77686_ldo11_data = {
	.constraints	= {
		.name		= "vdd_ldo11 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= true,
		.always_on	= true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo11,
};

static struct regulator_init_data __initdata max77686_ldo12_data = {
	.constraints	= {
		.name		= "vdd_ldo12 range",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= true,
		.always_on	= true,
		.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= 2,
	.consumer_supplies	= max77686_ldo12,
};

static struct regulator_init_data __initdata max77686_ldo13_data = {
	.constraints	= {
		.name		= "vdd_ldo13 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= true,
		.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo13,
};

static struct regulator_init_data __initdata max77686_ldo14_data = {
	.constraints	= {
		.name		= "vdd_ldo14 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= true,
		.always_on	= true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo14,
};

static struct regulator_init_data __initdata max77686_ldo15_data = {
	.constraints	= {
		.name		= "vdd_ldo15 range",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= true,
		.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= 2,
	.consumer_supplies	= max77686_ldo15,
};

static struct regulator_init_data __initdata max77686_ldo16_data = {
	.constraints	= {
		.name		= "vdd_ldo16 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= true,
		.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= 2,
	.consumer_supplies	= max77686_ldo16,
};

static struct regulator_init_data __initdata max77686_ldo17_data = {
	.constraints = {
		.name		= "CAM_BACK_1.2V",
		.min_uV 	= 1200000,
		.max_uV 	= 1200000,
		.apply_uV	= true,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_ldo17,
};

static struct regulator_init_data __initdata max77686_ldo18_data = {
	.constraints	= {
		.name		= "vdd_ldo18 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= true,
		.always_on	= true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo18,
};

static struct regulator_init_data __initdata max77686_ldo19_data = {
	.constraints	= {
		.name		= "vdd_ldo19 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= true,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo19,
};

static struct regulator_init_data __initdata max77686_ldo20_data = {
	.constraints	= {
		.name		= "vdd_ldo20 range",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= true,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo20,
};

static struct regulator_init_data __initdata max77686_ldo21_data = {
	.constraints = {
		.name		= "CAM_FRONT_2.8V",
		.min_uV 	= 2800000,
		.max_uV 	= 2800000,
		.apply_uV	= true,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		}
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_ldo21,
};

static struct regulator_init_data __initdata max77686_ldo22_data = {
	.constraints	= {
		.name		= "vdd_ldo22 range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= true,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo22,
};

static struct regulator_init_data __initdata max77686_ldo23_data = {
	.constraints = {
		.name		= "CAM_BACK_2.7V",
		.min_uV 	= 2700000,
		.max_uV 	= 2700000,
		.apply_uV	= true,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		}
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_ldo23,
};

static struct regulator_init_data __initdata max77686_ldo24_data = {
	.constraints = {
		.name		= "CAM_BACK_AF_2.7V",
		.min_uV 	= 2700000,
		.max_uV 	= 2700000,
		.apply_uV	= true,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		}
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_ldo24,
};

static struct regulator_init_data __initdata max77686_ldo25_data = {
	.constraints	= {
		.name		= "vdd_ldo25 range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= true,
		.always_on	= true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo25,
};

static struct regulator_init_data __initdata max77686_ldo26_data = {
	.constraints	= {
		.name		= "vdd_ldo26 range",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= true,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77686_ldo26,
};

static struct regulator_init_data max77686_enp32khz_data = {
	.constraints = {
		.name 		= "32KHZ_PMIC",
		.always_on 	= true,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= true,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_enp32khz,
};

static struct regulator_init_data max77686_en32khcp_data = {
	.constraints = {
		.name 		= "32KHZ_GPS",
		.always_on 	= true,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= true,
		},
	},
	.num_consumer_supplies 	= 1,
	.consumer_supplies 	= &max77686_en32khcp,
};

static struct max77686_regulator_data __initdata max77686_regulators[] = {
	/* System important power supply */
	{MAX77686_BUCK1, 	&max77686_buck1_data},
	{MAX77686_BUCK2, 	&max77686_buck2_data},
	{MAX77686_BUCK3, 	&max77686_buck3_data},
	{MAX77686_BUCK4, 	&max77686_buck4_data},
	{MAX77686_BUCK5, 	&max77686_buck5_data},
	{MAX77686_BUCK6, 	&max77686_buck6_data},
	{MAX77686_BUCK7, 	&max77686_buck7_data},
	{MAX77686_BUCK8, 	&max77686_buck8_data},
	{MAX77686_LDO1,		&max77686_ldo1_data},
	{MAX77686_LDO2,		&max77686_ldo2_data},
	{MAX77686_LDO3,		&max77686_ldo3_data},
	{MAX77686_LDO4,		&max77686_ldo4_data},
	{MAX77686_LDO6,		&max77686_ldo6_data},
	{MAX77686_LDO7,		&max77686_ldo7_data},
	{MAX77686_LDO11,	&max77686_ldo11_data},
	{MAX77686_LDO14,	&max77686_ldo14_data},
	{MAX77686_LDO22,	&max77686_ldo22_data},

	/* Other perpheral power supply  */
	{MAX77686_BUCK9, &max77686_buck9_data},
	{MAX77686_LDO5,	 &max77686_ldo5_data},
	{MAX77686_LDO8,	 &max77686_ldo8_data},
	{MAX77686_LDO9,	 &max77686_ldo9_data},
	{MAX77686_LDO10, &max77686_ldo10_data},
	{MAX77686_LDO12, &max77686_ldo12_data},
	{MAX77686_LDO13, &max77686_ldo13_data},
	{MAX77686_LDO15, &max77686_ldo15_data},
	{MAX77686_LDO16, &max77686_ldo16_data},
	{MAX77686_LDO17, &max77686_ldo17_data},
	{MAX77686_LDO18, &max77686_ldo18_data},
	{MAX77686_LDO19, &max77686_ldo19_data},
	{MAX77686_LDO20, &max77686_ldo20_data},
	{MAX77686_LDO21, &max77686_ldo21_data},
	{MAX77686_LDO23, &max77686_ldo23_data},
	{MAX77686_LDO24, &max77686_ldo24_data},
	{MAX77686_LDO25, &max77686_ldo25_data},
	{MAX77686_LDO26, &max77686_ldo26_data},
	{MAX77686_P32KH, &max77686_enp32khz_data},
	{MAX77686_EN32KHZ_CP, &max77686_en32khcp_data},
};

static struct max77686_opmode_data max77686_opmode_private_data[MAX77686_REG_MAX] = {
	[MAX77686_LDO2] 	= {MAX77686_LDO2, MAX77686_OPMODE_STANDBY},	/*MOFF*/
	[MAX77686_LDO3] 	= {MAX77686_LDO3, MAX77686_OPMODE_NORMAL},	/*DVDD1.8V*/
	[MAX77686_LDO4] 	= {MAX77686_LDO4, MAX77686_OPMODE_LP},		/*MMC IO*/
	[MAX77686_LDO6] 	= {MAX77686_LDO6, MAX77686_OPMODE_STANDBY},	/*MPLL*/
	[MAX77686_LDO7] 	= {MAX77686_LDO7, MAX77686_OPMODE_STANDBY},	/*APLL*/
	[MAX77686_LDO8] 	= {MAX77686_LDO8, MAX77686_OPMODE_STANDBY},	/*MIPI 1.0V*/
	[MAX77686_LDO10]	= {MAX77686_LDO10, MAX77686_OPMODE_STANDBY},	/*MIPI 1.8V*/
	[MAX77686_LDO11]	= {MAX77686_LDO11, MAX77686_OPMODE_LP},		/*ABB1*/
	[MAX77686_LDO12]	= {MAX77686_LDO12, MAX77686_OPMODE_LP},		/*OTG 3.0V*/
	[MAX77686_LDO14]	= {MAX77686_LDO14, MAX77686_OPMODE_LP}, 	/*ABB2*/
	[MAX77686_LDO15] 	= {MAX77686_LDO15, MAX77686_OPMODE_STANDBY},	/*OTG*/
	[MAX77686_LDO16] 	= {MAX77686_LDO16, MAX77686_OPMODE_STANDBY},	/*HSIC 1.8V*/
	[MAX77686_LDO18] 	= {MAX77686_LDO18, MAX77686_OPMODE_NORMAL},	/*AUDIO*/
	[MAX77686_LDO20] 	= {MAX77686_LDO20, MAX77686_OPMODE_LP},		/*MHL*/
	[MAX77686_LDO25] 	= {MAX77686_LDO25, MAX77686_OPMODE_LP},		/*TOUCH*/
	[MAX77686_LDO26] 	= {MAX77686_LDO26, MAX77686_OPMODE_LP},		/*MHL 3.0V*/

	[MAX77686_BUCK1]	= {MAX77686_BUCK1, MAX77686_OPMODE_STANDBY},	/*MIF*/
	[MAX77686_BUCK2]	= {MAX77686_BUCK2, MAX77686_OPMODE_STANDBY},	/*ARM*/
	[MAX77686_BUCK3]	= {MAX77686_BUCK3, MAX77686_OPMODE_STANDBY},	/*INT*/
	[MAX77686_BUCK4]	= {MAX77686_BUCK4, MAX77686_OPMODE_STANDBY},	/*G3D*/
	[MAX77686_BUCK8]	= {MAX77686_BUCK8, MAX77686_OPMODE_STANDBY},	/*INAND*/
	[MAX77686_BUCK9]	= {MAX77686_BUCK8, MAX77686_OPMODE_STANDBY},	/*ISP 1.2V*/
};

static struct max77686_platform_data __initdata m040_max77686_info = {
	.num_regulators = ARRAY_SIZE(max77686_regulators),
	.regulators 	= max77686_regulators,
	.irq_gpio	= M040_PMIC0_IRQ,
	.irq_base	= IRQ_BOARD_START,
	.wakeup		= true,

	.opmode_data = max77686_opmode_private_data,
	.ramp_rate = MAX77686_RAMP_RATE_27MV,
	.buck2_gpiodvs 	= true,
	.buck3_gpiodvs 	= true,
	.buck4_gpiodvs 	= true,

	.buck234_gpio_dvs = {
		M040_GPIO_DVS1,
		M040_GPIO_DVS2,
		M040_GPIO_DVS3,
	},
	.buck234_gpio_selb = {
		M040_BUCK2_SEL,
		M040_BUCK3_SEL,
		M040_BUCK4_SEL,
	},
	/* default exynos_result_of_asv = 7 */
	/* Arm core voltage */
	.buck2_voltage[0] = 1100000,
	.buck2_voltage[1] = 1050000,
	.buck2_voltage[2] = 1012500,
	.buck2_voltage[3] = 962500,
	.buck2_voltage[4] = 950000,
	.buck2_voltage[5] = 937500,
	.buck2_voltage[6] = 925000,
	.buck2_voltage[7] = 900000,

	/* Int voltage */
	.buck3_voltage[0] = 1000000,
	.buck3_voltage[1] = 975000,
	.buck3_voltage[2] = 950000,
	.buck3_voltage[3] = 925000,
	.buck3_voltage[4] = 900000,
	.buck3_voltage[5] = 875000,
	.buck3_voltage[6] = 862500,
	.buck3_voltage[7] = 850000,

	/* mali400 voltage */
	.buck4_voltage[0] = 1050000,
	.buck4_voltage[1] = 1025000,
	.buck4_voltage[2] = 1000000,
	.buck4_voltage[3] = 975000,
	.buck4_voltage[4] = 950000,
	.buck4_voltage[5] = 925000,
	.buck4_voltage[6] = 900000,
	.buck4_voltage[7] = 875000,
};
#endif

#ifdef CONFIG_MFD_MAX77665
static struct regulator_consumer_supply safeout1_supply[] = {
	REGULATOR_SUPPLY("safeout1", NULL),
};

static struct regulator_consumer_supply safeout2_supply[] = {
	REGULATOR_SUPPLY("safeout2", NULL),
};

static struct regulator_consumer_supply charger_supply[] = {
	REGULATOR_SUPPLY("vinchg1", "max77665-charger"),
};

static struct regulator_consumer_supply flash_led_supply[] = {
	REGULATOR_SUPPLY("flash_led", NULL),
};

static struct regulator_consumer_supply torch_led_supply[] = {
	REGULATOR_SUPPLY("torch_led", NULL),
};

static struct regulator_consumer_supply reverse_supply[] = {
	REGULATOR_SUPPLY("reverse", NULL),
};

static struct regulator_consumer_supply battery_supply[] = {
	REGULATOR_SUPPLY("battery", NULL),
};

static struct regulator_init_data safeout1_init_data = {
	.constraints	= {
		.name		= "safeout1 range",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on	= true,
		.state_mem	= {
			.enabled 	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout1_supply),
	.consumer_supplies	= safeout1_supply,
};

static struct regulator_init_data safeout2_init_data = {
	.constraints	= {
		.name		= "safeout2 range",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled 	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout2_supply),
	.consumer_supplies	= safeout2_supply,
};

static struct regulator_init_data charger_init_data = {
	.constraints	= {
		.name		= "CHARGER",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					      REGULATOR_CHANGE_CURRENT,
		.min_uA		= 60000,
		.max_uA		= 2580000,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(charger_supply),
	.consumer_supplies	= charger_supply,
};

static struct regulator_init_data flash_led_init_data = {
	.constraints	= {
		.name		= "FLASH LED",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					      REGULATOR_CHANGE_CURRENT,
		.boot_on	= false,
		.min_uA		= 15625,
		.max_uA		= 1000000,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(flash_led_supply),
	.consumer_supplies	= flash_led_supply,
};

static struct regulator_init_data torch_led_init_data = {
	.constraints	= {
		.name		= "TORCH LED",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					      REGULATOR_CHANGE_CURRENT,
		.boot_on	= false,
		.min_uA		= 15625,
		.max_uA		= 250000,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(torch_led_supply),
	.consumer_supplies	= torch_led_supply,
};

static struct regulator_init_data reverse_init_data = {
	.constraints	= {
		.name		= "REVERSE",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on	= false,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(reverse_supply),
	.consumer_supplies	= reverse_supply,
};

static struct regulator_init_data battery_init_data = {
	.constraints	= {
		.name		= "BATTERY",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					      REGULATOR_CHANGE_CURRENT,
		.boot_on	= true,
		.min_uA		= 0,
		.max_uA		= 2100000,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(battery_supply),
	.consumer_supplies	= battery_supply,
};

static struct max77665_regulator_data max77665_regulators[] = {
	{MAX77665_ESAFEOUT1, &safeout1_init_data,},
	{MAX77665_ESAFEOUT2, &safeout2_init_data,},
	{MAX77665_CHARGER, &charger_init_data,},
	{MAX77665_FLASH_LED, &flash_led_init_data,},
	{MAX77665_TORCH_LED, &torch_led_init_data,},
	{MAX77665_REVERSE, &reverse_init_data,},
	{MAX77665_BATTERY, &battery_init_data,},
};
#endif
