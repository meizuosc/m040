#if defined(CONFIG_REGULATOR_FIXED_VOLTAGE)
static struct regulator_consumer_supply wm8958_fixed_voltage0_supplies[] = {
	REGULATOR_SUPPLY("AVDD2", "0-001a"),
	REGULATOR_SUPPLY("CPVDD", "0-001a"),
	REGULATOR_SUPPLY("DBVDD1", "0-001a"),
	REGULATOR_SUPPLY("DBVDD2", "0-001a"),
	REGULATOR_SUPPLY("DBVDD3", "0-001a"),
};

static struct regulator_consumer_supply wm8958_fixed_voltage1_supplies[] = {
	REGULATOR_SUPPLY("SPKVDD1", "0-001a"),
	REGULATOR_SUPPLY("SPKVDD2", "0-001a"),
};

static struct regulator_consumer_supply wm8958_fixed_voltage2_supplies =
	REGULATOR_SUPPLY("DBVDD", "0-001a");

static struct regulator_init_data wm8958_fixed_voltage0_init_data = {
	.constraints = {
		.always_on = true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies = ARRAY_SIZE(wm8958_fixed_voltage0_supplies),
	.consumer_supplies = wm8958_fixed_voltage0_supplies,
};

static struct regulator_init_data wm8958_fixed_voltage1_init_data = {
	.constraints = {
		.always_on = true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies = ARRAY_SIZE(wm8958_fixed_voltage1_supplies),
	.consumer_supplies = wm8958_fixed_voltage1_supplies,
};

static struct regulator_init_data wm8958_fixed_voltage2_init_data = {
	.constraints = {
		.always_on = true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &wm8958_fixed_voltage2_supplies,
};

static struct fixed_voltage_config wm8958_fixed_voltage0_config = {
	.supply_name	= "VDD_1.8V",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &wm8958_fixed_voltage0_init_data,
};

static struct fixed_voltage_config wm8958_fixed_voltage1_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &wm8958_fixed_voltage1_init_data,
};

static struct fixed_voltage_config wm8958_fixed_voltage2_config = {
	.supply_name	= "VDD_3.3V",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.init_data	= &wm8958_fixed_voltage2_init_data,
};

static struct platform_device wm8958_fixed_voltage0 = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data = &wm8958_fixed_voltage0_config,
	},
};

static struct platform_device wm8958_fixed_voltage1 = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev		= {
		.platform_data = &wm8958_fixed_voltage1_config,
	},
};

static struct platform_device wm8958_fixed_voltage2 = {
	.name		= "reg-fixed-voltage",
	.id		= 2,
	.dev		= {
		.platform_data = &wm8958_fixed_voltage2_config,
	},
};

static struct regulator_consumer_supply m032_fixed_voltage0_supplies =
	REGULATOR_SUPPLY("LCD_5.5V", "ls040b3sx01");

static struct regulator_init_data m032_fixed_voltage0_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &m032_fixed_voltage0_supplies,
};

static struct fixed_voltage_config m032_fixed_voltage0_config = {
	.supply_name	= "VDD_5.5V",
	.microvolts	= 5500000,
	.gpio		= LCD_ON,
	.enable_high 	= true,
	.init_data	= &m032_fixed_voltage0_init_data,
};

static struct platform_device m032_fixed_voltage0 = {
	.name	= "reg-fixed-voltage",
	.id		= 3,
	.dev		= {
		.platform_data = &m032_fixed_voltage0_config,
	},
};

static struct regulator_consumer_supply m032_fixed_voltage1_supplies =
	REGULATOR_SUPPLY("spdif_en", NULL);

static struct regulator_init_data m032_fixed_voltage1_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &m032_fixed_voltage1_supplies,
};

static struct fixed_voltage_config m032_fixed_voltage1_config = {
	.supply_name	= "SPDIF 1.8V",
	.microvolts	= 1800000,
	.gpio		= SPDIF_OUT,
	.init_data	= &m032_fixed_voltage1_init_data,
};

static struct platform_device m032_fixed_voltage1 = {
	.name	= "reg-fixed-voltage",
	.id		= 4,
	.dev		= {
		.platform_data = &m032_fixed_voltage1_config,
	},
};

static struct regulator_consumer_supply m032_fixed_voltage2_supplies =
	REGULATOR_SUPPLY("sensor_power", NULL);

static struct regulator_init_data m032_fixed_voltage2_init_data = {
	.constraints = {
		.boot_on = true,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &m032_fixed_voltage2_supplies,
};

static struct fixed_voltage_config m032_fixed_voltage2_config = {
	.supply_name	= "SENSOR_POWER 2.8V",
	.microvolts	= 2800000,
	.gpio		= SENSOR_EN,
	.enable_high 	= true,
	.init_data	= &m032_fixed_voltage2_init_data,
};

static struct platform_device m032_fixed_voltage2 = {
	.name	= "reg-fixed-voltage",
	.id		= 5,
	.dev		= {
		.platform_data = &m032_fixed_voltage2_config,
	},
};

static struct regulator_consumer_supply m032_fixed_voltage3_supplies =
	REGULATOR_SUPPLY("iNAND", "dw_mmc");

static struct regulator_init_data m032_fixed_voltage3_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &m032_fixed_voltage3_supplies,
};

static struct fixed_voltage_config m032_fixed_voltage3_config = {
	.supply_name	= "iNAND_POWER 2.8V",
	.microvolts	= 2800000,
	.gpio		= INAND_EN,
	.enable_high 	= true,
	.init_data	= &m032_fixed_voltage3_init_data,
};

static struct platform_device m032_fixed_voltage3 = {
	.name	= "reg-fixed-voltage",
	.id		= 6,
	.dev		= {
		.platform_data = &m032_fixed_voltage3_config,
	},
};

/* HSIC core 1.0v power */
static struct regulator_consumer_supply m032_fixed_voltage4_supplies[] ={
	REGULATOR_SUPPLY("pd_hsic", "s3c-usbgadget"),
	REGULATOR_SUPPLY("pd_hsic", "s5p-ehci"),
};

static struct regulator_init_data m032_fixed_voltage4_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies = 2,
	.consumer_supplies = m032_fixed_voltage4_supplies,
};

static struct fixed_voltage_config m032_fixed_voltage4_config = {
	.supply_name	= "HSIC_1.0V",
	.microvolts	= 1000000,
	.gpio		= GPIO_HSIC_EN,
	.enable_high 	= true,
	.init_data	= &m032_fixed_voltage4_init_data,
};

static struct platform_device m032_fixed_voltage4 = {
	.name	= "reg-fixed-voltage",
	.id		= 7,
	.dev		= {
		.platform_data = &m032_fixed_voltage4_config,
	},
};

static struct regulator_consumer_supply m032_hdmi_fixed_voltage_supplies =
	REGULATOR_SUPPLY("HDMI_1.0V", NULL);

static struct regulator_init_data m032_hdmi_fixed_voltage_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies= 1,
	.consumer_supplies	= &m032_hdmi_fixed_voltage_supplies,
};

static struct fixed_voltage_config m032_hdmi_fixed_voltage_config = {
	.supply_name	= "VDD_1.0V",
	.microvolts	= 1000000,
	.gpio		= HDMI_P10EN,
	.enable_high	= true,
	.init_data	= &m032_hdmi_fixed_voltage_init_data,
};

static struct platform_device m032_hdmi_fixed_voltage = {
	.name	= "reg-fixed-voltage",
	.id		= 8,
	.dev	= {
		.platform_data = &m032_hdmi_fixed_voltage_config,
	},
};

static struct regulator_consumer_supply m032_mhl_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("MHL_1.2V", NULL),
};

static struct regulator_init_data m032_mhl_fixed_voltage_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies= 1,
	.consumer_supplies	= m032_mhl_fixed_voltage_supplies,
};

static struct fixed_voltage_config m032_mhl_fixed_voltage_config = {
	.supply_name	= "VDD_1.2V",
	.microvolts	= 1200000,
	.gpio		= MHL12_EN,
	.enable_high	= true,
	.init_data	= &m032_mhl_fixed_voltage_init_data,
};

static struct platform_device m032_mhl_fixed_voltage = {
	.name	= "reg-fixed-voltage",
	.id		= 9,
	.dev		= {
		.platform_data = &m032_mhl_fixed_voltage_config,
	},
};

static struct regulator_consumer_supply m032_isp_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("cam0_isp_1.2v", NULL),
};

static struct regulator_init_data m032_isp_fixed_voltage_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies= 1,
	.consumer_supplies	= m032_isp_fixed_voltage_supplies,
};

static struct fixed_voltage_config m032_isp_fixed_voltage_config = {
	.supply_name	= "VDD_1.2V",
	.microvolts	= 1200000,
	.gpio		= CAMERA_ISP_EN,
	.enable_high	= true,
	.init_data	= &m032_isp_fixed_voltage_init_data,
};

static struct platform_device m032_isp_fixed_voltage = {
	.name	= "reg-fixed-voltage",
	.id		= 10,
	.dev		= {
		.platform_data = &m032_isp_fixed_voltage_config,
	},
};
#endif


