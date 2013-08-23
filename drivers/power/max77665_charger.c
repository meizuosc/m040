/*

 * Battery driver for Maxim MAX77665
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author:  Lvcha qiu <lvcha@meizu.com> Chwei <Chwei@meizu.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/mfd/max77665.h>
#include <linux/mfd/max77665-private.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/android_alarm.h>
#include <mach/usb-detect.h>
#include <linux/workqueue.h>
#include <plat/adc.h>
#include <linux/bq27541-private.h>

#define MAX_AC_CURRENT          1000  
#define CHGIN_USB_CURRENT	450
#define TEMP_CHECK_DELAY        (60*HZ) 
#define WAKE_ALARM_INT          (60) 
#define CURRENT_INCREMENT_STEP  100   /*mA*/ 
#define COMPLETE_TIMEOUT        200   /*ms*/ 
#define MA_TO_UA                1000  
#define ADC_READ_AMOUNT		20

#define BATTERY_TEMP_2		20		/*2oC*/
#define BATTERY_TEMP_12		120		/*12oC*/
#define BATTERY_TEMP_17		170		/*17oC*/
#define BATTERY_TEMP_20		200		/*20oC*/
#define BATTERY_TEMP_23		230		/*23oC*/
#define BATTERY_TEMP_27		270		/*27oC*/
#define BATTERY_TEMP_45		450		/*45oC*/
#define BATTERY_430V_CURRENT_01C 167  
#define BATTERY_430V_CURRENT_03C 534  
#define BATTERY_430V_CURRENT_04C 720  
#define BATTERY_430V_CURRENT_05C 900  
#define BATTERY_435V_CURRENT_03C 567 
#define BATTERY_435V_CURRENT_05C 950  
#define BATTERY_TEMP_4VOLTAGE    4000 
#define BATTERY_TEMP_42VOLTAGE   4200 

#define MAX77665_CHGIN_DTLS       0x60 
#define MAX77665_CHGIN_DTLS_SHIFT 5    
#define MAX77665_CHG_DTLS         0x0F 
#define MAX77665_CHG_DTLS_SHIFT   0    
#define MAX77665_BAT_DTLS         0x70 
#define MAX77665_BAT_DTLS_SHIFT   4    
#define MAX77665_BYP_DTLS         0x0F 
#define MAX77665_BYP_DTLS_SHIFT   0    
#define MAX77665_BYP_OK           0x01 
#define MAX77665_BYP_OK_SHIFT     0    
#define MAX77665_CHGIN_OK         0x40 
#define MAX77665_CHGIN_OK_SHIFT   6    

#define ADB_CLOSE	0
#define ADB_OPEN	1
#define STORAGE_OPEN	2
#define STORAGE_CLOSE	3
#define	RNDIS_OPEN	4
#define	RNDIS_CLOSE	5

struct max77665_charger
{
	struct device *dev;
	struct power_supply psy_usb;
	struct power_supply psy_ac;
	struct power_supply psy_charger;
	struct max77665_dev *iodev;
	struct wake_lock wake_lock;
	struct delayed_work dwork;
	struct delayed_work poll_dwork;
	struct workqueue_struct *ad_curr;
	struct delayed_work ad_work;
	struct work_struct chgin_work;
	struct delayed_work adjust_dwork;
	struct delayed_work usb_notifier_work;
	struct regulator *ps;
	struct regulator *reverse;
	struct regulator *battery;
	struct mutex mutex_t;
	struct completion byp_complete;	

	enum cable_status_t {
		CABLE_TYPE_NONE = 0,
		CABLE_TYPE_USB,
		CABLE_TYPE_AC,
		CABLE_TYPE_UNKNOW,
	} cable_status;

	enum chg_status_t {
		CHG_STATUS_FAST,
		CHG_STATUS_DONE,
		CHG_STATUS_RECHG,
	} chg_status;

	bool chgin;
	int chgin_irq;
	int bypass_irq;
	int chr_pin;
	int chgin_ilim_usb;	/* 60mA ~ 500mA */
	int chgin_ilim_ac;	/* 60mA ~ 2.58A */
	int fast_charge_current;
	int (*usb_attach) (bool);
	int irq_reg;
	struct alarm alarm;
	bool done;
	bool adc_flag;
	struct s3c_adc_client *adc;
	int battery_health;
	struct notifier_block usb_notifer;
	bool adb_open;
	bool storage_open;
	bool rndis_open;
	int fastcharging;
	bool adjust_done;
	int adjust_count;
	struct alarm adjust_alarm;
};

enum {
 	BATTERY_HEALTH_UNKNOW = 0,
 	BATTERY_HEALTH_GOOD,
 	BATTERY_HEALTH_LOW1,
 	BATTERY_HEALTH_LOW2,
  	BATTERY_HEALTH_LOW3,
	BATTERY_HEALTH_OVERHEAT,
	BATTERY_HEALTH_COLD,
};
extern struct class *power_supply_class;

#ifdef CONFIG_USB_GADGET 
extern int register_usb_gadget_notifier(struct notifier_block *nb);
#else
static int register_usb_gadget_notifier(struct notifier_block *nb) {return 0;}
#endif

static BLOCKING_NOTIFIER_HEAD(max77665_charger_chain_head);

int register_max77665_charger_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&max77665_charger_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_max77665_charger_notifier);

int unregister_max77665_charger_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&max77665_charger_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_max77665_charger_notifier);

static int max77665_charger_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&max77665_charger_chain_head, val, NULL)
			== NOTIFY_BAD) ? -EINVAL : 0;
}

static char *supply_list[] = {
	"battery",
};

static enum power_supply_property max77665_power_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
};

static int max77665_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max77665_charger *charger =
		container_of(psy, struct max77665_charger, psy_usb);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the AC charger is connected */
	val->intval = (charger->cable_status == CABLE_TYPE_USB);

	return 0;
}

static int max77665_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max77665_charger *charger =
		container_of(psy, struct max77665_charger, psy_ac);
	
	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the AC charger is connected */
	val->intval = (charger->cable_status == CABLE_TYPE_AC);
	
	return 0;
}

static enum power_supply_property max77665_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
};

static int max77665_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max77665_charger *charger =
		container_of(psy, struct max77665_charger, psy_charger);
	
	if (psp != POWER_SUPPLY_PROP_STATUS)
		return -EINVAL;

	if (charger->cable_status == CABLE_TYPE_USB || 
		charger->cable_status == CABLE_TYPE_AC) {
		if (charger->chg_status == CHG_STATUS_FAST)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_FULL;
	} else {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	
	return 0;
}

static void set_alarm(struct alarm *alarm, int seconds)
{
	ktime_t interval = ktime_set(seconds, 0);
	ktime_t now = alarm_get_elapsed_realtime();
	ktime_t next = ktime_add(now, interval);

	pr_info("set alarm after %d seconds\n", seconds);
	alarm_start_range(alarm, next, next);
}

static void adjust_current_alarm(struct alarm *alarm)
{
	struct max77665_charger *charger = container_of(alarm, struct max77665_charger,
			adjust_alarm);

	wake_lock_timeout(&charger->wake_lock, 3 * HZ);

	if (charger->chgin) {
		if (delayed_work_pending(&charger->adjust_dwork))
			cancel_delayed_work(&charger->adjust_dwork);
		schedule_delayed_work_on(0, &charger->adjust_dwork, 0);
	}
}

static void charger_bat_alarm(struct alarm *alarm)
{
	struct max77665_charger *chg = container_of(alarm, struct max77665_charger, alarm);

	wake_lock_timeout(&chg->wake_lock, 3 * HZ);
	set_alarm(&chg->alarm, WAKE_ALARM_INT);
}

#define CHG_ADC_CHANNEL 2
static int adc_threshold_check(struct max77665_charger *charger)
{
	int adc_value = 0;
	int i = 0, adc_sum = 0;

	for (i = 0; i < ADC_READ_AMOUNT; i++) {
		adc_value = s3c_adc_read(charger->adc, CHG_ADC_CHANNEL);
		adc_sum += adc_value;
	}
		
	adc_value = adc_sum / ADC_READ_AMOUNT;
	if (adc_value > 1024) {
		return 1;
	} else {
		return 0;
	}
}

static void max77665_usb_charger(struct max77665_charger *charger)
{
	if (!regulator_is_enabled(charger->ps))
		regulator_enable(charger->ps);

	regulator_set_current_limit(charger->ps, CHGIN_USB_CURRENT * MA_TO_UA,
			MAX_AC_CURRENT * MA_TO_UA);
}

static int max77665_battery_temp_status(struct max77665_charger *charger)
{
	struct power_supply *fuelgauge_ps
		= power_supply_get_by_name("fuelgauge");
	union power_supply_propval val;
	int battery_temp = 0, battery_voltage = 0;
	int battery_current = min(MAX_AC_CURRENT, charger->fast_charge_current);
	int health = BATTERY_HEALTH_GOOD;
	char battery_manufacturer[10] = {0};

	if (fuelgauge_ps) {
		if(fuelgauge_ps->get_property(fuelgauge_ps, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val) == 0)
			battery_voltage = val.intval;
		if (fuelgauge_ps->get_property(fuelgauge_ps, POWER_SUPPLY_PROP_MANUFACTURER, &val) == 0)
			strcpy(battery_manufacturer, val.strval);
		if(fuelgauge_ps->get_property(fuelgauge_ps, POWER_SUPPLY_PROP_TEMP, &val) == 0) {
			battery_temp = val.intval;

			/*the adjustment programs suitable for ATL battery*/
			if (!strcmp("SWD M040", battery_manufacturer)) {
				if (battery_temp <= BATTERY_TEMP_2) {
					battery_current = 0;
					health = BATTERY_HEALTH_COLD;
				} else if (battery_temp > BATTERY_TEMP_45) {
					battery_current = 0;
					health = BATTERY_HEALTH_OVERHEAT;
				} else if (battery_temp <= BATTERY_TEMP_17) {
					battery_current = min(battery_current, BATTERY_430V_CURRENT_03C);
				} else {
					battery_current = min(battery_current, BATTERY_430V_CURRENT_05C);
				}
			} else if (!strncmp("M04S", battery_manufacturer, 4)) {
				if (battery_temp <= BATTERY_TEMP_2) {
					battery_current = 0;
					health = BATTERY_HEALTH_COLD;
				} else if (battery_temp > BATTERY_TEMP_45) {
					battery_current = 0;
					health = BATTERY_HEALTH_OVERHEAT;
				} else if (battery_temp <= BATTERY_TEMP_17) {
					battery_current = min(battery_current, BATTERY_435V_CURRENT_03C);
				} else {
					battery_current = min(battery_current, BATTERY_435V_CURRENT_05C);
				}
			} else {
				/*the adjustment programs suitable for Guangyu battery*/
				if (battery_temp <= BATTERY_TEMP_2) {
					battery_current = 0;
					health = BATTERY_HEALTH_COLD;
				} else if (battery_temp > BATTERY_TEMP_45) {
					battery_current = 0;
					health = BATTERY_HEALTH_OVERHEAT;
				} else if (battery_temp <= BATTERY_TEMP_12) {
					battery_current = min(battery_current, BATTERY_430V_CURRENT_01C);
				} else if (battery_temp <= BATTERY_TEMP_20) {
					if (battery_voltage > BATTERY_TEMP_42VOLTAGE * MA_TO_UA) {
						battery_current = min(battery_current, BATTERY_430V_CURRENT_01C);
					} else {
						battery_current = min(battery_current, BATTERY_430V_CURRENT_03C);
					}
				} else if (battery_temp <= BATTERY_TEMP_23) {
					if (battery_voltage > BATTERY_TEMP_4VOLTAGE * MA_TO_UA) {
						battery_current = min(battery_current, BATTERY_430V_CURRENT_03C);
					} else {
						battery_current = min(battery_current, BATTERY_430V_CURRENT_05C);
					}
				} else if (battery_temp <= BATTERY_TEMP_27) {
					battery_current = min(battery_current, BATTERY_430V_CURRENT_04C);	
				} else {
					battery_current = min(battery_current, BATTERY_430V_CURRENT_05C);
				}
			}
		}
	} else {
		battery_current = min(battery_current, BATTERY_430V_CURRENT_01C);
	}

	do {
		int ret;
		int now_current = regulator_get_current_limit(charger->battery) / 1000;
		if(!(now_current <= battery_current && battery_current <= now_current + CHG_CC_STEP)) {
			pr_info("now_current %d current %d\n", now_current, battery_current);
			ret = regulator_set_current_limit(charger->battery,
					battery_current * MA_TO_UA,
					(battery_current + CHG_CC_STEP) * MA_TO_UA);
			if (ret) {
				pr_err("failed to set battery current limit\n");
			}
		}
	} while(0);

	return health;
}

static int max77665_adjust_current(struct max77665_charger *charger,
		int chgin_ilim)
{
	int ret = 0, ad_current;
	unsigned long expire;
	int MAX_INPUT_CURRENT = charger->chgin_ilim_ac; 

	expire = msecs_to_jiffies(COMPLETE_TIMEOUT);
	charger->adc_flag = false;
	
	for (ad_current = chgin_ilim; ad_current <= MAX_INPUT_CURRENT;
			ad_current += CURRENT_INCREMENT_STEP) {
		
		ret = regulator_set_current_limit(charger->ps,
				ad_current*MA_TO_UA,
				(ad_current*MA_TO_UA+CURRENT_INCREMENT_STEP*MA_TO_UA));
		if (ret) {
			pr_err("failed to set current limit\n");
			return ret;
		}
		pr_info("%d,waiting..............\n", ad_current);
		
		init_completion(&charger->byp_complete);
		ret = wait_for_completion_timeout(&charger->byp_complete, expire);
		if (ret > 0) {
			pr_info("by pass not OK!!!\n");
			ad_current -= CURRENT_INCREMENT_STEP;
			ret = regulator_set_current_limit(charger->ps, 
					ad_current * MA_TO_UA,
					(ad_current * MA_TO_UA + 
					CURRENT_INCREMENT_STEP * MA_TO_UA));
			if (ret) {
				pr_err("adjust current to %d failed\n", ad_current);
			}
			break;
		}
		if (charger->adc_flag || !(adc_threshold_check(charger))) 
			break;
	}
	return ret;
}

static int max77665_charger_types(struct max77665_charger *charger)
{
	enum cable_status_t cable_status = charger->cable_status;
	int chgin_ilim = 0;
	int battery_status;

	chgin_ilim = charger->chgin_ilim_usb;
	battery_status = max77665_battery_temp_status(charger);
	if (!(	battery_status == BATTERY_HEALTH_LOW1
			|| battery_status == BATTERY_HEALTH_LOW2
			|| battery_status == BATTERY_HEALTH_LOW3)) {
		switch (cable_status) {
		case CABLE_TYPE_USB:
			if (!charger->fastcharging) {
				regulator_set_current_limit(charger->ps, chgin_ilim * MA_TO_UA, 
						MAX_AC_CURRENT * MA_TO_UA);
				break;
			}
		case CABLE_TYPE_AC:	
			if (false == charger-> done) {
				pr_info("we want to adjust the input current now\n");
				if (charger->cable_status == CABLE_TYPE_USB) {
					if (charger->adb_open || charger->storage_open
							|| charger->rndis_open) 
						max77665_usb_charger(charger);
					else 
						max77665_adjust_current(charger, chgin_ilim);
				} else {
					max77665_adjust_current(charger, chgin_ilim);
				}
				charger->done = true;
			} else {
				pr_info("we have adjusted already\n");
			}
			break;
		default:
			chgin_ilim = 0;
			break;
		}
	}
	return 0;
}

static void max77665_adjust_work(struct work_struct *work)
{
	struct max77665_charger *charger = container_of(work, 
			struct max77665_charger, adjust_dwork.work);
	int now_current;
	int battery_status;

	pr_info("ENTER %s########\n", __func__);

	if (charger->adjust_done) {
		charger->adjust_done = false;
		return;
	}

	now_current = regulator_get_current_limit(charger->ps);
	battery_status = max77665_battery_temp_status(charger);
	if (battery_status == BATTERY_HEALTH_GOOD) {
		if ((charger->chgin)
				&& now_current != charger->chgin_ilim_ac) {
			if (charger->cable_status == CABLE_TYPE_USB) {
				if (charger->adb_open || charger->storage_open
					|| charger->rndis_open)  
					return;	
			} 
			if (charger->adc_flag == true)
				charger->adc_flag = false;
			charger->done = false;
			max77665_charger_types(charger);	
		}	
	}
}

static void max77665_work_func(struct work_struct *work)
{
	struct max77665_charger *charger =
		container_of(work, struct max77665_charger, dwork.work);
	enum cable_status_t cable_status = CABLE_TYPE_NONE;

	mutex_lock(&charger->mutex_t);
	
	if (charger->chgin) {
#ifndef CONFIG_MACH_M040
		if (mx_is_usb_dock_insert()) {
			pr_info("found dock inserted, treat it as AC\n");
			cable_status = CABLE_TYPE_AC;
		} else {
			if (charger->usb_attach && !charger->usb_attach(true)) {
				msleep(2000);

				if (gpio_get_value(charger->chr_pin)) {
					cable_status = CABLE_TYPE_AC;
				} else {
					cable_status = CABLE_TYPE_USB;
				}
			}
		}
#else
		if (mx_is_usb_dock_insert()) {
			pr_info("found dock inserted, treat it as AC\n");
			cable_status = CABLE_TYPE_AC;
		}else {
			u8 reg_data;
			max77665_read_reg(charger->iodev->muic, MAX77665_MUIC_REG_CDETCTRL1, &reg_data);
			max77665_write_reg(charger->iodev->muic, MAX77665_MUIC_REG_CDETCTRL1, reg_data | 0x02);
			pr_info("#############usb start detect\n");
			msleep(500);

			max77665_read_reg(charger->iodev->muic, MAX77665_MUIC_REG_STATUS2, &reg_data);
			pr_info("#############usb end detect (0x%02x)\n", reg_data);
			if (reg_data & 0x08) {
				pr_info("wait, running#####\n");
				max77665_read_reg(charger->iodev->muic, MAX77665_MUIC_REG_CDETCTRL1, 
						&reg_data);
				max77665_write_reg(charger->iodev->muic, MAX77665_MUIC_REG_CDETCTRL1,
						reg_data | 0x02);
				pr_info("#############usb start detect again\n");
				msleep(500);
				max77665_read_reg(charger->iodev->muic, MAX77665_MUIC_REG_STATUS2, 
						&reg_data);
				pr_info("#############usb end detect (0x%02x)\n", reg_data);
				if (reg_data & 0x08) {
					pr_info("muic detect usb is error\n");
					cable_status = CABLE_TYPE_USB;
				} else {
					if ((reg_data & 0x07) == 0x01) {
						cable_status = CABLE_TYPE_USB;
					} else 
						cable_status = CABLE_TYPE_AC;
				}
			} else {
				if ((reg_data & 0x07) == 0x01) {
						cable_status = CABLE_TYPE_USB;
					} else 
						cable_status = CABLE_TYPE_AC;
			}
		}
		if (!regulator_is_enabled(charger->ps))
			regulator_enable(charger->ps);
#endif
	} else {
		charger->done = false;
		charger->adc_flag = false;
		regulator_set_current_limit(charger->ps,
				charger->chgin_ilim_usb * MA_TO_UA,
				MAX_AC_CURRENT * MA_TO_UA);
		cable_status = CABLE_TYPE_NONE;
	}

	charger->cable_status = cable_status;
	
	power_supply_changed(&charger->psy_ac);
	power_supply_changed(&charger->psy_usb);

	max77665_charger_types(charger);

	alarm_cancel(&charger->adjust_alarm);
	set_alarm(&charger->adjust_alarm, WAKE_ALARM_INT);

	if (delayed_work_pending(&charger->poll_dwork))
		cancel_delayed_work(&charger->poll_dwork);
	schedule_delayed_work_on(0, &charger->poll_dwork, 0);

	if (cable_status == CABLE_TYPE_USB) {
		if (charger->usb_attach)
			charger->usb_attach(true);	
		max77665_charger_notifier_call_chain(1);
	} else {
		msleep(500);
		if (charger->usb_attach)
			charger->usb_attach(false);
		max77665_charger_notifier_call_chain(0);
	}
	wake_lock_timeout(&charger->wake_lock, 3 * HZ);

	mutex_unlock(&charger->mutex_t);
}
	
static void max77665_poll_work_func(struct work_struct *work)
{
	struct max77665_charger *charger =
		container_of(work, struct max77665_charger, poll_dwork.work);
	struct power_supply *fuelgauge_ps
		= power_supply_get_by_name("fuelgauge");
	union power_supply_propval val;
	int battery_health = BATTERY_HEALTH_UNKNOW;

	mutex_lock(&charger->mutex_t);
	battery_health = max77665_battery_temp_status(charger);

	if (charger->chg_status == CHG_STATUS_FAST ||
			charger->chg_status == CHG_STATUS_RECHG) {
		struct i2c_client *i2c = charger->iodev->i2c;
		u8 reg_data;

		if(max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_DETAILS_01, &reg_data) >= 0) {
			if((reg_data & 0x0F) == 0x04) {
				charger->chg_status = CHG_STATUS_DONE;
			}
		}
	}
	if ( charger->cable_status == CABLE_TYPE_USB ||
			charger->cable_status == CABLE_TYPE_AC) {

		if (battery_health == BATTERY_HEALTH_COLD 
				|| battery_health == BATTERY_HEALTH_OVERHEAT 
				|| battery_health == BATTERY_HEALTH_UNKNOW) {
			charger->done = false;
			pr_info("----------battery unhealthy, disable charging\n");
			if (regulator_is_enabled(charger->ps)) {
				regulator_disable(charger->ps);
			}
		} else {
			if (charger->battery_health != battery_health) {
				if (!regulator_is_enabled(charger->ps))
					regulator_enable(charger->ps);
				max77665_charger_types(charger);
			}
			if (regulator_is_enabled(charger->ps)) {
				if (charger->chg_status == CHG_STATUS_DONE && fuelgauge_ps) {
					int soc = 100;
					if(fuelgauge_ps->get_property(fuelgauge_ps, POWER_SUPPLY_PROP_CAPACITY, &val) == 0)
						soc = val.intval;
					if(soc <= 98) 
						charger->chg_status = CHG_STATUS_RECHG;
				}
			} else {
				pr_info("----------battery healthy good, enable charging\n");
				regulator_enable(charger->ps);
			}
		}
		schedule_delayed_work_on(0, &charger->poll_dwork, TEMP_CHECK_DELAY);
	} else {
		if (regulator_is_enabled(charger->ps)) {
			pr_info("--------------charger remove, disable charging\n");
			regulator_disable(charger->ps);
		}
	}
	pr_debug("###########the current = %d\n",regulator_get_current_limit(charger->ps));
	charger->battery_health = battery_health;
	mutex_unlock(&charger->mutex_t);
}

static int max77665_reg_dump(struct max77665_charger *charger)
{
	struct i2c_client *i2c = charger->iodev->i2c;
	u8 int_ok, dtls_00, dtls_01, dtls_02;
	u8 chgin_dtls, chg_dtls, bat_dtls, byp_dtls;
	int ret;
	ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK,
			&int_ok);
	/* charger */
	ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_DETAILS_00,
			&dtls_00);
	ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_DETAILS_01,
			&dtls_01);
	ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_DETAILS_02,
			&dtls_02);
	chgin_dtls = ((dtls_00 & MAX77665_CHGIN_DTLS) >>
			MAX77665_CHGIN_DTLS_SHIFT);
	chg_dtls = ((dtls_01 & MAX77665_CHG_DTLS) >>
			MAX77665_CHG_DTLS_SHIFT);
	bat_dtls = ((dtls_01 & MAX77665_BAT_DTLS) >>
			MAX77665_BAT_DTLS_SHIFT);
	byp_dtls = ((dtls_02 & MAX77665_BYP_DTLS) >>
			MAX77665_BYP_DTLS_SHIFT);
	pr_info("INT_OK(0x%x), CHGIN(0x%x), CHG(0x%x), BAT(0x%x),BYP_DTLS(0x%02x)\n",
			int_ok, chgin_dtls,\
			chg_dtls, bat_dtls, byp_dtls);\
		return ret;
}

static void max77665_second_adjust_current(struct work_struct *work)
{
	struct max77665_charger *charger = container_of((struct delayed_work*)work,
			struct max77665_charger, ad_work);
	struct i2c_client *i2c = charger->iodev->i2c;
	int now_cur, ret;
	u8 int_ok = 0;

	do{
		now_cur = regulator_get_current_limit(charger->ps);
		
		pr_info("%s: %d\n",__func__, now_cur);
		if (now_cur < CHGIN_USB_CURRENT * MA_TO_UA)
			break;
		regulator_set_current_limit(charger->ps, 
				now_cur - CURRENT_INCREMENT_STEP*MA_TO_UA,
				now_cur);
		msleep(100);	
		ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK, 
				&int_ok);
	} while (int_ok != 0x5d);
	if (now_cur < CHGIN_USB_CURRENT * MA_TO_UA) {
		msleep(100);
		ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK, 
				&int_ok);
		if (int_ok != 0x5d) {
			charger->done = false;
			charger->adc_flag = false;
			charger->cable_status = CABLE_TYPE_NONE;
			charger->chgin = false;
			power_supply_changed(&charger->psy_usb);
			power_supply_changed(&charger->psy_ac);
			return;
		}
	}
}

static void max77665_chgin_irq_handler(struct work_struct *work)
{
	struct max77665_charger *charger = container_of(work,
			struct max77665_charger, chgin_work);
	struct i2c_client *i2c = charger->iodev->i2c;
	u8 int_ok, prev_int_ok;
	int ret;
	int chgin = 0, now_current = 0;

	pr_info("ENTER %s##################\n", __func__);
	if (regulator_is_enabled(charger->reverse)) {
		pr_info("usb host insert, dismiss this isr\n");
		return;
	}
	
	wake_lock(&charger->wake_lock);

	ret =  max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK,
			&prev_int_ok);
	if (unlikely(ret < 0)) {
		pr_err("Failed to read MAX77665_CHG_REG_CHG_INT: %d\n", ret);
		wake_unlock(&charger->wake_lock);
	}
	msleep(20);
	ret =  max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK,
			&int_ok);
	if (unlikely(ret < 0)) {
		pr_err("Failed to read MAX77665_CHG_REG_CHG_INT: %d\n", ret);
		wake_unlock(&charger->wake_lock);
	}
	if ((charger->irq_reg == int_ok) && (prev_int_ok != int_ok)) {
		pr_info("%s:irq debounced(0x%x, 0x%x, 0x%x),return\n",
				__func__, charger->irq_reg,
				prev_int_ok, int_ok);
		wake_unlock(&charger->wake_lock);
	} else {
		chgin = !!(int_ok & 0x40);
	}

	max77665_reg_dump(charger);
	charger->irq_reg = int_ok;
	
	pr_info("-----%s %s\n", __func__, chgin ? "insert" : "remove");

	if ((!chgin) && (adc_threshold_check(charger))) {
		charger->adc_flag = true;
		now_current = regulator_get_current_limit(charger->ps);
		do {
			now_current -= CURRENT_INCREMENT_STEP * MA_TO_UA;
			if (now_current < CHGIN_USB_CURRENT * MA_TO_UA)
				break;
			regulator_set_current_limit(charger->ps,
					now_current,
					now_current + CURRENT_INCREMENT_STEP*MA_TO_UA);
			msleep(100);
			max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK,
					&int_ok);
			pr_info("current %d\n", now_current);
			if (int_ok == 0X5d) {
				if (charger->adjust_count > 6) {
					charger->adjust_done = true;
					charger->adjust_count = 0;
				}
				charger->adjust_count++;
				return;
			}
		} while (now_current > CHGIN_USB_CURRENT * MA_TO_UA);
	}

	if (charger->chgin != chgin) {
		alarm_cancel(&charger->alarm);
		if(chgin)
			set_alarm(&charger->alarm, WAKE_ALARM_INT);
		charger->chgin = chgin;	
		charger->chg_status = CHG_STATUS_FAST;
	
		if (delayed_work_pending(&charger->dwork))
			cancel_delayed_work(&charger->dwork);
			
		schedule_delayed_work(&charger->dwork, HZ/4);
	} else {
		pr_info("unstable charger isr, dismiss this isr\n");
		msleep(2000);
		wake_unlock(&charger->wake_lock);
	}
}

static irqreturn_t max77665_charger_isr(int irq, void *dev_id)
{
	struct max77665_charger *charger = dev_id;
	schedule_work_on(0, &charger->chgin_work);
	return IRQ_HANDLED;
}

#define MAX77665_BYP_DTLS3 0x08
static irqreturn_t max77665_bypass_irq(int irq, void *dev_id)
{
	struct max77665_charger *charger = dev_id;
	struct i2c_client *i2c = charger->iodev->i2c;
	u8 reg_data = 0, bypass_flag, int_ok;
	int ret, now_current;

	wake_lock(&charger->wake_lock);

	pr_info("###################ENTER %s\n", __func__);
	max77665_reg_dump(charger);

	ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_DETAILS_02,
			&reg_data);
	if (unlikely(ret < 0)) {
		pr_err("Failed to read MAX77665_CHG_REG_CHG_DETAILS_02: %d\n", ret);
		wake_unlock(&charger->wake_lock);
		return IRQ_HANDLED;
	} else { 
		bypass_flag = reg_data & 0x0f;
	}
	/*cancel the chgin interrupt*/
	cancel_delayed_work(&charger->dwork);

	switch(bypass_flag){
	case MAX77665_BYP_DTLS3: /*VCHGINLIM*/
		pr_info("################## MAX77665_BYP_DTLS3:\n");
		if (charger->done == true) {
			now_current = regulator_get_current_limit(charger->ps);
			if (now_current < CHGIN_USB_CURRENT * MA_TO_UA);
				break;
			regulator_set_current_limit(charger->ps,
					now_current-CURRENT_INCREMENT_STEP*MA_TO_UA,
					now_current);
		} else {
			complete(&charger->byp_complete);
		}
		break;
	case 0:
		pr_info("######################BYPASS OK!\n");
		ret = max77665_read_reg(i2c,
				MAX77665_CHG_REG_CHG_INT_OK,
				&int_ok);
		if (int_ok != 0x5d) 
			queue_delayed_work(charger->ad_curr, &charger->ad_work, HZ/50);
		break;
	default:
		break;
	}
	wake_unlock(&charger->wake_lock);
	return IRQ_HANDLED; 
}

static ssize_t usb_fastcharging_store(struct device *dev, 
		struct device_attribute *attr,
		const char *buf, size_t count)
{	
	struct max77665_charger *charger = dev_get_drvdata(dev);

	charger->fastcharging = simple_strtol(buf, NULL, 10);
	if ((charger->chgin) && 
			(charger->cable_status == CABLE_TYPE_USB) ) {
		if (charger->fastcharging) {
			if (charger->done)
				charger->done = false;
		}
		max77665_charger_types(charger);
	}
	pr_info("%s:fastcharging = %d\n", __func__, charger->fastcharging);
	
	return count;
}

static ssize_t usb_fastcharging_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{	
	struct max77665_charger *charger = dev_get_drvdata(dev);
	int fastcharging = 0;

	fastcharging = charger->fastcharging;

	return sprintf(buf, "%d\n", fastcharging);
}
static DEVICE_ATTR(fastcharging, 0644, usb_fastcharging_show, usb_fastcharging_store);

static __devinit int max77665_init(struct max77665_charger *charger)
{
	struct max77665_dev *iodev = dev_get_drvdata(charger->dev->parent);
	struct max77665_platform_data *pdata = dev_get_platdata(iodev->dev);	
	struct i2c_client *i2c = iodev->i2c;
	int ret = EINVAL;
	u8 reg_data = 0;
	
	/* Unlock protected registers */
	ret = max77665_write_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_06, 0x0C);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX8957_REG_CHG_CNFG_06: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_FCHGTIME_4H, (u8)pdata->fast_charge_timer);
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_01, reg_data, 0x7<<0);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_01: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_CHG_RSTRT_100MV, (u8)pdata->charging_restart_thresold);
	reg_data <<= 4;
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_01, reg_data, 0x3<<4);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_01: %d\n", ret);
		goto error;
	}

	reg_data = 1<<7;
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_01, reg_data, 0x1<<7);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_01: %d\n", ret);
		goto error;
	}

	reg_data = (min(MAX_AC_CURRENT, pdata->fast_charge_current) /CHG_CC_STEP);
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_02, reg_data, 0x3f<<0);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_02: %d\n", ret);
		goto error;
	}

	reg_data = 1<<7;
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_02, reg_data, 0x1<<7);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_02: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_CHG_TO_ITH_100MA, (u8)pdata->top_off_current_thresold);
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_03, reg_data, 0x7<<0);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_03: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_CHG_TO_TIME_10MIN, (u8)pdata->top_off_current_thresold);
	reg_data <<= 3;
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_03, reg_data, 0x7<<3);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_03: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_CHG_CV_PRM_4200MV, (u8)pdata->charger_termination_voltage);
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_04, reg_data, 0x1f<<0);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_04: %d\n", ret);
		goto error;
	}

	/* Lock protected registers */
	ret = max77665_write_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_06, 0x00);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX8957_REG_CHG_CNFG_06: %d\n", ret);
		goto error;
	}

	/* disable muic ctrl */
	reg_data = 1<<5;
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_00, reg_data, 0x1<<5);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_00: %d\n", ret);
		goto error;
	}
	return 0;

error:
	return ret;
}

static void max77665_usb_notifier_dwork(struct work_struct *work)
{
	struct max77665_charger *charger = container_of(work,
			struct max77665_charger, usb_notifier_work.work);

	max77665_charger_types(charger);
}

static int max77665_charger_event(struct notifier_block *this, unsigned long event,
		void *ptr)
{
	struct max77665_charger *charger = container_of(this,
			struct max77665_charger, usb_notifer);

	switch (event) {
		case ADB_OPEN:
			pr_info("adb open##########\n");
			if (charger->cable_status != CABLE_TYPE_USB)
				return event;
			charger->adb_open = true;
			break;
		case STORAGE_OPEN:
			pr_info("usb mass storage open######\n");
			charger->storage_open = true;
			break;
		case RNDIS_OPEN:
			pr_info("rndis open########\n");
			charger->rndis_open = true;
			break;
		case ADB_CLOSE:
			pr_info("adb close##########\n");
			if (!charger->adb_open) 
				return event;
			charger->done = false;
			charger->adb_open = false;
			break;
		case STORAGE_CLOSE:
			pr_info("usb mass stroage close######\n");
			if (!charger->storage_open) 
				return event;
			charger->done = false;
			charger->storage_open = false;
			break;
		case RNDIS_CLOSE:
			pr_info("rndis close########\n");
			if (!charger->rndis_open)
				return event;
			charger->done = false;
			charger->rndis_open = false;
			break;
		default:
			break;
	}
	pr_info("adb_open:%d, storage_open:%d, rndis open:%d\n",
			charger->adb_open, charger->storage_open, charger->rndis_open);
	
	if (charger->chgin) 
		schedule_delayed_work_on(0, &charger->usb_notifier_work, HZ/10);

	return event;
}

static __devinit int max77665_charger_probe(struct platform_device *pdev)
{
	struct max77665_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct max77665_platform_data *pdata = dev_get_platdata(iodev->dev);	
	struct max77665_charger *charger;
	struct power_supply *fuelgauge_ps =
		power_supply_get_by_name("fuelgauge");
	union power_supply_propval val;
	u8 reg_data;
	int ret = EINVAL;

	charger = kzalloc(sizeof(struct max77665_charger), GFP_KERNEL);
	if (unlikely(!charger))
		return -ENOMEM;

	platform_set_drvdata(pdev, charger);

	if (fuelgauge_ps) {
		char manufacturer_name[10] = {0};
		if (fuelgauge_ps->get_property(fuelgauge_ps, POWER_SUPPLY_PROP_MANUFACTURER, &val) == 0)
			strcpy(manufacturer_name, val.strval);
		
		if (!strncmp(manufacturer_name, "M04S", 4)) {
			pdata->charger_termination_voltage = MAX77665_CHG_CV_PRM_4350MV;
			pdata->fast_charge_current = 934;
			pdata->chgin_ilim_ac = 1000;
		}
	}
	charger->iodev = iodev;
	charger->dev = &pdev->dev;
	charger->usb_attach = pdata->usb_attach;
	charger->chgin_ilim_usb = pdata->chgin_ilim_usb;
	charger->chgin_ilim_ac = pdata->chgin_ilim_ac;
	charger->fast_charge_current = pdata->fast_charge_current;
	charger->chr_pin = pdata->charger_pin;
	charger->done = false;
	charger->adc_flag = false;
	charger->adb_open = false;
	charger->storage_open = false;
	charger->rndis_open = false;
	charger->fastcharging = 0;
	charger->adjust_count = 0;
	charger->usb_notifer.notifier_call = max77665_charger_event;
	register_usb_gadget_notifier(&charger->usb_notifer);
	
	init_completion(&charger->byp_complete);
	mutex_init(&charger->mutex_t);
	wake_lock_init(&charger->wake_lock, WAKE_LOCK_SUSPEND, pdata->name);
	INIT_DELAYED_WORK(&charger->dwork, max77665_work_func);
	INIT_DELAYED_WORK(&charger->poll_dwork, max77665_poll_work_func);
	charger->ad_curr = create_singlethread_workqueue("max77665_sed_ad_curr");
	INIT_DELAYED_WORK(&charger->ad_work,max77665_second_adjust_current);
	INIT_WORK(&charger->chgin_work, max77665_chgin_irq_handler);
	INIT_DELAYED_WORK(&charger->adjust_dwork, max77665_adjust_work);
	INIT_DELAYED_WORK(&charger->usb_notifier_work, max77665_usb_notifier_dwork);

	charger->ps = regulator_get(charger->dev, pdata->supply);
	if (IS_ERR(charger->ps)) {
		dev_err(&pdev->dev, "Failed to regulator_get ps: %ld\n", 
				PTR_ERR(charger->ps));
		goto err_free;
	}

	charger->reverse = regulator_get(NULL, "reverse");
	if (IS_ERR(charger->reverse)) {
		dev_err(&pdev->dev, "Failed to regulator_get reverse: %ld\n",
				PTR_ERR(charger->reverse));
		goto err_put1;
	}

	charger->battery = regulator_get(NULL, "battery");
	if (IS_ERR(charger->battery)) {
		dev_err(&pdev->dev, "Failed to regulator_get battery: %ld\n",
				PTR_ERR(charger->battery));
		goto err_put0;
	}

	charger->adc = s3c_adc_register(pdev, NULL, NULL, 0);
	if (IS_ERR(charger->adc)) {
		dev_err(&pdev->dev, "cannot register adc\n");
		ret = PTR_ERR(charger->adc);
		goto err_put;
	}
	
	if (!power_supply_class) 
		return -ENXIO;

	charger->dev = device_create(power_supply_class, charger->dev->parent, 
			0, charger, "charging-switch");
	ret = device_create_file(charger->dev, &dev_attr_fastcharging);

	charger->psy_charger.name = "charger";
	charger->psy_charger.type = POWER_SUPPLY_TYPE_BATTERY;
	charger->psy_charger.properties = max77665_charger_props,
	charger->psy_charger.num_properties = ARRAY_SIZE(max77665_charger_props),
	charger->psy_charger.get_property = max77665_charger_get_property,
	ret = power_supply_register(&pdev->dev, &charger->psy_charger);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "Failed to power_supply_register psy_charger: %d\n", ret);
		goto err_adc_unregister;
	}

	charger->psy_usb.name = "usb";
	charger->psy_usb.type = POWER_SUPPLY_TYPE_USB;
	charger->psy_usb.supplied_to = supply_list,
	charger->psy_usb.num_supplicants = ARRAY_SIZE(supply_list),
	charger->psy_usb.properties = max77665_power_props,
	charger->psy_usb.num_properties = ARRAY_SIZE(max77665_power_props),
	charger->psy_usb.get_property = max77665_usb_get_property,
	ret = power_supply_register(&pdev->dev, &charger->psy_usb);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "Failed to power_supply_register psy_usb: %d\n", ret);
		goto err_unregister0;
	}

	charger->psy_ac.name = "ac";
	charger->psy_ac.type = POWER_SUPPLY_TYPE_MAINS;
	charger->psy_ac.supplied_to = supply_list,
	charger->psy_ac.num_supplicants = ARRAY_SIZE(supply_list),
	charger->psy_ac.properties = max77665_power_props,
	charger->psy_ac.num_properties = ARRAY_SIZE(max77665_power_props),
	charger->psy_ac.get_property = max77665_ac_get_property,
	ret = power_supply_register(&pdev->dev, &charger->psy_ac);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "Failed to power_supply_register psy_ac: %d\n", ret);
		goto err_unregister1;
	}
	
	ret = max77665_init(charger);

	alarm_init(&charger->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
				charger_bat_alarm);
	alarm_init(&charger->adjust_alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
				adjust_current_alarm);

	ret = max77665_read_reg(iodev->i2c, MAX77665_CHG_REG_CHG_INT_OK, &reg_data);
	if (unlikely(ret < 0))
		pr_err("Failed to read MAX77665_CHG_REG_CHG_INT: %d\n", ret);
	else {
		if (!(regulator_is_enabled(charger->reverse)) 
				&& (reg_data & 0x40)) {//CHGIN 
			charger->chgin = true;
			schedule_delayed_work_on(0, &charger->dwork, 0);
			set_alarm(&charger->alarm, WAKE_ALARM_INT);
		}
	}

	charger->chgin_irq = pdata->irq_base + MAX77665_CHG_IRQ_CHGIN_I;
	ret = request_threaded_irq(charger->chgin_irq, 0, max77665_charger_isr,
			0, pdev->name, charger);
	if (unlikely(ret < 0)) {
		dev_err(&pdev->dev, "max77665: failed to request CHGIN IRQ %d\n", charger->chgin_irq);
		goto err_unregister2;
	}

	charger->bypass_irq = pdata->irq_base + MAX77665_CHG_IRQ_BYP_I;
	ret = request_threaded_irq(charger->bypass_irq, NULL,
			max77665_bypass_irq, 0, "bypass-irq", charger);
	if (unlikely(ret < 0)) {
		dev_err(&pdev->dev, "max77665: failed to request BYPASS IRQ %d\n", charger->chgin_irq);
		goto err_unregister2;
	}
	return 0;

err_unregister2:
	alarm_cancel(&charger->alarm);
	alarm_cancel(&charger->adjust_alarm);
	power_supply_unregister(&charger->psy_ac);
err_unregister1:	
	power_supply_unregister(&charger->psy_usb);
err_unregister0:	
	power_supply_unregister(&charger->psy_charger);
err_adc_unregister:
	s3c_adc_release(charger->adc);
err_put:
	regulator_put(charger->battery);
err_put0:
	regulator_put(charger->reverse);
err_put1:
	regulator_put(charger->ps);
err_free:
	wake_lock_destroy(&charger->wake_lock);
	platform_set_drvdata(pdev, NULL);
	kfree(charger);
	return ret;
}

static __devexit int max77665_charger_remove(struct platform_device *pdev)
{
	struct max77665_charger *charger = platform_get_drvdata(pdev);

	alarm_cancel(&charger->alarm);
	alarm_cancel(&charger->adjust_alarm);
	cancel_delayed_work_sync(&charger->poll_dwork);

	free_irq(charger->chgin_irq, charger);
	free_irq(charger->bypass_irq, charger);
	regulator_put(charger->battery);
	regulator_put(charger->reverse);
	regulator_put(charger->ps);
	power_supply_unregister(&charger->psy_usb);
	power_supply_unregister(&charger->psy_ac);
	platform_set_drvdata(pdev, NULL);
	kfree(charger);
	return 0;
}

static void max77665_shutdown(struct platform_device *pdev)
{
	struct max77665_charger *charger = platform_get_drvdata(pdev);
	if(regulator_is_enabled(charger->reverse))
		regulator_disable(charger->reverse);
}

#ifdef CONFIG_PM
static int max77665_suspend(struct device *dev)
{
	struct max77665_charger *charger = dev_get_drvdata(dev);

	dev_dbg(charger->dev, "%s\n", __func__);
	cancel_delayed_work_sync(&charger->poll_dwork);

	return 0;
}

static int max77665_resume(struct device *dev)
{
	struct max77665_charger *charger = dev_get_drvdata(dev);

	dev_dbg(charger->dev, "%s\n", __func__);
	schedule_delayed_work_on(0, &charger->poll_dwork, HZ);

	return 0;
}

static const struct dev_pm_ops max77665_pm_ops = {
	.suspend		= max77665_suspend,
	.resume		= max77665_resume,
};
#else
#define max77665_pm_ops NULL
#endif

static struct platform_driver max77665_charger_driver =
{
	.driver = {
		.name  = "max77665-charger", 
		.owner = THIS_MODULE,        
		.pm    = &max77665_pm_ops,   
	},
		.probe    = max77665_charger_probe,               
		.remove   = __devexit_p(max77665_charger_remove), 
		.shutdown = max77665_shutdown,                    
};

static int __init max77665_charger_init(void)
{
	return platform_driver_register(&max77665_charger_driver);
}
late_initcall(max77665_charger_init);

static void __exit max77665_charger_exit(void)
{
	platform_driver_unregister(&max77665_charger_driver);
}
module_exit(max77665_charger_exit);

MODULE_DESCRIPTION("Charger driver for MAX77665");
MODULE_AUTHOR("Lvcha qiu <lvcha@meizu.com>;Chwei <Chwei@meizu.com>");
MODULE_LICENSE("GPLV2");
