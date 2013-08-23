/* linux/drivers/thermal/exynos_thermal.c
  *
  * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
  *		http://www.samsung.com
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  */

#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/pm_qos.h>
#include <linux/cpu_cooling.h>
#include <linux/exynos_thermal.h>

#include <plat/cpu.h>

#include <mach/smc.h>
#include <mach/map.h>

#ifdef CONFIG_EXYNOS_TMU_TC
#include <linux/exynos-cpufreq.h>
#include <mach/dev.h>
#include <mach/busfreq_exynos4.h>

extern int mali_voltage_lock_init(void);
extern int mali_voltage_lock_deinit(void);
extern int mali_voltage_lock_push(int lock_vol);
extern int mali_voltage_lock_pop(void);
#endif


#define MAX_COOLING_DEVICE 4
#define COOLING_DEVICE_NUM 3

struct exynos4_thermal_zone {
	unsigned long user_temp;
	unsigned int idle_interval;
	unsigned int active_interval;
	struct thermal_zone_device *therm_dev;
	struct thermal_cooling_device *cool_dev[MAX_COOLING_DEVICE];
	unsigned int cool_dev_size;
	struct thermal_sensor_conf *sensor_conf;
};

static struct exynos4_thermal_zone *th_zone;
static BLOCKING_NOTIFIER_HEAD(exynos_tmu_chain_head);

static int exynos4_get_mode(struct thermal_zone_device *thermal,
			    enum thermal_device_mode *mode)
{
	if (!th_zone->sensor_conf) {
		pr_info("Temperature sensor not initialised\n");
		*mode = THERMAL_DEVICE_DISABLED;
	} else
		*mode = THERMAL_DEVICE_ENABLED;
	return 0;
}

static int exynos4_set_mode(struct thermal_zone_device *thermal,
			    enum thermal_device_mode mode)
{
	if (!th_zone->therm_dev) {
		pr_notice("thermal zone not registered\n");
		return 0;
	}
	if (mode == THERMAL_DEVICE_ENABLED)
		th_zone->therm_dev->polling_delay =
				th_zone->active_interval;
	else
		th_zone->therm_dev->polling_delay =
				th_zone->idle_interval;

	thermal_zone_device_update(th_zone->therm_dev);
	pr_info("thermal polling set for duration=%d ms\n",
				th_zone->therm_dev->polling_delay);
	return 0;
}

/*This may be called from interrupt based temperature sensor*/
void exynos4_report_trigger(void)
{
	unsigned int monitor_temp;

	if (!th_zone || !th_zone->therm_dev)
		return;

	monitor_temp = th_zone->sensor_conf->trip_data.trip_val[0];

	thermal_zone_device_update(th_zone->therm_dev);

	mutex_lock(&th_zone->therm_dev->lock);
	if (th_zone->therm_dev->last_temperature > monitor_temp)
		th_zone->therm_dev->polling_delay =
					th_zone->active_interval;
	else
		th_zone->therm_dev->polling_delay =
					th_zone->idle_interval;
	kobject_uevent(&th_zone->therm_dev->device.kobj, KOBJ_CHANGE);
	mutex_unlock(&th_zone->therm_dev->lock);
}

static int exynos4_get_trip_type(struct thermal_zone_device *thermal, int trip,
				 enum thermal_trip_type *type)
{
	switch (trip) {
		case 0 ... 2:
			*type = THERMAL_TRIP_STATE_ACTIVE;
			break;
		case 3: 
			*type = THERMAL_TRIP_CRITICAL;
			break;
		case 4:
			*type = THERMAL_TRIP_START_COLD_TC;
			break;
		case 5:
			*type = THERMAL_TRIP_STOP_COLD_TC;
			break;
		case 6:
			*type = THERMAL_TRIP_START_MEM_TH;
			break;
		case 7:
			*type = THERMAL_TRIP_STOP_MEM_TH;
			break;
		default:
			pr_err("%s: error trip %d\n", __func__, trip);
			return -EINVAL;
	}

	return 0;
}

static int exynos4_get_trip_temp(struct thermal_zone_device *thermal, int trip,
				 unsigned long *temp)
{
	switch (trip) {
		case 0 ... 3:
			*temp = th_zone->sensor_conf->trip_data.trip_val[trip];
			break;
		case 4:
			*temp = th_zone->sensor_conf->tc_data.start_tc;
			break;
		case 5:
			*temp = th_zone->sensor_conf->tc_data.stop_tc;
			break;
		case 6:
			*temp = th_zone->sensor_conf->tc_data.start_mem_th;
			break;
		case 7:
			*temp = th_zone->sensor_conf->tc_data.stop_mem_th;
			break;
		default:
			pr_err("%s error trip %d\n", __func__, trip);
			return -EINVAL;
	}
	/*convert the temperature into millicelsius*/
	*temp = *temp * 1000;

	return 0;
}

static int exynos4_get_crit_temp(struct thermal_zone_device *thermal,
				 unsigned long *temp)
{
	/*Panic zone*/
	*temp = th_zone->sensor_conf->trip_data.trip_val[3];
	/*convert the temperature into millicelsius*/
	*temp = *temp * 1000;
	return 0;
}

static int exynos4_bind(struct thermal_zone_device *thermal,
			struct thermal_cooling_device *cdev)
{
	int i;
	/* if the cooling device is the one from exynos4 bind it */
	if (cdev != th_zone->cool_dev[0])
		return 0;
	
	for (i=0; i<COOLING_DEVICE_NUM; i++) {
		if (thermal_zone_bind_cooling_device(thermal, i, cdev)) {
			pr_err("error binding cooling dev\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int exynos4_unbind(struct thermal_zone_device *thermal,
			  struct thermal_cooling_device *cdev)
{
	int i;

	if (cdev != th_zone->cool_dev[0])
		return 0;
	
	for (i=0; i<COOLING_DEVICE_NUM; i++) {
		if (thermal_zone_unbind_cooling_device(thermal, i, cdev)) {
			pr_err("error unbinding cooling dev\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int exynos4_get_temp(struct thermal_zone_device *thermal,
			       unsigned long *temp)
{
	void *data;

	if (!th_zone->sensor_conf) {
		pr_debug("Temperature sensor not initialised\n");
		return -EINVAL;
	}

	if (th_zone->user_temp) {
		*temp = th_zone->user_temp;
		pr_debug("user_temp mode\n");
	} else {
		data = th_zone->sensor_conf->private_data;
		*temp = th_zone->sensor_conf->read_temperature(data);
		/*convert the temperature into millicelsius*/
		*temp = *temp * 1000;
	}

	pr_debug("%s: temp is :%lu\n", __func__, *temp);
	
	return 0;
}

/*tmu mem throttle refresh*/
static unsigned int get_refresh_interval(unsigned int freq_ref,
					unsigned int refresh_nsec)
{
	unsigned int uRlk, refresh = 0;

	/*
	 * uRlk = FIN / 100000;
	 * refresh_usec =  (unsigned int)(fMicrosec * 10);
	 * uRegVal = ((unsigned int)(uRlk * uMicroSec / 100)) - 1;
	 * refresh =
	 * (unsigned int)(freq_ref * (unsigned int)(refresh_usec * 10) /
	 * 100) - 1;
	*/
	uRlk = freq_ref / 1000000;
	refresh = ((unsigned int)(uRlk * refresh_nsec / 1000));

	pr_info("@@@ get_refresh_interval = 0x%02x\n", refresh);
	return refresh;
}

static void set_refresh_rate(unsigned int auto_refresh)
{
	/*
	 * uRlk = FIN / 100000;
	 * refresh_usec =  (unsigned int)(fMicrosec * 10);
	 * uRegVal = ((unsigned int)(uRlk * uMicroSec / 100)) - 1;
	*/
	pr_debug("@@@ set_auto_refresh = 0x%02x\n", auto_refresh);

	if (trustzone_on()) {
		exynos_smc(SMC_CMD_REG,
			SMC_REG_ID_SFR_W((EXYNOS4_PA_DMC0_4212 + TIMING_AREF_OFFSET)),
			auto_refresh, 0);
		exynos_smc(SMC_CMD_REG,
			SMC_REG_ID_SFR_W((EXYNOS4_PA_DMC1_4212 + TIMING_AREF_OFFSET)),
			auto_refresh, 0);
	} else {
		/* change auto refresh period in TIMING_AREF register of dmc0  */
		__raw_writel(auto_refresh, S5P_VA_DMC0 + TIMING_AREF_OFFSET);

		/* change auto refresh period in TIMING_AREF regisger of dmc1 */
		__raw_writel(auto_refresh, S5P_VA_DMC1 + TIMING_AREF_OFFSET);
	}
}

static int exynos4_notify(struct thermal_zone_device *thermal,int val,
				enum thermal_trip_type trip_type)
{
#ifdef CONFIG_EXYNOS_TMU_TC
	int ret;
	unsigned int cpufreq;
	unsigned long busfreq;
	unsigned int arm_volt = 0, bus_volt = 0, g3d_volt = 0;
	unsigned int refresh_rate;
	
	switch (trip_type) {
	case THERMAL_TRIP_START_COLD_TC:
		pr_info("THERMAL_TRIP_COLD_VC:\n");
		
		/*arm*/
		arm_volt = th_zone->sensor_conf->tc_data.arm_volt;
		ret = exynos_find_cpufreq_by_volt(arm_volt, &cpufreq);
		if (ret < 0) {
			pr_err("%s: Find cpufreq erro\n", __func__);
			goto err_lock;
		}
		pm_qos_update_request(&thermal->qos_cpu_tmu_tc, cpufreq);
		
		/*bus*/
		bus_volt = th_zone->sensor_conf->tc_data.bus_volt;
		ret = exynos4x12_find_busfreq_by_volt(bus_volt, &busfreq);
		if (ret < 0) {
			pr_err("%s: Find busfreq erro\n", __func__);
			goto err_lock;
		}

		ret = dev_lock(th_zone->therm_dev->bus_dev, 
			&th_zone->therm_dev->device, busfreq);
		if (ret) {
			pr_err("TMU: Bus lock erro\n");
			goto err_lock;
		}

		/*g3d*/
		g3d_volt = th_zone->sensor_conf->tc_data.g3d_volt;
		ret = mali_voltage_lock_push(g3d_volt);
		if (ret < 0) {
			pr_err("TMU: g3d_push error: %u uV\n", g3d_volt);
			goto err_lock;
		}
		break;
	case THERMAL_TRIP_STOP_COLD_TC:	
		pr_info("THERMAL_TRIP_EXIT_COLD_VC:\n");
		pm_qos_update_request(&thermal->qos_cpu_tmu_tc, 
					PM_QOS_DEFAULT_VALUE);
	
		ret = dev_unlock(th_zone->therm_dev->bus_dev,
				&th_zone->therm_dev->device);
		if (ret) {
			pr_err("TMU: Bus unlock erro\n");
			goto err_unlock;
		}

		ret = mali_voltage_lock_pop();
		if (ret < 0) {
			pr_err("TMU: g3d_pop error\n");
			goto err_unlock;
		}
		break;
	case THERMAL_TRIP_START_MEM_TH:
		pr_info("THERMAL_TRIP_START_MEM_TH\n");
		refresh_rate = get_refresh_interval(FREQ_IN_PLL,
				 AUTO_REFRESH_PERIOD_TQ0);
		set_refresh_rate(refresh_rate);
		break;
	case THERMAL_TRIP_STOP_MEM_TH:
		pr_info("THERMAL_TRIP_STOP_MEM_TH\n");
		refresh_rate = get_refresh_interval(FREQ_IN_PLL, 
				AUTO_REFRESH_PERIOD_NORMAL);
		set_refresh_rate(refresh_rate);
		break;
	default:
		pr_debug("%s: Haven't implemented for type %d\n",
					 __func__, trip_type);
	}

err_lock:
err_unlock:
#endif
	return 0;
}

static ssize_t exynos4_user_temp_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", th_zone->user_temp);
}

static ssize_t exynos4_user_temp_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long user_temp = 0;

	sscanf(buf, "%lu\n", &user_temp);
	
	th_zone->user_temp = user_temp*1000;

	return count;
}

/*usr_temp: for test*/
static DEVICE_ATTR(user_temp, 0644,exynos4_user_temp_show,
		   exynos4_user_temp_store);


/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops exynos4_dev_ops = {
	.bind = exynos4_bind,
	.unbind = exynos4_unbind,
	.get_temp = exynos4_get_temp,
	.get_mode = exynos4_get_mode,
	.set_mode = exynos4_set_mode,
	.get_trip_type = exynos4_get_trip_type,
	.get_trip_temp = exynos4_get_trip_temp,
	.get_crit_temp = exynos4_get_crit_temp,
	.notify = exynos4_notify,
};

int exynos4_register_thermal(struct thermal_sensor_conf *sensor_conf)
{
	int ret, count, tab_size;
	int trip_num = 4;

	struct freq_pctg_table *tab_ptr;

	if (!sensor_conf || !sensor_conf->read_temperature) {
		pr_err("Temperature sensor not initialised\n");
		return -EINVAL;
	}

	th_zone = kzalloc(sizeof(struct exynos4_thermal_zone), GFP_KERNEL);
	if (!th_zone) {
		ret = -ENOMEM;
		goto err_unregister;
	}

	th_zone->sensor_conf = sensor_conf;

	tab_ptr = (struct freq_pctg_table *)sensor_conf->cooling_data.freq_data;
	tab_size = sensor_conf->cooling_data.freq_pctg_count;

	/*Register the cpufreq cooling device*/
	th_zone->cool_dev_size = 1;
	count = 0;
	th_zone->cool_dev[count] = cpufreq_cooling_register(
			(struct freq_pctg_table *)&(tab_ptr[count]),
			tab_size, cpumask_of(0));

	if (IS_ERR(th_zone->cool_dev[count])) {
		pr_err("Failed to register cpufreq cooling device\n");
		ret = -EINVAL;
		th_zone->cool_dev_size = 0;
		goto err_unregister;
	}
	
#ifdef CONFIG_EXYNOS_TMU_TC
	if (soc_is_exynos4212() || soc_is_exynos4412())
		trip_num = 8;
#endif
	th_zone->therm_dev = thermal_zone_device_register(sensor_conf->name,
				trip_num, NULL, &exynos4_dev_ops, 0, 0, 0, 1000);
	if (IS_ERR(th_zone->therm_dev)) {
		pr_err("Failed to register thermal zone device\n");
		ret = -EINVAL;
		goto err_unregister;
	}

	th_zone->active_interval = 200; //200ms
	th_zone->idle_interval = 2*1000;  //2s

	exynos4_set_mode(th_zone->therm_dev, THERMAL_DEVICE_DISABLED);
	
	ret = device_create_file(&th_zone->therm_dev->device, &dev_attr_user_temp);
	if (ret)
		goto err_unregister;

#ifdef CONFIG_EXYNOS_TMU_TC	
	pm_qos_add_request(&th_zone->therm_dev->qos_cpu_tmu_tc, 
				PM_QOS_CPUFREQ_MIN, PM_QOS_DEFAULT_VALUE);
	th_zone->therm_dev->bus_dev = dev_get("exynos-busfreq");

	ret = mali_voltage_lock_init();
	if (ret) {
		pr_err("Failed to initialize mail voltage lock.\n");
		goto err_unregister;
	}

	th_zone->therm_dev->initialized = true;
#endif

	pr_debug("Exynos: Kernel Thermal management registered\n");

	return 0;

err_unregister:
	exynos4_unregister_thermal();
	return ret;
}
EXPORT_SYMBOL(exynos4_register_thermal);

void exynos4_unregister_thermal(void)
{
	unsigned int i;

	for (i = 0; i < th_zone->cool_dev_size; i++) {
		if (th_zone && th_zone->cool_dev[i])
			cpufreq_cooling_unregister(th_zone->cool_dev[i]);
	}

	if (th_zone && th_zone->therm_dev)
		thermal_zone_device_unregister(th_zone->therm_dev);
#ifdef CONFIG_EXYNOS_TMU_TC	
	pm_qos_remove_request(&th_zone->therm_dev->qos_cpu_tmu_tc);
	mali_voltage_lock_deinit();
#endif
	kfree(th_zone);

	pr_info("Exynos: Kernel Thermal management unregistered\n");
}
EXPORT_SYMBOL(exynos4_unregister_thermal);
