/* linux/arch/arm/mach-exynos/cpufreq.c
 *
 * Copyright (C) 2012 ZhuHai MEIZU Technology Co., Ltd.
 *	  http://www.meizu.com
 *	Lvcha Qiu <lvcha@meizu.com>
 *	Li Tao <litao@meizu.com>
 *
 * EXYNOS - CPU frequency scaling support for EXYNOS series
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_qos.h>
#include <linux/performance.h>
#include <linux/exynos-cpufreq.h>

#define CPUFREQ_DEBUG

#ifdef CONFIG_EXYNOS4X12_CPUFREQ
extern int exynos4x12_cpufreq_init(struct exynos_dvfs_info *info);
#else
static int exynos4x12_cpufreq_init(struct exynos_dvfs_info *info) {return 0;}
#endif

#ifdef CONFIG_EXYNOS4210_CPUFREQ
extern int exynos4210_cpufreq_init(struct exynos_dvfs_info *info);
#else
static int exynos4210_cpufreq_init(struct exynos_dvfs_info *info) {return 0;}
#endif

static BLOCKING_NOTIFIER_HEAD(exynos_cpufreq_policy_chain_head);

static unsigned int exynos_get_safe_armvolt(struct exynos_dvfs_info *cpu_info, 
		unsigned int old_index, unsigned int new_index)
{
	unsigned int safe_arm_volt = 0;
	struct cpufreq_frequency_table *freq_table = cpu_info->used_freq_table;
	unsigned int *volt_table = cpu_info->full_volt_table;

	/*
	 * ARM clock source will be changed APLL to MPLL temporary
	 * To support this level, need to control regulator for
	 * reguired voltage level
	 */

	if (cpu_info->need_apll_change != NULL) {
		if (cpu_info->need_apll_change(old_index, new_index) &&
			(freq_table[new_index].frequency < cpu_info->mpll_freq_khz) &&
			(freq_table[old_index].frequency < cpu_info->mpll_freq_khz)) {
				safe_arm_volt = volt_table[cpu_info->pll_safe_idx];
			}
	}

	return safe_arm_volt;
}

static int exynos_verify_speed(struct cpufreq_policy *policy)
{
	struct exynos_dvfs_info *cpu_info = dev_get_drvdata(policy->dev);

	if (IS_ERR_OR_NULL(cpu_info))
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy,
					      cpu_info->used_freq_table);
}

static unsigned int exynos_getspeed(unsigned int cpu)
{
	struct clk *cpu_clk = NULL;
	unsigned long rate;

	cpu_clk = clk_get(NULL, "armclk");
	rate = clk_get_rate(cpu_clk) / 1000;
	clk_put(cpu_clk);

#if 0	//just for test	
	if (rate > 1600*1000)
		rate = 1600*1000;
#endif
	
	return rate;
}

static void regulate_target_freq(struct exynos_dvfs_info *cpu_info, 
		struct cpufreq_policy *policy, unsigned int *target_freq)
{
	int freq = *target_freq;
	
	/* 
	 * ignore qos if not accessable
	 * priority: lock target > qos
	 */
	if (!cpu_info->cpufreq_accessable)
		return;
	
	/* qos min */
	if (cpu_info->qos_min_freq < policy->cpuinfo.min_freq)
		cpu_info->qos_min_freq = policy->cpuinfo.min_freq;
#if 0
	/* do not regulate qos max */
	if (cpu_info->qos_max_freq > policy->cpuinfo.max_freq)
		cpu_info->qos_max_freq = policy->cpuinfo.max_freq;
#endif
	if (freq < cpu_info->qos_min_freq)
		freq = cpu_info->qos_min_freq;

	if (freq > cpu_info->qos_max_freq && 
		cpu_info->qos_min_freq < cpu_info->qos_max_freq) {
		freq = cpu_info->qos_max_freq;
	}
	
	*target_freq = freq;
}

static int exynos_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	static bool first_run = true;
	unsigned int index, old_index;
	unsigned int arm_volt, safe_arm_volt = 0;
	struct exynos_dvfs_info *cpu_info = dev_get_drvdata(policy->dev);
	struct cpufreq_frequency_table *freq_table = NULL;
	unsigned int *volt_table = NULL; 
	int ret = 0;

	if (IS_ERR_OR_NULL(cpu_info))
		return -EINVAL;

	mutex_lock(&cpu_info->freq_lock);

	freq_table = cpu_info->used_freq_table;
	volt_table = cpu_info->full_volt_table;

	if ((relation & ENABLE_FURTHER_CPUFREQ) &&
			(relation & DISABLE_FURTHER_CPUFREQ)) {
		/* Invalidate both if both marked */
		relation &= ~ENABLE_FURTHER_CPUFREQ;
		relation &= ~DISABLE_FURTHER_CPUFREQ;
		pr_err("%s:%d denied marking \"FURTHER_CPUFREQ\""
				" as both marked.\n",
				__FILE__, __LINE__);
	}

	if (relation & ENABLE_FURTHER_CPUFREQ)
		cpu_info->cpufreq_accessable = true;

	if (cpu_info->cpufreq_accessable == false) {
#ifdef CONFIG_CPU_FREQ_DEBUG
		pr_err("%s:%d denied access to %s as it is disabled"
			       " temporarily\n", __FILE__, __LINE__, __func__);
#endif
		ret = -EINVAL;
		goto out;
	}

	if (relation & DISABLE_FURTHER_CPUFREQ)
		cpu_info->cpufreq_accessable = false;
	relation &= ~MASK_FURTHER_CPUFREQ;

	/*depends on qos*/
	regulate_target_freq(cpu_info, policy, &target_freq);

	cpu_info->freqs.old = exynos_getspeed(policy->cpu);
	
	/*Find index*/
	ret = cpufreq_frequency_table_target(policy, freq_table,
						cpu_info->freqs.old,
						relation, &old_index);	
	if (ret < 0) 
		goto out;

	ret = cpufreq_frequency_table_target(policy, freq_table,
					   	target_freq, relation, &index);
	if (ret < 0)	
		goto out;

	cpu_info->freqs.new = freq_table[index].frequency;
	cpu_info->freqs.cpu = policy->cpu;

	/*
	 * Run this function unconditionally until s3c_freqs.freqs.new
	 * and s3c_freqs.freqs.old are both set by this function.
	 */
	if (cpu_info->freqs.new == cpu_info->freqs.old && !first_run)
		goto out;

	pr_debug("%s old: %dMHz L(%d)---->new: %dMhz L(%d)\n", __func__, 
					cpu_info->freqs.old/1000, old_index, 
					cpu_info->freqs.new/1000, index);

	safe_arm_volt = exynos_get_safe_armvolt(cpu_info, old_index, index);
	arm_volt = volt_table[index];

	cpufreq_notify_transition(&cpu_info->freqs, CPUFREQ_PRECHANGE);

	/* When the new frequency is higher than current frequency */
	if ((cpu_info->freqs.new > cpu_info->freqs.old) && !safe_arm_volt) {
		/* Firstly, voltage up to increase frequency */
		regulator_set_voltage(cpu_info->vdd_arm, arm_volt,
				     arm_volt + 25000);
	}

	if (safe_arm_volt)
		regulator_set_voltage(cpu_info->vdd_arm, safe_arm_volt,
				     safe_arm_volt + 25000);

	if (cpu_info->freqs.new != cpu_info->freqs.old)
		cpu_info->set_freq(cpu_info, old_index, index);

	cpufreq_notify_transition(&cpu_info->freqs, CPUFREQ_POSTCHANGE);

	/* When the new frequency is lower than current frequency */
	if ((cpu_info->freqs.new < cpu_info->freqs.old) ||
	   ((cpu_info->freqs.new > cpu_info->freqs.old) && safe_arm_volt)) {
		/* down the voltage after frequency change */
		regulator_set_voltage(cpu_info->vdd_arm, arm_volt,
				     arm_volt + 25000);
	}

	if (first_run)
		first_run = false;
out:
	mutex_unlock(&cpu_info->freq_lock);
	return ret;
}

static int exynos_cpu_init(struct cpufreq_policy *policy)
{
	struct exynos_dvfs_info *cpu_info = dev_get_drvdata(policy->dev);
	int i = 0, ret = 0;

	if (IS_ERR_OR_NULL(cpu_info)) {
		WARN_ON(1);
		ret =  -EINVAL;
		goto cpu_init_err;
	}

	policy->cur = policy->min = policy->max = exynos_getspeed(policy->cpu);

	pr_debug("num_online_cpus = %d, policy->cpu = %d\n",
			num_online_cpus(), policy->cpu);
	do {
		pr_debug("used_freq_table[%d] = %d\n", i,
				cpu_info->used_freq_table[i].frequency);
	} while(cpu_info->used_freq_table[++i].frequency != CPUFREQ_TABLE_END);

	cpufreq_frequency_table_get_attr(cpu_info->used_freq_table, policy->cpu);

	/* set the transition latency value */
	policy->cpuinfo.transition_latency = 100000;
	

	/*
	 * EXYNOS4 multi-core processors has 2 or 4 cores
	 * that the frequency cannot be set independently.
	 * Each cpu is bound to the same speed.
	 * So the affected cpu is all of the cpus.
	 */
	if (num_online_cpus() == 1) {
		cpumask_copy(policy->related_cpus, cpu_possible_mask);
		cpumask_copy(policy->cpus, cpu_online_mask);
	} else {
		cpumask_setall(policy->cpus);
	}

	ret= cpufreq_frequency_table_cpuinfo(policy, cpu_info->used_freq_table);
	if (unlikely(ret)) {
		cpufreq_frequency_table_put_attr(policy->cpu);
		goto cpu_init_err;
	} 
	
	return 0;
cpu_init_err:
	return ret;
}

static int exynos_cpufreq_pm(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct exynos_dvfs_info *cpu_info = 
			list_entry(this, struct exynos_dvfs_info, pm_notifier);
	struct cpufreq_policy *policy;
	unsigned int frequency;
	int ret = NOTIFY_OK;

	if (IS_ERR_OR_NULL(cpu_info))
		return -EINVAL;

	policy = cpufreq_cpu_get(0);
	frequency = cpu_info->pm_lock_freq;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		ret = cpufreq_driver_target(policy, frequency,
				DISABLE_FURTHER_CPUFREQ);
		if (ret < 0)
			ret = NOTIFY_BAD;
		pr_debug("PM_SUSPEND_PREPARE for CPUFREQ\n");
		break;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		pr_debug("PM_POST_SUSPEND for CPUFREQ: %d\n", ret);
		ret = cpufreq_driver_target(policy, frequency,
				ENABLE_FURTHER_CPUFREQ);
		if (ret < 0)
			ret =  NOTIFY_BAD;
		break;
	default:
		ret = NOTIFY_DONE;
		break;
	}

	cpufreq_cpu_put(policy);
	return ret;
}

static int exynos_cpufreq_reboot(struct notifier_block *this,
				   unsigned long code, void *_cmd)
{
	struct exynos_dvfs_info *cpu_info = 
			list_entry(this, struct exynos_dvfs_info, reboot_notifier);
	struct cpufreq_policy *policy;
	int ret = 0;

	if (IS_ERR_OR_NULL(cpu_info))
		return -EINVAL;

	policy = cpufreq_cpu_get(0);
	ret = cpufreq_driver_target(policy, cpu_info->pm_lock_freq,
			DISABLE_FURTHER_CPUFREQ);
	cpufreq_cpu_put(policy);
	if (ret < 0)
		return NOTIFY_BAD;

	printk(KERN_INFO "REBOOT Notifier for CPUFREQ\n");

	return NOTIFY_DONE;
}

static int exynos_update_table(struct exynos_dvfs_info *cpu_info)
{
	int *freq_profile_id = cpu_info->freq_profile_index[cpu_info->pfm_id];
	int i, j;
	int index;

	/* first copy all freqence info to used_freq_table */
	for (i = 0; i<cpu_info->full_freq_num; i++) {
		cpu_info->used_freq_table[i].index = cpu_info->full_freq_table[i].index;
		cpu_info->used_freq_table[i].frequency = cpu_info->full_freq_table[i].frequency;
	}

	/* Sortting availabe index and freq */
	for (i = 0, j = 0; i<cpu_info->full_freq_num-1; i++) {
		if (freq_profile_id[j] != cpu_info->used_freq_table[i].index)
			cpu_info->used_freq_table[i].frequency = CPUFREQ_ENTRY_INVALID;
		else
			j++;
	}

	/* Set all initial voltage at 1.1V */
	for (i = 0; i < cpu_info->volt_table_num; i++) 
		cpu_info->used_volt_table[i] = 1100000;

	/* Set fix voltage */
	for (i=0,j=0; freq_profile_id[i] != L_END; i++) {
		index = freq_profile_id[i];
		if (cpu_info->used_freq_table[index].frequency != CPUFREQ_ENTRY_INVALID) {
			cpu_info->used_volt_table[j] = cpu_info->full_volt_table[index];
			pr_debug("used_volt_table[%d] = %u\n", j, cpu_info->used_volt_table[j]);
			j++;
		}
	}

	return 0;
}

static int exynos_cpufreq_update_voltage(struct exynos_dvfs_info *cpu_info)
{
	int i, ret = 0;
	if (cpu_info->update_dvs_voltage) {
		ret = cpu_info->update_dvs_voltage (cpu_info->r_dev,
							cpu_info->used_volt_table, 
							cpu_info->volt_table_num);
		if (!ret) {
			for (i=0;i<cpu_info->volt_table_num;i++)
				pr_debug("volt_table[%d] = %d\n",
					 i, cpu_info->used_volt_table[i]);
		}
	}
	return ret;
}

int register_exynos_cpufreq_policy_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&exynos_cpufreq_policy_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_exynos_cpufreq_policy_notifier);

int unregister_exynos_cpufreq_policy_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&exynos_cpufreq_policy_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_exynos_cpufreq_policy_notifier);

static int exynos_cpufreq_policy_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&exynos_cpufreq_policy_chain_head, val, NULL)
			== NOTIFY_BAD) ? -EINVAL : 0;
}

static int exynos_update_cpufreq_profile(struct cpufreq_policy *policy)
{
	struct exynos_dvfs_info *cpu_info = dev_get_drvdata(policy->dev);
	int ret;

	if (IS_ERR_OR_NULL(cpu_info))
		return -EINVAL;
	
	ret = cpufreq_frequency_table_cpuinfo(policy, cpu_info->used_freq_table);
	if (!ret) {
		cpufreq_frequency_table_get_attr(cpu_info->used_freq_table, policy->cpu);
		/* Important: updating necessary policy components of user */
		policy->user_policy.max = policy->cpuinfo.max_freq;
		policy->user_policy.min = policy->cpuinfo.min_freq;
		exynos_cpufreq_policy_notifier_call_chain(policy->cpuinfo.max_freq);
	} else {
		pr_err("%s: to do cpufreq_frequency_table_cpuinfo failed\n", __func__);
	}

	return ret;
}

static int exynos_cpufreq_pfm(struct notifier_block *this,
				   unsigned long code, void *_cmd)
{
	struct exynos_dvfs_info *cpu_info = 
			list_entry(this, struct exynos_dvfs_info, pfm_notifier);
	struct cpufreq_policy *policy = NULL;
	int ret = 0;

	if (IS_ERR_OR_NULL(cpu_info))
		return -EINVAL;

	mutex_lock(&cpu_info->pfm_lock);

	if (cpu_info->pfm_id != code) {
		policy = cpufreq_cpu_get(0);
		if (!policy) {
			ret = -EINVAL;
			goto err;
		}

		/* We will lock cpufreq to 200MHZ firstly*/
		ret = cpufreq_driver_target(policy, policy->cpuinfo.min_freq,
						DISABLE_FURTHER_CPUFREQ);
		if (WARN_ON(ret < 0)) {
			cpufreq_cpu_put(policy);
			goto err;
		}
		cpu_info->pfm_id = code;

		/* To prepare new freq table */
		exynos_update_table(cpu_info);

		/* To update pmic voltage table */
		ret = exynos_cpufreq_update_voltage(cpu_info);

		/* To update new freq table */
		ret = exynos_update_cpufreq_profile(policy);

		/* Last we unlock cpufreq */
		ret = cpufreq_driver_target(policy, cpu_info->pm_lock_freq,
						ENABLE_FURTHER_CPUFREQ);

		cpufreq_cpu_put(policy);
	}

err:
	mutex_unlock(&cpu_info->pfm_lock);

	return (ret < 0) ? NOTIFY_DONE : NOTIFY_OK;
}

static int exynos_qos_min_notifier_call(struct notifier_block *nb,
				     unsigned long value, void *devp)
{
	struct exynos_dvfs_info *cpu_info = 
			list_entry(nb, struct exynos_dvfs_info, qos_min_nb);
	
	cpu_info->qos_min_freq = value;
	pr_info("%s: qos_min_freq: %ld\n", __func__, value);
	
	return NOTIFY_OK;
}

static int exynos_qos_max_notifier_call(struct notifier_block *nb,
				     unsigned long value, void *devp)
{
	struct exynos_dvfs_info *cpu_info = 
			list_entry(nb, struct exynos_dvfs_info, qos_max_nb);
	
	cpu_info->qos_max_freq = value;
	pr_info("%s: qos_max_freq: %ld\n", __func__, value);

	return NOTIFY_OK;
}

#ifdef CONFIG_EXYNOS_TMU_TC
static struct exynos_dvfs_info *cpu_info_glb = NULL;
int exynos_find_cpufreq_by_volt(const unsigned int voltage, unsigned int *freq)
{
	int i, tmp_freq = -EINVAL;
	struct exynos_dvfs_info *cpu_info = cpu_info_glb;
	
	if (!cpu_info) {
		pr_err("%s: there is no cpu info\n", __func__);
		return -EINVAL;
	}

	for (i=0; cpu_info->full_freq_table[i].frequency 
			!= CPUFREQ_TABLE_END; i++) {
		/*Find the lowest level*/
		if (cpu_info->full_volt_table[i] < voltage && 
				cpu_info->full_volt_table[i] != 0)
			break;

		tmp_freq = cpu_info->full_freq_table[i].frequency;
	}
	
	if (tmp_freq > 0) {
		*freq = tmp_freq;
		return 0;
	} else {
		pr_err("%s: Can not find suitable cpufreq\n", __func__);
		return -EINVAL;
	}
}
#endif

static int __devinit exynos_cpufreq_probe(struct platform_device *pdev)
{
	struct exynos_cpufreq_platdata *pdata = pdev->dev.platform_data;
	struct exynos_dvfs_info *cpu_info;
	int ret = -EINVAL;

	cpu_info = kzalloc(sizeof(*cpu_info), GFP_KERNEL);
	if (NULL == cpu_info)
		return -ENOMEM;

#ifdef CONFIG_EXYNOS_TMU_TC	
	cpu_info_glb = cpu_info;
#endif
	cpu_info->cputype = platform_get_device_id(pdev)->driver_data;
	cpu_info->dev = &pdev->dev;
	cpu_info->gpio_dvfs = pdata->gpio_dvfs;

	BLOCKING_INIT_NOTIFIER_HEAD(&cpu_info->exynos_dvfs);

	switch (cpu_info->cputype) {
	case TYPE_CPUFREQ_EXYNOS4210:
		ret = exynos4210_cpufreq_init(cpu_info);
		break;
	case TYPE_CPUFREQ_EXYNOS4212:
	case TYPE_CPUFREQ_EXYNOS4412:
		ret = exynos4x12_cpufreq_init(cpu_info);
		break;
	default:
		pr_err("The cpu type is unknow\n");
		ret = -EINVAL;
		goto err_free;
	}

	if (ret || NULL == cpu_info->set_freq) {
		pr_err("Failed to initialize cpu frequnece\n");
		ret = -EINVAL;
		goto err_free;
	}

	cpu_info->vdd_arm = regulator_get(cpu_info->dev, pdata->regulator);
	if (IS_ERR(cpu_info->vdd_arm)) {
		printk(KERN_ERR "failed to get resource %s\n", "vdd_arm");
		ret = PTR_ERR(cpu_info->vdd_arm);
		goto err_free;
	}

	cpu_info->r_dev = regulator_get_dev(cpu_info->vdd_arm);
	if (IS_ERR(cpu_info->r_dev)) {
		pr_err("failed to regulator_get_dev\n");
		ret = PTR_ERR(cpu_info->r_dev);
		goto err_vdd_arm;
	}

	exynos_update_table(cpu_info);
	ret = exynos_cpufreq_update_voltage(cpu_info);

	mutex_init(&cpu_info->freq_lock);
	mutex_init(&cpu_info->pfm_lock);

	cpu_info->cpufreq_accessable = true;
	cpu_info->driver.flags = CPUFREQ_STICKY;
	cpu_info->driver.verify	= exynos_verify_speed;
	cpu_info->driver.target	= exynos_target;
	cpu_info->driver.get	= exynos_getspeed;
	cpu_info->driver.init	= exynos_cpu_init;
	strncpy(cpu_info->driver.name, pdev->name, sizeof(cpu_info->driver.name));

	dev_set_drvdata(cpu_info->dev, cpu_info);

	cpu_info->reboot_notifier.notifier_call = exynos_cpufreq_reboot;
	ret = register_reboot_notifier(&cpu_info->reboot_notifier);
	if (ret) {
		pr_err("failed to register_reboot_notifier\n");
		goto err_notifier1;
	}

	cpu_info->pm_notifier.notifier_call = exynos_cpufreq_pm;
	ret = register_pm_notifier(&cpu_info->pm_notifier);
	if (ret) {
		pr_err("failed to register_pm_notifier\n");
		goto err_notifier2;
	}

	cpu_info->pfm_notifier.notifier_call = exynos_cpufreq_pfm;
	ret = register_pfm_notifier(&cpu_info->pfm_notifier);
	if (ret) {
		pr_err("failed to register_pfm_notifier\n");
		goto err_notifier3;
	}

	/*Add cpufreq qos min notifier*/
	cpu_info->qos_min_freq = PM_QOS_CPUFREQ_MIN_DEFAULT_VALUE;
	cpu_info->qos_min_nb.notifier_call = exynos_qos_min_notifier_call;
	ret = pm_qos_add_notifier(PM_QOS_CPUFREQ_MIN, &cpu_info->qos_min_nb);
	if (ret) {
		pr_err("failed to add cpufreq qos min notifier\n");
		goto err_qos_min_nb;
	}

	/*Add cpufreq qos max notifier*/
	cpu_info->qos_max_freq = PM_QOS_CPUFREQ_MAX_DEFAULT_VALUE;
	cpu_info->qos_max_nb.notifier_call = exynos_qos_max_notifier_call;
	ret = pm_qos_add_notifier(PM_QOS_CPUFREQ_MAX, &cpu_info->qos_max_nb);
	if (ret) {
		pr_err("failed to add cpufreq qos max notifier\n");
		goto err_qos_max_nb;
	}

	ret = cpufreq_register_driver(&cpu_info->driver);
	if (ret) {
		pr_err("failed to register cpufreq driver\n");
		goto err_cpufreq;
	}

	pr_debug("%s done\n", __func__);

	return 0;

err_cpufreq:
	pm_qos_remove_notifier(PM_QOS_CPUFREQ_MAX, &cpu_info->qos_max_nb);
err_qos_max_nb:
	pm_qos_remove_notifier(PM_QOS_CPUFREQ_MIN, &cpu_info->qos_min_nb);
err_qos_min_nb:
	unregister_pfm_notifier(&cpu_info->pfm_notifier);
err_notifier3:
	unregister_pm_notifier(&cpu_info->pm_notifier);
err_notifier2:
	unregister_reboot_notifier(&cpu_info->reboot_notifier);
err_notifier1:
	dev_set_drvdata(cpu_info->dev, NULL);
	cpu_info->r_dev = NULL;
err_vdd_arm:
	regulator_put(cpu_info->vdd_arm);
err_free:
	kfree(cpu_info);
	return ret;
}

/* exynos_cpufreq_remove
 *
 * called when device is removed from the bus
*/
static int __devexit exynos_cpufreq_remove(struct platform_device *pdev)
{
	struct exynos_dvfs_info *cpu_info = platform_get_drvdata(pdev);

	regulator_put(cpu_info->vdd_arm);
	unregister_reboot_notifier(&cpu_info->reboot_notifier);
	unregister_pm_notifier(&cpu_info->pm_notifier);
	unregister_pfm_notifier(&cpu_info->pfm_notifier);
	kfree(cpu_info);

	return 0;
}

#ifdef CONFIG_PM
static int exynos_cpufreq_suspend(struct device *dev)
{
	return 0;
}

static int exynos_cpufreq_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops exynos_cpufreq_dev_pm_ops = {
	.suspend_noirq = exynos_cpufreq_suspend,
	.resume = exynos_cpufreq_resume,
};

#define EXYNOS_DEV_PM_OPS (&exynos_cpufreq_dev_pm_ops)
#else
#define EXYNOS_DEV_PM_OPS NULL
#endif

/* device driver for platform bus bits */

static struct platform_device_id exynos_cpufreq_driver_ids[] = {
	{ "exynos4210-cpufreq", TYPE_CPUFREQ_EXYNOS4210 },
	{ "exynos4212-cpufreq", TYPE_CPUFREQ_EXYNOS4212 },
	{ "exynos4412-cpufreq", TYPE_CPUFREQ_EXYNOS4412 },
	{ }
};
MODULE_DEVICE_TABLE(platform, exynos_cpufreq_driver_ids);

static struct platform_driver exynos_cpufreq_driver = {
	.probe		= exynos_cpufreq_probe,
	.remove		= exynos_cpufreq_remove,
	.id_table	= exynos_cpufreq_driver_ids,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "exynos-cpufreq",
		.pm	= EXYNOS_DEV_PM_OPS,
	},
};

static int __init exynos_cpufreq_init(void)
{
	return platform_driver_register(&exynos_cpufreq_driver);
}
subsys_initcall(exynos_cpufreq_init);

static void __exit exynos_cpufreq_exit(void)
{
	platform_driver_unregister(&exynos_cpufreq_driver);
}
module_exit(exynos_cpufreq_exit);

MODULE_LICENSE("GPLV2");
MODULE_AUTHOR("lvcha qiu <lvcha@meizu.com>");
MODULE_AUTHOR("litao <litao@meizu.com>");
MODULE_DESCRIPTION("EXYNOS CPU Frequency driver");
