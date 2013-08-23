#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <mach/dev.h>
#include <asm/mach-types.h>
#include <plat/cpu.h>
#ifdef CONFIG_FB_DYNAMIC_FREQ
#include <linux/fb.h>
#endif
#include <mach/touch_booster.h>

#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/err.h>

#define DEFUALT_BOOST_CPUFREQ		800000//800mhz
#define DEFUALT_BOOST_FREETIME	1000//ms
#define DEFUALT_BOOST_4210_BUSFREQ		267000
#define DEFUALT_BOOST_4412_BUSFREQ		267200

static int boost_device_count = 0;

static void start_vsync_boost(struct tb_private_info *info)
{
	int ret = 0;

#ifdef CONFIG_FB_DYNAMIC_FREQ
	do {
		struct fb_event event;
		event.data = info;
		ret = fb_notifier_call_chain(FB_EVENT_MODE_PAN, &event);
	} while(0);
#endif
}
static ssize_t set_vsync_pulse(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int pulse = 0;

	sscanf(buf, "%u", &pulse);
	if(pulse)
		start_vsync_boost(info);
	return count; 
}
static SYSDEV_CLASS_ATTR(vsync_pulse, 0644, NULL, set_vsync_pulse);

static void start_touch_boost(struct tb_private_info *info)
{
	int ret = 0;
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	ktime_t delta_total, rettime_total;
	long long delta_us = 0;

	if(info->boost_debug)
		rettime_total = ktime_get();
	if (policy) {
		if (policy->cur < info->boost_cpufreq) {
			ret = cpufreq_driver_target(policy, info->boost_cpufreq, CPUFREQ_RELATION_L);
			pr_debug("dev_lock\n");
#ifdef CONFIG_BUSFREQ_OPP
			dev_lock_timeout(info->bus_dev, &info->dev, info->lock_busfreq, info->down_time);
#endif
		}
		cpufreq_cpu_put(policy);
	}
	if(info->boost_debug){
		delta_total= ktime_sub(ktime_get(),rettime_total);
		delta_us = ktime_to_us(delta_total);
		pr_info("start_touch_boost time = %Lu uS, ret = %d\n", delta_us, ret);
	}
}
static ssize_t set_touch_pulse(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int pulse = 0;

	sscanf(buf, "%u", &pulse);
	if(pulse)
		start_touch_boost(info);
	return count; 
}
static SYSDEV_CLASS_ATTR(touch_pulse, 0644, NULL, set_touch_pulse);

static ssize_t get_boost_cpufreq(struct sysdev_class *class,
				struct sysdev_class_attribute *attr, char *buf)
{
	struct tb_private_info *data = list_entry(class, struct tb_private_info, tb_class);
	return sprintf(buf,"%d\n", data->boost_cpufreq);
}

static ssize_t set_boost_cpufreq(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int cpurate = 0;

	sscanf(buf, "%u", &cpurate);
	info->boost_cpufreq = cpurate;

	return count; 
}

static SYSDEV_CLASS_ATTR(boost_cpufreq, 0644, get_boost_cpufreq, set_boost_cpufreq);

static ssize_t get_boost_debug(struct sysdev_class *class,
				struct sysdev_class_attribute *attr, char *buf)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	return sprintf(buf,"%d\n", info->boost_debug);
}

static ssize_t set_boost_debug(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int boost_debug = 0;

	sscanf(buf, "%u", &boost_debug);
	info->boost_debug = !!boost_debug;

	return count; 
}

static SYSDEV_CLASS_ATTR(boost_debug, 0644, get_boost_debug, set_boost_debug);

static ssize_t get_lock_busfreq(struct sysdev_class *class,
				struct sysdev_class_attribute *attr, char *buf)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	return sprintf(buf,"%d\n", info->lock_busfreq);
}

static ssize_t set_lock_busfreq(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int busrate = 0;

	sscanf(buf, "%u", &busrate);
	info->lock_busfreq = busrate;

	return count; 
}
static SYSDEV_CLASS_ATTR(lock_busfreq, 0644, get_lock_busfreq, set_lock_busfreq);

static ssize_t get_lock_time(struct sysdev_class *class,
				struct sysdev_class_attribute *attr, char *buf)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	return sprintf(buf,"%d ms\n", info->down_time);
}

static ssize_t set_lock_time(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int time = 0;

	sscanf(buf, "%u", &time);

	if(time<100)
		time=100;
	
	info->down_time = time;

	return count; 
}

static SYSDEV_CLASS_ATTR(lock_time, 0644, get_lock_time, set_lock_time);

static struct sysdev_class_attribute *tb_sysdev_class_attrs[] = {
	&attr_boost_debug,
	&attr_vsync_pulse,
	&attr_touch_pulse,
	&attr_boost_cpufreq,
	&attr_lock_busfreq,
	&attr_lock_time,
	NULL
};

static void tb_boost_fn(struct work_struct *work)
{
	struct tb_private_info *info=
		list_entry(work, struct tb_private_info, boost_work);
	start_touch_boost(info);
}

static void tb_event(struct input_handle *handle, 
	unsigned int type, unsigned int code, int value)
{
	struct tb_private_info *info = (struct tb_private_info*)handle->private;

	/* To queue work when touch device and press only */
	if (code == BTN_TOUCH && value == 1)
		queue_work(info->wq, &info->boost_work);
	if (code == KEY_POWER && value == 1)
		queue_work(info->wq, &info->boost_work);
	if (code == KEY_HOME && value == 1)
		queue_work(info->wq, &info->boost_work);
	if (code == KEY_AGAIN && value == 1)
		queue_work(info->wq, &info->boost_work);
}
static bool  tb_match(struct input_handler *handler, struct input_dev *dev)
{
	/* Avoid touchpads and touchscreens */
	if (test_bit(EV_ABS, dev->evbit) && test_bit(BTN_TOUCH, dev->keybit))
		return true;
	/* Avoid gpio keyboard */
	if (test_bit(EV_KEY, dev->evbit) && test_bit(KEY_POWER, dev->keybit) && dev->id.vendor == 0x0001)
		return true;
	/* Avoid touchpad */
	if (test_bit(EV_KEY, dev->evbit) && test_bit(KEY_HOME, dev->keybit) && dev->id.vendor ==0x1111)
		return true;
	return false;
}
static int tb_connect(struct input_handler *handler,
				    struct input_dev *dev,
				    const struct input_device_id *id)
{
	int ret;
	int num;
	struct tb_private_info *info = handler->private;

	if(boost_device_count >= MAX_BOOST_DEVICE)
		return -ENODEV;
	num = boost_device_count;
	info->handle[num].dev = dev;
	info->handle[num].handler = handler;
	info->handle[num].open = 0;
	info->handle[num].name = "touchbooster";
	info->handle[num].private = info;
	ret = input_register_handle(&info->handle[num]);
	if (ret) {
		pr_err("%s: register input handler error!\n", __func__);
		goto err_reg;
	}
	ret = input_open_device(&info->handle[num]);
	if (ret) {
		pr_err("%s: Failed to open input device, error %d\n",
			__func__, ret);
		goto err_open;
	}
	boost_device_count++;
	dev_info(&info->dev, "Register input handler for %s\n", dev_name(&dev->dev));
	return 0;

err_open:
	input_unregister_handle(&info->handle[num]);
err_reg:
	return ret;

	
}

static void tb_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	boost_device_count--;
}

static const struct input_device_id tb_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.keybit = {[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
	},{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT  | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = {[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER) },
	},{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT  | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = {[BIT_WORD(KEY_HOME)] = BIT_MASK(KEY_HOME) },
	},
	{ }	/* Terminating entry */
};

static struct input_handler tb_handler = {
	.event		= tb_event,
	.match		= tb_match,
	.connect		=  tb_connect,
	.disconnect	= tb_disconnect,
	.name		= "touchbooster",
	.id_table		= tb_ids,
};

static int __init tb_init(void)
{
	int ret ;
	struct tb_private_info *info;
		
	info = kzalloc(sizeof(struct tb_private_info), GFP_KERNEL);
	if (IS_ERR_OR_NULL(info))
		return PTR_ERR(info);

	info->bus_dev = dev_get("exynos-busfreq");
	info->wq = create_singlethread_workqueue("touchbooster");
	if (!info->wq) {
		pr_err("%s: Failed to create_singlethread_workqueue\n", __func__);
		ret = -ENOMEM;
		goto  err_create;
	}
	
	INIT_WORK(&info->boost_work, tb_boost_fn);
	info->key_time= ktime_get();
	info->down_time = DEFUALT_BOOST_FREETIME;//ms
	info->boost_cpufreq = DEFUALT_BOOST_CPUFREQ;

#ifdef CONFIG_BUSFREQ_OPP
	if (soc_is_exynos4210())
		info->lock_busfreq = DEFUALT_BOOST_4210_BUSFREQ;
	else
		info->lock_busfreq = DEFUALT_BOOST_4412_BUSFREQ;

	device_initialize(&info->dev);
	dev_set_name(&info->dev, "touchbooster");
	ret = device_add(&info->dev);
	if (ret < 0) {
		printk(KERN_ERR "can't %s %s, status %d\n",
				"add", dev_name(&info->dev), ret);
		goto err_device;
	}
#endif

	info->tb_class.name = "touchbooster";
	info->tb_class.attrs = tb_sysdev_class_attrs;
	ret = sysdev_class_register(&info->tb_class);
	if (ret) {
		pr_err("%s: register sysdev performance error!\n", __func__);
		goto err_sysdev;
	}
	
	tb_handler.private = info;
	ret = input_register_handler(&tb_handler);
	if (ret) {
		pr_err("Unable to register input handler, error: %d\n",
			ret);
		goto err_handler;
	}
	return 0;
err_handler:
	input_unregister_handler(&tb_handler);
#ifdef CONFIG_BUSFREQ_OPP
err_sysdev:
	device_del(&info->dev);
#endif
err_device:
	destroy_workqueue(info->wq);
err_create:
	kfree(info);
	return ret;
}

static void __exit tb_exit(void)
{
	struct tb_private_info *info = tb_handler.private;

	input_unregister_handler(&tb_handler);
#ifdef CONFIG_BUSFREQ_OPP
	device_del(&info->dev);
#endif
	destroy_workqueue(info->wq);
	kfree(info);
}

subsys_initcall(tb_init);
module_exit(tb_exit);

/* Module information */
MODULE_AUTHOR("Lvcha qiu <lvcha@meizu.com>");
MODULE_AUTHOR("Wenbin Wu <wenbinwu@meizu.com>");
MODULE_DESCRIPTION("Touch Boost driver");
MODULE_LICENSE("GPL");

