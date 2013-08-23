#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <mach/dev.h>
#include <asm/mach-types.h>
#include <plat/cpu.h>
#include <linux/sysdev.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/err.h>

#define QM_MINORS		2
#define QM_HOME_WAIT_TIMEOUT	150//ms

struct qm_filter_info {
	struct input_dev *input_dev;
	struct input_handle handle;
};

struct qm_filter_global_data {
	int qm_filter_count;
	struct qm_filter_info *qm_table[QM_MINORS];
	
	unsigned int left;
	unsigned int right;
	unsigned int top;
	unsigned int bottom;

	/*touch infomation*/
	int touch;
	int finger;
	int position_x;
	int position_y;

	int home_key_drop;
	int home_key_status;
	int touch_status;

	int back_key_report;
	
	ktime_t touch_time;
	ktime_t key_time;
};

static struct qm_filter_global_data *qm_filter_data = NULL;

static bool qm_filter_in_rect(struct qm_filter_global_data *global_data, unsigned int x, unsigned int y)
{
	if( (x >= global_data->left) &&  (x <= global_data->right) && (y >= global_data->top) && (y <= global_data->bottom))
	{
		return true;
	}
	return false;
}

static bool qm_filter_check_home(struct qm_filter_global_data *global_data, int value)
{
	ktime_t delta_total;
	long long delta_ms = 0;
	
	delta_total= ktime_sub(ktime_get(), global_data->touch_time);
	delta_ms = ktime_to_ms(delta_total);	

	if(delta_ms > QM_HOME_WAIT_TIMEOUT){
		global_data->home_key_drop = 1;
		return false;
	}
	else if(global_data->home_key_drop && global_data->home_key_status==0){
		global_data->home_key_drop = 0;
		return false;
	}
	global_data->home_key_drop = 0;
	return true;
}

static bool qm_filter_filter(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	struct qm_filter_global_data *global_data = (struct qm_filter_global_data *)handle->private;

	switch(code){
	case KEY_HOME:
		global_data->home_key_status = value;
		global_data->key_time = ktime_get();
		return qm_filter_check_home(global_data, value);
	case ABS_MT_TRACKING_ID:
		global_data->finger = value;
		break;
	case ABS_MT_POSITION_X:
		global_data->position_x= value;
		break;
	case ABS_MT_POSITION_Y:
		global_data->position_y= value;
		break;
	case SYN_REPORT:
		if(test_bit(EV_ABS, handle->dev->evbit) && qm_filter_in_rect(global_data, global_data->position_x, global_data->position_y))
			global_data->touch_time = ktime_get();
		break;
	default:
		break;
	}
	return false;
}
	
static bool  qm_filter_match(struct input_handler *handler, struct input_dev *dev)
{
	/* Synaptics touchscreens is valid*/
	if (test_bit(EV_ABS, dev->evbit) && test_bit(ABS_MT_POSITION_X, dev->absbit))
		return true;

	/*meizu touchpads is valid*/
	if (test_bit(EV_KEY, dev->evbit) && test_bit(KEY_HOME, dev->keybit) && dev->id.vendor == 0x1111)
		return true;

	return false;
}
static int qm_filter_connect(struct input_handler *handler,
				    struct input_dev *dev,
				    const struct input_device_id *id)
{
	int ret;
	struct qm_filter_info *info;
	struct qm_filter_global_data *global_data = (struct qm_filter_global_data *)handler->private;
	
	if(global_data->qm_filter_count >= QM_MINORS)
		return -ENODEV;
	
	info = kzalloc(sizeof(struct qm_filter_info), GFP_KERNEL);
	if (IS_ERR_OR_NULL(info))
		return PTR_ERR(info);

	info->input_dev = dev;
	info->handle.dev = dev;
	info->handle.handler = handler;
	info->handle.private = global_data;
	info->handle.open = 0;
	info->handle.name = dev_name(&info->input_dev->dev);
	ret = input_register_handle(&info->handle);
	if (ret) {
		pr_err("%s: register input handler error!\n", __func__);
		goto err_reg;
	}
	ret = input_open_device(&info->handle);
	if (ret) {
		pr_err("%s: Failed to open input device, error %d\n",
			__func__, ret);
		goto err_open;
	}

	if(test_bit(EV_ABS, dev->evbit))
	{
		global_data->left = 400;
		global_data->right = 800;
		global_data->top  = 1700;
		global_data->bottom = 1900;
		if(dev->absinfo){
			global_data->left = dev->absinfo[ABS_MT_POSITION_X].minimum + (dev->absinfo[ABS_MT_POSITION_X].maximum - dev->absinfo[ABS_MT_POSITION_X].minimum)/4;
			global_data->right = dev->absinfo[ABS_MT_POSITION_X].maximum - global_data->left;
			global_data->top = dev->absinfo[ABS_MT_POSITION_Y].minimum+ (dev->absinfo[ABS_MT_POSITION_Y].maximum - dev->absinfo[ABS_MT_POSITION_Y].minimum)/10*9;
			global_data->bottom = dev->absinfo[ABS_MT_POSITION_Y].maximum;
		}
		pr_info("qm_filter_connect left=%d, right=%d, top=%d, bottom=%d\n", global_data->left, global_data->right, global_data->top, global_data->bottom);
	}

	global_data->qm_filter_count++;
	return 0;

err_open:
	input_unregister_handle(&info->handle);
err_reg:
	kfree(info);
	return ret;

	
}

static void qm_filter_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
}

static const struct input_device_id qm_filter_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = {[BIT_WORD(ABS_MT_POSITION_X)] = BIT_MASK(ABS_MT_POSITION_X) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = {[BIT_WORD(KEY_HOME)] = BIT_MASK(KEY_HOME) },
	},
	{ }	/* Terminating entry */
};

static struct input_handler qm_filter_handler = {
	.filter		= qm_filter_filter,
	.match		= qm_filter_match,
	.connect		=  qm_filter_connect,
	.disconnect	= qm_filter_disconnect,
	.name		= "qm_filter_keyboard",
	.id_table		= qm_filter_ids,
};

static int __init qm_filter_init(void)
{
	int ret ;
	struct qm_filter_global_data *global_data;
		
	global_data = kzalloc(sizeof(struct qm_filter_global_data), GFP_KERNEL);
	if (IS_ERR_OR_NULL(global_data))
		return PTR_ERR(global_data);
	global_data->touch_time = ktime_get();

	qm_filter_handler.private = global_data;
	ret = input_register_handler(&qm_filter_handler);
	if (ret) {
		pr_err("Unable to register input handler, error: %d\n",
			ret);
		goto fail;
	}
	qm_filter_data = global_data;
	return 0;
fail:
	kfree(global_data);
	return ret;
}

static void __exit qm_filter_exit(void)
{
	input_unregister_handler(&qm_filter_handler);
	kfree(qm_filter_data);}

module_init(qm_filter_init);
module_exit(qm_filter_exit);

/* Module information */
MODULE_AUTHOR("Wenbin Wu <wenbinwu@meizu.com>");
MODULE_DESCRIPTION("QM Touchpad filter driver");
MODULE_LICENSE("GPL");


