/* drivers/modem/modemctl_device_xmm6260.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/platform_data/modem.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>

#include <mach/gpio.h>
#include "modem_prj.h"

static char modem_event_str[8][25] = {
	"MODEM_EVENT_POWEROFF",
	"MODEM_EVENT_RESET",
	"MODEM_EVENT_CRASH",
	"MODEM_EVENT_DUMP",
	"MODEM_EVENT_CONN",
	"MODEM_EVENT_DISCONN",
	"MODEM_EVENT_SIM",
	"MODEM_EVENT_BOOT_INIT",
};

#ifdef CONFIG_USB_EHCI_S5P
extern int s5p_ehci_port_power(int state);
extern int  s5p_ehci_power(int value);
#else
int  s5p_ehci_power(int value){return 0;}
int s5p_ehci_port_power(int state){return 0;}
#endif

static struct modem_ctl *global_mc = NULL;
DEFINE_SEMAPHORE(modem_downlock);

static int xmm6260_ehci_power(struct modem_ctl *mc, int state)
{
	s5p_ehci_power(state);
	return 0;
}
static int __init modem_debug_setup(char *args)
{
	int error;
	unsigned long val;

	error = strict_strtoul(args, 0, &val);
	if (!error)
		modem_debug = val;

	return error;
}
__setup("modem_debug=", modem_debug_setup);

static ssize_t show_modem_debug(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;

	p += sprintf(buf, "modem_debug=%d\n", modem_debug);

	return p - buf;
}
static ssize_t store_modem_debug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int error;
	unsigned long val;

	error = strict_strtoul(buf, 10, &val);
	if (!error)
		modem_debug = val;

	return count;
}
static struct device_attribute attr_modem_debug = __ATTR(modem_debug,
		S_IRUGO | S_IWUSR, show_modem_debug, store_modem_debug);

#ifdef CONFIG_MACH_M040
static ssize_t show_modem_usb_power(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;

	if(global_mc->modem_usb_regulator)
		p += scnprintf(p, PAGE_SIZE, "%d\n", regulator_is_enabled(global_mc->modem_usb_regulator));
	else
		p += scnprintf(p, PAGE_SIZE, "No support\n");

	return p - buf;
}
static ssize_t store_modem_usb_power(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int error;
	unsigned long val;

	error = strict_strtoul(buf, 10, &val);
	if (!error){
		if (global_mc->modem_usb_regulator) {
			if(val && !regulator_is_enabled(global_mc->modem_usb_regulator))
				regulator_enable(global_mc->modem_usb_regulator);
			else if(regulator_is_enabled(global_mc->modem_usb_regulator))
				regulator_disable(global_mc->modem_usb_regulator);
		}
	}

	return count;
}
static struct device_attribute attr_modem_usb_power = __ATTR(modem_usb_power,
		S_IRUGO | S_IWUSR, show_modem_usb_power, store_modem_usb_power);
#endif
static void modem_wake_lock_initial(struct modem_ctl *mc)
{
	wake_lock_init(&mc->modem_wakelock, WAKE_LOCK_SUSPEND, "modemctl");
}

static void modem_wake_lock_destroy(struct modem_ctl *mc)
{
	wake_lock_destroy(&mc->modem_wakelock);
}

static void modem_wake_lock(struct modem_ctl *mc)
{
	wake_lock(&mc->modem_wakelock);
}

static void modem_wake_lock_timeout(struct modem_ctl *mc,  int timeout)
{
	wake_lock_timeout(&mc->modem_wakelock, timeout);
}

void modem_notify_event(int type)
{
	if (!global_mc)
		return;

	switch (type) {
		case MODEM_EVENT_POWEROFF:
			global_mc->cp_flag &= MODEM_SIM_DETECT_FLAG;
			wake_up_interruptible(&global_mc->read_wq);
			break;

		case MODEM_EVENT_RESET:
			global_mc->cp_flag |= MODEM_RESET_FLAG;
			wake_up_interruptible(&global_mc->read_wq);
			break;

		case MODEM_EVENT_CRASH:
			if(global_mc->enum_done) {
				global_mc->cp_flag |= MODEM_CRASH_FLAG;
				wake_up_interruptible(&global_mc->read_wq);
			}
			break;

		case MODEM_EVENT_CONN:
			if(global_mc->enum_done) {
				global_mc->cp_flag &= ~(MODEM_DISCONNECT_FLAG);
				global_mc->cp_flag |= MODEM_CONNECT_FLAG;
				wake_up_interruptible(&global_mc->conn_wq);
			}
			break;

		case MODEM_EVENT_DISCONN:
			if(global_mc->enum_done) {
				global_mc->cp_flag &= ~(MODEM_CONNECT_FLAG);
				global_mc->cp_flag |= MODEM_DISCONNECT_FLAG;
				wake_up_interruptible(&global_mc->read_wq);
			}
			break;

		case MODEM_EVENT_SIM:
			global_mc->cp_flag |= MODEM_SIM_DETECT_FLAG;
			global_mc->cp_flag |= MODEM_CRASH_FLAG;
			wake_up_interruptible(&global_mc->read_wq);
			break;

		case MODEM_EVENT_BOOT_INIT:
			global_mc->cp_flag =
				MODEM_RESET_FLAG | MODEM_INIT_ON_FLAG;
			wake_up_interruptible(&global_mc->read_wq);
			break;
	}

	pr_info("%s:%s, cp_flag:0x%x\n", __func__, modem_event_str[type],
							global_mc->cp_flag);
}

int modem_is_on(void)
{
        struct modem_ctl *mc = global_mc;

        if (!mc)
		return 0;
	else
		return gpio_get_value(mc->gpio_cp_reset);
}
EXPORT_SYMBOL_GPL(modem_is_on);

int modem_is_host_wakeup(void)
{
	struct modem_ctl *mc = global_mc;

	if (!mc)
		return 0;
	else
		return (gpio_get_value(mc->gpio_hostwake)
					== HOSTWAKE_TRIGLEVEL) ? 1 : 0;
}
EXPORT_SYMBOL_GPL(modem_is_host_wakeup);

int modem_set_slave_wakeup(int val)
{
	struct modem_ctl *mc = global_mc;

	if (!mc)
		return 0;

	if (gpio_get_value(mc->gpio_hostwake)) {
		pr_info("[MODEM_IF] Slave Wake\n");
		if (gpio_get_value(mc->gpio_slavewake)) {
			gpio_direction_output(mc->gpio_slavewake, 0);
			mdelay(10);
		}
		gpio_direction_output(mc->gpio_slavewake, 1);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(modem_set_slave_wakeup);

static int xmm6260_on(struct modem_ctl *mc)
{
	MIF_INFO("xmm6260_on()\n");

	if (!mc->gpio_cp_reset || !mc->gpio_cp_on || !mc->gpio_reset_req_n) {
		MIF_ERR("no gpio data\n");
		return -ENXIO;
	}

	gpio_set_value(mc->gpio_cp_on, 0);
	gpio_set_value(mc->gpio_reset_req_n, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);
	gpio_set_value(mc->gpio_host_active, 0);
	/* must be >500ms for CP can boot up under -20 degrees */
	msleep(500);
	gpio_set_value(mc->gpio_cp_reset, 1);
	mdelay(1);
	gpio_set_value(mc->gpio_reset_req_n, 1);
	mdelay(2);
	gpio_set_value(mc->gpio_cp_on, 1);
	udelay(80);
	gpio_set_value(mc->gpio_cp_on, 0);
	if (mc->gpio_revers_bias_restore)
		mc->gpio_revers_bias_restore();

	return 0;
}

static int xmm6260_off(struct modem_ctl *mc)
{
	MIF_INFO("xmm6260_off()\n");

	mc->cp_flag &= MODEM_SIM_DETECT_FLAG;
	gpio_set_value(mc->gpio_cp_on, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);
	gpio_set_value(mc->gpio_reset_req_n, 0);
	modem_wake_lock_timeout(mc, 10 * HZ);

	return 0;
}

static int xmm6260_main_enum(struct modem_ctl *mc)
{
	struct completion done;

	wake_up_interruptible(&mc->read_wq);
	modem_wake_lock(mc);
	mc->enum_done = 0;
	xmm6260_off(mc);
	xmm6260_ehci_power(mc, 0);
	modem_set_active_state(0);
	modem_set_slave_wakeup(0);
	msleep(100);
	mc->cp_flag &= MODEM_SIM_DETECT_FLAG;
	xmm6260_on(mc);
	init_completion(&done);
	mc->l2_done = &done;
	wait_for_completion_timeout(&done, 20*HZ);
	mc->l2_done = NULL;
	modem_set_slave_wakeup(1);
	xmm6260_ehci_power(mc, 1);
	modem_set_active_state(1);
	mc->enum_done = 1;
	modem_wake_lock_timeout(mc, 5*HZ);

	return 0;
}

static int xmm6260_renum(struct modem_ctl *mc)
{
	wake_up_interruptible(&mc->read_wq);
	modem_wake_lock(mc);
	mc->enum_done = 0;
	mc->cp_flag &= MODEM_SIM_DETECT_FLAG;
	modem_set_active_state(0);
	s5p_ehci_port_power(0);
	modem_set_slave_wakeup(0);
	msleep(100);
	modem_set_slave_wakeup(1);
	s5p_ehci_port_power(1);
	modem_set_active_state(1);
	mc->enum_done = 1;
	modem_wake_lock_timeout(mc, 5*HZ);

	return 0;
}


static int xmm6260_reset(struct modem_ctl *mc)
{
	MIF_INFO("xmm6260_reset()\n");
	wake_up_interruptible(&mc->read_wq);
	modem_set_active_state(0);
	s5p_ehci_port_power(0);
	xmm6260_on(mc);
	modem_wake_lock_timeout(mc, 5*HZ);
	return 0;
}

static int xmm6260_flash_enum(struct modem_ctl *mc)
{
	wake_up_interruptible(&mc->read_wq);
	modem_wake_lock(mc);
	mc->cp_flag &= MODEM_SIM_DETECT_FLAG;
	mc->enum_done = 0;
	modem_set_active_state(0);
	s5p_ehci_port_power(0);
	modem_set_slave_wakeup(0);
	msleep(100);
	modem_set_slave_wakeup(1);
	s5p_ehci_port_power(1);
	modem_set_active_state(1);
	xmm6260_on(mc);
	mc->enum_done = 1;
	modem_wake_lock_timeout(mc, 5*HZ);

	return 0;
}

static irqreturn_t sim_detect_irq_handler(int irq, void *_mc)
{
	struct modem_ctl *mc = (struct modem_ctl *)_mc;

	mc->sim_state = !gpio_get_value(mc->gpio_sim_detect);
	modem_wake_lock_timeout(mc,  HZ*30);
	MIF_ERR("SIM %s\n", mc->sim_state ? "insert" : "removed");
	modem_notify_event(MODEM_EVENT_SIM);
	mc->ops.modem_off(mc);
	mdelay(200);

	return IRQ_HANDLED;
}

static irqreturn_t modem_cpreset_irq(int irq, void *dev_id)
{
	struct modem_ctl *mc = (struct modem_ctl *)dev_id;
	int val;

	if(global_mc->enum_done) {
		val = gpio_get_value(mc->gpio_cp_reset);
		modem_wake_lock_timeout(mc, HZ * 30);
		modem_notify_event(MODEM_EVENT_RESET);
		pr_info("%s CP_RESET_INT:%d\n",  __func__, val);
	}

	return IRQ_HANDLED;
}

static void xmm6260_get_ops(struct modem_ctl *mc)
{
	mc->ops.modem_on         = xmm6260_on;
	mc->ops.modem_off        = xmm6260_off;
	mc->ops.modem_reset      = xmm6260_reset;
	mc->ops.modem_renum      = xmm6260_renum;
	mc->ops.modem_flash_enum = xmm6260_flash_enum;
}

int modem_open(struct inode *inode, struct file *file)
{
	return 0;
}

int modem_close (struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t
modem_write(struct file *filp, const char __user *buffer, size_t count,
							loff_t *offset)
{
	if(!global_mc)
		return -1;

	pr_info("%s:%s\n", __func__, buffer);

	if(count >= 4 && !strncmp(buffer, "renum", 4)) {
		if (down_interruptible(&modem_downlock) == 0) {
			xmm6260_renum(global_mc);
			up(&modem_downlock);
		}
	}

	if(count >= 4 && !strncmp(buffer, "main", 4)) {
		if (down_interruptible(&modem_downlock) == 0) {
			xmm6260_main_enum(global_mc);
			up(&modem_downlock);
		}
	}

	if(count >= 5 && !strncmp(buffer, "flash", 5))
	{
		if (down_interruptible(&modem_downlock)==0)
		{
			xmm6260_flash_enum(global_mc);
			up(&modem_downlock);
		}
	}

	if(count >= 3 && !strncmp(buffer, "off", 3))
	{
		if (down_interruptible(&modem_downlock) == 0) {
			xmm6260_off(global_mc);
			modem_notify_event(MODEM_EVENT_POWEROFF);
			up(&modem_downlock);
		}
	}

	if(count >= 2 && !strncmp(buffer, "on", 2)) {
		if (down_interruptible(&modem_downlock) == 0) {
			xmm6260_on(global_mc);
			up(&modem_downlock);
		}
	}

	if(count >= 5 && !strncmp(buffer, "reset", 5)) {
		if (down_interruptible(&modem_downlock) == 0) {
			xmm6260_reset(global_mc);
			up(&modem_downlock);
		}
	}

	if(count >= 7 && !strncmp(buffer, "debug=", 6)) {
		int error;
		unsigned long val;

		error = strict_strtoul(buffer + 6, 10, &val);
		if (!error)
			modem_debug = val;
	}

	return count;
}

static ssize_t
modem_read(struct file *filp, char __user * buffer, size_t count,
							loff_t * offset)
{
	int flag = 0;

	if(!global_mc)
		return -EFAULT;

	wait_event_interruptible(global_mc->read_wq,
			(global_mc->cp_flag & MODEM_EVENT_MASK));

	flag = global_mc->cp_flag & MODEM_EVENT_MASK;
	if(copy_to_user(buffer, &flag, sizeof(flag)))
		return -EFAULT;
	pr_info("%s: modem event = 0x%x\n", __func__, flag);

	global_mc->cp_flag &= MODEM_CONNECT_FLAG;

	return 1;
}

long modem_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	if(!global_mc)
		return -ENODEV;

	pr_info("%s cmd:0x%x, arg:0x%lx\n", __func__, cmd, arg);

	return 0;
}

unsigned int modem_poll (struct file *filp, struct poll_table_struct *wait)
{
	u32 mask = 0;

	if(!global_mc)
		return -1;

	if (global_mc->enum_done && (global_mc->cp_flag & MODEM_CONNECT_FLAG))
		mask = POLLIN | POLLOUT | POLLRDNORM;
	else
		poll_wait(filp, &global_mc->conn_wq, wait);

	return mask;
}

static struct file_operations modem_file_ops = {
	.owner          = THIS_MODULE,
	.poll           = modem_poll,
	.open           = modem_open,
	.read           = modem_read,
	.write          = modem_write,
	.release        = modem_close,
	.unlocked_ioctl = modem_ioctl,
};

static struct miscdevice modem_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "modemctl",
	.fops  = &modem_file_ops
};

int xmm6260_init_modemctl_device(struct modem_ctl *mc,
			struct modem_data *pdata)
{
	int ret = 0;
	struct platform_device *pdev;

	global_mc   = mc;
	mc->l2_done = NULL;

	mc->gpio_cp_on        = pdata->gpio_cp_on;
	mc->gpio_cp_reset     = pdata->gpio_cp_reset;
	mc->gpio_hostwake     = pdata->gpio_hostwake;
	mc->gpio_slavewake    = pdata->gpio_slavewake;
	mc->gpio_sim_detect   = pdata->gpio_sim_detect;
	mc->gpio_cp_dump_int  = pdata->gpio_cp_dump_int;
	mc->gpio_host_active  = pdata->gpio_host_active;
	mc->gpio_reset_req_n  = pdata->gpio_reset_req_n;
	mc->gpio_cp_reset_int = pdata->gpio_cp_reset_int;

	mc->gpio_revers_bias_clear   = pdata->gpio_revers_bias_clear;
	mc->gpio_revers_bias_restore = pdata->gpio_revers_bias_restore;
#ifdef CONFIG_MACH_M040
	mc->modem_usb_regulator = regulator_get(NULL, "safeout2");
#endif
	init_waitqueue_head(&mc->read_wq);
	init_waitqueue_head(&mc->conn_wq);
	modem_wake_lock_initial(mc);

	pdev = to_platform_device(mc->dev);

	if (mc->gpio_sim_detect)
		mc->irq_sim_detect = gpio_to_irq(mc->gpio_sim_detect);

	xmm6260_get_ops(mc);

	/* initialize sim_state if gpio_sim_detect exists */
	mc->sim_state = false;
	if (mc->gpio_sim_detect) {
		ret = request_threaded_irq(mc->irq_sim_detect, NULL,
				sim_detect_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"sim_detect", mc);
		if (ret) {
			MIF_ERR("failed to request_irq: %d\n", ret);
			goto err_sim_detect_request_irq;
		}
		ret = enable_irq_wake(mc->irq_sim_detect);
		if (ret) {
			MIF_ERR("failed to enable_irq_wake: %d\n", ret);
			goto err_sim_detect_set_wake_irq;
		}
		/* initialize sim_state => insert: gpio=0, remove: gpio=1 */
		mc->sim_state = !gpio_get_value(mc->gpio_sim_detect);
	}

	if (mc->gpio_cp_reset_int) {
		mc->irq_modem_reset = gpio_to_irq(mc->gpio_cp_reset_int);
		ret = request_threaded_irq(mc->irq_modem_reset, NULL,
			modem_cpreset_irq, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"CP_RESET_INT", mc);
		if (ret) {
			pr_err("Failed register gpio_cp_reset_int irq(%d)!\n",
					mc->irq_modem_reset);
			goto err_cp_reset_irq;
		}
		ret = enable_irq_wake(mc->irq_modem_reset);
		if (ret) {
			MIF_ERR("failed to enable_irq_wake of modem reset:%d\n",
					ret);
			goto err_reset_irq_enable_wake;
		}

	}

	ret = misc_register(&modem_miscdev);
	if(ret) {
		pr_err("Failed to register modem control device\n");
		goto err_misc_register;
	}

	ret = device_create_file(modem_miscdev.this_device, &attr_modem_debug);
	if (ret) {
		pr_err("failed to create sysfs file:attr_modem_debug!\n");
		goto err_device_create_file;
	}
#ifdef CONFIG_MACH_M040
	ret = device_create_file(modem_miscdev.this_device, &attr_modem_usb_power);
	if (ret) {
		pr_err("failed to create sysfs file:attr_modem_usb_power!\n");
		goto err_device_create_file;
	}
#endif
	return ret;

err_device_create_file:
	misc_deregister(&modem_miscdev);
err_misc_register:
err_reset_irq_enable_wake:
	free_irq(mc->irq_modem_reset, mc);
err_cp_reset_irq:
err_sim_detect_set_wake_irq:
	free_irq(mc->irq_sim_detect, mc);
err_sim_detect_request_irq:
	modem_wake_lock_destroy(mc);
	mc->sim_state = false;

	return ret;
}

