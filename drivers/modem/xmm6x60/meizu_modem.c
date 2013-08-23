/**
 * linux/drivers/modem/meizu_modem.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_variation.h"
#include "modem_utils.h"

int modem_debug = 0;
static struct modem_ctl *create_modemctl_device(struct platform_device *pdev)
{
	int ret = 0;
	struct modem_data *pdata;
	struct modem_ctl *modemctl;
	struct device *dev = &pdev->dev;

	modemctl = kzalloc(sizeof(struct modem_ctl), GFP_KERNEL);
	if (!modemctl)
		return NULL;

	modemctl->dev = dev;
	pdata = pdev->dev.platform_data;
	modemctl->mdm_data = pdata;
	modemctl->name = pdata->name;

	INIT_LIST_HEAD(&modemctl->commons.link_dev_list);

	/* initialize tree of io devices */
	modemctl->commons.iodevs_tree_chan = RB_ROOT;

	/* init modemctl device for getting modemctl operations */
	ret = call_modem_init_func(modemctl, pdata);
	if (ret) {
		kfree(modemctl);
		return NULL;
	}
	modem_tty_driver_init(modemctl);
	MIF_INFO("%s is created!!!\n", pdata->name);

	return modemctl;
}
static struct io_device *create_io_device(struct modem_io_t *io_t,
		struct modem_ctl *modemctl, struct modem_data *pdata)
{
	int ret = 0;
	struct io_device *iod = NULL;

	iod = kzalloc(sizeof(struct io_device), GFP_KERNEL);
	if (!iod) {
		MIF_ERR("iod == NULL\n");
		return NULL;
	}

	rb_init_node(&iod->node_chan);

	iod->name = io_t->name;
	iod->id = io_t->id;
	iod->format = io_t->format;
	iod->io_typ = io_t->io_type;
	iod->link_types = io_t->links;
	iod->phone_net_type = pdata->modem_net;
	atomic_set(&iod->opened, 0);
	wake_lock_init(&iod->wakelock, WAKE_LOCK_SUSPEND, iod->name);

	iod->mc = modemctl;

	/* register misc device or net device */
	ret = meizu_ipc_init_io_device(iod);
	if (ret) {
		kfree(iod);
		MIF_ERR("meizu_ipc_init_io_device fail (%d)\n", ret);
		return NULL;
	}

	MIF_DEBUG("%s is created!!!\n", iod->name);
	return iod;
}

static int attach_devices(struct modem_ctl *mc, struct io_device *iod,
		enum modem_link tx_link)
{
	struct link_device *ld;

	insert_iod_with_channel(&mc->commons, iod->id, iod);

	/* find link type for this io device */
	list_for_each_entry(ld, &mc->commons.link_dev_list, list) {
		if (IS_CONNECTED(iod, ld)) {
			/* The count 1 bits of iod->link_types is count
			 * of link devices of this iod.
			 * If use one link device,
			 * or, 2+ link devices and this link is tx_link,
			 * set iod's link device with ld
			 */
			if ((countbits(iod->link_types) <= 1) ||
					(tx_link == ld->link_type)) {
				MIF_DEBUG("set %s->%s\n", iod->name, ld->name);
				set_current_link(iod, ld);
			}
		}
	}

	/* if use rx dynamic switch, set tx_link at modem_io_t of
	 * board-*-modems.c
	 */
	if (!get_current_link(iod)) {
		MIF_ERR("%s->link == NULL\n", iod->name);
		BUG();
	}

	return 0;
}
static int __devinit modem_probe(struct platform_device *pdev)
{
	int i;
	struct modem_data *pdata = pdev->dev.platform_data;
	struct modem_ctl *modemctl;
	struct io_device *iod[pdata->num_iodevs];
	struct link_device *ld;

	memset(iod, 0, sizeof(iod));

	modemctl = create_modemctl_device(pdev);
	if (!modemctl) {
		MIF_ERR("modemctl == NULL\n");
		goto err_free_modemctl;
	}

	platform_set_drvdata(pdev, modemctl);
	modemctl->rx_wq = create_singlethread_workqueue("modem_rx_wq");
	if (!modemctl->rx_wq) {
		MIF_ERR("fail to create wq\n");
		return -EINVAL;
	}
	for (i = 0; i < LINKDEV_MAX ; i++) {
		if (pdata->link_types & LINKTYPE(i)) {
			ld = call_link_init_func(pdev, i);
			if (!ld)
				goto err_free_modemctl;

			MIF_ERR("link created: %s\n", ld->name);
			ld->link_type = i;
			ld->mc = modemctl;
			list_add(&ld->list, &modemctl->commons.link_dev_list);
		}
	}
	/* create io deivces and connect to modemctl device */
	for (i = 0; i < pdata->num_iodevs; i++) {
		iod[i] = create_io_device(&pdata->iodevs[i], modemctl, pdata);
		if (!iod[i]) {
			MIF_ERR("iod[%d] == NULL\n", i);
			goto err_free_modemctl;
		}

		attach_devices(modemctl, iod[i],
				pdata->iodevs[i].tx_link);
	}
	MIF_INFO("Complete!!!\n");

	return 0;

err_free_modemctl:
	for (i = 0; i < pdata->num_iodevs; i++)
		if (iod[i] != NULL)
			kfree(iod[i]);

	if (modemctl != NULL)
		kfree(modemctl);

	return -ENOMEM;
}

static void modem_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct modem_ctl *mc = dev_get_drvdata(dev);

	free_irq(mc->irq_modem_reset, mc);
	mc->cp_flag &= MODEM_SIM_DETECT_FLAG;
	mc->ops.modem_off(mc);
}

static int modem_suspend(struct device *dev)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);

	disable_irq(mc->irq_hostwake);
	irq_set_irq_type(mc->irq_hostwake, IRQF_NO_SUSPEND |\
			IRQF_TRIGGER_LOW | IRQF_ONESHOT);

	return 0;
}

static int modem_resume(struct device *dev)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);

	irq_set_irq_type(mc->irq_hostwake, IRQF_NO_SUSPEND |\
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
	enable_irq(mc->irq_hostwake);

	return 0;
}

static const struct dev_pm_ops modem_pm_ops = {
	.suspend = modem_suspend,
	.resume  = modem_resume,
};

static struct platform_driver modem_driver = {
	.probe    = modem_probe,
	.shutdown = modem_shutdown,
	.driver   = {
		.name = "modem_ifx_6260",
		.pm   = &modem_pm_ops,
	},
};

static int __init modem_driver_init(void)
{
	return platform_driver_register(&modem_driver);
}

module_init(modem_driver_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("KarlZheng<zhengkl@meizu.com>");
MODULE_DESCRIPTION("Meizu Modem Interface Driver");
