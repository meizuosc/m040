/* drivers/modem/modem_link_device_hsic.c
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

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_data/modem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/usb/cdc.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/wakelock.h>
#include <mach/modem.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "meizu_modem_hsic.h"

#define ACM_CTRL_DTR	0x01
#define ACM_CTRL_RTS	0x02

extern void pm_runtime_init(struct device *dev);
#define HSIC_MAX_PIPE_ORDER_NR 3

static struct modem_ctl *get_modemctl(struct link_pm_data *pm_data);
static int hsic_pm_runtime_get_active(struct link_pm_data *pm_data);
static int hsic_tx_urb_with_skb(struct usb_device *usbdev, struct sk_buff *skb,
					struct if_usb_devdata *pipe_data);
static void hsic_rx_complete(struct urb *urb);

static struct usb_device *global_usbdev = NULL;

static int hsic_set_slave_wakeup_gpio(unsigned int gpio, int val)
{
	int v = gpio_get_value(gpio);
	gpio_set_value(gpio, val);
	MIF_DEBUG("called(%pF) [SWK][%d]=>[%d]:[%d]\n",  \
			 __builtin_return_address(0), v, \
			 val, gpio_get_value(gpio));

	return 0;
}

static int pm_data_get_device(struct link_pm_data *pm_data, int new_state)
{
	DECLARE_WAITQUEUE(wait, current);

	while (1) {
		spin_lock(&pm_data->pm_data_lock);
		if (pm_data->state == ACM_READY) {
			pm_data->state = new_state;
			spin_unlock(&pm_data->pm_data_lock);
			break;
		}
		if (new_state == ACM_SUSPEND) {
			MIF_ERR("new_state == ACM_SUSPEND\n");
			spin_unlock(&pm_data->pm_data_lock);
			return -EAGAIN;
		}
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&pm_data->waitqueue, &wait);
		spin_unlock(&pm_data->pm_data_lock);
		schedule();
		remove_wait_queue(&pm_data->waitqueue, &wait);
	}

	return 0;
}

static void pm_data_release_device(struct link_pm_data *pm_data)
{
	spin_lock(&pm_data->pm_data_lock);
	pm_data->state = ACM_READY;
	wake_up(&pm_data->waitqueue);
	spin_unlock(&pm_data->pm_data_lock);
}

static void hsic_set_autosuspend_delay(struct usb_device *usbdev, int delay)
{
	pm_runtime_set_autosuspend_delay(&usbdev->dev, delay);
}

static int hsic_start_channel(struct link_device *ld, struct io_device *iod)
{
	int err = 0;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct link_pm_data *pm_data = usb_ld->link_pm_data;
	struct device *dev = &usb_ld->usbdev->dev;

	if (!usb_ld->if_usb_connected) {
		MIF_DEBUG("HSIC not connected, skip start ipc\n");
		err = -ENODEV;
		goto exit;
	}

retry:
	if (!(ld->mc->cp_flag & MODEM_CONNECT_FLAG)) {
		MIF_DEBUG("MODEM is not online, skip start ipc\n");
		err = -ENODEV;
		goto exit;
	}

	/* check usb runtime pm first */
	if (dev->power.runtime_status != RPM_ACTIVE) {
		if (!pm_data->resume_requested) {
			MIF_DEBUG("QW PM\n");
			INIT_COMPLETION(pm_data->active_done);
			queue_delayed_work(pm_data->wq,
					&pm_data->hsic_pm_work, 0);
		}
		MIF_DEBUG("Wait pm\n");
		err = wait_for_completion_timeout(&pm_data->active_done,
						msecs_to_jiffies(1000));
		/* timeout or -ERESTARTSYS */
		if (err <= 0)
			goto retry;
	}
	usb_mark_last_busy(usb_ld->usbdev);
exit:
	return err;
}

static int hsic_init_channel(struct link_device *ld,
			struct io_device *iod)
{
	struct task_struct *task = get_current();
	char str[TASK_COMM_LEN];

	MIF_DEBUG("%d:%s\n", task->pid, get_task_comm(str, task));

	hsic_start_channel(ld, iod);

	return 0;
}

static void hsic_close_channel(struct link_device *ld,
			struct io_device *iod)
{
	/*ld->com_state = COM_NONE;*/
	pr_info("%s,channel id:%d\n", __func__, iod->id);
}

static int hsic_rx_submit(struct usb_link_device *usb_ld,
					struct if_usb_devdata *pipe_data,
					gfp_t gfp_flags)
{
	int ret;
	struct urb *urb;

	if (pipe_data->disconnected)
		return -ENOENT;

	urb = pipe_data->urb;
	if (urb == NULL) {
		printk("%s urb is NULL!!\n", __func__);
		return -EPIPE;
	}
	usb_mark_last_busy(usb_ld->usbdev);

	/*urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;*/
	urb->transfer_flags = 0;
	usb_fill_bulk_urb(urb, pipe_data->usbdev,
				pipe_data->rx_pipe, pipe_data->rx_buf,
				pipe_data->rx_buf_size, hsic_rx_complete,
				(void *)pipe_data);

	if (!usb_ld->if_usb_connected || !usb_ld->usbdev)
		return -ENOENT;

	ret = usb_submit_urb(urb, gfp_flags);
	if (ret)
		MIF_ERR("submit urb fail with ret (%d)\n", ret);
	usb_mark_last_busy(usb_ld->usbdev);

	return ret;
}

static void hsic_rx_retry_work(struct work_struct *work)
{
	int ret = 0;
	struct usb_link_device *usb_ld =
		container_of(work, struct usb_link_device, rx_retry_work.work);
	struct urb *urb = usb_ld->retry_urb;
	struct if_usb_devdata *pipe_data = urb->context;
	struct io_device *iod;

	if (!usb_ld->if_usb_connected || !usb_ld->usbdev)
		return;

	if (usb_ld->usbdev)
		usb_mark_last_busy(usb_ld->usbdev);

	iod = link_get_iod_with_channel(&usb_ld->ld, pipe_data->channel_id);
	if (iod) {
		ret = iod->recv(iod, &usb_ld->ld, (char *)urb->transfer_buffer,
			urb->actual_length);
		if (ret == -ENOMEM) {
			/* TODO: check the retry count */
			/* retry the delay work after 20ms and resubit*/
			MIF_ERR("ENOMEM, +retry 20ms\n");
			if (usb_ld->usbdev)
				usb_mark_last_busy(usb_ld->usbdev);
			usb_ld->retry_urb = urb;
			if (usb_ld->rx_retry_cnt++ < 10)
				queue_delayed_work(usb_ld->ld.tx_wq,
					&usb_ld->rx_retry_work,	10);
			return;
		}
		if (ret < 0)
			MIF_ERR("io device recv error (%d)\n", ret);
		usb_ld->rx_retry_cnt = 0;
	}

	if (usb_ld->usbdev)
		usb_mark_last_busy(usb_ld->usbdev);
	hsic_rx_submit(usb_ld, pipe_data, GFP_ATOMIC);
}

static void hsic_rx_complete(struct urb *urb)
{
	struct if_usb_devdata *pipe_data = urb->context;
	struct usb_link_device *usb_ld = pipe_data->usb_ld;
	struct io_device *iod;
	int ret;

	if (usb_ld->usbdev)
		usb_mark_last_busy(usb_ld->usbdev);

	switch (urb->status) {
	case -ENOENT:
		/* case for 'link pm suspended but rx data had remained' */
	case 0:
		pipe_data->hsic_channel_rx_count ++;
		if (!urb->actual_length)
			goto rx_submit;
		usb_ld->link_pm_data->rx_cnt++;

		iod = link_get_iod_with_channel(&usb_ld->ld,
						pipe_data->channel_id);
		if (iod) {
			if (iod->atdebug)
				iod->atdebugfunc(iod, urb->transfer_buffer,
							urb->actual_length);

			ret = iod->recv(iod, &usb_ld->ld, urb->transfer_buffer,
					urb->actual_length);
			if (ret == -ENOMEM) {
				/* retry the delay work and resubit*/
				MIF_ERR("ENOMEM, retry\n");
				if (usb_ld->usbdev)
					usb_mark_last_busy(usb_ld->usbdev);
				usb_ld->retry_urb = urb;
				queue_delayed_work(usb_ld->ld.tx_wq,
					&usb_ld->rx_retry_work, 0);
				return;
			} else {
				if (ret < 0)
					pr_err("io device recv err:%d\n", ret);
				else
					pipe_data->hsic_channel_rx_count --;
			}
		}
rx_submit:
		if (urb->status == 0) {
			if (usb_ld->usbdev)
				usb_mark_last_busy(usb_ld->usbdev);
			hsic_rx_submit(usb_ld, pipe_data, GFP_ATOMIC);
		}
		break;
	default:
		MIF_ERR("urb err status = %d\n", urb->status);
		break;
	}
}

static int hsic_send(struct link_device *ld, struct io_device *iod,
			struct sk_buff *skb)
{
	struct sk_buff_head *txq;
	size_t tx_size;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct link_pm_data *pm_data = usb_ld->link_pm_data;

	if (usb_ld->ld.com_state != COM_ONLINE) {
		//modem_notify_event(MODEM_EVENT_DISCONN);
		return -ENODEV;
	}

	if (iod->send_delay && (iod->io_typ == IODEV_NET) \
			&& (1400 == skb->len))
		udelay(iod->send_delay);

	txq = &ld->sk_raw_tx_q;
	tx_size = skb->len;
	skb_queue_tail(txq, skb);
	usb_ld->devdata[iod->id].hsic_channel_tx_count ++;

	/* Hold wake_lock for getting schedule the tx_work */
	wake_lock(&pm_data->tx_async_wake);

	if (!work_pending(&ld->tx_delayed_work.work))
		queue_delayed_work(ld->tx_wq, &ld->tx_delayed_work, 0);

	return tx_size;
}

static void hsic_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct io_device *iod = skbpriv(skb)->iod;
	struct link_device *linkdev = get_current_link(iod);
	struct usb_link_device *usb_ld = to_usb_link_device(linkdev);

	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
		MIF_DEBUG("iod %d TX error (%d)\n", iod->id, urb->status);
		break;
	default:
		MIF_INFO("iod %d TX error (%d)\n", iod->id, urb->status);
	}

	if (iod->atdebug)
		iod->atdebugfunc(iod, skb->data, skb->len);

	usb_ld->devdata[iod->id].hsic_channel_tx_count --;

	if (urb->dev)
		usb_mark_last_busy(urb->dev);
	usb_free_urb(urb);

	dev_kfree_skb_any(skb);
}

/* Even if hsic_tx_urb_with_skb is failed, does not release the skb to retry */
static int hsic_tx_urb_with_skb(struct usb_device *usbdev, struct sk_buff *skb,
					struct if_usb_devdata *pipe_data)
{
	int ret;
	struct urb *urb;

	if (pipe_data->disconnected)
		return -ENOENT;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		MIF_ERR("alloc urb error\n");
		return -ENOMEM;
	}
	urb->transfer_flags = URB_ZERO_PACKET;
	pm_data_get_device(pipe_data->usb_ld->link_pm_data, ACM_WRITE);
	usb_fill_bulk_urb(urb, pipe_data->usbdev, pipe_data->tx_pipe, skb->data,
			skb->len, hsic_tx_complete, (void *)skb);
	usb_mark_last_busy(usbdev);
	ret = usb_submit_urb(urb, GFP_KERNEL);
	pm_data_release_device(pipe_data->usb_ld->link_pm_data);
	if (ret < 0) {
		MIF_ERR("usb_submit_urb with ret(%d)\n", ret);
		usb_free_urb(urb);
		return ret;
	}

	return 0;
}


static int _hsic_tx_work(struct sk_buff *skb)
{
	struct sk_buff_head *txq;
	struct io_device *iod = skbpriv(skb)->iod;
	struct link_device *ld = skbpriv(skb)->ld;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct if_usb_devdata *pipe_data;

	pipe_data = &usb_ld->devdata[iod->id];
	txq = &ld->sk_raw_tx_q;

	if (!pipe_data)
		return -ENOENT;

	return hsic_tx_urb_with_skb(usb_ld->usbdev, skb,	pipe_data);
}


static void hsic_tx_work(struct work_struct *work)
{
	int ret = 0;
	struct link_device *ld =
		container_of(work, struct link_device, tx_delayed_work.work);
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct sk_buff *skb;
	struct link_pm_data *pm_data = usb_ld->link_pm_data;

	if (!usb_ld->usbdev) {
		MIF_DEBUG("usbdev is invalid\n");
		return;
	}

	pm_data->tx_cnt++;

	while (ld->sk_raw_tx_q.qlen) {
		wake_lock(&pm_data->tx_async_wake);
		ret = hsic_pm_runtime_get_active(pm_data);
		if (ret < 0) {
			MIF_ERR("link not avail. ret:%d\n", ret);
			if (ret == -ENODEV)
				goto exit;
			else
				goto retry_tx_work;
		}

		/* If AP try to tx when interface disconnect->reconnect probe,
		 * usbdev was created but one of interface channel device are
		 * probing, _hsic_tx_work return to -ENOENT then runtime usage
		 * count allways positive and never enter to L2
		 */
		if (!usb_ld->if_usb_connected) {
			MIF_DEBUG("link is available, but it was not readey!\n");
			goto retry_tx_work;
		}
		pm_runtime_get_sync(&usb_ld->usbdev->dev);

		ret = 0;
		/* send skb from raw_txq*/

		skb = skb_dequeue(&ld->sk_raw_tx_q);
		if (skb)
			ret = _hsic_tx_work(skb);

		if (ret) {
			MIF_ERR("hsic_tx_urb_with_skb for raw_q %d\n", ret);
			skb_queue_head(&ld->sk_raw_tx_q, skb);

			if (ret == -ENODEV || ret == -ENOENT)
				goto exit;

			pm_runtime_put(&usb_ld->usbdev->dev);
			goto retry_tx_work;
		}

		pm_runtime_put(&usb_ld->usbdev->dev);
	}
	wake_unlock(&pm_data->tx_async_wake);
exit:
	return;

retry_tx_work:
	queue_delayed_work(ld->tx_wq, &ld->tx_delayed_work,
		msecs_to_jiffies(20));
	return;
}

/*
#ifdef CONFIG_LINK_PM
*/
static int hsic_pm_runtime_get_active(struct link_pm_data *pm_data)
{
	int ret;
	struct usb_link_device *usb_ld = pm_data->usb_ld;
	struct device *dev = &usb_ld->usbdev->dev;

	if (!usb_ld->if_usb_connected || usb_ld->ld.com_state == COM_NONE) {
		pr_err("%s if_usb_connected:%d, com_state:%d\n", __func__,
				usb_ld->if_usb_connected, usb_ld->ld.com_state);
		return -ENODEV;
	}

	if (pm_data->dpm_suspending) {
		/* during dpm_suspending...if AP get tx data, wake up. */
		wake_lock(&pm_data->l2_wake);
		MIF_ERR("Kernel in suspending try get_active later\n");
		return -EAGAIN;
	}

	if (dev->power.runtime_status == RPM_ACTIVE) {
		pm_data->resume_retry_cnt = 0;
		return 0;
	}

	if (!pm_data->resume_requested) {
		MIF_DEBUG("QW PM\n");
		queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_work, 0);
	} else
		MIF_ERR("pm_data->resume_requested is true, not QW PM!\n");

	MIF_DEBUG("Wait pm\n");
	INIT_COMPLETION(pm_data->active_done);
	ret = wait_for_completion_timeout(&pm_data->active_done,
						msecs_to_jiffies(2000));
	if (!ret)
		pr_info("%s wait_for_completion_timeout!\n", __func__);

	/* If usb link was disconnected while waiting ACTIVE State, usb device
	 * was removed, usb_ld->usbdev->dev is invalid and below
	 * dev->power.runtime_status is also invalid address.
	 * It will be occured LPA L3 -> AP iniated L0 -> disconnect -> link
	 * timeout
	 */
	if (!usb_ld->if_usb_connected || usb_ld->ld.com_state == COM_NONE) {
		MIF_INFO("link is not connected!\n");
		return -ENODEV;
	}

	if (dev->power.runtime_status != RPM_ACTIVE) {
		MIF_ERR("link_active (%d) retry\n",
				dev->power.runtime_status);
		return -EAGAIN;
	} else
		MIF_DEBUG("link_active success(%d)\n", ret);

	return 0;
}

static void hsic_pm_runtime_start(struct work_struct *work)
{
	struct link_pm_data *pm_data =
		container_of(work, struct link_pm_data, hsic_pm_start.work);
	struct usb_device *usbdev = pm_data->usb_ld->usbdev;
	struct device *dev, *ppdev;
	struct link_device *ld = &pm_data->usb_ld->ld;

	if (!pm_data->usb_ld->if_usb_connected) {
		MIF_DEBUG("disconnect status, ignore\n");
		return;
	}

	dev = &pm_data->usb_ld->usbdev->dev;

	/* wait interface driver resumming */
	if (dev->power.runtime_status == RPM_SUSPENDED) {
		MIF_ERR("suspended yet, delayed work\n");
		queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_start,
			msecs_to_jiffies(10));
		return;
	}

	if (pm_data->usb_ld->usbdev && dev->parent) {
		MIF_DEBUG("rpm_status: %d\n", dev->power.runtime_status);
		/* retry prvious link tx q */
		queue_delayed_work(ld->tx_wq, &ld->tx_delayed_work, 0);
	}
}

static inline int hsic_pm_slave_wake(struct link_pm_data *pm_data)
{
	int ret = MC_HOST_SUCCESS;
	int spin = 100;
	int val;

	val = gpio_get_value(pm_data->gpio_hostwake);
	if (val != HOSTWAKE_TRIGLEVEL) {
		if (gpio_get_value(pm_data->gpio_slavewake)) {
			hsic_set_slave_wakeup_gpio(pm_data->gpio_slavewake, 0);
			MIF_DEBUG("[SWK][1]=>[0]:[%d]\n",
				gpio_get_value(pm_data->gpio_slavewake));
			mdelay(5);
		}
		hsic_set_slave_wakeup_gpio(pm_data->gpio_slavewake, 1);
		mdelay(1);
		while (spin-- && gpio_get_value(pm_data->gpio_hostwake) !=
							HOSTWAKE_TRIGLEVEL)
			mdelay(1);
		if (spin <= 0)
			ret = MC_HOST_TIMEOUT;
	} else {
		MIF_DEBUG("HOST_WUP:HOSTWAKE_TRIGLEVEL!\n");
		ret = MC_HOST_HIGH;
	}

	return ret;
}

static void hsic_pm_runtime_work(struct work_struct *work)
{
	int ret;
	struct link_pm_data *pm_data =
		container_of(work, struct link_pm_data, hsic_pm_work.work);
	struct usb_device *usbdev = pm_data->usb_ld->usbdev;
	struct device *dev = &usbdev->dev;
	int spin1 = 1000;
	int spin2 = 1000;

	if (pm_data->dpm_suspending) {
		wake_lock_timeout(&pm_data->l2_wake, msecs_to_jiffies(5000));
		MIF_ERR("pm_data->dpm_suspending:%d\n", \
				pm_data->dpm_suspending);
		queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_work,
							msecs_to_jiffies(20));
		return;
	}
retry:
	if (!pm_data->usb_ld->if_usb_connected) {
		MIF_ERR("pm_data->usb_ld->if_usb_connected:%d\n", \
				pm_data->usb_ld->if_usb_connected);
		modem_notify_event(MODEM_EVENT_DISCONN);
		return;
	}
	if (pm_data->usb_ld->ld.com_state == COM_NONE) {
		MIF_ERR("pm_data->usb_ld->ld.com_state:%d\n", \
				pm_data->usb_ld->ld.com_state);
		modem_notify_event(MODEM_EVENT_DISCONN);
		return;
	}
	MIF_DEBUG("for dev 0x%p : current %d\n", dev,
				dev->power.runtime_status);

	switch (dev->power.runtime_status) {
	case RPM_ACTIVE:
		pm_data->resume_retry_cnt = 0;
		pm_data->resume_requested = false;
		complete(&pm_data->active_done);

		return;
	case RPM_SUSPENDED:
		if (pm_data->resume_requested)
			break;
		pm_data->resume_requested = true;
		wake_lock(&pm_data->rpm_wake);
		ret = hsic_pm_slave_wake(pm_data);
		if (MC_HOST_TIMEOUT == ret) {
			pm_data->resume_requested = false;
			break;
		}
		if (!pm_data->usb_ld->if_usb_connected) {
			modem_notify_event(MODEM_EVENT_DISCONN);
			wake_unlock(&pm_data->rpm_wake);
			return;
		}
		ret = pm_runtime_resume(dev);
		if (ret < 0) {
			MIF_ERR("resume error(%d)\n", ret);
			if (!pm_data->usb_ld->if_usb_connected) {
				modem_notify_event(MODEM_EVENT_DISCONN);
				wake_unlock(&pm_data->rpm_wake);
				return;
			}
			/* force to go runtime idle before retry resume */
			if (dev->power.timer_expires == 0 &&
						!dev->power.request_pending) {
				MIF_ERR("run time idle\n");
				pm_runtime_idle(dev);
				modem_notify_event(MODEM_EVENT_DISCONN);
			}
		}
		wake_unlock(&pm_data->rpm_wake);
		break;
	case RPM_SUSPENDING:
		MIF_DEBUG("RPM Suspending, spin:%d\n", spin1);
		if (spin1-- == 0) {
			MIF_ERR("Modem suspending timeout\n");
			modem_notify_event(MODEM_EVENT_DISCONN);
			break;
		}
		msleep(1);
		goto retry;
	case RPM_RESUMING:
		MIF_DEBUG("RPM Resuming, spin:%d\n", spin2);
		if (spin2-- == 0) {
			MIF_ERR("Modem resume timeout\n");
			modem_notify_event(MODEM_EVENT_DISCONN);
			break;
		}
		msleep(1);
		goto retry;
	default:
		break;
	}
	pm_data->resume_requested = false;
	/* check until runtime_status goes to active */
	if (dev->power.runtime_status == RPM_ACTIVE) {
		pm_data->resume_retry_cnt = 0;
		complete(&pm_data->active_done);
	} else if (pm_data->resume_retry_cnt++ > 10) {
		MIF_ERR("runtime_status:%d, retry_cnt:%d, notify MODEM_EVENT_DISCONN\n",
			dev->power.runtime_status, pm_data->resume_retry_cnt);
		modem_notify_event(MODEM_EVENT_DISCONN);
	} else {
		MIF_ERR("runtime_status:%d, retry_cnt:%d, redo hsic_pm_work\n",
			dev->power.runtime_status, pm_data->resume_retry_cnt);
		queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_work,
							msecs_to_jiffies(20));
	}
}

static irqreturn_t host_wakeup_irq_handler(int irq, void *data)
{
	int value;
	struct link_pm_data *pm_data = data;
	struct platform_device *pdev_modem = pm_data->pdev_modem;
	struct modem_ctl *mc = platform_get_drvdata(pdev_modem);

	value = gpio_get_value(pm_data->gpio_hostwake);
	MIF_DEBUG("\n[HWK]<=[%d]\n", value);

	wake_lock(&pm_data->l2_wake);

	if (pm_data->dpm_suspending)
		MIF_ERR("%s when suspending\n", __func__);

	if (!mc->enum_done) {
		if (value == HOSTWAKE_TRIGLEVEL) {
			if (mc->l2_done) {
				complete(mc->l2_done);
				mc->l2_done = NULL;
				hsic_set_slave_wakeup_gpio(pm_data->gpio_slavewake, 0);
			}
		}
	} else {
		if (value != HOSTWAKE_TRIGLEVEL) {
			if (mc->l2_done) {
				complete(mc->l2_done);
				mc->l2_done = NULL;
			}
			hsic_set_slave_wakeup_gpio(pm_data->gpio_slavewake, 0);
		} else {
			if (mc->l2_done) {
				complete(mc->l2_done);
				mc->l2_done = NULL;
			}
			queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_work, 0);
		}
	}

	return IRQ_HANDLED;
}

static int hsic_pm_notifier_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	struct link_pm_data *pm_data =
			container_of(this, struct link_pm_data,	pm_notifier);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		if(pm_data_get_device(pm_data, ACM_SUSPEND))
			return NOTIFY_BAD;
		pm_data->dpm_suspending = true;
		MIF_INFO("dpm suspending set to true\n");
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		pm_data_release_device(pm_data);
		pm_data->dpm_suspending = false;
		if (gpio_get_value(pm_data->gpio_hostwake)
			== HOSTWAKE_TRIGLEVEL) {
			wake_lock(&pm_data->l2_wake);
			queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_work,
				0);
			MIF_INFO("post resume\n");
		}
		MIF_INFO("dpm suspending set to false\n");
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static struct modem_ctl *get_modemctl(struct link_pm_data *pm_data)
{
	struct platform_device *pdev_modem = pm_data->pdev_modem;
	struct modem_ctl *mc = platform_get_drvdata(pdev_modem);

	return mc;
}
static int modem_hsic_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct if_usb_devdata *devdata = usb_get_intfdata(intf);
	struct link_pm_data *pm_data = devdata->usb_ld->link_pm_data;

	if (!devdata->disconnected && devdata->state == STATE_RESUMED) {
		usb_kill_urb(devdata->urb);
		devdata->state = STATE_SUSPENDED;
	}

	devdata->usb_ld->suspended++;

	if (devdata->usb_ld->suspended == 1)
		MIF_INFO("%s begin suspend!\n", __func__);

	if (devdata->usb_ld->suspended == IF_USB_DEVNUM_MAX*2) {
		MIF_INFO("[modem_hsic_suspended]\n");
		wake_lock_timeout(&pm_data->l2_wake, msecs_to_jiffies(500));
		if (!pm_data->rx_cnt && !pm_data->tx_cnt) {
			if (pm_data->ipc_debug_cnt++ > 10) {
				MIF_ERR("No TX/RX after resume 10times\n");
				modem_notify_event(MODEM_EVENT_DISCONN);
			}
		} else {
			pm_data->ipc_debug_cnt = 0;
			pm_data->rx_cnt = 0;
			pm_data->tx_cnt = 0;
		}
	}

	return 0;
}

static int modem_hsic_resume(struct usb_interface *intf)
{
	int ret;
	struct if_usb_devdata *devdata = usb_get_intfdata(intf);
	struct link_pm_data *pm_data = devdata->usb_ld->link_pm_data;

	if (!devdata->disconnected && devdata->state == STATE_SUSPENDED) {
		ret = hsic_rx_submit(devdata->usb_ld, devdata, GFP_ATOMIC);
		if (ret < 0) {
			MIF_ERR("hsic_rx_submit error with (%d)\n", ret);
			return ret;
		}
		devdata->state = STATE_RESUMED;
	}

	/* For debugging -  nomal case, never reach below... */
	if (pm_data->resume_retry_cnt > 5) {
		MIF_ERR("retry_cnt=%d, rpm_status=%d",
			pm_data->resume_retry_cnt,
			devdata->usb_ld->usbdev->dev.power.runtime_status);
		pm_data->resume_retry_cnt = 0;
	}

	if (devdata->usb_ld->suspended == IF_USB_DEVNUM_MAX*2)
		MIF_INFO("%s begin resume!\n", __func__);

	devdata->usb_ld->suspended--;
	wake_lock(&pm_data->l2_wake);
	queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_start, 0);

	if (!devdata->usb_ld->suspended)
		MIF_INFO("[modem_hsic_resumed]\n");

	return 0;
}

static int modem_hsic_reset_resume(struct usb_interface *intf)
{
	int ret;
	struct if_usb_devdata *devdata = usb_get_intfdata(intf);
	struct link_pm_data *pm_data = devdata->usb_ld->link_pm_data;

	ret = modem_hsic_resume(intf);
	pm_data->ipc_debug_cnt = 0;
	/*
	 * for runtime suspend, kick runtime pm at L3 -> L0 reset resume
	*/
	if (!devdata->usb_ld->suspended)
		queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_start, 0);

	return ret;
}

static void modem_hsic_disconnect(struct usb_interface *intf)
{
	struct if_usb_devdata *devdata = usb_get_intfdata(intf);
	struct link_pm_data *pm_data = devdata->usb_ld->link_pm_data;
	struct link_device *ld = &devdata->usb_ld->ld;

	if (devdata->disconnected)
		return;

	devdata->state = STATE_SUSPENDED;
	devdata->usb_ld->ld.com_state = COM_NONE;
	devdata->disconnected = 1;
	devdata->usb_ld->if_usb_connected = 0;
	devdata->usb_ld->suspended = 0;
	
	/* cancel runtime start delayed works */
	cancel_delayed_work(&pm_data->hsic_pm_start);
	cancel_delayed_work(&ld->tx_delayed_work);
	
	wake_lock_timeout(&devdata->usb_ld->link_pm_data->l2_wake, \
			msecs_to_jiffies(1000));

	usb_driver_release_interface(to_usb_driver(intf->dev.driver), intf);

	usb_kill_urb(devdata->urb);

	MIF_DEBUG("dev 0x%p\n", devdata->usbdev);
	usb_put_dev(devdata->usbdev);

	devdata->data_intf = NULL;
	devdata->usbdev = NULL;
	pm_data->ipc_debug_cnt = 0;

	usb_set_intfdata(intf, NULL);

	modem_notify_event(MODEM_EVENT_DISCONN);

	return;
}

static int modem_hsic_set_pipe(struct usb_link_device *usb_ld,
			const struct usb_host_interface *desc, int pipe)
{
	if (pipe < 0 || pipe >= IF_USB_DEVNUM_MAX) {
		MIF_ERR("undefined endpoint, exceed max\n");
		return -EINVAL;
	}

	MIF_DEBUG("set %d\n", pipe);

	if ((usb_pipein(desc->endpoint[0].desc.bEndpointAddress)) &&
	    (usb_pipeout(desc->endpoint[1].desc.bEndpointAddress))) {
		usb_ld->devdata[pipe].rx_pipe = usb_rcvbulkpipe(usb_ld->usbdev,
				desc->endpoint[0].desc.bEndpointAddress);
		usb_ld->devdata[pipe].tx_pipe = usb_sndbulkpipe(usb_ld->usbdev,
				desc->endpoint[1].desc.bEndpointAddress);
	} else if ((usb_pipeout(desc->endpoint[0].desc.bEndpointAddress)) &&
		   (usb_pipein(desc->endpoint[1].desc.bEndpointAddress))) {
		usb_ld->devdata[pipe].rx_pipe = usb_rcvbulkpipe(usb_ld->usbdev,
				desc->endpoint[1].desc.bEndpointAddress);
		usb_ld->devdata[pipe].tx_pipe = usb_sndbulkpipe(usb_ld->usbdev,
				desc->endpoint[0].desc.bEndpointAddress);
	} else {
		MIF_ERR("undefined endpoint\n");
		return -EINVAL;
	}

	return 0;
}


static struct usb_id_info hsic_channel_info;

static int __devinit modem_hsic_probe(struct usb_interface *intf,
					const struct usb_device_id *id)
{
	int err;
	int pipe;
	const struct usb_cdc_union_desc *union_hdr;
	const struct usb_host_interface *data_desc;
	unsigned char *buf = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *data_intf;
	struct usb_device *usbdev = interface_to_usbdev(intf);
	struct usb_driver *usbdrv = to_usb_driver(intf->dev.driver);
	struct usb_id_info *info = (struct usb_id_info *)id->driver_info;
	struct usb_link_device *usb_ld = info->usb_ld;
	struct usb_interface *control_interface;
	struct usb_device *root_usbdev= to_usb_device(intf->dev.parent->parent);

	pr_info("hsic usbdev=0x%p, intf->dev=0x%p\n", usbdev, &intf->dev);
	global_usbdev = usbdev;
	pm_runtime_init(&intf->dev);

	usb_ld->usbdev = usbdev;
	pm_runtime_forbid(&usbdev->dev);
	usb_ld->link_pm_data->dpm_suspending = false;
	usb_ld->link_pm_data->ipc_debug_cnt = 0;

	union_hdr = NULL;
	/* for WMC-ACM compatibility, WMC-ACM use an end-point for control msg*/
	if (intf->altsetting->desc.bInterfaceSubClass != USB_CDC_SUBCLASS_ACM) {
		MIF_ERR("ignore Non ACM end-point\n");
		return -EINVAL;
	}
	if (!buflen) {
		if (intf->cur_altsetting->endpoint->extralen &&
				    intf->cur_altsetting->endpoint->extra) {
			buflen = intf->cur_altsetting->endpoint->extralen;
			buf = intf->cur_altsetting->endpoint->extra;
		} else {
			MIF_ERR("Zero len descriptor reference\n");
			return -EINVAL;
		}
	}
	while (buflen > 0) {
		if (buf[1] == USB_DT_CS_INTERFACE) {
			switch (buf[2]) {
			case USB_CDC_UNION_TYPE:
				if (union_hdr)
					break;
				union_hdr = (struct usb_cdc_union_desc *)buf;
				break;
			default:
				break;
			}
		}
		buf += buf[0];
		buflen -= buf[0];
	}
	if (!union_hdr) {
		MIF_ERR("USB CDC is not union type\n");
		return -EINVAL;
	}
	control_interface = usb_ifnum_to_if(usbdev, union_hdr->bMasterInterface0);
	control_interface->needs_remote_wakeup = 0;
	pm_runtime_set_autosuspend_delay(&root_usbdev->dev, 200); /*200ms*/

	err = usb_control_msg(usbdev, usb_sndctrlpipe(usbdev, 0),
			USB_CDC_REQ_SET_CONTROL_LINE_STATE,
			USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			ACM_CTRL_DTR | ACM_CTRL_RTS,
			control_interface->altsetting[0].desc.bInterfaceNumber,
			NULL, 0, 5000);
	if (err < 0 )
		MIF_ERR("set RTS/CTS failed\n");
 
	data_intf = usb_ifnum_to_if(usbdev, union_hdr->bSlaveInterface0);
	if (!data_intf) {
		MIF_ERR("data_inferface is NULL\n");
		return -ENODEV;
	}

	data_desc = data_intf->altsetting;
	if (!data_desc) {
		MIF_ERR("data_desc is NULL\n");
		return -ENODEV;
	}

	pipe = intf->altsetting->desc.bInterfaceNumber / 2;
	if (modem_hsic_set_pipe(usb_ld, data_desc, pipe) < 0)
		return -EINVAL;

	usb_ld->devdata[pipe].usbdev                = usb_get_dev(usbdev);
	usb_ld->devdata[pipe].state                 = STATE_RESUMED;
	usb_ld->devdata[pipe].usb_ld                = usb_ld;
	usb_ld->devdata[pipe].data_intf             = data_intf;
	usb_ld->devdata[pipe].channel_id            = pipe;
	usb_ld->devdata[pipe].disconnected          = 0;
	usb_ld->devdata[pipe].control_interface     = control_interface;
	usb_ld->devdata[pipe].hsic_channel_rx_count = 0;
	usb_ld->devdata[pipe].hsic_channel_tx_count = 0;

	MIF_DEBUG("devdata usbdev = 0x%p\n", usb_ld->devdata[pipe].usbdev);

	usb_ld->suspended = 0;

	err = usb_driver_claim_interface(usbdrv, data_intf,
				(void *)&usb_ld->devdata[pipe]);
	if (err < 0) {
		MIF_ERR("usb_driver_claim() failed\n");
		return err;
	}

	pm_suspend_ignore_children(&usbdev->dev, true);

	usb_set_intfdata(intf, (void *)&usb_ld->devdata[pipe]);

	/* rx start for this endpoint */
	hsic_rx_submit(usb_ld, &usb_ld->devdata[pipe], GFP_KERNEL);

	wake_lock(&usb_ld->link_pm_data->l2_wake);

	if (pipe == HSIC_MAX_PIPE_ORDER_NR) {
		pr_info("%s pipe:%d\n", __func__, HSIC_MAX_PIPE_ORDER_NR);
		spin_lock_init(&usb_ld->link_pm_data->pm_data_lock);
		init_waitqueue_head(&usb_ld->link_pm_data->waitqueue);
		usb_ld->link_pm_data->state = ACM_READY;
		usb_ld->if_usb_connected = 1;
		modem_notify_event(MODEM_EVENT_CONN);

		hsic_set_autosuspend_delay(usbdev, 200);
		pm_runtime_allow(&usbdev->dev);
		pm_runtime_allow(usbdev->dev.parent->parent);
		pm_suspend_ignore_children(&usbdev->dev, true);
		
		skb_queue_purge(&usb_ld->ld.sk_raw_tx_q);
		usb_ld->ld.com_state = COM_ONLINE;
	}

	MIF_DEBUG("successfully done\n");

	return 0;
}

static void modem_hsic_free_pipe_data(struct usb_link_device *usb_ld)
{
	int i;

	for (i = 0; i < IF_USB_DEVNUM_MAX; i++) {
		if (usb_ld->devdata[i].rx_buf)
			kfree(usb_ld->devdata[i].rx_buf);
		if (usb_ld->devdata[i].urb)
			usb_kill_urb(usb_ld->devdata[i].urb);
	}
}

static struct usb_id_info hsic_channel_info = {
	.intf_id = IPC_CHANNEL,
};

static struct usb_device_id modem_hsic_usb_ids[] = {
	{
          USB_DEVICE(IMC_MAIN_VID, IMC_MAIN_PID),
	  .driver_info = (unsigned long)&hsic_channel_info,
	},
	{}
};
MODULE_DEVICE_TABLE(usb, modem_hsic_usb_ids);

static struct usb_driver modem_hsic_driver = {
	.name                 = "cdc_modem",
	.probe                = modem_hsic_probe,
	.disconnect           = modem_hsic_disconnect,
	.id_table             = modem_hsic_usb_ids,
	.suspend              = modem_hsic_suspend,
	.resume               = modem_hsic_resume,
	.reset_resume         = modem_hsic_reset_resume,
	.supports_autosuspend = 1,
};

static int modem_hsic_init(struct link_device *ld)
{
	int ret;
	int i;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct if_usb_devdata *pipe_data;
	struct usb_id_info *id_info;

	/* to connect usb link device with usb interface driver */
	for (i = 0; i < ARRAY_SIZE(modem_hsic_usb_ids); i++) {
		id_info = (struct usb_id_info *)modem_hsic_usb_ids[i].driver_info;
		if (id_info)
			id_info->usb_ld = usb_ld;
	}

	/* allocate rx buffer for usb receive */
	for (i = 0; i < IF_USB_DEVNUM_MAX; i++) {
		pipe_data = &usb_ld->devdata[i];
		pipe_data->channel_id = i;
		pipe_data->rx_buf_size = 16 * 1024;

		pipe_data->rx_buf = kmalloc(pipe_data->rx_buf_size,
						GFP_DMA | GFP_KERNEL);
		if (!pipe_data->rx_buf) {
			modem_hsic_free_pipe_data(usb_ld);
			ret = -ENOMEM;
			break;
		}

		pipe_data->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!pipe_data->urb) {
			MIF_ERR("alloc urb fail\n");
			modem_hsic_free_pipe_data(usb_ld);
			return -ENOMEM;
		}
	}

	ret = usb_register(&modem_hsic_driver);
	if (ret) {
		MIF_ERR("usb_register_driver() fail : %d\n", ret);
		return ret;
	}

	MIF_INFO("modem_hsic_init() done : %d, usb_ld (0x%p)\n", ret, usb_ld);

	return ret;
}

static int hsic_pm_init(struct usb_link_device *usb_ld, void *data)
{
	int r;
	struct platform_device *pdev = (struct platform_device *)data;
	struct modem_data *pdata =
			(struct modem_data *)pdev->dev.platform_data;
	struct modemlink_pm_data *pm_pdata = pdata->link_pm_data;
	struct link_pm_data *pm_data =
			kzalloc(sizeof(struct link_pm_data), GFP_KERNEL);
	if (!pm_data) {
		MIF_ERR("link_pm_data is NULL\n");
		return -ENOMEM;
	}
	/* get link pm data from modemcontrol's platform data */
	pm_data->pdev_modem = data;

	pm_data->gpio_link_active = pdata->gpio_host_active;

	pm_data->gpio_hostwake    = pm_pdata->gpio_hostwake;
	pm_data->gpio_slavewake   = pm_pdata->gpio_slavewake;
	pm_data->gpio_link_enable = pm_pdata->gpio_link_enable;

	pm_data->irq_hostwake = gpio_to_irq(pm_data->gpio_hostwake);

	get_modemctl(pm_data)->irq_hostwake = pm_data->irq_hostwake;

	pm_data->usb_ld = usb_ld;
	pm_data->ipc_debug_cnt = 0;
	usb_ld->link_pm_data = pm_data;

	wake_lock_init(&pm_data->l2_wake, WAKE_LOCK_SUSPEND, "l2_hsic");
	wake_lock_init(&pm_data->rpm_wake, WAKE_LOCK_SUSPEND, "rpm_hsic");
	wake_lock_init(&pm_data->tx_async_wake, WAKE_LOCK_SUSPEND, "tx_hsic");

	r = request_threaded_irq(pm_data->irq_hostwake,
		NULL, host_wakeup_irq_handler,
		IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		"modem_hostwake", (void *)pm_data);
	if (r) {
		MIF_ERR("fail to request irq(%d)\n", r);
		goto err_request_irq;
	}

	r = enable_irq_wake(pm_data->irq_hostwake);
	if (r) {
		MIF_ERR("failed to enable_irq_wake:%d\n", r);
		goto err_set_wake_irq;
	}

	/* create work queue & init work for runtime pm */
	pm_data->wq = create_singlethread_workqueue("linkpmd");
	if (!pm_data->wq) {
		MIF_ERR("fail to create wq\n");
		goto err_create_wq;
	}

	pm_data->pm_notifier.notifier_call = hsic_pm_notifier_event;
	register_pm_notifier(&pm_data->pm_notifier);

	init_completion(&pm_data->active_done);
	INIT_DELAYED_WORK(&pm_data->hsic_pm_work, hsic_pm_runtime_work);
	INIT_DELAYED_WORK(&pm_data->hsic_pm_start, hsic_pm_runtime_start);

	return 0;

err_create_wq:
	disable_irq_wake(pm_data->irq_hostwake);
err_set_wake_irq:
	free_irq(pm_data->irq_hostwake, (void *)pm_data);
err_request_irq:
	kfree(pm_data);
	return r;
}

struct link_device *hsic_create_link_device(void *data)
{
	int ret;
	struct usb_link_device *usb_ld;
	struct link_device *ld;

	usb_ld = kzalloc(sizeof(struct usb_link_device), GFP_KERNEL);
	if (!usb_ld)
		return NULL;

	INIT_LIST_HEAD(&usb_ld->ld.list);
	skb_queue_head_init(&usb_ld->ld.sk_raw_tx_q);

	ld = &usb_ld->ld;

	ld->name = "hsic";
	ld->init_comm = hsic_init_channel;
	ld->terminate_comm = hsic_close_channel;
	ld->send = hsic_send;
	ld->com_state = COM_NONE;

	ld->tx_wq = create_singlethread_workqueue("usb_tx_wq");
	if (!ld->tx_wq) {
		MIF_ERR("fail to create work Q.\n");
		goto err;
	}

	INIT_DELAYED_WORK(&ld->tx_delayed_work, hsic_tx_work);
	INIT_DELAYED_WORK(&usb_ld->rx_retry_work, hsic_rx_retry_work);
	usb_ld->rx_retry_cnt = 0;

	/* create link pm device */
	ret = hsic_pm_init(usb_ld, data);
	if (ret)
		goto err;

	ret = modem_hsic_init(ld);
	if (ret)
		goto err;

	MIF_INFO("%s : create_link_device DONE\n", usb_ld->ld.name);

	modem_notify_event(MODEM_EVENT_BOOT_INIT);

	return (void *)ld;
err:
	kfree(usb_ld);
	return NULL;
}

static void __exit modem_hsic_exit(void)
{
	usb_deregister(&modem_hsic_driver);
}

