/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
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

#ifndef __MODEM_LINK_DEVICE_USB_H__
#define __MODEM_LINK_DEVICE_USB_H__


enum {
	IF_USB_TTY0_EP = 0,
	IF_USB_TTY1_EP,
	IF_USB_TTY2_EP,
	IF_USB_TTY3_EP,
	IF_USB_DEVNUM_MAX,
};

/* each pipe has 2 ep for in/out */
#define LINKPM_DEV_NUM	(IF_USB_DEVNUM_MAX * 2)
/******************/
/* xmm6260 specific */

#define IOCTL_LINK_CONTROL_ENABLE	_IO('o', 0x30)
#define IOCTL_LINK_CONTROL_ACTIVE	_IO('o', 0x31)
#define IOCTL_LINK_GET_HOSTWAKE		_IO('o', 0x32)
#define IOCTL_LINK_CONNECTED		_IO('o', 0x33)
#define IOCTL_LINK_SET_BIAS_CLEAR	_IO('o', 0x34)

/* VID,PID for IMC - XMM6260, XMM6262*/
#define IMC_BOOT_VID		0x058b
#define IMC_BOOT_PID		0x0041
#define IMC_MAIN_VID		0x1519
#define IMC_MAIN_PID		0x0020
/* VID,PID for STE - M7400 */
#define STE_BOOT_VID		0x04cc
#define STE_BOOT_PID		0x7400
#define STE_MAIN_VID		0x04cc
#define STE_MAIN_PID		0x2333

enum {
	BOOT_DOWN = 0,
	IPC_CHANNEL
};

enum ch_state {
	STATE_SUSPENDED,
	STATE_RESUMED,
};

/******************/

struct link_pm_info {
	struct usb_link_device *usb_ld;
};

struct usb_id_info {
	int intf_id;
	struct usb_link_device *usb_ld;
};

typedef enum {
	ACM_READY,
	ACM_READ,
	ACM_WRITE,
	ACM_CONNECT,
	ACM_SUSPEND,
	ACM_RESUME,
	ACM_UNKNOWN
} acm_wait_state_t;

struct link_pm_data {
	struct usb_link_device *usb_ld;
	unsigned irq_hostwake;
	unsigned gpio_link_enable;
	unsigned gpio_link_active;
	unsigned gpio_hostwake;
	unsigned gpio_slavewake;

	struct workqueue_struct *wq;
	struct completion active_done;
	struct delayed_work hsic_pm_work;
	struct delayed_work hsic_pm_start;
	bool resume_requested;
	int resume_retry_cnt;

	struct wake_lock l2_wake;
	struct wake_lock rpm_wake;
	struct wake_lock tx_async_wake;
	struct notifier_block pm_notifier;
	bool dpm_suspending;
	
	spinlock_t      pm_data_lock; 
	wait_queue_head_t  waitqueue;
	acm_wait_state_t   state;

	/* Host wakeup toggle debugging */
	unsigned ipc_debug_cnt;
	unsigned long tx_cnt;
	unsigned long rx_cnt;

	void *pdev_modem;
};

struct if_usb_devdata {
	struct usb_interface *data_intf;
	struct usb_interface *control_interface;
	struct usb_link_device *usb_ld;
	struct usb_device *usbdev;
	unsigned int tx_pipe;
	unsigned int rx_pipe;
	u8 disconnected;

	int channel_id;
	struct urb *urb;
	void *rx_buf;
	unsigned int rx_buf_size;
	enum ch_state state;
	
	int hsic_channel_tx_count;
	int hsic_channel_rx_count;
};

struct usb_link_device {
	/*COMMON LINK DEVICE*/
	struct link_device ld;

	/*USB SPECIFIC LINK DEVICE*/
	struct usb_device	*usbdev;
	struct if_usb_devdata	devdata[IF_USB_DEVNUM_MAX];
	unsigned int		dev_count;
	unsigned int		suspended;
	int if_usb_connected;

	/* LINK PM DEVICE DATA */
	struct link_pm_data *link_pm_data;

	/*RX retry work by -ENOMEM*/
	struct delayed_work rx_retry_work;
	struct urb *retry_urb;
	unsigned rx_retry_cnt;
};
/* converts from struct link_device* to struct xxx_link_device* */
#define to_usb_link_device(linkdev) \
			container_of(linkdev, struct usb_link_device, ld)

#endif
