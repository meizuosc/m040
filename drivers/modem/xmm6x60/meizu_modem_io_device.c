/* /drivers/modem/meizu_ipc_io_device.c
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_utils.h"

int atdebugchannel = 0;
int atdebuglen = 0;

static int __init atdebugchannel_setup(char *args)
{
	int error;
	unsigned long val;

	error = strict_strtoul(args, 16, &val);
	if (!error)
		atdebugchannel = val;

	return error;
}
__setup("atdebugchannel=", atdebugchannel_setup);

static int __init atdebuglen_setup(char *args)
{
	int error;
	unsigned long val;

	error = strict_strtoul(args, 10, &val);
	if (!error)
		atdebuglen = val;

	return error;
}
__setup("atdebuglen=", atdebuglen_setup);

static int atdebugfunc(struct io_device *iod, const char* buf, int len)
{
	int atdebuglen = 0;

	if (iod->atdebug) {
		char *atdebugbuf;

		atdebuglen = iod->atdebug > len ? len : iod->atdebug;
		atdebugbuf = format_hex_string(buf, atdebuglen);
		pr_info("\n%pF, iod id:%d, data len:%d, data:\n%s\n",
			__builtin_return_address(0), iod->id, len, atdebugbuf);
	}

	return atdebuglen;
}

static ssize_t show_atdebug(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct io_device *iod = dev_get_drvdata(dev);
	char *p = buf;
	int atdebug;

	atdebug = iod->atdebug;
	p += sprintf(buf, "iod id:%d, atdebug:%d\n", iod->id, atdebug);

	return p - buf;
}

static ssize_t store_atdebug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long atdebug;
	struct io_device *iod = dev_get_drvdata(dev);

	ret = strict_strtoul(buf, 10, &atdebug);
	if (ret)
		return count;

	iod->atdebug = atdebug;

	return count;
}

static struct device_attribute attr_atdebug =
	__ATTR(atdebug, S_IRUGO | S_IWUSR, show_atdebug, store_atdebug);

static ssize_t show_send_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct io_device *iod = dev_get_drvdata(dev);
	char *p = buf;
	int send_delay;

	send_delay = iod->send_delay;
	p += sprintf(buf, "iod id:%d, send_delay:%d\n", iod->id, send_delay);

	return p - buf;
}

static ssize_t store_send_delay(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long send_delay;
	struct io_device *iod = dev_get_drvdata(dev);

	ret = strict_strtoul(buf, 10, &send_delay);
	if (ret)
		return count;

	iod->send_delay = send_delay;

	return count;
}

static struct device_attribute attr_send_delay =
	__ATTR(send_delay, S_IRUGO | S_IWUSR, show_send_delay, store_send_delay);

static inline int is_iod_handle_data(struct io_device *iod)
{
	return (atomic_read(&iod->is_iod_op));
}

static int vnet_is_opened(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);

	return (atomic_read(&vnet->iod->opened));
}

static int get_ip_packet_sz(struct io_device *iod, struct sk_buff *skb)
{
	struct iphdr rx_ip_hdr;
	int pkt_sz = 0;

	if (!skb) {
		return 0;
	}
	if (iod->atdebug)
		iod->atdebugfunc(iod, skb->data, skb->len);

	if (skb->len >= sizeof(struct iphdr))
		memcpy(&rx_ip_hdr, skb->data, sizeof(struct iphdr));
	else
		return -EINVAL;

	if(rx_ip_hdr.ihl != 5 && rx_ip_hdr.version != 4) {
		pr_err("%s no IP packet!\n", __func__);
		return -EINVAL;
	}
	pkt_sz = ntohs(rx_ip_hdr.tot_len);
	switch(rx_ip_hdr.protocol) {
	case IPPROTO_IP:
		MIF_DEBUG("IP dummy packet, sz:%d", pkt_sz);
		break;
	case IPPROTO_ICMP:
		MIF_DEBUG("IP ICMP packet, sz:%d", pkt_sz);
		break;
	case IPPROTO_IGMP:
		MIF_DEBUG("IP IGMP packet, sz:%d", pkt_sz);
		break;
	case IPPROTO_IPIP:
		MIF_DEBUG("IP tunnel packet, sz:%d", pkt_sz);
		break;
	case IPPROTO_TCP:
		MIF_DEBUG("IP TCP packet, sz:%d", pkt_sz);
		break;
	case IPPROTO_UDP:
		MIF_DEBUG("IP UDP packet, sz:%d", pkt_sz);
		break;
	default:
		break;
	}

	return pkt_sz;
}

static void hsic_tty_data_handler(struct io_device *iod)
{
	struct tty_struct *tty = iod->tty;
	struct sk_buff *skb = NULL;
	unsigned char *buf;
	int count;

	if(!(tty && tty->driver_data)) {
		skb_queue_purge(&iod->rx_q);
		return;
	}
	skb = skb_dequeue(&iod->rx_q);
	while(skb) {
		if (test_bit(TTY_THROTTLED, &tty->flags))
			break;
		wake_lock_timeout(&iod->wakelock, HZ*0.5);
		count = tty_prepare_flip_string(tty, &buf, skb->len);
		if (count <= 0) {
			MIF_ERR("%s:tty buffer avail size=%d\n",
							__func__, count);
			break;
		}
		if (skb->len > count) {
			MIF_ERR("skb->len %d > count %d\n", skb->len, count);
			memcpy(buf, skb->data, count);
			if (iod->atdebug)
				iod->atdebugfunc(iod, skb->data, count);
			skb_pull(skb, count);
			if (skb->len) {
				MIF_DEBUG("queue-head, skb->len = %d\n",
						skb->len);
				skb_queue_head(&iod->rx_q, skb);
			}
		} else {
			memcpy(buf, skb->data, count);
			if (iod->atdebug)
				iod->atdebugfunc(iod, skb->data, count);
			dev_kfree_skb_any(skb);
		}
		if(tty && tty->driver_data)
			tty_flip_buffer_push(tty);
		else
			break;
		skb = skb_dequeue(&iod->rx_q);
	}
	return;
}

static void hsic_net_data_handler(struct io_device *iod)
{
	struct net_device *ndev = iod->ndev;
	struct vnet *vnet = netdev_priv(ndev);
	struct sk_buff *skb = NULL;

	if (!vnet_is_opened(ndev)) {
		skb_queue_purge(&iod->rx_q);
		vnet->pkt_sz = 0;
		skb = vnet->skb;
		vnet->skb = NULL;
		dev_kfree_skb_any(skb);
		return;
	}

	do {
		wake_lock_timeout(&iod->wakelock, HZ * 0.5);
		skb = skb_dequeue(&iod->rx_q);
		if (!skb)
			break;
		vnet->pkt_sz = get_ip_packet_sz(iod, skb);
		if (vnet->pkt_sz <= 0) {
			dev_kfree_skb_any(skb);
			break;
		}
		skb->dev = ndev;
		skb->protocol = htons(ETH_P_IP);
		if (iod->atdebug)
			iod->atdebugfunc(iod, skb->data, skb->len);
		skb_reset_mac_header(skb);
		vnet->stats.rx_packets ++;
		vnet->stats.rx_bytes += vnet->pkt_sz;
		netif_rx(skb);
		pr_debug("%s rx size=%d", __func__, vnet->pkt_sz);
	} while(skb);

	return;
}

static void recv_data_handler_work(struct work_struct *work)
{
	struct io_device *iod = container_of(work, struct io_device,
				rx_work.work);
	
	switch (iod->io_typ) {
	case IODEV_TTY:
		mutex_lock(&iod->op_mutex);
		if (iod->tty)
			hsic_tty_data_handler(iod);
		mutex_unlock(&iod->op_mutex);
		break;
	case IODEV_NET:
		mutex_lock(&iod->op_mutex);
		hsic_net_data_handler(iod);
		mutex_unlock(&iod->op_mutex);
		break;
	default:
		MIF_ERR("wrong io_type : %d\n", iod->io_typ);
		break;
	}
}

/* called from link device when a packet arrives for this io device */
static int recv_data_handler(struct io_device *iod,
		struct link_device *ld, const char *data, unsigned int len)
{
	struct sk_buff *skb;
	int ret = len;

	// just return ok when no receiver
	if (atomic_read(&iod->opened) == 0)
		return ret;

	skb = rx_alloc_skb(len, GFP_ATOMIC, iod, ld);
	if (!skb) {
		MIF_ERR("fail alloc skb (%d)\n", __LINE__);
		return -ENOMEM;
	}

	switch (iod->io_typ) {
	case IODEV_TTY:
		memcpy(skb_put(skb, len), data, len);
		skb_queue_tail(&iod->rx_q, skb);
		atomic_inc(&iod->is_iod_op);
		queue_delayed_work(iod->mc->rx_wq, &iod->rx_work, 0);
		atomic_dec(&iod->is_iod_op);
		break;
	case IODEV_NET:
		memcpy(skb_put(skb, len), data, len);
		skb_queue_tail(&iod->rx_q, skb);
		atomic_inc(&iod->is_iod_op);
		queue_delayed_work(iod->mc->rx_wq, &iod->rx_work, 0);
		atomic_dec(&iod->is_iod_op);
		break;
	default:
		MIF_ERR("wrong io_type : %d\n", iod->io_typ);
		dev_kfree_skb_any(skb);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int vnet_open(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = vnet->iod;

	netif_start_queue(ndev);
	mutex_lock(&iod->op_mutex);
	atomic_inc(&iod->opened);
	mutex_unlock(&iod->op_mutex);

	return 0;
}

static int vnet_stop(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = vnet->iod;
	struct mif_common *commons = &iod->mc->commons;
	struct link_device *ld;

	mutex_lock(&iod->op_mutex);
	atomic_dec(&iod->opened);
	if (atomic_read(&iod->opened) == 0) {
		while (is_iod_handle_data(iod)) {
			mutex_unlock(&iod->op_mutex);
			msleep(1);
			mutex_lock(&iod->op_mutex);
		}
		flush_delayed_work(&iod->rx_work);
		skb_queue_purge(&iod->rx_q);
		netif_stop_queue(ndev);
		MIF_INFO("close iod = %s\n", iod->name);
		list_for_each_entry(ld, &commons->link_dev_list, list) {
			if (IS_CONNECTED(iod, ld) && ld->terminate_comm)
				ld->terminate_comm(ld, iod);
		}
	}

	mutex_unlock(&iod->op_mutex);
	return 0;
}

static int vnet_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = vnet->iod;
	struct link_device *ld = get_current_link(iod);
	int is_ip_packet = 0;
	int data_len  = 0;
	int ret = 0;

	if (iod->atdebug)
		iod->atdebugfunc(iod, skb->data, skb->len);
	wake_lock_timeout(&iod->wakelock, HZ * 0.5);
	is_ip_packet = (skb->protocol == htons(ETH_P_IP)) ? 1 : 0;
	skbpriv(skb)->iod = iod;
	skbpriv(skb)->ld = ld;
	data_len = skb->len;
	ret = ld->send(ld, iod, skb);
	if (ret < 0) {
		dev_kfree_skb_any(skb);
		ret = NETDEV_TX_BUSY;
	} else {
		if(is_ip_packet) {
			vnet->stats.tx_packets ++;
			vnet->stats.tx_bytes += data_len;
		}
	}
	ret = NETDEV_TX_OK;

	return ret;
}

static struct net_device_stats *vnet_get_stats(struct net_device *dev)
{
	struct vnet *vnet = netdev_priv(dev);

	return &vnet->stats;
}

static struct net_device_ops vnet_ops = {
	.ndo_open       = vnet_open,
	.ndo_stop       = vnet_stop,
	.ndo_get_stats  = vnet_get_stats,
	.ndo_start_xmit = vnet_xmit,
};

static void vnet_setup(struct net_device *ndev)
{
	ndev->mtu             = ETH_DATA_LEN;
	ndev->type            = ARPHRD_NONE;
	ndev->flags           = IFF_POINTOPOINT | IFF_NOARP;
	ndev->features        = 0;
	ndev->addr_len        = 0;
	ndev->destructor      = free_netdev;
	ndev->netdev_ops      = &vnet_ops;
	ndev->tx_queue_len    = 1000;
	ndev->watchdog_timeo  = 20 * HZ;
	ndev->hard_header_len = 0;
}

static int modem_tty_open(struct tty_struct *tty, struct file *f)
{
	struct modem_ctl *modemctl = tty->driver->driver_state;
	struct mif_common *commons;
	struct link_device *ld;
	struct io_device *iod;
	int ret = 0;

	iod = get_iod_with_channel(&modemctl->commons, tty->index);
	commons = &iod->mc->commons;

	mutex_lock(&iod->op_mutex);
	tty->driver_data = iod;
	iod->tty = tty;
	atomic_inc(&iod->opened);
	mutex_unlock(&iod->op_mutex);
	tty->low_latency = 1;

	list_for_each_entry(ld, &commons->link_dev_list, list) {
		if (IS_CONNECTED(iod, ld) && ld->init_comm) {
			ret = ld->init_comm(ld, iod);
			if (ret < 0) {
				MIF_ERR("%s: init_comm error: %d\n",
						ld->name, ret);
				break;
			}
		}
	}
	

	return ret;
}

static void modem_tty_close(struct tty_struct *tty, struct file *f)
{
	struct io_device *iod = tty->driver_data;
	struct mif_common *commons;
	struct link_device *ld;

	if(!iod)
		return;
	commons = &iod->mc->commons;
	mutex_lock(&iod->op_mutex);
	atomic_dec(&iod->opened);
	if (atomic_read(&iod->opened) == 0) {
		while (is_iod_handle_data(iod)) {
			mutex_unlock(&iod->op_mutex);
			msleep(1);
			mutex_lock(&iod->op_mutex);
		}
		skb_queue_purge(&iod->rx_q);
		flush_delayed_work(&iod->rx_work);
		MIF_INFO("close iod = %s\n", iod->name);
		tty->driver_data = NULL;
		iod->tty = NULL;
		list_for_each_entry(ld, &commons->link_dev_list, list) {
			if (IS_CONNECTED(iod, ld) && ld->terminate_comm)
				ld->terminate_comm(ld, iod);
		}
	}
	mutex_unlock(&iod->op_mutex);
}

static int modem_tty_write_room(struct tty_struct *tty)
{
	int ret = 128 * 1024;

	return ret;
}

static int
modem_tty_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	struct io_device *iod = tty->driver_data;
	struct link_device *ld;
	struct sk_buff *skb;
	size_t tx_size;
	int err;

	if (!iod) {
		pr_info("%s iod -ENODEV!\n", __func__);
		return -ENODEV;
	}

	ld = get_current_link(iod);

	skb = alloc_skb(count, GFP_ATOMIC);
	if (!skb) {
		MIF_ERR("fail alloc skb (%d)\n", __LINE__);
		return -ENOMEM;
	}
	wake_lock_timeout(&iod->wakelock, HZ * 0.5);

	memcpy(skb_put(skb, count), buf, count);

	if (iod->atdebug) {
		char *atdebugbuf;
		int atdebuglen = iod->atdebug > count ? count : iod->atdebug;

		atdebugbuf = format_hex_string(skb->data, atdebuglen);
		pr_info("%s, iod id:%d, data:\n%s\n", __func__,
				iod->id, atdebugbuf);
	}

	tx_size = skb->len;

	skbpriv(skb)->iod = iod;
	skbpriv(skb)->ld = ld;

	err = ld->send(ld, iod, skb);
	if (err < 0) {
		MIF_DEBUG("%s ld send error:%d!\n", __func__, err);
		dev_kfree_skb_any(skb);
		return err;
	}
	if (err != tx_size)
		MIF_DEBUG("WARNNING: wrong tx size: %s, format=%d "
			"count=%d, tx_size=%d, return_size=%d",
			iod->name, iod->format, count, tx_size, err);
	count = err;

	return count;
}

static void modem_tty_throttle(struct tty_struct *tty)
{
	struct io_device *iod = tty->driver_data;
	
	if (!iod)
		return;

	skb_queue_purge(&iod->rx_q);
}

static struct tty_operations tty_ops = {
	.open       = modem_tty_open,
	.close      = modem_tty_close,
	.write      = modem_tty_write,
	.throttle   = modem_tty_throttle,
	.write_room = modem_tty_write_room,
};

int modem_tty_driver_init(struct modem_ctl *modemctl)
{
	struct tty_driver *tty_driver;
	int ret;

	tty_driver = alloc_tty_driver(4);
	if (tty_driver == 0) {
		pr_err("%s alloc_tty_driver -ENOMEM!!\n", __func__);
		return -ENOMEM;
	}
	tty_driver->name         = "ttyACM";
	tty_driver->type         = TTY_DRIVER_TYPE_SERIAL;
	tty_driver->major        = 0;
	tty_driver->owner        = THIS_MODULE;
	tty_driver->subtype      = SERIAL_TYPE_NORMAL;
	tty_driver->minor_start  = 0;
	tty_driver->driver_name  = "tty_driver";
	tty_driver->init_termios = tty_std_termios;
	tty_driver->init_termios.c_iflag = 0;
	tty_driver->init_termios.c_oflag = 0;
	tty_driver->init_termios.c_cflag = B4000000 | CS8 | CREAD;
	tty_driver->init_termios.c_lflag = 0;
	tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
				TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(tty_driver, &tty_ops);
	ret = tty_register_driver(tty_driver);
	if (ret) {
		pr_err("%s error!!\n", __func__);
		return ret;
	}
	tty_driver->driver_state = modemctl;
	modemctl->tty_driver = tty_driver;

	return 0;
}

int meizu_ipc_init_io_device(struct io_device *iod)
{
	int ret = 0;
	struct vnet *vnet;

	iod->recv = recv_data_handler;
	switch (iod->io_typ) {
	case IODEV_TTY:
		skb_queue_head_init(&iod->rx_q);
		mutex_init(&iod->op_mutex);
		INIT_DELAYED_WORK(&iod->rx_work, recv_data_handler_work);
		iod->atdebugfunc = atdebugfunc;
		if (atdebugchannel & (0x1 << iod->id))
			if (atdebuglen)
				iod->atdebug = atdebuglen;
			else
				iod->atdebug = 255;
		else
			iod->atdebug = 0;
		iod->send_delay = 0;
		iod->ttydev = tty_register_device
			(iod->mc->tty_driver, iod->id, NULL);

		dev_set_drvdata(iod->ttydev, iod);
		ret = device_create_file(iod->ttydev, &attr_atdebug);
		if (ret)
			MIF_ERR("failed to create sysfs file : %s\n",
					iod->name);
		ret = device_create_file(iod->ttydev, &attr_send_delay);
		if (ret)
			MIF_ERR("failed to create send_delay sysfs file : %s\n",
					iod->name);
		break;
	case IODEV_NET:
		skb_queue_head_init(&iod->rx_q);
		mutex_init(&iod->op_mutex);
		INIT_DELAYED_WORK(&iod->rx_work, recv_data_handler_work);
		iod->atdebugfunc = atdebugfunc;
		if (atdebugchannel & (0x1 << iod->id))
			if (atdebuglen)
				iod->atdebug = atdebuglen;
			else
				iod->atdebug = 255;
		else
			iod->atdebug = 0;
		iod->send_delay = 5000;
		iod->ndev = alloc_netdev(sizeof(struct vnet), iod->name,
			vnet_setup);
		if (!iod->ndev) {
			MIF_ERR("failed to alloc netdev\n");
			return -ENOMEM;
		}
		ret = register_netdev(iod->ndev);
		if (ret)
			free_netdev(iod->ndev);

		dev_set_drvdata(&iod->ndev->dev, iod);
		ret = device_create_file(&iod->ndev->dev, &attr_atdebug);
		if (ret)
			MIF_ERR("failed to create sysfs file : %s\n",
					iod->name);
		ret = device_create_file(&iod->ndev->dev, &attr_send_delay);
		if (ret)
			MIF_ERR("failed to create send_delay sysfs file : %s\n",
					iod->name);
		vnet = netdev_priv(iod->ndev);
		MIF_DEBUG("(vnet:0x%p)\n", vnet);
		vnet->pkt_sz = 0;
		vnet->iod = iod;
		vnet->skb = NULL;
		break;
	default:
		MIF_ERR("wrong io_type : %d\n", iod->io_typ);
		return -EINVAL;
	}

	MIF_DEBUG("%s(%d) : init_io_device() done : %d\n",
				iod->name, iod->io_typ, ret);
	return ret;
}
