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

#ifndef __MODEM_PRJ_H__
#define __MODEM_PRJ_H__

#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <linux/netdevice.h>
#include <linux/rbtree.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>

enum com_state {
	COM_NONE,
	COM_ONLINE,
	COM_HANDSHAKE,
	COM_BOOT,
	COM_CRASH,
};

enum link_mode {
	LINK_MODE_INVALID = 0,
	LINK_MODE_IPC,
	LINK_MODE_BOOT,
	LINK_MODE_DLOAD,
	LINK_MODE_ULOAD,
};

struct vnet {
	int pkt_sz;
	struct io_device *iod;
	struct sk_buff *skb;
	struct net_device_stats stats;
};

/** struct skbuff_priv - private data of struct sk_buff
 * this is matched to char cb[48] of struct sk_buff
 */
struct skbuff_private {
	struct io_device *iod;
	struct link_device *ld;
};

static inline struct skbuff_private *skbpriv(struct sk_buff *skb)
{
	BUILD_BUG_ON(sizeof(struct skbuff_private) > sizeof(skb->cb));
	return (struct skbuff_private *)&skb->cb;
}

/** rx_alloc_skb - allocate an skbuff and set skb's iod, ld
 * @length:	length to allocate
 * @gfp_mask:	get_free_pages mask, passed to alloc_skb
 * @iod:	struct io_device *
 * @ld:		struct link_device *
 *
 * %NULL is returned if there is no free memory.
 */
static inline struct sk_buff *rx_alloc_skb(unsigned int length,
		gfp_t gfp_mask, struct io_device *iod, struct link_device *ld)
{
	struct sk_buff *skb = alloc_skb(length, gfp_mask);
	if (likely(skb)) {
		skbpriv(skb)->iod = iod;
		skbpriv(skb)->ld = ld;
	}
	return skb;
}

struct io_device {
	/* rb_tree node for an io device */
	struct rb_node node_chan;

	/* Name of the IO device */
	char *name;

	atomic_t opened;
	atomic_t is_iod_op;

	struct device *ttydev;
	struct net_device *ndev;
	struct tty_struct *tty;
	struct tty_driver *tty_driver;

	/* ID for channel on the link */
	unsigned id;
	enum modem_link link_types;
	enum dev_format format;
	enum modem_io io_typ;
	enum modem_network phone_net_type;

	bool use_handover;	/* handover 2+ link devices */

	/* Rx queue of sk_buff */
	struct sk_buff_head rx_q;

	/*
	** work for each io device, when delayed work needed
	** use this for private io device rx action
	*/
	struct delayed_work rx_work;

	/* called from linkdevice when a packet arrives for this iodevice */
	int (*recv)(struct io_device *iod, struct link_device *ld,
					const char *data, unsigned int len);

	struct modem_ctl *mc;

	struct wake_lock wakelock;
	long waketime;
	int atdebug;
	int (*atdebugfunc)(struct io_device *iod, const char* buf, int len);
	int send_delay;

	/* DO NOT use __current_link directly
	 * you MUST use skbpriv(skb)->ld in mc, link, etc..
	 */
	struct link_device *__current_link;
	struct mutex op_mutex;
};
#define to_io_device(misc) container_of(misc, struct io_device, miscdev)

/* get_current_link, set_current_link don't need to use locks.
 * In ARM, set_current_link and get_current_link are compiled to
 * each one instruction (str, ldr) as atomic_set, atomic_read.
 * And, the order of set_current_link and get_current_link is not important.
 */
#define get_current_link(iod) ((iod)->__current_link)
#define set_current_link(iod, ld) ((iod)->__current_link = (ld))

struct link_device {
	struct list_head  list;
	char *name;

	enum modem_link link_type;
	unsigned aligned;

	/* Modem data */
	struct modem_data *mdm_data;

	/* Modem control */
	struct modem_ctl *mc;

	/* Operation mode of the link device */
	enum link_mode mode;

	/* TX queue of socket buffers */
	struct sk_buff_head sk_raw_tx_q;

	struct sk_buff_head *skb_txq[MAX_IPC_DEV];

	struct workqueue_struct *tx_wq;
	struct work_struct tx_work;
	struct delayed_work tx_delayed_work;
	struct delayed_work tx_dwork;

	struct workqueue_struct *rx_wq;
	struct work_struct rx_work;
	struct delayed_work rx_delayed_work;

	enum com_state com_state;

	/* init communication - setting link driver */
	int (*init_comm)(struct link_device *ld, struct io_device *iod);

	/* terminate communication */
	void (*terminate_comm)(struct link_device *ld, struct io_device *iod);

	/* called by an io_device when it has a packet to send over link
	 * - the io device is passed so the link device can look at id and
	 *   format fields to determine how to route/format the packet
	 */
	int (*send)(struct link_device *ld, struct io_device *iod,
			struct sk_buff *skb);

};

struct modemctl_ops {
	int (*modem_on) (struct modem_ctl *);
	int (*modem_off) (struct modem_ctl *);
	int (*modem_reset) (struct modem_ctl *);
	int (*modem_renum) (struct modem_ctl *);
	int (*modem_flash_enum) (struct modem_ctl *);
	int (*modem_force_crash_exit) (struct modem_ctl *);
	int (*modem_dump_reset) (struct modem_ctl *);
};

/* mif_common - common data for all io devices and link devices and a modem ctl
 * commons : mc : iod : ld = 1 : 1 : M : 1
 */
struct mif_common {
	/* list of link devices */
	struct list_head link_dev_list;

	/* rb_tree root of io devices. */
	struct rb_root iodevs_tree_chan; /* group by channel */
};

struct modem_ctl {
	struct device *dev;
	struct tty_driver *tty_driver;
	char *name;
	struct modem_data *mdm_data;

	struct mif_common commons;

	int sim_state;
	unsigned gpio_cp_on;
	unsigned gpio_reset_req_n;
	unsigned gpio_cp_reset;
	unsigned gpio_host_active;
	unsigned gpio_phone_active;
	unsigned gpio_cp_reset_int;
	unsigned gpio_cp_dump_int;
	unsigned gpio_ap_dump_int;
	unsigned gpio_cp_off;
	unsigned gpio_sim_detect;
	unsigned gpio_dynamic_switching;
	unsigned gpio_hostwake;
	unsigned gpio_slavewake;

	int irq_phone_active;
	int irq_modem_reset;
	int irq_sim_detect;
	int irq_hostwake;

	struct workqueue_struct *rx_wq;

	struct modemctl_ops ops;
	struct io_device *iod;

	void (*gpio_revers_bias_clear)(void);
	void (*gpio_revers_bias_restore)(void);

	struct completion *l2_done;
	int enum_done;

	wait_queue_head_t  read_wq;
	wait_queue_head_t  conn_wq;
	struct wake_lock   modem_wakelock;
	int cp_flag;
	struct regulator *modem_usb_regulator;
};

#define to_modem_ctl(mif_common) \
		container_of(mif_common, struct modem_ctl, commons)

int meizu_ipc_init_io_device(struct io_device *iod);
int modem_tty_driver_init(struct modem_ctl *modemctl);

#endif
