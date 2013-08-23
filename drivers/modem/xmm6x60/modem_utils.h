/*
 * Copyright (C) 2011 Samsung Electronics.
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

#ifndef __MODEM_UTILS_H__
#define __MODEM_UTILS_H__

#include <linux/rbtree.h>

char* format_hex_string(const unsigned char *buf, int count);

#define IS_CONNECTED(iod, ld) ((iod)->link_types & LINKTYPE((ld)->link_type))

 /*
  *find_linkdev - find a link device
  *@commons:	struct mif_common
  */
static inline struct link_device *find_linkdev(struct mif_common *commons,
		enum modem_link link_type)
{
	struct link_device *ld;
	list_for_each_entry(ld, &commons->link_dev_list, list) {
		if (ld->link_type == link_type)
			return ld;
	}
	return NULL;
}

/*
 *countbits - count number of 1 bits as fastest way
 *@n: number
 */
static inline unsigned int countbits(unsigned int n)
{
	unsigned int i;
	for (i = 0; n != 0; i++)
		n &= (n - 1);
	return i;
}

/* get iod from tree functions */

struct io_device *get_iod_with_channel(struct mif_common *commons,
					unsigned channel);

static inline struct io_device *link_get_iod_with_channel(
			struct link_device *ld, unsigned channel)
{
	struct io_device *iod;
	iod = get_iod_with_channel(&ld->mc->commons, channel);
	return (iod && IS_CONNECTED(iod, ld)) ? iod : NULL;
}

/* insert iod to tree functions */
struct io_device *insert_iod_with_channel(struct mif_common *commons,
			unsigned channel, struct io_device *iod);

/* iodev for each */
typedef void (*action_fn)(struct io_device *iod, void *args);
void iodevs_for_each(struct mif_common *commons, action_fn action, void *args);

/* netif wake/stop queue of iod */
void iodev_netif_wake(struct io_device *iod, void *args);
void iodev_netif_stop(struct io_device *iod, void *args);

#endif/*__MODEM_UTILS_H__*/
