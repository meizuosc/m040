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

#ifndef __MODEM_VARIATION_H__
#define __MODEM_VARIATION_H__

#include <linux/platform_data/modem.h>

#define DECLARE_MODEM_INIT(type)	\
	int type ## _init_modemctl_device(struct modem_ctl *mc,	\
				struct modem_data *pdata)
#define DECLARE_MODEM_INIT_DUMMY(type)	\
	static DECLARE_MODEM_INIT(type) { return 0; }

#define DECLARE_LINK_INIT(type)	\
		struct link_device *type ## _create_link_device(	\
		struct platform_device *pdev)
#define DECLARE_LINK_INIT_DUMMY(type)	\
	static DECLARE_LINK_INIT(type) { return NULL; }

#define MODEM_INIT_CALL(type)	type ## _init_modemctl_device
#define LINK_INIT_CALL(type)	type ## _create_link_device

/* add declaration of modem & link type */
/* modem device support */
DECLARE_MODEM_INIT_DUMMY(dummy)

#ifdef CONFIG_UMTS_MODEM_XMM6260
DECLARE_MODEM_INIT(xmm6260);
#else
DECLARE_MODEM_INIT_DUMMY(xmm6260)
#endif

#ifdef CONFIG_UMTS_MODEM_XMM6262
DECLARE_MODEM_INIT(xmm6262);
#else
DECLARE_MODEM_INIT_DUMMY(xmm6262)
#endif

/* link device support */
DECLARE_LINK_INIT_DUMMY(undefined)

#ifdef CONFIG_LINK_DEVICE_SPI
DECLARE_LINK_INIT(spi);
#else
DECLARE_LINK_INIT_DUMMY(spi)
#endif

#ifdef CONFIG_LINK_DEVICE_HSIC
DECLARE_LINK_INIT(hsic);
#else
DECLARE_LINK_INIT_DUMMY(hsic)
#endif

typedef int (*modem_init_call)(struct modem_ctl *, struct modem_data *);
static modem_init_call modem_init_func[] = {
	MODEM_INIT_CALL(xmm6260),
	MODEM_INIT_CALL(xmm6262),
	MODEM_INIT_CALL(dummy),
};

typedef struct link_device *(*link_init_call)(struct platform_device *);
static link_init_call link_init_func[] = {
	LINK_INIT_CALL(undefined),
	LINK_INIT_CALL(spi),
	LINK_INIT_CALL(hsic),
};

static int call_modem_init_func(struct modem_ctl *mc, struct modem_data *pdata)
{
	if (modem_init_func[pdata->modem_type])
		return modem_init_func[pdata->modem_type](mc, pdata);
	else
		return -ENOTSUPP;
}

static struct link_device *call_link_init_func(struct platform_device *pdev,
			enum modem_link link_type)
{
	if (link_init_func[link_type])
		return link_init_func[link_type](pdev);
	else
		return NULL;
}

#endif
