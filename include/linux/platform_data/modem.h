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

#ifndef __MODEM_IF_H__
#define __MODEM_IF_H__

enum modem_t {
	IMC_XMM6260,
	IMC_XMM6262,
	DUMMY,
};

#define  MODEM_CONNECT_FLAG     0x0001
#define  MODEM_RESET_FLAG       0x0002
#define  MODEM_CRASH_FLAG       0x0004
#define  MODEM_DUMP_FLAG        0x0008
#define  MODEM_DISCONNECT_FLAG  0x0010
#define  MODEM_SIM_DETECT_FLAG  0x0020
#define  MODEM_INIT_ON_FLAG     0x0040

#define  MODEM_EVENT_MASK       0x007E

enum MODEM_EVENT_TYPE {
	MODEM_EVENT_POWEROFF,
	MODEM_EVENT_RESET,
	MODEM_EVENT_CRASH,
	MODEM_EVENT_DUMP,
	MODEM_EVENT_CONN,
	MODEM_EVENT_DISCONN,
	MODEM_EVENT_SIM,
	MODEM_EVENT_BOOT_INIT,
};

enum dev_format {
	IPC_FMT,
	IPC_RAW,
	IPC_RFS,
	IPC_CMD,
	IPC_BOOT,
	IPC_MULTI_RAW,
	IPC_RAMDUMP,
	MAX_DEV_FORMAT,
};
#define MAX_IPC_DEV	(IPC_RFS + 1)

enum modem_io {
	IODEV_TTY,
	IODEV_NET,
	IODEV_DUMMY,
};

enum modem_link {
	LINKDEV_UNDEFINED,
	LINKDEV_SPI,
	LINKDEV_HSIC,
	LINKDEV_MAX,
};
#define LINKTYPE(modem_link) (1u << (modem_link))

enum modem_network {
	UMTS_NETWORK,
	CDMA_NETWORK,
	LTE_NETWORK,
};

enum sipc_ver {
	NO_SIPC_VER = 0,
	SIPC_VER_40 = 40,
	SIPC_VER_41 = 41,
	SIPC_VER_42 = 42,
	SIPC_VER_50 = 50,
	MAX_SIPC_VER,
};

/**
 * struct modem_io_t - declaration for io_device
 * @name:	device name
 * @id:		contain format & channel information
 *		(id & 11100000b)>>5 = format  (eg, 0=FMT, 1=RAW, 2=RFS)
 *		(id & 00011111b)    = channel (valid only if format is RAW)
 * @format:	device format
 * @io_type:	type of this io_device
 * @links:	list of link_devices to use this io_device
 *		for example, if you want to use DPRAM and USB in an io_device.
 *		.links = LINKTYPE(LINKDEV_DPRAM) | LINKTYPE(LINKDEV_USB)
 * @tx_link:	when you use 2+ link_devices, set the link for TX.
 *		If define multiple link_devices in @links,
 *		you can receive data from them. But, cannot send data to all.
 *		TX is only one link_device.
 *
 * This structure is used in board-*-modem.c
 */
struct modem_io_t {
	char *name;
	int   id;
	enum dev_format format;
	enum modem_io io_type;
	enum modem_link links;
	enum modem_link tx_link;
};

struct modemlink_pm_data {
	char *name;
	/* link power contol 2 types : pin & regulator control */
	unsigned gpio_link_enable;
	/*unsigned gpio_link_active;*/
	unsigned gpio_hostwake;
	unsigned gpio_slavewake;
	int (*port_enable)(int, int);
	int *p_hub_status;
	bool has_usbhub;

	atomic_t freqlock;
	int (*cpufreq_lock)(void);
	int (*cpufreq_unlock)(void);

	int autosuspend_delay_ms; /* if zero, the default value is used */
};

struct modemlink_pm_link_activectl {
	int gpio_initialized;
	int gpio_request_host_active;
};

/* platform data */
struct modem_data {
	char *name;

	unsigned gpio_cp_on;
	unsigned gpio_cp_off;
	unsigned gpio_reset_req_n;
	unsigned gpio_cp_reset;
	unsigned gpio_host_active;
#ifndef	CONFIG_MACH_M03X
	unsigned gpio_phone_active;
#endif
	unsigned gpio_cp_reset_int;
	unsigned gpio_cp_dump_int;
	unsigned gpio_ap_dump_int;
	unsigned gpio_sim_detect;
	unsigned gpio_hostwake;
	unsigned gpio_slavewake;

	/* Modem component */
	enum modem_network  modem_net;
	enum modem_t        modem_type;
	enum modem_link     link_types;
	char               *link_name;

	/* Information of IO devices */
	unsigned  num_iodevs;
	struct    modem_io_t   *iodevs;

	/* Modem link PM support */
	struct modemlink_pm_data *link_pm_data;

	void (*gpio_revers_bias_clear)(void);
	void (*gpio_revers_bias_restore)(void);
};

#define  MC_HOST_SUCCESS        0
#define  MC_HOST_HIGH           1
#define  MC_HOST_TIMEOUT        2
#define  MC_HOST_HALT           3

#define  HOSTWAKE_TRIGLEVEL	0

void modem_notify_event(int type);

extern int  modem_debug;
extern void modem_set_active_state(int state);

#define LOG_TAG "MODEMIF:"

#define MIF_ERR(fmt, ...) if (modem_debug > 0) \
	pr_err(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#define MIF_INFO(fmt, ...) if (modem_debug > 1) \
	pr_info(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#define MIF_TRACE(fmt, ...) if (modem_debug > 2) \
	pr_info("mif: %s: %d: called(%pF): " fmt, __func__,\
			__LINE__, __builtin_return_address(0), ##__VA_ARGS__)

#define MIF_DEBUG(fmt, ...) if (modem_debug > 3) \
	pr_info(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#endif
