/* linux/arch/arm/mach-exynos/include/mach/pm-core.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Based on arch/arm/mach-s3c2410/include/mach/pm-core.h,
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * EXYNOS4210 - PM core support for arch/arm/plat-s5p/pm.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#ifndef _PM_CORE_H_
#define  _PM_CORE_H_

#include <mach/regs-pmu.h>
#include <mach/regs-gpio.h>
#include <asm/mach-types.h>

#if defined (CONFIG_MX_SERIAL_TYPE) || defined(CONFIG_MX2_SERIAL_TYPE)

typedef enum {
	EINT_GROUP0,
	EINT_GROUP1,
	EINT_GROUP2,
	EINT_GROUP3,
	OTHER_INT,
} mx_int_type;

static inline void m030_set_wakeup_type(mx_int_type group, int pending)
{
	switch(group) {
	case EINT_GROUP0:
		if (pending & (1 << 4))
			mx_set_wakeup_type(MX_LOWBAT_WAKE);
		if (pending & (1 << 5))
			mx_set_wakeup_type(MX_CHARG_WAKE | MX_ALARM_WAKE);
		break;
	case EINT_GROUP1:
		if (pending & (1 << 0))
			mx_set_wakeup_type(MX_WIFI_WAKE);
		if (pending & (1 << 1))
			mx_set_wakeup_type(MX_BLUETOOTH_WAKE);
		if (pending & (1 << 3))
			mx_set_wakeup_type(MX_MODEM_RST_WAKE);
		if (pending & (1 << 6))
			mx_set_wakeup_type(MX_MODEM_WAKE);
		break;
	case EINT_GROUP2:
		if (pending & (1 << 3))
			mx_set_wakeup_type(MX_USB_HOST_WAKE);
		if (pending & (1 << 5))
			mx_set_wakeup_type(MX_USB_WAKE);
		if (pending & (1 << 6))
			mx_set_wakeup_type(MX_PLUS_KEY_WAKE);
		break;
	case EINT_GROUP3:
		if (pending & (1 << 0))
			mx_set_wakeup_type(MX_MINUS_KEY_WAKE);
		if (pending & (1 << 1))
			mx_set_wakeup_type(MX_KEY_POWER_WAKE);
		if (pending & (1 << 3))
			mx_set_wakeup_type(MX_KEY_HOME_WAKE);
		if (pending & (1 << 4))
			mx_set_wakeup_type(MX_IR_WAKE);
		break;
	case OTHER_INT:
		if (pending & S5P_WAKEUP_STAT_RTCALARM)
			mx_set_wakeup_type(MX_ALARM_WAKE);
		else if (pending & S5P_WAKEUP_STAT_RTCALARM)
			mx_set_wakeup_type(MX_TICK_WAKE);
		else if (pending & S5P_WAKEUP_STAT_AUDIO)
			mx_set_wakeup_type(MX_I2S_WAKE);
		else if (pending & S5P_WAKEUP_STAT_SYSTIMER)
			mx_set_wakeup_type(MX_SYSTIMER_WAKE);
	default:
		break;
	}
}

static inline void m032_set_wakeup_type(mx_int_type group, int pending)
{
	switch(group) {
	case EINT_GROUP0:
		if (pending & (1 << 0))
			mx_set_wakeup_type(MX_KEY_POWER_WAKE);
		if (pending & (1 << 2))
			mx_set_wakeup_type(MX_KEY_HOME_WAKE);
		if (pending & (1 << 3))
			mx_set_wakeup_type(MX_USB_WAKE);
		if (pending & (1 << 4))
			mx_set_wakeup_type(MX_ALARM_WAKE);
		if (pending & (1 << 5))
			mx_set_wakeup_type(MX_CHARG_WAKE);
		if (pending & (1 << 6))
			mx_set_wakeup_type(MX_IR_WAKE);
		break;
	case EINT_GROUP1:
		if (pending & (1 << 2))
			mx_set_wakeup_type(MX_USB_HOST_WAKE);//USB HOST IRQ
		break;
	case EINT_GROUP2:
		if (pending & (1 << 0))
			mx_set_wakeup_type(MX_MINUS_KEY_WAKE);
		if (pending & (1 << 1))
			mx_set_wakeup_type(MX_LOWBAT_WAKE);
		if (pending & (1 << 3))
			mx_set_wakeup_type(MX_WIFI_WAKE);
		if (pending & (1 << 4))
			mx_set_wakeup_type(MX_BLUETOOTH_WAKE);
		if (pending & (1 << 5))
			mx_set_wakeup_type(MX_PLUS_KEY_WAKE);
		break;
	case EINT_GROUP3:
		if (pending & (1 << 0))
			mx_set_wakeup_type(MX_MODEM_RST_WAKE);
		if (pending & (1 << 5))
			mx_set_wakeup_type(MX_MODEM_WAKE);
		break;
	case OTHER_INT:
		if (pending & S5P_WAKEUP_STAT_RTCALARM) {
			mx_set_wakeup_type(MX_ALARM_WAKE);
		} else if (pending & S5P_WAKEUP_STAT_RTCTICK) {
			mx_set_wakeup_type(MX_TICK_WAKE);
		} else if (pending & S5P_WAKEUP_STAT_AUDIO) {
			mx_set_wakeup_type(MX_I2S_WAKE);
		} else if (pending & S5P_WAKEUP_STAT_SYSTIMER) {
			mx_set_wakeup_type(MX_SYSTIMER_WAKE);
		}
	default:
		break;
	}
}

static inline void m040_set_wakeup_type(mx_int_type group, int mask)
{
	switch(group) {
	case 0:
		if (mask & (1 << 0)) {
			mx_set_wakeup_type(MX_KEY_POWER_WAKE);
		}
		if (mask & (1 << 2)) {
			mx_set_wakeup_type(MX_USB_WAKE);
		}
		if (mask & (1 << 3)) {
			mx_set_wakeup_type(MX_CHARG_WAKE);//PIMC1 IRQ
		}
		if (mask & (1 << 4)) {
			mx_set_wakeup_type(MX_ALARM_WAKE);//PIMC IRQ
		}
		if (mask & (1 << 5)) {
			mx_set_wakeup_type(MX_KEY_HOME_WAKE);
		}	
		if (mask & (1 << 6)) {
			mx_set_wakeup_type(MX_IR_WAKE);
		}			
		break;
	case EINT_GROUP1:
		if (mask & (1 << 0)) {
			mx_set_wakeup_type(MX_KEY_HOME_WAKE);//TOUCH PAD
		}
		if (mask & (1 << 4)) {
			mx_set_wakeup_type(MX_WIFI_WAKE);//
		}	
		if (mask & (1 << 7)) {
			mx_set_wakeup_type(MX_MIC_WAKE);//
		}	
		break;
	case EINT_GROUP2:
		if (mask & (1 << 1)) {
			mx_set_wakeup_type(MX_BLUETOOTH_WAKE);
		}
		if (mask & (1 << 2)) {
			mx_set_wakeup_type(MX_USB_WAKE);
		}
		if (mask & (1 << 4)) {
			mx_set_wakeup_type(MX_MODEM_WAKE);
		}		
		if (mask & (1 << 5)) {
			mx_set_wakeup_type(MX_PLUS_KEY_WAKE);
		}
		break;
	case EINT_GROUP3:
		if (mask & (1 << 0)) {
			mx_set_wakeup_type(MX_LOWBAT_WAKE);
		}
		if (mask & (1 << 2)) {
			mx_set_wakeup_type(MX_MINUS_KEY_WAKE);
		}
		if (mask & (1 << 3)) {
			mx_set_wakeup_type(MX_MODEM_RST_WAKE);
		}
		break;

	case OTHER_INT:
		if (mask & 0x2) {
			mx_set_wakeup_type(MX_ALARM_WAKE);
		} else if (mask & 0x4) {
			mx_set_wakeup_type(MX_TICK_WAKE);
		} else if (mask & 0x2000) {
			mx_set_wakeup_type(MX_I2S_WAKE);
		} else if (mask & 0x4000) {
			mx_set_wakeup_type(MX_SYSTIMER_WAKE);
		}
		break;
	default:
		break;
	}
}

static inline void set_wakeup_type(mx_int_type group, int pending)
{
#if defined(CONFIG_MACH_M040) || defined(CONFIG_MACH_M041)
	m040_set_wakeup_type(group, pending);
#else
	if(machine_is_m030()){
		m030_set_wakeup_type(group, pending);
	}else{
		m032_set_wakeup_type(group, pending);
	}
#endif
}
#endif

static inline void s3c_pm_debug_init_uart(void)
{
	/* nothing here yet */
}

static inline void s3c_pm_arch_prepare_irqs(void)
{
	unsigned long wakeup_mask_bit = S5P_WAKEUP_MASK_BIT;

	/* Mask externel GIC and GPS_ALIVE wakeup source */
	if (soc_is_exynos4212() || soc_is_exynos4412()) {
		s3c_irqwake_intmask |= 0x3BF0000;
		wakeup_mask_bit = 0x3FFFFFF;
	}

	__raw_writel((s3c_irqwake_intmask & wakeup_mask_bit), S5P_WAKEUP_MASK);
	__raw_writel(s3c_irqwake_eintmask, S5P_EINT_WAKEUP_MASK);
}

static inline void s3c_pm_arch_stop_clocks(void)
{
	/* nothing here yet */
}

static inline void s3c_pm_arch_show_resume_irqs(void)
{
#if defined (CONFIG_MX_SERIAL_TYPE) || defined(CONFIG_MX2_SERIAL_TYPE)
	unsigned int state;

	/* first: clean wakeup mark lastime */
	mx_set_wakeup_type(0);

	/* second: get wakeup info to handle */
	state = __raw_readl(S5P_WAKEUP_STAT);
	if (state & S5P_WAKEUP_STAT_EINT) {
		int eint_pend, i;
		for (i=EINT_GROUP0; i<=EINT_GROUP3; i++) {
			eint_pend = __raw_readl(S5P_EINT_PEND(i))&0xff;
			if (eint_pend)
				set_wakeup_type(i, eint_pend);
		}
	}

	if (state & ~S5P_WAKEUP_STAT_EINT)
		set_wakeup_type(OTHER_INT, state);
#else
	/* nothing here yet */
#endif
}

static inline void s3c_pm_arch_update_uart(void __iomem *regs,
					   struct pm_uart_save *save)
{
	/* nothing here yet */
}

static inline void s3c_pm_restored_gpios(void)
{
	/* nothing here yet */
}

static inline void s3c_pm_saved_gpios(void)
{
	/* nothing here yet */
}
#endif	/*_PM_CORE_H_*/
