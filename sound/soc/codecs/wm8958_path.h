/*
 * linux/sound/soc/codecs/wm8958_path.h
 *
 *
 * Copyright (C) 2011 Meizu, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 */

#include "wm8994.h"

#include <linux/mfd/wm8994/core.h>
#include <linux/mfd/wm8994/registers.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>

#ifndef	__WM8958_PATH_H__
#define	__WM8958_PATH_H__

#if defined(CONFIG_MACH_M040) || defined(CONFIG_MACH_M041)
#define	VOL_NORMAL_SPK				(0x39+0)							//0dB 0dB = 0x39
#else
#define	VOL_NORMAL_SPK				(0x39+4)							//4dB 0dB = 0x39
#endif
#define	VOL_NORMAL_SPK_BOOST		(0<<WM8994_SPKOUTL_BOOST_SHIFT)		// 0dB
#define	VOL_NORMAL_REC				(0x39+4)							// 4dB
#define	VOL_NORMAL_HP2				(0<<WM8994_HPOUT2_VOL_SHIFT)		// 0:0dB  1: -6dB
#define	VOL_NORMAL_HP				(0x39-6)							// -6dB
#define	VOL_NORMAL_HP_NR			(0x39-18)							// -18dB
#define	VOL_NORMAL_HP_OSEA				(0x39-10)							// -10dB
#define	VOL_NORMAL_HP_NR_OSEA			(0x39-22)							// -22dB

#define	VOL_RING_SPK				(0x39+4)							//4dB 0dB = 0x39
#define	VOL_RING_SPK_BOOST			(0<<WM8994_SPKOUTL_BOOST_SHIFT)		// 0dB
#define	VOL_RING_REC				(0x39+4)							// 4dB
#define	VOL_RING_HP2				(0<<WM8994_HPOUT2_VOL_SHIFT)		// 0:0dB  1: -6dB
#define	VOL_RING_HP					(0x39-24)							// -24dB

#define	VOL_INCALL_SPK_BOOST		(7<<WM8994_SPKOUTL_BOOST_SHIFT)		// 12dB
#define	VOL_INCALL_SPK				(0x39+3)							// 6dB 0dB = 0x39
#define	VOL_INCALL_REC				(0x39+6)							// 4dB 0dB = 0x39
#define	VOL_INCALL_AIF2DAC_BOOST	(0<<WM8994_AIF2DAC_BOOST_SHIFT)		// 0dB
#define	VOL_INCALL_HP				(0x39+2)							// 4dB
#define	VOL_INCALL_HP2				(0<<WM8994_HPOUT2_VOL_SHIFT)		// 0:0dB  1: -6dB

#define	VOL_VOIP_SPK_BOOST			(7<<WM8994_SPKOUTL_BOOST_SHIFT)		// 6dB
#define	VOL_VOIP_SPK				(0x39+3)							//4dB 0dB = 0x39
#define	VOL_VOIP_REC				(0x39+6)							// 6dB
#define	VOL_VOIP_AIF2DAC_BOOST		(0<<WM8994_AIF2DAC_BOOST_SHIFT)		// 6dB
#define	VOL_VOIP_HP					(0x39+2)							// 0dB
#define	VOL_VOIP_HP2				(0<<WM8994_HPOUT2_VOL_SHIFT)		// 0:0dB  1: -6dB

// 0x19 = +21dB  0x1F = +30dB 0x07 = -6dB
#define	VOL_IN1L_BOOST_SPK					(1<<WM8994_IN1L_MIXINL_VOL_SHIFT)	// +30dB
#define	VOL_IN1R_BOOST_SPK					(1<<WM8994_IN1R_MIXINR_VOL_SHIFT)	// +30dB
#define	VOL_IN2L_BOOST_SPK					(1<<WM8994_IN2L_MIXINL_VOL_SHIFT)	// +30dB
#define	VOL_IN1L_SPK					(0x11)	// +9dB
#define	VOL_IN1R_SPK					(0x0B)	// +0dB
#define	VOL_IN2L_SPK					(0x1B)	// +24dB

// 0x19 = +21dB  0x1F = +30dB 0x07 = -6dB
#define	VOL_IN1L_BOOST_REC					(0<<WM8994_IN1L_MIXINL_VOL_SHIFT)	// 0dB
#define	VOL_IN1R_BOOST_REC					(0<<WM8994_IN1R_MIXINR_VOL_SHIFT)	// 0dB
#define	VOL_IN2L_BOOST_REC					(1<<WM8994_IN2L_MIXINL_VOL_SHIFT)	// 0dB
#define	VOL_IN1L_REC					(0x19)	// +24dB
#define	VOL_IN1R_REC					(0x19)	// +24dB
#define	VOL_IN2L_REC					(0x0D)	// +3dB

#define	VOL_AIF1_ADC1_RIGHT				(0xE0<<WM8994_AIF1ADC1R_VOL_SHIFT)	// +12dB
#define	VOL_AIF1_ADC1_LEFT				(0xE0<<WM8994_AIF1ADC1L_VOL_SHIFT)	// +12dB

#endif
