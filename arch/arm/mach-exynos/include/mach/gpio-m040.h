/* linux/arch/arm/mach-exynos/include/mach/gpio-m040.h
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *
 * Author: lvcha qiu <lvcha@meizu.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.,
 *
 * Revision History
 *
 * Inital code : Apr 16 , 2012 : lvcha@meizu.com
 * 1. modify code for m040 : july  6, 2012 : wenbinwu@meizu.com
*/

#ifndef _MX040_GPIO_H
#define _MX040_GPIO_H

typedef enum {
	/* M340 EINT */
	M040_POWER_KEY = EXYNOS4_GPX0(0),
	M040_TOUCH_IRQ,
	M040_USB_IRQ,
	M040_PMIC1_IRQ,
	M040_PMIC0_IRQ,
	M040_HOME_KEY,
	M040_IR_IRQ,
	M040_CODEC_IRQ,

	M040_TOUCHPAD_IRQ = EXYNOS4_GPX1(0),
	M040_PMIC2_RESET_IRQ,
	M040_BL_IR_IRQ,
	M040_ACC_IRQ1,
	M040_WL_HOST_WAKE,
	M040_IPC_TRIG_IRQ,
	M040_IPC_SRDY_IRQ,
	M040_HDETEC_IRQ,

	M040_ACC_IRQ2 = EXYNOS4_GPX2(0),
	M040_BT_SUSPEND_REQ,
	M040_MHL_USB_IRQ,
	M040_GYR_DRDY_IRQ,
	M040_IPC_WAKEUP_IRQ,
	M040_VOLUMEUP_KEY,
	M040_DOCK_IRQ,
	M040_MHL_IRQ,

	M040_BAT_LOW_IRQ = EXYNOS4_GPX3(0),
	M040_ST2_IRQ,
	M040_VOLUMEDOWN_KEY,
	M040_BB_RST_IRQ,
	M040_FUEL_IRQ,
	M040_GYR_IRQ,
	M040_NULL_IRQ,
	M040_HDMI_HPD_IRQ,

	/* M30X GPIO INT */
	M040_CAMERA_ISP_IRQ = EXYNOS4_GPF2(3),
	M040_CHARGER_IRQ = EXYNOS4_GPY4(1),
	M040_TOUCHPADINT0_IRQ = EXYNOS4_GPY5(2),
} M040_IRQ;

typedef enum {
	M040_ADC_BAT,	/* battery voltage measure */
	M040_ADC_THERM,	/* Thermal Sensor measure */
	M040_ADC_ARM,	/* arm core voltage measure */
	M040_ADC_MIC,	/* mic key measure */
} M040_ADC;

typedef enum {
	M040_LED_ID1 =EXYNOS4_GPL2(0) ,
	M040_LED_ID2,
	M040_LED_ID3,
	M040_LED_ID4,
	M040_LED_ID5,
	M040_LED_ID6,
} M040_LED;

/* wifi and bluetooth zone */
typedef enum {
	M040_BT_RXD    = EXYNOS4_GPA0(0),
	M040_BT_TXD    = EXYNOS4_GPA0(1),
	M040_BT_CTS    = EXYNOS4_GPA0(2),
	M040_BT_RTS    = EXYNOS4_GPA0(3),
	M040_BT_RESET  = EXYNOS4_GPF0(3),
	M040_WL_POWER  = EXYNOS4_GPF1(2),
	M040_WL_WIFICS = EXYNOS4_GPY0(2),
	M040_BT_WAKE   = EXYNOS4_GPF0(1),
	M040_BT_POWER  = EXYNOS4_GPF3(5),
} M040_WIFI;

/* arm core, int, g3d voltage control pin */
typedef enum {
	M040_GPIO_DVS1 = EXYNOS4212_GPM0(5),
	M040_GPIO_DVS2 = EXYNOS4212_GPM0(1),
	M040_GPIO_DVS3 = EXYNOS4212_GPM0(2),
	M040_BUCK2_SEL = EXYNOS4212_GPM1(0),
	M040_BUCK3_SEL = EXYNOS4212_GPM0(3),
	M040_BUCK4_SEL = EXYNOS4212_GPM1(4),
} M040_DVFS;

/* modem control */
typedef enum {
	M040_IPC_TRIGIN     = EXYNOS4_GPY6(0),
	M040_MODEM_ON       = EXYNOS4_GPY2(3),
	M040_MODEM_RST0     = EXYNOS4_GPY0(4),  /*reset  PMU*/
	M040_MODEM_RST1     = EXYNOS4_GPY6(4),  /*reset  bb*/
	M040_IPC_SLAVE_WAKE = EXYNOS4_GPY2(2),
} M040_MODEM;

/* version control */
typedef enum {
	M040_VER_PIN0 = EXYNOS4_GPL0(1),
	M040_VER_PIN1 = EXYNOS4_GPL0(6),
//	M040_VER_PIN2,
//	M040_VER_PIN3,
} M040_VERSION;

/* Camera0 and Camera1 zone */
#define M040_ISP_EN	EXYNOS4212_GPM4(1)    /* isp 1.2v power(buck9) enable */
#define M040_FRONT_CAM_EN	EXYNOS4212_GPM3(2)    /* front camera 2.8v power(ldo 21) enable */
#define M040_FRONT_CAM_PWDN	EXYNOS4212_GPM2(3)    /* front camera pwdn */
#define M040_FRONT_CAM_SHUTDOWN	EXYNOS4212_GPM3(7)    /* front camera shutdown */
#define M040_FRONT_CAM_STROBE	EXYNOS4212_GPM1(3)    /* front camera strobe output */
#define M040_FRONT_CAM_FSIN	EXYNOS4212_GPM4(5)    /* front camera frame sync input */
#define M040_ISP_RST	EXYNOS4_GPF0(6)       /* isp reset */
#define M040_ISP_YCVZ	EXYNOS4_GPF1(3)       /* isp control ycv pin status */

/*codec zone*/
#define M040_NOISE_CANCELLER_WAKE	EXYNOS4_GPF1(7)
#define M040_NOISE_CANCELLER_RST	EXYNOS4_GPF2(0)

//#define M040_PCM_SELECT		EXYNOS4_GPF0(7)
#define M040_CODEC_LDO1_EN	EXYNOS4_GPF0(0)
#define M040_CODEC_LDO2_EN	EXYNOS4_GPF0(7)
#define M040_CODEC_V28_ON		EXYNOS4_GPF0(5)//TI CODEC

/* hdmi power 1.0v enable pin */
#define  M040_HDMI_P10EN            EXYNOS4212_GPJ1(0)
#define  M040_MHL_WAKE              EXYNOS4_GPY4(3)
#define  M040_MHL_RST               EXYNOS4_GPY4(5)
#define  M040_MHL12_EN              EXYNOS4212_GPM3(3)

#define  M040_GPIO_HOST_ACTIVE      EXYNOS4_GPY6(0)
#define  M040_GPIO_MODEM_ON         EXYNOS4_GPY2(3)
#define  M040_GPIO_MODEM_RST_FULL   EXYNOS4_GPY0(4)
#define  M040_GPIO_MODEM_RST        EXYNOS4_GPY6(4)
#define  M040_GPIO_SLAVE_WAKEUP     EXYNOS4_GPY2(2)
#define  M040_INT_USB               EXYNOS4_GPX0(2)
#define  M040_INT_USB_MHL           EXYNOS4_GPX2(2)
#define  M040_GPIO_MODEM_DUMP_INT   EXYNOS4_GPX1(6)
#define  M040_GPIO_MODEM_RESET_INT  EXYNOS4_GPX3(3)
#define  M040_GPIO_REQUEST_SUSPEND  EXYNOS4_GPX1(5)
#define  M040_GPIO_HOST_WAKEUP      EXYNOS4_GPX2(4)

/*SC8803G gpio define*/
#define  M041_GPIO_MODEM_POWER_ON         	EXYNOS4_GPY2(3)
#define  M041_GPIO_AP_RESEND      		EXYNOS4_GPY6(0)
#define  M041_GPIO_AP_RDY   		EXYNOS4_GPY0(4)
#define  M041_GPIO_AP_RTS     		EXYNOS4_GPY2(2)
#define  M041_GPIO_MODEM_RDY   EXYNOS4_GPX1(6)
#define  M041_GPIO_MODEM_RESEND  EXYNOS4_GPX1(5)
#define  M041_GPIO_MODEM_RTS      EXYNOS4_GPX2(4)
#define  M041_GPIO_MODEM_ALIVE  EXYNOS4_GPX3(3)
/*reserved gpio*/
#define  M041_GPIO_MODEM_TO_AP1        	EXYNOS4_GPY2(0)
#define  M041_GPIO_MODEM_TO_AP2        	EXYNOS4_GPX3(6)
#define  M041_GPIO_AP_TO_MODEM1        	EXYNOS4_GPY6(4)
#define  M041_GPIO_AP_TO_MODEM2        	EXYNOS4_GPY1(3)

/* touch pannel reset pin */
#define M040_TOUCH_RESET		EXYNOS4_GPC1(1)

/* touch pad */
#define M040_TOUCHPAD_INT		EXYNOS4_GPX1(0)
#define M040_TOUCHPAD_RESET		EXYNOS4_GPY6(6)
#define M040_TOUCHPAD_WAKE		EXYNOS4_GPY5(2)

/* spdif out enable pin */
#define M040_SPDIF_OUT			EXYNOS4_GPC1(2)

/* iNAND power enable pin */
#define M040_INAND_EN			EXYNOS4_GPK0(2)

/* gps zone */
#define M040_GPS_PWRON			EXYNOS4_GPF1(6)
#define M040_GPS_RST			EXYNOS4_GPF2(2)
#define M040_GPS_RXD			EXYNOS4_GPA1(0)
#define M040_GPS_TXD			EXYNOS4_GPA1(1)
#define M040_GPS_CTS			EXYNOS4_GPA1(2)
#define M040_GPS_RTS			EXYNOS4_GPA1(3)

/* All sensor power enable pin */
#define M040_SENSOR_EN		EXYNOS4_GPF3(2)

/* DOCK output pin */
#define M040_DOCK_OUTPUT		EXYNOS4_GPY5(7)

/* lcd reset pin */
#define  M040_LCD_RST    EXYNOS4212_GPM4(7)
#define  M040_LCD_TE     EXYNOS4_GPF0(2)
#define  M040_LCD_2V8ON  EXYNOS4212_GPM4(6)
#define  M040_LCD_N5VON  EXYNOS4212_GPM3(5)
#define  M040_LCD_5VON   EXYNOS4212_GPM4(4)
#define  M040_LCD_BL_EN  EXYNOS4212_GPM3(6)

/* Sub pmic zone */
#define  M040_TORCH_EN    EXYNOS4_GPY4(4)
#define  M040_USB_SELECT  EXYNOS4_GPY3(3)

/* gpio i2c define zone */
#define  M040_SCL_FUEL0     EXYNOS4_GPY6(3)
#define  M040_SDA_FUEL0     EXYNOS4_GPY6(7)
#define  M040_SCL_MFUEL0    EXYNOS4212_GPM2(2)
#define  M040_SDA_MFUEL0    EXYNOS4212_GPM0(0)
#define  M040_SCL_IR        EXYNOS4212_GPM1(1)
#define  M040_SDA_IR        EXYNOS4212_GPM4(0)
#define  M040_SCL_MHL       EXYNOS4_GPY4(2)
#define  M040_SDA_MHL       EXYNOS4_GPY4(6)
#define  M040_SCL_HPD       EXYNOS4_GPY4(0)
#define  M040_SDA_HPD       EXYNOS4_GPY4(7)
#define  M041_SCL_TOUCHPAD  EXYNOS4_GPY2(4)
#define  M041_SDA_TOUCHPAD  EXYNOS4_GPY3(7)
#define  M040_SCL_TOUCHPAD  EXYNOS4_GPA0(7)//EXYNOS4_GPY2(0)
#define  M040_SDA_TOUCHPAD  EXYNOS4_GPA0(6)//EXYNOS4_GPY1(3)
#define  M040_SCL_GY        EXYNOS4_GPY3(5)
#define  M040_SDA_GY        EXYNOS4_GPY2(5)
#define  M040_SCL_GS        EXYNOS4_GPF2(1)
#define  M040_SDA_GS        EXYNOS4_GPF3(0)
#define  M040_SCL_CP        EXYNOS4_GPY5(4)
#define  M040_SDA_CP        EXYNOS4_GPY3(1)
#define  M040_SCL_CHG       EXYNOS4_GPY1(1)
#define  M040_SDA_CHG       EXYNOS4_GPY5(5)
#define  M040_SCL_EARPHONE   EXYNOS4212_GPM4(2)
#define  M040_SDA_EARPHONE   EXYNOS4212_GPM4(3)

/*factory mode define zone*/
#define M040_GPIO_FACTORY_MODE EXYNOS4_GPL2(4)
#define M040_GPIO_TEST_LED EXYNOS4_GPL0(0)

#endif	/* _MX040_GPIO_H */
