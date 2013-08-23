/*
 * Touch key & led driver for meizu m040
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author:		
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
	  
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include	<linux/mx_qm.h>

#define	MODE_CURRENT			0x00
#define	MODE_PWM				0x01
#define	MODE_SLOPE			0x02
#define	MODE_SHOWTYPE		0x03
#define	MODE_STATE			0x04

#define DEVICE_MODE_OFF          0x00
#define DEVICE_MODE_OFF_1        0x01
#define DEVICE_MODE_ON_PWM0      0x02
#define DEVICE_MODE_ON_PWM1      0x03
#define DEVICE_MODE_ON_FULL      0x04
#define DEVICE_MODE_ON_BRHT       0x05
#define DEVICE_MODE_ON_BANK0      0x06
#define DEVICE_MODE_ON_BANK1      0x07
#define DEVICE_MODE_MAX      0x08

#define	DEVICE_STATE_IDLE		(0)
#define	DEVICE_STATE_NORMAL	(1)
#define	DEVICE_STATE_SLEEP	(2)
#define	DEVICE_STATE_SHUTDOWN	(3)

#define	GET_MODE(x)	((x>>8)&0x0F)
//////////////////////////////////////////////////////////////////
union LEDType                       // LED show byte 
{
    unsigned char byte;
    struct
    {
        unsigned char led1:1;      
        unsigned char led2:1;      
        unsigned char led3:1;      
        unsigned char led4:1;      
        unsigned char led0:1;      //Cycle LED
        unsigned char delay_cnt:3; 
    };
}; 
unsigned char   ledarra0[] = {0xFF}; 
unsigned char   ledarra1[] = {((2<<5)|(1<<0)),((2<<5)|(1<<1)),((2<<5)|(1<<2)),((2<<5)|(1<<3)),((2<<5)|(1<<3)),((2<<5)|(1<<2)),((2<<5)|(1<<1)),((2<<5)|(1<<0)),0xFF}; 
unsigned char   ledarra2[] = {((4<<5)|0x01),((4<<5)|0x03),((4<<5)|0x06),((4<<5)|0x0C),((4<<5)|0x08),0xFF}; 
unsigned char   ledarra3[] = {((2<<5)|(1<<3)),((2<<5)|(1<<2)),((2<<5)|(1<<1)),((2<<5)|(1<<0)),0xFF}; 
unsigned char   ledarra4[] = {0x2F,0x20,0x2F,0x20,0xFF}; 
unsigned char   ledarra5[] = {((1<<5)|0x08),((1<<5)|0x0C),((1<<5)|0x0E),((3<<5)|0x0F),0xFF}; 
unsigned char   ledarra6[] = {((1<<5)|0x01),((1<<5)|0x03),((1<<5)|0x07),((3<<5)|0x0F),0xFF}; 
unsigned char   ledarra7[] = {((4<<5)|0x01),((4<<5)|0x02),((4<<5)|0x04),((4<<5)|0x08),((6<<5)|0x00),((6<<5)|0x00),((6<<5)|0x00),(0xFF-3)}; 
////////////////////////////////////////////////////////////////// 

 /*led private data*/
struct mx_qm_led {
	 struct mx_qm_data *data;
	 struct led_classdev led_cdev;
	 struct mutex mutex;
	 int id;
#ifdef CONFIG_HAS_EARLYSUSPEND
	 struct early_suspend early_suspend;
#endif
};
static int gSlope = 0;
static int gPWM = 0;
 
 static int mx_qm_set_led_current(struct led_classdev *led_cdev, int cur)
 {
	 struct mx_qm_led *led =
			 container_of(led_cdev, struct mx_qm_led, led_cdev);
	 struct mx_qm_data *mx = led->data;
	 int ret = 0;
	 
	 /*led current level, from 0~15
	 default we set the slope cycle to 3 second */
	cur = (3<<6) | ( (cur >> 4) & 0xF );
	 
	pr_debug("%s: id = %d   cur  = 0x%.2X\n", __func__,led->id,cur); 

	ret = mx->i2c_writebyte(mx->client,(LED_REG_CUR0+led->id), cur);

	return ret;
}
 
static int mx_qm_set_led_pwm(struct led_classdev *led_cdev, int pwm)
{
	struct mx_qm_led *led =
		 container_of(led_cdev, struct mx_qm_led, led_cdev);
	struct mx_qm_data *mx = led->data;
	int ret = 0;
	
	gPWM = pwm;

	//pwm = pwm *63 /100;

	/*led pwm level, from 0~63*/
	pwm = (pwm >> 2) & 0x3F;

	pr_debug("%s: pwm  = 0x%X\n", __func__,pwm); 

	ret = mx->i2c_writebyte(mx->client, LED_REG_PWM, (u8)pwm);

	return ret;
}
 
static int mx_qm_set_led_slope(struct led_classdev *led_cdev, int enable)
{
	struct mx_qm_led *led =
		 container_of(led_cdev, struct mx_qm_led, led_cdev);
	struct mx_qm_data *mx = led->data;
	int ret = 0;

	 gSlope = !!enable;

	pr_debug("%s: enable  = 0x%X\n", __func__,enable); 

	//if(enable)
	//	ret = mx->i2c_writebyte(mx->client,LED_REG_CUR0,0xCF);	  
	ret = mx->i2c_writebyte(mx->client,LED_REG_SLOPE,enable);	  

	return ret;
}  

static int mx_qm_set_led_mode(struct led_classdev *led_cdev, int mode)
{
	struct mx_qm_led *led =
		 container_of(led_cdev, struct mx_qm_led, led_cdev);
	struct mx_qm_data *mx = led->data;
	int ret = 0;
	unsigned char * pLedarr;
	unsigned char Ledsize;
	
	dev_dbg(led_cdev->dev, "%d\n",mode);

	//ret = mx->i2c_writebyte(mx->client,LED_REG_LEDMAUTO,mode);
	
	switch(mode	)
	{
		case 0:
			pLedarr = (unsigned char * )ledarra0;
			Ledsize = sizeof(ledarra0);
			break;
			
		case 1:
			pLedarr = (unsigned char * )ledarra1;
			Ledsize = sizeof(ledarra1);
			break;
			
		case 2:
			pLedarr = (unsigned char * )ledarra2;
			Ledsize = sizeof(ledarra2);
			break;
			
		case 3:
			pLedarr = (unsigned char * )ledarra3;
			Ledsize = sizeof(ledarra3);
			break;
			
		case 4:
			pLedarr = (unsigned char * )ledarra4;
			Ledsize = sizeof(ledarra4);
			break;
			
		case 5:
			pLedarr = (unsigned char * )ledarra5;
			Ledsize = sizeof(ledarra5);
			break;
			
		case 6:
			pLedarr = (unsigned char * )ledarra6;
			Ledsize = sizeof(ledarra6);
			break;
			
		case 7:
			pLedarr = (unsigned char * )ledarra7;
			Ledsize = sizeof(ledarra7);
			break;

		default:
			pLedarr = (unsigned char * )ledarra0;
			Ledsize = sizeof(ledarra0);
			break;

	}
	ret = mx->i2c_writebuf(mx->client,LED_REG_LEDMAUTO,Ledsize,pLedarr);

	return ret;
}  

static int mx_qm_set_led_state(struct led_classdev *led_cdev, int state)
{
	 struct mx_qm_led *led =container_of(led_cdev, struct mx_qm_led, led_cdev);
	 struct mx_qm_data *mx = led->data;
	 int mx_state = -1;
	 int ret = 0;
	 
	switch( state )
	{
		case DEVICE_STATE_IDLE:
			mx_state  = QM_STATE_IDLE;
			break;
			
		case DEVICE_STATE_NORMAL:
			mx_state  = QM_STATE_NORMAL;
			break;
			
		case DEVICE_STATE_SLEEP:
			mx_state  = QM_STATE_SLEEP;
			break;
			
		case DEVICE_STATE_SHUTDOWN:
			mx_state  = QM_STATE_SHUTDOWN;
			break;

		default:
			ret = -1;
			break;
	}

	if( mx_state != -1 )
		ret = mx->i2c_writebyte(mx->client,QM_REG_STATUS,mx_state);

	return ret;	 
}


static void mx_qm_led_brightness_set(struct led_classdev *led_cdev,
		 enum led_brightness value)
{
	//struct mx_qm_led *led =container_of(led_cdev, struct mx_qm_led, led_cdev);
	//struct mx_qm_data *mx = led->data;
	int ret = 0;
	int mode;
	int data;

	dev_dbg(led_cdev->dev, "value = 0x%.4X \n",value);

	mode = GET_MODE(value);
	data = value & 0xFF; 

	switch( mode )
	{
	case MODE_CURRENT:
		ret = mx_qm_set_led_current(led_cdev,data);
		break;

	case MODE_PWM:
		ret = mx_qm_set_led_pwm(led_cdev,data);
		break;

	case MODE_SLOPE:
		ret = mx_qm_set_led_slope(led_cdev,data);
		break;
		
	case MODE_SHOWTYPE:
		ret = mx_qm_set_led_mode(led_cdev,data);	
		break;

	case MODE_STATE:
		ret = mx_qm_set_led_state(led_cdev,data);	
		break;

	default:		
		dev_err(led_cdev->dev, "mode  %d is valite \n",mode);
		ret = -EINVAL;			
		break;
	}

	if(ret < 0)
	dev_err(led_cdev->dev, "brightness set failed ret = %d \n",ret);

}
 
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mx_qm_led_early_suspend(struct early_suspend *h)
{
/*
	 struct mx_qm_led *led =
			 container_of(h, struct mx_qm_led, early_suspend);
	struct mx_qm_data *mx = led->data;
 
	 if( led->id == 0)
	 {		 
		 if( mx->LedVer == LED_VERSION1)
		 	tca6507_set_led_pwm(&led->led_cdev,0x1FF);
		 else
		 	bu26507_set_led_pwm(&led->led_cdev,0x1FF);
	 }
*/	 
}
 
static void mx_qm_led_late_resume(struct early_suspend *h)
{
/*
	 struct mx_qm_led *led =
			 container_of(h, struct mx_qm_led, early_suspend);
 
	 //led->led_cdev.brightness_set(&led->led_cdev,LED_OFF);
	 if( led->id == 0)
		led->led_cdev.brightness_set(&led->led_cdev,((MODE_PWM<<8) | gPWM)); 	
*/		
}
#endif
 
static int __devinit mx_qm_led_probe(struct platform_device *pdev)
{
	 struct mx_qm_data *data = dev_get_drvdata(pdev->dev.parent);
	 struct mx_qm_platform_data *pdata = dev_get_platdata(data->dev);
	 struct mx_qm_led *led;
	 char name[20];
	 int ret;
 
	 if (!pdata) {
		 dev_err(&pdev->dev, "no platform data\n");
		 ret = -ENODEV;
	 }
 
	 led = kzalloc(sizeof(*led), GFP_KERNEL);
	 if (led == NULL) {
		 ret = -ENOMEM;
		 goto err_mem;
	 }
 
	 led->id = pdev->id; 
	 led->data = data;
 
	 snprintf(name, sizeof(name), "%s%d", pdev->name, pdev->id);
	 led->led_cdev.name = name;
	 led->led_cdev.brightness = 0;
	 led->led_cdev.max_brightness= 0xFFF;
	 
	 led->led_cdev.brightness_set = mx_qm_led_brightness_set;
 
	 mutex_init(&led->mutex);
	 platform_set_drvdata(pdev, led);
 
	 ret = led_classdev_register(&pdev->dev, &led->led_cdev);
	 if (ret < 0)
		 goto err_register_led;
 
#ifdef CONFIG_HAS_EARLYSUSPEND
	 led->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	 led->early_suspend.suspend = mx_qm_led_early_suspend;
	 led->early_suspend.resume = mx_qm_led_late_resume;
	 register_early_suspend(&led->early_suspend);
#endif
 
	 return 0;
 
 err_register_led:
	 kfree(led);
 err_mem:
	 return ret;
}
 
static int __devexit mx_qm_led_remove(struct platform_device *pdev)
{
	 struct mx_qm_led *led = platform_get_drvdata(pdev);
 
#ifdef CONFIG_HAS_EARLYSUSPEND
	 unregister_early_suspend(&led->early_suspend);
#endif
	 led_classdev_unregister(&led->led_cdev);
	 kfree(led);
 
	 return 0;
}
 
const struct platform_device_id mx_qm_id[] = {
	 { "mx-qm-led",0 },
	 { },
};
 
static struct platform_driver mx_qm_led_driver = {
	 .driver = {
		 .name	= "mx-qm-led",
		 .owner = THIS_MODULE,
	 },
	 .probe  = mx_qm_led_probe,
	 .remove = __devexit_p(mx_qm_led_remove),
	 .id_table = mx_qm_id,
};
 
static int __init mx_qm_led_init(void)
{
	return platform_driver_register(&mx_qm_led_driver);
}
module_init(mx_qm_led_init);
 
static void __exit mx_qm_led_exit(void)
{
	platform_driver_unregister(&mx_qm_led_driver);
}
module_exit(mx_qm_led_exit); 


MODULE_AUTHOR("Chwei <Chwei@meizu.com>");
MODULE_DESCRIPTION("MX QMatrix Sensor Leds");
MODULE_LICENSE("GPL");
