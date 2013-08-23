/*
 * fsa8108.c -- FSA8108 Jack detection driver
 *
 * Copyright (C) 2012 Fairchild semiconductor Co.Ltd
 * Author: Chris Jeong <Chris.jeong@fairchildsemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h> 
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/fsa8108.h>
#include <mach/gpio-common.h>
#include <plat/adc.h>



/***********************************************
* Register addresses	
***********************************************/
#define FSA8108_REG_DEVICE_ID	             1
#define FSA8108_REG_INT_1	                 2
#define FSA8108_REG_INT_2                    3
#define FSA8108_REG_INT_MASK_1	             4
#define FSA8108_REG_INT_MASK_2               5
#define FSA8108_REG_GLOBAL_MUL               6
#define FSA8108_REG_JDET_T                   7
#define FSA8108_REG_KEY_PRS_T                8
#define FSA8108_REG_MP3_MODE_T               9
#define FSA8108_REG_DET_T                 0x0a
#define FSA8108_REG_DEBOUNCE_T            0x0b
#define FSA8108_REG_CON                   0x0c
#define FSA8108_REG_COMPARATOR_12         0x0d
#define FSA8108_REG_COMPARATOR_34         0x0e
#define FSA8108_REG_RESET                 0x0f

/********************************************
* Register bits	
*********************************************/

/* FSA8108_REG_DEVICE_ID (0x01) */
#define FSA8108_VER_ID             (0x0f << 4)
#define FSA8108_VER_ID_SHIFT        4

/* FSA8108_REG_INT_1 (0x02) */
#define FSA8108_3POLE_CONNECT       0x01
#define FSA8108_4POLE_CONNECT       0x02
#define FSA8108_PLUG_DISCONNECT     0x04
#define FSA8108_SEND_END_PRESS      0x08
#define FSA8108_SEND_END_DOUBLE     0x10
#define FSA8108_SEND_END_LONG       0x20

/* FSA8108_REG_INT_2 (0x03) */
#define FSA8108_VOL_UP              0x01
#define FSA8108_VOL_UP_LONG_P       0x02
#define FSA8108_VOL_UP_LONG_R       0x04
#define FSA8108_VOL_DOWN            0x08
#define FSA8108_VOL_DOWN_LONG_P     0x10
#define FSA8108_VOL_DOWN_LOGN_R     0x20

/* FSA8108_REG_INT_MASK_1 (0x04) */

/* FSA8108_REG_INT_MASK_2 (0x05) */

/* FSA8108_REG_GLOBAL_MUL (0x06) */

/* FSA8108_REG_JDET_T (0x07) */
#define FSA8108_TDET_REM                   (0x0f)
#define FSA8108_TDET_REM_SHIFT              0
#define FSA8108_TDET_IN                    (0x0f << 4)
#define FSA8108_TDET_IN_SHIFT               4

/* FSA8108_REG_KEY_PRS_T (0x08) */
#define FSA8108_TLONG                      (0x0f)
#define FSA8108_TLONG_SHIFT                 0
#define FSA8108_TDOUBLE                    (0x0f << 4)
#define FSA8108_TDOUBLE_SHIFT               4

/* FSA8108_REG_MP3_MODE_T (0x09) */
#define FSA8108_TWAIT                      (0x0f << 4)
#define FSA8108_TWAIT_SHIFT                 0
#define FSA8108_TPOLL                      (0x0f << 4)
#define FSA8108_TPOLL_SHIFT                 4

/* FSA8108_REG_DET_T (0x0a) */
#define FSA8108_TDET_MIC                   (0x0f)
#define FSA8108_TDET_MIC_SHIFT              0
#define FSA8108_TKEY                       (0x0f << 4)
#define FSA8108_TKEY_SHIFT                  4

/* FSA8108_REG_DEBOUNCE_T (0x0b) */
#define FSA8108_KPRS_DEBOUNCE              (0x0f)
#define FSA8108_KPRS_DEBOUNCE_SHIFT         0
#define FSA8108_DEBOUNCE_ESD               (0x0f << 4)
#define FSA8108_DEBOUNCE_ESD_SHIFT          4

/* FSA8108_REG_CON (0x0c) */
#define FSA8108_LDO_OUT                    (0x01)
#define FSA8108_LDO_OUT_SHIFT               0
#define FSA8108_MIC_DET                    (0x01 << 1)
#define FSA8108_MIC_DET_SHIFT               1
#define FSA8108_JACK_DET                   (0x01 << 2)
#define FSA8108_JACK_DET_SHIFT              2
#define FSA8108_MP3_MODE                   (0x01 << 3)
#define FSA8108_MP3_MODE_SHIFT              3
#define FSA8108_LONG_KEY_FUNCTION          (0x01 << 4)
#define FSA8108_LONG_KEY_FUNCTION_SHIFT     4
#define FSA8108_DOUBLE_KEY_FUNCTION        (0x01 << 5)
#define FSA8108_DOUBLE_KEY_FUNCTION_SHIFT   5
#define FSA8108_ALL_KEY_SE_FUNCTION        (0x01 << 6)
#define FSA8108_ALL_KEY_SE_FUNCTION_SHIFT   6
#define FSA8108_STUCK_KEY_FUNCTION         (0x01 << 7)
#define FSA8108_STUCK_KEY_FUNCTION_SHIFT    7

/* FSA8108_REG_COMPARATOR_12 (0x0d) */
#define FSA8108_NO_SE_KEY_CMP              (0x0f)
#define FSA8108_NO_SE_KEY_CMP_SHIFT         0
#define FSA8108_NC_SE_KEY_CMP              (0x0f << 4)
#define FSA8108_NC_SE_KEY_CMP_SHIFT         4

/* FSA8108_REG_COMPARATOR_34 (0x0e) */
#define FSA8108_VOL_UP_CMP                 (0x0f)
#define FSA8108_VOL_UP_CMP_SHIFT            0
#define FSA8108_VOL_DOWN_CMP               (0x0f << 4)
#define FSA8108_VOL_DOWN__CMP_SHIFT         4

/* FSA8108_REG_RESET (0x0f) */
#define FSA8108_GLOBAL_RES                 (0x01)
#define FSA8108_GLOBAL_RES_SHIFT            0
#define FSA8108_JACK_REM_RESET             (0x01 << 1)
#define FSA8108_JACK_REM_RESET_SHIFT        1

/******************************************************************************
* bit definitions
******************************************************************************/
/* FSA8108_REG_DEVICE_ID (0x01) */

/* FSA8108_REG_INT_1 (0x02) */

/* FSA8108_REG_INT_2 (0x03) */

/* FSA8108_REG_INT_MASK_1 (0x04) */

/* FSA8108_REG_INT_MASK_2 (0x05) */

/* FSA8108_REG_GLOBAL_MUL (0x06) */
// Global Multiplier Number
#define GMUL_1_16                    0
#define GMUL_1_8                     1
#define GMUL_1_4                     2
#define GMUL_1_2                     3
#define GMUL_1                       4
#define GMUL_2                       5
#define GMUL_4                       6
#define GMUL_8                       7

/* FSA8108_REG_JDET_T (0x07) */
/* FSA8108_REG_KEY_PRS_T (0x08) */
//FSA8108_TLONG [3:0]
#define TLONG_850       6
#define TLONG_900       7  //default
#define TLONG_1000      8
//FSA8108_TDOUBLE [7:4]
#define TDOUBLE_800     6
#define TDOUBLE_1000    7  //default
#define TDOUBLE_1100    8

/* FSA8108_REG_MP3_MODE_T (0x09) */
/* FSA8108_REG_DET_T (0x0a) */
/* FSA8108_REG_DEBOUNCE_T (0x0b) */

/* FSA8108_REG_CON (0x0c) */
#define CONTROL_ON     0
#define CONTROL_OFF    1

/* FSA8108_REG_COMPARATOR_12 (0x0d) */
//FSA8108_NO_SE_KEY_CMP [3:0]
#define NO_SE_110     0x09
#define NO_SE_120     0x0a  //default
#define NO_SE_130     0x0b
#define NO_SE_170     0x0f
//FSA8108_NC_SE_KEY_CMP [7:4]
#define NC_SE_1000    0x00
#define NC_SE_2200    0x0d
#define NC_SE_2300    0x0e  //default
#define NC_SE_2400    0x0f

/* FSA8108_REG_COMPARATOR_34 (0x0e) */

/* FSA8108_REG_RESET (0x0f) */

/**********************************************/
#define TRUE 1
#define FALSE 0
/*******************key macro area*****************/
#define KEY_HEADSETHOOK                       226 
#define KEY_VOLUME_UP                         115 
#define KEY_VOLUME_DOWN                       114 
#define KEY_EVENT_RELEASE                     0
#define KEY_EVENT_PRESS						  1
/***********************************************/
#define __FSA8108_DEBUG__ 				0
#define EARPHONE_ADC_CAHNNEL 			3
#define MIC_DETECT_NOPRESSED			200 //adc value when no key press
#define WAKE_LOCK_TIME					8*HZ

struct fsa8108_info {
	struct i2c_client		*client;	
	struct mutex		mutex;
	struct input_dev *input;
	struct work_struct  det_work;
	struct work_struct  reset_work;
	struct delayed_work correct_4pole_work;
	unsigned int cur_jack_type;
	struct s3c_adc_client	*adc_client;
	struct delayed_work   adc_read_work;		
	struct wake_lock det_wake_lock;
	unsigned long jack_plug_timeout;
};

static struct switch_dev switch_jack_detection = {
	.name = "h2w",
};

static struct switch_dev switch_sendend = {
	.name = "send_end",
};

struct fsa8108_info *g_fsa8108_info;
struct platform_device *g_plat_dev;


static int fsa8108_write_reg(int reg, int val)
{
    int ret;
	struct i2c_client *this_client = g_fsa8108_info->client;
	
	ret = i2c_smbus_write_byte_data(this_client, reg, val);
		
	if (ret < 0){
		pr_err("%s: error = %d , try again", __func__, ret);
		ret = i2c_smbus_write_byte_data(this_client, reg, val);
		if (ret < 0)
			pr_err("%s: error = %d", __func__, ret);
	}

    return ret;
}

static int fsa8108_read_reg(int reg)
{
    int ret;
	struct i2c_client *this_client = g_fsa8108_info->client;
	
	ret = i2c_smbus_read_byte_data(this_client, reg);
		
	if (ret < 0){
		pr_err("%s: error = %d , try again", __func__, ret);
		ret = i2c_smbus_read_byte_data(this_client, reg);
		if (ret < 0)
			pr_err("%s: error = %d", __func__, ret);
	}
    return ret;
}

static void fsa8108_set_value(u8 reg, u8 reg_bit, u8 reg_shift, u8 val)
{
	u8 tmp;

	tmp = fsa8108_read_reg(reg);
	tmp &= (~reg_bit);
	tmp |= (val << reg_shift);

	fsa8108_write_reg(reg, tmp);
}

#if __FSA8108_DEBUG__
static ssize_t fsa8108_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf);

static ssize_t fsa8108_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count);

#define FSA_ATTR(_name)\
{\
    .attr = { .name = #_name, .mode = S_IRUGO | S_IWUSR},\
    .show = fsa8108_show,\
    .store = fsa8108_store,\
}

static struct device_attribute fsa8108_attrs[] = {
    FSA_ATTR(RegW),
    FSA_ATTR(RegR),
};
enum {
	REG_SETVALUE,
	REG_GETVALUE,
};

static ssize_t fsa8108_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
	int i = 0;
	ptrdiff_t off;
	off = attr - fsa8108_attrs;

	switch(off){
	case REG_SETVALUE:
		i += scnprintf(buf+i, PAGE_SIZE-i, "Error\n");
		break;
	case REG_GETVALUE:
		{
			int reg,value;
			for(reg = FSA8108_REG_DEVICE_ID;reg <= FSA8108_REG_RESET;reg++)
			{
				value = fsa8108_read_reg(reg);
				pr_info("%s: R=0x%.2X D=0x%.2X \n", __func__, reg,value);
			}
		}
		i += scnprintf(buf+i, PAGE_SIZE-i, "0x%.2X 0x%.2X\n",fsa8108_read_reg(2),
			fsa8108_read_reg(3));
		break;
	default:
		i += scnprintf(buf+i, PAGE_SIZE-i, "Error\n");
		break;	
	}
	return i;
}

static ssize_t fsa8108_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int reg,value;
	int ret = 0;
	ptrdiff_t off;

	off = attr - fsa8108_attrs;

	switch(off){
	case REG_SETVALUE:
		if (sscanf(buf, "%X %X \n", &reg,&value) == 2) {	
			fsa8108_write_reg(reg,value);
			pr_info("%s: R=0x%.2X D=0x%.2X \n", __func__, reg,value);
		}
		ret = count;
		break;
	case REG_GETVALUE:
		if (sscanf(buf, "%X\n", &reg) == 1) {	
			
			value = fsa8108_read_reg(reg);
			pr_info("%s: R=0x%.2X D=0x%.2X \n", __func__, reg,value);
			
		}
		ret = count;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;	
}

static int fsa8108_create_attrs(struct device * dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(fsa8108_attrs); i++) {
		rc = device_create_file(dev, &fsa8108_attrs[i]);
		if (rc)
			goto fsa8108_attrs_failed;
	}
	goto succeed;

fsa8108_attrs_failed:
	printk(KERN_INFO "%s(): failed!!!\n", __func__);	
	while (i--)
		device_remove_file(dev, &fsa8108_attrs[i]);
succeed:		
	return rc;

}

static void fsa8108_destroy_atts(struct device * dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(fsa8108_attrs); i++)
		device_remove_file(dev, &fsa8108_attrs[i]);
}
#endif
/*
* mask key interrupts when 3pole plug in
*/
static void fsa8108_mask_int(int onoff)
{
	if(onoff){
		fsa8108_write_reg(FSA8108_REG_INT_MASK_1,0xF8);
		fsa8108_write_reg(FSA8108_REG_INT_MASK_2,0xFF);
	}else{
		fsa8108_write_reg(FSA8108_REG_INT_MASK_1,0xC0);
		fsa8108_write_reg(FSA8108_REG_INT_MASK_2,0xC0);
	}
}

void fsa8108_MP3_mode(int onoff)
{
    if(onoff)
        fsa8108_set_value(FSA8108_REG_CON, FSA8108_MP3_MODE, 
        	FSA8108_MP3_MODE_SHIFT, CONTROL_ON);
	else
		fsa8108_set_value(FSA8108_REG_CON, FSA8108_MP3_MODE, 
			FSA8108_MP3_MODE_SHIFT, CONTROL_OFF);
		
}
EXPORT_SYMBOL(fsa8108_MP3_mode);

void fsa8108_LDO_output(int onoff)
{
    if(onoff)
        fsa8108_set_value(FSA8108_REG_CON, FSA8108_LDO_OUT, 
        	FSA8108_LDO_OUT_SHIFT, CONTROL_ON);
	else
		fsa8108_set_value(FSA8108_REG_CON, FSA8108_LDO_OUT, 
			FSA8108_LDO_OUT_SHIFT, CONTROL_OFF);

}
EXPORT_SYMBOL(fsa8108_LDO_output);

static void fsa8108_report_key_event(int key_type,int updown)
{
	input_report_key(g_fsa8108_info->input, key_type, updown);
	input_sync(g_fsa8108_info->input);
	switch_set_state(&switch_sendend,updown);
	wake_lock_timeout(&g_fsa8108_info->det_wake_lock, WAKE_LOCK_TIME);
}

static void fsa8108_adc_read_func(struct work_struct *work)
{
	struct fsa8108_info *info =
		container_of(work, struct fsa8108_info, adc_read_work.work);	

	if (s3c_adc_read(info->adc_client,  EARPHONE_ADC_CAHNNEL) > MIC_DETECT_NOPRESSED){
		pr_info("%s OKOKOKOKOOOK__LONG RELEASED\n",__func__);		
		
		fsa8108_report_key_event(KEY_HEADSETHOOK, KEY_EVENT_RELEASE);

		pr_info("[fsa8108] key release \n");		
		return;
	}	
	schedule_delayed_work(&info->adc_read_work,msecs_to_jiffies(40));
}

static void fsa8108_4pole_correct_func(struct work_struct *work)
{
	struct fsa8108_info *info =
		container_of(work, struct fsa8108_info, correct_4pole_work.work);	
	int adc_value = s3c_adc_read(info->adc_client,  EARPHONE_ADC_CAHNNEL);

	pr_info("adc value is %d\n",  adc_value);
}
static void process_int(int intr_type,struct fsa8108_info* info)
{
	unsigned char val1, val2;
#if __FSA8108_DEBUG__
	int reg,value;
#endif	

	val1 = intr_type & 0xff;
	val2 = intr_type >> 8;	
	
	pr_info("\nvalue = 0x%.4X,val1 = 0x%x,val2 = 0x%x\n",intr_type, val1, val2);
	
#if __FSA8108_DEBUG__
	for(reg = FSA8108_REG_DEVICE_ID;reg <= FSA8108_REG_RESET;reg++)
	{
		value = fsa8108_read_reg(reg);
		pr_info("%s: R=0x%.2X D=0x%.2X \n", __func__, reg,value);
	}
#endif

	if(val1)
	{
	    switch(val1){
			case FSA8108_3POLE_CONNECT:
				pr_err("%s 3pole connect",__func__);
				info->cur_jack_type = FSA_HEADSET_3POLE;
				fsa8108_mask_int(1);

				fsa8108_LDO_output(0);
				switch_set_state(&switch_jack_detection, FSA_HEADSET_3POLE);
				schedule_delayed_work(&info->correct_4pole_work,2*HZ);
				break;
				
			case FSA8108_4POLE_CONNECT:
				pr_err("%s 4pole connect",__func__);
				info->cur_jack_type = FSA_HEADSET_4POLE;			
				switch_set_state(&switch_jack_detection, FSA_HEADSET_4POLE);				
				fsa8108_mask_int(0);/*recover key interrupts after 4pole connect*/
				info->jack_plug_timeout = jiffies +2*HZ ;
				schedule_delayed_work(&info->correct_4pole_work,2*HZ);
				break;
				
			case FSA8108_PLUG_DISCONNECT:
				pr_err("%s plug disconnect",__func__);
				info->cur_jack_type = FSA_JACK_NO_DEVICE;
				fsa8108_mask_int(1);
				fsa8108_LDO_output(1);
				switch_set_state(&switch_jack_detection, FSA_JACK_NO_DEVICE);				
				break;		
				
			case FSA8108_SEND_END_LONG:  //TBD
				if(time_before(jiffies,info->jack_plug_timeout))
					break;/*deglitch for plug-in triggered key press event*/
				
				pr_info("%s OKOKOKOKOOOK__LONG PRESSED\n",__func__);				
				fsa8108_report_key_event(KEY_HEADSETHOOK,KEY_EVENT_PRESS);
				
				schedule_delayed_work(&info->adc_read_work,msecs_to_jiffies(40));
				break;	
				
			case FSA8108_SEND_END_PRESS:
				if(time_before(jiffies,info->jack_plug_timeout))
					break;
				
				pr_info("%s OKOKOKOKOOOK PRESSED\n",__func__);
				
				fsa8108_report_key_event(KEY_HEADSETHOOK,KEY_EVENT_PRESS);
				msleep(20);
				fsa8108_report_key_event(KEY_HEADSETHOOK,KEY_EVENT_RELEASE);
				
				break;
			case FSA8108_SEND_END_DOUBLE:
				pr_info("%s OKOKOKOKOOOK DOUBLE\n",__func__);
				fsa8108_report_key_event(KEY_HEADSETHOOK,KEY_EVENT_PRESS);
				msleep(20);
				fsa8108_report_key_event(KEY_HEADSETHOOK,KEY_EVENT_RELEASE);
				msleep(200);
				fsa8108_report_key_event(KEY_HEADSETHOOK,KEY_EVENT_PRESS);
				msleep(20);
				fsa8108_report_key_event(KEY_HEADSETHOOK,KEY_EVENT_RELEASE);
				break;
			default:
				break;
	    }
	}
	else if(val2)
	{
	    switch(val2){
			case FSA8108_VOL_UP_LONG_P: 
				if(time_before(jiffies,info->jack_plug_timeout))
					break;
				
				pr_info("%s volumn ++++++LONG_PRESSED\n",__func__);				
				fsa8108_report_key_event(KEY_VOLUME_UP,KEY_EVENT_PRESS);
				break;
				
			case FSA8108_VOL_UP_LONG_R: 
				if(time_before(jiffies,info->jack_plug_timeout))
					break;
				
				pr_info("%s volumn ++++++LONG_RELEASED\n",__func__);
				fsa8108_report_key_event(KEY_VOLUME_UP,KEY_EVENT_RELEASE);
				break;
				
			case FSA8108_VOL_DOWN_LONG_P:
				if(time_before(jiffies,info->jack_plug_timeout))
					break;
				
				pr_info("%s volumn ------LONG_PRESSED\n",__func__);				
				fsa8108_report_key_event(KEY_VOLUME_DOWN,KEY_EVENT_PRESS);
				break;
				
			case FSA8108_VOL_DOWN_LOGN_R: 
				if(time_before(jiffies,info->jack_plug_timeout))
					break;
				
				pr_err("%s volumn ------LONG_RELEASE\n",__func__);
				fsa8108_report_key_event(KEY_VOLUME_DOWN,KEY_EVENT_RELEASE);
				break;	
				
			case FSA8108_VOL_UP:
				if(time_before(jiffies,info->jack_plug_timeout))
					break;
				
				pr_info("%s volumn ++++++ PRESSED\n",__func__);
				fsa8108_report_key_event(KEY_VOLUME_UP,KEY_EVENT_PRESS);
				msleep(20);
				fsa8108_report_key_event(KEY_VOLUME_UP,KEY_EVENT_RELEASE);
				break;
				
			case FSA8108_VOL_DOWN:
				if(time_before(jiffies,info->jack_plug_timeout))
					break;
				
				pr_info("%s volumn ------ PRESSED\n",__func__);
				fsa8108_report_key_event(KEY_VOLUME_DOWN,KEY_EVENT_PRESS);
				msleep(20);
				fsa8108_report_key_event(KEY_VOLUME_DOWN,KEY_EVENT_RELEASE);
				break;
				
			default:
				break;
	    }	
	}
	else
	{
	    pr_err("%s : [fsa8108]interrupt status: intr_type = 0x%.4X \n", __func__,intr_type);
	}

}

static irqreturn_t fsa8108_irq_thread(int irq, void *handle)
{
	struct fsa8108_info *info = (struct fsa8108_info *)handle;
	
	pr_info("[fsa8108] irq triggered\n");
	schedule_work(&info->det_work);

	return IRQ_HANDLED;
}

static void fsa8081_jack_det_work_func(struct work_struct *work)
{
	int intr_type;	
	struct fsa8108_info *info =
		container_of(work, struct fsa8108_info, det_work);
	struct i2c_client *client = info->client;	

	mutex_lock(&info->mutex);
	intr_type = i2c_smbus_read_word_data(client, FSA8108_REG_INT_1);	
	process_int(intr_type,info);
	mutex_unlock(&info->mutex);
}
static void fsa8108_reset_work(struct work_struct *work)
{	
	fsa8108_write_reg(FSA8108_REG_RESET,1);
	msleep(100);
	fsa8108_write_reg(FSA8108_REG_RESET,0);
	
	fsa8108_set_value(FSA8108_REG_GLOBAL_MUL,0x07,0,0x03);
	fsa8108_set_value(FSA8108_REG_KEY_PRS_T,FSA8108_TDOUBLE,FSA8108_TDOUBLE_SHIFT,0x00);
	fsa8108_set_value(FSA8108_REG_JDET_T,FSA8108_TDET_IN,FSA8108_TDET_IN_SHIFT,0x0F);
	/*** Set Timing parameters shorten long press time***/		
	fsa8108_write_reg(FSA8108_REG_KEY_PRS_T,0);	
	/*span insert time*/
	//fsa8108_write_reg(FSA8108_REG_JDET_T,0xF2);
	fsa8108_mask_int(1);/*mask key interrupts before plug in*/
	process_int(i2c_smbus_read_word_data(g_fsa8108_info->client, FSA8108_REG_INT_1),g_fsa8108_info);
}

static int fsa8108_i2c_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{

	int i,ret = 0;
	struct fsa8108_info *info;

	int fsa8108_jack_keycode[] = {KEY_HEADSETHOOK, KEY_VOLUME_UP, KEY_VOLUME_DOWN};
	int device_id = -1;	

	pr_info("[fsa8108]earphone detection fsa8108_probe begin\n");	
	
	info = kzalloc(sizeof(struct fsa8108_info), GFP_KERNEL);
	if(info == NULL){
		pr_info("%s  allocate memory for info failed\n",__func__);
	}
	
	g_fsa8108_info = info;
	info->client = client;	
	i2c_set_clientdata(client, info);
	
	mutex_init(&info->mutex);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto check_funcionality_failed;
	}	
	device_id = fsa8108_read_reg(FSA8108_REG_DEVICE_ID);
	pr_info("%s: FSADEVICE_ID =%d \n",__func__,device_id);
	if(device_id < 0){
		pr_err("%s: i2c check device error\n", __func__);
		ret = -ENODEV;
		goto check_device_failed;
	}	
	
	info->input = input_allocate_device();
	if (info->input == NULL) {
		ret = -ENOMEM;
		pr_err("%s : Failed to allocate input device.\n", __func__);
		goto err_request_input_dev;
	}

	info->input->name = "mx-earphone-keypad";

	wake_lock_init(&info->det_wake_lock, WAKE_LOCK_SUSPEND, "mx2_jack_det");
	
	for (i = 0 ; i < sizeof(fsa8108_jack_keycode)/sizeof(int); i++)
		input_set_capability(info->input, EV_KEY, fsa8108_jack_keycode[i]);
	
	ret = input_register_device(info->input);
	if (ret) {
		pr_err("%s : Failed to register driver\n", __func__);
		goto err_register_input_dev;
	}

	ret = switch_dev_register(&switch_jack_detection);
	if (ret < 0) {
		pr_err("%s : Failed to register switch device\n", __func__);
		goto err_switch_dev_register;
	}

	ret = switch_dev_register(&switch_sendend);
	if (ret < 0) {
		pr_err("%s : Failed to register switch device\n", __func__);
		goto err_switch_dev_register;
	}

	INIT_WORK(&info->det_work, fsa8081_jack_det_work_func);
	INIT_WORK(&info->reset_work, fsa8108_reset_work);
	INIT_DELAYED_WORK(&info->adc_read_work, fsa8108_adc_read_func);
	INIT_DELAYED_WORK(&info->correct_4pole_work, fsa8108_4pole_correct_func);
	// Reset
	//fsa8108_reset();
	schedule_work(&info->reset_work);	

	info->adc_client = s3c_adc_register(g_plat_dev, NULL, NULL, 0);
	if (IS_ERR(info->adc_client)) {
		pr_err("[fsa8108] cannot register adc\n");
		ret = PTR_ERR(info->adc_client);
		goto err_switch_dev_register;
	}
	
	client->irq = gpio_to_irq(client->irq);
	if (client->irq < 0){
		pr_err("%s : Failed gpio_to_irq\n", __func__);
		goto err_switch_dev_register;
	}
	if (client->irq) {
	    ret = request_threaded_irq(client->irq, NULL,
		    fsa8108_irq_thread, IRQF_TRIGGER_FALLING,
		    "fsa8108 jack interrupt", info);
	    if (ret) {
		    dev_err(&client->dev, "failed to reqeust IRQ\n");
		    return ret;
	    }
    }	
	
	ret = enable_irq_wake(client->irq);
	if (ret < 0)
		dev_err(&client->dev,
			"failed to enable wakeup src %d\n", ret);
	
#if __FSA8108_DEBUG__
	fsa8108_create_attrs(&client->dev);
#endif	
	return ret;

err_switch_dev_register:
	input_unregister_device(info->input);	
err_register_input_dev:
	input_free_device(info->input);

err_request_input_dev:
check_funcionality_failed:
check_device_failed:
	i2c_set_clientdata(client, NULL);
	mutex_destroy(&info->mutex);
    kfree(info);
	
	return ret;	

}
static int fsa8108_i2c_remove(struct i2c_client *client)
{
    struct fsa8108_info *info = i2c_get_clientdata(client);
#if __FSA8108_DEBUG__
	fsa8108_destroy_atts(&client->dev);	
#endif
	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, info);
	}

	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);	
	wake_lock_destroy(&info->det_wake_lock);
	kfree(info);
	
	return 0;
}

static const struct i2c_device_id fsa8108_i2c_id[] = {
	{ "fairchild_fsa8108", 0 },
	{ }
};
 
static struct i2c_driver fsa8108_i2c_driver = {
	.driver = {
		.name = "fairchild_fsa8108",
		.owner = THIS_MODULE,			
	},
	.probe    = fsa8108_i2c_probe,
	.remove   = __devexit_p(fsa8108_i2c_remove),
	.suspend  = NULL,
	.resume	  = NULL,
	.id_table = fsa8108_i2c_id,
};

static  int fsa8108_i2c_init(void)
{

	return i2c_add_driver(&fsa8108_i2c_driver);
}

static  void fsa8108_i2c_exit(void)
{
	i2c_del_driver(&fsa8108_i2c_driver);
}

static int __devinit fsa8108_probe(struct platform_device *pdev)
{
	int ret;

	pr_info("[fsa8108] platform_device probe\n");
	g_plat_dev = pdev;
	
	ret = fsa8108_i2c_init();
	
	return ret;
}

static int __devexit fsa8108_remove(struct platform_device *pdev)
{
	fsa8108_i2c_exit();	
	return 0;
}

static struct platform_driver fsa8108_driver={
	.probe = fsa8108_probe,
	.remove = fsa8108_remove,	
	.driver		= {
		.name	= "fsa8108_adc",
		.owner	= THIS_MODULE,
	},
};

static int __init fsa8108_init(void)
{
	int ret;
	
	ret = platform_driver_register(&fsa8108_driver);
	if (ret)
		return -ENOMEM;
	
	return ret;
}

static void __exit fsa8108_exit(void)
{
	platform_driver_unregister(&fsa8108_driver);
}

module_init(fsa8108_init);
module_exit(fsa8108_exit);

//module_init(fsa8108_i2c_init);
//module_exit(fsa8108_i2c_exit);
