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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <linux/i2c-gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-m040.h>
#include <asm/mach-types.h>
#include <linux/firmware.h>
#include	<linux/mx_qm.h>

#define QM_MX_FW 		"qm/qm_mx_led.bin"

#define 	RESET_COLD	1
#define	RESET_SOFT	0

//#define	VERIFY_CRC

static volatile int force_update = false;
static volatile int is_update = false;
static int key_wakeup_type = 0;

struct mx_qm_reg_data {
	unsigned char addr;
	unsigned char value;
};

//initial registers
const struct mx_qm_reg_data init_regs[] = {
	{QM_REG_STATUS, QM_STATE_IDLE},
	{LED_REG_PWM, 0xFF},
	{LED_REG_CUR0, 0x02},
//	{LED_REG_CUR1, 0x00},
//	{LED_REG_CUR2, 0x00},
//	{LED_REG_CUR3, 0x00},
//	{LED_REG_CUR4, 0x00},		
	{QM_REG_MASK_POS, 0x09},
	{QM_REG_QM_DET_TH, 0x0A},
	{QM_REG_QM_MASK_TH, 0x06},	
//	{QM_REG_WAKEUP_TYPE, 0x02},// µ¥»÷
//	{QM_REG_QM_BURST, 0x40},
	{},
};

	
static struct mfd_cell mx_qm_devs[] = {
	{ .name = "mx-qm-touch", .id = 0 },
	{ .name = "mx-qm-led", .id = 1 },
//	{ .name = "mx-qm-led", .id = 2 },
//	{ .name = "mx-qm-led", .id = 3 },
//	{ .name = "mx-qm-led", .id = 4 },
//	{ .name = "mx-qm-led", .id = 5 },
//	{ .name = "mx-qm-led", .id = 6 },
};

static int mx_qm_getimgfwversion(struct mx_qm_data *mx);
static int mx_qm_update(struct mx_qm_data *mx);
static void mx_qm_reset(struct mx_qm_data *mx,int m);
static void mx_qm_wakeup(struct mx_qm_data *mx,int bEn);

static int mx_qm_read(struct i2c_client *client,
				  int bytes, void *dest)
{
	struct i2c_msg xfer;
	int ret;

	xfer.addr =  client->addr;
	xfer.flags = I2C_M_RD;
	xfer.len = bytes;
	xfer.buf = (char *)dest;

	ret = i2c_transfer(client->adapter, &xfer, 1);

	if (ret < 0)
		goto err_read;
	if (ret != 1) {
		ret = -EIO;
		goto err_read;
	}

	return 0;

err_read:
	return ret;
}

static int mx_qm_write(struct i2c_client *client,
				int bytes, const void *src)
{
	struct i2c_msg xfer;
	int ret;

	xfer.addr = client->addr;
	xfer.flags = 0;
	xfer.len = bytes;
	xfer.buf = (char *)src;

	ret = i2c_transfer(client->adapter, &xfer, 1);
	if (ret < 0)
		goto err_write;
	if (ret != 1) {
		ret = -EIO;
		goto err_write;
	}

	return 0;

err_write:
	return ret;
}

static int mx_qm_readbyte(struct i2c_client *client, u8 reg)
{
	struct mx_qm_data *mx = i2c_get_clientdata(client);
	int ret = 0;

	if( is_update )
		return -EBUSY;
	
	mutex_lock(&mx->iolock);

	ret = mx_qm_write(client,1,&reg);
	if (ret < 0)
		dev_err(&client->dev,
			"can not write register, returned %d at line %d\n", ret,__LINE__);
	
	ret = mx_qm_read(client, 1,&reg);
	if (ret < 0)
		dev_err(&client->dev,
			"can not read register, returned %d at line %d\n", ret,__LINE__);
	else
		ret = reg;

	mutex_unlock(&mx->iolock);

	return ret;
}

static int mx_qm_writebyte(struct i2c_client *client, u8 reg, u8 data)
{
	struct mx_qm_data *mx = i2c_get_clientdata(client);
	int ret = 0;	
	unsigned char buf[2];

	if( is_update )
		return -EBUSY;

	mutex_lock(&mx->iolock);

	buf[0] = reg;
	buf[1] = data;
	ret = mx_qm_write(client,2,buf);
	if (ret < 0)
		dev_err(&client->dev,
			"can not write register, returned %d at line %d\n", ret,__LINE__);

	mutex_unlock(&mx->iolock);

	return ret;
}

#if 0
static int mx_qm_readword(struct i2c_client *client, u8 reg)
{
	struct mx_qm_data *mx = i2c_get_clientdata(client);
	int ret;	
	u16 val;

	if( is_update )
		return -EBUSY;

	mutex_lock(&mx->iolock);

	ret = mx_qm_write(client,1,&reg);
	if (ret < 0)
		dev_err(&client->dev,
			"can not read register, returned %d at line %d\n", ret,__LINE__);
	msleep(10);
	ret = mx_qm_read(client, 2,&val);
	if (ret < 0)
		dev_err(&client->dev,
			"can not read register, returned %d\n", ret);
	else
		ret = val;


	mutex_unlock(&mx->iolock);

	return ret;
}

static int mx_qm_writeword(struct i2c_client *client, u8 reg, u16 data)
{
	struct mx_qm_data *mx = i2c_get_clientdata(client);
	int ret;	
	unsigned char buf[3];

	if( is_update )
		return -EBUSY;

	mutex_lock(&mx->iolock);

	buf[0] = reg;
	buf[1] = data & 0xFF;
	buf[2] = (data >>8 ) & 0xFF;
	ret = mx_qm_write(client,3,buf);
	if (ret < 0)
		dev_err(&client->dev,
			"can not write register, returned %d\n", ret);


	mutex_unlock(&mx->iolock);

	return ret;
}
#endif

static int mx_qm_readdata(struct i2c_client *client, u8 reg,int bytes,void *dest)
{
	struct mx_qm_data *mx = i2c_get_clientdata(client);
	int ret;	

	if( is_update )
		return -EBUSY;

	mutex_lock(&mx->iolock);

	ret = mx_qm_write(client,1,&reg);
	if (ret < 0)
		dev_err(&client->dev,
			"can not read register, returned %d at line %d\n", ret,__LINE__);
	msleep(10);
	ret = mx_qm_read(client, bytes,dest);
	if (ret < 0)
		dev_err(&client->dev,
			"can not read register, returned %d\n", ret);

	mutex_unlock(&mx->iolock);

	return ret;
}

static int mx_qm_writedata(struct i2c_client *client, u8 reg, int bytes, const void *src)
{
	struct mx_qm_data *mx = i2c_get_clientdata(client);
	int ret;	
	unsigned char buf[256];
	int size;

	if( is_update )
		return -EBUSY;

	mutex_lock(&mx->iolock);
	if(bytes > 255 )
		bytes = 255;

	buf[0] = reg;
	memcpy(&buf[1],src,bytes);

	size = bytes + 1;
	ret = mx_qm_write(client,size,buf);
	if (ret < 0)
		dev_err(&client->dev,
			"can not write register, returned %d\n", ret);


	mutex_unlock(&mx->iolock);

	return ret;
}

static void __devinit mx_qm_init_registers(struct mx_qm_data *mx)
{
	int i, ret;
	unsigned char buf[2];

	for (i=0; i<ARRAY_SIZE(init_regs); i++) {		
		buf[0] = init_regs[i].addr;
		buf[1] = init_regs[i].value;
		ret = mx_qm_write(mx->client,2,buf);
		if (ret) {
			dev_err(mx->dev, "failed to init reg[%d], ret = %d\n", i, ret);
		}
	}
}

static bool __devinit mx_qm_identify(struct mx_qm_data *mx)
{
	int id, ver,img_ver;
	struct i2c_client *client = mx->client;

	/* Read Chip ID */
	id = mx_qm_readbyte(client, QM_REG_DEVICE_ID);
	if (id != MX_QM_DEVICE_ID) {
		dev_err(&client->dev, "ID %d not supported\n", id);
		goto upd_ext;
	}

	/* Read firmware version */
	ver = mx_qm_readbyte(client, QM_REG_VERSION);
	if (ver < 0) {
		dev_err(&client->dev, "could not read the firmware version\n");
		goto upd_ext;
	}

	img_ver = mx_qm_getimgfwversion(mx);
	if( img_ver == 0xFF )
		img_ver = FW_VERSION;
		
	if( ver < img_ver)
	{
		dev_info(&client->dev, "Old firmware version %d.%d , img ver %d.%d,need be update\n", ((ver>>4)&0x0F),(ver&0x0F), ((img_ver>>4)&0x0F),(img_ver&0x0F));
upd_ext:		
		mx_qm_update(mx);
		mx_qm_wakeup(mx,true);
		
		/* Read Chip ID */
		id = mx_qm_readbyte(client, QM_REG_DEVICE_ID);
		if (id != MX_QM_DEVICE_ID) {
			dev_err(&client->dev, "ID %d not supported\n", id);
			return false;
		}
		
		/* Read firmware version again*/
		ver = mx_qm_readbyte(client, QM_REG_VERSION);
		if (ver < 0) {
			dev_err(&client->dev, "could not read the firmware version\n");
			return false;
		}
	}

	mx->AVer = ver;

	/* Read led ID */
	mx->LedVer = mx_qm_readbyte(client, QM_REG_LEDVERSION);

	dev_info(&client->dev, "MX_QM id 0x%.2X firmware version %d.%d(%c) \n", id,((ver>>4)&0x0F),(ver&0x0F),mx->LedVer);

	return true;
}

/*This function forces a recalibration of qm touch sensor.*/
static void mx_qm_recalibration(struct mx_qm_data *mx)
{	
	unsigned char msg[2];
	msg[0] = QM_REG_CALIBRATE;
	msg[1] = 1;			
	mx_qm_write(mx->client,2,msg);
	
	dev_dbg(&mx->client->dev, "force a recalibration\n");
}

static void mx_qm_reset(struct mx_qm_data *mx,int m)
{
	if(m)
	{	
		s3c_gpio_setpull(mx->gpio_reset, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(mx->gpio_reset, S3C_GPIO_OUTPUT);
		gpio_set_value(mx->gpio_reset,  0);
		msleep(100);
		gpio_set_value(mx->gpio_reset,  1);
		s3c_gpio_cfgpin(mx->gpio_reset, S3C_GPIO_INPUT);		
	}
	else
	{
		unsigned char msg[2];
		msg[0] = QM_REG_SOFTRESET;
		msg[1] = 1;			
		mx_qm_write(mx->client,2,msg);
	}
	key_wakeup_type = 0;
	dev_info(&mx->client->dev, "%s reset. \n", m?"cold":"soft");
}


static void mx_qm_wakeup(struct mx_qm_data *mx,int bEn)
{
	if( bEn )
	{
		s3c_gpio_setpull(mx->gpio_wake, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(mx->gpio_wake, S3C_GPIO_OUTPUT);
		gpio_set_value(mx->gpio_wake,  0);
	}
	else
	{	
		s3c_gpio_setpull(mx->gpio_wake, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(mx->gpio_wake, S3C_GPIO_INPUT);
	}	

	msleep(50);

	dev_dbg(&mx->client->dev, "mx_qm_wakeup %s\n",bEn?"enable":"disable");
}

static int mx_qm_getimgfwversion(struct mx_qm_data *mx)
{
	int err = 0;
	const struct firmware *fw;
	const char *fw_name;
	int img_fwversion;
	
	fw_name = QM_MX_FW;

	err = request_firmware(&fw, fw_name,  &mx->client->dev);
	if (err) {
		printk(KERN_ERR "Failed to load firmware \"%s\"\n", fw_name);
		return err;
	}
	img_fwversion = fw->data[FLASH_ADDR_FW_VERSION];
	release_firmware(fw);
	
	return img_fwversion;
}

#define TWI_CMD_BOOT      'D'
#define TWI_CMD_UPD  	 'U'
#define TWI_CMD_END  	 'E'
#define TWI_CMD_CRC	'C'//get the flash crc
#define TWI_CMD_BVER	'B'//get the bootloader revision
#define	PAGESIZE		64
static int mx_qm_update(struct mx_qm_data *mx)
{
	const struct firmware *fw;
	const char *fw_name;

	int ret = 0;

	unsigned char cmd;
	int i,cnt,size,try_cnt;
	unsigned char buf[PAGESIZE+3];
	const unsigned char * ptr;
	unsigned short crc1;
	unsigned short crc2;
	char boot_ver;

	wake_lock(&mx->wake_lock);
		
	is_update = true;
	
	disable_irq(mx->irq);	
	msleep(100);

	mx_qm_reset(mx,RESET_COLD);
	mx_qm_wakeup(mx,true);
	
	msleep(50);

	cmd = TWI_CMD_BOOT;
	ret = mx_qm_write(mx->client,1,&cmd);
	if(ret < 0 )
	{
		dev_err(&mx->client->dev,"can not write register, returned %d at line %d\n", ret,__LINE__);
		goto err_exit10;

	}
	dev_info(&mx->client->dev,"mx qmatrix sensor updating ... \n");
	msleep(10);

	cmd = TWI_CMD_BVER; //
	ret = 0;
	
	ret = mx_qm_write(mx->client,1,&cmd);
	if (ret < 0)
		dev_err(&mx->client->dev,
			"can not write register, returned %d at line %d\n", ret,__LINE__);
	
	ret = mx_qm_read(mx->client, 1,&boot_ver);
	if (ret < 0)
		dev_err(&mx->client->dev,
			"can not read register, returned %d at line %d\n", ret,__LINE__);
	
	if(ret < 0 )
	{
		dev_err(&mx->client->dev,"can not read the bootloader revision, returned %d at line %d\n", ret,__LINE__);
		goto err_exit10;
	}
	
	mx->BVer = boot_ver;
	
	dev_info(&mx->client->dev,"bootloader revision %d.%d\n", ((boot_ver>>4)&0x0F),(boot_ver&0x0F));
	if( boot_ver > 3 )	{
		fw_name = QM_MX_FW;	
	}
	else		{
		dev_err(&mx->client->dev,"This version is not support \n");
		ret = -EINVAL;
		goto err_exit10;
	}
	
	ret = request_firmware(&fw, fw_name,  &mx->client->dev);
	if (ret) {
		dev_err(&mx->client->dev,"Failed to load firmware \"%s\"\n", fw_name);
		goto err_exit;
	}

	try_cnt = 3;
	size = fw->size;	// 1024*6
	ptr = fw->data;
	dev_info(&mx->client->dev,"load firmware %s\n",fw_name);
	
	do
	{
		buf[0] = TWI_CMD_UPD;
		for(i=0;i<size;i+=PAGESIZE)
		{
			buf[1] = i & 0xFF;
			buf[2] = (i>>8) & 0xFF;

			memcpy(&buf[3],ptr+i,PAGESIZE);

			cnt = 3;

			do
			{
				ret = mx_qm_write(mx->client,sizeof(buf),buf);
			}while(ret < 0 && cnt--);
			
			if(ret < 0 )
				dev_err(&mx->client->dev,"can not write register, returned %d at addres 0x%.4X(page %d)\n", ret,i,(i/PAGESIZE+1));
		}	
		
		if(ret < 0 )
			goto err_exit;

		cmd = TWI_CMD_CRC;
		ret = mx_qm_write(mx->client,1,&cmd);
		msleep(100);
		ret = mx_qm_read(mx->client,1,&crc1);
		crc1 <<= 8;
		ret = mx_qm_read(mx->client,1,&crc1);
		crc2  = *(unsigned short *)(fw->data+fw->size-2);
		if( crc1 != crc2)
		{
			dev_err(&mx->client->dev,"crc check 0x%.4X (0x%.4X) failed !!!\n", crc1,crc2);
		}
		else
		{
			dev_info(&mx->client->dev,"Verifying flash OK!\n");
			break;
		}		
	}while(try_cnt--);
	
	cmd = TWI_CMD_END;
	ret = mx_qm_write(mx->client,1,&cmd);
	if(ret < 0 )
	{
		dev_err(&mx->client->dev,"can not write register, returned %d at line %d\n", ret,__LINE__);
		goto err_exit;

	}
	
	dev_info(&mx->client->dev, "Update completed. \n");
	release_firmware(fw);
	goto exit;
	
err_exit:	
	release_firmware(fw);
err_exit10:	
	dev_info(&mx->client->dev, "Update failed !! \n");
	
exit:
	mx_qm_wakeup(mx,false);
	mx_qm_reset(mx,RESET_COLD); 
	msleep(250);	
	mx_qm_wakeup(mx,true);

	is_update = false;

	/*initial registers*/
	if(!ret)
		mx_qm_init_registers(mx);
	
	enable_irq(mx->irq);
	
	wake_unlock(&mx->wake_lock);
	return ret;
}

static ssize_t qm_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf);
static ssize_t qm_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count);

#define QM_ATTR(_name)\
{\
    .attr = { .name = #_name, .mode = S_IRUGO | S_IWUSR},\
    .show = qm_show_property,\
    .store = qm_store,\
}

static struct device_attribute qm_attrs[] = {
    QM_ATTR(status),
    QM_ATTR(position),
    QM_ATTR(cmd),
    QM_ATTR(reset),
    QM_ATTR(calibrate),
    QM_ATTR(version),
    QM_ATTR(led),
    QM_ATTR(update),
    QM_ATTR(key_wakeup_count),
    QM_ATTR(key_wakeup_type),
    QM_ATTR(irq),
};
enum {
	QM_STATUS,
	QM_POS,
	QM_CMD,
	QM_RESET,
	QM_CAL,
	QM_FWR_VER,
	QM_LED,
	QM_UPD,
	QM_CNT,
	QM_WAKEUP_TYPE,
	QM_IRQ,
};
static ssize_t qm_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
	int i = 0;
	ptrdiff_t off;
	struct mx_qm_data *qm = (struct mx_qm_data*)dev_get_drvdata(dev);
	
	if(!qm)
	{
		pr_err("%s(): failed!!!\n", __func__);
		return -ENODEV;
	}

	off = attr - qm_attrs;

	switch(off){
	case QM_POS:
		{
			u16 info[13]; 
			memset(info,0,sizeof(info));
			mx_qm_readdata(qm->client, QM_REG_DBGINFO,sizeof(info),info);
			i += scnprintf(buf+i, PAGE_SIZE-i, 
				"ref = %.3d  %.3d  %.3d  %.3d  \nsig = %.3d  %.3d  %.3d  %.3d  \ndel = %.3d  %.3d  %.3d  %.3d \npos = %d \ndect = %d\n",
				info[0],info[1],info[2],info[3],
				info[4],info[5],info[6],info[7],
				 (s16)info[8],(s16)info[9],(s16)info[10],(s16)info[11],
				 (info[12] & 0xFF),((info[12] >>8)& 0xFF));
		}
		break;
	case QM_STATUS:
		//i += scnprintf(buf+i, PAGE_SIZE-i, "%d\n",mx_qm_readbyte(qm->client,QM_REG_STATUS));
		break;
	case QM_CMD:
		{
			int ret,val;

			val = 0x00;
			ret = mx_qm_read(qm->client,1,&val);
			if (ret < 0)
				pr_err("mx_qm_readbyte error at %d line\n", __LINE__);
			i += scnprintf(buf+i, PAGE_SIZE-i, "0x%.2X\n",val);
		}
		break;
	case QM_FWR_VER:
		i += scnprintf(buf+i, PAGE_SIZE-i, "ID = %c ,Ver.0x%X,LED Ver.%c\n",mx_qm_readbyte(qm->client,QM_REG_DEVICE_ID),
			mx_qm_readbyte(qm->client,QM_REG_VERSION),qm->LedVer);
		break;
	case QM_RESET:
		i += scnprintf(buf+i, PAGE_SIZE-i, "\n");
		break;
	case QM_CAL:
		mx_qm_recalibration(qm);
		i += scnprintf(buf+i, PAGE_SIZE-i, "Sensor recalibration. \n");
		break;
	case QM_LED:
		i += scnprintf(buf+i, PAGE_SIZE-i, "\n");
		break;
	case QM_UPD:
		mx_qm_update(qm);
		i += scnprintf(buf+i, PAGE_SIZE-i, "\n");
		break;
	case QM_CNT:
		i += scnprintf(buf+i, PAGE_SIZE-i, "%d\n",mx_qm_readbyte(qm->client,QM_REG_WAKEUP_CNT));
		break;
	case QM_WAKEUP_TYPE:
		i += scnprintf(buf+i, PAGE_SIZE-i, "%d\n",key_wakeup_type);
		break;
	case QM_IRQ:
		i += scnprintf(buf+i, PAGE_SIZE-i, "Error\n");
		break;
	default:
		i += scnprintf(buf+i, PAGE_SIZE-i, "Error\n");
		break;	
	}
	return i;
}

static ssize_t qm_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int reg,value;
	int ret = 0;
	ptrdiff_t off;
	struct mx_qm_data *qm = (struct mx_qm_data*)dev_get_drvdata(dev);
	
	if(!qm)
	{
		pr_err("%s(): failed!!!\n", __func__);
		return -ENODEV;
	}

	off = attr - qm_attrs;

	switch(off){
	case QM_POS:
		if (sscanf(buf, "%d\n", &value) == 1) {	
			
		}
		ret = count;
		break;
	case QM_STATUS:
		if (sscanf(buf, "%d\n", &value) == 1) {	
			ret = mx_qm_writebyte(qm->client,QM_REG_STATUS,value);
			if (ret < 0)
				pr_err("mx_qm_writebyte error at %d line\n", __LINE__);			
		}
		ret = count;
		break;
	case QM_CMD:
		if (sscanf(buf, "%x %x", &reg, &value) == 2) {
			int ret;
			unsigned char msg[2];
			msg[0] = reg;
			msg[1] = value;			
			dev_info(dev, "R:0x%.2X V:0x%.2X \n", reg,value);
			ret = mx_qm_write(qm->client,2,msg);
			if (ret < 0)
				pr_err("mx_qm_writebyte error at %d line\n", __LINE__);
		}
		else if (sscanf(buf, "%x\n", &value) == 1) {	
			int ret;
			unsigned char msg[2];
			msg[0] = value;			
			dev_info(dev, "V:0x%.2X \n", value);
			ret = mx_qm_write(qm->client,1,msg);
			if (ret < 0)
				pr_err("mx_qm_writebyte error at %d line\n", __LINE__);
			
		}
		else {			
			pr_err("%s(): failed !!!\n", __func__);
		}
		ret = count;
		break;
	case QM_RESET:
		if (sscanf(buf, "%x\n", &value) == 1) {	

			mx_qm_reset(qm,value);
			pr_info("Sensor %s reset. \n", value?"cold":"soft");
		}
		ret = count;
		break;
	case QM_CAL:
		mx_qm_recalibration(qm);
		pr_info("Sensor recalibration. \n");
		ret = count;
		break;
	case QM_FWR_VER:
		ret = count;
		break;
	case QM_LED:
		ret = count;
		if (sscanf(buf, "%x %x", &reg, &value) == 2) {
			reg = reg+LED_REG_CUR0;
			
			if(reg <= LED_REG_CUR4 && value < 0xFF)
			{				
				ret = mx_qm_writebyte(qm->client,reg,value);
				if (ret < 0)
				{
					pr_err("mx_qm_writebyte error at %d line\n", __LINE__);
					ret = -EINVAL;
				}
				else
				{
					ret = count;
				}
				
			}
			else
			{
				ret = -EINVAL;
			}
		}
		break;
	case QM_UPD:
		mx_qm_update(qm);
		ret = count;
		break;
	case QM_CNT:
		ret = count;
		break;
	case QM_WAKEUP_TYPE:
		if (sscanf(buf, "%d\n", &value) == 1) {	
			int ret;
			dev_info(dev, "V:0x%.2X \n", value);
			if( value < 3 )
			{
				ret = mx_qm_writebyte(qm->client,QM_REG_WAKEUP_TYPE,value);
				if (ret < 0)
					pr_err("mx_qm_writebyte error at %d line\n", __LINE__);
				else
					key_wakeup_type = value;
			}		
			else
			{
				pr_err("Invalid argument\n");
			}
		}
		else {			
			pr_err("%s(): failed !!!\n", __func__);
		}
		ret = count;
		break;
	case QM_IRQ:
		if (sscanf(buf, "%x\n", &value) == 1) {	
			if( value )
				enable_irq(qm->irq);
			else
				disable_irq(qm->irq);
			pr_info("%s irq. \n", value?"enable":"disable");
		}
		ret = count;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;	
}

static int qm_create_attrs(struct device * dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(qm_attrs); i++) {
		rc = device_create_file(dev, &qm_attrs[i]);
		if (rc)
			goto qm_attrs_failed;
	}
	goto succeed;

qm_attrs_failed:
	printk(KERN_INFO "%s(): failed!!!\n", __func__);	
	while (i--)
		device_remove_file(dev, &qm_attrs[i]);
succeed:		
	return rc;

}

static void qm_destroy_atts(struct device * dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(qm_attrs); i++)
		device_remove_file(dev, &qm_attrs[i]);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
 static void mx_qm_early_suspend(struct early_suspend *h)
 {
	 struct mx_qm_data *mx =
			 container_of(h, struct mx_qm_data, early_suspend);

	//mx_qm_reset(mx,0);// Soft reset sensor
	mx_qm_recalibration(mx);// ReCalibration sensor
 }
 
 static void mx_qm_late_resume(struct early_suspend *h)
 {
	 struct mx_qm_data *mx =
			 container_of(h, struct mx_qm_data, early_suspend);
	 
	mx->wakeup(mx,true);	
 }
#endif

static int __devinit mx_qm_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct mx_qm_platform_data *pdata = client->dev.platform_data;
	struct mx_qm_data *data;
	int err = 0;
	pr_debug("%s:++\n",__func__);
	
	s3c_gpio_setpull(pdata->gpio_irq, S3C_GPIO_PULL_DOWN);
	s3c_gpio_cfgpin(pdata->gpio_irq, S3C_GPIO_INPUT);
	
	s3c_gpio_setpull(pdata->gpio_wake, S3C_GPIO_PULL_UP);
	s3c_gpio_cfgpin(pdata->gpio_wake, S3C_GPIO_INPUT);
	
	s3c_gpio_setpull(pdata->gpio_reset, S3C_GPIO_PULL_UP);
	s3c_gpio_cfgpin(pdata->gpio_reset, S3C_GPIO_OUTPUT);
	gpio_set_value(pdata->gpio_reset,  0);
	msleep(50);
	gpio_set_value(pdata->gpio_reset,  1);
	s3c_gpio_cfgpin(pdata->gpio_reset, S3C_GPIO_INPUT);

	msleep(250);	

	data = kzalloc(sizeof(struct mx_qm_data), GFP_KERNEL);
	data->dev = &client->dev;
	
	data->gpio_wake = pdata->gpio_wake;
	data->gpio_reset = pdata->gpio_reset;
	data->gpio_irq = pdata->gpio_irq;

	data->client = client;
	data->irq = __gpio_to_irq(data->gpio_irq);//client->irq;//
	data->i2c_readbyte = mx_qm_readbyte;
	data->i2c_writebyte= mx_qm_writebyte;
	data->i2c_readbuf = mx_qm_readdata;
	data->i2c_writebuf = mx_qm_writedata;
	data->reset = mx_qm_reset;
	data->wakeup= mx_qm_wakeup;
	data->update= mx_qm_update;	
		
	i2c_set_clientdata(client, data);
	mutex_init(&data->iolock);
	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "qm_pad");

	if( force_update )
	{
		pr_info("mx_qm:force update...\n");
		mx_qm_update(data);
	}
	
	mx_qm_wakeup(data,true);
	/* Identify the mx_qm chip */
	if (!mx_qm_identify(data))
	{
		struct device *parent = client->adapter->dev.parent;
		struct i2c_gpio_platform_data *pi2cdata;

#ifdef CONFIG_MACH_M041
		if(machine_is_m041()){
			err = -ENODEV;	
			goto err_free_mem;	
		}
#endif
		pi2cdata = parent->platform_data;

		gpio_free(pi2cdata->sda_pin);
		gpio_free(pi2cdata->scl_pin);
		
		//#define  M040_SCL_TOUCHPAD  EXYNOS4_GPY2(0)
		//#define  M040_SDA_TOUCHPAD  EXYNOS4_GPY1(3)

		pi2cdata->scl_pin = EXYNOS4_GPY2(0);
		pi2cdata->sda_pin = EXYNOS4_GPY1(3);
		
		gpio_request(pi2cdata->sda_pin, "sda");
		gpio_request(pi2cdata->scl_pin, "scl");
		msleep(5);	
		
		if (!mx_qm_identify(data))
		{
			err = -ENODEV;	
			goto err_free_mem;					
		}
		else
		{
			pr_info("mx_qm:old hardware version!!!\n");
		}
	}

	qm_create_attrs(&client->dev);
		
	/*initial registers*/
	mx_qm_init_registers(data);

	err = mfd_add_devices(data->dev, -1, mx_qm_devs,
			ARRAY_SIZE(mx_qm_devs), NULL, 0);
	if (err) {
		dev_err(&client->dev,"MX QMatrix Sensor mfd add devices failed\n");
		goto err_free_mem;
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
		 data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
		 data->early_suspend.suspend = mx_qm_early_suspend;
		 data->early_suspend.resume = mx_qm_late_resume;
		 register_early_suspend(&data->early_suspend);
#endif

	pr_debug("%s:--\n",__func__);
	return 0;

err_free_mem:	
	pr_err("%s:init failed! \n",__func__);
	mutex_destroy(&data->iolock);
	wake_lock_destroy(&data->wake_lock);
	i2c_set_clientdata(client, NULL);
	kfree(data);
	return err;
}

#ifdef CONFIG_PM
static int mx_qm_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct mx_qm_data *mx = i2c_get_clientdata(i2c);

	disable_irq(mx->irq);
	enable_irq_wake(mx->irq);

	mx->wakeup(mx,false);

	return 0;
}

static int mx_qm_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct mx_qm_data *mx = i2c_get_clientdata(i2c);
	
	mx->wakeup(mx,true);

	disable_irq_wake(mx->irq);
	enable_irq(mx->irq);

	return 0;
}

#else
#define mx_qm_suspend	NULL
#define mx_qm_resume	NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops mx_qm_pm = {
	.suspend = mx_qm_suspend,
	.resume = mx_qm_resume,
};

static int __devexit mx_qm_remove(struct i2c_client *client)
{
	struct mx_qm_data *data = i2c_get_clientdata(client);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	mutex_destroy(&data->iolock);
	wake_lock_destroy(&data->wake_lock);
	qm_destroy_atts(&client->dev);
	
	kfree(data);

	i2c_set_clientdata(client, NULL);

	return 0;
}

void mx_qm_shutdown(struct i2c_client *client)
{
	int ret = 0;

	ret = mx_qm_writebyte(client,QM_REG_STATUS,QM_STATE_SHUTDOWN);
	if (ret < 0)
		pr_err("mx_qm_writebyte error at %d line\n", __LINE__);
}

static const struct i2c_device_id mx_qm_id[] = {
	{ "mx_qm", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mx_qm_id);

static struct i2c_driver mx_qm_driver = {
	.driver	= {
		.name	= "mx_qm",
		.owner	= THIS_MODULE,
		.pm = &mx_qm_pm,
	},
	.id_table	= mx_qm_id,
	.probe		= mx_qm_probe,
	.remove		= __devexit_p(mx_qm_remove),
	.shutdown	= mx_qm_shutdown,
};

static int __init mx_qm_init(void)
{
	return i2c_add_driver(&mx_qm_driver);
}
module_init(mx_qm_init);

static void __exit mx_qm_exit(void)
{
	i2c_del_driver(&mx_qm_driver);
}
module_exit(mx_qm_exit);

MODULE_AUTHOR("Chwei <Chwei@meizu.com>");
MODULE_DESCRIPTION("MX QMatrix Sensor");
MODULE_LICENSE("GPL");
