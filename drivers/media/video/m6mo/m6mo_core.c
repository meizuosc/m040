#include <linux/i2c.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/m6mo.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h> 

#include <linux/delay.h>

#include "m6mo.h"
#include "m6mo_regs.h"
#include "m6mo_ctrl.h"

#define DEFAULT_DEBUG 0

#define DEFAULT_CAPTURE_WIDTH 3264
#define DEFAULT_CAPTURE_HEIGHT 2448

/* default setting registers */
static struct m6mo_reg m6mo_default_regs[] = {
	{I2C_8BIT, OUT_SELECT_REG, MIPI_IF},
	{I2C_8BIT, FSHD_EN_REG, FSHD_ON},
	{I2C_8BIT, JPEG_RATIO_REG, 0x62},
	{I2C_32BIT, IASTER_CLK_FOR_SENSOR_REG, 0x06ddd000}, /* 24MHZ clock */
};

/* preview size regs */
static struct m6mo_reg m6mo_prev_vga[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_VGA},
};
static struct m6mo_reg m6mo_prev_854x640[] = { 
	{I2C_8BIT, MON_SIZE_REG, MON_854X640},
};
static struct m6mo_reg m6mo_prev_960x720[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_960X720},
};
static struct m6mo_reg m6mo_prev_720p[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_720P},
};
static struct m6mo_reg m6mo_prev_1216x912[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_1216X912},
};
static struct m6mo_reg m6mo_prev_1080p[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_1080P},
};

/* capture format regs */
static struct m6mo_reg m6mo_fmt_yuv422[] = {
	{I2C_8BIT, YUVOUT_MAIN_REG, MAIN_OUTPUT_YUV422},
};
static struct m6mo_reg m6mo_fmt_jpeg[] = {
	{I2C_8BIT, YUVOUT_MAIN_REG, MAIN_OUTPUT_JPEG_422},
};

/* capture size regs */
static struct m6mo_reg m6mo_cap_vga[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_640_480},
};
static struct m6mo_reg m6mo_cap_720p[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_720p},
};
static struct m6mo_reg m6mo_cap_1m[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_1280_960},
};
static struct m6mo_reg m6mo_cap_2m[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_1600_1200},
};
static struct m6mo_reg m6mo_cap_3m[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_2048_1536},
};
static struct m6mo_reg m6mo_cap_5m[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_2560_1920},
};
static struct m6mo_reg m6mo_cap_8m[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_3264_2448},
};

/* preview size structs */
static struct m6mo_size_struct m6mo_prev_sizes[] = {
	{  /* 1080P */
		.width		= 1920,
		.height		= 1088,
		.regs 		= m6mo_prev_1080p,
		.size			= ARRAY_SIZE(m6mo_prev_1080p),
	},
	{  /* 1216 * 912 */
		.width		= 1216,
		.height		= 912,
		.regs 		= m6mo_prev_1216x912,
		.size			= ARRAY_SIZE(m6mo_prev_1216x912),
	},
	{  /* 720P */
		.width		= 1280,
		.height		= 720,
		.regs 		= m6mo_prev_720p,
		.size			= ARRAY_SIZE(m6mo_prev_720p),
	},
	{  /* 960 * 720*/
		.width		= 960,
		.height		= 720,
		.regs 		= m6mo_prev_960x720,
		.size			= ARRAY_SIZE(m6mo_prev_960x720),
	},
	{ /* 854 * 640 */
		.width		= 854,
		.height		= 640,
		.regs 		= m6mo_prev_854x640,
		.size			= ARRAY_SIZE(m6mo_prev_854x640),
	},	
	{ /* VGA */
		.width		= 640,
		.height		= 480,
		.regs 		= m6mo_prev_vga,
		.size			= ARRAY_SIZE(m6mo_prev_vga),
	},	
};

/* capture format structs */
static struct m6mo_format_struct m6mo_cap_formats[] = {
	{
		.desc = "YUYV 4:2:2",
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.mbus_code = V4L2_MBUS_FMT_VYUY8_2X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.bpp = 2,
		.regs = m6mo_fmt_yuv422,
		.size = ARRAY_SIZE(m6mo_fmt_yuv422),
	},
	{
		.desc = "JPEG encoded data",
		.pixelformat = V4L2_PIX_FMT_JPEG,
		.mbus_code = V4L2_MBUS_FMT_JPEG_1X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.bpp = 0,
		.regs = m6mo_fmt_jpeg,
		.size = ARRAY_SIZE(m6mo_fmt_jpeg),
	},
};

/* capture size structs */
static struct m6mo_size_struct m6mo_cap_sizes[] = {
	{  /* 8M */
		.width		= 3264,
		.height		= 2448,
		.regs 		= m6mo_cap_8m,
		.size			= ARRAY_SIZE(m6mo_cap_8m),
	},	
	{  /* 5M */
		.width		= 2560,
		.height		= 1920,
		.regs 		= m6mo_cap_5m,
		.size			= ARRAY_SIZE(m6mo_cap_5m),
	},
	{  /* 3M */
		.width		= 2048,
		.height		= 1536,
		.regs 		= m6mo_cap_3m,
		.size			= ARRAY_SIZE(m6mo_cap_3m),
	},	
	{  /* 2M */
		.width		= 1600,
		.height		= 1200,
		.regs 		= m6mo_cap_2m,
		.size			= ARRAY_SIZE(m6mo_cap_2m),
	},
	{  /* 1M */
		.width		= 1280,
		.height		= 960,
		.regs 		= m6mo_cap_1m,
		.size			= ARRAY_SIZE(m6mo_cap_1m),
	},
	{  /* 720P */
		.width		= 1280,
		.height		= 720,
		.regs 		= m6mo_cap_720p,
		.size			= ARRAY_SIZE(m6mo_cap_720p),
	},
	{  /* VGA */
		.width		= 640,
		.height		= 480,
		.regs 		= m6mo_cap_vga,
		.size			= ARRAY_SIZE(m6mo_cap_vga),
	},
};

static u32 m6mo_swap_byte(u8 *data, enum m6mo_i2c_size size)
{
	if (size == I2C_8BIT)
		return *data;
	else if (size == I2C_16BIT)
		return be16_to_cpu(*((u16 *)data));
	else 
		return be32_to_cpu(*((u32 *)data));
}

/*
  * I2C read reg function
  * size: read size, 8 bit
  * addr: 16bit address, higher byte is category, lower byte cmd
  * val: store for read value
*/
int m6mo_read_reg(struct v4l2_subdev *sd,
		u16 addr, u32 *val,
		enum m6mo_i2c_size size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m6mo_state *state = to_state(sd);
	struct i2c_msg msg[2];
	u8 wbuf[5], rbuf[I2C_MAX + 1];
	u8 category = (addr >> 8) & 0xff;
	u8 cmd = addr & 0xff;
	int ret;

	if (!client->adapter)
		return -ENODEV;

	if (size != I2C_8BIT && size != I2C_16BIT && size != I2C_32BIT)
		return -EINVAL;

	/* 1st I2C operation for writing category & command. */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 5;		/* 1(cmd size per bytes) + 4 */
	msg[0].buf = wbuf;
	wbuf[0] = 5;		/* same right above this */
	wbuf[1] = CMD_READ_PARAMETER;
	wbuf[2] = category;
	wbuf[3] = cmd;
	wbuf[4] = size;

	/* 2nd I2C operation for reading data. */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size + 1;
	msg[1].buf = rbuf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "failed READ[%d] at "
				"cat[%02x] cmd[%02x]\n",
				size, category, cmd);
		return ret;
	}

	*val = m6mo_swap_byte(&rbuf[1], size);

	if (state->debug) 
		pr_info("read: 0x%04x = 0x%x\n", addr, *val);

	return 0;
}

/*
  * I2C write reg function
  * size: write size, 8 bit
  * addr: 16bit address, higher byte is category, lower byte cmd
  * val: write value
*/
int m6mo_write_reg(struct v4l2_subdev *sd,
		u16 addr, u32 val,
		enum m6mo_i2c_size size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m6mo_state *state = to_state(sd);
	struct device *cdev = &client->dev;
	struct i2c_msg msg[1];
	u8 wbuf[I2C_MAX + 4];
	u32 *buf = (u32 *)&wbuf[4];
	u8 category = (addr >> 8) & 0xff;
	u8 cmd = addr & 0xff;
	int ret;

	if (!client->adapter)
		return -ENODEV;

	if (size != I2C_8BIT && size != I2C_16BIT && size != I2C_32BIT) {
		dev_err(cdev, "Wrong data size\n");
		return -EINVAL;
	}

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = size + 4;
	msg->buf = wbuf;
	wbuf[0] = size + 4;
	wbuf[1] = CMD_WRITE_PARAMETER;
	wbuf[2] = category;
	wbuf[3] = cmd;

	*buf = m6mo_swap_byte((u8 *)&val, size);

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "failed WRITE[%d] at "
				"cat[%02x] cmd[%02x], ret %d\n",
				size, msg->buf[2], msg->buf[3], ret);
		return ret;
	}

	if (state->debug) 
		pr_info("write: 0x%04x = 0x%x\n", addr, val);

	return 0;
}

/*
  * I2C read memory function
  * data: write data
  * addr: 32bit address
  * size: write size, max 16 bit
*/
int m6mo_write_memory(struct v4l2_subdev *sd,
		u32 addr, const char *data,
		int size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m6mo_state *state = to_state(sd);
	struct i2c_msg msg[1];
	int ret;

	/*firmware buffer should be alloc memory before download*/
	if (!state->fw_buffer) {
		pr_err("%s: firmware buffer is NULL\n", __func__);
		return -EINVAL;
	}

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = size + 8;
	msg->buf = state->fw_buffer;
	msg->buf[0] = 0;
	msg->buf[1] = CMD_WRITE_8BIT_MEMORY;
	msg->buf[2] = (addr >> 24) & 0xff;
	msg->buf[3] = (addr >> 16) & 0xff;
	msg->buf[4] = (addr >> 8) & 0xff;
	msg->buf[5] = addr & 0xff;
	msg->buf[6] = (size >> 8) & 0xff;
	msg->buf[7] = size & 0xff;
	memcpy(&msg->buf[8], data, size);

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "failed WRITE[%d] at "
				"cat[%02x] cmd[%02x], ret %d\n",
				size, msg->buf[2], msg->buf[3], ret);
		return ret;
	}

	return 0;
}

int m6mo_write_regs(struct v4l2_subdev *sd, struct m6mo_reg *regs, int size)
{
	int ret = 0, i;
	struct m6mo_reg reg;

	for (i = 0; i < size; i++) {
		reg = regs[i];
		ret = m6mo_write_reg(sd, reg.addr, reg.val, reg.size);
		CHECK_ERR(ret);
	}

	return 0;
}

static bool m6mo_is_power(struct m6mo_state *state)
{
	return state->isp_power;
}


/*****************************************/
/**********       sys interface    **************/
/*****************************************/

/*
  * show firmware status
*/
static ssize_t show_firmware_status(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct m6mo_state *state = to_state(sd);	

	return sprintf(buf, "%d\n", state->fw_status);
}

static ssize_t store_register(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct m6mo_state *state = to_state(sd);
	int ret, type;
	u32 addr, val;

	if (!m6mo_is_power(state)) {
		pr_err("Please power on first\n");
		return -EINVAL;
	}
	
	if (sscanf(buf, "w%d %x=%x", &type, &addr, &val) == 3) {
		switch (type) {
		case 8:
			ret = m6mo_w8(sd, (u16)addr, val);
			break;
		case 16:
			ret = m6mo_w16(sd, (u16)addr, val);
			break;			
		case 32:
			ret = m6mo_w32(sd, (u16)addr, val);
			break;			
		default:
			ret = -EINVAL;
			break;			
		}
		CHECK_ERR(ret);
		pr_info("write: 0x%04x = 0x%x\n", addr, val); 
	} else if (sscanf(buf, "r%d %x", &type, &addr) == 2) {
		switch (type) {
		case 8:
			ret = m6mo_r8(sd, (u16)addr, &val);
			break;
		case 16:
			ret = m6mo_r16(sd, (u16)addr, &val);
			break;			
		case 32:
			ret = m6mo_r32(sd, (u16)addr, &val);		
		default:
			ret = -EINVAL;
			break;			
		}
		CHECK_ERR(ret);
		pr_info("read: 0x%04x = 0x%x\n", addr, val); 
	} else {
		ret = -EINVAL;
		pr_err("Invalid format. write format: echo > \"w8/16/32 addr=value\" > register; \
			read format: echo > \"r8/16/32 addr\n");
	}
	
	return count;
}

static ssize_t store_erase(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct v4l2_subdev *sd = dev_get_drvdata(d);

	ret = m6mo_erase_firmware(sd);
	if (ret) return -1;
	
	return count;
}

static ssize_t store_debug(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct m6mo_state *state = to_state(sd);

	if (sscanf(buf, "%d", &val) == 1)
		state->debug = !!val;

	return count;
}

static ssize_t show_download_firmware(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct m6mo_state *state = to_state(sd);	
	
	if (state->fw_status == FIRMWARE_CHECKED)
		return sprintf(buf, "FIRMWARE_VERSION:0x%x\n", state->fw_version);
	else
		return sprintf(buf, "FIRMWARE_VERSION:0x%x\n", 0);  /* means incorrect */
}

static ssize_t store_download_firmware(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct v4l2_subdev *sd = dev_get_drvdata(d);

	ret = m6mo_load_firmware_sys(d, sd);
	if (ret) 
		return -EINVAL;
	else 
		return count;
}

static ssize_t show_pre_flash_current(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct m6mo_state *state = to_state(sd);	

	return sprintf(buf, "%d\n", state->pre_flash_current / 1000);
}

static ssize_t store_pre_flash_current(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct m6mo_state *state = to_state(sd);

	if (sscanf(buf, "%d", &val) == 1) {
		val = val * 1000;
		if (val > MAX_FLASH_CURRENT) {
			pr_err("%s:flash current is too large!!\n", __func__);
			return -EINVAL;
		}
		state->pre_flash_current = val;
	} else 
		return -EINVAL;

	pr_info("%s:pre flash current = %d uA\n", __func__, state->pre_flash_current);
	
	return count;
}

static ssize_t show_full_flash_current(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct m6mo_state *state = to_state(sd);	

	return sprintf(buf, "%d\n", state->full_flash_current / 1000);
}

static ssize_t store_full_flash_current(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct m6mo_state *state = to_state(sd);

	if (sscanf(buf, "%d", &val) == 1) {
		val = val * 1000;
		if (val > MAX_FLASH_CURRENT) {
			pr_err("%s:flash current is too large!!\n", __func__);
			return -EINVAL;
		}
		state->full_flash_current = val;
	} else 
		return -EINVAL;

	pr_info("%s:full flash current = %d uA\n", __func__, state->full_flash_current);
	
	return count;
}

static DEVICE_ATTR(firmware_status, 0444, show_firmware_status, NULL);
static DEVICE_ATTR(register, 0220, NULL, store_register);
static DEVICE_ATTR(erase, 0220, NULL, store_erase);
static DEVICE_ATTR(debug, 0220, NULL, store_debug);
static DEVICE_ATTR(download_firmware, 0660, 
	show_download_firmware, store_download_firmware);
static DEVICE_ATTR(pre_flash_current, 0660, 
	show_pre_flash_current, store_pre_flash_current);
static DEVICE_ATTR(full_flash_current, 0660, 
	show_full_flash_current, store_full_flash_current);

static struct attribute *m6mo_attributes[] = {
	&dev_attr_firmware_status.attr,
	&dev_attr_register.attr,
	&dev_attr_erase.attr,
	&dev_attr_debug.attr,
	&dev_attr_download_firmware.attr,
	&dev_attr_pre_flash_current.attr,
	&dev_attr_full_flash_current.attr,
	NULL
};

static const struct attribute_group m6mo_group = {
	.attrs = m6mo_attributes,
};

/*
  * clear completion counter function
  * before waiting completion, we should do this
*/
void m6mo_prepare_wait(struct v4l2_subdev *sd)
{
	struct m6mo_state *state = to_state(sd);

	init_completion(&state->completion);
}

/*
  * wait interrupt event function without checking
*/
int m6mo_wait_irq(struct v4l2_subdev *sd, const unsigned int timeout)
{
	struct m6mo_state *state = to_state(sd);
	int ret;

	ret = wait_for_completion_interruptible_timeout(&state->completion, 
		msecs_to_jiffies(timeout));
	if (ret <= 0)
		return -ETIME;
	return 0;
}

/*
  * wait interrupt event function with timeout
  * if proper event comes, return 0
*/
int m6mo_wait_irq_and_check(struct v4l2_subdev *sd, u8 mask,
	const unsigned int timeout)
{
	struct m6mo_state *state = to_state(sd);
	int ret;

	ret = wait_for_completion_interruptible_timeout(&state->completion, 
		msecs_to_jiffies(timeout));
	if (ret <= 0) {
		pr_info("%s: timeout in %u ms when wating for %d\n", 
			__func__, timeout, mask);
		return -ETIME;
	}

	mutex_lock(&state->mutex);

	if ((state->irq_status & mask) == mask)
		ret = 0;
	else 
		ret = -EINVAL;
	mutex_unlock(&state->mutex);	
	
	return ret;
}


int m6mo_enable_root_irq(struct v4l2_subdev *sd)
{
	return m6mo_w8(sd, INT_ROOR_ENABLE_REG, 0x01);
}

int m6mo_enable_irq(struct v4l2_subdev *sd)
{
	int ret;

	ret = m6mo_w8(sd, INT_ENABLE_REG, 0xff);
	CHECK_ERR(ret);
	
	return m6mo_w8(sd, INT_ROOR_ENABLE_REG, 0x01);
}

/*
  * set ISP operating mode function
  * parameter, monitor or capture
*/
int m6mo_set_sys_mode(struct v4l2_subdev *sd, enum isp_mode mode)
{
	int ret;
	enum sys_mode smode;
	struct m6mo_state *state = to_state(sd);

	if (state->mode == mode) return 0;

	pr_info("%s(), ISP mode will be set to %d\n", __func__, mode);

	switch (mode) {
	case PARAMETER_MODE:
		smode = SYS_PARAMETER_MODE;
		break;
	case MONITOR_MODE:
		smode = SYS_MONITOR_MODE;
		break;
	case CAPTURE_MODE:
		smode = SYS_CAPTURE_MODE;
		break;
	default:
		return -EINVAL;
	}

	m6mo_prepare_wait(sd);
	
	ret = m6mo_enable_irq(sd);
	if (ret) return ret;
	
	ret = m6mo_w8(sd,  SYS_MODE_REG, smode);	
	CHECK_ERR(ret);

	ret = m6mo_wait_irq_and_check(sd, INT_MASK_MODE, WAIT_TIMEOUT);
	if (ret) return ret;

	state->mode = mode;

	return 0;
}

/*
  * set isp parameter mode function
*/
static int m6mo_set_parameter_mode(struct v4l2_subdev *sd)
{
	int ret, i, retry = 3;
	struct m6mo_state *state = to_state(sd);

	if (state->mode == PARAMETER_MODE) return 0;

	m6mo_prepare_wait(sd);

	ret = m6mo_w8(sd,  SYS_MODE_REG, SYS_PARAMETER_MODE);	
	CHECK_ERR(ret);
	
	for (i = 0; i < retry; i++) {
		ret = wait_for_completion_interruptible_timeout(&state->completion, 
			msecs_to_jiffies(2000));
		if (ret <= 0) {
			pr_err("%s: timeout in %u ms\n", __func__, 2000);
			return -ETIME;
		}

		if (state->irq_status & INT_MASK_MODE)
			break;
	}

	if (i == retry) return -EINVAL;

	state->mode = PARAMETER_MODE;

	return 0;
}

/*
  * set isp normal monitor(preview) mode function
*/
static int m6mo_set_monitor_mode(struct v4l2_subdev *sd)
{
	int ret, i, retry = 3;
	struct m6mo_state *state = to_state(sd);

	if (state->mode == MONITOR_MODE) return 0;

	/* ensure special panorama off */
	pr_info("%s(), set before change to monitor mode\n", __func__);
	ret = m6mo_w8(sd, SPECIAL_MON_REG, SPECIAL_OFF);
	CHECK_ERR(ret);

	m6mo_prepare_wait(sd);

	ret = m6mo_w8(sd,  SYS_MODE_REG, SYS_MONITOR_MODE);	
	CHECK_ERR(ret);
	
	for (i = 0; i < retry; i++) {
		ret = wait_for_completion_interruptible_timeout(&state->completion, 
			msecs_to_jiffies(2000));
		if (ret <= 0) {
			pr_err("%s: timeout in %u ms\n", __func__, 2000);
			return -ETIME;
		}

		if (state->irq_status & INT_MASK_MODE)
			break;
	}

	if (i == retry) return -EINVAL;

	/* set capture normal mode */
	ret = m6mo_w8(sd, CAP_MODE_REG, CAP_MODE_NORMAL);
	CHECK_ERR(ret);

	state->mode = MONITOR_MODE;

	return 0;
}

/*
  * set isp capture mode function
*/
static int m6mo_set_capture_mode(struct v4l2_subdev *sd)
{
	int ret, i, retry = 5;
	struct m6mo_state *state = to_state(sd);

	pr_info("%s:wdr = %d\n", __func__, state->userset.wdr);

	if (state->mode == CAPTURE_MODE) return 0;

	m6mo_prepare_wait(sd);

	ret = m6mo_w8(sd,  SYS_MODE_REG, SYS_CAPTURE_MODE);	
	CHECK_ERR(ret);
	
	for (i = 0; i < retry; i++) {
		ret = wait_for_completion_interruptible_timeout(&state->completion, 
			msecs_to_jiffies(5000));
		if (ret <= 0) {
			pr_err("%s: timeout in %u ms\n", __func__, 5000);
			return -ETIME;
		}

		mutex_lock(&state->mutex);

		/* change flash to full current */
		if (state->irq_status & INT_MASK_MODE)
			if (state->userset.flash_mode != M6MO_FLASH_OFF)
				m6mo_set_flash_current(state, state->full_flash_current);

		if (state->irq_status & INT_MASK_CAPTURE) {
			/* recovery flash to pre current */
			if (state->userset.flash_mode != M6MO_FLASH_OFF)
				m6mo_set_flash_current(state, state->pre_flash_current);
			mutex_unlock(&state->mutex);
			break;
		}
		mutex_unlock(&state->mutex);	
	}

	if (i == retry) return -EINVAL;

	state->mode = CAPTURE_MODE;

	return 0;
}

int m6mo_set_mode(struct v4l2_subdev *sd, enum isp_mode mode)
{
	int ret;

	if (to_state(sd)->mode == mode) {
		pr_warn("%s(), ISP mode has already been set to %d\n",
			__func__, mode);
		return 0;
	} else {
		pr_info("%s(), set ISP mode to %d\n", __func__, mode);
	}

	switch (mode) {	
	case PARAMETER_MODE:
		ret = m6mo_set_parameter_mode(sd);
		break;
	case MONITOR_MODE:
		ret = m6mo_set_monitor_mode(sd);
		break;
	case CAPTURE_MODE:
		ret = m6mo_set_capture_mode(sd);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int m6mo_reset_ae_awb_lock(struct v4l2_subdev *sd)
{
	int ret;
	ret = m6mo_w8(sd, AE_LOCK_REG, 0x00);	
	CHECK_ERR(ret);
	
	return m6mo_w8(sd, AWB_LOCK_REG, 0x00);
}

/*
  * set isp panorama preview mode
*/
static int m6mos_set_panorama_mode(struct v4l2_subdev *sd)
{
	int ret;

	/* shoud set parameter mode first */
	ret = m6mo_set_mode(sd, PARAMETER_MODE);
	if (ret) return ret;

	/* enable special panorama first */
	ret = m6mo_w8(sd, SPECIAL_MON_REG, SPECIAL_PANORAMA);
	CHECK_ERR(ret);

	/* enter monitor mode */
	ret = m6mo_set_sys_mode(sd, MONITOR_MODE);
	if (ret) return ret;
	
	/* reset AE and AWB Lock */
	ret = m6mo_reset_ae_awb_lock(sd);
	if (!ret) 
		pr_info("%s():change camera to panorama monitor mode.\n", __func__);

	return ret;
}

static void set_prev_special_resolution_val(struct m6mo_state *state,
	enum v4l2_camera_mode mode)
{
	/* normal preview and record */
	if (mode != V4L2_CAMERA_PANORAMA) {
		m6mo_prev_1216x912[0].val = 0x36;
	} else {
		/* panorama preview mode */
		m6mo_prev_1216x912[0].val = 0x24;
	}

	/*
	* For Front camera record mode's frame rate
	*/
	if (FRONT_CAMERA == state->cam_id  &&
		V4L2_CAMERA_RECORD == mode) {
		m6mo_prev_720p[0].val = 0x28;		
	} else {
		m6mo_prev_720p[0].val = 0x21;
	}
}

static int m6mo_set_preview_size(struct v4l2_subdev *sd, 
	struct v4l2_mbus_framefmt *fmt, enum v4l2_camera_mode mode)
{
	struct m6mo_state *state = to_state(sd);
	int i, ret;
	bool update_isp = false;
	static unsigned int last_reg_val = 0;
	struct m6mo_size_struct *sizes = m6mo_prev_sizes;
	int len = ARRAY_SIZE(m6mo_prev_sizes);
		
	/* look down to find the smaller preview size */
	for (i = len - 1; i >= 0; i--) 
		if ((fmt->width <= sizes[i].width) &&
			(fmt->height <= sizes[i].height))
			break;

	if (i < 0) {
		pr_err("%s: can not mach preview size(%d, %d)\n", 
			__func__, fmt->width, fmt->height);
		return -EINVAL;
	}

	fmt->width = sizes[i].width;
	fmt->height = sizes[i].height;

	set_prev_special_resolution_val(state, mode);

	if (last_reg_val != sizes[i].regs->val ||
		state->prev_size.width != fmt->width ||
		state->prev_size.height != fmt->height) {
		update_isp = true;
	}

	state->prev_size = sizes[i];

	if (update_isp) {		
		/* should be set parameter mode first */
		ret = m6mo_set_mode(sd, PARAMETER_MODE);
		if (ret) return ret;

		pr_info("%s(), write ISP reg 0x%04x to value 0x%x\n"
			"Last reg value is 0x%x", __func__,
			state->prev_size.regs->addr, state->prev_size.regs->val, last_reg_val);
		ret = m6mo_write_regs(sd, state->prev_size.regs, state->prev_size.size);
		if (ret) return ret;

		last_reg_val = state->prev_size.regs->val;
		
	} else {
		pr_info("%s(), no Need to update ISP register\n", __func__);
	}
	
	return 0;
}

int m6mo_set_capture_format(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	struct m6mo_state *state = to_state(sd);
	struct m6mo_format_struct *formats = m6mo_cap_formats;
	int i, ret, len = ARRAY_SIZE(m6mo_cap_formats);
		
	for (i = 0; i < len; i++)
		if (fmt->code == formats[i].mbus_code)
			break;

	if (i == len) {
		pr_err("%s: can not mach format(%d)\n", __func__, fmt->code);
		return -EINVAL;
	}

	if (state->cap_fmt.mbus_code == fmt->code)
		return 0;

	state->cap_fmt = formats[i];
	
	ret = m6mo_write_regs(sd, state->cap_fmt.regs, state->cap_fmt.size);
	if (ret) return ret;

	return 0;
}

int m6mo_set_capture_size(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	struct m6mo_state *state = to_state(sd);
	struct m6mo_size_struct *sizes = m6mo_cap_sizes;
	int i, ret, len = ARRAY_SIZE(m6mo_cap_sizes);

	pr_info("%s(), user wants %d * %d capture size\n",
		__func__, fmt->width, fmt->height);
	for (i = 0; i < len; i++)
		if ((fmt->width == sizes[i].width) &&
			(fmt->height == sizes[i].height))
			break;

	if (i == len) {
		pr_err("%s: can not mach capture size(%d, %d)\n", 
			__func__, fmt->width, fmt->height);
		return -EINVAL;
	}

	fmt->width = sizes[i].width;
	fmt->height = sizes[i].height;

	if (state->cap_size.width == fmt->width &&
		state->cap_size.height == fmt->height)
		return 0;

	state->cap_size = sizes[i];

	ret = m6mo_write_regs(sd, state->cap_size.regs, state->cap_size.size);
	if (ret) return ret;

	return 0;
}

static int 
m6mo_get_camera_mode_type(struct m6mo_state *state, enum v4l2_camera_mode mode)
{
	if (mode < V4L2_CAMERA_SINGLE_CAPTURE)
		return PREVIEW_MODE_TYPE;
	else if (mode <= V4L2_CAMERA_PANORAMA_CAPTURE)
		return CAPTURE_MODE_TYPE;
	else 
		return -EINVAL;
}

static int m6mo_s_fmt(struct v4l2_subdev *sd, 
	struct v4l2_mbus_framefmt *fmt)
{
	int ret;
	int mode = fmt->reserved[0];
	struct m6mo_state *state = to_state(sd);
	enum camera_mode_type type;

	pr_info("%s: >> (w, h) = (%d, %d), fmt = %d, mode =%d\n", 
		__func__, fmt->width, fmt->height, fmt->code, mode);

	fmt->field = V4L2_FIELD_NONE;

	type = m6mo_get_camera_mode_type(state, mode);
	if (type < 0) return -EINVAL;
	
	if (type == PREVIEW_MODE_TYPE) {
		ret = m6mo_set_preview_size(sd, fmt, mode);
		if (ret) return ret;
	} else {  /* CAPTURE_MODE_TYPE */
		/* do nothing */
	}

	state->camera_mode = mode;

	pr_info("%s: << (w, h) = (%d, %d), fmt = %d, mode =%d\n", 
		__func__, fmt->width, fmt->height, fmt->code, mode);
	
	return 0;
}

static int m6mo_enum_framesizes(struct v4l2_subdev *sd, 
					struct v4l2_frmsizeenum *fsize)
{
	struct m6mo_state *state = to_state(sd);
	enum camera_mode_type type;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	
	type = m6mo_get_camera_mode_type(state, state->camera_mode);
	if (type < 0) return -EINVAL;
	
	if (type == PREVIEW_MODE_TYPE) {
		fsize->discrete.width = state->prev_size.width;
		fsize->discrete.height = state->prev_size.height;	
	} else {
		fsize->discrete.width = state->cap_size.width;
		fsize->discrete.height = state->cap_size.height;	
	}

	return 0;
}

static int m6mo_stream_on(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m6mo_state *state = to_state(sd);	
	int ret = 0;

	v4l_info(client, "%s: stream on, camera_mode is %d\n", 
		__func__, state->camera_mode);

	/* reset this flag in case of wrong wake up */
	fimc_reset_wakeup_flag();
	
	switch (state->camera_mode) {
	case  V4L2_CAMERA_PREVIEW:
	case  V4L2_CAMERA_RECORD:
		pr_info("%s(), will change to non-panorama preview mode\n", __func__);
		ret = m6mo_set_mode(sd, MONITOR_MODE);
		break;
	case V4L2_CAMERA_PANORAMA:
		ret = m6mos_set_panorama_mode(sd);
		break;
	default:
		break;
	}

	if (!ret) state->stream_on = true;

	return ret;
}

static int m6mo_stream_off(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m6mo_state *state = to_state(sd);	

	v4l_info(client, "%s:  stream off\n", __func__);
	
	if (state->stream_on) state->stream_on = false;
	
	return 0;
}

static int m6mo_s_stream(struct v4l2_subdev *sd, int enable)
{
	if(enable)
		return m6mo_stream_on(sd);
	else
		return m6mo_stream_off(sd);
}

/*Remove m6mo_fps_to_sensor_step() as we don't use it any more*/

static int m6mo_s_parm(struct v4l2_subdev *sd, 
	struct v4l2_streamparm *param)
{
	/* do nothing, we don't set framerate registers any more */
	
	return 0;
}

static int m6mo_reset(struct v4l2_subdev *sd, u32 val)
{
	struct m6mo_state *state = to_state(sd);

	state->pdata->reset();
	
	return 0;
}

static int m6mo_set_sensor_power(struct m6mo_state *state, bool enable)
{
	int ret;
	
	if (state->sensor_power == enable)
		return -EBUSY;

	ret = state->pdata->set_sensor_power(state->cam_id, enable);
	if (!ret) state->sensor_power = enable;

	return ret;
}

int m6mo_set_power_clock(struct m6mo_state *state, bool enable)
{
	struct m6mo_platform_data *pdata = state->pdata;
	struct v4l2_subdev *sd = &state->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	
	if (state->isp_power == enable) {
		pr_err("%s(), ISP Power has already been set to %d!\n",
			__func__, enable);
		return -EBUSY;
	}
	
	if (enable) {
		ret = pdata->set_isp_power(true);
		if (ret) {
			pr_err("%s(), set ISP powr failed!\n", __func__);
			return ret;
		}

		ret = pdata->clock_enable(&client->dev, true);
		if (ret) return ret;
		state->isp_power = enable;	
	} else {
		state->isp_power = enable;
		pdata->clock_enable(&client->dev, false);
		pdata->set_isp_power(false);
		if (state->sensor_power)   
			m6mo_set_sensor_power(state, false);
	}

	pr_info("%s(), set power to %d successed!\n", __func__, enable);

	return ret;
}

int m6mo_s_power(struct v4l2_subdev *sd, int on)
{
	struct m6mo_state *state = to_state(sd);
	bool enable = !!on;

	if (state->fw_status == FIRMWARE_REQUESTING)
		return -EBUSY;
	
	return m6mo_set_power_clock(state, enable);
}

static int m6mo_init(struct v4l2_subdev *sd, u32 cam_id)
{
	int ret;
	struct m6mo_state *state = to_state(sd);	
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (cam_id != BACK_CAMERA && cam_id != FRONT_CAMERA) {
		pr_err("unsupport camera id %d\n", cam_id);
		return -EINVAL;
	}

	state->cam_id = cam_id;

	/* set sensor power */
	ret = m6mo_set_sensor_power(state, true);
	if (ret) {
		pr_err("%s(), set sensor powr failed!\n", __func__);
		return ret;
	}

	m6mo_s_power(sd, 1);

	/* run firmware first */
	ret = m6mo_run_firmware(sd);	
	if (ret) goto err_power;

	/* init default registers */
	ret = m6mo_write_regs(sd, m6mo_default_regs, ARRAY_SIZE(m6mo_default_regs));
	if (ret) goto err_power;

	/* enable all irq */
	ret = m6mo_enable_irq(sd);
	if (ret) goto err_power;

	if (state->cam_id == BACK_CAMERA)
		ret = m6mo_w8(sd, 0x013f, 0x00);
	else
		ret = m6mo_w8(sd, 0x013f, 0x01);
	if (ret) goto err_power;
	
	state->fps = M6MO_FR_AUTO;
	state->mode = PARAMETER_MODE;
	state->stream_on = false;
	memset(&state->prev_size, 0, sizeof(struct m6mo_size_struct));

	state->cap_size.width = DEFAULT_CAPTURE_WIDTH;
	state->cap_size.height = DEFAULT_CAPTURE_HEIGHT;
	state->cap_fmt.mbus_code = V4L2_MBUS_FMT_JPEG_1X8;

	/* init default userset parameter, this is the reset value of ISP */
	state->userset.manual_wb = M6MO_WB_AUTO;
	state->userset.scene = M6MO_SCENE_AUTO;	
	state->userset.zoom_level = M6MO_ZL_1;
	state->userset.wdr = M6MO_WDR_OFF;
	state->userset.iso = M6MO_ISO_AUTO;
	state->userset.flash_mode = M6MO_FLASH_OFF;
	state->userset.rotation = M6MO_ROTATE_0;
	state->userset.mirror = M6MO_NO_MIRROR;
	state->userset.reverse = M6MO_NO_REVERSE;
	state->userset.af_mode = M6MO_FOCUS_AUTO;
	
	v4l_info(client, "%s: camera initialization finished\n", __func__);

	return 0;
	
err_power:
	m6mo_set_sensor_power(state, false);
	pr_err("%s(), camera initialization failed!!\n", __func__);
	return ret;
}

static void m6mo_handle_normal_cap(struct m6mo_state *state, int irq_status)
{
	pr_info("%s: irq status = 0x%02x\n", __func__, irq_status);
	complete(&state->completion);  /* just to wake up any waiters */
}

static void m6mo_handle_work(struct work_struct *work)
{
	struct m6mo_state *state = container_of(work, struct m6mo_state, work);
	int ret;
	u32 irq_status;

	if (!m6mo_is_power(state)) return;

	/* read interrupt status */
	ret = m6mo_r8(&state->sd, INT_FACTOR_REG, &irq_status);
	if (ret) return;	

	/* save the irq status */
	mutex_lock(&state->mutex);
	state->irq_status = irq_status;
	mutex_unlock(&state->mutex);

	switch (state->cap_mode) {
	case CAP_NORMAL_MODE:
		m6mo_handle_normal_cap(state, irq_status);
		break;
	case CAP_PANORAMA_MODE:
		m6mo_handle_panorama_cap(state, irq_status);
		break;
	case CAP_MULTI_CAP_MODE:
		m6mo_handle_multi_cap(state, irq_status);
		break;
	case CAP_SMILE_CAP_MODE:
		m6mo_handle_smile_cap(state, irq_status);
		break;
	default:
		break;
	}
}

/*
  * ISP interrupt handle function
  * we can operate i2c because it is in irq thread
*/
static irqreturn_t m6mo_irq_handler(int irq, void *dev_id)
{
	struct m6mo_state *state = (struct m6mo_state *)dev_id;

	queue_work(state->wq, &state->work);
	
	return IRQ_HANDLED;
}

/*
  * ***************** v4l2 subdev functions  *****************
*/
static const struct v4l2_subdev_core_ops m6mo_core_ops = {
	.init = m6mo_init,
	.load_fw = m6mo_load_firmware,
	.reset = m6mo_reset,
	.s_power = m6mo_s_power,
	.g_ctrl = m6mo_g_ctrl,
	.s_ctrl = m6mo_s_ctrl,
	.g_ext_ctrls = m6mo_g_ext_ctrls,
};

static const struct v4l2_subdev_video_ops m6mo_video_ops = {
	.enum_framesizes = m6mo_enum_framesizes,
	.s_mbus_fmt = m6mo_s_fmt,
	.s_stream = m6mo_s_stream,
	.s_parm = m6mo_s_parm,
};

static const struct v4l2_subdev_ops m6mo_ops = {
	.core = &m6mo_core_ops,
	.video = &m6mo_video_ops,
};

static int m6mo_check_pdata(struct m6mo_platform_data *pdata)
{
	if (pdata == NULL || !pdata->init_gpio || !pdata->init_clock || 
		!pdata->set_isp_power || !pdata->set_sensor_power || !pdata->clock_enable) {
		pr_err("platform data is uncorrect.\n");
		return -EINVAL;
	}

	return 0;
}

static int m6mo_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct m6mo_state *state;
	struct v4l2_subdev *sd = NULL;	
	struct m6mo_platform_data *pdata = client->dev.platform_data;

	/* check platform data */
	ret = m6mo_check_pdata(pdata);
	if (ret) {
		dev_err(&client->dev, "platform data is incorrect\n");
		return ret;
	}
	
	/* alloc m6mo private data */
	state = devm_kzalloc(&client->dev,
			sizeof(struct m6mo_state), GFP_KERNEL);
	if (!state) {
		dev_err(&client->dev, "can not alloc memory.\n");
		return -ENOMEM;
	}

	/* init members */
	state->pdata = pdata;
	state->isp_power = false;
	state->sensor_power = false;
	state->debug = DEFAULT_DEBUG;
	state->fled_regulator = NULL;
	state->cap_mode = CAP_NORMAL_MODE;
	state->pre_flash_current = PRE_FLASH_CURRENT;
	state->full_flash_current = FULL_FLASH_CURRENT;
	mutex_init(&state->mutex);
	init_completion(&state->completion);
	wake_lock_init(&state->wake_lock, WAKE_LOCK_SUSPEND, "m6mo");

	/* create a workqueue */
	state->wq = create_singlethread_workqueue("m6mo_wq");
	if(!state->wq){
		printk("Failed to setup workqueue - m6mo_wq \n");
		ret = -EINVAL;
		goto err_create_workqueue;
	}
	INIT_WORK(&state->work, m6mo_handle_work);

	/* register interrupt */
	ret = gpio_request(client->irq, "m6mo");
	if (ret) {
		dev_err(&client->dev, "can not request gpio (%d).\n", client->irq);
		ret = -EINVAL;
		goto err_gpio_request;
	}
	state->irq = gpio_to_irq(client->irq);
	if (state->irq < 0) {
		ret = -EINVAL;
		goto err_gpio_to_irq;
	}

	pr_debug("irq = %d, gpio=%d.\n", state->irq,  client->irq);

	ret = request_irq(state->irq, m6mo_irq_handler, IRQF_TRIGGER_RISING,
	    		client->name, state);
	if (ret) {
		dev_err(&client->dev, "request irq(%d) fail.\n", state->irq);
		goto err_gpio_to_irq;
	}

	/* register subdev */	
	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &m6mo_ops);

	/* init gpios */
	ret = pdata->init_gpio(&client->dev);
	if (ret) {
		dev_err(&client->dev, "platform init fail.\n");
		goto err_pdata_init;
	}

	/* init clocks */
	ret = pdata->init_clock(&client->dev);
	if (ret) {
		dev_err(&client->dev, "platform init fail.\n");
		goto err_pdata_init;
	}

	/* create sys interface */
	ret = sysfs_create_group(&client->dev.kobj, &m6mo_group);
	if (ret) {
		dev_err(&client->dev, "failed to create sysfs files\n");
		goto err_pdata_init;
	}

	m6mo_load_firmware(sd);
	
	dev_info(&client->dev, "m6mo has been probed\n");
	
	return 0;

err_pdata_init:
	free_irq(state->irq, state);
	v4l2_device_unregister_subdev(sd);
err_gpio_to_irq:
	gpio_free(client->irq);
err_gpio_request:
	destroy_workqueue(state->wq);
err_create_workqueue:
	devm_kfree(&client->dev, state);
	
	return ret;
}

static int m6mo_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct m6mo_state *state = to_state(sd);

	sysfs_remove_group(&client->dev.kobj, &m6mo_group);
	v4l2_device_unregister_subdev(sd);
	free_irq(state->irq, state);
	gpio_free(client->irq);
	destroy_workqueue(state->wq);
	devm_kfree(&client->dev, state);
	
	dev_info(&client->dev, "m6mo has been removed\n");	
	
	return 0;
}

static const struct i2c_device_id m6mo_id[] = {
	{M6MO_DRIVER_NAME, 0},
	{},
};

static struct i2c_driver m6mo_i2c_driver = {
	.driver = {
		.name = M6MO_DRIVER_NAME,
	}, 
	.probe = m6mo_probe,
	.remove = m6mo_remove,
	.id_table = m6mo_id,
};

static int __init m6mo_module_init(void)
{
	return i2c_add_driver(&m6mo_i2c_driver);
}

static void __exit m6mo_module_exit(void)
{
	i2c_del_driver(&m6mo_i2c_driver);
}

module_init(m6mo_module_init);
module_exit(m6mo_module_exit);

MODULE_DESCRIPTION("Fujitsu m6mo ISP driver");
MODULE_AUTHOR("Jerry Mo<jerrymo@meizu.com>");
MODULE_LICENSE("GPL");
