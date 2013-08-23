#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif
#include <linux/completion.h>
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <mach/gpio-common.h>
#include <linux/vmalloc.h>

#include "m6mo.h"
#include "m6mo_regs.h"

/* firmware file */
#define M6MO_FIRMWARE_FILE_NAME "isp_firmware.bin"
#define M6MO_FIRMWARE_FILE_SIZE (2016 * 1024)  /*abount 2 M*/

/* this size for 64KB download */
#define M6MO_SECTION64_WRITE_SIZE (1984 * 1024)
/* this size for 8KB download */
#define M6MO_SECTION8_WRITE_SIZE \
	M6MO_FIRMWARE_FILE_SIZE - M6MO_SECTION64_WRITE_SIZE

#define M6MO_SECTION64_FRAME_SIZE 		(64 * 1024)  /* 64KB */
#define M6MO_SECTION32_FRAME_SIZE 		(32 * 1024)  /* 32KB */
#define M6MO_SECTION8_FRAME_SIZE 		(8 * 1024) /* 8KB */
#define M6MO_SECTION64_FLASH_ADDRESS 	0x10000000
#define M6MO_SECTION8_FLASH_ADDRESS 	0x101f0000
#define M6MO_INTERNAL_RAM_ADDRESS 		0x68000000

/* firmware address stored in file*/
#define NEW_FIRMWARE_HIGH_ADDRESS 		0x0016fffc
#define NEW_FIRMWARE_LOW_ADDRESS 		0x0016fffd

/****************************************/
/***********   factory test functions    **********/
/****************************************/

static bool m6mo_is_factory_test_mode(void)
{
	return !!mx_is_factory_test_mode(MX_FACTORY_TEST_CAMERA);
}

static void m6mo_factory_test_init(void)
{
	pr_debug("%s\n", __func__);
	
	mx_set_factory_test_led(1);
}

static void m6mo_factory_test_success(void)
{
	int onoff = 0;

	pr_info("%s()\n", __func__);

	/*we don't return in factory test mode*/
	while (1) {
		mx_set_factory_test_led(onoff);
		msleep(200);
		onoff = !onoff;
	}
}

static void m6mo_factory_test_fail(void)
{
	pr_err("%s()\n", __func__);
	
	mx_set_factory_test_led(0);
}

/******************************************/

/*
  * set firmware status function
  * NONE, REQUESTING, LOADED_OK or LOADED_FAIL
  * If in factory test mode, should set factory test status
*/
static void m6mo_set_firmware_status(struct v4l2_subdev *sd, 
	enum firmware_status status)
{
	struct m6mo_state *state = to_state(sd);
	bool test_mode = m6mo_is_factory_test_mode();

	switch (status) {
	case FIRMWARE_NONE:
		break;
	case FIRMWARE_REQUESTING:
		if (test_mode) {
			state->fw_updated = false;
			m6mo_factory_test_init();
		}
		break;
	case FIRMWARE_CHECKED:
		if (test_mode) {
			wake_lock(&state->wake_lock);   /* don't sleep and return */
			if (state->fw_updated)
				m6mo_factory_test_success();
			else 
				m6mo_factory_test_fail();
		}
		break;
	case FIRMWARE_UPDATE_FAIL:
		if (test_mode) {
			wake_lock(&state->wake_lock);
			m6mo_factory_test_fail();
		}
		break;
	default:
		return;
	}

	pr_info("%s:status = %d\n", __func__, status);
	
	state->fw_status = status;
}

/*
  * download section data to ISP flash
*/
static int m6mo_download_section_firmware(struct v4l2_subdev *sd, 
	int flashaddress, 
	const char *data, 
	int size, 
	int interval)
{
	u32 val;
	int offset, i, ret, retry_count, count = 0;
	
	for(offset = 0; offset < size; offset += interval) {	
		/* set erase address */
		pr_info("set flash erase address:0x%x\n", flashaddress + offset);	

		ret = m6mo_w32(sd, FLASH_ADDRESS_REG, flashaddress + offset);
		CHECK_ERR(ret);
		
		/* send erase command */
		ret = m6mo_w8(sd, FLASH_ERASE_CMD_REG, 0x01);
		CHECK_ERR(ret);
		
		/* wait for erase finished */
		retry_count = 20;
		while (retry_count--) {  /* abount 300ms */
			ret = m6mo_r8(sd, FLASH_ERASE_CMD_REG, &val);
			CHECK_ERR(ret);	
			
			if(!val)
				break;
			msleep(50);
		}

		if (retry_count <= 0) {
			printk("%s: get FLASH_ERASE_CMD fail\n", __func__);	
			return -EINVAL;
		}

		/* set program bytes */
		ret = m6mo_w16(sd, FLASH_SIZE_REG, 
			(interval == M6MO_SECTION64_FRAME_SIZE)?0:interval);
		CHECK_ERR(ret);
		
		/* clear RAM */
		ret = m6mo_w8(sd, RAM_CLEAN_CMD_REG, 0x01);
		CHECK_ERR(ret);			
		
		/* wait for clear finished */
		retry_count = 100;
		while(retry_count--) {
			ret = m6mo_r8(sd, RAM_CLEAN_CMD_REG, &val);

			if(!val)
				break;
			msleep(10);
		}
		if (retry_count <= 0) {
			pr_err("%s: get FLASH_ERASE_CMD fail\n", __func__);	
			return -EINVAL;
		}

		pr_debug("begin write block data, block size=0x%x\n", interval);
		
		for(i = 0; i < interval; i += M6MO_SECTION8_FRAME_SIZE) {
			ret = m6mo_write_memory(sd, 
				M6MO_INTERNAL_RAM_ADDRESS + i, 
				data + offset + i, 
				M6MO_SECTION8_FRAME_SIZE);
			CHECK_ERR(ret);	
		}
		pr_debug("end write block data, block size=0x%x\n", interval);
		
		/* send flash write command */
		ret = m6mo_w8(sd, FLASH_WRITE_CMD_REG, 0x01);
		CHECK_ERR(ret);		
		
		/* wait for writing flash finished */
		retry_count = 20;
		while(retry_count--) {
			ret = m6mo_r8(sd, FLASH_WRITE_CMD_REG, &val);
			CHECK_ERR(ret);	
			
			if(!val)
				break;
			msleep(50);
		}

		if (retry_count <= 0) {
			pr_err("%s: get FLASH_ERASE_CMD fail\n", __func__);	
			return -EINVAL;
		}
		
		count++;
		pr_info("write count %d##########\n", count);	
	}

	return 0;
}

static int m6mo_download_firmware(struct v4l2_subdev *sd, 
	const char *data, int size)
{
	int ret;
	struct m6mo_state *state = to_state(sd);

	state->fw_buffer = vmalloc(M6MO_SECTION64_FRAME_SIZE + 8);
	if (!state->fw_buffer) return -ENOMEM;
	
	ret = m6mo_w8(sd, CAMERA_FLASH_TYPE_REG, 0x00);
	if (ret) {
		pr_err("%s: set CAMERA_FLASH_TYPE fail\n", __func__);	
		goto exit_free_memory;
	}

	/* download 64KB section */
	ret = m6mo_download_section_firmware(sd, 
		M6MO_SECTION64_FLASH_ADDRESS,
		data, 
		M6MO_SECTION64_WRITE_SIZE, 
		M6MO_SECTION64_FRAME_SIZE);
	if (ret) {
		pr_err("%s: download section 1 fail\n", __func__);	
		goto exit_free_memory;
	}

	/* download 8KB section */
	ret = m6mo_download_section_firmware(sd, 
		M6MO_SECTION8_FLASH_ADDRESS,
		data + M6MO_SECTION64_WRITE_SIZE, 
		M6MO_SECTION8_WRITE_SIZE, 
		M6MO_SECTION8_FRAME_SIZE);
	if (ret)
		pr_err("%s: download section 2 fail\n", __func__);	

exit_free_memory:
	vfree(state->fw_buffer);
	
	return ret;
}


/*
  * check the integrity of the firmware, should do after power on and finishing download firmware  
  * if checksum is ok, checksum value must be zero
*/
static int m6mo_get_checksum(struct v4l2_subdev *sd)
{
	u32 val, chsum;
	int	ret, acc, i, loop = 5;
	u32 chk_addr, chk_size, set_size;
	unsigned short ret_sum = 0;

	chk_addr = M6MO_SECTION64_FLASH_ADDRESS;
	chk_size = M6MO_FIRMWARE_FILE_SIZE;
	acc = 0x02;	/* 16bit unit */

	while (chk_size > 0) {
		if (chk_size >= M6MO_SECTION8_FRAME_SIZE)
			set_size = M6MO_SECTION8_FRAME_SIZE;
		else
			set_size = chk_size;
		
		/* set the start address */
		ret = m6mo_w32(sd, FLASH_ADDRESS_REG, chk_addr);
		CHECK_ERR(ret);
		
		/* set the size for checksum */
		ret = m6mo_w16(sd, FLASH_SIZE_REG, set_size);
		CHECK_ERR(ret);

		/* start to get the checksum */
		ret = m6mo_w8(sd, FLASH_CHKSUM_CMD_REG, acc);
		CHECK_ERR(ret);

		/* wait for getting the checksum */
		for(i = 0; i < loop; i++) {
			msleep(10);
			
			ret = m6mo_r8(sd, FLASH_CHKSUM_CMD_REG, &val);
			CHECK_ERR(ret);

			if (!val) { 
				ret = m6mo_r16(sd, FLASH_CHKSUM_RESULT_REG, &chsum);
				CHECK_ERR(ret);

				ret_sum += chsum;
				break;
			}
		}

		if (i == loop) {
			pr_err("%s(), get checksum failed at %d retries\n",
				__func__, loop);
			return -EINVAL;
		}
		
		/* next iteration */
		chk_addr += set_size;
		chk_size -= set_size;
	}
	
	return ret_sum;
}

/*
  * get current firmware version from ISP
*/
static int m6mo_get_firmware_version(struct v4l2_subdev *sd)
{
	int ret;
	u32 val;

	ret = m6mo_r16(sd, FIRMWARE_MINOR_VER_REG, &val);
	CHECK_ERR(ret);

	return val;
}

/*
  * get firmware version from file
*/
static int m6mo_get_new_firmware_version(const struct firmware *fw)
{
	
	return (fw->data[NEW_FIRMWARE_HIGH_ADDRESS] << 8) 
		| fw->data[NEW_FIRMWARE_LOW_ADDRESS];
}

/*
  * download firmware data to ISP flash
*/
static int m6mo_update_firmware(struct v4l2_subdev *sd, 
	const struct firmware *fw)
{
	int ret;
	struct m6mo_state *state = to_state(sd);

	pr_info("begin to download firmware\n");

	wake_lock(&state->wake_lock);

	ret = m6mo_set_power_clock(state, true);
	if (ret) goto exit_update;

	ret = m6mo_download_firmware(sd, fw->data, fw->size);
	if (ret) goto exit_update;
	
	if((ret = m6mo_get_checksum(sd))) {
		pr_err("%s: get checksum fail, checksum = %d\n",
			__func__, ret);
		ret = -EINVAL;
		goto exit_update;
	}

	if (m6mo_is_factory_test_mode()) state->fw_updated = true;
	
	pr_info("finish downloading firmware !\n");	

exit_update:
	wake_unlock(&state->wake_lock);
	if (state->isp_power) m6mo_set_power_clock(state, false);

	return ret;
}

int m6mo_run_firmware(struct v4l2_subdev *sd)
{
	int ret;

	m6mo_prepare_wait(sd);

	ret = m6mo_w8(sd, CAMERA_START_CMD_REG, 0x01);
	CHECK_ERR(ret);

	ret = m6mo_wait_irq_and_check(sd, INT_MASK_MODE, WAIT_TIMEOUT);
	if (ret) {
		pr_err("%s: wait timeout in %u miliseconds\n", 
			__func__, WAIT_TIMEOUT);
		return ret;
	}
	
	return 0;
}

/*
  * erase ISP firmware function
*/
int m6mo_erase_firmware(struct v4l2_subdev *sd)
{
	int ret;
	u32 val;
	struct m6mo_state *state = to_state(sd);

	/* if has power, return */
	if (state->isp_power) return -EINVAL;

	ret = m6mo_set_power_clock(state, true);
	if (ret) {
		pr_err("%s():power fail\n", __func__);
		return ret;
	}

	/* set flash type */
	ret = m6mo_w8(sd, CAMERA_FLASH_TYPE_REG, 0x00);
	if(ret) goto exit;
	
	/* set erase address */	
	ret =m6mo_w32(sd, FLASH_ADDRESS_REG, M6MO_SECTION64_FLASH_ADDRESS);
	if(ret) goto exit;
	
	/* send chip erase command */
	ret =m6mo_w8(sd, FLASH_ERASE_CMD_REG, 0x02);
	if(ret) goto exit;
	
	/* wait for erase finished */
	
	pr_info("%s: erase wait...\n", __func__);	
	
	while(1) {
		ret = m6mo_r8(sd, FLASH_ERASE_CMD_REG, &val);
		if(ret) goto exit;
		
		if(val == 0x00)
			break;
		msleep(50);
	}
	
	pr_info("%s: chip erase OK!\n", __func__);

exit:
	m6mo_set_power_clock(state, false);
	return ret;
}

/*
  * make a decision for update firmware, return true for update
  * get update decision sequence
  * (1) get new firmware version number from file
  * (2) if in factory test mode, return true, force to update 
  * (2) power on ISP
  * (3) do a checksum, if checksum fail, return true
  * (4) run firmware in order to get current current ISP firmware version
  * (5) compare old and new firmware version number, if not be equal, return true
  * (6) power off
*/
static bool m6mo_get_update_decision(struct v4l2_subdev *sd, const struct firmware *fw)
{
	int old_version, ret;
	struct m6mo_state *state = to_state(sd);
	bool decision = false;

	/* get new firmware version */
	state->fw_version = m6mo_get_new_firmware_version(fw);

	/* if in factory test mode, force to update */
	if (m6mo_is_factory_test_mode()) return true;

	/* power on before get update decision */
	ret = m6mo_set_power_clock(state, true);
	if (ret) goto exit_decision;
	
	msleep(100);  /* delay is necessary for the system boot on ? */

	/* if checksum fail means the firmware is not integrity */
	ret = m6mo_get_checksum(sd);
	if (ret) {
		pr_err("%s:do checksum error, checksum is %d\n",
			__func__, ret);
		decision = true;
		goto exit_decision;
	}

	/* after checksum, check the version num between old and new firmware */
	ret = m6mo_run_firmware(sd);
	if (ret) goto exit_decision;

#ifdef CONFIG_SKIP_CAMERA_UPDATE   /* used for eng */
	decision = false;
	goto exit_decision;
#else
	old_version = m6mo_get_firmware_version(sd);

	pr_info("firmware old version = 0x%x, new version = 0x%x\n", 
		old_version, state->fw_version);

	/* if old version is not equal to new version , we should update firmware */
	if (old_version != state->fw_version) {
		decision = true;
		goto exit_decision;
	}
#endif

exit_decision:
	if (state->isp_power) m6mo_set_power_clock(state, false);
	return decision;
}

static void m6mo_fw_request_complete_handler(const struct firmware *fw,
						  void *context)
{
	int ret = 0, retry_count = 3;
	struct v4l2_subdev *sd = (struct v4l2_subdev *)context;
	enum firmware_status fw_status = FIRMWARE_CHECKED;

	if (!fw) {
		pr_err("load m6mo firmware fail\n");
		goto exit_firmware;
	}

	/* check firmware size */
	if (fw->size != M6MO_FIRMWARE_FILE_SIZE) {
		pr_err("m6mo: firmware incorrect size(%d)\n", fw->size);
		goto exit_firmware;
	}	

	/* get update decision, retry 3 times */
	if (m6mo_get_update_decision(sd, fw)) {
retry:
		ret = m6mo_update_firmware(sd, fw);
		if (ret) {
			if (retry_count--) {
				goto retry;
			} else {
				fw_status = FIRMWARE_UPDATE_FAIL;
				goto exit_firmware;
			}
		}
	}

exit_firmware:
	m6mo_set_firmware_status(sd, fw_status);
}

int m6mo_load_firmware(struct v4l2_subdev *sd)
{
	int ret = 0;	
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	m6mo_set_firmware_status(sd, FIRMWARE_REQUESTING);

	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_HOTPLUG,
				      M6MO_FIRMWARE_FILE_NAME,
				      &client->dev,
				      GFP_KERNEL,
				      sd,
				      m6mo_fw_request_complete_handler);
	if (ret) {
		dev_err(&client->dev, "could not load firmware (err=%d)\n", ret);
		m6mo_set_firmware_status(sd, FIRMWARE_CHECKED);
	}

	return ret;
}

/*
  * load firmware from sys interface: download_firmware
*/
int m6mo_load_firmware_sys(struct device *dev, struct v4l2_subdev *sd)
{
	int ret;
	struct m6mo_state *state = to_state(sd);
	const struct firmware *fw = NULL;
	enum firmware_status fw_status = state->fw_status;

	if (state->isp_power) {
		pr_err("camera has power on, please power off first\n");
		return -EINVAL;
	}

	ret = m6mo_set_power_clock(state, true);
	if (ret) {
		pr_err("%s():power fail", __func__);
		return -EINVAL;
	}

	wake_lock(&state->wake_lock);

	m6mo_set_firmware_status(sd, FIRMWARE_REQUESTING);
	
	ret = request_firmware(&fw, M6MO_FIRMWARE_FILE_NAME, dev);
	if (ret) {
		pr_err("%s() Upload failed. (file %s not found?)\n", __func__, M6MO_FIRMWARE_FILE_NAME);
		goto exit;
	}
	
	pr_info("%s() firmware read %d bytes.\n", __func__, fw->size);

	if (fw->size != M6MO_FIRMWARE_FILE_SIZE) {
		pr_err("m6mo: firmware incorrect size %d\n", fw->size);
		ret = -EINVAL;
		goto exit;
	}	

	state->fw_version = m6mo_get_new_firmware_version(fw);
	
	ret = m6mo_download_firmware(sd, fw->data, fw->size);
	if (ret) {
		pr_err("m6mo: download firmware fail\n");
		fw_status = FIRMWARE_UPDATE_FAIL;
		goto exit;
	}		

	fw_status = FIRMWARE_CHECKED;

exit:
	wake_unlock(&state->wake_lock);
	if (state->isp_power) m6mo_set_power_clock(state, false);
	m6mo_set_firmware_status(sd, fw_status);
	
	return ret;
}
