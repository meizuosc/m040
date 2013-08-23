#include <linux/i2c.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/m6mo.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif
#include <linux/completion.h>
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>

#include "m6mo.h"
#include "m6mo_regs.h"
#include "m6mo_ctrl.h"

  #define MIN_ZOOM_POS 		0x01
  #define MIN_ZOOM_STEP	0x02

#define CHECK_USERSET(v) do {\
	if (state->userset.v == ctrl->value) \
		return 0;\
	} while (0);

#define SET_USERSET(v) do {\
	state->userset.v = ctrl->value;\
	} while (0);

#define CHECK_CTRL_VAL(array) do {\
	if (ctrl->value < 0 || ctrl->value >= ARRAY_SIZE(array)) \
		return -EINVAL;\
	} while (0);
	
#define MAX_JPEG_SIZE (5 * 1024 * 1024)   /* 5 M */
#define MAX_SMILE_PERCENT 100

static u8 m6mo_wb_regs[M6MO_WB_MAX] = {
	[M6MO_WB_INCANDESCENT] = AWB_INCANDESCENT,
	[M6MO_WB_FLUORESCENT_HIGH] = AWB_FLUORESCENT_HIGH,
	[M6MO_WB_FLUORESCENT_LOW] = AWB_FLUORESCENT_LOW,
	[M6MO_WB_SUNNY] = AWB_DAYLIGHT,
	[M6MO_WB_CLOUDY] = AWB_CLOUDY,
	[M6MO_WB_SHADE] = AWB_SHADE,
	[M6MO_WB_HORIZON] = AWB_HORIZON,
};

static u8 m6mo_brightness_regs[M6MO_EV_MAX] = {
	[M6MO_EV_MINUS_2] = EV_M2,
	[M6MO_EV_MINUS_1_5] = EV_M_1_5,
	[M6MO_EV_MINUS_1] = EV_M1,
	[M6MO_EV_MINUS_0_5] = EV_M_0_5,
	[M6MO_EV_DEFAULT] = EV_00,
	[M6MO_EV_PLUS_0_5] = EV_P_0_5,
	[M6MO_EV_PLUS_1] = EV_P1,
	[M6MO_EV_PLUS_1_5] = EV_P_1_5,
	[M6MO_EV_PLUS_2] = EV_P2,
};

static u8 m6mo_scene_regs[M6MO_SCENE_MAX] = {
	[M6MO_SCENE_NONE] = SCENE_OFF,
	[M6MO_SCENE_AUTO] = SCENE_AUTO,
	[M6MO_SCENE_PORTRAIT] = SCENE_PORTRAIT,
	[M6MO_SCENE_LANDSCAPE] = SCENE_LANDSCAPE,
	[M6MO_SCENE_SPORTS] = SCENE_SPORT,
	[M6MO_SCENE_NIGHTSHOT] = SCENE_NIGHT,
	[M6MO_SCENE_SUNSET] = SCENE_SUNSET,
	[M6MO_SCENE_MICRO] = SCENE_MARCO,
	[M6MO_SCENE_CHARACTER] = SCENE_CHARACTER,
};

static u8 m6mo_af_window_regs[M6MO_FOCUS_MAX] = {
	[M6MO_FOCUS_AUTO] = CENTRE_LARGE,
	[M6MO_FOCUS_MACRO] = CENTRE_LARGE,
	[M6MO_FOCUS_MACRO_CAF] = CENTRE_LARGE,
	[M6MO_FOCUS_FD] = BY_FACE_DETECT,
	[M6MO_FOCUS_FD_CAF] = BY_FACE_DETECT,
	[M6MO_FOCUS_TOUCH] = BY_USER,
	[M6MO_FOCUS_TOUCH_CAF] = BY_USER,
	[M6MO_FOCUS_AUTO_CAF] = CENTRE_LARGE,
};

static u8 m6mo_af_scan_mode_regs[M6MO_FOCUS_MAX] = {
	[M6MO_FOCUS_AUTO] = AF_FAST_SCAN,
	[M6MO_FOCUS_MACRO] = AF_FAST_SCAN,
	[M6MO_FOCUS_MACRO_CAF] = AF_CONTINUOUS_FOCUS,
	[M6MO_FOCUS_FD] = AF_FAST_SCAN,
	[M6MO_FOCUS_FD_CAF] = AF_CONTINUOUS_FOCUS,
	[M6MO_FOCUS_TOUCH] = AF_FAST_SCAN,
	[M6MO_FOCUS_TOUCH_CAF] = AF_CONTINUOUS_FOCUS,
	[M6MO_FOCUS_AUTO_CAF] = AF_CONTINUOUS_FOCUS,
};

static u8 m6mo_auto_focus_regs[AUTO_FOCUS_MAX] = {
	[AUTO_FOCUS_OFF] = AF_STOP,
	[AUTO_FOCUS_ON] = AF_START,
};

static u8 m6mo_iso_regs[M6MO_ISO_MAX] = {
	[M6MO_ISO_AUTO] = ISO_SEL_AUTO,
	[M6MO_ISO_50] = ISO_SEL_50,
	[M6MO_ISO_100] = ISO_SEL_100,
	[M6MO_ISO_200] = ISO_SEL_200,
	[M6MO_ISO_400] = ISO_SEL_400,
	[M6MO_ISO_800] = ISO_SEL_800,
	[M6MO_ISO_1600] = ISO_SEL_1600,
	[M6MO_ISO_3200] = ISO_SEL_3200,
};

static u8 m6mo_wdr_regs[M6MO_WDR_MAX] = {
	[M6MO_WDR_LOW] = PART_WDR_LOW,
	[M6MO_WDR_MIDDLE] = PART_WDR_MIDDLE,
	[M6MO_WDR_HIGH] = PART_WDR_HIGH,
};

static u8 m6mo_flash_regs[M6MO_FLASH_MAX] = {
	[M6MO_FLASH_OFF] = LED_FLASH_OFF,
	[M6MO_FLASH_AUTO] = LED_FLASH_AUTO,
	[M6MO_FLASH_ON] = LED_FLASH_ON,
};

/* MAIN_MIRROR, MAIN_REVERSE, MAIN_ROTATION, PREVIEW_ROTATION, THUMB_ROTATION */
static u8 m6mo_rotation_regs[M6MO_ROTATE_MAX][5] = {
	[M6MO_ROTATE_0] = {0x00, 0x00, 0x00, 0x00, 0x00},
	[M6MO_ROTATE_90] = {0x00, 0x00, 0X01, 0X01, 0X01},
	[M6MO_ROTATE_180] = {0x01, 0x01, 0x00, 0x00, 0x00},
	[M6MO_ROTATE_270] = {0x00, 0x00, 0x02, 0x02, 0x02},
};

/*************************************************/
/***********  panorama capture functions  *************/
/*************************************************/
/*
  * set panorama capture start and stop function
  * panorama capture sequence
  * (1) lock ae and awb
  * (1) set panorama mode on
  * (2) enable all interrupt
  * (3) set panorama capture start
  * (4) wait every picture finishing interrupt
  * (5) wait all picture finishing interrupt
  * (6) wait all picture stitch interrupt
*/
int m6mo_set_panorama_capture(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct m6mo_state *state = to_state(sd);
	
	if (ctrl->value) {
		int i, ret;
		
		if (state->cap_mode == CAP_PANORAMA_MODE) return 0;

		state->cap_mode = CAP_PANORAMA_MODE;
		state->pano.counter = 0;
		state->pano.stitch_status = PANORAMA_STITCH_INIT;
		/* initialize all panorama picture info */
		for (i = 0; i < PANORAMA_MAX_PICTURE; i++) {
			state->pano.pictures[i].status = PANORAMA_UNKNOWN_ERR;
			state->pano.pictures[i].extra = 0;
		}
		m6mo_prepare_wait(sd);

		/* set multi-cap regs */
		do {
			struct m6mo_reg regs[] = {
				{I2C_8BIT, AE_LOCK_REG, 0x01},
				{I2C_8BIT, AWB_LOCK_REG, 0x01},
				{I2C_8BIT, CAP_MODE_REG, CAP_MODE_PARORAMA},
				{I2C_8BIT, PANO_CTRL_REG, 0x00},
				{I2C_8BIT, INT_ENABLE_REG, 0xff},
				{I2C_8BIT, INT_ROOR_ENABLE_REG, 0x01},
				{I2C_8BIT, PANO_CAP_READY_REG, PANO_CAP_READY_START},  /* start capture */
			};
			
			ret = m6mo_write_regs(sd, regs, ARRAY_SIZE(regs));
			if (ret) {
				state->cap_mode = CAP_NORMAL_MODE; /* recovery normal mode */
				return ret;
			}
		} while (0);

		pr_info("%s():start panorama capture.\n", __func__);
	} else {
		pr_info("%s():stop panorama capture.\n", __func__);
		
		if (state->cap_mode == CAP_PANORAMA_MODE)
			state->cap_mode = CAP_NORMAL_MODE;
	}
	
	return 0;
}

static void set_pano_picture_status(struct m6mo_state *state, int index,
	enum panorama_picture_status status)
{
	struct v4l2_subdev *sd = &state->sd;
	int ret;

	/*lock first*/
	mutex_lock(&state->mutex);
	
	switch (status) {
	case PANORAMA_SUCCESS:
	case PANORAMA_FATAL_ERR:
	case PANORAMA_UNKNOWN_ERR:
	case PANORAMA_COMPLETE:
		break;
	case PANORAMA_RETRY_ERR:
		do {
			int x, y;
			ret = m6mo_r16(sd, PANO_OFFSETX_H_REG, &x);
			if (ret) goto exit_unlock_mutex;
			ret = m6mo_r16(sd, PANO_OFFSETY_H_REG, &y);
			if (ret) goto exit_unlock_mutex;
			state->pano.pictures[index].extra = ((x & 0xffff) << 16) | (y & 0xffff);
		} while (0);
		break;
	case PANORAMA_BIG_ERR:
		do {
			int val;
			ret = m6mo_r8(sd, PANO_ERROR_NO_REG, &val);
			if (ret) goto exit_unlock_mutex;
			pr_info("%s(), big error found, err: %d\n", __func__, (signed char)val);
			state->pano.pictures[index].extra = val;
		} while (0);
		break;
	default:
		return;
	}

	state->pano.pictures[index].status = status;

exit_unlock_mutex:
	mutex_unlock(&state->mutex);
}

static void set_pano_stitch_status(struct m6mo_state *state)
{
	int ret;
	u32 val;
	
	ret = m6mo_r8(&state->sd, PANO_ERROR_NO_REG,  &val);
	if(ret) return;

	mutex_lock(&state->mutex);
	
	if (!val) {
		state->pano.stitch_status = PANORAMA_STITCH_OK;
		printk("stitching all images success\n");
	} else {
		state->pano.stitch_status = PANORAMA_STITCH_FAIL;
		printk("stitching all images fail, 0x%02x reg is 0x%02x.\n", PANO_ERROR_NO_REG, val);
	}
	
	mutex_unlock(&state->mutex);
}

void m6mo_handle_panorama_cap(struct m6mo_state *state, int irq_status)
{
	if (irq_status & INT_MASK_ZOOM) {   /* stitch interrupt */
		set_pano_stitch_status(state);
		complete(&state->completion);
	} else if (irq_status & INT_MASK_MODE) {  /* finish all capture interrupt */
		set_pano_picture_status(state, state->pano.counter++, PANORAMA_COMPLETE);
		pr_info("Finish panorama capture!\n");		
	} else if (irq_status & INT_MASK_CAPTURE) {   /* finish one capture interrupt */
		if (state->pano.counter >= PANORAMA_MAX_PICTURE) {
			set_pano_picture_status(state, state->pano.counter, PANORAMA_FATAL_ERR);
			pr_err("***panorama picture counter is overflow***\n");
		} else {
			set_pano_picture_status(state, state->pano.counter++, PANORAMA_SUCCESS);
			pr_info("Valid %d-th image captured.\n", state->pano.counter);
		}
	} else if (irq_status & INT_MASK_FRAMESYNC) {  /* mirror error */
		set_pano_picture_status(state, state->pano.counter, PANORAMA_RETRY_ERR);
	} else if (irq_status & INT_MASK_FD) {   /* big error */
		set_pano_picture_status(state, state->pano.counter, PANORAMA_BIG_ERR);
	} else if (irq_status & INT_MASK_SOUND) {  /* fatal error */
		set_pano_picture_status(state, state->pano.counter, PANORAMA_FATAL_ERR);
		pr_err("Fatal error found in captured image!\n");
	} else {   /* unknown error */
		set_pano_picture_status(state, state->pano.counter, PANORAMA_UNKNOWN_ERR);
		pr_err("%s():other interrupt status in panorama capture process!0x%02x\n", 
			__func__, irq_status);
	}
}

static int m6mo_wait_panorama_stitch(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct m6mo_state *state = to_state(sd);

	mutex_lock(&state->mutex);

	if (state->pano.stitch_status == PANORAMA_STITCH_INIT) {
		int ret;
		mutex_unlock(&state->mutex);
		ret = m6mo_wait_irq(sd, 8000);
		if (ret) return ret;
		mutex_lock(&state->mutex);
	}

	ctrl->value = state->pano.stitch_status;
	
	mutex_unlock(&state->mutex);
	
	return 0;
}

/*
  * terminate panorama capture manual
*/
static int m6mo_terminate_panorama(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return m6mo_w8(sd, PANO_CAP_READY_REG, PANO_CAP_READY_STOP);
}

/*
  * get current panorama picture information, should be modified in future
*/
static int m6mos_set_cur_panorama_info(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct m6mo_state *state = to_state(sd);
	int index = ctrl->value;
	int ret;

	if (index < 1 || index > PANORAMA_MAX_PICTURE) {
		pr_err("wrong pan num: %d!!!\n", index);
		return -EINVAL;
	}

	index--;
	mutex_lock(&state->mutex);
	ret = state->pano.pictures[index].status;
	ctrl->value = state->pano.pictures[index].extra;
	mutex_unlock(&state->mutex);
	
	return ret;
}

static int m6mo_get_pan_direction(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err, value;

	err = m6mo_r8(sd, PANO_CAP_DIRECTION_REG, &value);
	if (!err) ctrl->value = value;
	return err;
}

/*************************************************/
/***********  multi capture functions  ****************/
/*************************************************/
/*
  * set multi capture start or stop function
  * multi capture seqence
  * (1) set multi cap number 
  * (2) set auto multi cap mode
  * (3) enable interrupt
  * (4) start capture
  * (5) wait every multi cap picture interrupt until finish capture
  * (6) get every multi cap pictures
*/
static int m6mo_set_multi_capture(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{

	struct m6mo_state *state = to_state(sd);
	int ret = 0;

	if (ctrl->value) {
		int i;
		
		pr_info("multi capture start, number is %d\n", ctrl->value);

		/* check ctrl value */
		if (ctrl->value > MULTI_CAP_MAX_PICTURE) return -EINVAL;
		
		if (state->cap_mode == CAP_MULTI_CAP_MODE) return 0;

		state->multi_cap.numbers = ctrl->value;  /* set multi-cap picture number */
		state->multi_cap.counter = 0;  /* reset counter */
		state->multi_cap.ready = MULTI_CAP_READY_INIT;
		/* initialize all multi-picture information */
		for (i = 0; i < state->multi_cap.numbers; i++) 
			state->multi_cap.pictures[i].status = MULTI_CAP_INIT;
		m6mo_prepare_wait(sd);

		/* set multi-cap regs */
		ret = m6mo_w8(sd, CAP_FRM_COUNT_REG, state->multi_cap.numbers);
		CHECK_ERR(ret);
		ret = m6mo_w8(sd, CAP_MODE_REG, CAP_MODE_AUTO_MULTICAP);
		CHECK_ERR(ret);
		ret = m6mo_set_sys_mode(sd, CAPTURE_MODE);
		if (ret) return ret;
		state->cap_mode = CAP_MULTI_CAP_MODE;
	} else {
		if (state->cap_mode == CAP_MULTI_CAP_MODE)
			state->cap_mode = CAP_NORMAL_MODE;  /* recover capture normal mode */
	}

	return 0;
}

/*
  * handle multi capture interrupt function
*/
void m6mo_handle_multi_cap(struct m6mo_state *state, int irq_status)
{
	struct v4l2_subdev *sd = &state->sd;
	
	pr_info("%s: irq status = 0x%02x\n", __func__, irq_status);

	/* has wait all multi capture pictures, wait ready flag */
	if (state->multi_cap.counter >= state->multi_cap.numbers) {
		mutex_lock(&state->mutex);
		if (irq_status & INT_MASK_CAPTURE)
			state->multi_cap.ready = MULTI_CAP_READY_SUCCESS;
		else 
			state->multi_cap.ready = MULTI_CAP_READY_FAIL;
		mutex_unlock(&state->mutex);
		complete(&state->completion);
	} else if ((irq_status & INT_MASK_SOUND) && 
				(irq_status & INT_MASK_FRAMESYNC)) {  /* wait the i-th picture success */
		mutex_lock(&state->mutex);
		state->multi_cap.pictures[state->multi_cap.counter++].status 
			= MULTI_CAP_SUCCESS;
		mutex_unlock(&state->mutex);

		pr_info("wait %d-th multi capture picture success.\n", state->multi_cap.counter);
		
		m6mo_enable_root_irq(sd);
		complete(&state->completion);
	} else if (irq_status & INT_MASK_MODE) {
		pr_info("change to multi-cap mode success !");
	}
}

/*
  * wait finishing the i-th multi-cap
  * if it has been finished, it's status is not equal to MULTI_CAP_INIT
*/
static int m6mo_wait_multi_capture_picture(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int i, ret;
	struct m6mo_state *state = to_state(sd);
	
	if (ctrl->value  < 1 || ctrl->value > state->multi_cap.numbers) {
		pr_err("%s():ctrl->value %d is out of range[1-%d].", 
			__func__, ctrl->value, state->multi_cap.numbers);
		return -EINVAL;
	}
	
	i = ctrl->value - 1;  /* get the multi cap index */

	mutex_lock(&state->mutex);

	/* 
	  * if this picture has not come, try to wait
	  * else the picture has come
	*/
	if (state->multi_cap.pictures[i].status == MULTI_CAP_INIT) {
		mutex_unlock(&state->mutex);  /* free the lock first */
		ret = m6mo_wait_irq(sd, WAIT_TIMEOUT);
		if (ret) return ret;
		mutex_lock(&state->mutex);
	}

	/* set the picture status */
	ctrl->value = state->multi_cap.pictures[i].status;

	pr_info("%s: status = %d\n", __func__, state->multi_cap.pictures[i].status);

	mutex_unlock(&state->mutex);

	return 0;
}

static int m6mo_wait_multi_capture_ready(struct v4l2_subdev *sd, 
	struct v4l2_control *ctrl)
{
	int ret;
	struct m6mo_state *state = to_state(sd);

	mutex_lock(&state->mutex);

	/* this multi-cap picture is not ready */
	if (state->multi_cap.ready == MULTI_CAP_READY_INIT) {
		mutex_unlock(&state->mutex);
		ret = m6mo_wait_irq(sd, WAIT_TIMEOUT);
		if (ret) return ret;
		mutex_lock(&state->mutex);
	}

	/* get the status */
	ctrl->value = state->multi_cap.ready;

	pr_info("%s: ready = %d\n", __func__, state->multi_cap.ready);

	mutex_unlock(&state->mutex);
	
	return 0;
}

/*
  * get the i-th mul-cap picture functions
  * sequence
  * (1) enable root irq
  * (2) select frame image
  * (3) wait irq 
  * (4) enable root irq
  * (5) set transfer start
  * (6) wait irq
*/
static int m6mo_get_multi_capture_picture(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret;
	struct m6mo_state *state = to_state(sd);

	pr_info("%s(), get %d picture of multi-cap\n", __func__, ctrl->value);

	/* check ctrl value */
	if (ctrl->value < 1 || ctrl->value > state->multi_cap.numbers) {
		pr_err("%s():ctrl->value %d is out of range[1-%d]", 
			__func__, ctrl->value, state->multi_cap.numbers);
		return -EINVAL;
	}

	m6mo_prepare_wait(sd);
	ret = m6mo_enable_root_irq(sd);
	CHECK_ERR(ret);
	ret = m6mo_w8(sd, CAP_SEL_FRAME_MAIN_REG, ctrl->value);
	CHECK_ERR(ret);
	ret = m6mo_wait_irq_and_check(sd, INT_MASK_CAPTURE, WAIT_TIMEOUT);
	if (ret) return ret;
	
	ret = m6mo_enable_root_irq(sd);
	CHECK_ERR(ret);
	ret = m6mo_w8(sd, CAP_TRANSFER_START_REG, CAP_TRANSFER_MAIN);
	CHECK_ERR(ret);
	ret = m6mo_wait_irq_and_check(sd, INT_MASK_CAPTURE, WAIT_TIMEOUT);
	if (ret) return ret;

	/*set ctrl value to 0 means get picture success*/
	ctrl->value = 0;
	
	return 0;
}

/*****************************************/
/********  face detection capture ************/
/*****************************************/
/*
  * enable smile face detection function
  * smile face capture sequence
  * (1) set face detection on
  * (2) set smile level [0~100]
  * (3) wait smile detection interrupt
*/
static int m6mo_set_smile_face_detection(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret;
	struct m6mo_state *state = to_state(sd);

	if (ctrl->value < 0 || ctrl->value > 100) {
		pr_err("wrong smile level value!please set[0-100]\n");
		return -EINVAL;
	}

	if (ctrl->value) {
		pr_info("%s():smile face detection enable.\n", __func__);
		state->smile_cap.detection = SMILE_NO_DETECTION;
		state->cap_mode = CAP_SMILE_CAP_MODE;
		ret = m6mo_w8(sd, FACE_DETECT_MAX_REG, 0x0b);
		CHECK_ERR(ret);
		ret = m6mo_w8(sd, FACE_DETECT_CTL_REG, SMILE_FACE_DETECT_ON);
		CHECK_ERR(ret);
		ret = m6mo_w8(sd, FD_SMILE_LEVEL_THRS_REG, ctrl->value);
		CHECK_ERR(ret);
	} else {
		pr_info("%s():smile face detection disable.\n", __func__);
		ret = m6mo_w8(sd, FD_SMILE_LEVEL_THRS_REG, 0x00);
		CHECK_ERR(ret);
		ret = m6mo_w8(sd, FACE_DETECT_CTL_REG, SMILE_FACE_DETECT_OFF);
		CHECK_ERR(ret);
		state->cap_mode = CAP_NORMAL_MODE;
	}
	
	return 0;
}

/*
  * handle smile interrupt function
*/
void m6mo_handle_smile_cap(struct m6mo_state *state, int irq_status)
{
	if (irq_status & INT_MASK_FRAMESYNC) {
		pr_info("*********%s():smile detection********\n", __func__);
		state->smile_cap.detection = SMILE_DETECTION;
		complete(&state->completion);
	}
}

static int m6mo_get_smile_face_detection(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct m6mo_state *state = to_state(sd);
	
	ctrl->value = state->smile_cap.detection;
	
	return 0;
}

static int m6mo_get_face_detected_num(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int val, ret;

	ret = m6mo_r8(sd, FACE_DETECT_NUM_REG, &val);
	CHECK_ERR(ret);

	ctrl->value = val;
	
	return ret;
}

static int m6mo_set_face_selection(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int val, ret;
	int retry_count = 100;
	
	ret =  m6mo_w8(sd, FACE_DETECT_READ_SEL_REG, ctrl->value);
	CHECK_ERR(ret);

	while (retry_count--) {
		ret = m6mo_r8(sd, FACE_DETECT_READ_SEL_REG, &val);
		if (!ret && val == 0xff) 
			return 0;
		msleep(5);
	}

	return -EINVAL;
}

static int m6mo_get_selected_face_location(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret, x, y;

	ret = m6mo_r16(sd, FACE_DETECT_X_LOCATION_REG, &x);
	CHECK_ERR(ret);
	ret = m6mo_r16(sd, FACE_DETECT_Y_LOCATION_REG, &y);
	CHECK_ERR(ret);
	
	ctrl->value = ((x & 0xffff) << 16) | (y & 0xffff);
	
	return 0;
}

static int m6mo_get_selected_face_size(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret, w, h;

	ret = m6mo_r16(sd, FACE_DETECT_FRAME_WIDTH_REG, &w);
	CHECK_ERR(ret);
	ret = m6mo_r16(sd, FACE_DETECT_FRAME_HEIGH_REG, &h);
	CHECK_ERR(ret);
	
	ctrl->value = ((w & 0xffff) << 16) | (h & 0xffff);
	
	return 0;
}

/*
  * set face detection direction, 0, 90, 180 or 270 angle
*/
static int m6mo_set_face_detection_direction(struct v4l2_subdev *sd, 
	struct v4l2_control *ctrl)
{
	return m6mo_w8(sd, FACE_DETECT_DIRECTION_REG, ctrl->value);
}

static int m6mo_get_jpeg_mem_size(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	ctrl->value = MAX_JPEG_SIZE;
	
	return 0;
}

static void m6mo_get_af_touch_coordinate(int val, int *x, int *y)
{
	int row, col;

	row = val / AF_TOUCH_ROW;
	col = val - row * AF_TOUCH_ROW;
	
	if(row > AF_TOUCH_ROW - 1)
		row = AF_TOUCH_ROW - 1;
	if(row < 0)
		row = 0;
	if(col > AF_TOUCH_COL - 1)
		col = AF_TOUCH_COL - 1;
	if(col < 0)
		col = 0;

	pr_info("%s(): row %d, col %d.\n", __func__, row, col);
	
	/*x stand for col, y stand for row*/
	*x = col * AF_TOUCH_WIDTH;
	*y = row * AF_TOUCH_HEIGHT;
}

static int m6mo_set_focus_position(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct m6mo_state *state = to_state(sd);	
	int x, y;

	CHECK_USERSET(focus_position);

	m6mo_get_af_touch_coordinate(ctrl->value, &x, &y);
	
	ret = m6mo_w16(sd, AF_TOUCH_WIN_W_REG, AF_TOUCH_WIDTH);
	CHECK_ERR(ret);
	ret = m6mo_w16(sd, AF_TOUCH_WIN_H_REG, AF_TOUCH_HEIGHT);	
	CHECK_ERR(ret);
	
	ret = m6mo_w16(sd, AF_TOUCH_WIN_X_REG, x);
	CHECK_ERR(ret);
	ret = m6mo_w16(sd, AF_TOUCH_WIN_Y_REG, y);
	CHECK_ERR(ret);

	SET_USERSET(focus_position);
	
	return 0;
}

static int m6mo_set_wb_preset(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct m6mo_state *state = to_state(sd);
	int ret = 0;

	CHECK_CTRL_VAL(m6mo_wb_regs);
	CHECK_USERSET(manual_wb);

	/* set manual wb first */
	if(ctrl->value == M6MO_WB_AUTO) {
		ret = m6mo_w8(sd, AWB_MODE_REG, AWB_AUTO);
	} else {
		ret = m6mo_w8(sd, AWB_MODE_REG, AWB_MANUAL);	
		CHECK_ERR(ret);
		ret = m6mo_w8(sd, AWB_MANUAL_REG, m6mo_wb_regs[ctrl->value]);
		CHECK_ERR(ret);
	}
	
	SET_USERSET(manual_wb);
	
	return 0;
}

static int m6mo_set_image_brightness(struct v4l2_subdev *sd, 
	struct v4l2_control *ctrl)
{
	struct m6mo_state *state = to_state(sd);
	int ret = 0;

	CHECK_CTRL_VAL(m6mo_brightness_regs);
	CHECK_USERSET(brightness);

	pr_info("%s(), wirte 0x%x to register 0x%x\n", __func__,
		m6mo_brightness_regs[ctrl->value], EV_BIAS_REG);

	ret = m6mo_w8(sd, EV_BIAS_REG, m6mo_brightness_regs[ctrl->value]);
	CHECK_ERR(ret);

	SET_USERSET(brightness);
	
	return 0;
}

static int m6mo_set_scenemode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct m6mo_state *state = to_state(sd);

	CHECK_CTRL_VAL(m6mo_scene_regs);
	CHECK_USERSET(scene);

	ret = m6mo_w8(sd, SCENE_MODE_REG, m6mo_scene_regs[ctrl->value]);
	CHECK_ERR(ret);

	SET_USERSET(scene);
 
	return 0;
}

static int m6mo_set_focus(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;

	CHECK_CTRL_VAL(m6mo_auto_focus_regs);

	ret = m6mo_w8(sd, AF_START_REG, m6mo_auto_focus_regs[ctrl->value]);
	CHECK_ERR(ret);

	return 0;
}

static int m6mo_set_focus_mode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct m6mo_state *state = to_state(sd);

	CHECK_USERSET(af_mode);

	if (ctrl->value < M6MO_FOCUS_AUTO || ctrl->value >= M6MO_FOCUS_MAX)
		return -EINVAL;

	ret = m6mo_w8(sd, AF_WINDOW_REG, m6mo_af_window_regs[ctrl->value]);
	pr_info("%s(), ctrl->value %d, write %d to reg\n",
		__func__, ctrl->value, m6mo_af_window_regs[ctrl->value]);
	CHECK_ERR(ret);
	ret = m6mo_w8(sd, AF_SCAN_MODE_REG, m6mo_af_scan_mode_regs[ctrl->value]);
	CHECK_ERR(ret);

	SET_USERSET(af_mode);

	return 0;
}

static int m6mo_set_zoom_level(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct m6mo_state *state = to_state(sd);

	CHECK_USERSET(zoom_level);

	if (ctrl->value < M6MO_ZL_1)
		ctrl->value = M6MO_ZL_1;
	if(ctrl->value > M6MO_ZL_70)
		ctrl->value = M6MO_ZL_70;
	
	ret = m6mo_w8(sd, ZOOM_POSITOIN_REG, ctrl->value);
	CHECK_ERR(ret);

	SET_USERSET(zoom_level);
	
	return 0;
}

static int m6mo_set_iso(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct m6mo_state *state = to_state(sd);

	CHECK_CTRL_VAL(m6mo_iso_regs);
	CHECK_USERSET(iso);

	ret = m6mo_w8(sd, ISO_SEL_REG, m6mo_iso_regs[ctrl->value]);
	CHECK_ERR(ret);

	SET_USERSET(iso);
	
	return 0;
}

static int m6mo_set_wdr(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct m6mo_state *state = to_state(sd);

	CHECK_CTRL_VAL(m6mo_wdr_regs);
	CHECK_USERSET(wdr);

	pr_info("%s(), set New WDR to %d, the last is %d", __func__,
		ctrl->value, state->userset.wdr);

	if (ctrl->value == M6MO_WDR_OFF)
		ret = m6mo_w8(sd, PART_WDR_EN_REG, PART_WDR_OFF);
	else if (ctrl->value == M6MO_WDR_AUTO)
		ret = m6mo_w8(sd, PART_WDR_EN_REG, PART_WDR_AUTO);
	else {
		ret = m6mo_w8(sd, PART_WDR_EN_REG, PART_WDR_ON);
		CHECK_ERR(ret);
		ret = m6mo_w8(sd, PART_WDR_LVL_REG, m6mo_wdr_regs[ctrl->value]);
	}
	CHECK_ERR(ret);

	SET_USERSET(wdr);
	
	return 0;
}

int m6mo_set_flash_current(struct m6mo_state *state, int cur)
{
	if ((!state->fled_regulator) || (cur > MAX_FLASH_CURRENT)) 
		return -EINVAL;

	return regulator_set_current_limit(state->fled_regulator, cur, MAX_FLASH_CURRENT);
}

static int m6mo_select_flash_led(struct m6mo_state *state)
{
	int ret;
	
	if (machine_is_m030())
		ret = m6mo_w8(&state->sd, FLASHLED_SELECT_REG, FLASHLED_M030);
	else
		ret = m6mo_w8(&state->sd, FLASHLED_SELECT_REG, FLASHLED_M03X);
	return ret;
}

static int m6mo_enable_flash_led(struct m6mo_state *state, struct v4l2_control *ctrl) 
{
	int ret = 0;
	
	if (ctrl->value != M6MO_FLASH_OFF) {
		/* if the first time to open flash led */
		if (!state->fled_regulator) {
			pr_info("turn on flash led!\n");

			/* select flash led category */
			ret = m6mo_select_flash_led(state);
			if (ret) return ret;

			/* open the flash voltage */
			state->fled_regulator = regulator_get(NULL, FLASH_LED_NAME);
			if (IS_ERR(state->fled_regulator)) {
				pr_err("%s()->%d:regulator get fail !!\n", __FUNCTION__, __LINE__);
				state->fled_regulator = NULL;
				return -ENODEV;
			}
		
			ret = m6mo_set_flash_current(state, state->pre_flash_current);
			if (ret) goto err_exit;
			ret = regulator_enable(state->fled_regulator);
			if (ret) goto err_exit;
		}
	} else {
		if (state->fled_regulator) {
			pr_info("turn off flash led!\n");
			
			ret = regulator_disable(state->fled_regulator);	
			if (ret) goto err_exit;
			regulator_put(state->fled_regulator);
			state->fled_regulator = NULL;
		}
	}
	
	return 0;
	
err_exit:
	regulator_put(state->fled_regulator);
	state->fled_regulator = NULL;
	return ret;
}

static int m6mo_set_flash_mode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct m6mo_state *state = to_state(sd);

	CHECK_CTRL_VAL(m6mo_flash_regs);
	CHECK_USERSET(flash_mode);

	ret = m6mo_w8(sd, LED_FLASH_CONTROL_REG, m6mo_flash_regs[ctrl->value]);
	CHECK_ERR(ret);
	
	ret = m6mo_enable_flash_led(state, ctrl);
	if (ret) return ret;

	SET_USERSET(flash_mode);
	
	return 0;
}

static int m6mo_set_rotation(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct m6mo_state *state = to_state(sd);
	int ret = 0;

	struct m6mo_reg regs[] = {
		{I2C_8BIT, MAIN_MIRROR_REG, m6mo_rotation_regs[ctrl->value][0]},
		{I2C_8BIT, MAIN_REVERSE_REG, m6mo_rotation_regs[ctrl->value][1]},
		{I2C_8BIT, MAIN_ROTATION_REG, m6mo_rotation_regs[ctrl->value][2]},
		{I2C_8BIT, PREVIEW_ROTATION_REG, m6mo_rotation_regs[ctrl->value][3]},
		{I2C_8BIT, THUMB_ROTATION_REG, m6mo_rotation_regs[ctrl->value][4]},
	};

	CHECK_CTRL_VAL(m6mo_rotation_regs);
	CHECK_USERSET(rotation);

	ret = m6mo_write_regs(sd, regs, ARRAY_SIZE(regs));
	CHECK_ERR(ret);

	SET_USERSET(rotation);
	
	return 0;
}

static int m6mo_get_scene_ev(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return m6mo_r16(sd, SCENE_EV_REG, &ctrl->value);
}

/*
  * get auto focus result (0-operating, 1-success, 2-fail, 3-stopped at edge)
*/
static int m6mo_get_auto_focus_result(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return m6mo_r8(sd, AF_RESULT_REG, &ctrl->value);
}

static int m6mo_transfer_capture_data(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	pr_info("%s(), ctr->value is %d\n", __func__, ctrl->value);

	switch (ctrl->value) {
	case 0:
		break;
	case 1:
		m6mo_prepare_wait(sd);

		ret = m6mo_w8(sd, CAP_TRANSFER_START_REG, CAP_TRANSFER_MAIN);
		CHECK_ERR(ret);
		
		ret = m6mo_wait_irq_and_check(sd, INT_MASK_CAPTURE, WAIT_TIMEOUT);  /* wait interrupt */
		if (ret) return ret;
		break;
	default:
		return -EINVAL;
	}
	
	return ret;
}

static int m6mo_start_capture(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return m6mo_set_mode(sd, CAPTURE_MODE);
}

static int m6mo_wakeup_preview(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	/* to let poll exit in hal layer preview thread */
	fimc_wakeup_preview(); 

	return 0;
}

static int m6mo_get_jpeg_main_size(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	
	ret = m6mo_r32(sd, JPEG_IMAGE_SIZE_REG,  &ctrl->value);
	CHECK_ERR(ret);
	
	pr_info("%s():jpeg size is %d.\n", __func__, ctrl->value);
	WARN(0 == ctrl->value, "JPEG Image size is 0!\n");
	
	return 0;
}

static int m6mo_set_reverse(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct m6mo_state *state = to_state(sd);
	int ret;

	CHECK_USERSET(reverse);
	
	ret = m6mo_w8(sd, MON_REVERSE_ISP_REG, !!ctrl->value);
	CHECK_ERR(ret);

	SET_USERSET(reverse);

	return 0;
}

static int m6mo_set_mirror(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct m6mo_state *state = to_state(sd);
	int ret;

	CHECK_USERSET(mirror);
	
	ret = m6mo_w8(sd, MON_MIRROR_ISP_REG, !!ctrl->value);
	CHECK_ERR(ret);
	
	SET_USERSET(mirror);

	return 0;
}

static int m6mo_set_colorbar(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return m6mo_w8(sd, COLOR_BAR_REG, ENABLE_COLOR_BAR);
}

static int m6mo_s_cap_format(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct v4l2_mbus_framefmt fmt;

	switch (ctrl->value) {
	case V4L2_PIX_FMT_YUYV:
		fmt.code = V4L2_MBUS_FMT_VYUY8_2X8;
		break;
	case V4L2_PIX_FMT_JPEG:
		fmt.code = V4L2_MBUS_FMT_JPEG_1X8;
		break;
	default:
		return -EINVAL;
	}

	return m6mo_set_capture_format(sd, &fmt);
}

static int m6mo_s_capture_size(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct v4l2_mbus_framefmt fmt;
	
	fmt.width = (ctrl->value >> 16) & 0xffff;
	fmt.height = ctrl->value & 0xffff;

	return m6mo_set_capture_size(sd, &fmt);
}

/*
* For light info, from which we can know whether light on is needed.
*/
static int m6mo_get_light_info(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	m6mo_r8(sd, INFO_LIGHT_REG, &ctrl->value);
	pr_info("%s(), reg value is %d\n", __func__, ctrl->value);
	return ctrl->value;
}

/*************************************************/
/***********  exif information functions  ****************/
/*************************************************/

static int m6mo_get_exif_exptime_n(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r32(sd, INFO_EXPTIME_NUMERATOR_REG, &ctrl->value);
}

static int m6mo_get_exif_exptime_d(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r32(sd, INFO_EXPTIME_DENUMINATOR_REG, &ctrl->value);
}

static int m6mo_get_exif_tv_n(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r32(sd, INFO_TV_NUMERATOR_REG, &ctrl->value);
}

static int m6mo_get_exif_tv_d(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r32(sd, INFO_TV_DENUMINATOR_REG, &ctrl->value);
}

static int m6mo_get_exif_av_n(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r32(sd, INFO_AV_NUMERATOR_REG, &ctrl->value);
}

static int m6mo_get_exif_av_d(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r32(sd, INFO_AV_DENUMINATOR_REG, &ctrl->value);
}

static int m6mo_get_exif_bv_n(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r32(sd, INFO_BV_NUMERATOR_REG, &ctrl->value);
}

static int m6mo_get_exif_bv_d(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r32(sd, INFO_BV_DENUMINATOR_REG, &ctrl->value);
}

static int m6mo_get_exif_ebv_n(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r32(sd, INFO_EBV_NUMERATOR_REG, &ctrl->value);
}

static int m6mo_get_exif_ebv_d(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r32(sd, INFO_EBV_DENUMINATOR_REG, &ctrl->value);
}

static int m6mo_get_exif_iso_ext(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r16(sd, INFO_ISO_REG, &ctrl->value);
}

static int m6mo_get_exif_iso(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return m6mo_r16(sd, INFO_ISO_REG, &ctrl->value);
}

static int m6mo_get_exif_flash(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r16(sd, INFO_FLASH_REG, &ctrl->value);
}

static int m6mo_get_exif_sdr(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r16(sd, INFO_SDR_REG, &ctrl->value);
}

static int m6mo_get_exif_qval(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return m6mo_r16(sd, INFO_QVAL_REG, &ctrl->value);
}

/*********************************************/

static int m6mo_g_ext_ctrl(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	int ret = 0;
	
	switch (ctrl->id) {
	case V4L2_CTRL_CLASS_CAMERA_REGISTER:
		ret = m6mo_read_reg(sd, (u16)ctrl->value, &ctrl->value, ctrl->size);
		break;
	case V4L2_CTRL_CLASS_CAMERA_EXPTIME_N:
		ret = m6mo_get_exif_exptime_n(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_EXPTIME_D:
		ret = m6mo_get_exif_exptime_d(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_TV_N:
		ret = m6mo_get_exif_tv_n(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_TV_D:
		ret = m6mo_get_exif_tv_d(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_AV_N:
		ret = m6mo_get_exif_av_n(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_AV_D:
		ret = m6mo_get_exif_av_d(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_BV_N:
		ret = m6mo_get_exif_bv_n(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_BV_D:
		ret = m6mo_get_exif_bv_d(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_EBV_N:
		ret = m6mo_get_exif_ebv_n(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_EBV_D:
		ret = m6mo_get_exif_ebv_d(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_ISO:
		ret = m6mo_get_exif_iso_ext(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_FLASH:
		ret = m6mo_get_exif_flash(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_SDR:
		ret = m6mo_get_exif_sdr(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_QV:
		ret = m6mo_get_exif_qval(sd, ctrl);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

int m6mo_g_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	struct v4l2_ext_control *ctrl = ctrls->controls;
	int i, ret = 0;

	for (i = 0; i < ctrls->count; i++, ctrl++) {
		ret = m6mo_g_ext_ctrl(sd, ctrl);
		if (ret) {
			ctrls->error_idx = i;
			break;
		}
	}
	return ret;
}

/*exif informaion*/
static int m6mo_get_exif_exptime(struct v4l2_subdev *sd, struct v4l2_control *ctrl, int flag)
{
	if (flag) 
		return m6mo_r32(sd, INFO_EXPTIME_DENUMINATOR_REG, &ctrl->value);
	else 
		return m6mo_r32(sd, INFO_EXPTIME_NUMERATOR_REG, &ctrl->value);
}

int m6mo_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct m6mo_state *state = to_state(sd);
	
	switch (ctrl->id) {
	case V4L2_CID_CAMERA_WHITE_BALANCE:		
		ctrl->value = state->userset.manual_wb;
		break;
	case V4L2_CID_CAMERA_BRIGHTNESS:
		ctrl->value = state->userset.brightness;
		break;
	case V4L2_CID_ZOOM_ABSOLUTE:
		ctrl->value = state->userset.zoom_level;
		break;
	case V4L2_CID_CAMERA_FOCUS_WINDOW:
		ctrl->value = state->userset.focus_position;
		break;
	case V4L2_CID_CAM_JPEG_MAIN_OFFSET:
	case V4L2_CID_CAM_JPEG_THUMB_OFFSET:
	case V4L2_CID_CAM_JPEG_POSTVIEW_OFFSET:		
		ctrl->value =  0;
		break;
	case V4L2_CID_CAM_FW_MINOR_VER:
		ctrl->value = state->fw_version;
		break;
	case V4L2_CID_CAM_JPEG_MEMSIZE:
		ret = m6mo_get_jpeg_mem_size(sd, ctrl);
		break;	
	case V4L2_CID_CAM_JPEG_MAIN_SIZE:
		ret = m6mo_get_jpeg_main_size(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SCENE_EV:
		ret = m6mo_get_scene_ev(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT:
		ret = m6mo_get_auto_focus_result(sd, ctrl); 
		break;

	case V4L2_CID_CAMERA_PANO_PICTURE_NUM:
		ctrl->value = state->pano.counter + 1;
		break;
	case V4L2_CID_CAMERA_PANO_READY:
		ret = m6mo_wait_panorama_stitch(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_PANO_DIRECTION:
		ret = m6mo_get_pan_direction(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_MULTI_CAPTURE_READY:
		ret = m6mo_wait_multi_capture_ready(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_SMILE_FACE_DETECTION:
		ret = m6mo_get_smile_face_detection(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FACE_DET_NUM:
		ret = m6mo_get_face_detected_num(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SELECTED_FACE_LOCATION:
		ret = m6mo_get_selected_face_location(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SELECTED_FACE_SIZE:
		ret = m6mo_get_selected_face_size(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_EXIF_EXPTIME_NUMERATOR:
		pr_info("LEGACY %s(), ctrl->id is 0x%x\n", __func__, ctrl->id);
		ret = m6mo_get_exif_exptime(sd, ctrl, 0);
		break;

	case V4L2_CID_CAMERA_EXIF_EXPTIME_DENUMINATOR:
		pr_info("LEGACY %s(), ctrl->id is 0x%x\n", __func__, ctrl->id);		
		ret = m6mo_get_exif_exptime(sd, ctrl, 1);
		break;

	case V4L2_CID_CAMERA_EXIF_ISOV:
		pr_info("LEGACY %s(), ctrl->id is 0x%x\n", __func__, ctrl->id);
		ret = m6mo_get_exif_iso(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_LIGHT_INFO:
		ret = m6mo_get_light_info(sd, ctrl);
		break;
		
	default:
		pr_err("%s: no such control, ctrl->id=%x, ctrl->value = %d\n", __func__,ctrl->id,ctrl->value);
		return -EINVAL;
	}
	
	return ret;
}

int m6mo_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	
	switch (ctrl->id) {
	case V4L2_CID_CAMERA_CAPTURE:
		ret = m6mo_transfer_capture_data(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_WHITE_BALANCE:
		ret = m6mo_set_wb_preset(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_BRIGHTNESS:	
		ret = m6mo_set_image_brightness(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SCENE_MODE:
		ret = m6mo_set_scenemode(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FOCUS_MODE:
		ret = m6mo_set_focus_mode(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
		ret = m6mo_set_focus(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FOCUS_WINDOW:
		ret = m6mo_set_focus_position(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_ZOOM:
		ret = m6mo_set_zoom_level(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_ISO:
		ret = m6mo_set_iso(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_WDR:
		pr_info("%s(), set WDR to %d\n", __func__, ctrl->value);
		ret = m6mo_set_wdr(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FLASH_MODE:
		ret = m6mo_set_flash_mode(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_ROTATION:
		ret = m6mo_set_rotation(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_ISP_REVERSE:
		ret = m6mo_set_reverse(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_ISP_MIRROR:
		ret = m6mo_set_mirror(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_COLORBAR:
		ret = m6mo_set_colorbar(sd, ctrl);
		break;
		
	case V4L2_CID_CAMERA_CAPTURE_FORMAT:
		ret = m6mo_s_cap_format(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_CAPTURE_SIZE:
		ret = m6mo_s_capture_size(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_QUICK_CAPTURE:
		ret = m6mo_start_capture(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_WAKEUP_PREVIEW:
		ret = m6mo_wakeup_preview(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_PANO_CAPTURE:
		ret = m6mo_set_panorama_capture(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_TERMINATE_PANO:
		ret = m6mo_terminate_panorama(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_PANO_STATUS:
		ret = m6mos_set_cur_panorama_info(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_MULTI_CAPTURE:
		ret = m6mo_set_multi_capture(sd, ctrl);	
		break;
	case V4L2_CID_CAMERA_WAIT_MULTI_CAPTURE:
		ret = m6mo_wait_multi_capture_picture(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_MULTI_CAPTURE_PICTURE:
		ret = m6mo_get_multi_capture_picture(sd, ctrl);	
		break;

	case V4L2_CID_CAMERA_SMILE_FACE_DETECTION:
		ret = m6mo_set_smile_face_detection(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FACE_DET_SELECT:
		ret = m6mo_set_face_selection(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FACE_DETECTION_DIRECTION:
		ret = m6mo_set_face_detection_direction(sd, ctrl);
		break;
		
	default:
		pr_err("%s: no such control, ctrl->id=%x\n", __func__,ctrl->id);
		return -EINVAL;
	}
	
	return ret;
}
