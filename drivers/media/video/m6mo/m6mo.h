#ifndef __M6MO_CORE_H__ 
#define __M6MO_CORE_H__ 

#define M6MO_DRIVER_NAME "m6mo"

/* the time for wait completion */
#define WAIT_TIMEOUT 5000  /*in milisecond*/

#define M6MO_DEBUG 1
#define m6mo_dbg(fmt, ...) pr_info("%s:"fmt, __func__, ##__VA_ARGS__)
#define m6mo_trace() pr_info("%s:line = %d\n", __func__, __LINE__)

#define CHECK_ERR(x) if ((x) < 0) { \
				pr_err("%s():line = %d, i2c failed, ret = %d\n", \
				__func__, __LINE__, x); \
				return x; \
			}

#define BACK_CAMERA 0
#define FRONT_CAMERA 1  

/* flash */
#define FLASH_LED_NAME "flash_led"
/* unit is uA */
#define PRE_FLASH_CURRENT 100000
#define FULL_FLASH_CURRENT 175000
#define MAX_FLASH_CURRENT 1000000

enum m6mo_i2c_size {
	I2C_8BIT = 1,
	I2C_16BIT = 2,
	I2C_32BIT = 4,
	I2C_MAX = 4,
};

struct m6mo_reg {
	enum m6mo_i2c_size size;
	unsigned int addr;
	unsigned int val;
};

/* 
  * firmware status: indicate ISP firmware checking or updating status
  * FIRMWARE_NONE: ISP firmware has not been checked
  * FIRMWARE_REQUESTING: in the firmware checking or downloading progress
  * FIRMWARE_CHECKED: has been finished checking or downloading firmware
  * FIRMWARE_UPDATE_FAIL:fail to download data to ISP
*/
enum firmware_status {
	FIRMWARE_NONE,
	FIRMWARE_REQUESTING,
	FIRMWARE_CHECKED,
	FIRMWARE_UPDATE_FAIL,
};

/*
  * ISP read or write mode, for register or memory
 */
enum cmd_type {
	CMD_READ_PARAMETER = 1,
	CMD_WRITE_PARAMETER,
	CMD_READ_8BIT_MEMORY,
	CMD_WRITE_8BIT_MEMORY,
	CMD_READ_16BIT_MEMORY,
	CMD_WRITE_16BIT_MEMORY,
	CMD_READ_32BIT_MEMORY,
	CMD_WRITE_32BIT_MEMORY,
};

enum isp_mode {
	INITIALIZE_MODE,
	PARAMETER_MODE,
	MONITOR_MODE,
	CAPTURE_MODE,
};

/*
  * camera preview, record, and capture mode
*/
enum v4l2_camera_mode {
	V4L2_CAMERA_PREVIEW,
	V4L2_CAMERA_RECORD,
	V4L2_CAMERA_PANORAMA,
	V4L2_CAMERA_SINGLE_CAPTURE,
	V4L2_CAMERA_MULTI_CAPTURE,
	V4L2_CAMERA_PANORAMA_CAPTURE,
};

enum camera_mode_type {
	PREVIEW_MODE_TYPE,
	CAPTURE_MODE_TYPE,
};

/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 */
struct m6mo_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;
	enum v4l2_colorspace colorspace;	
	int bpp;
	struct m6mo_reg *regs;
	int size;
};

struct m6mo_size_struct {
	int width;
	int height;
	struct m6mo_reg *regs;
	int size;
};

/* user configure setting, it should be initialized in init function */
struct m6mo_userset {
	unsigned int manual_wb;
	unsigned int brightness;
	unsigned int scene;
	unsigned int zoom_level;
	unsigned int wdr;	
	unsigned int iso;
	unsigned int flash_mode;
	unsigned int rotation;
	unsigned int mirror;
	unsigned int reverse;
	unsigned int af_mode;
	unsigned int focus_position;
};

enum cap_mode {
	CAP_NORMAL_MODE,
	CAP_PANORAMA_MODE,
	CAP_MULTI_CAP_MODE,
	CAP_SMILE_CAP_MODE,
};

#define PANORAMA_MAX_PICTURE 40

/* panorama status */
enum panorama_picture_status {
	PANORAMA_SUCCESS,
	PANORAMA_RETRY_ERR,
	PANORAMA_BIG_ERR,
	PANORAMA_FATAL_ERR,	
	PANORAMA_UNKNOWN_ERR,
	PANORAMA_COMPLETE,  /* means for complete panorama capture */
};

/* panorama stitch status */
enum panorama_stitch_status {
	PANORAMA_STITCH_OK,
	PANORAMA_STITCH_FAIL,
	PANORAMA_STITCH_INIT,
};

/* panorama picture information */
struct panorama_picture {
	enum panorama_picture_status status;  /* status */
	unsigned int extra;  /* extra information */
};

/* panorama data struct */
struct panorama_struct {
	int counter;  /* picture counter */
	struct panorama_picture pictures[PANORAMA_MAX_PICTURE];
	enum panorama_stitch_status stitch_status;
};

#define MULTI_CAP_MAX_PICTURE 9

/* multi capture ready status */
enum multi_cap_picture_status {
	MULTI_CAP_SUCCESS,
	MULTI_CAP_FAIL,
	MULTI_CAP_INIT,
};

enum multi_cap_ready_status {
	MULTI_CAP_READY_SUCCESS,
	MULTI_CAP_READY_FAIL,
	MULTI_CAP_READY_INIT,
};

struct multi_cap_picture {
	enum multi_cap_picture_status status;
};

struct multi_cap_struct {
	int numbers;
	int counter;
	struct multi_cap_picture pictures[MULTI_CAP_MAX_PICTURE];
	enum multi_cap_ready_status ready;
};

enum smile_detection {
	SMILE_NO_DETECTION,
	SMILE_DETECTION,
};

/* smile cap data struct */
struct smile_cap_struct {
	enum smile_detection detection;
};

struct m6mo_state {
	struct m6mo_platform_data *pdata;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	
	struct v4l2_mbus_framefmt fmt; /* current format and size */
	struct m6mo_size_struct prev_size;  /* last preview size*/
	struct m6mo_size_struct cap_size;   /* last capture size */
	struct m6mo_format_struct cap_fmt;  /* last capture format */

	int irq;  /* irq number */
	int irq_status;

	int fps;
	enum isp_mode mode;   /* isp operating mode:parameter, monitor or capture */
	enum v4l2_camera_mode camera_mode;   /* user modes: preview, capture or record */
	bool stream_on;
	struct m6mo_userset userset;   /* user setting */
	bool isp_power;
	bool sensor_power;
	bool debug;
	int cam_id;   /* used distinguishing between front and back camera */
	struct regulator *fled_regulator;
	int pre_flash_current;
	int full_flash_current;
	
	struct completion completion;
	struct mutex mutex;

	int fw_version;
	enum firmware_status fw_status;
	u8 *fw_buffer;
	bool fw_updated;  /* used by factory test */
	struct wake_lock wake_lock;

	struct work_struct work;  /* work for panorama cap, multi cap and smile cap */
	struct workqueue_struct *wq;
	enum cap_mode cap_mode;
	
	struct panorama_struct pano;  /* panorama capture data */
	struct multi_cap_struct	multi_cap;   /* multi-capture data */
	struct smile_cap_struct smile_cap;  /* smile capture data */
};

/*
  * I2C operation functions, implement in m6mo_drv.c
*/
int m6mo_read_reg(struct v4l2_subdev *sd, u16 addr, u32 *val, enum m6mo_i2c_size size);
int m6mo_write_reg(struct v4l2_subdev *sd, u16 addr, u32 val, enum m6mo_i2c_size size);
int m6mo_write_memory(struct v4l2_subdev *sd,
		u32 addr, const char *data,
		int size);
int m6mo_write_regs(struct v4l2_subdev *sd, struct m6mo_reg *regs, int size);

#define m6mo_r8(sd, addr, val) m6mo_read_reg(sd, addr, val, I2C_8BIT)
#define m6mo_r16(sd, addr, val) m6mo_read_reg(sd, addr, val, I2C_16BIT)
#define m6mo_r32(sd, addr, val) m6mo_read_reg(sd, addr, val, I2C_32BIT)

#define m6mo_w8(sd, addr, val) m6mo_write_reg(sd, addr, val, I2C_8BIT)
#define m6mo_w16(sd, addr, val) m6mo_write_reg(sd, addr, val, I2C_16BIT)
#define m6mo_w32(sd, addr, val) m6mo_write_reg(sd, addr, val, I2C_32BIT)


int m6mo_set_sys_mode(struct v4l2_subdev *sd, enum isp_mode mode);
int m6mo_set_power_clock(struct m6mo_state *state, bool enable);
int m6mo_s_power(struct v4l2_subdev *sd, int on);
void m6mo_prepare_wait(struct v4l2_subdev *sd);
int m6mo_enable_root_irq(struct v4l2_subdev *sd);
int m6mo_enable_irq(struct v4l2_subdev *sd);
int m6mo_wait_irq(struct v4l2_subdev *sd, const unsigned int timeout);
int m6mo_wait_irq_and_check(struct v4l2_subdev *sd, u8 mask,
	const unsigned int timeout);
int m6mo_set_capture_format(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt);
int m6mo_set_capture_size(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt);
int m6mo_set_mode(struct v4l2_subdev *sd, enum isp_mode mode);

/* m6mo ctrl functions */
int m6mo_g_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls);
int m6mo_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
int m6mo_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
int m6mo_set_flash_current(struct m6mo_state *state, 
	int cur);
void m6mo_handle_multi_cap(struct m6mo_state *state, int irq_status);
void m6mo_handle_panorama_cap(struct m6mo_state *state, int irq_status);
void m6mo_handle_smile_cap(struct m6mo_state *state, int irq_status);

/* m6mo firmware functions */
int m6mo_load_firmware(struct v4l2_subdev *sd);
int m6mo_run_firmware(struct v4l2_subdev *sd);
int m6mo_erase_firmware(struct v4l2_subdev *sd);
int m6mo_load_firmware_sys(struct device *dev, struct v4l2_subdev *sd);

static inline struct m6mo_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct m6mo_state, sd);
}

extern void fimc_wakeup_preview(void);
extern void fimc_reset_wakeup_flag(void);

#endif
