#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/m040_l3gd20.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

#define L3G4200D_MAJOR   102
#define L3G4200D_MINOR   4

/* l3g4200d gyroscope registers */
#define WHO_AM_I    0x0F

#define CTRL_REG1       0x20    /* power control reg */
#define CTRL_REG2       0x21    /* power control reg */
#define CTRL_REG3       0x22    /* power control reg */
#define CTRL_REG4       0x23    /* interrupt control reg */
#define CTRL_REG5       0x24    /* interrupt control reg */
#define OUT_TEMP	 (0x26)
#define STATUS_REG      0x27
#define AXISDATA_REG    0x28

#define DEBUG 0

#define L3G4200D_FS_DPS L3G4200D_FS_250DPS
/* For L3GD20,ODR is 95Hz, BW is 25Hz */
#define L3G4200D_ODR_BW ODR95_BW25

/* add by jerrymo, default delay 200 ms */
#define L3G4200D_DEFAULT_DELAY 200000	/* us */
#define US_TO_JIFFIES(us) ((unsigned long)(us) / (USEC_PER_SEC / HZ))

static atomic_t suspend_flag = ATOMIC_INIT(0);

struct {
	unsigned int cutoff_us;
	unsigned int odr_bw;
} l3g4200d_gyro_odr_table[] = {
		{    1250, ODR760_BW100 },
		{    2500, ODR380_BW50 },
		{    5000, ODR190_BW25 },
		{   10000, ODR95_BW12_5  },
};

/*
 * L3G4200D gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * signed short
 */

struct l3g4200d_t {
	short	x,	/* x-axis angular rate data. Range -2048 to 2047. */
		y,	/* y-axis angluar rate data. Range -2048 to 2047. */
		z;	/* z-axis angular rate data. Range -2048 to 2047. */
};

/* static struct i2c_client *l3g4200d_client; */

struct l3g4200d_data {
	struct i2c_client *client;
	struct l3g4200d_platform_data *pdata;
	/* u8 sensitivity; */

	/*add by jerrymo*/
	struct delayed_work dwork;
	int enabled;
	int64_t delay;  /*in ns*/
	struct input_dev *input_dev;
	struct early_suspend esuspend;
	int mode;
	struct mutex ioctl_lock;
	struct miscdevice misc_device;
};

static struct l3g4200d_data *gyro = NULL;

static char l3g4200d_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len);

static char l3g4200d_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len);

static int l3gd20_read_temp(char *temp)
{
	int ret = 0;
	WARN_ON(!temp);

	ret = l3g4200d_i2c_read(OUT_TEMP, (unsigned char *)temp, 1);
	if (ret < 0) {
		pr_err("%s() error!\n", __func__);
		return ret;
	}

	pr_info("%s() read %d from OUT_TEMP register\n", __func__, temp[0]);
	return 0;
}

/* set l3g4200d digital gyroscope bandwidth */
int l3g4200d_set_bandwidth(char bw)
{
	int ret = 0;
	unsigned char data;

	if (!gyro || !gyro->client) {
		pr_err("%s(), Error!gyro has not been initialized properly!\n", __func__);
		return -1;
	}

	ret = i2c_smbus_read_word_data(gyro->client, CTRL_REG1);
	if (ret >= 0)
		data = ret & 0x000F;

	data = data + bw;
	ret = l3g4200d_i2c_write(CTRL_REG1, &data, 1);
	return ret;
}

/* read selected bandwidth from l3g4200d */
int l3g4200d_get_bandwidth(unsigned char *bw)
{
	int ret = 1;
	/* TO DO */
	return ret;
}

int l3g4200d_set_mode(char mode)
{
	int ret = 0;
	unsigned char data;

	if (!gyro || !gyro->client) {
		pr_err("%s(), Sorry, Gyro driver L3GD20 has not been initialized properly!\n", __func__);
		return -1;
	}

	pr_info("%s(), gyro is %p, gyro->client is %p\n", __func__, gyro, gyro->client);

	ret = i2c_smbus_read_word_data(gyro->client, CTRL_REG1);
	if (ret >= 0)
		data = ret & 0x00F7;

	data = mode + data;

	ret = l3g4200d_i2c_write(CTRL_REG1, &data, 1);
	return ret;
}

int l3g4200d_set_range(char range)
{
	int ret = 0;
	unsigned char data;

	if (!gyro || !gyro->client) {
		pr_err("%s(), Sorry, Gyro driver L3GD20 has not been initialized properly!\n", __func__);
		return -1;
	}

	ret = i2c_smbus_read_word_data(gyro->client, CTRL_REG4);
	if (ret >= 0)
		data = ret & 0x00CF;

	data = range + data;
	ret = l3g4200d_i2c_write(CTRL_REG4, &data, 1);
	return ret;
}

/* gyroscope data readout */
static int l3g4200d_read_gyro_values(struct l3g4200d_t *data)
{	
	struct l3g4200d_platform_data *pdata;
	int ret;
	/* x,y,z hardware data */
	short hw_d[3] = { 0 };

	if (!gyro || !gyro->client) {
		pr_err("%s(), Sorry, Gyro driver L3GD20 has not been initialized properly!\n", __func__);
		return -1;
	}

	pdata = gyro->pdata;

	ret = l3g4200d_i2c_read(AXISDATA_REG, (unsigned char *)hw_d, 6);
	if (ret < 0) {
		pr_err("%s() fails\n", __func__);
		return ret;
	}
	/* adjust x, y, z coordinates */
	data->x = ((pdata->negate_x) ? (-hw_d[pdata->axis_map_x])
		   : (hw_d[pdata->axis_map_x]));
	data->y = ((pdata->negate_y) ? (-hw_d[pdata->axis_map_y])
		   : (hw_d[pdata->axis_map_y]));
	data->z = ((pdata->negate_z) ? (-hw_d[pdata->axis_map_z])
		   : (hw_d[pdata->axis_map_z]));

#if DEBUG
	pr_info("=gyro=: %d, %d, %d.\n", data->x, data->y, data->z);
#endif

	return 0;
}


/*
  * add by jerrymo
  * for delay work function, update data
*/
static void l3g4200d_work_func(struct work_struct *work)
{
	int ret;
	struct l3g4200d_t data;

	ret = l3g4200d_read_gyro_values(&data);
	if (ret < 0) {
		pr_err("%s()->%d:read gyro values fail!\n",
			__func__, __LINE__);
		return;
	}

	input_report_abs(gyro->input_dev, ABS_X, data.x);
	input_report_abs(gyro->input_dev, ABS_Y, data.y);
	input_report_abs(gyro->input_dev, ABS_Z, data.z);
	input_sync(gyro->input_dev);

#if 0
	pr_info("%s():x = %d, y = %d, z = %d.\n", __func__,
		data.x, data.y, data.z);
#endif

	if (gyro->enabled)
		schedule_delayed_work(&gyro->dwork, US_TO_JIFFIES(gyro->delay));
}

/* Device Initialization  */
static int device_init(void)
{
	int ret;
	unsigned char buf[5];
	buf[0] = 0x27 | L3G4200D_ODR_BW;   /*ODR 100Hz, cut off 0Hz, x,y,z enable, power down mode*/
	buf[1] = 0x00;   /*high pass filter disable*/
	buf[2] = 0x00;   /*don't use interrupt*/
	buf[3] = 0x00;   /*250 dps*/
	buf[4] = 0x00;   /**/
	ret = l3g4200d_i2c_write(CTRL_REG1, &buf[0], 5);
	return ret;
}

static int l3g4200d_gyro_update_odr(int poll_interval_us)
{
	int err = -1;
	int i;

	for (i = ARRAY_SIZE(l3g4200d_gyro_odr_table) - 1; i >= 0; i--) {
		if (l3g4200d_gyro_odr_table[i].cutoff_us < poll_interval_us)
			break;
	}
	WARN_ON(i < 0);
	if (i < 0) {
		pr_warn("%s(), @poll_intervals_us is too small!\n", __func__);
		i = 0;
	}

	pr_info("%s: poll_interval_us = %d, set odr to %x\n", __func__, poll_interval_us, l3g4200d_gyro_odr_table[i].odr_bw);
	err = l3g4200d_set_bandwidth(l3g4200d_gyro_odr_table[i].odr_bw);
	if (err) {
		pr_err("%s: l3g4200d_set_bandwidth fail\n", __func__);
		return err;
	}

	return 0;
}

/*
 * configure device registers
 * add by jerrymo
 */
static int l3g4200d_hw_init(void)
{
	int ret;

	ret = l3g4200d_set_mode(PM_NORMAL);
	if (ret)
		goto err_exit;

	ret = l3g4200d_set_range(L3G4200D_FS_DPS);
	if (ret) {
		pr_info("%s: set range fail\n", __func__);
		goto err_exit;
	}

	ret = l3g4200d_set_bandwidth(L3G4200D_ODR_BW);
	if (ret) {
		pr_info("%s: set bandwidth fail\n", __func__);
		goto err_exit;
	}

	return 0;

err_exit:
	pr_err("%s()->%d: error happen(ret = %d)!\n",
		__func__, __LINE__, ret);

	return ret;
}

static int l3g4200d_power_down(void)
{
	return l3g4200d_set_mode(PM_OFF);
}

static int l3g4200d_start_work(struct l3g4200d_data *l3g4200d)
{
	int ret;

	ret = l3g4200d_hw_init();
	if (ret)
		return ret;

	schedule_delayed_work(&l3g4200d->dwork, US_TO_JIFFIES(l3g4200d->delay));

	return 0;
}

static void l3g4200d_stop_work(struct l3g4200d_data *l3g4200d)
{
	cancel_delayed_work_sync(&gyro->dwork);
	l3g4200d_power_down();
}

static int l3g4200d_set_enable(struct l3g4200d_data *l3g4200d, int enable)
{
	if (enable == l3g4200d->enabled)
		return 0;

	if (enable) {
		int ret = l3g4200d_start_work(l3g4200d);
		if (ret)
			return ret;

		l3g4200d->enabled = 1;
	} else {
		l3g4200d->enabled = 0;
		l3g4200d_stop_work(l3g4200d);
	}

	return 0;
}

/* i2c write routine for l3g4200d digital gyroscope */
static char l3g4200d_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len)
{
	int dummy;
	int i;

	if (!gyro || !gyro->client) {
		pr_err("%s(), Sorry, Gyro driver L3GD20 has not been initialized properly!\n", __func__);
		return -1;
	}

	for (i = 0; i < len; i++) {
		dummy = i2c_smbus_write_byte_data(gyro->client,
						  reg_addr++, data[i]);
		if (dummy) {
			#if DEBUG
			pr_err("i2c write error\n");
			#endif
			return dummy;
		}
	}
	return 0;
}

/* i2c read routine for l3g4200d digital gyroscope */
static char l3g4200d_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len)
{
	struct i2c_client *client;
	int dummy = 0;
	int i = 0;

	if (!gyro || !gyro->client) {
		pr_err("%s(), Sorry, Gyro driver L3GD20 has not been initialized properly!\n", __func__);
		return -1;
	}

	client = gyro->client;

	while (i < len) {
		dummy = i2c_smbus_read_word_data(client, reg_addr++);
		if (dummy >= 0) {
			data[i] = dummy & 0x00ff;
			i++;
		} else {
			pr_err(" i2c read error\n ");
			break;
		}
		dummy = len;
	}
	return dummy;
}

/*  read command for l3g4200d device file  */
static ssize_t l3g4200d_read(struct file *file, char __user *buf,
				  size_t count, loff_t *offset)
{
#if DEBUG
	struct l3g4200d_t data;
#endif
	if (!gyro || !gyro->client) {
		pr_err("%s(), Sorry, Gyro driver L3GD20 has not been initialized properly!\n", __func__);
		return -1;
	}
#if DEBUG
	l3g4200d_read_gyro_values(&data);
	pr_info("X axis: %d\n", data.x);
	pr_info("Y axis: %d\n", data.y);
	pr_info("Z axis: %d\n", data.z);
#endif
	return 0;
}

/*  write command for l3g4200d device file */
static ssize_t l3g4200d_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *offset)
{
	if (!gyro || !gyro->client) {
		pr_err("%s(), Sorry, Gyro driver L3GD20 has not been initialized properly!\n", __func__);
		return -1;
	}
#if DEBUG
		pr_info("l3g4200d should be accessed with ioctl command\n");
#endif
	return 0;
}

/*  open command for l3g4200d device file  */
static int l3g4200d_open(struct inode *inode, struct file *file)
{
	if (!gyro || gyro->client == NULL) {
#if DEBUG
		pr_err("%s(), Sorry, Gyro driver L3GD20 has not been initialized properly!\n", __func__);
#endif
		return -1;
	}
	device_init();

#if DEBUG
	pr_info("l3gd20 has been opened\n");
#endif
	return 0;
}

/*  release command for l3g4200d device file */
static int l3g4200d_close(struct inode *inode, struct file *file)
{
#if DEBUG
	pr_info("L3GD20 has been closed\n");
#endif
	return 0;
}

/* Noted by qudao, For l3gd20, do NOT support self test. */
/* selftest start */
#define SELFTEST_MEASURE_TIMEOUT 100
#define I2C_RETRY_DELAY 10
#define SELFTEST_ZYXDA (0x1 << 3)
#define SELFTEST_SAMPLES 5

static int selftest_init_l3g4200d(void)
{
	unsigned char buf[5];
	pr_info("%s\n", __func__);

	/* BDU=1, ODR=200Hz, Cut-Off Freq=50Hz, FS=2000 DPS */
	/* Noted by qudao, for l3gd20:
	 * ODR = 190Hz, Cut-Off Freq = 50Hz, x/y/z anxis enabled, Normal mode, FS = 2000DPS
	 */
	buf[0] = 0x6F;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0xA0;
	buf[4] = 0x02;

	return l3g4200d_i2c_write(CTRL_REG1, &buf[0], 5);
}

static int selftest_enable(void)
{
	/*
	* For l3g4200d, Self test 0 (+)
	*
	*/
	u8 buf = 0xA2;
	pr_info("%s\n", __func__);

	return l3g4200d_i2c_write(CTRL_REG4, &buf, 1);
}


static void selftest_disable(void)
{
	u8 buf = 0x00;
	pr_info("%s\n", __func__);

	/* Disable sensor */
	l3g4200d_i2c_write(CTRL_REG1, &buf, 1);
	/* Disable selftest */
	l3g4200d_i2c_write(CTRL_REG4, &buf, 1);
}

static int selftest_wait_ZYXDA(void)
{
	int i, ret;
	unsigned char data_ready;

	pr_info("%s\n", __func__);

	for (i = SELFTEST_MEASURE_TIMEOUT; i != 0; i--) {
		data_ready = 0;
		ret = l3g4200d_i2c_read(STATUS_REG, &data_ready, 1);
		if (ret < 0) {
			pr_err("%s: l3g4200d_i2c_read fail, retry %d\n", __func__, i);
			msleep(I2C_RETRY_DELAY);
			continue;
		} else if (data_ready & SELFTEST_ZYXDA) {
			pr_info("%s: data ready\n", __func__);
			break;
		}
	}
	if (i == 0) {
		pr_err("%s: failed\n", __func__);
		return ret;
	}

	return 0;
}

static int selftest_read(struct l3g4200d_t *data)
{
	int total[3];
	int i, ret;

	pr_info("%s\n", __func__);

	total[0] = 0;
	total[1] = 0;
	total[2] = 0;
	for (i = 0; i < SELFTEST_SAMPLES; i++) {
		ret = selftest_wait_ZYXDA();
		if (ret) {
			pr_err("%s: selftest_check_ZYXDA fail\n", __func__);
			return ret;
		}
		ret = l3g4200d_read_gyro_values(data);
		if (ret < 0) {
			pr_err("%s: l3gd20_read_gyro_values fail\n", __func__);
			return ret;
		}
		pr_info("%s: data: x = %d, y = %d, z = %d\n", __func__, data->x, data->y, data->z);
		total[0] += data->x;
		total[1] += data->y;
		total[2] += data->z;
		pr_info("%s: total: x = %d, y = %d, z = %d\n", __func__, total[0], total[1], total[2]);
	}
	data->x = total[0] / SELFTEST_SAMPLES;
	data->y = total[1] / SELFTEST_SAMPLES;
	data->z = total[2] / SELFTEST_SAMPLES;
	pr_info("%s: average: x = %d, y = %d, z = %d\n", __func__, data->x, data->y, data->z);

	return 0;
}

/*
 * Part Number Min_X Max_X Min_Y Max_Y Min_Z Max_Z Unit
 * L3G4200D 175  875 175 875 175 875 DPS (@ FS = +/-2000dps)
 */

#define SELFTEST_MIN (175UL * 1000)	/* mdps */
#define SELFTEST_MAX (875UL * 1000)	/* mdps */
#define CONVERT_TO_MDPS	70UL		/* for range = 2000 DPS */

#define SELFTEST_NORMAL(st, nost, axis)			\
({							\
	unsigned long __abs_data = abs(st->axis - nost->axis) * CONVERT_TO_MDPS;	\
	int __ret;					\
	__ret = (__abs_data <= SELFTEST_MAX) && (__abs_data >= SELFTEST_MIN);	\
	__ret;									\
})

static inline int selftest_check(struct l3g4200d_t *data_nost, struct l3g4200d_t *data_st)
{
	pr_info("%s\n", __func__);
	pr_info("%s:  MAX: %lu, MIN: %lu\n", __func__, SELFTEST_MAX, SELFTEST_MIN);
	pr_info("%s:X: %lu\t", __func__, abs(data_st->x - data_nost->x) * CONVERT_TO_MDPS);
	pr_info("%s:Y: %lu\t", __func__, abs(data_st->y - data_nost->y) * CONVERT_TO_MDPS);
	pr_info("%s:Z: %lu\t", __func__, abs(data_st->z - data_nost->z) * CONVERT_TO_MDPS);

	/* Pass return 0, fail return -1 */
	if (SELFTEST_NORMAL(data_st, data_nost, x) \
		&& SELFTEST_NORMAL(data_st, data_nost, y) \
		&& SELFTEST_NORMAL(data_st, data_nost, z)) {
		return 0;
	}

	return -1;
}

static int l3g4200d_selftest(int *test_result)
{
	int ret;
	struct l3g4200d_t data_nost, data_st;

	/* Initialize Sensor, turn on sensor, enable P/R/Y */
	ret = selftest_init_l3g4200d();
	if (ret < 0) {
		pr_err("%s: selftest_init_l3g4200d fail\n", __func__);
		return ret;
	}
	/* Wait for stable output */
	pr_info("%s: wait for stable output\n", __func__);
	msleep(800);
	/* Read out normal output */
	ret = selftest_read(&data_nost);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}
	pr_info("%s: normal output: x = %d, y = %d, z = %d\n",
		__func__, data_nost.x, data_nost.y, data_nost.z);

	/* Enable self test */
	ret = selftest_enable();
	if (ret < 0) {
		pr_err("%s: selftest_enable failed\n", __func__);
		return ret;
	}
	/* ODR=200HZ, wait for 3 * ODR */
	mdelay(3 * (1000 / 200));
	/* Read out selftest output */
	ret = selftest_read(&data_st);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}
	/* Check output */
	ret = selftest_check(&data_nost, &data_st);
	if (ret < 0) {
		pr_err("%s: ***fail***\n", __func__);
		*test_result = 0;
	} else {
		pr_info("%s: ***success***\n", __func__);
		*test_result = 1;
	}
	/* selftest disable */
	selftest_disable();

	return ret;
}
/* selftest end */

/*  ioctl command for l3g4200d device file */
static long l3g4200d_ioctl_int(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	int enable;
	int64_t delay;
	int test_result;
	int status;
	char temp[1] = {0};

	/* check l3g4200d_client */
	if (!gyro || gyro->client == NULL) {
#if DEBUG
		pr_err("%s(), Sorry, Gyro driver L3GD20 has not been initialized properly!\n", __func__);
#endif
		return -EFAULT;
	}

	/* cmd mapping */

	switch (cmd) {
	case L3GD20_GET_TEMP:
		if (l3gd20_read_temp(temp) < 0) {
			pr_err("%s(), L3GD20_GET_TEMP ioctl error!\n", __func__);
			return -EFAULT;
		}

		if (copy_to_user((char __user *)arg, temp, 1) != 0) {
			#if DEBUG
			pr_err("L3GD20_GET_TEMP copy_to error\n");
			#endif
			return -EFAULT;
		}

		return 0;


	case L3G4200D_SELFTEST:
		err = l3g4200d_selftest(&test_result);

		pr_info("%s: self test\n", __func__);
		if (copy_to_user((void __user *)arg, &test_result, sizeof(int)) != 0) {
			pr_err("%s: copy_to_user error\n", __func__);
			return -EFAULT;
		}
		return err;

	case L3G4200D_SET_RANGE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			pr_err("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = l3g4200d_set_range(*data);
		return err;

	case L3G4200D_SET_MODE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			pr_err("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		err = l3g4200d_set_mode(*data);
		if (!err)
			gyro->mode = data[0];

		return err;

	case L3G4200D_SET_BANDWIDTH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			pr_err("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = l3g4200d_set_bandwidth(*data);
		return err;

	case L3G4200D_READ_GYRO_VALUES:
		err = l3g4200d_read_gyro_values(
				(struct l3g4200d_t *)data);

		if (copy_to_user((struct l3g4200d_t *)arg,
				 (struct l3g4200d_t *)data, 6) != 0) {
#if DEBUG
			pr_err("copy_to error\n");
#endif
			return -EFAULT;
		}
		return err;

	/*add by jerrymo*/
	case L3G4200D_SET_ENABLE:
		if (copy_from_user(&enable, (void __user *)arg, sizeof(int))) {
			pr_err("%s()->%d:copy from user fail!\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		pr_info("%s():set enable %d.\n", __func__, enable);
		err = l3g4200d_set_enable(gyro, enable);
		return err;
		break;
	case L3G4200D_GET_ENABLE:
		if (copy_to_user((void __user *)arg, &gyro->enabled, sizeof(int))) {
			pr_err("%s()->%d:copy to user fail!\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		pr_info("%s():get enable %d.\n", __func__, gyro->enabled);
		break;
	case L3G4200D_SET_DELAY:
		if (copy_from_user(&delay, (void __user *)arg, sizeof(int64_t))) {
			pr_err("%s()->%d:copy from user fail!\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		/* delay in us */
		gyro->delay = delay;
		l3g4200d_gyro_update_odr(gyro->delay);
		pr_debug("%s():set delay %lld.\n", __func__, delay);
		break;
	case L3G4200D_GET_DELAY:
		if (copy_to_user((void __user *)arg, &gyro->delay, sizeof(int64_t))) {
			pr_err("%s()->%d:copy to user fail!\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		pr_debug("%s():get delay %lld.\n", __func__, gyro->delay);
		break;
	case L3G4200D_GET_SUSPEND_STATUS:
		status = atomic_read(&suspend_flag);
		if (copy_to_user((void __user *)arg, &status, sizeof(status))) {
			pr_err("%s()->%d:copy to user fail!\n",
				__func__, __LINE__);
			return -EFAULT;
		}
		pr_debug("%s():get delay %d.\n", __func__, atomic_read(&suspend_flag));
		break;
	/*end add*/
	default:
		return 0;
	}

	return 0;
}

static long l3g4200d_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = -EFAULT;
	if (gyro) {
		mutex_lock(&gyro->ioctl_lock);
		ret = l3g4200d_ioctl_int(file, cmd, arg);
		mutex_unlock(&gyro->ioctl_lock);
	}
	return ret;
}

static const struct file_operations l3g4200d_fops = {
	.owner = THIS_MODULE,
	.read = l3g4200d_read,
	.write = l3g4200d_write,
	.open = l3g4200d_open,
	.release = l3g4200d_close,
	.unlocked_ioctl = l3g4200d_ioctl,
};

static int l3g4200d_validate_pdata(struct l3g4200d_data *gyro)
{
	if (gyro->pdata->axis_map_x > 2 ||
	    gyro->pdata->axis_map_y > 2 ||
	    gyro->pdata->axis_map_z > 2) {
		dev_err(&gyro->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			gyro->pdata->axis_map_x, gyro->pdata->axis_map_y,
			gyro->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (gyro->pdata->negate_x > 1 ||
	    gyro->pdata->negate_y > 1 ||
	    gyro->pdata->negate_z > 1) {
		dev_err(&gyro->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			gyro->pdata->negate_x, gyro->pdata->negate_y,
			gyro->pdata->negate_z);
		return -EINVAL;
	}

	return 0;
}

/*add by jerrymo*/
static int l3g4200d_create_input(struct l3g4200d_data *l3g4200d)
{
	int ret;
	struct input_dev *dev;

	dev = input_allocate_device();
	if (!dev) {
		pr_err("%s()->%d:can not alloc memory to L3GD20 input device!\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	set_bit(EV_ABS, dev->evbit);
	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_Y);
	input_set_capability(dev, EV_ABS, ABS_Z);
	input_set_abs_params(dev, ABS_X, 0, 65535, 0, 0);  /*the max value 16bit*/
	input_set_abs_params(dev, ABS_Y, 0, 65535, 0, 0);  /*the max value 16bit*/
	input_set_abs_params(dev, ABS_Z, 0, 65535, 0, 0);  /*the max value 16bit*/
	dev->name = "gyroscope";
	dev->dev.parent = &l3g4200d->client->dev;

	ret = input_register_device(dev);
	if (ret < 0) {
		pr_err("%s()->%d:can not register L3GD20 input device!\n",
			__func__, __LINE__);
		input_free_device(dev);
		return ret;
	}

	l3g4200d->input_dev = dev;
	input_set_drvdata(l3g4200d->input_dev, l3g4200d);

	return 0;
}

static void l3g4200d_free_input(struct l3g4200d_data *l3g4200d)
{
	input_unregister_device(l3g4200d->input_dev);
	input_free_device(l3g4200d->input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gyro_early_suspend(struct early_suspend *handler)
{
	pr_debug("%s\n", __func__);
	atomic_set(&suspend_flag, 1);
}

static void gyro_early_resume(struct early_suspend *handler)
{
	pr_debug("%s\n", __func__);

	atomic_set(&suspend_flag, 0);
}

struct early_suspend gyro_esuspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = gyro_early_suspend,
	.resume = gyro_early_resume,
};
#endif

#ifdef CONFIG_PM
static int l3g4200d_suspend(struct device *dev)
{
	int ret;
	pr_debug("%s\n", __func__);

	if (!gyro || !gyro->client) {
		pr_err("%s(), Sorry, Gyro driver L3GD20 has not been initialized properly!\n", __func__);
		return -1;
	}

	if (gyro->mode == PM_NORMAL) {
		ret = l3g4200d_set_mode(PM_OFF);
		if (ret < 0)
			pr_err("%s():set pm off mode fail(ret = %d)!\n", __func__, ret);
	}

	return 0;
}
static int l3g4200d_resume(struct device *dev)
{
	int ret;
	pr_debug("%s\n", __func__);

	if (!gyro || !gyro->client) {
		pr_err("%s(), Sorry, Gyro driver L3GD20 has not been initialized properly!\n", __func__);
		return -1;
	}

	if (gyro->mode == PM_NORMAL) {
		ret = l3g4200d_set_mode(PM_NORMAL);
		if (ret < 0)
			pr_err("%s():set pm normal mode fail(ret = %d)!\n", __func__, ret);
	}

	return 0;
}
#endif

static void l3g4200d_shutdown(struct i2c_client *client)
{
	l3g4200d_set_mode(PM_OFF);
}

static int __devinit l3gd20_probe(struct i2c_client *client,
			       const struct i2c_device_id *devid)
{
	struct l3g4200d_data *data;
	int err = 0;
	int tempvalue = 0;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		err = -ENODEV;
		goto exit;
	}

	/*
	 * OK. For now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 */
	data = kzalloc(sizeof(struct l3g4200d_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto exit;
	}

	mutex_init(&data->ioctl_lock);
	i2c_set_clientdata(client, data);
	data->client = client;
	gyro = data;

	data->pdata = kmemdup(client->dev.platform_data,
							sizeof(*data->pdata), GFP_KERNEL);
	if (!data->pdata)
		goto exit_kfree;

	err = l3g4200d_validate_pdata(data);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	err = i2c_smbus_read_byte(client);
	
	if (err < 0) {
#if DEBUG
		pr_err("i2c_smbus_read_byte error!!\n");
		pr_info("%s(), client's content: addr: 0x%x, adapter: %p \n"
			"name: %s, driver:%p, client->adapter.name is %s\n", __func__, client->addr,
			client->adapter, client->name, client->driver,
			client->adapter->name);
#endif
		goto exit_kfree_pdata;
	} else {
#if DEBUG
		pr_info("L3GD20 DSevice detected!\n");
#endif
	}

	/* read chip id */
	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);
	printk("%s(), tempvalue is 0x%x\n", __func__, tempvalue);
	if ((tempvalue & 0x00FF) == GYROSCPOE_CHIP_ID) {
#if DEBUG
		pr_info("I2C driver registered!\n");
#endif
	} else {
		pr_err("%s(), Read Chip ID Error!\n", __func__);
		data->client = NULL;
		err = -ENODEV;
		goto exit_kfree_pdata;
	}

	data->misc_device.minor = MISC_DYNAMIC_MINOR;
	data->misc_device.name = GYROSCPOE_CHIP_NAME;
	data->misc_device.fops = &l3g4200d_fops;
	err = misc_register(&data->misc_device);
	if (err < 0) {
		pr_err("%s()->%d:can not create misc device!\n",
			__func__, __LINE__);
		goto exit_kfree_pdata;
	}

	/* add by jerrymo, create input device. */
	err = l3g4200d_create_input(data);
	if (err)
		goto exit_misc_deregister;

	INIT_DELAYED_WORK(&data->dwork, l3g4200d_work_func);
	data->delay = L3G4200D_DEFAULT_DELAY;
	l3g4200d_gyro_update_odr(data->delay);

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->esuspend = gyro_esuspend;
	register_early_suspend(&data->esuspend);
#endif

#ifdef CONFIG_SENSORS_L3GD20_SELFTEST
	do {
		int test_result;

		pr_info("%s(), Initial self test\n", __func__);
		err = l3g4200d_selftest(&test_result);
		if (err < 0)
			pr_err("%s: fail\n", __func__);
		else
			pr_info("%s: self test\n", __func__);

		if (test_result == 1)
			pr_info("%s: PASS\n", __func__);
		else
			pr_info("%s: FAIL\n", __func__);
	} while (0);
#endif

#if DEBUG
	pr_info("L3GD20 device created successfully\n");
#endif

	return 0;

exit_misc_deregister:
	misc_deregister(&data->misc_device);
exit_kfree_pdata:
	kfree(data->pdata);
exit_kfree:
	kfree(data);
exit:
	gyro = NULL;
#if DEBUG
	pr_err("%s: Driver Initialization failed\n", __FILE__);
#endif
	return err;
}

static int __devexit l3g4200d_remove(struct i2c_client *client)
{
#if DEBUG
	pr_info("L3GD20 driver removing\n");
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&gyro->esuspend);
#endif

	/*add by jerrymo*/
	if (gyro->enabled)
		l3g4200d_set_enable(gyro, 0);
	l3g4200d_free_input(gyro);
	misc_deregister(&gyro->misc_device);

	kfree(gyro->pdata);
	kfree(gyro);
	gyro = NULL;
	return 0;
}

static const struct i2c_device_id l3gd20_id[] = {
	{ GYROSCPOE_CHIP_NAME, 0 },
	{},
};

#ifdef CONFIG_PM
static const struct dev_pm_ops l3g4200d_pm_ops = {
	.suspend = l3g4200d_suspend,
	.resume = l3g4200d_resume,
};
#endif

MODULE_DEVICE_TABLE(i2c, l3gd20_id);

static struct i2c_driver l3gd20_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = l3gd20_probe,
	.remove = __devexit_p(l3g4200d_remove),
	.shutdown = l3g4200d_shutdown,
	.id_table = l3gd20_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = "l3gd20",
#ifdef CONFIG_PM
		.pm = &l3g4200d_pm_ops,
#endif
	},
};

static int __init l3gd20_init(void)
{
#if DEBUG
	pr_info("L3GD20 init driver\n");
#endif
	return i2c_add_driver(&l3gd20_driver);
}

static void __exit l3gd20_exit(void)
{
#if DEBUG
	pr_info("L3GD20 exit\n");
#endif
	i2c_del_driver(&l3gd20_driver);
	return;
}

module_init(l3gd20_init);
module_exit(l3gd20_exit);

MODULE_DESCRIPTION("l3gd20 digital gyroscope driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");
