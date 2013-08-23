#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/earlysuspend.h>
#include <linux/mx_akm8963.h>

#define AKM8963_DEBUG		0
#define AKM8963_DEBUG_MSG	0
#define AKM8963_DEBUG_FUNC	0
#define AKM8963_DEBUG_DATA	0
#define MAX_FAILURE_COUNT	3
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		10
#define AKM8963_DEFAULT_DELAY	100

#if AKM8963_DEBUG_MSG
#define AKMDBG(format, ...)	printk(KERN_INFO "AKM8963 " format "\n", ## __VA_ARGS__)
#else
#define AKMDBG(format, ...)
#endif

#if AKM8963_DEBUG_FUNC
#define AKMFUNC(func) printk(KERN_INFO "AKM8963 " func " is called\n")
#else
#define AKMFUNC(func)
#endif

struct akm8963_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct miscdevice akmd_device;
	struct early_suspend akm_early_suspend;
	struct mutex akm_aot_ioctl_lock;
	struct mutex akmd_ioctl_lock;
	atomic_t open_flag;
	atomic_t reserve_open_flag;
	wait_queue_head_t open_wq;
	u8 asa[3];
	int test_flag;
};

struct s_xyz{
	s16 x;
	s16 y;
	s16 z;		
};

static char sData[SENSOR_DATA_SIZE]; /* for GETDATA */


static atomic_t suspend_flag = ATOMIC_INIT(0);

static int AKI2C_RxData(struct i2c_client *client, char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
#if AKM8963_DEBUG_DATA
	int i;
	char addr = rxData[0];
#endif
#ifdef AKM8963_DEBUG
	/* Caller should check parameter validity.*/
	if ((rxData == NULL) || (length < 1)) {
		return -EINVAL;
	}
#endif
	for (loop_i = 0; loop_i < I2C_RETRIES; loop_i++) {
		if (i2c_transfer(client->adapter, msgs, 2) > 0) {
			break;
		}
		pr_err("%s: i2c_transfer fail, retry %d\n", __func__, loop_i + 1);
		msleep_interruptible(I2C_RETRY_DELAY);
	}
	
	if (loop_i >= I2C_RETRIES) {
		pr_err("%s retry over %d\n", __func__, I2C_RETRIES);
		return -EIO;
	}
#if AKM8963_DEBUG_DATA
	printk(KERN_INFO "RxData: len=%02x, addr=%02x\n  data=", length, addr);
	for (i = 0; i < length; i++) {
		printk(KERN_INFO " %02x", rxData[i]);
	}
    printk(KERN_INFO "\n");
#endif
	return 0;
}

static int AKI2C_TxData(struct i2c_client *client, char *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};
#if AKM8963_DEBUG_DATA
	int i;
#endif
#ifdef AKM8963_DEBUG
	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2)) {
		return -EINVAL;
	}
#endif	
	for (loop_i = 0; loop_i < I2C_RETRIES; loop_i++) {
		if (i2c_transfer(client->adapter, msg, 1) > 0) {
			break;
		}
		pr_err("%s: i2c_transfer fail, retry %d\n", __func__, loop_i);
		msleep_interruptible(I2C_RETRY_DELAY);
	}
	
	if (loop_i >= I2C_RETRIES) {
		pr_err("%s retry over %d\n", __func__, I2C_RETRIES);
		return -EIO;
	}
#if AKM8963_DEBUG_DATA
	printk(KERN_INFO "TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
	for (i = 0; i < (length-1); i++) {
		printk(KERN_INFO " %02x", txData[i + 1]);
	}
	printk(KERN_INFO "\n");
#endif
	return 0;
}

static int AKECS_SetMode_SngMeasure(struct akm8963_data *akm)
{
	char buffer[2];

	buffer[0] = AK8963_REG_CNTL1;
	buffer[1] = AK8963_MODE_SNG_MEASURE | AKM8963_OUTPUT_BIT_SET_16; 
	
	return AKI2C_TxData(akm->client, buffer, 2);
}

static int AKECS_SetMode_Cont1Measure(struct akm8963_data *akm)
{
	char buffer[2];

	buffer[0] = AK8963_REG_CNTL1;
	buffer[1] = AK8963_MODE_CONT1_MEASURE | AKM8963_OUTPUT_BIT_SET_16;
	
	return AKI2C_TxData(akm->client, buffer, 2);
}

static int AKECS_SetMode_Cont2Measure(struct akm8963_data *akm)
{
	char buffer[2];

	buffer[0] = AK8963_REG_CNTL1;
	buffer[1] = AK8963_MODE_CONT2_MEASURE | AKM8963_OUTPUT_BIT_SET_16;
	
	return AKI2C_TxData(akm->client, buffer, 2);
}

static int AKECS_SetMode_TrigMeasure(struct akm8963_data *akm)
{
	char buffer[2];

	buffer[0] = AK8963_REG_CNTL1;
	buffer[1] = AK8963_MODE_TRIGGER_MEASURE | AKM8963_OUTPUT_BIT_SET_16;
	
	return AKI2C_TxData(akm->client, buffer, 2);
}

static int AKECS_SetMode_SelfTest(struct akm8963_data *akm)
{
	char buffer[2];
	
	buffer[0] = AK8963_REG_CNTL1;
	buffer[1] = AK8963_MODE_SELF_TEST | AKM8963_OUTPUT_BIT_SET_16;
	
	return AKI2C_TxData(akm->client, buffer, 2);
}

static int AKECS_SetMode_FUSEAccess(struct akm8963_data *akm)
{
	char buffer[2];
	
	buffer[0] = AK8963_REG_CNTL1;
	buffer[1] = AK8963_MODE_FUSE_ACCESS | AKM8963_OUTPUT_BIT_SET_16;
	
	return AKI2C_TxData(akm->client, buffer, 2);
}

static int AKECS_SetMode_PowerDown(struct akm8963_data *akm)
{
	char buffer[2];
	
	buffer[0] = AK8963_REG_CNTL1;
	buffer[1] = AK8963_MODE_POWERDOWN | AKM8963_OUTPUT_BIT_SET_16;
	
	return AKI2C_TxData(akm->client, buffer, 2);
}

static int AKECS_SetMode(struct akm8963_data *akm, char mode)
{
	int ret;

	switch (mode) {
		case AK8963_MODE_SNG_MEASURE:
			ret = AKECS_SetMode_SngMeasure(akm);
			break;
		case AK8963_MODE_CONT1_MEASURE:
			ret = AKECS_SetMode_Cont1Measure(akm);
			break;
		case AK8963_MODE_CONT2_MEASURE:
			ret = AKECS_SetMode_Cont2Measure(akm);
			break;
		case AK8963_MODE_TRIGGER_MEASURE:
			ret = AKECS_SetMode_TrigMeasure(akm);
			break;
		case AK8963_MODE_SELF_TEST:
			ret = AKECS_SetMode_SelfTest(akm);
			break;
		case AK8963_MODE_FUSE_ACCESS:
			ret = AKECS_SetMode_FUSEAccess(akm);
			break;
		case AK8963_MODE_POWERDOWN:
			ret = AKECS_SetMode_PowerDown(akm);
			/* wait at least 100us after changing mode */
			udelay(100);
			break;
		default:
			AKMDBG("%s: Unknown mode(%d)", __func__, mode);
			return -EINVAL;
	}

	return ret;
}

static int AKECS_CheckDevice(struct akm8963_data *akm)
{
	char buffer[2];
	int ret;
	
	buffer[0] = AK8963_REG_WIA;
	
	/* Read the DEVICE ID */
	ret = AKI2C_RxData(akm->client, buffer, 1);
	if (ret < 0) {
		return ret;
	}
	/* Check read data */
	if (buffer[0] != 0x48) {
		return -ENXIO;
	}
	return 0;
}

/*set AK8963N softreset as normal,do not set softreset.*/
static void AKECS_Softreset(struct akm8963_data *akm, int ifreset)
{
	int ret;
	
	ret = i2c_smbus_write_byte_data(akm->client, AK8963_REG_CNTL2, ifreset);
	if(ret < 0){
		printk("%s:i2c write reg failed\n",__func__);
	}
}

static int AKECS_GetOpenStatus(struct akm8963_data *akm)
{
	wait_event_interruptible(akm->open_wq, (atomic_read(&akm->open_flag) != 0));
	return atomic_read(&akm->open_flag);
}

static int AKECS_GetCloseStatus(struct akm8963_data *akm)
{
	wait_event_interruptible(akm->open_wq, (atomic_read(&akm->open_flag) <= 0));
	return atomic_read(&akm->open_flag);
}

static inline int AKECS_GetSuspendStatus(void)
{
	return atomic_read(&suspend_flag);
}

static void AKECS_CloseDone(void)
{
}

#define AK8963_MEASURE_TIMEOUT 100

static inline int AKECS_GetData(struct akm8963_data *akm)
{
	int res, i;
	u8 x = 0, y = 0, z = 0;
	
	/* Single Measure Mode is set from user-space */
	for (i = AK8963_MEASURE_TIMEOUT; i != 0; i--) {
		/* Get Data Ready Status */
		res = AKI2C_RxData(akm->client, sData, 1);
		if (res) {
			pr_err("%s: AKI2C_RxData 1 error.\n", __func__);
			return res;
		}
		/* If data ready, read them */
		if (sData[0] & 0x01) {
			sData[0] = AK8963_REG_ST1;
			/* Data ready Status, X(H, L), Y(H, L), Z(H, L) data, DERR Status*/
			res = AKI2C_RxData(akm->client, sData, SENSOR_DATA_SIZE);
			if (res) {
				pr_err("%s: AKI2C_RxData 2 error.\n", __func__);
				return res;
			}
			break;
		}
		msleep_interruptible(I2C_RETRY_DELAY);
	}
	if (i == 0) {
		pr_err("%s: DRDY timeout.\n", __func__);
		return -EIO;
	}
	
	if(sData[0] & 0x01){
		x = (sData[2] << 8) | sData[1];
		y = (sData[4] << 8) | sData[3];
		z = (sData[6] << 8) | sData[5];
		pr_debug("%s():x = %d, y = %d, z = %d\n",__func__, x, y, z);
	}else{
		pr_err("%s:read data error, ST1 = %d######3\n",__func__,sData[0] & 0x01);
	}

	return 0;
}

/***** akmd functions ********************************************/
static int akmd_open(struct inode *inode, struct file *file)
{
	AKMFUNC("akmd_open");
	return nonseekable_open(inode, file);
}

static int akmd_release(struct inode *inode, struct file *file)
{
	AKMFUNC("akmd_release");
	AKECS_CloseDone();
	return 0;
}

static long akmd_ioctl_int(struct file *file, unsigned int cmd, 
		unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct akm8963_data *akm = container_of(file->private_data, 
			struct akm8963_data, akmd_device);

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char rwbuf[RWBUF_SIZE];		/* for READ/WRITE */
	char mode;			/* for SET_MODE*/
	short value[12];		/* for SET_YPR */
	short delay;			/* for GET_DELAY */
	int status;			/* for OPEN/CLOSE_STATUS */
	int ret = -1;			/* Return value. */
	
	switch (cmd) {
		case ECS_IOCTL_WRITE:
		case ECS_IOCTL_READ:
			if (argp == NULL) {
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
				AKMDBG("copy_from_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_SET_MODE:
			if (argp == NULL) {
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			if (copy_from_user(&mode, argp, sizeof(mode))) {
				AKMDBG("copy_from_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_SET_YPR:
			if (argp == NULL) {
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			if (copy_from_user(&value, argp, sizeof(value))) {
				AKMDBG("copy_from_user failed.");
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	
	switch (cmd) {
		case ECS_IOCTL_WRITE:
			AKMFUNC("IOCTL_WRITE");
			if ((rwbuf[0] < 2) || (rwbuf[0] > (RWBUF_SIZE-1))) {
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			ret = AKI2C_TxData(akm->client, &rwbuf[1], rwbuf[0]);
			if (ret < 0) {
				return ret;
			}
			break;
		case ECS_IOCTL_READ:
			AKMFUNC("IOCTL_READ");
			if ((rwbuf[0] < 1) || (rwbuf[0] > (RWBUF_SIZE-1))) {
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			ret = AKI2C_RxData(akm->client, &rwbuf[1], rwbuf[0]);
			if (ret < 0) {
				return ret;
			}
			break;
		case ECS_IOCTL_SET_MODE:
			AKMFUNC("IOCTL_SET_MODE");
			ret = AKECS_SetMode(akm, mode);
			if (ret < 0) {
				return ret;
			}
			break;
		case ECS_IOCTL_GETDATA:
			AKMFUNC("IOCTL_GET_DATA");
			ret = AKECS_GetData(akm);
			if (ret < 0) {
				pr_err("%s: IOCTL_GET_DATA error\n", __func__);
				return ret;
			}
			break;
		case ECS_IOCTL_GET_OPEN_STATUS:
			AKMFUNC("IOCTL_GET_OPEN_STATUS");
			status = AKECS_GetOpenStatus(akm);
			AKMDBG("AKECS_GetOpenStatus returned (%d)", status);
			break;
		case ECS_IOCTL_GET_CLOSE_STATUS:
			AKMFUNC("IOCTL_GET_CLOSE_STATUS");
			status = AKECS_GetCloseStatus(akm);
			AKMDBG("AKECS_GetCloseStatus returned (%d)", status);
			break;
		case ECS_IOCTL_GET_DELAY:
			AKMFUNC("IOCTL_GET_DELAY");
			delay = AKM8963_DEFAULT_DELAY;
			break;
		case ECS_IOCTL_GET_SUSPEND_STATUS:
			status = AKECS_GetSuspendStatus();
			break;
		default:
			return -ENOTTY;
	}
	
	switch (cmd) {
		case ECS_IOCTL_READ:
			if (copy_to_user(argp, &rwbuf, rwbuf[0]+1)) {
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GETDATA:
			if (copy_to_user(argp, &sData, SENSOR_DATA_SIZE)) {
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GET_OPEN_STATUS:
		case ECS_IOCTL_GET_CLOSE_STATUS:
			if (copy_to_user(argp, &status, sizeof(status))) {
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GET_SUSPEND_STATUS:
			if (copy_to_user(argp, &status, sizeof(status))) {
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GET_DELAY:
			if (copy_to_user(argp, &delay, sizeof(delay))) {
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	
	return 0;
}

static long akmd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret;
	struct akm8963_data *akm = container_of(file->private_data,
			struct akm8963_data, akmd_device);
	mutex_lock(&akm->akmd_ioctl_lock);
	ret = akmd_ioctl_int(file, cmd, arg);
	mutex_unlock(&akm->akmd_ioctl_lock);
	return ret;
}

static int akm_self_test(struct akm8963_data *s_akm, struct s_xyz* akm_sxyz)
{
	int err = 0;
	u8 buf[2], sbuf[1], s_data[6];
	s16 x = 0, y = 0, z = 0;

	err = AKECS_SetMode(s_akm, AK8963_MODE_POWERDOWN);
	if(err < 0)
		return err;

	/*set ASTC self test bit to 1*/
	buf[0] = AK8963_REG_ASTC;
	buf[1] = 0x40;
	AKI2C_TxData(s_akm->client, buf, 2);

	/*start self test*/
	err = AKECS_SetMode(s_akm, AK8963_MODE_SELF_TEST);	

	sbuf[0] = AK8963_REG_ST1;
	/* wait for data ready :read st1*/
	while(1){
		msleep(200);
		err = AKI2C_RxData(s_akm->client, sbuf,1);
		if(err < 0)
			return err;
		if(sbuf[0] == 1) 
			break;	
	}

	i2c_smbus_read_i2c_block_data(s_akm->client, AK8963_REG_HXL, 
			sizeof(s_data), s_data);	
	
	/*set ASTC self test bit to 0*/
	buf[1] = 0x00;
	AKI2C_TxData(s_akm->client, buf, 2);

	x = (s_data[1] << 8) | s_data[0];
	y = (s_data[3] << 8) | s_data[2];
	z = (s_data[5] << 8) | s_data[4];

	/* Hadj = (H*(Asa+128))/256 */
	x = (x*(s_akm->asa[0] + 128)) >> 8;
	y = (y*(s_akm->asa[1] + 128)) >> 8;
	z = (z*(s_akm->asa[2] + 128)) >> 8;

	/*pr_info("%s: self test x = %d, y = %d, z = %d\n",__func__, x, y, z);*/
	if ((x >= -200) && (x <= 200))
		pr_info("%s: x passed self test, expect -200<=%d<=200\n",__func__, x);
	else
		pr_info("%s: x failed self test, expect -200<=%d<=200\n",__func__, x);
	if ((y >= -200) && (y <= 200))
		pr_info("%s: y passed self test, expect -200<=%d<=200\n",__func__, y);
	else
		pr_info("%s: y failed self test, expect -200<=%d<=200\n",__func__, y);
	if ((z >= -3200) && (z <= -800))
		pr_info("%s: z passed self test, expect -3200<=%d<=-800\n",__func__, z);
	else
		pr_info("%s: z failed self test, expect -3200<=%d<=-800\n",__func__, z);

	if (((x >= -200) && (x <= 200)) && ((y >= -200) && (y <= 200)) &&
	    ((z >= -3200) && (z <= -800))) {
		s_akm->test_flag = 1;
	} else {
		s_akm->test_flag = 0;
	}
	akm_sxyz->x = x;
	akm_sxyz->y = y;
	akm_sxyz->z = z;

	pr_debug("akm_sx = %d, akm_sy = %d, akm_sz = %d\n",akm_sxyz->x, akm_sxyz->y, akm_sxyz->z);
	
	return s_akm->test_flag; 

}

static ssize_t akm_mode_store(struct device *dev, struct device_attribute *attr, 
		const char *buf, size_t count)
{
	unsigned long mode = 0;
	struct akm8963_data *akm = dev_get_drvdata(dev);

	if(strict_strtoul(buf, 10, &mode)){
		pr_info("%s():the mode is %ld\n",__func__, mode);
		return -EINVAL;
	}
	
	if(AKECS_SetMode(akm, mode) < 0){
		pr_info("%s:#######set mode failed\n",__func__);
		return -EINVAL;
	}else{
		pr_info("%s:set mode %ld\n",__func__,mode);
	}

	return count;
}

static ssize_t akm_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	u8  x = 0, y = 0, z = 0;

	if(sData[0] & 0x01){
		x = (sData[2] << 8) | sData[1];
		y = (sData[4] << 8) | sData[3]; 	
		z = (sData[6] << 8) | sData[5];
	}

	ret = sprintf(buf,"%d, %d, %d\n",x, y, z);
	return ret;
}

static ssize_t akm_asa_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t  ret; 
	int err;
	struct akm8963_data *akm = dev_get_drvdata(dev);

	err = AKECS_SetMode_FUSEAccess(akm);
	if(err < 0){
		pr_err("%s:unable to set fuse rom mode\n",__func__);	
	}

	err = i2c_smbus_read_i2c_block_data(akm->client, AK8963_FUSE_ASAX, 
			sizeof(akm->asa), akm->asa);
	if(err != sizeof(akm->asa)){
		pr_err("%s: unable to read the sensitivity adjust values\n",__func__);
	}else{
		pr_debug("%s: asa_x = %d, asa_y = %d, asa_z = %d \n",__func__, akm->asa[0],
			akm->asa[1],akm->asa[2]);
	}
	
	err = AKECS_SetMode_PowerDown(akm);
	if(err < 0){
		pr_err("%s:unable to set power down mode\n",__func__);	
	}

	udelay(100);
		
	ret = sprintf(buf, "%d, %d, %d\n", akm->asa[0], akm->asa[1], akm->asa[2]);
	
	return ret;
}

static ssize_t akm_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	char name[32] = "compass(akm8963)";

	ret = sprintf(buf, "%s\n", name);
	
	return ret;
}

static ssize_t akm_self_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct s_xyz sxyz = {0,};
	struct akm8963_data *s_akm = dev_get_drvdata(dev);

	ret = akm_self_test(s_akm, &sxyz);
	
	ret = sprintf(buf,"%d,%d, %d, %d\n",ret,sxyz.x,sxyz.y,sxyz.z);
	return ret; 	

}

struct device_attribute attributes[] = {
	__ATTR(akm8963_mode, 0644, NULL, akm_mode_store),
	__ATTR(akm8963_data, 0644, akm_data_show, NULL),
	__ATTR(akm8963_name, 0644, akm_name_show, NULL),
	__ATTR(akm8963_asa_data, 0644, akm_asa_show, NULL),
	__ATTR(akm8963_self_test, 0644, akm_self_show, NULL),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for(i = 0; i < ARRAY_SIZE(attributes); i++)
		if(device_create_file(dev, attributes + i))
			goto err;
	return 0;

err:
	for(i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	dev_err(dev,"%s():create sysfs interface error\n",__func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for(i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void akm8963_create_input(struct akm8963_data *akm)
{
	int err;
	/* Declare input device */
	akm->input_dev = input_allocate_device();
	if (!akm->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "AKM8963 akm8963_probe: Failed to allocate input device\n");
	}
	/* Setup input device */
	set_bit(EV_ABS, akm->input_dev->evbit);
	/* yaw (0, 360) */
	input_set_abs_params(akm->input_dev, ABS_RX, 0, 23040, 0, 0);
	/* pitch (-180, 180) */
	input_set_abs_params(akm->input_dev, ABS_RY, -11520, 11520, 0, 0);
	/* roll (-90, 90) */
	input_set_abs_params(akm->input_dev, ABS_RZ, -5760, 5760, 0, 0);
	
	/* x-axis acceleration (720 x 8G) */
	input_set_abs_params(akm->input_dev, ABS_X, -5760, 5760, 0, 0);
	/* y-axis acceleration (720 x 8G) */
	input_set_abs_params(akm->input_dev, ABS_Y, -5760, 5760, 0, 0);
	/* z-axis acceleration (720 x 8G) */
	input_set_abs_params(akm->input_dev, ABS_Z, -5760, 5760, 0, 0);

	/* status of magnetic sensor */
	input_set_abs_params(akm->input_dev, ABS_RUDDER, -32768, 3, 0, 0);
	/* status of acceleration sensor */
	input_set_abs_params(akm->input_dev, ABS_WHEEL, -32768, 3, 0, 0);
	/* x-axis of raw magnetic vector (-4096, 4095) */
	input_set_abs_params(akm->input_dev, ABS_HAT0X, -20480, 20479, 0, 0);
	/* y-axis of raw magnetic vector (-4096, 4095) */
	input_set_abs_params(akm->input_dev, ABS_HAT0Y, -20480, 20479, 0, 0);
	/* z-axis of raw magnetic vector (-4096, 4095) */
	input_set_abs_params(akm->input_dev, ABS_BRAKE, -20480, 20479, 0, 0);
	
	akm->input_dev->name = "compass";
	akm->input_dev->dev.parent = &akm->client->dev;
	
	err = input_register_device(akm->input_dev);
	if (err) {
		printk(KERN_ERR
		       "AKM8963 akm8963_probe: Unable to register input device\n");
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void akm8963_early_suspend(struct early_suspend *handler)
{
	struct akm8963_data *akm = container_of(handler, struct akm8963_data, 
			akm_early_suspend);

	AKMFUNC("akm8963_early_suspend");
	atomic_set(&suspend_flag, 1);
	atomic_set(&akm->reserve_open_flag, atomic_read(&akm->open_flag));
	atomic_set(&akm->open_flag, 0);
	wake_up(&akm->open_wq);
	AKMDBG("suspended with flag=%d", 
	       atomic_read(&akm->reserve_open_flag));
}

static void akm8963_early_resume(struct early_suspend *handler)
{
	struct akm8963_data *akm = container_of(handler, struct akm8963_data,
		       	akm_early_suspend);
	
	AKMFUNC("akm8963_early_resume");
	atomic_set(&akm->open_flag, atomic_read(&akm->reserve_open_flag));
	wake_up(&akm->open_wq);
	AKMDBG("resumed with flag=%d", 
	atomic_read(&akm->reserve_open_flag));
	atomic_set(&suspend_flag, 0);
}
#endif

static int akm8963_suspend(struct device *dev)
{
	struct akm8963_data *akm = dev_get_drvdata(dev);

	/* Powerdown it in suspend */
	AKECS_SetMode_PowerDown(akm);
	return 0;
}

static int akm8963_resume(struct device *dev)
{
	return 0;
}

static struct file_operations akmd_fops = {
	.owner = THIS_MODULE,
	.open = akmd_open,
	.release = akmd_release,
	.unlocked_ioctl = akmd_ioctl,
};

static int __devinit akm8963_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct akm8963_data *akm;
	int err = 0;
	
	pr_info("%s(),the client address is 0x%02x\n",__func__, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "AKM8963 akm8963_probe: check_functionality failed.\n");
		err = -ENODEV;
		goto exit0;
	}
		
	/* Allocate memory for driver data */
	akm = kzalloc(sizeof(struct akm8963_data), GFP_KERNEL);
	if (!akm) {
		printk(KERN_ERR "AKM8963 akm8963_probe: memory allocation failed.\n");
		err = -ENOMEM;
		goto exit0;
	}
	mutex_init(&akm->akm_aot_ioctl_lock);
	mutex_init(&akm->akmd_ioctl_lock);

	i2c_set_clientdata(client, akm);
	akm->client = client;
	akm->test_flag = 0;
	
	/* Check connection */
	err = AKECS_CheckDevice(akm);
	if (err < 0) {
		printk(KERN_ERR "AKM8963 check device error\n");
		goto exit1;
	}

	/* register input device*/
	akm8963_create_input(akm);
	
	/*create sysfs interface*/
	err = create_sysfs_interfaces(&client->dev);
	if(err < 0){
		dev_err(&client->dev, "create interface error\n");
		remove_sysfs_interfaces(&client->dev);
	}
	
	akm->akmd_device.minor = MISC_DYNAMIC_MINOR;
	akm->akmd_device.name = "akm8975_dev";
	akm->akmd_device.fops = &akmd_fops;

	err = misc_register(&akm->akmd_device);
	if (err) {
		printk(KERN_ERR
			   "AKM8963 akm8963_probe: akmd_device register failed\n");
		goto exit3;
	}
	
	init_waitqueue_head(&akm->open_wq);

	/*In order to read correct data from fuse rom,we should set softreset*/
	AKECS_Softreset(akm, 1);

	/*ASAX, ASAY, ASAZ: sensitivity adjustment values read from Fuse ROM*/
	err = AKECS_SetMode_FUSEAccess(akm);	
	if(err < 0){
		pr_err("%s:unable to set fuse rom mode\n",__func__);	
	}

	err = i2c_smbus_read_i2c_block_data(client, AK8963_FUSE_ASAX, sizeof(akm->asa), akm->asa);
	if(err != sizeof(akm->asa)){
		pr_err("%s: unable to read the sensitivity adjust values\n",__func__);
	}else{
	
		pr_info("%s: asa_x = %d, asa_y = %d, asa_z = %d \n",__func__, akm->asa[0],
			akm->asa[1],akm->asa[2]);
	}
	
	/*default don't use softreset*/
	AKECS_Softreset(akm, 0);

	err = AKECS_SetMode_PowerDown(akm);
	if(err < 0){
		pr_err("%s:unable to set power down mode\n",__func__);	
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	akm->akm_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	akm->akm_early_suspend.suspend = akm8963_early_suspend;
	akm->akm_early_suspend.resume = akm8963_early_resume;
	register_early_suspend(&akm->akm_early_suspend);
#endif

	AKMDBG("successfully probed."); 
	return 0;
	
exit3:
	input_unregister_device(akm->input_dev);
exit1:
	kfree(akm);
exit0:
	return err;
}

static int __devexit akm8963_remove(struct i2c_client *client)
{
	struct akm8963_data *akm = i2c_get_clientdata(client);
	AKMFUNC("akm8963_remove");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&akm->akm_early_suspend);
#endif
	
	misc_deregister(&akm->akmd_device);
	input_unregister_device(akm->input_dev);
	remove_sysfs_interfaces(&client->dev);
	kfree(akm);
	AKMDBG("successfully removed.");
	return 0;
}

static void akm8963_shutdown(struct i2c_client *client)
{
	struct akm8963_data *akm = i2c_get_clientdata(client);
	AKECS_SetMode_PowerDown(akm);
}

static const struct i2c_device_id akm8963_id[] = {
	{AKM8963_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_PM
static const struct dev_pm_ops akm8963_pm_ops = {
	.suspend = akm8963_suspend,
	.resume = akm8963_resume,
};
#endif

static struct i2c_driver akm8963_driver = {
	.probe		= akm8963_probe,
	.shutdown	= akm8963_shutdown,
	.remove		= __devexit_p(akm8963_remove),
	.id_table	= akm8963_id,
	.driver = {
		.name = AKM8963_I2C_NAME,
#ifdef CONFIG_PM	
		.pm = &akm8963_pm_ops,
#endif	
	},
};

static int __init akm8963_init(void)
{
	return i2c_add_driver(&akm8963_driver);
}

static void __exit akm8963_exit(void)
{
	i2c_del_driver(&akm8963_driver);
}

module_init(akm8963_init);
module_exit(akm8963_exit);

MODULE_AUTHOR("viral wang <viral_wang@htc.com>");
MODULE_DESCRIPTION("AKM8963 compass driver");
MODULE_LICENSE("GPL");

