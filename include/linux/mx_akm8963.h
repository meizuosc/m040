/*
 * Definitions for akm8963 compass chip.
 */
#ifndef AKM8963_H
#define AKM8963_H

#include <linux/ioctl.h>

#define AKM8963_I2C_NAME "akm8963"

/*Defines an operation mode and output bit setting of the AK8963.
  BIT:"0" 14-bit output ; "1" 16-bit putput
  the bits of output affect the sensitivity.
  AK8963N: 16bit
 */
#define AKM8963_OUTPUT_BIT_SET_16           1 << 4
#define AKM8963_OUTPUT_BIT_SET_14           0 << 4
#define AK8963_MODE_SNG_MEASURE		    0x01
#define AK8963_MODE_CONT1_MEASURE	    0x02
#define AK8963_MODE_CONT2_MEASURE	    0x06
#define AK8963_MODE_TRIGGER_MEASURE	    0x04
#define	AK8963_MODE_SELF_TEST		    0x08
#define	AK8963_MODE_FUSE_ACCESS		    0x0F
#define	AK8963_MODE_POWERDOWN		    0x00

#define SENSOR_DATA_SIZE                    8	  /* Rx buffer size, i.e from ST1 to ST2 */
#define RWBUF_SIZE			    16	  /* Read/Write buffer size.*/

/*Defines a register address of the AK8963.*/
#define AK8963_REG_WIA			0x00 /*read-only*/
#define AK8963_REG_INFO			0x01 /*read-only*/
#define AK8963_REG_ST1			0x02 /*read-only*/
#define AK8963_REG_HXL			0x03
#define AK8963_REG_HXH			0x04
#define AK8963_REG_HYL			0x05
#define AK8963_REG_HYH			0x06
#define AK8963_REG_HZL			0x07
#define AK8963_REG_HZH			0x08
#define AK8963_REG_ST2			0x09
#define AK8963_REG_CNTL1		0x0A
#define AK8963_REG_CNTL2		0x0B /*soft reset*/
#define AK8963_REG_ASTC			0x0C /*self-test*/
#define AK8963_REG_TS1			0x0D /*do not access*/
#define AK8963_REG_TS2			0x0E /*do not access*/
#define AK8963_REG_I2CDIS		0x0F
#define AK8963_REG_RSV			0x13 /*do not access*/

/*Defines a read-only address of the fuse ROM of the AK8963.*/
#define AK8963_FUSE_ASAX	0x10
#define AK8963_FUSE_ASAY	0x11
#define AK8963_FUSE_ASAZ	0x12

#define AKMIO                   0xA1

/* IOCTLs for AKM library */
#define ECS_IOCTL_WRITE                 _IOW(AKMIO, 0x01, char*)
#define ECS_IOCTL_READ                  _IOWR(AKMIO, 0x02, char*)
#define ECS_IOCTL_RESET      	        _IO(AKMIO, 0x03) /* NOT used in AK8963 */
#define ECS_IOCTL_SET_MODE              _IOW(AKMIO, 0x04, short)
#define ECS_IOCTL_GETDATA               _IOR(AKMIO, 0x05, char[SENSOR_DATA_SIZE])
#define ECS_IOCTL_SET_YPR               _IOW(AKMIO, 0x06, short[12])
#define ECS_IOCTL_GET_OPEN_STATUS       _IOR(AKMIO, 0x07, int)
#define ECS_IOCTL_GET_CLOSE_STATUS      _IOR(AKMIO, 0x08, int)
#define ECS_IOCTL_GET_DELAY             _IOR(AKMIO, 0x30, short)
#define ECS_IOCTL_GET_PROJECT_NAME      _IOR(AKMIO, 0x0D, char[64])
#define ECS_IOCTL_GET_MATRIX            _IOR(AKMIO, 0x0E, short [4][3][3])
#define ECS_IOCTL_GET_SUSPEND_STATUS	_IOR(AKMIO, 0x09, int)

/* IOCTLs for APPs */
#define ECS_IOCTL_APP_SET_MODE		_IOW(AKMIO, 0x10, short)
#define ECS_IOCTL_APP_SET_MFLAG		_IOW(AKMIO, 0x11, short)
#define ECS_IOCTL_APP_GET_MFLAG		_IOW(AKMIO, 0x12, short)
#define ECS_IOCTL_APP_SET_AFLAG		_IOW(AKMIO, 0x13, short)
#define ECS_IOCTL_APP_GET_AFLAG		_IOR(AKMIO, 0x14, short)
#define ECS_IOCTL_APP_SET_TFLAG		_IOR(AKMIO, 0x15, short)/* NOT use */
#define ECS_IOCTL_APP_GET_TFLAG		_IOR(AKMIO, 0x16, short)/* NOT use */
#define ECS_IOCTL_APP_RESET_PEDOMETER   _IO(AKMIO, 0x17)	/* NOT use */
#define ECS_IOCTL_APP_SET_DELAY		_IOW(AKMIO, 0x18, short)
#define ECS_IOCTL_APP_GET_DELAY		ECS_IOCTL_GET_DELAY
#define ECS_IOCTL_APP_SET_MVFLAG	_IOW(AKMIO, 0x19, short)
#define ECS_IOCTL_APP_GET_MVFLAG	_IOR(AKMIO, 0x1A, short)

struct akm8963_platform_data {
	char layouts[3][3];
	char project_name[64];
	int gpio_DRDY;
};

#endif

