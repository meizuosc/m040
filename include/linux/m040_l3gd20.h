#ifndef __L3GD20_H__
#define __L3GD20_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

/* L3GD20 registers */
#undef FIFO_CTRL_REG
#define FIFO_CTRL_REG (0x2E)
#define FIFO_SRC_REG (0x2F)

#define FIFO_MODE_SHIFT (5)
#define FIFO_BYPASS_MODE (0<<FIFO_MODE_SHIFT)
#define FIFO_FIFO_MODE (1<<FIFO_MODE_SHIFT)
#define FIFO_STREAM_MODE (2<<FIFO_MODE_SHIFT)
#define FIFO_STREAM2FIFO_MODE (3<<FIFO_MODE_SHIFT)
#define FIFO_BYPASS2STREAM_MODE (4<<FIFO_MODE_SHIFT)

#define L3G4200D_IOCTL_BASE 'g'
/* The following define the IOCTL command values via the ioctl macros */
#define L3G4200D_SELFTEST		_IOW(L3G4200D_IOCTL_BASE, 0, int)
#define L3G4200D_SET_RANGE		_IOW(L3G4200D_IOCTL_BASE, 1, int)
#define L3G4200D_SET_MODE		_IOW(L3G4200D_IOCTL_BASE, 2, int)
#define L3G4200D_SET_BANDWIDTH		_IOW(L3G4200D_IOCTL_BASE, 3, int)
#define L3G4200D_READ_GYRO_VALUES	_IOW(L3G4200D_IOCTL_BASE, 4, int)
#define L3G4200D_GET_SUSPEND_STATUS	_IOW(L3G4200D_IOCTL_BASE, 9, int)

/* add by jerrymo */
#define L3G4200D_SET_ENABLE _IOW(L3G4200D_IOCTL_BASE, 5, int)
#define L3G4200D_GET_ENABLE _IOR(L3G4200D_IOCTL_BASE, 6, int)
#define L3G4200D_SET_DELAY _IOW(L3G4200D_IOCTL_BASE, 7, int64_t)
#define L3G4200D_GET_DELAY _IOR(L3G4200D_IOCTL_BASE, 8, int64_t)
/*end add*/

#define L3GD20_GET_TEMP _IOR(L3G4200D_IOCTL_BASE, 10, int)

#define L3G4200D_FS_250DPS	0x00
#define L3G4200D_FS_500DPS	0x10
#define L3G4200D_FS_2000DPS	0x30

#define PM_OFF		0x00
#define PM_NORMAL	0x08
#define ENABLE_ALL_AXES	0x07

/* Added by qudao, for L3G2D */
#define ODR95_BW12_5	(0x00)	/* ODR = 95Hz; BW = 12.5Hz */
#define ODR95_BW25		(0x10)
#define ODR190_BW12_5	(0x40)
#define ODR190_BW25	(0x50)
#define ODR190_BW50	(0x60)
#define ODR190_BW70	(0x70)
#define ODR380_BW20	(0x80)
#define ODR380_BW25	(0x90)
#define ODR380_BW50	(0xA0)
#define ODR380_BW100	(0xB0)
#define ODR760_BW30	(0xC0)
#define ODR760_BW35	(0xD0)
#define ODR760_BW50	(0xE0)
#define ODR760_BW100	(0xF0)

#ifdef __KERNEL__
struct l3g4200d_platform_data {
	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

#endif /* __KERNEL__ */

#define GYROSCPOE_CHIP_ID (0x00D4)
#define GYROSCPOE_CHIP_NAME "l3gd20"

#endif  /* __L3GD20_H__ */
