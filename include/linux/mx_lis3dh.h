#ifndef	__LIS3DH_H__
#define	__LIS3DH_H__

#include <linux/ioctl.h>

#define	LIS3DH_ACC_DEV_NAME	"lis3dh"

/************************************************/
/* 	Accelerometer defines section	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define	LIS3DH_ACC_FS_MASK		0x30
#define LIS3DH_ACC_G_2G 		0x00
#define LIS3DH_ACC_G_4G 		0x10
#define LIS3DH_ACC_G_8G 		0x20
#define LIS3DH_ACC_G_16G		0x30

/*ioctl cmds*/
#define LIS3DH_IOCTL_BASE 'l'
#define LIS3DH_IOCTL_SET_ENABLE 	_IOW(LIS3DH_IOCTL_BASE, 0, int)
#define LIS3DH_IOCTL_GET_ENABLE 	_IOR(LIS3DH_IOCTL_BASE, 1, int)
#define LIS3DH_IOCTL_SET_DELAY	   	_IOW(LIS3DH_IOCTL_BASE, 2, int)
#define LIS3DH_IOCTL_GET_DELAY		_IOR(LIS3DH_IOCTL_BASE, 3, int)
#define LIS3DH_IOCTL_READ_ACCEL_XYZ  _IOR(LIS3DH_IOCTL_BASE, 4, int[3])
#define LIS3DH_IOCTL_GET_TEMP		_IOR(LIS3DH_IOCTL_BASE, 5, int)
#define LIS3DH_IOCTL_SELFTEST		_IOR(LIS3DH_IOCTL_BASE, 6, int)
#define LIS3DH_IOCTL_GET_SUSPEND_STATUS _IOR(LIS3DH_IOCTL_BASE, 7, int)

#ifdef	__KERNEL__
struct lis3dh_acc_platform_data {
	int poll_interval;
	int min_interval;

	u8 g_range;

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
#endif	/* __KERNEL__ */

#endif	/* __LIS3DH_H__ */
