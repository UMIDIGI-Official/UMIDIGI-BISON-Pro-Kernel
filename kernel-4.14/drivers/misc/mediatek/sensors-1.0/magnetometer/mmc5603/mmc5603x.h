/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information and source code
 *  contained herein is confidential. The software including the source code
 *  may not be copied and the information contained herein may not be used or
 *  disclosed except with the written permission of MEMSIC Inc. (C) 2017
 *****************************************************************************/
 
/*
 * mmc5603x.h - Definitions for mmc5603x magnetic sensor chip.
 */
 
 
#ifndef __MMC5603x_H__
#define __MMC5603x_H__

#include <linux/ioctl.h>

#define CALIBRATION_DATA_SIZE	12
#define SENSOR_DATA_SIZE 	9
#ifndef TRUE
	#define TRUE 		1
#endif
#define MMC5603x_I2C_ADDR	0x30	// 8-bit  0x60
#define MMC5603x_DEVICE_ID	0x10

/* MMC5603x register address */
#define MMC5603X_REG_CTRL0			0x1B
#define MMC5603X_REG_CTRL1			0x1C
#define MMC5603X_REG_CTRL2			0x1D
#define MMC5603X_REG_ODR			0x1A
#define MMC5603X_REG_BITS		0x09
#define MMC5603X_REG_DATA		0x00
#define MMC5603X_REG_DS			0x07
#define MMC5603X_REG_PRODUCTID		0x39

/* MMC5603x control bit */
#define MMC5603X_CTRL_TM		0x01
#define MMC5603X_CTRL_CM		0x02
#define MMC5603X_CTRL_50HZ		0x00
#define MMC5603X_CTRL_25HZ		0x04
#define MMC5603X_CTRL_12HZ		0x08
#define MMC5603X_CTRL_SET  	        0x08
#define MMC5603X_CTRL_RESET             0x10
#define MMC5603X_CTRL_REFILL            0x20

#define MMC5603X_CMD_BW_01         0x01
#define MMC5603X_CMD_AUTO_SR	    0xa0
#define MMC5603X_CMD_CM            0x10
#define MMC5603X_SAMPLE_RATE  100

// conversion of magnetic data to uT units
// 32768 = 1Guass = 100 uT
// 100 / 32768 = 25 / 8192
// 65536 = 360Degree
// 360 / 65536 = 45 / 8192


//#define CONVERT_M_DIV			8192
#define CONVERT_M_DIV			1024

// sensitivity 1024 count = 1 Guass = 100uT

#define MMC5603X_OFFSET_X		32768
#define MMC5603X_OFFSET_Y		32768
#define MMC5603X_OFFSET_Z		32768
#define MMC5603X_SENSITIVITY_X		1024
#define MMC5603X_SENSITIVITY_Y		1024
#define MMC5603X_SENSITIVITY_Z		1024


#if 0
#define ECOMPASS_IOC_SET_YPR            	_IOW(MSENSOR, 0x21, int[CALIBRATION_DATA_SIZE])
#define COMPAT_ECOMPASS_IOC_SET_YPR            _IOW(MSENSOR, 0x21, compat_int_t[CALIBRATION_DATA_SIZE])
#endif
#define COMPAT_MMC5603X_IOC_READ_REG           _IOWR(MSENSOR, 0x32, unsigned char)
#define COMPAT_MMC5603X_IOC_WRITE_REG          _IOW(MSENSOR, 0x33, unsigned char[2])
#define COMPAT_MMC5603X_IOC_READ_REGS          _IOWR(MSENSOR, 0x34, unsigned char[10])
#define MMC5603X_IOC_READ_REG		    _IOWR(MSENSOR, 0x23, unsigned char)
#define MMC5603X_IOC_WRITE_REG		    _IOW(MSENSOR,  0x24, unsigned char[2])
#define MMC5603X_IOC_READ_REGS		    _IOWR(MSENSOR, 0x25, unsigned char[10])

#endif /* __MMC5603x_H__ */

