/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * Initial Code:
 *	Robbie Cao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*
 * Definitions for MXC400X accelorometer sensor chip.
 */
#ifndef __MXC400X_H__
#define __MXC400X_H__


#include	<linux/ioctl.h>	/* For IOCTL macros */
#include	<linux/input.h>



/************************************************/
/* 	Accelerometer defines section	 	*/
/************************************************/
#define MXC400X_DEV_NAME		"MXC400X"

#define MXC400X_I2C_ADDR     	0x15
#define MXC400X_ID				0x02	

/* MXC400X register address */
#define MXC400X_REG_X			0x03
#define MXC400X_REG_Y			0x05
#define MXC400X_REG_Z			0x07
#define MXC400X_REG_TEMP        0x09
#define MXC400X_REG_CTRL		0x0D
#define MXC400X_REG_ID          0x0E
//#define MXC400X_REG_FAC         0x0E
//#define MXC400X_REG_FSRE        0x17
#define MXC400X_REG_TEST1        0x13
#define MXC400X_REG_TEST2        0x14
#define MXC400X_REG_TEST3        0x15 


/*para setting*/
//#define MXC400X_PASSWORD        0x93
//#define MXC400X_RANGE_8G        0x5B    /*full scale range +/-8g*/
#define MXC400X_AWAKE		0x40	/* power on */
#define MXC400X_SLEEP		0x01	/* power donw */

#define MXC400X_BW_50HZ    0x00
#define MXC400X_RANGE_2G   0x00
#define MXC400X_RANGE_4G   0x20
#define MXC400X_RANGE_8G   0x40


/*ERR code*/
#define MXC400X_SUCCESS						0
#define MXC400X_ERR_I2C						-1
#define MXC400X_ERR_STATUS					-3
#define MXC400X_ERR_SETUP_FAILURE			-4
#define MXC400X_ERR_GETGSENSORDATA			-5
#define MXC400X_ERR_IDENTIFICATION			-6

#define MXC400X_BUFSIZE				256

#if defined(C2_PROJ)
#define ACC_USE_SYS_I2C
#endif

#ifdef	__KERNEL__
struct mxc400x_acc_platform_data {
	int poll_interval;
	int min_interval;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif	/* __KERNEL__ */

#endif /* __MXC400X_H__ */

