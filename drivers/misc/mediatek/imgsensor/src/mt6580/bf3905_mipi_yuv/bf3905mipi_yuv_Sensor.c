/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information contained
 *  herein is confidential. The software may not be copied and the information
 *  contained herein may not be used or disclosed except with the written
 *  permission of MediaTek Inc. (C) 2005
 *
 *  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 *  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
 *  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 *  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 *  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 *  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
 *  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
 *  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
 *  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 *  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 *  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
 *  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
 *  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
 *  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
 *  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
 *  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
 *
 *****************************************************************************/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   bf3905yuv_Sensor.c
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *   V1.1.0
 *
 * Author:
 * -------
 *   travis
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 *   
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "bf3905mipi_yuv_Sensor.h"
#include "bf3905mipi_yuv_Camera_Sensor_para.h"
#include "bf3905mipi_yuv_CameraCustomized.h"

//#define BF3905MIPIYUV_DEBUG

#define BF3905MIPI_TEST_PATTERN_CHECKSUM 0x4e908d1c

#ifdef BF3905MIPIYUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

//#define DEBUG_SENSOR_BF3905MIPI//T_flash Tuning
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);






kal_uint16 BF3905MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char puSendCmd[1] = {(char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 1, (u8*)&get_byte,1,BF3905MIPI_WRITE_ID);
	SENSORDB("BF3905_MIPI_read_cmos_sensor reg 0x%x = 0x%x\n",addr,get_byte);
	return get_byte;

}

inline void BF3905MIPI_write_cmos_sensor(u16 addr, u32 para)
{
   char puSendCmd[2] = {(char)(addr & 0xFF) ,((char)(para & 0xFF))};

   iWriteRegI2C(puSendCmd , 2,BF3905MIPI_WRITE_ID);
   
   SENSORDB("BF3905_MIPI_write_cmos_sensor reg 0x%x = 0x%x\n",addr,para);
}

/*******************************************************************************
 * // Adapter for Winmo typedef
 ********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

kal_bool   BF3905MIPI_MPEG4_encode_mode = KAL_FALSE;
kal_uint16 BF3905MIPI_dummy_pixels = 0, BF3905MIPI_dummy_lines = 0;
kal_bool   BF3905MIPI_MODE_CAPTURE = KAL_FALSE;
kal_bool   BF3905MIPI_NIGHT_MODE = KAL_FALSE;

kal_uint32 BF3905MIPI_isp_master_clock;
static kal_uint32 BF3905MIPI_g_fPV_PCLK = 26;
static kal_uint32  BF3905MIPI_sensor_pclk=260;
kal_uint8 BF3905MIPI_sensor_write_I2C_address = BF3905MIPI_WRITE_ID;
kal_uint8 BF3905MIPI_sensor_read_I2C_address = BF3905MIPI_READ_ID;

UINT8 BF3905MIPIPixelClockDivider=0;

MSDK_SENSOR_CONFIG_STRUCT BF3905MIPISensorConfigData;

/*************************************************************************
 * FUNCTION
 *	BF3905MIPI_SetShutter
 *
 * DESCRIPTION
 *	This function set e-shutter of BF3905MIPI to change exposure time.
 *
 * PARAMETERS
 *   iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void BF3905MIPI_Set_Shutter(kal_uint16 iShutter)
{
} /* Set_BF3905MIPI_Shutter */


/*************************************************************************
 * FUNCTION
 *	BF3905MIPI_read_Shutter
 *
 * DESCRIPTION
 *	This function read e-shutter of BF3905MIPI .
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 BF3905MIPI_Read_Shutter(void)
{
    	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 shutter;

	temp_reg1 = BF3905MIPI_read_cmos_sensor(0x8d);
	temp_reg2 = BF3905MIPI_read_cmos_sensor(0x8c);

	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

	return shutter;
} /* BF3905MIPI_read_shutter */


/*************************************************************************
 * FUNCTION
 *	BF3905MIPI_write_reg
 *
 * DESCRIPTION
 *	This function set the register of BF3905MIPI.
 *
 * PARAMETERS
 *	addr : the register index of BF3905MIPI
 *  para : setting parameter of the specified register of BF3905MIPI
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void BF3905MIPI_write_reg(kal_uint32 addr, kal_uint32 para)
{
	BF3905MIPI_write_cmos_sensor(addr, para);
} /* BF3905MIPI_write_reg() */


/*************************************************************************
 * FUNCTION
 *	BF3905MIPI_read_cmos_sensor
 *
 * DESCRIPTION
 *	This function read parameter of specified register from BF3905MIPI.
 *
 * PARAMETERS
 *	addr : the register index of BF3905MIPI
 *
 * RETURNS
 *	the data that read from BF3905MIPI
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint32 BF3905MIPI_read_reg(kal_uint32 addr)
{
	return BF3905MIPI_read_cmos_sensor(addr);
} /* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	BF3905MIPI_awb_enable
*
* DESCRIPTION
*	This function enable or disable the awb (Auto White Balance).
*
* PARAMETERS
*	1. kal_bool : KAL_TRUE - enable awb, KAL_FALSE - disable awb.
*
* RETURNS
*	kal_bool : It means set awb right or not.
*
*************************************************************************/
static void BF3905MIPI_awb_enable(kal_bool enalbe)
{	 
	kal_uint16 temp_AWB_reg = 0;

	temp_AWB_reg = BF3905MIPI_read_cmos_sensor(0x13);
	
	if (enalbe)
	{
		BF3905MIPI_write_cmos_sensor(0x13, (temp_AWB_reg |0x02));
	}
	else
	{
		BF3905MIPI_write_cmos_sensor(0x13, (temp_AWB_reg & (~0x02)));
	}

}


/*************************************************************************
* FUNCTION
*	BF3905MIPI_GAMMA_Select
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate GAMMA curve.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void BF3905MIPIGammaSelect(kal_uint32 GammaLvl)
{
	switch(GammaLvl)
		{
		case BF3905MIPI_RGB_Gamma_m1:						//smallest gamma curve
		BF3905MIPI_write_cmos_sensor(0x40,0x3b);
		BF3905MIPI_write_cmos_sensor(0x41,0x36);
		BF3905MIPI_write_cmos_sensor(0x42,0x2b);
		BF3905MIPI_write_cmos_sensor(0x43,0x1d);
		BF3905MIPI_write_cmos_sensor(0x44,0x1a);
		BF3905MIPI_write_cmos_sensor(0x45,0x14);
		BF3905MIPI_write_cmos_sensor(0x46,0x11);
		BF3905MIPI_write_cmos_sensor(0x47,0x0e);
		BF3905MIPI_write_cmos_sensor(0x48,0x0d);
		BF3905MIPI_write_cmos_sensor(0x49,0x0c);
		BF3905MIPI_write_cmos_sensor(0x4b,0x0b);
		BF3905MIPI_write_cmos_sensor(0x4c,0x09);
		BF3905MIPI_write_cmos_sensor(0x4e,0x08);
		BF3905MIPI_write_cmos_sensor(0x4f,0x07);
		BF3905MIPI_write_cmos_sensor(0x50,0x07);

		break;
		case BF3905MIPI_RGB_Gamma_m2:
		BF3905MIPI_write_cmos_sensor(0x40,0x3b);
		BF3905MIPI_write_cmos_sensor(0x41,0x36);
		BF3905MIPI_write_cmos_sensor(0x42,0x2b);
		BF3905MIPI_write_cmos_sensor(0x43,0x1d);
		BF3905MIPI_write_cmos_sensor(0x44,0x1a);
		BF3905MIPI_write_cmos_sensor(0x45,0x14);
		BF3905MIPI_write_cmos_sensor(0x46,0x11);
		BF3905MIPI_write_cmos_sensor(0x47,0x0e);
		BF3905MIPI_write_cmos_sensor(0x48,0x0d);
		BF3905MIPI_write_cmos_sensor(0x49,0x0c);
		BF3905MIPI_write_cmos_sensor(0x4b,0x0b);
		BF3905MIPI_write_cmos_sensor(0x4c,0x09);
		BF3905MIPI_write_cmos_sensor(0x4e,0x08);
		BF3905MIPI_write_cmos_sensor(0x4f,0x07);
		BF3905MIPI_write_cmos_sensor(0x50,0x07);

		break;

		case BF3905MIPI_RGB_Gamma_m3:			
		BF3905MIPI_write_cmos_sensor(0x40,0x3b);
		BF3905MIPI_write_cmos_sensor(0x41,0x36);
		BF3905MIPI_write_cmos_sensor(0x42,0x2b);
		BF3905MIPI_write_cmos_sensor(0x43,0x1d);
		BF3905MIPI_write_cmos_sensor(0x44,0x1a);
		BF3905MIPI_write_cmos_sensor(0x45,0x14);
		BF3905MIPI_write_cmos_sensor(0x46,0x11);
		BF3905MIPI_write_cmos_sensor(0x47,0x0e);
		BF3905MIPI_write_cmos_sensor(0x48,0x0d);
		BF3905MIPI_write_cmos_sensor(0x49,0x0c);
		BF3905MIPI_write_cmos_sensor(0x4b,0x0b);
		BF3905MIPI_write_cmos_sensor(0x4c,0x09);
		BF3905MIPI_write_cmos_sensor(0x4e,0x08);
		BF3905MIPI_write_cmos_sensor(0x4f,0x07);
		BF3905MIPI_write_cmos_sensor(0x50,0x07);

		break;

		case BF3905MIPI_RGB_Gamma_m4:
		BF3905MIPI_write_cmos_sensor(0x40,0x3b);
		BF3905MIPI_write_cmos_sensor(0x41,0x36);
		BF3905MIPI_write_cmos_sensor(0x42,0x2b);
		BF3905MIPI_write_cmos_sensor(0x43,0x1d);
		BF3905MIPI_write_cmos_sensor(0x44,0x1a);
		BF3905MIPI_write_cmos_sensor(0x45,0x14);
		BF3905MIPI_write_cmos_sensor(0x46,0x11);
		BF3905MIPI_write_cmos_sensor(0x47,0x0e);
		BF3905MIPI_write_cmos_sensor(0x48,0x0d);
		BF3905MIPI_write_cmos_sensor(0x49,0x0c);
		BF3905MIPI_write_cmos_sensor(0x4b,0x0b);
		BF3905MIPI_write_cmos_sensor(0x4c,0x09);
		BF3905MIPI_write_cmos_sensor(0x4e,0x08);
		BF3905MIPI_write_cmos_sensor(0x4f,0x07);
		BF3905MIPI_write_cmos_sensor(0x50,0x07);

		break;

		case BF3905MIPI_RGB_Gamma_m5:
		BF3905MIPI_write_cmos_sensor(0x40,0x3b);
		BF3905MIPI_write_cmos_sensor(0x41,0x36);
		BF3905MIPI_write_cmos_sensor(0x42,0x2b);
		BF3905MIPI_write_cmos_sensor(0x43,0x1d);
		BF3905MIPI_write_cmos_sensor(0x44,0x1a);
		BF3905MIPI_write_cmos_sensor(0x45,0x14);
		BF3905MIPI_write_cmos_sensor(0x46,0x11);
		BF3905MIPI_write_cmos_sensor(0x47,0x0e);
		BF3905MIPI_write_cmos_sensor(0x48,0x0d);
		BF3905MIPI_write_cmos_sensor(0x49,0x0c);
		BF3905MIPI_write_cmos_sensor(0x4b,0x0b);
		BF3905MIPI_write_cmos_sensor(0x4c,0x09);
		BF3905MIPI_write_cmos_sensor(0x4e,0x08);
		BF3905MIPI_write_cmos_sensor(0x4f,0x07);
		BF3905MIPI_write_cmos_sensor(0x50,0x07);

		break;

		case BF3905MIPI_RGB_Gamma_m6:										// largest gamma curve
		BF3905MIPI_write_cmos_sensor(0x40,0x3b);
		BF3905MIPI_write_cmos_sensor(0x41,0x36);
		BF3905MIPI_write_cmos_sensor(0x42,0x2b);
		BF3905MIPI_write_cmos_sensor(0x43,0x1d);
		BF3905MIPI_write_cmos_sensor(0x44,0x1a);
		BF3905MIPI_write_cmos_sensor(0x45,0x14);
		BF3905MIPI_write_cmos_sensor(0x46,0x11);
		BF3905MIPI_write_cmos_sensor(0x47,0x0e);
		BF3905MIPI_write_cmos_sensor(0x48,0x0d);
		BF3905MIPI_write_cmos_sensor(0x49,0x0c);
		BF3905MIPI_write_cmos_sensor(0x4b,0x0b);
		BF3905MIPI_write_cmos_sensor(0x4c,0x09);
		BF3905MIPI_write_cmos_sensor(0x4e,0x08);
		BF3905MIPI_write_cmos_sensor(0x4f,0x07);
		BF3905MIPI_write_cmos_sensor(0x50,0x07);

		break;
		case BF3905MIPI_RGB_Gamma_night:									//Gamma for night mode
		BF3905MIPI_write_cmos_sensor(0x40,0x3b);
		BF3905MIPI_write_cmos_sensor(0x41,0x36);
		BF3905MIPI_write_cmos_sensor(0x42,0x2b);
		BF3905MIPI_write_cmos_sensor(0x43,0x1d);
		BF3905MIPI_write_cmos_sensor(0x44,0x1a);
		BF3905MIPI_write_cmos_sensor(0x45,0x14);
		BF3905MIPI_write_cmos_sensor(0x46,0x11);
		BF3905MIPI_write_cmos_sensor(0x47,0x0e);
		BF3905MIPI_write_cmos_sensor(0x48,0x0d);
		BF3905MIPI_write_cmos_sensor(0x49,0x0c);
		BF3905MIPI_write_cmos_sensor(0x4b,0x0b);
		BF3905MIPI_write_cmos_sensor(0x4c,0x09);
		BF3905MIPI_write_cmos_sensor(0x4e,0x08);
		BF3905MIPI_write_cmos_sensor(0x4f,0x07);
		BF3905MIPI_write_cmos_sensor(0x50,0x07);

		break;
		default:
		BF3905MIPI_write_cmos_sensor(0x40,0x3b);
		BF3905MIPI_write_cmos_sensor(0x41,0x36);
		BF3905MIPI_write_cmos_sensor(0x42,0x2b);
		BF3905MIPI_write_cmos_sensor(0x43,0x1d);
		BF3905MIPI_write_cmos_sensor(0x44,0x1a);
		BF3905MIPI_write_cmos_sensor(0x45,0x14);
		BF3905MIPI_write_cmos_sensor(0x46,0x11);
		BF3905MIPI_write_cmos_sensor(0x47,0x0e);
		BF3905MIPI_write_cmos_sensor(0x48,0x0d);
		BF3905MIPI_write_cmos_sensor(0x49,0x0c);
		BF3905MIPI_write_cmos_sensor(0x4b,0x0b);
		BF3905MIPI_write_cmos_sensor(0x4c,0x09);
		BF3905MIPI_write_cmos_sensor(0x4e,0x08);
		BF3905MIPI_write_cmos_sensor(0x4f,0x07);
		BF3905MIPI_write_cmos_sensor(0x50,0x07);

		break;
		}
}

/*************************************************************************
 * FUNCTION
 *	BF3905MIPI_config_window
 *
 * DESCRIPTION
 *	This function config the hardware window of BF3905MIPI for getting specified
 *  data of that window.
 *
 * PARAMETERS
 *	start_x : start column of the interested window
 *  start_y : start row of the interested window
 *  width  : column widht of the itnerested window
 *  height : row depth of the itnerested window
 *
 * RETURNS
 *	the data that read from BF3905MIPI
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void BF3905MIPI_config_window(kal_uint16 startx, kal_uint16 starty, kal_uint16 width, kal_uint16 height)
{
} /* BF3905MIPI_config_window */


/*************************************************************************
 * FUNCTION
 *	BF3905MIPI_SetGain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *   iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 BF3905MIPI_SetGain(kal_uint16 iGain)
{
	return iGain;
}


/*************************************************************************
 * FUNCTION
 *	BF3905MIPI_NightMode
 *
 * DESCRIPTION
 *	This function night mode of BF3905MIPI.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void BF3905MIPINightMode(kal_bool bEnable)
{
	if (bEnable)
	{	
		if(BF3905MIPI_MPEG4_encode_mode == KAL_TRUE)
			BF3905MIPI_write_cmos_sensor(0x89,0x7d);
		else
			BF3905MIPI_write_cmos_sensor(0x89,0x9d);	
			BF3905MIPI_NIGHT_MODE = KAL_TRUE;
	}
	else 
	{
		if(BF3905MIPI_MPEG4_encode_mode == KAL_TRUE)
			BF3905MIPI_write_cmos_sensor(0x89,0x7d);
		else
			BF3905MIPI_write_cmos_sensor(0x89,0x7d);				   
			BF3905MIPI_NIGHT_MODE = KAL_FALSE;
	}
} /* BF3905MIPI_NightMode */

/*************************************************************************
* FUNCTION
*	BF3905MIPI_Sensor_Init
*
* DESCRIPTION
*	This function apply all of the initial setting to sensor.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
*************************************************************************/
void BF3905MIPI_Sensor_Init(void)
{

 	
	SENSORDB("BF3905_MIPI_Initialize_Setting st\n");

	//BF3905_MIPI_write_cmos_sensor(0x12,0x80);   ///yzx 2.25
	BF3905MIPI_write_cmos_sensor(0x20,0x09);///09   //2014.1.25     ///49
	BF3905MIPI_write_cmos_sensor(0x09,0x00);
	BF3905MIPI_write_cmos_sensor(0x12,0x00);
	BF3905MIPI_write_cmos_sensor(0x3a,0x20);
	BF3905MIPI_write_cmos_sensor(0x17,0x00);
	BF3905MIPI_write_cmos_sensor(0x18,0xa0);
	BF3905MIPI_write_cmos_sensor(0x19,0x00);
	BF3905MIPI_write_cmos_sensor(0x1a,0x78);
	BF3905MIPI_write_cmos_sensor(0x03,0x00);
	//analog signals   
	BF3905MIPI_write_cmos_sensor(0x5d,0xb3);
	BF3905MIPI_write_cmos_sensor(0xbf,0x08);
	BF3905MIPI_write_cmos_sensor(0xc3,0x08);
	BF3905MIPI_write_cmos_sensor(0xca,0x10);
	BF3905MIPI_write_cmos_sensor(0x15,0x12);
	BF3905MIPI_write_cmos_sensor(0x62,0x00);
	BF3905MIPI_write_cmos_sensor(0x63,0x00);
	
	BF3905MIPI_write_cmos_sensor(0x1e,0x70);//0x70//60//40//50//mirror
	BF3905MIPI_write_cmos_sensor(0x2a,0x00); //dummy pixel
	BF3905MIPI_write_cmos_sensor(0x2b,0x06); //dummy pixel
	BF3905MIPI_write_cmos_sensor(0x92,0x00); //dummy line
	BF3905MIPI_write_cmos_sensor(0x93,0x00); //dummy line

	BF3905MIPI_write_cmos_sensor(0xd9,0x25);
	BF3905MIPI_write_cmos_sensor(0xdf,0x25);
	
	BF3905MIPI_write_cmos_sensor(0x4a,0x0c);
	BF3905MIPI_write_cmos_sensor(0xda,0x00);
	BF3905MIPI_write_cmos_sensor(0xdb,0xa2);
	BF3905MIPI_write_cmos_sensor(0xdc,0x00);
	BF3905MIPI_write_cmos_sensor(0xdd,0x7a);
	BF3905MIPI_write_cmos_sensor(0xde,0x00);
	BF3905MIPI_write_cmos_sensor(0x1b,0x2e);///  0e///ryx  1 bei pin  ///0e
	BF3905MIPI_write_cmos_sensor(0x60,0xe5);
	BF3905MIPI_write_cmos_sensor(0x61,0xf2);
	BF3905MIPI_write_cmos_sensor(0x6d,0xc0);
	BF3905MIPI_write_cmos_sensor(0xb9,0x00);
	BF3905MIPI_write_cmos_sensor(0x64,0x00);
	BF3905MIPI_write_cmos_sensor(0xbb,0x10);
	BF3905MIPI_write_cmos_sensor(0x08,0x02);
	BF3905MIPI_write_cmos_sensor(0x21,0x4f);
	BF3905MIPI_write_cmos_sensor(0x3e,0x83);
	BF3905MIPI_write_cmos_sensor(0x16,0xa1);
	BF3905MIPI_write_cmos_sensor(0x2f,0xc4);
	BF3905MIPI_write_cmos_sensor(0x13,0x00);
	BF3905MIPI_write_cmos_sensor(0x01,0x15);
	BF3905MIPI_write_cmos_sensor(0x02,0x23);
	BF3905MIPI_write_cmos_sensor(0x9d,0x20);
	BF3905MIPI_write_cmos_sensor(0x8c,0x03);
	BF3905MIPI_write_cmos_sensor(0x8d,0x11);
	BF3905MIPI_write_cmos_sensor(0x33,0x10);
	BF3905MIPI_write_cmos_sensor(0x34,0x1d);
	BF3905MIPI_write_cmos_sensor(0x36,0x45);
	BF3905MIPI_write_cmos_sensor(0x6e,0x20);
	BF3905MIPI_write_cmos_sensor(0xbc,0x0d);
		//lens shading
	BF3905MIPI_write_cmos_sensor(0x35,0x30);
	BF3905MIPI_write_cmos_sensor(0x65,0x2a);
	BF3905MIPI_write_cmos_sensor(0x66,0x2a);
	BF3905MIPI_write_cmos_sensor(0xbd,0xf4);
	BF3905MIPI_write_cmos_sensor(0xbe,0x44);
	BF3905MIPI_write_cmos_sensor(0x9b,0xf4);
	BF3905MIPI_write_cmos_sensor(0x9c,0x44);
	BF3905MIPI_write_cmos_sensor(0x37,0xf4);
	BF3905MIPI_write_cmos_sensor(0x38,0x44);
		 //denoise and edge enhancement   
	BF3905MIPI_write_cmos_sensor(0x70,0x0b);
	BF3905MIPI_write_cmos_sensor(0x71,0x0f);
	BF3905MIPI_write_cmos_sensor(0x72,0x4c);
	BF3905MIPI_write_cmos_sensor(0x73,0x38);//a1 //ryx  2014.08.22 0x1a
	BF3905MIPI_write_cmos_sensor(0x75,0x8a);
	BF3905MIPI_write_cmos_sensor(0x76,0x98);
	BF3905MIPI_write_cmos_sensor(0x77,0x5a);
	BF3905MIPI_write_cmos_sensor(0x78,0xff);
	BF3905MIPI_write_cmos_sensor(0x79,0x44);//0x24
	BF3905MIPI_write_cmos_sensor(0x7a,0x24);///12 //ryx  2014.08.22
	BF3905MIPI_write_cmos_sensor(0x7b,0x58);
	BF3905MIPI_write_cmos_sensor(0x7c,0x55);
	BF3905MIPI_write_cmos_sensor(0x7d,0x00);
	BF3905MIPI_write_cmos_sensor(0x7e,0x84);
	BF3905MIPI_write_cmos_sensor(0x7f,0x3c);
	 //AE  
	BF3905MIPI_write_cmos_sensor(0x13,0x07);
	BF3905MIPI_write_cmos_sensor(0x24,0x45);
	BF3905MIPI_write_cmos_sensor(0x25,0x88);
	BF3905MIPI_write_cmos_sensor(0x80,0x92);
	BF3905MIPI_write_cmos_sensor(0x81,0x00);
	BF3905MIPI_write_cmos_sensor(0x82,0x2a);
	BF3905MIPI_write_cmos_sensor(0x83,0x54);
	BF3905MIPI_write_cmos_sensor(0x84,0x39);
	BF3905MIPI_write_cmos_sensor(0x85,0x5d);
	BF3905MIPI_write_cmos_sensor(0x86,0x77);//88     //ryx
	BF3905MIPI_write_cmos_sensor(0x89,0x7d);//63  yzx 1.6//b3//ryx   //   yzx 4.12
	BF3905MIPI_write_cmos_sensor(0x8a,0x4c);//ryx  2014.08.22
	BF3905MIPI_write_cmos_sensor(0x8b,0x3f); //ryx  2014.08.22
	BF3905MIPI_write_cmos_sensor(0x8f,0x82);
	BF3905MIPI_write_cmos_sensor(0x94,0x62);//92 //42->92 avoid ae vabration
	BF3905MIPI_write_cmos_sensor(0x95,0x84);
	BF3905MIPI_write_cmos_sensor(0x96,0xb3);
	BF3905MIPI_write_cmos_sensor(0x97,0x40);
	BF3905MIPI_write_cmos_sensor(0x98,0x8a);
	BF3905MIPI_write_cmos_sensor(0x99,0x10);
	BF3905MIPI_write_cmos_sensor(0x9a,0x50);
	BF3905MIPI_write_cmos_sensor(0x9f,0x64);
	BF3905MIPI_write_cmos_sensor(0x39,0x98);   ///98   //ryx
	BF3905MIPI_write_cmos_sensor(0x3f,0x98);    ///98 //ryx
	BF3905MIPI_write_cmos_sensor(0x90,0x20);
	BF3905MIPI_write_cmos_sensor(0x91,0xd0);

	#if 1
       //gamma1   default  
    BF3905MIPI_write_cmos_sensor(0x40,0x3b);
	BF3905MIPI_write_cmos_sensor(0x41,0x36);
	BF3905MIPI_write_cmos_sensor(0x42,0x2b);
	BF3905MIPI_write_cmos_sensor(0x43,0x1d);
	BF3905MIPI_write_cmos_sensor(0x44,0x1a);
	BF3905MIPI_write_cmos_sensor(0x45,0x14);
	BF3905MIPI_write_cmos_sensor(0x46,0x11);
	BF3905MIPI_write_cmos_sensor(0x47,0x0e);
	BF3905MIPI_write_cmos_sensor(0x48,0x0d);
	BF3905MIPI_write_cmos_sensor(0x49,0x0c);
	BF3905MIPI_write_cmos_sensor(0x4b,0x0b);
	BF3905MIPI_write_cmos_sensor(0x4c,0x09);
	BF3905MIPI_write_cmos_sensor(0x4e,0x08);
	BF3905MIPI_write_cmos_sensor(0x4f,0x07);
	BF3905MIPI_write_cmos_sensor(0x50,0x07);
      #endif
 


	#if 0                                                                           
	 //gamma 1guobao du hao                                         
	BF3905MIPI_write_cmos_sensor(0x40, 0x36);                
	BF3905MIPI_write_cmos_sensor(0x41, 0x33);                 
	BF3905MIPI_write_cmos_sensor(0x42, 0x2a);                 
	BF3905MIPI_write_cmos_sensor(0x43, 0x22);                 
	BF3905MIPI_write_cmos_sensor(0x44, 0x1b);                 
	BF3905MIPI_write_cmos_sensor(0x45, 0x16);                 
	BF3905MIPI_write_cmos_sensor(0x46, 0x13);
	BF3905MIPI_write_cmos_sensor(0x47, 0x10);
	BF3905MIPI_write_cmos_sensor(0x48, 0x0e);
	BF3905MIPI_write_cmos_sensor(0x49, 0x0c);
	BF3905MIPI_write_cmos_sensor(0x4b, 0x0b);
	BF3905MIPI_write_cmos_sensor(0x4c, 0x0a);
	BF3905MIPI_write_cmos_sensor(0x4e, 0x09);
	BF3905MIPI_write_cmos_sensor(0x4f, 0x08);
	BF3905MIPI_write_cmos_sensor(0x50, 0x08);     
	 #endif 

	#if 0                                                                        
	//gamma ?clear brighting                                                        
	BF3905MIPI_write_cmos_sensor(0x40, 0x20);
	BF3905MIPI_write_cmos_sensor(0x41, 0x28);
	BF3905MIPI_write_cmos_sensor(0x42, 0x26);
	BF3905MIPI_write_cmos_sensor(0x43, 0x25);
	BF3905MIPI_write_cmos_sensor(0x44, 0x1f);
	BF3905MIPI_write_cmos_sensor(0x45, 0x1a);
	BF3905MIPI_write_cmos_sensor(0x46, 0x16);
	BF3905MIPI_write_cmos_sensor(0x47, 0x12);
	BF3905MIPI_write_cmos_sensor(0x48, 0x0f);
	BF3905MIPI_write_cmos_sensor(0x49, 0x0D);
	BF3905MIPI_write_cmos_sensor(0x4b, 0x0b);
	BF3905MIPI_write_cmos_sensor(0x4c, 0x0a);
	BF3905MIPI_write_cmos_sensor(0x4e, 0x08);
	BF3905MIPI_write_cmos_sensor(0x4f, 0x06);
	BF3905MIPI_write_cmos_sensor(0x50, 0x06);
	#endif 
	 
    #if 0
    //gamma  low denoise                	
    BF3905MIPI_write_cmos_sensor(0x40, 0x24);	
    BF3905MIPI_write_cmos_sensor(0x41, 0x30);	
    BF3905MIPI_write_cmos_sensor(0x42, 0x24);	
    BF3905MIPI_write_cmos_sensor(0x43, 0x1d);	
    BF3905MIPI_write_cmos_sensor(0x44, 0x1a);	
    BF3905MIPI_write_cmos_sensor(0x45, 0x14);	
    BF3905MIPI_write_cmos_sensor(0x46, 0x11);	
    BF3905MIPI_write_cmos_sensor(0x47, 0x0e);	
    BF3905MIPI_write_cmos_sensor(0x48, 0x0d);	
    BF3905MIPI_write_cmos_sensor(0x49, 0x0c);	
    BF3905MIPI_write_cmos_sensor(0x4b, 0x0b);	
    BF3905MIPI_write_cmos_sensor(0x4c, 0x09);	
    BF3905MIPI_write_cmos_sensor(0x4e, 0x09);	
    BF3905MIPI_write_cmos_sensor(0x4f, 0x08);	
    BF3905MIPI_write_cmos_sensor(0x50, 0x07);

	 #endif


     //color   out door
	BF3905MIPI_write_cmos_sensor(0x5a,0x56);
    BF3905MIPI_write_cmos_sensor(0x51,0x13);
	BF3905MIPI_write_cmos_sensor(0x52,0x05);
	BF3905MIPI_write_cmos_sensor(0x53,0x91);
	BF3905MIPI_write_cmos_sensor(0x54,0x72);
	BF3905MIPI_write_cmos_sensor(0x57,0x96);
	BF3905MIPI_write_cmos_sensor(0x58,0x35);
	/*
       //color  fu se hao 
	BF3905MIPI_write_cmos_sensor(0x5a,0xd6);
	BF3905MIPI_write_cmos_sensor(0x51,0x17);
	BF3905MIPI_write_cmos_sensor(0x52,0x13);
	BF3905MIPI_write_cmos_sensor(0x53,0x5e);
	BF3905MIPI_write_cmos_sensor(0x54,0x38);
	BF3905MIPI_write_cmos_sensor(0x57,0x38);
	BF3905MIPI_write_cmos_sensor(0x58,0x02);
	*/
	/*//color  indoor
	BF3905MIPI_write_cmos_sensor(0x5a,0xd6);
	BF3905MIPI_write_cmos_sensor(0x51,0x29);
	BF3905MIPI_write_cmos_sensor(0x52,0x0D);
	BF3905MIPI_write_cmos_sensor(0x53,0x91);
	BF3905MIPI_write_cmos_sensor(0x54,0x81);
	BF3905MIPI_write_cmos_sensor(0x57,0x56);
	BF3905MIPI_write_cmos_sensor(0x58,0x09);*/
   //color default 
	/* 
	BF3905MIPI_write_cmos_sensor(0x5a,0xd6);
	BF3905MIPI_write_cmos_sensor(0x51,0x28);
	BF3905MIPI_write_cmos_sensor(0x52,0x23);
	BF3905MIPI_write_cmos_sensor(0x53,0x9f);
	BF3905MIPI_write_cmos_sensor(0x54,0x73);
	BF3905MIPI_write_cmos_sensor(0x57,0x50);
	BF3905MIPI_write_cmos_sensor(0x58,0x08);
      //color se cai  yan li 
	BF3905MIPI_write_cmos_sensor(0x5a,0xd6);
	BF3905MIPI_write_cmos_sensor(0x51,0x2d);
	BF3905MIPI_write_cmos_sensor(0x52,0x52);
	BF3905MIPI_write_cmos_sensor(0x53,0xb7);
	BF3905MIPI_write_cmos_sensor(0x54,0xa9);
	BF3905MIPI_write_cmos_sensor(0x57,0xa4);
	BF3905MIPI_write_cmos_sensor(0x58,0x04);
	*/
       //color  fu se hao 
	BF3905MIPI_write_cmos_sensor(0x5a,0xd6);
	BF3905MIPI_write_cmos_sensor(0x51,0x17);
	BF3905MIPI_write_cmos_sensor(0x52,0x13);
	BF3905MIPI_write_cmos_sensor(0x53,0x5e);
	BF3905MIPI_write_cmos_sensor(0x54,0x38);
	BF3905MIPI_write_cmos_sensor(0x57,0x38);
	BF3905MIPI_write_cmos_sensor(0x58,0x02);
	
	
	BF3905MIPI_write_cmos_sensor(0x5b,0x02);
	BF3905MIPI_write_cmos_sensor(0x5c,0x30);
	BF3905MIPI_write_cmos_sensor(0xb0,0xe0);
	BF3905MIPI_write_cmos_sensor(0xb3,0x5a);//58//5c//5f
	BF3905MIPI_write_cmos_sensor(0xb4,0xe3);
	BF3905MIPI_write_cmos_sensor(0xb1,0xe0);  //ef   //ryx
	BF3905MIPI_write_cmos_sensor(0xb2,0xb0);  //ef  //ryx
	BF3905MIPI_write_cmos_sensor(0xb4,0x63);
	BF3905MIPI_write_cmos_sensor(0xb1,0xa8);//0xc8 //blue values. 0xa0
	BF3905MIPI_write_cmos_sensor(0xb2,0x90);//0xb0//read values. 0x80
	BF3905MIPI_write_cmos_sensor(0x55,0x00);//00  yzx 1.6
	
	BF3905MIPI_write_cmos_sensor(0x56,0x48);	//middle   //40  yzx  1.6
	BF3905MIPI_write_cmos_sensor(0x6a,0x81);
	BF3905MIPI_write_cmos_sensor(0x69,0x00); // effect normal 
	BF3905MIPI_write_cmos_sensor(0x67,0x80); // Normal, 
	BF3905MIPI_write_cmos_sensor(0x68,0x80); // Normal, 
	BF3905MIPI_write_cmos_sensor(0xb4,0x63); // Normal, 
	BF3905MIPI_write_cmos_sensor(0x23,0x55);
	BF3905MIPI_write_cmos_sensor(0xa0,0x00);
	BF3905MIPI_write_cmos_sensor(0xa1,0x51);//0x31
	BF3905MIPI_write_cmos_sensor(0xa2,0x0d);
	BF3905MIPI_write_cmos_sensor(0xa3,0x26);
	BF3905MIPI_write_cmos_sensor(0xa4,0x09);    //0a ///ryx 0x09
	BF3905MIPI_write_cmos_sensor(0xa5,0x22);     ///2c  //ryx 0x22
	BF3905MIPI_write_cmos_sensor(0xa6,0x03);   //0x04
	BF3905MIPI_write_cmos_sensor(0xa7,0x1a);
	BF3905MIPI_write_cmos_sensor(0xa8,0x18);
	BF3905MIPI_write_cmos_sensor(0xa9,0x13);
	BF3905MIPI_write_cmos_sensor(0xaa,0x18);
	BF3905MIPI_write_cmos_sensor(0xab,0x24);
	BF3905MIPI_write_cmos_sensor(0xac,0x3c);
	BF3905MIPI_write_cmos_sensor(0xad,0xf0);
	BF3905MIPI_write_cmos_sensor(0xae,0x59);
	
	BF3905MIPI_write_cmos_sensor(0xc5,0x55);  //0xaa
	BF3905MIPI_write_cmos_sensor(0xc6,0x88);  //0xbb
	BF3905MIPI_write_cmos_sensor(0xc7,0x30);
	BF3905MIPI_write_cmos_sensor(0xc8,0x0d);
	BF3905MIPI_write_cmos_sensor(0xc9,0x10);
	BF3905MIPI_write_cmos_sensor(0xd0,0xa3);
	BF3905MIPI_write_cmos_sensor(0xd1,0x00);
	BF3905MIPI_write_cmos_sensor(0xd2,0x58);
	BF3905MIPI_write_cmos_sensor(0xd3,0x09);
	BF3905MIPI_write_cmos_sensor(0xd4,0x24);
	BF3905MIPI_write_cmos_sensor(0xee,0x30);
   SENSORDB("BF3905_MIPI_Initialize_Setting end\n");
	
}


UINT32 BF3905MIPIGetSensorID(UINT32 *sensorID)
{
    int  retry = 3; 
    // check if sensor ID correct
    do {
        *sensorID=((BF3905MIPI_read_cmos_sensor(0xfc)<< 8)|(BF3905MIPI_read_cmos_sensor(0xfd)));
        if (*sensorID == BF3905MIPI_SENSOR_ID)
            break; 
        SENSORDB("Read Sensor ID Fail = 0x%04x\n", *sensorID); 
        retry--; 
    } while (retry > 0);

    if (*sensorID != BF3905MIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;    
}




/*************************************************************************
* FUNCTION
*	BF3905MIPI_Write_More_Registers
*
* DESCRIPTION
*	This function is served for FAE to modify the necessary Init Regs. Do not modify the regs
*     in init_BF3905MIPI() directly.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void BF3905MIPI_Write_More_Registers(void)
{

}


/*************************************************************************
 * FUNCTION
 *	BF3905MIPIOpen
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 BF3905MIPIOpen(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	printk("<Jet> Entry BF3905MIPIOpen!!!\r\n");

	Sleep(10);


	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = ((BF3905MIPI_read_cmos_sensor(0xfc) << 8) | (BF3905MIPI_read_cmos_sensor(0xfd)));
		if(sensor_id != BF3905MIPI_SENSOR_ID)  
		{
			SENSORDB("BF3905MIPI Read Sensor ID Fail[open] = 0x%x\n", sensor_id); 
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	
	SENSORDB("BF3905MIPI_ Sensor Read ID OK \r\n");
	BF3905MIPI_Sensor_Init();

#if 0//DEBUG_SENSOR_BF3905MIPI  
		struct file *fp; 
		mm_segment_t fs; 
		loff_t pos = 0; 
		static char buf[60*1024] ;

		printk("bf3905MIPI open debug \n");

		fp = filp_open("/mnt/sdcard/bf3905MIPI_sd.txt", O_RDONLY , 0); 

		if (IS_ERR(fp)) 
		{ 
			fromsd = 0;   
			printk("bf3905MIPI open file error\n");
		} 
		else 
		{
			fromsd = 1;
			printk("bf3905MIPI open file ok\n");
	
			filp_close(fp, NULL); 
			set_fs(fs);
		}

		if(fromsd == 1)
		{
			printk("bf3905MIPI open from t!\n");
			BF3905MIPI_Initialize_from_T_Flash();
		}
		else
		{
			printk("bf3905MIPI open not from t!\n");		
		}

#endif
	BF3905MIPI_Write_More_Registers();

	return ERROR_NONE;
} /* BF3905MIPIOpen */


/*************************************************************************
 * FUNCTION
 *	BF3905MIPIClose
 *
 * DESCRIPTION
 *	This function is to turn off sensor module power.
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 BF3905MIPIClose(void)
{
    return ERROR_NONE;
} /* BF3905MIPIClose */


/*************************************************************************
 * FUNCTION
 * BF3905MIPIPreview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 BF3905MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    kal_uint32 iTemp;
    kal_uint16 iStartX = 0, iStartY = 1;

    if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        RETAILMSG(1, (TEXT("Camera Video preview\r\n")));
        BF3905MIPI_MPEG4_encode_mode = KAL_TRUE;
       
    }
    else
    {
        RETAILMSG(1, (TEXT("Camera preview\r\n")));
        BF3905MIPI_MPEG4_encode_mode = KAL_FALSE;
    }

   // image_window->GrabStartX= IMAGE_SENSOR_VGA_GRAB_PIXELS;
    //image_window->GrabStartY= IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;

    // copy sensor_config_data
    memcpy(&BF3905MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* BF3905MIPIPreview */


/*************************************************************************
 * FUNCTION
 *	BF3905MIPICapture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 BF3905MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    BF3905MIPI_MODE_CAPTURE=KAL_TRUE;

   // image_window->GrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
   // image_window->GrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth= IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = IMAGE_SENSOR_FULL_HEIGHT;

    // copy sensor_config_data
    memcpy(&BF3905MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* BF3905MIPI_Capture() */



UINT32 BF3905MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorVideoWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorVideoHeight=IMAGE_SENSOR_PV_HEIGHT;
    return ERROR_NONE;
} /* BF3905MIPIGetResolution() */


UINT32 BF3905MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_HEIGHT;

    pSensorInfo->SensorCameraPreviewFrameRate=15;
    pSensorInfo->SensorVideoFrameRate=10;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=1;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;//MIPI setting
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->CaptureDelayFrame = 3;
    pSensorInfo->PreviewDelayFrame = 3;
    pSensorInfo->VideoDelayFrame = 4;


    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    default:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = 0;
        pSensorInfo->SensorGrabStartY = 0;
	//MIPI setting
	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 	
	pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
	pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
	pSensorInfo->SensorPacketECCOrder = 1;

        break;

    }
    BF3905MIPIPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &BF3905MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* BF3905MIPIGetInfo() */


UINT32 BF3905MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    default:
	 BF3905MIPIPreview(pImageWindow, pSensorConfigData);
        break;
    }


    return TRUE;
}	/* BF3905MIPIControl() */

BOOL BF3905MIPI_set_param_wb(UINT16 para)
{

	 kal_uint8 temp_r=0;
	//if(BF3905_MIPICurrentStatus.iWB == para)
		//return TRUE;
	SENSORDB("[Enter]BF3905_MIPI set_param_wb func:para = %d\n",para);
	temp_r=BF3905MIPI_read_cmos_sensor(0x13);
	
	switch (para)
	{
		case AWB_MODE_OFF:

		break;
		
		case AWB_MODE_AUTO:
			BF3905MIPI_write_cmos_sensor(0x13, temp_r|0x02); //bit[1]:AWB Auto:1 menual:0
			BF3905MIPI_write_cmos_sensor(0x01, 0x15);
			BF3905MIPI_write_cmos_sensor(0x02, 0x23);	
			break;

		case AWB_MODE_CLOUDY_DAYLIGHT:	
			//======================================================================
			//	MWB : Cloudy_D65										 
			//======================================================================	
			BF3905MIPI_write_cmos_sensor(0x13, temp_r&0xfd); //bit[1]:AWB Auto:1 menual:0
			BF3905MIPI_write_cmos_sensor(0x01, 0x10);
			BF3905MIPI_write_cmos_sensor(0x02, 0x28);
			break;

		case AWB_MODE_DAYLIGHT:
			//==============================================
			//	MWB : sun&daylight_D50						
			//==============================================
			BF3905MIPI_write_cmos_sensor(0x13, temp_r&0xfd); //bit[1]:AWB Auto:1 menual:0
			BF3905MIPI_write_cmos_sensor(0x01, 0x11);
			BF3905MIPI_write_cmos_sensor(0x02, 0x26);  
			break;

		case AWB_MODE_INCANDESCENT:
			//==============================================									   
			//	MWB : Incand_Tungsten						
			//==============================================
			BF3905MIPI_write_cmos_sensor(0x13, temp_r&0xfd); //bit[1]:AWB Auto:1 menual:0
			BF3905MIPI_write_cmos_sensor(0x01, 0x1f);
			BF3905MIPI_write_cmos_sensor(0x02, 0x15); 		
			break;

		case AWB_MODE_FLUORESCENT:
			//==================================================================
			//	MWB : Florescent_TL84							  
			//==================================================================
			BF3905MIPI_write_cmos_sensor(0x13, temp_r&0xfd); //bit[1]:AWB Auto:1 menual:0
			BF3905MIPI_write_cmos_sensor(0x01, 0x1a);
			BF3905MIPI_write_cmos_sensor(0x02, 0x1e); 	 
			break;
		//case AWB_MODE_TUNGSTEN:	
			//BF3905MIPI_write_cmos_sensor(0x13, temp_r&0xfd); //bit[1]:AWB Auto:1 menual:0
			//BF3905MIPI_write_cmos_sensor(0x01, 0x1a);
			//BF3905MIPI_write_cmos_sensor(0x02, 0x1e); 	  
			//break;
		default:
		return FALSE;
	}

	return TRUE;
} /* BF3905MIPI_set_param_wb */


BOOL BF3905MIPI_set_param_effect(UINT16 para)
{
	 kal_uint32 ret = KAL_TRUE;

   /*----------------------------------------------------------------*/
   /* Code Body 													 */
   /*----------------------------------------------------------------*/

   //if(BF3905_MIPICurrentStatus.iEffect == para)
	  //return TRUE;

   SENSORDB("[Enter]bf3905mipi set_param_effect func:para = %d\n",para);

   switch (para)
	{
		case MEFFECT_OFF:
			BF3905MIPI_write_cmos_sensor(0x69,0x00); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x67,0x80); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x68,0x80); // Normal, 
			
			BF3905MIPI_write_cmos_sensor(0x70,0x0b); // Normal, 
			BF3905MIPI_write_cmos_sensor(0xb4,0x63); // Normal, 
		break;
		
		case MEFFECT_SEPIA:
			
			BF3905MIPI_write_cmos_sensor(0x70,0x0b); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x7a,0x12); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x73,0x27); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x69,0x20); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x67,0x60); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x68,0xa0); // Normal, 
			BF3905MIPI_write_cmos_sensor(0xb4,0x03); // Normal, 


		break;
		
		case MEFFECT_NEGATIVE:
		    BF3905MIPI_write_cmos_sensor(0x70,0x0b); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x7a,0x12); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x73,0x27); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x69,0x01); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x67,0x80); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x68,0x80); // Normal, 
			BF3905MIPI_write_cmos_sensor(0xb4,0x03); // Normal, 
		break;
		
		case MEFFECT_SEPIAGREEN:
		//	BF3905MIPI_write_cmos_sensor(0x43 , 0x02);
		//	BF3905MIPI_write_cmos_sensor(0xda , 0xc0);
		//	BF3905MIPI_write_cmos_sensor(0xdb , 0xc0);
		break;
		
		case MEFFECT_SEPIABLUE:
		//	BF3905MIPI_write_cmos_sensor(0x43 , 0x02);
		//	BF3905MIPI_write_cmos_sensor(0xda , 0x50);
		//	BF3905MIPI_write_cmos_sensor(0xdb , 0xe0);
		break;

		case MEFFECT_MONO:
			BF3905MIPI_write_cmos_sensor(0x70,0x0b); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x7a,0x12); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x73,0x27); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x69,0x20); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x67,0x80); // Normal, 
			BF3905MIPI_write_cmos_sensor(0x68,0x80); // Normal, 
			BF3905MIPI_write_cmos_sensor(0xb4,0x03); // Normal, 

		break;
		default:
			ret = FALSE;
	}

	return ret;

} /* BF3905MIPI_set_param_effect */


BOOL BF3905MIPI_set_param_banding(UINT16 para)
{

//	kal_uint32 int_step_50=0,int_step_60=0;
   // SENSORDB("BF3905_MIPI_set_param_banding st\n");

    switch (para)
	{
		case AE_FLICKER_MODE_50HZ:		
		//	SENSORDB("BF3905MIPI_set_param_banding 50hz \n");
			BF3905MIPI_write_cmos_sensor(0x8a,0x4c);//int_step_50);
			break;
		case AE_FLICKER_MODE_60HZ:		
		//	SENSORDB("BF3905MIPI_set_param_banding 60hz \n");
			BF3905MIPI_write_cmos_sensor(0x8b,0x3f);//int_step_60);//7d //ryx//24000000/2/(784+16)/120
			break;
		default:
		//	SENSORDB("BF3905_MIPI_set_param_banding default,we choose 50hz instead\n");
		//	BF3905MIPI_write_cmos_sensor(0x8a,0x4c);//int_step_50);
		//	break;
			return FALSE;

	}

	return TRUE;
} /* BF3905MIPI_set_param_banding */


BOOL BF3905MIPI_set_param_exposure(UINT16 para)
{


	switch (para)
	{
		case AE_EV_COMP_n30:
			BF3905MIPI_write_cmos_sensor(0x24, 0x45);
		break;
		
		case AE_EV_COMP_n20:
			BF3905MIPI_write_cmos_sensor(0x24, 0x45);
		break;
		
		case AE_EV_COMP_n10:
			BF3905MIPI_write_cmos_sensor(0x24, 0x45);
		break;					
		
		case AE_EV_COMP_00:		
			BF3905MIPI_write_cmos_sensor(0x24, 0x45);
		break;
		
		case AE_EV_COMP_10:			
			BF3905MIPI_write_cmos_sensor(0x24, 0x45);
		break;
		
		case AE_EV_COMP_20:
			BF3905MIPI_write_cmos_sensor(0x24, 0x45);
		break;
		
		case AE_EV_COMP_30:
			BF3905MIPI_write_cmos_sensor(0x24, 0x45);
		break;
		default:
		return FALSE;
	}

	return TRUE;
} /* BF3905MIPI_set_param_exposure */



UINT32 BF3905MIPIYUVSetVideoMode(UINT16 u2FrameRate)    // lanking add
{
  
        BF3905MIPI_MPEG4_encode_mode = KAL_TRUE;
     if (u2FrameRate == 30)
   	{
   	
   	    /*********video frame ************/
		
   	}
    else if (u2FrameRate == 15)       
    	{
    	
   	    /*********video frame ************/
		
    	}
    else
   	{
   	
            SENSORDB("Wrong Frame Rate"); 
			
   	}

      return TRUE;

}


UINT32 BF3905MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{

#ifdef DEBUG_SENSOR_BF3905MIPI
		printk("______%s______ Tflash debug \n",__func__);
		return TRUE;
#endif

    switch (iCmd) {
    case FID_AWB_MODE:
        BF3905MIPI_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        BF3905MIPI_set_param_effect(iPara);
        break;
    case FID_AE_EV:
        BF3905MIPI_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:
        BF3905MIPI_set_param_banding(iPara);
		break;
    case FID_SCENE_MODE:
	 BF3905MIPINightMode(iPara);
        break;
    default:
        break;
    }
    return TRUE;
} /* BF3905MIPIYUVSensorSetting */

#if 0
static void BF3905MIPIFlashTriggerCheck(unsigned int *pFeatureReturnPara32) 
{ 

	unsigned int ag; 
	 

	ag = BF3905MIPI_read_cmos_sensor(0x8c);
	 
	if(ag >= 0x04)
		*pFeatureReturnPara32 = TRUE;
	else
		*pFeatureReturnPara32 = FALSE;
 
	return;

} 
#endif 

UINT32 BF3905MIPISetTestPatternMode(kal_bool bEnable)
{
	SENSORDB("test pattern bEnable:=%d\n",bEnable);
//	return ERROR_NONE;
		if(bEnable)
	{
		BF3905MIPI_write_cmos_sensor(0xb9,0x8f);
		BF3905MIPI_write_cmos_sensor(0xb6,0x80);
		BF3905MIPI_write_cmos_sensor(0xb7,0x80);
		BF3905MIPI_write_cmos_sensor(0xb8,0x80);
	}
	else
	{
		BF3905MIPI_write_cmos_sensor(0xb9,0x00);
	}
	return ERROR_NONE;
}

UINT32 BF3905MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
        UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    unsigned long long *feature_data=(unsigned long long *) pFeaturePara;
    unsigned long long *feature_return_para=(unsigned long long *) pFeaturePara;
	
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=VGA_PERIOD_PIXEL_NUMS;
			*pFeatureReturnPara16=VGA_PERIOD_LINE_NUMS;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = BF3905MIPI_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			//BF3905MIPI_night_mode((BOOL) *feature_data);
		break;
		case SENSOR_FEATURE_SET_GAIN:
		 break;		
		//case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO: 
		 //	BF3905MIPIFlashTriggerCheck(pFeatureData32); 
		// break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			BF3905MIPI_isp_master_clock=*pFeatureData32;
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			BF3905MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = BF3905MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &BF3905MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:

		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
                        *pFeatureReturnPara32++=0;
                        *pFeatureParaLen=4;	    
		    break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_YUV_CMD:
		       printk("BF3905MIPI YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			BF3905MIPIYUVSensorSetting((FEATURE_ID)*feature_data, *(feature_data+1));
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		       BF3905MIPIYUVSetVideoMode(*feature_data);
		       break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			BF3905MIPIGetSensorID(pFeatureData32);
		break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:			 
			BF3905MIPISetTestPatternMode((BOOL)*pFeatureData16);			
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32 = BF3905MIPI_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
		break;
    default:
        break;
	}
return ERROR_NONE;
}	/* BF3905MIPIFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncBF3905MIPIYUV=
{
	BF3905MIPIOpen,
	BF3905MIPIGetInfo,
	BF3905MIPIGetResolution,
	BF3905MIPIFeatureControl,
	BF3905MIPIControl,
	BF3905MIPIClose
};


UINT32 BF3905_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncBF3905MIPIYUV;
	return ERROR_NONE;
} /* SensorInit() */
