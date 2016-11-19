/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
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
 *   sensor.c
 *
 * Project:
 * --------
 *  
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Leo Lee
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * [GC2035MIPIYUV V1.0.0]
 * 8.17.2012 Leo.Lee
 * .First Release
 *
 *  V2.1.0 modified for DPC by travis 20140505
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GalaxyCoreinc. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
//#include <windows.h>
//#include <memory.h>
//#include <nkintr.h>
//#include <ceddk.h>
//#include <ceddk_exp.h>

//#include "kal_release.h"
//#include "i2c_exp.h"
//#include "gpio_exp.h"
//#include "msdk_exp.h"
//#include "msdk_sensor_exp.h"
//#include "msdk_isp_exp.h"
//#include "base_regs.h"
//#include "Sensor.h"
//#include "camera_sensor_para.h"
//#include "CameraCustomized.h"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "gc2035mipi_yuv_Sensor.h"
#include "gc2035mipi_yuv_Camera_Sensor_para.h"
#include "gc2035mipi_yuv_CameraCustomized.h"

#define GC2035MIPIYUV_DEBUG
#ifdef GC2035MIPIYUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

//#define GC2035MIPI_SUBSAMPLE
#define GC2035MIPI_TEST_PATTERN_CHECKSUM 0x54ddf19b 

//#define DEBUG_SENSOR_GC2035MIPI

#define  GC2035MIPI_SET_PAGE0    GC2035MIPI_write_cmos_sensor(0xfe,0x00)
#define  GC2035MIPI_SET_PAGE1    GC2035MIPI_write_cmos_sensor(0xfe,0x01)
#define  GC2035MIPI_SET_PAGE2    GC2035MIPI_write_cmos_sensor(0xfe,0x02)
#define  GC2035MIPI_SET_PAGE3    GC2035MIPI_write_cmos_sensor(0xfe,0x03)



extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
/*************************************************************************
* FUNCTION
*    GC2035MIPI_write_cmos_sensor
*
* DESCRIPTION
*    This function wirte data to CMOS sensor through I2C
*
* PARAMETERS
*    addr: the 16bit address of register
*    para: the 8bit value of register
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void GC2035MIPI_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), GC2035MIPI_WRITE_ID); 

#if (defined(__GC2035MIPI_DEBUG_TRACE__))
  if (sizeof(out_buff) != rt) printk("I2C write %x, %x error\n", addr, para);
#endif
}

/*************************************************************************
* FUNCTION
*    GC2035MIPI_read_cmos_sensor
*
* DESCRIPTION
*    This function read data from CMOS sensor through I2C.
*
* PARAMETERS
*    addr: the 16bit address of register
*
* RETURNS
*    8bit data read through I2C
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint8 GC2035MIPI_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;

    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), GC2035MIPI_WRITE_ID)) {
        SENSORDB("ERROR: GC2035MIPI_read_cmos_sensor \n");
    }

#if (defined(__GC2035MIPI_DEBUG_TRACE__))
  if (size != rt) printk("I2C read %x error\n", addr);
#endif

  return in_buff[0];
}


#ifdef DEBUG_SENSOR_GC2035MIPI
#define gc2035mipi_OP_CODE_INI		0x00		/* Initial value. */
#define gc2035mipi_OP_CODE_REG		0x01		/* Register */
#define gc2035mipi_OP_CODE_DLY		0x02		/* Delay */
#define gc2035mipi_OP_CODE_END		0x03		/* End of initial setting. */
static kal_uint16 fromsd;

typedef struct
{
	u16 init_reg;
	u16 init_val;	/* Save the register value and delay tick */
	u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
} gc2035mipi_initial_set_struct;

gc2035mipi_initial_set_struct gc2035mipi_Init_Reg[5000];

static u32 strtol(const char *nptr, u8 base)
{

	printk("gc2035mipi___%s____\n",__func__); 

	u8 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("gc2035mipi %s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{
		if((base==16 && *nptr>='A' && *nptr<='F') || 
				(base==16 && *nptr>='a' && *nptr<='f') || 
				(base>=10 && *nptr>='0' && *nptr<='9') ||
				(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

static u8 GC2035MIPI_Initialize_from_T_Flash()
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0, j = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */

	printk("gc2035mipi___%s____11111111111111\n",__func__); 



	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static u8 data_buff[10*1024] ;

	fp = filp_open("/mnt/sdcard/gc2035mipi_sd.txt", O_RDONLY , 0); 
	if (IS_ERR(fp)) 
	{ 
		printk("2035 create file error 1111111\n");  
		return -1; 
	} 
	else
	{
		printk("2035 create file error 2222222\n");  
	}
	fs = get_fs(); 
	set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	//printk("%s %d %d\n", buf,iFileLen,pos); 
	filp_close(fp, NULL); 
	set_fs(fs);


	printk("gc2035mipi___%s____22222222222222222\n",__func__); 



	/* Start parse the setting witch read from t-flash. */
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size))
	{
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
		{
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
			{
				curr_ptr++;		/* Skip block comment code. */
			}

			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}

		if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
		{
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}
		/* This just content one enter line. */
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		//printk(" curr_ptr1 = %s\n",curr_ptr);
		memcpy(func_ind, curr_ptr, 3);


		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */
			gc2035mipi_Init_Reg[i].op_code = gc2035mipi_OP_CODE_REG;

			gc2035mipi_Init_Reg[i].init_reg = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */

			gc2035mipi_Init_Reg[i].init_val = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */

		}
		else									/* DLY */
		{
			/* Need add delay for this setting. */ 
			curr_ptr += 4;	
			gc2035mipi_Init_Reg[i].op_code = gc2035mipi_OP_CODE_DLY;

			gc2035mipi_Init_Reg[i].init_reg = 0xFF;
			gc2035mipi_Init_Reg[i].init_val = strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
		}
		i++;


		/* Skip to next line directly. */
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}

	/* (0xFFFF, 0xFFFF) means the end of initial setting. */
	gc2035mipi_Init_Reg[i].op_code = gc2035mipi_OP_CODE_END;
	gc2035mipi_Init_Reg[i].init_reg = 0xFF;
	gc2035mipi_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
	printk("gc2035mipi %x  ==  %x\n",gc2035mipi_Init_Reg[j].init_reg, gc2035mipi_Init_Reg[j].init_val);
	
	printk("gc2035mipi___%s____3333333333333333\n",__func__); 

	/* Start apply the initial setting to sensor. */
#if 1
	for (j=0; j<i; j++)
	{
		if (gc2035mipi_Init_Reg[j].op_code == gc2035mipi_OP_CODE_END)	/* End of the setting. */
		{
			printk("gc2035mipi REG OK -----------------END!\n");
		
			break ;
		}
		else if (gc2035mipi_Init_Reg[j].op_code == gc2035mipi_OP_CODE_DLY)
		{
			msleep(gc2035mipi_Init_Reg[j].init_val);		/* Delay */
			printk("gc2035mipi REG OK -----------------DLY!\n");			
		}
		else if (gc2035mipi_Init_Reg[j].op_code == gc2035mipi_OP_CODE_REG)
		{

			GC2035MIPI_write_cmos_sensor(gc2035mipi_Init_Reg[j].init_reg, gc2035mipi_Init_Reg[j].init_val);
			printk("gc2035mipi REG OK!-----------------REG(0x%x,0x%x)\n",gc2035mipi_Init_Reg[j].init_reg, gc2035mipi_Init_Reg[j].init_val);			
			printk("gc2035mipi REG OK!-----------------REG(0x%x,0x%x)\n",gc2035mipi_Init_Reg[j].init_reg, gc2035mipi_Init_Reg[j].init_val);			
			printk("gc2035mipi REG OK!-----------------REG(0x%x,0x%x)\n",gc2035mipi_Init_Reg[j].init_reg, gc2035mipi_Init_Reg[j].init_val);			
			
		}
		else
		{
			printk("gc2035mipi REG ERROR!\n");
		}
	}
#endif
	return 1;	
}

#endif

/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/
/* Global Valuable */

static kal_uint32 zoom_factor = 0; 

static kal_bool GC2035MIPI_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool GC2035MIPI_sensor_cap_state = KAL_FALSE; //Preview or Capture

static kal_uint16 GC2035MIPI_exposure_lines=0, GC2035MIPI_extra_exposure_lines = 0;

static kal_uint16 GC2035MIPI_Capture_Shutter=0;
static kal_uint16 GC2035MIPI_Capture_Extra_Lines=0;

kal_uint32 GC2035MIPI_PV_dummy_pixels = 0,  GC2035MIPI_PV_dummy_lines = 0 ,GC2035MIPI_CAP_dummy_pixels = 0, GC2035MIPI_isp_master_clock=0 , DUMMY = 1000;

static kal_uint32  GC2035MIPI_sensor_pclk=390;

static kal_uint32 Preview_Shutter = 0;
static kal_uint32 Capture_Shutter = 0;

MSDK_SENSOR_CONFIG_STRUCT GC2035MIPISensorConfigData;

kal_uint16 GC2035MIPI_read_shutter(void)
{
	return  (GC2035MIPI_read_cmos_sensor(0x03) << 8)|GC2035MIPI_read_cmos_sensor(0x04) ;
} /* GC2035MIPI read_shutter */



static void GC2035MIPI_write_shutter(kal_uint32 shutter)
{

	if(shutter < 1)	
 	return;

	GC2035MIPI_write_cmos_sensor(0x03, (shutter >> 8) & 0xff);
	GC2035MIPI_write_cmos_sensor(0x04, shutter & 0xff);
}    /* GC2035MIPI_write_shutter */




static void GC2035MIPI_set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 GC2035MIPI_HV_Mirror;

	switch (image_mirror) 
	{
		case IMAGE_NORMAL:
			GC2035MIPI_HV_Mirror = 0x14; 
		    break;
		case IMAGE_H_MIRROR:
			GC2035MIPI_HV_Mirror = 0x15;
		    break;
		case IMAGE_V_MIRROR:
			GC2035MIPI_HV_Mirror = 0x16; 
		    break;
		case IMAGE_HV_MIRROR:
			GC2035MIPI_HV_Mirror = 0x17;
		    break;
		default:
		    break;
	}
	GC2035MIPI_write_cmos_sensor(0x17, GC2035MIPI_HV_Mirror);
}

static void GC2035MIPI_set_AE_mode(kal_bool AE_enable)
{
	kal_uint8 temp_AE_reg = 0;

	GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
	if (AE_enable == KAL_TRUE)
	{
		// turn on AEC/AGC
		GC2035MIPI_write_cmos_sensor(0xb6, 0x03);
	}
	else
	{
		// turn off AEC/AGC
		GC2035MIPI_write_cmos_sensor(0xb6, 0x00);
	}
}
 

static void GC2035MIPI_set_AWB_mode(kal_bool AWB_enable)
{
	kal_uint8 temp_AWB_reg = 0;

	GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
	if (AWB_enable == KAL_TRUE)
	{
		//enable Auto WB
		GC2035MIPI_write_cmos_sensor(0x82, 0xfe);
	}
	else
	{
		//turn off AWB
		GC2035MIPI_write_cmos_sensor(0x82, 0xfc);
	}
}


/*************************************************************************
* FUNCTION
*	GC2035MIPI_night_mode
*
* DESCRIPTION
*	This function night mode of GC2035MIPI.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void GC2035MIPI_night_mode(kal_bool enable)
{
	
		/* ==Video Preview, Auto Mode, use 39MHz PCLK, 30fps; Night Mode use 39M, 15fps */
		if (GC2035MIPI_sensor_cap_state == KAL_FALSE) 
		{
			if (enable) 
			{
				if (GC2035MIPI_VEDIO_encode_mode == KAL_TRUE) 
				{
					GC2035MIPI_write_cmos_sensor(0xfe, 0x01);
					GC2035MIPI_write_cmos_sensor(0x3e, 0x60);
					GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
				}
				else 
				{
					GC2035MIPI_write_cmos_sensor(0xfe, 0x01);
					GC2035MIPI_write_cmos_sensor(0x3e, 0x60);
					GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
				}
			}
			else 
			{
				/* when enter normal mode (disable night mode) without light, the AE vibrate */
				if (GC2035MIPI_VEDIO_encode_mode == KAL_TRUE) 
				{
					GC2035MIPI_write_cmos_sensor(0xfe, 0x01);
					GC2035MIPI_write_cmos_sensor(0x3e, 0x40);
					GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
				}
				else 
				{
					GC2035MIPI_write_cmos_sensor(0xfe, 0x01);
					GC2035MIPI_write_cmos_sensor(0x3e, 0x40);
					GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
				}
		}
	}
}	/* GC2035MIPI_night_mode */



/*************************************************************************
* FUNCTION
*	GC2035MIPI_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
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
static kal_uint32 GC2035MIPI_GetSensorID(kal_uint32 *sensorID)

{
    int  retry = 3; 
    // check if sensor ID correct
    do {
        *sensorID=((GC2035MIPI_read_cmos_sensor(0xf0)<< 8)|GC2035MIPI_read_cmos_sensor(0xf1));
        if (*sensorID == GC2035MIPI_SENSOR_ID)
            break; 
        SENSORDB("Read Sensor ID Fail = 0x%04x\n", *sensorID); 
        retry--; 
    } while (retry > 0);

    if (*sensorID != GC2035MIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;    
}   /* GC2035MIPIOpen  */

static void GC2035MIPI_Sensor_Init(void)
{
	zoom_factor = 0; 
	SENSORDB("GC2035MIPI_Sensor_Init");
	GC2035MIPI_write_cmos_sensor(0xfe , 0x80);  
	GC2035MIPI_write_cmos_sensor(0xfe , 0x80);  
	GC2035MIPI_write_cmos_sensor(0xfe , 0x80);  
	GC2035MIPI_write_cmos_sensor(0xfc , 0x06);  
	GC2035MIPI_write_cmos_sensor(0xf2 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xf3 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xf4 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xf5 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xf9 , 0xfe);  
	GC2035MIPI_write_cmos_sensor(0xfa , 0x00);  
	GC2035MIPI_write_cmos_sensor(0xf6 , 0x00);  
	                              
	GC2035MIPI_write_cmos_sensor(0xf7 , 0x05);  // don't change
	GC2035MIPI_write_cmos_sensor(0xf8 , 0x84);  // don't change
	                              
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x82 , 0x00);  
	GC2035MIPI_write_cmos_sensor(0xb3 , 0x60);  
	GC2035MIPI_write_cmos_sensor(0xb4 , 0x40);  
	GC2035MIPI_write_cmos_sensor(0xb5 , 0x60);  
	GC2035MIPI_write_cmos_sensor(0x03 , 0x02);  
	GC2035MIPI_write_cmos_sensor(0x04 , 0xdc);
	
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);  
	GC2035MIPI_write_cmos_sensor(0xec , 0x06);//04 
	GC2035MIPI_write_cmos_sensor(0xed , 0x06);//04 
	GC2035MIPI_write_cmos_sensor(0xee , 0x62);//60 
	GC2035MIPI_write_cmos_sensor(0xef , 0x92);//90 
	
	GC2035MIPI_write_cmos_sensor(0x0a , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x0c , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x0d , 0x04);  
	GC2035MIPI_write_cmos_sensor(0x0e , 0xc0);  
	GC2035MIPI_write_cmos_sensor(0x0f , 0x06);  
	GC2035MIPI_write_cmos_sensor(0x10 , 0x58);  
	GC2035MIPI_write_cmos_sensor(0x17 , 0x14);
	GC2035MIPI_write_cmos_sensor(0x18 , 0x0e); //0a 2012.10.26
	GC2035MIPI_write_cmos_sensor(0x19 , 0x0c);  
	GC2035MIPI_write_cmos_sensor(0x1a , 0x01);  
	GC2035MIPI_write_cmos_sensor(0x1b , 0x8b);
	GC2035MIPI_write_cmos_sensor(0x1c , 0x05); // add by lanking 20130403
	GC2035MIPI_write_cmos_sensor(0x1e , 0x88);  
	GC2035MIPI_write_cmos_sensor(0x1f , 0x08); //[3] tx-low en//
	GC2035MIPI_write_cmos_sensor(0x20 , 0x05);  
	GC2035MIPI_write_cmos_sensor(0x21 , 0x0f);  
	GC2035MIPI_write_cmos_sensor(0x22 , 0xf0);//// d0 20130628 
	GC2035MIPI_write_cmos_sensor(0x23 , 0xc3);  
	GC2035MIPI_write_cmos_sensor(0x24 , 0x17); //pad drive  16
	//AWB_gray
	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);
	GC2035MIPI_write_cmos_sensor(0x4f , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x32); // 30
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04); 
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x42); // 40
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04); 
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x52); // 50
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04); 
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x62); // 60
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04); 
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x72); // 70
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04); 
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x82); // 80
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04); 
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x92); // 90
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04); 
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4f , 0x01);
	GC2035MIPI_write_cmos_sensor(0x50 , 0x88);
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x82 , 0xfe);
	///////awb start ////////////////
	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);
	GC2035MIPI_write_cmos_sensor(0x50 , 0x88);//c0//[6]green mode
	GC2035MIPI_write_cmos_sensor(0x52 , 0x40);
	GC2035MIPI_write_cmos_sensor(0x54 , 0x60);
	GC2035MIPI_write_cmos_sensor(0x56 , 0x06);
	GC2035MIPI_write_cmos_sensor(0x57 , 0x20);//pre adjust
	GC2035MIPI_write_cmos_sensor(0x58 , 0x01); 
	GC2035MIPI_write_cmos_sensor(0x5b , 0x02);//AWB_gain_delta
	GC2035MIPI_write_cmos_sensor(0x61 , 0xaa);//R/G stand
	GC2035MIPI_write_cmos_sensor(0x62 , 0xaa);//R/G stand
	GC2035MIPI_write_cmos_sensor(0x71 , 0x00);
	GC2035MIPI_write_cmos_sensor(0x74 , 0x10);//AWB_C_max
	GC2035MIPI_write_cmos_sensor(0x77 , 0x08);//AWB_p2_x
	GC2035MIPI_write_cmos_sensor(0x78 , 0xfd);//AWB_p2_y
	GC2035MIPI_write_cmos_sensor(0x86 , 0x30);
	GC2035MIPI_write_cmos_sensor(0x87 , 0x00);
	GC2035MIPI_write_cmos_sensor(0x88 , 0x04);//[1]dark mode
	GC2035MIPI_write_cmos_sensor(0x8a , 0xc0);//awb move mode
	GC2035MIPI_write_cmos_sensor(0x89 , 0x75);
	GC2035MIPI_write_cmos_sensor(0x84 , 0x08);//auto_window
	GC2035MIPI_write_cmos_sensor(0x8b , 0x00);//awb compare luma
	GC2035MIPI_write_cmos_sensor(0x8d , 0x70);//awb gain limit R 
	GC2035MIPI_write_cmos_sensor(0x8e , 0x70);//G
	GC2035MIPI_write_cmos_sensor(0x8f , 0xf4);//B
  /////////awb end /////////////
	//AEC
	GC2035MIPI_write_cmos_sensor(0xfe, 0x01);  
	GC2035MIPI_write_cmos_sensor(0x11 , 0x20);//AEC_out_slope , 0x
	GC2035MIPI_write_cmos_sensor(0x1f , 0xc0);//max_post_gain
	GC2035MIPI_write_cmos_sensor(0x20 , 0x60);//max_pre_gain
	GC2035MIPI_write_cmos_sensor(0x47 , 0x30);//AEC_outdoor_th
	GC2035MIPI_write_cmos_sensor(0x0b , 0x10);//
	GC2035MIPI_write_cmos_sensor(0x13 , 0x75);//y_target
	
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x05 , 0x01);//  HB
	GC2035MIPI_write_cmos_sensor(0x06 , 0x25);  
	GC2035MIPI_write_cmos_sensor(0x07 , 0x00);//  VB
	GC2035MIPI_write_cmos_sensor(0x08 , 0x14);  
	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);  
	GC2035MIPI_write_cmos_sensor(0x27 , 0x00);//  step
	GC2035MIPI_write_cmos_sensor(0x28 , 0x83);  
	GC2035MIPI_write_cmos_sensor(0x29 , 0x04);//  level 0 13.75
	GC2035MIPI_write_cmos_sensor(0x2a , 0x9b);  
	GC2035MIPI_write_cmos_sensor(0x2b , 0x04);//  level 1 13.75
	GC2035MIPI_write_cmos_sensor(0x2c , 0x9b);  
	GC2035MIPI_write_cmos_sensor(0x2d , 0x05);//  level 2 10
	GC2035MIPI_write_cmos_sensor(0x2e , 0xa1);  
	GC2035MIPI_write_cmos_sensor(0x2f , 0x07);//  level 3 7.5
	GC2035MIPI_write_cmos_sensor(0x30 , 0x2a); 
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);   
	
	GC2035MIPI_write_cmos_sensor(0xb6 , 0x03);  
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x3f , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x40 , 0x77);  
	GC2035MIPI_write_cmos_sensor(0x42 , 0x7f);  
	GC2035MIPI_write_cmos_sensor(0x43 , 0x30);  
	GC2035MIPI_write_cmos_sensor(0x5c , 0x08);  
	GC2035MIPI_write_cmos_sensor(0x5e , 0x20);  
	GC2035MIPI_write_cmos_sensor(0x5f , 0x20);  
	GC2035MIPI_write_cmos_sensor(0x60 , 0x20);  
	GC2035MIPI_write_cmos_sensor(0x61 , 0x20);  
	GC2035MIPI_write_cmos_sensor(0x62 , 0x20);  
	GC2035MIPI_write_cmos_sensor(0x63 , 0x20);  
	GC2035MIPI_write_cmos_sensor(0x64 , 0x20);  
	GC2035MIPI_write_cmos_sensor(0x65 , 0x20);  

	///block////////////
	GC2035MIPI_write_cmos_sensor(0x80 , 0xff);  
	GC2035MIPI_write_cmos_sensor(0x81 , 0x26);  
	GC2035MIPI_write_cmos_sensor(0x87 , 0x90); //[7]middle gamma 
	GC2035MIPI_write_cmos_sensor(0x84 , 0x03);//output put foramat
	GC2035MIPI_write_cmos_sensor(0x86 , 0x06); //02 //sync plority 
	GC2035MIPI_write_cmos_sensor(0x8b , 0xbc);
	GC2035MIPI_write_cmos_sensor(0xb0 , 0x80); //globle gain
	GC2035MIPI_write_cmos_sensor(0xc0 , 0x40);//Yuv bypass
	//////lsc/////////////
	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);
	GC2035MIPI_write_cmos_sensor(0xc2 , 0x38);
	GC2035MIPI_write_cmos_sensor(0xc3 , 0x25);
	GC2035MIPI_write_cmos_sensor(0xc4 , 0x21);
	GC2035MIPI_write_cmos_sensor(0xc8 , 0x19);
	GC2035MIPI_write_cmos_sensor(0xc9 , 0x12);
	GC2035MIPI_write_cmos_sensor(0xca , 0x0e);
	GC2035MIPI_write_cmos_sensor(0xbc , 0x43);
	GC2035MIPI_write_cmos_sensor(0xbd , 0x18);
	GC2035MIPI_write_cmos_sensor(0xbe , 0x1b);
	GC2035MIPI_write_cmos_sensor(0xb6 , 0x40);
	GC2035MIPI_write_cmos_sensor(0xb7 , 0x2e);
	GC2035MIPI_write_cmos_sensor(0xb8 , 0x26);
	GC2035MIPI_write_cmos_sensor(0xc5 , 0x05);
	GC2035MIPI_write_cmos_sensor(0xc6 , 0x03);
	GC2035MIPI_write_cmos_sensor(0xc7 , 0x04);
	GC2035MIPI_write_cmos_sensor(0xcb , 0x00);
	GC2035MIPI_write_cmos_sensor(0xcc , 0x00);
	GC2035MIPI_write_cmos_sensor(0xcd , 0x00);
	GC2035MIPI_write_cmos_sensor(0xbf , 0x14);
	GC2035MIPI_write_cmos_sensor(0xc0 , 0x22);
	GC2035MIPI_write_cmos_sensor(0xc1 , 0x1b);
	GC2035MIPI_write_cmos_sensor(0xb9 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xba , 0x05);
	GC2035MIPI_write_cmos_sensor(0xbb , 0x05);
	GC2035MIPI_write_cmos_sensor(0xaa , 0x35);
	GC2035MIPI_write_cmos_sensor(0xab , 0x33);
	GC2035MIPI_write_cmos_sensor(0xac , 0x33);
	GC2035MIPI_write_cmos_sensor(0xad , 0x25);
	GC2035MIPI_write_cmos_sensor(0xae , 0x22);
	GC2035MIPI_write_cmos_sensor(0xaf , 0x27);
	GC2035MIPI_write_cmos_sensor(0xb0 , 0x1d);
	GC2035MIPI_write_cmos_sensor(0xb1 , 0x20);
	GC2035MIPI_write_cmos_sensor(0xb2 , 0x22);
	GC2035MIPI_write_cmos_sensor(0xb3 , 0x14);
	GC2035MIPI_write_cmos_sensor(0xb4 , 0x15);
	GC2035MIPI_write_cmos_sensor(0xb5 , 0x16);
	GC2035MIPI_write_cmos_sensor(0xd0 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xd2 , 0x07);
	GC2035MIPI_write_cmos_sensor(0xd3 , 0x08);
	GC2035MIPI_write_cmos_sensor(0xd8 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xda , 0x13);
	GC2035MIPI_write_cmos_sensor(0xdb , 0x17);
	GC2035MIPI_write_cmos_sensor(0xdc , 0x00);
	GC2035MIPI_write_cmos_sensor(0xde , 0x0a);
	GC2035MIPI_write_cmos_sensor(0xdf , 0x08);
	GC2035MIPI_write_cmos_sensor(0xd4 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xd6 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xd7 , 0x0c);
	GC2035MIPI_write_cmos_sensor(0xa4 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xa5 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xa6 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xa7 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xa8 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xa9 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xa1 , 0x80);
	GC2035MIPI_write_cmos_sensor(0xa2 , 0x80);
 
	//////////cc//////////////	
	GC2035MIPI_write_cmos_sensor(0xfe , 0x02);  
	GC2035MIPI_write_cmos_sensor(0xc0 , 0x01);  
	GC2035MIPI_write_cmos_sensor(0xc1 , 0x40);  
	GC2035MIPI_write_cmos_sensor(0xc2 , 0xfc);  
	GC2035MIPI_write_cmos_sensor(0xc3 , 0x05);  
	GC2035MIPI_write_cmos_sensor(0xc4 , 0xec);  
	GC2035MIPI_write_cmos_sensor(0xc5 , 0x42);  
	GC2035MIPI_write_cmos_sensor(0xc6 , 0xf8);  
	GC2035MIPI_write_cmos_sensor(0xc7 , 0x40);  
	GC2035MIPI_write_cmos_sensor(0xc8 , 0xf8);  
	GC2035MIPI_write_cmos_sensor(0xc9 , 0x06);  
	GC2035MIPI_write_cmos_sensor(0xca , 0xfd);  
	GC2035MIPI_write_cmos_sensor(0xcb , 0x3e);  
	GC2035MIPI_write_cmos_sensor(0xcc , 0xf3);  
	GC2035MIPI_write_cmos_sensor(0xcd , 0x36);  
	GC2035MIPI_write_cmos_sensor(0xce , 0xf6);  
	GC2035MIPI_write_cmos_sensor(0xcf , 0x04);  
	GC2035MIPI_write_cmos_sensor(0xe3 , 0x0c);  
	GC2035MIPI_write_cmos_sensor(0xe4 , 0x44);  
	GC2035MIPI_write_cmos_sensor(0xe5 , 0xe5); 
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);  

	///==========asde	
	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);  
	GC2035MIPI_write_cmos_sensor(0x21 , 0xbf);  
	GC2035MIPI_write_cmos_sensor(0xfe , 0x02);  
	GC2035MIPI_write_cmos_sensor(0xa4 , 0x00);//
	GC2035MIPI_write_cmos_sensor(0xa5 , 0x40); //lsc_th
	GC2035MIPI_write_cmos_sensor(0xa2 , 0xa0); //lsc_dec_slope
	GC2035MIPI_write_cmos_sensor(0x86 , 0x27);//add for DPC travis 20140505   
	GC2035MIPI_write_cmos_sensor(0x8a , 0x33);//add for DPC travis 20140505 
	GC2035MIPI_write_cmos_sensor(0x8d , 0x85);//add for DPC travis 20140505 
	GC2035MIPI_write_cmos_sensor(0xa6 , 0xf0);//80//change for DPC travis 20140505
	GC2035MIPI_write_cmos_sensor(0xa7 , 0x80); //ot_th
	GC2035MIPI_write_cmos_sensor(0xab , 0x31); //
	GC2035MIPI_write_cmos_sensor(0xa9 , 0x6f); //
	GC2035MIPI_write_cmos_sensor(0xb0 , 0x99); //0x//edge effect slope low
	GC2035MIPI_write_cmos_sensor(0xb1 , 0x34);//edge effect slope low
	GC2035MIPI_write_cmos_sensor(0xb3 , 0x80); //saturation dec slope
	GC2035MIPI_write_cmos_sensor(0xde , 0xb6);  //
	GC2035MIPI_write_cmos_sensor(0x38 , 0x0f); // 
	GC2035MIPI_write_cmos_sensor(0x39 , 0x60); //
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x81 , 0x26);  
	GC2035MIPI_write_cmos_sensor(0xfe , 0x02);  
	GC2035MIPI_write_cmos_sensor(0x83 , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x84 , 0x45);  
	GC2035MIPI_write_cmos_sensor(0xd1 , 0x38);  
	GC2035MIPI_write_cmos_sensor(0xd2 , 0x38);  
	GC2035MIPI_write_cmos_sensor(0xd3 , 0x40);//contrast ?	
	GC2035MIPI_write_cmos_sensor(0xd4 , 0x80);//contrast center 
	GC2035MIPI_write_cmos_sensor(0xd5 , 0x00);//luma_offset 
	GC2035MIPI_write_cmos_sensor(0xdc , 0x30);
	GC2035MIPI_write_cmos_sensor(0xdd , 0xb8);//edge_sa_g,b
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);
	///////dndd///////////
	GC2035MIPI_write_cmos_sensor(0xfe , 0x02);
	GC2035MIPI_write_cmos_sensor(0x88 , 0x15);//dn_b_base
	GC2035MIPI_write_cmos_sensor(0x8c , 0xf6); //[2]b_in_dark_inc
	GC2035MIPI_write_cmos_sensor(0x89 , 0x03); //dn_c_weight
	////////EE ///////////
	GC2035MIPI_write_cmos_sensor(0xfe , 0x02);
	GC2035MIPI_write_cmos_sensor(0x90 , 0x6c);// EEINTP mode1
	GC2035MIPI_write_cmos_sensor(0x97 , 0x45);// edge effect
	////==============RGB Gamma 
	GC2035MIPI_write_cmos_sensor(0xfe , 0x02);
	GC2035MIPI_write_cmos_sensor(0x15 , 0x0a);
	GC2035MIPI_write_cmos_sensor(0x16 , 0x12);
	GC2035MIPI_write_cmos_sensor(0x17 , 0x19);
	GC2035MIPI_write_cmos_sensor(0x18 , 0x1f);
	GC2035MIPI_write_cmos_sensor(0x19 , 0x2c);
	GC2035MIPI_write_cmos_sensor(0x1a , 0x38);
	GC2035MIPI_write_cmos_sensor(0x1b , 0x42);
	GC2035MIPI_write_cmos_sensor(0x1c , 0x4e);
	GC2035MIPI_write_cmos_sensor(0x1d , 0x63);
	GC2035MIPI_write_cmos_sensor(0x1e , 0x76);
	GC2035MIPI_write_cmos_sensor(0x1f , 0x87);
	GC2035MIPI_write_cmos_sensor(0x20 , 0x96);
	GC2035MIPI_write_cmos_sensor(0x21 , 0xa2);
	GC2035MIPI_write_cmos_sensor(0x22 , 0xb8);
	GC2035MIPI_write_cmos_sensor(0x23 , 0xca);
	GC2035MIPI_write_cmos_sensor(0x24 , 0xd8);
	GC2035MIPI_write_cmos_sensor(0x25 , 0xe3);
	GC2035MIPI_write_cmos_sensor(0x26 , 0xf0);
	GC2035MIPI_write_cmos_sensor(0x27 , 0xf8);
	GC2035MIPI_write_cmos_sensor(0x28 , 0xfd);
	GC2035MIPI_write_cmos_sensor(0x29 , 0xff);
	       //// small  RGB gamma////
	/*
	GC2035MIPI_write_cmos_sensor(0xfe , 0x02);
	GC2035MIPI_write_cmos_sensor(0x15 , 0x0b);
	GC2035MIPI_write_cmos_sensor(0x16 , 0x0e);
	GC2035MIPI_write_cmos_sensor(0x17 , 0x10);
	GC2035MIPI_write_cmos_sensor(0x18 , 0x12);
	GC2035MIPI_write_cmos_sensor(0x19 , 0x19);
	GC2035MIPI_write_cmos_sensor(0x1a , 0x21);
	GC2035MIPI_write_cmos_sensor(0x1b , 0x29);
	GC2035MIPI_write_cmos_sensor(0x1c , 0x31);
	GC2035MIPI_write_cmos_sensor(0x1d , 0x41);
	GC2035MIPI_write_cmos_sensor(0x1e , 0x50);
	GC2035MIPI_write_cmos_sensor(0x1f , 0x5f);
	GC2035MIPI_write_cmos_sensor(0x20 , 0x6d);
	GC2035MIPI_write_cmos_sensor(0x21 , 0x79);
	GC2035MIPI_write_cmos_sensor(0x22 , 0x91);
	GC2035MIPI_write_cmos_sensor(0x23 , 0xa5);
	GC2035MIPI_write_cmos_sensor(0x24 , 0xb9);
	GC2035MIPI_write_cmos_sensor(0x25 , 0xc9);
	GC2035MIPI_write_cmos_sensor(0x26 , 0xe1);
	GC2035MIPI_write_cmos_sensor(0x27 , 0xee);
	GC2035MIPI_write_cmos_sensor(0x28 , 0xf7);
	GC2035MIPI_write_cmos_sensor(0x29 , 0xff);
	*/
	///=================y gamma
	GC2035MIPI_write_cmos_sensor(0xfe , 0x02);  
	GC2035MIPI_write_cmos_sensor(0x2b , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x2c , 0x04);  
	GC2035MIPI_write_cmos_sensor(0x2d , 0x09);  
	GC2035MIPI_write_cmos_sensor(0x2e , 0x18);  
	GC2035MIPI_write_cmos_sensor(0x2f , 0x27);  
	GC2035MIPI_write_cmos_sensor(0x30 , 0x37);  
	GC2035MIPI_write_cmos_sensor(0x31 , 0x49);  
	GC2035MIPI_write_cmos_sensor(0x32 , 0x5c);  
	GC2035MIPI_write_cmos_sensor(0x33 , 0x7e);  
	GC2035MIPI_write_cmos_sensor(0x34 , 0xa0);  
	GC2035MIPI_write_cmos_sensor(0x35 , 0xc0);  
	GC2035MIPI_write_cmos_sensor(0x36 , 0xe0);  
	GC2035MIPI_write_cmos_sensor(0x37 , 0xff);  
	Sleep(250);
	//AWB clear
	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);
	GC2035MIPI_write_cmos_sensor(0x4f , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x10); // 10
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x20); // 20
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x30);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00); // 30
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x40); // 40
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x50); // 50
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x60); // 60
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x70); // 70
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x80); // 80
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x90); // 90
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0xa0); // a0
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0xb0); // b0
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0xc0); // c0
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0xd0); // d0
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4f , 0x01);
	/////// awb value////////
	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);
	GC2035MIPI_write_cmos_sensor(0x4f , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x30);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x80);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x80);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x02);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x02);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x40);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x80);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x80);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x02);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x02);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x02);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x53);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x08);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x04);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x62);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x10);
	GC2035MIPI_write_cmos_sensor(0x4d , 0x72);
	GC2035MIPI_write_cmos_sensor(0x4e , 0x20);
	GC2035MIPI_write_cmos_sensor(0x4f , 0x01);
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x82 , 0xfe);

        /////////mipi setting////////


                                
	GC2035MIPI_write_cmos_sensor(0xfe , 0x03);
	GC2035MIPI_write_cmos_sensor(0x01 , 0x03);
	GC2035MIPI_write_cmos_sensor(0x02 , 0x11);//[6:4]lane0 driver [2:0]clk driver
	GC2035MIPI_write_cmos_sensor(0x03 , 0x11);//[4]clk delay [2:0]lane1 driver
	GC2035MIPI_write_cmos_sensor(0x06 , 0x80);
	GC2035MIPI_write_cmos_sensor(0x11 , 0x1E);
	GC2035MIPI_write_cmos_sensor(0x12 , 0x80);
	GC2035MIPI_write_cmos_sensor(0x13 , 0x0c);
	GC2035MIPI_write_cmos_sensor(0x15 , 0x10);//12 travis 151113
	GC2035MIPI_write_cmos_sensor(0x04 , 0x20);//fifo
	GC2035MIPI_write_cmos_sensor(0x05 , 0x00);
	GC2035MIPI_write_cmos_sensor(0x17 , 0x00);
	                              
	GC2035MIPI_write_cmos_sensor(0x40 , 0x40);//output buf
	GC2035MIPI_write_cmos_sensor(0x41 , 0x02);
	GC2035MIPI_write_cmos_sensor(0x42 , 0x40);
	GC2035MIPI_write_cmos_sensor(0x43 , 0x06);
                                
	GC2035MIPI_write_cmos_sensor(0x21 , 0x02);
	GC2035MIPI_write_cmos_sensor(0x29 , 0x02);
	GC2035MIPI_write_cmos_sensor(0x2a , 0x03);
	GC2035MIPI_write_cmos_sensor(0x2b , 0x08);
                                
	GC2035MIPI_write_cmos_sensor(0x10 , 0x94);
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);

}
  
static void GC2035MIPI_Sensor_SVGA(void)
{
	SENSORDB("GC2035MIPI_Sensor_SVGA");

#if defined(GC2035MIPI_SUBSAMPLE)
	/////////subsample 800x600 /////////
	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);
	GC2035MIPI_write_cmos_sensor(0x21 , 0xbf);  //add for DPC travis 20140505  
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);	
	GC2035MIPI_write_cmos_sensor(0xc8 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xfa , 0x00);
                                
	GC2035MIPI_write_cmos_sensor(0x99 , 0x22);
	GC2035MIPI_write_cmos_sensor(0x9a , 0x06);
	GC2035MIPI_write_cmos_sensor(0x9b , 0x00);	
	GC2035MIPI_write_cmos_sensor(0x9c , 0x00);
	GC2035MIPI_write_cmos_sensor(0x9d , 0x00);
	GC2035MIPI_write_cmos_sensor(0x9e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x9f , 0x00);	
	GC2035MIPI_write_cmos_sensor(0xa0 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xa1 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xa2 , 0x00);
                                
	GC2035MIPI_write_cmos_sensor(0x90 , 0x01);
	GC2035MIPI_write_cmos_sensor(0x95 , 0x02);
	GC2035MIPI_write_cmos_sensor(0x96 , 0x58);
	GC2035MIPI_write_cmos_sensor(0x97 , 0x03);
	GC2035MIPI_write_cmos_sensor(0x98 , 0x20);
                                
	GC2035MIPI_write_cmos_sensor(0xfe , 0x03);
	GC2035MIPI_write_cmos_sensor(0x12 , 0x40);
	GC2035MIPI_write_cmos_sensor(0x13 , 0x06);
	GC2035MIPI_write_cmos_sensor(0x04 , 0x90);
	GC2035MIPI_write_cmos_sensor(0x05 , 0x01);
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);
#else
	/////////sclaer 800x600 /////////
	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);
	GC2035MIPI_write_cmos_sensor(0x21 , 0xbf);  //add for DPC travis 20140505 	
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);		
	GC2035MIPI_write_cmos_sensor(0xc8 , 0x14);
	GC2035MIPI_write_cmos_sensor(0xfa , 0x00);
                                
	GC2035MIPI_write_cmos_sensor(0x90 , 0x01);
	GC2035MIPI_write_cmos_sensor(0x95 , 0x02);
	GC2035MIPI_write_cmos_sensor(0x96 , 0x58);
	GC2035MIPI_write_cmos_sensor(0x97 , 0x03);
	GC2035MIPI_write_cmos_sensor(0x98 , 0x20);
                                
	GC2035MIPI_write_cmos_sensor(0xfe , 0x03);
	GC2035MIPI_write_cmos_sensor(0x12 , 0x40);
	GC2035MIPI_write_cmos_sensor(0x13 , 0x06);
	GC2035MIPI_write_cmos_sensor(0x04 , 0x90);
	GC2035MIPI_write_cmos_sensor(0x05 , 0x01);
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);

#endif

}

static void GC2035MIPI_Sensor_2M(void)
{
	SENSORDB("GC2035MIPI_Sensor_2M");
	
#if defined(GC2035MIPI_SUBSAMPLE)
	/////////subsample 800x600 /////////
 	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);
	GC2035MIPI_write_cmos_sensor(0x21 , 0xdf);  //add for DPC travis 20140505 
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);		  	
	GC2035MIPI_write_cmos_sensor(0xc8 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xfa , 0x11);
                                
	GC2035MIPI_write_cmos_sensor(0x99 , 0x11);
	GC2035MIPI_write_cmos_sensor(0x9a , 0x06);
	GC2035MIPI_write_cmos_sensor(0x9b , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x9c , 0x00);
	GC2035MIPI_write_cmos_sensor(0x9d , 0x00);
	GC2035MIPI_write_cmos_sensor(0x9e , 0x00);
	GC2035MIPI_write_cmos_sensor(0x9f , 0x00);  
	GC2035MIPI_write_cmos_sensor(0xa0 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xa1 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xa2 , 0x00);
                                
	GC2035MIPI_write_cmos_sensor(0x90 , 0x01);
	GC2035MIPI_write_cmos_sensor(0x95 , 0x04);
	GC2035MIPI_write_cmos_sensor(0x96 , 0xb0);
	GC2035MIPI_write_cmos_sensor(0x97 , 0x06);
	GC2035MIPI_write_cmos_sensor(0x98 , 0x40);
                                
	GC2035MIPI_write_cmos_sensor(0xfe , 0x03);
	GC2035MIPI_write_cmos_sensor(0x12 , 0x80);
	GC2035MIPI_write_cmos_sensor(0x13 , 0x0c);
	GC2035MIPI_write_cmos_sensor(0x04 , 0x20);
	GC2035MIPI_write_cmos_sensor(0x05 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);

#else
	///////sclaer  1600X1200///////
 	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);
	GC2035MIPI_write_cmos_sensor(0x21 , 0xdf);  //add for DPC travis 20140505  
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);		 	
	GC2035MIPI_write_cmos_sensor(0xc8 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xfa , 0x11);
	                              
	GC2035MIPI_write_cmos_sensor(0x90 , 0x01);
	GC2035MIPI_write_cmos_sensor(0x95 , 0x04);
	GC2035MIPI_write_cmos_sensor(0x96 , 0xb0);
	GC2035MIPI_write_cmos_sensor(0x97 , 0x06);
	GC2035MIPI_write_cmos_sensor(0x98 , 0x40);
                                
	GC2035MIPI_write_cmos_sensor(0xfe , 0x03);
	GC2035MIPI_write_cmos_sensor(0x12 , 0x80);
	GC2035MIPI_write_cmos_sensor(0x13 , 0x0c);
	GC2035MIPI_write_cmos_sensor(0x04 , 0x20);
	GC2035MIPI_write_cmos_sensor(0x05 , 0x00);
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);

#endif
	

}
static void GC2035MIPI_Write_More(void)
{
  //////////////For FAE ////////////////
  #if 0   
        /////////  re zao///
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);
	GC2035MIPI_write_cmos_sensor(0xfe , 0x01);
	GC2035MIPI_write_cmos_sensor(0x21 , 0xff);
	GC2035MIPI_write_cmos_sensor(0xfe , 0x02);  
	GC2035MIPI_write_cmos_sensor(0x8a , 0x33);
	GC2035MIPI_write_cmos_sensor(0x8c , 0x76);
	GC2035MIPI_write_cmos_sensor(0x8d , 0x85);
	GC2035MIPI_write_cmos_sensor(0xa6 , 0xf0);	
	GC2035MIPI_write_cmos_sensor(0xae , 0x9f);
	GC2035MIPI_write_cmos_sensor(0xa2 , 0x90);
	GC2035MIPI_write_cmos_sensor(0xa5 , 0x40);  
	GC2035MIPI_write_cmos_sensor(0xa7 , 0x30);
	GC2035MIPI_write_cmos_sensor(0xb0 , 0x88);
	GC2035MIPI_write_cmos_sensor(0x38 , 0x0b);
	GC2035MIPI_write_cmos_sensor(0x39 , 0x30);
	GC2035MIPI_write_cmos_sensor(0xfe , 0x00);  
	GC2035MIPI_write_cmos_sensor(0x87 , 0xb0);
	
 	////dark sun/////
	GC2035MIPI_write_cmos_sensor(0xfe , 0x02);
	GC2035MIPI_write_cmos_sensor(0x40 , 0x06);
	GC2035MIPI_write_cmos_sensor(0x41 , 0x23);
	GC2035MIPI_write_cmos_sensor(0x42 , 0x3f);
	GC2035MIPI_write_cmos_sensor(0x43 , 0x06);
	GC2035MIPI_write_cmos_sensor(0x44 , 0x00);
	GC2035MIPI_write_cmos_sensor(0x45 , 0x00);
	GC2035MIPI_write_cmos_sensor(0x46 , 0x14);
	GC2035MIPI_write_cmos_sensor(0x47 , 0x09);
 
  #endif

}
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	GC2035MIPIOpen
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
UINT32 GC2035MIPIOpen(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	zoom_factor = 0; 
	Sleep(10);


	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = (GC2035MIPI_read_cmos_sensor(0xf0) << 8) | GC2035MIPI_read_cmos_sensor(0xf1);
		if(sensor_id != GC2035MIPI_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	
	SENSORDB("GC2035MIPI Sensor Read ID OK \r\n");
	GC2035MIPI_Sensor_Init();
	GC2035MIPI_Write_More();

#ifdef DEBUG_SENSOR_GC2035MIPI  
		struct file *fp; 
		mm_segment_t fs; 
		loff_t pos = 0; 
		static char buf[60*1024] ;

		printk("open 2035 debug \n");
		printk("open 2035 debug \n");
		printk("open 2035 debug \n");	


		fp = filp_open("/mnt/sdcard/gc2035mipi_sd.txt", O_RDONLY , 0); 

		if (IS_ERR(fp)) 
		{ 

			fromsd = 0;   
			printk("open 2035 file error\n");
			printk("open 2035 file error\n");
			printk("open 2035 file error\n");		


		} 
		else 
		{
			fromsd = 1;
			printk("open 2035 file ok\n");
			printk("open 2035 file ok\n");
			printk("open 2035 file ok\n");

			//gc2035mipi_Initialize_from_T_Flash();
			
			filp_close(fp, NULL); 
			set_fs(fs);
		}

		if(fromsd == 1)
		{
			printk("________________2035 from t!\n");
			printk("________________2035 from t!\n");
			printk("________________2035 from t!\n");		
			GC2035MIPI_Initialize_from_T_Flash();
			printk("______after_____2035 from t!\n");	
			//GC2035MIPI_Sensor_Init(); 	
		}
		else
		{
			//GC2035MIPI_MPEG4_encode_mode = KAL_FALSE;
			printk("________________2035 not from t!\n");	
			printk("________________2035 not from t!\n");
			printk("________________2035 not from t!\n");		
			RETAILMSG(1, (TEXT("Sensor Read ID OK \r\n"))); 
		}

#endif
	Preview_Shutter =GC2035MIPI_read_shutter();
	
	return ERROR_NONE;
}	/* GC2035MIPIOpen() */

/*************************************************************************
* FUNCTION
*	GC2035MIPIClose
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
UINT32 GC2035MIPIClose(void)
{
//	CISModulePowerOn(FALSE);
	return ERROR_NONE;
}	/* GC2035MIPIClose() */

/*************************************************************************
* FUNCTION
*	GC2035MIPIPreview
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
UINT32 GC2035MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint8 iTemp, temp_AE_reg, temp_AWB_reg;
	kal_uint16 iStartX = 0, iStartY = 0;

	SENSORDB("GC2035MIPIPrevie\n");

	GC2035MIPI_sensor_cap_state = KAL_FALSE;



	GC2035MIPI_Sensor_SVGA();

 	GC2035MIPI_write_shutter(Preview_Shutter);
	GC2035MIPI_set_AE_mode(KAL_TRUE); 

	memcpy(&GC2035MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* GC2035MIPIPreview() */




UINT32 GC2035MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
		volatile kal_uint32 shutter = 0;

    SENSORDB("GC2035MIPICapture\n");

  if(GC2035MIPI_sensor_cap_state == KAL_FALSE)
 	{
    // turn off AEC/AGC
		GC2035MIPI_set_AE_mode(KAL_FALSE);

		shutter = GC2035MIPI_read_shutter();
		Preview_Shutter = shutter;

		GC2035MIPI_Sensor_2M();

    	shutter = shutter / 2;
	 	Capture_Shutter = shutter;
        
		// set shutter
		GC2035MIPI_write_shutter(Capture_Shutter);
		Sleep(400);
      }

     GC2035MIPI_sensor_cap_state = KAL_TRUE;

		SENSORDB("GC2035MIPICapture 2M\n");
	image_window->GrabStartX=1;
	image_window->GrabStartY=1;
	image_window->ExposureWindowWidth=GC2035MIPI_IMAGE_SENSOR_FULL_WIDTH - image_window->GrabStartX - 2;
	image_window->ExposureWindowHeight=GC2035MIPI_IMAGE_SENSOR_FULL_HEIGHT -image_window->GrabStartY - 2;    	 

    memcpy(&GC2035MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* GC2035MIPICapture() */



UINT32 GC2035MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=GC2035MIPI_IMAGE_SENSOR_FULL_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorFullHeight=GC2035MIPI_IMAGE_SENSOR_FULL_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	pSensorResolution->SensorPreviewWidth=GC2035MIPI_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorPreviewHeight=GC2035MIPI_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	pSensorResolution->SensorVideoWidth=GC2035MIPI_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorVideoHeight=GC2035MIPI_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;

	return ERROR_NONE;
}	/* GC2035MIPIGetResolution() */

UINT32 GC2035MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=GC2035MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=GC2035MIPI_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX=GC2035MIPI_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=GC2035MIPI_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	/*??? */
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->CaptureDelayFrame = 4; 
	pSensorInfo->PreviewDelayFrame = 1; 
	pSensorInfo->VideoDelayFrame = 0; 
	pSensorInfo->YUVAwbDelayFrame = 2;  // add by lanking
	pSensorInfo->YUVEffectDelayFrame = 2;  // add by lanking
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_6MA;


	pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_MIPI;

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = 2; 
			pSensorInfo->SensorGrabStartY = 2;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;

	
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = 2; 
            pSensorInfo->SensorGrabStartY = 2;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;

			
		break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
            pSensorInfo->SensorGrabStartX = 2; 
            pSensorInfo->SensorGrabStartY = 2;             
			
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;			
		break;
	}
	memcpy(pSensorConfigData, &GC2035MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* GC2035MIPIGetInfo() */


UINT32 GC2035MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			GC2035MIPIPreview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			GC2035MIPICapture(pImageWindow, pSensorConfigData);
		break;
		default:
		    break; 
	}
	return TRUE;
}	/* GC2035MIPIControl() */

BOOL GC2035MIPI_set_param_wb(UINT16 para)
{
	switch (para)
	{
		case AWB_MODE_AUTO:
			GC2035MIPI_write_cmos_sensor(0xb3, 0x61);
			GC2035MIPI_write_cmos_sensor(0xb4, 0x40);
			GC2035MIPI_write_cmos_sensor(0xb5, 0x61);
			GC2035MIPI_set_AWB_mode(KAL_TRUE);
		break;
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			GC2035MIPI_set_AWB_mode(KAL_FALSE);
			GC2035MIPI_write_cmos_sensor(0xb3, 0x58);
			GC2035MIPI_write_cmos_sensor(0xb4, 0x40);
			GC2035MIPI_write_cmos_sensor(0xb5, 0x50);
		break;
		case AWB_MODE_DAYLIGHT: //sunny
			GC2035MIPI_set_AWB_mode(KAL_FALSE);
			GC2035MIPI_write_cmos_sensor(0xb3, 0x70);
			GC2035MIPI_write_cmos_sensor(0xb4, 0x40);
			GC2035MIPI_write_cmos_sensor(0xb5, 0x50);
		break;
		case AWB_MODE_INCANDESCENT: //office
			GC2035MIPI_set_AWB_mode(KAL_FALSE);
			GC2035MIPI_write_cmos_sensor(0xb3, 0x50);
			GC2035MIPI_write_cmos_sensor(0xb4, 0x40);
			GC2035MIPI_write_cmos_sensor(0xb5, 0xa8);
		break;
		case AWB_MODE_TUNGSTEN: //home
			GC2035MIPI_set_AWB_mode(KAL_FALSE);
			GC2035MIPI_write_cmos_sensor(0xb3, 0xa0);
			GC2035MIPI_write_cmos_sensor(0xb4, 0x45);
			GC2035MIPI_write_cmos_sensor(0xb5, 0x40);
		break;
		case AWB_MODE_FLUORESCENT:
			GC2035MIPI_set_AWB_mode(KAL_FALSE);
			GC2035MIPI_write_cmos_sensor(0xb3, 0x72);
			GC2035MIPI_write_cmos_sensor(0xb4, 0x40);
			GC2035MIPI_write_cmos_sensor(0xb5, 0x5b);
		break;	
		default:
		return FALSE;
	}
	return TRUE;
} /* GC2035MIPI_set_param_wb */

BOOL GC2035MIPI_set_param_effect(UINT16 para)
{
	kal_uint32 ret = KAL_TRUE;
	switch (para)
	{
		case MEFFECT_OFF:
			GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
			GC2035MIPI_write_cmos_sensor(0x83, 0xe0);
		break;

		case MEFFECT_SEPIA:
			GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
			GC2035MIPI_write_cmos_sensor(0x83, 0x82);
		break;  

		case MEFFECT_NEGATIVE:		
			GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
			GC2035MIPI_write_cmos_sensor(0x83, 0x01);
		break; 

		case MEFFECT_SEPIAGREEN:		
			GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
			GC2035MIPI_write_cmos_sensor(0x83, 0x52);
		break;

		case MEFFECT_SEPIABLUE:	
			GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
			GC2035MIPI_write_cmos_sensor(0x83, 0x62);
		break;

		case MEFFECT_MONO:				
			GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
			GC2035MIPI_write_cmos_sensor(0x83, 0x12);
		break;

		default:
		return FALSE;
	}

	return ret;
} /* GC2035MIPI_set_param_effect */

BOOL GC2035MIPI_set_param_banding(UINT16 para)
{
    switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
		GC2035MIPI_write_cmos_sensor(0xfe , 0x00);  
		GC2035MIPI_write_cmos_sensor(0x05 , 0x01);//  HB
		GC2035MIPI_write_cmos_sensor(0x06 , 0x25);  
		GC2035MIPI_write_cmos_sensor(0x07 , 0x00);//  VB
		GC2035MIPI_write_cmos_sensor(0x08 , 0x14);  
		GC2035MIPI_write_cmos_sensor(0xfe , 0x01);  
		GC2035MIPI_write_cmos_sensor(0x27 , 0x00);//  step
		GC2035MIPI_write_cmos_sensor(0x28 , 0x83);  
		GC2035MIPI_write_cmos_sensor(0x29 , 0x04);//  level 0 13.75
		GC2035MIPI_write_cmos_sensor(0x2a , 0x9b);  
		GC2035MIPI_write_cmos_sensor(0x2b , 0x04);//  level 1 13.75
		GC2035MIPI_write_cmos_sensor(0x2c , 0x9b);  
		GC2035MIPI_write_cmos_sensor(0x2d , 0x05);//  level 2 10
		GC2035MIPI_write_cmos_sensor(0x2e , 0xa1);  
		GC2035MIPI_write_cmos_sensor(0x2f , 0x07);//  level 3 7.5
		GC2035MIPI_write_cmos_sensor(0x30 , 0x2a); 
		GC2035MIPI_write_cmos_sensor(0xfe , 0x00);    
            break;

        case AE_FLICKER_MODE_60HZ:
		GC2035MIPI_write_cmos_sensor(0xfe, 0x00);  
		GC2035MIPI_write_cmos_sensor(0x05, 0x01);//
		GC2035MIPI_write_cmos_sensor(0x06, 0x08);  
		GC2035MIPI_write_cmos_sensor(0x07, 0x00);//
		GC2035MIPI_write_cmos_sensor(0x08, 0x14);  
		GC2035MIPI_write_cmos_sensor(0xfe, 0x01);  
		GC2035MIPI_write_cmos_sensor(0x27, 0x00);//
		GC2035MIPI_write_cmos_sensor(0x28, 0x70);  
		GC2035MIPI_write_cmos_sensor(0x29, 0x04);//
		GC2035MIPI_write_cmos_sensor(0x2a, 0xd0);  
		GC2035MIPI_write_cmos_sensor(0x2b, 0x04);//
		GC2035MIPI_write_cmos_sensor(0x2c, 0xd0);  
		GC2035MIPI_write_cmos_sensor(0x2d, 0x05);//
		GC2035MIPI_write_cmos_sensor(0x2e, 0xb0);  
		GC2035MIPI_write_cmos_sensor(0x2f, 0x07);//
		GC2035MIPI_write_cmos_sensor(0x30, 0x00);  
		GC2035MIPI_write_cmos_sensor(0xfe, 0x00); 
            break;

          default:
              return FALSE;
    }

    return TRUE;
} /* GC2035MIPI_set_param_banding */

BOOL GC2035MIPI_set_param_exposure(UINT16 para)
{
	switch (para)
	{
		case AE_EV_COMP_n30:
			GC2035MIPI_SET_PAGE1;
			GC2035MIPI_write_cmos_sensor(0x13,0x20);
			GC2035MIPI_SET_PAGE0;
		break;	
		case AE_EV_COMP_n20:
			GC2035MIPI_SET_PAGE1;
			GC2035MIPI_write_cmos_sensor(0x13,0x40);
			GC2035MIPI_SET_PAGE0;
		break;
		case AE_EV_COMP_n10:
			GC2035MIPI_SET_PAGE1;
			GC2035MIPI_write_cmos_sensor(0x13,0x60);
			GC2035MIPI_SET_PAGE0;
		break;

		case AE_EV_COMP_00:
			GC2035MIPI_SET_PAGE1;
			GC2035MIPI_write_cmos_sensor(0x13,0x80);
			GC2035MIPI_SET_PAGE0;
		break;

		case AE_EV_COMP_10:
			GC2035MIPI_SET_PAGE1;
			GC2035MIPI_write_cmos_sensor(0x13,0xa0);
			GC2035MIPI_SET_PAGE0;
		break;
		case AE_EV_COMP_20:
			GC2035MIPI_SET_PAGE1;
			GC2035MIPI_write_cmos_sensor(0x13,0xc0);
			GC2035MIPI_SET_PAGE0;
		break;
		case AE_EV_COMP_30:
			GC2035MIPI_SET_PAGE1;
			GC2035MIPI_write_cmos_sensor(0x13,0xe0);
			GC2035MIPI_SET_PAGE0;
		break;
		default:
		return FALSE;
	}
	return TRUE;
} /* GC2035MIPI_set_param_exposure */

UINT32 GC2035MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
#ifdef DEBUG_SENSOR_GC2035MIPI
		printk("______%s______GC2035MIPI YUV setting\n",__func__);
		return TRUE;
#endif

	switch (iCmd) {
	case FID_SCENE_MODE:	    
//	    printk("Set Scene Mode:%d\n", iPara); 
	    if (iPara == SCENE_MODE_OFF)
	    {
	        GC2035MIPI_night_mode(0); 
	    }
	    else if (iPara == SCENE_MODE_NIGHTSCENE)
	    {
               GC2035MIPI_night_mode(1); 
	    }	    
	    break; 	    
	case FID_AWB_MODE:
//	    printk("Set AWB Mode:%d\n", iPara); 	    
           GC2035MIPI_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
//	    printk("Set Color Effect:%d\n", iPara); 	    	    
           GC2035MIPI_set_param_effect(iPara);
	break;
	case FID_AE_EV:
//           printk("Set EV:%d\n", iPara); 	    	    
           GC2035MIPI_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
//           printk("Set Flicker:%d\n", iPara); 	    	    	    
           GC2035MIPI_set_param_banding(iPara);
	break;
        case FID_AE_SCENE_MODE: 
            if (iPara == AE_MODE_OFF) {
                GC2035MIPI_set_AE_mode(KAL_FALSE);
            }
            else {
                GC2035MIPI_set_AE_mode(KAL_TRUE);
	    }
            break; 
	case FID_ZOOM_FACTOR:
	    zoom_factor = iPara; 
        break; 
	default:
	break;
	}
	return TRUE;
}   /* GC2035MIPIYUVSensorSetting */

UINT32 GC2035MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 iTemp;
    /* to fix VSYNC, to fix frame rate */
    //printk("Set YUV Video Mode \n");  

    if (u2FrameRate == 30)
    {
    }
    else if (u2FrameRate == 15)       
    {
    }
    else 
    {
        printk("Wrong frame rate setting \n");
    }
    GC2035MIPI_VEDIO_encode_mode = KAL_TRUE; 
        
    return TRUE;
}

static void GC2035MIPIFlashTriggerCheck(unsigned int *pFeatureReturnPara32) 
{ 
	unsigned int dg_pre,dg_post; 
	
	GC2035MIPI_write_cmos_sensor(0xfe,0x00); 
	dg_pre = GC2035MIPI_read_cmos_sensor(0xb1);
	dg_post = GC2035MIPI_read_cmos_sensor(0xb2);

	if((dg_pre >= 0x48)&&(dg_post >= 0x60))
		*pFeatureReturnPara32 = TRUE;
	else
		*pFeatureReturnPara32 = FALSE;

	return;
} 


UINT32 GC2035MIPISetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[GC2035MIPISetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
    if(bEnable)
    {
	GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
	GC2035MIPI_write_cmos_sensor(0x18, 0x06);
	GC2035MIPI_write_cmos_sensor(0x40, 0x00); 
	GC2035MIPI_write_cmos_sensor(0xb6, 0x00);
	GC2035MIPI_write_cmos_sensor(0x03, 0x00);
	GC2035MIPI_write_cmos_sensor(0x04, 0x01);
	GC2035MIPI_write_cmos_sensor(0xb0, 0x40);
	GC2035MIPI_write_cmos_sensor(0xb1, 0x40);
	GC2035MIPI_write_cmos_sensor(0xb2, 0x40);
	GC2035MIPI_write_cmos_sensor(0x80, 0x08);
	GC2035MIPI_write_cmos_sensor(0x81, 0x00);
	GC2035MIPI_write_cmos_sensor(0x82, 0x00);
	GC2035MIPI_write_cmos_sensor(0x87, 0x00);
	GC2035MIPI_write_cmos_sensor(0xb3, 0x40);
	GC2035MIPI_write_cmos_sensor(0xb4, 0x40);
	GC2035MIPI_write_cmos_sensor(0xb5, 0x40);
	GC2035MIPI_write_cmos_sensor(0xfe, 0x02);
	GC2035MIPI_write_cmos_sensor(0xa2, 0x00);
	GC2035MIPI_write_cmos_sensor(0xaa, 0x00);
	GC2035MIPI_write_cmos_sensor(0xa9, 0x00);
	GC2035MIPI_write_cmos_sensor(0xae, 0x00);
	GC2035MIPI_write_cmos_sensor(0xb0, 0x00);
	GC2035MIPI_write_cmos_sensor(0xb1, 0x00);
	GC2035MIPI_write_cmos_sensor(0xb2, 0x00);
	GC2035MIPI_write_cmos_sensor(0xb3, 0x00);
	GC2035MIPI_write_cmos_sensor(0xb5, 0x00);
	GC2035MIPI_write_cmos_sensor(0x38, 0x00);
	GC2035MIPI_write_cmos_sensor(0x4a, 0x00);
	GC2035MIPI_write_cmos_sensor(0x3a, 0x00);
	GC2035MIPI_write_cmos_sensor(0xa4, 0x00);
	GC2035MIPI_write_cmos_sensor(0x4a, 0x00);
	GC2035MIPI_write_cmos_sensor(0xdf, 0x00);
	GC2035MIPI_write_cmos_sensor(0xd0, 0x40);
	GC2035MIPI_write_cmos_sensor(0xd1, 0x20);
	GC2035MIPI_write_cmos_sensor(0xd2, 0x20);
	GC2035MIPI_write_cmos_sensor(0xd3, 0x40); 
	GC2035MIPI_write_cmos_sensor(0xdd, 0x00);
	GC2035MIPI_write_cmos_sensor(0xed, 0x00);
	GC2035MIPI_write_cmos_sensor(0xee, 0x00);
	GC2035MIPI_write_cmos_sensor(0xfe, 0x01);
	GC2035MIPI_write_cmos_sensor(0x9e, 0xc0);
	GC2035MIPI_write_cmos_sensor(0x9f, 0x40);
	GC2035MIPI_write_cmos_sensor(0xfe, 0x00);
	GC2035MIPI_write_cmos_sensor(0x8c, 0x01);
    }
    else
    {
	GC2035MIPI_SET_PAGE0; 
	GC2035MIPI_write_cmos_sensor(0x8c,0x00);
    }
    return ERROR_NONE;
}

UINT32 GC2035MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=GC2035MIPI_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=GC2035MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=GC2035MIPI_IMAGE_SENSOR_PV_WIDTH;
			*pFeatureReturnPara16=GC2035MIPI_IMAGE_SENSOR_PV_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			//*pFeatureReturnPara32 = GC2035MIPI_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			GC2035MIPI_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
		break;
   		case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO: 		
			GC2035MIPIFlashTriggerCheck(pFeatureData32); 
		break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			GC2035MIPI_isp_master_clock=*pFeatureData32;
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			GC2035MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = GC2035MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &GC2035MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			 GC2035MIPI_GetSensorID(pFeatureData32);
			 break;
		case SENSOR_FEATURE_SET_YUV_CMD:
		       //printk("GC2035MIPI YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			GC2035MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		       GC2035MIPIYUVSetVideoMode(*pFeatureData16);
		       break; 
	        case SENSOR_FEATURE_SET_TEST_PATTERN:	        
			GC2035MIPISetTestPatternMode((BOOL)*pFeatureData16);    
	        	break;
	        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:	
	             *pFeatureReturnPara32=GC2035MIPI_TEST_PATTERN_CHECKSUM;
	             *pFeatureParaLen=4;
	        	break;
		default:
			break;			
	}
	return ERROR_NONE;
}	/* GC2035MIPIFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncGC2035MIPI=
{
	GC2035MIPIOpen,
	GC2035MIPIGetInfo,
	GC2035MIPIGetResolution,
	GC2035MIPIFeatureControl,
	GC2035MIPIControl,
	GC2035MIPIClose
};

UINT32 GC2035MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncGC2035MIPI;

	return ERROR_NONE;
}	/* SensorInit() */
