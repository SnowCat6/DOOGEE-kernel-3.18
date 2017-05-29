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
 *   sp0a19yuv_sub_Sensor.c
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:


 * ------------
 *   Image sensor driver function
 *   V1.2.3
 *
 * Author:
 * -------
 *   Leo
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 * 2012.02.29  kill bugs
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

#include "sp0a19yuv_Sensor.h"
#include "sp0a19yuv_Camera_Sensor_para.h"
#include "sp0a19yuv_CameraCustomized.h"

//#include <mt6575_gpio.h>

#define SP0A19YUV_DEBUG
#ifdef SP0A19YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif
//#define DEBUG_SENSOR_SP0A19		//for T-card
#ifdef DEBUG_SENSOR_SP0A19
static kal_uint8 SP0A19_fromsd = 0;
kal_uint16 SP0A19_write_cmos_sensor(kal_uint8 addr, kal_uint8 para);
#define SP0A19_OP_CODE_INI		0x00		/* Initial value. */
#define SP0A19_OP_CODE_REG		0x01		/* Register */
#define SP0A19_OP_CODE_DLY		0x02		/* Delay */
#define SP0A19_OP_CODE_END		0x03		/* End of initial setting. */
typedef struct
{
	u16 init_reg;
	u16 init_val;	/* Save the register value and delay tick */
	u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
} SP0A19_initial_set_struct;
SP0A19_initial_set_struct SP0A19_Init_Reg[1000];
static u32 SP0A19_strtol(const char *nptr, u8 base)
{
	u8 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
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
u8 SP0A19_Initialize_from_T_Flash()
{
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	u32 i = 0, j = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */
	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static u8 data_buff[10*1024] ;
	fp = filp_open("/mnt/sdcard/SP0A19_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) { 
		printk("create file error\n"); 
		return -1; 
	} 
	fs = get_fs(); 
	set_fs(KERNEL_DS); 
	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	filp_close(fp, NULL); 
	set_fs(fs);
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
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		memcpy(func_ind, curr_ptr, 3);
		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */
			SP0A19_Init_Reg[i].op_code = SP0A19_OP_CODE_REG;
			SP0A19_Init_Reg[i].init_reg = SP0A19_strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */
			SP0A19_Init_Reg[i].init_val = SP0A19_strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */
		}
		else									/* DLY */
		{
			curr_ptr += 4;	
			SP0A19_Init_Reg[i].op_code = SP0A19_OP_CODE_DLY;
			SP0A19_Init_Reg[i].init_reg = 0xFF;
			SP0A19_Init_Reg[i].init_val = SP0A19_strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
		}
		i++;
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}
	SP0A19_Init_Reg[i].op_code = SP0A19_OP_CODE_END;
	SP0A19_Init_Reg[i].init_reg = 0xFF;
	SP0A19_Init_Reg[i].init_val = 0xFF;
	i++;
#if 1
	for (j=0; j<i; j++)
	{
		if (SP0A19_Init_Reg[j].op_code == SP0A19_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (SP0A19_Init_Reg[j].op_code == SP0A19_OP_CODE_DLY)
		{
			msleep(SP0A19_Init_Reg[j].init_val);		/* Delay */
		}
		else if (SP0A19_Init_Reg[j].op_code == SP0A19_OP_CODE_REG)
		{
			SP0A19_write_cmos_sensor(SP0A19_Init_Reg[j].init_reg, SP0A19_Init_Reg[j].init_val);
		}
		else
		{
			printk("REG ERROR!\n");
		}
	}
#endif
	return 1;	
}
#endif

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_uint16 SP0A19_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 2, SP0A19_WRITE_ID);

}
kal_uint16 SP0A19_read_cmos_sensor(kal_uint8 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, SP0A19_WRITE_ID);
	
    return get_byte;
}


/*******************************************************************************
 * // Adapter for Winmo typedef
 ********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

kal_bool   SP0A19_MPEG4_encode_mode = KAL_FALSE;
kal_uint16 SP0A19_dummy_pixels = 0, SP0A19_dummy_lines = 0;
kal_bool   SP0A19_MODE_CAPTURE = KAL_FALSE;
kal_bool   SP0A19_NIGHT_MODE = KAL_FALSE;

kal_uint32 SP0A19_isp_master_clock;
static kal_uint32 SP0A19_g_fPV_PCLK = 24;

kal_uint8 SP0A19_sensor_write_I2C_address = SP0A19_WRITE_ID;
kal_uint8 SP0A19_sensor_read_I2C_address = SP0A19_READ_ID;

UINT8 SP0A19PixelClockDivider=0;

kal_uint8 sp0a19_isBanding = 0; // 0: 50hz  1:60hz

MSDK_SENSOR_CONFIG_STRUCT SP0A19SensorConfigData;

#define SP0A19_SET_PAGE0 	SP0A19_write_cmos_sensor(0xfd, 0x00)
#define SP0A19_SET_PAGE1 	SP0A19_write_cmos_sensor(0xfd, 0x01)


/*************************************************************************
 * FUNCTION
 *	SP0A19_SetShutter
 *
 * DESCRIPTION
 *	This function set e-shutter of SP0A19 to change exposure time.
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
void SP0A19_Set_Shutter(kal_uint16 iShutter)
{
} /* Set_SP0A19_Shutter */


/*************************************************************************
 * FUNCTION
 *	SP0A19_read_Shutter
 *
 * DESCRIPTION
 *	This function read e-shutter of SP0A19 .
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
kal_uint16 SP0A19_Read_Shutter(void)
{
    	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 shutter;

	temp_reg1 = SP0A19_read_cmos_sensor(0x04);
	temp_reg2 = SP0A19_read_cmos_sensor(0x03);

	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

	return shutter;
} /* SP0A19_read_shutter */


/*************************************************************************
 * FUNCTION
 *	SP0A19_write_reg
 *
 * DESCRIPTION
 *	This function set the register of SP0A19.
 *
 * PARAMETERS
 *	addr : the register index of SP0A19
 *  para : setting parameter of the specified register of SP0A19
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void SP0A19_write_reg(kal_uint32 addr, kal_uint32 para)
{
	SP0A19_write_cmos_sensor(addr, para);
} /* SP0A19_write_reg() */


/*************************************************************************
 * FUNCTION
 *	SP0A19_read_cmos_sensor
 *
 * DESCRIPTION
 *	This function read parameter of specified register from SP0A19.
 *
 * PARAMETERS
 *	addr : the register index of SP0A19
 *
 * RETURNS
 *	the data that read from SP0A19
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint32 SP0A19_read_reg(kal_uint32 addr)
{
	return SP0A19_read_cmos_sensor(addr);
} /* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	SP0A19_awb_enable
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
static void SP0A19_awb_enable(kal_bool enalbe)
{	 
	kal_uint16 temp_AWB_reg = 0;

	temp_AWB_reg = SP0A19_read_cmos_sensor(0x42);
	
	if (enalbe)
	{
		SP0A19_write_cmos_sensor(0x42, (temp_AWB_reg |0x02));
	}
	else
	{
		SP0A19_write_cmos_sensor(0x42, (temp_AWB_reg & (~0x02)));
	}

}


/*************************************************************************
 * FUNCTION
 *	SP0A19_config_window
 *
 * DESCRIPTION
 *	This function config the hardware window of SP0A19 for getting specified
 *  data of that window.
 *
 * PARAMETERS
 *	start_x : start column of the interested window
 *  start_y : start row of the interested window
 *  width  : column widht of the itnerested window
 *  height : row depth of the itnerested window
 *
 * RETURNS
 *	the data that read from SP0A19
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void SP0A19_config_window(kal_uint16 startx, kal_uint16 starty, kal_uint16 width, kal_uint16 height)
{
} /* SP0A19_config_window */


/*************************************************************************
 * FUNCTION
 *	SP0A19_SetGain
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
kal_uint16 SP0A19_SetGain(kal_uint16 iGain)
{
	return iGain;
}


/*************************************************************************
 * FUNCTION
 *	SP0A19_NightMode
 *
 * DESCRIPTION
 *	This function night mode of SP0A19.
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
void SP0A19GammaSelect(kal_uint32 GammaLvl);
void SP0A19NightMode(kal_bool bEnable)
	{
         // kal_uint8 temp = SP0A19_read_cmos_sensor(0x3B);

	if(bEnable)//night mode
	{ 
		SP0A19_write_cmos_sensor(0xfd,0x0 );
		SP0A19_write_cmos_sensor(0xb2,0x25);
		SP0A19_write_cmos_sensor(0xb3,0x1f);
					
	   if(SP0A19_MPEG4_encode_mode == KAL_TRUE)
		{
				if(sp0a19_isBanding== 0)
				{
				//Video record night 24M 50hz 12-12FPS maxgain:0x8c
					SP0A19_write_cmos_sensor(0xfd,0x00);
					SP0A19_write_cmos_sensor(0x03,0x00);
					SP0A19_write_cmos_sensor(0x04,0x7b);
					SP0A19_write_cmos_sensor(0x06,0x00);
					SP0A19_write_cmos_sensor(0x09,0x08);
					SP0A19_write_cmos_sensor(0x0a,0x1d);
					SP0A19_write_cmos_sensor(0xf0,0x29);
					SP0A19_write_cmos_sensor(0xf1,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0x90,0x0c);
					SP0A19_write_cmos_sensor(0x92,0x01);
					SP0A19_write_cmos_sensor(0x98,0x29);
					SP0A19_write_cmos_sensor(0x99,0x00);
					SP0A19_write_cmos_sensor(0x9a,0x01);
					SP0A19_write_cmos_sensor(0x9b,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0xce,0xec);
					SP0A19_write_cmos_sensor(0xcf,0x01);
					SP0A19_write_cmos_sensor(0xd0,0xec);
					SP0A19_write_cmos_sensor(0xd1,0x01);
					SP0A19_write_cmos_sensor(0xfd,0x00);
				       SENSORDB(" video 50Hz night\r\n");
				}
				else if(sp0a19_isBanding == 1)
				{
				//SI13_SP0A19 24M 1分频 50Hz 8.0392-8fps AE_Parameters_ maxgain 80
					//dbg_print("SP0A19_VID_NIGTHT_60HZ  \r\n");
					SP0A19_write_cmos_sensor(0xfd,0x00);
					SP0A19_write_cmos_sensor(0x03,0x00);
					SP0A19_write_cmos_sensor(0x04,0x69);
					SP0A19_write_cmos_sensor(0x06,0x00);
					SP0A19_write_cmos_sensor(0x09,0x07);
					SP0A19_write_cmos_sensor(0x0a,0xd7);
					SP0A19_write_cmos_sensor(0xf0,0x23);
					SP0A19_write_cmos_sensor(0xf1,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0x90,0x0f);
					SP0A19_write_cmos_sensor(0x92,0x01);
					SP0A19_write_cmos_sensor(0x98,0x23);
					SP0A19_write_cmos_sensor(0x99,0x00);
					SP0A19_write_cmos_sensor(0x9a,0x01);
					SP0A19_write_cmos_sensor(0x9b,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0xce,0x0d);
					SP0A19_write_cmos_sensor(0xcf,0x02);
					SP0A19_write_cmos_sensor(0xd0,0x0d);
					SP0A19_write_cmos_sensor(0xd1,0x02);
					SP0A19_write_cmos_sensor(0xfd,0x00);	
				       SENSORDB(" video 60Hz night\r\n");
				}
   		  	}	
	    else 
	   {  
				
			       if(sp0a19_isBanding== 0)
				{
				//capture preview night 24M 50hz 20-6FPS maxgain:0x78	 
					SP0A19_write_cmos_sensor(0xfd,0x00);
					SP0A19_write_cmos_sensor(0x03,0x01);
					SP0A19_write_cmos_sensor(0x04,0x32);
					SP0A19_write_cmos_sensor(0x06,0x00);
					SP0A19_write_cmos_sensor(0x09,0x01);
					SP0A19_write_cmos_sensor(0x0a,0x46);
					SP0A19_write_cmos_sensor(0xf0,0x66);
					SP0A19_write_cmos_sensor(0xf1,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0x90,0x10);
					SP0A19_write_cmos_sensor(0x92,0x01);
					SP0A19_write_cmos_sensor(0x98,0x66);
					SP0A19_write_cmos_sensor(0x99,0x00);
					SP0A19_write_cmos_sensor(0x9a,0x01);
					SP0A19_write_cmos_sensor(0x9b,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0xce,0x60);
					SP0A19_write_cmos_sensor(0xcf,0x06);
					SP0A19_write_cmos_sensor(0xd0,0x60);
					SP0A19_write_cmos_sensor(0xd1,0x06);
					SP0A19_write_cmos_sensor(0xfd,0x00);	
				       SENSORDB(" priview 50Hz night\r\n");	
				}  
				else if(sp0a19_isBanding== 1)
				{
				//capture preview night 24M 60hz 20-6FPS maxgain:0x78
					SP0A19_write_cmos_sensor(0xfd,0x00);
					SP0A19_write_cmos_sensor(0x03,0x00);
					SP0A19_write_cmos_sensor(0x04,0xff);
					SP0A19_write_cmos_sensor(0x06,0x00);
					SP0A19_write_cmos_sensor(0x09,0x01);
					SP0A19_write_cmos_sensor(0x0a,0x46);
					SP0A19_write_cmos_sensor(0xf0,0x55);
					SP0A19_write_cmos_sensor(0xf1,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0x90,0x14);
					SP0A19_write_cmos_sensor(0x92,0x01);
					SP0A19_write_cmos_sensor(0x98,0x55);
					SP0A19_write_cmos_sensor(0x99,0x00);
					SP0A19_write_cmos_sensor(0x9a,0x01);
					SP0A19_write_cmos_sensor(0x9b,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0xce,0xa4);
					SP0A19_write_cmos_sensor(0xcf,0x06);
					SP0A19_write_cmos_sensor(0xd0,0xa4);
					SP0A19_write_cmos_sensor(0xd1,0x06);
					SP0A19_write_cmos_sensor(0xfd,0x00);
				       SENSORDB(" priview 60Hz night\r\n");	
				}
			       } 		
	}
	else    // daylight mode
	{
		
					SP0A19_write_cmos_sensor(0xfd,0x0 );
				SP0A19_write_cmos_sensor(0xb2,0x08);
				SP0A19_write_cmos_sensor(0xb3,0x1f);
					
	         if(SP0A19_MPEG4_encode_mode == KAL_TRUE)
	          {
				
				if(sp0a19_isBanding== 0)
				{
				//Video record daylight 24M 50hz 14-14FPS maxgain:0x80
					SP0A19_write_cmos_sensor(0xfd,0x00);
					SP0A19_write_cmos_sensor(0x03,0x00);
					SP0A19_write_cmos_sensor(0x04,0xd8);
					SP0A19_write_cmos_sensor(0x06,0x00);
					SP0A19_write_cmos_sensor(0x09,0x03);
					SP0A19_write_cmos_sensor(0x0a,0x31);
					SP0A19_write_cmos_sensor(0xf0,0x48);
					SP0A19_write_cmos_sensor(0xf1,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0x90,0x07);
					SP0A19_write_cmos_sensor(0x92,0x01);
					SP0A19_write_cmos_sensor(0x98,0x48);
					SP0A19_write_cmos_sensor(0x99,0x00);
					SP0A19_write_cmos_sensor(0x9a,0x01);
					SP0A19_write_cmos_sensor(0x9b,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0xce,0xf8);
					SP0A19_write_cmos_sensor(0xcf,0x01);
					SP0A19_write_cmos_sensor(0xd0,0xf8);
					SP0A19_write_cmos_sensor(0xd1,0x01);
					SP0A19_write_cmos_sensor(0xfd,0x00);        			
				       SENSORDB(" video 50Hz normal\r\n");				
				}
				else if(sp0a19_isBanding == 1)
				{
				//Video record daylight 24M 60Hz 14-14FPS maxgain:0x80
					SP0A19_write_cmos_sensor(0xfd,0x00);
					SP0A19_write_cmos_sensor(0x03,0x00);
					SP0A19_write_cmos_sensor(0x04,0xb4);
					SP0A19_write_cmos_sensor(0x06,0x00);
					SP0A19_write_cmos_sensor(0x09,0x03);
					SP0A19_write_cmos_sensor(0x0a,0x31);
					SP0A19_write_cmos_sensor(0xf0,0x3c);
					SP0A19_write_cmos_sensor(0xf1,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0x90,0x08);
					SP0A19_write_cmos_sensor(0x92,0x01);
					SP0A19_write_cmos_sensor(0x98,0x3c);
					SP0A19_write_cmos_sensor(0x99,0x00);
					SP0A19_write_cmos_sensor(0x9a,0x01);
					SP0A19_write_cmos_sensor(0x9b,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0xce,0xe0);
					SP0A19_write_cmos_sensor(0xcf,0x01);
					SP0A19_write_cmos_sensor(0xd0,0xe0);
					SP0A19_write_cmos_sensor(0xd1,0x01);
					SP0A19_write_cmos_sensor(0xfd,0x00);			
				       SENSORDB(" video 60Hz normal \n");	
				}
			   }
		else 
			{
			   //	SENSORDB(" SP0A19_banding=%x\r\n",SP0A19_banding);
			       if(sp0a19_isBanding== 0)
				{
					#if 0
				//capture preview daylight 24M 50hz 20-8FPS maxgain:0x70   
					SP0A19_write_cmos_sensor(0xfd,0x00);
					SP0A19_write_cmos_sensor(0x03,0x01);
					SP0A19_write_cmos_sensor(0x04,0x32);
					SP0A19_write_cmos_sensor(0x06,0x00);
					SP0A19_write_cmos_sensor(0x09,0x01);
					SP0A19_write_cmos_sensor(0x0a,0x46);
					SP0A19_write_cmos_sensor(0xf0,0x66);
					SP0A19_write_cmos_sensor(0xf1,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0x90,0x0c);
					SP0A19_write_cmos_sensor(0x92,0x01);
					SP0A19_write_cmos_sensor(0x98,0x66);
					SP0A19_write_cmos_sensor(0x99,0x00);
					SP0A19_write_cmos_sensor(0x9a,0x01);
					SP0A19_write_cmos_sensor(0x9b,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0xce,0xc8);
					SP0A19_write_cmos_sensor(0xcf,0x04);
					SP0A19_write_cmos_sensor(0xd0,0xc8);
					SP0A19_write_cmos_sensor(0xd1,0x04);
					SP0A19_write_cmos_sensor(0xfd,0x00);	
					#else
					SP0A19_write_cmos_sensor(0xfd,0x00);
					SP0A19_write_cmos_sensor(0x03,0x00);
					SP0A19_write_cmos_sensor(0x04,0x99);
					SP0A19_write_cmos_sensor(0x06,0x00);
					SP0A19_write_cmos_sensor(0x09,0x05);
					SP0A19_write_cmos_sensor(0x0a,0xdf);
					SP0A19_write_cmos_sensor(0xf0,0x33);
					SP0A19_write_cmos_sensor(0xf1,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0x90,0x0e);
					SP0A19_write_cmos_sensor(0x92,0x01);
					SP0A19_write_cmos_sensor(0x98,0x33);
					SP0A19_write_cmos_sensor(0x99,0x00);
					SP0A19_write_cmos_sensor(0x9a,0x01);
					SP0A19_write_cmos_sensor(0x9b,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0xce,0xca);
					SP0A19_write_cmos_sensor(0xcf,0x02);
					SP0A19_write_cmos_sensor(0xd0,0xca);
					SP0A19_write_cmos_sensor(0xd1,0x02);
					SP0A19_write_cmos_sensor(0xfd,0x00);
					#endif	
				       SENSORDB(" priview 50Hz normal\r\n");
				}
				else if(sp0a19_isBanding== 1)
				{
					#if 0
				//capture preview daylight 24M 60hz 20-8FPS maxgain:0x70   
					SP0A19_write_cmos_sensor(0xfd,0x00);
					SP0A19_write_cmos_sensor(0x03,0x00);
					SP0A19_write_cmos_sensor(0x04,0xff);
					SP0A19_write_cmos_sensor(0x06,0x00);
					SP0A19_write_cmos_sensor(0x09,0x01);
					SP0A19_write_cmos_sensor(0x0a,0x46);
					SP0A19_write_cmos_sensor(0xf0,0x55);
					SP0A19_write_cmos_sensor(0xf1,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0x90,0x0f);
					SP0A19_write_cmos_sensor(0x92,0x01);
					SP0A19_write_cmos_sensor(0x98,0x55);
					SP0A19_write_cmos_sensor(0x99,0x00);
					SP0A19_write_cmos_sensor(0x9a,0x01);
					SP0A19_write_cmos_sensor(0x9b,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0xce,0xfb);
					SP0A19_write_cmos_sensor(0xcf,0x04);
					SP0A19_write_cmos_sensor(0xd0,0xfb);
					SP0A19_write_cmos_sensor(0xd1,0x04);
					SP0A19_write_cmos_sensor(0xfd,0x00);	
					#else
					SP0A19_write_cmos_sensor(0xfd,0x00);
					SP0A19_write_cmos_sensor(0x03,0x00);
					SP0A19_write_cmos_sensor(0x04,0x81);
					SP0A19_write_cmos_sensor(0x06,0x00);
					SP0A19_write_cmos_sensor(0x09,0x05);
					SP0A19_write_cmos_sensor(0x0a,0xc4);
					SP0A19_write_cmos_sensor(0xf0,0x2b);
					SP0A19_write_cmos_sensor(0xf1,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0x90,0x11);
					SP0A19_write_cmos_sensor(0x92,0x01);
					SP0A19_write_cmos_sensor(0x98,0x2b);
					SP0A19_write_cmos_sensor(0x99,0x00);
					SP0A19_write_cmos_sensor(0x9a,0x01);
					SP0A19_write_cmos_sensor(0x9b,0x00);
					SP0A19_write_cmos_sensor(0xfd,0x01);
					SP0A19_write_cmos_sensor(0xce,0xdb);
					SP0A19_write_cmos_sensor(0xcf,0x02);
					SP0A19_write_cmos_sensor(0xd0,0xdb);
					SP0A19_write_cmos_sensor(0xd1,0x02);
					SP0A19_write_cmos_sensor(0xfd,0x00);
					#endif
				        SENSORDB(" priview 60Hz normal\r\n");
				}
			       }
	   
	}  
}

/*************************************************************************
* FUNCTION
*	SP0A19_Sensor_Init
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
#if 0
void SP0A19_Sensor_Init(void)
{  
//SP0A19 ini
	  SP0A19_write_cmos_sensor(0xfd,0x00);
	  SP0A19_write_cmos_sensor(0x1C,0x28);
	  SP0A19_write_cmos_sensor(0x32,0x00);
	  SP0A19_write_cmos_sensor(0x0f,0x2f);
	  SP0A19_write_cmos_sensor(0x10,0x2e);
	  SP0A19_write_cmos_sensor(0x11,0x00);
	  SP0A19_write_cmos_sensor(0x12,0x18);
	  SP0A19_write_cmos_sensor(0x13,0x2f);
	  SP0A19_write_cmos_sensor(0x14,0x00);
	  SP0A19_write_cmos_sensor(0x15,0x3f);
	  SP0A19_write_cmos_sensor(0x16,0x00);
	  SP0A19_write_cmos_sensor(0x17,0x18);
	  SP0A19_write_cmos_sensor(0x25,0x40);
	  SP0A19_write_cmos_sensor(0x1a,0x0b);
	  SP0A19_write_cmos_sensor(0x1b,0xc );
	  SP0A19_write_cmos_sensor(0x1e,0xb );
	  SP0A19_write_cmos_sensor(0x20,0x3f); // add
	  SP0A19_write_cmos_sensor(0x21,0x13); // 0x0c 24
	  SP0A19_write_cmos_sensor(0x22,0x19);
	  SP0A19_write_cmos_sensor(0x26,0x1a);
	  SP0A19_write_cmos_sensor(0x27,0xab);
	  SP0A19_write_cmos_sensor(0x28,0xfd);
	  SP0A19_write_cmos_sensor(0x30,0x00);
	  SP0A19_write_cmos_sensor(0x31,0x00);//0x10 2014-6-30 miao
	  SP0A19_write_cmos_sensor(0xfb,0x30); // 0x33
	  SP0A19_write_cmos_sensor(0x1f,0x10);
	  
	  
	//Blacklevel
	  SP0A19_write_cmos_sensor(0xfd,0x00);
	  SP0A19_write_cmos_sensor(0x65,0x06);//blue_suboffset
	  SP0A19_write_cmos_sensor(0x66,0x06);//red_suboffset
	  SP0A19_write_cmos_sensor(0x67,0x06);//gr_suboffset
	  SP0A19_write_cmos_sensor(0x68,0x06);//gb_suboffset
	  SP0A19_write_cmos_sensor(0x45,0x00);
	  SP0A19_write_cmos_sensor(0x46,0x0f);
	//ae setting
	  SP0A19_write_cmos_sensor(0xfd,0x00);
	  SP0A19_write_cmos_sensor(0x03,0x01);
	  SP0A19_write_cmos_sensor(0x04,0x32);
	  SP0A19_write_cmos_sensor(0x06,0x00);
	  SP0A19_write_cmos_sensor(0x09,0x01);
	  SP0A19_write_cmos_sensor(0x0a,0x46);
	  SP0A19_write_cmos_sensor(0xf0,0x66);
	  SP0A19_write_cmos_sensor(0xf1,0x00);
	  SP0A19_write_cmos_sensor(0xfd,0x01);
	  SP0A19_write_cmos_sensor(0x90,0x0c);
	  SP0A19_write_cmos_sensor(0x92,0x01);
	  SP0A19_write_cmos_sensor(0x98,0x66);
	  SP0A19_write_cmos_sensor(0x99,0x00);
	  SP0A19_write_cmos_sensor(0x9a,0x01);
	  SP0A19_write_cmos_sensor(0x9b,0x00);
	  
	//Status
	  SP0A19_write_cmos_sensor(0xfd,0x01);
	  SP0A19_write_cmos_sensor(0xce,0x98);
	  SP0A19_write_cmos_sensor(0xcf,0x04);
	  SP0A19_write_cmos_sensor(0xd0,0x98);
	  SP0A19_write_cmos_sensor(0xd1,0x04); 
	  
	  SP0A19_write_cmos_sensor(0xfd,0x01);
	  SP0A19_write_cmos_sensor(0xc4,0x56);//70
	  SP0A19_write_cmos_sensor(0xc5,0x8f);//74
	  SP0A19_write_cmos_sensor(0xca,0x30);
	  SP0A19_write_cmos_sensor(0xcb,0x45);
	  SP0A19_write_cmos_sensor(0xcc,0x70);//rpc_heq_low
	  SP0A19_write_cmos_sensor(0xcd,0x48);//rpc_heq_dummy
	  SP0A19_write_cmos_sensor(0xfd,0x00);

	  //lsc  for st 
	  SP0A19_write_cmos_sensor(0xfd,0x01);
	  SP0A19_write_cmos_sensor(0x35,0x15);
	  SP0A19_write_cmos_sensor(0x36,0x15); //20
	  SP0A19_write_cmos_sensor(0x37,0x15);
	  SP0A19_write_cmos_sensor(0x38,0x15);
	  SP0A19_write_cmos_sensor(0x39,0x15);
	  SP0A19_write_cmos_sensor(0x3a,0x15); //15
	  SP0A19_write_cmos_sensor(0x3b,0x13);
	  SP0A19_write_cmos_sensor(0x3c,0x15);
	  SP0A19_write_cmos_sensor(0x3d,0x15);
	  SP0A19_write_cmos_sensor(0x3e,0x15); //12
	  SP0A19_write_cmos_sensor(0x3f,0x15);
	  SP0A19_write_cmos_sensor(0x40,0x18);
	  SP0A19_write_cmos_sensor(0x41,0x00);
	  SP0A19_write_cmos_sensor(0x42,0x04);
	  SP0A19_write_cmos_sensor(0x43,0x04);
	  SP0A19_write_cmos_sensor(0x44,0x00);
	  SP0A19_write_cmos_sensor(0x45,0x00);
	  SP0A19_write_cmos_sensor(0x46,0x00);
	  SP0A19_write_cmos_sensor(0x47,0x00);
	  SP0A19_write_cmos_sensor(0x48,0x00);
	  SP0A19_write_cmos_sensor(0x49,0xfd);
	  SP0A19_write_cmos_sensor(0x4a,0x00);
	  SP0A19_write_cmos_sensor(0x4b,0x00);
	  SP0A19_write_cmos_sensor(0x4c,0xfd);
	  SP0A19_write_cmos_sensor(0xfd,0x00);

	//awb 1
	  SP0A19_write_cmos_sensor(0xfd,0x01);
	  SP0A19_write_cmos_sensor(0x28,0xc5);
	  SP0A19_write_cmos_sensor(0x29,0x9b);
	//SP0A19_write_cmos_sensor(0x10,0x08);
	//SP0A19_write_cmos_sensor(0x11,0x14);	
	//SP0A19_write_cmos_sensor(0x12,0x14);
	  SP0A19_write_cmos_sensor(0x2e,0x02);	
	  SP0A19_write_cmos_sensor(0x2f,0x16);
	  SP0A19_write_cmos_sensor(0x17,0x17);
	  SP0A19_write_cmos_sensor(0x18,0x19);	//0x29	 0813
	  SP0A19_write_cmos_sensor(0x19,0x45);	

	//SP0A19_write_cmos_sensor(0x1a,0x9e);//a1;a5   
	//SP0A19_write_cmos_sensor(0x1b,0xae);//b0;9a
	//SP0A19_write_cmos_sensor(0x33,0xef);
	  SP0A19_write_cmos_sensor(0x2a,0xef);
	  SP0A19_write_cmos_sensor(0x2b,0x15);

	  //awb2
	  SP0A19_write_cmos_sensor(0xfd,0x01);
	  SP0A19_write_cmos_sensor(0x73,0x80);
	  SP0A19_write_cmos_sensor(0x1a,0x80);
	  SP0A19_write_cmos_sensor(0x1b,0x80); 
	//d65
	  SP0A19_write_cmos_sensor(0x65,0xd5); //d6
	  SP0A19_write_cmos_sensor(0x66,0xfa); //f0
	  SP0A19_write_cmos_sensor(0x67,0x72); //7a
	  SP0A19_write_cmos_sensor(0x68,0x8a); //9a
	//indoor
	  SP0A19_write_cmos_sensor(0x69,0xc6); //ab
	  SP0A19_write_cmos_sensor(0x6a,0xee); //ca
	  SP0A19_write_cmos_sensor(0x6b,0x94); //a3
	  SP0A19_write_cmos_sensor(0x6c,0xab); //c1
	//f 
	  SP0A19_write_cmos_sensor(0x61,0x7a); //82
	  SP0A19_write_cmos_sensor(0x62,0x98); //a5
	  SP0A19_write_cmos_sensor(0x63,0xc5); //d6
	  SP0A19_write_cmos_sensor(0x64,0xe6); //ec
	  //cwf
	  SP0A19_write_cmos_sensor(0x6d,0xb9); //a5
	  SP0A19_write_cmos_sensor(0x6e,0xde); //c2
	  SP0A19_write_cmos_sensor(0x6f,0xb2); //a7
	  SP0A19_write_cmos_sensor(0x70,0xd5); //c5
	 
	//skin detect
	 SP0A19_write_cmos_sensor(0xfd,0x01);
	 SP0A19_write_cmos_sensor(0x08,0x15);
	 SP0A19_write_cmos_sensor(0x09,0x04);
	 SP0A19_write_cmos_sensor(0x0a,0x20);
	 SP0A19_write_cmos_sensor(0x0b,0x12);
	 SP0A19_write_cmos_sensor(0x0c,0x27);
	 SP0A19_write_cmos_sensor(0x0d,0x06);
	 SP0A19_write_cmos_sensor(0x0f,0x63);//0x5f	0813

	   //BPC_grad
	  SP0A19_write_cmos_sensor(0xfd,0x00);
	  SP0A19_write_cmos_sensor(0x79,0xf0);
	  SP0A19_write_cmos_sensor(0x7a,0x80);  //f0
	  SP0A19_write_cmos_sensor(0x7b,0x80);  //f0
	  SP0A19_write_cmos_sensor(0x7c,0x20); //f0
	  
	//smooth
	  SP0A19_write_cmos_sensor(0xfd,0x00);

	  //
	  SP0A19_write_cmos_sensor(0x57,0x06); //raw_dif_thr_outdoor
	  SP0A19_write_cmos_sensor(0x58,0x0d); //raw_dif_thr_normal
	  SP0A19_write_cmos_sensor(0x56,0x10); //raw_dif_thr_dummy
	  SP0A19_write_cmos_sensor(0x59,0x10); //raw_dif_thr_lowlight
		//GrGb
	  SP0A19_write_cmos_sensor(0x89,0x06); //raw_grgb_thr_outdoor 
	  SP0A19_write_cmos_sensor(0x8a,0x0d); //raw_grgb_thr_normal	
	  SP0A19_write_cmos_sensor(0x9c,0x10); //raw_grgb_thr_dummy	
	  SP0A19_write_cmos_sensor(0x9d,0x10); //raw_grgb_thr_lowlight
		//Gr\Gb
	  SP0A19_write_cmos_sensor(0x81,0xe0); //raw_gflt_fac_outdoor
	  SP0A19_write_cmos_sensor(0x82,0xe0); //raw_gflt_fac_normal
	  SP0A19_write_cmos_sensor(0x83,0x80); //raw_gflt_fac_dummy
	  SP0A19_write_cmos_sensor(0x84,0x40); //raw_gflt_fac_lowlight
		//GrGb
	  SP0A19_write_cmos_sensor(0x85,0xe0); //raw_gf_fac_outdoor  
	  SP0A19_write_cmos_sensor(0x86,0xc0); //raw_gf_fac_normal  
	  SP0A19_write_cmos_sensor(0x87,0x80); //raw_gf_fac_dummy   
	  SP0A19_write_cmos_sensor(0x88,0x40); //raw_gf_fac_lowlight
		//
	  SP0A19_write_cmos_sensor(0x5a,0xff);  //raw_rb_fac_outdoor
	  SP0A19_write_cmos_sensor(0x5b,0xe0);  //raw_rb_fac_normal
	  SP0A19_write_cmos_sensor(0x5c,0x80);  //raw_rb_fac_dummy
	  SP0A19_write_cmos_sensor(0x5d,0x00);  //raw_rb_fac_lowlight
	  
	//sharpen 
	  SP0A19_write_cmos_sensor(0xfd,0x01); 
	  SP0A19_write_cmos_sensor(0xe2,0x30); //sharpen_y_base
	  SP0A19_write_cmos_sensor(0xe4,0xa0); //sharpen_y_max

	  SP0A19_write_cmos_sensor(0xe5,0x04); //rangek_neg_outdoor  //0x08
	  SP0A19_write_cmos_sensor(0xd3,0x04); //rangek_pos_outdoor	//0x08
	  SP0A19_write_cmos_sensor(0xd7,0x04); //range_base_outdoor	//0x08
	 
	  SP0A19_write_cmos_sensor(0xe6,0x04); //rangek_neg_normal   // 0x08
	  SP0A19_write_cmos_sensor(0xd4,0x04); //rangek_pos_normal   // 0x08
	  SP0A19_write_cmos_sensor(0xd8,0x04); //range_base_normal   // 0x08
	  
	  SP0A19_write_cmos_sensor(0xe7,0x08); //rangek_neg_dummy   // 0x10
	  SP0A19_write_cmos_sensor(0xd5,0x08); //rangek_pos_dummy   // 0x10
	  SP0A19_write_cmos_sensor(0xd9,0x08); //range_base_dummy    // 0x10
		
	  SP0A19_write_cmos_sensor(0xd2,0x10); //rangek_neg_lowlight
	  SP0A19_write_cmos_sensor(0xd6,0x10); //rangek_pos_lowlight
	  SP0A19_write_cmos_sensor(0xda,0x10); //range_base_lowlight
	 
	  SP0A19_write_cmos_sensor(0xe8,0x20);  //sharp_fac_pos_outdoor  // 0x35
	  SP0A19_write_cmos_sensor(0xec,0x20);  //sharp_fac_neg_outdoor
	  SP0A19_write_cmos_sensor(0xe9,0x20);  //sharp_fac_pos_nr	  // 0x35
	  SP0A19_write_cmos_sensor(0xed,0x20);  //sharp_fac_neg_nr
	  SP0A19_write_cmos_sensor(0xea,0x20);  //sharp_fac_pos_dummy   // 0x30
	  SP0A19_write_cmos_sensor(0xef,0x20);  //sharp_fac_neg_dummy   // 0x20
	  SP0A19_write_cmos_sensor(0xeb,0x10);  //sharp_fac_pos_low
	  SP0A19_write_cmos_sensor(0xf0,0x20);  //sharp_fac_neg_low 
	  
	//CCM
	  SP0A19_write_cmos_sensor(0xfd,0x01);
	  SP0A19_write_cmos_sensor(0xa0,0x85);  //0x80  //zouyu 20121009
	  SP0A19_write_cmos_sensor(0xa1,0x00);
	  SP0A19_write_cmos_sensor(0xa2,0x00);
	  SP0A19_write_cmos_sensor(0xa3,0xf3);  // 0xf6
	  SP0A19_write_cmos_sensor(0xa4,0x8e);  // 0x99
	  SP0A19_write_cmos_sensor(0xa5,0x00);  // 0xf2
	  SP0A19_write_cmos_sensor(0xa6,0x00);  // 0x0d
	  SP0A19_write_cmos_sensor(0xa7,0xe6);   // 0xda
	  SP0A19_write_cmos_sensor(0xa8,0x9a);//0xa0 0813  // 0x98
	  SP0A19_write_cmos_sensor(0xa9,0x00);
	  SP0A19_write_cmos_sensor(0xaa,0x03);  // 0x33
	  SP0A19_write_cmos_sensor(0xab,0x0c);
	  SP0A19_write_cmos_sensor(0xfd,0x00);

			//gamma  
	  SP0A19_write_cmos_sensor(0xfd,0x00);
	  SP0A19_write_cmos_sensor(0x8b,0x0 );  // 00;0 ;0 
	  SP0A19_write_cmos_sensor(0x8c,0xC );  // 0f;C ;11
	  SP0A19_write_cmos_sensor(0x8d,0x19);  // 1e;19;19
	  SP0A19_write_cmos_sensor(0x8e,0x2C);  // 3d;2C;28
	  SP0A19_write_cmos_sensor(0x8f,0x49);  // 6c;49;46
	  SP0A19_write_cmos_sensor(0x90,0x61);  // 92;61;61
	  SP0A19_write_cmos_sensor(0x91,0x77);  // aa;77;78
	  SP0A19_write_cmos_sensor(0x92,0x8A);  // b9;8A;8A
	  SP0A19_write_cmos_sensor(0x93,0x9B);  // c4;9B;9B
	  SP0A19_write_cmos_sensor(0x94,0xA9);  // cf;A9;A9
	  SP0A19_write_cmos_sensor(0x95,0xB5);  // d4;B5;B5
	  SP0A19_write_cmos_sensor(0x96,0xC0);  // da;C0;C0
	  SP0A19_write_cmos_sensor(0x97,0xCA);  // e0;CA;CA
	  SP0A19_write_cmos_sensor(0x98,0xD4);  // e4;D4;D4
	  SP0A19_write_cmos_sensor(0x99,0xDD);  // e8;DD;DD
	  SP0A19_write_cmos_sensor(0x9a,0xE6);  // ec;E6;E6
	  SP0A19_write_cmos_sensor(0x9b,0xEF);  // f1;EF;EF
	  SP0A19_write_cmos_sensor(0xfd,0x01);  // 01;01;01
	  SP0A19_write_cmos_sensor(0x8d,0xF7);  // f7;F7;F7
	  SP0A19_write_cmos_sensor(0x8e,0xFF);  // ff;FF;FF		 
	  SP0A19_write_cmos_sensor(0xfd,0x00);  //

	   //rpc
	  SP0A19_write_cmos_sensor(0xfd,0x00); 
	  SP0A19_write_cmos_sensor(0xe0,0x4c); //  4c;44;4c;3e;3c;3a;38;rpc_1base_max
	  SP0A19_write_cmos_sensor(0xe1,0x3c); //  3c;36;3c;30;2e;2c;2a;rpc_2base_max
	  SP0A19_write_cmos_sensor(0xe2,0x34); //  34;2e;34;2a;28;26;26;rpc_3base_max
	  SP0A19_write_cmos_sensor(0xe3,0x2e); //  2e;2a;2e;26;24;22;rpc_4base_max
	  SP0A19_write_cmos_sensor(0xe4,0x2e); //  2e;2a;2e;26;24;22;rpc_5base_max
	  SP0A19_write_cmos_sensor(0xe5,0x2c); //  2c;28;2c;24;22;20;rpc_6base_max
	  SP0A19_write_cmos_sensor(0xe6,0x2c); //  2c;28;2c;24;22;20;rpc_7base_max
	  SP0A19_write_cmos_sensor(0xe8,0x2a); //  2a;26;2a;22;20;20;1e;rpc_8base_max
	  SP0A19_write_cmos_sensor(0xe9,0x2a); //  2a;26;2a;22;20;20;1e;rpc_9base_max 
	  SP0A19_write_cmos_sensor(0xea,0x2a); //  2a;26;2a;22;20;20;1e;rpc_10base_max
	  SP0A19_write_cmos_sensor(0xeb,0x28); //  28;24;28;20;1f;1e;1d;rpc_11base_max
	  SP0A19_write_cmos_sensor(0xf5,0x28); //  28;24;28;20;1f;1e;1d;rpc_12base_max
	  SP0A19_write_cmos_sensor(0xf6,0x28); //  28;24;28;20;1f;1e;1d;rpc_13base_max	

	//ae min gain  
	  SP0A19_write_cmos_sensor(0xfd,0x01);
	  SP0A19_write_cmos_sensor(0x94,0xa0);  //rpc_max_indr
	  SP0A19_write_cmos_sensor(0x95,0x28);   // 1e rpc_min_indr 
	  SP0A19_write_cmos_sensor(0x9c,0xa0);  //rpc_max_outdr
	  SP0A19_write_cmos_sensor(0x9d,0x28);  //rpc_min_outdr	 
	//ae target
	  SP0A19_write_cmos_sensor(0xfd,0x00); 
	  SP0A19_write_cmos_sensor(0xed,0x8c); //80 
	  SP0A19_write_cmos_sensor(0xf7,0x88); //7c 
	  SP0A19_write_cmos_sensor(0xf8,0x80); //70 
	  SP0A19_write_cmos_sensor(0xec,0x7c); //6c  
	  
	  SP0A19_write_cmos_sensor(0xef,0x74); //99
	  SP0A19_write_cmos_sensor(0xf9,0x70); //90
	  SP0A19_write_cmos_sensor(0xfa,0x68); //80
	  SP0A19_write_cmos_sensor(0xee,0x64); //78

		
	//gray detect
	  SP0A19_write_cmos_sensor(0xfd,0x01);
	  SP0A19_write_cmos_sensor(0x30,0x40);
	  //add 0813 
	  SP0A19_write_cmos_sensor(0x31,0x70);
	  SP0A19_write_cmos_sensor(0x32,0x40);
	  SP0A19_write_cmos_sensor(0x33,0xef);
	  SP0A19_write_cmos_sensor(0x34,0x05);
	  SP0A19_write_cmos_sensor(0x4d,0x2f);
	  SP0A19_write_cmos_sensor(0x4e,0x20);
	  SP0A19_write_cmos_sensor(0x4f,0x16);

	//lowlight lum
	  SP0A19_write_cmos_sensor(0xfd,0x00); //
	  SP0A19_write_cmos_sensor(0xb2,0x20); //lum_limit  // 0x10
	  SP0A19_write_cmos_sensor(0xb3,0x1f); //lum_set
	  SP0A19_write_cmos_sensor(0xb4,0x30); //black_vt  // 0x20
	  SP0A19_write_cmos_sensor(0xb5,0x45); //white_vt

	//saturation
	  SP0A19_write_cmos_sensor(0xfd,0x00); 
	  SP0A19_write_cmos_sensor(0xbe,0xff); 
	  SP0A19_write_cmos_sensor(0xbf,0x01); 
	  SP0A19_write_cmos_sensor(0xc0,0xff); 
	  SP0A19_write_cmos_sensor(0xc1,0xd8); 
	  SP0A19_write_cmos_sensor(0xd3,0x88); //0x78
	  SP0A19_write_cmos_sensor(0xd4,0x88); //0x78
	  SP0A19_write_cmos_sensor(0xd6,0x70); //0x78 	   (0xd7,0x60); //0x78
	  SP0A19_write_cmos_sensor(0xd7,0x60);

	//HEQ
	  SP0A19_write_cmos_sensor(0xfd,0x00); 
	  SP0A19_write_cmos_sensor(0xdc,0x00); 
	  SP0A19_write_cmos_sensor(0xdd,0x80); //0x80 0813  // 0x78
	  SP0A19_write_cmos_sensor(0xde,0xa8); //80  0x88  0813
	  SP0A19_write_cmos_sensor(0xdf,0x80); 
	   
	//func enable
	  SP0A19_write_cmos_sensor(0xfd,0x00);  
	  SP0A19_write_cmos_sensor(0x32,0x15);  //0x0d
	  SP0A19_write_cmos_sensor(0x34,0x76);  //16
	  SP0A19_write_cmos_sensor(0x35,0x40);  //00
	  SP0A19_write_cmos_sensor(0x33,0xef);  
	  SP0A19_write_cmos_sensor(0x5f,0x51); 
}
#else
void SP0A19_Sensor_Init(void)
{
	SP0A19_write_cmos_sensor(0xfd,0x00);
	SP0A19_write_cmos_sensor(0x1C,0x28);
	SP0A19_write_cmos_sensor(0x32,0x00);
	SP0A19_write_cmos_sensor(0x0f,0x2f);
	SP0A19_write_cmos_sensor(0x10,0x2e);
	SP0A19_write_cmos_sensor(0x11,0x00);
	SP0A19_write_cmos_sensor(0x12,0x18);
	SP0A19_write_cmos_sensor(0x13,0x2f);
	SP0A19_write_cmos_sensor(0x14,0x00);
	SP0A19_write_cmos_sensor(0x15,0x3f);
	SP0A19_write_cmos_sensor(0x16,0x00);
	SP0A19_write_cmos_sensor(0x17,0x18);
	SP0A19_write_cmos_sensor(0x25,0x40);
	SP0A19_write_cmos_sensor(0x1a,0x0b);
	SP0A19_write_cmos_sensor(0x1b,0xc );
	SP0A19_write_cmos_sensor(0x1e,0xb );
	SP0A19_write_cmos_sensor(0x20,0x3f); // add
	SP0A19_write_cmos_sensor(0x21,0x13); // 0x0c 24
	SP0A19_write_cmos_sensor(0x22,0x19);
	SP0A19_write_cmos_sensor(0x26,0x1a);
	SP0A19_write_cmos_sensor(0x27,0xab);
	SP0A19_write_cmos_sensor(0x28,0xfd);
	SP0A19_write_cmos_sensor(0x30,0x00);
	SP0A19_write_cmos_sensor(0x31,0x00);//0x10 2014-6-30 miao
	SP0A19_write_cmos_sensor(0xfb,0x33); // 0x33
	SP0A19_write_cmos_sensor(0x1f,0x08);
	SP0A19_write_cmos_sensor(0xfd,0x00);
	SP0A19_write_cmos_sensor(0x65,0x00);//blue_suboffset
	SP0A19_write_cmos_sensor(0x66,0x00);//red_suboffset
	SP0A19_write_cmos_sensor(0x67,0x00);//gr_suboffset
	SP0A19_write_cmos_sensor(0x68,0x00);//gb_suboffset
	SP0A19_write_cmos_sensor(0x45,0x00);
	SP0A19_write_cmos_sensor(0x46,0x0f);
	SP0A19_write_cmos_sensor(0xfd,0x00);
	SP0A19_write_cmos_sensor(0x03,0x00);
	SP0A19_write_cmos_sensor(0x04,0x99);
	SP0A19_write_cmos_sensor(0x06,0x00);
	SP0A19_write_cmos_sensor(0x09,0x05);
	SP0A19_write_cmos_sensor(0x0a,0xdf);
	SP0A19_write_cmos_sensor(0xf0,0x33);
	SP0A19_write_cmos_sensor(0xf1,0x00);
	SP0A19_write_cmos_sensor(0xfd,0x01);
	SP0A19_write_cmos_sensor(0x90,0x0e);
	SP0A19_write_cmos_sensor(0x92,0x01);
	SP0A19_write_cmos_sensor(0x98,0x33);
	SP0A19_write_cmos_sensor(0x99,0x00);
	SP0A19_write_cmos_sensor(0x9a,0x01);
	SP0A19_write_cmos_sensor(0x9b,0x00);
	SP0A19_write_cmos_sensor(0xfd,0x01);
	SP0A19_write_cmos_sensor(0xce,0xca);
	SP0A19_write_cmos_sensor(0xcf,0x02);
	SP0A19_write_cmos_sensor(0xd0,0xca);
	SP0A19_write_cmos_sensor(0xd1,0x02);
	SP0A19_write_cmos_sensor(0xfd,0x00);
	SP0A19_write_cmos_sensor(0xfd,0x01);
	SP0A19_write_cmos_sensor(0xc4,0x56);//70
	SP0A19_write_cmos_sensor(0xc5,0x8f);//74
	SP0A19_write_cmos_sensor(0xca,0x30);
	SP0A19_write_cmos_sensor(0xcb,0x45);
	SP0A19_write_cmos_sensor(0xcc,0x70);//rpc_heq_low
	SP0A19_write_cmos_sensor(0xcd,0x48);//rpc_heq_dummy
	SP0A19_write_cmos_sensor(0xfd,0x00);
	SP0A19_write_cmos_sensor(0xfd,0x01);
	SP0A19_write_cmos_sensor(0x35,0x15);
	SP0A19_write_cmos_sensor(0x36,0x15); //20
	SP0A19_write_cmos_sensor(0x37,0x15);
	SP0A19_write_cmos_sensor(0x38,0x15);
	SP0A19_write_cmos_sensor(0x39,0x15);
	SP0A19_write_cmos_sensor(0x3a,0x15); //15
	SP0A19_write_cmos_sensor(0x3b,0x13);
	SP0A19_write_cmos_sensor(0x3c,0x15);
	SP0A19_write_cmos_sensor(0x3d,0x15);
	SP0A19_write_cmos_sensor(0x3e,0x15); //12
	SP0A19_write_cmos_sensor(0x3f,0x15);
	SP0A19_write_cmos_sensor(0x40,0x18);
	SP0A19_write_cmos_sensor(0x41,0x00);
	SP0A19_write_cmos_sensor(0x42,0x04);
	SP0A19_write_cmos_sensor(0x43,0x04);
	SP0A19_write_cmos_sensor(0x44,0x00);
	SP0A19_write_cmos_sensor(0x45,0x00);
	SP0A19_write_cmos_sensor(0x46,0x00);
	SP0A19_write_cmos_sensor(0x47,0x00);
	SP0A19_write_cmos_sensor(0x48,0x00);
	SP0A19_write_cmos_sensor(0x49,0xfd);
	SP0A19_write_cmos_sensor(0x4a,0x00);
	SP0A19_write_cmos_sensor(0x4b,0x00);
	SP0A19_write_cmos_sensor(0x4c,0xfd);
	SP0A19_write_cmos_sensor(0xfd,0x00);
	SP0A19_write_cmos_sensor(0xfd,0x01);
	SP0A19_write_cmos_sensor(0x28,0xc5);
	SP0A19_write_cmos_sensor(0x29,0x9b);
	SP0A19_write_cmos_sensor(0x2e,0x02);	
	SP0A19_write_cmos_sensor(0x2f,0x16);
	SP0A19_write_cmos_sensor(0x17,0x17);
	SP0A19_write_cmos_sensor(0x18,0x19);	//0x29	 0813
	SP0A19_write_cmos_sensor(0x19,0x45);	
	SP0A19_write_cmos_sensor(0x2a,0xef);
	SP0A19_write_cmos_sensor(0x2b,0x15);
	SP0A19_write_cmos_sensor(0xfd,0x01);
	SP0A19_write_cmos_sensor(0x73,0x80);
	SP0A19_write_cmos_sensor(0x1a,0x80);
	SP0A19_write_cmos_sensor(0x1b,0x80); 
	SP0A19_write_cmos_sensor(0x65,0xd5); //d6
	SP0A19_write_cmos_sensor(0x66,0xfa); //f0
	SP0A19_write_cmos_sensor(0x67,0x72); //7a
	SP0A19_write_cmos_sensor(0x68,0x8a); //9a
	SP0A19_write_cmos_sensor(0x69,0xc6); //ab
	SP0A19_write_cmos_sensor(0x6a,0xee); //ca
	SP0A19_write_cmos_sensor(0x6b,0x94); //a3
	SP0A19_write_cmos_sensor(0x6c,0xab); //c1
	SP0A19_write_cmos_sensor(0x61,0x7a); //82
	SP0A19_write_cmos_sensor(0x62,0x98); //a5
	SP0A19_write_cmos_sensor(0x63,0xc5); //d6
	SP0A19_write_cmos_sensor(0x64,0xe6); //ec
	SP0A19_write_cmos_sensor(0x6d,0xb9); //a5
	SP0A19_write_cmos_sensor(0x6e,0xde); //c2
	SP0A19_write_cmos_sensor(0x6f,0xb2); //a7
	SP0A19_write_cmos_sensor(0x70,0xd5); //c5
	SP0A19_write_cmos_sensor(0xfd,0x01);
	SP0A19_write_cmos_sensor(0x08,0x15);
	SP0A19_write_cmos_sensor(0x09,0x04);
	SP0A19_write_cmos_sensor(0x0a,0x20);
	SP0A19_write_cmos_sensor(0x0b,0x12);
	SP0A19_write_cmos_sensor(0x0c,0x27);
	SP0A19_write_cmos_sensor(0x0d,0x06);
	SP0A19_write_cmos_sensor(0x0f,0x63);//0x5f	0813
	SP0A19_write_cmos_sensor(0xfd,0x00);
	SP0A19_write_cmos_sensor(0x79,0xf0);
	SP0A19_write_cmos_sensor(0x7a,0x80);  //f0
	SP0A19_write_cmos_sensor(0x7b,0x80);  //f0
	SP0A19_write_cmos_sensor(0x7c,0x20); //f0
	SP0A19_write_cmos_sensor(0xfd,0x00);
	SP0A19_write_cmos_sensor(0x57,0x08);	//raw_dif_thr_outdoor
	SP0A19_write_cmos_sensor(0x58,0x0d); //raw_dif_thr_normal
	SP0A19_write_cmos_sensor(0x56,0x12); //raw_dif_thr_dummy
	SP0A19_write_cmos_sensor(0x59,0x15); //raw_dif_thr_lowlight
	SP0A19_write_cmos_sensor(0x89,0x08);	//raw_grgb_thr_outdoor 
	SP0A19_write_cmos_sensor(0x8a,0x0d); //raw_grgb_thr_normal  
	SP0A19_write_cmos_sensor(0x9c,0x12); //raw_grgb_thr_dummy   
	SP0A19_write_cmos_sensor(0x9d,0x15); //raw_grgb_thr_lowlight
	SP0A19_write_cmos_sensor(0x81,0xe0);    //raw_gflt_fac_outdoor
	SP0A19_write_cmos_sensor(0x82,0x80); //80//raw_gflt_fac_normal
	SP0A19_write_cmos_sensor(0x83,0x40);    //raw_gflt_fac_dummy
	SP0A19_write_cmos_sensor(0x84,0x20);    //raw_gflt_fac_lowlight
	SP0A19_write_cmos_sensor(0x85,0xe0); //raw_gf_fac_outdoor  
	SP0A19_write_cmos_sensor(0x86,0x80); //raw_gf_fac_normal  
	SP0A19_write_cmos_sensor(0x87,0x40); //raw_gf_fac_dummy   
	SP0A19_write_cmos_sensor(0x88,0x20); //raw_gf_fac_lowlight
	SP0A19_write_cmos_sensor(0x5a,0xff);		 //raw_rb_fac_outdoor
	SP0A19_write_cmos_sensor(0x5b,0xe0); //40//raw_rb_fac_normal
	SP0A19_write_cmos_sensor(0x5c,0x80); 	 //raw_rb_fac_dummy
	SP0A19_write_cmos_sensor(0x5d,0x20); 	 //raw_rb_fac_lowlight
	SP0A19_write_cmos_sensor(0xfd,0x01); 
	SP0A19_write_cmos_sensor(0xe2,0x30); //sharpen_y_base
	SP0A19_write_cmos_sensor(0xe4,0xa0); //sharpen_y_max
	SP0A19_write_cmos_sensor(0xe5,0x04); //rangek_neg_outdoor  //0x08
	SP0A19_write_cmos_sensor(0xd3,0x04); //rangek_pos_outdoor	//0x08
	SP0A19_write_cmos_sensor(0xd7,0x04); //range_base_outdoor	//0x08
	SP0A19_write_cmos_sensor(0xe6,0x07); //rangek_neg_normal   // 0x08
	SP0A19_write_cmos_sensor(0xd4,0x07); //rangek_pos_normal   // 0x08
	SP0A19_write_cmos_sensor(0xd8,0x07); //range_base_normal   // 0x08
	SP0A19_write_cmos_sensor(0xe7,0x0a); //rangek_neg_dummy   // 0x10
	SP0A19_write_cmos_sensor(0xd5,0x0a); //rangek_pos_dummy   // 0x10
	SP0A19_write_cmos_sensor(0xd9,0x0a); //range_base_dummy    // 0x10
	SP0A19_write_cmos_sensor(0xd2,0x12); //rangek_neg_lowlight
	SP0A19_write_cmos_sensor(0xd6,0x12); //rangek_pos_lowlight
	SP0A19_write_cmos_sensor(0xda,0x12); //range_base_lowlight
	SP0A19_write_cmos_sensor(0xe8,0x20);  //sharp_fac_pos_outdoor  // 0x35
	SP0A19_write_cmos_sensor(0xec,0x2c);  //sharp_fac_neg_outdoor
	SP0A19_write_cmos_sensor(0xe9,0x20);  //sharp_fac_pos_nr	  // 0x35
	SP0A19_write_cmos_sensor(0xed,0x2c);  //sharp_fac_neg_nr
	SP0A19_write_cmos_sensor(0xea,0x20);  //sharp_fac_pos_dummy   // 0x30
	SP0A19_write_cmos_sensor(0xef,0x24);  //sharp_fac_neg_dummy   // 0x20
	SP0A19_write_cmos_sensor(0xeb,0x10);  //sharp_fac_pos_low
	SP0A19_write_cmos_sensor(0xf0,0x1c);  //sharp_fac_neg_low 
	SP0A19_write_cmos_sensor(0xfd,0x01);
	SP0A19_write_cmos_sensor(0xa0,0x80);  //0x80  //zouyu 20121009
	SP0A19_write_cmos_sensor(0xa1,0x00);
	SP0A19_write_cmos_sensor(0xa2,0x00);
	SP0A19_write_cmos_sensor(0xa3,0xf3);  // 0xf6
	SP0A19_write_cmos_sensor(0xa4,0x8e);  // 0x99
	SP0A19_write_cmos_sensor(0xa5,0x00);  // 0xf2
	SP0A19_write_cmos_sensor(0xa6,0x00);  // 0x0d
	SP0A19_write_cmos_sensor(0xa7,0xe6);   // 0xda
	SP0A19_write_cmos_sensor(0xa8,0x9a);//0xa0 0813  // 0x98
	SP0A19_write_cmos_sensor(0xa9,0x00);
	SP0A19_write_cmos_sensor(0xaa,0x03);  // 0x33
	SP0A19_write_cmos_sensor(0xab,0x0c);
	SP0A19_write_cmos_sensor(0xfd,0x00);
	SP0A19_write_cmos_sensor(0xfd,0x00);
	SP0A19_write_cmos_sensor(0x8b,0x0 );//0 
	SP0A19_write_cmos_sensor(0x8c,0x9 );//11
	SP0A19_write_cmos_sensor(0x8d,0x14);//19 
	SP0A19_write_cmos_sensor(0x8e,0x26);//28 
	SP0A19_write_cmos_sensor(0x8f,0x45);//46 
	SP0A19_write_cmos_sensor(0x90,0x5e);//61 
	SP0A19_write_cmos_sensor(0x91,0x74);//78 
	SP0A19_write_cmos_sensor(0x92,0x88);//8A 
	SP0A19_write_cmos_sensor(0x93,0x99);//9B 
	SP0A19_write_cmos_sensor(0x94,0xA7);//A9 
	SP0A19_write_cmos_sensor(0x95,0xB5);//B5 
	SP0A19_write_cmos_sensor(0x96,0xC0);//C0 
	SP0A19_write_cmos_sensor(0x97,0xCA);//CA 
	SP0A19_write_cmos_sensor(0x98,0xD4);//D4 
	SP0A19_write_cmos_sensor(0x99,0xDc);//DD 
	SP0A19_write_cmos_sensor(0x9a,0xE3);//E6 
	SP0A19_write_cmos_sensor(0x9b,0xE9);//EF 
	SP0A19_write_cmos_sensor(0xfd,0x01);//01 
	SP0A19_write_cmos_sensor(0x8d,0xee);//F7 
	SP0A19_write_cmos_sensor(0x8e,0xF2);//FF 
	SP0A19_write_cmos_sensor(0xfd,0x00);  //
	SP0A19_write_cmos_sensor(0xfd,0x00); 
	SP0A19_write_cmos_sensor(0xe0,0x4c); //  4c;44;4c;3e;3c;3a;38;rpc_1base_max
	SP0A19_write_cmos_sensor(0xe1,0x3c); //  3c;36;3c;30;2e;2c;2a;rpc_2base_max
	SP0A19_write_cmos_sensor(0xe2,0x34); //  34;2e;34;2a;28;26;26;rpc_3base_max
	SP0A19_write_cmos_sensor(0xe3,0x2e); //  2e;2a;2e;26;24;22;rpc_4base_max
	SP0A19_write_cmos_sensor(0xe4,0x2e); //  2e;2a;2e;26;24;22;rpc_5base_max
	SP0A19_write_cmos_sensor(0xe5,0x2c); //  2c;28;2c;24;22;20;rpc_6base_max
	SP0A19_write_cmos_sensor(0xe6,0x2c); //  2c;28;2c;24;22;20;rpc_7base_max
	SP0A19_write_cmos_sensor(0xe8,0x2a); //  2a;26;2a;22;20;20;1e;rpc_8base_max
	SP0A19_write_cmos_sensor(0xe9,0x2a); //  2a;26;2a;22;20;20;1e;rpc_9base_max 
	SP0A19_write_cmos_sensor(0xea,0x2a); //  2a;26;2a;22;20;20;1e;rpc_10base_max
	SP0A19_write_cmos_sensor(0xeb,0x28); //  28;24;28;20;1f;1e;1d;rpc_11base_max
	SP0A19_write_cmos_sensor(0xf5,0x28); //  28;24;28;20;1f;1e;1d;rpc_12base_max
	SP0A19_write_cmos_sensor(0xf6,0x28); //  28;24;28;20;1f;1e;1d;rpc_13base_max	
	SP0A19_write_cmos_sensor(0xfd,0x01);
	SP0A19_write_cmos_sensor(0x94,0xa0);  //rpc_max_indr
	SP0A19_write_cmos_sensor(0x95,0x28);   // 1e rpc_min_indr 
	SP0A19_write_cmos_sensor(0x9c,0xa0);  //rpc_max_outdr
	SP0A19_write_cmos_sensor(0x9d,0x28);  //rpc_min_outdr	 
	SP0A19_write_cmos_sensor(0xfd,0x00); 
	SP0A19_write_cmos_sensor(0xed,0x8c); //80 
	SP0A19_write_cmos_sensor(0xf7,0x88); //7c 
	SP0A19_write_cmos_sensor(0xf8,0x80); //70 
	SP0A19_write_cmos_sensor(0xec,0x7c); //6c  
	SP0A19_write_cmos_sensor(0xef,0x74); //99
	SP0A19_write_cmos_sensor(0xf9,0x70); //90
	SP0A19_write_cmos_sensor(0xfa,0x68); //80
	SP0A19_write_cmos_sensor(0xee,0x64); //78
	SP0A19_write_cmos_sensor(0xfd,0x01);
	SP0A19_write_cmos_sensor(0x30,0x40);
	SP0A19_write_cmos_sensor(0x31,0x70);
	SP0A19_write_cmos_sensor(0x32,0x40);
	SP0A19_write_cmos_sensor(0x33,0xef);
	SP0A19_write_cmos_sensor(0x34,0x05);
	SP0A19_write_cmos_sensor(0x4d,0x2f);
	SP0A19_write_cmos_sensor(0x4e,0x20);
	SP0A19_write_cmos_sensor(0x4f,0x16);
	SP0A19_write_cmos_sensor(0xfd,0x00); //
	SP0A19_write_cmos_sensor(0xb2,0x20); //lum_limit  // 0x10
	SP0A19_write_cmos_sensor(0xb3,0x1f); //lum_set
	SP0A19_write_cmos_sensor(0xb4,0x30); //black_vt  // 0x20
	SP0A19_write_cmos_sensor(0xb5,0x45); //white_vt
	SP0A19_write_cmos_sensor(0xfd,0x00); 
	SP0A19_write_cmos_sensor(0xbe,0xff); 
	SP0A19_write_cmos_sensor(0xbf,0x01); 
	SP0A19_write_cmos_sensor(0xc0,0xff); 
	SP0A19_write_cmos_sensor(0xc1,0xd8); 
	SP0A19_write_cmos_sensor(0xd3,0x80); //0x78
	SP0A19_write_cmos_sensor(0xd4,0x7c); //0x78
	SP0A19_write_cmos_sensor(0xd6,0x74); //0x78 	   (0xd7,0x60); //0x78
	SP0A19_write_cmos_sensor(0xd7,0x50);
	SP0A19_write_cmos_sensor(0xfd,0x00); 
	SP0A19_write_cmos_sensor(0xdc,0x00); 
	SP0A19_write_cmos_sensor(0xdd,0x78); //0x80 0813  // 0x78
	SP0A19_write_cmos_sensor(0xde,0xb8); //80  0x88  0813
	SP0A19_write_cmos_sensor(0xdf,0x80); 
	SP0A19_write_cmos_sensor(0xfd,0x00);  
	SP0A19_write_cmos_sensor(0x32,0x15);  //0x0d
	SP0A19_write_cmos_sensor(0x34,0x76);  //16
	SP0A19_write_cmos_sensor(0x35,0x40);  //00
	SP0A19_write_cmos_sensor(0x33,0xef);  
	SP0A19_write_cmos_sensor(0x5f,0x51); 
	SP0A19_write_cmos_sensor(0xf4,0x09);
}
#endif


/*************************************************************************
* FUNCTION
*	GC329_Lens_Select
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate lens parameter.
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
void SP0A19_Lens_Select(kal_uint8 Lens_Tag)
{
	switch(Lens_Tag)
	{
		case CHT_806C_2:
			#if 0
			SP0A19_write_cmos_sensor(0xfe, 0x01);
			SP0A19_write_cmos_sensor(0xa0, 0x00);
			SP0A19_write_cmos_sensor(0xa1, 0x3c);
			SP0A19_write_cmos_sensor(0xa2, 0x50);
			SP0A19_write_cmos_sensor(0xa3, 0x00);
			SP0A19_write_cmos_sensor(0xa4, 0x00);
			SP0A19_write_cmos_sensor(0xa5, 0x00);
			SP0A19_write_cmos_sensor(0xa6, 0x00);
			SP0A19_write_cmos_sensor(0xa7, 0x04);
			
			SP0A19_write_cmos_sensor(0xa8, 0x0f);
			SP0A19_write_cmos_sensor(0xa9, 0x08);
			SP0A19_write_cmos_sensor(0xaa, 0x00);
			SP0A19_write_cmos_sensor(0xab, 0x04);
			SP0A19_write_cmos_sensor(0xac, 0x00);
			SP0A19_write_cmos_sensor(0xad, 0x07);
			SP0A19_write_cmos_sensor(0xae, 0x0e);
			SP0A19_write_cmos_sensor(0xaf, 0x00);
			SP0A19_write_cmos_sensor(0xb0, 0x00);
			SP0A19_write_cmos_sensor(0xb1, 0x09);
			SP0A19_write_cmos_sensor(0xb2, 0x00);
			SP0A19_write_cmos_sensor(0xb3, 0x00);

			SP0A19_write_cmos_sensor(0xb4, 0x30);
			SP0A19_write_cmos_sensor(0xb5, 0x19);
			SP0A19_write_cmos_sensor(0xb6, 0x21);
			SP0A19_write_cmos_sensor(0xba, 0x3e);
			SP0A19_write_cmos_sensor(0xbb, 0x26);
			SP0A19_write_cmos_sensor(0xbc, 0x2f);
			SP0A19_write_cmos_sensor(0xc0, 0x15);
			SP0A19_write_cmos_sensor(0xc1, 0x11);
			SP0A19_write_cmos_sensor(0xc2, 0x15);
			SP0A19_write_cmos_sensor(0xc6, 0x1f);
			SP0A19_write_cmos_sensor(0xc7, 0x16);
			SP0A19_write_cmos_sensor(0xc8, 0x16);

			SP0A19_write_cmos_sensor(0xb7, 0x00);
			SP0A19_write_cmos_sensor(0xb8, 0x00);
			SP0A19_write_cmos_sensor(0xb9, 0x00);
			SP0A19_write_cmos_sensor(0xbd, 0x00);
			SP0A19_write_cmos_sensor(0xbe, 0x00);
			SP0A19_write_cmos_sensor(0xbf, 0x00);
			SP0A19_write_cmos_sensor(0xc3, 0x00);
			SP0A19_write_cmos_sensor(0xc4, 0x00);
			SP0A19_write_cmos_sensor(0xc5, 0x00);
			SP0A19_write_cmos_sensor(0xc9, 0x0d);
			SP0A19_write_cmos_sensor(0xca, 0x00);
			SP0A19_write_cmos_sensor(0xcb, 0x00);
			
			SP0A19_write_cmos_sensor(0xfe, 0x00);
			#endif
			break;

		case CHT_808C_2:
			#if 0
			SP0A19_write_cmos_sensor(0xfe, 0x01);
			SP0A19_write_cmos_sensor(0xa0, 0x00);
			SP0A19_write_cmos_sensor(0xa1, 0x3c);
			SP0A19_write_cmos_sensor(0xa2, 0x50);
			SP0A19_write_cmos_sensor(0xa3, 0x00);
			SP0A19_write_cmos_sensor(0xa4, 0x00);
			SP0A19_write_cmos_sensor(0xa5, 0x02);
			SP0A19_write_cmos_sensor(0xa6, 0x00);
			SP0A19_write_cmos_sensor(0xa7, 0x00);

			SP0A19_write_cmos_sensor(0xa8, 0x0c);
			SP0A19_write_cmos_sensor(0xa9, 0x03);
			SP0A19_write_cmos_sensor(0xaa, 0x00);
			SP0A19_write_cmos_sensor(0xab, 0x05);
			SP0A19_write_cmos_sensor(0xac, 0x01);
			SP0A19_write_cmos_sensor(0xad, 0x07);
			SP0A19_write_cmos_sensor(0xae, 0x0e);
			SP0A19_write_cmos_sensor(0xaf, 0x00);
			SP0A19_write_cmos_sensor(0xb0, 0x00);
			SP0A19_write_cmos_sensor(0xb1, 0x08);
			SP0A19_write_cmos_sensor(0xb2, 0x02);
			SP0A19_write_cmos_sensor(0xb3, 0x00);

			SP0A19_write_cmos_sensor(0xb4, 0x30);
			SP0A19_write_cmos_sensor(0xb5, 0x0f);
			SP0A19_write_cmos_sensor(0xb6, 0x16);
			SP0A19_write_cmos_sensor(0xba, 0x44);
			SP0A19_write_cmos_sensor(0xbb, 0x24);
			SP0A19_write_cmos_sensor(0xbc, 0x2a);
			SP0A19_write_cmos_sensor(0xc0, 0x13);
			SP0A19_write_cmos_sensor(0xc1, 0x0e);
			SP0A19_write_cmos_sensor(0xc2, 0x11);
			SP0A19_write_cmos_sensor(0xc6, 0x28);
			SP0A19_write_cmos_sensor(0xc7, 0x21);
			SP0A19_write_cmos_sensor(0xc8, 0x20);

			SP0A19_write_cmos_sensor(0xb7, 0x00);
			SP0A19_write_cmos_sensor(0xb8, 0x00);
			SP0A19_write_cmos_sensor(0xb9, 0x01);
			SP0A19_write_cmos_sensor(0xbd, 0x00);
			SP0A19_write_cmos_sensor(0xbe, 0x00);
			SP0A19_write_cmos_sensor(0xbf, 0x00);
			SP0A19_write_cmos_sensor(0xc3, 0x00);
			SP0A19_write_cmos_sensor(0xc4, 0x00);
			SP0A19_write_cmos_sensor(0xc5, 0x00);
			SP0A19_write_cmos_sensor(0xc9, 0x00);
			SP0A19_write_cmos_sensor(0xca, 0x00);
			SP0A19_write_cmos_sensor(0xcb, 0x00);

			SP0A19_write_cmos_sensor(0xfe, 0x00);
			#endif
			break;
			
		case LY_982A_H114:
			#if 0
			SP0A19_write_cmos_sensor(0xfe, 0x01);
			SP0A19_write_cmos_sensor(0xa0, 0x00);
			SP0A19_write_cmos_sensor(0xa1, 0x3c);
			SP0A19_write_cmos_sensor(0xa2, 0x50);
			SP0A19_write_cmos_sensor(0xa3, 0x00);
			SP0A19_write_cmos_sensor(0xa4, 0x00);
			SP0A19_write_cmos_sensor(0xa5, 0x00);
			SP0A19_write_cmos_sensor(0xa6, 0x00);
			SP0A19_write_cmos_sensor(0xa7, 0x00);

			SP0A19_write_cmos_sensor(0xa8, 0x0c);
			SP0A19_write_cmos_sensor(0xa9, 0x06);
			SP0A19_write_cmos_sensor(0xaa, 0x02);
			SP0A19_write_cmos_sensor(0xab, 0x13);
			SP0A19_write_cmos_sensor(0xac, 0x06);
			SP0A19_write_cmos_sensor(0xad, 0x05);
			SP0A19_write_cmos_sensor(0xae, 0x0b);
			SP0A19_write_cmos_sensor(0xaf, 0x03);
			SP0A19_write_cmos_sensor(0xb0, 0x00);
			SP0A19_write_cmos_sensor(0xb1, 0x08);
			SP0A19_write_cmos_sensor(0xb2, 0x01);
			SP0A19_write_cmos_sensor(0xb3, 0x00);

			SP0A19_write_cmos_sensor(0xb4, 0x34);
			SP0A19_write_cmos_sensor(0xb5, 0x29);
			SP0A19_write_cmos_sensor(0xb6, 0x2e);
			SP0A19_write_cmos_sensor(0xba, 0x30);
			SP0A19_write_cmos_sensor(0xbb, 0x24);
			SP0A19_write_cmos_sensor(0xbc, 0x28);
			SP0A19_write_cmos_sensor(0xc0, 0x1c);
			SP0A19_write_cmos_sensor(0xc1, 0x19);
			SP0A19_write_cmos_sensor(0xc2, 0x19);
			SP0A19_write_cmos_sensor(0xc6, 0x1a);
			SP0A19_write_cmos_sensor(0xc7, 0x19);
			SP0A19_write_cmos_sensor(0xc8, 0x1b);

			SP0A19_write_cmos_sensor(0xb7, 0x01);
			SP0A19_write_cmos_sensor(0xb8, 0x01);
			SP0A19_write_cmos_sensor(0xb9, 0x00);
			SP0A19_write_cmos_sensor(0xbd, 0x00);
			SP0A19_write_cmos_sensor(0xbe, 0x00);
			SP0A19_write_cmos_sensor(0xbf, 0x00);
			SP0A19_write_cmos_sensor(0xc3, 0x00);
			SP0A19_write_cmos_sensor(0xc4, 0x00);
			SP0A19_write_cmos_sensor(0xc5, 0x03);
			SP0A19_write_cmos_sensor(0xc9, 0x00);
			SP0A19_write_cmos_sensor(0xca, 0x00);
			SP0A19_write_cmos_sensor(0xcb, 0x00);

			SP0A19_write_cmos_sensor(0xfe, 0x00);
			#endif
			break;

		case XY_046A:
			#if 0
			SP0A19_write_cmos_sensor(0xfe, 0x01);
			SP0A19_write_cmos_sensor(0xa0, 0x00);
			SP0A19_write_cmos_sensor(0xa1, 0x3c);
			SP0A19_write_cmos_sensor(0xa2, 0x50);
			SP0A19_write_cmos_sensor(0xa3, 0x00);
			SP0A19_write_cmos_sensor(0xa4, 0x00);
			SP0A19_write_cmos_sensor(0xa5, 0x00);
			SP0A19_write_cmos_sensor(0xa6, 0x10);
			SP0A19_write_cmos_sensor(0xa7, 0x00);

			SP0A19_write_cmos_sensor(0xa8, 0x11);
			SP0A19_write_cmos_sensor(0xa9, 0x0a);
			SP0A19_write_cmos_sensor(0xaa, 0x05);
			SP0A19_write_cmos_sensor(0xab, 0x04);
			SP0A19_write_cmos_sensor(0xac, 0x03);
			SP0A19_write_cmos_sensor(0xad, 0x00);
			SP0A19_write_cmos_sensor(0xae, 0x08);
			SP0A19_write_cmos_sensor(0xaf, 0x01);
			SP0A19_write_cmos_sensor(0xb0, 0x00);
			SP0A19_write_cmos_sensor(0xb1, 0x09);
			SP0A19_write_cmos_sensor(0xb2, 0x02);
			SP0A19_write_cmos_sensor(0xb3, 0x03);

			SP0A19_write_cmos_sensor(0xb4, 0x2e);
			SP0A19_write_cmos_sensor(0xb5, 0x16);
			SP0A19_write_cmos_sensor(0xb6, 0x24);
			SP0A19_write_cmos_sensor(0xba, 0x3a);
			SP0A19_write_cmos_sensor(0xbb, 0x1e);
			SP0A19_write_cmos_sensor(0xbc, 0x24);
			SP0A19_write_cmos_sensor(0xc0, 0x09);
			SP0A19_write_cmos_sensor(0xc1, 0x02);
			SP0A19_write_cmos_sensor(0xc2, 0x06);
			SP0A19_write_cmos_sensor(0xc6, 0x25);
			SP0A19_write_cmos_sensor(0xc7, 0x21);
			SP0A19_write_cmos_sensor(0xc8, 0x23);

			SP0A19_write_cmos_sensor(0xb7, 0x00);
			SP0A19_write_cmos_sensor(0xb8, 0x00);
			SP0A19_write_cmos_sensor(0xb9, 0x0f);
			SP0A19_write_cmos_sensor(0xbd, 0x00);
			SP0A19_write_cmos_sensor(0xbe, 0x00);
			SP0A19_write_cmos_sensor(0xbf, 0x00);
			SP0A19_write_cmos_sensor(0xc3, 0x00);
			SP0A19_write_cmos_sensor(0xc4, 0x00);
			SP0A19_write_cmos_sensor(0xc5, 0x00);
			SP0A19_write_cmos_sensor(0xc9, 0x00);
			SP0A19_write_cmos_sensor(0xca, 0x00);
			SP0A19_write_cmos_sensor(0xcb, 0x00);

			SP0A19_write_cmos_sensor(0xfe, 0x00);
			#endif
			break;

		case XY_0620:
			#if 0
			SP0A19_write_cmos_sensor(0xfe, 0x01);
			SP0A19_write_cmos_sensor(0xa0, 0x00);
			SP0A19_write_cmos_sensor(0xa1, 0x3c);
			SP0A19_write_cmos_sensor(0xa2, 0x50);
			SP0A19_write_cmos_sensor(0xa3, 0x00);
			SP0A19_write_cmos_sensor(0xa4, 0x00);
			SP0A19_write_cmos_sensor(0xa5, 0x00);
			SP0A19_write_cmos_sensor(0xa6, 0x00);
			SP0A19_write_cmos_sensor(0xa7, 0x00);

			SP0A19_write_cmos_sensor(0xa8, 0x0f);
			SP0A19_write_cmos_sensor(0xa9, 0x06);
			SP0A19_write_cmos_sensor(0xaa, 0x00);
			SP0A19_write_cmos_sensor(0xab, 0x07);
			SP0A19_write_cmos_sensor(0xac, 0x05);
			SP0A19_write_cmos_sensor(0xad, 0x08);
			SP0A19_write_cmos_sensor(0xae, 0x13);
			SP0A19_write_cmos_sensor(0xaf, 0x06);
			SP0A19_write_cmos_sensor(0xb0, 0x00);
			SP0A19_write_cmos_sensor(0xb1, 0x06);
			SP0A19_write_cmos_sensor(0xb2, 0x01);
			SP0A19_write_cmos_sensor(0xb3, 0x04);

			SP0A19_write_cmos_sensor(0xb4, 0x2d);
			SP0A19_write_cmos_sensor(0xb5, 0x18);
			SP0A19_write_cmos_sensor(0xb6, 0x22);
			SP0A19_write_cmos_sensor(0xba, 0x45);
			SP0A19_write_cmos_sensor(0xbb, 0x2d);
			SP0A19_write_cmos_sensor(0xbc, 0x34);
			SP0A19_write_cmos_sensor(0xc0, 0x16);
			SP0A19_write_cmos_sensor(0xc1, 0x13);
			SP0A19_write_cmos_sensor(0xc2, 0x19);
			SP0A19_write_cmos_sensor(0xc6, 0x21);
			SP0A19_write_cmos_sensor(0xc7, 0x1c);
			SP0A19_write_cmos_sensor(0xc8, 0x18);

			SP0A19_write_cmos_sensor(0xb7, 0x00);
			SP0A19_write_cmos_sensor(0xb8, 0x00);
			SP0A19_write_cmos_sensor(0xb9, 0x00);
			SP0A19_write_cmos_sensor(0xbd, 0x00);
			SP0A19_write_cmos_sensor(0xbe, 0x00);
			SP0A19_write_cmos_sensor(0xbf, 0x08);
			SP0A19_write_cmos_sensor(0xc3, 0x00);
			SP0A19_write_cmos_sensor(0xc4, 0x00);
			SP0A19_write_cmos_sensor(0xc5, 0x01);
			SP0A19_write_cmos_sensor(0xc9, 0x00);
			SP0A19_write_cmos_sensor(0xca, 0x00);
			SP0A19_write_cmos_sensor(0xcb, 0x10);

			SP0A19_write_cmos_sensor(0xfe, 0x00);
			#endif
			break;

		case XY_078V: 
			#if 0
			SP0A19_write_cmos_sensor(0xfe, 0x01);
			SP0A19_write_cmos_sensor(0xa0, 0x00);
			SP0A19_write_cmos_sensor(0xa1, 0x3c);
			SP0A19_write_cmos_sensor(0xa2, 0x50);
			SP0A19_write_cmos_sensor(0xa3, 0x00);
			SP0A19_write_cmos_sensor(0xa4, 0x00);
			SP0A19_write_cmos_sensor(0xa5, 0x00);
			SP0A19_write_cmos_sensor(0xa6, 0x00);
			SP0A19_write_cmos_sensor(0xa7, 0x00);

			SP0A19_write_cmos_sensor(0xa8, 0x14);
			SP0A19_write_cmos_sensor(0xa9, 0x08);
			SP0A19_write_cmos_sensor(0xaa, 0x0a);
			SP0A19_write_cmos_sensor(0xab, 0x11);
			SP0A19_write_cmos_sensor(0xac, 0x05);
			SP0A19_write_cmos_sensor(0xad, 0x07);
			SP0A19_write_cmos_sensor(0xae, 0x0b);
			SP0A19_write_cmos_sensor(0xaf, 0x03);
			SP0A19_write_cmos_sensor(0xb0, 0x00);
			SP0A19_write_cmos_sensor(0xb1, 0x09);
			SP0A19_write_cmos_sensor(0xb2, 0x04);
			SP0A19_write_cmos_sensor(0xb3, 0x01);

			SP0A19_write_cmos_sensor(0xb4, 0x2f);
			SP0A19_write_cmos_sensor(0xb5, 0x2a);
			SP0A19_write_cmos_sensor(0xb6, 0x2c);
			SP0A19_write_cmos_sensor(0xba, 0x3a);
			SP0A19_write_cmos_sensor(0xbb, 0x2b);
			SP0A19_write_cmos_sensor(0xbc, 0x32);
			SP0A19_write_cmos_sensor(0xc0, 0x1b);
			SP0A19_write_cmos_sensor(0xc1, 0x18);
			SP0A19_write_cmos_sensor(0xc2, 0x1a);
			SP0A19_write_cmos_sensor(0xc6, 0x12);
			SP0A19_write_cmos_sensor(0xc7, 0x10);
			SP0A19_write_cmos_sensor(0xc8, 0x12);

			SP0A19_write_cmos_sensor(0xb7, 0x0a);
			SP0A19_write_cmos_sensor(0xb8, 0x00);
			SP0A19_write_cmos_sensor(0xb9, 0x00);
			SP0A19_write_cmos_sensor(0xbd, 0x00);
			SP0A19_write_cmos_sensor(0xbe, 0x00);
			SP0A19_write_cmos_sensor(0xbf, 0x00);
			SP0A19_write_cmos_sensor(0xc3, 0x00);
			SP0A19_write_cmos_sensor(0xc4, 0x00);
			SP0A19_write_cmos_sensor(0xc5, 0x00);
			SP0A19_write_cmos_sensor(0xc9, 0x0d);
			SP0A19_write_cmos_sensor(0xca, 0x00);
			SP0A19_write_cmos_sensor(0xcb, 0x00);

			SP0A19_write_cmos_sensor(0xfe, 0x00);
			#endif
			break;

		case YG1001A_F:
			#if 0
			SP0A19_write_cmos_sensor(0xfe, 0x01);
			SP0A19_write_cmos_sensor(0xa0, 0x00);
			SP0A19_write_cmos_sensor(0xa1, 0x3c);
			SP0A19_write_cmos_sensor(0xa2, 0x50);
			SP0A19_write_cmos_sensor(0xa3, 0x00);
			SP0A19_write_cmos_sensor(0xa4, 0x00);
			SP0A19_write_cmos_sensor(0xa5, 0x00);
			SP0A19_write_cmos_sensor(0xa6, 0x00);
			SP0A19_write_cmos_sensor(0xa7, 0x00);

			SP0A19_write_cmos_sensor(0xa8, 0x0e);
			SP0A19_write_cmos_sensor(0xa9, 0x05);
			SP0A19_write_cmos_sensor(0xaa, 0x01);
			SP0A19_write_cmos_sensor(0xab, 0x07);
			SP0A19_write_cmos_sensor(0xac, 0x00);
			SP0A19_write_cmos_sensor(0xad, 0x07);
			SP0A19_write_cmos_sensor(0xae, 0x0e);
			SP0A19_write_cmos_sensor(0xaf, 0x02);
			SP0A19_write_cmos_sensor(0xb0, 0x00);
			SP0A19_write_cmos_sensor(0xb1, 0x0d);
			SP0A19_write_cmos_sensor(0xb2, 0x00);
			SP0A19_write_cmos_sensor(0xb3, 0x00);

			SP0A19_write_cmos_sensor(0xb4, 0x2a);
			SP0A19_write_cmos_sensor(0xb5, 0x0f);
			SP0A19_write_cmos_sensor(0xb6, 0x14);
			SP0A19_write_cmos_sensor(0xba, 0x40);
			SP0A19_write_cmos_sensor(0xbb, 0x26);
			SP0A19_write_cmos_sensor(0xbc, 0x2a);
			SP0A19_write_cmos_sensor(0xc0, 0x0e);
			SP0A19_write_cmos_sensor(0xc1, 0x0a);
			SP0A19_write_cmos_sensor(0xc2, 0x0d);
			SP0A19_write_cmos_sensor(0xc6, 0x27);
			SP0A19_write_cmos_sensor(0xc7, 0x20);
			SP0A19_write_cmos_sensor(0xc8, 0x1f);

			SP0A19_write_cmos_sensor(0xb7, 0x00);
			SP0A19_write_cmos_sensor(0xb8, 0x00);
			SP0A19_write_cmos_sensor(0xb9, 0x00);
			SP0A19_write_cmos_sensor(0xbd, 0x00);
			SP0A19_write_cmos_sensor(0xbe, 0x00);
			SP0A19_write_cmos_sensor(0xbf, 0x00);
			SP0A19_write_cmos_sensor(0xc3, 0x00);
			SP0A19_write_cmos_sensor(0xc4, 0x00);
			SP0A19_write_cmos_sensor(0xc5, 0x00);
			SP0A19_write_cmos_sensor(0xc9, 0x00);
			SP0A19_write_cmos_sensor(0xca, 0x00);
			SP0A19_write_cmos_sensor(0xcb, 0x00);

			SP0A19_write_cmos_sensor(0xfe, 0x00);
			#endif
			break;

		default:
			break;
	}
}


/*************************************************************************
* FUNCTION
*	SP0A19_GAMMA_Select
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
UINT32 SP0A19GetSensorID(UINT32 *sensorID)
{
    kal_uint16 sensor_id=0;
    int retry=3;

     SENSORDB("SP0A19GetSensorID \n");	

       // check if sensor ID correct
	do {
		
		SP0A19_write_cmos_sensor(0xfd,0x00);
	    sensor_id=SP0A19_read_cmos_sensor(0x02);
         SENSORDB("Read Sensor ID arthur = 0x%x\n", sensor_id); 
    	    if (sensor_id == SP0A19_SENSOR_ID) {
                 break; 
    	    }
         SENSORDB("Read Sensor ID Fail = 0x%x\n", sensor_id); 
    	    
    	    retry--; 
	}while (retry > 0); 
	
	if (sensor_id != SP0A19_SENSOR_ID) {
		
		*sensorID = 0xFFFFFFFF;
	    return ERROR_SENSOR_CONNECT_FAIL;
	}

      *sensorID = sensor_id;
       RETAILMSG(1, (TEXT("Sensor Read ID OK \r\n")));
	
    return ERROR_NONE;
}

void SP0A19GammaSelect(kal_uint32 GammaLvl)
{

	
}



/*************************************************************************
* FUNCTION
*	SP0A19_Write_More_Registers
*
* DESCRIPTION
*	This function is served for FAE to modify the necessary Init Regs. Do not modify the regs
*     in init_SP0A19() directly.
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
void SP0A19_Write_More_Registers(void)
{
	////////////20120427/////////////////////////
	
}


/*************************************************************************
 * FUNCTION
 *	SP0A19Open
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
UINT32 SP0A19Open(void)
{
    kal_uint16 sensor_id=0;
    int retry = 3; 

     SENSORDB("SP0A19Open \n");

     do {
		
		SP0A19_write_cmos_sensor(0xfd,0x00);
	    sensor_id=SP0A19_read_cmos_sensor(0x02);
    	    if (sensor_id == SP0A19_SENSOR_ID) {
                 break; 
    	    }
         SENSORDB("Read Sensor ID Fail = 0x%x\n", sensor_id); 
    	    
    	    retry--; 
	}while (retry > 0); 
	
	if (sensor_id != SP0A19_SENSOR_ID) 
	   {
	      return ERROR_SENSOR_CONNECT_FAIL;
	   }
         SENSORDB("SP0A19 Sensor id read OK, ID = %x\n", sensor_id);

     Sleep(10);
 #ifdef DEBUG_SENSOR_SP0A19  //gepeiwei   120903
	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static char buf[10*1024] ;
	fp = filp_open("/mnt/sdcard/SP0A19_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) { 
		SP0A19_fromsd = 0;   
		printk("open file error\n");
	} 
	else 
	{
		SP0A19_fromsd = 1;
		printk("open file ok\n");
		filp_close(fp, NULL); 
		set_fs(fs);
	}
	if(SP0A19_fromsd == 1)//是否从SD读取//gepeiwei   120903
	{
		printk("________________from t!\n");
		SP0A19_Initialize_from_T_Flash();//从SD卡读取的主要函数
	}
	else
	{
    SP0A19_Sensor_Init();		
		SP0A19_Write_More_Registers();//added for FAE to debut
	}
#else  
	SP0A19_Sensor_Init();
	SP0A19_Write_More_Registers();//added for FAE to debut
#endif
    SENSORDB("SP0A19Open end \n");
    return ERROR_NONE;
} /* SP0A19Open */


/*************************************************************************
 * FUNCTION
 *	SP0A19Close
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
UINT32 SP0A19Close(void)
{
     SENSORDB("SP0A19Close\n");
    return ERROR_NONE;
} /* SP0A19Close */


/*************************************************************************
 * FUNCTION
 * SP0A19Preview
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
UINT32 SP0A19Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    kal_uint32 iTemp;
    kal_uint16 iStartX = 0, iStartY = 1;

    if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        RETAILMSG(1, (TEXT("Camera Video preview\r\n")));
        SP0A19_MPEG4_encode_mode = KAL_TRUE;
       
    }
    else
    {
        RETAILMSG(1, (TEXT("Camera preview\r\n")));
        SP0A19_MPEG4_encode_mode = KAL_FALSE;
    }

    image_window->GrabStartX= IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY= IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;

    // copy sensor_config_data
    memcpy(&SP0A19SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* SP0A19Preview */


/*************************************************************************
 * FUNCTION
 *	SP0A19Capture
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
UINT32 SP0A19Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    SP0A19_MODE_CAPTURE=KAL_TRUE;

    image_window->GrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth= IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = IMAGE_SENSOR_FULL_HEIGHT;

    // copy sensor_config_data
    memcpy(&SP0A19SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* SP0A19_Capture() */



UINT32 SP0A19GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth=IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorVideoHeight=IMAGE_SENSOR_FULL_HEIGHT;
    return ERROR_NONE;
} /* SP0A19GetResolution() */


UINT32 SP0A19GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_WIDTH;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=1;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;
#if 0
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=FALSE;
#endif
    pSensorInfo->CaptureDelayFrame = 1;
    pSensorInfo->PreviewDelayFrame = 0;
    pSensorInfo->VideoDelayFrame = 4;
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
//    case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=	3;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;

        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
  //  case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
        break;
    default:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
        break;
    }
    SP0A19PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &SP0A19SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* SP0A19GetInfo() */


UINT32 SP0A19Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
//    case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        SP0A19Preview(pImageWindow, pSensorConfigData);
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
//    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        SP0A19Capture(pImageWindow, pSensorConfigData);
        break;
    }


    return TRUE;
}	/* SP0A19Control() */

BOOL SP0A19_set_param_wb(UINT16 para)
{
	#if 1//lj_test
	SP0A19_write_cmos_sensor(0xfd, 0x00);
	SP0A19_write_cmos_sensor(0x36, 0x20);
	SP0A19_write_cmos_sensor(0xe7, 0x03);
	SP0A19_write_cmos_sensor(0xe7, 0x00);	
	#endif
	
	switch (para)
	{            
		case AWB_MODE_AUTO:
			{
		        //SP0A19_reg_WB_auto         自动
			SP0A19_write_cmos_sensor(0xfd,0x01);                                                          
			SP0A19_write_cmos_sensor(0x28,0xc4);		                                                       
			SP0A19_write_cmos_sensor(0x29,0x9e);			
			SP0A19_write_cmos_sensor(0xfd,0x00);
			SP0A19_write_cmos_sensor(0xe7, 0x03);
			SP0A19_write_cmos_sensor(0xe7, 0x00);			
			SP0A19_write_cmos_sensor(0xfd,0x00);  // AUTO 3000K~7000K   	  
			SP0A19_write_cmos_sensor(0x32,0x15); //0x0d			
			SP0A19_write_cmos_sensor(0xe7, 0x03);
			SP0A19_write_cmos_sensor(0xe7, 0x00);
            }                
		    break;
		case AWB_MODE_CLOUDY_DAYLIGHT:
			{
		       // SP0A19_reg_WB_auto   阴天
			SP0A19_write_cmos_sensor(0xfd,0x00);   //7000K                                     
			SP0A19_write_cmos_sensor(0x32,0x05);                                                          
			SP0A19_write_cmos_sensor(0xfd,0x01);                                                          
			SP0A19_write_cmos_sensor(0x28,0xbf);		                                                       
			SP0A19_write_cmos_sensor(0x29,0x89);		                                                       
			SP0A19_write_cmos_sensor(0xfd,0x00); 
			SP0A19_write_cmos_sensor(0xe7, 0x03);
			SP0A19_write_cmos_sensor(0xe7, 0x00);			
	        }			   
		    break;
		case AWB_MODE_DAYLIGHT:
		    {
	           // SP0A19_reg_WB_auto  白天 
			SP0A19_write_cmos_sensor(0xfd,0x00);  //6500K                                     
			SP0A19_write_cmos_sensor(0x32,0x05);                                                          
			SP0A19_write_cmos_sensor(0xfd,0x01);                                                          
			SP0A19_write_cmos_sensor(0x28,0xbc);		                                                       
			SP0A19_write_cmos_sensor(0x29,0x5d);		                                                       
			SP0A19_write_cmos_sensor(0xfd,0x00); 
			SP0A19_write_cmos_sensor(0xe7, 0x03);
			SP0A19_write_cmos_sensor(0xe7, 0x00);
            }      
		    break;
		case AWB_MODE_INCANDESCENT:	
		    {
		       // SP0A19_reg_WB_auto 白炽灯 
			SP0A19_write_cmos_sensor(0xfd,0x00);  //2800K~3000K                                     
			SP0A19_write_cmos_sensor(0x32,0x05);                                                          
			SP0A19_write_cmos_sensor(0xfd,0x01);                                                          
			SP0A19_write_cmos_sensor(0x28,0x89);		                                                       
			SP0A19_write_cmos_sensor(0x29,0xb8);		                                                       
			SP0A19_write_cmos_sensor(0xfd,0x00); 
			SP0A19_write_cmos_sensor(0xe7, 0x03);
			SP0A19_write_cmos_sensor(0xe7, 0x00);

            }		
		    break;  
		case AWB_MODE_FLUORESCENT:
		    {
	           //SP0A19_reg_WB_auto  荧光灯 
			SP0A19_write_cmos_sensor(0xfd,0x00);  //4200K~5000K                                     
			SP0A19_write_cmos_sensor(0x32,0x05);                                                          
			SP0A19_write_cmos_sensor(0xfd,0x01);                                                          
			SP0A19_write_cmos_sensor(0x28,0xb5);		                                                       
			SP0A19_write_cmos_sensor(0x29,0xa5);		                                                       
			SP0A19_write_cmos_sensor(0xfd,0x00);
			SP0A19_write_cmos_sensor(0xe7, 0x03);
			SP0A19_write_cmos_sensor(0xe7, 0x00);			
            }	
		    break;  
		case AWB_MODE_TUNGSTEN:
		   {
	           // SP0A19_reg_WB_auto 白热光
			SP0A19_write_cmos_sensor(0xfd,0x00);  //4000K                                   
			SP0A19_write_cmos_sensor(0x32,0x05);                                                          
			SP0A19_write_cmos_sensor(0xfd,0x01);                                                          
			SP0A19_write_cmos_sensor(0x28,0xaf);		                                                       
			SP0A19_write_cmos_sensor(0x29,0x99);		                                                       
			SP0A19_write_cmos_sensor(0xfd,0x00);  
			SP0A19_write_cmos_sensor(0xe7, 0x03);
			SP0A19_write_cmos_sensor(0xe7, 0x00);			
           }
		    break;

		default:
			break;//return FALSE;
	}

	#if 1//lj_test
	SP0A19_write_cmos_sensor(0xe7, 0x03);
	SP0A19_write_cmos_sensor(0xe7, 0x00);	
	Sleep(50);
	SP0A19_write_cmos_sensor(0xfd, 0x00);
	SP0A19_write_cmos_sensor(0x36, 0x00);
	SP0A19_write_cmos_sensor(0xe7, 0x03);
	SP0A19_write_cmos_sensor(0xe7, 0x00);	
	#endif

	return TRUE;
	
} /* SP0A19_set_param_wb */


BOOL SP0A19_set_param_effect(UINT16 para)
{
//	kal_uint32  ret = KAL_TRUE;

	#if 1 //lj_test
	SP0A19_write_cmos_sensor(0xfd, 0x00);
	SP0A19_write_cmos_sensor(0x36, 0x20);
	SP0A19_write_cmos_sensor(0xe7, 0x03);
	SP0A19_write_cmos_sensor(0xe7, 0x00);	
	#endif
	
	switch (para)
	{
		case MEFFECT_OFF:
			{
	        SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0x62, 0x00);
			SP0A19_write_cmos_sensor(0x63, 0x80);
			SP0A19_write_cmos_sensor(0x64, 0x80);
            }
	        break;
		case MEFFECT_SEPIA:
			{
	            SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0x62, 0x10);
			SP0A19_write_cmos_sensor(0x63, 0xc0);
			SP0A19_write_cmos_sensor(0x64, 0x20);

            }	
			break;  
		case MEFFECT_NEGATIVE:
			{
	            SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0x62, 0x04);
			SP0A19_write_cmos_sensor(0x63, 0x80);
			SP0A19_write_cmos_sensor(0x64, 0x80);
            }
			break; 
		case MEFFECT_SEPIAGREEN:		
			{
	         SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0x62, 0x10);
			SP0A19_write_cmos_sensor(0x63, 0x20);
			SP0A19_write_cmos_sensor(0x64, 0x20);
            }	
			break;
		case MEFFECT_SEPIABLUE:
			{
			  	SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0x62, 0x10);
			SP0A19_write_cmos_sensor(0x63, 0x20);
			SP0A19_write_cmos_sensor(0x64, 0xf0);
		    }     
			break;        
		case MEFFECT_MONO:			
			{
				SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0x62, 0x20);
			SP0A19_write_cmos_sensor(0x63, 0x80);
			SP0A19_write_cmos_sensor(0x64, 0x80);
            }
			break;

		default:
			break;//return KAL_FALSE;
	}
	
	#if 1 //lj_test
	SP0A19_write_cmos_sensor(0xe7, 0x03);
	SP0A19_write_cmos_sensor(0xe7, 0x00);	
	Sleep(100);
	SP0A19_write_cmos_sensor(0xfd, 0x00);
	SP0A19_write_cmos_sensor(0x36, 0x00);
	SP0A19_write_cmos_sensor(0xe7, 0x03);
	SP0A19_write_cmos_sensor(0xe7, 0x00);	
	#endif
	
	return KAL_TRUE;

} /* SP0A19_set_param_effect */


BOOL SP0A19_set_param_banding(UINT16 para)
{
	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			sp0a19_isBanding = 0;
			break;

		case AE_FLICKER_MODE_60HZ:
			sp0a19_isBanding = 1;
		break;
		default:
		return FALSE;
	}
	 SP0A19NightMode(KAL_FALSE);

	return TRUE;
} /* SP0A19_set_param_banding */


BOOL SP0A19_set_param_exposure(UINT16 para)
{

	switch (para)
	{
		case AE_EV_COMP_13:  //+4 EV
			SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0xdc, 0x40);
			break;  
		case AE_EV_COMP_10:  //+3 EV
			SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0xdc, 0x30);
			break;    
		case AE_EV_COMP_07:  //+2 EV
			SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0xdc, 0x20);
			break;    
		case AE_EV_COMP_03:	 //	+1 EV	
			SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0xdc, 0x10);
			break;    
		case AE_EV_COMP_00:  // +0 EV
		    SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0xdc, 0x00);//0xfa before
			break;    
		case AE_EV_COMP_n03:  // -1 EV
			SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0xdc, 0xf0);
			break;    
		case AE_EV_COMP_n07:	// -2 EV		
			SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0xdc, 0xe0);
			break;    
		case AE_EV_COMP_n10:   //-3 EV
			SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0xdc, 0xd0);
			break;
		case AE_EV_COMP_n13:  // -4 EV
			SP0A19_write_cmos_sensor(0xfd, 0x00);
			SP0A19_write_cmos_sensor(0xdc, 0xc0);
			break;
		default:
			return FALSE;
	}

	return TRUE;
	
} /* SP0A19_set_param_exposure */


UINT32 SP0A19YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
#ifdef DEBUG_SENSOR_SP0A19
	return TRUE;
#endif
    switch (iCmd) {
    case FID_AWB_MODE:
        SP0A19_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        SP0A19_set_param_effect(iPara);
        break;
    case FID_AE_EV:
        SP0A19_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:
        SP0A19_set_param_banding(iPara);
		break;
	case FID_SCENE_MODE:
		SP0A19NightMode(iPara);
        break;
    default:
        break;
    }
    return TRUE;
} /* SP0A19YUVSensorSetting */


UINT32 SP0A19FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
        UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SP0A19SensorRegNumber;
    UINT32 i;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

    RETAILMSG(1, (_T("gaiyang SP0A19FeatureControl FeatureId=%d\r\n"), FeatureId));

    switch (FeatureId)
    {
    case SENSOR_FEATURE_GET_RESOLUTION:
        *pFeatureReturnPara16++=IMAGE_SENSOR_FULL_WIDTH;
        *pFeatureReturnPara16=IMAGE_SENSOR_FULL_HEIGHT;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *pFeatureReturnPara16++=(VGA_PERIOD_PIXEL_NUMS)+SP0A19_dummy_pixels;
        *pFeatureReturnPara16=(VGA_PERIOD_LINE_NUMS)+SP0A19_dummy_lines;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        *pFeatureReturnPara32 = SP0A19_g_fPV_PCLK;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:
        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
		#ifndef DEBUG_SENSOR_SP0A19		
        SP0A19NightMode((BOOL) *pFeatureData16);
		#endif
        break;
    case SENSOR_FEATURE_SET_GAIN:
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        SP0A19_isp_master_clock=*pFeatureData32;
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        SP0A19_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        pSensorRegData->RegData = SP0A19_read_cmos_sensor(pSensorRegData->RegAddr);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
        memcpy(pSensorConfigData, &SP0A19SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
        *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
        break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_GET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_GET_GROUP_COUNT:
    case SENSOR_FEATURE_GET_GROUP_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
        break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
        // if EEPROM does not exist in camera module.
        *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_YUV_CMD:
        SP0A19YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
        break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:


	SP0A19GetSensorID(pFeatureData32);
	break;
    default:
        break;
	}
return ERROR_NONE;
}	/* SP0A19FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncSP0A19YUV=
{
	SP0A19Open,
	SP0A19GetInfo,
	SP0A19GetResolution,
	SP0A19FeatureControl,
	SP0A19Control,
	SP0A19Close
};


UINT32 SP0A19_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncSP0A19YUV;
	return ERROR_NONE;
} /* SensorInit() */
