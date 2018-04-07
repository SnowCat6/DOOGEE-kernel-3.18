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
 *   gc0310yuv_Sensor.c
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

#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "sp0a20_yuv_Sensor.h"

#define SP0A20YUV_DEBUG

#ifdef SP0A20YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif
kal_bool   SP0A20_TST_PATTEN = KAL_FALSE;  //wxl
//#define DEBUG_SENSOR_SP0A20//T_flash Tuning
#define SP0A20_TEST_PATTERN_CHECKSUM (0xaf010c1c)

#ifdef SLT_DEVINFO_CMM 
#include  <linux/dev_info.h>
static struct devinfo_struct *s_DEVINFO_ccm;   //suppose 10 max lcm device 
#endif

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
static struct SP0A20_Sensor_Struct SP0A20_Sensor_Driver;
kal_uint8 isBanding = 0; // 0: 50hz  1:60hz

kal_uint16 SP0A20_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 2, SP0A20_WRITE_ID);
        return 0;

}

kal_uint16 SP0A20_read_cmos_sensor(kal_uint8 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, SP0A20_WRITE_ID);
	
    return get_byte;
}


#ifdef DEBUG_SENSOR_SP0A20
#define SP0A20_OP_CODE_INI		0x00		/* Initial value. */
#define SP0A20_OP_CODE_REG		0x01		/* Register */
#define SP0A20_OP_CODE_DLY		0x02		/* Delay */
#define SP0A20_OP_CODE_END	0x03		/* End of initial setting. */
	kal_uint16 fromsd;

typedef struct
{
	u16 init_reg;
	u16 init_val;	/* Save the register value and delay tick */
	u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
} SP0A20_initial_set_struct;

SP0A20_initial_set_struct SP0A20_Init_Reg[5000];

static u32 strtol(const char *nptr, u8 base)
{

	printk("SP0A20___%s____\n",__func__); 

	u8 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("SP0A20 %s(): NULL pointer input\n", __FUNCTION__);
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

static u8 SP0A20_Initialize_from_T_Flash()
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0, j = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */


	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static u8 data_buff[10*1024] ;

    fp = filp_open("/mnt/sdcard/sp0a20_sd", O_RDONLY , 0); 
    if (IS_ERR(fp)) { 
        printk("create file error\n");  
		return -1; 
	} 
	fs = get_fs(); 
	set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	//printk("%s %d %d\n", buf,iFileLen,pos); 
	filp_close(fp, NULL); 
	set_fs(fs);

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
			SP0A20_Init_Reg[i].op_code = SP0A20_OP_CODE_REG;

			SP0A20_Init_Reg[i].init_reg = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */

			SP0A20_Init_Reg[i].init_val = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */

		}
		else									/* DLY */
		{
			/* Need add delay for this setting. */ 
			curr_ptr += 4;	
			SP0A20_Init_Reg[i].op_code = SP0A20_OP_CODE_DLY;

			SP0A20_Init_Reg[i].init_reg = 0xFF;
			SP0A20_Init_Reg[i].init_val = strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
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
	SP0A20_Init_Reg[i].op_code = SP0A20_OP_CODE_END;
	SP0A20_Init_Reg[i].init_reg = 0xFF;
	SP0A20_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
		//printk(" %x  ==  %x\n",SP0A20_Init_Reg[j].init_reg, SP0A20_Init_Reg[j].init_val);

	/* Start apply the initial setting to sensor. */
#if 1
	for (j=0; j<i; j++)
	{
		if (SP0A20_Init_Reg[j].op_code == SP0A20_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (SP0A20_Init_Reg[j].op_code == SP0A20_OP_CODE_DLY)
		{
			msleep(SP0A20_Init_Reg[j].init_val);		/* Delay */
		}
		else if (SP0A20_Init_Reg[j].op_code == SP0A20_OP_CODE_REG)
		{
			SP0A20_write_cmos_sensor(SP0A20_Init_Reg[j].init_reg, SP0A20_Init_Reg[j].init_val);
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

/*******************************************************************************
 * // Adapter for Winmo typedef
 ********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

kal_bool   SP0A20_MPEG4_encode_mode = KAL_FALSE;
kal_uint16 SP0A20_dummy_pixels = 0, SP0A20_dummy_lines = 0;
kal_bool   SP0A20_MODE_CAPTURE = KAL_FALSE;
kal_bool   SP0A20_NIGHT_MODE = KAL_FALSE;

kal_uint32 SP0A20_isp_master_clock;
static kal_uint32 SP0A20_g_fPV_PCLK = 24;

kal_uint8 SP0A20_sensor_write_I2C_address = SP0A20_WRITE_ID;
kal_uint8 SP0A20_sensor_read_I2C_address = SP0A20_READ_ID;

UINT8 SP0A20PixelClockDivider=0;

MSDK_SENSOR_CONFIG_STRUCT SP0A20SensorConfigData;

/*************************************************************************
 * FUNCTION
 *	SP0A20_SetShutter
 *
 * DESCRIPTION
 *	This function set e-shutter of SP0A20 to change exposure time.
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
void SP0A20_Set_Shutter(kal_uint16 iShutter)
{
} /* Set_SP0A20_Shutter */


/*************************************************************************
 * FUNCTION
 *	SP0A20_read_Shutter
 *
 * DESCRIPTION
 *	This function read e-shutter of SP0A20 .
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
kal_uint16 SP0A20_Read_Shutter(void)
{
    	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 shutter;

	temp_reg1 = SP0A20_read_cmos_sensor(0x04);
	temp_reg2 = SP0A20_read_cmos_sensor(0x03);

	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

	return shutter;
} /* SP0A20_read_shutter */


/*************************************************************************
 * FUNCTION
 *	SP0A20_write_reg
 *
 * DESCRIPTION
 *	This function set the register of SP0A20.
 *
 * PARAMETERS
 *	addr : the register index of SP0A20
 *  para : setting parameter of the specified register of SP0A20
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void SP0A20_write_reg(kal_uint32 addr, kal_uint32 para)
{
	SP0A20_write_cmos_sensor(addr, para);
} /* SP0A20_write_reg() */


/*************************************************************************
 * FUNCTION
 *	SP0A20_read_cmos_sensor
 *
 * DESCRIPTION
 *	This function read parameter of specified register from SP0A20.
 *
 * PARAMETERS
 *	addr : the register index of SP0A20
 *
 * RETURNS
 *	the data that read from SP0A20
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint32 SP0A20_read_reg(kal_uint32 addr)
{
	return SP0A20_read_cmos_sensor(addr);
} /* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	SP0A20_awb_enable
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
static void SP0A20_awb_enable(kal_bool enalbe)
{	 
	kal_uint16 temp_AWB_reg = 0;

	temp_AWB_reg = SP0A20_read_cmos_sensor(0x42);
	
	if (enalbe)
	{
		SP0A20_write_cmos_sensor(0x42, (temp_AWB_reg |0x02));
	}
	else
	{
		SP0A20_write_cmos_sensor(0x42, (temp_AWB_reg & (~0x02)));
	}

}


/*************************************************************************
 * FUNCTION
 *	SP0A20_config_window
 *
 * DESCRIPTION
 *	This function config the hardware window of SP0A20 for getting specified
 *  data of that window.
 *
 * PARAMETERS
 *	start_x : start column of the interested window
 *  start_y : start row of the interested window
 *  width  : column widht of the itnerested window
 *  height : row depth of the itnerested window
 *
 * RETURNS
 *	the data that read from SP0A20
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void SP0A20_config_window(kal_uint16 startx, kal_uint16 starty, kal_uint16 width, kal_uint16 height)
{
} /* SP0A20_config_window */


/*************************************************************************
 * FUNCTION
 *	SP0A20_SetGain
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
kal_uint16 SP0A20_SetGain(kal_uint16 iGain)
{
	return iGain;
}


/*************************************************************************
 * FUNCTION
 *	SP0A20_NightMode
 *
 * DESCRIPTION
 *	This function night mode of SP0A20.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED   ///070312101020  
 *
 *************************************************************************/
static void SP0A20_night_mode(kal_bool bEnable)
{
    if(SP0A20_TST_PATTEN==KAL_TRUE)
		return;
	if (!SP0A20_Sensor_Driver.MODE_CAPTURE) { 
		if(bEnable)//night mode
		{ 
			SP0A20_Sensor_Driver.bNight_mode = KAL_TRUE;
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0xcd,0x20);
			SP0A20_write_cmos_sensor(0xce,0x1f);

			if(SP0A20_Sensor_Driver.MPEG4_encode_mode == KAL_TRUE)
			{
				if(isBanding== 0)
				{
					printk("video 50Hz night\n");	
#if 0
					//Video record night 24M 50hz 10.02-10fps maxgain				                     
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x04 , 0x2c);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x06);
					SP0A20_write_cmos_sensor(0x0a , 0x14);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x32);
					SP0A20_write_cmos_sensor(0x02 , 0x0a);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x32);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0xf4);
					SP0A20_write_cmos_sensor(0xbf , 0x01);
					SP0A20_write_cmos_sensor(0xd0 , 0xf4);
					SP0A20_write_cmos_sensor(0xd1 , 0x01);
#else//Video record night 24M 50hz 10-10fps maxgain	
					SP0A20_write_cmos_sensor(0xfd,0x00);
					SP0A20_write_cmos_sensor(0x03,0x01);
					SP0A20_write_cmos_sensor(0x04,0x2c);
					SP0A20_write_cmos_sensor(0x05,0x00);
					SP0A20_write_cmos_sensor(0x06,0x00);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x00);
					SP0A20_write_cmos_sensor(0x09,0x06);
					SP0A20_write_cmos_sensor(0x0a,0x14);
					SP0A20_write_cmos_sensor(0xfd,0x01);
					SP0A20_write_cmos_sensor(0xf0,0x00);
					SP0A20_write_cmos_sensor(0xf7,0x32);
					SP0A20_write_cmos_sensor(0x02,0x0a);
					SP0A20_write_cmos_sensor(0x03,0x01);
					SP0A20_write_cmos_sensor(0x06,0x32);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x01);
					SP0A20_write_cmos_sensor(0x09,0x00);
					SP0A20_write_cmos_sensor(0xfd,0x02);
					SP0A20_write_cmos_sensor(0xbe,0xf4);
					SP0A20_write_cmos_sensor(0xbf,0x01);
					SP0A20_write_cmos_sensor(0xd0,0xf4);
					SP0A20_write_cmos_sensor(0xd1,0x01);
#endif

					//dbg_print(" video 50Hz night\r\n");
				}
				else if(isBanding == 1)
				{
					//Video record night 24M 60Hz 10-10FPS maxgain:
					SP0A20_write_cmos_sensor(0xfd,0x00);
					SP0A20_write_cmos_sensor(0x03,0x00);
					SP0A20_write_cmos_sensor(0x04,0xfc);
					SP0A20_write_cmos_sensor(0x05,0x00);
					SP0A20_write_cmos_sensor(0x06,0x00);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x00);
					SP0A20_write_cmos_sensor(0x09,0x06);
					SP0A20_write_cmos_sensor(0x0a,0x01);
					SP0A20_write_cmos_sensor(0xfd,0x01);
					SP0A20_write_cmos_sensor(0xf0,0x00);
					SP0A20_write_cmos_sensor(0xf7,0x2a);
					SP0A20_write_cmos_sensor(0x02,0x0c);
					SP0A20_write_cmos_sensor(0x03,0x01);
					SP0A20_write_cmos_sensor(0x06,0x2a);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x01);
					SP0A20_write_cmos_sensor(0x09,0x00);
					SP0A20_write_cmos_sensor(0xfd,0x02);
					SP0A20_write_cmos_sensor(0xbe,0xf8);
					SP0A20_write_cmos_sensor(0xbf,0x01);
					SP0A20_write_cmos_sensor(0xd0,0xf8);
					SP0A20_write_cmos_sensor(0xd1,0x01);


					printk(" video 60Hz night\r\n");
				}
			}	
			else 
			{
				//	dbg_print(" SP0A20_banding=%x\r\n",SP0A20_banding);
				if(isBanding== 0)
				{
					//capture preview night 24M 50hz 8-12fps maxgain:	 
						SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x04 , 0x68);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x04);
					SP0A20_write_cmos_sensor(0x0a , 0x84);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x3c);
					SP0A20_write_cmos_sensor(0x02 , 0x0c);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x3c);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0xd0);
					SP0A20_write_cmos_sensor(0xbf , 0x02);
					SP0A20_write_cmos_sensor(0xd0 , 0xd0);
					SP0A20_write_cmos_sensor(0xd1 , 0x02);

					printk(" priview 50Hz night\r\n");	
				}  
				else if(isBanding== 1)
				{
					//capture preview night 24M 60hz 8-12FPS maxgain:
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x04 , 0x2c);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x04);
					SP0A20_write_cmos_sensor(0x0a , 0x84);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x32);
					SP0A20_write_cmos_sensor(0x02 , 0x0f);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x32);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0xee);
					SP0A20_write_cmos_sensor(0xbf , 0x02);
					SP0A20_write_cmos_sensor(0xd0 , 0xee);
					SP0A20_write_cmos_sensor(0xd1 , 0x02);

					printk(" priview 60Hz night\r\n");	
				}
			} 		
		}
		else    // daylight mode
		{
			SP0A20_Sensor_Driver.bNight_mode = KAL_FALSE;
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0xcd,0x10);
			SP0A20_write_cmos_sensor(0xce,0x1f);  
			if(SP0A20_Sensor_Driver.MPEG4_encode_mode == KAL_TRUE)
			{
				//dbg_print(" SP0A20_banding=%x\r\n",SP0A20_banding);
				if(isBanding== 0)
				{
					//Video record daylight 24M 50hz 10-10FPS maxgain:                     
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x04 , 0x2c);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x06);
					SP0A20_write_cmos_sensor(0x0a , 0x14);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x32);
					SP0A20_write_cmos_sensor(0x02 , 0x0a);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x32);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0xf4);
					SP0A20_write_cmos_sensor(0xbf , 0x01);
					SP0A20_write_cmos_sensor(0xd0 , 0xf4);
					SP0A20_write_cmos_sensor(0xd1 , 0x01);

					printk(" video 50Hz normal\r\n");				
				}
				else if(isBanding == 1)
				{
					//Video record daylight 24M 60Hz 10-10FPS maxgain:
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x00);
					SP0A20_write_cmos_sensor(0x04 , 0xfc);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x06);
					SP0A20_write_cmos_sensor(0x0a , 0x01);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x2a);
					SP0A20_write_cmos_sensor(0x02 , 0x0c);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x2a);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0xf8);
					SP0A20_write_cmos_sensor(0xbf , 0x01);
					SP0A20_write_cmos_sensor(0xd0 , 0xf8);
					SP0A20_write_cmos_sensor(0xd1 , 0x01);
					printk(" video 60Hz normal\r\n");	
				}
			}
			else 
			{
				if(isBanding== 0)
				{	
					//capture preview daylight 24M 50hz 12-8FPS maxgain:   
					SP0A20_write_cmos_sensor(0xfd,0x00);
					SP0A20_write_cmos_sensor(0x03,0x01);
					SP0A20_write_cmos_sensor(0x04,0x68);
					SP0A20_write_cmos_sensor(0x05,0x00);
					SP0A20_write_cmos_sensor(0x06,0x00);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x00);
					SP0A20_write_cmos_sensor(0x09,0x04);
					SP0A20_write_cmos_sensor(0x0a,0x84);
					SP0A20_write_cmos_sensor(0xfd,0x01);
					SP0A20_write_cmos_sensor(0xf0,0x00);
					SP0A20_write_cmos_sensor(0xf7,0x3c);
					SP0A20_write_cmos_sensor(0x02,0x0c);
					SP0A20_write_cmos_sensor(0x03,0x01);
					SP0A20_write_cmos_sensor(0x06,0x3c);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x01);
					SP0A20_write_cmos_sensor(0x09,0x00);
					SP0A20_write_cmos_sensor(0xfd,0x02);
					SP0A20_write_cmos_sensor(0xbe,0xd0);
					SP0A20_write_cmos_sensor(0xbf,0x02);
					SP0A20_write_cmos_sensor(0xd0,0xd0);
					SP0A20_write_cmos_sensor(0xd1,0x02);
					
					printk(" priview 50Hz normal\r\n");
				}
				else if(isBanding== 1)
				{
					//capture preview daylight 24M 60hz 8-12FPS maxgain:
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x04 , 0x2c);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x04);
					SP0A20_write_cmos_sensor(0x0a , 0x84);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x32);
					SP0A20_write_cmos_sensor(0x02 , 0x0f);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x32);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0xee);
					SP0A20_write_cmos_sensor(0xbf , 0x02);
					SP0A20_write_cmos_sensor(0xd0 , 0xee);
					SP0A20_write_cmos_sensor(0xd1 , 0x02);

					printk(" priview 60Hz normal\r\n");
				}
			}

		}  
	}
}	/*	SP0A20_NightMode	*/

/*************************************************************************
* FUNCTION
*	SP0A20_Sensor_Init
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
void SP0A20_Sensor_Init(void)
 
{
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x36,0x02);
	SP0A20_write_cmos_sensor(0xfd,0x00);
	SP0A20_write_cmos_sensor(0x0c,0x00);
	SP0A20_write_cmos_sensor(0x12,0x02);
	SP0A20_write_cmos_sensor(0x13,0x2f);
	SP0A20_write_cmos_sensor(0x6d,0x32);
	SP0A20_write_cmos_sensor(0x6c,0x32);
	SP0A20_write_cmos_sensor(0x6f,0x33);
	SP0A20_write_cmos_sensor(0x6e,0x34);
	SP0A20_write_cmos_sensor(0xfd,0x00);
	SP0A20_write_cmos_sensor(0x92,0x11);
	SP0A20_write_cmos_sensor(0x99,0x05);
	SP0A20_write_cmos_sensor(0x16,0x38);
	SP0A20_write_cmos_sensor(0x17,0x38);
	SP0A20_write_cmos_sensor(0x70,0x3a);
	SP0A20_write_cmos_sensor(0x14,0x02);
	SP0A20_write_cmos_sensor(0x15,0x20);
	SP0A20_write_cmos_sensor(0x71,0x23);
	SP0A20_write_cmos_sensor(0x69,0x25);
	SP0A20_write_cmos_sensor(0x6a,0x1a);
	SP0A20_write_cmos_sensor(0x72,0x1c);
	SP0A20_write_cmos_sensor(0x75,0x1e);
	SP0A20_write_cmos_sensor(0x73,0x3c);
	SP0A20_write_cmos_sensor(0x74,0x21);
	SP0A20_write_cmos_sensor(0x79,0x00);
	SP0A20_write_cmos_sensor(0x77,0x10);
	SP0A20_write_cmos_sensor(0x1a,0x4d);
	SP0A20_write_cmos_sensor(0x1c,0x07);
	SP0A20_write_cmos_sensor(0x1e,0x15);
	SP0A20_write_cmos_sensor(0x21,0x08);//0x0e 2015-1-30 sp_miao
	SP0A20_write_cmos_sensor(0x22,0x28);
	SP0A20_write_cmos_sensor(0x26,0x66);
	SP0A20_write_cmos_sensor(0x28,0x0b);
SP0A20_write_cmos_sensor(0x37,0x5a);  //4a
SP0A20_write_cmos_sensor(0xfd,0x02);
SP0A20_write_cmos_sensor(0x01,0x80);

	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x41,0x00);
	SP0A20_write_cmos_sensor(0x42,0x00);
	SP0A20_write_cmos_sensor(0x43,0x00);
	SP0A20_write_cmos_sensor(0x44,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x00);
	SP0A20_write_cmos_sensor(0x03,0x01);
	SP0A20_write_cmos_sensor(0x04,0x68);
	SP0A20_write_cmos_sensor(0x05,0x00);
	SP0A20_write_cmos_sensor(0x06,0x00);
	SP0A20_write_cmos_sensor(0x07,0x00);
	SP0A20_write_cmos_sensor(0x08,0x00);
	SP0A20_write_cmos_sensor(0x09,0x04);
	SP0A20_write_cmos_sensor(0x0a,0x84);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xf0,0x00);
	SP0A20_write_cmos_sensor(0xf7,0x3c);
	SP0A20_write_cmos_sensor(0x02,0x0c);
	SP0A20_write_cmos_sensor(0x03,0x01);
	SP0A20_write_cmos_sensor(0x06,0x3c);
	SP0A20_write_cmos_sensor(0x07,0x00);
	SP0A20_write_cmos_sensor(0x08,0x01);
	SP0A20_write_cmos_sensor(0x09,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xbe,0xd0);
	SP0A20_write_cmos_sensor(0xbf,0x02);
	SP0A20_write_cmos_sensor(0xd0,0xd0);
	SP0A20_write_cmos_sensor(0xd1,0x02);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x5a,0x40);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xbc,0x70);
	SP0A20_write_cmos_sensor(0xbd,0x50);
SP0A20_write_cmos_sensor(0xb8,0x66);
SP0A20_write_cmos_sensor(0xb9,0x8f);
	SP0A20_write_cmos_sensor(0xba,0x30);
	SP0A20_write_cmos_sensor(0xbb,0x45);
	// 2015-1-30 sp_miao RPC
	SP0A20_write_cmos_sensor(0xfd,0x01);
SP0A20_write_cmos_sensor(0xe0,0x60);
SP0A20_write_cmos_sensor(0xe1,0x48);
SP0A20_write_cmos_sensor(0xe2,0x40);
SP0A20_write_cmos_sensor(0xe3,0x3a);
SP0A20_write_cmos_sensor(0xe4,0x3a);
SP0A20_write_cmos_sensor(0xe5,0x38);
SP0A20_write_cmos_sensor(0xe6,0x38);
SP0A20_write_cmos_sensor(0xe7,0x34);
SP0A20_write_cmos_sensor(0xe8,0x34);
SP0A20_write_cmos_sensor(0xe9,0x34);
SP0A20_write_cmos_sensor(0xea,0x32);
SP0A20_write_cmos_sensor(0xf3,0x32);
SP0A20_write_cmos_sensor(0xf4,0x32);
	SP0A20_write_cmos_sensor(0xfd,0x01);
SP0A20_write_cmos_sensor(0x04,0xa0);
SP0A20_write_cmos_sensor(0x05,0x32);
SP0A20_write_cmos_sensor(0x0a,0xa0);
SP0A20_write_cmos_sensor(0x0b,0x32);
	SP0A20_write_cmos_sensor(0xfd,0x01);
SP0A20_write_cmos_sensor(0xeb,0x7f);
SP0A20_write_cmos_sensor(0xec,0x7f);
SP0A20_write_cmos_sensor(0xed,0x05);
SP0A20_write_cmos_sensor(0xee,0x0a);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xf2,0x4d);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x5b,0x05);
	SP0A20_write_cmos_sensor(0x5c,0xa0);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x26,0x80);
	SP0A20_write_cmos_sensor(0x27,0x4f);
	SP0A20_write_cmos_sensor(0x28,0x00);
	SP0A20_write_cmos_sensor(0x29,0x20);
	SP0A20_write_cmos_sensor(0x2a,0x00);
	SP0A20_write_cmos_sensor(0x2b,0x03);
	SP0A20_write_cmos_sensor(0x2c,0x00);
	SP0A20_write_cmos_sensor(0x2d,0x20);
	SP0A20_write_cmos_sensor(0x30,0x00);
	SP0A20_write_cmos_sensor(0x31,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xa1,0x1a);
	SP0A20_write_cmos_sensor(0xa2,0x1e);
	SP0A20_write_cmos_sensor(0xa3,0x19);
	SP0A20_write_cmos_sensor(0xa4,0x1b);
	SP0A20_write_cmos_sensor(0xa5,0x10);
	SP0A20_write_cmos_sensor(0xa6,0x12);
	SP0A20_write_cmos_sensor(0xa7,0x13);
	SP0A20_write_cmos_sensor(0xa8,0x13);
	SP0A20_write_cmos_sensor(0xa9,0x10);
	SP0A20_write_cmos_sensor(0xaa,0x10);
	SP0A20_write_cmos_sensor(0xab,0x0d);
	SP0A20_write_cmos_sensor(0xac,0x0d);
	SP0A20_write_cmos_sensor(0xad,0x04);
	SP0A20_write_cmos_sensor(0xae,0x00);
	SP0A20_write_cmos_sensor(0xaf,0x00);
	SP0A20_write_cmos_sensor(0xb0,0x08);
	SP0A20_write_cmos_sensor(0xb1,0x04);
	SP0A20_write_cmos_sensor(0xb2,0x00);
	SP0A20_write_cmos_sensor(0xb3,0x00);
	SP0A20_write_cmos_sensor(0xb4,0x0a);
	SP0A20_write_cmos_sensor(0xb5,0x04);
	SP0A20_write_cmos_sensor(0xb6,0x00);
	SP0A20_write_cmos_sensor(0xb7,0x00);
	SP0A20_write_cmos_sensor(0xb8,0x08);
	SP0A20_write_cmos_sensor(0xfd,0x02);
SP0A20_write_cmos_sensor(0x09,0x09);
SP0A20_write_cmos_sensor(0x0d,0x1a);
SP0A20_write_cmos_sensor(0x1d,0x03);
SP0A20_write_cmos_sensor(0x1f,0x04);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x32,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x26,0xb8);
	SP0A20_write_cmos_sensor(0x27,0xa2);
	SP0A20_write_cmos_sensor(0xfd,0x00);
	SP0A20_write_cmos_sensor(0xe7,0x03);
	SP0A20_write_cmos_sensor(0xe7,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x10,0x22);
	SP0A20_write_cmos_sensor(0x11,0x00);
	SP0A20_write_cmos_sensor(0x1b,0x80);
	SP0A20_write_cmos_sensor(0x1a,0x80);
	SP0A20_write_cmos_sensor(0x18,0x27);
	SP0A20_write_cmos_sensor(0x19,0x26);
	SP0A20_write_cmos_sensor(0x2a,0x01);
	SP0A20_write_cmos_sensor(0x2b,0x08);
	SP0A20_write_cmos_sensor(0x28,0xf8);
	SP0A20_write_cmos_sensor(0x29,0x08);
	SP0A20_write_cmos_sensor(0x66,0x50);
	SP0A20_write_cmos_sensor(0x67,0x70);
	SP0A20_write_cmos_sensor(0x68,0xdc);
	SP0A20_write_cmos_sensor(0x69,0xf7);
	SP0A20_write_cmos_sensor(0x6a,0xa5);
	SP0A20_write_cmos_sensor(0x7c,0x2d);
	SP0A20_write_cmos_sensor(0x7d,0x4a);
	SP0A20_write_cmos_sensor(0x7e,0xf7);
	SP0A20_write_cmos_sensor(0x7f,0x1f);
	SP0A20_write_cmos_sensor(0x80,0xa6);
	SP0A20_write_cmos_sensor(0x70,0x27);
	SP0A20_write_cmos_sensor(0x71,0x40);
	SP0A20_write_cmos_sensor(0x72,0x20);
	SP0A20_write_cmos_sensor(0x73,0x49);
	SP0A20_write_cmos_sensor(0x74,0xaa);
	SP0A20_write_cmos_sensor(0x6b,0x0b);
	SP0A20_write_cmos_sensor(0x6c,0x27);
	SP0A20_write_cmos_sensor(0x6d,0x30);
	SP0A20_write_cmos_sensor(0x6e,0x4b);
	SP0A20_write_cmos_sensor(0x6f,0xaa);
	SP0A20_write_cmos_sensor(0x61,0xf7);
	SP0A20_write_cmos_sensor(0x62,0x14);
	SP0A20_write_cmos_sensor(0x63,0x4b);
	SP0A20_write_cmos_sensor(0x64,0x68);
	SP0A20_write_cmos_sensor(0x65,0x6a);
	SP0A20_write_cmos_sensor(0x75,0x80);
	SP0A20_write_cmos_sensor(0x76,0x09);
	SP0A20_write_cmos_sensor(0x77,0x02);
	SP0A20_write_cmos_sensor(0x24,0x25);
	SP0A20_write_cmos_sensor(0x0e,0x16);
	SP0A20_write_cmos_sensor(0x3b,0x09);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xcb,0x20);
	SP0A20_write_cmos_sensor(0xcc,0x2c);
	SP0A20_write_cmos_sensor(0xcd,0x2c);
	SP0A20_write_cmos_sensor(0xce,0x2c);
	SP0A20_write_cmos_sensor(0xde,0x0f);
	SP0A20_write_cmos_sensor(0xd7,0x08);
	SP0A20_write_cmos_sensor(0xd8,0x08);
	SP0A20_write_cmos_sensor(0xd9,0x10);
	SP0A20_write_cmos_sensor(0xda,0x14);
	SP0A20_write_cmos_sensor(0xe8,0x20);
	SP0A20_write_cmos_sensor(0xe9,0x20);
	SP0A20_write_cmos_sensor(0xea,0x20);
	SP0A20_write_cmos_sensor(0xeb,0x20);
	SP0A20_write_cmos_sensor(0xec,0x14);
	SP0A20_write_cmos_sensor(0xed,0x18);
	SP0A20_write_cmos_sensor(0xee,0x20);
	SP0A20_write_cmos_sensor(0xef,0x20);
	SP0A20_write_cmos_sensor(0xd3,0x20);
	SP0A20_write_cmos_sensor(0xd4,0x38);
	SP0A20_write_cmos_sensor(0xd5,0x20);
	SP0A20_write_cmos_sensor(0xd6,0x08);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xd1,0x20);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xdc,0x05);
	SP0A20_write_cmos_sensor(0x05,0x20);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x81,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xfc,0x00);
	SP0A20_write_cmos_sensor(0x7d,0x05);
	SP0A20_write_cmos_sensor(0x7e,0x05);
	SP0A20_write_cmos_sensor(0x7f,0x09);
	SP0A20_write_cmos_sensor(0x80,0x08);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xdd,0x0f);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x6d,0x0a);
	SP0A20_write_cmos_sensor(0x6e,0x0e);
	SP0A20_write_cmos_sensor(0x6f,0x20);
	SP0A20_write_cmos_sensor(0x70,0x20);
	SP0A20_write_cmos_sensor(0x86,0x18);
	SP0A20_write_cmos_sensor(0x71,0x0c);
	SP0A20_write_cmos_sensor(0x72,0x0e);
	SP0A20_write_cmos_sensor(0x73,0x20);
	SP0A20_write_cmos_sensor(0x74,0x20);
	SP0A20_write_cmos_sensor(0x75,0x08);
	SP0A20_write_cmos_sensor(0x76,0x0a);
	SP0A20_write_cmos_sensor(0x77,0x06);
	SP0A20_write_cmos_sensor(0x78,0x06);
	SP0A20_write_cmos_sensor(0x79,0x56);
	SP0A20_write_cmos_sensor(0x7a,0x45);
	SP0A20_write_cmos_sensor(0x7b,0x34);
	SP0A20_write_cmos_sensor(0x7c,0x22);
	SP0A20_write_cmos_sensor(0x81,0x0d);
	SP0A20_write_cmos_sensor(0x82,0x18);
	SP0A20_write_cmos_sensor(0x83,0x20);
	SP0A20_write_cmos_sensor(0x84,0x24);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x83,0x12);
	SP0A20_write_cmos_sensor(0x84,0x14);
	SP0A20_write_cmos_sensor(0x86,0x04);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x61,0x60);
	SP0A20_write_cmos_sensor(0x62,0x28);
	SP0A20_write_cmos_sensor(0x8a,0x10);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x8b,0x00);
	SP0A20_write_cmos_sensor(0x8c,0x0a);
	SP0A20_write_cmos_sensor(0x8d,0x18);
	SP0A20_write_cmos_sensor(0x8e,0x29);
	SP0A20_write_cmos_sensor(0x8f,0x39);
	SP0A20_write_cmos_sensor(0x90,0x4f);
	SP0A20_write_cmos_sensor(0x91,0x62);
	SP0A20_write_cmos_sensor(0x92,0x71);
	SP0A20_write_cmos_sensor(0x93,0x7f);
	SP0A20_write_cmos_sensor(0x94,0x93);
	SP0A20_write_cmos_sensor(0x95,0xa3);
	SP0A20_write_cmos_sensor(0x96,0xb0);
	SP0A20_write_cmos_sensor(0x97,0xbb);
	SP0A20_write_cmos_sensor(0x98,0xc6);
	SP0A20_write_cmos_sensor(0x99,0xce);
	SP0A20_write_cmos_sensor(0x9a,0xd5);
	SP0A20_write_cmos_sensor(0x9b,0xdc);
	SP0A20_write_cmos_sensor(0x9c,0xe3);
	SP0A20_write_cmos_sensor(0x9d,0xe8);
	SP0A20_write_cmos_sensor(0x9e,0xec);
	SP0A20_write_cmos_sensor(0x9f,0xf1);
	SP0A20_write_cmos_sensor(0xa0,0xf6);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x15,0xc0);
	SP0A20_write_cmos_sensor(0x16,0x8c);
	SP0A20_write_cmos_sensor(0xa0,0x86);
	SP0A20_write_cmos_sensor(0xa1,0xfa);
	SP0A20_write_cmos_sensor(0xa2,0x00);
	SP0A20_write_cmos_sensor(0xa3,0xdb);
	SP0A20_write_cmos_sensor(0xa4,0xc0);
	SP0A20_write_cmos_sensor(0xa5,0xe6);
	SP0A20_write_cmos_sensor(0xa6,0xed);
	SP0A20_write_cmos_sensor(0xa7,0xda);
	SP0A20_write_cmos_sensor(0xa8,0xb9);
	SP0A20_write_cmos_sensor(0xa9,0x0c);
	SP0A20_write_cmos_sensor(0xaa,0x33);
	SP0A20_write_cmos_sensor(0xab,0x0f);
	SP0A20_write_cmos_sensor(0xac,0x80);
	SP0A20_write_cmos_sensor(0xad,0xed);
	SP0A20_write_cmos_sensor(0xae,0x13);
	SP0A20_write_cmos_sensor(0xaf,0xcd);
	SP0A20_write_cmos_sensor(0xb0,0x99);
	SP0A20_write_cmos_sensor(0xb1,0x19);
	SP0A20_write_cmos_sensor(0xb2,0xc7);
	SP0A20_write_cmos_sensor(0xb3,0x9a);
	SP0A20_write_cmos_sensor(0xb4,0x20);
	SP0A20_write_cmos_sensor(0xb5,0x0c);
	SP0A20_write_cmos_sensor(0xb6,0x03);
	SP0A20_write_cmos_sensor(0xb7,0x1f);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xd3,0x70);
	SP0A20_write_cmos_sensor(0xd4,0x6c);
	SP0A20_write_cmos_sensor(0xd5,0x66);
	SP0A20_write_cmos_sensor(0xd6,0x5e);
	SP0A20_write_cmos_sensor(0xd7,0x70);
	SP0A20_write_cmos_sensor(0xd8,0x6c);
	SP0A20_write_cmos_sensor(0xd9,0x66);
	SP0A20_write_cmos_sensor(0xda,0x5e);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xdd,0x30);
	SP0A20_write_cmos_sensor(0xde,0x10);
	SP0A20_write_cmos_sensor(0xdf,0xff);
	SP0A20_write_cmos_sensor(0x00,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xc2,0x77);
	SP0A20_write_cmos_sensor(0xc3,0x77);
	SP0A20_write_cmos_sensor(0xc4,0x66);
	SP0A20_write_cmos_sensor(0xc5,0x55);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xcd,0x10);
	SP0A20_write_cmos_sensor(0xce,0x1f);
	SP0A20_write_cmos_sensor(0xcf,0x30);
	SP0A20_write_cmos_sensor(0xd0,0x45);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x31,0x60);
	SP0A20_write_cmos_sensor(0x32,0x60);
	SP0A20_write_cmos_sensor(0x33,0xc0);
	SP0A20_write_cmos_sensor(0x35,0x60);
	SP0A20_write_cmos_sensor(0x36,0x10);
	SP0A20_write_cmos_sensor(0x37,0x13);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x0e,0x80);
	SP0A20_write_cmos_sensor(0x0f,0x20);
	SP0A20_write_cmos_sensor(0x10,0x80);
	SP0A20_write_cmos_sensor(0x11,0x7c);
	SP0A20_write_cmos_sensor(0x12,0x78);
	SP0A20_write_cmos_sensor(0x13,0x70);
	SP0A20_write_cmos_sensor(0x14,0x88);
	SP0A20_write_cmos_sensor(0x15,0x84);
	SP0A20_write_cmos_sensor(0x16,0x80);
	SP0A20_write_cmos_sensor(0x17,0x7d);
	SP0A20_write_cmos_sensor(0xfd,0x00);
	SP0A20_write_cmos_sensor(0x31,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x32,0x15);
	SP0A20_write_cmos_sensor(0x33,0xef);
	SP0A20_write_cmos_sensor(0x34,0x07);
	SP0A20_write_cmos_sensor(0xd2,0x01);
	SP0A20_write_cmos_sensor(0xfb,0x25);
	SP0A20_write_cmos_sensor(0xf2,0x49);
	SP0A20_write_cmos_sensor(0x35,0x40);
	SP0A20_write_cmos_sensor(0x5d,0x11);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x36,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x1b,0x20);
}

//extern u32 pinSetIdx;
//extern u32 SensorID;
UINT32 SP0A20GetSensorID(UINT32 *sensorID)
{
    int  retry = 3; 
#ifdef SLT_DEVINFO_CMM 
 	s_DEVINFO_ccm =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);	
	s_DEVINFO_ccm->device_type = "CCM-S";
	s_DEVINFO_ccm->device_module = "PCOEJ0003A";//can change if got module id
	s_DEVINFO_ccm->device_vendor = "Sanlaishi";
	s_DEVINFO_ccm->device_ic = "SP0A20";
	s_DEVINFO_ccm->device_version = "SuperPix";
	s_DEVINFO_ccm->device_info = "30W";
#endif	

    // check if sensor ID correct
    do {
#if 1
        SP0A20_write_cmos_sensor(0xfd,0x00);
	*sensorID = SP0A20_read_cmos_sensor(0x02);
	SENSORDB("%s,Read Sensor ID = 0x%04x\n",__func__, *sensorID);
#else
        if(SensorID==4)
        {
//	SP0A20_write_cmos_sensor(0xfd,0x00);
//	*sensorID = SP0A20_read_cmos_sensor(0x02);
//	SENSORDB("%s,Read Sensor ID = 0x%04x\n",__func__, *sensorID);
        *sensorID = SP0A20_SENSOR_ID;
        }
        else
        {
            *sensorID = 0;
        }
	//SP0A20_write_cmos_sensor(0xfd,0x00);
	//*sensorID = SP0A20_read_cmos_sensor(0x02);
	//SENSORDB("%s,Read Sensor ID = 0x%04x\n",__func__, *sensorID);
#endif
        if (*sensorID == SP0A20_SENSOR_ID)
            break; 
        SENSORDB("Read Sensor ID Fail = 0x%04x\n", *sensorID); 
        retry--; 
    } while (retry > 0);

    if (*sensorID != SP0A20_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
	#ifdef SLT_DEVINFO_CMM 
		s_DEVINFO_ccm->device_used = DEVINFO_UNUSED;
		devinfo_check_add_device(s_DEVINFO_ccm);
	#endif
        return ERROR_SENSOR_CONNECT_FAIL;
    }

#ifdef SLT_DEVINFO_CMM 
	s_DEVINFO_ccm->device_used = DEVINFO_USED;
	devinfo_check_add_device(s_DEVINFO_ccm);
#endif	  
    return ERROR_NONE;    
}




/*************************************************************************
* FUNCTION
*	SP0A20_Write_More_Registers
*
* DESCRIPTION
*	This function is served for FAE to modify the necessary Init Regs. Do not modify the regs
*     in init_SP0A20() directly.
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
void SP0A20_Write_More_Registers(void)
{

}


/*************************************************************************
 * FUNCTION
 *	SP0A20Open
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
UINT32 SP0A20Open(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	printk("<Jet> Entry SP0A20Open!!!\r\n");

	//Sleep(10);


	//  Read sensor ID to adjust I2C is OK?
#if 1
for(i=0;i<3;i++)
{
        SP0A20_write_cmos_sensor(0xfd,0x00);
        sensor_id = SP0A20_read_cmos_sensor(0x02);
        SENSORDB("%s,Read Sensor ID = 0x%x\n", __func__,sensor_id);
        if(sensor_id == SP0A20_SENSOR_ID)  
        {
            SENSORDB("SP0A20_ Sensor Read ID OK \r\n");
                break;
        }
}
#else
if(SensorID==4)
{
/*
for(i=0;i<3;i++)
{
    SP0A20_write_cmos_sensor(0xfd,0x00);
        sensor_id = SP0A20_read_cmos_sensor(0x02);
        SENSORDB("%s,Read Sensor ID = 0x%x\n", __func__,sensor_id);
        if(sensor_id == SP0A20_SENSOR_ID)  
        {
            SENSORDB("SP0A20_ Sensor Read ID OK \r\n");
                break;
        }
}
*/    sensor_id = SP0A20_SENSOR_ID;
}
else
{
    sensor_id = 0;
}
#endif
	if(sensor_id != SP0A20_SENSOR_ID)  
	{
		SENSORDB("SP0A20 Read Sensor ID Fail[open] = 0x%x\n", sensor_id); 
		return ERROR_SENSOR_CONNECT_FAIL;
	}

#ifdef DEBUG_SENSOR_SP0A20  
		struct file *fp; 
		mm_segment_t fs; 
		loff_t pos = 0; 
		static char buf[60*1024] ;


	fp = filp_open("/mnt/sdcard/sp0a20_sd", O_RDONLY , 0); 

	if (IS_ERR(fp)) { 
		 
		fromsd = 0;   
		printk("open file error\n");
	} 
	else 
	{
		fromsd = 1;
		printk("open file ok\n");

		filp_close(fp, NULL); 
		set_fs(fs);
	}

	if(fromsd == 1)
	{
		printk("________________from t!\n");
		SP0A20_Initialize_from_T_Flash();
	}
	else
	{
		SP0A20_Sensor_Init();
		SP0A20_Write_More_Registers();//added for FAE to debut
	}
#else  
	//RETAILMSG(1, (TEXT("Sensor Read ID OK \r\n")));
	// initail sequence write in
	SP0A20_Sensor_Init();
	SP0A20_Write_More_Registers();//added for FAE to debut
#endif
    return ERROR_NONE;
} /* SP0A20Open */


/*************************************************************************
 * FUNCTION
 *	SP0A20Close
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
UINT32 SP0A20Close(void)
{
    return ERROR_NONE;
} /* SP0A20Close */


/*************************************************************************
 * FUNCTION
 * SP0A20Preview
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
UINT32 SP0A20Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
//    kal_uint32 iTemp;
//    kal_uint16 iStartX = 0, iStartY = 1;

    if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        RETAILMSG(1, (TEXT("Camera Video preview\r\n")));
        SP0A20_MPEG4_encode_mode = KAL_TRUE;
       
    }
    else
    {
        RETAILMSG(1, (TEXT("Camera preview\r\n")));
        SP0A20_MPEG4_encode_mode = KAL_FALSE;
    }

    image_window->GrabStartX= IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY= IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;

    // copy sensor_config_data
    memcpy(&SP0A20SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* SP0A20Preview */


/*************************************************************************
 * FUNCTION
 *	SP0A20Capture
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
UINT32 SP0A20Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    SP0A20_MODE_CAPTURE=KAL_TRUE;

    image_window->GrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth= IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = IMAGE_SENSOR_FULL_HEIGHT;

    // copy sensor_config_data
    memcpy(&SP0A20SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* SP0A20_Capture() */

/*static UINT32 SP0A20GetExposureTime(void)
{
    UINT8 high_base,low_base,high_exp,low_exp = 0;
    UINT32 EXP;
UINT32 temp1;
   	SP0A20_write_cmos_sensor(0xfd, 0x00);
	 
	high_exp =  SP0A20_read_cmos_sensor(0x03);
	low_exp = SP0A20_read_cmos_sensor(0x04);
	
	SP0A20_write_cmos_sensor(0xfd, 0x01);
	
	high_base = SP0A20_read_cmos_sensor(0xf0);
	
	low_base = SP0A20_read_cmos_sensor(0xf7);
	temp1 = (( (( high_exp << 8) | low_exp)) /( (high_base<<8) | low_base));
	EXP =(( (( high_exp << 8) | low_exp)) /( (high_base<<8) | low_base))* 10 *5000;//return us
printk("%s,EXP = %d,temp1 = %d\n",__func__,EXP,temp1);
	return EXP;
}
*/

/*void SP0A20GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;

	pExifInfo->CapExposureTime = SP0A20GetExposureTime();

}*/

UINT32 SP0A20GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorVideoWidth=IMAGE_SENSOR_PV_WIDTH;

    pSensorResolution->SensorVideoHeight=IMAGE_SENSOR_PV_HEIGHT;
    return ERROR_NONE;
} /* SP0A20GetResolution() */


UINT32 SP0A20GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_HEIGHT;

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
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;//MIPI setting
    pSensorInfo->CaptureDelayFrame = 2;
    pSensorInfo->PreviewDelayFrame = 1;
    pSensorInfo->VideoDelayFrame = 4;
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_2MA;

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
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
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
    SP0A20PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &SP0A20SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* SP0A20GetInfo() */


UINT32 SP0A20Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	printk("%s,MIPIControl start.\n",__func__);
    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    default:
	 SP0A20Preview(pImageWindow, pSensorConfigData);
        break;
    }

	printk("%s,MIPIControl end.\n",__func__);
    return TRUE;
}	/* SP0A20Control() */

BOOL SP0A20_set_param_wb(UINT16 para)
{
	if(SP0A20_TST_PATTEN==KAL_TRUE)
		return 0;

	printk("%s,enter, para = %x\n",__func__,para);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x36,0x02);

	switch (para)
	{
		case AWB_MODE_OFF:
			printk("%s,AWB_MODE_OFF.\n",__func__);
			break;
		
		case AWB_MODE_AUTO:
			SP0A20_awb_enable(KAL_TRUE);
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x32,0x15);
			SP0A20_write_cmos_sensor(0xfd,0x02);
			SP0A20_write_cmos_sensor(0x26,0xbf);
			SP0A20_write_cmos_sensor(0x27,0xa3);
			SP0A20_write_cmos_sensor(0xfd,0x00);
			SP0A20_write_cmos_sensor(0xe7,0x03);
			SP0A20_write_cmos_sensor(0xe7,0x00);
			break;
		
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			SP0A20_awb_enable(KAL_FALSE);
			SP0A20_write_cmos_sensor(0xfd,0x01);	//4200-5000K 								  
			
			SP0A20_write_cmos_sensor(0x32,0x05);															
			SP0A20_write_cmos_sensor(0xfd,0x02);											
			SP0A20_write_cmos_sensor(0x26,0x95);																
			SP0A20_write_cmos_sensor(0x27,0xba);																
			SP0A20_write_cmos_sensor(0xfd,0x00);
			SP0A20_write_cmos_sensor(0xe7,0x03);
			SP0A20_write_cmos_sensor(0xe7,0x00);	
			break;
		
		case AWB_MODE_DAYLIGHT: //sunny
			SP0A20_awb_enable(KAL_FALSE);
			// SP0A20_reg_WB_auto 
			SP0A20_write_cmos_sensor(0xfd,0x01);	//2800K~3000K									  
			SP0A20_write_cmos_sensor(0x32,0x05);															
			SP0A20_write_cmos_sensor(0xfd,0x02);															
			SP0A20_write_cmos_sensor(0x26,0x88);																
			SP0A20_write_cmos_sensor(0x27,0xb0);	//d0																
			SP0A20_write_cmos_sensor(0xfd,0x00);
			SP0A20_write_cmos_sensor(0xe7,0x03);
			SP0A20_write_cmos_sensor(0xe7,0x00);			
			break;
		
		case AWB_MODE_INCANDESCENT: //office
			SP0A20_awb_enable(KAL_FALSE);
			// SP0A20_reg_WB_auto 
			SP0A20_write_cmos_sensor(0xfd,0x01);	//2800K~3000K									  
			SP0A20_write_cmos_sensor(0x32,0x05);															
			SP0A20_write_cmos_sensor(0xfd,0x02);															
			SP0A20_write_cmos_sensor(0x26,0x88);																
			SP0A20_write_cmos_sensor(0x27,0xd0);																
			SP0A20_write_cmos_sensor(0xfd,0x00);
			SP0A20_write_cmos_sensor(0xe7,0x03);
			SP0A20_write_cmos_sensor(0xe7,0x00);	
			break;
		
		case AWB_MODE_TUNGSTEN: //home
			SP0A20_awb_enable(KAL_FALSE);
			SP0A20_write_cmos_sensor(0xfd,0x01);	//4000K 								  
			SP0A20_write_cmos_sensor(0x32,0x05);															
			SP0A20_write_cmos_sensor(0xfd,0x02);															
			SP0A20_write_cmos_sensor(0x26,0xac);																
			SP0A20_write_cmos_sensor(0x27,0xbe);																
			SP0A20_write_cmos_sensor(0xfd,0x00);
			SP0A20_write_cmos_sensor(0xe7,0x03);
			SP0A20_write_cmos_sensor(0xe7,0x00);
			break;
		case AWB_MODE_TWILIGHT:	//5 
			SP0A20_awb_enable(KAL_FALSE);
			SP0A20_write_cmos_sensor(0xfd,0x01);	 //7000K									 
			SP0A20_write_cmos_sensor(0x32,0x05);															
			SP0A20_write_cmos_sensor(0xfd,0x02);															
			SP0A20_write_cmos_sensor(0x26,0xdb);																
			SP0A20_write_cmos_sensor(0x27,0x80);	//70																
			SP0A20_write_cmos_sensor(0xfd,0x00);
			SP0A20_write_cmos_sensor(0xe7,0x03);
			SP0A20_write_cmos_sensor(0xe7,0x00);	
			break;
		case AWB_MODE_FLUORESCENT:
			SP0A20_awb_enable(KAL_FALSE);
			SP0A20_write_cmos_sensor(0xfd,0x01);	//4200-5000K 								  
			SP0A20_write_cmos_sensor(0x32,0x05);															
			SP0A20_write_cmos_sensor(0xfd,0x02);															
			SP0A20_write_cmos_sensor(0x26,0xbf);	//95															
			SP0A20_write_cmos_sensor(0x27,0x89);	//ba														
			SP0A20_write_cmos_sensor(0xfd,0x00);
			SP0A20_write_cmos_sensor(0xe7,0x03);
			SP0A20_write_cmos_sensor(0xe7,0x00);	
	      break;
		
		default:
			return FALSE;
	}

	SP0A20_write_cmos_sensor(0xfd,0x00);
	SP0A20_write_cmos_sensor(0xe7,0x03);
	SP0A20_write_cmos_sensor(0xe7,0x00);
	Sleep(10);

	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x36,0x00);

	printk("%s,exit.\n",__func__);
	return TRUE;
} /* SP0A20_set_param_wb */


BOOL SP0A20_set_param_effect(UINT16 para)
{
	kal_uint32  ret = KAL_TRUE;
    SENSORDB("%s,para= %d.\n",__func__,para);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x36,0x02);

	switch (para)
	{
		case MEFFECT_OFF:  
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x00);
			SP0A20_write_cmos_sensor(0x67,0x80);
			SP0A20_write_cmos_sensor(0x68,0x80);
			break;

		case  MEFFECT_SOLARIZE: 
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x10);
			SP0A20_write_cmos_sensor(0x67,0x98);
			SP0A20_write_cmos_sensor(0x68,0x58);                     
			break;
		case MEFFECT_SEPIA: 
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x10);
			SP0A20_write_cmos_sensor(0x67,0x98);
			SP0A20_write_cmos_sensor(0x68,0x58);
			break;
		case MEFFECT_NEGATIVE: 
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x04);
			SP0A20_write_cmos_sensor(0x67,0x80);
			SP0A20_write_cmos_sensor(0x68,0x80);
			break;

		case MEFFECT_SEPIAGREEN: 
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x10);
			SP0A20_write_cmos_sensor(0x67,0x50);
			SP0A20_write_cmos_sensor(0x68,0x50);
			break;

		case MEFFECT_SEPIABLUE:
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x10);
			SP0A20_write_cmos_sensor(0x67,0x50);
			SP0A20_write_cmos_sensor(0x68,0xc0);
			break;

		case MEFFECT_MONO: //B&Wdan se
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x20);
			SP0A20_write_cmos_sensor(0x67,0x80);
			SP0A20_write_cmos_sensor(0x68,0x80);   
			break;

		default: 
			return FALSE;
	}

	SP0A20_write_cmos_sensor(0xfd,0x00);
	SP0A20_write_cmos_sensor(0xe7,0x03);
	SP0A20_write_cmos_sensor(0xe7,0x00);
	Sleep(10);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x36,0x00);
	return ret;

} /* SP0A20_set_param_effect */


BOOL SP0A20_set_param_banding(UINT16 para)
{
	SP0A20_Sensor_Driver.bBanding_value = para;
	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			isBanding = 0;
			printk("SP0A20_set_param_banding_50hz\n");
			//SP0A20_set_banding_for_50Hz();
			break;
		case AE_FLICKER_MODE_60HZ:
			isBanding = 1;
			printk("SP0A20_set_param_banding_60hz\n");
			//SP0A20_set_banding_for_60Hz();
			break;
		default:
			return FALSE;
	}

	return TRUE;
} /* SP0A20_set_param_banding */


BOOL SP0A20_set_param_exposure(UINT16 para)
{


	switch (para)
	{
		case AE_EV_COMP_n13:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0xc0);
			break;

		case AE_EV_COMP_n10:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0xd0);
			break;

		case AE_EV_COMP_n07:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0xe0);
			break;

		case AE_EV_COMP_n03:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0xf0);
			break;

		case AE_EV_COMP_00:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0x00);//0xfa before
			break;


		case AE_EV_COMP_03:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0x10);
			break;

		case AE_EV_COMP_07:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0x20);
			break;

		case AE_EV_COMP_10:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0x30);
			break;

		case AE_EV_COMP_13:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0x40);
			break;

		default:
			return FALSE;
	}

	return TRUE;
} /* SP0A20_set_param_exposure */



UINT32 SP0A20YUVSetVideoMode(UINT16 u2FrameRate)    // lanking add
{
  
        SP0A20_MPEG4_encode_mode = KAL_TRUE;
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


kal_uint16 SP0A20SetTestPatternMode(kal_bool bEnable)
{
    kal_uint16 temp_tst_reg;
	SENSORDB("%s, Test pattern enable:%d\n",__func__, bEnable);
	 
	if(bEnable)    // enable test pattern output 
	{
		SP0A20_TST_PATTEN = KAL_TRUE;

		SP0A20_write_cmos_sensor(0xfd,0x01);	
		SP0A20_write_cmos_sensor(0x33,0x00);	
		SP0A20_write_cmos_sensor(0x34,0x00);
		SP0A20_write_cmos_sensor(0x36,0x02);

		SP0A20_write_cmos_sensor(0xfd,0x00);	
		SP0A20_write_cmos_sensor(0x1c,0x00);	
		SP0A20_write_cmos_sensor(0x1d,0x00);
		SP0A20_write_cmos_sensor(0x0a,0x50);

		SP0A20_write_cmos_sensor(0xfd,0x02);
		SP0A20_write_cmos_sensor(0xdd,0x00);
		SP0A20_write_cmos_sensor(0xde,0x00);

		SP0A20_write_cmos_sensor(0xfd,0x01);
		SP0A20_write_cmos_sensor(0xf2,0x0a);
		SP0A20_write_cmos_sensor(0xfb,0x00);
		SP0A20_write_cmos_sensor(0xd2,0x00);

		
		SP0A20_write_cmos_sensor(0xfd , 0x01);//sat u 
		SP0A20_write_cmos_sensor(0xd3 , 0x60);//6a
		SP0A20_write_cmos_sensor(0xd4 , 0x60);//6a
		SP0A20_write_cmos_sensor(0xd5 , 0x60);//56
		SP0A20_write_cmos_sensor(0xd6 , 0x60);//44
		//sat v
		SP0A20_write_cmos_sensor(0xd7 , 0x60);//6a  
		SP0A20_write_cmos_sensor(0xd8 , 0x60);//6a  
		SP0A20_write_cmos_sensor(0xd9 , 0x60);//56  
		SP0A20_write_cmos_sensor(0xda , 0x60);//44  


		SP0A20_write_cmos_sensor(0x10 , 0x80);//ku_outdoor
		SP0A20_write_cmos_sensor(0x11 , 0x80);//ku_nr
		SP0A20_write_cmos_sensor(0x12 , 0x80);//ku_dummy
		SP0A20_write_cmos_sensor(0x13 , 0x80);//ku_low
		SP0A20_write_cmos_sensor(0x14 , 0x80);//88//kl_outdoor 
		SP0A20_write_cmos_sensor(0x15 , 0x80);//88//kl_nr      
		SP0A20_write_cmos_sensor(0x16 , 0x80);//88//kl_dummy    
		SP0A20_write_cmos_sensor(0x17 , 0x80);//88//kl_low       

		SP0A20_write_cmos_sensor(0x00,0x12);
		SP0A20_write_cmos_sensor(0x32,0x80);	

		SP0A20_write_cmos_sensor(0xfd,0x02);	
		SP0A20_write_cmos_sensor(0x26,0x80);
		SP0A20_write_cmos_sensor(0x27,0x80);

		SP0A20_write_cmos_sensor(0xfd,0x01);	
		SP0A20_write_cmos_sensor(0x36,0x00);

		Sleep(100);
	}
	else        //disable test pattern output 
	{
        SP0A20_TST_PATTEN = KAL_FALSE;
	    Sleep(100);
		SP0A20_write_cmos_sensor(0xfd,0x01);	
		Sleep(100);
		temp_tst_reg =SP0A20_read_cmos_sensor(0x32);	
		Sleep(100);
		SP0A20_write_cmos_sensor(0x32, (temp_tst_reg &0x7f));	
		Sleep(100);
	}
               return ERROR_NONE;
}


UINT32 SP0A20YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
	SENSORDB("%s,para= %d.\n",__func__,iPara);
#ifdef DEBUG_SENSOR_SP0A20
		printk("______%s______ Tflash debug \n",__func__);
		return TRUE;
#endif

    switch (iCmd) {
    case FID_AWB_MODE:
        SP0A20_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        SP0A20_set_param_effect(iPara);
        break;
    case FID_AE_EV:
        SP0A20_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:
		SP0A20_set_param_banding(iPara);
		if (SP0A20_Sensor_Driver.bNight_mode == KAL_FALSE){
			SP0A20_night_mode(FALSE); 
		}else if (SP0A20_Sensor_Driver.bNight_mode == KAL_TRUE){
			SP0A20_night_mode(TRUE); 
		}	
		break;
    case FID_SCENE_MODE:
		SP0A20_night_mode(iPara); 
		break;
    default:
        break;
    }
    return TRUE;
} /* SP0A20YUVSensorSetting */

static void SP0A20_MIPI_GetYUVSensorBV(UINT32 *val)
{
       SP0A20_write_cmos_sensor(0xfd,0x01);
       *val = SP0A20_read_cmos_sensor(0xf1);
       printk("SP0A20_MIPI_GetYUVSensorBV val:%d\n",*val);
       SP0A20_write_cmos_sensor(0xfd,0x00);
}

UINT32 SP0A20FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
        UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    //UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    //UINT32 **ppFeatureData=(UINT32 **) pFeaturePara;
    unsigned long long *feature_data=(unsigned long long *) pFeaturePara;
    //unsigned long long *feature_return_para=(unsigned long long *) pFeaturePara;
    //UINT32 SP0A20SensorRegNumber;
    //UINT32 i;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

    RETAILMSG(1, (_T("gaiyang SP0A20FeatureControl FeatureId=%d\r\n"), FeatureId));

    switch (FeatureId)
    {
    case SENSOR_FEATURE_GET_RESOLUTION:
        *pFeatureReturnPara16++=IMAGE_SENSOR_FULL_WIDTH;
        *pFeatureReturnPara16=IMAGE_SENSOR_FULL_HEIGHT;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *pFeatureReturnPara16++=(VGA_PERIOD_PIXEL_NUMS)+SP0A20_dummy_pixels;
        *pFeatureReturnPara16=(VGA_PERIOD_LINE_NUMS)+SP0A20_dummy_lines;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        *pFeatureReturnPara32 = SP0A20_g_fPV_PCLK;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:
        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        //SP0A20NightMode((BOOL) *pFeatureData16);
        break;
    case SENSOR_FEATURE_SET_GAIN:
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        SP0A20_isp_master_clock=*pFeatureData32;
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        SP0A20_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        pSensorRegData->RegData = SP0A20_read_cmos_sensor(pSensorRegData->RegAddr);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
        memcpy(pSensorConfigData, &SP0A20SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
        SP0A20YUVSensorSetting((FEATURE_ID)*feature_data, *(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:    //  lanking
	 SP0A20YUVSetVideoMode(*feature_data);
	 break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
	SP0A20GetSensorID(pFeatureData32);
	break;
/*    case SENSOR_FEATURE_GET_EXIF_INFO:
		SP0A20GetExifInfo((uintptr_t)*feature_data);
		break;
                */
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		   SP0A20SetTestPatternMode((BOOL)*feature_data);
		   break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing			   
		 *pFeatureReturnPara32= SP0A20_TEST_PATTERN_CHECKSUM;
		 *pFeatureParaLen=4;						   
		   break;	  
	case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
		 printk("[SP0A20] SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO\n");
		 SP0A20_MIPI_GetYUVSensorBV(pFeatureData32);
		 break;
    default:
        break;
	}

return ERROR_NONE;
}	/* SP0A20FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncSP0A20YUV=
{
	SP0A20Open,
	SP0A20GetInfo,
	SP0A20GetResolution,
	SP0A20FeatureControl,
	SP0A20Control,
	SP0A20Close
};


UINT32 SP0A20_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncSP0A20YUV;
	return ERROR_NONE;
} /* SensorInit() */
