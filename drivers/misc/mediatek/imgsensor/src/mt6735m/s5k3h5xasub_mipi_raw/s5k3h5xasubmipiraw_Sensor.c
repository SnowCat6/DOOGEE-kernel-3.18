/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 S5K3H5XASUBmipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

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

#include "s5k3h5xasubmipiraw_Sensor.h"

#define PFX "S5K3H5XASUB_camera_sensor"
#define LOG_1 LOG_INF("S5K2P8,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2664*1500@30fps,888Mbps/lane; video 5328*3000@30fps,1390Mbps/lane; capture 16M@30fps,1390Mbps/lane\n")
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOGE(format, args...)   pr_err(PFX "[%s] " format, __FUNCTION__, ##args)
static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define SLOW_MOTION_120FPS

extern bool read_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size);
#ifdef PDAF_TEST
extern bool wrtie_eeprom(kal_uint16 addr, BYTE data[],kal_uint32 size );
char data[4096]= {0};
char data2[4096]= {0};
#endif
static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = S5K3H5XASUB_SENSOR_ID,
	
	.checksum_value = 0x82256eb5,
	
	.pre = {
		.pclk = 280000000,
		.linelength = 3688,
		.framelength = 2530,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 280000000,
		.linelength = 3688,
		.framelength = 2530,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 280000000,
		.linelength = 7376,
		.framelength = 2530,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 150,
	},
	.normal_video = {
		.pclk = 280000000,
		.linelength = 3688,
		.framelength = 2530,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 280000000,
		.linelength = 3688,
		.framelength = 632,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 800,
		.grabwindow_height = 600,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 280000000,
		.linelength = 3688,
		.framelength = 2530,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.margin = 16,
	.min_shutter = 3,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 3, 
	.pre_delay_frame = 3, 
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,
	
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 12,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20, 0x5a, 0xff},
    .i2c_speed = 300, 
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
    .current_fps = 30,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_TRUE for in test pattern mode, KAL_FALSE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 3280, 2464,	  8,	  8,   3264, 2448, 1632, 1224, 0000, 0000, 1632, 1224,	0,	0, 1600, 1200}, // Preview 
 { 3280, 2464,	  8,	  8,   3264, 2448, 3264, 2448, 0000, 0000, 3264, 2448,	0,	0, 3264, 2448}, // capture 
 { 3280, 2464,	  8,	  8,   3264, 2448, 3264, 2448, 0000, 0000, 3264, 2448,	0,	0, 3264, 2448}, // video 
 { 3280, 2464,	  8,    8,   3264, 2448,  816,  612, 0000, 0000,  816,  612,	0,	0,  800,  600}, //hight speed video 
 { 3280, 2464,	  360,  512, 2560, 1440, 1280,  720, 0000, 0000, 1280,  720,	0,	0, 1280,  720}};// slim video 


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor_8(0x0104, 0x01); 
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0342, imgsensor.line_length);
    write_cmos_sensor_8(0x0104, 0x00); 
}
		

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min_framelength_en=%d\n", framerate,min_framelength_en);
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	LOG_INF("frame_length =%d\n", frame_length);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
	{
		kal_uint16 realtime_fps = 0;
		kal_uint32 frame_length = 0;
	
		if(imgsensor.sensor_mode == IMGSENSOR_MODE_HIGH_SPEED_VIDEO)
		{
			if(shutter > imgsensor.min_frame_length - imgsensor_info.margin)
				shutter = imgsensor.min_frame_length - imgsensor_info.margin;
    write_cmos_sensor_8(0x0104, 0x01); 
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0202, shutter);
    write_cmos_sensor_8(0x0104, 0x00); 
			LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
			return;
		}
		spin_lock(&imgsensor_drv_lock);
		if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
			{
			imgsensor.frame_length = shutter + imgsensor_info.margin;
			}
		else
			{
			imgsensor.frame_length = imgsensor.min_frame_length;
			}
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			{
			imgsensor.frame_length = imgsensor_info.max_frame_length;
			}
		spin_unlock(&imgsensor_drv_lock);
		if (shutter < imgsensor_info.min_shutter) 
			shutter = imgsensor_info.min_shutter;
		
		if (imgsensor.autoflicker_en == KAL_TRUE) { 
			realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
			if(realtime_fps >= 297 && realtime_fps <= 305)
			{
				set_max_framerate(296,0);
				//set_dummy();
			}
			else if(realtime_fps >= 147 && realtime_fps <= 150)
			{
				set_max_framerate(146,0);	
				//set_dummy();
			}
			else{
    write_cmos_sensor_8(0x0104, 0x01); 
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor_8(0x0104, 0x00); 
			}
		} else {
			// Extend frame length
    write_cmos_sensor_8(0x0104, 0x01); 
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor_8(0x0104, 0x00); 
		}
	
		// Update Shutter
    write_cmos_sensor_8(0x0104, 0x01); 
    write_cmos_sensor(0x0202, shutter);
    write_cmos_sensor_8(0x0104, 0x00); 
 		LOG_INF("realtime_fps =%d\n", realtime_fps);
		LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
	
		//LOG_INF("frame_length = %d ", frame_length);
		
	}




/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	write_shutter(shutter);
}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	gain = gain / 2;
	
	 write_cmos_sensor_8(0x0104, 0x01);
	 write_cmos_sensor_8(0x0204,(gain>>8));
	 write_cmos_sensor_8(0x0205,(gain&0xff));
	 write_cmos_sensor_8(0x0104, 0x00);
}   /*  S5K3H5XASUBMIPI_SetGain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	
	switch (image_mirror)
    {
        case IMAGE_NORMAL: //B
            write_cmos_sensor_8(0x0101, 0x00);	//Set normal
            break;
        case IMAGE_V_MIRROR: //Gr X
            write_cmos_sensor_8(0x0101, 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR: //Gb
            write_cmos_sensor_8(0x0101, 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR: //R
            write_cmos_sensor_8(0x0101, 0x03);	//Set mirror and flip
            break;
    }

}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
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
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/

static void sensor_init(void)
{
  write_cmos_sensor(0xFCFC, 0xD000);
  write_cmos_sensor(0x6010, 0x0001);
  mdelay(10);
  write_cmos_sensor(0x6028, 0x7000);
  write_cmos_sensor(0x602a, 0x1870);
  write_cmos_sensor(0x6F12, 0x2DE9);
  write_cmos_sensor(0x6F12, 0xF84F);
  write_cmos_sensor(0x6F12, 0x9FE5);
  write_cmos_sensor(0x6F12, 0x6081);
  write_cmos_sensor(0x6F12, 0xA0E1);
  write_cmos_sensor(0x6F12, 0x0090);
  write_cmos_sensor(0x6F12, 0x98E5);
  write_cmos_sensor(0x6F12, 0x00A0);
  write_cmos_sensor(0x6F12, 0x9FE5);
  write_cmos_sensor(0x6F12, 0x5071);
  write_cmos_sensor(0x6F12, 0x9FE5);
  write_cmos_sensor(0x6F12, 0x5401);
  write_cmos_sensor(0x6F12, 0xD7E1);
  write_cmos_sensor(0x6F12, 0xBE5E);
  write_cmos_sensor(0x6F12, 0xD0E1);
  write_cmos_sensor(0x6F12, 0xBC03);
  write_cmos_sensor(0x6F12, 0xDAE5);
  write_cmos_sensor(0x6F12, 0x9D10);
  write_cmos_sensor(0x6F12, 0xDAE5);
  write_cmos_sensor(0x6F12, 0x9FB0);
  write_cmos_sensor(0x6F12, 0x40E0);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x6F12, 0x00E0);
  write_cmos_sensor(0x6F12, 0x9500);
  write_cmos_sensor(0x6F12, 0x55E1);
  write_cmos_sensor(0x6F12, 0x0B00);
  write_cmos_sensor(0x6F12, 0xA0E1);
  write_cmos_sensor(0x6F12, 0x2005);
  write_cmos_sensor(0x6F12, 0x002A);
  write_cmos_sensor(0x6F12, 0x1B00);
  write_cmos_sensor(0x6F12, 0x5BE1);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x008A);
  write_cmos_sensor(0x6F12, 0x0B00);
  write_cmos_sensor(0x6F12, 0xA0E3);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x6F12, 0xC7E1);
  write_cmos_sensor(0x6F12, 0xBEBE);
  write_cmos_sensor(0x6F12, 0x87E0);
  write_cmos_sensor(0x6F12, 0x0461);
  write_cmos_sensor(0x6F12, 0x96E5);
  write_cmos_sensor(0x6F12, 0xF800);
  write_cmos_sensor(0x6F12, 0xA0E1);
  write_cmos_sensor(0x6F12, 0x0B10);
  write_cmos_sensor(0x6F12, 0x00E0);
  write_cmos_sensor(0x6F12, 0x9500);
  write_cmos_sensor(0x6F12, 0x00EB);
  write_cmos_sensor(0x6F12, 0x4800);
  write_cmos_sensor(0x6F12, 0x84E2);
  write_cmos_sensor(0x6F12, 0x0140);
  write_cmos_sensor(0x6F12, 0x54E3);
  write_cmos_sensor(0x6F12, 0x0400);
  write_cmos_sensor(0x6F12, 0x86E5);
  write_cmos_sensor(0x6F12, 0xF800);
  write_cmos_sensor(0x6F12, 0xFF3A);
  write_cmos_sensor(0x6F12, 0xF6FF);
  write_cmos_sensor(0x6F12, 0x00EA);
  write_cmos_sensor(0x6F12, 0x0D00);
  write_cmos_sensor(0x6F12, 0xDAE5);
  write_cmos_sensor(0x6F12, 0x9E00);
  write_cmos_sensor(0x6F12, 0x50E1);
  write_cmos_sensor(0x6F12, 0x0500);
  write_cmos_sensor(0x6F12, 0x002A);
  write_cmos_sensor(0x6F12, 0x0A00);
  write_cmos_sensor(0x6F12, 0xA0E3);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x6F12, 0xC7E1);
  write_cmos_sensor(0x6F12, 0xBE0E);
  write_cmos_sensor(0x6F12, 0x87E0);
  write_cmos_sensor(0x6F12, 0x0461);
  write_cmos_sensor(0x6F12, 0x96E5);
  write_cmos_sensor(0x6F12, 0xF800);
  write_cmos_sensor(0x6F12, 0xDAE5);
  write_cmos_sensor(0x6F12, 0x9E10);
  write_cmos_sensor(0x6F12, 0x00E0);
  write_cmos_sensor(0x6F12, 0x9500);
  write_cmos_sensor(0x6F12, 0x00EB);
  write_cmos_sensor(0x6F12, 0x3900);
  write_cmos_sensor(0x6F12, 0x84E2);
  write_cmos_sensor(0x6F12, 0x0140);
  write_cmos_sensor(0x6F12, 0x54E3);
  write_cmos_sensor(0x6F12, 0x0400);
  write_cmos_sensor(0x6F12, 0x86E5);
  write_cmos_sensor(0x6F12, 0xF800);
  write_cmos_sensor(0x6F12, 0xFF3A);
  write_cmos_sensor(0x6F12, 0xF6FF);
  write_cmos_sensor(0x6F12, 0x9FE5);
  write_cmos_sensor(0x6F12, 0xC000);
  write_cmos_sensor(0x6F12, 0x87E2);
  write_cmos_sensor(0x6F12, 0xC830);
  write_cmos_sensor(0x6F12, 0x8DE5);
  write_cmos_sensor(0x6F12, 0x0030);
  write_cmos_sensor(0x6F12, 0xD0E5);
  write_cmos_sensor(0x6F12, 0xB920);
  write_cmos_sensor(0x6F12, 0xD0E1);
  write_cmos_sensor(0x6F12, 0xB618);
  write_cmos_sensor(0x6F12, 0x9FE5);
  write_cmos_sensor(0x6F12, 0xB000);
  write_cmos_sensor(0x6F12, 0xA0E1);
  write_cmos_sensor(0x6F12, 0x0930);
  write_cmos_sensor(0x6F12, 0x90E5);
  write_cmos_sensor(0x6F12, 0x28C0);
  write_cmos_sensor(0x6F12, 0x87E2);
  write_cmos_sensor(0x6F12, 0xE800);
  write_cmos_sensor(0x6F12, 0xA0E1);
  write_cmos_sensor(0x6F12, 0x0FE0);
  write_cmos_sensor(0x6F12, 0x2FE1);
  write_cmos_sensor(0x6F12, 0x1CFF);
  write_cmos_sensor(0x6F12, 0x50E3);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x001B);
  write_cmos_sensor(0x6F12, 0x2A00);
  write_cmos_sensor(0x6F12, 0x9FE5);
  write_cmos_sensor(0x6F12, 0x9400);
  write_cmos_sensor(0x6F12, 0x98E5);
  write_cmos_sensor(0x6F12, 0x0010);
  write_cmos_sensor(0x6F12, 0xD0E1);
  write_cmos_sensor(0x6F12, 0xBA05);
  write_cmos_sensor(0x6F12, 0xD1E5);
  write_cmos_sensor(0x6F12, 0x9C10);
  write_cmos_sensor(0x6F12, 0x9FE5);
  write_cmos_sensor(0x6F12, 0x8830);
  write_cmos_sensor(0x6F12, 0x50E1);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x6F12, 0xA0E3);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x002A);
  write_cmos_sensor(0x6F12, 0x1300);
  write_cmos_sensor(0x6F12, 0x00EA);
  write_cmos_sensor(0x6F12, 0x0500);
  write_cmos_sensor(0x6F12, 0xD1E1);
  write_cmos_sensor(0x6F12, 0xB210);
  write_cmos_sensor(0x6F12, 0x82E2);
  write_cmos_sensor(0x6F12, 0x0D22);
  write_cmos_sensor(0x6F12, 0xC2E1);
  write_cmos_sensor(0x6F12, 0xB010);
  write_cmos_sensor(0x6F12, 0x80E2);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x6F12, 0x50E3);
  write_cmos_sensor(0x6F12, 0x1400);
  write_cmos_sensor(0x6F12, 0x00AA);
  write_cmos_sensor(0x6F12, 0x0400);
  write_cmos_sensor(0x6F12, 0x80E0);
  write_cmos_sensor(0x6F12, 0x8010);
  write_cmos_sensor(0x6F12, 0x83E0);
  write_cmos_sensor(0x6F12, 0x8110);
  write_cmos_sensor(0x6F12, 0xD1E1);
  write_cmos_sensor(0x6F12, 0xB020);
  write_cmos_sensor(0x6F12, 0x52E3);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xFF1A);
  write_cmos_sensor(0x6F12, 0xF4FF);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0xF84F);
  write_cmos_sensor(0x6F12, 0x2FE1);
  write_cmos_sensor(0x6F12, 0x1EFF);
  write_cmos_sensor(0x6F12, 0xD1E1);
  write_cmos_sensor(0x6F12, 0xB410);
  write_cmos_sensor(0x6F12, 0x82E2);
  write_cmos_sensor(0x6F12, 0x0D22);
  write_cmos_sensor(0x6F12, 0xC2E1);
  write_cmos_sensor(0x6F12, 0xB010);
  write_cmos_sensor(0x6F12, 0x80E2);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x6F12, 0x50E3);
  write_cmos_sensor(0x6F12, 0x1400);
  write_cmos_sensor(0x6F12, 0xFFAA);
  write_cmos_sensor(0x6F12, 0xF7FF);
  write_cmos_sensor(0x6F12, 0x80E0);
  write_cmos_sensor(0x6F12, 0x8010);
  write_cmos_sensor(0x6F12, 0x83E0);
  write_cmos_sensor(0x6F12, 0x8110);
  write_cmos_sensor(0x6F12, 0xD1E1);
  write_cmos_sensor(0x6F12, 0xB020);
  write_cmos_sensor(0x6F12, 0x52E3);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xFF1A);
  write_cmos_sensor(0x6F12, 0xF4FF);
  write_cmos_sensor(0x6F12, 0xFFEA);
  write_cmos_sensor(0x6F12, 0xF1FF);
  write_cmos_sensor(0x6F12, 0x0070);
  write_cmos_sensor(0x6F12, 0x7014);
  write_cmos_sensor(0x6F12, 0x0070);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0070);
  write_cmos_sensor(0x6F12, 0x800C);
  write_cmos_sensor(0x6F12, 0x0070);
  write_cmos_sensor(0x6F12, 0x3013);
  write_cmos_sensor(0x6F12, 0x0070);
  write_cmos_sensor(0x6F12, 0xF804);
  write_cmos_sensor(0x6F12, 0x0070);
  write_cmos_sensor(0x6F12, 0x7015);
  write_cmos_sensor(0x6F12, 0x0070);
  write_cmos_sensor(0x6F12, 0x002D);
  write_cmos_sensor(0x6F12, 0x1FE5);
  write_cmos_sensor(0x6F12, 0x04F0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x20E1);
  write_cmos_sensor(0x6F12, 0x1FE5);
  write_cmos_sensor(0x6F12, 0x04F0);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x641C);
  write_cmos_sensor(0x6028, 0xD000);
  write_cmos_sensor(0x3902, 0x0002);
  write_cmos_sensor(0x3158, 0x0215);
  write_cmos_sensor(0x32B4, 0xF4B6);
  write_cmos_sensor(0x32B6, 0xF466);
  write_cmos_sensor(0x32B8, 0xF456);
  write_cmos_sensor(0x32BA, 0xF45E);
  write_cmos_sensor_8(0x32BC, 0x10);
  write_cmos_sensor_8(0x32BD, 0x00);
  write_cmos_sensor_8(0x32BE, 0x00);
  write_cmos_sensor(0x3338, 0x0214);
  write_cmos_sensor(0x6218, 0xF1D0);
  write_cmos_sensor(0x6214, 0xF9F0);
  write_cmos_sensor(0x6226, 0x0001);
  write_cmos_sensor(0xF446, 0x0029);
  write_cmos_sensor(0xF448, 0x001D);
  write_cmos_sensor(0xF440, 0x0071);
  write_cmos_sensor(0xF42E, 0x00C1);
  write_cmos_sensor(0xF42A, 0x0802);
  write_cmos_sensor(0xB0C8, 0x0044);
  write_cmos_sensor(0x6226, 0x0000);
  write_cmos_sensor(0x6218, 0xF9F0);
  write_cmos_sensor(0x34A2, 0x00D6);
  write_cmos_sensor(0x34B2, 0x01FA);
  write_cmos_sensor(0x34CA, 0x00D6);
  write_cmos_sensor(0x34DA, 0x01FA);
  write_cmos_sensor(0x3522, 0x00D6);
  write_cmos_sensor(0x3532, 0x01FA);
  write_cmos_sensor(0x3254, 0x79D3);
  write_cmos_sensor(0x3256, 0x79D3);
  write_cmos_sensor(0x3258, 0x79D3);
  write_cmos_sensor(0x325A, 0x79D3);
  write_cmos_sensor(0x325C, 0x79D3);
  write_cmos_sensor(0x325E, 0x79D3);
  write_cmos_sensor(0x357A, 0x00BD);
  write_cmos_sensor(0x32F6, 0x1110);
  write_cmos_sensor_8(0x012C, 0x60);
  write_cmos_sensor_8(0x012D, 0x4F);
  write_cmos_sensor_8(0x012E, 0x2F);
  write_cmos_sensor_8(0x012F, 0x40);
  write_cmos_sensor(0x6028, 0x7000);
  write_cmos_sensor(0x602A, 0x2D00);
  write_cmos_sensor(0x6F12, 0x30F4);
  write_cmos_sensor(0x6F12, 0xD370);
  write_cmos_sensor(0x6F12, 0xD379);
  write_cmos_sensor(0x6F12, 0x12F4);
  write_cmos_sensor(0x6F12, 0x0500);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x6F12, 0x4638);
  write_cmos_sensor(0x6F12, 0x0007);
  write_cmos_sensor(0x6F12, 0xF004);
  write_cmos_sensor(0x6F12, 0x5038);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x1002);
  write_cmos_sensor(0x6F12, 0xF838);
  write_cmos_sensor(0x6F12, 0xFAFF);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x4C38);
  write_cmos_sensor(0x6F12, 0x7805);
  write_cmos_sensor(0x6F12, 0x9C04);
  write_cmos_sensor(0x6F12, 0x78F4);
  write_cmos_sensor(0x6F12, 0x0700);
  write_cmos_sensor(0x6F12, 0x0700);
  write_cmos_sensor(0x6F12, 0x9AF4);
  write_cmos_sensor(0x6F12, 0x3100);
  write_cmos_sensor(0x6F12, 0x3100);
  write_cmos_sensor(0x6F12, 0x36F4);
  write_cmos_sensor(0x6F12, 0x0600);
  write_cmos_sensor(0x6F12, 0x0600);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6028, 0xD000);
  write_cmos_sensor(0x6226, 0x0001);
  write_cmos_sensor(0x6100, 0x0003);
  write_cmos_sensor(0x6110, 0x1CA0);
  write_cmos_sensor(0x6112, 0x1CA4);
  write_cmos_sensor(0x6150, 0x172C);
  write_cmos_sensor(0x6152, 0x1730);
  write_cmos_sensor(0x6028, 0x7000);
  write_cmos_sensor(0x602A, 0x172C);
  write_cmos_sensor(0x6F12, 0x1FE5);
  write_cmos_sensor(0x6F12, 0x04F0);
  write_cmos_sensor(0x6F12, 0x0070);
  write_cmos_sensor(0x6F12, 0x7018);
  write_cmos_sensor(0x6028, 0xD000);
  write_cmos_sensor(0x6226, 0x0000);	  	    
	  mdelay(5);
}   /*  S5K3H5XASUBMIPI_Sensor_Init  */


static void preview_setting(void)
{ 
	write_cmos_sensor_8(0x0100, 0x00);
	mdelay(10);
  write_cmos_sensor_8(0x0101, 0x00);
  write_cmos_sensor(0x0112, 0x0A0A);
  write_cmos_sensor_8(0x0114, 0x03);
  write_cmos_sensor_8(0x0120, 0x00);
  write_cmos_sensor(0x0200, 0x0BEF);
  write_cmos_sensor(0x0202, 0x09D9);
  write_cmos_sensor(0x0204, 0x0020);
  write_cmos_sensor(0x0300, 0x0002);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x008C);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x030C, 0x0006);
  write_cmos_sensor(0x030E, 0x00A5);
  write_cmos_sensor(0x0340, 0x09E2);
  write_cmos_sensor(0x0342, 0x0E68);
  write_cmos_sensor(0x32CE, 0x0094);
  write_cmos_sensor(0x32D0, 0x0024);
  write_cmos_sensor(0x0344, 0x0008);
  write_cmos_sensor(0x0346, 0x0008);
  write_cmos_sensor(0x0348, 0x0CC7);
  write_cmos_sensor(0x034A, 0x0997);
  write_cmos_sensor(0x034C, 0x0660);
  write_cmos_sensor(0x034E, 0x04C8);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0003);
  write_cmos_sensor_8(0x0900, 0x01);
  write_cmos_sensor_8(0x0901, 0x12);
  write_cmos_sensor_8(0x0902, 0x01);
  write_cmos_sensor_8(0x3011, 0x02);
  write_cmos_sensor_8(0x3293, 0x00);
  write_cmos_sensor_8(0x0100, 0x01);
}   /*  S5K3H5XASUBMIPI_Capture_Setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("yy_07\n");
	write_cmos_sensor_8(0x0100, 0x00);
	mdelay(10);
  write_cmos_sensor_8(0x0101, 0x00);
  write_cmos_sensor(0x0112, 0x0A0A);
  write_cmos_sensor_8(0x0114, 0x03);
  write_cmos_sensor_8(0x0120, 0x00);
  write_cmos_sensor(0x0200, 0x0BEF);
  write_cmos_sensor(0x0202, 0x09D9);
  write_cmos_sensor(0x0204, 0x0020);
  write_cmos_sensor(0x0300, 0x0002);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x008C);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x030C, 0x0006);
  write_cmos_sensor(0x030E, 0x00A5);
  write_cmos_sensor(0x0340, 0x09E2);
  write_cmos_sensor(0x0342, 0x0E68);
  write_cmos_sensor(0x32CE, 0x0094);
  write_cmos_sensor(0x32D0, 0x0024);
  write_cmos_sensor(0x0344, 0x0008);
  write_cmos_sensor(0x0346, 0x0008);
  write_cmos_sensor(0x0348, 0x0CC7);
  write_cmos_sensor(0x034A, 0x0997);
  write_cmos_sensor(0x034C, 0x0CC0);
  write_cmos_sensor(0x034E, 0x0990);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0001);
  write_cmos_sensor_8(0x0900, 0x01);
  write_cmos_sensor_8(0x0901, 0x11);
  write_cmos_sensor_8(0x0902, 0x01);
  write_cmos_sensor_8(0x3011, 0x01);
  write_cmos_sensor_8(0x3293, 0x00);
  write_cmos_sensor_8(0x0100, 0x01);
  mdelay(30);
}
static void normal_video_setting(kal_uint16 currefps)
{ 
	write_cmos_sensor_8(0x0100, 0x00);
	mdelay(10);
  write_cmos_sensor_8(0x0101, 0x00);
  write_cmos_sensor(0x0112, 0x0A0A);
  write_cmos_sensor_8(0x0114, 0x03);
  write_cmos_sensor_8(0x0120, 0x00);
  write_cmos_sensor(0x0200, 0x0BEF);
  write_cmos_sensor(0x0202, 0x09D9);
  write_cmos_sensor(0x0204, 0x0020);
  write_cmos_sensor(0x0300, 0x0002);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x008C);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x030C, 0x0006);
  write_cmos_sensor(0x030E, 0x00A5);
  write_cmos_sensor(0x0340, 0x09E2);
  write_cmos_sensor(0x0342, 0x0E68);
  write_cmos_sensor(0x32CE, 0x0094);
  write_cmos_sensor(0x32D0, 0x0024);
  write_cmos_sensor(0x0344, 0x0008);
  write_cmos_sensor(0x0346, 0x0008);
  write_cmos_sensor(0x0348, 0x0CC7);
  write_cmos_sensor(0x034A, 0x0997);
  write_cmos_sensor(0x034C, 0x0CC0);
  write_cmos_sensor(0x034E, 0x0990);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0001);
  write_cmos_sensor_8(0x0900, 0x01);
  write_cmos_sensor_8(0x0901, 0x11);
  write_cmos_sensor_8(0x0902, 0x01);
  write_cmos_sensor_8(0x3011, 0x01);
  write_cmos_sensor_8(0x3293, 0x00);
  write_cmos_sensor_8(0x0100, 0x01);
  mdelay(30);
}
static void hs_video_setting(void) 
{
	write_cmos_sensor_8(0x0100, 0x00);
	mdelay(10);
  write_cmos_sensor_8(0x0101, 0x00);
  write_cmos_sensor(0x0112, 0x0A0A);
  write_cmos_sensor_8(0x0114, 0x03);
  write_cmos_sensor_8(0x0120, 0x00);
  write_cmos_sensor(0x0200, 0x0BEF);
  write_cmos_sensor(0x0202, 0x09D9);
  write_cmos_sensor(0x0204, 0x0020);
  write_cmos_sensor(0x0300, 0x0002);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x008C);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x030C, 0x0006);
  write_cmos_sensor(0x030E, 0x00A5);
  write_cmos_sensor(0x0340, 0x0278);
  write_cmos_sensor(0x0342, 0x0E68);
  write_cmos_sensor(0x32CE, 0x0094);
  write_cmos_sensor(0x32D0, 0x0024);
  write_cmos_sensor(0x0344, 0x0008);
  write_cmos_sensor(0x0346, 0x0008);
  write_cmos_sensor(0x0348, 0x0CC7);
  write_cmos_sensor(0x034A, 0x0997);
  write_cmos_sensor(0x034C, 0x0330);
  write_cmos_sensor(0x034E, 0x0264);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0003);
  write_cmos_sensor_8(0x0900, 0x01);
  write_cmos_sensor_8(0x0901, 0x14);
  write_cmos_sensor_8(0x0902, 0x01);
  write_cmos_sensor_8(0x3011, 0x04);
  write_cmos_sensor_8(0x3293, 0x00);
  write_cmos_sensor_8(0x0100, 0x01);
}

static void slim_video_setting(void)
{
	write_cmos_sensor_8(0x0100, 0x00);
	mdelay(10);
  write_cmos_sensor_8(0x0101, 0x00);
  write_cmos_sensor(0x0112, 0x0A0A);
  write_cmos_sensor_8(0x0114, 0x03);
  write_cmos_sensor_8(0x0120, 0x00);
  write_cmos_sensor(0x0200, 0x0BEF);
  write_cmos_sensor(0x0202, 0x09D9);
  write_cmos_sensor(0x0204, 0x0020);
  write_cmos_sensor(0x0300, 0x0002);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x008C);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x030C, 0x0006);
  write_cmos_sensor(0x030E, 0x00A5);
  write_cmos_sensor(0x0340, 0x09E2);
  write_cmos_sensor(0x0342, 0x0E68);
  write_cmos_sensor(0x32CE, 0x0094);
  write_cmos_sensor(0x32D0, 0x0024);
  write_cmos_sensor(0x0344, 0x0168);
  write_cmos_sensor(0x0346, 0x0200);
  write_cmos_sensor(0x0348, 0x0B67);
  write_cmos_sensor(0x034A, 0x079F);
  write_cmos_sensor(0x034C, 0x0500);
  write_cmos_sensor(0x034E, 0x02D0);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0003);
  write_cmos_sensor_8(0x0900, 0x01);
  write_cmos_sensor_8(0x0901, 0x12);
  write_cmos_sensor_8(0x0902, 0x01);
  write_cmos_sensor_8(0x3011, 0x02);
  write_cmos_sensor_8(0x3293, 0x00);
  write_cmos_sensor_8(0x0100, 0x01);
}


/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
    int  retry = 1;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001)) + 1;
			if (*sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				break;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x,0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}



/*************************************************************************
* FUNCTION
*	open
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
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 1;
	kal_uint16 sensor_id = 0; 
	LOG_INF("PLATFORM:MT6595,MIPI 2LANE\n");
	LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001)) + 1;
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x,0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x,0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}		 
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	


	
	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
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
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
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
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor.current_fps;
	//imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
    set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}   /*  S5K3H5XASUBMIPIPreview   */

/*************************************************************************
* FUNCTION
*	capture
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
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		//imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		//imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	//imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
    set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}   /*  S5K3H5XASUBMIPIPreview   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
    set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}   /*  S5K3H5XASUBMIPIPreview   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
    set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}   /*  S5K3H5XASUBMIPIPreview   */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
  sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
	{
		LOG_INF("framerate = %d\n ", framerate);
		// SetVideoMode Function should fix framerate
		if (framerate == 0)
			// Dynamic frame rate
			return ERROR_NONE;
		spin_lock(&imgsensor_drv_lock);
		if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
			imgsensor.current_fps = 296;
		else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
			imgsensor.current_fps = 146;
		else
			imgsensor.current_fps = framerate;
		spin_unlock(&imgsensor_drv_lock);
		set_max_framerate(imgsensor.current_fps,1);
	
		return ERROR_NONE;
	}


static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_int16 dummyLine;
	kal_uint32 lineLength,frameHeight;
  
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);
				if(framerate == 0)
				return ERROR_NONE;
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frameHeight = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = frameHeight - imgsensor_info.pre.framelength;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;
			imgsensor.frame_length =imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frameHeight = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = frameHeight - imgsensor_info.normal_video.framelength;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);

			//set_dummy();			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			frameHeight = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = frameHeight - imgsensor_info.cap.framelength;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;
			imgsensor.frame_length =imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frameHeight = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = frameHeight - imgsensor_info.hs_video.framelength;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frameHeight = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = frameHeight - imgsensor_info.slim_video.framelength;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;			
			imgsensor.frame_length =imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
		default:  //coding with  preview scenario by default
			frameHeight = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = frameHeight - imgsensor_info.pre.framelength;
			if (imgsensor.dummy_line < 0)
				imgsensor.dummy_line = 0;
			imgsensor.frame_length =imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();	
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if(enable) 	 
		write_cmos_sensor(0x0601, 0x02);
	else		  
		write_cmos_sensor(0x0601, 0x00);  

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
	unsigned long long *feature_data=(unsigned long long *) feature_para;
	unsigned long long *feature_return_para=(unsigned long long *) feature_para;
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	//LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:	 
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;		   
		case SENSOR_FEATURE_SET_ESHUTTER:
			set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:		
			set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			set_video_mode(*feature_data);
			break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32); 
			break; 
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			//get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data_32, (MUINT32 *)(*(feature_data_32+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing			 
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;							 
			break;				
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);		
			break;
//		case SENSOR_FEATURE_SET_HDR:
	//		LOG_INF("ihdr enable :%d\n", *feature_data_16);
	//		spin_lock(&imgsensor_drv_lock);
//			imgsensor.ihdr_en = *feature_data_16;
	//		spin_unlock(&imgsensor_drv_lock);		
//			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
			//LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data_32);
			//wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(*(feature_data_32+1));
			wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;	  
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
		default:
			break;
	}
  
	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5K3H5XASUB_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	S5K3H5XASUB_MIPI_RAW_SensorInit	*/
