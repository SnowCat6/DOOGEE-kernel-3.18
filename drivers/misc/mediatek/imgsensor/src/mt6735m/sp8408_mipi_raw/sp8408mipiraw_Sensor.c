/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 SP8408mipiraw_sensor.c
 *
 * Project:
 * --------
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *	sp-liuxin 
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
//#include <asm/system.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "sp8408mipiraw_Sensor.h"
#include "sp8408mipiraw_Camera_Sensor_para.h"
#include "sp8408mipiraw_CameraCustomized.h"

#define PFX "SP8408R2A"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);


static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = SP8408MIPI_SENSOR_ID,		//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value = 0xd6d43c1f,		//checksum value for Camera Auto Test
	
	.pre = {
		.pclk = 108750000,				//record different mode's pclk
		.linelength  = 3450,				//record different mode's linelength
		.framelength = 1256,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1600,		//record different mode's width of grabwindow
		.grabwindow_height = 1200,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 23,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 72170000,
		.linelength  = 3440,
		.framelength = 2492,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.cap1 = {							//capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 72170000,
		.linelength  = 3440,
		.framelength = 2492,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 240,
	},
	.normal_video = {
		.pclk = 108750000,				//record different mode's pclk
		.linelength  = 3450,
		.framelength = 1256,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 144000000,				//record different mode's pclk
		.linelength  = 2566,				//record different mode's linelength
		.framelength = 1872,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 3264,		//record different mode's width of grabwindow
		.grabwindow_height = 2448,		//record different mode's height of grabwindow
		/*	 following for  MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 30,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.slim_video = {
		.pclk = 72000000,				//record different mode's pclk
		.linelength  = 1928,				//record different mode's linelength
		.framelength = 1244,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 30,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	

	},
	.margin = 6,			//sensor framelength & shutter margin
	.min_shutter = 4,		//min shutter
	.max_frame_length = 0xffff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 3,	  //support sensor mode num ,don't support Slow motion
	
	.cap_delay_frame = 3,		//enter capture delay frame num
	.pre_delay_frame = 3, 		//enter preview delay frame num
	.video_delay_frame = 3,		//enter video delay frame num
	.hs_video_delay_frame = 3,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,//enter slim video delay frame num
	
	.isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_2_LANE,//mipi lane num
	.i2c_addr_table = {0x6c, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff  //2015-1-14 sp_miao
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x00bb,					//current shutter
	.gain = 0x200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x6c,//record current sensor's i2c write id  //2015-1-14 sp_miao
};


/* Sensor output window information*/
/*
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 3296, 2528,	  12,	12, 3272, 2456, 3264, 2452,   04,	02, 1632, 1224,   0,	0, 1632, 1224}, // Preview 
 { 3296, 2528,	         12,	12, 3272, 2468, 3264, 2452,   04,   02, 3264, 2448,   0,	0, 3264, 2448}, // capture 
 { 3296, 2528,	  12,	12, 3272, 2456, 3264, 2452,   04,	02, 3264, 2448,   0,	0, 3264, 2448}, // video 
 { 3296, 2528,	  12,	12, 3272, 2456, 3264, 2452,   04,	02, 3264, 2448,   0,	0, 3264, 2448}, //hight speed video 
 { 3296, 2528,	  12,	12, 3272, 2456, 3264, 2452,   04,	02, 3264, 2448,   0,	0, 1632, 1224}};// slim video 
 */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 3296, 2480,	  12,	12, 3272, 2456, 3264, 2452,   4,	2, 1632, 1224,	 0, 0, 1632, 1224}, // Preview 
 { 3296, 2480,	  12,	12, 3272, 2456, 3264, 2452,   4,	2, 3264, 2448,	 0, 0, 3264, 2448}, // capture 
 { 3296, 2480,	  12,	12, 3272, 2456, 3264, 2452,   4,	2, 3264, 2448,	 0, 0, 3264, 2448}, // video 
 { 3296, 2480,	  12,	12, 3272, 2456, 3264, 2452,   4,	2, 3264, 2448,	 0, 0, 3264, 2448}, //hight speed video 
 { 3296, 2480,	  12,	12, 3272, 2456, 3264, 2452,   4,	2, 1632, 1224,	 0, 0, 1632, 1224}};// slim video 

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
   // LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
  
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

   // LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	
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


static  kal_uint32 tmp_frame_length;
static  kal_uint16 tmp_shutter;
static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	   
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    // Framelength should be an even number
    shutter = (shutter >> 1) << 1;
    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
	} else {
			// Extend frame length
			write_cmos_sensor(0x0340, (imgsensor.frame_length >>8) & 0xFF);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	}

	
	tmp_frame_length = imgsensor.frame_length;
	tmp_shutter = shutter;
	
    //LOG_INF("shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
	
}	/*	write_shutter  */



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
      LOG_INF("[SP8408MIPI_SetShutter]%s():shutter=%d\n",__FUNCTION__,shutter);
	if (shutter < 1)
          shutter = 1; 
	else if(shutter > 0xffff)
	   shutter = 0xffff;
	unsigned long flags;
	spin_lock(&imgsensor_drv_lock);
       imgsensor.shutter = shutter;	
	spin_unlock(&imgsensor_drv_lock);
       write_shutter(shutter);
	LOG_INF("[SP8408MIPI]exit SP8408MIPIGetSensorID function\n");
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	
	reg_gain = gain*2;
	//reg_gain = reg_gain & 0xFFFF;
	return (kal_uint16)reg_gain;
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
	LOG_INF("set_gain %d \n", gain);

	kal_uint16 reg_gain;
	if(gain >= BASEGAIN && gain <16*BASEGAIN)
   	 {   
   	    	 reg_gain =( 0x80 * gain/BASEGAIN) ;        //change mtk gain base to aptina gain base
 
		write_cmos_sensor(0x0104, 0x01); 
		write_cmos_sensor(0x0340, (tmp_frame_length >>8) & 0xFF);
		write_cmos_sensor(0x0341, tmp_frame_length & 0xFF);	  
		write_cmos_sensor(0x0202, (tmp_shutter >> 8) & 0xFF);
		write_cmos_sensor(0x0203, tmp_shutter  & 0xFF);		
		write_cmos_sensor(0x234, (char)((reg_gain>>8) & 0xFF));
	       write_cmos_sensor(0x235, (char)(reg_gain & 0xFF)); 			 			 
		write_cmos_sensor(0x0104, 0x00); 
   	 }	
   	 else
   	    	 LOG_INF("error gain setting");
	
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	if (imgsensor.ihdr_en) {
		
		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)		
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;
			
			
		// Extend frame length first
		write_cmos_sensor(0x0104, 0x01); 
		write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x0341,  imgsensor.frame_length & 0xFF);	  


		write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
		write_cmos_sensor(0x0203, le  & 0xFF);     

		write_cmos_sensor(0x0234, ((se>>8) & 0xFF));
		write_cmos_sensor(0x0235, (se& 0xFF));  
		write_cmos_sensor(0x0104, 0x00); 

		set_gain(gain);
	}

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
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x0101, 0x03);	//Set normal
		break;
		
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0101,  0x01);	//Set mirror
		break;
		
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0101,  0x02);	//Set flip
		break;
		
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0101, 0x00);	//Set mirror and flip
		break;
		
		default:
			
	
			LOG_INF("Error image_mirror setting\n");
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
static void r1asensor_init(void)
{
	LOG_INF("E\n");
	//SP8408 R1A key initial setting for 4lane
	

}
static void sensor_init(void)
{
	LOG_INF("E\n");
	//sensor_init

write_cmos_sensor(0x0103,0x0000);//, wm
write_cmos_sensor(0x0101,0x0003);//, wm  
write_cmos_sensor(0x4136,0x0018);//, EXTCLK_FRQ_MHZ[15:8];
write_cmos_sensor(0x4137,0x0000);//, EXTCLK_FRQ_MHZ[7:0];
write_cmos_sensor(0x3094,0x0001);//, LP_MODE[0];
write_cmos_sensor(0x0233,0x0001);//, Reserved;
write_cmos_sensor(0x4B06,0x0001);//, Reserved;
write_cmos_sensor(0x4B07,0x0001);//, Reserved;
write_cmos_sensor(0x3028,0x0001);//, Reserved;
write_cmos_sensor(0x3032,0x0014);//, Reserved;
write_cmos_sensor(0x305C,0x000C);//, Reserved;
write_cmos_sensor(0x306D,0x000A);//, Reserved;
write_cmos_sensor(0x3071,0x00FA);//, Reserved;
write_cmos_sensor(0x307E,0x000A);//, Reserved;
write_cmos_sensor(0x307F,0x00FC);//, Reserved;
write_cmos_sensor(0x3091,0x0004);//, Reserved;
write_cmos_sensor(0x3092,0x0060);//, Reserved;
write_cmos_sensor(0x3096,0x00C0);//, Reserved;
write_cmos_sensor(0x3139,0x0006);//, Reserved;
write_cmos_sensor(0x313A,0x0006);//, Reserved;
write_cmos_sensor(0x313B,0x0004);//, Reserved;
write_cmos_sensor(0x3143,0x0002);//, Reserved;
write_cmos_sensor(0x314F,0x000E);//, Reserved;
write_cmos_sensor(0x3169,0x0099);//, Reserved;wm
write_cmos_sensor(0x316A,0x0099);//, Reserved;wm
write_cmos_sensor(0x3171,0x0005);//, Reserved;
write_cmos_sensor(0x31A1,0x00A7);//, Reserved;
write_cmos_sensor(0x31A2,0x009C);//, Reserved;
write_cmos_sensor(0x31A3,0x008F);//, Reserved;
write_cmos_sensor(0x31A4,0x0075);//, Reserved;
write_cmos_sensor(0x31A5,0x00EE);//, Reserved;
write_cmos_sensor(0x31A6,0x00EA);//, Reserved;
write_cmos_sensor(0x31A7,0x00E4);//, Reserved;
write_cmos_sensor(0x31A8,0x00E4);//, Reserved;
write_cmos_sensor(0x31DF,0x0005);//, Reserved;
write_cmos_sensor(0x31EC,0x001B);//, Reserved;
write_cmos_sensor(0x31ED,0x001B);//, Reserved;
write_cmos_sensor(0x31EE,0x001B);//, Reserved;
write_cmos_sensor(0x31F0,0x001B);//, Reserved;
write_cmos_sensor(0x31F1,0x001B);//, Reserved;
write_cmos_sensor(0x31F2,0x001B);//, Reserved;
write_cmos_sensor(0x3204,0x003F);//, Reserved;wm
write_cmos_sensor(0x3205,0x0003);//, Reserved;wm
write_cmos_sensor(0x3210,0x0001);//, Reserved;wm
write_cmos_sensor(0x3212,0x0000);//, Reserved;wm  01
write_cmos_sensor(0x3216,0x0068);//, Reserved;
write_cmos_sensor(0x3217,0x0058);//, Reserved;
write_cmos_sensor(0x3218,0x0058);//, Reserved;
write_cmos_sensor(0x321A,0x0068);//, Reserved;
write_cmos_sensor(0x321B,0x0060);//, Reserved;
write_cmos_sensor(0x3238,0x0003);//, Reserved;wm
write_cmos_sensor(0x3239,0x0003);//, Reserved;wm
write_cmos_sensor(0x323A,0x0005);//, Reserved;wm
write_cmos_sensor(0x323B,0x0006);//, -/-/-/-/-/-/LSC_EN/SHD_GRID_EN;
write_cmos_sensor(0x3243,0x0000);//, -/-/-/-/-/-/LSC_EN/SHD_GRID_EN;
write_cmos_sensor(0x3244,0x0008);//, Reserved;
write_cmos_sensor(0x3245,0x0001);//, Reserved;
write_cmos_sensor(0x3307,0x0019);//, Reserved;
write_cmos_sensor(0x3308,0x0019);//, Reserved;
write_cmos_sensor(0x3320,0x0001);//, Reserved;
write_cmos_sensor(0x3326,0x0015);//, Reserved;
write_cmos_sensor(0x3327,0x000D);//, Reserved;
write_cmos_sensor(0x3328,0x0001);//, Reserved;
write_cmos_sensor(0x3380,0x0001);//, Reserved;
write_cmos_sensor(0x3394,0x0023);//, Reserved;
write_cmos_sensor(0x339E,0x0007);//, Reserved;
write_cmos_sensor(0x3424,0x0000);//, Reserved;
write_cmos_sensor(0x343C,0x0001);//, Reserved;
write_cmos_sensor(0x35E0,0x00B0);//, Reserved;
write_cmos_sensor(0x35E1,0x0090);//, Reserved;
write_cmos_sensor(0x35E2,0x0088);//, Reserved;
write_cmos_sensor(0x35E3,0x00B0);//, Reserved;
write_cmos_sensor(0x35E4,0x0090);//, Reserved;
write_cmos_sensor(0x35E5,0x0008);//, Reserved;
write_cmos_sensor(0x3398,0x0004);//, Reserved;
write_cmos_sensor(0x343A,0x0010);//, Reserved;
write_cmos_sensor(0x339A,0x0022);//, Reserved;
write_cmos_sensor(0x33B4,0x0000);//, Reserved;
write_cmos_sensor(0x3393,0x0001);//, Reserved;
write_cmos_sensor(0x33B3,0x006E);//, Reserved;
write_cmos_sensor(0x3433,0x0006);//, Reserved;
write_cmos_sensor(0x3433,0x0000);//, Reserved;
write_cmos_sensor(0x33B3,0x0000);//, Reserved;
write_cmos_sensor(0x3393,0x0003);//, Reserved;
write_cmos_sensor(0x33B4,0x0003);//, Reserved;
write_cmos_sensor(0x343A,0x0000);//, Reserved;
write_cmos_sensor(0x339A,0x0000);//, Reserved;
write_cmos_sensor(0x3398,0x0000);//, Reserved;
write_cmos_sensor(0x0104,0x0000);
write_cmos_sensor(0x3386,0x0000);  //wm
//write_cmos_sensor(0x0601,0x0002);//, Reserved;
//write_cmos_sensor(0x0100,0x0000);//, MODE_SELECT;

write_cmos_sensor(0x0202,0x0009);//, Reserved;wm
write_cmos_sensor(0x0203,0x00F0);//, Reserved;wm
write_cmos_sensor(0x0234,0x0000);//, Reserved;wm
write_cmos_sensor(0x0235,0x0080);//, Reserved;wm


}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF("E\n");
//preview settint
//1296x972

write_cmos_sensor(0x0112,0x000a);//,CSI_DATA_FORMAT[15:8];
write_cmos_sensor(0x0113,0x000a);//,CSI_DATA_FORMAT[7:0]; 
write_cmos_sensor(0x0114,0x0001);//,CSI_LANE_MODE[1:0];
write_cmos_sensor(0x4136,0x0018);//,EXTCLK_FRWQ_MHZ[15:8];
write_cmos_sensor(0x4137,0x0000);//,EXTCLK_FRWQ_MHZ[7:0];
write_cmos_sensor(0x0820,0x000A);//,MSB_LBRATE[31:24];
write_cmos_sensor(0x0821,0x0032);//,MSB_LBRATE[23:16];
write_cmos_sensor(0x0822,0x0000);//,MSB_LBRATE[15:8];
write_cmos_sensor(0x0823,0x0000);//,MSB_LBRATE[7:0];
write_cmos_sensor(0x0301,0x000c);//,-/-/-/VT_PIX_CLK_DIV[4:0];
write_cmos_sensor(0x0303,0x0002);//,-/-/-/-/VT_SYS_CLK_DIV[3:0];
write_cmos_sensor(0x0305,0x0004);//,-/-/-/-/PRE_PLL_CLK_DIV[3:0];
write_cmos_sensor(0x0306,0x0001);//,-/-/-/-/-/PLL_MULTIPLIER[10:8];
write_cmos_sensor(0x0307,0x00B3);//,PLL_MULTIPLIER[7:0];
write_cmos_sensor(0x030B,0x0004);//,-/-/OP_SYS_CLK_DIV[5:0];
write_cmos_sensor(0x034C,0x0006);//,X_OUTPUT_SIZE[15:8];
write_cmos_sensor(0x034D,0x0068);//,X_OUTPUT_SIZE[7:0];
write_cmos_sensor(0x034E,0x0004);//,Y_OUTPUT_SIZE[15:8];
write_cmos_sensor(0x034F,0x00ce);//,Y_OUTPUT_SIZE[7:0];
write_cmos_sensor(0x0340,0x0004);//,FR_LENGTH_LINES[15:8];
write_cmos_sensor(0x0341,0x00eb);//,FR_LENGTH_LINES[7:0];
write_cmos_sensor(0x0342,0x000d);//,LINE_LENGTH_PCK[15:8];
write_cmos_sensor(0x0343,0x007a);//,LINE_LENGTH_PCK[7:0];
write_cmos_sensor(0x0344,0x0000);//,X_ADDR_START[15:8];
write_cmos_sensor(0x0345,0x0000);//,X_ADDR_START[7:0];
write_cmos_sensor(0x0346,0x0000);//,Y_ADDR_START[15:8];
write_cmos_sensor(0x0347,0x0000);//,Y_ADDR_START[7:0];
write_cmos_sensor(0x0348,0x000c);//,X_ADDR_END[15:8];
write_cmos_sensor(0x0349,0x00cf);//,X_ADDR_END[7:0];
write_cmos_sensor(0x034A,0x0009);//,Y_ADDR_END[15:8];
write_cmos_sensor(0x034B,0x009f);//,Y_ADDR_END[7:0];
write_cmos_sensor(0x0408,0x0000);//,DCROP_XOFS[15:8];
write_cmos_sensor(0x0409,0x0000);//,DCROP_XOFS[7:0];
write_cmos_sensor(0x040A,0x0000);//,DCROP_YOFS[15:8];
write_cmos_sensor(0x040B,0x0002);//,DCROP_YOFS[7:0];
write_cmos_sensor(0x040C,0x0006);//,DCROP_WIDTH[15:8];
write_cmos_sensor(0x040D,0x0068);//,DCROP_WIDTH[7:0];
write_cmos_sensor(0x040E,0x0004);//,DCROP_HIGT[15:8];
write_cmos_sensor(0x040F,0x00d0);//,DCROP_HIGT[7:0];
write_cmos_sensor(0x0900,0x0001);//,-/-/-/-/-/-/-/BINNING_MODE;
write_cmos_sensor(0x0901,0x0022);//,BINNING_TYPE[7:0];
write_cmos_sensor(0x0902,0x0000);//,-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
write_cmos_sensor(0x4220,0x0000);//,-/-/-/HDR_MODE[4:0];
write_cmos_sensor(0x4222,0x0001);//,HDR_RATIO[7:0];
write_cmos_sensor(0x3380,0x0001);//,HDR_MODE2[0]
write_cmos_sensor(0x0100,0x0001);//,-/-/-/-/-/-/-/MODE_SELECT;

}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
write_cmos_sensor(0x0112,0x000a);//,CSI_DATA_FORMAT[15:8];
write_cmos_sensor(0x0113,0x000a);//,CSI_DATA_FORMAT[7:0]; 
write_cmos_sensor(0x0114,0x0001);//,CSI_LANE_MODE[1:0];
write_cmos_sensor(0x4136,0x0018);//,EXTCLK_FRWQ_MHZ[15:8];
write_cmos_sensor(0x4137,0x0000);//,EXTCLK_FRWQ_MHZ[7:0];
write_cmos_sensor(0x0820,0x0003);//,MSB_LBRATE[31:24];
write_cmos_sensor(0x0821,0x0062);//,MSB_LBRATE[23:16];
write_cmos_sensor(0x0822,0x0000);//,MSB_LBRATE[15:8];
write_cmos_sensor(0x0823,0x0000);//,MSB_LBRATE[7:0];
write_cmos_sensor(0x0301,0x000c);//,-/-/-/VT_PIX_CLK_DIV[4:0];
write_cmos_sensor(0x0303,0x0003);//,-/-/-/-/VT_SYS_CLK_DIV[3:0];
write_cmos_sensor(0x0305,0x0004);//,-/-/-/-/PRE_PLL_CLK_DIV[3:0];
write_cmos_sensor(0x0306,0x0001);//,-/-/-/-/-/PLL_MULTIPLIER[10:8];
write_cmos_sensor(0x0307,0x00b1);//,PLL_MULTIPLIER[7:0];
write_cmos_sensor(0x030B,0x0003);//,-/-/OP_SYS_CLK_DIV[5:0];
write_cmos_sensor(0x034C,0x000c);//,X_OUTPUT_SIZE[15:8];
write_cmos_sensor(0x034D,0x00d0);//,X_OUTPUT_SIZE[7:0];
write_cmos_sensor(0x034E,0x0009);//,Y_OUTPUT_SIZE[15:8];
write_cmos_sensor(0x034F,0x009e);//,Y_OUTPUT_SIZE[7:0];
write_cmos_sensor(0x0340,0x0009);//,FR_LENGTH_LINES[15:8];
write_cmos_sensor(0x0341,0x00bc);//,FR_LENGTH_LINES[7:0];
write_cmos_sensor(0x0342,0x000d);//,LINE_LENGTH_PCK[15:8];
write_cmos_sensor(0x0343,0x0070);//,LINE_LENGTH_PCK[7:0];
write_cmos_sensor(0x0344,0x0000);//,X_ADDR_START[15:8];
write_cmos_sensor(0x0345,0x0000);//,X_ADDR_START[7:0];
write_cmos_sensor(0x0346,0x0000);//,Y_ADDR_START[15:8];
write_cmos_sensor(0x0347,0x0000);//,Y_ADDR_START[7:0];
write_cmos_sensor(0x0348,0x000c);//,X_ADDR_END[15:8];
write_cmos_sensor(0x0349,0x00cf);//,X_ADDR_END[7:0];
write_cmos_sensor(0x034A,0x0009);//,Y_ADDR_END[15:8];
write_cmos_sensor(0x034B,0x009f);//,Y_ADDR_END[7:0];
write_cmos_sensor(0x0408,0x0000);//,DCROP_XOFS[15:8];
write_cmos_sensor(0x0409,0x0000);//,DCROP_XOFS[7:0];
write_cmos_sensor(0x040A,0x0000);//,DCROP_YOFS[15:8];
write_cmos_sensor(0x040B,0x0002);//,DCROP_YOFS[7:0];
write_cmos_sensor(0x040C,0x000c);//,DCROP_WIDTH[15:8];
write_cmos_sensor(0x040D,0x00d0);//,DCROP_WIDTH[7:0];
write_cmos_sensor(0x040E,0x0009);//,DCROP_HIGT[15:8];
write_cmos_sensor(0x040F,0x00a0);//,DCROP_HIGT[7:0];
write_cmos_sensor(0x0900,0x0000);//,-/-/-/-/-/-/-/BINNING_MODE;
write_cmos_sensor(0x0901,0x0000);//,BINNING_TYPE[7:0];
write_cmos_sensor(0x0902,0x0000);//,-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
write_cmos_sensor(0x4220,0x0000);//,-/-/-/HDR_MODE[4:0];
write_cmos_sensor(0x4222,0x0001);//,HDR_RATIO[7:0];
write_cmos_sensor(0x3380,0x0001);//,HDR_MODE2[0]
write_cmos_sensor(0x0100,0x0001);//,-/-/-/-/-/-/-/MODE_SELECT;
	LOG_INF("E! currefps:%d\n",currefps);
		
}

static void normal_video_setting(kal_uint16 currefps)
{
write_cmos_sensor(0x0112,0x000a);//,CSI_DATA_FORMAT[15:8];
write_cmos_sensor(0x0113,0x000a);//,CSI_DATA_FORMAT[7:0]; 
write_cmos_sensor(0x0114,0x0001);//,CSI_LANE_MODE[1:0];3
write_cmos_sensor(0x4136,0x0018);//,EXTCLK_FRWQ_MHZ[15:8];
write_cmos_sensor(0x4137,0x0000);//,EXTCLK_FRWQ_MHZ[7:0];
write_cmos_sensor(0x0820,0x000A);//,MSB_LBRATE[31:24];
write_cmos_sensor(0x0821,0x0032);//,MSB_LBRATE[23:16];
write_cmos_sensor(0x0822,0x0000);//,MSB_LBRATE[15:8];
write_cmos_sensor(0x0823,0x0000);//,MSB_LBRATE[7:0];
write_cmos_sensor(0x0301,0x000c);//,-/-/-/VT_PIX_CLK_DIV[4:0];
write_cmos_sensor(0x0303,0x0002);//,-/-/-/-/VT_SYS_CLK_DIV[3:0];
write_cmos_sensor(0x0305,0x0004);//,-/-/-/-/PRE_PLL_CLK_DIV[3:0];
write_cmos_sensor(0x0306,0x0001);//,-/-/-/-/-/PLL_MULTIPLIER[10:8];
write_cmos_sensor(0x0307,0x00B3);//,PLL_MULTIPLIER[7:0];
write_cmos_sensor(0x030B,0x0004);//,-/-/OP_SYS_CLK_DIV[5:0];
write_cmos_sensor(0x034C,0x0006);//,X_OUTPUT_SIZE[15:8];
write_cmos_sensor(0x034D,0x0068);//,X_OUTPUT_SIZE[7:0];
write_cmos_sensor(0x034E,0x0004);//,Y_OUTPUT_SIZE[15:8];
write_cmos_sensor(0x034F,0x00ce);//,Y_OUTPUT_SIZE[7:0];
write_cmos_sensor(0x0340,0x0004);//,FR_LENGTH_LINES[15:8];
write_cmos_sensor(0x0341,0x00e8);//,FR_LENGTH_LINES[7:0];
write_cmos_sensor(0x0342,0x000d);//,LINE_LENGTH_PCK[15:8];
write_cmos_sensor(0x0343,0x007a);//,LINE_LENGTH_PCK[7:0];
write_cmos_sensor(0x0344,0x0000);//,X_ADDR_START[15:8];
write_cmos_sensor(0x0345,0x0000);//,X_ADDR_START[7:0];
write_cmos_sensor(0x0346,0x0000);//,Y_ADDR_START[15:8];
write_cmos_sensor(0x0347,0x0000);//,Y_ADDR_START[7:0];
write_cmos_sensor(0x0348,0x000c);//,X_ADDR_END[15:8];
write_cmos_sensor(0x0349,0x00cf);//,X_ADDR_END[7:0];
write_cmos_sensor(0x034A,0x0009);//,Y_ADDR_END[15:8];
write_cmos_sensor(0x034B,0x009f);//,Y_ADDR_END[7:0];
write_cmos_sensor(0x0408,0x0000);//,DCROP_XOFS[15:8];
write_cmos_sensor(0x0409,0x0000);//,DCROP_XOFS[7:0];
write_cmos_sensor(0x040A,0x0000);//,DCROP_YOFS[15:8];
write_cmos_sensor(0x040B,0x0002);//,DCROP_YOFS[7:0];
write_cmos_sensor(0x040C,0x0006);//,DCROP_WIDTH[15:8];
write_cmos_sensor(0x040D,0x0068);//,DCROP_WIDTH[7:0];
write_cmos_sensor(0x040E,0x0004);//,DCROP_HIGT[15:8];
write_cmos_sensor(0x040F,0x00d0);//,DCROP_HIGT[7:0];
write_cmos_sensor(0x0900,0x0001);//,-/-/-/-/-/-/-/BINNING_MODE;
write_cmos_sensor(0x0901,0x0022);//,BINNING_TYPE[7:0];
write_cmos_sensor(0x0902,0x0000);//,-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
write_cmos_sensor(0x4220,0x0000);//,-/-/-/HDR_MODE[4:0];
write_cmos_sensor(0x4222,0x0001);//,HDR_RATIO[7:0];
write_cmos_sensor(0x3380,0x0001);//,HDR_MODE2[0]
write_cmos_sensor(0x0100,0x0001);//,-/-/-/-/-/-/-/MODE_SELECT;

	LOG_INF("E! currefps:%d\n",currefps);

}
static void hs_video_setting(void)
{
	LOG_INF("E\n");
	capture_setting(300);
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
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
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
			
			if (*sensor_id == imgsensor_info.sensor_id) {
				//2015-1-14 sp_miao
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				return ERROR_NONE;
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
#define SP8408MIPI_USE_OTP
#ifdef SP8408MIPI_USE_OTP
static void ReadOTP(int page, UINT8 *rbuf){
	int i;
	write_cmos_sensor( 0x4A00, 0x01);
	write_cmos_sensor( 0x4A02, page);
	write_cmos_sensor( 0x4A00, 0x81);
	while((read_cmos_sensor( 0x4A00) & 0x80) == 0x80) msleep(1);
	for(i=0;i<64;i++) rbuf[i]=read_cmos_sensor( 0x4A04+i);
	write_cmos_sensor( 0x4A00, 0x00);
}


static void ReadParam_012Page(UINT8 page, UINT8 *pbuf){
	int i;
	UINT8 rbuf[64];
	ReadOTP(page, rbuf);
	for(i=0;i<32;i++) pbuf[i]=rbuf[i] | rbuf[32+i];
}

static void SP8408MIPI_update_lenc(void)
{
	UINT8 sp8408_buf_3[64];
	UINT8 sp8408_buf_4[64];
	UINT8 sp8408_buf_5[64];
	UINT8 sp8408_buf_6[64];
	UINT8 sp8408_buf_34[64];
	UINT8 sp8408_buf_56[64];

	ReadOTP(3,sp8408_buf_3);
	ReadOTP(4,sp8408_buf_4);
	ReadOTP(5,sp8408_buf_5);
	ReadOTP(6,sp8408_buf_6);
	kal_uint16 i;
	kal_uint16 checksum34 = 0;
	kal_uint16 checksum56 = 0;
	for(i = 0; i < 64; i++)
	{
		 sp8408_buf_34[i] = sp8408_buf_3[i] | sp8408_buf_4[i];
		 sp8408_buf_56[i] = sp8408_buf_5[i] | sp8408_buf_6[i];
		 
	}
	for(i = 0; i < 63; i++)
	{
		 checksum34 += sp8408_buf_34[i];
		 checksum56 += sp8408_buf_56[i];
		 
	}
  // if(((checksum34&0xff) != sp8408_buf_34[63])||((checksum56&0xff) != sp8408_buf_56[63]))
   	//return;
   
    for(i = 0; i < 12; i++)
    {        
    	write_cmos_sensor(0x3618+i,sp8408_buf_34[i]);
		write_cmos_sensor(0x360c+i,sp8408_buf_56[i]);       
    	write_cmos_sensor(0x363c+i,sp8408_buf_34[12+i]);
		write_cmos_sensor(0x3630+i,sp8408_buf_56[12+i]);       
    	write_cmos_sensor(0x3660+i,sp8408_buf_34[24+i]);
		write_cmos_sensor(0x3648+i,sp8408_buf_56[24+i]);        
    	write_cmos_sensor(0x3684+i,sp8408_buf_34[36+i]);
		write_cmos_sensor(0x3678+i,sp8408_buf_56[36+i]);
    }
    

		write_cmos_sensor(0x3692,sp8408_buf_34[48]);
		write_cmos_sensor(0x3695,sp8408_buf_34[49]);
		write_cmos_sensor(0x3698,sp8408_buf_34[50]);
		write_cmos_sensor(0x369B,sp8408_buf_34[51]);
		write_cmos_sensor(0x369E,sp8408_buf_34[52]);
		write_cmos_sensor(0x36A1,sp8408_buf_34[53]);
		write_cmos_sensor(0x36A4,sp8408_buf_34[54]);
		write_cmos_sensor(0x36A7,sp8408_buf_34[55]);
		write_cmos_sensor(0x36AA,sp8408_buf_34[56]);
		write_cmos_sensor(0x3693,sp8408_buf_34[57]);
		write_cmos_sensor(0x3696,sp8408_buf_34[58]);
		write_cmos_sensor(0x3699,sp8408_buf_34[59]);
		write_cmos_sensor(0x369C,sp8408_buf_34[60]);
		write_cmos_sensor(0x36B0,sp8408_buf_34[61]);
		write_cmos_sensor(0x36B1,sp8408_buf_34[62]);

   
		write_cmos_sensor(0x3694,sp8408_buf_56[48]);
		write_cmos_sensor(0x3697,sp8408_buf_56[49]);
		write_cmos_sensor(0x369A,sp8408_buf_56[50]);
		write_cmos_sensor(0x369D,sp8408_buf_56[51]);
		write_cmos_sensor(0x36A0,sp8408_buf_56[52]);
		write_cmos_sensor(0x36A3,sp8408_buf_56[53]);
		write_cmos_sensor(0x36A6,sp8408_buf_56[54]);
		write_cmos_sensor(0x36A9,sp8408_buf_56[55]);
		write_cmos_sensor(0x36AC,sp8408_buf_56[56]);
		write_cmos_sensor(0x369F,sp8408_buf_56[57]);
		write_cmos_sensor(0x36A2,sp8408_buf_56[58]);
		write_cmos_sensor(0x36A5,sp8408_buf_56[59]);
		write_cmos_sensor(0x36A8,sp8408_buf_56[60]);
		write_cmos_sensor(0x36AB,sp8408_buf_56[61]);
		write_cmos_sensor(0x36AE,sp8408_buf_56[62]);



}
static kal_uint16 SP8408MIPI_update_lenc_register_from_otp(void)
{
	UINT8 sp8408_buf_0[64];
	UINT8 *p_buf = sp8408_buf_0;
	ReadOTP(0,p_buf);
    kal_uint16 lscflag = sp8408_buf_0[0];
	if(1 == lscflag)//load auto
		  return 0;
	else if(3 == lscflag)
	    write_cmos_sensor( 0x3551, 0x85);
	else if(7 == lscflag)
	    SP8408MIPI_update_lenc();
	else
	{
		
	       LOG_INF("[gpw ] no valid LSC OTP data!\n");
		   return 1;
	}
   	
   return 0;
   

}
static kal_uint16 GetOtpCpId(void)
{

	UINT8 sp8408_buf[64];
	UINT8 *p_buf = sp8408_buf;
	ReadOTP(15,p_buf);
   
	kal_uint8 sensorIDH0 =sp8408_buf[32] ;
    kal_uint8 sensorIDl0 =sp8408_buf[33] ; 
	kal_uint8 sensorIDH1 =sp8408_buf[34] ;
    kal_uint8 sensorIDl1 =sp8408_buf[35] ;
	kal_uint8 sensorIDH2 =sp8408_buf[36] ;
    kal_uint8 sensorIDl2 =sp8408_buf[37] ;
	kal_uint8 sensorIDH = sensorIDH0 | sensorIDH1 | sensorIDH2;
	kal_uint8 sensorIDl = sensorIDl0 | sensorIDl1 | sensorIDl2;
   return ((sensorIDH<<8) + sensorIDl);
}
static kal_uint16 GetOtpId(void)
{

	UINT8 sp8408_buf[32];
	UINT8 *p_buf = sp8408_buf;
	ReadParam_012Page(0,p_buf);
    kal_uint16 oknum = sp8408_buf[5];
	ReadParam_012Page(2,p_buf);
	oknum = 20+(((oknum&0x04)>>2)+((oknum&0x02)>>1)+(oknum&0x1))*3;
	 kal_uint8 sensorIDH =sp8408_buf[oknum] ;
    kal_uint8 sensorIDl =sp8408_buf[oknum+1] ; 
   return ((sensorIDH<<8) + sensorIDl);
}
#endif
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
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0; 
	LOG_INF("MIPI 4LANE\n");
	//LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
#ifdef SP8408MIPI_USE_OTP


   kal_uint16 sensorIDtest = GetOtpId() ;
   kal_uint16 sensorIDtest_Cp = GetOtpCpId();
   printk("gpw  sensorIDtest 0x%x, sensorIDtest_Cp:0x%x\n",sensorIDtest,sensorIDtest_Cp );	
   
   if ((sensorIDtest != 0x8408)&&(sensorIDtest_Cp != 0x8408 ))
   return ERROR_SENSOR_CONNECT_FAIL;

    SP8408MIPI_update_lenc_register_from_otp();
	
#endif
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
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
	mdelay(10);
	set_mirror_flip(imgsensor.mirror);
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x2D00;
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
  //  LOG_INF("E\n");

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
 //   LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	mdelay(10);
	return ERROR_NONE;
}	/*	preview   */

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
  //  LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
  //          LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	mdelay(10);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  //  LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	mdelay(10);
	
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
 //   LOG_INF("E\n");
	
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
	mdelay(10);
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
 //   LOG_INF("E\n");
	
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
	mdelay(10);
	
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
 //   LOG_INF("E\n");
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
   // LOG_INF("scenario_id = %d\n", scenario_id);

	
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
	//sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	//sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
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
  //  LOG_INF("scenario_id = %d\n", scenario_id);
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
  //          LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
  //  LOG_INF("framerate = %d\n ", framerate);
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
  //  LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
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
	kal_uint32 frame_length;
  
  //  LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();	
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
  //  LOG_INF("scenario_id = %d\n", scenario_id);

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

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		//write_cmos_sensor(0x5E00, 0x80);  //2015-1-14 sp_miao
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		//write_cmos_sensor(0x5E00, 0x00);   //2015-1-14 sp_miao
	}	 
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
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	LOG_INF("feature_id = %d\n", feature_id);
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
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing			 
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;							 
			break;				
		case SENSOR_FEATURE_SET_FRAMERATE:
    //        LOG_INF("current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_SET_HDR:
  //          LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
   //         LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
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
  //          LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
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

UINT32 SP8408_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5693_MIPI_RAW_SensorInit	*/
