/*****************************************************************************
 *
 * Filename:
 * ---------
 *   S5K3H7mipi_Sensor.c
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
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k3h7yxmipiraw_Sensor.h"


#define PFX "s5k3h7yx_camera_sensor"
#define LOG_1 LOG_INF("s5k3h7yx,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2664*1500@30fps,888Mbps/lane; video 5328*3000@30fps,1390Mbps/lane; capture 16M@30fps,1390Mbps/lane\n")
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)
#define LOGE(format, args...)   xlog_printk(ANDROID_LOG_ERROR, PFX, "[%s] " format, __FUNCTION__, ##args)

#define MIPI_SETTLEDELAY_AUTO     0
#define MIPI_SETTLEDELAY_MANNUAL  1

static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define SLOW_MOTION_120FPS

static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = S5K3H7YX_SENSOR_ID,
	.checksum_value = 0xe32ded8,
	.pre = {
		.pclk = 280000000,				//record different mode's pclk
		.linelength = 3688,				//record different mode's linelength
		.framelength =2530, //3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1600,		//record different mode's width of grabwindow
		.grabwindow_height = 1200,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 65,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 280000000,
		.linelength =3688,
		.framelength = 2530,
		.startx = 0,
		.starty = 0,
		.grabwindow_width =3264,//5334,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 65,//unit , ns
		.max_framerate = 300,
		},
	.cap1 = {
		.pclk = 280000000,
		.linelength =3688,
		.framelength = 2530,
		.startx = 0,
		.starty = 0,
		.grabwindow_width =3264,//5334,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 65,//unit , ns
		.max_framerate = 300,	
	},
	.normal_video = {
		.pclk = 280000000,
		.linelength = 3688,
		.framelength = 2530,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,//5334,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 65,//unit , ns
		.max_framerate = 300,
	},
	
	.hs_video = {
		.pclk = 280000000,
		.linelength = 3560,
		.framelength = 654,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 800,		//record different mode's width of grabwindow
		.grabwindow_height = 600,		//record different mode's height of grabwindow
		.mipi_data_lp2hs_settle_dc = 65,//unit , ns
		.max_framerate = 1200,
	},

	.slim_video = {
		.pclk = 280000000,
		.linelength = 3088,
		.framelength = 766,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1200,//1280,
		.grabwindow_height =700,// 720,
		.mipi_data_lp2hs_settle_dc = 65,//unit , ns
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
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANNUAL,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
//	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20, 0xff},
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
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x5A,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 3280, 2464,	  0,   0, 3280, 2464, 1634,  1224, 0000, 0000, 1634, 1224,0,   0, 1600,  1200}, // Preview 
 { 3280, 2464,	  0,   0, 3280, 2464, 3264, 2448, 0000, 0000, 3264, 2448,	 0,   0, 3200, 2400}, // capture 
 { 3280, 2464,	  0,   0, 3280, 2464, 3264, 2448, 0000, 0000, 3264, 2448,	 0,   0,3200,2400}, // video 
 { 3280, 2464,	  0,   0, 3280, 2464, 816, 612, 0000, 0000, 816, 612,	  0,   0, 800, 600}, //hight speed video 
 { 3280, 2464,	  0,   0, 3280, 2464, 1280,  720, 0000, 0000, 1280,  720,	 0,   0, 1200,  700}};// slim video 

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}


static void set_dummy()
{
	
	 LOG_INF("currently the mode is %d,dummyline = %d, dummypixels = %d ", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor_8(0x0104, 0x01); 
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0342, imgsensor.line_length);
    write_cmos_sensor_8(0x0104, 0x00); 

  
}	/*	set_dummy  */

static kal_uint32 return_sensor_id()
{
	return (read_cmos_sensor(0x0000));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
		kal_uint32 frame_length = imgsensor.frame_length;
		//unsigned long flags;
	
		LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);
	   
		frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
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
		unsigned long flags;
		spin_lock_irqsave(&imgsensor_drv_lock, flags);
		imgsensor.shutter = shutter;
		spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
		
		spin_lock_irqsave(&imgsensor_drv_lock, flags);
		if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
			imgsensor.frame_length = shutter + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length = imgsensor_info.max_frame_length;
		spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
		shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
		shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
		
		
		
 #if 0   
		if (imgsensor.autoflicker_en) { 
			realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
			if(realtime_fps >= 297 && realtime_fps <= 305)
				set_max_framerate(296,0);
			else if(realtime_fps >= 147 && realtime_fps <= 150)
				set_max_framerate(146,0);	
		} else {
			// Extend frame length
			write_cmos_sensor_8(0x0104,0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length);
			write_cmos_sensor_8(0x0104,0x00);
		}
#endif
		// Update Shutter
		write_cmos_sensor_8(0x0104,0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length);
		write_cmos_sensor(0x0202, shutter);
		write_cmos_sensor_8(0x0104,0x00);
        LOG_INF("Currently camera mode is %d,shutter is %d, framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.shutter,imgsensor.frame_length,imgsensor.line_length);

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

	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	write_shutter(shutter);
	LOG_INF("Currently camera mode is %d,framerate is %d , framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.current_fps,imgsensor.frame_length,imgsensor.line_length);

}	



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	 kal_uint16 reg_gain = 0x0;
    
    reg_gain = gain/2;
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
	   kal_uint16 reg_gain;
	
	 /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	 /* [0:3] = N meams N /16 X  */
	 /* [4:9] = M meams M X 	  */
	 /* Total gain = M + N /16 X   */
	
	 //
	 if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
		 LOG_INF("Error gain setting");
	
		 if (gain < BASEGAIN)
			 gain = BASEGAIN;
		 else if (gain > 32 * BASEGAIN)
			 gain = 32 * BASEGAIN;		  
	 }
	
	 reg_gain = gain2reg(gain);
	 spin_lock(&imgsensor_drv_lock);
	 imgsensor.gain = reg_gain; 
	 spin_unlock(&imgsensor_drv_lock);
	 LOG_INF("gain = %d , reg_gain = 0x%x,shutter=%d,the result of gain*shutter is %d ", gain, reg_gain,imgsensor.shutter,gain*(imgsensor.shutter));
	
	 write_cmos_sensor_8(0x0104, 0x01);
	 write_cmos_sensor_8(0x0204,(reg_gain>>8));
	 write_cmos_sensor_8(0x0205,(reg_gain&0xff));
	 write_cmos_sensor_8(0x0104, 0x00);


	 return gain;

}	

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
#if 1
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
			write_cmos_sensor_8(0x0104,0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length);
	
			//write_cmos_sensor(0x0202, se);		
			//write_cmos_sensor(0x021e,le); 
			write_cmos_sensor(0x602A,0x021e);
			write_cmos_sensor(0x6f12,le);
			write_cmos_sensor(0x602A,0x0202);
			write_cmos_sensor(0x6f12,se);
			 write_cmos_sensor_8(0x0104,0x00);
		LOG_INF("iHDR:imgsensor.frame_length=%d\n",imgsensor.frame_length);
			set_gain(gain);
		}
	
#endif





}



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d", image_mirror);

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
	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror; 
    spin_unlock(&imgsensor_drv_lock);
    switch (image_mirror) {

        case IMAGE_NORMAL:
            write_cmos_sensor_8(0x0101,0x00);   // Gr
            break;
        case IMAGE_H_MIRROR:
            write_cmos_sensor_8(0x0101,0x01);
            break;
        case IMAGE_V_MIRROR:
            write_cmos_sensor_8(0x0101,0x02);
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor_8(0x0101,0x03);//Gb
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


static void sensor_init(void)
{
      LOG_INF("enter\n");
	  
	 // SENSORDB("enter\n");
	//1600x1200
	  write_cmos_sensor(0x6010,0x0001);   // Reset
	  mdelay(50);//; delay(10ms)
	  write_cmos_sensor(0x38FA,   0x0030);	  
	  write_cmos_sensor(0x38FC,   0x0030);	  
	  write_cmos_sensor(0x0086,   0x01FF);	  
	  write_cmos_sensor(0x6218,   0xF1D0);	  
	  write_cmos_sensor(0x6214,   0xF9F0);	  
	  write_cmos_sensor(0x6226,   0x0001);	  
	  write_cmos_sensor(0xB0C0,   0x000C);
	  write_cmos_sensor(0xF400,   0x0BBC);	  
	  write_cmos_sensor(0xF616,   0x0004);	  
	  write_cmos_sensor(0x6226,   0x0000);	  
	  write_cmos_sensor(0x6218,   0xF9F0);	  
	  write_cmos_sensor(0x6218,   0xF1D0);	  
	  write_cmos_sensor(0x6214,   0xF9F0);	  
	  write_cmos_sensor(0x6226,   0x0001);	  
	  write_cmos_sensor(0xB0C0,   0x000C);
	  write_cmos_sensor(0x6226,   0x0000);	  
	  write_cmos_sensor(0x6218,   0xF9F0);	  
	  write_cmos_sensor(0x38FA,   0x0030);	  
	  write_cmos_sensor(0x38FC,   0x0030);	  
	  write_cmos_sensor(0x32CE,   0x0060);	  
	  write_cmos_sensor(0x32D0,   0x0024);	  
	  write_cmos_sensor_8(0x0114,   0x03	  );
	  write_cmos_sensor(0x030E,   0x00A5);	  
	  write_cmos_sensor(0x0342,   0x0E68);	  
	  write_cmos_sensor(0x0340,   0x09E2);
	  write_cmos_sensor(0x0200,   0x0618);	  
	  write_cmos_sensor(0x0202,   0x09C2);	  
	  write_cmos_sensor(0x0204,   0x0020);	  
	  write_cmos_sensor_8(0x3011,   0x02	  );
	  write_cmos_sensor_8(0x0900,   0x01	);
	  write_cmos_sensor_8(0x0901,   0x12	  );
	  write_cmos_sensor(0x034C,   0x0662);
	  write_cmos_sensor(0x034E,   0x04C8);
	  write_cmos_sensor(0x6028,   0xD000);
	  write_cmos_sensor(0x602A,   0x012A);
	  write_cmos_sensor(0x6F12,   0x0040);
	  write_cmos_sensor(0x6F12,   0x7077);
	  write_cmos_sensor(0x6F12,   0x7777);	  
	    
	  mdelay(2);
	  LOG_INF("exit init\n");

}	/*	sensor_init  */


static void preview_setting(void)
{
	//1600x1200

			LOG_INF("preview seting enter,I'm the new one\n");
	write_cmos_sensor(0x0100,   0x0000);
	mdelay(10);
	write_cmos_sensor_8(0x0105,	0x04 		); // mask interrupt
	write_cmos_sensor_8(0x0104,	0x01    );
	write_cmos_sensor_8(0x0114,	0x03	  ); //3             
	write_cmos_sensor(0x030E,	0x00A5	);
	write_cmos_sensor(0x0342,	0x0E68	);
	write_cmos_sensor(0x0340,	0x09E2	);
	write_cmos_sensor(0x0200,	0x0618	);
	write_cmos_sensor(0x0202,	0x09C2	);
	write_cmos_sensor(0x0204,	0x0020	);
	write_cmos_sensor_8(0x3011,	0x02	  );
	write_cmos_sensor_8(0x0900,	0x01    );
	write_cmos_sensor_8(0x0901,	0x12    );
	write_cmos_sensor(0x0346,	0x0004	);
	write_cmos_sensor(0x034A,	0x0993	);
	write_cmos_sensor(0x034C,	0x0662	);
	write_cmos_sensor(0x034E,	0x04C8	);
	write_cmos_sensor(0x6004,	0x0000  );
	write_cmos_sensor(0x6028,	0xD000  );
	write_cmos_sensor(0x602A,	0x012A  );
	write_cmos_sensor(0x6F12,	0x0040	);
	write_cmos_sensor(0x6F12,	0x7077	);
	write_cmos_sensor(0x6F12,	0x7777	);
	write_cmos_sensor_8(0x0104,	0x00    );
	mdelay(10);
	write_cmos_sensor(0x0100,   0x0100);	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.line_length = imgsensor_info.pre.linelength;
	spin_unlock(&imgsensor_drv_lock);
        mdelay(50);

}	/*	preview_setting  */


static void normal_capture_setting()
{
	LOG_INF("capture setting enter\n");
	
	write_cmos_sensor(0x0100,   0x0000);
	mdelay(10);
	write_cmos_sensor_8(0x0105,	0x04   );
	write_cmos_sensor_8(0x0104,	0x01   );
	write_cmos_sensor_8(0x0114,	0x03	 );
	write_cmos_sensor(0x030E,	0x00A5 );
	write_cmos_sensor(0x0342,	0x0E68 );
	write_cmos_sensor(0x0340,	0x09E2 );
	write_cmos_sensor(0x0200,	0x0618 );
	write_cmos_sensor(0x0202,	0x09C2 );
	write_cmos_sensor(0x0204,	0x0020 );
	write_cmos_sensor(0x0346,	0x0004 );
	write_cmos_sensor(0x034A,	0x0993 );
	write_cmos_sensor(0x034C,	0x0CC0 );
	write_cmos_sensor(0x034E,	0x0990 );
	write_cmos_sensor(0x0900,	0x0011 );
	write_cmos_sensor(0x0901,	0x0011 );
	write_cmos_sensor(0x3011,	0x0001 );
	write_cmos_sensor(0x6004,	0x0000 );
	write_cmos_sensor(0x6028,	0xD000 );
	write_cmos_sensor(0x602A,	0x012A );
	write_cmos_sensor(0x6F12,	0x0040 );
	write_cmos_sensor(0x6F12,	0x7077 );
	write_cmos_sensor(0x6F12,	0x7777 );
	write_cmos_sensor_8(0x0104,	0x00    );
	mdelay(10);
	write_cmos_sensor(0x0100,   0x0100);
	//write_cmos_sensor_8(0x0104, 0x00   );
	spin_lock(&imgsensor_drv_lock);
	imgsensor.line_length = imgsensor_info.cap.linelength;
	spin_unlock(&imgsensor_drv_lock);
		mdelay(2);

	



}

static void pip_capture_setting()
{
	normal_capture_setting();

	
}


static void capture_setting(kal_uint16 currefps)
{
	//capture_20fps();

		normal_capture_setting();

}

static void normal_video_setting(kal_uint16 currefps)
{
	   LOG_INF(" normal video setting,use capture setting,now grab window is %d * %d",imgsensor_info.normal_video.grabwindow_width,imgsensor_info.normal_video.grabwindow_height);
	#if 0

	    write_cmos_sensor(0x0100,   0x0000);
	    mdelay(10);
		write_cmos_sensor_8(0x0105,	0x04);	 
		write_cmos_sensor_8(0x0104,	0x01);	 
		write_cmos_sensor_8(0x0114,	  0x03);	 
		write_cmos_sensor(0x030E,	0x00A5);	 
		write_cmos_sensor(0x0342,	0x0E68);	 
		write_cmos_sensor(0x0340,	0x09E2);	 
		write_cmos_sensor(0x0200,	0x0618);	 
		write_cmos_sensor(0x0202,	0x09C2);	 
		write_cmos_sensor(0x0204,	0x0020);	 
		write_cmos_sensor(0x0346,	0x0136);	 
		write_cmos_sensor(0x034A,	0x0861);	 
		write_cmos_sensor(0x034C,	0x0CC0);	 
		write_cmos_sensor(0x034E,	0x072C);	 
		write_cmos_sensor(0x0900,	0x0011);   
		write_cmos_sensor(0x0901,	0x0011);   
		write_cmos_sensor(0x3011,	0x0001);   
		write_cmos_sensor(0x6004,	0x0000);   
		write_cmos_sensor(0x6028,	0xD000);   
		write_cmos_sensor(0x602A,	0x012A);   
		write_cmos_sensor(0x6F12,	0x0040);	 
		write_cmos_sensor(0x6F12,	0x7077);	 
		write_cmos_sensor(0x6F12,	0x7777);	 
		write_cmos_sensor_8(0x0104, 0x00	);
		write_cmos_sensor(0x0100,   0x0100);
	#endif
   // use capture setting
normal_capture_setting();



		
}
static void hs_video_setting()
{
    LOG_INF("enter hs_video setting\n");
	write_cmos_sensor(0x0100, 0x00);
	mdelay(10);
	write_cmos_sensor(0x6218, 0xF1D0);
	write_cmos_sensor(0x6214, 0xF9F0);
	write_cmos_sensor(0x6226, 0x0001);
	write_cmos_sensor(0xB0C0, 0x000C);
	write_cmos_sensor(0x6226, 0x0000);
	write_cmos_sensor(0x6218, 0xF9F0);
	write_cmos_sensor_8(0x0114,	0x03);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0002);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x008C);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A,0x0001);
	write_cmos_sensor(0x030C, 0x0006);
	write_cmos_sensor(0x030E, 0x00AF);
	write_cmos_sensor(0x0342, 0x0DE8);
	write_cmos_sensor(0x0340, 0x028E);
	write_cmos_sensor(0x0200, 0x0618);
	write_cmos_sensor(0x0202, 0x0002);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor_8(0x3011,	0x04);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0386, 0x0001);
	write_cmos_sensor_8(0x0900,	0x01);
	write_cmos_sensor_8(0x0901,	0x14);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0348, 0x0CC7);
	write_cmos_sensor(0x0346, 0x0004);
	write_cmos_sensor(0x034A, 0x0993);
	write_cmos_sensor(0x034C, 0x0330);
	write_cmos_sensor(0x034E, 0x0264);
	write_cmos_sensor(0x0400, 0x0002);
	write_cmos_sensor(0x0404, 0x0010);
	mdelay(10);
	write_cmos_sensor_8(0x0100,	0x01);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
    spin_unlock(&imgsensor_drv_lock);
    mdelay(2);






}

static void slim_video_setting()
{
    LOG_INF("enter slim_video setting\n");
	write_cmos_sensor(0x0100, 0x00);
	mdelay(10);
	write_cmos_sensor(0x6218, 0xF1D0);
	write_cmos_sensor(0x6214, 	0xF9F0);
	write_cmos_sensor(0x6226, 	0x0001);
	write_cmos_sensor(0xB0C0, 	0x000C);
	write_cmos_sensor(0x6226, 	0x0000);
	write_cmos_sensor(0x6218, 	0xF9F0);
	write_cmos_sensor_8(0x0114,		0x03);
	write_cmos_sensor(0x0136, 	0x1800);
	write_cmos_sensor(0x0300	,	0x0002);
	write_cmos_sensor(0x0302, 	0x0001);
	write_cmos_sensor(0x0304, 	0x0006);
	write_cmos_sensor(0x0306, 	0x008C);
	write_cmos_sensor(0x0308, 	0x0008);
	write_cmos_sensor(0x030A, 	0x0001);
	write_cmos_sensor(0x030C, 	0x0006);
	write_cmos_sensor(0x030E, 	0x00A5);
	write_cmos_sensor(0x0342, 	0x0C10);
	write_cmos_sensor(0x0340, 	0x02FE);
	write_cmos_sensor(0x0200, 	0x0618);
	write_cmos_sensor(0x0202, 	0x0002);
	write_cmos_sensor(0x0204, 	0x0020);
	write_cmos_sensor_8(0x3011,		0x01);
	write_cmos_sensor(0x0382, 	0x0003);
	write_cmos_sensor(0x0386, 	0x0003);
	write_cmos_sensor_8(0x0900,		0x01);
	write_cmos_sensor_8(0x0901,		0x22);
	write_cmos_sensor(0x0344, 	0x0164);
	write_cmos_sensor(0x0348, 	0x0B63);
	write_cmos_sensor(0x0346, 	0x01FC);
	write_cmos_sensor(0x034A, 	0x079B);
	write_cmos_sensor(0x034C, 	0x0500);
	write_cmos_sensor(0x034E, 	0x02D0);
	write_cmos_sensor(0x0400, 	0x0002);
	write_cmos_sensor(0x0404, 	0x0010);
	mdelay(10);
	write_cmos_sensor_8(0x0100,		0x01);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
  spin_unlock(&imgsensor_drv_lock);
  mdelay(2);


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
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
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
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0; 
	LOG_1;
	LOG_2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
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

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = KAL_FALSE;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

#ifdef CONFIG_HCT_AF_POWER_ON_NOISE
         //AF_VCC
        if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,"s5k3h7_sensor"))
        {
             LOG_INF("s5k3h7_sensor af poweron failed: \n");
        }
    
                mdelay(5);
#endif

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
#ifdef CONFIG_HCT_AF_POWER_ON_NOISE
        if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF, "s5k3h7_sensor"))
        {
             LOG_INF("down s5k3h7_sensor af powerdown failed: \n");
        }
        
#endif
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
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("Currently camera mode is %d, framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.frame_length,imgsensor.line_length);
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
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

    if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) // 30fps
    {
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
    {
		if (imgsensor.current_fps != imgsensor_info.cap1.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor_info.cap1.max_framerate/10);   
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} 

	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Caputre fps:%d\n",imgsensor.current_fps);
	set_dummy();
	capture_setting(imgsensor.current_fps); 
    set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("Currently camera mode is %d, framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.frame_length,imgsensor.line_length);
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("Currently camera mode is %d, framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.frame_length,imgsensor.line_length);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("hs_video enter i ");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
	hs_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("Currently camera mode is %d,framerate is %d , framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.current_fps,imgsensor.frame_length,imgsensor.line_length);
}	/*	hs_video   */


static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("slim video enter in");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
	slim_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("Currently camera mode is %d,framerate is %d , framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.current_fps,imgsensor.frame_length,imgsensor.line_length);
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF(" get resolution, now the mode is%d",imgsensor.sensor_mode);
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
	LOG_INF("scenario_id = %d", scenario_id);

	
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
	LOG_INF("scenario_id = %d", scenario_id);
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
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) 	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length;
  
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
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
			//set_dummy();			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if(framerate==300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			else
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			//set_dummy();			
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			LOG_INF("scenario hs_video:Currently camera mode is %d,framerate is %d , framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.current_fps,imgsensor.frame_length,imgsensor.line_length);
			//set_dummy();			
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
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

static kal_uint32 set_test_pattern_mode(kal_bool enable){
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0600, 0x0002);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0600, 0x0000);
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
     //unsigned long long *feature_return_para=(unsigned long long *) feature_para;
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	LOG_INF("feature_id = %d", feature_id);
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
			write_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
		     //night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:		
			set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			if((sensor_reg_data->RegData>>8)>0)
			   write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			else
				write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
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
			get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(uintptr_t)(*(feature_data+1)));
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
		case SENSOR_FEATURE_SET_HDR:
			//LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
			LOG_INF("Warning! Not Support IHDR Feature");
			spin_lock(&imgsensor_drv_lock);
			//imgsensor.ihdr_en = (BOOL)*feature_data;
            imgsensor.ihdr_en = KAL_FALSE;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
			LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data_32);
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
			break;
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

UINT32 S5K3H7YX_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
printk("Jeff,enter S5K3H7YX_MIPI_RAW_SensorInit\n");
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	s5k2p8_MIPI_RAW_SensorInit	*/
