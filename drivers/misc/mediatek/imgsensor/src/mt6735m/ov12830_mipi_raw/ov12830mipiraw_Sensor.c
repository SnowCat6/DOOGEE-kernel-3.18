/*****************************************************************************
 *
 * Filename:
 * ---------
 *     OV12830mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
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
//#include <linux/xlog.h>
#include "kd_camera_typedef.h"

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov12830mipiraw_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "OV12830_camera_sensor"
#define LOG_1 LOG_INF("OV12830,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2096*1552@30fps,640Mbps/lane; video 4192*3104@30fps,1.2Gbps/lane; capture 13M@30fps,1.2Gbps/lane\n")
/****************************   Modify end    *******************************************/

#if 0
#define LOG_INF(format, args...)    printk(PFX "[%s] " format, __FUNCTION__, ##args)
#else
#define LOG_INF(format, args...)   
#endif

extern u32 pinSetIdx;
static DEFINE_SPINLOCK(imgsensor_drv_lock);
#define MIPI_SETTLEDELAY_AUTO     0
#define MIPI_SETTLEDELAY_MANNUAL  1

int ov12830_chip_ver = OV12830_R1A;


static imgsensor_info_struct imgsensor_info = {
    .sensor_id = OV12830_SENSOR_ID,        //record sensor id defined in Kd_imgsensor.h

    .checksum_value = 0xa261a12b,//0xf86cfdf4,        //checksum value for Camera Auto Test

    .pre = {
        .pclk = 168000000,                //record different mode's pclk
        .linelength = 1949,                //record different mode's linelength
        .framelength = 2816,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2096,        //record different mode's width of grabwindow
        .grabwindow_height = 1500,        //record different mode's height of grabwindow
        /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        /*     following for GetDefaultFramerateByScenario()    */
        .max_framerate = 300,
    },
    .cap = {
        .pclk = 204000000,
        .linelength = 3972,
        .framelength = 4532,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4000, //4192,
        .grabwindow_height = 3000,//3104,
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        .max_framerate = 150,

    },
    .cap1 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 384000000,
        .linelength = 1949,
        .framelength = 2816,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4192,
        .grabwindow_height = 3000,
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        .max_framerate = 240,    //less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps
    },
    .normal_video = {
        .pclk = 168000000,             
        .linelength = 1949,           
        .framelength = 2816,            
        .startx = 0,                   
        .starty = 0,                   
        .grabwindow_width = 2096,      
				.grabwindow_height = 1500, 
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 168000000,
        .linelength = 2009,
        .framelength = 2816,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640,
        .grabwindow_height = 480,
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        .max_framerate = 1200,
    },
    .slim_video = {
        .pclk = 168000000,
        .linelength = 2009,//2400,
        .framelength = 2816,//3328,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 100,//unit , ns
        .max_framerate = 300,

    },
    .margin = 4,            //sensor framelength & shutter margin
    .min_shutter = 1,        //min shutter
    .max_frame_length = 0x7fff,//max framelength by sensor register's limitation
    .ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
    .ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 5,      //support sensor mode num

    .cap_delay_frame = 2,        //enter capture delay frame num
    .pre_delay_frame = 2,         //enter preview delay frame num
    .video_delay_frame = 2,        //enter video delay frame num
    .hs_video_delay_frame = 2,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 2,//enter slim video delay frame num

    .isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
    .i2c_addr_table = {0x6c,0x20,0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_V_MIRROR,                //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                    //current shutter
    .gain = 0x100,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x6c,//record current sensor's i2c write id
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{{  4000, 2996, 0, 0, 4000, 2996, 2104, 1560, 0, 0, 2104, 1560, 0, 0, 2096, 1500}, // Preview 
  { 4000, 2996, 0, 0, 4000, 2996, 4000, 2996, 0, 0, 4000, 2996, 0, 0, 4000, 2996}, // capture 
  { 4000, 2996, 0, 0, 4000, 2996, 2104, 1560, 0, 0, 2104, 1560, 0, 0, 2096, 1500}, // video 
  { 4000, 2996, 0, 0, 4000, 2996, 2104, 1560, 0, 0, 1056, 594, 0, 0, 640, 480}, //hight speed video 
	{ 4000, 2996, 0, 0, 4000, 2996, 1296, 730, 0, 0,  1296, 730, 0, 0, 1280, 720}, 

};


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
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    /* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
    write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)&0xFF);
    write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
    write_cmos_sensor(0x380c, (imgsensor.line_length >> 8)&0xFF);
    write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);

}    /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

    LOG_INF("framerate = %d, min_framelength_en = %d \n", framerate,min_framelength_en);

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
}    /*    set_max_framerate  */



/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	  LOG_INF("Enter! shutter =%d \n", shutter);

    //write_shutter(shutter);
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
        else {
        // Extend frame length
        write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)&0xff);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
        }
    } else {
        // Extend frame length
        //write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        //write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
         write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)&0xff);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
    }

    // Update Shutter
    // write_cmos_sensor(0x3502, (shutter << 4) & 0xFF);
    // write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
    // write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);
	   write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
		 write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
	   write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	
		
  
    LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    /*    set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	/*kal_uint16 iReg = 0x0000;
	kal_uint16 iGain=gain;
	ov12830_chip_ver=OV12830_R2A;
	if (ov12830_chip_ver == OV12830_R1A)
	{
		iReg = gain*32/BASEGAIN;
		if(iReg < 0x20)
		{
			iReg = 0x20;
		}
		if(iReg > 0xfc)
		{
			iReg = 0xfc;
		}
		//SENSORDB("[OV12830Gain2Reg]: isp gain:%d,sensor gain:0x%x\n",iGain,iReg);
	}
	else if(ov12830_chip_ver == OV12830_R2A)
	{
		iReg = gain*16/BASEGAIN;
		if(iReg < 0x10)
		{
			iReg = 0x10;
		}
		if(iReg > 0xf8)
		{
			iReg = 0xf8;
		}
	}//*/
	
	kal_uint16 iReg = 0x0000;
	kal_uint16 iGain=gain;
	
	if(iGain <  BASEGAIN) 
	{
		iReg =0;
	}    
	else if (iGain < 2 * BASEGAIN) 
	{
        iReg = 16 * iGain / BASEGAIN - 16;
  }
	else if (iGain < 4 * BASEGAIN) 
	{
        iReg |= 0x10;
        iReg |= (8 *iGain / BASEGAIN - 16);
  }
	else if (iGain < 8 * BASEGAIN) 
	{
        iReg |= 0x30;
        iReg |= (4 * iGain / BASEGAIN - 16);
  }
  else if (iGain < 16 * BASEGAIN) 
  {
        iReg |= 0x70;
        iReg |= (2 * iGain /BASEGAIN - 16);
  }
  else if(iGain < 32 * BASEGAIN) 
  {
        iReg |= 0xF0;
        iReg |= (iGain /BASEGAIN - 16);
  }
  else if(iGain <= 62 * BASEGAIN) 
  {
    	  iReg |= 0x1F0;
        iReg |= (iGain /BASEGAIN/2 - 16);
  } 
  
  
 
  
	return iReg;//ov12830. sensorGlobalGain
}

/*************************************************************************
* FUNCTION
*    set_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    iGain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;
		reg_gain = gain2reg(gain);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.gain = reg_gain; 
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	
		write_cmos_sensor(0x350a, reg_gain >> 8);
		write_cmos_sensor(0x350b, reg_gain & 0xFF);    
		
		return gain;
}    /*    set_gain  */

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
        write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

        write_cmos_sensor(0x3502, (le << 4) & 0xFF);
        write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
        write_cmos_sensor(0x3500, (le >> 12) & 0x0F);

        write_cmos_sensor(0x3508, (se << 4) & 0xFF);
        write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
        write_cmos_sensor(0x3506, (se >> 12) & 0x0F);

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
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFB) | 0x00));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFB) | 0x00));
					break;
				case IMAGE_H_MIRROR:
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFB) | 0x00));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFF) | 0x04));
					break;
				case IMAGE_V_MIRROR:
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFF) | 0x04));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFB) | 0x00));		
					break;
				case IMAGE_HV_MIRROR:
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFF) | 0x04));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFF) | 0x04));
					break;
				default:
					LOG_INF("Error image_mirror setting\n");
    }

}

/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}    /*    night_mode    */

static void sensor_init(void)
{
    LOG_INF("E\n");
 		write_cmos_sensor(0x0103,0x01);
		write_cmos_sensor(0x3001,0x06);
		write_cmos_sensor(0x3002,0x80);
		write_cmos_sensor(0x3011,0x41);
		write_cmos_sensor(0x3012,0x08);
		write_cmos_sensor(0x3014,0x16);
		write_cmos_sensor(0x3015,0x0b);
		write_cmos_sensor(0x3022,0x03);
		write_cmos_sensor(0x3090,0x02);
		write_cmos_sensor(0x3091,0x1b);
		write_cmos_sensor(0x3092,0x00);
		write_cmos_sensor(0x3093,0x00);
		write_cmos_sensor(0x3098,0x03);
		write_cmos_sensor(0x3099,0x11);
		write_cmos_sensor(0x309c,0x01);
		write_cmos_sensor(0x30b3,0x6e);
		write_cmos_sensor(0x30b4,0x04);
		write_cmos_sensor(0x30b5,0x04);
		write_cmos_sensor(0x3093,0x02);
		write_cmos_sensor(0x309b,0x02);
		write_cmos_sensor(0x3106,0x01);
		write_cmos_sensor(0x3304,0x28);
		write_cmos_sensor(0x3305,0x41);
		write_cmos_sensor(0x3306,0x30);
		write_cmos_sensor(0x3308,0x00);
		write_cmos_sensor(0x3309,0xc8);
		write_cmos_sensor(0x330a,0x01);
		write_cmos_sensor(0x330b,0x90);
		write_cmos_sensor(0x330c,0x02);
		write_cmos_sensor(0x330d,0x58);
		write_cmos_sensor(0x330e,0x03);
		write_cmos_sensor(0x330f,0x20);
		write_cmos_sensor(0x3300,0x00);
		write_cmos_sensor(0x3500,0x00);
		write_cmos_sensor(0x3501,0xbd);
		write_cmos_sensor(0x3502,0x40);
		write_cmos_sensor(0x3503,0x07);
		write_cmos_sensor(0x3508,0x00);
		write_cmos_sensor(0x3509,0x08);
		write_cmos_sensor(0x350a,0x00);
		write_cmos_sensor(0x350b,0x00);
		write_cmos_sensor(0x3602,0x28);
		write_cmos_sensor(0x3612,0x80);
		write_cmos_sensor(0x3622,0x0f);
		write_cmos_sensor(0x3631,0xb3);
		write_cmos_sensor(0x3634,0x04);
		write_cmos_sensor(0x3660,0x80);
		write_cmos_sensor(0x3662,0x10);
		write_cmos_sensor(0x3663,0xf0);
		write_cmos_sensor(0x3667,0x00);
		write_cmos_sensor(0x366f,0x20);
		write_cmos_sensor(0x3680,0xb5);
		write_cmos_sensor(0x3682,0x00);
		write_cmos_sensor(0x3701,0x12);
		write_cmos_sensor(0x3708,0xe3);
		write_cmos_sensor(0x3709,0xc3);
		write_cmos_sensor(0x370b,0xa8);
		write_cmos_sensor(0x370d,0x11);
		write_cmos_sensor(0x370e,0x00);
		write_cmos_sensor(0x371c,0x01);
		write_cmos_sensor(0x371f,0x1b);
		write_cmos_sensor(0x3726,0x00);
		write_cmos_sensor(0x372a,0x09);
		write_cmos_sensor(0x3739,0x80);
		write_cmos_sensor(0x373c,0x40);
		write_cmos_sensor(0x376b,0x44);
		write_cmos_sensor(0x377b,0x44);
		write_cmos_sensor(0x3780,0x22);
		write_cmos_sensor(0x3781,0xc8);
		write_cmos_sensor(0x3783,0x31);
		write_cmos_sensor(0x3786,0x16);
		write_cmos_sensor(0x3787,0x02);
		write_cmos_sensor(0x3796,0x84);
		write_cmos_sensor(0x379c,0x0c);
		write_cmos_sensor(0x37c5,0x00);
		write_cmos_sensor(0x37c6,0x00);
		write_cmos_sensor(0x37c7,0x00);
		write_cmos_sensor(0x37c9,0x00);
		write_cmos_sensor(0x37ca,0x00);
		write_cmos_sensor(0x37cb,0x00);
		write_cmos_sensor(0x37cc,0x00);
		write_cmos_sensor(0x37cd,0x00);
		write_cmos_sensor(0x37ce,0x10);
		write_cmos_sensor(0x37cf,0x00);
		write_cmos_sensor(0x37d0,0x00);
		write_cmos_sensor(0x37d1,0x00);
		write_cmos_sensor(0x37d2,0x00);
		write_cmos_sensor(0x37de,0x00);
		write_cmos_sensor(0x37df,0x00);
		write_cmos_sensor(0x3800,0x00);
		write_cmos_sensor(0x3801,0x00);
		write_cmos_sensor(0x3802,0x00);
		write_cmos_sensor(0x3803,0x00);
		write_cmos_sensor(0x3804,0x10);
		write_cmos_sensor(0x3805,0x9f);
		write_cmos_sensor(0x3806,0x0b);
		write_cmos_sensor(0x3807,0xc7);
		write_cmos_sensor(0x3808,0x10);
		write_cmos_sensor(0x3809,0x80);
		write_cmos_sensor(0x380a,0x0b);
		write_cmos_sensor(0x380b,0xb8);
		write_cmos_sensor(0x380c,0x11);
		write_cmos_sensor(0x380d,0x50);
		write_cmos_sensor(0x380e,0x0b);
		write_cmos_sensor(0x380f,0xe4);
		write_cmos_sensor(0x3810,0x00);
		write_cmos_sensor(0x3811,0x10);
		write_cmos_sensor(0x3812,0x00);
		write_cmos_sensor(0x3813,0x08);
		write_cmos_sensor(0x3814,0x11);
		write_cmos_sensor(0x3815,0x11);
		write_cmos_sensor(0x3820,0x10);
		write_cmos_sensor(0x3821,0x0e);
		write_cmos_sensor(0x3823,0x00);
		write_cmos_sensor(0x3824,0x00);
		write_cmos_sensor(0x3825,0x00);
		write_cmos_sensor(0x3826,0x00);
		write_cmos_sensor(0x3827,0x00);
		write_cmos_sensor(0x3829,0x0b);
		write_cmos_sensor(0x382b,0x6a);
		write_cmos_sensor(0x4000,0x18);
		write_cmos_sensor(0x4001,0x06);
		write_cmos_sensor(0x4002,0x45);
		write_cmos_sensor(0x4004,0x08);
		write_cmos_sensor(0x4005,0x19);
		write_cmos_sensor(0x4007,0x90);
		write_cmos_sensor(0x4008,0x20);
		write_cmos_sensor(0x4100,0x2d);
		write_cmos_sensor(0x4101,0x22);
		write_cmos_sensor(0x4102,0x04);
		write_cmos_sensor(0x4104,0x5c);
		write_cmos_sensor(0x4109,0xa3);
		write_cmos_sensor(0x410a,0x03);
		write_cmos_sensor(0x4300,0xff);
		write_cmos_sensor(0x4303,0x00);
		write_cmos_sensor(0x4304,0x08);
		write_cmos_sensor(0x4307,0x30);
		write_cmos_sensor(0x4311,0x04);
		write_cmos_sensor(0x4511,0x05);
		write_cmos_sensor(0x4816,0x52);
		write_cmos_sensor(0x481f,0x30);
		write_cmos_sensor(0x4826,0x2c);
		write_cmos_sensor(0x4a00,0xaa);
		write_cmos_sensor(0x4a03,0x01);
		write_cmos_sensor(0x4a05,0x08);
		write_cmos_sensor(0x4d00,0x05);
		write_cmos_sensor(0x4d01,0x19);
		write_cmos_sensor(0x4d02,0xfd);
		write_cmos_sensor(0x4d03,0xd1);
		write_cmos_sensor(0x4d04,0xff);
		write_cmos_sensor(0x4d05,0xff);
		write_cmos_sensor(0x4d07,0x04);
		write_cmos_sensor(0x4837,0x09);
		write_cmos_sensor(0x5000,0x06);
		write_cmos_sensor(0x5001,0x01);
		write_cmos_sensor(0x5002,0x00);
		write_cmos_sensor(0x5003,0x21);
		write_cmos_sensor(0x5043,0x48);
		write_cmos_sensor(0x5013,0x80);
		write_cmos_sensor(0x501f,0x00);
		write_cmos_sensor(0x5e00,0x00);
		write_cmos_sensor(0x5a01,0x00);
		write_cmos_sensor(0x5a02,0x00);
		write_cmos_sensor(0x5a03,0x00);
		write_cmos_sensor(0x5a04,0x10);
		write_cmos_sensor(0x5a05,0xa0);
		write_cmos_sensor(0x5a06,0x0c);
		write_cmos_sensor(0x5a07,0x78);
		write_cmos_sensor(0x5a08,0x00);
		write_cmos_sensor(0x5e00,0x00);
		write_cmos_sensor(0x5e01,0x41);
		write_cmos_sensor(0x5e11,0x30);
		write_cmos_sensor(0x3501,0xa0);
		write_cmos_sensor(0x0100,0x01);

}    


static void preview_setting(void)
{
  write_cmos_sensor(0x0100,0x00); 
	write_cmos_sensor(0x3501,0x70); 
	write_cmos_sensor(0x30b4,0x03); 
	write_cmos_sensor(0x30b3,0x54); 
	write_cmos_sensor(0x3106,0x21); 
	write_cmos_sensor(0x3090,0x01); 
	write_cmos_sensor(0x3091,0x38); 
	write_cmos_sensor(0x3708,0xe6); 
	write_cmos_sensor(0x3709,0xc7); 
	write_cmos_sensor(0x3801,0x00); 
	write_cmos_sensor(0x3802,0x00); 
	write_cmos_sensor(0x3803,0x00); 
	write_cmos_sensor(0x3805,0x9f); 
	write_cmos_sensor(0x3806,0x0b); 
	write_cmos_sensor(0x3807,0xc7); 
	write_cmos_sensor(0x3808,0x08); 
	write_cmos_sensor(0x3809,0x40); 
	write_cmos_sensor(0x380a,0x05); 
	write_cmos_sensor(0x380b,0xdc); 
	write_cmos_sensor(0x380c,0x0b); 
	write_cmos_sensor(0x380d,0x00); 
	write_cmos_sensor(0x380e,0x07); 
	write_cmos_sensor(0x380f,0x9D); 
	write_cmos_sensor(0x3811,0x08); 
	write_cmos_sensor(0x3813,0x02); 
	write_cmos_sensor(0x3814,0x31); 
	write_cmos_sensor(0x3815,0x31); 
	write_cmos_sensor(0x3820,0x14); 
	write_cmos_sensor(0x3821,0x0f); 
	write_cmos_sensor(0x4004,0x08); 
	write_cmos_sensor(0x4837,0x0b); 
	write_cmos_sensor(0x5002,0x00); 
	write_cmos_sensor(0x0100,0x01); 

}    /*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("E! currefps:%d\n",currefps);
  	write_cmos_sensor(0x0103,0x01);
		write_cmos_sensor(0x3001,0x06);
		write_cmos_sensor(0x3002,0x80);
		write_cmos_sensor(0x3011,0x21);
		write_cmos_sensor(0x3012,0x08);
		write_cmos_sensor(0x3014,0x16);
		write_cmos_sensor(0x3015,0x0b);
		write_cmos_sensor(0x3022,0x03);
		write_cmos_sensor(0x3090,0x02);
		write_cmos_sensor(0x3091,0x1b);
		write_cmos_sensor(0x3092,0x00);
		write_cmos_sensor(0x3093,0x00);
		write_cmos_sensor(0x3098,0x03);
		write_cmos_sensor(0x3099,0x11);
		write_cmos_sensor(0x309c,0x01);
		write_cmos_sensor(0x30b3,0x6e);
		write_cmos_sensor(0x30b4,0x04);
		write_cmos_sensor(0x30b5,0x04);
		write_cmos_sensor(0x3093,0x02);
		write_cmos_sensor(0x309b,0x02);
		write_cmos_sensor(0x3106,0x01);
		write_cmos_sensor(0x3304,0x28);
		write_cmos_sensor(0x3305,0x41);
		write_cmos_sensor(0x3306,0x30);
		write_cmos_sensor(0x3308,0x00);
		write_cmos_sensor(0x3309,0xc8);
		write_cmos_sensor(0x330a,0x01);
		write_cmos_sensor(0x330b,0x90);
		write_cmos_sensor(0x330c,0x02);
		write_cmos_sensor(0x330d,0x58);
		write_cmos_sensor(0x330e,0x03);
		write_cmos_sensor(0x330f,0x20);
		write_cmos_sensor(0x3300,0x00);
		write_cmos_sensor(0x3500,0x00);
		write_cmos_sensor(0x3501,0xbd);
		write_cmos_sensor(0x3502,0x40);
		write_cmos_sensor(0x3503,0x07);
		write_cmos_sensor(0x3508,0x00);
		write_cmos_sensor(0x3509,0x08);
		write_cmos_sensor(0x350a,0x00);
		write_cmos_sensor(0x350b,0x00);
		write_cmos_sensor(0x3602,0x28);
		write_cmos_sensor(0x3612,0x80);
		write_cmos_sensor(0x3622,0x0f);
		write_cmos_sensor(0x3631,0xb3);
		write_cmos_sensor(0x3634,0x04);
		write_cmos_sensor(0x3660,0x80);
		write_cmos_sensor(0x3662,0x10);
		write_cmos_sensor(0x3663,0xf0);
		write_cmos_sensor(0x3667,0x00);
		write_cmos_sensor(0x366f,0x20);
		write_cmos_sensor(0x3680,0xb5);
		write_cmos_sensor(0x3682,0x00);
		write_cmos_sensor(0x3701,0x12);
		write_cmos_sensor(0x3708,0xe3);
		write_cmos_sensor(0x3709,0xc3);
		write_cmos_sensor(0x370b,0xa8);
		write_cmos_sensor(0x370d,0x11);
		write_cmos_sensor(0x370e,0x00);
		write_cmos_sensor(0x371c,0x01);
		write_cmos_sensor(0x371f,0x1b);
		write_cmos_sensor(0x3726,0x00);
		write_cmos_sensor(0x372a,0x09);
		write_cmos_sensor(0x3739,0x80);
		write_cmos_sensor(0x373c,0x40);
		write_cmos_sensor(0x376b,0x44);
		write_cmos_sensor(0x377b,0x44);
		write_cmos_sensor(0x3780,0x22);
		write_cmos_sensor(0x3781,0xc8);
		write_cmos_sensor(0x3783,0x31);
		write_cmos_sensor(0x3786,0x16);
		write_cmos_sensor(0x3787,0x02);
		write_cmos_sensor(0x3796,0x84);
		write_cmos_sensor(0x379c,0x0c);
		write_cmos_sensor(0x37c5,0x00);
		write_cmos_sensor(0x37c6,0x00);
		write_cmos_sensor(0x37c7,0x00);
		write_cmos_sensor(0x37c9,0x00);
		write_cmos_sensor(0x37ca,0x00);
		write_cmos_sensor(0x37cb,0x00);
		write_cmos_sensor(0x37cc,0x00);
		write_cmos_sensor(0x37cd,0x00);
		write_cmos_sensor(0x37ce,0x10);
		write_cmos_sensor(0x37cf,0x00);
		write_cmos_sensor(0x37d0,0x00);
		write_cmos_sensor(0x37d1,0x00);
		write_cmos_sensor(0x37d2,0x00);
		write_cmos_sensor(0x37de,0x00);
		write_cmos_sensor(0x37df,0x00);
		write_cmos_sensor(0x3800,0x00);
		write_cmos_sensor(0x3801,0x00);
		write_cmos_sensor(0x3802,0x00);
		write_cmos_sensor(0x3803,0x00);
		write_cmos_sensor(0x3804,0x10);
		write_cmos_sensor(0x3805,0x9f);
		write_cmos_sensor(0x3806,0x0b);
		write_cmos_sensor(0x3807,0xc7);
		write_cmos_sensor(0x3808,0x10);
		write_cmos_sensor(0x3809,0x80);
		write_cmos_sensor(0x380a,0x0b);
		write_cmos_sensor(0x380b,0xb8);
		write_cmos_sensor(0x380c,0x11);
		write_cmos_sensor(0x380d,0x50);
		write_cmos_sensor(0x380e,0x0b);
		write_cmos_sensor(0x380f,0xe4);
		write_cmos_sensor(0x3810,0x00);
		write_cmos_sensor(0x3811,0x10);
		write_cmos_sensor(0x3812,0x00);
		write_cmos_sensor(0x3813,0x08);
		write_cmos_sensor(0x3814,0x11);
		write_cmos_sensor(0x3815,0x11);
		write_cmos_sensor(0x3820,0x10);
		write_cmos_sensor(0x3821,0x0e);
		write_cmos_sensor(0x3823,0x00);
		write_cmos_sensor(0x3824,0x00);
		write_cmos_sensor(0x3825,0x00);
		write_cmos_sensor(0x3826,0x00);
		write_cmos_sensor(0x3827,0x00);
		write_cmos_sensor(0x3829,0x0b);
		write_cmos_sensor(0x382b,0x6a);
		write_cmos_sensor(0x4000,0x18);
		write_cmos_sensor(0x4001,0x06);
		write_cmos_sensor(0x4002,0x45);
		write_cmos_sensor(0x4004,0x08);
		write_cmos_sensor(0x4005,0x19);
		write_cmos_sensor(0x4007,0x90);
		write_cmos_sensor(0x4008,0x20);
		write_cmos_sensor(0x4100,0x2d);
		write_cmos_sensor(0x4101,0x22);
		write_cmos_sensor(0x4102,0x04);
		write_cmos_sensor(0x4104,0x5c);
		write_cmos_sensor(0x4109,0xa3);
		write_cmos_sensor(0x410a,0x03);
		write_cmos_sensor(0x4300,0xff);
		write_cmos_sensor(0x4303,0x00);
		write_cmos_sensor(0x4304,0x08);
		write_cmos_sensor(0x4307,0x30);
		write_cmos_sensor(0x4311,0x04);
		write_cmos_sensor(0x4511,0x05);
		write_cmos_sensor(0x4816,0x52);
		write_cmos_sensor(0x481f,0x30);
		write_cmos_sensor(0x4826,0x2c);
		write_cmos_sensor(0x4a00,0xaa);
		write_cmos_sensor(0x4a03,0x01);
		write_cmos_sensor(0x4a05,0x08);
		write_cmos_sensor(0x4d00,0x05);
		write_cmos_sensor(0x4d01,0x19);
		write_cmos_sensor(0x4d02,0xfd);
		write_cmos_sensor(0x4d03,0xd1);
		write_cmos_sensor(0x4d04,0xff);
		write_cmos_sensor(0x4d05,0xff);
		write_cmos_sensor(0x4d07,0x04);
		write_cmos_sensor(0x4837,0x09);
		write_cmos_sensor(0x5000,0x06);
		write_cmos_sensor(0x5001,0x01);
		write_cmos_sensor(0x5002,0x00);
		write_cmos_sensor(0x5003,0x21);
		write_cmos_sensor(0x5043,0x48);
		write_cmos_sensor(0x5013,0x80);
		write_cmos_sensor(0x501f,0x00);
		write_cmos_sensor(0x5e00,0x00);
		write_cmos_sensor(0x5a01,0x00);
		write_cmos_sensor(0x5a02,0x00);
		write_cmos_sensor(0x5a03,0x00);
		write_cmos_sensor(0x5a04,0x10);
		write_cmos_sensor(0x5a05,0xa0);
		write_cmos_sensor(0x5a06,0x0c);
		write_cmos_sensor(0x5a07,0x78);
		write_cmos_sensor(0x5a08,0x00);
		write_cmos_sensor(0x5e00,0x00);
		write_cmos_sensor(0x5e01,0x41);
		write_cmos_sensor(0x5e11,0x30);
		write_cmos_sensor(0x3501,0xa0);
		write_cmos_sensor(0x0100,0x01);

}

static void normal_video_setting(kal_uint16 currefps)
{
  	write_cmos_sensor(0x0100,0x00); 
		write_cmos_sensor(0x3501,0x70); 
		write_cmos_sensor(0x30b4,0x03); 
		write_cmos_sensor(0x30b3,0x54); 
		write_cmos_sensor(0x3106,0x21); 
		write_cmos_sensor(0x3090,0x01); 
		write_cmos_sensor(0x3091,0x38); 
		write_cmos_sensor(0x3708,0xe6); 
		write_cmos_sensor(0x3709,0xc7); 
		write_cmos_sensor(0x3801,0x00); 
		write_cmos_sensor(0x3802,0x00); 
		write_cmos_sensor(0x3803,0x00); 
		write_cmos_sensor(0x3805,0x9f); 
		write_cmos_sensor(0x3806,0x0b); 
		write_cmos_sensor(0x3807,0xc7); 
		write_cmos_sensor(0x3808,0x08); 
		write_cmos_sensor(0x3809,0x40); 
		write_cmos_sensor(0x380a,0x05); 
		write_cmos_sensor(0x380b,0xdc); 
		write_cmos_sensor(0x380c,0x0b); 
		write_cmos_sensor(0x380d,0x00); 
		write_cmos_sensor(0x380e,0x07); 
		write_cmos_sensor(0x380f,0x9D); 
		write_cmos_sensor(0x3811,0x08); 
		write_cmos_sensor(0x3813,0x02); 
		write_cmos_sensor(0x3814,0x31); 
		write_cmos_sensor(0x3815,0x31); 
		write_cmos_sensor(0x3820,0x14); 
		write_cmos_sensor(0x3821,0x0f); 
		write_cmos_sensor(0x4004,0x08); 
		write_cmos_sensor(0x4837,0x0b); 
		write_cmos_sensor(0x5002,0x00); 
		write_cmos_sensor(0x0100,0x01); 

}
static void hs_video_setting(void)
{
    LOG_INF("E\n");

   write_cmos_sensor(0x0100,0x00); 
	write_cmos_sensor(0x3501,0x70); 
	write_cmos_sensor(0x30b4,0x03); 
	write_cmos_sensor(0x30b3,0x54); 
	write_cmos_sensor(0x3106,0x21); 
	write_cmos_sensor(0x3090,0x01); 
	write_cmos_sensor(0x3091,0x38); 
	write_cmos_sensor(0x3708,0xe6); 
	write_cmos_sensor(0x3709,0xc7); 
	write_cmos_sensor(0x3801,0x00); 
	write_cmos_sensor(0x3802,0x00); 
	write_cmos_sensor(0x3803,0x00); 
	write_cmos_sensor(0x3805,0x9f); 
	write_cmos_sensor(0x3806,0x0b); 
	write_cmos_sensor(0x3807,0xc7); 
	write_cmos_sensor(0x3808,0x08); 
	write_cmos_sensor(0x3809,0x40); 
	write_cmos_sensor(0x380a,0x05); 
	write_cmos_sensor(0x380b,0xdc); 
	write_cmos_sensor(0x380c,0x0b); 
	write_cmos_sensor(0x380d,0x00); 
	write_cmos_sensor(0x380e,0x07); 
	write_cmos_sensor(0x380f,0x9D); 
	write_cmos_sensor(0x3811,0x08); 
	write_cmos_sensor(0x3813,0x02); 
	write_cmos_sensor(0x3814,0x31); 
	write_cmos_sensor(0x3815,0x31); 
	write_cmos_sensor(0x3820,0x14); 
	write_cmos_sensor(0x3821,0x0f); 
	write_cmos_sensor(0x4004,0x08); 
	write_cmos_sensor(0x4837,0x0b); 
	write_cmos_sensor(0x5002,0x00); 
	write_cmos_sensor(0x0100,0x01); 

}

static void slim_video_setting(void)
{
	write_cmos_sensor(0x0100,0x00); 
	write_cmos_sensor(0x3501,0x70); 
	write_cmos_sensor(0x30b4,0x03); 
	write_cmos_sensor(0x30b3,0x54); 
	write_cmos_sensor(0x3106,0x21); 
	write_cmos_sensor(0x3090,0x01); 
	write_cmos_sensor(0x3091,0x38); 
	write_cmos_sensor(0x3708,0xe6); 
	write_cmos_sensor(0x3709,0xc7); 
	write_cmos_sensor(0x3801,0x00); 
	write_cmos_sensor(0x3802,0x00); 
	write_cmos_sensor(0x3803,0x00); 
	write_cmos_sensor(0x3805,0x9f); 
	write_cmos_sensor(0x3806,0x0b); 
	write_cmos_sensor(0x3807,0xc7); 
	write_cmos_sensor(0x3808,0x08); 
	write_cmos_sensor(0x3809,0x40); 
	write_cmos_sensor(0x380a,0x05); 
	write_cmos_sensor(0x380b,0xdc); 
	write_cmos_sensor(0x380c,0x0b); 
	write_cmos_sensor(0x380d,0x00); 
	write_cmos_sensor(0x380e,0x07); 
	write_cmos_sensor(0x380f,0x9D); 
	write_cmos_sensor(0x3811,0x08); 
	write_cmos_sensor(0x3813,0x02); 
	write_cmos_sensor(0x3814,0x31); 
	write_cmos_sensor(0x3815,0x31); 
	write_cmos_sensor(0x3820,0x14); 
	write_cmos_sensor(0x3821,0x0f); 
	write_cmos_sensor(0x4004,0x08); 
	write_cmos_sensor(0x4837,0x0b); 
	write_cmos_sensor(0x5002,0x00); 
	write_cmos_sensor(0x0100,0x01); 

}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);

    if (enable) {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x5E00, 0x80);
    } else {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x5E00, 0x00);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
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
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
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
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
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
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;
        
		if ((read_cmos_sensor(0x302a))==0xb1)
		{
				LOG_INF("----R1A---- \n");
				ov12830_chip_ver = OV12830_R1A;
		}else if((read_cmos_sensor(0x302a))==0xb2)
		{
				LOG_INF("----R2A---- \n");
				ov12830_chip_ver = OV12830_R2A;
		}
    /* initail sequence write in  */
    sensor_init();
	  //for OTP
	  //otp_cali(imgsensor.i2c_write_id);
	  write_cmos_sensor(0x0100, 0x00);
	  
    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    //for zsd multi capture setting
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}    /*    open  */



/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");

    /*No Need to implement this function*/

    return ERROR_NONE;
}    /*    close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
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
    preview_setting();
if(pinSetIdx)
    set_mirror_flip(imgsensor.mirror);
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    preview   */

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
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
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);
    capture_setting(imgsensor.current_fps);
    if(pinSetIdx)
	  set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}    /* capture() */
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
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);	
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    if(pinSetIdx)
		set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}    /*    normal_video   */

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
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();	
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    if(pinSetIdx)
		set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

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
    slim_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    if(pinSetIdx)
  	set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}    /*    slim_video     */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */

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

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
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
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
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
}    /*    get_info  */


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
}    /* control() */



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
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
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
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
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
            LOG_INF("current fps :%d\n", *feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data);

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
}    /*    feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 OV12830_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    /*    OV12830_MIPI_RAW_SensorInit    */
