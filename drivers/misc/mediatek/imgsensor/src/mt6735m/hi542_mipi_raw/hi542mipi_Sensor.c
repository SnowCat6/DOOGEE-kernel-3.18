/*****************************************************************************
 *
 * Filename:
 * ---------
 *     HI542mipi_Sensor.c
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
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi542mipi_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "HI542_camera_sensor"
#define LOG_1 LOG_INF("HI542,MIPI 2LANE\n")
#define LOG_2 LOG_INF("preview 1296*972@30fps; video 1296*972@30fps; capture 5M@30fps\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF printk
#ifdef SLT_DEVINFO_CMM 
#include  <linux/dev_info.h>
static struct devinfo_struct *s_DEVINFO_ccm;   //suppose 10 max lcm device 
#endif

static DEFINE_SPINLOCK(imgsensor_drv_lock);


static imgsensor_info_struct imgsensor_info = {
    .sensor_id = HI542MIPI_SENSOR_ID,        //record sensor id defined in Kd_imgsensor.h

    .checksum_value = 0x40aeb1f5,        //checksum value for Camera Auto Test

	.pre = {
		.pclk = 84000000, // 176000000,				//record different mode's pclk
		.linelength = 2791,				//record different mode's linelength
		.framelength = 1003,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1304,		//record different mode's width of grabwindow
		.grabwindow_height = 980,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 84000000,
		.linelength = 2791,
		.framelength = 1995,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2608,
		.grabwindow_height = 1960,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 84000000,
		.linelength = 2791,
		.framelength = 1995,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2608,
		.grabwindow_height = 1960,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 300,	
	},
	.normal_video = {
		.pclk = 84000000,
		.linelength = 2791,
		.framelength = 1003,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1304,
		.grabwindow_height = 980,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 84000000,
		.linelength = 2791,
		.framelength = 1003,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1304,
		.grabwindow_height = 980,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 84000000,
		.linelength = 2791,
		.framelength = 1003,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1304,
		.grabwindow_height = 980,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 300,
	},

    .margin = 4,            //sensor framelength & shutter margin
    .min_shutter = 1,        //min shutter
    .max_frame_length = 0xffff,//max framelength by sensor register's limitation
    .ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 1,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
    .ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 5,      //support sensor mode num

    .cap_delay_frame = 2,        //enter capture delay frame num
    .pre_delay_frame = 2,         //enter preview delay frame num
    .video_delay_frame = 2,        //enter video delay frame num
    .hs_video_delay_frame = 3,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 3,//enter slim video delay frame num

    .isp_driving_current = ISP_DRIVING_6MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,///SENSOR_OUTPUT_FORMAT_RAW_R,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_2_LANE,//mipi lane num
    .i2c_addr_table = {0x40, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,//IMAGE_HV_MIRROR,///IMAGE_H_MIRROR,//IMAGE_V_MIRROR,///IMAGE_NORMAL,///IMAGE_HV_MIRROR,                //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                    //current shutter
    .gain = 0x0,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x40,//record current sensor's i2c write id
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{{ 2608, 1960,	  0,	0, 2608, 1960, 1304,   980, 0000, 0000, 1304,   980,	  0,	0, 1304,   980}, // Preview 
 { 2608, 1960,	  0,	0, 2608, 1960, 2608,   1960, 0000, 0000, 2608, 1960,	  0,	0, 2608, 1960}, // capture 
 { 2608, 1960,	  0,	0, 2608, 1960, 1304,   980, 0000, 0000, 1304,   980,	  0,	0, 1304,   980}, // video 
 { 2608, 1960,	  0,	0, 2608, 1960, 1304,   980, 0000, 0000, 1304,   980,	  0,	0, 1304,   980}, //hight speed video 
 { 2608, 1960,	  0,	0, 2608, 1960, 1304,   980, 0000, 0000, 1304,   980,	  0,	0, 1304,   980}};// slim video 


#if 1//lc mike_zhu 20150529 
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void HI542MIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

#else
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

    return get_byte;
}

static void HI542MIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	iWriteReg((u16)addr, (u32)para, 2, imgsensor.i2c_write_id);
}
#endif
static void set_dummy(void)
{
	//LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);

	//HI542MIPI_write_cmos_sensor(0x0104,0x01);	
	//HI542MIPI_write_cmos_sensor(0x0006, imgsensor.frame_length);  
	//HI542MIPI_write_cmos_sensor(0x0008, imgsensor.line_length);
	//HI542MIPI_write_cmos_sensor(0x0104,0x00);	
	

}    /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
    return (read_cmos_sensor(0x0004));
}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

//    LOG_INF("framerate = %d, min framelength should enable? = %d\n", framerate,min_framelength_en);

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
	    kal_uint32 iExp= shutter;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

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

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        // Extend frame length
	//HI542MIPI_write_cmos_sensor(0x0104,0x01);
	//HI542MIPI_write_cmos_sensor(0x0630, imgsensor.frame_length);
	//HI542MIPI_write_cmos_sensor(0x0104,0x00);
        }
    } else {
        // Extend frame length
           // HI542MIPI_write_cmos_sensor(0x0104,0x01);
			//HI542MIPI_write_cmos_sensor(0x0630, imgsensor.frame_length);
			//HI542MIPI_write_cmos_sensor(0x0104,0x00);
    }
	if(shutter<4)
	{
	shutter=4;
	}
	else
	{
	shutter=shutter;
	}	
      iExp=shutter*2791;
    // Update Shutter
	//HI542MIPI_write_cmos_sensor(0x0104,0x01);	
///HI542MIPI_write_cmos_sensor(0x0104,0x01);


    HI542MIPI_write_cmos_sensor(0x0120, ((iExp + 2*2791) >> 24) & 0xFF);
    HI542MIPI_write_cmos_sensor(0x0121, ((iExp + 2*2791) >> 16) & 0xFF);
    HI542MIPI_write_cmos_sensor(0x0122, ((iExp + 2*2791)  >> 8 ) & 0xFF);
    HI542MIPI_write_cmos_sensor(0x0123, (iExp + 2*2791) & 0xFF);

    HI542MIPI_write_cmos_sensor(0x011C, (iExp >> 24) & 0xFF);
    HI542MIPI_write_cmos_sensor(0x011D, (iExp >> 16) & 0xFF);
    HI542MIPI_write_cmos_sensor(0x011E, (iExp >> 8 ) & 0xFF);
    HI542MIPI_write_cmos_sensor(0x011F, (iExp) & 0xFF);
 

    HI542MIPI_write_cmos_sensor(0x0115, (iExp >> 24) & 0xFF);
    HI542MIPI_write_cmos_sensor(0x0116, (iExp >> 16) & 0xFF);
    HI542MIPI_write_cmos_sensor(0x0117, (iExp >> 8 ) & 0xFF);
    HI542MIPI_write_cmos_sensor(0x0118, (iExp) & 0xFF);			
	//HI542MIPI_write_cmos_sensor(0x0104,0x00);	
	


    LOG_INF("hi542 Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    /*    set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	 kal_uint8 iReg;
	kal_uint8 iBaseGain = 64;



	iReg = 256*iBaseGain/gain - 32;

    return iReg;

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
       kal_uint8 iReg;
	kal_uint8 iBaseGain = 64;
 
///	mdelay(10);

	iReg = 256*iBaseGain/gain - 32;
 
		  HI542MIPI_write_cmos_sensor(0x0129, iReg);  
		  
////		  HI542MIPI_write_cmos_sensor(0x0104,0x00);
   
}    /*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    LOG_INF("hi542 le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
}



static void set_mirror_flip(kal_uint8 image_mirror)
{
    kal_uint8 iTemp = 0;


	//iTemp = read_cmos_sensor((0x0011) & 0xFC);	//Clear the mirror and flip bits.
    switch (image_mirror)
	{
	    case IMAGE_NORMAL:
	        HI542MIPI_write_cmos_sensor(0x0011,  0x00);	//Set normal
			
                //SET_FIRST_GRAB_COLOR(BAYER_B);
	        break;

		case IMAGE_V_MIRROR:
			HI542MIPI_write_cmos_sensor(0x0011,  0x02);	//Set flip
			
                //SET_FIRST_GRAB_COLOR(BAYER_Gr);
			break;
			
		case IMAGE_H_MIRROR:
			HI542MIPI_write_cmos_sensor(0x0011, 0x01);	//Set mirror
                //SET_FIRST_GRAB_COLOR(BAYER_Gb);
			break;
				
	    case IMAGE_HV_MIRROR:
	        HI542MIPI_write_cmos_sensor(0x0011,  0x03);	//Set mirror and flip
                //SET_FIRST_GRAB_COLOR(BAYER_R);
	        break;
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
	//[SENSOR_INITIALIZATION]
	//DISP_DATE = "2010-02-09 13:27:00"
	//DISP_WIDTH = 1288
	//DISP_HEIGHT = 968
	//DISP_FORMAT = BAYER10
	//DISP_DATABUS = 16
	//DISP_DATAORDER = BG
	//MCLK = 24.00
	//PLL = 2.00
	
	//BEGIN
	//I2C_ID = 0x14
	//I2C_BYTE	= 0x21
	//
	////// Mode Controls
#if 0
	HI542MIPI_write_cmos_sensor(0x0014,0x42);	// B[7] Justification control (Data Justification on output Data)
	HI542MIPI_write_cmos_sensor(0x0015,0x00);	// B[7] Tris,PCLK Inverting
	HI542MIPI_write_cmos_sensor(0x0036,0x00);	// MIPI Spec --> 0.9
	HI542MIPI_write_cmos_sensor(0x0017,0x2B);	// YUV422:0x18 RAW10:0x2B RAW8:0x2A RAW12:0x2C
	HI542MIPI_write_cmos_sensor(0x0019,0x04);	// Select Data_ID by REGISTER(0x0017)
	HI542MIPI_write_cmos_sensor(0x0002,0x15);	// B[7:2] ui_x4_clk_lane (Unit interval time step 0.25ns) 3ns  0.25 X 12
	HI542MIPI_write_cmos_sensor(0x0003,0x00);	// B[4] hs_invert_clk_lane (Invert HS signals)
	HI542MIPI_write_cmos_sensor(0x0004,0x02);	// B[4] hs_rx_term_e_subLVDS_clk_lane (Enables the hs-termination for subLVDs mode)
	HI542MIPI_write_cmos_sensor(0x0005,0x03);	// B[5] force_rx_mod_data_lane (Force lane module into receive mode wait for stop state)
	HI542MIPI_write_cmos_sensor(0x0006,0x01);	// B[5] Cd_off Contention_detector on/off
	HI542MIPI_write_cmos_sensor(0x0009,0x01);	// B[5] force_rx_mod_data_lane (Force lane module into receive mode wait for stop state)
	HI542MIPI_write_cmos_sensor(0x000A,0x01);	// B[5] Cd_off Contention_detector on/off
#endif
	//I2C_ID = 0x40
	//I2C_BYTE	= 0x21
	///////////////////////////////////////////////////
	///////////////////////////////////////////////////

	
	HI542MIPI_write_cmos_sensor(0x0001,0x02);//  sw reset			 
	HI542MIPI_write_cmos_sensor(0x0001,0x01);  //sleep
	
	HI542MIPI_write_cmos_sensor(0x03d4,0x18);     //  internal   dvdd   
	mDELAY(10);
	///////////////////////////////////////////////////
	///////////////////////////////////////////////////
	
	//HI542MIPI_write_cmos_sensor(0x0010,0x00);  
	HI542MIPI_write_cmos_sensor(0x0011,0x90);  
	//HI542MIPI_write_cmos_sensor(0x0012,0x08);  
	//HI542MIPI_write_cmos_sensor(0x0013,0x00);  
	HI542MIPI_write_cmos_sensor(0x0020,0x00);  
	HI542MIPI_write_cmos_sensor(0x0021,0x00);  
	HI542MIPI_write_cmos_sensor(0x0022,0x00);  
	HI542MIPI_write_cmos_sensor(0x0023,0x00);  
	
	//HI542MIPI_write_cmos_sensor(0x0024,0x07);  
	//HI542MIPI_write_cmos_sensor(0x0025,0xA8);  
	//HI542MIPI_write_cmos_sensor(0x0026,0x0A);  
	//HI542MIPI_write_cmos_sensor(0x0027,0xB0);  
	
	HI542MIPI_write_cmos_sensor(0x0038,0x02);  
	HI542MIPI_write_cmos_sensor(0x0039,0x2C); 
	
	//HI542MIPI_write_cmos_sensor(0x003A,0x02);  
	//HI542MIPI_write_cmos_sensor(0x003B,0x2C);  
	
	HI542MIPI_write_cmos_sensor(0x003C,0x00);  
	HI542MIPI_write_cmos_sensor(0x003D,0x0C);  
	HI542MIPI_write_cmos_sensor(0x003E,0x00);  
	HI542MIPI_write_cmos_sensor(0x003F,0x0C);  
	HI542MIPI_write_cmos_sensor(0x0040,0x00);  //Hblank H
	HI542MIPI_write_cmos_sensor(0x0041,0x35);  //2E} Hblank L
//changed by zhiqiang original 35

	HI542MIPI_write_cmos_sensor(0x0042,0x00);  
	HI542MIPI_write_cmos_sensor(0x0043,0x14);  
	HI542MIPI_write_cmos_sensor(0x0045,0x07);  
	HI542MIPI_write_cmos_sensor(0x0046,0x01);  
	HI542MIPI_write_cmos_sensor(0x0047,0xD0);  
	
	//HI542MIPI_write_cmos_sensor(0x004A,0x02);   
	//HI542MIPI_write_cmos_sensor(0x004B,0xD8);    
	//HI542MIPI_write_cmos_sensor(0x004C,0x05);  
	//HI542MIPI_write_cmos_sensor(0x004D,0x08);  

	HI542MIPI_write_cmos_sensor(0x0050,0x00);  
	HI542MIPI_write_cmos_sensor(0x0052,0x10);  
	HI542MIPI_write_cmos_sensor(0x0053,0x10);  
	HI542MIPI_write_cmos_sensor(0x0054,0x10);  
	HI542MIPI_write_cmos_sensor(0x0055,0x08);  
	HI542MIPI_write_cmos_sensor(0x0056,0x80);  
	HI542MIPI_write_cmos_sensor(0x0057,0x08);  
	HI542MIPI_write_cmos_sensor(0x0058,0x08);  
	HI542MIPI_write_cmos_sensor(0x0059,0x08);  
	HI542MIPI_write_cmos_sensor(0x005A,0x08);  
	HI542MIPI_write_cmos_sensor(0x005B,0x02);  
	HI542MIPI_write_cmos_sensor(0x0070,0x03);	//EMI OFF
	
	//HI542MIPI_write_cmos_sensor(0x0080,0xC0);  
	HI542MIPI_write_cmos_sensor(0x0081,0x01);  //09},//0B},BLC scheme
	HI542MIPI_write_cmos_sensor(0x0082,0x23);  
	HI542MIPI_write_cmos_sensor(0x0083,0x00);  

	//HI542MIPI_write_cmos_sensor(0x0084,0x30);  
	HI542MIPI_write_cmos_sensor(0x0085,0x00);  
	HI542MIPI_write_cmos_sensor(0x0086,0x00);  
	HI542MIPI_write_cmos_sensor(0x008C,0x02);  
	//HI542MIPI_write_cmos_sensor(0x008D,0xFA);  
	//HI542MIPI_write_cmos_sensor(0x0090,0x0b);  
	HI542MIPI_write_cmos_sensor(0x00A0,0x0f);  //0C},//0B},RAMP DC OFFSET
	HI542MIPI_write_cmos_sensor(0x00A1,0x00);  
	HI542MIPI_write_cmos_sensor(0x00A2,0x00);  
	HI542MIPI_write_cmos_sensor(0x00A3,0x00);  
	HI542MIPI_write_cmos_sensor(0x00A4,0xFF);  
	HI542MIPI_write_cmos_sensor(0x00A5,0x00);  
	HI542MIPI_write_cmos_sensor(0x00A6,0x00);  
	//HI542MIPI_write_cmos_sensor(0x00A7,0xa6);  
	HI542MIPI_write_cmos_sensor(0x00A8,0x7F);  
	HI542MIPI_write_cmos_sensor(0x00A9,0x7F);  
	HI542MIPI_write_cmos_sensor(0x00AA,0x7F);  
	HI542MIPI_write_cmos_sensor(0x00B4,0x00);  //08},BLC offset
	HI542MIPI_write_cmos_sensor(0x00B5,0x00);  //08},
	HI542MIPI_write_cmos_sensor(0x00B6,0x02);  //07},
	HI542MIPI_write_cmos_sensor(0x00B7,0x01);  //07},
	HI542MIPI_write_cmos_sensor(0x00D4,0x00);  
	HI542MIPI_write_cmos_sensor(0x00D5,0xaa);  //a9},RAMP T1
	HI542MIPI_write_cmos_sensor(0x00D6,0x01);  
	HI542MIPI_write_cmos_sensor(0x00D7,0xc9);  
	HI542MIPI_write_cmos_sensor(0x00D8,0x05);  
	HI542MIPI_write_cmos_sensor(0x00D9,0x59);  
	HI542MIPI_write_cmos_sensor(0x00DA,0x00);  
	HI542MIPI_write_cmos_sensor(0x00DB,0xb0);  
	HI542MIPI_write_cmos_sensor(0x00DC,0x01);  
	HI542MIPI_write_cmos_sensor(0x00DD,0xc9);  //c5},
	//HI542MIPI_write_cmos_sensor(0x0119,0x00);  
	//HI542MIPI_write_cmos_sensor(0x011A,0x00);  
	//HI542MIPI_write_cmos_sensor(0x011B,0x00);  
	HI542MIPI_write_cmos_sensor(0x011C,0x1F);  
	HI542MIPI_write_cmos_sensor(0x011D,0xFF);  
	HI542MIPI_write_cmos_sensor(0x011E,0xFF);  
	HI542MIPI_write_cmos_sensor(0x011F,0xFF);  
	
	//HI542MIPI_write_cmos_sensor(0x0115,0x00);  
	//HI542MIPI_write_cmos_sensor(0x0116,0x16);  
	//HI542MIPI_write_cmos_sensor(0x0117,0x27);  
	//HI542MIPI_write_cmos_sensor(0x0118,0xE0);  
	HI542MIPI_write_cmos_sensor(0x012A,0xFF);  
	HI542MIPI_write_cmos_sensor(0x012B,0x00);  
	HI542MIPI_write_cmos_sensor(0x0129,0x40);  
	HI542MIPI_write_cmos_sensor(0x0210,0x00);  
	HI542MIPI_write_cmos_sensor(0x0212,0x00);  
	HI542MIPI_write_cmos_sensor(0x0213,0x00);  
	HI542MIPI_write_cmos_sensor(0x0216,0x00);  
	//HI542MIPI_write_cmos_sensor(0x0217,0x40);  
	//HI542MIPI_write_cmos_sensor(0x0218,0x00);  
	HI542MIPI_write_cmos_sensor(0x0219,0x33);  //66},Pixel bias
	//HI542MIPI_write_cmos_sensor(0x021A,0x15);  //15},
	HI542MIPI_write_cmos_sensor(0x021B,0x55);  
	HI542MIPI_write_cmos_sensor(0x021C,0x85);  
	HI542MIPI_write_cmos_sensor(0x021D,0xFF);  
	HI542MIPI_write_cmos_sensor(0x021E,0x01);  
	HI542MIPI_write_cmos_sensor(0x021F,0x00);  
	HI542MIPI_write_cmos_sensor(0x0220,0x02);  
	HI542MIPI_write_cmos_sensor(0x0221,0x00);  
	HI542MIPI_write_cmos_sensor(0x0222,0xA0);  
	
	//HI542MIPI_write_cmos_sensor(0x0223,0x2D);  
	//HI542MIPI_write_cmos_sensor(0x0224,0x24);  
	//HI542MIPI_write_cmos_sensor(0x0225,0x00);  
	//HI542MIPI_write_cmos_sensor(0x0226,0x3F);  
	HI542MIPI_write_cmos_sensor(0x0227,0x0A);  
	HI542MIPI_write_cmos_sensor(0x0228,0x5C);  
	HI542MIPI_write_cmos_sensor(0x0229,0x2d);  //41},//00},//2C},RAMP swing range
	HI542MIPI_write_cmos_sensor(0x022A,0x04);  
	
	//HI542MIPI_write_cmos_sensor(0x022B,0x9f);  
	HI542MIPI_write_cmos_sensor(0x022C,0x01);  
	HI542MIPI_write_cmos_sensor(0x022D,0x23);  
	//HI542MIPI_write_cmos_sensor(0x0232,0x10);  
	HI542MIPI_write_cmos_sensor(0x0237,0x00);  
	HI542MIPI_write_cmos_sensor(0x0238,0x00);  
	HI542MIPI_write_cmos_sensor(0x0239,0xA5);  
	HI542MIPI_write_cmos_sensor(0x023A,0x20);  
	HI542MIPI_write_cmos_sensor(0x023B,0x00);  
	HI542MIPI_write_cmos_sensor(0x023C,0x22);  
	
	//HI542MIPI_write_cmos_sensor(0x023E,0x00);  
	HI542MIPI_write_cmos_sensor(0x023F,0x80);  
	HI542MIPI_write_cmos_sensor(0x0240,0x04);  
	HI542MIPI_write_cmos_sensor(0x0241,0x07);  
	HI542MIPI_write_cmos_sensor(0x0242,0x00);  
	HI542MIPI_write_cmos_sensor(0x0243,0x01);  
	HI542MIPI_write_cmos_sensor(0x0244,0x80);  
	HI542MIPI_write_cmos_sensor(0x0245,0xE0);  
	HI542MIPI_write_cmos_sensor(0x0246,0x00);  
	HI542MIPI_write_cmos_sensor(0x0247,0x00);  
	HI542MIPI_write_cmos_sensor(0x024A,0x00);  
	HI542MIPI_write_cmos_sensor(0x024B,0x14);  
	HI542MIPI_write_cmos_sensor(0x024D,0x00);  
	HI542MIPI_write_cmos_sensor(0x024E,0x03);  
	HI542MIPI_write_cmos_sensor(0x024F,0x00);  
	HI542MIPI_write_cmos_sensor(0x0250,0x53);  
	HI542MIPI_write_cmos_sensor(0x0251,0x00);  
	HI542MIPI_write_cmos_sensor(0x0252,0x07);  
	HI542MIPI_write_cmos_sensor(0x0253,0x00);  
	HI542MIPI_write_cmos_sensor(0x0254,0x4F);  
	HI542MIPI_write_cmos_sensor(0x0255,0x00);  
	HI542MIPI_write_cmos_sensor(0x0256,0x07);  
	HI542MIPI_write_cmos_sensor(0x0257,0x00);  
	HI542MIPI_write_cmos_sensor(0x0258,0x4F);  
	HI542MIPI_write_cmos_sensor(0x0259,0x0C);  
	HI542MIPI_write_cmos_sensor(0x025A,0x0C);  
	HI542MIPI_write_cmos_sensor(0x025B,0x0C);  
	HI542MIPI_write_cmos_sensor(0x026C,0x00);  
	HI542MIPI_write_cmos_sensor(0x026D,0x09);  
	HI542MIPI_write_cmos_sensor(0x026E,0x00);  
	HI542MIPI_write_cmos_sensor(0x026F,0x4B);  
	HI542MIPI_write_cmos_sensor(0x0270,0x00);  
	HI542MIPI_write_cmos_sensor(0x0271,0x09);  
	HI542MIPI_write_cmos_sensor(0x0272,0x00);  
	HI542MIPI_write_cmos_sensor(0x0273,0x4B);  
	HI542MIPI_write_cmos_sensor(0x0274,0x00);  
	HI542MIPI_write_cmos_sensor(0x0275,0x09);  
	HI542MIPI_write_cmos_sensor(0x0276,0x00);  
	HI542MIPI_write_cmos_sensor(0x0277,0x4B);  
	HI542MIPI_write_cmos_sensor(0x0278,0x00);  
	HI542MIPI_write_cmos_sensor(0x0279,0x01);  
	HI542MIPI_write_cmos_sensor(0x027A,0x00);  
	HI542MIPI_write_cmos_sensor(0x027B,0x55);  
	HI542MIPI_write_cmos_sensor(0x027C,0x00);  
	HI542MIPI_write_cmos_sensor(0x027D,0x00);  
	HI542MIPI_write_cmos_sensor(0x027E,0x05);  
	HI542MIPI_write_cmos_sensor(0x027F,0x5E);  
	HI542MIPI_write_cmos_sensor(0x0280,0x00);  
	HI542MIPI_write_cmos_sensor(0x0281,0x03);  
	HI542MIPI_write_cmos_sensor(0x0282,0x00);  
	HI542MIPI_write_cmos_sensor(0x0283,0x45);  
	HI542MIPI_write_cmos_sensor(0x0284,0x00);  
	HI542MIPI_write_cmos_sensor(0x0285,0x03);  
	HI542MIPI_write_cmos_sensor(0x0286,0x00);  
	HI542MIPI_write_cmos_sensor(0x0287,0x45);  
	HI542MIPI_write_cmos_sensor(0x0288,0x05);  
	HI542MIPI_write_cmos_sensor(0x0289,0x5c);  
	HI542MIPI_write_cmos_sensor(0x028A,0x05);  
	HI542MIPI_write_cmos_sensor(0x028B,0x60);  
	HI542MIPI_write_cmos_sensor(0x02A0,0x01);  
	HI542MIPI_write_cmos_sensor(0x02A1,0xe0);  
	HI542MIPI_write_cmos_sensor(0x02A2,0x02);  
	HI542MIPI_write_cmos_sensor(0x02A3,0x22);  
	HI542MIPI_write_cmos_sensor(0x02A4,0x05);  
	HI542MIPI_write_cmos_sensor(0x02A5,0x5C);  
	HI542MIPI_write_cmos_sensor(0x02A6,0x05);  
	HI542MIPI_write_cmos_sensor(0x02A7,0x60);  
	HI542MIPI_write_cmos_sensor(0x02A8,0x05);  
	HI542MIPI_write_cmos_sensor(0x02A9,0x5C);  
	HI542MIPI_write_cmos_sensor(0x02AA,0x05);  
	HI542MIPI_write_cmos_sensor(0x02AB,0x60);  
	HI542MIPI_write_cmos_sensor(0x02D2,0x0F);  
	HI542MIPI_write_cmos_sensor(0x02DB,0x00);  
	HI542MIPI_write_cmos_sensor(0x02DC,0x00);  
	HI542MIPI_write_cmos_sensor(0x02DD,0x00);  
	HI542MIPI_write_cmos_sensor(0x02DE,0x0C);  
	HI542MIPI_write_cmos_sensor(0x02DF,0x00);  
	HI542MIPI_write_cmos_sensor(0x02E0,0x04);  
	HI542MIPI_write_cmos_sensor(0x02E1,0x00);  
	HI542MIPI_write_cmos_sensor(0x02E2,0x00);  
	HI542MIPI_write_cmos_sensor(0x02E3,0x00);  
	HI542MIPI_write_cmos_sensor(0x02E4,0x0F);  
	HI542MIPI_write_cmos_sensor(0x02F0,0x05);  
	HI542MIPI_write_cmos_sensor(0x02F1,0x05);  
	HI542MIPI_write_cmos_sensor(0x0310,0x00);  
	HI542MIPI_write_cmos_sensor(0x0311,0x01);  
	HI542MIPI_write_cmos_sensor(0x0312,0x05);  
	HI542MIPI_write_cmos_sensor(0x0313,0x5A);  
	HI542MIPI_write_cmos_sensor(0x0314,0x00);  
	HI542MIPI_write_cmos_sensor(0x0315,0x01);  
	HI542MIPI_write_cmos_sensor(0x0316,0x05);  
	HI542MIPI_write_cmos_sensor(0x0317,0x5A);  
	HI542MIPI_write_cmos_sensor(0x0318,0x00);  
	HI542MIPI_write_cmos_sensor(0x0319,0x05);  
	HI542MIPI_write_cmos_sensor(0x031A,0x00);  
	HI542MIPI_write_cmos_sensor(0x031B,0x2F);  
	HI542MIPI_write_cmos_sensor(0x031C,0x00);  
	HI542MIPI_write_cmos_sensor(0x031D,0x05);  
	HI542MIPI_write_cmos_sensor(0x031E,0x00);  
	HI542MIPI_write_cmos_sensor(0x031F,0x2F);  
	HI542MIPI_write_cmos_sensor(0x0320,0x00);  
	HI542MIPI_write_cmos_sensor(0x0321,0xAB);  
	HI542MIPI_write_cmos_sensor(0x0322,0x02);  
	HI542MIPI_write_cmos_sensor(0x0323,0x55);  
	HI542MIPI_write_cmos_sensor(0x0324,0x00);  
	HI542MIPI_write_cmos_sensor(0x0325,0xAB);  
	HI542MIPI_write_cmos_sensor(0x0326,0x02);  
	HI542MIPI_write_cmos_sensor(0x0327,0x55);  
	HI542MIPI_write_cmos_sensor(0x0328,0x00);  
	HI542MIPI_write_cmos_sensor(0x0329,0x01);  
	HI542MIPI_write_cmos_sensor(0x032A,0x00);  
	HI542MIPI_write_cmos_sensor(0x032B,0x10);  
	HI542MIPI_write_cmos_sensor(0x032C,0x00);  
	HI542MIPI_write_cmos_sensor(0x032D,0x01);  
	HI542MIPI_write_cmos_sensor(0x032E,0x00);  
	HI542MIPI_write_cmos_sensor(0x032F,0x10);  
	HI542MIPI_write_cmos_sensor(0x0330,0x00);  
	HI542MIPI_write_cmos_sensor(0x0331,0x02);  
	HI542MIPI_write_cmos_sensor(0x0332,0x00);  
	HI542MIPI_write_cmos_sensor(0x0333,0x2e);  
	HI542MIPI_write_cmos_sensor(0x0334,0x00);  
	HI542MIPI_write_cmos_sensor(0x0335,0x02);  
	HI542MIPI_write_cmos_sensor(0x0336,0x00);  
	HI542MIPI_write_cmos_sensor(0x0337,0x2e);  
	HI542MIPI_write_cmos_sensor(0x0358,0x00);  
	HI542MIPI_write_cmos_sensor(0x0359,0x46);  
	HI542MIPI_write_cmos_sensor(0x035A,0x05);  
	HI542MIPI_write_cmos_sensor(0x035B,0x59);  
	HI542MIPI_write_cmos_sensor(0x035C,0x00);  
	HI542MIPI_write_cmos_sensor(0x035D,0x46);  
	HI542MIPI_write_cmos_sensor(0x035E,0x05);  
	HI542MIPI_write_cmos_sensor(0x035F,0x59);  
	HI542MIPI_write_cmos_sensor(0x0360,0x00);  
	HI542MIPI_write_cmos_sensor(0x0361,0x46);  
	HI542MIPI_write_cmos_sensor(0x0362,0x00);  
	HI542MIPI_write_cmos_sensor(0x0363,0xa4);  //a2},Black sun
	HI542MIPI_write_cmos_sensor(0x0364,0x00);  
	HI542MIPI_write_cmos_sensor(0x0365,0x46);  
	HI542MIPI_write_cmos_sensor(0x0366,0x00);  
	HI542MIPI_write_cmos_sensor(0x0367,0xa4);  //a2},Black sun
	HI542MIPI_write_cmos_sensor(0x0368,0x00);  
	HI542MIPI_write_cmos_sensor(0x0369,0x46);  
	HI542MIPI_write_cmos_sensor(0x036A,0x00);  
	HI542MIPI_write_cmos_sensor(0x036B,0xa6);  //a9},S2 off
	HI542MIPI_write_cmos_sensor(0x036C,0x00);  
	HI542MIPI_write_cmos_sensor(0x036D,0x46);  
	HI542MIPI_write_cmos_sensor(0x036E,0x00);  
	HI542MIPI_write_cmos_sensor(0x036F,0xa6);  //a9},S2 off
	HI542MIPI_write_cmos_sensor(0x0370,0x00);  
	HI542MIPI_write_cmos_sensor(0x0371,0xb0);  
	HI542MIPI_write_cmos_sensor(0x0372,0x05);  
	HI542MIPI_write_cmos_sensor(0x0373,0x59);  
	HI542MIPI_write_cmos_sensor(0x0374,0x00);  
	HI542MIPI_write_cmos_sensor(0x0375,0xb0);  
	HI542MIPI_write_cmos_sensor(0x0376,0x05);  
	HI542MIPI_write_cmos_sensor(0x0377,0x59);  
	HI542MIPI_write_cmos_sensor(0x0378,0x00);  
	HI542MIPI_write_cmos_sensor(0x0379,0x45);  
	HI542MIPI_write_cmos_sensor(0x037A,0x00);  
	HI542MIPI_write_cmos_sensor(0x037B,0xAA);  
	HI542MIPI_write_cmos_sensor(0x037C,0x00);  
	HI542MIPI_write_cmos_sensor(0x037D,0x99);  
	HI542MIPI_write_cmos_sensor(0x037E,0x01);  
	HI542MIPI_write_cmos_sensor(0x037F,0xAE);  
	HI542MIPI_write_cmos_sensor(0x0380,0x01);  
	HI542MIPI_write_cmos_sensor(0x0381,0xB1);  
	HI542MIPI_write_cmos_sensor(0x0382,0x02);  
	HI542MIPI_write_cmos_sensor(0x0383,0x56);  
	HI542MIPI_write_cmos_sensor(0x0384,0x05);  
	HI542MIPI_write_cmos_sensor(0x0385,0x6D);  
	HI542MIPI_write_cmos_sensor(0x0386,0x00);  
	HI542MIPI_write_cmos_sensor(0x0387,0xDC);  
	HI542MIPI_write_cmos_sensor(0x03A0,0x05);  
	HI542MIPI_write_cmos_sensor(0x03A1,0x5E);  
	HI542MIPI_write_cmos_sensor(0x03A2,0x05);  
	HI542MIPI_write_cmos_sensor(0x03A3,0x62);  
	HI542MIPI_write_cmos_sensor(0x03A4,0x01);  
	HI542MIPI_write_cmos_sensor(0x03A5,0xc9);  
	HI542MIPI_write_cmos_sensor(0x03A6,0x01);  
	HI542MIPI_write_cmos_sensor(0x03A7,0x27);  
	HI542MIPI_write_cmos_sensor(0x03A8,0x05);  
	HI542MIPI_write_cmos_sensor(0x03A9,0x59);  
	HI542MIPI_write_cmos_sensor(0x03AA,0x02);  
	HI542MIPI_write_cmos_sensor(0x03AB,0x55);  
	HI542MIPI_write_cmos_sensor(0x03AC,0x01);  
	HI542MIPI_write_cmos_sensor(0x03AD,0xc5);  
	HI542MIPI_write_cmos_sensor(0x03AE,0x01);  
	HI542MIPI_write_cmos_sensor(0x03AF,0x27);  
	HI542MIPI_write_cmos_sensor(0x03B0,0x05);  
	HI542MIPI_write_cmos_sensor(0x03B1,0x55);  
	HI542MIPI_write_cmos_sensor(0x03B2,0x02);  
	HI542MIPI_write_cmos_sensor(0x03B3,0x55);  
	HI542MIPI_write_cmos_sensor(0x03B4,0x00);  
	HI542MIPI_write_cmos_sensor(0x03B5,0x0A);  
	
	//HI542MIPI_write_cmos_sensor(0x03D0,0xee);  
	//HI542MIPI_write_cmos_sensor(0x03D1,0x15);  
	//HI542MIPI_write_cmos_sensor(0x03D2,0xb0);  
	HI542MIPI_write_cmos_sensor(0x03D3,0x08);  
	//HI542MIPI_write_cmos_sensor(0x03D4,0x18);  //08},LDO OUTPUT 
	HI542MIPI_write_cmos_sensor(0x03D5,0x44);  
	HI542MIPI_write_cmos_sensor(0x03D6,0x51);  
	HI542MIPI_write_cmos_sensor(0x03D7,0x56);  
	HI542MIPI_write_cmos_sensor(0x03D8,0x44);  
	HI542MIPI_write_cmos_sensor(0x03D9,0x06);  

	//HI542MIPI_write_cmos_sensor(0x0500,0x18);  
	HI542MIPI_write_cmos_sensor(0x0580,0x01);  
	HI542MIPI_write_cmos_sensor(0x0581,0x00);  
	HI542MIPI_write_cmos_sensor(0x0582,0x80);  
	HI542MIPI_write_cmos_sensor(0x0583,0x00);  
	HI542MIPI_write_cmos_sensor(0x0584,0x80);  
	HI542MIPI_write_cmos_sensor(0x0585,0x00);  
	HI542MIPI_write_cmos_sensor(0x0586,0x80);  
	HI542MIPI_write_cmos_sensor(0x0587,0x00);  
	HI542MIPI_write_cmos_sensor(0x0588,0x80);  
	HI542MIPI_write_cmos_sensor(0x0589,0x00);  
	HI542MIPI_write_cmos_sensor(0x058A,0x80);  
	
	//HI542MIPI_write_cmos_sensor(0x05A0,0x01);  
	//HI542MIPI_write_cmos_sensor(0x05B0,0x01);  
	HI542MIPI_write_cmos_sensor(0x05C2,0x00);  
	HI542MIPI_write_cmos_sensor(0x05C3,0x00);  
	HI542MIPI_write_cmos_sensor(0x0080,0xC7);  
	HI542MIPI_write_cmos_sensor(0x0119,0x00);  
	HI542MIPI_write_cmos_sensor(0x011A,0x15);  
	HI542MIPI_write_cmos_sensor(0x011B,0xC0);  
/*
	HI542MIPI_write_cmos_sensor(0x0115,0x00);  
	HI542MIPI_write_cmos_sensor(0x0116,0x2A);  
	HI542MIPI_write_cmos_sensor(0x0117,0x4C);  
	HI542MIPI_write_cmos_sensor(0x0118,0x20);  
*/
	HI542MIPI_write_cmos_sensor(0x0223,0xED);  
	HI542MIPI_write_cmos_sensor(0x0224,0xE4);  
	HI542MIPI_write_cmos_sensor(0x0225,0x09);  
	HI542MIPI_write_cmos_sensor(0x0226,0x36);  
	HI542MIPI_write_cmos_sensor(0x023E,0x80);  
	HI542MIPI_write_cmos_sensor(0x05B0,0x00);  
	//HI542MIPI_write_cmos_sensor(0x03D0,0xe9);  
	//HI542MIPI_write_cmos_sensor(0x03D1,0x75);  
	HI542MIPI_write_cmos_sensor(0x03D2,0xAD);

	HI542MIPI_write_cmos_sensor(0x0616,0x00);
	HI542MIPI_write_cmos_sensor(0x0616,0x01);
	HI542MIPI_write_cmos_sensor(0x03D2,0xAC);
	HI542MIPI_write_cmos_sensor(0x03D0,0xe9);  
	HI542MIPI_write_cmos_sensor(0x03D1,0x75);  


	
	HI542MIPI_write_cmos_sensor(0x0800,0x07);  //07},//0F},EMI disable
	HI542MIPI_write_cmos_sensor(0x0801,0x08);  
	HI542MIPI_write_cmos_sensor(0x0802,0x02);  //04},//00},apb clock speed down
	//HI542MIPI_write_cmos_sensor(0x0010,0x05);  
	HI542MIPI_write_cmos_sensor(0x0012,0x00);  
	HI542MIPI_write_cmos_sensor(0x0013,0x00);  
	HI542MIPI_write_cmos_sensor(0x0024,0x07);  
	HI542MIPI_write_cmos_sensor(0x0025,0xA8);  
	HI542MIPI_write_cmos_sensor(0x0026,0x0A);  
	HI542MIPI_write_cmos_sensor(0x0027,0x30);  
	HI542MIPI_write_cmos_sensor(0x0030,0x00);  
	HI542MIPI_write_cmos_sensor(0x0031,0x03);  
	HI542MIPI_write_cmos_sensor(0x0032,0x07);  
	HI542MIPI_write_cmos_sensor(0x0033,0xAC);  
	HI542MIPI_write_cmos_sensor(0x0034,0x03);  
	HI542MIPI_write_cmos_sensor(0x0035,0xD4);  
	HI542MIPI_write_cmos_sensor(0x003A,0x00);  
	HI542MIPI_write_cmos_sensor(0x003B,0x2E);  
	HI542MIPI_write_cmos_sensor(0x004A,0x03);  
	HI542MIPI_write_cmos_sensor(0x004B,0xD4);  
	HI542MIPI_write_cmos_sensor(0x004C,0x05);  
	HI542MIPI_write_cmos_sensor(0x004D,0x18);  
	HI542MIPI_write_cmos_sensor(0x0C98,0x05);  
	HI542MIPI_write_cmos_sensor(0x0C99,0x5E);  
	HI542MIPI_write_cmos_sensor(0x0C9A,0x05);  
	HI542MIPI_write_cmos_sensor(0x0C9B,0x62);
	
	//HI542MIPI_write_cmos_sensor(0x0500,0x18);  
	HI542MIPI_write_cmos_sensor(0x05A0,0x01);  
	HI542MIPI_write_cmos_sensor(0x0084,0x30);  //10},BLC control
	HI542MIPI_write_cmos_sensor(0x008D,0xFF);  
	HI542MIPI_write_cmos_sensor(0x0090,0x02);  //0b},BLC defect pixel th
	HI542MIPI_write_cmos_sensor(0x00A7,0x80);  //FF},
	HI542MIPI_write_cmos_sensor(0x021A,0x15);  
	HI542MIPI_write_cmos_sensor(0x022B,0xb0);  //f0},RAMP filter
	HI542MIPI_write_cmos_sensor(0x0232,0x37);  //17},black sun enable
	HI542MIPI_write_cmos_sensor(0x0010,0x41);  //01},
	HI542MIPI_write_cmos_sensor(0x0740,0x1A);  
	HI542MIPI_write_cmos_sensor(0x0742,0x1A);  
	HI542MIPI_write_cmos_sensor(0x0743,0x1A);  
	HI542MIPI_write_cmos_sensor(0x0744,0x1A);  
	HI542MIPI_write_cmos_sensor(0x0745,0x04);  
	HI542MIPI_write_cmos_sensor(0x0746,0x32);  
	HI542MIPI_write_cmos_sensor(0x0747,0x05);  
	HI542MIPI_write_cmos_sensor(0x0748,0x01);  
	HI542MIPI_write_cmos_sensor(0x0749,0x90);  
	HI542MIPI_write_cmos_sensor(0x074A,0x1A);  
	HI542MIPI_write_cmos_sensor(0x074B,0xB1);							 
	HI542MIPI_write_cmos_sensor(0x0500,0x19);  //0x19},//1b},LSC disable
	HI542MIPI_write_cmos_sensor(0x0510,0x10);  
									   
	HI542MIPI_write_cmos_sensor(0x0217,0x44);  //adaptive NCP on
	HI542MIPI_write_cmos_sensor(0x0218,0x00);  //scn_sel
									   
	HI542MIPI_write_cmos_sensor(0x02ac,0x00);  //outdoor on
	HI542MIPI_write_cmos_sensor(0x02ad,0x00);  
	HI542MIPI_write_cmos_sensor(0x02ae,0x00);  //outdoor off
	HI542MIPI_write_cmos_sensor(0x02af,0x00);  
	HI542MIPI_write_cmos_sensor(0x02b0,0x00);  //indoor on
	HI542MIPI_write_cmos_sensor(0x02b1,0x00);  
	HI542MIPI_write_cmos_sensor(0x02b2,0x00);  //indoor off
	HI542MIPI_write_cmos_sensor(0x02b3,0x00);  
	HI542MIPI_write_cmos_sensor(0x02b4,0x60);  //dark1 on
	HI542MIPI_write_cmos_sensor(0x02b5,0x21);  
	HI542MIPI_write_cmos_sensor(0x02b6,0x66);  //dark1 off
	HI542MIPI_write_cmos_sensor(0x02b7,0x8a);  
									   
	HI542MIPI_write_cmos_sensor(0x02c0,0x36);  //outdoor NCP en
	HI542MIPI_write_cmos_sensor(0x02c1,0x36);  //indoor NCP en
	HI542MIPI_write_cmos_sensor(0x02c2,0x36);  //dark1 NCP en
	HI542MIPI_write_cmos_sensor(0x02c3,0x36);  //3f},//dark2 NCP disable
	HI542MIPI_write_cmos_sensor(0x02c4,0xE4);  //outdoor NCP voltage
	HI542MIPI_write_cmos_sensor(0x02c5,0xE4);  //indoor NCP voltage
	HI542MIPI_write_cmos_sensor(0x02c6,0xE4);  //dark1 NCP voltage
	HI542MIPI_write_cmos_sensor(0x02c7,0xdb);  //24},//dark2 NCP voltage

	//#if define MIPI_INTERFACE
	HI542MIPI_write_cmos_sensor(0x061A,0x01);  
	HI542MIPI_write_cmos_sensor(0x061B,0x03);  
	HI542MIPI_write_cmos_sensor(0x061C,0x00);  
	HI542MIPI_write_cmos_sensor(0x061D,0x00);  
	HI542MIPI_write_cmos_sensor(0x061E,0x00);  
	HI542MIPI_write_cmos_sensor(0x061F,0x03);  
	HI542MIPI_write_cmos_sensor(0x0613,0x01);  
	HI542MIPI_write_cmos_sensor(0x0615,0x01);  
	HI542MIPI_write_cmos_sensor(0x0616,0x01);  
	HI542MIPI_write_cmos_sensor(0x0617,0x00);  
	HI542MIPI_write_cmos_sensor(0x0619,0x00); 

	HI542MIPI_write_cmos_sensor(0x0661,0x03);  
	HI542MIPI_write_cmos_sensor(0x0650,0x03);  
	HI542MIPI_write_cmos_sensor(0x0651,0x02);  
	HI542MIPI_write_cmos_sensor(0x0652,0x10);  
	HI542MIPI_write_cmos_sensor(0x0655,0x04);  
	HI542MIPI_write_cmos_sensor(0x0654,0x08); 

	
	HI542MIPI_write_cmos_sensor(0x0008,0x0F);  
	HI542MIPI_write_cmos_sensor(0x0630,0x05);  
	HI542MIPI_write_cmos_sensor(0x0631,0x18);  
	HI542MIPI_write_cmos_sensor(0x0632,0x03);  
	HI542MIPI_write_cmos_sensor(0x0633,0xD4);  
	HI542MIPI_write_cmos_sensor(0x0663,0x05);  //0a},trail time 
	HI542MIPI_write_cmos_sensor(0x0660,0x03);  	
	
	
	HI542MIPI_write_cmos_sensor(0x0001,0x00);	//PWRCTLB  
	
	//END
	//[END]
	}    /*    sensor_init  */


static void preview_setting(void)
{
    
	
	HI542MIPI_write_cmos_sensor(0x0001, 0x01);	  
	HI542MIPI_write_cmos_sensor(0x0617, 0x01);
	HI542MIPI_write_cmos_sensor(0x0001, 0x00);
	HI542MIPI_write_cmos_sensor(0x0001, 0x01);
	HI542MIPI_write_cmos_sensor(0x0617, 0x00);
	
	HI542MIPI_write_cmos_sensor(0x0010, 0x41);
	HI542MIPI_write_cmos_sensor(0x0011, 0x04);
	HI542MIPI_write_cmos_sensor(0x0034, 0x03);
	HI542MIPI_write_cmos_sensor(0x0035, 0xD4);
	HI542MIPI_write_cmos_sensor(0x0040, 0x00);
	HI542MIPI_write_cmos_sensor(0x0041, 0x35);
//changed by zhiqiang

	HI542MIPI_write_cmos_sensor(0x0042, 0x00);
	HI542MIPI_write_cmos_sensor(0x0043, 0x14);
	HI542MIPI_write_cmos_sensor(0x0500, 0x19);

	HI542MIPI_write_cmos_sensor(0x0630, 0x05);
	HI542MIPI_write_cmos_sensor(0x0631, 0x18);//08},
	HI542MIPI_write_cmos_sensor(0x0632, 0x03);
	HI542MIPI_write_cmos_sensor(0x0633, 0xD4);//C8},
#if 0
	HI542MIPI_write_cmos_sensor(0x0011, 0x03); //fixed frame setting turn off
	HI542MIPI_write_cmos_sensor(0x0013, 0x00); //fixed frame setting turn off
#else
HI542MIPI_write_cmos_sensor(0x0011, 0x04); //fixed frame setting turn off
HI542MIPI_write_cmos_sensor(0x0013, 0x40); //fixed frame setting turn off

#endif



	
	HI542MIPI_write_cmos_sensor(0x011C,0x1F); // Max exposure time 
	HI542MIPI_write_cmos_sensor(0x011D,0xFF); // Max exposure time 
	HI542MIPI_write_cmos_sensor(0x011E,0xFF); // Max exposure time/
	HI542MIPI_write_cmos_sensor(0x011F,0xFF); // Max exposure time 

	HI542MIPI_write_cmos_sensor(0x0001, 0x00);	 
	
	//END

}    /*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("hi542 capture_setting E! currefps:%d\n",currefps);
    if (currefps == 150) {
	
	
	
	
		//HI542MIPI_write_cmos_sensor(0x0632, 0x00);
		//HI542MIPI_write_cmos_sensor(0x0633, 0x00);
		HI542MIPI_write_cmos_sensor(0x0001, 0x01);	  
		HI542MIPI_write_cmos_sensor(0x0617, 0x01);
		HI542MIPI_write_cmos_sensor(0x0001, 0x00);
		HI542MIPI_write_cmos_sensor(0x0001, 0x01);
		HI542MIPI_write_cmos_sensor(0x0617, 0x00);
		
	//	HI542MIPI_write_cmos_sensor(0x0020, 0x00);
	//	HI542MIPI_write_cmos_sensor(0x0021, 0x00);
	//	HI542MIPI_write_cmos_sensor(0x0022, 0x00);
	//	HI542MIPI_write_cmos_sensor(0x0023, 0x00);
	
	
		HI542MIPI_write_cmos_sensor(0x0010, 0x40);
		//HI542MIPI_write_cmos_sensor(0x0011, 0x04);
		HI542MIPI_write_cmos_sensor(0x0013, 0x40);
	
		HI542MIPI_write_cmos_sensor(0x0034, 0x07);
		HI542MIPI_write_cmos_sensor(0x0035, 0xA8);
	
		HI542MIPI_write_cmos_sensor(0x0500, 0x11);
		
		HI542MIPI_write_cmos_sensor(0x0630, 0x0A);
		HI542MIPI_write_cmos_sensor(0x0631, 0x30);
		HI542MIPI_write_cmos_sensor(0x0632, 0x07);
		HI542MIPI_write_cmos_sensor(0x0633, 0xA8);
	
		HI542MIPI_write_cmos_sensor(0x0001, 0x00);	
	
	} else {
	
	
	
	
	
		//HI542MIPI_write_cmos_sensor(0x0632, 0x00);
		//HI542MIPI_write_cmos_sensor(0x0633, 0x00);
		HI542MIPI_write_cmos_sensor(0x0001, 0x01);	  
		HI542MIPI_write_cmos_sensor(0x0617, 0x01);
		HI542MIPI_write_cmos_sensor(0x0001, 0x00);
		HI542MIPI_write_cmos_sensor(0x0001, 0x01);
		HI542MIPI_write_cmos_sensor(0x0617, 0x00);
		
	//	HI542MIPI_write_cmos_sensor(0x0020, 0x00);
	//	HI542MIPI_write_cmos_sensor(0x0021, 0x00);
	//	HI542MIPI_write_cmos_sensor(0x0022, 0x00);
	//	HI542MIPI_write_cmos_sensor(0x0023, 0x00);
	
	
		HI542MIPI_write_cmos_sensor(0x0010, 0x40);
		//HI542MIPI_write_cmos_sensor(0x0011, 0x07);
		HI542MIPI_write_cmos_sensor(0x0013, 0x40);
	
		HI542MIPI_write_cmos_sensor(0x0034, 0x07);
		HI542MIPI_write_cmos_sensor(0x0035, 0xA8);
	
		HI542MIPI_write_cmos_sensor(0x0500, 0x11);
		
		HI542MIPI_write_cmos_sensor(0x0630, 0x0A);
		HI542MIPI_write_cmos_sensor(0x0631, 0x30);
		HI542MIPI_write_cmos_sensor(0x0632, 0x07);
		HI542MIPI_write_cmos_sensor(0x0633, 0xA8);
	
		HI542MIPI_write_cmos_sensor(0x0001, 0x00);	
	
	
	
	
}

}

static void normal_video_setting(kal_uint16 currefps)
{
    
	
	HI542MIPI_write_cmos_sensor(0x0001, 0x01);	  
	HI542MIPI_write_cmos_sensor(0x0617, 0x01);
	HI542MIPI_write_cmos_sensor(0x0001, 0x00);
	HI542MIPI_write_cmos_sensor(0x0001, 0x01);
	HI542MIPI_write_cmos_sensor(0x0617, 0x00);
	
	HI542MIPI_write_cmos_sensor(0x0010, 0x41);
	//HI542MIPI_write_cmos_sensor(0x0011, 0x03);
	HI542MIPI_write_cmos_sensor(0x0034, 0x03);
	HI542MIPI_write_cmos_sensor(0x0035, 0xD4);
	HI542MIPI_write_cmos_sensor(0x0040, 0x00);
	HI542MIPI_write_cmos_sensor(0x0041, 0x35);
//changed by zhiqiang

	HI542MIPI_write_cmos_sensor(0x0042, 0x00);
	HI542MIPI_write_cmos_sensor(0x0043, 0x14);
	HI542MIPI_write_cmos_sensor(0x0500, 0x19);

	HI542MIPI_write_cmos_sensor(0x0630, 0x05);
	HI542MIPI_write_cmos_sensor(0x0631, 0x18);//08},
	HI542MIPI_write_cmos_sensor(0x0632, 0x03);
	HI542MIPI_write_cmos_sensor(0x0633, 0xD4);//C8},
#if 0
	HI542MIPI_write_cmos_sensor(0x0011, 0x03); //fixed frame setting turn off
	HI542MIPI_write_cmos_sensor(0x0013, 0x00); //fixed frame setting turn off
#else
//HI542MIPI_write_cmos_sensor(0x0011, 0x07); //fixed frame setting turn off
HI542MIPI_write_cmos_sensor(0x0013, 0x40); //fixed frame setting turn off

#endif



	
	HI542MIPI_write_cmos_sensor(0x011C,0x1F); // Max exposure time 
	HI542MIPI_write_cmos_sensor(0x011D,0xFF); // Max exposure time 
	HI542MIPI_write_cmos_sensor(0x011E,0xFF); // Max exposure time/
	HI542MIPI_write_cmos_sensor(0x011F,0xFF); // Max exposure time 

	HI542MIPI_write_cmos_sensor(0x0001, 0x00);	 
	
	//END

}

static void hs_video_setting(void)
{
    
	
	HI542MIPI_write_cmos_sensor(0x0001, 0x01);	  
	HI542MIPI_write_cmos_sensor(0x0617, 0x01);
	HI542MIPI_write_cmos_sensor(0x0001, 0x00);
	HI542MIPI_write_cmos_sensor(0x0001, 0x01);
	HI542MIPI_write_cmos_sensor(0x0617, 0x00);
	
	HI542MIPI_write_cmos_sensor(0x0010, 0x41);
	//HI542MIPI_write_cmos_sensor(0x0011, 0x03);
	HI542MIPI_write_cmos_sensor(0x0034, 0x03);
	HI542MIPI_write_cmos_sensor(0x0035, 0xD4);
	HI542MIPI_write_cmos_sensor(0x0040, 0x00);
	HI542MIPI_write_cmos_sensor(0x0041, 0x35);
//changed by zhiqiang

	HI542MIPI_write_cmos_sensor(0x0042, 0x00);
	HI542MIPI_write_cmos_sensor(0x0043, 0x14);
	HI542MIPI_write_cmos_sensor(0x0500, 0x19);

	HI542MIPI_write_cmos_sensor(0x0630, 0x05);
	HI542MIPI_write_cmos_sensor(0x0631, 0x18);//08},
	HI542MIPI_write_cmos_sensor(0x0632, 0x03);
	HI542MIPI_write_cmos_sensor(0x0633, 0xD4);//C8},
#if 0
	HI542MIPI_write_cmos_sensor(0x0011, 0x03); //fixed frame setting turn off
	HI542MIPI_write_cmos_sensor(0x0013, 0x00); //fixed frame setting turn off
#else
//HI542MIPI_write_cmos_sensor(0x0011, 0x07); //fixed frame setting turn off
HI542MIPI_write_cmos_sensor(0x0013, 0x40); //fixed frame setting turn off

#endif



	
	HI542MIPI_write_cmos_sensor(0x011C,0x1F); // Max exposure time 
	HI542MIPI_write_cmos_sensor(0x011D,0xFF); // Max exposure time 
	HI542MIPI_write_cmos_sensor(0x011E,0xFF); // Max exposure time/
	HI542MIPI_write_cmos_sensor(0x011F,0xFF); // Max exposure time 

	HI542MIPI_write_cmos_sensor(0x0001, 0x00);	 
	
	//END

}


static void slim_video_setting(void)
{
    
	
	HI542MIPI_write_cmos_sensor(0x0001, 0x01);	  
	HI542MIPI_write_cmos_sensor(0x0617, 0x01);
	HI542MIPI_write_cmos_sensor(0x0001, 0x00);
	HI542MIPI_write_cmos_sensor(0x0001, 0x01);
	HI542MIPI_write_cmos_sensor(0x0617, 0x00);
	
	HI542MIPI_write_cmos_sensor(0x0010, 0x41);
	//HI542MIPI_write_cmos_sensor(0x0011, 0x03);
	HI542MIPI_write_cmos_sensor(0x0034, 0x03);
	HI542MIPI_write_cmos_sensor(0x0035, 0xD4);
	HI542MIPI_write_cmos_sensor(0x0040, 0x00);
	HI542MIPI_write_cmos_sensor(0x0041, 0x35);
//changed by zhiqiang

	HI542MIPI_write_cmos_sensor(0x0042, 0x00);
	HI542MIPI_write_cmos_sensor(0x0043, 0x14);
	HI542MIPI_write_cmos_sensor(0x0500, 0x19);

	HI542MIPI_write_cmos_sensor(0x0630, 0x05);
	HI542MIPI_write_cmos_sensor(0x0631, 0x18);//08},
	HI542MIPI_write_cmos_sensor(0x0632, 0x03);
	HI542MIPI_write_cmos_sensor(0x0633, 0xD4);//C8},
#if 0
	HI542MIPI_write_cmos_sensor(0x0011, 0x03); //fixed frame setting turn off
	HI542MIPI_write_cmos_sensor(0x0013, 0x00); //fixed frame setting turn off
#else
//HI542MIPI_write_cmos_sensor(0x0011, 0x07); //fixed frame setting turn off
HI542MIPI_write_cmos_sensor(0x0013, 0x40); //fixed frame setting turn off

#endif



	
	HI542MIPI_write_cmos_sensor(0x011C,0x1F); // Max exposure time 
	HI542MIPI_write_cmos_sensor(0x011D,0xFF); // Max exposure time 
	HI542MIPI_write_cmos_sensor(0x011E,0xFF); // Max exposure time/
	HI542MIPI_write_cmos_sensor(0x011F,0xFF); // Max exposure time 

	HI542MIPI_write_cmos_sensor(0x0001, 0x00);	 
	
	//END

}


static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("hi542 set_test_pattern_mode  enable: %d\n", enable);

    if (enable) {
        //write_cmos_sensor(0x020a,0x0200);        

    } else {
       // write_cmos_sensor(0x020a,0x0000); 
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
	   #ifdef SLT_DEVINFO_CMM 
	char mid_info=0;
	char dev_vendorlist[0x20][20]={"null","Hynix","sunny","truly","A-kerr","LiteArray","Darling","Qtech",
		"OFlim","Huaquan","sunrise"};
 	s_DEVINFO_ccm =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);	
	s_DEVINFO_ccm->device_type = "CCM-M";//
	s_DEVINFO_ccm->device_module = "CM05AF036_V0";
	s_DEVINFO_ccm->device_vendor = "Hynix";
	s_DEVINFO_ccm->device_ic = "hi542";
	s_DEVINFO_ccm->device_version = "Hynix";
	s_DEVINFO_ccm->device_info = "500W";	
#endif

    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("hi542 i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
#ifdef SLT_DEVINFO_CMM 
				s_DEVINFO_ccm->device_used = DEVINFO_USED;
				devinfo_check_add_device(s_DEVINFO_ccm);
#endif	
                return ERROR_NONE;
            }
            LOG_INF("hi542 Read sensor id fail, write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
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
                LOG_INF("hi542 i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("hi542 Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
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
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);
printk("hi542 open end\n");
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
    LOG_INF("hi542 close E\n");

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
    LOG_INF("hi542 preview E\n");

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
//set_mirror_flip(imgsensor.mirror);
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
    LOG_INF("hi542 capture E\n");
	
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
//	set_mirror_flip(imgsensor.mirror);
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
//	set_mirror_flip(imgsensor.mirror);
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
//	set_mirror_flip(imgsensor.mirror);
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
//	set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /*    slim_video     */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("hi542 get_resolution  E\n");
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
    LOG_INF("hi542 get_info scenario_id = %d\n", scenario_id);


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
    LOG_INF("hi542 scenario_id = %d\n", scenario_id);
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
            LOG_INF("hi542 Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{//This Function not used after ROME
    LOG_INF("hi542 set_video_mode framerate = %d\n ", framerate);
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

    LOG_INF("hi542 scenario_id = %d, framerate = %d\n", scenario_id, framerate);

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

    printk("feature_id = %d\n", feature_id);
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
            HI542MIPI_write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
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
//            LOG_INF("current fps :%d\n", *feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
           // LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
           // LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data);

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

UINT32 HI542_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    
