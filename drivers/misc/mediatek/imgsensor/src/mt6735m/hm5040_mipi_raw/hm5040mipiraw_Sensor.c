/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 HM5040mipiraw_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6595
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *	PengtaoFan
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

#include "hm5040mipiraw_Sensor.h"

#define PFX "HM5040R2A"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);


static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = HM5040_SENSOR_ID,		//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value = 0xd6d43c1f,		//checksum value for Camera Auto Test
	
	.pre = {
		.pclk = 168000000,				//record different mode's pclk
		.linelength  = 2750,				//record different mode's linelength
		.framelength = 1988,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2600,		//record different mode's width of grabwindow
		.grabwindow_height = 1952,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 23,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 168000000,
		.linelength  = 2750,
		.framelength = 1988,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 2600,
		.grabwindow_height = 1952,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.cap1 = {							//capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 168000000,
		.linelength  = 2750,
		.framelength = 1988,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 2600,
		.grabwindow_height = 1952,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 240,
	},
	.normal_video = {
		.linelength  = 2750,
		.framelength = 1988,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 2600,
		.grabwindow_height = 1952,
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
	.margin = 4,			//sensor framelength & shutter margin
	.min_shutter = 4,		//min shutter
	.max_frame_length = 0x90f7,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 3,	  //support sensor mode num ,don't support Slow motion
	
	.cap_delay_frame = 6,		//enter capture delay frame num
	.pre_delay_frame = 3, 		//enter preview delay frame num
	.video_delay_frame = 3,		//enter video delay frame num
	.hs_video_delay_frame = 3,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,//enter slim video delay frame num
	
	.isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_2_LANE,//mipi lane num
	.i2c_addr_table = {0x36, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};

static struct HM5040_GAIN
{
    kal_uint16 gainvalue;
    kal_uint16 analoggain;
};

static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x4C00,					//current shutter
	.gain = 0x200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x36,//record current sensor's i2c write id
};
static struct HM5040_GAIN HM5040_gain_table[]=
{
//0x0204/0x0205
    {0x0000, 100},
    {0x0010, 106},
    {0x0020, 109},
    {0x0030, 118},
    {0x0040, 129},
    {0x0050, 150},
    {0x0060, 159},
    {0x0070, 179},
    {0x0080, 200},
    {0x0090, 229},
    {0x00A0, 268},
    {0x00B0, 318},
    {0x00C0, 400},
    {0x00D0, 532},
    {0x00E0, 800},
    {0x00E4, 909},
    {0x00E8, 1068},
    {0x00EC, 1279},
    {0x00F0, 1600},
    {0xFFFF, 0},
};

static struct HM5040_GAIN HM5040_D_gain_table[]=
{
  {0x0000, 100},
	{0x0008, 103},
	{0x0010, 106},
	{0x0018, 109},
	{0x0020, 113},
	{0x0028, 116},
	{0x0030, 119},
	{0x0038, 122},
	{0x0040, 125},
	{0x0048, 128},
	{0x0050, 131},
	{0x0058, 134},
	{0x0060, 138},
	{0x0068, 141},
	{0x0070, 144},
	{0x0078, 147},
	{0x0080, 150},
	{0x0088, 153},
	{0x0090, 156},
	{0x0098, 159},
	{0x00A0, 163},
	{0x00A8, 166},
	{0x00B0, 169},
	{0x00B8, 172},
	{0x00C0, 175},
	{0x00C8, 178},
	{0x00D0, 181},
	{0x00D8, 184},
	{0x00E0, 188},
	{0x00E8, 191},
	{0x00F0, 194},
	{0x00F8, 197},
  {0xFFFF, 0},
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
{//{ 2600, 1952,	  0,	0, 2600, 1952, 1296,  976,   0,	0, 1296, 976,	 0, 0, 1296, 976}, // Preview 
 { 2600, 1952,	  0,	0, 2600, 1952, 2600, 1952,   0,	0, 2592, 1944,	 0, 0, 2592, 1944}, // Preview 
 { 2600, 1952,	  0,	0, 2600, 1952, 2600, 1952,   0,	0, 2592, 1944,	 0, 0, 2592, 1944}, // capture 
 { 2600, 1952,	   0,  0, 2600, 1952, 2600, 1952,	0,	 0, 2592, 1944,	  0, 0, 2592, 1944}, // 5M video 
 //{ 2600, 1952,	  0,	0, 2600, 1952, 1296,  976,   0,	0, 1296, 976,	 0, 0, 1296, 976}, // video 
 { 2600, 1952,	  0,	0, 2600, 1952, 1296,  976,   0,	0, 1296, 976,	 0, 0, 1296, 976}, //hight speed video 
 { 2600, 1952,	  0,	0, 2600, 1952, 1296,  976,   0,	0, 1296, 976,	 0, 0, 1296, 976}};// slim video 
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

static void set_dummy()
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	/*
	write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
	*/
  
}	/*	set_dummy  */


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
	#if 1
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
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	}

	// Update Shutter
	write_cmos_sensor(0x0202, (shutter>>8) & 0xFF);
	write_cmos_sensor(0x0203, (shutter & 0xFF));
	
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
	#endif

	//LOG_INF("frame_length = %d ", frame_length);
	
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
	#if 1
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
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
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	}

	// Update Shutter
	write_cmos_sensor(0x0202, (shutter>>8) & 0xFF);
	write_cmos_sensor(0x0203, (shutter & 0xFF));	
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
	#endif

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint8 i = 0;
    kal_uint8 gain_tmp1=0,Reg_Cgain=0,Reg_Dgain=0;
    kal_uint16 gain_tmp0 = 0;
    //iGain_temp=(float)iGain;
	#if 1
    gain_tmp0=(gain*100) / BASEGAIN;
    //LOG_INF("[HM5040MIPI]mycat BASEGAIN=%d\n",BASEGAIN);
    
    //gain_tmp1=(iGain % BASEGAIN);
    do {
        if (gain_tmp0 < HM5040_gain_table[i].analoggain)
            break;
        i++;
    } while (HM5040_gain_table[i].analoggain != 0);

    if (i == 0)
    {

        Reg_Cgain = 0x00;
       // LOG_INF("[brad_gain] iGain=%d gain_tmp0=%d  analoggain=%d  Reg_Cgain = 0x%x    \n",iGain,gain_tmp0,HM5040_gain_table[i].analoggain,Reg_Cgain);

    }
    else
    {
        Reg_Cgain = HM5040_gain_table[i-1].gainvalue & 0xFF;
      //  LOG_INF("[brad_gain] iGain=%d gain_tmp0=%d  analoggain=%d  Reg_Cgain = 0x%x    \n",iGain,gain_tmp0,HM5040_gain_table[i-1].analoggain,Reg_Cgain);

    }
Reg_Cgain = Reg_Cgain & 0xFF;
    //HM5040MIPI_write_cmos_sensor(0x0204,0x00);
//    HM5040MIPI_write_cmos_sensor(0x0205,Reg_Cgain);
//    HM5040MIPI_write_cmos_sensor(0x0104, 0x00); 
	//LOG_INF("[E], analog Reg_Cgain = 0x%x", Reg_Cgain);
#endif
 


#if 1
	//gain_tmp0 = (HM5040_gain_table[i].analoggain * 100) / gain_tmp0;
	gain_tmp0 = (gain_tmp0 * 100) /  HM5040_gain_table[i-1].analoggain;
	i = 0;
	do {
        if (gain_tmp0 < HM5040_D_gain_table[i].analoggain)
            break;
		LOG_INF("[1], HM5040_D_gain_table[i].analoggain = %d", HM5040_D_gain_table[i].analoggain);
        i++;
    } while (HM5040_D_gain_table[i].analoggain != 0);
	if (i == 0)
    {
        Reg_Dgain = 0x00;
    }
    else
    {
        Reg_Dgain = HM5040_D_gain_table[i-1].gainvalue & 0xFF;
    }


		write_cmos_sensor(0x020f,Reg_Dgain);
		write_cmos_sensor(0x0211,Reg_Dgain);
		write_cmos_sensor(0x0213,Reg_Dgain);
		write_cmos_sensor(0x0215,Reg_Dgain);
		write_cmos_sensor(0x0205,Reg_Cgain);

		LOG_INF("[brad_digitG], digital Reg_Dgain = 0x%x", Reg_Dgain);


#endif

//mdelay(30);

    write_cmos_sensor(0x0104, 0x00); 
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
#if 1
	write_cmos_sensor(0x0104, 0x01);//20140523
	gain2reg(gain);
	//HM5040MIPIGain2Reg(gain);
	write_cmos_sensor(0x0104, 0x00);//20140523
#endif

	
#if 0
	kal_uint16 reg_gain;
	if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 32 * BASEGAIN)
			gain = 32 * BASEGAIN;		 
	}

	//reg_gain = gain2reg(gain);
	reg_gain = gain*2;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x3508, (reg_gain>>8));
	write_cmos_sensor(0x3509, (reg_gain&0xFF));    

	return gain;
	#endif
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	#if 0
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
	#endif
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
			//write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			//write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
			break;
		case IMAGE_H_MIRROR:
			//write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			//write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
			break;
		case IMAGE_V_MIRROR:
			//write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
			//write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));		
			break;
		case IMAGE_HV_MIRROR:
			//write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
			//write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
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
	#if 0
	//HM5040 R1A key initial setting for 4lane
	write_cmos_sensor(0x103 , 0x01);
	write_cmos_sensor(0x100 , 0x00);
	write_cmos_sensor(0x302 , 0x1e);
	write_cmos_sensor(0x303 , 0x00);
	write_cmos_sensor(0x304 , 0x03);
	write_cmos_sensor(0x30e , 0x02);
	write_cmos_sensor(0x30f , 0x04);
	write_cmos_sensor(0x312 , 0x03);
	write_cmos_sensor(0x31e , 0x0c);
	write_cmos_sensor(0x3600, 0x00);
	write_cmos_sensor(0x3601, 0x00);
	write_cmos_sensor(0x3602, 0x00);
	write_cmos_sensor(0x3603, 0x00);
	write_cmos_sensor(0x3604, 0x22);
	write_cmos_sensor(0x3605, 0x30);
	write_cmos_sensor(0x3606, 0x00);
	write_cmos_sensor(0x3607, 0x20);
	write_cmos_sensor(0x3608, 0x11);
	write_cmos_sensor(0x3609, 0x28);
	write_cmos_sensor(0x360a, 0x00);
	write_cmos_sensor(0x360b, 0x06);
	write_cmos_sensor(0x360c, 0xdc);
	write_cmos_sensor(0x360d, 0x40);
	write_cmos_sensor(0x360e, 0x0c);
	write_cmos_sensor(0x360f, 0x20);
	write_cmos_sensor(0x3610, 0x07);
	write_cmos_sensor(0x3611, 0x20);
	write_cmos_sensor(0x3612, 0x88);
	write_cmos_sensor(0x3613, 0x80);
	write_cmos_sensor(0x3614, 0x58);
	write_cmos_sensor(0x3615, 0x00);
	write_cmos_sensor(0x3616, 0x4a);
	write_cmos_sensor(0x3617, 0x90);
	write_cmos_sensor(0x3618, 0x56);
	write_cmos_sensor(0x3619, 0x70);
	write_cmos_sensor(0x361a, 0x99);
	write_cmos_sensor(0x361b, 0x00);
	write_cmos_sensor(0x361c, 0x07);
	write_cmos_sensor(0x361d, 0x00);
	write_cmos_sensor(0x361e, 0x00);
	write_cmos_sensor(0x361f, 0x00);
	write_cmos_sensor(0x3638, 0xff);
	write_cmos_sensor(0x3633, 0x0c);
	write_cmos_sensor(0x3634, 0x0c);
	write_cmos_sensor(0x3635, 0x0c);
	write_cmos_sensor(0x3636, 0x0c);
	write_cmos_sensor(0x3645, 0x13);
	write_cmos_sensor(0x3646, 0x83);
	write_cmos_sensor(0x364a, 0x07);
	write_cmos_sensor(0x3015, 0x00);
	write_cmos_sensor(0x3018, 0x72);
	write_cmos_sensor(0x3020, 0x93);
	write_cmos_sensor(0x3022, 0x01);
	write_cmos_sensor(0x3031, 0x0a);
	write_cmos_sensor(0x3034, 0x00);
	write_cmos_sensor(0x3106, 0x01);
	write_cmos_sensor(0x3305, 0xf1);
	write_cmos_sensor(0x3308, 0x00);
	write_cmos_sensor(0x3309, 0x28);
	write_cmos_sensor(0x330a, 0x00);
	write_cmos_sensor(0x330b, 0x20);
	write_cmos_sensor(0x330c, 0x00);
	write_cmos_sensor(0x330d, 0x00);
	write_cmos_sensor(0x330e, 0x00);
	write_cmos_sensor(0x330f, 0x40);
	write_cmos_sensor(0x3307, 0x04);
	write_cmos_sensor(0x3500, 0x00);
	write_cmos_sensor(0x3501, 0x4b);
	write_cmos_sensor(0x3502, 0xc0);
	write_cmos_sensor(0x3503, 0x00);
	write_cmos_sensor(0x3505, 0x80);
	write_cmos_sensor(0x3508, 0x01);
	write_cmos_sensor(0x3509, 0x00);
	write_cmos_sensor(0x350c, 0x00);
	write_cmos_sensor(0x350d, 0x80);
	write_cmos_sensor(0x3510, 0x00);
	write_cmos_sensor(0x3511, 0x02);
	write_cmos_sensor(0x3512, 0x00);
	write_cmos_sensor(0x3700, 0x18);
	write_cmos_sensor(0x3701, 0x0c);
	write_cmos_sensor(0x3702, 0x28);
	write_cmos_sensor(0x3703, 0x19);
	write_cmos_sensor(0x3704, 0x14);
	write_cmos_sensor(0x3705, 0x00);
	write_cmos_sensor(0x3706, 0x6a);
	write_cmos_sensor(0x3707, 0x04);
	write_cmos_sensor(0x3708, 0x24);
	write_cmos_sensor(0x3709, 0x33);
	write_cmos_sensor(0x370a, 0x01);
	write_cmos_sensor(0x370b, 0x6a);
	write_cmos_sensor(0x370c, 0x04);
	write_cmos_sensor(0x3718, 0x12);
	write_cmos_sensor(0x3719, 0x31);
	write_cmos_sensor(0x3712, 0x42);
	write_cmos_sensor(0x3714, 0x24);
	write_cmos_sensor(0x371e, 0x19);
	write_cmos_sensor(0x371f, 0x40);
	write_cmos_sensor(0x3720, 0x05);
	write_cmos_sensor(0x3721, 0x05);
	write_cmos_sensor(0x3724, 0x06);
	write_cmos_sensor(0x3725, 0x01);
	write_cmos_sensor(0x3726, 0x06);
	write_cmos_sensor(0x3728, 0x05);
	write_cmos_sensor(0x3729, 0x02);
	write_cmos_sensor(0x372a, 0x03);
	write_cmos_sensor(0x372b, 0x53);
	write_cmos_sensor(0x372c, 0xa3);
	write_cmos_sensor(0x372d, 0x53);
	write_cmos_sensor(0x372e, 0x06);
	write_cmos_sensor(0x372f, 0x10);
	write_cmos_sensor(0x3730, 0x01);
	write_cmos_sensor(0x3731, 0x06);
	write_cmos_sensor(0x3732, 0x14);
	write_cmos_sensor(0x3733, 0x10);
	write_cmos_sensor(0x3734, 0x40);
	write_cmos_sensor(0x3736, 0x20);
	write_cmos_sensor(0x373a, 0x05);
	write_cmos_sensor(0x373b, 0x06);
	write_cmos_sensor(0x373c, 0x0a);
	write_cmos_sensor(0x373e, 0x03);
	write_cmos_sensor(0x3755, 0x10);
	write_cmos_sensor(0x3758, 0x00);
	write_cmos_sensor(0x3759, 0x4c);
	write_cmos_sensor(0x375a, 0x06);
	write_cmos_sensor(0x375b, 0x13);
	write_cmos_sensor(0x375c, 0x20);
	write_cmos_sensor(0x375d, 0x02);
	write_cmos_sensor(0x375e, 0x00);
	write_cmos_sensor(0x375f, 0x14);
	write_cmos_sensor(0x3768, 0x22);
	write_cmos_sensor(0x3769, 0x44);
	write_cmos_sensor(0x376a, 0x44);
	write_cmos_sensor(0x3761, 0x00);
	write_cmos_sensor(0x3762, 0x00);
	write_cmos_sensor(0x3763, 0x00);
	write_cmos_sensor(0x3766, 0xff);
	write_cmos_sensor(0x376b, 0x00);
	write_cmos_sensor(0x3772, 0x23);
	write_cmos_sensor(0x3773, 0x02);
	write_cmos_sensor(0x3774, 0x16);
	write_cmos_sensor(0x3775, 0x12);
	write_cmos_sensor(0x3776, 0x04);
	write_cmos_sensor(0x3777, 0x00);
	write_cmos_sensor(0x3778, 0x17);
	write_cmos_sensor(0x37a0, 0x44);
	write_cmos_sensor(0x37a1, 0x3d);
	write_cmos_sensor(0x37a2, 0x3d);
	write_cmos_sensor(0x37a3, 0x00);
	write_cmos_sensor(0x37a4, 0x00);
	write_cmos_sensor(0x37a5, 0x00);
	write_cmos_sensor(0x37a6, 0x00);
	write_cmos_sensor(0x37a7, 0x44);
	write_cmos_sensor(0x37a8, 0x4c);
	write_cmos_sensor(0x37a9, 0x4c);
	write_cmos_sensor(0x3760, 0x00);
	write_cmos_sensor(0x376f, 0x01);
	write_cmos_sensor(0x37aa, 0x44);
	write_cmos_sensor(0x37ab, 0x2e);
	write_cmos_sensor(0x37ac, 0x2e);
	write_cmos_sensor(0x37ad, 0x33);
	write_cmos_sensor(0x37ae, 0x0d);
	write_cmos_sensor(0x37af, 0x0d);
	write_cmos_sensor(0x37b0, 0x00);
	write_cmos_sensor(0x37b1, 0x00);
	write_cmos_sensor(0x37b2, 0x00);
	write_cmos_sensor(0x37b3, 0x42);
	write_cmos_sensor(0x37b4, 0x42);
	write_cmos_sensor(0x37b5, 0x33);
	write_cmos_sensor(0x37b6, 0x00);
	write_cmos_sensor(0x37b7, 0x00);
	write_cmos_sensor(0x37b8, 0x00);
	write_cmos_sensor(0x37b9, 0xff);
	write_cmos_sensor(0x3800, 0x00);
	write_cmos_sensor(0x3801, 0x0c);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x0c);
	write_cmos_sensor(0x3804, 0x0c);
	write_cmos_sensor(0x3805, 0xd3);
	write_cmos_sensor(0x3806, 0x09);
	write_cmos_sensor(0x3807, 0xa3);
	write_cmos_sensor(0x3808, 0x06);
	write_cmos_sensor(0x3809, 0x60);
	write_cmos_sensor(0x380a, 0x04);
	write_cmos_sensor(0x380b, 0xc8);
	write_cmos_sensor(0x380c, 0x07);
	write_cmos_sensor(0x380d, 0x88);
	write_cmos_sensor(0x380e, 0x04);
	write_cmos_sensor(0x380f, 0xdc);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0x04);
	write_cmos_sensor(0x3813, 0x02);
	write_cmos_sensor(0x3814, 0x03);
	write_cmos_sensor(0x3815, 0x01);
	write_cmos_sensor(0x3820, 0x00);
	write_cmos_sensor(0x3821, 0x67);
	write_cmos_sensor(0x382a, 0x03);
	write_cmos_sensor(0x382b, 0x01);
	write_cmos_sensor(0x3830, 0x08);
	write_cmos_sensor(0x3836, 0x02);
	write_cmos_sensor(0x3837, 0x18);
	write_cmos_sensor(0x3841, 0xff);
	write_cmos_sensor(0x3846, 0x48);
	write_cmos_sensor(0x3d85, 0x14);
	write_cmos_sensor(0x3f08, 0x08);
	write_cmos_sensor(0x3f0a, 0x00);
	write_cmos_sensor(0x4000, 0xf1);
	write_cmos_sensor(0x4001, 0x10);
	write_cmos_sensor(0x4005, 0x10);
	write_cmos_sensor(0x4002, 0x27);
	write_cmos_sensor(0x4009, 0x81);
	write_cmos_sensor(0x400b, 0x0c);
	write_cmos_sensor(0x401b, 0x00);
	write_cmos_sensor(0x401d, 0x00);
	write_cmos_sensor(0x4020, 0x00);
	write_cmos_sensor(0x4021, 0x04);
	write_cmos_sensor(0x4022, 0x04);
	write_cmos_sensor(0x4023, 0xb9);
	write_cmos_sensor(0x4024, 0x05);
	write_cmos_sensor(0x4025, 0x2a);
	write_cmos_sensor(0x4026, 0x05);
	write_cmos_sensor(0x4027, 0x2b);
	write_cmos_sensor(0x4028, 0x00);
	write_cmos_sensor(0x4029, 0x02);
	write_cmos_sensor(0x402a, 0x04);
	write_cmos_sensor(0x402b, 0x04);
	write_cmos_sensor(0x402c, 0x02);
	write_cmos_sensor(0x402d, 0x02);
	write_cmos_sensor(0x402e, 0x08);
	write_cmos_sensor(0x402f, 0x02);
	write_cmos_sensor(0x401f, 0x00);
	write_cmos_sensor(0x4034, 0x3f);
	write_cmos_sensor(0x403d, 0x04);
	write_cmos_sensor(0x4300, 0xff);
	write_cmos_sensor(0x4301, 0x00);
	write_cmos_sensor(0x4302, 0x0f);
	write_cmos_sensor(0x4316, 0x00);
	write_cmos_sensor(0x4500, 0x58);
	write_cmos_sensor(0x4503, 0x18);
	write_cmos_sensor(0x4600, 0x00);
	write_cmos_sensor(0x4601, 0xcb);
	write_cmos_sensor(0x481f, 0x32);
	write_cmos_sensor(0x4837, 0x16);
	write_cmos_sensor(0x4850, 0x10);
	write_cmos_sensor(0x4851, 0x32);
	write_cmos_sensor(0x4b00, 0x2a);
	write_cmos_sensor(0x4b0d, 0x00);
	write_cmos_sensor(0x4d00, 0x04);
	write_cmos_sensor(0x4d01, 0x18);
	write_cmos_sensor(0x4d02, 0xc3);
	write_cmos_sensor(0x4d03, 0xff);
	write_cmos_sensor(0x4d04, 0xff);
	write_cmos_sensor(0x4d05, 0xff);
	write_cmos_sensor(0x5000, 0x7e);
	write_cmos_sensor(0x5001, 0x01);
	write_cmos_sensor(0x5002, 0x08);
	write_cmos_sensor(0x5003, 0x20);
	write_cmos_sensor(0x5046, 0x12);
	write_cmos_sensor(0x5780, 0xfc);
	write_cmos_sensor(0x5784, 0x0c);
	write_cmos_sensor(0x5787, 0x40);
	write_cmos_sensor(0x5788, 0x08);
	write_cmos_sensor(0x578a, 0x02);
	write_cmos_sensor(0x578b, 0x01);
	write_cmos_sensor(0x578c, 0x01);
	write_cmos_sensor(0x578e, 0x02);
	write_cmos_sensor(0x578f, 0x01);
	write_cmos_sensor(0x5790, 0x01);
	write_cmos_sensor(0x5901, 0x00);
	write_cmos_sensor(0x5b00, 0x02);
	write_cmos_sensor(0x5b01, 0x10);
	write_cmos_sensor(0x5b02, 0x03);
	write_cmos_sensor(0x5b03, 0xcf);
	write_cmos_sensor(0x5b05, 0x6c);
	write_cmos_sensor(0x5e00, 0x00);
	write_cmos_sensor(0x5e01, 0x41);
	write_cmos_sensor(0x382d, 0x7f);
	write_cmos_sensor(0x4825, 0x3a);
	write_cmos_sensor(0x4826, 0x40);
	write_cmos_sensor(0x4808, 0x25);
	write_cmos_sensor(0x0100, 0x01);
	#endif

}
static void sensor_init(void)
{
	LOG_INF("E\n");
	//HM5040 R2A setting
	//3.1 Initialization (Global Setting)
	//XVCLK=24Mhz, SCLK=72Mhz, MIPI 720Mbps, DACCLK=180Mhz
	write_cmos_sensor(0x0103,0x01); //2 1
    mdelay(50);   //20140523
    write_cmos_sensor(0x0100,0x00); //2 1
    write_cmos_sensor(0x3002,0x32); //2 1
    write_cmos_sensor(0x3016,0x46); //2 1
    write_cmos_sensor(0x3017,0x29); //2 1
    write_cmos_sensor(0x3003,0x03); //2 1
    write_cmos_sensor(0x3045,0x03); //2 1
    write_cmos_sensor(0xFBD7,0x00); //2 1  // FF_SUB_G1 HI
    write_cmos_sensor(0xFBD8,0x00); //2 1  // FF_SUB_G1 LO
    write_cmos_sensor(0xFBD9,0x00); //2 1  // FF_DIV_G1 HI
    write_cmos_sensor(0xFBDA,0x00); //2 1  // FF_DIV_G1 LO
    write_cmos_sensor(0xFBDB,0x00); //2 1  // FF_SUB_G2 HI
    write_cmos_sensor(0xFBDC,0x00); //2 1  // FF_SUB_G2 LO
    write_cmos_sensor(0xFBDD,0x00); //2 1  // FF_DIV_G2 HI
    write_cmos_sensor(0xFBDE,0x00); //2 1  // FF_DIV_G2 LO
    write_cmos_sensor(0xFBDF,0x00); //2 1  // FF_SUB_G4 HI
    write_cmos_sensor(0xFBE0,0x00); //2 1  // FF_SUB_G4 LO
    write_cmos_sensor(0xFBE1,0x00); //2 1  // FF_DIV_G4 HI
    write_cmos_sensor(0xFBE2,0x00); //2 1  // FF_DIV_G4 LO
    write_cmos_sensor(0xFBE3,0x00); //2 1  // FF_SUB_G8 HI
    write_cmos_sensor(0xFBE4,0x00); //2 1  // FF_SUB_G8 LO
    write_cmos_sensor(0xFBE5,0x00); //2 1  // FF_DIV_G8 HI
    write_cmos_sensor(0xFBE6,0x00); //2 1  // FF_DIV_G8 LO
    write_cmos_sensor(0xFBE7,0x00); //2 1  // FF_SUB_G16 HI
    write_cmos_sensor(0xFBE8,0x00); //2 1  // FF_SUB_G16 LO
    write_cmos_sensor(0xFBE9,0x00); //2 1  // FF_DIV_G16 HI
    write_cmos_sensor(0xFBEA,0x00); //2 1  // FF_DIV_G16 LO
    //AB2 Dark Shading Setting
    write_cmos_sensor(0xFBEB,0x4E); //2 1  // AB_SUB_G1 HI
    write_cmos_sensor(0xFBEC,0xC5); //2 1  // AB_SUB_G1 LO
    write_cmos_sensor(0xFBED,0x43); //2 1  // AB_DIV_G1 HI
    write_cmos_sensor(0xFBEE,0xDA); //2 1  // AB_DIV_G1 LO
    write_cmos_sensor(0xFBEF,0x4E); //2 1  // AB_SUB_G2 HI
    write_cmos_sensor(0xFBF0,0xB0); //2 1  // AB_SUB_G2 LO
    write_cmos_sensor(0xFBF1,0x43); //2 1  // AB_DIV_G2 HI
    write_cmos_sensor(0xFBF2,0x8E); //2 1  // AB_DIV_G2 LO
    write_cmos_sensor(0xFBF3,0x4E); //2 1  // AB_SUB_G4 HI
    write_cmos_sensor(0xFBF4,0xAF); //2 1  // AB_SUB_G4 LO
    write_cmos_sensor(0xFBF5,0x43); //2 1  // AB_DIV_G4 HI
    write_cmos_sensor(0xFBF6,0x76); //2 1  // AB_DIV_G4 LO
    write_cmos_sensor(0xFBF7,0x4E); //2 1  // AB_SUB_G8 HI
    write_cmos_sensor(0xFBF8,0xB7); //2 1  // AB_SUB_G8 LO
    write_cmos_sensor(0xFBF9,0x43); //2 1  // AB_DIV_G8 HI
    write_cmos_sensor(0xFBFA,0x80); //2 1  // AB_DIV_G8 LO
    write_cmos_sensor(0xFBFB,0x4E); //2 1  // AB_SUB_G16 HI
    write_cmos_sensor(0xFBFC,0xC4); //2 1  // AB_SUB_G16 LO
    write_cmos_sensor(0xFBFD,0x43); //2 1  // AB_DIV_G16 HI
    write_cmos_sensor(0xFBFE,0x87); //2 1  // AB_DIV_G16 LO
    write_cmos_sensor(0xFB00,0x51); //2 1
    write_cmos_sensor(0xF800,0xc0); //2 1
    write_cmos_sensor(0xF801,0x24); //2 1
    write_cmos_sensor(0xF802,0x7c); //2 1
    write_cmos_sensor(0xF803,0xfb); //2 1
    write_cmos_sensor(0xF804,0x7d); //2 1
    write_cmos_sensor(0xF805,0xc7); //2 1
    write_cmos_sensor(0xF806,0x7b); //2 1
    write_cmos_sensor(0xF807,0x10); //2 1
    write_cmos_sensor(0xF808,0x7f); //2 1
    write_cmos_sensor(0xF809,0x72); //2 1
    write_cmos_sensor(0xF80A,0x7e); //2 1
    write_cmos_sensor(0xF80B,0x30); //2 1
    write_cmos_sensor(0xF80C,0x12); //2 1
    write_cmos_sensor(0xF80D,0x09); //2 1
    write_cmos_sensor(0xF80E,0x47); //2 1
    write_cmos_sensor(0xF80F,0xd0); //2 1
    write_cmos_sensor(0xF810,0x24); //2 1
    write_cmos_sensor(0xF811,0x90); //2 1
    write_cmos_sensor(0xF812,0x02); //2 1
    write_cmos_sensor(0xF813,0x05); //2 1
    write_cmos_sensor(0xF814,0xe0); //2 1
    write_cmos_sensor(0xF815,0xf5); //2 1
    write_cmos_sensor(0xF816,0x77); //2 1
    write_cmos_sensor(0xF817,0xe5); //2 1
    write_cmos_sensor(0xF818,0x77); //2 1
    write_cmos_sensor(0xF819,0xc3); //2 1
    write_cmos_sensor(0xF81A,0x94); //2 1
    write_cmos_sensor(0xF81B,0x80); //2 1
    write_cmos_sensor(0xF81C,0x50); //2 1
    write_cmos_sensor(0xF81D,0x08); //2 1
    write_cmos_sensor(0xF81E,0x75); //2 1
    write_cmos_sensor(0xF81F,0x7a); //2 1
    write_cmos_sensor(0xF820,0xfb); //2 1
    write_cmos_sensor(0xF821,0x75); //2 1
    write_cmos_sensor(0xF822,0x7b); //2 1
    write_cmos_sensor(0xF823,0xd7); //2 1
    write_cmos_sensor(0xF824,0x80); //2 1
    write_cmos_sensor(0xF825,0x33); //2 1
    write_cmos_sensor(0xF826,0xe5); //2 1
    write_cmos_sensor(0xF827,0x77); //2 1
    write_cmos_sensor(0xF828,0xc3); //2 1
    write_cmos_sensor(0xF829,0x94); //2 1
    write_cmos_sensor(0xF82A,0xc0); //2 1
    write_cmos_sensor(0xF82B,0x50); //2 1
    write_cmos_sensor(0xF82C,0x08); //2 1
    write_cmos_sensor(0xF82D,0x75); //2 1
    write_cmos_sensor(0xF82E,0x7a); //2 1
    write_cmos_sensor(0xF82F,0xfb); //2 1
    write_cmos_sensor(0xF830,0x75); //2 1
    write_cmos_sensor(0xF831,0x7b); //2 1
    write_cmos_sensor(0xF832,0xdb); //2 1
    write_cmos_sensor(0xF833,0x80); //2 1
    write_cmos_sensor(0xF834,0x24); //2 1
    write_cmos_sensor(0xF835,0xe5); //2 1
    write_cmos_sensor(0xF836,0x77); //2 1
    write_cmos_sensor(0xF837,0xc3); //2 1
    write_cmos_sensor(0xF838,0x94); //2 1
    write_cmos_sensor(0xF839,0xe0); //2 1
    write_cmos_sensor(0xF83A,0x50); //2 1
    write_cmos_sensor(0xF83B,0x08); //2 1
    write_cmos_sensor(0xF83C,0x75); //2 1
    write_cmos_sensor(0xF83D,0x7a); //2 1
    write_cmos_sensor(0xF83E,0xfb); //2 1
    write_cmos_sensor(0xF83F,0x75); //2 1
    write_cmos_sensor(0xF840,0x7b); //2 1
    write_cmos_sensor(0xF841,0xdf); //2 1
    write_cmos_sensor(0xF842,0x80); //2 1
    write_cmos_sensor(0xF843,0x15); //2 1
    write_cmos_sensor(0xF844,0xe5); //2 1
    write_cmos_sensor(0xF845,0x77); //2 1
    write_cmos_sensor(0xF846,0xc3); //2 1
    write_cmos_sensor(0xF847,0x94); //2 1
    write_cmos_sensor(0xF848,0xf0); //2 1
    write_cmos_sensor(0xF849,0x50); //2 1
    write_cmos_sensor(0xF84A,0x08); //2 1
    write_cmos_sensor(0xF84B,0x75); //2 1
    write_cmos_sensor(0xF84C,0x7a); //2 1
    write_cmos_sensor(0xF84D,0xfb); //2 1
    write_cmos_sensor(0xF84E,0x75); //2 1
    write_cmos_sensor(0xF84F,0x7b); //2 1
    write_cmos_sensor(0xF850,0xe3); //2 1
    write_cmos_sensor(0xF851,0x80); //2 1
    write_cmos_sensor(0xF852,0x06); //2 1
    write_cmos_sensor(0xF853,0x75); //2 1
    write_cmos_sensor(0xF854,0x7a); //2 1
    write_cmos_sensor(0xF855,0xfb); //2 1
    write_cmos_sensor(0xF856,0x75); //2 1
    write_cmos_sensor(0xF857,0x7b); //2 1
    write_cmos_sensor(0xF858,0xe7); //2 1
    write_cmos_sensor(0xF859,0xe5); //2 1
    write_cmos_sensor(0xF85A,0x55); //2 1
    write_cmos_sensor(0xF85B,0x7f); //2 1
    write_cmos_sensor(0xF85C,0x00); //2 1
    write_cmos_sensor(0xF85D,0xb4); //2 1
    write_cmos_sensor(0xF85E,0x22); //2 1
    write_cmos_sensor(0xF85F,0x02); //2 1
    write_cmos_sensor(0xF860,0x7f); //2 1
    write_cmos_sensor(0xF861,0x01); //2 1
    write_cmos_sensor(0xF862,0xe5); //2 1
    write_cmos_sensor(0xF863,0x53); //2 1
    write_cmos_sensor(0xF864,0x5f); //2 1
    write_cmos_sensor(0xF865,0x60); //2 1
    write_cmos_sensor(0xF866,0x05); //2 1
    write_cmos_sensor(0xF867,0x74); //2 1
    write_cmos_sensor(0xF868,0x14); //2 1
    write_cmos_sensor(0xF869,0x12); //2 1
    write_cmos_sensor(0xF86A,0xfa); //2 1
    write_cmos_sensor(0xF86B,0x4c); //2 1
    write_cmos_sensor(0xF86C,0x75); //2 1
    write_cmos_sensor(0xF86D,0x7c); //2 1
    write_cmos_sensor(0xF86E,0xfb); //2 1
    write_cmos_sensor(0xF86F,0x75); //2 1
    write_cmos_sensor(0xF870,0x7d); //2 1
    write_cmos_sensor(0xF871,0xc7); //2 1
    write_cmos_sensor(0xF872,0x75); //2 1
    write_cmos_sensor(0xF873,0x7e); //2 1
    write_cmos_sensor(0xF874,0x30); //2 1
    write_cmos_sensor(0xF875,0x75); //2 1
    write_cmos_sensor(0xF876,0x7f); //2 1
    write_cmos_sensor(0xF877,0x62); //2 1
    write_cmos_sensor(0xF878,0xe4); //2 1
    write_cmos_sensor(0xF879,0xf5); //2 1
    write_cmos_sensor(0xF87A,0x77); //2 1
    write_cmos_sensor(0xF87B,0xe5); //2 1
    write_cmos_sensor(0xF87C,0x77); //2 1
    write_cmos_sensor(0xF87D,0xc3); //2 1
    write_cmos_sensor(0xF87E,0x94); //2 1
    write_cmos_sensor(0xF87F,0x08); //2 1
    write_cmos_sensor(0xF880,0x40); //2 1
    write_cmos_sensor(0xF881,0x03); //2 1
    write_cmos_sensor(0xF882,0x02); //2 1
    write_cmos_sensor(0xF883,0xf9); //2 1
    write_cmos_sensor(0xF884,0x0e); //2 1
    write_cmos_sensor(0xF885,0x85); //2 1
    write_cmos_sensor(0xF886,0x7d); //2 1
    write_cmos_sensor(0xF887,0x82); //2 1
    write_cmos_sensor(0xF888,0x85); //2 1
    write_cmos_sensor(0xF889,0x7c); //2 1
    write_cmos_sensor(0xF88A,0x83); //2 1
    write_cmos_sensor(0xF88B,0xe0); //2 1
    write_cmos_sensor(0xF88C,0xfe); //2 1
    write_cmos_sensor(0xF88D,0xa3); //2 1
    write_cmos_sensor(0xF88E,0xe0); //2 1
    write_cmos_sensor(0xF88F,0xff); //2 1
    write_cmos_sensor(0xF890,0x12); //2 1
    write_cmos_sensor(0xF891,0x21); //2 1
    write_cmos_sensor(0xF892,0x22); //2 1
    write_cmos_sensor(0xF893,0x8e); //2 1
    write_cmos_sensor(0xF894,0x78); //2 1
    write_cmos_sensor(0xF895,0x8f); //2 1
    write_cmos_sensor(0xF896,0x79); //2 1
    write_cmos_sensor(0xF897,0x12); //2 1
    write_cmos_sensor(0xF898,0xfa); //2 1
    write_cmos_sensor(0xF899,0x40); //2 1
    write_cmos_sensor(0xF89A,0x12); //2 1
    write_cmos_sensor(0xF89B,0x22); //2 1
    write_cmos_sensor(0xF89C,0x93); //2 1
    write_cmos_sensor(0xF89D,0x50); //2 1
    write_cmos_sensor(0xF89E,0x07); //2 1
    write_cmos_sensor(0xF89F,0xe4); //2 1
    write_cmos_sensor(0xF8A0,0xf5); //2 1
    write_cmos_sensor(0xF8A1,0x78); //2 1
    write_cmos_sensor(0xF8A2,0xf5); //2 1
    write_cmos_sensor(0xF8A3,0x79); //2 1
    write_cmos_sensor(0xF8A4,0x80); //2 1
    write_cmos_sensor(0xF8A5,0x33); //2 1
    write_cmos_sensor(0xF8A6,0x12); //2 1
    write_cmos_sensor(0xF8A7,0xfa); //2 1
    write_cmos_sensor(0xF8A8,0x40); //2 1
    write_cmos_sensor(0xF8A9,0x7b); //2 1
    write_cmos_sensor(0xF8AA,0x01); //2 1
    write_cmos_sensor(0xF8AB,0xaf); //2 1
    write_cmos_sensor(0xF8AC,0x79); //2 1
    write_cmos_sensor(0xF8AD,0xae); //2 1
    write_cmos_sensor(0xF8AE,0x78); //2 1
    write_cmos_sensor(0xF8AF,0x12); //2 1
    write_cmos_sensor(0xF8B0,0x22); //2 1
    write_cmos_sensor(0xF8B1,0x4f); //2 1
    write_cmos_sensor(0xF8B2,0x74); //2 1
    write_cmos_sensor(0xF8B3,0x02); //2 1
    write_cmos_sensor(0xF8B4,0x12); //2 1
    write_cmos_sensor(0xF8B5,0xfa); //2 1
    write_cmos_sensor(0xF8B6,0x4c); //2 1
    write_cmos_sensor(0xF8B7,0x85); //2 1
    write_cmos_sensor(0xF8B8,0x7b); //2 1
    write_cmos_sensor(0xF8B9,0x82); //2 1
    write_cmos_sensor(0xF8BA,0xf5); //2 1
    write_cmos_sensor(0xF8BB,0x83); //2 1
    write_cmos_sensor(0xF8BC,0xe0); //2 1
    write_cmos_sensor(0xF8BD,0xfe); //2 1
    write_cmos_sensor(0xF8BE,0xa3); //2 1
    write_cmos_sensor(0xF8BF,0xe0); //2 1
    write_cmos_sensor(0xF8C0,0xff); //2 1
    write_cmos_sensor(0xF8C1,0x7d); //2 1
    write_cmos_sensor(0xF8C2,0x03); //2 1
    write_cmos_sensor(0xF8C3,0x12); //2 1
    write_cmos_sensor(0xF8C4,0x17); //2 1
    write_cmos_sensor(0xF8C5,0xd8); //2 1
    write_cmos_sensor(0xF8C6,0x12); //2 1
    write_cmos_sensor(0xF8C7,0x1b); //2 1
    write_cmos_sensor(0xF8C8,0x9b); //2 1
    write_cmos_sensor(0xF8C9,0x8e); //2 1
    write_cmos_sensor(0xF8CA,0x78); //2 1
    write_cmos_sensor(0xF8CB,0x8f); //2 1
    write_cmos_sensor(0xF8CC,0x79); //2 1
    write_cmos_sensor(0xF8CD,0x74); //2 1
    write_cmos_sensor(0xF8CE,0xfe); //2 1
    write_cmos_sensor(0xF8CF,0x25); //2 1
    write_cmos_sensor(0xF8D0,0x7b); //2 1
    write_cmos_sensor(0xF8D1,0xf5); //2 1
    write_cmos_sensor(0xF8D2,0x7b); //2 1
    write_cmos_sensor(0xF8D3,0x74); //2 1
    write_cmos_sensor(0xF8D4,0xff); //2 1
    write_cmos_sensor(0xF8D5,0x35); //2 1
    write_cmos_sensor(0xF8D6,0x7a); //2 1
    write_cmos_sensor(0xF8D7,0xf5); //2 1
    write_cmos_sensor(0xF8D8,0x7a); //2 1
    write_cmos_sensor(0xF8D9,0x78); //2 1
    write_cmos_sensor(0xF8DA,0x24); //2 1
    write_cmos_sensor(0xF8DB,0xe6); //2 1
    write_cmos_sensor(0xF8DC,0xff); //2 1
    write_cmos_sensor(0xF8DD,0xc3); //2 1
    write_cmos_sensor(0xF8DE,0x74); //2 1
    write_cmos_sensor(0xF8DF,0x20); //2 1
    write_cmos_sensor(0xF8E0,0x9f); //2 1
    write_cmos_sensor(0xF8E1,0x7e); //2 1
    write_cmos_sensor(0xF8E2,0x00); //2 1
    write_cmos_sensor(0xF8E3,0x25); //2 1
    write_cmos_sensor(0xF8E4,0x79); //2 1
    write_cmos_sensor(0xF8E5,0xff); //2 1
    write_cmos_sensor(0xF8E6,0xee); //2 1
    write_cmos_sensor(0xF8E7,0x35); //2 1
    write_cmos_sensor(0xF8E8,0x78); //2 1
    write_cmos_sensor(0xF8E9,0x85); //2 1
    write_cmos_sensor(0xF8EA,0x7f); //2 1
    write_cmos_sensor(0xF8EB,0x82); //2 1
    write_cmos_sensor(0xF8EC,0x85); //2 1
    write_cmos_sensor(0xF8ED,0x7e); //2 1
    write_cmos_sensor(0xF8EE,0x83); //2 1
    write_cmos_sensor(0xF8EF,0xf0); //2 1
    write_cmos_sensor(0xF8F0,0xa3); //2 1
    write_cmos_sensor(0xF8F1,0xef); //2 1
    write_cmos_sensor(0xF8F2,0xf0); //2 1
    write_cmos_sensor(0xF8F3,0x05); //2 1
    write_cmos_sensor(0xF8F4,0x77); //2 1
    write_cmos_sensor(0xF8F5,0x74); //2 1
    write_cmos_sensor(0xF8F6,0x02); //2 1
    write_cmos_sensor(0xF8F7,0x25); //2 1
    write_cmos_sensor(0xF8F8,0x7d); //2 1
    write_cmos_sensor(0xF8F9,0xf5); //2 1
    write_cmos_sensor(0xF8FA,0x7d); //2 1
    write_cmos_sensor(0xF8FB,0xe4); //2 1
    write_cmos_sensor(0xF8FC,0x35); //2 1
    write_cmos_sensor(0xF8FD,0x7c); //2 1
    write_cmos_sensor(0xF8FE,0xf5); //2 1
    write_cmos_sensor(0xF8FF,0x7c); //2 1
    write_cmos_sensor(0xF900,0x74); //2 1
    write_cmos_sensor(0xF901,0x02); //2 1
    write_cmos_sensor(0xF902,0x25); //2 1
    write_cmos_sensor(0xF903,0x7f); //2 1
    write_cmos_sensor(0xF904,0xf5); //2 1
    write_cmos_sensor(0xF905,0x7f); //2 1
    write_cmos_sensor(0xF906,0xe4); //2 1
    write_cmos_sensor(0xF907,0x35); //2 1
    write_cmos_sensor(0xF908,0x7e); //2 1
    write_cmos_sensor(0xF909,0xf5); //2 1
    write_cmos_sensor(0xF90A,0x7e); //2 1
    write_cmos_sensor(0xF90B,0x02); //2 1
    write_cmos_sensor(0xF90C,0xf8); //2 1
    write_cmos_sensor(0xF90D,0x7b); //2 1
    write_cmos_sensor(0xF90E,0x22); //2 1
    write_cmos_sensor(0xF90F,0x90); //2 1
    write_cmos_sensor(0xF910,0x30); //2 1
    write_cmos_sensor(0xF911,0x47); //2 1
    write_cmos_sensor(0xF912,0x74); //2 1
    write_cmos_sensor(0xF913,0x98); //2 1
    write_cmos_sensor(0xF914,0xf0); //2 1
    write_cmos_sensor(0xF915,0x90); //2 1
    write_cmos_sensor(0xF916,0x30); //2 1
    write_cmos_sensor(0xF917,0x36); //2 1
    write_cmos_sensor(0xF918,0x74); //2 1
    write_cmos_sensor(0xF919,0x1e); //2 1
    write_cmos_sensor(0xF91A,0xf0); //2 1
    write_cmos_sensor(0xF91B,0x90); //2 1
    write_cmos_sensor(0xF91C,0x30); //2 1
    write_cmos_sensor(0xF91D,0x42); //2 1
    write_cmos_sensor(0xF91E,0x74); //2 1
    write_cmos_sensor(0xF91F,0x24); //2 1
    write_cmos_sensor(0xF920,0xf0); //2 1
    write_cmos_sensor(0xF921,0xe5); //2 1
    write_cmos_sensor(0xF922,0x53); //2 1
    write_cmos_sensor(0xF923,0x60); //2 1
    write_cmos_sensor(0xF924,0x42); //2 1
    write_cmos_sensor(0xF925,0x78); //2 1
    write_cmos_sensor(0xF926,0x2b); //2 1
    write_cmos_sensor(0xF927,0x76); //2 1
    write_cmos_sensor(0xF928,0x01); //2 1
    write_cmos_sensor(0xF929,0xe5); //2 1
    write_cmos_sensor(0xF92A,0x55); //2 1
    write_cmos_sensor(0xF92B,0xb4); //2 1
    write_cmos_sensor(0xF92C,0x22); //2 1
    write_cmos_sensor(0xF92D,0x17); //2 1
    write_cmos_sensor(0xF92E,0x90); //2 1
    write_cmos_sensor(0xF92F,0x30); //2 1
    write_cmos_sensor(0xF930,0x36); //2 1
    write_cmos_sensor(0xF931,0x74); //2 1
    write_cmos_sensor(0xF932,0x46); //2 1
    write_cmos_sensor(0xF933,0xf0); //2 1
    write_cmos_sensor(0xF934,0x78); //2 1
    write_cmos_sensor(0xF935,0x28); //2 1
    write_cmos_sensor(0xF936,0x76); //2 1
    write_cmos_sensor(0xF937,0x31); //2 1
    write_cmos_sensor(0xF938,0x90); //2 1
    write_cmos_sensor(0xF939,0x30); //2 1
    write_cmos_sensor(0xF93A,0x0e); //2 1
    write_cmos_sensor(0xF93B,0xe0); //2 1
    write_cmos_sensor(0xF93C,0xc3); //2 1
    write_cmos_sensor(0xF93D,0x13); //2 1
    write_cmos_sensor(0xF93E,0x30); //2 1
    write_cmos_sensor(0xF93F,0xe0); //2 1
    write_cmos_sensor(0xF940,0x04); //2 1
    write_cmos_sensor(0xF941,0x78); //2 1
    write_cmos_sensor(0xF942,0x26); //2 1
    write_cmos_sensor(0xF943,0x76); //2 1
    write_cmos_sensor(0xF944,0x40); //2 1
    write_cmos_sensor(0xF945,0xe5); //2 1
    write_cmos_sensor(0xF946,0x55); //2 1
    write_cmos_sensor(0xF947,0xb4); //2 1
    write_cmos_sensor(0xF948,0x44); //2 1
    write_cmos_sensor(0xF949,0x21); //2 1
    write_cmos_sensor(0xF94A,0x90); //2 1
    write_cmos_sensor(0xF94B,0x30); //2 1
    write_cmos_sensor(0xF94C,0x47); //2 1
    write_cmos_sensor(0xF94D,0x74); //2 1
    write_cmos_sensor(0xF94E,0x9a); //2 1
    write_cmos_sensor(0xF94F,0xf0); //2 1
    write_cmos_sensor(0xF950,0x90); //2 1
    write_cmos_sensor(0xF951,0x30); //2 1
    write_cmos_sensor(0xF952,0x42); //2 1
    write_cmos_sensor(0xF953,0x74); //2 1
    write_cmos_sensor(0xF954,0x64); //2 1
    write_cmos_sensor(0xF955,0xf0); //2 1
    write_cmos_sensor(0xF956,0x90); //2 1
    write_cmos_sensor(0xF957,0x30); //2 1
    write_cmos_sensor(0xF958,0x0e); //2 1
    write_cmos_sensor(0xF959,0xe0); //2 1
    write_cmos_sensor(0xF95A,0x13); //2 1
    write_cmos_sensor(0xF95B,0x13); //2 1
    write_cmos_sensor(0xF95C,0x54); //2 1
    write_cmos_sensor(0xF95D,0x3f); //2 1
    write_cmos_sensor(0xF95E,0x30); //2 1
    write_cmos_sensor(0xF95F,0xe0); //2 1
    write_cmos_sensor(0xF960,0x0a); //2 1
    write_cmos_sensor(0xF961,0x78); //2 1
    write_cmos_sensor(0xF962,0x24); //2 1
    write_cmos_sensor(0xF963,0xe4); //2 1
    write_cmos_sensor(0xF964,0xf6); //2 1
    write_cmos_sensor(0xF965,0x80); //2 1
    write_cmos_sensor(0xF966,0x04); //2 1
    write_cmos_sensor(0xF967,0x78); //2 1
    write_cmos_sensor(0xF968,0x2b); //2 1
    write_cmos_sensor(0xF969,0xe4); //2 1
    write_cmos_sensor(0xF96A,0xf6); //2 1
    write_cmos_sensor(0xF96B,0x90); //2 1
    write_cmos_sensor(0xF96C,0x30); //2 1
    write_cmos_sensor(0xF96D,0x88); //2 1
    write_cmos_sensor(0xF96E,0x02); //2 1
    write_cmos_sensor(0xF96F,0x1d); //2 1
    write_cmos_sensor(0xF970,0x4f); //2 1
    write_cmos_sensor(0xF971,0x22); //2 1
    write_cmos_sensor(0xF972,0x90); //2 1
    write_cmos_sensor(0xF973,0x0c); //2 1
    write_cmos_sensor(0xF974,0x1a); //2 1
    write_cmos_sensor(0xF975,0xe0); //2 1
    write_cmos_sensor(0xF976,0x30); //2 1
    write_cmos_sensor(0xF977,0xe2); //2 1
    write_cmos_sensor(0xF978,0x18); //2 1
    write_cmos_sensor(0xF979,0x90); //2 1
    write_cmos_sensor(0xF97A,0x33); //2 1
    write_cmos_sensor(0xF97B,0x68); //2 1
    write_cmos_sensor(0xF97C,0xe0); //2 1
    write_cmos_sensor(0xF97D,0x64); //2 1
    write_cmos_sensor(0xF97E,0x05); //2 1
    write_cmos_sensor(0xF97F,0x70); //2 1
    write_cmos_sensor(0xF980,0x2f); //2 1
    write_cmos_sensor(0xF981,0x90); //2 1
    write_cmos_sensor(0xF982,0x30); //2 1
    write_cmos_sensor(0xF983,0x38); //2 1
    write_cmos_sensor(0xF984,0xe0); //2 1
    write_cmos_sensor(0xF985,0x70); //2 1
    write_cmos_sensor(0xF986,0x02); //2 1
    write_cmos_sensor(0xF987,0xa3); //2 1
    write_cmos_sensor(0xF988,0xe0); //2 1
    write_cmos_sensor(0xF989,0xc3); //2 1
    write_cmos_sensor(0xF98A,0x70); //2 1
    write_cmos_sensor(0xF98B,0x01); //2 1
    write_cmos_sensor(0xF98C,0xd3); //2 1
    write_cmos_sensor(0xF98D,0x40); //2 1
    write_cmos_sensor(0xF98E,0x21); //2 1
    write_cmos_sensor(0xF98F,0x80); //2 1
    write_cmos_sensor(0xF990,0x1b); //2 1
    write_cmos_sensor(0xF991,0x90); //2 1
    write_cmos_sensor(0xF992,0x33); //2 1
    write_cmos_sensor(0xF993,0x68); //2 1
    write_cmos_sensor(0xF994,0xe0); //2 1
    write_cmos_sensor(0xF995,0xb4); //2 1
    write_cmos_sensor(0xF996,0x05); //2 1
    write_cmos_sensor(0xF997,0x18); //2 1
    write_cmos_sensor(0xF998,0xc3); //2 1
    write_cmos_sensor(0xF999,0x90); //2 1
    write_cmos_sensor(0xF99A,0x30); //2 1
    write_cmos_sensor(0xF99B,0x3b); //2 1
    write_cmos_sensor(0xF99C,0xe0); //2 1
    write_cmos_sensor(0xF99D,0x94); //2 1
    write_cmos_sensor(0xF99E,0x0d); //2 1
    write_cmos_sensor(0xF99F,0x90); //2 1
    write_cmos_sensor(0xF9A0,0x30); //2 1
    write_cmos_sensor(0xF9A1,0x3a); //2 1
    write_cmos_sensor(0xF9A2,0xe0); //2 1
    write_cmos_sensor(0xF9A3,0x94); //2 1
    write_cmos_sensor(0xF9A4,0x00); //2 1
    write_cmos_sensor(0xF9A5,0x50); //2 1
    write_cmos_sensor(0xF9A6,0x02); //2 1
    write_cmos_sensor(0xF9A7,0x80); //2 1
    write_cmos_sensor(0xF9A8,0x01); //2 1
    write_cmos_sensor(0xF9A9,0xc3); //2 1
    write_cmos_sensor(0xF9AA,0x40); //2 1
    write_cmos_sensor(0xF9AB,0x04); //2 1
    write_cmos_sensor(0xF9AC,0x75); //2 1
    write_cmos_sensor(0xF9AD,0x10); //2 1
    write_cmos_sensor(0xF9AE,0x01); //2 1
    write_cmos_sensor(0xF9AF,0x22); //2 1
    write_cmos_sensor(0xF9B0,0x02); //2 1
    write_cmos_sensor(0xF9B1,0x16); //2 1
    write_cmos_sensor(0xF9B2,0xe1); //2 1
    write_cmos_sensor(0xF9B3,0x22); //2 1
    write_cmos_sensor(0xF9B4,0x90); //2 1
    write_cmos_sensor(0xF9B5,0xff); //2 1
    write_cmos_sensor(0xF9B6,0x33); //2 1
    write_cmos_sensor(0xF9B7,0xe0); //2 1
    write_cmos_sensor(0xF9B8,0x90); //2 1
    write_cmos_sensor(0xF9B9,0xff); //2 1
    write_cmos_sensor(0xF9BA,0x34); //2 1
    write_cmos_sensor(0xF9BB,0xe0); //2 1
    write_cmos_sensor(0xF9BC,0x60); //2 1
    write_cmos_sensor(0xF9BD,0x0d); //2 1
    write_cmos_sensor(0xF9BE,0x7c); //2 1
    write_cmos_sensor(0xF9BF,0xfb); //2 1
    write_cmos_sensor(0xF9C0,0x7d); //2 1
    write_cmos_sensor(0xF9C1,0xd7); //2 1
    write_cmos_sensor(0xF9C2,0x7b); //2 1
    write_cmos_sensor(0xF9C3,0x28); //2 1
    write_cmos_sensor(0xF9C4,0x7f); //2 1
    write_cmos_sensor(0xF9C5,0x34); //2 1
    write_cmos_sensor(0xF9C6,0x7e); //2 1
    write_cmos_sensor(0xF9C7,0xff); //2 1
    write_cmos_sensor(0xF9C8,0x12); //2 1
    write_cmos_sensor(0xF9C9,0x09); //2 1
    write_cmos_sensor(0xF9CA,0x47); //2 1
    write_cmos_sensor(0xF9CB,0x7f); //2 1
    write_cmos_sensor(0xF9CC,0x20); //2 1
    write_cmos_sensor(0xF9CD,0x7e); //2 1
    write_cmos_sensor(0xF9CE,0x01); //2 1
    write_cmos_sensor(0xF9CF,0x7d); //2 1
    write_cmos_sensor(0xF9D0,0x00); //2 1
    write_cmos_sensor(0xF9D1,0x7c); //2 1
    write_cmos_sensor(0xF9D2,0x00); //2 1
    write_cmos_sensor(0xF9D3,0x12); //2 1
    write_cmos_sensor(0xF9D4,0x12); //2 1
    write_cmos_sensor(0xF9D5,0xa4); //2 1
    write_cmos_sensor(0xF9D6,0xe4); //2 1
    write_cmos_sensor(0xF9D7,0x90); //2 1
    write_cmos_sensor(0xF9D8,0x3e); //2 1
    write_cmos_sensor(0xF9D9,0x44); //2 1
    write_cmos_sensor(0xF9DA,0xf0); //2 1
    write_cmos_sensor(0xF9DB,0x02); //2 1
    write_cmos_sensor(0xF9DC,0x16); //2 1
    write_cmos_sensor(0xF9DD,0x7e); //2 1
    write_cmos_sensor(0xF9DE,0x22); //2 1
    write_cmos_sensor(0xF9DF,0xe5); //2 1
    write_cmos_sensor(0xF9E0,0x44); //2 1
    write_cmos_sensor(0xF9E1,0x60); //2 1
    write_cmos_sensor(0xF9E2,0x10); //2 1
    write_cmos_sensor(0xF9E3,0x90); //2 1
    write_cmos_sensor(0xF9E4,0xf6); //2 1
    write_cmos_sensor(0xF9E5,0x2c); //2 1
    write_cmos_sensor(0xF9E6,0x74); //2 1
    write_cmos_sensor(0xF9E7,0x04); //2 1
    write_cmos_sensor(0xF9E8,0xf0); //2 1
    write_cmos_sensor(0xF9E9,0x90); //2 1
    write_cmos_sensor(0xF9EA,0xf6); //2 1
    write_cmos_sensor(0xF9EB,0x34); //2 1
    write_cmos_sensor(0xF9EC,0xf0); //2 1
    write_cmos_sensor(0xF9ED,0x90); //2 1
    write_cmos_sensor(0xF9EE,0xf6); //2 1
    write_cmos_sensor(0xF9EF,0x3c); //2 1
    write_cmos_sensor(0xF9F0,0xf0); //2 1
    write_cmos_sensor(0xF9F1,0x80); //2 1
    write_cmos_sensor(0xF9F2,0x0e); //2 1
    write_cmos_sensor(0xF9F3,0x90); //2 1
    write_cmos_sensor(0xF9F4,0xf5); //2 1
    write_cmos_sensor(0xF9F5,0xc0); //2 1
    write_cmos_sensor(0xF9F6,0x74); //2 1
    write_cmos_sensor(0xF9F7,0x04); //2 1
    write_cmos_sensor(0xF9F8,0xf0); //2 1
    write_cmos_sensor(0xF9F9,0x90); //2 1
    write_cmos_sensor(0xF9FA,0xf5); //2 1
    write_cmos_sensor(0xF9FB,0xc8); //2 1
    write_cmos_sensor(0xF9FC,0xf0); //2 1
    write_cmos_sensor(0xF9FD,0x90); //2 1
    write_cmos_sensor(0xF9FE,0xf5); //2 1
    write_cmos_sensor(0xF9FF,0xd0); //2 1
    write_cmos_sensor(0xFA00,0xf0); //2 1
    write_cmos_sensor(0xFA01,0x90); //2 1
    write_cmos_sensor(0xFA02,0xfb); //2 1
    write_cmos_sensor(0xFA03,0x7f); //2 1
    write_cmos_sensor(0xFA04,0x02); //2 1
    write_cmos_sensor(0xFA05,0x19); //2 1
    write_cmos_sensor(0xFA06,0x0b); //2 1
    write_cmos_sensor(0xFA07,0x22); //2 1
    write_cmos_sensor(0xFA08,0x90); //2 1
    write_cmos_sensor(0xFA09,0x0c); //2 1
    write_cmos_sensor(0xFA0A,0x1a); //2 1
    write_cmos_sensor(0xFA0B,0xe0); //2 1
    write_cmos_sensor(0xFA0C,0x20); //2 1
    write_cmos_sensor(0xFA0D,0xe2); //2 1
    write_cmos_sensor(0xFA0E,0x15); //2 1
    write_cmos_sensor(0xFA0F,0xe4); //2 1
    write_cmos_sensor(0xFA10,0x90); //2 1
    write_cmos_sensor(0xFA11,0x30); //2 1
    write_cmos_sensor(0xFA12,0xf8); //2 1
    write_cmos_sensor(0xFA13,0xf0); //2 1
    write_cmos_sensor(0xFA14,0xa3); //2 1
    write_cmos_sensor(0xFA15,0xf0); //2 1
    write_cmos_sensor(0xFA16,0x90); //2 1
    write_cmos_sensor(0xFA17,0x30); //2 1
    write_cmos_sensor(0xFA18,0xf1); //2 1
    write_cmos_sensor(0xFA19,0xe0); //2 1
    write_cmos_sensor(0xFA1A,0x44); //2 1
    write_cmos_sensor(0xFA1B,0x08); //2 1
    write_cmos_sensor(0xFA1C,0xf0); //2 1
    write_cmos_sensor(0xFA1D,0x90); //2 1
    write_cmos_sensor(0xFA1E,0x30); //2 1
    write_cmos_sensor(0xFA1F,0xf0); //2 1
    write_cmos_sensor(0xFA20,0xe0); //2 1
    write_cmos_sensor(0xFA21,0x44); //2 1
    write_cmos_sensor(0xFA22,0x08); //2 1
    write_cmos_sensor(0xFA23,0xf0); //2 1
    write_cmos_sensor(0xFA24,0x02); //2 1
    write_cmos_sensor(0xFA25,0x03); //2 1
    write_cmos_sensor(0xFA26,0xde); //2 1
    write_cmos_sensor(0xFA27,0x22); //2 1
    write_cmos_sensor(0xFA28,0x90); //2 1
    write_cmos_sensor(0xFA29,0x0c); //2 1
    write_cmos_sensor(0xFA2A,0x1a); //2 1
    write_cmos_sensor(0xFA2B,0xe0); //2 1
    write_cmos_sensor(0xFA2C,0x30); //2 1
    write_cmos_sensor(0xFA2D,0xe2); //2 1
    write_cmos_sensor(0xFA2E,0x0d); //2 1
    write_cmos_sensor(0xFA2F,0xe0); //2 1
    write_cmos_sensor(0xFA30,0x20); //2 1
    write_cmos_sensor(0xFA31,0xe0); //2 1
    write_cmos_sensor(0xFA32,0x06); //2 1
    write_cmos_sensor(0xFA33,0x90); //2 1
    write_cmos_sensor(0xFA34,0xfb); //2 1
    write_cmos_sensor(0xFA35,0x85); //2 1
    write_cmos_sensor(0xFA36,0x74); //2 1
    write_cmos_sensor(0xFA37,0x00); //2 1
    write_cmos_sensor(0xFA38,0xa5); //2 1
    write_cmos_sensor(0xFA39,0x12); //2 1
    write_cmos_sensor(0xFA3A,0x16); //2 1
    write_cmos_sensor(0xFA3B,0xa0); //2 1
    write_cmos_sensor(0xFA3C,0x02); //2 1
    write_cmos_sensor(0xFA3D,0x18); //2 1
    write_cmos_sensor(0xFA3E,0xac); //2 1
    write_cmos_sensor(0xFA3F,0x22); //2 1
    write_cmos_sensor(0xFA40,0x85); //2 1
    write_cmos_sensor(0xFA41,0x7b); //2 1
    write_cmos_sensor(0xFA42,0x82); //2 1
    write_cmos_sensor(0xFA43,0x85); //2 1
    write_cmos_sensor(0xFA44,0x7a); //2 1
    write_cmos_sensor(0xFA45,0x83); //2 1
    write_cmos_sensor(0xFA46,0xe0); //2 1
    write_cmos_sensor(0xFA47,0xfc); //2 1
    write_cmos_sensor(0xFA48,0xa3); //2 1
    write_cmos_sensor(0xFA49,0xe0); //2 1
    write_cmos_sensor(0xFA4A,0xfd); //2 1
    write_cmos_sensor(0xFA4B,0x22); //2 1
    write_cmos_sensor(0xFA4C,0x25); //2 1
    write_cmos_sensor(0xFA4D,0x7b); //2 1
    write_cmos_sensor(0xFA4E,0xf5); //2 1
    write_cmos_sensor(0xFA4F,0x7b); //2 1
    write_cmos_sensor(0xFA50,0xe4); //2 1
    write_cmos_sensor(0xFA51,0x35); //2 1
    write_cmos_sensor(0xFA52,0x7a); //2 1
    write_cmos_sensor(0xFA53,0xf5); //2 1
    write_cmos_sensor(0xFA54,0x7a); //2 1
    write_cmos_sensor(0xFA55,0x22); //2 1
    write_cmos_sensor(0xFA56,0xc0); //2 1
    write_cmos_sensor(0xFA57,0xd0); //2 1
    write_cmos_sensor(0xFA58,0x90); //2 1
    write_cmos_sensor(0xFA59,0x35); //2 1
    write_cmos_sensor(0xFA5A,0xb5); //2 1
    write_cmos_sensor(0xFA5B,0xe0); //2 1
    write_cmos_sensor(0xFA5C,0x54); //2 1
    write_cmos_sensor(0xFA5D,0xfc); //2 1
    write_cmos_sensor(0xFA5E,0x44); //2 1
    write_cmos_sensor(0xFA5F,0x01); //2 1
    write_cmos_sensor(0xFA60,0xf0); //2 1
    write_cmos_sensor(0xFA61,0x12); //2 1
    write_cmos_sensor(0xFA62,0x1f); //2 1
    write_cmos_sensor(0xFA63,0x5f); //2 1
    write_cmos_sensor(0xFA64,0xd0); //2 1
    write_cmos_sensor(0xFA65,0xd0); //2 1
    write_cmos_sensor(0xFA66,0x02); //2 1
    write_cmos_sensor(0xFA67,0x0a); //2 1
    write_cmos_sensor(0xFA68,0x16); //2 1
    write_cmos_sensor(0xFA69,0x22); //2 1
    write_cmos_sensor(0xFA6A,0x90); //2 1
    write_cmos_sensor(0xFA6B,0x0c); //2 1
    write_cmos_sensor(0xFA6C,0x1a); //2 1
    write_cmos_sensor(0xFA6D,0xe0); //2 1
    write_cmos_sensor(0xFA6E,0x20); //2 1
    write_cmos_sensor(0xFA6F,0xe0); //2 1
    write_cmos_sensor(0xFA70,0x06); //2 1
    write_cmos_sensor(0xFA71,0x90); //2 1
    write_cmos_sensor(0xFA72,0xfb); //2 1
    write_cmos_sensor(0xFA73,0x85); //2 1
    write_cmos_sensor(0xFA74,0x74); //2 1
    write_cmos_sensor(0xFA75,0x00); //2 1
    write_cmos_sensor(0xFA76,0xa5); //2 1
    write_cmos_sensor(0xFA77,0xe5); //2 1
    write_cmos_sensor(0xFA78,0x10); //2 1
    write_cmos_sensor(0xFA79,0x02); //2 1
    write_cmos_sensor(0xFA7A,0x1e); //2 1
    write_cmos_sensor(0xFA7B,0x8f); //2 1
    write_cmos_sensor(0xFA7C,0x22); //2 1
    write_cmos_sensor(0xFA7D,0x90); //2 1
    write_cmos_sensor(0xFA7E,0xfb); //2 1
    write_cmos_sensor(0xFA7F,0x85); //2 1
    write_cmos_sensor(0xFA80,0x74); //2 1
    write_cmos_sensor(0xFA81,0x00); //2 1
    write_cmos_sensor(0xFA82,0xa5); //2 1
    write_cmos_sensor(0xFA83,0xe5); //2 1
    write_cmos_sensor(0xFA84,0x1a); //2 1
    write_cmos_sensor(0xFA85,0x60); //2 1
    write_cmos_sensor(0xFA86,0x03); //2 1
    write_cmos_sensor(0xFA87,0x02); //2 1
    write_cmos_sensor(0xFA88,0x17); //2 1
    write_cmos_sensor(0xFA89,0x47); //2 1
    write_cmos_sensor(0xFA8A,0x22); //2 1
    write_cmos_sensor(0xFA8B,0x90); //2 1
    write_cmos_sensor(0xFA8C,0xfb); //2 1
    write_cmos_sensor(0xFA8D,0x84); //2 1
    write_cmos_sensor(0xFA8E,0x02); //2 1
    write_cmos_sensor(0xFA8F,0x18); //2 1
    write_cmos_sensor(0xFA90,0xd9); //2 1
    write_cmos_sensor(0xFA91,0x22); //2 1
    write_cmos_sensor(0xFA92,0x02); //2 1
    write_cmos_sensor(0xFA93,0x1f); //2 1
    write_cmos_sensor(0xFA94,0xb1); //2 1
    write_cmos_sensor(0xFA95,0x22); //2 1
    write_cmos_sensor(0x35D8,0x01); //2 1
    write_cmos_sensor(0x35D9,0x0F); //2 1
    write_cmos_sensor(0x35DA,0x01); //2 1
    write_cmos_sensor(0x35DB,0x72); //2 1
    write_cmos_sensor(0x35DC,0x01); //2 1
    write_cmos_sensor(0x35DD,0xB4); //2 1
    write_cmos_sensor(0x35DE,0x01); //2 1
    write_cmos_sensor(0x35DF,0xDF); //2 1
    write_cmos_sensor(0x35E0,0x02); //2 1
    write_cmos_sensor(0x35E1,0x08); //2 1
    write_cmos_sensor(0x35E2,0x02); //2 1
    write_cmos_sensor(0x35E3,0x28); //2 1
    write_cmos_sensor(0x35E4,0x02); //2 1
    write_cmos_sensor(0x35E5,0x56); //2 1
    write_cmos_sensor(0x35E6,0x02); //2 1
    write_cmos_sensor(0x35E7,0x6A); //2 1
    write_cmos_sensor(0x35E8,0x02); //2 1
    write_cmos_sensor(0x35E9,0x7D); //2 1
    write_cmos_sensor(0x35EA,0x02); //2 1
    write_cmos_sensor(0x35EB,0x8B); //2 1
    write_cmos_sensor(0x35EC,0x02); //2 1
    write_cmos_sensor(0x35ED,0x92); //2 1
    write_cmos_sensor(0x35EF,0x22); //2 1
    write_cmos_sensor(0x35F1,0x23); //2 1
    write_cmos_sensor(0x35F3,0x22); //2 1
    write_cmos_sensor(0x35F6,0x19); //2 1
    write_cmos_sensor(0x35F7,0x55); //2 1
    write_cmos_sensor(0x35F8,0x1D); //2 1
    write_cmos_sensor(0x35F9,0x4C); //2 1
    write_cmos_sensor(0x35FA,0x16); //2 1
    write_cmos_sensor(0x35FB,0xC7); //2 1
    write_cmos_sensor(0x35FC,0x1A); //2 1
    write_cmos_sensor(0x35FD,0xA0); //2 1
    write_cmos_sensor(0x35FE,0x18); //2 1
    write_cmos_sensor(0x35FF,0xD6); //2 1
    write_cmos_sensor(0x3600,0x03); //2 1
    write_cmos_sensor(0x3601,0xD4); //2 1
    write_cmos_sensor(0x3602,0x18); //2 1
    write_cmos_sensor(0x3603,0x8A); //2 1
    write_cmos_sensor(0x3604,0x0A); //2 1
    write_cmos_sensor(0x3605,0x0D); //2 1
    write_cmos_sensor(0x3606,0x1E); //2 1
    write_cmos_sensor(0x3607,0x8D); //2 1
    write_cmos_sensor(0x3608,0x17); //2 1
    write_cmos_sensor(0x3609,0x43); //2 1
    write_cmos_sensor(0x360A,0x19); //2 1
    write_cmos_sensor(0x360B,0x16); //2 1
    write_cmos_sensor(0x360C,0x1F); //2 1
    write_cmos_sensor(0x360D,0xAD); //2 1
    write_cmos_sensor(0x360E,0x19); //2 1
    write_cmos_sensor(0x360F,0x08); //2 1
    write_cmos_sensor(0x3610,0x14); //2 1
    write_cmos_sensor(0x3611,0x26); //2 1
    write_cmos_sensor(0x3612,0x1A); //2 1
    write_cmos_sensor(0x3613,0xB3); //2 1
    write_cmos_sensor(0x35D2,0x7F); //2 1
    write_cmos_sensor(0x35D3,0xFF); //2 1
    write_cmos_sensor(0x35D4,0x70); //2 1
    write_cmos_sensor(0x35D0,0x01); //2 1
    write_cmos_sensor(0x3E44,0x01); //2 1
//checked below
    write_cmos_sensor(0x0111,0x02);//CSI2
    write_cmos_sensor(0x0114,0x01);// 2 lane
    write_cmos_sensor(0x2136,0x18); //2 1
    write_cmos_sensor(0x2137,0x00); //2 1
    write_cmos_sensor(0x0112,0x0A);//csi_data_format 10 bit
    write_cmos_sensor(0x0113,0x0A);//csi_data_format 10 bit
    write_cmos_sensor(0x3016,0x46); //2 1
    write_cmos_sensor(0x3017,0x29); //2 1
    write_cmos_sensor(0x3003,0x03); //2 1
    write_cmos_sensor(0x3045,0x03); //2 1
    write_cmos_sensor(0x3047,0x98); //2 1
    //write_cmos_sensor(0x0305,0x02);//pre_pll_div
    write_cmos_sensor(0x0305,0x04);//pre_pll_div
    write_cmos_sensor(0x0306,0x01);//pll_mult_dummy_hi
    write_cmos_sensor(0x0307,0x18);//pll_mult
    write_cmos_sensor(0x0301,0x0A);//vt_pix_clk_div
    write_cmos_sensor(0x0309,0x0A);//op_pix_clk_div
    //write_cmos_sensor(0x0340,0x07);//frame length
    //write_cmos_sensor(0x0341,0xb6);// Hex(7b6) = 1974
    write_cmos_sensor(0x0340,0x08);//frame length
    write_cmos_sensor(0x0341,0x24);// Hex(824) = 2084
    write_cmos_sensor(0x0344,0x00);//x start
    write_cmos_sensor(0x0345,0x00);
    write_cmos_sensor(0x0346,0x00);//y start
    write_cmos_sensor(0x0347,0x00);
    write_cmos_sensor(0x0348,0x0A);//x end
    write_cmos_sensor(0x0349,0x27);// 2599
    write_cmos_sensor(0x034A,0x07);//y end
    write_cmos_sensor(0x034B,0x9F);// 1951
    write_cmos_sensor(0x034C,0x0A);//;x_output_size
    write_cmos_sensor(0x034D,0x28);//   2600
    write_cmos_sensor(0x034E,0x07);//;Y_output_size
    write_cmos_sensor(0x034F,0xA0);//   1952
    write_cmos_sensor(0x0383,0x01);//x odd
    write_cmos_sensor(0x0387,0x01);//y odd
    write_cmos_sensor(0x0202,0x07);//coarse integration time
    write_cmos_sensor(0x0203,0xad);// 1965
    write_cmos_sensor(0x0205,0xC0);//gain code
    write_cmos_sensor(0x0900,0x00);//disable binning
    write_cmos_sensor(0x0901,0x00);//binning type
    write_cmos_sensor(0x0902,0x00);//binning weighting
    write_cmos_sensor(0x040C,0x0A);//image width after digital crop
    write_cmos_sensor(0x040D,0x28);// HEX(A28) = 2600
    write_cmos_sensor(0x040E,0x07);//image high after digital crop
    write_cmos_sensor(0x040F,0xA0);// HEX(7A0) = 1952
    write_cmos_sensor(0x0101,0x00);//flip+mirror
    write_cmos_sensor(0x0100,0x01);//Streaming

}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF("E\n");

#if 0
		write_cmos_sensor(0x0100,0x00);
		mdelay(50);
    write_cmos_sensor(0x3570,0x01);
    write_cmos_sensor(0x0900,0x01);
    write_cmos_sensor(0x0901,0x22); 
    write_cmos_sensor(0x0300,0x00); 
    write_cmos_sensor(0x0302,0x00);
    write_cmos_sensor(0x0303,0x01); 
    write_cmos_sensor(0x0304,0x00);
    write_cmos_sensor(0x0308,0x00);
    write_cmos_sensor(0x030a,0x00); 
    write_cmos_sensor(0x030b,0x01);
    write_cmos_sensor(0x0340,0x03);
    write_cmos_sensor(0x0341,0xe6); 
    write_cmos_sensor(0x034c,0x05); //x_output_size	***
    write_cmos_sensor(0x034d,0x10); // 1296		***
    write_cmos_sensor(0x034e,0x03); //Y_output_size	***
    write_cmos_sensor(0x034f,0xd0); 
    write_cmos_sensor(0x040c,0x05); //image width after digital crop ***
    write_cmos_sensor(0x040d,0x10); // 1296		***
    write_cmos_sensor(0x040e,0x03); //image high after digital crop ***
    write_cmos_sensor(0x040f,0xd0); // 976               ***
    write_cmos_sensor(0x0202,0x03); //coarse integration time ***
    write_cmos_sensor(0x0203,0xdd); // 989		***
    write_cmos_sensor(0x0205,0x80); //gain code		***
    write_cmos_sensor(0x0306,0x00);
    write_cmos_sensor(0x0307,0xC8);
    write_cmos_sensor(0x0340,0x05);
    write_cmos_sensor(0x0341,0xAF); 
	write_cmos_sensor(0x0100,0x01);
		mdelay(50);
#endif
write_cmos_sensor(0x0100,0x00);
mdelay(50);
//write_cmos_sensor(0x3570,0x);
write_cmos_sensor(0x0900,0x00); //2 1
write_cmos_sensor(0x0901,0x00); //2 1
write_cmos_sensor(0x0300,0x00); 
write_cmos_sensor(0x0302,0x00);
write_cmos_sensor(0x0303,0x01); 
write_cmos_sensor(0x0304,0x00);
write_cmos_sensor(0x0306,0x01);//pll_mult_dummy_hi
write_cmos_sensor(0x0307,0x18);//pll_mult    
write_cmos_sensor(0x0308,0x00);
write_cmos_sensor(0x030a,0x00); 
write_cmos_sensor(0x030b,0x01);
write_cmos_sensor(0x0340,0x07); //2 1
write_cmos_sensor(0x0341,0xc4); //2 1
write_cmos_sensor(0x034C,0x0A); //2 1
write_cmos_sensor(0x034D,0x28); //2 1   ;x_output_size -2600
write_cmos_sensor(0x034E,0x07); //2 1
write_cmos_sensor(0x034F,0xA0); //2 1   ;Y_output_size -1952
write_cmos_sensor(0x040C,0x0A); //2 1
write_cmos_sensor(0x040D,0x28); //2 1
write_cmos_sensor(0x040E,0x07); //2 1
write_cmos_sensor(0x040F,0xA0); //2 1
write_cmos_sensor(0x0202,0x0e); //2 1
write_cmos_sensor(0x0203,0x50); //2 1
write_cmos_sensor(0x0205,0xC0); //2 1
write_cmos_sensor(0x0100,0x01);
mdelay(50);

}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
		write_cmos_sensor(0x0100,0x00);
    mdelay(50);
    //write_cmos_sensor(0x3570,0x);
    write_cmos_sensor(0x0900,0x00); //2 1
    write_cmos_sensor(0x0901,0x00); //2 1
    write_cmos_sensor(0x0300,0x00); 
    write_cmos_sensor(0x0302,0x00);
    write_cmos_sensor(0x0303,0x01); 
    write_cmos_sensor(0x0304,0x00);
    write_cmos_sensor(0x0306,0x01);//pll_mult_dummy_hi
    write_cmos_sensor(0x0307,0x18);//pll_mult    
    write_cmos_sensor(0x0308,0x00);
    write_cmos_sensor(0x030a,0x00); 
    write_cmos_sensor(0x030b,0x01);
    write_cmos_sensor(0x0340,0x07); //2 1
    write_cmos_sensor(0x0341,0xc4); //2 1
    write_cmos_sensor(0x034C,0x0A); //2 1
    write_cmos_sensor(0x034D,0x28); //2 1   ;x_output_size -2600
    write_cmos_sensor(0x034E,0x07); //2 1
    write_cmos_sensor(0x034F,0xA0); //2 1   ;Y_output_size -1952
    write_cmos_sensor(0x040C,0x0A); //2 1
    write_cmos_sensor(0x040D,0x28); //2 1
    write_cmos_sensor(0x040E,0x07); //2 1
    write_cmos_sensor(0x040F,0xA0); //2 1
    write_cmos_sensor(0x0202,0x0e); //2 1
    write_cmos_sensor(0x0203,0x50); //2 1
    write_cmos_sensor(0x0205,0xC0); //2 1
    write_cmos_sensor(0x0100,0x01);
    mdelay(50);
	
		
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);

}
static void hs_video_setting()
{
	LOG_INF("E\n");
	capture_setting(300);
}

static void slim_video_setting()
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
*	*sensor_id : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	
	kal_uint8 retry = 3;
		do {
			*sensor_id = ((read_cmos_sensor(0x2016) << 8) | read_cmos_sensor(0x2017));
			LOG_INF("gaochao_5040_get_imgsensor_id: 0x%x\n", *sensor_id);
			if (*sensor_id != imgsensor_info.sensor_id) {
				*sensor_id =0xffffffff;
				LOG_INF("read HM5040 R1A R2A bate fail\n");	  
				return ERROR_SENSOR_CONNECT_FAIL;
				}
			 else
		{
			*sensor_id=imgsensor_info.sensor_id;
		        break;
		}
			
			LOG_INF("Read sensor id ok, id: 0x%x\n", imgsensor.i2c_write_id);
			//LOG_INF("gaochao_get_imgsensor_id: 0x%x\n", *sensor_id);
			retry--;
		} while(retry > 0);
		
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
	LOG_INF("PLATFORM:MT6595,MIPI 4LANE\n");
	//LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	//kal_uint8 retry = 3;
		do {
			sensor_id = ((read_cmos_sensor(0x2016) << 8) | read_cmos_sensor(0x2017));
			LOG_INF("gaochao_5040_get_imgsensor_id: 0x%x\n", sensor_id);
			if (sensor_id != imgsensor_info.sensor_id) {
				sensor_id =0xffffffff;
				LOG_INF("read HM5040 R1A R2A bate fail\n");	  
				return ERROR_SENSOR_CONNECT_FAIL;
				}
			 else
		{
			sensor_id=imgsensor_info.sensor_id;
		        break;
		}
			
			LOG_INF("Read sensor id ok, id: 0x%x\n", imgsensor.i2c_write_id);
			//LOG_INF("gaochao_get_imgsensor_id: 0x%x\n", *sensor_id);
			retry--;
		} while(retry > 0);	 
	sensor_init();
	mdelay(10);
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
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	mdelay(10);

	#if 0
	if(imgsensor.test_pattern == KAL_TRUE)
	{
		write_cmos_sensor(0x3282,0x01);//DPU OFF
		
		write_cmos_sensor(0x0600,0x00);
		write_cmos_sensor(0x0601,0x02);
		
	}
#endif
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
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	mdelay(10);
	
	
	return ERROR_NONE;
}	/*	normal_video   */

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
	mdelay(10);
	
	return ERROR_NONE;
}	/*	hs_video   */

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
	mdelay(10);
	
	return ERROR_NONE;
}	/*	slim_video	 */



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
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
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
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
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

UINT32 HM5040MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5693_MIPI_RAW_SensorInit	*/
