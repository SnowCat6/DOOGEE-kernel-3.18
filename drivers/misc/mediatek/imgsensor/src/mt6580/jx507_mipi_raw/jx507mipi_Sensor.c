/*****************************************************************************
 *
 * Filename:
 * ---------
 *     jx507mipiraw_Sensor.c
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

#include "jx507mipi_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
//#define __SENSOR_WINDSZIE_INFO__
#define PFX "JX507_camera_sensor"
#define LOG_1 printk( "JX507,MIPI 2LANE\n")
#define LOG_2 printk( "preview 1296*972@30fps; video 1296*972@30fps; capture 5M@30fps\n")
/****************************   Modify end    *******************************************/

static DEFINE_SPINLOCK( imgsensor_drv_lock );

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = JX507_SENSOR_ID,	//0xA507	//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value = 0x40aeb1f5,		//checksum value for Camera Auto Test
	
	.pre = {
		.pclk = 62400000,				//record different mode's pclk
		.linelength = 1930,				//record different mode's linelength
		.framelength = 1078,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width =1264,// 1280,//1312,//1280,		//record different mode's width of grabwindow
		.grabwindow_height = 948,//960,//978,//960,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 91200000,
		.linelength = 3032,
		.framelength = 2006,
		.startx = 0,
		.starty = 0,
		.grabwindow_width =2528,// 2560,//2624,//2560,
		.grabwindow_height = 1896,//1920,//1956,//1920,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 150,
	},
	.cap1 = {
		.pclk = 91200000,
		.linelength = 3032,
		.framelength = 2006,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2528,//2560,//2624,//2560,
		.grabwindow_height =1896,// 1920,//1956,//1920,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 150,
	},
	.normal_video = {
		.pclk = 62400000,
		.linelength = 1930,
		.framelength = 1078,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1264,//1280,//1312,//1280,
		.grabwindow_height =948,// 960,//978,//960,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 62400000,
		.linelength = 2880,
		.framelength = 506,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640,//672,//640,
		.grabwindow_height = 480,//504,//480,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 62400000,
		.linelength = 1930,
		.framelength = 1078,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,//1312,//1280,
		.grabwindow_height = 720,//744,//720,
		.mipi_data_lp2hs_settle_dc = 14,
		.max_framerate = 300,
	},
	
	.margin = 4,						//sensor framelength & shutter margin
	.min_shutter = 1,					//min shutter
	.max_frame_length = 0xffff,			//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,			//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 1,	//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,		//isp gain delay frame for AE cycle
	.ihdr_support = 0,					//1, support; 0,not support
	.ihdr_le_firstline = 0,				//1, le first ; 0, se first
	.sensor_mode_num = 5,				//support sensor mode num
	
	.cap_delay_frame = 4,//4,				//enter capture delay frame num
	.pre_delay_frame = 4,				//enter preview delay frame num
	.video_delay_frame = 4,				//enter video delay frame num
	.hs_video_delay_frame = 3,			//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,		//enter slim video delay frame num
	
	.isp_driving_current		= ISP_DRIVING_6MA,				//mclk driving current
	.sensor_interface_type		= SENSOR_INTERFACE_TYPE_MIPI,	//sensor_interface_type
	.mipi_sensor_type			= MIPI_OPHY_NCSI2,				//0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode		= MIPI_SETTLEDELAY_AUTO,		//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat	= SENSOR_OUTPUT_FORMAT_RAW_Gb,	//sensor output first pixel color
	.mclk						= 24,							//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num				= SENSOR_MIPI_2_LANE,			//mipi lane num
	.i2c_addr_table				= {0x60, 0x68, 0xff},			//record sensor support all write id addr, only supprt 4, must end with 0xff
};

static imgsensor_struct imgsensor = {
	.mirror				 = IMAGE_HV_MIRROR,					//mirrorflip information
	.sensor_mode		 = IMGSENSOR_MODE_INIT,				//IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter			 = 0x3D0,							//current shutter
	.gain				 = 0x0,								//current gain
	.dummy_pixel		 = 0,								//current dummypixel
	.dummy_line			 = 0,								//current dummyline
	.current_fps		 = 300,								//full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en		 = KAL_FALSE,						//auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern		 = KAL_FALSE,						//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	//current scenario id
	.ihdr_en			 = 0,								//sensor need support LE, SE with HDR feature
	.i2c_write_id		 = 0x60,							//record current sensor's i2c write id
};

/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
#if 1  //eric r2
	{ 2592, 1944, 0, 0, 2592, 1944, 1296,  972, 0000, 0000, 1296,  972, 0, 0, 1264,  948},	// Preview 
	{ 2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0, 0, 2528, 1896},	// capture 
	{ 2592, 1944, 0, 0, 2592, 1944, 1296,  972, 0000, 0000, 1296,  972, 0, 0, 1264,  948},	// video 
	{ 2592, 1944, 0, 0, 2592, 1944,  648,  488, 0000, 0000,  648,  488, 0, 0,  648,  480},	// high-speed video 
	{ 2592, 1944, 0, 0, 2592, 1944, 1296,  972, 0000, 0000, 1296,  972, 0, 0, 1264,  948}	// slim video 
#endif
#if 0   //eric r1 can preview
	{ 2560, 1920, 0, 0, 2560, 1920, 1280,  960, 0000, 0000, 1280,  960, 0, 0, 1280,  960},	// Preview 
	{ 2560, 1920, 0, 0, 2560, 1920, 2560, 1920, 0000, 0000, 2560, 1920, 0, 0, 2560, 1920},	// capture 
	{ 2560, 1920, 0, 0, 2560, 1920, 1280,  960, 0000, 0000, 1280,  960, 0, 0, 1280,  960},	// video 
	{ 2560, 1920, 0, 0, 2560, 1920,  648,  488, 0000, 0000,  648,  488, 0, 0,  640,  480},	// hight speed video 
	{ 2560, 1920, 0, 0, 2560, 1920, 1280,  960, 0000, 0000, 1280,  960, 0, 0, 1280,  960}	// slim video 
#endif
#if 0  //wang
	{ 2560, 1920, 0, 0, 2560, 1920, 1312,  978, 0000, 0000, 1280,  960, 0, 0, 1280,  960},	// Preview 
	{ 2560, 1920, 0, 0, 2560, 1920, 2624, 1956, 0000, 0000, 2560, 1920, 0, 0, 2560, 1920},	// capture 
	{ 2560, 1920, 0, 0, 2560, 1920, 1312,  978, 0000, 0000, 1280,  960, 0, 0, 1280,  960},	// video 
	{ 2560, 1920, 0, 0, 2560, 1920,  672,  504, 0000, 0000,  648,  488, 0, 0,  640,  480},	// hight speed video 
	{ 2560, 1920, 0, 0, 2560, 1920, 1312,  978, 0000, 0000, 1280,  960, 0, 0, 1280,  960}	// slim video 
#endif
};

////////////////////////////////////////////////////////////////////////////////////////////////////

static UINT8	gVersion_ID	= 0x00;							//min@20140925

//min@20140805
#ifdef _SENSOR_INITIAL_FLIP
static UINT8	gMirrorFlip = 0x03;
#else
static UINT8	gMirrorFlip = 0x00;
#endif

static UINT8	gBlacksun	= 1;
static UINT32	gBlcCnt		= 0;
static UINT32	gBlcDiv		= 1;
static UINT16	gBlcGain	= 0xFFFF;

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	u8		get_byte = 0;
	char	puSendCmd[2] = {0};
	
	puSendCmd[0] = (char)(addr & 0xFF);
	puSendCmd[1] = 0x00;
	iReadRegI2C( (u8 *)puSendCmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id );
	return (kal_uint16) get_byte;
}
//static kal_uint16 read_cmos_sensor(kal_uint32 addr)
//{
//	kal_uint16 get_byte = 0;
//	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
//	
//	iReadRegI2C( pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id );
//	
//	return get_byte;
//}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[3] = {0};
	
	puSendCmd[0] = (char)(addr & 0xFF);
	puSendCmd[1] = (char)(para & 0xFF);
	puSendCmd[2] = 0x00;
	iWriteRegI2C( (u8 *)puSendCmd, 2, imgsensor.i2c_write_id );
}
//static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
//{
//	iWriteReg( (u16)addr, (u32)para, 2, imgsensor.i2c_write_id );
//}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

static void set_dummy(void)
{
	kal_uint16	line_length	 = 0;
	kal_uint16	frame_length = 0;
	
	printk( "set_dummy()\n" );
	printk( "dummyline=%d, dummypixels=%d\n", imgsensor.dummy_line, imgsensor.dummy_pixel );
	
	frame_length = imgsensor.frame_length;
	line_length	 = imgsensor.line_length;
	// Set total line length
	write_cmos_sensor( 0x21, (line_length  >> 8) & 0xff );
	write_cmos_sensor( 0x20, (line_length      ) & 0xff );
	// Set total frame length
	write_cmos_sensor( 0x23, (frame_length >> 8) & 0xff );
	write_cmos_sensor( 0x22, (frame_length     ) & 0xff );
}

static kal_uint32 return_sensor_id(void)
{
	return( (read_cmos_sensor( 0x0A ) << 8) | read_cmos_sensor( 0x0B ) );
}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16	dummy_line;
	kal_uint32	frame_length = imgsensor.frame_length;
	//unsigned long	flags;
	
	printk( "framerate=%d, min framelength should enable? \n", framerate, min_framelength_en );
	
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	
	spin_lock( &imgsensor_drv_lock );
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line   = imgsensor.frame_length - imgsensor.min_frame_length;
	
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if( dummy_line < 0 )
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	
	if( imgsensor.frame_length > imgsensor_info.max_frame_length )
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line   = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if( min_framelength_en )
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock( &imgsensor_drv_lock );
	
	set_dummy();
}

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
	unsigned long	flags;
	kal_uint16		realtime_fps = 0;
	kal_uint32		frame_length = 0;
	
	spin_lock_irqsave( &imgsensor_drv_lock, flags );
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore( &imgsensor_drv_lock, flags );
	
	//write_shutter( shutter );
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock( &imgsensor_drv_lock );
	if( shutter > imgsensor.min_frame_length - imgsensor_info.margin )
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	
	if( imgsensor.frame_length > imgsensor_info.max_frame_length )
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock( &imgsensor_drv_lock );
	
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if( imgsensor.autoflicker_en ) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if( realtime_fps >= 297 && realtime_fps <= 305 )
			set_max_framerate( 296,0 );
		else if( realtime_fps >= 147 && realtime_fps <= 150 )
			set_max_framerate( 146,0 );
		else {
			// Extend frame length
			write_cmos_sensor( 0x23, (imgsensor.frame_length >> 8) & 0xff );
			write_cmos_sensor( 0x22, (imgsensor.frame_length     ) & 0xff );
		}
	} else {
		// Extend frame length
		write_cmos_sensor( 0x23, (imgsensor.frame_length >> 8) & 0xff );
		write_cmos_sensor( 0x22, (imgsensor.frame_length     ) & 0xff );
	}
	
	// Update Shutter
	write_cmos_sensor( 0x02, (shutter >> 8) & 0xff );
	write_cmos_sensor( 0x01, (shutter     ) & 0xff );
	printk( "Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length );
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16	iReg	 = 0x0000;
	kal_uint16	iGain	 = gain;
	kal_uint16	BaseGain = 0x40;
	
	if( iGain <  BaseGain ) {
		iReg =0;
	} else if( iGain < 2 * BaseGain ) {
		iReg = 16 * iGain / BaseGain - 16;
	} else if( iGain < 4 * BaseGain ) {
		iReg |= 0x10;
		iReg |= (8 *iGain / BaseGain - 16);
	} else if( iGain < 8 * BaseGain ) {
		iReg |= 0x30;
		iReg |= (4 * iGain / BaseGain - 16);
	} else if( iGain < 16 * BaseGain ) {
		iReg |= 0x70;
		iReg |= (2 * iGain /BaseGain - 16);
	} else if( iGain < 32 * BaseGain ) {
		iReg |= 0xF0;
		iReg |= (iGain /BaseGain - 16);
	} else {

	}
	printk( "gain2reg: isp gain=%d, sensor gain=0x%x\n", iGain, iReg );
	
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
*    iGain : sensor global gain( base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16	reg_gain = 0;
	
	reg_gain = gain2reg( gain );
	
	spin_lock( &imgsensor_drv_lock );
	imgsensor.gain = reg_gain;
	spin_unlock( &imgsensor_drv_lock );
	printk( "gain=0x%x, reg_gain=0x%x\n ", gain, reg_gain );
	
	write_cmos_sensor( 0x00, reg_gain & 0xff );
	
	return gain;
}

static void ihdr_write_shutter_gain( kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	printk( "le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain );
}

static void set_mirror_flip(kal_uint8 image_mirror)
{
	#if 0
	kal_int8 tmp = 0;
	
	printk( "set_mirror_flip(): image_mirror=%d\n", image_mirror );
	printk( "image_mirror=%d\n", image_mirror );
	
	tmp = (kal_int8)read_cmos_sensor( 0x12 );
	switch( image_mirror ) {
		case IMAGE_NORMAL:
			#ifdef _SENSOR_INITIAL_FLIP //min@20131224
			tmp = (tmp & 0xCF) | 0x10;
			#else
			tmp = tmp & 0xCF;
			#endif
			write_cmos_sensor( 0x12, (kal_int32)tmp );
			break;
		case IMAGE_H_MIRROR:
			#ifdef _SENSOR_INITIAL_FLIP //min@20131224
			tmp = tmp | 0x30;
			#else
			tmp = (tmp & 0xCF) | 0x20;
			#endif
			write_cmos_sensor( 0x12, (kal_int32)tmp );
			break;
		case IMAGE_V_MIRROR:
			#ifdef _SENSOR_INITIAL_FLIP //min@20131224
			tmp = tmp & 0xCF;
			#else
			tmp = (tmp & 0xCF) | 0x10;
			#endif
			write_cmos_sensor( 0x12, (kal_int32)tmp );
			break;
		case IMAGE_HV_MIRROR:
			#ifdef _SENSOR_INITIAL_FLIP //min@20131224
			tmp = (tmp & 0xCF) | 0x20;
			#else
			tmp = tmp | 0x30;
			#endif
			write_cmos_sensor( 0x12, (kal_int32)tmp );
			break;
		default:
			printk( "Error image_mirror setting" );
	}
	#endif
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
static void night_mode( kal_bool enable)
{
	/*No Need to implement this function*/
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

static void sensor_init(void)
{
	printk( ">> sensor_init()\n" );
	
	printk( "Reset sensor...\n" );
	write_cmos_sensor( 0x12, 0x80 );
	mDELAY( 6 );
	
	#ifdef _JX507_INI_20140820 //Terra20141103-05 //Terra20140820-01
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x73 );
		#else
		write_cmos_sensor( 0x12, 0x43 );
		#endif
		#ifdef _BLACKSUN
		write_cmos_sensor( 0x0C, 0x00 );			//min@20150225
		#endif
		write_cmos_sensor( 0x0D, 0x00 );
		write_cmos_sensor( 0x0E, 0x10 );
		write_cmos_sensor( 0x0F, 0x04 );
		write_cmos_sensor( 0x10, 0x0D );
		write_cmos_sensor( 0x11, 0x80 );
		write_cmos_sensor( 0x20, 0x8A );
		write_cmos_sensor( 0x21, 0x07 );
		write_cmos_sensor( 0x22, 0x36 );
		write_cmos_sensor( 0x23, 0x04 );
		write_cmos_sensor( 0x24, 0x00 );
		write_cmos_sensor( 0x25, 0xC0 );
		write_cmos_sensor( 0x26, 0x35 );
		write_cmos_sensor( 0x27, 0x97 ); 
		write_cmos_sensor( 0x28, 0x07 );
		write_cmos_sensor( 0x29, 0x02 );
		write_cmos_sensor( 0x2A, 0x81 );
		write_cmos_sensor( 0x2B, 0x2A );
		write_cmos_sensor( 0x2C, 0x06 );
		write_cmos_sensor( 0x2D, 0x03 );
		write_cmos_sensor( 0x2E, 0xE8 );
		write_cmos_sensor( 0x2F, 0x04 );
		write_cmos_sensor( 0x30, 0x94 );			//Terra20141103-05
		if( gVersion_ID == 9 ) //AH,AI
		write_cmos_sensor( 0x31, 0x0A );			//Terra20141103-05
		else //AG2
		write_cmos_sensor( 0x31, 0x0E );
		write_cmos_sensor( 0x32, 0xB4 );			//Terra20141103-05
		write_cmos_sensor( 0x33, 0x14 );			//Terra20141103-05
		write_cmos_sensor( 0x34, 0x2A );			//Terra20141103-05
		write_cmos_sensor( 0x35, 0xCA );
		write_cmos_sensor( 0x1D, 0x00 );
		write_cmos_sensor( 0x1E, 0x00 );
		write_cmos_sensor( 0x6C, 0x40 );
		write_cmos_sensor( 0x73, 0x33 );
		write_cmos_sensor( 0x70, 0x69 );
		write_cmos_sensor( 0x76, 0x40 );
		write_cmos_sensor( 0x77, 0x06 );
		write_cmos_sensor( 0x74, 0x02 );
		write_cmos_sensor( 0x78, 0x2C );
		write_cmos_sensor( 0x5F, 0x43 );
		write_cmos_sensor( 0x60, 0x23 );	//0x27
		write_cmos_sensor( 0x61, 0xF8 );
		write_cmos_sensor( 0x62, 0x04 );	//0x00
		write_cmos_sensor( 0x63, 0xC0 );
		write_cmos_sensor( 0x64, 0x07 );
		write_cmos_sensor( 0x65, 0x00 );
		if( gVersion_ID == 9 ) //AH,AI
		write_cmos_sensor( 0x67, 0x50 );	//0x58
		else //AG2
		write_cmos_sensor( 0x67, 0x70 );	//0x79
		write_cmos_sensor( 0x6A, 0x3A );	//0x3F
		write_cmos_sensor( 0x69, 0x74 );
		write_cmos_sensor( 0x13, 0x81 );
		write_cmos_sensor( 0x14, 0x80 );
		write_cmos_sensor( 0x16, 0xC0 );
		write_cmos_sensor( 0x17, 0x40 );
		write_cmos_sensor( 0x18, 0x43 );
		write_cmos_sensor( 0x19, 0x29 );
		write_cmos_sensor( 0x38, 0xE9 );
		write_cmos_sensor( 0x39, 0x20 );
		write_cmos_sensor( 0x4a, 0x03 );
		write_cmos_sensor( 0x49, 0x10 );
		write_cmos_sensor( 0x50, 0x10 );
		write_cmos_sensor( 0x6D, 0x0A );
		write_cmos_sensor( 0x71, 0x6A );
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x33 );
		#else
		write_cmos_sensor( 0x12, 0x03 );
		#endif
	#endif //_JX507_INI_20140820
	////////////////////////////////////////////////////////////////////////////
	#ifdef _JX507_INI_20140211
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x73 );
		#else
		write_cmos_sensor( 0x12, 0x43 );
		#endif
		write_cmos_sensor( 0x0D, 0x00 );
		write_cmos_sensor( 0x0E, 0x10 );
		write_cmos_sensor( 0x0F, 0x04 );
		write_cmos_sensor( 0x10, 0x13 );
		write_cmos_sensor( 0x11, 0x80 );
		write_cmos_sensor( 0x20, 0x88 );
		write_cmos_sensor( 0x21, 0x07 );
		write_cmos_sensor( 0x22, 0x2A );
		write_cmos_sensor( 0x23, 0x06 );
		write_cmos_sensor( 0x24, 0x00 );
		write_cmos_sensor( 0x25, 0xC0 );
		write_cmos_sensor( 0x26, 0x35 );
		write_cmos_sensor( 0x27, 0x96 );
		write_cmos_sensor( 0x28, 0x07 );
		write_cmos_sensor( 0x29, 0x02 );
		write_cmos_sensor( 0x2A, 0x81 );
		write_cmos_sensor( 0x2B, 0x2A );
		write_cmos_sensor( 0x2C, 0x06 );
///		write_cmos_sensor( 0x2C, 0x02 );
		write_cmos_sensor( 0x2D, 0x03 );
		write_cmos_sensor( 0x2E, 0xE8 );
		write_cmos_sensor( 0x2F, 0x04 );
		write_cmos_sensor( 0x31, 0x14 );
		write_cmos_sensor( 0x32, 0xBE );
		write_cmos_sensor( 0x33, 0x18 );
		write_cmos_sensor( 0x34, 0x3E );
		write_cmos_sensor( 0x35, 0xEC );
		#ifdef _DRIVE_OPT
		write_cmos_sensor( 0x5F, 0x43 );
		#else
		write_cmos_sensor( 0x5F, 0x03 );
		#endif
		write_cmos_sensor( 0x60, 0x27 );
		write_cmos_sensor( 0x61, 0xFC );
		write_cmos_sensor( 0x62, 0x00 );
		write_cmos_sensor( 0x63, 0xC0 );
		write_cmos_sensor( 0x64, 0x07 );
		write_cmos_sensor( 0x67, 0x79 );
		write_cmos_sensor( 0x69, 0x74 );
		write_cmos_sensor( 0x6A, 0x3A );
		write_cmos_sensor( 0x13, 0x81 );
		write_cmos_sensor( 0x14, 0x80 );
		write_cmos_sensor( 0x16, 0xC0 );
		write_cmos_sensor( 0x17, 0x40 );
		write_cmos_sensor( 0x18, 0xD9 );
		write_cmos_sensor( 0x19, 0x29 );
		write_cmos_sensor( 0x38, 0xE9 );
		write_cmos_sensor( 0x39, 0x20 );
		write_cmos_sensor( 0x4a, 0x03 );
		write_cmos_sensor( 0x49, 0x10 );
		write_cmos_sensor( 0x50, 0x10 );
		write_cmos_sensor( 0x1D, 0x00 );
		write_cmos_sensor( 0x1E, 0x00 );
		write_cmos_sensor( 0x68, 0x00 );
		write_cmos_sensor( 0x6C, 0x00 );
		write_cmos_sensor( 0x73, 0x33 );
		write_cmos_sensor( 0x70, 0x69 );
		write_cmos_sensor( 0x76, 0x40 );
		write_cmos_sensor( 0x77, 0x06 );
		write_cmos_sensor( 0x6D, 0x09 );
		write_cmos_sensor( 0x74, 0x02 );
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x33 );
		#else
		write_cmos_sensor( 0x12, 0x03 );
		#endif
	#endif //_JX507_INI_20140211
	
	printk( "<< sensor_init()\n" );
}

///static void preview_setting(void)
static void sensor_1M_setting(void)
{
	printk( ">> sensor_1M_setting()\n" );
	
	#ifdef _JX507_INI_20140820 //Terra20141103-05 //Terra20140820-01
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x73 );
		#else
		write_cmos_sensor( 0x12, 0x43 );
		#endif
		#ifdef _BLACKSUN
		if( gBlacksun )
			write_cmos_sensor( 0x0C, 0x00 );		//min@20150225
		else
			write_cmos_sensor( 0x0C, 0x40 );		//min@20150225
		#endif
		write_cmos_sensor( 0x0D, 0x00 );
		write_cmos_sensor( 0x0E, 0x10 );
		write_cmos_sensor( 0x0F, 0x04 );
		write_cmos_sensor( 0x10, 0x0D );
		write_cmos_sensor( 0x11, 0x80 );
		write_cmos_sensor( 0x20, 0x8A );
		write_cmos_sensor( 0x21, 0x07 );
		write_cmos_sensor( 0x22, 0x36 );
		write_cmos_sensor( 0x23, 0x04 );
		write_cmos_sensor( 0x24, 0x00 );
		write_cmos_sensor( 0x25, 0xC0 );
		write_cmos_sensor( 0x26, 0x35 );
		write_cmos_sensor( 0x27, 0x9b ); //97 hhh
		write_cmos_sensor( 0x28, 0x05 );
		write_cmos_sensor( 0x29, 0x02 );
		write_cmos_sensor( 0x2A, 0x81 );
		write_cmos_sensor( 0x2B, 0x2A );
		write_cmos_sensor( 0x2C, 0x00 ); //06 hhh
		write_cmos_sensor( 0x2D, 0x03 );
		write_cmos_sensor( 0x2E, 0xE8 );
		write_cmos_sensor( 0x2F, 0x04 );
		write_cmos_sensor( 0x30, 0x94 );			//Terra20141103-05
		if( gVersion_ID == 9 ) //AH,AI
		write_cmos_sensor( 0x31, 0x0A );			//Terra20141103-05
		else //AG2
		write_cmos_sensor( 0x31, 0x0E );
		write_cmos_sensor( 0x32, 0xB4 );			//Terra20141103-05
		write_cmos_sensor( 0x33, 0x14 );			//Terra20141103-05
		write_cmos_sensor( 0x34, 0x2A );			//Terra20141103-05
		write_cmos_sensor( 0x35, 0xCA );
		write_cmos_sensor( 0x1D, 0x00 );
		write_cmos_sensor( 0x1E, 0x00 );
		write_cmos_sensor( 0x6C, 0x40 );
		write_cmos_sensor( 0x73, 0x33 );
		write_cmos_sensor( 0x70, 0x69 );
		write_cmos_sensor( 0x76, 0x40 );
		write_cmos_sensor( 0x77, 0x06 );
		write_cmos_sensor( 0x74, 0x02 );
		write_cmos_sensor( 0x78, 0x2C );
		write_cmos_sensor( 0x5F, 0x43 );
		write_cmos_sensor( 0x60, 0x23 );	//0x27
		write_cmos_sensor( 0x61, 0xF8 );
		write_cmos_sensor( 0x62, 0x04 );	//0x00
		write_cmos_sensor( 0x63, 0xC0 );
		write_cmos_sensor( 0x64, 0x07 );
		write_cmos_sensor( 0x65, 0x00 );
		if( gVersion_ID == 9 ) //AH,AI
		write_cmos_sensor( 0x67, 0x50 );	//0x58
		else //AG2
		write_cmos_sensor( 0x67, 0x70 );	//0x79
		write_cmos_sensor( 0x6A, 0x3A );	//0x3F
		write_cmos_sensor( 0x69, 0x74 );
		write_cmos_sensor( 0x13, 0x81 );
		write_cmos_sensor( 0x14, 0x80 );
		write_cmos_sensor( 0x16, 0xC0 );
		write_cmos_sensor( 0x17, 0x40 );
		write_cmos_sensor( 0x18, 0x43 );
		write_cmos_sensor( 0x19, 0x29 );
		write_cmos_sensor( 0x38, 0xE9 );
		write_cmos_sensor( 0x39, 0x20 );
		write_cmos_sensor( 0x4a, 0x03 );
		write_cmos_sensor( 0x49, 0x10 );
		write_cmos_sensor( 0x50, 0x10 );
		write_cmos_sensor( 0x6D, 0x0A );
		write_cmos_sensor( 0x71, 0x6A );
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x33 );
		#else
		write_cmos_sensor( 0x12, 0x03 );
		#endif
	#endif //_JX507_INI_20140820
	////////////////////////////////////////////////////////////////////////////
	#ifdef _JX507_INI_20140211
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x73 );
		#else
		write_cmos_sensor( 0x12, 0x43 );
		#endif
		write_cmos_sensor( 0x0D, 0x00 );
		write_cmos_sensor( 0x0E, 0x10 );
		write_cmos_sensor( 0x0F, 0x04 );
		write_cmos_sensor( 0x10, 0x13 );
		write_cmos_sensor( 0x11, 0x80 );
		write_cmos_sensor( 0x20, 0x88 );
		write_cmos_sensor( 0x21, 0x07 );
		write_cmos_sensor( 0x22, 0x2A );
		write_cmos_sensor( 0x23, 0x06 );
		write_cmos_sensor( 0x24, 0x00 );
		write_cmos_sensor( 0x25, 0xC0 );
		write_cmos_sensor( 0x26, 0x35 );
		write_cmos_sensor( 0x27, 0x9a ); //96  hhh
		write_cmos_sensor( 0x28, 0x07 );
		write_cmos_sensor( 0x29, 0x02 );
		write_cmos_sensor( 0x2A, 0x81 );
		write_cmos_sensor( 0x2B, 0x2A );
		write_cmos_sensor( 0x2C, 0x02 ); //06 hhh
///		write_cmos_sensor( 0x2C, 0x02 );
		write_cmos_sensor( 0x2D, 0x03 );
		write_cmos_sensor( 0x2E, 0xE8 );
		write_cmos_sensor( 0x2F, 0x04 );
		write_cmos_sensor( 0x31, 0x14 );
		write_cmos_sensor( 0x32, 0xBE );
		write_cmos_sensor( 0x33, 0x18 );
		write_cmos_sensor( 0x34, 0x3E );
		write_cmos_sensor( 0x35, 0xEC );
		#ifdef _DRIVE_OPT
		write_cmos_sensor( 0x5F, 0x43 );
		#else
		write_cmos_sensor( 0x5F, 0x03 );
		#endif
		write_cmos_sensor( 0x60, 0x27 );
		write_cmos_sensor( 0x61, 0xFC );
		write_cmos_sensor( 0x62, 0x00 );
		write_cmos_sensor( 0x63, 0xC0 );
		write_cmos_sensor( 0x64, 0x07 );
	 	write_cmos_sensor( 0x67, 0x79 );
		write_cmos_sensor( 0x69, 0x74 );
		write_cmos_sensor( 0x6A, 0x3A );
		write_cmos_sensor( 0x13, 0x81 );
		write_cmos_sensor( 0x14, 0x80 );
		write_cmos_sensor( 0x16, 0xC0 );
		write_cmos_sensor( 0x17, 0x40 );
		write_cmos_sensor( 0x18, 0xD9 );
		write_cmos_sensor( 0x19, 0x29 );
		write_cmos_sensor( 0x38, 0xE9 );
		write_cmos_sensor( 0x39, 0x20 );
		write_cmos_sensor( 0x4a, 0x03 );
		write_cmos_sensor( 0x49, 0x10 );
		write_cmos_sensor( 0x50, 0x10 );
		write_cmos_sensor( 0x1D, 0x00 );
		write_cmos_sensor( 0x1E, 0x00 );
		write_cmos_sensor( 0x68, 0x00 );
		write_cmos_sensor( 0x6C, 0x00 );
		write_cmos_sensor( 0x73, 0x33 );
		write_cmos_sensor( 0x70, 0x69 );
		write_cmos_sensor( 0x76, 0x40 );
		write_cmos_sensor( 0x77, 0x06 );
		write_cmos_sensor( 0x6D, 0x09 );
		write_cmos_sensor( 0x74, 0x02 );
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x33 );
		#else
		write_cmos_sensor( 0x12, 0x03 );
		#endif
	#endif //_JX507_INI_20140211
	
	printk( "<< sensor_1M_setting()\n" );
}

///static void capture_setting(kal_uint16 currefps)
static void sensor_5M_setting(kal_uint16 currefps)
{
	printk( ">> sensor_5M_setting(): currefps=%d\n", currefps );
	printk( "E! currefps:%d\n", currefps );
	
	#ifdef _JX507_INI_20140820 //Terra20141103-05
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x70 );
		#else
		write_cmos_sensor( 0x12, 0x40 );
		#endif
		#ifdef _BLACKSUN
		if( gBlacksun )
			write_cmos_sensor( 0x0C, 0x00 );		//min@20150225
		else
			write_cmos_sensor( 0x0C, 0x40 );		//min@20150225
		#endif
		write_cmos_sensor( 0x0D, 0x00 );
		write_cmos_sensor( 0x0E, 0x10 );
		write_cmos_sensor( 0x0F, 0x04 );
		write_cmos_sensor( 0x10, 0x13 );
		write_cmos_sensor( 0x11, 0x80 );
		write_cmos_sensor( 0x20, 0xE6 );
		write_cmos_sensor( 0x21, 0x05 );
		write_cmos_sensor( 0x22, 0xD6 );
		write_cmos_sensor( 0x23, 0x07 );
		write_cmos_sensor( 0x24, 0x10 );
		write_cmos_sensor( 0x25, 0x98 );
		write_cmos_sensor( 0x26, 0x75 );
		write_cmos_sensor( 0x27, 0xE8 );  // e4  hhh
		write_cmos_sensor( 0x28, 0x13 );  //0d  hhh
		write_cmos_sensor( 0x29, 0x00 );
		write_cmos_sensor( 0x2A, 0xD9 );
		write_cmos_sensor( 0x2B, 0x28 );
		write_cmos_sensor( 0x2C, 0x00 );
		write_cmos_sensor( 0x2D, 0x00 );
		write_cmos_sensor( 0x2E, 0xEB );
		write_cmos_sensor( 0x2F, 0x04 );
		write_cmos_sensor( 0x30, 0x8E );
		if( gVersion_ID == 9 ) //AH,AI
		write_cmos_sensor( 0x31, 0x08 );			//Terra20141103-05
		else //AG2
		write_cmos_sensor( 0x31, 0x0A );
		write_cmos_sensor( 0x32, 0xA6 );			//Terra20141103-05
		write_cmos_sensor( 0x33, 0x0E );			//Terra20141103-05
		write_cmos_sensor( 0x34, 0x1F );
		write_cmos_sensor( 0x35, 0xB6 );
		write_cmos_sensor( 0x1D, 0x00 );
		write_cmos_sensor( 0x1E, 0x00 );
		write_cmos_sensor( 0x6C, 0x40 );
		write_cmos_sensor( 0x73, 0x33 );
		write_cmos_sensor( 0x70, 0x68 );
		write_cmos_sensor( 0x76, 0xA8 );
		write_cmos_sensor( 0x77, 0x0C );
		write_cmos_sensor( 0x74, 0x02 );
		write_cmos_sensor( 0x78, 0x18 );			//Terra20141103-05
		write_cmos_sensor( 0x5F, 0x43 );
		write_cmos_sensor( 0x60, 0x23 );	//0x27
		write_cmos_sensor( 0x61, 0xF8 );
		write_cmos_sensor( 0x62, 0x04 );	//0x00
		write_cmos_sensor( 0x63, 0xC0 );
		write_cmos_sensor( 0x64, 0x07 );
		write_cmos_sensor( 0x65, 0x00 );
		if( gVersion_ID == 9 ) //AH,AI
		write_cmos_sensor( 0x67, 0x50 );	//0x58
		else //AG2
		write_cmos_sensor( 0x67, 0x70 );	//0x79
		write_cmos_sensor( 0x6A, 0x3A );	//0x3F
		write_cmos_sensor( 0x69, 0x74 );
		write_cmos_sensor( 0x13, 0x81 );
		write_cmos_sensor( 0x14, 0x80 );
		write_cmos_sensor( 0x16, 0xC0 );
		write_cmos_sensor( 0x17, 0x40 );
		write_cmos_sensor( 0x18, 0x2C );
		write_cmos_sensor( 0x19, 0x21 );
		write_cmos_sensor( 0x38, 0xE7 );
		write_cmos_sensor( 0x39, 0x34 );
		write_cmos_sensor( 0x4a, 0x03 );
		write_cmos_sensor( 0x49, 0x10 );
		write_cmos_sensor( 0x50, 0x10 );
		write_cmos_sensor( 0x6D, 0x0A );
		write_cmos_sensor( 0x71, 0x6A );
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x30 );
		#else
		write_cmos_sensor( 0x12, 0x00 );
		#endif
	#endif //_JX507_INI_20140820
	////////////////////////////////////////////////////////////////////////////
	#ifdef _JX507_INI_20140211
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x70 );
		#else
		write_cmos_sensor( 0x12, 0x40 );
		#endif
		write_cmos_sensor( 0x0D, 0x00 );
		write_cmos_sensor( 0x0E, 0x10 );
		write_cmos_sensor( 0x0F, 0x04 );
		write_cmos_sensor( 0x10, 0x13 );
		write_cmos_sensor( 0x11, 0x80 );
		write_cmos_sensor( 0x20, 0xA8 );
		write_cmos_sensor( 0x21, 0x0B );
		write_cmos_sensor( 0x22, 0xF6 );
		write_cmos_sensor( 0x23, 0x07 );
		write_cmos_sensor( 0x24, 0x20 );
		write_cmos_sensor( 0x25, 0x98 );
		write_cmos_sensor( 0x26, 0x7A );
		write_cmos_sensor( 0x27, 0x9f );  //9d  hhh
///		write_cmos_sensor( 0x27, 0x99 );
		write_cmos_sensor( 0x28, 0x19 );
///		write_cmos_sensor( 0x28, 0x0D );
		write_cmos_sensor( 0x29, 0x01 );
		write_cmos_sensor( 0x2A, 0x80 );
		write_cmos_sensor( 0x2B, 0x29 );
		write_cmos_sensor( 0x2C, 0x01 );
///		write_cmos_sensor( 0x2C, 0x00 );
		write_cmos_sensor( 0x2D, 0x00 );
		write_cmos_sensor( 0x2E, 0xEA );
		write_cmos_sensor( 0x2F, 0x04 );
		write_cmos_sensor( 0x31, 0x14 );
		write_cmos_sensor( 0x32, 0xBE );
		write_cmos_sensor( 0x33, 0x18 );
		write_cmos_sensor( 0x34, 0x3E );
		write_cmos_sensor( 0x35, 0xEC );
		#ifdef _DRIVE_OPT //min@20140331
		write_cmos_sensor( 0x5F, 0x43 );
		#else
		write_cmos_sensor( 0x5F, 0x03 );
		#endif
		write_cmos_sensor( 0x60, 0x27 );
		write_cmos_sensor( 0x61, 0xFC );
		write_cmos_sensor( 0x62, 0x00 );
		write_cmos_sensor( 0x63, 0xC0 );
		write_cmos_sensor( 0x64, 0x07 );
		write_cmos_sensor( 0x67, 0x79 );
		write_cmos_sensor( 0x69, 0x74 );
		write_cmos_sensor( 0x6A, 0x3A );
		write_cmos_sensor( 0x13, 0x81 );
		write_cmos_sensor( 0x14, 0x80 );
		write_cmos_sensor( 0x16, 0xC0 );
		write_cmos_sensor( 0x17, 0x40 );
		write_cmos_sensor( 0x18, 0x31 );
		write_cmos_sensor( 0x19, 0x29 );
		write_cmos_sensor( 0x38, 0xE7 );
		write_cmos_sensor( 0x39, 0x34 );
		write_cmos_sensor( 0x4a, 0x03 );
		write_cmos_sensor( 0x49, 0x10 );
		write_cmos_sensor( 0x50, 0x10 );
		write_cmos_sensor( 0x1D, 0x00 );
		write_cmos_sensor( 0x1E, 0x00 );
		write_cmos_sensor( 0x68, 0x00 );
		write_cmos_sensor( 0x6C, 0x00 );
		write_cmos_sensor( 0x73, 0x33 );
		write_cmos_sensor( 0x70, 0x69 );
		write_cmos_sensor( 0x76, 0xA8 );
		write_cmos_sensor( 0x77, 0x0C );
		write_cmos_sensor( 0x6D, 0x09 );
		write_cmos_sensor( 0x74, 0x02 );
		#ifdef _SENSOR_INITIAL_FLIP
		write_cmos_sensor( 0x12, 0x30 );
		#else
		write_cmos_sensor( 0x12, 0x00 );
		#endif
	#endif //_JX507_INI_20140211
	
	mDELAY( 30 );
	
	printk( "<< sensor_5M_setting()\n" );
}

static void normal_video_setting(kal_uint16 currefps)
{
	printk( ">> normal_video_setting(): currefps=%d\n", currefps );
	printk( "E! currefps:%d\n", currefps );
	
	//TODO:
	mDELAY( 30 );
	
	printk( "<< normal_video_setting()\n" );
}

static void hs_video_setting(void)
{
	printk( ">> hs_video_setting()\n" );
	
	//TODO:
	mDELAY( 30 );
	
	printk( "<< hs_video_setting()\n" );
}

static void slim_video_setting(void)
{
	printk( ">> slim_video_setting()\n" );
	
	//TODO:
	mDELAY( 30 );
	
	printk( "<< slim_video_setting()\n" );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	u16	tmp = 0;
	
	printk( ">> set_test_pattern_mode(): enable=%d\n", enable );
	printk( "enable: %d\n", enable );
	
	if( enable ) {
		tmp = read_cmos_sensor( 0x0C ) | 0x80;
		write_cmos_sensor( 0x0C, tmp );
	} else {
		tmp = read_cmos_sensor( 0x0C ) & 0x7F;
		write_cmos_sensor( 0x0C, tmp );
	}
	
	spin_lock( &imgsensor_drv_lock );
	imgsensor.test_pattern = enable;
	spin_unlock( &imgsensor_drv_lock );
	
	printk( "<< set_test_pattern_mode()\n" );
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
	kal_uint8	i = 0;
	kal_uint8	retry = 2;
	
	printk( ">> get_imgsensor_id()\n" );
	
	//sensor have two i2c address 0x60 0x61 & 0x68 0x69, we should detect the module used i2c address.
	while( imgsensor_info.i2c_addr_table[i] != 0xff ) {
		spin_lock( &imgsensor_drv_lock );
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock( &imgsensor_drv_lock );
		
		do {
			*sensor_id = return_sensor_id();
			if( *sensor_id == imgsensor_info.sensor_id ) {
				printk( "[JX507] i2c_write_id=0x%x, sensor_id=0x%x\n", imgsensor.i2c_write_id, *sensor_id );
				printk( "<< get_imgsensor_id(): OK\n" );
				return ERROR_NONE;
			}
			printk( "[JX507] Read sensor id fail, write_id=0x%x, sensor_id=0x%x\n", imgsensor.i2c_write_id, *sensor_id );
			retry--;
		} while( retry > 0 );
		
		i++;
		retry = 2;
	}
	if( *sensor_id != imgsensor_info.sensor_id ) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
		*sensor_id = 0xFFFFFFFF;				/// Invalid sensor ID
		printk( "<< get_imgsensor_id(): FAIL\n" );
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	
	printk( "<< get_imgsensor_id()\n" );
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
	kal_uint8	i = 0;
	kal_uint8	retry = 2;
	kal_uint32	sensor_id = 0;
	kal_uint8	value = 0;
	
	printk( ">> open()\n" );
	
	LOG_1;
	LOG_2;
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while( imgsensor_info.i2c_addr_table[i] != 0xff ) {
		spin_lock( &imgsensor_drv_lock );
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock( &imgsensor_drv_lock );
		
		do {
			sensor_id = return_sensor_id();
			if( sensor_id == imgsensor_info.sensor_id ) {
				printk( "[JX507] i2c_write_id=0x%x, sensor_id=0x%x\n", imgsensor.i2c_write_id, sensor_id );
				break;
			}
			printk( "[JX507] Read sensor id fail, write_id=0x%x, sensor_id=0x%x\n", imgsensor.i2c_write_id, sensor_id );
			retry--;
		} while( retry > 0 );
		
		i++;
		if( sensor_id == imgsensor_info.sensor_id ) {
			//min@20141111 begin
			value = read_cmos_sensor( 0x1E );
			if( value == 0x1C )	gVersion_ID = 7;	//AG
			else				gVersion_ID = 9;	//AH,AI
			printk( "[JX507] gVersion_ID=%d\n", gVersion_ID );
			//min@20141111 end
			break;
		}
		retry = 2;
	}
	if( imgsensor_info.sensor_id != sensor_id ) {
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	
	/* initail sequence write in  */
	sensor_init();
	
	spin_lock( &imgsensor_drv_lock );
	imgsensor.autoflicker_en	= KAL_FALSE;
	imgsensor.sensor_mode		= IMGSENSOR_MODE_INIT;
	imgsensor.pclk				= imgsensor_info.pre.pclk;
	imgsensor.frame_length		= imgsensor_info.pre.framelength;
	imgsensor.line_length		= imgsensor_info.pre.linelength;
	imgsensor.min_frame_length	= imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel		= 0;
	imgsensor.dummy_line		= 0;
	imgsensor.ihdr_en			= 0;
	imgsensor.test_pattern		= KAL_FALSE;
	imgsensor.current_fps		= imgsensor_info.pre.max_framerate;
	spin_unlock( &imgsensor_drv_lock );
	
	printk( "<< open()\n" );
	return ERROR_NONE;
}

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
	printk( ">> close()\n" );
	
	/*No Need to implement this function*/
	write_cmos_sensor( 0x12, 0x40 );
	write_cmos_sensor( 0x6c, 0x80 );
	
	printk( "<< close()\n" );
	return ERROR_NONE;
}

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
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> preview()\n" );
	
	spin_lock( &imgsensor_drv_lock );
	imgsensor.sensor_mode		= IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk				= imgsensor_info.pre.pclk;
	//imgsensor.video_mode		= KAL_FALSE;
	imgsensor.line_length		= imgsensor_info.pre.linelength;
	imgsensor.frame_length		= imgsensor_info.pre.framelength;
	imgsensor.min_frame_length	= imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en	= KAL_FALSE;
	spin_unlock( &imgsensor_drv_lock );
	
	sensor_1M_setting();
///	preview_setting();
//	set_mirror_flip( imgsensor.mirror );
	
	printk( "<< preview()\n" );
	return ERROR_NONE;
}

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
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> capture(()\n" );
	
	spin_lock( &imgsensor_drv_lock );
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if( imgsensor.current_fps == imgsensor_info.cap1.max_framerate ) { //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk				= imgsensor_info.cap1.pclk;
		imgsensor.line_length		= imgsensor_info.cap1.linelength;
		imgsensor.frame_length		= imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length	= imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en	= KAL_FALSE;
	} else {
		if( imgsensor.current_fps != imgsensor_info.cap.max_framerate )
			printk( "Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n", imgsensor.current_fps, imgsensor_info.cap.max_framerate/10 );
		
		imgsensor.pclk				= imgsensor_info.cap.pclk;
		imgsensor.line_length		= imgsensor_info.cap.linelength;
		imgsensor.frame_length		= imgsensor_info.cap.framelength;
		imgsensor.min_frame_length	= imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en	= KAL_FALSE;
	}
	spin_unlock( &imgsensor_drv_lock );
	
	sensor_5M_setting(30);
///	capture_setting( imgsensor.current_fps );
//	set_mirror_flip( imgsensor.mirror );
	
	printk( "<< capture(()\n" );
	return ERROR_NONE;
}

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> normal_video()\n" );
	
	spin_lock( &imgsensor_drv_lock );
	imgsensor.sensor_mode		= IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk				= imgsensor_info.normal_video.pclk;
	imgsensor.line_length		= imgsensor_info.normal_video.linelength;
	imgsensor.frame_length		= imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length	= imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps		= 300;
	imgsensor.autoflicker_en	= KAL_FALSE;
	spin_unlock( &imgsensor_drv_lock );
	
	sensor_1M_setting();
///	normal_video_setting( imgsensor.current_fps );
//	set_mirror_flip( imgsensor.mirror );
	
	printk( "<< normal_video()\n" );
	return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> hs_video()\n" );
	
	spin_lock( &imgsensor_drv_lock );
	imgsensor.sensor_mode		= IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk				= imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode		= KAL_TRUE;
	imgsensor.line_length		= imgsensor_info.hs_video.linelength;
	imgsensor.frame_length		= imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length	= imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line		= 0;
	imgsensor.dummy_pixel		= 0;
	imgsensor.autoflicker_en	= KAL_FALSE;
	spin_unlock( &imgsensor_drv_lock );
	
	hs_video_setting();
//	set_mirror_flip( imgsensor.mirror );
	
	printk( "<< hs_video()\n" );
	return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> slim_video()\n" );
	
	spin_lock( &imgsensor_drv_lock );
	imgsensor.sensor_mode		= IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk				= imgsensor_info.slim_video.pclk;
	imgsensor.line_length		= imgsensor_info.slim_video.linelength;
	imgsensor.frame_length		= imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length	= imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line		= 0;
	imgsensor.dummy_pixel		= 0;
	imgsensor.autoflicker_en	= KAL_FALSE;
	spin_unlock( &imgsensor_drv_lock );
	
	slim_video_setting();
//	set_mirror_flip( imgsensor.mirror );
	
	printk( "<< slim_video()\n" );
	return ERROR_NONE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	printk( ">> get_resolution()\n" );
	
	sensor_resolution->SensorFullWidth				= imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight				= imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth			= imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight			= imgsensor_info.pre.grabwindow_height;
	
	sensor_resolution->SensorVideoWidth				= imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight			= imgsensor_info.normal_video.grabwindow_height;
	
	sensor_resolution->SensorHighSpeedVideoWidth	= imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	= imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth			= imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight		= imgsensor_info.slim_video.grabwindow_height;
	
	printk( "<< get_resolution()\n" );
	return ERROR_NONE;
}

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_INFO_STRUCT *sensor_info, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> get_info()\n" );
	printk( "scenario_id=%d\n", scenario_id );
	
	//sensor_info->SensorVideoFrameRate				= imgsensor_info.normal_video.max_framerate/10;	/* not use */
	//sensor_info->SensorStillCaptureFrameRate		= imgsensor_info.cap.max_framerate/10;			/* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate	= imgsensor_info.v.max_framerate;				/* not use */
	
	sensor_info->SensorClockPolarity		= SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity	= SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity		= SENSOR_CLOCK_POLARITY_LOW;	// inverse with datasheet
	sensor_info->SensorVsyncPolarity		= SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines	= 4;		/* not use */
	sensor_info->SensorResetActiveHigh		= FALSE;	/* not use */
	sensor_info->SensorResetDelayCount		= 5;		/* not use */
	
	sensor_info->SensroInterfaceType		= imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType				= imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode			= imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat		= imgsensor_info.sensor_output_dataformat;
	
	sensor_info->CaptureDelayFrame			= imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame			= imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame			= imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame	= imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame		= imgsensor_info.slim_video_delay_frame;
	
	sensor_info->SensorMasterClockSwitch	= 0;		/* not use */
	sensor_info->SensorDrivingCurrent		= imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame			= imgsensor_info.ae_shut_delay_frame;			/* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame		= imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame		= imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support				= imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine			= imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum				= imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber		= imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq			= imgsensor_info.mclk;
	sensor_info->SensorClockDividCount		= 3;		/* not use */
	sensor_info->SensorClockRisingCount		= 0;
	sensor_info->SensorClockFallingCount	= 2;		/* not use */
	sensor_info->SensorPixelClockCount		= 3;		/* not use */
	sensor_info->SensorDataLatchCount		= 2;		/* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount	= 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount	= 0;
	sensor_info->SensorWidthSampling		= 0;		// 0 is default 1x
	sensor_info->SensorHightSampling		= 0;		// 0 is default 1x
	sensor_info->SensorPacketECCOrder		= 1;
	
	switch( scenario_id )
	{
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
	
	printk( "<< get_info()\n" );
	return ERROR_NONE;
}

static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> control()\n" );
	printk( "scenario_id=%d\n", scenario_id );
	
	spin_lock( &imgsensor_drv_lock );
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock( &imgsensor_drv_lock );
	
	switch( scenario_id )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview( image_window, sensor_config_data );
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture( image_window, sensor_config_data );
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video( image_window, sensor_config_data );
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video( image_window, sensor_config_data );
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video( image_window, sensor_config_data );
			break;
		default:
			printk( "Error ScenarioId setting" );
			preview( image_window, sensor_config_data );
			return ERROR_INVALID_SCENARIO_ID;
	}
	
	printk( "<< control()\n" );
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(UINT16 framerate)
{	//This Function not used after ROME
	printk( ">> set_video_mode(): framerate=%d\n", framerate );
	printk( "framerate=%d\n", framerate );
	
	// SetVideoMode Function should fix framerate
	if( framerate == 0 )
		// Dynamic frame rate
		return ERROR_NONE;
	
	spin_lock( &imgsensor_drv_lock );
	if( (framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE) )
		imgsensor.current_fps = 296;
	else if( (framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE) )
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock( &imgsensor_drv_lock );
	
	set_max_framerate( imgsensor.current_fps, 1 );
	
	printk( "<< set_video_mode()\n" );
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	printk( ">> set_auto_flicker_mode(): enable=%d, framerate=%d\n", enable, framerate );
	printk( "enable=%d, framerate=%d \n", enable, framerate );
	
	spin_lock( &imgsensor_drv_lock );
	if( enable ) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock( &imgsensor_drv_lock );
	
	printk( "<< set_auto_flicker_mode()\n" );
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32	frame_length;
	
	printk( ">> set_max_framerate_by_scenario(): scenario_id=%d, framerate=%d\n", scenario_id, framerate );
	printk( "scenario_id=%d, framerate=%d\n", scenario_id, framerate );
	
	switch( scenario_id )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock( &imgsensor_drv_lock );
			imgsensor.dummy_line		= (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length		= imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length	= imgsensor.frame_length;
			spin_unlock( &imgsensor_drv_lock );
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if( framerate == 0 )
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock( &imgsensor_drv_lock );
			imgsensor.dummy_line		= (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length		= imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length	= imgsensor.frame_length;
			spin_unlock( &imgsensor_drv_lock );
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if( imgsensor.current_fps == imgsensor_info.cap1.max_framerate ) {
				frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
				spin_lock( &imgsensor_drv_lock );
				imgsensor.dummy_line		= (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
				imgsensor.frame_length		= imgsensor_info.cap1.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length	= imgsensor.frame_length;
				spin_unlock( &imgsensor_drv_lock );
			} else {
				if( imgsensor.current_fps != imgsensor_info.cap.max_framerate )
					printk( "Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n", framerate, imgsensor_info.cap.max_framerate/10 );
				frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
				spin_lock( &imgsensor_drv_lock );
				imgsensor.dummy_line		= (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
				imgsensor.frame_length		= imgsensor_info.cap.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length	= imgsensor.frame_length;
				spin_unlock( &imgsensor_drv_lock );
			}
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock( &imgsensor_drv_lock );
			imgsensor.dummy_line		= (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length		= imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length	= imgsensor.frame_length;
			spin_unlock( &imgsensor_drv_lock );
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock( &imgsensor_drv_lock );
			imgsensor.dummy_line		= (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength) : 0;
			imgsensor.frame_length		= imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length	= imgsensor.frame_length;
			spin_unlock( &imgsensor_drv_lock );
			//set_dummy();
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock( &imgsensor_drv_lock );
			imgsensor.dummy_line		= (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length		= imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length	= imgsensor.frame_length;
			spin_unlock( &imgsensor_drv_lock );
			//set_dummy();
			printk( "error scenario_id = %d, we use preview scenario \n", scenario_id );
			break;
	}
	
	printk( "<< set_max_framerate_by_scenario()\n" );
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	printk( ">> get_default_framerate_by_scenario()\n" );
	printk( "scenario_id = %d\n", scenario_id );
	
	switch( scenario_id )
	{
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
	
	printk( "<< get_default_framerate_by_scenario()\n" );
	return ERROR_NONE;
}

#ifdef __SENSOR_WINDSZIE_INFO__
static SENSOR_WINSIZE_INFO_STRUCT ImgsensorWinSizeInfo[5]=
{
	{
	   (2624),(1956),
		8,8,(2624),(1956),(1312),(978),
		0,0,(1312),(978),
		JX507MIPI_PV_X_START,JX507MIPI_PV_Y_START,JX507MIPI_IMAGE_SENSOR_PV_WIDTH,JX507MIPI_IMAGE_SENSOR_PV_HEIGHT
	},//Preview

	{
		(2624),(1956),
		8,8,(2624),(2455),(2624),(1956),
		0,0,(2624),(1956),
		JX507MIPI_FULL_X_START,JX507MIPI_FULL_Y_START,JX507MIPI_IMAGE_SENSOR_FULL_WIDTH,JX507MIPI_IMAGE_SENSOR_FULL_HEIGHT
	},//Capture
	{
		(2624),(1956),
		8,8,(3271),(2455),(1632),(1224),
		0,0,(1632),(1224),
		JX507MIPI_VIDEO_X_START,JX507MIPI_VIDEO_Y_START,JX507MIPI_IMAGE_SENSOR_VIDEO_WIDTH,JX507MIPI_IMAGE_SENSOR_VIDEO_HEIGHT
	},//Video
	
	{
		(2624),(1956),
		8,8,(3271),(2455),(1632),(1224),
		0,0,(1632),(1224),
		JX507MIPI_PV_X_START,JX507MIPI_PV_Y_START,JX507MIPI_IMAGE_SENSOR_PV_WIDTH,JX507MIPI_IMAGE_SENSOR_VIDEO_HEIGHT
	},//High Speed Video
	
	{
		(2624),(1956),
		8,8,(3271),(2455),(1632),(1224),
		0,0,(1632),(1224),
		JX507MIPI_PV_X_START,JX507MIPI_PV_Y_START,JX507MIPI_IMAGE_SENSOR_PV_WIDTH,JX507MIPI_IMAGE_SENSOR_PV_HEIGHT
	},//Slim Video

};

#ifdef __64BIT_ARCH__
UINT32 JX507MIPIGetCropInfo(unsigned long long *pFeaturePara)
#else
UINT32 JX507MIPIGetCropInfo(UINT8 *pFeaturePara)
#endif
{
	SENSOR_WINSIZE_INFO_STRUCT *WinInfo;
	UINT32 *pFeatureData32 = (UINT32 * )pFeaturePara;

	WinInfo=(SENSOR_WINSIZE_INFO_STRUCT *)(*(pFeatureData32+1));

	switch(*pFeatureData32){
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)WinInfo,(void *)&ImgsensorWinSizeInfo[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)WinInfo,(void *)&ImgsensorWinSizeInfo[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)WinInfo,(void *)&ImgsensorWinSizeInfo[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)WinInfo,(void *)&ImgsensorWinSizeInfo[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)WinInfo,(void *)&ImgsensorWinSizeInfo[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		default:
			memcpy((void *)WinInfo,(void *)&ImgsensorWinSizeInfo[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
	}
}

#endif

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id, UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16	= (UINT16 *) feature_para;
	UINT16 *feature_data_16			= (UINT16 *) feature_para;
	UINT32 *feature_return_para_32	= (UINT32 *) feature_para;
	UINT32 *feature_data_32			= (UINT32 *) feature_para;
	unsigned long long *feature_data		= (unsigned long long *) feature_para;
	unsigned long long *feature_return_para	= (unsigned long long *) feature_para;
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
	
	printk( "feature_id=%d\n", feature_id );
	switch( feature_id )
	{
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			printk( "feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps );
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			set_shutter( *feature_data );
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			night_mode( (BOOL) *feature_data );
			break;
		case SENSOR_FEATURE_SET_GAIN:
			set_gain( (UINT16) *feature_data );
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor( sensor_reg_data->RegAddr, sensor_reg_data->RegData );
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor( sensor_reg_data->RegAddr );
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			set_video_mode( *feature_data );
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id( feature_return_para_32 );
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode( (BOOL)*feature_data_16,*( feature_data_16+1) );
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			set_max_framerate_by_scenario( (MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1) );
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			get_default_framerate_by_scenario( (MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)( uintptr_t)( *(feature_data+1)) );
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			set_test_pattern_mode( (BOOL)*feature_data );
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
			//printk( "current fps :%d\n", *feature_data );
			spin_lock( &imgsensor_drv_lock );
			imgsensor.current_fps = *feature_data;
			spin_unlock( &imgsensor_drv_lock );
			break;
		case SENSOR_FEATURE_SET_HDR:
			//printk( "ihdr enable :%d\n", (BOOL)*feature_data );
			spin_lock( &imgsensor_drv_lock );
			imgsensor.ihdr_en = (BOOL)*feature_data;
			spin_unlock( &imgsensor_drv_lock );
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
			#if 0
#ifdef __SENSOR_WINDSZIE_INFO__
#ifdef __64BIT_ARCH__
			JX507MIPIGetCropInfo(feature_data);
#else	
			JX507MIPIGetCropInfo(feature_data);
#endif
			break;
#endif
#else
			wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)( *(feature_data+1) );
			
			switch( *feature_data_32 )
			{
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy( (void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof( SENSOR_WINSIZE_INFO_STRUCT) );
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy( (void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof( SENSOR_WINSIZE_INFO_STRUCT) );
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy( (void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof( SENSOR_WINSIZE_INFO_STRUCT) );
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy( (void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof( SENSOR_WINSIZE_INFO_STRUCT) );
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy( (void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof( SENSOR_WINSIZE_INFO_STRUCT) );
					break;
			}
                   #endif
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
			//printk( "SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16)*feature_data, (UINT16)*( feature_data+1), (UINT16)*( feature_data+2) );
			ihdr_write_shutter_gain( (UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2) );
			break;
		default:
			break;
	}
	
	return ERROR_NONE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 JX507_MIPI_RAW_SensorInit( PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if( pfFunc != NULL )
		*pfFunc = &sensor_func;
	
	return ERROR_NONE;
}    /*    JX507_MIPI_RAW_SensorInit    */
