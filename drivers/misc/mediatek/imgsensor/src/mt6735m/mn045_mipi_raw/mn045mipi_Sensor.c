/*****************************************************************************
 *
 * Filename:
 * ---------
 *     MN045mipi_Sensor.c
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

#include "mn045mipi_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "MN045_camera_sensor"
#define LOG_1 LOG_INF("MN045,MIPI 2LANE\n")
#define LOG_2 LOG_INF("preview 1280*960@30fps,420Mbps/lane; video 1280*960@30fps,420Mbps/lane; capture 5M@15fps,420Mbps/lane\n")
/****************************   Modify end    *******************************************/

//#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF(fmt,arg...)    printk("<<MN045>>[%s:%d] "fmt"\n", __func__, __LINE__, ##arg)

static DEFINE_SPINLOCK(imgsensor_drv_lock);
kal_uint8 preview_pre = 0;

static imgsensor_info_struct imgsensor_info = {
    .sensor_id = MN045MIPI_SENSOR_ID,

    .checksum_value = 0xf7375923,        //checksum value for Camera Auto Test

    .pre = {
        /*.pclk = 86400000,            //record different mode's pclk
        .linelength = 2784,            //record different mode's linelength
        .framelength = 1016,            //record different mode's framelength
        .startx = 2,                    //record different mode's startx of grabwindow
        .starty = 2,                    //record different mode's starty of grabwindow
        .grabwindow_width = 1280,        //record different mode's width of grabwindow
        .grabwindow_height = 960,        //record different mode's height of grabwindow*/
        
        .pclk = 86400000,            //record different mode's pclk
        .linelength = 2784,
        .framelength = 2030,
        .startx = 2,
        .starty = 6,
        .grabwindow_width = 2560,
        .grabwindow_height = 1920,
        
        /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        /*     following for GetDefaultFramerateByScenario()    */
        .max_framerate = 300,
    },
    .cap = {
        .pclk = 86400000,
        .linelength = 2784,
        .framelength = 2030,
        .startx = 2,
        .starty = 6,
        .grabwindow_width = 2560,
        .grabwindow_height = 1920,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 150,
    },
    .cap1 = {
        .pclk = 86400000,
        .linelength = 2784,
        .framelength = 2030,
        .startx = 2,
        .starty = 6,
        .grabwindow_width = 2560,
        .grabwindow_height = 1920,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 150,
    },
    .normal_video = {
        .pclk = 86400000,
        .linelength = 2784,
        .framelength = 1016,
        .startx = 2,
        .starty = 2,
        .grabwindow_width = 1280,
        .grabwindow_height = 960,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 84000000,
        .linelength = 2500,
        .framelength = 1120,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1920,
        .grabwindow_height = 1080,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .slim_video = {
        .pclk = 84000000,
        .linelength = 3728,
        .framelength = 748,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
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

    .cap_delay_frame = 2,
    .pre_delay_frame = 2,
    .video_delay_frame = 2,
    .hs_video_delay_frame = 2,
    .slim_video_delay_frame = 2,

    .isp_driving_current = ISP_DRIVING_2MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_2_LANE,//mipi lane num
    .i2c_addr_table = {0x34, 0xff},//YZH
    .i2c_speed = 100,
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,                //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                    //current shutter
    .gain = 0x100,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x34,//record current sensor's i2c write id
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5]=
{//{ 2624, 1956,      0,    0, 2624, 1956, 1312,  978,      8,   4, 1296,  972,  4,    8, 1280,  960},  // Preview
	{ 2624, 1956,      0,    0, 2624, 1956, 2624, 1956,     16,   6, 2592, 1944,  2,    2, 2560,  1920},   // Preview
 { 2624, 1956,      0,    0, 2624, 1956, 2624, 1956,     16,   6, 2592, 1944,  2,    2, 2560,  1920}, // capture
 { 2624, 1956,      0,    0, 2624, 1956, 1312,  978,      8,   4, 1296,  972,  4,    8, 1280,  960},  // video
 { 2624, 1956,    336,  434, 1952, 1088, 1952, 1088,     16,   4, 1920, 1080,  0,    0, 1920,  1080}, //hight speed video
 { 2624, 1956,     16,  254, 2592, 1448, 1296,  724,      8,   2, 1280,  720,  0,    0, 1280,  720}}; // slim video


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	//kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor

    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

    return get_byte;
}

static kal_uint16 read_cmos_sensor_2bit(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	//kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor

    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);

    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	//kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor

    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_16(kal_uint16 addr, kal_uint16 para)
{
    //kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}
#define MaxGainIndex (82)
kal_uint16 sensorGainMapping[MaxGainIndex][2] ={	
	{64 ,0	},
	{66 ,1	},
	{68 ,2  },
	{70 ,3  },
	{73 ,4  },
	{76 ,5 },
	{78 ,6 },
	{81 ,7 },
	{84 ,8 },
	{87 ,9 },
	{90 ,10 },
	{93 ,11 },
	{96 ,12 },
	{100 ,13 },
	{103 ,14 },
	{107 ,15 },
	{111 ,16},
	{115 ,17},
	{119 ,18},
	{123 ,19},
	{127 ,20},
	{132 ,21},
	{136 ,22},
	{141 ,23},
	{146 ,24},
	{151 ,25},
	{157 ,26},
	{162 ,27},
	{168 ,28},
	{174,29},
	{180,30},
	{186,31},
	{193,32},
	{200,33},
	{207,34},
	{214,35},
	{221,36},
	{229,37},
	{237,38},
	{246,39},
	{254,40},
	{263,41},
	{273,42},
	{282,43},
	{292,44},
	{302,45},
	{313,46},
	{324,47},
	{335,48},
	{347,49},
	{359,50},
	{372,51},
	{385,52},
	{399,53},
	{413,54},
	{427,55},
	{442,56},
	{458,57},
	{474,58},
	{491,59},
	{508,60},
	{526,61},
	{544,62},
	{563,63},
	{583,64},
	{604,65},
	{625,66},
	{647,67},
	{670,68},
	{693,69},
	{718,70},
	{743,71},
	{769,72},
	{796,73},
	{824,74},
	{853,75},
	{883,76},
	{914,77},
	{946,78},
	{979,79},
	{1014,80},
	{1049,81},
};

static void set_dummy(void)
{
#if 1
	kal_uint32  framelength = 0 , linelength = 0;
	

    if((imgsensor.sensor_mode == IMGSENSOR_MODE_PREVIEW)||(imgsensor.sensor_mode == IMGSENSOR_MODE_VIDEO))
	{
		framelength = imgsensor.frame_length - 1016 ;
		//linelength = imgsensor.line_length - 2784;
		write_cmos_sensor_16(0x018E, framelength);
		//write_cmos_sensor_16(0x6AA0, linelength);
		//framelength = read_cmos_sensor(0x018E);
		//linelength = read_cmos_sensor(0x6AA0);
		LOG_INF("isx012 PREVIEW or VIDEO 0x6A92 = %x, 0x6AA0 = %x\n", framelength, linelength);	
	}
	else if(imgsensor.sensor_mode == IMGSENSOR_MODE_CAPTURE)
	{
		framelength = imgsensor.frame_length - 2030 ;
		//linelength = imgsensor.line_length - 2784;
		write_cmos_sensor_16(0x018C, framelength);
		//write_cmos_sensor_16(0x6A9E, linelength);
		LOG_INF("isx012 CAPTURE 0x6A90 = %x, 0x6A9E = %x\n", framelength, linelength);
	}
#endif	
}    /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
  //  return ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
    return (read_cmos_sensor_2bit(0x0000)) ;
}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    kal_uint32 frame_length = imgsensor.frame_length;

    LOG_INF("isx012 framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);

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
}

static void write_shutter(kal_uint16 shutter)
{
    kal_uint16 realtime_fps = 0, reg0146 = 0;

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
    } 
	
	reg0146 = read_cmos_sensor(0x0146);
	LOG_INF("isx012 reg0146 =%x\n", reg0146);
	
	write_cmos_sensor(0x0146, reg0146 | 0x01);
    write_cmos_sensor_16(0x0148, shutter);
	write_cmos_sensor(0x0146, reg0146 & 0xfe);
    LOG_INF("isx012 shutter =%d\n", shutter);
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
    unsigned long flags;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 iI;

    for (iI = 0; iI < MaxGainIndex; iI++) 
	{
		if(gain < sensorGainMapping[iI][0])
		{                
			return sensorGainMapping[iI][1];       
		}
			

    }
	if(iI != MaxGainIndex)
	{
    	if(gain != sensorGainMapping[iI][0])
    	{
        	 LOG_INF("Gain mapping don't correctly:%d %d \n", gain, sensorGainMapping[iI][0]);
    	}
	}
	LOG_INF("exit IMX230MIPIGain2Reg function\n");
    return sensorGainMapping[iI-1][1];
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
    kal_uint16 reg_gain=0, reg0146=0;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X    */
    /* [4:9] = M meams M X         */
    /* Total gain = M + N /16 X   */

    //
    if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > 16 * BASEGAIN)
            gain = 16 * BASEGAIN;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	
	reg0146 = read_cmos_sensor(0x0146);
	LOG_INF("isx012 reg0146 =%x\n", reg0146);
	
	write_cmos_sensor(0x0146, reg0146 | 0x01);
    write_cmos_sensor(0x014A, reg_gain);
	write_cmos_sensor(0x0146, reg0146 & 0xfe);
    return gain;
}    /*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    //not support HDR
    //LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
}

#if 1
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
            write_cmos_sensor(0x008C,0x00);
            write_cmos_sensor(0x008D,0x00);
	     write_cmos_sensor(0x008E,0x00);
            break;
        case IMAGE_H_MIRROR:
            write_cmos_sensor(0x008C,0x01);
            write_cmos_sensor(0x008D,0x01);
	     write_cmos_sensor(0x008E,0x01);
            break;
        case IMAGE_V_MIRROR:
            write_cmos_sensor(0x008C,0x02);
            write_cmos_sensor(0x008D,0x02);
	     write_cmos_sensor(0x008E,0x02);
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor(0x008C,0x03);
            write_cmos_sensor(0x008D,0x03);
	     write_cmos_sensor(0x008E,0x03);
            break;
        default:
            LOG_INF("Error image_mirror setting\n");
    }

}
#endif

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
spin_lock(&imgsensor_drv_lock);
imgsensor.i2c_write_id =0x34;
spin_unlock(&imgsensor_drv_lock);

write_cmos_sensor(0x0004,0x02);  
write_cmos_sensor(0x0007,0x00);                 
write_cmos_sensor(0x0008,0x00);                
write_cmos_sensor(0x00C4,0x11);                
write_cmos_sensor(0x00C5,0x11);                
write_cmos_sensor(0x00C6,0x11);                
write_cmos_sensor(0x00C7,0x11);                
write_cmos_sensor(0x00C8,0x11);                
write_cmos_sensor(0x00C9,0x11);                
write_cmos_sensor(0x00CA,0x11);                
write_cmos_sensor(0x00CC,0x11);                
write_cmos_sensor(0x00CD,0x11);                
write_cmos_sensor(0x6A12,0x50);                
write_cmos_sensor(0x6A13,0x50);                
write_cmos_sensor(0x6A14,0x50);                
write_cmos_sensor(0x6A15,0x50);                
write_cmos_sensor_16(0x018C,0x0000);           
write_cmos_sensor_16(0x018E,0x0000);           
write_cmos_sensor_16(0x0190,0x0000);           
write_cmos_sensor_16(0x0192,0x0000);           
write_cmos_sensor_16(0x0194,0x0027);           
write_cmos_sensor_16(0x0196,0x0015);           
write_cmos_sensor_16(0x6A16,0x0440);           
write_cmos_sensor_16(0x6A18,0x03C0);           
write_cmos_sensor_16(0x6A1A,0x01E0);           
write_cmos_sensor_16(0x6A1C,0x00E0);           
write_cmos_sensor_16(0x6A1E,0x0420);           
write_cmos_sensor_16(0x6A20,0x02C0);           
write_cmos_sensor_16(0x0016,0x0010);           
write_cmos_sensor(0x5C01,0x00);                
write_cmos_sensor_16(0x0148,0x0300);           
write_cmos_sensor(0x0145,0x01);                
write_cmos_sensor(0x5C04,0x04);                
write_cmos_sensor(0x5C05,0x03);                
write_cmos_sensor(0x5C06,0x0E);                
write_cmos_sensor(0x5C07,0x02);                
write_cmos_sensor(0x5C08,0x0B);                
write_cmos_sensor(0x5C09,0x05);                
write_cmos_sensor(0x5C0A,0x07);                
write_cmos_sensor(0x5C0B,0x03);                
write_cmos_sensor(0x5C0C,0x07);                
write_cmos_sensor(0x5C0D,0x05);                
write_cmos_sensor(0x5C0E,0x01);    

// Moni H1/2V1/2, Cap H1/1V1/1, Movie HD1080 H1
 write_cmos_sensor(0x00AD, 0x10); 
 write_cmos_sensor(0x00AC, 0x04); 
 write_cmos_sensor(0x0009, 0x01); 
 write_cmos_sensor(0x000a, 0x01);//add 
 write_cmos_sensor(0x00D0, 0x30);  
 write_cmos_sensor(0x00D1, 0x30);  
 write_cmos_sensor(0x00D2, 0x30); 
 write_cmos_sensor(0x00D3, 0x30); 
 write_cmos_sensor(0x00D4, 0x30);  
 write_cmos_sensor(0x00D5, 0x30);  
 write_cmos_sensor(0x00D6, 0x30);  
 write_cmos_sensor(0x00D7, 0x30); 
 write_cmos_sensor(0x00D8, 0x50);  
 write_cmos_sensor(0x00D9, 0x50);  
 write_cmos_sensor(0x00DA, 0x30);  
 write_cmos_sensor(0x00DB, 0x30); 
 write_cmos_sensor(0x0083, 0x01); 
 write_cmos_sensor(0x0084, 0x00); 
 write_cmos_sensor(0x0085, 0x00); 
 write_cmos_sensor(0x0086, 0x02); 
 write_cmos_sensor(0x0087, 0x03); 
 write_cmos_sensor(0x0088, 0x02); 
 write_cmos_sensor(0x0089, 0x0F); 
 write_cmos_sensor(0x008A, 0x0F); 
 write_cmos_sensor(0x008B, 0x0F); 
 write_cmos_sensor(0x00AF, 0x01); 
 write_cmos_sensor_16(0x0090, 0x0518); 
 write_cmos_sensor_16(0x0092, 0x0A30); 
 write_cmos_sensor_16(0x0094, 0x0A30); 
 write_cmos_sensor_16(0x0096, 0x03D4); 
 write_cmos_sensor_16(0x0098, 0x07A8); 
 write_cmos_sensor_16(0x009A, 0x0448); 
 write_cmos_sensor(0x0006, 0x16);

 spin_lock(&imgsensor_drv_lock);
 imgsensor.i2c_write_id =0x78;
 spin_unlock(&imgsensor_drv_lock);
		
//	mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_MODE_00); // GPIO_CAMERA_CMRST1_PIN
//	mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT);
//	mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO); //  GPIO_OUT_ZERO GPIO_OUT_ONE
//mdelay(50);
//	mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_MODE_00); // GPIO_CAMERA_CMRST1_PIN
//	mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT);
//	mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ONE); //  GPIO_OUT_ZERO GPIO_OUT_ONE
 mdelay(500);
 write_cmos_sensor(0x0140, 0x01);
 //PWDN HIGH
// mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ONE); //  GPIO_OUT_ZERO GPIO_OUT_ONE
}    /*    MIPI_sensor_Init  */


static void preview_setting(void)
{
/* write_cmos_sensor(0x0081, 0x00); 
 write_cmos_sensor(0x0085, 0x00); 
 write_cmos_sensor(0x0088, 0x02); 
 write_cmos_sensor(0x008B, 0x0F); 
 write_cmos_sensor(0x00AF, 0x01); 
 write_cmos_sensor_16(0x0094, 0x0A30);
 write_cmos_sensor_16(0x009A, 0x0448); 
 write_cmos_sensor(0x000a, 0x01);*/
  write_cmos_sensor(0x0081, 0x02);
} 


static void capture_setting(void)
{
 write_cmos_sensor(0x0081, 0x02);
}    /*    capture_setting  */

static void normal_video_setting(kal_uint16 currefps)
{
	
}

static void video_1080p_setting(void)
{

}

static void video_720p_setting(void)
{
	
}

static void hs_video_setting(void)
{
    LOG_INF("E\n");

    video_1080p_setting();
}

static void slim_video_setting(void)
{
    LOG_INF("E\n");

    video_720p_setting();
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
            *sensor_id =  return_sensor_id();// return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, write id:0x%x id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF; // 0xFFFFFFFF
        return ERROR_SENSOR_CONNECT_FAIL;//   
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
            sensor_id =  return_sensor_id();// return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id:0x%x id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
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
	preview_pre = 0;
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
	if((imgsensor.sensor_mode == IMGSENSOR_MODE_PREVIEW)||(imgsensor.sensor_mode == IMGSENSOR_MODE_VIDEO));
	else
	{
		if( preview_pre == 0)
		{
			capture_setting();
			mdelay(500);
			spin_lock(&imgsensor_drv_lock);
			preview_pre = 1;
			spin_unlock(&imgsensor_drv_lock);
		}
		 preview_setting();
	}
	
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	
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
            LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);
    capture_setting();
	if( preview_pre == 0)
	{
		spin_lock(&imgsensor_drv_lock);
		preview_pre = 1;
		spin_unlock(&imgsensor_drv_lock);
	}
    //set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
	if((imgsensor.sensor_mode == IMGSENSOR_MODE_PREVIEW)||(imgsensor.sensor_mode == IMGSENSOR_MODE_VIDEO));
	else
	{
		if( preview_pre == 0)
		{
			capture_setting();
			mdelay(500);
			spin_lock(&imgsensor_drv_lock);
			preview_pre = 1;
			spin_unlock(&imgsensor_drv_lock);
		}
		 preview_setting();
	}
   
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    //set_mirror_flip(sensor_config_data->SensorImageMirror);
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
// return;
    // 0x503D[8]: 1 enable,  0 disable
    // 0x503D[1:0]; 00 Color bar, 01 Random Data, 10 Square
    if(enable){
	 write_cmos_sensor(0x5020, 0x01);	
        write_cmos_sensor(0x5023, 0x04);
	 write_cmos_sensor(0x5022, 0x02);
    	}		
    else {
		
       write_cmos_sensor(0x5020, 0x01);	
        write_cmos_sensor(0x5023, 0x00);
	 write_cmos_sensor(0x5022, 0x00);
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

    //printk("feature_id = %d\n", feature_id);
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
            break;
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

UINT32 MN045MIPISensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    /*    MN045MIPISensorInit    */
