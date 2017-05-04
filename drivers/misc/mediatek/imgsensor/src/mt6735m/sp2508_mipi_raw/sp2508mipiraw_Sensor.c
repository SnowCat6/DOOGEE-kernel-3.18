/*****************************************************************************
 *
 * Filename:
 * ---------
 *     SP2508mipi_Sensor.c
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
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "sp2508mipiraw_Sensor.h"

#ifdef SLT_DEVINFO_CMM_SP2508 	
#include  <linux/dev_info.h> 
static struct devinfo_struct *s_DEVINFO_ccm;   //suppose 10 max lcm device 
#endif

/****************************Modify Following Strings for Debug****************************/
#define PFX "SP2508_camera_sensor"

#define LOG_1 LOG_INF("SP2508,MIPI 1LANE\n")
#define LOG_2 LOG_INF("preview 1600*1200@30fps,864Mbps/lane; video 1600*1200@30fps,864Mbps/lane; capture 2M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)


/*****************************************************************************/
//	T CARD DEBUG
//	debug by sp_zze 2015-06-03
/*****************************************************************************/
//#define DEBUG_SENSOR_SP2508		//for T-card
#ifdef DEBUG_SENSOR_SP2508	//sp_zze
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>

#define SP2508_SD_FILE_PATH					"/mnt/sdcard/SP2508_sd"
#define	SP_I2C_WRITE						write_cmos_sensor
#define	SP_I2C_READ							read_cmos_sensor
#define SP_LOG								LOG_INF
#define SP_DELAY							msleep
#define U32									kal_uint32
#define U16									kal_uint16
#define U8									kal_uint8
#define DATA_BUFF_SIZE						(25*1024)//25k, must be larger than size of sd file

typedef struct
{
	U16 addr;
	U16 val;
} SP2508_init_regs_struct;

static kal_uint16 read_cmos_sensor(kal_uint32 addr);
static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para);

static U8 sp_tolower(U8 c)
{
	if (c <= 'Z' && c >= 'A')
		return (c + 'a' - 'A');
	
	return c;
}
#define BASE_MAX	36
static const char digits[] = {"0123456789abcdefghijklmnopqrstuvwxyz"};
static U32 sp_strtoul(const char *str, char **endptr, U8 base)	//sp_zhangzhaoen
{
	const char *sc;
	const char *s1, *s2;
	U32 x = 0;
	
	for (sc = str; ; sc++) {	// skip space
		if (!(*sc == ' ' || *sc == '\t'))
			break;
	}

	if (base == 1 || base > BASE_MAX) {
		if (endptr)
			*endptr = (char *)str;		
		return 0;
	} else if (base) {
		if (base == 16 && *sc == '0' 
			&& (sc[1] == 'x' || sc[1] == 'X'))
			sc += 2;
	} else if (*sc != '0') {
		base = 10;
	} else if (sc[1] == 'x' || sc[1] == 'X') {
		base = 16;
		sc += 2;
	} else {
		base = 8;
	}
	
	for (s1 = sc; *sc == '0'; ++sc)
		;
	
	for (; ; ++sc) {
		s2 = memchr(digits, sp_tolower(*sc), base);
		if (s2 == NULL)
			break;

		x = x * base + (s2 - digits);
	}
	if (s1 == sc) {
		if (endptr)
			*endptr = (char *)str;		
		return 0;
	}

	// DEBUG: when overflow
	
	if (endptr)
		*endptr = (char *)sc;
	
	return (x);
}
static U32 SP2508_Initialize_from_T_Flash(char *file_path)
{
	U8 *curr_ptr = NULL;
	U8 *tmp = NULL;
	U32 file_size = 0;
	U32 regs_total = 0;
	U8 func_ind[4] = {0};	/* REG or DLY */
	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static U8 data_buff[DATA_BUFF_SIZE];
	SP2508_init_regs_struct SP2508_reg;
	
	fp = filp_open(file_path, O_RDONLY , 0); 
	if (IS_ERR(fp)) { 
		SP_LOG("create file error\n"); 
		return 0; 
	} 
	fs = get_fs(); 
	set_fs(KERNEL_DS); 
	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	filp_close(fp, NULL); 
	set_fs(fs);
	
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size)) {
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;		
			
		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*')) {
			curr_ptr += 2;	//skip '/' and '*'	added by sp_zze
			
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/'))) {
				curr_ptr += 2;		/* Skip block comment code. */
			}
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A))) {
				curr_ptr++;
				
				if (curr_ptr >= (data_buff + file_size))
					break;
			}
			curr_ptr += 2;						/* Skip the enter line */
			continue ;
		}
		
		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '/'))	{	/* Comment line, skip it. */
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A))) {
				curr_ptr++;
				
				if (curr_ptr >= (data_buff + file_size))
					break;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}

		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A)) {
			curr_ptr += 2;
			continue ;
		}
		
		memcpy(func_ind, curr_ptr, 3);		
		if (!strcmp((const char *)func_ind, "REG")) {		/* REG */
			curr_ptr += 4;				/* Skip "REG{" */
			SP2508_reg.addr = sp_strtoul((const char *)curr_ptr, &tmp, 16);
			curr_ptr = tmp;
			curr_ptr ++;	/* Skip "," */
			SP2508_reg.val = sp_strtoul((const char *)curr_ptr, &tmp, 16);
			curr_ptr += 2;	/* Skip "};" */
			
			//init
			SP_I2C_WRITE(SP2508_reg.addr, SP2508_reg.val);
			regs_total++;
		} else if (!strcmp((const char *)func_ind, "DLY")) {	/* DLY */
			curr_ptr += 4;				/* Skip "DLY(" */
			SP2508_reg.val = sp_strtoul((const char *)curr_ptr, &tmp, 10);
			curr_ptr = tmp;

			SP_DELAY(SP2508_reg.val);
		}

		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A))) {
			curr_ptr++;
			
			if (curr_ptr >= (data_buff + file_size))
				break;
		}
		curr_ptr += 2;
	}

	return regs_total;
}
static U8 read_from_extern_flash(char *file_path)
{
	struct file *fp; 

	fp = filp_open(file_path, O_RDONLY , 0); 
	if (IS_ERR(fp)) {   
		SP_LOG("open file error\n");
		return 0;
	} else {
		SP_LOG("open file ok\n");
		filp_close(fp, NULL); 
	}
	
	return 1;
}

#define SP2508_OTHER_DEBUG_PATH					"/mnt/sdcard/SP2508_debug"
static imgsensor_info_struct imgsensor_info;
static void SP2508_other_debug(char *file_path)	//sp_zze
{
	U8 *curr_ptr = NULL;
	U8 *tmp = NULL;
	U32 file_size = 0;
	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static U8 data_buff[1024];
	
	fp = filp_open(file_path, O_RDONLY , 0); 
	if (IS_ERR(fp)) { 
		SP_LOG("create file error\n"); 
		return; 
	} 
	fs = get_fs(); 
	set_fs(KERNEL_DS); 
	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	filp_close(fp, NULL); 
	set_fs(fs);
	
	curr_ptr = data_buff;
	
	imgsensor_info.ae_shut_delay_frame = sp_strtoul((const char *)curr_ptr, &tmp, 10);
	curr_ptr = tmp;
	curr_ptr ++;	/* Skip "," */
	imgsensor_info.ae_sensor_gain_delay_frame = sp_strtoul((const char *)curr_ptr, &tmp, 10);
	curr_ptr = tmp;
	curr_ptr ++;	/* Skip "," */
	imgsensor_info.ae_ispGain_delay_frame = sp_strtoul((const char *)curr_ptr, &tmp, 10);
	//sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
    //sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
    //sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	
	return;
}
#endif	//END of T CARD DEBUG

static DEFINE_SPINLOCK(imgsensor_drv_lock);


static imgsensor_info_struct imgsensor_info = { 
    .sensor_id = SP2508_SENSOR_ID,
    .checksum_value = 0xffd8e112,        //checksum value for Camera Auto Test

    .pre = {
        .pclk = 30000000,              //record different mode's pclk //21
        .linelength = 1160,             //record different mode's linelength
        .framelength = 1241,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
		.grabwindow_width = 1600,		//record different mode's width of grabwindow
		.grabwindow_height = 1200,		//record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 85,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,   
    },
    .cap = {
        .pclk = 30000000,
        .linelength = 1160,
        .framelength = 1241,
        .startx = 0,
        .starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .cap1 = {
        .pclk = 30000000,
        .linelength = 1160,
        .framelength = 1241,
        .startx = 0,
        .starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,   
    },
    .normal_video = {
        .pclk = 30000000,
        .linelength = 1160,
		.framelength = 1241,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 30000000,
        .linelength = 1160,
        .framelength = 1241,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1600,
        .grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 600,
    },
    .slim_video = {
        .pclk = 30000000,
        .linelength = 1160,
        .framelength = 1241,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1600,
        .grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .margin = 4,            //sensor framelength & shutter margin
    .min_shutter = 7,
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
    .ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 10,      //support sensor mode num

    .cap_delay_frame = 3,        //enter capture delay frame num
    .pre_delay_frame = 3,         //enter preview delay frame num
    .video_delay_frame = 5,        //enter video delay frame num
    .hs_video_delay_frame = 5,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 5,//enter slim video delay frame num

    .isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_1_LANE,//mipi lane num
    .i2c_addr_table = {0x78, 0x7a, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,             //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x46e,                   //current shutter   // Danbo ??
    .gain = 0x40,                      //current gain     // Danbo ??
    .dummy_pixel = 272,                   //current dummypixel
    .dummy_line = 32,                    //current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,      //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x78,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
 { 1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, // Preview 2112*1558
 { 1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, // capture 4206*3128
 { 1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, // video 
 { 1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, //hight speed video 
 { 1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}};// slim video



static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[1] = {(char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 1, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;

}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
		char pu_send_cmd[2] = {(char)(addr & 0xFF), (char)(para & 0xFF)};
		iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{   
}    /*    set_dummy  */


static kal_uint32 return_sensor_id(void)
{
    write_cmos_sensor(0xfd, 0x00);
	return ((read_cmos_sensor(0x02) << 8) | read_cmos_sensor(0x03));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;

    LOG_INF("framerate = %d, min framelength= %d should enable? \n", framerate,min_framelength_en);

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
}    /*    set_max_framerate  */



static void write_shutter(kal_uint16 shutter)
{  
        write_cmos_sensor(0xfd, 0x01); 
        write_cmos_sensor(0x03, (shutter >> 8) & 0xFF);
        write_cmos_sensor(0x04, shutter  & 0xFF); 
        write_cmos_sensor(0x01, 0x01); 
        LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}   /*  write_shutter  */



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
    if (shutter < 7)
          shutter = 7; 
	else if(shutter > 0xffff)
	   shutter = 0xffff;
	
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
    
	write_shutter(shutter);

}    /*    set_shutter */




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
         kal_uint8  iReg;

		if(gain >= 0x10 && gain <= 15*0x10)
		{   
			 iReg = 0x10 * gain/0x10;        //change mtk gain base to aptina gain base
			 //iReg += (gain%BASEGAIN)/(0x10/BASEGAIN);
						   	
   	    	 if(iReg<=0x10)
   	    	 {
   	    	    	 write_cmos_sensor(0xfd, 0x01);
   	    	    	 write_cmos_sensor(0x24, 0x10);//0x23
   	    	    	 write_cmos_sensor(0x01, 0x01);
                      LOG_INF("SP2508MIPI_SetGain = 16");
   	    	 }
   	    	 else if(iReg>= 0xa0)//gpw
   	    	 {
   	    	    	 write_cmos_sensor(0xfd, 0x01);
   	    	    	 write_cmos_sensor(0x24,0xa0);
   	    	    	 write_cmos_sensor(0x01, 0x01); 
                      LOG_INF("SP2508MIPI_SetGain = 160"); 
	        }        	
   	    	 else
   	    	 {
   	    	    	write_cmos_sensor(0xfd, 0x01);
   	    	    	write_cmos_sensor(0x24, (kal_uint8)iReg);
   	    	    	write_cmos_sensor(0x01, 0x01);
			LOG_INF("SP2508MIPI_SetGain = %d",iReg);		 
	       }	
   	 }	
   	 else
   	    	 LOG_INF("error gain setting");	 
	 
        return gain;
}    /*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
}

static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);
       write_cmos_sensor(0xfd,0x01);
	switch (image_mirror)
	{
		case IMAGE_NORMAL://IMAGE_NORMAL:			
         		write_cmos_sensor(0x3f,0x00);
			break;
		case IMAGE_H_MIRROR://IMAGE_H_MIRROR:
         		write_cmos_sensor(0x3f,0x01);
			break;
		case IMAGE_V_MIRROR://IMAGE_V_MIRROR:
         		write_cmos_sensor(0x3f,0x02);
			break;
		case IMAGE_HV_MIRROR://IMAGE_HV_MIRROR:
         		write_cmos_sensor(0x3f,0x03);
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
	LOG_INF("Enter sensor_init.\n");
	//update by xingdy, 20150309
	//pll clk 60M
	//p1:0x2b=0xc4, p1:0x58=0x30, pixel bias 2u
	//add offset h'20 and  remove the digital gain
	//T_horizontal=1160, base is h'103, 21fps
	//add digital gain 1.3x 解决高亮发粉问题
	{
	write_cmos_sensor(0xfd,0x00);
	write_cmos_sensor(0x1c,0x03);
	write_cmos_sensor(0x35,0x20); //pll bias
	write_cmos_sensor(0x2f,0x08); //pll clk 60M            
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0x03,0x03); //exp time, 3 base
	write_cmos_sensor(0x04,0x09);
	write_cmos_sensor(0x06,0x10); //vblank
	write_cmos_sensor(0x24,0xa0); //pga gain 10x
	write_cmos_sensor(0x01,0x01); //enable reg write
	write_cmos_sensor(0x2b,0xc4); //readout vref
	write_cmos_sensor(0x2e,0x20); //dclk delay
	write_cmos_sensor(0x79,0x42); //p39 p40
	write_cmos_sensor(0x85,0x0f); //p51
	write_cmos_sensor(0x09,0x01); //hblank
	write_cmos_sensor(0x0a,0x40);
	write_cmos_sensor(0x21,0xef); //pcp tx 4.05v
	write_cmos_sensor(0x25,0xf0); //reg dac 2.7v, enable bl_en,vbl 1.28v 原为0xf2 改为0xf0 降低太阳黑子替换电压
	write_cmos_sensor(0x26,0x00); //vref2 1v, disable ramp driver
	write_cmos_sensor(0x2a,0xea); //bypass dac res, adc range 0.745, vreg counter 0.9
	write_cmos_sensor(0x2c,0xf0); //high 8bit, pldo 2.7v
	write_cmos_sensor(0x8a,0x44); //pixel bias 1.58uA 原为0x55 改为0x44 降低pixel bias
	write_cmos_sensor(0x8b,0x44); //原为0x55 改为0x44 降低pixel bias
	write_cmos_sensor(0x19,0xf3); //icom1 1.7u, icom2 0.6u 
	write_cmos_sensor(0x11,0x30); //rst num
	write_cmos_sensor(0xd0,0x01); //disable boost
	write_cmos_sensor(0xd1,0x01);
	write_cmos_sensor(0xd2,0xd0);
	write_cmos_sensor(0x55,0x10);
	write_cmos_sensor(0x58,0x30);
	write_cmos_sensor(0x5d,0x15);
	write_cmos_sensor(0x5e,0x05);
	write_cmos_sensor(0x64,0x40);
	write_cmos_sensor(0x65,0x00);
	write_cmos_sensor(0x66,0x66);
	write_cmos_sensor(0x67,0x00);
	write_cmos_sensor(0x68,0x68);
	write_cmos_sensor(0x72,0x70);
	write_cmos_sensor(0xfb,0x25);
	write_cmos_sensor(0xf0,0x00);//offset
	write_cmos_sensor(0xf1,0x00);
	write_cmos_sensor(0xf2,0x00);
	write_cmos_sensor(0xf3,0x00);
	write_cmos_sensor(0xfd,0x02);//raw data digital gain
	write_cmos_sensor(0x00,0xb0);
	write_cmos_sensor(0x01,0xb0);
	write_cmos_sensor(0x03,0xb0);
	write_cmos_sensor(0x04,0xb0);
	write_cmos_sensor(0xfd,0x01);//mipi
	write_cmos_sensor(0xb3,0x00);
	write_cmos_sensor(0x93,0x01);
	write_cmos_sensor(0x9d,0x17);
	write_cmos_sensor(0xc5,0x01);
	write_cmos_sensor(0xc6,0x00);
	write_cmos_sensor(0xb1,0x01);
	write_cmos_sensor(0x8e,0x06);
	write_cmos_sensor(0x8f,0x50);
	write_cmos_sensor(0x90,0x04);
	write_cmos_sensor(0x91,0xc0);
	write_cmos_sensor(0x92,0x01);
	write_cmos_sensor(0xa1,0x05);
	write_cmos_sensor(0xaa,0x01);
	write_cmos_sensor(0xac,0x01);
	}
 	 LOG_INF("Exit sensor_init.\n");  
}    /*    sensor_init  */

static void preview_setting(void)
{
}    /*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
}

static void normal_video_setting(kal_uint16 currefps)
{
}

static void hs_video_setting(void)
{
}

static void slim_video_setting(void)
{
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);
    if(enable)
    {
        write_cmos_sensor(0xfd,0x01);
        write_cmos_sensor(0x0d,0x01);
    }
	else
	{
        write_cmos_sensor(0xfd,0x01);
        write_cmos_sensor(0x0d,0x00);
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
#ifdef SLT_DEVINFO_CMM_SP2508 
 	s_DEVINFO_ccm =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);	
	s_DEVINFO_ccm->device_type = "CCM-S";
	s_DEVINFO_ccm->device_module = "PCOFL0001B";//can change if got module id
	s_DEVINFO_ccm->device_vendor = "Sanlaishi";
	s_DEVINFO_ccm->device_ic = "SP2508";
	s_DEVINFO_ccm->device_version = "Superpix";
	s_DEVINFO_ccm->device_info = "200W";
#endif
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {               
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);      
			#ifdef SLT_DEVINFO_CMM_SP2508 
				s_DEVINFO_ccm->device_used = DEVINFO_USED;
				devinfo_check_add_device(s_DEVINFO_ccm);
			#endif	     
                return ERROR_NONE;
            }   
            LOG_INF("Read sensor id fail,i2c_write_id:0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
        *sensor_id = 0xFFFFFFFF;
	#ifdef SLT_DEVINFO_CMM_SP2508 
		s_DEVINFO_ccm->device_used = DEVINFO_UNUSED;
		devinfo_check_add_device(s_DEVINFO_ccm);
	#endif
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
			 LOG_INF("Read sensor id fail,i2c_write_id:0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
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
#ifdef DEBUG_SENSOR_SP2508
	if(read_from_extern_flash(SP2508_SD_FILE_PATH)) {
		SP_LOG("________________from t!\n");
		SP2508_Initialize_from_T_Flash(SP2508_SD_FILE_PATH);
	} else {
		preview_setting();
		sensor_init();	
    } 
#else  
	/* initail sequence write in  */
	sensor_init();
#endif
	
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
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
#ifndef DEBUG_SENSOR_SP2508
    preview_setting();
#endif

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
#ifndef DEBUG_SENSOR_SP2508
    capture_setting(imgsensor.current_fps);
#endif

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
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
#ifndef DEBUG_SENSOR_SP2508
    normal_video_setting(imgsensor.current_fps);
#endif

    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
#ifndef DEBUG_SENSOR_SP2508
    hs_video_setting();
#endif

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
#ifndef DEBUG_SENSOR_SP2508
    slim_video_setting();
#endif

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


#ifdef DEBUG_SENSOR_SP2508
	if(read_from_extern_flash(SP2508_OTHER_DEBUG_PATH)) {
		printk("sp_zze: other debug!\n");
		SP2508_other_debug(SP2508_OTHER_DEBUG_PATH);
	}
#endif

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
	
	printk("sp_zze: sensor_info->AEShutDelayFrame = %d\n", sensor_info->AEShutDelayFrame);
	printk("sp_zze: sensor_info->AESensorGainDelayFrame = %d\n", sensor_info->AESensorGainDelayFrame);
	printk("sp_zze: sensor_info->AEISPGainDelayFrame = %d\n", sensor_info->AEISPGainDelayFrame);

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
{//This Function not used after ROME
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
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
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
}    /*    feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 SP2508_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    /*    SP2508_MIPI_RAW_SensorInit    */
