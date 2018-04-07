/*
NOTE:
The modification is appended to initialization of image sensor. 
After sensor initialization, use the function
bool otp_update_wb()
and
bool otp_update_lenc(void)
and
then the calibration of AWB & LSC & BLC will be applied. 
After finishing the OTP written, we will provide you the typical value of golden sample.
*/


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
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
	
#include "ov8856mipiraw_Sensor.h"
//#include "ov8856mipiraw_Camera_Sensor_para.h"
//#include "ov8856mipiraw_CameraCustomized.h"


extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);



//#define SUPPORT_FLOATING

#define OTP_LOAD_ADDR         0x3D81
#define OTP_BANK_ADDR         0x3D84

#define LENC_START_ADDR       0x5900
#define LENC_REG_SIZE         240
			
#define OTP_LENC_GROUP_FLAG   0x7028
#define OTP_LENC_GROUP_ADDR   0x7029

#define OTP_BASIC_GROUP_FLAG         0x7010
#define OTP_BASIC_GROUP_ADDR         0x7011

#define OTP_H_START_ADDR     0x3D88
#define OTP_L_START_ADDR     0x3D89
#define OTP_H_END_ADDR       0x3D8A
#define OTP_L_END_ADDR       0x3D8B
#define OTP_GROUP_SIZE       8

#define GAIN_RH_ADDR          0x5019
#define GAIN_RL_ADDR          0x501A
#define GAIN_GH_ADDR          0x501B
#define GAIN_GL_ADDR          0x501C
#define GAIN_BH_ADDR          0x501D
#define GAIN_BL_ADDR          0x501E

#define GAIN_DEFAULT_VALUE    0x0400 // 1x gain

#define OTP_MID               0x02
#define TRULY_TYPICAL_RG      0x135
#define TRULY_TYPICAL_BG      0x12F

#define PFX "OV8856_OTP"
#define OV8856_OTP_DEBUG
#ifdef OV8856_OTP_DEBUG
#define LOG_INF(format, args...)	printk(PFX "[%s] " format, __FUNCTION__, ##args)
#else 
#define LOG_INF(format, args...)
#endif


static unsigned char OV8856MIPI_WRITE_ID = 0x42;

#define Delay(ms)  mdelay(ms)

// R/G and B/G of current camera module
unsigned short rg_ratio = 0;
unsigned short bg_ratio = 0;
unsigned short golden_rg = 0;
unsigned short golden_bg = 0;
unsigned char otp_lenc_data[LENC_REG_SIZE];
unsigned char otp_lenc_data2[LENC_REG_SIZE];
void DPCFuncEnable(void);
void DPCFuncDisable(void);
// Enable OTP read function


kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	iReadReg((u16) addr ,(u8*)&get_byte,OV8856MIPI_WRITE_ID);
	return get_byte;

}

void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	iWriteReg((u16) addr , (u32) para , 1, OV8856MIPI_WRITE_ID);
}

void otp_read_enable(void)
{
	write_cmos_sensor(OTP_LOAD_ADDR, 0x01);
	mdelay(15); // 
}

// Disable OTP read function
void otp_read_disable(void)
{
	write_cmos_sensor(OTP_LOAD_ADDR, 0x00);
	mdelay(5); //
}

void otp_read(unsigned short otp_addr, unsigned char* otp_data)
{
	otp_read_enable();
	*otp_data = read_cmos_sensor(otp_addr);
//	otp_read_disable();
}

/*******************************************************************************
* Function    :  otp_clear
* Description :  Clear OTP buffer 
* Parameters  :  none
* Return      :  none
*******************************************************************************/	
void otp_clear(unsigned short star, unsigned short end)
{
	unsigned short i;
	// After read/write operation, the OTP buffer should be cleared to avoid accident write
	for ( i=star; i<end; i++) 
	{
		write_cmos_sensor(i, 0x00);
	}
	
}

/*******************************************************************************
* Function    :  otp_check_wb_group
* Description :  Check OTP Space Availability
* Parameters  :  [in] index : index of otp group (0, 1, 2)
* Return      :  0, group index is empty
                 1, group index has invalid data
                 2, group index has valid data
                -1, group index error
*******************************************************************************/	
signed char otp_check_wb_group(int index)
{   
	unsigned char  flagBasic;
	unsigned short otp_addr  = OTP_BASIC_GROUP_FLAG;
    if (index > 2)
	{
		LOG_INF("OTP input wb group index %d error\n", index);
		return -1;
	}

	DPCFuncDisable();	
	// select base information flag

	write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	write_cmos_sensor(OTP_H_END_ADDR, (otp_addr>>8) & 0xff);
	write_cmos_sensor(OTP_L_END_ADDR, otp_addr & 0xff);
	msleep(5);
    otp_read(otp_addr, &flagBasic);
	write_cmos_sensor(otp_addr, 0x00);
	DPCFuncEnable();

	// Check all bytes of a group. If all bytes are '0', then the group is empty. 
	// Check from group 1 to group 2, then group 3.
	
    if (index==0)
	{

		flagBasic = (flagBasic>>6) & 0x03;
		if (!flagBasic)
		{
			LOG_INF("wb group %d is empty", index);
			return 0;
		}
		else if (flagBasic == 0x01 )
		{
			LOG_INF("wb group %d has valid data", index);;
			return 2;
		}
		else //if (flagBasic == 0x11)
		{
			LOG_INF("wb group %d has invalid data", index);
			return 1;
		}
	}

	else
	{

		flagBasic=(flagBasic>>4) & 0x03;
		
		if (!flagBasic)
		{
			LOG_INF("wb group %d is empty", index);
			return 0;
		}
		else if (flagBasic == 0x01 )
		{
			LOG_INF("wb group %d has valid data", index);
			return 2;
		}
		else //if (flagBasic == 0x11)
		{
			LOG_INF("wb group %d has invalid data", index);
			return 1;
		}
	}
}

/*******************************************************************************
* Function    :  otp_read_wb_group
* Description :  Read group value and store it in OTP Struct 
* Parameters  :  [in] index : index of otp group (0, 1, 2)
* Return      :  group index (0, 1, 2)
                 -1, error
*******************************************************************************/	
signed char otp_read_wb_group(void)
{
	unsigned char  mid, AWB_light_LSB, rg_ratio_MSB, bg_ratio_MSB,lens; 
	unsigned short otp_addr=0;
	// Check first OTP with valid data
	int index =0;
	for (index=0; index<2; index++)
	{
		if (otp_check_wb_group(index) == 2)
		{
			LOG_INF("read wb from group %d\n", index);
			break;
		}
	}

	if (index == 2)
	{
		LOG_INF("no group has valid data\n");
		return -1;
	}
	DPCFuncDisable();
	// select adress
    otp_addr = OTP_BASIC_GROUP_ADDR + index * OTP_GROUP_SIZE;

	write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	write_cmos_sensor(OTP_H_END_ADDR, ((otp_addr+7)>>8) & 0xff);
	write_cmos_sensor(OTP_L_END_ADDR, (otp_addr+7) & 0xff);
	mdelay(5);
	otp_read(otp_addr, &mid);
	otp_read(otp_addr+1, &lens);

	if (mid == OTP_MID)
	{
		golden_rg = TRULY_TYPICAL_RG;
		golden_bg = TRULY_TYPICAL_BG;
		LOG_INF("This Module is Truly Module\n");
	}
	else
	{
		LOG_INF("This Module is Other Module\n");
		//ÆäËû²Ù×÷
		return -1;
	}
/*	if (lens == 0x01)
	{
		LOG_INF("This LensID is Truly Module\n");
	}
	else
	{
		LOG_INF("This LensID is Other Module\n");
		//ÆäËû²Ù×÷
		return -1;
	}*/
	otp_read(otp_addr+5,  &rg_ratio_MSB);
	otp_read(otp_addr+6,  &bg_ratio_MSB);
	otp_read(otp_addr+7, &AWB_light_LSB);	
	otp_clear(otp_addr,otp_addr+7);
	DPCFuncEnable();
	
	rg_ratio = (rg_ratio_MSB<<2) | ((AWB_light_LSB & 0xC0)>>6);
	bg_ratio = (bg_ratio_MSB<<2) | ((AWB_light_LSB & 0x30)>>4);


	
	LOG_INF("rg_ratio=0x%x, bg_ratio=0x%x\n", rg_ratio, bg_ratio);
	
	if ((rg_ratio_MSB == 0) || (bg_ratio_MSB == 0))
	{
		DPCFuncDisable();
	// select adress
    otp_addr = OTP_BASIC_GROUP_ADDR + index * OTP_GROUP_SIZE;
    
    write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
    write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
    write_cmos_sensor(OTP_H_END_ADDR, ((otp_addr+7)>>8) & 0xff);
   	write_cmos_sensor(OTP_L_END_ADDR, (otp_addr+7) & 0xff);
    mdelay(5);
		otp_read(otp_addr+5,  &rg_ratio_MSB);
		otp_read(otp_addr+6,  &bg_ratio_MSB);
		otp_read(otp_addr+7, &AWB_light_LSB);	
		otp_clear(otp_addr,otp_addr+7);
		DPCFuncEnable();	
	
		rg_ratio = (rg_ratio_MSB<<2) | ((AWB_light_LSB & 0xC0)>>6);
		bg_ratio = (bg_ratio_MSB<<2) | ((AWB_light_LSB & 0x30)>>4);
		LOG_INF("rg_ratio=0x%x, bg_ratio=0x%x\n", rg_ratio, bg_ratio);
	
		if ((rg_ratio_MSB == 0) || (bg_ratio_MSB == 0))
		{
			return -1;		
		}
	}
	LOG_INF("read wb finished\n");
	return index;
}

#ifdef SUPPORT_FLOATING //Use this if support floating point values
/*******************************************************************************
* Function    :  otp_apply_wb
* Description :  Calcualte and apply R, G, B gain to module
* Parameters  :  [in] golden_rg : R/G of golden camera module
                 [in] golden_bg : B/G of golden camera module
* Return      :  1, success; 0, fail
*******************************************************************************/	
bool otp_apply_wb()
{
	unsigned short gain_r = GAIN_DEFAULT_VALUE;
	unsigned short gain_g = GAIN_DEFAULT_VALUE;
	unsigned short gain_b = GAIN_DEFAULT_VALUE;

	double ratio_r, ratio_g, ratio_b;
	double cmp_rg, cmp_bg;

	if (!golden_rg || !golden_bg)
	{
		LOG_INF("golden_rg / golden_bg can not be zero\n");
		return 0;
	}
	// Calcualte R, G, B gain of current module from R/G, B/G of golden module
        // and R/G, B/G of current module
	cmp_rg = 1.0 * rg_ratio / golden_rg;
	cmp_bg = 1.0 * bg_ratio / golden_bg;

	if ((cmp_rg<1) && (cmp_bg<1))
	{
		// R/G < R/G golden, B/G < B/G golden
		ratio_g = 1;
		ratio_r = 1 / cmp_rg;
		ratio_b = 1 / cmp_bg;
	}
	else if (cmp_rg > cmp_bg)
	{
		// R/G >= R/G golden, B/G < B/G golden
		// R/G >= R/G golden, B/G >= B/G golden
		ratio_r = 1;
		ratio_g = cmp_rg;
		ratio_b = cmp_rg / cmp_bg;
	}
	else
	{
		// B/G >= B/G golden, R/G < R/G golden
		// B/G >= B/G golden, R/G >= R/G golden
		ratio_b = 1;
		ratio_g = cmp_bg;
		ratio_r = cmp_bg / cmp_rg;
	}

	// write sensor wb gain to registers
	// 0x0400 = 1x gain
	if (ratio_r != 1)
	{
		gain_r = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_r);
		write_cmos_sensor(GAIN_RH_ADDR, gain_r >> 8);
		write_cmos_sensor(GAIN_RL_ADDR, gain_r & 0x00ff);
	}

	if (ratio_g != 1)
	{
		gain_g = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_g);
		write_cmos_sensor(GAIN_GH_ADDR, gain_g >> 8);
		write_cmos_sensor(GAIN_GL_ADDR, gain_g & 0x00ff);
	}

	if (ratio_b != 1)
	{
		gain_b = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_b);
		write_cmos_sensor(GAIN_BH_ADDR, gain_b >> 8);
		write_cmos_sensor(GAIN_BL_ADDR, gain_b & 0x00ff);
	}

	LOG_INF("cmp_rg=%f, cmp_bg=%f\n", cmp_rg, cmp_bg);
	LOG_INF("ratio_r=%f, ratio_g=%f, ratio_b=%f\n", ratio_r, ratio_g, ratio_b);
	LOG_INF("gain_r=0x%x, gain_g=0x%x, gain_b=0x%x\n", gain_r, gain_g, gain_b);
	return 1;
}

#else //Use this if not support floating point values

#define OTP_MULTIPLE_FAC	10000
bool otp_apply_wb(void)
{
	unsigned short gain_r = GAIN_DEFAULT_VALUE;
	unsigned short gain_g = GAIN_DEFAULT_VALUE;
	unsigned short gain_b = GAIN_DEFAULT_VALUE;

	unsigned short ratio_r, ratio_g, ratio_b;
	unsigned short cmp_rg, cmp_bg;
	bool is_rg_valid = KAL_FALSE; // default: false
	bool is_bg_valid = KAL_FALSE; // default: false

	if (!golden_rg || !golden_bg)
	{
		LOG_INF("golden_rg / golden_bg can not be zero\n");
		return 0;
	}
	
	// Calcualte R, G, B gain of current module from R/G, B/G of golden module
    // and R/G, B/G of current module
	cmp_rg = OTP_MULTIPLE_FAC * rg_ratio / golden_rg;
	cmp_bg = OTP_MULTIPLE_FAC * bg_ratio / golden_bg;

// heafei 20170213 ++
// rg_ratio/golden_rg is in [0.8, 1.2], cmp_rg is in [8000, 12000]
// bg_ratio/golden_bg is in [0.8, 1.2], cmp_bg is in [8000, 12000]
/* Vanzo:yangbinbin on: Mon, 13 Feb 2017 21:15:08 +0800
 * validate cmp_rg&cmp_bg whether golden value 0.8 to 1.2
	is_rg_valid = (cmp_rg > OTP_MULTIPLE_FAC*0.8) && (cmp_rg < OTP_MULTIPLE_FAC*1.2);
	is_bg_valid = (cmp_bg > OTP_MULTIPLE_FAC*0.8) && (cmp_bg < OTP_MULTIPLE_FAC*1.2);
 */
	is_rg_valid = (cmp_rg > ((OTP_MULTIPLE_FAC*4)/5)) && (cmp_rg < ((OTP_MULTIPLE_FAC*6)/5));
	is_bg_valid = (cmp_bg > ((OTP_MULTIPLE_FAC*4)/5)) && (cmp_bg < ((OTP_MULTIPLE_FAC*6)/5));
// End of Vanzo: yangbinbin

	if (!is_rg_valid|| !is_bg_valid)
	{
		LOG_INF("[heafei] rg_ratio / bg_ratio invalid\n");
		return KAL_FALSE;
	}
// heafei 20170213 --


	if ((cmp_rg < 1 * OTP_MULTIPLE_FAC) && (cmp_bg < 1 * OTP_MULTIPLE_FAC))
	{
		// R/G < R/G golden, B/G < B/G golden
		ratio_g = 1 * OTP_MULTIPLE_FAC;
		ratio_r = 1 * OTP_MULTIPLE_FAC * OTP_MULTIPLE_FAC / cmp_rg;
		ratio_b = 1 * OTP_MULTIPLE_FAC * OTP_MULTIPLE_FAC / cmp_bg;
	}
	else if (cmp_rg > cmp_bg)
	{
		// R/G >= R/G golden, B/G < B/G golden
		// R/G >= R/G golden, B/G >= B/G golden
		ratio_r = 1 * OTP_MULTIPLE_FAC;
		ratio_g = cmp_rg;
		ratio_b = OTP_MULTIPLE_FAC * cmp_rg / cmp_bg;
	}
	else
	{
		// B/G >= B/G golden, R/G < R/G golden
		// B/G >= B/G golden, R/G >= R/G golden
		ratio_b = 1 * OTP_MULTIPLE_FAC;
		ratio_g = cmp_bg;
		ratio_r = OTP_MULTIPLE_FAC * cmp_bg / cmp_rg;
	}

	// write sensor wb gain to registers
	// 0x0400 = 1x gain
	if (ratio_r != 1 * OTP_MULTIPLE_FAC)
	{
		gain_r = GAIN_DEFAULT_VALUE * ratio_r / OTP_MULTIPLE_FAC;
		write_cmos_sensor(GAIN_RH_ADDR, gain_r >> 8);
		write_cmos_sensor(GAIN_RL_ADDR, gain_r & 0x00ff);
	}

	if (ratio_g != 1 * OTP_MULTIPLE_FAC)
	{
		gain_g = GAIN_DEFAULT_VALUE * ratio_g / OTP_MULTIPLE_FAC;
		write_cmos_sensor(GAIN_GH_ADDR, gain_g >> 8);
		write_cmos_sensor(GAIN_GL_ADDR, gain_g & 0x00ff);
	}

	if (ratio_b != 1 * OTP_MULTIPLE_FAC)
	{
		gain_b = GAIN_DEFAULT_VALUE * ratio_b / OTP_MULTIPLE_FAC;
		write_cmos_sensor(GAIN_BH_ADDR, gain_b >> 8);
		write_cmos_sensor(GAIN_BL_ADDR, gain_b & 0x00ff);
	}

	LOG_INF("cmp_rg=%d, cmp_bg=%d\n", cmp_rg, cmp_bg);
	LOG_INF("ratio_r=%d, ratio_g=%d, ratio_b=%d\n", ratio_r, ratio_g, ratio_b);
	LOG_INF("gain_r=0x%x, gain_g=0x%x, gain_b=0x%x\n", gain_r, gain_g, gain_b);
	return 1;
}
#endif /* SUPPORT_FLOATING */

/*******************************************************************************
* Function    :  otp_update_wb
* Description :  Update white balance settings from OTP
* Parameters  :  void
* Return      :  1, success; 0, fail
*******************************************************************************/	
bool otp_update_wb(void) 
{
	LOG_INF("start wb update\n");

	if (otp_read_wb_group() != -1)
	{
		if (otp_apply_wb() == 1)
		{
			LOG_INF("wb update finished\n");
			return 1;
		}
	}
	LOG_INF("wb update failed\n");
	return 0;
}

/*******************************************************************************
* Function    :  otp_check_lenc_group
* Description :  Check OTP Space Availability
* Parameters  :  [in] int  index : index of otp group (0, 1, 2)
* Return      :  0, group index is empty
                 1, group index has invalid data
                 2, group index has valid data
                -1, group index error
*******************************************************************************/	
signed char otp_check_lenc_group(int  index)
{   
	unsigned char  flag;
	// select lenc flag
    unsigned short otp_addr = OTP_LENC_GROUP_FLAG;
    
    DPCFuncDisable();

    write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	write_cmos_sensor(OTP_H_END_ADDR, (otp_addr>>8) & 0xff);
	write_cmos_sensor(OTP_L_END_ADDR, otp_addr & 0xff);
	msleep(5);
    otp_read(otp_addr, &flag);
 //   otp_read_disable();
	write_cmos_sensor(otp_addr, 0x00);
	DPCFuncEnable();

	// Check all bytes of a group. If all bytes are '0', then the group is empty. 
	// Check from group 1 to group 2, then group 3.
	if (index==0)
	{
      flag=(flag>>6) & 0x03;

	  if (!flag)
	  {
		  LOG_INF("lenc group %d is empty", index);
		  return 0;
	  }
	  else if (flag == 0x01)
	  {
		  LOG_INF("lenc group %d has valid data", index);
		  return 2;
	  }
	  else //if (flag == 0x11)
	  {
		  LOG_INF("lenc group %d has invalid data", index);
		  return 1;
	  }
	}

	else
	{

		flag=(flag>>4) & 0x03;
		
		if (!flag)
		{
			LOG_INF("lenc group %d is empty", index);
			return 0;
		}
		else if (flag == 0x01)
		{
			LOG_INF("lenc group %d has valid data", index);
			return 2;
		}
		else //if (flag == 0x11)
		{
			LOG_INF("lenc group %d has invalid data", index);
			return 1;
		}


	}

}

void otp_check_lenc(void)
{
    int i;
    for (i=0; i<LENC_REG_SIZE; i++) 
    {
        if (otp_lenc_data2[i] != otp_lenc_data[i])
        {
            if (otp_lenc_data[i] == 0)
            {
                otp_lenc_data[i] = otp_lenc_data2[i];
            }
            else if (otp_lenc_data2[i] == 0)
            {
            }
            else
            {
                otp_lenc_data[i] = (otp_lenc_data[i] + otp_lenc_data2[i]) / 2;
            }
        }
    }
}


/*******************************************************************************
* Function    :  otp_read_lenc_group
* Description :  Read group value and store it in OTP Struct 
* Parameters  :  [in] int index : index of otp group (0, 1, 2)
* Return      :  group index (0, 1, 2)
                 -1, error
*******************************************************************************/	
signed char otp_read_lenc_group(void)
{
	int i,index =0;
	unsigned short otp_addr=0;
	for (index=0; index<2; index++)
	{
		if (otp_check_lenc_group(index) == 2)
		{
			LOG_INF("read lenc from group %d\n", index);
			break;
		}
	}

	if (index == 2)
	{
		LOG_INF("no group has valid data\n");
		return -1;
	}
	DPCFuncDisable();

	// read lenc data
	otp_addr = OTP_LENC_GROUP_ADDR + index * (LENC_REG_SIZE+1);
	write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	write_cmos_sensor(OTP_H_END_ADDR, ((otp_addr+LENC_REG_SIZE)>>8) & 0xff);
	write_cmos_sensor(OTP_L_END_ADDR, (otp_addr+LENC_REG_SIZE) & 0xff);
	otp_read_enable();
	for (i=0; i<LENC_REG_SIZE; i++) 
	{
		otp_lenc_data[i] = read_cmos_sensor(otp_addr);
		otp_addr++;
	}
//	otp_read_disable();
	otp_clear(otp_addr,otp_addr+LENC_REG_SIZE);
	DPCFuncEnable();
	
	// second
	DPCFuncDisable();

	// read lenc data
	otp_addr = OTP_LENC_GROUP_ADDR + index * (LENC_REG_SIZE+1);
	write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	write_cmos_sensor(OTP_H_END_ADDR, ((otp_addr+LENC_REG_SIZE)>>8) & 0xff);
	write_cmos_sensor(OTP_L_END_ADDR, (otp_addr+LENC_REG_SIZE) & 0xff);
	otp_read_enable();
	for (i=0; i<LENC_REG_SIZE; i++) 
	{
		otp_lenc_data2[i] = read_cmos_sensor(otp_addr);
		otp_addr++;
	}
//	otp_read_disable();
	otp_clear(otp_addr,otp_addr+LENC_REG_SIZE);
	DPCFuncEnable();
	
	otp_check_lenc();

	LOG_INF("read lenc finished\n");
	return index;
}

/*******************************************************************************
* Function    :  otp_apply_lenc
* Description :  Apply lens correction setting to module
* Parameters  :  none
* Return      :  none
*******************************************************************************/	
void otp_apply_lenc(void)
{
	int i;
	unsigned char temp=0;
	// write lens correction setting to registers
	LOG_INF("apply lenc setting\n");
  
  //Enable LSC
	temp = read_cmos_sensor(0x5000);
	temp = temp | 0x20;  //0x80 ?
	write_cmos_sensor(0x5000, temp);
	
	for (i=0; i<LENC_REG_SIZE; i++)
	{
		write_cmos_sensor(LENC_START_ADDR+i, otp_lenc_data[i]);
		LOG_INF("0x%x, 0x%x\n", LENC_START_ADDR+i, otp_lenc_data[i]);
	}
	
	LOG_INF("LSC_Enable:0x%x\n", temp);
}

/*******************************************************************************
* Function    :  otp_update_lenc
* Description :  Get lens correction setting from otp, then apply to module
* Parameters  :  none
* Return      :  1, success; 0, fail
*******************************************************************************/	
bool otp_update_lenc(void) 
{
	LOG_INF("start lenc update\n");

	if (otp_read_lenc_group() != -1)
	{
		otp_apply_lenc();
		LOG_INF("lenc update finished\n");
		return 1;
	}

	LOG_INF("lenc update failed\n");
	return 0;
}
/*****************************************************************************
*Function    :  DPC Function
*Description :  To avoid  OTP memory R/W error 
 Before doing OTP read/write,register 0x5002[3] must be set to ¡°0¡±. 
 After OTP memory access,set register 0x5002[3] back to ¡°1¡±.
******************************************************************************/
void DPCFuncEnable(void)
{
	unsigned short ctrl;
	ctrl = read_cmos_sensor(0x5001);
	ctrl = ctrl | 0x08;
	write_cmos_sensor(0x5001,ctrl);
	msleep(10);
}
void DPCFuncDisable(void)
{
	unsigned short ctrl;
	ctrl = read_cmos_sensor(0x5001);
	ctrl = ctrl & (~0x08);
	write_cmos_sensor(0x5001,ctrl);
	msleep(10);

}


void ov8856_otp_update(void)
{
    otp_update_lenc();
    otp_update_wb();
}






