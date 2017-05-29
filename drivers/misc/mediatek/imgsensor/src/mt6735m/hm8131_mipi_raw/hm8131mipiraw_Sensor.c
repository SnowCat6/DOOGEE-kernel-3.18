/*******************************************************************************************/
	  

/*******************************************************************************************/

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
#include <linux/timer.h>
#include <linux/jiffies.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "cust_gpio_usage.h"

#include "hm8131mipiraw_Sensor.h"
#include "hm8131mipiraw_Camera_Sensor_para.h"
#include "hm8131mipiraw_CameraCustomized.h"


#define __DEBUG_SIOSENSOR__
#ifdef __DEBUG_SIOSENSOR__
#define SIOSENSOR_INFO(fmt, args...)\
        do{    \
           xlog_printk(ANDROID_LOG_INFO, "[SIOSENSOR]" , fmt, ##args); \
        } while(0)
#define SIOSENSOR_ERR(fmt, args...) \
        do {    \
			printk("Error Error Error,Check Lines=%d",__LINE__);\
           	xlog_printk(ANDROID_LOG_ERROR, "[SIOSENSOR]" , fmt, ##args); \
        } while(0)
#else
#define SIOSENSOR_INFO(a,...)
#define SIOSENSOR_ERR(a,...)
#endif

#define SIOSENSOR(fmt, args...)\
        do{    \
           xlog_printk(ANDROID_LOG_INFO, "[SIOSENSOR]" , fmt, ##args); \
        } while(0)


/* FIXME: old factors and DIDNOT use now. s*/
kal_uint32	HM8131MIPI_FAC_SENSOR_REG;
static kal_uint8 chipVersion = 0;
MSDK_SENSOR_CONFIG_STRUCT	HM8131SensorConfigData;

SENSOR_REG_STRUCT	HM8131SensorCCT[]						= HM8131MIPI_CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT	HM8131SensorReg[HM8131MIPI_ENGINEER_END]	= HM8131MIPI_CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

//#define __SENSOR_CHECK__  //2014

static DEFINE_SPINLOCK(HM8131mipi_drv_lock);
#define		mDELAY(ms)		mdelay(ms)


static HM8131MIPI_PARA_STRUCT HM8131SensorDriver={
	//.SlaveAddr = 

	.SensorMode=SENSOR_MODE_INIT,
	
	.PClk	= HM8131MIPI_PREVIEW_CLK,//91.2M
	.DummyLines  = 0,
	.DummyPixels = 0,

	.LineLength			=HM8131MIPI_PV_PERIOD_PIXEL_NUMS,
	.FrameHeight		=HM8131MIPI_PV_PERIOD_LINE_NUMS,
	.MaxExposureLines 	= HM8131MIPI_PV_PERIOD_LINE_NUMS - 4, // TODO:

	.AutoFlickerMode=KAL_FALSE,

	.BaseGain	= BASEGAIN, //64
	.Gain2Reg	= 0x00,
	.Reg2Gain	= 0x00,
	.Shutter 	=0x00,

	.SensorFps	=300,
	.MirrorFlip =IMAGE_NORMAL,
};


extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_uint16 HM8131MIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[3] = {(char)((addr & 0xFF00)>>8), (char)(addr & 0xFF) , (char)(para & 0xFF)};
	
	iWriteRegI2C( (u8 *)puSendCmd, 3, HM8131SensorDriver.SlaveAddr );
	return TRUE;
}

kal_uint16 HM8131MIPI_read_cmos_sensor(kal_uint32 addr)
{
	u8	get_byte = 0;
    char puSendCmd[2] = {(char)((addr & 0xFF00)>>8), (char)(addr & 0xFF)};
	
	iReadRegI2C( (u8 *)puSendCmd, 2, (u8 *)&get_byte, 1, HM8131SensorDriver.SlaveAddr );
	return (kal_uint16) get_byte;
}

#define		Sleep(ms)	mdelay(ms)

static void HM8131SetSensorSetting(struct HM8131REG *SensorSettings)
{
	int Cnt=0;

	for(Cnt=0;SensorSettings[Cnt].Reg != SETTING_END;Cnt++){

		if(SensorSettings[Cnt].Reg == SETTING_DELAY)
			mdelay(SensorSettings[Cnt].Value);
		else
			HM8131MIPI_write_cmos_sensor(SensorSettings[Cnt].Reg & 0xFFFF,SensorSettings[Cnt].Value& 0xFF);
	}

}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
static void HM8131MIPI_Set_Dummy(const kal_uint32 iPixels, const kal_uint32 iLines)
{
	kal_uint32	LineLength	= HM8131MIPI_PV_PERIOD_PIXEL_NUMS;
	kal_uint32	FrameHeight = HM8131MIPI_PV_PERIOD_LINE_NUMS;
	
	SIOSENSOR_INFO( ">>> HM8131MIPI_Set_Dummy(): iPixels=%d,iLines=%d\n",iPixels ,iLines);

	if(HM8131SensorDriver.SensorMode == SENSOR_MODE_PREVIEW){
		LineLength=HM8131MIPI_PV_PERIOD_PIXEL_NUMS + iPixels;
		FrameHeight=HM8131MIPI_PV_PERIOD_LINE_NUMS + iLines;
	}else if(HM8131SensorDriver.SensorMode == SENSOR_MODE_VIDEO){
		LineLength=HM8131MIPI_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		FrameHeight=HM8131MIPI_VIDEO_PERIOD_LINE_NUMS + iLines;
	}else if(HM8131SensorDriver.SensorMode == SENSOR_MODE_CAPTURE){
		LineLength=HM8131MIPI_FULL_PERIOD_PIXEL_NUMS + iPixels;
		FrameHeight=HM8131MIPI_FULL_PERIOD_LINE_NUMS + iLines;
	}

	// Set total line length
	if(LineLength !=HM8131SensorDriver.LineLength){
		if(LineLength > HM8131MIPI_MAX_LINE_LENGTH){
			SIOSENSOR_ERR("Set Dummy Error\n ");
			return;
		}
		HM8131SensorDriver.LineLength = LineLength;
		SIOSENSOR_INFO( "HM8131MIPI_Set_Dummy(): LineLength Change to =%d\n", LineLength );
		HM8131MIPI_write_cmos_sensor( 0x0342, (HM8131SensorDriver.LineLength >> 8) & 0xff );
		HM8131MIPI_write_cmos_sensor( 0x0343, (HM8131SensorDriver.LineLength	 ) & 0xff );
	}

	// Set total frame length
	if(FrameHeight !=HM8131SensorDriver.FrameHeight){
		if(FrameHeight > HM8131MIPI_MAX_FRAME_LENGTH){
			SIOSENSOR_ERR("Set Dummy Error\n ");
			return;
		}
		HM8131SensorDriver.FrameHeight = FrameHeight;
		SIOSENSOR_INFO( "HM8131MIPI_Set_Dummy(): FrameHeight Change to =%d\n", FrameHeight );
		HM8131MIPI_write_cmos_sensor( 0x0340, (HM8131SensorDriver.FrameHeight >> 8) & 0xff );
		HM8131MIPI_write_cmos_sensor( 0x0341, (HM8131SensorDriver.FrameHeight	 ) & 0xff );
	}


	// TODO:
	if((FrameHeight !=HM8131SensorDriver.FrameHeight)||(LineLength !=HM8131SensorDriver.LineLength))
    	HM8131MIPI_write_cmos_sensor(0x0104, 0x00);
		
	
	SIOSENSOR_INFO("LineLength=%x,FrameHeight=%x\n",HM8131SensorDriver.LineLength,HM8131SensorDriver.FrameHeight);
	SIOSENSOR_INFO("MaxExposureLines=%x\n",HM8131SensorDriver.MaxExposureLines);

	SIOSENSOR_INFO( "<<< HM8131MIPI_Set_Dummy()\n" );

}

static void HM8131MIPI_Write_Shutter(kal_uint32 iShutter)
{	
	SIOSENSOR_INFO( ">>> HM8131MIPI_Write_Shutter()\n" );
	
	//This is Special case for HM8131SensorDriver, It Need Write FrameLength when shutter >FrameHeight
	//In here,Don't Increase this avariable HM8131SensorDriver.FrameHeight
	if( iShutter > HM8131SensorDriver.FrameHeight){
		HM8131MIPI_write_cmos_sensor( 0x0340, (iShutter >> 8) & 0xff );
		HM8131MIPI_write_cmos_sensor( 0x0341, (iShutter	 ) & 0xff );
	}else{
		HM8131MIPI_write_cmos_sensor( 0x0340, (HM8131SensorDriver.FrameHeight >> 8) & 0xff );
		HM8131MIPI_write_cmos_sensor( 0x0341, (HM8131SensorDriver.FrameHeight	 ) & 0xff );
	}

	SIOSENSOR_INFO("[ABC]iShutter=%d,HM8131SensorDriver.FrameHeight=%d",iShutter,HM8131SensorDriver.FrameHeight);

	if( iShutter < 3 )
		iShutter = 3;
	
	// Set Shutter. (Coarse integration time, uint: lines.)
	HM8131MIPI_write_cmos_sensor( 0x0202, (iShutter >> 8) & 0xff );
	HM8131MIPI_write_cmos_sensor( 0x0203, (iShutter	 ) & 0xff );

	// TODO:
    HM8131MIPI_write_cmos_sensor(0x0104, 0x00);
	
	SIOSENSOR_INFO( "<<< HM8131MIPI_Write_Shutter()\n" );
}


/*************************************************************************
* FUNCTION
*   HM8131MIPI_SetShutter
*
* DESCRIPTION
*   This function set e-Shutter of HM8131MIPI to change exposure time.
*
* PARAMETERS
*   iShutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void HM8131MIPI_SetShutter(kal_uint32 iShutter)
{
	SIOSENSOR_INFO( ">>> HM8131MIPI_SetShutter(): iShutter=0x%X, %d\n", iShutter, iShutter );
	
	//if( HM8131SensorDriver.Shutter == iShutter )
	//	return;
	
	spin_lock( &HM8131mipi_drv_lock );
	HM8131SensorDriver.Shutter = iShutter;
	spin_unlock( &HM8131mipi_drv_lock );
	
	HM8131MIPI_Write_Shutter( iShutter );
	
	SIOSENSOR_INFO( "<<< HM8131MIPI_SetShutter()\n" );
}

/*************************************************************************
* FUNCTION
*   HM8131MIPI_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   Shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HM8131MIPI_read_shutter(void)
{
	kal_uint16	temp_reg1, temp_reg2;
	UINT32		Shutter = 0;
	
	temp_reg1 = HM8131MIPI_read_cmos_sensor( 0x0202 );
	temp_reg2 = HM8131MIPI_read_cmos_sensor( 0x0203 );
	Shutter	  = (temp_reg1 << 8) | temp_reg2;
	return Shutter;
}

/*******************************************************************************
*
********************************************************************************/
static kal_uint16 HM8131MIPIReg2Gain(const kal_uint16 iReg)
{
#if 0
	kal_uint8	iI;
	kal_uint16	iGain = HM8131SensorDriver.BaseGain;		// 1x-gain base
	
	// Range: 1x to 32x
	// Gain = (GAIN[7] + 1) * (GAIN[6] + 1) * (GAIN[5] + 1) * (GAIN[4] + 1) * (1 + GAIN[3:0] / 16)
	for( iI = 8; iI >= 4; iI-- ) {
		iGain *= (((iReg >> iI) & 0x01) + 1);
	}
	return iGain +  (iGain * (iReg & 0x0F)) / 16; //HM8131SensorDriver.Reg2Gain
#endif
}

/*******************************************************************************
*
********************************************************************************/
static struct HM8131_GAIN
{
    kal_uint16 gainvalue;
    kal_uint32 analoggain;
};

static struct HM8131_GAIN HM8131_gain_table[]=
{
	{0x0000, 10000},
	{0x0001, 10625},
	{0x0002, 11250},
	{0x0003, 11875},
	{0x0004, 12500},
	{0x0005, 13125},
	{0x0006, 13750},
	{0x0007, 14375},
	{0x0008, 15000},
	{0x0009, 15625},
	{0x000A, 16250},
	{0x000B, 16875},
	{0x000C, 17500},
	{0x000D, 18125},
	{0x000E, 18750},
	{0x000F, 19375},
	{0x0010, 20000},
	{0x0012, 21250},
	{0x0014, 22500},
	{0x0016, 23750},
	{0x0018, 25000},
	{0x001A, 26250},
	{0x001C, 27500},
	{0x001E, 28750},
	{0x0020, 30000},
	{0x0022, 31250},
	{0x0024, 32500},
	{0x0026, 33750},
	{0x0028, 35000},
	{0x002A, 36250},
	{0x002C, 37500},
	{0x002E, 38750},
	{0x0030, 40000},
	{0x0034, 42500},
	{0x0038, 45000},
	{0x003C, 47500},
	{0x0040, 50000},
	{0x0044, 52500},
	{0x0048, 55000},
	{0x004C, 57500},
	{0x0050, 60000},
	{0x0054, 62500},
	{0x0058, 65000},
	{0x005C, 67500},
	{0x0060, 70000},
	{0x0064, 72500},
	{0x0068, 75000},
	{0x006C, 77500},
	{0x0070, 80000},
	{0x0078, 85000},
	{0x0080, 90000},
	{0x0088, 95000},
	{0x0090, 100000},
	{0x0098, 105000},
	{0x00A0, 110000},
	{0x00A8, 115000},
	{0x00B0, 120000},
	{0x00B8, 125000},
	{0x00C0, 130000},
	{0x00C8, 135000},
	{0x00D0, 140000},
	{0x00D8, 145000},
	{0x00E0, 150000},
	{0x00E8, 155000},
	{0x00F0, 160000},
	{0xFFFF, 0},
};

static kal_uint32 HM8131MIPIGain2Reg(kal_uint16 Gain)
{
#if 0
    kal_uint8 i = 0;

    kal_uint16 iReg = 0x0000;
	kal_uint16 iGain=Gain;
    kal_uint32 gain_tmp0 = 0;

    gain_tmp0=((kal_uint32)iGain*10000) / BASEGAIN;
    do
    {
        if (gain_tmp0 < HM8131_gain_table[i].analoggain)
            break;
        i++;
    }while (HM8131_gain_table[i].analoggain != 0);

    if (i == 0){
        iReg = 0x00;
    }else{
        iReg = HM8131_gain_table[i-1].gainvalue & 0xFF;
    }

    return (iReg & 0x007F);
#else
	kal_uint8 iReg=0x00;
	kal_uint8 iRegH=0x00;
	kal_uint8 iRegL=0x00;

	kal_uint16 iGain=Gain;

	if(iGain>BASEGAIN)
		iRegH=(iGain/BASEGAIN)-1;

	iRegL=(iGain%BASEGAIN)/4;
	
	iReg=((iRegH<<4)&0xF0) | (iRegL & 0x0F);
	
    return (iReg & 0xFF);

#endif
}

/*************************************************************************
* FUNCTION
*	HM8131MIPI_SetGain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	gain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
void HM8131MIPI_SetGain(UINT16 iGain)
{
	unsigned long flags;
	
	SIOSENSOR_INFO( ">>> HM8131MIPI_SetGain(): iGain=0x%X\n", iGain );

	//if(iGain==HM8131SensorDriver.Reg2Gain)
	//	return;
	
	spin_lock_irqsave( &HM8131mipi_drv_lock, flags );
	HM8131SensorDriver.Reg2Gain = iGain;
	HM8131SensorDriver.Gain2Reg = HM8131MIPIGain2Reg( iGain );
	spin_unlock_irqrestore( &HM8131mipi_drv_lock, flags );

	SIOSENSOR_INFO( "[ABC]:Gain2Reg=0x%x, Reg2Gain=%d\n", HM8131SensorDriver.Gain2Reg, HM8131SensorDriver.Reg2Gain );

	SIOSENSOR_INFO( "HM8131MIPI_SetGain:Gain2Reg=0x%x, Reg2Gain=%d\n", HM8131SensorDriver.Gain2Reg, HM8131SensorDriver.Reg2Gain );
	HM8131MIPI_write_cmos_sensor(0x0205, HM8131SensorDriver.Gain2Reg );
    HM8131MIPI_write_cmos_sensor(0x0104, 0x00);

	SIOSENSOR_INFO( "<<< HM8131MIPI_SetGain()\n" );
}

/*************************************************************************
* FUNCTION
*	read_HM8131_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	None
*
* RETURNS
*	gain : sensor global gain
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 read_HM8131_gain(void)
{
#if 0
	kal_uint16 read_gain = 0;
	
	read_gain = HM8131MIPI_read_cmos_sensor( 0x00 );
	
	spin_lock( &HM8131mipi_drv_lock );
	HM8131SensorDriver.Gain2Reg = read_gain;
	HM8131SensorDriver.Reg2Gain = HM8131MIPIReg2Gain( HM8131SensorDriver.Gain2Reg );
	spin_unlock( &HM8131mipi_drv_lock );
	
	SIOSENSOR_INFO( "HM8131SensorDriver.Gain2Reg=0x%X, HM8131SensorDriver.Reg2Gain=%d\n", HM8131SensorDriver.Gain2Reg, HM8131SensorDriver.Reg2Gain );
	
	return HM8131SensorDriver.Gain2Reg;
#endif	
}

void HM8131MIPI_camera_para_to_sensor(void)
{
//	kal_uint32	i;
//	
//	for( i = 0; 0xFFFFFFFF != HM8131SensorReg[i].Addr; i++ )
//	{
//		HM8131MIPI_write_cmos_sensor( HM8131SensorReg[i].Addr, HM8131SensorReg[i].Para );
//	}
//	for( i = HM8131MIPI_ENGINEER_START_ADDR; 0xFFFFFFFF != HM8131SensorReg[i].Addr; i++ )
//	{
//		HM8131MIPI_write_cmos_sensor( HM8131SensorReg[i].Addr, HM8131SensorReg[i].Para );
//	}
//	for( i = HM8131MIPI_FACTORY_START_ADDR; i < HM8131MIPI_FACTORY_END_ADDR; i++ )
//	{
//		HM8131MIPI_write_cmos_sensor( HM8131SensorCCT[i].Addr, HM8131SensorCCT[i].Para );
//	}
}

/*************************************************************************
* FUNCTION
*	HM8131MIPI_sensor_to_camera_para
*
* DESCRIPTION
*	// update camera_para from sensor register
*
* PARAMETERS
*	None
*
* RETURNS
*	gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
void HM8131MIPI_sensor_to_camera_para(void)
{
//	kal_uint32	i, temp_data;
//	
//	for( i = 0; 0xFFFFFFFF != HM8131SensorReg[i].Addr; i++ )
//	{
//		temp_data = HM8131MIPI_read_cmos_sensor( HM8131SensorReg[i].Addr );
//		spin_lock( &HM8131mipi_drv_lock );
//		HM8131SensorReg[i].Para = temp_data;
//		spin_unlock( &HM8131mipi_drv_lock );
//	}
//	for( i = HM8131MIPI_ENGINEER_START_ADDR; 0xFFFFFFFF != HM8131SensorReg[i].Addr; i++ )
//	{
//		temp_data = HM8131MIPI_read_cmos_sensor( HM8131SensorReg[i].Addr );
//		spin_lock( &HM8131mipi_drv_lock );
//		HM8131SensorReg[i].Para = temp_data;
//		spin_unlock( &HM8131mipi_drv_lock );
//	}
}

/*************************************************************************
* FUNCTION
*	HM8131MIPI_get_sensor_group_count
*
* DESCRIPTION
*	//
*
* PARAMETERS
*	None
*
* RETURNS
*	gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int32  HM8131MIPI_get_sensor_group_count(void)
{
	return HM8131MIPI_GROUP_TOTAL_NUMS;
}

void HM8131MIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
	switch( group_idx )
	{
		case HM8131MIPI_PRE_GAIN:
			sprintf( (char *)group_name_ptr, "CCT" );
			*item_count_ptr = 2;
			break;
		case HM8131MIPI_CMMCLK_CURRENT:
			sprintf( (char *)group_name_ptr, "CMMCLK Current" );
			*item_count_ptr = 1;
			break;
		case HM8131MIPI_FRAME_RATE_LIMITATION:
			sprintf( (char *)group_name_ptr, "Frame Rate Limitation" );
			*item_count_ptr = 2;
			break;
		case HM8131MIPI_REGISTER_EDITOR:
			sprintf( (char *)group_name_ptr, "Register Editor" );
			*item_count_ptr = 2;
			break;
		default:
			ASSERT( 0 );
	}
}

void HM8131MIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
	kal_int16	temp_reg=0;
	kal_uint16	temp_gain=0, temp_addr=0, temp_para=0;
	
	switch( group_idx )
	{
		case HM8131MIPI_PRE_GAIN:
			switch( item_idx )
			{
				case 0:
					sprintf( (char *)info_ptr->ItemNamePtr, "Pregain-R" );
					temp_addr = HM8131MIPI_PRE_GAIN_R_INDEX;
					break;
				case 1:
					sprintf( (char *)info_ptr->ItemNamePtr, "Pregain-Gr" );
					temp_addr = HM8131MIPI_PRE_GAIN_Gr_INDEX;
					break;
				case 2:
					sprintf( (char *)info_ptr->ItemNamePtr, "Pregain-Gb" );
					temp_addr = HM8131MIPI_PRE_GAIN_Gb_INDEX;
					break;
				case 3:
					sprintf( (char *)info_ptr->ItemNamePtr, "Pregain-B" );
					temp_addr = HM8131MIPI_PRE_GAIN_B_INDEX;
					break;
				case 4:
					sprintf( (char *)info_ptr->ItemNamePtr, "SENSOR_BASEGAIN" );
					temp_addr = HM8131MIPI_SENSOR_BASEGAIN;
					break;
				default:
					ASSERT( 0 );
			}
			
			temp_para = HM8131SensorCCT[temp_addr].Para;
			//temp_gain = (temp_para/HM8131SensorDriver.sensorBaseGain) * 1000;
			
			info_ptr->ItemValue		= temp_gain;
			info_ptr->IsTrueFalse	= KAL_FALSE;
			info_ptr->IsReadOnly	= KAL_FALSE;
			info_ptr->IsNeedRestart	= KAL_FALSE;
			info_ptr->Min			= HM8131MIPI_MIN_ANALOG_GAIN * 1000;
			info_ptr->Max			= HM8131MIPI_MAX_ANALOG_GAIN * 1000;
			break;
		case HM8131MIPI_CMMCLK_CURRENT:
			switch( item_idx )
			{
				case 0:
					sprintf( (char *)info_ptr->ItemNamePtr, "Drv Cur[2,4,6,8]mA" );
					
					//temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
					temp_reg = ISP_DRIVING_2MA;
					if( temp_reg==ISP_DRIVING_2MA )
					{
						info_ptr->ItemValue = 2;
					}
					else if( temp_reg==ISP_DRIVING_4MA )
					{
						info_ptr->ItemValue = 4;
					}
					else if( temp_reg==ISP_DRIVING_6MA )
					{
						info_ptr->ItemValue = 6;
					}
					else if( temp_reg==ISP_DRIVING_8MA )
					{
						info_ptr->ItemValue = 8;
					}
					
					info_ptr->IsTrueFalse	= KAL_FALSE;
					info_ptr->IsReadOnly	= KAL_FALSE;
					info_ptr->IsNeedRestart	= KAL_TRUE;
					info_ptr->Min			= 2;
					info_ptr->Max			= 8;
					break;
				default:
					ASSERT( 0 );
			}
			break;
		case HM8131MIPI_FRAME_RATE_LIMITATION:
			switch( item_idx )
			{
				case 0:
					sprintf( (char *)info_ptr->ItemNamePtr, "Max Exposure Lines" );
					info_ptr->ItemValue		= 111;  //MT9P017_MAX_EXPOSURE_LINES;
					info_ptr->IsTrueFalse	= KAL_FALSE;
					info_ptr->IsReadOnly	= KAL_TRUE;
					info_ptr->IsNeedRestart	= KAL_FALSE;
					info_ptr->Min			= 0;
					info_ptr->Max			= 0;
					break;
				case 1:
					sprintf( (char *)info_ptr->ItemNamePtr, "Min Frame Rate" );
					info_ptr->ItemValue		= 12;
					info_ptr->IsTrueFalse	= KAL_FALSE;
					info_ptr->IsReadOnly	= KAL_TRUE;
					info_ptr->IsNeedRestart	= KAL_FALSE;
					info_ptr->Min			= 0;
					info_ptr->Max			= 0;
					break;
				default:
					ASSERT( 0 );
			}
			break;
		case HM8131MIPI_REGISTER_EDITOR:
			switch( item_idx )
			{
				case 0:
					sprintf( (char *)info_ptr->ItemNamePtr, "REG Addr." );
					info_ptr->ItemValue		= 0;
					info_ptr->IsTrueFalse	= KAL_FALSE;
					info_ptr->IsReadOnly	= KAL_FALSE;
					info_ptr->IsNeedRestart	= KAL_FALSE;
					info_ptr->Min			= 0;
					info_ptr->Max			= 0xFFFF;
					break;
				case 1:
					sprintf( (char *)info_ptr->ItemNamePtr, "REG Value" );
					info_ptr->ItemValue		= 0;
					info_ptr->IsTrueFalse	= KAL_FALSE;
					info_ptr->IsReadOnly	= KAL_FALSE;
					info_ptr->IsNeedRestart	= KAL_FALSE;
					info_ptr->Min			= 0;
					info_ptr->Max			= 0xFFFF;
					break;
				default:
				ASSERT( 0 );
			}
			break;
		default:
			ASSERT( 0 );
	}
}

kal_bool HM8131MIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//	kal_int16	temp_reg;
	kal_uint16	temp_gain=0, temp_addr=0, temp_para=0;
	
	switch( group_idx )
	{
		case HM8131MIPI_PRE_GAIN:
			switch( item_idx )
			{
				case 0:
					temp_addr = HM8131MIPI_PRE_GAIN_R_INDEX;
					break;
				case 1:
					temp_addr = HM8131MIPI_PRE_GAIN_Gr_INDEX;
					break;
				case 2:
					temp_addr = HM8131MIPI_PRE_GAIN_Gb_INDEX;
					break;
				case 3:
					temp_addr = HM8131MIPI_PRE_GAIN_B_INDEX;
					break;
				case 4:
					temp_addr = HM8131MIPI_SENSOR_BASEGAIN;
					break;
				default:
					ASSERT( 0 );
			}
			
			temp_gain = ((ItemValue * BASEGAIN + 500) / 1000);	//+500:get closed integer value
			
			if( temp_gain >= 1*BASEGAIN && temp_gain <= 16*BASEGAIN )
			{
//				temp_para = (temp_gain * HM8131SensorDriver.sensorBaseGain + BASEGAIN/2) / BASEGAIN;
			}
			else
				ASSERT( 0 );
			
			spin_lock( &HM8131mipi_drv_lock );
			HM8131SensorCCT[temp_addr].Para = temp_para;
			spin_unlock( &HM8131mipi_drv_lock );
			HM8131MIPI_write_cmos_sensor( HM8131SensorCCT[temp_addr].Addr, temp_para );
			break;
		case HM8131MIPI_CMMCLK_CURRENT:
			switch( item_idx )
			{
				case 0:
					//no need to apply this item for driving current
					break;
				default:
					ASSERT( 0 );
			}
			break;
		case HM8131MIPI_FRAME_RATE_LIMITATION:
			ASSERT( 0 );
			break;
		case HM8131MIPI_REGISTER_EDITOR:
			switch( item_idx )
			{
				case 0:
					spin_lock( &HM8131mipi_drv_lock );
					HM8131MIPI_FAC_SENSOR_REG = ItemValue;
					spin_unlock( &HM8131mipi_drv_lock );
					break;
				case 1:
					HM8131MIPI_write_cmos_sensor( HM8131MIPI_FAC_SENSOR_REG, ItemValue );
					break;
				default:
					ASSERT( 0 );
			}
			break;
		default:
			ASSERT( 0 );
	}
	return KAL_TRUE;
}

void HM8131MIPISetFlipMirror(kal_int32 ImgMirror)
{
	kal_int8 Val = (kal_int8)HM8131MIPI_read_cmos_sensor(0x0101);
/*Reg0x0101
bit[4]: Flip: 1=on, 0=off
bit[5]: Mirror:1=on, 0=off
*/		
	switch( ImgMirror )
	{
		case IMAGE_NORMAL:
			Val &= 0xFC;
			break;
		case IMAGE_H_MIRROR:
			Val &= 0xFC;
			Val |= 0x20;
			break;
		case IMAGE_V_MIRROR:
			Val &= 0xFC;
			Val |= 0x10;
			break;
		case IMAGE_HV_MIRROR:
			Val |= 0x03;
			break;
	}

	HM8131MIPI_write_cmos_sensor( 0x0101, (kal_int8)Val );
}

/*************************************************************************
* FUNCTION
*   HM8131MIPIOpen
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HM8131MIPIOpen(void)
{
	int  retry = 1;
	volatile signed int i = 0;
	kal_uint16 sensor_id = 0;
	
	SIOSENSOR_INFO( ">>> Open()\n" );

#ifdef HM8131MIPI_WRITE_ID_0
	do {
		HM8131SensorDriver.SlaveAddr=HM8131MIPI_WRITE_ID_0;
		HM8131SensorDriver.SensorID=((HM8131MIPI_read_cmos_sensor(0x0000) << 8) | HM8131MIPI_read_cmos_sensor(0x0001));
		SIOSENSOR( "Slave Addr=%x, Sensor ID=0x%x\n", HM8131SensorDriver.SlaveAddr,HM8131SensorDriver.SensorID );
		if(HM8131SensorDriver.SensorID == HM8131MIPI_SENSOR_ID )
			goto SUC;

		retry--;
	} while( retry > 0 );
#endif

#ifdef HM8131MIPI_WRITE_ID_1
		do {
			HM8131SensorDriver.SlaveAddr=HM8131MIPI_WRITE_ID_1;
			HM8131SensorDriver.SensorID=((HM8131MIPI_read_cmos_sensor(0x0000) << 8) | HM8131MIPI_read_cmos_sensor(0x0001));
			SIOSENSOR( "Slave Addr=%x, Sensor ID=0x%x\n", HM8131SensorDriver.SlaveAddr,HM8131SensorDriver.SensorID );
			if(HM8131SensorDriver.SensorID == HM8131MIPI_SENSOR_ID )
				goto SUC;
	
			retry--;
		} while( retry > 0 );
#endif


	//Check Sensor ID Fail
	if( HM8131SensorDriver.SensorID != HM8131MIPI_SENSOR_ID )
	{
		return ERROR_SENSOR_CONNECT_FAIL;
	}

SUC:	
	//Sensor Init Setting
	
	SIOSENSOR_INFO( ">> Apply HM8131InitSetting\n" );
	//HM8131SetSensorSetting(HM8131_InitSettings);
	chipVersion=HM8131MIPI_read_cmos_sensor(0x0002);
	SIOSENSOR_INFO( "<< Apply HM8131InitSetting\n" );
	
	spin_lock( &HM8131mipi_drv_lock );
	HM8131SensorDriver.SensorMode=SENSOR_MODE_INIT;
	
	HM8131SensorDriver.PClk	= HM8131MIPI_PREVIEW_CLK;//91.2M

	HM8131SensorDriver.Shutter =0x00;

	spin_unlock( &HM8131mipi_drv_lock );

	SIOSENSOR_INFO("SW_Ver=%s, HW_Ver=%s\n",__SW_VER__,__HW_VER__);

	SIOSENSOR_INFO( "<<< Open()\n" );
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   HM8131MIPIGetSensorID
*
* DESCRIPTION
*   This function get the sensor ID
*
* PARAMETERS
*   *sensorID : return the sensor ID
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HM8131MIPIGetSensorID(UINT32 *sensorID)
{
	int  retry = 1;
	
	SIOSENSOR_INFO( ">>> GetSensorID()\n" );	
	
	// check if sensor ID correct
#ifdef HM8131MIPI_WRITE_ID_0
	do {
		HM8131SensorDriver.SlaveAddr=HM8131MIPI_WRITE_ID_0;
		HM8131SensorDriver.SensorID=((HM8131MIPI_read_cmos_sensor(0x0000) << 8) | HM8131MIPI_read_cmos_sensor(0x0001));
		SIOSENSOR( "Slave Addr=%x, Sensor ID=0x%x\n", HM8131SensorDriver.SlaveAddr,HM8131SensorDriver.SensorID );
		if(HM8131SensorDriver.SensorID == HM8131MIPI_SENSOR_ID )
			goto SUC;

		retry--;
	} while( retry > 0 );
#endif

#ifdef HM8131MIPI_WRITE_ID_1
	do {
		HM8131SensorDriver.SlaveAddr=HM8131MIPI_WRITE_ID_1;
		HM8131SensorDriver.SensorID=((HM8131MIPI_read_cmos_sensor(0x0000) << 8) | HM8131MIPI_read_cmos_sensor(0x0001));
		SIOSENSOR( "Slave Addr=%x, Sensor ID=0x%x\n", HM8131SensorDriver.SlaveAddr,HM8131SensorDriver.SensorID );
		if(HM8131SensorDriver.SensorID == HM8131MIPI_SENSOR_ID )
			goto SUC;

		retry--;
	} while( retry > 0 );
#endif


SUC:	
	if( HM8131SensorDriver.SensorID != HM8131MIPI_SENSOR_ID ) {
		SIOSENSOR_ERR("GetSensorID Erro!!!\n");
		*sensorID = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}else{
		SIOSENSOR_ERR("GetSensorID Success!!!\n");	
		*sensorID = HM8131SensorDriver.SensorID;
		SIOSENSOR_INFO( "Slave Addr=%x, Sensor ID=0x%x\n", HM8131SensorDriver.SlaveAddr,HM8131SensorDriver.SensorID );
		return ERROR_NONE;
	}

	SIOSENSOR_INFO( "<<< GetSensorID()\n" );	
}


/*************************************************************************
* FUNCTION
*   HM8131MIPIClose
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HM8131MIPIClose(void)
{
#ifdef HM8131MIPI_DRIVER_TRACE
	SIOSENSOR_INFO( "HM8131MIPIClose()\n" );
#endif
	//CISModulePowerOn( FALSE );
	//DRV_I2CClose( HM8131MIPIhDrvI2C );

	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   HM8131MIPIPreview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HM8131MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 dummy_line = 0;
	
	SIOSENSOR_INFO(">>> Preview()\n");

	if(HM8131SensorDriver.SensorMode != SENSOR_MODE_PREVIEW){	
		SIOSENSOR_INFO( ">> Apply HM8131_1MSettings\n" );
		HM8131MIPI_write_cmos_sensor(0x0100,0x00);
	    mdelay(72);
	    HM8131MIPI_write_cmos_sensor(0x0103,0x00);
	    HM8131MIPI_write_cmos_sensor(0x0100,0x02);
		mdelay(10);
		HM8131SetSensorSetting(HM8131_PreviewSettings);
	    HM8131MIPI_write_cmos_sensor(0x0100,0x01);
		SIOSENSOR_INFO( "<< Apply HM8131_1MSettings\n" );
	}

	spin_lock( &HM8131mipi_drv_lock );
	HM8131SensorDriver.SensorMode  = SENSOR_MODE_PREVIEW;	
	HM8131SensorDriver.PClk	= HM8131MIPI_PREVIEW_CLK;

	HM8131SensorDriver.LineLength=HM8131MIPI_PV_PERIOD_PIXEL_NUMS;
	HM8131SensorDriver.FrameHeight=HM8131MIPI_PV_PERIOD_LINE_NUMS;
	HM8131SensorDriver.MaxExposureLines	= HM8131SensorDriver.FrameHeight - 4;

	spin_unlock( &HM8131mipi_drv_lock );

	HM8131SensorDriver.DummyPixels = 0;
	HM8131SensorDriver.DummyLines = 0;
	HM8131MIPI_Set_Dummy( HM8131SensorDriver.DummyPixels, HM8131SensorDriver.DummyLines );

	// TODO:
	//HM8131MIPI_write_cmos_sensor(0x0104, 0x00);


	SIOSENSOR_INFO("<<< Preview()\n");
	return ERROR_NONE;
}

UINT32 HM8131MIPIVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 dummy_line = 0;
	
	SIOSENSOR_INFO(">>> Video()\n");
	
	if(HM8131SensorDriver.SensorMode != SENSOR_MODE_VIDEO){
		SIOSENSOR_INFO( ">> Apply HM8131_1MSettings\n" );
		HM8131MIPI_write_cmos_sensor(0x0100,0x00);
	    mdelay(72);
	    HM8131MIPI_write_cmos_sensor(0x0103,0x00);
	    HM8131MIPI_write_cmos_sensor(0x0100,0x02);
		HM8131SetSensorSetting(HM8131_VideoSettings);
	    HM8131MIPI_write_cmos_sensor(0x0100,0x01);
		SIOSENSOR_INFO( "<< Apply HM8131_1MSettings\n" );
	}
	
	spin_lock( &HM8131mipi_drv_lock );
	HM8131SensorDriver.SensorMode  = SENSOR_MODE_VIDEO;
	
	HM8131SensorDriver.PClk			= HM8131MIPI_VIDEO_CLK;
	
	HM8131SensorDriver.LineLength=HM8131MIPI_VIDEO_PERIOD_PIXEL_NUMS;
	HM8131SensorDriver.FrameHeight=HM8131MIPI_VIDEO_PERIOD_LINE_NUMS;
	HM8131SensorDriver.MaxExposureLines	= HM8131SensorDriver.FrameHeight - 4;
	spin_unlock( &HM8131mipi_drv_lock );


	HM8131SensorDriver.DummyPixels = 0;
	HM8131SensorDriver.DummyLines = 0;
	HM8131MIPI_Set_Dummy( HM8131SensorDriver.DummyPixels, HM8131SensorDriver.DummyLines );
	
	SIOSENSOR_INFO("<<< Video()\n");
	return ERROR_NONE;
}

UINT32 HM8131MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 cap_fps;
	
	SIOSENSOR_INFO(">>> Capture()\n");
	
	if(HM8131SensorDriver.SensorMode != SENSOR_MODE_CAPTURE){
		SIOSENSOR_INFO( ">> Apply HM8131_1080PSettings\n" );
		HM8131MIPI_write_cmos_sensor(0x0100,0x00);
	    mdelay(38);// very important!! Without this, it fails!

	    HM8131MIPI_write_cmos_sensor(0x0103,0x00);
	    HM8131MIPI_write_cmos_sensor(0x0100,0x02);
		HM8131SetSensorSetting(HM8131_FullSizeSettings);
	    HM8131MIPI_write_cmos_sensor(0x0100,0x01);
		SIOSENSOR_INFO( "<< Apply HM8131_1080PSettings\n" );
	}
	
	spin_lock( &HM8131mipi_drv_lock );
	HM8131SensorDriver.SensorMode  = SENSOR_MODE_CAPTURE;	
	HM8131SensorDriver.PClk	= HM8131MIPI_CAPTURE_CLK;
	HM8131SensorDriver.LineLength=HM8131MIPI_FULL_PERIOD_PIXEL_NUMS;
	HM8131SensorDriver.FrameHeight=HM8131MIPI_FULL_PERIOD_LINE_NUMS;
	HM8131SensorDriver.MaxExposureLines	= HM8131SensorDriver.FrameHeight - 4;
	spin_unlock( &HM8131mipi_drv_lock );
	
	HM8131SensorDriver.DummyPixels = 0;
	HM8131SensorDriver.DummyLines = 0;	
	HM8131MIPI_Set_Dummy( HM8131SensorDriver.DummyPixels, HM8131SensorDriver.DummyLines );
	
	SIOSENSOR_INFO("<<< Capture()\n");
	return ERROR_NONE;
}

UINT32 HM8131MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	SIOSENSOR_INFO(">>> GetResolution!!\n");
	
	pSensorResolution->SensorPreviewWidth	= HM8131MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight	= HM8131MIPI_IMAGE_SENSOR_PV_HEIGHT;
	
	pSensorResolution->SensorFullWidth		= HM8131MIPI_IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight	= HM8131MIPI_IMAGE_SENSOR_FULL_HEIGHT;
	
	pSensorResolution->SensorVideoWidth	= HM8131MIPI_IMAGE_SENSOR_VIDEO_WIDTH;
	pSensorResolution->SensorVideoHeight	= HM8131MIPI_IMAGE_SENSOR_VIDEO_HEIGHT;

	SIOSENSOR_INFO("<<< GetResolution!!\n");
	return ERROR_NONE;
}   /* HM8131MIPIGetResolution() */

UINT32 HM8131MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	SIOSENSOR_INFO(">>> GetInfo!!\n");

	//Mask????
	pSensorInfo->SensorPreviewResolutionX = HM8131MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY = HM8131MIPI_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX = HM8131MIPI_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY = HM8131MIPI_IMAGE_SENSOR_FULL_HEIGHT;
	
	pSensorInfo->SensroInterfaceType		= SENSOR_INTERFACE_TYPE_MIPI;
	pSensorInfo->SensorOutputDataFormat		= HM8131MIPI_COLOR_FORMAT;
	pSensorInfo->SensorClockPolarity		= SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorClockFallingPolarity	= SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity		= SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity		= SENSOR_CLOCK_POLARITY_LOW;//high
	
	pSensorInfo->SensorDrivingCurrent	= ISP_DRIVING_8MA;
    pSensorInfo->SensorMasterClockSwitch=0;
	
	pSensorInfo->AEShutDelayFrame		= 0;/* The frame of setting Shutter default 0 for TG int */
	pSensorInfo->AESensorGainDelayFrame	= 0;/* The frame of setting sensor gain */
	pSensorInfo->AEISPGainDelayFrame	= 2;
	
	pSensorInfo->CaptureDelayFrame	= 2;
	pSensorInfo->PreviewDelayFrame	= 2;
	pSensorInfo->VideoDelayFrame	= 2;

	spin_lock( &HM8131mipi_drv_lock );
	HM8131SensorDriver.MirrorFlip = pSensorConfigData->SensorImageMirror;
	spin_unlock( &HM8131mipi_drv_lock );
//add  2014

        pSensorInfo->SensorModeNum	= 3;
	pSensorInfo->IHDR_Support	= 0;
	pSensorInfo->IHDR_LE_FirstLine	= 0;

        pSensorInfo->SensorWidthSampling	= 3;
	pSensorInfo->SensorHightSampling	= 0;



 //add 
	switch( ScenarioId )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pSensorInfo->SensorClockFreq	= 24;	///26;
			pSensorInfo->SensorClockRisingCount	= 0;
			
			pSensorInfo->SensorGrabStartX = HM8131MIPI_PV_X_START;
			pSensorInfo->SensorGrabStartY = HM8131MIPI_PV_Y_START;
			
			pSensorInfo->SensorMIPILaneNumber	= SENSOR_MIPI_4_LANE;
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->SensorPacketECCOrder					  = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq	= 24;	///26;
			pSensorInfo->SensorClockRisingCount	= 0;
			
			pSensorInfo->SensorGrabStartX = HM8131MIPI_VIDEO_X_START;
			pSensorInfo->SensorGrabStartY = HM8131MIPI_VIDEO_Y_START;
			
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount	  = 0;
		 	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->SensorPacketECCOrder					  = 1;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq	= 24;	///26;
			pSensorInfo->SensorClockRisingCount	= 0;
			
			pSensorInfo->SensorGrabStartX = HM8131MIPI_FULL_X_START;
			pSensorInfo->SensorGrabStartY = HM8131MIPI_FULL_Y_START;	
			
			pSensorInfo->SensorMIPILaneNumber					  = SENSOR_MIPI_4_LANE;
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->SensorPacketECCOrder					  = 1;
			break;
		default:
			pSensorInfo->SensorClockFreq	= 24;	///26;
			pSensorInfo->SensorClockRisingCount	= 0;
			
			pSensorInfo->SensorGrabStartX = HM8131MIPI_PV_X_START;
			pSensorInfo->SensorGrabStartY = HM8131MIPI_PV_Y_START;
			
			pSensorInfo->SensorMIPILaneNumber	= SENSOR_MIPI_4_LANE;
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount	  = 0;
		 	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount	  = 0;
			pSensorInfo->SensorPacketECCOrder					  = 1;
			break;
	}
	
	memcpy( pSensorConfigData, &HM8131SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT) );

	SIOSENSOR_INFO("<<< GetInfo!!\n");
	return ERROR_NONE;
}

UINT32 HM8131MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	SIOSENSOR_INFO(">>> Control()\n");

	SIOSENSOR_INFO( "CurrentScenarioId=%d\n", ScenarioId );
	
	switch( ScenarioId )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			HM8131MIPIPreview( pImageWindow, pSensorConfigData );
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			HM8131MIPIVideo( pImageWindow, pSensorConfigData );
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			HM8131MIPICapture( pImageWindow, pSensorConfigData );
			break;
		default:
			return ERROR_INVALID_SCENARIO_ID;
	}
		
	SIOSENSOR_INFO("<<< Control()\n");
	return ERROR_NONE;
}

UINT32 HM8131MIPISetVideoMode(UINT16 u2FrameRate)
{
	kal_uint32 FrameLength= 0;
	
	SIOSENSOR_INFO( ">>> HM8131MIPISetVideoMode(): frame rate=%d\n", u2FrameRate );

	if( u2FrameRate == 0 ){
		SIOSENSOR_INFO("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if( u2FrameRate > 30 || u2FrameRate < 5 )
		SIOSENSOR_ERR( "Error frame rate setting\n" );

	// TODO: 
	if( SENSOR_MODE_VIDEO == HM8131SensorDriver.SensorMode ) { //min@20140411
		HM8131SensorDriver.AutoFlickerMode = KAL_FALSE;
		return KAL_TRUE;
	}

	if(HM8131SensorDriver.SensorFps == u2FrameRate*10)
		return KAL_TRUE;

	HM8131SensorDriver.SensorFps=u2FrameRate*10;


#ifdef __SENSOR_CHECK__
	FrameLength = (HM8131SensorDriver.PClk*1000*10) / (HM8131SensorDriver.LineLength * 2 *HM8131SensorDriver.SensorFps);
#else
	FrameLength = (HM8131SensorDriver.PClk*1000*10) / (HM8131SensorDriver.LineLength *HM8131SensorDriver.SensorFps);
#endif

	SIOSENSOR_INFO( "HM8131MIPISetVideoMode(): FrameLength Change to =%d\n", FrameLength );

	if(FrameLength > HM8131SensorDriver.FrameHeight)
		HM8131SensorDriver.DummyLines = FrameLength - HM8131SensorDriver.FrameHeight;
	else
		HM8131SensorDriver.DummyLines=0;

	HM8131MIPI_Set_Dummy( 0, HM8131SensorDriver.DummyLines );

	SIOSENSOR_INFO( "<<< HM8131MIPISetVideoMode()\n" );
	
	return KAL_TRUE;
}

UINT32 HM8131SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	kal_uint32	FrameLength=HM8131SensorDriver.FrameHeight;

	SIOSENSOR_INFO( ">>> HM8131SetAutoFlickerMode(): Frame rate(10base)= %d, %d\n", bEnable, u2FrameRate );
	
	if( bEnable ) {
		spin_lock( &HM8131mipi_drv_lock );
		HM8131SensorDriver.AutoFlickerMode = KAL_TRUE;
		HM8131SensorDriver.SensorFps = u2FrameRate;
		spin_unlock( &HM8131mipi_drv_lock );
		SIOSENSOR_INFO( "Enable Auto flicker\n" );
	}
	else {
		spin_lock( &HM8131mipi_drv_lock );
		HM8131SensorDriver.AutoFlickerMode = KAL_FALSE;
		HM8131SensorDriver.SensorFps = 300;// TODO:
		spin_unlock( &HM8131mipi_drv_lock );
		SIOSENSOR_INFO( "Disable Auto flicker\n" );
	}
	
	if( HM8131SensorDriver.AutoFlickerMode == KAL_TRUE )
	{		
		switch( HM8131SensorDriver.SensorMode)
		{
			case SENSOR_MODE_PREVIEW:
				HM8131SensorDriver.SensorFps=296;
				break;
			case SENSOR_MODE_VIDEO:
				if( HM8131SensorDriver.SensorFps ==300 ) {
					HM8131SensorDriver.SensorFps=306;	
				}else if( HM8131SensorDriver.SensorFps ==150 ){
					HM8131SensorDriver.SensorFps=148;
				}
				break;
			case SENSOR_MODE_CAPTURE:
				HM8131SensorDriver.SensorFps=148;
				break;
			default:
				HM8131SensorDriver.SensorFps=296;
				break;
		}
		
#ifdef __SENSOR_CHECK__
		FrameLength = (HM8131SensorDriver.PClk*1000*10) /(HM8131SensorDriver.LineLength * 2 *HM8131SensorDriver.SensorFps);
#else
		FrameLength = (HM8131SensorDriver.PClk*1000*10) /(HM8131SensorDriver.LineLength*HM8131SensorDriver.SensorFps);
#endif

		if( FrameLength >HM8131SensorDriver.FrameHeight ){
			HM8131SensorDriver.DummyLines = FrameLength - HM8131SensorDriver.FrameHeight;
		}else
			HM8131SensorDriver.DummyLines =0;
		
		HM8131MIPI_Set_Dummy(0,HM8131SensorDriver.DummyLines);

	}

	SIOSENSOR_INFO( "<<< HM8131SetAutoFlickerMode()\n" );
	
	return ERROR_NONE;
}

UINT32 HM8131SetTestPatternMode(kal_bool bEnable)
{
	SIOSENSOR_INFO("[HM8131SetTestPatternMode] Test pattern enable:%d\n", bEnable);
	
	return TRUE;
}

UINT32 HM8131MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
	kal_uint32	FrameLength=HM8131SensorDriver.FrameHeight;

	SIOSENSOR_INFO( ">>> SetMaxFramerateByScenario(): scenarioId=%d, frame rate=%d\n", scenarioId, frameRate );
	
	switch( scenarioId )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			spin_lock( &HM8131mipi_drv_lock );
			HM8131SensorDriver.SensorMode = SENSOR_MODE_PREVIEW;
			spin_unlock( &HM8131mipi_drv_lock );					
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			spin_lock( &HM8131mipi_drv_lock );
			HM8131SensorDriver.SensorMode = SENSOR_MODE_VIDEO;
			spin_unlock( &HM8131mipi_drv_lock );
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			spin_lock( &HM8131mipi_drv_lock );
			HM8131SensorDriver.SensorMode = SENSOR_MODE_CAPTURE;
			spin_unlock( &HM8131mipi_drv_lock );
			break;
		default:
			break;
	}	

	HM8131SensorDriver.SensorFps = frameRate;
	FrameLength = (HM8131SensorDriver.PClk*1000*10) /(HM8131SensorDriver.LineLength*HM8131SensorDriver.SensorFps);
	
	if( FrameLength >HM8131SensorDriver.FrameHeight )
		HM8131SensorDriver.DummyLines = FrameLength - HM8131SensorDriver.FrameHeight;
	else
		HM8131SensorDriver.DummyLines =0;
	
	HM8131MIPI_Set_Dummy(0,HM8131SensorDriver.DummyLines);

	SIOSENSOR_INFO( "<<< SetMaxFramerateByScenario()\n");

	return ERROR_NONE;
}

UINT32 HM8131MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
	SIOSENSOR_INFO( ">>> GetDefaultFramerateByScenario()\n");

#if 0
	switch( scenarioId )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*pframeRate = 300;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			*pframeRate = 150;
			break;
		default:
			break;
	}
#else
	*pframeRate = HM8131SensorDriver.SensorFps;
#endif

	SIOSENSOR_INFO( "<<< GetDefaultFramerateByScenario(): FrameRate=%d\n",*pframeRate);
	return ERROR_NONE;
}

#ifdef __MT6752_32__
static SENSOR_WINSIZE_INFO_STRUCT ImgsensorWinSizeInfo[5]=
{
	{
		(3280),(2464),
		8,8,(3271),(2455),(1632),(1224),
		0,0,(1632),(1224),
		HM8131MIPI_PV_X_START,HM8131MIPI_PV_Y_START,HM8131MIPI_IMAGE_SENSOR_PV_WIDTH,HM8131MIPI_IMAGE_SENSOR_PV_HEIGHT
	},//Preview

	{
		(3280),(2464),
		8,8,(3271),(2455),(3264),(2448),
		0,0,(3264),(2448),
		HM8131MIPI_FULL_X_START,HM8131MIPI_FULL_Y_START,HM8131MIPI_IMAGE_SENSOR_FULL_WIDTH,HM8131MIPI_IMAGE_SENSOR_FULL_HEIGHT
	},//Capture
	{
		(3280),(2464),
		8,8,(3271),(2455),(1632),(1224),
		0,0,(1632),(1224),
		HM8131MIPI_VIDEO_X_START,HM8131MIPI_VIDEO_Y_START,HM8131MIPI_IMAGE_SENSOR_VIDEO_WIDTH,HM8131MIPI_IMAGE_SENSOR_VIDEO_HEIGHT
	},//Video
	
	{
		(3280),(2464),
		8,8,(3271),(2455),(1632),(1224),
		0,0,(1632),(1224),
		HM8131MIPI_PV_X_START,HM8131MIPI_PV_Y_START,HM8131MIPI_IMAGE_SENSOR_PV_WIDTH,HM8131MIPI_IMAGE_SENSOR_VIDEO_HEIGHT
	},//High Speed Video
	
	{
		(3280),(2464),
		8,8,(3271),(2455),(1632),(1224),
		0,0,(1632),(1224),
		HM8131MIPI_PV_X_START,HM8131MIPI_PV_Y_START,HM8131MIPI_IMAGE_SENSOR_PV_WIDTH,HM8131MIPI_IMAGE_SENSOR_PV_HEIGHT
	},//Slim Video

};
UINT32 HM8131MIPIGetCropInfo(UINT8 *pFeaturePara)
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

UINT32 HM8131MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16	= (UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16			= (UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32	= (UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32			= (UINT32 *) pFeaturePara;
	UINT32 SensorRegNumber;
	UINT32 i;
	PNVRAM_SENSOR_DATA_STRUCT	  pSensorDefaultData = (PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT	  *pSensorConfigData = (MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT	  *pSensorRegData	 = (MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo	 = (MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_ITEM_INFO_STRUCT  *pSensorItemInfo	 = (MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_ENG_INFO_STRUCT   *pSensorEngInfo	 = (MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    unsigned long long *pFeatureData=(unsigned long long *) pFeaturePara;
    unsigned long long *pFeatureReturnPara=(unsigned long long *) pFeaturePara;



	SIOSENSOR_INFO(">>> FeatureControl():FeatureId=%x\n",FeatureId);
	switch( FeatureId )
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++= HM8131MIPI_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16 = HM8131MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
#ifdef __SENSOR_CHECK__
			*pFeatureReturnPara16++= HM8131SensorDriver.LineLength * 2;
#else
			*pFeatureReturnPara16++= HM8131SensorDriver.LineLength;
#endif
			*pFeatureReturnPara16 = HM8131SensorDriver.FrameHeight;
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = HM8131SensorDriver.PClk * 1000;
			*pFeatureParaLen = 4;			
			break;
#ifdef __MT6752_32__
		case SENSOR_FEATURE_GET_CROP_INFO:
			HM8131MIPIGetCropInfo(pFeaturePara);
			break;
#endif
		case SENSOR_FEATURE_SET_ESHUTTER:
			printk("[soso]Set Shutter=%d\n",(UINT16)*pFeatureData);
			HM8131MIPI_SetShutter( *pFeatureData );
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			//HM8131MIPI_NightMode( (BOOL) *pFeatureData16 );
			break;
		case SENSOR_FEATURE_SET_GAIN:
			printk("[soso]Set Gain=%d\n",(UINT16)*pFeatureData);
			HM8131MIPI_SetGain( (UINT16) *pFeatureData );
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			//HM8131MIPI_isp_master_clock = *pFeatureData32;
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			HM8131MIPI_write_cmos_sensor( pSensorRegData->RegAddr, pSensorRegData->RegData );
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = HM8131MIPI_read_cmos_sensor( pSensorRegData->RegAddr );
			break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
			SensorRegNumber = HM8131MIPI_FACTORY_END_ADDR;
			for( i=0; i<SensorRegNumber; i++ )
			{
				spin_lock( &HM8131mipi_drv_lock );
				HM8131SensorCCT[i].Addr=*pFeatureData32++;
				HM8131SensorCCT[i].Para=*pFeatureData32++;
				spin_unlock( &HM8131mipi_drv_lock );
			}
			break;
		case SENSOR_FEATURE_GET_CCT_REGISTER:
			SensorRegNumber = HM8131MIPI_FACTORY_END_ADDR;
			if( *pFeatureParaLen < (SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4) )
				return FALSE;
			*pFeatureData32++=SensorRegNumber;
			for( i=0; i<SensorRegNumber; i++ )
			{
				*pFeatureData32++=HM8131SensorCCT[i].Addr;
				*pFeatureData32++=HM8131SensorCCT[i].Para;
			}
			break;
		case SENSOR_FEATURE_SET_ENG_REGISTER:
			SensorRegNumber = HM8131MIPI_ENGINEER_END;
			for( i=0; i<SensorRegNumber; i++ )
			{
				spin_lock( &HM8131mipi_drv_lock );
				HM8131SensorReg[i].Addr=*pFeatureData32++;
				HM8131SensorReg[i].Para=*pFeatureData32++;
				spin_unlock( &HM8131mipi_drv_lock );
			}
			break;
		case SENSOR_FEATURE_GET_ENG_REGISTER:
			SensorRegNumber = HM8131MIPI_ENGINEER_END;
			if( *pFeatureParaLen < (SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4) )
				return FALSE;
			*pFeatureData32++=SensorRegNumber;
			for( i=0; i<SensorRegNumber; i++ )
			{
				*pFeatureData32++=HM8131SensorReg[i].Addr;
				*pFeatureData32++=HM8131SensorReg[i].Para;
			}
			break;
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
			if( *pFeatureParaLen >= sizeof(NVRAM_SENSOR_DATA_STRUCT) )
			{
				pSensorDefaultData->Version	 = NVRAM_CAMERA_SENSOR_FILE_VERSION;
				pSensorDefaultData->SensorId = HM8131MIPI_SENSOR_ID;
				memcpy( pSensorDefaultData->SensorEngReg, HM8131SensorReg, sizeof(SENSOR_REG_STRUCT) * HM8131MIPI_ENGINEER_END );
				memcpy( pSensorDefaultData->SensorCCTReg, HM8131SensorCCT, sizeof(SENSOR_REG_STRUCT) * HM8131MIPI_FACTORY_END_ADDR );
			}
			else
				return FALSE;
			*pFeatureParaLen = sizeof(NVRAM_SENSOR_DATA_STRUCT);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy( pSensorConfigData, &HM8131SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT) );
			*pFeatureParaLen = sizeof(MSDK_SENSOR_CONFIG_STRUCT);
			break;
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
			HM8131MIPI_camera_para_to_sensor();
			break;
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
			HM8131MIPI_sensor_to_camera_para();
			break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
			*pFeatureReturnPara32++=HM8131MIPI_get_sensor_group_count();
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_GET_GROUP_INFO:
			HM8131MIPI_get_sensor_group_info( pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount );
			*pFeatureParaLen = sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ITEM_INFO:
			HM8131MIPI_get_sensor_item_info( pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo );
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_SET_ITEM_INFO:
			HM8131MIPI_set_sensor_item_info( pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue );
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ENG_INFO:
			pSensorEngInfo->SensorId			   = 129;
			pSensorEngInfo->SensorType			   = CMOS_SENSOR;
			pSensorEngInfo->SensorOutputDataFormat = HM8131MIPI_COLOR_FORMAT;
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			*pFeatureReturnPara32 = LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_INITIALIZE_AF:
		case SENSOR_FEATURE_CONSTANT_AF:
		case SENSOR_FEATURE_MOVE_FOCUS_LENS:
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			HM8131MIPIGetSensorID( pFeatureReturnPara32 );
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			HM8131MIPISetVideoMode( *pFeatureData );
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			HM8131SetAutoFlickerMode( (BOOL)*pFeatureData16, *(pFeatureData16+1) );
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			HM8131SetTestPatternMode( (BOOL)*pFeatureData );
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			HM8131MIPISetMaxFramerateByScenario( (MSDK_SCENARIO_ID_ENUM)*pFeatureData, *(pFeatureData+1) );
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			HM8131MIPIGetDefaultFramerateByScenario( (MSDK_SCENARIO_ID_ENUM)*pFeatureData, (MUINT32 *)(uintptr_t)(*(pFeatureData+1)) );
			break;
			case SENSOR_FEATURE_SET_FRAMERATE:
			
				break;
			case SENSOR_FEATURE_SET_HDR:
		
				break;
			case SENSOR_FEATURE_GET_CROP_INFO:
				
				break;
		
			case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:

				break;

		default:
			break;
	}

	
	SIOSENSOR_INFO(">>> FeatureControl()\n");
	return ERROR_NONE;
} 

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

SENSOR_FUNCTION_STRUCT	SensorFuncHM8131MIPI = {
	HM8131MIPIOpen,
	HM8131MIPIGetInfo,
	HM8131MIPIGetResolution,
	HM8131MIPIFeatureControl,
	HM8131MIPIControl,
	HM8131MIPIClose
};

UINT32 HM8131MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if( pfFunc != NULL )
		*pfFunc = &SensorFuncHM8131MIPI;
	
	return ERROR_NONE;
}   /* SensorInit() */
