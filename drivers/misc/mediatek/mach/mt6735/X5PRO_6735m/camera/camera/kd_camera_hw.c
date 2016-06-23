#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>


#include "kd_camera_hw.h"


#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

#include "cust_gpio_usage.h" 


/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_ERR, PFX , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif


#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4


#ifndef BOOL
typedef unsigned char BOOL;
#endif

extern void ISP_MCLK1_EN(BOOL En);

extern char 	g_MainSensorName[32];
extern char 	g_SubSensorName[32];
unsigned long	g_main_rst_output_when_sensor_disable 	= Vol_Low;
unsigned long	g_main_pwd_output_when_sensor_disable 	= Vol_High;
unsigned long	g_sub_rst_output_when_sensor_disable 	= Vol_Low;
unsigned long	g_sub_pwd_output_when_sensor_disable	= Vol_High;

u32 pinSetIdx = 0;//default main sensor
u32 pinSet[3][8] = {
                        //for main sensor
                     {  CAMERA_CMRST_PIN,
                        CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,              /* ON state */
                        GPIO_OUT_ZERO,             /* OFF state */
                        CAMERA_CMPDN_PIN,
                        CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for sub sensor
                     {  CAMERA_CMRST1_PIN,
                        CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        CAMERA_CMPDN1_PIN,
                        CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                   };

PowerCust PowerCustList={
	{
	{GPIO_UNSUPPORTED,GPIO_MODE_GPIO,Vol_Low},   //for AVDD;
	{GPIO_UNSUPPORTED,GPIO_MODE_GPIO,Vol_Low},   //for DVDD;
	{GPIO_UNSUPPORTED,GPIO_MODE_GPIO,Vol_Low},   //for DOVDD;
	{GPIO_UNSUPPORTED,GPIO_MODE_GPIO,Vol_Low},   //for AFVDD;
	{GPIO_UNSUPPORTED,GPIO_MODE_GPIO,Vol_Low},   //for AFEN;
	}
};

PowerUp PowerOnList={
	{
#if defined(OV9760_MIPI_RAW)
		{SENSOR_DRVNAME_OV9760_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 10},
				{AVDD,  Vol_2800, 10},
				{DVDD,  Vol_1200, 10},
				{AFVDD, Vol_2800, 15},
				{PDN,   Vol_Low,  10},
				{RST,   Vol_Low,  10},
				{PDN,   Vol_High, 10},
				{RST,   Vol_High, 10}
			},
		},
#endif
#if defined(OV13850_MIPI_RAW)
		{SENSOR_DRVNAME_OV13850_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 1},
				{AVDD,  Vol_2800, 1},
				{DVDD,  Vol_1200, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  1},
				{RST,   Vol_Low,  1},
				{PDN,   Vol_High, 0},
				{RST,   Vol_High, 0}
			},
		},
#endif
#if defined(OV8865_MIPI_RAW)
		{SENSOR_DRVNAME_OV8865_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 1},
				{AVDD,  Vol_2800, 1},
				{DVDD,  Vol_1200, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  1},
				{RST,   Vol_Low,  1},
				{PDN,   Vol_High, 0},
				{RST,   Vol_High, 0}
			},
		},
#endif
#if defined(OV8858R2A_MIPI_RAW)
		{SENSOR_DRVNAME_OV8858_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{AVDD,  Vol_2800, 1},
				{DOVDD, Vol_1800, 1},
				{DVDD,  Vol_1200, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  1},
				{RST,   Vol_Low,  1},
				{PDN,   Vol_High, 0},
				{RST,   Vol_High, 0}
			},
		},
#endif
#if defined(S5K5E2YA_MIPI_RAW)
		{SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD,  Vol_2800, 0},
				{DVDD,  Vol_1200, 0},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  4},
				{PDN,   Vol_High, 0},
				{RST,   Vol_Low,  1},
				{RST,   Vol_High, 0},
			},
		},
#endif
#if defined(S5K2P8_MIPI_RAW)
		{SENSOR_DRVNAME_S5K2P8_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD,  Vol_2800, 0},
				{DVDD,  Vol_1200, 0},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  4},
				{PDN,   Vol_High, 0},
				{RST,   Vol_Low,  1},
				{RST,   Vol_High, 0},
			},
		},
#endif
#if defined(OV5648_MIPI_RAW)
		{SENSOR_DRVNAME_OV5648_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 1},
				{AVDD,  Vol_2800, 1},
				{DVDD,  Vol_1500, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  1},
				{RST,   Vol_Low,  1},
				{PDN,   Vol_High, 0},
				{RST,   Vol_High, 0}
			},
		},
#endif
#if defined(OV2680_MIPI_RAW)
		{SENSOR_DRVNAME_OV2680_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 1},
				{AVDD,  Vol_2800, 1},
				{DVDD,  Vol_1500, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  1},
				{RST,   Vol_Low,  1},
				{PDN,   Vol_High, 0},
				{RST,   Vol_High, 0}
			},
		},
#endif
#if defined(OV16825_MIPI_RAW)
		{SENSOR_DRVNAME_OV16825_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD,  Vol_2800, 0},
				{DVDD,  Vol_1200, 0},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  0},
				{RST,   Vol_Low,  0},
				{RST,   Vol_High, 0},
			},
		},
#endif
#if defined(IMX135_MIPI_RAW)
		{SENSOR_DRVNAME_IMX135_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{AVDD,	Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{DVDD,	Vol_1100, 10},
				{AFVDD, Vol_2800, 5},
                {PDN,	Vol_Low, 0},
                {PDN,	Vol_High, 0},
                {RST,   Vol_Low,  0},
				{RST,	Vol_High, 0}
			},
		},
#endif
#if defined(IMX175_MIPI_RAW)
		{SENSOR_DRVNAME_IMX175_MIPI_RAW,
			{
				{AVDD, Vol_2800, 1},
				{DOVDD,	Vol_1800, 1},
				{SensorMCLK,Vol_High, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,	Vol_Low,  1},
				{RST,	Vol_Low,  1},
				{PDN,	Vol_High, 1},
				{RST,	Vol_High, 1},
				{DVDD,	Vol_1200, 1}

			},
		},
#endif
#if defined(IMX166_MIPI_RAW)
		{SENSOR_DRVNAME_IMX166_MIPI_RAW,
			{
				{AVDD, Vol_2800, 1},
				{DOVDD,	Vol_1800, 1},
				{SensorMCLK,Vol_High, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,	Vol_Low,  1},
				{RST,	Vol_Low,  1},
				{PDN,	Vol_High, 1},
				{RST,	Vol_High, 1},
				{DVDD,	Vol_1200, 1}

			},
		},
#endif
#if defined(IMX164_MIPI_RAW)
		{SENSOR_DRVNAME_IMX164_MIPI_RAW,
			{
				{AVDD, Vol_2800, 1},
				{DOVDD,	Vol_1800, 1},
				{SensorMCLK,Vol_High, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,	Vol_Low,  1},
				{RST,	Vol_Low,  1},
				{PDN,	Vol_High, 1},
				{RST,	Vol_High, 1},
				{DVDD,	Vol_1200, 1}
			},
		},
#endif
#if defined(IMX145_MIPI_RAW)
		{SENSOR_DRVNAME_IMX145_MIPI_RAW,
			{
				{AVDD, Vol_2800, 1},
				{DOVDD,	Vol_1800, 1},
				{SensorMCLK,Vol_High, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,	Vol_Low,  1},
				{RST,	Vol_Low,  1},
				{PDN,	Vol_High, 1},
				{RST,	Vol_High, 1},
				{DVDD,	Vol_1200, 1}
			},
		},
#endif
#if defined(IMX179_MIPI_RAW)
		{SENSOR_DRVNAME_IMX179_MIPI_RAW,
			{
				{AVDD, Vol_2800, 1},
				{DOVDD,	Vol_1800, 1},

				{SensorMCLK,Vol_High, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,	Vol_Low,  1},
				{RST,	Vol_Low,  1},
				{PDN,	Vol_High, 1},
				{RST,	Vol_High, 1},
				{DVDD,	Vol_1200, 1}

			},
		},
#endif
#if defined(HI545_MIPI_RAW)
		{SENSOR_DRVNAME_HI545_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 1},
				{AVDD,  Vol_2800, 1},
				{DVDD,  Vol_1200, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  1},
				{RST,   Vol_Low,  1},
				{PDN,   Vol_High, 0},
				{RST,   Vol_High, 0}
			},
		},
#endif
#if defined(HI544_MIPI_RAW)
		{SENSOR_DRVNAME_HI544_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 1},
				{AVDD,  Vol_2800, 1},
				{DVDD,  Vol_1200, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  1},
				{RST,   Vol_Low,  1},
				{PDN,   Vol_High, 0},
				{RST,   Vol_High, 0}
			},
		},
#endif
#if defined(GC0310_MIPI_YUV)
		{SENSOR_DRVNAME_GC0310_MIPI_YUV,
			{
				{DOVDD,	Vol_1800, 1},
				{DVDD,	Vol_1500, 1},
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{AVDD,	Vol_2800, 1},
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low, 0},
				{RST,	Vol_High, 0}
			},
		},
#endif
#if defined(GC2355_MIPI_RAW)
		{SENSOR_DRVNAME_GC2355_MIPI_RAW,
			{
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{DOVDD,	Vol_1800, 1},
				{DVDD,	Vol_1500, 1},
				{AVDD,	Vol_2800, 1},
				{AFVDD, Vol_2800, 5},
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low, 0},
				{RST,	Vol_High, 0}
			},
		},
#endif
#if defined(GC0409_MIPI_RAW)
		{SENSOR_DRVNAME_GC0409MIPI_RAW,
			{
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{DOVDD,	Vol_1800, 1},
				{DVDD,	Vol_1800, 1},
				{AVDD,	Vol_2800, 1},
				{AFVDD, Vol_2800, 5},
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low, 0},
				{RST,	Vol_High, 0}
			},
		},
#endif
#if defined(GC5004_MIPI_RAW)
		{SENSOR_DRVNAME_GC5004MIPI_RAW,
			{
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{DOVDD,	Vol_1800, 1},
				{DVDD,	Vol_1500, 1},
				{AVDD,	Vol_2800, 1},
				{AFVDD, Vol_2800, 5},
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low, 0},
				{RST,	Vol_High, 0}
			},
		},
#endif

#if defined(GC5024_MIPI_RAW)
		{SENSOR_DRVNAME_GC5024MIPI_RAW,
			{
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{DOVDD,	Vol_1800, 1},
				{DVDD,	Vol_1500, 1},
				{AVDD,	Vol_2800, 1},
				{AFVDD, Vol_2800, 5},
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low, 0},
				{RST,	Vol_High, 0}
			},
		},
#endif
#if defined(GC2355_RAW)
		{SENSOR_DRVNAME_GC2355_RAW,
			{
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{DOVDD,	Vol_1800, 1},
				{DVDD,	Vol_1500, 1},
				{AVDD,	Vol_2800, 1},
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low, 0},
				{RST,	Vol_High, 0}
			},
		},
#endif

#if defined(GC2145_YUV)
		{SENSOR_DRVNAME_GC2145_YUV,
			{
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{DOVDD,	Vol_1800, 1},
				{DVDD,	Vol_1500, 1},//no used
				{AVDD,	Vol_2800, 1},
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low, 0},
				{RST,	Vol_High, 0}
			},
		},
#endif

#if defined(GC0329_YUV)
		{SENSOR_DRVNAME_GC0329_YUV,
			{
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{DOVDD,	Vol_1800, 1},
				{DVDD,	Vol_1500, 1},//no used
				{AVDD,	Vol_2800, 1},
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low, 0},
				{RST,	Vol_High, 0}
			},
		},
#endif

#if defined(GC0312_YUV)
		{SENSOR_DRVNAME_GC0312_YUV,
			{
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{DOVDD,	Vol_1800, 1},
				{DVDD,	Vol_1500, 1},
				{AVDD,	Vol_2800, 1},
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low, 0},
				{RST,	Vol_High, 0}
			},
		},
#endif

#if defined(HM8131_MIPI_RAW)
				{SENSOR_DRVNAME_HM8131_MIPI_RAW,
					{
						{AVDD,  Vol_2800, 1},
						{DVDD,	Vol_1500, 1},
						{DOVDD,	Vol_1800, 1},
						{AFVDD, Vol_2800, 5},
						{PDN,	Vol_Low,  1},
						{RST,	Vol_Low,  1},
						{SensorMCLK,Vol_High, 0},
						{PDN,	Vol_High, 0},
						{RST,	Vol_High, 0}
					},
				},
#endif
#if defined(HM5040_MIPI_RAW)
				{SENSOR_DRVNAME_HM5040_MIPI_RAW,
					{
						{AVDD,  Vol_2800, 1},
						{DVDD,	Vol_1500, 1},
						{DOVDD,	Vol_1800, 1},
						{AFVDD, Vol_2800, 5},
						{PDN,	Vol_Low,  1},
						{RST,	Vol_Low,  1},
						{SensorMCLK,Vol_High, 0},
						{PDN,	Vol_High, 0},
						{RST,	Vol_High, 0}
					},
				},
#endif
#if defined(GC2755_MIPI_RAW)
		{SENSOR_DRVNAME_GC2755_MIPI_RAW,
			{
				{DOVDD,	Vol_1800, 1},
				{AVDD,	Vol_2800, 1},
				{DVDD,	Vol_1500, 1},//no used
				{AFVDD, Vol_2800, 5},
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low, 0},
				{RST,	Vol_High, 0}
			},
		},
#endif
#if defined(SP5409_MIPI_RAW)
		{SENSOR_DRVNAME_SP5409_MIPI_RAW,
			{
				{AVDD, Vol_2800, 1},
				{DOVDD,	Vol_1800, 1},
#if defined(CONFIG_T93A_PROJ)
				{DVDD,	Vol_1800, 1},  // this voltage need check with fae , every module may different
#else
				{DVDD,	Vol_1500, 1},  // this voltage need check with fae , every module may different
#endif
				{SensorMCLK,Vol_High, 0},
				{AFVDD, Vol_2800, 5},
				{PDN,	Vol_Low,  1},
				{RST,	Vol_Low,  1},
				{PDN,	Vol_High, 1},
				{PDN,	Vol_Low,  0},
				{RST,	Vol_High, 0}
			},
		},
#endif
#if defined(SP2508_MIPI_RAW)
		{SENSOR_DRVNAME_SP2508_MIPI_RAW,
			{
				{AVDD, Vol_2800, 1},
				{DOVDD,	Vol_1800, 1},
				{DVDD,	Vol_1500, 1},  // this voltage need check with fae , every module may different
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low,  5},
				{PDN,	Vol_High, 5},
				{PDN,	Vol_Low,  5},
				{RST,	Vol_Low,  5},
				{RST,	Vol_High, 5}
			},
		},
#endif
#if defined(SP8408_MIPI_RAW)
               {SENSOR_DRVNAME_SP8408_MIPI_RAW,
                      	{
							   {DVDD,  Vol_1200, 1},
                               {AVDD, Vol_2800, 1},
                               {DOVDD, Vol_1800, 1},
                               {SensorMCLK,Vol_High, 0},
                               {AFVDD, Vol_2800, 5},
                               {PDN,   Vol_Low,  1},
                               {RST,   Vol_Low,  1},
                               {PDN,   Vol_High, 1},
                               {RST,   Vol_High, 0}
						},
		},
#endif
#if defined(IMX219_MIPI_RAW)
		{SENSOR_DRVNAME_IMX219_MIPI_RAW,
			{
				{AVDD, Vol_2800, 1},
				{DVDD,	Vol_1200, 1},
				{DOVDD,	Vol_1800, 1},
				{SensorMCLK,Vol_High, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,	Vol_Low,  1},
				{RST,	Vol_Low,  1},
				{PDN,	Vol_High, 1},
				{RST,	Vol_High, 1}
			},
		},
#endif
#if defined(OV5670_MIPI_RAW)
		{SENSOR_DRVNAME_OV5670_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 1},
				{AVDD,  Vol_2800, 1},
				{DVDD,  Vol_1200, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  1},
				{RST,   Vol_Low,  1},
				{PDN,   Vol_High, 0},
				{RST,   Vol_High, 0}
			},
		},
#endif
#if defined(OV5693_MIPI_RAW)
		{SENSOR_DRVNAME_OV5693_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 1},
				{AVDD,  Vol_2800, 1},
				{DVDD,  Vol_1500, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  1},
				{RST,   Vol_Low,  1},
				{PDN,   Vol_High, 0},
				{RST,   Vol_High, 0}
			},
		},
#endif
#if defined(SP0A19_YUV)
		{SENSOR_DRVNAME_SP0A19_YUV,
			{
				{PDN,	Vol_High, 1},
				{RST,   Vol_Low,  1},
				{DOVDD,	Vol_1800, 1},
				{DVDD,	Vol_1500, 1},//no used
				{AVDD,	Vol_2800, 1},
				{SensorMCLK,Vol_High, 0},
				{PDN,	Vol_Low, 0},
				{RST,	Vol_High, 0}
			},
		},
#endif
#if defined(HI551_MIPI_RAW)
		{SENSOR_DRVNAME_HI551_MIPI_RAW,
			{
				{PDN, Vol_Low, 1},
				{RST, Vol_Low, 1},
				{DOVDD, Vol_1800, 1},
				{AVDD, Vol_2800, 1},
				{DVDD, Vol_1200, 1},
				{AFVDD, Vol_2800, 5},
				{PDN, Vol_High, 0},
				{SensorMCLK, Vol_High, 0},
				{RST, Vol_High, 0}
			},
		},
#endif
#if defined(S5K4H5YC_MIPI_RAW)
		{SENSOR_DRVNAME_S5K4H5YC_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD,  Vol_2800, 0},
				{DVDD,  Vol_1200, 0},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  4},
				{PDN,   Vol_High, 0},
				{RST,   Vol_Low,  1},
				{RST,   Vol_High, 0},
			},
		},
#endif
#if defined(S5K3H7YX_MIPI_RAW)
		{SENSOR_DRVNAME_S5K3H7YX_MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD,  Vol_2800, 0},
				{DVDD,  Vol_1200, 0},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  4},
				{PDN,   Vol_High, 0},
				{RST,   Vol_Low,  1},
				{RST,   Vol_High, 0},
			},
		},
#endif
#if defined(GS8604_MIPI_RAW)
		{SENSOR_DRVNAME_GS8604MIPI_RAW,
			{
				{SensorMCLK,Vol_High, 1},
				{DOVDD, Vol_1800, 1},
				{AVDD,  Vol_2800, 1},
				{DVDD,  Vol_1200, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_Low,  4},
				{PDN,   Vol_High, 1},
				{RST,   Vol_Low,  1},
				{RST,   Vol_High, 1},
			},
		},
#endif
#if defined(JX507_MIPI_RAW)
		{SENSOR_DRVNAME_JX507_MIPI_RAW,
			{
				{AVDD,  Vol_2800, 1},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK,Vol_High, 0},
				{DVDD,  Vol_1200, 1},
				{AFVDD, Vol_2800, 5},
				{PDN,   Vol_High,  1},
				{RST,   Vol_Low,  1},
				{PDN,   Vol_Low, 0},
				{RST,   Vol_High, 0}
			},
		},
#endif
		//add new sensor before this line
		{NULL,},
	}
};

static void config_sensor_rst_pwd(char *currSensorName)
{
	int pwListIdx,pwIdx;
	for(pwListIdx=0 ; pwListIdx<16; pwListIdx++)
	{
		//currSensorName
		if(currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName!=NULL) && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
		{
			for(pwIdx=0;pwIdx<12;pwIdx++)
			{  
				if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType == RST)
				{
					if(pinSetIdx==0)
						g_main_rst_output_when_sensor_disable = PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Voltage;
					else
						g_sub_rst_output_when_sensor_disable = PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Voltage;
					break;
				}
			}
			for(pwIdx=0;pwIdx<12;pwIdx++)
			{  
				if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType == PDN)
				{
					if(pinSetIdx==0)
						g_main_pwd_output_when_sensor_disable = PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Voltage;
					else
						g_sub_pwd_output_when_sensor_disable = PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Voltage;
					break;
				}
			}
		}
	}

	for(pwListIdx=0 ; pwListIdx<16; pwListIdx++)
	{
		//g_MainSensorName
		if(g_MainSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName!=NULL) && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,g_MainSensorName)))
		{
			for(pwIdx=0;pwIdx<12;pwIdx++)
			{  
				if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType == RST)
				{
					g_main_rst_output_when_sensor_disable = PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Voltage;
					break;
				}
			}
			for(pwIdx=0;pwIdx<12;pwIdx++)
			{  
				if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType == PDN)
				{
					g_main_pwd_output_when_sensor_disable = PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Voltage;
					break;
				}
			}
		}
		//g_SubSensorName
		if(g_SubSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName!=NULL) && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,g_SubSensorName)))
		{
			for(pwIdx=0;pwIdx<12;pwIdx++)
			{  
				if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType == RST)
				{
					g_sub_rst_output_when_sensor_disable = PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Voltage;
					break;
				}
			}
			for(pwIdx=0;pwIdx<12;pwIdx++)
			{  
				if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType == PDN)
				{
					g_sub_pwd_output_when_sensor_disable = PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Voltage;
					break;
				}
			}
		}
	}
	PK_DBG("[imgsensor_poweron] config_sensor_rst_pwd mainrst=%d, mainpwd=%d, subrst=%d, subpwd=%d\n",g_main_rst_output_when_sensor_disable, g_main_pwd_output_when_sensor_disable, g_sub_rst_output_when_sensor_disable, g_sub_pwd_output_when_sensor_disable);	
	
	if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
	if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
	if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
	if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
	if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
	if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}

	if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST], g_main_rst_output_when_sensor_disable)){PK_DBG("[CAMERA main CMRST] set gpio failed!! \n");}
	if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN], g_main_pwd_output_when_sensor_disable)){PK_DBG("[CAMERA main CMPDN] set gpio failed!! \n");}

	if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST], g_sub_rst_output_when_sensor_disable)){PK_DBG("[CAMERA sub CMRST] set gpio failed!! \n");}
	if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN], g_sub_pwd_output_when_sensor_disable)){PK_DBG("[CAMERA sub CMPDN] set gpio failed!! \n");}
}

static void config_subsensor_rst_pwd_when_nosub(char *currSensorName)
{
	int pwListIdx,pwIdx;
	if((0==strcmp(g_SubSensorName,KDIMGSENSOR_NOSENSOR)) && (pinSetIdx==0))
	{
		for(pwListIdx=0 ; pwListIdx<16; pwListIdx++)
		{
			//currSensorName
			if(currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName!=NULL) && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
			{
				for(pwIdx=0;pwIdx<12;pwIdx++)
				{  
					if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType == RST)
					{
						g_sub_rst_output_when_sensor_disable = PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Voltage;
						break;
					}
				}
				for(pwIdx=0;pwIdx<12;pwIdx++)
				{  
					if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType == PDN)
					{
						g_sub_pwd_output_when_sensor_disable = PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Voltage;
						break;
					}
				}
			}
		
		}
		PK_DBG("[imgsensor_poweron] config_subsensor_rst_pwd_when_nosub subrst=%d, subpwd=%d\n",g_sub_rst_output_when_sensor_disable, g_sub_pwd_output_when_sensor_disable);	
	
		if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
		if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
		if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST], g_sub_rst_output_when_sensor_disable)){PK_DBG("[CAMERA sub CMRST] set gpio failed!! \n");}

		if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN], g_sub_pwd_output_when_sensor_disable)){PK_DBG("[CAMERA sub CMPDN] set gpio failed!! \n");}
	}
}

BOOL hwpoweron(PowerInformation pwInfo, char* mode_name)
{
	if(pwInfo.PowerType == AVDD)
	{
		if(PowerCustList.PowerCustInfo[0].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if(pinSetIdx == 1)
			{
				PK_DBG("[CAMERA SENSOR] Sub camera VCAM_A power on");
				if(TRUE != hwPowerOn(SUB_CAMERA_POWER_VCAM_A,pwInfo.Voltage,mode_name))
       	 		{
            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            		return FALSE;
        		}
#ifdef GPIO_CAMERA_SUB_AVDD_EN_PIN
				PK_DBG("[CAMERA SENSOR] Sub camera VAM_A=GPIO-0x%x power on",GPIO_CAMERA_SUB_AVDD_EN_PIN);
				if(mt_set_gpio_mode(GPIO_CAMERA_SUB_AVDD_EN_PIN,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(GPIO_CAMERA_SUB_AVDD_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");} 					
				if(mt_set_gpio_out(GPIO_CAMERA_SUB_AVDD_EN_PIN,Vol_High)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#endif
			}	
			else
			{
				PK_DBG("[CAMERA SENSOR] Main camera VAM_A power on");
				if(TRUE != hwPowerOn(pwInfo.PowerType,pwInfo.Voltage,mode_name))
	       	 	{
	            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
	            		return FALSE;
	        	}
#ifdef GPIO_CAMERA_MIAN_AVDD_EN_PIN
				PK_DBG("[CAMERA SENSOR] Main camera VAM_A=GPIO-0x%x power on",GPIO_CAMERA_MIAN_AVDD_EN_PIN);
				if(mt_set_gpio_mode(GPIO_CAMERA_MIAN_AVDD_EN_PIN,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(GPIO_CAMERA_MIAN_AVDD_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}					
				if(mt_set_gpio_out(GPIO_CAMERA_MIAN_AVDD_EN_PIN,Vol_High)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#endif				
        	}
		}
		else{
			if(mt_set_gpio_mode(PowerCustList.PowerCustInfo[0].Gpio_Pin,PowerCustList.PowerCustInfo[0].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(PowerCustList.PowerCustInfo[0].Gpio_Pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
			if(mt_set_gpio_out(PowerCustList.PowerCustInfo[0].Gpio_Pin,PowerCustList.PowerCustInfo[0].Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			}			
	}
	else if(pwInfo.PowerType == DVDD)
	{
		if(PowerCustList.PowerCustInfo[1].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if(pinSetIdx == 1)
			{
				PK_DBG("[CAMERA SENSOR] Sub camera VCAM_D power on");
				if(TRUE != hwPowerOn(SUB_CAMERA_POWER_VCAM_D,pwInfo.Voltage,mode_name))
       	 		{
            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            		return FALSE;
        		}
#ifdef GPIO_CAMERA_SUB_DVDD_EN_PIN
				PK_DBG("[CAMERA SENSOR] Sub camera VAM_D=GPIO-0x%x power on",GPIO_CAMERA_SUB_DVDD_EN_PIN);
				if(mt_set_gpio_mode(GPIO_CAMERA_SUB_DVDD_EN_PIN,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(GPIO_CAMERA_SUB_DVDD_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}					
				if(mt_set_gpio_out(GPIO_CAMERA_SUB_DVDD_EN_PIN,Vol_High)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#endif								
			}	
			else
			{
				PK_DBG("[CAMERA SENSOR] Main camera VAM_D power on");
				if(TRUE != hwPowerOn(pwInfo.PowerType,pwInfo.Voltage,mode_name))
	       	 	{
	            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
	            		return FALSE;
	        	}
#ifdef GPIO_CAMERA_MIAN_DVDD_EN_PIN
				PK_DBG("[CAMERA SENSOR] Main camera VAM_D=GPIO-0x%x power on",GPIO_CAMERA_MIAN_DVDD_EN_PIN);
				if(mt_set_gpio_mode(GPIO_CAMERA_MIAN_DVDD_EN_PIN,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(GPIO_CAMERA_MIAN_DVDD_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}					
				if(mt_set_gpio_out(GPIO_CAMERA_MIAN_DVDD_EN_PIN,Vol_High)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#endif				
#ifdef GPIO_CAMERA_IMX135_DVDD_EN_PIN
				PK_DBG("[CAMERA SENSOR] Main camera VAM_D=GPIO-0x%x power on",GPIO_CAMERA_IMX135_DVDD_EN_PIN);
				if(mt_set_gpio_mode(GPIO_CAMERA_IMX135_DVDD_EN_PIN,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(GPIO_CAMERA_IMX135_DVDD_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
				if(mt_set_gpio_out(GPIO_CAMERA_IMX135_DVDD_EN_PIN,Vol_High)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#endif
			}
		}
		else{
			if(mt_set_gpio_mode(PowerCustList.PowerCustInfo[1].Gpio_Pin,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(PowerCustList.PowerCustInfo[1].Gpio_Pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
			if(mt_set_gpio_out(PowerCustList.PowerCustInfo[1].Gpio_Pin,PowerCustList.PowerCustInfo[1].Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			}			
	}
	else if(pwInfo.PowerType == DOVDD)
	{
		if(PowerCustList.PowerCustInfo[2].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if(TRUE != hwPowerOn(pwInfo.PowerType,pwInfo.Voltage,mode_name))
       	 	{
            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            		return FALSE;
        	}
		}
		else{
			if(mt_set_gpio_mode(PowerCustList.PowerCustInfo[2].Gpio_Pin,PowerCustList.PowerCustInfo[2].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(PowerCustList.PowerCustInfo[2].Gpio_Pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
			if(mt_set_gpio_out(PowerCustList.PowerCustInfo[2].Gpio_Pin,PowerCustList.PowerCustInfo[2].Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			}			
	}
#ifndef CONFIG_HCT_AF_POWER_ON_NOISE
	else if(pwInfo.PowerType == AFVDD)
	{
		if(PowerCustList.PowerCustInfo[3].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if(TRUE != hwPowerOn(pwInfo.PowerType,pwInfo.Voltage,mode_name))
       	 	{
            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            		return FALSE;
        	}
		}
		else{
			if(mt_set_gpio_mode(PowerCustList.PowerCustInfo[3].Gpio_Pin,PowerCustList.PowerCustInfo[3].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(PowerCustList.PowerCustInfo[3].Gpio_Pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
			if(mt_set_gpio_out(PowerCustList.PowerCustInfo[3].Gpio_Pin,PowerCustList.PowerCustInfo[3].Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			
			if(PowerCustList.PowerCustInfo[4].Gpio_Pin != GPIO_UNSUPPORTED)
			{
				mdelay(5);
				if(mt_set_gpio_mode(PowerCustList.PowerCustInfo[3].Gpio_Pin,PowerCustList.PowerCustInfo[3].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(PowerCustList.PowerCustInfo[3].Gpio_Pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
				if(mt_set_gpio_out(PowerCustList.PowerCustInfo[3].Gpio_Pin,PowerCustList.PowerCustInfo[3].Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			}	
		}			
	}
#endif
	else if(pwInfo.PowerType==PDN)
	{
		PK_DBG("hwPowerOn: PDN %d \n",pwInfo.Voltage);
		
		if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], pwInfo.Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
	}
	else if(pwInfo.PowerType==RST)
	{
		PK_DBG("hwPowerOn: RST %d \n",pwInfo.Voltage);

		if(pinSetIdx==0)
		{
#ifndef MTK_MT6306_SUPPORT
        if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
        if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], pwInfo.Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#else
		if(mt6306_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
		if(mt6306_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pwInfo.Voltage)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");} 				 

#endif 
		}
		else if(pinSetIdx==1)
		{
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
	        if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
	        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], pwInfo.Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		}

		
	}
	else if(pwInfo.PowerType==SensorMCLK)
	{
		if(pinSetIdx==0)
		{
			PK_DBG("Sensor MCLK1 On");
			ISP_MCLK1_EN(TRUE);
		}
		else if(pinSetIdx==1)
		{
			PK_DBG("Sensor MCLK1 On");
			ISP_MCLK1_EN(TRUE);
		}
	}
	else{}
	if(pwInfo.Delay>0)
		mdelay(pwInfo.Delay);
	return TRUE;
}
	


BOOL hwpowerdown(PowerInformation pwInfo, char* mode_name)
{
	if(pwInfo.PowerType == AVDD)
	{
		if(PowerCustList.PowerCustInfo[0].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if(pinSetIdx==1)
			{
				if(TRUE != hwPowerDown(SUB_CAMERA_POWER_VCAM_A,mode_name))
       	 		{
            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            		return FALSE;
        		}
#ifdef GPIO_CAMERA_SUB_AVDD_EN_PIN
				if(mt_set_gpio_mode(GPIO_CAMERA_SUB_AVDD_EN_PIN,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(GPIO_CAMERA_SUB_AVDD_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");} 					
				if(mt_set_gpio_out(GPIO_CAMERA_SUB_AVDD_EN_PIN,Vol_Low)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#endif			
			}
			else
			{
#ifdef GPIO_CAMERA_MIAN_AVDD_EN_PIN
				if(mt_set_gpio_mode(GPIO_CAMERA_MIAN_AVDD_EN_PIN,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(GPIO_CAMERA_MIAN_AVDD_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}					
				if(mt_set_gpio_out(GPIO_CAMERA_MIAN_AVDD_EN_PIN,Vol_Low)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#endif					
			if(TRUE != hwPowerDown(pwInfo.PowerType,mode_name))
       	 	{
            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            		return FALSE;
        	}
			else{}
			}
		}
		else{
			if(mt_set_gpio_mode(PowerCustList.PowerCustInfo[0].Gpio_Pin,PowerCustList.PowerCustInfo[0].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(PowerCustList.PowerCustInfo[0].Gpio_Pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
			if(mt_set_gpio_out(PowerCustList.PowerCustInfo[0].Gpio_Pin,PowerCustList.PowerCustInfo[0].Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			}			
	}
	else if(pwInfo.PowerType == DVDD)
	{
		if(PowerCustList.PowerCustInfo[1].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if(pinSetIdx==1)
			{
				if(TRUE != hwPowerDown(SUB_CAMERA_POWER_VCAM_D,mode_name))
       	 		{
            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            		return FALSE;
        		}
#ifdef GPIO_CAMERA_SUB_DVDD_EN_PIN
				if(mt_set_gpio_mode(GPIO_CAMERA_SUB_DVDD_EN_PIN,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(GPIO_CAMERA_SUB_DVDD_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}					
				if(mt_set_gpio_out(GPIO_CAMERA_SUB_DVDD_EN_PIN,Vol_Low)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#endif									
			}
			else
			{
#ifdef GPIO_CAMERA_MIAN_DVDD_EN_PIN
				if(mt_set_gpio_mode(GPIO_CAMERA_MIAN_DVDD_EN_PIN,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(GPIO_CAMERA_MIAN_DVDD_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}					
				if(mt_set_gpio_out(GPIO_CAMERA_MIAN_DVDD_EN_PIN,Vol_Low)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#endif									
#ifdef GPIO_CAMERA_IMX135_DVDD_EN_PIN
				if(mt_set_gpio_mode(GPIO_CAMERA_IMX135_DVDD_EN_PIN,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(GPIO_CAMERA_IMX135_DVDD_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
				if(mt_set_gpio_out(GPIO_CAMERA_IMX135_DVDD_EN_PIN,Vol_Low)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#endif
				if(TRUE != hwPowerDown(pwInfo.PowerType,mode_name))
       	 	{
            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            		return FALSE;
        	}
			else{}
			}
		}
		else{
			if(mt_set_gpio_mode(PowerCustList.PowerCustInfo[1].Gpio_Pin,PowerCustList.PowerCustInfo[1].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(PowerCustList.PowerCustInfo[1].Gpio_Pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
			if(mt_set_gpio_out(PowerCustList.PowerCustInfo[1].Gpio_Pin,PowerCustList.PowerCustInfo[1].Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			}			
	}
	else if(pwInfo.PowerType == DOVDD)
	{
		if(PowerCustList.PowerCustInfo[2].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if(TRUE != hwPowerDown(pwInfo.PowerType,mode_name))
       	 	{
            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            		return FALSE;
        	}
		}
		else{
			if(mt_set_gpio_mode(PowerCustList.PowerCustInfo[2].Gpio_Pin,PowerCustList.PowerCustInfo[2].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(PowerCustList.PowerCustInfo[2].Gpio_Pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
			if(mt_set_gpio_out(PowerCustList.PowerCustInfo[2].Gpio_Pin,PowerCustList.PowerCustInfo[2].Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			}			
	}
#ifndef CONFIG_HCT_AF_POWER_ON_NOISE
	else if(pwInfo.PowerType == AFVDD)
	{
		if(PowerCustList.PowerCustInfo[3].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if(TRUE != hwPowerDown(pwInfo.PowerType,mode_name))
       	 	{
            		PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            		return FALSE;
        	}
		}
		else{
			if(mt_set_gpio_mode(PowerCustList.PowerCustInfo[3].Gpio_Pin,PowerCustList.PowerCustInfo[3].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(PowerCustList.PowerCustInfo[3].Gpio_Pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
			if(mt_set_gpio_out(PowerCustList.PowerCustInfo[3].Gpio_Pin,PowerCustList.PowerCustInfo[3].Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			
			if(PowerCustList.PowerCustInfo[4].Gpio_Pin != GPIO_UNSUPPORTED)
			{
				mdelay(5);
				if(mt_set_gpio_mode(PowerCustList.PowerCustInfo[3].Gpio_Pin,PowerCustList.PowerCustInfo[3].Gpio_Mode)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(PowerCustList.PowerCustInfo[3].Gpio_Pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}						
				if(mt_set_gpio_out(PowerCustList.PowerCustInfo[3].Gpio_Pin,PowerCustList.PowerCustInfo[3].Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			}	
		}			
	}
#endif
	else if(pwInfo.PowerType==PDN)
	{
		PK_DBG("hwPowerDown: PDN %d \n",pwInfo.Voltage);

		if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], pwInfo.Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		msleep(1);
	}
	else if(pwInfo.PowerType==RST)
	{
		PK_DBG("hwPowerDown: RST %d \n",pwInfo.Voltage);
		if(pinSetIdx==0)
		{
#ifndef MTK_MT6306_SUPPORT
		if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
		if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], pwInfo.Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
#else
		if(mt6306_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
		if(mt6306_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], pwInfo.Voltage)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}				 
#endif 
		}
		else if(pinSetIdx==1)
		{
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], pwInfo.Voltage)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		}

	}
	else if(pwInfo.PowerType==SensorMCLK)
	{
		if(pinSetIdx==0)
		{
			ISP_MCLK1_EN(FALSE);
		}
		else if(pinSetIdx==1)
		{
			ISP_MCLK1_EN(FALSE);
		}
	}
	else{}
	return TRUE;
}




int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{

	int pwListIdx,pwIdx;
    BOOL sensorInPowerList = KAL_FALSE;

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }
	
    //power ON
    if (On) {
		PK_DBG("kdCISModulePowerOn -on:currSensorName=%s\n",currSensorName);
		PK_DBG("kdCISModulePowerOn -on:pinSetIdx=%d\n",pinSetIdx);
		#if defined(CONFIG_T925C_LG) || defined(CONFIG_T925U_YS_T925) || defined(CONFIG_T825_PROJ) || defined(CONFIG_T985D_HD_PROJ) || defined(OV9760_MIPI_RAW)
			config_subsensor_rst_pwd_when_nosub(currSensorName);
		#endif
		for(pwListIdx=0 ; pwListIdx<16; pwListIdx++)
		{
			if(currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName!=NULL) && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
			{
				PK_DBG("kdCISModulePowerOn get in--- \n");
				PK_DBG("sensorIdx:%d \n",SensorIdx);

                sensorInPowerList = KAL_TRUE;

				for(pwIdx=0;pwIdx<10;pwIdx++)
				{  
					if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType != VDD_None)
					{
						if(hwpoweron(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx],mode_name)==FALSE)
							goto _kdCISModulePowerOn_exit_;
					}					
					else
					{
						PK_DBG("pwIdx=%d \n",pwIdx);
						break;
					}
				}
				break;
			}
			else if(PowerOnList.PowerSeq[pwListIdx].SensorName == NULL)
			{	
				break;
			}
			else{}
		}

        // Temp solution: default power on/off sequence
        if(KAL_FALSE == sensorInPowerList)
        {
            PK_DBG("Default power on sequence");
            
            if(pinSetIdx == 0 ) {
                ISP_MCLK1_EN(1);
            }
            else if (pinSetIdx == 1) {
                ISP_MCLK1_EN(1);
            }

            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(0 == pinSetIdx) {
#ifndef MTK_MT6306_SUPPORT
                    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
#else
                    if(mt6306_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                    if(mt6306_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}                                   
#endif                    
                }
                else {
                    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
                }                
            }

            //VCAM_IO
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_A
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }
#ifndef CONFIG_HCT_AF_POWER_ON_NOISE
             //AF_VCC
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(5);
#endif
            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            mdelay(1);

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                 if(0 == pinSetIdx) {
#ifndef MTK_MT6306_SUPPORT
                    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
#else
                    if(mt6306_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                    if(mt6306_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}                
#endif 

                }
                else {  
                    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}         
                }
            }

            
        

        }
		/*
		if(pinSetIdx==0)
			for(;;)
				{}
		*/
		/*
		if(pinSetIdx==1)
			for(;;)
				{}
 		*/
		}
    else {//power OFF
	for(pwListIdx=0 ; pwListIdx<16; pwListIdx++)
		{
			if(currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName!=NULL) && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
			{
				PK_DBG("kdCISModulePowerOn get in--- \n");
				PK_DBG("sensorIdx:%d \n",SensorIdx);

                sensorInPowerList = KAL_TRUE;

				for(pwIdx=9;pwIdx>=0;pwIdx--)
				{  
					if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType != VDD_None)
					{
						if(hwpowerdown(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx],mode_name)==FALSE)
							goto _kdCISModulePowerOn_exit_;
						if(pwIdx>0)
						{
							if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx-1].Delay > 0)
								mdelay(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx-1].Delay);
						}
					}					
					else
					{
						PK_DBG("pwIdx=%d \n",pwIdx);
					}
				}
			}
			else if(PowerOnList.PowerSeq[pwListIdx].SensorName == NULL)
			{	
				break;
			}
			else{}
		}

        // Temp solution: default power on/off sequence
        if(KAL_FALSE == sensorInPowerList)
        {
            PK_DBG("Default power off sequence");
            
            if(pinSetIdx == 0 ) {
                ISP_MCLK1_EN(0);
            }
            else if (pinSetIdx == 1) {
                ISP_MCLK1_EN(0);
            }

            //Set Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(0 == pinSetIdx) {
#ifndef MTK_MT6306_SUPPORT
                    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
#else
                    if(mt6306_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                    if(mt6306_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}                  
#endif
                }
                else {
                    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}               
                }               
            }
            

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_A
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_IO
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#ifndef CONFIG_HCT_AF_POWER_ON_NOISE
            //AF_VCC
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#endif
        }
	config_sensor_rst_pwd(currSensorName);
    }//

	return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);
