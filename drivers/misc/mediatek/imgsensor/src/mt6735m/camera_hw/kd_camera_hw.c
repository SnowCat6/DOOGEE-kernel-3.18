/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include "kd_camera_hw.h"
#include "tb_kd_camera_hw.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG(fmt, arg...)			pr_err(PFX fmt, ##arg)
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...) 		pr_err(PFX fmt, ##arg)
#else
#define PK_DBG(fmt, arg...)			pr_debug(PFX fmt, ##arg)
#define PK_ERR(fmt, arg...)			pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...)		pr_debug(PFX fmt, ##arg)
#endif


#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4


extern void ISP_MCLK1_EN(BOOL En);

u32 pinSetIdx = 0;		/* default main sensor */
u32 pinSet[3][8] = {
	/* for main sensor */
	{CAMERA_CMRST_PIN,
	 CAMERA_CMRST_PIN_M_GPIO,	/* mode */
	 GPIO_OUT_ONE,		/* ON state */
	 GPIO_OUT_ZERO,		/* OFF state */
	 CAMERA_CMPDN_PIN,
	 CAMERA_CMPDN_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 },
	/* for sub sensor */
	{CAMERA_CMRST1_PIN,
	 CAMERA_CMRST1_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 CAMERA_CMPDN1_PIN,
	 CAMERA_CMPDN1_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 },
	/* for Main2 sensor */
	{CAMERA_CMRST2_PIN,
	 CAMERA_CMRST2_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 CAMERA_CMPDN2_PIN,
	 CAMERA_CMPDN2_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 },
};
#ifndef CONFIG_MTK_LEGACY
#define CUST_AVDD AVDD - AVDD
#define CUST_DVDD DVDD - AVDD
#define CUST_DOVDD DOVDD - AVDD
#define CUST_AFVDD AFVDD - AVDD
#define CUST_SUB_AVDD SUB_AVDD - AVDD
#define CUST_SUB_DVDD SUB_DVDD - AVDD
#define CUST_SUB_DOVDD SUB_DOVDD - AVDD
#define CUST_MAIN2_AVDD MAIN2_AVDD - AVDD
#define CUST_MAIN2_DVDD MAIN2_DVDD - AVDD
#define CUST_MAIN2_DOVDD MAIN2_DVDD - AVDD

#endif


PowerCust PowerCustList = {
	{
	 //{GPIO_SUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for AVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for DVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for DVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for DOVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for AFVDD; */
	 //{GPIO_SUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for SUB_AVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for SUB_DVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for SUB_DVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for SUB_DOVDD; */
	 }
};



PowerUp PowerOnList = {
	{
#if defined(S5K3M2_MIPI_RAW)
	  {SENSOR_DRVNAME_S5K3M2_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {DOVDD, Vol_1800, 0},
	   {AVDD, Vol_2800, 0},
	   {DVDD, Vol_1200, 0},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 4},
	   {PDN, Vol_High, 0},
	   {RST, Vol_Low, 1},
	   {RST, Vol_High, 0},
	   },
	  },
#endif
#if defined(S5K3L2_MIPI_RAW)
      {SENSOR_DRVNAME_S5K3L2_MIPI_RAW,
      {    
       {SensorMCLK, Vol_High, 0},
       {DOVDD, Vol_1800, 0},
       {AVDD, Vol_2800, 0},
       {DVDD, Vol_1200, 0},
       {AFVDD, Vol_2800, 5},
       {PDN, Vol_Low, 4},
       {PDN, Vol_High, 0},
       {RST, Vol_Low, 1},
       {RST, Vol_High, 0},
       },   
      },   
#endif
#if defined(S5K4H8_MIPI_RAW)
      {SENSOR_DRVNAME_S5K4H8_MIPI_RAW,
      {    
       {SensorMCLK, Vol_High, 0},
       {DOVDD, Vol_1800, 0},
       {AVDD, Vol_2800, 0},
       {DVDD, Vol_1200, 0},
       {AFVDD, Vol_2800, 5},
       {PDN, Vol_Low, 4},
       {PDN, Vol_High, 0},
       {RST, Vol_Low, 1},
       {RST, Vol_High, 0},
       },   
      },   
#endif
#if defined(S5K5E2YA_MIPI_RAW)
	 {SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {DOVDD, Vol_1800, 0},
	   {AVDD, Vol_2800, 0},
	   {DVDD, Vol_1200, 0},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 4},
	   {PDN, Vol_High, 0},
	   {RST, Vol_Low, 1},
	   {RST, Vol_High, 0},
	   },
	  },
#endif
#if defined(S5K2P8_MIPI_RAW)
	 {SENSOR_DRVNAME_S5K2P8_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {DOVDD, Vol_1800, 0},
	   {AVDD, Vol_2800, 0},
	   {DVDD, Vol_1200, 0},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 4},
	   {PDN, Vol_High, 0},
	   {RST, Vol_Low, 1},
	   {RST, Vol_High, 0},
	   },
	  },
#endif
#if defined(S5K3H5XA_MIPI_RAW)
   {SENSOR_DRVNAME_S5K3H5XA_MIPI_RAW,
     {
       {SensorMCLK, Vol_High, 0},
       {DOVDD, Vol_1800, 0},
       {AVDD, Vol_2800, 0},
       {DVDD, Vol_1200, 0},
       {AFVDD, Vol_2800, 5},
       {PDN, Vol_Low, 4},
       {PDN, Vol_High, 0},
       {RST, Vol_Low, 1},
       {RST, Vol_High, 0},
     },
   },
#endif
#if defined(S5K3H5XASUB_MIPI_RAW)
   {SENSOR_DRVNAME_S5K3H5XASUB_MIPI_RAW,
     {
       {SensorMCLK, Vol_High, 0},
       {DOVDD, Vol_1800, 0},
       {AVDD, Vol_2800, 0},
       {DVDD, Vol_1200, 0},
       {AFVDD, Vol_2800, 5},
       {PDN, Vol_Low, 4},
       {PDN, Vol_High, 0},
       {RST, Vol_Low, 1},
       {RST, Vol_High, 0},
     },
   },
#endif
#if defined(HI553_MIPI_RAW)
       {SENSOR_DRVNAME_HI553_MIPI_RAW,
       {
        {SensorMCLK, Vol_High, 0},
        {PDN, Vol_Low, 5},
        {RST, Vol_Low, 5},
        {DOVDD, Vol_1800, 1},
        {AVDD, Vol_2800, 1},
        {DVDD, Vol_1200, 1},
        {AFVDD, Vol_2800, 5},
        {PDN, Vol_High, 5},
        {RST, Vol_High, 5}
        },
       },
#endif

#if defined(IMX214_MIPI_RAW)
	  {SENSOR_DRVNAME_IMX214_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {PDN, Vol_High, 5},
	   {DOVDD, Vol_1800, 10},
	   {AVDD, Vol_2800, 10},
	   {DVDD, Vol_1000, 10},
	   {AFVDD, Vol_2800, 5},
	   {RST, Vol_High, 5},
	   {RST, Vol_Low, 0}
	   },
	  },
#endif
#if defined(MN045_MIPI_RAW)
	  {SENSOR_DRVNAME_MN045_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1000, 10},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 0},
	   {PDN, Vol_High, 0},
	   {RST, Vol_Low, 0},
	   {RST, Vol_High, 0}
	   },
	  },
#endif
#if defined(IMX166_MIPI_RAW)
  {SENSOR_DRVNAME_IMX166_MIPI_RAW,
    {
      {SensorMCLK, Vol_High, 0},
      {PDN, Vol_Low, 5},
      {RST, Vol_Low, 5},
      {DOVDD, Vol_1800, 1},
      {AVDD, Vol_2800, 1},
      {DVDD, Vol_1500, 1},
      {AFVDD, Vol_2800, 5},
      {PDN, Vol_High, 5},
      {RST, Vol_High, 5}
    },
  },
#endif
#if defined(IMX219_MIPI_RAW)
	  {SENSOR_DRVNAME_IMX219_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1200, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_High, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(IMX149_MIPI_RAW)
	  {SENSOR_DRVNAME_IMX149_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1200, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_High, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(IMX175_MIPI_RAW)
	  {SENSOR_DRVNAME_IMX175_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1200, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_High, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(HI542MIPI_SENSOR_ID)
	  {SENSOR_DRVNAME_HI542MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1200, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_High, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(HI544_MIPI_RAW)
	  {SENSOR_DRVNAME_HI544_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1200, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_High, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(HI545_MIPI_RAW)
	  {SENSOR_DRVNAME_HI545_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1200, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_High, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(HI551_MIPI_RAW)
	  {SENSOR_DRVNAME_HI551_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1200, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_High, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(HI841_MIPI_RAW)
      {SENSOR_DRVNAME_HI841_MIPI_RAW,
      {
       {SensorMCLK, Vol_High, 0},
       {PDN, Vol_High, 5},
       {RST, Vol_Low, 5},
       {DOVDD, Vol_1800, 1},
       {AVDD, Vol_2800, 1},
       {DVDD, Vol_1200, 1},
       {AFVDD, Vol_2800, 5},
       {PDN, Vol_Low, 5},
       {RST, Vol_High, 5}
       },
      },
#endif
#if defined(HI843B_MIPI_RAW)
	  {SENSOR_DRVNAME_HI843B_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1200, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_High, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif

#if defined(IMX164_MIPI_RAW)
	  {SENSOR_DRVNAME_IMX164_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1500, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_High, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(OV5648_MIPI_RAW)
	  {SENSOR_DRVNAME_OV5648_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 0},
	   {RST, Vol_Low, 0},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1500, 10},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_High, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(OV8858_MIPI_RAW)
  {SENSOR_DRVNAME_OV8858_MIPI_RAW,
    {
      {SensorMCLK, Vol_High, 0},
      {PDN, Vol_Low, 0},
      {RST, Vol_Low, 0},
      {AVDD, Vol_2800, 10},
      {DOVDD, Vol_1800, 10},
      {DVDD, Vol_1200, 10},
	  {AFVDD, Vol_2800, 5},
      {PDN, Vol_High, 5},
      {RST, Vol_High, 5}
    },
  },
#endif
#if defined(OV8865_MIPI_RAW)
  {SENSOR_DRVNAME_OV8865_MIPI_RAW,
    {
      {SensorMCLK, Vol_High, 0},
      {PDN, Vol_Low, 0},
      {RST, Vol_Low, 0},
      {AVDD, Vol_2800, 10},
      {DOVDD, Vol_1800, 10},
      {DVDD, Vol_1200, 10},
	   {AFVDD, Vol_2800, 5},
      {PDN, Vol_High, 5},
      {RST, Vol_High, 5}
    },
  },
#endif
#if defined(OV8856_MIPI_RAW)
  {SENSOR_DRVNAME_OV8856_MIPI_RAW,
    {
      {SensorMCLK, Vol_High, 0},
      {PDN, Vol_Low, 0},
      {RST, Vol_Low, 0},
      {AVDD, Vol_2800, 10},
      {DOVDD, Vol_1800, 10},
      {DVDD, Vol_1200, 10},
      {AFVDD, Vol_2800, 5},
      {PDN, Vol_High, 5},
      {RST, Vol_High, 5}
    },
  },
#endif
#if defined(OV2680_MIPI_RAW)
  {SENSOR_DRVNAME_OV2680_MIPI_RAW,
    {
      {SensorMCLK, Vol_High, 0},
      {PDN, Vol_Low, 0},
      {RST, Vol_Low, 0},
      {AVDD, Vol_2800, 10},
      {DOVDD, Vol_1800, 10},
      {DVDD, Vol_1200, 10},
      {PDN, Vol_High, 5},
      {RST, Vol_High, 5}
    },
  },
#endif
#if defined(OV13850_MIPI_RAW)
   {SENSOR_DRVNAME_OV13850_MIPI_RAW,
     {
       {SensorMCLK,Vol_High, 0},
       {PDN, Vol_Low, 0},
       {RST, Vol_Low, 0},
       {AVDD,    Vol_2800, 10},
       {DOVDD, Vol_1800, 10},
       {DVDD,    Vol_1200, 10},
       {AFVDD, Vol_2800, 5},
       {PDN, Vol_High, 0},
       {RST, Vol_High, 0}
     },
   },
#endif

#if defined(SP8408_MIPI_RAW)
	  {SENSOR_DRVNAME_SP8408_MIPI_RAW,
	  {
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1200, 10},
	   {AVDD, Vol_2800, 10},
	   {AFVDD, Vol_2800, 5},
	   {SensorMCLK, Vol_High, 10},
	   {PDN, Vol_High, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(SP5409_MIPI_RAW)
	  {SENSOR_DRVNAME_SP5409_MIPI_RAW,
	  {
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1500, 10},
	   {AVDD, Vol_2800, 10},
	   {AFVDD, Vol_2800, 5},
	   {SensorMCLK, Vol_High, 10},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(SP2509_MIPI_RAW)
	  {SENSOR_DRVNAME_SP2509_MIPI_RAW,
	  {
	   {PDN, Vol_High, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1500, 10},
	   {AVDD, Vol_2800, 10},
	   {SensorMCLK, Vol_High, 10},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(SP2509SUB_MIPI_RAW)
	  {SENSOR_DRVNAME_SP2509SUB_MIPI_RAW,
	  {
	   {PDN, Vol_High, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1500, 10},
	   {AVDD, Vol_2800, 10},
	   {SensorMCLK, Vol_High, 10},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(SP250A_MIPI_RAW)
	  {SENSOR_DRVNAME_SP250A_MIPI_RAW,
	  {
	   {PDN, Vol_High, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1500, 10},
	   {AVDD, Vol_2800, 10},
	   {SensorMCLK, Vol_High, 10},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(SP2508_MIPI_RAW)
	  {SENSOR_DRVNAME_SP2508_MIPI_RAW,
	  {
	   {PDN, Vol_High, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1500, 10},
	   {AVDD, Vol_2800, 10},
	   {SensorMCLK, Vol_High, 10},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(SP0A09_MIPI_RAW)
	  {SENSOR_DRVNAME_SP0A09_MIPI_RAW,
	  {
	   {PDN, Vol_High, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1500, 10},
	   {AVDD, Vol_2800, 10},
	   {SensorMCLK, Vol_High, 10},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC5024_MIPI_RAW)
	  {SENSOR_DRVNAME_GC5024_MIPI_RAW,
	  {
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1500, 10},
	   {AVDD, Vol_2800, 10},
       {AFVDD, Vol_2800, 5},
	   {SensorMCLK, Vol_High, 10},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC2355_MIPI_RAW)
	  {SENSOR_DRVNAME_GC2355_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
      {DVDD, Vol_1800, 10},
      {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC8024_MIPI_RAW)
	  {SENSOR_DRVNAME_GC8024_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
      {DVDD, Vol_1200, 10},
	   {AFVDD, Vol_2800, 10},
      {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC13003_MIPI_RAW)
	  {SENSOR_DRVNAME_GC13003MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_High, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 10},
       {DVDD, Vol_1200, 10},
	   {AVDD, Vol_2800, 10},
	   {AFVDD, Vol_2800, 10},
       {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC2145_MIPI_YUV)
	  {SENSOR_DRVNAME_GC2145_MIPI_YUV,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
      {DVDD, Vol_1800, 10},
      {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC5005_MIPI_RAW)
	  {SENSOR_DRVNAME_GC5005_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
       {DVDD, Vol_1200, 10},
       {AFVDD, Vol_2800,10 },
       {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC5005SUB_MIPI_RAW)
	  {SENSOR_DRVNAME_GC5005SUB_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
       {DVDD, Vol_1200, 10},
       {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif


#if defined(OV5670_MIPI_RAW)
  {SENSOR_DRVNAME_OV5670_MIPI_RAW,
    {
      {SensorMCLK,Vol_High, 0},
      {PDN,	Vol_Low, 0},
      {RST,	Vol_Low, 0}, 
      {AVDD,	Vol_2800, 10},
      {DOVDD, Vol_1800, 10},
      {DVDD,	Vol_1200, 10},
      {AFVDD, Vol_2800, 5},
      {PDN,	Vol_High, 0},
      {RST,	Vol_High, 0}
    },
  },      
#endif
#if defined(OV5675_MIPI_RAW)
  {SENSOR_DRVNAME_OV5675_MIPI_RAW,
    {
      {SensorMCLK,Vol_High, 0},
      {PDN,	Vol_Low, 0},
      {RST,	Vol_Low, 0}, 
      {AVDD,	Vol_2800, 10},
      {DOVDD, Vol_1800, 10},
      {DVDD,	Vol_1200, 10},
      {AFVDD, Vol_2800, 5},
      {PDN,	Vol_High, 0},
      {RST,	Vol_High, 0}
    },
  },      
#endif
#if defined(OV5675SUB_MIPI_RAW)
  {SENSOR_DRVNAME_OV5675SUB_MIPI_RAW,
    {
      {SensorMCLK,Vol_High, 0},
      {PDN,	Vol_Low, 0},
      {RST,	Vol_Low, 0}, 
      {AVDD,	Vol_2800, 10},
      {DOVDD, Vol_1800, 10},
      {DVDD,	Vol_1200, 10},
      {AFVDD, Vol_2800, 5},
      {PDN,	Vol_High, 0},
      {RST,	Vol_High, 0}
    },
  },      
#endif
#if defined(OV5693_MIPI_RAW)
  {SENSOR_DRVNAME_OV5693_MIPI_RAW,
    {
      {SensorMCLK,Vol_High, 0},
      {PDN,	Vol_Low, 0},
      {RST,	Vol_Low, 0}, 
      {AVDD,	Vol_2800, 10},
      {DOVDD, Vol_1800, 10},
      {DVDD,	Vol_1200, 10},
      {AFVDD, Vol_2800, 5},
      {PDN,	Vol_High, 0},
      {RST,	Vol_High, 0}
    },
  },      
#endif
#if defined(OV9762_MIPI_RAW)
  {SENSOR_DRVNAME_OV9762_MIPI_RAW,
    {
      {SensorMCLK,Vol_High, 0},
      {PDN,	Vol_Low, 0},
      {RST,	Vol_Low, 0}, 
      {AVDD,	Vol_2800, 10},
      {DOVDD, Vol_1800, 10},
      {DVDD,	Vol_1500, 10},
      {AFVDD, Vol_2800, 5},
      {PDN,	Vol_High, 0},
      {RST,	Vol_High, 0}
    },
  },      
#endif
#if defined(OV9760_MIPI_RAW)
  {SENSOR_DRVNAME_OV9760_MIPI_RAW,
    {
      {SensorMCLK,Vol_High, 0},
      {PDN,	Vol_Low, 0},
      {RST,	Vol_Low, 0}, 
      {AVDD,	Vol_2800, 10},
      {DOVDD, Vol_1800, 10},
      {DVDD,	Vol_1500, 10},
      {AFVDD, Vol_2800, 5},
      {PDN,	Vol_High, 0},
      {RST,	Vol_High, 0}
    },
  },      
#endif
#if defined(OV9762SUB_MIPI_RAW)
  {SENSOR_DRVNAME_OV9762SUB_MIPI_RAW,
    {
      {SensorMCLK,Vol_High, 0},
      {PDN,	Vol_Low, 0},
      {RST,	Vol_Low, 0}, 
      {AVDD,	Vol_2800, 10},
      {DOVDD, Vol_1800, 10},
      {DVDD,	Vol_1500, 10},
      {AFVDD, Vol_2800, 5},
      {PDN,	Vol_High, 0},
      {RST,	Vol_High, 0}
    },
  },      
#endif

#if defined(GC2365_MIPI_RAW)
	  {SENSOR_DRVNAME_GC2365_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_High, 5},
	   {RST, Vol_Low, 5},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
       {DVDD, Vol_1800, 10},
       {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC2365SUB_MIPI_RAW)
	  {SENSOR_DRVNAME_GC2365SUB_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_High, 5},
	   {RST, Vol_Low, 5},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
       {DVDD, Vol_1800, 10},
       {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC2375_MIPI_RAW)
	  {SENSOR_DRVNAME_GC2375_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_High, 5},
	   {RST, Vol_Low, 5},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
       {DVDD, Vol_1800, 10},
       {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC0409_MIPI_RAW)
	  {SENSOR_DRVNAME_GC0409MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
       {DVDD, Vol_1800, 10},
       {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(JX507_MIPI_RAW)
	  {SENSOR_DRVNAME_JX507_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_High, 5},
	   {RST, Vol_Low, 5},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
       {DVDD, Vol_1500, 10},
	   {AFVDD, Vol_2800, 10},
       {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC2755_MIPI_RAW)
	  {SENSOR_DRVNAME_GC2755_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {DOVDD, Vol_1800, 10},
       {DVDD, Vol_1800, 10},
	   {AVDD, Vol_2800, 10},       
       {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
#if defined(GC030A_MIPI_RAW)
	  {SENSOR_DRVNAME_GC030A_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 5},
	   {RST, Vol_Low, 5},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
       {DVDD, Vol_1200, 10},
       {PDN, Vol_Low, 5},
	   {RST, Vol_High, 5}
	   },
	  },
#endif
	 /* add new sensor before this line */
	 {NULL,},
	 }
};


#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1220 1220000
#define VOL_1000 1000000

/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;/* main cam */
struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;/* sub cam */
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam_ldo_vcama_h = NULL;/* for AVDD */
struct pinctrl_state *cam_ldo_vcama_l = NULL;
struct pinctrl_state *cam_ldo_sub_vcama_h = NULL;/* for SUB_DVDD */
struct pinctrl_state *cam_ldo_sub_vcama_l = NULL;
struct pinctrl_state *cam_af_pdn_h = NULL;
struct pinctrl_state *cam_af_pdn_l = NULL;


int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}
	/*Cam0 Power/Rst Ping initialization */
	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h)) {
		ret = PTR_ERR(cam0_pnd_h);
		PK_ERR("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}

	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l)) {
		ret = PTR_ERR(cam0_pnd_l);
		PK_ERR("%s : pinctrl err, cam0_pnd_l\n", __func__);
	}


	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		PK_ERR("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		PK_ERR("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

	/*Cam1 Power/Rst Ping initialization */
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h)) {
		ret = PTR_ERR(cam1_pnd_h);
		PK_ERR("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}

	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l)) {
		ret = PTR_ERR(cam1_pnd_l);
		PK_ERR("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}


	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		PK_ERR("%s : pinctrl err, cam1_rst_h\n", __func__);
	}


	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		PK_ERR("%s : pinctrl err, cam1_rst_l\n", __func__);
	}

	/*externel LDO enable */
	/*GPIO 253*/
	cam_ldo_vcama_h = pinctrl_lookup_state(camctrl, "cam_ldo_vcama_1");
	if (IS_ERR(cam_ldo_vcama_h)) {
		ret = PTR_ERR(cam_ldo_vcama_h);
		PK_ERR("%s : pinctrl err, cam_ldo_vcama_h\n", __func__);
	}

	cam_ldo_vcama_l = pinctrl_lookup_state(camctrl, "cam_ldo_vcama_0");
	if (IS_ERR(cam_ldo_vcama_l)) {
		ret = PTR_ERR(cam_ldo_vcama_l);
		PK_ERR("%s : pinctrl err, cam_ldo_vcama_l\n", __func__);
	}
    /*GPIO 110*/
	cam_ldo_sub_vcama_h= pinctrl_lookup_state(camctrl, "cam_ldo_sub_vcamd_1");
	if (IS_ERR(cam_ldo_sub_vcama_h)) {
		ret = PTR_ERR(cam_ldo_sub_vcama_h);
		PK_ERR("%s : pinctrl err, cam_ldo_sub_vcama_h\n", __func__);
	}

	cam_ldo_sub_vcama_l= pinctrl_lookup_state(camctrl, "cam_ldo_sub_vcamd_0");
	if (IS_ERR(cam_ldo_sub_vcama_l)) {
		ret = PTR_ERR(cam_ldo_sub_vcama_l);
		PK_ERR("%s : pinctrl err, cam_ldo_sub_vcama_l\n", __func__);
	}

    /*GPIO 110*/
	cam_af_pdn_h= pinctrl_lookup_state(camctrl, "cam_af_pdn_1");
	if (IS_ERR(cam_af_pdn_h)) {
		ret = PTR_ERR(cam_af_pdn_h);
		PK_ERR("%s : pinctrl err, cam_af_pdn_h\n", __func__);
	}

	cam_af_pdn_l= pinctrl_lookup_state(camctrl, "cam_af_pdn_0");
	if (IS_ERR(cam_af_pdn_l)) {
		ret = PTR_ERR(cam_af_pdn_l);
		PK_ERR("%s : pinctrl err, cam_af_pdn_l\n", __func__);
	}    
	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;
	static signed int mAVDD_usercounter = 0;
	static signed int subAVDD_usercounter = 0;

	if (IS_ERR(camctrl)) {
		return -1;
	}
	
	switch (PwrType) {
	case RST:
		if (PinIdx == 0) {
			if (Val == 0 && !IS_ERR(cam0_rst_l))
				pinctrl_select_state(camctrl, cam0_rst_l);
			else if (Val == 1 && !IS_ERR(cam0_rst_h))
				pinctrl_select_state(camctrl, cam0_rst_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx ,Val);
		} else if (PinIdx == 1) {
			if (Val == 0 && !IS_ERR(cam1_rst_l))
				pinctrl_select_state(camctrl, cam1_rst_l);
			else if (Val == 1 && !IS_ERR(cam1_rst_h))
				pinctrl_select_state(camctrl, cam1_rst_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx ,Val);
		} else {
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx ,Val);
		}
		break;
	case PDN:
		if (PinIdx == 0) {
			if (Val == 0 && !IS_ERR(cam0_pnd_l))
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else if (Val == 1 && !IS_ERR(cam0_pnd_h))
				pinctrl_select_state(camctrl, cam0_pnd_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);
		} else if (PinIdx == 1) {
			if (Val == 0 && !IS_ERR(cam1_pnd_l))
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else if (Val == 1 && !IS_ERR(cam1_pnd_h))
				pinctrl_select_state(camctrl, cam1_pnd_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);
		} else {
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);
		}
		break;
	case AVDD:
	case MAIN2_AVDD:
		/*Main & Main2 use same cotrol GPIO */
		PK_DBG("mAVDD_usercounter(%d)\n",mAVDD_usercounter);
		if (Val == 0 && !IS_ERR(cam_ldo_vcama_l)){
			mAVDD_usercounter --;
			if(mAVDD_usercounter <= 0)
			{
				if(mAVDD_usercounter < 0)
					PK_ERR("Please check AVDD pin control\n");

				mAVDD_usercounter = 0;
				pinctrl_select_state(camctrl, cam_ldo_vcama_l);
			}
			
		}
		else if (Val == 1 && !IS_ERR(cam_ldo_vcama_h)){
		        PK_DBG("-----ldo vcama enbale------\n");
			mAVDD_usercounter ++;
			pinctrl_select_state(camctrl, cam_ldo_vcama_h);
		}
		break;
	case DVDD:
	case MAIN2_DVDD:
	case DOVDD:
	case AFVDD:
		if (Val == 0 && !IS_ERR(cam_af_pdn_l))
			pinctrl_select_state(camctrl, cam_af_pdn_l);
		else if (Val == 1 && !IS_ERR(cam_af_pdn_h))
			pinctrl_select_state(camctrl, cam_af_pdn_h);
		else
			PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);  
        break;
        
	case SUB_AVDD:
		/*Main & Main2 use same cotrol GPIO */
		PK_DBG("subAVDD_usercounter(%d)\n",subAVDD_usercounter);
		if (Val == 0 && !IS_ERR(cam_ldo_sub_vcama_l)){
			subAVDD_usercounter --;
			if(subAVDD_usercounter <= 0)
			{
				if(subAVDD_usercounter < 0)
					PK_ERR("Please check subAVDD pin control\n");

				subAVDD_usercounter = 0;
				pinctrl_select_state(camctrl, cam_ldo_sub_vcama_l);
			}
			
		}
		else if (Val == 1 && !IS_ERR(cam_ldo_sub_vcama_h)){
		        PK_DBG("-----ldo vcama enbale------\n");
			subAVDD_usercounter ++;
			pinctrl_select_state(camctrl, cam_ldo_sub_vcama_h);
		}
		break;
	case SUB_DVDD:
	default:
		PK_ERR("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}

BOOL hwpoweron(PowerInformation pwInfo, char *mode_name)
{
    if (pwInfo.PowerType == AVDD) {
        if (pinSetIdx == 2) {
            if (PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
                if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage)) {
                    PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
                    return FALSE;
                }
            } else {
                if (mtkcam_gpio_set(pinSetIdx, MAIN2_AVDD, PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Voltage)) {
                    PK_INFO("[CAMERA CUST_AVDD] set gpio failed!!\n");
                }
            }
        }
        else if(pinSetIdx == 1){
            if (PowerCustList.PowerCustInfo[CUST_SUB_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
                if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage)) {
                    PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
                    return FALSE;
                }
                if (mtkcam_gpio_set(pinSetIdx, SUB_AVDD, Vol_High)) {
                    PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
                }
            }
        }
        else
        {
            if (PowerCustList.PowerCustInfo[CUST_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
                if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage))
                {
                    PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
                    return FALSE;
                }
                if (mtkcam_gpio_set(pinSetIdx, AVDD, Vol_High)) 
                {
                    PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
                    return FALSE;
                }

            }
        }

    } else if (pwInfo.PowerType == DVDD) {
        if (pinSetIdx == 2) {
            if (PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
                PK_DBG("[CAMERA SENSOR] Sub camera VCAM_D power on");
                /*vcamd: unsupportable voltage range: 1500000-1210000uV*/
                if (pwInfo.Voltage == Vol_1200) {
                    pwInfo.Voltage = Vol_1210;
                    //PK_INFO("[CAMERA SENSOR] Main2 camera VCAM_D power 1.2V to 1.21V\n");
                }
                if (TRUE != _hwPowerOn(DVDD, pwInfo.Voltage)) {
                    PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
                    return FALSE;
                }
            } else {
                if (mtkcam_gpio_set(pinSetIdx, MAIN2_DVDD, PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Voltage)) {
                    PK_ERR("[CAMERA CUST_MAIN2_DVDD] set gpio failed!!\n");
                }
            }
        } else if (pinSetIdx == 1) {
            if (PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
                PK_DBG("[CAMERA SENSOR] camera sub VCAM_D power on");
                if (TRUE != _hwPowerOn(SUB_DVDD, pwInfo.Voltage)) {
                    PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
                    return FALSE;
                }
            } else {
                if (mtkcam_gpio_set(pinSetIdx, SUB_DVDD, PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Voltage)) {
                    PK_ERR("[CAMERA CUST_SUB_DVDD] set gpio failed!!\n");
                }
            }
        } else {
            if (PowerCustList.PowerCustInfo[CUST_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
                PK_DBG("[CAMERA SENSOR] Main camera VCAM_D power on");
                if (TRUE != _hwPowerOn(DVDD, pwInfo.Voltage)) {
                    PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
                    return FALSE;
                }
            } else {
                if (mtkcam_gpio_set(pinSetIdx, DVDD, PowerCustList.PowerCustInfo[CUST_DVDD].Voltage)) {
                    PK_ERR("[CAMERA CUST_DVDD] set gpio failed!!\n");
                }
            }
        }
    } else if (pwInfo.PowerType == DOVDD) {
        PK_INFO("[CAMERA SENSOR]  DOVDD setting\n");
        if (PowerCustList.PowerCustInfo[CUST_DOVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
            if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage)) {
                PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
                return FALSE;
            }
        } else {
            if (mtkcam_gpio_set(pinSetIdx, DOVDD, PowerCustList.PowerCustInfo[CUST_DOVDD].Voltage)) {
                PK_ERR("[CAMERA CUST_DOVDD] set gpio failed!!\n");
            }

        }
    } else if (pwInfo.PowerType == AFVDD) {
        PK_INFO("[CAMERA SENSOR]  AFVDD setting\n");
        if (PowerCustList.PowerCustInfo[CUST_AFVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
            if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage)) {
                PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
                return FALSE;
            }
            
            if (mtkcam_gpio_set(pinSetIdx, AFVDD, Vol_High)) {
                PK_ERR("[CAMERA CUST_AF PWDN] set gpio failed!!\n");
            }            
        } else {
            if (mtkcam_gpio_set(pinSetIdx, AFVDD, PowerCustList.PowerCustInfo[CUST_AFVDD].Voltage)) {
                PK_ERR("[CAMERA CUST_DOVDD] set gpio failed!!\n");
            }
        }
    } else if (pwInfo.PowerType == PDN) {
        PK_INFO("hwPowerOn: PDN %d\n", pwInfo.Voltage);
        /*mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);*/
        if (pwInfo.Voltage == Vol_High) {
            if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON])) {
                PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
            }
        } else {
            if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {

                PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
            }
        }
    } else if (pwInfo.PowerType == RST) {
        /*mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);*/
        PK_INFO("hwPowerOn: RST %d\n", pwInfo.Voltage);
        if (pwInfo.Voltage == Vol_High) {
            if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
                PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
            }
        } else {
            if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {

                PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
            }
        }

    } else if (pwInfo.PowerType == SensorMCLK) {
        if (pinSetIdx == 0) {
            PK_INFO("Sensor MCLK1 On");
            ISP_MCLK1_EN(TRUE);
        } else if (pinSetIdx == 1) {
            PK_INFO("Sensor MCLK2 On");
            ISP_MCLK1_EN(TRUE);
        } else {
            /* PK_INFO("Sensor MCLK3 On"); */
        }
    } else {
    }
    if (pwInfo.Delay > 0)
        mdelay(pwInfo.Delay);
    return TRUE;
}



BOOL hwpowerdown(PowerInformation pwInfo, char *mode_name)
{
	if (pwInfo.PowerType == AVDD) {
		if (pinSetIdx == 2) {
			if (PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (TRUE != _hwPowerDown(pwInfo.PowerType)) {
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, MAIN2_AVDD, 1-PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Voltage)) {
						PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		}
		else if(pinSetIdx == 1){
			if (PowerCustList.PowerCustInfo[CUST_SUB_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (TRUE != _hwPowerDown(pwInfo.PowerType)) {
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
				if (mtkcam_gpio_set(pinSetIdx, SUB_AVDD, Vol_Low)) {
						PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		}
		else{
			if (PowerCustList.PowerCustInfo[CUST_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (TRUE != _hwPowerDown(pwInfo.PowerType)) {
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			//} else {
				if (mtkcam_gpio_set(pinSetIdx, AVDD, Vol_Low)) {
						PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		}

		
	} else if (pwInfo.PowerType == DVDD) {
		if (pinSetIdx == 2) {
			if (PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (TRUE != _hwPowerDown(DVDD)) {
						PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
						return FALSE;
					}
				} else {
					if (mtkcam_gpio_set(pinSetIdx, MAIN2_DVDD, 1-PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Voltage)) {
						PK_ERR("[CAMERA CUST_MAIN2_DVDD] set gpio failed!!\n");
					}
				}
			} else if (pinSetIdx == 1) {
				if (PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
					if (TRUE != _hwPowerDown(SUB_DVDD)) {
						PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
						return FALSE;
					}
				} else {
					if (mtkcam_gpio_set(pinSetIdx, SUB_DVDD, 1-PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Voltage)) {
						PK_ERR("[CAMERA CUST_SUB_DVDD] set gpio failed!!\n");
					}
				}
			} else {
				if (PowerCustList.PowerCustInfo[CUST_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
					if (TRUE != _hwPowerDown(DVDD)) {
						PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
						return FALSE;
					}
				} else {
					if (mtkcam_gpio_set(pinSetIdx, DVDD, 1-PowerCustList.PowerCustInfo[CUST_DVDD].Voltage)) {
						PK_ERR("[CAMERA CUST_DVDD] set gpio failed!!\n");
					}
				}
			}
	} else if (pwInfo.PowerType == DOVDD) {
		if (PowerCustList.PowerCustInfo[CUST_DOVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != _hwPowerDown(pwInfo.PowerType)) {
				PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mtkcam_gpio_set(pinSetIdx, DOVDD, 1-PowerCustList.PowerCustInfo[CUST_DOVDD].Voltage)) {
				PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");/* 1-voltage for reverse*/
			}
		}
	} else if (pwInfo.PowerType == AFVDD) {
		if (PowerCustList.PowerCustInfo[CUST_AFVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != _hwPowerDown(pwInfo.PowerType)) {
				PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
            
            if (mtkcam_gpio_set(pinSetIdx, AFVDD, Vol_Low)) {
                PK_ERR("[CAMERA CUST_AF PWDN] set gpio failed!!\n");
            }
		} else {
			if (mtkcam_gpio_set(pinSetIdx, AFVDD, 1-PowerCustList.PowerCustInfo[CUST_AFVDD].Voltage)) {
				PK_ERR("[CAMERA CUST_DOVDD] set gpio failed!!\n");/* 1-voltage for reverse*/
			}
		}
	} else if (pwInfo.PowerType == PDN) {
		/*PK_INFO("hwPowerDown: PDN %d\n", pwInfo.Voltage);*/
		if (pwInfo.Voltage == Vol_High) {
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON])) {
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		} else {
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {

				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == RST) {
		/*PK_INFO("hwPowerDown: RST %d\n", pwInfo.Voltage);*/
		if (pwInfo.Voltage == Vol_High) {
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		} else {
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {

				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}

	} else if (pwInfo.PowerType == SensorMCLK) {
		if (pinSetIdx == 0) {
			ISP_MCLK1_EN(FALSE);
		} else if (pinSetIdx == 1) {
			ISP_MCLK1_EN(FALSE);
		} else {
		}
	} else {
	}
	return TRUE;
}




int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On,
		       char *mode_name)
{

	int pwListIdx, pwIdx;
	BOOL sensorInPowerList = KAL_FALSE;

	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx) {
		pinSetIdx = 0;
	} else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
		pinSetIdx = 1;
	} else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
		pinSetIdx = 2;
	}
	/* power ON */
	if (On) {
		PK_INFO("PowerOn:SensorName=%s, pinSetIdx=%d, sensorIdx:%d\n", currSensorName, pinSetIdx, SensorIdx);
		//PK_INFO("kdCISModulePowerOn -on:pinSetIdx=%d\n", pinSetIdx);
		//ISP_MCLK1_EN(1);

		for (pwListIdx = 0; pwListIdx < 16; pwListIdx++) {
			if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
			    && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName))) {
				//PK_INFO("sensorIdx:%d\n", SensorIdx);

				sensorInPowerList = KAL_TRUE;

				for (pwIdx = 0; pwIdx < 10; pwIdx++) {
					PK_DBG("PowerType:%d\n", PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType);
					if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].
					    PowerType != VDD_None) {
						if (hwpoweron(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx], mode_name) == FALSE)
                        PK_DBG("fail to power on PowerType:%d\n", PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType);
						//	goto _kdCISModulePowerOn_exit_;
					} else {
						PK_DBG("pwIdx=%d\n", pwIdx);
						break;
					}
				}
				break;
			} else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL) {
				break;
			} else {
			}
		}
	} else {		
	    /* power OFF */
		PK_INFO("PowerOFF:pinSetIdx=%d, sensorIdx:%d\n", pinSetIdx, SensorIdx);

		for (pwListIdx = 0; pwListIdx < 16; pwListIdx++) {
			if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
			    && (0 ==
				strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,
				       currSensorName))) {
				/*PK_INFO("kdCISModulePowerOn get in---\n");*/
				//PK_INFO("sensorIdx:%d\n", SensorIdx);

				sensorInPowerList = KAL_TRUE;

				for (pwIdx = 9; pwIdx >= 0; pwIdx--) {
					PK_DBG("PowerType:%d\n", PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType);
					if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].
					    PowerType != VDD_None) {
						if (hwpowerdown
						    (PowerOnList.PowerSeq[pwListIdx].
						     PowerInfo[pwIdx], mode_name) == FALSE)
                        PK_DBG("fail to power down PowerType:%d\n", PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType);
						//	goto _kdCISModulePowerOn_exit_;
						if (pwIdx > 0) {
							if (PowerOnList.PowerSeq[pwListIdx].
							    PowerInfo[pwIdx - 1].Delay > 0)
								mdelay(PowerOnList.
								       PowerSeq[pwListIdx].
								       PowerInfo[pwIdx - 1].Delay);
						}
					} else {
						PK_DBG("pwIdx=%d\n", pwIdx);
					}
				}
			} else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL) {
				break;
			} else {
			}
		}
		//ISP_MCLK1_EN(0);
	}			/*  */

	return 0;

//_kdCISModulePowerOn_exit_:
	return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);

void checkPowerBeforClose(char *mode_name)
{
}
/* !-- */
/*  */
