#ifndef _KD_CAMERA_HW_H_
#define _KD_CAMERA_HW_H_

#include <mach/mt_gpio.h>

#ifdef MTK_MT6306_SUPPORT
#include <mach/dcl_sim_gpio.h>
#endif

#include <mach/mt_pm_ldo.h>
#include "pmic_drv.h"

//
//Analog
#define CAMERA_POWER_VCAM_A		PMIC_APP_MAIN_CAMERA_POWER_A
#define SUB_CAMERA_POWER_VCAM_A		PMIC_APP_SUB_CAMERA_POWER_A
//Digital
#define CAMERA_POWER_VCAM_D		PMIC_APP_MAIN_CAMERA_POWER_D
#define SUB_CAMERA_POWER_VCAM_D		PMIC_APP_SUB_CAMERA_POWER_D
//digital io
#define CAMERA_POWER_VCAM_IO		PMIC_APP_MAIN_CAMERA_POWER_IO
#define SUB_CAMERA_POWER_VCAM_IO		PMIC_APP_SUB_CAMERA_POWER_IO
//AF
#define CAMERA_POWER_VCAM_AF		PMIC_APP_MAIN_CAMERA_POWER_AF
//FIXME, should defined in DCT tool

//Main sensor
#define CAMERA_CMRST_PIN            GPIO_CAMERA_CMRST_PIN
#define CAMERA_CMRST_PIN_M_GPIO     GPIO_CAMERA_CMRST_PIN_M_GPIO

#define CAMERA_CMPDN_PIN            GPIO_CAMERA_CMPDN_PIN
#define CAMERA_CMPDN_PIN_M_GPIO     GPIO_CAMERA_CMPDN_PIN_M_GPIO

//FRONT sensor
#define CAMERA_CMRST1_PIN           GPIO_CAMERA_CMRST1_PIN
#define CAMERA_CMRST1_PIN_M_GPIO    GPIO_CAMERA_CMRST1_PIN_M_GPIO

#define CAMERA_CMPDN1_PIN           GPIO_CAMERA_CMPDN1_PIN
#define CAMERA_CMPDN1_PIN_M_GPIO    GPIO_CAMERA_CMPDN1_PIN_M_GPIO

// Define I2C Bus Num
#define SUPPORT_I2C_BUS_NUM1        0
#define SUPPORT_I2C_BUS_NUM2        0

typedef enum{
	VDD_None,
	PDN,
	RST,
	SensorMCLK,
	AVDD  = CAMERA_POWER_VCAM_A,
	DVDD  = CAMERA_POWER_VCAM_D,
	DOVDD = CAMERA_POWER_VCAM_IO,
	AFVDD = CAMERA_POWER_VCAM_AF
}PowerType;

typedef enum{
	Vol_Low =0,
	Vol_High=1,
	Vol_900  = VOL_0900,
    Vol_1000 = VOL_1000,
    Vol_1100 = VOL_1100,
    Vol_1200 = VOL_1200,	
    Vol_1300 = VOL_1300,    
    Vol_1350 = VOL_1350,   
    Vol_1500 = VOL_1500,    
    Vol_1800 = VOL_1800,    
    Vol_2000 = VOL_2000,
    Vol_2100 = VOL_2100,
    Vol_2500 = VOL_2500,    
    Vol_2800 = VOL_2800, 
    Vol_3000 = VOL_3000,
    Vol_3300 = VOL_3300,
    Vol_3400 = VOL_3400, 
    Vol_3500 = VOL_3500,
    Vol_3600 = VOL_3600  
}Voltage;


typedef struct{
	PowerType PowerType;
	Voltage Voltage;
	u32 Delay;
}PowerInformation;


typedef struct{
	char* SensorName;
	PowerInformation PowerInfo[12];
}PowerSequence;

typedef struct{
	PowerSequence PowerSeq[16];	
}PowerUp;

typedef struct{
	u32 Gpio_Pin;  
	u32 Gpio_Mode;
	Voltage Voltage;
}PowerCustInfo;

typedef struct{
	PowerCustInfo PowerCustInfo[6];
}PowerCust;
#endif

