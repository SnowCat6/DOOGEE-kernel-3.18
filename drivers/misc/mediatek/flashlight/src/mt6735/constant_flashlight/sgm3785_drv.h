/************************************************************************************************
*							*file name : sgm3785_drv.h
*							*Version : v1.0
*							*Author : erick
*							*Date : 2015.4.16
*************************************************************************************************/
#ifndef _SGM3785_DRV_H_
#define _SGM3785_DRV_H_
#include <linux/delay.h>
#include <linux/xlog.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_pwm.h>
#include <mach/mt_pwm_hal.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>

#if defined(CONFIG_T925_PROJ)
	#define PWM_NO PWM3   	//HWpwm0 --> SWpwm1
	#define F_DUTY 10		// 0,1,2,3,4,5,6,7,8,9,10
	#define T_DUTY 6  		// 0,1,2,3,4,5,6,7,8,9,10
#elif defined(CONFIG_T875_PROJ)
	#define PWM_NO PWM3   	//HWpwm0 --> SWpwm1
	#define F_DUTY 8		// 0,1,2,3,4,5,6,7,8,9,10
	#define T_DUTY 6  		// 0,1,2,3,4,5,6,7,8,9,10
#elif defined(CONFIG_T87_PROJ)
	#define PWM_NO PWM3   	//HWpwm0 --> SWpwm1
	#define F_DUTY 7		// 0,1,2,3,4,5,6,7,8,9,10
	#define T_DUTY 6  		// 0,1,2,3,4,5,6,7,8,9,10
#elif defined(CONFIG_T991_PROJ)
	#define PWM_NO PWM4   	//HWpwm0 --> SWpwm1
	#define F_DUTY 8		// 0,1,2,3,4,5,6,7,8,9,10
	#define T_DUTY 10  		// 0,1,2,3,4,5,6,7,8,9,10
#else
	#define PWM_NO PWM4   	//HWpwm0 --> SWpwm1
	#define F_DUTY 9		// 0,1,2,3,4,5,6,7,8,9,10
	#define T_DUTY 8  		// 0,1,2,3,4,5,6,7,8,9,10
#endif

#define ENF		GPIO_CAMERA_FLASH_MODE_PIN
#define ENM		GPIO_CAMERA_FLASH_EN_PIN
#define ENM_PWM_MODE	GPIO_CAMERA_FLASH_EN_PIN_M_PWM

#define SGM3138_DEBUG
#ifdef SGM3138_DEBUG
#define sgm3138_dbg printk
#else
#define sgm3138_dbg //
#endif

enum SGM3785_MODE{
	MODE_MIN = 0,
	PWD_M = MODE_MIN,
	FLASH_M,
	TORCH_M,
	MODE_MAX
};

#endif
