#ifndef _DCL_SIM_GPIO_H_
#define _DCL_SIM_GPIO_H_

#include <mach/mt_gpio.h>

#if 0
/*----------------------------------------------------------------------------*/
/* GPIO DIRECTION */
typedef enum {
    GPIO_DIR_UNSUPPORTED = -1,
    GPIO_DIR_IN     = 0,
    GPIO_DIR_OUT    = 1,

    GPIO_DIR_MAX,
    GPIO_DIR_DEFAULT = GPIO_DIR_IN,
} GPIO_DIR;
/*----------------------------------------------------------------------------*/
/* GPIO OUTPUT */
typedef enum {
    GPIO_OUT_UNSUPPORTED = -1,
    GPIO_OUT_ZERO = 0,
    GPIO_OUT_ONE  = 1,

    GPIO_OUT_MAX,
    GPIO_OUT_DEFAULT = GPIO_OUT_ZERO,
    GPIO_DATA_OUT_DEFAULT = GPIO_OUT_ZERO,  /*compatible with DCT*/
} GPIO_OUT;
#endif

/******************************************************************************
* GPIO Driver interface 
******************************************************************************/
/*direction*/
int mt6306_set_gpio_dir(unsigned long pin, unsigned long dir);
int mt6306_get_gpio_dir(unsigned long pin);

/*output*/
int mt6306_set_gpio_out(unsigned long pin, unsigned long output);
int mt6306_get_gpio_out(unsigned long pin);

#endif //_DCL_SIM_GPIO_H_
