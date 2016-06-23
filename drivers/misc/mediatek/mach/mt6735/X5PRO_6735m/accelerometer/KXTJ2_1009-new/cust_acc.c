#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>

#if defined(CONFIG_T96_PROJ)||defined(CONFIG_T89_PROJ)
	#define ACC_I2C_BUS_NUM		2
	#define ACC_DIRECTION		4
#elif defined(CONFIG_T933_PROJ)||defined(CONFIG_T935_PROJ)
	#define ACC_I2C_BUS_NUM		2
	#define ACC_DIRECTION		6
#elif defined(CONFIG_T93_PROJ)
	#define ACC_I2C_BUS_NUM		2
	#define ACC_DIRECTION		5
#elif defined(CONFIG_T875_PROJ)
	#define ACC_I2C_BUS_NUM		2
	#define ACC_DIRECTION		0
#else
	#define ACC_I2C_BUS_NUM		2
	#define ACC_DIRECTION		5
#endif
/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = ACC_I2C_BUS_NUM,
    .direction = ACC_DIRECTION,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
    .is_batch_supported = false,
};
/*---------------------------------------------------------------------------*/
struct acc_hw* get_cust_acc_hw(void)
{
    return &cust_acc_hw;
}
