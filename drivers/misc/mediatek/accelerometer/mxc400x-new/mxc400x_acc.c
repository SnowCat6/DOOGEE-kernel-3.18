#include <cust_acc.h>
#include "accel.h"
#include "mach/upmu_sw.h"
#include <mt-plat/upmu_common.h>

/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = 2,
    .direction = 0,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
    .is_batch_supported = false,
};
/*---------------------------------------------------------------------------*/
struct acc_hw* mxc400x_get_cust_acc_hw(void)
{
    return &cust_acc_hw;
}
