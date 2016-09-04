#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
#include <mach/upmu_common.h>

 #define PS_HIGH 300
 #define PS_LOW  150

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x49, 0, 0, 0},

    /* MTK: modified to support AAL */
	.als_level	= {15, 20, 35,  55, 75, 90, 100, 1000, 2000, 3000, 6000, 10000, 14000, 18000, 20000},
	.als_value	= {20, 40, 90, 100, 160, 180, 225, 320, 640, 1280, 1280, 2600, 2600, 2600, 10240, 10240},
    .ps_threshold_high = PS_HIGH,
    .ps_threshold_low = PS_LOW,
    .is_batch_supported_ps = true,
    .is_batch_supported_als = false,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

