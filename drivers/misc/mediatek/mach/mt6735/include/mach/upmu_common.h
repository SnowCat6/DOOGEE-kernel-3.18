#ifndef _MT_PMIC_COMMON_H_
#define _MT_PMIC_COMMON_H_

#include <mach/mt_typedefs.h>

#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>

extern U32 pmic_read_interface (U32 RegNum, U32 *val, U32 MASK, U32 SHIFT);
extern U32 pmic_config_interface (U32 RegNum, U32 val, U32 MASK, U32 SHIFT);
extern U32 pmic_read_interface_nolock (U32 RegNum, U32 *val, U32 MASK, U32 SHIFT);
extern U32 pmic_config_interface_nolock (U32 RegNum, U32 val, U32 MASK, U32 SHIFT);
extern void pmic_lock(void);extern void pmic_unlock(void);
extern void upmu_set_reg_value(kal_uint32 reg, kal_uint32 reg_val);

extern kal_uint16 pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname,kal_uint32 val);
extern kal_uint16 pmic_get_register_value(PMU_FLAGS_LIST_ENUM flagname);
extern kal_uint16 bc11_set_register_value(PMU_FLAGS_LIST_ENUM flagname,kal_uint32 val);
extern kal_uint16 bc11_get_register_value(PMU_FLAGS_LIST_ENUM flagname);

extern void pmic_enable_interrupt(kal_uint32 intNo,kal_uint32 en,char *str);
extern void pmic_register_interrupt_callback(kal_uint32 intNo,void (EINT_FUNC_PTR)(void));
extern kal_uint16 is_battery_remove_pmic(void);

extern kal_int32 PMIC_IMM_GetCurrent(void);
extern kal_uint32 PMIC_IMM_GetOneChannelValue(mt6328_adc_ch_list_enum dwChannel, int deCount, int trimd);


#endif // _MT_PMIC_COMMON_H_
