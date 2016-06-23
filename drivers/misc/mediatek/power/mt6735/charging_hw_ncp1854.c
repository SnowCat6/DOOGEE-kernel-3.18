/*****************************************************************************
 *
 * Filename:
 * ---------
 *    charging_pmic.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   11 Aug 2005 10:28:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *
 * 03 05 2015 wy.chuang
 * [ALPS01921641] [L1_merge] for PMIC and charging
 * .
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <mach/charging.h>
#include "ncp1854.h"
#include <mach/upmu_common.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <mach/upmu_hw.h>
#include <mach/upmu_sw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>
//#include <mach/mt_board_type.h>  //mtk71259 build error  20140523
#include <linux/spinlock.h>
#include <cust_charging.h>
#include <mach/mt_gpio.h>
#include <linux/wakelock.h>
#include <mach/mt6311.h>

 // ============================================================ //
 //define
 // ============================================================ //
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))


 // ============================================================ //
 //global variable
 // ============================================================ //

int gpio_off_dir  = GPIO_DIR_OUT;
int gpio_off_out  = GPIO_OUT_ONE;
int gpio_on_dir   = GPIO_DIR_OUT;
int gpio_on_out   = GPIO_OUT_ZERO;

/*#ifndef GPIO_CHR_SPM_PIN GPIO_SWCHARGER_EN_PIN
#define GPIO_CHR_SPM_PIN 65
#endif  */
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0

    #if defined(GPIO_PWR_AVAIL_WLC)
        kal_uint32 wireless_charger_gpio_number = GPIO_PWR_AVAIL_WLC; 
    #else
        kal_uint32 wireless_charger_gpio_number = 0; 
    #endif
    
#endif

static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;

kal_bool charging_type_det_done = KAL_TRUE;

//As 82 platform mach/charging.h could not cover all voltage setting, just hardcoded below settings
const kal_uint32 VBAT_CV_VTH[]=
{
	3300000,    3325000,    3350000,    3375000,
	3400000,    3425000,    3450000,    3475000,
	3500000,    3525000,    3550000,    3575000,
	3600000,    3625000,    3650000,    3675000,
	3700000,    3725000,    3750000,    3775000,
	3800000,    3825000,    3850000,    3875000,
	3900000,    3925000,    3950000,    3975000,
	4000000,    4025000,    4050000,    4075000,
	4100000,    4125000,    4150000,    4175000,
	4200000,    4225000,    4250000,    4275000,
	4300000,    4325000,    4350000,    4375000,
	4400000,    4425000,    4450000,    4475000,
};

/*
const kal_uint32 CS_VTH[]=
{
	CHARGE_CURRENT_450_00_MA,   CHARGE_CURRENT_550_00_MA,	CHARGE_CURRENT_650_00_MA, CHARGE_CURRENT_750_00_MA,
	CHARGE_CURRENT_850_00_MA,   CHARGE_CURRENT_950_00_MA,	CHARGE_CURRENT_1050_00_MA, CHARGE_CURRENT_1150_00_MA,
	CHARGE_CURRENT_1250_00_MA,   CHARGE_CURRENT_1350_00_MA,	CHARGE_CURRENT_1450_00_MA, CHARGE_CURRENT_1550_00_MA,
	CHARGE_CURRENT_1650_00_MA,   CHARGE_CURRENT_1750_00_MA,	CHARGE_CURRENT_1850_00_MA, CHARGE_CURRENT_1950_00_MA
}; 
*/

/* hardcoded current define which defined in NCP1854 IC spec, as common define doesnot cover all define
 * double confirmed with onsemi register set in spec has issue,below is the correct setting */
const kal_uint32 CS_VTH[]=
{
    45000,  50000,  60000,  70000,
	80000,  90000,  100000, 110000,
	120000, 130000, 140000, 150000,
	160000, 170000, 180000, 190000
};

const kal_uint32 INPUT_CS_VTH[]=
 {
	 CHARGE_CURRENT_100_00_MA,  CHARGE_CURRENT_500_00_MA
 }; 

const kal_uint32 INPUT_CS_VTH_TA[]=
 {
	 CHARGE_CURRENT_600_00_MA,  CHARGE_CURRENT_700_00_MA,  CHARGE_CURRENT_800_00_MA,
	 CHARGE_CURRENT_900_00_MA,  CHARGE_CURRENT_1000_00_MA,  CHARGE_CURRENT_1100_00_MA,
	 CHARGE_CURRENT_1200_00_MA,  CHARGE_CURRENT_1300_00_MA,  CHARGE_CURRENT_1400_00_MA,
	 CHARGE_CURRENT_1500_00_MA,  CHARGE_CURRENT_1600_00_MA,  170000,
	 180000, 190000, 200000
 }; 

const kal_uint32 VCDT_HV_VTH[]=
 {
	  BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_350000_V,
	  BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,   BATTERY_VOLT_04_550000_V,
	  BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,   BATTERY_VOLT_07_000000_V,
	  BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,   BATTERY_VOLT_10_500000_V		  
 };

 // ============================================================ //
 // function prototype
 // ============================================================ //
 
 
 // ============================================================ //
 //extern variable
 // ============================================================ //
 
 // ============================================================ //
 //extern function
 // ============================================================ //
 extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
 extern bool mt_usb_is_device(void);
 extern void Charger_Detect_Init(void);
 extern void Charger_Detect_Release(void);
extern int hw_charging_get_charger_type(void);
 extern void mt_power_off(void);
 extern kal_uint32 mt6311_get_chip_id(void);
 extern int is_mt6311_exist(void);
 extern int is_mt6311_sw_ready(void);
 

 kal_uint32 current_high_flag = 0;
 // ============================================================ //
#ifdef MTK_POWER_EXT_DETECT
static kal_uint32 mt_get_board_type(void)
{

	/*
  	*  Note: Don't use it in IRQ context
  	*/
#if 1
	 static int board_type = MT_BOARD_NONE;

	 if (board_type != MT_BOARD_NONE)
	 	return board_type;

	 spin_lock(&mt_board_lock);

	 /* Enable AUX_IN0 as GPI */
	 mt_set_gpio_ies(GPIO_PHONE_EVB_DETECT, GPIO_IES_ENABLE);

	 /* Set internal pull-down for AUX_IN0 */
	 mt_set_gpio_pull_select(GPIO_PHONE_EVB_DETECT, GPIO_PULL_DOWN);
	 mt_set_gpio_pull_enable(GPIO_PHONE_EVB_DETECT, GPIO_PULL_ENABLE);

	 /* Wait 20us */
	 udelay(20);

	 /* Read AUX_INO's GPI value*/
	 mt_set_gpio_mode(GPIO_PHONE_EVB_DETECT, GPIO_MODE_00);
	 mt_set_gpio_dir(GPIO_PHONE_EVB_DETECT, GPIO_DIR_IN);

	 if (mt_get_gpio_in(GPIO_PHONE_EVB_DETECT) == 1) {
		 /* Disable internal pull-down if external pull-up on PCB(leakage) */
		 mt_set_gpio_pull_enable(GPIO_PHONE_EVB_DETECT, GPIO_PULL_DISABLE);
		 board_type = MT_BOARD_EVB;
	 } else {
	 	 /* Disable internal pull-down if external pull-up on PCB(leakage) */
		 mt_set_gpio_pull_enable(GPIO_PHONE_EVB_DETECT, GPIO_PULL_DISABLE);
		 board_type = MT_BOARD_PHONE;
	 }
	 spin_unlock(&mt_board_lock);
	 pr_notice("[Kernel] Board type is %s\n", (board_type == MT_BOARD_EVB) ? "EVB" : "PHONE");
	 return board_type;
#else
	 return MT_BOARD_EVB;
#endif
}
#endif

 kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	if (val < array_size)
	{
		return parameter[val];
	}
	else
	{
		pr_notice("Can't find the parameter \r\n");	
		return parameter[0];
	}
}

 
 kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	kal_uint32 i;

    pr_notice("array_size = %d \r\n", array_size);
    
	for(i=0;i<array_size;i++)
	{
		if (val == *(parameter + i))
		{
				return i;
		}
	}

    pr_notice("NO register value match. val=%d\r\n", val);
	//TODO: ASSERT(0);	// not find the value
	return 0;
}


 static kal_uint32 bmt_find_closest_level(const kal_uint32 *pList,kal_uint32 number,kal_uint32 level)
 {
	 kal_uint32 i;
	 kal_uint32 max_value_in_last_element;
 
	 if(pList[0] < pList[1])
		 max_value_in_last_element = KAL_TRUE;
	 else
		 max_value_in_last_element = KAL_FALSE;
 
	 if(max_value_in_last_element == KAL_TRUE)
	 {
		 for(i = (number-1); i != 0; i--)	 //max value in the last element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

 		 pr_notice("Can't find closest level, small value first \r\n");
		 return pList[0];
		 //return CHARGE_CURRENT_0_00_MA;
	 }
	 else
	 {
		 for(i = 0; i< number; i++)  // max value in the first element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

		 pr_notice("Can't find closest level, large value first \r\n"); 	 
		 return pList[number -1];
  		 //return CHARGE_CURRENT_0_00_MA;
	 }
 }

#if 0
static void hw_bc11_dump_register(void)
{
	kal_uint32 reg_val = 0;
	kal_uint32 reg_num = CHR_CON18;
	kal_uint32 i = 0;

	for(i=reg_num ; i<=CHR_CON19 ; i+=2)
	{
		reg_val = upmu_get_reg_value(i);
		pr_info("Chr Reg[0x%x]=0x%x \r\n", i, reg_val);
	}
}


 static void hw_bc11_init(void)
 {
	 Charger_Detect_Init();
		 
	 //RG_BC11_BIAS_EN=1	
	 upmu_set_rg_bc11_bias_en(0x1);
	 //RG_BC11_VSRC_EN[1:0]=00
	 upmu_set_rg_bc11_vsrc_en(0x0);
	 //RG_BC11_VREF_VTH = [1:0]=00
	 upmu_set_rg_bc11_vref_vth(0x0);
	 //RG_BC11_CMP_EN[1.0] = 00
	 upmu_set_rg_bc11_cmp_en(0x0);
	 //RG_BC11_IPU_EN[1.0] = 00
	 upmu_set_rg_bc11_ipu_en(0x0);
	 //RG_BC11_IPD_EN[1.0] = 00
	 upmu_set_rg_bc11_ipd_en(0x0);
	 //BC11_RST=1
	 upmu_set_rg_bc11_rst(0x1);
	 //BC11_BB_CTRL=1
	 upmu_set_rg_bc11_bb_ctrl(0x1);
 
 	 //msleep(10);
 	 mdelay(50);

	 if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	 {
    		pr_info("hw_bc11_init() \r\n");
		hw_bc11_dump_register();
	 }	
	 
 }
 
 
 static U32 hw_bc11_DCD(void)
 {
	 U32 wChargerAvail = 0;
 
	 //RG_BC11_IPU_EN[1.0] = 10
	 upmu_set_rg_bc11_ipu_en(0x2);
	 //RG_BC11_IPD_EN[1.0] = 01
	 upmu_set_rg_bc11_ipd_en(0x1);
	 //RG_BC11_VREF_VTH = [1:0]=01
	 upmu_set_rg_bc11_vref_vth(0x1);
	 //RG_BC11_CMP_EN[1.0] = 10
	 upmu_set_rg_bc11_cmp_en(0x2);
 
	 //msleep(20);
	 mdelay(80);

 	 wChargerAvail = upmu_get_rgs_bc11_cmp_out();
	 
	 if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	 {
		pr_info("hw_bc11_DCD() \r\n");
		hw_bc11_dump_register();
	 }
	 
	 //RG_BC11_IPU_EN[1.0] = 00
	 upmu_set_rg_bc11_ipu_en(0x0);
	 //RG_BC11_IPD_EN[1.0] = 00
	 upmu_set_rg_bc11_ipd_en(0x0);
	 //RG_BC11_CMP_EN[1.0] = 00
	 upmu_set_rg_bc11_cmp_en(0x0);
	 //RG_BC11_VREF_VTH = [1:0]=00
	 upmu_set_rg_bc11_vref_vth(0x0);
 
	 return wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepA1(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_IPU_EN[1.0] = 10
	upmu_set_rg_bc11_ipu_en(0x2);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 10
	upmu_set_rg_bc11_cmp_en(0x2);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		pr_info("hw_bc11_stepA1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepB1(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_IPU_EN[1.0] = 01
	//upmu_set_rg_bc11_ipu_en(0x1);
	upmu_set_rg_bc11_ipd_en(0x1);      
	//RG_BC11_VREF_VTH = [1:0]=10
	//upmu_set_rg_bc11_vref_vth(0x2);
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		pr_info("hw_bc11_stepB1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepC1(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_IPU_EN[1.0] = 01
	upmu_set_rg_bc11_ipu_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		pr_info("hw_bc11_stepC1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepA2(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_VSRC_EN[1.0] = 10 
	upmu_set_rg_bc11_vsrc_en(0x2);
	//RG_BC11_IPD_EN[1:0] = 01
	upmu_set_rg_bc11_ipd_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		pr_info("hw_bc11_stepA2() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_VSRC_EN[1:0]=00
	upmu_set_rg_bc11_vsrc_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepB2(void)
 {
	U32 wChargerAvail = 0;
 
	//RG_BC11_IPU_EN[1:0]=10
	upmu_set_rg_bc11_ipu_en(0x2);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x1);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		pr_info("hw_bc11_stepB2() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 
 
 static void hw_bc11_done(void)
 {
	//RG_BC11_VSRC_EN[1:0]=00
	upmu_set_rg_bc11_vsrc_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=0
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//RG_BC11_BIAS_EN=0
	upmu_set_rg_bc11_bias_en(0x0); 
 
	Charger_Detect_Release();

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		pr_info("hw_bc11_done() \r\n");
		hw_bc11_dump_register();
	}
    
 }
#endif

#if 0
 static kal_uint32 is_chr_det(void)
 {
	 kal_uint32 val=0;
   
	 val = mt6325_upmu_get_rgs_chrdet();
 
	 pr_notice("[is_chr_det] %d\n", val);
	 
	 return val;
 }
#endif
 static kal_uint32 charging_hw_init(void *data)
{
   //static kal_uint32 run_hw_init_once_flag=1;

    kal_uint32 ncp1854_status;
 	kal_uint32 status = STATUS_OK;

    if (Enable_BATDRV_LOG == 1) {
        pr_notice("[BATTERY:ncp1854] ChargerHwInit_ncp1854\n" );
    }

    ncp1854_status = ncp1854_get_chip_status();
#ifdef DISABLE_NCP1854_FACTORY_MODE
	ncp1854_set_fctry_mode(0x0);//if you want disable factory mode,the FTRY pin and FCTRY_MOD_REG need differ
#endif
    ncp1854_set_otg_en(0x0);
    ncp1854_set_trans_en(0);
    ncp1854_set_tj_warn_opt(0x0);//set at disabled, by MT6325 BATON
//  ncp1854_set_int_mask(0x0); //disable all interrupt
    ncp1854_set_int_mask(0x1); //enable all interrupt for boost mode status monitor
   // ncp1854_set_tchg_rst(0x1); //reset charge timer
#ifdef NCP1854_PWR_PATH
    ncp1854_set_pwr_path(0x1);
#else
    ncp1854_set_pwr_path(0x0);
#endif

   ncp1854_set_chgto_dis(0x1); //disable charge timer
    if((ncp1854_status == 0x8) || (ncp1854_status == 0x9) || (ncp1854_status == 0xA)) //WEAK WAIT, WEAK SAFE, WEAK CHARGE
        ncp1854_set_ctrl_vbat(0x1C); //VCHG = 4.0V

   	//if(run_hw_init_once_flag)
	//{
         ncp1854_set_ieoc(0x0);
         ncp1854_set_iweak(0x3); //weak charge current = 300mA
      // run_hw_init_once_flag=0;
	//}

	ncp1854_set_aicl_en(0x1); //enable AICL as PT team suggest

	ncp1854_set_iinset_pin_en(0x0); //Input current limit and AICL control by I2C

    ncp1854_set_ctrl_vfet(0x3); // VFET = 3.4V

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
		if(wireless_charger_gpio_number!=0)
		{
			mt_set_gpio_mode(wireless_charger_gpio_number,0); // 0:GPIO mode
			mt_set_gpio_dir(wireless_charger_gpio_number,0); // 0: input, 1: output
		}
#endif

	return status;
}

 static kal_uint32 charging_dump_register(void *data)
 {
 	kal_uint32 status = STATUS_OK;

    pr_notice("charging_dump_register\r\n");

    ncp1854_dump_register();
   	
	return status;
 }	


 static kal_uint32 charging_enable(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	if(KAL_TRUE == enable)
	{
		ncp1854_set_chg_en(0x1); // charger enable
		//Set SPM = 1		
#ifdef GPIO_SWCHARGER_EN_PIN

		mt_set_gpio_mode(GPIO_SWCHARGER_EN_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_SWCHARGER_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_SWCHARGER_EN_PIN, GPIO_OUT_ONE);

#endif
	}
	else
	{
#if defined(CONFIG_USB_MTK_HDRC_HCD)
   		if(mt_usb_is_device())
#endif 			
    	{
			ncp1854_set_chg_en(0x0); // charger disable
    	}
	}
		
	return status;
 }


 static kal_uint32 charging_set_cv_voltage(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint16 register_value;
	kal_uint32 cv_value = *(kal_uint32 *)(data);	
	kal_uint32 array_size;
	kal_uint32 set_chr_cv;	

	#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
		cv_value = BATTERY_VOLT_04_350000_V;
	#endif

	//use nearest value
	array_size = GETARRAYNUM(VBAT_CV_VTH);
	set_chr_cv = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv_value);
	
	register_value = charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH), set_chr_cv);


#if 1
    ncp1854_set_ctrl_vbat(register_value);
#else
    //PCB workaround
    if(mt6325_upmu_get_swcid() == PMIC6325_E1_CID_CODE)
    {
        ncp1854_set_ctrl_vbat(0x14); //3.8v
        pr_notice("[charging_set_cv_voltage] set low CV by 6325 E1\n");
    }
    else
    {
        if(is_mt6311_exist())
        {
            if(mt6311_get_chip_id()==PMIC6311_E1_CID_CODE)
            {
                ncp1854_set_ctrl_vbat(0x14); //3.8v
                pr_notice("[charging_set_cv_voltage] set low CV by 6311 E1\n");
            }
            else
            {
                ncp1854_set_ctrl_vbat(register_value);
            }
        }
        else
        {
            ncp1854_set_ctrl_vbat(register_value);
        } 
    }  

#endif

	return status;
 } 	


 static kal_uint32 charging_get_current(void *data)
 {
    kal_uint32 status = STATUS_OK;
    kal_uint32 array_size; 
    kal_uint8 ret_val=0;    
	    
    //Get current level
	//ret_val = ncp1854_get_ichg();
    //ncp1854_read_interface(NCP1854_CON15, &ret_val, CON15_ICHG_MASK, CON15_ICHG_SHIFT);						    
    //Parsing
   // ret_val = (ret_val*100) + 400;
	
    array_size = GETARRAYNUM(CS_VTH);
    ret_val = ncp1854_get_ichg();	//IINLIM
    if(current_high_flag==1)
	*(kal_uint32 *)data = charging_value_to_parameter(CS_VTH,array_size,ret_val)+ 160000;
	else
    *(kal_uint32 *)data = charging_value_to_parameter(CS_VTH,array_size,ret_val);
	
    return status;
 }


 static kal_uint32 charging_set_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;
	kal_uint32 current_value = *(kal_uint32 *)data;
	//kal_uint32 current_high_flag = 0;
	
	array_size = GETARRAYNUM(CS_VTH);
	if (current_value <=190000)
	{
	    set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
	    register_value = charging_parameter_to_value(CS_VTH, array_size ,set_chr_current);
		current_high_flag = 0x0;
	} else {
	    set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value - 160000);
	    register_value = charging_parameter_to_value(CS_VTH, array_size ,set_chr_current);
		current_high_flag = 0x1;
	}

	//current set by SW and disable automatic charge current
	//ncp1854_set_aicl_en(0x0); //disable AICL
	//set which register first? mmz
	ncp1854_set_ichg_high(current_high_flag);
	ncp1854_set_ichg(register_value);       	
	
	return status;
 } 	
 

 static kal_uint32 charging_set_input_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;
	kal_uint32 current_value = *(kal_uint32 *)data;
    
	if (current_value < 60000)
	{
	    array_size = GETARRAYNUM(INPUT_CS_VTH);
	    set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, current_value);
	    register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size ,set_chr_current);	
	    ncp1854_set_iinlim(register_value);
	    ncp1854_set_iinlim_ta(0x0);
	} else {
	    array_size = GETARRAYNUM(INPUT_CS_VTH_TA);
	    set_chr_current = bmt_find_closest_level(INPUT_CS_VTH_TA, array_size, current_value);
	    register_value = charging_parameter_to_value(INPUT_CS_VTH_TA, array_size ,set_chr_current);	
	    ncp1854_set_iinlim_ta(register_value);
	}
        
	//ncp1854_set_iinset_pin_en(0x0); //Input current limit and AICL control by I2C
	ncp1854_set_iinlim_en(0x1); //enable input current limit
	//ncp1854_set_aicl_en(0x0); //disable AICL

	return status;
 }


 static kal_uint32 charging_get_charging_status(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 ret_val;

	ret_val = ncp1854_get_chip_status();
	//check whether chargeing DONE
	if (ret_val == 0x6)
	{
		*(kal_uint32 *)data = KAL_TRUE;
	} else {
		*(kal_uint32 *)data = KAL_FALSE;
	}
	
	return status;
 } 	

void kick_charger_wdt(void)
{
	ncp1854_set_wdto_dis(0x0);
}

 static kal_uint32 charging_reset_watch_dog_timer(void *data)
 {
	 kal_uint32 status = STATUS_OK;
 
     pr_notice("charging_reset_watch_dog_timer\r\n");
 
	 kick_charger_wdt();
	 return status;
 }
 
 
  static kal_uint32 charging_set_hv_threshold(void *data)
  {
	 kal_uint32 status = STATUS_OK;
 
	 kal_uint32 set_hv_voltage;
	 kal_uint32 array_size;
	 kal_uint16 register_value;
	 kal_uint32 voltage = *(kal_uint32*)(data);
	 
	 array_size = GETARRAYNUM(VCDT_HV_VTH);
	 set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	 register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size ,set_hv_voltage);
	 pmic_set_register_value(PMIC_RG_VCDT_HV_VTH,register_value);
	 return status;
  }
 
 
  static kal_uint32 charging_get_hv_status(void *data)
  {
		  kal_uint32 status = STATUS_OK;
	  
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
		  *(kal_bool*)(data) = 0;
		  pr_notice("[charging_get_hv_status] charger ok for bring up.\n");
#else		 
		 *(kal_bool*)(data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
#endif
		   
		  return status;

  }


 static kal_uint32 charging_get_battery_status(void *data)
 {
		 kal_uint32 status = STATUS_OK;
	 	 kal_uint32 val = 0;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
		 *(kal_bool*)(data) = 0; // battery exist
		 pr_notice("[charging_get_battery_status] battery exist for bring up.\n");
#else
	val=pmic_get_register_value(PMIC_BATON_TDET_EN);
	pr_info("[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
	pmic_set_register_value(PMIC_BATON_TDET_EN,1);
	pmic_set_register_value(PMIC_RG_BATON_EN,1);
	*(kal_bool*)(data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool*)(data) =  KAL_FALSE;
	}
#endif
		  
		 return status;

 }


 static kal_uint32 charging_get_charger_det_status(void *data)
 {
		 kal_uint32 status = STATUS_OK;
		 kal_uint32 val = 0;
	 
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
		 val = 1;
		 pr_notice("[charging_get_charger_det_status] charger exist for bring up.\n"); 
#else    
	 	 val = pmic_get_register_value(PMIC_RGS_CHRDET);
#endif
		 
		 *(kal_bool*)(data) = val;
		 if(val == 0)
			 g_charger_type = CHARGER_UNKNOWN;
			   
		 return status;

 }


kal_bool charging_type_detection_done(void)
{
	 return charging_type_det_done;
}

//extern CHARGER_TYPE hw_charger_type_detection(void);

 static kal_uint32 charging_get_charger_type(void *data)
 {
	 kal_uint32 status = STATUS_OK;
	 CHARGER_TYPE charger_type = CHARGER_UNKNOWN;
#if defined(CONFIG_POWER_EXT)
	 *(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else
	 charging_type_det_done = KAL_FALSE;	
	 charger_type = hw_charging_get_charger_type();
	 pr_notice("charging_get_charger_type = %d\r\n", charger_type);
 
	 *(CHARGER_TYPE*)(data) = charger_type;
	 charging_type_det_done = KAL_TRUE;
	 g_charger_type = *(CHARGER_TYPE*)(data);
#endif
    return status;

	
#if 0 
		kal_uint32 status = STATUS_OK;
		 
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
		*(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else
		
    #if defined(MTK_WIRELESS_CHARGER_SUPPORT)
		int wireless_state = 0;
		if(wireless_charger_gpio_number!=0)
		{
			wireless_state = mt_get_gpio_in(wireless_charger_gpio_number);
			if(wireless_state == WIRELESS_CHARGER_EXIST_STATE)
			{
				*(CHARGER_TYPE*)(data) = WIRELESS_CHARGER;
				pr_notice("WIRELESS_CHARGER!\n");
				return status;
			}
		}
		else
		{
			pr_notice("wireless_charger_gpio_number=%d\n", wireless_charger_gpio_number);
		}
		
		if(g_charger_type!=CHARGER_UNKNOWN && g_charger_type!=WIRELESS_CHARGER)
		{
			*(CHARGER_TYPE*)(data) = g_charger_type;
			pr_notice("return %d!\n", g_charger_type);
			return status;
		}
    #endif
		
		if(is_chr_det()==0)
		{
			g_charger_type = CHARGER_UNKNOWN; 
			*(CHARGER_TYPE*)(data) = CHARGER_UNKNOWN;
			pr_notice("[charging_get_charger_type] return CHARGER_UNKNOWN\n");
			return status;
		}
		
		charging_type_det_done = KAL_FALSE;
	
		*(CHARGER_TYPE*)(data) = hw_charging_get_charger_type();
		//*(CHARGER_TYPE*)(data) = STANDARD_HOST;
		//*(CHARGER_TYPE*)(data) = STANDARD_CHARGER;
	
		charging_type_det_done = KAL_TRUE;
	
		g_charger_type = *(CHARGER_TYPE*)(data);
		
#endif
	
		return status;
#endif
}

static kal_uint32 charging_get_is_pcm_timer_trigger(void *data)
{
	kal_uint32 status = STATUS_OK;
	*(kal_bool*)(data) = KAL_FALSE;
	return status;
}

static kal_uint32 charging_set_platform_reset(void *data)
{
    kal_uint32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)    
#else 
    pr_notice("charging_set_platform_reset\n");
 
    arch_reset(0,NULL);
#endif
        
    return status;
}

static kal_uint32 charging_get_platfrom_boot_mode(void *data)
{
    kal_uint32 status = STATUS_OK;
  
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)   
#else   
    *(kal_uint32*)(data) = get_boot_mode();

    pr_notice("get_boot_mode=%d\n", get_boot_mode());
#endif
         
    return status;
}

static kal_uint32 charging_set_power_off(void *data)
{
    kal_uint32 status = STATUS_OK;
  
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
    pr_notice("charging_set_power_off\n");
    mt_power_off();
#endif
         
    return status;
}

 static kal_uint32 charging_get_power_source(void *data)
 {
 	kal_uint32 status = STATUS_OK;

#if defined(MTK_POWER_EXT_DETECT)
 	if (MT_BOARD_PHONE == mt_get_board_type())
		*(kal_bool *)data = KAL_FALSE;
	else
 	 	*(kal_bool *)data = KAL_TRUE;
#else
	*(kal_bool *)data = KAL_FALSE;
#endif

	 return status;
 }


 static kal_uint32 charging_get_csdac_full_flag(void *data)
 {
 	kal_uint32 status = STATUS_OK;

	return status;	
 }


 static kal_uint32 charging_set_ta_current_pattern(void *data)
 {
	 kal_uint32 status = STATUS_OK;
	 kal_uint32 increase = *(kal_uint32*)(data);
	 
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	 BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_350000_V;
#else
	 BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_200000_V;
#endif

	 charging_set_cv_voltage(&cv_voltage);	//Set CV
	 ncp1854_set_ichg(0x01);  //Set charging current 500ma
	 ncp1854_set_ichg_high(0x00);
	 
	 ncp1854_set_iinlim_ta(0x00);
	 ncp1854_set_iinlim(0x01);	//set charging iinlim 500ma
	 
	 ncp1854_set_iinlim_en(0x01); //enable iinlim
	 ncp1854_set_chg_en(0x01);  //Enable Charging
	 
	// ncp1854_dump_register();
	 
	 if(increase == KAL_TRUE)
	 {
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_increase() on 1");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_increase() off 1");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_increase() on 2");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_increase() off 2");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_increase() on 3");
		 msleep(281);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_increase() off 3");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_increase() on 4");
		 msleep(281);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_increase() off 4");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_increase() on 5");
		 msleep(281);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_increase() off 5");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_increase() on 6");
		 msleep(485);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_increase() off 6");
		 msleep(50);
		 
		 pr_notice("mtk_ta_increase() end \n");
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 msleep(200);
	 }
	 else
	 {
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_decrease() on 1");
		 msleep(281);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_decrease() off 1");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_decrease() on 2");
		 msleep(281);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_decrease() off 2");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_decrease() on 3");
		 msleep(281);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_decrease() off 3");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_decrease() on 4");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_decrease() off 4");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_decrease() on 5");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_decrease() off 5");
		 msleep(85);
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
		 pr_info("mtk_ta_decrease() on 6");
		 msleep(485);
		 
		 ncp1854_set_iinlim(0x0); /* 100mA */
		 pr_info("mtk_ta_decrease() off 6");
		 msleep(50);
		 
		 pr_notice("mtk_ta_decrease() end \n");
		 
		 ncp1854_set_iinlim(0x1); /* 500mA */
	 }
	 
	return status;	
 }

static kal_uint32 charging_get_error_state(void *data)
{
	return STATUS_UNSUPPORTED;
}

static kal_uint32 charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;
}

 static kal_uint32 (* const charging_func[CHARGING_CMD_NUMBER])(void *data)=
 {
 	  charging_hw_init
	,charging_dump_register
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platfrom_boot_mode
	,charging_set_power_off
	,charging_get_power_source
	,charging_get_csdac_full_flag
	,charging_set_ta_current_pattern
	,charging_set_error_state
 };

 
 /*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION															 
 *		 This function is called to set the charger hw
 *
 * CALLS  
 *
 * PARAMETERS
 *		None
 *	 
 * RETURNS
 *		
 *
 * GLOBALS AFFECTED
 *	   None
 */
 kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
 {
	 kal_int32 status;
	 if(cmd < CHARGING_CMD_NUMBER)
		 status = charging_func[cmd](data);
	 else
		 return STATUS_UNSUPPORTED;
 
	 return status;
 }


