/************************************************************************************************
*							*file name : sgm3785_drv.c
*							*Version : v1.0
*							*Author : erick
*							*Date : 2015.4.16
*************************************************************************************************/
#include <sgm3785_drv.h>

static kal_bool active_flag = KAL_FALSE;

struct pwm_spec_config sgm3785_config = {
    .pwm_no = PWM_NO,
    .mode = PWM_MODE_OLD,
    .clk_div = CLK_DIV32,//CLK_DIV16: 147.7kHz    CLK_DIV32: 73.8kHz
    .clk_src = PWM_CLK_OLD_MODE_BLOCK,
    .pmic_pad = false,
    .PWM_MODE_OLD_REGS.IDLE_VALUE = IDLE_FALSE,
    .PWM_MODE_OLD_REGS.GUARD_VALUE = GUARD_FALSE,
    .PWM_MODE_OLD_REGS.GDURATION = 0,
    .PWM_MODE_OLD_REGS.WAVE_NUM = 0,
    .PWM_MODE_OLD_REGS.DATA_WIDTH = 10,
    .PWM_MODE_OLD_REGS.THRESH = 5,
};

/*flash mode
ENF: 1                           _____________
         0 ____________|       660ms       |____
ENM:1    _      _     _     _      _     _     _      _     _
         0 _| |_| |_| |_| |_| |_| |_| |_| |_| |_
*/
static S32 sgm3785_set_flash_mode(U16 duty)
{	
	S32 ret;

	if(active_flag == KAL_TRUE)
		return;

	if(mt_set_gpio_mode(ENF,GPIO_MODE_00)){sgm3138_dbg("[sgm3785] set enf gpio mode failed!! \n");}
	if(mt_set_gpio_dir(ENF,GPIO_DIR_OUT)){sgm3138_dbg("[sgm3785] set  enf gpio dir failed!! \n");}
	mt_set_gpio_out(ENF,GPIO_OUT_ZERO);

	if(mt_set_gpio_mode(ENM,ENM_PWM_MODE)){sgm3138_dbg("[sgm3785] set enf gpio mode failed!! \n");}
	sgm3785_config.PWM_MODE_OLD_REGS.THRESH = duty;
	ret = pwm_set_spec_config(&sgm3785_config);	

	mt_set_gpio_out(ENF,GPIO_OUT_ONE);	

	active_flag = KAL_TRUE;
	return ret;
}

/*movie/torch mode
ENF: 1
          0 _____________________________________
ENM:1	 ______	     _     _      _     _      _	
         0 _| >5ms |_| |_| |_| |_| |_| |________
*/	
static S32 sgm3785_set_torch_mode(U16 duty)
{
	S32 ret;

	if(active_flag == KAL_TRUE)
		return;

	if(mt_set_gpio_mode(ENF,GPIO_MODE_00)){sgm3138_dbg("[sgm3785] set enf gpio mode failed!! \n");}
	if(mt_set_gpio_dir(ENF,GPIO_DIR_OUT)){sgm3138_dbg("[sgm3785] set  enf gpio dir failed!! \n");}
	mt_set_gpio_out(ENF,GPIO_OUT_ZERO);

	if(mt_set_gpio_mode(ENM,GPIO_MODE_00)){sgm3138_dbg("[sgm3785] set enf gpio mode failed!! \n");}
	if(mt_set_gpio_dir(ENM,GPIO_DIR_OUT)){sgm3138_dbg("[sgm3785] set  enf gpio dir failed!! \n");}
	mt_set_gpio_out(ENM,GPIO_OUT_ZERO);
	mdelay(1);
	mt_set_gpio_out(ENM,GPIO_OUT_ONE);
	mdelay(6);

	if(mt_set_gpio_mode(ENM,ENM_PWM_MODE)){sgm3138_dbg("[sgm3785] set enf gpio mode failed!! \n");}
	sgm3785_config.PWM_MODE_OLD_REGS.THRESH = duty;
	ret = pwm_set_spec_config(&sgm3785_config);

	active_flag = KAL_TRUE;
	return ret;
}

static S32 sgm3785_shutdown(void)
{
	if(active_flag == KAL_FALSE)
		return;

	mt_pwm_disable(PWM_NO, false);

	if(mt_set_gpio_mode(ENF,GPIO_MODE_00)){sgm3138_dbg("[sgm3785] set enf gpio mode failed!! \n");}
	if(mt_set_gpio_mode(ENM,GPIO_MODE_00)){sgm3138_dbg("[sgm3785] set enf gpio mode failed!! \n");}
	if(mt_set_gpio_dir(ENF,GPIO_DIR_OUT)){sgm3138_dbg("[sgm3785] set  enf gpio dir failed!! \n");}
	if(mt_set_gpio_dir(ENM,GPIO_DIR_OUT)){sgm3138_dbg("[sgm3785] set  enf gpio dir failed!! \n");}
	mt_set_gpio_out(ENF,GPIO_OUT_ZERO);
	mt_set_gpio_out(ENM,GPIO_OUT_ZERO);
	mdelay(5);

	active_flag = KAL_FALSE;
}

int sgm3785_ioctr(U16 mode, U16 duty)
{
	sgm3138_dbg("[sgm3785] mode %d , duty %d\n", mode, duty);

	if(mode < MODE_MIN || mode >= MODE_MAX)
	{
		sgm3138_dbg("[sgm3785] mode error!!!\n");
		return 1;
	}
	
	if(duty < 0 || duty > 10)
	{
		sgm3138_dbg("[sgm3785] duty error!!!\n");
		return 1;
	}

	switch(mode){
		case FLASH_M:
			sgm3785_set_flash_mode(duty);
			break;

		case TORCH_M:
			sgm3785_set_torch_mode(duty);
			break;

		case PWD_M:
			sgm3785_shutdown();
			break;

		default:
			sgm3138_dbg("[sgm3785] error ioctr!!!\n");
			break;
	}

	return 0;
}

void sgm3785_FL_Enable(int duty)
{
	if(duty > 0)//flashlight mode
		sgm3785_ioctr(FLASH_M, F_DUTY);
	else //torch mode
		sgm3785_ioctr(TORCH_M, T_DUTY);
}

void sgm3785_FL_Disable(void)
{
	sgm3785_ioctr(PWD_M, 0);
}

