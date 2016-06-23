#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/aee.h>
#include <linux/xlog.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/earlysuspend.h>
#include <linux/seq_file.h>
#include <linux/time.h>

#include <asm/uaccess.h>

#include <mach/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>
#include <mach/mt_pmic_wrap.h>
#include <mach/mt_gpio.h>
#include <mach/mtk_rtc.h>
#include <mach/mt_spm_mtcmos.h>

#include <mach/battery_common.h>
#include <mach/pmic_mt6325_sw.h>
#include <mach/mt6311.h>

#include <cust_pmic.h>
#include <cust_battery_meter.h>

#include <linux/rtc.h>

#include <pmic_dvt.h>

extern kal_uint32 PMIC_IMM_GetOneChannelValueMD(kal_uint8 dwChannel, int deCount, int trimd);
extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
extern void upmu_set_reg_value(kal_uint32 reg, kal_uint32 reg_val);

extern kal_uint16 mt6328_set_register_value(PMU_FLAGS_LIST_ENUM flagname,kal_uint32 val);
extern kal_uint16 mt6328_get_register_value(PMU_FLAGS_LIST_ENUM flagname);

void setSrclkenMode(kal_uint8 mode);

void pmic6328_auxadc1_basic111(void)
{

	int i=0;
	int ret;

	printk("[pmic6328_auxadc1_basic111]\n");
	for (i=0;i<16;i++)
	{
		printk("[%d]=%d\n",i,PMIC_IMM_GetOneChannelValue(i,0,0));
	}
/*
	printk("[md0]=%d\n",PMIC_IMM_GetOneChannelValueMD(0,0,0));
	printk("[md1]=%d\n",PMIC_IMM_GetOneChannelValueMD(1,0,0));
	printk("[md4]=%d\n",PMIC_IMM_GetOneChannelValueMD(4,0,0));
	printk("[md7]=%d\n",PMIC_IMM_GetOneChannelValueMD(7,0,0));
	printk("[md8]=%d\n",PMIC_IMM_GetOneChannelValueMD(8,0,0));
*/
}


#define AUXADC_EFUSE_GAIN 0xff
#define AUXADC_EFUSE_OFFSET 0xff

/*
//for ch7 15bit case , output is 15bit. 
Y[14:0]= Y0[14:0]*(1+GE[11:0]/32768)+ OE[10:0]

//for other 12bit case , output is 12bit. 
Y1[14:0]= {Y0[11:0],3'h0 }*(1+GE[11:0]/32768)+ OE[10:0]
Y[11:0]=Y1[14:3]

*/

kal_uint8 pmic_auxadc_trim_check(kal_uint32 trimdata,kal_uint32 rawdata,kal_uint8 type,kal_uint32 offset,kal_uint32 gain)
{
	kal_uint32 trimdata2,rawdata2,rawdata3,rawdata4;
	kal_uint32 rawdatabackup;

	if (type==0)// 15bit
	{
		trimdata2=rawdata+offset+(rawdata*gain/32768);	
		 printk("[pmic_auxadc_trim_check][15bit] rawdata=%d trimdata=%d trimdata2=%d diff=%d offset=%d gain=%d\n", rawdata,trimdata,trimdata2,trimdata2-trimdata,offset,gain);
	}
	else
	{
		rawdata2=rawdata;
		rawdata3=rawdata2+offset+(rawdata2*gain/32768);
		rawdata4=rawdata3>>3;
		printk("[pmic_auxadc_trim_check][12bit] rawdata=%d:%d:%d:%d trimdata=%d diff=%d offset=%d gain=%d\n",
			rawdata,rawdata2,rawdata3,rawdata4,trimdata,rawdata4-trimdata,offset,gain);
	}

}



void pmic6328_auxadc2_trim111(int ch)
{
	int i=0,j=ch;
	kal_uint32 trimdata,rawdata;
	kal_uint32 gain0=0x60,offset0=0x60;
	kal_uint32 gain1=0xff,offset1=0xff;
	kal_uint32 gain2=0x3ff,offset2=0x3ff;
	kal_uint32 gain3=0xfff,offset3=0x7ff;	

	printk("[pmic6328_auxadc2_trim111]\n");
	
	mt6328_set_register_value(PMIC_EFUSE_GAIN_CH0_TRIM,gain0);
	mt6328_set_register_value(PMIC_EFUSE_OFFSET_CH0_TRIM,offset0);

	mt6328_set_register_value(PMIC_EFUSE_GAIN_CH4_TRIM,gain1);
	mt6328_set_register_value(PMIC_EFUSE_OFFSET_CH4_TRIM,offset1);

	mt6328_set_register_value(PMIC_EFUSE_GAIN_CH7_TRIM,gain2);
	mt6328_set_register_value(PMIC_EFUSE_OFFSET_CH7_TRIM,offset2);

	mt6328_set_register_value(PMIC_AUXADC_SW_GAIN_TRIM,gain3);
	mt6328_set_register_value(PMIC_AUXADC_SW_OFFSET_TRIM,offset3);	

	for (i=0;i<4;i++)
	{
		printk("[trim_sel]:%d\n",i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH0_SEL,i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH1_SEL,i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH2_SEL,i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH3_SEL,i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH4_SEL,i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH5_SEL,i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH6_SEL,i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH7_SEL,i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH8_SEL,i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH9_SEL,i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH10_SEL,i);
		mt6328_set_register_value(PMIC_AUXADC_TRIM_CH11_SEL,i);			
		//for (j=0;j<11;j++)
		{
			
			trimdata=PMIC_IMM_GetOneChannelValue(j,0,0);
			rawdata=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_RAW);
			
			if (j==0 || j==1 || j==7)
			{
				if(i==0)
				{				
					pmic_auxadc_trim_check(trimdata,rawdata,0,offset0,gain0);
				}
				else if(i==1)
				{				
					pmic_auxadc_trim_check(trimdata,rawdata,0,offset1,gain1);
				}
				else if(i==2)
				{				
					pmic_auxadc_trim_check(trimdata,rawdata,0,offset2,gain2);
				}
				else if(i==3)
				{				
					pmic_auxadc_trim_check(trimdata,rawdata,0,offset3,gain3);
				}				
			}
			else
			{
				if(i==0)
				{				
					pmic_auxadc_trim_check(trimdata,rawdata,1,offset0,gain0);
				}
				else if(i==1)
				{				
					pmic_auxadc_trim_check(trimdata,rawdata,1,offset1,gain1);
				}
				else if(i==2)
				{				
					pmic_auxadc_trim_check(trimdata,rawdata,1,offset2,gain2);
				}
				else if(i==3)
				{				
					pmic_auxadc_trim_check(trimdata,rawdata,1,offset3,gain3);
				}	
			}
		}	
	}

}

void pmic6328_set_32k(void)
{
	mt6328_set_register_value(PMIC_RG_AUXADC_CK_TSTSEL,1);	
}


kal_int32 pmicno[20];
kal_int32 pmicval[20];
kal_int32 pmiciv[20];

void pmic6328_priority_check(kal_uint16 ap,kal_uint16 md)
{
	kal_uint32 i=0;

	kal_uint8 idx=1;



	for(i=0;i<20;i++)
	{
		pmicno[i]=0;
		pmicval[i]=0;
		pmiciv[i]=0;
	}

	i=0;

	do
	{
		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH0_BY_AP) == 1 && pmicno[0]==0 && (ap&(1<<0))   )
		{
			pmicval[0]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH0_BY_AP); 
			pmicno[0]=idx;
			pmiciv[0]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH1_BY_AP) == 1&& pmicno[1]==0 && (ap&(1<<1))   )
		{
			pmicval[1]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH1_BY_AP); 
			pmicno[1]=idx;
			pmiciv[1]=i;
			idx=idx+1;
		}
		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH2) == 1&& pmicno[2]==0&& (ap&(1<<2))   )
		{
			pmicval[2]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH2); 
			pmicno[2]=idx;
			pmiciv[2]=i;
			idx=idx+1;
		}
		
		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH3) == 1&& pmicno[3]==0&& (ap&(1<<3))   )
		{
			pmicval[3]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH3); 
			pmicno[3]=idx;
			pmiciv[3]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH4) == 1&& pmicno[4]==0&& (ap&(1<<4))   )
		{
			pmicval[4]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH4); 
			pmicno[4]=idx;
			pmiciv[4]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH5) == 1&& pmicno[5]==0&& (ap&(1<<5))   )
		{
			pmicval[5]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH5); 
			pmicno[5]=idx;
			pmiciv[5]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH6) == 1&& pmicno[6]==0&& (ap&(1<<6))   )
		{
			pmicval[6]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH6); 
			pmicno[6]=idx;
			pmiciv[6]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH7_BY_AP) == 1&& pmicno[7]==0&& (ap&(1<<7))   )
		{
			pmicval[7]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH7_BY_AP); 
			pmicno[7]=idx;
			pmiciv[7]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH8) == 1&& pmicno[8]==0&& (ap&(1<<8))   )
		{
			pmicval[8]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH8); 
			pmicno[8]=idx;
			pmiciv[8]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH9) == 1&& pmicno[9]==0&& (ap&(1<<9))   )
		{
			pmicval[9]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH9); 
			pmicno[9]=idx;
			pmiciv[9]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH10) == 1&& pmicno[10]==0&& (ap&(1<<10))   )
		{
			pmicval[10]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH10); 
			pmicno[10]=idx;
			pmiciv[10]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH11) == 1&& pmicno[11]==0&& (ap&(1<<11))   )
		{
			pmicval[11]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH11); 
			pmicno[11]=idx;
			pmiciv[11]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH12_15) == 1&& pmicno[12]==0&& (ap&(1<<12))   )
		{
			pmicval[12]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH12_15); 
			pmicno[12]=idx;
			pmiciv[12]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH0_BY_MD) == 1&& pmicno[13]==0&& (md&(1<<0))   )
		{
			pmicval[13]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH0_BY_MD); 
			pmicno[13]=idx;
			pmiciv[13]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH1_BY_MD) == 1&& pmicno[14]==0&& (md&(1<<1))   )
		{
			pmicval[14]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH1_BY_MD); 
			pmicno[14]=idx;
			pmiciv[14]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH4_BY_MD) == 1&& pmicno[15]==0&& (md&(1<<4))   )
		{
			pmicval[15]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH4_BY_MD); 
			pmicno[15]=idx;
			pmiciv[15]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH7_BY_MD) == 1&& pmicno[16]==0&& (md&(1<<7))   )
		{
			pmicval[16]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH7_BY_MD); 
			pmicno[16]=idx;
			pmiciv[16]=i;
			idx=idx+1;
		}

		if(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH7_BY_GPS) == 1&& pmicno[17]==0&& (md&(1<<8))   )
		{
			pmicval[17]=mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH7_BY_GPS); 
			pmicno[17]=idx;
			pmiciv[17]=i;
			idx=idx+1;
		}

		i++;
	}while(i<5000);

	printk("[ch00_AP]=%2d data=%d i=%d\n",pmicno[0],pmicval[0],pmiciv[0]);
	printk("[ch01_AP]=%2d data=%d i=%d\n",pmicno[1],pmicval[1],pmiciv[1]);
	printk("[ch02_AP]=%2d data=%d i=%d\n",pmicno[2],pmicval[2],pmiciv[2]);
	printk("[ch03_AP]=%2d data=%d i=%d\n",pmicno[3],pmicval[3],pmiciv[3]);
	printk("[ch04_AP]=%2d data=%d i=%d\n",pmicno[4],pmicval[4],pmiciv[4]);
	printk("[ch05_AP]=%2d data=%d i=%d\n",pmicno[5],pmicval[5],pmiciv[5]);
	printk("[ch06_AP]=%2d data=%d i=%d\n",pmicno[6],pmicval[6],pmiciv[6]);
	printk("[ch07_AP]=%2d data=%d i=%d\n",pmicno[7],pmicval[7],pmiciv[7]);
	printk("[ch08_AP]=%2d data=%d i=%d\n",pmicno[8],pmicval[8],pmiciv[8]);
	printk("[ch09_AP]=%2d data=%d i=%d\n",pmicno[9],pmicval[9],pmiciv[9]);
	printk("[ch10_AP]=%2d data=%d i=%d\n",pmicno[10],pmicval[10],pmiciv[10]);
	printk("[ch11_AP]=%2d data=%d i=%d\n",pmicno[11],pmicval[11],pmiciv[11]);
	printk("[ch12_AP]=%2d data=%d i=%d\n",pmicno[12],pmicval[12],pmiciv[12]);	
	printk("[ch00_MD]=%2d data=%d i=%d\n",pmicno[13],pmicval[13],pmiciv[13]);
	printk("[ch01_MD]=%2d data=%d i=%d\n",pmicno[14],pmicval[14],pmiciv[14]);
	printk("[ch04_MD]=%2d data=%d i=%d\n",pmicno[15],pmicval[15],pmiciv[15]);
	printk("[ch07_MD]=%2d data=%d i=%d\n",pmicno[16],pmicval[16],pmiciv[16]);
	printk("[ch07_GPS]=%2d data=%d i=%d\n",pmicno[17],pmicval[17],pmiciv[17]);	



}



void pmic6328_auxadc3_prio111(void)
{
	kal_int32 ret=0;
	pmic6328_set_32k();
	printk("[pmic6328_auxadc3_prio111] %d\n",upmu_get_reg_value(0xe18));
	ret=pmic_config_interface(MT6328_AUXADC_RQST0_SET,0xffff,0xffff,0);
	ret=pmic_config_interface(MT6328_AUXADC_RQST1_SET,0x193,0xffff,0);

	udelay(100);

	pmic6328_priority_check(0x1fff,0x193);

}

void pmic6328_auxadc3_prio112(void)
{
	kal_int32 ret=0;
	pmic6328_set_32k();
	printk("[pmic6328_auxadc3_prio112]\n");
	ret=pmic_config_interface(MT6328_AUXADC_RQST0_SET,0x7f,0xffff,0);
	udelay(1);
	ret=pmic_config_interface(MT6328_AUXADC_RQST1_SET,0x190,0xffff,0);

	udelay(100);
	
	pmic6328_priority_check(0x7f,0x190);

}

void pmic6328_auxadc3_prio113(void)
{
	kal_int32 ret=0;
	pmic6328_set_32k();
	printk("[pmic6328_auxadc3_prio113]\n");
	ret=pmic_config_interface(MT6328_AUXADC_RQST1_SET,0x100,0xffff,0);
	udelay(1);
	ret=pmic_config_interface(MT6328_AUXADC_RQST1_SET,0x0080,0xffff,0);

	udelay(100);
	
	pmic6328_priority_check(0x00,0x0180);

}

void pmic6328_auxadc3_prio114(void)
{
	kal_int32 ret=0;
	kal_uint32 i=0,j=0;
	kal_uint8 v1,v2;
	pmic6328_set_32k();
	printk("[pmic6328_auxadc3_prio114]\n");
	ret=pmic_config_interface(MT6328_AUXADC_RQST1_SET,0x100,0xffff,0);

	udelay(100);
	while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH7_BY_GPS) != 1 )
	{
		i++;
	}

	ret=pmic_config_interface(MT6328_AUXADC_RQST1_SET,0x80,0xffff,0);
	while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH7_BY_MD) != 1 )
	{
		j++;
	}

	v1= mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH7_BY_GPS); 		
	v2= mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH7_BY_MD); 


	printk("[pmic6328_auxadc3_prio114]i=%d j=%d v1=%d v2;%d\n",i,j,v1,v2);	


}

void pmic6328_auxadc3_prio115(void)
{
	kal_int32 ret=0;
	pmic6328_set_32k();
	printk("[pmic6328_auxadc3_prio115]\n");

	ret=pmic_config_interface(MT6328_AUXADC_RQST1_SET,0x180,0xffff,0);

	ret=pmic_config_interface(MT6328_AUXADC_RQST0_SET,0xffd,0xffff,0);
	udelay(1);
	ret=pmic_config_interface(MT6328_AUXADC_RQST0_SET,0x2,0xffff,0);

	udelay(100);
	pmic6328_priority_check(0xfff,0x180);
}

//LBAT1.1.1 : Test low battery voltage interrupt
void pmic6328_lbat111(void)
{
	int i=0;

	setSrclkenMode(1);
	printk("[LBAT1.1.1 : Test low battery voltage interrupt]\n");
	// 0:set interrupt 
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,1);

	// 1:setup max voltage threshold 4.2v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MAX,0x0c91);

	// 2.setup min voltage threshold 3.4v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,0x018e);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,0x0aa0);	

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DET_PRD_15_0,0x7d0);

	// 4.setup max/min debounce time
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MAX,0x3);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,1);


	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_BAT_L)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_LBAT_DEBOUNCE_COUNT_MIN)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_LBAT)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_LBAT));
		i++;
		if (i>=50)
			break;
	}


	// 7. Monitor Debounce counts

	mt6328_get_register_value(PMIC_AUXADC_LBAT_DEBOUNCE_COUNT_MIN);
	
	//ReadReg(AUXADC_LBAT_DEBOUNCE_COUNT_MAX);
	// ReadReg(BAUXADC_LBAT_DEBOUNCE_COUNT_MIN);
	// 8. Read LowBattery Detect Value
	//while(! ReadReg(BRW_AUXADC_ADC_RDY_LBAT));
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_7_0);
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_9_8);
    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_H,0);


}



//LBAT1.1.2 : Test high battery voltage interrupt 
void pmic6328_lbat112(void)
{
	int i=0;
	setSrclkenMode(1);
	printk("[LBAT1.1.2 : Test high battery voltage interrupt ]\n");
	// 0:set interrupt 
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,1);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT_H,1);	

	// 1:setup max voltage threshold 4.2v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MAX,0x0c91);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MAX,0x0a60);

	// 2.setup min voltage threshold 3.4v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,0x018e);

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DET_PRD_15_0,0x7d0);

	// 4.setup max/min debounce time
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MAX,0x3);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0x1);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,1);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);

	// 7. Monitor Debounce counts
	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_BAT_H)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_LBAT_DEBOUNCE_COUNT_MAX)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_LBAT)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_LBAT));

		i++;
		if (i>=50)
			break;		
	}

    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT_H,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,0);
	

}



//LBAT1.1.3 : test low battery voltage interrupt  in sleep mode
void pmic6328_lbat113(void)
{
	int i=0;
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SW,1);
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SEL,1);

	printk("[LBAT1.1.3 : test low battery voltage interrupt  in sleep mode]\n");
	// 0:set interrupt 
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,1);

	// 1:setup max voltage threshold 4.2v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MAX,0x0c91);

	// 2.setup min voltage threshold 3.4v
	mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,0x0aa0);

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DET_PRD_15_0,0x7d0);

	// 4.setup max/min debounce time
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MAX,0x3);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,1);


	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_BAT_L)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_LBAT_DEBOUNCE_COUNT_MIN)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_LBAT)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_LBAT));
		i++;
		if (i>=50)
			break;
		
	}

	// 7. Monitor Debounce counts

	mt6328_get_register_value(PMIC_AUXADC_LBAT_DEBOUNCE_COUNT_MIN);
	
	//ReadReg(AUXADC_LBAT_DEBOUNCE_COUNT_MAX);
	// ReadReg(BAUXADC_LBAT_DEBOUNCE_COUNT_MIN);
	// 8. Read LowBattery Detect Value
	//while(! ReadReg(BRW_AUXADC_ADC_RDY_LBAT));
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_7_0);
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_9_8);
    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_H,0);

}

//LBAT1.1.4 : test high battery voltage interrupt in sleep mode
void pmic6328_lbat114(void)
{
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SW,1);
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SEL,1);
	
	printk("[LBAT1.1.4 : test high battery voltage interrupt in sleep mode ]\n");
	// 0:set interrupt 
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,1);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT_H,1);	

	// 1:setup max voltage threshold 4.2v
	mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MAX,0x0a60);

	// 2.setup min voltage threshold 3.4v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,0x018e);

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DET_PRD_15_0,0x7d0);

	// 4.setup max/min debounce time
	mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MAX,0x3);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0x1);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,1);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);

	// 7. Monitor Debounce counts
	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_BAT_H)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_LBAT_DEBOUNCE_COUNT_MAX)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_LBAT)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_LBAT));
	}

    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT_H,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,0);
	
}

//LBAT2 1.1.1 : Test low battery voltage interrupt
void pmic6328_lbat2111(void)
{
	int i=0;

	setSrclkenMode(1);
	printk("[LBAT2 1.1.1 : Test low battery voltage interrupt]\n");
	// 0:set interrupt 
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT2_L,1);

	// 1:setup max voltage threshold 4.2v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MAX,0x0c91);

	// 2.setup min voltage threshold 3.4v
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_VOLT_MIN,0x0aa0);

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DET_PRD_15_0,0x7d0);

	// 4.setup max/min debounce time
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MAX,0x3);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_EN_MIN,1);


	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_BAT2_L)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_LBAT2_DEBOUNCE_COUNT_MIN)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_LBAT2)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_LBAT2));
		i++;
		if (i>=50)
			break;

	}


	// 7. Monitor Debounce counts

	mt6328_get_register_value(PMIC_AUXADC_LBAT2_DEBOUNCE_COUNT_MIN);
	
	//ReadReg(AUXADC_LBAT_DEBOUNCE_COUNT_MAX);
	// ReadReg(BAUXADC_LBAT_DEBOUNCE_COUNT_MIN);
	// 8. Read LowBattery Detect Value
	//while(! ReadReg(BRW_AUXADC_ADC_RDY_LBAT));
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_7_0);
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_9_8);
    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_IRQ_EN_MIN,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT2_L,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_H,0);


}



//LBAT1.1.2 : Test high battery voltage interrupt 
void pmic6328_lbat2112(void)
{
	int i=0;

	setSrclkenMode(1);
	printk("[LBAT2 1.1.2 : Test high battery voltage interrupt ]\n");
	// 0:set interrupt 
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,1);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT2_H,1);	

	// 1:setup max voltage threshold 4.2v
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_VOLT_MAX,0x0a60);

	// 2.setup min voltage threshold 3.4v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,0x018e);

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DET_PRD_15_0,0x7d0);

	// 4.setup max/min debounce time
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DEBT_MAX,0x3);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_IRQ_EN_MAX,0x1);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_EN_MAX,1);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);

	// 7. Monitor Debounce counts
	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_BAT2_H)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_LBAT2_DEBOUNCE_COUNT_MAX)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_LBAT2)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_LBAT2));

		i++;
		if (i>=50)
			break;
	}

    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_IRQ_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT2_H,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,0);
	


}

//LBAT1.1.3 : test low battery voltage interrupt  in sleep mode
void pmic6328_lbat2113(void)
{
	int i=0;
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SW,1);
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SEL,1);

	printk("[LBAT1.1.3 : test low battery voltage interrupt  in sleep mode]\n");
	// 0:set interrupt 
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT2_L,1);

	// 1:setup max voltage threshold 4.2v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MAX,0x0c91);

	// 2.setup min voltage threshold 3.4v
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_VOLT_MIN,0x0aa0);

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DET_PRD_15_0,0x7d0);

	// 4.setup max/min debounce time
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MAX,0x3);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_EN_MIN,1);


	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_BAT2_L)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_LBAT2_DEBOUNCE_COUNT_MIN)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_LBAT2)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_LBAT2));
		i++;
		if (i>=50)
			break;

	}

	// 7. Monitor Debounce counts

	mt6328_get_register_value(PMIC_AUXADC_LBAT2_DEBOUNCE_COUNT_MIN);
	
	//ReadReg(AUXADC_LBAT_DEBOUNCE_COUNT_MAX);
	// ReadReg(BAUXADC_LBAT_DEBOUNCE_COUNT_MIN);
	// 8. Read LowBattery Detect Value
	//while(! ReadReg(BRW_AUXADC_ADC_RDY_LBAT));
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_7_0);
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_9_8);
    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_IRQ_EN_MIN,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT2_L,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_H,0);

}

//LBAT2 1.1.4 : test high battery voltage interrupt in sleep mode
void pmic6328_lbat2114(void)
{

	int i=0;
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SW,1);
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SEL,1);

	printk("[LBAT2 1.1.4 : test high battery voltage interrupt in sleep mode ]\n");
	// 0:set interrupt 
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,1);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT2_H,1);	

	// 1:setup max voltage threshold 4.2v
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_VOLT_MAX,0x0a60);

	// 2.setup min voltage threshold 3.4v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,0x018e);

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DET_PRD_15_0,0x7d0);

	// 4.setup max/min debounce time
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_DEBT_MAX,0x3);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_IRQ_EN_MAX,0x1);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_EN_MAX,1);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);

	// 7. Monitor Debounce counts
	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_BAT2_H)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_LBAT2_DEBOUNCE_COUNT_MAX)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_LBAT2)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_LBAT2));

		i++;
		if (i>=50)
			break;
	}

    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_IRQ_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0);
	mt6328_set_register_value(PMIC_AUXADC_LBAT2_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_BAT2_H,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,0);
	
}

//THR1.1.1 : Thermal interrupt for over temperature
void pmic6328_thr111(void)
{
	printk("[THR1.1.1 : Thermal interrupt for over temperature]\n");
	// 0:set interrupt 
	mt6328_set_register_value(PMIC_RG_INT_EN_THR_L,1);

	// 1:setup max voltage threshold 4.2v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MAX,0x0c91);

	// 2.setup min voltage threshold 3.4v
	mt6328_set_register_value(PMIC_AUXADC_THR_VOLT_MIN,0x018e);

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_THR_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_THR_DET_PRD_15_0,0x00a);

	// 4.setup max/min debounce time
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MAX,0x3);
	mt6328_set_register_value(PMIC_AUXADC_THR_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0x0);
	mt6328_set_register_value(PMIC_AUXADC_THR_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_THR_EN_MIN,1);


	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_THR_L)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_THR_DEBOUNCE_COUNT_MIN)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_THR_HW)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_THR_HW));
	}


	// 7. Monitor Debounce counts

	mt6328_get_register_value(PMIC_AUXADC_THR_DEBOUNCE_COUNT_MIN);
	
	//ReadReg(AUXADC_LBAT_DEBOUNCE_COUNT_MAX);
	// ReadReg(BAUXADC_LBAT_DEBOUNCE_COUNT_MIN);
	// 8. Read LowBattery Detect Value
	//while(! ReadReg(BRW_AUXADC_ADC_RDY_LBAT));
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_7_0);
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_9_8);
    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_THR_IRQ_EN_MIN,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_THR_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_THR_L,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_H,0);


}



//THR1.1.2 : Thermal interrupt for under temperature
void pmic6328_thr112(void)
{
	printk("[THR1.1.2 : Thermal interrupt for under temperature ]\n");
	// 0:set interrupt 
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,1);
	mt6328_set_register_value(PMIC_RG_INT_EN_THR_H,1);	

	// 1:setup max voltage threshold 4.2v
	mt6328_set_register_value(PMIC_AUXADC_THR_VOLT_MAX,0x0c91);

	// 2.setup min voltage threshold 3.4v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,0x018e);

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_THR_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_THR_DET_PRD_15_0,0x00a);

	// 4.setup max/min debounce time
	mt6328_set_register_value(PMIC_AUXADC_THR_DEBT_MAX,0x3);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	mt6328_set_register_value(PMIC_AUXADC_THR_IRQ_EN_MAX,0x0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	mt6328_set_register_value(PMIC_AUXADC_THR_EN_MAX,1);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);

	// 7. Monitor Debounce counts
	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_THR_H)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_THR_DEBOUNCE_COUNT_MIN)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_THR_HW)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_THR_HW));
	}

    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	mt6328_set_register_value(PMIC_AUXADC_THR_IRQ_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0);
	mt6328_set_register_value(PMIC_AUXADC_THR_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_THR_H,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,0);
	


}

//THR1.1.3 : Thermal interrupt  in sleep mode
void pmic6328_thr113(void)
{
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SW,1);
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SEL,1);

	printk("[THR1.1.3 : Thermal interrupt min  in sleep mode]\n");
	// 0:set interrupt 
	mt6328_set_register_value(PMIC_RG_INT_EN_THR_L,1);

	// 1:setup max voltage threshold 4.2v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MAX,0x0c91);

	// 2.setup min voltage threshold 3.4v
	mt6328_set_register_value(PMIC_AUXADC_THR_VOLT_MIN,0x018e);

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_THR_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_THR_DET_PRD_15_0,0x00a);

	// 4.setup max/min debounce time
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MAX,0x3);
	mt6328_set_register_value(PMIC_AUXADC_THR_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0x0);
	mt6328_set_register_value(PMIC_AUXADC_THR_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_THR_EN_MIN,1);


	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_THR_L)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_THR_DEBOUNCE_COUNT_MIN)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_THR_HW)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_THR_HW));
	}

	// 7. Monitor Debounce counts

	mt6328_get_register_value(PMIC_AUXADC_THR_DEBOUNCE_COUNT_MIN);
	
	//ReadReg(AUXADC_LBAT_DEBOUNCE_COUNT_MAX);
	// ReadReg(BAUXADC_LBAT_DEBOUNCE_COUNT_MIN);
	// 8. Read LowBattery Detect Value
	//while(! ReadReg(BRW_AUXADC_ADC_RDY_LBAT));
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_7_0);
	//ReadReg(BRW_AUXADC_ADC_OUT_LBAT_9_8);
    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_THR_IRQ_EN_MIN,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,0);
	mt6328_set_register_value(PMIC_AUXADC_THR_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_THR_L,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_H,0);

}

//THR1.1.4 : Thermal interrupt in sleep mode
void pmic6328_thr114(void)
{
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SW,1);
	mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SEL,1);
	
	printk("[THR1.1.4 : Thermal interrupt in sleep mode ]\n");
	// 0:set interrupt 
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,1);
	mt6328_set_register_value(PMIC_RG_INT_EN_THR_H,1);	

	// 1:setup max voltage threshold 4.2v
	mt6328_set_register_value(PMIC_AUXADC_THR_VOLT_MAX,0x0c91);

	// 2.setup min voltage threshold 3.4v
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,0x018e);

	// 3.setup detection period
	mt6328_set_register_value(PMIC_AUXADC_THR_DET_PRD_19_16,0x0);
	mt6328_set_register_value(PMIC_AUXADC_THR_DET_PRD_15_0,0x00a);

	// 4.setup max/min debounce time
	mt6328_set_register_value(PMIC_AUXADC_THR_DEBT_MAX,0x3);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_DEBT_MIN,0x3);	

	// 5.turn on IRQ
	mt6328_set_register_value(PMIC_AUXADC_THR_IRQ_EN_MAX,0x0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0x1);		

	// 6. turn on LowBattery Detection
	mt6328_set_register_value(PMIC_AUXADC_THR_EN_MAX,1);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);

	// 7. Monitor Debounce counts
	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_THR_H)!=1)
	{
		msleep(1000);
		printk("debounce count:%d %d %d\n",mt6328_get_register_value(PMIC_AUXADC_THR_DEBOUNCE_COUNT_MIN)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_THR_HW)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_THR_HW));
	}

    
	// 9. Test on VBAT = 3.5 -> 3.4 -> 3.3 and receive interrupt

	//10. Clear interrupt after receiving interrupt
	mt6328_set_register_value(PMIC_AUXADC_THR_IRQ_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,0);
	mt6328_set_register_value(PMIC_AUXADC_THR_EN_MAX,0);
	//mt6328_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,0);
	mt6328_set_register_value(PMIC_RG_INT_EN_THR_H,0);
	//mt6328_set_register_value(PMIC_RG_INT_EN_BAT_L,0);
	
}


//ACC1.1.1 accdet auto sampling
void pmic6328_acc111(void)
{
	kal_uint32 ret_data;
	kal_uint32 i=0,j=0;
	printk("[ACC1.1.1 accdet auto sampling]\n");
	mt6328_set_register_value(PMIC_AUXADC_ACCDET_AUTO_SPL,1);

	while(i<20)
	{
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH5) != 1 );
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH5);
		printk("[ACC1.1.1 accdet auto sampling]idx=%d data=%d \n",i,ret_data);
		i++;
		msleep(10);
	}

}

//IMP1.1.1 Impedance measurement for BATSNS
void pmic6328_IMP111(void)
{
	kal_uint32 ret_data;
	kal_uint32 i=0;
	//setSrclkenMode(1);
	printk("[IMP1.1.1 Impedance measurement for BATSNS]:%d \n",mt6328_get_register_value(PMIC_RG_INT_STATUS_AUXADC_IMP));

	//clear IRQ
	mt6328_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,1);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,1);	

	//restore to initial state
	mt6328_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,0);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,0);	

	//set issue interrupt
	mt6328_set_register_value(PMIC_RG_INT_EN_AUXADC_IMP,1);
	
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_CHSEL,0);
	mt6328_set_register_value(PMIC_AUXADC_IMP_AUTORPT_EN,1);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_CNT,4);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_MODE,1);

	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_AUXADC_IMP)!=1)
	{
		//polling IRQ
		printk("rdy bit:%d \n",mt6328_get_register_value(PMIC_RG_INT_STATUS_AUXADC_IMP));

	}

	printk("rdy bit:%d i:%d\n",mt6328_get_register_value(PMIC_RG_INT_STATUS_AUXADC_IMP),i);
/*
	//clear IRQ
	mt6328_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,1);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,1);	

	//restore to initial state
	mt6328_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,0);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,0);		

	//turn off interrupt
	mt6328_set_register_value(PMIC_RG_INT_EN_AUXADC_IMP,0);
*/
}


//IMP1.1.2 Impedance measurement for ISENSE
void pmic6328_IMP112(void)
{
	kal_uint32 ret_data;
	kal_uint32 i=0;
	//setSrclkenMode(1);
	printk("[IMP1.1.2 Impedance measurement for ISENSE] %d \n",mt6328_get_register_value(PMIC_RG_INT_STATUS_AUXADC_IMP));

	//clear IRQ
	mt6328_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,1);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,1);	

	//restore to initial state
	mt6328_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,0);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,0);	

	//set issue interrupt
	mt6328_set_register_value(PMIC_RG_INT_EN_AUXADC_IMP,1);
	
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_CHSEL,1);
	mt6328_set_register_value(PMIC_AUXADC_IMP_AUTORPT_EN,1);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_CNT,4);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_MODE,1);

/*
	while(mt6328_get_register_value(PMIC_RG_INT_STATUS_AUXADC_IMP)!=1 && i<15)
	{
		//polling IRQ
		msleep(1000);
		printk("rdy bit:%d \n",mt6328_get_register_value(PMIC_RG_INT_STATUS_AUXADC_IMP));
		i++;

	}

	printk("rdy bit:%d i:%d\n",mt6328_get_register_value(PMIC_RG_INT_STATUS_AUXADC_IMP),i);


	//clear IRQ
	mt6328_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,1);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,1);	

	//restore to initial state
	mt6328_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,0);
	mt6328_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,0);		

	//turn off interrupt
	mt6328_set_register_value(PMIC_RG_INT_EN_AUXADC_IMP,0);
*/
}

//CCK.1.1 Real-time & wakeup measurement 
void pmic6328_cck11(void)
{
	kal_uint32 ret_data;
	kal_uint32 i=0;
	printk("[CCK.1.1 Real-time & wakeup measurement ]\n");

	mt6328_set_register_value(PMIC_RG_BUCK_OSC_SEL_SW,0); // 0: power on; 1: power down smps aud clock
	mt6328_set_register_value(PMIC_RG_BUCK_OSC_SEL_MODE,1); // SW mode
	mdelay(1);


	setSrclkenMode(1);

	//set issue interrupt
	mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_WKUP_EN,1);
	mdelay(1);
	
	
	//define period of measurement
	//mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_PRD,10);

	//setting of enabling measurement 
	mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_EN,0);



	setSrclkenMode(0);

	setSrclkenMode(1);


	printk("[wake up]\n");
	while(i<20)
	{
		printk("i:%d rdy:%d out:%d\n",i,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_MDRT)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_MDRT));
		udelay(1000);
		i++;

	}



	i=0;
// Check AUXADC_MDRT_DET function
           //define period of measurement
           mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_PRD,10); // 5ms, AUXADC_ADC_OUT_MDRT will update periodically based on this time 

           //setting of enabling measurement 
           mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_EN,1);


	printk("[real time]\n");
	while(i<20)
	{	
		printk("i:%d rdy:%d out:%d\n",i,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_MDRT)
						  		 ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_MDRT));
		udelay(1000);
		i++;

	}

}


void pmic6328_cck13(void)
{
                kal_uint32 ret_data;
                kal_uint32 i=0;
                printk("[CCK.1.3 Real-time & wakeup measurement ]\n");

                //mt6328_set_register_value(PMIC_RG_BUCK_OSC_SEL_SW,0); // 0: power on; 1: power down smps aud clock
                //mt6328_set_register_value(PMIC_RG_BUCK_OSC_SEL_MODE,1); // SW mode
                //mdelay(1);


                //setSrclkenMode(1);

                //set issue interrupt
                mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_WKUP_EN,1);
                mdelay(1);
                
                
                //define period of measurement
                //mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_PRD,10);

                //setting of enabling measurement 
                //mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_EN,0);


                //setSrclkenMode(0);

                //setSrclkenMode(1);

                mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_WKUP_START_SEL,1); // SW mode
                mdelay(10);
                mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_WKUP_START,0);
                mdelay(10);
               mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_WKUP_START,1);
                mdelay(10);
                mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_WKUP_START,0);
                mdelay(10);


                printk("[wake up]\n");
                while(i<20)
                {
                                printk("i:%d rdy:%d out:%d\n",i,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_MDRT)
                                                                                                                                ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_MDRT));
                                udelay(1000);
                                i++;

                }

}


void pmic6328_cck14(void)
{
                kal_uint32 ret_data;
                kal_uint32 i=0;
                printk("[CCK.1.4 Real-time & wakeup measurement ]\n");


                //set issue interrupt
                mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_WKUP_EN,1);

                mt6328_set_register_value(PMIC_RG_OSC_SEL_HW_SRC_SEL,3);

                mt6328_set_register_value(PMIC_RG_SRCLKEN_IN0_HW_MODE,0);
                mt6328_set_register_value(PMIC_RG_SRCLKEN_IN0_EN,1);
                mt6328_set_register_value(PMIC_RG_SRCLKEN_IN1_EN,1);        

                mt6328_set_register_value(PMIC_RG_SRCLKEN_IN0_HW_MODE,0);
                mt6328_set_register_value(PMIC_RG_SRCLKEN_IN0_EN,0);
                mt6328_set_register_value(PMIC_RG_SRCLKEN_IN1_EN,0);        
  
               mdelay(2);

                mt6328_set_register_value(PMIC_RG_SRCLKEN_IN0_HW_MODE,0);
                mt6328_set_register_value(PMIC_RG_SRCLKEN_IN0_EN,1);
                mt6328_set_register_value(PMIC_RG_SRCLKEN_IN1_EN,1);        


                printk("[wake up]\n");
                while(i<20)
                {
                                printk("i:%d rdy:%d out:%d\n",i,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_MDRT)
                                                                                                                                ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_MDRT));
                                udelay(1000);
                                i++;

                }

}


void pmic6328_cck15(void)
{
                kal_uint32 ret_data;
                kal_uint32 i=0;
                printk("[CCK.1.1 Real-time & wakeup measurement ]\n");
 
 
                //set issue interrupt
                mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_WKUP_EN,1);
 
                        // Allowing SPI R/W for Sleep mode (For DVT verification only)
                        mt6328_set_register_value(PMIC_RG_SLP_RW_EN,1);
                        mt6328_set_register_value(PMIC_RG_G_SMPS_AUD_CK_PDN_HWEN,0); 
                        mt6328_set_register_value(PMIC_RG_G_SMPS_AUD_CK_PDN,0);     
 
                               mt6328_set_register_value(PMIC_RG_OSC_SEL_HW_SRC_SEL,3);
                              mt6328_set_register_value(PMIC_AUXADC_SRCLKEN_SRC_SEL,1); // Make AUXADC source SRCLKEN_IN1
                              
                               mt6328_set_register_value(PMIC_RG_SRCLKEN_IN1_HW_MODE,0);
                               mt6328_set_register_value(PMIC_RG_SRCLKEN_IN1_EN,1);        
 
                 mt6328_set_register_value(PMIC_RG_SRCLKEN_IN1_HW_MODE,0);
                mt6328_set_register_value(PMIC_RG_SRCLKEN_IN1_EN,0);        
  
               mdelay(2);
 
                 mt6328_set_register_value(PMIC_RG_SRCLKEN_IN1_HW_MODE,0);
                mt6328_set_register_value(PMIC_RG_SRCLKEN_IN1_EN,1);        
 
 
                printk("[wake up]\n");
                while(i<20)
                {
                                printk("i:%d rdy:%d out:%d\n",i,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_MDRT)
                                                                                                                                ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_MDRT));
                                udelay(1000);
                                i++;
 
                }

}

void pmic6328_cck16(void)
{


                kal_uint32 ret_data;
                kal_uint32 i=0;
                printk("[CCK.1.6 Real-time & wakeup measurement ]\n");
 
                
               // set to avoid SRCLKEN toggling to reset auxadc in sleep mode
               mt6328_set_register_value(PMIC_STRUP_AUXADC_START_SEL,1);
               mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SW,1);
               mt6328_set_register_value(PMIC_STRUP_AUXADC_RSTB_SEL,1);
 
               //set issue interrupt
               mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_WKUP_EN,1);

               setSrclkenMode(1);

               setSrclkenMode(0);
 
               setSrclkenMode(1);
 
 
               printk("[wake up]\n");
               while(i<20)
               {
                                printk("i:%d rdy:%d out:%d\n",i,mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_MDRT)
                                                                                                                                ,mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_MDRT));
                                udelay(1000);
                                i++;
 
               }
}







void pmic6328_scan2(void)
{
	printk("[scan]0x%x \n",upmu_get_reg_value(0xf0a));
       printk("dbg[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_MDBG),
                    mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_MDBG));
	
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_00),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_00));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_01),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_01));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_02),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_02));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_03),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_03));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_04),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_04));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_05),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_05));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_06),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_06));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_07),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_07));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_08),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_08));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_09),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_09));

	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_10),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_10));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_11),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_11));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_12),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_12));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_13),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_13));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_14),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_14));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_15),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_15));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_16),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_16));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_17),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_17));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_18),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_18));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_19),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_19));

	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_20),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_20));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_21),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_21));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_22),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_22));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_23),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_23));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_24),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_24));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_25),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_25));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_26),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_26));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_27),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_27));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_28),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_28));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_29),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_29));


	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_30),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_30));
	printk("[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_31),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_31));

}


void pmic6328_scan(void)
{
	printk("[scan]0x%x \n",upmu_get_reg_value(0xf0a));
       printk("dbg[%d]::%d\n",mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_MDBG),
                    mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_MDBG));
	
	printk("[00]:%2d:%d  [01]:%2d:%d  [02]:%2d:%d  [03]:%2d:%d  [04]:%2d:%d  \n",
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_00),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_00),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_01),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_01),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_02),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_02),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_03),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_03),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_04),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_04));

	printk("[05]:%2d:%d  [06]:%2d:%d  [07]:%2d:%d  [08]:%2d:%d  [09]:%2d:%d  \n",
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_05),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_05),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_06),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_06),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_07),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_07),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_08),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_08),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_09),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_09));

	printk("[10]:%2d:%d  [11]:%2d:%d  [12]:%2d:%d  [13]:%2d:%d  [14]:%2d:%d  \n",
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_10),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_10),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_11),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_11),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_12),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_12),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_13),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_13),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_14),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_14));

	printk("[15]:%2d:%d  [16]:%2d:%d  [17]:%2d:%d  [18]:%2d:%d  [19]:%2d:%d  \n",
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_15),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_15),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_16),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_16),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_17),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_17),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_18),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_18),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_19),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_19));

	printk("[20]:%2d:%d  [21]:%2d:%d  [22]:%2d:%d  [23]:%2d:%d  [24]:%2d:%d  \n",
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_20),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_20),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_21),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_21),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_22),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_22),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_23),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_23),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_24),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_24));

	printk("[25]:%2d:%d  [26]:%2d:%d  [27]:%2d:%d  [28]:%2d:%d  [29]:%2d:%d  \n",
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_25),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_25),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_26),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_26),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_27),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_27),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_28),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_28),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_29),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_29));

	printk("[30]:%2d:%d  [31]:%2d:%d   \n",
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_30),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_30),
		mt6328_get_register_value(PMIC_AUXADC_BUF_RDY_31),
		mt6328_get_register_value(PMIC_AUXADC_BUF_OUT_31));



	


}



//CCK.1.2 Background measurement
void pmic6328_cck12(void)
{
	kal_uint32 ret_data;
	kal_uint32 i=0;
	printk("[CCK.1.2 Background measurement]\n");

	//set issue interrupt
	//mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_WKUP_EN,1);
	
	//define period of measurement
	//mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_PRD,0x1);
	mt6328_set_register_value(PMIC_AUXADC_MDBG_DET_PRD,0x20);  

	//setting of enabling measurement 
	//mt6328_set_register_value(PMIC_AUXADC_MDRT_DET_EN,1);
	mt6328_set_register_value(PMIC_AUXADC_MDBG_DET_EN,1);

/*
	while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_MDRT)!=1)
	{
	}
*/

	for(i=0;i<100;i++)
	{	
	      mdelay(16);
   	      pmic6328_scan();
		
	}

}



void pmic6328_efuse11(void)
{
	kal_uint32 i;
	printk("1.1 Compare EFUSE bit #0 ~ 511 which are HW auto-loaded \n");	

	printk("0x%x =0x%x\n",0x0c1e,upmu_get_reg_value(0x0c1e));	
	printk("0x%x =0x%x\n",0x0c20,upmu_get_reg_value(0x0c20));	
	printk("0x%x =0x%x\n",0x0c22,upmu_get_reg_value(0x0c22));	
	printk("0x%x =0x%x\n",0x0c24,upmu_get_reg_value(0x0c24));	
	printk("0x%x =0x%x\n",0x0c26,upmu_get_reg_value(0x0c26));	
	printk("0x%x =0x%x\n",0x0c28,upmu_get_reg_value(0x0c28));	
	printk("0x%x =0x%x\n",0x0c2a,upmu_get_reg_value(0x0c2a));	
	printk("0x%x =0x%x\n",0x0c2c,upmu_get_reg_value(0x0c2c));	
	printk("0x%x =0x%x\n",0x0c2e,upmu_get_reg_value(0x0c2e));	
	printk("0x%x =0x%x\n",0x0c30,upmu_get_reg_value(0x0c30));	
	printk("0x%x =0x%x\n",0x0c32,upmu_get_reg_value(0x0c32));	
	printk("0x%x =0x%x\n",0x0c34,upmu_get_reg_value(0x0c34));	
	printk("0x%x =0x%x\n",0x0c36,upmu_get_reg_value(0x0c36));	
	printk("0x%x =0x%x\n",0x0c38,upmu_get_reg_value(0x0c38));	
	printk("0x%x =0x%x\n",0x0c3a,upmu_get_reg_value(0x0c3a));	
	printk("0x%x =0x%x\n",0x0c3c,upmu_get_reg_value(0x0c3c));	
	printk("0x%x =0x%x\n",0x0c3e,upmu_get_reg_value(0x0c3e));	
	printk("0x%x =0x%x\n",0x0c5c,upmu_get_reg_value(0x0c5c));	


}


void pmic6328_efuse21(void)
{
	kal_uint32 i,j;
	kal_uint32 val;
	printk("2.1 Compare EFUSE bit #512 ~ 1023 which are SW triggered read  \n");	
	mt6328_set_register_value(PMIC_RG_EFUSE_CK_PDN_HWEN,0);
	mt6328_set_register_value(PMIC_RG_EFUSE_CK_PDN,0);
	mt6328_set_register_value(PMIC_RG_OTP_RD_SW,1);

	for (i=0;i<0x1f;i++)
	{
		mt6328_set_register_value(PMIC_RG_OTP_PA,i*2);

		if(mt6328_get_register_value(PMIC_RG_OTP_RD_TRIG)==0)
		{
			mt6328_set_register_value(PMIC_RG_OTP_RD_TRIG,1);
		}
		else
		{
			mt6328_set_register_value(PMIC_RG_OTP_RD_TRIG,0);
		}

		while(mt6328_get_register_value(PMIC_RG_OTP_RD_BUSY)==1)
		{
		}

		//printk("[%d -%d] RG_OTP_PA:0x%x =0x%x\n",512+16*i,527+16*i,i*2,mt6328_get_register_value(PMIC_RG_OTP_DOUT_SW));
		//printk("[%d] PMIC_RG_OTP_DOUT_SW=0x%x\n",i,mt6328_get_register_value(PMIC_RG_OTP_DOUT_SW));

		val=mt6328_get_register_value(PMIC_RG_OTP_DOUT_SW);

		for (j=0;j<16;j++)
		{
			kal_uint16 x;
			if(val & (1<<j))
			{
				x=1;
			}
			else
			{
				x=0;
			}

			printk("[%d] =%d\n",512+16*i+j,x);			
		}





		
		
	}

	mt6328_set_register_value(PMIC_RG_EFUSE_CK_PDN_HWEN,1);
	mt6328_set_register_value(PMIC_RG_EFUSE_CK_PDN,1);

}



void pmic6328_ldo_vio18(void)
{
	int x,y;
	upmu_set_reg_value(0x000e, 0x7ff3);
	upmu_set_reg_value(0x000a, 0x8000);
	upmu_set_reg_value(0x0a80, 0x0000);

	printk("VIO18 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VIO18_ON_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VIO18_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VIO18_EN);
	if(mt6328_get_register_value(PMIC_QI_VIO18_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VIO18_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VIO18_EN);
		if(mt6328_get_register_value(PMIC_QI_VIO18_EN)==1)
		{
			printk("VIO18 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);



	printk("VIO18 Ldo on/off control HW mode\n");
	mt6328_set_register_value(PMIC_RG_VIO18_ON_CTRL,1);
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VIO18_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VIO18_EN)==1)
		{
			printk("VIO18 pass\n");
		}
	}


}




void pmic6328_ldo1_sw_on_control(void)
{
	int x,y;
	printk("1 Ldo on/off control sw mode \n");

	printk("VAUX18 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VAUX18_ON_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VAUX18_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VAUX18_EN);
	if(mt6328_get_register_value(PMIC_QI_VAUX18_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VAUX18_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VAUX18_EN);
		if(mt6328_get_register_value(PMIC_QI_VAUX18_EN)==1)
		{
			printk("VAUX18 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);


	printk("VTCXO 0 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VTCXO_0_ON_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VTCXO_0_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VTCXO_0_EN);	
	if(mt6328_get_register_value(PMIC_QI_VTCXO_0_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VTCXO_0_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VTCXO_0_EN);
		if(mt6328_get_register_value(PMIC_QI_VTCXO_0_EN)==1)
		{
			printk("VTCXO 0 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);


	printk("VTCXO 1 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VTCXO_1_ON_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VTCXO_1_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VTCXO_1_EN);		
	if(mt6328_get_register_value(PMIC_QI_VTCXO_1_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VTCXO_1_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VTCXO_1_EN);	
		if(mt6328_get_register_value(PMIC_QI_VTCXO_1_EN)==1)
		{
			printk("VTCXO 1 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);


	printk("VAUD28 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VAUD28_ON_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VAUD28_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VAUD28_EN);
	if(mt6328_get_register_value(PMIC_QI_VAUD28_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VAUD28_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VAUD28_EN);
		if(mt6328_get_register_value(PMIC_QI_VAUD28_EN)==1)
		{
			printk("VAUD28 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);



	printk("VCN28 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCN28_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VCN28_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VCN28_EN);	
	if(mt6328_get_register_value(PMIC_QI_VCN28_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCN28_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VCN28_EN);
		if(mt6328_get_register_value(PMIC_QI_VCN28_EN)==1)
		{
			printk("VCN28 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);

	printk("VCAMA Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCAMA_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VCAMA_EN);		
	if(mt6328_get_register_value(PMIC_QI_VCAMA_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCAMA_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VCAMA_EN);	
		if(mt6328_get_register_value(PMIC_QI_VCAMA_EN)==1)
		{
			printk("VCAMA pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);


	printk("VCN33 BT Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCN33_ON_CTRL_BT,0);	
	mt6328_set_register_value(PMIC_RG_VCN33_EN_BT,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VCN33_EN);			
	if(mt6328_get_register_value(PMIC_QI_VCN33_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCN33_EN_BT,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VCN33_EN);				
		if(mt6328_get_register_value(PMIC_QI_VCN33_EN)==1)
		{
			printk("VCN33 BT pass\n");
		}
	}
	mt6328_set_register_value(PMIC_RG_VCN33_EN_BT,0);
	printk("disable:%d enable:%d\n",x,y);


	printk("VCN33 WIFI Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCN33_ON_CTRL_WIFI,0);	
	mt6328_set_register_value(PMIC_RG_VCN33_EN_WIFI,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VCN33_EN);		
	if(mt6328_get_register_value(PMIC_QI_VCN33_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCN33_EN_WIFI,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VCN33_EN);			
		if(mt6328_get_register_value(PMIC_QI_VCN33_EN)==1)
		{
			printk("VCN33 WIFI pass\n");
		}
	}
	mt6328_set_register_value(PMIC_RG_VCN33_EN_WIFI,0);
	printk("disable:%d enable:%d\n",x,y);


	printk("VUSB33 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VUSB33_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VUSB33_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VUSB33_EN);		
	if(mt6328_get_register_value(PMIC_QI_VUSB33_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VUSB33_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VUSB33_EN);			
		if(mt6328_get_register_value(PMIC_QI_VUSB33_EN)==1)
		{
			printk("VUSB33 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);
	

	printk("VEFUSE Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VEFUSE_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VEFUSE_EN);		
	if(mt6328_get_register_value(PMIC_QI_VEFUSE_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VEFUSE_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VEFUSE_EN);			
		if(mt6328_get_register_value(PMIC_QI_VEFUSE_EN)==1)
		{
			printk("VEFUSE pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);

	printk("VSIM1 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VSIM1_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VSIM1_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VSIM1_EN);	
	if(mt6328_get_register_value(PMIC_QI_VSIM1_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VSIM1_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VSIM1_EN);		
		if(mt6328_get_register_value(PMIC_QI_VSIM1_EN)==1)
		{
			printk("VSIM1 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);
	

	printk("VSIM2 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VSIM2_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VSIM2_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VSIM2_EN);		
	if(mt6328_get_register_value(PMIC_QI_VSIM2_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VSIM2_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VSIM2_EN);	
		if(mt6328_get_register_value(PMIC_QI_VSIM2_EN)==1)
		{
			printk("VSIM2 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);

	printk("VEMC_3V3 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VEMC_3V3_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VEMC_3V3_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VEMC_3V3_EN);		
	if(mt6328_get_register_value(PMIC_QI_VEMC_3V3_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VEMC_3V3_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VEMC_3V3_EN);			
		if(mt6328_get_register_value(PMIC_QI_VEMC_3V3_EN)==1)
		{
			printk("VEMC_3V3 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);
	
	printk("VMCH Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VMCH_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VMCH_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VMCH_EN);		
	if(mt6328_get_register_value(PMIC_QI_VMCH_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VMCH_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VMCH_EN);	
		if(mt6328_get_register_value(PMIC_QI_VMCH_EN)==1)
		{
			printk("VMCH pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);
	

	//VTREF
	printk("VTREF Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_TREF_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_TREF_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_TREF_EN);		
	if(mt6328_get_register_value(PMIC_QI_TREF_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_TREF_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_TREF_EN);			
		if(mt6328_get_register_value(PMIC_QI_TREF_EN)==1)
		{
			printk("TREF pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);


	printk("VMC Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VMC_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VMC_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VMC_EN);	
	if(mt6328_get_register_value(PMIC_QI_VMC_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VMC_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VMC_EN);		
		if(mt6328_get_register_value(PMIC_QI_VMC_EN)==1)
		{
			printk("VMC pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);

	printk("VCAMAF Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCAMAF_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VCAMAF_EN);	
	if(mt6328_get_register_value(PMIC_QI_VCAMAF_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCAMAF_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VCAMAF_EN);
		if(mt6328_get_register_value(PMIC_QI_VCAMAF_EN)==1)
		{
			printk("VCAMAF pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);


	printk("VIO28 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VIO28_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VIO28_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VIO28_EN);	
	if(mt6328_get_register_value(PMIC_QI_VIO28_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VIO28_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VIO28_EN);		
		if(mt6328_get_register_value(PMIC_QI_VIO28_EN)==1)
		{
			printk("VIO28 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);


	printk("VGP1 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VGP1_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VGP1_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VGP1_EN);		
	if(mt6328_get_register_value(PMIC_QI_VGP1_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VGP1_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VGP1_EN);			
		if(mt6328_get_register_value(PMIC_QI_VGP1_EN)==1)
		{
			printk("VGP1 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);

	printk("VIBR Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VIBR_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VIBR_EN);	
	if(mt6328_get_register_value(PMIC_QI_VIBR_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VIBR_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VIBR_EN);	
		if(mt6328_get_register_value(PMIC_QI_VIBR_EN)==1)
		{
			printk("VIBR pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);

	printk("VCAMD Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCAMD_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VCAMD_EN);		
	if(mt6328_get_register_value(PMIC_QI_VCAMD_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCAMD_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VCAMD_EN);			
		if(mt6328_get_register_value(PMIC_QI_VCAMD_EN)==1)
		{
			printk("VCAMD pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);


	printk("VRF18_0 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VRF18_0_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VRF18_0_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VRF18_0_EN);
	if(mt6328_get_register_value(PMIC_QI_VRF18_0_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VRF18_0_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VRF18_0_EN);
		if(mt6328_get_register_value(PMIC_QI_VRF18_0_EN)==1)
		{
			printk("VRF18_0 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);

	printk("VRF18_1 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VRF18_1_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VRF18_1_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VRF18_1_EN);	
	if(mt6328_get_register_value(PMIC_QI_VRF18_1_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VRF18_1_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VRF18_1_EN);		
		if(mt6328_get_register_value(PMIC_QI_VRF18_1_EN)==1)
		{
			printk("VRF18_1 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);


	printk("VCN18 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCN18_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VCN18_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VCN18_EN);
	if(mt6328_get_register_value(PMIC_QI_VCN18_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCN18_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VCN18_EN);
		if(mt6328_get_register_value(PMIC_QI_VCN18_EN)==1)
		{
			printk("VCN18 pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);

	printk("VCAMIO Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCAMIO_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VCAMIO_EN);
	if(mt6328_get_register_value(PMIC_QI_VCAMIO_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCAMIO_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VCAMIO_EN);
		if(mt6328_get_register_value(PMIC_QI_VCAMIO_EN)==1)
		{
			printk("VCAMIO pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);

	printk("VSRAM Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VSRAM_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VSRAM_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VSRAM_EN);
	if(mt6328_get_register_value(PMIC_QI_VSRAM_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VSRAM_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VSRAM_EN);
		if(mt6328_get_register_value(PMIC_QI_VSRAM_EN)==1)
		{
			printk("VSRAM pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);


	printk("VM Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VM_ON_CTRL,0);	
	mt6328_set_register_value(PMIC_RG_VM_EN,0);
	mdelay(50);
	x=mt6328_get_register_value(PMIC_QI_VM_EN);	
	if(mt6328_get_register_value(PMIC_QI_VM_EN)==0)
	{
		mt6328_set_register_value(PMIC_RG_VM_EN,1);
		mdelay(50);
		y=mt6328_get_register_value(PMIC_QI_VM_EN);
		if(mt6328_get_register_value(PMIC_QI_VM_EN)==1)
		{
			printk("VM pass\n");
		}
	}
	printk("disable:%d enable:%d\n",x,y);




}

void setSrclkenMode(kal_uint8 mode)
{
	//Switch to SW mode
	mt6328_set_register_value(PMIC_RG_SRCLKEN_IN0_HW_MODE,0);

	// Allowing SPI R/W for Sleep mode (For DVT verification only)
	mt6328_set_register_value(PMIC_RG_SLP_RW_EN,1);
	mt6328_set_register_value(PMIC_RG_G_SMPS_AUD_CK_PDN_HWEN,0);	
	mt6328_set_register_value(PMIC_RG_G_SMPS_AUD_CK_PDN,0);	

	
	mt6328_set_register_value(PMIC_RG_SRCLKEN_IN0_EN,mode);
	mt6328_set_register_value(PMIC_RG_SRCLKEN_IN1_EN,mode);	
	mdelay(50);
}

void pmic6328_vtcxo_1(void)
{
	int x,y;

	printk("VTCXO 1 Ldo on/off control sw mode\n");

	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));	

	
	mt6328_set_register_value(PMIC_RG_VTCXO_1_ON_CTRL,1);

	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));	

	mt6328_set_register_value(PMIC_RG_VTCXO_1_SRCLK_EN_SEL,0);

	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	
	setSrclkenMode(0);

	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));	
	
	x=mt6328_get_register_value(PMIC_QI_VTCXO_1_EN);

	printk("[0x%x]=0x%x\n",0x204,upmu_get_reg_value(0x204));
	printk("[0x%x]=0x%x\n",0x2d2,upmu_get_reg_value(0x2d2));	
	printk("[0x%x]=0x%x\n",0x278,upmu_get_reg_value(0x278));	
	printk("[0x%x]=0x%x\n",0x23c,upmu_get_reg_value(0x23c));		
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));

	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	
	if(mt6328_get_register_value(PMIC_QI_VTCXO_1_EN)==0)
	{
		printk("VTCXO 1: 0 pass\n");
	}
	else
	{
		printk("VTCXO 1: 0 fail\n");
	}

	setSrclkenMode(1);
	y=mt6328_get_register_value(PMIC_QI_VTCXO_1_EN);

	printk("[0x%x]=0x%x\n",0x204,upmu_get_reg_value(0x204));
	printk("[0x%x]=0x%x\n",0x2d2,upmu_get_reg_value(0x2d2));	
	printk("[0x%x]=0x%x\n",0x278,upmu_get_reg_value(0x278));	
	printk("[0x%x]=0x%x\n",0x23c,upmu_get_reg_value(0x23c));		
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	if(mt6328_get_register_value(PMIC_QI_VTCXO_1_EN)==1)
	{
		printk("VTCXO 1: 1 pass\n");
	}
	else
	{
		printk("VTCXO 1: 1 fail\n");
	}


	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));
	printk("[0x%x]=0x%x\n",0xa02,upmu_get_reg_value(0xa02));

	printk("x:%d y:%d\n",x,y);

}

void pmic6328_ldo1_hw_on_control(void)
{
	int x,y;
	printk("1 Ldo on/off control hw mode \n");

	printk("VAUX18 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VAUX18_ON_CTRL,1);
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VAUX18_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VAUX18_EN)==1)
		{
			printk("VAUX18 pass\n");
		}
	}


	printk("VTCXO 0 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VTCXO_0_ON_CTRL,1);
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VTCXO_0_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VTCXO_0_EN)==1)
		{
			printk("VTCXO 0 pass\n");
		}
	}


	printk("VTCXO 1 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VTCXO_1_ON_CTRL,1);
	setSrclkenMode(0);
	x=mt6328_get_register_value(PMIC_QI_VTCXO_1_EN);
	if(mt6328_get_register_value(PMIC_QI_VTCXO_1_EN)==0)
	{
		setSrclkenMode(1);
		y=mt6328_get_register_value(PMIC_QI_VTCXO_1_EN);
		if(mt6328_get_register_value(PMIC_QI_VTCXO_1_EN)==1)
		{
			printk("VTCXO 1 pass\n");
		}
		else
		{
			printk("VTCXO 1: 1 fail\n");
		}
	}
	else
	{
		printk("VTCXO 1: 0 fail\n");
	}
	printk("x:%d y:%d\n",x,y);


	printk("VAUD28 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VAUD28_ON_CTRL,1);
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VAUD28_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VAUD28_EN)==1)
		{
			printk("VAUD28 pass\n");
		}
	}


	printk("VCN28 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCN28_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VCN28_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VCN28_EN)==1)
		{
			printk("VCN28 pass\n");
		}
	}


	printk("VCN33 BT Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCN33_ON_CTRL_BT,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VCN33_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VCN33_EN)==1)
		{
			printk("VCN33 BT pass\n");
		}
	}
	


	printk("VCN33 WIFI Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCN33_ON_CTRL_WIFI,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VCN33_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VCN33_EN)==1)
		{
			printk("VCN33 WIFI pass\n");
		}
	}
	


	printk("VUSB33 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VUSB33_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VUSB33_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VUSB33_EN)==1)
		{
			printk("VUSB33 pass\n");
		}
	}


	printk("VSIM1 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VSIM1_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VSIM1_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VSIM1_EN)==1)
		{
			printk("VSIM1 pass\n");
		}
	}


	printk("VSIM2 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VSIM2_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VSIM2_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VSIM2_EN)==1)
		{
			printk("VSIM2 pass\n");
		}
	}


	printk("VEMC_3V3 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VEMC_3V3_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VEMC_3V3_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VEMC_3V3_EN)==1)
		{
			printk("VEMC_3V3 pass\n");
		}
	}

	
	printk("VMCH Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VMCH_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VMCH_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VMCH_EN)==1)
		{
			printk("VMCH pass\n");
		}
	}


	//VTREF
	printk("VTREF Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_TREF_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_TREF_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_TREF_EN)==1)
		{
			printk("TREF pass\n");
		}
	}


	printk("VMC Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VMC_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VMC_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VMC_EN)==1)
		{
			printk("VMC pass\n");
		}
	}


	printk("VIO28 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VIO28_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VIO28_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VIO28_EN)==1)
		{
			printk("VIO28 pass\n");
		}
	}


	printk("VGP1 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VGP1_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VGP1_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VGP1_EN)==1)
		{
			printk("VGP1 pass\n");
		}
	}


	printk("VRF18_0 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VRF18_0_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VRF18_0_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VRF18_0_EN)==1)
		{
			printk("VRF18_0 pass\n");
		}
	}


	printk("VRF18_1 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VRF18_1_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VRF18_1_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VRF18_1_EN)==1)
		{
			printk("VRF18_1 pass\n");
		}
	}


	printk("VCN18 Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCN18_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VCN18_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VCN18_EN)==1)
		{
			printk("VCN18 pass\n");
		}
	}

	printk("VSRAM Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VSRAM_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VSRAM_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VSRAM_EN)==1)
		{
			printk("VSRAM pass\n");
		}
	}


	printk("VM Ldo on/off control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VM_ON_CTRL,1);	
	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_QI_VM_EN)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VM_EN)==1)
		{
			printk("VM pass\n");
		}
	}

}































void pmic6328_ldo2_vosel_test(int idx)
{
	kal_uint8 i;
	printk("2 Ldo volsel test\n");


	if(idx==PMIC_6328_VCAMA)
	{
		printk("VCAMA volsel test\n");
		mt6328_set_register_value(PMIC_RG_VCAMA_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VCAMA_EN,1);

		for(i=0;i<4;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCAMA_VOSEL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VCN33)
	{
		printk("VCN33 volsel test\n");
		mt6328_set_register_value(PMIC_RG_VCN33_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VCN33_ON_CTRL_BT,0);
		mt6328_set_register_value(PMIC_RG_VCN33_EN_BT,1);

		for(i=0;i<7;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCN33_VOSEL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VEFUSE)
	{
		printk("VEFUSE volsel test\n");
		mt6328_set_register_value(PMIC_RG_VEFUSE_VOSEL,0);
		//mt6328_set_register_value(PMIC_RG_VEFUSE_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VEFUSE_EN,1);

		for(i=0;i<5;i++)
		{
			mt6328_set_register_value(PMIC_RG_VEFUSE_VOSEL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VSIM1)
	{
		printk("VSIM1 volsel test\n");
		mt6328_set_register_value(PMIC_RG_VSIM1_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VSIM1_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VSIM1_EN,1);

		mt6328_set_register_value(PMIC_RG_VSIM1_VOSEL,1);
		mdelay(300);
		mt6328_set_register_value(PMIC_RG_VSIM1_VOSEL,2);
		mdelay(300);
		mt6328_set_register_value(PMIC_RG_VSIM1_VOSEL,3);
		mdelay(300);
		mt6328_set_register_value(PMIC_RG_VSIM1_VOSEL,5);
		mdelay(300);
		mt6328_set_register_value(PMIC_RG_VSIM1_VOSEL,6);
		mdelay(300);
		mt6328_set_register_value(PMIC_RG_VSIM1_VOSEL,7);
		mdelay(300);		
		
	}
	else if(idx==PMIC_6328_VSIM2)
	{
		printk("VSIM2 volsel test\n");
		mt6328_set_register_value(PMIC_RG_VSIM2_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VSIM2_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VSIM2_EN,1);

		mt6328_set_register_value(PMIC_RG_VSIM2_VOSEL,1);
		mdelay(300);
		mt6328_set_register_value(PMIC_RG_VSIM2_VOSEL,2);
		mdelay(300);
		mt6328_set_register_value(PMIC_RG_VSIM2_VOSEL,3);
		mdelay(300);
		mt6328_set_register_value(PMIC_RG_VSIM2_VOSEL,5);
		mdelay(300);
		mt6328_set_register_value(PMIC_RG_VSIM2_VOSEL,6);
		mdelay(300);
		mt6328_set_register_value(PMIC_RG_VSIM2_VOSEL,7);
		mdelay(300);		
		
	}
	else if(idx==PMIC_6328_VEMC33)
	{
		printk("VEMC33 volsel test\n");
		mt6328_set_register_value(PMIC_RG_VEMC_3V3_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VEMC_3V3_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VEMC_3V3_EN,1);

		for(i=0;i<2;i++)
		{
			mt6328_set_register_value(PMIC_RG_VEMC_3V3_VOSEL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VMCH)
	{
		printk("VMCH volsel test\n");
		mt6328_set_register_value(PMIC_RG_VMCH_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VMCH_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VMCH_EN,1);

		for(i=0;i<3;i++)
		{
			mt6328_set_register_value(PMIC_RG_VMCH_VOSEL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VMC)
	{
		printk("VMC volsel test\n");
		mt6328_set_register_value(PMIC_RG_VMC_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VMC_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VMC_EN,1);

		for(i=0;i<4;i++)
		{
			mt6328_set_register_value(PMIC_RG_VMC_VOSEL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VCAM_AF)
	{
		printk("VCAMAF volsel test\n");
		mt6328_set_register_value(PMIC_RG_VCAMAF_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VCAMAF_EN,1);

		for(i=0;i<8;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCAMAF_VOSEL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VGP1)
	{
		printk("VGP1 volsel test\n");
		mt6328_set_register_value(PMIC_RG_VGP1_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VGP1_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VGP1_EN,1);

		for(i=0;i<8;i++)
		{
			mt6328_set_register_value(PMIC_RG_VGP1_VOSEL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VIBR)
	{
		printk("VIBR volsel test\n");
		mt6328_set_register_value(PMIC_RG_VIBR_EN,0);
		mt6328_set_register_value(PMIC_RG_VIBR_VOSEL,0);
		//mt6328_set_register_value(PMIC_RG_VIBR_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VIBR_EN,1);

		for(i=0;i<8;i++)
		{
			mt6328_set_register_value(PMIC_RG_VIBR_VOSEL,i);
			mdelay(300);
		}	
		
	}	
	else if(idx==PMIC_6328_VCAMD)
	{
		printk("VCAMD volsel test\n");
		mt6328_set_register_value(PMIC_RG_VCAMD_VOSEL,0);
		//mt6328_set_register_value(PMIC_RG_VCAMD_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VCAMD_EN,1);

		for(i=0;i<6;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCAMD_VOSEL,i);
			mdelay(300);
		}	
		
	}
	else if(idx==PMIC_6328_VRF18_1)
	{
		printk("VRF18_1 volsel test\n");
		mt6328_set_register_value(PMIC_RG_VRF18_1_EN,0);
		mt6328_set_register_value(PMIC_RG_VRF18_1_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VRF18_1_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VRF18_1_EN,1);

		for(i=0;i<4;i++)
		{
			mt6328_set_register_value(PMIC_RG_VRF18_1_VOSEL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VCAM_IO)
	{
		printk("VCAM_IO volsel test\n");
		mt6328_set_register_value(PMIC_RG_VCAMIO_VOSEL,0);
		//mt6328_set_register_value(PMIC_RG_VCAMIO_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VCAMIO_EN,1);

		for(i=0;i<4;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCAMIO_VOSEL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VSRAM)
	{
		printk("VSRAM volsel test\n");
		mt6328_set_register_value(PMIC_RG_VSRAM_EN,0);
		mt6328_set_register_value(PMIC_RG_VSRAM_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VSRAM_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VSRAM_EN,1);

		for(i=0;i<128;i++)
		{
			mt6328_set_register_value(PMIC_RG_VSRAM_VOSEL,i);
			mdelay(50);
		}	
		
	}
	else if(idx==PMIC_6328_VM)
	{
		printk("VM volsel test\n");
		mt6328_set_register_value(PMIC_RG_VM_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VM_ON_CTRL,0);
		mt6328_set_register_value(PMIC_RG_VM_EN,1);

		for(i=0;i<3;i++)
		{
			mt6328_set_register_value(PMIC_RG_VM_VOSEL,i);
			mdelay(300);
		}	
		
	}

}



void pmic6328_ldo3_cal_test(int idx)
{
	kal_uint8 i;
	printk("3 Ldo cal test\n");

	if(idx==PMIC_6328_VAUX18)
	{
		printk("VAUX18 cal test\n");
		//mt6328_set_register_value(PMIC_RG_VAUX18_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VAUX18_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VAUX18_CAL,i);
			mdelay(300);
		}
	}
	if(idx==PMIC_6328_VTCXO_0)
	{
		printk("VTCXO0 cal test\n");
		//mt6328_set_register_value(PMIC_RG_VTCXO_0_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VTCXO_0_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VTCXO_0_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VTCXO_1)
	{
		printk("VTCXO1 cal test\n");
		//mt6328_set_register_value(PMIC_RG_VTCXO_1_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VTCXO_1_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VTCXO_1_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VAUD28)
	{
		printk("VAUD28 cal test\n");
		//mt6328_set_register_value(PMIC_RG_VAUD28_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VAUD28_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VAUD28_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VCN28)
	{
		printk("VCN28 cal test\n");
		//mt6328_set_register_value(PMIC_RG_VAUD28_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VCN28_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCN28_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VCAMA)
	{
		printk("VCAMA cal test\n");
		mt6328_set_register_value(PMIC_RG_VCAMA_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VCAMA_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCAMA_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VCN33)
	{
		printk("VCN33 cal test\n");
		mt6328_set_register_value(PMIC_RG_VCN33_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VCN33_EN_BT,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCN33_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VUSB33)
	{
		printk("VUSB33 cal test\n");
		//mt6328_set_register_value(PMIC_RG_VUSB33_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VUSB33_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VUSB33_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VEFUSE)
	{
		printk("VEFUSE cal test\n");
		mt6328_set_register_value(PMIC_RG_VEFUSE_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VEFUSE_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VEFUSE_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VSIM1)
	{
		printk("VSIM1 cal test\n");
		mt6328_set_register_value(PMIC_RG_VSIM1_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VSIM1_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VSIM1_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VSIM2)
	{
		printk("VSIM2 cal test\n");
		mt6328_set_register_value(PMIC_RG_VSIM2_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VSIM2_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VSIM2_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VEMC33)
	{
		printk("VEMC33 cal test\n");
		mt6328_set_register_value(PMIC_RG_VEMC_3V3_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VEMC_3V3_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VEMC_3V3_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VMCH)
	{
		printk("VMCH cal test\n");
		mt6328_set_register_value(PMIC_RG_VMCH_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VMCH_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VMCH_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VMC)
	{
		printk("VMC cal test\n");
		mt6328_set_register_value(PMIC_RG_VMC_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VMC_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VMC_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VCAM_AF)
	{
		printk("VCAMAF cal test\n");
		mt6328_set_register_value(PMIC_RG_VCAMAF_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VCAMAF_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCAMAF_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VIO28)
	{
		printk("VIO28 cal test\n");
		//mt6328_set_register_value(PMIC_RG_VCAMA_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VIO28_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VIO28_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VGP1)
	{
		printk("VGP1 cal test\n");
		mt6328_set_register_value(PMIC_RG_VGP1_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VGP1_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VGP1_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VIBR)
	{
		printk("VIBR cal test\n");
		mt6328_set_register_value(PMIC_RG_VIBR_EN,0);
		mt6328_set_register_value(PMIC_RG_VIBR_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VIBR_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VIBR_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VCAMD)
	{
		printk("VCAMD cal test\n");
		mt6328_set_register_value(PMIC_RG_VCAMD_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VCAMD_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCAMD_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VRF18_0)
	{
		printk("VRF18_0 cal test\n");
		//mt6328_set_register_value(PMIC_RG_VRF18_0_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VRF18_0_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VRF18_0_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VRF18_1)
	{
		printk("VRF18_1 cal test\n");
		mt6328_set_register_value(PMIC_RG_VRF18_1_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VRF18_1_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VRF18_1_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VIO18)
	{
		printk("VIO18 cal test\n");
		//mt6328_set_register_value(PMIC_RG_VIO18_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VIO18_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VIO18_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VCN18)
	{
		printk("VCN18 cal test\n");
		//mt6328_set_register_value(PMIC_RG_VIO18_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VCN18_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCN18_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VCAM_IO)
	{
		printk("VCAMIO cal test\n");
		mt6328_set_register_value(PMIC_RG_VCAMIO_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VCAMIO_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VCAMIO_CAL,i);
			mdelay(300);
		}
	}
	else if(idx==PMIC_6328_VM)
	{
		printk("VCAMIO cal test\n");
		mt6328_set_register_value(PMIC_RG_VM_VOSEL,0);
		mt6328_set_register_value(PMIC_RG_VM_EN,1);

		for(i=0;i<16;i++)
		{
			mt6328_set_register_value(PMIC_RG_VM_CAL,i);
			mdelay(300);
		}
	}
	
}

void pmic6328_ldo4_lp_test(void)
{
	printk("1 Ldo LP control \n");

	printk("VAUX18 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VAUX18_AUXADC_PWDB_EN,1);
	mt6328_set_register_value(PMIC_RG_VAUX18_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VAUX18_MODE_SET,0);
	if(mt6328_get_register_value(PMIC_QI_VAUX18_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VAUX18_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VAUX18_MODE)==1)
		{
			printk("VAUX18 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VAUX18_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VAUX18_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VAUX18_MODE)==1)
		{
			printk("VAUX18 Ldo LP control hw mode pass\n");
		}
	}



	printk("VTCXO_0 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VTCXO_0_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VTCXO_0_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VTCXO_0_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VTCXO_0_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VTCXO_0_MODE)==1)
		{
			printk("VTCXO_0 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VTCXO_0_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VTCXO_0_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VTCXO_0_MODE)==1)
		{
			printk("VTCXO_0 Ldo LP control hw mode pass\n");
		}
	}



	printk("VTCXO_1 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VTCXO_1_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VTCXO_1_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VTCXO_1_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VTCXO_1_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VTCXO_1_MODE)==1)
		{
			printk("VTCXO_1 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VTCXO_1_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VTCXO_1_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VTCXO_1_MODE)==1)
		{
			printk("VTCXO_1 Ldo LP control hw mode pass\n");
		}
	}



	printk("VAUD28 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VAUD28_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VAUD28_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VAUD28_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VAUD28_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VAUD28_MODE)==1)
		{
			printk("VAUD28 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VAUD28_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VAUD28_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VAUD28_MODE)==1)
		{
			printk("VTCXO_1 Ldo LP control hw mode pass\n");
		}
	}



	printk("VCN28 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCN28_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VCN28_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VCN28_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCN28_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VCN28_MODE)==1)
		{
			printk("VCN28 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VCN28_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VCN28_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VCN28_MODE)==1)
		{
			printk("VCN28 Ldo LP control hw mode pass\n");
		}
	}




	printk("VCN33 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCN33_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VCN33_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VCN33_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCN33_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VCN33_MODE)==1)
		{
			printk("VCN33 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VCN33_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VCN33_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VCN33_MODE)==1)
		{
			printk("VCN33 Ldo LP control hw mode pass\n");
		}
	}



	printk("VUSB33 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VUSB33_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VUSB33_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VUSB33_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VUSB33_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VUSB33_MODE)==1)
		{
			printk("VUSB33 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VUSB33_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VUSB33_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VUSB33_MODE)==1)
		{
			printk("VUSB33 Ldo LP control hw mode pass\n");
		}
	}



	printk("VEFUSE Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VEFUSE_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VEFUSE_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VEFUSE_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VEFUSE_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VEFUSE_MODE)==1)
		{
			printk("VEFUSE Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VEFUSE_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VEFUSE_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VEFUSE_MODE)==1)
		{
			printk("VEFUSE Ldo LP control hw mode pass\n");
		}
	}



	printk("VSIM1 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VSIM1_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VSIM1_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VSIM1_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VSIM1_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VSIM1_MODE)==1)
		{
			printk("VSIM1 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VSIM1_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VSIM1_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VSIM1_MODE)==1)
		{
			printk("VSIM1 Ldo LP control hw mode pass\n");
		}
	}



	printk("VSIM2 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VSIM2_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VSIM2_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VSIM2_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VSIM2_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VSIM2_MODE)==1)
		{
			printk("VSIM2 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VSIM1_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VSIM1_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VSIM1_MODE)==1)
		{
			printk("VSIM2 Ldo LP control hw mode pass\n");
		}
	}



	printk("VEMC_3V3 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VEMC_3V3_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VEMC_3V3_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VEMC_3V3_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VEMC_3V3_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VEMC_3V3_MODE)==1)
		{
			printk("VEMC_3V3 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VEMC_3V3_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VEMC_3V3_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VEMC_3V3_MODE)==1)
		{
			printk("VEMC_3V3 Ldo LP control hw mode pass\n");
		}
	}



	printk("VMCH Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VMCH_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VMCH_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VMCH_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VMCH_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VMCH_MODE)==1)
		{
			printk("VMCH Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VMCH_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VMCH_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VMCH_MODE)==1)
		{
			printk("VMCH Ldo LP control hw mode pass\n");
		}
	}



	printk("VMC Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VMC_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VMC_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VMC_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VMC_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VMC_MODE)==1)
		{
			printk("VMC Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VMC_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VMC_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VMC_MODE)==1)
		{
			printk("VMC Ldo LP control hw mode pass\n");
		}
	}



	printk("VCAMAF Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCAMAF_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VCAMAF_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VCAMAF_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCAMAF_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VCAMAF_MODE)==1)
		{
			printk("VCAMAF Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VCAMAF_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VCAMAF_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VCAMAF_MODE)==1)
		{
			printk("VCAMAF Ldo LP control hw mode pass\n");
		}
	}


 
	printk("VIO28 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VIO28_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VIO28_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VIO28_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VIO28_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VIO28_MODE)==1)
		{
			printk("VIO28 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VIO28_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VIO28_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VIO28_MODE)==1)
		{
			printk("VIO28 Ldo LP control hw mode pass\n");
		}
	}


 
	printk("VGP1 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VGP1_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VGP1_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VGP1_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VGP1_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VGP1_MODE)==1)
		{
			printk("VGP1 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VGP1_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VGP1_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VGP1_MODE)==1)
		{
			printk("VGP1 Ldo LP control hw mode pass\n");
		}
	}



	printk("VIBR Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VIBR_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VIBR_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VIBR_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VIBR_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VIBR_MODE)==1)
		{
			printk("VIBR Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VIBR_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VIBR_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VIBR_MODE)==1)
		{
			printk("VIBR Ldo LP control hw mode pass\n");
		}
	}



	printk("VCAMD Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCAMD_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VCAMD_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VCAMD_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCAMD_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VCAMD_MODE)==1)
		{
			printk("VCAMD Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VCAMD_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VCAMD_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VCAMD_MODE)==1)
		{
			printk("VCAMD Ldo LP control hw mode pass\n");
		}
	}



	printk("VRF18_0 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VRF18_0_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VRF18_0_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VRF18_0_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VRF18_0_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VRF18_0_MODE)==1)
		{
			printk("VRF18_0 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VRF18_0_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VRF18_0_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VRF18_0_MODE)==1)
		{
			printk("VRF18_0 Ldo LP control hw mode pass\n");
		}
	}



	printk("VRF18_1 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VRF18_1_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VRF18_1_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VRF18_1_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VRF18_1_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VRF18_1_MODE)==1)
		{
			printk("VRF18_1 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VRF18_1_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VRF18_1_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VRF18_1_MODE)==1)
		{
			printk("VRF18_1 Ldo LP control hw mode pass\n");
		}
	}


/*
	printk("VIO18 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VIO18_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VIO18_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VIO18_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VIO18_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VIO18_MODE)==1)
		{
			printk("VIO18 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VIO18_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VIO18_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VIO18_MODE)==1)
		{
			printk("VIO18 Ldo LP control hw mode pass\n");
		}
	}

*/

	printk("VCN18 Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCN18_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VCN18_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VCN18_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCN18_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VCN18_MODE)==1)
		{
			printk("VCN18 Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VCN18_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VCN18_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VCN18_MODE)==1)
		{
			printk("VCN18 Ldo LP control hw mode pass\n");
		}
	}



	printk("VCAMIO Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VCAMIO_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VCAMIO_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VCAMIO_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VCAMIO_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VCAMIO_MODE)==1)
		{
			printk("VCAMIO Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VCAMIO_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VCAMIO_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VCAMIO_MODE)==1)
		{
			printk("VCAMIO Ldo LP control hw mode pass\n");
		}
	}



	printk("VSRAM Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VSRAM_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VSRAM_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VSRAM_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VSRAM_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VSRAM_MODE)==1)
		{
			printk("VSRAM Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VSRAM_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VSRAM_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VSRAM_MODE)==1)
		{
			printk("VSRAM Ldo LP control hw mode pass\n");
		}
	}



	printk("VM Ldo LP control sw mode\n");
	mt6328_set_register_value(PMIC_RG_VM_MODE_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VM_MODE_SET,0);

	if(mt6328_get_register_value(PMIC_QI_VM_MODE)==0)
	{
		mt6328_set_register_value(PMIC_RG_VM_MODE_SET,1);
		if(mt6328_get_register_value(PMIC_QI_VM_MODE)==1)
		{
			printk("VM Ldo LP control sw mode pass\n");
		}
	}
	setSrclkenMode(0);
	mt6328_set_register_value(PMIC_RG_VM_MODE_SET,1);
	if(mt6328_get_register_value(PMIC_QI_VM_MODE)==0)
	{
		setSrclkenMode(1);
		if(mt6328_get_register_value(PMIC_QI_VM_MODE)==1)
		{
			printk("VM Ldo LP control hw mode pass\n");
		}
	}

}

void pmic6328_ldo5_fast_tran(void)
{
	printk("5.FAST TRAN Test\n");
	//if(idx==PMIC_6328_VEMC33)
	{
		printk("VEMC33 FAST TRAN Test\n");
		
		mt6328_set_register_value(PMIC_RG_VEMC_3V3_FAST_TRAN_EN,1);
		mt6328_set_register_value(PMIC_RG_VEMC_3V3_SRCLK_FAST_TRAN_SEL,0);

		setSrclkenMode(0);
		if(mt6328_get_register_value(PMIC_QI_VEMC_3V3_FAST_TRAN_EN)==0)
		{
			setSrclkenMode(1);
			if(mt6328_get_register_value(PMIC_QI_VEMC_3V3_FAST_TRAN_EN)==1)
			{
				printk("VEMC33 FAST TRAN Test pass\n");
			}
		}
	}
	//else if(idx==PMIC_6328_VMCH)
	{
		printk("VMCH FAST TRAN Test\n");
		
		mt6328_set_register_value(PMIC_RG_VMCH_FAST_TRAN_EN,1);
		mt6328_set_register_value(PMIC_RG_VMCH_SRCLK_FAST_TRAN_SEL,0);

		setSrclkenMode(0);
		if(mt6328_get_register_value(PMIC_QI_VMCH_FAST_TRAN_EN)==0)
		{
			setSrclkenMode(1);
			if(mt6328_get_register_value(PMIC_QI_VMCH_FAST_TRAN_EN)==1)
			{
				printk("VMCH FAST TRAN Test pass\n");
			}
		}
	}
	//else if(idx==PMIC_6328_VGP1)
	{
		printk("VGP1 FAST TRAN Test\n");
		
		mt6328_set_register_value(PMIC_RG_VGP1_FAST_TRAN_EN,1);
		mt6328_set_register_value(PMIC_RG_VGP1_SRCLK_FAST_TRAN_SEL,0);

		setSrclkenMode(0);
		if(mt6328_get_register_value(PMIC_QI_VGP1_FAST_TRAN_EN)==0)
		{
			setSrclkenMode(1);
			if(mt6328_get_register_value(PMIC_QI_VGP1_FAST_TRAN_EN)==1)
			{
				printk("VGP1 FAST TRAN Test pass\n");
			}
		}
	}
	//else if(idx==PMIC_6328_VRF18_0)
	{
		printk("VRF18_0 FAST TRAN Test\n");
		
		mt6328_set_register_value(PMIC_RG_VRF18_0_FAST_TRAN_EN,1);
		mt6328_set_register_value(PMIC_RG_VRF18_0_SRCLK_FAST_TRAN_SEL,0);

		setSrclkenMode(0);
		if(mt6328_get_register_value(PMIC_QI_VRF18_0_FAST_TRAN_EN)==0)
		{
			setSrclkenMode(1);
			if(mt6328_get_register_value(PMIC_QI_VRF18_0_FAST_TRAN_EN)==1)
			{
				printk("VRF18_0 FAST TRAN Test pass\n");
			}
		}
	}
	//else if(idx==PMIC_6328_VRF18_1)
	{
		printk("VRF18_1 FAST TRAN Test\n");
		
		mt6328_set_register_value(PMIC_RG_VRF18_1_FAST_TRAN_EN,1);
		mt6328_set_register_value(PMIC_RG_VRF18_1_SRCLK_FAST_TRAN_SEL,0);

		setSrclkenMode(0);
		if(mt6328_get_register_value(PMIC_QI_VRF18_1_FAST_TRAN_EN)==0)
		{
			setSrclkenMode(1);
			if(mt6328_get_register_value(PMIC_QI_VRF18_1_FAST_TRAN_EN)==1)
			{
				printk("VRF18_1 FAST TRAN Test pass\n");
			}
		}
	}
	//else if(idx==PMIC_6328_VSRAM)
	{
		printk("VSRAM FAST TRAN Test\n");
		
		mt6328_set_register_value(PMIC_RG_VSRAM_FAST_TRAN_EN,1);
		mt6328_set_register_value(PMIC_RG_VSRAM_SRCLK_FAST_TRAN_SEL,0);

		setSrclkenMode(0);
		if(mt6328_get_register_value(PMIC_QI_VSRAM_FAST_TRAN_EN)==0)
		{
			setSrclkenMode(1);
			if(mt6328_get_register_value(PMIC_QI_VSRAM_FAST_TRAN_EN)==1)
			{
				printk("VSRAM FAST TRAN Test pass\n");
			}
		}
	}
	//else if(idx==PMIC_6328_VM)
	{
		printk("VM FAST TRAN Test\n");
		
		mt6328_set_register_value(PMIC_RG_VM_FAST_TRAN_EN,1);
		mt6328_set_register_value(PMIC_RG_VM_SRCLK_FAST_TRAN_SEL,0);

		setSrclkenMode(0);
		if(mt6328_get_register_value(PMIC_QI_VM_FAST_TRAN_EN)==0)
		{
			setSrclkenMode(1);
			if(mt6328_get_register_value(PMIC_QI_VM_FAST_TRAN_EN)==1)
			{
				printk("VM FAST TRAN Test pass\n");
			}
		}
	}


}


void pmic6328_vpa11(void)
{
	kal_uint8 i;

	printk("\npmic6328_vpa11\n");
	mt6328_set_register_value(PMIC_VPA_VOSEL_CTRL,0);
	mt6328_set_register_value(PMIC_VPA_EN_CTRL,0);
	mt6328_set_register_value(PMIC_RG_VPA_EN,1);

	for(i=0;i<64;i++)
	{
		mt6328_set_register_value(PMIC_VPA_VOSEL,i);

		printk("VPA_VOSEL:%d QI_VPA_VOSEL:%d QI_VPA_DLC:%d\n",i,
			mt6328_get_register_value(PMIC_NI_VPA_VOSEL),
			mt6328_get_register_value(PMIC_QI_VPA_DLC));
	}
	
}

void pmic6328_vpa12(void)
{
	kal_uint8 i;
	kal_uint8 vpass=0;
	printk("\npmic6328_vpa12\n");

	for(i=0;i<4;i++)
	{
		mt6328_set_register_value(PMIC_VPA_BURSTH,i);

		if(mt6328_get_register_value(PMIC_QI_VPA_BURSTH)!=i)
		{
			printk("VPA BURSTH test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VPA_BURSTH));
			vpass=1;
		}

		printk("VPA BURSTH :VPA_BURSTH=%d    QI_VPA_BURSTH=%d \n",
			mt6328_get_register_value(PMIC_VPA_BURSTH),
			mt6328_get_register_value(PMIC_QI_VPA_BURSTH));
	}

	for(i=0;i<4;i++)
	{
		mt6328_set_register_value(PMIC_VPA_BURSTL,i);

		if(mt6328_get_register_value(PMIC_QI_VPA_BURSTL)!=i)
		{
			printk("VPA BURSTL test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VPA_BURSTL));
			vpass=1;
		}
	}

	if(vpass==0)
	{
		printk("VPA BURSTH test pass \n");
	}

	
}

void pmic_6328_reg(void)
{

	upmu_set_reg_value(0x000C, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x000C,upmu_get_reg_value(0x000C));
	upmu_set_reg_value(0x000C, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x000C,upmu_get_reg_value(0x000C));

	upmu_set_reg_value(0x0288, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0288,upmu_get_reg_value(0x0288));
	upmu_set_reg_value(0x0288, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0288,upmu_get_reg_value(0x0288));

	upmu_set_reg_value(0x02B8, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x02B8,upmu_get_reg_value(0x02B8));
	upmu_set_reg_value(0x02B8, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x02B8,upmu_get_reg_value(0x02B8));

	upmu_set_reg_value(0x041A, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x041A,upmu_get_reg_value(0x041A));
	upmu_set_reg_value(0x041A, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x041A,upmu_get_reg_value(0x041A));

	upmu_set_reg_value(0x046E, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x046E,upmu_get_reg_value(0x046E));
	upmu_set_reg_value(0x046E, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x046E,upmu_get_reg_value(0x046E));

	upmu_set_reg_value(0x0488, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0488,upmu_get_reg_value(0x0488));
	upmu_set_reg_value(0x0488, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0488,upmu_get_reg_value(0x0488));

	upmu_set_reg_value(0x04B0, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x04B0,upmu_get_reg_value(0x04B0));
	upmu_set_reg_value(0x04B0, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x04B0,upmu_get_reg_value(0x04B0));

	upmu_set_reg_value(0x04D4, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x04D4,upmu_get_reg_value(0x04D4));
	upmu_set_reg_value(0x04D4, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x04D4,upmu_get_reg_value(0x04D4));

	upmu_set_reg_value(0x0614, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0614,upmu_get_reg_value(0x0614));
	upmu_set_reg_value(0x0614, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0614,upmu_get_reg_value(0x0614));

	upmu_set_reg_value(0x063C, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x063C,upmu_get_reg_value(0x063C));
	upmu_set_reg_value(0x063C, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x063C,upmu_get_reg_value(0x063C));

	upmu_set_reg_value(0x0818, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0818,upmu_get_reg_value(0x0818));
	upmu_set_reg_value(0x0818, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0818,upmu_get_reg_value(0x0818));

	upmu_set_reg_value(0x0A5E, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0A5E,upmu_get_reg_value(0x0A5E));
	upmu_set_reg_value(0x0A5E, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0A5E,upmu_get_reg_value(0x0A5E));

	upmu_set_reg_value(0x0A9A, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0A9A,upmu_get_reg_value(0x0A9A));
	upmu_set_reg_value(0x0A9A, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0A9A,upmu_get_reg_value(0x0A9A));


	upmu_set_reg_value(0x0CA0, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0CA0,upmu_get_reg_value(0x0CA0));
	upmu_set_reg_value(0x0CA0, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0CA0,upmu_get_reg_value(0x0CA0));


	upmu_set_reg_value(0x0CBC, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0CBC,upmu_get_reg_value(0x0CBC));
	upmu_set_reg_value(0x0CBC, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0CBC,upmu_get_reg_value(0x0CBC));


	upmu_set_reg_value(0x0CE8, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0CE8,upmu_get_reg_value(0x0CE8));
	upmu_set_reg_value(0x0CE8, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0CE8,upmu_get_reg_value(0x0CE8));


	upmu_set_reg_value(0x0E9C, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0E9C,upmu_get_reg_value(0x0E9C));
	upmu_set_reg_value(0x0E9C, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0E9C,upmu_get_reg_value(0x0E9C));

	upmu_set_reg_value(0x0F2A, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0F2A,upmu_get_reg_value(0x0F2A));
	upmu_set_reg_value(0x0F2A, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0F2A,upmu_get_reg_value(0x0F2A));

	upmu_set_reg_value(0x0F7A, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x0F7A,upmu_get_reg_value(0x0F7A));
	upmu_set_reg_value(0x0F7A, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x0F7A,upmu_get_reg_value(0x0F7A));

	upmu_set_reg_value(0x6080, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x6080,upmu_get_reg_value(0x6080));
	upmu_set_reg_value(0x6080, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x6080,upmu_get_reg_value(0x6080));

	upmu_set_reg_value(0x4024, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x4024,upmu_get_reg_value(0x4024));
	upmu_set_reg_value(0x4024, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x4024,upmu_get_reg_value(0x4024));

	upmu_set_reg_value(0x2040, 0);
	printk("write 0 [0x%x]= 0x%x \n",0x2040,upmu_get_reg_value(0x2040));
	upmu_set_reg_value(0x2040, 0xffff);
	printk("write 0 [0x%x]= 0x%x \n",0x2040,upmu_get_reg_value(0x2040));


}


void pmic6328_buck211(void)
{
	kal_uint8 i;
	int x,y;

	upmu_set_reg_value(0x000e, 0x7ff3);
	upmu_set_reg_value(0x000a, 0x8000);
	upmu_set_reg_value(0x0a80, 0x0000);

	printk("VSYS22 sw en test :\n");
	mt6328_set_register_value(PMIC_VSYS22_EN_CTRL,0);
	mt6328_set_register_value(PMIC_VSYS22_EN,1);
	if(mt6328_get_register_value(PMIC_QI_VSYS22_EN)==1)
	{
		mt6328_set_register_value(PMIC_VSYS22_EN,0);
		if(mt6328_get_register_value(PMIC_QI_VSYS22_EN)==0)
		{
			printk("VSYS22 sw en test pass\n");
		}
	}	


	mt6328_set_register_value(PMIC_VSYS22_EN_CTRL,1);
	setSrclkenMode(1);
	x=mt6328_get_register_value(PMIC_QI_VSYS22_EN);

		printk("[0x%x]=0x%x\n",0x636,upmu_get_reg_value(0x636));
		printk("[0x%x]=0x%x\n",0x638,upmu_get_reg_value(0x638));
		printk("[0x%x]=0x%x\n",0x63a,upmu_get_reg_value(0x63a));
	
	if(mt6328_get_register_value(PMIC_QI_VSYS22_EN)==1)
	{
		setSrclkenMode(0);
		y=mt6328_get_register_value(PMIC_QI_VSYS22_EN);

		printk("[0x%x]=0x%x\n",0x636,upmu_get_reg_value(0x636));
		printk("[0x%x]=0x%x\n",0x638,upmu_get_reg_value(0x638));
		printk("[0x%x]=0x%x\n",0x63a,upmu_get_reg_value(0x63a));
		if(mt6328_get_register_value(PMIC_QI_VSYS22_EN)==0)
		{
			printk("VSYS22 hw en test pass\n");
		}
		else
		{
			printk("VSYS22 hw en test 0 fail\n");
		}
	}
	else
	{
		printk("VSYS22 hw en test 1 fail\n");
	}


}

void pmic6328_buck21(void)
{


	printk("VLTE sw en test :\n");
	mt6328_set_register_value(PMIC_VLTE_EN_CTRL,0);
	mt6328_set_register_value(PMIC_VLTE_EN,1);

	printk("VLTE sw enable:en:%d \n",mt6328_get_register_value(PMIC_QI_VLTE_EN));
	if(mt6328_get_register_value(PMIC_QI_VLTE_EN)==1)
	{
		mt6328_set_register_value(PMIC_VLTE_EN,0);
		printk("VLTE sw disable:en:%d \n",mt6328_get_register_value(PMIC_QI_VLTE_EN));
		if(mt6328_get_register_value(PMIC_QI_VLTE_EN)==0)
		{
			printk("VLTE sw en test pass\n");
		}
	}	

	mt6328_set_register_value(PMIC_VLTE_EN_CTRL,1);
	setSrclkenMode(1);
	printk("VLTE hw enable:en:%d \n",mt6328_get_register_value(PMIC_QI_VLTE_EN));
	if(mt6328_get_register_value(PMIC_QI_VLTE_EN)==1)
	{
		setSrclkenMode(0);
		printk("VLTE hw disable:en:%d \n",mt6328_get_register_value(PMIC_QI_VLTE_EN));
		if(mt6328_get_register_value(PMIC_QI_VLTE_EN)==0)
		{
			printk("VLTE hw en test pass\n");
		}
	}

	//vproc/vcore1/vsys22/vlte
	printk("VPROC sw en test :\n");
	mt6328_set_register_value(PMIC_VPROC_EN_CTRL,0);
	mt6328_set_register_value(PMIC_VPROC_EN,1);
	if(mt6328_get_register_value(PMIC_QI_VPROC_EN)==1)
	{
		mt6328_set_register_value(PMIC_VPROC_EN,0);
		if(mt6328_get_register_value(PMIC_QI_VPROC_EN)==0)
		{
			printk("VPROC sw en test pass\n");
		}
	}	


	mt6328_set_register_value(PMIC_VPROC_EN_CTRL,1);
	setSrclkenMode(1);
	if(mt6328_get_register_value(PMIC_QI_VPROC_EN)==1)
	{
		setSrclkenMode(0);
		if(mt6328_get_register_value(PMIC_QI_VPROC_EN)==0)
		{
			printk("VPROC hw en test pass\n");
		}
	}	


	printk("VCORE1 sw en test :\n");
	mt6328_set_register_value(PMIC_VCORE1_EN_CTRL,0);
	mt6328_set_register_value(PMIC_VCORE1_EN,1);
	if(mt6328_get_register_value(PMIC_QI_VCORE1_EN)==1)
	{
		mt6328_set_register_value(PMIC_VCORE1_EN,0);
		if(mt6328_get_register_value(PMIC_QI_VCORE1_EN)==0)
		{
			printk("VCORE1 sw en test pass\n");
		}
	}	


	mt6328_set_register_value(PMIC_VCORE1_EN_CTRL,1);
	setSrclkenMode(1);
	if(mt6328_get_register_value(PMIC_QI_VCORE1_EN)==1)
	{
		setSrclkenMode(0);
		if(mt6328_get_register_value(PMIC_QI_VCORE1_EN)==0)
		{
			printk("VCORE1 hw en test pass\n");
		}
	}


}


void pmic6328_buck25(void)
{
	kal_uint8 i;
	kal_uint8 value;
	//vproc/vcore1/vsys22/vltetest


	printk("VSYS22 VOSEL test pass\n");
	mt6328_set_register_value(PMIC_VSYS22_VOSEL,0);
	mt6328_set_register_value(PMIC_VSYS22_VOSEL_CTRL,0);

	for(i=0;i<128;i++)
	{
		mt6328_set_register_value(PMIC_VSYS22_VOSEL,i);

		value=mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL);
		if(i==value)
		{
			printk("VSYS22 %d %d\n",i,mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL));
		}
		else 
		{
			printk("VSYS22 %d %d %d %d %d??\n",i,value,
				mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL),
				mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL),
				mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL));
		}

		
	}

	mt6328_set_register_value(PMIC_VSYS22_VOSEL_CTRL,1);
	mt6328_set_register_value(PMIC_VSYS22_VOSEL_SLEEP,10);
	mt6328_set_register_value(PMIC_VSYS22_VOSEL_ON,50);

	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL)!=10)
	{
		printk("VOSEL hw mode fail\n");
	}

	setSrclkenMode(1);
	if(mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL)!=50)
	{
		printk("VOSEL hw mode fail\n");
	}


	printk("VPROC VOSEL test pass\n");
	mt6328_set_register_value(PMIC_VPROC_VOSEL,0);
	mt6328_set_register_value(PMIC_VPROC_VOSEL_CTRL,0);

	for(i=0;i<128;i++)
	{
		mt6328_set_register_value(PMIC_VPROC_VOSEL,i);

		if(i==mt6328_get_register_value(PMIC_NI_VPROC_VOSEL))
		{
			printk("VPROC %d %d\n",i,mt6328_get_register_value(PMIC_NI_VPROC_VOSEL));
		}
		else 
		{
			printk("VPROC %d %d ??\n",i,mt6328_get_register_value(PMIC_NI_VPROC_VOSEL));
		}
	}

	mt6328_set_register_value(PMIC_VPROC_VOSEL_CTRL,1);
	mt6328_set_register_value(PMIC_VPROC_VOSEL_SLEEP,10);
	mt6328_set_register_value(PMIC_VPROC_VOSEL_ON,50);

	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_NI_VPROC_VOSEL)!=10)
	{
		printk("VOSEL hw mode fail\n");
	}

	setSrclkenMode(1);
	if(mt6328_get_register_value(PMIC_NI_VPROC_VOSEL)!=50)
	{
		printk("VOSEL hw mode fail\n");
	}



	printk("VCORE1 VOSEL test pass\n");
	mt6328_set_register_value(PMIC_VCORE1_VOSEL,0);
	mt6328_set_register_value(PMIC_VCORE1_VOSEL_CTRL,0);

	for(i=0;i<128;i++)
	{
		mt6328_set_register_value(PMIC_VCORE1_VOSEL,i);

		if(i==mt6328_get_register_value(PMIC_NI_VCORE1_VOSEL))
		{
			printk("VCORE1 %d %d\n",i,mt6328_get_register_value(PMIC_NI_VCORE1_VOSEL));
		}
		else 
		{
			printk("VCORE1 %d %d ??\n",i,mt6328_get_register_value(PMIC_NI_VCORE1_VOSEL));
		}

		
	}

	mt6328_set_register_value(PMIC_VCORE1_VOSEL_CTRL,1);
	mt6328_set_register_value(PMIC_VCORE1_VOSEL_SLEEP,10);
	mt6328_set_register_value(PMIC_VCORE1_VOSEL_ON,50);

	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_NI_VCORE1_VOSEL)!=10)
	{
		printk("VOSEL hw mode fail\n");
	}

	setSrclkenMode(1);
	if(mt6328_get_register_value(PMIC_NI_VCORE1_VOSEL)!=50)
	{
		printk("VOSEL hw mode fail\n");
	}



	printk("VSYS22 VOSEL test pass\n");
	mt6328_set_register_value(PMIC_VSYS22_VOSEL,0);
	mt6328_set_register_value(PMIC_VSYS22_VOSEL_CTRL,0);

	for(i=0;i<128;i++)
	{
		mt6328_set_register_value(PMIC_VSYS22_VOSEL,i);

		if(i==mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL))
		{
			printk("VSYS22 %d %d\n",i,mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL));
		}
		else 
		{
			printk("VSYS22 %d %d ??\n",i,mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL));
		}

		
	}

	mt6328_set_register_value(PMIC_VSYS22_VOSEL_CTRL,1);
	mt6328_set_register_value(PMIC_VSYS22_VOSEL_SLEEP,10);
	mt6328_set_register_value(PMIC_VSYS22_VOSEL_ON,50);

	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL)!=10)
	{
		printk("VOSEL hw mode fail\n");
	}

	setSrclkenMode(1);
	if(mt6328_get_register_value(PMIC_NI_VSYS22_VOSEL)!=50)
	{
		printk("VOSEL hw mode fail\n");
	}



	printk("VLTE VOSEL test pass\n");
	mt6328_set_register_value(PMIC_VLTE_VOSEL,0);
	mt6328_set_register_value(PMIC_VLTE_VOSEL_CTRL,0);

	for(i=0;i<128;i++)
	{
		mt6328_set_register_value(PMIC_VLTE_VOSEL,i);

		if(i==mt6328_get_register_value(PMIC_NI_VLTE_VOSEL))
		{
			printk("VLTE %d %d\n",i,mt6328_get_register_value(PMIC_NI_VLTE_VOSEL));
		}
		else 
		{
			printk("VLTE %d %d ??\n",i,mt6328_get_register_value(PMIC_NI_VLTE_VOSEL));
		}
		
	}

	mt6328_set_register_value(PMIC_VLTE_VOSEL_CTRL,1);
	mt6328_set_register_value(PMIC_VLTE_VOSEL_SLEEP,10);
	mt6328_set_register_value(PMIC_VLTE_VOSEL_ON,50);

	setSrclkenMode(0);
	if(mt6328_get_register_value(PMIC_NI_VLTE_VOSEL)!=10)
	{
		printk("VOSEL hw mode fail\n");
	}

	setSrclkenMode(1);
	if(mt6328_get_register_value(PMIC_NI_VLTE_VOSEL)!=50)
	{
		printk("VOSEL hw mode fail\n");
	}


}



//vproc/vcore1/vsys22/vltetest
void pmic6328_buck27(void)
{
	kal_uint8 i;
	kal_uint8 vpass=0;

	for(i=0;i<8;i++)
	{
		mt6328_set_register_value(PMIC_VPROC_BURST,i);

		if(mt6328_get_register_value(PMIC_QI_VPROC_BURST)!=i)
		{
			printk("VPROC BURST test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VPROC_BURST));
			vpass=1;
		}
	}

	for(i=0;i<8;i++)
	{
		mt6328_set_register_value(PMIC_VCORE1_BURST,i);

		if(mt6328_get_register_value(PMIC_QI_VCORE1_BURST)!=i)
		{
			printk("VCORE1 BURST test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VCORE1_BURST));
			vpass=1;
		}
	}

	for(i=0;i<8;i++)
	{
		mt6328_set_register_value(PMIC_VSYS22_BURST,i);

		if(mt6328_get_register_value(PMIC_QI_VSYS22_BURST)!=i)
		{
			printk("VSYS22 BURST test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VSYS22_BURST));
			vpass=1;
		}
	}	

	for(i=0;i<8;i++)
	{
		mt6328_set_register_value(PMIC_VLTE_BURST,i);

		if(mt6328_get_register_value(PMIC_QI_VLTE_BURST)!=i)
		{
			printk("VLTE BURST test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VLTE_BURST));
			vpass=1;
		}
	}

	if(vpass==0)
	{
		printk("BURSTH test pass \n");
	}

}


//vproc/vcore1/vsys22/vltetest
void pmic6328_buck28(void)
{
	kal_uint8 i;
	kal_uint8 vpass=0;

	for(i=0;i<4;i++)
	{
		mt6328_set_register_value(PMIC_VPROC_DLC,i);

		if(mt6328_get_register_value(PMIC_QI_VPROC_DLC)!=i)
		{
			printk("VPROC DLC test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VPROC_DLC));
			vpass=1;
		}
	}

	for(i=0;i<4;i++)
	{
		mt6328_set_register_value(PMIC_VCORE1_DLC,i);

		if(mt6328_get_register_value(PMIC_QI_VCORE1_DLC)!=i)
		{
			printk("VCORE1 DLC test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VCORE1_DLC));
			vpass=1;
		}
	}

	for(i=0;i<4;i++)
	{
		mt6328_set_register_value(PMIC_VSYS22_DLC,i);

		if(mt6328_get_register_value(PMIC_QI_VSYS22_DLC)!=i)
		{
			printk("VSYS22 DLC test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VSYS22_DLC));
			vpass=1;
		}
	}	

	for(i=0;i<4;i++)
	{
		mt6328_set_register_value(PMIC_VLTE_DLC,i);

		if(mt6328_get_register_value(PMIC_QI_VLTE_DLC)!=i)
		{
			printk("VLTE DLC test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VLTE_DLC));
			vpass=1;
		}
	}

	if(vpass==0)
	{
		printk("DLC test pass \n");
	}



}



//vproc/vcore1/vsys22/vltetest
void pmic6328_buck29(void)
{
	kal_uint8 i;
	kal_uint8 vpass=0;

	for(i=0;i<4;i++)
	{
		mt6328_set_register_value(PMIC_VPROC_DLC_N,i);

		if(mt6328_get_register_value(PMIC_QI_VPROC_DLC_N)!=i)
		{
			printk("VPROC DLC_N test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VPROC_DLC_N));
			vpass=1;
		}
	}

	for(i=0;i<4;i++)
	{
		mt6328_set_register_value(PMIC_VCORE1_DLC_N,i);

		if(mt6328_get_register_value(PMIC_QI_VCORE1_DLC_N)!=i)
		{
			printk("VCORE1 DLC_N test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VCORE1_DLC_N));
			vpass=1;
		}
	}

	for(i=0;i<4;i++)
	{
		mt6328_set_register_value(PMIC_VSYS22_DLC_N,i);

		if(mt6328_get_register_value(PMIC_QI_VSYS22_DLC_N)!=i)
		{
			printk("VSYS22 DLC_N test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VSYS22_DLC_N));
			vpass=1;
		}
	}	

	for(i=0;i<4;i++)
	{
		mt6328_set_register_value(PMIC_VLTE_DLC_N,i);

		if(mt6328_get_register_value(PMIC_QI_VLTE_DLC_N)!=i)
		{
			printk("VLTE DLC_N test fail %d !=%d \n",i,mt6328_get_register_value(PMIC_QI_VLTE_DLC_N));
			vpass=1;
		}
	}

	if(vpass==0)
	{
		printk("DLC test pass \n");
	}



}


extern void pmic6328_regulator_test(void);

extern const PMU_FLAG_TABLE_ENTRY pmu_flags_table[];

void pmic6328_getid(void)
{
	printk("PMIC CID=0x%x  \n", mt6328_get_register_value(PMIC_SWCID));

}

extern void dump_ldo_status_read_debug(void);


extern void mt6328_dump_register(void);

PMU_FLAGS_LIST_ENUM pmuflag;

void pmic_dvt_entry(int test_id)
{
    printk("[pmic_dvt_entry] start : test_id=%d\n", test_id);


/*
	CH0: BATSNS
	CH1: ISENSE
	CH2: VCDT
	CH3: BAT TEMP
	CH4: PMIC TEMP
	CH5:
	CH6:
	CH7: TSX
*/


    switch (test_id)
    {


	//ADC
	 case 50:PMIC_IMM_GetOneChannelValue(0,0,0);break;
	 case 51:PMIC_IMM_GetOneChannelValue(1,0,0);break;
	 case 52:PMIC_IMM_GetOneChannelValue(2,0,0);break;
	 case 53:PMIC_IMM_GetOneChannelValue(3,0,0);break;
	 case 54:PMIC_IMM_GetOneChannelValue(4,0,0);break;
 	 case 55:PMIC_IMM_GetOneChannelValue(5,0,0);break;
	 case 56:PMIC_IMM_GetOneChannelValue(7,0,0);break;
	 


        case 61: pmic6328_auxadc2_trim111(0); break;
        case 62: pmic6328_auxadc2_trim111(1); break;
        case 63: pmic6328_auxadc2_trim111(2); break;
        case 64: pmic6328_auxadc2_trim111(3); break;
        case 65: pmic6328_auxadc2_trim111(7); break;		

        case 0: pmic6328_auxadc1_basic111(); break;

        case 2: pmic6328_auxadc3_prio111(); break;
        case 3: pmic6328_auxadc3_prio112(); break;
        case 4: pmic6328_auxadc3_prio113(); break;
        case 5: pmic6328_auxadc3_prio114(); break;
        case 6: pmic6328_auxadc3_prio115(); break;


        case 7: pmic6328_lbat111(); break;
        case 8: pmic6328_lbat112(); break;
        case 9: pmic6328_lbat113(); break;
        case 10: pmic6328_lbat114(); break;
		
        case 11: pmic6328_lbat2111(); break;
        case 12: pmic6328_lbat2112(); break;
        case 13: pmic6328_lbat2113(); break;
        case 14: pmic6328_lbat2114(); break;

        case 15: pmic6328_thr111(); break;
        case 16: pmic6328_thr112(); break;
        case 17: pmic6328_thr113(); break;
        case 18: pmic6328_thr114(); break;

        case 19: pmic6328_acc111(); break;

        case 20: pmic6328_IMP111(); break;
        case 21: pmic6328_IMP112(); break;

        case 22: pmic6328_cck11(); break;
        case 23: pmic6328_cck12(); break;
        case 24: pmic6328_cck13(); break;
        case 25: pmic6328_cck14(); break;
        case 26: pmic6328_cck15(); break;   
	 case 27: pmic6328_cck16(); break;

	//efuse
        case 100: pmic6328_efuse11(); break;
        case 101: pmic6328_efuse21(); break;

	//ldo
	 case 200: pmic6328_ldo1_sw_on_control(); break;
        case 201: pmic6328_ldo1_hw_on_control(); break;
	 case 202: pmic6328_ldo4_lp_test(); break;
        case 203: pmic6328_ldo5_fast_tran(); break;	

	 case 204: pmic6328_vtcxo_1();break;
	 case 205: pmic6328_ldo_vio18();break;

		
        case 211: pmic6328_ldo2_vosel_test(PMIC_6328_VCAMA); break;
        case 212: pmic6328_ldo2_vosel_test(PMIC_6328_VCN33); break;
        case 213: pmic6328_ldo2_vosel_test(PMIC_6328_VEFUSE); break;
        case 214: pmic6328_ldo2_vosel_test(PMIC_6328_VSIM1); break;
        case 215: pmic6328_ldo2_vosel_test(PMIC_6328_VSIM2); break;
        case 216: pmic6328_ldo2_vosel_test(PMIC_6328_VEMC33); break;
        case 217: pmic6328_ldo2_vosel_test(PMIC_6328_VMCH); break;
        case 218: pmic6328_ldo2_vosel_test(PMIC_6328_VMC); break;
        case 219: pmic6328_ldo2_vosel_test(PMIC_6328_VCAM_AF); break;
        case 220: pmic6328_ldo2_vosel_test(PMIC_6328_VGP1); break;
        case 221: pmic6328_ldo2_vosel_test(PMIC_6328_VIBR); break;
        case 222: pmic6328_ldo2_vosel_test(PMIC_6328_VCAMD); break;
        case 223: pmic6328_ldo2_vosel_test(PMIC_6328_VRF18_1); break;
        case 224: pmic6328_ldo2_vosel_test(PMIC_6328_VCAM_IO); break;
        case 225: pmic6328_ldo2_vosel_test(PMIC_6328_VSRAM); break;
        case 226: pmic6328_ldo2_vosel_test(PMIC_6328_VM); break;		

		
        case 251: pmic6328_ldo3_cal_test(PMIC_6328_VAUX18); break;
        case 252: pmic6328_ldo3_cal_test(PMIC_6328_VTCXO_0); break;
        case 253: pmic6328_ldo3_cal_test(PMIC_6328_VTCXO_1); break;	
        case 254: pmic6328_ldo3_cal_test(PMIC_6328_VAUD28); break;
        case 255: pmic6328_ldo3_cal_test(PMIC_6328_VCN28); break;		
        case 256: pmic6328_ldo3_cal_test(PMIC_6328_VCAMA); break;
        case 257: pmic6328_ldo3_cal_test(PMIC_6328_VCN33); break;
        case 258: pmic6328_ldo3_cal_test(PMIC_6328_VUSB33); break;	
        case 259: pmic6328_ldo3_cal_test(PMIC_6328_VEFUSE); break;
        case 260: pmic6328_ldo3_cal_test(PMIC_6328_VSIM1); break;	
        case 261: pmic6328_ldo3_cal_test(PMIC_6328_VSIM2); break;
        case 262: pmic6328_ldo3_cal_test(PMIC_6328_VEMC33); break;
        case 263: pmic6328_ldo3_cal_test(PMIC_6328_VMCH); break;	
        case 264: pmic6328_ldo3_cal_test(PMIC_6328_VMC); break;
        case 265: pmic6328_ldo3_cal_test(PMIC_6328_VCAM_AF); break;	
        case 266: pmic6328_ldo3_cal_test(PMIC_6328_VIO28); break;
        case 267: pmic6328_ldo3_cal_test(PMIC_6328_VGP1); break;
        case 268: pmic6328_ldo3_cal_test(PMIC_6328_VIBR); break;	
        case 269: pmic6328_ldo3_cal_test(PMIC_6328_VCAMD); break;
        case 270: pmic6328_ldo3_cal_test(PMIC_6328_VRF18_0); break;	
        case 271: pmic6328_ldo3_cal_test(PMIC_6328_VRF18_1); break;
        case 272: pmic6328_ldo3_cal_test(PMIC_6328_VIO18); break;
        case 273: pmic6328_ldo3_cal_test(PMIC_6328_VCN18); break;	
        case 274: pmic6328_ldo3_cal_test(PMIC_6328_VCAM_IO); break;
        case 275: pmic6328_ldo3_cal_test(PMIC_6328_VM); break;			


	//VPA
	 case 300: pmic6328_vpa11(); break;
        case 301: pmic6328_vpa12(); break;

	//BUCK
	 case 400: pmic6328_buck21(); break;	 
        case 401: pmic6328_buck25(); break;
	 case 402: pmic6328_buck27(); break;
        case 403: pmic6328_buck28(); break;
	 case 404: pmic6328_buck29(); break;
 	 case 405: pmic6328_buck211(); break;
	 

	//reg
	 case 410: pmic_6328_reg();break;
	

	//regulator
	 case 500: pmic6328_regulator_test(); break;
	 case 501: dump_ldo_status_read_debug(); break;


	 case 600: pmic6328_getid(); break;
	 case 601: mt6328_dump_register(); break;
	 case 602: 
	 	printk("flag=%d val=0x%x\n",pmuflag,mt6328_get_register_value(pmuflag));
		break;
	
        default:
            printk("[pmic_dvt_entry] test_id=%d\n", test_id);
            break;
    }

    printk("[pmic_dvt_entry] end\n\n");
}
