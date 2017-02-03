#ifndef BUILD_LK
	#include <linux/string.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
//#include <mach/mt_gpio.h>
	#include <mt-plat/mt_gpio.h>
//#include <linux/xlog.h>
#endif

//#include <cust_adc.h>
//#include <cust_gpio_usage.h>
// ---------------------------------------------------------------------------
//  Local Constantsq
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)

// physical dimension
#define PHYSICAL_WIDTH        (68)
#define PHYSICAL_HIGHT         (121)

#define REGFLAG_DELAY             							0XFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER
#define LCM_ID_ILI9806E 										(0x98)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)							lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)																						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)												lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)																							lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   									lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

#define   LCM_DSI_CMD_MODE							(0)

#ifndef BUILD_LK
extern atomic_t ESDCheck_byCPU;
#endif

struct LCM_setting_table {
    unsigned int cmd;
    unsigned int count;
    unsigned char para_list[64];
};
							
static struct LCM_setting_table lcm_initialization_setting[] = {

{0XFF, 5, {0XFF, 0X98, 0X06, 0X04, 0X01}},
{0X8, 1, {0X10}},
{0X21, 1, {0X01}},
{0X30, 1, {0X01}},
{0X31, 1, {0X02}},
{0X40, 1, {0X11}},
{0X41, 1, {0X77}},
{0X42, 1, {0X03}},
{0X43, 1, {0X09}},
{0X44, 1, {0X07}},
{0X50, 1, {0X78}},
{0X51, 1, {0X78}},
{0X52, 1, {0X00}},
{0X53, 1, {0X54}},
{0X57, 1, {0X50}},
{0X60, 1, {0X07}},
{0X61, 1, {0X00}},
{0X62, 1, {0X08}},
{0X63, 1, {0X00}},
{0XA0, 1, {0X00}},
{0XA1, 1, {0X03}},
{0XA2, 1, {0X0A}},
{0XA3, 1, {0X0E}},
{0XA4, 1, {0X09}},
{0XA5, 1, {0X18}},
{0XA6, 1, {0X09}},
{0XA7, 1, {0X08}},
{0XA8, 1, {0X03}},
{0XA9, 1, {0X09}},
{0XAA, 1, {0X04}},
{0XAB, 1, {0X03}},
{0XAC, 1, {0X08}},
{0XAD, 1, {0X36}},
{0XAE, 1, {0X31}},
{0XAF, 1, {0X00}},
{0XC0, 1, {0X00}},
{0XC1, 1, {0X02}},
{0XC2, 1, {0X07}},
{0XC3, 1, {0X0D}},
{0XC4, 1, {0X05}},
{0XC5, 1, {0X12}},
{0XC6, 1, {0X09}},
{0XC7, 1, {0X08}},
{0XC8, 1, {0X03}},
{0XC9, 1, {0X06}},
{0XCA, 1, {0X09}},
{0XCB, 1, {0X03}},
{0XCC, 1, {0X0D}},
{0XCD, 1, {0X29}},
{0XCE, 1, {0X22}},
{0XCF, 1, {0X00}},
{0XFF, 5, {0XFF, 0X98, 0X06, 0X04, 0X06}},
{0X0, 1, {0X21}},
{0X1, 1, {0X0A}},
{0X2, 1, {0X00}},
{0X3, 1, {0X00}},
{0X4, 1, {0X01}},
{0X5, 1, {0X01}},
{0X6, 1, {0X80}},
{0X7, 1, {0X06}},
{0X8, 1, {0X01}},
{0X9, 1, {0X80}},
{0XA, 1, {0X00}},
{0XB, 1, {0X00}},
{0XC, 1, {0X0A}},
{0XD, 1, {0X0A}},
{0XE, 1, {0X00}},
{0XF, 1, {0X00}},
{0X10, 1, {0XF0}},
{0X11, 1, {0XF4}},
{0X12, 1, {0X04}},
{0X13, 1, {0X00}},
{0X14, 1, {0X00}},
{0X15, 1, {0XC0}},
{0X16, 1, {0X08}},
{0X17, 1, {0X00}},
{0X18, 1, {0X00}},
{0X19, 1, {0X00}},
{0X1A, 1, {0X00}},
{0X1B, 1, {0X00}},
{0X1C, 1, {0X00}},
{0X1D, 1, {0X00}},
{0X20, 1, {0X01}},
{0X21, 1, {0X23}},
{0X22, 1, {0X45}},
{0X23, 1, {0X67}},
{0X24, 1, {0X01}},
{0X25, 1, {0X23}},
{0X26, 1, {0X45}},
{0X27, 1, {0X67}},
{0X30, 1, {0X01}},
{0X31, 1, {0X11}},
{0X32, 1, {0X00}},
{0X33, 1, {0XEE}},
{0X34, 1, {0XFF}},
{0X35, 1, {0XBB}},
{0X36, 1, {0XCA}},
{0X37, 1, {0XDD}},
{0X38, 1, {0XAC}},
{0X39, 1, {0X76}},
{0X3A, 1, {0X67}},
{0X3B, 1, {0X22}},
{0X3C, 1, {0X22}},
{0X3D, 1, {0X22}},
{0X3E, 1, {0X22}},
{0X3F, 1, {0X22}},
{0X40, 1, {0X22}},
{0X52, 1, {0X10}},
{0X53, 1, {0X10}},
{0XFF, 5, {0XFF, 0X98, 0X06, 0X04, 0X07}},
{0X17, 1, {0X22}},
{0X2, 1, {0X77}},
{0XE1, 1, {0X79}},
{0XFF, 5, {0XFF, 0X98, 0X06, 0X04, 0X00}},
{0X35, 1, {0X00}},
{0X21, 1, {0X00}},
{0X11, 1, {0X00}},
{REGFLAG_DELAY, 120, {}},
{0X29, 1, {0X00}},
{REGFLAG_DELAY, 20, {}},	  	 

};

#if 0 //Singh
static struct LCM_setting_table lcm_sleep_out_setting[] = {
  // Sleep Out
	{0x11, 0, {0x00}},
  {REGFLAG_DELAY, 120, {}},

  // Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
  
  // Sleep Mode On
	{0x10, 1, {0x00}},
  {REGFLAG_DELAY, 120, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


//static struct LCM_setting_table lcm_backlight_level_setting[] = {
	//{0x51, 1, {0xFF}},
	//{REGFLAG_END_OF_TABLE, 0x00, {}}
//};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

  for(i = 0; i < count; i++)
  {		
  	unsigned cmd;
    cmd = table[i].cmd;		
        
    switch (cmd)
    {			
    	case REGFLAG_DELAY :
      	MDELAY(table[i].count);
        break;
				
      case REGFLAG_END_OF_TABLE :
        break;
				
      default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
     }
   }	
}

static void init_lcm_registers(void)
{
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS * params)
{
	memset(params, 0, sizeof(LCM_PARAMS)); 

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

   params->physical_width=PHYSICAL_WIDTH;
   params->physical_height=PHYSICAL_HIGHT;

	// enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine. 

	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;	
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST; 
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB; 
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888; 

	// Highly depends on LCD driver capability. 
	params->dsi.packet_size = 256; 
	// Video mode setting 
	params->dsi.intermediat_buffer_num = 2; 
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888; 

	params->dsi.vertical_sync_active = 2; 
	params->dsi.vertical_backporch = 20; 
	params->dsi.vertical_frontporch = 20; 
	params->dsi.vertical_active_line = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 60;
	params->dsi.horizontal_frontporch				= 200;
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;

	// Bit rate calculation
	//params->dsi.pll_div1=35;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
	//params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)

	/* ESD or noise interference recovery For video mode LCM only. */
	// Send TE packet to LCM in a period of n frames and check the response.
	//params->dsi.lcm_int_te_monitor = FALSE;
	//params->dsi.lcm_int_te_period = 1;		// Unit : frames

	// Need longer FP for more opportunity to do int. TE monitor applicably.
	//if(params->dsi.lcm_int_te_monitor)
	//	params->dsi.vertical_frontporch *= 2;

	// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.)
	//params->dsi.lcm_ext_te_monitor = FALSE;
	// Non-continuous clock
	//params->dsi.noncont_clock = TRUE;
	//params->dsi.noncont_clock_period = 2;	// Unit : frames

	// DSI MIPI Spec parameters setting
	/*params->dsi.HS_TRAIL = 6;
	params->dsi.HS_ZERO = 9;
	params->dsi.HS_PRPR = 5;
	params->dsi.LPX = 4;
	params->dsi.TA_SACK = 1;
	params->dsi.TA_GET = 20;
	params->dsi.TA_SURE = 6;
	params->dsi.TA_GO = 16;
	params->dsi.CLK_TRAIL = 5;
	params->dsi.CLK_ZERO = 18;
	params->dsi.LPX_WAIT = 1;
	params->dsi.CONT_DET = 0;
	params->dsi.CLK_HS_PRPR = 4;*/
	// Bit rate calculation
	params->dsi.PLL_CLOCK = 241;
}



static void lcm_suspend(void)
{
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	
	SET_RESET_PIN(1);	
	MDELAY(10);	
	SET_RESET_PIN(0);
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
		       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] =
	    (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] =
	    (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

//extern void DSI_clk_HS_mode(char enter);
//extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
static unsigned int lcm_compare_id(void)
{
				int array[4];
        char buffer[5];

        int id=0;
 //       int lcm_adc = 0, data[4] = {0,0,0,0};
        //Do reset here
//      return 1;
        SET_RESET_PIN(1);
        SET_RESET_PIN(0);
        MDELAY(25);       
        SET_RESET_PIN(1);
        MDELAY(50);      
       
        array[0]=0x00063902;
        array[1]=0x0698ffff;
        array[2]=0x00000104;
        dsi_set_cmdq(array, 3, 1);
        MDELAY(10);
 
        array[0]=0x00013700;
        dsi_set_cmdq(array, 1, 1);
        //read_reg_v2(0x04, buffer, 3);//if read 0x04,should get 0x008000,that is both OK.
    
        read_reg_v2(0x00, buffer,1);
        id = buffer[0]; ///////////////////////0x98
 
 /*
        read_reg_v2(0x01, buffer,1);
        id_midd = buffer[0]; ///////////////////////0x06
 
        read_reg_v2(0x02, buffer,1);
        id_low = buffer[0]; ////////////////////////0x04
 
        id =(id_high << 16) | (id_midd << 8) | id_low;
 */
//        IMM_GetOneChannelValue(12,data,&lcm_adc); //read lcd _id
#if defined(BUILD_LK)
			  printf("ili9806e_hlt_hd720_dsi lk -- ili9806e 0x%x\n",id);
#else
			  printk("ili9806e_hlt_hd720_dsi kernel -- ili9806e 0x%x\n",  id);
#endif

        if(LCM_ID_ILI9806E == id) //&& (lcm_adc > 3000))
					return 1;
				else
					return 0;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
		
		int array[4];
		char buf[5];
		char id_high=0;
		char id_low=0;
		int id=0;
	
		array[0] = 0x00063902;
		array[1] = 0x0698FFFF;
		array[2] = 0x00000104;
		dsi_set_cmdq(array, 3, 1);
	
		atomic_set(&ESDCheck_byCPU,1);
		array[0] = 0x00013700;
		dsi_set_cmdq(array, 1, 1);
		read_reg_v2(0x00, &buf[0], 1);  //0x98
	
		array[0] = 0x00013700;		
		dsi_set_cmdq(array, 1, 1);
		read_reg_v2(0x01, &buf[1], 1);  //0x06
	
		array[0] = 0x00013700;
		dsi_set_cmdq(array, 1, 1);
		read_reg_v2(0x02, &buf[2], 1);  //0x04
	
		id_high = buf[0];
		id_low = buf[1];
		id = (id_high<<8) | id_low;
		atomic_set(&ESDCheck_byCPU,0);
		printk("[%s=%d line]id=0x%x\n,",__FUNCTION__,__LINE__,id); 
	  return (0x9806 == id)?1:0;

}
static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(100);
	init_lcm_registers();
}
static void lcm_resume(void)
{
	lcm_init();
	
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806e_hlt_hd720_dsi = {
	.name = "ili9806e_hlt_hd720_dsi",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params 		= lcm_get_params,
	.init 					= lcm_init,
	.suspend 				= lcm_suspend,
	.resume 				= lcm_resume,
	.compare_id = lcm_compare_id,
	.ata_check		= lcm_ata_check,
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
  .update         = lcm_update,
#endif
};

