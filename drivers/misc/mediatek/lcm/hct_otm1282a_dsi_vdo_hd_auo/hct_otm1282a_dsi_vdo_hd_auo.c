/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
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
//	#include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define LCM_ID_OTM1282 (0x1282)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define REGFLAG_DELAY                                                                   0XFE
#define REGFLAG_END_OF_TABLE                                                            0x100   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)                lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)           lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                                                          lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                                      lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                                                                   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

#define   LCM_DSI_CMD_MODE                                                      0

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

	{0x0, 1, {0x00}},
	{0xFF, 3, {0x12, 0x82, 0x01}},
	{0x0, 1, {0x80}},
	{0xFF, 2, {0x12, 0x82}},

	{0x0, 1, {0x92}},
	{0xFF, 2, {0x20, 0x02}},
	{0x0, 1, {0xB4}},
	{0xC0, 1, {0x40}},
	{0x0, 1, {0x91}},
	{0xB3, 2, {0x08, 0x10}},
	{0x0, 1, {0xB3}},
	{0xC0, 1, {0x33}},
	{0x0, 1, {0x00}},
	{0x1C, 1, {0x32}},
	{0x0, 1, {0x84}},
	{0xA4, 1, {0x00}},

/*TE CLK change from 65Hz to 60 Hz*/
	{0x0, 1, {0x80}},
	{0xC0, 14, {0x00, 0x85, 0x00, 0x2C, 0x2C, 0x00, 0x7B, 0x2C, 0x2C, 0x00, 0x7B, 0x00, 0x2C, 0x2C}},

	{0x0, 1, {0xA0}},
	{0xC0, 7, {0x00, 0x00, 0x00, 0x07, 0x00, 0x19, 0x09}},

	{0x0, 1, {0xD0}},
	{0xC0, 7, {0x00, 0x00, 0x00, 0x07, 0x00, 0x19, 0x09}},

	{0x0, 1, {0x80}},
	{0xC1, 2, {0x55, 0x55}},
	{0x0, 1, {0x90}},
	{0xC1, 3, {0x66, 0x00, 0x00}},
	{0x0, 1, {0x80}},
	{0xC2, 4, {0x83, 0x01, 0x45, 0x45}},
	{0x0, 1, {0x90}},
	{0xC2, 15, {0xA9, 0x2C, 0x01, 0x00, 0x00, 0xAB, 0x2C, 0x01, 0x00, 0x00, 0xAA, 0x2C, 0x01, 0x00, 0x00}},
	{0x0, 1, {0xA0}},
	{0xC2, 5, {0xA8, 0x2C, 0x01, 0x00, 0x00}},
	{0x0, 1, {0xEC}},
	{0xC2, 1, {0x00}},
	{0x0, 1, {0xFA}},
	{0xC2, 3, {0x00, 0x80, 0x01}},
	{0x0, 1, {0x80}},
	{0xC3, 4, {0x83, 0x01, 0x45, 0x45}},
	{0x0, 1, {0x90}},
	{0xC3, 15, {0xA9, 0x2C, 0x01, 0x00, 0x00, 0xAB, 0x2C, 0x01, 0x00, 0x00, 0xAA, 0x2C, 0x01, 0x00, 0x00}},
	{0x0, 1, {0xA0}},
	{0xC3, 5, {0xA8, 0x2C, 0x01, 0x00, 0x00}},
	{0x0, 1, {0xEC}},
	{0xC3, 1, {0x00}},
	{0x0, 1, {0x90}},
	{0xCB, 15, {0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x00}},
	{0x0, 1, {0xA0}},
	{0xCB, 15, {0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x0, 1, {0xB0}},
	{0xCB, 15, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0xC0, 0x00, 0x00, 0x00, 0x00}},
	{0x0, 1, {0xC0}},
	{0xCB, 15, {0xFF, 0x28, 0xEB, 0x28, 0xEB, 0x14, 0x14, 0x14, 0x14, 0x00, 0x14, 0x14, 0xD7, 0x28, 0x28}},
	{0x0, 1, {0xD0}},
	{0xCB, 15, {0x28, 0x28, 0x14, 0x14, 0x14, 0xD7, 0x04, 0xF7, 0x04, 0x14, 0x14, 0x14, 0x14, 0x04, 0x04}},
	{0x0, 1, {0xE0}},
	{0xCB, 15, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x15, 0x07, 0x14, 0x14, 0x00, 0x00}},
	{0x0, 1, {0xF0}},
	{0xCB, 12, {0x30, 0x03, 0xFC, 0x03, 0xF0, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x0, 1, {0x80}},
	{0xCC, 10, {0x0F, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x10}},
	{0x0, 1, {0xB0}},
	{0xCC, 10, {0x0F, 0x06, 0x05, 0x04, 0x03, 0x07, 0x08, 0x09, 0x0A, 0x10}},
	{0x0, 1, {0x8A}},
	{0xCD, 1, {0x0B}},
	{0x0, 1, {0xA0}},
	{0xCD, 15, {0x12, 0x14, 0x15, 0x04, 0x05, 0x01, 0x0A, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x26, 0x25, 0x24}},
	{0x0, 1, {0xB0}},
	{0xCD, 15, {0x23, 0x22, 0x21, 0x20, 0x1F, 0x2D, 0x2D, 0x2D, 0x2D, 0x13, 0x0B, 0x0C, 0x2D, 0x2D, 0x2D}},
	{0x0, 1, {0xC0}},
	{0xCD, 10, {0x2D, 0x2D, 0x2D, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x1D, 0x2D}},
	{0x0, 1, {0xD0}},
	{0xCD, 15, {0x12, 0x14, 0x15, 0x02, 0x03, 0x01, 0x0A, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x26, 0x25, 0x24}},
	{0x0, 1, {0xE0}},
	{0xCD, 15, {0x23, 0x22, 0x21, 0x20, 0x1F, 0x2D, 0x2D, 0x2D, 0x2D, 0x13, 0x0B, 0x0C, 0x2D, 0x2D, 0x2D}},
	{0x0, 1, {0xF0}},
	{0xCD, 10, {0x2D, 0x2D, 0x2D, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x1D, 0x2D}},
	{0x0, 1, {0x00}},
	{0xD9, 1, {0x72}},
	{0x0, 1, {0x00}},
	{0xD8, 2, {0x34, 0x34}},
	{0x0, 1, {0x90}},
	{0xC5, 4, {0x92, 0xD6, 0xAD, 0xB0}},
	{0x0, 1, {0xA0}},
	{0xC5, 4, {0x92, 0xD6, 0xAD, 0xB0}},
/*use gamma 2.2 parameters*/
	{0x0, 1, {0x00}},
/*Gamma Red+ setting*/
	{0xE1, 24, {0x1B, 0x21, 0x2F, 0x3C, 0x46, 0x4E, 0x5B, 0x6D, 0x79, 0x89, 0x94, 0x9B, 0x60, 0x5D, 0x5A, 0x50, 0x42, 0x34, 0x2A, 0x24, 0x1F, 0x19, 0x18, 0x17}},
	{0x0, 1, {0x00}},
/*Gamma Red- setting*/
	{0xE2, 24, {0x1B, 0x21, 0x2F, 0x3C, 0x46, 0x4E, 0x5B, 0x6D, 0x79, 0x89, 0x94, 0x9B, 0x60, 0x5D, 0x5A, 0x50, 0x42, 0x34, 0x2A, 0x24, 0x1F, 0x19, 0x18, 0x17}},
	{0x0, 1, {0x00}},
/*Gamma Green+ setting*/
	{0xE3, 24, {0x1B, 0x21, 0x2F, 0x3C, 0x46, 0x4E, 0x5B, 0x6D, 0x79, 0x89, 0x94, 0x9B, 0x60, 0x5D, 0x5A, 0x50, 0x42, 0x34, 0x2A, 0x24, 0x1F, 0x19, 0x18, 0x17}},
	{0x0, 1, {0x00}},
/*Gamma Green- setting*/
	{0xE4, 24, {0x1B, 0x21, 0x2F, 0x3C, 0x46, 0x4E, 0x5B, 0x6D, 0x79, 0x89, 0x94, 0x9B, 0x60, 0x5D, 0x5A, 0x50, 0x42, 0x34, 0x2A, 0x24, 0x1F, 0x19, 0x18, 0x17}},
	{0x0, 1, {0x00}},
/*Gamma Blue+ setting*/
	{0xE5, 24, {0x1B, 0x21, 0x2F, 0x3C, 0x46, 0x4E, 0x5B, 0x6D, 0x79, 0x89, 0x94, 0x9B, 0x60, 0x5D, 0x5A, 0x50, 0x42, 0x34, 0x2A, 0x24, 0x1F, 0x19, 0x18, 0x17}},
	{0x0, 1, {0x00}},
/*Gamma Blue- setting*/
	{0xE6, 24, {0x1B, 0x21, 0x2F, 0x3C, 0x46, 0x4E, 0x5B, 0x6D, 0x79, 0x89, 0x94, 0x9B, 0x60, 0x5D, 0x5A, 0x50, 0x42, 0x34, 0x2A, 0x24, 0x1F, 0x19, 0x18, 0x17}},
	{0x0, 1, {0x93}},

	{0xF5, 1, {0x10}},
	{0x0, 1, {0x97}},
	{0xC5, 2, {0x55, 0x50}},
	{0x0, 1, {0xA7}},
	{0xC5, 2, {0x55, 0x50}},
/*Disable level 2 command*/
	{0x0, 1, {0x80}},
	{0xA5, 1, {0x0C}},
	{0x0, 1, {0xB3}},
	{0xC0, 1, {0x33}},
	{0x0, 1, {0x80}},
	{0xC4, 2, {0x04, 0x0F}},
	{0x0, 1, {0xA0}},
	{0xC1, 2, {0x02, 0xE0}},
	{0x0, 1, {0x00}},
	{0xFF, 3, {0xFF, 0xFF, 0xFF}},

	// Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 160, {}},

	// Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 60, {}},
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},
};

static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
        // Display off sequence
        {0x28, 0, {0x00}},
        {REGFLAG_DELAY, 100, {}},

    // Sleep Mode On
        {0x10, 0, {0x00}},
        {REGFLAG_DELAY, 200, {}},
};
static struct LCM_setting_table lcm_compare_id_setting[] = {
        // Display off sequence
        {0xF0,  5,      {0x55, 0xaa, 0x52,0x08,0x00}},
        {REGFLAG_DELAY, 10, {}},
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
        unsigned int i;
	unsigned cmd;

	for(i = 0; i < count; i++) {

		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY :
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE :
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
			MDELAY(2);
		}
	}
        
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

//	params->dbi.te_mode 	= LCM_DBI_TE_MODE_DISABLED;
//	params->dsi.mode	= SYNC_EVENT_VDO_MODE;
	// enable tearing-free
	params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;//LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
	params->dsi.mode   = SYNC_EVENT_VDO_MODE;

	params->dsi.intermediat_buffer_num = 4;


	// 1 Three lane or Four lane
	params->dsi.LANE_NUM               = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active                = 4;//0x3;// 3    2
	params->dsi.vertical_backporch                  = 20;//0x10;// 20   1
	params->dsi.vertical_frontporch                 = 20;//0x10; // 1  12
  	params->dsi.vertical_frontporch_for_low_power 	= 20;
	params->dsi.vertical_active_line                = FRAME_HEIGHT; 

	params->dsi.ufoe_params.vlc_disable 		= 1;
	params->dsi.ufoe_params.vlc_disable 		= 1;
	params->dsi.ufoe_params.lr_mode_en 		= 1;

	params->dsi.horizontal_sync_active              = 8;//0x0B;// 50  2
	params->dsi.horizontal_backporch                = 65;//0x40 ;
	params->dsi.horizontal_frontporch               = 75;//0x40 ;
	params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

	params->dsi.cont_clock	= 0; //1;

	params->dsi.esd_check_enable 			= 1;
	params->dsi.customization_esd_check_enable	= 1;
	params->dsi.lcm_esd_check_table[0].cmd		= 0x0a;
	params->dsi.lcm_esd_check_table[0].count	= 1;
	params->dsi.lcm_esd_check_table[0].para_list[0]	= 0x9c;

	params->dsi.PLL_CLOCK=240;//227;//254;//254//247
}

static void lcm_init(void)
{
        SET_RESET_PIN(1);
        MDELAY(50); 
        SET_RESET_PIN(0);
        MDELAY(50); 
        
        SET_RESET_PIN(1);
        MDELAY(120);      

        push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
#ifndef BUILD_LK
	push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
#endif
}


static void lcm_resume(void)
{
#ifndef BUILD_LK
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
#endif
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
        unsigned int x0 = x;
        unsigned int y0 = y;
        unsigned int x1 = x0 + width - 1;
        unsigned int y1 = y0 + height - 1;

        unsigned char x0_MSB = ((x0>>8)&0xFF);
        unsigned char x0_LSB = (x0&0xFF);
        unsigned char x1_MSB = ((x1>>8)&0xFF);
        unsigned char x1_LSB = (x1&0xFF);
        unsigned char y0_MSB = ((y0>>8)&0xFF);
        unsigned char y0_LSB = (y0&0xFF);
        unsigned char y1_MSB = ((y1>>8)&0xFF);
        unsigned char y1_LSB = (y1&0xFF);

        unsigned int data_array[16];

        data_array[0]= 0x00053902;
        data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
        data_array[2]= (x1_LSB);
        dsi_set_cmdq(data_array, 3, 1);
        
        data_array[0]= 0x00053902;
        data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
        data_array[2]= (y1_LSB);
        dsi_set_cmdq(data_array, 3, 1);

        data_array[0]= 0x00290508; //HW bug, so need send one HS packet
        dsi_set_cmdq(data_array, 1, 1);
        
        data_array[0]= 0x002c3909;
        dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
        unsigned int id0,id1,id=0;
        unsigned char buffer[5];
        unsigned int array[16];  

        SET_RESET_PIN(1);
        SET_RESET_PIN(0);
        MDELAY(1);
        
        SET_RESET_PIN(1);
        MDELAY(20); 

        array[0] = 0x00053700;// read id return two byte,version and id
        dsi_set_cmdq(array, 1, 1);
        
        read_reg_v2(0xA1, buffer, 5);   //018B1283ff
        id0 = buffer[2];
        id1 = buffer[3];
        id=(id0<<8)|id1;
        
    #ifdef BUILD_LK
                printf("%s, LK otm1282a debug: otm1282a id = 0x%08x\n", __func__, id);
    #else
                printk("%s, kernel otm1282a horse debug: otm1282a id = 0x%08x\n", __func__, id);
    #endif

    if(id == LCM_ID_OTM1282)
        return 1;
    else
        return 0;
}

static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	//hct_set_hs_read();
	array[0] = 0x00013708;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0a, buffer, 1);

	if(buffer[0]==0x9c)
	{
		return FALSE;
	}
	else
	{                        
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
        lcm_init();
        lcm_resume();

        return TRUE;
}

LCM_DRIVER hct_otm1282a_dsi_vdo_hd_auo = 
{
	.name		= "hct_otm1282a_dsi_vdo_hd_auo",
	.set_util_funcs	= lcm_set_util_funcs,
	.get_params	= lcm_get_params,
	.init		= lcm_init,
	.suspend	= lcm_suspend,
	.resume		= lcm_resume,
	.compare_id	= lcm_compare_id,        
#if (LCM_DSI_CMD_MODE)
	//.set_backlight= lcm_setbacklight,
	.update         = lcm_update,
#endif
	//.esd_check = lcm_esd_check,
	//.esd_recover=lcm_esd_recover,
};
