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

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)
#define LCM_ID                       						(0x1284)

#define REGFLAG_DELAY             							(0XFFFE)
#define REGFLAG_END_OF_TABLE      							(0xFFFF)	// END OF REGISTERS MARKER


#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

#define LCM_RM68200_ID 		(0x6820)

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {

	/*
Note :

Data ID will depends on the following rule.

count of parameters > 1      => Data ID = 0x39
count of parameters = 1      => Data ID = 0x15
count of parameters = 0      => Data ID = 0x05

Struclcm_deep_sleep_mode_in_settingture Format :

{DCS command, count of parameters, {parameter list}}
{REGFLAG_DELAY, milliseconds of time, {}},

...

Setting ending by predefined flag

{REGFLAG_END_OF_TABLE, 0x00, {}}
*/
//RM38200+HSD5.94 YKL CODE
{0xfe, 1, {0x01}},
{0x24, 1, {0xc0}},
{0x25, 1, {0x53}},
{0x26, 1, {0x00}},
{0x2b, 1, {0xe5}},
{0x27, 1, {0x0a}},
{0x29, 1, {0x0a}},
{0x2f, 1, {0x44}},
{0x34, 1, {0x59}},
{0x1b, 1, {0x00}},
{0x12, 1, {0x08}},
{0x46, 1, {0x90}},
{0x52, 1, {0x8a}},
{0x53, 1, {0x00}},
{0x54, 1, {0x8a}},
{0x55, 1, {0x00}},
{0x5f, 1, {0x12}},
{0xfe, 1, {0x03}},
{0x0, 1, {0x05}},
{0x1, 1, {0x16}},
{0x2, 1, {0x01}},
{0x3, 1, {0x08}},
{0x4, 1, {0x00}},
{0x5, 1, {0x00}},
{0x6, 1, {0x50}},
{0x7, 1, {0x00}},
{0x8, 1, {0x00}},
{0x9, 1, {0x00}},
{0xa, 1, {0x00}},
{0xb, 1, {0x00}},
{0xc, 1, {0x00}},
{0xd, 1, {0x00}},
{0xe, 1, {0x05}},
{0xf, 1, {0x06}},
{0x10, 1, {0x07}},
{0x11, 1, {0x08}},
{0x12, 1, {0x00}},
{0x13, 1, {0x40}},
{0x14, 1, {0x00}},
{0x15, 1, {0xc5}},
{0x16, 1, {0x08}},
{0x17, 1, {0x09}},
{0x18, 1, {0x0a}},
{0x19, 1, {0x0b}},
{0x1a, 1, {0x0c}},
{0x1b, 1, {0x00}},
{0x1c, 1, {0x40}},
{0x1d, 1, {0x00}},
{0x1e, 1, {0x85}},
{0x1f, 1, {0x08}},
{0x20, 1, {0x00}},
{0x21, 1, {0x00}},
{0x22, 1, {0x05}},
{0x23, 1, {0x04}},
{0x24, 1, {0x06}},
{0x25, 1, {0x2d}},
{0x26, 1, {0x00}},
{0x27, 1, {0x08}},
{0x28, 1, {0x0a}},
{0x29, 1, {0x2d}},
{0x2a, 1, {0x00}},
{0x2b, 1, {0x00}},
{0x2d, 1, {0x00}},
{0x2f, 1, {0x00}},
{0x30, 1, {0x00}},
{0x31, 1, {0x00}},
{0x32, 1, {0x00}},
{0x33, 1, {0x00}},
{0x34, 1, {0x00}},
{0x35, 1, {0x00}},
{0x36, 1, {0x00}},
{0x37, 1, {0x00}},
{0x38, 1, {0x00}},
{0x39, 1, {0x00}},
{0x3a, 1, {0x00}},
{0x3b, 1, {0x00}},
{0x3d, 1, {0x00}},
{0x3f, 1, {0x00}},
{0x40, 1, {0x00}},
{0x41, 1, {0x00}},
{0x42, 1, {0x00}},
{0x43, 1, {0x00}},
{0x44, 1, {0x00}},
{0x45, 1, {0x00}},
{0x46, 1, {0x00}},
{0x47, 1, {0x00}},
{0x48, 1, {0x00}},
{0x49, 1, {0x00}},
{0x4a, 1, {0x00}},
{0x4b, 1, {0x00}},
{0x4c, 1, {0x00}},
{0x4d, 1, {0x00}},
{0x4e, 1, {0x00}},
{0x4f, 1, {0x00}},
{0x50, 1, {0x00}},
{0x51, 1, {0x00}},
{0x52, 1, {0x00}},
{0x53, 1, {0x00}},
{0x54, 1, {0x00}},
{0x55, 1, {0x00}},
{0x56, 1, {0x00}},
{0x58, 1, {0x00}},
{0x59, 1, {0x00}},
{0x5a, 1, {0x00}},
{0x5b, 1, {0x00}},
{0x5c, 1, {0x00}},
{0x5d, 1, {0x00}},
{0x5e, 1, {0x00}},
{0x5f, 1, {0x00}},
{0x60, 1, {0x00}},
{0x61, 1, {0x00}},
{0x62, 1, {0x00}},
{0x63, 1, {0x00}},
{0x64, 1, {0x00}},
{0x65, 1, {0x00}},
{0x66, 1, {0x00}},
{0x67, 1, {0x00}},
{0x68, 1, {0x00}},
{0x69, 1, {0x00}},
{0x6a, 1, {0x00}},
{0x6b, 1, {0x00}},
{0x6c, 1, {0x00}},
{0x6d, 1, {0x00}},
{0x6e, 1, {0x00}},
{0x6f, 1, {0x00}},
{0x70, 1, {0x00}},
{0x71, 1, {0x00}},
{0x72, 1, {0x00}},
{0x73, 1, {0x00}},
{0x74, 1, {0x01}},
{0x75, 1, {0x01}},
{0x76, 1, {0x01}},
{0x77, 1, {0x01}},
{0x78, 1, {0x01}},
{0x79, 1, {0x01}},
{0x7a, 1, {0x00}},
{0x7b, 1, {0x00}},
{0x7c, 1, {0x00}},
{0x7d, 1, {0x00}},
{0x7e, 1, {0x01}},
{0x7f, 1, {0x09}},
{0x80, 1, {0x0b}},
{0x81, 1, {0x0d}},
{0x82, 1, {0x0f}},
{0x83, 1, {0x3f}},
{0x84, 1, {0x3f}},
{0x85, 1, {0x1c}},
{0x86, 1, {0x1c}},
{0x87, 1, {0x3f}},
{0x88, 1, {0x3f}},
{0x89, 1, {0x3f}},
{0x8a, 1, {0x3f}},
{0x8b, 1, {0x3f}},
{0x8c, 1, {0x3f}},
{0x8d, 1, {0x3f}},
{0x8e, 1, {0x3f}},
{0x8f, 1, {0x3f}},
{0x90, 1, {0x3f}},
{0x91, 1, {0x1d}},
{0x92, 1, {0x1c}},
{0x93, 1, {0x03}},
{0x94, 1, {0x02}},
{0x95, 1, {0x1c}},
{0x96, 1, {0x1d}},
{0x97, 1, {0x3f}},
{0x98, 1, {0x3f}},
{0x99, 1, {0x3f}},
{0x9a, 1, {0x3f}},
{0x9b, 1, {0x3f}},
{0x9c, 1, {0x3f}},
{0x9d, 1, {0x3f}},
{0x9e, 1, {0x3f}},
{0x9f, 1, {0x3f}},
{0xa0, 1, {0x3f}},
{0xa2, 1, {0x1c}},
{0xa3, 1, {0x1c}},
{0xa4, 1, {0x3f}},
{0xa5, 1, {0x3f}},
{0xa6, 1, {0x0e}},
{0xa7, 1, {0x0c}},
{0xa9, 1, {0x0a}},
{0xaa, 1, {0x08}},
{0xab, 1, {0x00}},
{0xac, 1, {0x02}},
{0xad, 1, {0x0e}},
{0xae, 1, {0x0c}},
{0xaf, 1, {0x0a}},
{0xb0, 1, {0x08}},
{0xb1, 1, {0x3f}},
{0xb2, 1, {0x3f}},
{0xb3, 1, {0x1c}},
{0xb4, 1, {0x1c}},
{0xb5, 1, {0x3f}},
{0xb6, 1, {0x3f}},
{0xb7, 1, {0x3f}},
{0xb8, 1, {0x3f}},
{0xb9, 1, {0x3f}},
{0xba, 1, {0x3f}},
{0xbb, 1, {0x3f}},
{0xbc, 1, {0x3f}},
{0xbd, 1, {0x3f}},
{0xbe, 1, {0x3f}},
{0xbf, 1, {0x1c}},
{0xc0, 1, {0x1d}},
{0xc1, 1, {0x00}},
{0xc2, 1, {0x01}},
{0xc3, 1, {0x1d}},
{0xc4, 1, {0x1c}},
{0xc5, 1, {0x3f}},
{0xc6, 1, {0x3f}},
{0xc7, 1, {0x3f}},
{0xc8, 1, {0x3f}},
{0xc9, 1, {0x3f}},
{0xca, 1, {0x3f}},
{0xcb, 1, {0x3f}},
{0xcc, 1, {0x3f}},
{0xcd, 1, {0x3f}},
{0xce, 1, {0x3f}},
{0xcf, 1, {0x1c}},
{0xd0, 1, {0x1c}},
{0xd1, 1, {0x3f}},
{0xd2, 1, {0x3f}},
{0xd3, 1, {0x09}},
{0xd4, 1, {0x0b}},
{0xd5, 1, {0x0d}},
{0xd6, 1, {0x0f}},
{0xd7, 1, {0x03}},
{0xdf, 1, {0x00}},
{0xdc, 1, {0x02}},
{0xde, 1, {0x08}},
{0xfe, 1, {0x0e}},
{0x1, 1, {0x75}},
{0xfe, 1, {0x04}},
{0x60, 1, {0x00}},
{0x61, 1, {0x06}},
{0x62, 1, {0x0a}},
{0x63, 1, {0x0c}},
{0x64, 1, {0x04}},
{0x65, 1, {0x10}},
{0x66, 1, {0x0e}},
{0x67, 1, {0x0a}},
{0x68, 1, {0x17}},
{0x69, 1, {0x0c}},
{0x6a, 1, {0x10}},
{0x6b, 1, {0x07}},
{0x6c, 1, {0x0f}},
{0x6d, 1, {0x0b}},
{0x6e, 1, {0x01}},
{0x6f, 1, {0x00}},
{0x70, 1, {0x00}},
{0x71, 1, {0x06}},
{0x72, 1, {0x0a}},
{0x73, 1, {0x0c}},
{0x74, 1, {0x04}},
{0x75, 1, {0x10}},
{0x76, 1, {0x0e}},
{0x77, 1, {0x0a}},
{0x78, 1, {0x17}},
{0x79, 1, {0x0c}},
{0x7a, 1, {0x10}},
{0x7b, 1, {0x07}},
{0x7c, 1, {0x0f}},
{0x7d, 1, {0x0b}},
{0x7e, 1, {0x01}},
{0x7f, 1, {0x00}},
{0xfe, 1, {0x00}},
{0x58, 1, {0xa9}},
{0x35, 0, {}},
{0x11, 1, {0x00}},
		{REGFLAG_DELAY, 120, {}},
{0x29, 1, {}},
		{REGFLAG_DELAY, 20, {}},

		{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},

	{0x4f, 0, {0x01}},
	{REGFLAG_DELAY, 150, {}},
	

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count,
		unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {

		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
					table[i].para_list, force_update);
		}
	}

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

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;//LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode   = SYNC_EVENT_VDO_MODE;//SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;////
#endif

	// DSI
	/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	
	
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	
#if (LCM_DSI_CMD_MODE)
	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
	params->dsi.word_count=FRAME_WIDTH*3;	//DSI CMD mode need set these two bellow params, different to 6577
#else
	params->dsi.intermediat_buffer_num = 0;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
#endif

	// Video mode setting
	params->dsi.packet_size=256;

	params->dsi.vertical_sync_active				=  2;//2
	params->dsi.vertical_backporch					= 14;//50;
	params->dsi.vertical_frontporch					= 16;//50;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 8;//10
	params->dsi.horizontal_backporch				= 80;//34; 
	params->dsi.horizontal_frontporch				= 80;//24;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	// Bit rate calculation
//params->dsi.ssc_disable = 1;
	//1 Every lane speed
params->dsi.PLL_CLOCK=210;//195

//ESD
    params->dsi.esd_check_enable            = 1;
    params->dsi.customization_esd_check_enable  = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

    params->dsi.noncont_clock = 1;
    params->dsi.noncont_clock_period = 1;

}

static void lcm_init(void)
{

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting,sizeof(lcm_initialization_setting) /sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	unsigned int array[16];
	array[0] = 0x00FE1500;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(50);
	array[0] = 0x00011500;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(50);
	array[0] = 0x00280500;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(50);
	array[0] = 0x00100500;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(50);
	array[0] = 0x014F1500;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(50);
}


static void lcm_resume(void)
{   
	lcm_init();
}


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
	data_array[3] = 0x00053902;
	data_array[4] =
		(y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[5] = (y1_LSB);
	data_array[6] = 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}


static unsigned int lcm_compare_id(void)
{
		int array[4];
		char buffer[5];
		char id_high=0;
		char id_low=0;
		int id1=0;
		int id2=0;

		SET_RESET_PIN(1);
		MDELAY(10);
		SET_RESET_PIN(0);
		MDELAY(10);
		SET_RESET_PIN(1);
		MDELAY(200);
		array[0]=0x01FE1500;
		dsi_set_cmdq(array,1, 1);

		array[0] = 0x00013700;
		dsi_set_cmdq(array, 1, 1);
		read_reg_v2(0xde, buffer, 1);

		id_high = buffer[0];
		read_reg_v2(0xdf, buffer, 1);
		id_low = buffer[0];
		id1 = (id_high<<8) | id_low;

		#if defined(BUILD_LK)
		printf("rm68200a %s id1 = 0x%04x, id2 = 0x%04x\n", __func__, id1,id2);
		#else
		printk("rm68200a %s id1 = 0x%04x, id2 = 0x%04x\n", __func__, id1,id2);
		#endif
		return (LCM_RM68200_ID == id1)?1:0;

}
//no use
static unsigned int lcm_esd_recover(void)
{
    unsigned char para = 0;
	unsigned int data_array1[16];

#ifndef BUILD_LK
    printk("RM68190 lcm_esd_recover enter\n");
#endif
    

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(130);
    #if 0
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    	MDELAY(10);
    #else
        lcm_init();
    #endif
   
   data_array1[0]= 0x00320500;
   dsi_set_cmdq(data_array1, 1, 1);
   MDELAY(50);

    return 1;
}
static unsigned int lcm_esd_check(void)
{
    unsigned char buffer[1] ={0};
    //unsigned int data_array[1];
   // data_array[0] = 0x00013700;// read id return two byte,version and id 3 byte 
  // dsi_set_cmdq(&data_array, 1, 1);
   read_reg_v2(0x0a, buffer, 1);
   
#ifndef BUILD_LK
    printk("RM68190 lcm_esd_check enter %x\n",buffer[0]);
#endif
#ifndef BUILD_LK
        if(buffer[0] == 0x9C)
        {
          #ifndef BUILD_LK
          printk("RM68190 lcm_esd_check false \n");
          #endif

            return false;
        }
        else
        {      
           #ifndef BUILD_LK
          printk("RM68190 lcm_esd_check true \n");
          #endif
           //lcm_esd_recover();
            return true;
        }
#endif
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_rm68200_dsi_vdo_hd_cpt = 
{
	.name			= "hct_rm68200_dsi_vdo_hd_cpt",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,	
	.compare_id     = lcm_compare_id,	
    .esd_check   	= lcm_esd_check,	
    .esd_recover   	= lcm_esd_recover,	
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif	//wqtao
};

