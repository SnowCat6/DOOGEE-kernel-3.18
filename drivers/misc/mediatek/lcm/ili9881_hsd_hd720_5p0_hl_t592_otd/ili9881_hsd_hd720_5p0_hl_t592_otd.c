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

#if defined(BUILD_LK)
#else

#include <linux/proc_fs.h>   //proc file use 
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                         (720)
#define FRAME_HEIGHT                                        (1280)
#define LCM_ID                       (0x1283)

#define REGFLAG_DELAY               (0XFE)
#define REGFLAG_END_OF_TABLE        (0x100) // END OF REGISTERS MARKER


#define LCM_DSI_CMD_MODE                                    0

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

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    // Display ON
    //{0x2C, 1, {0x00}},
    //{0x13, 1, {0x00}},
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
    // Display off sequence
    {0xf0, 5, {0x55, 0xaa, 0x52, 0x08, 0x01}},
    {REGFLAG_DELAY, 10, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

//static int vcom=0x40;
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {
        
        unsigned cmd;
        cmd = table[i].cmd;
        
        switch (cmd) {
			/*case 0xd9:
			table[i].para_list[0]=vcom;
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
            vcom+=2;
			break;
			*/
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

    // enable tearing-free
    params->dbi.te_mode             = LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity    = LCM_POLARITY_RISING;

    params->dsi.mode   = SYNC_EVENT_VDO_MODE;
params->dsi.ssc_disable = 1;
  params->dsi.clk_lp_per_line_enable = 1;
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                = LCM_THREE_LANE;
    //The following defined the fomat for data coming from LCD engine. 
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST; 
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    params->dsi.packet_size=256;

    // Video mode setting       
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.vertical_sync_active                = 8;
    params->dsi.vertical_backporch                  = 24;
    params->dsi.vertical_frontporch                 = 16;
    params->dsi.vertical_active_line                = FRAME_HEIGHT; 
    params->dsi.horizontal_sync_active              = 10;
    params->dsi.horizontal_backporch                = 60;
    params->dsi.horizontal_frontporch               = 60;
    params->dsi.horizontal_active_pixel            = FRAME_WIDTH;
    // Bit rate calculation
  params->dsi.HS_TRAIL = 20;

  params->dsi.esd_check_enable = 1;
  params->dsi.customization_esd_check_enable = 1;
  params->dsi.lcm_esd_check_table[0].count = 1;
  params->dsi.lcm_esd_check_table[0].cmd = 10;
  params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9Cu;

    params->dsi.PLL_CLOCK=260;

/*
  params->dsi.LANE_NUM = 3;
  params->dsi.packet_size = 256;
  params->dsi.vertical_sync_active = 8;
  params->dsi.vertical_backporch = 24;
  params->dsi.vertical_frontporch = 16;
  params->dsi.HS_TRAIL = 20;
  params->dsi.PLL_CLOCK = 260;
  params->dsi.lcm_esd_check_table[0].para_list[0] = -100;
  params->type = 2;
  params->dsi.mode = 2;
  params->dsi.data_format.format = 2;
  params->dsi.PS = 2;
  params->width = 720;
  params->dsi.horizontal_active_pixel = 720;
  params->height = 1280;
  params->dsi.vertical_active_line = 1280;
  params->dbi.te_mode = 1;
  params->dsi.ssc_disable = 1;
  params->dsi.clk_lp_per_line_enable = 1;
  params->dsi.esd_check_enable = 1;
  params->dsi.customization_esd_check_enable = 1;
  params->dsi.lcm_esd_check_table[0].count = 1;
  params->dbi.te_edge_polarity = 0;
  params->dsi.data_format.color_order = 0;
  params->dsi.data_format.trans_seq = 0;
  params->dsi.data_format.padding = 0;
  params->dsi.horizontal_sync_active = 10;
  params->dsi.lcm_esd_check_table[0].cmd = 10;
  params->dsi.horizontal_backporch = 60;
  params->dsi.horizontal_frontporch = 60;
*/
}

static void lcm_set_reg(int dsi_cmd, int dsi_value)
{
    unsigned int data_array[2];
    data_array[0] = 0x22902;
    data_array[1] = dsi_cmd | (dsi_value << 8);
    dsi_set_cmdq(data_array, 2, 1);
}

static void lcm_set(int dsi_cmd)
{
    unsigned int data_array[1];
    data_array[0] =  (dsi_cmd << 16) | 0x500;
    dsi_set_cmdq(data_array, 1, 1);
}

static void lcm_init(void)
{
    unsigned int data_array[16];

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);

  data_array[0] = 0x42902;
  data_array[1] = 0x38198FF;
  dsi_set_cmdq(data_array, 2, 1);
  lcm_set_reg(1, 0);
  lcm_set_reg(2, 0);
  lcm_set_reg(3, 115);
  lcm_set_reg(4, 0);
  lcm_set_reg(5, 0);
  lcm_set_reg(6, 10);
  lcm_set_reg(7, 0);
  lcm_set_reg(8, 0);
  lcm_set_reg(9, 1);
  lcm_set_reg(10, 0);
  lcm_set_reg(11, 0);
  lcm_set_reg(12, 1);
  lcm_set_reg(13, 0);
  lcm_set_reg(14, 0);
  lcm_set_reg(15, 29);
  lcm_set_reg(16, 29);
  lcm_set_reg(17, 0);
  lcm_set_reg(18, 0);
  lcm_set_reg(19, 0);
  lcm_set_reg(20, 0);
  lcm_set_reg(21, 0);
  lcm_set_reg(22, 0);
  lcm_set_reg(23, 0);
  lcm_set_reg(24, 0);
  lcm_set_reg(25, 0);
  lcm_set_reg(26, 0);
  lcm_set_reg(27, 0);
  lcm_set_reg(28, 0);
  lcm_set_reg(29, 0);
  lcm_set_reg(30, 64);
  lcm_set_reg(31, 128);
  lcm_set_reg(32, 6);
  lcm_set_reg(33, 2);
  lcm_set_reg(34, 0);
  lcm_set_reg(35, 0);
  lcm_set_reg(36, 0);
  lcm_set_reg(37, 0);
  lcm_set_reg(38, 0);
  lcm_set_reg(39, 0);
  lcm_set_reg(40, 51);
  lcm_set_reg(41, 3);
  lcm_set_reg(42, 0);
  lcm_set_reg(43, 0);
  lcm_set_reg(44, 0);
  lcm_set_reg(45, 0);
  lcm_set_reg(46, 0);
  lcm_set_reg(47, 0);
  lcm_set_reg(48, 0);
  lcm_set_reg(49, 0);
  lcm_set_reg(50, 0);
  lcm_set_reg(51, 0);
  lcm_set_reg(52, 4);
  lcm_set_reg(53, 0);
  lcm_set_reg(54, 0);
  lcm_set_reg(55, 0);
  lcm_set_reg(56, 60);
  lcm_set_reg(57, 0);
  lcm_set_reg(58, 64);
  lcm_set_reg(59, 64);
  lcm_set_reg(60, 0);
  lcm_set_reg(61, 0);
  lcm_set_reg(62, 0);
  lcm_set_reg(63, 0);
  lcm_set_reg(64, 0);
  lcm_set_reg(65, 0);
  lcm_set_reg(66, 0);
  lcm_set_reg(67, 0);
  lcm_set_reg(68, 0);
  lcm_set_reg(80, 1);
  lcm_set_reg(81, 35);
  lcm_set_reg(82, 69);
  lcm_set_reg(83, 103);
  lcm_set_reg(84, 137);
  lcm_set_reg(85, 171);
  lcm_set_reg(86, 1);
  lcm_set_reg(87, 35);
  lcm_set_reg(88, 69);
  lcm_set_reg(89, 103);
  lcm_set_reg(90, 137);
  lcm_set_reg(91, 171);
  lcm_set_reg(92, 205);
  lcm_set_reg(93, 239);
  lcm_set_reg(94, 17);
  lcm_set_reg(95, 1);
  lcm_set_reg(96, 0);
  lcm_set_reg(97, 21);
  lcm_set_reg(98, 20);
  lcm_set_reg(99, 14);
  lcm_set_reg(100, 15);
  lcm_set_reg(101, 12);
  lcm_set_reg(102, 13);
  lcm_set_reg(103, 6);
  lcm_set_reg(104, 2);
  lcm_set_reg(105, 7);
  lcm_set_reg(106, 2);
  lcm_set_reg(107, 2);
  lcm_set_reg(108, 2);
  lcm_set_reg(109, 2);
  lcm_set_reg(110, 2);
  lcm_set_reg(111, 2);
  lcm_set_reg(112, 2);
  lcm_set_reg(113, 2);
  lcm_set_reg(114, 2);
  lcm_set_reg(115, 2);
  lcm_set_reg(116, 2);
  lcm_set_reg(117, 1);
  lcm_set_reg(118, 0);
  lcm_set_reg(119, 20);
  lcm_set_reg(120, 21);
  lcm_set_reg(121, 14);
  lcm_set_reg(122, 15);
  lcm_set_reg(123, 12);
  lcm_set_reg(124, 13);
  lcm_set_reg(125, 6);
  lcm_set_reg(126, 2);
  lcm_set_reg(127, 7);
  lcm_set_reg(128, 2);
  lcm_set_reg(129, 2);
  lcm_set_reg(130, 2);
  lcm_set_reg(131, 2);
  lcm_set_reg(132, 2);
  lcm_set_reg(133, 2);
  lcm_set_reg(134, 2);
  lcm_set_reg(135, 2);
  lcm_set_reg(136, 2);
  lcm_set_reg(137, 2);
  lcm_set_reg(138, 2);
  data_array[1] = 75602175;
  data_array[0] = 272642;
  dsi_set_cmdq(data_array, 2, 1);
  lcm_set_reg(0, 0);
  lcm_set_reg(108, 21);
  lcm_set_reg(110, 42);
  lcm_set_reg(111, 53);
  lcm_set_reg(58, 148);
  lcm_set_reg(141, 20);
  lcm_set_reg(135, 186);
  lcm_set_reg(38, 118);
  lcm_set_reg(178, 209);
  lcm_set_reg(181, 6);
  data_array[0] = 272642;
  data_array[1] = 25270527;
  dsi_set_cmdq(data_array, 2, 1);
  lcm_set_reg(34, 9);
  lcm_set_reg(49, 0);
  lcm_set_reg(83, 150);
  lcm_set_reg(85, 150);
  lcm_set_reg(80, 183);
  lcm_set_reg(81, 183);
  lcm_set_reg(96, 34);
  lcm_set_reg(97, 0);
  lcm_set_reg(98, 25);
  lcm_set_reg(99, 16);
  lcm_set_reg(160, 8);
  lcm_set_reg(161, 26);
  lcm_set_reg(162, 39);
  lcm_set_reg(163, 21);
  lcm_set_reg(164, 23);
  lcm_set_reg(165, 42);
  lcm_set_reg(166, 30);
  lcm_set_reg(167, 31);
  lcm_set_reg(168, 139);
  lcm_set_reg(169, 27);
  lcm_set_reg(170, 39);
  lcm_set_reg(171, 120);
  lcm_set_reg(172, 24);
  lcm_set_reg(173, 24);
  lcm_set_reg(174, 76);
  lcm_set_reg(175, 33);
  lcm_set_reg(176, 39);
  lcm_set_reg(177, 84);
  lcm_set_reg(178, 103);
  lcm_set_reg(179, 57);
  lcm_set_reg(192, 8);
  lcm_set_reg(193, 26);
  lcm_set_reg(194, 39);
  lcm_set_reg(195, 21);
  lcm_set_reg(196, 23);
  lcm_set_reg(197, 42);
  lcm_set_reg(198, 30);
  lcm_set_reg(199, 31);
  lcm_set_reg(200, 139);
  lcm_set_reg(201, 27);
  lcm_set_reg(202, 39);
  lcm_set_reg(203, 120);
  lcm_set_reg(204, 24);
  lcm_set_reg(205, 24);
  lcm_set_reg(206, 76);
  lcm_set_reg(207, 33);
  lcm_set_reg(208, 39);
  lcm_set_reg(209, 84);
  lcm_set_reg(210, 103);
  lcm_set_reg(211, 57);
  data_array[1] = 25270527;
  data_array[0] = 272642;
  dsi_set_cmdq(data_array, 2, 1);
  lcm_set_reg(96, 34);
  data_array[1] = 8493311;
  data_array[0] = 272642;
  dsi_set_cmdq(data_array, 2, 1);
  lcm_set_reg(54, 3);

  lcm_set(0x11);
  MDELAY(120);
  lcm_set(0x29);
  MDELAY(60);

#ifdef BUILD_LK
    printf("[erick-lk]%s\n", __func__);
#else
    printk("[erick-k]%s\n", __func__);
#endif
}


static void lcm_suspend(void)
{
#ifndef BUILD_LK
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);   //wqtao. enable
    #ifdef BUILD_LK
        printf("[erick-lk]%s\n", __func__);
    #else
        printk("[erick-k]%s\n", __func__);
    #endif
#endif
}


static void lcm_resume(void)
{
#ifndef BUILD_LK
    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    #ifdef BUILD_LK
        printf("[erick-lk]%s\n", __func__);
    #else
        printk("[erick-k]%s\n", __func__);
    #endif
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

#if 0   //wqtao.        
static void lcm_setbacklight(unsigned int level)
{
    unsigned int default_level = 145;
    unsigned int mapped_level = 0;

    //for LGE backlight IC mapping table
    if(level > 255) 
            level = 255;

    if(level >0) 
            mapped_level = default_level+(level)*(255-default_level)/(255);
    else
            mapped_level=0;

    // Refresh value of backlight level.
    lcm_backlight_level_setting[0].para_list[0] = mapped_level;

    push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}
#endif

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
    if(lcm_esd_test)
    {
        lcm_esd_test = FALSE;
        return TRUE;
    }

    /// please notice: the max return packet size is 1
    /// if you want to change it, you can refer to the following marked code
    /// but read_reg currently only support read no more than 4 bytes....
    /// if you need to read more, please let BinHan knows.
    /*
            unsigned int data_array[16];
            unsigned int max_return_size = 1;
            
            data_array[0]= 0x00003700 | (max_return_size << 16);    
            
            dsi_set_cmdq(&data_array, 1, 1);
    */

    if(read_reg(0x0a) == 0x9c)
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

    return TRUE;
}


static unsigned int lcm_compare_id(void)
{
        int array[4];
        char buffer[5];
        char id_high=0;
        char id_midd=0;
        char id_low=0;
        int id=0;
        //Do reset here
        SET_RESET_PIN(1);
        SET_RESET_PIN(0);
        MDELAY(25);       
        SET_RESET_PIN(1);
        MDELAY(50);      
       
        array[0]=0x00043902;
        array[1]=0x018198FF;
        dsi_set_cmdq(array, 3, 1);
        MDELAY(10);
        array[0]=0x00023700;
        dsi_set_cmdq(array, 1, 1);
        //read_reg_v2(0x04, buffer, 3);
    
        read_reg_v2(0x00, buffer,1);
        id_high = buffer[0]; ///////////////////////0x98
        read_reg_v2(0x01, buffer,1);
        id_midd = buffer[1]; ///////////////////////0x81
        read_reg_v2(0x02, buffer,1);
        id_low = buffer[2]; ////////////////////////0x00
       // id = (id_midd &lt;&lt; 8) | id_low;

#ifdef BUILD_LK
    printf("[erick-lk]%s,  9881 id = 0x%08x,0x%08x\n", __func__, id_high,id_midd);
#else
    printk("[erick-k]%s,  9881 id = 0x%08x,0x%08x\n", __func__, id_high,id_midd);
#endif
	if((0x98 == id_high)&&(0x81 == id_midd))
	{
		return 1;
	}
	else
	{
		return 0;
	}
    //return (0x98 == id_high)?1:0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9881_hsd_hd720_5p0_hl_t592_otd = 
{
    .name           = "ili9881_hsd_hd720_5p0_hl_t592_otd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,   
    .compare_id    = lcm_compare_id,    
#if (LCM_DSI_CMD_MODE)
    //.set_backlight    = lcm_setbacklight,
    //.esd_check   = lcm_esd_check, 
    //.esd_recover   = lcm_esd_recover, 
    .update         = lcm_update,
#endif  //wqtao
};

