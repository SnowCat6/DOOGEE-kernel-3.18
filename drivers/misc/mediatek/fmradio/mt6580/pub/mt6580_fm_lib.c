#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "osal_typedef.h"
#include "stp_exp.h"
#include "wmt_exp.h"

#include "fm_typedef.h"
#include "fm_dbg.h"
#include "fm_err.h"
#include "fm_interface.h"
#include "fm_stdlib.h"
#include "fm_patch.h"
#include "fm_utils.h"
#include "fm_link.h"
#include "fm_config.h"
#include "fm_private.h"

#include "mt6580_fm_reg.h"
#include "mt6580_fm.h"
#include "mt6580_fm_lib.h"
#include "mt6580_fm_cmd.h"
#include "mt6580_fm_cust_cfg.h"
/* #include "mach/mt_gpio.h" */

static struct fm_patch_tbl mt6580_patch_tbl[5] = {
	{FM_ROM_V1, "/etc/firmware/mt6580/mt6580_fm_v1_patch.bin",
	 "/etc/firmware/mt6580/mt6580_fm_v1_coeff.bin", NULL, NULL},
	{FM_ROM_V2, "/etc/firmware/mt6580/mt6580_fm_v2_patch.bin",
	 "/etc/firmware/mt6580/mt6580_fm_v2_coeff.bin", NULL, NULL},
	{FM_ROM_V3, "/etc/firmware/mt6580/mt6580_fm_v3_patch.bin",
	 "/etc/firmware/mt6580/mt6580_fm_v3_coeff.bin", NULL, NULL},
	{FM_ROM_V4, "/etc/firmware/mt6580/mt6580_fm_v4_patch.bin",
	 "/etc/firmware/mt6580/mt6580_fm_v4_coeff.bin", NULL, NULL},
	{FM_ROM_V5, "/etc/firmware/mt6580/mt6580_fm_v5_patch.bin",
	 "/etc/firmware/mt6580/mt6580_fm_v5_coeff.bin", NULL, NULL},
};

static struct fm_hw_info mt6580_hw_info = {
	.chip_id = 0x00006580,
	.eco_ver = 0x00000000,
	.rom_ver = 0x00000000,
	.patch_ver = 0x00000000,
	.reserve = 0x00000000,
};

#define PATCH_SEG_LEN 512

static fm_u8 *cmd_buf;
static struct fm_lock *cmd_buf_lock;
static struct fm_callback *fm_cb_op;
static struct fm_res_ctx *mt6580_res;
/* static fm_s32 Chip_Version = mt6580_E1; */

/* static fm_bool rssi_th_set = fm_false; */

#if 0				/* def CONFIG_MTK_FM_50KHZ_SUPPORT */
static struct fm_fifo *cqi_fifo;
#endif
static fm_s32 mt6580_is_dese_chan(fm_u16 freq);

#if 0
static fm_s32 mt6580_mcu_dese(fm_u16 freq, void *arg);
static fm_s32 mt6580_gps_dese(fm_u16 freq, void *arg);
static fm_s32 mt6580_I2s_Setting(fm_s32 onoff, fm_s32 mode, fm_s32 sample);
#endif
static fm_u16 mt6580_chan_para_get(fm_u16 freq);
static fm_s32 mt6580_desense_check(fm_u16 freq, fm_s32 rssi);
/*static fm_bool mt6580_TDD_chan_check(fm_u16 freq);*/
static fm_s32 mt6580_soft_mute_tune(fm_u16 freq, fm_s32 *rssi, fm_bool *valid);
static fm_s32 mt6580_pwron(fm_s32 data)
{
	/*//Turn on FM on 6627 chip by WMT driver
	   if(MTK_WCN_BOOL_FALSE == mtk_wcn_wmt_func_on(WMTDRV_TYPE_LPBK)){
	   WCN_DBG(FM_ALT|CHIP,"WMT turn on LPBK Fail!\n");
	   return -FM_ELINK;
	   }else{
	   WCN_DBG(FM_ALT|CHIP,"WMT turn on LPBK OK!\n");
	   //return 0;
	   } */
	if (MTK_WCN_BOOL_FALSE == mtk_wcn_wmt_func_on(WMTDRV_TYPE_FM)) {
		WCN_DBG(FM_ALT | CHIP, "WMT turn on FM Fail!\n");
		return -FM_ELINK;
	}

	WCN_DBG(FM_ALT | CHIP, "WMT turn on FM OK!\n");
	return 0;
}

static fm_s32 mt6580_pwroff(fm_s32 data)
{
	if (MTK_WCN_BOOL_FALSE == mtk_wcn_wmt_func_off(WMTDRV_TYPE_FM)) {
		WCN_DBG(FM_ALT | CHIP, "WMT turn off FM Fail!\n");
		return -FM_ELINK;
	}

	WCN_DBG(FM_NTC | CHIP, "WMT turn off FM OK!\n");
	return 0;
}

static fm_s32 Delayms(fm_u32 data)
{
	WCN_DBG(FM_DBG | CHIP, "delay %dms\n", data);
	msleep(data);
	return 0;
}

static fm_s32 Delayus(fm_u32 data)
{
	WCN_DBG(FM_DBG | CHIP, "delay %dus\n", data);
	udelay(data);
	return 0;
}

fm_s32 mt6580_get_read_result(struct fm_res_ctx *result)
{
	if (result == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}
	mt6580_res = result;

	return 0;
}

static fm_s32 mt6580_read(fm_u8 addr, fm_u16 *val)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;

	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_get_reg(cmd_buf, TX_BUF_SIZE, addr);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_FSPI_RD, SW_RETRY_CNT, FSPI_RD_TIMEOUT, mt6580_get_read_result);

	if (!ret && mt6580_res)
		*val = mt6580_res->fspi_rd;

	FM_UNLOCK(cmd_buf_lock);

	return ret;
}

static fm_s32 mt6580_write(fm_u8 addr, fm_u16 val)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;

	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_set_reg(cmd_buf, TX_BUF_SIZE, addr, val);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_FSPI_WR, SW_RETRY_CNT, FSPI_WR_TIMEOUT, NULL);
	FM_UNLOCK(cmd_buf_lock);

	return ret;
}

static fm_s32 mt6580_set_bits(fm_u8 addr, fm_u16 bits, fm_u16 mask)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;

	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_set_bits_reg(cmd_buf, TX_BUF_SIZE, addr, bits, mask);
	ret = fm_cmd_tx(cmd_buf, pkt_size, (1 << 0x11), SW_RETRY_CNT, FSPI_WR_TIMEOUT, NULL);
	/* 0x11 this opcode won't be parsed as an opcode, so set here as spcial case. */
	FM_UNLOCK(cmd_buf_lock);

	return ret;
}

static fm_s32 mt6580_top_read(fm_u16 addr, fm_u32 *val)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;

	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_top_get_reg(cmd_buf, TX_BUF_SIZE, addr);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_CSPI_READ, SW_RETRY_CNT, FSPI_RD_TIMEOUT, mt6580_get_read_result);

	if (!ret && mt6580_res)
		*val = mt6580_res->cspi_rd;

	FM_UNLOCK(cmd_buf_lock);

	return ret;
}

static fm_s32 mt6580_top_write(fm_u16 addr, fm_u32 val)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;

	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_top_set_reg(cmd_buf, TX_BUF_SIZE, addr, val);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_CSPI_WRITE, SW_RETRY_CNT, FSPI_WR_TIMEOUT, NULL);
	FM_UNLOCK(cmd_buf_lock);

	return ret;
}

/*static fm_s32 mt6580_top_set_bits(fm_u16 addr, fm_u32 bits, fm_u32 mask)
{
    fm_s32 ret = 0;
    fm_u32 val;

    ret = mt6580_top_read(addr, &val);

    if (ret)
	return ret;

    val = ((val & (mask)) | bits);
    ret = mt6580_top_write(addr, val);

    return ret;
}*/

static fm_s32 mt6580_host_read(fm_u32 addr, fm_u32 *val)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;

	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_host_get_reg(cmd_buf, TX_BUF_SIZE, addr);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_HOST_READ, SW_RETRY_CNT, FSPI_RD_TIMEOUT, mt6580_get_read_result);

	if (!ret && mt6580_res)
		*val = mt6580_res->cspi_rd;

	FM_UNLOCK(cmd_buf_lock);

	return ret;
}

static fm_s32 mt6580_host_write(fm_u32 addr, fm_u32 val)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;

	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_host_set_reg(cmd_buf, TX_BUF_SIZE, addr, val);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_HOST_WRITE, SW_RETRY_CNT, FSPI_WR_TIMEOUT, NULL);
	FM_UNLOCK(cmd_buf_lock);

	return ret;
}

/*static fm_s32 mt6580_DSP_write(fm_u16 addr, fm_u16 val)
{
    mt6580_write(0xE2, addr);
    mt6580_write(0xE3, val);
    mt6580_write(0xE1, 0x0002);
    return 0;
}
static fm_s32 mt6580_DSP_read(fm_u16 addr, fm_u16 *val)
{
    fm_s32 ret=-1;
    mt6580_write(0xE2, addr);
    mt6580_write(0xE1, 0x0001);
    ret = mt6580_read(0xE4, val);
    return ret;
}*/
static fm_u16 mt6580_get_chipid(void)
{
	return 0x6627;
}

/*  MT6580_SetAntennaType - set Antenna type
 *  @type - 1,Short Antenna;  0, Long Antenna
 */
static fm_s32 mt6580_SetAntennaType(fm_s32 type)
{
	fm_u16 dataRead;

	WCN_DBG(FM_DBG | CHIP, "set ana to %s\n", type ? "short" : "long");
	mt6580_read(FM_MAIN_CG2_CTRL, &dataRead);

	if (type)
		dataRead |= ANTENNA_TYPE;
	else
		dataRead &= (~ANTENNA_TYPE);

	mt6580_write(FM_MAIN_CG2_CTRL, dataRead);

	return 0;
}

static fm_s32 mt6580_GetAntennaType(void)
{
	fm_u16 dataRead;

	mt6580_read(FM_MAIN_CG2_CTRL, &dataRead);
	WCN_DBG(FM_DBG | CHIP, "get ana type: %s\n", (dataRead & ANTENNA_TYPE) ? "short" : "long");

	if (dataRead & ANTENNA_TYPE)
		return FM_ANA_SHORT;	/* short antenna */
	else
		return FM_ANA_LONG;	/* long antenna */
}

static fm_s32 mt6580_Mute(fm_bool mute)
{
	fm_s32 ret = 0;
	fm_u16 dataRead;

	WCN_DBG(FM_DBG | CHIP, "set %s\n", mute ? "mute" : "unmute");
	/* mt6580_read(FM_MAIN_CTRL, &dataRead); */
	mt6580_read(0x9C, &dataRead);

	/* mt6580_top_write(0x0050,0x00000007); */

	if (mute == 1)
		ret = mt6580_write(0x9C, (dataRead & 0xFFFC) | 0x0003);
	else
		ret = mt6580_write(0x9C, (dataRead & 0xFFFC));

	/* mt6580_top_write(0x0050,0x0000000F); */

	return ret;
}

/*static fm_s32 mt6580_set_RSSITh(fm_u16 TH_long, fm_u16 TH_short)
{
    mt6580_write(0xE2, 0x3072);
    mt6580_write(0xE3, TH_long);
    mt6580_write(0xE1, 0x0002);
    Delayms(1);
    mt6580_write(0xE2, 0x307A);
    mt6580_write(0xE3, TH_short);
    mt6580_write(0xE1, 0x0002);

    WCN_DBG(FM_DBG | CHIP, "RSSI TH, long:0x%04x, short:0x%04x", TH_long, TH_short);
    return 0;
}
*/
/*
static fm_s32 mt6580_set_SMGTh(fm_s32 ver, fm_u16 TH_smg)
{
    if (mt6580_E1 == ver) {
	mt6580_write(0xE2, 0x321E);
	mt6580_write(0xE3, TH_smg);
	mt6580_write(0xE1, 0x0002);
    } else {
	mt6580_write(0xE2, 0x3218);
	mt6580_write(0xE3, TH_smg);
	mt6580_write(0xE1, 0x0002);
    }

    WCN_DBG(FM_DBG | CHIP, "Soft-mute gain TH %d\n", (int)TH_smg);
    return 0;
}
*/
static fm_s32 mt6580_RampDown(void)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;

	WCN_DBG(FM_DBG | CHIP, "ramp down\n");
	ret = mt6580_write(0x60, 0x00000007);
	if (ret) {
		WCN_DBG(FM_ERR | CHIP, "ramp down wr 0x60 failed\n");
		return ret;
	}
	ret = mt6580_write(0x60, 0x0000000f);
	if (ret) {
		WCN_DBG(FM_ERR | CHIP, "ramp down wr 0x60 failed\n");
		return ret;
	}

	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_rampdown(cmd_buf, TX_BUF_SIZE);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_RAMPDOWN, SW_RETRY_CNT, RAMPDOWN_TIMEOUT, NULL);
	FM_UNLOCK(cmd_buf_lock);
	if (ret) {
		WCN_DBG(FM_ERR | CHIP, "ramp down failed\n");
		return ret;
	}

	return ret;
}

static fm_s32 mt6580_get_rom_version(void)
{
	fm_u16 tmp;
	fm_s32 ret;

	mt6580_write(0x90, 0xe);
	mt6580_write(0x92, 0x0);
	mt6580_write(0x90, 0x40);
	mt6580_write(0x90, 0x0);

	/* DSP rom code version request enable --- set 0x61 b15=1 */
	mt6580_set_bits(0x61, 0x8000, 0x7FFF);

	/* Release ASIP reset --- set 0x61 b1=1 */
	mt6580_set_bits(0x61, 0x0002, 0xFFFD);

	/* Enable ASIP power --- set 0x61 b0=0 */
	mt6580_set_bits(0x61, 0x0000, 0xFFFE);

	/* Wait DSP code version ready --- wait 1ms */
	do {
		Delayus(1000);
		ret = mt6580_read(0x84, &tmp);
		/* ret=-4 means signal got when control FM. usually get sig 9 to kill FM process. */
		/* now cancel FM power up sequence is recommended. */
		if (ret)
			return ret;

		WCN_DBG(FM_NTC | CHIP, "0x84=%x\n", tmp);
	} while (tmp != 0x0001);

	/* Get FM DSP code version --- rd 0x83[15:8] */
	mt6580_read(0x83, &tmp);
	tmp = (tmp >> 8);

	/* DSP rom code version request disable --- set 0x61 b15=0 */
	mt6580_set_bits(0x61, 0x0000, 0x7FFF);

	/* Reset ASIP --- set 0x61[1:0] = 1 */
	mt6580_set_bits(0x61, 0x0001, 0xFFFC);

	/* WCN_DBG(FM_NTC | CHIP, "ROM version: v%d\n", (fm_s32)tmp); */
	return (fm_s32) tmp;
}

static fm_s32 mt6580_get_patch_path(fm_s32 ver, const fm_s8 **ppath)
{
	fm_s32 i;
	fm_s32 max = sizeof(mt6580_patch_tbl) / sizeof(mt6580_patch_tbl[0]);

	/* check if the ROM version is defined or not */
	for (i = 0; i < max; i++) {
		if ((mt6580_patch_tbl[i].idx == ver)
		    && (fm_file_exist(mt6580_patch_tbl[i].patch) == 0)) {
			*ppath = mt6580_patch_tbl[i].patch;
			WCN_DBG(FM_NTC | CHIP, "Get ROM version OK\n");
			return 0;
		}
	}

	/* the ROM version isn't defined, find a latest patch instead */
	for (i = max; i > 0; i--) {
		if (fm_file_exist(mt6580_patch_tbl[i - 1].patch) == 0) {
			*ppath = mt6580_patch_tbl[i - 1].patch;
			WCN_DBG(FM_WAR | CHIP, "undefined ROM version\n");
			return 1;
		}
	}

	/* get path failed */
	WCN_DBG(FM_ERR | CHIP, "No valid patch file\n");
	return -FM_EPATCH;
}

static fm_s32 mt6580_get_coeff_path(fm_s32 ver, const fm_s8 **ppath)
{
	fm_s32 i;
	fm_s32 max = sizeof(mt6580_patch_tbl) / sizeof(mt6580_patch_tbl[0]);

	/* check if the ROM version is defined or not */
	for (i = 0; i < max; i++) {
		if ((mt6580_patch_tbl[i].idx == ver)
		    && (fm_file_exist(mt6580_patch_tbl[i].coeff) == 0)) {
			*ppath = mt6580_patch_tbl[i].coeff;
			WCN_DBG(FM_NTC | CHIP, "Get ROM version OK\n");
			return 0;
		}
	}

	/* the ROM version isn't defined, find a latest patch instead */
	for (i = max; i > 0; i--) {
		if (fm_file_exist(mt6580_patch_tbl[i - 1].coeff) == 0) {
			*ppath = mt6580_patch_tbl[i - 1].coeff;
			WCN_DBG(FM_WAR | CHIP, "undefined ROM version\n");
			return 1;
		}
	}

	/* get path failed */
	WCN_DBG(FM_ERR | CHIP, "No valid coeff file\n");
	return -FM_EPATCH;
}

/*
*  mt6580_DspPatch - DSP download procedure
*  @img - source dsp bin code
*  @len - patch length in byte
*  @type - rom/patch/coefficient/hw_coefficient
*/
static fm_s32 mt6580_DspPatch(const fm_u8 *img, fm_s32 len, enum IMG_TYPE type)
{
	fm_u8 seg_num;
	fm_u8 seg_id = 0;
	fm_s32 seg_len;
	fm_s32 ret = 0;
	fm_u16 pkt_size;

	if (img == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}

	if (len <= 0)
		return -1;

	seg_num = len / PATCH_SEG_LEN + 1;
	WCN_DBG(FM_NTC | CHIP, "binary len:%d, seg num:%d\n", len, seg_num);

	switch (type) {
#if 0
	case IMG_ROM:

		for (seg_id = 0; seg_id < seg_num; seg_id++) {
			seg_len = ((seg_id + 1) < seg_num) ? PATCH_SEG_LEN : (len % PATCH_SEG_LEN);
			WCN_DBG(FM_NTC | CHIP, "rom,[seg_id:%d],  [seg_len:%d]\n", seg_id, seg_len);
			if (FM_LOCK(cmd_buf_lock))
				return -FM_ELOCK;
			pkt_size =
			    mt6580_rom_download(cmd_buf, TX_BUF_SIZE, seg_num, seg_id,
						&img[seg_id * PATCH_SEG_LEN], seg_len);
			WCN_DBG(FM_NTC | CHIP, "pkt_size:%d\n", (fm_s32) pkt_size);
			ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_ROM, SW_RETRY_CNT, ROM_TIMEOUT, NULL);
			FM_UNLOCK(cmd_buf_lock);

			if (ret) {
				WCN_DBG(FM_ALT | CHIP, "mt6580_rom_download failed\n");
				return ret;
			}
		}

		break;
#endif
	case IMG_PATCH:

		for (seg_id = 0; seg_id < seg_num; seg_id++) {
			seg_len = ((seg_id + 1) < seg_num) ? PATCH_SEG_LEN : (len % PATCH_SEG_LEN);
			WCN_DBG(FM_NTC | CHIP, "patch,[seg_id:%d],  [seg_len:%d]\n", seg_id, seg_len);
			if (FM_LOCK(cmd_buf_lock))
				return -FM_ELOCK;
			pkt_size =
			    mt6580_patch_download(cmd_buf, TX_BUF_SIZE, seg_num, seg_id,
						  &img[seg_id * PATCH_SEG_LEN], seg_len);
			WCN_DBG(FM_NTC | CHIP, "pkt_size:%d\n", (fm_s32) pkt_size);
			ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_PATCH, SW_RETRY_CNT, PATCH_TIMEOUT, NULL);
			FM_UNLOCK(cmd_buf_lock);

			if (ret) {
				WCN_DBG(FM_ALT | CHIP, "mt6580_patch_download failed\n");
				return ret;
			}
		}

		break;
#if 0
	case IMG_HW_COEFFICIENT:

		for (seg_id = 0; seg_id < seg_num; seg_id++) {
			seg_len = ((seg_id + 1) < seg_num) ? PATCH_SEG_LEN : (len % PATCH_SEG_LEN);
			WCN_DBG(FM_NTC | CHIP, "hwcoeff,[seg_id:%d],  [seg_len:%d]\n", seg_id, seg_len);
			if (FM_LOCK(cmd_buf_lock))
				return -FM_ELOCK;
			pkt_size =
			    mt6580_hwcoeff_download(cmd_buf, TX_BUF_SIZE, seg_num, seg_id,
						    &img[seg_id * PATCH_SEG_LEN], seg_len);
			WCN_DBG(FM_NTC | CHIP, "pkt_size:%d\n", (fm_s32) pkt_size);
			ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_HWCOEFF, SW_RETRY_CNT, HWCOEFF_TIMEOUT, NULL);
			FM_UNLOCK(cmd_buf_lock);

			if (ret) {
				WCN_DBG(FM_ALT | CHIP, "mt6580_hwcoeff_download failed\n");
				return ret;
			}
		}

		break;
#endif
	case IMG_COEFFICIENT:

		for (seg_id = 0; seg_id < seg_num; seg_id++) {
			seg_len = ((seg_id + 1) < seg_num) ? PATCH_SEG_LEN : (len % PATCH_SEG_LEN);
			WCN_DBG(FM_NTC | CHIP, "coeff,[seg_id:%d],  [seg_len:%d]\n", seg_id, seg_len);
			if (FM_LOCK(cmd_buf_lock))
				return -FM_ELOCK;
			pkt_size =
			    mt6580_coeff_download(cmd_buf, TX_BUF_SIZE, seg_num, seg_id,
						  &img[seg_id * PATCH_SEG_LEN], seg_len);
			WCN_DBG(FM_NTC | CHIP, "pkt_size:%d\n", (fm_s32) pkt_size);
			ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_COEFF, SW_RETRY_CNT, COEFF_TIMEOUT, NULL);
			FM_UNLOCK(cmd_buf_lock);

			if (ret) {
				WCN_DBG(FM_ALT | CHIP, "mt6580_coeff_download failed\n");
				return ret;
			}
		}

		break;
	default:
		break;
	}

	return 0;
}

static fm_s32 mt6580_enable_pmic_tldo(void)
{
	fm_s32 ret = 0;
	fm_u32 hostreg = 0;

	/* set 26M clock mannual on */
	ret = mt6580_host_read(MCUPLL_CON1, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd MCUPLL_CON1 failed\n");
		return ret;
	}
	ret = mt6580_host_write(MCUPLL_CON1, hostreg | (0x1 << 0));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr MCUPLL_CON1 failed\n");
		return ret;
	}

	ret = mt6580_host_read(MCUPLL_CON1, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd MCUPLL_CON1 failed\n");
		return ret;
	}
	ret = mt6580_host_write(MCUPLL_CON1, hostreg | (0x1 << 6));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr MCUPLL_CON1 failed\n");
		return ret;
	}

	ret = mt6580_host_read(MCUPLL_CON1, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd MCUPLL_CON1 failed\n");
		return ret;
	}
	ret = mt6580_host_write(MCUPLL_CON1, hostreg | (0x1 << 16));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr MCUPLL_CON1 failed\n");
		return ret;
	}

	ret = mt6580_host_read(MCUPLL_CON1, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd MCUPLL_CON1 failed\n");
		return ret;
	}
	ret = mt6580_host_write(MCUPLL_CON1, hostreg | (0x1 << 22));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr MCUPLL_CON1 failed\n");
		return ret;
	}

	/* set RX_DET_OUT Gating off */
	ret = mt6580_host_read(CONN_RF_CG, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd CONN_RF_CG failed\n");
		return ret;
	}
	ret = mt6580_host_write(CONN_RF_CG, hostreg | (0x1 << 16));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr CONN_RF_CG failed\n");
		return ret;
	}

	/* set ADC_QD Gating off */
	ret = mt6580_host_read(CONN_RF_CG, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd CONN_RF_CG failed\n");
		return ret;
	}
	ret = mt6580_host_write(CONN_RF_CG, hostreg | (0x1 << 15));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr CONN_RF_CG failed\n");
		return ret;
	}

	/* set ADC_ID Gating off */
	ret = mt6580_host_read(CONN_RF_CG, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd CONN_RF_CG failed\n");
		return ret;
	}
	ret = mt6580_host_write(CONN_RF_CG, hostreg | (0x1 << 14));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr CONN_RF_CG failed\n");
		return ret;
	}

	/* set ADC_CK Gating off */
	ret = mt6580_host_read(CONN_RF_CG, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd CONN_RF_CG failed\n");
		return ret;
	}
	ret = mt6580_host_write(CONN_RF_CG, hostreg | (0x1 << 7));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr CONN_RF_CG failed\n");
		return ret;
	}

	/* set DIG_CK Gating off */
	ret = mt6580_host_read(CONN_RF_CG, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd CONN_RF_CG failed\n");
		return ret;
	}
	ret = mt6580_host_write(CONN_RF_CG, hostreg | (0x1 << 6));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr CONN_RF_CG failed\n");
		return ret;
	}

	return ret;
}

static fm_s32 mt6580_disable_pmic_tldo(void)
{
	fm_s32 ret = 0;
	fm_u32 hostreg = 0;

	/* set 26M clock mannual on */
	ret = mt6580_host_read(MCUPLL_CON1, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd MCUPLL_CON1 failed\n");
		return ret;
	}
	ret = mt6580_host_write(MCUPLL_CON1, hostreg & (~(0x1 << 22)));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr MCUPLL_CON1 failed\n");
		return ret;
	}

	ret = mt6580_host_read(MCUPLL_CON1, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd MCUPLL_CON1 failed\n");
		return ret;
	}
	ret = mt6580_host_write(MCUPLL_CON1, hostreg & (~(0x1 << 16)));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr MCUPLL_CON1 failed\n");
		return ret;
	}

	ret = mt6580_host_read(MCUPLL_CON1, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd MCUPLL_CON1 failed\n");
		return ret;
	}
	ret = mt6580_host_write(MCUPLL_CON1, hostreg & (~(0x1 << 6)));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr MCUPLL_CON1 failed\n");
		return ret;
	}

	ret = mt6580_host_read(MCUPLL_CON1, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd MCUPLL_CON1 failed\n");
		return ret;
	}
	ret = mt6580_host_write(MCUPLL_CON1, hostreg & (~(0x1 << 0)));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr MCUPLL_CON1 failed\n");
		return ret;
	}

	/* set RX_DET_OUT Gating off */
	ret = mt6580_host_read(CONN_RF_CG, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd CONN_RF_CG failed\n");
		return ret;
	}
	ret = mt6580_host_write(CONN_RF_CG, hostreg & (~(0x1 << 16)));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr CONN_RF_CG failed\n");
		return ret;
	}

	/* set ADC_QD Gating off */
	ret = mt6580_host_read(CONN_RF_CG, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd CONN_RF_CG failed\n");
		return ret;
	}
	ret = mt6580_host_write(CONN_RF_CG, hostreg & (~(0x1 << 15)));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr CONN_RF_CG failed\n");
		return ret;
	}

	/* set ADC_ID Gating off */
	ret = mt6580_host_read(CONN_RF_CG, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd CONN_RF_CG failed\n");
		return ret;
	}
	ret = mt6580_host_write(CONN_RF_CG, hostreg & (~(0x1 << 14)));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr CONN_RF_CG failed\n");
		return ret;
	}

	/* set ADC_CK Gating off */
	ret = mt6580_host_read(CONN_RF_CG, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd CONN_RF_CG failed\n");
		return ret;
	}
	ret = mt6580_host_write(CONN_RF_CG, hostreg & (~(0x1 << 7)));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr CONN_RF_CG failed\n");
		return ret;
	}

	/* set DIG_CK Gating off */
	ret = mt6580_host_read(CONN_RF_CG, &hostreg);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup rd CONN_RF_CG failed\n");
		return ret;
	}
	ret = mt6580_host_write(CONN_RF_CG, hostreg & (~(0x1 << 6)));
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " pwrup wr CONN_RF_CG failed\n");
		return ret;
	}

	return ret;
}

static fm_s32 mt6580_PowerUp(fm_u16 *chip_id, fm_u16 *device_id)
{
#define PATCH_BUF_SIZE (4096*6)
	fm_s32 ret = 0;
	fm_u16 pkt_size;
	fm_u16 tmp_reg = 0;

	const fm_s8 *path_patch = NULL;
	const fm_s8 *path_coeff = NULL;
	fm_s32 patch_len = 0;
	fm_u8 *dsp_buf = NULL;

	if (chip_id == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}
	if (device_id == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}

	WCN_DBG(FM_DBG | CHIP, "pwr on seq......\n");

	mt6580_enable_pmic_tldo();

	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_pwrup_clock_on(cmd_buf, TX_BUF_SIZE);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_EN, SW_RETRY_CNT, EN_TIMEOUT, NULL);
	FM_UNLOCK(cmd_buf_lock);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, "mt6580_pwrup_clock_on failed\n");
		return ret;
	}

	mt6580_read(0x62, &tmp_reg);
	*chip_id = tmp_reg;
	*device_id = tmp_reg;
	mt6580_hw_info.chip_id = (fm_s32) tmp_reg;
	WCN_DBG(FM_NTC | CHIP, "chip_id:0x%04x\n", tmp_reg);

	if (mt6580_hw_info.chip_id != 0x6580) {
		WCN_DBG(FM_NTC | CHIP, "fm sys error, reset hw\n");
		return -FM_EFW;
	}

	mt6580_hw_info.eco_ver = (fm_s32) mtk_wcn_wmt_hwver_get();
	WCN_DBG(FM_NTC | CHIP, "ECO version:0x%08x\n", mt6580_hw_info.eco_ver);
	mt6580_hw_info.eco_ver += 1;

	/* get mt6580 DSP rom version */
	ret = mt6580_get_rom_version();
	if (ret >= 0) {
		mt6580_hw_info.rom_ver = ret;
		WCN_DBG(FM_NTC | CHIP, "ROM version: v%d\n", mt6580_hw_info.rom_ver);
	} else {
		WCN_DBG(FM_ERR | CHIP, "get ROM version failed\n");
		/* ret=-4 means signal got when control FM. usually get sig 9 to kill FM process. */
		/* now cancel FM power up sequence is recommended. */
		return ret;
	}

	dsp_buf = fm_vmalloc(PATCH_BUF_SIZE);
	if (!dsp_buf) {
		WCN_DBG(FM_ALT | CHIP, "-ENOMEM\n");
		return -ENOMEM;
	}

	ret = mt6580_get_patch_path(mt6580_hw_info.rom_ver, &path_patch);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " mt6580_get_patch_path failed\n");
		return ret;
	}
	patch_len = fm_file_read(path_patch, dsp_buf, PATCH_BUF_SIZE, 0);
	ret = mt6580_DspPatch((const fm_u8 *)dsp_buf, patch_len, IMG_PATCH);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " DL DSPpatch failed\n");
		return ret;
	}

	ret = mt6580_get_coeff_path(mt6580_hw_info.rom_ver, &path_coeff);
	patch_len = fm_file_read(path_coeff, dsp_buf, PATCH_BUF_SIZE, 0);

	mt6580_hw_info.rom_ver += 1;

	tmp_reg = dsp_buf[38] | (dsp_buf[39] << 8);	/* to be confirmed */
	mt6580_hw_info.patch_ver = (fm_s32) tmp_reg;
	WCN_DBG(FM_NTC | CHIP, "Patch version: 0x%08x\n", mt6580_hw_info.patch_ver);

	if (ret == 1) {
		dsp_buf[4] = 0x00;	/* if we found rom version undefined, we should disable patch */
		dsp_buf[5] = 0x00;
	}

	ret = mt6580_DspPatch((const fm_u8 *)dsp_buf, patch_len, IMG_COEFFICIENT);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, " DL DSPcoeff failed\n");
		return ret;
	}
	mt6580_write(0x92, 0x0000);
	mt6580_write(0x90, 0x0040);
	mt6580_write(0x90, 0x0000);

	if (dsp_buf) {
		fm_vfree(dsp_buf);
		dsp_buf = NULL;
	}

	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_pwrup_digital_init(cmd_buf, TX_BUF_SIZE);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_EN, SW_RETRY_CNT, EN_TIMEOUT, NULL);
	FM_UNLOCK(cmd_buf_lock);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, "mt6580_pwrup_digital_init failed\n");
		return ret;
	}

	/* set audio output I2S TX mode  */
	mt6580_write(0x9B, 0x3);

	WCN_DBG(FM_NTC | CHIP, "pwr on seq ok\n");

	return ret;
}

static fm_s32 mt6580_PowerDown(void)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;
	fm_u16 dataRead;

	WCN_DBG(FM_DBG | CHIP, "pwr down seq\n");

	/*SW work around for MCUFA issue.
	 *if interrupt happen before doing rampdown, DSP can't switch MCUFA back well.
	 * In case read interrupt, and clean if interrupt found before rampdown.
	 */
	mt6580_read(FM_MAIN_INTR, &dataRead);

	if (dataRead & 0x1) {
		ret = mt6580_write(FM_MAIN_INTR, dataRead);	/* clear status flag */
		if (ret)
			WCN_DBG(FM_ALT | CHIP, "mt6580_pwrdown wr FM_MAIN_INTR failed\n");
	}
	/* mt6580_RampDown(); */

	/* set audio output I2X Rx mode: */
	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_pwrdown(cmd_buf, TX_BUF_SIZE);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_EN, SW_RETRY_CNT, EN_TIMEOUT, NULL);
	FM_UNLOCK(cmd_buf_lock);
	if (ret) {
		WCN_DBG(FM_ALT | CHIP, "mt6580_pwrdown failed\n");
		return ret;
	}

	mt6580_disable_pmic_tldo();

	return ret;
}

static fm_bool mt6580_SetFreq(fm_u16 freq)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;
	fm_u16 chan_para = 0;
	fm_u16 freq_reg = 0;

	fm_cb_op->cur_freq_set(freq);

	chan_para = mt6580_chan_para_get(freq);
	WCN_DBG(FM_DBG | CHIP, "%d chan para = %d\n", (fm_s32) freq, (fm_s32) chan_para);
	freq_reg = freq;
	if (0 == fm_get_channel_space(freq_reg))
		freq_reg *= 10;
	WCN_DBG(FM_DBG | CHIP, "freq_reg = %d\n", freq_reg);
	ret = mt6580_write(0x60, 0x00000007);
	if (ret)
		WCN_DBG(FM_ERR | CHIP, "set freq wr 0x60 failed\n");
	/* add this paragragh to resolve Rainier FM sensitivity bad in low band issue */
#if 0
	if (mt6580_TDD_chan_check(freq)) {
		ret = mt6580_set_bits(0x39, 0x0008, 0xFFF3);	/* use TDD solution */
		WCN_DBG(FM_ERR | CHIP, "set freq wr 0x30 failed\n");
	} else {
		ret = mt6580_set_bits(0x39, 0x0000, 0xFFF3);	/* default use FDD solution */
		WCN_DBG(FM_ERR | CHIP, "set freq wr 0x30 failed\n");
	}
#endif
	if ((6500 <= freq_reg) && (freq_reg <= 7290)) {
		ret = mt6580_write(0x39, 0xd002);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else if ((7295 <= freq_reg) && (freq_reg <= 8410)) {
		ret = mt6580_write(0x39, 0xce02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else if ((8415 <= freq_reg) && (freq_reg <= 9815)) {
		ret = mt6580_write(0x39, 0xcc02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else if ((9820 <= freq_reg) && (freq_reg <= 9830)) {
		ret = mt6580_write(0x39, 0xca02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else if ((9835 <= freq_reg) && (freq_reg <= 9940)) {
		ret = mt6580_write(0x39, 0xcc02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else if ((9845 <= freq_reg) && (freq_reg <= 10800)) {
		ret = mt6580_write(0x39, 0xca02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else {
		ret = mt6580_write(0x39, 0xca02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	}

	/* end */
	ret = mt6580_write(0x6a, 0x00000021);
	if (ret)
		WCN_DBG(FM_ERR | CHIP, "set freq wr 0x6a failed\n");

	ret = mt6580_write(0x6b, 0x00000021);
	if (ret)
		WCN_DBG(FM_ERR | CHIP, "set freq wr 0x6b failed\n");

	ret = mt6580_write(0x60, 0x0000000F);
	if (ret)
		WCN_DBG(FM_ERR | CHIP, "set freq wr 0x60 failed\n");


	freq_reg = (freq_reg - 6400) * 2 / 10;
	ret = mt6580_set_bits(0x65, freq_reg, 0xFC00);
	if (ret) {
		WCN_DBG(FM_ERR | CHIP, "set freq wr 0x65 failed\n");
		return fm_false;
	}
	ret = mt6580_set_bits(0x65, (chan_para << 12), 0x0FFF);
	if (ret) {
		WCN_DBG(FM_ERR | CHIP, "set freq wr 0x65 failed\n");
		return fm_false;
	}

	if (FM_LOCK(cmd_buf_lock))
		return fm_false;
	pkt_size = mt6580_tune(cmd_buf, TX_BUF_SIZE, freq, chan_para);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_TUNE | FLAG_TUNE_DONE, SW_RETRY_CNT, TUNE_TIMEOUT, NULL);
	FM_UNLOCK(cmd_buf_lock);

	if (ret) {
		WCN_DBG(FM_ALT | CHIP, "mt6580_tune failed\n");
		return fm_false;
	}

	WCN_DBG(FM_DBG | CHIP, "set freq to %d ok\n", freq);

	return fm_true;
}

#if 0
/*
* mt6580_Seek
* @pFreq - IN/OUT parm, IN start freq/OUT seek valid freq
* @seekdir - 0:up, 1:down
* @space - 1:50KHz, 2:100KHz, 4:200KHz
* return fm_true:seek success; fm_false:seek failed
*/
static fm_bool mt6580_Seek(fm_u16 min_freq, fm_u16 max_freq, fm_u16 *pFreq, fm_u16 seekdir, fm_u16 space)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size, temp;

	mt6580_RampDown();
	mt6580_read(FM_MAIN_CTRL, &temp);
	mt6580_Mute(fm_true);

	if (FM_LOCK(cmd_buf_lock))
		return fm_false;
	pkt_size = mt6580_seek(cmd_buf, TX_BUF_SIZE, seekdir, space, max_freq, min_freq);
	ret =
	    fm_cmd_tx(cmd_buf, pkt_size, FLAG_SEEK | FLAG_SEEK_DONE, SW_RETRY_CNT, SEEK_TIMEOUT,
		      mt6580_get_read_result);
	FM_UNLOCK(cmd_buf_lock);

	if (!ret && mt6580_res) {
		*pFreq = mt6580_res->seek_result;
		/* fm_cb_op->cur_freq_set(*pFreq); */
	} else {
		WCN_DBG(FM_ALT | CHIP, "mt6580_seek failed\n");
		return ret;
	}

	/* get the result freq */
	WCN_DBG(FM_NTC | CHIP, "seek, result freq:%d\n", *pFreq);
	mt6580_RampDown();
	if ((temp & 0x0020) == 0)
		mt6580_Mute(fm_false);

	return fm_true;
}
#endif
#define FM_CQI_LOG_PATH "/mnt/sdcard/fmcqilog"

static fm_s32 mt6580_full_cqi_get(fm_s32 min_freq, fm_s32 max_freq, fm_s32 space, fm_s32 cnt)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;
	fm_u16 freq, orig_freq;
	fm_s32 i, j, k;
	fm_s32 space_val, max, min, num;
	struct mt6580_full_cqi *p_cqi;
	fm_u8 *cqi_log_title = "Freq, RSSI, PAMD, PR, FPAMD, MR, ATDC, PRX, ATDEV, SMGain, DltaRSSI\n";
	fm_u8 cqi_log_buf[100] = { 0 };
	fm_s32 pos;
	fm_u8 cqi_log_path[100] = { 0 };

	WCN_DBG(FM_NTC | CHIP, "6627 cqi log start\n");
	/* for soft-mute tune, and get cqi */
	freq = fm_cb_op->cur_freq_get();
	if (0 == fm_get_channel_space(freq))
		freq *= 10;

	/* get cqi */
	orig_freq = freq;
	if (0 == fm_get_channel_space(min_freq))
		min = min_freq * 10;
	else
		min = min_freq;

	if (0 == fm_get_channel_space(max_freq))
		max = max_freq * 10;
	else
		max = max_freq;

	if (space == 0x0001)
		space_val = 5;	/* 50Khz */
	else if (space == 0x0002)
		space_val = 10;	/* 100Khz */
	else if (space == 0x0004)
		space_val = 20;	/* 200Khz */
	else
		space_val = 10;

	num = (max - min) / space_val + 1;	/* Eg, (8760 - 8750) / 10 + 1 = 2 */
	for (k = 0; (10000 == orig_freq) && (0xffffffff == g_dbg_level) && (k < cnt); k++) {
		WCN_DBG(FM_NTC | CHIP, "cqi file:%d\n", k + 1);
		freq = min;
		pos = 0;
		fm_memcpy(cqi_log_path, FM_CQI_LOG_PATH, strlen(FM_CQI_LOG_PATH));
		sprintf(&cqi_log_path[strlen(FM_CQI_LOG_PATH)], "%d.txt", k + 1);
		fm_file_write(cqi_log_path, cqi_log_title, strlen(cqi_log_title), &pos);
		for (j = 0; j < num; j++) {
			if (FM_LOCK(cmd_buf_lock))
				return -FM_ELOCK;
			pkt_size = mt6580_full_cqi_req(cmd_buf, TX_BUF_SIZE, &freq, 1, 1);
			ret =
			    fm_cmd_tx(cmd_buf, pkt_size, FLAG_SM_TUNE, SW_RETRY_CNT,
				      SM_TUNE_TIMEOUT, mt6580_get_read_result);
			FM_UNLOCK(cmd_buf_lock);

			if (!ret && mt6580_res) {
				WCN_DBG(FM_NTC | CHIP, "smt cqi size %d\n", mt6580_res->cqi[0]);
				p_cqi = (struct mt6580_full_cqi *)&mt6580_res->cqi[2];
				for (i = 0; i < mt6580_res->cqi[1]; i++) {
					/* just for debug */
					WCN_DBG(FM_NTC | CHIP,
						"freq %d, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x\n",
						p_cqi[i].ch, p_cqi[i].rssi, p_cqi[i].pamd,
						p_cqi[i].pr, p_cqi[i].fpamd, p_cqi[i].mr,
						p_cqi[i].atdc, p_cqi[i].prx, p_cqi[i].atdev,
						p_cqi[i].smg, p_cqi[i].drssi);
					/* format to buffer */
					sprintf(cqi_log_buf,
						"%04d,%04x,%04x,%04x,%04x,%04x,%04x,%04x,%04x,%04x,%04x,\n",
						p_cqi[i].ch, p_cqi[i].rssi, p_cqi[i].pamd,
						p_cqi[i].pr, p_cqi[i].fpamd, p_cqi[i].mr,
						p_cqi[i].atdc, p_cqi[i].prx, p_cqi[i].atdev,
						p_cqi[i].smg, p_cqi[i].drssi);
					/* write back to log file */
					fm_file_write(cqi_log_path, cqi_log_buf, strlen(cqi_log_buf), &pos);
				}
			} else {
				WCN_DBG(FM_ALT | CHIP, "smt get CQI failed\n");
				ret = -1;
			}
			freq += space_val;
		}
		fm_cb_op->cur_freq_set(0);	/* avoid run too much times */
	}
	WCN_DBG(FM_NTC | CHIP, "6627 cqi log done\n");

	return ret;
}

/*
 * mt6580_GetCurRSSI - get current freq's RSSI value
 * RS=RSSI
 * If RS>511, then RSSI(dBm)= (RS-1024)/16*6
 *				   else RSSI(dBm)= RS/16*6
 */
static fm_s32 mt6580_GetCurRSSI(fm_s32 *pRSSI)
{
	fm_u16 tmp_reg;

	mt6580_read(FM_RSSI_IND, &tmp_reg);
	tmp_reg = tmp_reg & 0x03ff;

	if (pRSSI) {
		*pRSSI = (tmp_reg > 511) ? (((tmp_reg - 1024) * 6) >> 4) : ((tmp_reg * 6) >> 4);
		WCN_DBG(FM_DBG | CHIP, "rssi:%d, dBm:%d\n", tmp_reg, *pRSSI);
	} else {
		WCN_DBG(FM_ERR | CHIP, "get rssi para error\n");
		return -FM_EPARA;
	}

	return 0;
}

static fm_u16 mt6580_vol_tbl[16] = { 0x0000, 0x0519, 0x066A, 0x0814,
	0x0A2B, 0x0CCD, 0x101D, 0x1449,
	0x198A, 0x2027, 0x287A, 0x32F5,
	0x4027, 0x50C3, 0x65AD, 0x7FFF
};

static fm_s32 mt6580_SetVol(fm_u8 vol)
{
	fm_s32 ret = 0;

	vol = (vol > 15) ? 15 : vol;
	ret = mt6580_write(0x7D, mt6580_vol_tbl[vol]);
	if (ret) {
		WCN_DBG(FM_ERR | CHIP, "Set vol=%d Failed\n", vol);
		return ret;
	}

	WCN_DBG(FM_DBG | CHIP, "Set vol=%d OK\n", vol);

	if (vol == 10) {
		fm_print_cmd_fifo();	/* just for debug */
		fm_print_evt_fifo();
	}
	return 0;
}

static fm_s32 mt6580_GetVol(fm_u8 *pVol)
{
	int ret = 0;
	fm_u16 tmp;
	fm_s32 i;

	if (pVol == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}

	ret = mt6580_read(0x7D, &tmp);
	if (ret) {
		*pVol = 0;
		WCN_DBG(FM_ERR | CHIP, "Get vol Failed\n");
		return ret;
	}

	for (i = 0; i < 16; i++) {
		if (mt6580_vol_tbl[i] == tmp) {
			*pVol = i;
			break;
		}
	}

	WCN_DBG(FM_DBG | CHIP, "Get vol=%d OK\n", *pVol);
	return 0;
}

static fm_s32 mt6580_dump_reg(void)
{
	fm_s32 i;
	fm_u16 TmpReg;

	for (i = 0; i < 0xff; i++) {
		mt6580_read(i, &TmpReg);
		WCN_DBG(FM_NTC | CHIP, "0x%02x=0x%04x\n", i, TmpReg);
	}
	return 0;
}

/*0:mono, 1:stereo*/
static fm_bool mt6580_GetMonoStereo(fm_u16 *pMonoStereo)
{
#define FM_BF_STEREO 0x1000
	fm_u16 TmpReg;

	if (pMonoStereo) {
		mt6580_read(FM_RSSI_IND, &TmpReg);
		*pMonoStereo = (TmpReg & FM_BF_STEREO) >> 12;
	} else {
		WCN_DBG(FM_ERR | CHIP, "MonoStero: para err\n");
		return fm_false;
	}

	FM_LOG_NTC(CHIP, "Get MonoStero:0x%04x\n", *pMonoStereo);
	return fm_true;
}

static fm_s32 mt6580_SetMonoStereo(fm_s32 MonoStereo)
{
	fm_s32 ret = 0;

	FM_LOG_NTC(CHIP, "set to %s\n", MonoStereo ? "mono" : "auto");
	mt6580_top_write(0x50, 0x0007);

	if (MonoStereo) {	/*mono */
		ret = mt6580_set_bits(0x75, 0x0008, ~0x0008);
	} else {		/*auto switch */

		ret = mt6580_set_bits(0x75, 0x0000, ~0x0008);
	}

	mt6580_top_write(0x50, 0x000F);
	return ret;
}

static fm_s32 mt6580_GetCapArray(fm_s32 *ca)
{
	fm_u16 dataRead;
	fm_u16 tmp = 0;

	if (ca == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}
	mt6580_read(0x60, &tmp);
	mt6580_write(0x60, tmp & 0xFFF7);	/* 0x60 D3=0 */

	mt6580_read(0x26, &dataRead);
	*ca = dataRead;

	mt6580_write(0x60, tmp);	/* 0x60 D3=1 */
	return 0;
}

/*
 * mt6580_GetCurPamd - get current freq's PAMD value
 * PA=PAMD
 * If PA>511 then PAMD(dB)=  (PA-1024)/16*6,
 *				else PAMD(dB)=PA/16*6
 */
static fm_bool mt6580_GetCurPamd(fm_u16 *pPamdLevl)
{
	fm_u16 tmp_reg;
	fm_u16 dBvalue, valid_cnt = 0;
	int i, total = 0;

	for (i = 0; i < 8; i++) {
		if (mt6580_read(FM_ADDR_PAMD, &tmp_reg)) {
			*pPamdLevl = 0;
			return fm_false;
		}

		tmp_reg &= 0x03FF;
		dBvalue = (tmp_reg > 256) ? ((512 - tmp_reg) * 6 / 16) : 0;
		if (dBvalue != 0) {
			total += dBvalue;
			valid_cnt++;
			WCN_DBG(FM_DBG | CHIP, "[%d]PAMD=%d\n", i, dBvalue);
		}
		Delayms(3);
	}
	if (valid_cnt != 0)
		*pPamdLevl = total / valid_cnt;
	else
		*pPamdLevl = 0;

	WCN_DBG(FM_NTC | CHIP, "PAMD=%d\n", *pPamdLevl);
	return fm_true;
}

static fm_s32 mt6580_i2s_info_get(fm_s32 *ponoff, fm_s32 *pmode, fm_s32 *psample)
{
	if (ponoff == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}
	if (pmode == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}
	if (psample == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}

	*ponoff = mt6580_fm_config.aud_cfg.i2s_info.status;
	*pmode = mt6580_fm_config.aud_cfg.i2s_info.mode;
	*psample = mt6580_fm_config.aud_cfg.i2s_info.rate;

	return 0;
}

static fm_s32 mt6580fm_get_audio_info(fm_audio_info_t *data)
{
	memcpy(data, &mt6580_fm_config.aud_cfg, sizeof(fm_audio_info_t));
	return 0;
}

static fm_s32 mt6580_hw_info_get(struct fm_hw_info *req)
{
	if (req == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}

	req->chip_id = mt6580_hw_info.chip_id;
	req->eco_ver = mt6580_hw_info.eco_ver;
	req->patch_ver = mt6580_hw_info.patch_ver;
	req->rom_ver = mt6580_hw_info.rom_ver;

	return 0;
}

static fm_s32 mt6580_pre_search(void)
{
	mt6580_RampDown();
	/* disable audio output I2S Rx mode */
	mt6580_host_write(0x80101054, 0x00000000);
	/* disable audio output I2S Tx mode */
	mt6580_write(0x9B, 0x0000);
	/*FM_LOG_NTC(FM_NTC | CHIP, "search threshold: RSSI=%d,de-RSSI=%d,smg=%d %d\n",
	   mt6580_fm_config.rx_cfg.long_ana_rssi_th, mt6580_fm_config.rx_cfg.desene_rssi_th,
	   mt6580_fm_config.rx_cfg.smg_th); */
	return 0;
}

static fm_s32 mt6580_restore_search(void)
{
	mt6580_RampDown();
	/* set audio output I2S Tx mode */
	mt6580_write(0x9B, 0xF9AB);
	/* set audio output I2S Rx mode */
	mt6580_host_write(0x80101054, 0x00003f35);
	return 0;
}

static fm_s32 mt6580_soft_mute_tune(fm_u16 freq, fm_s32 *rssi, fm_bool *valid)
{
	fm_s32 ret = 0;
	fm_u16 pkt_size;
	struct mt6580_full_cqi *p_cqi;
	fm_s32 RSSI = 0, PAMD = 0, MR = 0, ATDC = 0;
	fm_u32 PRX = 0, ATDEV = 0;
	fm_u16 softmuteGainLvl = 0;
	fm_u16 freq_reg = 0;

	/* add this paragragh to resolve Rainier FM sensitivity bad in low band issue */
#if 0
	ret = mt6580_chan_para_get(freq);
	if (ret == 2)
		ret = mt6580_set_bits(FM_CHANNEL_SET, 0x2000, 0x0FFF);	/* mdf HiLo */
	else
		ret = mt6580_set_bits(FM_CHANNEL_SET, 0x0000, 0x0FFF);	/* clear FA/HL/ATJ */
#endif
	freq_reg = freq;
	if (0 == fm_get_channel_space(freq_reg))
		freq_reg *= 10;

	ret = mt6580_write(0x60, 0x00000007);
	if (ret)
		WCN_DBG(FM_ERR | CHIP, "set freq wr 0x60 failed\n");

	if ((6500 <= freq_reg) && (freq_reg <= 7290)) {
		ret = mt6580_write(0x39, 0xd002);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else if ((7295 <= freq_reg) && (freq_reg <= 8410)) {
		ret = mt6580_write(0x39, 0xce02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else if ((8415 <= freq_reg) && (freq_reg <= 9815)) {
		ret = mt6580_write(0x39, 0xcc02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else if ((9820 <= freq_reg) && (freq_reg <= 9830)) {
		ret = mt6580_write(0x39, 0xca02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else if ((9835 <= freq_reg) && (freq_reg <= 9940)) {
		ret = mt6580_write(0x39, 0xcc02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else if ((9845 <= freq_reg) && (freq_reg <= 10800)) {
		ret = mt6580_write(0x39, 0xca02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	} else {
		ret = mt6580_write(0x39, 0xca02);
		if (ret)
			WCN_DBG(FM_ERR | CHIP, "set freq wr 0x39 failed\n");
	}

	ret = mt6580_write(0x60, 0x0000000f);
	if (ret)
		WCN_DBG(FM_ERR | CHIP, "set freq wr 0x60 failed\n");
	/* end */

	if (FM_LOCK(cmd_buf_lock))
		return -FM_ELOCK;
	pkt_size = mt6580_full_cqi_req(cmd_buf, TX_BUF_SIZE, &freq, 1, 1);
	ret = fm_cmd_tx(cmd_buf, pkt_size, FLAG_SM_TUNE, SW_RETRY_CNT, SM_TUNE_TIMEOUT, mt6580_get_read_result);
	FM_UNLOCK(cmd_buf_lock);

	if (!ret && mt6580_res) {
		WCN_DBG(FM_NTC | CHIP, "smt cqi size %d\n", mt6580_res->cqi[0]);
		p_cqi = (struct mt6580_full_cqi *)&mt6580_res->cqi[2];
		/* just for debug */
		WCN_DBG(FM_NTC | CHIP,
			"freq %d, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x\n",
			p_cqi->ch, p_cqi->rssi, p_cqi->pamd, p_cqi->pr, p_cqi->fpamd, p_cqi->mr,
			p_cqi->atdc, p_cqi->prx, p_cqi->atdev, p_cqi->smg, p_cqi->drssi);
		RSSI = ((p_cqi->rssi & 0x03FF) >= 512) ? ((p_cqi->rssi & 0x03FF) - 1024) : (p_cqi->rssi & 0x03FF);
		PAMD = ((p_cqi->pamd & 0x1FF) >= 256) ? ((p_cqi->pamd & 0x01FF) - 512) : (p_cqi->pamd & 0x01FF);
		MR = ((p_cqi->mr & 0x01FF) >= 256) ? ((p_cqi->mr & 0x01FF) - 512) : (p_cqi->mr & 0x01FF);
		ATDC = (p_cqi->atdc >= 32768) ? (65536 - p_cqi->atdc) : (p_cqi->atdc);
		if (ATDC < 0)
			ATDC = (~(ATDC)) - 1;	/* Get abs value of ATDC */

		PRX = (p_cqi->prx & 0x00FF);
		ATDEV = p_cqi->atdev;
		softmuteGainLvl = p_cqi->smg;
		/* check if the channel is valid according to each CQIs */
		if ((RSSI >= mt6580_fm_config.rx_cfg.long_ana_rssi_th)
		    && (PAMD <= mt6580_fm_config.rx_cfg.pamd_th)
		    && (ATDC <= mt6580_fm_config.rx_cfg.atdc_th)
		    && (MR >= mt6580_fm_config.rx_cfg.mr_th)
		    && (PRX >= mt6580_fm_config.rx_cfg.prx_th)
		    && (ATDEV >= ATDC)	/* sync scan algorithm */
		    && (softmuteGainLvl >= mt6580_fm_config.rx_cfg.smg_th)) {
			*valid = fm_true;
		} else {
			*valid = fm_false;
		}
		*rssi = RSSI;
/*		if(RSSI < -296)
			WCN_DBG(FM_NTC | CHIP, "rssi\n");
		else if(PAMD > -12)
			WCN_DBG(FM_NTC | CHIP, "PAMD\n");
		else if(ATDC > 3496)
			WCN_DBG(FM_NTC | CHIP, "ATDC\n");
		else if(MR < -67)
			WCN_DBG(FM_NTC | CHIP, "MR\n");
		else if(PRX < 80)
			WCN_DBG(FM_NTC | CHIP, "PRX\n");
		else if(ATDEV < ATDC)
			WCN_DBG(FM_NTC | CHIP, "ATDEV\n");
		else if(softmuteGainLvl < 16421)
			WCN_DBG(FM_NTC | CHIP, "softmuteGainLvl\n");
			*/
	} else {
		WCN_DBG(FM_ALT | CHIP, "smt get CQI failed\n");
		return fm_false;
	}
	WCN_DBG(FM_NTC | CHIP, "valid=%d\n", *valid);
	return fm_true;
}

static fm_bool mt6580_em_test(fm_u16 group_idx, fm_u16 item_idx, fm_u32 item_value)
{
	return fm_true;
}

/*
parm:
	parm.th_type: 0, RSSI. 1,desense RSSI. 2,SMG.
    parm.th_val: threshold value
*/
static fm_s32 mt6580_set_search_th(fm_s32 idx, fm_s32 val, fm_s32 reserve)
{
	switch (idx) {
	case 0:
		{
			mt6580_fm_config.rx_cfg.long_ana_rssi_th = val;
			WCN_DBG(FM_NTC | CHIP, "set rssi th =%d\n", val);
			break;
		}
	case 1:
		{
			mt6580_fm_config.rx_cfg.desene_rssi_th = val;
			WCN_DBG(FM_NTC | CHIP, "set desense rssi th =%d\n", val);
			break;
		}
	case 2:
		{
			mt6580_fm_config.rx_cfg.smg_th = val;
			WCN_DBG(FM_NTC | CHIP, "set smg th =%d\n", val);
			break;
		}
	default:
		break;
	}
	return 0;
}

static fm_s32 MT6580fm_low_power_wa_default(fm_s32 fmon)
{
	return 0;
}

fm_s32 MT6580fm_low_ops_register(struct fm_lowlevel_ops *ops)
{
	fm_s32 ret = 0;
	/* Basic functions. */

	if (ops == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}
	if (ops->cb.cur_freq_get == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}
	if (ops->cb.cur_freq_set == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}
	fm_cb_op = &ops->cb;

	ops->bi.pwron = mt6580_pwron;
	ops->bi.pwroff = mt6580_pwroff;
	ops->bi.msdelay = Delayms;
	ops->bi.usdelay = Delayus;
	ops->bi.read = mt6580_read;
	ops->bi.write = mt6580_write;
	ops->bi.top_read = mt6580_top_read;
	ops->bi.top_write = mt6580_top_write;
	ops->bi.host_read = mt6580_host_read;
	ops->bi.host_write = mt6580_host_write;
	ops->bi.setbits = mt6580_set_bits;
	ops->bi.chipid_get = mt6580_get_chipid;
	ops->bi.mute = mt6580_Mute;
	ops->bi.rampdown = mt6580_RampDown;
	ops->bi.pwrupseq = mt6580_PowerUp;
	ops->bi.pwrdownseq = mt6580_PowerDown;
	ops->bi.setfreq = mt6580_SetFreq;
	ops->bi.low_pwr_wa = MT6580fm_low_power_wa_default;
	ops->bi.get_aud_info = mt6580fm_get_audio_info;
#if 0
	ops->bi.seek = mt6580_Seek;
	ops->bi.seekstop = mt6580_SeekStop;
	ops->bi.scan = mt6580_Scan;
	ops->bi.cqi_get = mt6580_CQI_Get;
#ifdef CONFIG_MTK_FM_50KHZ_SUPPORT
	ops->bi.scan = mt6580_Scan_50KHz;
	ops->bi.cqi_get = mt6580_CQI_Get_50KHz;
#endif
	ops->bi.scanstop = mt6580_ScanStop;
	ops->bi.i2s_set = mt6580_I2s_Setting;
#endif
	ops->bi.rssiget = mt6580_GetCurRSSI;
	ops->bi.volset = mt6580_SetVol;
	ops->bi.volget = mt6580_GetVol;
	ops->bi.dumpreg = mt6580_dump_reg;
	ops->bi.msget = mt6580_GetMonoStereo;
	ops->bi.msset = mt6580_SetMonoStereo;
	ops->bi.pamdget = mt6580_GetCurPamd;
	ops->bi.em = mt6580_em_test;
	ops->bi.anaswitch = mt6580_SetAntennaType;
	ops->bi.anaget = mt6580_GetAntennaType;
	ops->bi.caparray_get = mt6580_GetCapArray;
	ops->bi.hwinfo_get = mt6580_hw_info_get;
	ops->bi.i2s_get = mt6580_i2s_info_get;
	ops->bi.is_dese_chan = mt6580_is_dese_chan;
	ops->bi.softmute_tune = mt6580_soft_mute_tune;
	ops->bi.desense_check = mt6580_desense_check;
	ops->bi.cqi_log = mt6580_full_cqi_get;
	ops->bi.pre_search = mt6580_pre_search;
	ops->bi.restore_search = mt6580_restore_search;
	ops->bi.set_search_th = mt6580_set_search_th;

	cmd_buf_lock = fm_lock_create("27_cmd");
	ret = fm_lock_get(cmd_buf_lock);

	cmd_buf = fm_zalloc(TX_BUF_SIZE + 1);

	if (!cmd_buf) {
		WCN_DBG(FM_ALT | CHIP, "6627 fm lib alloc tx buf failed\n");
		ret = -1;
	}
#if 0				/* def CONFIG_MTK_FM_50KHZ_SUPPORT */
	cqi_fifo = fm_fifo_create("6628_cqi_fifo", sizeof(struct adapt_fm_cqi), 640);
	if (!cqi_fifo) {
		WCN_DBG(FM_ALT | CHIP, "6627 fm lib create cqi fifo failed\n");
		ret = -1;
	}
#endif

	return ret;
}

fm_s32 MT6580fm_low_ops_unregister(struct fm_lowlevel_ops *ops)
{
	fm_s32 ret = 0;
	/* Basic functions. */
	if (ops == NULL) {
		pr_err("%s,invalid pointer\n", __func__);
		return -FM_EPARA;
	}

#if 0				/* def CONFIG_MTK_FM_50KHZ_SUPPORT */
	fm_fifo_release(cqi_fifo);
#endif

	if (cmd_buf) {
		fm_free(cmd_buf);
		cmd_buf = NULL;
	}

	ret = fm_lock_put(cmd_buf_lock);
	fm_memset(&ops->bi, 0, sizeof(struct fm_basic_interface));
	return ret;
}

/* static struct fm_pub pub; */
/* static struct fm_pub_cb *pub_cb = &pub.pub_tbl; */

static const fm_u16 mt6580_mcu_dese_list[] = {
	0
};

static const fm_u16 mt6580_gps_dese_list[] = {
	7850, 7860
};

static const fm_s8 mt6580_chan_para_map[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 6500~6595 */
	0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0,	/* 6600~6695 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 6700~6795 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 6800~6895 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0,	/* 6900~6995 */
	2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 7000~7095 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 7100~7195 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0,	/* 7200~7295 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2,	/* 7300~7395 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 7400~7495 */
	0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,	/* 7500~7595 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 7600~7695 */
	0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 7700~7795 */
	8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0,	/* 7800~7895 */
	0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 7900~7995 */
	0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 8000~8095 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0,	/* 8100~8195 */
	0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 8200~8295 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0,	/* 8300~8395 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 8400~8495 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 8500~8595 */
	0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,	/* 8600~8695 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 8700~8795 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 8800~8895 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 8900~8995 */
	0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 9000~9095 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 9100~9195 */
	0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0,	/* 9200~9295 */
	0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 9300~9395 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 9400~9495 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 9500~9595 */
	2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0,	/* 9600~9695 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 9700~9795 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 9800~9895 */
	0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 9900~9995 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 10000~10095 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 10100~10195 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 10200~10295 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 10300~10395 */
	8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 10400~10495 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 10500~10595 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 10600~10695 */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	/* 10700~10795 */
	0			/* 10800 */
};

static const fm_u16 mt6580_scan_dese_list[] = {
	6700, 7800, 9210, 9220, 9300, 1040, 1041
};

static const fm_u16 mt6580_TDD_list[] = {
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 6500~6595 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 6600~6695 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 6700~6795 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 6800~6895 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 6900~6995 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 7000~7095 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 7100~7195 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 7200~7295 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 7300~7395 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 7400~7495 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 7500~7595 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 7600~7695 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 7700~7795 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 7800~7895 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 7900~7995 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 8000~8095 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 8100~8195 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 8200~8295 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 8300~8395 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 8400~8495 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 8500~8595 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 8600~8695 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 8700~8795 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 8800~8895 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 8900~8995 */
	0x0000, 0x0000, 0x0000, 0x0100, 0x0000,	/* 9000~9095 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 9100~9195 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 9200~9295 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 9300~9395 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 9400~9495 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 9500~9595 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 9600~9695 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 9700~9795 */
	0x0000, 0x0101, 0x0000, 0x0000, 0x0000,	/* 9800~9895 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 9900~9995 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 10000~10095 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 10100~10195 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 10200~10295 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 10300~10395 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 10400~10495 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 10500~10595 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 10600~10695 */
	0x0000, 0x0001, 0x0000, 0x0000, 0x0000,	/* 10700~10795 */
	0x0000			/* 10800 */
};

static const fm_u16 mt6580_TDD_Mask[] = {
	0x0001, 0x0010, 0x0100, 0x1000
};

/* return value: 0, not a de-sense channel; 1, this is a de-sense channel; else error no */
static fm_s32 mt6580_is_dese_chan(fm_u16 freq)
{
	fm_s32 size;

	/* return 0;//HQA only :skip desense channel check. */
	size = sizeof(mt6580_scan_dese_list) / sizeof(mt6580_scan_dese_list[0]);

	if (0 == fm_get_channel_space(freq))
		freq *= 10;

	while (size) {
		if (mt6580_scan_dese_list[size - 1] == freq)
			return 1;

		size--;
	}

	return 0;
}

/*  return value:
1, is desense channel and rssi is less than threshold;
0, not desense channel or it is but rssi is more than threshold.*/
static fm_s32 mt6580_desense_check(fm_u16 freq, fm_s32 rssi)
{
	if (mt6580_is_dese_chan(freq)) {
		if (rssi < mt6580_fm_config.rx_cfg.desene_rssi_th)
			return 1;

		WCN_DBG(FM_DBG | CHIP, "desen_rssi %d th:%d\n", rssi, mt6580_fm_config.rx_cfg.desene_rssi_th);
	}
	return 0;
}
#if 0
static fm_bool mt6580_TDD_chan_check(fm_u16 freq)
{
	fm_u32 i = 0;
	fm_u16 freq_tmp = freq;
	fm_s32 ret = 0;

	ret = fm_get_channel_space(freq_tmp);
	if (0 == ret)
		freq_tmp *= 10;
	else if (-1 == ret)
		return fm_false;

	i = (freq_tmp - 6500) / 5;

	WCN_DBG(FM_NTC | CHIP, "Freq %d is 0x%4x, mask is 0x%4x\n", freq, (mt6580_TDD_list[i / 4]),
		mt6580_TDD_Mask[i % 4]);
	if (mt6580_TDD_list[i / 4] & mt6580_TDD_Mask[i % 4]) {
		WCN_DBG(FM_NTC | CHIP, "Freq %d use TDD solution\n", freq);
		return fm_true;
	} else
		return fm_false;
}
#endif
/* get channel parameter, HL side/ FA / ATJ */
static fm_u16 mt6580_chan_para_get(fm_u16 freq)
{
	fm_s32 pos, size;

	/* return 0;//for HQA only: skip FA/HL/ATJ */
	if (0 == fm_get_channel_space(freq))
		freq *= 10;

	if (freq < 6500)
		return 0;

	pos = (freq - 6500) / 5;

	size = sizeof(mt6580_chan_para_map) / sizeof(mt6580_chan_para_map[0]);

	pos = (pos < 0) ? 0 : pos;
	pos = (pos > (size - 1)) ? (size - 1) : pos;

	return mt6580_chan_para_map[pos];
}
