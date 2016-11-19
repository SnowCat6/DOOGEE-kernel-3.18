/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __MTK_NAND_H
#define __MTK_NAND_H


struct nfi_saved_para {
	u8	suspend_flag;
	u16 sNFI_CNFG_REG16;
	u32 sNFI_PAGEFMT_REG16;
	u32 sNFI_CON_REG16;
	u32 sNFI_ACCCON_REG32;
	u16 sNFI_INTR_EN_REG16;
	u16 sNFI_IOCON_REG16;
	u16 sNFI_CSEL_REG16;
	u16 sNFI_DEBUG_CON1_REG16;

	u32 sECC_ENCCNFG_REG32;
	u32 sECC_FDMADDR_REG32;
	u32 sECC_DECCNFG_REG32;

	u32 sSNAND_MISC_CTL;
	u32 sSNAND_MISC_CTL2;
	u32 sSNAND_DLY_CTL1;
	u32 sSNAND_DLY_CTL2;
	u32 sSNAND_DLY_CTL3;
	u32 sSNAND_DLY_CTL4;
	u32 sSNAND_CNFG;
};

struct mtk_nand_pl_test {
	suseconds_t last_erase_time;
	suseconds_t last_prog_time;
	u32 nand_program_wdt_enable;
	u32 nand_erase_wdt_enable;
};

struct mtk_nand_host {
	struct nand_chip		nand_chip;
	struct mtd_info			mtd;
	struct mtk_nand_host_hw	*hw;
#ifdef CONFIG_PM
	struct nfi_saved_para   saved_para;
#endif
#ifdef CONFIG_PWR_LOSS_MTK_SPOH
	struct mtk_nand_pl_test pl;
#endif
	struct mtd_erase_region_info erase_region[20];
};

extern struct mtk_nand_host *host;

#endif
