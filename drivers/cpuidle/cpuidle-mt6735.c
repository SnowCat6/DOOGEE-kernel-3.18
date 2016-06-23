/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <linux/module.h>
#include <linux/printk.h>
//#include <asm/system_misc.h>
#include <mach/mt_spm.h>
#include <mach/mt_spm_idle.h>
#include <mach/mt_thermal.h>
#include "mtk_cpuidle_internal.h"

#define IDLE_TAG     "[Power/swap]"
#define idle_ver(fmt, args...)		pr_info(IDLE_TAG fmt, ##args)	/* pr_debug show nothing */

//FIXME: Denali early porting
#if 1
void __attribute__((weak))
bus_dcm_enable(void)
{
	//FIXME: Denali early porting
}
void __attribute__((weak))
bus_dcm_disable(void)
{
	//FIXME: Denali early porting
}
void __attribute__((weak))
tscpu_cancel_thermal_timer(void)
{
	//FIXME: Denali early porting
}

void __attribute__((weak))
tscpu_start_thermal_timer(void)
{
	//FIXME: Denali early porting
}
#endif

u32 slp_spm_deepidle_flags = {
	SPM_MD_VRF18_DIS//SPM_CPU_PDN_DIS
	//SPM_MD_VRF18_DIS|SPM_CPU_PDN_DIS|SPM_CPU_DVS_DIS|SPM_DISABLE_ATF_ABORT
};

static inline void dpidle_pre_handler(void)
{
#ifndef CONFIG_MTK_FPGA
#ifdef CONFIG_THERMAL
    //cancel thermal hrtimer for power saving
    tscpu_cancel_thermal_timer();
	mtkts_bts_cancel_thermal_timer();
    mtkts_btsmdpa_cancel_thermal_timer();
    mtkts_pmic_cancel_thermal_timer();
    mtkts_battery_cancel_thermal_timer();
    mtkts_pa_cancel_thermal_timer();
	mtkts_allts_cancel_thermal_timer();
#endif
#endif
#if 0//FIXME: K2 early porting
    // disable gpu dvfs timer
    mtk_enable_gpu_dvfs_timer(false);

    // disable cpu dvfs timer
    hp_enable_timer(0);
#endif

}

static inline void dpidle_post_handler(void)
{
#if 0//FIXME: K2 early porting
    // disable cpu dvfs timer
    hp_enable_timer(1);

    // enable gpu dvfs timer
    mtk_enable_gpu_dvfs_timer(true);
#endif
#ifndef CONFIG_MTK_FPGA
#ifdef CONFIG_THERMAL
    //restart thermal hrtimer for update temp info
    tscpu_start_thermal_timer();
	mtkts_bts_start_thermal_timer();
    mtkts_btsmdpa_start_thermal_timer();
    mtkts_pmic_start_thermal_timer();
    mtkts_battery_start_thermal_timer();
    mtkts_pa_start_thermal_timer();
	mtkts_allts_start_thermal_timer();
#endif
#endif
}

static int mt6735_dpidle_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
    if (printk_ratelimit())
        printk(KERN_WARNING "MT6735 cpuidle DPIDLE\n");

#if 1
    dpidle_pre_handler();
    spm_go_to_dpidle(slp_spm_deepidle_flags, 0);
    dpidle_post_handler();
#else
	cpu_do_idle();
#endif

    idle_cnt_inc(IDLE_TYPE_DP, 0);

	return index;
}

static int mt6735_soidle_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
    if (printk_ratelimit())
        printk(KERN_WARNING "MT6735 cpuidle SODI\n");

	cpu_do_idle();

    idle_cnt_inc(IDLE_TYPE_SO, 0);

	return index;
}
static int mt6735_dcm_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
    if (printk_ratelimit())
        printk(KERN_WARNING "MT6735 cpuidle DCM\n");

	cpu_do_idle();

    idle_cnt_inc(IDLE_TYPE_SL, smp_processor_id());

	return index;
}

static int mt6735_rgidle_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
    if (printk_ratelimit())
        printk(KERN_WARNING "MT6735 cpuidle rgidle\n");

	cpu_do_idle();

    idle_cnt_inc(IDLE_TYPE_RG, smp_processor_id());

	return index;
}

static struct cpuidle_driver mt6735_cpuidle_driver = {
	.name             = "mt6735_cpuidle",
	.owner            = THIS_MODULE,
	.states[0] = {
		.enter            = mt6735_dpidle_enter,
		.exit_latency     = 2000,            /* 2,000 us = 2 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIME_VALID,
		.name             = "MT6735 dpidle",
		.desc             = "deepidle",
	},
	.states[1] = {
		.enter            = mt6735_soidle_enter,
		.exit_latency     = 2000,            /* 2,000 us = 2 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIME_VALID,
		.name             = "MT6735 SODI",
		.desc             = "SODI",
	},
	.states[2] = {
		.enter            = mt6735_dcm_enter,
		.exit_latency     = 2000,            /* 2,000 us = 2 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIME_VALID,
		.name             = "MT6735 DCM",
		.desc             = "DCM",
	},
	.states[3] = {
		.enter            = mt6735_rgidle_enter,
		.exit_latency     = 2000,            /* 2,000 us = 2 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIME_VALID,
		.name             = "MT6735 rgidle",
		.desc             = "WFI",
	},
	.state_count = 4,
	.safe_state_index = 0,
};

int __init mt6735_cpuidle_init(void)
{
	return cpuidle_register(&mt6735_cpuidle_driver, NULL);
}
device_initcall(mt6735_cpuidle_init);
