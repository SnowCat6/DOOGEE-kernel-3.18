/**
 * @file mt_cpufreq.h
 * @brief CPU DVFS driver interface
 */

#ifndef __MT_CPUFREQ_H__
#define __MT_CPUFREQ_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __MT_CPUFREQ_C__
#define CPUFREQ_EXTERN
#else
#define CPUFREQ_EXTERN extern
#endif

/*=============================================================*/
// Include files
/*=============================================================*/

// system includes

// project includes

// local includes

// forward references


/*=============================================================*/
// Macro definition
/*=============================================================*/

/*=============================================================*/
// Type definition
/*=============================================================*/

enum mt_cpu_dvfs_id {
    MT_CPU_DVFS_LITTLE,
    NR_MT_CPU_DVFS,
};

enum top_ckmuxsel {
    TOP_CKMUXSEL_CLKSQ   = 0, /* i.e. reg setting */
    TOP_CKMUXSEL_ARMPLL  = 1,
    TOP_CKMUXSEL_MAINPLL = 2,

    NR_TOP_CKMUXSEL,
} ;

/*
 * PMIC_WRAP
 */

/* Phase */
enum pmic_wrap_phase_id {
    PMIC_WRAP_PHASE_NORMAL,
    PMIC_WRAP_PHASE_SUSPEND,
    PMIC_WRAP_PHASE_DEEPIDLE,

    NR_PMIC_WRAP_PHASE,
};

/* IDX mapping */
#ifdef CONFIG_ARCH_MT6735M
enum {
    IDX_NM_VCORE_TRANS4,	/* 0 */ /* PMIC_WRAP_PHASE_NORMAL */
    IDX_NM_VCORE_TRANS3,	/* 1 */
    IDX_NM_VCORE_HPM,		/* 2 */
    IDX_NM_VCORE_TRANS2,	/* 3 */
    IDX_NM_VCORE_TRANS1,	/* 4 */
    IDX_NM_VCORE_LPM,		/* 5 */
    IDX_NM_VCORE_UHPM,		/* 6 */
    IDX_NM_VRF18_0_PWR_ON,	/* 7 */
    IDX_NM_NOT_USED,		/* 8 */
    IDX_NM_VRF18_0_SHUTDOWN,	/* 9 */

    NR_IDX_NM,
};
#else   /* !CONFIG_ARCH_MT6735M */
enum {
    IDX_NM_NOT_USED1,		/* 0 */ /* PMIC_WRAP_PHASE_NORMAL */
    IDX_NM_NOT_USED2,		/* 1 */
    IDX_NM_VCORE_HPM,		/* 2 */
    IDX_NM_VCORE_TRANS2,	/* 3 */
    IDX_NM_VCORE_TRANS1,	/* 4 */
    IDX_NM_VCORE_LPM,		/* 5 */
    IDX_NM_VRF18_0_SHUTDOWN,	/* 6 */
    IDX_NM_VRF18_0_PWR_ON,	/* 7 */

    NR_IDX_NM,
};
#endif

enum {
    IDX_SP_VPROC_PWR_ON,	/* 0 */ /* PMIC_WRAP_PHASE_SUSPEND */
    IDX_SP_VPROC_SHUTDOWN,	/* 1 */
    IDX_SP_VCORE_HPM,		/* 2 */
    IDX_SP_VCORE_TRANS2,	/* 3 */
    IDX_SP_VCORE_TRANS1,	/* 4 */
    IDX_SP_VCORE_LPM,		/* 5 */
    IDX_SP_VSRAM_SHUTDOWN,	/* 6 */
    IDX_SP_VRF18_0_PWR_ON,	/* 7 */
    IDX_SP_VSRAM_PWR_ON,	/* 8 */
    IDX_SP_VRF18_0_SHUTDOWN,	/* 9 */

    NR_IDX_SP,
};
enum {
    IDX_DI_VPROC_NORMAL,	/* 0 */ /* PMIC_WRAP_PHASE_DEEPIDLE */
    IDX_DI_VPROC_SLEEP,		/* 1 */
    IDX_DI_VCORE_HPM,		/* 2 */
    IDX_DI_VCORE_TRANS2,	/* 3 */
    IDX_DI_VCORE_TRANS1,	/* 4 */
    IDX_DI_VCORE_LPM,		/* 5 */
    IDX_DI_VSRAM_SLEEP,		/* 6 */
    IDX_DI_VRF18_0_PWR_ON,	/* 7 */
    IDX_DI_VSRAM_NORMAL,	/* 8 */
    IDX_DI_VRF18_0_SHUTDOWN,	/* 9 */
#ifdef CONFIG_ARCH_MT6753
    IDX_DI_VCORE_IDLE_LPM,	/* 10 */
    IDX_DI_VSRAM_SLEEP_FOR_TURBO,	/* 11 */
    IDX_DI_VSRAM_NORMAL_FOR_TURBO,	/* 12 */
#endif

    NR_IDX_DI,
};


typedef void (*cpuVoltsampler_func)(enum mt_cpu_dvfs_id , unsigned int mv);
/*=============================================================*/
// Global variable definition
/*=============================================================*/


/*=============================================================*/
// Global function definition
/*=============================================================*/

/* PMIC WRAP */
CPUFREQ_EXTERN void mt_cpufreq_set_pmic_phase(enum pmic_wrap_phase_id phase);
CPUFREQ_EXTERN void mt_cpufreq_set_pmic_cmd(enum pmic_wrap_phase_id phase, int idx, unsigned int cmd_wdata);
CPUFREQ_EXTERN void mt_cpufreq_apply_pmic_cmd(int idx);

/* Dormant profiling */
CPUFREQ_EXTERN unsigned int mt_cpufreq_get_cur_freq(enum mt_cpu_dvfs_id id);

/* PTP-OD */
CPUFREQ_EXTERN unsigned int mt_cpufreq_get_freq_by_idx(enum mt_cpu_dvfs_id id, int idx);
CPUFREQ_EXTERN int mt_cpufreq_update_volt(enum mt_cpu_dvfs_id id, unsigned int *volt_tbl, int nr_volt_tbl);
CPUFREQ_EXTERN void mt_cpufreq_restore_default_volt(enum mt_cpu_dvfs_id id);
CPUFREQ_EXTERN unsigned int mt_cpufreq_get_cur_volt(enum mt_cpu_dvfs_id id);
CPUFREQ_EXTERN void mt_cpufreq_enable_by_ptpod(enum mt_cpu_dvfs_id id);
CPUFREQ_EXTERN unsigned int mt_cpufreq_disable_by_ptpod(enum mt_cpu_dvfs_id id);
CPUFREQ_EXTERN int  mt_cpufreq_set_lte_volt(int pmic_val);

/* Thermal */
CPUFREQ_EXTERN void mt_cpufreq_thermal_protect(unsigned int limited_power);

/* PBM */
CPUFREQ_EXTERN void mt_cpufreq_set_power_limit_by_pbm(unsigned int limited_power);
CPUFREQ_EXTERN unsigned int mt_cpufreq_get_leakage_mw(enum mt_cpu_dvfs_id id);

/* SDIO */
#if 0   // moved to Vcore DVFS
CPUFREQ_EXTERN void mt_vcore_dvfs_disable_by_sdio(unsigned int type, bool disabled);
CPUFREQ_EXTERN void mt_vcore_dvfs_volt_set_by_sdio(unsigned int volt);
CPUFREQ_EXTERN unsigned int mt_vcore_dvfs_volt_get_by_sdio(void);

CPUFREQ_EXTERN unsigned int mt_get_cur_volt_vcore_ao(void);
//CPUFREQ_EXTERN unsigned int mt_get_cur_volt_vcore_pdn(void);
#endif

/* Generic */
//CPUFREQ_EXTERN int mt_cpufreq_state_set(int enabled);
CPUFREQ_EXTERN int mt_cpufreq_set_cpu_clk_src(enum mt_cpu_dvfs_id id, enum top_ckmuxsel sel);
CPUFREQ_EXTERN enum top_ckmuxsel mt_cpufreq_get_cpu_clk_src(enum mt_cpu_dvfs_id id);
CPUFREQ_EXTERN void mt_cpufreq_setvolt_registerCB(cpuVoltsampler_func pCB);
CPUFREQ_EXTERN bool mt_cpufreq_earlysuspend_status_get(void);

#ifdef CONFIG_CPU_FREQ_GOV_HOTPLUG
CPUFREQ_EXTERN void mt_cpufreq_set_ramp_down_count_const(enum mt_cpu_dvfs_id id, int count);
#endif

#ifndef __KERNEL__
CPUFREQ_EXTERN int mt_cpufreq_pdrv_probe(void);
CPUFREQ_EXTERN int mt_cpufreq_set_opp_volt(enum mt_cpu_dvfs_id id, int idx);
CPUFREQ_EXTERN int mt_cpufreq_set_freq(enum mt_cpu_dvfs_id id, int idx);
CPUFREQ_EXTERN unsigned int dvfs_get_cpu_freq(enum mt_cpu_dvfs_id id);
CPUFREQ_EXTERN void dvfs_set_cpu_freq_FH(enum mt_cpu_dvfs_id id, int freq);
CPUFREQ_EXTERN unsigned int dvfs_get_cur_oppidx(enum mt_cpu_dvfs_id id);
CPUFREQ_EXTERN unsigned int cpu_frequency_output_slt(enum mt_cpu_dvfs_id id);
CPUFREQ_EXTERN unsigned int mt_get_cur_volt_lte(void);
CPUFREQ_EXTERN unsigned int dvfs_get_cpu_volt(enum mt_cpu_dvfs_id id);
CPUFREQ_EXTERN void dvfs_set_cpu_volt(enum mt_cpu_dvfs_id id, int volt);
CPUFREQ_EXTERN void dvfs_set_gpu_volt(int pmic_val);
CPUFREQ_EXTERN void dvfs_set_vcore_ao_volt(int pmic_val);
//CPUFREQ_EXTERN void dvfs_set_vcore_pdn_volt(int pmic_val);
CPUFREQ_EXTERN void dvfs_disable_by_ptpod(int id);
CPUFREQ_EXTERN void dvfs_enable_by_ptpod(int id);
#endif /* ! __KERNEL__ */

#undef CPUFREQ_EXTERN

#ifdef __cplusplus
}
#endif

#endif // __MT_CPUFREQ_H__
