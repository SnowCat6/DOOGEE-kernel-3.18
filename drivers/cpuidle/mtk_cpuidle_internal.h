#ifndef _MTK_CPUIDLE_INTERNAL_H_
#define _MTK_CPUIDLE_INTERNAL_H_

#include <mach/mt_clkmgr.h>

enum idle_lock_spm_id{
    IDLE_SPM_LOCK_VCORE_DVFS= 0,
};

enum {
    IDLE_TYPE_DP = 0,
    IDLE_TYPE_SO = 1,
    IDLE_TYPE_SL = 2,
    IDLE_TYPE_RG = 3,
    NR_TYPES     = 4,
};

enum {
    BY_CPU       = 0,
    BY_CLK       = 1,
    BY_TMR       = 2,
    BY_OTH       = 3,
    BY_VTG       = 4,
    NR_REASONS   = 5
};


extern unsigned int     dpidle_condition_mask[NR_GRPS];
extern unsigned int     dpidle_block_mask[NR_GRPS];
extern unsigned int     dpidle_time_critera;
extern unsigned int     dpidle_block_time_critera;//default 30sec
extern bool             dpidle_by_pass_cg;

extern const char *reason_name[NR_REASONS];

int mtk_cpuidle_debugfs_init(void);

void idle_cnt_inc(int idle_type, int cpu);
unsigned long idle_cnt_get(int idle_type, int cpu);

void idle_block_cnt_inc(int idle_type, int reason);
unsigned long idle_block_cnt_get(int idle_type, int reason);
void idle_block_cnt_clr(int idle_type);

int idle_switch_get(int idle_type);
#endif
