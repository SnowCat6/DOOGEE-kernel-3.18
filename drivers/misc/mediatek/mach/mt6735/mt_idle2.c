#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpu.h>

#include <linux/types.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mtk_gpu_utility.h>
#include <linux/spinlock.h>
#include <linux/cpumask.h>

#include <mach/mt_cirq.h>
#include <asm/system_misc.h>
#include <mach/mt_typedefs.h>
#include <mach/sync_write.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_dcm.h>
#include <mach/mt_gpt.h>
#include <mach/mt_cpuxgpt.h>
#include <mach/mt_spm.h>
#include <mach/mt_spm_idle.h>
#include <mach/mt_spm_sleep.h>
#include <mach/hotplug.h>
#include <mach/mt_cpufreq.h>
#include <mach/mt_power_gs.h>
#include <mach/mt_ptp.h>
#include <mach/mt_timer.h>
#include <mach/irqs.h>
#include <mach/mt_thermal.h>
#include <mach/mt_idle.h>
#include <mach/mt_boot.h>

#define IDLE_TAG     "[Power/swap]"
#define spm_emerg(fmt, args...)		pr_emerg(IDLE_TAG fmt, ##args)
#define spm_alert(fmt, args...)		pr_alert(IDLE_TAG fmt, ##args)
#define spm_crit(fmt, args...)		pr_crit(IDLE_TAG fmt, ##args)
#define idle_err(fmt, args...)		pr_err(IDLE_TAG fmt, ##args)
#define idle_dbg(fmt, args...)		pr_warn(IDLE_TAG fmt, ##args)
#define spm_notice(fmt, args...)	pr_notice(IDLE_TAG fmt, ##args)
#define idle_info(fmt, args...)		pr_info(IDLE_TAG fmt, ##args)
#define idle_ver(fmt, args...)		pr_info(IDLE_TAG fmt, ##args)	/* pr_debug show nothing */

#define idle_gpt GPT4

#define idle_readl(addr) \
    DRV_Reg32(addr)

#define idle_writel(addr, val)   \
    mt65xx_reg_sync_writel(val, addr)

#define idle_setl(addr, val) \
    mt65xx_reg_sync_writel(idle_readl(addr) | (val), addr)

#define idle_clrl(addr, val) \
    mt65xx_reg_sync_writel(idle_readl(addr) & ~(val), addr)


extern unsigned long localtimer_get_counter(void);
extern int localtimer_set_next_event(unsigned long evt);
extern void hp_enable_timer(int enable);
extern bool clkmgr_idle_can_enter(unsigned int *condition_mask, unsigned int *block_mask);
extern void clkmgr_faudintbus_pll2sq(void);
extern void clkmgr_faudintbus_sq2pll(void);

static unsigned long rgidle_cnt[NR_CPUS] = {0};
static bool mt_idle_chk_golden = 0;
static bool mt_dpidle_chk_golden = 0;


#define INVALID_GRP_ID(grp) (grp < 0 || grp >= NR_GRPS)


#if 1
void __attribute__((weak)) bus_dcm_enable(void) {}
void __attribute__((weak)) bus_dcm_disable(void) {}
void __attribute__((weak)) tscpu_cancel_thermal_timer(void) {}
void __attribute__((weak)) tscpu_start_thermal_timer(void) {}
void __attribute__((weak)) mtkts_bts_cancel_thermal_timer(void) {}
void __attribute__((weak)) mtkts_btsmdpa_cancel_thermal_timer(void) {}
void __attribute__((weak)) mtkts_pmic_cancel_thermal_timer(void) {}
void __attribute__((weak)) mtkts_battery_cancel_thermal_timer(void) {}
void __attribute__((weak)) mtkts_pa_cancel_thermal_timer(void) {}
void __attribute__((weak)) mtkts_allts_cancel_thermal_timer(void) {}
void __attribute__((weak)) mtkts_bts_start_thermal_timer(void) {}
void __attribute__((weak)) mtkts_btsmdpa_start_thermal_timer(void) {}
void __attribute__((weak)) mtkts_pmic_start_thermal_timer(void) {}
void __attribute__((weak)) mtkts_battery_start_thermal_timer(void) {}
void __attribute__((weak)) mtkts_pa_start_thermal_timer(void) {}
void __attribute__((weak)) mtkts_allts_start_thermal_timer(void) {}
bool __attribute__((weak)) mt_cpufreq_earlysuspend_status_get(void)
{
	return false;
}
#endif


enum {
    IDLE_TYPE_DP = 0,
    IDLE_TYPE_SO = 1,
    IDLE_TYPE_SL = 2,
    IDLE_TYPE_RG = 3,
    NR_TYPES	= 4,
};

enum {
    BY_CPU = 0,
    BY_CLK = 1,
    BY_TMR = 2,
    BY_OTH = 3,
    BY_VTG = 4,
    NR_REASONS = 5
};

/*Idle handler on/off*/
static int idle_switch[NR_TYPES] = {
    1,  //dpidle switch
    1,  //soidle switch
    1,  //slidle switch
    1,  //rgidle switch
};

static unsigned int dpidle_condition_mask[NR_GRPS] = {
    0x0000008A, //INFRA:
    0x37FA1FFD, //PERI0:
    0x000FFFFF, //DISP0:
    0x0000003F, //DISP1:
    0x00000FE1, //IMAGE:
    0x00000001, //MFG:
    0x00000000, //AUDIO:
    0x00000001, //VDEC0:
    0x00000001, //VDEC1:
    // 0x00001111, //VENC:
};

static unsigned int soidle_condition_mask[NR_GRPS] = {
    0x00000088, //INFRA:
    0x37FC0FFC, //PERI0:
    0x000033FC, //DISP0:
    0x00000030, //DISP1:
    0x00000FE1, //IMAGE:
    0x00000001, //MFG:
    0x00000000, //AUDIO:
    0x00000001, //VDEC0:
    0x00000001, //VDEC1:
    // 0x00001111, //VENC:
};

static unsigned int slidle_condition_mask[NR_GRPS] = {
    0x00000000, //INFRA:
    0x07C01000, //PERI0:
    0x00000000, //DISP0:
    0x00000000, //DISP1:
    0x00000000, //IMAGE:
    0x00000000, //MFG:
    0x00000000, //AUDIO:
    0x00000000, //VDEC0:
    0x00000000, //VDEC1:
    // 0x00000000, //VENC:
};

static const char *idle_name[NR_TYPES] = {
    "dpidle",
    "soidle",
    "slidle",
    "rgidle",
};

static const char *reason_name[NR_REASONS] = {
    "by_cpu",
    "by_clk",
    "by_tmr",
    "by_oth",
    "by_vtg",
};

/*Slow Idle*/
static unsigned int slidle_block_mask[NR_GRPS] = {0x0};
static unsigned long slidle_cnt[NR_CPUS] = {0};
static unsigned long slidle_block_cnt[NR_REASONS] = {0};

/*SODI*/
static unsigned int     soidle_block_mask[NR_GRPS] = {0x0};
static unsigned int     soidle_timer_left;
static unsigned int     soidle_timer_left2;
#ifndef CONFIG_SMP
static unsigned int     soidle_timer_cmp;
#endif
static unsigned int     soidle_time_critera = 26000; //FIXME: need fine tune
static unsigned int     soidle_block_time_critera = 30000;//default 30sec
static unsigned long    soidle_cnt[NR_CPUS] = {0};
static unsigned long    soidle_block_cnt[NR_CPUS][NR_REASONS] = {{0}};
static unsigned long long soidle_block_prev_time = 0;
static bool             soidle_by_pass_cg = 0;

/*DeepIdle*/
static unsigned int     dpidle_block_mask[NR_GRPS] = {0x0};
static unsigned int     dpidle_timer_left;
static unsigned int     dpidle_timer_left2;
#ifndef CONFIG_SMP
static unsigned int     dpidle_timer_cmp;
#endif
static unsigned int     dpidle_time_critera = 26000;
static unsigned int     dpidle_block_time_critera = 30000;//default 30sec
static unsigned long    dpidle_cnt[NR_CPUS] = {0};
static unsigned long    dpidle_block_cnt[NR_REASONS] = {0};
static unsigned long long dpidle_block_prev_time = 0;
static bool             dpidle_by_pass_cg = 0;

static unsigned int 	idle_spm_lock = 0;

static long int idle_get_current_time_ms(void)
{
    struct timeval t;
    do_gettimeofday(&t);
    return ((t.tv_sec & 0xFFF) * 1000000 + t.tv_usec) / 1000;
}

static DEFINE_SPINLOCK(idle_spm_spin_lock);

void idle_lock_spm(enum idle_lock_spm_id id)
{
    unsigned long flags;
    spin_lock_irqsave(&idle_spm_spin_lock, flags);
    idle_spm_lock|=(1<<id);
    spin_unlock_irqrestore(&idle_spm_spin_lock, flags);
}

void idle_unlock_spm(enum idle_lock_spm_id id)
{
    unsigned long flags;
    spin_lock_irqsave(&idle_spm_spin_lock, flags);
    idle_spm_lock&=~(1<<id);
    spin_unlock_irqrestore(&idle_spm_spin_lock, flags);
}


/************************************************
 * SODI part
 ************************************************/
static DEFINE_MUTEX(soidle_locked);

static void enable_soidle_by_mask(int grp, unsigned int mask)
{
    mutex_lock(&soidle_locked);
    soidle_condition_mask[grp] &= ~mask;
    mutex_unlock(&soidle_locked);
}

static void disable_soidle_by_mask(int grp, unsigned int mask)
{
    mutex_lock(&soidle_locked);
    soidle_condition_mask[grp] |= mask;
    mutex_unlock(&soidle_locked);
}

void enable_soidle_by_bit(int id)
{
    int grp = id / 32;
    unsigned int mask = 1U << (id % 32);
    BUG_ON(INVALID_GRP_ID(grp));
    enable_soidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(enable_soidle_by_bit);

void disable_soidle_by_bit(int id)
{
    int grp = id / 32;
    unsigned int mask = 1U << (id % 32);
    BUG_ON(INVALID_GRP_ID(grp));
    disable_soidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(disable_soidle_by_bit);

#define SODI_DVT                    0

#if SODI_DVT
#define SODI_APxGPT_TimerCount      0
#define SODI_DISPLAY_DRV_CHK_DIS    0
#define SODI_CG_CHK_DIS             1
#endif

bool soidle_can_enter(int cpu)
{
    int reason = NR_REASONS;
    unsigned long long soidle_block_curr_time = 0;

#ifdef CONFIG_SMP
    if ((atomic_read(&is_in_hotplug) == 1)||(num_online_cpus() != 1)) {
        reason = BY_CPU;
        goto out;
    }
#endif

    if(idle_spm_lock){
        reason = BY_VTG;
        goto out;
    }

#if !defined(SODI_DISPLAY_DRV_CHK_DIS) || (SODI_DISPLAY_DRV_CHK_DIS == 0)
    // decide when to enable SODI by display driver
    if(spm_get_sodi_en()==0){
        reason = BY_OTH;
        goto out;
	}
#endif

    if (soidle_by_pass_cg == 0) {
        memset(soidle_block_mask, 0, NR_GRPS * sizeof(unsigned int));
        if (!clkmgr_idle_can_enter(soidle_condition_mask, soidle_block_mask)) {
#if !defined(SODI_CG_CHK_DIS) || (SODI_CG_CHK_DIS == 0)
            reason = BY_CLK;
            goto out;
#endif
        }
    }

#ifdef CONFIG_SMP
    soidle_timer_left = localtimer_get_counter();
    if ((int)soidle_timer_left < soidle_time_critera ||
            ((int)soidle_timer_left) < 0) {
        reason = BY_TMR;
        goto out;
    }
#else
    gpt_get_cnt(GPT1, &soidle_timer_left);
    gpt_get_cmp(GPT1, &soidle_timer_cmp);
    if((soidle_timer_cmp-soidle_timer_left)<soidle_time_critera)
    {
        reason = BY_TMR;
        goto out;
    }
#endif

out:
    if (reason < NR_REASONS) {
        if( soidle_block_prev_time == 0 )
            soidle_block_prev_time = idle_get_current_time_ms();

        soidle_block_curr_time = idle_get_current_time_ms();
        if((soidle_block_curr_time - soidle_block_prev_time) > soidle_block_time_critera)
        {
            if ((smp_processor_id() == 0))
            {
                int i = 0;

                for (i = 0; i < nr_cpu_ids; i++) {
                    idle_ver("soidle_cnt[%d]=%lu, rgidle_cnt[%d]=%lu\n",
                            i, soidle_cnt[i], i, rgidle_cnt[i]);
                }

                for (i = 0; i < NR_REASONS; i++) {
                    idle_ver("[%d]soidle_block_cnt[0][%s]=%lu\n", i, reason_name[i],
                            soidle_block_cnt[0][i]);
                }

                for (i = 0; i < NR_GRPS; i++) {
                    idle_ver("[%02d]soidle_condition_mask[%-8s]=0x%08x\t\t"
                            "soidle_block_mask[%-8s]=0x%08x\n", i,
                            grp_get_name(i), soidle_condition_mask[i],
                            grp_get_name(i), soidle_block_mask[i]);
                }

                memset(soidle_block_cnt, 0, sizeof(soidle_block_cnt));
                soidle_block_prev_time = idle_get_current_time_ms();
            }
        }

        soidle_block_cnt[cpu][reason]++;
        return false;
    } else {
        soidle_block_prev_time = idle_get_current_time_ms();
        return true;
    }
}

void soidle_before_wfi(int cpu)
{
#ifdef CONFIG_SMP
	#if !defined(SODI_APxGPT_TimerCount) || (SODI_APxGPT_TimerCount == 0)
	soidle_timer_left2 = localtimer_get_counter();
	#else
	soidle_timer_left2 = 13000000*SODI_APxGPT_TimerCount;
	#endif

    if( (int)soidle_timer_left2 <=0 )
    {
        gpt_set_cmp(idle_gpt, 1);//Trigger idle_gpt Timerout imediately
    }
    else
    {
        gpt_set_cmp(idle_gpt, soidle_timer_left2);
    }

    start_gpt(idle_gpt);
#else
    gpt_get_cnt(GPT1, &soidle_timer_left2);
#endif

}

void soidle_after_wfi(int cpu)
{
#ifdef CONFIG_SMP
    if (gpt_check_and_ack_irq(idle_gpt)) {
        localtimer_set_next_event(1);
    } else {
        /* waked up by other wakeup source */
        unsigned int cnt, cmp;
        gpt_get_cnt(idle_gpt, &cnt);
        gpt_get_cmp(idle_gpt, &cmp);
        if (unlikely(cmp < cnt)) {
            idle_err("[%s]GPT%d: counter = %10u, compare = %10u\n", __func__,
                    idle_gpt + 1, cnt, cmp);
            BUG();
        }

        localtimer_set_next_event(cmp-cnt);
        stop_gpt(idle_gpt);
    }
#endif
    soidle_cnt[cpu]++;
}

/************************************************
 * deep idle part
 ************************************************/
static DEFINE_MUTEX(dpidle_locked);

static void enable_dpidle_by_mask(int grp, unsigned int mask)
{
    mutex_lock(&dpidle_locked);
    dpidle_condition_mask[grp] &= ~mask;
    mutex_unlock(&dpidle_locked);
}

static void disable_dpidle_by_mask(int grp, unsigned int mask)
{
    mutex_lock(&dpidle_locked);
    dpidle_condition_mask[grp] |= mask;
    mutex_unlock(&dpidle_locked);
}

void enable_dpidle_by_bit(int id)
{
    int grp = id / 32;
    unsigned int mask = 1U << (id % 32);
    BUG_ON(INVALID_GRP_ID(grp));
    enable_dpidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(enable_dpidle_by_bit);

void disable_dpidle_by_bit(int id)
{
    int grp = id / 32;
    unsigned int mask = 1U << (id % 32);
    BUG_ON(INVALID_GRP_ID(grp));
    disable_dpidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(disable_dpidle_by_bit);

static bool dpidle_can_enter(void)
{
    int reason = NR_REASONS;
    int i = 0;
    unsigned long long dpidle_block_curr_time = 0;

    if(dpidle_by_pass_cg==0){
        if (!mt_cpufreq_earlysuspend_status_get()){
            reason = BY_VTG;
            goto out;
        }
    }

#ifdef CONFIG_SMP
    if ((atomic_read(&is_in_hotplug) >= 1)||(num_online_cpus() != 1)) {
        reason = BY_CPU;
        goto out;
    }
#endif

    if(idle_spm_lock){
        reason = BY_VTG;
        goto out;
    }

    if(dpidle_by_pass_cg==0){
	    memset(dpidle_block_mask, 0, NR_GRPS * sizeof(unsigned int));
	    if (!clkmgr_idle_can_enter(dpidle_condition_mask, dpidle_block_mask)) {
	        reason = BY_CLK;
	        goto out;
	    }
    }
#ifdef CONFIG_SMP
    dpidle_timer_left = localtimer_get_counter();
    if ((int)dpidle_timer_left < dpidle_time_critera ||
            ((int)dpidle_timer_left) < 0) {
        reason = BY_TMR;
        goto out;
    }
#else
    gpt_get_cnt(GPT1, &dpidle_timer_left);
    gpt_get_cmp(GPT1, &dpidle_timer_cmp);
    if((dpidle_timer_cmp-dpidle_timer_left)<dpidle_time_critera)
    {
        reason = BY_TMR;
        goto out;
    }
#endif

out:
    if (reason < NR_REASONS) {
        if( dpidle_block_prev_time == 0 )
            dpidle_block_prev_time = idle_get_current_time_ms();

        dpidle_block_curr_time = idle_get_current_time_ms();
        if((dpidle_block_curr_time - dpidle_block_prev_time) > dpidle_block_time_critera)
        {
            if ((smp_processor_id() == 0))
            {
                for (i = 0; i < nr_cpu_ids; i++) {
                    idle_ver("dpidle_cnt[%d]=%lu, rgidle_cnt[%d]=%lu\n",
                            i, dpidle_cnt[i], i, rgidle_cnt[i]);
                }

                for (i = 0; i < NR_REASONS; i++) {
                    idle_ver("[%d]dpidle_block_cnt[%s]=%lu\n", i, reason_name[i],
                            dpidle_block_cnt[i]);
                }

                for (i = 0; i < NR_GRPS; i++) {
                    idle_ver("[%02d]dpidle_condition_mask[%-8s]=0x%08x\t\t"
                            "dpidle_block_mask[%-8s]=0x%08x\n", i,
                            grp_get_name(i), dpidle_condition_mask[i],
                            grp_get_name(i), dpidle_block_mask[i]);
                }
                memset(dpidle_block_cnt, 0, sizeof(dpidle_block_cnt));
                dpidle_block_prev_time = idle_get_current_time_ms();

            }


        }
        dpidle_block_cnt[reason]++;
        return false;
    } else {
        dpidle_block_prev_time = idle_get_current_time_ms();
        return true;
    }

}

void spm_dpidle_before_wfi(void)
{
    if (TRUE == mt_dpidle_chk_golden)
    {
    //FIXME:
#if 0
        mt_power_gs_dump_dpidle();
#endif
    }
    bus_dcm_enable();
    clkmgr_faudintbus_pll2sq();
    //clkmux_sel(MT_MUX_AUDINTBUS, 0, "Deepidle"); //select 26M


#ifdef CONFIG_SMP
    dpidle_timer_left2 = localtimer_get_counter();

    if( (int)dpidle_timer_left2 <=0 )
        gpt_set_cmp(idle_gpt, 1);//Trigger GPT4 Timerout imediately
    else
        gpt_set_cmp(idle_gpt, dpidle_timer_left2);

    start_gpt(idle_gpt);
#else
    gpt_get_cnt(idle_gpt, &dpidle_timer_left2);
#endif
}

void spm_dpidle_after_wfi(void)
{
#ifdef CONFIG_SMP
        //if (gpt_check_irq(GPT4)) {
        if (gpt_check_and_ack_irq(idle_gpt)) {
            /* waked up by WAKEUP_GPT */
            localtimer_set_next_event(1);
        } else {
            /* waked up by other wakeup source */
            unsigned int cnt, cmp;
            gpt_get_cnt(idle_gpt, &cnt);
            gpt_get_cmp(idle_gpt, &cmp);
            if (unlikely(cmp < cnt)) {
                idle_err("[%s]GPT%d: counter = %10u, compare = %10u\n", __func__,
                        idle_gpt + 1, cnt, cmp);
                BUG();
            }

            localtimer_set_next_event(cmp-cnt);
            stop_gpt(idle_gpt);
            //GPT_ClearCount(WAKEUP_GPT);
        }
#endif

    //clkmux_sel(MT_MUX_AUDINTBUS, 1, "Deepidle"); //mainpll
    clkmgr_faudintbus_sq2pll();
    bus_dcm_disable();

    dpidle_cnt[0]++;
}

/************************************************
 * slow idle part
 ************************************************/
static DEFINE_MUTEX(slidle_locked);

static void enable_slidle_by_mask(int grp, unsigned int mask)
{
    mutex_lock(&slidle_locked);
    slidle_condition_mask[grp] &= ~mask;
    mutex_unlock(&slidle_locked);
}

static void disable_slidle_by_mask(int grp, unsigned int mask)
{
    mutex_lock(&slidle_locked);
    slidle_condition_mask[grp] |= mask;
    mutex_unlock(&slidle_locked);
}

void enable_slidle_by_bit(int id)
{
    int grp = id / 32;
    unsigned int mask = 1U << (id % 32);
    BUG_ON(INVALID_GRP_ID(grp));
    enable_slidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(enable_slidle_by_bit);

void disable_slidle_by_bit(int id)
{
    int grp = id / 32;
    unsigned int mask = 1U << (id % 32);
    BUG_ON(INVALID_GRP_ID(grp));
    disable_slidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(disable_slidle_by_bit);

#if EN_PTP_OD
extern u32 ptp_data[3];
#endif

static bool slidle_can_enter(void)
{
    int reason = NR_REASONS;

    if ((atomic_read(&is_in_hotplug) == 1)||(num_online_cpus() != 1)) {
        reason = BY_CPU;
        goto out;
    }

    memset(slidle_block_mask, 0, NR_GRPS * sizeof(unsigned int));
    if (!clkmgr_idle_can_enter(slidle_condition_mask, slidle_block_mask)) {
        reason = BY_CLK;
        goto out;
    }

#if EN_PTP_OD
    if (ptp_data[0]) {
        reason = BY_OTH;
        goto out;
    }
#endif

out:
    if (reason < NR_REASONS) {
        slidle_block_cnt[reason]++;
        return false;
    } else {
        return true;
    }
}

static void slidle_before_wfi(int cpu)
{
    mt_dcm_topckg_enable();
}

static void slidle_after_wfi(int cpu)
{
    mt_dcm_topckg_disable();
    slidle_cnt[cpu]++;
}

static void go_to_slidle(int cpu)
{
    slidle_before_wfi(cpu);

    dsb();
    __asm__ __volatile__("wfi" ::: "memory");

    slidle_after_wfi(cpu);
}

/************************************************
 * regular idle part
 ************************************************/
static void rgidle_before_wfi(int cpu)
{

}

static void rgidle_after_wfi(int cpu)
{
    rgidle_cnt[cpu]++;
}

static void noinline go_to_rgidle(int cpu)
{
    rgidle_before_wfi(cpu);
    isb();
    dsb();
    __asm__ __volatile__("wfi" ::: "memory");

    rgidle_after_wfi(cpu);
}

/************************************************
 * idle task flow part
 ************************************************/
extern bool mtk_gpu_sodi_entry(void); 
extern bool mtk_gpu_sodi_exit(void);

static inline void soidle_pre_handler(void)
{
    //stop Mali dvfs_callback timer
    if (!mtk_gpu_sodi_entry())
    {
        idle_ver("not stop GPU timer in SODI\n");
    }
    
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
    mtkts_wmt_cancel_thermal_timer();
#endif
#endif
}

static inline void soidle_post_handler(void)
{
    //restart Mali dvfs_callback timer
    if (!mtk_gpu_sodi_exit())
    {
        idle_ver("not restart GPU timer outside SODI\n");
    }
    
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
    mtkts_wmt_start_thermal_timer();
#endif
#endif
}
/*
 * xxidle_handler return 1 if enter and exit the low power state
 */

static u32 slp_spm_SODI_flags = {
	0//SPM_MD_VRF18_DIS
};

//#define DEFAULT_MMP_ENABLE
#ifdef DEFAULT_MMP_ENABLE
extern void MMProfileEnable(int enable);
extern void MMProfileStart(int start);

static SODI_MMP_Events_t SODI_MMP_Events;

void init_sodi_mmp_events(void)
{
	if (SODI_MMP_Events.SODI == 0)
	{
		SODI_MMP_Events.SODI = MMProfileRegisterEvent(MMP_RootEvent, "SODI");
		SODI_MMP_Events.sodi_enable = MMProfileRegisterEvent(SODI_MMP_Events.SODI, "sodi_enable");
		SODI_MMP_Events.self_refresh_cnt = MMProfileRegisterEvent(SODI_MMP_Events.SODI, "self_refresh_cnt");
		SODI_MMP_Events.sodi_status  = MMProfileRegisterEvent(SODI_MMP_Events.SODI, "sodi_status");				  
		
		MMProfileEnableEventRecursive(SODI_MMP_Events.SODI, 1);
		MMProfileEnableEventRecursive(SODI_MMP_Events.sodi_enable, 1);       
		MMProfileEnableEventRecursive(SODI_MMP_Events.self_refresh_cnt, 1);
		MMProfileEnableEventRecursive(SODI_MMP_Events.sodi_status, 1);
	}
}

SODI_MMP_Events_t * sodi_mmp_get_events(void)
{
    return &SODI_MMP_Events;
}

void sodi_mmp_init(void)
{
    printk("sodi_mmp_init\n");
    MMProfileEnable(1);
    init_sodi_mmp_events();
    MMProfileStart(1);
}
#endif //DEFAULT_MMP_ENABLE

#ifdef SPM_SODI_PROFILE_TIME
unsigned int soidle_profile[4];
#endif
static inline int soidle_handler(int cpu)
{
    if (idle_switch[IDLE_TYPE_SO]) {
#ifdef SPM_SODI_PROFILE_TIME
        gpt_get_cnt(SPM_SODI_PROFILE_APXGPT,&soidle_profile[0]);
#endif
        if (soidle_can_enter(cpu)) {
            
            soidle_pre_handler();
            
            #ifdef DEFAULT_MMP_ENABLE
			MMProfileLogEx(sodi_mmp_get_events()->sodi_enable, MMProfileFlagStart, 0, 0);
			#endif //DEFAULT_MMP_ENABLE
			
            spm_go_to_sodi(slp_spm_SODI_flags, 0);
            
            #ifdef DEFAULT_MMP_ENABLE
            MMProfileLogEx(sodi_mmp_get_events()->sodi_enable, MMProfileFlagEnd, 0, spm_read(SPM_PCM_PASR_DPD_3));
			#endif //DEFAULT_MMP_ENABLE

            soidle_post_handler();

            #if 0 //removed unused log
            #ifdef CONFIG_SMP
            idle_ver("SO:timer_left=%d, timer_left2=%d, delta=%d\n",
                soidle_timer_left, soidle_timer_left2, soidle_timer_left-soidle_timer_left2);
            #else
            idle_ver("SO:timer_left=%d, timer_left2=%d, delta=%d,timeout val=%d\n",
                soidle_timer_left, soidle_timer_left2, soidle_timer_left2-soidle_timer_left,soidle_timer_cmp-soidle_timer_left);
            #endif
            #endif
            
            #if 0 //for DVT test only
            idle_switch[IDLE_TYPE_SO] = 0;
            #endif

#ifdef SPM_SODI_PROFILE_TIME
            gpt_get_cnt(SPM_SODI_PROFILE_APXGPT,&soidle_profile[3]);
            idle_ver("SODI: cpu_freq:%u, 1=>2:%u, 2=>3:%u, 3=>4:%u\n", mt_cpufreq_get_cur_freq(0),
                 soidle_profile[1]-soidle_profile[0], soidle_profile[2]-soidle_profile[1], soidle_profile[3]-soidle_profile[2]);
#endif
            return 1;
        }
    }

    return 0;
}
u32 slp_spm_deepidle_flags = {
	0//SPM_MD_VRF18_DIS//SPM_CPU_PDN_DIS
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
    mtkts_wmt_cancel_thermal_timer();
#endif
#endif
#if 0
    // disable gpu dvfs timer
    mtk_enable_gpu_dvfs_timer(false);

    // disable cpu dvfs timer
    hp_enable_timer(0);
#endif

}
static inline void dpidle_post_handler(void)
{
#if 0
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
    mtkts_wmt_start_thermal_timer();
#endif
#endif
}
#ifdef SPM_DEEPIDLE_PROFILE_TIME
unsigned int dpidle_profile[4];
#endif
static inline int dpidle_handler(int cpu)
{
    int ret = 0;
    if (idle_switch[IDLE_TYPE_DP]) {
#ifdef SPM_DEEPIDLE_PROFILE_TIME
        gpt_get_cnt(SPM_PROFILE_APXGPT,&dpidle_profile[0]);
#endif
        if (dpidle_can_enter()) {
            dpidle_pre_handler();
            spm_go_to_dpidle(slp_spm_deepidle_flags, 0);
            dpidle_post_handler();
            ret = 1;

#ifdef CONFIG_SMP
            idle_ver("DP:timer_left=%d, timer_left2=%d, delta=%d\n",
                dpidle_timer_left, dpidle_timer_left2, dpidle_timer_left-dpidle_timer_left2);
#else
            idle_ver("DP:timer_left=%d, timer_left2=%d, delta=%d, timeout val=%d\n",
                dpidle_timer_left, dpidle_timer_left2, dpidle_timer_left2-dpidle_timer_left,dpidle_timer_cmp-dpidle_timer_left);
#endif
#ifdef SPM_DEEPIDLE_PROFILE_TIME
            gpt_get_cnt(SPM_PROFILE_APXGPT,&dpidle_profile[3]);
            idle_ver("1:%u, 2:%u, 3:%u, 4:%u\n",
                 dpidle_profile[0], dpidle_profile[1], dpidle_profile[2],dpidle_profile[3]);
#endif
        }
    }

    return ret;
}

static inline int slidle_handler(int cpu)
{
    int ret = 0;
    if (idle_switch[IDLE_TYPE_SL]) {
        if (slidle_can_enter()) {
            go_to_slidle(cpu);
            ret = 1;
        }
    }

    return ret;
}

static inline int rgidle_handler(int cpu)
{
    int ret = 0;
    if (idle_switch[IDLE_TYPE_RG]) {
        go_to_rgidle(cpu);
        ret = 1;
    }

    return ret;
}

static int (*idle_handlers[NR_TYPES])(int) = {
    dpidle_handler,
    soidle_handler,
    slidle_handler,
    rgidle_handler,
};

void arch_idle(void)
{
    int cpu = smp_processor_id();
    int i;

    for (i = 0; i < NR_TYPES; i++) {
        if (idle_handlers[i](cpu))
            break;
    }

}

#define idle_attr(_name)                         \
static struct kobj_attribute _name##_attr = {   \
    .attr = {                                   \
        .name = __stringify(_name),             \
        .mode = 0644,                           \
    },                                          \
    .show = _name##_show,                       \
    .store = _name##_store,                     \
}

extern struct kobject *power_kobj;

static ssize_t soidle_state_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
    int len = 0;
    char *p = buf;

    int cpus, reason, i;

    p += sprintf(p, "*********** Screen on idle state ************\n");
    p += sprintf(p, "soidle_time_critera=%u\n", soidle_time_critera);

    for (cpus = 0; cpus < nr_cpu_ids; cpus++) {
        p += sprintf(p, "cpu:%d\n", cpus);
        for (reason = 0; reason < NR_REASONS; reason++) {
            p += sprintf(p, "[%d]soidle_block_cnt[%s]=%lu\n", reason,
                    reason_name[reason], soidle_block_cnt[cpus][reason]);
        }
        p += sprintf(p, "\n");
    }

    for (i = 0; i < NR_GRPS; i++) {
        p += sprintf(p, "[%02d]soidle_condition_mask[%-8s]=0x%08x\t\t"
                "soidle_block_mask[%-8s]=0x%08x\n", i,
                grp_get_name(i), soidle_condition_mask[i],
                grp_get_name(i), soidle_block_mask[i]);
    }

    p += sprintf(p, "soidle_bypass_cg=%u\n", soidle_by_pass_cg);

    p += sprintf(p, "\n********** soidle command help **********\n");
    p += sprintf(p, "soidle help:   cat /sys/power/soidle_state\n");
    p += sprintf(p, "switch on/off: echo [soidle] 1/0 > /sys/power/soidle_state\n");
    p += sprintf(p, "en_so_by_bit:  echo enable id > /sys/power/soidle_state\n");
    p += sprintf(p, "dis_so_by_bit: echo disable id > /sys/power/soidle_state\n");
    p += sprintf(p, "modify tm_cri: echo time value(dec) > /sys/power/soidle_state\n");
    p += sprintf(p, "bypass cg:     echo bypass 1/0 > /sys/power/soidle_state\n");

    len = p - buf;
    return len;
}

static ssize_t soidle_state_store(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf, size_t n)
{
    char cmd[32];
    int param;

    if (sscanf(buf, "%s %d", cmd, &param) == 2) {
        if (!strcmp(cmd, "soidle")) {
            idle_switch[IDLE_TYPE_SO] = param;
        } else if (!strcmp(cmd, "enable")) {
            enable_soidle_by_bit(param);
        } else if (!strcmp(cmd, "disable")) {
            disable_soidle_by_bit(param);
        } else if (!strcmp(cmd, "time")) {
            soidle_time_critera = param;
        } else if (!strcmp(cmd, "bypass")) {
            soidle_by_pass_cg = param;
        }
        return n;
    } else if (sscanf(buf, "%d", &param) == 1) {
        idle_switch[IDLE_TYPE_SO] = param;
        return n;
    }

    return -EINVAL;
}
idle_attr(soidle_state);

static ssize_t dpidle_state_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
    int len = 0;
    char *p = buf;

    int i;

    p += sprintf(p, "*********** deep idle state ************\n");
    p += sprintf(p, "dpidle_time_critera=%u\n", dpidle_time_critera);

    for (i = 0; i < NR_REASONS; i++) {
        p += sprintf(p, "[%d]dpidle_block_cnt[%s]=%lu\n", i, reason_name[i],
                dpidle_block_cnt[i]);
    }

    p += sprintf(p, "\n");

    for (i = 0; i < NR_GRPS; i++) {
        p += sprintf(p, "[%02d]dpidle_condition_mask[%-8s]=0x%08x\t\t"
                "dpidle_block_mask[%-8s]=0x%08x\n", i,
                grp_get_name(i), dpidle_condition_mask[i],
                grp_get_name(i), dpidle_block_mask[i]);
    }

    p += sprintf(p, "dpidle_bypass_cg=%u\n", dpidle_by_pass_cg);

    p += sprintf(p, "\n*********** dpidle command help  ************\n");
    p += sprintf(p, "dpidle help:   cat /sys/power/dpidle_state\n");
    p += sprintf(p, "switch on/off: echo [dpidle] 1/0 > /sys/power/dpidle_state\n");
    p += sprintf(p, "cpupdn on/off: echo cpupdn 1/0 > /sys/power/dpidle_state\n");
    p += sprintf(p, "en_dp_by_bit:  echo enable id > /sys/power/dpidle_state\n");
    p += sprintf(p, "dis_dp_by_bit: echo disable id > /sys/power/dpidle_state\n");
    p += sprintf(p, "modify tm_cri: echo time value(dec) > /sys/power/dpidle_state\n");
    p += sprintf(p, "bypass cg:     echo bypass 1/0 > /sys/power/dpidle_state\n");

    len = p - buf;
    return len;
}

static ssize_t dpidle_state_store(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf, size_t n)
{
    char cmd[32];
    int param;

    if (sscanf(buf, "%s %d", cmd, &param) == 2) {
        if (!strcmp(cmd, "dpidle")) {
            idle_switch[IDLE_TYPE_DP] = param;
        } else if (!strcmp(cmd, "enable")) {
            enable_dpidle_by_bit(param);
        } else if (!strcmp(cmd, "disable")) {
            disable_dpidle_by_bit(param);
        } else if (!strcmp(cmd, "time")) {
            dpidle_time_critera = param;
        }else if (!strcmp(cmd, "bypass")) {
            dpidle_by_pass_cg = param;
        }
        return n;
    } else if (sscanf(buf, "%d", &param) == 1) {
        idle_switch[IDLE_TYPE_DP] = param;
        return n;
    }

    return -EINVAL;
}
idle_attr(dpidle_state);

static ssize_t slidle_state_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
    int len = 0;
    char *p = buf;

    int i;

    p += sprintf(p, "*********** slow idle state ************\n");
    for (i = 0; i < NR_REASONS; i++) {
        p += sprintf(p, "[%d]slidle_block_cnt[%s]=%lu\n",
                i, reason_name[i], slidle_block_cnt[i]);
    }

    p += sprintf(p, "\n");

    for (i = 0; i < NR_GRPS; i++) {
        p += sprintf(p, "[%02d]slidle_condition_mask[%-8s]=0x%08x\t\t"
                "slidle_block_mask[%-8s]=0x%08x\n", i,
                grp_get_name(i), slidle_condition_mask[i],
                grp_get_name(i), slidle_block_mask[i]);
    }

    p += sprintf(p, "\n********** slidle command help **********\n");
    p += sprintf(p, "slidle help:   cat /sys/power/slidle_state\n");
    p += sprintf(p, "switch on/off: echo [slidle] 1/0 > /sys/power/slidle_state\n");

    len = p - buf;
    return len;
}

static ssize_t slidle_state_store(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf, size_t n)
{
    char cmd[32];
    int param;

    if (sscanf(buf, "%s %d", cmd, &param) == 2) {
        if (!strcmp(cmd, "slidle")) {
            idle_switch[IDLE_TYPE_SL] = param;
        } else if (!strcmp(cmd, "enable")) {
            enable_slidle_by_bit(param);
        } else if (!strcmp(cmd, "disable")) {
            disable_slidle_by_bit(param);
        }
        return n;
    } else if (sscanf(buf, "%d", &param) == 1) {
        idle_switch[IDLE_TYPE_SL] = param;
        return n;
    }

    return -EINVAL;
}
idle_attr(slidle_state);

static ssize_t rgidle_state_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
    int len = 0;
    char *p = buf;

    p += sprintf(p, "*********** regular idle state ************\n");
    p += sprintf(p, "\n********** rgidle command help **********\n");
    p += sprintf(p, "rgidle help:   cat /sys/power/rgidle_state\n");
    p += sprintf(p, "switch on/off: echo [rgidle] 1/0 > /sys/power/rgidle_state\n");

    len = p - buf;
    return len;
}

static ssize_t rgidle_state_store(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf, size_t n)
{
    char cmd[32];
    int param;

    if (sscanf(buf, "%s %d", cmd, &param) == 2) {
        if (!strcmp(cmd, "rgidle")) {
            idle_switch[IDLE_TYPE_RG] = param;
        }
        return n;
    } else if (sscanf(buf, "%d", &param) == 1) {
        idle_switch[IDLE_TYPE_RG] = param;
        return n;
    }

    return -EINVAL;
}
idle_attr(rgidle_state);

static ssize_t idle_state_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
    int len = 0;
    char *p = buf;

    int i;

    p += sprintf(p, "********** idle state dump **********\n");

    for (i = 0; i < nr_cpu_ids; i++) {
        p += sprintf(p, "soidle_cnt[%d]=%lu, dpidle_cnt[%d]=%lu, "
                "slidle_cnt[%d]=%lu, rgidle_cnt[%d]=%lu\n",
                i, soidle_cnt[i], i, dpidle_cnt[i],
                i, slidle_cnt[i], i, rgidle_cnt[i]);
    }

    p += sprintf(p, "\n********** variables dump **********\n");
    for (i = 0; i < NR_TYPES; i++) {
        p += sprintf(p, "%s_switch=%d, ", idle_name[i], idle_switch[i]);
    }
    p += sprintf(p, "\n");

    p += sprintf(p, "\n********** idle command help **********\n");
    p += sprintf(p, "status help:   cat /sys/power/idle_state\n");
    p += sprintf(p, "switch on/off: echo switch mask > /sys/power/idle_state\n");

    p += sprintf(p, "soidle help:   cat /sys/power/soidle_state\n");
    p += sprintf(p, "dpidle help:   cat /sys/power/dpidle_state\n");
    p += sprintf(p, "slidle help:   cat /sys/power/slidle_state\n");
    p += sprintf(p, "rgidle help:   cat /sys/power/rgidle_state\n");

    len = p - buf;
    return len;
}

static ssize_t idle_state_store(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf, size_t n)
{
    char cmd[32];
    int idx;
    int param;

    if (sscanf(buf, "%s %x", cmd, &param) == 2) {
        if (!strcmp(cmd, "switch")) {
            for (idx = 0; idx < NR_TYPES; idx++) {
                idle_switch[idx] = (param & (1U << idx)) ? 1 : 0;
            }
        }
        return n;
    }

    return -EINVAL;
}
idle_attr(idle_state);

void mt_idle_init(void)
{

    int err = 0;
    int i = 0;

    idle_info("[%s]entry!!\n", __func__);

    arm_pm_idle = arch_idle;

    err = request_gpt(idle_gpt, GPT_ONE_SHOT, GPT_CLK_SRC_SYS, GPT_CLK_DIV_1,
                0, NULL, GPT_NOAUTOEN);
    if (err) {
        idle_info("[%s]fail to request GPT%d\n", __func__,idle_gpt+1);
    }

    err = 0;

    for(i=0;i<NR_CPUS;i++){
        err |= cpu_xgpt_register_timer(i,NULL);
    }

    if (err) {
        idle_info("[%s]fail to request cpuxgpt\n", __func__);
    }
#if defined(CONFIG_PM)
    err = sysfs_create_file(power_kobj, &idle_state_attr.attr);
    err |= sysfs_create_file(power_kobj, &soidle_state_attr.attr);
    err |= sysfs_create_file(power_kobj, &dpidle_state_attr.attr);
    err |= sysfs_create_file(power_kobj, &slidle_state_attr.attr);
    err |= sysfs_create_file(power_kobj, &rgidle_state_attr.attr);
    if (err) {
        idle_err("[%s]: fail to create sysfs\n", __func__);
    }
#endif
}

module_param(mt_idle_chk_golden, bool, 0644);
module_param(mt_dpidle_chk_golden, bool, 0644);
