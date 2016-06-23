#include <linux/kernel.h>
#include <linux/init.h>
#if defined (__KERNEL__)	//|| !defined (__CTP__)
#include <linux/jiffies.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/irqchip/arm-gic.h>

#include <asm/psci.h>
// #include <asm/smp_scu.h>
// #include <asm/cpuidle.h>
#endif //#if !defined (__CTP__)

#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/tlbflush.h>
#include <asm/memory.h>
#include <asm/cacheflush.h>
#include <asm/neon.h>

// #include <asm/system.h>
#if !defined (__KERNEL__)	//|| defined (__CTP__)
#include "reg_base.H"
#include "mt_dormant.h"
#include "mt_cpuidle.h"
#include "smp.h"
#include "mt_spm.h"
#include "irq.h"
#include "sync_write.h"
//#include "mt_dbg_v71.h"
#include "gic.h"
#else //#if !defined (__KERNEL__) //|| defined(__CTP__)
// #include <asm/idmap.h>
#include <asm/irqflags.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_dormant.h>
#include <mach/mt_cpuidle.h>
#include <mach/mt_spm.h>
#include <mach/mt_spm_idle.h>
// #include <mach/smp.h>
#include <mach/mt_irq.h>
#include <mach/sync_write.h>
// #include <mach/mt_dbg_v71.h>
#include <mach/mt_boot.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <mach/mt_dbg.h>
#endif //#if !defined (__KERNEL__) //|| defined(__CTP__)

#if defined(MT_DORMANT_UT)
#include <mach/smp.h>
#endif // #if defined(MT_DORMANT_UT)

#ifdef CONFIG_MTK_RAM_CONSOLE
#include <mach/mt_secure_api.h>
#endif

/*********************************
 * macro
 **********************************/

#define DBGAPB_CORE_OFFSET (0x00100000)

#if defined (__KERNEL__)
//#define CA15L_CONFIG_BASE 0xf0200200

static unsigned long dbgapb_base;
static unsigned long mcucfg_base;
#if defined(CONFIG_ARCH_MT6735) || defined(CONFIG_ARCH_MT6735M)
static unsigned long biu_base;
#endif

static unsigned long infracfg_ao_base;
static unsigned long gic_id_base;
static unsigned long gic_ci_base;

static unsigned int kp_irq_bit;
static unsigned int conn_wdt_irq_bit;
static unsigned int lowbattery_irq_bit;
static unsigned int md1_wdt_bit;
#if defined(CONFIG_ARCH_MT6735) && defined(CONFIG_MTK_C2K_SUPPORT)
static unsigned int c2k_wdt_bit;
#endif

#define DBGAPB_NODE      "mediatek,DBG_DEBUG"
#define MCUCFG_NODE      "mediatek,MCUCFG"
#if defined(CONFIG_ARCH_MT6735) || defined(CONFIG_ARCH_MT6735M)
#define BIU_NODE         "mediatek,MCU_BIU"
#endif
#define INFRACFG_AO_NODE "mediatek,INFRACFG_AO"
#define GIC_NODE         "mtk,mt-gic"

#define KP_NODE          "mediatek,KP"
#define CONSYS_NODE      "mediatek,CONSYS"
#define AUXADC_NODE      "mediatek,AUXADC"
#define MDCLDMA_NODE     "mediatek,MDCLDMA"
#if defined(CONFIG_ARCH_MT6735) && defined(CONFIG_MTK_C2K_SUPPORT)
#define MDC2K_NODE       "mediatek,MDC2K"
#endif

#define DMT_MP0_DBGAPB_BASE       (dbgapb_base)
#define DMT_MP1_DBGAPB_BASE       (dbgapb_base + (DBGAPB_CORE_OFFSET*4))
#define DMT_MCUCFG_BASE           (mcucfg_base)	//0x1020_0000
#if defined(CONFIG_ARCH_MT6735) || defined(CONFIG_ARCH_MT6735M)
#define DMT_BIU_BASE              (biu_base)	//0x1030_0000
#endif

#define DMT_INFRACFG_AO_BASE      (infracfg_ao_base)	//0x1000_1000
#define DMT_GIC_CPU_BASE          (gic_ci_base)
#define DMT_GIC_DIST_BASE         (gic_id_base)

#define DMT_KP_IRQ_BIT            (kp_irq_bit)
#define DMT_CONN_WDT_IRQ_BIT      (conn_wdt_irq_bit)
#define DMT_LOWBATTERY_IRQ_BIT    (lowbattery_irq_bit)
#define DMT_MD1_WDT_BIT           (md1_wdt_bit)
#if defined(CONFIG_ARCH_MT6735) && defined(CONFIG_MTK_C2K_SUPPORT)
#define DMT_C2K_WDT_BIT           (c2k_wdt_bit)
#endif

#else //#if defined (__KERNEL__)
//#define CA15L_CONFIG_BASE 0x10200200
#define DMT_MP0_DBGAPB_BASE    (0x10810000)
#define DMT_MP1_DBGAPB_BASE    (0x10C10000)
#define DMT_MCUCFG_BASE        (0x10200000)	//0x1020_0000
#if defined(CONFIG_ARCH_MT6735) || defined(CONFIG_ARCH_MT6735M)
#define DMT_BIU_BASE           (0x10300000)	//0x1030_0000
#endif

#define DMT_INFRACFG_AO_BASE   (0x10001000)	//0x1000_1000

typedef unsigned long long u64;
#define ____cacheline_aligned __attribute__((aligned(8)))
#define __weak __attribute__((weak))
#define __naked __attribute__((naked))
typedef enum {
	CHIP_SW_VER_01 = 0x0000,
	CHIP_SW_VER_02 = 0x0001
} CHIP_SW_VER;
#define local_fiq_enable() do {} while(0)
#endif //#if defined (__KERNEL__)

#define MP0_CA7L_CACHE_CONFIG   (DMT_MCUCFG_BASE + 0)
#define MP1_CA7L_CACHE_CONFIG   (DMT_MCUCFG_BASE + 0x200)
#define L2RSTDISABLE 		(1 << 4)

#define MP0_AXI_CONFIG          (DMT_MCUCFG_BASE + 0x2C)
#define MP1_AXI_CONFIG          (DMT_MCUCFG_BASE + 0x22C)
#define ACINACTM                (1 << 4)

#if defined(CONFIG_ARCH_MT6735) || defined(CONFIG_ARCH_MT6735M)
#define BIU_CONTROL		(DMT_BIU_BASE + 0x0000)	//0x10300000
#define TLB_ULTRA_EN            (1 << 8)
#define DCM_EN                  (1 << 1)
#define CMD_QUEUE_EN            (1 << 0)
#endif

#ifdef CONFIG_MTK_RAM_CONSOLE
#define CPU_DORMANT_AEE_RR_REC 1
#else
#define CPU_DORMANT_AEE_RR_REC 0
#endif

#define reg_read(addr)          __raw_readl(IOMEM(addr))	//(*(volatile unsigned long *)(addr))
#define reg_write(addr, val)    mt_reg_sync_writel(val, addr)
#define _and(a, b) 	((a) & (b))
#define _or(a, b) 	((a) | (b))
#define _aor(a, b, c) 	_or(_and(a, b), (c))

#define read_cntpct()					\
	({						\
		register u64 cntpct;			\
		__asm__ __volatile__(			\
			"MRS  %x0, CNTPCT_EL0 \n\t"	\
			:"=r"(cntpct)			\
			:				\
			:"memory");			\
		cntpct;					\
	})

#define read_cntpctl()					\
	({						\
		register u32 cntpctl;			\
		__asm__ __volatile__(			\
			"MRS %x0, CNTP_CTL_EL0 \n\t"    \
			:"=r"(cntpctl)			\
			:				\
			:"memory");			\
		cntpctl;				\
	})

#define write_cntpctl(cntpctl)				\
	do {						\
                register u32 t = (u32)cntpctl;          \
		__asm__ __volatile__(			\
			"MSR CNTP_CTL_EL0, %x0 \n\t"    \
			:				\
			:"r"(t));			\
	} while (0)

/*********************************
 * macro for log
 **********************************/
#define CPU_DORMANT_LOG_WITH_NONE                           0
#define CPU_DORMANT_LOG_WITH_DEBUG                          1

#define CPU_DORMANT_LOG_PRINT CPU_DORMANT_LOG_WITH_NONE

#if (CPU_DORMANT_LOG_PRINT == CPU_DORMANT_LOG_WITH_NONE)
#define CPU_DORMANT_INFO(fmt, args...)          do { } while(0)
#elif (CPU_DORMANT_LOG_PRINT == CPU_DORMANT_LOG_WITH_DEBUG)
#define CPU_DORMANT_INFO(fmt, args...)		do { pr_debug("[Power/cpu_dormant] "fmt, ##args); } while(0)
#endif

#define MT_DORMANT_DEBUG

#if defined (MT_DORMANT_DEBUG)
#define SENTINEL_CHECK(data, p) BUG_ON((unsigned long)(p) > ((unsigned long)(&data) + sizeof(data)))
#else //#if defined (MT_DORMANT_DEBGU)
#define SENTINEL_CHECK(a, b) do{} while(0)
#endif //#if defined (MT_DORMANT_DEBGU)

/* debug facility */
#define DEBUG_DORMANT_BYPASS (1==0)

// #define TSLOG_ENABLE
#if defined (TSLOG_ENABLE)
#define TSLOG(a, b) do { (a) = (b); } while(0)
#else
#define TSLOG(a, b) do {} while(0)
#endif

/*********************************
 * struct
 **********************************/
typedef struct dmnt_cpu_context {
	unsigned int banked_regs[32];
	unsigned int pmu_data[20];
	unsigned int vfp_data[32 * 2 + 8];
	unsigned long timer_data[8];	/* Global timers if the NS world has access to them */
	volatile u64 timestamp[5];
	unsigned int count, rst, abt, brk;
} core_context;

#define MAX_CORES (4)		//core num per cluster

typedef struct cluster_context {
	core_context core[MAX_CORES] ____cacheline_aligned;
	unsigned long dbg_data[40];
	int l2rstdisable;
	int l2rstdisable_rfcnt;
} cluster_context;

#define MAX_CLUSTER (2)
/*
 * Top level structure to hold the complete context of a multi cluster system
 */
typedef struct system_context {
	cluster_context cluster[2];
	struct _data_poc {
		void (*cpu_resume_phys) (void);	// this is referenced by cpu_resume_wrapper
		unsigned long l2ectlr, l2actlr;
		CHIP_SW_VER chip_ver;
		unsigned long *cpu_dormant_aee_rr_rec;
	} poc ____cacheline_aligned;
} system_context;

#if CPU_DORMANT_AEE_RR_REC
extern phys_addr_t sleep_aee_rec_cpu_dormant;
extern unsigned long *sleep_aee_rec_cpu_dormant_va;
extern unsigned long *aee_rr_rec_cpu_dormant(void);
extern unsigned long *aee_rr_rec_cpu_dormant_pa(void);
#endif

/*********************************
 * extern
 **********************************/
void __disable_dcache__inner_flush_dcache_L1__inner_clean_dcache_L2(void);
void __disable_dcache__inner_flush_dcache_L1__inner_flush_dcache_L2(void);
void __disable_dcache__inner_flush_dcache_L1(void);

unsigned int *mt_save_banked_registers(unsigned int *container)
{
	return container;
}

void mt_restore_banked_registers(unsigned int *container)
{
}

extern void cpu_wake_up(void);
extern void __disable_dcache(void);
extern void __enable_dcache(void);
extern void __disable_icache(void);
extern void __enable_icache(void);
extern void v7_flush_kern_dcache_louis(void);
extern void v7_flush_kern_dcache_all(void);
extern void cpu_resume(void);
extern void trace_start_dormant(void);

//check COREn IRQ       
//check COREn FIQ       
#define SPM_CORE_ID() core_idx()
#define SPM_IS_CPU_IRQ_OCCUR(core_id)                                   \
        ({                                                              \
                (!!(spm_read(SPM_SLEEP_WAKEUP_MISC) & ((0x101<<(core_id))))); \
        })

#if CPU_DORMANT_AEE_RR_REC
#define DORMANT_LOG(cid,pattern) do {						\
	if ( dormant_data[0].poc.cpu_dormant_aee_rr_rec != 0) {			\
		(dormant_data[0].poc.cpu_dormant_aee_rr_rec)[cid] = pattern;	\
	}                                                       		\
} while(0)

#else
#define DORMANT_LOG(cid,pattern) do { } while(0)

#endif

/*********************************
 * glabal variable
 **********************************/
/*
 * Top level structure which encapsulates the context of the entire
 * Kingfisher system
 */

system_context dormant_data[1];
volatile int debug_dormant_bypass = 0;
static int mt_dormant_initialized = 0;

/*********************************
 * function
 **********************************/

#define read_mpidr()							\
	({								\
		register u64 ret;                                       \
		__asm__ __volatile__ ("MRS %x0, MPIDR_EL1  \n\t"        \
				      :"=r"(ret));			\
		ret;							\
	})

//inline unsigned read_midr(void)
#define read_midr()							\
	({								\
		register u32 ret;				\
		__asm__ __volatile__ ("MRS %x0, MIDR_EL1 \n\t" \
				      :"=r"(ret));			\
		ret;							\
	})

#define CA12_TYPEID     0x410FC0D0
#define CA17_TYPEID     0x410FC0E0
#define CA7_TYPEID      0x410FC070
#define CA7L_TYPEID     0x410FD030
#define CA53_TYPEID     0x410FD030
#define CPU_TYPEID_MASK 0xfffffff0

//inline int is_cpu_type(int type) 
#define is_cpu_type(type)						\
	({								\
		((read_midr() & CPU_TYPEID_MASK) == type) ? 1 : 0;	\
	})

//inline int cpu_id(void)
#define cpu_id()				\
	({					\
		(read_mpidr() & 0x0ff);		\
	})

//inline int cluster_id(void)
#define cluster_id()				\
	({					\
		((read_mpidr() >> 8) & 0x0ff);	\
	})

#define core_idx()                                                      \
	({                                                              \
                int mpidr = read_mpidr();                               \
		((( mpidr & (0x0ff << 8)) >> 6) | (mpidr & 0xff));	\
	})

inline int read_id(int *cpu_id, int *cluster_id)
{
	int mpidr = read_mpidr();

	*cpu_id = mpidr & 0x0f;
	*cluster_id = (mpidr >> 8) & 0x0f;

	return mpidr;
}

#define system_cluster(system, clusterid)	(&((system_context *)system)->cluster[clusterid])
#define cluster_core(cluster, cpuid)	(&((cluster_context *)cluster)->core[cpuid])

void *_get_data(int core_or_cluster)
{
	int cpuid, clusterid;
	cluster_context *cluster;
	core_context *core;

	read_id(&cpuid, &clusterid);

	cluster = system_cluster(dormant_data, clusterid);
	if (core_or_cluster == 1)
		return (void *)cluster;

	core = cluster_core(cluster, cpuid);
	return (void *)core;
}

#define GET_CORE_DATA() ((core_context *)_get_data(0))
#define GET_CLUSTER_DATA() ((cluster_context *)_get_data(1))
#define GET_SYSTEM_DATA() ((system_context *)dormant_data)

/********************/
/* .global save_vfp */
/********************/
unsigned *save_vfp(unsigned int *container)
{
	return container;
}

/***********************/
/* .global restore_vfp */
/***********************/
void restore_vfp(unsigned int *container)
{
	return;
}

/*************************************/
/* .global save_performance_monitors */
/*************************************/
unsigned *save_pmu_context(unsigned *container)
{
	return container;
}

/****************************************/
/* .global restore_performance_monitors */
/****************************************/
void restore_pmu_context(int *container)
{
	return;
}

/***********************************************************************************/
/* @ If r1 is 0, we assume that the OS is not using the Virtualization extensions, */
/* @ and that the warm boot code will set up CNTHCTL correctly. If r1 is non-zero  */
/* @ then CNTHCTL is saved and restored						   */
/* @ CNTP_CVAL will be preserved as it is in the always-on domain.		   */
/***********************************************************************************/
unsigned int *mt_save_generic_timer(unsigned int *container, int sw)
{
	__asm__ __volatile__ (
		" mrs   x3, CNTKCTL_EL1         \n\t"
		" str   x3, [%0, #0]           \n\t"
		" mrs   x2, CNTP_CTL_EL0        \n\t"
		" mrs   x3, CNTP_TVAL_EL0       \n\t"
		" stp   x2, x3, [%0, #8]       \n\t"
		" mrs   x2, CNTV_CTL_EL0        \n\t"
		" mrs   x3, CNTV_TVAL_EL0       \n\t"
		" stp   x2, x3, [%0, #24]!       \n\t"
		: "+r"(container)
		: "r"(sw)
		:"r2", "r3");

	return container;
}

void mt_restore_generic_timer(unsigned int *container, int sw)
{
	__asm__ __volatile__ (
		" ldr   x3, [%0, #0]       \n\t"
		" msr   CNTKCTL_EL1, x3         \n\t"
		" ldp   x2, x3, [%0, #8]       \n\t"
		" msr   CNTP_CTL_EL0, x2        \n\t"
		" msr   CNTP_TVAL_EL0, x3       \n\t"
		" ldp   x2, x3, [%0, #24]      \n\t"
		" msr   CNTV_CTL_EL0, x2       \n\t"
		" msr   CNTV_TVAL_EL0, x3       \n\t"
		:
		:"r"(container), "r"(sw)
		:"r2", "r3");
	
	return;
}

void stop_generic_timer(void)
{
	/*
	 * Disable the timer and mask the irq to prevent
	 * suprious interrupts on this cpu interface. It
	 * will bite us when we come back if we don't. It
	 * will be replayed on the inbound cluster.
	 */
	write_cntpctl(read_cntpctl() & ~1);
	return;
}

void start_generic_timer(void)
{
	write_cntpctl(read_cntpctl() | 1);
	return;
}

struct set_and_clear_regs {
	volatile unsigned int set[32], clear[32];
};

typedef struct {
	volatile unsigned int control;	/* 0x000 */
	const unsigned int controller_type;
	const unsigned int implementer;
	const char padding1[116];
	volatile unsigned int security[32];	/* 0x080 */
	struct set_and_clear_regs enable;	/* 0x100 */
	struct set_and_clear_regs pending;	/* 0x200 */
	struct set_and_clear_regs active;	/* 0x300 */
	volatile unsigned int priority[256];	/* 0x400 */
	volatile unsigned int target[256];	/* 0x800 */
	volatile unsigned int configuration[64];	/* 0xC00 */
	const char padding3[256];	/* 0xD00 */
	volatile unsigned int non_security_access_control[64];	/* 0xE00 */
	volatile unsigned int software_interrupt;	/* 0xF00 */
	volatile unsigned int sgi_clr_pending[4];	/* 0xF10 */
	volatile unsigned int sgi_set_pending[4];	/* 0xF20 */
	const char padding4[176];
	unsigned const int peripheral_id[4];	/* 0xFE0 */
	unsigned const int primecell_id[4];	/* 0xFF0 */
} interrupt_distributor;

typedef struct {
	volatile unsigned int control;	/* 0x00 */
	volatile unsigned int priority_mask;	/* 0x04 */
	volatile unsigned int binary_point;	/* 0x08 */
	volatile unsigned const int interrupt_ack;	/* 0x0c */
	volatile unsigned int end_of_interrupt;	/* 0x10 */
	volatile unsigned const int running_priority;	/* 0x14 */
	volatile unsigned const int highest_pending;	/* 0x18 */
	volatile unsigned int aliased_binary_point;	/* 0x1c */
	volatile unsigned const int aliased_interrupt_ack;	/* 0x20 */
	volatile unsigned int alias_end_of_interrupt;	/* 0x24 */
	volatile unsigned int aliased_highest_pending;	/* 0x28 */
} cpu_interface;

/**
 *       @ This function takes three arguments
 *       @ r0: Destination start address (must be word aligned)
 *       @ r1: Source start address (must be word aligned)
 *       @ r2: Number of words to copy
 *       @ Return value is updated destination pointer (first unwritten word)
 **/
static unsigned int *copy_words(volatile unsigned int *dst,
				volatile unsigned int *src, unsigned int num)
{
	while (num-- > 0) {
		*dst++ = *src++;
	}

	return (unsigned int *)dst;
}

typedef struct ns_gic_cpu_context {
	unsigned int gic_cpu_if_regs[32];	/* GIC context local to the CPU */
	unsigned int gic_dist_if_pvt_regs[32];	/* GIC SGI/PPI context local to the CPU */
	unsigned int gic_dist_if_regs[512];	/* GIC distributor context to be saved by the last cpu. */
} gic_cpu_context;

gic_cpu_context gic_data[1];
#define gic_data_base() ((gic_cpu_context *)&gic_data[0])

/*
 * Saves the GIC CPU interface context
 * Requires 3 words of memory
 */
static void save_gic_interface(u32 * pointer,
			       unsigned long gic_interface_address)
{
	cpu_interface *ci = (cpu_interface *) gic_interface_address;

	pointer[0] = ci->control;
	pointer[1] = ci->priority_mask;
	pointer[2] = ci->binary_point;
	pointer[3] = ci->aliased_binary_point;

	pointer[4] = ci->aliased_highest_pending;

	/* TODO: add nonsecure stuff */

}

/*
 * Saves this CPU's banked parts of the distributor
 * Returns non-zero if an SGI/PPI interrupt is pending (after saving all required context)
 * Requires 19 words of memory
 */
static void save_gic_distributor_private(u32 * pointer,
					 unsigned long gic_distributor_address)
{
	interrupt_distributor *id =
	    (interrupt_distributor *) gic_distributor_address;
	unsigned int *ptr = 0x0;

	/*  Save SGI,PPI enable status */
	*pointer = id->enable.set[0];
	++pointer;
	/*  Save SGI,PPI priority status */
	pointer = copy_words(pointer, id->priority, 8);
	/*  Save SGI,PPI target status */
	pointer = copy_words(pointer, id->target, 8);
	/*  Save just the PPI configurations (SGIs are not configurable) */
	*pointer = id->configuration[1];
	++pointer;
	/*  Save SGI,PPI security status */
	*pointer = id->security[0];
	++pointer;

	/*  Save SGI Non-security status (PPI is read-only) */
	*pointer = id->non_security_access_control[0] & 0x0ffff;
	++pointer;
#if 0
	/*
	 * Private peripheral interrupts need to be replayed on
	 * the destination cpu interface for consistency. This
	 * is the responsibility of the peripheral driver. When
	 * it sees a pending interrupt while saving its context
	 * it should record enough information to recreate the
	 * interrupt while restoring.
	 * We don't save the Pending/Active status and clear it
	 * so that it does not interfere when we are back.
	 */
	/*  Clear PPI pending status */
	id->pending.clear[0] = 0xffffffff;
	id->active.clear[0] = 0xffffffff;
#endif
#if 1
	/*  Save SGI,PPI pending status */
	*pointer = id->pending.set[0];
	++pointer;
#endif
	/*
	 * IPIs are different and can be replayed just by saving
	 * and restoring the set/clear pending registers
	 */
	ptr = pointer;
	copy_words(pointer, id->sgi_set_pending, 4);
	pointer += 8;

	/*
	 * Clear the pending SGIs on this cpuif so that they don't
	 * interfere with the wfi later on.
	 */
	copy_words(id->sgi_clr_pending, ptr, 4);

}

/*
 * Saves the shared parts of the distributor
 * Requires 1 word of memory, plus 20 words for each block of 32 SPIs (max 641 words)
 * Returns non-zero if an SPI interrupt is pending (after saving all required context)
 */
static void save_gic_distributor_shared(u32 * pointer,
					unsigned long gic_distributor_address)
{
	interrupt_distributor *id =
	    (interrupt_distributor *) gic_distributor_address;
	unsigned num_spis, *saved_pending;

	/* Calculate how many SPIs the GIC supports */
	num_spis = 32 * (id->controller_type & 0x1f);

	/* TODO: add nonsecure stuff */

	/* Save rest of GIC configuration */
	if (num_spis) {
		pointer =
		    copy_words(pointer, id->enable.set + 1, num_spis / 32);
		pointer = copy_words(pointer, id->priority + 8, num_spis / 4);
		pointer = copy_words(pointer, id->target + 8, num_spis / 4);
		pointer =
		    copy_words(pointer, id->configuration + 2, num_spis / 16);
		pointer = copy_words(pointer, id->security + 1, num_spis / 32);
		saved_pending = pointer;
		pointer =
		    copy_words(pointer, id->pending.set + 1, num_spis / 32);

		pointer =
		    copy_words(pointer, id->non_security_access_control + 1,
			       num_spis / 16);
	}

	/* Save control register */
	*pointer = id->control;
}

static void restore_gic_interface(u32 * pointer,
				  unsigned long gic_interface_address)
{
	cpu_interface *ci = (cpu_interface *) gic_interface_address;

	/* TODO: add nonsecure stuff */

	ci->priority_mask = pointer[1];
	ci->binary_point = pointer[2];
	ci->aliased_binary_point = pointer[3];

	ci->aliased_highest_pending = pointer[4];

	/* Restore control register last */
	ci->control = pointer[0];
}

static void restore_gic_distributor_private(u32 * pointer,
					    unsigned long
					    gic_distributor_address)
{
	interrupt_distributor *id =
	    (interrupt_distributor *) gic_distributor_address;
	unsigned tmp;
	//unsigned ctr, prev_val = 0, prev_ctr = 0;

	/* First disable the distributor so we can write to its config registers */
	tmp = id->control;
	id->control = 0;
	/* Restore SGI,PPI enable status */
	id->enable.set[0] = *pointer;
	++pointer;
	/* Restore SGI,PPI priority  status */
	copy_words(id->priority, pointer, 8);
	pointer += 8;
	/* Restore SGI,PPI target status */
	copy_words(id->target, pointer, 8);
	pointer += 8;
	/* Restore just the PPI configurations (SGIs are not configurable) */
	id->configuration[1] = *pointer;
	++pointer;
	/* Restore SGI,PPI security status */
	id->security[0] = *pointer;
	++pointer;

	/* restore SGI Non-security status (PPI is read-only) */
	id->non_security_access_control[0] =
	    (id->non_security_access_control[0] & 0x0ffff0000) | (*pointer);
	++pointer;

#if 0
	/*
	 * Clear active and  pending PPIs as they will be recreated by the
	 * peripiherals
	 */
	id->active.clear[0] = 0xffffffff;
	id->pending.clear[0] = 0xffffffff;
#endif
#if 1
	/*  Restore SGI,PPI pending status */
	id->pending.set[0] = *pointer;
	++pointer;
#endif
	/*
	 * Restore pending SGIs
	 */
	copy_words(id->sgi_set_pending, pointer, 4);
	pointer += 4;

	id->control = tmp;
}

static void restore_gic_spm_irq(unsigned long gic_distributor_address)
{
	interrupt_distributor *id =
	    (interrupt_distributor *) gic_distributor_address;
	unsigned int backup;
	int i, j;

	/* First disable the distributor so we can write to its config registers */

	backup = id->control;
	id->control = 0;

	/* Set the pending bit for spm wakeup source that is edge triggerd */
	if (reg_read(SPM_SLEEP_ISR_RAW_STA) & WAKE_SRC_KP) {
		i = DMT_KP_IRQ_BIT / GIC_PRIVATE_SIGNALS;
		j = DMT_KP_IRQ_BIT % GIC_PRIVATE_SIGNALS;
		id->pending.set[i] |= (1 << j);
	}
	if (reg_read(SPM_SLEEP_ISR_RAW_STA) & WAKE_SRC_CONN_WDT) {
		i = DMT_CONN_WDT_IRQ_BIT / GIC_PRIVATE_SIGNALS;
		j = DMT_CONN_WDT_IRQ_BIT % GIC_PRIVATE_SIGNALS;
		id->pending.set[i] |= (1 << j);
	}
	if (reg_read(SPM_SLEEP_ISR_RAW_STA) & WAKE_SRC_LOW_BAT) {
		i = DMT_LOWBATTERY_IRQ_BIT / GIC_PRIVATE_SIGNALS;
		j = DMT_LOWBATTERY_IRQ_BIT % GIC_PRIVATE_SIGNALS;
		id->pending.set[i] |= (1 << j);
	}
	if (reg_read(SPM_SLEEP_ISR_RAW_STA) & WAKE_SRC_MD_WDT) {
		i = DMT_MD1_WDT_BIT / GIC_PRIVATE_SIGNALS;
		j = DMT_MD1_WDT_BIT % GIC_PRIVATE_SIGNALS;
		id->pending.set[i] |= (1 << j);
	}
#if defined(CONFIG_ARCH_MT6735) && defined(CONFIG_MTK_C2K_SUPPORT)
	if (reg_read(SPM_SLEEP_ISR_RAW_STA) & WAKE_SRC_C2K_WDT) {
		i = DMT_C2K_WDT_BIT / GIC_PRIVATE_SIGNALS;
		j = DMT_C2K_WDT_BIT % GIC_PRIVATE_SIGNALS;
		id->pending.set[i] |= (1 << j);
	}
#endif

	/* We assume the I and F bits are set in the CPSR so that we will not respond to interrupts! */
	/* Restore control register */
	id->control = backup;
}

static void restore_gic_distributor_shared(u32 * pointer,
					   unsigned long
					   gic_distributor_address)
{
	interrupt_distributor *id =
	    (interrupt_distributor *) gic_distributor_address;
	unsigned num_spis;

	/* First disable the distributor so we can write to its config registers */
	id->control = 0;

	/* Calculate how many SPIs the GIC supports */
	num_spis = 32 * ((id->controller_type) & 0x1f);

	/* TODO: add nonsecure stuff */

	/* Restore rest of GIC configuration */
	if (num_spis) {
		copy_words(id->enable.set + 1, pointer, num_spis / 32);
		pointer += num_spis / 32;
		copy_words(id->priority + 8, pointer, num_spis / 4);
		pointer += num_spis / 4;
		copy_words(id->target + 8, pointer, num_spis / 4);
		pointer += num_spis / 4;
		copy_words(id->configuration + 2, pointer, num_spis / 16);
		pointer += num_spis / 16;
		copy_words(id->security + 1, pointer, num_spis / 32);
		pointer += num_spis / 32;
		copy_words(id->pending.set + 1, pointer, num_spis / 32);
		pointer += num_spis / 32;

		copy_words(id->non_security_access_control + 1, pointer,
			   num_spis / 16);
		pointer += num_spis / 16;

		restore_gic_spm_irq(gic_distributor_address);

	}

	/* We assume the I and F bits are set in the CPSR so that we will not respond to interrupts! */
	/* Restore control register */
	id->control = *pointer;
}

static void gic_cpu_save(void)
{
	save_gic_interface(gic_data_base()->gic_cpu_if_regs, DMT_GIC_CPU_BASE);
	/*
	 * TODO:
	 * Is it safe for the secondary cpu to save its context
	 * while the GIC distributor is on. Should be as its
	 * banked context and the cpu itself is the only one
	 * who can change it. Still have to consider cases e.g
	 * SGIs/Localtimers becoming pending.
	 */
	/* Save distributoer interface private context */
	save_gic_distributor_private(gic_data_base()->gic_dist_if_pvt_regs,
				     DMT_GIC_DIST_BASE);
}

static void gic_dist_save(void)
{
	/* Save distributoer interface global context */
	save_gic_distributor_shared(gic_data_base()->gic_dist_if_regs,
				    DMT_GIC_DIST_BASE);
}

static void gic_dist_restore(void)
{
	/*restores the global context  */
	restore_gic_distributor_shared(gic_data_base()->gic_dist_if_regs,
				       DMT_GIC_DIST_BASE);
}

void gic_cpu_restore(void)
{
	/*restores the private context  */
	restore_gic_distributor_private(gic_data_base()->gic_dist_if_pvt_regs,
					DMT_GIC_DIST_BASE);
	/* Restore GIC context */
	restore_gic_interface(gic_data_base()->gic_cpu_if_regs,
			      DMT_GIC_CPU_BASE);
}

#if defined(CONFIG_ARCH_MT6735) || defined(CONFIG_ARCH_MT6735M)
static inline void biu_reconfig(void)
{
	int val;
	val = reg_read(BIU_CONTROL);
	val = _or(val, TLB_ULTRA_EN);
	val = _or(val, DCM_EN);
	val = _or(val, CMD_QUEUE_EN);
	reg_write(BIU_CONTROL, val);
}
#endif

static inline void mp0_l2rstdisable(int flags)
{
}

static inline void mp1_l2rstdisable(int flags)
{
}

static inline void mp0_l2rstdisable_restore(int flags)
{
}

static inline void mp1_l2rstdisable_restore(int flags)
{
}

/* cluster_save_context: */
static void mt_cluster_save(int flags)
{
	/***************************************/
	/* int cpuid, clusterid;               */
	/* read_id(&cpuid, &clusterid);        */
	/* BUG_ON(cpuid != 0);                 */
	/***************************************/

	if (cluster_id() == 0) {
		mp0_l2rstdisable(flags);
	} else {
		mp1_l2rstdisable(flags);
	}
}

/* cluster_save_context: */
static void mt_cluster_restore(int flags)
{
	int cpuid, clusterid;

	/*************************************/
	/* if (flag != SHUTDOWN_MODE)        */
	/*      return;                      */
	/*************************************/

	read_id(&cpuid, &clusterid);

	if (cluster_id() == 0) {
		mp0_l2rstdisable_restore(flags);
	} else {
		mp1_l2rstdisable_restore(flags);
	}

#if defined(CONFIG_ARCH_MT6735) || defined(CONFIG_ARCH_MT6735M)
	biu_reconfig();
#endif
}

__weak unsigned int *mt_save_dbg_regs(unsigned int *p, unsigned int cpuid)
{
	return p;
}

__weak void mt_restore_dbg_regs(unsigned int *p, unsigned int cpuid)
{
	return;
}

__weak void mt_copy_dbg_regs(int to, int from)
{
	return;
}

void mt_cpu_save(void)
{
	core_context *core;
	cluster_context *cluster;
	unsigned int *ret;
	unsigned long dbg_base;
	unsigned int sleep_sta;
	int cpuid, clusterid;

	read_id(&cpuid, &clusterid);

	core = GET_CORE_DATA();

	ret = mt_save_generic_timer((unsigned int *)core->timer_data, 0x0);
	stop_generic_timer();	//disable timer irq, and upper layer should enable again.

	SENTINEL_CHECK(core->timer_data, ret);

	ret = save_pmu_context(core->pmu_data);
	SENTINEL_CHECK(core->pmu_data, ret);

	ret = save_vfp(core->vfp_data);
	SENTINEL_CHECK(core->vfp_data, ret);

	/** FIXME,
         * To restore preious backup context makeing MCDIed core geting inconsistent.
         * But to do a copy is not 100% right for Multi-core debuging or non-attached cores, 
         * which need to backup/restore itself.
         * However, SW is not able to aware of this 2 conditions.
         *
         * Right now, copy is prefered for internal debug usage.
         * And, save/restore is by cluster.
         **/
	if (clusterid == 0) {
		sleep_sta = (spm_read(SPM_SLEEP_TIMER_STA) >> 16) & 0x0f;
		dbg_base = DMT_MP0_DBGAPB_BASE;
	} else {
		sleep_sta = (spm_read(SPM_SLEEP_TIMER_STA) >> 20) & 0x0f;
		dbg_base = DMT_MP1_DBGAPB_BASE;
	}

	if ((sleep_sta | (1 << cpuid)) == 0x0f) {	// last core
		cluster = GET_CLUSTER_DATA();
		ret =
		    mt_save_dbg_regs((unsigned int *)cluster->dbg_data,
				     cpuid + (clusterid * 4));
		SENTINEL_CHECK(cluster->dbg_data, ret);
	} else {
		/** do nothing **/
	}

	ret = mt_save_banked_registers(core->banked_regs);
	SENTINEL_CHECK(core->banked_regs, ret);
}

void mt_cpu_restore(void)
{
	core_context *core;
	cluster_context *cluster;
	unsigned long dbg_base;
	unsigned int sleep_sta;
	int cpuid, clusterid;

	read_id(&cpuid, &clusterid);

	core = GET_CORE_DATA();

	mt_restore_banked_registers(core->banked_regs);

	/** FIXME,
         * To restore preious backup context makeing MCDIed core geting inconsistent.
         * But to do a copy is not 100% right for Multi-core debuging or non-attached cores, 
         * which need to backup/restore itself.
         * However, SW is not able to aware of this 2 conditions.
         *
         * Right now, copy is prefered for internal debug usage.
         * And, save/restore is by cluster.
         **/
	if (clusterid == 0) {
		sleep_sta = (spm_read(SPM_SLEEP_TIMER_STA) >> 16) & 0x0f;
		dbg_base = DMT_MP0_DBGAPB_BASE;
	} else {
		sleep_sta = (spm_read(SPM_SLEEP_TIMER_STA) >> 20) & 0x0f;
		dbg_base = DMT_MP1_DBGAPB_BASE;
	}

	sleep_sta = (sleep_sta | (1 << cpuid));

	if (sleep_sta == 0x0f) {	// first core
		cluster = GET_CLUSTER_DATA();
		mt_restore_dbg_regs((unsigned int *)cluster->dbg_data,
				    cpuid + (clusterid * 4));
	} else {		//otherwise, do copy from anyone
		int any = __builtin_ffs(~sleep_sta) - 1;
		mt_copy_dbg_regs(cpuid + (clusterid * 4),
				 any + (clusterid * 4));
	}

	restore_vfp(core->vfp_data);

	restore_pmu_context(core->pmu_data);

	mt_restore_generic_timer((unsigned int *)core->timer_data, 0x0);

}

void mt_platform_save_context(int flags)
{
	/* mcusys_save_context: */
	mt_cpu_save();
	mt_cluster_save(flags);

	if (IS_DORMANT_GIC_OFF(flags)) {
		//mt_gic_save_contex;
		gic_cpu_save();
		gic_dist_save();
	}

	/* infrasys_save_context: */
	/* misc_save_context; */

}

void mt_platform_restore_context(int flags)
{
	/* misc_restore_context: */
	/* infrasys_restore_context: */

	/* mcusys_restore_context: */
	mt_cluster_restore(flags);
	mt_cpu_restore();

	if (IS_DORMANT_GIC_OFF(flags)) {
		gic_dist_restore();
		gic_cpu_restore();
	}
}

#define _get_sp() ({ register void *SP asm("sp"); SP; })

int mt_cpu_dormant_psci(unsigned long flags)
{
	int ret = 1;
	struct psci_power_state pps = {
		.type = PSCI_POWER_STATE_TYPE_POWER_DOWN,
		.affinity_level = 1,
	};

	int cpuid, clusterid;
	read_id(&cpuid, &clusterid);

	if (psci_ops.cpu_suspend) {
		DORMANT_LOG(clusterid * 4 + cpuid, 0x203);
		ret = psci_ops.cpu_suspend(pps, virt_to_phys(cpu_resume));
	}

	BUG();

	return ret;
}

static int mt_cpu_dormant_abort(unsigned long flags)
{
	int cpuid, clusterid;

	read_id(&cpuid, &clusterid);

	/* restore l2rstdisable setting */
	if (cluster_id() == 0) {
		mp0_l2rstdisable_restore(flags);
	} else {
		mp1_l2rstdisable_restore(flags);
	}
#if defined(CONFIG_ARCH_MT6735) || defined(CONFIG_ARCH_MT6735M)
	biu_reconfig();
#endif

	// enable generic timer
	start_generic_timer();

	/* // unlock dbg oslock/dlock        */
	/* write_dbgoslar(0);                */
	/* isb(); */
	/* write_dbgosdlr(0);                */

	return 0;
}

int mt_cpu_dormant(unsigned long flags)
{
	int ret;
	int cpuid, clusterid;
	static unsigned int dormant_count;
	core_context *core = GET_CORE_DATA();

#if defined(MT_DORMANT_UT)
	extern int mt_irq_mask_all(struct mtk_irq_mask *mask);
	extern int mt_irq_mask_restore(struct mtk_irq_mask *mask);
	struct mtk_irq_mask mask;
#endif //#if defined(MT_DORMANT_UT)

	if (mt_dormant_initialized == 0)
		return MT_CPU_DORMANT_BYPASS;

	// debug purpose, just bypass
	if (DEBUG_DORMANT_BYPASS || debug_dormant_bypass == 1)
		return MT_CPU_DORMANT_BYPASS;

	read_id(&cpuid, &clusterid);

	DORMANT_LOG(clusterid * 4 + cpuid, 0x101);

	dormant_count++;
	core->count++;

	CPU_DORMANT_INFO("dormant(%d) flags:%lu start (cluster/cpu:%d/%d) !\n",
			 dormant_count, flags, clusterid, cpuid);

	TSLOG(core->timestamp[0], read_cntpct());

	BUG_ON(!irqs_disabled());
	// to make sure idle task no need to save VFP context.
	BUG_ON(current->mm && !test_thread_flag(TIF_FOREIGN_FPSTATE));

	// to mark as cpu clobs vfp register.
	kernel_neon_begin();

	// dormant break
	if (IS_DORMANT_BREAK_CHECK(flags) &&
	    SPM_IS_CPU_IRQ_OCCUR(SPM_CORE_ID())) {
		ret = MT_CPU_DORMANT_BREAK_V(IRQ_PENDING_1);
		goto _back;
	}

#if defined(MT_DORMANT_UT)
	mt_irq_mask_all(&mask);
#endif //#if defined(MT_DORMANT_UT)

	//
	TSLOG(core->timestamp[1], read_cntpct());
	mt_platform_save_context(flags);

	DORMANT_LOG(clusterid * 4 + cpuid, 0x102);
	TSLOG(core->timestamp[2], read_cntpct());

	// dormant break
	if (IS_DORMANT_BREAK_CHECK(flags) &&
	    SPM_IS_CPU_IRQ_OCCUR(SPM_CORE_ID())) {
		mt_cpu_dormant_abort(flags);
		ret = MT_CPU_DORMANT_BREAK_V(IRQ_PENDING_2);
		goto _back;
	}

	DORMANT_LOG(clusterid * 4 + cpuid, 0x103);

	ret = cpu_suspend(flags);

	DORMANT_LOG(clusterid * 4 + cpuid, 0x601);
	TSLOG(core->timestamp[3], read_cntpct());

	switch (ret) {
	case 0:		// back from dormant reset
		mt_platform_restore_context(flags);
#ifdef CONFIG_MTK_ETM
		trace_start_dormant();
#endif
		core->rst++;
		ret = MT_CPU_DORMANT_RESET;
		break;

	case 1:		// back from dormant abort,
		mt_cpu_dormant_abort(flags);
		core->abt++;
		ret = MT_CPU_DORMANT_ABORT;
		break;
	case 2:
		mt_cpu_dormant_abort(flags);
		core->brk++;
		ret = MT_CPU_DORMANT_BREAK_V(IRQ_PENDING_3);
		break;
	default:		// back from dormant break, do nothing for return
		CPU_DORMANT_INFO("EOPNOTSUPP \n");
		break;
	}

	DORMANT_LOG(clusterid * 4 + cpuid, 0x602);

#if defined(MT_DORMANT_UT)
	mt_irq_mask_restore(&mask);
#endif //#if defined(MT_DORMANT_UT)

	local_fiq_enable();  /** cpu mask F-bit at reset, but nobody clear that **/

_back:

	TSLOG(core->timestamp[4], read_cntpct());

#if defined(TSLOG_ENABLE)
	if (MT_CPU_DORMANT_BREAK & ret)
		CPU_DORMANT_INFO("dormant BREAK(%d) !! \n\t", ret);
	if (MT_CPU_DORMANT_ABORT & ret)
		CPU_DORMANT_INFO("dormant ABORT(%d) !! \n\t", ret);

	CPU_DORMANT_INFO
	    ("dormant(flags:%d) (ret:%d) (core:%d/%d) cnt:%d, rst:%d, abt:%d, brk:%d\n",
	     flags, ret, clusterid, cpuid, core->count, core->rst, core->abt,
	     core->brk);
	CPU_DORMANT_INFO("dormant timing: %llu, %llu, %llu, %llu, %llu\n",
			 core->timestamp[0], core->timestamp[1],
			 core->timestamp[2], core->timestamp[3],
			 core->timestamp[4]);
#endif

	kernel_neon_end();

	DORMANT_LOG(clusterid * 4 + cpuid, 0x0);

	return ret & 0x0ff;

}

#if defined (CONFIG_OF)
static int mt_dormant_dts_map(void)
{
	struct device_node *node;
	u32 kp_interrupt[3];
	u32 consys_interrupt[6];
	u32 auxadc_interrupt[3];
	u32 mdcldma_interrupt[9];
#if defined(CONFIG_ARCH_MT6735) && defined(CONFIG_MTK_C2K_SUPPORT)
	u32 mdc2k_interrupt[3];
#endif

	/* dbgapb */
	node = of_find_compatible_node(NULL, NULL, DBGAPB_NODE);
	if (!node) {
		CPU_DORMANT_INFO("error: cannot find node " DBGAPB_NODE);
		BUG();
	}
	dbgapb_base = (unsigned long)of_iomap(node, 0);
	if (!dbgapb_base) {
		CPU_DORMANT_INFO("error: cannot iomap " DBGAPB_NODE);
		BUG();
	}
	of_node_put(node);

	/* mcucfg */
	node = of_find_compatible_node(NULL, NULL, MCUCFG_NODE);
	if (!node) {
		CPU_DORMANT_INFO("error: cannot find node " MCUCFG_NODE);
		BUG();
	}
	mcucfg_base = (unsigned long)of_iomap(node, 0);
	if (!mcucfg_base) {
		CPU_DORMANT_INFO("error: cannot iomap " MCUCFG_NODE);
		BUG();
	}
	of_node_put(node);

#if defined(CONFIG_ARCH_MT6735) || defined(CONFIG_ARCH_MT6735M)
	/* BIU */
	node = of_find_compatible_node(NULL, NULL, BIU_NODE);
	if (!node) {
		CPU_DORMANT_INFO("error: cannot find node " BIU_NODE);
		BUG();
	}
	biu_base = (unsigned long)of_iomap(node, 0);
	if (!biu_base) {
		CPU_DORMANT_INFO("error: cannot iomap " BIU_NODE);
		BUG();
	}
	of_node_put(node);
#endif

	/* infracfg_ao */
	node = of_find_compatible_node(NULL, NULL, INFRACFG_AO_NODE);
	if (!node) {
		CPU_DORMANT_INFO("error: cannot find node " INFRACFG_AO_NODE);
		BUG();
	}
	infracfg_ao_base = (unsigned long)of_iomap(node, 0);
	if (!infracfg_ao_base) {
		CPU_DORMANT_INFO("error: cannot iomap " INFRACFG_AO_NODE);
		BUG();
	}
	of_node_put(node);

	/* gic */
	node = of_find_compatible_node(NULL, NULL, GIC_NODE);
	if (!node) {
		CPU_DORMANT_INFO("error: cannot find node " GIC_NODE);
		BUG();
	}
	gic_id_base = (unsigned long)of_iomap(node, 0);
	gic_ci_base = (unsigned long)of_iomap(node, 1);
	if (!gic_id_base || !gic_ci_base) {
		CPU_DORMANT_INFO("error: cannot iomap " GIC_NODE);
		BUG();
	}
	of_node_put(node);

	/* kp_irq_bit */
	node = of_find_compatible_node(NULL, NULL, KP_NODE);
	if (!node) {
		CPU_DORMANT_INFO("error: cannot find node " KP_NODE);
		BUG();
	}
	if (of_property_read_u32_array(node, "interrupts",
				       kp_interrupt,
				       ARRAY_SIZE(kp_interrupt))) {
		CPU_DORMANT_INFO("error: cannot property_read " KP_NODE);
		BUG();
	}
	kp_irq_bit = ((1 - kp_interrupt[0]) << 5) + kp_interrupt[1];	// irq[0] = 0 => spi
	of_node_put(node);
	CPU_DORMANT_INFO("kp_irq_bit = %u\n", kp_irq_bit);

	/* conn_wdt_irq_bit */
	node = of_find_compatible_node(NULL, NULL, CONSYS_NODE);
	if (!node) {
		CPU_DORMANT_INFO("error: cannot find node " CONSYS_NODE);
		BUG();
	}
	if (of_property_read_u32_array(node, "interrupts",
				       consys_interrupt,
				       ARRAY_SIZE(consys_interrupt))) {
		CPU_DORMANT_INFO("error: cannot property_read " CONSYS_NODE);
		BUG();
	}
	conn_wdt_irq_bit = ((1 - consys_interrupt[3]) << 5) + consys_interrupt[4];	// irq[0] = 0 => spi
	of_node_put(node);
	CPU_DORMANT_INFO("conn_wdt_irq_bit = %u\n", conn_wdt_irq_bit);

	/* lowbattery_irq_bit */
	node = of_find_compatible_node(NULL, NULL, AUXADC_NODE);
	if (!node) {
		CPU_DORMANT_INFO("error: cannot find node " AUXADC_NODE);
		BUG();
	}
	if (of_property_read_u32_array(node, "interrupts",
				       auxadc_interrupt,
				       ARRAY_SIZE(auxadc_interrupt))) {
		CPU_DORMANT_INFO("error: cannot property_read " AUXADC_NODE);
		BUG();
	}
	lowbattery_irq_bit = ((1 - auxadc_interrupt[0]) << 5) + auxadc_interrupt[1];	// irq[0] = 0 => spi
	of_node_put(node);
	CPU_DORMANT_INFO("lowbattery_irq_bit = %u\n", lowbattery_irq_bit);

	/* md1_wdt_bit */
	node = of_find_compatible_node(NULL, NULL, MDCLDMA_NODE);
	if (!node) {
		CPU_DORMANT_INFO("error: cannot find node " MDCLDMA_NODE);
		BUG();
	}
	if (of_property_read_u32_array(node, "interrupts",
				       mdcldma_interrupt,
				       ARRAY_SIZE(mdcldma_interrupt))) {
		CPU_DORMANT_INFO("error: cannot property_read " MDCLDMA_NODE);
		BUG();
	}
	md1_wdt_bit = ((1 - mdcldma_interrupt[6]) << 5) + mdcldma_interrupt[7];	// irq[0] = 0 => spi
	of_node_put(node);
	CPU_DORMANT_INFO("md1_wdt_bit = %u\n", md1_wdt_bit);

#if defined(CONFIG_ARCH_MT6735) && defined(CONFIG_MTK_C2K_SUPPORT)
	/* c2k_wdt_bit */
	node = of_find_compatible_node(NULL, NULL, MDC2K_NODE);
	if (!node) {
		CPU_DORMANT_INFO("error: cannot find node " MDC2K_NODE);
		BUG();
	}
	if (of_property_read_u32_array(node, "interrupts",
				       mdc2k_interrupt,
				       ARRAY_SIZE(mdc2k_interrupt))) {
		CPU_DORMANT_INFO("error: cannot property_read " MDC2K_NODE);
		BUG();
	}
	c2k_wdt_bit = ((1 - mdc2k_interrupt[0]) << 5) + mdc2k_interrupt[1];	// irq[0] = 0 => spi
	of_node_put(node);
	CPU_DORMANT_INFO("c2k_wdt_bit = %u\n", c2k_wdt_bit);
#endif

	return 0;
}
#else //#if definded(CONFIG_OF)
static int mt_dormant_dts_map(void)
{
	return 0;
}
#endif //#if definded(CONFIG_OF)

int mt_cpu_dormant_init(void)
{
	int cpuid, clusterid;
	read_id(&cpuid, &clusterid);

	if (mt_dormant_initialized == 1)
		return 0;

	// map base address
	mt_dormant_dts_map();

#if CPU_DORMANT_AEE_RR_REC
	sleep_aee_rec_cpu_dormant_va =
	    dormant_data[0].poc.cpu_dormant_aee_rr_rec =
	    aee_rr_rec_cpu_dormant();
	sleep_aee_rec_cpu_dormant = (phys_addr_t) aee_rr_rec_cpu_dormant_pa();

	kernel_smc_msg(0, 2, sleep_aee_rec_cpu_dormant);

	CPU_DORMANT_INFO("dormant init aee_rec_cpu_dormant: va:%p pa:%llu\n",
			 sleep_aee_rec_cpu_dormant_va,
			 (long long) sleep_aee_rec_cpu_dormant);
#endif

	mt_dormant_initialized = 1;

#if defined (MT_DORMANT_UT)
	{
#include <asm/system_misc.h>
		int mt_cpu_dormant_test(void);
		arm_pm_idle = mt_cpu_dormant_test;
	}
#endif

	return 0;
}

// move to mt_pm_init to resolve dependency with others.
//late_initcall(mt_cpu_dormant_init);

#if defined (MT_DORMANT_UT)
volatile int mt_cpu_dormant_test_mode = (CPU_SUSPEND_MODE | 0);

int mt_cpu_dormant_test(void)
{
	return mt_cpu_dormant(mt_cpu_dormant_test_mode);
}

#endif //#if defined (MT_DORMANT_UT)
