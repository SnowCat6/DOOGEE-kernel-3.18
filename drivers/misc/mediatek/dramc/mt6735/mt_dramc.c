#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kallsyms.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/vmalloc.h>
#include <linux/memblock.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_freqhopping.h>
#include <mach/emi_bwl.h>
#include <mach/mt_typedefs.h>
#include <mach/memory.h>
#include <mach/mt_sleep.h>
#include <mach/mt_dramc.h>
#include <mach/dma.h>
#include <mach/mt_dcm.h>
#include <mach/sync_write.h>
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#include <linux/of.h>
#include <linux/of_address.h>

#ifdef FREQ_HOPPING_TEST
#include <mach/mt_freqhopping.h>
#endif

#ifdef VCORE1_ADJ_TEST
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#endif

static void __iomem *APMIXED_BASE_ADDR;
static void __iomem *CQDMA_BASE_ADDR;
static void __iomem *DRAMCAO_BASE_ADDR;
static void __iomem *DDRPHY_BASE_ADDR;
static void __iomem *DRAMCNAO_BASE_ADDR;

volatile unsigned char *dst_array_v;
volatile unsigned char *src_array_v;
volatile unsigned int dst_array_p;
volatile unsigned int src_array_p;
char dfs_dummy_buffer[BUFF_LEN] __attribute__ ((aligned(PAGE_SIZE)));
int init_done = 0;
static DEFINE_MUTEX(dram_dfs_mutex);
int org_dram_data_rate = 0;

void Reg_Sync_Writel(unsigned long addr, unsigned int val)
{
	(*(volatile unsigned int *)(addr)) = val;
	dsb();
}

unsigned int Reg_Readl(unsigned long addr)
{
	unsigned int val;
	val = (*(volatile unsigned int *)(addr));

	return val;
}
/*----------------------------------------------------------------------------------------*/
/* Sampler function for profiling purpose                                                 */
/*----------------------------------------------------------------------------------------*/
typedef void (*dram_sampler_func) (unsigned int);
static dram_sampler_func g_pDramFreqSampler;

void mt_dramfreq_setfreq_registerCB(dram_sampler_func pCB)
{
	g_pDramFreqSampler = pCB;
}

EXPORT_SYMBOL(mt_dramfreq_setfreq_registerCB);

void mt_dramfreq_setfreq_notify(unsigned int freq)
{
	if (NULL != g_pDramFreqSampler)
		g_pDramFreqSampler(freq);
}

#ifdef VCORE1_ADJ_TEST
void pmic_voltage_read(unsigned int nAdjust)
{
	int ret_val = 0;
	unsigned int OldVcore1 = 0;
	unsigned int OldVmem = 0, OldVmem1 = 0;

	pr_warn("[PMIC]pmic_voltage_read : \r\n");

	ret_val = pmic_read_interface(MT6328_VCORE1_CON11, &OldVcore1, 0x7F, 0);
	ret_val = pmic_read_interface(MT6328_SLDO_ANA_CON0, &OldVmem, 0x7F, 0);
	ret_val = pmic_read_interface(MT6328_SLDO_ANA_CON1, &OldVmem1, 0x0F, 8);

	pr_warn("[Vcore] MT6328_VCORE1_CON11=0x%x,\r\n[Vmem] MT6328_SLDO_ANA_CON0/1=0x%x 0x%x\r\n",
		OldVcore1, OldVmem, OldVmem1);
}

void pmic_Vcore_adjust(int nAdjust)
{
	switch (nAdjust) {
	case 0:		/* HV 1280MHz */
		/*pmic_config_interface(MT6328_VCORE1_CON11, 0x6B, 0x7F, 0);*/    /* 1.265V */
		pmic_config_interface(MT6328_VCORE1_CON11, 0x65, 0x7F, 0);	/* 1.230V */
		break;
	case 1:		/* HV 938MHz */
		/*pmic_config_interface(MT6328_VCORE1_CON11, 0x59, 0x7F, 0);*/    /* 1.155V */
		pmic_config_interface(MT6328_VCORE1_CON11, 0x54, 0x7F, 0);	/* 1.124V */
		break;
	case 2:		/* NV 1280MHz */
		pmic_config_interface(MT6328_VCORE1_CON11, 0x58, 0x7F, 0);	/* 1.15V */
		break;
	case 3:		/* NV 938MHz */
		pmic_config_interface(MT6328_VCORE1_CON11, 0x48, 0x7F, 0);	/* 1.05V */
		break;
	case 4:		/* LV 1280MHz */
		/*pmic_config_interface(MT6328_VCORE1_CON11, 0x46, 0x7F, 0);*/    /* 1.035V */
		pmic_config_interface(MT6328_VCORE1_CON11, 0x4B, 0x7F, 0);	/* 1.070V */
		break;
	case 5:		/* LV 938MHz */
		/*pmic_config_interface(MT6328_VCORE1_CON11, 0x38, 0x7F, 0);*/    /* 0.945V */
		pmic_config_interface(MT6328_VCORE1_CON11, 0x3C, 0x7F, 0);	/* 0.976V */
		break;
	default:
		pr_warn("Enter incorrect config number !!!\n");
		break;
	}

}

void pmic_Vmem_adjust(int nAdjust)
{
	switch (nAdjust) {
	case 0:
		pmic_config_interface(MT6328_SLDO_ANA_CON0, 0, 0x3, 0);	/* 1.24V */
		break;
	case 1:
		pmic_config_interface(MT6328_SLDO_ANA_CON0, 1, 0x3, 0);	/* 1.39V */
		break;
	case 2:
		pmic_config_interface(MT6328_SLDO_ANA_CON0, 2, 0x3, 0);	/* 1.54V */
		break;
	default:
		pmic_config_interface(MT6328_SLDO_ANA_CON0, 0, 0x3, 0);	/* 1.24V */
		break;
	}
}

void pmic_Vmem_Cal_adjust(int nAdjust)
{
	switch (nAdjust) {
	case 0:
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x0D, 0xF, 8);	/* +0.06V */
		break;
	case 1:
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x01, 0xF, 8);	/* -0.02V */
		break;
	case 2:
		/*pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x04, 0xF, 8);*/    /* -0.08V */
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x05, 0xF, 8);	/* -1.00V */
		break;
	default:
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x00, 0xF, 8);	/* 1.24V */
		break;
	}
}

void pmic_HQA_NoSSC_Voltage_adjust(int nAdjust)
{
	switch (nAdjust) {
	case 0:		/* Vm = 1.18V & Vc = 1.15V */
		pmic_config_interface(MT6328_VCORE1_CON11, 0x58, 0x7F, 0);	/* 1.15V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x58, 0x7F, 0);	/* 1.15V */
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x03, 0xF, 8);	/* -0.06V (1.18V) */
		pr_warn("========== Vm = 1.18V & Vc = 1.15V ==========\r\n");
		break;
	case 1:		/* Vm = 1.26V & Vc = 1.15V */
		pmic_config_interface(MT6328_VCORE1_CON11, 0x58, 0x7F, 0);	/* 1.15V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x58, 0x7F, 0);	/* 1.15V */
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x0F, 0xF, 8);	/* +0.02V (1.26V) */
		pr_warn("========== Vm = 1.26V & Vc = 1.15V ==========\r\n");
		break;
	case 2:		/* Vm = 1.22V & Vc = 1.11V */
		pmic_config_interface(MT6328_VCORE1_CON11, 0x52, 0x7F, 0);	/* 1.11V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x52, 0x7F, 0);	/* 1.11V */
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x01, 0xF, 8);	/* -0.02V (1.22V) */
		pr_warn("========== Vm = 1.22V & Vc = 1.11V ==========\r\n");
		break;
	case 3:		/* Vm = 1.22V & Vc = 1.19V */
		pmic_config_interface(MT6328_VCORE1_CON11, 0x5F, 0x7F, 0);	/* 1.19V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x5F, 0x7F, 0);	/* 1.19V */
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x01, 0xF, 8);	/* -0.02V (1.22V) */
		pr_warn("========== Vm = 1.22V & Vc = 1.19V ==========\r\n");
		break;
	case 4:		/*NV*/
		pmic_config_interface(MT6328_VCORE1_CON11, 0x58, 0x7F, 0);	/* 1.15V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x58, 0x7F, 0);	/* 1.15V */
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x01, 0xF, 8);	/* -0.02V (1.22V) */
		pr_warn("========== NV ==========\r\n");
		break;
	default:	/*NV*/
		pmic_config_interface(MT6328_VCORE1_CON11, 0x58, 0x7F, 0);	/* 1.15V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x58, 0x7F, 0);	/* 1.15V */
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x01, 0xF, 8);	/* -0.02V (1.22V) */
		pr_warn("========== NV ==========\r\n");
		break;
	}
}

void pmic_HQA_Voltage_adjust(int nAdjust)
{
	switch (nAdjust) {
	case 0:		/*HVcHVm*/
		/*pmic_config_interface(MT6328_VCORE1_CON11, 0x6B, 0x7F, 0);*/    /* 1.265V */
#if defined(CONFIG_ARCH_MT6753)
		pmic_config_interface(MT6328_VCORE1_CON11, 0x71, 0x7F, 0);	/* 1.310V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x71, 0x7F, 0);	/* 1.310V */
#else
		pmic_config_interface(MT6328_VCORE1_CON11, 0x65, 0x7F, 0);	/* 1.230V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x65, 0x7F, 0);	/* 1.230V */
#endif
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x0D, 0xF, 8);	/* +0.06V (1.3V) */
		pr_warn("========== HVcHVm ==========\r\n");
		break;
	case 1:		/*HVcLVm*/
		/*pmic_config_interface(MT6328_VCORE1_CON11, 0x6B, 0x7F, 0);*/    /* 1.265V */
#if defined(CONFIG_ARCH_MT6753)
		pmic_config_interface(MT6328_VCORE1_CON11, 0x71, 0x7F, 0);	/* 1.310V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x71, 0x7F, 0);	/* 1.310V */
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x04, 0xF, 8);	/* -0.08V (1.16V) */
#else
		pmic_config_interface(MT6328_VCORE1_CON11, 0x65, 0x7F, 0);	/* 1.230V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x65, 0x7F, 0);	/* 1.230V */
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x05, 0xF, 8);	/* -1.00V (1.14V) */
#endif
		pr_warn("========== HVcLVm ==========\r\n");
		break;
	case 2:		/*LVcHVm*/
		/*pmic_config_interface(MT6328_VCORE1_CON11, 0x46, 0x7F, 0);*/    /* 1.035V */
#if defined(CONFIG_ARCH_MT6753)
		pmic_config_interface(MT6328_VCORE1_CON11, 0x54, 0x7F, 0);	/* 1.125V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x54, 0x7F, 0);	/* 1.125V */
#else
		pmic_config_interface(MT6328_VCORE1_CON11, 0x4B, 0x7F, 0);	/* 1.070V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x4B, 0x7F, 0);	/* 1.070V */
#endif
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x0D, 0xF, 8);	/* +0.06V (1.3V) */
		pr_warn("========== LVcHVm ==========\r\n");
		break;
	case 3:		/*LVcLVm*/
		/*pmic_config_interface(MT6328_VCORE1_CON11, 0x46, 0x7F, 0);*/    /* 1.035V */
#if defined(CONFIG_ARCH_MT6753)
		pmic_config_interface(MT6328_VCORE1_CON11, 0x54, 0x7F, 0);	/* 1.125V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x54, 0x7F, 0);	/* 1.125V */
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x04, 0xF, 8);	/* -0.08V (1.16V)*/
#else
		pmic_config_interface(MT6328_VCORE1_CON11, 0x4B, 0x7F, 0);	/* 1.070V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x4B, 0x7F, 0);	/* 1.070V */
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x05, 0xF, 8);	/* -1.00V (1.14V) */
#endif
		pr_warn("========== LVcLVm ==========\r\n");
		break;
	case 4:		/*NV*/
#if defined(CONFIG_ARCH_MT6753)
		pmic_config_interface(MT6328_VCORE1_CON11, 0x68, 0x7F, 0);	/* 1.25V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x68, 0x7F, 0);	/* 1.25V */
#else
		pmic_config_interface(MT6328_VCORE1_CON11, 0x58, 0x7F, 0);	/* 1.15V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x58, 0x7F, 0);	/* 1.15V */
#endif
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x01, 0xF, 8);	/* -0.02V (1.22V) */
		pr_warn("========== NV ==========\r\n");
		break;
	default:	/*NV*/
#if defined(CONFIG_ARCH_MT6753)
		pmic_config_interface(MT6328_VCORE1_CON11, 0x68, 0x7F, 0);	/* 1.25V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x68, 0x7F, 0);	/* 1.25V */
#else
		pmic_config_interface(MT6328_VCORE1_CON11, 0x58, 0x7F, 0);	/* 1.15V */
		pmic_config_interface(MT6328_VCORE1_CON12, 0x58, 0x7F, 0);	/* 1.15V */
#endif
		pmic_config_interface(MT6328_SLDO_ANA_CON1, 0x01, 0xF, 8);	/* -0.02V (1.22V) */
		pr_warn("========== NV ==========\r\n");
		break;
	}
}
#endif

#ifdef DUMP_DDR_RESERVED_MODE
void dump_DMA_Reserved_AREA(void)
{
	int i;
	int j;
	int k;
	/*unsigned int *debug_p_src;*/
	   /* unsigned int *debug_p_dst; */

	   /* debug_p_src = src_array_p; */
	   /* debug_p_dst = dst_array_p; */

	   pr_warn("=====DMA DUMP Reserved SRC AREA======\n");
	   /*  dump memory info to make sure 0x6a6a6a6a already write into memory */
	   for (i = 0; i < 8; i++) {
		pr_warn("[SRC] ");
		for (j = 0; j < 8; j++)
			pr_warn("0x%p=0x%x ", ((debug_p_src+i*8)+j), *((debug_p_src+i*8)+j));
		/* pr_warn("&0x%p=0x%x ", ((debug_p_src+i*8)+j), readl(IOMEM((debug_p_src+i*8)+j))); */
		pr_warn("\n");
	   }

	   pr_warn("=====DMA DUMP Reserved SRC AREA END======\n\n");

	pr_warn("=====DMA DUMP Reserved DST AREA======\n");
	for (i = 0; i < 8; i++) {
		pr_warn("[DST] ");
		for (j = 0; j < 8; j += 4) {
			pr_warn("val=0x%02X",
			       dfs_dummy_buffer[(i * 8) + (j + 3)]);
			pr_warn("%02X", dfs_dummy_buffer[(i * 8) + (j + 2)]);
			pr_warn("%02X", dfs_dummy_buffer[(i * 8) + (j + 1)]);
			pr_warn("%02X ", dfs_dummy_buffer[(i * 8) + j]);
		}
		/*pr_warn("&0x%p=0x%x ", ((debug_p_dst+i*8)+j), readl(IOMEM((debug_p_dst+i*8)+j)));*/
		pr_warn("\n");
	}
	pr_warn("=====DMA DUMP Reserved DST AREA END======\n\n");

	enable_clock(MT_CG_INFRA_GCE, "CQDMA");
	pr_warn("=====DMA DUMP REGS======\n");
	for (k = 0; k < 25; k++) {
		pr_warn("[CQDMA] addr:0x%p, value:%x\n",
		       (CQDMA_BASE_ADDR + k * 4),
		       readl(IOMEM(CQDMA_BASE_ADDR + k * 4)));
	}
	disable_clock(MT_CG_INFRA_GCE, "CQDMA");

	pr_warn("=====END=====\n");
}
#endif

#if defined(CONFIG_ARCH_MT6753)
int enter_pasr_dpd_config(unsigned char segment_rank0,
			   unsigned char segment_rank1)
{
	/* for D-3, support run time MRW */
	unsigned int rank_pasr_segment[2];
	unsigned int dramc0_spcmd;
	unsigned int i, cnt = 1000;

	rank_pasr_segment[0] = segment_rank0 & 0xFF;	/* for rank0 */
	rank_pasr_segment[1] = segment_rank1 & 0xFF;	/* for rank1 */
	pr_warn("[DRAMC0] PASR r0 = 0x%x  r1 = 0x%x\n", rank_pasr_segment[0], rank_pasr_segment[1]);

	/* backup original data */
	dramc0_spcmd = readl(PDEF_DRAMC0_REG_1E4);

	for (i = 0; i < 2; i++) {
		/* set MRS settings include rank number, segment information and MRR17 */
		writel(((i << 28) | (rank_pasr_segment[i] << 16) | 0x00000011), PDEF_DRAMC0_REG_088);
		/* Mode register write command enable */
		writel(0x00000001, PDEF_DRAMC0_REG_1E4);

		/* wait MRW command response */
		/* wait >1us */
		/* gpt_busy_wait_us(1); */
		do {
			if (cnt-- == 0) {
				pr_warn("[DRAMC0] PASR MRW fail!\n");
				return -1;
				break;
			}
			udelay(1);
		} while ((readl(PDEF_DRAMC0_REG_3B8) & 0x00000001) == 0x0);
		dsb();

		/* Mode register write command disable */
		writel(0x0, PDEF_DRAMC0_REG_1E4);
	}
	/* writel(0x00000004, PDEF_DRAMC0_REG_088); */
	pr_warn("[DRAMC0] PASR offset 0x88 = 0x%x\n", readl(PDEF_DRAMC0_REG_088));
	writel(dramc0_spcmd, PDEF_DRAMC0_REG_1E4);

	return 0;
#if 0
	if (segment_rank1 == 0xFF) {	/*all segments of rank1 are not reserved -> rank1 enter DPD*/
		slp_dpd_en(1);
	}
#endif

	/*slp_pasr_en(1, segment_rank0 | (segment_rank1 << 8));*/
}

int exit_pasr_dpd_config(void)
{
	int ret;
	/*slp_dpd_en(0);*/
	/*slp_pasr_en(0, 0);*/
	ret = enter_pasr_dpd_config(0, 0);

	return ret;
}
#endif



#define MEM_TEST_SIZE 0x2000
#define PATTERN1 0x5A5A5A5A
#define PATTERN2 0xA5A5A5A5
int Binning_DRAM_complex_mem_test(void)
{
	unsigned char *MEM8_BASE;
	unsigned short *MEM16_BASE;
	unsigned int *MEM32_BASE;
	unsigned int *MEM_BASE;
	unsigned long mem_ptr;
	unsigned char pattern8;
	unsigned short pattern16;
	unsigned int i, j, size, pattern32;
	unsigned int value;
	unsigned int len = MEM_TEST_SIZE;
	void *ptr;
	ptr = vmalloc(PAGE_SIZE * 2);
	MEM8_BASE = (unsigned char *)ptr;
	MEM16_BASE = (unsigned short *)ptr;
	MEM32_BASE = (unsigned int *)ptr;
	MEM_BASE = (unsigned int *)ptr;
	pr_warn("Test DRAM start address 0x%lx\n", (unsigned long)ptr);
	pr_warn("Test DRAM SIZE 0x%x\n", MEM_TEST_SIZE);
	size = len >> 2;

	/* === Verify the tied bits (tied high) === */
	for (i = 0; i < size; i++)
		MEM32_BASE[i] = 0;

	for (i = 0; i < size; i++) {
		if (MEM32_BASE[i] != 0) {
			vfree(ptr);
			return -1;
		} else {
			MEM32_BASE[i] = 0xffffffff;
		}
	}

	/* === Verify the tied bits (tied low) === */
	for (i = 0; i < size; i++) {
		if (MEM32_BASE[i] != 0xffffffff) {
			vfree(ptr);
			return -2;
		} else
			MEM32_BASE[i] = 0x00;
	}

	/* === Verify pattern 1 (0x00~0xff) === */
	pattern8 = 0x00;
	for (i = 0; i < len; i++)
		MEM8_BASE[i] = pattern8++;
	pattern8 = 0x00;
	for (i = 0; i < len; i++) {
		if (MEM8_BASE[i] != pattern8++) {
			vfree(ptr);
			return -3;
		}
	}

	/* === Verify pattern 2 (0x00~0xff) === */
	pattern8 = 0x00;
	for (i = j = 0; i < len; i += 2, j++) {
		if (MEM8_BASE[i] == pattern8)
			MEM16_BASE[j] = pattern8;
		if (MEM16_BASE[j] != pattern8) {
			vfree(ptr);
			return -4;
		}
		pattern8 += 2;
	}

	/* === Verify pattern 3 (0x00~0xffff) === */
	pattern16 = 0x00;
	for (i = 0; i < (len >> 1); i++)
		MEM16_BASE[i] = pattern16++;
	pattern16 = 0x00;
	for (i = 0; i < (len >> 1); i++) {
		if (MEM16_BASE[i] != pattern16++) {
			vfree(ptr);
			return -5;
		}
	}

	/* === Verify pattern 4 (0x00~0xffffffff) === */
	pattern32 = 0x00;
	for (i = 0; i < (len >> 2); i++)
		MEM32_BASE[i] = pattern32++;
	pattern32 = 0x00;
	for (i = 0; i < (len >> 2); i++) {
		if (MEM32_BASE[i] != pattern32++) {
			vfree(ptr);
			return -6;
		}
	}

	/* === Pattern 5: Filling memory range with 0x44332211 === */
	for (i = 0; i < size; i++)
		MEM32_BASE[i] = 0x44332211;

	/* === Read Check then Fill Memory with a5a5a5a5 Pattern === */
	for (i = 0; i < size; i++) {
		if (MEM32_BASE[i] != 0x44332211) {
			vfree(ptr);
			return -7;
		} else {
			MEM32_BASE[i] = 0xa5a5a5a5;
		}
	}

	/* === Read Check then Fill Memory with 00 Byte Pattern at offset 0h === */
	for (i = 0; i < size; i++) {
		if (MEM32_BASE[i] != 0xa5a5a5a5) {
			vfree(ptr);
			return -8;
		} else {
			MEM8_BASE[i * 4] = 0x00;
		}
	}

	/* === Read Check then Fill Memory with 00 Byte Pattern at offset 2h === */
	for (i = 0; i < size; i++) {
		if (MEM32_BASE[i] != 0xa5a5a500) {
			vfree(ptr);
			return -9;
		} else {
			MEM8_BASE[i * 4 + 2] = 0x00;
		}
	}

	/* === Read Check then Fill Memory with 00 Byte Pattern at offset 1h === */
	for (i = 0; i < size; i++) {
		if (MEM32_BASE[i] != 0xa500a500) {
			vfree(ptr);
			return -10;
		} else {
			MEM8_BASE[i * 4 + 1] = 0x00;
		}
	}

	/* === Read Check then Fill Memory with 00 Byte Pattern at offset 3h === */
	for (i = 0; i < size; i++) {
		if (MEM32_BASE[i] != 0xa5000000) {
			vfree(ptr);
			return -11;
		} else {
			MEM8_BASE[i * 4 + 3] = 0x00;
		}
	}

	/* === Read Check then Fill Memory with ffff Word Pattern at offset 1h == */
	for (i = 0; i < size; i++) {
		if (MEM32_BASE[i] != 0x00000000) {
			vfree(ptr);
			return -12;
		} else {
			MEM16_BASE[i * 2 + 1] = 0xffff;
		}
	}

	/* === Read Check then Fill Memory with ffff Word Pattern at offset 0h == */
	for (i = 0; i < size; i++) {
		if (MEM32_BASE[i] != 0xffff0000) {
			vfree(ptr);
			return -13;
		} else {
			MEM16_BASE[i * 2] = 0xffff;
		}
	}
    /*===  Read Check === */
	for (i = 0; i < size; i++) {
		if (MEM32_BASE[i] != 0xffffffff) {
			vfree(ptr);
			return -14;
		}
	}

    /************************************************
    * Additional verification
    ************************************************/
	/* === stage 1 => write 0 === */

	for (i = 0; i < size; i++)
		MEM_BASE[i] = PATTERN1;

	/* === stage 2 => read 0, write 0xF === */
	for (i = 0; i < size; i++) {
		value = MEM_BASE[i];

		if (value != PATTERN1) {
			vfree(ptr);
			return -15;
		}
		MEM_BASE[i] = PATTERN2;
	}

	/* === stage 3 => read 0xF, write 0 === */
	for (i = 0; i < size; i++) {
		value = MEM_BASE[i];
		if (value != PATTERN2) {
			vfree(ptr);
			return -16;
		}
		MEM_BASE[i] = PATTERN1;
	}

	/* === stage 4 => read 0, write 0xF === */
	for (i = 0; i < size; i++) {
		value = MEM_BASE[i];
		if (value != PATTERN1) {
			vfree(ptr);
			return -17;
		}
		MEM_BASE[i] = PATTERN2;
	}

	/* === stage 5 => read 0xF, write 0 === */
	for (i = 0; i < size; i++) {
		value = MEM_BASE[i];
		if (value != PATTERN2) {
			vfree(ptr);
			return -18;
		}
		MEM_BASE[i] = PATTERN1;
	}

	/* === stage 6 => read 0 === */
	for (i = 0; i < size; i++) {
		value = MEM_BASE[i];
		if (value != PATTERN1) {
			vfree(ptr);
			return -19;
		}
	}

	/* === 1/2/4-byte combination test === */
	mem_ptr = (unsigned long)MEM_BASE;
	while (mem_ptr < ((unsigned long)MEM_BASE + (size << 2))) {
		*((unsigned char *)mem_ptr) = 0x78;
		mem_ptr += 1;
		*((unsigned char *)mem_ptr) = 0x56;
		mem_ptr += 1;
		*((unsigned short *)mem_ptr) = 0x1234;
		mem_ptr += 2;
		*((unsigned int *)mem_ptr) = 0x12345678;
		mem_ptr += 4;
		*((unsigned short *)mem_ptr) = 0x5678;
		mem_ptr += 2;
		*((unsigned char *)mem_ptr) = 0x34;
		mem_ptr += 1;
		*((unsigned char *)mem_ptr) = 0x12;
		mem_ptr += 1;
		*((unsigned int *)mem_ptr) = 0x12345678;
		mem_ptr += 4;
		*((unsigned char *)mem_ptr) = 0x78;
		mem_ptr += 1;
		*((unsigned char *)mem_ptr) = 0x56;
		mem_ptr += 1;
		*((unsigned short *)mem_ptr) = 0x1234;
		mem_ptr += 2;
		*((unsigned int *)mem_ptr) = 0x12345678;
		mem_ptr += 4;
		*((unsigned short *)mem_ptr) = 0x5678;
		mem_ptr += 2;
		*((unsigned char *)mem_ptr) = 0x34;
		mem_ptr += 1;
		*((unsigned char *)mem_ptr) = 0x12;
		mem_ptr += 1;
		*((unsigned int *)mem_ptr) = 0x12345678;
		mem_ptr += 4;
	}
	for (i = 0; i < size; i++) {
		value = MEM_BASE[i];
		if (value != 0x12345678) {
			vfree(ptr);
			return -20;
		}
	}

	/* === Verify pattern 1 (0x00~0xff) === */
	pattern8 = 0x00;
	MEM8_BASE[0] = pattern8;
	for (i = 0; i < size * 4; i++) {
		unsigned char waddr8, raddr8;
		waddr8 = i + 1;
		raddr8 = i;
		if (i < size * 4 - 1)
			MEM8_BASE[waddr8] = pattern8 + 1;
		if (MEM8_BASE[raddr8] != pattern8) {
			vfree(ptr);
			return -21;
		}
		pattern8++;
	}

	/* === Verify pattern 2 (0x00~0xffff) === */
	pattern16 = 0x00;
	MEM16_BASE[0] = pattern16;
	for (i = 0; i < size * 2; i++) {
		if (i < size * 2 - 1)
			MEM16_BASE[i + 1] = pattern16 + 1;
		if (MEM16_BASE[i] != pattern16) {
			vfree(ptr);
			return -22;
		}
		pattern16++;
	}
	/* === Verify pattern 3 (0x00~0xffffffff) === */
	pattern32 = 0x00;
	MEM32_BASE[0] = pattern32;
	for (i = 0; i < size; i++) {
		if (i < size - 1)
			MEM32_BASE[i + 1] = pattern32 + 1;
		if (MEM32_BASE[i] != pattern32) {
			vfree(ptr);
			return -23;
		}
		pattern32++;
	}
	pr_warn("complex R/W mem test pass\n");
	vfree(ptr);
	return 1;
}

U32 ucDram_Register_Read(unsigned long u4reg_addr)
{
	U32 pu4reg_value;

	pu4reg_value = (*(volatile unsigned int *)(DRAMCAO_BASE_ADDR + (u4reg_addr))) |
					(*(volatile unsigned int *)(DDRPHY_BASE_ADDR + (u4reg_addr))) |
					(*(volatile unsigned int *)(DRAMCNAO_BASE_ADDR + (u4reg_addr)));
	return pu4reg_value;
}

void ucDram_Register_Write(unsigned long u4reg_addr, unsigned int u4reg_value)
{
	(*(volatile unsigned int *)(DRAMCAO_BASE_ADDR + (u4reg_addr))) = u4reg_value;
	(*(volatile unsigned int *)(DDRPHY_BASE_ADDR + (u4reg_addr))) = u4reg_value;
	(*(volatile unsigned int *)(DRAMCNAO_BASE_ADDR + (u4reg_addr))) = u4reg_value;
	dsb();
}

unsigned int dram_support_1600_freq(void)
{
	unsigned int spbin = ((get_devinfo_with_index(5) & (1<<20)) && (get_devinfo_with_index(5) & (1<<21)) &&
				(get_devinfo_with_index(5) & (1<<22))) ? 1 : 0;	/* MT6753 or MT6753T */
	unsigned int ddr1466 = (get_devinfo_with_index(15) & (1<<8)) ? 1 : 0;	/* DDR1466 or DDR1600 */
	unsigned result = 0;

	if (spbin == 1) {
		if (ddr1466 == 1)
			result = 0;
		else
			result = 1;
	} else
		result = 0;

	return result;
}

int dram_fh_steps_freq(unsigned int step)
{
	int freq;

#if defined(CONFIG_ARCH_MT6735)
	switch (step) {
	case 0:
		freq = 1280;
		break;
	case 1:
		freq = 938;
		break;
	default:
		return -1;
	}
#elif defined(CONFIG_ARCH_MT6735M)
	switch (step) {
	case 0:
		freq = 1066;
		break;
	case 1:
		freq = 800;
		break;
	default:
		return -1;
	}
#elif defined(CONFIG_ARCH_MT6753)
	unsigned int temp = dram_support_1600_freq();

	switch (step) {
	case 0:
		/* need porting to eFuse check for MT6753T (1600) */
		if (temp == 1)
			freq = 1600;
		else
			freq = 1466;
		break;
	case 1:
		freq = 1313;
		break;
	default:
		return -1;
	}
#else
	pr_warn("DRAM FH steps not define\n");
#endif

	return freq;
}

int dram_can_support_fh(void)
{
	unsigned int value;

	value = ucDram_Register_Read(0xf4);
	/*value = (0x1 <<15);
	pr_warn("dummy regs 0x%x\n", value);
	pr_warn("check 0x%x\n",(0x1 <<15));*/

	if (value & (0x1 << 15)) {
		/*pr_warn("DFS could be enabled\n");*/
		return 1;
	} else {
		/*pr_warn("DFS could NOT be enabled\n");*/
		return 0;
	}
}

#ifdef FREQ_HOPPING_TEST
int dram_do_dfs_by_fh(unsigned int freq)
{
	int detect_fh = dram_can_support_fh();
	unsigned int target_dds;
	unsigned int ddr_type = get_ddr_type();

	if (detect_fh == 0)
		return -1;

	switch (freq) {
	case 800000:
		target_dds = 0xe55ee;	/* ///< 800Mbps */
		break;
	case 900000:
		target_dds = 0xdbba0;	/* ///< 900Mbps */
		break;
	case 938000:
		if (ddr_type == TYPE_LPDDR2)
			target_dds = 0x10cefe;	/* ///< 938Mbps for LPDDR2 */
		else
			target_dds = 0xe38fe;	/* ///< 938Mbps for LPDDR3 */
		break;
	case 1066000:
		target_dds = 0x131A2D;	/* ///< 1066Mbps */
		break;
	case 1280000:
		target_dds = 0x136885;	/* ///< 1280Mbps */
		break;
	case 1313000:
		target_dds = 0x127C92;	/* ///< 1313Mbps */
		break;
	case 1333000:
		target_dds = 0x13f01A;	/* ///< 1333Mbps */
		break;
	case 1466000:
		target_dds = 0x14A40B;	/* ///< 1466Mbps */
		break;
	case 1600000:
		target_dds = 0x168708;	/* ///< 1600Mbps */
		break;
	default:
		return -1;
	}

	/*pr_warn("dram_do_dfs_by_fh to 0x%x ddr_type = %d\n", target_dds, ddr_type);*/
	enable_clock(MT_CG_INFRA_GCE, "CQDMA");
	mt_dfs_mempll(target_dds);
	disable_clock(MT_CG_INFRA_GCE, "CQDMA");

	/*Notify profiling engine */
	mt_dramfreq_setfreq_notify(freq);

	return 0;
}
#endif

bool pasr_is_valid(void)
{
	unsigned int ddr_type = 0;

	ddr_type = get_ddr_type();
	/* Following DDR types can support PASR */
	if ((ddr_type == TYPE_LPDDR3) || (ddr_type == TYPE_LPDDR2))
		return true;

	return false;
}

/*-------------------------------------------------------------------------*/
/** Round_Operation
 *  Round operation of A/B
 *  @param  A
 *  @param  B
 *  @retval round(A/B)
 */
/*-------------------------------------------------------------------------*/
U32 Round_Operation(U32 A, U32 B)
{
	U32 temp;

	if (B == 0)
		return 0xffffffff;

	temp = A / B;

	if ((A - temp * B) >= ((temp + 1) * B - A))
		return temp++;
	else
		return temp;
}

int get_dram_data_rate(void)
{
	unsigned int MEMPLL1_DIV, MEMPLL1_NCPO, MEMPLL1_FOUT;
	unsigned int MEMPLL2_FOUT, MEMPLL2_FBSEL, MEMPLL2_FBDIV;
	unsigned int MEMPLL2_M4PDIV;

	MEMPLL1_DIV = (ucDram_Register_Read(0x0604) & (0x0000007f << 25)) >> 25;
	MEMPLL1_NCPO = (ucDram_Register_Read(0x0624) & (0x7fffffff << 1)) >> 1;
	MEMPLL2_FBSEL = (ucDram_Register_Read(0x0608) & (0x00000003 << 10)) >> 10;
	MEMPLL2_FBSEL = 1 << MEMPLL2_FBSEL;
	MEMPLL2_FBDIV = (ucDram_Register_Read(0x0618) & (0x0000007f << 2)) >> 2;
	MEMPLL2_M4PDIV = (ucDram_Register_Read(0x060c) & (0x00000003 << 10)) >> 10;
	MEMPLL2_M4PDIV = 1 << (MEMPLL2_M4PDIV + 1);

	/*  1PLL:  26*MEMPLL1_NCPO/MEMPLL1_DIV*MEMPLL2_FBSEL*MEMPLL2_FBDIV/2^24 */
	/*  3PLL:  26*MEMPLL1_NCPO/MEMPLL1_DIV*MEMPLL2_M4PDIV*MEMPLL2_FBDIV*2/2^24 */

	MEMPLL1_FOUT = (MEMPLL1_NCPO / MEMPLL1_DIV) * 26;
	if ((ucDram_Register_Read(0x0640) & 0x3) == 3) {
		/*  1PLL */
		MEMPLL2_FOUT = (((MEMPLL1_FOUT * MEMPLL2_FBSEL) >> 12) * MEMPLL2_FBDIV) >> 12;
	} else {
		/*  2 or 3 PLL */
		MEMPLL2_FOUT = (((MEMPLL1_FOUT * MEMPLL2_M4PDIV * 2) >> 12) * MEMPLL2_FBDIV) >> 12;
	}

	/* pr_warn("MEMPLL1_DIV=%d, MEMPLL1_NCPO=%d, MEMPLL2_FBSEL=%d, MEMPLL2_FBDIV=%d\n",
			MEMPLL1_DIV, MEMPLL1_NCPO, MEMPLL2_FBSEL, MEMPLL2_FBDIV); */
	/* pr_warn("MEMPLL2_M4PDIV=%d, MEMPLL1_FOUT=%d, MEMPLL2_FOUT=%d\n",
			MEMPLL2_M4PDIV, MEMPLL1_FOUT, MEMPLL2_FOUT); */

	/*  workaround (Darren) */
	MEMPLL2_FOUT++;

	switch (MEMPLL2_FOUT) {
	case 800:
	case 938:
	case 1066:
	case 1280:
	case 1313:
	case 1466:
	case 1600:
		break;

	default:
		pr_warn("[DRAMC] MemFreq region is incorrect MEMPLL2_FOUT=%d\n", MEMPLL2_FOUT);
		/* return -1; */
	}

	return MEMPLL2_FOUT;
}

unsigned int DRAM_MRR(int MRR_num)
{
	unsigned int MRR_value = 0x0;
	unsigned int u4value;

	/*  set DQ bit 0, 1, 2, 3, 4, 5, 6, 7 pinmux for LPDDR3 */
	ucDram_Register_Write(DRAMC_REG_RRRATE_CTL, 0x13121110);
	ucDram_Register_Write(DRAMC_REG_MRR_CTL, 0x17161514);

	ucDram_Register_Write(DRAMC_REG_MRS, MRR_num);
	ucDram_Register_Write(DRAMC_REG_SPCMD, ucDram_Register_Read(DRAMC_REG_SPCMD) | 0x00000002);
	/* udelay(1); */
	while ((ucDram_Register_Read(DRAMC_REG_SPCMDRESP) & 0x02) == 0)
		;
	ucDram_Register_Write(DRAMC_REG_SPCMD, ucDram_Register_Read(DRAMC_REG_SPCMD) & 0xFFFFFFFD);

	u4value = ucDram_Register_Read(DRAMC_REG_SPCMDRESP);
	MRR_value = (u4value >> 20) & 0xFF;

	return MRR_value;
}

unsigned int read_dram_temperature(void)
{
	unsigned int value;

	value = DRAM_MRR(4) & 0x7;
	return value;
}

int get_ddr_type(void)
{
	unsigned int value;

	value = ucDram_Register_Read(DRAMC_LPDDR2);
	if ((value >> 28) & 0x1) {	/* check LPDDR2_EN */
		return TYPE_LPDDR2;
	}

	value = ucDram_Register_Read(DRAMC_PADCTL4);
	if ((value >> 7) & 0x1) {	/* check DDR3_EN */
		return TYPE_PCDDR3;
	}

	value = ucDram_Register_Read(DRAMC_ACTIM1);
	if ((value >> 28) & 0x1) {	/* check LPDDR3_EN */
		return TYPE_LPDDR3;
	}

	return TYPE_DDR1;
}

#if 0
volatile int shuffer_done;

void dram_dfs_ipi_handler(int id, void *data, unsigned int len)
{
	shuffer_done = 1;
}
#endif


#if 0
__attribute__ ((__section__(".sram.func"))) void uart_print(unsigned char ch)
{
	int i;
	for (i = 0; i < 5; i++)
		(*(volatile unsigned int *)(0xF1003000)) = ch;
}
#endif

#ifdef FREQ_HOPPING_TEST
static ssize_t freq_hopping_test_show(struct device_driver *driver, char *buf)
{
	int dfs_ability = 0;
	unsigned int temp = dram_support_1600_freq();

	dfs_ability = dram_can_support_fh();

	if (dfs_ability == 0)
		return snprintf(buf, PAGE_SIZE, "DRAM DFS can not be enabled, 1600 = %d\n", temp);
	else
		return snprintf(buf, PAGE_SIZE, "DRAM DFS can be enabled, 1600 = %d\n", temp);
}

static ssize_t freq_hopping_test_store(struct device_driver *driver,
				       const char *buf, size_t count)
{
	unsigned int freq;

	if (sscanf(buf, "%u", &freq) != 1)
		return -EINVAL;

	pr_warn("[DRAM DFS] freqency hopping to %dKHz\n", freq);
	dram_do_dfs_by_fh(freq);

	return count;
}
#endif

#ifdef DFS_TEST
void do_DRAM_DFS(int high_freq)
{
	U8 ucstatus = 0;
	U32 u4value, u4HWTrackR0, u4HWTrackR1, u4HWGatingEnable;
	int ddr_type;

	mutex_lock(&dram_dfs_mutex);

	ddr_type = get_ddr_type();
	switch (ddr_type) {
	case TYPE_DDR1:
		pr_warn("not support DDR1\n");
		BUG();
	case TYPE_LPDDR2:
		pr_warn("[DRAM DFS] LPDDR2\n");
		break;
	case TYPE_LPDDR3:
		pr_warn("[DRAM DFS] LPDDR3\n");
		break;
	case TYPE_PCDDR3:
		pr_warn("not support PCDDR3\n");
		BUG();
	default:
		BUG();
	}

	if (high_freq == 1)
		pr_warn("[DRAM DFS] Switch to high frequency\n");
	else
		pr_warn("[DRAM DFS] switch to low frequency\n");

	/* DramcEnterSelfRefresh(p, 1); // enter self refresh */
	/* mcDELAY_US(1); */
	/* Read back HW tracking first. After shuffle finish, need to copy this value into SW fine tune. */

	if (Reg_Readl((CHA_DRAMCAO_BASE + 0x1c0)) & 0x80000000)
		u4HWGatingEnable = 1;
	else
		u4HWGatingEnable = 0;

	if (u4HWGatingEnable) {
		Reg_Sync_Writel((CHA_DRAMCAO_BASE + 0x028),
				Reg_Readl((CHA_DRAMCAO_BASE + 0x028)) & (~(0x01 << 30)));	/*  cha DLLFRZ=0 */
		u4HWTrackR0 = Reg_Readl((CHA_DRAMCNAO_BASE + 0x374));	/*  cha r0 */
		u4HWTrackR1 = Reg_Readl((CHA_DRAMCNAO_BASE + 0x378));	/*  cha r1 */
		Reg_Sync_Writel((CHA_DRAMCAO_BASE + 0x028),
				Reg_Readl((CHA_DRAMCAO_BASE + 0x028)) | (0x01 << 30));	/*  cha DLLFRZ=1 */
	}
	/* shuffer_done = 0; */

	if (high_freq == 1) {
		/*  Shuffle to high */
#ifndef SHUFFER_BY_MD32
		U32 read_data;
		U32 bak_data1;
		U32 bak_data2;
		U32 bak_data3;
		U32 bak_data4;
#else
		U32 shuffer_to_high = 1;
#endif

		if (u4HWGatingEnable) {
			/*  Current is low frequency. Save to low frequency fine tune here */
			/* because shuffle enable will cause HW GW reload. */
			Reg_Sync_Writel((CHA_DRAMCAO_BASE + 0x840), u4HWTrackR0);
			Reg_Sync_Writel((CHA_DRAMCAO_BASE + 0x844), u4HWTrackR1);
		}
		/*  MR2 RL/WL set */
#ifdef DUAL_FREQ_DIFF_RLWL
		ucDram_Register_Write(0x088, (ddr_type == TYPE_LPDDR3) ? LPDDR3_MODE_REG_2 : LPDDR2_MODE_REG_2);
#else
		ucDram_Register_Write(0x088, (ddr_type == TYPE_LPDDR3) ? LPDDR3_MODE_REG_2 : LPDDR2_MODE_REG_2);
#endif

#ifndef SHUFFER_BY_MD32
		bak_data1 = Reg_Readl((CHA_DRAMCAO_BASE + (0x77 << 2)));
		Reg_Sync_Writel((CHA_DRAMCAO_BASE + (0x77 << 2)), bak_data1 & ~(0xc0000000));

		Reg_Sync_Writel((CLK_CFG_0_CLR), 0x300);
		Reg_Sync_Writel((CLK_CFG_0_SET), 0x100);

		bak_data2 = Reg_Readl((CHA_DDRPHY_BASE + (0x190 << 2)));
		bak_data4 = Reg_Readl((PCM_INI_PWRON0_REG));

		/*  Shuffle to high start. */
		/*  Reg.28h[17]=1 R_DMSHU_DRAMC */
		/*  Reg.28h[16]=0 R_DMSHU_LOW */
		bak_data3 = Reg_Readl((CHA_DRAMCAO_BASE + (0x00a << 2)));

		/*  Block EMI start. */
		Reg_Sync_Writel((CHA_DRAMCAO_BASE + (0x00a << 2)),
				(bak_data3 & (~0x00030000)) | 0x20000);

		/*  Wait shuffle_end Reg.16ch[0] == 1 */
		read_data = Reg_Readl((CHA_DRAMCAO_BASE + (0x05b << 2)));
		while ((read_data & 0x01) != 0x01)
			read_data = Reg_Readl((CHA_DRAMCAO_BASE + (0x05b << 2)));

		/*  [3:0]=1 : VCO/4 . [4]=0 : RG_MEMPLL_ONEPLLSEL. [12:5] RG_MEMPLL_RSV. RG_MEMPLL_RSV[1]=1 */
		Reg_Sync_Writel((CHA_DDRPHY_BASE + (0x1a6 << 2)), 0x00001e41);
		Reg_Sync_Writel((PCM_INI_PWRON0_REG), bak_data4 & (~0x8000000));
		/*  K2?? *(volatile unsigned int*)(0x10006000) = 0x0b160001?? */

		mcDELAY_US(10);	/*  Wait 1us. */

		Reg_Sync_Writel((CLK_CFG_UPDATE), 0x02);

		Reg_Sync_Writel((CHA_DDRPHY_BASE + (0x190 << 2)), bak_data2 & (~0x01));	/* sync = 1 */
		Reg_Sync_Writel((CHA_DDRPHY_BASE + (0x190 << 2)), bak_data2 | 0x01);
		/*  sync back to original. Should be 0. Bu in case of SPM control, */
		/* need to make sure SPM is not toggling. */

		/* block EMI end. */
		Reg_Sync_Writel((CHA_DRAMCAO_BASE + (0x00a << 2)), bak_data3 & (~0x30000));

		Reg_Sync_Writel((CHA_DRAMCAO_BASE + (0x77 << 2)), bak_data1);
#else
		while (md32_ipi_send(IPI_DFS, (void *)&shuffer_to_high, sizeof(U32), 1) != DONE)
			;

		/* while(shuffer_done == 0); */
		pr_warn("[DRAM DFS] Shuffer to high by MD32\n");
#endif

		mt_dcm_emi_3pll_mode();

		/*  [3:0]=1 : VCO/4 . [4]=0 : RG_MEMPLL_ONEPLLSEL. [12:5] RG_MEMPLL_RSV. */
		/* RG_MEMPLL_RSV[1]=0 ==> disable output. */
		Reg_Sync_Writel((CHA_DDRPHY_BASE + (0x1a6 << 2)), 0x00001e01);
	} else {
		/*  Shuffle to low */
#ifndef SHUFFER_BY_MD32
		U32 read_data;
		U32 bak_data1;
		U32 bak_data2;
		U32 bak_data3;
#else
		U32 shuffer_to_high = 0;
#endif

		if (u4HWGatingEnable) {
			/*  Current is low frequency. Save to high frequency fine tune here */
			/* because shuffle enable will cause HW GW reload. */
			Reg_Sync_Writel((CHA_DRAMCAO_BASE + 0x94), u4HWTrackR0);
			Reg_Sync_Writel((CHA_DRAMCAO_BASE + 0x98), u4HWTrackR1);
		}
		/*  MR2 RL/WL set */
#ifdef DUAL_FREQ_DIFF_RLWL
		ucDram_Register_Write(0x088, (ddr_type == TYPE_LPDDR3) ? LPDDR3_MODE_REG_2_LOW : LPDDR2_MODE_REG_2_LOW);
#else
		ucDram_Register_Write(0x088, (ddr_type == TYPE_LPDDR3) ? LPDDR3_MODE_REG_2 : LPDDR2_MODE_REG_2);
#endif

		/*  [3:0]=1 : VCO/4 . [4]=0 : RG_MEMPLL_ONEPLLSEL. */
		/* [12:5] RG_MEMPLL_RSV. RG_MEMPLL_RSV[1]=1 */
		Reg_Sync_Writel((CHA_DDRPHY_BASE + (0x1a6 << 2)), 0x00001e41);

		Reg_Sync_Writel((CHA_DDRPHY_BASE + (0x186 << 2)),
				Reg_Readl(CHA_DDRPHY_BASE + (0x186 << 2)) | 0x10000);
		/*  Switch MEMPLL2 reset mode select */
		Reg_Sync_Writel((CHA_DDRPHY_BASE + (0x189 << 2)),
				Reg_Readl(CHA_DDRPHY_BASE + (0x189 << 2)) | 0x10000);
		/*  Switch MEMPLL3 reset mode select */

		mt_dcm_emi_1pll_mode();

#ifndef SHUFFER_BY_MD32
		bak_data1 = Reg_Readl((CHA_DRAMCAO_BASE + (0x20a << 2)));
		Reg_Sync_Writel((CHA_DRAMCAO_BASE + (0x20a << 2)), bak_data1 & (~0xc0000000));

		Reg_Sync_Writel((CLK_CFG_0_CLR), 0x300);
		Reg_Sync_Writel((CLK_CFG_0_SET), 0x200);

		bak_data2 = Reg_Readl((CHA_DDRPHY_BASE + (0x190 << 2)));

		/*  Shuffle to low. */
		/*  Reg.28h[17]=1 R_DMSHU_DRAMC */
		/*  Reg.28h[16]=1 R_DMSHU_LOW */
		bak_data3 = Reg_Readl((CHA_DRAMCAO_BASE + (0x00a << 2)));

		/*  Block EMI start. */
		Reg_Sync_Writel((CHA_DRAMCAO_BASE + (0x00a << 2)), bak_data3 | 0x30000);

		/*  Wait shuffle_end Reg.16ch[0] == 1 */
		read_data = Reg_Readl((CHA_DRAMCAO_BASE + (0x05b << 2)));
		while ((read_data & 0x01) != 0x01)
			read_data = Reg_Readl((CHA_DRAMCAO_BASE + (0x05b << 2)));

		/*  [3:0]=1 : VCO/4 . [4]=1 : RG_MEMPLL_ONEPLLSEL. [12:5] RG_MEMPLL_RSV. RG_MEMPLL_RSV[1]=1. [15]=1?? */
		Reg_Sync_Writel((CHA_DDRPHY_BASE + (0x1a6 << 2)), 0x00009e51);

		Reg_Sync_Writel((CLK_CFG_UPDATE), 0x02);

		Reg_Sync_Writel((CHA_DDRPHY_BASE + (0x190 << 2)), bak_data2 & (~0x01));
		Reg_Sync_Writel((CHA_DDRPHY_BASE + (0x190 << 2)), bak_data2 | 0x01);

		/*  Block EMI end. */
		Reg_Sync_Writel((CHA_DRAMCAO_BASE + (0x00a << 2)), bak_data3 & (~0x30000));
		Reg_Sync_Writel((CHA_DRAMCAO_BASE + (0x20a << 2)), bak_data1);

		Reg_Sync_Writel((PCM_INI_PWRON0_REG), Reg_Readl(PCM_INI_PWRON0_REG) | 0x8000000);
#else
		while (md32_ipi_send(IPI_DFS, (void *)&shuffer_to_high, sizeof(U32), 1) != DONE)
			;

		/* while(shuffer_done == 0); */
		pr_warn("[DRAM DFS] Shuffer to low by MD32\n");
#endif
	}

	mutex_unlock(&dram_dfs_mutex);

	/* DramcEnterSelfRefresh(p, 0); // exit self refresh */
}
#endif

int DFS_APDMA_early_init(void)
{
	phys_addr_t max_dram_size = get_max_DRAM_size();
	phys_addr_t dummy_read_center_address = 0;

	if (init_done == 0) {
		if (max_dram_size == 0x100000000ULL) {	/* dram size = 4GB */
			dummy_read_center_address = 0x80000000ULL;
		} else if (max_dram_size <= 0xC0000000)	{	/* dram size <= 3GB */
			dummy_read_center_address = DRAM_BASE + (max_dram_size >> 1);
		} else {
			ASSERT(0);
		}

		src_array_p = (volatile unsigned int)(dummy_read_center_address - (BUFF_LEN >> 1));
		dst_array_p = __pa(dfs_dummy_buffer);
		pr_warn("[DFS]dfs_dummy_buffer va: 0x%p, dst_pa: 0x%llx, src_pa: 0x%llx  size: %d\n",
				(void *)dfs_dummy_buffer, (unsigned long long)dst_array_p,
				(unsigned long long)src_array_p, BUFF_LEN);

#ifdef APDMAREG_DUMP
		src_array_v =
				ioremap(rounddown(src_array_p, IOREMAP_ALIGMENT),
			    IOREMAP_ALIGMENT << 1) + IOREMAP_ALIGMENT -
				(BUFF_LEN >> 1);
		dst_array_v = src_array_v + BUFF_LEN;
#endif
		/* memset(src_array_v, 0x6a6a6a6a, BUFF_LEN); */

		init_done = 1;
	}

	return 1;
}

int DFS_APDMA_Init(void)
{
	writel(((~DMA_GSEC_EN_BIT) & readl(DMA_GSEC_EN)), DMA_GSEC_EN);
	return 1;
}

int DFS_APDMA_Enable(void)
{
#ifdef APDMAREG_DUMP
	int i;
#endif

	while (readl(DMA_START) & 0x1)
		;
	writel(src_array_p, DMA_SRC);
	writel(dst_array_p, DMA_DST);
	writel(BUFF_LEN, DMA_LEN1);
	writel(DMA_CON_BURST_8BEAT, DMA_CON);

#ifdef APDMAREG_DUMP
	pr_warn("src_p=0x%x, dst_p=0x%x, src_v=0x%x, dst_v=0x%x, len=%d\n",
	       src_array_p, dst_array_p, (unsigned int)src_array_v,
	       (unsigned int)dst_array_v, BUFF_LEN);
	for (i = 0; i < 0x60; i += 4) {
		pr_warn("[Before]addr:0x%x, value:%x\n",
		       (unsigned int)(DMA_BASE + i),
		       *((volatile int *)(DMA_BASE + i)));
	}

#ifdef APDMA_TEST
	for (i = 0; i < BUFF_LEN / sizeof(unsigned int); i++) {
		dst_array_v[i] = 0;
		src_array_v[i] = i;
	}
#endif
#endif

	mt_reg_sync_writel(0x1, DMA_START);

#ifdef APDMAREG_DUMP
	for (i = 0; i < 0x60; i += 4) {
		pr_warn("[AFTER]addr:0x%x, value:%x\n",
		       (unsigned int)(DMA_BASE + i),
		       *((volatile int *)(DMA_BASE + i)));
	}

#ifdef APDMA_TEST
	for (i = 0; i < BUFF_LEN / sizeof(unsigned int); i++) {
		if (dst_array_v[i] != src_array_v[i]) {
			pr_warn("DMA ERROR at Address %x\n (i=%d, value=0x%x(should be 0x%x))",
				(unsigned int)&dst_array_v[i], i, dst_array_v[i],
				src_array_v[i]);
			ASSERT(0);
		}
	}
	pr_warn("Channe0 DFS DMA TEST PASS\n");
#endif
#endif
	return 1;
}

int DFS_APDMA_END(void)
{
	while (readl(DMA_START))
		;
	return 1;
}

void DFS_APDMA_dummy_read_preinit(void)
{
	DFS_APDMA_early_init();
}

void DFS_APDMA_dummy_read_deinit(void)
{
}

void dma_dummy_read_for_vcorefs(int loops)
{
	int i, count;
	unsigned long long start_time, end_time, duration;

	DFS_APDMA_early_init();
	enable_clock(MT_CG_INFRA_GCE, "CQDMA");
	for (i = 0; i < loops; i++) {
		count = 0;
		start_time = sched_clock();
		do {
			DFS_APDMA_Enable();
			DFS_APDMA_END();
			end_time = sched_clock();
			duration = end_time - start_time;
			count++;
		} while (duration < 4000L);
		/* pr_warn("[DMA_dummy_read[%d], duration=%lld, count = %d\n", duration, count); */
	}
	disable_clock(MT_CG_INFRA_GCE, "CQDMA");
}

#if 0
/*
 * XXX: Reserved memory in low memory must be 1MB aligned.
 *     This is because the Linux kernel always use 1MB section to map low memory.
 *
 *    We Reserved the memory regien which could cross rank for APDMA to do dummy read.
 *
 */

void DFS_Reserved_Memory(void)
{
	phys_addr_t high_memory_phys;
	phys_addr_t DFS_dummy_read_center_address;
	phys_addr_t max_dram_size = get_max_DRAM_size();

	high_memory_phys = virt_to_phys(high_memory);

	if (max_dram_size == 0x100000000ULL) {	/* dram size = 4GB */
		DFS_dummy_read_center_address = 0x80000000ULL;
	} else if (max_dram_size <= 0xC0000000) {	/* dram size <= 3GB */
		DFS_dummy_read_center_address = DRAM_BASE + (max_dram_size >> 1);
	} else {
		ASSERT(0);
	}

	/*For DFS Purpose, we remove this memory block for Dummy read/write by APDMA. */
	pr_warn("[DFS Check]DRAM SIZE:0x%llx\n",
	       (unsigned long long)max_dram_size);
	pr_warn("[DFS Check]DRAM Dummy read from:0x%llx to 0x%llx\n",
			(unsigned long long)(DFS_dummy_read_center_address - (BUFF_LEN >> 1)),
			(unsigned long long)(DFS_dummy_read_center_address + (BUFF_LEN >> 1)));
	pr_warn("[DFS Check]DRAM Dummy read center address:0x%llx\n",
	       (unsigned long long)DFS_dummy_read_center_address);
	pr_warn("[DFS Check]High Memory start address 0x%llx\n",
	       (unsigned long long)high_memory_phys);

	if ((DFS_dummy_read_center_address - SZ_4K) >= high_memory_phys) {
		pr_warn("[DFS Check]DFS Dummy read reserved 0x%llx to 0x%llx\n",
		       (unsigned long long)(DFS_dummy_read_center_address - SZ_4K),
		       (unsigned long long)(DFS_dummy_read_center_address + SZ_4K));
		memblock_reserve(DFS_dummy_read_center_address - SZ_4K, (SZ_4K << 1));
		memblock_free(DFS_dummy_read_center_address - SZ_4K, (SZ_4K << 1));
		memblock_remove(DFS_dummy_read_center_address - SZ_4K, (SZ_4K << 1));
	} else {
#ifndef CONFIG_ARM_LPAE
		pr_warn("[DFS Check]DFS Dummy read reserved 0x%llx to 0x%llx\n",
		       (unsigned long long)(DFS_dummy_read_center_address - SZ_1M),
		       (unsigned long long)(DFS_dummy_read_center_address + SZ_1M));
		memblock_reserve(DFS_dummy_read_center_address - SZ_1M, (SZ_1M << 1));
		memblock_free(DFS_dummy_read_center_address - SZ_1M, (SZ_1M << 1));
		memblock_remove(DFS_dummy_read_center_address - SZ_1M, (SZ_1M << 1));
#else
		pr_warn("[DFS Check]DFS Dummy read reserved 0x%llx to 0x%llx\n",
		       (unsigned long long)(DFS_dummy_read_center_address - SZ_2M),
		       (unsigned long long)(DFS_dummy_read_center_address + SZ_2M));
		memblock_reserve(DFS_dummy_read_center_address - SZ_2M, (SZ_2M << 1));
		memblock_free(DFS_dummy_read_center_address - SZ_2M, (SZ_2M << 1));
		memblock_remove(DFS_dummy_read_center_address - SZ_2M, (SZ_2M << 1));
#endif
	}

	return;
}

void sync_hw_gating_value(void)
{
	unsigned int reg_val;

	reg_val = (*(volatile unsigned int *)(0xF0004028)) & (~(0x01 << 30));	/*  cha DLLFRZ=0 */
	mt_reg_sync_writel(reg_val, 0xF0004028);
	reg_val = (*(volatile unsigned int *)(0xF0011028)) & (~(0x01 << 30));	/*  chb DLLFRZ=0 */
	mt_reg_sync_writel(reg_val, 0xF0011028);

	mt_reg_sync_writel((*(volatile unsigned int *)(0xF020e374)), 0xF0004094);	/*  cha r0 */
	mt_reg_sync_writel((*(volatile unsigned int *)(0xF020e378)), 0xF0004098);	/*  cha r1 */
	mt_reg_sync_writel((*(volatile unsigned int *)(0xF0213374)), 0xF0011094);	/*  chb r0 */
	mt_reg_sync_writel((*(volatile unsigned int *)(0xF0213378)), 0xF0011098);	/*  chb r1 */

	reg_val = (*(volatile unsigned int *)(0xF0004028)) | (0x01 << 30);	/*  cha DLLFRZ=1 */
	mt_reg_sync_writel(reg_val, 0xF0004028);
	reg_val = (*(volatile unsigned int *)(0xF0011028)) | (0x01 << 30);	/*  chb DLLFRZ=0 */
	mt_reg_sync_writel(reg_val, 0xF0011028);
}
#endif

unsigned int is_one_pll_mode(void)
{
	int data;

	data = *(volatile unsigned int *)(0xF0004000 + (0x00a << 2));
	if (data & 0x10000) {
		/* print("It is 1-PLL mode (value = 0x%x)\n", data); */
		return 1;
	} else {
		/* print("It is 3-PLL mode (value = 0x%x)\n", data); */
		return 0;
	}
}

static ssize_t complex_mem_test_show(struct device_driver *driver, char *buf)
{
	int ret;
	ret = Binning_DRAM_complex_mem_test();
	if (ret > 0)
		return snprintf(buf, PAGE_SIZE, "MEM Test all pass\n");
	else
		return snprintf(buf, PAGE_SIZE, "MEM TEST failed %d\n", ret);
}

static ssize_t complex_mem_test_store(struct device_driver *driver,
				      const char *buf, size_t count)
{
	return count;
}

#ifdef APDMA_TEST
static ssize_t DFS_APDMA_TEST_show(struct device_driver *driver, char *buf)
{
	dma_dummy_read_for_vcorefs(7);
#ifdef DUMP_DDR_RESERVED_MODE
	dump_DMA_Reserved_AREA();
#endif
	return snprintf(buf, PAGE_SIZE, "DFS APDMA Dummy Read Address 0x%x\n", (unsigned int)src_array_p);
}

static ssize_t DFS_APDMA_TEST_store(struct device_driver *driver,
				    const char *buf, size_t count)
{
	return count;
}
#endif

#ifdef READ_DRAM_TEMP_TEST
static ssize_t read_dram_temp_show(struct device_driver *driver, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "DRAM MR4 = 0x%x\n", read_dram_temperature());
}

static ssize_t read_dram_temp_store(struct device_driver *driver,
				    const char *buf, size_t count)
{
	return count;
}
#endif

#ifdef READ_DRAM_FREQ_TEST
static ssize_t read_dram_data_rate_show(struct device_driver *driver, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "DRAM data rate = %d\n", get_dram_data_rate());
}

static ssize_t read_dram_data_rate_store(struct device_driver *driver,
					 const char *buf, size_t count)
{
	return count;
}
#endif

#ifdef DFS_TEST
int dfs_status;
static ssize_t dram_dfs_show(struct device_driver *driver, char *buf)
{
	dfs_status = is_one_pll_mode() ? 0 : 1;
	return snprintf(buf, PAGE_SIZE,
			"Current DRAM DFS status : %s, emi_freq = %d\n",
			dfs_status ? "High frequency" : "Low frequency",
			mt_get_emi_freq());
}

static ssize_t dram_dfs_store(struct device_driver *driver, const char *buf,
			      size_t count)
{
	char *p = (char *)buf;
	unsigned int num;

	num = simple_strtoul(p, &p, 10);
	dfs_status = is_one_pll_mode() ? 0 : 1;
	if (num == dfs_status) {
		if (num == 1)
			pr_warn("[DRAM DFS] Current DRAM frequency is already high freqency\n");
		else
			pr_warn("[DRAM DFS] Current DRAM frequency is already low freqency\n");

		return count;
	}

	switch (num) {
	case 0:
		do_DRAM_DFS(0);
		break;
	case 1:
		do_DRAM_DFS(1);
		break;
	default:
		break;
	}

	return count;
}
#endif

DRIVER_ATTR(emi_clk_mem_test, 0664, complex_mem_test_show,
	    complex_mem_test_store);

#ifdef APDMA_TEST
DRIVER_ATTR(dram_dummy_read_test, 0664, DFS_APDMA_TEST_show,
	    DFS_APDMA_TEST_store);
#endif

#ifdef READ_DRAM_TEMP_TEST
DRIVER_ATTR(read_dram_temp_test, 0664, read_dram_temp_show,
	    read_dram_temp_store);
#endif

#ifdef READ_DRAM_FREQ_TEST
DRIVER_ATTR(read_dram_data_rate, 0664, read_dram_data_rate_show,
	    read_dram_data_rate_store);
#endif

#ifdef FREQ_HOPPING_TEST
DRIVER_ATTR(freq_hopping_test, 0664, freq_hopping_test_show,
	    freq_hopping_test_store);
#endif

#ifdef DFS_TEST
DRIVER_ATTR(dram_dfs, 0664, dram_dfs_show, dram_dfs_store);
#endif

static struct device_driver dram_test_drv = {
	.name = "emi_clk_test",
	.bus = &platform_bus_type,
	.owner = THIS_MODULE,
};

static int dram_dt_init(void)
{
	int ret = 0;
	struct device_node *node = NULL;

	/* DTS version */
	node = of_find_compatible_node(NULL, NULL, "mediatek,APMIXED");
	if (node) {
		APMIXED_BASE_ADDR = of_iomap(node, 0);
		pr_warn("get APMIXED_BASE_ADDR @ %p\n", APMIXED_BASE_ADDR);
	} else {
		pr_warn("can't find compatible node\n");
		return -1;
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek,CQDMA");
	if (node) {
		CQDMA_BASE_ADDR = of_iomap(node, 0);
		pr_warn("[DRAMC]get CQDMA_BASE_ADDR @ %p\n", CQDMA_BASE_ADDR);
	} else {
		pr_warn("[DRAMC]can't find compatible node\n");
		return -1;
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek,DRAMC0");
	if (node) {
		DRAMCAO_BASE_ADDR = of_iomap(node, 0);
		pr_warn("[DRAMC]get DRAMCAO_BASE_ADDR @ %p\n", DRAMCAO_BASE_ADDR);
	} else {
		pr_warn("[DRAMC]can't find DRAMC0 compatible node\n");
		return -1;
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek,DDRPHY");
	if (node) {
		DDRPHY_BASE_ADDR = of_iomap(node, 0);
		pr_warn("[DRAMC]get DDRPHY_BASE_ADDR @ %p\n", DDRPHY_BASE_ADDR);
	} else {
		pr_warn("[DRAMC]can't find DDRPHY compatible node\n");
		return -1;
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek,DRAMC_NAO");
	if (node) {
		DRAMCNAO_BASE_ADDR = of_iomap(node, 0);
		pr_warn("[DRAMC]get DRAMCNAO_BASE_ADDR @ %p\n", DRAMCNAO_BASE_ADDR);
	} else {
		pr_warn("[DRAMC]can't find DRAMCNAO compatible node\n");
		return -1;
	}

	return ret;
}

/* extern char __ssram_text, _sram_start, __esram_text; */
int __init dram_test_init(void)
{
	int ret = 0;

	/* unsigned char * dst = &__ssram_text; */
	/* unsigned char * src = &_sram_start; */

	ret = dram_dt_init();
	if (ret) {
		pr_warn("[DRAMC] Device Tree Init Fail\n");
		return ret;
	}
	/* printk(KERN_ERR "sram start = 0x%x, sram end = 0x%x,
		src=0x%x, dst=0x%x\n",&__ssram_text,&__esram_text, src, dst); */
	ret = driver_register(&dram_test_drv);
	if (ret) {
		pr_err("fail to create the dram_test driver\n");
		return ret;
	}

	ret = driver_create_file(&dram_test_drv, &driver_attr_emi_clk_mem_test);
	if (ret) {
		pr_err("fail to create the emi_clk_mem_test sysfs files\n");
		return ret;
	}
#ifdef APDMA_TEST
	ret = driver_create_file(&dram_test_drv, &driver_attr_dram_dummy_read_test);
	if (ret) {
		pr_err("fail to create the DFS sysfs files\n");
		return ret;
	}
#endif

#ifdef READ_DRAM_TEMP_TEST
	ret = driver_create_file(&dram_test_drv, &driver_attr_read_dram_temp_test);
	if (ret) {
		pr_err("fail to create the read dram temp sysfs files\n");
		return ret;
	}
#endif

#ifdef READ_DRAM_FREQ_TEST
	ret = driver_create_file(&dram_test_drv, &driver_attr_read_dram_data_rate);
	if (ret) {
		pr_err("fail to create the read dram data rate sysfs files\n");
		return ret;
	}
#endif

#ifdef FREQ_HOPPING_TEST
	ret = driver_create_file(&dram_test_drv, &driver_attr_freq_hopping_test);
	if (ret) {
		pr_err("fail to create the read dram temp sysfs files\n");
		return ret;
	}
#endif

#ifdef DFS_TEST
	ret = driver_create_file(&dram_test_drv, &driver_attr_dram_dfs);
	if (ret) {
		pr_err("fail to create the dram dfs sysfs files\n");
		return ret;
	}
#endif
	org_dram_data_rate = get_dram_data_rate();
	pr_warn("[DRAMC Driver] Dram Data Rate = %d\n", org_dram_data_rate);

	if (dram_can_support_fh())
		pr_warn(KERN_INFO "[DRAMC Driver] dram can support Frequency Hopping\n");
	else
		pr_warn(KERN_INFO "[DRAMC Driver] dram can not support Frequency Hopping\n");

	return ret;
}

arch_initcall(dram_test_init);
