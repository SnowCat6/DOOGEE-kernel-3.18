#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>

#include <mach/mt_vcore_dvfs.h>
#include <mach/mt_cpufreq.h>
#include "mt_spm_internal.h"

#define SPM_DVS_US	(160+20)

static const u32 vcore_dvfs_binary[] = {
	0x1840001f, 0x00000001, 0x1990001f, 0x10006b08, 0xe8208000, 0x10006b18,
	0x00000000, 0xe8208000, 0x10006310, 0x0b1600f8, 0x1b00001f, 0x10000001,
	0x1b80001f, 0x90100000, 0x88900001, 0x10006814, 0xd8200142, 0x17c07c1f,
	0x1910001f, 0x100062c4, 0x80841002, 0xd80003c2, 0x17c07c1f, 0xc8a004a2,
	0x17c07c1f, 0x1910001f, 0x100062c4, 0x80809001, 0xc8a00982, 0x17c07c1f,
	0x1ac0001f, 0x55aa55aa, 0x1940001f, 0xaa55aa55, 0x1b80001f, 0x00001000,
	0xf0000000, 0x81429801, 0xd80005a5, 0x17c07c1f, 0x1a00001f, 0x10006604,
	0xe2200006, 0xc0c01160, 0x17c07c1f, 0x1a00001f, 0x100062c4, 0x1890001f,
	0x100062c4, 0xa0940402, 0xe2000002, 0x8085b001, 0x1b00001f, 0x10000001,
	0xc8800982, 0x17c07c1f, 0xf0000000, 0x17c07c1f, 0x1a00001f, 0x100062c4,
	0x1890001f, 0x100062c4, 0x80b40402, 0xe2000002, 0x81429801, 0xd8000905,
	0x17c07c1f, 0x1a00001f, 0x10006604, 0xe2200007, 0xc0c01160, 0x17c07c1f,
	0x1b00001f, 0x00801001, 0xf0000000, 0x17c07c1f, 0x81441801, 0xd8200c05,
	0x17c07c1f, 0x1a00001f, 0x10006604, 0xe2200004, 0xc0c01160, 0x17c07c1f,
	0xc0c01320, 0x17c07c1f, 0xe2200003, 0xc0c01160, 0x17c07c1f, 0xc0c01320,
	0x17c07c1f, 0xe2200002, 0xc0c01160, 0x17c07c1f, 0xc0c01320, 0x17c07c1f,
	0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4, 0xa0908402, 0xe2000002,
	0x80840801, 0xd8000d62, 0x17c07c1f, 0x1b00001f, 0x00801001, 0xf0000000,
	0x17c07c1f, 0x1a00001f, 0x100062c4, 0x1890001f, 0x100062c4, 0x80b08402,
	0xe2000002, 0x81441801, 0xd82010e5, 0x17c07c1f, 0x1a00001f, 0x10006604,
	0xe2200003, 0xc0c01160, 0x17c07c1f, 0xc0c01320, 0x17c07c1f, 0xe2200004,
	0xc0c01160, 0x17c07c1f, 0xc0c01320, 0x17c07c1f, 0xe2200005, 0xc0c01160,
	0x17c07c1f, 0xc0c01320, 0x17c07c1f, 0x1b00001f, 0x00800801, 0xf0000000,
	0x17c07c1f, 0x81481801, 0xd8201245, 0x17c07c1f, 0x1b80001f, 0x20000050,
	0xd80012e5, 0x17c07c1f, 0x18d0001f, 0x10006604, 0x10cf8c1f, 0xd8201243,
	0x17c07c1f, 0xf0000000, 0x17c07c1f, 0x1092041f, 0x81499801, 0xd82014a5,
	0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f, 0x60000000, 0xd80013a2,
	0x00a00402, 0xd8201762, 0x17c07c1f, 0x814a1801, 0xd8201605, 0x17c07c1f,
	0x18d0001f, 0x40000000, 0x18d0001f, 0x80000000, 0xd8001502, 0x00a00402,
	0xd8201762, 0x17c07c1f, 0x814a9801, 0xd8201765, 0x17c07c1f, 0x18d0001f,
	0x40000000, 0x18d0001f, 0x80000000, 0xd8001662, 0x00a00402, 0xd8201762,
	0x17c07c1f, 0xf0000000, 0x17c07c1f
};

static struct pcm_desc vcore_dvfs_pcm = {
	.version	= "pcm_vcore_dvfs_v0.1_20141124",
	.base		= vcore_dvfs_binary,
	.size		= 189,
	.sess		= 1,
	.replace	= 1,
	.vec0		= EVENT_VEC(23, 1, 0, 37),	/* FUNC_MD_VRF18_WAKEUP */
	.vec1		= EVENT_VEC(28, 1, 0, 58),	/* FUNC_MD_VRF18_SLEEP */
	.vec2		= EVENT_VEC(11, 1, 0, 76),	/* FUNC_26M_WAKEUP */
	.vec3		= EVENT_VEC(12, 1, 0, 109),	/* FUNC_26M_SLEEP */
};

static struct pwr_ctrl vcore_dvfs_ctrl = {
	.md1_req_mask		= 0,
	.md2_req_mask		= 0,
	.conn_mask		= 0,
	.pcm_f26m_req		= 1,
};

struct spm_lp_scen __spm_vcore_dvfs = {
	.pcmdesc	= &vcore_dvfs_pcm,
	.pwrctrl	= &vcore_dvfs_ctrl,
};

void spm_vcorefs_dump_regs(void)
{
	spm_crit("[VcoreFS] PCM_REG13_DATA   0x%p = 0x%x\n", SPM_PCM_REG13_DATA, spm_read(SPM_PCM_REG13_DATA));
	spm_crit("[VcoreFS] SPM_PCM_SRC_REQ  0x%p = 0x%x\n", SPM_PCM_SRC_REQ, spm_read(SPM_PCM_SRC_REQ));
	spm_crit("[VcoreFS] POWER_ON_VAL1    0x%p = 0x%x\n", SPM_POWER_ON_VAL1, spm_read(SPM_POWER_ON_VAL1));
}

int spm_get_vcore_dvs_voltage(void)
{
	return spm_read(SPM_MFG_ASYNC_PWR_CON) & PCM_ASYNC_REQ;
}

int spm_set_vcore_dvs_voltage(int opp)
{
	unsigned long flags;
	int i=0;

	spin_lock_irqsave(&__spm_lock, flags);

	/* set control bit */
	switch(opp) {
		case OPP_0:
			spm_write(SPM_PCM_SRC_REQ, spm_read(SPM_PCM_SRC_REQ) | SR_PCM_F26M_REQ);
			break;
		case OPP_1:
			spm_write(SPM_PCM_SRC_REQ, spm_read(SPM_PCM_SRC_REQ) & ~SR_PCM_F26M_REQ);
			/* set control bit to low power mode and return */
			spin_unlock_irqrestore(&__spm_lock, flags);
			return PASS;
			break;
		default:
			spm_crit("[VcoreFS] *** FAILED: OPP INDEX IS INCORRECT ***\n");
			spin_unlock_irqrestore(&__spm_lock, flags);
			return ERR_VCORE_DVS;
			break;
	}

	/* check status bit only for HPM*/
	while (!spm_get_vcore_dvs_voltage()) {
		udelay(1);
		if (i++ >= SPM_DVS_US) {
			spm_vcorefs_dump_regs();
			spin_unlock_irqrestore(&__spm_lock, flags);
			return ERR_VCORE_DVS;
		}
	}

	spin_unlock_irqrestore(&__spm_lock, flags);
	
	return PASS;
}

void spm_go_to_vcore_dvfs(u32 spm_flags, u32 spm_data)
{
	struct pcm_desc *pcmdesc = __spm_vcore_dvfs.pcmdesc;
	struct pwr_ctrl *pwrctrl = __spm_vcore_dvfs.pwrctrl;
	unsigned long flags;

	spin_lock_irqsave(&__spm_lock, flags);

	#if defined(CONFIG_ARCH_MT6735)
	mt_cpufreq_set_pmic_phase(PMIC_WRAP_PHASE_NORMAL);
	#endif

	set_pwrctrl_pcm_flags(pwrctrl, spm_flags);

	__spm_reset_and_init_pcm(pcmdesc);

	__spm_kick_im_to_fetch(pcmdesc);

	__spm_init_pcm_register();

	__spm_init_event_vector(pcmdesc);

	__spm_set_power_control(pwrctrl);

	__spm_set_wakeup_event(pwrctrl);

	__spm_kick_pcm_to_run(pwrctrl);

	spin_unlock_irqrestore(&__spm_lock, flags);
}

MODULE_DESCRIPTION("SPM-VCORE_DVFS Driver v0.1");
