#define pr_fmt(fmt)	"[VcoreFS] " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/sched/rt.h>

#include <mach/mt_vcore_dvfs.h>
#include <mach/mt_pmic_wrap.h>
#include <mach/mt_cpufreq.h>
#include <mach/mt_dramc.h>
#include <mach/mt_spm.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_freqhopping.h>
#include <mach/board.h>
#include <mt_sd_func.h>
#include <primary_display.h>

/**************************************
 * Macro and Inline
 **************************************/
#define vcorefs_emerg(fmt, args...)	pr_emerg(fmt, ##args)
#define vcorefs_alert(fmt, args...)	pr_alert(fmt, ##args)
#define vcorefs_crit(fmt, args...)	pr_crit(fmt, ##args)
#define vcorefs_err(fmt, args...)	pr_err(fmt, ##args)
#define vcorefs_warn(fmt, args...)	pr_warn(fmt, ##args)
#define vcorefs_notice(fmt, args...)	pr_notice(fmt, ##args)
#define vcorefs_info(fmt, args...)	pr_info(fmt, ##args)
#define vcorefs_debug(fmt, args...)	pr_info(fmt, ##args)	/* pr_debug show nothing */

#define DEFINE_ATTR_RO(_name)			\
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = #_name,			\
		.mode = 0444,			\
	},					\
	.show	= _name##_show,			\
}

#define DEFINE_ATTR_RW(_name)			\
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = #_name,			\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define __ATTR_OF(_name)	(&_name##_attr.attr)

/**************************************
 * Define and Declare
 **************************************/
struct vcorefs_profile {
	bool recover_en;
	bool vcore_dvs;
	bool freq_dfs;
	bool ddr_dfs;
	bool sdio_trans_pause;
	bool sdio_lock;
	int  late_init_opp_done;

	int error_code;

        unsigned int curr_ddr_khz;
        unsigned int curr_venc_khz;
	unsigned int curr_axi_khz;
        unsigned int curr_qtrhalf_khz;
};

struct opp_profile {
	unsigned int vcore_uv;
	unsigned int ddr_khz;
	unsigned int venc_khz;
	unsigned int axi_khz;
	unsigned int qtrhalf_khz;
};

struct kicker_profile {
	int opp;
};

/**************************************
 * 
 **************************************/
static DEFINE_MUTEX(vcorefs_mutex);
static DEFINE_MUTEX(fuct_check_mutex);

/**************************************
 * __nosavedata will not be restored after IPO-H boot
 **************************************/
static unsigned int trans[2] __nosavedata;
static int vcore_dvfs_opp __nosavedata = OPP_1;
static bool feature_en __nosavedata = 0;	/* if feature disable, then keep HPM */

static struct vcorefs_profile vcorefs_ctrl = {
	/* for framework */
	.recover_en		= 1,
	.vcore_dvs		= 1,
	.freq_dfs		= 1,
	.ddr_dfs		= 1,
	.sdio_trans_pause	= 0,
	.sdio_lock		= 0,
	.late_init_opp_done	= 0,

	.error_code		= PASS,

	/* for Freq control */
	.curr_ddr_khz		= FDDR_S0_KHZ,
	.curr_venc_khz		= FVENC_S0_KHZ,
	.curr_axi_khz           = FAXI_S0_KHZ,
	.curr_qtrhalf_khz	= FQTRHALF_S0_KHZ,
};

static struct opp_profile opp_table[] __nosavedata = {
	[OPP_0] = {					/* HPM */
		.vcore_uv	= VCORE_1_P_15_UV,
		.ddr_khz	= FDDR_S0_KHZ,
		.venc_khz	= FVENC_S0_KHZ,
		.axi_khz        = FAXI_S0_KHZ,
		.qtrhalf_khz	= FQTRHALF_S0_KHZ,
	},
	[OPP_1] = {					/* LPM */
		.vcore_uv	= VCORE_1_P_05_UV,
		.ddr_khz	= FDDR_S1_KHZ,
		.venc_khz	= FVENC_S1_KHZ,
		.axi_khz        = FAXI_S1_KHZ,
		.qtrhalf_khz	= FQTRHALF_S1_KHZ,
	}
};

static struct kicker_profile kicker_table[] = {
        [KIR_GPU] = {
                .opp    = OPP_OFF,
        },
        [KIR_MM] = {
                .opp    = OPP_OFF,
        },
        [KIR_EMIBW] = {
                .opp    = OPP_OFF,
        },
        [KIR_SDIO] = {
                .opp    = OPP_OFF,
        },
        [KIR_USB] = {
                .opp    = OPP_OFF,
        },
        [KIR_SYSFS] = {
                .opp    = OPP_OFF,
        }
};

/**************************************
 * Vcore Control Function
 **************************************/
static unsigned int get_vcore_pdn(void)
{
        unsigned int vcore_pdn, uv;

        pwrap_read(PMIC_VCORE_PDN_VOSEL_ON, &vcore_pdn);
        uv = vcore_pmic_to_uv(vcore_pdn);

	#if 0
        /* out of safe range */
        if (uv > VCORE_1_P_15_UV || uv < VCORE_1_P_05_UV)
                uv = VCORE_1_P_15_UV;
	#endif

        return uv;
}

static void __update_vcore_pdn(struct opp_profile *opp_ctrl_table)
{
        if (opp_ctrl_table[OPP_0].vcore_uv == VCORE_1_P_05_UV) {
                /* for boot up init */
                trans[TRANS2] = VCORE_1_P_05_UV;
                trans[TRANS1] = VCORE_1_P_05_UV;
        } else if (opp_ctrl_table[OPP_0].vcore_uv == opp_ctrl_table[OPP_1].vcore_uv) {
                /* for sysfs opp_table*/
                trans[TRANS2] = opp_ctrl_table[OPP_1].vcore_uv;
                trans[TRANS1] = opp_ctrl_table[OPP_1].vcore_uv;
        } else {
                trans[TRANS2] = opp_ctrl_table[OPP_1].vcore_uv + ((opp_ctrl_table[OPP_0].vcore_uv - opp_ctrl_table[OPP_1].vcore_uv)/3) * 2;
                trans[TRANS1] = opp_ctrl_table[OPP_1].vcore_uv + ((opp_ctrl_table[OPP_0].vcore_uv - opp_ctrl_table[OPP_1].vcore_uv)/3) * 1;
        }

        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_NORMAL, IDX_NM_VCORE_HPM,    vcore_uv_to_pmic(opp_ctrl_table[OPP_0].vcore_uv));
        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_NORMAL, IDX_NM_VCORE_TRANS2, vcore_uv_to_pmic(trans[TRANS2]));
        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_NORMAL, IDX_NM_VCORE_TRANS1, vcore_uv_to_pmic(trans[TRANS1]));
        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_NORMAL, IDX_NM_VCORE_LPM,    vcore_uv_to_pmic(opp_ctrl_table[OPP_1].vcore_uv));

        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_SUSPEND, IDX_SP_VCORE_HPM,    vcore_uv_to_pmic(opp_ctrl_table[OPP_0].vcore_uv));
        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_SUSPEND, IDX_SP_VCORE_TRANS2, vcore_uv_to_pmic(trans[TRANS2]));
        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_SUSPEND, IDX_SP_VCORE_TRANS1, vcore_uv_to_pmic(trans[TRANS1]));
        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_SUSPEND, IDX_SP_VCORE_LPM,    vcore_uv_to_pmic(opp_ctrl_table[OPP_1].vcore_uv));

        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_DEEPIDLE, IDX_DI_VCORE_HPM,    vcore_uv_to_pmic(opp_ctrl_table[OPP_0].vcore_uv));
        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_DEEPIDLE, IDX_DI_VCORE_TRANS2, vcore_uv_to_pmic(trans[TRANS2]));
        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_DEEPIDLE, IDX_DI_VCORE_TRANS1, vcore_uv_to_pmic(trans[TRANS1]));
        mt_cpufreq_set_pmic_cmd(PMIC_WRAP_PHASE_DEEPIDLE, IDX_DI_VCORE_LPM,    vcore_uv_to_pmic(opp_ctrl_table[OPP_1].vcore_uv));
}

/**************************************
 * External Control Function
 **************************************/
unsigned int vcorefs_get_curr_voltage(void)
{
	unsigned int uv;

	uv = get_vcore_pdn();
	vcorefs_crit("VCORE_PDN: %u (0x%x)\n", uv, vcore_uv_to_pmic(uv));
	spm_vcorefs_dump_regs(); /* R13, SRC_REG, PWRON1 */

	return uv;
}

unsigned int get_ddr_khz(void)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;
	int ddr_mhz;

	ddr_mhz = get_dram_data_rate();

	/* return -1 when dram not meet spec */
	if (ddr_mhz < 0)
		return pwrctrl->curr_ddr_khz;

	return ddr_mhz * 1000;
}

int is_vcorefs_can_work(void)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;

	if (pwrctrl->late_init_opp_done)
		return feature_en;
	else
		return ERR_LATE_INIT_OPP;
}

/**************************************
 * Vcore DVS/DFS Function
 **************************************/
static int set_vcore_with_opp(int opp)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;

	#ifdef SDIO_MULTI_AUTOK
	int sdio_ret = -1;
	#endif
	int r = FAIL;

	vcorefs_crit("[%s] opp: %d, sdio_trans_pause: %d %s\n", __func__,
				opp, pwrctrl->sdio_trans_pause, pwrctrl->vcore_dvs ? "" : "[X]");

	if (!pwrctrl->vcore_dvs)
		return PASS;

	if (pwrctrl->sdio_trans_pause) {
		#ifdef SDIO_MULTI_AUTOK
		/* SDIO need multi vcore voltage */
		sdio_ret = sdio_stop_transfer();
		r = spm_set_vcore_dvs_voltage(opp);
		if (!sdio_ret)
			sdio_start_ot_transfer();
		#endif
	}
	else {
		/* SDIO only need HPM */
		r = spm_set_vcore_dvs_voltage(opp);
	}

	return r;
}

static int set_freq_with_opp(int opp)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;
	struct opp_profile *opp_ctrl_table = (struct opp_profile *)&opp_table;
	int r = FAIL;

	vcorefs_crit("[%s] opp: %d, faxi: %u(%u), vencpll: %u(%u) %s\n", __func__,
				opp,
				opp_ctrl_table[opp].axi_khz, pwrctrl->curr_axi_khz,
				opp_ctrl_table[opp].venc_khz, pwrctrl->curr_venc_khz,
				pwrctrl->freq_dfs ? "" : "[X]");

	if (!pwrctrl->freq_dfs)
		return PASS;
	if (opp_ctrl_table[opp].axi_khz == pwrctrl->curr_axi_khz && opp_ctrl_table[opp].venc_khz == pwrctrl->curr_venc_khz)
		return PASS;

	if (opp == OPP_0) {
		r = mt_dfs_vencpll(0x1713B1);		// 300MHz
		clkmux_sel(MT_MUX_AXI, 2, "vcorefs");	// 218MHz - syspll_D5
	} else {
		r = mt_dfs_vencpll(0xCC4EC);		// 166MHz
		clkmux_sel(MT_MUX_AXI, 3, "vcorefs");	// 136MHz - syspll_D4
	}

	if (r)
		vcorefs_crit("[%s] FAILED: VENCPLL COULD NOT BE FREQ HOPPING\n", __func__);
	else
		pwrctrl->curr_venc_khz = opp_ctrl_table[opp].venc_khz;

	pwrctrl->curr_axi_khz		= opp_ctrl_table[opp].axi_khz;
        pwrctrl->curr_qtrhalf_khz	= opp_ctrl_table[opp].qtrhalf_khz;

	return r;
}

static int set_fddr_with_opp(int opp)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;
	struct opp_profile *opp_ctrl_table = (struct opp_profile *)&opp_table;
	int r = FAIL;

	pwrctrl->curr_ddr_khz = get_ddr_khz();

	vcorefs_crit("[%s] opp: %d, ddr: %u(%u) %s\n", __func__,
				opp, opp_ctrl_table[opp].ddr_khz, pwrctrl->curr_ddr_khz, pwrctrl->ddr_dfs ? "" : "[X]");

	if (!pwrctrl->ddr_dfs)
		return PASS;
	if (opp_ctrl_table[opp].ddr_khz == pwrctrl->curr_ddr_khz)
		return PASS;

	/* success when reture 0 */
	r = dram_do_dfs_by_fh(opp_ctrl_table[opp].ddr_khz);

	if (r)
		vcorefs_crit("[%s] FAILED: DRAM COULD NOT BE FREQ HOPPING\n", __func__);
	else
		pwrctrl->curr_ddr_khz = opp_ctrl_table[opp].ddr_khz;

	return r;
}

/**************************************
 *
 **************************************/
static int find_min_opp(void)
{
	struct kicker_profile *kicker_ctrl_table = (struct kicker_profile *)&kicker_table;
	int min, i;

        vcorefs_crit("[%s] [%d, %d, %d, %d, %d, %d]\n", __func__,
                                kicker_ctrl_table[KIR_GPU].opp,
                                kicker_ctrl_table[KIR_MM].opp,
                                kicker_ctrl_table[KIR_EMIBW].opp,
                                kicker_ctrl_table[KIR_SDIO].opp,
                                kicker_ctrl_table[KIR_USB].opp,
                                kicker_ctrl_table[KIR_SYSFS].opp);

	/* find the min opp from kicker table */
	min = kicker_ctrl_table[0].opp;
	for (i=1; i<NUM_KICKER; i++) {
		if (kicker_ctrl_table[i].opp < min)
			min = kicker_ctrl_table[i].opp;
	}

	/* if all no have request, so set to LPM */
	if (min == OPP_OFF)
		min = OPP_1;

	return min;
}

static int vcorefs_find_opp_index_by_request(enum dvfs_kicker kicker, enum dvfs_opp new_opp)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;
	struct kicker_profile *kicker_ctrl_table = (struct kicker_profile *)&kicker_table;

	int old_opp;

	/* compare kicker table opp with new opp */
	old_opp = kicker_ctrl_table[kicker].opp;

	if (old_opp == new_opp) {
		if (pwrctrl->error_code == PASS) {
			pwrctrl->error_code = ERR_NO_CHANGE;
			return vcore_dvfs_opp;
		}
		if (pwrctrl->error_code == ERR_NO_CHANGE) {
			return vcore_dvfs_opp;
		}
	}

	pwrctrl->error_code = PASS;
	kicker_ctrl_table[kicker].opp = new_opp;

	return find_min_opp();
}

static int is_need_recover_dvfs(enum dvfs_kicker kicker, int opp)
{
        struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;
        int recover_opp;

        /* recover check table */
        if (pwrctrl->recover_en && feature_en && kicker<NUM_KICKER) {
		mutex_lock(&fuct_check_mutex);
		recover_opp = find_min_opp();
		mutex_unlock(&fuct_check_mutex);

		if (opp == OPP_0 && recover_opp == OPP_1) {
			vcorefs_crit("[%s] Restore to LPM, restore DVS\n", __func__);
			vcore_dvfs_opp = OPP_1;
			set_vcore_with_opp(vcore_dvfs_opp);
			return PASS;
		}
		else if (opp == OPP_1 && recover_opp == OPP_0) {
			vcorefs_crit("[%s] Restore to HPM, restore DFS\n", __func__);
			vcore_dvfs_opp = OPP_0;
			set_freq_with_opp(vcore_dvfs_opp);
			set_fddr_with_opp(vcore_dvfs_opp);
			return PASS;
		}
	}

        return FAIL;
}

/**************************************
 *
 **************************************/
static int do_dvfs_for_performance(enum dvfs_kicker kicker, int opp)
{
        int r = FAIL;
        vcorefs_crit("[%s] kicker: %d, opp: %d\n", __func__, kicker, opp);

        r = set_vcore_with_opp(opp);
        if (r)
		return ERR_VCORE_DVS;

	/* if new request is LPM, then recover vcore voltage */
	r = is_need_recover_dvfs(kicker, opp);
	if (!r)
		return PASS;

	/* SDIO/USB only need vcore voltage */
	if (kicker == KIR_SDIO || kicker == KIR_USB)
		return PASS;

        r = set_freq_with_opp(opp);
	if (r)
		return ERR_VENCPLL_FH;

        r = set_fddr_with_opp(opp);
        if (r)
                return ERR_DDR_DFS;

        return r;
}

static int do_dvfs_for_low_power(enum dvfs_kicker kicker, int opp)
{
        int r = FAIL;
        vcorefs_crit("[%s] kicker: %d, opp: %d\n", __func__, kicker, opp);

        r = set_fddr_with_opp(opp);
        if (r)
                return ERR_DDR_DFS;

        r = set_freq_with_opp(opp);
	if (r)
		return ERR_VENCPLL_FH;

	/* if new request is HPM, then recover DDR/Freq */
	r = is_need_recover_dvfs(kicker, opp);
	if (!r)
		return PASS;

        r = set_vcore_with_opp(opp);
        if (r)
                return ERR_VCORE_DVS;

        return r;
}

/**************************************
 *
 **************************************/
static int kick_dvfs_by_opp_index(enum dvfs_kicker kicker, int opp)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;
        int r = FAIL;

	if (kicker != KIR_SDIO_AUTOK) {
		if (pwrctrl->sdio_lock)
			return ERR_SDIO_AUTOK;
	}

	if (opp < vcore_dvfs_opp || opp == OPP_0)
		r = do_dvfs_for_performance(kicker, opp);
	else
		r = do_dvfs_for_low_power(kicker, opp);

	vcore_dvfs_opp = opp;

        return r;
}

static int vcorefs_func_enable_check(enum dvfs_kicker kicker, enum dvfs_opp new_opp)
{
        struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;

	vcorefs_crit("[%s] feature_en: %d(%d), recover_en: %d, vcore_dvfs_opp: %d, kicker: %d, new_opp: %d\n", __func__,
					feature_en, pwrctrl->error_code, pwrctrl->recover_en, vcore_dvfs_opp, kicker, new_opp);

	if (!feature_en)
		return ERR_FEATURE_DISABLE;
	if (!pwrctrl->late_init_opp_done)
		return ERR_LATE_INIT_OPP;
	if (pwrctrl->sdio_lock)
		return ERR_SDIO_AUTOK;
	if (kicker>=NUM_KICKER || kicker<KIR_GPU)
		return ERR_KICKER;
	if (new_opp>=NUM_OPP || new_opp<OPP_0)
		return ERR_OPP;

	return PASS;
}

/***********************************************************
 * kicker Idx   Condition       HPM             LPM
 * ---------------------------------------------------------
 * [0][GPU]     GPU loading     >280MHz         <=280MHz
 * [1][MM]      Video format    >720p           <=720p
 * [1][MM]      ISP             On              Off
 * [2][EMI]     EMI BW          >threshold      <=threshold
 * [3][SDIO]    SDIO            on              off
 * [4][USB]     USB             on              off
 * [5][SYSFS]   Command
 ***********************************************************/
int vcorefs_request_dvfs_opp(enum dvfs_kicker kicker, enum dvfs_opp new_opp)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;
	int request_opp;
	int enable_check;

	///////////////////////////////////////
	mutex_lock(&fuct_check_mutex);
	enable_check = vcorefs_func_enable_check(kicker, new_opp);
	if (enable_check) {
		vcorefs_crit("[%s] *** VCORE DVFS STOP (%d)***\n", __func__, enable_check);
		mutex_unlock(&fuct_check_mutex);
		return enable_check;
	}

	request_opp = vcorefs_find_opp_index_by_request(kicker, new_opp);
	if (pwrctrl->error_code) {
		vcorefs_crit("[%s] *** VCORE DVFS OPP NO CHANGE (%d)***\n", __func__, pwrctrl->error_code);
		mutex_unlock(&fuct_check_mutex);
		return pwrctrl->error_code;
	}
	mutex_unlock(&fuct_check_mutex);

	///////////////////////////////////////
	mutex_lock(&vcorefs_mutex);
	pwrctrl->error_code = kick_dvfs_by_opp_index(kicker, request_opp);
	if (pwrctrl->error_code) {
		vcorefs_crit("[%s] *** VCORE DVFS NOT FULL SUCCESS (%d)***\n", __func__, pwrctrl->error_code);
		mutex_unlock(&vcorefs_mutex);
		return pwrctrl->error_code;
	}
	mutex_unlock(&vcorefs_mutex);

	return pwrctrl->error_code;
}

/**************************************
 * SDIO AutoK related API
 **************************************/
int vcorefs_sdio_lock_dvfs(bool is_online_tuning)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;

	if (is_online_tuning) {	/* avoid OT thread sleeping in vcorefs_mutex */
		vcorefs_crit("[%s] sdio: lock in online-tuning\n", __func__);
		return PASS;
	}

	mutex_lock(&vcorefs_mutex);
	pwrctrl->sdio_lock = 1;
	vcorefs_crit("[%s] sdio: set lock: %d, vcore_dvfs_opp: %d\n", __func__, pwrctrl->sdio_lock, vcore_dvfs_opp);
	mutex_unlock(&vcorefs_mutex);

	return PASS;
}

unsigned int vcorefs_sdio_get_vcore_nml(void)
{
	struct opp_profile *opp_ctrl_table = (struct opp_profile *)&opp_table;
	unsigned int voltage, ret;

	mutex_lock(&vcorefs_mutex);
	voltage = vcorefs_get_curr_voltage();
	vcorefs_crit("[%s] sdio: get vcore dvfs voltage: %u\n", __func__, voltage);
	if (voltage == opp_ctrl_table[OPP_0].vcore_uv)
		ret = VCORE_1_P_15_UV;
	else
		ret = VCORE_1_P_05_UV;
	mutex_unlock(&vcorefs_mutex);

	return ret;
}

int vcorefs_sdio_set_vcore_nml(unsigned int autok_vol_uv)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;

	vcorefs_crit("[%s] sdio: autok_vol_uv: %d, vcore_dvfs_opp: %d, sdio_lock: %d\n", __func__,
				autok_vol_uv, vcore_dvfs_opp, pwrctrl->sdio_lock);

	if (!pwrctrl->sdio_lock)
		return FAIL;

	mutex_lock(&vcorefs_mutex);
	if (autok_vol_uv == VCORE_1_P_15_UV)
		kick_dvfs_by_opp_index(KIR_SDIO_AUTOK, OPP_0);
	else
		kick_dvfs_by_opp_index(KIR_SDIO_AUTOK, OPP_1);
	mutex_unlock(&vcorefs_mutex);

	return PASS;
}

int vcorefs_sdio_unlock_dvfs(bool is_online_tuning)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;

	if (is_online_tuning) {	/* avoid OT thread sleeping in vcorefs_mutex */
		vcorefs_crit("[%s] sdio: unlock in online-tuning\n", __func__);
		return PASS;
	}

	mutex_lock(&vcorefs_mutex);
	kick_dvfs_by_opp_index(KIR_SDIO_AUTOK, vcore_dvfs_opp);

	pwrctrl->sdio_lock = 0;
	vcorefs_crit("[%s] sdio: set unlock: %d, vcore_dvfs_opp: %d\n", __func__,
					pwrctrl->sdio_lock, vcore_dvfs_opp);
	mutex_unlock(&vcorefs_mutex);

	return PASS;
}

bool vcorefs_sdio_need_multi_autok(void)
{
	/* true : do LPM+HPM AutoK */
	/* false: do HPM AutoK     */

	/* SDIO only work on HPM */
	return false;
}

static bool is_fhd_segment(void)
{
        return (DISP_GetScreenWidth() * DISP_GetScreenHeight() > SCREEN_RES_720P);
}

/**************************************
 * init boot-up OPP from late init
 **************************************/
static int set_init_opp_index(struct vcorefs_profile *pwrctrl)
{
	int index = vcore_dvfs_opp;
        if (feature_en) {
                if (!dram_can_support_fh()) {
                        feature_en = 0;
                        index = OPP_0;
                        vcorefs_crit("[%s] DISABLE DVFS DUE TO NOT SUPPORT DRAM FP\n", __func__);
                }
                if (is_fhd_segment()) {
                        feature_en = 0;
                        index = OPP_0;
                        vcorefs_crit("[%s] DISABLE DVFS DUE TO NOT SUPPORT FHD\n", __func__);
                }
        } else {
                index = OPP_0;
        }
	return index;
}

static int late_init_to_lowpwr_opp(void)
{
        struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;

        mutex_lock(&vcorefs_mutex);
        vcore_dvfs_opp = set_init_opp_index(pwrctrl);
	pwrctrl->late_init_opp_done = 1;

	vcorefs_crit("[%s] feature_en: %d, vcore_dvfs_opp: %d\n", __func__,
			feature_en, vcore_dvfs_opp);

        kick_dvfs_by_opp_index(KIR_LATE_INIT, vcore_dvfs_opp);
        mutex_unlock(&vcorefs_mutex);

        return PASS;
}

/**************************************
 * 
 **************************************/
static ssize_t opp_table_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	struct opp_profile *opp_ctrl_table = (struct opp_profile *)&opp_table;
	char *p = buf;

	p += sprintf(p, "[OPP_0] ddr_khz:     %d\n",   opp_ctrl_table[OPP_0].ddr_khz);
	p += sprintf(p, "[OPP_0] venc_khz:    %d\n",   opp_ctrl_table[OPP_0].venc_khz);
	p += sprintf(p, "[OPP_0] axi_khz:     %d\n",   opp_ctrl_table[OPP_0].axi_khz);
	p += sprintf(p, "[OPP_0] qtrhalf_khz: %d\n\n", opp_ctrl_table[OPP_0].qtrhalf_khz);

	p += sprintf(p, "[OPP_1] ddr_khz:     %d\n",   opp_ctrl_table[OPP_1].ddr_khz);
	p += sprintf(p, "[OPP_1] venc_khz:    %d\n",   opp_ctrl_table[OPP_1].venc_khz);
	p += sprintf(p, "[OPP_1] axi_khz:     %d\n",   opp_ctrl_table[OPP_1].axi_khz);
	p += sprintf(p, "[OPP_1] qtrhalf_khz: %d\n\n", opp_ctrl_table[OPP_1].qtrhalf_khz);

	p += sprintf(p, "[HPM   ] %u (0x%x)\n", opp_ctrl_table[OPP_0].vcore_uv, vcore_uv_to_pmic(opp_ctrl_table[OPP_0].vcore_uv));
	p += sprintf(p, "[TRANS2] %u (0x%x)\n", trans[TRANS2], vcore_uv_to_pmic(trans[TRANS2]));
	p += sprintf(p, "[TRANS1] %u (0x%x)\n", trans[TRANS1], vcore_uv_to_pmic(trans[TRANS1]));
	p += sprintf(p, "[LPM   ] %u (0x%x)\n", opp_ctrl_table[OPP_1].vcore_uv, vcore_uv_to_pmic(opp_ctrl_table[OPP_1].vcore_uv));

	return p - buf;
}

static ssize_t opp_table_store(struct kobject *kobj, struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	struct opp_profile *opp_ctrl_table = (struct opp_profile *)&opp_table;
	u32 val, uv;
	char cmd[32];

	if (sscanf(buf, "%31s %u", cmd, &val) != 2)
		return -EPERM;

	uv = vcore_pmic_to_uv(val);

	vcorefs_crit("[%s] opp_table: cmd: %s, val: %u, uv: %u\n", __func__, cmd, val, uv);

	if (!strcmp(cmd, "HPM")) {
		if (uv >= opp_ctrl_table[OPP_1].vcore_uv) {
			opp_ctrl_table[OPP_0].vcore_uv = uv;
			__update_vcore_pdn(opp_ctrl_table);
		}
	}
	else if (!strcmp(cmd, "LPM")) {
		if (uv <= opp_ctrl_table[OPP_0].vcore_uv) {
			opp_ctrl_table[OPP_1].vcore_uv = uv;
			__update_vcore_pdn(opp_ctrl_table);
		}
	}
	else {
		return -EINVAL;
	}

	return count;
}

static ssize_t error_table_show(struct kobject *kobj, struct kobj_attribute *attr,
                                char *buf)
{
	char *p = buf;

	p += sprintf(p, "%d: PASS\n", PASS);
	p += sprintf(p, "%d: FAIL\n", FAIL);
	p += sprintf(p, "%d: ERR_FEATURE_DISABLE\n", ERR_FEATURE_DISABLE);
	p += sprintf(p, "%d: ERR_SDIO_AUTOK\n", ERR_SDIO_AUTOK);
	p += sprintf(p, "%d: ERR_OPP\n", ERR_OPP);
	p += sprintf(p, "%d: ERR_KICKER\n", ERR_KICKER);
	p += sprintf(p, "%d: ERR_NO_CHANGE\n", ERR_NO_CHANGE);
	p += sprintf(p, "%d: ERR_VCORE_DVS\n", ERR_VCORE_DVS);
	p += sprintf(p, "%d: ERR_DDR_DFS\n", ERR_DDR_DFS);
	p += sprintf(p, "%d: ERR_VENCPLL_FH\n", ERR_VENCPLL_FH);
	p += sprintf(p, "%d: ERR_LATE_INIT_OPP\n", ERR_LATE_INIT_OPP);

	return p - buf;
}

static ssize_t error_table_store(struct kobject *kobj, struct kobj_attribute *attr,
                                 const char *buf, size_t count)
{
	return 0;
}

static ssize_t vcore_debug_show(struct kobject *kobj, struct kobj_attribute *attr,
                                char *buf)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;
	struct kicker_profile *kicker_ctrl_table = (struct kicker_profile *)&kicker_table;
	char *p = buf;

	p += sprintf(p, "\nOPP: 0=HPM, 1=LPM, 2=OFF\n\n");

	p += sprintf(p, "[feature_en]: %d[*]\n", feature_en);
	p += sprintf(p, "[recover_en]: %d[*]\n", pwrctrl->recover_en);
	p += sprintf(p, "[vcore_dvs ]: %d[*]\n", pwrctrl->vcore_dvs);
	p += sprintf(p, "[freq_dfs  ]: %d[*]\n", pwrctrl->freq_dfs);
	p += sprintf(p, "[ddr_dfs   ]: %d[*]\n", pwrctrl->ddr_dfs);
	#ifdef SDIO_MULTI_AUTOK
	p += sprintf(p, "[sdio_trans_pause]: %d\n", pwrctrl->sdio_trans_pause);
	#endif
	p += sprintf(p, "[sdio_lock ]: %d\n", pwrctrl->sdio_lock);
	p += sprintf(p, "[ERROR_CODE]: %d\n\n", pwrctrl->error_code);

	p += sprintf(p, "[voltage] uv : %u\n", vcorefs_get_curr_voltage());
	p += sprintf(p, "[ddr    ] khz: %u\n", get_ddr_khz());
	p += sprintf(p, "[venc   ] khz: %u\n", pwrctrl->curr_venc_khz);
	p += sprintf(p, "[axi    ] khz: %u\n", pwrctrl->curr_axi_khz);
	p += sprintf(p, "[qtrhalf] khz: %u\n\n", pwrctrl->curr_qtrhalf_khz);

	p += sprintf(p, "vcore_dvfs_opp: %d\n\n", vcore_dvfs_opp);

	p += sprintf(p, "[KIR_GPU  ] opp: %d[*]\n", kicker_ctrl_table[KIR_GPU].opp);
	p += sprintf(p, "[KIR_MM   ] opp: %d[*]\n", kicker_ctrl_table[KIR_MM].opp);
	p += sprintf(p, "[KIR_EMIBW] opp: %d[*]\n", kicker_ctrl_table[KIR_EMIBW].opp);
	p += sprintf(p, "[KIR_SDIO ] opp: %d[*]\n", kicker_ctrl_table[KIR_SDIO].opp);
	p += sprintf(p, "[KIR_USB  ] opp: %d[*]\n", kicker_ctrl_table[KIR_USB].opp);
	p += sprintf(p, "[KIR_SYSFS] opp: %d[*]\n", kicker_ctrl_table[KIR_SYSFS].opp);
	
	return p - buf;
}

static ssize_t vcore_debug_store(struct kobject *kobj, struct kobj_attribute *attr,
                                 const char *buf, size_t count)
{
	struct vcorefs_profile *pwrctrl = (struct vcorefs_profile *)&vcorefs_ctrl;
	struct kicker_profile *kicker_ctrl_table = (struct kicker_profile *)&kicker_table;
	unsigned int val;
	char cmd[32];

	if (sscanf(buf, "%31s %d", cmd, &val) != 2)
		return -EPERM;
	
	vcorefs_crit("[%s] cmd: %s, val: %d\n", __func__, cmd, val);

	if (!strcmp(cmd, "feature_en")) {
		feature_en = val;

		if (!feature_en || !dram_can_support_fh() || is_fhd_segment()) {
			feature_en = 0;

			/* if feature disable, then set to HPM */
			mutex_lock(&vcorefs_mutex);
			vcore_dvfs_opp = OPP_0;
			kick_dvfs_by_opp_index(KIR_SYSFS, vcore_dvfs_opp);
			mutex_unlock(&vcorefs_mutex);
		} else {
			/* need to find min opp from opp table */
			vcorefs_request_dvfs_opp(KIR_SYSFS, kicker_ctrl_table[KIR_SYSFS].opp);
		}
	}
	else if (!strcmp(cmd, "recover_en")) {
		pwrctrl->recover_en = val;
	}
	else if (!strcmp(cmd, "vcore_dvs")) {
		pwrctrl->vcore_dvs = val;

		mutex_lock(&fuct_check_mutex);
		vcore_dvfs_opp = find_min_opp();
		mutex_unlock(&fuct_check_mutex);

		mutex_lock(&vcorefs_mutex);
		kick_dvfs_by_opp_index(KIR_SYSFS, vcore_dvfs_opp);
		mutex_unlock(&vcorefs_mutex);
	}
	else if (!strcmp(cmd, "freq_dfs")) {
		pwrctrl->freq_dfs = val;

                mutex_lock(&fuct_check_mutex);
                vcore_dvfs_opp = find_min_opp();
                mutex_unlock(&fuct_check_mutex);

                mutex_lock(&vcorefs_mutex);
                kick_dvfs_by_opp_index(KIR_SYSFS, vcore_dvfs_opp);
                mutex_unlock(&vcorefs_mutex);
	}
	else if (!strcmp(cmd, "ddr_dfs")) {
		pwrctrl->ddr_dfs = val;

                mutex_lock(&fuct_check_mutex);
                vcore_dvfs_opp = find_min_opp();
                mutex_unlock(&fuct_check_mutex);

                mutex_lock(&vcorefs_mutex);
                kick_dvfs_by_opp_index(KIR_SYSFS, vcore_dvfs_opp);
                mutex_unlock(&vcorefs_mutex);
	}
	else if (!strcmp(cmd, "KIR_GPU")) {
		if (val<NUM_OPP && val>=OPP_0)
			vcorefs_request_dvfs_opp(KIR_GPU, val);
	}
	else if (!strcmp(cmd, "KIR_MM"))	{
		if (val<NUM_OPP && val>=OPP_0)
			vcorefs_request_dvfs_opp(KIR_MM, val);
	}
	else if (!strcmp(cmd, "KIR_EMIBW")) {
		if (val<NUM_OPP && val>=OPP_0)
			vcorefs_request_dvfs_opp(KIR_EMIBW, val);
	}
	else if (!strcmp(cmd, "KIR_SDIO")) {
		if (val<NUM_OPP && val>=OPP_0)
			vcorefs_request_dvfs_opp(KIR_SDIO, val);
	}
	else if (!strcmp(cmd, "KIR_USB")) {
		if (val<NUM_OPP && val>=OPP_0)
			vcorefs_request_dvfs_opp(KIR_USB, val);
	}
	else if (!strcmp(cmd, "KIR_SYSFS")) {
		if (val<NUM_OPP && val>=OPP_0)
			vcorefs_request_dvfs_opp(KIR_SYSFS, val);
	}
	else if (!strcmp(cmd, "KIR_SYSFSX")) { // force voltage
		if (val == OPP_0 || val == OPP_1) {
			vcore_dvfs_opp = val;
			feature_en = 0;

			mutex_lock(&vcorefs_mutex);
			kick_dvfs_by_opp_index(KIR_SYSFS, vcore_dvfs_opp);
			mutex_unlock(&vcorefs_mutex);
		}
		else if (val == OPP_OFF) {
			mutex_lock(&fuct_check_mutex);
			vcore_dvfs_opp = find_min_opp();
			mutex_unlock(&fuct_check_mutex);

			feature_en = 1;

			mutex_lock(&vcorefs_mutex);
			kick_dvfs_by_opp_index(KIR_SYSFS, vcore_dvfs_opp);
			mutex_unlock(&vcorefs_mutex);
		}
	}
	else if (!strcmp(cmd, "LATE_INIT")) {
		vcorefs_sdio_lock_dvfs(0);
		vcorefs_sdio_get_vcore_nml();
		if (val == 0)
			vcorefs_sdio_set_vcore_nml(VCORE_1_P_15_UV);
		else if (val == 1)
			vcorefs_sdio_set_vcore_nml(VCORE_1_P_05_UV);
		vcorefs_sdio_unlock_dvfs(0);
	}
	else
		return -EINVAL;
	
	return count;
}

DEFINE_ATTR_RW(opp_table);
DEFINE_ATTR_RW(error_table);
DEFINE_ATTR_RW(vcore_debug);

static struct attribute *vcorefs_attrs[] = {
	__ATTR_OF(opp_table),
	__ATTR_OF(error_table),
	__ATTR_OF(vcore_debug),
	NULL,
};

static struct attribute_group vcorefs_attr_group = {
	.name	= "vcorefs",
	.attrs	= vcorefs_attrs,
};

/**************************************
 * Init Function
 **************************************/
static int init_vcorefs_pwrctrl(void)
{
	struct opp_profile *opp_ctrl_table = (struct opp_profile *)&opp_table;

        mutex_lock(&vcorefs_mutex);
	opp_ctrl_table[OPP_0].vcore_uv = get_vcore_pdn(); // voltage binning
	opp_ctrl_table[OPP_1].vcore_uv = VCORE_1_P_05_UV;

	__update_vcore_pdn(opp_ctrl_table);

	vcorefs_crit("[%s] HPM   : %u (0x%x)\n", __func__, opp_ctrl_table[OPP_0].vcore_uv, vcore_uv_to_pmic(opp_ctrl_table[OPP_0].vcore_uv));
	vcorefs_crit("[%s] TRANS2: %u (0x%x)\n", __func__, trans[TRANS2], vcore_uv_to_pmic(trans[TRANS2]));
	vcorefs_crit("[%s] TRANS1: %u (0x%x)\n", __func__, trans[TRANS1], vcore_uv_to_pmic(trans[TRANS1]));
	vcorefs_crit("[%s] LPM   : %u (0x%x)\n", __func__, opp_ctrl_table[OPP_1].vcore_uv, vcore_uv_to_pmic(opp_ctrl_table[OPP_1].vcore_uv));
        mutex_unlock(&vcorefs_mutex);

        return PASS;
}

static int __init vcorefs_module_init(void)
{
	int r = FAIL;

	r = init_vcorefs_pwrctrl();
	if (r) {
		vcorefs_err("[%s] FAILED TO INIT PWRCTRL\n", __func__);
		return r;
	}

	r = sysfs_create_group(power_kobj, &vcorefs_attr_group);
	if (r)
		vcorefs_err("FAILED TO CREATE /sys/power/vcorefs (%d)\n", r);

	return r;
}

module_init(vcorefs_module_init);
late_initcall_sync(late_init_to_lowpwr_opp);

MODULE_DESCRIPTION("Vcore DVFS Driver v0.1");
