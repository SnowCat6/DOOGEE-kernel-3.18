#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/xlog.h>

#include "mach/mt_reg_base.h"
#include "mach/emi_bwl.h"
#include "mach/sync_write.h"

DEFINE_SEMAPHORE(emi_bwl_sem);

static struct platform_driver mem_bw_ctrl = {
    .driver     = {
        .name = "mem_bw_ctrl",
        .owner = THIS_MODULE,
    },
};

static struct platform_driver ddr_type = {
    .driver     = {
        .name = "ddr_type",
        .owner = THIS_MODULE,
    },
};

/* define EMI bandwiwth limiter control table */
static struct emi_bwl_ctrl ctrl_tbl[NR_CON_SCE];

/* current concurrency scenario */
static int cur_con_sce = 0x0FFFFFFF;

/* define concurrency scenario strings */
static const char *con_sce_str[] =
{
#define X_CON_SCE(con_sce, arba, arbb, arbc, arbd, arbe, arbf, arbg, arbh) #con_sce,
#include "mach/con_sce_lpddr3_1600.h"
#undef X_CON_SCE
};


/****************** For LPDDR3-1600******************/

static const unsigned int emi_arba_lpddr3_1600_val[] =
{
#define X_CON_SCE(con_sce, arba, arbb, arbc, arbd, arbe, arbf, arbg, arbh) arba,
#include "mach/con_sce_lpddr3_1600.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbb_lpddr3_1600_val[] =
{
#define X_CON_SCE(con_sce, arba, arbb, arbc, arbd, arbe, arbf, arbg, arbh) arbb,
#include "mach/con_sce_lpddr3_1600.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbc_lpddr3_1600_val[] =
{
#define X_CON_SCE(con_sce, arba, arbb, arbc, arbd, arbe, arbf, arbg, arbh) arbc,
#include "mach/con_sce_lpddr3_1600.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbd_lpddr3_1600_val[] =
{
#define X_CON_SCE(con_sce, arba, arbb, arbc, arbd, arbe, arbf, arbg, arbh) arbd,
#include "mach/con_sce_lpddr3_1600.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbe_lpddr3_1600_val[] =
{
#define X_CON_SCE(con_sce, arba, arbb, arbc, arbd, arbe, arbf, arbg, arbh) arbe,
#include "mach/con_sce_lpddr3_1600.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbf_lpddr3_1600_val[] =
{
#define X_CON_SCE(con_sce, arba, arbb, arbc, arbd, arbe, arbf, arbg, arbh) arbf,
#include "mach/con_sce_lpddr3_1600.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbg_lpddr3_1600_val[] =
{
#define X_CON_SCE(con_sce, arba, arbb, arbc, arbd, arbe, arbf, arbg, arbh) arbg,
#include "mach/con_sce_lpddr3_1600.h"
#undef X_CON_SCE
};
static const unsigned int emi_arbh_lpddr3_1600_val[] =
{
#define X_CON_SCE(con_sce, arba, arbb, arbc, arbd, arbe, arbf, arbg, arbh) arbh,
#include "mach/con_sce_lpddr3_1600.h"
#undef X_CON_SCE
};

#if 0
int get_ddr_type(void)
{
  unsigned int value;
  
  value = DRAMC_READ(DRAMC_LPDDR2);
  if((value>>28) & 0x1) //check LPDDR2_EN
  {
    return LPDDR2;
  }  
  
  value = DRAMC_READ(DRAMC_ACTIM1);
  if((value>>28) & 0x1) //check LPDDR3_EN
  {
       
	value = readl(IOMEM(EMI_CONA));       
	if (value & 0x01)   //support 2 channel
	{
	    return DUAL_LPDDR3_1600;
	}
	else
	{
	    return LPDDR3_1866;
	}	
  }  
  
  return mDDR;  
}
#endif 

/*
 * mtk_mem_bw_ctrl: set EMI bandwidth limiter for memory bandwidth control
 * @sce: concurrency scenario ID
 * @op: either ENABLE_CON_SCE or DISABLE_CON_SCE
 * Return 0 for success; return negative values for failure.
 */
int mtk_mem_bw_ctrl(int sce, int op)
{
  int i, highest;
  
  if (sce >= NR_CON_SCE) {
    return -1;
  }
  
  if (op != ENABLE_CON_SCE && op != DISABLE_CON_SCE) {
    return -1;
  }
  if (in_interrupt()) {
   return -1;
  }
  
  down(&emi_bwl_sem);
  
  if (op == ENABLE_CON_SCE) {
    ctrl_tbl[sce].ref_cnt++;
  } 
  else if (op == DISABLE_CON_SCE) {
    if (ctrl_tbl[sce].ref_cnt != 0) {
      ctrl_tbl[sce].ref_cnt--;
    } 
  }
  
  /* find the scenario with the highest priority */
  highest = -1;
  for (i = 0; i < NR_CON_SCE; i++) {
      if (ctrl_tbl[i].ref_cnt != 0) {
          highest = i;
          break;
      }
  }
  if (highest == -1) {
      highest = CON_SCE_NORMAL;
  }
  
  /* set new EMI bandwidth limiter value */
  if (highest != cur_con_sce) {
  
    mt_reg_sync_writel(emi_arba_lpddr3_1600_val[highest], EMI_ARBA);
    mt_reg_sync_writel(emi_arbb_lpddr3_1600_val[highest], EMI_ARBB);	   
	mt_reg_sync_writel(emi_arbc_lpddr3_1600_val[highest], EMI_ARBC);
	mt_reg_sync_writel(emi_arbd_lpddr3_1600_val[highest], EMI_ARBD);
	mt_reg_sync_writel(emi_arbe_lpddr3_1600_val[highest], EMI_ARBE);
	mt_reg_sync_writel(emi_arbf_lpddr3_1600_val[highest], EMI_ARBF);
	mt_reg_sync_writel(emi_arbg_lpddr3_1600_val[highest], EMI_ARBG);
	mt_reg_sync_writel(emi_arbh_lpddr3_1600_val[highest], EMI_ARBH);
	

    cur_con_sce = highest;
  }
  
  up(&emi_bwl_sem);

  return 0;  
}

/*
 * ddr_type_show: sysfs ddr_type file show function.
 * @driver:
 * @buf: the string of ddr type
 * Return the number of read bytes.
 */
static ssize_t ddr_type_show(struct device_driver *driver, char *buf)
{
    sprintf(buf, "LPDDR3\n");
    return strlen(buf);
}

/*
 * ddr_type_store: sysfs ddr_type file store function.
 * @driver:
 * @buf:
 * @count:
 * Return the number of write bytes.
 */
static ssize_t ddr_type_store(struct device_driver *driver, const char *buf, size_t count)
{
  /*do nothing*/
  return count;
}

DRIVER_ATTR(ddr_type, 0644, ddr_type_show, ddr_type_store);

/*
 * con_sce_show: sysfs con_sce file show function.
 * @driver:
 * @buf:
 * Return the number of read bytes.
 */
static ssize_t con_sce_show(struct device_driver *driver, char *buf)
{
    if (cur_con_sce >= NR_CON_SCE) {
        sprintf(buf, "none\n");
    } else {
        sprintf(buf, "%s\n", con_sce_str[cur_con_sce]);
    }

    return strlen(buf);
}

/*
 * con_sce_store: sysfs con_sce file store function.
 * @driver:
 * @buf:
 * @count:
 * Return the number of write bytes.
 */
static ssize_t con_sce_store(struct device_driver *driver, const char *buf, size_t count)
{
  int i;

  for (i = 0; i < NR_CON_SCE; i++) {
    if (!strncmp(buf, con_sce_str[i], strlen(con_sce_str[i]))) {
      if (!strncmp(buf + strlen(con_sce_str[i]) + 1, EN_CON_SCE_STR, strlen(EN_CON_SCE_STR))) {
        mtk_mem_bw_ctrl(i, ENABLE_CON_SCE);
        printk("concurrency scenario %s ON\n", con_sce_str[i]);
        break;
      } 
      else if (!strncmp(buf + strlen(con_sce_str[i]) + 1, DIS_CON_SCE_STR, strlen(DIS_CON_SCE_STR))) {
        mtk_mem_bw_ctrl(i, DISABLE_CON_SCE);
        printk("concurrency scenario %s OFF\n", con_sce_str[i]);
        break;
      }
    }
  }

  return count;
}

DRIVER_ATTR(concurrency_scenario, 0644, con_sce_show, con_sce_store);

/*
 * emi_bwl_mod_init: module init function.
 */
static int __init emi_bwl_mod_init(void)
{
  int ret;    
  
  ret = mtk_mem_bw_ctrl(CON_SCE_NORMAL, ENABLE_CON_SCE);
  if (ret) {
    xlog_printk(ANDROID_LOG_ERROR, "EMI/BWL", "fail to set EMI bandwidth limiter\n");
  }

  /* Register BW ctrl interface */
  ret = platform_driver_register(&mem_bw_ctrl);
  if (ret) {
    xlog_printk(ANDROID_LOG_ERROR, "EMI/BWL", "fail to register EMI_BW_LIMITER driver\n");
  }

  ret = driver_create_file(&mem_bw_ctrl.driver, &driver_attr_concurrency_scenario);
  if (ret) {
    xlog_printk(ANDROID_LOG_ERROR, "EMI/BWL", "fail to create EMI_BW_LIMITER sysfs file\n");
  }

  /* Register DRAM type information interface */
  ret = platform_driver_register(&ddr_type);
  if (ret) {
    xlog_printk(ANDROID_LOG_ERROR, "EMI/BWL", "fail to register DRAM_TYPE driver\n");
  }
  
  ret = driver_create_file(&ddr_type.driver, &driver_attr_ddr_type);
  if (ret) {
    xlog_printk(ANDROID_LOG_ERROR, "EMI/BWL", "fail to create DRAM_TYPE sysfs file\n");
  }
  
  return 0;  
}

/*
 * emi_bwl_mod_exit: module exit function.
 */
static void __exit emi_bwl_mod_exit(void)
{
}

//EXPORT_SYMBOL(get_ddr_type);
 
module_init(emi_bwl_mod_init);
module_exit(emi_bwl_mod_exit);

