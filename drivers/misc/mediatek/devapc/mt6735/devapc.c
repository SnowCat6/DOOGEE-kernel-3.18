#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/xlog.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/types.h>
#ifdef CONFIG_MTK_HIBERNATION
#include <mach/mtk_hibernate_dpm.h>
#include <mach/irqs.h>
#endif
#if defined(CONFIG_MTK_LEGACY)
#include <mach/mt_clkmgr.h>
#else
#include <linux/clk.h>
#endif /* defined(CONFIG_MTK_LEGACY) */
#include "mach/mt_reg_base.h"
#include "mach/mt_device_apc.h"
#include "mach/mt_typedefs.h"
#include "mach/sync_write.h"
#include <mach/mt_secure_api.h>
#include "devapc.h"

/* Debug message event */
#define DEVAPC_LOG_NONE        0x00000000
#define DEVAPC_LOG_ERR         0x00000001
#define DEVAPC_LOG_WARN        0x00000002
#define DEVAPC_LOG_INFO        0x00000004
#define DEVAPC_LOG_DBG         0x00000008

#define DEVAPC_LOG_LEVEL      (DEVAPC_LOG_DBG)

#define DEVAPC_MSG(fmt, args...) \
    do {    \
    if (DEVAPC_LOG_LEVEL & DEVAPC_LOG_DBG) { \
        pr_debug(fmt, ##args); \
    } else if (DEVAPC_LOG_LEVEL & DEVAPC_LOG_INFO) { \
        pr_info(fmt, ##args); \
    } else if (DEVAPC_LOG_LEVEL & DEVAPC_LOG_WARN) { \
        pr_warn(fmt, ##args); \
    } else if (DEVAPC_LOG_LEVEL & DEVAPC_LOG_ERR) { \
        pr_err(fmt, ##args); \
    } \
    } while(0)


#if !defined(CONFIG_MTK_LEGACY)
static struct clk *dapc_clk;
#endif /* !defined(CONFIG_MTK_LEGACY) */

static struct cdev *g_devapc_ctrl = NULL;
static unsigned int devapc_irq = 0;
static void __iomem *devapc_ao_base = NULL;
static void __iomem *devapc_pd_base = NULL;

#if defined(CONFIG_ARCH_MT6735)

static DEVICE_INFO devapc_devices[] = {
    /*0*/
    {"INFRA_AO_INFRASYS_CONFIG_REGS",           FALSE},
    {"INFRA_AO_PMIC_WRAP_CONTROL_REG",          FALSE},
    {"INFRA_AO_PERISYS_CONFIG_REGS",            FALSE},
    {"INFRA_AO_KPAD_CONTROL_REG",               TRUE},
    {"INFRA_AO_GPT",                            TRUE},
    {"INFRA_AO_APMCU_EINT_CONTROLLER",          TRUE},
    {"INFRA_AO_TOP_LEVEL_SLP_MANAGER",          FALSE},
    {"INFRA_AO_DEVICE_APC_AO",                  TRUE},
    {"INFRA_AO_SEJ",                            TRUE},
    {"INFRA_AO_RSVD",                           TRUE},
    
     /*10*/
    {"INFRA_AO_CLDMA_AO_TOP_AP",                FALSE},
    {"INFRA_AO_CLDMA_AO_TOP_MD",                FALSE},
    {"INFRASYS_MCUSYS_CONFIG_REG",              TRUE},
    {"INFRASYS_CONTROL_REG",                    TRUE},
    {"INFRASYS_BOOTROM/SRAM",                   TRUE},
    {"INFRASYS_EMI_BUS_INTERFACE",              FALSE},
    {"INFRASYS_SYSTEM_CIRQ",                    TRUE},
    {"INFRASYS_MM_IOMMU_CONFIGURATION",         TRUE},
    {"INFRASYS_EFUSEC",                         FALSE},
    {"INFRASYS_DEVICE_APC_MONITOR",             TRUE},
    
     /*20*/
    {"INFRASYS_MCU_BIU_CONFIGURATION",          TRUE},
    {"INFRASYS_AP_MIXED_CONTROL_REG",           FALSE},
    {"INFRASYS_CA7_AP_CCIF",                    FALSE},
    {"INFRASYS_CA7_MD_CCIF",                    FALSE},
    {"RSVD",                                    TRUE},
    {"INFRASYS_GPIO1_CONTROLLER",               TRUE},
    {"INFRASYS_MBIST_CONTROL_REG",              TRUE},
    {"INFRASYS_TRNG",                           TRUE},
    {"INFRA_AO_TOP_LEVEL_CLOCK_GENERATOR",      FALSE},
    {"INFRASYS_GPIO1_CONTROLLER",               FALSE},
    
     /*30*/
    {"INFRA_AO_TOP_LEVEL_REST_GENERATOR",       TRUE},
    {"INFRASYS_DDRPHY",                         TRUE},  
    {"INFRASYS_DRAM_CONTROLLER",                TRUE},
    {"INFRASYS_MIPI_RX_ANA",                    TRUE},
    {"INFRASYS_GCPU",                           TRUE},
    {"INFRASYS_GCE",                            TRUE},
    {"INFRASYS_CCIF_AP_1",                      FALSE},
    {"INFRASYS_CCIF_MD_1",                      FALSE},
    {"INFRASYS_CLDMA_PDN_AP",                   FALSE},
    {"INFRASYS_CLDMA_PDN_MD",                   FALSE},

     /*40*/
    {"INFRASYS_MD2MD_CCIF0",                    FALSE},
    {"INFRASYS_MD2MD_CCIF1",                    FALSE},
    {"INFRASYS_MDSYSINTF",                      TRUE},
    {"DEGBUGSYS",                               FALSE},
    {"DMA",                                     TRUE},
    {"AUXADC",                                  TRUE},
    {"UART0",                                   TRUE},
    {"UART1",                                   TRUE},
    {"UART2",                                   TRUE},
    {"UART3",                                   TRUE},
    
     /*50*/
    {"PWM",                                     FALSE},
    {"I2C0",                                    TRUE},
    {"I2C1",                                    TRUE},
    {"I2C2",                                    TRUE},
    {"SPI0",                                    TRUE},
    {"PTP_THERMAL_CTL",                         FALSE},
    {"BTIF",                                    TRUE},
    {"UART4",                                   TRUE},
    {"DISP_PWM",                                TRUE},
    {"I2C3",                                    TRUE},
    
     /*60*/
    {"IRDA",                                    TRUE},
    {"IR_TX",                                   TRUE},
    {"USB2.0",                                  FALSE},
    {"USB2.0 SIF",                              FALSE},
    {"AUDIO",                                   FALSE},
    {"MSDC0",                                   TRUE},
    {"MSDC1",                                   TRUE},
    {"MSDC2",                                   TRUE},
    {"USB3.0",                                  TRUE},
    {"WCN_AHB_SLAVE",                           FALSE},
    
     /*70*/
    {"MD2_PERIPHERALS",                         TRUE},
    {"MD3_PERIPHERALS",                         TRUE},
    {"G3D_CONFIG",                              TRUE},
    {"MALI",                                    TRUE},
    {"MMSYS_CONFIG",                            TRUE},
    {"MDP_RDMA",                                TRUE},
    {"MDP_RSZ0",                                TRUE},
    {"MDP_RSZ1",                                TRUE},
    {"MDP_WDMA",                                TRUE},
    {"MDP_WROT",                                TRUE},
    
     /*80*/
    {"MDP_TDSHP",                               TRUE},
    {"DISP_OVL",                                TRUE},
    {"DISP_RDMA0",                              TRUE},
    {"DISP_RDMA1",                              TRUE},
    {"DISP_WDMA",                               TRUE},
    {"DISP_COLOR",                              TRUE},
    {"DISP_CCORR",                              TRUE},
    {"DISP_AAL",                                TRUE},
    {"DISP_GAMMA",                              TRUE},
    {"DISP_DITHER",                             TRUE},
    
     /*90*/
    {"Reserved",                                TRUE},
    {"DSI",                                     TRUE},
    {"DPI",                                     TRUE},
    {"Reserved",                                TRUE},
    {"MM_MUTEX",                                TRUE},
    {"SMI_LARB0",                               TRUE},
    {"SMI_COMMON",                              TRUE},
    {"MIPI_TX_CONFIG",                          TRUE},
    {"IMGSYS_CONFIG",                           TRUE},
    {"IMGSYS_SMI_LARB2",                        TRUE},
    
     /*100*/
    {"IMGSYS_CAM1",                             TRUE},
    {"IMGSYS_CAM2",                             TRUE},
    {"IMGSYS_CAM3",                             TRUE},
    {"IMGSYS_CAM4",                             TRUE},
    {"IMGSYS_SENINF",                           TRUE},
    {"IMGSYS_CAMSV",                            TRUE},
    {"IMGSYS_FDVT",                             TRUE}, 
    {"IMGSYS_CAM5",                             TRUE},
    {"IMGSYS_CAM6",                             TRUE},
    {"IMGSYS_CAM7",                             TRUE},
    
     /*110*/
    {"VDECSYS_GLOBAL_CONFIGURATION",            TRUE},
    {"SMI_LARB1",                               TRUE},
    {"VDEC_FULL_TOP",                           TRUE},
    {"VENC_GLOBAL_CON",                         TRUE},
    {"SMI_LARB3",                               TRUE},
    {"VENC",                                    TRUE},
    {"JPEG_ENC",                                TRUE},
    {"JPEG_DEC",                                TRUE},

};

#elif defined(CONFIG_ARCH_MT6735M)

static DEVICE_INFO devapc_devices[] = { 
    //Slave                                     Is Forbidden?
    {"INFRA_AO_INFRASYS_CONFIG_REGS",           FALSE},
    {"INFRA_AO_PMIC_WRAP_CONTROL_REG",          FALSE},
    {"INFRA_AO_PERISYS_CONFIG_REGS",            FALSE},
    {"INFRA_AO_KPAD_CONTROL_REG",               TRUE},
    {"INFRA_AO_GPT",                            TRUE},
    {"INFRA_AO_APMCU_EINT_CONTROLLER",          TRUE},
    {"INFRA_AO_TOP_LEVEL_SLP_MANAGER",          FALSE},
    {"INFRA_AO_DEVICE_APC_AO",                  TRUE},
    {"INFRA_AO_SEJ",                            TRUE},
    {"INFRA_AO_RSVD",                           TRUE},

     /*10*/
    {"INFRA_AO_CLDMA_AO_TOP_AP",                FALSE},
    {"INFRA_AO_CLDMA_AO_TOP_MD",                FALSE},
    {"Reserved",                                TRUE},
    {"INFRASYS_MCUSYS_CONFIG_REG",              TRUE},
    {"INFRASYS_CONTROL_REG",                    TRUE},
    {"INFRASYS_BOOTROM/SRAM",                   TRUE},
    {"INFRASYS_EMI_BUS_INTERFACE",              FALSE},
    {"INFRASYS_SYSTEM_CIRQ",                    TRUE},
    {"INFRASYS_MM_IOMMU_CONFIGURATION",         TRUE},
    {"INFRASYS_EFUSEC",                         FALSE},
    
     /*20*/
    {"INFRASYS_DEVICE_APC_MONITOR",             TRUE},
    {"INFRASYS_MCU_BIU_CONFIGURATION",          TRUE},
    {"INFRASYS_AP_MIXED_CONTROL_REG",           FALSE},
    {"INFRASYS_CA7_AP_CCIF",                    FALSE},
    {"INFRASYS_CA7_MD_CCIF",                    FALSE},
    {"RSVD",                                    TRUE},
    {"INFRASYS_GPIO1_CONTROLLER",               TRUE},
    {"INFRASYS_MBIST_CONTROL_REG",              TRUE},
    {"INFRASYS_TRNG",                           TRUE},
    {"INFRA_AO_TOP_LEVEL_CLOCK_GENERATOR",      FALSE},
    
     /*30*/
    {"INFRASYS_GPIO1_CONTROLLER",               FALSE},
    {"INFRA_AO_TOP_LEVEL_REST_GENERATOR",       FALSE},
    {"INFRASYS_DDRPHY",                         TRUE},  
    {"INFRASYS_DRAM_CONTROLLER",                TRUE},
    {"INFRASYS_MIPI_RX_ANA",                    TRUE},
    {"INFRASYS_GCPU",                           TRUE},
    {"INFRASYS_GCE",                            TRUE},
    {"INFRASYS_CCIF_AP_1",                      FALSE},
    {"INFRASYS_CCIF_MD_1",                      FALSE},
    {"INFRASYS_CLDMA_PDN_AP",                   FALSE},
    
     /*40*/
    {"INFRASYS_CLDMA_PDN_MD",                   FALSE},
    {"INFRASYS_MDSYSINTF",                      TRUE},
    {"DEGBUGSYS",                               FALSE},
    {"DMA",                                     TRUE},
    {"AUXADC",                                  TRUE},
    {"UART0",                                   TRUE},
    {"UART1",                                   TRUE},
    {"UART2",                                   TRUE},
    {"UART3",                                   TRUE},
    {"PWM",                                     FALSE},

     /*50*/
    {"I2C0",                                    TRUE},
    {"I2C1",                                    TRUE},
    {"I2C2",                                    TRUE},
    {"SPI0",                                    TRUE},
    {"PTP_THERMAL_CTL",                         FALSE},
    {"BTIF",                                    TRUE},
    {"Reserved",                                TRUE},
    {"DISP_PWM",                                TRUE},
    {"I2C3",                                    TRUE},
    {"IRDA",                                    TRUE},

     /*60*/
    {"IR_TX",                                   TRUE},
    {"USB2.0",                                  FALSE},
    {"USB2.0 SIF",                              FALSE},
    {"AUDIO",                                   FALSE},
    {"MSDC0",                                   TRUE},
    {"MSDC1",                                   TRUE},
    {"RESERVE",                                 TRUE},
    {"RESERVE",                                 TRUE},
    {"WCN_AHB_SLAVE",                           FALSE},
    {"MD_PERIPHERALS",                          TRUE},
    
     /*70*/
    {"RESERVE",                                 TRUE},
    {"G3D_CONFIG",                              TRUE},
    {"MALI",                                    TRUE},
    {"MMSYS_CONFIG",                            TRUE},
    {"MDP_RDMA",                                TRUE},
    {"MDP_RSZ0",                                TRUE},
    {"MDP_RSZ1",                                TRUE},
    {"MDP_WDMA",                                TRUE},
    {"MDP_WROT",                                TRUE},
    {"MDP_TDSHP",                               TRUE},

     /*80*/
    {"DISP_OVL0",                               TRUE},
    {"DISP_OVL1",                               TRUE},
    {"DISP_RDMA0",                              TRUE},
    {"DISP_RDMA1",                              TRUE},
    {"DISP_WDMA",                               TRUE},
    {"DISP_COLOR",                              TRUE},
    {"DISP_CCORR",                              TRUE},
    {"DISP_AAL",                                TRUE},
    {"DISP_GAMMA",                              TRUE},
    {"DISP_DITHER",                             TRUE},
    
     /*90*/
    {"Reserved",                                TRUE},
    {"DPI",                                     TRUE},
    {"DSI",                                     TRUE},
    {"Reserved",                                TRUE},
    {"MM_MUTEX",                                TRUE},
    {"SMI_LARB0",                               TRUE},
    {"SMI_COMMON",                              TRUE},
    {"MIPI_TX_CONFIG",                          TRUE},
    {"IMGSYS_CONFIG",                           TRUE},
    {"IMGSYS_SMI_LARB2",                        TRUE},

     /*100*/
    {"IMGSYS_CAM1",                             TRUE},
    {"IMGSYS_CAM2",                             TRUE},
    {"IMGSYS_SENINF",                           TRUE},
    {"VENC",                                    TRUE},
    {"JPGENC",                                  TRUE},
    {"VDEC",                                    TRUE},
    {"VDEC_GLOBAL_CON",                         TRUE},
    {"SMI_LARB1",                               TRUE},
    {"VDEC_FULL_TOP",                           TRUE},

};

#elif defined(CONFIG_ARCH_MT6753)

static DOMAIN_INFO domain_settings[] = {
    {"AP"},
    {"MD1"},
    {"CONN"},
    {"Reserved"},
    {"MM"},
    {"MD3"},
    {"MFG"},
    {"Reserved"},
};

static DEVICE_INFO devapc_devices[] = {
    {"INFRA_AO_INFRASYS_CONFIG_REGS",           FALSE},
    {"INFRA_AO_PMIC_WRAP_CONTROL_REG",          FALSE},
    {"INFRA_AO_PERISYS_CONFIG_REGS",            FALSE},
    {"INFRA_AO_KPAD_CONTROL_REG",               TRUE},
    {"INFRA_AO_GPT",                            TRUE},
    {"INFRA_AO_APMCU_EINT_CONTROLLER",          TRUE},
    {"INFRA_AO_TOP_LEVEL_SLP_MANAGER",          FALSE},
    {"INFRA_AO_DEVICE_APC_AO",                  TRUE},
    {"INFRA_AO_SEJ",                            TRUE},
    {"INFRA_AO_RSVD",                           TRUE},
    
     /*10*/
    {"INFRA_AO_CLDMA_AO_TOP_AP",                FALSE},
    {"INFRA_AO_CLDMA_AO_TOP_MD",                FALSE},
    {"INFRASYS_MCUSYS_CONFIG_REG",              TRUE},
    {"INFRASYS_CONTROL_REG",                    TRUE},
    {"INFRASYS_BOOTROM/SRAM",                   TRUE},
    {"INFRASYS_EMI_BUS_INTERFACE",              FALSE},
    {"INFRASYS_SYSTEM_CIRQ",                    TRUE},
    {"INFRASYS_MM_IOMMU_CONFIGURATION",         TRUE},
    {"INFRASYS_EFUSEC",                         FALSE},
    {"INFRASYS_DEVICE_APC_MONITOR",             TRUE},
    
     /*20*/
    {"INFRASYS_MCU_BIU_CONFIGURATION",          TRUE},
    {"INFRASYS_AP_MIXED_CONTROL_REG",           FALSE},
    {"INFRASYS_CA7_AP_CCIF",                    FALSE},
    {"INFRASYS_CA7_MD_CCIF",                    FALSE},
    {"RSVD",                                    TRUE},
    {"INFRASYS_MBIST_CONTROL_REG",              TRUE},
    {"INFRASYS_DRAM_CONTROLLER",                TRUE},
    {"INFRASYS_TRNG",                           TRUE},
    {"INFRA_AO_TOP_LEVEL_CLOCK_GENERATOR",      FALSE},
    {"INFRASYS_GPIO1_CONTROLLER",               FALSE},
    
     /*30*/
    {"INFRA_AO_TOP_LEVEL_REST_GENERATOR",       TRUE},
    {"INFRASYS_DDRPHY",                         TRUE},  
    {"INFRASYS_DRAM_CONTROLLER",                TRUE},
    {"INFRASYS_MIPI_RX_ANA",                    TRUE},
    {"INFRASYS_GCPU",                           TRUE},
    {"INFRASYS_GCE",                            TRUE},
    {"INFRASYS_CCIF_AP_1",                      FALSE},
    {"INFRASYS_CCIF_MD_1",                      FALSE},
    {"INFRASYS_CLDMA_PDN_AP",                   FALSE},
    {"INFRASYS_CLDMA_PDN_MD",                   FALSE},

     /*40*/
    {"INFRASYS_MD2MD_CCIF0",                    FALSE},
    {"INFRASYS_MD2MD_CCIF1",                    FALSE},
    {"INFRASYS_MDSYSINTF",                      TRUE},
    {"DEGBUGSYS",                               FALSE},
    {"DMA",                                     TRUE},
    {"AUXADC",                                  TRUE},
    {"UART0",                                   TRUE},
    {"UART1",                                   TRUE},
    {"UART2",                                   TRUE},
    {"UART3",                                   TRUE},
    
     /*50*/
    {"PWM",                                     FALSE},
    {"I2C0",                                    TRUE},
    {"I2C1",                                    TRUE},
    {"I2C2",                                    TRUE},
    {"SPI0",                                    TRUE},
    {"PTP_THERMAL_CTL",                         FALSE},
    {"BTIF",                                    TRUE},
    {"UART4",                                   TRUE},
    {"DISP_PWM",                                TRUE},
    {"I2C3",                                    TRUE},
    
     /*60*/
    {"IRDA",                                    TRUE},
    {"IR_TX",                                   TRUE},
    {"I2C4",                                    TRUE},
    {"USB2.0",                                  FALSE},
    {"USB2.0 SIF",                              FALSE},
    {"AUDIO",                                   FALSE},
    {"MSDC0",                                   TRUE},
    {"MSDC1",                                   TRUE},
    {"MSDC2",                                   TRUE},
    {"MSDC3",                                   TRUE},

    /*70*/
    {"WCN_AHB_SLAVE",                           FALSE},
    {"MD_PERIPHERALS",                          TRUE},
    {"MD2_PERIPHERALS",                         TRUE},
    {"G3D_CONFIG",                              TRUE},
    {"MALI",                                    TRUE},
    {"MMSYS_CONFIG",                            TRUE},
    {"MDP_RDMA",                                TRUE},
    {"MDP_RSZ0",                                TRUE},
    {"MDP_RSZ1",                                TRUE},
    {"MDP_WDMA",                                TRUE},

    /*80*/
    {"MDP_WROT",                                TRUE},
    {"MDP_TDSHP",                               TRUE},
    {"DISP_OVL0",                               TRUE},
    {"DISP_OVL1",                               TRUE},
    {"DISP_RDMA0",                              TRUE},
    {"DISP_RDMA1",                              TRUE},
    {"DISP_WDMA",                               TRUE},
    {"DISP_COLOR",                              TRUE},
    {"DISP_CCORR",                              TRUE},
    {"DISP_AAL",                                TRUE},

    /*90*/
    {"DISP_GAMMA",                              TRUE},
    {"DISP_DITHER",                             TRUE},
    {"Reserved",                                TRUE},
    {"DISP_OD",                                 TRUE},
    {"DSI",                                     TRUE},
    {"DPI",                                     TRUE},
    {"MM_MUTEX",                                TRUE},
    {"SMI_LARB0",                               TRUE},
    {"SMI_COMMON",                              TRUE},
    {"IMGSYS_CONFIG",                           TRUE},

    /*100*/
    {"IMGSYS_SMI_LARB2",                        TRUE},
    {"IMGSYS_CAM1",                             TRUE},
    {"IMGSYS_CAM2",                             TRUE},
    {"IMGSYS_CAM3",                             TRUE},
    {"IMGSYS_CAM4",                             TRUE},
    {"IMGSYS_SENINF",                           TRUE},
    {"IMGSYS_CAMSV",                            TRUE},
    {"IMGSYS_FDVT",                             TRUE}, 
    {"IMGSYS_CAM5",                             TRUE},
    {"IMGSYS_CAM6",                             TRUE},

    /*110*/
    {"IMGSYS_CAM7",                             TRUE},
    {"VDECSYS_GLOBAL_CONFIGURATION",            TRUE},
    {"SMI_LARB1",                               TRUE},
    {"VDEC_FULL_TOP",                           TRUE},
    {"VENC_GLOBAL_CON",                         TRUE},
    {"SMI_LARB3",                               TRUE},
    {"VENC",                                    TRUE},
    {"JPEG_ENC",                                TRUE},
    {"JPEG_DEC",                                TRUE},

};


#else

#error "Wrong Config type"

#endif

/*****************************************************************************
*FUNCTION DEFINITION
*****************************************************************************/
static int clear_vio_status(unsigned int module);
static int devapc_ioremap(void);
/**************************************************************************
*EXTERN FUNCTION
**************************************************************************/
int mt_devapc_check_emi_violation(void)
{

    if ((readl(IOMEM(DEVAPC0_D0_VIO_STA_3)) & ABORT_EMI) == 0) {
        return -1;
    } else {
        pr_debug("EMI violation! It should be cleared by EMI MPU driver later!\n");
        return 0;
    }
}

int mt_devapc_emi_initial(void)
{
    DEVAPC_MSG("EMI_DAPC Init start \n");
    
    devapc_ioremap();

    if (NULL != devapc_ao_base)
    {
        mt_reg_sync_writel(readl(IOMEM(DEVAPC0_APC_CON)) & (0xFFFFFFFF ^ (1 << 2)), DEVAPC0_APC_CON);
        mt_reg_sync_writel(readl(IOMEM(DEVAPC0_PD_APC_CON)) & (0xFFFFFFFF ^ (1 << 2)), DEVAPC0_PD_APC_CON);
        mt_reg_sync_writel(ABORT_EMI, DEVAPC0_D0_VIO_STA_3);
        mt_reg_sync_writel(readl(IOMEM(DEVAPC0_D0_VIO_MASK_3)) & (0xFFFFFFFF ^ (ABORT_EMI)), DEVAPC0_D0_VIO_MASK_3);
        DEVAPC_MSG("EMI_DAPC Init done \n");
        return 0;
    }
    
    return -1;
}

int mt_devapc_clear_emi_violation(void)
{
    if ((readl(IOMEM(DEVAPC0_D0_VIO_STA_3)) & ABORT_EMI) != 0) {
        mt_reg_sync_writel(ABORT_EMI, DEVAPC0_D0_VIO_STA_3);
    }
    return 0;
}


 /*
 * mt_devapc_set_permission: set module permission on device apc.
 * @module: the moudle to specify permission
 * @domain_num: domain index number
 * @permission_control: specified permission
 * no return value.
 */
int mt_devapc_set_permission(unsigned int module, E_MASK_DOM domain_num, APC_ATTR permission)
{
    volatile unsigned int* base;
    unsigned int clr_bit = 0x3 << ((module % 16) * 2);
    unsigned int set_bit = permission << ((module % 16) * 2);

    if(module >= DEVAPC_DEVICE_NUMBER) {
        DEVAPC_MSG("[DEVAPC] ERROR, device number %d exceeds the max number!\n", module);
        return -1;
    }

    if (DEVAPC_DOMAIN_AP == domain_num) {
        base = DEVAPC0_D0_APC_0 + (module / 16) * 4;
    } else if (DEVAPC_DOMAIN_MD1 == domain_num) {
        base = DEVAPC0_D1_APC_0 + (module / 16) * 4;
    } else if (DEVAPC_DOMAIN_CONN == domain_num) {
        base = DEVAPC0_D2_APC_0 + (module / 16) * 4;
    } else if (DEVAPC_DOMAIN_MD32 == domain_num) {
        base = DEVAPC0_D3_APC_0 + (module / 16) * 4;
#if defined(CONFIG_ARCH_MT6735)
    } else if (DEVAPC_DOMAIN_MM == domain_num) {
        base = DEVAPC0_D4_APC_0 + (module / 16) * 4;
    } else if (DEVAPC_DOMAIN_MD3 == domain_num) {
        base = DEVAPC0_D5_APC_0 + (module / 16) * 4;
    } else if (DEVAPC_DOMAIN_MFG == domain_num) {
        base = DEVAPC0_D6_APC_0 + (module / 16) * 4;
#elif defined(CONFIG_ARCH_MT6735M)
    //blank intentionally
#elif defined(CONFIG_ARCH_MT6753)
    } else if (DEVAPC_DOMAIN_MM == domain_num) {
        base = DEVAPC0_D4_APC_0 + (module / 16) * 4;
    } else if (DEVAPC_DOMAIN_MD3 == domain_num) {
        base = DEVAPC0_D5_APC_0 + (module / 16) * 4;
    } else if (DEVAPC_DOMAIN_MFG == domain_num) {
        base = DEVAPC0_D6_APC_0 + (module / 16) * 4;
#else
#error "Wrong Config type"
#endif
    } else {
        DEVAPC_MSG("[DEVAPC] ERROR, domain number %d exceeds the max number!\n", domain_num);
        return -2;
    }

    mt_reg_sync_writel(readl(base) & ~clr_bit, base);
    mt_reg_sync_writel(readl(base) | set_bit, base);
    return 0;
}
/**************************************************************************
*STATIC FUNCTION
**************************************************************************/

static int devapc_ioremap(void)
{
    struct device_node *node = NULL;
    /*IO remap*/
    node = of_find_compatible_node(NULL, NULL, "mediatek,DEVAPC_AO");
    if (node) {
       devapc_ao_base = of_iomap(node, 0);
       DEVAPC_MSG("[DEVAPC] AO_ADDRESS %p \n",devapc_ao_base);
    } else {
       pr_err("[DEVAPC] can't find DAPC_AO compatible node \n");
       return -1;
    }

    node = of_find_compatible_node(NULL, NULL, "mediatek,DEVAPC");
    if (node) {
       devapc_pd_base = of_iomap(node, 0);
       devapc_irq = irq_of_parse_and_map(node, 0);
       DEVAPC_MSG("[DEVAPC] PD_ADDRESS %p, IRD: %d \n",devapc_pd_base,devapc_irq );
    } else {
       pr_err("[DEVAPC] can't find DAPC_PD compatible node \n");
       return -1;
    }

    return 0;
}

#ifdef CONFIG_MTK_HIBERNATION
extern void mt_irq_set_sens(unsigned int irq, unsigned int sens);
extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity);
static int devapc_pm_restore_noirq(struct device *device)
{
    if (devapc_irq != 0) {
        mt_irq_set_sens(devapc_irq, MT_LEVEL_SENSITIVE);
        mt_irq_set_polarity(devapc_irq, MT_POLARITY_LOW);
    }

    return 0;
}
#endif

static int check_vio_status(unsigned int module)
{
    unsigned int apc_index = 0;
    unsigned int apc_bit_index = 0;
    unsigned int vio_status = 0;
    
    apc_index = module / (MOD_NO_IN_1_DEVAPC*2);
    apc_bit_index = module % (MOD_NO_IN_1_DEVAPC*2);

    switch (apc_index){
        case 0: 
           vio_status = (*DEVAPC0_D0_VIO_STA_0 & (0x1 << apc_bit_index));
           break;
        case 1:
           vio_status = (*DEVAPC0_D0_VIO_STA_1 & (0x1 << apc_bit_index));
           break;
        case 2:
           vio_status = (*DEVAPC0_D0_VIO_STA_2 & (0x1 << apc_bit_index));
           break;
        case 3:
           vio_status = (*DEVAPC0_D0_VIO_STA_3 & (0x1 << apc_bit_index));
           break;
        case 4:
           vio_status = (*DEVAPC0_D0_VIO_STA_4 & (0x1 << apc_bit_index));
           break;
        default:
           break;
    }

    if (vio_status) {
        return 1;
    }

    return 0;
    
}


#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT) || defined(CONFIG_ARM_PSCI) || defined(CONFIG_MTK_PSCI)
//ATF or TEE runs this path
static void start_devapc(void)
{

    mt_reg_sync_writel(readl(DEVAPC0_PD_APC_CON) & (0xFFFFFFFF ^ (1<<2)), DEVAPC0_PD_APC_CON);

    DEVAPC_MSG("[DEVAPC] Making SMC call to ATF.\n");

    /*Set DAPC in ATF*/
    mt_secure_call(MTK_SIP_KERNEL_DAPC_INIT, 0, 0, 0);

}

#else

//This path will not be executed (Denali has only ATF and TEE, no kernel only)


/*
 * set_module_apc: set module permission on device apc.
 * @module: the moudle to specify permission
 * @devapc_num: device apc index number (device apc 0 or 1)
 * @domain_num: domain index number (AP or MD domain)
 * @permission_control: specified permission
 * no return value.
 */

static void set_module_apc(unsigned int module, E_MASK_DOM domain_num , APC_ATTR permission_control)
{

    volatile unsigned int* base= 0;
    unsigned int clr_bit = 0x3 << ((module % 16) * 2);
    unsigned int set_bit = permission_control << ((module % 16) * 2);

    if(module >= DEVAPC_DEVICE_NUMBER) {
        DEVAPC_MSG("set_module_apc : device number %d exceeds the max number!\n", module);
        return;
    }

    if (DEVAPC_DOMAIN_AP == domain_num) {
        /*base = (unsigned int*)((uintptr_t)DEVAPC0_D0_APC_0 + (module / 16) * 4);*/
    } else if (DEVAPC_DOMAIN_MD1 == domain_num) {
        base = (unsigned int*)((uintptr_t)DEVAPC0_D1_APC_0 + (module / 16) * 4);
#if defined(CONFIG_ARCH_MT6735)
    } else if (DEVAPC_DOMAIN_MD3 == domain_num) {
        base = (unsigned int*)((uintptr_t)DEVAPC0_D5_APC_0 + (module / 16) * 4);
#elif defined(CONFIG_ARCH_MT6735M)
    //blank intentionally
#elif defined(CONFIG_ARCH_MT6753)
    } else if (DEVAPC_DOMAIN_MD3 == domain_num) {
        base = (unsigned int*)((uintptr_t)DEVAPC0_D5_APC_0 + (module / 16) * 4);
#else
#error "Wrong Config type"
#endif
    } else {
        DEVAPC_MSG("set_module_apc : domain number %d exceeds the max number!\n", domain_num);
        return;
    }

    mt_reg_sync_writel( readl(base) & ~clr_bit, base);
    mt_reg_sync_writel( readl(base) | set_bit, base);
}

/*
 * unmask_module_irq: unmask device apc irq for specified module.
 * @module: the moudle to unmask
 * @devapc_num: device apc index number (device apc 0 or 1)
 * @domain_num: domain index number (AP or MD domain)
 * no return value.
 */
static int unmask_module_irq(unsigned int module)
{

    unsigned int apc_index = 0;
    unsigned int apc_bit_index = 0;

    apc_index = module / (MOD_NO_IN_1_DEVAPC*2);
    apc_bit_index = module % (MOD_NO_IN_1_DEVAPC*2);

    switch (apc_index) {
    case 0:
        *DEVAPC0_D0_VIO_MASK_0 &= ~(0x1 << apc_bit_index);
        break;
    case 1:
        *DEVAPC0_D0_VIO_MASK_1 &= ~(0x1 << apc_bit_index);
        break;
    case 2:
        *DEVAPC0_D0_VIO_MASK_2 &= ~(0x1 << apc_bit_index);
        break;
    case 3:
        *DEVAPC0_D0_VIO_MASK_3 &= ~(0x1 << apc_bit_index);
        break;
    case 4:
        *DEVAPC0_D0_VIO_MASK_4 &= ~(0x1 << apc_bit_index);
        break;
    default:
        DEVAPC_MSG("UnMask_Module_IRQ_ERR :check if domain master setting is correct or not !\n");
        break;
    }
    return 0;
}

static void init_devpac(void)
{
   /* clear violation*/
   mt_reg_sync_writel(0x80000000, DEVAPC0_VIO_DBG0);
   mt_reg_sync_writel(readl(DEVAPC0_APC_CON) &  (0xFFFFFFFF ^ (1<<2)), DEVAPC0_APC_CON);
   mt_reg_sync_writel(readl(DEVAPC0_PD_APC_CON) & (0xFFFFFFFF ^ (1<<2)), DEVAPC0_PD_APC_CON);
}


/*
 * start_devapc: start device apc for MD
 */
static int start_devapc(void)
{
    int i = 0;

    DEVAPC_MSG("[DEVAPC] Kernel started");

    init_devpac();
    
    for (i = 0; i < (sizeof(devapc_devices) / sizeof(devapc_devices[0])); i++) {
        if (TRUE == devapc_devices[i].forbidden) {
            clear_vio_status(i);
            unmask_module_irq(i);
            /*MD1 is domain1*/
            set_module_apc(i, DEVAPC_DOMAIN_MD1, E_L3);

#if defined(CONFIG_ARCH_MT6735)
            /*MD3 is domain5*/            
            set_module_apc(i, DEVAPC_DOMAIN_MD3, E_L3);
#elif defined(CONFIG_ARCH_MT6735M)
            //blank intentionally
#elif defined(CONFIG_ARCH_MT6753)
            /*MD3 is domain5*/            
            set_module_apc(i, DEVAPC_DOMAIN_MD3, E_L3);
#else
#error "Wrong Config type"
#endif

        }
    }
    return 0;
}


#endif


/*
 * clear_vio_status: clear violation status for each module.
 * @module: the moudle to clear violation status
 * @devapc_num: device apc index number (device apc 0 or 1)
 * @domain_num: domain index number (AP or MD domain)
 * no return value.
 */
static int clear_vio_status(unsigned int module)
{

    unsigned int apc_index = 0;
    unsigned int apc_bit_index = 0;

    apc_index = module / (MOD_NO_IN_1_DEVAPC*2);
    apc_bit_index = module % (MOD_NO_IN_1_DEVAPC*2);

    switch (apc_index) {
    case 0:
       *DEVAPC0_D0_VIO_STA_0 = (0x1 << apc_bit_index);
       break;
    case 1:
       *DEVAPC0_D0_VIO_STA_1 = (0x1 << apc_bit_index);
       break;
    case 2:
       *DEVAPC0_D0_VIO_STA_2 = (0x1 << apc_bit_index);
        break;
    case 3:
       *DEVAPC0_D0_VIO_STA_3 = (0x1 << apc_bit_index);
        break;
    case 4:
       *DEVAPC0_D0_VIO_STA_4 = (0x1 << apc_bit_index);
        break;
    default:
       break;
    }

    return 0;
}

static irqreturn_t devapc_violation_irq(int irq, void *dev_id)
{
    unsigned int dbg0 = 0, dbg1 = 0;
    unsigned int master_id;
    unsigned int domain_id;
    unsigned int r_w_violation;
    int i;

    dbg0 = readl(DEVAPC0_VIO_DBG0);
    dbg1 = readl(DEVAPC0_VIO_DBG1);
    master_id = dbg0 & VIO_DBG_MSTID;
    domain_id = dbg0 & VIO_DBG_DMNID;
    r_w_violation = dbg0 & VIO_DBG_RW;

#if defined(CONFIG_ARCH_MT6753)
    /* violation information improvement for Denali-3 */
    if ((domain_id >=0) && (domain_id < (sizeof(domain_settings) / sizeof(domain_settings[0])))) {
        if (1 == r_w_violation) {
            pr_debug("[DEVAPC] Device Access Permission Write Violation - Process:%s PID:%i Vio Addr:0x%x , Bus ID:0x%x, Dom ID:0x%x (%s), VIO_DBG0:0x%x\n",
                current->comm, current->pid, dbg1, master_id, domain_id, domain_settings[domain_id].name, (*DEVAPC0_VIO_DBG0));
        } else {
            pr_debug("[DEVAPC] Device Access Permission Read Violation - Process:%s PID:%i Vio Addr:0x%x , Bus ID:0x%x, Dom ID:0x%x (%s), VIO_DBG0:0x%x\n",
                current->comm, current->pid, dbg1, master_id, domain_id, domain_settings[domain_id].name, (*DEVAPC0_VIO_DBG0));
        } 
    } else {
        if (1 == r_w_violation) {
            pr_debug("[DEVAPC] Device Access Permission Write Violation - Process:%s PID:%i Vio Addr:0x%x , Bus ID:0x%x , Dom ID:0x%x, VIO_DBG0:0x%x\n",
                current->comm, current->pid, dbg1, master_id, domain_id, (*DEVAPC0_VIO_DBG0));
        } else {
            pr_debug("[DEVAPC] Device Access Permission Read Violation - Process:%s PID:%i Vio Addr:0x%x , Bus ID:0x%x , Dom ID:0x%x, VIO_DBG0:0x%x\n",
                current->comm, current->pid, dbg1, master_id, domain_id, (*DEVAPC0_VIO_DBG0));
        } 
    }
#else
    if (1 == r_w_violation) {
        pr_debug("[DEVAPC] Device Access Permission Write Violation - Process:%s PID:%i Vio Addr:0x%x , Bus ID:0x%x , Dom ID:0x%x\n",
            current->comm, current->pid, dbg1, master_id, domain_id);
    } else {
        pr_debug("[DEVAPC] Device Access Permission Read Violation - Process:%s PID:%i Vio Addr:0x%x , Bus ID:0x%x , Dom ID:0x%x\n",
            current->comm, current->pid, dbg1, master_id, domain_id);
    } 
#endif

    pr_debug("[DEVAPC] VIO_STA 0:0x%x, 1:0x%x, 2:0x%x, 3:0x%x, 4:0x%x\n",
        readl(DEVAPC0_D0_VIO_STA_0 ), readl(DEVAPC0_D0_VIO_STA_1 ), readl(DEVAPC0_D0_VIO_STA_2 ),
        readl(DEVAPC0_D0_VIO_STA_3 ), readl(DEVAPC0_D0_VIO_STA_4 ));

    for (i = 0; i < (sizeof(devapc_devices) / sizeof(devapc_devices[0])); i++) {
        
#if defined(CONFIG_ARCH_MT6753)
        /* violation information improvement */

        if (check_vio_status(i)) {
            pr_debug("[DEVAPC] Access Violation Slave: %s (index=%d)\n", devapc_devices[i].device, i);
        }
#endif

        clear_vio_status(i);
    }

    mt_reg_sync_writel(VIO_DBG_CLR , DEVAPC0_VIO_DBG0);
    dbg0 = readl(DEVAPC0_VIO_DBG0);
    dbg1 = readl(DEVAPC0_VIO_DBG1);

    if ((dbg0 != 0) || (dbg1 != 0)) {
        pr_debug("[DEVAPC] Multi-violation!\n");
        pr_debug("[DEVAPC] DBG0 = %x, DBG1 = %x\n", dbg0, dbg1);
    }

    return IRQ_HANDLED;
}

static int devapc_probe(struct platform_device *dev)
{
    int ret;

    DEVAPC_MSG("[DEVAPC] module probe. \n");
    /*IO remap*/
    devapc_ioremap();

    /*
    * Interrupts of vilation (including SPC in SMI, or EMI MPU) are triggered by the device APC.
    * need to share the interrupt with the SPC driver.
    */
    ret = request_irq(devapc_irq, (irq_handler_t)devapc_violation_irq,
        IRQF_TRIGGER_LOW | IRQF_SHARED, "devapc", &g_devapc_ctrl);
    if (ret) {
        pr_err("[DEVAPC] Failed to request irq! (%d)\n", ret);
        return ret;
    }

#if !defined(CONFIG_MTK_LEGACY)
    dapc_clk = devm_clk_get(&dev->dev, "devapc-main");
    if(IS_ERR(dapc_clk)) {
        pr_err("[DEVAPC] cannot get dapc clock.\n");
        return PTR_ERR(dapc_clk);
    }
    clk_prepare_enable(dapc_clk);
#else
    enable_clock(MT_CG_INFRA_DEVAPC ,"DEVAPC");
#endif /* !defined(CONFIG_MTK_LEGACY) */

#ifdef CONFIG_MTK_HIBERNATION
    register_swsusp_restore_noirq_func(ID_M_DEVAPC, devapc_pm_restore_noirq, NULL);
#endif

    start_devapc();

    return 0;
}


static int devapc_remove(struct platform_device *dev)
{
    return 0;
}

static int devapc_suspend(struct platform_device *dev, pm_message_t state)
{
    return 0;
}

static int devapc_resume(struct platform_device *dev)
{
    DEVAPC_MSG("[DEVAPC] module resume. \n");
    start_devapc();

    return 0;
}

struct platform_device devapc_device = {
    .name = "devapc",
    .id = -1,
};

static const struct of_device_id mt_dapc_of_match[] = {
    { .compatible = "mediatek,DEVAPC", },
    {/* sentinel */},
};

MODULE_DEVICE_TABLE(of, mt_dapc_of_match);

static struct platform_driver devapc_driver = {
    .probe = devapc_probe,
    .remove = devapc_remove,
    .suspend = devapc_suspend,
    .resume = devapc_resume,
    .driver = {
        .name = "dapc",
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
    .of_match_table = mt_dapc_of_match,
#endif
    },
};

/*
 * devapc_init: module init function.
 */
static int __init devapc_init(void)
{
    int ret;

    DEVAPC_MSG("[DEVAPC] module init. \n");

    ret = platform_device_register(&devapc_device);
    if (ret) {
        pr_err("[DEVAPC] Unable to do device register(%d)\n", ret);
        return ret;
    }

    ret = platform_driver_register(&devapc_driver);
    if (ret) {
        pr_err("[DEVAPC] Unable to register driver (%d)\n", ret);
        platform_device_unregister(&devapc_device);
        return ret;
    }

    g_devapc_ctrl = cdev_alloc();
    if (!g_devapc_ctrl) {
        pr_err("[DEVAPC] Failed to add devapc device! (%d)\n", ret);
        platform_driver_unregister(&devapc_driver);
        platform_device_unregister(&devapc_device);
        return ret;
    }
    g_devapc_ctrl->owner = THIS_MODULE;

    return 0;
}

/*
 * devapc_exit: module exit function.
 */
static void __exit devapc_exit(void)
{
    DEVAPC_MSG("[DEVAPC] DEVAPC module exit\n");
#ifdef CONFIG_MTK_HIBERNATION
    unregister_swsusp_restore_noirq_func(ID_M_DEVAPC);
#endif

}

late_initcall(devapc_init);
module_exit(devapc_exit);
MODULE_LICENSE("GPL");
