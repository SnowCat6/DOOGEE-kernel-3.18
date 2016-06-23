#ifndef _DAPC_H
#define _DAPC_H
#include <mach/mt_typedefs.h>
#define MOD_NO_IN_1_DEVAPC                  16
#define DEVAPC_TAG                          "DEVAPC"

#if defined(CONFIG_ARCH_MT6735)
    /*For EMI API DEVAPC0_D0_VIO_STA_3, idx:124*/
    #define ABORT_EMI                0x10000000
#elif defined(CONFIG_ARCH_MT6735M)
    /*For EMI API DEVAPC0_D0_VIO_STA_3, idx:115*/
    #define ABORT_EMI                0x80000
#elif defined(CONFIG_ARCH_MT6753)
    /*For EMI API DEVAPC0_D0_VIO_STA_3, idx:125*/
    #define ABORT_EMI                0x20000000
#else

#error "Wrong Config type"

#endif

/*Define constants*/
#define DEVAPC_DEVICE_NUMBER    140  //This number must be bigger than total slave
#define DEVAPC_DOMAIN_AP        0
#define DEVAPC_DOMAIN_MD1       1
#define DEVAPC_DOMAIN_CONN      2
#define DEVAPC_DOMAIN_MD32      3

#if defined(CONFIG_ARCH_MT6735)

#define DEVAPC_DOMAIN_MM        4
#define DEVAPC_DOMAIN_MD3       5
#define DEVAPC_DOMAIN_MFG       6

#define VIO_DBG_MSTID 0x000003FF
#define VIO_DBG_DMNID 0x00003C00
#define VIO_DBG_RW    0x30000000
#define VIO_DBG_CLR   0x80000000

#elif defined(CONFIG_ARCH_MT6735M)

#define VIO_DBG_MSTID 0x000003FF
#define VIO_DBG_DMNID 0x00000C00
#define VIO_DBG_RW    0x30000000
#define VIO_DBG_CLR   0x80000000

#elif defined(CONFIG_ARCH_MT6753)

#define DEVAPC_DOMAIN_MM        4
#define DEVAPC_DOMAIN_MD3       5
#define DEVAPC_DOMAIN_MFG       6

#define VIO_DBG_MSTID 0x00000FFF
#define VIO_DBG_DMNID 0x0000F000
#define VIO_DBG_RW    0x30000000
#define VIO_DBG_CLR   0x80000000

#else

#error "Wrong MACH type"

#endif


/******************************************************************************
*REGISTER ADDRESS DEFINATION
******************************************************************************/
#define DEVAPC0_D0_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0000))
#define DEVAPC0_D0_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0004))
#define DEVAPC0_D0_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0008))
#define DEVAPC0_D0_APC_3            ((P_kal_uint32)(devapc_ao_base+0x000C))
#define DEVAPC0_D0_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0010))
#define DEVAPC0_D0_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0014))
#define DEVAPC0_D0_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0018))
#define DEVAPC0_D0_APC_7            ((P_kal_uint32)(devapc_ao_base+0x001C))
#define DEVAPC0_D0_APC_8            ((P_kal_uint32)(devapc_ao_base+0x0020))

#define DEVAPC0_D1_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0100))
#define DEVAPC0_D1_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0104))
#define DEVAPC0_D1_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0108))
#define DEVAPC0_D1_APC_3            ((P_kal_uint32)(devapc_ao_base+0x010C))
#define DEVAPC0_D1_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0110))
#define DEVAPC0_D1_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0114))
#define DEVAPC0_D1_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0118))
#define DEVAPC0_D1_APC_7            ((P_kal_uint32)(devapc_ao_base+0x011C))
#define DEVAPC0_D1_APC_8            ((P_kal_uint32)(devapc_ao_base+0x0120))

#define DEVAPC0_D2_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0200))
#define DEVAPC0_D2_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0204))
#define DEVAPC0_D2_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0208))
#define DEVAPC0_D2_APC_3            ((P_kal_uint32)(devapc_ao_base+0x020C))
#define DEVAPC0_D2_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0210))
#define DEVAPC0_D2_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0214))
#define DEVAPC0_D2_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0218))
#define DEVAPC0_D2_APC_7            ((P_kal_uint32)(devapc_ao_base+0x021C))
#define DEVAPC0_D2_APC_8            ((P_kal_uint32)(devapc_ao_base+0x0220))

#define DEVAPC0_D3_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0300))
#define DEVAPC0_D3_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0304))
#define DEVAPC0_D3_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0308))
#define DEVAPC0_D3_APC_3            ((P_kal_uint32)(devapc_ao_base+0x030C))
#define DEVAPC0_D3_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0310))
#define DEVAPC0_D3_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0314))
#define DEVAPC0_D3_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0318))
#define DEVAPC0_D3_APC_7            ((P_kal_uint32)(devapc_ao_base+0x031C))
#define DEVAPC0_D3_APC_8            ((P_kal_uint32)(devapc_ao_base+0x0320))

#if defined(CONFIG_ARCH_MT6735)

#define DEVAPC0_D4_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0400))
#define DEVAPC0_D4_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0404))
#define DEVAPC0_D4_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0408))
#define DEVAPC0_D4_APC_3            ((P_kal_uint32)(devapc_ao_base+0x040C))
#define DEVAPC0_D4_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0410))
#define DEVAPC0_D4_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0414))
#define DEVAPC0_D4_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0418))

#define DEVAPC0_D5_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0500))
#define DEVAPC0_D5_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0504))
#define DEVAPC0_D5_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0508))
#define DEVAPC0_D5_APC_3            ((P_kal_uint32)(devapc_ao_base+0x050C))
#define DEVAPC0_D5_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0510))
#define DEVAPC0_D5_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0514))
#define DEVAPC0_D5_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0518))

#define DEVAPC0_D6_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0600))
#define DEVAPC0_D6_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0604))
#define DEVAPC0_D6_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0608))
#define DEVAPC0_D6_APC_3            ((P_kal_uint32)(devapc_ao_base+0x060C))
#define DEVAPC0_D6_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0610))
#define DEVAPC0_D6_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0614))
#define DEVAPC0_D6_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0618))

#define DEVAPC0_D7_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0700))
#define DEVAPC0_D7_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0704))
#define DEVAPC0_D7_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0708))
#define DEVAPC0_D7_APC_3            ((P_kal_uint32)(devapc_ao_base+0x070C))
#define DEVAPC0_D7_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0710))
#define DEVAPC0_D7_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0714))
#define DEVAPC0_D7_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0718))

#define DEVAPC0_MAS_DOM_0           ((P_kal_uint32)(devapc_ao_base+0x0A00))
#define DEVAPC0_MAS_DOM_1           ((P_kal_uint32)(devapc_ao_base+0x0A04))
#define DEVAPC0_MAS_SEC             ((P_kal_uint32)(devapc_ao_base+0x0B00))

#elif defined(CONFIG_ARCH_MT6735M)

#define DEVAPC0_MAS_DOM_0           ((P_kal_uint32)(devapc_ao_base+0x0400))
#define DEVAPC0_MAS_DOM_1           ((P_kal_uint32)(devapc_ao_base+0x0404))
#define DEVAPC0_MAS_SEC             ((P_kal_uint32)(devapc_ao_base+0x0500))

#elif defined(CONFIG_ARCH_MT6753)

#define DEVAPC0_D4_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0400))
#define DEVAPC0_D4_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0404))
#define DEVAPC0_D4_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0408))
#define DEVAPC0_D4_APC_3            ((P_kal_uint32)(devapc_ao_base+0x040C))
#define DEVAPC0_D4_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0410))
#define DEVAPC0_D4_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0414))
#define DEVAPC0_D4_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0418))

#define DEVAPC0_D5_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0500))
#define DEVAPC0_D5_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0504))
#define DEVAPC0_D5_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0508))
#define DEVAPC0_D5_APC_3            ((P_kal_uint32)(devapc_ao_base+0x050C))
#define DEVAPC0_D5_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0510))
#define DEVAPC0_D5_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0514))
#define DEVAPC0_D5_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0518))

#define DEVAPC0_D6_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0600))
#define DEVAPC0_D6_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0604))
#define DEVAPC0_D6_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0608))
#define DEVAPC0_D6_APC_3            ((P_kal_uint32)(devapc_ao_base+0x060C))
#define DEVAPC0_D6_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0610))
#define DEVAPC0_D6_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0614))
#define DEVAPC0_D6_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0618))

#define DEVAPC0_D7_APC_0            ((P_kal_uint32)(devapc_ao_base+0x0700))
#define DEVAPC0_D7_APC_1            ((P_kal_uint32)(devapc_ao_base+0x0704))
#define DEVAPC0_D7_APC_2            ((P_kal_uint32)(devapc_ao_base+0x0708))
#define DEVAPC0_D7_APC_3            ((P_kal_uint32)(devapc_ao_base+0x070C))
#define DEVAPC0_D7_APC_4            ((P_kal_uint32)(devapc_ao_base+0x0710))
#define DEVAPC0_D7_APC_5            ((P_kal_uint32)(devapc_ao_base+0x0714))
#define DEVAPC0_D7_APC_6            ((P_kal_uint32)(devapc_ao_base+0x0718))

#define DEVAPC0_MAS_DOM_0           ((P_kal_uint32)(devapc_ao_base+0x0A00))
#define DEVAPC0_MAS_DOM_1           ((P_kal_uint32)(devapc_ao_base+0x0A04))
#define DEVAPC0_MAS_SEC             ((P_kal_uint32)(devapc_ao_base+0x0B00))

#else

#error "Wrong Config type"

#endif


#define DEVAPC0_APC_CON             ((P_kal_uint32)(devapc_ao_base+0x0F00))
#define DEVAPC0_APC_LOCK_0          ((P_kal_uint32)(devapc_ao_base+0x0F04))
#define DEVAPC0_APC_LOCK_1          ((P_kal_uint32)(devapc_ao_base+0x0F08))
#define DEVAPC0_APC_LOCK_2          ((P_kal_uint32)(devapc_ao_base+0x0F0C))
#define DEVAPC0_APC_LOCK_3          ((P_kal_uint32)(devapc_ao_base+0x0F10))
#define DEVAPC0_APC_LOCK_4          ((P_kal_uint32)(devapc_ao_base+0x0F14))

#define DEVAPC0_PD_APC_CON          ((P_kal_uint32)(devapc_pd_base+0x0F00))
#define DEVAPC0_D0_VIO_MASK_0       ((P_kal_uint32)(devapc_pd_base+0x0000))
#define DEVAPC0_D0_VIO_MASK_1       ((P_kal_uint32)(devapc_pd_base+0x0004))
#define DEVAPC0_D0_VIO_MASK_2       ((P_kal_uint32)(devapc_pd_base+0x0008))
#define DEVAPC0_D0_VIO_MASK_3       ((P_kal_uint32)(devapc_pd_base+0x000C))
#define DEVAPC0_D0_VIO_MASK_4       ((P_kal_uint32)(devapc_pd_base+0x0010))
#define DEVAPC0_D0_VIO_STA_0        ((P_kal_uint32)(devapc_pd_base+0x0400))
#define DEVAPC0_D0_VIO_STA_1        ((P_kal_uint32)(devapc_pd_base+0x0404))
#define DEVAPC0_D0_VIO_STA_2        ((P_kal_uint32)(devapc_pd_base+0x0408))
#define DEVAPC0_D0_VIO_STA_3        ((P_kal_uint32)(devapc_pd_base+0x040C))
#define DEVAPC0_D0_VIO_STA_4        ((P_kal_uint32)(devapc_pd_base+0x0410))
#define DEVAPC0_VIO_DBG0            ((P_kal_uint32)(devapc_pd_base+0x0900))
#define DEVAPC0_VIO_DBG1            ((P_kal_uint32)(devapc_pd_base+0x0904))
#define DEVAPC0_DEC_ERR_CON         ((P_kal_uint32)(devapc_pd_base+0x0F80))
#define DEVAPC0_DEC_ERR_ADDR        ((P_kal_uint32)(devapc_pd_base+0x0F84))
#define DEVAPC0_DEC_ERR_ID          ((P_kal_uint32)(devapc_pd_base+0x0F88))

typedef struct {
    const char      *device;
    bool            forbidden;
} DEVICE_INFO;

typedef struct {
    const char      *name;
} DOMAIN_INFO;


#endif

