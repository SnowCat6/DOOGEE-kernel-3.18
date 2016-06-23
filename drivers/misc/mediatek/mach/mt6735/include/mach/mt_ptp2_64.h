#ifndef __MT_PTP2_H__
#define __MT_PTP2_H__



#include <linux/kernel.h>
#include <mach/sync_write.h>
#include "mach/mt_reg_base.h"
#include "mach/mt_typedefs.h"
#include "mach/mt_secure_api.h"
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#ifdef __MT_PTP2_C__
	#define PTP2_EXTERN
#else
	#define PTP2_EXTERN extern
#endif



#define PTP2_REG_NUM             2



/**
* PTP2 control register 0
*/
//0x10200678
#define PTP2_DET_SWRST                     31:31
#define PTP2_DET_RAMPSTART                 13:12
#define PTP2_DET_RAMPSTEP                  11:8
#define PTP2_DET_DELAY                      7:4
#define PTP2_DET_AUTO_STOP_BYPASS_ENABLE    3:3
#define PTP2_DET_TRIGGER_PUL_DELAY          2:2
#define PTP2_CTRL_ENABLE                    1:1
#define PTP2_DET_ENABLE                     0:0



/**
* PTP2 control register 0
*/
//0x1020067C
#define PTP2_MP0_nCORERESET        31:28
#define PTP2_MP0_STANDBYWFE        27:24
#define PTP2_MP0_STANDBYWFI        23:20
#define PTP2_MP0_STANDBYWFIL2      19:19
#define PTP2_MP1_nCORERESET        18:15
#define PTP2_MP1_STANDBYWFE        14:11
#define PTP2_MP1_STANDBYWFI        10:7
#define PTP2_MP1_STANDBYWFIL2       6:6



/*
 * PTP2 rampstart rate
 */
enum {
    PTP2_RAMPSTART_0 = 0b00,
    PTP2_RAMPSTART_1 = 0b01,
    PTP2_RAMPSTART_2 = 0b10,
    PTP2_RAMPSTART_3 = 0b11
};



/**
 * PTP2 LO trigger
 */
enum {
    PTP2_CORE_RESET  = 0,
    PTP2_DEBUG_RESET = 1,
    PTP2_STANDBYWFI  = 2,
    PTP2_STANDBYWFE  = 3,
    PTP2_STANDBYWFI2 = 4,
    PTP2_TRIG_NUM    = 5
};



/**
* PTP2 register setting
*/
struct PTP2_data {
    unsigned int SWRST; //31:31
    unsigned int RAMPSTART; //13:12
    unsigned int RAMPSTEP; //11:8
    unsigned int DELAY; //7:4
    unsigned int AUTO_STOP_BYPASS_ENABLE; //3:3
    unsigned int TRIGGER_PUL_DELAY; //2:2
    unsigned int CTRL_ENABLE; //1:1
    unsigned int DET_ENABLE; //0:0
};

struct PTP2_trig {
    unsigned int mp0_nCORE_RESET; //31:28
    unsigned int mp0_STANDBY_WFE; //27:24
    unsigned int mp0_STANDBY_WFI; //23:20
    unsigned int mp0_STANDBY_WFIL2; //19:19
    unsigned int mp1_nCORE_RESET; //18:15
    unsigned int mp1_STANDBY_WFE; //14:11
    unsigned int mp1_STANDBY_WFI; //10:7
    unsigned int mp1_STANDBY_WFIL2; //6:6
};



PTP2_EXTERN void turn_on_LO(void);
PTP2_EXTERN void turn_off_LO(void);



#undef PTP2_EXTERN
#endif  // __MT_PTP2_H__
