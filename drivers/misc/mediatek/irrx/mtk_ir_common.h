
#ifndef __MTK_IR_COMMON_H__
#define __MTK_IR_COMMON_H__


#include "mtk_ir_cus_define.h"

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/kernel.h>
#else
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#endif


struct mtk_ir_msg {
	u32 scancode;		/* rc scan code */
	u32 keycode;		/* linux input code */
	/* u32 rep; // repeat times ,driver not care repeat status to userspace */
};

#if MTK_IRRX_AS_MOUSE_INPUT
struct mtk_ir_mouse_code {
	u32 scanleft;
	u32 scanright;
	u32 scanup;
	u32 scandown;
	u32 scanenter;
	u32 scanswitch;
};

typedef enum {
	MTK_IR_AS_IRRX = 0,
	MTK_IR_AS_MOUSE,
} MTK_IR_DEVICE_MODE;

#endif



typedef enum {
	MTK_IR_FACTORY = 0,
	MTK_IR_NORMAL,
	MTK_IR_MAX,
} MTK_IR_MODE;



#define BTN_NONE                    0XFFFFFFFF
#define BTN_INVALID_KEY             -1

#define MTK_IR_CHUNK_SIZE sizeof(struct mtk_ir_msg)
#define MTK_IR_DEVICE_NODE   1	/* whther has mtk_ir_dev.c */


#ifdef __KERNEL__


#define IRRX_TAG      "[irrx_kernel]"

extern int ir_log_debug_on;
extern void mtk_ir_core_log_always(const char *fmt, ...);

#define IR_W_REGS_LOG(fmt, arg...) \
    do { \
        if (ir_log_debug_on)  printk(IRRX_TAG"%s, line(%d)\n"fmt, __func__, __LINE__, ##arg); \
    } while (0)


#define IR_LOG_DEBUG(fmt, arg...) \
	do { \
		if (ir_log_debug_on) mtk_ir_core_log_always(IRRX_TAG"%s, line(%d)\n"fmt, __func__, __LINE__, ##arg); \
	} while (0)

#define IR_LOG_DEBUG_TO_KERNEL(fmt, arg...) \
			do { \
				if (ir_log_debug_on) printk(IRRX_TAG"%s, line(%d)\n"fmt, __func__, __LINE__, ##arg); \
			} while (0)

#if 1
#define IR_LOG_ALWAYS(fmt, arg...)  \
	do { \
		 printk(IRRX_TAG"%s, line(%d)\n"fmt, __func__, __LINE__, ##arg);	 \
	} while (0)

#endif


#if 1
#define IR_LOG_KEY(fmt, arg...)  \
	do { \
		 mtk_ir_core_log_always(""fmt, ##arg);\
	} while (0)
#endif


#define IR_LOG_TO_KERNEL(fmt, arg...)  \
		do { \
			printk(IRRX_TAG"%s, line =%d\n"fmt, __func__, __LINE__, ##arg);	 \
		} while (0)

extern void AssertIR(const char *szExpress, const char *szFile, int i4Line);

#undef ASSERT
#define ASSERT(x)		((x) ? (void)0 : AssertIR(#x, __FILE__, __LINE__))
#else				/* usr space use */

#define IRRX_TAG      "[irrx_usr]"

#define IR_LOG_ALWAYS(fmt, arg...) \
			 printf(IRRX_TAG"%s,line(%d)\n"fmt, __func__, __LINE__, ##arg)

extern void Assert(const char *szExpress, const char *szFile, int i4Line);

#define ASSERT(x)        ((x) ? (void)0 : Assert(#x, __FILE__, __LINE__))

#endif



#endif
