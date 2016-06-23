#ifndef __EXTDISP_DRV_PLATFORM_H__
#define __EXTDISP_DRV_PLATFORM_H__

#include <linux/dma-mapping.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/m4u.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_irq.h>
#include <board-custom.h>
#include <linux/disp_assert_layer.h>
#include "ddp_hal.h"


#define MAX_SESSION_COUNT		5

//#define MTK_LCD_HW_3D_SUPPORT
#define ALIGN_TO(x, n)  \
	(((x) + ((n) - 1)) & ~((n) - 1))

#define MTK_EXT_DISP_ALIGNMENT 32
#define MTK_EXT_DISP_START_DSI_ISR
#define MTK_EXT_DISP_OVERLAY_SUPPORT
#define MTK_EXT_DISP_SYNC_SUPPORT
#define MTK_EXT_DISP_ION_SUPPORT

///#define EXTD_DBG_USE_INNER_BUF

#define HW_OVERLAY_COUNT                 (4)


#endif //__DISP_DRV_PLATFORM_H__
