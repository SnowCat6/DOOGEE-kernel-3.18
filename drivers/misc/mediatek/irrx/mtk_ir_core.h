
#ifndef __MTK_IR_RECV_H__
#define __MTK_IR_RECV_H__

#include <media/lirc_dev.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/types.h>
#include <linux/input.h>
#include <media/rc-core.h>

#include "mtk_ir_common.h"
#include "mtk_ir_regs.h"

#if MTK_IR_DEVICE_NODE
#include "mtk_ir_dev.h"
#endif


#define IR_BUS    BUS_HOST
#define IR_VERSION 11
#define IR_PRODUCT 11
#define IR_VENDOR 11


#define RC_MAP_MTK_NEC "rc_map_mtk_nec"
#define RC_MAP_MTK_RC6 "rc_map_mtk_rc6"
#define RC_MAP_MTK_RC5 "rc_map_mtk_rc5"



#define MTK_IR_DRIVER_NAME	"mtk_ir"

#define MTK_INPUT_NEC_DEVICE_NAME	"NEC_Remote_Controller"	/* here is for input device name */
#define MTK_INPUT_RC6_DEVICE_NAME	"RC6_Remote_Controller"	/* here is for input device name */
#define MTK_INPUT_RC5_DEVICE_NAME	"RC5_Remote_Controller"	/* here is for input device name */

#define SEMA_STATE_LOCK   (0)
#define SEMA_STATE_UNLOCK (1)

#define SEMA_LOCK_OK                    ((int)   0)
#define SEMA_LOCK_TIMEOUT               ((int)  -1)
#define SEMA_LOCK_FAIL                  ((int)  -2)


struct mtk_rc_core {

	unsigned int irq;	/* ir irq number */
	bool irq_register;	/* whether ir irq has been registered */
	struct rc_dev *rcdev;	/* current rcdev */
	struct lirc_driver *drv;	/* lirc_driver */
	struct platform_device *dev_current;	/* current activing device */
	struct platform_device *dev_parent;	/* this is form device tree */
	struct task_struct *k_thread;	/* input thread */
	void __iomem *reg_irrx;

#if MTK_IRRX_AS_MOUSE_INPUT
	struct input_dev *p_devmouse;	/* IR as mouse */
#endif

};

struct mtk_ir_device {
	struct list_head list;
	struct platform_device dev_platform;
};

struct mtk_ir_core_platform_data {

	const char *input_name;	/* /proc/bus/devices/input/input_name */
	struct rc_map_list *p_map_list;	/* rc map list */
	int i4_keypress_timeout;
	int (*init_hw) (void);	/* init ir_hw */
	int (*uninit_hw) (void);	/* uint ir_hw */
	 u32(*ir_hw_decode) (void *preserve);	/* decode function. preserve for future use */
	 u32(*get_customer_code) (void);	/* get customer code function */
	void (*set_customer_code) (void *preserve);	/* set customer code function */

	void (*early_suspend) (void *preserve);
	void (*late_resume) (void *preserve);

	int (*suspend) (void *preserve);
	int (*resume) (void *preserve);
#if MTK_IRRX_AS_MOUSE_INPUT
	struct mtk_ir_mouse_code mouse_code;	/* IRRX as mouse code */
	char mousename[32];
#endif

};

extern struct mtk_rc_core mtk_rc_core;

extern struct mtk_ir_device mtk_ir_dev_nec;
extern struct mtk_ir_device mtk_ir_dev_rc6;
extern struct mtk_ir_device mtk_ir_dev_rc5;
extern struct list_head mtk_ir_device_list;

extern void release(struct device *dev);

extern int mtk_ir_core_create_thread(int (*threadfn) (void *data),
				     void *data,
				     const char *ps_name,
				     struct task_struct **p_struct_out, unsigned int ui4_priority);

extern void mtk_ir_core_send_scancode(u32 scancode);
extern void mtk_ir_core_send_mapcode(u32 mapcode, int stat);

extern void mtk_ir_core_send_mapcode_auto_up(u32 mapcode, u32 ms);

extern void mtk_ir_core_get_msg_by_scancode(u32 scancode, struct mtk_ir_msg *msg);
extern void mtk_ir_core_clear_hwirq(void);
extern MTK_IR_MODE mtk_ir_core_getmode(void);
extern void rc_set_keypress_timeout(int i4value);

/* enable or disable clock */
extern int mt_ir_core_clock_enable(bool bEnable);
extern int mtk_ir_hal_clock_probe(void);

extern void mt_ir_reg_remap(unsigned long pyaddress, unsigned long value, bool bWrite);
extern int mt_ir_core_irda_clock_enable(bool bEnable);
extern int mt_ir_hal_power_enable(bool bEnable);
extern int mt_ir_hal_power_probe(void);




extern void mtk_ir_core_disable_hwirq(void);
extern void mtk_ir_core_clear_hwirq(void);
extern void mtk_ir_core_direct_clear_hwirq(void);
extern int mtk_ir_check_cnt(void);

extern int mtk_ir_core_create_attr(struct device *dev);
extern void mtk_ir_core_remove_attr(struct device *dev);

extern int mtk_ir_core_register_swirq(int trigger_type);
extern void mtk_ir_core_free_swirq(void);

extern int mtk_ir_core_pin_set(void);



#if MTK_IRRX_AS_MOUSE_INPUT
extern MTK_IR_DEVICE_MODE mtk_ir_mouse_get_device_mode(void);
extern void mtk_ir_mouse_set_device_mode(MTK_IR_DEVICE_MODE devmode);
extern struct input_dev *mtk_ir_mouse_register_input(struct platform_device *pdev);
extern int mtk_ir_mouse_proc_key(u32 scancode, struct mtk_rc_core *p_mtk_rc_core);
extern void mtk_ir_mouse_unregister_input(struct input_dev *dev);

extern int mtk_ir_mouse_get_x_smallstep(void);
extern int mtk_ir_mouse_get_y_smallstep(void);
extern int mtk_ir_mouse_get_x_largestep(void);
extern int mtk_ir_mouse_get_y_largestep(void);

extern void mtk_ir_mouse_set_x_smallstep(int xs);
extern void mtk_ir_mouse_set_y_smallstep(int ys);
extern void mtk_ir_mouse_set_x_largestep(int xl);
extern void mtk_ir_mouse_set_y_largestep(int yl);

#endif


#endif
