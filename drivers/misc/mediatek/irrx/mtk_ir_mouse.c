/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <media/rc-core.h>
#include <linux/rtpm_prio.h>	/* scher_rr */
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/device.h>


#include "mtk_ir_core.h"
#include "mtk_ir_common.h"

#if MTK_IRRX_AS_MOUSE_INPUT

static MTK_IR_DEVICE_MODE g_ir_device_mode = MTK_IR_AS_IRRX;



static int x_small_step = MOUSE_SMALL_X_STEP;
static int y_small_step = MOUSE_SMALL_Y_STEP;

static int x_large_step = MOUSE_LARGE_X_STEP;
static int y_large_step = MOUSE_LARGE_Y_STEP;

int mtk_ir_mouse_get_x_smallstep(void)
{
	return x_small_step;
}

int mtk_ir_mouse_get_y_smallstep(void)
{
	return y_small_step;
}

int mtk_ir_mouse_get_x_largestep(void)
{
	return x_large_step;
}

int mtk_ir_mouse_get_y_largestep(void)
{
	return y_large_step;
}

void mtk_ir_mouse_set_x_smallstep(int xs)
{
	x_small_step = xs;
	IR_LOG_ALWAYS("x_small_step = %d\n", x_small_step);
}

void mtk_ir_mouse_set_y_smallstep(int ys)
{
	y_small_step = ys;
	IR_LOG_ALWAYS("y_small_step = %d\n", y_small_step);
}

void mtk_ir_mouse_set_x_largestep(int xl)
{
	x_large_step = xl;
	IR_LOG_ALWAYS("x_large_step = %d\n", x_large_step);
}

void mtk_ir_mouse_set_y_largestep(int yl)
{
	y_large_step = yl;
	IR_LOG_ALWAYS("y_large_step = %d\n", y_large_step);
}


MTK_IR_DEVICE_MODE mtk_ir_mouse_get_device_mode(void)
{
	return g_ir_device_mode;
}

void mtk_ir_mouse_set_device_mode(MTK_IR_DEVICE_MODE devmode)
{
	g_ir_device_mode = devmode;
	IR_LOG_ALWAYS("g_ir_device_mode = %d\n", g_ir_device_mode);
}

int mtk_ir_mouse_proc_key(u32 scancode, struct mtk_rc_core *p_mtk_rc_core)
{
	struct input_dev *p_devmouse = p_mtk_rc_core->p_devmouse;
	struct mtk_ir_core_platform_data *pdata = p_mtk_rc_core->dev_current->dev.platform_data;
	struct rc_dev *rcdev = p_mtk_rc_core->rcdev;
	struct mtk_ir_mouse_code *p_mousecode = NULL;
	ASSERT(p_devmouse != NULL);
	ASSERT(pdata != NULL);
	p_mousecode = &(pdata->mouse_code);

	if (scancode == p_mousecode->scanleft) {
		if (!(rcdev->keypressed))	/* last key was released, this is the first time switch code down. */
		{
			IR_LOG_ALWAYS("MOUSE X_LEFT PRESS\n");
			input_report_rel(p_devmouse, REL_X, -x_small_step);	/* first time small step */
			input_report_rel(p_devmouse, REL_Y, 0);
		} else {
			IR_LOG_ALWAYS("MOUSE X_LEFT REPEAT\n");
			input_report_rel(p_devmouse, REL_X, -x_large_step);	/* repeat large step */
			input_report_rel(p_devmouse, REL_Y, 0);
		}
	} else if (scancode == p_mousecode->scanright) {
		if (!(rcdev->keypressed))	/* last key was released, this is the first time switch code down. */
		{
			IR_LOG_ALWAYS("MOUSE X_RIGHT PRESS\n");
			input_report_rel(p_devmouse, REL_X, x_small_step);	/* first time small step */
			input_report_rel(p_devmouse, REL_Y, 0);
		} else {
			IR_LOG_ALWAYS("MOUSE X_RIGHT REPEAT\n");
			input_report_rel(p_devmouse, REL_X, x_large_step);	/* repeat large step */
			input_report_rel(p_devmouse, REL_Y, 0);
		}

	} else if (scancode == p_mousecode->scanup) {
		if (!(rcdev->keypressed))	/* last key was released, this is the first time switch code down. */
		{
			IR_LOG_ALWAYS("MOUSE Y_UP PRESS\n");
			input_report_rel(p_devmouse, REL_X, 0);	/* first time small step */
			input_report_rel(p_devmouse, REL_Y, -y_small_step);
		} else {
			IR_LOG_ALWAYS("MOUSE Y_UP REPEAT\n");
			input_report_rel(p_devmouse, REL_X, 0);	/* repeat large step */
			input_report_rel(p_devmouse, REL_Y, -y_large_step);
		}


	} else if (scancode == p_mousecode->scandown) {
		if (!(rcdev->keypressed))	/* last key was released, this is the first time switch code down. */
		{
			IR_LOG_ALWAYS("MOUSE Y_DOWN PRESS\n");
			input_report_rel(p_devmouse, REL_X, 0);	/* first time small step */
			input_report_rel(p_devmouse, REL_Y, y_small_step);
		} else {
			IR_LOG_ALWAYS("MOUSE Y_DOWN REPEAT\n");
			input_report_rel(p_devmouse, REL_X, 0);	/* repeat large step */
			input_report_rel(p_devmouse, REL_Y, y_large_step);
		}

	} else if (scancode == p_mousecode->scanenter) {
		if (!(rcdev->keypressed))	/* last key was released, this is the first time switch code down. */
		{
			IR_LOG_ALWAYS("MOUSE BTN_LEFT PRESS\n");
			input_report_key(p_devmouse, BTN_LEFT, 1);
			input_report_rel(p_devmouse, REL_X, 0);
			input_report_rel(p_devmouse, REL_Y, 0);
			input_sync(p_devmouse);

			IR_LOG_ALWAYS("MOUSE BTN_LEFT RELEASE\n");
			input_report_key(p_devmouse, BTN_LEFT, 0);
			input_report_rel(p_devmouse, REL_X, 0);
			input_report_rel(p_devmouse, REL_Y, 0);

		} else {

		}



	}			/* another code as EV_KEY to send */
	else {
		return 0;
	}
	rc_keydown(rcdev, 0xffff, 0);
	input_sync(p_devmouse);
	return 1;
}

struct input_dev *mtk_ir_mouse_register_input(struct platform_device *pdev)
{
	struct input_dev *input_dev = NULL;
	struct mtk_ir_core_platform_data *pdata = NULL;
	int ret = 0;
	ASSERT(pdev != NULL);

	pdata = pdev->dev.platform_data;
	input_dev = input_allocate_device();

	if (!input_dev) {
		IR_LOG_ALWAYS("not enough memory for input device\n");
		goto end;
	}

	input_dev->name = pdata->mousename;
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.version = IR_VERSION + 1;
	input_dev->id.product = IR_PRODUCT + 1;
	input_dev->id.vendor = IR_VENDOR + 1;


	input_set_capability(input_dev, EV_REL, REL_X);
	input_set_capability(input_dev, EV_REL, REL_Y);
	input_set_capability(input_dev, EV_KEY, BTN_LEFT);
	input_set_capability(input_dev, EV_KEY, BTN_MIDDLE);
	input_set_capability(input_dev, EV_KEY, BTN_RIGHT);

	ret = input_register_device(input_dev);
	if (ret) {
		IR_LOG_ALWAYS("could not register input device\n");
		goto fail_reg_dev;
	}

	IR_LOG_ALWAYS("register %s success\n", input_dev->name);
	goto end;

fail_reg_dev:
	input_free_device(input_dev);
	input_dev = NULL;
end:

	return input_dev;
}

void mtk_ir_mouse_unregister_input(struct input_dev *dev)
{
	ASSERT(dev != NULL);
	input_unregister_device(dev);
	IR_LOG_ALWAYS("success\n");
}
#endif
