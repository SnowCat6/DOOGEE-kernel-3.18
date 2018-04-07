#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

#
# Makefile for misc devices that really don't fit anywhere else.
#
# Vanzo:yucheng on: Sun, 24 Jan 2016 18:13:26 +0800
# Modified for vanzo customization
#include $(srctree)/drivers/misc/mediatek/Makefile.custom
# End of Vanzo: yucheng
define FILTER_PROJECT_DRV
ifeq ($(wildcard $(srctree)/drivers/misc/mediatek/imgsensor/src/$(MTK_PLATFORM)/camera_project/$(ARCH_MTK_PROJECT)/$(1)),)
platform_drv += $(1)
else
project_drv += camera_project/$(ARCH_MTK_PROJECT)/$(1)
endif
endef

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

obj-y				+= kd_sensorlist.o

$(info -= project path  $(srctree)/drivers/misc/mediatek/imgsensor/src/$(MTK_PLATFORM)/camera_project/$(ARCH_MTK_PROJECT)/ =-)

ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/imgsensor/src/$(MTK_PLATFORM)/camera_project/$(ARCH_MTK_PROJECT)/camera_hw),)
obj-y				+= camera_project/$(ARCH_MTK_PROJECT)/camera_hw/
$(info by porject kd_camera_hw)
else
obj-y                           += camera_hw/
endif

$(foreach drv,$(subst $\",,$(CONFIG_CUSTOM_KERNEL_IMGSENSOR)),$(eval $(call FILTER_PROJECT_DRV,$(drv))))
custom_kernel_imgsensor_by_platform := $(addsuffix /, $(shell echo $(platform_drv)))
custom_kernel_imgsensor_by_project := $(addsuffix /, $(shell echo $(project_drv)))

# Vanzo:wangfei on: Wed, 12 Nov 2014 21:06:42 +0800
# for compatible the old style ,so use CUSTOM_KERNEL_IMGSENSOR
custom_kernel_imgsensor := $(addsuffix /, $(shell echo $(CUSTOM_KERNEL_IMGSENSOR)))

ifeq ($(custom_kernel_imgsensor),)
# End of Vanzo:wangfei
    $(foreach drv,$(subst $\",,$(CONFIG_CUSTOM_KERNEL_IMGSENSOR)),$(eval $(call FILTER_PROJECT_DRV,$(drv))))
    custom_kernel_imgsensor := $(addsuffix /, $(shell echo $(platform_drv)))
# Vanzo:wangfei on: Thu, 13 Nov 2014 16:15:59 +0800
# added for project config prio the defconfig
endif
# End of Vanzo:wangfei
# Vanzo:yucheng on: Sat, 16 Jan 2016 17:58:29 +0800
# Modified for camera customization
#obj-y += $(custom_kernel_imgsensor_by_platform)
#obj-y += $(custom_kernel_imgsensor_by_project)

obj-y += $(custom_kernel_imgsensor)
# End of Vanzo: yucheng
$(info imgsensor drv by platform $(platform_drv))
$(info imgsensor drv by project $(project_drv))
