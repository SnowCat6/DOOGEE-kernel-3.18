#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/imgsensor/src/Makefile.custom


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
obj-y += $(custom_kernel_imgsensor)
#obj-y += $(custom_kernel_imgsensor_by_platform)
#obj-y += $(custom_kernel_imgsensor_by_project)

$(info imgsensor drv by platform $(platform_drv))
$(info imgsensor drv by project $(project_drv))
