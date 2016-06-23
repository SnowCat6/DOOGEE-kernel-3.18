#
# Makefile for the ir drivers.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom


ifeq ($(MTK_ALPS_BOX_SUPPORT),yes)
ccflags-y += -DMTK_IRRX_SUPPORT_ENABLE
endif



MTK_IRRX_INCLUDE := $(subst ",,$(CONFIG_MTK_PLATFORM))
ccflags-y += -I$(src)/$(MTK_IRRX_INCLUDE)/
obj-y += mtk_ir_core.o mtk_ir_dev.o mtk_ir_mouse.o mtk_ir_netlink.o
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/