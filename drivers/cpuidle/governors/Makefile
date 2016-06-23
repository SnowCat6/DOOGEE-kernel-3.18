#
# Makefile for cpuidle governors.
#

ifneq ($(CONFIG_MTK_LEGACY),y)
include $(srctree)/drivers/misc/mediatek/Makefile.custom
endif

obj-$(CONFIG_CPU_IDLE_GOV_LADDER) += ladder.o
obj-$(CONFIG_CPU_IDLE_GOV_MENU) += menu.o
ifneq ($(CONFIG_MTK_LEGACY),y)
obj-y += mtk_governor.o
endif
