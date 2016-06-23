#
# Makefile for cpuidle.
#

ifneq ($(CONFIG_MTK_LEGACY),y)
include $(srctree)/drivers/misc/mediatek/Makefile.custom
endif

obj-y += cpuidle.o driver.o governor.o sysfs.o governors/
ifneq ($(CONFIG_MTK_LEGACY),y)
obj-y += cpuidle-mt6735.o
obj-y += mtk_cpuidle_internal.o
endif
obj-$(CONFIG_ARCH_NEEDS_CPU_IDLE_COUPLED) += coupled.o

obj-$(CONFIG_CPU_IDLE_CALXEDA) += cpuidle-calxeda.o
obj-$(CONFIG_ARCH_KIRKWOOD) += cpuidle-kirkwood.o
