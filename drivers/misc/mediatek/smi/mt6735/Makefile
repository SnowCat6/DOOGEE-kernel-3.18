include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y += smi_debug.o
obj-y += mmdvfs_mgr.o

ifeq ($(CONFIG_ARCH_MT6735),y)
obj-y += smi_common_d1.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6735/
ccflags-y += -DD1
endif

ifeq ($(CONFIG_ARCH_MT6735M),y)
obj-y += smi_common_d2.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6735m/
ccflags-y += -DD2
endif

ifeq ($(CONFIG_ARCH_MT6753),y)
obj-y += smi_common_d3.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6753/
ccflags-y += -DD3
endif