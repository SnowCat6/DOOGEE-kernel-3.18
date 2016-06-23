include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(MTK_CUSTOM_PATH)/mmc-host/$(MTK_PLATFORM)
ccflags-y += -I$(MTK_CUSTOM_PATH)/videox/$(MTK_PLATFORM)

ifeq ($(CONFIG_ARCH_MT6735),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dispsys/$(MTK_PLATFORM)/mt6735/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6735/
obj-y += mt_vcore_dvfs_1.o
endif
ifeq ($(CONFIG_ARCH_MT6735M),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dispsys/$(MTK_PLATFORM)/mt6735m/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6735m/
obj-y += mt_vcore_dvfs_2.o
endif
ifeq ($(CONFIG_ARCH_MT6753),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dispsys/$(MTK_PLATFORM)/mt6753/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6753/
obj-y += mt_vcore_dvfs_3.o
endif

