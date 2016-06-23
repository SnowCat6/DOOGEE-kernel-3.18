ifeq ($(CONFIG_MTK_TABLET_HARDWARE),y)
ccflags-y += -DMTK_TABLET_DPIDLE
endif

include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y	+= mt_spm.o
obj-y	+= mt_spm_internal.o
ifneq ($(CONFIG_MTK_FPGA),y)
obj-$(CONFIG_PM)	+= mt_spm_fs.o
obj-y	+= mt_spm_sleep.o

ifeq ($(CONFIG_ARCH_MT6735),y)
obj-y	+= mt_spm_vcore_dvfs_1.o
obj-y	+= mt_spm_sodi.o
obj-y	+= mt_spm_dpidle.o
endif
ifeq ($(CONFIG_ARCH_MT6735M),y)
obj-y	+= mt_spm_vcore_dvfs_2.o
obj-y	+= mt_spm_sodi2.o
obj-y	+= mt_spm_dpidle2.o
endif
ifeq ($(CONFIG_ARCH_MT6753),y)
obj-y	+= mt_spm_vcore_dvfs_3.o
obj-y	+= mt_spm_sodi3.o
obj-y	+= mt_spm_dpidle3.o
endif

endif
#obj-y	+= mt_spm_mcdi.o
#obj-y	+= mt_spm_talking.o
