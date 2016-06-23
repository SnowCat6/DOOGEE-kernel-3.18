#
# Makefile for external display driver.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/misc/mediatek/ext_disp/  \
             -I$(srctree)/drivers/misc/mediatek/video/     \
             -I$(srctree)/drivers/misc/mediatek/videox/$(MTK_PLATFORM)/ \
             -I$(srctree)/drivers/misc/mediatek/dispsys/ \
             -I$(srctree)/drivers/misc/mediatek/sync/ \
             -I$(srctree)/drivers/misc/mediatek/hdmi/$(CONFIG_CUSTOM_KERNEL_HDMI)/ \
             -I$(srctree)/drivers/misc/mediatek/cmdq/ \
             -I$(srctree)/drivers/misc/mediatek/hdmi/inc/ \
             -I$(srctree)/drivers/staging/android/ion

ifeq ($(CONFIG_ARCH_MT6735),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dispsys/$(MTK_PLATFORM)/mt6735/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6735/
endif
ifeq ($(CONFIG_ARCH_MT6735M),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dispsys/$(MTK_PLATFORM)/mt6735m/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6735m/
endif
ifeq ($(CONFIG_ARCH_MT6753),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dispsys/$(MTK_PLATFORM)/mt6753/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6753/
endif

ifeq ($(CONFIG_ARCH_MT6735),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/videox/$(MTK_PLATFORM)/mt6735/
endif
ifeq ($(CONFIG_ARCH_MT6735M),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/videox/$(MTK_PLATFORM)/mt6735m/
endif
ifeq ($(CONFIG_ARCH_MT6753),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/videox/$(MTK_PLATFORM)/mt6753/
endif


ifeq ($(CONFIG_CUSTOM_KERNEL_HDMI),"Sii8338")
	ccflags-y += 	-DMHL_SII8338
endif

ifeq ($(CONFIG_CUSTOM_KERNEL_HDMI),"Sii8348")
	ccflags-y += 	-DMHL_SII8348
endif

ifeq ($(CONFIG_CUSTOM_KERNEL_HDMI), "MT8193")
ccflags-y += 	-DHDMI_MT8193_SUPPORT
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hdmi/mt8193/
endif

obj-y :=  \
    mtk_ext_disp_mgr.o   \
    extd_lcm.o \
    extd_fence.o \
    external_display.o \
    extd_debug.o \
    extd_utils.o \
    Kara_1280x720.o \
    extd_hdmi_drv.o \
    extd_factory.o
