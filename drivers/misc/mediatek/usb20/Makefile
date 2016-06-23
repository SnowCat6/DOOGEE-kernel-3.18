#
# for USB OTG silicon based on Mentor Graphics INVENTRA designs
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(MTK_ALPS_BOX_SUPPORT),yes)
ccflags-y += -DMTK_ALPS_BOX_SUPPORT
endif

ifneq (,$(filter $(CONFIG_ARCH_MT6735) $(CONFIG_ARCH_MT6735M) $(CONFIG_ARCH_MT6753), y))
ccflags-y += -DMUSB_QMU_SUPPORT
#ccflags-y += -DQMU_TASKLET
endif

obj-$(CONFIG_ARCH_MT2601) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_ARCH_MT6795) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_ARCH_MT6752) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_ARCH_MT8127) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_ARCH_MT6582) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_ARCH_MT6572) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_ARCH_MT6592) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_ARCH_MT6735) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_ARCH_MT6735M) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_ARCH_MT6753) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_ARCH_MT6580) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_ARCH_MT8163) += $(subst ",,$(CONFIG_MTK_PLATFORM))/

obj-$(CONFIG_USB_MTK_HDRC) += musb_hdrc.o

musb_hdrc-y := musb_core.o
musb_hdrc-y					+= musb_gadget_ep0.o musb_gadget.o
musb_hdrc-y					+= musb_virthub.o musb_host.o
musb_hdrc-y					+= musbhsdma.o
musb_hdrc-$(CONFIG_USB_MTK_DEBUG_FS)		+= musb_debugfs.o
ifneq (,$(filter $(CONFIG_ARCH_MT6735) $(CONFIG_ARCH_MT6735M) $(CONFIG_ARCH_MT6753), y))
musb_hdrc-y  				+= mtk_qmu.o musb_qmu.o
endif

