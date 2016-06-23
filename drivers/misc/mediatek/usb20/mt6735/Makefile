#
# for USB OTG silicon based on Mentor Graphics INVENTRA designs
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq (,$(filter $(CONFIG_ARCH_MT6735) $(CONFIG_ARCH_MT6735M) $(CONFIG_ARCH_MT6753), y))
ccflags-y += -DMUSB_QMU_SUPPORT
#ccflags-y += -DQMU_TASKLET
endif

obj-$(CONFIG_USB_MTK_HDRC)	:= usb20.o usb20_phy.o
obj-$(CONFIG_USB_MTK_HDRC)  += usb20_host.o
obj-$(CONFIG_USB_MTK_OTG)   += usb20_otg_if.o
obj-$(CONFIG_USB_MTK_HDRC)	+= usb20_phy_debugfs.o
ifeq ($(CONFIG_MTK_AEE_KDUMP),y)
obj-$(CONFIG_USB_MTK_HDRC) += usb_dump.o
endif
