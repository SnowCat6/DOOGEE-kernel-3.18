#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y				+= leds_strobe.o

ifeq ($(CONFIG_HCT_LED_SGM3785),y)
obj-y += sgm3785_drv.o
endif
