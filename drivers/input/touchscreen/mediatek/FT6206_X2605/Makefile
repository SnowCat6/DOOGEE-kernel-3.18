include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/FT6206_X2605/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/FT6206_X2605/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  focaltech_ctl.o
obj-y	+=  focaltech_driver.o
obj-y	+=  focaltech_ex_fun.o

ifneq ($(CONFIG_COMPAT),y)
obj-$(CONFIG_HCT_TP_GESTRUE)   += ft_gesture_32bit_lib_v1.0_20140820.a
else
obj-$(CONFIG_HCT_TP_GESTRUE)   += ft_gesture_64bit_lib_v1.0_20140820.a
endif

