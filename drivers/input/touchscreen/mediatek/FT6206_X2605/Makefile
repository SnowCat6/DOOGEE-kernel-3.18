include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/FT6206_X2605/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/FT6206_X2605/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  focaltech_ctl.o
obj-y	+=  focaltech_driver.o
obj-y	+=  focaltech_ex_fun.o
