ccflags-y += -Wno-unused-variable -g
ccflags-y += -Wno-unused-function -g

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ft5x0x/include/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ft5x0x/include/$(CONFIG_FT5X0X_FIRMWARE)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/accdet/$(MTK_PLATFORM)/

obj-y	+= ft5x0x_driver.o
ifneq ($(CONFIG_COMPAT),y)
obj-$(CONFIG_HCT_TP_GESTRUE)   += lib/ft_gesture_32bit_lib_v1.0_20140820.a
else
obj-$(CONFIG_HCT_TP_GESTRUE)   += lib/ft_gesture_64bit_lib_v1.0_20140820.a
endif

