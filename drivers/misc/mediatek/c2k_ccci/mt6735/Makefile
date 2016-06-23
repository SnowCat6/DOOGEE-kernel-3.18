ifeq ($(CONFIG_MTK_C2K_SUPPORT), y)

include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y := c2k_platform.o

else
obj-  := dummy.o # avoid build error
endif
