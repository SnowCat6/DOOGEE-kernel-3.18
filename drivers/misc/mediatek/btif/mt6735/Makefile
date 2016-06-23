# BTIF HAL driver for SOC
# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
  ccflags-y += -I$(src)/inc -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/include/mach -I$(srctree)/drivers/misc/mediatek/btif/hal_inc
  obj-y += btif_hal.o
  btif_hal-y	+= hal_btif.o hal_btif_dma.o 
# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else
    KERNELDIR ?= /lib/modules/$(shell uname -r)/build
    PWD  := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
endif
