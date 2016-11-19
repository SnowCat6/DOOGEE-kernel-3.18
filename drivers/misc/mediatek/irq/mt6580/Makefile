MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))

ifeq ($(CONFIG_MT_ENG_BUILD),y)
CFLAGS_irq.o += -DDEBUG
endif

obj-y += irq.o
