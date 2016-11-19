
# Makefile for the linux kernel.
#
# Only aarch32 need to build core/ (need machine descriptor)


obj-y += mt_gpt.o
obj-y += ca7_timer.o
obj-y += mtk_ccci_helper.o

## Hibernation
obj-$(CONFIG_MTK_HIBERNATION)	+= cpu_hibernate.o

#common part
