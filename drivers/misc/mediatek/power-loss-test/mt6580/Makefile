#include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/misc/mediatek/mmc-host/$(CONFIG_MTK_PLATFORM)

obj-$(CONFIG_PWR_LOSS_MTK_TEST) += power_loss_mtk_test.o

power_loss_mtk_test-y += power_loss_test.o

ifeq ($(CONFIG_MTK_EMMC_SUPPORT),y)
power_loss_mtk_test-y += power_loss_emmc_test.o
endif

#obj-$(CONFIG_PWR_LOSS_MTK_TEST) := power_loss_test.o
#power_loss-objs += power_loss_test.o

#ifeq ($(CONFIG_MTK_EMMC_SUPPORT),y)
#obj-$(CONFIG_PWR_LOSS_MTK_TEST) += power_loss_emmc_test.o
#endif
