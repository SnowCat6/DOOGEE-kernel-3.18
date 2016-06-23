
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

#obj-y   +=  lcm/
obj-y   +=  camera/camera/
obj-$(CONFIG_MTK_VIBRATOR)   +=  vibrator/
obj-$(CONFIG_MTK_LEDS)   +=  leds/
obj-y   +=  keypad/
#obj-y   +=  sound/
obj-y   +=  accdet/

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  alsps/cm36652-new/
endif

ifeq ($(CONFIG_MTK_EPL2182_NEW),y)
obj-y	+=  alsps/EPL2182-new/
endif
ifeq ($(CONFIG_MTK_EPL2182_NEW_FAE),y)
obj-y	+=  alsps/EPL2182-new-fae/
endif

ifeq ($(CONFIG_MTK_STK3X1X_DRIVER),y)
obj-y	+=  alsps/stk3x1x_driver/
endif
ifeq ($(CONFIG_MTK_EPL259X),y)
obj-y	+=  alsps/epl259x/
endif

ifeq ($(CONFIG_MTK_EPL89XX),y)
obj-y	+=  alsps/epl89xx/
endif

ifeq ($(CONFIG_MTK_GP2AP054A_PSGS_ALS),y)
obj-y	+=  alsps/gp2ap054a_psgs_als/
endif

ifeq ($(CONFIG_MTK_EPL8862),y)
obj-y	+=  alsps/EPL8862/
endif

ifeq ($(CONFIG_MTK_EPL8865),y)
obj-y	+=  alsps/epl8865/
endif

ifeq ($(CONFIG_MTK_AP3216C),y)
obj-y	+=  alsps/ap3216c/
endif


ifeq ($(CONFIG_MTK_MC3410_NEW),y)
obj-y   +=  accelerometer/mc3410-new/
endif

ifeq ($(CONFIG_MTK_KXTJ2_1009_NEW),y)
obj-y   +=  accelerometer/KXTJ2_1009-new/
endif
ifeq ($(CONFIG_MTK_MXC400X_NEW),y)
obj-y   +=  accelerometer/mxc400x_acc/
endif

ifeq ($(CONFIG_MTK_ITG1010_NEW),y)
obj-y   +=  gyroscope/ITG1010-new/
endif

ifeq ($(CONFIG_MTK_AKM09911_NEW),y)
obj-y	+=  magnetometer/akm09911-new/
endif
ifeq ($(CONFIG_MTK_MMC3416X_NEW),y)
obj-y	+=  magnetometer/mmc3416x-new/
endif
ifeq ($(CONFIG_MTK_BMC156_ACC),y)
obj-y	+=  accelerometer/bmc156_acc/
endif
ifeq ($(CONFIG_MTK_BMC156_MAG),y)
obj-y	+=  magnetometer/bmc156_mag/
endif
ifeq ($(CONFIG_MTK_BMA056),y)
obj-y	+=  accelerometer/bma056/
endif
ifeq ($(CONFIG_MTK_BMG160),y)
obj-y	+=  gyroscope/bmg160/
endif
ifeq ($(CONFIG_MTK_BMM056),y)
obj-y	+=  magnetometer/bmm056/
endif
endif
