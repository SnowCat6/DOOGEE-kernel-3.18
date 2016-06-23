include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors
obj-y   +=  mag.o
obj-y   +=  mag_factory.o

ifeq ($(CONFIG_MTK_AKM8963_NEW),y)
obj-y	+=  akm8963-new/
endif

ifeq ($(CONFIG_MTK_BMM050_NEW),y)
obj-y	+=  bmm050-new/
endif

ifeq ($(CONFIG_MTK_BMM056_NEW),y)
obj-y	+=  bmm056-new/
endif
ifeq ($(CONFIG_MTK_BMM056_NEW),y)
obj-y	+=  bmm056-new/
endif
ifeq ($(CONFIG_MTK_AKM8963),y)
obj-y	+=  akm8963/
endif

ifeq ($(CONFIG_MTK_YAS532),y)
obj-y   +=  yamaha532/
endif

ifeq ($(CONFIG_MTK_YAS532_NEW),y)
obj-y   +=  yamaha532-new/
endif

ifeq ($(CONFIG_MTK_AKM09911),y)
obj-y   +=  akm09911/
endif

ifeq ($(CONFIG_MTK_AKM09911_NEW),y)
obj-y   +=  akm09911-new/
endif

ifeq ($(CONFIG_MTK_S62X),y)
obj-y   +=  s62x/
endif

ifeq ($(CONFIG_MTK_IST8303),y)
obj-y   +=  ist8303-new/
endif

ifeq ($(CONFIG_MTK_ST480),y)
obj-y   +=  st480/
endif