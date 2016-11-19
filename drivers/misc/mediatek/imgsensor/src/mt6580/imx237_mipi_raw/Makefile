include $(srctree)/drivers/misc/mediatek/Makefile.custom

# sensor name
sensor_name := imx237_mipi_raw

# sensor driver code
driver_src_file   := $(srctree)/drivers/misc/mediatek/imgsensor/src/$(MTK_PLATFORM)/$(sensor_name)/imx237_mipiraw_Sensor.c
driverprebuilt_src   := $(srctree)/drivers/misc/mediatek/imgsensor/src/$(MTK_PLATFORM)/$(sensor_name)/
driverprebuilt_obj   := $(objtree)/drivers/misc/mediatek/imgsensor/src/$(MTK_PLATFORM)/$(sensor_name)/

CONFIG_IMG_DRIVER_COMPILE := $(shell if [ -x "$(driver_src_file)" ]; then echo yes; \
	  else echo no; fi)

$(info imgsensor $(sensor_name) build on $(MTK_PLATFORM) by code type is $(CONFIG_IMG_DRIVER_COMPILE))

ifeq ("$(CONFIG_IMG_DRIVER_COMPILE)","yes")
# normal driver compile follow
obj-y += imx237_mipiraw_Sensor.o
else
# sensor driver obj copy follow
imgprebuilt_copy   := $(shell cp -rf $(driverprebuilt_src)/*.o  $(driverprebuilt_obj))
obj-y += imx237_mipiraw_Sensor.o
$(info imagesensor $(sensor_name) objcopy done!)
endif
