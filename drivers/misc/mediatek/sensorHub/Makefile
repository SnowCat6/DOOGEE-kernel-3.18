include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors

### Workaround ###
ifneq ($(CONFIG_ARCH_MT2601),y)
obj-y += SCP_sensorHub/
obj-y += SCP_shf/
else
obj-y += CwMcuSensor/
endif
ifeq ($(CONFIG_CUSTOM_KERNEL_SENSORHUB),"OTHER_VENDOR")
obj-y += other_vendor/
endif
