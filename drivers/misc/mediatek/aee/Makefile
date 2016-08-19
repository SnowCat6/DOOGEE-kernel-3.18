MTK_PLATFORM:= $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/videox/
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/mrdump


ifdef CONFIG_MTK_ION
subdir-ccflags-y += -I$(srctree)/drivers/staging/android/ion
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
endif

ifdef CONFIG_MTK_GPU_SUPPORT
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
endif


obj-y	+= common/
obj-$(CONFIG_MTK_AEE_FEATURE)	+= aed/
obj-$(CONFIG_MTK_AEE_FEATURE)	+= ipanic/

obj-$(CONFIG_MTK_AEE_MRDUMP)	+= mrdump/

