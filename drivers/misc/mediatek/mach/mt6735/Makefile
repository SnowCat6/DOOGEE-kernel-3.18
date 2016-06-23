
# Makefile for the linux kernel.
#
# Only aarch32 need to build core/ (need machine descriptor)

include $(srctree)/drivers/misc/mediatek/Makefile.custom


ifeq ($(CONFIG_ARCH_MT6735),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6735/
endif
ifeq ($(CONFIG_ARCH_MT6735M),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6735m/
EXTRA_CFLAGS += -I$(srctree)/sound/soc/mediatek/mt_soc_audio_v3
endif
ifeq ($(CONFIG_ARCH_MT6753),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/$(MTK_PLATFORM)/mt6753/
endif


ifneq ($(CONFIG_ARM64), y)
# For arm32
#ifeq ($(CONFIG_MTK_INTERNAL), yes)
#ifeq ($(MTK_USE_RESERVED_EXT_MEM), yes)
#KBUILD_CFLAGS += -DMTK_USE_RESERVED_EXT_MEM
#endif
#endif
#
obj-y := core.o
#obj-y += irq.o
#obj-y += timer.o
obj-y += mt_devs.o
#obj-y += mt_boot.o
#obj-y += board.o
obj-y += mt_cpuxgpt.o
obj-y += mt_gpt.o
obj-y += ca53_timer.o
##obj-y += mt_dma.o
##obj-y += mt_reg_dump.o
#obj-y += mt_device_apc.o
obj-y += mt_pm_init.o
obj-y += mt-smp.o
obj-y += hotplug.o
obj-y += mt-headsmp.o
obj-y += mt_hotplug.o
#obj-y += mt_hotplug_mechanism.o
obj-y += mt_hotplug_strategy_main.o mt_hotplug_strategy_core.o mt_hotplug_strategy_procfs.o mt_hotplug_strategy_cpu.o mt_hotplug_strategy_api.o mt_hotplug_strategy_algo.o

#obj-y += irq.o
obj-y += innercache.o
#obj-y += mt_l2c.o
#obj-y += eint.o
obj-y += mt_freqhopping.o
#obj-$(CONFIG_MT65XX_TRACER) += pmu_v7.o mon_interface.o mt_mon.o mt_emi_bm.o
#obj-y += emi_bwl.o
#obj-y += emi_mpu.o
ifeq ($(strip $(CONFIG_ARCH_MT6735M)),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq
obj-y += camera_isp_D2.o
obj-y += camera_sysram_D2.o
obj-y += camera_pipe_mgr_D2.o
else
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq
obj-y += camera_isp_D1.o
obj-y += camera_sysram_D1.o
obj-y += camera_fdvt_D1.o
endif
#obj-y += mt_pm_ldo.o
#obj-y += innercache.o
#obj-y += mt_l2c.o
#obj-$(CONFIG_MTK_CIRQ) += mt_cirq.o
obj-y += mt_spm_mtcmos.o
ifneq ($(CONFIG_MTK_FPGA),y)
obj-y += mt_sleep.o
endif
obj-y += mt_cpufreq.o mt_cpu_ss.o
obj-y += mt_gpufreq.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic_wrap/$(MTK_PLATFORM)
obj-y += mt_pbm.o
##obj-y += mt_emifreq.o
#
#########################
# CPU Dormant
#########################
ifneq ($(CONFIG_MTK_FPGA),y)
obj-y += mt_cpuidle.o
KBUILD_CFLAGS += -DMT_CPUIDLE
KBUILD_AFLAGS += -DMT_CPUIDLE
obj-y += cpu_dormant.o
# KBUILD_CFLAGS += -DMT_DORMANT_UT
else
obj-y += mt_cpuidle_bringup.o
endif
#########################

#
ifneq ($(CONFIG_MTK_FPGA),y)
ifeq ($(strip $(CONFIG_ARCH_MT6735)),y)
ifeq ($(CONFIG_MTK_LEGACY),y)
obj-y += mt_idle_legacy.o
else
obj-y += mt_idle.o
endif
obj-y += mt_clkbuf_ctl.o
endif
ifeq ($(strip $(CONFIG_ARCH_MT6735M)),y)
obj-y += mt_idle2.o
obj-y += mt_clkbuf_ctl2.o
endif
ifeq ($(strip $(CONFIG_ARCH_MT6753)),y)
obj-y += mt_idle3.o
obj-y += mt_clkbuf_ctl3.o
endif
endif

# DCM: ARM32
ifneq ($(CONFIG_MTK_FPGA),y)
obj-y += mt_dcm.o
else
obj-y += mt_dcm_bringup.o
endif

ifeq ($(CONFIG_MTK_LEGACY),y)
obj-y += mt_clkmgr_common_legacy.o
else
obj-y += mt_clkmgr_common.o
endif

ifeq ($(strip $(CONFIG_ARCH_MT6735M)),y)
obj-y += mt_clkmgr2.o
else ifeq ($(strip $(CONFIG_ARCH_MT6753)),y)
obj-y += mt_clkmgr3.o
else
ifeq ($(CONFIG_MTK_LEGACY),y)
obj-y += mt_clkmgr_legacy.o
else
obj-y += mt_clkmgr.o
endif
endif

#obj-y += sbchk_base.o
##obj-y += mtk_ccci_helper.o
#
#ifneq ($(CONFIG_MTK_CCCI_DEVICES),y)
#ccflags-y  += -D__USING_DUMMY_CCCI_API__
#endif
obj-y += mt_static_power.o
obj-y += mt_ptp.o
##obj-y += mssv.o
#
## Hibernation
#AFLAGS_swsusp.o	:= -DTEXT_OFFSET=$(TEXT_OFFSET)
obj-$(CONFIG_HIBERNATION)	+= cpu_hibernate.o swsusp.o
#
#KBUILD_AFLAGS	:=$(KBUILD_AFLAGS:-msoft-float=-Wa,-mfpu=neon)
#
##KBUILD_CFLAGS += -DMT_DORMANT_UT
##KBUILD_CFLAGS += -DMT_SMP_VIRTUAL_BOOT_ADDR

# Module project related part
obj-y += $(ARCH_MTK_PROJECT)/


else
############################ For arm64 #############################
# DCM: ARM64
ifneq ($(CONFIG_MTK_FPGA),y)
obj-y += mt_dcm.o
else
obj-y += mt_dcm_bringup.o
endif


#########################
# CPU Dormant
#########################
ifneq ($(CONFIG_MTK_FPGA),y)
obj-y += mt_cpuidle64.o
KBUILD_CFLAGS += -DMT_CPUIDLE
KBUILD_AFLAGS += -DMT_CPUIDLE
# KBUILD_CFLAGS += -DMT_DORMANT_UT
else
obj-y += mt_cpuidle_bringup.o
endif
#########################

## Hibernation
obj-$(CONFIG_HIBERNATION)  += hibernate64.o

obj-y += irq_write_protect.o
obj-y += mt_cpuxgpt.o
obj-y += mt_gpt.o
obj-y += ca53_timer.o
obj-y += mt_spm_mtcmos.o
ifeq ($(CONFIG_MTK_LEGACY),y)
obj-y += mt_clkmgr_common_legacy.o
else
obj-y += mt_clkmgr_common.o
endif

ifeq ($(strip $(CONFIG_ARCH_MT6735M)),y)
obj-y += mt_clkmgr2.o
else ifeq ($(strip $(CONFIG_ARCH_MT6753)),y)
obj-y += mt_clkmgr3.o
else
ifeq ($(CONFIG_MTK_LEGACY),y)
obj-y += mt_clkmgr_legacy.o
else
obj-y += mt_clkmgr.o
endif
endif

#obj-y += mt_pm_ldo.o
ifneq ($(CONFIG_MTK_FPGA),y)
obj-y += mt_sleep.o
ifeq ($(strip $(CONFIG_ARCH_MT6735)),y)
ifeq ($(CONFIG_MTK_LEGACY),y)
obj-y += mt_idle_legacy.o
else
obj-y += mt_idle.o
endif
obj-y += mt_clkbuf_ctl.o
endif
ifeq ($(strip $(CONFIG_ARCH_MT6735M)),y)
obj-y += mt_idle2.o
obj-y += mt_clkbuf_ctl2.o
endif
ifeq ($(strip $(CONFIG_ARCH_MT6753)),y)
obj-y += mt_idle3.o
obj-y += mt_clkbuf_ctl3.o
endif
endif
obj-y += mt_freqhopping.o
obj-y += mt_cpufreq.o mt_cpu_ss.o
obj-y += mt_gpufreq.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic_wrap/$(MTK_PLATFORM)
obj-y += mt_static_power.o
obj-y += mt_ptp.o
ifeq (,$(filter $(CONFIG_ARCH_MT6735M), y))
obj-y += mt_ptp2_64.o
endif
obj-y += mt_pbm.o
ifeq ($(strip $(CONFIG_ARCH_MT6735M)),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq
obj-y += camera_isp_D2.o
obj-y += camera_sysram_D2.o
obj-y += camera_pipe_mgr_D2.o
else
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq
obj-y += camera_isp_D1.o
obj-y += camera_sysram_D1.o
obj-y += camera_fdvt_D1.o
endif
# Module project related part
obj-y += $(ARCH_MTK_PROJECT)/
obj-y += innercache.o
obj-y += mt_pm_init.o
# Mediatek psci wrapper
obj-$(CONFIG_MTK_PSCI)	+= mt_psci.o
#obj-y += mt_device_apc.o

obj-y += mt_hotplug_strategy_main.o mt_hotplug_strategy_core.o mt_hotplug_strategy_procfs.o mt_hotplug_strategy_cpu.o mt_hotplug_strategy_api.o mt_hotplug_strategy_algo.o

endif

#common part
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/cam_cal),)
obj-$(CONFIG_MTK_CAM_CAL)   +=  $(ARCH_MTK_PROJECT)/cam_cal/
endif

ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/imgsensor),)
obj-y   +=  $(ARCH_MTK_PROJECT)/imgsensor/
endif

#EMI monitor & Fliper
obj-y += mt_emi_bm.o
obj-y += mt_mem_bw.o
obj-y += fliper/
