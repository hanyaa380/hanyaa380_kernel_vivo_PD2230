# SPDX-License-Identifier: GPL-2.0 */
#
# Copyright (c) 2017 MediaTek Inc.
#

ifneq ($(CONFIG_FPGA_EARLY_PORTING), y)
obj-$(CONFIG_MACH_MT6877) += ptp3_v1/
obj-$(CONFIG_MACH_MT6853) += ptp3_v1/
obj-$(CONFIG_MACH_MT6833) += ptp3_v1/
obj-$(CONFIG_MACH_MT6873) += credit_didt_v1/
obj-$(CONFIG_MACH_MT6873) += brisket_v1/
obj-$(CONFIG_MACH_MT6761) += $(subst ",,$(CONFIG_MTK_PLATFORM))/

endif
obj-$(CONFIG_MACH_MT8173) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_MACH_MT6779) += drcc_v1/
#obj-$(CONFIG_MACH_MT6768) += drcc_v1/
obj-$(CONFIG_MACH_MT6765) += spm/
obj-$(CONFIG_MACH_MT6761) += spm/
obj-$(CONFIG_MACH_MT6768) += spm/
obj-$(CONFIG_MACH_MT6768) += upower_v2/
obj-$(CONFIG_MACH_MT6877) += upower_v2/
obj-$(CONFIG_MACH_MT6768) += mcdi/
obj-$(CONFIG_MACH_MT6768) += cpuidle_v3/
obj-$(CONFIG_MACH_MT6779) += spm/
obj-$(CONFIG_MACH_MT6785) += spm/
obj-$(CONFIG_MACH_MT6785) += power_gs_v1/
obj-$(CONFIG_MTK_STATIC_POWER) += leakage_table_v2/
obj-$(CONFIG_MTK_CPU_FREQ) += cpufreq_v1/
obj-$(CONFIG_MACH_MT6739) += cpufreq_v2/
ifneq ($(CONFIG_FPGA_EARLY_PORTING),y)
obj-$(CONFIG_MTK_PTPOD_GPU) += eemgpu_v2/
endif
ifeq ($(CONFIG_MTK_UNIFY_POWER), y)
obj-$(CONFIG_MACH_MT6833) += upower_v2/
obj-$(CONFIG_MACH_MT6885) += upower_v2/
endif
obj-$(CONFIG_MTK_UNIFY_POWER) += upower_v2/
obj-$(CONFIG_MACH_MT6739) += cpufreq_v2/
obj-$(CONFIG_MACH_MT6739) += upower_v2/
obj-$(CONFIG_MACH_MT6739) += cpuidle_v3/
obj-$(CONFIG_MACH_MT6739) += power_gs_v1/
obj-$(CONFIG_MACH_MT6739) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6739) += mcdi/
obj-$(CONFIG_MACH_MT6739) += ppm_v3/
obj-$(CONFIG_MACH_MT6739) += spm_v4/
obj-$(CONFIG_MACH_MT6771) += spm_v4/
ifeq ($(CONFIG_MTK_QOS_SUPPORT), y)
obj-$(CONFIG_MACH_MT6771) += vcorefs_v3/
endif
obj-$(CONFIG_MACH_MT6739) += vcorefs_v3/
obj-$(CONFIG_MTK_SWPM) += swpm_v1/
obj-$(CONFIG_MACH_MT6779) += dcm_v1/
obj-$(CONFIG_MACH_MT6781) += dcm_v1/
obj-$(CONFIG_MACH_MT6739) += dcm_v1/
obj-$(CONFIG_MACH_MT6768) += dcm_v1/
obj-$(CONFIG_MACH_MT6877) += dcm_v1/
obj-$(CONFIG_MACH_MT6833) += dcm_v1/
obj-$(CONFIG_MACH_MT6885) += dcm_v1/
obj-$(CONFIG_MACH_MT6893) += dcm_v1/
obj-$(CONFIG_MACH_MT6873) += dcm_v1/
obj-$(CONFIG_MACH_MT6853) += dcm_v1/
ifneq ($(CONFIG_FPGA_EARLY_PORTING),y)
#obj-$(CONFIG_MACH_MT6761) += clkbuf_v1/
#obj-$(CONFIG_MACH_MT6779) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6781) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6785) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6768) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6739) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6833) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6877) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6771) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6885) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6893) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6853) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6873) += clkbuf_v1/
obj-$(CONFIG_MACH_MT6885) += srclken_rc_v1/
obj-$(CONFIG_MACH_MT6893) += srclken_rc_v1/
obj-$(CONFIG_MACH_MT6873) += srclken_rc_v1/
obj-$(CONFIG_MACH_MT6853) += srclken_rc_v1/
obj-$(CONFIG_MACH_MT6771) += pbm_v3/
obj-$(CONFIG_MACH_MT6739) += pbm_v3/
obj-$(CONFIG_MACH_MT6768) += mdpm_v1/
obj-$(CONFIG_MACH_MT6761) += mdpm_v1/
obj-$(CONFIG_MACH_MT6779) += mdpm_v2/
obj-$(CONFIG_MACH_MT6781) += mdpm_v2/
obj-$(CONFIG_MACH_MT6785) += mdpm_v2/
obj-$(CONFIG_MACH_MT6877) += mdpm_v2/
obj-$(CONFIG_MACH_MT6885) += mdpm_v2/
obj-$(CONFIG_MACH_MT6893) += mdpm_v2/
obj-$(CONFIG_MACH_MT6873) += mdpm_v2/
obj-$(CONFIG_MACH_MT6853) += mdpm_v2/
obj-$(CONFIG_MACH_MT6781) += power_throttling/
obj-$(CONFIG_MACH_MT6877) += power_throttling/
obj-$(CONFIG_MACH_MT6833) += power_throttling/
obj-$(CONFIG_MACH_MT6873) += power_throttling/
obj-$(CONFIG_MACH_MT6853) += power_throttling/
obj-$(CONFIG_MACH_MT6885) += power_throttling/
obj-$(CONFIG_MACH_MT6893) += power_throttling/
endif
obj-$(CONFIG_MACH_MT8173) += hps_v1/
obj-$(CONFIG_MACH_MT6765) += hps_v3/
obj-$(CONFIG_MACH_MT6761) += hps_v3/
obj-$(CONFIG_MACH_MT6739) += cpuidle_v3/
obj-$(CONFIG_MACH_MT6765) += cpuidle_v3/
obj-$(CONFIG_MACH_MT6761) += cpuidle_v3/
obj-$(CONFIG_MACH_MT6779) += cpuidle_v3/
obj-$(CONFIG_MACH_MT6785) += cpuidle_v3/
obj-$(CONFIG_MACH_MT6771) += cpuidle_v3/
#obj-$(CONFIG_MACH_MT6739) += cpuhotplug/
obj-$(CONFIG_MACH_MT6771) += cpuhotplug/
obj-$(CONFIG_MACH_MT6779) += cpuhotplug/
obj-$(CONFIG_MACH_MT6768) += cpuhotplug/
obj-$(CONFIG_MACH_MT6781) += cpuhotplug/
obj-$(CONFIG_MACH_MT6785) += cpuhotplug/
obj-$(CONFIG_MACH_MT6877) += cpuhotplug/
obj-$(CONFIG_MACH_MT6833) += cpuhotplug/
obj-$(CONFIG_MACH_MT6885) += cpuhotplug/
obj-$(CONFIG_MACH_MT6893) += cpuhotplug/
obj-$(CONFIG_MACH_MT6739) += cpuhotplug/
obj-$(CONFIG_MACH_MT6873) += cpuhotplug/
obj-$(CONFIG_MACH_MT6853) += cpuhotplug/
obj-$(CONFIG_MACH_MT6739) += power_gs_v1/
obj-$(CONFIG_MACH_MT6765) += power_gs_v1/
obj-$(CONFIG_MACH_MT6768) += power_gs_v1/
obj-$(CONFIG_MACH_MT6761) += power_gs_v1/
obj-$(CONFIG_MACH_MT6779) += power_gs_v1/
obj-$(CONFIG_MACH_MT6873) += power_gs_v1/
obj-$(CONFIG_MACH_MT6853) += power_gs_v1/
obj-$(CONFIG_MACH_MT6765) += udi_v1/
obj-$(CONFIG_MACH_MT6761) += udi_v1/
obj-$(CONFIG_MACH_MT6779) += udi_v2/
obj-$(CONFIG_MACH_MT6768) += udi_v2/
obj-$(CONFIG_MACH_MT6785) += udi_v2/
obj-$(CONFIG_MACH_MT6739) += mcdi/
obj-$(CONFIG_MACH_MT6765) += mcdi/
obj-$(CONFIG_MACH_MT6761) += mcdi/
obj-$(CONFIG_MACH_MT6779) += mcdi/
obj-$(CONFIG_MACH_MT6768) += mcdi/
obj-$(CONFIG_MACH_MT6785) += mcdi/
obj-$(CONFIG_MACH_MT6771) += mcdi/
ifeq ($(CONFIG_MTK_CM_MGR_LEGACY), y)
obj-$(CONFIG_MACH_MT6765) += cm_mgr_v1/
obj-$(CONFIG_MACH_MT6761) += cm_mgr_v1/
obj-$(CONFIG_MACH_MT6877) += cm_mgr_v1/
obj-$(CONFIG_MACH_MT6833) += cm_mgr_v1/
obj-$(CONFIG_MACH_MT6781) += cm_mgr_v1/
obj-$(CONFIG_MACH_MT6768) += cm_mgr_v1/
obj-$(CONFIG_MACH_MT6853) += cm_mgr_v1/
obj-$(CONFIG_MACH_MT6873) += cm_mgr_v1/
obj-$(CONFIG_MACH_MT6785) += cm_mgr_v1/
obj-$(CONFIG_MACH_MT6885) += cm_mgr_v1/
obj-$(CONFIG_MACH_MT6893) += cm_mgr_v1/
obj-$(CONFIG_MACH_MT6771) += cm_mgr_v1/
endif
obj-$(CONFIG_MTK_QOS_FRAMEWORK) += qos/
obj-$(CONFIG_MACH_MT6893) += dvfsrc/
obj-$(CONFIG_MACH_MT6885) += dvfsrc/
