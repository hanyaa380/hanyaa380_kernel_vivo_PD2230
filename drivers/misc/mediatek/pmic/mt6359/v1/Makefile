ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic_wrap/$(CONFIG_MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/battery
ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/charger
ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/misc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/usb_c/tcpc/inc

subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/pmic/include

ifeq ($(CONFIG_MTK_TINYSYS_SSPM_SUPPORT), y)
ifneq ($(strip $(CONFIG_MTK_TINYSYS_SSPM_V2)),)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sspm/v2
else ifneq ($(CONFIG_MTK_TINYSYS_SSPM_VERSION),"")
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sspm/$(CONFIG_MTK_TINYSYS_SSPM_VERSION)
else
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sspm
endif
obj-y += pmic_ipi.o
endif

ifeq ($(CONFIG_MTK_GAUGE_VERSION),30)
obj-y += mt6359_gauge.o
endif

obj-$(CONFIG_MTK_PMIC_CHIP_MT6359)	+= pmic.o pmic_common.o \
		pmic_debugfs.o \
		pmic_irq.o \
		pmic_initial_setting.o \
		pmic_api.o \
		pmic_auxadc.o \
		regulator_codegen.o \
		pmic_lp_api.o \
		pmic_efuse.o \

