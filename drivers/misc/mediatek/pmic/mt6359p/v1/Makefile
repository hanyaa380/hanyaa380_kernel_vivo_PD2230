
ifeq ($(CONFIG_MTK_GCOV_KERNEL),y)
GCOV_PROFILE_pmic_throttling_dlpt.o := y
endif

ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic_wrap/$(CONFIG_MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/battery
ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/charger
ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/misc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/usb_c/tcpc/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/subpmic

subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/pmic/include

ifeq ($(CONFIG_MTK_TINYSYS_SSPM_SUPPORT), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sspm/v2
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sspm/$(MTK_PLATFORM)/
obj-y += pmic_ipi.o
endif


obj-$(CONFIG_MTK_PMIC_CHIP_MT6359P)	+= pmic.o pmic_common.o \
		pmic_debugfs.o \
		pmic_irq.o \
		pmic_initial_setting.o \
		pmic_api.o \
		pmic_auxadc.o \
		regulator_codegen.o \
		pmic_lp_api.o \
		pmic_efuse.o \
		pmic_throttling_dlpt.o \

obj-$(CONFIG_MTK_PMIC_CHIP_MT6359P) += pmic_oc_debug.o

