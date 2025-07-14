#
# chenhan add for ois devices Makefile
#
MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek

ifeq ($(CONFIG_MTK_CAM_PD2204F_EX), y)
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/ois/DW9781C
obj-y += DW9781C/
endif

ifeq ($(CONFIG_MTK_CAM_PD2215F_EX), y)
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/ois/DW9781C
obj-y += DW9781C/
endif

ifeq ($(CONFIG_MTK_CAM_PD2257F_EX), y)
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/mtk/main/common/ois/DW9781C
obj-y += DW9781C/
endif
ifneq ($(findstring $(CONFIG_MTK_CAM_PD2282), y),)
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/ois/RUMBAS4SW
obj-y += RUMBAS4SW/
endif

obj-y += ois_core.o ois_util.o
