# SPDX-License-Identifier: GPL-2.0
snd-soc-core-objs := soc-core.o soc-dapm.o soc-jack.o soc-utils.o
snd-soc-core-objs += soc-pcm.o soc-io.o soc-devres.o soc-ops.o
snd-soc-core-$(CONFIG_SND_SOC_COMPRESS) += soc-compress.o

ifneq ($(CONFIG_SND_SOC_TOPOLOGY),)
snd-soc-core-objs += soc-topology.o
endif

ifneq ($(CONFIG_SND_SOC_GENERIC_DMAENGINE_PCM),)
snd-soc-core-objs += soc-generic-dmaengine-pcm.o
endif

ifneq ($(CONFIG_SND_SOC_AC97_BUS),)
snd-soc-core-objs += soc-ac97.o
endif

ifneq ($(CONFIG_SND_SOC_ACPI),)
snd-soc-acpi-objs := soc-acpi.o
endif

obj-$(CONFIG_SND_SOC_ACPI) += snd-soc-acpi.o

obj-$(CONFIG_SND_SOC)	+= snd-soc-core.o
obj-$(CONFIG_SND_SOC)	+= generic/
obj-$(CONFIG_SND_SOC)	+= adi/
obj-$(CONFIG_SND_SOC)	+= amd/
obj-$(CONFIG_SND_SOC)	+= atmel/
obj-$(CONFIG_SND_SOC)	+= au1x/
obj-$(CONFIG_SND_SOC)	+= bcm/
obj-$(CONFIG_SND_SOC)	+= cirrus/
obj-$(CONFIG_SND_SOC)	+= davinci/
obj-$(CONFIG_SND_SOC)	+= dwc/
obj-$(CONFIG_SND_SOC)	+= fsl/
obj-$(CONFIG_SND_SOC)	+= hisilicon/
obj-$(CONFIG_SND_SOC)	+= jz4740/
obj-$(CONFIG_SND_SOC)	+= img/
obj-$(CONFIG_SND_SOC)	+= intel/
obj-$(CONFIG_SND_SOC)	+= mediatek/
obj-$(CONFIG_SND_SOC)	+= meson/
obj-$(CONFIG_SND_SOC)	+= mxs/
obj-$(CONFIG_SND_SOC)	+= nuc900/
obj-$(CONFIG_SND_SOC)	+= omap/
obj-$(CONFIG_SND_SOC)	+= kirkwood/
obj-$(CONFIG_SND_SOC)	+= pxa/
obj-$(CONFIG_SND_SOC)	+= qcom/
obj-$(CONFIG_SND_SOC)	+= rockchip/
obj-$(CONFIG_SND_SOC)	+= samsung/
obj-$(CONFIG_SND_SOC)	+= sh/
obj-$(CONFIG_SND_SOC)	+= sirf/
obj-$(CONFIG_SND_SOC)	+= spear/
obj-$(CONFIG_SND_SOC)	+= sti/
obj-$(CONFIG_SND_SOC)	+= stm/
obj-$(CONFIG_SND_SOC)	+= sunxi/
obj-$(CONFIG_SND_SOC)	+= tegra/
obj-$(CONFIG_SND_SOC)	+= txx9/
obj-$(CONFIG_SND_SOC)	+= uniphier/
obj-$(CONFIG_SND_SOC)	+= ux500/
obj-$(CONFIG_SND_SOC)	+= xtensa/
obj-$(CONFIG_SND_SOC)	+= zte/

