// SPDX-License-Identifier: GPL-2.0
/*
 *  mt6833-mt6359.c  --  mt6833 mt6359 ALSA SoC machine driver
 *
 *  Copyright (c) 2020 MediaTek Inc.
 *  Author: Eason Yen <eason.yen@mediatek.com>
 */

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>


#include "../common/mtk-afe-platform-driver.h"
#include "mt6833-afe-common.h"
#include "mt6833-afe-clk.h"
#include "mt6833-afe-gpio.h"
#include "../../codecs/mt6359.h"
#include "../common/mtk-sp-spk-amp.h"

#ifdef CONFIG_SND_SOC_AW87339
#include "aw87339.h"
#endif

#ifdef CONFIG_SND_SOC_SIA8109
#include "sia81xx_common.h"
#endif
/*add by yanghen for vivo audio */
#ifdef CONFIG_VIVO_CODEC_SIA81XX 
#include "../../vivo/sipa/sipa_aux_dev_if.h"
#endif
/*
 * if need additional control for the ext spk amp that is connected
 * after Lineout Buffer / HP Buffer on the codec, put the control in
 * mt6833_mt6359_spk_amp_event()
 */
#define EXT_SPK_AMP_W_NAME "Ext_Speaker_Amp"

static int box_ver_gpio = 0;
static int box_ver_gpio_val = 1;

static const char *const mt6833_spk_type_str[] = {MTK_SPK_NOT_SMARTPA_STR,
						  MTK_SPK_RICHTEK_RT5509_STR,
						  MTK_SPK_MEDIATEK_MT6660_STR,
						  MTK_SPK_NXP_TFA98XX_STR
						  };
static const char *const
	mt6833_spk_i2s_type_str[] = {MTK_SPK_I2S_0_STR,
				     MTK_SPK_I2S_1_STR,
				     MTK_SPK_I2S_2_STR,
				     MTK_SPK_I2S_3_STR,
				     MTK_SPK_I2S_5_STR,
				     };

static const struct soc_enum mt6833_spk_type_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mt6833_spk_type_str),
			    mt6833_spk_type_str),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mt6833_spk_i2s_type_str),
			    mt6833_spk_i2s_type_str),
};

static int mt6833_spk_type_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int idx = mtk_spk_get_type();

	pr_debug("%s() = %d\n", __func__, idx);
	ucontrol->value.integer.value[0] = idx;
	return 0;
}

static int mt6833_spk_i2s_out_type_get(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int idx = mtk_spk_get_i2s_out_type();

	pr_debug("%s() = %d\n", __func__, idx);
	ucontrol->value.integer.value[0] = idx;
	return 0;
}

static int mt6833_spk_i2s_in_type_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int idx = mtk_spk_get_i2s_in_type();

	pr_debug("%s() = %d\n", __func__, idx);
	ucontrol->value.integer.value[0] = idx;
	return 0;
}

static int mt6833_mt6359_spk_amp_event(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol,
				       int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;

	dev_info(card->dev, "%s(), event %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* spk amp on control */
#ifdef CONFIG_SND_SOC_AW87339
		aw87339_spk_enable_set(true);
#endif

#ifdef CONFIG_SND_SOC_SIA8109
		sia81xx_start();
#endif
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/* spk amp off control */
#ifdef CONFIG_SND_SOC_AW87339
		aw87339_spk_enable_set(false);
#endif

#ifdef CONFIG_SND_SOC_SIA8109
		sia81xx_stop();
#endif
		break;
	default:
		break;
	}

	return 0;
};

static const struct snd_soc_dapm_widget mt6833_mt6359_widgets[] = {
	SND_SOC_DAPM_SPK(EXT_SPK_AMP_W_NAME, mt6833_mt6359_spk_amp_event),
};

static const struct snd_soc_dapm_route mt6833_mt6359_routes[] = {
	{EXT_SPK_AMP_W_NAME, NULL, "LINEOUT L"},
	{EXT_SPK_AMP_W_NAME, NULL, "Headphone L Ext Spk Amp"},
	{EXT_SPK_AMP_W_NAME, NULL, "Headphone R Ext Spk Amp"},
};

static const struct snd_kcontrol_new mt6833_mt6359_controls[] = {
	SOC_DAPM_PIN_SWITCH(EXT_SPK_AMP_W_NAME),
	SOC_ENUM_EXT("MTK_SPK_TYPE_GET", mt6833_spk_type_enum[0],
		     mt6833_spk_type_get, NULL),
	SOC_ENUM_EXT("MTK_SPK_I2S_OUT_TYPE_GET", mt6833_spk_type_enum[1],
		     mt6833_spk_i2s_out_type_get, NULL),
	SOC_ENUM_EXT("MTK_SPK_I2S_IN_TYPE_GET", mt6833_spk_type_enum[1],
		     mt6833_spk_i2s_in_type_get, NULL),
};

/*
 * define mtk_spk_i2s_mck node in dts when need mclk,
 * BE i2s need assign snd_soc_ops = mt6833_mt6359_i2s_ops
 */
static int mt6833_mt6359_i2s_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int rate = params_rate(params);
	unsigned int mclk_fs_ratio = 128;
	unsigned int mclk_fs = rate * mclk_fs_ratio;

	return snd_soc_dai_set_sysclk(rtd->cpu_dai,
				      0, mclk_fs, SND_SOC_CLOCK_OUT);
}

static const struct snd_soc_ops mt6833_mt6359_i2s_ops = {
	.hw_params = mt6833_mt6359_i2s_hw_params,
};

static int mt6833_mt6359_mtkaif_calibration(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	struct snd_soc_component *codec_component =
		snd_soc_rtdcom_lookup(rtd, CODEC_MT6359_NAME);
	int phase;
	unsigned int monitor;
	int test_done_1, test_done_2, test_done_3;
	int cycle_1, cycle_2, cycle_3;
	int prev_cycle_1, prev_cycle_2, prev_cycle_3;
	int counter;
	int mtkaif_calib_ok;
	bool need_calib_miso0, need_calib_miso1, need_calib_miso2;

	dev_info(afe->dev, "%s(), start\n", __func__);

	pm_runtime_get_sync(afe->dev);

	need_calib_miso0 = mt6833_afe_gpio_is_prepare(MT6833_AFE_GPIO_DAT_MISO0_ON);
	need_calib_miso1 = mt6833_afe_gpio_is_prepare(MT6833_AFE_GPIO_DAT_MISO1_ON);
	need_calib_miso2 = mt6833_afe_gpio_is_prepare(MT6833_AFE_GPIO_DAT_MISO2_ON);
	mt6833_afe_gpio_request(afe, true, MT6833_DAI_ADDA, 1);
	mt6833_afe_gpio_request(afe, true, MT6833_DAI_ADDA, 0);

	mt6359_mtkaif_calibration_enable(codec_component);

	/* set clock protocol 2 */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x39);

	/* set test type to synchronizer pulse */
	regmap_update_bits(afe_priv->topckgen,
			   CKSYS_AUD_TOP_CFG, 0xffff, 0x4);

	mtkaif_calib_ok = true;
	afe_priv->mtkaif_calibration_num_phase = 42;	/* mt6359: 0 ~ 42 */
	afe_priv->mtkaif_chosen_phase[0] = -1;
	afe_priv->mtkaif_chosen_phase[1] = -1;
	afe_priv->mtkaif_chosen_phase[2] = -1;

	for (phase = 0;
	     phase <= afe_priv->mtkaif_calibration_num_phase &&
	     mtkaif_calib_ok;
	     phase++) {
		mt6359_set_mtkaif_calibration_phase(codec_component,
						    phase, phase, phase);

		regmap_update_bits(afe_priv->topckgen,
				   CKSYS_AUD_TOP_CFG, 0x1, 0x1);

		test_done_1 = need_calib_miso0 ? 0 : -1;
		test_done_2 = need_calib_miso1 ? 0 : -1;
		test_done_3 = need_calib_miso2 ? 0 : -1;
		cycle_1 = -1;
		cycle_2 = -1;
		cycle_3 = -1;
		counter = 0;
		while (test_done_1 == 0 ||
		       test_done_2 == 0 ||
		       test_done_3 == 0) {
			regmap_read(afe_priv->topckgen,
				    CKSYS_AUD_TOP_MON, &monitor);
			/* get calibration done state */
			if (test_done_1 == 0)
				test_done_1 = (monitor >> 28) & 0x1;

			if (test_done_2 == 0)
				test_done_2 = (monitor >> 29) & 0x1;

			if (test_done_3 == 0)
				test_done_3 = (monitor >> 30) & 0x1;

			/* get delay cycle */
			if (test_done_1 == 1)
				cycle_1 = monitor & 0xf;

			if (test_done_2 == 1)
				cycle_2 = (monitor >> 4) & 0xf;

			if (test_done_3 == 1)
				cycle_3 = (monitor >> 8) & 0xf;

			/* handle if never test done */
			if (++counter > 10000) {
				dev_err(afe->dev, "%s(), test fail, cycle_1 %d, cycle_2 %d, cycle_3 %d, monitor 0x%x\n",
					__func__,
					cycle_1, cycle_2, cycle_3, monitor);
				mtkaif_calib_ok = false;
				break;
			}
		}

		if (phase == 0) {
			prev_cycle_1 = cycle_1;
			prev_cycle_2 = cycle_2;
			prev_cycle_3 = cycle_3;
		}

		if (cycle_1 != prev_cycle_1 && need_calib_miso0) {
			afe_priv->mtkaif_chosen_phase[0] = phase - 1;
			afe_priv->mtkaif_phase_cycle[0] = prev_cycle_1;
			need_calib_miso0 = false;
		}

		if (cycle_2 != prev_cycle_2 && need_calib_miso1) {
			afe_priv->mtkaif_chosen_phase[1] = phase - 1;
			afe_priv->mtkaif_phase_cycle[1] = prev_cycle_2;
			need_calib_miso1 = false;
		}

		if (cycle_3 != prev_cycle_3 && need_calib_miso2) {
			afe_priv->mtkaif_chosen_phase[2] = phase - 1;
			afe_priv->mtkaif_phase_cycle[2] = prev_cycle_3;
			need_calib_miso2 = false;
		}

		regmap_update_bits(afe_priv->topckgen,
				   CKSYS_AUD_TOP_CFG, 0x1, 0x0);

		if (!need_calib_miso0 && !need_calib_miso1 && !need_calib_miso2)
			break;
	}

	mt6359_set_mtkaif_calibration_phase(codec_component,
		(afe_priv->mtkaif_chosen_phase[0] < 0) ?
		0 : afe_priv->mtkaif_chosen_phase[0],
		(afe_priv->mtkaif_chosen_phase[1] < 0) ?
		0 : afe_priv->mtkaif_chosen_phase[1],
		(afe_priv->mtkaif_chosen_phase[2] < 0) ?
		0 : afe_priv->mtkaif_chosen_phase[2]);

	/* disable rx fifo */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);

	mt6359_mtkaif_calibration_disable(codec_component);

	mt6833_afe_gpio_request(afe, false, MT6833_DAI_ADDA, 1);
	mt6833_afe_gpio_request(afe, false, MT6833_DAI_ADDA, 0);

	/* disable syncword if miso pin not prepared */
	if (!mt6833_afe_gpio_is_prepare(MT6833_AFE_GPIO_DAT_MISO0_ON)) {
		dev_info(afe->dev, "%s(), miso0 syncword disabled\n",
			 __func__);
		regmap_update_bits(afe->regmap, AFE_ADDA_MTKAIF_SYNCWORD_CFG,
				   0x1 << 11, 0x1 << 11);
	}

	if (!mt6833_afe_gpio_is_prepare(MT6833_AFE_GPIO_DAT_MISO1_ON)) {
		dev_info(afe->dev, "%s(), miso1 syncword disabled\n",
			 __func__);
		regmap_update_bits(afe->regmap, AFE_ADDA_MTKAIF_SYNCWORD_CFG,
				   0x1 << 15, 0x1 << 15);
	}

	pm_runtime_put(afe->dev);

	dev_info(afe->dev, "%s(), mtkaif_chosen_phase[0/1/2]:%d/%d/%d\n",
		 __func__,
		 afe_priv->mtkaif_chosen_phase[0],
		 afe_priv->mtkaif_chosen_phase[1],
		 afe_priv->mtkaif_chosen_phase[2]);

	return 0;
}

static int mt6833_mt6359_init(struct snd_soc_pcm_runtime *rtd)
{
	struct mt6359_codec_ops ops;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	struct snd_soc_dapm_context *dapm = &rtd->card->dapm;
	struct snd_soc_component *codec_component =
		snd_soc_rtdcom_lookup(rtd, CODEC_MT6359_NAME);

	ops.enable_dc_compensation = mt6833_enable_dc_compensation;
	ops.set_lch_dc_compensation = mt6833_set_lch_dc_compensation;
	ops.set_rch_dc_compensation = mt6833_set_rch_dc_compensation;
	ops.adda_dl_gain_control = mt6833_adda_dl_gain_control;
	mt6359_set_codec_ops(codec_component, &ops);

	/* set mtkaif protocol */
	mt6359_set_mtkaif_protocol(codec_component,
				   MT6359_MTKAIF_PROTOCOL_2_CLK_P2);
	afe_priv->mtkaif_protocol = MTKAIF_PROTOCOL_2_CLK_P2;

	/* mtkaif calibration */
	if (afe_priv->mtkaif_protocol == MTKAIF_PROTOCOL_2_CLK_P2)
		mt6833_mt6359_mtkaif_calibration(rtd);

	/* disable ext amp connection */
	snd_soc_dapm_disable_pin(dapm, EXT_SPK_AMP_W_NAME);

	return 0;
}

static int mt6833_i2s_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				      struct snd_pcm_hw_params *params)
{
	dev_info(rtd->dev, "%s(), fix format to 32bit\n", __func__);

	/* fix BE i2s format to 32bit, clean param mask first */
	snd_mask_reset_range(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
			     0, SNDRV_PCM_FORMAT_LAST);

	params_set_format(params, SNDRV_PCM_FORMAT_S32_LE);
	return 0;
}

#ifdef CONFIG_MTK_VOW_SUPPORT
static const struct snd_pcm_hardware mt6833_mt6359_vow_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.period_bytes_min = 256,
	.period_bytes_max = 2 * 1024,
	.periods_min = 2,
	.periods_max = 4,
	.buffer_bytes_max = 2 * 2 * 1024,
};

static int mt6833_mt6359_vow_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	struct snd_soc_component *comp = NULL;
	struct snd_soc_rtdcom_list *rtdcom = NULL;

	dev_info(afe->dev, "%s(), start\n", __func__);
	snd_soc_set_runtime_hwparams(substream, &mt6833_mt6359_vow_hardware);

	mt6833_afe_gpio_request(afe, true, MT6833_DAI_VOW, 0);

	/* ASoC will call pm_runtime_get, but vow don't need */
	for_each_rtdcom(rtd, rtdcom) {
		comp = rtdcom->component;
		pm_runtime_put_autosuspend(comp->dev);
	}

	return 0;
}

static void mt6833_mt6359_vow_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	struct snd_soc_component *comp = NULL;
	struct snd_soc_rtdcom_list *rtdcom = NULL;

	dev_info(afe->dev, "%s(), end\n", __func__);
	mt6833_afe_gpio_request(afe, false, MT6833_DAI_VOW, 0);

	/* restore to fool ASoC */
	for_each_rtdcom(rtd, rtdcom) {
		comp = rtdcom->component;
		pm_runtime_get_sync(comp->dev);
	}
}

static const struct snd_soc_ops mt6833_mt6359_vow_ops = {
	.startup = mt6833_mt6359_vow_startup,
	.shutdown = mt6833_mt6359_vow_shutdown,
};
#endif  // #ifdef CONFIG_MTK_VOW_SUPPORT

static struct snd_soc_dai_link mt6833_mt6359_dai_links[] = {
	/* Front End DAI links */
	{
		.name = "Playback_1",
		.stream_name = "Playback_1",
		.cpu_dai_name = "DL1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_12",
		.stream_name = "Playback_12",
		.cpu_dai_name = "DL12",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_2",
		.stream_name = "Playback_2",
		.cpu_dai_name = "DL2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_3",
		.stream_name = "Playback_3",
		.cpu_dai_name = "DL3",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_4",
		.stream_name = "Playback_4",
		.cpu_dai_name = "DL4",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_5",
		.stream_name = "Playback_5",
		.cpu_dai_name = "DL5",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_6",
		.stream_name = "Playback_6",
		.cpu_dai_name = "DL6",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_7",
		.stream_name = "Playback_7",
		.cpu_dai_name = "DL7",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_8",
		.stream_name = "Playback_8",
		.cpu_dai_name = "DL8",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Playback_9",
		.stream_name = "Playback_9",
		.cpu_dai_name = "DL9",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "Capture_1",
		.stream_name = "Capture_1",
		.cpu_dai_name = "UL1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_2",
		.stream_name = "Capture_2",
		.cpu_dai_name = "UL2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_3",
		.stream_name = "Capture_3",
		.cpu_dai_name = "UL3",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_4",
		.stream_name = "Capture_4",
		.cpu_dai_name = "UL4",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_5",
		.stream_name = "Capture_5",
		.cpu_dai_name = "UL5",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_6",
		.stream_name = "Capture_6",
		.cpu_dai_name = "UL6",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_7",
		.stream_name = "Capture_7",
		.cpu_dai_name = "UL7",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_8",
		.stream_name = "Capture_8",
		.cpu_dai_name = "UL8",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_Mono_1",
		.stream_name = "Capture_Mono_1",
		.cpu_dai_name = "UL_MONO_1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_Mono_2",
		.stream_name = "Capture_Mono_2",
		.cpu_dai_name = "UL_MONO_2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "Capture_Mono_3",
		.stream_name = "Capture_Mono_3",
		.cpu_dai_name = "UL_MONO_3",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
	},

	{
		.name = "Hostless_LPBK",
		.stream_name = "Hostless_LPBK",
		.cpu_dai_name = "Hostless LPBK DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_FM",
		.stream_name = "Hostless_FM",
		.cpu_dai_name = "Hostless FM DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_Speech",
		.stream_name = "Hostless_Speech",
		.cpu_dai_name = "Hostless Speech DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_Sph_Echo_Ref",
		.stream_name = "Hostless_Sph_Echo_Ref",
		.cpu_dai_name = "Hostless_Sph_Echo_Ref_DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_Spk_Init",
		.stream_name = "Hostless_Spk_Init",
		.cpu_dai_name = "Hostless_Spk_Init_DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_ADDA_DL_I2S_OUT",
		.stream_name = "Hostless_ADDA_DL_I2S_OUT",
		.cpu_dai_name = "Hostless_ADDA_DL_I2S_OUT DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_SRC_1",
		.stream_name = "Hostless_SRC_1",
		.cpu_dai_name = "Hostless_SRC_1_DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_SRC_Bargein",
		.stream_name = "Hostless_SRC_Bargein",
		.cpu_dai_name = "Hostless_SRC_Bargein_DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	/* Back End DAI links */
	{
		.name = "Primary Codec",
		.cpu_dai_name = "ADDA",
		.codec_dai_name = "mt6359-snd-codec-aif1",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.init = mt6833_mt6359_init,
	},
	{
		.name = "AP_DMIC",
		.cpu_dai_name = "AP_DMIC",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "I2S3",
		.cpu_dai_name = "I2S3",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6833_i2s_hw_params_fixup,
	},
	{
		.name = "I2S0",
		.cpu_dai_name = "I2S0",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6833_i2s_hw_params_fixup,
	},
	{
		.name = "I2S1",
		.cpu_dai_name = "I2S1",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6833_i2s_hw_params_fixup,
	},
	{
		.name = "I2S2",
		.cpu_dai_name = "I2S2",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6833_i2s_hw_params_fixup,
	},
	{
		.name = "I2S5",
		.cpu_dai_name = "I2S5",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.ignore_suspend = 1,
		.be_hw_params_fixup = mt6833_i2s_hw_params_fixup,
	},
	{
		.name = "HW Gain 1",
		.cpu_dai_name = "HW Gain 1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "HW Gain 2",
		.cpu_dai_name = "HW Gain 2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "HW_SRC_1",
		.cpu_dai_name = "HW_SRC_1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "HW_SRC_2",
		.cpu_dai_name = "HW_SRC_2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "CONNSYS_I2S",
		.cpu_dai_name = "CONNSYS_I2S",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "PCM 1",
		.cpu_dai_name = "PCM 1",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "PCM 2",
		.cpu_dai_name = "PCM 2",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	/* dummy BE for ul memif to record from dl memif */
	{
		.name = "Hostless_UL1",
		.cpu_dai_name = "Hostless_UL1 DAI",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_UL2",
		.cpu_dai_name = "Hostless_UL2 DAI",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_UL3",
		.cpu_dai_name = "Hostless_UL3 DAI",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_UL6",
		.cpu_dai_name = "Hostless_UL6 DAI",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_HW_Gain_AAudio",
		.stream_name = "Hostless_HW_Gain_AAudio",
		.cpu_dai_name = "Hostless HW Gain AAudio DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	{
		.name = "Hostless_SRC_AAudio",
		.stream_name = "Hostless_SRC_AAudio",
		.cpu_dai_name = "Hostless SRC AAudio DAI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
	},
	/* BTCVSD */
#ifdef CONFIG_SND_SOC_MTK_BTCVSD
	{
		.name = "BTCVSD",
		.stream_name = "BTCVSD",
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.platform_name  = "18050000.mtk-btcvsd-snd",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
#endif
#ifdef CONFIG_MTK_VOW_SUPPORT
	{
		.name = "VOW_Capture",
		.stream_name = "VOW_Capture",
		.cpu_dai_name = "snd-soc-dummy-dai",
		.codec_dai_name = "mt6359-snd-codec-vow",
		.ignore_suspend = 1,
		.ops = &mt6833_mt6359_vow_ops,
	},
#endif  // #ifdef CONFIG_MTK_VOW_SUPPORT
#if defined(CONFIG_SND_SOC_MTK_SCP_SMARTPA)
	{
		.name = "SCP_SPK_Playback",
		.stream_name = "SCP_SPK_Playback",
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd_scp_spk",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
	},
#endif
#if defined(CONFIG_MTK_ULTRASND_PROXIMITY)
	{
		.name = "SCP_ULTRA_Playback",
		.stream_name = "SCP_ULTRA_Playback",
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd_scp_ultra",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
	},
#endif
 /*add by yanghen for vivo audio */
  	{
  		.name = "VIVOCODECPLAYBACK",
  		.stream_name = "Vivocodec_Playback",
  		.cpu_dai_name = "snd-soc-dummy-dai",
  		.platform_name = "snd-soc-dummy",
  		.codec_dai_name = "snd-soc-dummy-dai",
  		.codec_name = "snd-soc-dummy",
  	},
};

static struct snd_soc_card mt6833_mt6359_soc_card = {
	.name = "mt6833-mt6359",
	.owner = THIS_MODULE,
	.dai_link = mt6833_mt6359_dai_links,
	.num_links = ARRAY_SIZE(mt6833_mt6359_dai_links),

	.controls = mt6833_mt6359_controls,
	.num_controls = ARRAY_SIZE(mt6833_mt6359_controls),
	.dapm_widgets = mt6833_mt6359_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6833_mt6359_widgets),
	.dapm_routes = mt6833_mt6359_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6833_mt6359_routes),
};
/*add by yanghen for vivo audio*/
static const char *product_name;

static ssize_t product_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *ubuf)
{
	int n = 64;
	n = scnprintf(ubuf, n - 1, "%s", product_name ? product_name : "");
	ubuf[n] = 0;
	return n;
}

static struct kobj_attribute dev_attr_product =
	__ATTR(product, 0664, product_show, NULL);

static struct attribute *sys_node_attributes[] = {
	&dev_attr_product.attr,
	NULL
};

static struct attribute_group node_attribute_group = {
	.name = NULL,
	.attrs = sys_node_attributes
};

static int class_attr_create(struct device_node *np)
{
	static struct kobject *product_kobj;
	struct kobject *kobj = product_kobj;
	const char *fname = "audio-conf";
	int ret;

	if (product_kobj)
		return 0;

	ret = of_property_read_string(np, "vivo,product-model",
		&product_name);
	if (ret < 0) {
		product_name = NULL;
	}else{
		/*vivo audio Guoyijia add for distinguish PD2279F_EX(Y36) with PD2279GF_EX(Y27) begin*/
		if(!strcmp(product_name, "PD2279F_EX")){
			if(1 == box_ver_gpio_val){
				product_name = "PD2279F_EX_Y36";
			}else{
				product_name = "PD2279F_EX_Y27";
			}
			pr_info("%s: product_name: %s\n",__func__,product_name);
		}
		/*vivo audio Guoyijia add for distinguish PD2279F_EX(Y36) with PD2279GF_EX(Y27) begin*/
	}
	kobj = kobject_create_and_add(fname, kernel_kobj);
	if (!kobj) {
		pr_err("%s: kobject_create_and_add %s failed\n", __func__, fname);
		return 0;
	}

	ret = sysfs_create_group(kobj, &node_attribute_group);
	if (ret) {
		kobject_del(kobj);
		kobj = NULL;
		pr_err("%s: sysfs_create_group %s failed\n", __func__, fname);
	}

	product_kobj = kobj;
	pr_info("%s: sysfs create successful\n", __func__);
	return ret;
}
/*add by yanghen for vivo audio end*/
static int mt6833_mt6359_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt6833_mt6359_soc_card;
	struct device_node *platform_node, *codec_node, *spk_node;
	struct snd_soc_dai_link *spk_out_dai_link, *spk_iv_dai_link;
	int ret, i;
	int spk_out_dai_link_idx, spk_iv_dai_link_idx;


	dev_info(&pdev->dev, "%s(), ++\n", __func__);

	ret = mtk_spk_update_info(card, pdev,
				  &spk_out_dai_link_idx, &spk_iv_dai_link_idx,
				  &mt6833_mt6359_i2s_ops);
	if (ret) {
		dev_err(&pdev->dev, "%s(), mtk_spk_update_info error\n",
			__func__);
		return -EINVAL;
	}

	dev_info(&pdev->dev, "%s(), update spk dai\n", __func__);

	spk_out_dai_link = &mt6833_mt6359_dai_links[spk_out_dai_link_idx];
	spk_iv_dai_link = &mt6833_mt6359_dai_links[spk_iv_dai_link_idx];
	if (!spk_out_dai_link->codec_dai_name &&
	    !spk_iv_dai_link->codec_dai_name) {
		spk_node = of_get_child_by_name(pdev->dev.of_node,
					"mediatek,speaker-codec");
		if (!spk_node) {
			dev_err(&pdev->dev,
				"spk_codec of_get_child_by_name fail\n");
			return -EINVAL;
		}
		ret = snd_soc_of_get_dai_link_codecs(
				&pdev->dev, spk_node, spk_out_dai_link);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"i2s out get_dai_link_codecs fail\n");
			return -EINVAL;
		}
		ret = snd_soc_of_get_dai_link_codecs(
				&pdev->dev, spk_node, spk_iv_dai_link);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"i2s in get_dai_link_codecs fail\n");
			return -EINVAL;
		}
	}
	/*add by yanghen for vivo audio*/
	if (vivo_codec_name) {
  		for (i = 0; i < card->num_links; i++) {
  			if (!strcmp(mt6833_mt6359_dai_links[i].name, "VIVOCODECPLAYBACK")) {
  				mt6833_mt6359_dai_links[i].codec_name = vivo_codec_name;
  				mt6833_mt6359_dai_links[i].codec_dai_name = "VIVO-Codec-dai";
  			}
  		}
  	}
  	/*add by yanghen for vivo audio end*/
	dev_info(&pdev->dev, "%s(), update platform dai\n", __func__);

	platform_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,platform", 0);
	if (!platform_node) {
		dev_err(&pdev->dev, "Property 'platform' missing or invalid\n");
		return -EINVAL;
	}
	for (i = 0; i < card->num_links; i++) {
		if (mt6833_mt6359_dai_links[i].platform_name)
			continue;
		mt6833_mt6359_dai_links[i].platform_of_node = platform_node;
	}

	dev_info(&pdev->dev, "%s(), update audio-codec dai\n", __func__);

	/*vivo audio Guoyijia add for distinguish PD2279F_EX(Y36) with PD2279GF_EX(Y27) begin*/
	box_ver_gpio = of_get_named_gpio(pdev->dev.of_node, "box-version-gpio", 0);
	if (gpio_is_valid(box_ver_gpio)) {
		pr_info("%s: get modul ver gpio name success\n",__func__);
		ret = devm_gpio_request_one(&pdev->dev, box_ver_gpio,
			GPIOF_IN, "module-version-gpio");
		if (ret) {
			pr_err("%s module version GPIO requeset failed %d.\n", __func__, ret);
			devm_gpio_free(&pdev->dev, box_ver_gpio);
		} else {
			box_ver_gpio_val = gpio_get_value(box_ver_gpio);
			pr_info("%s: module version GPIO val %d.\n", __func__, box_ver_gpio_val);
		}
	} else {
		pr_err("%s: get modul ver gpio name failed\n",__func__);
	}
	/*vivo audio Guoyijia add for distinguish PD2279F_EX(Y36) with PD2279GF_EX(Y27) end*/

	codec_node = of_parse_phandle(pdev->dev.of_node,
				      "mediatek,audio-codec", 0);
	if (!codec_node) {
		dev_err(&pdev->dev,
			"Property 'audio-codec' missing or invalid\n");
		return -EINVAL;
	}
	for (i = 0; i < card->num_links; i++) {
		if (mt6833_mt6359_dai_links[i].codec_name ||
		    i == spk_out_dai_link_idx ||
		    i == spk_iv_dai_link_idx)
			continue;
		mt6833_mt6359_dai_links[i].codec_of_node = codec_node;
	}

	card->dev = &pdev->dev;
	/*add by yanghen for vivo audio*/
	class_attr_create(pdev->dev.of_node);
	/*add by yanghen for vivo audio */
#ifdef CONFIG_VIVO_CODEC_SIA81XX
	soc_aux_init_only_sia81xx(pdev,card);
#endif
	/*add by yanghen for vivo audio end */
	dev_info(&pdev->dev, "%s(), devm_snd_soc_register_card\n", __func__);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n",
			__func__, ret);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id mt6833_mt6359_dt_match[] = {
	{.compatible = "mediatek,mt6833-mt6359-sound",},
	{}
};
#endif

static const struct dev_pm_ops mt6833_mt6359_pm_ops = {
	.poweroff = snd_soc_poweroff,
	.restore = snd_soc_resume,
};

static struct platform_driver mt6833_mt6359_driver = {
	.driver = {
		.name = "mt6833-mt6359",
#ifdef CONFIG_OF
		.of_match_table = mt6833_mt6359_dt_match,
#endif
		.pm = &mt6833_mt6359_pm_ops,
	},
	.probe = mt6833_mt6359_dev_probe,
};

module_platform_driver(mt6833_mt6359_driver);

/* Module information */
MODULE_DESCRIPTION("MT6833 MT6359 ALSA SoC machine driver");
MODULE_AUTHOR("Eason Yen <eason.yen@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("mt6833 mt6359 soc card");
