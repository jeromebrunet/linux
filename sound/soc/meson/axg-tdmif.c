// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
// Copyright (c) 2018 BayLibre, SAS.
// Author: Jerome Brunet <jbrunet@baylibre.com>

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>
#include "axg-tdm.h"

enum {
	TDMIF_PAD,
	TDMIF_LOOPBACK,
};

static int axg_tdmif_set_sysclk(struct snd_soc_dai *dai,
				int clk_id, unsigned int freq, int dir)
{
	struct axg_tdmif *tdmif = snd_soc_dai_get_drvdata(dai);
	int ret = -ENOTSUPP;

	if (dir == SND_SOC_CLOCK_OUT && clk_id == 0) {
		if (!tdmif->mclk) {
			dev_warn(dai->dev, "master clock not provided\n");
		} else {
			ret = clk_set_rate(tdmif->mclk, freq);
			if (!ret)
				tdmif->mclk_rate = freq;
		}
	}

	return ret;
}

static int axg_tdmif_set_tdm_slot(struct snd_soc_dai *dai, u32 tx_mask,
				  u32 rx_mask, int slots, int slot_width)
{
	struct axg_tdmif *tdmif = snd_soc_dai_get_drvdata(dai);

	dev_dbg(dai->dev, "tx: 0x%08x, rx: 0x%08x, slots %d, width %d\n",
		tx_mask, rx_mask, slots, slot_width);

	switch (slot_width) {
	case 0:
		/* DEBUG - REMOVE ME */
		dev_dbg(dai->dev, "skipping auto slot width\n");
		break;
	case 8:
	case 16:
	case 24:
	case 32:
		tdmif->slot_width = slot_width;
		break;
	default:
		dev_err(dai->dev, "unsupported slot width: %d\n", slot_width);
		return -EINVAL;
	}

	if (!tx_mask && !rx_mask) {
		dev_err(dai->dev, "bad masks: no slot\n");
		return -EINVAL;
	}

	if (!slots) {
		slots = fls(max(tx_mask, rx_mask));
		if (slots % 2)
			slots += 1;
	} else {
		if (slots > 32 || slots % 2) {
			dev_err(dai->dev, "bad slot number: %d\n", slots);
			return -EINVAL;
		}

		if (tx_mask >= (1<<slots) || rx_mask >= (1<<slots)) {
			dev_err(dai->dev,
				"bad tdm mask tx: 0x%08x rx: 0x%08x slots %d\n",
				tx_mask, rx_mask, slots);
		}
	}

	tdmif->slots = slots;
	tdmif->tx_mask = tx_mask;
	tdmif->rx_mask = rx_mask;

	return 0;
}

static int axg_tdmif_set_lrclk(struct snd_soc_dai *dai,
			       struct snd_pcm_hw_params *params)
{
	struct axg_tdmif *tdmif = snd_soc_dai_get_drvdata(dai);
	unsigned int ratio_num;
	int ret;

	ret = clk_set_rate(tdmif->lrclk, params_rate(params));
	if (ret) {
		dev_err(dai->dev, "setting sample clock failed: %d\n", ret);
		return ret;
	}

	/* Save rate to apply the component symmetry */
	tdmif->rate = params_rate(params);

	switch (tdmif->fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_RIGHT_J:
		ratio_num = 1;
		break;

	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		/*
		 * A zero duty cycle ratio will result in setting the mininum
		 * ratio possible which, for this clock, is 1 cycle of the
		 * parent mclk clock high and the rest low, This is exactly
		 * what we want here.
		 */
	        ratio_num = 0;
		break;

	default:
		return -EINVAL;
	}

	ret = clk_set_duty_cycle(tdmif->lrclk, ratio_num, 2);
	if (ret) {
		dev_err(dai->dev,
			"setting sample clock duty cycle failed: %d\n", ret);
		return ret;
	}

	ret = clk_set_phase(tdmif->lrclk,
			    axg_tdm_lrclk_invert(tdmif->fmt) ? 180 : 0);
	if (ret) {
		dev_err(dai->dev,
			"setting sample clock phase failed: %d\n", ret);
		return ret;
	}

	return 0;
}


static int axg_tdmif_sclk_rate(struct axg_tdmif *tdmif,
			       struct snd_pcm_hw_params *params)
{
	return tdmif->slots * tdmif->slot_width * params_rate(params);
}

static int axg_tdmif_set_sclk(struct snd_soc_dai *dai,
			      struct snd_pcm_hw_params *params)
{
	struct axg_tdmif *tdmif = snd_soc_dai_get_drvdata(dai);
	unsigned long srate = axg_tdmif_sclk_rate(tdmif, params);
	int ret;

	if (!tdmif->mclk_rate) {
		clk_set_rate(tdmif->mclk, 4 * srate);
	} else {
		if (tdmif->mclk_rate % srate) {
			dev_err(dai->dev,
				"can't derive sclk %lu from mclk %lu\n",
				srate, tdmif->mclk_rate);
			return -EINVAL;
		}
	}

	ret = clk_set_rate(tdmif->sclk, srate);
	if (ret) {
		dev_err(dai->dev, "setting bit clock failed: %d\n", ret);
		return ret;
	}

	ret = clk_set_phase(tdmif->sclk,
			    axg_tdm_sclk_invert(tdmif->fmt) ? 0 : 180);
	if (ret) {
		dev_err(dai->dev, "setting bit clock phase failed: %d\n", ret);
		return ret;
	}

	return ret;
}


static int axg_tdmif_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct axg_tdmif *tdmif = snd_soc_dai_get_drvdata(dai);

	/* These modes are not supported */
	if (fmt & (SND_SOC_DAIFMT_CBS_CFM | SND_SOC_DAIFMT_CBM_CFS)) {
		dev_err(dai->dev, "only CBS_CFS and CBM_CFM are supported\n");
		return -EINVAL;
	}

	if (!tdmif->mclk && (fmt & SND_SOC_DAIFMT_CBS_CFS)) {
		dev_err(dai->dev, "cpu clock master: mclk missing\n");
		return -ENODEV;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_RIGHT_J:
		if (tdmif->slots != 2) {
			dev_err(dai->dev, "bad slot number for format: %d\n",
				tdmif->slots);
			return -EINVAL;
		}
	}

	tdmif->fmt = fmt;
	return 0;
}

static int axg_tdmif_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct axg_tdmif *tdmif = snd_soc_dai_get_drvdata(dai);
	struct axg_tdm_stream *sdata =
		snd_soc_dai_get_dma_data(dai, substream);
	int ret;

	if (!(tdmif->fmt & SND_SOC_DAIFMT_CBS_CFS)) {
		dev_dbg(dai->dev, "clock slave - using clocks as provided\n");
		return 0;
	}

	ret = axg_tdmif_set_sclk(dai, params);
	if (ret)
		return ret;

	ret = axg_tdmif_set_lrclk(dai, params);
	if (ret)
		return ret;

	/* Save the parameter for tdmout/tdmin widgets */
	memcpy(&sdata->params, params, sizeof(*params));

	return 0;
}

static int axg_tdmif_apply_symmetry(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct axg_tdmif *tdmif = snd_soc_dai_get_drvdata(dai);
	int ret;

	if (dai->component->active) {
		ret = snd_pcm_hw_constraint_single(substream->runtime,
						   SNDRV_PCM_HW_PARAM_RATE,
						   tdmif->rate);
		if (ret) {
			dev_err(dai->dev,
				"can't set tdmif rate constraint\n");
			return ret;
		}
	}

	return 0;
}

static int axg_tdmif_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct axg_tdmif *tdmif = snd_soc_dai_get_drvdata(dai);
	struct axg_tdm_stream *sdata =
		snd_soc_dai_get_dma_data(dai, substream);
	int ret;

	dev_dbg(dai->dev, "DAI starting up now\n");

	ret = axg_tdmif_apply_symmetry(substream, dai);
	if (ret)
		return ret;

	if ((dai->id == TDMIF_PAD) &&
	    (substream->stream == SNDRV_PCM_STREAM_CAPTURE))
	        sdata->mask = tdmif->rx_mask;
	else
		/* Loopback capture uses the tx_mask */
	        sdata->mask = tdmif->tx_mask;

	/* Can't take more channels than we have slots in the mask */
	snd_pcm_hw_constraint_minmax(substream->runtime,
				     SNDRV_PCM_HW_PARAM_CHANNELS,
				     0, hweight32(sdata->mask));

	if (tdmif->slot_width)
		snd_pcm_hw_constraint_minmax(substream->runtime,
					     SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
					     8, tdmif->slot_width);
	return 0;
}

static const struct snd_soc_dai_ops axg_tdmif_ops = {
	.set_sysclk	= axg_tdmif_set_sysclk,
	.set_tdm_slot	= axg_tdmif_set_tdm_slot,
	.set_fmt	= axg_tdmif_set_fmt,
	.hw_params	= axg_tdmif_hw_params,
	.startup	= axg_tdmif_startup,
};

int axg_tdmif_remove_dai(struct snd_soc_dai *dai)
{
	if (dai->capture_dma_data)
	        kfree(dai->capture_dma_data);

	if (dai->playback_dma_data)
	        kfree(dai->playback_dma_data);

	return 0;
}

int axg_tdmif_probe_dai(struct snd_soc_dai *dai)
{
	if (dai->driver->capture.channels_min) {
		dai->capture_dma_data =
			kzalloc(sizeof(struct axg_tdm_stream), GFP_KERNEL);
		if (!dai->capture_dma_data)
			return -ENOMEM;
	}

	if (dai->driver->playback.channels_min) {
		dai->playback_dma_data =
			kzalloc(sizeof(struct axg_tdm_stream), GFP_KERNEL);
		if (!dai->playback_dma_data) {
			axg_tdmif_remove_dai(dai);
			return -ENOMEM;
		}
	}

	return 0;
}


/* TDM Backend DAIs */
static struct snd_soc_dai_driver axg_tdmif_dai_drv[] = {
	[TDMIF_PAD] = {
		.name = "Pad",
		.playback = {
			.stream_name	= "Playback",
			.channels_min	= 1,
			.channels_max	= AXG_TDM_CHANNEL_MAX,
			.rates		= AXG_TDM_RATES,
			.formats	= AXG_TDM_FORMATS,
		},
		.capture = {
			.stream_name	= "Capture",
			.channels_min	= 1,
			.channels_max	= AXG_TDM_CHANNEL_MAX,
			.rates		= AXG_TDM_RATES,
			.formats	= AXG_TDM_FORMATS,
		},
		.id = TDMIF_PAD,
		.ops = &axg_tdmif_ops,
		.probe = axg_tdmif_probe_dai,
		.remove = axg_tdmif_remove_dai,
	},
	[TDMIF_LOOPBACK] = {
		.name = "Loopback",
		.capture = {
			.stream_name	= "Loopback",
			.channels_min	= 1,
			.channels_max	= AXG_TDM_CHANNEL_MAX,
			.rates		= AXG_TDM_RATES,
			.formats	= AXG_TDM_FORMATS,
		},
		.id = TDMIF_LOOPBACK,
		.ops = &axg_tdmif_ops,
		.probe = axg_tdmif_probe_dai,
		.remove = axg_tdmif_remove_dai,
	},
};

static int axg_tdmif_set_bias_level(struct snd_soc_component *component,
				    enum snd_soc_bias_level level)
{
	struct axg_tdmif *tdmif = snd_soc_component_get_drvdata(component);
	enum snd_soc_bias_level now =
		snd_soc_component_get_bias_level(component);
	int ret = 0;

	dev_dbg(component->dev, "%s: got bias level %d\n", __func__, level);

	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		if (now == SND_SOC_BIAS_STANDBY)
			ret = clk_prepare_enable(tdmif->mclk);
		break;

	case SND_SOC_BIAS_STANDBY:
		if (now == SND_SOC_BIAS_PREPARE)
			clk_disable_unprepare(tdmif->mclk);
		break;

	case SND_SOC_BIAS_OFF:
	case SND_SOC_BIAS_ON:
		break;
	}

	return ret;
}

static int axg_tdmif_component_probe(struct snd_soc_component *component)
{
	struct axg_tdmif *tdmif = snd_soc_component_get_drvdata(component);

	/*
	 * Init masks and slot:
	 * This will be overwritten if DT provides something
	 */
	tdmif->slots = 2;
	tdmif->tx_mask = 0x3;
	tdmif->rx_mask = 0x3;

	/* When we have something better in place, we could remove this */
	tdmif->slot_width = 32;

	return 0;
}

static const struct snd_soc_component_driver axg_tdmif_component_drv = {
	.probe		= axg_tdmif_component_probe,
	.set_bias_level	= axg_tdmif_set_bias_level,
};

static const struct of_device_id axg_tdmif_of_match[] = {
	{ .compatible = "amlogic,meson-axg-tdmif", },
	{}
};
MODULE_DEVICE_TABLE(of, axg_tdmif_of_match);

static int axg_tdmif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct axg_tdmif *tdmif;
	int ret;

	tdmif = devm_kzalloc(dev, sizeof(*tdmif), GFP_KERNEL);
	if (!tdmif)
		return -ENOMEM;
	platform_set_drvdata(pdev, tdmif);

	tdmif->sclk = devm_clk_get(dev, "sclk");
	if(IS_ERR(tdmif->sclk)) {
		ret = PTR_ERR(tdmif->sclk);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get sclk: %d\n", ret);
		return ret;
	}

	tdmif->lrclk = devm_clk_get(dev, "lrclk");
	if(IS_ERR(tdmif->lrclk)) {
		ret = PTR_ERR(tdmif->lrclk);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get lrclk: %d\n", ret);
		return ret;
	}

	/*
	 * mclk maybe be missing when the cpu dai is in slave mode and
	 * the codec does not require it to provide a master clock.
	 * At this point, ignore the error if mclk is missing. We'll
	 * throw an error if the cpu dai is master and mclk is missing
	 */
	tdmif->mclk = devm_clk_get(dev, "mclk");
	if(IS_ERR(tdmif->mclk)) {
		ret = PTR_ERR(tdmif->mclk);
		if (ret == -ENOENT) {
			tdmif->mclk = NULL;
		} else {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to get mclk: %d\n", ret);
			return ret;
		}
	}

	return devm_snd_soc_register_component(dev, &axg_tdmif_component_drv,
					       axg_tdmif_dai_drv,
					       ARRAY_SIZE(axg_tdmif_dai_drv));
}

static struct platform_driver axg_tdmif_pdrv = {
	.probe = axg_tdmif_probe,
	.driver = {
		.name = "meson-axg-tdmif",
		.of_match_table = axg_tdmif_of_match,
	},
};
module_platform_driver(axg_tdmif_pdrv);

MODULE_DESCRIPTION("Amlogic A113 TDM Pin driver");
MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_LICENSE("GPL v2");
