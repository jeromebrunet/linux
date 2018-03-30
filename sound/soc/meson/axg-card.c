// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
// Copyright (c) 2018 BayLibre, SAS.
// Author: Jerome Brunet <jbrunet@baylibre.com>

#include <linux/module.h>
#include <sound/soc.h>

enum {
	DAI_LINK_FE_TODDR_A,
	DAI_LINK_FE_TODDR_B,
	DAI_LINK_FE_TODDR_C,
	DAI_LINK_FE_FRDDR_A,
	DAI_LINK_FE_FRDDR_B,
	DAI_LINK_FE_FRDDR_C,
	DAI_LINK_BE_TDMIF_A_PAD,
	DAI_LINK_BE_TDMIF_A_LOOPBACK,
	DAI_LINK_BE_TDMIF_B_PAD,
	DAI_LINK_BE_TDMIF_B_LOOPBACK,
	DAI_LINK_BE_TDMIF_C_PAD,
	DAI_LINK_BE_TDMIF_C_LOOPBACK,
};

#define AXG_TODDR_ROUTES(_name)			\
	{ _name" IN 0", NULL, "TDMIN_A OUT" },	\
	{ _name" IN 1", NULL, "TDMIN_B OUT" },	\
	{ _name" IN 2", NULL, "TDMIN_C OUT" },	\
	{ _name" IN 6", NULL, "TDMIN_LB OUT" }

#define AXG_FRDDR_ROUTES(_name, _sink)			\
	{ "TDMOUT_A "_sink, NULL, _name" OUT 0" },	\
	{ "TDMOUT_B "_sink, NULL, _name" OUT 1" },	\
	{ "TDMOUT_C "_sink, NULL, _name" OUT 2" }

#define AXG_TDMIN_ROUTES(_name)				\
	{ _name" IN 0", NULL, "TDMIF_A Capture" },	\
	{ _name" IN 1", NULL, "TDMIF_B Capture" },	\
	{ _name" IN 2", NULL, "TDMIF_C Capture" },	\
	{ _name" IN 3", NULL, "TDMIF_A Loopback" },	\
	{ _name" IN 4", NULL, "TDMIF_B Loopback" },	\
	{ _name" IN 5", NULL, "TDMIF_C Loopback" }

#define AXG_TDMIN_LB_ROUTES(_name)			\
	{ _name" IN 0", NULL, "TDMIF_A Loopback" },	\
	{ _name" IN 1", NULL, "TDMIF_B Loopback" },	\
	{ _name" IN 2", NULL, "TDMIF_C Loopback" },	\
	{ _name" IN 3", NULL, "TDMIF_A Capture" },	\
	{ _name" IN 4", NULL, "TDMIF_B Capture" },	\
	{ _name" IN 5", NULL, "TDMIF_C Capture" }

static struct snd_soc_dapm_route axg_card_intercon[] = {
	/* Capture routes */
	AXG_TODDR_ROUTES("TODDR_A"),
	AXG_TODDR_ROUTES("TODDR_B"),
	AXG_TODDR_ROUTES("TODDR_C"),
	AXG_TDMIN_ROUTES("TDMIN_A"),
	AXG_TDMIN_ROUTES("TDMIN_B"),
	AXG_TDMIN_ROUTES("TDMIN_C"),
	AXG_TDMIN_LB_ROUTES("TDMIN_LB"),

	/* Playback routes */
	AXG_FRDDR_ROUTES("FRDDR_A", "IN 0"),
	AXG_FRDDR_ROUTES("FRDDR_B", "IN 1"),
	AXG_FRDDR_ROUTES("FRDDR_C", "IN 2"),
	{ "TDMIF_A Playback", NULL, "TDMOUT_A OUT" },
	{ "TDMIF_B Playback", NULL, "TDMOUT_B OUT" },
	{ "TDMIF_C Playback", NULL, "TDMOUT_C OUT" }
};

static struct snd_soc_dai_link axg_card_dai_links[] = {
	/* Frontent DAI Links */
	[DAI_LINK_FE_TODDR_A] = {
		.name = "Capture Port A",
		.cpu_name = "ff642100.audio-controller",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_capture = 1,
	},
	[DAI_LINK_FE_TODDR_B] = {
		.name = "Capture Port B",
		.cpu_name = "ff642140.audio-controller",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_capture = 1,
	},
	[DAI_LINK_FE_TODDR_C] = {
		.name = "Capture Port C",
		.cpu_name = "ff642180.audio-controller",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_capture = 1,
	},
	[DAI_LINK_FE_FRDDR_A] = {
		.name = "Playback Port A",
		.cpu_name = "ff6421c0.audio-controller",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
	},
	[DAI_LINK_FE_FRDDR_B] = {
		.name = "Playback Port B",
		.cpu_name = "ff642200.audio-controller",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
	},
	[DAI_LINK_FE_FRDDR_C] = {
		.name = "Playback Port C",
		.cpu_name = "ff642240.audio-controller",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
	},

	/* Backend DAI Links */
	[DAI_LINK_BE_TDMIF_A_PAD] = {
		.name = "TDM Interface A Pad",
		.cpu_name = "tdmif-a",
		.cpu_dai_name = "Pad",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.dai_fmt = (SND_SOC_DAIFMT_DSP_A |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS),
	},
	[DAI_LINK_BE_TDMIF_A_LOOPBACK] = {
		.name = "TDM Interface A Loopback",
		.cpu_name = "tdmif-a",
		.cpu_dai_name = "Loopback",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.dai_fmt = (SND_SOC_DAIFMT_DSP_A |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS),
	},
	[DAI_LINK_BE_TDMIF_B_PAD] = {
		.name = "TDM Interface B Pad",
		.cpu_name = "tdmif-b",
		.cpu_dai_name = "Pad",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.dai_fmt = (SND_SOC_DAIFMT_DSP_A |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS),
	},
	[DAI_LINK_BE_TDMIF_B_LOOPBACK] = {
		.name = "TDM Interface B Loopback",
		.cpu_name = "tdmif-b",
		.cpu_dai_name = "Loopback",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.dai_fmt = (SND_SOC_DAIFMT_DSP_A |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS),
	},
	[DAI_LINK_BE_TDMIF_C_PAD] = {
		.name = "TDM Interface C Pad",
		.cpu_name = "tdmif-c",
		.cpu_dai_name = "Pad",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.dai_fmt = (SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS),
	},
	[DAI_LINK_BE_TDMIF_C_LOOPBACK] = {
		.name = "TDM Interface C Loopback",
		.cpu_name = "tdmif-c",
		.cpu_dai_name = "Loopback",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.dai_fmt = (SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS),
	},
};

static struct snd_soc_aux_dev axg_card_fixed_aux[] = {
	{ .name = "ff642300.audio-controller" },
	{ .name = "ff642340.audio-controller" },
	{ .name = "ff642380.audio-controller" },
	{ .name = "ff6423c0.audio-controller" },
	{ .name = "ff642500.audio-controller" },
	{ .name = "ff642540.audio-controller" },
	{ .name = "ff642580.audio-controller" }
};

static struct snd_soc_codec_conf axg_card_codec_conf[] = {
	{
		.dev_name = "ff642100.audio-controller",
		.name_prefix = "TODDR_A",
	}, {
		.dev_name = "ff642140.audio-controller",
		.name_prefix = "TODDR_B",
	}, {
		.dev_name = "ff642180.audio-controller",
		.name_prefix = "TODDR_C",
	}, {
		.dev_name = "ff6421c0.audio-controller",
		.name_prefix = "FRDDR_A",
	}, {
		.dev_name = "ff642200.audio-controller",
		.name_prefix = "FRDDR_B",
	}, {
		.dev_name = "ff642240.audio-controller",
		.name_prefix = "FRDDR_C",
	}, {
		.dev_name = "ff642300.audio-controller",
		.name_prefix = "TDMIN_A",
	}, {
		.dev_name = "ff642340.audio-controller",
		.name_prefix = "TDMIN_B",
	}, {
		.dev_name = "ff642380.audio-controller",
		.name_prefix = "TDMIN_C",
	}, {
		.dev_name = "ff6423c0.audio-controller",
		.name_prefix = "TDMIN_LB",
	}, {
		.dev_name = "ff642500.audio-controller",
		.name_prefix = "TDMOUT_A",
	}, {
		.dev_name = "ff642540.audio-controller",
		.name_prefix = "TDMOUT_B",
	}, {
		.dev_name = "ff642580.audio-controller",
		.name_prefix = "TDMOUT_C",
	}, {
		.dev_name = "tdmif-a",
		.name_prefix =  "TDMIF_A",
	}, {
		.dev_name = "tdmif-b",
		.name_prefix =  "TDMIF_B",
	}, {
		.dev_name = "tdmif-c",
		.name_prefix =  "TDMIF_C",
	}
};

static struct snd_soc_card axg_card = {
	.name = "meson-axg",
	.owner = THIS_MODULE,
	.codec_conf = axg_card_codec_conf,
	.num_configs = ARRAY_SIZE(axg_card_codec_conf),
	.dai_link = axg_card_dai_links,
	.num_links = ARRAY_SIZE(axg_card_dai_links),
	.dapm_routes = axg_card_intercon,
	.num_dapm_routes = ARRAY_SIZE(axg_card_intercon),
	.aux_dev = axg_card_fixed_aux,
	.num_aux_devs = ARRAY_SIZE(axg_card_fixed_aux),
};

static const struct of_device_id axg_card_of_match[] = {
	{ .compatible = "amlogic,meson-axg-audio-card", },
	{}
};
MODULE_DEVICE_TABLE(of, axg_card_of_match);

static int axg_card_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
//	int ret;

	axg_card.dev = dev;

	return devm_snd_soc_register_card(dev, &axg_card);
}

static struct platform_driver axg_card_pdrv = {
	.probe = axg_card_probe,
	.driver = {
		.name = "meson-axg-audio-card",
		.of_match_table = axg_card_of_match,
	},
};
module_platform_driver(axg_card_pdrv);

MODULE_DESCRIPTION("Amlogic A113 ALSA machine driver");
MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_LICENSE("GPL v2");
