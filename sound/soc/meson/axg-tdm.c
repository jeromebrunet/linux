// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
// Copyright (c) 2018 BayLibre, SAS.
// Author: Jerome Brunet <jbrunet@baylibre.com>

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include "axg-tdm.h"

#define TDM_CTRL0	0x00
#define TDM_WORK_EN	BIT(31)
#define TDM_RST_OUT	BIT(29)
#define TDM_RST_IN	BIT(28)

enum {
	TDM_SCLK,
	TDM_LRCLK,
	TDM_PCLK,
	TDM_SCLK_SEL,
	TDM_LRCLK_SEL,
	TDM_CLK_NUM,
};

const static char * const clk_names[] = {
	[TDM_SCLK]	= "sclk",
	[TDM_LRCLK]	= "lrclk",
	[TDM_PCLK]	= "pclk",
	[TDM_SCLK_SEL]	= "sclk_sel",
	[TDM_LRCLK_SEL]	= "lrclk_sel",
};

struct axg_tdm {
	struct regmap *map;
	struct clk_bulk_data clks[TDM_CLK_NUM];
};

struct axg_tdm_match_data {
	const struct snd_soc_component_driver *component_drv;
	const struct regmap_config *regmap_cfg;
};

static void axg_tdm_afifo_reset(struct axg_tdm *tdm)
{
	/* Apply reset */
	regmap_update_bits(tdm->map, TDM_CTRL0, TDM_RST_OUT | TDM_RST_IN, 0);

	/* RST IN must be set to 1 after RST OUT */
	regmap_update_bits(tdm->map, TDM_CTRL0, TDM_RST_OUT, TDM_RST_OUT);
	regmap_update_bits(tdm->map, TDM_CTRL0, TDM_RST_IN, TDM_RST_IN);
}

static int axg_tdm_enable(struct axg_tdm *tdm)
{
	int ret;

	axg_tdm_afifo_reset(tdm);

	ret = clk_bulk_prepare_enable(2, tdm->clks);
	if (ret)
		return ret;

	regmap_update_bits(tdm->map, TDM_CTRL0, TDM_WORK_EN, TDM_WORK_EN);
	return 0;
}

static void axg_tdm_disable(struct axg_tdm *tdm)
{
        regmap_update_bits(tdm->map, TDM_CTRL0, TDM_WORK_EN, 0);
        clk_bulk_disable_unprepare(2, tdm->clks);
}

/* FIXME: REPLACE WITH INIT VAL */
static void axg_tdm_lane_mapping_reset(struct axg_tdm *tdm, unsigned int reg)
{
	/* On reset, send each lane (channel) to its corresponding output */
	regmap_write(tdm->map, reg, 0x76543210);
}

static int axg_tdm_clk_set_parent(struct axg_tdm *tdm, struct axg_tdmif *tdmif)
{
	int ret;

	ret = clk_set_parent(tdm->clks[TDM_SCLK_SEL].clk, tdmif->sclk);
	if (ret)
		return ret;

	return clk_set_parent(tdm->clks[TDM_LRCLK_SEL].clk, tdmif->lrclk);
}

static int axg_tdm_set_bias_level(struct snd_soc_component *component,
				  enum snd_soc_bias_level level)
{
	struct axg_tdm *tdm = snd_soc_component_get_drvdata(component);
	enum snd_soc_bias_level now =
		snd_soc_component_get_bias_level(component);
	int ret = 0;

	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		if (now == SND_SOC_BIAS_STANDBY)
			ret = clk_prepare_enable(tdm->clks[TDM_PCLK].clk);
		break;

	case SND_SOC_BIAS_STANDBY:
		if (now == SND_SOC_BIAS_PREPARE)
			clk_disable_unprepare(tdm->clks[TDM_PCLK].clk);
		break;

	case SND_SOC_BIAS_OFF:
	case SND_SOC_BIAS_ON:
		break;
	}

	return ret;
}

#define TDMOUT_CTRL0			0x00
#define  TDMOUT_CTRL0_BITNUM_MASK	GENMASK(4, 0)
#define  TDMOUT_CTRL0_BITNUM(x)		((x) << 0)
#define  TDMOUT_CTRL0_SLOTNUM_MASK	GENMASK(9, 5)
#define  TDMOUT_CTRL0_SLOTNUM(x)	((x) << 5)
#define  TDMOUT_CTRL0_INIT_BITNUM_MASK	GENMASK(19, 15)
#define  TDMOUT_CTRL0_INIT_BITNUM(x)	((x) << 15)
#define TDMOUT_CTRL1			0x04
#define  TDMOUT_CTRL1_TYPE_MASK		GENMASK(6, 4)
#define  TDMOUT_CTRL1_TYPE(x)		((x) << 4)
#define  TDMOUT_CTRL1_MSB_POS_MASK	GENMASK(12, 8)
#define  TDMOUT_CTRL1_MSB_POS(x)	((x) << 8)
#define  TDMOUT_CTRL1_SEL_SHIFT		24
#define  TDMOUT_CTRL1_REV_WS		BIT(28)
#define TDMOUT_SWAP			0x08
#define  TDMOUT_SWAP_LANE0_SHIFT	0
#define  TDMOUT_SWAP_LANE1_SHIFT	8
#define  TDMOUT_SWAP_LANE2_SHIFT	16
#define  TDMOUT_SWAP_LANE3_SHIFT	24
#define  TDMOUT_SWAP_LANE_MASK		0xff
#define TDMOUT_MASK0		0x0c
#define TDMOUT_MASK1		0x10
#define TDMOUT_MASK2		0x14
#define TDMOUT_MASK3		0x18
#define TDMOUT_STAT		0x1c
#define TDMOUT_GAIN0		0x20
#define TDMOUT_GAIN1		0x24
#define TDMOUT_MUTE_VAL		0x28
#define TDMOUT_MUTE0		0x2c
#define TDMOUT_MUTE1		0x30
#define TDMOUT_MUTE2		0x34
#define TDMOUT_MUTE3		0x38
#define TDMOUT_MASK_VAL		0x3c

static const struct regmap_config axg_tdmout_regmap_cfg = {
	.reg_bits 	= 32,
	.val_bits 	= 32,
	.reg_stride 	= 4,
	.max_register	= TDMOUT_MASK_VAL,
};

static const char * const tdmout_sel_texts[] = {
	"IN 0", "IN 1", "IN 2",
};

static SOC_ENUM_SINGLE_DECL(axg_tdmout_sel_enum, TDMOUT_CTRL1,
			    TDMOUT_CTRL1_SEL_SHIFT, tdmout_sel_texts);

static const struct snd_kcontrol_new axg_tdmout_in_mux =
	SOC_DAPM_ENUM("Input Source", axg_tdmout_sel_enum);

static struct snd_soc_dai *
axg_tdmout_get_be(struct snd_soc_dapm_widget *w)
{
	struct snd_soc_dapm_path *p = NULL;
	struct snd_soc_dai *be;

        snd_soc_dapm_widget_for_each_sink_path(w, p) {
		if (!p->connect)
			continue;

		if(p->sink->id == snd_soc_dapm_dai_in)
			return (struct snd_soc_dai *)p->sink->priv;

		be = axg_tdmout_get_be(p->sink);
		if (be)
			return be;
	}

	return NULL;
}

static int axg_tdmout_clk_setup(struct axg_tdm *tdm, struct axg_tdmif *tdmif)
{
	int ret;

	ret = axg_tdm_clk_set_parent(tdm, tdmif);
	if (ret)
		return ret;

	regmap_update_bits(tdm->map, TDMOUT_CTRL1, TDMOUT_CTRL1_REV_WS,
			   axg_tdm_lrclk_invert(tdmif->fmt) ?
			   TDMOUT_CTRL1_REV_WS : 0);

	/* TDMOUT changes the data on rising edges */
        return clk_set_phase(tdm->clks[TDM_SCLK].clk,
			      axg_tdm_sclk_invert(tdmif->fmt) ? 0 : 180);
}

static int axg_tdmout_set_skew(struct axg_tdm *tdm, struct axg_tdmif *tdmif)
{
	unsigned int skew;

	switch (tdmif->fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_DSP_A:
		skew = 1;
		break;

	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_DSP_B:
		skew = 2;
		break;

	default:
		return -EINVAL;
	}

	regmap_update_bits(tdm->map, TDMOUT_CTRL0,
			   TDMOUT_CTRL0_INIT_BITNUM_MASK,
			   TDMOUT_CTRL0_INIT_BITNUM(skew));

	return 0;
}

static int axg_tdmout_set_slot(struct axg_tdm *tdm,
			       struct axg_tdmif *tdmif)
{
	/* set slot width  */
	regmap_update_bits(tdm->map, TDMOUT_CTRL0, TDMOUT_CTRL0_BITNUM_MASK,
			   TDMOUT_CTRL0_BITNUM(tdmif->slot_width - 1));

	/* set slot number */
	regmap_update_bits(tdm->map, TDMOUT_CTRL0, TDMOUT_CTRL0_SLOTNUM_MASK,
			   TDMOUT_CTRL0_SLOTNUM(tdmif->slots - 1));

	return 0;
}

static int axg_tdmout_set_format(struct axg_tdm *tdm,
				 struct axg_tdmif *tdmif,
				 struct axg_tdm_stream *sdata)
{
	snd_pcm_format_t format = params_format(&sdata->params);
	unsigned int val;

	/* DDR data are arranged in 64bits data samples */
	switch (snd_pcm_format_physical_width(format)) {
	case 8:
		val = 0; /* 8 samples of 8 bits */
		break;
	case 16:
		val = 2; /* 4 samples of 16 bits - right justified */
		break;
	case 32:
		val = 4; /* 2 samples of 32 bits - right justified */
		break;
	default:
		return -EINVAL;
	}

	/* set memory layout type */
	regmap_update_bits(tdm->map, TDMOUT_CTRL1, TDMOUT_CTRL1_TYPE_MASK,
			   TDMOUT_CTRL1_TYPE(val));

	val = snd_pcm_format_width(format) - 1;
	/* the sample format width of each sample */
	regmap_update_bits(tdm->map, TDMOUT_CTRL1, TDMOUT_CTRL1_MSB_POS_MASK,
			   TDMOUT_CTRL1_MSB_POS(val));

	return 0;
}

static int axg_tdmout_set_channel_mask(struct axg_tdm *tdm,
				       struct axg_tdmif *tdmif,
				       struct axg_tdm_stream *sdata)
{
	/* FIXME --- FOR TESTING ONLY ON S400 LINEOUT */
	regmap_write(tdm->map, TDMOUT_MASK0, 0x0);
	regmap_write(tdm->map, TDMOUT_MASK1, 0x0);
	regmap_write(tdm->map, TDMOUT_MASK2, 0x3);
	regmap_write(tdm->map, TDMOUT_MASK3, 0x0);

	return 0;
}

static int axg_tdmout_prepare(struct snd_soc_dapm_widget *w,
			      struct axg_tdm *tdm)
{
	struct snd_soc_dai *be = axg_tdmout_get_be(w);
	struct axg_tdm_stream *sdata;
	struct axg_tdmif *tdmif;
	int ret;

	if (!be)
		return -EIO;

	tdmif = snd_soc_dai_get_drvdata(be);
	sdata = (struct axg_tdm_stream *)be->playback_dma_data;

        ret = axg_tdmout_clk_setup(tdm, tdmif);
	if (ret)
		return ret;

	ret = axg_tdmout_set_skew(tdm, tdmif);
	if (ret)
		return ret;

	ret = axg_tdmout_set_slot(tdm, tdmif);
	if (ret)
		return ret;

	ret = axg_tdmout_set_format(tdm, tdmif, sdata);
	if (ret)
		return ret;

	return axg_tdmout_set_channel_mask(tdm, tdmif, sdata);
}

static int axg_tdmout_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *control, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	struct axg_tdm *tdm = snd_soc_component_get_drvdata(c);
	int ret = 0;

	dev_dbg(c->dev, "%s: got dapm event %d\n", __func__, event);

	switch(event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = axg_tdmout_prepare(w, tdm);
		break;

	case SND_SOC_DAPM_POST_PMU:
		ret = axg_tdm_enable(tdm);
		break;

	case SND_SOC_DAPM_PRE_PMD:
	        axg_tdm_disable(tdm);
		break;

	case SND_SOC_DAPM_POST_PMD:
	case SND_SOC_DAPM_PRE_REG:
	case SND_SOC_DAPM_POST_REG:
		/* Not sure what do to with this yet */
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static const char * const axg_tdmout_swap_txt[] = {
	"LANE 0", "LANE 1", "LANE 2", "LANE 3"
};

static const unsigned int axg_tdmout_swap_val[] = {
	0x10, 0x32, 0x54, 0x76
};

static const struct soc_enum axg_tdmout_lane_swap[] = {
	SOC_VALUE_ENUM_SINGLE(TDMOUT_SWAP, TDMOUT_SWAP_LANE0_SHIFT,
			      TDMOUT_SWAP_LANE_MASK,
			      ARRAY_SIZE(axg_tdmout_swap_txt),
			      axg_tdmout_swap_txt, axg_tdmout_swap_val),
	SOC_VALUE_ENUM_SINGLE(TDMOUT_SWAP, TDMOUT_SWAP_LANE1_SHIFT,
			      TDMOUT_SWAP_LANE_MASK,
			      ARRAY_SIZE(axg_tdmout_swap_txt),
			      axg_tdmout_swap_txt, axg_tdmout_swap_val),
	SOC_VALUE_ENUM_SINGLE(TDMOUT_SWAP, TDMOUT_SWAP_LANE2_SHIFT,
			      TDMOUT_SWAP_LANE_MASK,
			      ARRAY_SIZE(axg_tdmout_swap_txt),
			      axg_tdmout_swap_txt, axg_tdmout_swap_val),
	SOC_VALUE_ENUM_SINGLE(TDMOUT_SWAP, TDMOUT_SWAP_LANE3_SHIFT,
			      TDMOUT_SWAP_LANE_MASK,
			      ARRAY_SIZE(axg_tdmout_swap_txt),
			      axg_tdmout_swap_txt, axg_tdmout_swap_val),
};

static const struct snd_kcontrol_new axg_tdmout_controls[] = {
	SOC_ENUM("DOUT 0 SRC", axg_tdmout_lane_swap[0]),
	SOC_ENUM("DOUT 1 SRC", axg_tdmout_lane_swap[1]),
	SOC_ENUM("DOUT 2 SRC", axg_tdmout_lane_swap[2]),
	SOC_ENUM("DOUT 3 SRC", axg_tdmout_lane_swap[3]),
};

static const struct snd_soc_dapm_widget axg_tdmout_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("IN 0", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 1", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 2", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_MUX("SRC SEL", SND_SOC_NOPM, 0, 0, &axg_tdmout_in_mux),
	SND_SOC_DAPM_PGA_E("ENC", SND_SOC_NOPM, 0, 0, NULL, 0, axg_tdmout_event,
			   (SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
			    SND_SOC_DAPM_PRE_REG | SND_SOC_DAPM_POST_REG)),
	SND_SOC_DAPM_AIF_OUT("OUT", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route axg_tdmout_dapm_routes[] = {
	{ "SRC SEL", "IN 0", "IN 0" },
	{ "SRC SEL", "IN 1", "IN 1" },
	{ "SRC SEL", "IN 2", "IN 2" },
	{ "ENC", NULL, "SRC SEL" },
	{ "OUT", NULL, "ENC" },
};

static int axg_tdmout_probe_component(struct snd_soc_component *c)
{
	struct axg_tdm *tdm = snd_soc_component_get_drvdata(c);

	axg_tdm_lane_mapping_reset(tdm, TDMOUT_SWAP);

	return 0;
}

static const struct snd_soc_component_driver axg_tdmout_component_drv = {
	.probe			= axg_tdmout_probe_component,
	.dapm_widgets		= axg_tdmout_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(axg_tdmout_dapm_widgets),
	.dapm_routes		= axg_tdmout_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(axg_tdmout_dapm_routes),
	.controls		= axg_tdmout_controls,
	.num_controls		= ARRAY_SIZE(axg_tdmout_controls),
	.set_bias_level		= axg_tdm_set_bias_level,
};

static const struct axg_tdm_match_data axg_tdmout_match_data = {
	.component_drv	= &axg_tdmout_component_drv,
	.regmap_cfg	= &axg_tdmout_regmap_cfg,
};

#define TDMIN_CTRL			0x00
#define  TDMIN_CTRL_I2S_MODE		BIT(30)
#define  TDMIN_CTRL_REV_WS		BIT(25)
#define  TDMIN_CTRL_SEL_SHIFT		20
#define  TDMIN_CTRL_IN_BIT_SKEW_MASK	GENMASK(18, 16)
#define  TDMIN_CTRL_IN_BIT_SKEW(x)	((x) << 16)
#define  TDMIN_CTRL_MSB_FIRST		BIT(5)
#define  TDMIN_CTRL_BITNUM_MASK	GENMASK(4, 0)
#define  TDMIN_CTRL_BITNUM(x)		((x) << 0)
#define TDMIN_SWAP			0x04
#define  TDMIN_SWAP_LANE0_SHIFT		0
#define  TDMIN_SWAP_LANE1_SHIFT		8
#define  TDMIN_SWAP_LANE2_SHIFT		16
#define  TDMIN_SWAP_LANE3_SHIFT		24
#define  TDMIN_SWAP_LANE_MASK		0xff
#define TDMIN_MASK0			0x08
#define TDMIN_MASK1			0x0c
#define TDMIN_MASK2			0x10
#define TDMIN_MASK3			0x14
#define TDMIN_STAT			0x18
#define TDMIN_MUTE_VAL			0x1c
#define TDMIN_MUTE0			0x20
#define TDMIN_MUTE1			0x24
#define TDMIN_MUTE2			0x28
#define TDMIN_MUTE3			0x2c

static const struct regmap_config axg_tdmin_regmap_cfg = {
	.reg_bits 	= 32,
	.val_bits 	= 32,
	.reg_stride 	= 4,
	.max_register	= TDMIN_MUTE3,
};

static const char * const axg_tdmin_sel_texts[] = {
	"IN 0", "IN 1", "IN 2", "IN 3", "IN 4", "IN 5",
};

/* Change to special mux control to reset dapm */
static SOC_ENUM_SINGLE_DECL(axg_tdmin_sel_enum, TDMIN_CTRL,
			    TDMIN_CTRL_SEL_SHIFT, axg_tdmin_sel_texts);

static const struct snd_kcontrol_new axg_tdmin_in_mux =
	SOC_DAPM_ENUM("Input Source", axg_tdmin_sel_enum);

static struct snd_soc_dai *
axg_tdmin_get_be(struct snd_soc_dapm_widget *w)
{
	struct snd_soc_dapm_path *p = NULL;
	struct snd_soc_dai *be;

        snd_soc_dapm_widget_for_each_source_path(w, p) {
		if (!p->connect)
			continue;

		if(p->source->id == snd_soc_dapm_dai_out)
			return (struct snd_soc_dai *)p->source->priv;

		be = axg_tdmin_get_be(p->source);
		if (be)
			return be;
	}

	return NULL;
}

static int axg_tdmin_clk_setup(struct axg_tdm *tdm, struct axg_tdmif *tdmif)
{
	int ret;

	ret = axg_tdm_clk_set_parent(tdm, tdmif);
	if (ret)
		return ret;

	regmap_update_bits(tdm->map, TDMIN_CTRL, TDMIN_CTRL_REV_WS,
			   axg_tdm_lrclk_invert(tdmif->fmt) ?
			   TDMIN_CTRL_REV_WS : 0);

	/*
	 * TDMIN samples data on the rising edge,
	 * so change them on falling edge
	 */
        return clk_set_phase(tdm->clks[TDM_SCLK].clk,
			     axg_tdm_sclk_invert(tdmif->fmt) ? 180 : 0);
}

static int axg_tdmin_set_skew(struct axg_tdm *tdm, struct axg_tdmif *tdmif)
{
	unsigned int skew;

	switch (tdmif->fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_DSP_A:
		skew = 3;
		break;

	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_DSP_B:
		skew = 2;
		break;

	default:
		return -EINVAL;
	}

	regmap_update_bits(tdm->map, TDMIN_CTRL, TDMIN_CTRL_IN_BIT_SKEW_MASK,
			   TDMIN_CTRL_IN_BIT_SKEW(skew));

	return 0;
}

static int axg_tdmin_set_mode(struct axg_tdm *tdm, struct axg_tdmif *tdmif)
{
	bool i2s_mode;

	switch (tdmif->fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_RIGHT_J:
		i2s_mode = true;
		break;

	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		i2s_mode = false;
		break;

	default:
		return -EINVAL;
	}

	regmap_update_bits(tdm->map, TDMIN_CTRL, TDMIN_CTRL_I2S_MODE,
			   i2s_mode ? TDMIN_CTRL_I2S_MODE : 0);

	return 0;
}

static int axg_tdmin_set_slot(struct axg_tdm *tdm,
			      struct axg_tdmif *tdmif)
{
	/* set slot width  */
	regmap_update_bits(tdm->map, TDMIN_CTRL, TDMIN_CTRL_BITNUM_MASK,
			   TDMIN_CTRL_BITNUM(tdmif->slot_width - 1));

	return 0;
}

static int axg_tdmin_set_format(struct axg_tdm *tdm,
				 struct axg_tdmif *tdmif,
				 struct axg_tdm_stream *sdata)
{
	/*
	 * This place the first bit received at bit 31 in the FIFO.
	 * It is the only mode supported at the moment. Whether we
	 * need to support LSB first remains to defined
	 * Maybe this is something FRDDR should be able to query ?
	 */
	regmap_update_bits(tdm->map, TDMIN_CTRL, TDMIN_CTRL_MSB_FIRST,
			   TDMIN_CTRL_MSB_FIRST);

	return 0;
}

static int axg_tdmin_set_channel_mask(struct axg_tdm *tdm,
				       struct axg_tdmif *tdmif,
				       struct axg_tdm_stream *sdata)
{
	/* FIXME --- FOR TESTING ONLY ON S400 LINEIN */
	regmap_write(tdm->map, TDMIN_MASK0, 0x0);
	regmap_write(tdm->map, TDMIN_MASK1, 0x3);
	regmap_write(tdm->map, TDMIN_MASK2, 0x0);
	regmap_write(tdm->map, TDMIN_MASK3, 0x0);

	return 0;
}

static int axg_tdmin_prepare(struct snd_soc_dapm_widget *w,
			     struct axg_tdm *tdm)
{
	struct snd_soc_dai *be = axg_tdmin_get_be(w);
	struct axg_tdmif *tdmif;
	struct axg_tdm_stream *sdata;
	int ret;

	if (!be)
		return -EIO;

	tdmif = snd_soc_dai_get_drvdata(be);
	sdata = (struct axg_tdm_stream *)be->capture_dma_data;

        ret = axg_tdmin_clk_setup(tdm, tdmif);
	if (ret)
		return ret;

	ret = axg_tdmin_set_skew(tdm, tdmif);
	if (ret)
		return ret;

        ret = axg_tdmin_set_mode(tdm, tdmif);
	if (ret)
		return ret;

	ret = axg_tdmin_set_slot(tdm, tdmif);
	if (ret)
		return ret;

	axg_tdmin_set_format(tdm, tdmif, sdata);
	if (ret)
		return ret;

	return axg_tdmin_set_channel_mask(tdm, tdmif, sdata);
}

static int axg_tdmin_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *control, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	struct axg_tdm *tdm = snd_soc_component_get_drvdata(c);
	int ret = 0;

	switch(event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = axg_tdmin_prepare(w, tdm);
		break;

	case SND_SOC_DAPM_POST_PMU:
		ret = axg_tdm_enable(tdm);
		break;

	case SND_SOC_DAPM_PRE_PMD:
	        axg_tdm_disable(tdm);
		break;

	case SND_SOC_DAPM_POST_PMD:
	case SND_SOC_DAPM_PRE_REG:
	case SND_SOC_DAPM_POST_REG:
		/* Not sure what do to with this yet */
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static const char * const axg_tdmin_swap_txt[] = {
	"DIN 0", "DIN 1", "DIN 2", "DIN 3"
};

static const unsigned int axg_tdmin_swap_val[] = {
	0x10, 0x32, 0x54, 0x76
};

const struct soc_enum axg_tdmin_din_swap[] = {
	SOC_VALUE_ENUM_SINGLE(TDMIN_SWAP, TDMIN_SWAP_LANE0_SHIFT,
			      TDMIN_SWAP_LANE_MASK,
			      ARRAY_SIZE(axg_tdmin_swap_txt),
			      axg_tdmin_swap_txt, axg_tdmin_swap_val),
	SOC_VALUE_ENUM_SINGLE(TDMIN_SWAP, TDMIN_SWAP_LANE1_SHIFT,
			      TDMIN_SWAP_LANE_MASK,
			      ARRAY_SIZE(axg_tdmin_swap_txt),
			      axg_tdmin_swap_txt, axg_tdmin_swap_val),
	SOC_VALUE_ENUM_SINGLE(TDMIN_SWAP, TDMIN_SWAP_LANE2_SHIFT,
			      TDMIN_SWAP_LANE_MASK,
			      ARRAY_SIZE(axg_tdmin_swap_txt),
			      axg_tdmin_swap_txt, axg_tdmin_swap_val),
	SOC_VALUE_ENUM_SINGLE(TDMIN_SWAP, TDMIN_SWAP_LANE3_SHIFT,
			      TDMIN_SWAP_LANE_MASK,
			      ARRAY_SIZE(axg_tdmin_swap_txt),
			      axg_tdmin_swap_txt, axg_tdmin_swap_val),
};

static const struct snd_kcontrol_new axg_tdmin_controls[] = {
	SOC_ENUM("LANE 0 SRC", axg_tdmin_din_swap[0]),
	SOC_ENUM("LANE 1 SRC", axg_tdmin_din_swap[1]),
	SOC_ENUM("LANE 2 SRC", axg_tdmin_din_swap[2]),
	SOC_ENUM("LANE 3 SRC", axg_tdmin_din_swap[3]),
};

static const struct snd_soc_dapm_widget axg_tdmin_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("IN 0", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 1", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 2", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 3", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 4", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 5", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_MUX("SRC SEL", SND_SOC_NOPM, 0, 0, &axg_tdmin_in_mux),
	SND_SOC_DAPM_PGA_E("DEC", SND_SOC_NOPM, 0, 0, NULL, 0, axg_tdmin_event,
			   (SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
			    SND_SOC_DAPM_PRE_REG | SND_SOC_DAPM_POST_REG)),
	SND_SOC_DAPM_AIF_OUT("OUT", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route axg_tdmin_dapm_routes[] = {
	{ "SRC SEL", "IN 0", "IN 0" },
	{ "SRC SEL", "IN 1", "IN 1" },
	{ "SRC SEL", "IN 2", "IN 2" },
	{ "SRC SEL", "IN 3", "IN 3" },
	{ "SRC SEL", "IN 4", "IN 4" },
	{ "SRC SEL", "IN 5", "IN 5" },
	{ "DEC", NULL, "SRC SEL" },
	{ "OUT", NULL, "DEC" },
};

static int axg_tdmin_probe_component(struct snd_soc_component *c)
{
	struct axg_tdm *tdm = snd_soc_component_get_drvdata(c);

	axg_tdm_lane_mapping_reset(tdm, TDMIN_SWAP);

	return 0;
}

static const struct snd_soc_component_driver axg_tdmin_component_drv = {
	.probe			= axg_tdmin_probe_component,
	.dapm_widgets		= axg_tdmin_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(axg_tdmin_dapm_widgets),
	.dapm_routes		= axg_tdmin_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(axg_tdmin_dapm_routes),
	.controls		= axg_tdmin_controls,
	.num_controls		= ARRAY_SIZE(axg_tdmin_controls),
	.set_bias_level		= axg_tdm_set_bias_level,
};

static const struct axg_tdm_match_data axg_tdmin_match_data = {
	.component_drv	= &axg_tdmin_component_drv,
	.regmap_cfg	= &axg_tdmin_regmap_cfg,
};

static const struct of_device_id axg_tdm_of_match[] = {
	{
		.compatible = "amlogic,meson-axg-tdmout",
		.data = &axg_tdmout_match_data,
	}, {
		.compatible = "amlogic,meson-axg-tdmin",
		.data = &axg_tdmin_match_data,
	}, {}
};
MODULE_DEVICE_TABLE(of, axg_tdm_of_match);

static int axg_tdm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct axg_tdm_match_data *data;
	struct axg_tdm *tdm;
	struct resource *res;
	void __iomem *regs;
	int i, ret;

	data = of_device_get_match_data(dev);
	if (!data) {
		dev_err(dev, "failed to match device\n");
		return -ENODEV;
	}

	tdm = devm_kzalloc(dev, sizeof(*tdm), GFP_KERNEL);
	if (!tdm)
		return -ENOMEM;
	platform_set_drvdata(pdev, tdm);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	tdm->map = devm_regmap_init_mmio(dev, regs, data->regmap_cfg);
	if (IS_ERR(tdm->map)) {
		dev_err(dev, "failed to init regmap: %ld\n", PTR_ERR(tdm->map));
		return PTR_ERR(tdm->map);
	}

	for (i = 0; i < TDM_CLK_NUM; i++)
		tdm->clks[i].id = clk_names[i];

	ret = devm_clk_bulk_get(dev, TDM_CLK_NUM, tdm->clks);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get clocks: %d\n", ret);
		return ret;
	}

	return devm_snd_soc_register_component(dev, data->component_drv,
					      NULL, 0);
}

static struct platform_driver axg_tdm_pdrv = {
	.probe = axg_tdm_probe,
	.driver = {
		.name = "meson-axg-tdm",
		.of_match_table = axg_tdm_of_match,
	},
};
module_platform_driver(axg_tdm_pdrv);

MODULE_DESCRIPTION("Amlogic A113 TDM Encoder/Decoder driver");
MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_LICENSE("GPL v2");
