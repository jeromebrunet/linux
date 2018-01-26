// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
// Copyright (c) 2018 BayLibre, SAS.
// Author: Jerome Brunet <jbrunet@baylibre.com>

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>

/*
 * Auge based SoCs have 2 types of memory interfaces:
 * TODDRs are FIFO sending captured data to the memory
 * FRDDRs are FIFO sending memory content to the playback interfaces
 *
 * The 2 are very similar but not exactly the same.
 * MaYbE some MoRe BlAbLA
 */

#define MEMIF_CHANNEL_MAX 		32
#define MEMIF_RATES	 		(SNDRV_PCM_RATE_5512 |		\
					 SNDRV_PCM_RATE_8000_192000)
#define MEMIF_FORMATS			(SNDRV_PCM_FMTBIT_S8 |		\
					 SNDRV_PCM_FMTBIT_S16_LE |	\
					 SNDRV_PCM_FMTBIT_S24_LE |	\
					 SNDRV_PCM_FMTBIT_S32_LE)

#define MEMIF_FIFO_BURST		8
#define MEMIF_FIFO_MIN_CNT		64
#define MEMIF_FIFO_MIN_DEPTH		(MEMIF_FIFO_BURST * MEMIF_FIFO_MIN_CNT)

#define MEMIF_INT_ADDR_FINISH		BIT(0)
#define MEMIF_INT_ADDR_INT		BIT(1)
#define MEMIF_INT_COUNT_REPEAT		BIT(2)
#define MEMIF_INT_COUNT_ONCE		BIT(3)
#define MEMIF_INT_FIFO_ZERO		BIT(4)
#define MEMIF_INT_FIFO_DEPTH		BIT(5)
#define MEMIF_INT_MASK			GENMASK(7, 0)

#define MEMIF_CTRL0			0x00
#define  CTRL0_DMA_EN			BIT(31)
#define  CTRL0_FRDDR_PP_MODE		BIT(30)
#define  CTRL0_TODDR_SEL_RESAMPLE	BIT(30)
#define  CTRL0_TODDR_EXT_SIGNED		BIT(29)
#define  CTRL0_TODDR_PP_MODE		BIT(28)
#define  CTRL0_INT_EN(x)		((x) << 16)
#define  CTRL0_TODDR_TYPE_MASK		GENMASK(15, 13)
#define  CTRL0_TODDR_TYPE(x)		((x) << 13)
#define  CTRL0_TODDR_MSB_POS_MASK	GENMASK(12, 8)
#define  CTRL0_TODDR_MSB_POS(x)		((x) << 8)
#define  CTRL0_TODDR_LSB_POS_MASK	GENMASK(7, 3)
#define  CTRL0_TODDR_LSB_POS(x)		((x) << 3)
#define  CTRL0_SEL_MASK			GENMASK(2, 0)
#define  CTRL0_SEL_SHIFT		0
#define MEMIF_CTRL1			0x04
#define  CTRL1_INT_CLR(x)		((x) << 0)
#define  CTRL1_STATUS2_SEL_MASK		GENMASK(11, 8)
#define  CTRL1_STATUS2_SEL(x)		((x) << 8)
#define   STATUS2_SEL_DDR_READ		0
#define  CTRL1_THRESHOLD_MASK		GENMASK(23, 16)
#define  CTRL1_THRESHOLD(x)		((x) << 16)
#define  CTRL1_FRDDR_DEPTH_MASK		GENMASK(31, 24)
#define  CTRL1_FRDDR_DEPTH(x)		((x) << 24)
#define MEMIF_START_ADDR		0x08
#define MEMIF_FINISH_ADDR		0x0c
#define MEMIF_INT_ADDR			0x10
#define MEMIF_STATUS1			0x14
#define  STATUS1_INT_STS(x)		((x) << 0)
#define MEMIF_STATUS2			0x18

struct axg_memif {
	struct regmap *map;
	struct clk *pclk;
	struct reset_control *arb;
	int irq;
};

static struct snd_pcm_hardware axg_memif_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_PAUSE),

	/* Let's revisit this later on */
	.formats = MEMIF_FORMATS,
	.rate_min = 5512,
	.rate_max = 192000,
	.channels_min = 1,
	.channels_max = MEMIF_CHANNEL_MAX, /* tdm 32 ? 32 * 4 ? */
	.period_bytes_min = MEMIF_FIFO_MIN_DEPTH,
	.period_bytes_max = UINT_MAX,
	.periods_min = 2,
	.periods_max = UINT_MAX,

	/* No real justification for this */
	.buffer_bytes_max = 1 * 1024 * 1024,
};

static struct snd_soc_dai *axg_memif_dai(struct snd_pcm_substream *ss)
{
	struct snd_soc_pcm_runtime *rtd = ss->private_data;

	return rtd->cpu_dai;
}

static struct axg_memif *axg_memif_data(struct snd_pcm_substream *ss)
{
	struct snd_soc_dai *dai = axg_memif_dai(ss);

	return snd_soc_dai_get_drvdata(dai);
}

static struct device *axg_memif_dev(struct snd_pcm_substream *ss)
{
	struct snd_soc_dai *dai = axg_memif_dai(ss);

	return dai->dev;
}

static void __dma_enable(struct axg_memif *memif,  bool enable)
{
	regmap_update_bits(memif->map, MEMIF_CTRL0, CTRL0_DMA_EN,
			   enable ? CTRL0_DMA_EN : 0);
}

static int axg_memif_pcm_trigger(struct snd_pcm_substream *ss, int cmd)
{
	struct axg_memif *memif = axg_memif_data(ss);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		__dma_enable(memif, true);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
		__dma_enable(memif, false);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t axg_memif_pcm_pointer(struct snd_pcm_substream *ss)
{
	struct axg_memif *memif = axg_memif_data(ss);
	struct snd_pcm_runtime *runtime = ss->runtime;
	unsigned int addr;

	regmap_read(memif->map, MEMIF_STATUS2, &addr);

	return bytes_to_frames(runtime, addr - (unsigned int)runtime->dma_addr);
}

static int axg_memif_pcm_hw_params(struct snd_pcm_substream *ss,
				    struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct axg_memif *memif = axg_memif_data(ss);
	dma_addr_t end_ptr;
	unsigned int burst_num;
	int ret;

	ret = snd_pcm_lib_malloc_pages(ss, params_buffer_bytes(params));
	if (ret < 0)
		return ret;

	/* Setup dma memory pointers */
	end_ptr = runtime->dma_addr + runtime->dma_bytes - MEMIF_FIFO_BURST;
	regmap_write(memif->map, MEMIF_START_ADDR, runtime->dma_addr);
	regmap_write(memif->map, MEMIF_FINISH_ADDR, end_ptr);

	/* Setup interrupt periodicity */
	burst_num = params_period_bytes(params) / MEMIF_FIFO_BURST;
	regmap_write(memif->map, MEMIF_INT_ADDR, burst_num);

	/* Enable block count irq */
	regmap_update_bits(memif->map, MEMIF_CTRL0,
			   CTRL0_INT_EN(MEMIF_INT_COUNT_REPEAT),
			   CTRL0_INT_EN(MEMIF_INT_COUNT_REPEAT));

	return 0;
}

static int axg_memif_pcm_hw_free(struct snd_pcm_substream *ss)
{
	struct axg_memif *memif = axg_memif_data(ss);

	/* Disable the block count irq */
	regmap_update_bits(memif->map, MEMIF_CTRL0,
			   CTRL0_INT_EN(MEMIF_INT_COUNT_REPEAT), 0);

	return snd_pcm_lib_free_pages(ss);
}

static void axg_memif_ack_irq(struct axg_memif *memif, u8 mask)
{
	regmap_update_bits(memif->map, MEMIF_CTRL1,
			   CTRL1_INT_CLR(MEMIF_INT_MASK),
			   CTRL1_INT_CLR(mask));
	regmap_update_bits(memif->map, MEMIF_CTRL1,
			   CTRL1_INT_CLR(MEMIF_INT_MASK),
			   0);
}

static irqreturn_t axg_memif_pcm_irq_block(int irq, void *dev_id)
{
	struct snd_pcm_substream *ss = dev_id;
	struct axg_memif *memif = axg_memif_data(ss);
	unsigned int status;

	regmap_read(memif->map, MEMIF_STATUS1, &status);

	status = STATUS1_INT_STS(status) & MEMIF_INT_MASK;
	if (status & MEMIF_INT_COUNT_REPEAT)
		snd_pcm_period_elapsed(ss);
	else
		dev_dbg(axg_memif_dev(ss), "unexpected irq - STS 0x%02x\n",
			status);

	/* Ack irqs */
	axg_memif_ack_irq(memif, status);

	return !status ? IRQ_NONE : IRQ_HANDLED;
}

static int axg_memif_pcm_open(struct snd_pcm_substream *ss)
{
	struct axg_memif *memif = axg_memif_data(ss);
	struct device *dev = axg_memif_dev(ss);
	int ret;

	snd_soc_set_runtime_hwparams(ss, &axg_memif_hw);

	/*
	 * Make sure the buffer and period size are multiple of the FIFO
	 * minimum depth size
	 */
	ret = snd_pcm_hw_constraint_step(ss->runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
					 MEMIF_FIFO_MIN_DEPTH);
	if (ret)
		return ret;

	ret = snd_pcm_hw_constraint_step(ss->runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
					 MEMIF_FIFO_MIN_DEPTH);
	if (ret)
		return ret;

	ret = request_irq(memif->irq, axg_memif_pcm_irq_block, 0,
			  dev_name(dev), ss);

	/* Enable pclk to access registers and clock the memif ip */
	ret = clk_prepare_enable(memif->pclk);
	if (ret)
		return ret;

	/* Setup status2 so it reports the memory pointer */
	regmap_update_bits(memif->map, MEMIF_CTRL1,
			   CTRL1_STATUS2_SEL_MASK,
			   CTRL1_STATUS2_SEL(STATUS2_SEL_DDR_READ));

	/* Make sure the dma is initially disabled */
	__dma_enable(memif, false);

	/* Disable irqs until params are ready */
	regmap_update_bits(memif->map, MEMIF_CTRL0,
			   CTRL0_INT_EN(MEMIF_INT_MASK), 0);

	/* Clear any pending interrupt */
	axg_memif_ack_irq(memif, MEMIF_INT_MASK);

	/* Take memory arbitror out of reset */
	ret = reset_control_deassert(memif->arb);
	if (ret)
		clk_disable_unprepare(memif->pclk);

	return ret;
}

static int axg_memif_pcm_close(struct snd_pcm_substream *ss)
{
	struct axg_memif *memif = axg_memif_data(ss);
	int ret;

	/* Put the memory arbitror in reset */
	ret = reset_control_assert(memif->arb);

	/* Disable memif ip and register access */
	clk_disable_unprepare(memif->pclk);

	/* remove IRQ */
	free_irq(memif->irq, ss);

	return ret;
}

static const struct snd_pcm_ops axg_memif_pcm_ops = {
	.open =		axg_memif_pcm_open,
	.close =        axg_memif_pcm_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	axg_memif_pcm_hw_params,
	.hw_free =      axg_memif_pcm_hw_free,
	.pointer =	axg_memif_pcm_pointer,
	.trigger =	axg_memif_pcm_trigger,
};

static int axg_memif_pcm_new(struct snd_soc_pcm_runtime *rtd,
			 unsigned int type)
{
	struct snd_card *card = rtd->card->snd_card;
	size_t size = axg_memif_hw.buffer_bytes_max;

	return snd_pcm_lib_preallocate_pages(
		rtd->pcm->streams[type].substream,
		SNDRV_DMA_TYPE_DEV, card->dev, size, size);
}

static void axg_memif_dai_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct axg_memif *memif = snd_soc_dai_get_drvdata(dai);

	clk_disable_unprepare(memif->pclk);
}

static int axg_toddr_pcm_new(struct snd_soc_pcm_runtime *rtd,
			 struct snd_soc_dai *dai)
{
	return axg_memif_pcm_new(rtd, SNDRV_PCM_STREAM_CAPTURE);
}

static int axg_toddr_dai_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params,
				   struct snd_soc_dai *dai)
{
	struct axg_memif *memif = snd_soc_dai_get_drvdata(dai);
	snd_pcm_format_t format = params_format(params);
	unsigned int type, width, msb = 31;

	/* FIMXE: SPDIF IN will require msb = 28 ... need to query backend */

	switch (snd_pcm_format_physical_width(format)) {
	case 8:
		type = 0; /* 8 samples of 8 bits */
		break;
	case 16:
		type = 2; /* 4 samples of 16 bits - right justified */
		break;
	case 32:
		type = 4; /* 2 samples of 32 bits - right justified */
		break;
	default:
		return -EINVAL;
	}

	width = snd_pcm_format_width(format);

	regmap_update_bits(memif->map, MEMIF_CTRL0,
			   CTRL0_TODDR_TYPE_MASK |
			   CTRL0_TODDR_MSB_POS_MASK |
			   CTRL0_TODDR_LSB_POS_MASK,
			   CTRL0_TODDR_TYPE(type) |
			   CTRL0_TODDR_MSB_POS(msb) |
			   CTRL0_TODDR_MSB_POS(msb - (width - 1)));

	return 0;
}


static int axg_toddr_dai_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct axg_memif *memif = snd_soc_dai_get_drvdata(dai);
	unsigned int fifo_threshold;
	int ret;

	/* Enable pclk to access registers and clock the memif ip */
	ret = clk_prepare_enable(memif->pclk);
	if (ret)
		return ret;

	/* Select orginal data - resampling not supported ATM */
	regmap_update_bits(memif->map, MEMIF_CTRL0, CTRL0_TODDR_SEL_RESAMPLE,
			   0);

	/* Only signed format are supported ATM */
	regmap_update_bits(memif->map, MEMIF_CTRL0, CTRL0_TODDR_EXT_SIGNED,
			   CTRL0_TODDR_EXT_SIGNED);

	/* Apply single buffer mode to the interface */
	regmap_update_bits(memif->map, MEMIF_CTRL0, CTRL0_TODDR_PP_MODE, 0);

	/* TODDR does have a configurable fifo depth */
	fifo_threshold = MEMIF_FIFO_MIN_CNT - 1;
	regmap_update_bits(memif->map, MEMIF_CTRL1, CTRL1_THRESHOLD_MASK,
			   CTRL1_THRESHOLD(fifo_threshold));

	return 0;
}

static const struct snd_soc_dai_ops axg_toddr_ops = {
	.hw_params	= axg_toddr_dai_hw_params,
	.startup	= axg_toddr_dai_startup,
	.shutdown	= axg_memif_dai_shutdown,
};

/* Frontend Capture DAI */
static struct snd_soc_dai_driver axg_toddr_dai_drv = {
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= MEMIF_CHANNEL_MAX,
		.rates		= MEMIF_RATES,
		.formats	= MEMIF_FORMATS,
	},
	.ops		= &axg_toddr_ops,
	.pcm_new	= axg_toddr_pcm_new,
};

static const char * const axg_toddr_sel_texts[] = {
	"IN 0", "IN 1", "IN 2", "IN 3", "IN 4", "IN 6"
};

static const unsigned int axg_toddr_sel_values[] = {
	0, 1, 2, 3, 4, 6
};

static SOC_VALUE_ENUM_SINGLE_DECL(axg_toddr_sel_enum, MEMIF_CTRL0,
				  CTRL0_SEL_SHIFT, CTRL0_SEL_MASK,
				  axg_toddr_sel_texts, axg_toddr_sel_values);

static const struct snd_kcontrol_new axg_toddr_in_mux =
	SOC_DAPM_ENUM("Input Source", axg_toddr_sel_enum);

static const struct snd_soc_dapm_widget axg_toddr_dapm_widgets[] = {
	SND_SOC_DAPM_MUX("SRC SEL", SND_SOC_NOPM, 0, 0, &axg_toddr_in_mux),
	SND_SOC_DAPM_AIF_IN("IN 0", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 1", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 2", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 3", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 4", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN 6", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route axg_toddr_dapm_routes[] = {
	{ "Capture", NULL, "SRC SEL" },
	{ "SRC SEL", "IN 0", "IN 0" },
	{ "SRC SEL", "IN 1", "IN 1" },
	{ "SRC SEL", "IN 2", "IN 2" },
	{ "SRC SEL", "IN 3", "IN 3" },
	{ "SRC SEL", "IN 4", "IN 4" },
	{ "SRC SEL", "IN 6", "IN 6" },
};

static const struct snd_soc_component_driver axg_toddr_component_drv = {
	.dapm_widgets		= axg_toddr_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(axg_toddr_dapm_widgets),
	.dapm_routes		= axg_toddr_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(axg_toddr_dapm_routes),
	.ops 			= &axg_memif_pcm_ops
};

static int axg_frddr_dai_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct axg_memif *memif = snd_soc_dai_get_drvdata(dai);
	unsigned int fifo_depth, fifo_threshold;
	int ret;

	/* Enable pclk to access registers and clock the memif ip */
	ret = clk_prepare_enable(memif->pclk);
	if (ret)
		return ret;

	/* Apply single buffer mode to the interface */
	regmap_update_bits(memif->map, MEMIF_CTRL0, CTRL0_FRDDR_PP_MODE, 0);

	/*
	 * TODO: We could adapt the fifo depth and the fifo threshold
	 * depending on the expected memory throughput and lantencies
	 * For now, we'll just use the same values as the vendor kernel
	 * Depth and threshhold are zero based.
	 */
	fifo_depth = MEMIF_FIFO_MIN_CNT - 1;
	fifo_threshold = (MEMIF_FIFO_MIN_CNT / 2) - 1;
	regmap_update_bits(memif->map, MEMIF_CTRL1,
			   CTRL1_FRDDR_DEPTH_MASK | CTRL1_THRESHOLD_MASK,
			   CTRL1_FRDDR_DEPTH(fifo_depth) |
			   CTRL1_THRESHOLD(fifo_threshold));

	return 0;
}

static int axg_frddr_pcm_new(struct snd_soc_pcm_runtime *rtd,
			     struct snd_soc_dai *dai)
{
	return axg_memif_pcm_new(rtd, SNDRV_PCM_STREAM_PLAYBACK);
}

static const struct snd_soc_dai_ops axg_frddr_ops = {
	.startup	= axg_frddr_dai_startup,
	.shutdown	= axg_memif_dai_shutdown,
};

/* Frontend Playback DAI */
static struct snd_soc_dai_driver axg_frddr_dai_drv = {
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= MEMIF_CHANNEL_MAX,
		.rates		= MEMIF_RATES,
		.formats	= MEMIF_FORMATS,
	},
	.ops		= &axg_frddr_ops,
	.pcm_new	= axg_frddr_pcm_new,
};

static const char * const axg_frddr_sel_texts[] = {
	"OUT 0", "OUT 1", "OUT 2", "OUT 3"
};

static SOC_ENUM_SINGLE_DECL(axg_frddr_sel_enum, MEMIF_CTRL0, CTRL0_SEL_SHIFT,
			    axg_frddr_sel_texts);

static const struct snd_kcontrol_new axg_frddr_out_demux =
	SOC_DAPM_ENUM("Output Sink", axg_frddr_sel_enum);

static const struct snd_soc_dapm_widget axg_frddr_dapm_widgets[] = {
	SND_SOC_DAPM_DEMUX("SINK SEL", SND_SOC_NOPM, 0, 0, &axg_frddr_out_demux),
	SND_SOC_DAPM_AIF_OUT("OUT 0", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("OUT 1", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("OUT 2", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("OUT 3", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route axg_frddr_dapm_routes[] = {
	{ "SINK SEL", NULL, "Playback" },
	{ "OUT 0", "OUT 0",  "SINK SEL" },
	{ "OUT 1", "OUT 1",  "SINK SEL" },
	{ "OUT 2", "OUT 2",  "SINK SEL" },
	{ "OUT 3", "OUT 3",  "SINK SEL" },
};

static const struct snd_soc_component_driver axg_frddr_component_drv = {
	.dapm_widgets		= axg_frddr_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(axg_frddr_dapm_widgets),
	.dapm_routes		= axg_frddr_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(axg_frddr_dapm_routes),
	.ops 			= &axg_memif_pcm_ops
};

static const struct regmap_config axg_memif_regmap_cfg = {
	.reg_bits 	= 32,
	.val_bits 	= 32,
	.reg_stride 	= 4,
	.max_register	= MEMIF_STATUS2,
};

struct axg_memif_match_data {
	const struct snd_soc_component_driver *component_drv;
	struct snd_soc_dai_driver *dai_drv;
};

static const struct axg_memif_match_data axg_toddr_match_data = {
	.component_drv	= &axg_toddr_component_drv,
	.dai_drv	= &axg_toddr_dai_drv
};

static const struct axg_memif_match_data axg_frddr_match_data = {
	.component_drv	= &axg_frddr_component_drv,
	.dai_drv	= &axg_frddr_dai_drv
};

static const struct of_device_id axg_memif_of_match[] = {
	{
		.compatible = "amlogic,meson-axg-toddr",
		.data = &axg_toddr_match_data,
	}, {
		.compatible = "amlogic,meson-axg-frddr",
		.data = &axg_frddr_match_data,
	}, {}
};
MODULE_DEVICE_TABLE(of, axg_memif_of_match);

static int axg_memif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct axg_memif_match_data *data;
	struct axg_memif *memif;
	struct resource *res;
	void __iomem *regs;

	data = of_device_get_match_data(dev);
	if (!data) {
		dev_err(dev, "failed to match device\n");
		return -ENODEV;
	}

	memif = devm_kzalloc(dev, sizeof(memif), GFP_KERNEL);
	if (!memif)
		return -ENOMEM;
	platform_set_drvdata(pdev, memif);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	memif->map = devm_regmap_init_mmio(dev, regs, &axg_memif_regmap_cfg);
	if (IS_ERR(memif->map)) {
		dev_err(dev, "failed to init regmap: %ld\n",
			PTR_ERR(memif->map));
		return PTR_ERR(memif->map);
	}

	memif->pclk = devm_clk_get(dev, NULL);
	if (IS_ERR(memif->pclk)) {
		if (PTR_ERR(memif->pclk) != -EPROBE_DEFER)
			dev_err(dev, "failed to get pclk: %ld\n",
				PTR_ERR(memif->pclk));
		return PTR_ERR(memif->pclk);
	}

	memif->arb = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(memif->arb)) {
		if (PTR_ERR(memif->arb) != -EPROBE_DEFER)
			dev_err(dev, "failed to get arb reset: %ld\n",
				PTR_ERR(memif->arb));
		return PTR_ERR(memif->arb);
	}

	memif->irq = of_irq_get(dev->of_node, 0);
	if (memif->irq <= 0) {
		dev_err(dev, "failed to get irq: %d\n", memif->irq);
	        return memif->irq;
	}

	return devm_snd_soc_register_component(dev, data->component_drv,
					       data->dai_drv, 1);
}

static struct platform_driver axg_memif_pdrv = {
	.probe = axg_memif_probe,
	.driver = {
		.name = "meson-axg-audio-memif",
		.of_match_table = axg_memif_of_match,
	},
};
module_platform_driver(axg_memif_pdrv);

MODULE_DESCRIPTION("Amlogic A113 Audio Memory Interface driver");
MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_LICENSE("GPL v2");
