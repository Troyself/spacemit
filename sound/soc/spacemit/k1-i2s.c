#include "sound/pcm.h"
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <linux/reset.h>

#define SSCR		0x00	
#define SSFCR		0x04
#define SSINTEN		0x08
#define SSTO		0x0C
#define SSDATR		0x10
#define SSSR		0x14
#define SSPSP		0x18
#define SSNWCR 		0x1C
#define SSNWS		0x20
#define SSRWT		0x24
#define SSRWTCC		0x28	
#define SSRWTCV		0x2C	/* root counter value write for read request register */

#define SSCR_TTELP		(1 << 18)
#define SSCR_TTE		(1 << 17)
#define SSCR_SCFR		(1 << 16)
#define SSCR_IFS		(1 << 15)
#define SSCR_HOLD_FRAME_LOW	(1 << 14)
#define SSCR_TRAIL_PXA		(0 << 13)
#define SSCR_DW_8BYTE		(0x7 << 5)
#define SSCR_DW_16BYTE		(0xf << 5)
#define SSCR_DW_18BYTE		(0x11 << 5)
#define SSCR_DW_32BYTE		(0x1f << 5)

#define SPACEMIT_PCM_RATES			SNDRV_PCM_RATE_8000_192000
#define SPACEMIT_PCM_FORMATS			(SNDRV_PCM_FMTBIT_S8 | \
						 SNDRV_PCM_FMTBIT_S16_LE | \
						 SNDRV_PCM_FMTBIT_S24_LE | \
						 SNDRV_PCM_FMTBIT_S32_LE)

#define SSPA_CLK_BASE	0xd4015000
#define SSP0_CLK	0x80
#define I2S_CLK_BASE	0xd4050000
#define ISCCR1		0x44

struct spacemit_i2s_dev {
	struct device *dev;

	void __iomem *base;
	/*struct regmap *regmap;*/
	struct resource *res;
	struct reset_control *reset;
	struct clk *sysclk;
	struct clk *bclk;
	struct clk *sspa_clk;
	void __iomem *regs;
	void __iomem *sspa_clk_base;
	void __iomem *i2s_clk_base;
    unsigned long sysclk_rate;
	
	struct snd_dmaengine_dai_dma_data capture_dma_data;
	struct snd_dmaengine_dai_dma_data playback_dma_data;
	
	bool has_capture;
	bool has_playback;
	
	/* tmp */
	int dai_fmt;

	int started_count;
};

static int spacemit_i2s_runtime_suspend(struct device *dev)
{
	return 0;
}

static int spacemit_i2s_runtime_resume(struct device *dev)
{
	return 0;
}

#define I2S_PERIOD_SIZE          1024
#define I2S_PERIOD_COUNT         4
static const struct snd_pcm_hardware spacemit_pcm_hardware = {
	.info		  = SNDRV_PCM_INFO_INTERLEAVED |
			    SNDRV_PCM_INFO_BATCH,
	.formats          = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates            = SNDRV_PCM_RATE_48000,
	.rate_min         = SNDRV_PCM_RATE_48000,
	.rate_max         = SNDRV_PCM_RATE_48000,
	.channels_min     = 1,
	.channels_max     = 2,
	.buffer_bytes_max = I2S_PERIOD_SIZE * I2S_PERIOD_COUNT * 4,
	.period_bytes_min = I2S_PERIOD_SIZE * 4,
	.period_bytes_max = I2S_PERIOD_SIZE * 4,
	.periods_min	  = I2S_PERIOD_COUNT,
	.periods_max	  = I2S_PERIOD_COUNT,
};

static const struct snd_dmaengine_pcm_config spacemit_dmaengine_pcm_config = {
	.pcm_hardware = &spacemit_pcm_hardware,
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.chan_names = {"tx", "rx"},
	.prealloc_buffer_size = 16 * 1024,
};

/*static int spacemit_snd_txctrl(struct rk_i2s_dev *i2s, int on)*/
/*{*/
	/*return 0;*/
/*}*/

/*static int spacemit_snd_rxctrl(struct rk_i2s_dev *i2s, int on)*/
/*{*/
	/*int ret = 0;*/
	/*return ret;*/
/*}*/

#define SSCR_SSE			BIT(0)		/* SPI/I2S Enable */
#define SSCR_FRF			GENMASK(2, 1)	/* Frame Format*/
#define SSCR_TRAIL			BIT(13)

#define SSFCR_TFT			GENMASK(3, 0)   /* TXFIFO Trigger Threshold */
#define SSFCR_RFT			GENMASK(8, 5)   /* RXFIFO Trigger Threshold */
#define SSFCR_TSRE			BIT(10)		/* Transmit Service Request Enable */
#define SSFCR_RSRE			BIT(11)		/* Receive Service Request Enable */

#define SSPSP_FSRT			BIT(3)		/* Frame Sync Relative Timing Bit */
#define SSPSP_SFRMP			BIT(4)		/* Serial Frame Polarity */
#define SSPSP_SFRMWDTH(x)		((x) << 12)	/* Serial Frame Width field  */

static int spacemit_i2s_startup(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct spacemit_i2s_dev *i2s = snd_soc_dai_get_drvdata(dai);
	u32 sscr_val, sspsp_val, ssinten_val, ssto_val, ssfcr_val;
	u32 val;

	dev_err(i2s->dev, "enter startup");
	sscr_val = SSCR_TRAIL | SSCR_DW_32BYTE | (0x3 << 1);
	ssfcr_val = (0xf << 0) | (0xf << 5)| SSFCR_RSRE | SSFCR_TSRE;
	sspsp_val = SSPSP_SFRMP;

	val = readl(i2s->base + SSCR);

	if (val & SSCR_SSE)
		return 0;

	switch (i2s->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/*sspsp_val |= SSPSP_FSRT | SSPSP_SFRMWDTH(0x10);*/
		sspsp_val |= (0x10 << 12)| (0x1 << 3);
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/*sspsp_val |= SSPSP_SFRMWDTH(0x1);*/
		sspsp_val |= (0x1 << 12);
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/*sspsp_val |= SSPSP_FSRT | SSPSP_SFRMWDTH(0x1);*/
		sspsp_val |= (0x1 << 12)| (0x1 << 3);
		break;
	default:
		dev_err(i2s->dev, "unexpected format type");
		return -EINVAL;
	}

	writel(sscr_val, i2s->base + SSCR);
	writel(ssfcr_val, i2s->base + SSFCR);
	writel(sspsp_val, i2s->base + SSPSP);
	writel(0, i2s->base + SSINTEN);

	val = readl(i2s->base + SSRWT);
	val |= 1;
	writel(val, i2s->base + SSRWT);
	/*regmap_write(i2s->regmap, SSTO, val);*/
	dev_err(i2s->dev, "cr:0x%x\nfcr:0x%x\npsp:0x%x\nto:0x%x\nssinten:0x%x\n",
				readl(i2s->base + SSCR),
				readl(i2s->base + SSFCR),
				readl(i2s->base + SSPSP),
				readl(i2s->base + SSTO),
				readl(i2s->base + SSINTEN));

	return 0;
}

static int spacemit_i2s_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct spacemit_i2s_dev *i2s = snd_soc_dai_get_drvdata(dai);
	u32 data_width = 0, data_bits = 0, channels, fs;
	struct snd_dmaengine_dai_dma_data *dma_data;
	u32 val;
	u32 target;
	unsigned long bclk_rate;
	int ret;
	unsigned int mclk_fs;

	dev_err(i2s->dev, "enter hw_params");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &i2s->playback_dma_data;
	else
		dma_data = &i2s->capture_dma_data;

	dev_err(i2s->dev, "dma_data: %p", dma_data);

	dma_data->addr = i2s->res->start + SSDATR;
	/*dev_err(i2s->dev, "i2s->res->start: %lld", i2s->res->start);*/
	dev_err(i2s->dev, "dma->addr:0x %llx", dma_data->addr);
	
	dev_err(i2s->dev, "dma_data: %p", dma_data);
	
	dev_err(i2s->dev, "hw params rate : %ld", params_rate(params));

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		data_bits = 8;
		data_width = SSCR_DW_8BYTE;
		dma_data->maxburst = 8;
		dma_data->addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		data_bits = 16;
		data_width = SSCR_DW_16BYTE;
		dma_data->maxburst = 16;
		dma_data->addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		if ((i2s->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_I2S) {
			data_width = SSCR_DW_32BYTE;
			dma_data->maxburst = 32;
			dma_data->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		}
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data_bits = 32;
		data_width = SSCR_DW_32BYTE;
		dma_data->maxburst = 32;
		dma_data->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		pr_debug("%s, unexpected data width type\n", __func__);
		return -EINVAL;
	}
	dev_err(i2s->dev, "hw params set over");

	/*regmap_update_bits(i2s->regmap, SSCR,*/
			   /*SSCR_DW_32BYTE, data_width);*/
	val = readl(i2s->base + SSCR);
	val &= ~SSCR_DW_32BYTE;
	val |= data_width;
	writel(val, i2s->base + SSCR);
	dev_err(i2s->dev, "hw params update over");

	snd_soc_dai_set_dma_data(dai, substream, dma_data);
	dev_err(i2s->dev, "hw params set dmadata over");

	mclk_fs = i2s->sysclk_rate / params_rate(params);
	dev_err(i2s->dev, "mclk_fs: %d", mclk_fs);
	switch(mclk_fs) {
	case 64:
		clk_set_rate(i2s->sysclk, 3072000);
		break;
	case 128:
		clk_set_rate(i2s->sysclk, 6144000);
		break;
	case 256:
		clk_set_rate(i2s->sysclk, 12288000);
		break;
	default:
		clk_set_rate(i2s->sysclk, 12288000);
		break;
	}
	dev_err(i2s->dev, "the real sysclk: %ld", clk_get_rate(i2s->sysclk));

	bclk_rate = params_channels(params) *
		params_rate(params) *
		data_bits;
	ret = clk_set_rate(i2s->bclk, bclk_rate);
	if (ret)
	dev_err(i2s->dev, "Fail to set bclk %d\n", ret);
	ret = clk_set_rate(i2s->sspa_clk, bclk_rate);
	if (ret)
	dev_err(i2s->dev, "Fail to set sspa_clk %d\n", ret);

	dev_err(i2s->dev, "the rate we want: %ld", bclk_rate);

	dev_err(i2s->dev, "test rate: bclk: %ld, sspa_clk: %ld\n", clk_get_rate(i2s->bclk), clk_get_rate(i2s->sspa_clk));


	
	u32 i2s_val =  readl(i2s->i2s_clk_base+ ISCCR1);
	val = readl(i2s->sspa_clk_base + SSP0_CLK);
	pr_err("hw_params: sspa_val: 0x%x; i2s_val: 0x%x\n", val, i2s_val);
	pr_err("hw_params: data_bits = %d, data_width = 0x%x, channels = %d\n", data_bits, data_width, params_channels(params));
	
	dev_err(i2s->dev, "cr:0x%x\nfcr:0x%x\npsp:0x%x\nto:0x%x\nssinten:0x%x\n",
				readl(i2s->base + SSCR),
				readl(i2s->base + SSFCR),
				readl(i2s->base + SSPSP),
				readl(i2s->base + SSTO),
				readl(i2s->base + SSINTEN));
	
	return 0;
}

static int spacemit_i2s_set_sysclk(struct snd_soc_dai *cpu_dai, int clk_id,
				   unsigned int freq, int dir)
{
	struct spacemit_i2s_dev *i2s = dev_get_drvdata(cpu_dai->dev);
	int ret = 0;

	if (freq == 0)
		return 0;

    /*ret = clk_set_rate(i2s->sysclk, freq);*/
    /*if (ret)*/
        /*dev_err(i2s->dev, "Fail to set sysclk %d\n", ret);*/

    dev_err(i2s->dev, "set sysclk: %d", freq);
    i2s->sysclk_rate = freq;

	return ret;
}

static int spacemit_i2s_set_fmt(struct snd_soc_dai *cpu_dai,
				unsigned int fmt)
{
	struct spacemit_i2s_dev *i2s = dev_get_drvdata(cpu_dai->dev);

	dev_err(i2s->dev, "enter set_fmt");
	
	i2s->dai_fmt = fmt;
	switch(fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		cpu_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
		cpu_dai->driver->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		cpu_dai->driver->playback.channels_min = 1;
		cpu_dai->driver->playback.channels_max = 1;
		cpu_dai->driver->capture.channels_min = 1;
		cpu_dai->driver->capture.channels_max = 1;
		cpu_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S32_LE;
		cpu_dai->driver->capture.formats = SNDRV_PCM_FMTBIT_S32_LE;
		break;
	default:
		dev_err(i2s->dev, "unexpected format type");
		return -EINVAL;
	}
	
	return 0;
}

static int spacemit_i2s_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	struct spacemit_i2s_dev *i2s = snd_soc_dai_get_drvdata(dai);
	u32 val;
	int ret = 0;

	dev_err(i2s->dev, "enter trigger");

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (!i2s->started_count) {
			dev_err(i2s->dev, "start!!!");
			val = readl(i2s->base + SSCR);
			if (ret) {
				dev_err(i2s->dev, "failed to read sscr in trigger");
				return ret;
			}
			val |= SSCR_SSE;
			writel(val, i2s->base + SSCR);
			/*regmap_write(i2s->regmap, SSCR, val);*/
		} else {
			dev_err(i2s->dev, "started!!!");
		}
		i2s->started_count++;
		dev_err(i2s->dev, "trigger uh start");
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (i2s->started_count)
			i2s->started_count --;
		if (!i2s->started_count) {
			dev_err(i2s->dev, "stop!!!");
			val = readl(i2s->base + SSCR);
			if (ret) {
				dev_err(i2s->dev, "failed to read sscr in trigger");
				return ret;
			}
			val &= ~SSCR_SSE;
			writel(val, i2s->base + SSCR);
		} else {
			dev_err(i2s->dev, "stoped!!!");
		}
		dev_err(i2s->dev, "trigger uh stop");
		break;
	default:
		ret = -EINVAL;
		dev_err(i2s->dev, "trigger fuck");
		break;
	}

	dev_err(i2s->dev, "cr:0x%x\nfcr:0x%x\npsp:0x%x\nto:0x%x\nssinten:0x%x\n",
				readl(i2s->base + SSCR),
				readl(i2s->base + SSFCR),
				readl(i2s->base + SSPSP),
				readl(i2s->base + SSTO),
				readl(i2s->base + SSINTEN));

	return ret;
}

static int spacemit_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct spacemit_i2s_dev *i2s = snd_soc_dai_get_drvdata(dai);

	dev_err(i2s->dev, "enter dai probe");

	snd_soc_dai_init_dma_data(dai,
		i2s->has_playback ? &i2s->playback_dma_data : NULL,
		i2s->has_capture  ? &i2s->capture_dma_data  : NULL);

	reset_control_deassert(i2s->reset);

	return 0;
}

static const struct snd_soc_dai_ops spacemit_i2s_dai_ops = {
	.probe = spacemit_i2s_dai_probe,
	.startup = spacemit_i2s_startup,
	.hw_params = spacemit_i2s_hw_params,
	.set_sysclk = spacemit_i2s_set_sysclk,
	.set_fmt = spacemit_i2s_set_fmt,
	.trigger = spacemit_i2s_trigger,
};

static struct snd_soc_dai_driver spacemit_i2s_dai = {
	.ops = &spacemit_i2s_dai_ops,
    .playback = {
        .channels_min = 1,
        .channels_max = 2,
        .rates = SNDRV_PCM_RATE_48000,
        .formats = SPACEMIT_PCM_FORMATS,
    },
    .capture = {
        .channels_min = 1,
        .channels_max = 2,
        .rates = SNDRV_PCM_RATE_48000,
        .formats = SPACEMIT_PCM_FORMATS,
    },
	.symmetric_rate = 1,
};

static const struct snd_soc_component_driver spacemit_i2s_component = {
	.name = "i2s-k1",
	.legacy_dai_naming = 1,
};

static int spacemit_i2s_init_dai(struct spacemit_i2s_dev *i2s,
				 struct snd_soc_dai_driver **dp)
{
	struct device_node *node = i2s->dev->of_node;
	struct snd_soc_dai_driver *dai;
	struct property *dma_names;
	const char *dma_name;

	of_property_for_each_string(node, "dma-names", dma_names, dma_name) {
		if (!strcmp(dma_name, "tx"))
			i2s->has_playback = true;
		if (!strcmp(dma_name, "rx"))
			i2s->has_capture = true;
	}

	dai = devm_kmemdup(i2s->dev, &spacemit_i2s_dai,
			   sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	if (i2s->has_playback) {
		dev_err(i2s->dev, "init dai: has playback");
		dai->playback.stream_name = "Playback";
		dai->playback.channels_min = 1;
		dai->playback.channels_max = 2;
		dai->playback.rates = SPACEMIT_PCM_RATES;
		dai->playback.formats = SPACEMIT_PCM_FORMATS;

		i2s->playback_dma_data.addr = i2s->res->start + SSDATR;
		i2s->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		i2s->playback_dma_data.maxburst = 32;
	}

	if (i2s->has_capture) {
		dev_err(i2s->dev, "init dai: has capture");
		dai->capture.stream_name = "Capture";
		dai->capture.channels_min = 1;
		dai->capture.channels_max = 2;
		dai->capture.rates = SPACEMIT_PCM_RATES;
		dai->capture.formats = SPACEMIT_PCM_FORMATS;

		i2s->capture_dma_data.addr = i2s->res->start + SSDATR;
		i2s->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		i2s->capture_dma_data.maxburst = 32;
	}

	if (dp)
		*dp = dai;

	return 0;
}
static int spacemit_i2s_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_driver *dai;
	struct spacemit_i2s_dev *i2s;
	int ret;
	struct clk *clk;
	u32 val;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	i2s->dev = &pdev->dev;
	
	dev_err(i2s->dev, "ready to get sysclk");
    i2s->sysclk = devm_clk_get_enabled(i2s->dev, "sysclk");
    if(IS_ERR(i2s->sysclk))
        return dev_err_probe(i2s->dev, PTR_ERR(i2s->sysclk), "failed to enable sysbase clock\n");

    i2s->bclk= devm_clk_get_enabled(i2s->dev, "bclk");
    if (IS_ERR(i2s->bclk))
        return dev_err_probe(i2s->dev, PTR_ERR(i2s->bclk), "failed to enable bit clock\n");

    clk = devm_clk_get_enabled(i2s->dev, "sspa_bus");
    if (IS_ERR(clk))
        return dev_err_probe(i2s->dev, PTR_ERR(clk), "failed to enable sspa_bus clock\n");
	
    i2s->sspa_clk = devm_clk_get_enabled(i2s->dev, "sspa");
    if (IS_ERR(clk))
        return dev_err_probe(i2s->dev, PTR_ERR(clk), "failed to enable sspa clock\n");


	i2s->base = devm_platform_get_and_ioremap_resource(pdev, 0, &i2s->res);
	if (IS_ERR(i2s->base))
		return dev_err_probe(i2s->dev, PTR_ERR(i2s->base), "failed to map registers\n");

	/*i2s->regmap = devm_regmap_init_mmio(i2s->dev, regs,*/
					    /*&spacemit_i2s_regmap_config);*/
	/*if (IS_ERR(i2s->regmap))*/
		/*return dev_err_probe(i2s->dev, PTR_ERR(i2s->regmap),*/
				     /*"failed to initialise managed register map");*/

	i2s->sspa_clk_base = ioremap(SSPA_CLK_BASE, 0x100);
	if (IS_ERR(i2s->sspa_clk_base))
		return dev_err_probe(i2s->dev, PTR_ERR(i2s->sspa_clk_base), "failed to ioremap sspa_clk_base");


	i2s->i2s_clk_base = ioremap(I2S_CLK_BASE, 0x100);
	if (IS_ERR(i2s->i2s_clk_base))
		return dev_err_probe(i2s->dev, PTR_ERR(i2s->i2s_clk_base), "failed to ioremap i2s_clk_base");


	val = readl(i2s->sspa_clk_base + SSP0_CLK);
	/* i2s as parent; enable i2s as parent; enable function; enable bus */
	val |= (7<<4) | (1<<3) | (1<<1) | (1<<0);
	writel(val, i2s->sspa_clk_base + SSP0_CLK);

	i2s->reset =  devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(i2s->reset))
		return dev_err_probe(i2s->dev, PTR_ERR(i2s->reset),
				     "failed to get reset control");

	dev_set_drvdata(i2s->dev, i2s);

	spacemit_i2s_init_dai(i2s, &dai);

	ret = devm_snd_soc_register_component(i2s->dev,
					      &spacemit_i2s_component,
					      dai, 1);
	if (ret)
		return dev_err_probe(i2s->dev, ret, "failed to register component");

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, &spacemit_dmaengine_pcm_config, 0);
	/*ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);*/
	dev_err(i2s->dev, "dmaengine pcm: %d", ret);
	return ret;
}

static void spacemit_i2s_remove(struct platform_device *pdev)
{

}

static const struct of_device_id spacemit_i2s_of_match[] = {
	{ .compatible = "spacemit,k1-i2s", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spacemit_i2s_of_match);

static struct platform_driver spacemit_i2s_driver = {
	.probe = spacemit_i2s_probe,
	.remove = spacemit_i2s_remove,
	.driver = {
		.name = "i2s-k1",
		.of_match_table = spacemit_i2s_of_match,
	},
};
module_platform_driver(spacemit_i2s_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2S bus driver for SpacemiT K1 SoC");
