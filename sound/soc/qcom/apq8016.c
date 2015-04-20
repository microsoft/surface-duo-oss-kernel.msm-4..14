/*
 * Copyright (c) 2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

struct msm8x16_data {
	struct clk *mclk;

	/* pdm */
	struct pinctrl *pinctrl;
	struct pinctrl_state *cdc_lines_sus;
	struct pinctrl_state *cdc_lines_act;

	/* tlmm */
	struct pinctrl_state *tlmm_sus;
	struct pinctrl_state *tlmm_act;

	void __iomem *mic_iomux;
	void __iomem *spkr_iomux;
};

static int mi2s_rx_bit_format = SNDRV_PCM_FORMAT_S16_LE;

static int qcom_ops_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_soc_card *card = soc_runtime->card;
	snd_pcm_format_t format = params_format(params);
	unsigned int rate = params_rate(params);
	unsigned int sysclk_freq;
	int bitwidth, ret;

	snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT), mi2s_rx_bit_format);

	bitwidth = snd_pcm_format_width(format);
	if (bitwidth < 0) {
		dev_err(card->dev, "%s() invalid bit width given: %d\n",
				__func__, bitwidth);
		return bitwidth;
	}
	sysclk_freq = rate * bitwidth * 2;
	ret = snd_soc_dai_set_sysclk(soc_runtime->cpu_dai, 0, sysclk_freq, 0);
	if (ret) {
		dev_err(card->dev, "%s() error setting sysclk to %u: %d\n",
				__func__, sysclk_freq, ret);
		return ret;
	}

	return 0;
}

#define MIC_CTRL_QUA_WS_SLAVE	BIT(17)
#define MIC_CTRL_TLMM_SCLK_EN	BIT(1)
#define SPKR_CTL_TLMM_MCLK_EN	BIT(1)

static int conf_ext_secondary_mux(struct msm8x16_data *pdata)
{
	/* Enable MCLK */
	writel(readl(pdata->spkr_iomux) | SPKR_CTL_TLMM_MCLK_EN,
		     pdata->spkr_iomux);
	/* Configure the Quat MI2S to TLMM */
	writel(readl(pdata->mic_iomux) | 0x02020002, pdata->mic_iomux);

	return 0;
}

static int msm_ext_mi2s_snd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm8x16_data *pdata = snd_soc_card_get_drvdata(card);
	int ret = 0;

	
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		pr_info("%s: Secondary Mi2s does not support capture\n",
					__func__);
		return 0;
	}

	ret = conf_ext_secondary_mux(pdata);
	if (ret < 0) {
		pr_err("%s: failed to conf internal codec mux\n",
						__func__);
		return ret;
	}

	ret = pinctrl_select_state(pdata->pinctrl, pdata->tlmm_act);
	if (ret < 0) {
		pr_err("failed to enable codec gpios\n");
		return -EINVAL;
	}
	//FIXME set DAIFMT_CBS_CFS ???
	
	return 0;
}
static struct snd_soc_ops qcom_internal_codec_soc_ops = {
	.hw_params	= qcom_ops_hw_params,
};

static struct snd_soc_ops qcom_external_codec_soc_ops = {
	.startup 	= msm_ext_mi2s_snd_startup,
	.hw_params	= qcom_ops_hw_params,
};

static struct snd_soc_card qcom_soc_card = {
	.name	= "qcom-apq8016",
	.dev	= NULL,
};

struct qcom_card_data {
	int var;
	struct snd_soc_dai_link dai_link[];	/* dynamically allocated */
};

static int qcom_parse_of(struct snd_soc_card *card)
{
	int num_links;
	struct device *dev = card->dev;
	struct snd_soc_dai_link *dai_link;
	struct device_node *np, *codec, *cpu, *node  = dev->of_node;
	struct msm8x16_data *data = snd_soc_card_get_drvdata(card);
	struct qcom_card_data *priv;
	struct pinctrl *pinctrl;
	char *name;
	int ret;

	data->mclk = of_clk_get_by_name(dev->of_node, "mclk");

	//FIXME 
	if (IS_ERR(data->mclk)) {
		dev_err(dev, "error getting mlck: %ld\n", PTR_ERR(data->mclk));
	} else {
		clk_set_rate(data->mclk, 9600000);
		clk_prepare_enable(data->mclk);
	}

	/* Get pinctrl for both internal and external */
	pinctrl = devm_pinctrl_get(dev);
	data->pinctrl = pinctrl;

	data->tlmm_sus = pinctrl_lookup_state(pinctrl,  "ext_tlmm_lines_sus");
	if (IS_ERR(data->tlmm_sus)) {
		dev_err(dev, "Unable to get pinctrl disable state handle\n");
		return -EINVAL;
	}
	data->tlmm_act = pinctrl_lookup_state(pinctrl,
			"ext_tlmm_lines_act");
	if (IS_ERR(data->tlmm_act)) {
		dev_err(dev, "Unable to get pinctrl disable state handle\n");
		return -EINVAL;
	}

	/* Populate links */
	num_links = of_get_child_count(node);

	/* Allocate the private data and the DAI link array */
	priv = devm_kzalloc(dev, sizeof(*priv) + sizeof(*dai_link) * num_links, GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	card->dai_link	= &priv->dai_link[0];
	card->num_links	= num_links;

	dai_link = priv->dai_link;
	
	for_each_child_of_node(node, np) {
	 	cpu = of_get_child_by_name(np, "cpu");
	 	codec = of_get_child_by_name(np, "codec");

		if (!cpu || !codec) {
			dev_err(dev, "%s: Can't find cpu/codec DT node\n", __func__);
			return -EINVAL;
		}

		dai_link->cpu_of_node = of_parse_phandle(cpu, "sound-dai", 0);
		if (!dai_link->cpu_of_node) {
			dev_err(card->dev, "%s() error getting cpu phandle\n",
					__func__);
			return -EINVAL;
		}
		
		ret = snd_soc_of_get_dai_name(cpu, &dai_link->cpu_dai_name);

		dai_link->platform_of_node = dai_link->cpu_of_node;

		dai_link->codec_of_node = of_parse_phandle(codec, "sound-dai", 0);
		if (!dai_link->codec_of_node) {
			dev_err(card->dev, "%s() error getting codec phandle\n",
					__func__);
			return -EINVAL;
		}
		ret = snd_soc_of_get_dai_name(codec, &dai_link->codec_dai_name);

		/* DAI link name is created from CPU/CODEC dai name */
		name = devm_kzalloc(dev,
			    strlen(dai_link->cpu_dai_name)   +
			    strlen(dai_link->codec_dai_name) + 2,
			    GFP_KERNEL);
		sprintf(name, "%s-%s", dai_link->cpu_dai_name,
				dai_link->codec_dai_name);
		dai_link->name = dai_link->stream_name = name;

		dev_info(dev, "\tname : %s\n", dai_link->stream_name);

		if (of_property_read_bool(np, "external-primary-codec")) {
			dai_link->ops = &qcom_external_codec_soc_ops;
		} else if (of_property_read_bool(np, "external-secodary-codec")) {
			dai_link->ops = &qcom_external_codec_soc_ops;
		} else {
			dai_link->ops = &qcom_internal_codec_soc_ops;
		}

		dai_link++;
	}

	return 0;
}

static int qcom_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card = &qcom_soc_card;
	struct msm8x16_data *data;
	struct resource *res;
	int ret;
	
	if (card->dev) {
		dev_err(&pdev->dev, "%s() error, existing soundcard\n",
				__func__);
		return -ENODEV;
	}

	card->dev = dev;
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mic-iomux");
	data->mic_iomux = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->mic_iomux))
		return PTR_ERR(data->mic_iomux);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "spkr-iomux");
	data->spkr_iomux = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->spkr_iomux))
		return PTR_ERR(data->spkr_iomux);

	platform_set_drvdata(pdev, data);
	snd_soc_card_set_drvdata(card, data);
 
	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret) {
		dev_err(&pdev->dev, "%s() error parsing card name: %d\n",
				__func__, ret);
		return ret;
	}

 	ret = qcom_parse_of(card);
	if (ret) {
		dev_err(&pdev->dev, "%s() error resolving dai links: %d\n",
				__func__, ret);
		return ret;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret == -EPROBE_DEFER) {
		card->dev = NULL;
		return ret;
	} else if (ret) {
		dev_err(&pdev->dev, "%s() error registering soundcard: %d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id qcom_device_id[]  = {
	{ .compatible = "qcom,msm8916-sndcard" },
	{},
};
MODULE_DEVICE_TABLE(of, qcom_device_id);

static struct platform_driver qcom_platform_driver = {
	.driver = {
		.name = "qcom-apq8016",
		.of_match_table =
			of_match_ptr(qcom_device_id),
	},
	.probe = qcom_platform_probe,
};
module_platform_driver(qcom_platform_driver);

MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("APQ8016 Machine Driver");
MODULE_LICENSE("GPL v2");
