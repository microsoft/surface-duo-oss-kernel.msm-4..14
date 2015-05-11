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
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

struct apq8016_card_data {
	void __iomem *mic_iomux;
	void __iomem *spkr_iomux;
	struct snd_soc_dai_link dai_link[];	/* dynamically allocated */
};

#define MIC_CTRL_QUA_WS_SLAVE_SEL_10	BIT(17)
#define MIC_CTRL_TLMM_SCLK_EN		BIT(1)
#define	SPKR_CTL_PRI_WS_SLAVE_SEL_11	(BIT(17) | BIT(16))

static int msm_ext_mi2s_snd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct apq8016_card_data *pdata = snd_soc_card_get_drvdata(card);

	/* Configure the Quat MI2S to TLMM */
	writel(readl(pdata->mic_iomux) |
			MIC_CTRL_QUA_WS_SLAVE_SEL_10 |
			MIC_CTRL_TLMM_SCLK_EN,
			pdata->mic_iomux);

	return 0;
}

static int msm_int_mi2s_snd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct apq8016_card_data *pdata = snd_soc_card_get_drvdata(card);

	writel(readl(pdata->spkr_iomux) | SPKR_CTL_PRI_WS_SLAVE_SEL_11,
		pdata->spkr_iomux);

	return 0;
}
static struct snd_soc_ops qcom_internal_codec_soc_ops = {
	.startup	= msm_int_mi2s_snd_startup,
};

static struct snd_soc_ops qcom_external_codec_soc_ops = {
	.startup	= msm_ext_mi2s_snd_startup,
};

static struct snd_soc_card qcom_soc_card = {
	.name	= "apq8016",
};

static struct apq8016_card_data *qcom_parse_of(struct snd_soc_card *card)
{
	int num_links;
	struct device *dev = card->dev;
	struct snd_soc_dai_link *dai_link;
	struct device_node *np, *codec, *cpu, *node  = dev->of_node;
	struct apq8016_card_data *data;
	char *name;
	int ret;

	/* Populate links */
	num_links = of_get_child_count(node);

	/* Allocate the private data and the DAI link array */
	data = devm_kzalloc(dev, sizeof(*data) + sizeof(*dai_link) * num_links,
			    GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	card->dai_link	= &data->dai_link[0];
	card->num_links	= num_links;

	dai_link = data->dai_link;

	for_each_child_of_node(node, np) {
		cpu = of_get_child_by_name(np, "cpu");
		codec = of_get_child_by_name(np, "codec");

		if (!cpu || !codec) {
			dev_err(dev, "Can't find cpu/codec DT node\n");
			return ERR_PTR(-EINVAL);
		}

		dai_link->cpu_of_node = of_parse_phandle(cpu, "sound-dai", 0);
		if (!dai_link->cpu_of_node) {
			dev_err(card->dev, "error getting cpu phandle\n");
			return ERR_PTR(-EINVAL);
		}

		dai_link->codec_of_node = of_parse_phandle(codec,
							   "sound-dai",
							   0);
		if (!dai_link->codec_of_node) {
			dev_err(card->dev, "error getting codec phandle\n");
			return ERR_PTR(-EINVAL);
		}

		ret = snd_soc_of_get_dai_name(cpu, &dai_link->cpu_dai_name);
		if (ret) {
			dev_err(card->dev, "error getting cpu dai name\n");
			return ERR_PTR(ret);
		}

		ret = snd_soc_of_get_dai_name(codec, &dai_link->codec_dai_name);
		if (ret) {
			dev_err(card->dev, "error getting codec dai name\n");
			return ERR_PTR(ret);
		}

		dai_link->platform_of_node = dai_link->cpu_of_node;
		/* For now we only support playback */
		dai_link->playback_only = true;

		if (of_property_read_bool(np, "external")) {
			name = "HDMI";
			dai_link->ops = &qcom_external_codec_soc_ops;

		} else {
			name = "Headset";
			dai_link->ops = &qcom_internal_codec_soc_ops;
		}

		dai_link->name = dai_link->stream_name = name;

		dai_link++;
	}

	return data;
}

static int qcom_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card = &qcom_soc_card;
	struct apq8016_card_data *data;
	struct resource *res;
	int ret;

	card->dev = dev;
	data = qcom_parse_of(card);
	if (IS_ERR(data)) {
		dev_err(&pdev->dev, "Error resolving dai links: %d\n",
			PTR_ERR(data));
		return PTR_ERR(data);
	}

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
		dev_err(&pdev->dev, "Error parsing card name: %d\n", ret);
		return ret;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret == -EPROBE_DEFER) {
		card->dev = NULL;
		return ret;
	} else if (ret) {
		dev_err(&pdev->dev, "Error registering soundcard: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id qcom_device_id[]  = {
	{ .compatible = "qcom,apq8016-sndcard" },
	{},
};
MODULE_DEVICE_TABLE(of, qcom_device_id);

static struct platform_driver qcom_platform_driver = {
	.driver = {
		.name = "qcom-apq8016",
		.of_match_table = of_match_ptr(qcom_device_id),
	},
	.probe = qcom_platform_probe,
};
module_platform_driver(qcom_platform_driver);

MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("APQ8016 ASoC Machine Driver");
MODULE_LICENSE("GPL v2");
