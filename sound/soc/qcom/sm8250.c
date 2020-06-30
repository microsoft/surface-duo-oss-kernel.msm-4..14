// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020, Linaro Limited

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include "qdsp6/q6afe.h"

#define MI2S_BCLK_RATE		1536000
#define DEFAULT_MCLK_RATE	24576000

static int sm8250_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	return 0;
}

static int sm8250_snd_startup(struct snd_pcm_substream *substream)
{
	unsigned int fmt = SND_SOC_DAIFMT_CBS_CFS;
	unsigned int codec_dai_fmt = SND_SOC_DAIFMT_CBS_CFS;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF;

		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_MCLK_1,
			DEFAULT_MCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);

		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	default:
		break;
	}
	return 0;
}

static const struct snd_soc_ops sm8250_be_ops = {
	.startup = sm8250_snd_startup,
};

static void sm8250_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->be_hw_params_fixup = sm8250_be_hw_params_fixup;
			link->ops = &sm8250_be_ops;
		}
	}
}

int sm8250_snd_parse_of(struct snd_soc_card *card)
{
	struct device_node *np;
	struct device_node *codec = NULL;
	struct device_node *platform = NULL;
	struct device_node *cpu = NULL;
	struct device *dev = card->dev;
	struct snd_soc_dai_link *link;
	struct of_phandle_args args;
	struct snd_soc_dai_link_component *dlc;
	int ret, num_links;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret) {
		dev_err(dev, "Error parsing card name: %d\n", ret);
		return ret;
	}

	/* DAPM routes */
	if (of_property_read_bool(dev->of_node, "audio-routing")) {
		ret = snd_soc_of_parse_audio_routing(card,
				"audio-routing");
		if (ret)
			return ret;
	}

	/* Populate links */
	num_links = of_get_child_count(dev->of_node);

	/* Allocate the DAI link array */
	card->dai_link = kcalloc(num_links, sizeof(*link), GFP_KERNEL);
	if (!card->dai_link)
		return -ENOMEM;

	card->num_links = num_links;
	link = card->dai_link;

	for_each_child_of_node(dev->of_node, np) {
		dlc = devm_kzalloc(dev, 2 * sizeof(*dlc), GFP_KERNEL);
		if (!dlc)
			return -ENOMEM;

		link->cpus	= &dlc[0];
		link->platforms	= &dlc[1];

		link->num_cpus		= 1;
		link->num_platforms	= 1;

		ret = of_property_read_string(np, "link-name", &link->name);
		if (ret) {
			dev_err(card->dev, "error getting codec dai_link name\n");
			goto err;
		}

		cpu = of_get_child_by_name(np, "cpu");
		platform = of_get_child_by_name(np, "platform");

		if (!cpu) {
			dev_err(dev, "%s: Can't find cpu DT node\n", link->name);
			ret = -EINVAL;
			goto err;
		}

		ret = of_parse_phandle_with_args(cpu, "sound-dai",
					"#sound-dai-cells", 0, &args);
		if (ret) {
			dev_err(card->dev, "%s: error getting cpu phandle\n", link->name);
			goto err;
		}
		link->cpus->of_node = args.np;
		link->id = args.args[0];

		ret = snd_soc_of_get_dai_name(cpu, &link->cpus->dai_name);
		if (ret) {
			dev_err(card->dev, "%s: error getting cpu dai name\n", link->name);
			goto err;
		}

		if (platform) {
			link->platforms->of_node = of_parse_phandle(platform,
					"sound-dai",
					0);
			if (!link->platforms->of_node) {
				dev_err(card->dev, "%s: platform dai not found\n", link->name);
				ret = -EINVAL;
				goto err;
			}

			dlc = devm_kzalloc(dev, sizeof(*dlc), GFP_KERNEL);
			if (!dlc)
				return -ENOMEM;

			link->codecs	 = dlc;
			link->num_codecs = 1;
			link->codecs->dai_name = "snd-soc-dummy-dai";
			link->codecs->name = "snd-soc-dummy";

			link->dynamic = 1;
			link->no_pcm = 1;
			link->ignore_pmdown_time = 1;

			if (q6afe_is_rx_port(link->id)) {
				link->dpcm_playback = 1;
				link->dpcm_capture = 0;
			} else {
				link->dpcm_playback = 0;
				link->dpcm_capture = 1;
			}

		} else {
			dlc = devm_kzalloc(dev, sizeof(*dlc), GFP_KERNEL);
			if (!dlc)
				return -ENOMEM;

			link->codecs	 = dlc;
			link->num_codecs = 1;

			link->platforms->of_node = link->cpus->of_node;
			link->codecs->dai_name = "snd-soc-dummy-dai";
			link->codecs->name = "snd-soc-dummy";
			link->dynamic = 1;
			link->dpcm_playback = 1;
			link->dpcm_capture = 1;
		}

		link->ignore_suspend = 1;
		link->nonatomic = 1;
		link->stream_name = link->name;
		link++;

		of_node_put(cpu);
		of_node_put(platform);
	}

	return 0;
err:
	of_node_put(np);
	of_node_put(cpu);
	of_node_put(codec);
	of_node_put(platform);
	kfree(card->dai_link);
	return ret;
}

static int sm8250_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct device *dev = &pdev->dev;
	int ret;

	card = kzalloc(sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->dev = dev;
	dev_set_drvdata(dev, card);
	ret = sm8250_snd_parse_of(card);
	if (ret)
		goto err;

	sm8250_add_be_ops(card);
	ret = snd_soc_register_card(card);
	if (ret)
		goto err_card_register;

	return 0;

err_card_register:
	kfree(card->dai_link);
err:
	kfree(card);
	return ret;
}

static int sm8250_platform_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_card(card);
	kfree(card->dai_link);
	kfree(card);

	return 0;
}

static const struct of_device_id snd_sm8250_dt_match[] = {
	{.compatible = "qcom,sm8250-sndcard"},
	{}
};

MODULE_DEVICE_TABLE(of, snd_sm8250_dt_match);

static struct platform_driver snd_sm8250_driver = {
	.probe  = sm8250_platform_probe,
	.remove = sm8250_platform_remove,
	.driver = {
		.name = "snd-sm8250",
		.of_match_table = snd_sm8250_dt_match,
	},
};
module_platform_driver(snd_sm8250_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("SM8250 ASoC Machine Driver");
MODULE_LICENSE("GPL v2");
