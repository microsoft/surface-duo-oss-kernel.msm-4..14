/*
 * linux/sound/soc/hisilicon/hi6210-hdmi-codec.c
 *
 * Copyright (C) 2015 Linaro, Ltd
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 */

#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/soc.h>

static struct snd_soc_dai_driver hi6210_hdmi_dai = {
	.name = "hi6210_hdmi_dai",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static struct snd_soc_codec_driver hi6210_hdmi_codec;

static int hi6210_hdmi_probe(struct platform_device *pdev)
{
	int ret;

	ret = snd_soc_register_codec(&pdev->dev, &hi6210_hdmi_codec,
			&hi6210_hdmi_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_codec failed (%d)\n",
					ret);
		return ret;
	}
	return 0;
}

static int hi6210_hdmi_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static const struct of_device_id hi6210_hdmi_dt_ids[] = {
	{ .compatible = "hisilicon,hi6210-hdmi-audio-codec" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, hi6210_hdmi_dt_ids);

static struct platform_driver hi6210_hdmi_driver = {
	.driver = {
		.name = "hi6210-hdmi-audio",
		.of_match_table = hi6210_hdmi_dt_ids,
	},
	.probe = hi6210_hdmi_probe,
	.remove = hi6210_hdmi_remove,
};

module_platform_driver(hi6210_hdmi_driver);

MODULE_AUTHOR("andy.green@linaro.org");
MODULE_DESCRIPTION("Hisilicon HDMI codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hi6210-hdmi-audio");
