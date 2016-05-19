
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of_device.h>

#include <linux/pm_runtime.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include "msm-pcm-routing.h"

static int msm_hdmi_rx_ch = 2;
static int hdmi_rate_variable;

static const char *hdmi_rx_ch_text[] = {"Two", "Three", "Four", "Five",
	"Six", "Seven", "Eight"};
static const char * const hdmi_rate[] = {"Default", "Variable"};

static const struct soc_enum msm_enum[] = {
	SOC_ENUM_SINGLE_EXT(7, hdmi_rx_ch_text),
	SOC_ENUM_SINGLE_EXT(2, hdmi_rate),
};

static int msm_hdmi_rx_ch_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm_hdmi_rx_ch  = %d\n", __func__,
			msm_hdmi_rx_ch);
	ucontrol->value.integer.value[0] = msm_hdmi_rx_ch - 2;
	return 0;
}

static int msm_hdmi_rx_ch_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	msm_hdmi_rx_ch = ucontrol->value.integer.value[0] + 2;

	pr_debug("%s: msm_hdmi_rx_ch = %d\n", __func__,
		msm_hdmi_rx_ch);
	return 1;
}
	
static int msm_hdmi_rate_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	hdmi_rate_variable = ucontrol->value.integer.value[0];
	pr_debug("%s: hdmi_rate_variable = %d\n", __func__, hdmi_rate_variable);
	return 0;
}

static int msm_hdmi_rate_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = hdmi_rate_variable;
	return 0;
}

static const struct snd_kcontrol_new tabla_msm_controls[] = {
	SOC_ENUM_EXT("HDMI_RX Channels", msm_enum[0],
		msm_hdmi_rx_ch_get, msm_hdmi_rx_ch_put),
	SOC_ENUM_EXT("HDMI RX Rate", msm_enum[1],
					msm_hdmi_rate_get,
					msm_hdmi_rate_put),
};

int msm_hdmi_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s channels->min %u channels->max %u ()\n", __func__,
			channels->min, channels->max);

	if (!hdmi_rate_variable)
		rate->min = rate->max = 48000;
	channels->min = channels->max = msm_hdmi_rx_ch;
	if (channels->max < 2)
		channels->min = channels->max = 2;

	return 0;
}

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link msm_dai[] = {
	/* FrontEnd DAI Links */
	{
		.name = "MultiMedia1 PCM",
		.stream_name = "MultiMedia1 Playback",
		.cpu_dai_name	= "MultiMedia1",
		.platform_name  = "soc:msm_pcm",
		.dynamic = 1,
		.dpcm_playback = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* this dainlink has playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA1,
		.be_hw_params_fixup = msm_hdmi_be_hw_params_fixup,

	},
	/* HDMI BACK END DAI Link */
	{
		.name = LPASS_BE_HDMI,
		.stream_name = "HDMI Playback",
		.cpu_dai_name = "HDMI",
		.platform_name = "soc:msm_pcm_routing",
		.codec_name = "hdmi-audio-codec.0.auto",
		.codec_dai_name = "i2s-hifi",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.be_id = MSM_BACKEND_DAI_HDMI_RX,
		.be_hw_params_fixup = msm_hdmi_be_hw_params_fixup,

	},
};

static struct snd_soc_card snd_soc_card_msm = {
	.name		= "apq8064-tabla-snd-card",
	.dai_link	= msm_dai,
	.num_links	= ARRAY_SIZE(msm_dai),
	.owner 		= THIS_MODULE,
	.controls = tabla_msm_controls,
	.num_controls = ARRAY_SIZE(tabla_msm_controls),
};

static int msm_snd_apq8064_probe(struct platform_device *pdev)
{
	int ret;

	snd_soc_card_msm.dev = &pdev->dev;
	ret = snd_soc_register_card(&snd_soc_card_msm);
	if (ret)
		dev_err(&pdev->dev, "Error: snd_soc_register_card failed (%d)!\n", ret);

	return ret;

}

static  int msm_snd_apq8064_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id msm_snd_apq8064_dt_match[] = {
	{.compatible = "qcom,snd-apq8064"},
	{}
};

static struct platform_driver msm_snd_apq8064_driver = {
	.probe  = msm_snd_apq8064_probe,
	.remove = msm_snd_apq8064_remove,
	.driver = {
		.name = "msm-snd-apq8064",
		.owner = THIS_MODULE,
		.of_match_table = msm_snd_apq8064_dt_match,
	},
};
module_platform_driver(msm_snd_apq8064_driver);
/* Module information */

MODULE_DESCRIPTION("ALSA SoC msm");
MODULE_LICENSE("GPL v2");

