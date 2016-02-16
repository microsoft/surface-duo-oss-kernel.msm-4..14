#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>

#include "msm8x16-wcd.h"
#include "msm8x16_wcd_registers.h"

#define MSM8X16_WCD_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000)
#define MSM8X16_WCD_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
		SNDRV_PCM_FMTBIT_S24_LE)

#define TOMBAK_VERSION_1_0	0
#define TOMBAK_IS_1_0(ver) \
	((ver == TOMBAK_VERSION_1_0) ? 1 : 0)

#define HPHL_PA_DISABLE (0x01 << 1)
#define HPHR_PA_DISABLE (0x01 << 2)
#define EAR_PA_DISABLE (0x01 << 3)
#define SPKR_PA_DISABLE (0x01 << 4)

#define MICBIAS_DEFAULT_VAL 1800000
#define MICBIAS_MIN_VAL 1600000
#define MICBIAS_STEP_SIZE 50000

#define DEFAULT_BOOST_VOLTAGE 5000
#define MIN_BOOST_VOLTAGE 4000
#define MAX_BOOST_VOLTAGE 5550
#define BOOST_VOLTAGE_STEP 50

#define VOLTAGE_CONVERTER(value, min_value, step_size)\
	((value - min_value)/step_size);

enum {
	AIF1_PB = 0,
	AIF1_CAP,
	NUM_CODEC_DAIS,
};

static unsigned long rx_digital_gain_reg[] = {
	MSM8X16_WCD_A_CDC_RX1_VOL_CTL_B2_CTL,
	MSM8X16_WCD_A_CDC_RX2_VOL_CTL_B2_CTL,
	MSM8X16_WCD_A_CDC_RX3_VOL_CTL_B2_CTL,
};

static unsigned long tx_digital_gain_reg[] = {
	MSM8X16_WCD_A_CDC_TX1_VOL_CTL_GAIN,
	MSM8X16_WCD_A_CDC_TX2_VOL_CTL_GAIN,
};

struct wcd_chip {
	struct regmap	*analog_map;
	struct regmap	*digital_map;
	unsigned int	analog_base;
	u16 pmic_rev;
	u16 codec_version;
	bool spk_boost_set;
	u32 mute_mask;
	u32 rx_bias_count;
	bool ear_pa_boost_set;
	bool lb_mode;
	struct clk *mclk;

	struct regulator *vddio;
	struct regulator *vdd_pa;
	struct regulator *vdd_px;
	struct regulator *vdd_cp;
	struct regulator *vdd_mic_bias;
};

static int msm8x16_wcd_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
	return msm8x16_wcd_reg_readonly[reg];
}

static int msm8x16_wcd_readable(struct snd_soc_codec *ssc, unsigned int reg)
{
	return msm8x16_wcd_reg_readable[reg];
}

static int __msm8x16_wcd_reg_write(struct snd_soc_codec *codec,
			unsigned short reg, u8 val)
{
	int ret = -EINVAL;
	struct wcd_chip *chip = dev_get_drvdata(codec->dev);

	if (MSM8X16_WCD_IS_TOMBAK_REG(reg)) {
		ret = regmap_write(chip->analog_map,
				   chip->analog_base + reg, val);
	} else if (MSM8X16_WCD_IS_DIGITAL_REG(reg)) {
		u32 temp = val & 0x000000FF;
		u16 offset = (reg ^ 0x0200) & 0x0FFF;

		ret = regmap_write(chip->digital_map, offset, temp);
	}

	return ret;
}

static int msm8x16_wcd_write(struct snd_soc_codec *codec, unsigned int reg,
			     unsigned int value)
{
	if (reg == SND_SOC_NOPM)
		return 0;

	BUG_ON(reg > MSM8X16_WCD_MAX_REGISTER);
	if (!msm8x16_wcd_volatile(codec, reg))
		msm8x16_wcd_reset_reg_defaults[reg] = value;

	return __msm8x16_wcd_reg_write(codec, reg, (u8)value);
}

static int __msm8x16_wcd_reg_read(struct snd_soc_codec *codec,
				unsigned short reg)
{
	int ret = -EINVAL;
	u32 temp = 0;
	struct wcd_chip *chip = dev_get_drvdata(codec->dev);

	if (MSM8X16_WCD_IS_TOMBAK_REG(reg)) {
		ret = regmap_read(chip->analog_map,
				  chip->analog_base + reg, &temp);
	} else if (MSM8X16_WCD_IS_DIGITAL_REG(reg)) {
		u32 val;
		u16 offset = (reg ^ 0x0200) & 0x0FFF;

		ret = regmap_read(chip->digital_map, offset, &val);
		temp = (u8)val;
	}

	if (ret < 0) {
		dev_err(codec->dev,
				"%s: codec read failed for reg 0x%x\n",
				__func__, reg);
		return ret;
	}

	dev_dbg(codec->dev, "Read 0x%02x from 0x%x\n", temp, reg);

	return temp;
}

static unsigned int msm8x16_wcd_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	unsigned int val;

	if (reg == SND_SOC_NOPM)
		return 0;

	BUG_ON(reg > MSM8X16_WCD_MAX_REGISTER);

	if (!msm8x16_wcd_volatile(codec, reg) &&
	    msm8x16_wcd_readable(codec, reg) &&
		reg < codec->driver->reg_cache_size) {
		return msm8x16_wcd_reset_reg_defaults[reg];
	}

	val = __msm8x16_wcd_reg_read(codec, reg);

	return val;
}

static const struct msm8x16_wcd_reg_mask_val msm8x16_wcd_reg_defaults[] = {
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_SPKR_DAC_CTL, 0x03),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_CURRENT_LIMIT, 0x82),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_SPKR_OCP_CTL, 0xE1),
};

static const struct msm8x16_wcd_reg_mask_val msm8x16_wcd_reg_defaults_2_0[] = {
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_DIGITAL_PERPH_RESET_CTL3, 0x0F),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_TX_1_2_OPAMP_BIAS, 0x4B),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_NCP_FBCTRL, 0x28),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_SPKR_DRV_CTL, 0x69),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_SPKR_DRV_DBG, 0x01),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_BOOST_EN_CTL, 0x5F),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_SLOPE_COMP_IP_ZERO, 0x88),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_PERPH_RESET_CTL3, 0x0F),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_CURRENT_LIMIT, 0x82),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_SPKR_DAC_CTL, 0x03),
	MSM8X16_WCD_REG_VAL(MSM8X16_WCD_A_ANALOG_SPKR_OCP_CTL, 0xE1),
};

static int msm8x16_wcd_bringup(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, MSM8X16_WCD_A_DIGITAL_PERPH_RESET_CTL4, 0x01);
	snd_soc_write(codec, MSM8X16_WCD_A_ANALOG_PERPH_RESET_CTL4, 0x01);
	return 0;
}

static const struct msm8x16_wcd_reg_mask_val
	msm8x16_wcd_codec_reg_init_val[] = {

	/* Initialize current threshold to 350MA
	 * number of wait and run cycles to 4096
	 */
	{MSM8X16_WCD_A_ANALOG_RX_COM_OCP_CTL, 0xFF, 0xD1},
	{MSM8X16_WCD_A_ANALOG_RX_COM_OCP_COUNT, 0xFF, 0xFF},
};

static void msm8x16_wcd_codec_init_reg(struct snd_soc_codec *codec)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(msm8x16_wcd_codec_reg_init_val); i++)
		snd_soc_update_bits(codec,
				    msm8x16_wcd_codec_reg_init_val[i].reg,
				    msm8x16_wcd_codec_reg_init_val[i].mask,
				    msm8x16_wcd_codec_reg_init_val[i].val);
}

static void msm8x16_wcd_update_reg_defaults(struct snd_soc_codec *codec)
{
	u32 i;
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);

	if (TOMBAK_IS_1_0(msm8x16_wcd->pmic_rev)) {
		for (i = 0; i < ARRAY_SIZE(msm8x16_wcd_reg_defaults); i++)
			snd_soc_write(codec, msm8x16_wcd_reg_defaults[i].reg,
					msm8x16_wcd_reg_defaults[i].val);
	} else {
		for (i = 0; i < ARRAY_SIZE(msm8x16_wcd_reg_defaults_2_0); i++)
			snd_soc_write(codec,
				msm8x16_wcd_reg_defaults_2_0[i].reg,
				msm8x16_wcd_reg_defaults_2_0[i].val);
	}
}

static int msm8x16_wcd_device_up(struct snd_soc_codec *codec)
{
	u32 reg;

	dev_dbg(codec->dev, "%s: device up!\n", __func__);
	msm8x16_wcd_bringup(codec);

	for (reg = 0; reg < ARRAY_SIZE(msm8x16_wcd_reset_reg_defaults); reg++)
		if (msm8x16_wcd_reg_readable[reg])
			msm8x16_wcd_write(codec,
				reg, msm8x16_wcd_reset_reg_defaults[reg]);

	/* delay is required to make sure sound card state updated */
	usleep_range(5000, 5100);

	msm8x16_wcd_codec_init_reg(codec);
	msm8x16_wcd_update_reg_defaults(codec);

	return 0;
}

static int msm8x16_wcd_codec_enable_clock_block(struct snd_soc_codec *codec,
						int enable)
{
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);
	unsigned long mclk_rate;

	if (enable) {
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_CDC_CLK_MCLK_CTL, 0x01, 0x01);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_CDC_CLK_PDM_CTL, 0x03, 0x03);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_ANALOG_MASTER_BIAS_CTL, 0x30, 0x30);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_RST_CTL, 0x80, 0x80);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_TOP_CLK_CTL, 0x0C, 0x0C);

		mclk_rate = clk_get_rate(msm8x16_wcd->mclk);

		if (mclk_rate == 12288000)
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_CDC_TOP_CTL, 0x01, 0x00);
		else if (mclk_rate == 9600000)
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_CDC_TOP_CTL, 0x01, 0x01);
	} else {
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_TOP_CLK_CTL, 0x0C, 0x00);
		snd_soc_update_bits(codec,
				MSM8X16_WCD_A_CDC_CLK_PDM_CTL, 0x03, 0x00);

	}
	return 0;
}

#define MICBIAS_EXT_BYP_CAP 0x00
#define MICBIAS_NO_EXT_BYP_CAP 0x01

static void msm8x16_wcd_configure_cap(struct snd_soc_codec *codec,
		bool micbias1, bool micbias2)
{

//	struct msm8916_asoc_mach_data *pdata = NULL;
//FIXME should come from DT
	int micbias1_cap_mode = MICBIAS_EXT_BYP_CAP, micbias2_cap_mode = MICBIAS_NO_EXT_BYP_CAP;

	//pdata = snd_soc_card_get_drvdata(codec->card);

	pr_debug("\n %s: micbias1 %x micbias2 = %d\n", __func__, micbias1,
			micbias2);
	if (micbias1 && micbias2) {
		if ((micbias1_cap_mode
		     == MICBIAS_EXT_BYP_CAP) ||
		    (micbias2_cap_mode
		     == MICBIAS_EXT_BYP_CAP))
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_MICB_1_EN,
				0x40, (MICBIAS_EXT_BYP_CAP << 6));
		else
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_MICB_1_EN,
				0x40, (MICBIAS_NO_EXT_BYP_CAP << 6));
	} else if (micbias2) {
		snd_soc_update_bits(codec, MSM8X16_WCD_A_ANALOG_MICB_1_EN,
				0x40, (micbias2_cap_mode << 6));
	} else if (micbias1) {
		snd_soc_update_bits(codec, MSM8X16_WCD_A_ANALOG_MICB_1_EN,
				0x40, (micbias1_cap_mode << 6));
	} else {
		snd_soc_update_bits(codec, MSM8X16_WCD_A_ANALOG_MICB_1_EN,
				0x40, 0x00);
	}
}

static int msm8x16_wcd_codec_probe(struct snd_soc_codec *codec)
{
	struct wcd_chip *chip = dev_get_drvdata(codec->dev);
	int err;

	snd_soc_codec_set_drvdata(codec, chip);
	chip->pmic_rev = snd_soc_read(codec, MSM8X16_WCD_A_DIGITAL_REVISION1);
	dev_info(codec->dev, "%s :PMIC REV: %d", __func__,
					chip->pmic_rev);

	chip->codec_version = snd_soc_read(codec,
			MSM8X16_WCD_A_DIGITAL_PERPH_SUBTYPE);
	dev_info(codec->dev, "%s :CODEC Version: %d", __func__,
				chip->codec_version);

	msm8x16_wcd_device_up(codec);

	/* Set initial cap mode */
	msm8x16_wcd_configure_cap(codec, false, false);

	regulator_set_voltage(chip->vddio, 1800000, 1800000);
	err = regulator_enable(chip->vddio);
	if (err < 0) {
		dev_err(codec->dev, "failed to enable VDD regulator\n");
		return err;
	}
	regulator_set_voltage(chip->vdd_pa, 1800000, 2200000);
	err = regulator_enable(chip->vdd_pa);
	if (err < 0) {
		dev_err(codec->dev, "failed to enable VDD regulator\n");
		return err;
	}

	regulator_set_voltage(chip->vdd_mic_bias, 3075000, 3075000);
	err = regulator_enable(chip->vdd_mic_bias);
	if (err < 0) {
		dev_err(codec->dev, "failed to enable micbias regulator\n");
		return err;
	}
	msm8x16_wcd_codec_enable_clock_block(codec, 1);

	return 0;
}

static int msm8x16_wcd_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	dev_dbg(dai->codec->dev, "%s(): substream = %s  stream = %d\n",
		__func__,
		substream->name, substream->stream);
	return 0;
}

static void msm8x16_wcd_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	dev_dbg(dai->codec->dev,
		"%s(): substream = %s  stream = %d\n", __func__,
		substream->name, substream->stream);
}

static int msm8x16_wcd_set_interpolator_rate(struct snd_soc_dai *dai,
	u8 rx_fs_rate_reg_val, u32 sample_rate)
{
	return 0;
}

static int msm8x16_wcd_set_decimator_rate(struct snd_soc_dai *dai,
	u8 tx_fs_rate_reg_val, u32 sample_rate)
{

	return 0;
}

static int msm8x16_wcd_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	u8 tx_fs_rate, rx_fs_rate;
	int ret;

	dev_err(dai->codec->dev,
		"%s: dai_name = %s DAI-ID %x rate %d num_ch %d format %d\n",
		__func__, dai->name, dai->id, params_rate(params),
		params_channels(params), params_format(params));

	switch (params_rate(params)) {
	case 8000:
		tx_fs_rate = 0x00;
		rx_fs_rate = 0x00;
		break;
	case 16000:
		tx_fs_rate = 0x01;
		rx_fs_rate = 0x20;
		break;
	case 32000:
		tx_fs_rate = 0x02;
		rx_fs_rate = 0x40;
		break;
	case 48000:
		tx_fs_rate = 0x03;
		rx_fs_rate = 0x60;
		break;
	case 96000:
		tx_fs_rate = 0x04;
		rx_fs_rate = 0x80;
		break;
	case 192000:
		tx_fs_rate = 0x05;
		rx_fs_rate = 0xA0;
		break;
	default:
		dev_err(dai->codec->dev,
			"%s: Invalid sampling rate %d\n", __func__,
			params_rate(params));
		return -EINVAL;
	}

	switch (substream->stream) {
	case SNDRV_PCM_STREAM_CAPTURE:
		snd_soc_update_bits(dai->codec,
				MSM8X16_WCD_A_CDC_CLK_TX_I2S_CTL, 0x07, tx_fs_rate);
		ret = msm8x16_wcd_set_decimator_rate(dai, tx_fs_rate,
					       params_rate(params));
		if (ret < 0) {
			dev_err(dai->codec->dev,
				"%s: set decimator rate failed %d\n", __func__,
				ret);
			return ret;
		}
		break;
	case SNDRV_PCM_STREAM_PLAYBACK:
		ret = msm8x16_wcd_set_interpolator_rate(dai, rx_fs_rate,
						  params_rate(params));
		if (ret < 0) {
			dev_err(dai->codec->dev,
				"%s: set decimator rate failed %d\n", __func__,
				ret);
			return ret;
		}
		break;
	default:
		dev_err(dai->codec->dev,
			"%s: Invalid stream type %d\n", __func__,
			substream->stream);
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		snd_soc_update_bits(dai->codec,
				MSM8X16_WCD_A_CDC_CLK_TX_I2S_CTL, 0x20, 0x20);
		snd_soc_update_bits(dai->codec,
				MSM8X16_WCD_A_CDC_CLK_RX_I2S_CTL, 0x20, 0x20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		snd_soc_update_bits(dai->codec,
				MSM8X16_WCD_A_CDC_CLK_TX_I2S_CTL, 0x20, 0x00);
		snd_soc_update_bits(dai->codec,
				MSM8X16_WCD_A_CDC_CLK_RX_I2S_CTL, 0x20, 0x00);
		break;
	default:
		dev_err(dai->dev, "%s: wrong format selected\n",
				__func__);
		return -EINVAL;
	}

	return 0;
}

static int msm8x16_wcd_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	dev_dbg(dai->codec->dev, "%s\n", __func__);
	return 0;
}

static int msm8x16_wcd_set_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)

{
	dev_dbg(dai->codec->dev, "%s\n", __func__);
	return 0;
}

static int msm8x16_wcd_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	dev_dbg(dai->codec->dev, "%s\n", __func__);

	return 0;
}

static struct snd_soc_dai_ops msm8x16_wcd_dai_ops = {
	.startup = msm8x16_wcd_startup,
	.shutdown = msm8x16_wcd_shutdown,
	.hw_params = msm8x16_wcd_hw_params,
	.set_sysclk = msm8x16_wcd_set_dai_sysclk,
	.set_fmt = msm8x16_wcd_set_dai_fmt,
	.set_channel_map = msm8x16_wcd_set_channel_map,
};

static struct snd_soc_dai_driver msm8x16_wcd_codec_dai[] = {
	[0] = {
		.name = "msm8x16_wcd_i2s_rx1",
		.id = AIF1_PB,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = MSM8X16_WCD_RATES,
			.formats = MSM8X16_WCD_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 3,
		},
		.ops = &msm8x16_wcd_dai_ops,
	},
	[1] = {
		.name = "msm8x16_wcd_i2s_tx1",
		.id = AIF1_CAP,
		.capture = {
			.stream_name = "AIF1 Capture",
			.rates = MSM8X16_WCD_RATES,
			.formats = MSM8X16_WCD_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &msm8x16_wcd_dai_ops,
	},
};

static int msm8x16_wcd_codec_remove(struct snd_soc_codec *codec)
{
	/* TODO */
	return 0;
};

static int msm8x16_wcd_spk_boost_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct wcd_chip *msm8x16_wcd = dev_get_drvdata(codec->dev);

	if (msm8x16_wcd->spk_boost_set == false) {
		ucontrol->value.integer.value[0] = 0;
	} else if (msm8x16_wcd->spk_boost_set == true) {
		ucontrol->value.integer.value[0] = 1;
	} else  {
		dev_err(codec->dev, "%s: ERROR: Unsupported Speaker Boost = %d\n",
			__func__, msm8x16_wcd->spk_boost_set);
		return -EINVAL;
	}

	dev_dbg(codec->dev, "%s: msm8x16_wcd->spk_boost_set = %d\n", __func__,
			msm8x16_wcd->spk_boost_set);
	return 0;
}

static int msm8x16_wcd_spk_boost_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);

	switch (ucontrol->value.integer.value[0]) {
	case 0:
		msm8x16_wcd->spk_boost_set = false;
		break;
	case 1:
		msm8x16_wcd->spk_boost_set = true;
		break;
	default:
		return -EINVAL;
	}
	dev_dbg(codec->dev, "%s: msm8x16_wcd->spk_boost_set = %d\n",
		__func__, msm8x16_wcd->spk_boost_set);
	return 0;
}

static const char * const hph_text[] = {
	"ZERO", "Switch",
};

static const struct soc_enum hph_enum =
	SOC_ENUM_SINGLE(0, 0, ARRAY_SIZE(hph_text), hph_text);

static const struct snd_kcontrol_new hphl_mux[] = {
	SOC_DAPM_ENUM("HPHL", hph_enum)
};

static const struct snd_kcontrol_new hphr_mux[] = {
	SOC_DAPM_ENUM("HPHR", hph_enum)
};

static const struct snd_kcontrol_new spkr_switch[] = {
	SOC_DAPM_SINGLE("Switch",
		MSM8X16_WCD_A_ANALOG_SPKR_DAC_CTL, 7, 1, 0)
};

static void msm8x16_wcd_codec_enable_adc_block(struct snd_soc_codec *codec,
					 int enable)
{
	//struct msm8x16_wcd_priv *wcd8x16 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s %d\n", __func__, enable);

	if (enable) {
		//wcd8x16->adc_count++;
		snd_soc_update_bits(codec,
				    MSM8X16_WCD_A_DIGITAL_CDC_ANA_CLK_CTL,
				    0x20, 0x20);
		snd_soc_update_bits(codec,
				    MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
				    0x10, 0x10);
	} else {
		//wcd8x16->adc_count--;
		//if (!wcd8x16->adc_count) {
			snd_soc_update_bits(codec,
				    MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
				    0x10, 0x00);
			snd_soc_update_bits(codec,
				    MSM8X16_WCD_A_DIGITAL_CDC_ANA_CLK_CTL,
					    0x20, 0x0);
		//}
	}
}

static const DECLARE_TLV_DB_SCALE(digital_gain, 0, 1, 0);
static const DECLARE_TLV_DB_SCALE(analog_gain, 0, 25, 1);

static const char * const rx_mix1_text[] = {
	"ZERO", "IIR1", "IIR2", "RX1", "RX2", "RX3"
};

static const char * const rx_mix2_text[] = {
	"ZERO", "IIR1", "IIR2"
};

static const char * const dec_mux_text[] = {
	"ZERO", "ADC1", "ADC2", "ADC3", "DMIC1", "DMIC2"
};

static const char * const adc2_mux_text[] = {
	"ZERO", "INP2", "INP3"
};

static const char * const rdac2_mux_text[] = {
	"ZERO", "RX2", "RX1"
};

static const char * const iir_inp1_text[] = {
	"ZERO", "DEC1", "DEC2", "RX1", "RX2", "RX3"
};

static const char * const iir1_inp1_text[] = {
	"ZERO", "DEC1", "DEC2", "RX1", "RX2", "RX3"
};

static const struct soc_enum adc2_enum =
	SOC_ENUM_SINGLE(0, 0, ARRAY_SIZE(adc2_mux_text), adc2_mux_text);

/* RX1 MIX1 */
static const struct soc_enum rx_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_RX1_B1_CTL,
		0, 6, rx_mix1_text);

static const struct soc_enum rx_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_RX1_B1_CTL,
		3, 6, rx_mix1_text);

static const struct soc_enum rx_mix1_inp3_chain_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_RX1_B2_CTL,
		0, 6, rx_mix1_text);
/* RX1 MIX2 */
static const struct soc_enum rx_mix2_inp1_chain_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_RX1_B3_CTL,
		0, 3, rx_mix2_text);

/* RX2 MIX1 */
static const struct soc_enum rx2_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_RX2_B1_CTL,
		0, 6, rx_mix1_text);

static const struct soc_enum rx2_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_RX2_B1_CTL,
		3, 6, rx_mix1_text);

static const struct soc_enum rx2_mix1_inp3_chain_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_RX2_B1_CTL,
		0, 6, rx_mix1_text);

/* RX2 MIX2 */
static const struct soc_enum rx2_mix2_inp1_chain_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_RX2_B3_CTL,
		0, 3, rx_mix2_text);

/* RX3 MIX1 */
static const struct soc_enum rx3_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_RX3_B1_CTL,
		0, 6, rx_mix1_text);

static const struct soc_enum rx3_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_RX3_B1_CTL,
		3, 6, rx_mix1_text);

/* DEC */
static const struct soc_enum dec1_mux_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_TX_B1_CTL,
		0, 6, dec_mux_text);

static const struct soc_enum dec2_mux_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_TX_B1_CTL,
		3, 6, dec_mux_text);

static const struct soc_enum rdac2_mux_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_DIGITAL_CDC_CONN_HPHR_DAC_CTL,
		0, 3, rdac2_mux_text);

static const struct soc_enum iir1_inp1_mux_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_EQ1_B1_CTL,
		0, 6, iir_inp1_text);

static const struct soc_enum iir2_inp1_mux_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_EQ2_B1_CTL,
		0, 6, iir_inp1_text);
static const struct snd_kcontrol_new iir2_inp1_mux =
	SOC_DAPM_ENUM("IIR2 INP1 Mux", iir2_inp1_mux_enum);

static const struct soc_enum rx3_mix1_inp3_chain_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_CONN_RX3_B1_CTL,
		0, 6, rx_mix1_text);
static const struct snd_kcontrol_new rx_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP1 Mux", rx_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new dec1_mux =
	SOC_DAPM_ENUM("DEC1 MUX Mux", dec1_mux_enum);

static const struct snd_kcontrol_new dec2_mux =
	SOC_DAPM_ENUM("DEC2 MUX Mux", dec2_mux_enum);

static const struct snd_kcontrol_new rdac2_mux =
	SOC_DAPM_ENUM("RDAC2 MUX Mux", rdac2_mux_enum);

static const struct snd_kcontrol_new iir1_inp1_mux =
	SOC_DAPM_ENUM("IIR1 INP1 Mux", iir1_inp1_mux_enum);

static const struct snd_kcontrol_new rx_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP2 Mux", rx_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx_mix1_inp3_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP3 Mux", rx_mix1_inp3_chain_enum);

static const struct snd_kcontrol_new rx2_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP1 Mux", rx2_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx2_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP2 Mux", rx2_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx2_mix1_inp3_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP3 Mux", rx2_mix1_inp3_chain_enum);

static const struct snd_kcontrol_new rx3_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP1 Mux", rx3_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx3_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP2 Mux", rx3_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx3_mix1_inp3_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP3 Mux", rx3_mix1_inp3_chain_enum);

static const struct snd_kcontrol_new rx1_mix2_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX2 INP1 Mux", rx_mix2_inp1_chain_enum);

static const struct snd_kcontrol_new rx2_mix2_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX2 INP1 Mux", rx2_mix2_inp1_chain_enum);

static const struct snd_kcontrol_new tx_adc2_mux =
	SOC_DAPM_ENUM("ADC2 MUX Mux", adc2_enum);

static const char * const msm8x16_wcd_loopback_mode_ctrl_text[] = {
		"DISABLE", "ENABLE"};
static const struct soc_enum msm8x16_wcd_loopback_mode_ctl_enum[] = {
		SOC_ENUM_SINGLE_EXT(2, msm8x16_wcd_loopback_mode_ctrl_text),
};

static int msm8x16_wcd_codec_enable_on_demand_supply(
		struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = regulator_enable(msm8x16_wcd->vdd_mic_bias);
		if (ret)
		dev_err(codec->dev, "%s: Failed to enable vdd micbias\n",
			__func__);
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = regulator_disable(msm8x16_wcd->vdd_mic_bias);
		if (ret)
			dev_err(codec->dev, "%s: Failed to disable vdd-micbias\n",
				__func__);
		break;
	default:
		break;
	}

	return ret;
}

static const char * const msm8x16_wcd_ear_pa_boost_ctrl_text[] = {
		"DISABLE", "ENABLE"};
static const struct soc_enum msm8x16_wcd_ear_pa_boost_ctl_enum[] = {
		SOC_ENUM_SINGLE_EXT(2, msm8x16_wcd_ear_pa_boost_ctrl_text),
};

static const char * const msm8x16_wcd_ear_pa_gain_text[] = {
		"POS_6_DB", "POS_1P5_DB"};
static const struct soc_enum msm8x16_wcd_ear_pa_gain_enum[] = {
		SOC_ENUM_SINGLE_EXT(2, msm8x16_wcd_ear_pa_gain_text),
};

static const char * const msm8x16_wcd_spk_boost_ctrl_text[] = {
		"DISABLE", "ENABLE"};
static const struct soc_enum msm8x16_wcd_spk_boost_ctl_enum[] = {
		SOC_ENUM_SINGLE_EXT(2, msm8x16_wcd_spk_boost_ctrl_text),
};

/*cut of frequency for high pass filter*/
static const char * const cf_text[] = {
	"MIN_3DB_4Hz", "MIN_3DB_75Hz", "MIN_3DB_150Hz"
};

static const struct soc_enum cf_dec1_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_TX1_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec2_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_TX2_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_rxmix1_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_RX1_B4_CTL, 0, 3, cf_text);

static const struct soc_enum cf_rxmix2_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_RX2_B4_CTL, 0, 3, cf_text);

static const struct soc_enum cf_rxmix3_enum =
	SOC_ENUM_SINGLE(MSM8X16_WCD_A_CDC_RX3_B4_CTL, 0, 3, cf_text);

static const struct snd_kcontrol_new msm8x16_wcd_snd_controls[] = {

	SOC_ENUM_EXT("Speaker Boost", msm8x16_wcd_spk_boost_ctl_enum[0],
		msm8x16_wcd_spk_boost_get, msm8x16_wcd_spk_boost_set),

	SOC_SINGLE_TLV("ADC1 Volume", MSM8X16_WCD_A_ANALOG_TX_1_EN, 3,
					8, 0, analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", MSM8X16_WCD_A_ANALOG_TX_2_EN, 3,
					8, 0, analog_gain),
	SOC_SINGLE_TLV("ADC3 Volume", MSM8X16_WCD_A_ANALOG_TX_3_EN, 3,
					8, 0, analog_gain),

	SOC_SINGLE_SX_TLV("RX1 Digital Volume",
			  MSM8X16_WCD_A_CDC_RX1_VOL_CTL_B2_CTL,
			0,  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX2 Digital Volume",
			  MSM8X16_WCD_A_CDC_RX2_VOL_CTL_B2_CTL,
			0,  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX3 Digital Volume",
			  MSM8X16_WCD_A_CDC_RX3_VOL_CTL_B2_CTL,
			0,  -84, 40, digital_gain),

	SOC_SINGLE("RX1 HPF Switch",
		MSM8X16_WCD_A_CDC_RX1_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX2 HPF Switch",
		MSM8X16_WCD_A_CDC_RX2_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX3 HPF Switch",
		MSM8X16_WCD_A_CDC_RX3_B5_CTL, 2, 1, 0),

	SOC_ENUM("RX1 HPF cut off", cf_rxmix1_enum),
	SOC_ENUM("RX2 HPF cut off", cf_rxmix2_enum),
	SOC_ENUM("RX3 HPF cut off", cf_rxmix3_enum),
};

static const struct snd_kcontrol_new ear_pa_switch[] = {
	SOC_DAPM_SINGLE("Switch",
		MSM8X16_WCD_A_ANALOG_RX_EAR_CTL, 5, 1, 0)
};

static int msm8x16_wcd_codec_enable_ear_pa(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dev_dbg(codec->dev,
			"%s: Sleeping 20ms after select EAR PA\n",
			__func__);
		snd_soc_update_bits(codec, MSM8X16_WCD_A_ANALOG_RX_EAR_CTL,
			    0x80, 0x80);
		break;
	case SND_SOC_DAPM_POST_PMU:
		dev_dbg(codec->dev,
			"%s: Sleeping 20ms after enabling EAR PA\n",
			__func__);
		snd_soc_update_bits(codec, MSM8X16_WCD_A_ANALOG_RX_EAR_CTL,
			    0x40, 0x40);
		usleep_range(7000, 7100);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_CDC_RX1_B6_CTL, 0x01, 0x00);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_CDC_RX1_B6_CTL, 0x01, 0x01);
		msleep(20);
		msm8x16_wcd->mute_mask |= EAR_PA_DISABLE;
		break;
	case SND_SOC_DAPM_POST_PMD:
		dev_dbg(codec->dev,
			"%s: Sleeping 7ms after disabling EAR PA\n",
			__func__);
		snd_soc_update_bits(codec, MSM8X16_WCD_A_ANALOG_RX_EAR_CTL,
			    0x40, 0x00);
		usleep_range(7000, 7100);
		/*
		 * Reset pa select bit from ear to hph after ear pa
		 * is disabled to reduce ear turn off pop
		 */
		snd_soc_update_bits(codec, MSM8X16_WCD_A_ANALOG_RX_EAR_CTL,
			    0x80, 0x00);
		break;
	}
	return 0;
}

static int msm8x16_wcd_codec_enable_adc(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	//struct snd_soc_codec *codec = w->codec;
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	u16 adc_reg;
	u8 init_bit_shift;

	dev_dbg(codec->dev, "%s %d\n", __func__, event);

	adc_reg = MSM8X16_WCD_A_ANALOG_TX_1_2_TEST_CTL_2;

	if (w->reg == MSM8X16_WCD_A_ANALOG_TX_1_EN)
		init_bit_shift = 5;
	else if ((w->reg == MSM8X16_WCD_A_ANALOG_TX_2_EN) ||
		 (w->reg == MSM8X16_WCD_A_ANALOG_TX_3_EN))
		init_bit_shift = 4;
	else {
		dev_err(codec->dev, "%s: Error, invalid adc register\n",
			__func__);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		msm8x16_wcd_codec_enable_adc_block(codec, 1);
		if (w->reg == MSM8X16_WCD_A_ANALOG_TX_2_EN)
			snd_soc_update_bits(codec,
			MSM8X16_WCD_A_ANALOG_MICB_1_CTL, 0x02, 0x02);
		/*
		 * Add delay of 10 ms to give sufficient time for the voltage
		 * to shoot up and settle so that the txfe init does not
		 * happen when the input voltage is changing too much.
		 */
		usleep_range(10000, 10010);
		snd_soc_update_bits(codec, adc_reg, 1 << init_bit_shift,
				1 << init_bit_shift);
		if (w->reg == MSM8X16_WCD_A_ANALOG_TX_1_EN)
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_DIGITAL_CDC_CONN_TX1_CTL,
				0x03, 0x00);
		else if ((w->reg == MSM8X16_WCD_A_ANALOG_TX_2_EN) ||
			(w->reg == MSM8X16_WCD_A_ANALOG_TX_3_EN))
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_DIGITAL_CDC_CONN_TX2_CTL,
				0x03, 0x00);
		usleep_range(CODEC_DELAY_1_MS, CODEC_DELAY_1_1_MS);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/*
		 * Add delay of 12 ms before deasserting the init
		 * to reduce the tx pop
		 */
	usleep_range(12000, 12010);
		snd_soc_update_bits(codec, adc_reg, 1 << init_bit_shift, 0x00);
		usleep_range(CODEC_DELAY_1_MS, CODEC_DELAY_1_1_MS);
		break;
	case SND_SOC_DAPM_POST_PMD:
		msm8x16_wcd_codec_enable_adc_block(codec, 0);
		if (w->reg == MSM8X16_WCD_A_ANALOG_TX_2_EN)
			snd_soc_update_bits(codec,
			MSM8X16_WCD_A_ANALOG_MICB_1_CTL, 0x02, 0x00);
		if (w->reg == MSM8X16_WCD_A_ANALOG_TX_1_EN)
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_DIGITAL_CDC_CONN_TX1_CTL,
				0x03, 0x02);
		else if ((w->reg == MSM8X16_WCD_A_ANALOG_TX_2_EN) ||
			(w->reg == MSM8X16_WCD_A_ANALOG_TX_3_EN))
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_DIGITAL_CDC_CONN_TX2_CTL,
				0x03, 0x02);

		break;
	}
	return 0;
}

static int msm8x16_wcd_codec_enable_spk_pa(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_ANA_CLK_CTL, 0x10, 0x10);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_ANALOG_SPKR_PWRSTG_CTL, 0x01, 0x01);
		if (!msm8x16_wcd->spk_boost_set)
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_SPKR_DAC_CTL, 0x10, 0x10);
		usleep_range(CODEC_DELAY_1_MS, CODEC_DELAY_1_1_MS);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_ANALOG_SPKR_PWRSTG_CTL, 0xE0, 0xE0);
		if (!TOMBAK_IS_1_0(msm8x16_wcd->pmic_rev))
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_RX_EAR_CTL, 0x01, 0x01);
		break;
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(CODEC_DELAY_1_MS, CODEC_DELAY_1_1_MS);
		if (msm8x16_wcd->spk_boost_set)
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_SPKR_DRV_CTL, 0xEF, 0xEF);
		else
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_SPKR_DAC_CTL, 0x10, 0x00);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_CDC_RX3_B6_CTL, 0x01, 0x00);
		snd_soc_update_bits(codec, w->reg, 0x80, 0x80);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_CDC_RX3_B6_CTL, 0x01, 0x01);
		msleep(20);
		msm8x16_wcd->mute_mask |= SPKR_PA_DISABLE;
		snd_soc_update_bits(codec, w->reg, 0x80, 0x00);
		if (msm8x16_wcd->spk_boost_set)
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_SPKR_DRV_CTL, 0xEF, 0x00);
		else
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_SPKR_DAC_CTL, 0x10, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_ANALOG_SPKR_PWRSTG_CTL, 0xE0, 0x00);
		if (!TOMBAK_IS_1_0(msm8x16_wcd->pmic_rev))
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_RX_EAR_CTL, 0x01, 0x00);
		usleep_range(CODEC_DELAY_1_MS, CODEC_DELAY_1_1_MS);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_ANALOG_SPKR_PWRSTG_CTL, 0x01, 0x00);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_ANA_CLK_CTL, 0x10, 0x00);
		break;
	}
	return 0;
}

static void msm8x16_wcd_micbias_2_enable(struct snd_soc_codec *codec, bool on)
{
	if (on) {
		snd_soc_update_bits(codec, MSM8X16_WCD_A_ANALOG_MICB_1_CTL,
					0x60, 0x60);
		snd_soc_write(codec, MSM8X16_WCD_A_ANALOG_MICB_1_VAL,
					0xC0);
		/*
		 * Special headset needs MICBIAS as 2.7V so wait for
		 * 50 msec for the MICBIAS to reach 2.7 volts.
		 */
		msleep(50);
		snd_soc_update_bits(codec, MSM8X16_WCD_A_ANALOG_MICB_1_CTL,
				0x60, 0x00);
	}
}

static s32 g_dmic_clk_cnt;
static int msm8x16_wcd_codec_enable_dmic(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	u8  dmic_clk_en;
	u16 dmic_clk_reg;
	s32 *dmic_clk_cnt;
	unsigned int dmic;
	int ret;
	char *dec_num = strpbrk(w->name, "12");

	if (dec_num == NULL) {
		dev_err(codec->dev, "%s: Invalid DMIC\n", __func__);
		return -EINVAL;
	}

	ret = kstrtouint(dec_num, 10, &dmic);
	if (ret < 0) {
		dev_err(codec->dev,
			"%s: Invalid DMIC line on the codec\n", __func__);
		return -EINVAL;
	}

	switch (dmic) {
	case 1:
	case 2:
		dmic_clk_en = 0x01;
		dmic_clk_cnt = &g_dmic_clk_cnt;
		dmic_clk_reg = MSM8X16_WCD_A_CDC_CLK_DMIC_B1_CTL;
		dev_dbg(codec->dev,
			"%s() event %d DMIC%d dmic_1_2_clk_cnt %d\n",
			__func__, event,  dmic, *dmic_clk_cnt);
		break;
	default:
		dev_err(codec->dev, "%s: Invalid DMIC Selection\n", __func__);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		(*dmic_clk_cnt)++;
		if (*dmic_clk_cnt == 1) {
			snd_soc_update_bits(codec, dmic_clk_reg,
					0x0E, 0x02);
			snd_soc_update_bits(codec, dmic_clk_reg,
					dmic_clk_en, dmic_clk_en);
		}
		if (dmic == 1)
			snd_soc_update_bits(codec,
			MSM8X16_WCD_A_CDC_TX1_DMIC_CTL, 0x07, 0x01);
		if (dmic == 2)
			snd_soc_update_bits(codec,
			MSM8X16_WCD_A_CDC_TX2_DMIC_CTL, 0x07, 0x01);
		break;
	case SND_SOC_DAPM_POST_PMD:
		(*dmic_clk_cnt)--;
		if (*dmic_clk_cnt  == 0)
			snd_soc_update_bits(codec, dmic_clk_reg,
					dmic_clk_en, 0);
		break;
	}
	return 0;
}

static int msm8x16_wcd_codec_enable_micbias(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);
	u16 micb_int_reg;
	char *internal1_text = "Internal1";
	char *internal2_text = "Internal2";
	char *internal3_text = "Internal3";
	char *external2_text = "External2";
	char *external_text = "External";
	bool micbias2;

	switch (w->reg) {
	case MSM8X16_WCD_A_ANALOG_MICB_1_EN:
	case MSM8X16_WCD_A_ANALOG_MICB_2_EN:
		micb_int_reg = MSM8X16_WCD_A_ANALOG_MICB_1_INT_RBIAS;
		break;
	default:
		dev_err(codec->dev,
			"%s: Error, invalid micbias register 0x%x\n",
			__func__, w->reg);
		return -EINVAL;
	}

	micbias2 = (snd_soc_read(codec, MSM8X16_WCD_A_ANALOG_MICB_2_EN) & 0x80);
	
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (strnstr(w->name, internal1_text, 30)) {
			snd_soc_update_bits(codec, micb_int_reg, 0x80, 0x80);
		} else if (strnstr(w->name, internal2_text, 30)) {
			snd_soc_update_bits(codec, micb_int_reg, 0x10, 0x10);
			snd_soc_update_bits(codec, w->reg, 0x60, 0x00);
		} else if (strnstr(w->name, internal3_text, 30)) {
			snd_soc_update_bits(codec, micb_int_reg, 0x2, 0x2);
		}
		if (!strnstr(w->name, external_text, 30))
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_MICB_1_EN, 0x05, 0x04);
		if (w->reg == MSM8X16_WCD_A_ANALOG_MICB_1_EN)
			msm8x16_wcd_configure_cap(codec, true, micbias2);

		break;
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(20000, 20100);
		if (strnstr(w->name, internal1_text, 30)) {
			snd_soc_update_bits(codec, micb_int_reg, 0x40, 0x40);
		} else if (strnstr(w->name, internal2_text, 30)) {
			snd_soc_update_bits(codec, micb_int_reg, 0x08, 0x08);
			msm8x16_wcd_micbias_2_enable(codec, true);

			msm8x16_wcd_configure_cap(codec, false, true);
			regmap_write(msm8x16_wcd->analog_map, 0xf144, 0x95);
		} else if (strnstr(w->name, internal3_text, 30)) {
			snd_soc_update_bits(codec, micb_int_reg, 0x01, 0x01);
		}  else if (strnstr(w->name, external2_text, 30)) {
			msm8x16_wcd_micbias_2_enable(codec, true);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (strnstr(w->name, internal1_text, 30)) {
			snd_soc_update_bits(codec, micb_int_reg, 0xC0, 0x40);
		} else if (strnstr(w->name, internal2_text, 30)) {
			msm8x16_wcd_micbias_2_enable(codec, false);
		} else if (strnstr(w->name, internal3_text, 30)) {
			snd_soc_update_bits(codec, micb_int_reg, 0x2, 0x0);
		} else if (strnstr(w->name, external2_text, 30)) {
			/*
			 * send micbias turn off event to mbhc driver and then
			 * break, as no need to set MICB_1_EN register.
			 */
			msm8x16_wcd_micbias_2_enable(codec, false);
			break;
		}
		if (w->reg == MSM8X16_WCD_A_ANALOG_MICB_1_EN)
			msm8x16_wcd_configure_cap(codec, false, micbias2);
		break;
	}

	return 0;
}

#define  TX_MUX_CTL_CUT_OFF_FREQ_MASK	0x30
#define  CF_MIN_3DB_4HZ			0x0
#define  CF_MIN_3DB_75HZ		0x1
#define  CF_MIN_3DB_150HZ		0x2

static int msm8x16_wcd_codec_enable_dec(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	unsigned int decimator;
	char *dec_name = NULL;
	char *widget_name = NULL;
	char *temp;
	int ret = 0;
	u16 dec_reset_reg, tx_vol_ctl_reg, tx_mux_ctl_reg;
	u8 dec_hpf_cut_of_freq;
	int offset;
	char *dec_num;
	dev_dbg(codec->dev, "%s %d\n", __func__, event);

	widget_name = kstrndup(w->name, 15, GFP_KERNEL);
	if (!widget_name)
		return -ENOMEM;
	temp = widget_name;

	dec_name = strsep(&widget_name, " ");
	widget_name = temp;
	if (!dec_name) {
		dev_err(codec->dev,
			"%s: Invalid decimator = %s\n", __func__, w->name);
		ret = -EINVAL;
		goto out;
	}

	dec_num = strpbrk(dec_name, "12");
	if (dec_num == NULL) {
		dev_err(codec->dev, "%s: Invalid Decimator\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	ret = kstrtouint(dec_num, 10, &decimator);
	if (ret < 0) {
		dev_err(codec->dev,
			"%s: Invalid decimator = %s\n", __func__, dec_name);
		ret = -EINVAL;
		goto out;
	}

	dev_err(codec->dev,
		"%s(): widget = %s dec_name = %s decimator = %u\n", __func__,
		w->name, dec_name, decimator);

	if (w->reg == MSM8X16_WCD_A_CDC_CLK_TX_CLK_EN_B1_CTL) {
		dec_reset_reg = MSM8X16_WCD_A_CDC_CLK_TX_RESET_B1_CTL;
		offset = 0;
	} else {
		dev_err(codec->dev, "%s: Error, incorrect dec\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	tx_vol_ctl_reg = MSM8X16_WCD_A_CDC_TX1_VOL_CTL_CFG +
			 32 * (decimator - 1);
	tx_mux_ctl_reg = MSM8X16_WCD_A_CDC_TX1_MUX_CTL +
			  32 * (decimator - 1);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Enableable TX digital mute */
		snd_soc_update_bits(codec, tx_vol_ctl_reg, 0x01, 0x01);
		dec_hpf_cut_of_freq = snd_soc_read(codec, tx_mux_ctl_reg);
		dec_hpf_cut_of_freq = (dec_hpf_cut_of_freq & 0x30) >> 4;
		if ((dec_hpf_cut_of_freq != CF_MIN_3DB_150HZ)) {

			/* set cut of freq to CF_MIN_3DB_150HZ (0x1); */
			snd_soc_update_bits(codec, tx_mux_ctl_reg, 0x30,
					    CF_MIN_3DB_150HZ << 4);
		}
		snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_TX_1_2_TXFE_CLKDIV,
				0xFF, 0x42);

		break;
	case SND_SOC_DAPM_POST_PMU:
		/* enable HPF */
		snd_soc_update_bits(codec, tx_mux_ctl_reg , 0x08, 0x00);
		/* apply the digital gain after the decimator is enabled*/
		if ((w->shift) < ARRAY_SIZE(tx_digital_gain_reg))
			snd_soc_write(codec,
				  tx_digital_gain_reg[w->shift + offset],
				  snd_soc_read(codec,
				  tx_digital_gain_reg[w->shift + offset])
				  );
		snd_soc_update_bits(codec, tx_vol_ctl_reg, 0x01, 0x00);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, tx_vol_ctl_reg, 0x01, 0x01);
		msleep(20);
		snd_soc_update_bits(codec, tx_mux_ctl_reg, 0x08, 0x08);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, dec_reset_reg, 1 << w->shift,
			1 << w->shift);
		snd_soc_update_bits(codec, dec_reset_reg, 1 << w->shift, 0x0);
		snd_soc_update_bits(codec, tx_mux_ctl_reg, 0x08, 0x08);
		snd_soc_update_bits(codec, tx_vol_ctl_reg, 0x01, 0x00);
		break;
	}

out:
	kfree(widget_name);
	return ret;
}

static int msm8x16_wcd_codec_enable_interpolator(struct snd_soc_dapm_widget *w,
						 struct snd_kcontrol *kcontrol,
						 int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* apply the digital gain after the interpolator is enabled*/
		if ((w->shift) < ARRAY_SIZE(rx_digital_gain_reg))
			snd_soc_write(codec,
				  rx_digital_gain_reg[w->shift],
				  snd_soc_read(codec,
				  rx_digital_gain_reg[w->shift])
				  );
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_CDC_CLK_RX_RESET_CTL,
			1 << w->shift, 1 << w->shift);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_CDC_CLK_RX_RESET_CTL,
			1 << w->shift, 0x0);
		/*
		 * disable the mute enabled during the PMD of this device
		 */
		if (msm8x16_wcd->mute_mask & HPHL_PA_DISABLE) {
			pr_debug("disabling HPHL mute\n");
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_CDC_RX1_B6_CTL, 0x01, 0x00);
			msm8x16_wcd->mute_mask &= ~(HPHL_PA_DISABLE);
		}
		if (msm8x16_wcd->mute_mask & HPHR_PA_DISABLE) {
			pr_debug("disabling HPHR mute\n");
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_CDC_RX2_B6_CTL, 0x01, 0x00);
			msm8x16_wcd->mute_mask &= ~(HPHR_PA_DISABLE);
		}
		if (msm8x16_wcd->mute_mask & SPKR_PA_DISABLE) {
			pr_debug("disabling SPKR mute\n");
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_CDC_RX3_B6_CTL, 0x01, 0x00);
			msm8x16_wcd->mute_mask &= ~(SPKR_PA_DISABLE);
		}
		if (msm8x16_wcd->mute_mask & EAR_PA_DISABLE) {
			pr_debug("disabling EAR mute\n");
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_CDC_RX1_B6_CTL, 0x01, 0x00);
			msm8x16_wcd->mute_mask &= ~(EAR_PA_DISABLE);
		}
	}
	return 0;
}

static int msm8x16_wcd_codec_enable_dig_clk(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (w->shift == 2)
			snd_soc_update_bits(codec, w->reg, 0x80, 0x80);
		if (msm8x16_wcd->spk_boost_set) {
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_SEC_ACCESS,
					0xA5, 0xA5);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_PERPH_RESET_CTL3,
					0x0F, 0x0F);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_CURRENT_LIMIT,
					0x82, 0x82);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
					0x20, 0x20);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_BOOST_EN_CTL,
					0xDF, 0xDF);
			usleep_range(CODEC_DELAY_1_MS, CODEC_DELAY_1_1_MS);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_CURRENT_LIMIT,
					0x83, 0x83);
		} else if (msm8x16_wcd->ear_pa_boost_set) {
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_SEC_ACCESS,
					0xA5, 0xA5);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_PERPH_RESET_CTL3,
					0x07, 0x07);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_BYPASS_MODE,
					0x40, 0x40);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_BYPASS_MODE,
					0x80, 0x80);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_BYPASS_MODE,
					0x02, 0x02);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_BOOST_EN_CTL,
					0xDF, 0xDF);
		} else {
			snd_soc_update_bits(codec, w->reg, 1<<w->shift,
					1<<w->shift);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (msm8x16_wcd->spk_boost_set) {
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_BOOST_EN_CTL,
					0xDF, 0x5F);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
					0x20, 0x00);
		} else if (msm8x16_wcd->ear_pa_boost_set) {
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_BOOST_EN_CTL,
					0x80, 0x00);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_BYPASS_MODE,
					0x80, 0x00);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_BYPASS_MODE,
					0x02, 0x00);
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_BYPASS_MODE,
					0x40, 0x00);
		} else {
			snd_soc_update_bits(codec, w->reg, 1<<w->shift, 0x00);
		}
		break;
	}
	return 0;
}

static int msm8x16_wcd_codec_enable_rx_chain(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec,
				MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
				0x80, 0x80);
		dev_dbg(codec->dev,
			"%s: PMU:Sleeping 20ms after disabling mute\n",
			__func__);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec,
				MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
				0x80, 0x00);
		dev_dbg(codec->dev,
			"%s: PMD:Sleeping 20ms after disabling mute\n",
			__func__);
		snd_soc_update_bits(codec, w->reg,
			    1 << w->shift, 0x00);
		msleep(20);
		break;
	}
	return 0;
}

static int msm8x16_wcd_codec_enable_rx_bias(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		msm8x16_wcd->rx_bias_count++;
		if (msm8x16_wcd->rx_bias_count == 1)
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_RX_COM_BIAS_DAC,
					0x81, 0x81);
		break;
	case SND_SOC_DAPM_POST_PMD:
		msm8x16_wcd->rx_bias_count--;
		if (msm8x16_wcd->rx_bias_count == 0)
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_ANALOG_RX_COM_BIAS_DAC,
					0x81, 0x00);
		break;
	}
	dev_dbg(codec->dev, "%s rx_bias_count = %d\n",
			__func__, msm8x16_wcd->rx_bias_count);
	return 0;
}

static int msm8x16_wcd_codec_enable_charge_pump(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (!(strcmp(w->name, "EAR CP")))
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
					0x80, 0x80);
		else
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
					0xC0, 0xC0);
		break;
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(CODEC_DELAY_1_MS, CODEC_DELAY_1_1_MS);
		break;
	case SND_SOC_DAPM_POST_PMD:
		usleep_range(CODEC_DELAY_1_MS, CODEC_DELAY_1_1_MS);
		if (!(strcmp(w->name, "EAR CP")))
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
					0x80, 0x00);
		else {
			snd_soc_update_bits(codec,
					MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
					0x40, 0x00);
			if (msm8x16_wcd->rx_bias_count == 0)
				snd_soc_update_bits(codec,
					MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
					0x80, 0x00);
			dev_dbg(codec->dev, "%s: rx_bias_count = %d\n",
					__func__, msm8x16_wcd->rx_bias_count);
		}
		break;
	}
	return 0;
}

static int msm8x16_wcd_hphl_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_ANALOG_RX_HPH_L_PA_DAC_CTL, 0x02, 0x02);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL, 0x01, 0x01);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_ANA_CLK_CTL, 0x02, 0x02);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_ANALOG_RX_HPH_L_PA_DAC_CTL, 0x02, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_ANA_CLK_CTL, 0x02, 0x00);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL, 0x01, 0x00);
		break;
	}
	return 0;
}

static int msm8x16_wcd_hph_pa_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd_chip *msm8x16_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (w->shift == 5) {
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_RX_HPH_L_TEST, 0x04, 0x04);
		} else if (w->shift == 4) {
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_RX_HPH_R_TEST, 0x04, 0x04);
		}
		snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_NCP_FBCTRL, 0x20, 0x20);
		break;

	case SND_SOC_DAPM_POST_PMU:
		usleep_range(4000, 4100);
		if (w->shift == 5)
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_CDC_RX1_B6_CTL, 0x01, 0x00);
		else if (w->shift == 4)
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_CDC_RX2_B6_CTL, 0x01, 0x00);
		usleep_range(10000, 10100);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		if (w->shift == 5) {
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_CDC_RX1_B6_CTL, 0x01, 0x01);
			msleep(20);
			msm8x16_wcd->mute_mask |= HPHL_PA_DISABLE;
		} else if (w->shift == 4) {
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_CDC_RX2_B6_CTL, 0x01, 0x01);
			msleep(20);
			msm8x16_wcd->mute_mask |= HPHR_PA_DISABLE;
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (w->shift == 5) {
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_RX_HPH_L_TEST, 0x04, 0x00);

		} else if (w->shift == 4) {
			snd_soc_update_bits(codec,
				MSM8X16_WCD_A_ANALOG_RX_HPH_R_TEST, 0x04, 0x00);
		}
		usleep_range(4000, 4100);

		usleep_range(CODEC_DELAY_1_MS, CODEC_DELAY_1_1_MS);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL, 0x40, 0x40);
		dev_dbg(codec->dev,
			"%s: sleep 10 ms after %s PA disable.\n", __func__,
			w->name);
		usleep_range(10000, 10100);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_route audio_map[] = {
	{"RX_I2S_CLK", NULL, "CDC_CONN"},
	{"I2S RX1", NULL, "RX_I2S_CLK"},
	{"I2S RX2", NULL, "RX_I2S_CLK"},
	{"I2S RX3", NULL, "RX_I2S_CLK"},

	{"I2S TX1", NULL, "TX_I2S_CLK"},
	{"I2S TX2", NULL, "TX_I2S_CLK"},

	{"I2S TX1", NULL, "DEC1 MUX"},
	{"I2S TX2", NULL, "DEC2 MUX"},

	/* RDAC Connections */
	{"HPHR DAC", NULL, "RDAC2 MUX"},
	{"RDAC2 MUX", "RX1", "RX1 CHAIN"},
	{"RDAC2 MUX", "RX2", "RX2 CHAIN"},

	/* Earpiece (RX MIX1) */
	{"EAR", NULL, "EAR_S"},
	{"EAR_S", "Switch", "EAR PA"},
	{"EAR PA", NULL, "RX_BIAS"},
	{"EAR PA", NULL, "HPHL DAC"},
	{"EAR PA", NULL, "HPHR DAC"},
	{"EAR PA", NULL, "EAR CP"},

	/* Headset (RX MIX1 and RX MIX2) */
	{"HEADPHONE", NULL, "HPHL PA"},
	{"HEADPHONE", NULL, "HPHR PA"},

	{"HPHL PA", NULL, "HPHL"},
	{"HPHR PA", NULL, "HPHR"},
	{"HPHL", "Switch", "HPHL DAC"},
	{"HPHR", "Switch", "HPHR DAC"},
	{"HPHL PA", NULL, "CP"},
	{"HPHL PA", NULL, "RX_BIAS"},
	{"HPHR PA", NULL, "CP"},
	{"HPHR PA", NULL, "RX_BIAS"},
	{"HPHL DAC", NULL, "RX1 CHAIN"},

	{"SPK_OUT", NULL, "SPK PA"},
	{"SPK PA", NULL, "SPK_RX_BIAS"},
	{"SPK PA", NULL, "SPK DAC"},
	{"SPK DAC", "Switch", "RX3 CHAIN"},
	{"SPK DAC", NULL, "VDD_SPKDRV"},

	{"RX1 CHAIN", NULL, "RX1 CLK"},
	{"RX2 CHAIN", NULL, "RX2 CLK"},
	{"RX3 CHAIN", NULL, "RX3 CLK"},
	{"RX1 CHAIN", NULL, "RX1 MIX2"},
	{"RX2 CHAIN", NULL, "RX2 MIX2"},
	{"RX3 CHAIN", NULL, "RX3 MIX1"},

	{"RX1 MIX1", NULL, "RX1 MIX1 INP1"},
	{"RX1 MIX1", NULL, "RX1 MIX1 INP2"},
	{"RX1 MIX1", NULL, "RX1 MIX1 INP3"},
	{"RX2 MIX1", NULL, "RX2 MIX1 INP1"},
	{"RX2 MIX1", NULL, "RX2 MIX1 INP2"},
	{"RX3 MIX1", NULL, "RX3 MIX1 INP1"},
	{"RX3 MIX1", NULL, "RX3 MIX1 INP2"},
	{"RX1 MIX2", NULL, "RX1 MIX1"},
	{"RX1 MIX2", NULL, "RX1 MIX2 INP1"},
	{"RX2 MIX2", NULL, "RX2 MIX1"},
	{"RX2 MIX2", NULL, "RX2 MIX2 INP1"},

	{"RX1 MIX1 INP1", "RX1", "I2S RX1"},
	{"RX1 MIX1 INP1", "RX2", "I2S RX2"},
	{"RX1 MIX1 INP1", "RX3", "I2S RX3"},
	{"RX1 MIX1 INP1", "IIR1", "IIR1"},
	{"RX1 MIX1 INP1", "IIR2", "IIR2"},
	{"RX1 MIX1 INP2", "RX1", "I2S RX1"},
	{"RX1 MIX1 INP2", "RX2", "I2S RX2"},
	{"RX1 MIX1 INP2", "RX3", "I2S RX3"},
	{"RX1 MIX1 INP2", "IIR1", "IIR1"},
	{"RX1 MIX1 INP2", "IIR2", "IIR2"},
	{"RX1 MIX1 INP3", "RX1", "I2S RX1"},
	{"RX1 MIX1 INP3", "RX2", "I2S RX2"},
	{"RX1 MIX1 INP3", "RX3", "I2S RX3"},

	{"RX2 MIX1 INP1", "RX1", "I2S RX1"},
	{"RX2 MIX1 INP1", "RX2", "I2S RX2"},
	{"RX2 MIX1 INP1", "RX3", "I2S RX3"},
	{"RX2 MIX1 INP1", "IIR1", "IIR1"},
	{"RX2 MIX1 INP1", "IIR2", "IIR2"},
	{"RX2 MIX1 INP2", "RX1", "I2S RX1"},
	{"RX2 MIX1 INP2", "RX2", "I2S RX2"},
	{"RX2 MIX1 INP2", "RX3", "I2S RX3"},
	{"RX2 MIX1 INP2", "IIR1", "IIR1"},
	{"RX2 MIX1 INP2", "IIR2", "IIR2"},

	{"RX3 MIX1 INP1", "RX1", "I2S RX1"},
	{"RX3 MIX1 INP1", "RX2", "I2S RX2"},
	{"RX3 MIX1 INP1", "RX3", "I2S RX3"},
	{"RX3 MIX1 INP1", "IIR1", "IIR1"},
	{"RX3 MIX1 INP1", "IIR2", "IIR2"},
	{"RX3 MIX1 INP2", "RX1", "I2S RX1"},
	{"RX3 MIX1 INP2", "RX2", "I2S RX2"},
	{"RX3 MIX1 INP2", "RX3", "I2S RX3"},
	{"RX3 MIX1 INP2", "IIR1", "IIR1"},
	{"RX3 MIX1 INP2", "IIR2", "IIR2"},

	{"RX1 MIX2 INP1", "IIR1", "IIR1"},
	{"RX2 MIX2 INP1", "IIR1", "IIR1"},
	{"RX1 MIX2 INP1", "IIR2", "IIR2"},
	{"RX2 MIX2 INP1", "IIR2", "IIR2"},

	/* Decimator Inputs */
	{"DEC1 MUX", "DMIC1", "DMIC1"},
	{"DEC1 MUX", "DMIC2", "DMIC2"},
	{"DEC1 MUX", "ADC1", "ADC1"},
	{"DEC1 MUX", "ADC2", "ADC2"},
	{"DEC1 MUX", "ADC3", "ADC3"},
	{"DEC1 MUX", NULL, "CDC_CONN"},

	{"DEC2 MUX", "DMIC1", "DMIC1"},
	{"DEC2 MUX", "DMIC2", "DMIC2"},
	{"DEC2 MUX", "ADC1", "ADC1"},
	{"DEC2 MUX", "ADC2", "ADC2"},
	{"DEC2 MUX", "ADC3", "ADC3"},
	{"DEC2 MUX", NULL, "CDC_CONN"},

	/* ADC Connections */
	{"ADC2", NULL, "ADC2 MUX"},
	{"ADC3", NULL, "ADC2 MUX"},
	{"ADC2 MUX", "INP2", "ADC2_INP2"},
	{"ADC2 MUX", "INP3", "ADC2_INP3"},

	{"ADC1", NULL, "AMIC1"},
	{"ADC2_INP2", NULL, "AMIC2"},
	{"ADC2_INP3", NULL, "AMIC3"},

	/* TODO: Fix this */
	{"IIR1", NULL, "IIR1 INP1 MUX"},
	{"IIR1 INP1 MUX", "DEC1", "DEC1 MUX"},
	{"IIR1 INP1 MUX", "DEC2", "DEC2 MUX"},
	{"IIR2", NULL, "IIR2 INP1 MUX"},
	{"IIR2 INP1 MUX", "DEC1", "DEC1 MUX"},
	{"IIR2 INP1 MUX", "DEC2", "DEC2 MUX"},
	{"MIC BIAS Internal1", NULL, "INT_LDO_H"},
	{"MIC BIAS Internal2", NULL, "INT_LDO_H"},
	{"MIC BIAS External", NULL, "INT_LDO_H"},
	{"MIC BIAS External2", NULL, "INT_LDO_H"},
	{"MIC BIAS Internal1", NULL, "MICBIAS_REGULATOR"},
	{"MIC BIAS Internal2", NULL, "MICBIAS_REGULATOR"},
	{"MIC BIAS External", NULL, "MICBIAS_REGULATOR"},
	{"MIC BIAS External2", NULL, "MICBIAS_REGULATOR"},
};

static int msm8x16_wcd_hphr_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_ANALOG_RX_HPH_R_PA_DAC_CTL, 0x02, 0x02);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL, 0x02, 0x02);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_ANA_CLK_CTL, 0x01, 0x01);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_ANALOG_RX_HPH_R_PA_DAC_CTL, 0x02, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_ANA_CLK_CTL, 0x01, 0x00);
		snd_soc_update_bits(codec,
			MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL, 0x02, 0x00);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget msm8x16_wcd_dapm_widgets[] = {
	/*RX stuff */
	SND_SOC_DAPM_OUTPUT("EAR"),

	SND_SOC_DAPM_PGA_E("EAR PA", SND_SOC_NOPM,
			0, 0, NULL, 0, msm8x16_wcd_codec_enable_ear_pa,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("EAR_S", SND_SOC_NOPM, 0, 0,
		ear_pa_switch, ARRAY_SIZE(ear_pa_switch)),

	SND_SOC_DAPM_AIF_IN("I2S RX1", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("I2S RX2", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("I2S RX3", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_SUPPLY("INT_LDO_H", SND_SOC_NOPM, 1, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("HEADPHONE"),
	SND_SOC_DAPM_PGA_E("HPHL PA", MSM8X16_WCD_A_ANALOG_RX_HPH_CNP_EN,
		5, 0, NULL, 0,
		msm8x16_wcd_hph_pa_event, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("HPHL", SND_SOC_NOPM, 0, 0, hphl_mux),

	SND_SOC_DAPM_MIXER_E("HPHL DAC",
		MSM8X16_WCD_A_ANALOG_RX_HPH_L_PA_DAC_CTL, 3, 0, NULL,
		0, msm8x16_wcd_hphl_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("HPHR PA", MSM8X16_WCD_A_ANALOG_RX_HPH_CNP_EN,
		4, 0, NULL, 0,
		msm8x16_wcd_hph_pa_event, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX("HPHR", SND_SOC_NOPM, 0, 0, hphr_mux),

	SND_SOC_DAPM_MIXER_E("HPHR DAC",
		MSM8X16_WCD_A_ANALOG_RX_HPH_R_PA_DAC_CTL, 3, 0, NULL,
		0, msm8x16_wcd_hphr_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER("SPK DAC", SND_SOC_NOPM, 0, 0,
		spkr_switch, ARRAY_SIZE(spkr_switch)),

	/* Speaker */
	SND_SOC_DAPM_OUTPUT("SPK_OUT"),

	SND_SOC_DAPM_PGA_E("SPK PA", MSM8X16_WCD_A_ANALOG_SPKR_DRV_CTL,
			6, 0, NULL, 0, msm8x16_wcd_codec_enable_spk_pa,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER_E("RX1 MIX1",
			MSM8X16_WCD_A_CDC_CLK_RX_B1_CTL, 0, 0, NULL, 0,
			msm8x16_wcd_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_REG|
			SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD|
			SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER_E("RX2 MIX1",
			MSM8X16_WCD_A_CDC_CLK_RX_B1_CTL, 1, 0, NULL, 0,
			msm8x16_wcd_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_REG|
			SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER_E("RX1 MIX2",
		MSM8X16_WCD_A_CDC_CLK_RX_B1_CTL, 0, 0, NULL,
		0, msm8x16_wcd_codec_enable_interpolator,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX2 MIX2",
		MSM8X16_WCD_A_CDC_CLK_RX_B1_CTL, 1, 0, NULL,
		0, msm8x16_wcd_codec_enable_interpolator,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX3 MIX1",
		MSM8X16_WCD_A_CDC_CLK_RX_B1_CTL, 2, 0, NULL,
		0, msm8x16_wcd_codec_enable_interpolator,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("RX1 CLK", MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
		0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("RX2 CLK", MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
		1, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("RX3 CLK", MSM8X16_WCD_A_DIGITAL_CDC_DIG_CLK_CTL,
		2, 0, msm8x16_wcd_codec_enable_dig_clk, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX1 CHAIN", MSM8X16_WCD_A_CDC_RX1_B6_CTL, 0, 0,
		NULL, 0,
		msm8x16_wcd_codec_enable_rx_chain,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX2 CHAIN", MSM8X16_WCD_A_CDC_RX2_B6_CTL, 0, 0,
		NULL, 0,
		msm8x16_wcd_codec_enable_rx_chain,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX3 CHAIN", MSM8X16_WCD_A_CDC_RX3_B6_CTL, 0, 0,
		NULL, 0,
		msm8x16_wcd_codec_enable_rx_chain,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("RX1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP3", SND_SOC_NOPM, 0, 0,
		&rx_mix1_inp3_mux),

	SND_SOC_DAPM_MUX("RX2 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx2_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx2_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP3", SND_SOC_NOPM, 0, 0,
		&rx2_mix1_inp3_mux),

	SND_SOC_DAPM_MUX("RX3 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx3_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx3_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP3", SND_SOC_NOPM, 0, 0,
		&rx3_mix1_inp3_mux),

	SND_SOC_DAPM_MUX("RX1 MIX2 INP1", SND_SOC_NOPM, 0, 0,
		&rx1_mix2_inp1_mux),
	SND_SOC_DAPM_MUX("RX2 MIX2 INP1", SND_SOC_NOPM, 0, 0,
		&rx2_mix2_inp1_mux),

	SND_SOC_DAPM_SUPPLY("MICBIAS_REGULATOR", SND_SOC_NOPM,
		ON_DEMAND_MICBIAS, 0,
		msm8x16_wcd_codec_enable_on_demand_supply,
		SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("CP", MSM8X16_WCD_A_ANALOG_NCP_EN, 0, 0,
		msm8x16_wcd_codec_enable_charge_pump, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU |	SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("EAR CP", MSM8X16_WCD_A_ANALOG_NCP_EN, 4, 0,
		msm8x16_wcd_codec_enable_charge_pump, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("RX_BIAS", SND_SOC_NOPM,
		0, 0, msm8x16_wcd_codec_enable_rx_bias,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("SPK_RX_BIAS", SND_SOC_NOPM, 0, 0,
		msm8x16_wcd_codec_enable_rx_bias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	/* TX */

	SND_SOC_DAPM_SUPPLY_S("CDC_CONN", -2, MSM8X16_WCD_A_CDC_CLK_OTHR_CTL,
		2, 0, NULL, 0),

	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_SUPPLY("MIC BIAS Internal1",
		MSM8X16_WCD_A_ANALOG_MICB_1_EN, 7, 0,
		msm8x16_wcd_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS Internal2",
		MSM8X16_WCD_A_ANALOG_MICB_2_EN, 7, 0,
		msm8x16_wcd_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS Internal3",
		MSM8X16_WCD_A_ANALOG_MICB_1_EN, 7, 0,
		msm8x16_wcd_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC1", NULL, MSM8X16_WCD_A_ANALOG_TX_1_EN, 7, 0,
		msm8x16_wcd_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC2_INP2",
		NULL, MSM8X16_WCD_A_ANALOG_TX_2_EN, 7, 0,
		msm8x16_wcd_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC2_INP3",
		NULL, MSM8X16_WCD_A_ANALOG_TX_3_EN, 7, 0,
		msm8x16_wcd_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER("ADC2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("ADC3", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("ADC2 MUX", SND_SOC_NOPM, 0, 0,
		&tx_adc2_mux),

	SND_SOC_DAPM_SUPPLY("MIC BIAS External",
		MSM8X16_WCD_A_ANALOG_MICB_1_EN, 7, 0,
		msm8x16_wcd_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("MIC BIAS External2",
		MSM8X16_WCD_A_ANALOG_MICB_2_EN, 7, 0,
		msm8x16_wcd_codec_enable_micbias, SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),


	SND_SOC_DAPM_INPUT("AMIC3"),

	SND_SOC_DAPM_MUX_E("DEC1 MUX",
		MSM8X16_WCD_A_CDC_CLK_TX_CLK_EN_B1_CTL, 0, 0,
		&dec1_mux, msm8x16_wcd_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC2 MUX",
		MSM8X16_WCD_A_CDC_CLK_TX_CLK_EN_B1_CTL, 1, 0,
		&dec2_mux, msm8x16_wcd_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("RDAC2 MUX", SND_SOC_NOPM, 0, 0, &rdac2_mux),

	SND_SOC_DAPM_INPUT("AMIC2"),

	SND_SOC_DAPM_AIF_OUT("I2S TX1", "AIF1 Capture", 0, SND_SOC_NOPM,
		0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S TX2", "AIF1 Capture", 0, SND_SOC_NOPM,
		0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S TX3", "AIF1 Capture", 0, SND_SOC_NOPM,
		0, 0),


	/* Digital Mic Inputs */
	SND_SOC_DAPM_ADC_E("DMIC1", NULL, SND_SOC_NOPM, 0, 0,
		msm8x16_wcd_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC2", NULL, SND_SOC_NOPM, 0, 0,
		msm8x16_wcd_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	/* Sidetone */
	SND_SOC_DAPM_MUX("IIR1 INP1 MUX", SND_SOC_NOPM, 0, 0, &iir1_inp1_mux),
	SND_SOC_DAPM_PGA("IIR1",
		MSM8X16_WCD_A_CDC_CLK_SD_CTL, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("IIR2 INP1 MUX", SND_SOC_NOPM, 0, 0, &iir2_inp1_mux),
	SND_SOC_DAPM_PGA("IIR2",
		MSM8X16_WCD_A_CDC_CLK_SD_CTL, 1, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("RX_I2S_CLK",
		MSM8X16_WCD_A_CDC_CLK_RX_I2S_CTL,	4, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("TX_I2S_CLK",
		MSM8X16_WCD_A_CDC_CLK_TX_I2S_CTL, 4, 0,
		NULL, 0),
};

static struct snd_soc_codec_driver msm8x16_wcd_codec = {
	.probe	= msm8x16_wcd_codec_probe,
	.remove	= msm8x16_wcd_codec_remove,
	.read = msm8x16_wcd_read,
	.write = msm8x16_wcd_write,
	.reg_cache_size = MSM8X16_WCD_CACHE_SIZE,
	.reg_cache_default = msm8x16_wcd_reset_reg_defaults,
	.reg_word_size = 1,
	.controls = msm8x16_wcd_snd_controls,
	.num_controls = ARRAY_SIZE(msm8x16_wcd_snd_controls),
	.dapm_widgets = msm8x16_wcd_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(msm8x16_wcd_dapm_widgets),
	.dapm_routes = audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),
};

static int msm8x16_wcd_codec_parse_dt(struct platform_device *pdev,
				      struct wcd_chip *chip)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;
	u32 res[2];

	ret = of_property_read_u32_array(np, "reg", res, 2);
	if (ret < 0)
		return ret;

	chip->analog_base = res[0];

	chip->digital_map = syscon_regmap_lookup_by_phandle(np, "digital");
	if (IS_ERR(chip->digital_map))
		return PTR_ERR(chip->digital_map);

	chip->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(chip->vddio)) {
		dev_err(dev, "Failed to get vdd supply\n");
		return PTR_ERR(chip->vddio);
	}

	chip->vdd_pa = devm_regulator_get(dev, "vdd-pa");
	if (IS_ERR(chip->vdd_pa)) {
		dev_err(dev, "Failed to get vdd supply\n");
		return PTR_ERR(chip->vdd_pa);
	}

	chip->vdd_mic_bias = devm_regulator_get(dev, "vdd-mic-bias");
	if (IS_ERR(chip->vdd_mic_bias)) {
		dev_err(dev, "Failed to get vdd micbias supply\n");
		return PTR_ERR(chip->vdd_mic_bias);
	}

	chip->mclk = devm_clk_get(dev, "mclk");

	return 0;
}

static int wcd_probe(struct platform_device *pdev)
{
	struct wcd_chip *chip;
	struct device *dev = &pdev->dev;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->analog_map = dev_get_regmap(dev->parent, NULL);
	if (!chip->analog_map)
		return -ENXIO;

	ret = msm8x16_wcd_codec_parse_dt(pdev, chip);
	if (IS_ERR_VALUE(ret))
		return ret;

	clk_set_rate(chip->mclk, 9600000);
	clk_prepare_enable(chip->mclk);

	dev_set_drvdata(dev, chip);

	return snd_soc_register_codec(dev, &msm8x16_wcd_codec,
				      msm8x16_wcd_codec_dai,
				      ARRAY_SIZE(msm8x16_wcd_codec_dai));
}

static int wcd_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static const struct of_device_id wcd_match_table[] = {
	{ .compatible = "qcom,apq8016-wcd-codec" },
	{ .compatible = "qcom,msm8x16-wcd-codec" },
	{ }
};
MODULE_DEVICE_TABLE(of, wcd_match_table);

static struct platform_driver wcd_driver = {
	.driver = {
		.name = "msm8x16-wcd-codec",
		.of_match_table = wcd_match_table,
	},
	.probe  = wcd_probe,
	.remove = wcd_remove,
};
module_platform_driver(wcd_driver);

MODULE_ALIAS("platform:spmi-wcd-codec");
MODULE_DESCRIPTION("SPMI PMIC WCD codec driver");
MODULE_LICENSE("GPL v2");
