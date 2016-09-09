/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>

#include "msm8916-wcd-registers.h"
#include "msm8916-wcd.h"
#include "dt-bindings/sound/msm8916-wcd.h"

#define MSM8916_WCD_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000)
#define MSM8916_WCD_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
		SNDRV_PCM_FMTBIT_S24_LE)

/* Internal status on mute_mask to track mute on different sinks */
#define MUTE_MASK_HPHL_PA_DISABLE		BIT(1)
#define MUTE_MASK_HPHR_PA_DISABLE		BIT(2)
#define MUTE_MASK_EAR_PA_DISABLE		BIT(3)
#define MUTE_MASK_SPKR_PA_DISABLE		BIT(4)

struct msm8916_wcd_chip {
	struct regmap *analog_map;
	struct regmap *digital_map;
	unsigned int analog_offset;
	u16 pmic_rev;
	u16 codec_version;

	struct clk *mclk;
	struct regulator *vddio;
	struct regulator *vdd_tx_rx;

	u32 mute_mask;
	u32 rx_bias_count;
	bool micbias1_cap_mode;
	bool micbias2_cap_mode;
	int dmic_clk_cnt;
};

static unsigned long rx_gain_reg[] = {
	LPASS_CDC_RX1_VOL_CTL_B2_CTL,
	LPASS_CDC_RX2_VOL_CTL_B2_CTL,
	LPASS_CDC_RX3_VOL_CTL_B2_CTL,
};

static unsigned long tx_gain_reg[] = {
	LPASS_CDC_TX1_VOL_CTL_GAIN,
	LPASS_CDC_TX2_VOL_CTL_GAIN,
};

static const char *const rx_mix1_text[] = {
	"ZERO", "IIR1", "IIR2", "RX1", "RX2", "RX3"
};

static const char *const dec_mux_text[] = {
	"ZERO", "ADC1", "ADC2", "ADC3", "DMIC1", "DMIC2"
};
static const char *const rx_mix2_text[] = { "ZERO", "IIR1", "IIR2" };
static const char *const adc2_mux_text[] = { "ZERO", "INP2", "INP3" };
static const char *const rdac2_mux_text[] = { "ZERO", "RX2", "RX1" };
static const char *const hph_text[] = { "ZERO", "Switch", };

/* RX1 MIX1 */
static const struct soc_enum rx_mix1_inp_enum[] = {
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX1_B1_CTL, 0, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX1_B1_CTL, 3, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX1_B2_CTL, 0, 6, rx_mix1_text),
};

/* RX1 MIX2 */
static const struct soc_enum rx_mix2_inp1_chain_enum =
SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX1_B3_CTL, 0, 3, rx_mix2_text);

/* RX2 MIX1 */
static const struct soc_enum rx2_mix1_inp_enum[] = {
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX2_B1_CTL, 0, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX2_B1_CTL, 3, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX2_B1_CTL, 0, 6, rx_mix1_text),
};

/* RX2 MIX2 */
static const struct soc_enum rx2_mix2_inp1_chain_enum =
SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX2_B3_CTL, 0, 3, rx_mix2_text);

/* RX3 MIX1 */
static const struct soc_enum rx3_mix1_inp_enum[] = {
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX3_B1_CTL, 0, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX3_B1_CTL, 3, 6, rx_mix1_text),
	SOC_ENUM_SINGLE(LPASS_CDC_CONN_RX3_B1_CTL, 0, 6, rx_mix1_text),
};

/* DEC */
static const struct soc_enum dec1_mux_enum =
SOC_ENUM_SINGLE(LPASS_CDC_CONN_TX_B1_CTL, 0, 6, dec_mux_text);

static const struct soc_enum dec2_mux_enum =
SOC_ENUM_SINGLE(LPASS_CDC_CONN_TX_B1_CTL, 3, 6, dec_mux_text);

/* RDAC2 MUX */
static const struct soc_enum rdac2_mux_enum =
SOC_ENUM_SINGLE(CDC_D_CDC_CONN_HPHR_DAC_CTL, 0, 3, rdac2_mux_text);

/* ADC2 MUX */
static const struct soc_enum adc2_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(adc2_mux_text), adc2_mux_text);

static const struct soc_enum hph_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hph_text), hph_text);

static const struct snd_kcontrol_new spkr_switch[] = {
	SOC_DAPM_SINGLE("Switch", CDC_A_SPKR_DAC_CTL, 7, 1, 0)
};

static const struct snd_kcontrol_new dec1_mux =
SOC_DAPM_ENUM("DEC1 MUX Mux", dec1_mux_enum);

static const struct snd_kcontrol_new dec2_mux =
SOC_DAPM_ENUM("DEC2 MUX Mux", dec2_mux_enum);

static const struct snd_kcontrol_new rdac2_mux =
SOC_DAPM_ENUM("RDAC2 MUX Mux", rdac2_mux_enum);

static const struct snd_kcontrol_new tx_adc2_mux =
SOC_DAPM_ENUM("ADC2 MUX Mux", adc2_enum);

static const struct snd_kcontrol_new rx_mix1_inp1_mux =
SOC_DAPM_ENUM("RX1 MIX1 INP1 Mux", rx_mix1_inp_enum[0]);

static const struct snd_kcontrol_new rx_mix1_inp2_mux =
SOC_DAPM_ENUM("RX1 MIX1 INP2 Mux", rx_mix1_inp_enum[1]);

static const struct snd_kcontrol_new rx_mix1_inp3_mux =
SOC_DAPM_ENUM("RX1 MIX1 INP3 Mux", rx_mix1_inp_enum[2]);

static const struct snd_kcontrol_new rx2_mix1_inp1_mux =
SOC_DAPM_ENUM("RX2 MIX1 INP1 Mux", rx2_mix1_inp_enum[0]);

static const struct snd_kcontrol_new rx2_mix1_inp2_mux =
SOC_DAPM_ENUM("RX2 MIX1 INP2 Mux", rx2_mix1_inp_enum[1]);

static const struct snd_kcontrol_new rx2_mix1_inp3_mux =
SOC_DAPM_ENUM("RX2 MIX1 INP3 Mux", rx2_mix1_inp_enum[2]);

static const struct snd_kcontrol_new rx3_mix1_inp1_mux =
SOC_DAPM_ENUM("RX3 MIX1 INP1 Mux", rx3_mix1_inp_enum[0]);

static const struct snd_kcontrol_new rx3_mix1_inp2_mux =
SOC_DAPM_ENUM("RX3 MIX1 INP2 Mux", rx3_mix1_inp_enum[1]);

static const struct snd_kcontrol_new rx3_mix1_inp3_mux =
SOC_DAPM_ENUM("RX3 MIX1 INP3 Mux", rx3_mix1_inp_enum[2]);

static const struct snd_kcontrol_new hphl_mux = SOC_DAPM_ENUM("HPHL", hph_enum);

static const struct snd_kcontrol_new hphr_mux = SOC_DAPM_ENUM("HPHR", hph_enum);

/* Digital Gain control -38.4 dB to +38.4 dB in 0.3 dB steps */
static const DECLARE_TLV_DB_SCALE(digital_gain, -3840, 30, 0);

/* Analog Gain control 0 dB to +24 dB in 6 dB steps */
static const DECLARE_TLV_DB_SCALE(analog_gain, 0, 600, 0);

static const struct snd_kcontrol_new msm8916_wcd_snd_controls[] = {
	SOC_SINGLE_TLV("ADC1 Volume", CDC_A_TX_1_EN, 3, 8, 0, analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", CDC_A_TX_2_EN, 3, 8, 0, analog_gain),
	SOC_SINGLE_TLV("ADC3 Volume", CDC_A_TX_3_EN, 3, 8, 0, analog_gain),
	SOC_SINGLE_S8_TLV("RX1 Digital Volume", LPASS_CDC_RX1_VOL_CTL_B2_CTL,
			  -128, 127, digital_gain),
	SOC_SINGLE_S8_TLV("RX2 Digital Volume", LPASS_CDC_RX2_VOL_CTL_B2_CTL,
			  -128, 127, digital_gain),
	SOC_SINGLE_S8_TLV("RX3 Digital Volume", LPASS_CDC_RX3_VOL_CTL_B2_CTL,
			  -128, 127, digital_gain),
};

static int msm8916_wcd_write(struct snd_soc_codec *codec, unsigned int reg,
			     unsigned int val)
{
	int ret = -EINVAL;
	struct msm8916_wcd_chip *chip = dev_get_drvdata(codec->dev);
	u8 *cache = codec->reg_cache;

	if (!msm8916_wcd_reg_readonly[reg])
		cache[reg] = val;

	if (MSM8916_WCD_IS_TOMBAK_REG(reg)) {
		/* codec registers inside pmic core */
		ret = regmap_write(chip->analog_map,
				   chip->analog_offset + reg, val);
	} else if (MSM8916_WCD_IS_DIGITAL_REG(reg)) {
		/* codec registers in the cpu core */
		u32 v = val & MSM8916_WCD_REG_VAL_MASK;
		u16 offset = MSM8916_WCD_DIGITAL_REG(reg);

		ret = regmap_write(chip->digital_map, offset, v);
	}

	return ret;
}

static unsigned int msm8916_wcd_read(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	int ret = -EINVAL;
	u32 val = 0;
	struct msm8916_wcd_chip *chip = dev_get_drvdata(codec->dev);
	u8 *cache = codec->reg_cache;

	if (!msm8916_wcd_reg_readonly[reg])
		return cache[reg];

	if (MSM8916_WCD_IS_TOMBAK_REG(reg)) {
		ret = regmap_read(chip->analog_map,
				  chip->analog_offset + reg, &val);
	} else if (MSM8916_WCD_IS_DIGITAL_REG(reg)) {
		u32 v;
		u16 offset = MSM8916_WCD_DIGITAL_REG(reg);

		ret = regmap_read(chip->digital_map, offset, &v);
		val = (u8) v;
	}

	return val;
}

static void msm8916_wcd_configure_cap(struct snd_soc_codec *codec,
				      bool micbias1, bool micbias2)
{
	struct msm8916_wcd_chip *wcd = snd_soc_codec_get_drvdata(codec);

	if (micbias1 && micbias2) {
		if ((wcd->micbias1_cap_mode == MICB_1_EN_EXT_BYP_CAP) ||
		    (wcd->micbias2_cap_mode == MICB_1_EN_EXT_BYP_CAP))
			snd_soc_update_bits(codec, CDC_A_MICB_1_EN,
					    MICB_1_EN_BYP_CAP_MASK,
					    MICB_1_EN_EXT_BYP_CAP);
		else
			snd_soc_update_bits(codec, CDC_A_MICB_1_EN,
					    MICB_1_EN_BYP_CAP_MASK,
					    MICB_1_EN_NO_EXT_BYP_CAP);
	} else if (micbias2) {
		snd_soc_update_bits(codec, CDC_A_MICB_1_EN,
				    MICB_1_EN_BYP_CAP_MASK,
				    wcd->micbias2_cap_mode);
	} else if (micbias1) {
		snd_soc_update_bits(codec, CDC_A_MICB_1_EN,
				    MICB_1_EN_BYP_CAP_MASK,
				    wcd->micbias1_cap_mode);
	} else {
		snd_soc_update_bits(codec, CDC_A_MICB_1_EN,
				    MICB_1_EN_BYP_CAP_MASK, 0);
	}
}

static int msm8916_wcd_hph_pa_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct msm8916_wcd_chip *msm8916_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, CDC_A_NCP_FBCTRL,
				    CDC_A_NCP_FBCTRL_FB_CLK_INV_MASK,
				    CDC_A_NCP_FBCTRL_FB_CLK_INV);
		break;

	case SND_SOC_DAPM_POST_PMU:
		if (w->shift == 5)
			snd_soc_update_bits(codec, LPASS_CDC_RX1_B6_CTL,
					    RXn_B6_CTL_MUTE_MASK, 0);
		else if (w->shift == 4)
			snd_soc_update_bits(codec, LPASS_CDC_RX2_B6_CTL,
					    RXn_B6_CTL_MUTE_MASK, 0);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		if (w->shift == 5) {
			snd_soc_update_bits(codec, LPASS_CDC_RX1_B6_CTL,
					    RXn_B6_CTL_MUTE_MASK,
					    RXn_B6_CTL_MUTE_ENABLE);
			msm8916_wcd->mute_mask |= MUTE_MASK_HPHL_PA_DISABLE;
		} else if (w->shift == 4) {
			snd_soc_update_bits(codec, LPASS_CDC_RX2_B6_CTL,
					    RXn_B6_CTL_MUTE_MASK,
					    RXn_B6_CTL_MUTE_ENABLE);
			msm8916_wcd->mute_mask |= MUTE_MASK_HPHR_PA_DISABLE;
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_NCP_CLK_EN,
				    DIG_CLK_CTL_NCP_CLK_EN);
		break;
	}
	return 0;
}

static int msm8916_wcd_hphl_dac_event(struct snd_soc_dapm_widget *w,
				      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, CDC_A_RX_HPH_L_PA_DAC_CTL,
				    RX_HPA_L_PA_DAC_CTL_DATA_RESET_MASK,
				    RX_HPA_L_PA_DAC_CTL_DATA_RESET_RESET);
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_RXD1_CLK_EN,
				    DIG_CLK_CTL_RXD1_CLK_EN);
		snd_soc_update_bits(codec, CDC_D_CDC_ANA_CLK_CTL,
				    ANA_CLK_CTL_EAR_HPHL_CLK_EN,
				    ANA_CLK_CTL_EAR_HPHL_CLK_EN);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, CDC_A_RX_HPH_L_PA_DAC_CTL,
				    RX_HPA_L_PA_DAC_CTL_DATA_RESET_MASK, 0);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, CDC_D_CDC_ANA_CLK_CTL,
				    ANA_CLK_CTL_EAR_HPHL_CLK_EN, 0);
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_RXD1_CLK_EN, 0);
		break;
	}
	return 0;
}

static int msm8916_wcd_codec_enable_spk_pa(struct snd_soc_dapm_widget *w,
					   struct snd_kcontrol *kcontrol,
					   int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct msm8916_wcd_chip *msm8916_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, CDC_D_CDC_ANA_CLK_CTL,
				    ANA_CLK_CTL_SPKR_CLK_EN_MASK,
				    ANA_CLK_CTL_SPKR_CLK_EN);
		snd_soc_update_bits(codec, CDC_A_SPKR_PWRSTG_CTL,
				    SPKR_PWRSTG_CTL_DAC_EN_MASK,
				    SPKR_PWRSTG_CTL_DAC_EN_ENABLE);
		snd_soc_update_bits(codec, CDC_A_SPKR_PWRSTG_CTL,
				    SPKR_PWRSTG_CTL_MASK,
				    SPKR_PWRSTG_CTL_DEFAULTS);
		if (!TOMBAK_IS_1_0(msm8916_wcd->pmic_rev))
			snd_soc_update_bits(codec, CDC_A_RX_EAR_CTL,
					    RX_EAR_CTL_SPK_VBAT_LDO_EN_MASK,
					    RX_EAR_CTL_SPK_VBAT_LDO_EN_ENABLE);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, CDC_A_SPKR_DRV_CTL,
				    SPKR_DRV_CTL_DEF_MASK,
				    SPKR_DRV_CTL_DEF_VAL);

		snd_soc_update_bits(codec, LPASS_CDC_RX3_B6_CTL,
				    RXn_B6_CTL_MUTE_MASK,
				    RXn_B6_CTL_MUTE_DISABLE);
		snd_soc_update_bits(codec, w->reg,
				    SPKR_DRV_CLASSD_PA_EN_MASK,
				    SPKR_DRV_CLASSD_PA_EN_ENABLE);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, CDC_A_SPKR_PWRSTG_CTL,
				    SPKR_PWRSTG_CTL_MASK, 0);
		snd_soc_update_bits(codec, CDC_A_SPKR_PWRSTG_CTL,
				    SPKR_PWRSTG_CTL_DAC_EN_MASK, 0);

		snd_soc_update_bits(codec, CDC_A_SPKR_DAC_CTL,
				    SPKR_DAC_CTL_DAC_RESET_MASK,
				    SPKR_DAC_CTL_DAC_RESET_NORMAL);
		snd_soc_update_bits(codec, CDC_D_CDC_ANA_CLK_CTL,
				    ANA_CLK_CTL_SPKR_CLK_EN, 0);
		if (!TOMBAK_IS_1_0(msm8916_wcd->pmic_rev))
			snd_soc_update_bits(codec, CDC_A_RX_EAR_CTL,
					    RX_EAR_CTL_SPK_VBAT_LDO_EN_MASK, 0);
		break;
	}
	return 0;
}

static int msm8916_wcd_codec_enable_dig_clk(struct snd_soc_dapm_widget *w,
					    struct snd_kcontrol *kcontrol,
					    int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct msm8916_wcd_chip *msm8916_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_RXD_PDM_CLK_EN_MASK,
				    DIG_CLK_CTL_RXD_PDM_CLK_EN);
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (msm8916_wcd->rx_bias_count == 0)
			snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
					    DIG_CLK_CTL_RXD_PDM_CLK_EN_MASK, 0);
		break;
	}

	return 0;
}

static int msm8916_wcd_codec_enable_rx_chain(struct snd_soc_dapm_widget *w,
					     struct snd_kcontrol *kcontrol,
					     int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_RXD_PDM_CLK_EN,
				    DIG_CLK_CTL_RXD_PDM_CLK_EN);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_RXD_PDM_CLK_EN, 0);
		snd_soc_update_bits(codec, w->reg, 1 << w->shift, 0x00);
		break;
	}
	return 0;
}

static int msm8916_wcd_codec_enable_charge_pump(struct snd_soc_dapm_widget *w,
						struct snd_kcontrol *kcontrol,
						int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct msm8916_wcd_chip *msm8916_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_RXD_PDM_CLK_EN_MASK |
				    DIG_CLK_CTL_NCP_CLK_EN_MASK,
				    DIG_CLK_CTL_RXD_PDM_CLK_EN |
				    DIG_CLK_CTL_NCP_CLK_EN);
		break;
	case SND_SOC_DAPM_POST_PMU:
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_NCP_CLK_EN_MASK, 0);
		if (msm8916_wcd->rx_bias_count == 0)
			snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
					    DIG_CLK_CTL_RXD_PDM_CLK_EN_MASK, 0);
		break;
	}
	return 0;
}

static int msm8916_wcd_codec_enable_rx_bias(struct snd_soc_dapm_widget *w,
					    struct snd_kcontrol *kcontrol,
					    int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct msm8916_wcd_chip *msm8916_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		msm8916_wcd->rx_bias_count++;
		if (msm8916_wcd->rx_bias_count == 1) {
			snd_soc_update_bits(codec, CDC_A_RX_COM_BIAS_DAC,
					    RX_COM_BIAS_DAC_RX_BIAS_EN_MASK |
					    RX_COM_BIAS_DAC_DAC_REF_EN_MASK,
					    RX_COM_BIAS_DAC_RX_BIAS_EN_ENABLE |
					    RX_COM_BIAS_DAC_DAC_REF_EN_ENABLE);
		}

		break;
	case SND_SOC_DAPM_POST_PMD:
		msm8916_wcd->rx_bias_count--;
		if (msm8916_wcd->rx_bias_count == 0) {
			snd_soc_update_bits(codec, CDC_A_RX_COM_BIAS_DAC,
					    RX_COM_BIAS_DAC_RX_BIAS_EN_MASK |
					    RX_COM_BIAS_DAC_DAC_REF_EN_MASK, 0);

		}
		break;
	}

	return 0;
}

static void msm8916_wcd_micbias2_enable(struct snd_soc_codec *codec, bool on)
{
	if (on) {
		snd_soc_update_bits(codec, CDC_A_MICB_1_CTL,
				    MICB_1_CTL_EXT_PRECHARG_EN_MASK |
				    MICB_1_CTL_INT_PRECHARG_BYP_MASK,
				    MICB_1_CTL_INT_PRECHARG_BYP_EXT_PRECHRG_SEL
				    | MICB_1_CTL_EXT_PRECHARG_EN_ENABLE);
		snd_soc_write(codec, CDC_A_MICB_1_VAL,
			      MICB_1_VAL_MICB_OUT_VAL_V2P70V);
		/*
		 * Special headset needs MICBIAS as 2.7V so wait for
		 * 50 msec for the MICBIAS to reach 2.7 volts.
		 */
		msleep(50);
		snd_soc_update_bits(codec, CDC_A_MICB_1_CTL,
				    MICB_1_CTL_EXT_PRECHARG_EN_MASK |
				    MICB_1_CTL_INT_PRECHARG_BYP_MASK, 0);
	}
}

static int msm8916_wcd_codec_enable_micbias(struct snd_soc_dapm_widget *w,
					    struct snd_kcontrol *kcontrol,
					    int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	char *internal1_text = "Internal1";
	char *internal2_text = "Internal2";
	char *internal3_text = "Internal3";
	char *external2_text = "External2";
	char *external_text = "External";
	bool micbias2;

	micbias2 = (snd_soc_read(codec, CDC_A_MICB_2_EN) & 0x80);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (strnstr(w->name, internal1_text, 30)) {
			snd_soc_update_bits(codec, CDC_A_MICB_1_INT_RBIAS,
					    MICB_1_INT_TX1_INT_RBIAS_EN_MASK,
					    MICB_1_INT_TX1_INT_RBIAS_EN_ENABLE);
		} else if (strnstr(w->name, internal2_text, 30)) {
			snd_soc_update_bits(codec, CDC_A_MICB_1_INT_RBIAS,
					    MICB_1_INT_TX2_INT_RBIAS_EN_MASK,
					    MICB_1_INT_TX2_INT_RBIAS_EN_ENABLE);
			snd_soc_update_bits(codec, w->reg,
					    MICB_1_EN_BYP_CAP_MASK |
					    MICB_1_EN_PULL_DOWN_EN_MASK, 0);
		} else if (strnstr(w->name, internal3_text, 30)) {
			snd_soc_update_bits(codec, CDC_A_MICB_1_INT_RBIAS,
					    MICB_1_INT_TX3_INT_RBIAS_EN_MASK,
					    MICB_1_INT_TX3_INT_RBIAS_EN_ENABLE);
		}
		if (!strnstr(w->name, external_text, 30))
			snd_soc_update_bits(codec, CDC_A_MICB_1_EN,
					    MICB_1_EN_OPA_STG2_TAIL_CURR_MASK |
					    MICB_1_EN_TX3_GND_SEL_MASK,
					    MICB_1_EN_OPA_STG2_TAIL_CURR_1_60UA);
		if (w->reg == CDC_A_MICB_1_EN)
			msm8916_wcd_configure_cap(codec, true, micbias2);

		break;
	case SND_SOC_DAPM_POST_PMU:
		if (strnstr(w->name, internal1_text, 30)) {
			snd_soc_update_bits(codec, CDC_A_MICB_1_INT_RBIAS,
					    MICB_1_INT_TX1_INT_PULLUP_EN_MASK,
					    MICB_1_INT_TX1_INT_PULLUP_EN_TX1N_TO_MICBIAS);
		} else if (strnstr(w->name, internal2_text, 30)) {
			snd_soc_update_bits(codec, CDC_A_MICB_1_INT_RBIAS,
					    MICB_1_INT_TX2_INT_PULLUP_EN_MASK,
					    MICB_1_INT_TX2_INT_PULLUP_EN_TX1N_TO_MICBIAS);
			msm8916_wcd_micbias2_enable(codec, true);
			msm8916_wcd_configure_cap(codec, false, true);
		} else if (strnstr(w->name, internal3_text, 30)) {
			snd_soc_update_bits(codec, CDC_A_MICB_1_INT_RBIAS,
					    MICB_1_INT_TX3_INT_PULLUP_EN_MASK,
					    MICB_1_INT_TX3_INT_PULLUP_EN_TX1N_TO_MICBIAS);
		} else if (strnstr(w->name, external2_text, 30)) {
			msm8916_wcd_micbias2_enable(codec, true);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (strnstr(w->name, internal1_text, 30)) {
			snd_soc_update_bits(codec, CDC_A_MICB_1_INT_RBIAS,
					    MICB_1_INT_TX1_INT_RBIAS_EN_MASK,
					    MICB_1_INT_TX1_INT_RBIAS_EN_DISABLE);
		} else if (strnstr(w->name, internal2_text, 30)) {
			msm8916_wcd_micbias2_enable(codec, false);
		} else if (strnstr(w->name, internal3_text, 30)) {
			snd_soc_update_bits(codec, CDC_A_MICB_1_INT_RBIAS,
					    MICB_1_INT_TX3_INT_RBIAS_EN_MASK,
					    MICB_1_INT_TX3_INT_RBIAS_EN_DISABLE);
		} else if (strnstr(w->name, external2_text, 30)) {
			msm8916_wcd_micbias2_enable(codec, false);
			break;
		}
		if (w->reg == CDC_A_MICB_1_EN)
			msm8916_wcd_configure_cap(codec, false, micbias2);
		break;
	}

	return 0;
}

static void msm8916_wcd_codec_enable_adc_block(struct snd_soc_codec *codec,
					       int enable)
{
	if (enable) {
		snd_soc_update_bits(codec, CDC_D_CDC_ANA_CLK_CTL,
				    ANA_CLK_CTL_TXA_CLK25_EN,
				    ANA_CLK_CTL_TXA_CLK25_EN);
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_TXD_CLK_EN,
				    DIG_CLK_CTL_TXD_CLK_EN);
	} else {
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_TXD_CLK_EN, 0);
		snd_soc_update_bits(codec, CDC_D_CDC_ANA_CLK_CTL,
				    ANA_CLK_CTL_TXA_CLK25_EN, 0);
	}
}

static int msm8916_wcd_codec_enable_adc(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	u16 adc_reg = CDC_A_TX_1_2_TEST_CTL_2;
	u8 init_bit_shift;

	if (w->reg == CDC_A_TX_1_EN)
		init_bit_shift = 5;
	else
		init_bit_shift = 4;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		msm8916_wcd_codec_enable_adc_block(codec, 1);
		if (w->reg == CDC_A_TX_2_EN)
			snd_soc_update_bits(codec, CDC_A_MICB_1_CTL,
					    MICB_1_CTL_CFILT_REF_SEL_MASK,
					    MICB_1_CTL_CFILT_REF_SEL_HPF_REF);
		/*
		 * Add delay of 10 ms to give sufficient time for the voltage
		 * to shoot up and settle so that the txfe init does not
		 * happen when the input voltage is changing too much.
		 */
		usleep_range(10000, 10010);
		snd_soc_update_bits(codec, adc_reg, 1 << init_bit_shift,
				    1 << init_bit_shift);
		if (w->reg == CDC_A_TX_1_EN)
			snd_soc_update_bits(codec, CDC_D_CDC_CONN_TX1_CTL,
					    CONN_TX1_SERIAL_TX1_MUX,
					    CONN_TX1_SERIAL_TX1_ADC_1);
		else if ((w->reg == CDC_A_TX_2_EN) || (w->reg == CDC_A_TX_3_EN))
			snd_soc_update_bits(codec, CDC_D_CDC_CONN_TX2_CTL,
					    CONN_TX2_SERIAL_TX2_MUX,
					    CONN_TX2_SERIAL_TX2_ADC_2);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/*
		 * Add delay of 12 ms before deasserting the init
		 * to reduce the tx pop
		 */
		usleep_range(12000, 12010);
		snd_soc_update_bits(codec, adc_reg, 1 << init_bit_shift, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		msm8916_wcd_codec_enable_adc_block(codec, 0);
		if (w->reg == CDC_A_TX_2_EN)
			snd_soc_update_bits(codec, CDC_A_MICB_1_CTL,
					    MICB_1_CTL_CFILT_REF_SEL_MASK, 0);
		if (w->reg == CDC_A_TX_1_EN)
			snd_soc_update_bits(codec, CDC_D_CDC_CONN_TX1_CTL,
					    CONN_TX1_SERIAL_TX1_MUX,
					    CONN_TX1_SERIAL_TX1_ZERO);
		else if ((w->reg == CDC_A_TX_2_EN) || (w->reg == CDC_A_TX_3_EN))
			snd_soc_update_bits(codec, CDC_D_CDC_CONN_TX2_CTL,
					    CONN_TX2_SERIAL_TX2_MUX,
					    CONN_TX2_SERIAL_TX2_ZERO);

		break;
	}
	return 0;
}

static int msm8916_wcd_hphr_dac_event(struct snd_soc_dapm_widget *w,
				      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec,
				    CDC_A_RX_HPH_R_PA_DAC_CTL,
				    RX_HPH_R_PA_DAC_CTL_DATA_RESET_MASK,
				    RX_HPH_R_PA_DAC_CTL_DATA_RESET);
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_RXD2_CLK_EN,
				    DIG_CLK_CTL_RXD2_CLK_EN);
		snd_soc_update_bits(codec, CDC_D_CDC_ANA_CLK_CTL,
				    ANA_CLK_CTL_EAR_HPHR_CLK_EN_MASK,
				    ANA_CLK_CTL_EAR_HPHR_CLK_EN);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, CDC_A_RX_HPH_R_PA_DAC_CTL,
				    RX_HPH_R_PA_DAC_CTL_DATA_RESET_MASK, 0);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, CDC_D_CDC_ANA_CLK_CTL,
				    ANA_CLK_CTL_EAR_HPHR_CLK_EN_MASK, 0);
		snd_soc_update_bits(codec, CDC_D_CDC_DIG_CLK_CTL,
				    DIG_CLK_CTL_RXD2_CLK_EN, 0);
		break;
	}
	return 0;
}

static int msm8916_wcd_codec_enable_interpolator(struct snd_soc_dapm_widget *w,
						 struct snd_kcontrol *kcontrol,
						 int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct msm8916_wcd_chip *msm8916_wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* apply the digital gain after the interpolator is enabled */
		snd_soc_write(codec, rx_gain_reg[w->shift],
			      snd_soc_read(codec, rx_gain_reg[w->shift]));
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, LPASS_CDC_CLK_RX_RESET_CTL,
				    1 << w->shift, 1 << w->shift);
		snd_soc_update_bits(codec, LPASS_CDC_CLK_RX_RESET_CTL,
				    1 << w->shift, 0x0);
		/* disable the mute enabled during the PMD of this device */
		if (msm8916_wcd->mute_mask & MUTE_MASK_HPHL_PA_DISABLE) {
			snd_soc_update_bits(codec,
					    LPASS_CDC_RX1_B6_CTL,
					    RXn_B6_CTL_MUTE_MASK,
					    RXn_B6_CTL_MUTE_DISABLE);
			msm8916_wcd->mute_mask &= ~(MUTE_MASK_HPHL_PA_DISABLE);
		}
		if (msm8916_wcd->mute_mask & MUTE_MASK_HPHR_PA_DISABLE) {
			snd_soc_update_bits(codec, LPASS_CDC_RX2_B6_CTL,
					    RXn_B6_CTL_MUTE_MASK,
					    RXn_B6_CTL_MUTE_DISABLE);

			msm8916_wcd->mute_mask &= ~(MUTE_MASK_HPHR_PA_DISABLE);
		}
		if (msm8916_wcd->mute_mask & MUTE_MASK_SPKR_PA_DISABLE) {
			snd_soc_update_bits(codec, LPASS_CDC_RX3_B6_CTL,
					    RXn_B6_CTL_MUTE_MASK,
					    RXn_B6_CTL_MUTE_DISABLE);

			msm8916_wcd->mute_mask &= ~(MUTE_MASK_SPKR_PA_DISABLE);
		}
	}
	return 0;
}

static int msm8916_wcd_codec_enable_dec(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	unsigned int decimator;
	char *dec_name = NULL;
	char *widget_name = NULL;
	char *temp;
	int ret = 0;
	u16 dec_reset_reg, tx_vol_ctl_reg, tx_mux_ctl_reg;
	u8 dec_hpf_cut_of_freq;
	char *dec_num;

	widget_name = kstrndup(w->name, 15, GFP_KERNEL);
	if (!widget_name)
		return -ENOMEM;
	temp = widget_name;

	dec_name = strsep(&widget_name, " ");
	widget_name = temp;
	if (!dec_name) {
		dev_err(codec->dev, "Invalid decimator = %s\n", w->name);
		ret = -EINVAL;
		goto out;
	}

	dec_num = strpbrk(dec_name, "12");
	if (dec_num == NULL) {
		dev_err(codec->dev, "Invalid Decimator\n");
		ret = -EINVAL;
		goto out;
	}

	ret = kstrtouint(dec_num, 10, &decimator);
	if (ret < 0) {
		dev_err(codec->dev, "Invalid decimator = %s\n", dec_name);
		ret = -EINVAL;
		goto out;
	}

	dec_reset_reg = LPASS_CDC_CLK_TX_RESET_B1_CTL;
	tx_vol_ctl_reg = LPASS_CDC_TX1_VOL_CTL_CFG + 32 * (decimator - 1);
	tx_mux_ctl_reg = LPASS_CDC_TX1_MUX_CTL + 32 * (decimator - 1);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Enableable TX digital mute */
		snd_soc_update_bits(codec, tx_vol_ctl_reg,
				    TX_VOL_CTL_CFG_MUTE_EN_MASK,
				    TX_VOL_CTL_CFG_MUTE_EN_ENABLE);
		dec_hpf_cut_of_freq =
		    snd_soc_read(codec,
				 tx_mux_ctl_reg) & TX_MUX_CTL_CUT_OFF_FREQ_MASK;
		dec_hpf_cut_of_freq >>= TX_MUX_CTL_CUT_OFF_FREQ_SHIFT;
		if (dec_hpf_cut_of_freq !=
		    TX_MUX_CTL_CUT_OFF_FREQ_CF_NEG_3DB_150HZ) {
			/* set cut of freq to CF_MIN_3DB_150HZ (0x1) */
			snd_soc_update_bits(codec, tx_mux_ctl_reg,
					    TX_MUX_CTL_CUT_OFF_FREQ_MASK,
					    TX_MUX_CTL_CUT_OFF_FREQ_CF_NEG_3DB_150HZ);
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* enable HPF */
		snd_soc_update_bits(codec, tx_mux_ctl_reg,
				    TX_MUX_CTL_HPF_BP_SEL_MASK,
				    TX_MUX_CTL_HPF_BP_SEL_NO_BYPASS);
		/* apply the digital gain after the decimator is enabled */
		snd_soc_write(codec, tx_gain_reg[w->shift],
			      snd_soc_read(codec, tx_gain_reg[w->shift]));
		snd_soc_update_bits(codec, tx_vol_ctl_reg,
				    TX_VOL_CTL_CFG_MUTE_EN_MASK, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, tx_vol_ctl_reg,
				    TX_VOL_CTL_CFG_MUTE_EN_MASK,
				    TX_VOL_CTL_CFG_MUTE_EN_ENABLE);
		snd_soc_update_bits(codec, tx_mux_ctl_reg,
				    TX_MUX_CTL_HPF_BP_SEL_MASK,
				    TX_MUX_CTL_HPF_BP_SEL_BYPASS);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, dec_reset_reg, 1 << w->shift,
				    1 << w->shift);
		snd_soc_update_bits(codec, dec_reset_reg, 1 << w->shift, 0x0);
		snd_soc_update_bits(codec, tx_mux_ctl_reg,
				    TX_MUX_CTL_HPF_BP_SEL_MASK,
				    TX_MUX_CTL_HPF_BP_SEL_BYPASS);
		snd_soc_update_bits(codec, tx_vol_ctl_reg,
				    TX_VOL_CTL_CFG_MUTE_EN_MASK, 0);
		break;
	}

out:
	kfree(widget_name);
	return ret;
}

static int msm8916_wcd_codec_enable_dmic(struct snd_soc_dapm_widget *w,
					 struct snd_kcontrol *kcontrol,
					 int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct msm8916_wcd_chip *msm8916_wcd = snd_soc_codec_get_drvdata(codec);
	u16 dmic_clk_reg = LPASS_CDC_CLK_DMIC_B1_CTL;
	unsigned int dmic;
	int ret;
	char *dec_num = strpbrk(w->name, "12");

	if (dec_num == NULL) {
		dev_err(codec->dev, "Invalid DMIC\n");
		return -EINVAL;
	}

	ret = kstrtouint(dec_num, 10, &dmic);
	if (ret < 0) {
		dev_err(codec->dev, "Invalid DMIC line on the codec\n");
		return -EINVAL;
	}
	if (dmic > 2) {
		dev_err(codec->dev, "%s: Invalid DMIC Selection\n", __func__);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (++msm8916_wcd->dmic_clk_cnt == 1) {
			snd_soc_update_bits(codec, dmic_clk_reg,
					    DMIC_B1_CTL_DMIC0_CLK_SEL_MASK,
					    DMIC_B1_CTL_DMIC0_CLK_SEL_DIV3);
			snd_soc_update_bits(codec, dmic_clk_reg,
					    DMIC_B1_CTL_DMIC0_CLK_EN_MASK,
					    DMIC_B1_CTL_DMIC0_CLK_EN_ENABLE);
		}
		if (dmic == 1)
			snd_soc_update_bits(codec, LPASS_CDC_TX1_DMIC_CTL,
					    TXN_DMIC_CTL_CLK_SEL_MASK,
					    TXN_DMIC_CTL_CLK_SEL_DIV3);
		if (dmic == 2)
			snd_soc_update_bits(codec, LPASS_CDC_TX2_DMIC_CTL,
					    TXN_DMIC_CTL_CLK_SEL_MASK,
					    TXN_DMIC_CTL_CLK_SEL_DIV3);
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (--msm8916_wcd->dmic_clk_cnt == 0)
			snd_soc_update_bits(codec, dmic_clk_reg,
					    DMIC_B1_CTL_DMIC0_CLK_EN_MASK, 0);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget msm8916_wcd_dapm_widgets[] = {
	/*RX stuff */
	SND_SOC_DAPM_AIF_IN("I2S RX1", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S RX2", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S RX3", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SUPPLY("INT_LDO_H", SND_SOC_NOPM, 1, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("HEADPHONE"),
	SND_SOC_DAPM_PGA_E("HPHL PA", CDC_A_RX_HPH_CNP_EN,
			   5, 0, NULL, 0,
			   msm8916_wcd_hph_pa_event, SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX("HPHL", SND_SOC_NOPM, 0, 0, &hphl_mux),
	SND_SOC_DAPM_MIXER_E("HPHL DAC",
			     CDC_A_RX_HPH_L_PA_DAC_CTL, 3, 0, NULL,
			     0, msm8916_wcd_hphl_dac_event,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			     SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("HPHR PA", CDC_A_RX_HPH_CNP_EN,
			   4, 0, NULL, 0,
			   msm8916_wcd_hph_pa_event, SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX("HPHR", SND_SOC_NOPM, 0, 0, &hphr_mux),
	SND_SOC_DAPM_MIXER_E("HPHR DAC",
			     CDC_A_RX_HPH_R_PA_DAC_CTL, 3, 0, NULL,
			     0, msm8916_wcd_hphr_dac_event,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			     SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("SPK DAC", SND_SOC_NOPM, 0, 0,
			   spkr_switch, ARRAY_SIZE(spkr_switch)),

	/* Speaker */
	SND_SOC_DAPM_OUTPUT("SPK_OUT"),
	SND_SOC_DAPM_PGA_E("SPK PA", CDC_A_SPKR_DRV_CTL,
			   6, 0, NULL, 0, msm8916_wcd_codec_enable_spk_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("RX1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER_E("RX1 MIX2", LPASS_CDC_CLK_RX_B1_CTL, 0, 0, NULL,
			     0, msm8916_wcd_codec_enable_interpolator,
			     SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX2 MIX2", LPASS_CDC_CLK_RX_B1_CTL, 1, 0, NULL,
			     0, msm8916_wcd_codec_enable_interpolator,
			     SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX3 MIX1", LPASS_CDC_CLK_RX_B1_CTL, 2, 0, NULL,
			     0, msm8916_wcd_codec_enable_interpolator,
			     SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("RX1 CLK", CDC_D_CDC_DIG_CLK_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("RX2 CLK", CDC_D_CDC_DIG_CLK_CTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("RX3 CLK", CDC_D_CDC_DIG_CLK_CTL,
			    2, 0, msm8916_wcd_codec_enable_dig_clk,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX1 CHAIN", LPASS_CDC_RX1_B6_CTL, 0, 0,
			     NULL, 0, msm8916_wcd_codec_enable_rx_chain,
			     SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX2 CHAIN", LPASS_CDC_RX2_B6_CTL, 0, 0,
			     NULL, 0, msm8916_wcd_codec_enable_rx_chain,
			     SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX3 CHAIN", LPASS_CDC_RX3_B6_CTL, 0, 0,
			     NULL, 0, msm8916_wcd_codec_enable_rx_chain,
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
	SND_SOC_DAPM_REGULATOR_SUPPLY("vdd-micbias", 0, 0),
	SND_SOC_DAPM_SUPPLY("CP", CDC_A_NCP_EN, 0, 0,
			    msm8916_wcd_codec_enable_charge_pump,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("RX_BIAS", SND_SOC_NOPM, 0, 0,
			    msm8916_wcd_codec_enable_rx_bias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("SPK_RX_BIAS", SND_SOC_NOPM, 0, 0,
			    msm8916_wcd_codec_enable_rx_bias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* TX */
	SND_SOC_DAPM_SUPPLY_S("CDC_CONN", -2, LPASS_CDC_CLK_OTHR_CTL,
			      2, 0, NULL, 0),
	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_SUPPLY("MIC BIAS Internal1", CDC_A_MICB_1_EN, 7, 0,
			    msm8916_wcd_codec_enable_micbias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS Internal2", CDC_A_MICB_2_EN, 7, 0,
			    msm8916_wcd_codec_enable_micbias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS Internal3", CDC_A_MICB_1_EN, 7, 0,
			    msm8916_wcd_codec_enable_micbias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC1", NULL, CDC_A_TX_1_EN, 7, 0,
			   msm8916_wcd_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC2_INP2", NULL, CDC_A_TX_2_EN, 7, 0,
			   msm8916_wcd_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC2_INP3", NULL, CDC_A_TX_3_EN, 7, 0,
			   msm8916_wcd_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("ADC2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("ADC3", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MUX("ADC2 MUX", SND_SOC_NOPM, 0, 0, &tx_adc2_mux),
	SND_SOC_DAPM_SUPPLY("MIC BIAS External", CDC_A_MICB_1_EN, 7, 0,
			    msm8916_wcd_codec_enable_micbias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS External2", CDC_A_MICB_2_EN, 7, 0,
			    msm8916_wcd_codec_enable_micbias,
			    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_INPUT("AMIC3"),
	SND_SOC_DAPM_MUX_E("DEC1 MUX", LPASS_CDC_CLK_TX_CLK_EN_B1_CTL, 0, 0,
			   &dec1_mux, msm8916_wcd_codec_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("DEC2 MUX", LPASS_CDC_CLK_TX_CLK_EN_B1_CTL, 1, 0,
			   &dec2_mux, msm8916_wcd_codec_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX("RDAC2 MUX", SND_SOC_NOPM, 0, 0, &rdac2_mux),
	SND_SOC_DAPM_INPUT("AMIC2"),
	SND_SOC_DAPM_AIF_OUT("I2S TX1", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S TX2", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S TX3", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),

	/* Digital Mic Inputs */
	SND_SOC_DAPM_ADC_E("DMIC1", NULL, SND_SOC_NOPM, 0, 0,
			   msm8916_wcd_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC2", NULL, SND_SOC_NOPM, 0, 0,
			   msm8916_wcd_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("RX_I2S_CLK", LPASS_CDC_CLK_RX_I2S_CTL,
			    4, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("TX_I2S_CLK", LPASS_CDC_CLK_TX_I2S_CTL, 4, 0,
			    NULL, 0),
};

static int msm8916_wcd_codec_parse_dt(struct platform_device *pdev,
				      struct msm8916_wcd_chip *chip)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;
	struct regulator_bulk_data regs[2];
	const char *ext1_cap = "qcom,micbias1-ext-cap";
	const char *ext2_cap = "qcom,micbias2-ext-cap";
	u32 res[2];

	ret = of_property_read_u32_array(np, "reg", res, 2);
	if (ret < 0)
		return ret;

	if (of_property_read_bool(pdev->dev.of_node, ext1_cap))
		chip->micbias1_cap_mode = MICB_1_EN_EXT_BYP_CAP;
	else
		chip->micbias1_cap_mode = MICB_1_EN_NO_EXT_BYP_CAP;

	if (of_property_read_bool(pdev->dev.of_node, ext2_cap))
		chip->micbias2_cap_mode = MICB_1_EN_EXT_BYP_CAP;
	else
		chip->micbias2_cap_mode = MICB_1_EN_NO_EXT_BYP_CAP;

	chip->analog_offset = res[0];
	chip->digital_map = syscon_regmap_lookup_by_phandle(np,
							    "qcom,lpass-codec-core");
	if (IS_ERR(chip->digital_map))
		return PTR_ERR(chip->digital_map);

	chip->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(chip->mclk)) {
		dev_err(dev, "failed to get mclk\n");
		return PTR_ERR(chip->mclk);
	}

	regs[0].supply = "vddio";
	regs[1].supply = "vdd-tx-rx";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(regs), regs);
	if (ret) {
		dev_err(dev, "Failed to get regulator supplies %d\n", ret);
		return ret;
	}
	chip->vddio = regs[0].consumer;
	chip->vdd_tx_rx = regs[1].consumer;

	return 0;
}

static int msm8916_wcd_codec_enable_clock_block(struct snd_soc_codec *codec,
						int enable)
{
	struct msm8916_wcd_chip *msm8916_wcd = snd_soc_codec_get_drvdata(codec);
	unsigned long mclk_rate;

	if (enable) {
		snd_soc_update_bits(codec, LPASS_CDC_CLK_MCLK_CTL,
				    MCLK_CTL_MCLK_EN_MASK,
				    MCLK_CTL_MCLK_EN_ENABLE);
		snd_soc_update_bits(codec, LPASS_CDC_CLK_PDM_CTL,
				    LPASS_CDC_CLK_PDM_CTL_PDM_EN_MASK |
				    LPASS_CDC_CLK_PDM_CTL_PDM_CLK_SEL_MASK,
				    LPASS_CDC_CLK_PDM_CTL_PDM_EN |
				    LPASS_CDC_CLK_PDM_CTL_PDM_CLK_SEL_FB);
		snd_soc_update_bits(codec, CDC_A_MASTER_BIAS_CTL,
				    MASTER_BIAS_CTL_MASTER_BIAS_EN_MASK |
				    MASTER_BIAS_CTL_V2L_BUFFER_EN_MASK,
				    MASTER_BIAS_CTL_MASTER_BIAS_EN_ENABLE |
				    MASTER_BIAS_CTL_V2L_BUFFER_EN_ENABLE);
		snd_soc_update_bits(codec, CDC_D_CDC_RST_CTL,
				    RST_CTL_DIG_SW_RST_N_MASK,
				    RST_CTL_DIG_SW_RST_N_REMOVE_RESET);

		snd_soc_update_bits(codec, CDC_D_CDC_TOP_CLK_CTL,
				    TOP_CLK_CTL_A_MCLK_MCLK2_EN_MASK,
				    TOP_CLK_CTL_A_MCLK_EN_ENABLE |
				    TOP_CLK_CTL_A_MCLK2_EN_ENABLE);

		mclk_rate = clk_get_rate(msm8916_wcd->mclk);

		if (mclk_rate == 12288000)
			snd_soc_update_bits(codec, LPASS_CDC_TOP_CTL,
					    TOP_CTL_DIG_MCLK_FREQ_MASK,
					    TOP_CTL_DIG_MCLK_FREQ_F_12_288MHZ);

		else if (mclk_rate == 9600000)
			snd_soc_update_bits(codec, LPASS_CDC_TOP_CTL,
					    TOP_CTL_DIG_MCLK_FREQ_MASK,
					    TOP_CTL_DIG_MCLK_FREQ_F_9_6MHZ);
	} else {
		snd_soc_update_bits(codec, CDC_D_CDC_TOP_CLK_CTL,
				    TOP_CLK_CTL_A_MCLK_MCLK2_EN_MASK, 0);
		snd_soc_update_bits(codec, LPASS_CDC_CLK_PDM_CTL,
				    LPASS_CDC_CLK_PDM_CTL_PDM_EN_MASK |
				    LPASS_CDC_CLK_PDM_CTL_PDM_CLK_SEL_MASK, 0);

	}
	return 0;
}

static const struct msm8916_wcd_reg_mask_val wcd_reg_defaults[] = {
	MSM8916_WCD_REG_VAL(CDC_A_SPKR_DAC_CTL, 0x03),
	MSM8916_WCD_REG_VAL(CDC_A_CURRENT_LIMIT, 0x82),
	MSM8916_WCD_REG_VAL(CDC_A_SPKR_OCP_CTL, 0xE1),
};

static const struct msm8916_wcd_reg_mask_val wcd_reg_defaults_2_0[] = {
	MSM8916_WCD_REG_VAL(CDC_D_SEC_ACCESS, 0xA5),
	MSM8916_WCD_REG_VAL(CDC_D_PERPH_RESET_CTL3, 0x0F),
	MSM8916_WCD_REG_VAL(CDC_A_TX_1_2_OPAMP_BIAS, 0x4F),
	MSM8916_WCD_REG_VAL(CDC_A_NCP_FBCTRL, 0x28),
	MSM8916_WCD_REG_VAL(CDC_A_SPKR_DRV_CTL, 0x69),
	MSM8916_WCD_REG_VAL(CDC_A_SPKR_DRV_DBG, 0x01),
	MSM8916_WCD_REG_VAL(CDC_A_BOOST_EN_CTL, 0x5F),
	MSM8916_WCD_REG_VAL(CDC_A_SLOPE_COMP_IP_ZERO, 0x88),
	MSM8916_WCD_REG_VAL(CDC_A_SEC_ACCESS, 0xA5),
	MSM8916_WCD_REG_VAL(CDC_A_PERPH_RESET_CTL3, 0x0F),
	MSM8916_WCD_REG_VAL(CDC_A_CURRENT_LIMIT, 0x82),
	MSM8916_WCD_REG_VAL(CDC_A_SPKR_DAC_CTL, 0x03),
	MSM8916_WCD_REG_VAL(CDC_A_SPKR_OCP_CTL, 0xE1),
};

static const struct msm8916_wcd_reg_mask_val msm8916_wcd_reg_init_val[] = {
	/**
	 * Initialize current threshold to 350MA
	 * number of wait and run cycles to 4096
	 */
	{CDC_A_RX_COM_OCP_CTL, 0xFF, 0xD1},
	{CDC_A_RX_COM_OCP_COUNT, 0xFF, 0xFF},
};

static int msm8916_wcd_device_up(struct snd_soc_codec *codec)
{
	struct msm8916_wcd_chip *msm8916_wcd = snd_soc_codec_get_drvdata(codec);
	u32 reg;

	snd_soc_write(codec, CDC_D_PERPH_RESET_CTL4, 0x01);
	snd_soc_write(codec, CDC_A_PERPH_RESET_CTL4, 0x01);

	for (reg = 0; reg < ARRAY_SIZE(msm8916_wcd_reg_init_val); reg++)
		snd_soc_update_bits(codec,
				    msm8916_wcd_reg_init_val[reg].reg,
				    msm8916_wcd_reg_init_val[reg].mask,
				    msm8916_wcd_reg_init_val[reg].val);

	if (TOMBAK_IS_1_0(msm8916_wcd->pmic_rev)) {
		for (reg = 0; reg < ARRAY_SIZE(wcd_reg_defaults); reg++)
			snd_soc_write(codec, wcd_reg_defaults[reg].reg,
				      wcd_reg_defaults[reg].val);
	} else {
		for (reg = 0; reg < ARRAY_SIZE(wcd_reg_defaults_2_0); reg++)
			snd_soc_write(codec, wcd_reg_defaults_2_0[reg].reg,
				      wcd_reg_defaults_2_0[reg].val);
	}

	return 0;
}

static int msm8916_wcd_codec_probe(struct snd_soc_codec *codec)
{
	struct msm8916_wcd_chip *chip = dev_get_drvdata(codec->dev);
	int err;

	err = regulator_enable(chip->vddio);
	if (err < 0) {
		dev_err(codec->dev, "failed to enable VDDIO regulator\n");
		return err;
	}

	err = regulator_enable(chip->vdd_tx_rx);
	if (err < 0) {
		dev_err(codec->dev, "failed to enable VDD_TX_RX regulator\n");
		return err;
	}

	snd_soc_codec_set_drvdata(codec, chip);
	chip->pmic_rev = snd_soc_read(codec, CDC_D_REVISION1);
	chip->codec_version = snd_soc_read(codec, CDC_D_PERPH_SUBTYPE);
	dev_info(codec->dev, "PMIC REV: %d\t CODEC Version: %d\n",
		 chip->pmic_rev, chip->codec_version);

	msm8916_wcd_device_up(codec);
	/* Set initial cap mode */
	msm8916_wcd_configure_cap(codec, false, false);

	return 0;
}

static int msm8916_wcd_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	u8 tx_fs_rate;

	switch (params_rate(params)) {
	case 8000:
		tx_fs_rate = TX_I2S_CTL_TX_I2S_FS_RATE_F_8_KHZ;
		break;
	case 16000:
		tx_fs_rate = TX_I2S_CTL_TX_I2S_FS_RATE_F_16_KHZ;
		break;
	case 32000:
		tx_fs_rate = TX_I2S_CTL_TX_I2S_FS_RATE_F_32_KHZ;
		break;
	case 48000:
		tx_fs_rate = TX_I2S_CTL_TX_I2S_FS_RATE_F_48_KHZ;
		break;
	case 96000:
		tx_fs_rate = TX_I2S_CTL_TX_I2S_FS_RATE_F_96_KHZ;
		break;
	case 192000:
		tx_fs_rate = TX_I2S_CTL_TX_I2S_FS_RATE_F_192_KHZ;
		break;
	default:
		dev_err(dai->codec->dev, "Invalid sampling rate %d\n",
			params_rate(params));
		return -EINVAL;
	}

	switch (substream->stream) {
	case SNDRV_PCM_STREAM_CAPTURE:
		snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_TX_I2S_CTL,
				    TX_I2S_CTL_TX_I2S_FS_RATE_MASK, tx_fs_rate);
		break;
	case SNDRV_PCM_STREAM_PLAYBACK:
		break;
	default:
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_TX_I2S_CTL,
				    TX_I2S_CTL_TX_I2S_MODE_MASK,
				    TX_I2S_CTL_TX_I2S_MODE_16);
		snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_RX_I2S_CTL,
				    RX_I2S_CTL_RX_I2S_MODE_MASK,
				    RX_I2S_CTL_RX_I2S_MODE_16);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_TX_I2S_CTL,
				    TX_I2S_CTL_TX_I2S_MODE_MASK,
				    TX_I2S_CTL_TX_I2S_MODE_32);
		snd_soc_update_bits(dai->codec, LPASS_CDC_CLK_RX_I2S_CTL,
				    RX_I2S_CTL_RX_I2S_MODE_MASK,
				    RX_I2S_CTL_RX_I2S_MODE_32);
		break;
	default:
		dev_err(dai->dev, "%s: wrong format selected\n", __func__);
		return -EINVAL;
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
	{"RX2 MIX1", NULL, "RX2 MIX1 INP3"},
	{"RX3 MIX1", NULL, "RX3 MIX1 INP1"},
	{"RX3 MIX1", NULL, "RX3 MIX1 INP2"},
	{"RX3 MIX1", NULL, "RX3 MIX1 INP3"},
	{"RX1 MIX2", NULL, "RX1 MIX1"},
	{"RX2 MIX2", NULL, "RX2 MIX1"},

	{"RX1 MIX1 INP1", "RX1", "I2S RX1"},
	{"RX1 MIX1 INP1", "RX2", "I2S RX2"},
	{"RX1 MIX1 INP1", "RX3", "I2S RX3"},
	{"RX1 MIX1 INP2", "RX1", "I2S RX1"},
	{"RX1 MIX1 INP2", "RX2", "I2S RX2"},
	{"RX1 MIX1 INP2", "RX3", "I2S RX3"},
	{"RX1 MIX1 INP3", "RX1", "I2S RX1"},
	{"RX1 MIX1 INP3", "RX2", "I2S RX2"},
	{"RX1 MIX1 INP3", "RX3", "I2S RX3"},

	{"RX2 MIX1 INP1", "RX1", "I2S RX1"},
	{"RX2 MIX1 INP1", "RX2", "I2S RX2"},
	{"RX2 MIX1 INP1", "RX3", "I2S RX3"},
	{"RX2 MIX1 INP2", "RX1", "I2S RX1"},
	{"RX2 MIX1 INP2", "RX2", "I2S RX2"},
	{"RX2 MIX1 INP2", "RX3", "I2S RX3"},
	{"RX2 MIX1 INP3", "RX1", "I2S RX1"},
	{"RX2 MIX1 INP3", "RX2", "I2S RX2"},
	{"RX2 MIX1 INP3", "RX3", "I2S RX3"},

	{"RX3 MIX1 INP1", "RX1", "I2S RX1"},
	{"RX3 MIX1 INP1", "RX2", "I2S RX2"},
	{"RX3 MIX1 INP1", "RX3", "I2S RX3"},
	{"RX3 MIX1 INP2", "RX1", "I2S RX1"},
	{"RX3 MIX1 INP2", "RX2", "I2S RX2"},
	{"RX3 MIX1 INP2", "RX3", "I2S RX3"},
	{"RX3 MIX1 INP3", "RX1", "I2S RX1"},
	{"RX3 MIX1 INP3", "RX2", "I2S RX2"},
	{"RX3 MIX1 INP3", "RX3", "I2S RX3"},

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

	{"MIC BIAS Internal1", NULL, "INT_LDO_H"},
	{"MIC BIAS Internal2", NULL, "INT_LDO_H"},
	{"MIC BIAS External", NULL, "INT_LDO_H"},
	{"MIC BIAS External2", NULL, "INT_LDO_H"},
	{"MIC BIAS Internal1", NULL, "vdd-micbias"},
	{"MIC BIAS Internal2", NULL, "vdd-micbias"},
	{"MIC BIAS External", NULL, "vdd-micbias"},
	{"MIC BIAS External2", NULL, "vdd-micbias"},
};

static int msm8916_wcd_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	msm8916_wcd_codec_enable_clock_block(dai->codec, 1);
	return 0;
}

static void msm8916_wcd_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	msm8916_wcd_codec_enable_clock_block(dai->codec, 0);
}

static struct snd_soc_dai_ops msm8916_wcd_dai_ops = {
	.startup = msm8916_wcd_startup,
	.shutdown = msm8916_wcd_shutdown,
	.hw_params = msm8916_wcd_hw_params,
};

static struct snd_soc_dai_driver msm8916_wcd_codec_dai[] = {
	[0] = {
	       .name = "msm8916_wcd_i2s_rx1",
	       .id = MSM8916_WCD_PLAYBACK_DAI,
	       .playback = {
			    .stream_name = "AIF1 Playback",
			    .rates = MSM8916_WCD_RATES,
			    .formats = MSM8916_WCD_FORMATS,
			    .rate_max = 192000,
			    .rate_min = 8000,
			    .channels_min = 1,
			    .channels_max = 3,
			    },
	       .ops = &msm8916_wcd_dai_ops,
	       },
	[1] = {
	       .name = "msm8916_wcd_i2s_tx1",
	       .id = MSM8916_WCD_CAPTURE_DAI,
	       .capture = {
			   .stream_name = "AIF1 Capture",
			   .rates = MSM8916_WCD_RATES,
			   .formats = MSM8916_WCD_FORMATS,
			   .rate_max = 192000,
			   .rate_min = 8000,
			   .channels_min = 1,
			   .channels_max = 4,
			   },
	       .ops = &msm8916_wcd_dai_ops,
	       },
};

static struct snd_soc_codec_driver msm8916_wcd_codec = {
	.probe = msm8916_wcd_codec_probe,
	.read = msm8916_wcd_read,
	.write = msm8916_wcd_write,
	.reg_cache_size = MSM8916_WCD_NUM_REGISTERS,
	.reg_cache_default = msm8916_wcd_reset_reg_defaults,
	.reg_word_size = 1,
	.controls = msm8916_wcd_snd_controls,
	.num_controls = ARRAY_SIZE(msm8916_wcd_snd_controls),
	.dapm_widgets = msm8916_wcd_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(msm8916_wcd_dapm_widgets),
	.dapm_routes = audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),
};

static int msm8916_wcd_probe(struct platform_device *pdev)
{
	struct msm8916_wcd_chip *chip;
	struct device *dev = &pdev->dev;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->analog_map = dev_get_regmap(dev->parent, NULL);
	if (!chip->analog_map)
		return -ENXIO;

	ret = msm8916_wcd_codec_parse_dt(pdev, chip);
	if (IS_ERR_VALUE(ret))
		return ret;

	ret = clk_prepare_enable(chip->mclk);
	if (ret < 0) {
		dev_err(dev, "failed to enable mclk %d\n", ret);
		return ret;
	}

	dev_set_drvdata(dev, chip);

	return snd_soc_register_codec(dev, &msm8916_wcd_codec,
				      msm8916_wcd_codec_dai,
				      ARRAY_SIZE(msm8916_wcd_codec_dai));
}

static int msm8916_wcd_remove(struct platform_device *pdev)
{
	struct msm8916_wcd_chip *chip = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(chip->mclk);
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static const struct of_device_id msm8916_wcd_match_table[] = {
	{.compatible = "qcom,msm8916-wcd-codec"},
	{}
};

MODULE_DEVICE_TABLE(of, msm8916_wcd_match_table);

static struct platform_driver msm8916_wcd_driver = {
	.driver = {
		   .name = "msm8916-wcd-codec",
		   .of_match_table = msm8916_wcd_match_table,
		   },
	.probe = msm8916_wcd_probe,
	.remove = msm8916_wcd_remove,
};

module_platform_driver(msm8916_wcd_driver);

MODULE_ALIAS("platform:spmi-wcd-codec");
MODULE_DESCRIPTION("SPMI PMIC WCD codec driver");
MODULE_LICENSE("GPL v2");
