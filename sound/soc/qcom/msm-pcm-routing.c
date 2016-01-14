/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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


#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/q6adm.h>
#include <sound/q6asm.h>
#include <sound/q6afe.h>
#include <sound/tlv.h>
#include "msm-pcm-routing.h"

struct msm_pcm_routing_bdai_data {
	u16 port_id; /* AFE port ID */
	u8 active; /* track if this backend is enabled */
	unsigned long fe_sessions; /* Front-end sessions */
	unsigned long port_sessions; /* track Tx BE ports -> Rx BE */
	unsigned int  sample_rate;
	unsigned int  channel;
	bool perf_mode;
};

struct msm_pcm_routing_fdai_data {
	u16 be_srate; /* track prior backend sample rate for flushing purpose */
	int strm_id; /* ASM stream ID */
	struct msm_pcm_routing_evt event_info;
};

#define INVALID_SESSION -1
#define SESSION_TYPE_RX 0
#define SESSION_TYPE_TX 1

static DEFINE_MUTEX(routing_lock);

static int srs_alsa_ctrl_ever_called;

#define INT_RX_VOL_MAX_STEPS 0x2000
#define INT_RX_VOL_GAIN 0x2000
#define INT_RX_LR_VOL_MAX_STEPS 0x20002000

static const DECLARE_TLV_DB_LINEAR(fm_rx_vol_gain, 0,
			INT_RX_VOL_MAX_STEPS);

static const DECLARE_TLV_DB_LINEAR(lpa_rx_vol_gain, 0,
			INT_RX_LR_VOL_MAX_STEPS);

static const DECLARE_TLV_DB_LINEAR(multimedia2_rx_vol_gain, 0,
			INT_RX_VOL_MAX_STEPS);

static const DECLARE_TLV_DB_LINEAR(multimedia5_rx_vol_gain, 0,
			INT_RX_VOL_MAX_STEPS);

static const DECLARE_TLV_DB_LINEAR(compressed_rx_vol_gain, 0,
			INT_RX_LR_VOL_MAX_STEPS);

/* Equal to Frontend after last of the MULTIMEDIA SESSIONS */
#define MAX_EQ_SESSIONS		MSM_FRONTEND_DAI_CS_VOICE

enum {
	EQ_BAND1 = 0,
	EQ_BAND2,
	EQ_BAND3,
	EQ_BAND4,
	EQ_BAND5,
	EQ_BAND6,
	EQ_BAND7,
	EQ_BAND8,
	EQ_BAND9,
	EQ_BAND10,
	EQ_BAND11,
	EQ_BAND12,
	EQ_BAND_MAX,
};

struct msm_audio_eq_band {
	uint16_t     band_idx; /* The band index, 0 .. 11 */
	uint32_t     filter_type; /* Filter band type */
	uint32_t     center_freq_hz; /* Filter band center frequency */
	uint32_t     filter_gain; /* Filter band initial gain (dB) */
			/* Range is +12 dB to -12 dB with 1dB increments. */
	uint32_t     q_factor;
} __packed;

struct msm_audio_eq_stream_config {
	uint32_t	enable; /* Number of consequtive bands specified */
	uint32_t	num_bands;
	struct msm_audio_eq_band	eq_bands[EQ_BAND_MAX];
} __packed;

struct msm_audio_eq_stream_config	eq_data[MAX_EQ_SESSIONS];

static void msm_send_eq_values(int eq_idx);
/* This array is indexed by back-end DAI ID defined in msm-pcm-routing.h
 * If new back-end is defined, add new back-end DAI ID at the end of enum
 */

union srs_trumedia_params_u {
	struct srs_trumedia_params srs_params;
	unsigned short int raw_params[1];
};
static union srs_trumedia_params_u msm_srs_trumedia_params[2];
static int srs_port_id = -1;

static void srs_send_params(int port_id, unsigned int techs,
		int param_block_idx) {
	/* only send commands to dsp if srs alsa ctrl was used
	   at least one time */
	if (!srs_alsa_ctrl_ever_called)
		return;

	pr_debug("SRS %s: called, port_id = %d, techs flags = %u,"
			" paramblockidx %d", __func__, port_id, techs,
			param_block_idx);
	/* force all if techs is set to 1 */
	if (techs == 1)
		techs = 0xFFFFFFFF;

	if (techs & (1 << SRS_ID_WOWHD))
		srs_trumedia_open(port_id, SRS_ID_WOWHD,
	(void *)&msm_srs_trumedia_params[param_block_idx].srs_params.wowhd);
	if (techs & (1 << SRS_ID_CSHP))
		srs_trumedia_open(port_id, SRS_ID_CSHP,
	(void *)&msm_srs_trumedia_params[param_block_idx].srs_params.cshp);
	if (techs & (1 << SRS_ID_HPF))
		srs_trumedia_open(port_id, SRS_ID_HPF,
	(void *)&msm_srs_trumedia_params[param_block_idx].srs_params.hpf);
	if (techs & (1 << SRS_ID_PEQ))
		srs_trumedia_open(port_id, SRS_ID_PEQ,
	(void *)&msm_srs_trumedia_params[param_block_idx].srs_params.peq);
	if (techs & (1 << SRS_ID_HL))
		srs_trumedia_open(port_id, SRS_ID_HL,
	(void *)&msm_srs_trumedia_params[param_block_idx].srs_params.hl);
	if (techs & (1 << SRS_ID_GLOBAL))
		srs_trumedia_open(port_id, SRS_ID_GLOBAL,
	(void *)&msm_srs_trumedia_params[param_block_idx].srs_params.global);
}

static struct msm_pcm_routing_bdai_data msm_bedais[MSM_BACKEND_DAI_MAX] = {
	{ PRIMARY_I2S_RX, 0, 0, 0, 0, 0},
	{ PRIMARY_I2S_TX, 0, 0, 0, 0, 0},
	{ SLIMBUS_0_RX, 0, 0, 0, 0, 0},
	{ SLIMBUS_0_TX, 0, 0, 0, 0, 0},
	{ HDMI_RX, 0, 0, 0, 0, 0},
	{ INT_BT_SCO_RX, 0, 0, 0, 0, 0},
	{ INT_BT_SCO_TX, 0, 0, 0, 0, 0},
	{ INT_FM_RX, 0, 0, 0, 0, 0},
	{ INT_FM_TX, 0, 0, 0, 0, 0},
	{ RT_PROXY_PORT_001_RX, 0, 0, 0, 0, 0},
	{ RT_PROXY_PORT_001_TX, 0, 0, 0, 0, 0},
	{ PCM_RX, 0, 0, 0, 0, 0},
	{ PCM_TX, 0, 0, 0, 0, 0},
	{ VOICE_PLAYBACK_TX, 0, 0, 0, 0, 0},
	{ VOICE_RECORD_RX, 0, 0, 0, 0, 0},
	{ VOICE_RECORD_TX, 0, 0, 0, 0, 0},
	{ MI2S_RX, 0, 0, 0, 0, 0},
	{ MI2S_TX, 0, 0, 0, 0},
	{ SECONDARY_I2S_RX, 0, 0, 0, 0, 0},
	{ SLIMBUS_1_RX, 0, 0, 0, 0, 0},
	{ SLIMBUS_1_TX, 0, 0, 0, 0, 0},
	{ SLIMBUS_4_RX, 0, 0, 0, 0, 0},
	{ SLIMBUS_4_TX, 0, 0, 0, 0, 0},
	{ SLIMBUS_3_RX, 0, 0, 0, 0, 0},
	{ SLIMBUS_3_TX, 0, 0, 0, 0, 0},
	{ SLIMBUS_EXTPROC_RX, 0, 0, 0, 0, 0},
	{ SLIMBUS_EXTPROC_RX, 0, 0, 0, 0, 0},
	{ SLIMBUS_EXTPROC_RX, 0, 0, 0, 0, 0},
	{ SECONDARY_PCM_RX, 0, 0, 0, 0, 0},
	{ SECONDARY_PCM_TX, 0, 0, 0, 0, 0},
};

/* Track ASM playback & capture sessions of DAI */
static struct msm_pcm_routing_fdai_data
	fe_dai_map[MSM_FRONTEND_DAI_MM_SIZE][2] = {
	/* MULTIMEDIA1 */
	{{0, INVALID_SESSION, {NULL, NULL} },
	{0, INVALID_SESSION, {NULL, NULL} } },
	/* MULTIMEDIA2 */
	{{0, INVALID_SESSION, {NULL, NULL} },
	{0, INVALID_SESSION, {NULL, NULL} } },
	/* MULTIMEDIA3 */
	{{0, INVALID_SESSION, {NULL, NULL} },
	{0, INVALID_SESSION, {NULL, NULL} } },
	/* MULTIMEDIA4 */
	{{0, INVALID_SESSION, {NULL, NULL} },
	{0, INVALID_SESSION,  {NULL, NULL} } },
	/* MULTIMEDIA5 */
	{{0, INVALID_SESSION, {NULL, NULL} },
	{0, INVALID_SESSION, {NULL, NULL} } },
	/* MULTIMEDIA6 */
	{{0, INVALID_SESSION, {NULL, NULL} },
	{0, INVALID_SESSION, {NULL, NULL} } },
	/* MULTIMEDIA7*/
	{{0, INVALID_SESSION, {NULL, NULL} },
	{0, INVALID_SESSION, {NULL, NULL} } },
	/* MULTIMEDIA8 */
	{{0, INVALID_SESSION, {NULL, NULL} },
	{0, INVALID_SESSION, {NULL, NULL} } },
};

static uint8_t is_be_dai_extproc(int be_dai)
{
	if (be_dai == MSM_BACKEND_DAI_EXTPROC_RX ||
	   be_dai == MSM_BACKEND_DAI_EXTPROC_TX ||
	   be_dai == MSM_BACKEND_DAI_EXTPROC_EC_TX)
		return 1;
	else
		return 0;
}

static void msm_pcm_routing_build_matrix(int fedai_id, int dspst_id,
	int path_type)
{
	int i, port_type;
	struct route_payload payload;

	payload.num_copps = 0;
	port_type = (path_type == ADM_PATH_PLAYBACK ?
		MSM_AFE_PORT_TYPE_RX : MSM_AFE_PORT_TYPE_TX);

	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		if (!is_be_dai_extproc(i) &&
		   (afe_get_port_type(msm_bedais[i].port_id) == port_type) &&
		   (msm_bedais[i].active) &&
		   (test_bit(fedai_id, &msm_bedais[i].fe_sessions)))
			payload.copp_ids[payload.num_copps++] =
					msm_bedais[i].port_id;
	}

	if (payload.num_copps)
		adm_matrix_map(dspst_id, path_type,
			payload.num_copps, payload.copp_ids, 0);
}

void msm_pcm_routing_reg_psthr_stream(int fedai_id, int dspst_id,
					int stream_type, int enable)
{
	int i, session_type, path_type, port_type;
	u32 mode = 0;

	if (fedai_id > MSM_FRONTEND_DAI_MM_MAX_ID) {
		/* bad ID assigned in machine driver */
		pr_err("%s: bad MM ID\n", __func__);
		return;
	}

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK) {
		session_type = SESSION_TYPE_RX;
		path_type = ADM_PATH_PLAYBACK;
		port_type = MSM_AFE_PORT_TYPE_RX;
	} else {
		session_type = SESSION_TYPE_TX;
		path_type = ADM_PATH_LIVE_REC;
		port_type = MSM_AFE_PORT_TYPE_TX;
	}

	mutex_lock(&routing_lock);

	fe_dai_map[fedai_id][session_type].strm_id = dspst_id;
	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		if (!is_be_dai_extproc(i) &&
		   (afe_get_port_type(msm_bedais[i].port_id) == port_type) &&
		   (msm_bedais[i].active) &&
		   (test_bit(fedai_id, &msm_bedais[i].fe_sessions))) {
			mode = afe_get_port_type(msm_bedais[i].port_id);
			if (enable)
				adm_connect_afe_port(mode, dspst_id,
					    msm_bedais[i].port_id);
			else
				adm_disconnect_afe_port(mode, dspst_id,
						msm_bedais[i].port_id);

			break;
		}
	}
	mutex_unlock(&routing_lock);
}

void msm_pcm_routing_reg_phy_stream(int fedai_id, bool perf_mode, int dspst_id,
							int stream_type)
{
	int i, session_type, path_type, port_type;
	struct route_payload payload;
	u32 channels;

	if (fedai_id > MSM_FRONTEND_DAI_MM_MAX_ID) {
		/* bad ID assigned in machine driver */
		pr_err("%s: bad MM ID %d\n", __func__, fedai_id);
		return;
	}

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK) {
		session_type = SESSION_TYPE_RX;
		path_type = ADM_PATH_PLAYBACK;
		port_type = MSM_AFE_PORT_TYPE_RX;
	} else {
		session_type = SESSION_TYPE_TX;
		path_type = ADM_PATH_LIVE_REC;
		port_type = MSM_AFE_PORT_TYPE_TX;
	}

	mutex_lock(&routing_lock);

	payload.num_copps = 0; /* only RX needs to use payload */
	fe_dai_map[fedai_id][session_type].strm_id = dspst_id;
	/* re-enable EQ if active */
	if (eq_data[fedai_id].enable)
		msm_send_eq_values(fedai_id);
	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		if (test_bit(fedai_id, &msm_bedais[i].fe_sessions))
			msm_bedais[i].perf_mode = perf_mode;
		if (!is_be_dai_extproc(i) &&
		   (afe_get_port_type(msm_bedais[i].port_id) == port_type) &&
		   (msm_bedais[i].active) &&
		   (test_bit(fedai_id, &msm_bedais[i].fe_sessions))) {

			channels = msm_bedais[i].channel;

			if ((stream_type == SNDRV_PCM_STREAM_PLAYBACK) &&
				((channels == 1) || (channels == 2)) &&
				msm_bedais[i].perf_mode) {
				pr_debug("%s configure COPP to lowlatency mode",
								__func__);
				adm_multi_ch_copp_open(msm_bedais[i].port_id,
				path_type,
				msm_bedais[i].sample_rate,
				msm_bedais[i].channel,
				DEFAULT_COPP_TOPOLOGY, msm_bedais[i].perf_mode);
			} else if ((stream_type == SNDRV_PCM_STREAM_PLAYBACK) &&
				(channels > 2))
				adm_multi_ch_copp_open(msm_bedais[i].port_id,
				path_type,
				msm_bedais[i].sample_rate,
				msm_bedais[i].channel,
				DEFAULT_COPP_TOPOLOGY, msm_bedais[i].perf_mode);
			else
				adm_open(msm_bedais[i].port_id,
				path_type,
				msm_bedais[i].sample_rate,
				msm_bedais[i].channel,
				DEFAULT_COPP_TOPOLOGY);

			payload.copp_ids[payload.num_copps++] =
				msm_bedais[i].port_id;
			srs_port_id = msm_bedais[i].port_id;
			srs_send_params(srs_port_id, 1, 0);
		}
	}
	if (payload.num_copps)
		adm_matrix_map(dspst_id, path_type,
			payload.num_copps, payload.copp_ids, 0);

	mutex_unlock(&routing_lock);
}
void msm_pcm_routing_reg_phy_stream_v2(int fedai_id, bool perf_mode,
				       int dspst_id, int stream_type,
				       struct msm_pcm_routing_evt event_info)
{
	msm_pcm_routing_reg_phy_stream(fedai_id, perf_mode, dspst_id,
				       stream_type);

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK)
		fe_dai_map[fedai_id][SESSION_TYPE_RX].event_info = event_info;
	else
		fe_dai_map[fedai_id][SESSION_TYPE_TX].event_info = event_info;

}

void msm_pcm_routing_dereg_phy_stream(int fedai_id, int stream_type)
{
	int i, port_type, session_type;

	if (fedai_id > MSM_FRONTEND_DAI_MM_MAX_ID) {
		/* bad ID assigned in machine driver */
		pr_err("%s: bad MM ID\n", __func__);
		return;
	}

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK) {
		port_type = MSM_AFE_PORT_TYPE_RX;
		session_type = SESSION_TYPE_RX;
	} else {
		port_type = MSM_AFE_PORT_TYPE_TX;
		session_type = SESSION_TYPE_TX;
	}

	mutex_lock(&routing_lock);

	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		if (!is_be_dai_extproc(i) &&
		   (afe_get_port_type(msm_bedais[i].port_id) == port_type) &&
		   (msm_bedais[i].active) &&
		   (test_bit(fedai_id, &msm_bedais[i].fe_sessions)))
			adm_close(msm_bedais[i].port_id);
	}

	fe_dai_map[fedai_id][session_type].strm_id = INVALID_SESSION;
	fe_dai_map[fedai_id][session_type].be_srate = 0;
	mutex_unlock(&routing_lock);
}

/* Check if FE/BE route is set */
static bool msm_pcm_routing_route_is_set(u16 be_id, u16 fe_id)
{
	bool rc = false;

	if (fe_id > MSM_FRONTEND_DAI_MM_MAX_ID) {
		/* recheck FE ID in the mixer control defined in this file */
		pr_err("%s: bad MM ID\n", __func__);
		return rc;
	}

	if (test_bit(fe_id, &msm_bedais[be_id].fe_sessions))
		rc = true;

	return rc;
}

static void msm_pcm_routing_process_audio(u16 reg, u16 val, int set)
{
	int session_type, path_type;
	u32 channels;
	struct msm_pcm_routing_fdai_data *fdai;


	if (val > MSM_FRONTEND_DAI_MM_MAX_ID) {
		/* recheck FE ID in the mixer control defined in this file */
		pr_err("%s: bad MM ID\n", __func__);
		return;
	}

	if (afe_get_port_type(msm_bedais[reg].port_id) ==
		MSM_AFE_PORT_TYPE_RX) {
		session_type = SESSION_TYPE_RX;
		path_type = ADM_PATH_PLAYBACK;
	} else {
		session_type = SESSION_TYPE_TX;
		path_type = ADM_PATH_LIVE_REC;
	}

	mutex_lock(&routing_lock);

	if (set) {

		set_bit(val, &msm_bedais[reg].fe_sessions);
		fdai = &fe_dai_map[val][session_type];
		if (msm_bedais[reg].active && fdai->strm_id !=
			INVALID_SESSION) {
			channels = msm_bedais[reg].channel;

			if (session_type == SESSION_TYPE_TX && fdai->be_srate &&
			    (fdai->be_srate != msm_bedais[reg].sample_rate)) {
				pr_debug("%s: flush strm %d due diff BE rates\n",
					__func__, fdai->strm_id);

				if (fdai->event_info.event_func)
					fdai->event_info.event_func(
						MSM_PCM_RT_EVT_BUF_RECFG,
						fdai->event_info.priv_data);
				fdai->be_srate = 0; /* might not need it */
			}

			if ((session_type == SESSION_TYPE_RX) &&
				((channels == 1) || (channels == 2))
				&& msm_bedais[reg].perf_mode) {
				adm_multi_ch_copp_open(msm_bedais[reg].port_id,
				path_type,
				msm_bedais[reg].sample_rate,
				channels,
				DEFAULT_COPP_TOPOLOGY,
				msm_bedais[reg].perf_mode);
				pr_debug("%s:configure COPP to lowlatency mode",
								 __func__);
			} else if ((session_type == SESSION_TYPE_RX)
					&& (channels > 2))
				adm_multi_ch_copp_open(msm_bedais[reg].port_id,
				path_type,
				msm_bedais[reg].sample_rate,
				channels,
				DEFAULT_COPP_TOPOLOGY,
				msm_bedais[reg].perf_mode);
			else
				adm_open(msm_bedais[reg].port_id,
				path_type,
				msm_bedais[reg].sample_rate, channels,
				DEFAULT_COPP_TOPOLOGY);

			if (session_type == SESSION_TYPE_RX &&
				fdai->event_info.event_func)
					fdai->event_info.event_func(
						MSM_PCM_RT_EVT_DEVSWITCH,
						fdai->event_info.priv_data);

			msm_pcm_routing_build_matrix(val,
				fdai->strm_id, path_type);
			srs_port_id = msm_bedais[reg].port_id;
			srs_send_params(srs_port_id, 1, 0);
		}
	} else {
		clear_bit(val, &msm_bedais[reg].fe_sessions);
		fdai = &fe_dai_map[val][session_type];
		if (msm_bedais[reg].active && fdai->strm_id !=
			INVALID_SESSION) {
			fdai->be_srate = msm_bedais[reg].sample_rate;
			adm_close(msm_bedais[reg].port_id);
			msm_pcm_routing_build_matrix(val,
				fdai->strm_id, path_type);
		}
	}

	mutex_unlock(&routing_lock);
}

static int msm_routing_get_audio_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	(struct soc_mixer_control *)kcontrol->private_value;

	if (test_bit(mc->shift, &msm_bedais[mc->reg].fe_sessions))
		ucontrol->value.integer.value[0] = 1;
	else
		ucontrol->value.integer.value[0] = 0;

	pr_debug("%s: reg %x shift %x val %ld\n", __func__, mc->reg, mc->shift,
	ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_routing_put_audio_mixer(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int mask = (1 << fls(mc->max)) - 1;
	unsigned short val = (ucontrol->value.integer.value[0] & mask);
	struct snd_soc_dapm_update update;
	struct snd_soc_card *card;

	card = dapm->card;
	update.kcontrol = kcontrol;
	update.reg = mc->reg;
	update.mask = mask;
	update.val = val;


	if (ucontrol->value.integer.value[0] &&
	   msm_pcm_routing_route_is_set(mc->reg, mc->shift) == false) {
		msm_pcm_routing_process_audio(mc->reg, mc->shift, 1);
		snd_soc_dapm_mixer_update_power(dapm, kcontrol, 1, &update);
	} else if (!ucontrol->value.integer.value[0] &&
		  msm_pcm_routing_route_is_set(mc->reg, mc->shift) == true) {
		msm_pcm_routing_process_audio(mc->reg, mc->shift, 0);
		snd_soc_dapm_mixer_update_power(dapm, kcontrol, 0, &update);
	}

	return 0;
}

static void msm_send_eq_values(int eq_idx)
{
	int result;
	struct audio_client *ac = q6asm_get_audio_client(
				fe_dai_map[eq_idx][SESSION_TYPE_RX].strm_id);

	if (ac == NULL) {
		pr_err("%s: Could not get audio client for session: %d\n",
		      __func__, fe_dai_map[eq_idx][SESSION_TYPE_RX].strm_id);
		goto done;
	}

	result = q6asm_equalizer(ac, &eq_data[eq_idx]);

	if (result < 0)
		pr_err("%s: Call to ASM equalizer failed, returned = %d\n",
		      __func__, result);
done:
	return;
}

static const struct snd_kcontrol_new hdmi_mixer_controls[] = {
	SOC_SINGLE_EXT("MultiMedia1", MSM_BACKEND_DAI_HDMI_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia2", MSM_BACKEND_DAI_HDMI_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia3", MSM_BACKEND_DAI_HDMI_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia4", MSM_BACKEND_DAI_HDMI_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia5", MSM_BACKEND_DAI_HDMI_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA5, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia6", MSM_BACKEND_DAI_HDMI_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA6, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia7", MSM_BACKEND_DAI_HDMI_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA7, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia8", MSM_BACKEND_DAI_HDMI_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA8, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_soc_dapm_widget msm_qdsp6_widgets[] = {
	/* Frontend AIF */
	/* Widget name equals to Front-End DAI name<Need confirmation>,
	* Stream name must contains substring of front-end dai name
	*/
	SND_SOC_DAPM_AIF_IN("MM_DL1", "MultiMedia1 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL2", "MultiMedia2 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL3", "MultiMedia3 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL4", "MultiMedia4 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL5", "MultiMedia5 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL6", "MultiMedia6 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL7", "MultiMedia7 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL8", "MultiMedia8 Playback", 0, 0, 0, 0),
	/* Backend AIF */
	/* Stream name equals to backend dai link stream name
	*/
	SND_SOC_DAPM_AIF_OUT("HDMI", "HDMI Playback", 0, 0, 0 , 0),
	/* incall */
	/* Mixer definitions */
	SND_SOC_DAPM_MIXER("HDMI Mixer", SND_SOC_NOPM, 0, 0,
	hdmi_mixer_controls, ARRAY_SIZE(hdmi_mixer_controls)),
	SND_SOC_DAPM_OUTPUT("BE_OUT"),
};

static const struct snd_soc_dapm_route intercon[] = {
	{"HDMI Mixer", "MultiMedia1", "MM_DL1"},
	{"HDMI Mixer", "MultiMedia2", "MM_DL2"},
	{"HDMI Mixer", "MultiMedia3", "MM_DL3"},
	{"HDMI Mixer", "MultiMedia4", "MM_DL4"},
	{"HDMI Mixer", "MultiMedia5", "MM_DL5"},
	{"HDMI Mixer", "MultiMedia6", "MM_DL6"},
	{"HDMI Mixer", "MultiMedia7", "MM_DL7"},
	{"HDMI Mixer", "MultiMedia8", "MM_DL8"},
	{"HDMI", NULL, "HDMI Mixer"},
	{"BE_OUT", NULL, "HDMI Playback"},
	{"HDMI Playback", NULL, "MultiMedia1 Playback"},
};

int msm_pcm_routing_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int be_id = rtd->dai_link->be_id;

	if (be_id >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: unexpected be_id %d\n", __func__, be_id);
		return -EINVAL;
	}

	mutex_lock(&routing_lock);
	msm_bedais[be_id].sample_rate = params_rate(params);
	msm_bedais[be_id].channel = params_channels(params);
	mutex_unlock(&routing_lock);
	return 0;
}

static int msm_pcm_routing_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int be_id = rtd->dai_link->be_id;
	int i, session_type;
	struct msm_pcm_routing_bdai_data *bedai;

	if (be_id >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: unexpected be_id %d\n", __func__, be_id);
		return -EINVAL;
	}

	bedai = &msm_bedais[be_id];
	session_type = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		0 : 1);

	mutex_lock(&routing_lock);

	for_each_set_bit(i, &bedai->fe_sessions, MSM_FRONTEND_DAI_MM_SIZE) {
		if (fe_dai_map[i][session_type].strm_id != INVALID_SESSION) {
			fe_dai_map[i][session_type].be_srate =
				bedai->sample_rate;
			adm_close(bedai->port_id);
			srs_port_id = -1;
		}
	}

	bedai->active = 0;
	bedai->sample_rate = 0;
	bedai->channel = 0;
	bedai->perf_mode = false;
	mutex_unlock(&routing_lock);

	return 0;
}

int msm_pcm_routing_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int be_id = rtd->dai_link->be_id;
	int i, path_type, session_type;
	struct msm_pcm_routing_bdai_data *bedai;
	u32 channels;
	bool playback, capture;
	struct msm_pcm_routing_fdai_data *fdai;

	if (be_id >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: unexpected be_id %d\n", __func__, be_id);
		return -EINVAL;
	}

	bedai = &msm_bedais[be_id];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		path_type = ADM_PATH_PLAYBACK;
		session_type = SESSION_TYPE_RX;
	} else {
		path_type = ADM_PATH_LIVE_REC;
		session_type = SESSION_TYPE_TX;
	}

	mutex_lock(&routing_lock);

	if (bedai->active == 1)
		goto done; /* Ignore prepare if back-end already active */

	/* AFE port is not active at this point. However, still
	* go ahead setting active flag under the notion that
	* QDSP6 is able to handle ADM starting before AFE port
	* is started.
	*/
	bedai->active = 1;
	playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	capture  = substream->stream == SNDRV_PCM_STREAM_CAPTURE;

	for_each_set_bit(i, &bedai->fe_sessions, MSM_FRONTEND_DAI_MM_SIZE) {
		fdai = &fe_dai_map[i][session_type];
		if (fdai->strm_id != INVALID_SESSION) {
			if (session_type == SESSION_TYPE_TX && fdai->be_srate &&
			    (fdai->be_srate != bedai->sample_rate)) {
				pr_debug("%s: flush strm %d due diff BE rates\n",
					__func__,
					fdai->strm_id);

				if (fdai->event_info.event_func)
					fdai->event_info.event_func(
						MSM_PCM_RT_EVT_BUF_RECFG,
						fdai->event_info.priv_data);
				fdai->be_srate = 0; /* might not need it */
			}

			channels = bedai->channel;
			if ((playback || capture)
				&& ((channels == 2) || (channels == 1)) &&
				bedai->perf_mode) {
				adm_multi_ch_copp_open(bedai->port_id,
				path_type,
				bedai->sample_rate,
				channels,
				DEFAULT_COPP_TOPOLOGY, bedai->perf_mode);
				pr_debug("%s:configure COPP to lowlatency mode",
								__func__);
			} else if ((playback || capture)
				&& (channels > 2))
				adm_multi_ch_copp_open(bedai->port_id,
				path_type,
				bedai->sample_rate,
				channels,
				DEFAULT_COPP_TOPOLOGY, bedai->perf_mode);
			else
				adm_open(bedai->port_id,
				path_type,
				bedai->sample_rate,
				channels,
				DEFAULT_COPP_TOPOLOGY);

			msm_pcm_routing_build_matrix(i,
				fdai->strm_id, path_type);
			srs_port_id = bedai->port_id;
			srs_send_params(srs_port_id, 1, 0);
		}
	}

done:
	mutex_unlock(&routing_lock);

	return 0;
}

static struct snd_pcm_ops msm_routing_pcm_ops = {
	.hw_params	= msm_pcm_routing_hw_params,
	.close          = msm_pcm_routing_close,
	.prepare        = msm_pcm_routing_prepare,
};

/* Not used but frame seems to require it */
static int msm_routing_probe(struct snd_soc_platform *platform)
{
	return 0;
}

static int msm_asoc_routing_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static struct snd_soc_platform_driver msm_soc_routing_platform = {
	.ops		= &msm_routing_pcm_ops,
	.probe		= msm_routing_probe,
	.pcm_new	= msm_asoc_routing_pcm_new,
	.component_driver =  {
		.name = "msm-qdsp6-routing-dai",
		.dapm_widgets = msm_qdsp6_widgets,
		.num_dapm_widgets = ARRAY_SIZE(msm_qdsp6_widgets),
		.dapm_routes = intercon,
		.num_dapm_routes = ARRAY_SIZE(intercon),
	},
};

static  int msm_routing_pcm_probe(struct platform_device *pdev)
{
	int ret;
	dev_dbg(&pdev->dev, "dev name %s\n", dev_name(&pdev->dev));
	ret = snd_soc_register_platform(&pdev->dev,
				  &msm_soc_routing_platform);
	return ret;
}

static int msm_routing_pcm_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_routing_dt_match[] = {
	{.compatible = "qcom,msm-pcm-routing"},
	{}
};

static struct platform_driver msm_routing_pcm_driver = {
	.driver = {
		.name = "msm-pcm-routing",
		.owner = THIS_MODULE,
		.of_match_table = msm_routing_dt_match,
	},
	.probe = msm_routing_pcm_probe,
	.remove = (msm_routing_pcm_remove),
};

int msm_routing_check_backend_enabled(int fedai_id)
{
	int i;

	if (fedai_id >= MSM_FRONTEND_DAI_MM_MAX_ID) {
		/* bad ID assigned in machine driver */
		pr_err("%s: bad MM ID\n", __func__);
		return 0;
	}
	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		if (test_bit(fedai_id, &msm_bedais[i].fe_sessions))
			return msm_bedais[i].active;
	}
	return 0;
}
module_platform_driver(msm_routing_pcm_driver);
MODULE_DESCRIPTION("MSM routing platform driver");
MODULE_LICENSE("GPL v2");
