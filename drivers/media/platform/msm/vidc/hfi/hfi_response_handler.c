/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/hash.h>

#include "msm_vidc_debug.h"
#include "msm_vidc_common.h"

#include "hfi/vidc_hfi_helper.h"
#include "hfi/vidc_hfi.h"

static enum vidc_error to_vidc_error(u32 hfi_err)
{
	switch (hfi_err) {
	case HFI_ERR_NONE:
	case HFI_ERR_SESSION_SAME_STATE_OPERATION:
		return VIDC_ERR_NONE;
	case HFI_ERR_SYS_FATAL:
		return VIDC_ERR_HW_FATAL;
	case HFI_ERR_SYS_VERSION_MISMATCH:
	case HFI_ERR_SYS_INVALID_PARAMETER:
	case HFI_ERR_SYS_SESSION_ID_OUT_OF_RANGE:
	case HFI_ERR_SESSION_INVALID_PARAMETER:
	case HFI_ERR_SESSION_INVALID_SESSION_ID:
	case HFI_ERR_SESSION_INVALID_STREAM_ID:
		return VIDC_ERR_BAD_PARAM;
	case HFI_ERR_SYS_INSUFFICIENT_RESOURCES:
	case HFI_ERR_SYS_UNSUPPORTED_DOMAIN:
	case HFI_ERR_SYS_UNSUPPORTED_CODEC:
	case HFI_ERR_SESSION_UNSUPPORTED_PROPERTY:
	case HFI_ERR_SESSION_UNSUPPORTED_SETTING:
	case HFI_ERR_SESSION_INSUFFICIENT_RESOURCES:
	case HFI_ERR_SESSION_UNSUPPORTED_STREAM:
		return VIDC_ERR_NOT_SUPPORTED;
	case HFI_ERR_SYS_MAX_SESSIONS_REACHED:
		return VIDC_ERR_MAX_CLIENTS;
	case HFI_ERR_SYS_SESSION_IN_USE:
		return VIDC_ERR_CLIENT_PRESENT;
	case HFI_ERR_SESSION_FATAL:
		return VIDC_ERR_CLIENT_FATAL;
	case HFI_ERR_SESSION_BAD_POINTER:
		return VIDC_ERR_BAD_PARAM;
	case HFI_ERR_SESSION_INCORRECT_STATE_OPERATION:
		return VIDC_ERR_BAD_STATE;
	case HFI_ERR_SESSION_STREAM_CORRUPT:
	case HFI_ERR_SESSION_STREAM_CORRUPT_OUTPUT_STALLED:
		return VIDC_ERR_BITSTREAM_ERR;
	case HFI_ERR_SESSION_SYNC_FRAME_NOT_DETECTED:
		return VIDC_ERR_IFRAME_EXPECTED;
	case HFI_ERR_SESSION_START_CODE_NOT_FOUND:
		return VIDC_ERR_START_CODE_NOT_FOUND;
	case HFI_ERR_SESSION_EMPTY_BUFFER_DONE_OUTPUT_PENDING:
	default:
		return VIDC_ERR_FAIL;
	}

	return VIDC_ERR_FAIL;
}

static struct hal_session *
to_hal_session(struct list_head *sessions, u32 session_id)
{
	struct hal_session *session = NULL;
	bool found = false;

	list_for_each_entry(session, sessions, list) {
		if (hash32_ptr(session) == session_id) {
			found = true;
			break;
		}
	}

	return found ? session : NULL;
}

static void event_seq_changed(u32 device_id, struct hal_session *session,
			      struct hfi_msg_event_notify_pkt *pkt)
{
	struct vidc_inst *inst = session->session_id;
	struct vidc_cb_event event = {0};
	int num_properties_changed;
	struct hfi_frame_size *frame_sz;
	struct hfi_profile_level *profile_level;
	u8 *data_ptr;
	u32 prop_id;

	switch (pkt->event_data1) {
	case HFI_EVENT_DATA_SEQUENCE_CHANGED_SUFFICIENT_BUFFER_RESOURCES:
		event.hal_event_type =
			HAL_EVENT_SEQ_CHANGED_SUFFICIENT_RESOURCES;
		break;
	case HFI_EVENT_DATA_SEQUENCE_CHANGED_INSUFFICIENT_BUFFER_RESOURCES:
		event.hal_event_type =
			HAL_EVENT_SEQ_CHANGED_INSUFFICIENT_RESOURCES;
		break;
	default:
		break;
	}

	num_properties_changed = pkt->event_data2;
	if (!num_properties_changed)
		goto done;

	data_ptr = (u8 *) &pkt->rg_ext_event_data[0];
	do {
		prop_id = *((u32 *)data_ptr);
		switch (prop_id) {
		case HFI_PROPERTY_PARAM_FRAME_SIZE:
			data_ptr += sizeof(u32);
			frame_sz = (struct hfi_frame_size *) data_ptr;
			event.width = frame_sz->width;
			event.height = frame_sz->height;
			data_ptr += sizeof(frame_sz);
			break;
		case HFI_PROPERTY_PARAM_PROFILE_LEVEL_CURRENT:
			data_ptr += sizeof(u32);
			profile_level = (struct hfi_profile_level *) data_ptr;
			event.profile = profile_level->profile;
			event.level = profile_level->level;
			data_ptr += sizeof(profile_level);
			break;
		default:
			dprintk(VIDC_DBG, "%s cmd: %#x not supported\n",
				__func__, prop_id);
			break;
		}
		num_properties_changed--;
	} while (num_properties_changed > 0);

done:
	if (!inst->event_notify)
		return;

	inst->error = VIDC_ERR_NONE;
	inst->event_notify(inst, SYS_EVENT_CHANGE, &event);
}

static void event_release_buffer_ref(u32 device_id, struct hal_session *session,
				     struct hfi_msg_event_notify_pkt *pkt)
{
	struct vidc_inst *inst = session->session_id;
	struct vidc_cb_event event = {0};
	struct hfi_msg_event_release_buffer_ref_pkt *data;

	data = (struct hfi_msg_event_release_buffer_ref_pkt *)
		pkt->rg_ext_event_data;

	event.hal_event_type = HAL_EVENT_RELEASE_BUFFER_REFERENCE;
	event.packet_buffer = data->packet_buffer;
	event.extra_data_buffer = data->extra_data_buffer;

	if (!inst->event_notify)
		return;

	inst->error = VIDC_ERR_NONE;
	inst->event_notify(inst, SYS_EVENT_CHANGE, &event);
}

static void event_sys_error(u32 device_id, u32 event)
{
	struct vidc_core *core;

	core = vidc_get_core(device_id);
	if (IS_ERR(core)) {
		dprintk(VIDC_ERR, "Got SYS_ERR but unable to identify core\n");
		return;
	}

	if (!core->event_notify)
		return;

	core->event_notify(core, device_id, event);
}

static void event_session_error(u32 device_id, struct hal_session *session,
				struct hfi_msg_event_notify_pkt *pkt)
{
	struct vidc_inst *inst = session->session_id;

	dprintk(VIDC_INFO, "received: SESSION_ERROR with event id:%d\n",
		pkt->event_data1);

	if (!inst->event_notify)
		return;

	switch (pkt->event_data1) {
	/* non fatal session errors */
	case HFI_ERR_SESSION_INVALID_SCALE_FACTOR:
	case HFI_ERR_SESSION_UNSUPPORT_BUFFERTYPE:
	case HFI_ERR_SESSION_UNSUPPORTED_SETTING:
	case HFI_ERR_SESSION_UPSCALE_NOT_SUPPORTED:
		inst->error = VIDC_ERR_NONE;
		break;
	default:
		inst->error = to_vidc_error(pkt->event_data1);
		inst->event_notify(inst, SESSION_ERROR, NULL);
		break;
	}
}

static void hfi_event_notify(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct hfi_msg_event_notify_pkt *pkt = packet;

	switch (pkt->event_id) {
	case HFI_EVENT_SYS_ERROR:
		dprintk(VIDC_ERR, "HFI_EVENT_SYS_ERROR: %d, %#x\n",
			pkt->event_data1, pkt->event_data2);
		event_sys_error(device_id, SYS_ERROR);
		break;
	}

	if (!session) {
		dprintk(VIDC_ERR, "%s: got invalid session id\n", __func__);
		return;
	}

	switch (pkt->event_id) {
	case HFI_EVENT_SESSION_ERROR:
		event_session_error(device_id, session, pkt);
		break;
	case HFI_EVENT_SESSION_SEQUENCE_CHANGED:
		event_seq_changed(device_id, session, pkt);
		break;
	case HFI_EVENT_RELEASE_BUFFER_REFERENCE:
		event_release_buffer_ref(device_id, session, pkt);
		break;
	case HFI_EVENT_SESSION_PROPERTY_CHANGED:
		break;
	default:
		break;
	}
}

static void hfi_sys_init_done(u32 device_id, void *sess, void *packet)
{
	struct hfi_msg_sys_init_done_pkt *pkt = packet;
	struct vidc_hal_sys_init_done sys_init_done = {0};
	u32 rem_bytes, read_bytes = 0, num_properties;
	struct vidc_core *core;
	enum vidc_error error;
	u8 *data_ptr;
	u32 prop_id;

	error = to_vidc_error(pkt->error_type);
	if (error != VIDC_ERR_NONE)
		goto err_no_prop;

	if (!pkt->num_properties) {
		dprintk(VIDC_ERR, "%s: no properties\n", __func__);
		error = VIDC_ERR_FAIL;
		goto err_no_prop;
	}

	rem_bytes = pkt->size;
	rem_bytes -= sizeof(struct hfi_msg_sys_init_done_pkt);
	rem_bytes += sizeof(u32);

	if (!rem_bytes) {
		dprintk(VIDC_ERR, "%s: missing property data\n", __func__);
		error = VIDC_ERR_FAIL;
		goto err_no_prop;
	}

	data_ptr = (u8 *) &pkt->rg_property_data[0];
	num_properties = pkt->num_properties;

	while (num_properties && rem_bytes >= sizeof(u32)) {
		prop_id = *((u32 *)data_ptr);
		data_ptr += sizeof(u32);

		switch (prop_id) {
		case HFI_PROPERTY_PARAM_CODEC_SUPPORTED: {
			struct hfi_codec_supported *prop;

			prop = (struct hfi_codec_supported *) data_ptr;

			if (rem_bytes < sizeof(*prop)) {
				error = VIDC_ERR_BAD_PARAM;
				break;
			}
			sys_init_done.dec_codec_supported =
				prop->decoder_codec_supported;
			sys_init_done.enc_codec_supported =
				prop->encoder_codec_supported;
			break;
		}
		default:
			dprintk(VIDC_ERR, "%s: bad property id: %x\n", __func__,
				prop_id);
			error = VIDC_ERR_BAD_PARAM;
			break;
		}

		if (!error) {
			rem_bytes -= read_bytes;
			data_ptr += read_bytes;
			num_properties--;
		}
	}

err_no_prop:
	core = vidc_get_core(device_id);
	if (IS_ERR(core)) {
		dprintk(VIDC_ERR, "wrong device id received\n");
		return;
	}

	core->enc_codecs = sys_init_done.enc_codec_supported;
	core->dec_codecs = sys_init_done.dec_codec_supported;

	if (core->id == VIDC_CORE_VENUS &&
	   (core->dec_codecs & HAL_VIDEO_CODEC_H264))
		core->dec_codecs |= HAL_VIDEO_CODEC_MVC;

	dprintk(VIDC_DBG, "supported_codecs: enc = %#x, dec = %#x\n",
		core->enc_codecs, core->dec_codecs);

	core->error = error;
	complete(&core->done);
}

static void
sys_get_prop_image_version(struct hfi_msg_sys_property_info_packet *pkt)
{
	int i = 0;
	char version[256];
	const u32 version_string_size = 128;
	u8 *str_image_version;
	int req_bytes;

	req_bytes = pkt->size - sizeof(*pkt);

	if (req_bytes < version_string_size || !pkt->rg_property_data[1] ||
	    pkt->num_properties > 1) {
		dprintk(VIDC_ERR, "%s: bad packet: %d\n", __func__, req_bytes);
		return;
	}

	str_image_version = (u8 *)&pkt->rg_property_data[1];

	/*
	 * The version string returned by firmware includes null
	 * characters at the start and in between. Replace the null
	 * characters with space, to print the version info.
	 */
	for (i = 0; i < version_string_size; i++) {
		if (str_image_version[i] != '\0')
			version[i] = str_image_version[i];
		else
			version[i] = ' ';
	}

	version[i] = '\0';

	dprintk(VIDC_DBG, "F/W version: %s\n", version);
}

static void hfi_sys_property_info(u32 device_id, void *sess, void *packet)
{
	struct hfi_msg_sys_property_info_packet *pkt = packet;

	if (!pkt->num_properties) {
		dprintk(VIDC_ERR, "%s: no properties\n", __func__);
		return;
	}

	switch (pkt->rg_property_data[0]) {
	case HFI_PROPERTY_SYS_IMAGE_VERSION:
		sys_get_prop_image_version(pkt);
		break;
	default:
		dprintk(VIDC_ERR, "%s: unknown_property data: %x\n", __func__,
			pkt->rg_property_data[0]);
		break;
	}
}

static void hfi_sys_rel_resource_done(u32 device_id, void *sess, void *packet)
{
	struct hfi_msg_sys_release_resource_done_pkt *pkt = packet;
	struct vidc_core *core;

	core = vidc_get_core(device_id);
	if (IS_ERR(core)) {
		dprintk(VIDC_ERR, "wrong core id received\n");
		return;
	}

	core->error = to_vidc_error(pkt->error_type);
	complete(&core->done);
}

static void hfi_copy_cap_prop(struct hfi_capability_supported *in,
			      struct vidc_hal_session_init_done *sess_init_done)
{
	struct hal_capability_supported *out;

	if (!in || !sess_init_done) {
		dprintk(VIDC_ERR, "%s: invalid input parameter\n", __func__);
		return;
	}

	switch (in->capability_type) {
	case HFI_CAPABILITY_FRAME_WIDTH:
		out = &sess_init_done->width;
		break;
	case HFI_CAPABILITY_FRAME_HEIGHT:
		out = &sess_init_done->height;
		break;
	case HFI_CAPABILITY_MBS_PER_FRAME:
		out = &sess_init_done->mbs_per_frame;
		break;
	case HFI_CAPABILITY_MBS_PER_SECOND:
		out = &sess_init_done->mbs_per_sec;
		break;
	case HFI_CAPABILITY_FRAMERATE:
		out = &sess_init_done->frame_rate;
		break;
	case HFI_CAPABILITY_SCALE_X:
		out = &sess_init_done->scale_x;
		break;
	case HFI_CAPABILITY_SCALE_Y:
		out = &sess_init_done->scale_y;
		break;
	case HFI_CAPABILITY_BITRATE:
		out = &sess_init_done->bitrate;
		break;
	case HFI_CAPABILITY_HIER_P_NUM_ENH_LAYERS:
		out = &sess_init_done->hier_p;
		break;
	case HFI_CAPABILITY_ENC_LTR_COUNT:
		out = &sess_init_done->ltr_count;
		break;
	case HFI_CAPABILITY_CP_OUTPUT2_THRESH:
		out = &sess_init_done->secure_output2_threshold;
		break;
	default:
		out = NULL;
		break;
	}

	if (out) {
		out->min = in->min;
		out->max = in->max;
		out->step_size = in->step_size;
	}
}

static void
session_get_prop_profile_level(struct hfi_msg_session_property_info_packet *pkt,
			       struct hfi_profile_level *profile_level)
{
	struct hfi_profile_level *hfi_profile_level;
	u32 req_bytes;

	req_bytes = pkt->size - sizeof(*pkt);

	if (!req_bytes || req_bytes % sizeof(struct hfi_profile_level)) {
		dprintk(VIDC_ERR, "%s: bad packet\n", __func__);
		return;
	}

	hfi_profile_level =
		(struct hfi_profile_level *) &pkt->rg_property_data[1];
	profile_level->profile = hfi_profile_level->profile;
	profile_level->level = hfi_profile_level->level;

	dprintk(VIDC_DBG, "%s profile: %d level: %d\n",
		__func__, profile_level->profile, profile_level->level);
}

static void
session_get_prop_buf_req(struct hfi_msg_session_property_info_packet *pkt,
			 struct buffer_requirements *buffreq)
{
	struct hfi_buffer_requirements *buf_req;
	u32 req_bytes;

	req_bytes = pkt->size - sizeof(*pkt);

	if (!req_bytes || req_bytes % sizeof(*buf_req) ||
	    !pkt->rg_property_data[1]) {
		dprintk(VIDC_ERR, "%s: bad packet\n", __func__);
		return;
	}

	buf_req = (struct hfi_buffer_requirements *) &pkt->rg_property_data[1];

	if (!buf_req) {
		dprintk(VIDC_ERR, "%s: invalid req buffer\n", __func__);
		return;
	}

	while (req_bytes) {
		if (buf_req->size && buf_req->count_min > buf_req->count_actual)
			dprintk(VIDC_WARN, "%s: bad req buffer\n", __func__);

		dprintk(VIDC_DBG, "got buffer requirements for: %x\n",
			buf_req->type);

		switch (buf_req->type) {
		case HFI_BUFFER_INPUT:
			memcpy(&buffreq->buffer[0], buf_req, sizeof(*buf_req));
			buffreq->buffer[0].type = HAL_BUFFER_INPUT;
			break;
		case HFI_BUFFER_OUTPUT:
			memcpy(&buffreq->buffer[1], buf_req, sizeof(*buf_req));
			buffreq->buffer[1].type = HAL_BUFFER_OUTPUT;
			break;
		case HFI_BUFFER_OUTPUT2:
			memcpy(&buffreq->buffer[2], buf_req, sizeof(*buf_req));
			buffreq->buffer[2].type = HAL_BUFFER_OUTPUT2;
			break;
		case HFI_BUFFER_EXTRADATA_INPUT:
			memcpy(&buffreq->buffer[3], buf_req, sizeof(*buf_req));
			buffreq->buffer[3].type = HAL_BUFFER_EXTRADATA_INPUT;
			break;
		case HFI_BUFFER_EXTRADATA_OUTPUT:
			memcpy(&buffreq->buffer[4], buf_req, sizeof(*buf_req));
			buffreq->buffer[4].type = HAL_BUFFER_EXTRADATA_OUTPUT;
			break;
		case HFI_BUFFER_EXTRADATA_OUTPUT2:
			memcpy(&buffreq->buffer[5], buf_req, sizeof(*buf_req));
			buffreq->buffer[5].type = HAL_BUFFER_EXTRADATA_OUTPUT2;
			break;
		case HFI_BUFFER_INTERNAL_SCRATCH:
			memcpy(&buffreq->buffer[6], buf_req, sizeof(*buf_req));
			buffreq->buffer[6].type = HAL_BUFFER_INTERNAL_SCRATCH;
			break;
		case HFI_BUFFER_INTERNAL_SCRATCH_1:
			memcpy(&buffreq->buffer[7], buf_req, sizeof(*buf_req));
			buffreq->buffer[7].type = HAL_BUFFER_INTERNAL_SCRATCH_1;
			break;
		case HFI_BUFFER_INTERNAL_SCRATCH_2:
			memcpy(&buffreq->buffer[8], buf_req, sizeof(*buf_req));
			buffreq->buffer[8].type = HAL_BUFFER_INTERNAL_SCRATCH_2;
			break;
		case HFI_BUFFER_INTERNAL_PERSIST:
			memcpy(&buffreq->buffer[9], buf_req, sizeof(*buf_req));
			buffreq->buffer[9].type = HAL_BUFFER_INTERNAL_PERSIST;
			break;
		case HFI_BUFFER_INTERNAL_PERSIST_1:
			memcpy(&buffreq->buffer[10], buf_req, sizeof(*buf_req));
			buffreq->buffer[10].type =
				HAL_BUFFER_INTERNAL_PERSIST_1;
			break;
		default:
			dprintk(VIDC_ERR, "%s: bad buffer type: %d\n", __func__,
				buf_req->type);
			break;
		}

		req_bytes -= sizeof(struct hfi_buffer_requirements);
		buf_req++;
	}
}

static void hfi_session_prop_info(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_msg_session_property_info_packet *pkt = packet;
	struct hfi_profile_level profile_level = {0};
	struct buffer_requirements bufreq;
	struct getprop_buf *getprop;
	void *data;
	size_t len;

	inst->error = VIDC_ERR_NONE;

	if (!pkt->num_properties) {
		dprintk(VIDC_ERR, "%s: no properties\n", __func__);
		return;
	}

	switch (pkt->rg_property_data[0]) {
	case HFI_PROPERTY_CONFIG_BUFFER_REQUIREMENTS:
		memset(&bufreq, 0, sizeof(bufreq));
		session_get_prop_buf_req(pkt, &bufreq);
		data = &bufreq;
		len = sizeof(bufreq);
		break;
	case HFI_PROPERTY_PARAM_PROFILE_LEVEL_CURRENT:
		session_get_prop_profile_level(pkt, &profile_level);
		data = &profile_level;
		len = sizeof(profile_level);
		break;
	default:
		dprintk(VIDC_DBG, "%s: unknown property id:%x\n", __func__,
			pkt->rg_property_data[0]);
		return;
	}

	getprop = kzalloc(sizeof(*getprop), GFP_KERNEL);
	if (!getprop) {
		dprintk(VIDC_ERR, "%s: getprop kzalloc failed\n", __func__);
		return;
	}

	getprop->data = kmemdup(data, len, GFP_KERNEL);
	if (!getprop->data) {
		dprintk(VIDC_ERR, "%s: kmemdup failed\n", __func__);
		kfree(getprop);
		return;
	}

	mutex_lock(&inst->pending_getpropq.lock);
	list_add_tail(&getprop->list, &inst->pending_getpropq.list);
	mutex_unlock(&inst->pending_getpropq.lock);

	complete(&inst->done);
}

static enum vidc_error
session_init_done_prop_read(struct hfi_msg_session_init_done_pkt *pkt,
			    struct vidc_hal_session_init_done *sess_init_done)
{
	u32 rem_bytes, num_properties;
	u32 prop_id, next_offset = 0;
	enum vidc_error error;
	u32 prop_count = 0;
	u8 *data_ptr;

	rem_bytes = pkt->size - sizeof(*pkt) + sizeof(u32);

	if (!rem_bytes) {
		dprintk(VIDC_ERR, "%s: missing property info\n", __func__);
		return VIDC_ERR_FAIL;
	}

	error = to_vidc_error(pkt->error_type);
	if (error)
		return error;

	data_ptr = (u8 *) &pkt->rg_property_data[0];
	num_properties = pkt->num_properties;

	while (error == VIDC_ERR_NONE && num_properties &&
	       rem_bytes >= sizeof(u32)) {
		prop_id = *((u32 *)data_ptr);
		next_offset = sizeof(u32);

		switch (prop_id) {
		case HFI_PROPERTY_PARAM_CAPABILITY_SUPPORTED: {
			struct hfi_capability_supported_info *prop =
				(struct hfi_capability_supported_info *)
				(data_ptr + next_offset);
			u32 num_caps;
			struct hfi_capability_supported *cap_ptr;

			if ((rem_bytes - next_offset) < sizeof(*cap_ptr)) {
				error = VIDC_ERR_BAD_PARAM;
				break;
			}

			num_caps = prop->num_capabilities;
			cap_ptr = &prop->rg_data[0];
			next_offset += sizeof(u32);

			while (num_caps &&
			      (rem_bytes - next_offset) >= sizeof(u32)) {
				hfi_copy_cap_prop(cap_ptr, sess_init_done);
				cap_ptr++;
				next_offset += sizeof(*cap_ptr);
				num_caps--;
			}
			num_properties--;
			break;
		}
		case HFI_PROPERTY_PARAM_UNCOMPRESSED_FORMAT_SUPPORTED: {
			struct hfi_uncompressed_format_supported *prop =
				(struct hfi_uncompressed_format_supported *)
				(data_ptr + next_offset);
			u32 num_format_entries;
			char *fmt_ptr;
			struct hfi_uncompressed_plane_info *plane_info;

			if ((rem_bytes - next_offset) < sizeof(*prop)) {
				error = VIDC_ERR_BAD_PARAM;
				break;
			}

			num_format_entries = prop->format_entries;
			next_offset = sizeof(*prop) - sizeof(u32);
			fmt_ptr = (char *)&prop->rg_format_info[0];

			while (num_format_entries) {
				u32 bytes_to_skip;
				plane_info =
				(struct hfi_uncompressed_plane_info *) fmt_ptr;

				if ((rem_bytes - next_offset) <
				     sizeof(*plane_info)) {
					error = VIDC_ERR_BAD_PARAM;
					break;
				}

				bytes_to_skip = sizeof(*plane_info) -
					sizeof(struct
					hfi_uncompressed_plane_constraints) +
					plane_info->num_planes *
					sizeof(struct
					hfi_uncompressed_plane_constraints);

				fmt_ptr +=  bytes_to_skip;
				next_offset += bytes_to_skip;
				num_format_entries--;
			}
			num_properties--;
			break;
		}
		case HFI_PROPERTY_PARAM_PROPERTIES_SUPPORTED: {
			struct hfi_properties_supported *prop =
				(struct hfi_properties_supported *)
				(data_ptr + next_offset);

			next_offset += sizeof(*prop) - sizeof(u32)
					+ prop->num_properties * sizeof(u32);
			num_properties--;
			break;
		}
		case HFI_PROPERTY_PARAM_PROFILE_LEVEL_SUPPORTED: {
			char *ptr = NULL;
			int count = 0;
			struct hfi_profile_level *prop_level;
			struct hfi_profile_level_supported *prop =
				(struct hfi_profile_level_supported *)
				(data_ptr + next_offset);

			ptr = (char *) &prop->rg_profile_level[0];
			dprintk(VIDC_DBG, "prop->profile_count: %d\n",
				prop->profile_count);
			prop_count = prop->profile_count;
			if (prop_count > MAX_PROFILE_COUNT) {
				prop_count = MAX_PROFILE_COUNT;
				dprintk(VIDC_WARN,
					"prop count exceeds max profile count\n");
			}
			while (prop_count) {
				ptr++;
				prop_level = (struct hfi_profile_level *) ptr;
				sess_init_done->
				profile_level.profile_level[count].profile
					= prop_level->profile;
				sess_init_done->
				profile_level.profile_level[count].level
					= prop_level->level;
				prop_count--;
				count++;
				ptr +=
				sizeof(struct hfi_profile_level) / sizeof(u32);
			}
			next_offset += sizeof(*prop) -
				sizeof(struct hfi_profile_level) +
				prop->profile_count *
				sizeof(struct hfi_profile_level);
			num_properties--;
			break;
		}
		case HFI_PROPERTY_PARAM_NAL_STREAM_FORMAT_SUPPORTED: {
			next_offset +=
				sizeof(struct hfi_nal_stream_format_supported);
			num_properties--;
			break;
		}
		case HFI_PROPERTY_PARAM_NAL_STREAM_FORMAT_SELECT: {
			next_offset += sizeof(u32);
			num_properties--;
			break;
		}
		case HFI_PROPERTY_PARAM_MAX_SEQUENCE_HEADER_SIZE: {
			next_offset += sizeof(u32);
			num_properties--;
			break;
		}
		case HFI_PROPERTY_PARAM_VENC_INTRA_REFRESH: {
			next_offset += sizeof(struct hfi_intra_refresh);
			num_properties--;
			break;
		}
		case HFI_PROPERTY_PARAM_BUFFER_ALLOC_MODE_SUPPORTED: {
			struct hfi_buffer_alloc_mode_supported *prop =
				(struct hfi_buffer_alloc_mode_supported *)
				(data_ptr + next_offset);
			int i;

			if (prop->buffer_type == HFI_BUFFER_OUTPUT ||
			    prop->buffer_type == HFI_BUFFER_OUTPUT2) {
				sess_init_done->alloc_mode_out = 0;

				for (i = 0; i < prop->num_entries; i++) {
					switch (prop->rg_data[i]) {
					case HFI_BUFFER_MODE_STATIC:
						sess_init_done->alloc_mode_out
						|= HAL_BUFFER_MODE_STATIC;
						break;
					case HFI_BUFFER_MODE_DYNAMIC:
						sess_init_done->alloc_mode_out
						|= HAL_BUFFER_MODE_DYNAMIC;
						break;
					}

					if (i >= 32) {
						dprintk(VIDC_ERR,
						"%s - num_entries: %d from f/w seems suspect\n",
						__func__, prop->num_entries);
						break;
					}
				}
			}
			next_offset += sizeof(*prop) -
				sizeof(u32) + prop->num_entries * sizeof(u32);
			num_properties--;
			break;
		}
		default:
			dprintk(VIDC_DBG, "%s: default case %#x\n", __func__,
				prop_id);
			break;
		}

		rem_bytes -= next_offset;
		data_ptr += next_offset;
	}

	return error;
}

static void hfi_session_init_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_device *hdev = inst->core->hfidev;
	struct hfi_msg_session_init_done_pkt *pkt = packet;
	struct vidc_core_capability *cap = &inst->capability;
	struct vidc_hal_session_init_done init_done;
	enum vidc_error error;

	error = to_vidc_error(pkt->error_type);
	if (error != VIDC_ERR_NONE)
		goto done;

	memset(&init_done, 0, sizeof(init_done));

	error = session_init_done_prop_read(pkt, &init_done);
	if (error != VIDC_ERR_NONE)
		goto done;

	cap->width = init_done.width;
	cap->height = init_done.height;
	cap->frame_rate = init_done.frame_rate;
	cap->scale_x = init_done.scale_x;
	cap->scale_y = init_done.scale_y;
	cap->hier_p = init_done.hier_p;
	cap->ltr_count = init_done.ltr_count;
	cap->pixelprocess_capabilities =
			call_hfi_op(hdev, get_core_capabilities);
	cap->mbs_per_frame = init_done.mbs_per_frame;
	cap->buffer_mode[CAPTURE_PORT] = init_done.alloc_mode_out;
	cap->secure_output2_threshold = init_done.secure_output2_threshold;
	cap->capability_set = true;

done:
	inst->error = error;
	complete(&inst->done);
}

static void hfi_session_load_res_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_msg_session_load_resources_done_packet *pkt = packet;

	inst->error = to_vidc_error(pkt->error_type);
	complete(&inst->done);
}

static void hfi_session_flush_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_msg_session_flush_done_packet *pkt = packet;

	inst->error = to_vidc_error(pkt->error_type);
	complete(&inst->done);
}

static void hfi_session_etb_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_msg_session_empty_buffer_done_packet *pkt = packet;
	u32 flags = 0;

	inst->error = to_vidc_error(pkt->error_type);

	if (inst->error == VIDC_ERR_NOT_SUPPORTED)
		flags |= V4L2_QCOM_BUF_INPUT_UNSUPPORTED;
	if (inst->error == VIDC_ERR_BITSTREAM_ERR)
		flags |= V4L2_QCOM_BUF_DATA_CORRUPT;
	if (inst->error == VIDC_ERR_START_CODE_NOT_FOUND)
		flags |= V4L2_MSM_VIDC_BUF_START_CODE_NOT_FOUND;

	if (!inst->empty_buf_done)
		return;

	inst->empty_buf_done(inst, pkt->input_tag, pkt->filled_len, pkt->offset,
			     flags);
}

static void hfi_session_ftb_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	bool is_decoder = session->is_decoder;
	struct vidc_hal_fbd fbd = {0};
	enum hal_buffer buffer_type;
	struct timeval timestamp;
	int64_t time_usec = 0;
	unsigned int error;
	u32 flags = 0;

	if (!is_decoder) {
		struct hfi_msg_session_fbd_compressed_packet *pkt = packet;

		fbd.timestamp_hi = pkt->time_stamp_hi;
		fbd.timestamp_lo = pkt->time_stamp_lo;
		fbd.flags1 = pkt->flags;
		fbd.offset1 = pkt->offset;
		fbd.alloc_len1 = pkt->alloc_len;
		fbd.filled_len1 = pkt->filled_len;
		fbd.picture_type = pkt->picture_type;
		fbd.packet_buffer1 = pkt->packet_buffer;
		fbd.extra_data_buffer = pkt->extra_data_buffer;
		fbd.buffer_type = HAL_BUFFER_OUTPUT;

		error = to_vidc_error(pkt->error_type);
	} else {
		struct hfi_msg_session_fbd_uncompressed_plane0_packet *pkt;

		pkt = packet;

		fbd.timestamp_hi = pkt->time_stamp_hi;
		fbd.timestamp_lo = pkt->time_stamp_lo;
		fbd.flags1 = pkt->flags;
		fbd.offset1 = pkt->offset;
		fbd.alloc_len1 = pkt->alloc_len;
		fbd.filled_len1 = pkt->filled_len;
		fbd.picture_type = pkt->picture_type;
		fbd.packet_buffer1 = pkt->packet_buffer;
		fbd.extra_data_buffer = pkt->extra_data_buffer;

		if (pkt->stream_id == 0)
			fbd.buffer_type = HAL_BUFFER_OUTPUT;
		else if (pkt->stream_id == 1)
			fbd.buffer_type = HAL_BUFFER_OUTPUT2;

		error = to_vidc_error(pkt->error_type);
	}

	buffer_type = vidc_comm_get_hal_output_buffer(inst);

	if (fbd.buffer_type != buffer_type)
		return;

	if (fbd.flags1 & HAL_BUFFERFLAG_READONLY)
		flags |= V4L2_QCOM_BUF_FLAG_READONLY;
	if (fbd.flags1 & HAL_BUFFERFLAG_EOS)
		flags |= V4L2_QCOM_BUF_FLAG_EOS;
	if (fbd.flags1 & HAL_BUFFERFLAG_CODECCONFIG)
		flags &= ~V4L2_QCOM_BUF_FLAG_CODECCONFIG;
	if (fbd.flags1 & HAL_BUFFERFLAG_SYNCFRAME)
		flags |= V4L2_QCOM_BUF_FLAG_IDRFRAME;
	if (fbd.flags1 & HAL_BUFFERFLAG_EOSEQ)
		flags |= V4L2_QCOM_BUF_FLAG_EOSEQ;
	if (fbd.flags1 & HAL_BUFFERFLAG_DECODEONLY)
		flags |= V4L2_QCOM_BUF_FLAG_DECODEONLY;
	if (fbd.flags1 & HAL_BUFFERFLAG_DATACORRUPT)
		flags |= V4L2_QCOM_BUF_DATA_CORRUPT;
	if (fbd.flags1 & HAL_BUFFERFLAG_DROP_FRAME)
		flags |= V4L2_QCOM_BUF_DROP_FRAME;
	if (fbd.flags1 & HAL_BUFFERFLAG_MBAFF)
		flags |= V4L2_MSM_BUF_FLAG_MBAFF;
	if (fbd.flags1 & HAL_BUFFERFLAG_TS_DISCONTINUITY)
		flags |= V4L2_QCOM_BUF_TS_DISCONTINUITY;
	if (fbd.flags1 & HAL_BUFFERFLAG_TS_ERROR)
		flags |= V4L2_QCOM_BUF_TS_ERROR;

	switch (fbd.picture_type) {
	case HAL_PICTURE_IDR:
		flags |= V4L2_QCOM_BUF_FLAG_IDRFRAME;
		flags |= V4L2_BUF_FLAG_KEYFRAME;
		break;
	case HAL_PICTURE_I:
		flags |= V4L2_BUF_FLAG_KEYFRAME;
		break;
	case HAL_PICTURE_P:
		flags |= V4L2_BUF_FLAG_PFRAME;
		break;
	case HAL_PICTURE_B:
		flags |= V4L2_BUF_FLAG_BFRAME;
		break;
	case HAL_FRAME_NOTCODED:
	case HAL_UNUSED_PICT:
	case HAL_FRAME_YUV:
		break;
	default:
		break;
	}

	if (!(fbd.flags1 & HAL_BUFFERFLAG_TIMESTAMPINVALID) &&
	      fbd.filled_len1) {
		time_usec = fbd.timestamp_hi;
		time_usec = (time_usec << 32) | fbd.timestamp_lo;
	}

	timestamp = ns_to_timeval(time_usec * NSEC_PER_USEC);

	if (!inst->fill_buf_done)
		return;

	inst->error = error;
	inst->fill_buf_done(inst, fbd.packet_buffer1, fbd.filled_len1,
			    fbd.offset1, flags, &timestamp);
}

static void hfi_session_start_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_msg_session_start_done_packet *pkt = packet;

	inst->error = to_vidc_error(pkt->error_type);
	complete(&inst->done);
}

static void hfi_session_stop_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_msg_session_stop_done_packet *pkt = packet;

	inst->error = to_vidc_error(pkt->error_type);
	complete(&inst->done);
}

static void hfi_session_rel_res_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_msg_session_release_resources_done_packet *pkt = packet;

	inst->error = to_vidc_error(pkt->error_type);
	complete(&inst->done);
}

static void hfi_session_rel_buf_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_msg_session_release_buffers_done_packet *pkt = packet;

	/*
	 * the address of the released buffer can be extracted:
	 * if (pkt->rg_buffer_info) {
	 *	cmd.data = &pkt->rg_buffer_info;
	 *	cmd.size = sizeof(struct hfi_buffer_info);
	 * }
	 */
	inst->error = to_vidc_error(pkt->error_type);
	complete(&inst->done);
}

static void hfi_session_end_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_msg_session_end_done_pkt *pkt = packet;

	inst->error = to_vidc_error(pkt->error_type);
	complete(&inst->done);
}

static void hfi_session_abort_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_msg_sys_session_abort_done_packet *pkt = packet;

	inst->error = to_vidc_error(pkt->error_type);
	complete(&inst->done);
}

static void
hfi_session_get_seq_hdr_done(u32 device_id, void *sess, void *packet)
{
	struct hal_session *session = sess;
	struct vidc_inst *inst = session->session_id;
	struct hfi_msg_session_get_sequence_hdr_done_pkt *pkt = packet;

	/*
	 * output_done.packet_buffer1 = pkt->sequence_header;
	 * output_done.filled_len1 = pkt->header_len;
	 */

	inst->error = to_vidc_error(pkt->error_type);
	complete(&inst->done);
}

typedef void (*done_handler)(u32, void *, void *);

struct hfi_done_handler {
	const char *name;
	u32 packet;
	u32 packet_sz;
	u32 packet_sz2;
	done_handler done;
	bool is_sys_pkt;
};

static const struct hfi_done_handler handlers[] = {
	{.name = "event_notify",
	 .packet = HFI_MSG_EVENT_NOTIFY,
	 .packet_sz = sizeof(struct hfi_msg_event_notify_pkt),
	 .done = hfi_event_notify,
	},
	{.name = "sys_init_done",
	 .packet = HFI_MSG_SYS_INIT_DONE,
	 .packet_sz = sizeof(struct hfi_msg_sys_init_done_pkt),
	 .done = hfi_sys_init_done,
	 .is_sys_pkt = true,
	},
	{.name = "sys_property_info",
	 .packet = HFI_MSG_SYS_PROPERTY_INFO,
	 .packet_sz = sizeof(struct hfi_msg_sys_property_info_packet),
	 .done = hfi_sys_property_info,
	 .is_sys_pkt = true,
	},
	{.name = "sys_release_resource",
	 .packet = HFI_MSG_SYS_RELEASE_RESOURCE,
	 .packet_sz = sizeof(struct hfi_msg_sys_release_resource_done_pkt),
	 .done = hfi_sys_rel_resource_done,
	 .is_sys_pkt = true,
	},
	{.name = "session_init_done",
	 .packet = HFI_MSG_SYS_SESSION_INIT_DONE,
	 .packet_sz = sizeof(struct hfi_msg_session_init_done_pkt),
	 .done = hfi_session_init_done,
	},
	{.name = "session_end_done",
	 .packet = HFI_MSG_SYS_SESSION_END_DONE,
	 .packet_sz = sizeof(struct hfi_msg_session_end_done_pkt),
	 .done = hfi_session_end_done,
	},
	{.name = "session_load_resources_done",
	 .packet = HFI_MSG_SESSION_LOAD_RESOURCES_DONE,
	 .packet_sz = sizeof(struct hfi_msg_session_load_resources_done_packet),
	 .done = hfi_session_load_res_done,
	},
	{.name = "session_start_done",
	 .packet = HFI_MSG_SESSION_START_DONE,
	 .packet_sz = sizeof(struct hfi_msg_session_start_done_packet),
	 .done = hfi_session_start_done,
	},
	{.name = "session_stop_done",
	 .packet = HFI_MSG_SESSION_STOP_DONE,
	 .packet_sz = sizeof(struct hfi_msg_session_stop_done_packet),
	 .done = hfi_session_stop_done,
	},
	{.name = "session_abort_done",
	 .packet = HFI_MSG_SYS_SESSION_ABORT_DONE,
	 .packet_sz = sizeof(struct hfi_msg_sys_session_abort_done_packet),
	 .done = hfi_session_abort_done,
	},
	{.name = "session_empty_buffer_done",
	 .packet = HFI_MSG_SESSION_EMPTY_BUFFER_DONE,
	 .packet_sz = sizeof(struct hfi_msg_session_empty_buffer_done_packet),
	 .done = hfi_session_etb_done,
	},
	{.name = "session_fill_buffer_done",
	 .packet = HFI_MSG_SESSION_FILL_BUFFER_DONE,
	 .packet_sz =
	 	sizeof(struct hfi_msg_session_fbd_uncompressed_plane0_packet),
	 .packet_sz2 = sizeof(struct hfi_msg_session_fbd_compressed_packet),
	 .done = hfi_session_ftb_done,
	},
	{.name = "session_flush_done",
	 .packet = HFI_MSG_SESSION_FLUSH_DONE,
	 .packet_sz = sizeof(struct hfi_msg_session_flush_done_packet),
	 .done = hfi_session_flush_done,
	},
	{.name = "session_propery_info",
	 .packet = HFI_MSG_SESSION_PROPERTY_INFO,
	 .packet_sz = sizeof(struct hfi_msg_session_property_info_packet),
	 .done = hfi_session_prop_info,
	},
	{.name = "session_release_resources_done",
	 .packet = HFI_MSG_SESSION_RELEASE_RESOURCES_DONE,
	 .packet_sz =
	 	sizeof(struct hfi_msg_session_release_resources_done_packet),
	 .done = hfi_session_rel_res_done,
	},
	{.name = "session_get_sequence_header_done",
	 .packet = HFI_MSG_SESSION_GET_SEQUENCE_HEADER_DONE,
	 .packet_sz = sizeof(struct hfi_msg_session_get_sequence_hdr_done_pkt),
	 .done = hfi_session_get_seq_hdr_done,
	},
	{.name = "session_release_buffers_done",
	 .packet = HFI_MSG_SESSION_RELEASE_BUFFERS_DONE,
	 .packet_sz =
	 	sizeof(struct hfi_msg_session_release_buffers_done_packet),
	 .done = hfi_session_rel_buf_done,
	},
};

void hfi_process_watchdog_timeout(u32 device_id)
{
	event_sys_error(device_id, SYS_WATCHDOG_TIMEOUT);
}

u32 hfi_process_msg_packet(u32 device_id, struct vidc_hal_msg_pkt_hdr *hdr,
			   struct list_head *sessions,
			   struct mutex *session_lock)
{
	struct hal_session *session;
	const struct hfi_done_handler *handler;
	unsigned int i;
	bool found = false;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		handler = &handlers[i];

		if (handler->packet != hdr->packet)
			continue;

		found = true;
		break;
	}

	if (found == false)
		return hdr->packet;

	if (hdr->size && hdr->size < handler->packet_sz &&
	    hdr->size < handler->packet_sz2) {
		pr_err("%s: bad packet size (%d should be %d, pkt type:%x)\n",
			__func__, hdr->size, handler->packet_sz, hdr->packet);

		return hdr->packet;
	}

	mutex_lock(session_lock);
	if (handler->is_sys_pkt) {
		session = NULL;
	} else {
		struct vidc_hal_session_cmd_pkt *pkt;

		pkt = (struct vidc_hal_session_cmd_pkt *)hdr;
		session = to_hal_session(sessions, pkt->session_id);

		/*
		 * Event of type HFI_EVENT_SYS_ERROR will not have any session
		 * associated with it
		 */
		if (!session && hdr->packet != HFI_MSG_EVENT_NOTIFY) {
			pr_err("%s: got invalid session id:%d\n", __func__,
				pkt->session_id);
			goto invalid_session;
		}
	}

	handler->done(device_id, session, hdr);

invalid_session:
	mutex_unlock(session_lock);

	return hdr->packet;
}
