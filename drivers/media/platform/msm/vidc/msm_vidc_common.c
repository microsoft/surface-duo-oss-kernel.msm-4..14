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

#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <media/videobuf2-dma-contig.h>
#include <asm/div64.h>

#include "msm_smem.h"
#include "msm_vdec.h"
#include "msm_vidc_common.h"
#include "hfi/vidc_hfi_api.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_load.h"

#define TIMEOUT		msecs_to_jiffies(vidc_hw_rsp_timeout)

enum multi_stream vidc_comm_get_stream_output_mode(struct vidc_inst *inst)
{
	return HAL_VIDEO_DECODER_PRIMARY;
}

enum hal_domain vidc_comm_get_hal_domain(int session_type)
{
	switch (session_type) {
	case VIDC_ENCODER:
		return HAL_VIDEO_DOMAIN_ENCODER;
	case VIDC_DECODER:
		return HAL_VIDEO_DOMAIN_DECODER;
	default:
		dprintk(VIDC_ERR, "Wrong domain\n");
		return HAL_UNUSED_DOMAIN;
	}
}

enum hal_video_codec vidc_comm_hal_codec_type(u32 pixfmt)
{
	dprintk(VIDC_DBG, "codec is %#x\n", pixfmt);

	switch (pixfmt) {
	case V4L2_PIX_FMT_H264:
	case V4L2_PIX_FMT_H264_NO_SC:
		return HAL_VIDEO_CODEC_H264;
	case V4L2_PIX_FMT_H264_MVC:
		return HAL_VIDEO_CODEC_MVC;
	case V4L2_PIX_FMT_H263:
		return HAL_VIDEO_CODEC_H263;
	case V4L2_PIX_FMT_MPEG1:
		return HAL_VIDEO_CODEC_MPEG1;
	case V4L2_PIX_FMT_MPEG2:
		return HAL_VIDEO_CODEC_MPEG2;
	case V4L2_PIX_FMT_MPEG4:
		return HAL_VIDEO_CODEC_MPEG4;
	case V4L2_PIX_FMT_VC1_ANNEX_G:
	case V4L2_PIX_FMT_VC1_ANNEX_L:
		return HAL_VIDEO_CODEC_VC1;
	case V4L2_PIX_FMT_VP8:
		return HAL_VIDEO_CODEC_VP8;
	default:
		dprintk(VIDC_ERR, "Wrong codec: %d\n", pixfmt);
		return HAL_UNUSED_CODEC;
	}
}

struct vidc_core *vidc_get_core(int core_id)
{
	struct vidc_core *core;
	int found = 0;

	if (core_id > VIDC_CORES_MAX)
		return ERR_PTR(-EINVAL);

	mutex_lock(&vidc_driver->lock);
	list_for_each_entry(core, &vidc_driver->cores, list) {
		if (core->id == core_id) {
			found = 1;
			break;
		}
	}
	mutex_unlock(&vidc_driver->lock);

	if (found)
		return core;

	return ERR_PTR(-ENOENT);
}

void vidc_inst_set_state(struct vidc_inst *inst, u32 state)
{
	mutex_lock(&inst->lock);
	if (inst->state == INST_INVALID) {
		dprintk(VIDC_DBG,
			"Inst: %p is in bad state can't change state\n",
			inst);
		goto exit;
	}

	dprintk(VIDC_DBG, "Moved inst: %p from state: %d to state: %d\n",
		inst, inst->state, state);

	inst->state = state;
exit:
	mutex_unlock(&inst->lock);
}

static unsigned int vidc_inst_get_state(struct vidc_inst *inst)
{
	unsigned int state;

	mutex_lock(&inst->lock);
	state = inst->state;
	mutex_unlock(&inst->lock);

	return state;
}

void vidc_queue_v4l2_event(struct vidc_inst *inst, int event_type)
{
	struct v4l2_event event = {.id = 0, .type = event_type};

	/* TODO: this prevents NULL pointer dereference */
	return;
	v4l2_event_queue_fh(&inst->fh, &event);
}

enum hal_buffer vidc_comm_get_hal_output_buffer(struct vidc_inst *inst)
{
	if (vidc_comm_get_stream_output_mode(inst) ==
	    HAL_VIDEO_DECODER_SECONDARY)
		return HAL_BUFFER_OUTPUT2;
	else
		return HAL_BUFFER_OUTPUT;
}

struct hal_buffer_requirements *
get_buff_req_buffer(struct vidc_inst *inst, enum hal_buffer type)
{
	int i;

	for (i = 0; i < HAL_BUFFER_MAX; i++) {
		if (inst->buff_req.buffer[i].type == type)
			return &inst->buff_req.buffer[i];
	}

	return NULL;
}

/* HFI interface functions start here */

int hfi_core_init(struct vidc_core *core)
{
	struct hfi_device *hdev = core->hfidev;
	int ret;

	mutex_lock(&core->lock);

	if (core->state >= CORE_INIT) {
		mutex_unlock(&core->lock);
		return 0;
	}

	init_completion(&core->done);

	ret = call_hfi_op(hdev, core_init, hdev->hfi_device_data);
	if (ret) {
		mutex_unlock(&core->lock);
		return ret;
	}

	ret = wait_for_completion_timeout(&core->done, TIMEOUT);
	if (!ret) {
		mutex_unlock(&core->lock);
		return -ETIMEDOUT;
	}

	core->state = CORE_INIT;

	mutex_unlock(&core->lock);

	return 0;
}

int hfi_core_deinit(struct vidc_core *core)
{
	mutex_lock(&core->lock);
	if (core->state == CORE_UNINIT) {
		mutex_unlock(&core->lock);
		return 0;
	}
	mutex_unlock(&core->lock);

	msm_comm_scale_clocks(core);

	mutex_lock(&core->lock);
	if (list_empty(&core->instances)) {
		/*
		 * Delay unloading of firmware. This is useful
		 * in avoiding firmware download delays in cases where we
		 * will have a burst of back to back video playback sessions
		 * e.g. thumbnail generation.
		 */
	}
	mutex_unlock(&core->lock);

	return 0;
}

int hfi_core_suspend(struct vidc_core *core)
{
	struct hfi_device *hdev = core->hfidev;

	return call_hfi_op(hdev, suspend, hdev->hfi_device_data);
}

int hfi_session_init(struct vidc_inst *inst, u32 pixfmt)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int state = vidc_inst_get_state(inst);
	enum hal_domain domain;
	enum hal_video_codec codec;
	int ret;

	if (state != INST_UNINIT)
		return -EINVAL;

	init_completion(&inst->done);

	domain = vidc_comm_get_hal_domain(inst->session_type);
	codec = vidc_comm_hal_codec_type(pixfmt);

	inst->session = call_hfi_op(hdev, session_init, hdev->hfi_device_data,
				    inst, domain, codec);
	if (IS_ERR(inst->session))
		return PTR_ERR(inst->session);

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret)
		return -ETIMEDOUT;

	vidc_inst_set_state(inst, INST_OPEN);

	return 0;
}

int hfi_session_deinit(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int state = vidc_inst_get_state(inst);
	int ret;

	if (state == INST_CLOSE)
		return 0;

	if (state < INST_OPEN)
		return -EINVAL;

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_end, inst->session);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret)
		return -ETIMEDOUT;

	vidc_inst_set_state(inst, INST_CLOSE);

	return 0;
}

int hfi_session_start(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int state = vidc_inst_get_state(inst);
	int ret;

	if (state != INST_LOAD_RESOURCES)
		return -EINVAL;

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_start, inst->session);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret)
		return -ETIMEDOUT;

	vidc_inst_set_state(inst, INST_START);

	return 0;
}

int hfi_session_stop(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int state = vidc_inst_get_state(inst);
	int ret;

	if (state != INST_START)
		return -EINVAL;

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_stop, inst->session);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret)
		return -ETIMEDOUT;

	vidc_inst_set_state(inst, INST_STOP);

	return 0;
}

int hfi_session_abort(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	int ret;

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_abort, inst->session);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret)
		return -ETIMEDOUT;

	vidc_inst_set_state(inst, INST_CLOSE);

	return 0;
}

int hfi_session_load_res(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	struct hfi_device *hdev = core->hfidev;
	unsigned int state = vidc_inst_get_state(inst);
	int ret;

	if (state != INST_OPEN)
		return -EINVAL;

	ret = msm_comm_check_overloaded(core);
	if (ret)
		return ret;

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_load_res, inst->session);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret)
		return -ETIMEDOUT;

	vidc_inst_set_state(inst, INST_LOAD_RESOURCES);

	return 0;
}

int hfi_session_release_res(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int state = vidc_inst_get_state(inst);
	int ret;

	if (state != INST_STOP)
		return -EINVAL;

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_release_res, inst->session);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret)
		return -ETIMEDOUT;

	vidc_inst_set_state(inst, INST_RELEASE_RESOURCES);

	return 0;
}

int hfi_session_flush(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	int ret;

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_flush, inst->session, HAL_FLUSH_ALL);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret)
		return -ETIMEDOUT;

	return 0;
}

void hfi_session_clean(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	int ret;

	mutex_lock(&inst->lock);
	if (hdev && inst->session) {
		ret = call_hfi_op(hdev, session_clean, inst->session);
		if (ret)
			dprintk(VIDC_ERR, "session clean failed\n");

		inst->session = NULL;
	}
	mutex_unlock(&inst->lock);
}

int hfi_session_release_buffers(struct vidc_inst *inst,
				struct vidc_buffer_addr_info *bai)
{
	struct hfi_device *hdev = inst->core->hfidev;
	int ret;

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_release_buffers, inst->session, bai);
	if (ret)
		return ret;

	if (!bai->response_required)
		return 0;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret)
		return -ETIMEDOUT;

	if (inst->error != VIDC_ERR_NONE)
		return -EIO;

	return 0;
}

int hfi_session_get_property(struct vidc_inst *inst, enum hal_property ptype,
			     union hal_get_property *hprop)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int state = vidc_inst_get_state(inst);
	struct getprop_buf *buf;
	int ret;

	if (state == INST_INVALID)
		return -EINVAL;

	mutex_lock(&inst->sync_lock);
	if (state < INST_OPEN || state >= INST_CLOSE) {
		ret = -EINVAL;
		goto exit;
	}

	switch (ptype) {
	case HAL_PARAM_PROFILE_LEVEL_CURRENT:
	case HAL_PARAM_GET_BUFFER_REQUIREMENTS:
		break;
	default:
		return -ENOTSUPP;
	}

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_get_property, inst->session, ptype);
	if (ret)
		goto exit;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		inst->state = INST_INVALID;
		vidc_comm_kill_session(inst);
		ret = -ETIMEDOUT;
		goto exit;
	}

	mutex_lock(&inst->pending_getpropq.lock);
	if (!list_empty(&inst->pending_getpropq.list)) {
		buf = list_first_entry(&inst->pending_getpropq.list,
				       struct getprop_buf, list);
		*hprop = *((union hal_get_property *) buf->data);
		kfree(buf->data);
		list_del(&buf->list);
		kfree(buf);
		ret = 0;
	} else {
		/* getprop list is empty */
		ret = -EINVAL;
	}
	mutex_unlock(&inst->pending_getpropq.lock);
exit:
	mutex_unlock(&inst->sync_lock);
	return ret;
}

int hfi_session_set_property(struct vidc_inst *inst, enum hal_property ptype,
			     void *pdata)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int state = vidc_inst_get_state(inst);
	int ret = 0;

	mutex_lock(&inst->sync_lock);
	if (state < INST_OPEN || state >= INST_CLOSE) {
		ret = -EINVAL;
		goto exit;
	}

	ret = call_hfi_op(hdev, session_set_property, inst->session, ptype,
			  pdata);
exit:
	mutex_unlock(&inst->sync_lock);

	return ret;
}

/* End of interface functions */

int vidc_comm_get_bufreqs(struct vidc_inst *inst)
{
	union hal_get_property hprop;
	int ret, i;

	ret = hfi_session_get_property(inst, HAL_PARAM_GET_BUFFER_REQUIREMENTS,
				       &hprop);
	if (ret)
		return ret;

	memcpy(&inst->buff_req, &hprop.buf_req, sizeof(inst->buff_req));

	for (i = 0; i < HAL_BUFFER_MAX; i++)
		dprintk(VIDC_DBG,
			"buftype: %03x, actual count: %02d, size: %d, "
			"count min: %d, hold count: %d, region size: %d\n",
			inst->buff_req.buffer[i].type,
			inst->buff_req.buffer[i].count_actual,
			inst->buff_req.buffer[i].size,
			inst->buff_req.buffer[i].count_min,
			inst->buff_req.buffer[i].hold_count,
			inst->buff_req.buffer[i].region_size);

	dprintk(VIDC_DBG, "\n");

	dprintk(VIDC_PROF, "Input buffers: %d, Output buffers: %d\n",
		inst->buff_req.buffer[0].count_actual,
		inst->buff_req.buffer[1].count_actual);

	return 0;
}

int vidc_comm_bufrequirements(struct vidc_inst *inst, enum hal_buffer type,
			      struct hal_buffer_requirements *out)
{
	enum hal_property ptype = HAL_PARAM_GET_BUFFER_REQUIREMENTS;
	union hal_get_property hprop;
	int ret, i;

	ret = hfi_session_get_property(inst, ptype, &hprop);
	if (ret)
		return ret;

	ret = -EINVAL;

	for (i = 0; i < HAL_BUFFER_MAX; i++) {
		if (hprop.buf_req.buffer[i].type != type)
			continue;

		if (out)
			memcpy(out, &hprop.buf_req.buffer[i], sizeof(*out));
		ret = 0;
		break;
	}

	return ret;
}

int vidc_comm_session_flush(struct vidc_inst *inst, u32 flags)
{
	struct vidc_core *core = inst->core;
	struct hfi_device *hdev = core->hfidev;
	bool ip_flush = false;
	bool op_flush = false;
	int ret =  0;

	ip_flush = flags & V4L2_QCOM_CMD_FLUSH_OUTPUT;
	op_flush = flags & V4L2_QCOM_CMD_FLUSH_CAPTURE;

	if (ip_flush && !op_flush) {
		dprintk(VIDC_INFO, "Input only flush not supported\n");
		return 0;
	}

	if (inst->state == INST_INVALID || core->state == CORE_INVALID) {
		dprintk(VIDC_ERR, "Core %p and inst %p are in bad state\n",
			core, inst);
		return 0;
	}

	if (inst->in_reconfig && !ip_flush && op_flush) {
		ret = call_hfi_op(hdev, session_flush, inst->session,
				  HAL_FLUSH_OUTPUT);
	} else {
		/*
		 * If flush is called after queueing buffers but before
		 * streamon driver should flush the pending queue
		 */

		/* Do not send flush in case of session_error */
		if (!(inst->state == INST_INVALID &&
		      core->state != CORE_INVALID))
			ret = call_hfi_op(hdev, session_flush, inst->session,
					  HAL_FLUSH_ALL);
	}

	return ret;
}

enum hal_extradata_id
vidc_comm_get_hal_extradata_index(enum v4l2_mpeg_vidc_extradata index)
{
	switch (index) {
	case V4L2_MPEG_VIDC_EXTRADATA_NONE:
		return HAL_EXTRADATA_NONE;
	case V4L2_MPEG_VIDC_EXTRADATA_MB_QUANTIZATION:
		return HAL_EXTRADATA_MB_QUANTIZATION;
	case V4L2_MPEG_VIDC_EXTRADATA_INTERLACE_VIDEO:
		return HAL_EXTRADATA_INTERLACE_VIDEO;
	case V4L2_MPEG_VIDC_EXTRADATA_VC1_FRAMEDISP:
		return HAL_EXTRADATA_VC1_FRAMEDISP;
	case V4L2_MPEG_VIDC_EXTRADATA_VC1_SEQDISP:
		return HAL_EXTRADATA_VC1_SEQDISP;
	case V4L2_MPEG_VIDC_EXTRADATA_TIMESTAMP:
		return HAL_EXTRADATA_TIMESTAMP;
	case V4L2_MPEG_VIDC_EXTRADATA_S3D_FRAME_PACKING:
		return HAL_EXTRADATA_S3D_FRAME_PACKING;
	case V4L2_MPEG_VIDC_EXTRADATA_FRAME_RATE:
		return HAL_EXTRADATA_FRAME_RATE;
	case V4L2_MPEG_VIDC_EXTRADATA_PANSCAN_WINDOW:
		return HAL_EXTRADATA_PANSCAN_WINDOW;
	case V4L2_MPEG_VIDC_EXTRADATA_RECOVERY_POINT_SEI:
		return HAL_EXTRADATA_RECOVERY_POINT_SEI;
	case V4L2_MPEG_VIDC_EXTRADATA_MULTISLICE_INFO:
		return HAL_EXTRADATA_MULTISLICE_INFO;
	case V4L2_MPEG_VIDC_EXTRADATA_NUM_CONCEALED_MB:
		return HAL_EXTRADATA_NUM_CONCEALED_MB;
	case V4L2_MPEG_VIDC_EXTRADATA_METADATA_FILLER:
		return HAL_EXTRADATA_METADATA_FILLER;
	case V4L2_MPEG_VIDC_EXTRADATA_ASPECT_RATIO:
		return HAL_EXTRADATA_ASPECT_RATIO;
	case V4L2_MPEG_VIDC_EXTRADATA_INPUT_CROP:
		return HAL_EXTRADATA_INPUT_CROP;
	case V4L2_MPEG_VIDC_EXTRADATA_DIGITAL_ZOOM:
		return HAL_EXTRADATA_DIGITAL_ZOOM;
	case V4L2_MPEG_VIDC_EXTRADATA_MPEG2_SEQDISP:
		return HAL_EXTRADATA_MPEG2_SEQDISP;
	case V4L2_MPEG_VIDC_EXTRADATA_STREAM_USERDATA:
		return HAL_EXTRADATA_STREAM_USERDATA;
	case V4L2_MPEG_VIDC_EXTRADATA_FRAME_QP:
		return HAL_EXTRADATA_FRAME_QP;
	case V4L2_MPEG_VIDC_EXTRADATA_FRAME_BITS_INFO:
		return HAL_EXTRADATA_FRAME_BITS_INFO;
	case V4L2_MPEG_VIDC_EXTRADATA_LTR:
		return HAL_EXTRADATA_LTR_INFO;
	case V4L2_MPEG_VIDC_EXTRADATA_METADATA_MBI:
		return HAL_EXTRADATA_METADATA_MBI;
	default:
		return V4L2_MPEG_VIDC_EXTRADATA_NONE;
	}

	return 0;
}

enum hal_buffer_layout_type
vidc_comm_get_hal_buffer_layout(enum v4l2_mpeg_vidc_video_mvc_layout index)
{
	switch (index) {
	case V4L2_MPEG_VIDC_VIDEO_MVC_SEQUENTIAL:
		return HAL_BUFFER_LAYOUT_SEQ;
	case V4L2_MPEG_VIDC_VIDEO_MVC_TOP_BOTTOM:
		return HAL_BUFFER_LAYOUT_TOP_BOTTOM;
	default:
		break;
	}

	return HAL_UNUSED_BUFFER_LAYOUT;
}

int vidc_trigger_ssr(struct vidc_core *core, enum hal_ssr_trigger_type type)
{
	struct hfi_device *hdev;
	int ret;

	if (!core || !core->hfidev)
		return -EINVAL;

	hdev = core->hfidev;

	if (core->state != CORE_INIT)
		return 0;

	ret = call_hfi_op(hdev, core_trigger_ssr, hdev->hfi_device_data, type);
	if (ret)
		return ret;

	return 0;
}

int vidc_check_session_supported(struct vidc_inst *inst)
{
	struct vidc_core *core =inst->core;
	struct vidc_core_capability *cap = &inst->capability;
	int ret = 0;

	if (inst->state == INST_OPEN) {
		ret = msm_comm_check_overloaded(core);
		if (ret)
			return ret;
	}

	if (!cap->capability_set)
		return 0;

	ret = -ENOTSUPP;

	if (inst->width < cap->width.min || inst->height < cap->height.min)
		goto err;

	if (inst->width > cap->width.max)
		goto err;

	if (inst->height * inst->width > cap->width.max * cap->height.max)
		goto err;

	return 0;
err:
	dprintk(VIDC_ERR, "resolution not supported\n");
	return ret;
}

int vidc_comm_set_color_format(struct vidc_inst *inst,
			       enum hal_buffer buffer_type, u32 pixfmt)
{
	struct hal_uncompressed_format_select hal_fmt;
	struct hfi_device *hdev = inst->core->hfidev;
	int ret;

	hal_fmt.buffer_type = buffer_type;

	switch (pixfmt) {
	case V4L2_PIX_FMT_NV12:
		dprintk(VIDC_DBG, "set color format: nv12\n");
		hal_fmt.format = HAL_COLOR_FORMAT_NV12;
		break;
	case V4L2_PIX_FMT_NV21:
		dprintk(VIDC_DBG, "set color format: nv21\n");
		hal_fmt.format = HAL_COLOR_FORMAT_NV21;
		break;
	default:
		return -ENOTSUPP;
	}

	ret = call_hfi_op(hdev, session_set_property, inst->session,
			  HAL_PARAM_UNCOMPRESSED_FORMAT_SELECT, &hal_fmt);
	if (ret) {
		dprintk(VIDC_ERR, "set uncompressed color format failed (%d)\n",
			ret);
		return ret;
	}

	return 0;
}

int vidc_comm_kill_session(struct vidc_inst *inst)
{
	int ret = 0;

	if (!inst || !inst->core || !inst->core->hfidev) {
		dprintk(VIDC_ERR, "%s: invalid input parameters\n", __func__);
		return -EINVAL;
	} else if (!inst->session) {
		/* There's no hfi session to kill */
		return 0;
	}

	/*
	 * We're internally forcibly killing the session, if fw is aware of
	 * the session send session_abort to firmware to clean up and release
	 * the session, else just kill the session inside the driver.
	 */
	if ((inst->state >= INST_OPEN && inst->state < INST_CLOSE) ||
	     inst->state == INST_INVALID) {
		ret = hfi_session_abort(inst);
		if (ret == -EBUSY) {
			inst->event_notify(inst, SESSION_ERROR, NULL);
			return 0;
		} else if (ret) {
			return ret;
		}
		vidc_inst_set_state(inst, INST_CLOSE);
	} else {
		inst->event_notify(inst, SESSION_ERROR, NULL);
	}

	return ret;
}
