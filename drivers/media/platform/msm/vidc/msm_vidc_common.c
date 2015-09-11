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

#define TOUT	msecs_to_jiffies(vidc_hw_rsp_timeout)

enum multi_stream vidc_comm_get_stream_output_mode(struct vidc_inst *inst)
{
	if (inst->session_type == VIDC_DECODER) {
		int rc = 0;
		struct v4l2_control ctrl = {
			.id = V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_MODE
		};
		rc = v4l2_g_ctrl(&inst->ctrl_handler, &ctrl);
		if (!rc && ctrl.value ==
		    V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_SECONDARY)
			return HAL_VIDEO_DECODER_SECONDARY;
	}

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

enum hal_video_codec vidc_comm_hal_codec_type(int fourcc)
{
	dprintk(VIDC_DBG, "codec is %#x\n", fourcc);

	switch (fourcc) {
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
		dprintk(VIDC_ERR, "Wrong codec: %d\n", fourcc);
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

void vidc_change_inst_state(struct vidc_inst *inst, u32 state)
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

static unsigned int get_inst_state(struct vidc_inst *inst)
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

int vidc_comm_suspend(struct vidc_core *core)
{
	struct hfi_device *hdev = core->hfidev;
	int ret;

	ret = call_hfi_op(hdev, suspend, hdev->hfi_device_data);
	if (ret) {
		dprintk(VIDC_WARN, "suspend failed (%d)\n", ret);
		return ret;
	}

	return 0;
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
		dprintk(VIDC_INFO, "core id:%d is already in state: %d\n",
			core->id, core->state);
		return 0;
	}

	init_completion(&core->done);

	ret = call_hfi_op(hdev, core_init, hdev->hfi_device_data);
	if (ret) {
		core->state = CORE_UNINIT;
		mutex_unlock(&core->lock);
		dprintk(VIDC_ERR, "failed to init core id:%d\n", core->id);
		return ret;
	}

	ret = wait_for_completion_timeout(&core->done, TOUT);
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

int hfi_session_init(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int core_state;
	unsigned int inst_state = get_inst_state(inst);
	enum hal_domain domain;
	enum hal_video_codec codec;
	u32 fourcc;
	int ret;

	mutex_lock(&core->lock);
	core_state = core->state;
	mutex_unlock(&core->lock);

	if (inst_state == INST_OPEN_DONE)
		return 0;

	if (inst_state != INST_UNINIT || core_state != CORE_INIT) {
		dprintk(VIDC_ERR, "%s: invalid state (core:%d, inst:%d)\n",
			__func__, core_state, inst_state);
		return -EINVAL;
	}

	if (inst->session_type == VIDC_DECODER)
		fourcc = inst->fmts[OUTPUT_PORT]->fourcc;
	else if (inst->session_type == VIDC_ENCODER)
		fourcc = inst->fmts[CAPTURE_PORT]->fourcc;
	else
		return -EINVAL;

	init_completion(&inst->done);

	domain = vidc_comm_get_hal_domain(inst->session_type);
	codec = vidc_comm_hal_codec_type(fourcc);

	inst->session = call_hfi_op(hdev, session_init, hdev->hfi_device_data,
				    inst, domain, codec);
	if (!inst->session) {
		dprintk(VIDC_ERR, "%s: session: init failed\n", __func__);
		return -EINVAL;
	}

	vidc_change_inst_state(inst, INST_OPEN);

	ret = wait_for_completion_timeout(&inst->done, TOUT);
	if (!ret) {
		dprintk(VIDC_ERR, "%s: wait interrupted or timedout\n",
			__func__);
		return -ETIMEDOUT;
	}

	vidc_change_inst_state(inst, INST_OPEN_DONE);

	return 0;
}

int hfi_session_deinit(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int state = get_inst_state(inst);
	int ret;

	if (state == INST_CLOSE_DONE)
		return 0;

	if (state != INST_RELEASE_RESOURCES_DONE) {
		dprintk(VIDC_ERR, "%s: invalid instance state (%d)\n",
			__func__, inst->state);
		return -EINVAL;
	}

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_end, inst->session);
	if (ret) {
		dprintk(VIDC_ERR, "%s: session: end failed (%d)\n", __func__,
			ret);
		return ret;
	}

	vidc_change_inst_state(inst, INST_CLOSE);

	ret = wait_for_completion_timeout(&inst->done, TOUT);
	if (!ret) {
		dprintk(VIDC_ERR, "%s: wait interrupted or timedout\n",
			__func__);
		return -ETIMEDOUT;
	}

	vidc_change_inst_state(inst, INST_CLOSE_DONE);

	return 0;
}

int hfi_session_start(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int state = get_inst_state(inst);
	int ret;

	if (state == INST_START_DONE)
		return 0;

	if (state != INST_LOAD_RESOURCES_DONE) {
		dprintk(VIDC_ERR, "%s: invalid instance state (%d)\n",
			__func__, inst->state);
		return -EINVAL;
	}

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_start, inst->session);
	if (ret) {
		dprintk(VIDC_ERR, "%s: session: start failed (%d)\n", __func__,
			ret);
		return ret;
	}

	vidc_change_inst_state(inst, INST_START);

	ret = wait_for_completion_timeout(&inst->done, TOUT);
	if (!ret) {
		dprintk(VIDC_ERR, "%s: wait interrupted or timedout\n",
			__func__);
		return -ETIMEDOUT;
	}

	vidc_change_inst_state(inst, INST_START_DONE);

	return 0;
}

int hfi_session_stop(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int state = get_inst_state(inst);
	int ret;

	if (state == INST_STOP_DONE)
		return 0;

	if (state != INST_START_DONE) {
		dprintk(VIDC_ERR, "%s: invalid instance state (%d)\n",
			__func__, inst->state);
		return -EINVAL;
	}

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_stop, inst->session);
	if (ret) {
		dprintk(VIDC_ERR, "%s: session: stop failed (%d)\n",
			__func__, ret);
		return ret;
	}

	vidc_change_inst_state(inst, INST_STOP);

	ret = wait_for_completion_timeout(&inst->done, TOUT);
	if (!ret) {
		dprintk(VIDC_ERR, "%s: wait interrupted or timedout\n",
			__func__);
		return -ETIMEDOUT;
	}

	vidc_change_inst_state(inst, INST_STOP_DONE);

	return 0;
}

int hfi_session_abort(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	int ret;

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_abort, inst->session);
	if (ret) {
		dprintk(VIDC_ERR, "%s: session: abort failed (%d)\n",
			__func__, ret);
		return ret;
	}

	vidc_change_inst_state(inst, INST_STOP);

	ret = wait_for_completion_timeout(&inst->done, TOUT);
	if (!ret) {
		dprintk(VIDC_ERR, "%s: wait interrupted or timedout\n",
			__func__);
		return -ETIMEDOUT;
	}

	vidc_change_inst_state(inst, INST_CLOSE_DONE);

	return 0;
}

int hfi_session_load_res(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	struct hfi_device *hdev = core->hfidev;
	unsigned int state = get_inst_state(inst);
	int ret;

	if (state == INST_LOAD_RESOURCES_DONE)
		return 0;

	if (state != INST_OPEN_DONE) {
		dprintk(VIDC_ERR, "%s: invalid instance state (%d)\n",
			__func__, inst->state);
		return -EINVAL;
	}

	ret = msm_comm_check_overloaded(core);
	if (ret)
		return ret;

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_load_res, inst->session);
	if (ret) {
		dprintk(VIDC_ERR, "%s: session: load resources failed (%d)\n",
			__func__, ret);
		return ret;
	}

	vidc_change_inst_state(inst, INST_LOAD_RESOURCES);

	ret = wait_for_completion_timeout(&inst->done, TOUT);
	if (!ret) {
		dprintk(VIDC_ERR, "%s: wait interrupted or timedout\n",
			__func__);
		return -ETIMEDOUT;
	}

	vidc_change_inst_state(inst, INST_LOAD_RESOURCES_DONE);

	return 0;
}

int hfi_session_release_res(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int state = get_inst_state(inst);
	int ret;

	if (state == INST_RELEASE_RESOURCES_DONE)
		return 0;

	if (state != INST_STOP_DONE) {
		dprintk(VIDC_ERR, "%s: invalid instance state (%d)\n",
			__func__, inst->state);
		return -EINVAL;
	}

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_release_res, inst->session);
	if (ret) {
		dprintk(VIDC_ERR,
			"%s: session: release resources failed (%d)\n",
			__func__, ret);
		return ret;
	}

	vidc_change_inst_state(inst, INST_RELEASE_RESOURCES);

	ret = wait_for_completion_timeout(&inst->done, TOUT);
	if (!ret) {
		dprintk(VIDC_ERR, "%s: wait interrupted or timedout\n",
			__func__);
		return -ETIMEDOUT;
	}

	vidc_change_inst_state(inst, INST_RELEASE_RESOURCES_DONE);

	return 0;
}

int hfi_session_flush(struct vidc_inst *inst)
{
	struct hfi_device *hdev = inst->core->hfidev;
	int ret;

	init_completion(&inst->done);

	ret = call_hfi_op(hdev, session_flush, inst->session, HAL_FLUSH_ALL);
	if (ret) {
		dprintk(VIDC_ERR, "%s: session: flush failed (%d)\n",
			__func__, ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&inst->done, TOUT);
	if (!ret) {
		dprintk(VIDC_ERR, "%s: wait interrupted or timedout\n",
			__func__);
		return -ETIMEDOUT;
	}

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
	if (ret) {
		dprintk(VIDC_ERR, "%s: session: release buffers failed (%d)\n",
			__func__, ret);
		return ret;
	}

	if (!bai->response_required)
		return 0;

	ret = wait_for_completion_timeout(&inst->done, TOUT);
	if (!ret) {
		dprintk(VIDC_ERR, "%s: wait interrupted or timedout\n",
			__func__);
		return -ETIMEDOUT;
	}

	if (inst->error != VIDC_ERR_NONE)
		return -EIO;

	return 0;
}

int hfi_session_get_property(struct vidc_inst *inst, enum hal_property ptype,
			     union hal_get_property *hprop)
{
	struct hfi_device *hdev = inst->core->hfidev;
	unsigned int inst_state = get_inst_state(inst);
	struct getprop_buf *buf;
	int ret = 0;

	if (inst_state == INST_INVALID || inst->core->state == CORE_INVALID) {
		dprintk(VIDC_ERR,
			"%s: invalid states for core and/or instance\n",
			__func__);
		return -EINVAL;
	}

	mutex_lock(&inst->sync_lock);
	if (inst_state < INST_OPEN_DONE || inst_state >= INST_CLOSE) {
		dprintk(VIDC_ERR, "%s: not in proper state\n", __func__);
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
	if (ret) {
		dprintk(VIDC_ERR, "get property (%x) failed (%d)\n",
			ptype, ret);
		goto exit;
	}

	ret = wait_for_completion_timeout(&inst->done, TOUT);
	if (!ret) {
		dprintk(VIDC_ERR, "%s: wait interrupted or timedout\n",
			__func__);
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
		dprintk(VIDC_ERR, "%s getprop list is empty\n", __func__);
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
	unsigned int state = get_inst_state(inst);
	int ret = 0;

	mutex_lock(&inst->sync_lock);
	if (state < INST_OPEN_DONE || state >= INST_CLOSE) {
		dprintk(VIDC_ERR, "state is invalid to set property\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = call_hfi_op(hdev, session_set_property, inst->session, ptype,
			  pdata);
	if (ret)
		dprintk(VIDC_ERR, "set property failed (%x)\n", ptype);

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
		if (!ret && vidc_comm_get_stream_output_mode(inst) ==
		    HAL_VIDEO_DECODER_SECONDARY)
			ret = call_hfi_op(hdev, session_flush, inst->session,
					  HAL_FLUSH_OUTPUT2);
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
		dprintk(VIDC_WARN, "Extradata not found: %d\n", index);
		break;
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

int vidc_check_scaling_supported(struct vidc_inst *inst)
{
	u32 x_min, x_max, y_min, y_max;
	u32 input_height, input_width, output_height, output_width;

	input_height = inst->prop.height_out;
	input_width = inst->prop.width_out;
	output_height = inst->prop.height_cap;
	output_width = inst->prop.width_cap;

	if (!input_height || !input_width || !output_height || !output_width) {
		dprintk(VIDC_ERR, "invalid input or output resolution\n");
		return -EINVAL;
	}

	if (!inst->capability.scale_x.min || !inst->capability.scale_x.max ||
	    !inst->capability.scale_y.min || !inst->capability.scale_y.max) {
		if (input_width * input_height != output_width * output_height) {
			dprintk(VIDC_ERR,
				"scaling is not supported (%dx%d != %dx%d)\n",
				input_width, input_height,
				output_width, output_height);
			return -ENOTSUPP;
		} else {
			dprintk(VIDC_DBG, "supported: %dx%d\n",
				input_width, input_height);
			return 0;
		}
	}

	x_min = (1 << 16) / inst->capability.scale_x.min;
	y_min = (1 << 16) / inst->capability.scale_y.min;
	x_max = inst->capability.scale_x.max >> 16;
	y_max = inst->capability.scale_y.max >> 16;

	if (input_height > output_height) {
		if (input_height / output_height > x_min) {
			dprintk(VIDC_ERR,
				"unsupported height downscale ratio %d vs %d\n",
				input_height/output_height, x_min);
			return -ENOTSUPP;
		}
	} else {
		if (input_height / output_height > x_max) {
			dprintk(VIDC_ERR,
				"unsupported height upscale ratio %d vs %d\n",
				input_height/output_height, x_max);
			return -ENOTSUPP;
		}
	}
	if (input_width > output_width) {
		if (input_width / output_width > y_min) {
			dprintk(VIDC_ERR,
				"unsupported width downscale ratio %d vs %d\n",
				input_width/output_width, y_min);
			return -ENOTSUPP;
		}
	} else {
		if (input_width / output_width > y_max) {
			dprintk(VIDC_ERR,
				"unsupported width upscale ratio %d vs %d\n",
				input_width/output_width, y_max);
			return -ENOTSUPP;
		}
	}

	return 0;
}

int vidc_check_session_supported(struct vidc_inst *inst)
{
	struct vidc_core_capability *capability;
	struct vidc_session_prop *prop;
	struct hfi_device *hdev;
	struct vidc_core *core;
	int ret = 0;

	if (!inst || !inst->core || !inst->core->hfidev)
		return -EINVAL;

	capability = &inst->capability;
	hdev = inst->core->hfidev;
	core = inst->core;
	prop = &inst->prop;

	if (inst->state == INST_OPEN_DONE) {
		ret = msm_comm_check_overloaded(core);
		if (ret) {
			vidc_change_inst_state(inst, INST_INVALID);
			return ret;
		}
	}

	if (capability->capability_set) {
		if (prop->width_cap < capability->width.min ||
		    prop->height_cap < capability->height.min) {
			dprintk(VIDC_ERR, "unsupported:%ux%u, min: %ux%u\n",
				prop->width_cap, prop->height_cap,
				capability->width.min, capability->height.min);
			ret = -ENOTSUPP;
		}

		if (!ret &&
		    prop->width_cap > capability->width.max) {
			dprintk(VIDC_ERR, "unsupported width:%u, max width:%u",
				prop->width_cap, capability->width.max);
			ret = -ENOTSUPP;
		}

		if (!ret && prop->height_cap * prop->width_cap >
		    capability->width.max * capability->height.max) {
			dprintk(VIDC_ERR, "unsupported:%ux%u, max:%ux%u\n",
			prop->width_cap, prop->height_cap,
			capability->width.max, capability->height.max);
			ret = -ENOTSUPP;
		}
	}

	if (ret) {
		vidc_change_inst_state(inst, INST_INVALID);
		dprintk(VIDC_ERR, "resolution unsupported\n");
		return ret;
	}

	return 0;
}

int vidc_comm_set_color_format(struct vidc_inst *inst,
			       enum hal_buffer buffer_type, u32 fourcc)
{
	struct hal_uncompressed_format_select hal_fmt;
	struct hfi_device *hdev = inst->core->hfidev;
	int ret;

	hal_fmt.buffer_type = buffer_type;

	switch (fourcc) {
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
	if ((inst->state >= INST_OPEN_DONE && inst->state < INST_CLOSE_DONE) ||
	     inst->state == INST_INVALID) {
		ret = hfi_session_abort(inst);
		if (ret == -EBUSY) {
			inst->event_notify(inst, SESSION_ERROR, NULL);
			return 0;
		} else if (ret) {
			return ret;
		}
		vidc_change_inst_state(inst, INST_CLOSE_DONE);
	} else {
		inst->event_notify(inst, SESSION_ERROR, NULL);
	}

	return ret;
}
