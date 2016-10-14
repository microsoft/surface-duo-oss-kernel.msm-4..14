/*
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2016 Linaro Ltd.
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
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>

#include "core.h"
#include "hfi.h"
#include "hfi_cmds.h"
#include "hfi_venus.h"

#define TIMEOUT		msecs_to_jiffies(1000)

static u32 to_codec_type(u32 pixfmt)
{
	switch (pixfmt) {
	case V4L2_PIX_FMT_H264:
	case V4L2_PIX_FMT_H264_NO_SC:
		return HFI_VIDEO_CODEC_H264;
	case V4L2_PIX_FMT_H263:
		return HFI_VIDEO_CODEC_H263;
	case V4L2_PIX_FMT_MPEG1:
		return HFI_VIDEO_CODEC_MPEG1;
	case V4L2_PIX_FMT_MPEG2:
		return HFI_VIDEO_CODEC_MPEG2;
	case V4L2_PIX_FMT_MPEG4:
		return HFI_VIDEO_CODEC_MPEG4;
	case V4L2_PIX_FMT_VC1_ANNEX_G:
	case V4L2_PIX_FMT_VC1_ANNEX_L:
		return HFI_VIDEO_CODEC_VC1;
	case V4L2_PIX_FMT_VP8:
		return HFI_VIDEO_CODEC_VP8;
	case V4L2_PIX_FMT_XVID:
		return HFI_VIDEO_CODEC_DIVX;
	default:
		return 0;
	}
}

int hfi_core_init(struct vidc_core *core)
{
	int ret = 0;

	mutex_lock(&core->lock);

	if (core->state >= CORE_INIT)
		goto unlock;

	init_completion(&core->done);

	ret = call_hfi_op(core, core_init, core);
	if (ret)
		goto unlock;

	ret = wait_for_completion_timeout(&core->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	ret = 0;

	if (core->error != HFI_ERR_NONE) {
		ret = -EIO;
		goto unlock;
	}

	core->state = CORE_INIT;
unlock:
	mutex_unlock(&core->lock);
	return ret;
}

int hfi_core_deinit(struct vidc_core *core)
{
	struct device *dev = core->dev;
	int ret = 0;

	mutex_lock(&core->lock);

	if (core->state == CORE_UNINIT)
		goto unlock;

	if (!list_empty(&core->instances)) {
		ret = -EBUSY;
		goto unlock;
	}

	ret = call_hfi_op(core, core_deinit, core);
	if (ret)
		dev_err(dev, "core deinit failed: %d\n", ret);

	core->state = CORE_UNINIT;

unlock:
	mutex_unlock(&core->lock);
	return ret;
}

int hfi_core_suspend(struct vidc_core *core)
{
	return call_hfi_op(core, suspend, core);
}

int hfi_core_resume(struct vidc_core *core)
{
	return call_hfi_op(core, resume, core);
}

int hfi_core_trigger_ssr(struct vidc_core *core, u32 type)
{
	int ret;

	ret = call_hfi_op(core, core_trigger_ssr, core, type);
	if (ret)
		return ret;

	return 0;
}

int hfi_core_ping(struct vidc_core *core)
{
	int ret;

	mutex_lock(&core->lock);

	ret = call_hfi_op(core, core_ping, core, 0xbeef);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&core->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}
	ret = 0;
	if (core->error != HFI_ERR_NONE)
		ret = -ENODEV;
unlock:
	mutex_unlock(&core->lock);
	return ret;
}

int hfi_session_create(struct vidc_inst *inst, const struct hfi_inst_ops *ops)
{
	struct vidc_core *core = inst->core;

	if (!ops)
		return -EINVAL;

	inst->state = INST_UNINIT;
	inst->ops = ops;

	mutex_lock(&core->lock);
	list_add_tail(&inst->list, &core->instances);
	mutex_unlock(&core->lock);

	return 0;
}

int hfi_session_init(struct vidc_inst *inst, u32 pixfmt, u32 session_type)
{
	struct vidc_core *core = inst->core;
	u32 codec;
	int ret;

	codec = to_codec_type(pixfmt);
	inst->session_type = session_type;
	init_completion(&inst->done);

	mutex_lock(&inst->lock);

	ret = call_hfi_op(core, session_init, core, inst, session_type, codec);
	if (ret)
		goto unlock;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	if (inst->error != HFI_ERR_NONE) {
		dev_err(core->dev, "%s: session init failed (%x)\n", __func__,
			inst->error);
		ret = -EIO;
		goto unlock;
	}

	ret = 0;
	inst->state = INST_INIT;

unlock:
	mutex_unlock(&inst->lock);

	return ret;
}

void hfi_session_destroy(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;

	mutex_lock(&core->lock);
	list_del(&inst->list);
	mutex_unlock(&core->lock);

	if (mutex_is_locked(&inst->lock))
		WARN(1, "session destroy");
}

int hfi_session_deinit(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);

	if (inst->state == INST_UNINIT) {
		ret = 0;
		goto unlock;
	}

	if (inst->state < INST_INIT) {
		ret = -EINVAL;
		goto unlock;
	}

	init_completion(&inst->done);

	ret = call_hfi_op(core, session_end, inst);
	if (ret)
		goto unlock;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	if (inst->error != HFI_ERR_NONE) {
		dev_err(core->dev, "session deinit error (%x)\n", inst->error);
		ret = -EIO;
		goto unlock;
	}

	ret = 0;
	inst->state = INST_UNINIT;

unlock:
	mutex_unlock(&inst->lock);

	return ret;
}

int hfi_session_start(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);

	if (inst->state != INST_LOAD_RESOURCES) {
		ret = -EINVAL;
		goto unlock;
	}

	init_completion(&inst->done);

	ret = call_hfi_op(core, session_start, inst);
	if (ret)
		goto unlock;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	ret = 0;

	inst->state = INST_START;
unlock:
	mutex_unlock(&inst->lock);

	return ret;
}

int hfi_session_stop(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);

	if (inst->state != INST_START) {
		ret = -EINVAL;
		goto unlock;
	}

	init_completion(&inst->done);

	ret = call_hfi_op(core, session_stop, inst);
	if (ret)
		goto unlock;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	ret = 0;

	inst->state = INST_STOP;
unlock:
	mutex_unlock(&inst->lock);

	return ret;
}

int hfi_session_continue(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;

	if (core->res->hfi_version != HFI_VERSION_3XX)
		return 0;

	return call_hfi_op(core, session_continue, inst);
}

int hfi_session_abort(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);

	init_completion(&inst->done);

	ret = call_hfi_op(core, session_abort, inst);
	if (ret)
		goto unlock;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	ret = 0;

unlock:
	mutex_unlock(&inst->lock);

	return ret;
}

int hfi_session_load_res(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);

	if (inst->state != INST_INIT) {
		ret = -EINVAL;
		goto unlock;
	}

	init_completion(&inst->done);

	ret = call_hfi_op(core, session_load_res, inst);
	if (ret)
		goto unlock;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	ret = 0;
	inst->state = INST_LOAD_RESOURCES;
unlock:
	mutex_unlock(&inst->lock);

	return ret;
}

int hfi_session_unload_res(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);

	if (inst->state != INST_STOP) {
		ret = -EINVAL;
		goto unlock;
	}

	init_completion(&inst->done);

	ret = call_hfi_op(core, session_release_res, inst);
	if (ret)
		goto unlock;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	ret = 0;
	inst->state = INST_RELEASE_RESOURCES;
unlock:
	mutex_unlock(&inst->lock);

	return ret;
}

int hfi_session_flush(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);
	init_completion(&inst->done);

	ret = call_hfi_op(core, session_flush, inst, HFI_FLUSH_ALL);
	if (ret)
		goto unlock;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	ret = 0;
unlock:
	mutex_unlock(&inst->lock);

	return ret;
}

int hfi_session_set_buffers(struct vidc_inst *inst, struct hfi_buffer_desc *bd)
{
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);
	ret = call_hfi_op(core, session_set_buffers, inst, bd);
	mutex_unlock(&inst->lock);

	return ret;
}

int hfi_session_unset_buffers(struct vidc_inst *inst,
			      struct hfi_buffer_desc *bd)
{
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);

	init_completion(&inst->done);

	ret = call_hfi_op(core, session_unset_buffers, inst, bd);
	if (ret)
		goto unlock;

	if (!bd->response_required) {
		ret = 0;
		goto unlock;
	}

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	ret = 0;

	if (inst->error != HFI_ERR_NONE) {
		dev_dbg(core->dev, "unset buffers error (%x)\n", inst->error);
		ret = -EIO;
	}

unlock:
	mutex_unlock(&inst->lock);

	return ret;
}

int hfi_session_get_property(struct vidc_inst *inst, u32 ptype,
			     union hfi_get_property *hprop)
{
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);

	if (inst->state < INST_INIT || inst->state >= INST_STOP) {
		ret = -EINVAL;
		goto unlock;
	}

	init_completion(&inst->done);

	ret = call_hfi_op(core, session_get_property, inst, ptype);
	if (ret)
		goto unlock;

	ret = wait_for_completion_timeout(&inst->done, TIMEOUT);
	if (!ret) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	if (inst->error != HFI_ERR_NONE) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = 0;
	*hprop = inst->hprop;
unlock:
	mutex_unlock(&inst->lock);

	return ret;
}

int hfi_session_set_property(struct vidc_inst *inst, u32 ptype, void *pdata)
{
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);

	if (inst->state < INST_INIT || inst->state >= INST_STOP) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = call_hfi_op(core, session_set_property, inst, ptype, pdata);
unlock:
	mutex_unlock(&inst->lock);

	if (ret)
		dev_err(core->dev, "set property %x failed (%d)\n", ptype, ret);

	return ret;
}

int hfi_session_etb(struct vidc_inst *inst, struct hfi_frame_data *fdata)
{
	struct vidc_core *core = inst->core;

	return call_hfi_op(core, session_etb, inst, fdata);
}

int hfi_session_ftb(struct vidc_inst *inst, struct hfi_frame_data *fdata)
{
	struct vidc_core *core = inst->core;

	return call_hfi_op(core, session_ftb, inst, fdata);
}

irqreturn_t hfi_isr_thread(struct vidc_core *core)
{
	return call_hfi_op(core, isr_thread, core);
}

irqreturn_t hfi_isr(struct vidc_core *core)
{
	return call_hfi_op(core, isr, core);
}

int hfi_create(struct vidc_core *core)
{
	if (!core->core_ops || !core->dev)
		return -EINVAL;

	core->state = CORE_UNINIT;
	pkt_set_version(core->res->hfi_version);

	return venus_hfi_create(core);
}

void hfi_destroy(struct vidc_core *core)
{
	venus_hfi_destroy(core);
}
