/*
 * Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <media/videobuf2-dma-sg.h>

#include "helpers.h"
#include "hfi_helper.h"

struct intbuf {
	struct list_head list;
	u32 type;
	size_t size;
	void *va;
	dma_addr_t da;
	struct dma_attrs attrs;
};

static int intbufs_set_buffer(struct vidc_inst *inst, u32 type)
{
	struct vidc_core *core = inst->core;
	struct device *dev = core->dev;
	struct hfi_buffer_requirements bufreq;
	struct hfi_buffer_desc bd;
	struct intbuf *buf;
	unsigned int i;
	int ret;

	ret = vidc_get_bufreq(inst, type, &bufreq);
	if (ret)
		return 0;

	if (!bufreq.size)
		return 0;

	for (i = 0; i < bufreq.count_actual; i++) {
		buf = kzalloc(sizeof(*buf), GFP_KERNEL);
		if (!buf) {
			ret = -ENOMEM;
			goto fail;
		}

		buf->type = bufreq.type;
		buf->size = bufreq.size;
		init_dma_attrs(&buf->attrs);
		dma_set_attr(DMA_ATTR_WRITE_COMBINE, &buf->attrs);
		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &buf->attrs);
		buf->va = dma_alloc_attrs(dev, buf->size, &buf->da, GFP_KERNEL,
					  &buf->attrs);
		if (!buf->va) {
			ret = -ENOMEM;
			goto fail;
		}

		memset(&bd, 0, sizeof(bd));
		bd.buffer_size = buf->size;
		bd.buffer_type = buf->type;
		bd.num_buffers = 1;
		bd.device_addr = buf->da;

		ret = hfi_session_set_buffers(inst, &bd);
		if (ret) {
			dev_err(dev, "set session buffers failed\n");
			goto fail;
		}

		mutex_lock(&inst->internalbufs_lock);
		list_add_tail(&buf->list, &inst->internalbufs);
		mutex_unlock(&inst->internalbufs_lock);
	}

	return 0;

fail:
	kfree(buf);
	return ret;
}

static int intbufs_unset_buffers(struct vidc_inst *inst)
{
	struct hfi_buffer_desc bd = {0};
	struct intbuf *buf, *n;
	int ret = 0;

	mutex_lock(&inst->internalbufs_lock);
	list_for_each_entry_safe(buf, n, &inst->internalbufs, list) {
		bd.buffer_size = buf->size;
		bd.buffer_type = buf->type;
		bd.num_buffers = 1;
		bd.device_addr = buf->da;
		bd.response_required = true;

		ret = hfi_session_unset_buffers(inst, &bd);

		list_del(&buf->list);
		dma_free_attrs(inst->core->dev, buf->size, buf->va, buf->da,
			       &buf->attrs);
		kfree(buf);
	}
	mutex_unlock(&inst->internalbufs_lock);

	return ret;
}

static const unsigned int intbuf_types[] = {
	HFI_BUFFER_INTERNAL_SCRATCH,
	HFI_BUFFER_INTERNAL_SCRATCH_1,
	HFI_BUFFER_INTERNAL_SCRATCH_2,
	HFI_BUFFER_INTERNAL_PERSIST,
	HFI_BUFFER_INTERNAL_PERSIST_1,
};

static int intbufs_alloc(struct vidc_inst *inst)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(intbuf_types); i++) {
		ret = intbufs_set_buffer(inst, intbuf_types[i]);
		if (ret)
			goto error;
	}

	return 0;

error:
	intbufs_unset_buffers(inst);
	return ret;
}

static int intbufs_free(struct vidc_inst *inst)
{
	return intbufs_unset_buffers(inst);
}

static u32 load_per_instance(struct vidc_inst *inst)
{
	u32 w = inst->width;
	u32 h = inst->height;
	u32 mbs;

	if (!inst || !(inst->state >= INST_INIT && inst->state < INST_STOP))
		return 0;

	mbs = (ALIGN(w, 16) / 16) * (ALIGN(h, 16) / 16);

	return mbs * inst->fps;
}

static u32 load_per_type(struct vidc_core *core, u32 session_type)
{
	struct vidc_inst *inst = NULL;
	u32 mbs_per_sec = 0;

	mutex_lock(&core->lock);
	list_for_each_entry(inst, &core->instances, list) {
		if (inst->session_type != session_type)
			continue;

		mbs_per_sec += load_per_instance(inst);
	}
	mutex_unlock(&core->lock);

	return mbs_per_sec;
}

static int load_scale_clocks(struct vidc_core *core)
{
	const struct freq_tbl *table = core->res->freq_tbl;
	unsigned int num_rows = core->res->freq_tbl_size;
	unsigned long freq = table[0].freq;
	struct clk *clk = core->clks[0];
	struct device *dev = core->dev;
	u32 mbs_per_sec;
	unsigned int i;
	int ret;

	mbs_per_sec = load_per_type(core, VIDC_SESSION_TYPE_ENC) +
		      load_per_type(core, VIDC_SESSION_TYPE_DEC);

	if (mbs_per_sec > core->res->max_load) {
		dev_warn(dev, "HW is overloaded, needed: %d max: %d\n",
			 mbs_per_sec, core->res->max_load);
		return -EBUSY;
	}

	if (!mbs_per_sec && num_rows > 1) {
		freq = table[num_rows - 1].freq;
		goto set_freq;
	}

	for (i = 0; i < num_rows; i++) {
		if (mbs_per_sec > table[i].load)
			break;
		freq = table[i].freq;
	}

set_freq:

	ret = clk_set_rate(clk, freq);
	if (ret) {
		dev_err(dev, "failed to set clock rate %lu (%d)\n", freq, ret);
		return ret;
	}

	return 0;
}

static int session_set_buf(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *q = vb->vb2_queue;
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct vidc_core *core = inst->core;
	struct device *dev = core->dev;
	struct vidc_buffer *buf = to_vidc_buffer(vbuf);
	struct hfi_frame_data fdata;
	int ret;

	memset(&fdata, 0, sizeof(fdata));

	fdata.alloc_len = vb2_plane_size(vb, 0);
	fdata.device_addr = buf->dma_addr;
	fdata.timestamp = timeval_to_ns(&vbuf->timestamp) / NSEC_PER_USEC;
	fdata.flags = 0;
	fdata.clnt_data = buf->dma_addr;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		fdata.buffer_type = HFI_BUFFER_INPUT;
		fdata.filled_len = vb2_get_plane_payload(vb, 0);
		fdata.offset = vb->planes[0].data_offset;

		if (vbuf->flags & V4L2_BUF_FLAG_LAST || !fdata.filled_len)
			fdata.flags |= HFI_BUFFERFLAG_EOS;

		if (inst->codec_cfg == false &&
		    inst->session_type == VIDC_SESSION_TYPE_DEC) {
			inst->codec_cfg = true;
			fdata.flags |= HFI_BUFFERFLAG_CODECCONFIG;
		}

		ret = hfi_session_etb(inst, &fdata);
	} else if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fdata.buffer_type = HFI_BUFFER_OUTPUT;
		fdata.filled_len = 0;
		fdata.offset = 0;

		ret = hfi_session_ftb(inst, &fdata);
	} else {
		ret = -EINVAL;
	}

	if (ret) {
		dev_err(dev, "failed to set session buffer (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int session_unregister_bufs(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	struct device *dev = core->dev;
	struct hfi_buffer_desc *bd;
	struct vidc_buffer *buf, *tmp;
	int ret = 0;

	if (core->res->hfi_version == HFI_VERSION_3XX)
		return 0;

	mutex_lock(&inst->registeredbufs_lock);
	list_for_each_entry_safe(buf, tmp, &inst->registeredbufs, hfi_list) {
		list_del(&buf->hfi_list);
		bd = &buf->bd;
		bd->response_required = 1;
		ret = hfi_session_unset_buffers(inst, bd);
		if (ret) {
			dev_err(dev, "%s: session release buffers failed\n",
				__func__);
			break;
		}
	}
	mutex_unlock(&inst->registeredbufs_lock);

	return ret;
}

static int session_register_bufs(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	struct device *dev = core->dev;
	struct hfi_buffer_desc *bd;
	struct vidc_buffer *buf, *tmp;
	int ret = 0;

	if (core->res->hfi_version == HFI_VERSION_3XX)
		return 0;

	mutex_lock(&inst->registeredbufs_lock);
	list_for_each_entry_safe(buf, tmp, &inst->registeredbufs, hfi_list) {
		bd = &buf->bd;
		ret = hfi_session_set_buffers(inst, bd);
		if (ret) {
			dev_err(dev, "%s: session: set buffer failed\n",
				__func__);
			break;
		}
	}
	mutex_unlock(&inst->registeredbufs_lock);

	return ret;
}

int vidc_get_bufreq(struct vidc_inst *inst, u32 type,
		    struct hfi_buffer_requirements *out)
{
	u32 ptype = HFI_PROPERTY_CONFIG_BUFFER_REQUIREMENTS;
	union hfi_get_property hprop;
	int ret, i;

	if (out)
		memset(out, 0, sizeof(*out));

	ret = hfi_session_get_property(inst, ptype, &hprop);
	if (ret)
		return ret;

	ret = -EINVAL;

	for (i = 0; i < HFI_BUFFER_TYPE_MAX; i++) {
		if (hprop.bufreq[i].type != type)
			continue;

		if (out)
			memcpy(out, &hprop.bufreq[i], sizeof(*out));
		ret = 0;
		break;
	}

	return ret;
}

int vidc_set_color_format(struct vidc_inst *inst, u32 type, u32 pixfmt)
{
	struct hfi_uncompressed_format_select fmt;
	u32 ptype = HFI_PROPERTY_PARAM_UNCOMPRESSED_FORMAT_SELECT;
	int ret;

	fmt.buffer_type = type;

	switch (pixfmt) {
	case V4L2_PIX_FMT_NV12:
		fmt.format = HFI_COLOR_FORMAT_NV12;
		break;
	case V4L2_PIX_FMT_NV21:
		fmt.format = HFI_COLOR_FORMAT_NV21;
		break;
	default:
		return -EINVAL;
	}

	ret = hfi_session_set_property(inst, ptype, &fmt);
	if (ret)
		return ret;

	return 0;
}

struct vb2_v4l2_buffer *
vidc_vb2_find_buf(struct vidc_inst *inst, dma_addr_t addr)
{
	struct vidc_buffer *buf;
	struct vb2_v4l2_buffer *vb = NULL;

	mutex_lock(&inst->bufqueue_lock);

	list_for_each_entry(buf, &inst->bufqueue, list) {
		if (buf->dma_addr == addr) {
			vb = &buf->vb;
			break;
		}
	}

	if (vb)
		list_del(&buf->list);

	mutex_unlock(&inst->bufqueue_lock);

	return vb;
}

int vidc_vb2_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *q = vb->vb2_queue;
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct vidc_buffer *buf = to_vidc_buffer(vbuf);
	struct hfi_buffer_desc *bd = &buf->bd;
	struct sg_table *sgt;

	memset(bd, 0, sizeof(*bd));

	if (q->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return 0;

	sgt = vb2_dma_sg_plane_desc(vb, 0);
	if (!sgt)
		return -EINVAL;

	bd->buffer_size = vb2_plane_size(vb, 0);
	bd->buffer_type = HFI_BUFFER_OUTPUT;
	bd->num_buffers = 1;
	bd->device_addr = sg_dma_address(sgt->sgl);

	mutex_lock(&inst->registeredbufs_lock);
	list_add_tail(&buf->hfi_list, &inst->registeredbufs);
	mutex_unlock(&inst->registeredbufs_lock);

	return 0;
}

int vidc_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vidc_buffer *buf = to_vidc_buffer(vbuf);
	struct sg_table *sgt;

	sgt = vb2_dma_sg_plane_desc(vb, 0);
	if (!sgt)
		return -EINVAL;

	buf->dma_addr = sg_dma_address(sgt->sgl);

	return 0;
}

void vidc_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vidc_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vidc_buffer *buf = to_vidc_buffer(vbuf);
	unsigned int state;
	int ret;

	mutex_lock(&inst->lock);
	state = inst->state;
	mutex_unlock(&inst->lock);

	if (state == INST_INVALID || state >= INST_STOP) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	mutex_lock(&inst->bufqueue_lock);
	list_add_tail(&buf->list, &inst->bufqueue);
	mutex_unlock(&inst->bufqueue_lock);

	if (!vb2_is_streaming(&inst->bufq_cap) ||
	    !vb2_is_streaming(&inst->bufq_out))
		return;

	ret = session_set_buf(vb);
	if (ret)
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
}

void vidc_vb2_stop_streaming(struct vb2_queue *q)
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct vidc_core *core = inst->core;
	struct device *dev = core->dev;
	struct vb2_queue *other_queue;
	struct vidc_buffer *buf, *n;
	enum vb2_buffer_state state;
	int ret;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		other_queue = &inst->bufq_cap;
	else
		other_queue = &inst->bufq_out;

	if (!vb2_is_streaming(other_queue))
		return;

	ret = hfi_session_stop(inst);
	if (ret) {
		dev_err(dev, "session: stop failed (%d)\n", ret);
		goto abort;
	}

	ret = hfi_session_unload_res(inst);
	if (ret) {
		dev_err(dev, "session: release resources failed (%d)\n", ret);
		goto abort;
	}

	ret = session_unregister_bufs(inst);
	if (ret) {
		dev_err(dev, "failed to release capture buffers: %d\n", ret);
		goto abort;
	}

	ret = intbufs_free(inst);

	if (inst->state == INST_INVALID || core->state == CORE_INVALID)
		ret = -EINVAL;

abort:
	if (ret)
		hfi_session_abort(inst);

	load_scale_clocks(core);

	ret = hfi_session_deinit(inst);

	pm_runtime_put_sync(dev);

	mutex_lock(&inst->bufqueue_lock);

	if (list_empty(&inst->bufqueue)) {
		mutex_unlock(&inst->bufqueue_lock);
		return;
	}

	if (ret)
		state = VB2_BUF_STATE_ERROR;
	else
		state = VB2_BUF_STATE_DONE;

	list_for_each_entry_safe(buf, n, &inst->bufqueue, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->list);
	}

	mutex_unlock(&inst->bufqueue_lock);
}

int vidc_vb2_start_streaming(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	struct vidc_buffer *buf, *n;
	int ret;

	ret = intbufs_alloc(inst);
	if (ret)
		return ret;

	ret = session_register_bufs(inst);
	if (ret)
		goto err_bufs_free;

	load_scale_clocks(core);

	ret = hfi_session_load_res(inst);
	if (ret)
		goto err_unreg_bufs;

	ret = hfi_session_start(inst);
	if (ret)
		goto err_unload_res;

	mutex_lock(&inst->bufqueue_lock);
	list_for_each_entry_safe(buf, n, &inst->bufqueue, list) {
		ret = session_set_buf(&buf->vb.vb2_buf);
		if (ret)
			break;
	}
	mutex_unlock(&inst->bufqueue_lock);

	if (ret)
		goto err_session_stop;

	return 0;

err_session_stop:
	hfi_session_stop(inst);
err_unload_res:
	hfi_session_unload_res(inst);
err_unreg_bufs:
	session_unregister_bufs(inst);
err_bufs_free:
	intbufs_free(inst);

	mutex_lock(&inst->bufqueue_lock);

	if (list_empty(&inst->bufqueue))
		goto err_done;

	list_for_each_entry_safe(buf, n, &inst->bufqueue, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
		list_del(&buf->list);
	}

err_done:
	mutex_unlock(&inst->bufqueue_lock);

	return ret;
}
