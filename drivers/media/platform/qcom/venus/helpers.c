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
#include <media/v4l2-mem2mem.h>

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

static int intbufs_set_buffer(struct venus_inst *inst, u32 type)
{
	struct venus_core *core = inst->core;
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

//		mutex_lock(&inst->lock);
		list_add_tail(&buf->list, &inst->internalbufs);
//		mutex_unlock(&inst->lock);
	}

	return 0;

fail:
	kfree(buf);
	return ret;
}

static int intbufs_unset_buffers(struct venus_inst *inst)
{
	struct hfi_buffer_desc bd = {0};
	struct intbuf *buf, *n;
	int ret = 0;

//	mutex_lock(&inst->lock);
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
//	mutex_unlock(&inst->lock);

	return ret;
}

static const unsigned int intbuf_types[] = {
	HFI_BUFFER_INTERNAL_SCRATCH,
	HFI_BUFFER_INTERNAL_SCRATCH_1,
	HFI_BUFFER_INTERNAL_SCRATCH_2,
	HFI_BUFFER_INTERNAL_PERSIST,
	HFI_BUFFER_INTERNAL_PERSIST_1,
};

static int intbufs_alloc(struct venus_inst *inst)
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

static int intbufs_free(struct venus_inst *inst)
{
	return intbufs_unset_buffers(inst);
}

static u32 load_per_instance(struct venus_inst *inst)
{
	u32 w = inst->width;
	u32 h = inst->height;
	u32 mbs;

	if (!inst || !(inst->state >= INST_INIT && inst->state < INST_STOP))
		return 0;

	mbs = (ALIGN(w, 16) / 16) * (ALIGN(h, 16) / 16);

	return mbs * inst->fps;
}

static u32 load_per_type(struct venus_core *core, u32 session_type)
{
	struct venus_inst *inst = NULL;
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

static int load_scale_clocks(struct venus_core *core)
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

static void print_header(struct vb2_buffer *vb)
{
	void *vaddr;

	vaddr = vb2_plane_vaddr(vb, 0);

	if (vaddr)
		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, vaddr, 96);
}

static int session_set_buf(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *q = vb->vb2_queue;
	struct venus_inst *inst = vb2_get_drv_priv(q);
	struct venus_core *core = inst->core;
	struct device *dev = core->dev;
	struct hfi_frame_data fdata;
	dma_addr_t dma_addr;
	struct sg_table *sgt;
	int ret;

	memset(&fdata, 0, sizeof(fdata));

	sgt = vb2_dma_sg_plane_desc(vb, 0);
	if (!sgt)
		return -EINVAL;

	dma_addr = sg_dma_address(sgt->sgl);

	fdata.alloc_len = vb2_plane_size(vb, 0);
	fdata.device_addr = dma_addr;
	fdata.timestamp = timeval_to_ns(&vbuf->timestamp) / NSEC_PER_USEC;
	fdata.flags = 0;
	fdata.clnt_data = dma_addr;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		fdata.buffer_type = HFI_BUFFER_INPUT;
		fdata.filled_len = vb2_get_plane_payload(vb, 0);
		fdata.offset = vb->planes[0].data_offset;

		if (vbuf->flags & V4L2_BUF_FLAG_LAST || !fdata.filled_len)
			fdata.flags |= HFI_BUFFERFLAG_EOS;

//		dev_err(core->dev, "etb: index: %u, filled: %u, off: %u, addr: %x\n", vb->index,
//			fdata.filled_len, fdata.offset, fdata.device_addr);

		if (inst->codec_cfg == false &&
		    inst->session_type == VIDC_SESSION_TYPE_DEC) {
			inst->codec_cfg = true;
			fdata.flags |= HFI_BUFFERFLAG_CODECCONFIG;
			print_header(vb);
		}

		ret = hfi_session_etb(inst, &fdata);
	} else if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fdata.buffer_type = HFI_BUFFER_OUTPUT;
		fdata.filled_len = 0;
		fdata.offset = 0;

//		dev_err(core->dev, "ftb: index: %u, addr: %x\n", vb->index,
//			fdata.device_addr);

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

static int
fill_buffer_desc(struct vb2_v4l2_buffer *vb2_v4l2, struct hfi_buffer_desc *bd)
{
	struct vb2_buffer *vb = &vb2_v4l2->vb2_buf;
	struct sg_table *sgt;

	sgt = vb2_dma_sg_plane_desc(vb, 0);
	if (!sgt)
		return -EINVAL;

	bd->buffer_type = HFI_BUFFER_OUTPUT;
	bd->buffer_size = vb2_plane_size(vb, 0);
	bd->num_buffers = 1;
	bd->device_addr = sg_dma_address(sgt->sgl);

	return 0;
}

static int session_unregister_bufs(struct venus_inst *inst)
{
	struct venus_core *core = inst->core;
	struct device *dev = core->dev;
	struct hfi_buffer_desc bd;
	struct v4l2_m2m_buffer *buf;
	int ret;

	if (core->res->hfi_version == HFI_VERSION_3XX)
		return 0;

//	mutex_lock(&inst->lock);
	v4l2_m2m_for_each_dst_buf(inst->m2m_ctx, buf) {
		fill_buffer_desc(&buf->vb, &bd);
		bd.response_required = 1;

		dev_err(dev, "%s: type:%u, size:%u, addr:%x\n", __func__,
			bd.buffer_type, bd.buffer_size, bd.device_addr);

		ret = hfi_session_unset_buffers(inst, &bd);
		if (ret)
			dev_err(dev, "%s: unset buffers failed\n", __func__);
	}
//	mutex_unlock(&inst->lock);

	return 0;
}

static int session_register_bufs(struct venus_inst *inst)
{
	struct venus_core *core = inst->core;
	struct device *dev = core->dev;
	struct hfi_buffer_desc bd;
	struct v4l2_m2m_buffer *buf;
	int ret = 0;

	if (core->res->hfi_version == HFI_VERSION_3XX)
		return 0;

//	mutex_lock(&inst->lock);
	v4l2_m2m_for_each_dst_buf(inst->m2m_ctx, buf) {
		fill_buffer_desc(&buf->vb, &bd);

		dev_err(dev, "%s: type:%u, size:%u, addr:%x\n", __func__,
			bd.buffer_type, bd.buffer_size, bd.device_addr);

		ret = hfi_session_set_buffers(inst, &bd);
		if (ret) {
			dev_err(dev, "%s: session: set buffer failed\n",
				__func__);
			break;
		}
	}
//	mutex_unlock(&inst->lock);

	return ret;
}

int vidc_get_bufreq(struct venus_inst *inst, u32 type,
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

int vidc_set_color_format(struct venus_inst *inst, u32 type, u32 pixfmt)
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
vidc_vb2_find_buf(struct venus_inst *inst, dma_addr_t addr, unsigned int type)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->m2m_ctx;
	struct v4l2_m2m_queue_ctx *m2m_qctx;
	struct v4l2_m2m_buffer *buf;
	struct vb2_buffer *vb;
	struct sg_table *sgt;
	struct vb2_v4l2_buffer *ret = NULL;

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		m2m_qctx = &m2m_ctx->out_q_ctx;
	else
		m2m_qctx = &m2m_ctx->cap_q_ctx;

	list_for_each_entry(buf, &m2m_qctx->rdy_queue, list) {
		vb = &buf->vb.vb2_buf;
		sgt = vb2_dma_sg_plane_desc(vb, 0);
		if (addr == sg_dma_address(sgt->sgl)) {
			list_del(&buf->list);
			m2m_qctx->num_rdy--;
			ret = &buf->vb;
			break;
		}
	}

//	if (ret)
//		v4l2_m2m_buf_remove(m2m_qctx);

	return ret;
}

int vidc_vb2_buf_init(struct vb2_buffer *vb)
{
	return 0;
}

int vidc_vb2_buf_prepare(struct vb2_buffer *vb)
{
	return 0;
}

void vidc_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct venus_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	int ret;

	v4l2_m2m_buf_queue(inst->m2m_ctx, vbuf);

	if (!inst->streamon)
		return;

	ret = session_set_buf(vb);
	if (ret) {
		if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			v4l2_m2m_src_buf_remove(inst->m2m_ctx);
		else
			v4l2_m2m_dst_buf_remove(inst->m2m_ctx);
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}
}

void vidc_vb2_buffers_done(struct venus_inst *inst, enum vb2_buffer_state state)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->m2m_ctx;
	struct v4l2_m2m_queue_ctx *m2m_qctx;
	struct v4l2_m2m_buffer *buf;

	m2m_qctx = &m2m_ctx->out_q_ctx;
	list_for_each_entry(buf, &m2m_qctx->rdy_queue, list)
		v4l2_m2m_buf_done(&buf->vb, state);

	m2m_qctx = &m2m_ctx->cap_q_ctx;
	list_for_each_entry(buf, &m2m_qctx->rdy_queue, list)
		v4l2_m2m_buf_done(&buf->vb, state);
}

void vidc_vb2_stop_streaming(struct vb2_queue *q)
{
	struct venus_inst *inst = vb2_get_drv_priv(q);
	struct venus_core *core = inst->core;
	struct device *dev = core->dev;
	int ret;

	mutex_lock(&inst->lock);

	if (!inst->streamon) {
		mutex_unlock(&inst->lock);
		return;
	}

	dev_err(core->dev, "%s: session stopping\n", __func__);

	ret = hfi_session_stop(inst);
	if (ret) {
		dev_err(dev, "session: stop failed (%d)\n", ret);
		goto abort;
	}

	dev_err(core->dev, "%s: session stopped\n", __func__);

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

	hfi_session_deinit(inst);

	dev_err(core->dev, "%s: actual stop stream\n", __func__);

	pm_runtime_put_sync(dev);

	vidc_vb2_buffers_done(inst, VB2_BUF_STATE_ERROR);

	inst->streamon = inst->streamon_cap = inst->streamon_out = 0;

	mutex_unlock(&inst->lock);
}

int vidc_vb2_start_streaming(struct venus_inst *inst)
{
	struct venus_core *core = inst->core;
	struct v4l2_m2m_buffer *buf;
	struct v4l2_m2m_ctx *m2m_ctx = inst->m2m_ctx;
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

	list_for_each_entry(buf, &m2m_ctx->cap_q_ctx.rdy_queue, list) {
		session_set_buf(&buf->vb.vb2_buf);
	}

	list_for_each_entry(buf, &m2m_ctx->out_q_ctx.rdy_queue, list) {
		session_set_buf(&buf->vb.vb2_buf);
	}

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
	return ret;
}
