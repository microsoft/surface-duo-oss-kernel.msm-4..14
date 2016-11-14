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
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-dma-sg.h>

#include "core.h"
#include "helpers.h"
#include "vdec.h"

#define MACROBLKS_PER_PIXEL	(16 * 16)

static u32 get_framesize_uncompressed(unsigned int plane, u32 width, u32 height)
{
	u32 y_stride, uv_stride, y_plane;
	u32 y_sclines, uv_sclines, uv_plane;
	u32 size;

	//1280x720 = 1280x736 + 1280x368 + 4096 + 8192 =
	//= 942080 + 471040 + 4K + 8K = 1425408
	//				1413248
	//				  12160 = 12K - 128

	y_stride = ALIGN(width, 128);
	uv_stride = ALIGN(width, 128);
	y_sclines = ALIGN(height, 32);
	uv_sclines = ALIGN(((height + 1) >> 1), 16);

	y_plane = y_stride * y_sclines;
	uv_plane = uv_stride * uv_sclines + SZ_4K;
	size = y_plane + uv_plane + SZ_8K;

	return ALIGN(size, SZ_4K);
}

static u32 get_framesize_compressed(u32 mbs_per_frame)
{
	return ((mbs_per_frame * MACROBLKS_PER_PIXEL * 3 / 2) / 2) + 128;
}

static const struct venus_format vdec_formats[] = {
	{
		.pixfmt = V4L2_PIX_FMT_NV12,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG4,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG2,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	}, {
		.pixfmt = V4L2_PIX_FMT_H263,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	}, {
		.pixfmt = V4L2_PIX_FMT_VC1_ANNEX_G,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	}, {
		.pixfmt = V4L2_PIX_FMT_VC1_ANNEX_L,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	}, {
		.pixfmt = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	}, {
		.pixfmt = V4L2_PIX_FMT_VP8,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	}, {
		.pixfmt = V4L2_PIX_FMT_XVID,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	},
};

static const struct venus_format *find_format(u32 pixfmt, u32 type)
{
	const struct venus_format *fmt = vdec_formats;
	unsigned int size = ARRAY_SIZE(vdec_formats);
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (fmt[i].pixfmt == pixfmt)
			break;
	}

	if (i == size || fmt[i].type != type)
		return NULL;

	return &fmt[i];
}

static const struct venus_format *find_format_by_index(int index, u32 type)
{
	const struct venus_format *fmt = vdec_formats;
	unsigned int size = ARRAY_SIZE(vdec_formats);
	int i, k = 0;

	if (index < 0 || index > size)
		return NULL;

	for (i = 0; i < size; i++) {
		if (fmt[i].type != type)
			continue;
		if (k == index)
			break;
		k++;
	}

	if (i == size)
		return NULL;

	return &fmt[i];
}

static const struct venus_format *
vdec_try_fmt_common(struct venus_inst *inst, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *pfmt = pixmp->plane_fmt;
	const struct venus_format *fmt;
	unsigned int p;

	memset(pfmt[0].reserved, 0, sizeof(pfmt[0].reserved));
	memset(pixmp->reserved, 0, sizeof(pixmp->reserved));

	fmt = find_format(pixmp->pixelformat, f->type);
	if (!fmt) {
		if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
			pixmp->pixelformat = V4L2_PIX_FMT_NV12;
		else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			pixmp->pixelformat = V4L2_PIX_FMT_H264;
		else
			return NULL;
		fmt = find_format(pixmp->pixelformat, f->type);
		pixmp->width = 1280;
		pixmp->height = 720;
	}

	pixmp->width = clamp(pixmp->width, inst->cap_width.min,
			     inst->cap_width.max);
	pixmp->height = clamp(pixmp->height, inst->cap_height.min,
			      inst->cap_height.max);

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		pixmp->height = ALIGN(pixmp->height, 32);

	if (pixmp->field == V4L2_FIELD_ANY)
		pixmp->field = V4L2_FIELD_NONE;
	pixmp->num_planes = fmt->num_planes;
	pixmp->flags = 0;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		for (p = 0; p < pixmp->num_planes; p++) {
			pfmt[p].sizeimage =
				get_framesize_uncompressed(p, pixmp->width,
							   pixmp->height);
			pfmt[p].bytesperline = ALIGN(pixmp->width, 128);
		}
	} else {
		u32 mbs = pixmp->width * pixmp->height / MACROBLKS_PER_PIXEL;

		pfmt[0].sizeimage = get_framesize_compressed(mbs);
		pfmt[0].bytesperline = 0;
	}

	return fmt;
}

static int vdec_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct venus_inst *inst = to_inst(file);
	const struct venus_format *fmt;

	fmt = vdec_try_fmt_common(inst, f);
	if (!fmt)
		return -EINVAL;

	return 0;
}

static int vdec_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct venus_inst *inst = to_inst(file);
	const struct venus_format *fmt = NULL;
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		fmt = inst->fmt_cap;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		fmt = inst->fmt_out;

	if (inst->in_reconfig) {
		struct v4l2_format format = {};

		inst->out_width = inst->reconfig_width;
		inst->out_height = inst->reconfig_height;
		inst->in_reconfig = false;

		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		format.fmt.pix_mp.pixelformat = inst->fmt_cap->pixfmt;
		format.fmt.pix_mp.width = inst->out_width;
		format.fmt.pix_mp.height = inst->out_height;

		vdec_try_fmt_common(inst, &format);

		inst->width = format.fmt.pix_mp.width;
		inst->height = format.fmt.pix_mp.height;
	}

	pixmp->pixelformat = fmt->pixfmt;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixmp->width = inst->width;
		pixmp->height = inst->height;
		pixmp->colorspace = inst->colorspace;
		pixmp->ycbcr_enc = inst->ycbcr_enc;
		pixmp->quantization = inst->quantization;
		pixmp->xfer_func = inst->xfer_func;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixmp->width = inst->out_width;
		pixmp->height = inst->out_height;
	}

	vdec_try_fmt_common(inst, f);

	pr_err("g_fmt: type:%u, %ux%u\n", f->type, pixmp->width, pixmp->height);

	return 0;
}

#if 0
static int
vdec_check_req(struct vidc_inst *inst, unsigned int width, unsigned int height,
		struct hfi_buffer_requirements *bufreq_in,
		struct hfi_buffer_requirements *bufreq_out)
{
	struct hfi_buffer_requirements bufreq;
	u32 pixfmt = inst->fmt_out->pixfmt;
	struct hfi_framesize fs;
	u32 ptype;
	int ret;

	ret = pm_runtime_get_sync(inst->core->dev);
	if (ret < 0)
		return ret;

	ret = hfi_session_init(inst, pixfmt, VIDC_SESSION_TYPE_DEC);
	if (ret)
		return ret;

	ptype = HFI_PROPERTY_PARAM_FRAME_SIZE;
	fs.buffer_type = HFI_BUFFER_INPUT;
	fs.width = width;
	fs.height = height;

	ret = hfi_session_set_property(inst, ptype, &fs);
	if (ret)
		goto err;

	pixfmt = inst->fmt_cap->pixfmt;

	ret = vidc_set_color_format(inst, HFI_BUFFER_OUTPUT, pixfmt);
	if (ret)
		goto err;

	ret = vidc_get_bufreq(inst, HFI_BUFFER_INPUT, &bufreq);
	if (ret)
		goto err;

	pr_err("%s: IN : type:%u, size:%u, min:%u, actual:%u\n", __func__,
		bufreq.type, bufreq.size, bufreq.count_min,
		bufreq.count_actual);
	if (bufreq_in)
		*bufreq_in = bufreq;

	memset(&bufreq, 0, sizeof(bufreq));
	ret = vidc_get_bufreq(inst, HFI_BUFFER_OUTPUT, &bufreq);
	if (ret)
		goto err;

	pr_err("%s: OUT: type:%u, size:%u, min:%u, actual:%u\n", __func__,
		bufreq.type, bufreq.size, bufreq.count_min,
		bufreq.count_actual);
	if (bufreq_out)
		*bufreq_out = bufreq;

	hfi_session_deinit(inst);
	pm_runtime_put_sync(inst->core->dev);
	return 0;

err:
	hfi_session_deinit(inst);
	pm_runtime_put_sync(inst->core->dev);
	return ret;
}
#endif

static int vdec_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct venus_inst *inst = to_inst(file);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	struct v4l2_pix_format_mplane orig_pixmp;
	const struct venus_format *fmt;
	struct v4l2_format format;
	u32 pixfmt_out = 0, pixfmt_cap = 0;

	orig_pixmp = *pixmp;

	pr_err("s_fmt: enter: type:%u, pixfmt:%u, %ux%u\n", f->type,
		pixmp->pixelformat, pixmp->width, pixmp->height);

	fmt = vdec_try_fmt_common(inst, f);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixfmt_out = pixmp->pixelformat;
		pixfmt_cap = inst->fmt_cap->pixfmt;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixfmt_cap = pixmp->pixelformat;
		pixfmt_out = inst->fmt_out->pixfmt;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_out;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	vdec_try_fmt_common(inst, &format);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		inst->out_width = format.fmt.pix_mp.width;
		inst->out_height = format.fmt.pix_mp.height;
		inst->colorspace = pixmp->colorspace;
		inst->ycbcr_enc = pixmp->ycbcr_enc;
		inst->quantization = pixmp->quantization;
		inst->xfer_func = pixmp->xfer_func;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_cap;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	vdec_try_fmt_common(inst, &format);

	inst->width = format.fmt.pix_mp.width;
	inst->height = format.fmt.pix_mp.height;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		inst->fmt_out = fmt;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		inst->fmt_cap = fmt;

	pr_err("s_fmt: exit: type:%u, %ux%u, out %ux%u (orig:%ux%u, ret:%ux%u)\n",
		f->type,
		inst->width, inst->height,
		inst->out_width, inst->out_height,
		orig_pixmp.width, orig_pixmp.height,
		pixmp->width, pixmp->height);

	return 0;
}

static int
vdec_g_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct venus_inst *inst = to_inst(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
		if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
			return -EINVAL;
		s->r.width = inst->out_width;
		s->r.height = inst->out_height;
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		s->r.width = inst->width;
		s->r.height = inst->height;
		break;
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE:
		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		s->r.width = inst->out_width;
		s->r.height = inst->out_height;
		break;
	default:
		return -EINVAL;
	}

	s->r.top = 0;
	s->r.left = 0;

	return 0;
}

static int
vdec_s_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct venus_inst *inst = to_inst(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
		return -EINVAL;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		return -EINVAL;
	case V4L2_SEL_TGT_COMPOSE:
		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		s->r.width = inst->out_width;
		s->r.height = inst->out_height;
		break;
	default:
		return -EINVAL;
	}

	s->r.top = 0;
	s->r.left = 0;

	return 0;
}

static int
vdec_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *b)
{
	struct vb2_queue *queue = to_vb2q(file, b->type);

	if (!queue)
		return -EINVAL;

	return vb2_reqbufs(queue, b);
}

static int
vdec_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strlcpy(cap->driver, "qcom-venus", sizeof(cap->driver));
	strlcpy(cap->card, "Qualcomm Venus video decoder", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:qcom-venus", sizeof(cap->bus_info));

	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int vdec_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	const struct venus_format *fmt;

	memset(f->reserved, 0, sizeof(f->reserved));

	fmt = find_format_by_index(f->index, f->type);
	if (!fmt)
		return -EINVAL;

	f->pixelformat = fmt->pixfmt;

	return 0;
}

static int vdec_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vb2_queue *queue = to_vb2q(file, b->type);
	unsigned int p;
	int ret;

	if (!queue)
		return -EINVAL;

	ret = vb2_querybuf(queue, b);
	if (ret)
		return ret;

	if (b->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    b->memory == V4L2_MEMORY_MMAP) {
		for (p = 0; p < b->length; p++)
			b->m.planes[p].m.mem_offset += DST_QUEUE_OFF_BASE;
	}

	return 0;
}

static int
vdec_create_bufs(struct file *file, void *fh, struct v4l2_create_buffers *b)
{
	struct vb2_queue *queue = to_vb2q(file, b->format.type);

	if (!queue)
		return -EINVAL;

	return vb2_create_bufs(queue, b);
}

static int vdec_prepare_buf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vb2_queue *queue = to_vb2q(file, b->type);

	if (!queue)
		return -EINVAL;

	return vb2_prepare_buf(queue, b);
}

static int vdec_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vb2_queue *queue = to_vb2q(file, b->type);

	if (!queue)
		return -EINVAL;

	return vb2_qbuf(queue, b);
}

static int
vdec_exportbuf(struct file *file, void *fh, struct v4l2_exportbuffer *b)
{
	struct vb2_queue *queue = to_vb2q(file, b->type);

	if (!queue)
		return -EINVAL;

	return vb2_expbuf(queue, b);
}

static int vdec_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vb2_queue *queue = to_vb2q(file, b->type);

	if (!queue)
		return -EINVAL;

	return vb2_dqbuf(queue, b, file->f_flags & O_NONBLOCK);
}

static int vdec_streamon(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct vb2_queue *queue = to_vb2q(file, type);

	if (!queue)
		return -EINVAL;

	return vb2_streamon(queue, type);
}

static int vdec_streamoff(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct vb2_queue *queue = to_vb2q(file, type);

	if (!queue)
		return -EINVAL;

	return vb2_streamoff(queue, type);
}

static int vdec_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct venus_inst *inst = to_inst(file);
	struct v4l2_captureparm *cap = &a->parm.capture;
	struct v4l2_fract *timeperframe = &cap->timeperframe;
	u64 us_per_frame, fps;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	memset(cap->reserved, 0, sizeof(cap->reserved));
	if (!timeperframe->denominator)
		timeperframe->denominator = inst->timeperframe.denominator;
	if (!timeperframe->numerator)
		timeperframe->numerator = inst->timeperframe.numerator;
	cap->readbuffers = 0;
	cap->extendedmode = 0;
	cap->capability = V4L2_CAP_TIMEPERFRAME;
	us_per_frame = timeperframe->numerator * (u64)USEC_PER_SEC;
	do_div(us_per_frame, timeperframe->denominator);

	if (!us_per_frame)
		return -EINVAL;

	fps = (u64)USEC_PER_SEC;
	do_div(fps, us_per_frame);

	inst->fps = fps;
	inst->timeperframe = *timeperframe;

	return 0;
}

static int vdec_enum_framesizes(struct file *file, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	struct venus_inst *inst = to_inst(file);
	const struct venus_format *fmt;

	fmt = find_format(fsize->pixel_format,
			  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (!fmt) {
		fmt = find_format(fsize->pixel_format,
				  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
		if (!fmt)
			return -EINVAL;
	}

	if (fsize->index)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;

	fsize->stepwise.min_width = inst->cap_width.min;
	fsize->stepwise.max_width = inst->cap_width.max;
	fsize->stepwise.step_width = inst->cap_width.step_size;
	fsize->stepwise.min_height = inst->cap_height.min;
	fsize->stepwise.max_height = inst->cap_height.max;
	fsize->stepwise.step_height = inst->cap_height.step_size;

	return 0;
}

static int vdec_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ioctl_ops vdec_ioctl_ops = {
	.vidioc_querycap = vdec_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = vdec_enum_fmt,
	.vidioc_enum_fmt_vid_out_mplane = vdec_enum_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vdec_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vdec_s_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vdec_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vdec_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vdec_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vdec_try_fmt,
	.vidioc_g_selection = vdec_g_selection,
	.vidioc_s_selection = vdec_s_selection,
	.vidioc_reqbufs = vdec_reqbufs,
	.vidioc_querybuf = vdec_querybuf,
	.vidioc_create_bufs = vdec_create_bufs,
	.vidioc_prepare_buf = vdec_prepare_buf,
	.vidioc_qbuf = vdec_qbuf,
	.vidioc_expbuf = vdec_exportbuf,
	.vidioc_dqbuf = vdec_dqbuf,
	.vidioc_streamon = vdec_streamon,
	.vidioc_streamoff = vdec_streamoff,
	.vidioc_s_parm = vdec_s_parm,
	.vidioc_enum_framesizes = vdec_enum_framesizes,
	.vidioc_subscribe_event = vdec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int vdec_set_properties(struct venus_inst *inst)
{
	struct vdec_controls *ctr = &inst->controls.dec;
	struct venus_core *core = inst->core;
	struct hfi_enable en = { .enable = 1 };
	u32 ptype;
	int ret;

	ptype = HFI_PROPERTY_PARAM_VDEC_CONTINUE_DATA_TRANSFER;
	ret = hfi_session_set_property(inst, ptype, &en);
	if (ret)
		return ret;

	if (core->res->hfi_version == HFI_VERSION_3XX) {
		struct hfi_buffer_alloc_mode mode;

		ptype = HFI_PROPERTY_PARAM_BUFFER_ALLOC_MODE;
		mode.type = HFI_BUFFER_OUTPUT;
		mode.mode = HFI_BUFFER_MODE_DYNAMIC;

		ret = hfi_session_set_property(inst, ptype, &mode);
		if (ret)
			return ret;
	}

	if (ctr->post_loop_deb_mode) {
		ptype = HFI_PROPERTY_CONFIG_VDEC_POST_LOOP_DEBLOCKER;
		en.enable = 1;
		ret = hfi_session_set_property(inst, ptype, &en);
		if (ret)
			return ret;
	}

	return 0;
}

static int vdec_init_session(struct venus_inst *inst)
{
	u32 pixfmt = inst->fmt_out->pixfmt;
	struct hfi_framesize fs;
	u32 ptype;
	int ret;

	ret = hfi_session_init(inst, pixfmt, VIDC_SESSION_TYPE_DEC);
	if (ret)
		return ret;

	ptype = HFI_PROPERTY_PARAM_FRAME_SIZE;
	fs.buffer_type = HFI_BUFFER_INPUT;
	fs.width = inst->out_width;
	fs.height = inst->out_height;

	ret = hfi_session_set_property(inst, ptype, &fs);
	if (ret)
		goto err;

	pixfmt = inst->fmt_cap->pixfmt;

	ret = vidc_set_color_format(inst, HFI_BUFFER_OUTPUT, pixfmt);
	if (ret)
		goto err;

	return 0;
err:
	hfi_session_deinit(inst);
	return ret;
}

static int vdec_cap_num_buffers(struct venus_inst *inst, unsigned int *num)
{
	struct venus_core *core = inst->core;
	struct hfi_buffer_requirements bufreq;
	struct device *dev = core->dev;
	int ret, ret2;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		return ret;

	ret = vdec_init_session(inst);
	if (ret)
		goto put_sync;

	ret = vidc_get_bufreq(inst, HFI_BUFFER_OUTPUT, &bufreq);

	*num = bufreq.count_actual;

	hfi_session_deinit(inst);

put_sync:
	ret2 = pm_runtime_put_sync(dev);

	return ret ? ret : ret2;
}
#if 1
static int vdec_queue_setup(struct vb2_queue *q, const void *parg,
			    unsigned int *num_buffers, unsigned int *num_planes,
			    unsigned int sizes[], void *alloc_ctxs[])
{
	struct venus_inst *inst = vb2_get_drv_priv(q);
	unsigned int p, num;
	int ret = 0;
	u32 mbs;

	switch (q->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		*num_planes = inst->fmt_out->num_planes;
		mbs = inst->out_width * inst->out_height / MACROBLKS_PER_PIXEL;
		sizes[0] = get_framesize_compressed(mbs);
		dev_err(inst->core->dev, "input : size:%u\n", sizes[0]);
		inst->num_input_bufs = *num_buffers;
		alloc_ctxs[0] = inst->alloc_ctx_out;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		*num_planes = inst->fmt_cap->num_planes;

		ret = vdec_cap_num_buffers(inst, &num);
		if (ret)
			break;

		*num_buffers = max(*num_buffers, num);

		for (p = 0; p < *num_planes; p++)
			sizes[p] = get_framesize_uncompressed(p, inst->width,
							      inst->height);

		dev_err(inst->core->dev, "output: size:%u\n", sizes[0]);

		inst->num_output_bufs = *num_buffers;
		inst->output_buf_size = sizes[0];
		alloc_ctxs[0] = inst->alloc_ctx_cap;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#else
static int vdec_queue_setup(struct vb2_queue *q, const void *parg,
			    unsigned int *num_buffers, unsigned int *num_planes,
			    unsigned int sizes[], void *alloc_ctxs[])
{
	struct venus_inst *inst = vb2_get_drv_priv(q);
	struct hfi_buffer_requirements bufreq;
	int ret = 0;

	pr_err("%s: enter\n", __func__);

	switch (q->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		*num_planes = inst->fmt_out->num_planes;
		ret = vdec_check_req(inst, inst->out_width, inst->out_height,
					&bufreq, NULL);
		if (ret)
			return ret;
		sizes[0] = round_up(bufreq.size, SZ_4K);
		if (*num_buffers < bufreq.count_min)
			*num_buffers = bufreq.count_min;
		inst->num_input_bufs = *num_buffers;
		alloc_ctxs[0] = inst->alloc_ctx_out;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		*num_planes = inst->fmt_cap->num_planes;
		ret = vdec_check_req(inst, inst->out_width, inst->out_height,
					NULL, &bufreq);
		if (ret)
			return ret;
		sizes[0] = round_up(bufreq.size, SZ_4K);
		if (*num_buffers < bufreq.count_min)
			*num_buffers = bufreq.count_min;
		inst->num_output_bufs = *num_buffers;
		inst->output_buf_size = sizes[0];
		alloc_ctxs[0] = inst->alloc_ctx_cap;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pr_err("%s: exit %d (in:%u out:%u)\n", __func__, ret,
		inst->num_input_bufs, inst->num_output_bufs);

	return ret;
}
#endif

static int vdec_check_configuration(struct venus_inst *inst)
{
	struct hfi_buffer_requirements bufreq;
	int ret;

	ret = vidc_get_bufreq(inst, HFI_BUFFER_OUTPUT, &bufreq);
	if (ret)
		return ret;

	pr_err("%s: OUT : type:%u, size:%u, min:%u, actual:%u\n", __func__,
		bufreq.type, bufreq.size, bufreq.count_min,
		bufreq.count_actual);

	if (inst->num_output_bufs < bufreq.count_actual ||
	    inst->num_output_bufs < bufreq.count_min)
		return -EINVAL;

	ret = vidc_get_bufreq(inst, HFI_BUFFER_INPUT, &bufreq);
	if (ret)
		return ret;

	pr_err("%s: IN : type:%u, size:%u, min:%u, actual:%u\n", __func__,
		bufreq.type, bufreq.size, bufreq.count_min,
		bufreq.count_actual);

	if (inst->num_input_bufs < bufreq.count_min)
		return -EINVAL;

	return 0;
}

static int vdec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct venus_inst *inst = vb2_get_drv_priv(q);
	struct venus_core *core = inst->core;
	struct device *dev = core->dev;
	struct hfi_buffer_count_actual buf_count;
	struct vb2_queue *other_queue;
	u32 ptype;
	int ret;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		other_queue = &inst->bufq_cap;
	else
		other_queue = &inst->bufq_out;

	if (!vb2_is_streaming(other_queue))
		return 0;

//	if (vb2_start_streaming_called(other_queue))
//		return 0;

	dev_err(core->dev, "%s: actual start stream (type:%u)\n", __func__,
		q->type);

	inst->in_reconfig = false;
	inst->sequence = 0;
	inst->codec_cfg = false;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		return ret;

	ret = vdec_init_session(inst);
	if (ret)
		goto put_sync;

	ret = vdec_set_properties(inst);
	if (ret)
		goto deinit_sess;

	if (core->res->hfi_version == HFI_VERSION_3XX) {
		struct hfi_buffer_size_actual buf_sz;

		ptype = HFI_PROPERTY_PARAM_BUFFER_SIZE_ACTUAL;
		buf_sz.type = HFI_BUFFER_OUTPUT;
		buf_sz.size = inst->output_buf_size;

		ret = hfi_session_set_property(inst, ptype, &buf_sz);
		if (ret)
			goto deinit_sess;
	}

	ptype = HFI_PROPERTY_PARAM_BUFFER_COUNT_ACTUAL;
	buf_count.type = HFI_BUFFER_INPUT;
	buf_count.count_actual = inst->num_input_bufs;

	ret = hfi_session_set_property(inst, ptype, &buf_count);
	if (ret)
		goto deinit_sess;

	ptype = HFI_PROPERTY_PARAM_BUFFER_COUNT_ACTUAL;
	buf_count.type = HFI_BUFFER_OUTPUT;
	buf_count.count_actual = inst->num_output_bufs;

	ret = hfi_session_set_property(inst, ptype, &buf_count);
	if (ret)
		goto deinit_sess;

	ret = vdec_check_configuration(inst);
	if (ret)
		goto deinit_sess;

	ret = vidc_vb2_start_streaming(inst);
	if (ret)
		goto deinit_sess;

	return 0;

deinit_sess:
	hfi_session_deinit(inst);
put_sync:
	pm_runtime_put_sync(dev);
	vidc_vb2_buffers_done(inst, VB2_BUF_STATE_QUEUED);
	return ret;
}

static const struct vb2_ops vdec_vb2_ops = {
	.queue_setup = vdec_queue_setup,
	.buf_init = vidc_vb2_buf_init,
	.buf_prepare = vidc_vb2_buf_prepare,
	.start_streaming = vdec_start_streaming,
	.stop_streaming = vidc_vb2_stop_streaming,
	.buf_queue = vidc_vb2_buf_queue,
};

static int vdec_empty_buf_done(struct venus_inst *inst, u32 addr, u32 bytesused,
			       u32 data_offset, u32 flags)
{
	struct vb2_v4l2_buffer *vbuf;
	struct vb2_buffer *vb;

	vbuf = vidc_vb2_find_buf(inst, addr);
	if (!vbuf)
		return -EINVAL;

	vb = &vbuf->vb2_buf;
	vbuf->flags = flags;

	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);

//	dev_err(inst->core->dev, "ebd: index: %u\n", vb->index);

	return 0;
}

static int vdec_fill_buf_done(struct venus_inst *inst, u32 addr, u32 bytesused,
			      u32 data_offset, u32 flags, u64 timestamp_us)
{
	struct vb2_v4l2_buffer *vbuf;
	struct vb2_buffer *vb;

	vbuf = vidc_vb2_find_buf(inst, addr);
	if (!vbuf)
		return -EINVAL;

	vb = &vbuf->vb2_buf;
	vb->planes[0].bytesused = max_t(unsigned int, inst->output_buf_size,
					bytesused);
	vb->planes[0].data_offset = data_offset;
	vbuf->timestamp = ns_to_timeval(timestamp_us * NSEC_PER_USEC);
	vbuf->flags = flags;
	vbuf->sequence = inst->sequence++;

	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);

	if (vbuf->flags & V4L2_BUF_FLAG_LAST) {
		const struct v4l2_event ev = { .type = V4L2_EVENT_EOS };

		v4l2_event_queue_fh(&inst->fh, &ev);
	}

//	dev_err(inst->core->dev, "fbd: index: %u, flags: %x, ts:%lld\n", vb->index, flags,
//		timestamp_us);

	return 0;
}

static int vdec_event_notify(struct venus_inst *inst, u32 event,
			     struct hfi_event_data *data)
{
	struct venus_core *core = inst->core;
	struct device *dev = core->dev;
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION };

	switch (event) {
	case EVT_SESSION_ERROR:
		if (inst) {
			mutex_lock(&inst->lock);
			inst->state = INST_INVALID;
			mutex_unlock(&inst->lock);
		}
		dev_err(dev, "dec: event session error (inst:%p)\n", inst);
		break;
	case EVT_SYS_EVENT_CHANGE:
		switch (data->event_type) {
		case HFI_EVENT_DATA_SEQUENCE_CHANGED_SUFFICIENT_BUF_RESOURCES:
			hfi_session_continue(inst);
			dev_err(dev, "event sufficient resources\n");
			break;
		case HFI_EVENT_DATA_SEQUENCE_CHANGED_INSUFFICIENT_BUF_RESOURCES:
			inst->reconfig_height = data->height;
			inst->reconfig_width = data->width;
			inst->in_reconfig = true;

			v4l2_event_queue_fh(&inst->fh, &ev);

			dev_err(dev, "event not sufficient resources (%ux%u)\n",
				data->width, data->height);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return 0;
}

static const struct hfi_inst_ops vdec_hfi_ops = {
	.empty_buf_done = vdec_empty_buf_done,
	.fill_buf_done = vdec_fill_buf_done,
	.event_notify = vdec_event_notify,
};

static void vdec_inst_init(struct venus_inst *inst)
{
	inst->fmt_out = &vdec_formats[6];
	inst->fmt_cap = &vdec_formats[0];
	inst->width = 1280;
	inst->height = ALIGN(720, 32);
	inst->out_width = 1280;
	inst->out_height = 720;
	inst->fps = 30;
	inst->timeperframe.numerator = 1;
	inst->timeperframe.denominator = 30;

	inst->cap_width.min = 64;
	inst->cap_width.max = 1920;
	inst->cap_width.step_size = 1;
	inst->cap_height.min = 64;
	inst->cap_height.max = ALIGN(1080, 32);
	inst->cap_height.step_size = 1;
	inst->cap_framerate.min = 1;
	inst->cap_framerate.max = 30;
	inst->cap_framerate.step_size = 1;
	inst->cap_mbs_per_frame.min = 16;
	inst->cap_mbs_per_frame.max = 8160;
}

int vdec_init(struct venus_core *core, struct video_device *dec,
	      const struct v4l2_file_operations *fops)
{
	int ret;

	dec->release = video_device_release;
	dec->fops = fops;
	dec->ioctl_ops = &vdec_ioctl_ops;
	dec->vfl_dir = VFL_DIR_M2M;
	dec->v4l2_dev = &core->v4l2_dev;

	ret = video_register_device(dec, VFL_TYPE_GRABBER, -1);
	if (ret)
		return ret;

	video_set_drvdata(dec, core);

	return 0;
}

void vdec_deinit(struct venus_core *core, struct video_device *dec)
{
	video_unregister_device(dec);
}

int vdec_open(struct venus_inst *inst)
{
	struct vb2_queue *q;
	int ret;

	inst->alloc_ctx_cap = vb2_dma_sg_init_ctx(inst->core->dev);
	if (!inst->alloc_ctx_cap)
		return -ENOMEM;

	inst->alloc_ctx_out = vb2_dma_sg_init_ctx(inst->core->dev);
	if (!inst->alloc_ctx_out) {
		vb2_dma_sg_cleanup_ctx(inst->alloc_ctx_cap);
		return -ENOMEM;
	}

	ret = vdec_ctrl_init(inst);
	if (ret)
		goto err_alloc_ctx;

	ret = hfi_session_create(inst, &vdec_hfi_ops);
	if (ret)
		goto err_ctrl_deinit;

	vdec_inst_init(inst);

	q = &inst->bufq_cap;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->ops = &vdec_vb2_ops;
	q->mem_ops = &vb2_dma_sg_memops;
	q->drv_priv = inst;
	q->buf_struct_size = sizeof(struct vidc_buffer);
	q->allow_zero_bytesused = 1;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_session_destroy;

	q = &inst->bufq_out;
	q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->ops = &vdec_vb2_ops;
	q->mem_ops = &vb2_dma_sg_memops;
	q->drv_priv = inst;
	q->buf_struct_size = sizeof(struct vidc_buffer);
	q->allow_zero_bytesused = 1;
	q->min_buffers_needed = 1;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_cap_queue_release;

	return 0;
err_cap_queue_release:
	vb2_queue_release(&inst->bufq_cap);
err_session_destroy:
	hfi_session_destroy(inst);
err_ctrl_deinit:
	vdec_ctrl_deinit(inst);
err_alloc_ctx:
	vb2_dma_sg_cleanup_ctx(inst->alloc_ctx_cap);
	vb2_dma_sg_cleanup_ctx(inst->alloc_ctx_out);
	return ret;
}

void vdec_close(struct venus_inst *inst)
{
	vb2_queue_release(&inst->bufq_out);
	vb2_queue_release(&inst->bufq_cap);
	vdec_ctrl_deinit(inst);
	hfi_session_destroy(inst);
	vb2_dma_sg_cleanup_ctx(inst->alloc_ctx_cap);
	vb2_dma_sg_cleanup_ctx(inst->alloc_ctx_out);
}
