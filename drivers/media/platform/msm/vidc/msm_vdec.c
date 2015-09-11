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
#include <linux/qcom_scm.h>
#include <linux/qcom_iommu.h>
#include <media/videobuf2-dma-contig.h>

#include "msm_vdec.h"
#include "msm_vdec-ctrls.h"
#include "msm_vidc_internal.h"
#include "msm_internal-buffers.h"
#include "msm_vidc_common.h"
#include "msm_vidc_load.h"
#include "hfi/vidc_hfi_api.h"
#include "msm_vidc_debug.h"
#include "msm_hfi_interface.h"

#define MIN_NUM_OUTPUT_BUFFERS		4
#define MAX_NUM_OUTPUT_BUFFERS		VIDEO_MAX_FRAME
#define MB_SIZE_IN_PIXEL		(16 * 16)

/* Offset base for buffers on the destination queue - used to distinguish
 * between source and destination buffers when mmapping - they receive the same
 * offsets but for different queues */
#define DST_QUEUE_OFF_BASE	(1 << 30)

struct vdec_buffer {
	struct vb2_buffer vb;
	struct list_head list;
	dma_addr_t dma_addr;
	struct buffer_info bi;
};

#define to_vdec_buffer(buf)	container_of(buf, struct vdec_buffer, vb)

static inline struct vidc_inst *to_inst(struct file *filp, void *fh)
{
	return container_of(filp->private_data, struct vidc_inst, fh);
}

static u32 get_frame_size_nv12(int plane, u32 height, u32 width)
{
	return VENUS_BUFFER_SIZE(COLOR_FMT_NV12, width, height);
}

static u32 get_frame_size_compressed(int plane, u32 max_mbs_per_frame,
				     u32 size_per_mb)
{
	return (max_mbs_per_frame * size_per_mb * 3 / 2) / 2;
}

static u32 get_frame_size(struct vidc_inst *inst, const struct vidc_format *fmt,
			  int fmt_type, int plane)
{
	u32 frame_size = 0;

	if (fmt_type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		frame_size = fmt->get_frame_size(plane,
					inst->capability.mbs_per_frame.max,
					MB_SIZE_IN_PIXEL);
		if (inst->capability.buffer_size_limit &&
		    inst->capability.buffer_size_limit < frame_size) {
			frame_size = inst->capability.buffer_size_limit;
			dprintk(VIDC_DBG, "input buffer size limited to %d\n",
				frame_size);
		} else {
			dprintk(VIDC_DBG, "set input buffer size to %d\n",
				frame_size);
		}
	} else if (fmt_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		frame_size = fmt->get_frame_size(plane,
					inst->capability.height.max,
					inst->capability.width.max);
		dprintk(VIDC_DBG, "set output buffer size to %d\n",
			frame_size);
	} else {
		dprintk(VIDC_WARN, "Wrong format type\n");
	}

	return frame_size;
}

static const struct vidc_format vdec_formats[] = {
	{
		.name = "YCbCr Semiplanar 4:2:0",
		.description = "Y/CbCr 4:2:0",
		.fourcc = V4L2_PIX_FMT_NV12,
		.num_planes = 1, /* TODO: was 2 */
		.get_frame_size = get_frame_size_nv12,
		.type = CAPTURE_PORT,
	},
	{
		.name = "Mpeg4",
		.description = "Mpeg4 compressed format",
		.fourcc = V4L2_PIX_FMT_MPEG4,
		.num_planes = 1,
		.get_frame_size = get_frame_size_compressed,
		.type = OUTPUT_PORT,
	},
	{
		.name = "Mpeg2",
		.description = "Mpeg2 compressed format",
		.fourcc = V4L2_PIX_FMT_MPEG2,
		.num_planes = 1,
		.get_frame_size = get_frame_size_compressed,
		.type = OUTPUT_PORT,
	},
	{
		.name = "H263",
		.description = "H263 compressed format",
		.fourcc = V4L2_PIX_FMT_H263,
		.num_planes = 1,
		.get_frame_size = get_frame_size_compressed,
		.type = OUTPUT_PORT,
	},
	{
		.name = "VC1",
		.description = "VC-1 compressed format",
		.fourcc = V4L2_PIX_FMT_VC1_ANNEX_G,
		.num_planes = 1,
		.get_frame_size = get_frame_size_compressed,
		.type = OUTPUT_PORT,
	},
	{
		.name = "VC1 SP",
		.description = "VC-1 compressed format G",
		.fourcc = V4L2_PIX_FMT_VC1_ANNEX_L,
		.num_planes = 1,
		.get_frame_size = get_frame_size_compressed,
		.type = OUTPUT_PORT,
	},
	{
		.name = "H264",
		.description = "H264 compressed format",
		.fourcc = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.get_frame_size = get_frame_size_compressed,
		.type = OUTPUT_PORT,
	},
	{
		.name = "VP8",
		.description = "VP8 compressed format",
		.fourcc = V4L2_PIX_FMT_VP8,
		.num_planes = 1,
		.get_frame_size = get_frame_size_compressed,
		.type = OUTPUT_PORT,
	},
};

static const struct vidc_format *find_format(u32 fourcc, int type)
{
	const struct vidc_format *fmt = vdec_formats;
	unsigned int size = ARRAY_SIZE(vdec_formats);
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (fmt[i].fourcc == fourcc)
			break;
	}

	if (i == size || fmt[i].type != type)
		return NULL;

	return &fmt[i];
}


static const struct vidc_format *find_format_by_index(int index, int type)
{
	const struct vidc_format *fmt = vdec_formats;
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

static struct vb2_queue *
vdec_to_vb2q(struct vidc_inst *inst, enum v4l2_buf_type type)
{
	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return &inst->bufq_cap;
	else if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return &inst->bufq_out;

	return NULL;
}

static int vdec_streamon(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = vdec_to_vb2q(inst, type);
	if (!queue)
		return -EINVAL;

	return vb2_streamon(queue, type);
}

static int vdec_streamoff(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = vdec_to_vb2q(inst, type);
	if (!queue)
		return -EINVAL;

	return vb2_streamoff(queue, type);
}

static int vdec_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;
	int ret = 0, i;

	if (b->memory != V4L2_MEMORY_MMAP) {
		dprintk(VIDC_ERR, "Only MMAP bufs can be queried\n");
		return -EINVAL;
	}

	queue = vdec_to_vb2q(inst, b->type);
	if (!queue)
		return -EINVAL;

	ret = vb2_querybuf(queue, b);
	if (ret)
		return ret;

	if (b->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		for (i = 0; i < b->length; i++)
			b->m.planes[i].m.mem_offset += DST_QUEUE_OFF_BASE;
	}

	return 0;
}

static int
vdec_exportbuf(struct file *file, void *fh, struct v4l2_exportbuffer *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = vdec_to_vb2q(inst, b->type);
	if (!queue)
		return -EINVAL;

	return vb2_expbuf(queue, b);
}

static int vdec_prepare_buf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = vdec_to_vb2q(inst, b->type);
	if (!queue)
		return -EINVAL;

	return vb2_prepare_buf(queue, b);
}

static int vdec_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;
	unsigned int state;

	queue = vdec_to_vb2q(inst, b->type);
	if (!queue)
		return -EINVAL;

	mutex_lock(&inst->lock);
	state = inst->state;
	mutex_unlock(&inst->lock);

	/*
	 * it is possible userspace to continue to queuing buffres even
	 * while we are in streamoff. Not sure is this a problem in
	 * videobuf2 core, still. Fix it here for now.
	 */
	if (state >= INST_STOP)
		return -EINVAL;

	return vb2_qbuf(queue, b);
}

static int vdec_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = vdec_to_vb2q(inst, b->type);
	if (!queue)
		return -EINVAL;

	return vb2_dqbuf(queue, b, file->f_flags & O_NONBLOCK);
}

static int
vdec_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = vdec_to_vb2q(inst, b->type);
	if (!queue)
		return -EINVAL;

	return vb2_reqbufs(queue, b);
}

static int vdec_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct device *dev = &inst->core->res.pdev->dev;
	const struct vidc_format *fmt = NULL;
	struct hfi_device *hdev;
	int i = 0, stride = 0, scanlines = 0;
	unsigned int *plane_sizes = NULL, extra_idx = 0;
	struct hal_buffer_requirements *bufreq;
	int ret;

	if (!inst || !f || !inst->core || !inst->core->hfidev)
		return -EINVAL;

	ret = vidc_comm_get_bufreqs(inst);
	if (ret) {
		dev_err(dev, "getting buffer requirements failed (%d)\n", ret);
		return ret;
	}

	hdev = inst->core->hfidev;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		fmt = inst->fmts[CAPTURE_PORT];
	else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		fmt = inst->fmts[OUTPUT_PORT];
	else
		return -EINVAL;

	f->fmt.pix_mp.pixelformat = fmt->fourcc;
	f->fmt.pix_mp.num_planes = fmt->num_planes;

	if (inst->in_reconfig) {
		if (vidc_comm_get_stream_output_mode(inst) ==
		    HAL_VIDEO_DECODER_PRIMARY) {
			inst->prop.height_cap = inst->reconfig_height;
			inst->prop.width_cap = inst->reconfig_width;
			inst->prop.height_out = inst->reconfig_height;
			inst->prop.width_out = inst->reconfig_width;
		} else {
			inst->prop.height_out = inst->reconfig_height;
			inst->prop.width_out = inst->reconfig_width;
		}

		ret = vidc_check_session_supported(inst);
		if (ret) {
			dev_err(dev, "%s: unsupported session\n", __func__);
			goto exit;
		}
	}

	f->fmt.pix_mp.height = inst->prop.height_cap;
	f->fmt.pix_mp.width = inst->prop.width_cap;
	stride = inst->prop.width_cap;
	scanlines = inst->prop.height_cap;

	ret = vidc_comm_get_bufreqs(inst);
	if (ret) {
		dev_err(dev, "%s: buffer requirements failed\n", __func__);
		goto exit;
	}

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		plane_sizes = &inst->bufq_out.plane_sizes[0];
		for (i = 0; i < fmt->num_planes; ++i) {
			if (!plane_sizes[i]) {
				f->fmt.pix_mp.plane_fmt[i].sizeimage =
					get_frame_size(inst, fmt, f->type, i);
				plane_sizes[i] =
					f->fmt.pix_mp.plane_fmt[i].sizeimage;
			} else {
				f->fmt.pix_mp.plane_fmt[i].sizeimage =
					plane_sizes[i];
			}
		}
	} else {
		switch (fmt->fourcc) {
		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_NV21:
			call_hfi_op(hdev, get_stride_scanline, COLOR_FMT_NV12,
				    inst->prop.width_cap, inst->prop.height_cap,
				    &stride, &scanlines);
			break;
		default:
			dev_warn(dev, "color format not recognized\n");
			break;
		}

		bufreq = get_buff_req_buffer(inst,
					vidc_comm_get_hal_output_buffer(inst));
		f->fmt.pix_mp.plane_fmt[0].sizeimage =
					bufreq ? bufreq->size : 0;

		extra_idx = EXTRADATA_IDX(fmt->num_planes);
		if (extra_idx && extra_idx < VIDEO_MAX_PLANES) {
			bufreq = get_buff_req_buffer(inst,
						HAL_BUFFER_EXTRADATA_OUTPUT);
			f->fmt.pix_mp.plane_fmt[extra_idx].sizeimage =
					bufreq ? bufreq->size : 0;
		}

		for (i = 0; i < fmt->num_planes; ++i)
			inst->bufq_cap.plane_sizes[i] =
					f->fmt.pix_mp.plane_fmt[i].sizeimage;
	}

	if (stride && scanlines) {
		f->fmt.pix_mp.plane_fmt[0].bytesperline = (__u16)stride;
		f->fmt.pix_mp.plane_fmt[0].reserved[0] = (__u16)scanlines;
	} else {
		f->fmt.pix_mp.plane_fmt[0].bytesperline =
			(__u16)inst->prop.width_cap;
		f->fmt.pix_mp.plane_fmt[0].reserved[0] =
			(__u16)inst->prop.height_cap;
	}

	if (vidc_comm_get_stream_output_mode(inst) ==
	    HAL_VIDEO_DECODER_SECONDARY) {
		if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			f->fmt.pix_mp.height = inst->prop.height_cap;
			f->fmt.pix_mp.width = inst->prop.width_cap;
		} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
			f->fmt.pix_mp.height = inst->prop.height_out;
			f->fmt.pix_mp.width = inst->prop.width_out;
			f->fmt.pix_mp.plane_fmt[0].bytesperline =
				(__u16)inst->prop.width_out;
			f->fmt.pix_mp.plane_fmt[0].reserved[0] =
				(__u16)inst->prop.height_out;
		}
	}
exit:
	return ret;
}

static int vdec_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct device *dev = &inst->core->res.pdev->dev;
	u64 us_per_frame = 0;
	int ret = 0, fps = 0;

	if (a->parm.output.timeperframe.denominator) {
		switch (a->type) {
		case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
			us_per_frame = a->parm.output.timeperframe.numerator *
				(u64)USEC_PER_SEC;
			do_div(us_per_frame,
			       a->parm.output.timeperframe.denominator);
			break;
		default:
			dev_err(dev, "scale clocks: nnknown buffer type %d\n",
				a->type);
			break;
		}
	}

	if (!us_per_frame) {
		dev_err(dev, "scale clocks: time between frames is 0\n");
		ret = -EINVAL;
		goto exit;
	}

	fps = USEC_PER_SEC;
	do_div(fps, us_per_frame);

	if (fps % 15 == 14 || fps % 24 == 23)
		fps = fps + 1;
	else if (fps % 24 == 1 || fps % 15 == 1)
		fps = fps - 1;

	if (inst->prop.fps != fps) {
		dev_dbg(dev, "reported fps changed for %p: %d->%d\n",
			inst, inst->prop.fps, fps);
		inst->prop.fps = fps;
		msm_comm_scale_clocks(inst->core);
	}
exit:
	return ret;
}

static int vdec_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	return -ENOTSUPP;
}

static int vdec_enum_framesizes(struct file *file, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vidc_core_capability *capability;

	capability = &inst->capability;
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = capability->width.min;
	fsize->stepwise.max_width = capability->width.max;
	fsize->stepwise.step_width = capability->width.step_size;
	fsize->stepwise.min_height = capability->height.min;
	fsize->stepwise.max_height = capability->height.max;
	fsize->stepwise.step_height = capability->height.step_size;

	return 0;
}

static int vdec_set_buffer_size(struct vidc_inst *inst, u32 buffer_size,
				enum hal_buffer buffer_type)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct hal_buffer_size_actual buffer_size_actual;
	struct hfi_device *hdev = inst->core->hfidev;
	int ret;

	dev_dbg(dev, "set actual buffer_size: %d for buffer type %d to fw\n",
		buffer_size, buffer_type);

	buffer_size_actual.buffer_type = buffer_type;
	buffer_size_actual.buffer_size = buffer_size;

	ret = call_hfi_op(hdev, session_set_property, inst->session,
			  HAL_PARAM_BUFFER_SIZE_ACTUAL, &buffer_size_actual);
	if (ret) {
		dev_err(dev, "%s: failed to set actual buffer size %u\n",
			__func__, buffer_size);
		return ret;
	}

	return 0;
}

static int vdec_update_out_buf_size(struct vidc_inst *inst,
				    struct v4l2_format *f, int num_planes)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct hal_buffer_requirements *bufreq;
	int ret = 0, i;

	/*
	 * Compare set buffer size and update to firmware if it's bigger
	 * then firmware returned buffer size.
	 */
	for (i = 0; i < num_planes; ++i) {
		enum hal_buffer type = vidc_comm_get_hal_output_buffer(inst);

		if (EXTRADATA_IDX(num_planes) &&
		    i == EXTRADATA_IDX(num_planes))
			type = HAL_BUFFER_EXTRADATA_OUTPUT;

		bufreq = get_buff_req_buffer(inst, type);
		if (!bufreq)
			goto exit;

		if (f->fmt.pix_mp.plane_fmt[i].sizeimage > bufreq->size) {
			ret = vdec_set_buffer_size(inst,
				f->fmt.pix_mp.plane_fmt[i].sizeimage, type);
			if (ret)
				goto exit;
		}
	}

	/* Query buffer requirements from firmware */
	ret = vidc_comm_get_bufreqs(inst);
	if (ret) {
		dev_err(dev, "Failed to get buf req, %d\n", ret);
		return ret;
	}

	/* Read back updated firmware size */
	for (i = 0; i < num_planes; ++i) {
		enum hal_buffer type = vidc_comm_get_hal_output_buffer(inst);

		if (EXTRADATA_IDX(num_planes) &&
		    i == EXTRADATA_IDX(num_planes))
			type = HAL_BUFFER_EXTRADATA_OUTPUT;

		bufreq = get_buff_req_buffer(inst, type);
		f->fmt.pix_mp.plane_fmt[i].sizeimage = bufreq ?
					bufreq->size : 0;
		dev_dbg(dev, "updated buffer size for plane[%d] = %d\n",
			i, f->fmt.pix_mp.plane_fmt[i].sizeimage);
	}

exit:
	return ret;
}

static int vdec_set_default_properties(struct vidc_inst *inst)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct hfi_device *hdev = inst->core->hfidev;
	struct v4l2_control ctrl = {0};
	enum hal_default_properties defaults;
	int ret = 0;

	defaults = call_hfi_op(hdev, get_default_properties,
			       hdev->hfi_device_data);
	if (defaults < 0)
		return defaults;

	if (defaults & HAL_VIDEO_DYNAMIC_BUF_MODE) {
		dev_dbg(dev, "Enable dynamic buffer mode\n");
		ctrl.id = V4L2_CID_MPEG_VIDC_VIDEO_ALLOC_MODE_OUTPUT;
		ctrl.value = V4L2_MPEG_VIDC_VIDEO_DYNAMIC;

		ret = v4l2_s_ctrl(NULL, &inst->ctrl_handler, &ctrl);
		if (ret)
			dev_err(dev, "set alloc_mode failed\n");
	}

	if (defaults & HAL_VIDEO_CONTINUE_DATA_TRANSFER) {
		dev_dbg(dev, "Enable continue_data_transfer\n");
		ctrl.id = V4L2_CID_MPEG_VIDC_VIDEO_CONTINUE_DATA_TRANSFER;
		ctrl.value = true;

		ret = v4l2_s_ctrl(NULL, &inst->ctrl_handler, &ctrl);
		if (ret)
			dev_err(dev, "set cont_data_transfer failed\n");
	}

	return ret;
}

static int vdec_s_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct device *dev = &inst->core->res.pdev->dev;
	struct hal_frame_size frame_sz;
	const struct vidc_format *fmt;
	u32 pixelformat = f->fmt.pix_mp.pixelformat;
	int ret, i;

	dev_dbg(dev, "%s: enter\n", __func__);

	fmt = find_format(pixelformat, CAPTURE_PORT);
	if (!fmt)
		return -EINVAL;

	inst->prop.width_cap = f->fmt.pix_mp.width;
	inst->prop.height_cap = f->fmt.pix_mp.height;

	if (vidc_comm_get_stream_output_mode(inst) ==
	    HAL_VIDEO_DECODER_PRIMARY) {
		inst->prop.width_out = f->fmt.pix_mp.width;
		inst->prop.height_out = f->fmt.pix_mp.height;
		ret = vidc_comm_set_color_format(inst, HAL_BUFFER_OUTPUT,
						 pixelformat);
		if (ret)
			return ret;
	}

	inst->fmts[fmt->type] = fmt;

	if (vidc_comm_get_stream_output_mode(inst) ==
	    HAL_VIDEO_DECODER_SECONDARY) {
		frame_sz.buffer_type = HAL_BUFFER_OUTPUT2;
		frame_sz.width = inst->prop.width_cap;
		frame_sz.height = inst->prop.height_cap;
		vidc_comm_set_color_format(inst, HAL_BUFFER_OUTPUT,
					   pixelformat);
		vidc_comm_set_color_format(inst, HAL_BUFFER_OUTPUT2,
					   pixelformat);
		dev_dbg(dev, "buffer type:%d, width:%d, height:%d\n",
			frame_sz.buffer_type, frame_sz.width,
			frame_sz.height);
		ret = hfi_session_set_property(inst, HAL_PARAM_FRAME_SIZE,
					       &frame_sz);
		if (ret)
			return ret;
	}

	ret = vidc_comm_get_bufreqs(inst);
	if (ret) {
		for (i = 0; i < fmt->num_planes; ++i) {
			f->fmt.pix_mp.plane_fmt[i].sizeimage =
				get_frame_size(inst, fmt, f->type, i);
		}
	} else {
		ret = vdec_update_out_buf_size(inst, f, fmt->num_planes);
		if (ret) {
			dev_err(dev, "%s: failed to update buffer size: %d\n",
				__func__, ret);
			return ret;
		}
	}

	f->fmt.pix_mp.num_planes = fmt->num_planes;

	for (i = 0; i < fmt->num_planes; ++i)
		inst->bufq_cap.plane_sizes[i] =
			f->fmt.pix_mp.plane_fmt[i].sizeimage;

	return 0;
}

static int vdec_s_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct device *dev = &inst->core->res.pdev->dev;
	struct hal_frame_size frame_sz;
	const struct vidc_format *fmt;
	u32 pixelformat = f->fmt.pix_mp.pixelformat;
	int max_input_size = 0;
	int ret, i;

	dev_dbg(dev, "%s: enter\n", __func__);

	inst->prop.width_out = f->fmt.pix_mp.width;
	inst->prop.height_out = f->fmt.pix_mp.height;

	if (vidc_comm_get_stream_output_mode(inst) ==
	    HAL_VIDEO_DECODER_PRIMARY) {
		inst->prop.width_cap = f->fmt.pix_mp.width;
		inst->prop.height_cap = f->fmt.pix_mp.height;
	}

	fmt = find_format(pixelformat, OUTPUT_PORT);
	if (!fmt) {
		dev_err(dev, "Format: %d not supported on OUTPUT port\n",
			pixelformat);
		return -EINVAL;
	}

	if (!(vidc_comm_hal_codec_type(fmt->fourcc) & inst->core->dec_codecs)) {
		dev_err(dev, "codec (%x) is not supported\n", fmt->fourcc);
		return -EINVAL;
	}

	inst->fmts[fmt->type] = fmt;

	ret = hfi_session_init(inst);
	if (ret)
		return ret;

	ret = vidc_check_session_supported(inst);
	if (ret) {
		dev_err(dev, "%s: session not supported\n", __func__);
		goto err;
	}

	frame_sz.buffer_type = HAL_BUFFER_INPUT;
	frame_sz.width = inst->prop.width_out;
	frame_sz.height = inst->prop.height_out;

	dev_dbg(dev, "buffer type = %d width = %d, height = %d\n",
		frame_sz.buffer_type, frame_sz.width, frame_sz.height);

	ret = hfi_session_set_property(inst, HAL_PARAM_FRAME_SIZE, &frame_sz);
	if (ret)
		goto err;

	max_input_size = get_frame_size(inst, fmt, f->type, 0);
	if (f->fmt.pix_mp.plane_fmt[0].sizeimage > max_input_size ||
	    !f->fmt.pix_mp.plane_fmt[0].sizeimage)
		f->fmt.pix_mp.plane_fmt[0].sizeimage = max_input_size;

	f->fmt.pix_mp.num_planes = fmt->num_planes;
	for (i = 0; i < fmt->num_planes; ++i) {
		inst->bufq_out.plane_sizes[i] =
				f->fmt.pix_mp.plane_fmt[i].sizeimage;
	}

	ret = vdec_set_default_properties(inst);
	if (ret)
		goto err;

	return 0;

err:
	hfi_session_deinit(inst);
	return ret;
}

static int
vdec_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strlcpy(cap->driver, VIDC_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->card, "video decoder", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform", sizeof(cap->bus_info));
	cap->version = VIDC_VERSION;

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
			    V4L2_CAP_VIDEO_OUTPUT_MPLANE |
			    V4L2_CAP_STREAMING |
			    V4L2_CAP_DEVICE_CAPS;
	cap->device_caps |= V4L2_CAP_EXT_PIX_FORMAT;

	memset(cap->reserved, 0, sizeof(cap->reserved));

	return 0;
}

static int
vdec_enum_fmt_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	const struct vidc_format *fmt;

	fmt = find_format_by_index(f->index, CAPTURE_PORT);

	memset(f->reserved, 0 , sizeof(f->reserved));

	/* no more formats */
	if (!fmt)
		return -EINVAL;

	strlcpy(f->description, fmt->description, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

	return 0;
}

static int
vdec_enum_fmt_out(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	const struct vidc_format *fmt;

	fmt = find_format_by_index(f->index, OUTPUT_PORT);

	f->flags = V4L2_FMT_FLAG_COMPRESSED;

	memset(f->reserved, 0 , sizeof(f->reserved));

	/* no more formats */
	if (!fmt)
		return -EINVAL;

	strlcpy(f->description, fmt->description, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

	return 0;
}

static int vdec_set_display_hold_count(struct vidc_inst *inst,
				       unsigned int display_count)
{
	struct hal_buffer_display_hold_count_actual display;
	struct device *dev = &inst->core->res.pdev->dev;
	struct hfi_device *hdev = inst->core->hfidev;
	int ret;

	display.buffer_type = vidc_comm_get_hal_output_buffer(inst);
	display.hold_count = display_count;

	ret = call_hfi_op(hdev, session_set_property, inst->session,
			  HAL_PARAM_BUFFER_DISPLAY_HOLD_COUNT_ACTUAL, &display);
	if (ret) {
		dev_err(dev, "failed to set display hold count %d\n", ret);
		return ret;
	}

	return 0;
}

static int vdec_queue_setup(struct vb2_queue *q, const struct v4l2_format *fmt,
			    unsigned int *num_buffers,
			    unsigned int *num_planes, unsigned int sizes[],
			    void *alloc_ctxs[])
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct device *dev = &inst->core->res.pdev->dev;
	struct hfi_device *hdev = inst->core->hfidev;
	struct hal_buffer_requirements *bufreq;
	struct hal_buffer_count_actual new_buf_count;
	enum hal_property property_id;
	int i, ret = 0;

	switch (q->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		*num_planes = inst->fmts[OUTPUT_PORT]->num_planes;

		if (*num_buffers < MIN_NUM_OUTPUT_BUFFERS ||
		    *num_buffers > MAX_NUM_OUTPUT_BUFFERS)
			*num_buffers = MIN_NUM_OUTPUT_BUFFERS;

		for (i = 0; i < *num_planes; i++) {
			sizes[i] = get_frame_size(inst, inst->fmts[OUTPUT_PORT],
						  q->type, i);
			alloc_ctxs[i] = inst->vb2_ctx_out;
		}

		property_id = HAL_PARAM_BUFFER_COUNT_ACTUAL;
		new_buf_count.buffer_type = HAL_BUFFER_INPUT;
		new_buf_count.buffer_count_actual = *num_buffers;

		ret = call_hfi_op(hdev, session_set_property,
				  inst->session, property_id, &new_buf_count);
		if (ret) {
			dev_err(dev, "set buffer count %d failed (%d)\n",
				new_buf_count.buffer_count_actual, ret);
		}
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		*num_planes = inst->fmts[CAPTURE_PORT]->num_planes;

		ret = vidc_comm_get_bufreqs(inst);
		if (ret) {
			dev_err(dev, "get buffer requirements failed (%d)\n",
				ret);
			break;
		}

		bufreq = get_buff_req_buffer(inst,
					vidc_comm_get_hal_output_buffer(inst));
		if (!bufreq) {
			dev_err(dev, "cannot get buffer requirements\n");
			ret = -EINVAL;
			break;
		}

		*num_buffers = max(*num_buffers, bufreq->count_min);

		if (*num_buffers != bufreq->count_actual) {
			property_id = HAL_PARAM_BUFFER_COUNT_ACTUAL;
			new_buf_count.buffer_type =
				vidc_comm_get_hal_output_buffer(inst);
			new_buf_count.buffer_count_actual = *num_buffers;
			ret = call_hfi_op(hdev, session_set_property,
					  inst->session, property_id,
					  &new_buf_count);
			if (ret) {
				dev_err(dev, "set buf count failed (%d)", ret);
				break;
			}

			ret = vdec_set_display_hold_count(inst,
					*num_buffers - bufreq->count_actual);
			if (ret) {
				dev_err(dev, "set display hold count failed (%d)",
					ret);
				break;
			}
		}

		if (*num_buffers != bufreq->count_actual) {
			ret = vidc_comm_get_bufreqs(inst);
			if (ret) {
				dev_err(dev, "failed to get buf req, %d\n",
					ret);
				break;
			}
		}

		dev_dbg(dev, "count =  %d, size = %d, alignment = %d\n",
			inst->buff_req.buffer[1].count_actual,
			inst->buff_req.buffer[1].size,
			inst->buff_req.buffer[1].alignment);

		sizes[0] = bufreq->size;
		alloc_ctxs[0] = inst->vb2_ctx_cap;

		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int vdec_buf_init(struct vb2_buffer *vb)
{
	struct vb2_queue *q = vb->vb2_queue;
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct device *dev = &inst->core->res.pdev->dev;
	struct hfi_device *hdev = inst->core->hfidev;
	struct vdec_buffer *buf = to_vdec_buffer(vb);
	struct vidc_buffer_addr_info *bai;
	struct buffer_info *bi;
	int ret;

	dev_dbg(dev, "buf_init %p (length: %ld)\n", vb, vb2_plane_size(vb, 0));

	bi = &buf->bi;
	bai = &bi->bai;

	memset(bai, 0, sizeof(*bai));

	if (q->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return 0;

	bai->buffer_size = vb2_plane_size(vb, 0);
	bai->buffer_type = vidc_comm_get_hal_output_buffer(inst);
	bai->num_buffers = 1;
	bai->align_device_addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	dev_dbg(dev, "%s: session: set buffer: vb: %p (size:%d, addr:%pa)\n",
		__func__, vb, bai->buffer_size, &bai->align_device_addr);

	ret = call_hfi_op(hdev, session_set_buffers, inst->session, bai);
	if (ret) {
		dev_err(dev, "%s: session: set buffer failed\n", __func__);
		return ret;
	}

	mutex_lock(&inst->registeredbufs.lock);
	list_add_tail(&bi->list, &inst->registeredbufs.list);
	mutex_unlock(&inst->registeredbufs.lock);

	return 0;
}

static int vdec_buf_prepare(struct vb2_buffer *vb)
{
	struct vdec_buffer *buf = to_vdec_buffer(vb);

	buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	return 0;
}

static void __fill_flags(struct vidc_frame_data *frame_data, __u32 vb_flags)
{
	u32 *flags = &frame_data->flags;

	if (vb_flags & V4L2_QCOM_BUF_FLAG_EOS)
		*flags = HAL_BUFFERFLAG_EOS;

	if (vb_flags & V4L2_MSM_BUF_FLAG_YUV_601_709_CLAMP)
		*flags |= HAL_BUFFERFLAG_YUV_601_709_CSC_CLAMP;

	if (vb_flags & V4L2_QCOM_BUF_FLAG_CODECCONFIG)
		*flags |= HAL_BUFFERFLAG_CODECCONFIG;

	if (vb_flags & V4L2_QCOM_BUF_FLAG_DECODEONLY)
		*flags |= HAL_BUFFERFLAG_DECODEONLY;

	if (vb_flags & V4L2_QCOM_BUF_TS_DISCONTINUITY)
		*flags |= HAL_BUFFERFLAG_TS_DISCONTINUITY;

	if (vb_flags & V4L2_QCOM_BUF_TS_ERROR)
		*flags |= HAL_BUFFERFLAG_TS_ERROR;

	if (vb_flags & V4L2_QCOM_BUF_TIMESTAMP_INVALID)
		frame_data->timestamp = LLONG_MAX;
}

static int vdec_set_session_buf(struct vb2_buffer *vb)
{
	struct vb2_queue *q = vb->vb2_queue;
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct vidc_core *core = inst->core;
	struct device *dev = &core->res.pdev->dev;
	struct hfi_device *hdev = core->hfidev;
	struct vdec_buffer *buf = to_vdec_buffer(vb);
	struct vidc_frame_data fdata;
	int64_t time_usec;
	int ret;

	if (inst->state == INST_INVALID || core->state == CORE_INVALID) {
		dev_err(dev, "core id:%d is in bad state\n", core->id);
		return -EINVAL;
	}

	dev_dbg(dev, "%s: vb:%p, type:%d, addr:%pa\n", __func__, vb,
		vb->vb2_queue->type, &buf->dma_addr);

	time_usec = timeval_to_ns(&vb->v4l2_buf.timestamp);
	do_div(time_usec, NSEC_PER_USEC);

	memset(&fdata, 0 , sizeof(fdata));

	fdata.alloc_len = vb2_plane_size(vb, 0);
	fdata.device_addr = buf->dma_addr;
	fdata.timestamp = time_usec;
	fdata.flags = 0;
	fdata.clnt_data = buf->dma_addr;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		fdata.buffer_type = HAL_BUFFER_INPUT;
		fdata.filled_len = vb2_get_plane_payload(vb, 0);
		fdata.offset = vb->v4l2_planes[0].data_offset;

		__fill_flags(&fdata, vb->v4l2_buf.flags);

		ret = call_hfi_op(hdev, session_etb, inst->session, &fdata);
	} else if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fdata.buffer_type = vidc_comm_get_hal_output_buffer(inst);
		fdata.filled_len = 0;
		fdata.offset = 0;

		ret = call_hfi_op(hdev, session_ftb, inst->session, &fdata);
	} else {
		ret = -EINVAL;
	}

	if (ret) {
		dev_err(dev, "failed to set session buffer\n");
		return ret;
	}

	return 0;
}

static int vdec_rel_session_bufs(struct vidc_inst *inst)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct vidc_buffer_addr_info *bai;
	struct buffer_info *bi, *tmp;
	int ret = 0;

	mutex_lock(&inst->registeredbufs.lock);
	list_for_each_entry_safe(bi, tmp, &inst->registeredbufs.list, list) {
		list_del(&bi->list);
		bai = &bi->bai;
		bai->response_required = 1;
		ret = hfi_session_release_buffers(inst, bai);
		if (ret) {
			dev_err(dev, "%s: session release buffers failed\n",
				__func__);
			break;
		}
	}
	mutex_unlock(&inst->registeredbufs.lock);

	return ret;
}

static int start_streaming(struct vidc_inst *inst)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct list_head *ptr, *next;
	struct vdec_buffer *buf;
	int ret;

	inst->in_reconfig = false;

	ret = set_scratch_buffers(inst);
	if (ret) {
		dev_err(dev, "failed to set scratch buffers: %d\n", ret);
		return ret;
	}

	ret = set_persist_buffers(inst);
	if (ret) {
		dev_err(dev, "Failed to set persist buffers: %d\n", ret);
		return ret;
	}

	msm_comm_scale_clocks(inst->core);

	ret = hfi_session_load_res(inst);
	if (ret)
		return ret;

	ret = hfi_session_start(inst);
	if (ret)
		return ret;

	mutex_lock(&inst->bufqueue_lock);
	list_for_each_safe(ptr, next, &inst->bufqueue) {
		buf = list_entry(ptr, struct vdec_buffer, list);

		ret = vdec_set_session_buf(&buf->vb);
		if (ret)
			break;
	}
	mutex_unlock(&inst->bufqueue_lock);

	return ret;
}

static int vdec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct device *dev = &inst->core->res.pdev->dev;
	struct vb2_queue *queue;

	dev_dbg(dev, "%s: type: %d, count: %d\n", __func__, q->type, count);

	switch (q->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		queue = &inst->bufq_cap;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		queue = &inst->bufq_out;
		break;
	default:
		return -EINVAL;
	}

	if (vb2_is_streaming(queue))
		return start_streaming(inst);

	return 0;
}

static int stop_streaming(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	struct device *dev = &core->res.pdev->dev;
	int ret, streamoff;

	mutex_lock(&inst->lock);
	streamoff = inst->streamoff;
	mutex_unlock(&inst->lock);

	if (streamoff)
		return 0;

	ret = hfi_session_stop(inst);
	if (ret) {
		dev_err(dev, "session: stop failed\n");
		goto abort;
	}

	ret = hfi_session_release_res(inst);
	if (ret) {
		dev_err(dev, "session: release resources failed\n");
		goto abort;
	}

	ret = vdec_rel_session_bufs(inst);
	if (ret) {
		dev_err(dev, "failed to release capture buffers: %d\n", ret);
		goto abort;
	}

	ret = release_scratch_buffers(inst, false);
	if (ret) {
		dev_err(dev, "failed to release scratch buffers: %d\n", ret);
		goto abort;
	}

	ret = release_persist_buffers(inst);
	if (ret) {
		dev_err(dev, "failed to release persist buffers: %d\n", ret);
		goto abort;
	}

	if (inst->state == INST_INVALID || core->state == CORE_INVALID) {
		dev_err(dev, "core id:%d is in invalid state\n", core->id);
		ret = -EINVAL;
		goto abort;
	}

	ret = hfi_session_deinit(inst);

	mutex_lock(&inst->lock);
	inst->streamoff = 1;
	mutex_unlock(&inst->lock);

abort:
	if (ret)
		hfi_session_abort(inst);

	return ret;
}

static void vdec_stop_streaming(struct vb2_queue *q)
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct device *dev = &inst->core->res.pdev->dev;
	int ret;

	dev_dbg(dev, "%s: type: %d\n", __func__, q->type);

	ret = stop_streaming(inst);

	msm_comm_scale_clocks(inst->core);

	if (ret)
		dev_err(dev, "stop streaming failed type: %d, ret: %d\n",
			q->type, ret);
}

static void vdec_buf_queue(struct vb2_buffer *vb)
{
	struct vidc_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vidc_core *core = inst->core;
	struct device *dev = &core->res.pdev->dev;
	struct vdec_buffer *buf = to_vdec_buffer(vb);
	int ret;

	if (inst->state == INST_INVALID || core->state == CORE_INVALID) {
		dev_err(dev, "core or instance are in invalid state\n");
		return;
	}

	dev_dbg(dev, "%s: vb:%p, type:%d, addr:%pa\n", __func__, vb,
		vb->vb2_queue->type, &buf->dma_addr);

	mutex_lock(&inst->bufqueue_lock);
	list_add_tail(&buf->list, &inst->bufqueue);
	mutex_unlock(&inst->bufqueue_lock);

	if (!vb2_is_streaming(&inst->bufq_cap) ||
	    !vb2_is_streaming(&inst->bufq_out))
		return;

	ret = vdec_set_session_buf(vb);
	if (ret)
		dev_err(dev, "cannot set session buffer\n");

	return;
}

static const struct vb2_ops vdec_vb2_ops = {
	.queue_setup = vdec_queue_setup,
	.buf_init = vdec_buf_init,
	.buf_prepare = vdec_buf_prepare,
	.start_streaming = vdec_start_streaming,
	.stop_streaming = vdec_stop_streaming,
	.buf_queue = vdec_buf_queue,
};

static struct vb2_buffer *
vdec_get_vb2buffer(struct vidc_inst *inst, dma_addr_t addr)
{
	struct vdec_buffer *buf;
	struct vb2_buffer *vb = NULL;

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

static int vdec_empty_buf_done(struct vidc_inst *inst, u32 addr,
			       u32 bytesused, u32 data_offset, u32 flags)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct vb2_buffer *buf;

	buf = vdec_get_vb2buffer(inst, addr);
	if (!buf)
		return -EFAULT;

	buf->v4l2_planes[0].bytesused = bytesused;
	buf->v4l2_planes[0].data_offset = data_offset;
	buf->v4l2_buf.flags = flags;

	if (buf->v4l2_planes[0].data_offset > buf->v4l2_planes[0].length)
		dev_dbg(dev, "data_offset overflow length\n");

	if (buf->v4l2_planes[0].bytesused > buf->v4l2_planes[0].length)
		dev_dbg(dev, "bytesused overflow length\n");

	if (flags & V4L2_QCOM_BUF_INPUT_UNSUPPORTED)
		dev_dbg(dev, "unsupported input stream\n");

	if (flags & V4L2_QCOM_BUF_DATA_CORRUPT)
		dev_dbg(dev, "corrupted input stream\n");

	if (flags & V4L2_MSM_VIDC_BUF_START_CODE_NOT_FOUND)
		dev_dbg(dev, "start code not found\n");

	inst->count.ebd++;

	vb2_buffer_done(buf, VB2_BUF_STATE_DONE);

	return 0;
}

static int vdec_fill_buf_done(struct vidc_inst *inst, u32 addr,
			      u32 bytesused, u32 data_offset, u32 flags,
			      struct timeval *timestamp)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct vb2_buffer *buf;

	buf = vdec_get_vb2buffer(inst, addr);
	if (!buf)
		return -EFAULT;

	buf->v4l2_planes[0].bytesused = bytesused;
	buf->v4l2_planes[0].data_offset = data_offset;
	buf->v4l2_buf.flags = flags;
	buf->v4l2_buf.timestamp = *timestamp;

	if (buf->v4l2_planes[0].data_offset > buf->v4l2_planes[0].length)
		dev_warn(dev, "overflow data_offset:%d, length:%d\n",
			 buf->v4l2_planes[0].data_offset,
			 buf->v4l2_planes[0].length);

	if (buf->v4l2_planes[0].bytesused > buf->v4l2_planes[0].length)
		dev_warn(dev, "overflow bytesused:%d, length:%d\n",
			 buf->v4l2_planes[0].bytesused,
			 buf->v4l2_planes[0].length);

	inst->count.fbd++;

	vb2_buffer_done(buf, VB2_BUF_STATE_DONE);

	return 0;
}

static int vdec_event_notify(struct vidc_inst *inst, u32 event,
			     struct vidc_cb_event *data)
{
	switch (event) {
	case SESSION_ERROR:
		/* the instance lock is already taken just change the state */
		inst->state = INST_INVALID;
		break;
	case SYS_EVENT_CHANGE:
		switch (data->hal_event_type) {
		case HAL_EVENT_SEQ_CHANGED_SUFFICIENT_RESOURCES:
			break;
		case HAL_EVENT_SEQ_CHANGED_INSUFFICIENT_RESOURCES:
			inst->reconfig_height = data->height;
			inst->reconfig_width = data->width;
			inst->in_reconfig = true;
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

static int vdec_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *dec)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vidc_core *core = inst->core;
	int ret = 0;

	return 0;

	switch (dec->cmd) {
	case V4L2_DEC_QCOM_CMD_FLUSH:
		if (core->state != CORE_INVALID &&
		    inst->state == INST_INVALID) {
			ret = vidc_comm_kill_session(inst);
			if (ret)
				dprintk(VIDC_ERR, "session kill (%d)\n", ret);
		}

		ret = vidc_comm_session_flush(inst, dec->flags);
		if (ret)
			dprintk(VIDC_ERR, "Failed to flush buffers: %d\n", ret);
		break;
	case V4L2_DEC_CMD_STOP:
		if (core->state != CORE_INVALID &&
		    inst->state == INST_INVALID) {
			ret = vidc_comm_kill_session(inst);
			if (ret)
				dprintk(VIDC_ERR, "session kill (%d)\n", ret);
		}

		ret = release_scratch_buffers(inst, false);
		if (ret)
			dprintk(VIDC_ERR,
				"release scratch buffers (%d)\n", ret);

		ret = release_persist_buffers(inst);
		if (ret)
			dprintk(VIDC_ERR,
				"release persist buffers (%d)\n", ret);

		if (inst->state == INST_INVALID ||
		    core->state == CORE_INVALID) {
			dprintk(VIDC_ERR,
				"core and/or instance are in invalid state\n");
			vidc_queue_v4l2_event(inst,
					      V4L2_EVENT_MSM_VIDC_CLOSE_DONE);
			goto exit;
		}

		ret = hfi_session_deinit(inst);

		/* Clients rely on this event for joining poll thread.
		 * This event should be returned even if firmware has
		 * failed to respond
		 */
		vidc_queue_v4l2_event(inst, V4L2_EVENT_MSM_VIDC_CLOSE_DONE);
		break;
	default:
		return -ENOTSUPP;
	}

exit:
	return ret;
}

static void vdec_inst_init(struct vidc_inst *inst)
{
	inst->fmts[OUTPUT_PORT] = &vdec_formats[2];
	inst->fmts[CAPTURE_PORT] = &vdec_formats[0];
	inst->prop.height_cap = DEFAULT_HEIGHT;
	inst->prop.width_cap = DEFAULT_WIDTH;
	inst->prop.height_out = DEFAULT_HEIGHT;
	inst->prop.width_out = DEFAULT_WIDTH;
	inst->capability.height.min = MIN_SUPPORTED_HEIGHT;
	inst->capability.height.max = DEFAULT_HEIGHT;
	inst->capability.width.min = MIN_SUPPORTED_WIDTH;
	inst->capability.width.max = DEFAULT_WIDTH;
	inst->capability.buffer_mode[OUTPUT_PORT] = HAL_BUFFER_MODE_STATIC;
	inst->capability.buffer_mode[CAPTURE_PORT] = HAL_BUFFER_MODE_STATIC;
	inst->capability.secure_output2_threshold.min = 0;
	inst->capability.secure_output2_threshold.max = 0;
	inst->buffer_mode_set[OUTPUT_PORT] = HAL_BUFFER_MODE_STATIC;
	inst->buffer_mode_set[CAPTURE_PORT] = HAL_BUFFER_MODE_STATIC;
	inst->prop.fps = 30;
}

static int vdec_subscribe_event(struct v4l2_fh *fh,
				const struct  v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ioctl_ops vdec_ioctl_ops = {
	.vidioc_querycap = vdec_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = vdec_enum_fmt_cap,
	.vidioc_enum_fmt_vid_out_mplane = vdec_enum_fmt_out,
	.vidioc_s_fmt_vid_cap_mplane = vdec_s_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane = vdec_s_fmt_out,
	.vidioc_g_fmt_vid_cap_mplane = vdec_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vdec_g_fmt,
	.vidioc_reqbufs = vdec_reqbufs,
	.vidioc_querybuf = vdec_querybuf,
	.vidioc_prepare_buf = vdec_prepare_buf,
	.vidioc_qbuf = vdec_qbuf,
	.vidioc_expbuf = vdec_exportbuf,
	.vidioc_dqbuf = vdec_dqbuf,
	.vidioc_streamon = vdec_streamon,
	.vidioc_streamoff = vdec_streamoff,
	.vidioc_s_ctrl = vdec_s_ctrl,
	.vidioc_g_ctrl = vdec_g_ctrl,
	.vidioc_s_parm = vdec_s_parm,
	.vidioc_g_parm = vdec_g_parm,
	.vidioc_enum_framesizes = vdec_enum_framesizes,
	.vidioc_subscribe_event = vdec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_decoder_cmd = vdec_cmd,
};

static void vdec_release_video_device(struct video_device *pvdev)
{
}

extern const struct v4l2_file_operations vidc_fops;

int vdec_init(struct vidc_core *core, struct video_device *dec)
{
	int ret;

	/* setup the decoder device */
	dec->release = vdec_release_video_device;
	dec->fops = &vidc_fops;
	dec->ioctl_ops = &vdec_ioctl_ops;
	dec->vfl_dir = VFL_DIR_M2M;
	dec->v4l2_dev = &core->v4l2_dev;

	ret = video_register_device(dec, VFL_TYPE_GRABBER, 32);
	if (ret) {
		dprintk(VIDC_ERR, "Failed to register video decoder device");
		return ret;
	}

	video_set_drvdata(dec, core);

	return 0;
}

void vdec_deinit(struct vidc_core *core, struct video_device *dec)
{
	video_unregister_device(dec);
}

int vdec_open(struct vidc_inst *inst)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct vb2_queue *q;
	struct device *iommu_dev;
	int ret;

	/* TODO: use iommu dt bindings when they are exist */
	iommu_dev = msm_iommu_get_ctx("venus_ns");
	if (IS_ERR(iommu_dev)) {
		dev_err(dev, "cannot find iommu nonsecure ctx\n");
		return PTR_ERR(iommu_dev);
	}

	inst->vb2_ctx_cap = vb2_dma_contig_init_ctx(iommu_dev);
	if (IS_ERR(inst->vb2_ctx_cap))
		return PTR_ERR(inst->vb2_ctx_cap);

	inst->vb2_ctx_out = vb2_dma_contig_init_ctx(iommu_dev);
	if (IS_ERR(inst->vb2_ctx_out)) {
		ret = PTR_ERR(inst->vb2_ctx_out);
		goto err_cleanup_cap;
	}

	if (inst->session_type == VIDC_DECODER) {
		vdec_inst_init(inst);
		ret = vdec_ctrl_init(inst);
		if (ret)
			goto err_cleanup_out;
	}

	q = &inst->bufq_cap;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->ops = &vdec_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->drv_priv = inst;
	q->buf_struct_size = sizeof(struct vdec_buffer);
	ret = vb2_queue_init(q);
	if (ret)
		goto err_ctrl_deinit;

	q = &inst->bufq_out;
	q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	q->io_modes = VB2_MMAP;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->ops = &vdec_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->drv_priv = inst;
	q->buf_struct_size = sizeof(struct vdec_buffer);
	ret = vb2_queue_init(q);
	if (ret)
		goto err_cap_queue_release;

	inst->empty_buf_done = vdec_empty_buf_done;
	inst->fill_buf_done = vdec_fill_buf_done;
	inst->event_notify = vdec_event_notify;

	return 0;

err_cap_queue_release:
	vb2_queue_release(&inst->bufq_cap);
err_ctrl_deinit:
	vdec_ctrl_deinit(inst);
err_cleanup_out:
	vb2_dma_contig_cleanup_ctx(inst->vb2_ctx_out);
err_cleanup_cap:
	vb2_dma_contig_cleanup_ctx(inst->vb2_ctx_cap);
	return ret;
}

void vdec_close(struct vidc_inst *inst)
{
	vb2_queue_release(&inst->bufq_out);
	vb2_queue_release(&inst->bufq_cap);

	vb2_dma_contig_cleanup_ctx(inst->vb2_ctx_out);
	vb2_dma_contig_cleanup_ctx(inst->vb2_ctx_cap);

	vdec_ctrl_deinit(inst);
}
