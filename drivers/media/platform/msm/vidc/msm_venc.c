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
#include <linux/qcom_iommu.h>
#include <media/videobuf2-dma-contig.h>

#include "msm_vidc_internal.h"
#include "msm_vidc_common.h"
#include "hfi/vidc_hfi_api.h"
#include "msm_vidc_debug.h"
#include "msm_internal-buffers.h"
#include "msm_vidc_load.h"
#include "msm_hfi_interface.h"
#include "msm_venc-ctrls.h"

#define MIN_NUM_OUTPUT_BUFFERS		4
#define MIN_NUM_CAPTURE_BUFFERS		4

/* Offset base for buffers on the destination queue - used to distinguish
 * between source and destination buffers when mmapping - they receive the same
 * offsets but for different queues */
#define DST_QUEUE_OFF_BASE	(1 << 30)

/*
 * Default 601 to 709 conversion coefficients for resolution: 176x144 negative
 * coeffs are converted to s4.9 format (e.g. -22 converted to ((1<<13) - 22)
 * 3x3 transformation matrix coefficients in s4.9 fixed point format
 */
static u32 vpe_csc_601_to_709_matrix_coeff[HAL_MAX_MATRIX_COEFFS] = {
	470, 8170, 8148, 0, 490, 50, 0, 34, 483
};

/* offset coefficients in s9 fixed point format */
static u32 vpe_csc_601_to_709_bias_coeff[HAL_MAX_BIAS_COEFFS] = {
	34, 0, 4
};

/* clamping value for Y/U/V([min,max] for Y/U/V) */
static u32 vpe_csc_601_to_709_limit_coeff[HAL_MAX_LIMIT_COEFFS] = {
	16, 235, 16, 240, 16, 240
};

struct venc_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t dma_addr;
	struct buffer_info bi;
};

#define to_venc_buffer(buf)	container_of(buf, struct venc_buffer, vb)

static inline struct vidc_inst *to_inst(struct file *filp, void *fh)
{
	return container_of(filp->private_data, struct vidc_inst, fh);
}

static u32 get_framesize_nv12(int plane, u32 height, u32 width)
{
	u32 y_stride, uv_stride, y_plane;
	u32 y_sclines, uv_sclines, uv_plane;
	u32 size;

	y_stride = ALIGN(width, 128);
	uv_stride = ALIGN(width, 128);
	y_sclines = ALIGN(height, 32);
	uv_sclines = ALIGN(((height + 1) >> 1), 16);

	y_plane = y_stride * y_sclines;
	uv_plane = uv_stride * uv_sclines + SZ_4K;
	size = y_plane + uv_plane + SZ_8K;
	size = ALIGN(size, SZ_4K);

	return size;
}

static u32 get_framesize_nv21(int plane, u32 height, u32 width)
{
	return get_framesize_nv12(plane, height, width);
}

static u32 get_framesize_compressed(int plane, u32 height, u32 width)
{
	u32 sz = ALIGN(height, 32) * ALIGN(width, 32) * 3 / 2;

	return ALIGN(sz, SZ_4K);
}

static const struct vidc_format venc_formats[] = {
	{
		.pixfmt = V4L2_PIX_FMT_NV12,
		.num_planes = 1,
		.get_framesize = get_framesize_nv12,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	},
	{
		.pixfmt = V4L2_PIX_FMT_NV21,
		.num_planes = 1,
		.get_framesize = get_framesize_nv21,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	},
	{
		.pixfmt = V4L2_PIX_FMT_MPEG4,
		.num_planes = 1,
		.get_framesize = get_framesize_compressed,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	},
	{
		.pixfmt = V4L2_PIX_FMT_H263,
		.num_planes = 1,
		.get_framesize = get_framesize_compressed,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	},
	{
		.pixfmt = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.get_framesize = get_framesize_compressed,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	},
	{
		.pixfmt = V4L2_PIX_FMT_VP8,
		.num_planes = 1,
		.get_framesize = get_framesize_compressed,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	},
};

static const struct vidc_format *find_format(u32 pixfmt, int type)
{
	const struct vidc_format *fmt = venc_formats;
	unsigned int size = ARRAY_SIZE(venc_formats);
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (fmt[i].pixfmt == pixfmt)
			break;
	}

	if (i == size || fmt[i].type != type)
		return NULL;

	return &fmt[i];
}

static const struct vidc_format *find_format_by_index(int index, int type)
{
	const struct vidc_format *fmt = venc_formats;
	unsigned int size = ARRAY_SIZE(venc_formats);
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
venc_to_vb2q(struct vidc_inst *inst, enum v4l2_buf_type type)
{
	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return &inst->bufq_cap;
	else if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return &inst->bufq_out;

	return NULL;
}

int venc_toggle_hier_p(struct vidc_inst *inst, int layers)
{
	int num_enh_layers = 0;
	u32 prop_id;
	int ret;

	if (inst->fmt_cap->pixfmt != V4L2_PIX_FMT_VP8 &&
	    inst->fmt_cap->pixfmt != V4L2_PIX_FMT_H264)
		return 0;

	num_enh_layers = layers ? : 0;

	dprintk(VIDC_DBG, "%s Hier-P in firmware\n",
		num_enh_layers ? "Enable" : "Disable");

	prop_id = HAL_PARAM_VENC_HIER_P_MAX_ENH_LAYERS;

	ret = hfi_session_set_property(inst, prop_id, &num_enh_layers);
	if (ret)
		return ret;

	return 0;
}

int venc_set_bitrate_for_each_layer(struct vidc_inst *inst, u32 num_enh_layers,
				    u32 total_bitrate)
{
	struct hal_bitrate bitrate;
	u32 bitrate_table[3][4] = {
		{50, 50, 0, 0},
		{34, 33, 33, 0},
		{25, 25, 25, 25}
	};
	u32 prop_id, i;
	int ret;

	if (!num_enh_layers || num_enh_layers > ARRAY_SIZE(bitrate_table))
		return -EINVAL;

	prop_id = HAL_CONFIG_VENC_TARGET_BITRATE;

	for (i = 0; !ret && i <= num_enh_layers; i++) {
		bitrate.bit_rate = (total_bitrate *
				bitrate_table[num_enh_layers - 1][i]) / 100;
		bitrate.layer_id = i;

		ret = hfi_session_set_property(inst, prop_id, &bitrate);
		if (ret)
			return ret;
	}

	return 0;
}

static int venc_set_csc(struct vidc_inst *inst)
{
	struct hal_vpe_color_space_conversion vpe_csc;
	enum hal_property prop_id;
	int ret, count = 0;

	if (!vidc_vpe_csc_601_to_709)
		return 0;

	while (count < HAL_MAX_MATRIX_COEFFS) {
		if (count < HAL_MAX_BIAS_COEFFS)
			vpe_csc.csc_bias[count] =
				vpe_csc_601_to_709_bias_coeff[count];
		if (count < HAL_MAX_LIMIT_COEFFS)
			vpe_csc.csc_limit[count] =
				vpe_csc_601_to_709_limit_coeff[count];
		vpe_csc.csc_matrix[count] =
			vpe_csc_601_to_709_matrix_coeff[count];
		count++;
	}

	prop_id = HAL_PARAM_VPE_COLOR_SPACE_CONVERSION;

	ret = hfi_session_set_property(inst, prop_id, &vpe_csc);
	if (ret) {
		dprintk(VIDC_ERR, "Setting VPE coefficients failed\n");
		return ret;
	}

	return 0;
}

static int venc_set_properties(struct vidc_inst *inst)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct hal_frame_rate framerate;
	struct v4l2_ctrl *ctrl;
	u32 prop_id;
	s32 value;
	int ret;

	prop_id = HAL_CONFIG_FRAME_RATE;
	framerate.buffer_type = HAL_BUFFER_OUTPUT;
	framerate.frame_rate = inst->fps * (1 << 16);

	ret = hfi_session_set_property(inst, prop_id, &framerate);
	if (ret) {
		dev_err(dev, "set framerate failed (%d)\n", ret);
		return ret;
	}

	/* set VUI timing info */
	ctrl = v4l2_ctrl_find(&inst->ctrl_handler,
			      V4L2_CID_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO);
	if (ctrl)
		ret = v4l2_ctrl_s_ctrl(ctrl,
			V4L2_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO_ENABLED);

	ctrl = v4l2_ctrl_find(&inst->ctrl_handler,
			      V4L2_CID_MPEG_VIDEO_BITRATE);
	if (ctrl) {
		value = v4l2_ctrl_g_ctrl(ctrl);

		ctrl = v4l2_ctrl_find(&inst->ctrl_handler,
				      V4L2_CID_MPEG_VIDEO_BITRATE_PEAK);
		if (ctrl) {
			value = value * 2;
			ret = v4l2_ctrl_s_ctrl(ctrl, value);
		}
	}

	return 0;
}

static int
venc_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strlcpy(cap->driver, VIDC_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->card, "video encoder", sizeof(cap->card));
	strlcpy(cap->bus_info, "media", sizeof(cap->bus_info));
	cap->version = VIDC_VERSION;

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
			    V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING |
			    V4L2_CAP_DEVICE_CAPS;
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
			   V4L2_CAP_VIDEO_OUTPUT_MPLANE |
			   V4L2_CAP_STREAMING;

	return 0;
}

static int venc_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	const struct vidc_format *fmt;

	fmt = find_format_by_index(f->index, f->type);

	memset(f->reserved, 0 , sizeof(f->reserved));

	if (!fmt)
		return -EINVAL;

	f->pixelformat = fmt->pixfmt;

	return 0;
}

static int venc_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	u32 pixelformat = pixmp->pixelformat;
	const struct vidc_format *fmt;
	struct vidc_core_capability *cap = &inst->capability;

	fmt = find_format(pixelformat, f->type);
	if (!fmt)
		return -EINVAL;

	pixmp->width = clamp(pixmp->width, cap->width.min, cap->width.max);
	pixmp->height = clamp(pixmp->height, cap->height.min, cap->height.max);
	pixmp->field = V4L2_FIELD_NONE;
	pixmp->colorspace = V4L2_COLORSPACE_DEFAULT;
	pixmp->num_planes = fmt->num_planes;
	pixmp->flags = 0;

	return 0;
}

static int venc_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct hfi_device *hdev = inst->core->hfidev;
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	const struct vidc_format *fmt;
	int ret, i;

	ret = venc_set_csc(inst);
	if (ret)
		return ret;

	inst->width = pixmp->width;
	inst->height = pixmp->height;

	fmt = find_format(pixmp->pixelformat, f->type);
	if (!fmt)
		return -EINVAL;

	ret = vidc_check_session_supported(inst);
	if (ret) {
		dprintk(VIDC_ERR, "%s: session not supported\n", __func__);
		return ret;
	}

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = hfi_session_init(inst, fmt->pixfmt);
		if (ret) {
			dprintk(VIDC_ERR, "Failed to init session (%d)\n", ret);
			return ret;
		}
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		struct hal_uncompressed_format_select hal_fmt;
		struct hal_buffer_requirements *bufreq;
		struct hal_frame_size fs;
		int extra_idx;

		fs.buffer_type = HAL_BUFFER_INPUT;
		fs.width = inst->width;
		fs.height = inst->height;

		ret = hfi_session_set_property(inst, HAL_PARAM_FRAME_SIZE, &fs);
		if (ret) {
			dprintk(VIDC_ERR,
				"set framesize for input failed (%d)\n", ret);
			return ret;
		}

		fs.buffer_type = HAL_BUFFER_OUTPUT;
		fs.width = inst->width;
		fs.height = inst->height;

		ret = hfi_session_set_property(inst, HAL_PARAM_FRAME_SIZE, &fs);
		if (ret) {
			dprintk(VIDC_ERR,
				"set framesize for output failed (%d)\n", ret);
			return ret;
		}

		switch (fmt->pixfmt) {
		case V4L2_PIX_FMT_NV12:
			hal_fmt.format = HAL_COLOR_FORMAT_NV12;
			break;
		case V4L2_PIX_FMT_NV21:
			hal_fmt.format = HAL_COLOR_FORMAT_NV21;
			break;
		default:
			return -ENOTSUPP;
		}

		hal_fmt.buffer_type = HAL_BUFFER_INPUT;

		ret = hfi_session_set_property(inst,
				HAL_PARAM_UNCOMPRESSED_FORMAT_SELECT, &hal_fmt);
		if (ret) {
			dprintk(VIDC_ERR,
				"setting uncompressed color format failed (%d)\n",
				ret);
			return ret;
		}

		extra_idx = EXTRADATA_IDX(fmt->num_planes);
		if (extra_idx && extra_idx < VIDEO_MAX_PLANES) {
			bufreq = get_buff_req_buffer(inst,
						HAL_BUFFER_EXTRADATA_INPUT);
			pixmp->plane_fmt[extra_idx].sizeimage =
						bufreq ? bufreq->size : 0;
		}
	}

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		inst->fmt_cap = fmt;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		inst->fmt_out = fmt;
	else
		return -EINVAL;

	pixmp->num_planes = fmt->num_planes;

	for (i = 0; i < fmt->num_planes; ++i) {
		pixmp->plane_fmt[i].sizeimage =
			fmt->get_framesize(i, pixmp->height, pixmp->width);

		pixmp->plane_fmt[i].bytesperline = 0;

		if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
			u32 bytesperline;

			call_hfi_op(hdev, get_stride_scanline, COLOR_FMT_NV12,
				    pixmp->width, pixmp->height,
				    &bytesperline, NULL);

			pixmp->plane_fmt[i].bytesperline = bytesperline;
		}
	}

	return 0;
}

static int venc_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct hfi_device *hdev = inst->core->hfidev;
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	struct hal_buffer_requirements *bufreq = NULL;
	const struct vidc_format *fmt = NULL;
	unsigned int extra_idx = 0;
	u32 height, width;
	int ret = 0, i;

	ret = vidc_comm_get_bufreqs(inst);
	if (ret) {
		dprintk(VIDC_WARN, "Getting buffer requirements failed: %d\n",
			ret);
		return ret;
	}

	height = inst->height;
	width = inst->width;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		fmt = inst->fmt_cap;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		fmt = inst->fmt_out;
	else
		return -ENOTSUPP;

	pixmp->pixelformat = fmt->pixfmt;
	pixmp->height = height;
	pixmp->width = width;
	pixmp->num_planes = fmt->num_planes;

	for (i = 0; i < fmt->num_planes; ++i) {
		pixmp->plane_fmt[i].sizeimage =
				fmt->get_framesize(i, height, width);

		pixmp->plane_fmt[i].bytesperline = 0;

		if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
			u32 bytesperline;

			call_hfi_op(hdev, get_stride_scanline, COLOR_FMT_NV12,
				    width, height, &bytesperline, NULL);

			pixmp->plane_fmt[i].bytesperline = bytesperline;
		}
	}

	extra_idx = EXTRADATA_IDX(fmt->num_planes);
	if (extra_idx && extra_idx < VIDEO_MAX_PLANES) {
		if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
			bufreq = get_buff_req_buffer(inst,
						HAL_BUFFER_EXTRADATA_OUTPUT);
		else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			bufreq = get_buff_req_buffer(inst,
						HAL_BUFFER_EXTRADATA_INPUT);

		pixmp->plane_fmt[extra_idx].sizeimage =
					bufreq ? bufreq->size : 0;
	}

	return ret;
}

static int
venc_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = venc_to_vb2q(inst, b->type);
	if (!queue)
		return -EINVAL;

	return vb2_reqbufs(queue, b);
}

static int venc_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;
	int ret = 0, i;

	if (b->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	queue = venc_to_vb2q(inst, b->type);
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

static int venc_prepare_buf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = venc_to_vb2q(inst, b->type);
	if (!queue)
		return -EINVAL;

	return vb2_prepare_buf(queue, b);
}

static int venc_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;
	unsigned int state;

	queue = venc_to_vb2q(inst, b->type);
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

static int
venc_exportbuf(struct file *file, void *fh, struct v4l2_exportbuffer *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = venc_to_vb2q(inst, b->type);
	if (!queue)
		return -EINVAL;

	return vb2_expbuf(queue, b);
}

static int venc_return_buf_error(struct vidc_inst *inst, struct v4l2_buffer *b)
{
	struct vb2_queue *qcap, *qout;
	struct list_head *ptr, *next;
	struct venc_buffer *buf;

	qcap = &inst->bufq_cap;
	qout = &inst->bufq_out;

	if (vb2_is_streaming(qcap) && vb2_is_streaming(qout))
		return 0;

	mutex_lock(&inst->bufqueue_lock);
	list_for_each_safe(ptr, next, &inst->bufqueue) {
		buf = list_entry(ptr, struct venc_buffer, list);

		if (buf->vb.vb2_buf.type != b->type)
			continue;

		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	mutex_unlock(&inst->bufqueue_lock);

	return 0;
}

static int venc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = venc_to_vb2q(inst, b->type);
	if (!queue)
		return -EINVAL;

	venc_return_buf_error(inst, b);

	return vb2_dqbuf(queue, b, file->f_flags & O_NONBLOCK);
}

static int venc_streamon(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = venc_to_vb2q(inst, type);
	if (!queue)
		return -EINVAL;

	return vb2_streamon(queue, type);
}

static int venc_streamoff(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vb2_queue *queue;

	queue = venc_to_vb2q(inst, type);
	if (!queue)
		return -EINVAL;

	return vb2_streamoff(queue, type);
}

static int venc_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct device *dev = &inst->core->res.pdev->dev;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u64 us_per_frame, fps;

	if (!timeperframe->denominator ||
	    a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	us_per_frame = timeperframe->numerator * (u64)USEC_PER_SEC;
	do_div(us_per_frame, timeperframe->denominator);

	if (!us_per_frame)
		return -EINVAL;

	fps = (u64)USEC_PER_SEC;
	do_div(fps, us_per_frame);

	dev_dbg(dev, "%s: num:%d, denom:%d\n", __func__,
		timeperframe->numerator, timeperframe->denominator);

	inst->fps = fps;
	inst->timeperframe = *timeperframe;

	return 0;
}

static int venc_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct vidc_inst *inst = to_inst(file, fh);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.capture.timeperframe = inst->timeperframe;

	return 0;
}

static int venc_enum_framesizes(struct file *file, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vidc_core_capability *cap = &inst->capability;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = cap->width.min;
	fsize->stepwise.max_width = cap->width.max;
	fsize->stepwise.step_width = cap->width.step_size;
	fsize->stepwise.min_height = cap->height.min;
	fsize->stepwise.max_height = cap->height.max;
	fsize->stepwise.step_height = cap->height.step_size;

	return 0;
}

static int venc_subscribe_event(struct v4l2_fh *fh,
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

static int venc_cmd(struct file *file, void *fh, struct v4l2_encoder_cmd *enc)
{
	struct vidc_inst *inst = to_inst(file, fh);
	struct vidc_core *core = inst->core;
	int ret = 0;

	switch (enc->cmd) {
	case V4L2_ENC_QCOM_CMD_FLUSH:
		ret = vidc_comm_session_flush(inst, enc->flags);
		break;
	case V4L2_ENC_CMD_STOP:
		if (inst->state == INST_INVALID || core->state == CORE_INVALID)
			return ret;

		ret = release_scratch_buffers(inst, false);
		if (ret)
			dprintk(VIDC_ERR, "Failed to release scratch buf:%d\n",
				ret);

		ret = release_persist_buffers(inst);
		if (ret)
			dprintk(VIDC_ERR, "Failed to release persist buf:%d\n",
				ret);

		ret = hfi_session_deinit(inst);
		if (ret)
			return ret;

		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static const struct v4l2_ioctl_ops venc_ioctl_ops = {
	.vidioc_querycap = venc_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = venc_enum_fmt,
	.vidioc_enum_fmt_vid_out_mplane = venc_enum_fmt,
	.vidioc_s_fmt_vid_cap_mplane = venc_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = venc_s_fmt,
	.vidioc_g_fmt_vid_cap_mplane = venc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = venc_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = venc_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = venc_try_fmt,
	.vidioc_reqbufs = venc_reqbufs,
	.vidioc_querybuf = venc_querybuf,
	.vidioc_prepare_buf = venc_prepare_buf,
	.vidioc_qbuf = venc_qbuf,
	.vidioc_expbuf = venc_exportbuf,
	.vidioc_dqbuf = venc_dqbuf,
	.vidioc_streamon = venc_streamon,
	.vidioc_streamoff = venc_streamoff,
	.vidioc_s_parm = venc_s_parm,
	.vidioc_g_parm = venc_g_parm,
	.vidioc_enum_framesizes = venc_enum_framesizes,
	.vidioc_subscribe_event = venc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_encoder_cmd = venc_cmd,
};

static int venc_queue_setup(struct vb2_queue *q, const void *parg,
			    unsigned int *num_buffers,
			    unsigned int *num_planes, unsigned int sizes[],
			    void *alloc_ctxs[])
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct device *dev = &inst->core->res.pdev->dev;
	struct hal_buffer_count_actual buf_count;
	struct hal_buffer_requirements *buff_req;
	enum hal_property property_id;
	int i, ret = 0;

	ret = vidc_comm_get_bufreqs(inst);
	if (ret) {
		dev_err(dev, "buffer requirements (%d)\n", ret);
		return ret;
	}

	switch (q->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		*num_planes = inst->fmt_cap->num_planes;

		buff_req = get_buff_req_buffer(inst, HAL_BUFFER_OUTPUT);
		if (!buff_req) {
			ret = -EINVAL;
			break;
		}

		*num_buffers = max(*num_buffers, buff_req->count_actual);

		*num_buffers = clamp_val(*num_buffers, MIN_NUM_CAPTURE_BUFFERS,
					 VIDEO_MAX_FRAME);

		for (i = 0; i < *num_planes; i++) {
			sizes[i] = inst->fmt_cap->get_framesize(i, inst->height,
								inst->width);
			alloc_ctxs[i] = inst->vb2_ctx_cap;
		}

		property_id = HAL_PARAM_BUFFER_COUNT_ACTUAL;
		buf_count.type = HAL_BUFFER_OUTPUT;
		buf_count.count_actual = *num_buffers;

		ret = hfi_session_set_property(inst, property_id, &buf_count);
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		*num_planes = inst->fmt_out->num_planes;

		buff_req = get_buff_req_buffer(inst, HAL_BUFFER_INPUT);
		if (!buff_req) {
			ret = -EINVAL;
			break;
		}

		*num_buffers = max(*num_buffers, buff_req->count_actual);

		property_id = HAL_PARAM_BUFFER_COUNT_ACTUAL;
		buf_count.type = HAL_BUFFER_INPUT;
		buf_count.count_actual = *num_buffers;

		ret = hfi_session_set_property(inst, property_id, &buf_count);
		if (ret)
			break;

		sizes[0] = inst->fmt_out->get_framesize(0, inst->height,
							inst->width);
		alloc_ctxs[0] = inst->vb2_ctx_out;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int venc_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *q = vb->vb2_queue;
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct device *dev = &inst->core->res.pdev->dev;
	struct hfi_device *hdev = inst->core->hfidev;
	struct venc_buffer *buf = to_venc_buffer(vbuf);
	struct vidc_buffer_addr_info *bai;
	struct buffer_info *bi;
	int ret;

	bi = &buf->bi;
	bai = &bi->bai;

	memset(bai, 0, sizeof(*bai));

	if (q->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return 0;

	bai->buffer_size = vb2_plane_size(vb, 0);
	bai->buffer_type = HAL_BUFFER_OUTPUT;
	bai->num_buffers = 1;
	bai->device_addr = vb2_dma_contig_plane_dma_addr(vb, 0);

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

static int venc_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct venc_buffer *buf = to_venc_buffer(vbuf);

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

static int venc_set_session_buf(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *q = vb->vb2_queue;
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct vidc_core *core = inst->core;
	struct device *dev = &core->res.pdev->dev;
	struct hfi_device *hdev = core->hfidev;
	struct venc_buffer *buf = to_venc_buffer(vbuf);
	struct vidc_frame_data fdata;
	s64 time_usec;
	int ret;

	if (inst->state == INST_INVALID || core->state == CORE_INVALID) {
		dev_err(dev, "core id:%d is in bad state\n", core->id);
		return -EINVAL;
	}

	time_usec = timeval_to_ns(&vbuf->timestamp);
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
		fdata.offset = vb->planes[0].data_offset;

		__fill_flags(&fdata, vbuf->flags);

		ret = call_hfi_op(hdev, session_etb, inst->session, &fdata);
	} else if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fdata.buffer_type = HAL_BUFFER_OUTPUT;
		fdata.filled_len = 0;
		fdata.offset = 0;

		ret = call_hfi_op(hdev, session_ftb, inst->session, &fdata);
	} else {
		ret = -EINVAL;
	}

	if (ret) {
		dev_err(dev, "failed to set session buffer (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int start_streaming(struct vidc_inst *inst)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct list_head *ptr, *next;
	struct venc_buffer *buf;
	int ret;

	ret = venc_set_properties(inst);
	if (ret) {
		dev_err(dev, "set properties (%d)\n", ret);
		return ret;
	}

	ret = vidc_comm_get_bufreqs(inst);
	if (ret) {
		dev_err(dev, "buffers requirements (%d)\n", ret);
		return ret;
	}

	ret = set_scratch_buffers(inst);
	if (ret) {
		dev_err(dev, "set scratch buffers failed (%d)\n", ret);
		return ret;
	}

	ret = set_persist_buffers(inst);
	if (ret) {
		dev_err(dev, "set persist buffers failed (%d)\n", ret);
		return ret;
	}

	msm_comm_scale_clocks(inst->core);

	ret = hfi_session_load_res(inst);
	if (ret) {
		dev_err(dev, "session load resources failed (%d)\n", ret);
		return ret;
	}

	ret = hfi_session_start(inst);
	if (ret) {
		dev_err(dev, "session start failed (%d)\n", ret);
		return ret;
	}

	mutex_lock(&inst->bufqueue_lock);
	list_for_each_safe(ptr, next, &inst->bufqueue) {
		buf = list_entry(ptr, struct venc_buffer, list);

		ret = venc_set_session_buf(&buf->vb.vb2_buf);
		if (ret)
			break;
	}
	mutex_unlock(&inst->bufqueue_lock);

	return 0;
}

static int venc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct vb2_queue *queue;

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

static void venc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vidc_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vidc_core *core = inst->core;
	struct device *dev = &core->res.pdev->dev;
	struct venc_buffer *buf = to_venc_buffer(vbuf);
	int ret;

	if (inst->state == INST_INVALID || core->state == CORE_INVALID) {
		dev_err(dev, "core or instance are in invalid state\n");
		return;
	}

	mutex_lock(&inst->bufqueue_lock);
	list_add_tail(&buf->list, &inst->bufqueue);
	mutex_unlock(&inst->bufqueue_lock);

	if (!vb2_is_streaming(&inst->bufq_cap) ||
	    !vb2_is_streaming(&inst->bufq_out))
		return;

	ret = venc_set_session_buf(vb);
	if (ret) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		dev_err(dev, "cannot set session buffer\n");
	}

	return;
}

static int venc_rel_session_bufs(struct vidc_inst *inst)
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

	ret = venc_rel_session_bufs(inst);
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

static void venc_stop_streaming(struct vb2_queue *q)
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

static const struct vb2_ops venc_vb2_ops = {
	.queue_setup = venc_queue_setup,
	.buf_init = venc_buf_init,
	.buf_prepare = venc_buf_prepare,
	.start_streaming = venc_start_streaming,
	.stop_streaming = venc_stop_streaming,
	.buf_queue = venc_buf_queue,
};

static struct vb2_v4l2_buffer *
venc_get_vb2buffer(struct vidc_inst *inst, dma_addr_t addr)
{
	struct venc_buffer *buf;
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

static int venc_empty_buf_done(struct vidc_inst *inst, u32 addr,
			       u32 bytesused, u32 data_offset, u32 flags)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct vb2_v4l2_buffer *vbuf;
	struct vb2_buffer *vb;

	vbuf = venc_get_vb2buffer(inst, addr);
	if (!vbuf)
		return -EFAULT;

	vb = &vbuf->vb2_buf;
	vb->planes[0].bytesused = bytesused;
	vb->planes[0].data_offset = data_offset;
	vbuf->flags = flags;

	if (vb->planes[0].data_offset > vb->planes[0].length)
		dev_dbg(dev, "data_offset overflow length\n");

	if (vb->planes[0].bytesused > vb->planes[0].length)
		dev_dbg(dev, "bytesused overflow length\n");

	if (flags & V4L2_QCOM_BUF_INPUT_UNSUPPORTED)
		dev_dbg(dev, "unsupported input stream\n");

	if (flags & V4L2_QCOM_BUF_DATA_CORRUPT)
		dev_dbg(dev, "corrupted input stream\n");

	if (flags & V4L2_MSM_VIDC_BUF_START_CODE_NOT_FOUND)
		dev_dbg(dev, "start code not found\n");

	inst->count.ebd++;

	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);

	return 0;
}

static int venc_fill_buf_done(struct vidc_inst *inst, u32 addr,
			      u32 bytesused, u32 data_offset, u32 flags,
			      struct timeval *timestamp)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct vb2_v4l2_buffer *vbuf;
	struct vb2_buffer *vb;

	vbuf = venc_get_vb2buffer(inst, addr);
	if (!vbuf)
		return -EFAULT;

	vb = &vbuf->vb2_buf;
	vb->planes[0].bytesused = bytesused;
	vb->planes[0].data_offset = data_offset;
	vbuf->flags = flags;
	vbuf->timestamp = *timestamp;

	if (vb->planes[0].data_offset > vb->planes[0].length)
		dev_warn(dev, "overflow data_offset:%d, length:%d\n",
			 vb->planes[0].data_offset, vb->planes[0].length);

	if (vb->planes[0].bytesused > vb->planes[0].length)
		dev_warn(dev, "overflow bytesused:%d, length:%d\n",
			 vb->planes[0].bytesused, vb->planes[0].length);

	inst->count.fbd++;

	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);

	return 0;
}

static int venc_event_notify(struct vidc_inst *inst, u32 event,
			     struct vidc_cb_event *data)
{
	struct device *dev = &inst->core->res.pdev->dev;

	switch (event) {
	case SESSION_ERROR:
		/* the instance lock is already taken just change the state */
		inst->state = INST_INVALID;
		dev_warn(dev, "event session error\n");
		break;
	case SYS_EVENT_CHANGE:
		switch (data->hal_event_type) {
		case HAL_EVENT_SEQ_CHANGED_SUFFICIENT_RESOURCES:
			dev_dbg(dev, "event sufficient resources\n");
			break;
		case HAL_EVENT_SEQ_CHANGED_INSUFFICIENT_RESOURCES:
			dev_dbg(dev, "event not sufficient resources\n");
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

static void venc_inst_init(struct vidc_inst *inst)
{
	inst->fmt_cap = &venc_formats[2];
	inst->fmt_out = &venc_formats[0];
	inst->height = DEFAULT_HEIGHT;
	inst->width = DEFAULT_WIDTH;
	inst->fps = 15;
	inst->capability.pixelprocess_capabilities = 0;
	inst->buffer_mode[OUTPUT_PORT] = HAL_BUFFER_MODE_STATIC;
	inst->buffer_mode[CAPTURE_PORT] = HAL_BUFFER_MODE_STATIC;
}

static void venc_release_video_device(struct video_device *pvdev)
{
}

extern const struct v4l2_file_operations vidc_fops;

int venc_init(struct vidc_core *core, struct video_device *enc)
{
	int ret;

	/* setup the decoder device */
	enc->release = venc_release_video_device;
	enc->fops = &vidc_fops;
	enc->ioctl_ops = &venc_ioctl_ops;
	enc->vfl_dir = VFL_DIR_M2M;
	enc->v4l2_dev = &core->v4l2_dev;

	ret = video_register_device(enc, VFL_TYPE_GRABBER, 33);
	if (ret) {
		dprintk(VIDC_ERR, "Failed to register video encoder device");
		return ret;
	}

	video_set_drvdata(enc, core);

	return 0;
}

void venc_deinit(struct vidc_core *core, struct video_device *enc)
{
	video_unregister_device(enc);
}

int venc_open(struct vidc_inst *inst)
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

	venc_inst_init(inst);

	ret = venc_ctrl_init(inst);
	if (ret)
		return ret;

	inst->vb2_ctx_cap = vb2_dma_contig_init_ctx(iommu_dev);
	if (IS_ERR(inst->vb2_ctx_cap))
		return PTR_ERR(inst->vb2_ctx_cap);

	inst->vb2_ctx_out = vb2_dma_contig_init_ctx(iommu_dev);
	if (IS_ERR(inst->vb2_ctx_out)) {
		ret = PTR_ERR(inst->vb2_ctx_out);
		goto err_cleanup_cap;
	}

	q = &inst->bufq_cap;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->ops = &venc_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->drv_priv = inst;
	q->buf_struct_size = sizeof(struct venc_buffer);
	q->allow_zero_bytesused = 1;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_cleanup_out;

	q = &inst->bufq_out;
	q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->ops = &venc_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->drv_priv = inst;
	q->buf_struct_size = sizeof(struct venc_buffer);
	q->allow_zero_bytesused = 1;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_cap_queue_release;

	inst->empty_buf_done = venc_empty_buf_done;
	inst->fill_buf_done = venc_fill_buf_done;
	inst->event_notify = venc_event_notify;

	return 0;

err_cap_queue_release:
	vb2_queue_release(&inst->bufq_cap);
err_cleanup_out:
	vb2_dma_contig_cleanup_ctx(inst->vb2_ctx_out);
err_cleanup_cap:
	vb2_dma_contig_cleanup_ctx(inst->vb2_ctx_cap);
	return ret;
}

void venc_close(struct vidc_inst *inst)
{
	vb2_queue_release(&inst->bufq_out);
	vb2_queue_release(&inst->bufq_cap);

	vb2_dma_contig_cleanup_ctx(inst->vb2_ctx_out);
	vb2_dma_contig_cleanup_ctx(inst->vb2_ctx_cap);

	venc_ctrl_deinit(inst);
}
