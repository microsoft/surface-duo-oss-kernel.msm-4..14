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

#ifndef __VIDC_CORE_H_
#define __VIDC_CORE_H_

#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>

#include "hfi.h"

#define VIDC_DRV_NAME		"vidc"

/* structures needed to diferenciate resources per version/SoC */
#define VIDC_CLKS_NUM_MAX	12

struct freq_tbl {
	unsigned int load;
	unsigned long freq;
};

struct reg_val {
	u32 reg;
	u32 value;
};

struct vidc_resources {
	u64 dma_mask;
	const struct freq_tbl *freq_tbl;
	unsigned int freq_tbl_size;
	const struct reg_val *reg_tbl;
	unsigned int reg_tbl_size;
	const char * const clks[VIDC_CLKS_NUM_MAX];
	unsigned int clks_num;
	enum hfi_version hfi_version;
	u32 max_load;
	unsigned int vmem_id;
	u32 vmem_size;
	u32 vmem_addr;
};

struct vidc_format {
	u32 pixfmt;
	int num_planes;
	u32 type;
};

struct vidc_core {
	void __iomem *base;
	int irq;
	struct clk *clks[VIDC_CLKS_NUM_MAX];
	struct video_device *vdev_dec;
	struct video_device *vdev_enc;
	struct v4l2_device v4l2_dev;
	const struct vidc_resources *res;
	struct rproc *rproc;
	struct device *dev;
	struct mutex lock;
	struct list_head instances;

	/* HFI core fields */
	unsigned int state;
	struct completion done;
	unsigned int error;

	/* core operations passed by outside world */
	const struct hfi_core_ops *core_ops;

	/* filled by sys core init */
	u32 enc_codecs;
	u32 dec_codecs;
	unsigned int max_sessions_supported;

	/* core capabilities */
#define ENC_ROTATION_CAPABILITY		0x1
#define ENC_SCALING_CAPABILITY		0x2
#define ENC_DEINTERLACE_CAPABILITY	0x4
#define DEC_MULTI_STREAM_CAPABILITY	0x8
	unsigned int core_caps;

	/* internal hfi operations */
	void *priv;
	const struct hfi_ops *ops;
};

struct vdec_controls {
	u32 post_loop_deb_mode;
	u32 profile;
	u32 level;
};

struct venc_controls {
	u16 gop_size;
	u32 idr_period;
	u32 num_p_frames;
	u32 num_b_frames;
	u32 bitrate_mode;
	u32 bitrate;
	u32 bitrate_peak;

	u32 h264_i_period;
	u32 h264_entropy_mode;
	u32 h264_i_qp;
	u32 h264_p_qp;
	u32 h264_b_qp;
	u32 h264_min_qp;
	u32 h264_max_qp;
	u32 h264_loop_filter_mode;
	u32 h264_loop_filter_alpha;
	u32 h264_loop_filter_beta;

	u32 vp8_min_qp;
	u32 vp8_max_qp;

	u32 multi_slice_mode;
	u32 multi_slice_max_bytes;
	u32 multi_slice_max_mb;

	u32 header_mode;

	u32 profile;
	u32 level;
};

struct vidc_inst {
	struct list_head list;
	struct mutex lock;

	struct vidc_core *core;

	struct list_head internalbufs;
	struct mutex internalbufs_lock;

	struct list_head registeredbufs;
	struct mutex registeredbufs_lock;

	struct list_head bufqueue;
	struct mutex bufqueue_lock;

	struct vb2_queue bufq_out;
	struct vb2_queue bufq_cap;

	struct v4l2_ctrl_handler ctrl_handler;
	union {
		struct vdec_controls dec;
		struct venc_controls enc;
	} controls;
	struct v4l2_fh fh;

	void *alloc_ctx_cap;
	void *alloc_ctx_out;

	/* v4l2 fields */
	u32 width;
	u32 height;
	u32 out_width;
	u32 out_height;
	u32 colorspace;
	u8 ycbcr_enc;
	u8 quantization;
	u8 xfer_func;
	u64 fps;
	struct v4l2_fract timeperframe;
	const struct vidc_format *fmt_out;
	const struct vidc_format *fmt_cap;
	unsigned int num_input_bufs;
	unsigned int num_output_bufs;
	unsigned int output_buf_size;
	bool in_reconfig;
	u32 reconfig_width;
	u32 reconfig_height;
	u64 sequence;
	bool codec_cfg;

	/* HFI instance fields */
	unsigned int state;
	struct completion done;
	unsigned int error;

	/* instance operations passed by outside world */
	const struct hfi_inst_ops *ops;
	void *priv;
	u32 session_type;
	union hfi_get_property hprop;

	/* capabilities filled by session_init */
	struct hfi_capability cap_width;
	struct hfi_capability cap_height;
	struct hfi_capability cap_mbs_per_frame;
	struct hfi_capability cap_mbs_per_sec;
	struct hfi_capability cap_framerate;
	struct hfi_capability cap_scale_x;
	struct hfi_capability cap_scale_y;
	struct hfi_capability cap_bitrate;
	struct hfi_capability cap_hier_p;
	struct hfi_capability cap_ltr_count;
	struct hfi_capability cap_secure_output2_threshold;
	bool cap_bufs_mode_static;
	bool cap_bufs_mode_dynamic;

	/* profile & level pairs supported */
	unsigned int pl_count;
	struct hfi_profile_level pl[HFI_MAX_PROFILE_COUNT];

	/* buffer requirements */
	struct hfi_buffer_requirements bufreq[HFI_BUFFER_TYPE_MAX];
};

#define ctrl_to_inst(ctrl)	\
	container_of(ctrl->handler, struct vidc_inst, ctrl_handler)

struct vidc_ctrl {
	u32 id;
	enum v4l2_ctrl_type type;
	s32 min;
	s32 max;
	s32 def;
	u32 step;
	u64 menu_skip_mask;
	u32 flags;
	const char * const *qmenu;
};

/*
 * Offset base for buffers on the destination queue - used to distinguish
 * between source and destination buffers when mmapping - they receive the same
 * offsets but for different queues
 */
#define DST_QUEUE_OFF_BASE	(1 << 30)

static inline struct vidc_inst *to_inst(struct file *filp)
{
	return container_of(filp->private_data, struct vidc_inst, fh);
}

static inline void *to_hfi_priv(struct vidc_core *core)
{
	return core->priv;
}

static inline struct vb2_queue *
vidc_to_vb2q(struct file *file, enum v4l2_buf_type type)
{
	struct vidc_inst *inst = to_inst(file);

	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return &inst->bufq_cap;
	else if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return &inst->bufq_out;

	return NULL;
}

#endif
