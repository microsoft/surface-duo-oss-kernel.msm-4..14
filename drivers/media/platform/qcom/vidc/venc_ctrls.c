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
#include <linux/types.h>
#include <media/v4l2-ctrls.h>

#include "core.h"

#define BITRATE_MIN		32000
#define BITRATE_MAX		160000000
#define BITRATE_DEFAULT		1000000
#define BITRATE_DEFAULT_PEAK	(BITRATE_DEFAULT * 2)
#define BITRATE_STEP		100
#define SLICE_BYTE_SIZE_MAX	1024
#define SLICE_BYTE_SIZE_MIN	1024
#define SLICE_MB_SIZE_MAX	300
#define INTRA_REFRESH_MBS_MAX	300
#define AT_SLICE_BOUNDARY	\
	V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY
static struct vidc_ctrl venc_ctrls[] = {
	{
		.id = V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR,
		.max = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR,
		.def = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR,
		.menu_skip_mask = ~((1 << V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) |
				    (1 << V4L2_MPEG_VIDEO_BITRATE_MODE_CBR)),
	}, {
		.id = V4L2_CID_MPEG_VIDEO_BITRATE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = BITRATE_MIN,
		.max = BITRATE_MAX,
		.def = BITRATE_DEFAULT,
		.step = BITRATE_STEP,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_BITRATE_PEAK,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = BITRATE_MIN,
		.max = BITRATE_MAX,
		.def = BITRATE_DEFAULT_PEAK,
		.step = BITRATE_STEP,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC,
		.max = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC,
		.def = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE,
		.max = V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_CODING_EFFICIENCY,
		.def = V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE) |
			(1 << V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_SIMPLE)
		),
	}, {
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_MPEG4_LEVEL_0,
		.max = V4L2_MPEG_VIDEO_MPEG4_LEVEL_5,
		.def = V4L2_MPEG_VIDEO_MPEG4_LEVEL_0,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.max = V4L2_MPEG_VIDEO_H264_PROFILE_MULTIVIEW_HIGH,
		.def = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
		.menu_skip_mask = ~(
		(1 << V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE) |
		(1 << V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE) |
		(1 << V4L2_MPEG_VIDEO_H264_PROFILE_MAIN) |
		(1 << V4L2_MPEG_VIDEO_H264_PROFILE_HIGH) |
		(1 << V4L2_MPEG_VIDEO_H264_PROFILE_STEREO_HIGH) |
		(1 << V4L2_MPEG_VIDEO_H264_PROFILE_MULTIVIEW_HIGH)
		),
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.max = V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
		.def = V4L2_MPEG_VIDEO_H264_LEVEL_5_0,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_VPX_PROFILE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 3,
		.def = 0,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 51,
		.def = 26,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 51,
		.def = 28,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 51,
		.def = 30,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 51,
		.def = 1,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 51,
		.def = 51,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE,
		.max = V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES,
		.def = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = SLICE_BYTE_SIZE_MIN,
		.max = SLICE_BYTE_SIZE_MAX,
		.def = SLICE_BYTE_SIZE_MIN,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = SLICE_MB_SIZE_MAX,
		.def = 1,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -6,
		.max = 6,
		.def = 0,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -6,
		.max = 6,
		.def = 0,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED,
		.max = AT_SLICE_BOUNDARY,
		.def = V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_HEADER_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE,
		.max = V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
		.def = V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE,
		.menu_skip_mask =
			1 << V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = INTRA_REFRESH_MBS_MAX,
		.def = 0,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.def = 0,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED,
		.max = V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED,
		.def = V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_GOP_SIZE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = (1 << 16) - 1,
		.def = 12,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = (1 << 16) - 1,
		.def = 0,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.def = 0,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_VPX_MIN_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 128,
		.def = 1,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_VPX_MAX_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 128,
		.def = 128,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_B_FRAMES,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = INT_MAX,
		.def = 0,
		.step = 1,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = (1 << 16) - 1,
		.step = 1,
		.def = 0,
	},
};

#define NUM_CTRLS	ARRAY_SIZE(venc_ctrls)

static int venc_op_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vidc_inst *inst = ctrl_to_inst(ctrl);
	struct venc_controls *ctr = &inst->controls.enc;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		ctr->bitrate_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ctr->bitrate = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE_PEAK:
		ctr->bitrate_peak = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE:
		ctr->h264_entropy_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE:
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
	case V4L2_CID_MPEG_VIDEO_VPX_PROFILE:
		ctr->profile = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL:
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		ctr->level = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		ctr->h264_i_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		ctr->h264_p_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP:
		ctr->h264_b_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		ctr->h264_min_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		ctr->h264_max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE:
		ctr->multi_slice_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES:
		ctr->multi_slice_max_bytes = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
		ctr->multi_slice_max_mb = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA:
		ctr->h264_loop_filter_alpha = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA:
		ctr->h264_loop_filter_beta = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE:
		ctr->h264_loop_filter_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		ctr->header_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB:
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctr->gop_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_PERIOD:
		ctr->h264_i_period = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE:
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC:
	case V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE:
	case V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM:
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_MIN_QP:
		ctr->vp8_min_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_MAX_QP:
		ctr->vp8_max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_B_FRAMES:
		ctr->num_b_frames = ctrl->val;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops venc_ctrl_ops = {
	.s_ctrl = venc_op_s_ctrl,
};

int venc_ctrl_init(struct vidc_inst *inst)
{
	unsigned int i;
	int ret;

	ret = v4l2_ctrl_handler_init(&inst->ctrl_handler, NUM_CTRLS);
	if (ret)
		return ret;

	for (i = 0; i < NUM_CTRLS; i++) {
		struct v4l2_ctrl *ctrl;

		if (venc_ctrls[i].type == V4L2_CTRL_TYPE_MENU) {
			ctrl = v4l2_ctrl_new_std_menu(&inst->ctrl_handler,
					&venc_ctrl_ops, venc_ctrls[i].id,
					venc_ctrls[i].max,
					venc_ctrls[i].menu_skip_mask,
					venc_ctrls[i].def);
		} else {
			ctrl = v4l2_ctrl_new_std(&inst->ctrl_handler,
					&venc_ctrl_ops, venc_ctrls[i].id,
					venc_ctrls[i].min,
					venc_ctrls[i].max,
					venc_ctrls[i].step,
					venc_ctrls[i].def);
		}

		ret = inst->ctrl_handler.error;
		if (ret) {
			v4l2_ctrl_handler_free(&inst->ctrl_handler);
			return ret;
		}
	}

	return 0;
}

void venc_ctrl_deinit(struct vidc_inst *inst)
{
	v4l2_ctrl_handler_free(&inst->ctrl_handler);
}
