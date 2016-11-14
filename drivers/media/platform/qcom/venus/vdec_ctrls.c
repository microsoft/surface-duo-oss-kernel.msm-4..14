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

static struct venus_ctrl vdec_ctrls[] = {
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE,
		.max = V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_CODING_EFFICIENCY,
		.def = V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
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
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.max = V4L2_MPEG_VIDEO_H264_PROFILE_MULTIVIEW_HIGH,
		.def = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
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
		.def = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_VPX_PROFILE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 3,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.def = 0,
	},
};

#define NUM_CTRLS	ARRAY_SIZE(vdec_ctrls)

static int vdec_op_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct venus_inst *inst = ctrl_to_inst(ctrl);
	struct vdec_controls *ctr = &inst->controls.dec;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER:
		ctr->post_loop_deb_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
	case V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE:
	case V4L2_CID_MPEG_VIDEO_VPX_PROFILE:
		ctr->profile = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
	case V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL:
		ctr->level = ctrl->val;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int vdec_op_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct venus_inst *inst = ctrl_to_inst(ctrl);
	struct vdec_controls *ctr = &inst->controls.dec;
	union hfi_get_property hprop;
	u32 ptype = HFI_PROPERTY_PARAM_PROFILE_LEVEL_CURRENT;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
	case V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE:
	case V4L2_CID_MPEG_VIDEO_VPX_PROFILE:
		ret = hfi_session_get_property(inst, ptype, &hprop);
		if (!ret)
			ctr->profile = hprop.profile_level.profile;
		ctrl->val = ctr->profile;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
	case V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL:
		ret = hfi_session_get_property(inst, ptype, &hprop);
		if (!ret)
			ctr->level = hprop.profile_level.level;
		ctrl->val = ctr->level;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER:
		ctrl->val = ctr->post_loop_deb_mode;
		break;
	default:
		return -EINVAL;
	};

	return 0;
}

static const struct v4l2_ctrl_ops vdec_ctrl_ops = {
	.s_ctrl = vdec_op_s_ctrl,
	.g_volatile_ctrl = vdec_op_g_volatile_ctrl,
};

int vdec_ctrl_init(struct venus_inst *inst)
{
	unsigned int i;
	int ret;

	ret = v4l2_ctrl_handler_init(&inst->ctrl_handler, NUM_CTRLS);
	if (ret)
		return ret;

	for (i = 0; i < NUM_CTRLS; i++) {
		struct v4l2_ctrl *ctrl;

		if (vdec_ctrls[i].type == V4L2_CTRL_TYPE_MENU) {
			ctrl = v4l2_ctrl_new_std_menu(&inst->ctrl_handler,
					&vdec_ctrl_ops,
					vdec_ctrls[i].id,
					vdec_ctrls[i].max,
					vdec_ctrls[i].menu_skip_mask,
					vdec_ctrls[i].def);
		} else {
			ctrl = v4l2_ctrl_new_std(&inst->ctrl_handler,
					&vdec_ctrl_ops,
					vdec_ctrls[i].id,
					vdec_ctrls[i].min,
					vdec_ctrls[i].max,
					vdec_ctrls[i].step,
					vdec_ctrls[i].def);
		}

		if (!ctrl)
			return -EINVAL;

		switch (vdec_ctrls[i].id) {
		case V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE:
		case V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL:
		case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		case V4L2_CID_MPEG_VIDEO_VPX_PROFILE:
			ctrl->flags |= vdec_ctrls[i].flags;
			break;
		}

		ret = inst->ctrl_handler.error;
		if (ret) {
			v4l2_ctrl_handler_free(&inst->ctrl_handler);
			return ret;
		}
	}

	return ret;
}

void vdec_ctrl_deinit(struct venus_inst *inst)
{
	v4l2_ctrl_handler_free(&inst->ctrl_handler);
}
