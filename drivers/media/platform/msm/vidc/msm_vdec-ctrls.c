/*
 * Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 Linaro Ltd.
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
#include <linux/cpumask.h>
#include <linux/types.h>
#include <linux/v4l2-controls.h>
#include <linux/qcom_scm.h>
#include <media/msm-v4l2-controls.h>
#include <linux/kernel.h>

#include "msm_vidc_debug.h"
#include "msm_vidc_common.h"
#include "msm_vidc_load.h"
#include "msm_hfi_interface.h"

#define DEFAULT_VIDEO_CONCEAL_COLOR_BLACK	0x8010
#define TZ_DYNAMIC_BUFFER_FEATURE_ID		12
#define TZ_FEATURE_VERSION(major, minor, patch) \
	(((major & 0x3FF) << 22) | ((minor & 0x3FF) << 12) | (patch & 0xFFF))

static const char *const mpeg_video_vidc_divx_format[] = {
	"DIVX Format 4",
	"DIVX Format 5",
	"DIVX Format 6",
	NULL
};
static const char *mpeg_video_stream_format[] = {
	"NAL Format Start Codes",
	"NAL Format One NAL Per Buffer",
	"NAL Format One Byte Length",
	"NAL Format Two Byte Length",
	"NAL Format Four Byte Length",
	NULL
};
static const char *const mpeg_video_output_order[] = {
	"Display Order",
	"Decode Order",
	NULL
};
static const char *const mpeg_video_vidc_extradata[] = {
	"Extradata none",
	"Extradata MB Quantization",
	"Extradata Interlace Video",
	"Extradata VC1 Framedisp",
	"Extradata VC1 Seqdisp",
	"Extradata timestamp",
	"Extradata S3D Frame Packing",
	"Extradata Frame Rate",
	"Extradata Panscan Window",
	"Extradata Recovery point SEI",
	"Extradata Closed Caption UD",
	"Extradata AFD UD",
	"Extradata Multislice info",
	"Extradata number of concealed MB",
	"Extradata metadata filler",
	"Extradata input crop",
	"Extradata digital zoom",
	"Extradata aspect ratio",
	"Extradata mpeg2 seqdisp",
};
static const char *const mpeg_vidc_video_alloc_mode_type[] = {
	"Buffer Allocation Static",
	"Buffer Allocation Ring Buffer",
	"Buffer Allocation Dynamic Buffer"
};

static const char *const perf_level[] = {
	"Nominal",
	"Performance",
	"Turbo"
};

static const char *const h263_level[] = {
	"1.0",
	"2.0",
	"3.0",
	"4.0",
	"4.5",
	"5.0",
	"6.0",
	"7.0",
};

static const char *const h263_profile[] = {
	"Baseline",
	"H320 Coding",
	"Backward Compatible",
	"ISWV2",
	"ISWV3",
	"High Compression",
	"Internet",
	"Interlace",
	"High Latency",
};

static const char *const vp8_profile_level[] = {
	"Unused",
	"0.0",
	"1.0",
	"2.0",
	"3.0",
};

static const char *const mpeg2_profile[] = {
	"Simple",
	"Main",
	"422",
	"SNR scalable",
	"Spatial scalable",
	"High",
};

static const char *const mpeg2_level[] = {
	"Level 0",
	"Level 1",
	"Level 2",
	"Level 3",
};

static const char *const mpeg_vidc_video_h264_mvc_layout[] = {
	"Frame packing arrangement sequential",
	"Frame packing arrangement top-bottom",
};

static struct vidc_ctrl vdec_ctrls[] = {
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_STREAM_FORMAT,
		.name = "NAL Format",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_STARTCODES,
		.maximum = V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_FOUR_BYTE_LENGTH,
		.default_value = V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_STARTCODES,
		.menu_skip_mask = ~(
		(1 << V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_STARTCODES) |
		(1 << V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_ONE_NAL_PER_BUFFER) |
		(1 << V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_ONE_BYTE_LENGTH) |
		(1 << V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_TWO_BYTE_LENGTH) |
		(1 << V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_FOUR_BYTE_LENGTH)
		),
		.qmenu = mpeg_video_stream_format,
		.step = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_OUTPUT_ORDER,
		.name = "Output Order",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_OUTPUT_ORDER_DISPLAY,
		.maximum = V4L2_MPEG_VIDC_VIDEO_OUTPUT_ORDER_DECODE,
		.default_value = V4L2_MPEG_VIDC_VIDEO_OUTPUT_ORDER_DISPLAY,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDC_VIDEO_OUTPUT_ORDER_DISPLAY) |
			(1 << V4L2_MPEG_VIDC_VIDEO_OUTPUT_ORDER_DECODE)
			),
		.qmenu = mpeg_video_output_order,
		.step = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_PICTURE_TYPE,
		.name = "Picture Type Decoding",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 15,
		.default_value = 15,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_KEEP_ASPECT_RATIO,
		.name = "Keep Aspect Ratio",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_POST_LOOP_DEBLOCKER_MODE,
		.name = "Deblocker Mode",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_DIVX_FORMAT,
		.name = "Divx Format",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_DIVX_FORMAT_4,
		.maximum = V4L2_MPEG_VIDC_VIDEO_DIVX_FORMAT_6,
		.default_value = V4L2_MPEG_VIDC_VIDEO_DIVX_FORMAT_4,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDC_VIDEO_DIVX_FORMAT_4) |
			(1 << V4L2_MPEG_VIDC_VIDEO_DIVX_FORMAT_5) |
			(1 << V4L2_MPEG_VIDC_VIDEO_DIVX_FORMAT_6)
			),
		.qmenu = mpeg_video_vidc_divx_format,
		.step = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_MB_ERROR_MAP_REPORTING,
		.name = "MB Error Map Reporting",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_CONTINUE_DATA_TRANSFER,
		.name = "Smooth streamng",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_SYNC_FRAME_DECODE,
		.name = "Sync Frame Decode",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = V4L2_MPEG_VIDC_VIDEO_SYNC_FRAME_DECODE_DISABLE,
		.maximum = V4L2_MPEG_VIDC_VIDEO_SYNC_FRAME_DECODE_ENABLE,
		.default_value = V4L2_MPEG_VIDC_VIDEO_SYNC_FRAME_DECODE_DISABLE,
		.step = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_SECURE,
		.name = "Secure mode",
		.type = V4L2_CTRL_TYPE_BUTTON,
		.minimum = 0,
		.maximum = 0,
		.default_value = 0,
		.step = 0,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_EXTRADATA,
		.name = "Extradata Type",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_EXTRADATA_NONE,
		.maximum = V4L2_MPEG_VIDC_EXTRADATA_FRAME_BITS_INFO,
		.default_value = V4L2_MPEG_VIDC_EXTRADATA_NONE,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDC_EXTRADATA_NONE) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_MB_QUANTIZATION) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_INTERLACE_VIDEO) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_VC1_FRAMEDISP) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_VC1_SEQDISP) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_TIMESTAMP) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_S3D_FRAME_PACKING) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_FRAME_RATE) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_PANSCAN_WINDOW) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_RECOVERY_POINT_SEI) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_MULTISLICE_INFO) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_NUM_CONCEALED_MB) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_METADATA_FILLER) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_INPUT_CROP) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_DIGITAL_ZOOM) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_ASPECT_RATIO) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_MPEG2_SEQDISP) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_STREAM_USERDATA) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_FRAME_QP) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_FRAME_BITS_INFO)
			),
		.qmenu = mpeg_video_vidc_extradata,
		.step = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_ALLOC_MODE_INPUT,
		.name = "Buffer allocation mode for input",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_STATIC,
		.maximum = V4L2_MPEG_VIDC_VIDEO_DYNAMIC,
		.default_value = V4L2_MPEG_VIDC_VIDEO_STATIC,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDC_VIDEO_STATIC) |
			(1 << V4L2_MPEG_VIDC_VIDEO_RING) |
			(1 << V4L2_MPEG_VIDC_VIDEO_DYNAMIC)
			),
		.qmenu = mpeg_vidc_video_alloc_mode_type,
		.step = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_ALLOC_MODE_OUTPUT,
		.name = "Buffer allocation mode for output",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_STATIC,
		.maximum = V4L2_MPEG_VIDC_VIDEO_DYNAMIC,
		.default_value = V4L2_MPEG_VIDC_VIDEO_STATIC,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDC_VIDEO_STATIC) |
			(1 << V4L2_MPEG_VIDC_VIDEO_RING) |
			(1 << V4L2_MPEG_VIDC_VIDEO_DYNAMIC)
			),
		.qmenu = mpeg_vidc_video_alloc_mode_type,
		.step = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_FRAME_ASSEMBLY,
		.name = "Video frame assembly",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = V4L2_MPEG_VIDC_FRAME_ASSEMBLY_DISABLE,
		.maximum = V4L2_MPEG_VIDC_FRAME_ASSEMBLY_ENABLE,
		.default_value =  V4L2_MPEG_VIDC_FRAME_ASSEMBLY_DISABLE,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_MODE,
		.name = "Video decoder multi stream",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_PRIMARY,
		.maximum = V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_SECONDARY,
		.default_value = V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_PRIMARY,
		.menu_skip_mask = 0,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE,
		.name = "MPEG4 Profile",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE,
		.maximum =
		V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_CODING_EFFICIENCY,
		.default_value = V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE,
		.menu_skip_mask = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL,
		.name = "MPEG4 Level",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_MPEG4_LEVEL_0,
		.maximum = V4L2_MPEG_VIDEO_MPEG4_LEVEL_5,
		.default_value = V4L2_MPEG_VIDEO_MPEG4_LEVEL_0,
		.menu_skip_mask = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.name = "H264 Profile",
		.type = V4L2_CTRL_TYPE_MENU,
		.maximum = V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH,
		.default_value = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.menu_skip_mask = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.name = "H264 Level",
		.type = V4L2_CTRL_TYPE_MENU,
		.maximum = V4L2_MPEG_VIDEO_H264_LEVEL_5_2,
		.default_value = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.menu_skip_mask = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_H263_PROFILE,
		.name = "H263 Profile",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_BASELINE,
		.maximum = V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_HIGHLATENCY,
		.default_value = V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_BASELINE,
		.menu_skip_mask = ~(
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_BASELINE) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_H320CODING) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_BACKWARDCOMPATIBLE) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_ISWV2) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_ISWV3) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_HIGHCOMPRESSION) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_INTERNET) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_INTERLACE) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_HIGHLATENCY)
		),
		.qmenu = h263_profile,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_H263_LEVEL,
		.name = "H263 Level",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_1_0,
		.maximum = V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_7_0,
		.default_value = V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_1_0,
		.menu_skip_mask = ~(
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_1_0) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_2_0) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_3_0) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_4_0) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_5_0) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_6_0) |
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_7_0)
		),
		.qmenu = h263_level,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_VP8_PROFILE_LEVEL,
		.name = "VP8 Profile Level",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_VP8_UNUSED,
		.maximum = V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_1,
		.default_value = V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_0,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDC_VIDEO_VP8_UNUSED) |
			(1 << V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_0) |
			(1 << V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_1)
		),
		.qmenu = vp8_profile_level,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_MPEG2_PROFILE,
		.name = "MPEG2 Profile",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_MPEG2_PROFILE_SIMPLE,
		.maximum = V4L2_MPEG_VIDC_VIDEO_MPEG2_PROFILE_HIGH,
		.default_value = V4L2_MPEG_VIDC_VIDEO_MPEG2_PROFILE_SIMPLE,
		.menu_skip_mask = 0,
		.qmenu = mpeg2_profile,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_MPEG2_LEVEL,
		.name = "MPEG2 Level",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_MPEG2_LEVEL_0,
		.maximum = V4L2_MPEG_VIDC_VIDEO_MPEG2_LEVEL_3,
		.default_value = V4L2_MPEG_VIDC_VIDEO_MPEG2_LEVEL_0,
		.menu_skip_mask = 0,
		.qmenu = mpeg2_level,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_SCS_THRESHOLD,
		.name = "Video start code search threshold",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = INT_MAX,
		.default_value = INT_MAX,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_MVC_BUFFER_LAYOUT,
		.name = "MVC buffer layout",
		.type = V4L2_CTRL_TYPE_MENU,
		.maximum = V4L2_MPEG_VIDC_VIDEO_MVC_TOP_BOTTOM,
		.default_value = V4L2_MPEG_VIDC_VIDEO_MVC_SEQUENTIAL,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDC_VIDEO_MVC_SEQUENTIAL) |
			(1 << V4L2_MPEG_VIDC_VIDEO_MVC_TOP_BOTTOM)
			),
		.qmenu = mpeg_vidc_video_h264_mvc_layout,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_CONCEAL_COLOR,
		.name = "Picture concealed color",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0x0,
		.maximum = 0xffffff,
		.default_value = DEFAULT_VIDEO_CONCEAL_COLOR_BLACK,
		.step = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_SECURE_SCALING_THRESHOLD,
		.name = "Secure scaling output2 threshold",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = INT_MAX,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_NON_SECURE_OUTPUT2,
		.name = "Non-Secure output2",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
};

#define NUM_CTRLS	ARRAY_SIZE(vdec_ctrls)

#define IS_PRIV_CTRL(idx)	( \
		(V4L2_CTRL_ID2CLASS(idx) == V4L2_CTRL_CLASS_MPEG) && \
		V4L2_CTRL_DRIVER_PRIV(idx))

static int check_tz_dynamic_buffer_support(struct vidc_inst *inst)
{
	struct device *dev = &inst->core->res.pdev->dev;
	int version = qcom_scm_get_feat_version(TZ_DYNAMIC_BUFFER_FEATURE_ID);

	/*
	 * if the version is < 1.1.0 then dynamic buffer allocation is
	 * not supported
	 */
	if (version < TZ_FEATURE_VERSION(1, 1, 0)) {
		dev_dbg(dev, "dynamic buffer mode not supported, "
			"tz version is %u vs required %u\n",
			version, TZ_FEATURE_VERSION(1, 1, 0));
		return -ENOTSUPP;
	}

	return 0;
}

static enum hal_buffer_mode_type get_buf_type(int val)
{
	switch (val) {
	case V4L2_MPEG_VIDC_VIDEO_STATIC:
		return HAL_BUFFER_MODE_STATIC;
	case V4L2_MPEG_VIDC_VIDEO_RING:
		return HAL_BUFFER_MODE_RING;
	case V4L2_MPEG_VIDC_VIDEO_DYNAMIC:
		return HAL_BUFFER_MODE_DYNAMIC;
	default:
		break;
	}

	return 0;
}

static int is_ctrl_valid_for_codec(struct vidc_inst *inst,
				   struct v4l2_ctrl *ctrl)
{
	struct device *dev = &inst->core->res.pdev->dev;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDC_VIDEO_MVC_BUFFER_LAYOUT:
		if (inst->fmt_out->pixfmt != V4L2_PIX_FMT_H264_MVC) {
			dev_err(dev, "control %x only valid for MVC\n",
				ctrl->id);
			ret = -ENOTSUPP;
			break;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		if (inst->fmt_out->pixfmt == V4L2_PIX_FMT_H264_MVC &&
			ctrl->val != V4L2_MPEG_VIDEO_H264_PROFILE_STEREO_HIGH) {
			dev_err(dev, "Profile %x not supported for MVC\n",
				ctrl->val);
			ret = -ENOTSUPP;
			break;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		if (inst->fmt_out->pixfmt == V4L2_PIX_FMT_H264_MVC &&
			ctrl->val >= V4L2_MPEG_VIDEO_H264_LEVEL_5_2) {
			dev_err(dev, "Level %x not supported for MVC\n",
				ctrl->val);
			ret = -ENOTSUPP;
			break;
		}
		break;
	default:
		break;
	}

	return ret;
}

static int try_get_ctrl(struct vidc_inst *inst, struct v4l2_ctrl *ctrl)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct hal_profile_level profile_level;
	union hal_get_property hprop;
	int ret = 0;

	dev_dbg(dev, "get ctrl id: %x\n", ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE:
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
	case V4L2_CID_MPEG_VIDC_VIDEO_H263_PROFILE:
	case V4L2_CID_MPEG_VIDC_VIDEO_VP8_PROFILE_LEVEL:
	case V4L2_CID_MPEG_VIDC_VIDEO_MPEG2_PROFILE:
		ret = hfi_session_get_property(inst,
					       HAL_PARAM_PROFILE_LEVEL_CURRENT,
					       &hprop);
		if (ret) {
			dev_err(dev, "getting profile property (%d)\n", ret);
			return ret;
		}

		profile_level = hprop.profile_level;
		ctrl->val = profile_level.profile;
		dev_dbg(dev, "PROFILE ctrl->id:%x ctrl->val:%d\n",
			ctrl->id, ctrl->val);
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL:
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
	case V4L2_CID_MPEG_VIDC_VIDEO_H263_LEVEL:
	case V4L2_CID_MPEG_VIDC_VIDEO_MPEG2_LEVEL:
		ret = hfi_session_get_property(inst,
					       HAL_PARAM_PROFILE_LEVEL_CURRENT,
					       &hprop);
		if (ret) {
			dev_err(dev, "getting level property(%d)\n", ret);
			return ret;
		}

		profile_level = hprop.profile_level;
		ctrl->val = profile_level.level;
		dev_dbg(dev, "LEVEL ctrl->id:%x ctrl->val:%d\n",
			ctrl->id, ctrl->val);
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_SECURE_SCALING_THRESHOLD:
		if (!(inst->flags & VIDC_SECURE) ||
		    !inst->capability.secure_output2_threshold.max) {
			dprintk(VIDC_ERR, "%s id:%x invalid configuration\n",
				__func__, ctrl->id);
			ret = -EINVAL;
			break;
		}
		dev_dbg(dev, "Secure Scaling Threshold is %d",
			inst->capability.secure_output2_threshold.max);
		ctrl->val = inst->capability.secure_output2_threshold.max;
		break;
	default:
		dev_dbg(dev, "ctrl id: %x not supported\n", ctrl->id);
		break;
	}

	return ret;
}

static int vdec_v4l2_to_hal(int id, int value)
{
	switch (id) {
		/* H264 */
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		switch (value) {
		case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
			return HAL_H264_PROFILE_BASELINE;
		case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
			return HAL_H264_PROFILE_CONSTRAINED_BASE;
		case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
			return HAL_H264_PROFILE_MAIN;
		case V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED:
			return HAL_H264_PROFILE_EXTENDED;
		case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
			return HAL_H264_PROFILE_HIGH;
		case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10:
			return HAL_H264_PROFILE_HIGH10;
		case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422:
			return HAL_H264_PROFILE_HIGH422;
		case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE:
			return HAL_H264_PROFILE_HIGH444;
		default:
			return -EINVAL;
		}
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		switch (value) {
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_0:
			return HAL_H264_LEVEL_1;
		case V4L2_MPEG_VIDEO_H264_LEVEL_1B:
			return HAL_H264_LEVEL_1b;
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_1:
			return HAL_H264_LEVEL_11;
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_2:
			return HAL_H264_LEVEL_12;
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_3:
			return HAL_H264_LEVEL_13;
		case V4L2_MPEG_VIDEO_H264_LEVEL_2_0:
			return HAL_H264_LEVEL_2;
		case V4L2_MPEG_VIDEO_H264_LEVEL_2_1:
			return HAL_H264_LEVEL_21;
		case V4L2_MPEG_VIDEO_H264_LEVEL_2_2:
			return HAL_H264_LEVEL_22;
		case V4L2_MPEG_VIDEO_H264_LEVEL_3_0:
			return HAL_H264_LEVEL_3;
		case V4L2_MPEG_VIDEO_H264_LEVEL_3_1:
			return HAL_H264_LEVEL_31;
		case V4L2_MPEG_VIDEO_H264_LEVEL_3_2:
			return HAL_H264_LEVEL_32;
		case V4L2_MPEG_VIDEO_H264_LEVEL_4_0:
			return HAL_H264_LEVEL_4;
		case V4L2_MPEG_VIDEO_H264_LEVEL_4_1:
			return HAL_H264_LEVEL_41;
		case V4L2_MPEG_VIDEO_H264_LEVEL_4_2:
			return HAL_H264_LEVEL_42;
		case V4L2_MPEG_VIDEO_H264_LEVEL_5_0:
			return HAL_H264_LEVEL_5;
		case V4L2_MPEG_VIDEO_H264_LEVEL_5_1:
			return HAL_H264_LEVEL_51;
		default:
			return -EINVAL;
		}
	}

	return -EINVAL;
}

static struct v4l2_ctrl *
get_ctrl_from_cluster(int id, struct v4l2_ctrl **cluster, int ncontrols)
{
	int c;

	for (c = 0; c < ncontrols; ++c)
		if (cluster[c]->id == id)
			return cluster[c];

	return NULL;
}

/* Small helper macro for quickly getting a control and err checking */
#define TRY_GET_CTRL(__ctrl_id) ({ \
		struct v4l2_ctrl *__temp; \
		__temp = get_ctrl_from_cluster( \
			__ctrl_id, \
			ctrl->cluster, ctrl->ncontrols); \
		if (!__temp) { \
			dprintk(VIDC_ERR, "Can't find %s (%x) in cluster\n", \
				#__ctrl_id, __ctrl_id); \
			/* Clusters are hardcoded, if we can't find */ \
			/* something then things are massively screwed up */ \
			BUG_ON(1); \
		} \
		__temp; \
	})

static int try_set_ctrl(struct vidc_inst *inst, struct v4l2_ctrl *ctrl)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct hal_nal_stream_format_supported stream_format;
	struct hal_enable_picture enable_picture;
	struct hal_enable hal_property;
	struct hal_extradata_enable extra;
	struct hal_buffer_alloc_mode alloc_mode;
	struct hal_multi_stream multi_stream;
	struct hfi_scs_threshold scs_threshold;
	struct hal_mvc_buffer_layout layout;
	struct v4l2_ctrl *temp_ctrl = NULL;
	struct hal_profile_level profile_level;
	struct hal_frame_size frame_sz;
	enum hal_property prop_id = 0;
	u32 property_val = 0;
	void *pdata = NULL;
	int ret = 0;

	ret = is_ctrl_valid_for_codec(inst, ctrl);
	if (ret)
		return ret;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDC_VIDEO_STREAM_FORMAT:
		prop_id = HAL_PARAM_NAL_STREAM_FORMAT_SELECT;
		stream_format.nal_stream_format_supported =
					(0x00000001 << ctrl->val);
		pdata = &stream_format;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_OUTPUT_ORDER:
		prop_id = HAL_PARAM_VDEC_OUTPUT_ORDER;
		property_val = ctrl->val;
		pdata = &property_val;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_PICTURE_TYPE:
		prop_id = HAL_PARAM_VDEC_PICTURE_TYPE_DECODE;
		enable_picture.picture_type = ctrl->val;
		pdata = &enable_picture;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_KEEP_ASPECT_RATIO:
		prop_id = HAL_PARAM_VDEC_OUTPUT2_KEEP_ASPECT_RATIO;
		hal_property.enable = ctrl->val;
		pdata = &hal_property;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_POST_LOOP_DEBLOCKER_MODE:
		prop_id = HAL_CONFIG_VDEC_POST_LOOP_DEBLOCKER;
		hal_property.enable = ctrl->val;
		pdata = &hal_property;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_DIVX_FORMAT:
		prop_id = HAL_PARAM_DIVX_FORMAT;
		property_val = ctrl->val;
		pdata = &property_val;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_MB_ERROR_MAP_REPORTING:
		prop_id = HAL_CONFIG_VDEC_MB_ERROR_MAP_REPORTING;
		hal_property.enable = ctrl->val;
		pdata = &hal_property;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_CONTINUE_DATA_TRANSFER:
		prop_id = HAL_PARAM_VDEC_CONTINUE_DATA_TRANSFER;
		hal_property.enable = ctrl->val;
		pdata = &hal_property;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_SYNC_FRAME_DECODE:
		switch (ctrl->val) {
		case V4L2_MPEG_VIDC_VIDEO_SYNC_FRAME_DECODE_DISABLE:
			inst->flags &= ~VIDC_THUMBNAIL;
			break;
		case V4L2_MPEG_VIDC_VIDEO_SYNC_FRAME_DECODE_ENABLE:
			inst->flags |= VIDC_THUMBNAIL;
			break;
		}

		prop_id = HAL_PARAM_VDEC_SYNC_FRAME_DECODE;
		hal_property.enable = ctrl->val;
		pdata = &hal_property;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_SECURE:
		inst->flags |= VIDC_SECURE;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_EXTRADATA:
		prop_id = HAL_PARAM_INDEX_EXTRADATA;
		extra.index = vidc_comm_get_hal_extradata_index(ctrl->val);
		extra.enable = 1;
		pdata = &extra;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_ALLOC_MODE_INPUT:
		if (ctrl->val == V4L2_MPEG_VIDC_VIDEO_DYNAMIC) {
			ret = -ENOTSUPP;
			break;
		}
		prop_id = HAL_PARAM_BUFFER_ALLOC_MODE;
		alloc_mode.mode = get_buf_type(ctrl->val);
		alloc_mode.type = HAL_BUFFER_INPUT;
		inst->buffer_mode[OUTPUT_PORT] = alloc_mode.mode;
		pdata = &alloc_mode;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_FRAME_ASSEMBLY:
		prop_id = HAL_PARAM_VDEC_FRAME_ASSEMBLY;
		hal_property.enable = ctrl->val;
		pdata = &hal_property;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_ALLOC_MODE_OUTPUT:
		prop_id = HAL_PARAM_BUFFER_ALLOC_MODE;
		alloc_mode.mode = get_buf_type(ctrl->val);

		if (!(alloc_mode.mode &
			inst->capability.buffer_mode[CAPTURE_PORT])) {
			ret = -ENOTSUPP;
			break;
		}

		if (alloc_mode.mode == HAL_BUFFER_MODE_DYNAMIC &&
		    (inst->flags & VIDC_SECURE) &&
		    check_tz_dynamic_buffer_support(inst)) {
				ret = -ENOTSUPP;
				break;
		}

		alloc_mode.type = HAL_BUFFER_OUTPUT;
		pdata = &alloc_mode;
		inst->buffer_mode[CAPTURE_PORT] = alloc_mode.mode;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_MODE:
		if (ctrl->val && !(inst->capability.pixelprocess_capabilities &
		    HAL_VIDEO_DECODER_MULTI_STREAM_CAPABILITY)) {
			dev_err(dev, "Downscaling not supported: %#x\n",
				ctrl->id);
			ret = -ENOTSUPP;
			break;
		}
		switch (ctrl->val) {
		case V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_PRIMARY:
			multi_stream.buffer_type = HAL_BUFFER_OUTPUT;
			multi_stream.enable = true;
			pdata = &multi_stream;
			prop_id = HAL_PARAM_VDEC_MULTI_STREAM;

			ret = hfi_session_set_property(inst, prop_id, pdata);
			if (ret) {
				dev_err(dev, "enabling OUTPUT port (%d)\n",
					ret);
				break;
			}
			multi_stream.buffer_type = HAL_BUFFER_OUTPUT2;
			multi_stream.enable = false;
			pdata = &multi_stream;

			ret = hfi_session_set_property(inst, prop_id, pdata);
			if (ret)
				dev_err(dev, "disabling OUTPUT2 port (%d)\n",
					ret);
			break;
		case V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_SECONDARY:
			multi_stream.buffer_type = HAL_BUFFER_OUTPUT2;
			multi_stream.enable = true;
			pdata = &multi_stream;
			prop_id = HAL_PARAM_VDEC_MULTI_STREAM;

			ret = hfi_session_set_property(inst, prop_id, pdata);
			if (ret) {
				dev_err(dev, "enabling OUTPUT2 port (%d)\n",
					ret);
					break;
			}
			multi_stream.buffer_type = HAL_BUFFER_OUTPUT;
			multi_stream.enable = false;
			pdata = &multi_stream;

			ret = hfi_session_set_property(inst, prop_id, pdata);
			if (ret) {
				dev_err(dev, "disabling OUTPUT port (%d)\n",
					ret);
				break;
			}

			frame_sz.buffer_type = HAL_BUFFER_OUTPUT2;
			frame_sz.width = inst->width;
			frame_sz.height = inst->height;
			pdata = &frame_sz;
			prop_id = HAL_PARAM_FRAME_SIZE;

			dev_dbg(dev, "buftype: %d width: %d, height: %d\n",
				frame_sz.buffer_type, frame_sz.width,
				frame_sz.height);

			ret = hfi_session_set_property(inst, prop_id, pdata);
			if (ret)
				dev_err(dev, "setting OUTPUT2 size (%d)\n",
					ret);
			break;
		default:
			dev_err(dev, "unsupported multi stream setting\n");
			ret = -ENOTSUPP;
			break;
		}
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_SCS_THRESHOLD:
		prop_id = HAL_PARAM_VDEC_SCS_THRESHOLD;
		scs_threshold.threshold_value = ctrl->val;
		pdata = &scs_threshold;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_MVC_BUFFER_LAYOUT:
		prop_id = HAL_PARAM_MVC_BUFFER_LAYOUT;
		layout.layout_type = vidc_comm_get_hal_buffer_layout(ctrl->val);
		layout.bright_view_first = 0;
		layout.ngap = 0;
		pdata = &layout;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_CONCEAL_COLOR:
		prop_id = HAL_PARAM_VDEC_CONCEAL_COLOR;
		property_val = ctrl->val;
		pdata = &property_val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_LEVEL);
		prop_id = HAL_PARAM_PROFILE_LEVEL_CURRENT;
		profile_level.profile = vdec_v4l2_to_hal(ctrl->id, ctrl->val);
		profile_level.level =
			vdec_v4l2_to_hal(V4L2_CID_MPEG_VIDEO_H264_LEVEL,
					 temp_ctrl->val);
		pdata = &profile_level;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_PROFILE);
		prop_id = HAL_PARAM_PROFILE_LEVEL_CURRENT;
		profile_level.level = vdec_v4l2_to_hal(ctrl->id, ctrl->val);
		profile_level.profile =
			vdec_v4l2_to_hal(V4L2_CID_MPEG_VIDEO_H264_PROFILE,
					 temp_ctrl->val);
		pdata = &profile_level;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_NON_SECURE_OUTPUT2:
		prop_id = HAL_PARAM_VDEC_NON_SECURE_OUTPUT2;
		hal_property.enable = ctrl->val;
		dev_dbg(dev, "%s non_secure output2\n",
			ctrl->val ? "Enabling" : "Disabling");
		pdata = &hal_property;
		break;
	default:
		break;
	}

	if (!ret && prop_id) {
		dev_dbg(dev,
			"Control: HAL property=%x, ctrl: id=%x, value=%x\n",
			prop_id, ctrl->id, ctrl->val);

		ret = hfi_session_set_property(inst, prop_id, pdata);
	}

	return ret;
}

static int vdec_op_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vidc_inst *inst =
		container_of(ctrl->handler, struct vidc_inst, ctrl_handler);
	struct device *dev = &inst->core->res.pdev->dev;
	int ret = 0, c = 0;

	for (c = 0; c < ctrl->ncontrols; ++c) {
		if (ctrl->cluster[c]->is_new) {
			ret = try_set_ctrl(inst, ctrl->cluster[c]);
			if (ret) {
				dprintk(VIDC_ERR, "Failed setting %x\n",
					ctrl->cluster[c]->id);
				break;
			}
		}
	}

	if (ret)
		dev_err(dev, "setting control: %x (%s)",
			ctrl->id, v4l2_ctrl_get_name(ctrl->id));

	return ret;
}

static int vdec_op_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vidc_inst *inst =
		container_of(ctrl->handler, struct vidc_inst, ctrl_handler);
	struct device *dev = &inst->core->res.pdev->dev;
	struct v4l2_ctrl *master = ctrl->cluster[0];
	int ret, c;

	for (c = 0; c < master->ncontrols; ++c) {
		int d;

		for (d = 0; d < NUM_CTRLS; ++d) {
			if (master->cluster[c]->id == inst->ctrls[d]->id &&
			    inst->ctrls[d]->flags & V4L2_CTRL_FLAG_VOLATILE) {
				ret = try_get_ctrl(inst, master->cluster[c]);
				if (ret) {
					dev_err(dev, "getting %x\n",
						master->cluster[c]->id);
					return ret;
				}
				break;
			}
		}
	}

	return ret;
}

static const struct v4l2_ctrl_ops vdec_ctrl_ops = {
	.s_ctrl = vdec_op_s_ctrl,
	.g_volatile_ctrl = vdec_op_g_volatile_ctrl,
};

static struct v4l2_ctrl **get_super_cluster(struct vidc_inst *inst, int *size)
{
	int c = 0, sz = 0;
	struct v4l2_ctrl **cluster = kmalloc(sizeof(struct v4l2_ctrl *) *
					     NUM_CTRLS, GFP_KERNEL);

	if (!size || !cluster || !inst)
		return NULL;

	for (c = 0; c < NUM_CTRLS; c++)
		cluster[sz++] = inst->ctrls[c];

	*size = sz;
	return cluster;
}

int vdec_ctrl_init(struct vidc_inst *inst)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct v4l2_ctrl_config cfg;
	int cluster_size = 0;
	int idx = 0;
	int ret = 0;

	inst->ctrls = kzalloc(sizeof(struct v4l2_ctrl *) * NUM_CTRLS,
			      GFP_KERNEL);
	if (!inst->ctrls)
		return -ENOMEM;

	ret = v4l2_ctrl_handler_init(&inst->ctrl_handler, NUM_CTRLS);
	if (ret) {
		dev_err(dev, "control handler init (%d)\n", ret);
		return ret;
	}

	for (; idx < NUM_CTRLS; idx++) {
		struct v4l2_ctrl *ctrl = NULL;

		if (IS_PRIV_CTRL(vdec_ctrls[idx].id)) {
			memset(&cfg, 0, sizeof(cfg));
			cfg.def = vdec_ctrls[idx].default_value;
			cfg.flags = 0;
			cfg.id = vdec_ctrls[idx].id;
			/* cfg.is_private = vdec_ctrls[idx].is_private;
			 * cfg.is_volatile = vdec_ctrls[idx].is_volatile;
			 */
			cfg.max = vdec_ctrls[idx].maximum;
			cfg.min = vdec_ctrls[idx].minimum;
			cfg.menu_skip_mask = vdec_ctrls[idx].menu_skip_mask;
			cfg.name = vdec_ctrls[idx].name;
			cfg.ops = &vdec_ctrl_ops;
			cfg.step = vdec_ctrls[idx].step;
			cfg.type = vdec_ctrls[idx].type;
			cfg.qmenu = vdec_ctrls[idx].qmenu;

			ctrl = v4l2_ctrl_new_custom(&inst->ctrl_handler, &cfg,
						    NULL);
		} else {
			if (vdec_ctrls[idx].type == V4L2_CTRL_TYPE_MENU) {
				ctrl = v4l2_ctrl_new_std_menu(
					&inst->ctrl_handler,
					&vdec_ctrl_ops,
					vdec_ctrls[idx].id,
					vdec_ctrls[idx].maximum,
					vdec_ctrls[idx].menu_skip_mask,
					vdec_ctrls[idx].default_value);
			} else {
				ctrl = v4l2_ctrl_new_std(&inst->ctrl_handler,
					&vdec_ctrl_ops,
					vdec_ctrls[idx].id,
					vdec_ctrls[idx].minimum,
					vdec_ctrls[idx].maximum,
					vdec_ctrls[idx].step,
					vdec_ctrls[idx].default_value);
			}
		}

		if (!ctrl) {
			dev_err(dev, "invalid ctrl\n");
			return -EINVAL;
		}

		switch (vdec_ctrls[idx].id) {
		case V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE:
		case V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL:
		case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		case V4L2_CID_MPEG_VIDC_VIDEO_H263_PROFILE:
		case V4L2_CID_MPEG_VIDC_VIDEO_H263_LEVEL:
		case V4L2_CID_MPEG_VIDC_VIDEO_VP8_PROFILE_LEVEL:
		case V4L2_CID_MPEG_VIDC_VIDEO_SECURE_SCALING_THRESHOLD:
			ctrl->flags |= vdec_ctrls[idx].flags;
			break;
		}

		ret = inst->ctrl_handler.error;
		if (ret) {
			dev_err(dev, "adding ctrl (%s) to ctrl handle (%d)\n",
				vdec_ctrls[idx].name,
				inst->ctrl_handler.error);
			return ret;
		}

		inst->ctrls[idx] = ctrl;
	}

	/* Construct a super cluster of all controls */
	inst->cluster = get_super_cluster(inst, &cluster_size);
	if (!inst->cluster || !cluster_size) {
		dev_err(dev, "setup super cluster\n");
		return -EINVAL;
	}

	v4l2_ctrl_cluster(cluster_size, inst->cluster);

	return ret;
}

void vdec_ctrl_deinit(struct vidc_inst *inst)
{
	kfree(inst->ctrls);
	kfree(inst->cluster);
	v4l2_ctrl_handler_free(&inst->ctrl_handler);
}
