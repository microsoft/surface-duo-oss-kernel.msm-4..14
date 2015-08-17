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
#include "msm_venc.h"

#define MIN_BIT_RATE		32000
#define MAX_BIT_RATE		160000000
#define DEFAULT_BIT_RATE	64000
#define BIT_RATE_STEP		100
#define DEFAULT_FRAME_RATE	15
#define MAX_SLICE_BYTE_SIZE	1024
#define MIN_SLICE_BYTE_SIZE	1024
#define MAX_SLICE_MB_SIZE	300
#define I_FRAME_QP		26
#define P_FRAME_QP		28
#define B_FRAME_QP		30
#define MAX_INTRA_REFRESH_MBS	300
#define MAX_NUM_B_FRAMES	4
#define MAX_LTR_FRAME_COUNT	10

#define L_MODE	\
		V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY
#define CODING	\
		V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_CODING_EFFICIENCY
#define BITSTREAM_RESTRICT_ENABLED	\
		V4L2_MPEG_VIDC_VIDEO_H264_VUI_BITSTREAM_RESTRICT_ENABLED
#define BITSTREAM_RESTRICT_DISABLED	\
	V4L2_MPEG_VIDC_VIDEO_H264_VUI_BITSTREAM_RESTRICT_DISABLED

#define MIN_TIME_RESOLUTION		1
#define MAX_TIME_RESOLUTION		0xffffff
#define DEFAULT_TIME_RESOLUTION		0x7530

#define IS_PRIV_CTRL(idx)	( \
		(V4L2_CTRL_ID2CLASS(idx) == V4L2_CTRL_CLASS_MPEG) && \
		V4L2_CTRL_DRIVER_PRIV(idx))

static const char *const mpeg_video_rate_control[] = {
	"No Rate Control",
	"VBR VFR",
	"VBR CFR",
	"CBR VFR",
	"CBR CFR",
	NULL
};

static const char *const mpeg_video_rotation[] = {
	"No Rotation",
	"90 Degree Rotation",
	"180 Degree Rotation",
	"270 Degree Rotation",
	NULL
};

static const char *const h264_video_entropy_cabac_model[] = {
	"Model 0",
	"Model 1",
	"Model 2",
	NULL
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

static const char *const hevc_tier_level[] = {
	"Main Tier Level 1",
	"Main Tier Level 2",
	"Main Tier Level 2.1",
	"Main Tier Level 3",
	"Main Tier Level 3.1",
	"Main Tier Level 4",
	"Main Tier Level 4.1",
	"Main Tier Level 5",
	"Main Tier Level 5.1",
	"Main Tier Level 5.2",
	"Main Tier Level 6",
	"Main Tier Level 6.1",
	"Main Tier Level 6.2",
	"High Tier Level 1",
	"High Tier Level 2",
	"High Tier Level 2.1",
	"High Tier Level 3",
	"High Tier Level 3.1",
	"High Tier Level 4",
	"High Tier Level 4.1",
	"High Tier Level 5",
	"High Tier Level 5.1",
	"High Tier Level 5.2",
	"High Tier Level 6",
	"High Tier Level 6.1",
	"High Tier Level 6.2",
};

static const char *const hevc_profile[] = {
	"Main",
	"Main10",
	"Main Still Pic",
};

static const char *const vp8_profile_level[] = {
	"Unused",
	"0.0",
	"1.0",
	"2.0",
	"3.0",
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
	"Extradata macroblock metadata",
};

static const char *const perf_level[] = {
	"Nominal",
	"Performance",
	"Turbo"
};

static const char *const intra_refresh_modes[] = {
	"None",
	"Cyclic",
	"Adaptive",
	"Cyclic Adaptive",
	"Random"
};

static const char *const timestamp_mode[] = {
	"Honor",
	"Ignore",
};

static struct vidc_ctrl venc_ctrls[] = {
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_IDR_PERIOD,
		.name = "IDR Period",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = INT_MAX,
		.default_value = DEFAULT_FRAME_RATE,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_NUM_P_FRAMES,
		.name = "Intra Period for P frames",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = INT_MAX,
		.default_value = 2 * DEFAULT_FRAME_RATE - 1,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_NUM_B_FRAMES,
		.name = "Intra Period for B frames",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = INT_MAX,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_REQUEST_IFRAME,
		.name = "Request I Frame",
		.type = V4L2_CTRL_TYPE_BUTTON,
		.minimum = 0,
		.maximum = 0,
		.default_value = 0,
		.step = 0,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL,
		.name = "Video Framerate and Bitrate Control",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_OFF,
		.maximum = V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_CFR,
		.default_value = V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_OFF,
		.step = 0,
		.menu_skip_mask = ~(
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_OFF) |
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_VFR) |
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_CFR) |
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_VFR) |
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_CFR)),
		.qmenu = mpeg_video_rate_control,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
		.name = "Bitrate Control",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR,
		.maximum = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR,
		.default_value = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR,
		.step = 0,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) |
			(1 << V4L2_MPEG_VIDEO_BITRATE_MODE_CBR)),
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_BITRATE,
		.name = "Bit Rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = MIN_BIT_RATE,
		.maximum = MAX_BIT_RATE,
		.default_value = DEFAULT_BIT_RATE,
		.step = BIT_RATE_STEP,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_BITRATE_PEAK,
		.name = "Peak Bit Rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = MIN_BIT_RATE,
		.maximum = MAX_BIT_RATE,
		.default_value = DEFAULT_BIT_RATE,
		.step = BIT_RATE_STEP,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE,
		.name = "Entropy Mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC,
		.maximum = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC,
		.default_value = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC,
		.step = 0,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC) |
			(1 << V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC)),
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL,
		.name = "CABAC Model",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_0,
		.maximum = V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_1,
		.default_value = V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_0,
		.step = 0,
		.menu_skip_mask = ~(
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_0) |
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_1) |
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_2)),
		.qmenu = h264_video_entropy_cabac_model,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE,
		.name = "MPEG4 Profile",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE,
		.maximum = CODING,
		.default_value = V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE,
		.step = 1,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL,
		.name = "MPEG4 Level",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_MPEG4_LEVEL_0,
		.maximum = V4L2_MPEG_VIDEO_MPEG4_LEVEL_5,
		.default_value = V4L2_MPEG_VIDEO_MPEG4_LEVEL_0,
		.step = 1,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.name = "H264 Profile",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.maximum = V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH,
		.default_value = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.step = 1,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.name = "H264 Level",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.maximum = V4L2_MPEG_VIDEO_H264_LEVEL_5_2,
		.default_value = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.step = 0,
		.menu_skip_mask = 0,
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
		(1 << V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_HIGHLATENCY)),
		.qmenu = h263_profile,
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
			(1 << V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_7_0)),
		.qmenu = h263_level,
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
			(1 << V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_1)),
		.qmenu = vp8_profile_level,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_HEVC_PROFILE,
		.name = "HEVC Profile",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_HEVC_PROFILE_MAIN,
		.maximum = V4L2_MPEG_VIDC_VIDEO_HEVC_PROFILE_MAIN_STILL_PIC,
		.default_value = V4L2_MPEG_VIDC_VIDEO_HEVC_PROFILE_MAIN,
		.step = 0,
		.menu_skip_mask =  ~(
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_PROFILE_MAIN) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_PROFILE_MAIN10) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_PROFILE_MAIN_STILL_PIC)),
		.qmenu = hevc_profile,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_HEVC_TIER_LEVEL,
		.name = "HEVC Tier and Level",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_1,
		.maximum = V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_6_2,
		.default_value =
			V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_1,
		.step = 0,
		.menu_skip_mask = ~(
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_1) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_2) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_2_1) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_3) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_3_1) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_4) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_4_1) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_5) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_5_1) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_1) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_2) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_2_1) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_3) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_3_1) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_4) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_4_1) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_5) |
		(1 << V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_5_1)
		),
		.qmenu = hevc_tier_level,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_ROTATION,
		.name = "Rotation",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_NONE,
		.maximum = V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_270,
		.default_value = V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_NONE,
		.step = 0,
		.menu_skip_mask = ~(
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_NONE) |
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_90) |
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_180) |
			(1 << V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_270)),
		.qmenu = mpeg_video_rotation,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
		.name = "I Frame Quantization",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 51,
		.default_value = I_FRAME_QP,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
		.name = "P Frame Quantization",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 51,
		.default_value = P_FRAME_QP,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP,
		.name = "B Frame Quantization",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 51,
		.default_value = B_FRAME_QP,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
		.name = "H264 Minimum QP",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 51,
		.default_value = 1,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
		.name = "H264 Maximum QP",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 51,
		.default_value = 51,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_VP8_MIN_QP,
		.name = "VP8 Minimum QP",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 128,
		.default_value = 1,
		.step = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_VP8_MAX_QP,
		.name = "VP8 Maximum QP",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 128,
		.default_value = 128,
		.step = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
		.name = "Slice Mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE,
		.maximum = V4L2_MPEG_VIDEO_MULTI_SLICE_GOB,
		.default_value = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE,
		.step = 1,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE) |
			(1 << V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB) |
			(1 << V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES) |
			(1 << V4L2_MPEG_VIDEO_MULTI_SLICE_GOB)),
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES,
		.name = "Slice Byte Size",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = MIN_SLICE_BYTE_SIZE,
		.maximum = MAX_SLICE_BYTE_SIZE,
		.default_value = MIN_SLICE_BYTE_SIZE,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB,
		.name = "Slice MB Size",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = MAX_SLICE_MB_SIZE,
		.default_value = 1,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_GOB,
		.name = "Slice GOB",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = MAX_SLICE_MB_SIZE,
		.default_value = 1,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_DELIVERY_MODE,
		.name = "Slice delivery mode",
		.type = V4L2_CTRL_TYPE_BUTTON,
		.minimum = 0,
		.maximum = 1,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_MODE,
		.name = "Intra Refresh Mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_NONE,
		.maximum = V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_RANDOM,
		.default_value = V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_NONE,
		.step = 0,
		.menu_skip_mask = ~(
		(1 << V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_NONE) |
		(1 << V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_CYCLIC) |
		(1 << V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_ADAPTIVE) |
		(1 << V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_CYCLIC_ADAPTIVE) |
		(1 << V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_RANDOM)),
		.qmenu = intra_refresh_modes,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_AIR_MBS,
		.name = "Intra Refresh AIR MBS",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = MAX_INTRA_REFRESH_MBS,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_AIR_REF,
		.name = "Intra Refresh AIR REF",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = MAX_INTRA_REFRESH_MBS,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_CIR_MBS,
		.name = "Intra Refresh CIR MBS",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = MAX_INTRA_REFRESH_MBS,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA,
		.name = "H.264 Loop Filter Alpha Offset",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = -6,
		.maximum = 6,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA,
		.name = "H.264 Loop Filter Beta Offset",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = -6,
		.maximum = 6,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE,
		.name = "H.264 Loop Filter Mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED,
		.maximum = L_MODE,
		.default_value = V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED,
		.step = 1,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED) |
			(1 << V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED) |
			(1 << L_MODE)),
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEADER_MODE,
		.name = "Sequence Header Mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE,
		.maximum = V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_I_FRAME,
		.default_value =
				V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_I_FRAME,
		.step = 1,
		.menu_skip_mask = ~(
			(1 << V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE) |
			(1 << V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_I_FRAME)),
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
		.maximum = V4L2_MPEG_VIDC_EXTRADATA_METADATA_MBI,
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
			(1 << V4L2_MPEG_VIDC_EXTRADATA_LTR) |
			(1 << V4L2_MPEG_VIDC_EXTRADATA_METADATA_MBI)),
		.qmenu = mpeg_video_vidc_extradata,
		.step = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO,
		.name = "H264 VUI Timing Info",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = V4L2_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO_DISABLED,
		.maximum = V4L2_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO_ENABLED,
		.default_value =
			V4L2_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO_DISABLED,
		.step = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_H264_AU_DELIMITER,
		.name = "H264 AU Delimiter",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = V4L2_MPEG_VIDC_VIDEO_H264_AU_DELIMITER_DISABLED,
		.maximum = V4L2_MPEG_VIDC_VIDEO_H264_AU_DELIMITER_ENABLED,
		.step = 1,
		.default_value =
			V4L2_MPEG_VIDC_VIDEO_H264_AU_DELIMITER_DISABLED,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB,
		.name = "Intra Refresh CIR MBS",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = MAX_INTRA_REFRESH_MBS,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_H264_VUI_BITSTREAM_RESTRICT,
		.name = "H264 VUI Timing Info",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = BITSTREAM_RESTRICT_DISABLED,
		.maximum = BITSTREAM_RESTRICT_ENABLED,
		.default_value = BITSTREAM_RESTRICT_ENABLED,
		.step = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_PRESERVE_TEXT_QUALITY,
		.name = "Preserve Text Qualty",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = V4L2_MPEG_VIDC_VIDEO_PRESERVE_TEXT_QUALITY_DISABLED,
		.maximum = V4L2_MPEG_VIDC_VIDEO_PRESERVE_TEXT_QUALITY_ENABLED,
		.default_value =
			V4L2_MPEG_VIDC_VIDEO_PRESERVE_TEXT_QUALITY_DISABLED,
		.step = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE,
		.name = "Deinterlace for encoder",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE_DISABLED,
		.maximum = V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE_ENABLED,
		.default_value = V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE_DISABLED,
		.step = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_MPEG4_TIME_RESOLUTION,
		.name = "Vop time increment resolution",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = MIN_TIME_RESOLUTION,
		.maximum = MAX_TIME_RESOLUTION,
		.default_value = DEFAULT_TIME_RESOLUTION,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_REQUEST_SEQ_HEADER,
		.name = "Request Seq Header",
		.type = V4L2_CTRL_TYPE_BUTTON,
		.minimum = 0,
		.maximum = 0,
		.default_value = 0,
		.step = 0,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_USELTRFRAME,
		.name = "H264 Use LTR",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = MAX_LTR_FRAME_COUNT - 1,
		.default_value = 0,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_LTRCOUNT,
		.name = "Ltr Count",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = MAX_LTR_FRAME_COUNT,
		.default_value = 0,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_LTRMODE,
		.name = "Ltr Mode",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = V4L2_MPEG_VIDC_VIDEO_LTR_MODE_DISABLE,
		.maximum = V4L2_MPEG_VIDC_VIDEO_LTR_MODE_PERIODIC,
		.default_value = V4L2_MPEG_VIDC_VIDEO_LTR_MODE_DISABLE,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_MARKLTRFRAME,
		.name = "H264 Mark LTR",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (MAX_LTR_FRAME_COUNT - 1),
		.default_value = 0,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_HIER_P_NUM_LAYERS,
		.name = "Set Hier P num layers",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 3,
		.default_value = 0,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_TIMESTAMP_MODE,
		.name = "Encoder Timestamp Mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum =
			V4L2_MPEG_VIDC_VIDEO_RATE_CONTROL_TIMESTAMP_MODE_HONOR,
		.maximum =
			V4L2_MPEG_VIDC_VIDEO_RATE_CONTROL_TIMESTAMP_MODE_IGNORE,
		.default_value =
			V4L2_MPEG_VIDC_VIDEO_RATE_CONTROL_TIMESTAMP_MODE_HONOR,
		.menu_skip_mask = ~(
		(1 << V4L2_MPEG_VIDC_VIDEO_RATE_CONTROL_TIMESTAMP_MODE_HONOR) |
		(1 << V4L2_MPEG_VIDC_VIDEO_RATE_CONTROL_TIMESTAMP_MODE_IGNORE)),
		.qmenu = timestamp_mode,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_VPX_ERROR_RESILIENCE,
		.name = "VP8 Error Resilience mode",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = V4L2_MPEG_VIDC_VIDEO_VPX_ERROR_RESILIENCE_DISABLED,
		.maximum = V4L2_MPEG_VIDC_VIDEO_VPX_ERROR_RESILIENCE_ENABLED,
		.default_value =
			V4L2_MPEG_VIDC_VIDEO_VPX_ERROR_RESILIENCE_DISABLED,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_INITIAL_QP,
		.name = "Enable setting initial QP",
		.type = V4L2_CTRL_TYPE_BITMASK,
		.minimum = 0,
		.maximum = V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_INITIAL_QP_IFRAME |
			V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_INITIAL_QP_PFRAME |
			V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_INITIAL_QP_BFRAME,
		.default_value = 0,
		.step = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_I_FRAME_QP,
		.name = "Iframe initial QP",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 51,
		.default_value = 1,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_P_FRAME_QP,
		.name = "Pframe initial QP",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 51,
		.default_value = 1,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_B_FRAME_QP,
		.name = "Bframe initial QP",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 51,
		.default_value = 1,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_IFRAME_X_RANGE,
		.name = "I-Frame X coordinate search range",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 4,
		.maximum = 128,
		.default_value = 4,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_IFRAME_Y_RANGE,
		.name = "I-Frame Y coordinate search range",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 4,
		.maximum = 128,
		.default_value = 4,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_PFRAME_X_RANGE,
		.name = "P-Frame X coordinate search range",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 4,
		.maximum = 128,
		.default_value = 4,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_PFRAME_Y_RANGE,
		.name = "P-Frame Y coordinate search range",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 4,
		.maximum = 128,
		.default_value = 4,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_BFRAME_X_RANGE,
		.name = "B-Frame X coordinate search range",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 4,
		.maximum = 128,
		.default_value = 4,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_BFRAME_Y_RANGE,
		.name = "B-Frame Y coordinate search range",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 4,
		.maximum = 128,
		.default_value = 4,
		.step = 1,
		.menu_skip_mask = 0,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_H264_NAL_SVC,
		.name = "Enable H264 SVC NAL",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = V4L2_CID_MPEG_VIDC_VIDEO_H264_NAL_SVC_DISABLED,
		.maximum = V4L2_CID_MPEG_VIDC_VIDEO_H264_NAL_SVC_ENABLED,
		.default_value = V4L2_CID_MPEG_VIDC_VIDEO_H264_NAL_SVC_DISABLED,
		.step = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_PERF_MODE,
		.name = "Set Encoder performance mode",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = V4L2_MPEG_VIDC_VIDEO_PERF_MAX_QUALITY,
		.maximum = V4L2_MPEG_VIDC_VIDEO_PERF_POWER_SAVE,
		.default_value = V4L2_MPEG_VIDC_VIDEO_PERF_MAX_QUALITY,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_HIER_B_NUM_LAYERS,
		.name = "Set Hier B num layers",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 3,
		.default_value = 0,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDC_VIDEO_HYBRID_HIERP_MODE,
		.name = "Set Hybrid Hier P mode",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 5,
		.default_value = 0,
		.step = 1,
		.qmenu = NULL,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED,
		.maximum = V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED,
		.default_value = V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_GOP_SIZE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 12,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
};

#define NUM_CTRLS	ARRAY_SIZE(venc_ctrls)

/* Helper function to translate V4L2_* to HAL_* */
static inline int venc_v4l2_to_hal(int id, int value)
{
	switch (id) {
	/* MPEG4 */
	case V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL:
		switch (value) {
		case V4L2_MPEG_VIDEO_MPEG4_LEVEL_0:
			return HAL_MPEG4_LEVEL_0;
		case V4L2_MPEG_VIDEO_MPEG4_LEVEL_0B:
			return HAL_MPEG4_LEVEL_0b;
		case V4L2_MPEG_VIDEO_MPEG4_LEVEL_1:
			return HAL_MPEG4_LEVEL_1;
		case V4L2_MPEG_VIDEO_MPEG4_LEVEL_2:
			return HAL_MPEG4_LEVEL_2;
		case V4L2_MPEG_VIDEO_MPEG4_LEVEL_3:
			return HAL_MPEG4_LEVEL_3;
		case V4L2_MPEG_VIDEO_MPEG4_LEVEL_4:
			return HAL_MPEG4_LEVEL_4;
		case V4L2_MPEG_VIDEO_MPEG4_LEVEL_5:
			return HAL_MPEG4_LEVEL_5;
		default:
			goto unknown_value;
		}
	case V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE:
		switch (value) {
		case V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE:
			return HAL_MPEG4_PROFILE_SIMPLE;
		case V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_SIMPLE:
			return HAL_MPEG4_PROFILE_ADVANCEDSIMPLE;
		default:
			goto unknown_value;
		}
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
		case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH:
			return HAL_H264_PROFILE_CONSTRAINED_HIGH;
		default:
			goto unknown_value;
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
		case V4L2_MPEG_VIDEO_H264_LEVEL_5_2:
			return HAL_H264_LEVEL_52;
		default:
			goto unknown_value;
		}
	/* H263 */
	case V4L2_CID_MPEG_VIDC_VIDEO_H263_PROFILE:
		switch (value) {
		case V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_BASELINE:
			return HAL_H263_PROFILE_BASELINE;
		case V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_H320CODING:
			return HAL_H263_PROFILE_H320CODING;
		case V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_BACKWARDCOMPATIBLE:
			return HAL_H263_PROFILE_BACKWARDCOMPATIBLE;
		case V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_ISWV2:
			return HAL_H263_PROFILE_ISWV2;
		case V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_ISWV3:
			return HAL_H263_PROFILE_ISWV3;
		case V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_HIGHCOMPRESSION:
			return HAL_H263_PROFILE_HIGHCOMPRESSION;
		case V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_INTERNET:
			return HAL_H263_PROFILE_INTERNET;
		case V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_INTERLACE:
			return HAL_H263_PROFILE_INTERLACE;
		case V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_HIGHLATENCY:
			return HAL_H263_PROFILE_HIGHLATENCY;
		default:
			goto unknown_value;
		}
	case V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE:
		switch (value) {
		case V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC:
			return HAL_H264_ENTROPY_CAVLC;
		case V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC:
			return HAL_H264_ENTROPY_CABAC;
		default:
			goto unknown_value;
		}
	case V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL:
		switch (value) {
		case V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_0:
			return HAL_H264_CABAC_MODEL_0;
		case V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_1:
			return HAL_H264_CABAC_MODEL_1;
		case V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_2:
			return HAL_H264_CABAC_MODEL_2;
		default:
			goto unknown_value;
		}
	case V4L2_CID_MPEG_VIDC_VIDEO_H263_LEVEL:
		switch (value) {
		case V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_1_0:
			return HAL_H263_LEVEL_10;
		case V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_2_0:
			return HAL_H263_LEVEL_20;
		case V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_3_0:
			return HAL_H263_LEVEL_30;
		case V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_4_0:
			return HAL_H263_LEVEL_40;
		case V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_4_5:
			return HAL_H263_LEVEL_45;
		case V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_5_0:
			return HAL_H263_LEVEL_50;
		case V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_6_0:
			return HAL_H263_LEVEL_60;
		case V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_7_0:
			return HAL_H263_LEVEL_70;
		default:
			goto unknown_value;
		}
	case V4L2_CID_MPEG_VIDC_VIDEO_VP8_PROFILE_LEVEL:
		switch (value) {
		case V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_0:
			return HAL_VPX_PROFILE_VERSION_0;
		case V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_1:
			return HAL_VPX_PROFILE_VERSION_1;
		case V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_2:
			return HAL_VPX_PROFILE_VERSION_2;
		case V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_3:
			return HAL_VPX_PROFILE_VERSION_3;
		case V4L2_MPEG_VIDC_VIDEO_VP8_UNUSED:
			return HAL_VPX_PROFILE_UNUSED;
		default:
			goto unknown_value;
		}
	case V4L2_CID_MPEG_VIDC_VIDEO_HEVC_PROFILE:
		switch (value) {
		case V4L2_MPEG_VIDC_VIDEO_HEVC_PROFILE_MAIN:
			return HAL_HEVC_PROFILE_MAIN;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_PROFILE_MAIN10:
			return HAL_HEVC_PROFILE_MAIN10;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_PROFILE_MAIN_STILL_PIC:
			return HAL_HEVC_PROFILE_MAIN_STILL_PIC;
		default:
			goto unknown_value;
		}
	case V4L2_CID_MPEG_VIDC_VIDEO_HEVC_TIER_LEVEL:
		switch (value) {
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_1:
			return HAL_HEVC_MAIN_TIER_LEVEL_1;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_2:
			return HAL_HEVC_MAIN_TIER_LEVEL_2;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_2_1:
			return HAL_HEVC_MAIN_TIER_LEVEL_2_1;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_3:
			return HAL_HEVC_MAIN_TIER_LEVEL_3;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_3_1:
			return HAL_HEVC_MAIN_TIER_LEVEL_3_1;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_4:
			return HAL_HEVC_MAIN_TIER_LEVEL_4;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_4_1:
			return HAL_HEVC_MAIN_TIER_LEVEL_4_1;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_5:
			return HAL_HEVC_MAIN_TIER_LEVEL_5;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_5_1:
			return HAL_HEVC_MAIN_TIER_LEVEL_5_1;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_5_2:
			return HAL_HEVC_MAIN_TIER_LEVEL_5_2;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_6:
			return HAL_HEVC_MAIN_TIER_LEVEL_6;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_6_1:
			return HAL_HEVC_MAIN_TIER_LEVEL_6_1;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_MAIN_TIER_LEVEL_6_2:
			return HAL_HEVC_MAIN_TIER_LEVEL_6_2;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_1:
			return HAL_HEVC_HIGH_TIER_LEVEL_1;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_2:
			return HAL_HEVC_HIGH_TIER_LEVEL_2;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_2_1:
			return HAL_HEVC_HIGH_TIER_LEVEL_2_1;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_3:
			return HAL_HEVC_HIGH_TIER_LEVEL_3;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_3_1:
			return HAL_HEVC_HIGH_TIER_LEVEL_3_1;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_4:
			return HAL_HEVC_HIGH_TIER_LEVEL_4;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_4_1:
			return HAL_HEVC_HIGH_TIER_LEVEL_4_1;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_5:
			return HAL_HEVC_HIGH_TIER_LEVEL_5;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_5_1:
			return HAL_HEVC_HIGH_TIER_LEVEL_5_1;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_5_2:
			return HAL_HEVC_HIGH_TIER_LEVEL_5_2;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_6:
			return HAL_HEVC_HIGH_TIER_LEVEL_6;
		case V4L2_MPEG_VIDC_VIDEO_HEVC_LEVEL_HIGH_TIER_LEVEL_6_1:
			return HAL_HEVC_HIGH_TIER_LEVEL_6_1;
		default:
			goto unknown_value;
		}
	case V4L2_CID_MPEG_VIDC_VIDEO_ROTATION:
		switch (value) {
		case V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_NONE:
			return HAL_ROTATE_NONE;
		case V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_90:
			return HAL_ROTATE_90;
		case V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_180:
			return HAL_ROTATE_180;
		case V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_270:
			return HAL_ROTATE_270;
		default:
			goto unknown_value;
		}
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE:
		switch (value) {
		case V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED:
			return HAL_H264_DB_MODE_DISABLE;
		case V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED:
			return HAL_H264_DB_MODE_ALL_BOUNDARY;
		case L_MODE:
			return HAL_H264_DB_MODE_SKIP_SLICE_BOUNDARY;
		default:
			goto unknown_value;
		}
	}

unknown_value:
	dprintk(VIDC_WARN, "Unknown control (%x, %d)\n", id, value);
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
	struct vidc_core_capability *cap = &inst->capability;
	struct hal_request_iframe request_iframe;
	struct hal_bitrate bitrate;
	struct hal_profile_level profile_level;
	struct hal_h264_entropy_control h264_entropy_control;
	struct hal_quantization quantization;
	struct hal_intra_period intra_period;
	struct hal_idr_period idr_period;
	struct hal_operations operations;
	struct hal_intra_refresh intra_refresh;
	struct hal_multi_slice_control multi_slice_control;
	struct hal_h264_db_control h264_db_control;
	struct hal_enable enable;
	struct hal_h264_vui_timing_info vui_timing_info;
	struct hal_quantization_range qp_range;
	struct hal_h264_vui_bitstream_restrc vui_bitstream_restrict;
	struct hal_preserve_text_quality preserve_text_quality;
	struct hal_extradata_enable extra;
	struct hal_mpeg4_time_resolution time_res;
	struct hal_ltr_use use_ltr;
	struct hal_ltr_mark mark_ltr;
	struct hal_hybrid_hierp hyb_hierp;
	struct hal_ltr_mode ltr_mode;
	struct hal_vc1e_perf_cfg_type search_range = { {0} };
	struct hal_initial_quantization quant;
	u32 hier_p_layers = 0;
	struct hal_venc_perf_mode venc_mode;
	u32 property_id = 0, property_val = 0;
	void *pdata = NULL;
	struct v4l2_ctrl *temp_ctrl = NULL;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDC_VIDEO_IDR_PERIOD:
		if (inst->fmt_cap->pixfmt != V4L2_PIX_FMT_H264 &&
		    inst->fmt_cap->pixfmt != V4L2_PIX_FMT_H264_NO_SC) {
			dev_err(dev, "Control 0x%x only valid for H264\n",
				ctrl->id);
			ret = -ENOTSUPP;
			break;
		}

		property_id = HAL_CONFIG_VENC_IDR_PERIOD;
		idr_period.idr_period = ctrl->val;
		pdata = &idr_period;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_NUM_B_FRAMES:
	case V4L2_CID_MPEG_VIDC_VIDEO_NUM_P_FRAMES: {
		int num_p, num_b;

		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_NUM_B_FRAMES);
		num_b = temp_ctrl->val;

		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_NUM_P_FRAMES);
		num_p = temp_ctrl->val;

		if (ctrl->id == V4L2_CID_MPEG_VIDC_VIDEO_NUM_P_FRAMES)
			num_p = ctrl->val;
		else if (ctrl->id == V4L2_CID_MPEG_VIDC_VIDEO_NUM_B_FRAMES)
			num_b = ctrl->val;

		if (num_b) {
			u32 max_num_b_frames = MAX_NUM_B_FRAMES;

			property_id = HAL_PARAM_VENC_MAX_NUM_B_FRAMES;
			pdata = &max_num_b_frames;

			ret = hfi_session_set_property(inst, property_id,
						       pdata);
			if (ret) {
				dev_err(dev,
					"Failed : Setprop MAX_NUM_B_FRAMES %d\n",
					ret);
				break;
			}
		}

		property_id = HAL_CONFIG_VENC_INTRA_PERIOD;
		intra_period.pframes = num_p;
		intra_period.bframes = num_b;
		pdata = &intra_period;
		break;
	}
	case V4L2_CID_MPEG_VIDC_VIDEO_REQUEST_IFRAME:
		property_id = HAL_CONFIG_VENC_REQUEST_IFRAME;
		request_iframe.enable = true;
		pdata = &request_iframe;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL:
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE: {
		struct v4l2_ctrl update_ctrl = {.id = 0};
		int final_mode = 0;

		/* V4L2_CID_MPEG_VIDEO_BITRATE_MODE and _RATE_CONTROL
		 * manipulate the same thing.  If one control's state
		 * changes, try to mirror the state in the other control's
		 * value */
		if (ctrl->id == V4L2_CID_MPEG_VIDEO_BITRATE_MODE) {
			if (ctrl->val == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) {
				final_mode = HAL_RATE_CONTROL_VBR_CFR;
				update_ctrl.val =
				V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_CFR;
			} else {/* ...if (ctrl->val == _BITRATE_MODE_CBR) */
				final_mode = HAL_RATE_CONTROL_CBR_CFR;
				update_ctrl.val =
				V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_CFR;
			}

			update_ctrl.id = V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL;

		} else if (ctrl->id == V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL) {
			switch (ctrl->val) {
			case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_OFF:
			case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_VFR:
			case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_CFR:
				update_ctrl.val =
					V4L2_MPEG_VIDEO_BITRATE_MODE_VBR;
			case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_VFR:
			case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_CFR:
				update_ctrl.val =
					V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
			}

			final_mode = ctrl->val;
			update_ctrl.id = V4L2_CID_MPEG_VIDEO_BITRATE_MODE;
		}

		if (update_ctrl.id) {
			temp_ctrl = TRY_GET_CTRL(update_ctrl.id);
			temp_ctrl->val = update_ctrl.val;
		}

		property_id = HAL_PARAM_VENC_RATE_CONTROL;
		property_val = final_mode;
		pdata = &property_val;

		break;
	}
	case V4L2_CID_MPEG_VIDEO_BITRATE: {
		struct v4l2_ctrl *hier_p = TRY_GET_CTRL(
				V4L2_CID_MPEG_VIDC_VIDEO_HIER_P_NUM_LAYERS);

		bitrate.layer_id = 0;

		if (hier_p->val && inst->fmt_cap->pixfmt == V4L2_PIX_FMT_H264) {
			ret = venc_set_bitrate_for_each_layer(inst, hier_p->val,
							      ctrl->val);
			if (ret) {
				dev_err(dev, "set bitrate for multiple layers\n");
				break;
			}
		} else {
			property_id = HAL_CONFIG_VENC_TARGET_BITRATE;
			bitrate.bit_rate = ctrl->val;
			bitrate.layer_id = 0;
			pdata = &bitrate;
		}
		break;
	}
	case V4L2_CID_MPEG_VIDEO_BITRATE_PEAK: {
		struct v4l2_ctrl *avg_bitrate =
				TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_BITRATE);

		if (ctrl->val < avg_bitrate->val) {
			dev_err(dev,
				"Peak bitrate (%d) is lower than average bitrate (%d)\n",
				ctrl->val, avg_bitrate->val);
			ret = -EINVAL;
			break;
		} else if (ctrl->val < avg_bitrate->val * 2) {
			dev_err(dev,
				"Peak bitrate (%d) ideally should be twice the average bitrate (%d)\n",
				ctrl->val, avg_bitrate->val);
		}

		property_id = HAL_CONFIG_VENC_MAX_BITRATE;
		bitrate.bit_rate = ctrl->val;
		bitrate.layer_id = 0;
		pdata = &bitrate;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE:
		temp_ctrl = TRY_GET_CTRL(
			V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL);

		property_id = HAL_PARAM_VENC_H264_ENTROPY_CONTROL;
		h264_entropy_control.entropy_mode = venc_v4l2_to_hal(
			V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE, ctrl->val);
		h264_entropy_control.cabac_model = venc_v4l2_to_hal(
			V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL,
			temp_ctrl->val);
		pdata = &h264_entropy_control;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL:
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE);

		property_id = HAL_PARAM_VENC_H264_ENTROPY_CONTROL;
		h264_entropy_control.cabac_model = venc_v4l2_to_hal(
			V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE, ctrl->val);
		h264_entropy_control.entropy_mode = venc_v4l2_to_hal(
			V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL,
			temp_ctrl->val);
		pdata = &h264_entropy_control;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE:
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL);

		property_id = HAL_PARAM_PROFILE_LEVEL_CURRENT;
		profile_level.profile = venc_v4l2_to_hal(ctrl->id, ctrl->val);
		profile_level.level = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL,
				temp_ctrl->val);
		pdata = &profile_level;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL:
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE);

		property_id = HAL_PARAM_PROFILE_LEVEL_CURRENT;
		profile_level.level = venc_v4l2_to_hal(ctrl->id, ctrl->val);
		profile_level.profile = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE,
				temp_ctrl->val);
		pdata = &profile_level;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_LEVEL);

		property_id = HAL_PARAM_PROFILE_LEVEL_CURRENT;
		profile_level.profile = venc_v4l2_to_hal(ctrl->id, ctrl->val);
		profile_level.level = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDEO_H264_LEVEL,
				temp_ctrl->val);
		pdata = &profile_level;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_PROFILE);

		property_id = HAL_PARAM_PROFILE_LEVEL_CURRENT;
		profile_level.level = venc_v4l2_to_hal(ctrl->id, ctrl->val);
		profile_level.profile = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDEO_H264_PROFILE,
				temp_ctrl->val);
		pdata = &profile_level;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_H263_PROFILE:
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_H263_LEVEL);

		property_id = HAL_PARAM_PROFILE_LEVEL_CURRENT;
		profile_level.profile = venc_v4l2_to_hal(ctrl->id, ctrl->val);
		profile_level.level = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDC_VIDEO_H263_LEVEL,
				temp_ctrl->val);
		pdata = &profile_level;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_H263_LEVEL:
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_H263_PROFILE);

		property_id = HAL_PARAM_PROFILE_LEVEL_CURRENT;
		profile_level.level = venc_v4l2_to_hal(ctrl->id, ctrl->val);
		profile_level.profile = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDC_VIDEO_H263_PROFILE,
				ctrl->val);
		pdata = &profile_level;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_VP8_PROFILE_LEVEL:
		property_id = HAL_PARAM_PROFILE_LEVEL_CURRENT;
		profile_level.profile = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDC_VIDEO_VP8_PROFILE_LEVEL,
				ctrl->val);
		profile_level.level = HAL_VPX_PROFILE_UNUSED;
		pdata = &profile_level;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_HEVC_PROFILE:
		temp_ctrl =
			TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_HEVC_TIER_LEVEL);

		property_id = HAL_PARAM_PROFILE_LEVEL_CURRENT;
		profile_level.profile = venc_v4l2_to_hal(ctrl->id, ctrl->val);
		profile_level.level = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDC_VIDEO_HEVC_TIER_LEVEL,
				temp_ctrl->val);
		pdata = &profile_level;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_HEVC_TIER_LEVEL:
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_HEVC_PROFILE);

		property_id = HAL_PARAM_PROFILE_LEVEL_CURRENT;
		profile_level.level = venc_v4l2_to_hal(ctrl->id, ctrl->val);
		profile_level.profile = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDC_VIDEO_HEVC_PROFILE,
				temp_ctrl->val);
		pdata = &profile_level;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_ROTATION: {
		struct v4l2_ctrl *deinterlace = NULL;

		if (!(inst->capability.pixelprocess_capabilities &
		    HAL_VIDEO_ENCODER_ROTATION_CAPABILITY)) {
			dev_err(dev, "Rotation not supported: 0x%x\n",
				ctrl->id);
			ret = -ENOTSUPP;
			break;
		}

		deinterlace =
			TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE);
		if (ctrl->val && deinterlace && deinterlace->val !=
		    V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE_DISABLED) {
			dev_err(dev,
				"Rotation not supported with deinterlacing\n");
			ret = -EINVAL;
			break;
		}
		property_id = HAL_CONFIG_VPE_OPERATIONS;
		operations.rotate = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDC_VIDEO_ROTATION,
				ctrl->val);
		operations.flip = HAL_FLIP_NONE;
		pdata = &operations;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP: {
		struct v4l2_ctrl *qpp, *qpb;

		qpp = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP);
		qpb = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP);

		property_id = HAL_PARAM_VENC_SESSION_QP;
		quantization.qpi = ctrl->val;
		quantization.qpp = qpp->val;
		quantization.qpb = qpb->val;
		quantization.layer_id = 0;

		pdata = &quantization;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP: {
		struct v4l2_ctrl *qpi, *qpb;

		qpi = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP);
		qpb = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP);

		property_id = HAL_PARAM_VENC_SESSION_QP;
		quantization.qpp = ctrl->val;
		quantization.qpi = qpi->val;
		quantization.qpb = qpb->val;
		quantization.layer_id = 0;

		pdata = &quantization;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP: {
		struct v4l2_ctrl *qpi, *qpp;

		qpi = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP);
		qpp = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP);

		property_id = HAL_PARAM_VENC_SESSION_QP;
		quantization.qpb = ctrl->val;
		quantization.qpi = qpi->val;
		quantization.qpp = qpp->val;
		quantization.layer_id = 0;

		pdata = &quantization;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP: {
		struct v4l2_ctrl *qp_max;

		qp_max = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_MAX_QP);
		if (ctrl->val >= qp_max->val) {
			dev_err(dev, "Bad range: Min QP (%d) > Max QP(%d)\n",
				ctrl->val, qp_max->val);
			ret = -ERANGE;
			break;
		}

		property_id = HAL_PARAM_VENC_SESSION_QP_RANGE;
		qp_range.layer_id = 0;
		qp_range.max_qp = qp_max->val;
		qp_range.min_qp = ctrl->val;

		pdata = &qp_range;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP: {
		struct v4l2_ctrl *qp_min;

		qp_min = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_MIN_QP);
		if (ctrl->val <= qp_min->val) {
			dev_err(dev, "Bad range: Max QP (%d) < Min QP(%d)\n",
				ctrl->val, qp_min->val);
			ret = -ERANGE;
			break;
		}

		property_id = HAL_PARAM_VENC_SESSION_QP_RANGE;
		qp_range.layer_id = 0;
		qp_range.max_qp = ctrl->val;
		qp_range.min_qp = qp_min->val;

		pdata = &qp_range;
		break;
	}
	case V4L2_CID_MPEG_VIDC_VIDEO_VP8_MIN_QP: {
		struct v4l2_ctrl *qp_max;

		qp_max = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_VP8_MAX_QP);
		property_id = HAL_PARAM_VENC_SESSION_QP_RANGE;
		qp_range.layer_id = 0;
		qp_range.max_qp = qp_max->val;
		qp_range.min_qp = ctrl->val;
		pdata = &qp_range;
		break;
	}
	case V4L2_CID_MPEG_VIDC_VIDEO_VP8_MAX_QP: {
		struct v4l2_ctrl *qp_min;

		qp_min = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_VP8_MIN_QP);
		property_id = HAL_PARAM_VENC_SESSION_QP_RANGE;
		qp_range.layer_id = 0;
		qp_range.max_qp = ctrl->val;
		qp_range.min_qp = qp_min->val;
		pdata = &qp_range;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE: {
		int temp = 0;

		switch (ctrl->val) {
		case V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB:
			temp = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB;
			break;
		case V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES:
			temp = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES;
			break;
		case V4L2_MPEG_VIDEO_MULTI_SLICE_GOB:
			temp = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_GOB;
			break;
		case V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE:
		default:
			temp = 0;
			break;
		}

		if (temp)
			temp_ctrl = TRY_GET_CTRL(temp);

		property_id = HAL_PARAM_VENC_MULTI_SLICE_CONTROL;
		multi_slice_control.multi_slice = ctrl->val;
		multi_slice_control.slice_size = temp ? temp_ctrl->val : 0;

		pdata = &multi_slice_control;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES:
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_GOB:
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE);

		property_id = HAL_PARAM_VENC_MULTI_SLICE_CONTROL;
		multi_slice_control.multi_slice = temp_ctrl->val;
		multi_slice_control.slice_size = ctrl->val;
		pdata = &multi_slice_control;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_DELIVERY_MODE: {
		temp_ctrl = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE);

		if ((temp_ctrl->val == V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB)
		    && (inst->fmt_cap->pixfmt == V4L2_PIX_FMT_H264 ||
			inst->fmt_cap->pixfmt == V4L2_PIX_FMT_H264_NO_SC)) {
			property_id = HAL_PARAM_VENC_SLICE_DELIVERY_MODE;
			enable.enable = true;
		} else {
			dev_warn(dev,
				"Failed : slice delivery mode is valid "\
				"only for H264 encoder and MB based slicing\n");
			enable.enable = false;
		}
		pdata = &enable;
		break;
	}
	case V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_MODE: {
		struct v4l2_ctrl *air_mbs, *air_ref, *cir_mbs;

		air_mbs = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_AIR_MBS);
		air_ref = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_AIR_REF);
		cir_mbs = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_CIR_MBS);

		property_id = HAL_PARAM_VENC_INTRA_REFRESH;

		intra_refresh.mode = ctrl->val;
		intra_refresh.air_mbs = air_mbs->val;
		intra_refresh.air_ref = air_ref->val;
		intra_refresh.cir_mbs = cir_mbs->val;

		pdata = &intra_refresh;
		break;
	}
	case V4L2_CID_MPEG_VIDC_VIDEO_AIR_MBS: {
		struct v4l2_ctrl *ir_mode, *air_ref, *cir_mbs;

		ir_mode = TRY_GET_CTRL(
				V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_MODE);
		air_ref = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_AIR_REF);
		cir_mbs = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_CIR_MBS);

		property_id = HAL_PARAM_VENC_INTRA_REFRESH;

		intra_refresh.air_mbs = ctrl->val;
		intra_refresh.mode = ir_mode->val;
		intra_refresh.air_ref = air_ref->val;
		intra_refresh.cir_mbs = cir_mbs->val;

		pdata = &intra_refresh;
		break;
	}
	case V4L2_CID_MPEG_VIDC_VIDEO_AIR_REF: {
		struct v4l2_ctrl *ir_mode, *air_mbs, *cir_mbs;

		ir_mode = TRY_GET_CTRL(
				V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_MODE);
		air_mbs = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_AIR_MBS);
		cir_mbs = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_CIR_MBS);

		property_id = HAL_PARAM_VENC_INTRA_REFRESH;

		intra_refresh.air_ref = ctrl->val;
		intra_refresh.air_mbs = air_mbs->val;
		intra_refresh.mode = ir_mode->val;
		intra_refresh.cir_mbs = cir_mbs->val;

		pdata = &intra_refresh;
		break;
	}
	case V4L2_CID_MPEG_VIDC_VIDEO_CIR_MBS: {
		struct v4l2_ctrl *ir_mode, *air_mbs, *air_ref;

		ir_mode = TRY_GET_CTRL(
				V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_MODE);
		air_mbs = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_AIR_MBS);
		air_ref = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_AIR_REF);

		property_id = HAL_PARAM_VENC_INTRA_REFRESH;

		intra_refresh.cir_mbs = ctrl->val;
		intra_refresh.air_mbs = air_mbs->val;
		intra_refresh.air_ref = air_ref->val;
		intra_refresh.mode = ir_mode->val;

		pdata = &intra_refresh;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB: {
		struct v4l2_ctrl *air_mbs, *air_ref;

		air_mbs = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_AIR_MBS);
		air_ref = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_AIR_REF);

		property_id = HAL_PARAM_VENC_INTRA_REFRESH;

		intra_refresh.cir_mbs = ctrl->val;
		intra_refresh.air_mbs = air_mbs->val;
		intra_refresh.air_ref = air_ref->val;
		intra_refresh.mode = HAL_INTRA_REFRESH_CYCLIC;

		pdata = &intra_refresh;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE: {
		struct v4l2_ctrl *alpha, *beta;

		alpha = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA);
		beta = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA);

		property_id = HAL_PARAM_VENC_H264_DEBLOCK_CONTROL;
		h264_db_control.slice_alpha_offset = alpha->val;
		h264_db_control.slice_beta_offset = beta->val;
		h264_db_control.mode = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE,
				ctrl->val);
		pdata = &h264_db_control;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA: {
		struct v4l2_ctrl *mode, *beta;

		mode = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE);
		beta = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA);

		property_id = HAL_PARAM_VENC_H264_DEBLOCK_CONTROL;
		h264_db_control.slice_alpha_offset = ctrl->val;
		h264_db_control.slice_beta_offset = beta->val;
		h264_db_control.mode = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE,
				mode->val);
		pdata = &h264_db_control;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA: {
		struct v4l2_ctrl *mode, *alpha;

		mode = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE);
		alpha = TRY_GET_CTRL(V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA);

		property_id = HAL_PARAM_VENC_H264_DEBLOCK_CONTROL;
		h264_db_control.slice_alpha_offset = alpha->val;
		h264_db_control.slice_beta_offset = ctrl->val;
		h264_db_control.mode = venc_v4l2_to_hal(
				V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE,
				mode->val);
		pdata = &h264_db_control;
		break;
	}
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		property_id = HAL_PARAM_VENC_SYNC_FRAME_SEQUENCE_HEADER;

		switch (ctrl->val) {
		case V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE:
			enable.enable = 0;
			break;
		case V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_I_FRAME:
			enable.enable = 1;
			break;
		case V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME:
		default:
			ret = -ENOTSUPP;
			break;
		}
		pdata = &enable;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_SECURE:
		inst->flags |= VIDC_SECURE;
		dev_info(dev, "Setting secure mode to: %d\n",
			!!(inst->flags & VIDC_SECURE));
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_EXTRADATA:
		property_id = HAL_PARAM_INDEX_EXTRADATA;
		extra.index = vidc_comm_get_hal_extradata_index(ctrl->val);
		extra.enable = 1;
		pdata = &extra;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO: {
		struct v4l2_ctrl *rc_mode;
		u32 cfr = 0;

		property_id = HAL_PARAM_VENC_H264_VUI_TIMING_INFO;
		rc_mode = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL);

		switch (rc_mode->val) {
		case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_CFR:
		case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_CFR:
			cfr = 1;
			break;
		default:
			cfr = 0;
			break;
		}

		switch (ctrl->val) {
		case V4L2_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO_DISABLED:
			vui_timing_info.enable = 0;
			break;
		case V4L2_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO_ENABLED:
			vui_timing_info.enable = 1;
			vui_timing_info.fixed_frame_rate = cfr;
			vui_timing_info.time_scale = NSEC_PER_SEC;
		}

		pdata = &vui_timing_info;
		break;
	}
	case V4L2_CID_MPEG_VIDC_VIDEO_H264_AU_DELIMITER:
		property_id = HAL_PARAM_VENC_H264_GENERATE_AUDNAL;

		switch (ctrl->val) {
		case V4L2_MPEG_VIDC_VIDEO_H264_AU_DELIMITER_DISABLED:
			enable.enable = 0;
			break;
		case V4L2_MPEG_VIDC_VIDEO_H264_AU_DELIMITER_ENABLED:
			enable.enable = 1;
			break;
		default:
			ret = -ENOTSUPP;
			break;
		}

		pdata = &enable;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_H264_VUI_BITSTREAM_RESTRICT:
		property_id = HAL_PARAM_VENC_H264_VUI_BITSTREAM_RESTRC;
		vui_bitstream_restrict.enable = ctrl->val;
		pdata = &vui_bitstream_restrict;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_PRESERVE_TEXT_QUALITY:
		property_id = HAL_PARAM_VENC_PRESERVE_TEXT_QUALITY;
		preserve_text_quality.enable = ctrl->val;
		pdata = &preserve_text_quality;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_MPEG4_TIME_RESOLUTION:
		property_id = HAL_PARAM_VENC_MPEG4_TIME_RESOLUTION;
		time_res.time_increment_resolution = ctrl->val;
		pdata = &time_res;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE: {
		struct v4l2_ctrl *rotation = NULL;

		if (!(inst->capability.pixelprocess_capabilities &
			HAL_VIDEO_ENCODER_DEINTERLACE_CAPABILITY)) {
			dev_err(dev, "Deinterlace not supported: 0x%x\n",
				ctrl->id);
			ret = -ENOTSUPP;
			break;
		}

		rotation = TRY_GET_CTRL(V4L2_CID_MPEG_VIDC_VIDEO_ROTATION);
		if (ctrl->val && rotation && rotation->val !=
			V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_NONE) {
			dev_err(dev, "Deinterlacing not supported with rotation");
			ret = -EINVAL;
			break;
		}
		property_id = HAL_CONFIG_VPE_DEINTERLACE;
		switch (ctrl->val) {
		case V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE_ENABLED:
			enable.enable = 1;
			break;
		case V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE_DISABLED:
		default:
			enable.enable = 0;
			break;
		}
		pdata = &enable;
		break;
	}
	case V4L2_CID_MPEG_VIDC_VIDEO_REQUEST_SEQ_HEADER:
		atomic_inc(&inst->seq_hdr_reqs);
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_USELTRFRAME:
		property_id = HAL_CONFIG_VENC_USELTRFRAME;
		use_ltr.ref_ltr = (1 << ctrl->val);
		use_ltr.use_constraint = false;
		use_ltr.frames = 0;
		pdata = &use_ltr;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_MARKLTRFRAME:
		property_id = HAL_CONFIG_VENC_MARKLTRFRAME;
		mark_ltr.mark_frame = ctrl->val;
		pdata = &mark_ltr;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_HIER_P_NUM_LAYERS:
		property_id = HAL_CONFIG_VENC_HIER_P_NUM_FRAMES;
		hier_p_layers = ctrl->val;

		ret = venc_toggle_hier_p(inst, ctrl->val);
		if (ret)
			break;

		if (hier_p_layers > inst->capability.hier_p.max) {
			dev_err(dev, "Error setting hier p num layers %d, max supported %d\n",
				hier_p_layers, inst->capability.hier_p.max);
			ret= -ENOTSUPP;
			break;
		}
		pdata = &hier_p_layers;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_TIMESTAMP_MODE:
		property_id = HAL_PARAM_VENC_DISABLE_RC_TIMESTAMP;
		enable.enable = (ctrl->val ==
			V4L2_MPEG_VIDC_VIDEO_RATE_CONTROL_TIMESTAMP_MODE_IGNORE);
		pdata = &enable;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_VPX_ERROR_RESILIENCE:
		property_id = HAL_PARAM_VENC_VPX_ERROR_RESILIENCE_MODE;
		enable.enable = ctrl->val;
		pdata = &enable;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_H264_NAL_SVC:
		property_id = HAL_PARAM_VENC_H264_NAL_SVC_EXT;
		enable.enable = ctrl->val;
		pdata = &enable;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_PERF_MODE:
		property_id = HAL_CONFIG_VENC_PERF_MODE;
		venc_mode.mode = ctrl->val;
		pdata = &venc_mode;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_HYBRID_HIERP_MODE:
		property_id = HAL_PARAM_VENC_HIER_P_HYBRID_MODE;
		hyb_hierp.layers = ctrl->val;
		pdata = &hyb_hierp;
		break;

	case V4L2_CID_MPEG_VIDC_VIDEO_LTRMODE:
		if (ctrl->val != V4L2_MPEG_VIDC_VIDEO_LTR_MODE_DISABLE) {
			ret = venc_toggle_hier_p(inst, false);
			if (ret)
				break;
		}

		ltr_mode.mode = ctrl->val;
		ltr_mode.trust_mode = 1;
		property_id = HAL_PARAM_VENC_LTRMODE;
		pdata = &ltr_mode;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_LTRCOUNT:
		ltr_mode.count = ctrl->val;
		if (ltr_mode.count > cap->ltr_count.max) {
			dev_err(dev, "Invalid LTR count %d, max supported %d\n",
				ltr_mode.count, cap->ltr_count.max);
			/*
			 * FIXME: Return an error (-EINVAL)
			 * here once VP8 supports LTR count
			 * capability
			 */
			ltr_mode.count = 1;
		}
		ltr_mode.trust_mode = 1;
		property_id = HAL_PARAM_VENC_LTRMODE;
		pdata = &ltr_mode;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_INITIAL_QP:
		property_id = HAL_PARAM_VENC_ENABLE_INITIAL_QP;
		quant.init_qp_enable = ctrl->val;
		pdata = &quant;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_I_FRAME_QP:
		quant.qpi = ctrl->val;
		property_id = HAL_PARAM_VENC_ENABLE_INITIAL_QP;
		pdata = &quant;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_P_FRAME_QP:
		quant.qpp = ctrl->val;
		property_id = HAL_PARAM_VENC_ENABLE_INITIAL_QP;
		pdata = &quant;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_B_FRAME_QP:
		quant.qpb = ctrl->val;
		property_id = HAL_PARAM_VENC_ENABLE_INITIAL_QP;
		pdata = &quant;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_IFRAME_X_RANGE:
		search_range.i_frame.x_subsampled = ctrl->val;
		property_id = HAL_PARAM_VENC_SEARCH_RANGE;
		pdata = &search_range;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_IFRAME_Y_RANGE:
		search_range.i_frame.y_subsampled = ctrl->val;
		property_id = HAL_PARAM_VENC_SEARCH_RANGE;
		pdata = &search_range;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_PFRAME_X_RANGE:
		search_range.p_frame.x_subsampled = ctrl->val;
		property_id = HAL_PARAM_VENC_SEARCH_RANGE;
		pdata = &search_range;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_PFRAME_Y_RANGE:
		search_range.p_frame.y_subsampled = ctrl->val;
		property_id = HAL_PARAM_VENC_SEARCH_RANGE;
		pdata = &search_range;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_BFRAME_X_RANGE:
		search_range.b_frame.x_subsampled = ctrl->val;
		property_id = HAL_PARAM_VENC_SEARCH_RANGE;
		pdata = &search_range;
		break;
	case V4L2_CID_MPEG_VIDC_VIDEO_BFRAME_Y_RANGE:
		search_range.b_frame.y_subsampled = ctrl->val;
		property_id = HAL_PARAM_VENC_SEARCH_RANGE;
		pdata = &search_range;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE:
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC:
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
	case V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE:
	case V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM:
		ret = 0;
		property_id = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (!ret && property_id) {
		dev_dbg(dev, "control: hal property:%x, value:%d\n",
			property_id, ctrl->val);
		ret = hfi_session_set_property(inst, property_id, pdata);
	}

	return ret;
}

static int venc_op_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vidc_inst *inst =
		container_of(ctrl->handler, struct vidc_inst, ctrl_handler);
	struct device *dev = &inst->core->res.pdev->dev;
	int ret = 0, c;

	for (c = 0; c < ctrl->ncontrols; ++c) {
		struct v4l2_ctrl *temp = ctrl->cluster[c];

		if (!ctrl->cluster[c]->is_new)
			continue;

		ret = try_set_ctrl(inst, temp);
		if (ret) {
			dev_err(dev, "try set ctrl: %s id: %x (%d)\n",
				v4l2_ctrl_get_name(temp->id), temp->id,
				ret);
			break;
		}
	}

	if (ret)
		dev_err(dev, "setting control: %x (%s) failed\n",
			ctrl->id, v4l2_ctrl_get_name(ctrl->id));

	return ret;
}

static const struct v4l2_ctrl_ops venc_ctrl_ops = {
	.s_ctrl = venc_op_s_ctrl,
};

static struct v4l2_ctrl **get_super_cluster(struct vidc_inst *inst, int *size)
{
	int c, sz = 0;
	struct v4l2_ctrl **cluster = kmalloc(sizeof(struct v4l2_ctrl *) *
					     NUM_CTRLS, GFP_KERNEL);

	if (!size || !cluster || !inst)
		return NULL;

	for (c = 0; c < NUM_CTRLS; c++)
		cluster[sz++] = inst->ctrls[c];

	*size = sz;

	return cluster;
}

int venc_ctrl_init(struct vidc_inst *inst)
{
	struct device *dev = &inst->core->res.pdev->dev;
	struct v4l2_ctrl_config cfg;
	int cluster_size = 0;
	unsigned int idx;
	int ret;

	inst->ctrls = kzalloc(sizeof(struct v4l2_ctrl *) * NUM_CTRLS,
			      GFP_KERNEL);
	if (!inst->ctrls)
		return -ENOMEM;

	ret = v4l2_ctrl_handler_init(&inst->ctrl_handler, NUM_CTRLS);
	if (ret) {
		dev_err(dev, "ctrl handler init (%d)\n", ret);
		return ret;
	}

	for (idx = 0; idx < NUM_CTRLS; idx++) {
		struct v4l2_ctrl *ctrl;

		if (IS_PRIV_CTRL(venc_ctrls[idx].id)) {
			memset(&cfg, 0, sizeof(cfg));
			cfg.def = venc_ctrls[idx].default_value;
			cfg.flags = 0;
			cfg.id = venc_ctrls[idx].id;
			cfg.max = venc_ctrls[idx].maximum;
			cfg.min = venc_ctrls[idx].minimum;
			cfg.menu_skip_mask = venc_ctrls[idx].menu_skip_mask;
			cfg.name = venc_ctrls[idx].name;
			cfg.ops = &venc_ctrl_ops;
			cfg.step = venc_ctrls[idx].step;
			cfg.type = venc_ctrls[idx].type;
			cfg.qmenu = venc_ctrls[idx].qmenu;
			ctrl = v4l2_ctrl_new_custom(&inst->ctrl_handler, &cfg,
						    NULL);
		} else {
			if (venc_ctrls[idx].type == V4L2_CTRL_TYPE_MENU) {
				ctrl = v4l2_ctrl_new_std_menu(
						&inst->ctrl_handler,
						&venc_ctrl_ops,
						venc_ctrls[idx].id,
						venc_ctrls[idx].maximum,
						venc_ctrls[idx].menu_skip_mask,
						venc_ctrls[idx].default_value);
			} else {
				ctrl = v4l2_ctrl_new_std(
						&inst->ctrl_handler,
						&venc_ctrl_ops,
						venc_ctrls[idx].id,
						venc_ctrls[idx].minimum,
						venc_ctrls[idx].maximum,
						venc_ctrls[idx].step,
						venc_ctrls[idx].default_value);
			}
		}

		ret = inst->ctrl_handler.error;
		if (ret) {
			dev_err(dev, "adding ctrl (%s) to ctrl handle (%d)\n",
				venc_ctrls[idx].name, ret);
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

	return 0;
}

void venc_ctrl_deinit(struct vidc_inst *inst)
{
	kfree(inst->ctrls);
	kfree(inst->cluster);
	v4l2_ctrl_handler_free(&inst->ctrl_handler);
}
