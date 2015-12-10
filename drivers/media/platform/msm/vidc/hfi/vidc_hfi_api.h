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

#ifndef __VIDC_HFI_API_H__
#define __VIDC_HFI_API_H__

#include <linux/types.h>

#include "msm_vidc.h"
#include "msm_vidc_resources.h"

#include "vidc_hfi_helper.h"
#include "vidc_hfi.h"

#define HAL_BUFFERFLAG_EOS			0x00000001
#define HAL_BUFFERFLAG_STARTTIME		0x00000002
#define HAL_BUFFERFLAG_DECODEONLY		0x00000004
#define HAL_BUFFERFLAG_DATACORRUPT		0x00000008
#define HAL_BUFFERFLAG_ENDOFFRAME		0x00000010
#define HAL_BUFFERFLAG_SYNCFRAME		0x00000020
#define HAL_BUFFERFLAG_EXTRADATA		0x00000040
#define HAL_BUFFERFLAG_CODECCONFIG		0x00000080
#define HAL_BUFFERFLAG_TIMESTAMPINVALID		0x00000100
#define HAL_BUFFERFLAG_READONLY			0x00000200
#define HAL_BUFFERFLAG_ENDOFSUBFRAME		0x00000400
#define HAL_BUFFERFLAG_EOSEQ			0x00200000
#define HAL_BUFFERFLAG_MBAFF			0x08000000
#define HAL_BUFFERFLAG_YUV_601_709_CSC_CLAMP	0x10000000
#define HAL_BUFFERFLAG_DROP_FRAME		0x20000000
#define HAL_BUFFERFLAG_TS_DISCONTINUITY		0x40000000
#define HAL_BUFFERFLAG_TS_ERROR			0x80000000

#define HAL_DEBUG_MSG_LOW			0x00000001
#define HAL_DEBUG_MSG_MEDIUM			0x00000002
#define HAL_DEBUG_MSG_HIGH			0x00000004
#define HAL_DEBUG_MSG_ERROR			0x00000008
#define HAL_DEBUG_MSG_FATAL			0x00000010
#define MAX_PROFILE_COUNT			16

#define HAL_MAX_MATRIX_COEFFS			9
#define HAL_MAX_BIAS_COEFFS			3
#define HAL_MAX_LIMIT_COEFFS			6

enum vidc_error {
	VIDC_ERR_NONE = 0x0,
	VIDC_ERR_FAIL = 0x80000000,
	VIDC_ERR_ALLOC_FAIL,
	VIDC_ERR_ILLEGAL_OP,
	VIDC_ERR_BAD_PARAM,
	VIDC_ERR_BAD_HANDLE,
	VIDC_ERR_NOT_SUPPORTED,
	VIDC_ERR_BAD_STATE,
	VIDC_ERR_MAX_CLIENTS,
	VIDC_ERR_IFRAME_EXPECTED,
	VIDC_ERR_HW_FATAL,
	VIDC_ERR_BITSTREAM_ERR,
	VIDC_ERR_INDEX_NOMORE,
	VIDC_ERR_SEQHDR_PARSE_FAIL,
	VIDC_ERR_INSUFFICIENT_BUFFER,
	VIDC_ERR_BAD_POWER_STATE,
	VIDC_ERR_NO_VALID_SESSION,
	VIDC_ERR_TIMEOUT,
	VIDC_ERR_CMDQFULL,
	VIDC_ERR_START_CODE_NOT_FOUND,
	VIDC_ERR_CLIENT_PRESENT = 0x90000001,
	VIDC_ERR_CLIENT_FATAL,
	VIDC_ERR_CMD_QUEUE_FULL,
	VIDC_ERR_UNUSED = 0x10000000
};

#define HAL_BUFFER_MAX		0xb

/* NOTE: if you change this enum you MUST update the
 * "buffer-type-tz-usage-table" for any affected target
 * in arch/arm/boot/dts/<arch>.dtsi
 */
enum hal_buffer {
	HAL_BUFFER_NONE			= 0x0,
	HAL_BUFFER_INPUT		= 0x1,
	HAL_BUFFER_OUTPUT		= 0x2,
	HAL_BUFFER_OUTPUT2		= 0x4,
	HAL_BUFFER_EXTRADATA_INPUT	= 0x8,
	HAL_BUFFER_EXTRADATA_OUTPUT	= 0x10,
	HAL_BUFFER_EXTRADATA_OUTPUT2	= 0x20,
	HAL_BUFFER_INTERNAL_SCRATCH	= 0x40,
	HAL_BUFFER_INTERNAL_SCRATCH_1	= 0x80,
	HAL_BUFFER_INTERNAL_SCRATCH_2	= 0x100,
	HAL_BUFFER_INTERNAL_PERSIST	= 0x200,
	HAL_BUFFER_INTERNAL_PERSIST_1	= 0x400,
	HAL_BUFFER_INTERNAL_CMD_QUEUE	= 0x800,
};

enum hal_extradata_id {
	HAL_EXTRADATA_NONE,
	HAL_EXTRADATA_MB_QUANTIZATION,
	HAL_EXTRADATA_INTERLACE_VIDEO,
	HAL_EXTRADATA_VC1_FRAMEDISP,
	HAL_EXTRADATA_VC1_SEQDISP,
	HAL_EXTRADATA_TIMESTAMP,
	HAL_EXTRADATA_S3D_FRAME_PACKING,
	HAL_EXTRADATA_FRAME_RATE,
	HAL_EXTRADATA_PANSCAN_WINDOW,
	HAL_EXTRADATA_RECOVERY_POINT_SEI,
	HAL_EXTRADATA_MULTISLICE_INFO,
	HAL_EXTRADATA_INDEX,
	HAL_EXTRADATA_NUM_CONCEALED_MB,
	HAL_EXTRADATA_METADATA_FILLER,
	HAL_EXTRADATA_ASPECT_RATIO,
	HAL_EXTRADATA_MPEG2_SEQDISP,
	HAL_EXTRADATA_STREAM_USERDATA,
	HAL_EXTRADATA_FRAME_QP,
	HAL_EXTRADATA_FRAME_BITS_INFO,
	HAL_EXTRADATA_INPUT_CROP,
	HAL_EXTRADATA_DIGITAL_ZOOM,
	HAL_EXTRADATA_LTR_INFO,
	HAL_EXTRADATA_METADATA_MBI,
};

enum hal_property {
	HAL_CONFIG_FRAME_RATE = 0x04000001,
	HAL_PARAM_UNCOMPRESSED_FORMAT_SELECT,			//2
	HAL_PARAM_UNCOMPRESSED_PLANE_ACTUAL_CONSTRAINTS_INFO,	//3
	HAL_PARAM_UNCOMPRESSED_PLANE_ACTUAL_INFO,		//4
	HAL_PARAM_EXTRA_DATA_HEADER_CONFIG,			//5
	HAL_PARAM_INDEX_EXTRADATA,				//6
	HAL_PARAM_FRAME_SIZE,					//7
	HAL_CONFIG_REALTIME,					//8
	HAL_PARAM_BUFFER_COUNT_ACTUAL,				//9
	HAL_PARAM_BUFFER_SIZE_ACTUAL,				//a
	HAL_PARAM_BUFFER_DISPLAY_HOLD_COUNT_ACTUAL,		//b
	HAL_PARAM_NAL_STREAM_FORMAT_SELECT,			//c
	HAL_PARAM_VDEC_OUTPUT_ORDER,				//d
	HAL_PARAM_VDEC_PICTURE_TYPE_DECODE,			//e
	HAL_PARAM_VDEC_OUTPUT2_KEEP_ASPECT_RATIO,		//f
	HAL_CONFIG_VDEC_POST_LOOP_DEBLOCKER,			//10
	HAL_PARAM_VDEC_MULTI_STREAM,				//11
	HAL_PARAM_VDEC_DISPLAY_PICTURE_BUFFER_COUNT,		//12
	HAL_PARAM_DIVX_FORMAT,					//13
	HAL_CONFIG_VDEC_MB_ERROR_MAP_REPORTING,			//14
	HAL_PARAM_VDEC_CONTINUE_DATA_TRANSFER,			//15
	HAL_CONFIG_VDEC_MB_ERROR_MAP,				//16
	HAL_CONFIG_VENC_REQUEST_IFRAME,				//17
	HAL_PARAM_VENC_MPEG4_SHORT_HEADER,			//18
	HAL_PARAM_VENC_MPEG4_AC_PREDICTION,			//19
	HAL_CONFIG_VENC_TARGET_BITRATE,				//1a
	HAL_PARAM_PROFILE_LEVEL_CURRENT,			//1b
	HAL_PARAM_VENC_H264_ENTROPY_CONTROL,			//1c
	HAL_PARAM_VENC_RATE_CONTROL,				//1d
	HAL_PARAM_VENC_MPEG4_TIME_RESOLUTION,			//1e
	HAL_PARAM_VENC_MPEG4_HEADER_EXTENSION,			//1f
	HAL_PARAM_VENC_H264_DEBLOCK_CONTROL,			//20
	HAL_PARAM_VENC_TEMPORAL_SPATIAL_TRADEOFF,		//21
	HAL_PARAM_VENC_SESSION_QP,				//22
	HAL_PARAM_VENC_SESSION_QP_RANGE,			//23
	HAL_CONFIG_VENC_INTRA_PERIOD,				//24
	HAL_CONFIG_VENC_IDR_PERIOD,				//25
	HAL_CONFIG_VPE_OPERATIONS,				//26
	HAL_PARAM_VENC_INTRA_REFRESH,				//27
	HAL_PARAM_VENC_MULTI_SLICE_CONTROL,			//28
	HAL_CONFIG_VPE_DEINTERLACE,				//29
	HAL_SYS_DEBUG_CONFIG,					//2a
	HAL_CONFIG_BUFFER_REQUIREMENTS,				//2b
	HAL_CONFIG_PRIORITY,					//2c
	HAL_CONFIG_BATCH_INFO,					//2d
	HAL_PARAM_METADATA_PASS_THROUGH,			//2e
	HAL_SYS_IDLE_INDICATOR,					//2f
	HAL_PARAM_UNCOMPRESSED_FORMAT_SUPPORTED,		//30
	HAL_PARAM_INTERLACE_FORMAT_SUPPORTED,			//31
	HAL_PARAM_CHROMA_SITE,					//32
	HAL_PARAM_PROPERTIES_SUPPORTED,				//33
	HAL_PARAM_PROFILE_LEVEL_SUPPORTED,			//34
	HAL_PARAM_CAPABILITY_SUPPORTED,				//35
	HAL_PARAM_NAL_STREAM_FORMAT_SUPPORTED,			//36
	HAL_PARAM_MULTI_VIEW_FORMAT,				//37
	HAL_PARAM_MAX_SEQUENCE_HEADER_SIZE,			//38
	HAL_PARAM_CODEC_SUPPORTED,				//39
	HAL_PARAM_VDEC_MULTI_VIEW_SELECT,			//3a
	HAL_PARAM_VDEC_MB_QUANTIZATION,				//3b
	HAL_PARAM_VDEC_NUM_CONCEALED_MB,			//3c
	HAL_PARAM_VDEC_H264_ENTROPY_SWITCHING,			//3d
	HAL_PARAM_VENC_SLICE_DELIVERY_MODE,			//3e
	HAL_PARAM_VENC_MPEG4_DATA_PARTITIONING,			//3f
	HAL_CONFIG_BUFFER_COUNT_ACTUAL,				//40
	HAL_CONFIG_VDEC_MULTI_STREAM,				//41
	HAL_PARAM_VENC_MULTI_SLICE_INFO,			//42
	HAL_CONFIG_VENC_TIMESTAMP_SCALE,			//43
	HAL_PARAM_VENC_LOW_LATENCY,				//44
	HAL_PARAM_VENC_SYNC_FRAME_SEQUENCE_HEADER,		//45
	HAL_PARAM_VDEC_SYNC_FRAME_DECODE,			//46
	HAL_PARAM_VENC_H264_ENTROPY_CABAC_MODEL,		//47
	HAL_CONFIG_VENC_MAX_BITRATE,				//48
	HAL_PARAM_VENC_H264_VUI_TIMING_INFO,			//49
	HAL_PARAM_VENC_H264_GENERATE_AUDNAL,			//4a
	HAL_PARAM_VENC_MAX_NUM_B_FRAMES,			//4b
	HAL_PARAM_BUFFER_ALLOC_MODE,				//4c
	HAL_PARAM_VDEC_FRAME_ASSEMBLY,				//4d
	HAL_PARAM_VENC_H264_VUI_BITSTREAM_RESTRC,		//4e
	HAL_PARAM_VENC_PRESERVE_TEXT_QUALITY,			//4f
	HAL_PARAM_VDEC_CONCEAL_COLOR,				//50
	HAL_PARAM_VDEC_SCS_THRESHOLD,				//51
	HAL_PARAM_GET_BUFFER_REQUIREMENTS,			//52
	HAL_PARAM_MVC_BUFFER_LAYOUT,				//53
	HAL_PARAM_VENC_LTRMODE,					//54
	HAL_CONFIG_VENC_MARKLTRFRAME,				//55
	HAL_CONFIG_VENC_USELTRFRAME,				//56
	HAL_CONFIG_VENC_LTRPERIOD,				//57
	HAL_CONFIG_VENC_HIER_P_NUM_FRAMES,			//58
	HAL_PARAM_VENC_HIER_P_MAX_ENH_LAYERS,			//59
	HAL_PARAM_VENC_DISABLE_RC_TIMESTAMP,			//5a
	HAL_PARAM_VENC_ENABLE_INITIAL_QP,			//5b
	HAL_PARAM_VENC_SEARCH_RANGE,				//5c
	HAL_PARAM_VPE_COLOR_SPACE_CONVERSION,			//5d
	HAL_PARAM_VENC_VPX_ERROR_RESILIENCE_MODE,		//5e
	HAL_PARAM_VENC_H264_NAL_SVC_EXT,			//5f
	HAL_CONFIG_VENC_PERF_MODE,				//60
	HAL_PARAM_VENC_HIER_B_MAX_ENH_LAYERS,			//61
	HAL_PARAM_VDEC_NON_SECURE_OUTPUT2,			//62
	HAL_PARAM_VENC_HIER_P_HYBRID_MODE,			//63
};

enum hal_domain {
	HAL_VIDEO_DOMAIN_VPE,
	HAL_VIDEO_DOMAIN_ENCODER,
	HAL_VIDEO_DOMAIN_DECODER,
	HAL_UNUSED_DOMAIN = 0x10000000,
};

enum multi_stream {
	HAL_VIDEO_DECODER_NONE		= 0x00000000,
	HAL_VIDEO_DECODER_PRIMARY	= 0x00000001,
	HAL_VIDEO_DECODER_SECONDARY	= 0x00000002,
	HAL_VIDEO_DECODER_BOTH_OUTPUTS	= 0x00000004,
	HAL_VIDEO_UNUSED_OUTPUTS	= 0x10000000,
};

enum hal_core_capabilities {
	HAL_VIDEO_ENCODER_ROTATION_CAPABILITY		= 0x00000001,
	HAL_VIDEO_ENCODER_SCALING_CAPABILITY		= 0x00000002,
	HAL_VIDEO_ENCODER_DEINTERLACE_CAPABILITY	= 0x00000004,
	HAL_VIDEO_DECODER_MULTI_STREAM_CAPABILITY	= 0x00000008,
	HAL_VIDEO_UNUSED_CAPABILITY			= 0x10000000,
};

enum hal_default_properties {
	HAL_VIDEO_DYNAMIC_BUF_MODE		= 0x00000001,
	HAL_VIDEO_CONTINUE_DATA_TRANSFER	= 0x00000002,
};

enum hal_video_codec {
	HAL_VIDEO_CODEC_UNKNOWN		= 0x00000000,
	HAL_VIDEO_CODEC_MVC		= 0x00000001,
	HAL_VIDEO_CODEC_H264		= 0x00000002,
	HAL_VIDEO_CODEC_H263		= 0x00000004,
	HAL_VIDEO_CODEC_MPEG1		= 0x00000008,
	HAL_VIDEO_CODEC_MPEG2		= 0x00000010,
	HAL_VIDEO_CODEC_MPEG4		= 0x00000020,
	HAL_VIDEO_CODEC_DIVX_311	= 0x00000040,
	HAL_VIDEO_CODEC_DIVX		= 0x00000080,
	HAL_VIDEO_CODEC_VC1		= 0x00000100,
	HAL_VIDEO_CODEC_SPARK		= 0x00000200,
	HAL_VIDEO_CODEC_VP6		= 0x00000400,
	HAL_VIDEO_CODEC_VP7		= 0x00000800,
	HAL_VIDEO_CODEC_VP8		= 0x00001000,
	HAL_VIDEO_CODEC_HEVC		= 0x00002000,
	HAL_VIDEO_CODEC_HEVC_HYBRID	= 0x00004000,
	HAL_UNUSED_CODEC		= 0x10000000,
};

enum hal_h263_profile {
	HAL_H263_PROFILE_BASELINE		= 0x00000001,
	HAL_H263_PROFILE_H320CODING		= 0x00000002,
	HAL_H263_PROFILE_BACKWARDCOMPATIBLE	= 0x00000004,
	HAL_H263_PROFILE_ISWV2			= 0x00000008,
	HAL_H263_PROFILE_ISWV3			= 0x00000010,
	HAL_H263_PROFILE_HIGHCOMPRESSION	= 0x00000020,
	HAL_H263_PROFILE_INTERNET		= 0x00000040,
	HAL_H263_PROFILE_INTERLACE		= 0x00000080,
	HAL_H263_PROFILE_HIGHLATENCY		= 0x00000100,
	HAL_UNUSED_H263_PROFILE			= 0x10000000,
};

enum hal_h263_level {
	HAL_H263_LEVEL_10	= 0x00000001,
	HAL_H263_LEVEL_20	= 0x00000002,
	HAL_H263_LEVEL_30	= 0x00000004,
	HAL_H263_LEVEL_40	= 0x00000008,
	HAL_H263_LEVEL_45	= 0x00000010,
	HAL_H263_LEVEL_50	= 0x00000020,
	HAL_H263_LEVEL_60	= 0x00000040,
	HAL_H263_LEVEL_70	= 0x00000080,
	HAL_UNUSED_H263_LEVEL	= 0x10000000,
};

enum hal_mpeg2_profile {
	HAL_MPEG2_PROFILE_SIMPLE	= 0x00000001,
	HAL_MPEG2_PROFILE_MAIN		= 0x00000002,
	HAL_MPEG2_PROFILE_422		= 0x00000004,
	HAL_MPEG2_PROFILE_SNR		= 0x00000008,
	HAL_MPEG2_PROFILE_SPATIAL	= 0x00000010,
	HAL_MPEG2_PROFILE_HIGH		= 0x00000020,
	HAL_UNUSED_MPEG2_PROFILE	= 0x10000000,
};

enum hal_mpeg2_level {
	HAL_MPEG2_LEVEL_LL	= 0x00000001,
	HAL_MPEG2_LEVEL_ML	= 0x00000002,
	HAL_MPEG2_LEVEL_H14	= 0x00000004,
	HAL_MPEG2_LEVEL_HL	= 0x00000008,
	HAL_UNUSED_MEPG2_LEVEL	= 0x10000000,
};

enum hal_mpeg4_profile {
	HAL_MPEG4_PROFILE_SIMPLE		= 0x00000001,
	HAL_MPEG4_PROFILE_ADVANCEDSIMPLE	= 0x00000002,
	HAL_MPEG4_PROFILE_CORE			= 0x00000004,
	HAL_MPEG4_PROFILE_MAIN			= 0x00000008,
	HAL_MPEG4_PROFILE_NBIT			= 0x00000010,
	HAL_MPEG4_PROFILE_SCALABLETEXTURE	= 0x00000020,
	HAL_MPEG4_PROFILE_SIMPLEFACE		= 0x00000040,
	HAL_MPEG4_PROFILE_SIMPLEFBA		= 0x00000080,
	HAL_MPEG4_PROFILE_BASICANIMATED		= 0x00000100,
	HAL_MPEG4_PROFILE_HYBRID		= 0x00000200,
	HAL_MPEG4_PROFILE_ADVANCEDREALTIME	= 0x00000400,
	HAL_MPEG4_PROFILE_CORESCALABLE		= 0x00000800,
	HAL_MPEG4_PROFILE_ADVANCEDCODING	= 0x00001000,
	HAL_MPEG4_PROFILE_ADVANCEDCORE		= 0x00002000,
	HAL_MPEG4_PROFILE_ADVANCEDSCALABLE	= 0x00004000,
	HAL_MPEG4_PROFILE_SIMPLESCALABLE	= 0x00008000,
	HAL_UNUSED_MPEG4_PROFILE		= 0x10000000,
};

enum hal_mpeg4_level {
	HAL_MPEG4_LEVEL_0			= 0x00000001,
	HAL_MPEG4_LEVEL_0b			= 0x00000002,
	HAL_MPEG4_LEVEL_1			= 0x00000004,
	HAL_MPEG4_LEVEL_2			= 0x00000008,
	HAL_MPEG4_LEVEL_3			= 0x00000010,
	HAL_MPEG4_LEVEL_4			= 0x00000020,
	HAL_MPEG4_LEVEL_4a			= 0x00000040,
	HAL_MPEG4_LEVEL_5			= 0x00000080,
	HAL_MPEG4_LEVEL_VENDOR_START_UNUSED	= 0x7F000000,
	HAL_MPEG4_LEVEL_6			= 0x7F000001,
	HAL_MPEG4_LEVEL_7			= 0x7F000002,
	HAL_MPEG4_LEVEL_8			= 0x7F000003,
	HAL_MPEG4_LEVEL_9			= 0x7F000004,
	HAL_MPEG4_LEVEL_3b			= 0x7F000005,
	HAL_UNUSED_MPEG4_LEVEL			= 0x10000000,
};

enum hal_h264_profile {
	HAL_H264_PROFILE_BASELINE		= 0x00000001,
	HAL_H264_PROFILE_MAIN			= 0x00000002,
	HAL_H264_PROFILE_HIGH			= 0x00000004,
	HAL_H264_PROFILE_EXTENDED		= 0x00000008,
	HAL_H264_PROFILE_HIGH10			= 0x00000010,
	HAL_H264_PROFILE_HIGH422		= 0x00000020,
	HAL_H264_PROFILE_HIGH444		= 0x00000040,
	HAL_H264_PROFILE_CONSTRAINED_BASE	= 0x00000080,
	HAL_H264_PROFILE_CONSTRAINED_HIGH	= 0x00000100,
	HAL_UNUSED_H264_PROFILE			= 0x10000000,
};

enum hal_h264_level {
	HAL_H264_LEVEL_1	= 0x00000001,
	HAL_H264_LEVEL_1b	= 0x00000002,
	HAL_H264_LEVEL_11	= 0x00000004,
	HAL_H264_LEVEL_12	= 0x00000008,
	HAL_H264_LEVEL_13	= 0x00000010,
	HAL_H264_LEVEL_2	= 0x00000020,
	HAL_H264_LEVEL_21	= 0x00000040,
	HAL_H264_LEVEL_22	= 0x00000080,
	HAL_H264_LEVEL_3	= 0x00000100,
	HAL_H264_LEVEL_31	= 0x00000200,
	HAL_H264_LEVEL_32	= 0x00000400,
	HAL_H264_LEVEL_4	= 0x00000800,
	HAL_H264_LEVEL_41	= 0x00001000,
	HAL_H264_LEVEL_42	= 0x00002000,
	HAL_H264_LEVEL_5	= 0x00004000,
	HAL_H264_LEVEL_51	= 0x00008000,
	HAL_H264_LEVEL_52	= 0x00010000,
	HAL_UNUSED_H264_LEVEL	= 0x10000000,
};

enum hal_hevc_profile {
	HAL_HEVC_PROFILE_MAIN		= 0x00000001,
	HAL_HEVC_PROFILE_MAIN10		= 0x00000002,
	HAL_HEVC_PROFILE_MAIN_STILL_PIC	= 0x00000004,
	HAL_UNUSED_HEVC_PROFILE		= 0x10000000,
};

enum hal_hevc_level {
	HAL_HEVC_MAIN_TIER_LEVEL_1	= 0x10000001,
	HAL_HEVC_MAIN_TIER_LEVEL_2	= 0x10000002,
	HAL_HEVC_MAIN_TIER_LEVEL_2_1	= 0x10000004,
	HAL_HEVC_MAIN_TIER_LEVEL_3	= 0x10000008,
	HAL_HEVC_MAIN_TIER_LEVEL_3_1	= 0x10000010,
	HAL_HEVC_MAIN_TIER_LEVEL_4	= 0x10000020,
	HAL_HEVC_MAIN_TIER_LEVEL_4_1	= 0x10000040,
	HAL_HEVC_MAIN_TIER_LEVEL_5	= 0x10000080,
	HAL_HEVC_MAIN_TIER_LEVEL_5_1	= 0x10000100,
	HAL_HEVC_MAIN_TIER_LEVEL_5_2	= 0x10000200,
	HAL_HEVC_MAIN_TIER_LEVEL_6	= 0x10000400,
	HAL_HEVC_MAIN_TIER_LEVEL_6_1	= 0x10000800,
	HAL_HEVC_MAIN_TIER_LEVEL_6_2	= 0x10001000,
	HAL_HEVC_HIGH_TIER_LEVEL_1	= 0x20000001,
	HAL_HEVC_HIGH_TIER_LEVEL_2	= 0x20000002,
	HAL_HEVC_HIGH_TIER_LEVEL_2_1	= 0x20000004,
	HAL_HEVC_HIGH_TIER_LEVEL_3	= 0x20000008,
	HAL_HEVC_HIGH_TIER_LEVEL_3_1	= 0x20000010,
	HAL_HEVC_HIGH_TIER_LEVEL_4	= 0x20000020,
	HAL_HEVC_HIGH_TIER_LEVEL_4_1	= 0x20000040,
	HAL_HEVC_HIGH_TIER_LEVEL_5	= 0x20000080,
	HAL_HEVC_HIGH_TIER_LEVEL_5_1	= 0x20000100,
	HAL_HEVC_HIGH_TIER_LEVEL_5_2	= 0x20000200,
	HAL_HEVC_HIGH_TIER_LEVEL_6	= 0x20000400,
	HAL_HEVC_HIGH_TIER_LEVEL_6_1	= 0x20000800,
	HAL_HEVC_HIGH_TIER_LEVEL_6_2	= 0x20001000,
	HAL_UNUSED_HEVC_TIER_LEVEL	= 0x80000000,
};

enum hal_hevc_tier {
	HAL_HEVC_TIER_MAIN	= 0x00000001,
	HAL_HEVC_TIER_HIGH	= 0x00000002,
	HAL_UNUSED_HEVC_TIER	= 0x10000000,
};

enum hal_vpx_profile {
	HAL_VPX_PROFILE_SIMPLE		= 0x00000001,
	HAL_VPX_PROFILE_ADVANCED	= 0x00000002,
	HAL_VPX_PROFILE_VERSION_0	= 0x00000004,
	HAL_VPX_PROFILE_VERSION_1	= 0x00000008,
	HAL_VPX_PROFILE_VERSION_2	= 0x00000010,
	HAL_VPX_PROFILE_VERSION_3	= 0x00000020,
	HAL_VPX_PROFILE_UNUSED		= 0x10000000,
};

enum hal_vc1_profile {
	HAL_VC1_PROFILE_SIMPLE		= 0x00000001,
	HAL_VC1_PROFILE_MAIN		= 0x00000002,
	HAL_VC1_PROFILE_ADVANCED	= 0x00000004,
	HAL_UNUSED_VC1_PROFILE		= 0x10000000,
};

enum hal_vc1_level {
	HAL_VC1_LEVEL_LOW	= 0x00000001,
	HAL_VC1_LEVEL_MEDIUM	= 0x00000002,
	HAL_VC1_LEVEL_HIGH	= 0x00000004,
	HAL_VC1_LEVEL_0		= 0x00000008,
	HAL_VC1_LEVEL_1		= 0x00000010,
	HAL_VC1_LEVEL_2		= 0x00000020,
	HAL_VC1_LEVEL_3		= 0x00000040,
	HAL_VC1_LEVEL_4		= 0x00000080,
	HAL_UNUSED_VC1_LEVEL	= 0x10000000,
};

enum hal_divx_format {
	HAL_DIVX_FORMAT_4,
	HAL_DIVX_FORMAT_5,
	HAL_DIVX_FORMAT_6,
	HAL_UNUSED_DIVX_FORMAT	= 0x10000000,
};

enum hal_divx_profile {
	HAL_DIVX_PROFILE_QMOBILE	= 0x00000001,
	HAL_DIVX_PROFILE_MOBILE		= 0x00000002,
	HAL_DIVX_PROFILE_MT		= 0x00000004,
	HAL_DIVX_PROFILE_HT		= 0x00000008,
	HAL_DIVX_PROFILE_HD		= 0x00000010,
	HAL_UNUSED_DIVX_PROFILE		= 0x10000000,
};

enum hal_mvc_profile {
	HAL_MVC_PROFILE_STEREO_HIGH	= 0x00001000,
	HAL_UNUSED_MVC_PROFILE		= 0x10000000,
};

enum hal_mvc_level {
	HAL_MVC_LEVEL_1		= 0x00000001,
	HAL_MVC_LEVEL_1b	= 0x00000002,
	HAL_MVC_LEVEL_11	= 0x00000004,
	HAL_MVC_LEVEL_12	= 0x00000008,
	HAL_MVC_LEVEL_13	= 0x00000010,
	HAL_MVC_LEVEL_2		= 0x00000020,
	HAL_MVC_LEVEL_21	= 0x00000040,
	HAL_MVC_LEVEL_22	= 0x00000080,
	HAL_MVC_LEVEL_3		= 0x00000100,
	HAL_MVC_LEVEL_31	= 0x00000200,
	HAL_MVC_LEVEL_32	= 0x00000400,
	HAL_MVC_LEVEL_4		= 0x00000800,
	HAL_MVC_LEVEL_41	= 0x00001000,
	HAL_MVC_LEVEL_42	= 0x00002000,
	HAL_MVC_LEVEL_5		= 0x00004000,
	HAL_MVC_LEVEL_51	= 0x00008000,
	HAL_UNUSED_MVC_LEVEL	= 0x10000000,
};

struct hal_frame_rate {
	enum hal_buffer buffer_type;
	u32 frame_rate;
};

enum hal_uncompressed_format {
	HAL_COLOR_FORMAT_MONOCHROME	= 0x00000001,
	HAL_COLOR_FORMAT_NV12		= 0x00000002,
	HAL_COLOR_FORMAT_NV21		= 0x00000004,
	HAL_COLOR_FORMAT_NV12_4x4TILE	= 0x00000008,
	HAL_COLOR_FORMAT_NV21_4x4TILE	= 0x00000010,
	HAL_COLOR_FORMAT_YUYV		= 0x00000020,
	HAL_COLOR_FORMAT_YVYU		= 0x00000040,
	HAL_COLOR_FORMAT_UYVY		= 0x00000080,
	HAL_COLOR_FORMAT_VYUY		= 0x00000100,
	HAL_COLOR_FORMAT_RGB565		= 0x00000200,
	HAL_COLOR_FORMAT_BGR565		= 0x00000400,
	HAL_COLOR_FORMAT_RGB888		= 0x00000800,
	HAL_COLOR_FORMAT_BGR888		= 0x00001000,
	HAL_COLOR_FORMAT_NV12_UBWC	= 0x00002000,
	HAL_COLOR_FORMAT_NV12_TP10_UBWC	= 0x00004000,
	HAL_UNUSED_COLOR		= 0x10000000,
};

enum hal_ssr_trigger_type {
	SSR_ERR_FATAL		= 1,
	SSR_SW_DIV_BY_ZERO,
	SSR_HW_WDOG_IRQ,
};

enum vidc_extradata_type {
	VIDC_EXTRADATA_NONE = 0x00000000,
	VIDC_EXTRADATA_MB_QUANTIZATION = 0x00000001,
	VIDC_EXTRADATA_INTERLACE_VIDEO = 0x00000002,
	VIDC_EXTRADATA_VC1_FRAMEDISP = 0x00000003,
	VIDC_EXTRADATA_VC1_SEQDISP = 0x00000004,
	VIDC_EXTRADATA_TIMESTAMP = 0x00000005,
	VIDC_EXTRADATA_S3D_FRAME_PACKING = 0x00000006,
	VIDC_EXTRADATA_FRAME_RATE = 0x00000007,
	VIDC_EXTRADATA_PANSCAN_WINDOW = 0x00000008,
	VIDC_EXTRADATA_RECOVERY_POINT_SEI = 0x00000009,
	VIDC_EXTRADATA_MPEG2_SEQDISP = 0x0000000D,
	VIDC_EXTRADATA_STREAM_USERDATA = 0x0000000E,
	VIDC_EXTRADATA_FRAME_QP = 0x0000000F,
	VIDC_EXTRADATA_FRAME_BITS_INFO = 0x00000010,
	VIDC_EXTRADATA_INPUT_CROP = 0x0700000E,
	VIDC_EXTRADATA_DIGITAL_ZOOM = 0x07000010,
	VIDC_EXTRADATA_MULTISLICE_INFO = 0x7F100000,
	VIDC_EXTRADATA_NUM_CONCEALED_MB = 0x7F100001,
	VIDC_EXTRADATA_INDEX = 0x7F100002,
	VIDC_EXTRADATA_ASPECT_RATIO = 0x7F100003,
	VIDC_EXTRADATA_METADATA_LTR = 0x7F100004,
	VIDC_EXTRADATA_METADATA_FILLER = 0x7FE00002,
	VIDC_EXTRADATA_METADATA_MBI = 0x7F100005,
};

struct hal_uncompressed_format_select {
	enum hal_buffer buffer_type;
	enum hal_uncompressed_format format;
};

struct hal_uncompressed_plane_actual {
	int actual_stride;
	u32 actual_plane_buffer_height;
};

struct hal_uncompressed_plane_actual_info {
	enum hal_buffer buffer_type;
	u32 num_planes;
	struct hal_uncompressed_plane_actual rg_plane_format[1];
};

struct hal_uncompressed_plane_constraints {
	u32 stride_multiples;
	u32 max_stride;
	u32 min_plane_buffer_height_multiple;
	u32 buffer_alignment;
};

struct hal_uncompressed_plane_actual_constraints_info {
	enum hal_buffer buffer_type;
	u32 num_planes;
	struct hal_uncompressed_plane_constraints rg_plane_format[1];
};

struct hal_extra_data_header_config {
	u32 type;
	enum hal_buffer buffer_type;
	u32 version;
	u32 port_index;
	u32 client_extradata_id;
};

struct hal_frame_size {
	enum hal_buffer buffer_type;
	u32 width;
	u32 height;
};

struct hal_enable {
	u32 enable;
};

struct hal_buffer_count_actual {
	enum hal_buffer type;
	u32 count_actual;
};

struct hal_buffer_size_actual {
	enum hal_buffer type;
	u32 size;
};

struct hal_buffer_display_hold_count_actual {
	enum hal_buffer buffer_type;
	u32 hold_count;
};

enum hal_nal_stream_format {
	HAL_NAL_FORMAT_STARTCODES		= 0x00000001,
	HAL_NAL_FORMAT_ONE_NAL_PER_BUFFER	= 0x00000002,
	HAL_NAL_FORMAT_ONE_BYTE_LENGTH		= 0x00000004,
	HAL_NAL_FORMAT_TWO_BYTE_LENGTH		= 0x00000008,
	HAL_NAL_FORMAT_FOUR_BYTE_LENGTH		= 0x00000010,
};

enum hal_output_order {
	HAL_OUTPUT_ORDER_DISPLAY,
	HAL_OUTPUT_ORDER_DECODE,
	HAL_UNUSED_OUTPUT	= 0x10000000,
};

enum hal_picture {
	HAL_PICTURE_I		= 0x01,
	HAL_PICTURE_P		= 0x02,
	HAL_PICTURE_B		= 0x04,
	HAL_PICTURE_IDR		= 0x08,
	HAL_FRAME_NOTCODED	= 0x7F002000,
	HAL_FRAME_YUV		= 0x7F004000,
	HAL_UNUSED_PICT		= 0x10000000,
};

struct hal_extradata_enable {
	u32 enable;
	enum hal_extradata_id index;
};

struct hal_enable_picture {
	u32 picture_type;
};

struct hal_multi_stream {
	enum hal_buffer buffer_type;
	u32 enable;
	u32 width;
	u32 height;
};

struct hal_display_picture_buffer_count {
	u32 enable;
	u32 count;
};

struct hal_mb_error_map {
	u32 error_map_size;
	u8 rg_error_map[1];
};

struct hal_request_iframe {
	u32 enable;
};

struct hal_bitrate {
	u32 bit_rate;
	u32 layer_id;
};

struct hal_profile_level {
	u32 profile;
	u32 level;
};

struct hal_profile_level_supported {
	u32 profile_count;
	struct hal_profile_level profile_level[MAX_PROFILE_COUNT];
};

enum hal_h264_entropy {
	HAL_H264_ENTROPY_CAVLC	= 1,
	HAL_H264_ENTROPY_CABAC	= 2,
	HAL_UNUSED_ENTROPY	= 0x10000000,
};

enum hal_h264_cabac_model {
	HAL_H264_CABAC_MODEL_0	= 1,
	HAL_H264_CABAC_MODEL_1	= 2,
	HAL_H264_CABAC_MODEL_2	= 4,
	HAL_UNUSED_CABAC	= 0x10000000,
};

struct hal_h264_entropy_control {
	enum hal_h264_entropy entropy_mode;
	enum hal_h264_cabac_model cabac_model;
};

enum hal_rate_control {
	HAL_RATE_CONTROL_OFF,
	HAL_RATE_CONTROL_VBR_VFR,
	HAL_RATE_CONTROL_VBR_CFR,
	HAL_RATE_CONTROL_CBR_VFR,
	HAL_RATE_CONTROL_CBR_CFR,
	HAL_UNUSED_RC = 0x10000000,
};

struct hal_mpeg4_time_resolution {
	u32 time_increment_resolution;
};

struct hal_mpeg4_header_extension {
	u32 header_extension;
};

enum hal_h264_db_mode {
	HAL_H264_DB_MODE_DISABLE,
	HAL_H264_DB_MODE_SKIP_SLICE_BOUNDARY,
	HAL_H264_DB_MODE_ALL_BOUNDARY,
	HAL_UNUSED_H264_DB = 0x10000000,
};

struct hal_h264_db_control {
	enum hal_h264_db_mode mode;
	int slice_alpha_offset;
	int slice_beta_offset;
};

struct hal_temporal_spatial_tradeoff {
	u32 ts_factor;
};

struct hal_quantization {
	u32 qpi;
	u32 qpp;
	u32 qpb;
	u32 layer_id;
};

struct hal_initial_quantization {
	u32 qpi;
	u32 qpp;
	u32 qpb;
	u32 init_qp_enable;
};

struct hal_quantization_range {
	u32 min_qp;
	u32 max_qp;
	u32 layer_id;
};

struct hal_intra_period {
	u32 pframes;
	u32 bframes;
};

struct hal_idr_period {
	u32 idr_period;
};

enum hal_rotate {
	HAL_ROTATE_NONE,
	HAL_ROTATE_90,
	HAL_ROTATE_180,
	HAL_ROTATE_270,
	HAL_UNUSED_ROTATE = 0x10000000,
};

enum hal_flip {
	HAL_FLIP_NONE,
	HAL_FLIP_HORIZONTAL,
	HAL_FLIP_VERTICAL,
	HAL_UNUSED_FLIP = 0x10000000,
};

struct hal_operations {
	enum hal_rotate rotate;
	enum hal_flip flip;
};

enum hal_intra_refresh_mode {
	HAL_INTRA_REFRESH_NONE,
	HAL_INTRA_REFRESH_CYCLIC,
	HAL_INTRA_REFRESH_ADAPTIVE,
	HAL_INTRA_REFRESH_CYCLIC_ADAPTIVE,
	HAL_INTRA_REFRESH_RANDOM,
	HAL_UNUSED_INTRA = 0x10000000,
};

struct hal_intra_refresh {
	enum hal_intra_refresh_mode mode;
	u32 air_mbs;
	u32 air_ref;
	u32 cir_mbs;
};

enum hal_multi_slice {
	HAL_MULTI_SLICE_OFF,
	HAL_MULTI_SLICE_BY_MB_COUNT,
	HAL_MULTI_SLICE_BY_BYTE_COUNT,
	HAL_MULTI_SLICE_GOB,
	HAL_UNUSED_SLICE = 0x10000000,
};

struct hal_multi_slice_control {
	enum hal_multi_slice multi_slice;
	u32 slice_size;
};

struct hal_debug_config {
	u32 debug_config;
};

struct hal_buffer_requirements {
	enum hal_buffer type;
	u32 size;
	u32 region_size;
	u32 hold_count;
	u32 count_min;
	u32 count_actual;
	u32 contiguous;
	u32 alignment;
};

enum hal_priority {/* Priority increases with number */
	HAL_PRIORITY_LOW = 10,
	HAL_PRIOIRTY_MEDIUM = 20,
	HAL_PRIORITY_HIGH = 30,
	HAL_UNUSED_PRIORITY = 0x10000000,
};

struct hal_batch_info {
	u32 input_batch_count;
	u32 output_batch_count;
};

struct hal_metadata_pass_through {
	u32 enable;
	u32 size;
};

struct hal_uncompressed_format_supported {
	enum hal_buffer buffer_type;
	u32 format_entries;
	u32 rg_format_info[1];
};

enum hal_interlace_format {
	HAL_INTERLACE_FRAME_PROGRESSIVE			= 0x01,
	HAL_INTERLACE_INTERLEAVE_FRAME_TOPFIELDFIRST	= 0x02,
	HAL_INTERLACE_INTERLEAVE_FRAME_BOTTOMFIELDFIRST	= 0x04,
	HAL_INTERLACE_FRAME_TOPFIELDFIRST		= 0x08,
	HAL_INTERLACE_FRAME_BOTTOMFIELDFIRST		= 0x10,
	HAL_UNUSED_INTERLACE				= 0x10000000,
};

struct hal_interlace_format_supported {
	enum hal_buffer buffer_type;
	enum hal_interlace_format format;
};

enum hal_chroma_site {
	HAL_CHROMA_SITE_0,
	HAL_CHROMA_SITE_1,
	HAL_UNUSED_CHROMA = 0x10000000,
};

struct hal_properties_supported {
	u32 num_properties;
	u32 rg_properties[1];
};

enum hal_capability {
	HAL_CAPABILITY_FRAME_WIDTH,
	HAL_CAPABILITY_FRAME_HEIGHT,
	HAL_CAPABILITY_MBS_PER_FRAME,
	HAL_CAPABILITY_MBS_PER_SECOND,
	HAL_CAPABILITY_FRAMERATE,
	HAL_CAPABILITY_SCALE_X,
	HAL_CAPABILITY_SCALE_Y,
	HAL_CAPABILITY_BITRATE,
	HAL_CAPABILITY_SECURE_OUTPUT2_THRESHOLD,
	HAL_UNUSED_CAPABILITY = 0x10000000,
};

struct hal_capability_supported {
	enum hal_capability capability_type;
	u32 min;
	u32 max;
	u32 step_size;
};

struct hal_capability_supported_info {
	u32 num_capabilities;
	struct hal_capability_supported rg_data[1];
};

struct hal_nal_stream_format_supported {
	u32 nal_stream_format_supported;
};

struct hal_nal_stream_format_select {
	u32 nal_stream_format_select;
};

struct hal_multi_view_format {
	u32 views;
	u32 rg_view_order[1];
};

enum hal_buffer_layout_type {
	HAL_BUFFER_LAYOUT_TOP_BOTTOM,
	HAL_BUFFER_LAYOUT_SEQ,
	HAL_UNUSED_BUFFER_LAYOUT = 0x10000000,
};

struct hal_mvc_buffer_layout {
	enum hal_buffer_layout_type layout_type;
	u32 bright_view_first;
	u32 ngap;
};

struct hal_seq_header_info {
	u32 nax_header_len;
};

struct hal_codec_supported {
	u32 decoder_codec_supported;
	u32 encoder_codec_supported;
};

struct hal_multi_view_select {
	u32 view_index;
};

struct hal_timestamp_scale {
	u32 time_stamp_scale;
};


struct hal_h264_vui_timing_info {
	u32 enable;
	u32 fixed_frame_rate;
	u32 time_scale;
};

struct hal_h264_vui_bitstream_restrc {
	u32 enable;
};

struct hal_preserve_text_quality {
	u32 enable;
};

struct hal_vc1e_perf_cfg_type {
	struct {
		u32 x_subsampled;
		u32 y_subsampled;
	} i_frame, p_frame, b_frame;
};

struct hal_vpe_color_space_conversion {
	u32 csc_matrix[HAL_MAX_MATRIX_COEFFS];
	u32 csc_bias[HAL_MAX_BIAS_COEFFS];
	u32 csc_limit[HAL_MAX_LIMIT_COEFFS];
};

enum vidc_resource_id {
	VIDC_RESOURCE_NONE,
	VIDC_RESOURCE_OCMEM,
	VIDC_RESOURCE_VMEM,
	VIDC_UNUSED_RESOURCE = 0x10000000,
};

struct vidc_resource_hdr {
	enum vidc_resource_id resource_id;
	void *resource_handle;
	u32 size;
};

struct vidc_buffer_addr_info {
	enum hal_buffer buffer_type;
	u32 buffer_size;
	u32 num_buffers;
	u32 device_addr;
	u32 extradata_addr;
	u32 extradata_size;
	u32 response_required;
};

/* Needs to be exactly the same as hfi_buffer_info */
struct hal_buffer_info {
	u32 buffer_addr;
	u32 extra_data_addr;
};

struct vidc_frame_plane_config {
	u32 left;
	u32 top;
	u32 width;
	u32 height;
	u32 stride;
	u32 scan_lines;
};

struct vidc_uncompressed_frame_config {
	struct vidc_frame_plane_config luma_plane;
	struct vidc_frame_plane_config chroma_plane;
};

struct vidc_frame_data {
	enum hal_buffer buffer_type;
	u32 device_addr;
	u32 extradata_addr;
	int64_t timestamp;
	u32 flags;
	u32 offset;
	u32 alloc_len;
	u32 filled_len;
	u32 mark_target;
	u32 mark_data;
	u32 clnt_data;
	u32 extradata_size;
};

struct vidc_seq_hdr {
	u32 seq_hdr;
	u32 seq_hdr_len;
};

enum hal_flush {
	HAL_FLUSH_INPUT,
	HAL_FLUSH_OUTPUT,
	HAL_FLUSH_OUTPUT2,
	HAL_FLUSH_ALL,
	HAL_UNUSED_FLUSH = 0x10000000,
};

enum hal_event_type {
	HAL_EVENT_SEQ_CHANGED_SUFFICIENT_RESOURCES,
	HAL_EVENT_SEQ_CHANGED_INSUFFICIENT_RESOURCES,
	HAL_EVENT_RELEASE_BUFFER_REFERENCE,
	HAL_UNUSED_SEQCHG = 0x10000000,
};

enum hal_buffer_mode_type {
	HAL_BUFFER_MODE_STATIC = 0x001,
	HAL_BUFFER_MODE_RING = 0x010,
	HAL_BUFFER_MODE_DYNAMIC = 0x100,
};

struct hal_buffer_alloc_mode {
	enum hal_buffer type;
	enum hal_buffer_mode_type mode;
};

enum ltr_mode {
	HAL_LTR_MODE_DISABLE,
	HAL_LTR_MODE_MANUAL,
	HAL_LTR_MODE_PERIODIC,
};

struct hal_ltr_mode {
	enum ltr_mode mode;
	u32 count;
	u32 trust_mode;
};

struct hal_ltr_use {
	u32 ref_ltr;
	u32 use_constraint;
	u32 frames;
};

struct hal_ltr_mark {
	u32 mark_frame;
};

struct hal_venc_perf_mode {
	u32 mode;
};

struct hal_hybrid_hierp {
	u32 layers;
};

struct hfi_scs_threshold {
	u32 threshold_value;
};

struct buffer_requirements {
	struct hal_buffer_requirements buffer[HAL_BUFFER_MAX];
};

union hal_get_property {
	struct hal_frame_rate frame_rate;
	struct hal_uncompressed_format_select format_select;
	struct hal_uncompressed_plane_actual plane_actual;
	struct hal_uncompressed_plane_actual_info plane_actual_info;
	struct hal_uncompressed_plane_constraints plane_constraints;
	struct hal_uncompressed_plane_actual_constraints_info
						plane_constraints_info;
	struct hal_extra_data_header_config extra_data_header_config;
	struct hal_frame_size frame_size;
	struct hal_enable enable;
	struct hal_buffer_count_actual buffer_count_actual;
	struct hal_extradata_enable extradata_enable;
	struct hal_enable_picture enable_picture;
	struct hal_multi_stream multi_stream;
	struct hal_display_picture_buffer_count display_picture_buffer_count;
	struct hal_mb_error_map mb_error_map;
	struct hal_request_iframe request_iframe;
	struct hal_bitrate bitrate;
	struct hal_profile_level profile_level;
	struct hal_profile_level_supported profile_level_supported;
	struct hal_h264_entropy_control h264_entropy_control;
	struct hal_mpeg4_time_resolution mpeg4_time_resolution;
	struct hal_mpeg4_header_extension mpeg4_header_extension;
	struct hal_h264_db_control h264_db_control;
	struct hal_temporal_spatial_tradeoff temporal_spatial_tradeoff;
	struct hal_quantization quantization;
	struct hal_quantization_range quantization_range;
	struct hal_intra_period intra_period;
	struct hal_idr_period idr_period;
	struct hal_operations operations;
	struct hal_intra_refresh intra_refresh;
	struct hal_multi_slice_control multi_slice_control;
	struct hal_debug_config debug_config;
	struct hal_batch_info batch_info;
	struct hal_metadata_pass_through metadata_pass_through;
	struct hal_uncompressed_format_supported uncompressed_format_supported;
	struct hal_interlace_format_supported interlace_format_supported;
	struct hal_properties_supported properties_supported;
	struct hal_capability_supported capability_supported;
	struct hal_capability_supported_info capability_supported_info;
	struct hal_nal_stream_format_supported nal_stream_format_supported;
	struct hal_nal_stream_format_select nal_stream_format_select;
	struct hal_multi_view_format multi_view_format;
	struct hal_seq_header_info seq_header_info;
	struct hal_codec_supported codec_supported;
	struct hal_multi_view_select multi_view_select;
	struct hal_timestamp_scale timestamp_scale;
	struct hal_h264_vui_timing_info h264_vui_timing_info;
	struct hal_h264_vui_bitstream_restrc h264_vui_bitstream_restrc;
	struct hal_preserve_text_quality preserve_text_quality;
	struct hal_buffer_info buffer_info;
	struct hal_buffer_alloc_mode buffer_alloc_mode;
	struct buffer_requirements buf_req;
};

/* HAL Response */

enum command_response {
	/* SYSTEM COMMANDS_DONE*/
	SYS_EVENT_CHANGE,
	SYS_INIT_DONE,
	SYS_SET_RESOURCE_DONE,
	SYS_RELEASE_RESOURCE_DONE,
	SYS_PING_ACK_DONE,
	SYS_PC_PREP_DONE,
	SYS_IDLE,
	SYS_DEBUG,
	SYS_WATCHDOG_TIMEOUT,
	SYS_ERROR,
	/* SESSION COMMANDS_DONE */
	SESSION_LOAD_RESOURCE_DONE,
	SESSION_INIT_DONE,
	SESSION_END_DONE,
	SESSION_ABORT_DONE,
	SESSION_START_DONE,
	SESSION_STOP_DONE,
	SESSION_ETB_DONE,
	SESSION_FTB_DONE,
	SESSION_FLUSH_DONE,
	SESSION_SUSPEND_DONE,
	SESSION_RESUME_DONE,
	SESSION_SET_PROP_DONE,
	SESSION_GET_PROP_DONE,
	SESSION_PARSE_SEQ_HDR_DONE,
	SESSION_GET_SEQ_HDR_DONE,
	SESSION_RELEASE_BUFFER_DONE,
	SESSION_RELEASE_RESOURCE_DONE,
	SESSION_PROPERTY_INFO,
	SESSION_ERROR,
	RESPONSE_UNUSED = 0x10000000,
};

/* Event Callback structure */

struct vidc_cb_event {
	u32 device_id;
	u32 session_id;
	enum vidc_error error;
	u32 height;
	u32 width;
	u32 hal_event_type;
	u32 packet_buffer;
	u32 extra_data_buffer;
	u32 profile;
	u32 level;
};

/* Data callback structure */

struct vidc_hal_ebd {
	u32 timestamp_hi;
	u32 timestamp_lo;
	u32 flags;
	enum vidc_error error;
	u32 mark_target;
	u32 mark_data;
	u32 stats;
	u32 offset;
	u32 alloc_len;
	u32 filled_len;
	enum hal_picture picture_type;
	u32 packet_buffer;
	u32 extra_data_buffer;
};

struct vidc_hal_fbd {
	u32 stream_id;
	u32 view_id;
	u32 timestamp_hi;
	u32 timestamp_lo;
	u32 flags1;
	u32 mark_target;
	u32 mark_data;
	u32 stats;
	u32 alloc_len1;
	u32 filled_len1;
	u32 offset1;
	u32 frame_width;
	u32 frame_height;
	u32 start_x_coord;
	u32 start_y_coord;
	u32 input_tag;
	u32 input_tag1;
	enum hal_picture picture_type;
	u32 packet_buffer1;
	u32 extra_data_buffer;
	u32 flags2;
	u32 alloc_len2;
	u32 filled_len2;
	u32 offset2;
	u32 packet_buffer2;
	u32 flags3;
	u32 alloc_len3;
	u32 filled_len3;
	u32 offset3;
	u32 packet_buffer3;
	enum hal_buffer buffer_type;
};

struct vidc_cb_data_done {
	u32 device_id;
	void *session_id;
	enum vidc_error error;
	u32 size;
	u32 clnt_data;
	union {
		struct vidc_hal_ebd input_done;
		struct vidc_hal_fbd output_done;
	};
};

struct vidc_hal_sys_init_done {
	u32 enc_codec_supported;
	u32 dec_codec_supported;
};

struct vidc_hal_session_init_done {
	struct hal_capability_supported width;
	struct hal_capability_supported height;
	struct hal_capability_supported mbs_per_frame;
	struct hal_capability_supported mbs_per_sec;
	struct hal_capability_supported frame_rate;
	struct hal_capability_supported scale_x;
	struct hal_capability_supported scale_y;
	struct hal_capability_supported bitrate;
	struct hal_capability_supported hier_p;
	struct hal_capability_supported ltr_count;
	struct hal_capability_supported secure_output2_threshold;
	struct hal_uncompressed_format_supported uncomp_format;
	struct hal_interlace_format_supported HAL_format;
	struct hal_nal_stream_format_supported nal_stream_format;
	struct hal_profile_level_supported profile_level;
	/* allocate and released memory for above */
	struct hal_intra_refresh intra_refresh;
	struct hal_seq_header_info seq_hdr_info;
	enum hal_buffer_mode_type alloc_mode_out;
};

enum vidc_hfi_type {
	VIDC_HFI_VENUS,
	VIDC_HFI_Q6,
};

#define call_hfi_op(hdev, op, args...)	\
	(((hdev) && (hdev)->ops && (hdev)->ops->op) ?	\
	((hdev)->ops->op(args)) : 0)

struct hfi_ops {
	int (*core_init)(void *device);
	int (*core_release)(void *device);
	int (*core_ping)(void *device);
	int (*core_trigger_ssr)(void *device, enum hal_ssr_trigger_type);

	void *(*session_init)(void *device, void *session_id,
			      enum hal_domain session_type,
			      enum hal_video_codec codec_type);
	int (*session_end)(void *session);
	int (*session_abort)(void *session);
	int (*session_clean)(void *sess);
	int (*session_flush)(void *sess, enum hal_flush flush_mode);
	int (*session_start)(void *sess);
	int (*session_stop)(void *sess);
	int (*session_etb)(void *sess, struct vidc_frame_data *input_frame);
	int (*session_ftb)(void *sess, struct vidc_frame_data *output_frame);
	int (*session_set_buffers)(void *sess,
				   struct vidc_buffer_addr_info *bai);
	int (*session_release_buffers)(void *sess,
				       struct vidc_buffer_addr_info *bai);
	int (*session_load_res)(void *sess);
	int (*session_release_res)(void *sess);
	int (*session_parse_seq_hdr)(void *sess, struct vidc_seq_hdr *seq_hdr);
	int (*session_get_seq_hdr)(void *sess, struct vidc_seq_hdr *seq_hdr);
	int (*session_set_property)(void *sess, enum hal_property ptype,
				    void *pdata);
	int (*session_get_property)(void *sess, enum hal_property ptype);

	int (*get_stride_scanline)(int color_fmt, int width, int height,
				   int *stride, int *scanlines);
	int (*get_core_capabilities)(void);
	int (*resume)(void *dev);
	int (*suspend)(void *dev);
	enum hal_default_properties (*get_default_properties)(void *dev);
};

struct hfi_device {
	void *hfi_device_data;
	const struct hfi_ops *ops;
};

void *vidc_hfi_init(enum vidc_hfi_type hfi_type, u32 device_id,
		    struct vidc_resources *res);
void vidc_hfi_deinit(enum vidc_hfi_type hfi_type, struct hfi_device *hdev);

#endif /*__VIDC_HFI_API_H__ */
