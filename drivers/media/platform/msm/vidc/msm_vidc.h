/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#ifndef _MSM_VIDC_H_
#define _MSM_VIDC_H_

#include <linux/poll.h>
#include <linux/videodev2.h>
#include <linux/types.h>
#include <linux/dma-attrs.h>

#define HAL_BUFFER_MAX	0xb

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

enum core_id {
	VIDC_CORE_VENUS = 0,
	VIDC_CORE_Q6,
	VIDC_CORES_MAX,
};

enum session_type {
	VIDC_ENCODER = 0,
	VIDC_DECODER,
	VIDC_MAX_DEVICES,
};

#endif
