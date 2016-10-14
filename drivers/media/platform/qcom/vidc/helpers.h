/*
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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
#ifndef __VIDC_COMMON_H__
#define __VIDC_COMMON_H__

#include <linux/list.h>
#include <media/videobuf2-v4l2.h>

#include "core.h"

struct vidc_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t dma_addr;
	struct list_head hfi_list;
	struct hfi_buffer_desc bd;
};

#define to_vidc_buffer(buf)	container_of(buf, struct vidc_buffer, vb)

struct vb2_v4l2_buffer *
vidc_vb2_find_buf(struct vidc_inst *inst, dma_addr_t addr);
int vidc_vb2_buf_init(struct vb2_buffer *vb);
int vidc_vb2_buf_prepare(struct vb2_buffer *vb);
void vidc_vb2_buf_queue(struct vb2_buffer *vb);
void vidc_vb2_stop_streaming(struct vb2_queue *q);
int vidc_vb2_start_streaming(struct vidc_inst *inst);
int vidc_get_bufreq(struct vidc_inst *inst, u32 type,
		    struct hfi_buffer_requirements *out);
int vidc_set_color_format(struct vidc_inst *inst, u32 type, u32 fmt);
#endif
