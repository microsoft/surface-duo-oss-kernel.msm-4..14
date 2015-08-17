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

#ifndef _MSM_VIDC_COMMON_H_
#define _MSM_VIDC_COMMON_H_

#include <media/msm-v4l2-controls.h>

#include "msm_vidc_internal.h"

struct getprop_buf {
	struct list_head list;
	void *data;
};

struct vidc_core *vidc_get_core(int core_id);

int hfi_session_set_property(struct vidc_inst *inst, enum hal_property ptype,
			     void *pdata);
int hfi_session_get_property(struct vidc_inst *inst, enum hal_property ptype,
			     union hal_get_property *hprop);
void hfi_session_clean(struct vidc_inst *inst);

int vidc_comm_session_flush(struct vidc_inst *inst, u32 flags);
int vidc_comm_get_bufreqs(struct vidc_inst *inst);
struct hal_buffer_requirements *get_buff_req_buffer(struct vidc_inst *inst,
						    u32 buffer_type);
int vidc_comm_suspend(struct vidc_core *core);
enum hal_extradata_id
vidc_comm_get_hal_extradata_index(enum v4l2_mpeg_vidc_extradata index);
enum hal_buffer_layout_type
vidc_comm_get_hal_buffer_layout(enum v4l2_mpeg_vidc_video_mvc_layout index);
enum multi_stream vidc_comm_get_stream_output_mode(struct vidc_inst *inst);
enum hal_buffer vidc_comm_get_hal_output_buffer(struct vidc_inst *inst);
enum hal_video_codec vidc_comm_hal_codec_type(int fourcc);
enum hal_domain vidc_comm_get_hal_domain(int session_type);
int vidc_comm_set_color_format(struct vidc_inst *inst,
			       enum hal_buffer buffer_type, u32 fourcc);
void vidc_change_inst_state(struct vidc_inst *inst, u32 state);
int vidc_comm_kill_session(struct vidc_inst *inst);

#endif
