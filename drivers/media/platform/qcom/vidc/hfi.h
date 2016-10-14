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
#ifndef __HFI_H__
#define __HFI_H__

#include <linux/interrupt.h>

#include "hfi_helper.h"

#define VIDC_SESSION_TYPE_VPE			0
#define VIDC_SESSION_TYPE_ENC			1
#define VIDC_SESSION_TYPE_DEC			2

#define VIDC_RESOURCE_NONE			0
#define VIDC_RESOURCE_OCMEM			1
#define VIDC_RESOURCE_VMEM			2

struct hfi_buffer_desc {
	u32 buffer_type;
	u32 buffer_size;
	u32 num_buffers;
	u32 device_addr;
	u32 extradata_addr;
	u32 extradata_size;
	u32 response_required;
};

struct hfi_frame_data {
	u32 buffer_type;
	u32 device_addr;
	u32 extradata_addr;
	u64 timestamp;
	u32 flags;
	u32 offset;
	u32 alloc_len;
	u32 filled_len;
	u32 mark_target;
	u32 mark_data;
	u32 clnt_data;
	u32 extradata_size;
};

union hfi_get_property {
	struct hfi_profile_level profile_level;
	struct hfi_buffer_requirements bufreq[HFI_BUFFER_TYPE_MAX];
};

/* HFI events */
#define EVT_SYS_EVENT_CHANGE			1
#define EVT_SYS_WATCHDOG_TIMEOUT		2
#define EVT_SYS_ERROR				3
#define EVT_SESSION_ERROR			4

/* HFI event callback structure */
struct hfi_event_data {
	u32 error;
	u32 height;
	u32 width;
	u32 event_type;
	u32 packet_buffer;
	u32 extradata_buffer;
	u32 profile;
	u32 level;
};

/* define core states */
#define CORE_UNINIT				0
#define CORE_INIT				1
#define CORE_INVALID				2

/* define instance states */
#define INST_INVALID				1
#define INST_UNINIT				2
#define INST_INIT				3
#define INST_LOAD_RESOURCES			4
#define INST_START				5
#define INST_STOP				6
#define INST_RELEASE_RESOURCES			7

#define call_hfi_op(hfi, op, args...)	\
	(((hfi) && (hfi)->ops && (hfi)->ops->op) ?	\
	((hfi)->ops->op(args)) : 0)

struct vidc_core;
struct vidc_inst;

struct hfi_core_ops {
	int (*event_notify)(struct vidc_core *core, u32 event);
};

struct hfi_inst_ops {
	int (*empty_buf_done)(struct vidc_inst *inst, u32 addr, u32 bytesused,
			      u32 data_offset, u32 flags);
	int (*fill_buf_done)(struct vidc_inst *inst, u32 addr, u32 bytesused,
			     u32 data_offset, u32 flags, u64 timestamp_us);
	int (*event_notify)(struct vidc_inst *inst, u32 event,
			    struct hfi_event_data *data);
};

struct hfi_ops {
	int (*core_init)(struct vidc_core *core);
	int (*core_deinit)(struct vidc_core *core);
	int (*core_ping)(struct vidc_core *core, u32 cookie);
	int (*core_trigger_ssr)(struct vidc_core *core, u32 trigger_type);

	int (*session_init)(struct vidc_core *core, struct vidc_inst *inst,
			    u32 session_type, u32 codec);
	int (*session_end)(struct vidc_inst *inst);
	int (*session_abort)(struct vidc_inst *inst);
	int (*session_flush)(struct vidc_inst *inst, u32 flush_mode);
	int (*session_start)(struct vidc_inst *inst);
	int (*session_stop)(struct vidc_inst *inst);
	int (*session_continue)(struct vidc_inst *inst);
	int (*session_etb)(struct vidc_inst *inst,
			   struct hfi_frame_data *input_frame);
	int (*session_ftb)(struct vidc_inst *inst,
			   struct hfi_frame_data *output_frame);
	int (*session_set_buffers)(struct vidc_inst *inst,
				   struct hfi_buffer_desc *bd);
	int (*session_unset_buffers)(struct vidc_inst *inst,
				     struct hfi_buffer_desc *bd);
	int (*session_load_res)(struct vidc_inst *inst);
	int (*session_release_res)(struct vidc_inst *inst);
	int (*session_parse_seq_hdr)(struct vidc_inst *inst, u32 seq_hdr,
				     u32 seq_hdr_len);
	int (*session_get_seq_hdr)(struct vidc_inst *inst, u32 seq_hdr,
				   u32 seq_hdr_len);
	int (*session_set_property)(struct vidc_inst *inst, u32 ptype,
				    void *pdata);
	int (*session_get_property)(struct vidc_inst *inst, u32 ptype);

	int (*resume)(struct vidc_core *core);
	int (*suspend)(struct vidc_core *core);

	/* interrupt operations */
	irqreturn_t (*isr)(struct vidc_core *core);
	irqreturn_t (*isr_thread)(struct vidc_core *core);
};

int hfi_create(struct vidc_core *core);
void hfi_destroy(struct vidc_core *core);

int hfi_core_init(struct vidc_core *core);
int hfi_core_deinit(struct vidc_core *core);
int hfi_core_suspend(struct vidc_core *core);
int hfi_core_resume(struct vidc_core *core);
int hfi_core_trigger_ssr(struct vidc_core *core, u32 type);
int hfi_core_ping(struct vidc_core *core);
int hfi_session_create(struct vidc_inst *inst, const struct hfi_inst_ops *ops);
void hfi_session_destroy(struct vidc_inst *inst);
int hfi_session_init(struct vidc_inst *inst, u32 pixfmt, u32 session_type);
int hfi_session_deinit(struct vidc_inst *inst);
int hfi_session_start(struct vidc_inst *inst);
int hfi_session_stop(struct vidc_inst *inst);
int hfi_session_continue(struct vidc_inst *inst);
int hfi_session_abort(struct vidc_inst *inst);
int hfi_session_load_res(struct vidc_inst *inst);
int hfi_session_unload_res(struct vidc_inst *inst);
int hfi_session_flush(struct vidc_inst *inst);
int hfi_session_set_buffers(struct vidc_inst *inst, struct hfi_buffer_desc *bd);
int hfi_session_unset_buffers(struct vidc_inst *inst,
			      struct hfi_buffer_desc *bd);
int hfi_session_get_property(struct vidc_inst *inst, u32 ptype,
			     union hfi_get_property *hprop);
int hfi_session_set_property(struct vidc_inst *inst, u32 ptype, void *pdata);
int hfi_session_etb(struct vidc_inst *inst, struct hfi_frame_data *fdata);
int hfi_session_ftb(struct vidc_inst *inst, struct hfi_frame_data *fdata);
irqreturn_t hfi_isr_thread(struct vidc_core *);
irqreturn_t hfi_isr(struct vidc_core *);

#endif
