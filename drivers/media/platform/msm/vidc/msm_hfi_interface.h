/*
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

#ifndef _MSM_HFI_INTERFACE_H_
#define _MSM_HFI_INTERFACE_H_

struct vidc_inst;
struct vidc_buffer_addr_info;

int hfi_core_init(struct vidc_core *core);
int hfi_core_deinit(struct vidc_core *core);

int hfi_session_init(struct vidc_inst *inst);
int hfi_session_deinit(struct vidc_inst *inst);
int hfi_session_start(struct vidc_inst *inst);
int hfi_session_stop(struct vidc_inst *inst);
int hfi_session_abort(struct vidc_inst *inst);
int hfi_session_load_res(struct vidc_inst *inst);
int hfi_session_release_res(struct vidc_inst *inst);
int hfi_session_flush(struct vidc_inst *inst);
int hfi_session_release_buffers(struct vidc_inst *inst,
				struct vidc_buffer_addr_info *bai);
#endif /* _MSM_HFI_INTERFACE_H_ */