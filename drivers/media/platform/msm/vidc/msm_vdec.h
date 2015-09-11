/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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
#ifndef _MSM_VDEC_H_
#define _MSM_VDEC_H_

#include "msm_vidc_internal.h"

int vdec_init(struct vidc_core *core, struct video_device *dec);
void vdec_deinit(struct vidc_core *core, struct video_device *dec);
int vdec_open(struct vidc_inst *inst);
void vdec_close(struct vidc_inst *inst);

#endif
