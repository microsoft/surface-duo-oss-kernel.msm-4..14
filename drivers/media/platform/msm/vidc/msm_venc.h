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
#ifndef _MSM_VENC_H_
#define _MSM_VENC_H_

#include "msm_vidc_internal.h"

int venc_init(struct vidc_core *core, struct video_device *enc);
void venc_deinit(struct vidc_core *core, struct video_device *enc);
int venc_open(struct vidc_inst *inst);
void venc_close(struct vidc_inst *inst);

int venc_set_bitrate_for_each_layer(struct vidc_inst *inst, u32 num_enh_layers,
				    u32 total_bitrate);
int venc_toggle_hier_p(struct vidc_inst *inst, int layers);

#endif
