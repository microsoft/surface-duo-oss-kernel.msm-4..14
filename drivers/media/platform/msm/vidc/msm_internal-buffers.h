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

#ifndef _MSM_VIDC_INTERNAL_BUFFERS_H_
#define _MSM_VIDC_INTERNAL_BUFFERS_H_

#include "msm_vidc_common.h"

int set_scratch_buffers(struct vidc_inst *);
int set_persist_buffers(struct vidc_inst *);
int release_persist_buffers(struct vidc_inst *);
int release_scratch_buffers(struct vidc_inst *, bool);

#endif /* _MSM_VIDC_INTERNAL_BUFFERS_H_ */
