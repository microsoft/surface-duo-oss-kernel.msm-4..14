/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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

#ifndef __MSM_VIDC_LOAD_H__
#define __MSM_VIDC_LOAD_H__

struct vidc_core;

void msm_comm_scale_clocks(struct vidc_core *core);
int msm_comm_check_overloaded(struct vidc_core *core);

#endif /* __MSM_VIDC_LOAD_H__ */
