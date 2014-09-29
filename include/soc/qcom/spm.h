/* Copyright (c) 2010-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __QCOM_SPM_H
#define __QCOM_SPM_H

enum {
	MSM_SPM_MODE_DISABLED,
	MSM_SPM_MODE_CLOCK_GATING,
	MSM_SPM_MODE_RETENTION,
	MSM_SPM_MODE_GDHS,
	MSM_SPM_MODE_POWER_COLLAPSE,
	MSM_SPM_MODE_NR
};

struct msm_spm_device;

#if defined(CONFIG_QCOM_PM)

int msm_spm_set_low_power_mode(u32 mode);

#else

static inline int msm_spm_set_low_power_mode(u32 mode)
{ return -ENOSYS; }

#endif  /* CONFIG_QCOM_PM */

#endif  /* __QCOM_SPM_H */
