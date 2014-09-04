/*
 * Copyright (c) 2009-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __QCOM_PM_H
#define __QCOM_PM_H

enum msm_pm_sleep_mode {
	MSM_PM_SLEEP_MODE_WFI,
	MSM_PM_SLEEP_MODE_RET,
	MSM_PM_SLEEP_MODE_SPC,
	MSM_PM_SLEEP_MODE_PC,
	MSM_PM_SLEEP_MODE_NR,
};

enum msm_pm_l2_scm_flag {
	MSM_SCM_L2_ON = 0,
	MSM_SCM_L2_OFF = 1
};

#endif  /* __QCOM_PM_H */
