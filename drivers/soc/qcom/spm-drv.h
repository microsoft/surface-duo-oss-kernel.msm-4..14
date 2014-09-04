/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
#ifndef __QCOM_SPM_DRIVER_H
#define __QCOM_SPM_DRIVER_H

enum {
	MSM_SPM_REG_SAW2_CFG,
	MSM_SPM_REG_SAW2_AVS_CTL,
	MSM_SPM_REG_SAW2_AVS_HYSTERESIS,
	MSM_SPM_REG_SAW2_SPM_CTL,
	MSM_SPM_REG_SAW2_PMIC_DLY,
	MSM_SPM_REG_SAW2_AVS_LIMIT,
	MSM_SPM_REG_SAW2_AVS_DLY,
	MSM_SPM_REG_SAW2_SPM_DLY,
	MSM_SPM_REG_SAW2_PMIC_DATA_0,
	MSM_SPM_REG_SAW2_PMIC_DATA_1,
	MSM_SPM_REG_SAW2_PMIC_DATA_2,
	MSM_SPM_REG_SAW2_PMIC_DATA_3,
	MSM_SPM_REG_SAW2_PMIC_DATA_4,
	MSM_SPM_REG_SAW2_PMIC_DATA_5,
	MSM_SPM_REG_SAW2_PMIC_DATA_6,
	MSM_SPM_REG_SAW2_PMIC_DATA_7,
	MSM_SPM_REG_SAW2_RST,

	MSM_SPM_REG_NR_INITIALIZE = MSM_SPM_REG_SAW2_RST,

	MSM_SPM_REG_SAW2_ID,
	MSM_SPM_REG_SAW2_SECURE,
	MSM_SPM_REG_SAW2_STS0,
	MSM_SPM_REG_SAW2_STS1,
	MSM_SPM_REG_SAW2_STS2,
	MSM_SPM_REG_SAW2_VCTL,
	MSM_SPM_REG_SAW2_SEQ_ENTRY,
	MSM_SPM_REG_SAW2_SPM_STS,
	MSM_SPM_REG_SAW2_AVS_STS,
	MSM_SPM_REG_SAW2_PMIC_STS,
	MSM_SPM_REG_SAW2_VERSION,

	MSM_SPM_REG_NR,
};

struct msm_spm_mode {
	u32 mode;
	u8 *cmd;
	u32 start_addr;
};

struct msm_spm_driver_data {
	void __iomem *reg_base_addr;
	u32 reg_shadow[MSM_SPM_REG_NR];
	u32 *reg_offsets;
	struct msm_spm_mode *modes;
	u32 num_modes;
};

int msm_spm_drv_init(struct msm_spm_driver_data *dev);
int msm_spm_drv_set_low_power_mode(struct msm_spm_driver_data *dev, u32 addr);
int msm_spm_drv_set_spm_enable(struct msm_spm_driver_data *dev, bool enable);

#endif /* __QCOM_SPM_DRIVER_H */
