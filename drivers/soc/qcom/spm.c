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
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/slab.h>

#include "spm-drv.h"

#define NUM_SEQ_ENTRY 32
#define SPM_CTL_ENABLE BIT(0)

static u32 msm_spm_reg_offsets_saw2_v2_1[MSM_SPM_REG_NR] = {
	[MSM_SPM_REG_SAW2_SECURE]		= 0x00,
	[MSM_SPM_REG_SAW2_ID]			= 0x04,
	[MSM_SPM_REG_SAW2_CFG]			= 0x08,
	[MSM_SPM_REG_SAW2_SPM_STS]		= 0x0C,
	[MSM_SPM_REG_SAW2_AVS_STS]		= 0x10,
	[MSM_SPM_REG_SAW2_PMIC_STS]		= 0x14,
	[MSM_SPM_REG_SAW2_RST]			= 0x18,
	[MSM_SPM_REG_SAW2_VCTL]			= 0x1C,
	[MSM_SPM_REG_SAW2_AVS_CTL]		= 0x20,
	[MSM_SPM_REG_SAW2_AVS_LIMIT]		= 0x24,
	[MSM_SPM_REG_SAW2_AVS_DLY]		= 0x28,
	[MSM_SPM_REG_SAW2_AVS_HYSTERESIS]	= 0x2C,
	[MSM_SPM_REG_SAW2_SPM_CTL]		= 0x30,
	[MSM_SPM_REG_SAW2_SPM_DLY]		= 0x34,
	[MSM_SPM_REG_SAW2_PMIC_DATA_0]		= 0x40,
	[MSM_SPM_REG_SAW2_PMIC_DATA_1]		= 0x44,
	[MSM_SPM_REG_SAW2_PMIC_DATA_2]		= 0x48,
	[MSM_SPM_REG_SAW2_PMIC_DATA_3]		= 0x4C,
	[MSM_SPM_REG_SAW2_PMIC_DATA_4]		= 0x50,
	[MSM_SPM_REG_SAW2_PMIC_DATA_5]		= 0x54,
	[MSM_SPM_REG_SAW2_PMIC_DATA_6]		= 0x58,
	[MSM_SPM_REG_SAW2_PMIC_DATA_7]		= 0x5C,
	[MSM_SPM_REG_SAW2_SEQ_ENTRY]		= 0x80,
	[MSM_SPM_REG_SAW2_VERSION]		= 0xFD0,
};

static void flush_shadow(struct msm_spm_driver_data *drv, u32 reg_index)
{
	writel_relaxed(drv->reg_shadow[reg_index],
			drv->reg_base_addr + drv->reg_offsets[reg_index]);
}

static void load_shadow(struct msm_spm_driver_data *drv, u32 reg_index)
{
	drv->reg_shadow[reg_index] = readl_relaxed(drv->reg_base_addr +
						drv->reg_offsets[reg_index]);
}

static inline void set_start_addr(struct msm_spm_driver_data *drv, u32 addr)
{
	/* Update bits 10:4 in the SPM CTL register */
	addr &= 0x7F;
	addr <<= 4;
	drv->reg_shadow[MSM_SPM_REG_SAW2_SPM_CTL] &= 0xFFFFF80F;
	drv->reg_shadow[MSM_SPM_REG_SAW2_SPM_CTL] |= addr;
}

int msm_spm_drv_set_low_power_mode(struct msm_spm_driver_data *drv, u32 mode)
{
	int i;
	u32 start_addr = 0;

	for (i = 0; i < drv->num_modes; i++) {
		if (drv->modes[i].mode == mode) {
			start_addr = drv->modes[i].start_addr;
			break;
		}
	}

	if (i == drv->num_modes)
		return -EINVAL;

	set_start_addr(drv, start_addr);
	flush_shadow(drv, MSM_SPM_REG_SAW2_SPM_CTL);
	/* Barrier to ensure we have written the start address */
	wmb();

	/* Update our shadow with the status changes, if any */
	load_shadow(drv, MSM_SPM_REG_SAW2_SPM_STS);

	return 0;
}

int msm_spm_drv_set_spm_enable(struct msm_spm_driver_data *drv, bool enable)
{
	u32 value = enable ? 0x01 : 0x00;

	/* Update SPM_CTL to enable/disable the SPM */
	if ((drv->reg_shadow[MSM_SPM_REG_SAW2_SPM_CTL] & SPM_CTL_ENABLE)
								!= value) {
		/* Clear the existing value and update */
		drv->reg_shadow[MSM_SPM_REG_SAW2_SPM_CTL] &= ~0x1;
		drv->reg_shadow[MSM_SPM_REG_SAW2_SPM_CTL] |= value;
		flush_shadow(drv, MSM_SPM_REG_SAW2_SPM_CTL);
		/* Ensure we have enabled/disabled before returning */
		wmb();
	}

	return 0;
}

static void flush_seq_data(struct msm_spm_driver_data *drv, u32 *reg_seq_entry)
{
	int i;

	/* Write the 32 byte array into the SPM registers */
	for (i = 0; i < NUM_SEQ_ENTRY; i++) {
		writel_relaxed(reg_seq_entry[i],
			drv->reg_base_addr
			+ drv->reg_offsets[MSM_SPM_REG_SAW2_SEQ_ENTRY]
			+ 4 * i);
	}
	/* Ensure that the changes are written */
	wmb();
}

static void write_seq_data(struct msm_spm_driver_data *drv,
		u32 *reg_seq_entry, u8 *cmd, u32 *offset)
{
	u32 cmd_w;
	u32 offset_w = *offset / 4;
	u8 last_cmd;

	while (1) {
		int i;

		cmd_w = 0;
		last_cmd = 0;
		cmd_w = reg_seq_entry[offset_w];

		for (i = (*offset % 4); i < 4; i++) {
			last_cmd = *(cmd++);
			cmd_w |=  last_cmd << (i * 8);
			(*offset)++;
			if (last_cmd == 0x0f)
				break;
		}

		reg_seq_entry[offset_w++] = cmd_w;
		if (last_cmd == 0x0f)
			break;
	}

}

int msm_spm_drv_init(struct msm_spm_driver_data *drv)
{
	int i;
	int offset = 0;
	u32 sequences[NUM_SEQ_ENTRY/4] = {0};

	drv->reg_offsets = msm_spm_reg_offsets_saw2_v2_1;

	/**
	 * Compose the uint32 array based on the individual bytes of the SPM
	 * sequence for each low power mode that we read from the DT.
	 * The sequences are appended if there is space available in the
	 * u32 after the end of the previous sequence.
	 */
	for (i = 0; i < drv->num_modes; i++) {
		drv->modes[i].start_addr = offset;
		write_seq_data(drv, &sequences[0], drv->modes[i].cmd, &offset);
	}

	/* Flush the integer array */
	flush_seq_data(drv, &sequences[0]);

	/**
	 * Initialize the hardware with the control registers that
	 * we have read.
	 */
	for (i = 0; i < MSM_SPM_REG_SAW2_PMIC_DATA_0; i++)
		flush_shadow(drv, i);

	return 0;
}
