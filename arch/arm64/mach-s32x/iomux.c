/*
 * Copyright 2015 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/gpio.h>


#include "iomux.h"

static void __iomem *base;

/*
 * configures a single pad in the iomuxer
 */
int siul2_setup_pad(siul2_cfg_t pad)
{
	u32 mux_ctrl_ofs = (pad & MUX_CTRL_OFS_MASK) >> MUX_CTRL_OFS_SHIFT;
	u32 mux_pad_cfg = (pad & MUX_PAD_CFG_MASK) >> MUX_PAD_CFG_SHIFT;

	u32 pad_data_output_ofs = (pad & MUX_PAD_DATA_OUTPUT_OFS_MASK) >> MUX_PAD_DATA_OUTPUT_OFS_SHIFT;

	u32 data_output_mask = (pad & MUX_DATA_OUTPUT_VAL_MASK) >> MUX_DATA_OUTPUT_VAL_SHIFT;

	/* As specified in the manual, GPDO register should be configured
	 * before MSCR register for general purpose output pin */
	if (pad_data_output_ofs)
		__raw_writel(data_output_mask, base + SIUL2_GPDOn_OFFSET + pad_data_output_ofs);

	if (mux_ctrl_ofs)
		__raw_writel(mux_pad_cfg, base + SIUL2_MSCRn_OFFSET + mux_ctrl_ofs);


	return 0;
}

int siul2_setup_multiple_pads(siul2_cfg_t *pad_list, unsigned count)
{
	siul2_cfg_t *p = pad_list;
	int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = siul2_setup_pad(*p);
		if (ret)
			return ret;
		p++;
	}
	return 0;
}

void siul2_init(void __iomem *siul2_base)
{
	base = siul2_base;
}
