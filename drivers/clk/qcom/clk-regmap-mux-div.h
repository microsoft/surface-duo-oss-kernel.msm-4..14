/*
 * Copyright (c) 2015, Linaro Limited
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __QCOM_CLK_REGMAP_MUX_DIV_H__
#define __QCOM_CLK_REGMAP_MUX_DIV_H__

#include <linux/clk-provider.h>
#include "clk-rcg.h"
#include "clk-regmap.h"

/**
 * struct mux_div_clk - combined mux/divider clock
 * @reg_offset: offset of the mux/divider register
 * @hid_width:	number of bits in half integer divider
 * @hid_shift:	lowest bit of hid value field
 * @src_width:	number of bits in source select
 * @src_shift:	lowest bit of source select field
 * @div:	the divider raw configuration value
 * @src:	the mux index which will be used if the clock is enabled
 * @parent_map:	pointer to parent_map struct
 * @clkr:	handle between common and hardware-specific interfaces
 * @clk_nb:	clock notifier registered for clock rate changes of the A53 PLL
 */

struct clk_regmap_mux_div {
	u32				reg_offset;
	u32				hid_width;
	u32				hid_shift;
	u32				src_width;
	u32				src_shift;
	u32				div;
	u32				src;
	const struct parent_map		*parent_map;
	struct clk_regmap		clkr;
	struct notifier_block		clk_nb;
};

extern const struct clk_ops clk_regmap_mux_div_ops;
int __mux_div_set_src_div(struct clk_regmap_mux_div *md, u32 src, u32 div);

#endif
