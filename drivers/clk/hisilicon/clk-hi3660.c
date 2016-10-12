/*
 * Hisilicon Hi3660 Clock Driver
 *
 * Copyright (c) 2016, Hisilicon Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#include <dt-bindings/clock/hi3660-clock.h>

#include "clk.h"

static struct hisi_fixed_rate_clock hi3660_fixed_rate_clks[] __initdata = {
	{ HI3660_REF32K,	"ref32k",	NULL, CLK_IS_ROOT, 32764,     },
	{ HI3660_CLK_TCXO,	"clk_tcxo",	NULL, CLK_IS_ROOT, 19200000,  },
};

static struct hisi_gate_clock hi3660_separated_gate_clks_crg[] __initdata = {
	{ HI3660_UART5_PCLK,    "uart5_pclk",    "clk_tcxo",      CLK_SET_RATE_PARENT, 0x020, 15, 0, },
};

static void __init hi3660_clk_crg_init(struct device_node *np)
{
	struct hisi_clock_data *clk_data_crg;

	clk_data_crg = hisi_clk_init(np, HI3660_CRG_NR_CLKS);
	if (!clk_data_crg)
		return;

	hisi_clk_register_fixed_rate(hi3660_fixed_rate_clks,
				ARRAY_SIZE(hi3660_fixed_rate_clks),
				clk_data_crg);

	hisi_clk_register_gate_sep(hi3660_separated_gate_clks_crg,
				ARRAY_SIZE(hi3660_separated_gate_clks_crg),
				clk_data_crg);
}
CLK_OF_DECLARE(hi3660_clk_crg, "hisilicon,hi3660-crgctrl", hi3660_clk_crg_init);
