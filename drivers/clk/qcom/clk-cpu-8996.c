/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "clk-alpha-pll.h"
#include "clk-pll.h"
#include "clk-regmap.h"
#include "clk-regmap-mux.h"

#define VCO(a, b, c) { \
	.val = a,\
	.min_freq = b,\
	.max_freq = c,\
}

static const struct alpha_pll_config hfpll_config = {
	.l = 60,
	.config_ctl_val = 0x200d4828,
	.config_ctl_hi_val = 0x006,
	.pre_div_mask = BIT(12),
	.post_div_mask = 0x3 << 8,
	.main_output_mask = BIT(0),
	.early_output_mask = BIT(3),
};

static struct clk_alpha_pll perfcl_pll = {
	.offset = 0x80000,
	.min_rate = 600000000,
	.max_rate = 3000000000,
	.flags = SUPPORTS_DYNAMIC_UPDATE | SUPPORTS_16BIT_ALPHA
			| SUPPORTS_FSM_MODE,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "perfcl_pll",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_alpha_pll_hwfsm_ops,
	},
};

static struct clk_alpha_pll pwrcl_pll = {
	.offset = 0x0,
	.min_rate = 600000000,
	.max_rate = 3000000000,
	.flags = SUPPORTS_DYNAMIC_UPDATE | SUPPORTS_16BIT_ALPHA
			| SUPPORTS_FSM_MODE,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "pwrcl_pll",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_alpha_pll_hwfsm_ops,
	},
};

static const struct pll_vco alt_pll_vco_modes[] = {
	VCO(3,  250000000,  500000000),
	VCO(2,  500000000,  750000000),
	VCO(1,  750000000, 1000000000),
	VCO(0, 1000000000, 2150400000),
};

static const struct alpha_pll_config altpll_config = {
	.l = 16,
	.vco_val = 0x3 << 20,
	.vco_mask = 0x3 << 20,
	.config_ctl_val = 0x4001051b,
	.post_div_mask = 0x3 << 8,
	.post_div_val = 0x1,
	.main_output_mask = BIT(0),
	.early_output_mask = BIT(3),
};

static struct clk_alpha_pll perfcl_alt_pll = {
	.offset = 0x80100,
	.vco_table = alt_pll_vco_modes,
	.num_vco = ARRAY_SIZE(alt_pll_vco_modes),
	.flags = SUPPORTS_OFFLINE_REQ | SUPPORTS_FSM_MODE,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "perfcl_alt_pll",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_alpha_pll_hwfsm_ops,
	},
};

static struct clk_alpha_pll pwrcl_alt_pll = {
	.offset = 0x100,
	.vco_table = alt_pll_vco_modes,
	.num_vco = ARRAY_SIZE(alt_pll_vco_modes),
	.flags = SUPPORTS_OFFLINE_REQ | SUPPORTS_FSM_MODE,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "pwrcl_alt_pll",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_alpha_pll_hwfsm_ops,
	},
};

static struct clk_regmap_mux pwrcl_pmux = {
	.reg = 0x40,
	.shift = 0,
	.width = 2,
	.table = (u32 []){0, 1, 3},
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "pwrcl_pmux",
		.parent_names = (const char *[]){
			"pwrcl_smux",
			"pwrcl_pll",
			"pwrcl_alt_pll",
		},
		.num_parents = 3,
		.ops = &clk_regmap_mux_closest_ops,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_regmap_mux pwrcl_smux = {
	.reg = 0x40,
	.shift = 2,
	.width = 2,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "pwrcl_smux",
		.parent_names = (const char *[]){
			"xo",
			"pwrcl_pll_main",
			"sys_apcscbf_clk",
			"sys_apcsaux_clk",
		},
		.num_parents = 4,
		.ops = &clk_regmap_mux_closest_ops,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_regmap_mux perfcl_pmux = {
	.reg = 0x80040,
	.shift = 0,
	.width = 2,
	.table = (u32 []){0, 1, 3},
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "perfcl_pmux",
		.parent_names = (const char *[]){
			"perfcl_smux",
			"perfcl_pll",
			"perfcl_alt_pll",
		},
		.num_parents = 3,
		.ops = &clk_regmap_mux_closest_ops,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_regmap_mux perfcl_smux = {
	.reg = 0x80040,
	.shift = 2,
	.width = 2,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "perfcl_smux",
		.parent_names = (const char *[]){
			"xo",
			"perfcl_pll_main",
			"sys_apcscbf_clk",
			"sys_apcsaux_clk",
		},
		.num_parents = 4,
		.ops = &clk_regmap_mux_closest_ops,
		.flags = CLK_SET_RATE_PARENT,
	},
};

struct clk_cpu_8996 {
	struct clk_hw *alt_clk;
	struct clk_hw *pll;
	struct clk_regmap clkr;
};

static inline struct clk_cpu_8996 *to_clk_cpu_8996(struct clk_hw *hw)
{
	return container_of(to_clk_regmap(hw), struct clk_cpu_8996, clkr);
}

static int clk_cpu_8996_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long prate)
{
	int ret;
	struct clk_cpu_8996 *cpuclk = to_clk_cpu_8996(hw);
	struct clk *alt_clk, *pll, *parent;

	alt_clk = clk_hw_get_clk(cpuclk->alt_clk);
	pll = clk_hw_get_clk(cpuclk->pll);
	parent = clk_hw_get_clk(clk_hw_get_parent(hw));

	/* Switch parent to alt clk */
	if (cpuclk->alt_clk) {
		ret = clk_set_parent(parent, alt_clk);
		if (ret)
			return ret;
	}

	/* Set the PLL to new rate */
	ret = clk_set_rate(pll, rate);
	if (ret)
		goto error;

	/* Switch back to primary pll */
	if (cpuclk->alt_clk) {
		ret = clk_set_parent(parent, pll);
		if (ret)
			goto error;
	}
	return 0;

error:
	if (cpuclk->alt_clk)
		clk_set_parent(parent, pll);

	return ret;
}

static unsigned long clk_cpu_8996_recalc_rate(struct clk_hw *hw,
					      unsigned long prate)
{
	return clk_hw_get_rate(clk_hw_get_parent(hw));
}

static long clk_cpu_8996_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *prate)
{
	return clk_hw_round_rate(clk_hw_get_parent(hw), rate);
}

static struct clk_ops clk_cpu_8996_ops = {
	.set_rate = clk_cpu_8996_set_rate,
	.recalc_rate = clk_cpu_8996_recalc_rate,
	.round_rate = clk_cpu_8996_round_rate,
};

static struct clk_cpu_8996 pwrcl_clk = {
	.alt_clk = &pwrcl_alt_pll.clkr.hw,
	.pll = &pwrcl_pll.clkr.hw,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "pwrcl_clk",
		.parent_names = (const char *[]){ "pwrcl_pmux" },
		.num_parents = 1,
		.ops = &clk_cpu_8996_ops,
	},
};

static struct clk_cpu_8996 perfcl_clk = {
	.alt_clk = &perfcl_alt_pll.clkr.hw,
	.pll = &perfcl_pll.clkr.hw,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "perfcl_clk",
		.parent_names = (const char *[]){ "perfcl_pmux" },
		.num_parents = 1,
		.ops = &clk_cpu_8996_ops,
	},
};

static const struct regmap_config cpu_msm8996_regmap_config = {
	.reg_bits		= 32,
	.reg_stride		= 4,
	.val_bits		= 32,
	.max_register		= 0x80210,
	.fast_io		= true,
	.val_format_endian	= REGMAP_ENDIAN_LITTLE,
};

static const struct of_device_id match_table[] = {
	{ .compatible = "qcom,cpu-clk-msm8996" },
	{}
};

struct clk_regmap *clks[] = {
	/* PLLs */
	&perfcl_pll.clkr,
	&pwrcl_pll.clkr,
	&perfcl_alt_pll.clkr,
	&pwrcl_alt_pll.clkr,
	/* MUXs */
	&perfcl_pmux.clkr,
	&perfcl_smux.clkr,
	&pwrcl_pmux.clkr,
	&pwrcl_smux.clkr,
	/* CPU clks */
	&perfcl_clk.clkr,
	&pwrcl_clk.clkr,
};

static int register_cpu_clocks(struct device *dev, struct regmap *regmap)
{
	int ret, i;
	struct clk *perf_pll, *pwr_pll, *perf_alt_pll, *pwr_alt_pll;
	struct clk *perf_clk, *pwr_clk;

	for (i = 0; i < ARRAY_SIZE(clks); i++) {
		ret = devm_clk_register_regmap(dev, clks[i]);
		if (ret)
			return ret;
	}

	clk_hw_register_fixed_factor(dev, "perfcl_pll_main", "perfcl_pll",
				     CLK_SET_RATE_PARENT, 1, 2);
	clk_hw_register_fixed_factor(dev, "pwrcl_pll_main", "pwrcl_pll",
				     CLK_SET_RATE_PARENT, 1, 2);

	clk_alpha_pll_configure(&perfcl_pll, regmap, &hfpll_config);
	clk_alpha_pll_configure(&pwrcl_pll, regmap, &hfpll_config);
	clk_alpha_pll_configure(&perfcl_alt_pll, regmap, &altpll_config);
	clk_alpha_pll_configure(&pwrcl_alt_pll, regmap, &altpll_config);

	perf_pll = clk_hw_get_clk(&perfcl_pll.clkr.hw);
	pwr_pll = clk_hw_get_clk(&pwrcl_pll.clkr.hw);
	perf_alt_pll = clk_hw_get_clk(&perfcl_alt_pll.clkr.hw);
	pwr_alt_pll = clk_hw_get_clk(&pwrcl_alt_pll.clkr.hw);
	pwr_clk = clk_hw_get_clk(&pwrcl_clk.clkr.hw);
	perf_clk = clk_hw_get_clk(&perfcl_clk.clkr.hw);

	/* Enable all PLLs and alt PLLs */
	clk_prepare_enable(perf_pll);
	clk_prepare_enable(pwr_pll);
	clk_prepare_enable(perf_alt_pll);
	clk_prepare_enable(pwr_alt_pll);

	/* init boot frequency setting for perf/pwr cluster */
	clk_set_rate(pwr_clk, 1228800000);
	clk_set_rate(perf_clk, 1555200000);

	return 0;
}

static int qcom_cpu_clk_msm8996_driver_probe(struct platform_device *pdev)
{
	int ret;
	void __iomem *base;
	struct resource *res;
	struct clk_hw_onecell_data *data;
	struct device *dev = &pdev->dev;
	struct regmap *regmap_cpu;


	data = devm_kzalloc(dev, sizeof(*data) + 2 * sizeof(struct clk_hw *),
		            GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap_cpu = devm_regmap_init_mmio(dev, base,
					   &cpu_msm8996_regmap_config);
	if (IS_ERR(regmap_cpu))
		return PTR_ERR(regmap_cpu);

	ret = register_cpu_clocks(dev, regmap_cpu);
	if (ret)
		return ret;

	data->hws[0] = &pwrcl_clk.clkr.hw;
	data->hws[1] = &perfcl_clk.clkr.hw;
	data->num = 2;

	return of_clk_add_hw_provider(dev->of_node, of_clk_hw_onecell_get, data);
}

static struct platform_driver qcom_cpu_clk_msm8996_driver = {
	.probe = qcom_cpu_clk_msm8996_driver_probe,
	.driver = {
		.name = "qcom-cpu-clk-msm8996",
		.of_match_table = match_table,
	},
};

builtin_platform_driver(qcom_cpu_clk_msm8996_driver);
