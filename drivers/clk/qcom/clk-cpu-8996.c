/*
 * Copyright (c) 2014-2016, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <asm/cputype.h>

#include "clk-alpha-pll.h"
#include "clk-regmap-mux.h"
#include "clk-regmap.h"

#define VCO(a, b, c) { \
	.val = a,\
	.min_freq = b,\
	.max_freq = c,\
}

static const struct pll_config hfpll_config = {
	.l = 60,
	.config_ctl_val = 0x200D4828,
	.config_ctl_hi_val = 0x006,
	.pre_div_mask = BIT(12),
	.post_div_mask = 0x3 << 8,
	.main_output_mask = BIT(0),
	.early_output_mask = BIT(3),
};

static struct clk_pll perfcl_pll = {
	.l_reg = 0x80004,
	.alpha_reg = 0x80008,
	.config_reg = 0x80010,
	.config_ctl_reg = 0x80018,
	.mode_reg = 0x80000,
	.status_reg = 0x80000,
	.status_bit = 31,
	.min_rate = 600000000,
	.max_rate = 3000000000,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "perfcl_pll",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_pll_hwfsm_ops,
	},
};

static struct clk_pll pwrcl_pll = {
	.l_reg = 0x0004,
	.alpha_reg = 0x0008,
	.config_reg = 0x0010,
	.config_ctl_reg = 0x0018,
	.mode_reg = 0x0000,
	.status_reg = 0x0000,
	.status_bit = 31,
	.min_rate = 600000000,
	.max_rate = 3000000000,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "pwrcl_pll",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_pll_hwfsm_ops,
	},
};

static const struct pll_config cbfpll_config = {
	.l = 32,
	.config_ctl_val = 0x200D4828,
	.config_ctl_hi_val = 0x006,
	.pre_div_mask = BIT(12),
	.post_div_mask = 0x3 << 8,
	.main_output_mask = BIT(0),
	.early_output_mask = BIT(3),
};

static struct clk_pll cbf_pll = {
	.l_reg = 0x0008,
	.alpha_reg = 0x0010,
	.config_reg = 0x0018,
	.config_ctl_reg = 0x0020,
	.mode_reg = 0x0000,
	.status_reg = 0x0000,
	.status_bit = 31,
	.min_rate = 600000000,
	.max_rate = 3000000000,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "cbf_pll",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_pll_hwfsm_ops,
	},
};

static const struct pll_vco alt_pll_vco_modes[] = {
	VCO(3,  250000000,  500000000),
	VCO(2,  500000000,  750000000),
	VCO(1,  750000000, 1000000000),
	VCO(0, 1000000000, 2150400000),
};

static const struct pll_config altpll_config = {
	.config_ctl_val = 0x4001051B,
	.post_div_mask = 0x3 << 8,
	.post_div_val = 0x1,
	.main_output_mask = BIT(0),
	.early_output_mask = BIT(3),
};

static struct clk_alpha_pll perfcl_alt_pll = {
	.offset = 0x80100,
	.vco_table = alt_pll_vco_modes,
	.num_vco = ARRAY_SIZE(alt_pll_vco_modes),
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
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "pwrcl_pmux",
		.parent_names = (const char *[]){
			"pwrcl_smux",
			"pwrcl_pll",
			"xo",
			"pwrcl_alt_pll",
		},
		.num_parents = 4,
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
			"xo",
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
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "perfcl_pmux",
		.parent_names = (const char *[]){
			"perfcl_smux",
			"perfcl_pll",
			"xo",
			"perfcl_alt_pll",
		},
		.num_parents = 4,
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
			"xo",
			"sys_apcsaux_clk",
		},
		.num_parents = 4,
		.ops = &clk_regmap_mux_closest_ops,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_regmap_mux cbf_pmux = {
	.reg = 0x18,
	.shift = 0,
	.width = 2,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "cbf_pmux",
		.parent_names = (const char *[]){
			"xo",
			"cbf_pll",
			"cbf_pll_main",
			"sys_apcsaux_clk",
		},
		.num_parents = 4,
		.ops = &clk_regmap_mux_closest_ops,
		.flags = CLK_SET_RATE_PARENT,
	},
};

struct clk_cpu_8996 {
	struct clk_hw *alt_pll;
	unsigned long *alt_pll_freqs;
	int n_alt_pll_freqs;
	unsigned long alt_pll_thresh;
	struct clk_hw *pll;
	struct clk_hw *pll_post_div;
	unsigned long post_div_thresh;
	struct clk_regmap clkr;
};

static unsigned long alt_pll_perfcl_freqs[] = {
	307200000,
	556800000,
};

static inline struct clk_cpu_8996 *to_clk_cpu_8996(struct clk_hw *hw)
{
	return container_of(to_clk_regmap(hw), struct clk_cpu_8996, clkr);
}

static int clk_cpu_8996_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long prate)
{
	struct clk_cpu_8996 *cpuclk = to_clk_cpu_8996(hw);
	unsigned long n_alt_pll_freqs = cpuclk->n_alt_pll_freqs;
	unsigned long alt_pll_rate, alt_pll_prev_rate;
	struct clk *alt_pll, *pll, *parent, *orig_pll = NULL;
	struct smux;
	int ret;

	if (rate < cpuclk->post_div_thresh)
		pll = clk_hw_get_clk(cpuclk->pll_post_div);
	else
		pll = clk_hw_get_clk(cpuclk->pll);

	parent = clk_hw_get_clk(clk_hw_get_parent(hw));
	alt_pll = clk_hw_get_clk(cpuclk->alt_pll);

	/* Check if the alt pll freq should be changed */
	if (cpuclk->alt_pll_thresh && (n_alt_pll_freqs == 2)) {
		alt_pll_prev_rate = clk_get_rate(alt_pll);
		alt_pll_rate = cpuclk->alt_pll_freqs[0];
		if (rate > cpuclk->alt_pll_thresh)
			alt_pll_rate = cpuclk->alt_pll_freqs[1];
		ret = clk_set_rate(alt_pll, alt_pll_rate);
		if (ret)
			return ret;
	}

	/* Switch parent to alt pll */
	if (cpuclk->alt_pll) {
		orig_pll = clk_get_parent(parent);
		ret = clk_set_parent(parent, alt_pll);
		if (ret)
			return ret;
	}

	/* Set the PLL to new rate */
	ret = clk_set_rate(pll, rate);
	if (ret)
		goto error;

	/* Switch back to primary pll */
	if (cpuclk->alt_pll) {
		ret = clk_set_parent(parent, pll);
		if (ret)
			goto error;
	}
	return 0;

error:
	if (cpuclk->alt_pll)
		clk_set_parent(parent, orig_pll);

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
	.alt_pll = &pwrcl_alt_pll.clkr.hw,
	.pll = &pwrcl_pll.clkr.hw,
	.pll_post_div = &pwrcl_smux.clkr.hw,
	.post_div_thresh = 600000000,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "pwrcl_clk",
		.parent_names = (const char *[]){ "pwrcl_pmux" },
		.num_parents = 1,
		.ops = &clk_cpu_8996_ops,
	},
};

static struct clk_cpu_8996 perfcl_clk = {
	.alt_pll = &perfcl_alt_pll.clkr.hw,
	.alt_pll_freqs = alt_pll_perfcl_freqs,
	.alt_pll_thresh = 1190400000,
	.n_alt_pll_freqs = ARRAY_SIZE(alt_pll_perfcl_freqs),
	.pll = &perfcl_pll.clkr.hw,
	.pll_post_div = &perfcl_smux.clkr.hw,
	.post_div_thresh = 600000000,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "perfcl_clk",
		.parent_names = (const char *[]){ "perfcl_pmux" },
		.num_parents = 1,
		.ops = &clk_cpu_8996_ops,
	},
};

static struct clk_cpu_8996 cbfcl_clk = {
	.pll = &cbf_pll.clkr.hw,
	.post_div_thresh = 600000000,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "cbf_clk",
		.parent_names = (const char *[]){ "cbf_pmux" },
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

#define cluster_clk_register(dev, clk, clkr) { \
	clk = devm_clk_register_regmap(dev, clkr); \
	if (IS_ERR(clk)) \
		return PTR_ERR(clk); }

#define cbf_clk_register(dev, clk, hw) { \
	clk = devm_clk_register(dev, hw); \
	if (IS_ERR(clk)) \
		return PTR_ERR(clk); }

#define cpu_clk_register_fixed(dev, clk, name, pname, flags, m, n) { \
	clk = clk_register_fixed_factor(dev, name, pname, flags, m, n); \
	if (IS_ERR(clk)) \
		return PTR_ERR(clk); }

#define cpu_set_rate(dev, clk, rate) { \
	if (clk_set_rate(clk, rate)) \
		dev_err(dev, "Failed to set " #clk " to " #rate "\n"); }

#define cpu_prepare_enable(dev, clk) { \
	if (clk_prepare_enable(clk)) \
		dev_err(dev, "Failed to enable " #clk "\n"); }

#define cpu_set_parent(dev, clk, parent) { \
	if (clk_set_parent(clk, parent)) \
		dev_err(dev, "Failed to set parent for " #clk "\n"); }

struct clk *sys_apcsaux, *pwr_clk, *perf_clk, *cbf_clk;

static int register_cpu_clocks(struct device *dev, struct regmap *regmap)
{
	/* clocks */
	struct clk *perf_alt_pll, *pwr_alt_pll, *perf_pll, *pwr_pll;
	struct clk *perf_pmux, *perf_smux, *pwr_pmux, *pwr_smux;
	struct clk *perf_pll_main, *pwr_pll_main;

	/* Initialise the PLLs */
	clk_pll_configure_variable_rate(&perfcl_pll, regmap, &hfpll_config);
	clk_pll_configure_variable_rate(&pwrcl_pll, regmap, &hfpll_config);
	clk_alpha_pll_configure(&perfcl_alt_pll, regmap, &altpll_config);
	clk_alpha_pll_configure(&pwrcl_alt_pll, regmap, &altpll_config);

	/* PLLs */
	cluster_clk_register(dev, perf_pll, &perfcl_pll.clkr);
	cluster_clk_register(dev, pwr_pll, &pwrcl_pll.clkr);
	cluster_clk_register(dev, perf_alt_pll, &perfcl_alt_pll.clkr);
	cluster_clk_register(dev, pwr_alt_pll, &pwrcl_alt_pll.clkr);

	/* MUXs */
	cluster_clk_register(dev, perf_pmux, &perfcl_pmux.clkr);
	cluster_clk_register(dev, perf_smux, &perfcl_smux.clkr);
	cluster_clk_register(dev, pwr_pmux, &pwrcl_pmux.clkr);
	cluster_clk_register(dev, pwr_smux, &pwrcl_smux.clkr);

	/* Fixed factor CLKs */
	cpu_clk_register_fixed(dev, perf_pll_main, "perfcl_pll_main",
			       "perfcl_pll", CLK_SET_RATE_PARENT, 1, 2);
	cpu_clk_register_fixed(dev, pwr_pll_main, "pwrcl_pll_main",
			       "pwrcl_pll", CLK_SET_RATE_PARENT, 1, 2);

	/* Init alt pll to boot frequency */
	cpu_set_rate(dev, perf_alt_pll, 307200000);
	cpu_set_rate(dev, pwr_alt_pll, 307200000);

	/* Enable all PLLs and alt PLLs */
	cpu_prepare_enable(dev, perf_pll);
	cpu_prepare_enable(dev, pwr_pll);
	cpu_prepare_enable(dev, perf_alt_pll);
	cpu_prepare_enable(dev, pwr_alt_pll);

	/* Init MUXes with default parents */
	cpu_set_parent(dev, perf_pmux, perf_pll);
	cpu_set_parent(dev, pwr_pmux, pwr_pll);
	cpu_set_parent(dev, perf_smux, perf_pll_main);
	cpu_set_parent(dev, pwr_smux, pwr_pll_main);

	/* Register CPU clocks */
	cluster_clk_register(dev, perf_clk, &perfcl_clk.clkr);
	cluster_clk_register(dev, pwr_clk, &pwrcl_clk.clkr);

	return 0;
}

static int register_cbf_clocks(struct device *dev, struct regmap *regmap)
{
	struct clk *cbf_pll_clk, *cbf_pmux_clk, *cbf_pll_main_clk;

	cbf_pll.clkr.regmap = regmap;
	cbf_pmux.clkr.regmap = regmap;
	cbfcl_clk.clkr.regmap = regmap;

	clk_pll_configure_variable_rate(&cbf_pll, regmap, &cbfpll_config);

	cbf_clk_register(dev, cbf_pll_clk, &cbf_pll.clkr.hw);
	cbf_clk_register(dev, cbf_pmux_clk, &cbf_pmux.clkr.hw);

	cpu_clk_register_fixed(dev, cbf_pll_main_clk, "cbf_pll_main", "cbf_pll",
			       CLK_SET_RATE_PARENT, 1, 2);

	cpu_prepare_enable(dev, cbf_pll_clk);
	cpu_set_parent(dev, cbf_pmux_clk, cbf_pll_clk);

	cbfcl_clk.alt_pll = __clk_get_hw(sys_apcsaux);
	cbfcl_clk.pll_post_div = __clk_get_hw(cbf_pll_main_clk);

	cbf_clk_register(dev, cbf_clk, &cbfcl_clk.clkr.hw);

	return 0;
}

static int qcom_cpu_clk_msm8996_driver_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *base;
	struct regmap *regmap_cpu, *regmap_cbf;
	struct clk_onecell_data *data;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->clks = devm_kcalloc(dev, 3, sizeof(struct clk *), GFP_KERNEL);
	if (!data->clks)
		return -ENOMEM;

	cpu_clk_register_fixed(dev, sys_apcsaux, "sys_apcsaux_clk",
			       "gpll0_early", 0, 1, 1);

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

	regmap_cbf = syscon_regmap_lookup_by_phandle(dev->of_node, "qcom,cbf");
	if (IS_ERR(regmap_cbf))
		return PTR_ERR(regmap_cbf);

	ret = register_cbf_clocks(dev, regmap_cbf);
	if (ret)
		return ret;

	data->clks[0] = pwr_clk;
	data->clks[1] = perf_clk;
	data->clks[2] = cbf_clk;

	return of_clk_add_provider(dev->of_node, of_clk_src_onecell_get, data);
}

static struct platform_driver qcom_cpu_clk_msm8996_driver = {
	.probe = qcom_cpu_clk_msm8996_driver_probe,
	.driver = {
		.name = "qcom-cpu-clk-msm8996",
		.of_match_table = match_table,
		.owner = THIS_MODULE,
	},
};

builtin_platform_driver(qcom_cpu_clk_msm8996_driver);

MODULE_DESCRIPTION("CPU clock driver for msm8996");
MODULE_LICENSE("GPL v2");
