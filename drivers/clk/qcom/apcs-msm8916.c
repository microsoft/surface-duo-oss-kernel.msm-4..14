/*
 * Copyright (c) 2017, Linaro Limited
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
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include "clk-regmap.h"
#include "clk-regmap-mux-div.h"

enum {
	P_GPLL0,
	P_A53PLL,
};

static const struct parent_map gpll0_a53cc_map[] = {
	{ P_GPLL0, 4 },
	{ P_A53PLL, 5 },
};

static const char * const gpll0_a53cc[] = {
	"gpll0_vote",
	"a53pll",
};

/*
 * We use the notifier function for switching to a temporary safe configuration
 * (mux and divider), while the A53 PLL is reconfigured.
 */
static int a53cc_notifier_cb(struct notifier_block *nb, unsigned long event,
			     void *data)
{
	int ret = 0;
	struct clk_regmap_mux_div *md = container_of(nb,
						     struct clk_regmap_mux_div,
						     clk_nb);
	if (event == PRE_RATE_CHANGE)
		/* set the mux and divider to safe frequency (400mhz) */
		ret = __mux_div_set_src_div(md, 4, 3);

	return notifier_from_errno(ret);
}

static int qcom_apcs_msm8916_clk_probe(struct platform_device *pdev)
{
	struct device *dev = pdev->dev.parent;
	struct device_node *np = dev->of_node;
	struct clk_regmap_mux_div *a53cc;
	struct regmap *regmap;
	struct clk_init_data init = { };
	int ret;

	regmap = dev_get_regmap(dev, NULL);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "failed to get regmap: %d\n", ret);
		return ret;
	}

	a53cc = devm_kzalloc(dev, sizeof(*a53cc), GFP_KERNEL);
	if (!a53cc)
		return -ENOMEM;

	init.name = "a53mux";
	init.parent_names = gpll0_a53cc;
	init.num_parents = ARRAY_SIZE(gpll0_a53cc);
	init.ops = &clk_regmap_mux_div_ops;
	init.flags = CLK_SET_RATE_PARENT;

	a53cc->clkr.hw.init = &init;
	a53cc->clkr.regmap = regmap;
	a53cc->reg_offset = 0x50;
	a53cc->hid_width = 5;
	a53cc->hid_shift = 0;
	a53cc->src_width = 3;
	a53cc->src_shift = 8;
	a53cc->parent_map = gpll0_a53cc_map;

	a53cc->pclk = devm_clk_get(dev, NULL);
	if (IS_ERR(a53cc->pclk)) {
		ret = PTR_ERR(a53cc->pclk);
		dev_err(dev, "failed to get clk: %d\n", ret);
		return ret;
	}

	a53cc->clk_nb.notifier_call = a53cc_notifier_cb;
	ret = clk_notifier_register(a53cc->pclk, &a53cc->clk_nb);
	if (ret) {
		dev_err(dev, "failed to register clock notifier: %d\n", ret);
		return ret;
	}

	ret = devm_clk_register_regmap(dev, &a53cc->clkr);
	if (ret) {
		dev_err(dev, "failed to register regmap clock: %d\n", ret);
		goto err;
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_simple_get,
				     &a53cc->clkr.hw);
	if (ret) {
		dev_err(dev, "failed to add clock provider: %d\n", ret);
		goto err;
	}

	platform_set_drvdata(pdev, a53cc);

	return 0;

err:
	clk_notifier_unregister(a53cc->pclk, &a53cc->clk_nb);
	return ret;
}

static int qcom_apcs_msm8916_clk_remove(struct platform_device *pdev)
{
	struct clk_regmap_mux_div *a53cc = platform_get_drvdata(pdev);

	clk_notifier_unregister(a53cc->pclk, &a53cc->clk_nb);
	of_clk_del_provider(pdev->dev.of_node);

	return 0;
}

static const struct of_device_id qcom_apcs_msm8916_clk_of_match[] = {
	{ .compatible = "qcom,msm8916-apcs-clk" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_apcs_msm8916_clk_of_match);

static struct platform_driver qcom_apcs_msm8916_clk_driver = {
	.probe = qcom_apcs_msm8916_clk_probe,
	.remove = qcom_apcs_msm8916_clk_remove,
	.driver = {
		.name = "qcom-apcs-msm8916-clk",
		.of_match_table = qcom_apcs_msm8916_clk_of_match,
	},
};
module_platform_driver(qcom_apcs_msm8916_clk_driver);

MODULE_AUTHOR("Georgi Djakov <georgi.djakov@linaro.org>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm MSM8916 APCS clock driver");
