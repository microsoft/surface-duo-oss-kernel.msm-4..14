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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "clk-rpm.h"
#include <dt-bindings/clock/qcom,rpmcc-msm8916.h>

#define CXO_ID			0x0
#define QDSS_ID			0x1
#define BUS_SCALING		0x2

#define PCNOC_ID		0x0
#define SNOC_ID			0x1
#define BIMC_ID			0x0

#define BB_CLK1_ID		1
#define BB_CLK2_ID		2
#define RF_CLK1_ID		4
#define RF_CLK2_ID		5

struct rpm_cc {
	struct clk_onecell_data data;
	struct clk *clks[];
};

/* SMD clocks */
DEFINE_CLK_RPM_SMD(pcnoc_clk, pcnoc_a_clk, QCOM_SMD_RPM_BUS_CLK, PCNOC_ID, NULL);
DEFINE_CLK_RPM_SMD(snoc_clk, snoc_a_clk, QCOM_SMD_RPM_BUS_CLK, SNOC_ID, NULL);
DEFINE_CLK_RPM_SMD(bimc_clk, bimc_a_clk, QCOM_SMD_RPM_MEM_CLK, BIMC_ID, NULL);

DEFINE_CLK_RPM_SMD_BRANCH(xo, xo_a, QCOM_SMD_RPM_MISC_CLK, CXO_ID, 19200000);
DEFINE_CLK_RPM_SMD_QDSS(qdss_clk, qdss_a_clk, QCOM_SMD_RPM_MISC_CLK, QDSS_ID);

/* SMD_XO_BUFFER */
DEFINE_CLK_RPM_SMD_XO_BUFFER(bb_clk1, bb_clk1_a, BB_CLK1_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER(bb_clk2, bb_clk2_a, BB_CLK2_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER(rf_clk1, rf_clk1_a, RF_CLK1_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER(rf_clk2, rf_clk2_a, RF_CLK2_ID);

DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(bb_clk1_pin, bb_clk1_a_pin, BB_CLK1_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(bb_clk2_pin, bb_clk2_a_pin, BB_CLK2_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(rf_clk1_pin, rf_clk1_a_pin, RF_CLK1_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(rf_clk2_pin, rf_clk2_a_pin, RF_CLK2_ID);

static struct clk_rpm *rpmcc_msm8916_clks[] = {
	[RPM_XO_CLK_SRC] = &xo,
	[RPM_XO_A_CLK_SRC] = &xo_a,
	[RPM_PCNOC_CLK] = &pcnoc_clk,
	[RPM_PCNOC_A_CLK] = &pcnoc_a_clk,
	[RPM_SNOC_CLK] = &snoc_clk,
	[RPM_SNOC_A_CLK] = &snoc_a_clk,
	[RPM_BIMC_CLK] = &bimc_clk,
	[RPM_BIMC_A_CLK] = &bimc_a_clk,
	[RPM_QDSS_CLK] = &qdss_clk,
	[RPM_QDSS_A_CLK] = &qdss_a_clk,
	[RPM_BB_CLK1] = &bb_clk1,
	[RPM_BB_CLK1_A] = &bb_clk1_a,
	[RPM_BB_CLK2] = &bb_clk2,
	[RPM_BB_CLK2_A] = &bb_clk2_a,
	[RPM_RF_CLK1] = &rf_clk1,
	[RPM_RF_CLK1_A] = &rf_clk1_a,
	[RPM_RF_CLK2] = &rf_clk2,
	[RPM_RF_CLK2_A] = &rf_clk2_a,
	[RPM_BB_CLK1_PIN] = &bb_clk1_pin,
	[RPM_BB_CLK1_A_PIN] = &bb_clk1_a_pin,
	[RPM_BB_CLK2_PIN] = &bb_clk2_pin,
	[RPM_BB_CLK2_A_PIN] = &bb_clk2_a_pin,
	[RPM_RF_CLK1_PIN] = &rf_clk1_pin,
	[RPM_RF_CLK1_A_PIN] = &rf_clk1_a_pin,
	[RPM_RF_CLK2_PIN] = &rf_clk2_pin,
	[RPM_RF_CLK2_A_PIN] = &rf_clk2_a_pin,
};

static int rpmcc_msm8916_probe(struct platform_device *pdev)
{
	struct clk **clks;
	struct clk *clk;
	struct rpm_cc *rcc;
	struct qcom_smd_rpm *rpm;
	struct clk_onecell_data *data;
	int num_clks = ARRAY_SIZE(rpmcc_msm8916_clks);
	int ret, i;

	if (!pdev->dev.of_node)
		return -ENODEV;

	rpm = dev_get_drvdata(pdev->dev.parent);
	if (!rpm) {
		dev_err(&pdev->dev, "Unable to retrieve handle to RPM\n");
		return -ENODEV;
	}

	ret = clk_rpm_enable_scaling(rpm);
	if (ret)
		return ret;

	rcc = devm_kzalloc(&pdev->dev, sizeof(*rcc) + sizeof(*clks) * num_clks,
			   GFP_KERNEL);
	if (!rcc)
		return -ENOMEM;

	clks = rcc->clks;
	data = &rcc->data;
	data->clks = clks;
	data->clk_num = num_clks;

	clk = clk_register_fixed_rate(&pdev->dev, "sleep_clk_src", NULL,
				      CLK_IS_ROOT, 32768);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	for (i = 0; i < num_clks; i++) {
		if (!rpmcc_msm8916_clks[i]) {
			clks[i] = ERR_PTR(-ENOENT);
			continue;
		}

		rpmcc_msm8916_clks[i]->rpm = rpm;
		clk = devm_clk_register(&pdev->dev, &rpmcc_msm8916_clks[i]->hw);
		if (IS_ERR(clk))
			return PTR_ERR(clk);

		clks[i] = clk;
	}

	ret = of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get,
				  data);
	if (ret)
		return ret;

	clk_set_rate(bimc_a_clk.hw.clk, 800000000);
	clk_prepare_enable(bimc_a_clk.hw.clk);

	return 0;
}

static int rpmcc_msm8916_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);
	return 0;
}

static const struct of_device_id rpmcc_msm8916_of_match[] = {
	{ .compatible = "qcom,rpmcc-msm8916" },
	{ },
};

static struct platform_driver rpmcc_msm8916_driver = {
	.driver = {
		.name = "qcom-rpmcc-msm8916",
		.of_match_table = rpmcc_msm8916_of_match,
	},
	.probe = rpmcc_msm8916_probe,
	.remove = rpmcc_msm8916_remove,
};

module_platform_driver(rpmcc_msm8916_driver);
core_initcall(rpmcc_msm8916_driver_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm MSM8916 RPM Clock Controller Driver");
