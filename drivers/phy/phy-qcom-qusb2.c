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
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>

#define QUSB2PHY_PLL_TEST		0x04
#define CLK_REF_SEL			BIT(7)

#define QUSB2PHY_PLL_TUNE		0x08
#define QUSB2PHY_PLL_USER_CTL1		0x0c
#define QUSB2PHY_PLL_USER_CTL2		0x10
#define QUSB2PHY_PLL_AUTOPGM_CTL1	0x1c
#define QUSB2PHY_PLL_PWR_CTRL		0x18

#define QUSB2PHY_PLL_STATUS		0x38
#define PLL_LOCKED			BIT(5)

#define QUSB2PHY_PORT_TUNE1             0x80
#define QUSB2PHY_PORT_TUNE2             0x84
#define QUSB2PHY_PORT_TUNE3             0x88
#define QUSB2PHY_PORT_TUNE4             0x8C
#define QUSB2PHY_PORT_TUNE5		0x90
#define QUSB2PHY_PORT_TEST2		0x9c

#define QUSB2PHY_PORT_POWERDOWN		0xB4
#define CLAMP_N_EN			BIT(5)
#define FREEZIO_N			BIT(1)
#define POWER_DOWN			BIT(0)

#define QUSB2PHY_REFCLK_ENABLE		BIT(0)

#define PHY_CLK_SCHEME_SEL		BIT(0)

struct qusb2_phy_init_tbl {
	unsigned int reg_offset;
	unsigned int cfg_val;
};
#define QCOM_QUSB2_PHY_INIT_CFG(reg, val) \
	{				\
		.reg_offset = reg,	\
		.cfg_val = val,		\
	}

static struct qusb2_phy_init_tbl msm8996_phy_init_tbl[] = {
	QCOM_QUSB2_PHY_INIT_CFG(QUSB2PHY_PORT_TUNE1, 0xF8),
	QCOM_QUSB2_PHY_INIT_CFG(QUSB2PHY_PORT_TUNE2, 0xB3),
	QCOM_QUSB2_PHY_INIT_CFG(QUSB2PHY_PORT_TUNE3, 0x83),
	QCOM_QUSB2_PHY_INIT_CFG(QUSB2PHY_PORT_TUNE4, 0xC0),
	QCOM_QUSB2_PHY_INIT_CFG(QUSB2PHY_PLL_TUNE, 0x30),
	QCOM_QUSB2_PHY_INIT_CFG(QUSB2PHY_PLL_USER_CTL1, 0x79),
	QCOM_QUSB2_PHY_INIT_CFG(QUSB2PHY_PLL_USER_CTL2, 0x21),
	QCOM_QUSB2_PHY_INIT_CFG(QUSB2PHY_PORT_TEST2, 0x14),
	QCOM_QUSB2_PHY_INIT_CFG(QUSB2PHY_PLL_AUTOPGM_CTL1, 0x9F),
	QCOM_QUSB2_PHY_INIT_CFG(QUSB2PHY_PLL_PWR_CTRL, 0x00),
};

struct qusb2_phy_init_cfg {
	struct qusb2_phy_init_tbl *phy_init_tbl;
	int phy_init_tbl_sz;
	/* offset to PHY_CLK_SCHEME register in TCSR map. */
	unsigned int clk_scheme_offset;
};

const struct qusb2_phy_init_cfg msm8996_phy_init_cfg = {
	.phy_init_tbl = msm8996_phy_init_tbl,
	.phy_init_tbl_sz = ARRAY_SIZE(msm8996_phy_init_tbl),
};

/**
 * struct qusb2_phy: Structure holding qusb2 phy attributes.
 *
 * @phy: pointer to generic phy.
 * @base: pointer to iomapped memory space for qubs2 phy.
 *
 * @cfg_ahb_clk: pointer to AHB2PHY interface clock.
 * @ref_clk: pointer to reference clock.
 * @ref_clk_src: pointer to source to reference clock.
 * @iface_src: pointer to phy interface clock.
 *
 * @phy_reset: Pointer to phy reset control
 *
 * @vdda_phy: vdd supply to the phy core block.
 * @vdda_pll: 1.8V vdd supply to ref_clk block.
 * @vdda_phy_dpdm: 3.1V vdd supply to Dp/Dm port signals.
 * @tcsr: pointer to TCSR syscon register map.
 *
 * @cfg: phy initialization config data
 * @has_se_clk_scheme: indicate if PHY has Single-ended ref clock scheme
 */
struct qusb2_phy {
	struct phy *phy;
	void __iomem *base;

	struct clk *cfg_ahb_clk;
	struct clk *ref_clk;
	struct clk *ref_clk_src;
	struct clk *iface_clk;

	struct reset_control *phy_reset;

	struct regulator *vdd_phy;
	struct regulator *vdda_pll;
	struct regulator *vdda_phy_dpdm;

	struct regmap *tcsr;

	const struct qusb2_phy_init_cfg *cfg;
	bool has_se_clk_scheme;
};

static inline void qusb2_setbits(void __iomem *reg, u32 val)
{
	u32 reg_val;

	reg_val = readl_relaxed(reg);
	reg_val |= val;
	writel_relaxed(reg_val, reg);

	/* Ensure above write is completed */
	mb();
}

static inline void qusb2_clrbits(void __iomem *reg, u32 val)
{
	u32 reg_val;

	reg_val = readl_relaxed(reg);
	reg_val &= ~val;
	writel_relaxed(reg_val, reg);

	/* Ensure above write is completed */
	mb();
}

static void qcom_qusb2_phy_configure(void __iomem *base,
				struct qusb2_phy_init_tbl init_tbl[],
				int init_tbl_sz)
{
	int i;

	for (i = 0; i < init_tbl_sz; i++) {
		writel_relaxed(init_tbl[i].cfg_val,
				base + init_tbl[i].reg_offset);
	}

	/* flush buffered writes */
	mb();
}

static void qusb2_phy_enable_clocks(struct qusb2_phy *qphy, bool on)
{
	if (on) {
		clk_prepare_enable(qphy->iface_clk);
		clk_prepare_enable(qphy->ref_clk_src);
	} else {
		clk_disable_unprepare(qphy->ref_clk_src);
		clk_disable_unprepare(qphy->iface_clk);
	}

	dev_vdbg(&qphy->phy->dev, "%s(): clocks enabled\n", __func__);
}

static int qusb2_phy_enable_power(struct qusb2_phy *qphy, bool on)
{
	int ret;
	struct device *dev = &qphy->phy->dev;

	if (!on)
		goto disable_vdda_phy_dpdm;

	ret = regulator_enable(qphy->vdd_phy);
	if (ret) {
		dev_err(dev, "Unable to enable vdd-phy:%d\n", ret);
		goto err_vdd_phy;
	}

	ret = regulator_enable(qphy->vdda_pll);
	if (ret) {
		dev_err(dev, "Unable to enable vdda-pll:%d\n", ret);
		goto disable_vdd_phy;
	}

	ret = regulator_enable(qphy->vdda_phy_dpdm);
	if (ret) {
		dev_err(dev, "Unable to enable vdda-phy-dpdm:%d\n", ret);
		goto disable_vdda_pll;
	}

	dev_vdbg(dev, "%s() regulators are turned on.\n", __func__);

	return ret;

disable_vdda_phy_dpdm:
	regulator_disable(qphy->vdda_phy_dpdm);
disable_vdda_pll:
	regulator_disable(qphy->vdda_pll);
disable_vdd_phy:
	regulator_disable(qphy->vdd_phy);
err_vdd_phy:
	dev_vdbg(dev, "%s() regulators are turned off.\n", __func__);
	return ret;
}

/*
 * Fetches HS Tx tuning value from e-fuse and sets QUSB2PHY_PORT_TUNE2
 * register.
 * For any error case, skip setting the value and use the default value.
 */
static int qusb2_phy_set_tune2_param(struct qusb2_phy *qphy)
{
	struct device *dev = &qphy->phy->dev;
	struct nvmem_cell *cell;
	ssize_t len;
	u8 *val;

	/*
	 * Read EFUSE register having TUNE2 parameter's high nibble.
	 * If efuse register shows value as 0x0, or if we fail to find
	 * a valid efuse register settings, then use default value
	 * as 0xB for high nibble that we have already set while
	 * configuring phy.
	 */
	cell = devm_nvmem_cell_get(dev, "tune2_hstx_trim_efuse");
	if (IS_ERR(cell)) {
		if (PTR_ERR(cell) == -EPROBE_DEFER)
			return PTR_ERR(cell);
		goto skip;
	}

	/*
	 * we need to read only one byte here, since the required
	 * parameter value fits in one nibble
	 */
	val = (u8 *)nvmem_cell_read(cell, &len);
	if (!IS_ERR(val)) {
		/* Fused TUNE2 value is the higher nibble only */
		qusb2_setbits(qphy->base + QUSB2PHY_PORT_TUNE2,
							val[0] << 0x4);
	} else {
		dev_dbg(dev, "failed reading hs-tx trim value: %ld\n",
							PTR_ERR(val));
	}

skip:
	return 0;
}

static int qusb2_phy_poweron(struct phy *phy)
{
	struct qusb2_phy *qphy = phy_get_drvdata(phy);
	int ret;

	dev_vdbg(&phy->dev, "Powering-on QUSB2 phy\n");

	ret = qusb2_phy_enable_power(qphy, true);
	if (ret)
		return ret;

	qusb2_phy_enable_clocks(qphy, true);

	return ret;
}

static int qusb2_phy_poweroff(struct phy *phy)
{
	struct qusb2_phy *qphy = phy_get_drvdata(phy);

	qusb2_phy_enable_clocks(qphy, false);
	qusb2_phy_enable_power(qphy, false);

	return 0;
}

static int qusb2_phy_init(struct phy *phy)
{
	struct qusb2_phy *qphy = phy_get_drvdata(phy);
	unsigned int reset_val;
	unsigned int clk_scheme;
	int ret;

	dev_vdbg(&phy->dev, "Initializing QUSB2 phy\n");

	/* enable ahb interface clock to program phy */
	clk_prepare_enable(qphy->cfg_ahb_clk);

	/* Perform phy reset */
	ret = reset_control_assert(qphy->phy_reset);
	if (ret) {
		dev_err(&phy->dev, "Failed to assert phy_reset\n");
		return ret;
	}
	/* 100 us delay to keep PHY in reset mode */
	usleep_range(100, 150);
	ret = reset_control_deassert(qphy->phy_reset);
	if (ret) {
		dev_err(&phy->dev, "Failed to de-assert phy_reset\n");
		return ret;
	}

	/* Disable the PHY */
	qusb2_setbits(qphy->base + QUSB2PHY_PORT_POWERDOWN,
			CLAMP_N_EN | FREEZIO_N | POWER_DOWN);

	/* save reset value to override based on clk scheme */
	reset_val = readl_relaxed(qphy->base + QUSB2PHY_PLL_TEST);

	qcom_qusb2_phy_configure(qphy->base, qphy->cfg->phy_init_tbl,
				qphy->cfg->phy_init_tbl_sz);

	/* Check for efuse value for tuning the PHY */
	ret = qusb2_phy_set_tune2_param(qphy);
	if (ret)
		return ret;

	/* Enable the PHY */
	qusb2_clrbits(qphy->base + QUSB2PHY_PORT_POWERDOWN, POWER_DOWN);

	/* Require to get phy pll lock successfully */
	usleep_range(150, 160);

	/* Default is Single-ended clock on msm8996 */
	qphy->has_se_clk_scheme = true;
	/*
	 * read TCSR_PHY_CLK_SCHEME register to check if Single-ended
	 * clock scheme is selected. If yes, then disable differential
	 * ref_clk and use single-ended clock, otherwise use differential
	 * ref_clk only.
	 */
	if (qphy->tcsr) {
		ret = regmap_read(qphy->tcsr, qphy->cfg->clk_scheme_offset,
							&clk_scheme);
		/* is it a differential clock scheme ? */
		if (!(clk_scheme & PHY_CLK_SCHEME_SEL)) {
			dev_vdbg(&phy->dev, "%s: select differential clk src\n",
								__func__);
			qphy->has_se_clk_scheme = false;
		} else {
			dev_vdbg(&phy->dev, "%s: select single-ended clk src\n",
								__func__);
		}
	}

	if (!qphy->has_se_clk_scheme) {
		reset_val &= ~CLK_REF_SEL;
		clk_prepare_enable(qphy->ref_clk);
	} else {
		reset_val |= CLK_REF_SEL;
	}

	writel_relaxed(reset_val, qphy->base + QUSB2PHY_PLL_TEST);

	/* Make sure that above write is completed to get PLL source clock */
	wmb();

	/* Required to get PHY PLL lock successfully */
	usleep_range(100, 110);

	if (!(readb_relaxed(qphy->base + QUSB2PHY_PLL_STATUS) &
					PLL_LOCKED)) {
		dev_err(&phy->dev, "QUSB PHY PLL LOCK fails:%x\n",
			readb_relaxed(qphy->base + QUSB2PHY_PLL_STATUS));
		return -EBUSY;
	}

	return 0;
}

static int qusb2_phy_exit(struct phy *phy)
{
	struct qusb2_phy *qphy = phy_get_drvdata(phy);

	/* Disable the PHY */
	qusb2_setbits(qphy->base + QUSB2PHY_PORT_POWERDOWN,
			CLAMP_N_EN | FREEZIO_N | POWER_DOWN);

	if (!qphy->has_se_clk_scheme)
		clk_disable_unprepare(qphy->ref_clk);

	clk_disable_unprepare(qphy->cfg_ahb_clk);

	return 0;
}

static const struct phy_ops qusb2_phy_gen_ops = {
	.init		= qusb2_phy_init,
	.exit		= qusb2_phy_exit,
	.power_on	= qusb2_phy_poweron,
	.power_off	= qusb2_phy_poweroff,
	.owner		= THIS_MODULE,
};

static const struct of_device_id qusb2_phy_of_match_table[] = {
	{
		.compatible	= "qcom,msm8996-qusb2-phy",
		.data		= &msm8996_phy_init_cfg,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, qusb2_phy_of_match_table);

static int qusb2_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qusb2_phy *qphy;
	struct phy_provider *phy_provider;
	struct phy *generic_phy;
	const struct of_device_id *match;
	struct resource *res;
	int ret;

	qphy = devm_kzalloc(dev, sizeof(*qphy), GFP_KERNEL);
	if (!qphy)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	qphy->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(qphy->base))
		return PTR_ERR(qphy->base);

	qphy->cfg_ahb_clk = devm_clk_get(dev, "cfg_ahb_clk");
	if (IS_ERR(qphy->cfg_ahb_clk)) {
		ret = PTR_ERR(qphy->cfg_ahb_clk);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get cfg_ahb_clk\n");
		return ret;
	}

	qphy->ref_clk_src = devm_clk_get(dev, "ref_clk_src");
	if (IS_ERR(qphy->ref_clk_src)) {
		ret = PTR_ERR(qphy->ref_clk_src);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "clk get failed for ref_clk_src\n");
		return ret;
	}

	qphy->ref_clk = devm_clk_get(dev, "ref_clk");
	if (IS_ERR(qphy->ref_clk)) {
		ret = PTR_ERR(qphy->ref_clk);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "clk get failed for ref_clk\n");
		return ret;
	} else {
		clk_set_rate(qphy->ref_clk, 19200000);
	}

	qphy->iface_clk = devm_clk_get(dev, "iface_clk");
	if (IS_ERR(qphy->iface_clk)) {
		ret = PTR_ERR(qphy->iface_clk);
		if (ret != -EPROBE_DEFER) {
			qphy->iface_clk = NULL;
			dev_dbg(dev, "clk get failed for iface_clk\n");
		} else {
			return ret;
		}
	}

	qphy->phy_reset = devm_reset_control_get(&pdev->dev, "phy");
	if (IS_ERR(qphy->phy_reset)) {
		dev_err(dev, "failed to get phy core reset\n");
		return PTR_ERR(qphy->phy_reset);
	}

	qphy->vdd_phy = devm_regulator_get(dev, "vdd-phy");
	if (IS_ERR(qphy->vdd_phy)) {
		dev_err(dev, "unable to get vdd-phy supply\n");
		return PTR_ERR(qphy->vdd_phy);
	}

	qphy->vdda_pll = devm_regulator_get(dev, "vdda-pll");
	if (IS_ERR(qphy->vdda_pll)) {
		dev_err(dev, "unable to get vdda-pll supply\n");
		return PTR_ERR(qphy->vdda_pll);
	}

	qphy->vdda_phy_dpdm = devm_regulator_get(dev, "vdda-phy-dpdm");
	if (IS_ERR(qphy->vdda_phy_dpdm)) {
		dev_err(dev, "unable to get vdda-phy-dpdm supply\n");
		return PTR_ERR(qphy->vdda_phy_dpdm);
	}

	/* Get the specific init parameters of QMP phy */
	match = of_match_node(qusb2_phy_of_match_table, dev->of_node);
	qphy->cfg = match->data;

	qphy->tcsr = syscon_regmap_lookup_by_phandle(dev->of_node,
							"qcom,tcsr-syscon");
	if (IS_ERR(qphy->tcsr)) {
		dev_dbg(dev, "Failed to lookup TCSR regmap\n");
		qphy->tcsr = NULL;
	}

	generic_phy = devm_phy_create(dev, NULL, &qusb2_phy_gen_ops);
	if (IS_ERR(generic_phy)) {
		ret = PTR_ERR(generic_phy);
		dev_err(dev, "%s: failed to create phy %d\n", __func__, ret);
		return ret;
	}
	qphy->phy = generic_phy;

	dev_set_drvdata(dev, qphy);
	phy_set_drvdata(generic_phy, qphy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		ret = PTR_ERR(phy_provider);
		dev_err(dev, "%s: failed to register phy %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static struct platform_driver qusb2_phy_driver = {
	.probe		= qusb2_phy_probe,
	.driver = {
		.name	= "qcom-qusb2-phy",
		.of_match_table = of_match_ptr(qusb2_phy_of_match_table),
	},
};

module_platform_driver(qusb2_phy_driver);

MODULE_AUTHOR("Vivek Gautam <vivek.gautam@codeaurora.org>");
MODULE_DESCRIPTION("Qualcomm QUSB2 PHY driver");
MODULE_LICENSE("GPL v2");
