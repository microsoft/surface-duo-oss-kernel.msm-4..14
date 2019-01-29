// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2012-2014,2017 The Linux Foundation. All rights reserved.
 * Copyright (c) 2018, Linaro Limited
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>

#define PHY_CTRL0			0x6C
#define PHY_CTRL1			0x70
#define PHY_CTRL2			0x74
#define PHY_CTRL4			0x7C

/* PHY_CTRL bits */
#define REF_PHY_EN			BIT(0)
#define LANE0_PWR_ON			BIT(2)
#define SWI_PCS_CLK_SEL			BIT(4)
#define TST_PWR_DOWN			BIT(4)
#define PHY_RESET			BIT(7)

enum phy_vdd_ctrl { ENABLE, DISABLE, };
enum phy_regulator { VDD, VDDA1P8, };

struct ssphy_priv {
	void __iomem *base;
	struct device *dev;
	struct reset_control *reset_com;
	struct reset_control *reset_phy;
	struct regulator *vbus;
	struct regulator_bulk_data *regs;
	int num_regs;
	struct clk_bulk_data *clks;
	int num_clks;
	enum phy_mode mode;
};

static inline void qcom_ssphy_updatel(void __iomem *addr, u32 mask, u32 val)
{
	writel((readl(addr) & ~mask) | val, addr);
}

static inline int qcom_ssphy_vbus_enable(struct regulator *vbus)
{
	return !regulator_is_enabled(vbus) ? regulator_enable(vbus) : 0;
}

static inline int qcom_ssphy_vbus_disable(struct regulator *vbus)
{
	return regulator_is_enabled(vbus) ? regulator_disable(vbus) : 0;
}

static int qcom_ssphy_vdd_ctrl(struct ssphy_priv *priv, enum phy_vdd_ctrl ctrl)
{
	const int vdd_min = ctrl == ENABLE ? 1050000 : 0;
	const int vdd_max = 1050000;
	int ret;

	ret = regulator_set_voltage(priv->regs[VDD].consumer, vdd_min, vdd_max);
	if (ret)
		dev_err(priv->dev, "Failed to set regulator vdd to %d\n",
			vdd_min);

	return ret;
}

static int qcom_ssphy_vbus_ctrl(struct regulator *vbus, enum phy_mode mode)
{
	if (!vbus)
		return 0;

	if (mode == PHY_MODE_INVALID)
		return 0;

	/* gadget attached */
	if (mode == PHY_MODE_USB_HOST)
		return qcom_ssphy_vbus_enable(vbus);

	/* USB_DEVICE: gadget removed: enable detection */
	return qcom_ssphy_vbus_disable(vbus);
}

static int qcom_ssphy_do_reset(struct ssphy_priv *priv)
{
	int ret;

	if (!priv->reset_com) {
		qcom_ssphy_updatel(priv->base + PHY_CTRL1, PHY_RESET,
				   PHY_RESET);
		usleep_range(10, 20);
		qcom_ssphy_updatel(priv->base + PHY_CTRL1, PHY_RESET, 0);
	} else {
		ret = reset_control_assert(priv->reset_com);
		if (ret) {
			dev_err(priv->dev, "Failed to assert reset com\n");
			return ret;
		}

		ret = reset_control_assert(priv->reset_phy);
		if (ret) {
			dev_err(priv->dev, "Failed to assert reset phy\n");
			return ret;
		}

		usleep_range(10, 20);

		ret = reset_control_deassert(priv->reset_com);
		if (ret) {
			dev_err(priv->dev, "Failed to deassert reset com\n");
			return ret;
		}

		ret = reset_control_deassert(priv->reset_phy);
		if (ret) {
			dev_err(priv->dev, "Failed to deassert reset phy\n");
			return ret;
		}
	}

	return 0;
}

static int qcom_ssphy_power_on(struct phy *phy)
{
	struct ssphy_priv *priv = phy_get_drvdata(phy);
	int ret;

	ret = qcom_ssphy_vdd_ctrl(priv, ENABLE);
	if (ret)
		return ret;

	ret = regulator_bulk_enable(priv->num_regs, priv->regs);
	if (ret)
		goto err1;

	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret)
		goto err2;

	ret = qcom_ssphy_vbus_ctrl(priv->vbus, priv->mode);
	if (ret)
		goto err3;

	ret = qcom_ssphy_do_reset(priv);
	if (ret)
		goto err4;

	writeb(SWI_PCS_CLK_SEL, priv->base + PHY_CTRL0);
	qcom_ssphy_updatel(priv->base + PHY_CTRL4, LANE0_PWR_ON, LANE0_PWR_ON);
	qcom_ssphy_updatel(priv->base + PHY_CTRL2, REF_PHY_EN, REF_PHY_EN);
	qcom_ssphy_updatel(priv->base + PHY_CTRL4, TST_PWR_DOWN, 0);

	return 0;
err4:
	if (priv->vbus && priv->mode != PHY_MODE_INVALID)
		qcom_ssphy_vbus_disable(priv->vbus);
err3:
	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
err2:
	regulator_bulk_disable(priv->num_regs, priv->regs);
err1:
	qcom_ssphy_vdd_ctrl(priv, DISABLE);

	return ret;
}

static int qcom_ssphy_power_off(struct phy *phy)
{
	struct ssphy_priv *priv = phy_get_drvdata(phy);

	qcom_ssphy_updatel(priv->base + PHY_CTRL4, LANE0_PWR_ON, 0);
	qcom_ssphy_updatel(priv->base + PHY_CTRL2, REF_PHY_EN, 0);
	qcom_ssphy_updatel(priv->base + PHY_CTRL4, TST_PWR_DOWN, TST_PWR_DOWN);

	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
	regulator_bulk_disable(priv->num_regs, priv->regs);

	if (priv->vbus && priv->mode != PHY_MODE_INVALID)
		qcom_ssphy_vbus_disable(priv->vbus);

	qcom_ssphy_vdd_ctrl(priv, DISABLE);

	return 0;
}

static int qcom_ssphy_init_clock(struct ssphy_priv *priv)
{
	const char * const clk_id[] = { "ref", "phy", "pipe", };
	int i;

	priv->num_clks = ARRAY_SIZE(clk_id);
	priv->clks = devm_kcalloc(priv->dev, priv->num_clks,
				  sizeof(*priv->clks), GFP_KERNEL);
	if (!priv->clks)
		return -ENOMEM;

	for (i = 0; i < priv->num_clks; i++)
		priv->clks[i].id = clk_id[i];

	return devm_clk_bulk_get(priv->dev, priv->num_clks, priv->clks);
}

static int qcom_ssphy_init_regulator(struct ssphy_priv *priv)
{
	const char * const reg_supplies[] = {
		[VDD] = "vdd",
		[VDDA1P8] = "vdda1p8",
	};
	int ret, i;

	priv->num_regs = ARRAY_SIZE(reg_supplies);
	priv->regs = devm_kcalloc(priv->dev, priv->num_regs,
				  sizeof(*priv->regs), GFP_KERNEL);
	if (!priv->regs)
		return -ENOMEM;

	for (i = 0; i < priv->num_regs; i++)
		priv->regs[i].supply = reg_supplies[i];

	ret = devm_regulator_bulk_get(priv->dev, priv->num_regs, priv->regs);
	if (ret)
		return ret;

	priv->vbus = devm_regulator_get_optional(priv->dev, "vbus");
	if (IS_ERR(priv->vbus))
		return PTR_ERR(priv->vbus);

	return 0;
}

static int qcom_ssphy_init_reset(struct ssphy_priv *priv)
{
	priv->reset_com = devm_reset_control_get_optional(priv->dev, "com");
	if (IS_ERR(priv->reset_com)) {
		dev_err(priv->dev, "Failed to get reset control com\n");
		return PTR_ERR(priv->reset_com);
	}

	if (priv->reset_com) {
		/* if reset_com is present, reset_phy is no longer optional */
		priv->reset_phy = devm_reset_control_get(priv->dev, "phy");
		if (IS_ERR(priv->reset_phy)) {
			dev_err(priv->dev, "Failed to get reset control phy\n");
			return PTR_ERR(priv->reset_phy);
		}
	}

	return 0;
}

static int qcom_ssphy_set_mode(struct phy *phy, enum phy_mode mode)
{
	struct ssphy_priv *priv = phy_get_drvdata(phy);

	if (!priv->vbus)
		return 0;

	if (mode != PHY_MODE_USB_HOST && mode != PHY_MODE_USB_DEVICE)
		return -EINVAL;

	priv->mode = mode;

	dev_dbg(priv->dev, "mode %d", mode);

	return qcom_ssphy_vbus_ctrl(priv->vbus, priv->mode);
}

static const struct phy_ops qcom_ssphy_ops = {
	.set_mode = qcom_ssphy_set_mode,
	.power_off = qcom_ssphy_power_off,
	.power_on = qcom_ssphy_power_on,
	.owner = THIS_MODULE,
};

static int qcom_ssphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct ssphy_priv *priv;
	struct resource *res;
	struct phy *phy;
	int ret;

	priv = devm_kzalloc(dev, sizeof(struct ssphy_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->mode = PHY_MODE_INVALID;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	ret = qcom_ssphy_init_clock(priv);
	if (ret)
		return ret;

	ret = qcom_ssphy_init_reset(priv);
	if (ret)
		return ret;

	ret = qcom_ssphy_init_regulator(priv);
	if (ret)
		return ret;

	phy = devm_phy_create(dev, dev->of_node, &qcom_ssphy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "Failed to create the SS phy\n");
		return PTR_ERR(phy);
	}

	phy_set_drvdata(phy, priv);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id qcom_ssphy_match[] = {
	{ .compatible = "qcom,usb-ssphy", },
	{ },
};
MODULE_DEVICE_TABLE(of, qcom_ssphy_match);

static struct platform_driver qcom_ssphy_driver = {
	.probe		= qcom_ssphy_probe,
	.driver = {
		.name	= "qcom_usb_ssphy",
		.of_match_table = qcom_ssphy_match,
	},
};
module_platform_driver(qcom_ssphy_driver);

MODULE_DESCRIPTION("Qualcomm Super-Speed USB PHY driver");
MODULE_LICENSE("GPL v2");
