/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>

/* PHY registers */
#define UNIPHY_PLL_REFCLK_CFG		0x000
#define UNIPHY_PLL_POSTDIV1_CFG		0x004
#define UNIPHY_PLL_CHGPUMP_CFG		0x008
#define UNIPHY_PLL_VCOLPF_CFG		0x00C
#define UNIPHY_PLL_VREG_CFG		0x010
#define UNIPHY_PLL_PWRGEN_CFG		0x014
#define UNIPHY_PLL_DMUX_CFG		0x018
#define UNIPHY_PLL_AMUX_CFG		0x01C
#define UNIPHY_PLL_GLB_CFG		0x020
#define UNIPHY_PLL_POSTDIV2_CFG		0x024
#define UNIPHY_PLL_POSTDIV3_CFG		0x028
#define UNIPHY_PLL_LPFR_CFG		0x02C
#define UNIPHY_PLL_LPFC1_CFG		0x030
#define UNIPHY_PLL_LPFC2_CFG		0x034
#define UNIPHY_PLL_SDM_CFG0		0x038
#define UNIPHY_PLL_SDM_CFG1		0x03C
#define UNIPHY_PLL_SDM_CFG2		0x040
#define UNIPHY_PLL_SDM_CFG3		0x044
#define UNIPHY_PLL_SDM_CFG4		0x048
#define UNIPHY_PLL_SSC_CFG0		0x04C
#define UNIPHY_PLL_SSC_CFG1		0x050
#define UNIPHY_PLL_SSC_CFG2		0x054
#define UNIPHY_PLL_SSC_CFG3		0x058
#define UNIPHY_PLL_LKDET_CFG0		0x05C
#define UNIPHY_PLL_LKDET_CFG1		0x060
#define UNIPHY_PLL_LKDET_CFG2		0x064
#define UNIPHY_PLL_TEST_CFG		0x068
#define UNIPHY_PLL_CAL_CFG0		0x06C
#define UNIPHY_PLL_CAL_CFG1		0x070
#define UNIPHY_PLL_CAL_CFG2		0x074
#define UNIPHY_PLL_CAL_CFG3		0x078
#define UNIPHY_PLL_CAL_CFG4		0x07C
#define UNIPHY_PLL_CAL_CFG5		0x080
#define UNIPHY_PLL_CAL_CFG6		0x084
#define UNIPHY_PLL_CAL_CFG7		0x088
#define UNIPHY_PLL_CAL_CFG8		0x08C
#define UNIPHY_PLL_CAL_CFG9		0x090
#define UNIPHY_PLL_CAL_CFG10		0x094
#define UNIPHY_PLL_CAL_CFG11		0x098
#define UNIPHY_PLL_EFUSE_CFG		0x09C
#define UNIPHY_PLL_DEBUG_BUS_SEL	0x0A0
#define UNIPHY_PLL_CTRL_42		0x0A4
#define UNIPHY_PLL_CTRL_43		0x0A8
#define UNIPHY_PLL_CTRL_44		0x0AC
#define UNIPHY_PLL_CTRL_45		0x0B0
#define UNIPHY_PLL_CTRL_46		0x0B4
#define UNIPHY_PLL_CTRL_47		0x0B8
#define UNIPHY_PLL_CTRL_48		0x0BC
#define UNIPHY_PLL_STATUS		0x0C0
#define UNIPHY_PLL_DEBUG_BUS0		0x0C4
#define UNIPHY_PLL_DEBUG_BUS1		0x0C8
#define UNIPHY_PLL_DEBUG_BUS2		0x0CC
#define UNIPHY_PLL_DEBUG_BUS3		0x0D0
#define UNIPHY_PLL_CTRL_54		0x0D4

#define SATA_PHY_SER_CTRL		0x100
#define SATA_PHY_TX_DRIV_CTRL0		0x104
#define SATA_PHY_TX_DRIV_CTRL1		0x108
#define SATA_PHY_TX_DRIV_CTRL2		0x10C
#define SATA_PHY_TX_DRIV_CTRL3		0x110
#define SATA_PHY_TX_IMCAL0		0x11C
#define SATA_PHY_TX_IMCAL1		0x120
#define SATA_PHY_TX_IMCAL2		0x124
#define SATA_PHY_RX_IMCAL0		0x128
#define SATA_PHY_RX_IMCAL1		0x12C
#define SATA_PHY_RX_IMCAL2		0x130
#define SATA_PHY_RX_TERM		0x134
#define SATA_PHY_EQUAL			0x13C
#define SATA_PHY_OOB_TERM		0x144
#define SATA_PHY_CDR_CTRL0		0x148
#define SATA_PHY_CDR_CTRL1		0x14C
#define SATA_PHY_CDR_CTRL2		0x150
#define SATA_PHY_CDR_CTRL3		0x154
#define SATA_PHY_CDR_CTRL4		0x158
#define SATA_PHY_FA_LOAD0		0x15C
#define SATA_PHY_FA_LOAD1		0x160
#define SATA_PHY_PI_CTRL0		0x168
#define SATA_PHY_PI_CTRL1		0x16C
#define SATA_PHY_AD_TPA_CTRL		0x178
#define SATA_PHY_REFCLK_CTRL		0x17C
#define SATA_PHY_POW_DWN_CTRL0		0x180
#define SATA_PHY_POW_DWN_CTRL1		0x184
#define SATA_PHY_TX_DATA_CTRL		0x188
#define SATA_PHY_BIST_GEN0		0x18C
#define SATA_PHY_BIST_GEN1		0x190
#define SATA_PHY_BIST_GEN2		0x194
#define SATA_PHY_BIST_GEN3		0x198
#define SATA_PHY_LBK_CTRL		0x19C
#define SATA_PHY_TEST_DEBUG_CTRL	0x1A0
#define SATA_PHY_ALIGNP			0x1A4
#define SATA_PHY_PRBS_CFG0		0x1A8
#define SATA_PHY_PRBS_CFG1		0x1AC
#define SATA_PHY_PRBS_CFG2		0x1B0
#define SATA_PHY_PRBS_CFG3		0x1B4
#define SATA_PHY_CHAN_COMP_CHK_CNT	0x1B8
#define SATA_PHY_RESET_CTRL		0x1BC
#define SATA_PHY_RX_CLR			0x1C0
#define SATA_PHY_RX_EBUF_CTRL		0x1C4
#define SATA_PHY_ID0			0x1C8
#define SATA_PHY_ID1			0x1CC
#define SATA_PHY_ID2			0x1D0
#define SATA_PHY_ID3			0x1D4
#define SATA_PHY_RX_CHK_ERR_CNT0	0x1D8
#define SATA_PHY_RX_CHK_ERR_CNT1	0x1DC
#define SATA_PHY_RX_CHK_STAT		0x1E0
#define SATA_PHY_TX_IMCAL_STAT		0x1E4
#define SATA_PHY_RX_IMCAL_STAT		0x1E8
#define SATA_PHY_RX_EBUF_STAT		0x1EC
#define SATA_PHY_DEBUG_BUS_STAT0	0x1F0
#define SATA_PHY_DEBUG_BUS_STAT1	0x1F4
#define SATA_PHY_DEBUG_BUS_STAT2	0x1F8
#define SATA_PHY_DEBUG_BUS_STAT3	0x1FC

#define UNIPHY_PLL_LOCK		BIT(0)
#define SATA_PHY_TX_CAL		BIT(0)
#define SATA_PHY_RX_CAL		BIT(0)

/* default timeout set to 1 sec */
#define TIMEOUT_MS		10000

struct qcom_apq8064_sata_phy {
	void __iomem *mmio;
	struct clk *cfg_clk;
	struct device *dev;
};

/* Helper function to do poll and timeout */
static int read_poll_timeout(void __iomem *addr, u32 mask)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(TIMEOUT_MS);
	u32 val;

	do {
		cpu_relax();
		val = readl_relaxed(addr);
		if (val & mask)
			break;
	} while (!time_after(jiffies, timeout));

	return (val & mask) ? 0 : -ETIMEDOUT;
}

static int qcom_apq8064_sata_phy_init(struct phy *generic_phy)
{
	struct qcom_apq8064_sata_phy *phy = phy_get_drvdata(generic_phy);
	void __iomem *base = phy->mmio;
	int ret = 0;

	/* SATA phy initialization */
	writel_relaxed(0x01, base + SATA_PHY_SER_CTRL);
	writel_relaxed(0xB1, base + SATA_PHY_POW_DWN_CTRL0);
	/* Make sure the power down happens before power up */
	mb();
	usleep_range(10, 60);

	writel_relaxed(0x01, base + SATA_PHY_POW_DWN_CTRL0);
	writel_relaxed(0x3E, base + SATA_PHY_POW_DWN_CTRL1);
	writel_relaxed(0x01, base + SATA_PHY_RX_IMCAL0);
	writel_relaxed(0x01, base + SATA_PHY_TX_IMCAL0);
	writel_relaxed(0x02, base + SATA_PHY_TX_IMCAL2);

	/* Write UNIPHYPLL registers to configure PLL */
	writel_relaxed(0x04, base + UNIPHY_PLL_REFCLK_CFG);
	writel_relaxed(0x00, base + UNIPHY_PLL_PWRGEN_CFG);

	writel_relaxed(0x0A, base + UNIPHY_PLL_CAL_CFG0);
	writel_relaxed(0xF3, base + UNIPHY_PLL_CAL_CFG8);
	writel_relaxed(0x01, base + UNIPHY_PLL_CAL_CFG9);
	writel_relaxed(0xED, base + UNIPHY_PLL_CAL_CFG10);
	writel_relaxed(0x02, base + UNIPHY_PLL_CAL_CFG11);

	writel_relaxed(0x36, base + UNIPHY_PLL_SDM_CFG0);
	writel_relaxed(0x0D, base + UNIPHY_PLL_SDM_CFG1);
	writel_relaxed(0xA3, base + UNIPHY_PLL_SDM_CFG2);
	writel_relaxed(0xF0, base + UNIPHY_PLL_SDM_CFG3);
	writel_relaxed(0x00, base + UNIPHY_PLL_SDM_CFG4);

	writel_relaxed(0x19, base + UNIPHY_PLL_SSC_CFG0);
	writel_relaxed(0xE1, base + UNIPHY_PLL_SSC_CFG1);
	writel_relaxed(0x00, base + UNIPHY_PLL_SSC_CFG2);
	writel_relaxed(0x11, base + UNIPHY_PLL_SSC_CFG3);

	writel_relaxed(0x04, base + UNIPHY_PLL_LKDET_CFG0);
	writel_relaxed(0xFF, base + UNIPHY_PLL_LKDET_CFG1);

	writel_relaxed(0x02, base + UNIPHY_PLL_GLB_CFG);
	/* make sure global config LDO power down happens before power up */
	mb();

	writel_relaxed(0x03, base + UNIPHY_PLL_GLB_CFG);
	writel_relaxed(0x05, base + UNIPHY_PLL_LKDET_CFG2);

	/* PLL Lock wait */
	ret = read_poll_timeout(base + UNIPHY_PLL_STATUS, UNIPHY_PLL_LOCK);
	if (ret) {
		dev_err(phy->dev, "poll timeout UNIPHY_PLL_STATUS\n");
		return ret;
	}

	/* TX Calibration */
	ret = read_poll_timeout(base + SATA_PHY_TX_IMCAL_STAT, SATA_PHY_TX_CAL);
	if (ret) {
		dev_err(phy->dev, "poll timeout SATA_PHY_TX_IMCAL_STAT\n");
		return ret;
	}

	/* RX Calibration */
	ret = read_poll_timeout(base + SATA_PHY_RX_IMCAL_STAT, SATA_PHY_RX_CAL);
	if (ret) {
		dev_err(phy->dev, "poll timeout SATA_PHY_RX_IMCAL_STAT\n");
		return ret;
	}

	/* SATA phy calibrated succesfully, power up to functional mode */
	writel_relaxed(0x3E, base + SATA_PHY_POW_DWN_CTRL1);
	writel_relaxed(0x01, base + SATA_PHY_RX_IMCAL0);
	writel_relaxed(0x01, base + SATA_PHY_TX_IMCAL0);

	writel_relaxed(0x00, base + SATA_PHY_POW_DWN_CTRL1);
	writel_relaxed(0x59, base + SATA_PHY_CDR_CTRL0);
	writel_relaxed(0x04, base + SATA_PHY_CDR_CTRL1);
	writel_relaxed(0x00, base + SATA_PHY_CDR_CTRL2);
	writel_relaxed(0x00, base + SATA_PHY_PI_CTRL0);
	writel_relaxed(0x00, base + SATA_PHY_CDR_CTRL3);
	writel_relaxed(0x01, base + SATA_PHY_POW_DWN_CTRL0);

	writel_relaxed(0x11, base + SATA_PHY_TX_DATA_CTRL);
	writel_relaxed(0x43, base + SATA_PHY_ALIGNP);
	writel_relaxed(0x04, base + SATA_PHY_OOB_TERM);

	writel_relaxed(0x01, base + SATA_PHY_EQUAL);
	writel_relaxed(0x09, base + SATA_PHY_TX_DRIV_CTRL0);
	writel_relaxed(0x09, base + SATA_PHY_TX_DRIV_CTRL1);

	return 0;
}

static int qcom_apq8064_sata_phy_exit(struct phy *generic_phy)
{
	struct qcom_apq8064_sata_phy *phy = phy_get_drvdata(generic_phy);
	void __iomem *base = phy->mmio;

	/* Power down PHY */
	writel_relaxed(0xF8, base + SATA_PHY_POW_DWN_CTRL0);
	writel_relaxed(0xFE, base + SATA_PHY_POW_DWN_CTRL1);

	/* Power down PLL block */
	writel_relaxed(0x00, base + UNIPHY_PLL_GLB_CFG);

	return 0;
}

static struct phy_ops qcom_apq8064_sata_phy_ops = {
	.init		= qcom_apq8064_sata_phy_init,
	.exit		= qcom_apq8064_sata_phy_exit,
	.owner		= THIS_MODULE,
};

static int qcom_apq8064_sata_phy_probe(struct platform_device *pdev)
{
	struct qcom_apq8064_sata_phy *phy;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct phy_provider *phy_provider;
	struct phy *generic_phy;
	int ret;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	phy->mmio = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy->mmio))
		return PTR_ERR(phy->mmio);

	generic_phy = devm_phy_create(dev, &qcom_apq8064_sata_phy_ops, NULL);
	if (IS_ERR(generic_phy)) {
		dev_err(dev, "%s: failed to create phy\n", __func__);
		return PTR_ERR(generic_phy);
	}

	phy->dev = dev;
	phy_set_drvdata(generic_phy, phy);

	phy->cfg_clk = devm_clk_get(dev, "cfg");
	if (IS_ERR(phy->cfg_clk)) {
		dev_err(dev, "Failed to get sata cfg clock\n");
		return PTR_ERR(phy->cfg_clk);
	}

	ret = clk_prepare_enable(phy->cfg_clk);
	if (ret)
		return ret;

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		clk_disable_unprepare(phy->cfg_clk);
		dev_err(dev, "%s: failed to register phy\n", __func__);
		return PTR_ERR(phy_provider);
	}

	return 0;
}

static int qcom_apq8064_sata_phy_remove(struct platform_device *pdev)
{
	struct qcom_apq8064_sata_phy *phy = platform_get_drvdata(pdev);

	clk_disable_unprepare(phy->cfg_clk);

	return 0;
}

static const struct of_device_id qcom_apq8064_sata_phy_of_match[] = {
	{ .compatible = "qcom,apq8064-sata-phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, qcom_apq8064_sata_phy_of_match);

static struct platform_driver qcom_apq8064_sata_phy_driver = {
	.probe	= qcom_apq8064_sata_phy_probe,
	.remove	= qcom_apq8064_sata_phy_remove,
	.driver = {
		.name	= "qcom-apq8064-sata-phy",
		.owner	= THIS_MODULE,
		.of_match_table	= qcom_apq8064_sata_phy_of_match,
	}
};
module_platform_driver(qcom_apq8064_sata_phy_driver);

MODULE_DESCRIPTION("QCOM apq8064 SATA PHY driver");
MODULE_LICENSE("GPL v2");
