/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/regulator/consumer.h>
#include <linux/elf.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/err.h>
#include <linux/clk.h>

//#include <mach/msm_bus.h>
//#include <mach/msm_iomap.h>

#include "peripheral-loader.h"
#include "pil-q6v4.h"
#include "scm-pas.h"

#define QDSP6SS_RST_EVB		0x0
#define QDSP6SS_RESET		0x04
#define QDSP6SS_STRAP_TCM	0x1C
#define QDSP6SS_STRAP_AHB	0x20
#define QDSP6SS_GFMUX_CTL	0x30
#define QDSP6SS_PWR_CTL		0x38

#define Q6SS_SS_ARES		BIT(0)
#define Q6SS_CORE_ARES		BIT(1)
#define Q6SS_ISDB_ARES		BIT(2)
#define Q6SS_ETM_ARES		BIT(3)
#define Q6SS_STOP_CORE_ARES	BIT(4)
#define Q6SS_PRIV_ARES		BIT(5)

#define Q6SS_L2DATA_SLP_NRET_N	BIT(0)
#define Q6SS_SLP_RET_N		BIT(1)
#define Q6SS_L1TCM_SLP_NRET_N	BIT(2)
#define Q6SS_L2TAG_SLP_NRET_N	BIT(3)
#define Q6SS_ETB_SLEEP_NRET_N	BIT(4)
#define Q6SS_ARR_STBY_N		BIT(5)
#define Q6SS_CLAMP_IO		BIT(6)

#define Q6SS_CLK_ENA		BIT(1)
#define Q6SS_SRC_SWITCH_CLK_OVR	BIT(8)

struct q6v4_data {
	void __iomem *base;
	unsigned long start_addr;
	struct regulator *vreg;
	struct regulator *pll_supply;
	bool vreg_enabled;
	struct clk *xo;
	struct pil_device *pil;

	/* platform specific data */
	u32 strap_tcm_base;
	u32 strap_ahb_upper;
	u32 strap_ahb_lower;
	u32 pas_id;
	struct	reset_control	*rstc;

};

static int pil_q6v4_init_image(struct pil_desc *pil, const u8 *metadata,
		size_t size)
{
	const struct elf32_hdr *ehdr = (struct elf32_hdr *)metadata;
	struct q6v4_data *drv = dev_get_drvdata(pil->dev);
	drv->start_addr = ehdr->e_entry;
	return 0;
}

static int pil_q6v4_make_proxy_votes(struct pil_desc *pil)
{
	const struct q6v4_data *drv = dev_get_drvdata(pil->dev);
	int ret;

	ret = clk_prepare_enable(drv->xo);
	if (ret) {
		dev_err(pil->dev, "Failed to enable XO\n");
//		goto err;
	}
	if (drv->pll_supply) {
		ret = regulator_enable(drv->pll_supply);
		if (ret) {
			dev_err(pil->dev, "Failed to enable pll supply\n");
			goto err_regulator;
		}
	}
	return 0;
err_regulator:
	clk_disable_unprepare(drv->xo);
//err:
	return ret;
}

static void pil_q6v4_remove_proxy_votes(struct pil_desc *pil)
{
	const struct q6v4_data *drv = dev_get_drvdata(pil->dev);
	if (drv->pll_supply)
		regulator_disable(drv->pll_supply);
	clk_disable_unprepare(drv->xo);
}

static int pil_q6v4_power_up(struct device *dev)
{
	int err;
	struct q6v4_data *drv = dev_get_drvdata(dev);

	//err = regulator_set_voltage(drv->vreg, 743750, 743750);
	err = regulator_set_voltage(drv->vreg,   750000, 750000);
	if (err) {
		dev_err(dev, "Failed to set regulator's voltage step.\n");
		return err;
	}
	err = regulator_enable(drv->vreg);
	if (err) {
		dev_err(dev, "Failed to enable regulator.\n");
		return err;
	}

	/*
	 * Q6 hardware requires a two step voltage ramp-up.
	 * Delay between the steps.
	 */
	udelay(100);

	err = regulator_set_voltage(drv->vreg, 1050000, 1050000);
	if (err) {
		dev_err(dev, "Failed to set regulator's voltage.\n");
		return err;
	}
	drv->vreg_enabled = true;
	return 0;
}

static DEFINE_MUTEX(pil_q6v4_modem_lock);
//static unsigned pil_q6v4_modem_count;

#if 0
/* Bring modem subsystem out of reset */
static void pil_q6v4_init_modem(void __iomem *base, void __iomem *jtag_clk)
{
	mutex_lock(&pil_q6v4_modem_lock);
	if (!pil_q6v4_modem_count) {
		/* Enable MSS clocks */
		writel_relaxed(0x10, SFAB_MSS_M_ACLK_CTL);
		writel_relaxed(0x10, SFAB_MSS_S_HCLK_CTL);
		writel_relaxed(0x10, MSS_S_HCLK_CTL);
		writel_relaxed(0x10, MSS_SLP_CLK_CTL);
		/* Wait for clocks to enable */
		mb();
		udelay(10);

		/* De-assert MSS reset */
		writel_relaxed(0x0, MSS_RESET);
		mb();
		udelay(10);
		/* Enable MSS */
		writel_relaxed(0x7, base);
	}

	/* Enable JTAG clocks */
	/* TODO: Remove if/when Q6 software enables them? */
	writel_relaxed(0x10, jtag_clk);

	pil_q6v4_modem_count++;
	mutex_unlock(&pil_q6v4_modem_lock);
}

/* Put modem subsystem back into reset */
static void pil_q6v4_shutdown_modem(void)
{
	mutex_lock(&pil_q6v4_modem_lock);
	if (pil_q6v4_modem_count)
		pil_q6v4_modem_count--;
	if (pil_q6v4_modem_count == 0)
		writel_relaxed(0x1, MSS_RESET);
	mutex_unlock(&pil_q6v4_modem_lock);
}
#endif
static int pil_q6v4_reset(struct pil_desc *pil)
{
	u32 reg, err;
	const struct q6v4_data *drv = dev_get_drvdata(pil->dev);
//	const struct pil_q6v4_pdata *pdata = pil->dev->platform_data;

	err = pil_q6v4_power_up(pil->dev);
	if (err)
		return err;
	/* Enable Q6 ACLK */
//BUG(); //FIXME
//	writel_relaxed(0x10, pdata->aclk_reg);
	if (drv->rstc)
		reset_control_deassert(drv->rstc);

	/* Unhalt bus port */
//	err = msm_bus_axi_portunhalt(pdata->bus_port);
//	if (err)
//		dev_err(pil->dev, "Failed to unhalt bus port\n");
//
	/* Deassert Q6SS_SS_ARES */
	reg = readl_relaxed(drv->base + QDSP6SS_RESET);
	reg &= ~(Q6SS_SS_ARES);
	writel_relaxed(reg, drv->base + QDSP6SS_RESET);

	/* Program boot address */
	writel_relaxed((drv->start_addr >> 8) & 0xFFFFFF,
			drv->base + QDSP6SS_RST_EVB);

	/* Program TCM and AHB address ranges */
	writel_relaxed(drv->strap_tcm_base, drv->base + QDSP6SS_STRAP_TCM);
	writel_relaxed(drv->strap_ahb_upper | drv->strap_ahb_lower,
		       drv->base + QDSP6SS_STRAP_AHB);

	/* Turn off Q6 core clock */
	writel_relaxed(Q6SS_SRC_SWITCH_CLK_OVR,
		       drv->base + QDSP6SS_GFMUX_CTL);

	/* Put memories to sleep */
	writel_relaxed(Q6SS_CLAMP_IO, drv->base + QDSP6SS_PWR_CTL);

	/* Assert resets */
	reg = readl_relaxed(drv->base + QDSP6SS_RESET);
	reg |= (Q6SS_CORE_ARES | Q6SS_ISDB_ARES | Q6SS_ETM_ARES
	    | Q6SS_STOP_CORE_ARES);
	writel_relaxed(reg, drv->base + QDSP6SS_RESET);

	/* Wait 8 AHB cycles for Q6 to be fully reset (AHB = 1.5Mhz) */
	mb();
	usleep_range(20, 30);

	/* Turn on Q6 memories */
	reg = Q6SS_L2DATA_SLP_NRET_N | Q6SS_SLP_RET_N | Q6SS_L1TCM_SLP_NRET_N
	    | Q6SS_L2TAG_SLP_NRET_N | Q6SS_ETB_SLEEP_NRET_N | Q6SS_ARR_STBY_N
	    | Q6SS_CLAMP_IO;
	writel_relaxed(reg, drv->base + QDSP6SS_PWR_CTL);

	/* Turn on Q6 core clock */
	reg = Q6SS_CLK_ENA | Q6SS_SRC_SWITCH_CLK_OVR;
	writel_relaxed(reg, drv->base + QDSP6SS_GFMUX_CTL);

	/* Remove Q6SS_CLAMP_IO */
	reg = readl_relaxed(drv->base + QDSP6SS_PWR_CTL);
	reg &= ~Q6SS_CLAMP_IO;
	writel_relaxed(reg, drv->base + QDSP6SS_PWR_CTL);

	/* Bring Q6 core out of reset and start execution. */
	writel_relaxed(0x0, drv->base + QDSP6SS_RESET);

	return 0;
}

static int pil_q6v4_shutdown(struct pil_desc *pil)
{
	u32 reg;
	struct q6v4_data *drv = dev_get_drvdata(pil->dev);
//	const struct pil_q6v4_pdata *pdata = pil->dev->platform_data;

	/* Make sure bus port is halted */
	//msm_bus_axi_porthalt(pdata->bus_port);

	/* Turn off Q6 core clock */
	writel_relaxed(Q6SS_SRC_SWITCH_CLK_OVR,
		       drv->base + QDSP6SS_GFMUX_CTL);

	/* Assert resets */
	reg = (Q6SS_SS_ARES | Q6SS_CORE_ARES | Q6SS_ISDB_ARES
	     | Q6SS_ETM_ARES | Q6SS_STOP_CORE_ARES | Q6SS_PRIV_ARES);
	writel_relaxed(reg, drv->base + QDSP6SS_RESET);

	/* Turn off Q6 memories */
	writel_relaxed(Q6SS_CLAMP_IO, drv->base + QDSP6SS_PWR_CTL);


	if (drv->vreg_enabled) {
		regulator_disable(drv->vreg);
		drv->vreg_enabled = false;
	}

	return 0;
}

static struct pil_reset_ops pil_q6v4_ops = {
	.init_image = pil_q6v4_init_image,
	.auth_and_reset = pil_q6v4_reset,
	.shutdown = pil_q6v4_shutdown,
	.proxy_vote = pil_q6v4_make_proxy_votes,
	.proxy_unvote = pil_q6v4_remove_proxy_votes,
};

static int pil_q6v4_init_image_trusted(struct pil_desc *pil,
		const u8 *metadata, size_t size)
{
	int ret;
	//const struct pil_q6v4_pdata *pdata = pil->dev->platform_data;
	struct q6v4_data *drv = dev_get_drvdata(pil->dev);

	ret =  pas_init_image(drv->pas_id, metadata, size);
	printk("DEBUG: %s ret %d \n", __func__, ret);
	return ret;
}

static int pil_q6v4_reset_trusted(struct pil_desc *pil)
{
	//const struct pil_q6v4_pdata *pdata = pil->dev->platform_data;
	struct q6v4_data *drv = dev_get_drvdata(pil->dev);
	int err;

	err = pil_q6v4_power_up(pil->dev);
	if (err)
		return err;

	/* Unhalt bus port */
//	err = msm_bus_axi_portunhalt(pdata->bus_port);
//	if (err)
//		dev_err(pil->dev, "Failed to unhalt bus port\n");
	return pas_auth_and_reset(drv->pas_id);
}

static int pil_q6v4_shutdown_trusted(struct pil_desc *pil)
{
	int ret;
	struct q6v4_data *drv = dev_get_drvdata(pil->dev);
//	struct pil_q6v4_pdata *pdata = pil->dev->platform_data;

	/* Make sure bus port is halted */
	//msm_bus_axi_porthalt(pdata->bus_port);

	ret = pas_shutdown(drv->pas_id);
	if (ret)
		return ret;

	if (drv->vreg_enabled) {
		regulator_disable(drv->vreg);
		drv->vreg_enabled = false;
	}

	return ret;
}

static struct pil_reset_ops pil_q6v4_ops_trusted = {
	.init_image = pil_q6v4_init_image_trusted,
	.auth_and_reset = pil_q6v4_reset_trusted,
	.shutdown = pil_q6v4_shutdown_trusted,
//	.proxy_vote = pil_q6v4_make_proxy_votes,
 //	.proxy_unvote = pil_q6v4_remove_proxy_votes,
};

static int pil_q6v4_driver_probe(struct platform_device *pdev)
{
	struct q6v4_data *drv;
	struct resource *res;
	struct pil_desc *desc;
//	int ret;
	struct device_node *np = pdev->dev.of_node;

	printk("DEBUG:::::::::::::: %s \n", __func__);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;
	platform_set_drvdata(pdev, drv);

	request_mem_region(res->start, resource_size(res), pdev->name);

	drv->base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!drv->base)
		return -ENOMEM;


	desc = devm_kzalloc(&pdev->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	if (np) {
		of_property_read_u32(np, "strap-tcm-base", &drv->strap_tcm_base);
		of_property_read_u32(np, "strap-ahb-lower", &drv->strap_ahb_lower);
		of_property_read_u32(np, "strap-ahb-upper", &drv->strap_ahb_upper);
		of_property_read_u32(np, "pas-id", &drv->pas_id);
		of_property_read_string(np, "pas-name", &desc->name);
		drv->rstc = reset_control_get_optional(&pdev->dev, "lpass");
		if (IS_ERR(drv->rstc))
			drv->rstc = NULL;
	} else  {
		const struct pil_q6v4_pdata *pdata;
		pdata = pdev->dev.platform_data;
		drv->strap_tcm_base = pdata->strap_tcm_base;
		drv->strap_ahb_lower = pdata->strap_ahb_lower;
		drv->strap_ahb_upper = pdata->strap_ahb_upper;
		//drv->aclk_reg = pdata->aclk_reg;

		drv->pas_id = pdata->pas_id;
		/* No support to clk reset here */
	}

	// desc->name = pdata->name;
//	desc->depends_on = pdata->depends;
	desc->dev = &pdev->dev;
	desc->owner = THIS_MODULE;
	desc->proxy_timeout = 10000;

	printk("DEBUG:::::::::::; pas id %d \n", drv->pas_id);

	if (pas_supported(drv->pas_id) > 0) {
		desc->ops = &pil_q6v4_ops_trusted;
		dev_info(&pdev->dev, "using secure boot\n");
	} else {
		desc->ops = &pil_q6v4_ops;
		dev_info(&pdev->dev, "using non-secure boot\n");
	}

	drv->vreg = devm_regulator_get(&pdev->dev, "core-vdd");
	if (IS_ERR(drv->vreg)) {
		dev_err(&pdev->dev, "Failed to set regulator's mode.\n");
		return PTR_ERR(drv->vreg);
	}

	drv->pil = msm_pil_register(desc);
	if (IS_ERR(drv->pil))
		return PTR_ERR(drv->pil);
	printk("DEBUG:::::::::::::: %s successful\n", __func__);
	return 0;
}

static int pil_q6v4_driver__exit(struct platform_device *pdev)
{
	struct q6v4_data *drv = platform_get_drvdata(pdev);
	msm_pil_unregister(drv->pil);
	return 0;
}

static struct of_device_id pil_q6v4_match[] = {
	{ .compatible = "qcom,pil-qdsp6v4", },
	{},
};

static struct platform_driver pil_q6v4_driver = {
	.probe = pil_q6v4_driver_probe,
	.remove = pil_q6v4_driver__exit,
	.driver = {
		.name = "pil_qdsp6v4",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pil_q6v4_match),
	},
};

module_platform_driver(pil_q6v4_driver);

MODULE_DESCRIPTION("Support for booting QDSP6v4 (Hexagon) processors");
MODULE_LICENSE("GPL v2");
