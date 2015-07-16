/*
 * PCIe host controller driver for Freescale S32V SoCs
 *
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2013 Kosagi
 *		http://www.kosagi.com
 *
 * Author: Sean Cross <xobs@kosagi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/s32v234_src.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <linux/regulator/consumer.h>

#include "pcie-designware.h"

#define to_s32v234_pcie(x)	container_of(x, struct s32v234_pcie, pp)

struct s32v234_pcie {
	int			reset_gpio;
	int			power_on_gpio;
	struct clk		*pcie_bus;
	struct clk		*pcie_phy;
	struct clk		*pcie;
	struct pcie_port	pp;
	struct regmap		*src;
	void __iomem		*mem_base;
};

/* PCIe Root Complex registers (memory-mapped) */
#define PCIE_RC_LCR				0x7c
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1	0x1
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2	0x2
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK	0xf

/* PCIe Port Logic registers (memory-mapped) */
#define PL_OFFSET 0x700
#define PCIE_PL_PFLR (PL_OFFSET + 0x08)
#define PCIE_PL_PFLR_LINK_STATE_MASK		(0x3f << 16)
#define PCIE_PL_PFLR_FORCE_LINK			(1 << 15)
#define PCIE_PHY_DEBUG_R0 (PL_OFFSET + 0x28)
#define PCIE_PHY_DEBUG_R1 (PL_OFFSET + 0x2c)
#define PCIE_PHY_DEBUG_R1_XMLH_LINK_IN_TRAINING	(1 << 29)
#define PCIE_PHY_DEBUG_R1_XMLH_LINK_UP		(1 << 4)

#define PCIE_PHY_CTRL (PL_OFFSET + 0x114)
#define PCIE_PHY_CTRL_DATA_LOC 0
#define PCIE_PHY_CTRL_CAP_ADR_LOC 16
#define PCIE_PHY_CTRL_CAP_DAT_LOC 17
#define PCIE_PHY_CTRL_WR_LOC 18
#define PCIE_PHY_CTRL_RD_LOC 19

#define PCIE_PHY_STAT (PL_OFFSET + 0x110)
#define PCIE_PHY_STAT_ACK_LOC 16

#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C
#define PORT_LOGIC_SPEED_CHANGE		(0x1 << 17)

/* PHY registers (not memory-mapped) */
#define PCIE_PHY_RX_ASIC_OUT 0x100D

#define PHY_RX_OVRD_IN_LO 0x1005
#define PHY_RX_OVRD_IN_LO_RX_DATA_EN (1 << 5)
#define PHY_RX_OVRD_IN_LO_RX_PLL_EN (1 << 3)

static void regmap_update_bits_local(struct regmap *map, unsigned int reg,
			unsigned int mask, unsigned int val)
{
	unsigned int *ptr;
	unsigned int tmp;

	tmp = 0x4007C000 + reg;
	ptr = (unsigned int *)ioremap(tmp, 0x10);
	tmp =	*ptr;
	tmp &= ~mask;
	tmp |= val & mask;
	*ptr = tmp;
}

static inline bool is_S32V234_pcie(struct s32v234_pcie *s32v234_pcie)
{
	struct pcie_port *pp	= &s32v234_pcie->pp;
	struct device_node *np	= pp->dev->of_node;

	return of_device_is_compatible(np, "fsl,s32v234-pcie");
}

static int pcie_phy_poll_ack(void __iomem *dbi_base, int exp_val)
{
	u32 val;
	u32 max_iterations = 10;
	u32 wait_counter = 0;

	do {
		val = readl(dbi_base + PCIE_PHY_STAT);
		val = (val >> PCIE_PHY_STAT_ACK_LOC) & 0x1;
		wait_counter++;

		if (val == exp_val)
			return 0;

		udelay(1);
	} while (wait_counter < max_iterations);

	return -ETIMEDOUT;
}

static int pcie_phy_wait_ack(void __iomem *dbi_base, int addr)
{
	u32 val;
	int ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	writel(val, dbi_base + PCIE_PHY_CTRL);

	val |= (0x1 << PCIE_PHY_CTRL_CAP_ADR_LOC);
	writel(val, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	writel(val, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	return 0;
}

/* Read from the 16-bit PCIe PHY control registers (not memory-mapped) */
static int pcie_phy_read(void __iomem *dbi_base, int addr , int *data)
{
	u32 val, phy_ctl;
	int ret;

	ret = pcie_phy_wait_ack(dbi_base, addr);
	if (ret)
		return ret;

	/* assert Read signal */
	phy_ctl = 0x1 << PCIE_PHY_CTRL_RD_LOC;
	writel(phy_ctl, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	val = readl(dbi_base + PCIE_PHY_STAT);
	*data = val & 0xffff;

	/* deassert Read signal */
	writel(0x00, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	return 0;
}

static int pcie_phy_write(void __iomem *dbi_base, int addr, int data)
{
	u32 var;
	int ret;

	/* write addr */
	/* cap addr */
	ret = pcie_phy_wait_ack(dbi_base, addr);
	if (ret)
		return ret;

	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* capture data */
	var |= (0x1 << PCIE_PHY_CTRL_CAP_DAT_LOC);
	writel(var, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	/* deassert cap data */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	/* assert wr signal */
	var = 0x1 << PCIE_PHY_CTRL_WR_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack */
	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	/* deassert wr signal */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	writel(0x0, dbi_base + PCIE_PHY_CTRL);

	return 0;
}

static int s32v234_pcie_assert_core_reset(struct pcie_port *pp)
{
	struct s32v234_pcie *s32v234_pcie = to_s32v234_pcie(pp);

	if (is_S32V234_pcie(s32v234_pcie)) {
		regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
				SRC_GPR5_GPR_PCIE_BUTTON_RST_N,
				SRC_GPR5_GPR_PCIE_BUTTON_RST_N);
	}

	return 0;
}

static int s32v234_pcie_deassert_core_reset(struct pcie_port *pp)
{
	struct s32v234_pcie *s32v234_pcie = to_s32v234_pcie(pp);

	/* allow the clocks to stabilize */
	udelay(200);
	/*
	 * Release the PCIe PHY reset here, that we have set in
	 * s32v234_pcie_init_phy() now
	 */
	if (is_S32V234_pcie(s32v234_pcie))
		regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
		SRC_GPR5_GPR_PCIE_BUTTON_RST_N, 0);

	return 0;

}

static int s32v234_pcie_init_phy(struct pcie_port *pp)
{
	struct s32v234_pcie *s32v234_pcie = to_s32v234_pcie(pp);

	regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
			SRC_GPR5_PCIE_APP_LTSSM_ENABLE, 0 << 10);

	regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
				SRC_GPR5_PCIE_DEVICE_TYPE_MASK,
				PCI_EXP_TYPE_ROOT_PORT << 1);
	regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
			SRC_GPR5_PCIE_PHY_LOS_LEVEL_MASK, (0x9 << 22));
	return 0;
}

static int s32v234_pcie_wait_for_link(struct pcie_port *pp)
{
	int count	= 200;

	while (!dw_pcie_link_up(pp)) {
		udelay(100);
		if (--count)
			continue;
		dev_err(pp->dev, "phy link never came up\n");
		dev_dbg(pp->dev, "DEBUG_R0: 0x%08x, DEBUG_R1: 0x%08x\n",
			readl(pp->dbi_base + PCIE_PHY_DEBUG_R0),
			readl(pp->dbi_base + PCIE_PHY_DEBUG_R1));
		return -EINVAL;
	}

	return 0;
}
#ifdef CONFIG_PCI_MSI
static irqreturn_t s32v234_pcie_msi_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	return dw_handle_msi_irq(pp);
}
#endif
static int s32v234_pcie_start_link(struct pcie_port *pp)
{
	struct s32v234_pcie *s32v234_pcie = to_s32v234_pcie(pp);
	uint32_t tmp;
	int ret, count;

	/*
	 * Force Gen1 operation when starting the link.  In case the link is
	 * started in Gen2 mode, there is a possibility the devices on the
	 * bus will not be detected at all.  This happens with PCIe switches.
	 */
	tmp = readl(pp->dbi_base + PCIE_RC_LCR);
	tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
	tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1;
	writel(tmp, pp->dbi_base + PCIE_RC_LCR);

	/* Start LTSSM. */

	regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
			SRC_GPR5_PCIE_APP_LTSSM_ENABLE,
			SRC_GPR5_PCIE_APP_LTSSM_ENABLE);

	ret = s32v234_pcie_wait_for_link(pp);

	udelay(200);
	if (ret)
		goto out;

	/* Allow Gen2 mode after the link is up. */
	tmp = readl(pp->dbi_base + PCIE_RC_LCR);
	tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
	tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2;
	writel(tmp, pp->dbi_base + PCIE_RC_LCR);

	/*
	 * Start Directed Speed Change so the best possible speed both link
	 * partners support can be negotiated.
	 */
	tmp = readl(pp->dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);
	tmp |= PORT_LOGIC_SPEED_CHANGE;
	writel(tmp, pp->dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);

	count = 200;
	while (count--) {
		tmp = readl(pp->dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);
		/* Test if the speed change finished. */
		if (!(tmp & PORT_LOGIC_SPEED_CHANGE))
			break;
		udelay(100);
	}

	/* Make sure link training is finished as well! */
	if (count)
		ret = s32v234_pcie_wait_for_link(pp);
	else
		ret = -EINVAL;

out:
	if (ret) {
		dev_err(pp->dev, "Failed to bring link up!\n");
	} else {
		tmp = readl(pp->dbi_base + 0x80);
		dev_dbg(pp->dev, "Link up, Gen=%i\n", (tmp >> 16) & 0xf);
	}
	return ret;
}

static int s32v234_pcie_host_init(struct pcie_port *pp)
{
	int ret;

	/* enable disp_mix power domain */
	pm_runtime_get_sync(pp->dev);

	s32v234_pcie_assert_core_reset(pp);

	ret = s32v234_pcie_init_phy(pp);
	if (ret < 0)
		return ret;

	ret = s32v234_pcie_deassert_core_reset(pp);
	if (ret < 0)
		return ret;

	dw_pcie_setup_rc(pp);

	ret = s32v234_pcie_start_link(pp);
	if (ret < 0)
		return ret;

	#ifdef CONFIG_PCI_MSI
	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_init(pp);
	#endif

	return 0;
}

static void s32v234_pcie_reset_phy(struct pcie_port *pp)
{
	uint32_t temp;

	pcie_phy_read(pp->dbi_base, PHY_RX_OVRD_IN_LO, &temp);
	temp |= (PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		 PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(pp->dbi_base, PHY_RX_OVRD_IN_LO, temp);

	udelay(2000);

	pcie_phy_read(pp->dbi_base, PHY_RX_OVRD_IN_LO, &temp);
	temp &= ~(PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		  PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(pp->dbi_base, PHY_RX_OVRD_IN_LO, temp);
}

static int s32v234_pcie_link_up(struct pcie_port *pp)
{
	u32 rc, debug_r0, rx_valid;
	int count = 500;

	/*
	 * Test if the PHY reports that the link is up and also that the LTSSM
	 * training finished. There are three possible states of the link when
	 * this code is called:
	 * 1) The link is DOWN (unlikely)
	 *     The link didn't come up yet for some reason. This usually means
	 *     we have a real problem somewhere. Reset the PHY and exit. This
	 *     state calls for inspection of the DEBUG registers.
	 * 2) The link is UP, but still in LTSSM training
	 *     Wait for the training to finish, which should take a very short
	 *     time. If the training does not finish, we have a problem and we
	 *     need to inspect the DEBUG registers. If the training does finish,
	 *     the link is up and operating correctly.
	 * 3) The link is UP and no longer in LTSSM training
	 *     The link is up and operating correctly.
	 */
	while (1) {
		rc = readl(pp->dbi_base + PCIE_PHY_DEBUG_R1);
		if (!(rc & PCIE_PHY_DEBUG_R1_XMLH_LINK_UP))
			break;
		if (!(rc & PCIE_PHY_DEBUG_R1_XMLH_LINK_IN_TRAINING))
			return 1;
		if (!count--)
			break;
		dev_dbg(pp->dev, "Link is up, but still in training\n");
		/*
		 * Wait a little bit, then re-check if the link finished
		 * the training.
		 */
		udelay(10);
	}
	/*
	 * From L0, initiate MAC entry to gen2 if EP/RC supports gen2.
	 * Wait 2ms (LTSSM timeout is 24ms, PHY lock is ~5us in gen2).
	 * If (MAC/LTSSM.state == Recovery.RcvrLock)
	 * && (PHY/rx_valid==0) then pulse PHY/rx_reset. Transition
	 * to gen2 is stuck
	 */
	pcie_phy_read(pp->dbi_base, PCIE_PHY_RX_ASIC_OUT, &rx_valid);
	debug_r0 = readl(pp->dbi_base + PCIE_PHY_DEBUG_R0);

	if (rx_valid & 0x01)
		return 0;

	if ((debug_r0 & 0x3f) != 0x0d)
		return 0;

	dev_err(pp->dev, "transition to gen2 is stuck, reset PHY!\n");
	dev_dbg(pp->dev, "debug_r0=%08x debug_r1=%08x\n", debug_r0, rc);

	s32v234_pcie_reset_phy(pp);

	return 0;
}

static struct pcie_host_ops s32v234_pcie_host_ops = {
	.link_up = s32v234_pcie_link_up,
	.host_init = (void(*)(struct pcie_port *pp))s32v234_pcie_host_init,
};

static int __init s32v234_add_pcie_port(struct pcie_port *pp,
			struct platform_device *pdev)
{
	int ret;

	pp->root_bus_nr = 0;
	pp->ops = &s32v234_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pci_s32v_suspend_noirq(struct device *dev)
{
	struct s32v234_pcie *s32v234_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &s32v234_pcie->pp;

	if (is_S32V234_pcie(s32v234_pcie)) {
		#ifdef CONFIG_PCI_MSI
		if (IS_ENABLED(CONFIG_PCI_MSI))
			dw_pcie_msi_cfg_store(pp);
		#endif
		if (IS_ENABLED(CONFIG_PCI_S32V234_EXTREMELY_PWR_SAVE)) {
			/* PM_TURN_OFF */
			regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
					SRC_GPR5_PCIE_APPS_PM_XMT_TURNOFF,
					SRC_GPR5_PCIE_APPS_PM_XMT_TURNOFF);
			udelay(10);
			regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
					SRC_GPR5_PCIE_APPS_PM_XMT_TURNOFF, 0);
		} else {
			/* PM_TURN_OFF */
			regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
					SRC_GPR5_PCIE_APPS_PM_XMT_TURNOFF,
					SRC_GPR5_PCIE_APPS_PM_XMT_TURNOFF);
			udelay(10);
			regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
					SRC_GPR5_PCIE_APPS_PM_XMT_TURNOFF, 0);
		}
	}
	return 0;
}

static int pci_s32v_resume_noirq(struct device *dev)
{
	int ret = 0;
	struct s32v234_pcie *s32v234_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &s32v234_pcie->pp;

	if (is_S32V234_pcie(s32v234_pcie)) {
		if (IS_ENABLED(CONFIG_PCI_S32V234_EXTREMELY_PWR_SAVE)) {
			s32v234_pcie_assert_core_reset(pp);

			ret = s32v234_pcie_init_phy(pp);
			if (ret < 0)
				return ret;

			ret = s32v234_pcie_deassert_core_reset(pp);
			if (ret < 0)
				return ret;

			else
				dw_pcie_setup_rc(pp);

			#ifdef CONFIG_PCI_MSI
			if (IS_ENABLED(CONFIG_PCI_MSI))
				dw_pcie_msi_cfg_restore(pp);
			#endif
			/* Start LTSSM. */
			regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
					SRC_GPR5_PCIE_APP_LTSSM_ENABLE,
					SRC_GPR5_PCIE_APP_LTSSM_ENABLE);
		} else {

			regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
					SRC_GPR5_GPR_PCIE_PERST_N,
					SRC_GPR5_GPR_PCIE_PERST_N);
			regmap_update_bits_local(s32v234_pcie->src, SRC_GPR5,
					SRC_GPR5_GPR_PCIE_PERST_N, 0);
			/*
			 * controller maybe turn off, re-configure again
			 */
			dw_pcie_setup_rc(pp);
			#ifdef CONFIG_PCI_MSI
			if (IS_ENABLED(CONFIG_PCI_MSI))
				dw_pcie_msi_cfg_restore(pp);
			#endif
		}
	}

	return 0;
}

static const struct dev_pm_ops pci_s32v_pm_ops = {
	.suspend_noirq = pci_s32v_suspend_noirq,
	.resume_noirq = pci_s32v_resume_noirq,
	.freeze_noirq = pci_s32v_suspend_noirq,
	.thaw_noirq = pci_s32v_resume_noirq,
	.poweroff_noirq = pci_s32v_suspend_noirq,
	.restore_noirq = pci_s32v_resume_noirq,
};
#endif

static int s32v234_pcie_probe(struct platform_device *pdev)
{
	struct s32v234_pcie *s32v234_pcie;
	struct pcie_port *pp;
	struct resource *dbi_base;
	int ret;

	s32v234_pcie = devm_kzalloc(&pdev->dev, sizeof(*s32v234_pcie)
					, GFP_KERNEL);
	if (!s32v234_pcie)
		return -ENOMEM;

	pp = &s32v234_pcie->pp;
	pp->dev = &pdev->dev;


	/* Added for PCI abort handling */
	dbi_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pp->dbi_base = devm_ioremap_resource(&pdev->dev, dbi_base);
	if (IS_ERR(pp->dbi_base))
		return PTR_ERR(pp->dbi_base);

	ret = s32v234_add_pcie_port(pp, pdev);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, s32v234_pcie);

	return 0;
}

static void s32v234_pcie_shutdown(struct platform_device *pdev)
{
	struct s32v234_pcie *s32v234_pcie = platform_get_drvdata(pdev);

	/* bring down link, so bootloader gets clean state in case of reboot */
	s32v234_pcie_assert_core_reset(&s32v234_pcie->pp);
}

static const struct of_device_id s32v234_pcie_of_match[] = {
	{ .compatible = "fsl,s32v234-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, s32v234_pcie_of_match);

static struct platform_driver s32v234_pcie_driver = {
	.driver = {
		.name	= "s32v234-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = s32v234_pcie_of_match,
		#ifdef CONFIG_PM_SLEEP
		.pm = &pci_s32v_pm_ops,
		#endif
	},
	.probe = s32v234_pcie_probe,
	.shutdown = s32v234_pcie_shutdown,
};

/* Freescale PCIe driver does not allow module unload */

static int __init s32v234_pcie_init(void)
{
	return platform_driver_probe(&s32v234_pcie_driver, s32v234_pcie_probe);
}
module_init(s32v234_pcie_init);

MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("Freescale S32V PCIe host controller driver");
MODULE_LICENSE("GPL v2");
