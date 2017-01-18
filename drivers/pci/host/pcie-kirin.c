/*
 * PCIe host controller driver for Kirin SoCs
 *
 * Copyright (C) 2015 Hilisicon Electronics Co., Ltd.
 *		http://www.huawei.com
 *
 * Author: Xiaowei Song <songxiaowei@huawei.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "pcie-kirin.h"

struct kirin_pcie g_kirin_pcie[] = {
	{
		.irq = {
				{ .name = "kirin-pcie0-inta", },
				{ .name = "kirin-pcie0-msi", },
				{ .name = "kirin-pcie0-intc", },
				{ .name = "kirin-pcie0-intd", },
				{ .name = "kirin-pcie0-linkdown", }
			},
	},
};

void enable_req_clk(struct kirin_pcie *pcie, u32 enable_flag)
{
	u32 val;

	val = kirin_elb_readl(pcie, SOC_PCIECTRL_CTRL1_ADDR);

	if (enable_flag)
		val &= ~PCIE_APB_CLK_REQ;
	else
		val |= PCIE_APB_CLK_REQ;

	kirin_elb_writel(pcie, val, SOC_PCIECTRL_CTRL1_ADDR);
}

static int kirin_pcie_link_up(struct pcie_port *pp);

void kirin_elb_writel(struct kirin_pcie *pcie, u32 val, u32 reg)
{
	writel(val, pcie->apb_base + reg);
}

u32 kirin_elb_readl(struct kirin_pcie *pcie, u32 reg)
{
	return readl(pcie->apb_base + reg);
}

/*Registers in PCIePHY*/
void kirin_phy_writel(struct kirin_pcie *pcie, u32 val, u32 reg)
{
	writel(val, pcie->phy_base + reg);
}

u32 kirin_phy_readl(struct kirin_pcie *pcie, u32 reg)
{
	return readl(pcie->phy_base + reg);
}

static int32_t kirin_pcie_get_clk(struct kirin_pcie *pcie, struct platform_device *pdev)
{
	pcie->phy_ref_clk = devm_clk_get(&pdev->dev, "pcie_phy_ref");
	if (IS_ERR(pcie->phy_ref_clk)) {
		PCIE_PR_ERR("Failed to get pcie_phy_ref clock");
		return PTR_ERR(pcie->phy_ref_clk);
	}

	pcie->pcie_aux_clk = devm_clk_get(&pdev->dev, "pcie_aux");
	if (IS_ERR(pcie->pcie_aux_clk)) {
		PCIE_PR_ERR("Failed to get pcie_aux clock");
		return PTR_ERR(pcie->pcie_aux_clk);
	}

	pcie->apb_phy_clk = devm_clk_get(&pdev->dev, "pcie_apb_phy");
	if (IS_ERR(pcie->apb_phy_clk)) {
		PCIE_PR_ERR("Failed to get pcie_apb_phy clock");
		return PTR_ERR(pcie->apb_phy_clk);
	}

	pcie->apb_sys_clk = devm_clk_get(&pdev->dev, "pcie_apb_sys");
	if (IS_ERR(pcie->apb_sys_clk)) {
		PCIE_PR_ERR("Failed to get pcie_apb_sys clock");
		return PTR_ERR(pcie->apb_sys_clk);
	}

	pcie->pcie_aclk = devm_clk_get(&pdev->dev, "pcie_aclk");
	if (IS_ERR(pcie->pcie_aclk)) {
		PCIE_PR_ERR("Failed to get pcie_aclk clock");
		return PTR_ERR(pcie->pcie_aclk);
	}
	PCIE_PR_INFO("Successed to get all clock");

	return 0;
}

static int32_t kirin_pcie_get_resource(struct pcie_port *pp, struct platform_device *pdev)
{
	struct resource *apb;
	struct resource *phy;
	struct resource *dbi;
	struct kirin_pcie *pcie = to_kirin_pcie(pp);

	apb = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apb");
	pcie->apb_base = devm_ioremap_resource(&pdev->dev, apb);
	if (IS_ERR(pcie->apb_base)) {
		PCIE_PR_ERR("cannot get apb base");
		return PTR_ERR(pcie->apb_base);
	}

	phy = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy");
	pcie->phy_base = devm_ioremap_resource(&pdev->dev, phy);
	if (IS_ERR(pcie->phy_base)) {
		PCIE_PR_ERR("cannot get PCIePHY base");
		return PTR_ERR(pcie->phy_base);
	}

	dbi = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pp->dbi_base = devm_ioremap_resource(&pdev->dev, dbi);
	if (IS_ERR(pp->dbi_base)) {
		PCIE_PR_ERR("cannot get PCIe dbi base");
		return PTR_ERR(pp->dbi_base);
	}

	pcie->crgctrl = syscon_regmap_lookup_by_compatible("hisilicon,hi3660-crgctrl");
	if (IS_ERR(pcie->crgctrl)) {
		PCIE_PR_ERR("cannot get crgctrl base\n");
		return PTR_ERR(pcie->crgctrl);
	}

	pcie->sysctrl = syscon_regmap_lookup_by_compatible("hisilicon,hi3660-sctrl");
	if (IS_ERR(pcie->sysctrl)) {
		PCIE_PR_ERR("cannot get sysctrl base\n");
		return PTR_ERR(pcie->sysctrl);
	}
	PCIE_PR_INFO("Successed to get all resource");
	return 0;
}

static int32_t kirin_pcie_get_pinctrl(struct kirin_pcie *pcie, struct platform_device *pdev)
{
        struct pinctrl *p;
        int gpio_id;

        p = devm_pinctrl_get(&pdev->dev);
        if (IS_ERR(p)) {
               PCIE_PR_ERR("could not get pinctrl");
                return -1;
        }

        gpio_id = of_get_named_gpio(pdev->dev.of_node, "reset-gpio", 0);
        if (gpio_id < 0) {
                PCIE_PR_ERR("can't get gpio number");
                return -1;
        }

        pcie->gpio_id_reset = gpio_id;
        pcie->pin = p;
        return 0;
}

static int32_t kirin_pcie_get_boardtype(struct kirin_pcie *pcie, struct platform_device *pdev)
{
	int len;
	int ret;
	u32 eye_param;
	size_t array_num = 2;
	struct device_node *np;
	u32 val[2];

	np = pdev->dev.of_node;
	if (of_property_read_u32(np, "board_type", &pcie->board_type)) {
		PCIE_PR_ERR("Failed to get BoardType");
		pcie->board_type = 2;
	}
	PCIE_PR_INFO("BoardType value [%d] ", pcie->board_type);

	if (of_find_property(np, "ep_flag", &len)) {
		pcie->ep_flag = 1;
		PCIE_PR_INFO("EndPoint Device");
	} else {
		pcie->ep_flag = 0;
		PCIE_PR_INFO("RootComplex");
	}
	ret = of_property_read_u32(np, "eye_param_ctrl2", &eye_param);
	if (ret) {
		PCIE_PR_ERR("Failed to get eye param");
		pcie->pcie_eye_param_ctrl2 = 0x0;
	} else {
		pcie->pcie_eye_param_ctrl2 = eye_param;
		PCIE_PR_INFO("eye param value [0x%x] ", eye_param);
	}

	ret = of_property_read_u32(np, "eye_param_ctrl3", &eye_param);
	if (ret) {
		PCIE_PR_ERR("Failed to get eye param");
		pcie->pcie_eye_param_ctrl3 = 0x0;
	} else {
		pcie->pcie_eye_param_ctrl3 = eye_param;
		PCIE_PR_INFO("eye param value [0x%x] ", eye_param);
	}
	if (of_property_read_u32_array(np, "isoen", val, array_num)) {
		PCIE_PR_ERR("Failed to read isoen dts info");
		return -1;
	}
	pcie->isoen_offset = val[0];
	pcie->isoen_value = val[1];
	if (of_property_read_u32_array(np, "isodis", val, array_num)) {
		PCIE_PR_ERR("Failed to read isodis dts info");
		return -1;
	}
	pcie->isodis_offset = val[0];
	pcie->isodis_value = val[1];

	if (of_property_read_u32_array(np, "phy_assert", val, array_num)) {
		PCIE_PR_ERR("Failed to read phy_assert dts info");
		return -1;
	}
	pcie->phy_assert_offset = val[0];
	pcie->phy_assert_value = val[1];
	if (of_property_read_u32_array(np, "phy_deassert", val, array_num)) {
		PCIE_PR_ERR("Failed to read phy_deassert dts info");
		return -1;
	}
	pcie->phy_deassert_offset = val[0];
	pcie->phy_deassert_value = val[1];
	if (of_property_read_u32_array(np, "core_assert", val, array_num)) {
		PCIE_PR_ERR("Failed to read phy_assert dts info");
		return -1;
	}
	pcie->core_assert_offset = val[0];
	pcie->core_assert_value = val[1];
	if (of_property_read_u32_array(np, "core_deassert", val, array_num)) {
		PCIE_PR_ERR("Failed to read phy_deassert dts info");
		return -1;
	}
	pcie->core_deassert_offset = val[0];
	pcie->core_deassert_value = val[1];

	return 0;
}

static void kirin_pcie_sideband_dbi_w_mode(struct pcie_port *pp, bool on)
{
	u32 val;
	struct kirin_pcie *pcie = to_kirin_pcie(pp);

	val = kirin_elb_readl(pcie, SOC_PCIECTRL_CTRL0_ADDR);
	if (on)
		val = val | PCIE_ELBI_SLV_DBI_ENABLE;
	else
		val = val & ~PCIE_ELBI_SLV_DBI_ENABLE;

	kirin_elb_writel(pcie, val, SOC_PCIECTRL_CTRL0_ADDR);
}

static void kirin_pcie_sideband_dbi_r_mode(struct pcie_port *pp, bool on)
{
	u32 val;
	struct kirin_pcie *pcie = to_kirin_pcie(pp);

	val = kirin_elb_readl(pcie, SOC_PCIECTRL_CTRL1_ADDR);
	if (on)
		val = val | PCIE_ELBI_SLV_DBI_ENABLE;
	else
		val = val & ~PCIE_ELBI_SLV_DBI_ENABLE;

	kirin_elb_writel(pcie, val, SOC_PCIECTRL_CTRL1_ADDR);
}

static int kirin_pcie_establish_link(struct pcie_port *pp)
{
	int count = 0;
	struct kirin_pcie *pcie = to_kirin_pcie(pp);

	PCIE_PR_INFO("++");

	if (kirin_pcie_link_up(pp)) {
		PCIE_PR_ERR("Link already up");
		return 0;
	}

	/* setup root complex */
	dw_pcie_setup_rc(pp);
	PCIE_PR_DEBUG("setup rc Done ");

	/* assert LTSSM enable */
	kirin_elb_writel(pcie, PCIE_LTSSM_ENABLE_BIT,
			  PCIE_APP_LTSSM_ENABLE);

	/* check if the link is up or not */
	while (!kirin_pcie_link_up(pp)) {
		mdelay(1);
		count++;
		if (count == 20) {
			PCIE_PR_ERR("Link Fail, Reg should be 0x11 or 0x41 not [0x%x] ",
				 kirin_elb_readl(pcie, SOC_PCIECTRL_STATE4_ADDR));
			return -EINVAL;
		}
	}

	PCIE_PR_INFO("PCIe Link Success ");
	return 0;
}

/*EP rigist hook fun for link event notification*/
int kirin_pcie_register_event(struct kirin_pcie_register_event *reg)
{
	int ret = 0;
	struct pci_dev *dev;
	struct pcie_port *pp;
	struct kirin_pcie *pcie;

	if (!reg || !reg->user) {
		PCIE_PR_INFO("Event registration or User of event is NULL");
		return -ENODEV;
	}

	dev = (struct pci_dev *)reg->user;
	pp = (struct pcie_port *)(dev->bus->sysdata);
	/*lint -e826 -esym(826,*)*/
	pcie = container_of(pp, struct kirin_pcie, pp);
	/*lint -e826 +esym(826,*)*/

	if (pp) {
		pcie->event_reg = reg;
		PCIE_PR_INFO("Event 0x%x is registered for RC", reg->events);
	} else {
		PCIE_PR_INFO("PCIe: did not find RC for pci endpoint device");
		ret = -ENODEV;
	}
	return ret;
}

int kirin_pcie_deregister_event(struct kirin_pcie_register_event *reg)
{
	int ret = 0;
	struct pci_dev *dev;
	struct pcie_port *pp;
	struct kirin_pcie *pcie;

	if (!reg || !reg->user) {
		PCIE_PR_INFO("Event registration or User of event is NULL");
		return -ENODEV;
	}

	dev = (struct pci_dev *)reg->user;
	pp = (struct pcie_port *)(dev->bus->sysdata);
	/*lint -e826 -esym(826,*)*/
	pcie = container_of(pp, struct kirin_pcie, pp);
	/*lint -e826 +esym(826,*)*/

	if (pp) {
		pcie->event_reg = NULL;
		PCIE_PR_INFO("deregistered ");
	} else {
		PCIE_PR_INFO("No RC for this EP device ");
		ret = -ENODEV;
	}
	return ret;
}

/*print apb and cfg-register of RC*/
static void dump_link_register(struct kirin_pcie *pcie)
{
	struct pcie_port *pp = &pcie->pp;
	int i;
	u32 j;
	u32 val0, val1, val2, val3;

	if (!pcie->is_power_on)
		return;

	PCIE_PR_INFO("####DUMP RC CFG Register ");
	for (i = 0; i < 0x18; i++) {
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0x0, 4, &val0);
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0x4, 4, &val1);
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0x8, 4, &val2);
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0xC, 4, &val3);
		printk(KERN_INFO "0x%-8x: %8x %8x %8x %8x \n", 0x10 * i, val0, val1, val2, val3);
	}
	for (i = 0; i < 6; i++) {
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0x0 + 0x700, 4, &val0);
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0x4 + 0x700, 4, &val1);
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0x8 + 0x700, 4, &val2);
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0xC + 0x700, 4, &val3);
		printk(KERN_INFO "0x%-8x: %8x %8x %8x %8x \n", 0x10 * i + 0x700, val0, val1, val2, val3);
	}
	for (i = 0; i < 0x9; i++) {
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0x0 + 0x8A0, 4, &val0);
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0x4 + 0x8A0, 4, &val1);
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0x8 + 0x8A0, 4, &val2);
		kirin_pcie_rd_own_conf(pp, 0x10 * i + 0xC + 0x8A0, 4, &val3);
		printk(KERN_INFO "0x%-8x: %8x %8x %8x %8x \n", 0x10 * i + 0x8A0, val0, val1, val2, val3);
	}



	PCIE_PR_INFO("####DUMP APB CORE Register : ");
	for (j = 0; j < 0x4; j++) {
		printk(KERN_INFO "0x%-8x: %8x %8x %8x %8x \n",
			0x10 * j,
			kirin_elb_readl(pcie, 0x10 * j + 0x0),
			kirin_elb_readl(pcie, 0x10 * j + 0x4),
			kirin_elb_readl(pcie, 0x10 * j + 0x8),
			kirin_elb_readl(pcie, 0x10 * j + 0xC));
	}
	for (j = 0; j < 0x2; j++) {
		printk(KERN_INFO "0x%-8x: %8x %8x %8x %8x \n",
			0x10 * j + 0x400,
			kirin_elb_readl(pcie, 0x10 * j + 0x0 + 0x400),
			kirin_elb_readl(pcie, 0x10 * j + 0x4 + 0x400),
			kirin_elb_readl(pcie, 0x10 * j + 0x8 + 0x400),
			kirin_elb_readl(pcie, 0x10 * j + 0xC + 0x400));
	}


	PCIE_PR_INFO("####DUMP APB PHY Register : ");
	printk(KERN_INFO "0x%-8x: %8x %8x %8x %8x %8x ",
		0x0,
		kirin_phy_readl(pcie, 0x0),
		kirin_phy_readl(pcie, 0x4),
		kirin_phy_readl(pcie, 0x8),
		kirin_phy_readl(pcie, 0xc),
		kirin_phy_readl(pcie, 0x400));
	printk("\n");

	return;

}


/*notify EP about link-down event*/
static void kirin_pcie_notify_callback(struct kirin_pcie *pcie, enum kirin_pcie_event event)
{
	if ((pcie->event_reg != NULL) && (pcie->event_reg->callback != NULL) &&
			(pcie->event_reg->events & event)) {
		struct kirin_pcie_notify *notify = &pcie->event_reg->notify;
		notify->event = event;
		notify->user = pcie->event_reg->user;
		PCIE_PR_INFO("Callback for the event : %d", event);
		pcie->event_reg->callback(notify);
	} else {
		PCIE_PR_INFO("EP does not register this event : %d", event);
	}
}

static void kirin_handle_work(struct work_struct *work)
{
	/*lint -e826 -esym(826,*)*/
	struct kirin_pcie *pcie = container_of(work, struct kirin_pcie, handle_work);
	/*lint -e826 +esym(826,*)*/

	kirin_pcie_notify_callback(pcie, KIRIN_PCIE_EVENT_LINKDOWN);

	dump_link_register(pcie);

}

#ifdef UNUSED_CODE
static irqreturn_t kirin_pcie_linkdown_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;
	struct kirin_pcie *pcie = to_kirin_pcie(pp);

	PCIE_PR_ERR("linkdown irq[%d] triggled", irq);

	schedule_work(&pcie->handle_work);

	return IRQ_HANDLED;
}
#endif /* UNUSED_CODE */

static irqreturn_t kirin_pcie_msi_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	PCIE_PR_ERR("msi irq[%d] triggled", irq);
	return dw_handle_msi_irq(pp);
}

static void kirin_pcie_msi_init(struct pcie_port *pp)
{
	dw_pcie_msi_init(pp);
}

static void kirin_pcie_enable_interrupts(struct pcie_port *pp)
{
	if (IS_ENABLED(CONFIG_PCI_MSI))
		kirin_pcie_msi_init(pp);
}

void kirin_pcie_readl_rc(struct pcie_port *pp,
					void __iomem *dbi_base, u32 *val)
{
	kirin_pcie_sideband_dbi_r_mode(pp, true);
	*val = readl(dbi_base);
	kirin_pcie_sideband_dbi_r_mode(pp, false);
}

void kirin_pcie_writel_rc(struct pcie_port *pp,
					u32 val, void __iomem *dbi_base)
{
	kirin_pcie_sideband_dbi_w_mode(pp, true);
	writel(val, dbi_base);
	kirin_pcie_sideband_dbi_w_mode(pp, false);
}

int kirin_pcie_rd_own_conf(struct pcie_port *pp, int where, int size,
				u32 *val)
{
	kirin_pcie_sideband_dbi_r_mode(pp, true);

	*val = readl(pp->dbi_base + (where & ~0x3));
	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;
	else if (size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;


	kirin_pcie_sideband_dbi_r_mode(pp, false);

	return PCIBIOS_SUCCESSFUL;
}

int kirin_pcie_wr_own_conf(struct pcie_port *pp, int where, int size,
				u32 val)
{
	int ret;

	kirin_pcie_sideband_dbi_w_mode(pp, true);

	if (size == 4)
		writel(val, pp->dbi_base + (where & ~0x3));
	else if (size == 2)
		writew(val, pp->dbi_base + (where & ~0x3) + (where & 2));
	else if (size == 1)
		writeb(val, pp->dbi_base + (where & ~0x3) + (where & 3));
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	kirin_pcie_sideband_dbi_w_mode(pp, false);

	return ret;
}

static int kirin_pcie_link_up(struct pcie_port *pp)
{
	struct kirin_pcie *pcie = to_kirin_pcie(pp);
	u32 val = kirin_elb_readl(pcie, PCIE_ELBI_RDLH_LINKUP);

	if ((val & PCIE_LINKUP_ENABLE) == PCIE_LINKUP_ENABLE)
		return 1;

	return 0;
}

static void kirin_pcie_host_init(struct pcie_port *pp)
{
	if (kirin_pcie_establish_link(pp))
		return;

	kirin_pcie_enable_interrupts(pp);

	return;
}

static struct pcie_host_ops kirin_pcie_host_ops = {
	.readl_rc = kirin_pcie_readl_rc,
	.writel_rc = kirin_pcie_writel_rc,
	.rd_own_conf = kirin_pcie_rd_own_conf,
	.wr_own_conf = kirin_pcie_wr_own_conf,
	.link_up = kirin_pcie_link_up,
	.host_init = kirin_pcie_host_init,
};

static int __init kirin_add_pcie_port(struct pcie_port *pp,
					   struct platform_device *pdev)
{
	int ret;
	int index;
	struct kirin_pcie *pcie = to_kirin_pcie(pp);

	PCIE_PR_INFO("++");
	for (index = 0; index < MAX_IRQ_NUM; index++) {
		pcie->irq[index].num = platform_get_irq(pdev, index);
		if (!pcie->irq[index].num) {
			PCIE_PR_ERR("failed to get [%s] irq ,num = [%d]", pcie->irq[index].name,
				pcie->irq[index].num);
			return -ENODEV;
		}
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = pcie->irq[1].num;
		ret = devm_request_irq(&pdev->dev, pp->msi_irq,
					kirin_pcie_msi_irq_handler,
					IRQF_SHARED | IRQF_TRIGGER_RISING, pcie->irq[1].name, pp);
		if (ret) {
			PCIE_PR_ERR("failed to request msi irq");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &kirin_pcie_host_ops;

	PCIE_PR_INFO("Add pcie port sucessed ");
	return 0;
}

static int kirin_pcie_probe(struct platform_device *pdev)
{
	struct kirin_pcie *pcie;
	struct pcie_port *pp;
	int ret;
	u32 rc_id;

	PCIE_PR_INFO("++");

	if (pdev->dev.of_node == NULL) {
		PCIE_PR_ERR("of node is NULL ");
		return -EINVAL;
	}

	if (of_property_read_u32(pdev->dev.of_node, "rc-id", &rc_id)) {
		dev_err(&pdev->dev, "Failed to get rc id form dts\n");
		return -EINVAL;
	}
	if (rc_id >= MAX_RC_NUM) {
		PCIE_PR_ERR("There is no rc_id = %d", rc_id);
		return -EINVAL;
	}
	PCIE_PR_INFO("PCIe No.%d probe", rc_id);
	pcie = &g_kirin_pcie[rc_id];
	pcie->rc_id = rc_id;
	pcie->rc_dev = NULL;
	pcie->ep_dev = NULL;
	pp = &pcie->pp;

	pp->dev = &(pdev->dev);

	ret = kirin_pcie_get_boardtype(pcie, pdev);
	if (ret != 0)
		return -ENODEV;

	ret = kirin_pcie_get_clk(pcie, pdev);
	if (ret != 0)
		return -ENODEV;

	ret = kirin_pcie_get_resource(pp, pdev);
	if (ret != 0)
		return -ENODEV;

	ret = kirin_pcie_get_pinctrl(pcie, pdev);
	if (ret != 0)
		return -ENODEV;

	PCIE_PR_INFO("###RC Mode");
	pcie->is_enumerated = 0;
	pcie->is_power_on = 0;
	pcie->usr_suspend = 1;
	ret = kirin_add_pcie_port(pp, pdev);
	if (ret < 0) {
		PCIE_PR_ERR("Failed to assign resource, ret=[%d]", ret);
		return ret;
	}

	platform_set_drvdata(pdev, pcie);

	INIT_WORK(&pcie->handle_work, kirin_handle_work);

	PCIE_PR_INFO("--");
	return 0;

}

void kirin_pcie_shutdown(struct platform_device *pdev)
{
	struct kirin_pcie *pcie;

	PCIE_PR_INFO("++");

	pcie = dev_get_drvdata(&(pdev->dev));
	if (pcie == NULL) {
		PCIE_PR_ERR("Can't get drvdata");
		return;
	}

	if (pcie->is_enumerated) {
		pcie->rc_dev->msi_enabled = 0;
	}

	if (pcie->is_power_on) {
		if (kirin_pcie_power_on((&pcie->pp), DISABLE)) {
			PCIE_PR_ERR("Failed to Power off");
			return;
		}
	}

	PCIE_PR_INFO("--");
}

static void kirin_pcie_shutdown_prepare(struct pci_dev *dev)
{
	int ret;
	u32 pm;
	u32 value;
	int index = 0;
	struct pcie_port *pp;
	struct kirin_pcie *pcie;

	PCIE_PR_INFO("++");

	pp = dev->sysdata;
	pcie = to_kirin_pcie(pp);

	/*Enable PME*/
	pm = pci_find_capability(dev, PCI_CAP_ID_PM);
	if (!pm) {
		PCIE_PR_ERR("Can't find PCI_CAP_ID_PM");
		return;
	}
	kirin_pcie_rd_own_conf(pp, pm + PCI_PM_CTRL, 4, &value);
	value |= 0x100;
	kirin_pcie_wr_own_conf(pp, pm + PCI_PM_CTRL, 4, value);

	/*Broadcast PME_turn_off MSG*/
	ret = kirin_elb_readl(pcie, SOC_PCIECTRL_CTRL7_ADDR);
	ret |= PME_TURN_OFF_BIT;
	kirin_elb_writel(pcie, ret, SOC_PCIECTRL_CTRL7_ADDR);

	do {
		ret = kirin_elb_readl(pcie, SOC_PCIECTRL_STATE1_ADDR);
		ret = ret & PME_ACK_BIT;
		if (index >= 10) {
			PCIE_PR_ERR("Failed to get PME_TO_ACK");
			break;
		}
		index++;
		udelay((unsigned long)1);
	} while (ret != PME_ACK_BIT);

	/*Entry L23*/
	ret = kirin_elb_readl(pcie, SOC_PCIECTRL_CTRL7_ADDR);
	ret |= ENTRY_L23_BIT;
	kirin_elb_writel(pcie, ret, SOC_PCIECTRL_CTRL7_ADDR);

	/*phy rst from sys to pipe */
	ret = kirin_phy_readl(pcie, SOC_PCIEPHY_CTRL1_ADDR);
	ret |= 0x1 << 17;
	kirin_phy_writel(pcie, ret, SOC_PCIEPHY_CTRL1_ADDR);

	ret = kirin_elb_readl(pcie, SOC_PCIECTRL_STATE4_ADDR);
	PCIE_PR_INFO("L23 register value: 0x%x ", ret);

	PCIE_PR_INFO("--");
}

static int kirin_pcie_usr_suspend(u32 rc_idx)
{
	int ret;
	struct pcie_port *pp;
	struct pci_dev *rc_dev;
	struct kirin_pcie *pcie = &g_kirin_pcie[rc_idx];

	PCIE_PR_INFO("++");
	PCIE_PR_DEBUG("rc_id = [%d] ", rc_idx);

	pp = &pcie->pp;
	rc_dev = pcie->rc_dev;
	if (rc_dev) {
		kirin_pcie_shutdown_prepare(rc_dev);
		rc_dev->msi_enabled = 0;
	}

	/*关闭所有时钟*/
	ret = kirin_pcie_power_on(pp, DISABLE);
	if (ret) {
		PCIE_PR_ERR("PCIe No.%d failed to Power OFF ", rc_idx);
		return -EINVAL;
	}
	pcie->usr_suspend = 1;

	PCIE_PR_INFO("--");
	return 0;
}

static int kirin_pcie_usr_resume(u32 rc_idx)
{
	int ret;
	struct pcie_port *pp;
	struct pci_dev *rc_dev;
	struct kirin_pcie *pcie = &g_kirin_pcie[rc_idx];

	PCIE_PR_INFO("++");
	PCIE_PR_DEBUG("rc_id = [%ud] ", rc_idx);

	pp = &pcie->pp;
	rc_dev = pcie->rc_dev;
	ret = kirin_pcie_power_on(pp, ENABLE);
	if (ret) {
		PCIE_PR_ERR("PCIe No.%d failed to Power ON ", rc_idx);
		return -EINVAL;
	}
	ret = kirin_pcie_establish_link(&pcie->pp);
	if (ret)
		return -EINVAL;
	if (rc_dev) {
		pci_load_saved_state(rc_dev, pcie->rc_saved_state);
		pci_restore_state(rc_dev);
		rc_dev->msi_enabled = 1;
	}
	pcie->usr_suspend = 0;

	PCIE_PR_INFO("--");

	return 0;
}

/*
* EP Power ON/OFF callback Function:
* param: rc_idx---which rc the EP link with
*        resume_flag---1:PowerOn, 0: PowerOFF
*/
int kirin_pcie_pm_control(int resume_flag, u32 rc_idx)
{
	PCIE_PR_DEBUG("RC = [%u] ", rc_idx);

	if (rc_idx >= MAX_RC_NUM) {
		PCIE_PR_ERR("There is no rc_id = %d", rc_idx);
		return -EINVAL;
	}
	if (resume_flag) {
		PCIE_PR_INFO("EP Notify to PowerON");
		return kirin_pcie_usr_resume(rc_idx);
	} else {
		PCIE_PR_INFO("EP Notify to Poweroff");
		return kirin_pcie_usr_suspend(rc_idx);
	}
}
EXPORT_SYMBOL_GPL(kirin_pcie_pm_control);

/*
* Enumerate Function:
* param: rc_idx---which rc the EP link with
*/
int kirin_pcie_enumerate(u32 rc_idx)
{
	int ret;
	u32 val;
	u32 dev_id;
	u32 vendor_id;
	struct pcie_port *pp;
	struct pci_bus *bus1;
	struct pci_dev *dev;
	struct kirin_pcie *pcie;

	PCIE_PR_INFO("++");
	PCIE_PR_DEBUG("RC[%u] begin to Enumerate ", rc_idx);

	if (rc_idx >= MAX_RC_NUM) {
		PCIE_PR_ERR("There is no rc_id = %d", rc_idx);
		return -EINVAL;
	}
	pcie = &g_kirin_pcie[rc_idx];
	pp = &pcie->pp;

	if (pcie->is_enumerated) {
		PCIE_PR_ERR("Enumeration was done Successed Before");
		return 0;
	}

	/*clk on*/
	ret = kirin_pcie_power_on(pp, ENABLE);
	if (ret) {
		PCIE_PR_ERR("Failed to Power RC");
		return ret;
	}
	kirin_pcie_readl_rc(pp, pp->dbi_base, &val);
	val += rc_idx;
	kirin_pcie_writel_rc(pp, val, pp->dbi_base);

	ret = dw_pcie_host_init(pp);
	if (ret) {
		PCIE_PR_ERR("failed to initialize host");
		return ret;
	}
	kirin_pcie_rd_own_conf(pp, PCI_VENDOR_ID, 2, &vendor_id);
	kirin_pcie_rd_own_conf(pp, PCI_DEVICE_ID, 2, &dev_id);
	pcie->rc_dev = pci_get_device(vendor_id, dev_id, pcie->rc_dev);
	if (!pcie->rc_dev) {
		PCIE_PR_ERR("Failed to get RC Device ");
		return -1;
	}
	ret = pci_save_state(pcie->rc_dev);
	pcie->rc_saved_state = pci_store_saved_state(pcie->rc_dev);
	if (ret) {
		PCIE_PR_ERR("Failed to save state of RC.");
		return ret;
	}
	bus1 = pcie->rc_dev->subordinate;
	if (bus1) {
		list_for_each_entry(dev, &bus1->devices, bus_list) {
			if (pci_is_pcie(dev)) {
				pcie->ep_dev = dev;
				pcie->is_enumerated = 1;
				pcie->ep_devid = dev->device;
				pcie->ep_venid = dev->vendor;
				PCIE_PR_INFO("ep vendorid = 0x%x, deviceid = 0x%x", pcie->ep_venid, pcie->ep_devid);
			}
		}
	} else {
		PCIE_PR_ERR("bus1 is NULL");
		pcie->ep_dev = NULL;
	}
	if (!pcie->ep_dev) {
		PCIE_PR_ERR("there is no ep dev");
		return -1;
	}
	pcie->usr_suspend = 0;

	PCIE_PR_INFO("--");

	return 0;
}
EXPORT_SYMBOL(kirin_pcie_enumerate);

static const struct of_device_id kirin_pcie_match[] = {
	{ .compatible = "hisilicon,kirin-pcie" },
	{},
};
MODULE_DEVICE_TABLE(of, kirin_pcie_match);

struct platform_driver kirin_pcie_driver = {
	.probe			= kirin_pcie_probe,
	.shutdown		= kirin_pcie_shutdown,
	.driver			= {
		.name			= "Kirin-pcie",
		.owner			= THIS_MODULE,
		.of_match_table = kirin_pcie_match,
	},
};

module_platform_driver(kirin_pcie_driver);

MODULE_AUTHOR("Xiaowei Song<songxiaowei@huawei.com>");
MODULE_DESCRIPTION("Hisilicon Kirin pcie driver");
MODULE_LICENSE("GPL");
