/*
 * PCIe host controller driver for Kirin 960 SoCs
 *
 * Copyright (C) 2015 Huawei Electronics Co., Ltd.
 *		http://www.huawei.com
 *
 * Author: Xiaowei Song <songxiaowei@huawei.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PCIE_KIRIN_H
#define _PCIE_KIRIN_H

#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <asm/compiler.h>
#include <linux/compiler.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/irq.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/pci_regs.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/wait.h>
#include <linux/freezer.h>
#include <linux/pinctrl/consumer.h>

#include "pcie-designware.h"

#define to_kirin_pcie(x)	container_of(x, struct kirin_pcie, pp)

#define MAX_RC_NUM 1
#define MAX_IRQ_NUM 5

#define REF_CLK_FREQ 100000000

/* PCIe ELBI registers */
#define SOC_PCIECTRL_CTRL0_ADDR 0x000
#define SOC_PCIECTRL_CTRL1_ADDR 0x004
#define SOC_PCIECTRL_CTRL7_ADDR 0x01c
#define SOC_PCIECTRL_CTRL12_ADDR 0x030
#define SOC_PCIECTRL_STATE1_ADDR 0x404
#define SOC_PCIEPHY_CTRL0_ADDR 0x000
#define SOC_PCIEPHY_CTRL1_ADDR 0x004
#define SOC_PCIEPHY_CTRL2_ADDR 0x008
#define SOC_PCIEPHY_CTRL3_ADDR 0x00c
#define SOC_PCIEPHY_STATE0_ADDR 0x400
#define SOC_PCIECTRL_STATE4_ADDR 0x410

#define PCIE_APB_CLK_REQ	(0x1 << 23)
#define PERST_FUN_SEC 0x2006
#define PERST_ASSERT_EN 0x1

#define ENTRY_L23_BIT (0x1 << 2)
#define PCIE_ELBI_SLV_DBI_ENABLE	(0x1 << 21)
#define PME_TURN_OFF_BIT (0x1 << 8)
#define PME_ACK_BIT (0x1<<16)

#define ENABLE 1
#define DISABLE 0

/* SYSCTRL register */
#define MTCMOS_CTRL_BIT    0x10
#define SCTRL_SCPWREN	0x60
#define SCTRL_SCPWRDIS	0x64
#define SCTRL_SCPERCLKEN2	0x190
#define SCTRL_SCPERCLKDIS2	0x194
#define SCTRL_SCPERCLKEN3	0x1A4
#define HW_AUTO_CF_BIT		((0x1 << 20) | (0x1 << 19) | (0x1 << 14))
#define IO_HARD_CTRL_DEBOUNCE_BYPASS (0x1 << 28)
#define IO_OE_EN_HARD_BYPASS (0x1 << 29)

#define PCIE_APP_LTSSM_ENABLE		0x01c
#define PCIE_ELBI_RDLH_LINKUP		0x400
#define PCIE_LINKUP_ENABLE		(0x8020)
#define PCIE_LTSSM_ENABLE_BIT	  (0x1 << 11)

struct kirin_pcie_irq_info {
	char *name;
	int num;
};

struct kirin_pcie {
	void __iomem		*apb_base;
	void __iomem		*phy_base;
	struct regmap *crgctrl;
	struct regmap *sysctrl;
	struct clk			*apb_sys_clk;
	struct clk			*apb_phy_clk;
	struct clk			*phy_ref_clk;
	struct clk			*pcie_aclk;
	struct clk			*pcie_aux_clk;
	struct pinctrl      *pin;
	int                 gpio_id_reset;
	u32					board_type;
	int                 ep_flag;
	struct  pcie_port	pp;
	struct  pci_dev		*rc_dev;
	struct  pci_dev		*ep_dev;
	unsigned int		ep_devid;
	unsigned int		ep_venid;
	unsigned int		ep_ltr_latency;
	unsigned int		ep_l1ss_ctrl2;
	unsigned int		l1ss_ctrl1;
	unsigned int		aspm_state;
	unsigned int		usr_suspend;
	unsigned int		is_power_on;
	unsigned int		is_enumerated;
	struct mutex		pm_lock;
	struct pci_saved_state *rc_saved_state;
	struct work_struct	handle_work;
	struct kirin_pcie_register_event *event_reg;
	u32 pcie_eye_param_ctrl2;	/* this param will be set to pcie phy ctrl2 */
	u32 pcie_eye_param_ctrl3;/* this param will be set to pcie phy ctrl3 */
	u32 isoen_offset;
	u32 isoen_value;
	u32 isodis_offset;
	u32 isodis_value;
	u32 phy_assert_offset;
	u32 phy_assert_value;
	u32 phy_deassert_offset;
	u32 phy_deassert_value;
	u32 core_assert_offset;
	u32 core_assert_value;
	u32 core_deassert_offset;
	u32 core_deassert_value;
	struct kirin_pcie_irq_info irq[5];
	u32 rc_id;
};

enum link_aspm_state {
	ASPM_CLOSE = 0,		/*disable aspm L0s L1*/
	ASPM_L0S = 1,		/* enable l0s  */
	ASPM_L1 = 2,		/* enable l1 */
	ASPM_L0S_L1 = 3,	/* enable l0s & l1*/
};

enum link_speed {
	GEN1 = 0,
	GEN2 = 1,
	GEN3 = 2,
};

enum l1ss_ctrl_state {
	L1SS_CLOSE = 0x0,		/*disable l1ss*/
	L1SS_PM_1_2 = 0x1,		/* pci-pm L1.2*/
	L1SS_PM_1_1 = 0x2,		/* pci-pm L1.1*/
	L1SS_PM_ALL = 0x3,		/* pci-pm L1.2 & L1.1*/
	L1SS_ASPM_1_2 = 0x4,	/* aspm L1.2 */
	L1SS_ASPM_1_1 = 0x8,	/* aspm L1.1 */
	L1SS_ASPM_ALL = 0xC,	/* aspm L1.2 & L1.1 */
	L1SS_PM_ASPM_ALL = 0xF,	/* aspm l1ss & pci-pm l1ss*/
};

enum kirin_pcie_event {
	KIRIN_PCIE_EVENT_MIN_INVALID = 0x0,		/*min invalid value*/
	KIRIN_PCIE_EVENT_LINKUP = 0x1,		/* linkup event  */
	KIRIN_PCIE_EVENT_LINKDOWN = 0x2,		/* linkdown event */
	KIRIN_PCIE_EVENT_WAKE = 0x4,	/* wake event*/
	KIRIN_PCIE_EVENT_L1SS = 0x8,	/* l1ss event*/
	KIRIN_PCIE_EVENT_MAX_INVALID = 0xF,	/* max invalid value*/
};

enum kirin_pcie_trigger {
	KIRIN_PCIE_TRIGGER_CALLBACK,
	KIRIN_PCIE_TRIGGER_COMPLETION,
};

struct kirin_pcie_notify {
	enum kirin_pcie_event event;
	void *user;
	void *data;
	u32 options;
};

struct kirin_pcie_register_event {
	u32 events;
	void *user;
	enum kirin_pcie_trigger mode;
	void (*callback)(struct kirin_pcie_notify *notify);
	struct kirin_pcie_notify notify;
	struct completion *completion;
	u32 options;
};

#define PCIE_PR_ERR(fmt, args ...)	do {	printk(KERN_ERR "%s(%d):" fmt "\n", __FUNCTION__, __LINE__, ##args); } while (0)
#define PCIE_PR_INFO(fmt, args ...)	do {	printk(KERN_INFO "%s(%d):" fmt "\n", __FUNCTION__, __LINE__, ##args); } while (0)
#define PCIE_PR_DEBUG(fmt, args ...)	do {	printk(KERN_DEBUG "%s(%d):" fmt "\n", __FUNCTION__, __LINE__, ##args); } while (0)

extern struct kirin_pcie g_kirin_pcie[MAX_RC_NUM];

void kirin_elb_writel(struct kirin_pcie *pcie, u32 val, u32 reg);

u32 kirin_elb_readl(struct kirin_pcie *pcie, u32 reg);

/*Registers in PCIePHY*/
void kirin_phy_writel(struct kirin_pcie *pcie, u32 val, u32 reg);

u32 kirin_phy_readl(struct kirin_pcie *pcie, u32 reg);

void kirin_pcie_readl_rc(struct pcie_port *pp,
					void __iomem *dbi_base, u32 *val);
void kirin_pcie_writel_rc(struct pcie_port *pp,
					u32 val, void __iomem *dbi_base);
int kirin_pcie_rd_own_conf(struct pcie_port *pp, int where, int size,
				u32 *val);
int kirin_pcie_wr_own_conf(struct pcie_port *pp, int where, int size,
				u32 val);
int kirin_pcie_power_on(struct pcie_port *pp, int on_flag);
void enable_req_clk(struct kirin_pcie *pcie, u32 enable_flag);
int kirin_pcie_register_event(struct kirin_pcie_register_event *reg);
int kirin_pcie_deregister_event(struct kirin_pcie_register_event *reg);
#endif

