/*
 * Synopsys Designware PCIe host controller driver
 *
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Jingoo Han <jg1.han@samsung.com>
 *
 * Customizations for the NXP S32V PCIE driver
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/hardirq.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include "pcie-designware.h"

/* Synopsis specific PCIE configuration registers */
#define PCIE_PORT_LINK_CONTROL		0x710
#define PORT_LINK_MODE_MASK			(0x3f << 16)
#define PORT_LINK_MODE_1_LANES		(0x1 << 16)
#define PORT_LINK_MODE_2_LANES		(0x3 << 16)
#define PORT_LINK_MODE_4_LANES		(0x7 << 16)

#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C
#define PORT_LOGIC_SPEED_CHANGE			(0x1 << 17)
#define PORT_LOGIC_LINK_WIDTH_MASK		(0x1ff << 8)
#define PORT_LOGIC_LINK_WIDTH_1_LANES	(0x1 << 8)
#define PORT_LOGIC_LINK_WIDTH_2_LANES	(0x2 << 8)
#define PORT_LOGIC_LINK_WIDTH_4_LANES	(0x4 << 8)

#define PCIE_MSI_ADDR_LO			0x820
#define PCIE_MSI_ADDR_HI			0x824
#define PCIE_MSI_INTR0_ENABLE		0x828
#define PCIE_MSI_INTR0_MASK			0x82C
#define PCIE_MSI_INTR0_STATUS		0x830

#define PCIE_ATU_VIEWPORT			0x900
#define PCIE_ATU_REGION_INBOUND		(0x1 << 31)
#define PCIE_ATU_REGION_OUTBOUND	(0x0 << 31)
#define PCIE_ATU_REGION_INDEX1		(0x1 << 0)
#define PCIE_ATU_REGION_INDEX0		(0x0 << 0)
#define PCIE_ATU_CR1				0x904
#define PCIE_ATU_TYPE_MEM			(0x0 << 0)
#define PCIE_ATU_TYPE_IO			(0x2 << 0)
#define PCIE_ATU_TYPE_CFG0			(0x4 << 0)
#define PCIE_ATU_TYPE_CFG1			(0x5 << 0)
#define PCIE_ATU_CR2				0x908
#define PCIE_ATU_ENABLE				(0x1 << 31)
#define PCIE_ATU_BAR_MODE_ENABLE	(0x1 << 30)
#define PCIE_ATU_LOWER_BASE			0x90C
#define PCIE_ATU_UPPER_BASE			0x910
#define PCIE_ATU_LIMIT				0x914
#define PCIE_ATU_LOWER_TARGET		0x918
#define PCIE_ATU_BUS(x)				(((x) & 0xff) << 24)
#define PCIE_ATU_DEV(x)				(((x) & 0x1f) << 19)
#define PCIE_ATU_FUNC(x)			(((x) & 0x7) << 16)
#define PCIE_ATU_UPPER_TARGET		0x91C

#ifdef CONFIG_PCI_DW_DMA
#define PCIE_DMA_WRITE_ENGINE_EN				0x97c
#define PCIE_DMA_WRITE_DOORBELL					0x980
#define PCIE_DMA_READ_ENGINE_EN					0x99C
#define PCIE_DMA_READ_DOORBELL					0x9A0
#define PCIE_DMA_WRITE_INT_STATUS				0x9BC
#define PCIE_DMA_WRITE_INT_MASK					0x9C4
#define PCIE_DMA_WRITE_INT_CLEAR				0x9C8
#define PCIE_DMA_WRITE_ERR_STATUS				0x9CC
#define PCIE_DMA_WRITE_DONE_IMWR_LOW			0x9D0
#define PCIE_DMA_WRITE_DONE_IMWR_HIGH			0x9D4
#define PCIE_DMA_WRITE_ABORT_IMWR_LOW			0x9D8
#define PCIE_DMA_WRITE_ABORT_IMWR_HIGH			0x9DC
#define PCIE_DMA_WRITE_CH01_IMWR_DATA			0x9E0
#define PCIE_DMA_WRITE_CH23_IMWR_DATA			0x9E4
#define PCIE_DMA_WRITE_CH45_IMWR_DATA			0x9E8
#define PCIE_DMA_WRITE_CH67_IMWR_DATA			0x9EC
#define PCIE_DMA_WRITE_LINKED_LIST_ERR_EN		0xA00

#define PCIE_DMA_READ_INT_STATUS				0xA10
#define PCIE_DMA_READ_INT_MASK					0xA18
#define PCIE_DMA_READ_INT_CLEAR					0xA1C
#define PCIE_DMA_READ_ERR_STATUS_LOW			0xA24
#define PCIE_DMA_READ_ERR_STATUS_HIGH			0xA28
#define PCIE_DMA_READ_LINKED_LIST_ERR_EN		0xA34
#define PCIE_DMA_READ_DONE_IMWR_LOW				0xA3C
#define PCIE_DMA_READ_DONE_IMWR_HIGH			0xA40
#define PCIE_DMA_READ_ABORT_IMWR_LOW			0xA44
#define PCIE_DMA_READ_ABORT_IMWR_HIGH			0xA48
#define PCIE_DMA_READ_CH01_IMWR_DATA			0xA4C
#define PCIE_DMA_READ_CH23_IMWR_DATA			0xA50
#define PCIE_DMA_READ_CH45_IMWR_DATA			0xA54
#define PCIE_DMA_READ_CH67_IMWR_DATA			0xA58

#define PCIE_DMA_VIEWPORT_SEL					0xA6C
#define PCIE_DMA_CH_CONTROL1					0xA70
#define PCIE_DMA_CH_CONTROL2					0xA74
#define PCIE_DMA_TRANSFER_SIZE					0xA78
#define PCIE_DMA_SAR_LOW						0xA7C
#define PCIE_DMA_SAR_HIGH						0xA80
#define PCIE_DMA_DAR_LOW						0xA84
#define PCIE_DMA_DAR_HIGH						0xA88
#define PCIE_DMA_LLP_LOW						0xA8C
#define PCIE_DMA_LLP_HIGH						0xA90

#endif /* CONFIG_PCI_DW_DMA */
static struct pci_ops dw_pcie_ops;

static unsigned long global_io_offset;

int dw_pcie_cfg_read(void __iomem *addr, int where, int size, u32 *val)
{
	*val = readl(addr);

	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;
	else if (size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}

int dw_pcie_cfg_write(void __iomem *addr, int where, int size, u32 val)
{
	if (size == 4)
		writel(val, addr);
	else if (size == 2)
		writew(val, addr + (where & 2));
	else if (size == 1)
		writeb(val, addr + (where & 3));
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}

static inline void dw_pcie_readl_rc(struct pcie_port *pp, u32 reg, u32 *val)
{
	if (pp->ops->readl_rc)
		pp->ops->readl_rc(pp, pp->dbi_base + reg, val);
	else
		*val = readl(pp->dbi_base + reg);
}

static inline void dw_pcie_writel_rc(struct pcie_port *pp, u32 val, u32 reg)
{
	if (pp->ops->writel_rc)
		pp->ops->writel_rc(pp, val, pp->dbi_base + reg);
	else
		writel(val, pp->dbi_base + reg);
}

static int dw_pcie_rd_own_conf(struct pcie_port *pp, int where, int size,
			       u32 *val)
{
	int ret;

	if (pp->ops->rd_own_conf)
		ret = pp->ops->rd_own_conf(pp, where, size, val);
	else
		ret = dw_pcie_cfg_read(pp->dbi_base + (where & ~0x3), where,
				size, val);

	return ret;
}

static int dw_pcie_wr_own_conf(struct pcie_port *pp, int where, int size,
			       u32 val)
{
	int ret;

	if (pp->ops->wr_own_conf)
		ret = pp->ops->wr_own_conf(pp, where, size, val);
	else
		ret = dw_pcie_cfg_write(pp->dbi_base + (where & ~0x3), where,
				size, val);

	return ret;
}

#ifdef CONFIG_PCI_DW_DMA
int dw_pcie_dma_write_en(struct pcie_port *pp)
{
	dw_pcie_writel_rc(pp, 0x1, PCIE_DMA_WRITE_ENGINE_EN);
	return 0;
}
int dw_pcie_dma_write_soft_reset(struct pcie_port *pp)
{
	dw_pcie_writel_rc(pp, 0x0, PCIE_DMA_WRITE_ENGINE_EN);
	while (readl(pp->dbi_base + PCIE_DMA_WRITE_ENGINE_EN) == 1)
		;
	pp->wr_ch.status = DMA_CH_STOPPED;
	dw_pcie_writel_rc(pp, 0x1, PCIE_DMA_WRITE_ENGINE_EN);
	return 0;
}
int dw_pcie_dma_read_en(struct pcie_port *pp)
{
	dw_pcie_writel_rc(pp, 0x1, PCIE_DMA_READ_ENGINE_EN);
	return 0;
}
int dw_pcie_dma_read_soft_reset(struct pcie_port *pp)
{
	dw_pcie_writel_rc(pp, 0x0, PCIE_DMA_READ_ENGINE_EN);
	while (readl(pp->dbi_base + PCIE_DMA_READ_ENGINE_EN) == 1)
		;
	pp->rd_ch.status = DMA_CH_STOPPED;
	dw_pcie_writel_rc(pp, 0x1, PCIE_DMA_READ_ENGINE_EN);
	return 0;
}
void dw_pcie_dma_set_wr_remote_done_int(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_WRITE_DONE_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_WRITE_DONE_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_set_wr_remote_abort_int(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_WRITE_ABORT_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_WRITE_ABORT_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_set_rd_remote_done_int(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_READ_DONE_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_READ_DONE_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_set_rd_remote_abort_int(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, lower_32_bits(val),
		PCIE_DMA_READ_ABORT_IMWR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val),
		PCIE_DMA_READ_ABORT_IMWR_HIGH);
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x10,
		PCIE_DMA_CH_CONTROL1);
}
void dw_pcie_dma_en_local_int(struct pcie_port *pp)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_CH_CONTROL1) | 0x8,
		PCIE_DMA_CH_CONTROL1);
}
/* Interrupts mask and clear functions */
int dw_pcie_dma_clear_wr_done_int_mask(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) & (~val),
		PCIE_DMA_WRITE_INT_MASK);
	return 0;
}
int dw_pcie_dma_clear_wr_abort_int_mask(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) & ((~val)<<16),
		PCIE_DMA_WRITE_INT_MASK);
	return 0;
}
int dw_pcie_dma_clear_rd_done_int_mask(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_READ_INT_MASK) & (~val),
		PCIE_DMA_READ_INT_MASK);
	return 0;
}
int dw_pcie_dma_clear_rd_abort_int_mask(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_READ_INT_MASK) & ((~val)<<16),
		PCIE_DMA_READ_INT_MASK);
	return 0;
}

int dw_pcie_dma_set_wr_done_int_mask(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val, PCIE_DMA_WRITE_INT_MASK);
	return 0;
}
int dw_pcie_dma_set_wr_abort_int_mask(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val<<16, PCIE_DMA_WRITE_INT_MASK);
	return 0;
}

int dw_pcie_dma_set_rd_done_int_mask(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val, PCIE_DMA_READ_INT_MASK);
	return 0;
}
int dw_pcie_dma_set_rd_abort_int_mask(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_WRITE_INT_MASK) | val<<16, PCIE_DMA_READ_INT_MASK);
	return 0;
}

int dw_pcie_dma_clear_wr_done_int(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_WRITE_INT_CLEAR) | val, PCIE_DMA_WRITE_INT_CLEAR);
	return 0;
}
int dw_pcie_dma_clear_wr_abort_int(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_WRITE_INT_CLEAR) | val<<16, PCIE_DMA_WRITE_INT_CLEAR);
	return 0;
}
int dw_pcie_dma_clear_rd_done_int(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_READ_INT_CLEAR) | val, PCIE_DMA_READ_INT_CLEAR);
	return 0;
}
int dw_pcie_dma_clear_rd_abort_int(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, readl(pp->dbi_base +
		PCIE_DMA_READ_INT_CLEAR) | val<<16, PCIE_DMA_READ_INT_CLEAR);
	return 0;
}
void dw_pcie_dma_set_sar(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, lower_32_bits(val), PCIE_DMA_SAR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val), PCIE_DMA_SAR_HIGH);
}
void dw_pcie_dma_set_dar(struct pcie_port *pp, u64 val)
{
	dw_pcie_writel_rc(pp, lower_32_bits(val), PCIE_DMA_DAR_LOW);
	dw_pcie_writel_rc(pp, upper_32_bits(val), PCIE_DMA_DAR_HIGH);
}
void dw_pcie_dma_set_transfer_size(struct pcie_port *pp, u32 val)
{
	dw_pcie_writel_rc(pp, val, PCIE_DMA_TRANSFER_SIZE);
}

void dw_pcie_dma_set_rd_viewport(struct pcie_port *pp, u8 ch_nr)
{
	dw_pcie_writel_rc(pp, (1<<31) | ch_nr, PCIE_DMA_VIEWPORT_SEL);
}
void dw_pcie_dma_set_viewport(struct pcie_port *pp, u8 ch_nr, u8 direction)
{
	if (direction == DMA_CH_WRITE)
		dw_pcie_writel_rc(pp, (0<<31) | ch_nr, PCIE_DMA_VIEWPORT_SEL);
	else
		dw_pcie_writel_rc(pp, (1<<31) | ch_nr, PCIE_DMA_VIEWPORT_SEL);
}
void dw_pcie_dma_set_wr_viewport(struct pcie_port *pp, u8 ch_nr)
{
	dw_pcie_writel_rc(pp, (0<<31) | ch_nr, PCIE_DMA_VIEWPORT_SEL);
}
void dw_pcie_dma_clear_regs(struct pcie_port *pp)
{
	pp->wr_ch.status = DMA_CH_STOPPED;
	pp->rd_ch.status = DMA_CH_STOPPED;
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_INT_MASK);
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_INT_CLEAR);
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_DONE_IMWR_LOW);
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_DONE_IMWR_HIGH);
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_ABORT_IMWR_LOW);
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_ABORT_IMWR_HIGH);
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_CH01_IMWR_DATA);
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_CH23_IMWR_DATA);
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_CH45_IMWR_DATA);
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_CH67_IMWR_DATA);
	writel(0, pp->dbi_base + PCIE_DMA_READ_INT_MASK);
	writel(0, pp->dbi_base + PCIE_DMA_READ_INT_CLEAR);
	writel(0, pp->dbi_base + PCIE_DMA_READ_DONE_IMWR_LOW);
	writel(0, pp->dbi_base + PCIE_DMA_READ_DONE_IMWR_HIGH);
	writel(0, pp->dbi_base + PCIE_DMA_READ_ABORT_IMWR_LOW);
	writel(0, pp->dbi_base + PCIE_DMA_READ_ABORT_IMWR_HIGH);
	writel(0, pp->dbi_base + PCIE_DMA_READ_CH01_IMWR_DATA);
	writel(0, pp->dbi_base + PCIE_DMA_READ_CH23_IMWR_DATA);
	writel(0, pp->dbi_base + PCIE_DMA_READ_CH45_IMWR_DATA);
	writel(0, pp->dbi_base + PCIE_DMA_READ_CH67_IMWR_DATA);
	writel(0, pp->dbi_base + PCIE_DMA_CH_CONTROL1);
	writel(0, pp->dbi_base + PCIE_DMA_CH_CONTROL2);
	writel(0, pp->dbi_base + PCIE_DMA_TRANSFER_SIZE);
	writel(0, pp->dbi_base + PCIE_DMA_SAR_LOW);
	writel(0, pp->dbi_base + PCIE_DMA_SAR_HIGH);
	writel(0, pp->dbi_base + PCIE_DMA_DAR_LOW);
	writel(0, pp->dbi_base + PCIE_DMA_DAR_HIGH);
	writel(0, pp->dbi_base + PCIE_DMA_LLP_LOW);
	writel(0, pp->dbi_base + PCIE_DMA_LLP_HIGH);

}
void dma_set_list_ptr(struct pcie_port *pp, u8 direction,
	u32 phy_list_addr)
{
	writel(lower_32_bits(phy_list_addr), pp->dbi_base + PCIE_DMA_LLP_LOW);
	writel(upper_32_bits(phy_list_addr), pp->dbi_base + PCIE_DMA_LLP_HIGH);
}
static void dma_set_link_elem(u32 *ptr_list_base,
	u8 arr_sz, u32 phy_list_addr) {

	*(ptr_list_base + (arr_sz * 6) + 0x1) = 0;
	*(ptr_list_base + (arr_sz * 6) + 0x2) = lower_32_bits(phy_list_addr);
	*(ptr_list_base + (arr_sz * 6) + 0x3) = upper_32_bits(phy_list_addr);
	/* LLP | TCB | CB */
	*(ptr_list_base + (arr_sz * 6) + 0) = 0x5;
}

static void dma_set_data_elem(u32 *ptr_list_base, u8 index,
	u64 sar, u64 dar, u32 size, u8 intr)
{
	u8 i = index;
	u8 ctrl_LIE = (intr) ? 0x8 : 0x0;

	*(ptr_list_base + (i * 6) + 0x1) = size;
	*(ptr_list_base + (i * 6) + 0x2) = lower_32_bits(sar);
	*(ptr_list_base + (i * 6) + 0x3) = upper_32_bits(sar);
	*(ptr_list_base + (i * 6) + 0x4) = lower_32_bits(dar);
	*(ptr_list_base + (i * 6) + 0x5) = upper_32_bits(dar);
	*(ptr_list_base + (i * 6) + 0) = ctrl_LIE | 0x1;
}

void dw_start_dma_llw(struct pcie_port *pp, u64 phy_list_addr)
{
	/* Program DMA regs for LL mode */
	writel(0x0, pp->dbi_base + PCIE_DMA_CH_CONTROL1);
	writel(0x1, pp->dbi_base + PCIE_DMA_WRITE_ENGINE_EN);
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_INT_MASK);
	writel(0x10000, pp->dbi_base +
		PCIE_DMA_WRITE_LINKED_LIST_ERR_EN);
	writel(0, pp->dbi_base + PCIE_DMA_VIEWPORT_SEL);
	writel(0x04000300, pp->dbi_base + PCIE_DMA_CH_CONTROL1);

	/* Set pointer to start of first list */
	dma_set_list_ptr(pp, DMA_CH_WRITE, phy_list_addr);
	/* Ring doorbell */
	pp->wr_ch.status = DMA_CH_RUNNING;
	writel(0, pp->dbi_base + PCIE_DMA_WRITE_DOORBELL);
}
EXPORT_SYMBOL(dw_start_dma_llw);

int dw_pcie_dma_start_linked_list(struct pcie_port *pp,
	u32 phy_list_addr,
	u8 direction)
{

	if (direction == DMA_CH_WRITE) {
		if (pp->wr_ch.status != DMA_CH_RUNNING) {
			/* Set ch_status */
			pp->wr_ch.status = DMA_CH_RUNNING;
			/* Set last data elem */
			dma_set_data_elem((u32 *)pp->wr_ch.virt_addr,
			pp->wr_ch.current_list_size - 1,
			pp->wr_ch.current_sar,
			pp->wr_ch.current_dar, pp->wr_ch.current_size, 1);

			/* Program DMA regs for LL mode */
			writel(0x0, pp->dbi_base + PCIE_DMA_CH_CONTROL1);
			pp->wr_ch.phy_list_addr = phy_list_addr;
			writel(0x1, pp->dbi_base + PCIE_DMA_WRITE_ENGINE_EN);
			writel(0, pp->dbi_base + PCIE_DMA_WRITE_INT_MASK);
			writel(0x10000, pp->dbi_base +
			PCIE_DMA_WRITE_LINKED_LIST_ERR_EN);
			writel(0, pp->dbi_base + PCIE_DMA_VIEWPORT_SEL);
			writel(0x04000300, pp->dbi_base + PCIE_DMA_CH_CONTROL1);

			/* Set pointer to start of first list */
			dma_set_list_ptr(pp, direction, phy_list_addr);
			/* Ring doorbell */
			writel(0, pp->dbi_base + PCIE_DMA_WRITE_DOORBELL);
		} else {
			return -EBUSY;
		}
	} else {/* Read request */
		if (pp->rd_ch.status != DMA_CH_RUNNING) {
			/* Set ch_status */
			pp->rd_ch.status = DMA_CH_RUNNING;
			/* Set last data elem */
			dma_set_data_elem((u32 *)pp->rd_ch.virt_addr,
			pp->rd_ch.current_list_size - 1,
			pp->rd_ch.current_sar,
			pp->rd_ch.current_dar,  pp->rd_ch.current_size, 1);

			pp->rd_ch.errors = 0;
			/* Clear CR1 for proper init */
			writel(0x0, pp->dbi_base + PCIE_DMA_CH_CONTROL1);

			pp->rd_ch.phy_list_addr = phy_list_addr;
			writel(0x1, pp->dbi_base + PCIE_DMA_READ_ENGINE_EN);
			writel(0, pp->dbi_base + PCIE_DMA_READ_INT_MASK);
			writel(0x10000, pp->dbi_base +
				PCIE_DMA_READ_LINKED_LIST_ERR_EN);
			writel(0x80000000, pp->dbi_base +
				PCIE_DMA_VIEWPORT_SEL);
			writel(0x04000300, pp->dbi_base + PCIE_DMA_CH_CONTROL1);

			/* Set pointer to start of first list */
			dma_set_list_ptr(pp, direction, phy_list_addr);
			/* Ring doorbell */
			writel(0, pp->dbi_base + PCIE_DMA_READ_DOORBELL);
		} else {
			return -EBUSY;
		}
	}
	return 0;
}

int dw_pcie_dma_load_linked_list(struct pcie_port *pp,
	struct dma_list (*arr_ll)[], u8 arr_sz, u32 phy_list_addr,
	u32 next_phy_list_addr, u8 direction)
{
	u8 i;
	u32 *ptr_list;

	struct dma_ch_info *ptr_ch = (direction == DMA_CH_WRITE) ?
		&pp->wr_ch : &pp->rd_ch;
	ptr_list = (u32 *)(ioremap(phy_list_addr, SZ_1K));
	if (!ptr_list)
		return -EFAULT;

	arr_sz = pp->ll_info.nr_elem;
	pp->ll_info.phy_list_addr = phy_list_addr;

	for (i = 0 ; i < arr_sz ; i++) {
		dma_set_data_elem((u32 *)ptr_list, i,
			(*arr_ll)[i].sar, (*arr_ll)[i].dar,
			(*arr_ll)[i].size, 0);
		ptr_ch->current_sar = (*arr_ll)[i].sar;
		ptr_ch->current_dar = (*arr_ll)[i].dar;
		ptr_ch->current_size = (*arr_ll)[i].size;
		ptr_ch->current_elem_idx = i;
		ptr_ch->current_list_size = arr_sz;
		ptr_ch->virt_addr = ptr_list;
	}

	dma_set_link_elem(ptr_list, arr_sz, next_phy_list_addr);

	iounmap(ptr_list);
	return 0;
}
int dw_pcie_dma_single_rw(struct pcie_port *pp,
	struct dma_data_elem *dma_single_rw)
{
	u32 flags;
	struct dma_ch_info *ptr_ch;
	/* Invalid channel number */
	if (dma_single_rw->ch_num > S32V_PCIE_DMA_NR_CH - 1)
		return -EINVAL;

	/* Invalid transfer size */
	if (dma_single_rw->size > S32V_PCIE_DMA_MAX_SIZE)
		return -EINVAL;

	flags = dma_single_rw->flags;
	ptr_ch = (flags & DMA_FLAG_WRITE_ELEM) ?
		&pp->wr_ch : &pp->rd_ch;

	if (flags & DMA_FLAG_WRITE_ELEM) {
		if (pp->wr_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		pp->wr_ch.status = DMA_CH_RUNNING;
		pp->wr_ch.errors = 0;
		dw_pcie_dma_write_en(pp);
		dw_pcie_dma_set_viewport(pp, dma_single_rw->ch_num,
			DMA_CH_WRITE);
	} else {
		if (pp->rd_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		pp->rd_ch.status = DMA_CH_RUNNING;
		pp->rd_ch.errors = 0;
		dw_pcie_dma_read_en(pp);

		dw_pcie_dma_set_viewport(pp, dma_single_rw->ch_num,
			DMA_CH_READ);
	}

	/* Clear CR1 for proper init */
	writel(0x0, pp->dbi_base + PCIE_DMA_CH_CONTROL1);

	if (flags & (DMA_FLAG_EN_DONE_INT | DMA_FLAG_EN_ABORT_INT)) {
		dw_pcie_dma_en_local_int(pp);

		if (flags & (DMA_FLAG_RIE | DMA_FLAG_LIE)) {
			if (flags & DMA_FLAG_WRITE_ELEM) {
				dw_pcie_dma_set_wr_remote_abort_int(pp,
					dma_single_rw->imwr);
				dw_pcie_dma_set_wr_remote_done_int(pp,
					dma_single_rw->imwr);
				dw_pcie_dma_clear_wr_done_int_mask(pp,
					(1 << dma_single_rw->ch_num));
				dw_pcie_dma_clear_wr_abort_int_mask(pp,
					(1 << dma_single_rw->ch_num));
			} else if (flags & DMA_FLAG_READ_ELEM) {
				dw_pcie_dma_set_rd_remote_abort_int(pp,
					dma_single_rw->imwr);
				dw_pcie_dma_set_rd_remote_done_int(pp,
					dma_single_rw->imwr);
				dw_pcie_dma_clear_rd_done_int_mask(pp,
					(1 << dma_single_rw->ch_num));
				dw_pcie_dma_clear_rd_abort_int_mask(pp,
					(1 << dma_single_rw->ch_num));
			}
		}
	}
	/* Set transfer size */
	dw_pcie_dma_set_transfer_size(pp, dma_single_rw->size);
	/* Set SAR & DAR */
	dw_pcie_dma_set_sar(pp, dma_single_rw->sar);
	dw_pcie_dma_set_dar(pp, dma_single_rw->dar);

	if (flags & DMA_FLAG_WRITE_ELEM)
		writel(0, pp->dbi_base + PCIE_DMA_WRITE_DOORBELL);
	else
		writel(0, pp->dbi_base + PCIE_DMA_READ_DOORBELL);

	return 0;
}

void dw_pcie_dma_check_errors(struct pcie_port *pp,
	u32 direction, u32 *error)
{
	u32 val = 0;
	*error = DMA_ERR_NONE;

	if (direction == DMA_CH_WRITE) {
		dw_pcie_readl_rc(pp, PCIE_DMA_WRITE_ERR_STATUS, &val);
		if (val & 0x1)
			*error |= DMA_ERR_WR;
		if (val & 0x10000)
			*error |= DMA_ERR_FETCH_LL;
	} else {
		/* Get error status low */
		dw_pcie_readl_rc(pp, PCIE_DMA_READ_ERR_STATUS_LOW, &val);
		if (val & 0x1)
			*error |= DMA_ERR_RD;
		if (val & 0x10000)
			*error |= DMA_ERR_FETCH_LL;
		/* Get error status high */
		dw_pcie_readl_rc(pp, PCIE_DMA_READ_ERR_STATUS_HIGH, &val);
		if (val & 0x1)
			*error |= DMA_ERR_UNSUPPORTED_REQ;
		if (val & 0x100)
			*error |= DMA_ERR_CPL_ABORT;
		if (val & 0x10000)
			*error |= DMA_ERR_CPL_TIMEOUT;
		if (val & 0x1000000)
			*error |= DMA_ERR_DATA_POISIONING;
	}
}
irqreturn_t dw_handle_dma_irq(struct pcie_port *pp)
{
	u32 val_write = 0;
	u32 val_read = 0;
	u32 err_type = DMA_ERR_NONE;
	irqreturn_t ret = IRQ_HANDLED;

	dw_pcie_readl_rc(pp, PCIE_DMA_WRITE_INT_STATUS, &val_write);
	dw_pcie_readl_rc(pp, PCIE_DMA_READ_INT_STATUS, &val_read);

	if (val_write) {
		if (pp->wr_ch.status == DMA_CH_RUNNING) {
			if (val_write & 0x10000) { /* Abort interrupt */
				/* Get error type */
				dw_pcie_dma_check_errors(pp,
					DMA_FLAG_WRITE_ELEM,
					&pp->wr_ch.errors);
					writel(0x00FF0000, pp->dbi_base +
						PCIE_DMA_WRITE_INT_CLEAR);
				err_type = pp->wr_ch.errors;
			} else { /* Done interrupt */
				writel(0x000000FF, pp->dbi_base +
					PCIE_DMA_WRITE_INT_CLEAR);
				/* Check channel list mode */
			}
			pp->wr_ch.status = DMA_CH_STOPPED;
			#ifdef CONFIG_PCI_S32V234
			if (pp->ops->send_signal_to_user)
				pp->ops->send_signal_to_user(pp);
			#endif
		} else
			writel(0x00FF00FF, pp->dbi_base +
				PCIE_DMA_WRITE_INT_CLEAR);

		#ifdef CONFIG_PCI_S32V234
		if (pp->call_back)
			pp->call_back(val_write);
		#endif
	}
	if (val_read) {
		if (pp->rd_ch.status == DMA_CH_RUNNING) {
			/* Search interrupt type, abort or done */
			/* Abort interrupt */
			if (val_read & 0x80000) {
				/* Get error type */
				dw_pcie_dma_check_errors(pp, DMA_FLAG_READ_ELEM,
					&pp->rd_ch.errors);
				writel(0x00FF0000, pp->dbi_base +
						PCIE_DMA_READ_INT_CLEAR);
				err_type = pp->rd_ch.errors;
			} else { /* Done interrupt */
				writel(0x000000FF, pp->dbi_base +
					PCIE_DMA_READ_INT_CLEAR);
				/* Check channel list mode */
			}
			pp->rd_ch.status = DMA_CH_STOPPED;
			#ifdef CONFIG_PCI_S32V234
			if (pp->ops->send_signal_to_user)
				pp->ops->send_signal_to_user(pp);
			#endif
		} else
			writel(0x00FF00FF, pp->dbi_base +
				PCIE_DMA_READ_INT_CLEAR);
	}
	if (pp->ptr_func)
		pp->ptr_func(err_type);

	return ret;
}
#endif /* CONFIG_PCI_DW_DMA */

static struct irq_chip dw_msi_irq_chip = {
	.name = "PCI-MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

/* MSI int handler */
irqreturn_t dw_handle_msi_irq(struct pcie_port *pp)
{
	unsigned long val;
	int i, pos, irq;
	irqreturn_t ret = IRQ_NONE;

	for (i = 0; i < MAX_MSI_CTRLS; i++) {
		dw_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_STATUS + i * 12, 4,
				(u32 *)&val);
		if (val) {
			ret = IRQ_HANDLED;
			pos = 0;
			while ((pos = find_next_bit(&val, 32, pos)) != 32) {
				irq = irq_find_mapping(pp->irq_domain,
						i * 32 + pos);
				dw_pcie_wr_own_conf(pp,
						PCIE_MSI_INTR0_STATUS + i * 12,
						4, 1 << pos);
				generic_handle_irq(irq);
				pos++;
			}
		}
	}

	return ret;
}

void dw_pcie_msi_init(struct pcie_port *pp)
{
	pp->msi_data = __get_free_pages(GFP_KERNEL, 0);

	/* program the msi_data */
	dw_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_LO, 4,
			virt_to_phys((void *)pp->msi_data));
	dw_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_HI, 4, 0);
}

static void dw_pcie_msi_clear_irq(struct pcie_port *pp, int irq)
{
	unsigned int res, bit, val;

	res = (irq / 32) * 12;
	bit = irq % 32;
	dw_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, &val);
	val &= ~(1 << bit);
	dw_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, val);
}

static void clear_irq_range(struct pcie_port *pp, unsigned int irq_base,
			    unsigned int nvec, unsigned int pos)
{
	unsigned int i;

	for (i = 0; i < nvec; i++) {
		irq_set_msi_desc_off(irq_base, i, NULL);
		/* Disable corresponding interrupt on MSI controller */
		if (pp->ops->msi_clear_irq)
			pp->ops->msi_clear_irq(pp, pos + i);
		else
			dw_pcie_msi_clear_irq(pp, pos + i);
	}

	bitmap_release_region(pp->msi_irq_in_use, pos, order_base_2(nvec));
}

static void dw_pcie_msi_set_irq(struct pcie_port *pp, int irq)
{
	unsigned int res, bit, val;

	res = (irq / 32) * 12;
	bit = irq % 32;
	dw_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, &val);
	val |= 1 << bit;
	dw_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE + res, 4, val);
}

static int assign_irq(int no_irqs, struct msi_desc *desc, int *pos)
{
	int irq, pos0, i;
	struct pcie_port *pp = desc->dev->bus->sysdata;

	pos0 = bitmap_find_free_region(pp->msi_irq_in_use, MAX_MSI_IRQS,
				       order_base_2(no_irqs));
	if (pos0 < 0)
		goto no_valid_irq;

	irq = irq_find_mapping(pp->irq_domain, pos0);
	if (!irq)
		goto no_valid_irq;

	/*
	 * irq_create_mapping (called from dw_pcie_host_init) pre-allocates
	 * descs so there is no need to allocate descs here. We can therefore
	 * assume that if irq_find_mapping above returns non-zero, then the
	 * descs are also successfully allocated.
	 */

	for (i = 0; i < no_irqs; i++) {
		if (irq_set_msi_desc_off(irq, i, desc) != 0) {
			clear_irq_range(pp, irq, i, pos0);
			goto no_valid_irq;
		}
		/*Enable corresponding interrupt in MSI interrupt controller */
		if (pp->ops->msi_set_irq)
			pp->ops->msi_set_irq(pp, pos0 + i);
		else
			dw_pcie_msi_set_irq(pp, pos0 + i);
	}

	*pos = pos0;
	return irq;

no_valid_irq:
	*pos = pos0;
	return -ENOSPC;
}

static int dw_msi_setup_irq(struct msi_controller *chip, struct pci_dev *pdev,
			struct msi_desc *desc)
{
	int irq, pos;
	struct msi_msg msg;
	struct pcie_port *pp = pdev->bus->sysdata;

	if (desc->msi_attrib.is_msix)
		return -EINVAL;

	irq = assign_irq(1, desc, &pos);
	if (irq < 0)
		return irq;

	if (pp->ops->get_msi_addr)
		msg.address_lo = pp->ops->get_msi_addr(pp);
	else
		msg.address_lo = virt_to_phys((void *)pp->msi_data);
	msg.address_hi = 0x0;

	if (pp->ops->get_msi_data)
		msg.data = pp->ops->get_msi_data(pp, pos);
	else
		msg.data = pos;
#ifdef CONFIG_PCI_MSI
	pci_write_msi_msg(irq, &msg);
#endif
	return 0;
}

static void dw_msi_teardown_irq(struct msi_controller *chip, unsigned int irq)
{
	struct irq_data *data = irq_get_irq_data(irq);
	struct msi_desc *msi = irq_data_get_msi(data);
	struct pcie_port *pp = msi->dev->bus->sysdata;

	clear_irq_range(pp, irq, 1, data->hwirq);
}

static struct msi_controller dw_pcie_msi_chip = {
	.setup_irq = dw_msi_setup_irq,
	.teardown_irq = dw_msi_teardown_irq,
};

int dw_pcie_link_up(struct pcie_port *pp)
{
	if (pp->ops->link_up)
		return pp->ops->link_up(pp);
	else
		return 0;
}

static int dw_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
			irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dw_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	set_irq_flags(irq, IRQF_VALID);

	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = dw_pcie_msi_map,
};

int __init dw_pcie_host_init(struct pcie_port *pp)
{
	struct device_node *np = pp->dev->of_node;
	struct platform_device *pdev = to_platform_device(pp->dev);
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	struct pci_bus *bus;
	struct resource *cfg_res;
	LIST_HEAD(res);
	u32 val, na, ns;
	const __be32 *addrp;
	int i, index, ret;

	/* Find the address cell size and the number of cells in order to get
	 * the untranslated address.
	 */
	of_property_read_u32(np, "#address-cells", &na);
	ns = of_n_size_cells(np);

	cfg_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "config");
	if (cfg_res) {
		pp->cfg0_size = resource_size(cfg_res)/2;
		pp->cfg1_size = resource_size(cfg_res)/2;
		pp->cfg0_base = cfg_res->start;
		pp->cfg1_base = cfg_res->start + pp->cfg0_size;

		/* Find the untranslated configuration space address */
		index = of_property_match_string(np, "reg-names", "config");
		addrp = of_get_address(np, index, NULL, NULL);
		pp->cfg0_mod_base = of_read_number(addrp, ns);
		pp->cfg1_mod_base = pp->cfg0_mod_base + pp->cfg0_size;
	} else {
		dev_err(pp->dev, "missing *config* reg space\n");
	}

	if (of_pci_range_parser_init(&parser, np)) {
		dev_err(pp->dev, "missing ranges property\n");
		return -EINVAL;
	}

	/* Get the I/O and memory ranges from DT */
	for_each_of_pci_range(&parser, &range) {
		unsigned long restype = range.flags & IORESOURCE_TYPE_BITS;

		if (restype == IORESOURCE_IO) {
			of_pci_range_to_resource(&range, np, &pp->io);
			pp->io.name = "I/O";
			pp->io.start = max_t(resource_size_t,
					     PCIBIOS_MIN_IO,
					     range.pci_addr + global_io_offset);
			pp->io.end = min_t(resource_size_t,
					   IO_SPACE_LIMIT,
					   range.pci_addr + range.size
					   + global_io_offset - 1);
			pp->io_size = resource_size(&pp->io);
			pp->io_bus_addr = range.pci_addr;
			pp->io_base = range.cpu_addr;

			/* Find the untranslated IO space address */
			pp->io_mod_base = of_read_number(parser.range -
							 parser.np + na, ns);
		}
		if (restype == IORESOURCE_MEM) {
			of_pci_range_to_resource(&range, np, &pp->mem);
			pp->mem.name = "MEM";
			pp->mem_size = resource_size(&pp->mem);
			pp->mem_bus_addr = range.pci_addr;

			/* Find the untranslated MEM space address */
			pp->mem_mod_base = of_read_number(parser.range -
							  parser.np + na, ns);
		}
		if (restype == 0) {
			of_pci_range_to_resource(&range, np, &pp->cfg);
			pp->cfg0_size = resource_size(&pp->cfg)/2;
			pp->cfg1_size = resource_size(&pp->cfg)/2;
			pp->cfg0_base = pp->cfg.start;
			pp->cfg1_base = pp->cfg.start + pp->cfg0_size;

			/* Find the untranslated configuration space address */
			pp->cfg0_mod_base = of_read_number(parser.range -
							   parser.np + na, ns);
			pp->cfg1_mod_base = pp->cfg0_mod_base +
					    pp->cfg0_size;
		}
	}

	ret = of_pci_parse_bus_range(np, &pp->busn);
	if (ret < 0) {
		pp->busn.name = np->name;
		pp->busn.start = 0;
		pp->busn.end = 0xff;
		pp->busn.flags = IORESOURCE_BUS;
		dev_dbg(pp->dev, "failed to parse bus-range property: %d, using default %pR\n",
			ret, &pp->busn);
	}

	if (!pp->dbi_base) {
		pp->dbi_base = devm_ioremap(pp->dev, pp->cfg.start,
					resource_size(&pp->cfg));
		if (!pp->dbi_base) {
			dev_err(pp->dev, "error with ioremap\n");
			return -ENOMEM;
		}
	}

	pp->mem_base = pp->mem.start;

	if (!pp->va_cfg0_base) {
		pp->va_cfg0_base = devm_ioremap(pp->dev, pp->cfg0_base,
						pp->cfg0_size);
		if (!pp->va_cfg0_base) {
			dev_err(pp->dev, "error with ioremap in function\n");
			return -ENOMEM;
		}
	}

	if (!pp->va_cfg1_base) {
		pp->va_cfg1_base = devm_ioremap(pp->dev, pp->cfg1_base,
						pp->cfg1_size);
		if (!pp->va_cfg1_base) {
			dev_err(pp->dev, "error with ioremap\n");
			return -ENOMEM;
		}
	}

	if (of_property_read_u32(np, "num-lanes", &pp->lanes)) {
		dev_err(pp->dev, "Failed to parse the number of lanes\n");
		return -EINVAL;
	}
#ifdef CONFIG_PCI_MSI
	if (!pp->ops->msi_host_init) {
		pp->irq_domain = irq_domain_add_linear(pp->dev->of_node,
					MAX_MSI_IRQS, &msi_domain_ops,
					&dw_pcie_msi_chip);
		if (!pp->irq_domain) {
			dev_err(pp->dev, "irq domain init failed\n");
			return -ENXIO;
		}

		for (i = 0; i < MAX_MSI_IRQS; i++)
			irq_create_mapping(pp->irq_domain, i);
	} else {
		ret = pp->ops->msi_host_init(pp, &dw_pcie_msi_chip);
		if (ret < 0)
			return ret;
	}
#endif
	if (pp->ops->host_init)
		pp->ops->host_init(pp);

	dw_pcie_wr_own_conf(pp, PCI_BASE_ADDRESS_0, 4, 0);

	/* program correct class for RC */
	dw_pcie_wr_own_conf(pp, PCI_CLASS_DEVICE, 2, PCI_CLASS_BRIDGE_PCI);

	dw_pcie_rd_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, &val);
	val |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_wr_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, val);

#ifdef CONFIG_ARM
	if (global_io_offset < SZ_1M && pp->io_size > 0) {
		pci_ioremap_io(global_io_offset, pp->io_base);
		global_io_offset += SZ_64K;
		pci_add_resource_offset(&res, &pp->io,
					global_io_offset - pp->io_bus_addr);
	}
	pci_add_resource_offset(&res, &pp->mem,
				pp->mem.start - pp->mem_bus_addr);
	pci_add_resource(&res, &pp->busn);
#else
	ret = of_pci_get_host_bridge_resources(np, 0, 0xff, &res, &pp->io_base);
	if (ret)
		return ret;
#endif
	bus = pci_create_root_bus(pp->dev, pp->root_bus_nr, &dw_pcie_ops,
			      pp, &res);
	if (!bus)
		return -ENOMEM;

#ifdef CONFIG_PCI_MSI
	bus->msi = &dw_pcie_msi_chip;
#endif

	pci_scan_child_bus(bus);
	if (pp->ops->scan_bus)
		pp->ops->scan_bus(pp);

#ifdef CONFIG_ARM
	pci_fixup_irqs(pci_common_swizzle, of_irq_parse_and_map_pci);
#endif
	pci_assign_unassigned_bus_resources(bus);
	pci_bus_add_devices(bus);
	return 0;
}

static void dw_pcie_prog_viewport_cfg0(struct pcie_port *pp, u32 busdev)
{
	/* Program viewport : OUTBOUND : CFG0 */
	dw_pcie_writel_rc(pp, PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX0,
			  PCIE_ATU_VIEWPORT);
	dw_pcie_writel_rc(pp, pp->cfg0_mod_base, PCIE_ATU_LOWER_BASE);
	dw_pcie_writel_rc(pp, (pp->cfg0_mod_base >> 32), PCIE_ATU_UPPER_BASE);
	dw_pcie_writel_rc(pp, pp->cfg0_mod_base + pp->cfg0_size - 1,
			  PCIE_ATU_LIMIT);
	dw_pcie_writel_rc(pp, busdev, PCIE_ATU_LOWER_TARGET);
	dw_pcie_writel_rc(pp, 0, PCIE_ATU_UPPER_TARGET);
	dw_pcie_writel_rc(pp, PCIE_ATU_TYPE_CFG0, PCIE_ATU_CR1);
	dw_pcie_writel_rc(pp, PCIE_ATU_ENABLE, PCIE_ATU_CR2);
}

static void dw_pcie_prog_viewport_cfg1(struct pcie_port *pp, u32 busdev)
{
	/* Program viewport : OUTBOUND : CFG1 */
	dw_pcie_writel_rc(pp, PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX1,
			  PCIE_ATU_VIEWPORT);
	dw_pcie_writel_rc(pp, PCIE_ATU_TYPE_CFG1, PCIE_ATU_CR1);
	dw_pcie_writel_rc(pp, pp->cfg1_mod_base, PCIE_ATU_LOWER_BASE);
	dw_pcie_writel_rc(pp, (pp->cfg1_mod_base >> 32), PCIE_ATU_UPPER_BASE);
	dw_pcie_writel_rc(pp, pp->cfg1_mod_base + pp->cfg1_size - 1,
			  PCIE_ATU_LIMIT);
	dw_pcie_writel_rc(pp, busdev, PCIE_ATU_LOWER_TARGET);
	dw_pcie_writel_rc(pp, 0, PCIE_ATU_UPPER_TARGET);
	dw_pcie_writel_rc(pp, PCIE_ATU_ENABLE, PCIE_ATU_CR2);
}

static void dw_pcie_prog_viewport_mem_outbound(struct pcie_port *pp)
{
	/* Program viewport : OUTBOUND : MEM */
	dw_pcie_writel_rc(pp, PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX0,
			  PCIE_ATU_VIEWPORT);
	dw_pcie_writel_rc(pp, PCIE_ATU_TYPE_MEM, PCIE_ATU_CR1);
	dw_pcie_writel_rc(pp, pp->mem_mod_base, PCIE_ATU_LOWER_BASE);
	dw_pcie_writel_rc(pp, (pp->mem_mod_base >> 32), PCIE_ATU_UPPER_BASE);
	dw_pcie_writel_rc(pp, pp->mem_mod_base + pp->mem_size - 1,
			  PCIE_ATU_LIMIT);
	dw_pcie_writel_rc(pp, pp->mem_bus_addr, PCIE_ATU_LOWER_TARGET);
	dw_pcie_writel_rc(pp, upper_32_bits(pp->mem_bus_addr),
			  PCIE_ATU_UPPER_TARGET);
	dw_pcie_writel_rc(pp, PCIE_ATU_ENABLE, PCIE_ATU_CR2);
}

static void dw_pcie_prog_viewport_io_outbound(struct pcie_port *pp)
{
	/* Program viewport : OUTBOUND : IO */
	dw_pcie_writel_rc(pp, PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX1,
			  PCIE_ATU_VIEWPORT);
	dw_pcie_writel_rc(pp, PCIE_ATU_TYPE_IO, PCIE_ATU_CR1);
	dw_pcie_writel_rc(pp, pp->io_mod_base, PCIE_ATU_LOWER_BASE);
	dw_pcie_writel_rc(pp, (pp->io_mod_base >> 32), PCIE_ATU_UPPER_BASE);
	dw_pcie_writel_rc(pp, pp->io_mod_base + pp->io_size - 1,
			  PCIE_ATU_LIMIT);
	dw_pcie_writel_rc(pp, pp->io_bus_addr, PCIE_ATU_LOWER_TARGET);
	dw_pcie_writel_rc(pp, upper_32_bits(pp->io_bus_addr),
			  PCIE_ATU_UPPER_TARGET);
	dw_pcie_writel_rc(pp, PCIE_ATU_ENABLE, PCIE_ATU_CR2);
}

static int dw_pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 *val)
{
	int ret = PCIBIOS_SUCCESSFUL;
	u32 address, busdev;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));
	address = where & ~0x3;

	if (bus->parent->number == pp->root_bus_nr) {
		dw_pcie_prog_viewport_cfg0(pp, busdev);
		ret = dw_pcie_cfg_read(pp->va_cfg0_base + address, where, size,
				val);
			dw_pcie_prog_viewport_mem_outbound(pp);
	} else {
		dw_pcie_prog_viewport_cfg1(pp, busdev);
		ret = dw_pcie_cfg_read(pp->va_cfg1_base + address, where, size,
				val);
			dw_pcie_prog_viewport_io_outbound(pp);
	}

	return ret;
}

static int dw_pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 val)
{
	int ret = PCIBIOS_SUCCESSFUL;
	u32 address, busdev;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));
	address = where & ~0x3;

	if (bus->parent->number == pp->root_bus_nr) {
		dw_pcie_prog_viewport_cfg0(pp, busdev);
		ret = dw_pcie_cfg_write(pp->va_cfg0_base + address, where, size,
				val);
			dw_pcie_prog_viewport_mem_outbound(pp);
	} else {
		dw_pcie_prog_viewport_cfg1(pp, busdev);
		ret = dw_pcie_cfg_write(pp->va_cfg1_base + address, where, size,
				val);
			dw_pcie_prog_viewport_io_outbound(pp);
	}

	return ret;
}

static int dw_pcie_valid_config(struct pcie_port *pp,
				struct pci_bus *bus, int dev)
{
	/* If there is no link, then there is no device */
	if (bus->number != pp->root_bus_nr) {
		if (!dw_pcie_link_up(pp))
			return 0;
	}

	/* access only one slot on each root port */
	if (bus->number == pp->root_bus_nr && dev > 0)
		return 0;

	/*
	 * do not read more than one device on the bus directly attached
	 * to RC's (Virtual Bridge's) DS side.
	 */
	if (bus->primary == pp->root_bus_nr && dev > 0)
		return 0;

	return 1;
}

static int dw_pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
			int size, u32 *val)
{
	struct pcie_port *pp = bus->sysdata;
	int ret;

	if (dw_pcie_valid_config(pp, bus, PCI_SLOT(devfn)) == 0) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (bus->number != pp->root_bus_nr)
		if (pp->ops->rd_other_conf)
			ret = pp->ops->rd_other_conf(pp, bus, devfn,
						where, size, val);
		else
			ret = dw_pcie_rd_other_conf(pp, bus, devfn,
						where, size, val);
	else
		ret = dw_pcie_rd_own_conf(pp, where, size, val);

	return ret;
}

static int dw_pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			int where, int size, u32 val)
{
	struct pcie_port *pp = bus->sysdata;
	int ret;

	if (dw_pcie_valid_config(pp, bus, PCI_SLOT(devfn)) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number != pp->root_bus_nr)
		if (pp->ops->wr_other_conf)
			ret = pp->ops->wr_other_conf(pp, bus, devfn,
						where, size, val);
		else
			ret = dw_pcie_wr_other_conf(pp, bus, devfn,
						where, size, val);
	else
		ret = dw_pcie_wr_own_conf(pp, where, size, val);

	return ret;
}

static struct pci_ops dw_pcie_ops = {
	.read = dw_pcie_rd_conf,
	.write = dw_pcie_wr_conf,
};

void dw_pcie_setup_rc(struct pcie_port *pp)
{
	u32 val;
	u32 membase;
	u32 memlimit;

	/* set ATUs setting for MEM and IO */

	/* set the number of lanes */
	dw_pcie_readl_rc(pp, PCIE_PORT_LINK_CONTROL, &val);
	val &= ~PORT_LINK_MODE_MASK;
	switch (pp->lanes) {
	case 1:
		val |= PORT_LINK_MODE_1_LANES;
		break;
	case 2:
		val |= PORT_LINK_MODE_2_LANES;
		break;
	case 4:
		val |= PORT_LINK_MODE_4_LANES;
		break;
	}
	dw_pcie_writel_rc(pp, val, PCIE_PORT_LINK_CONTROL);

	/* set link width speed control register */
	dw_pcie_readl_rc(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, &val);
	val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
	switch (pp->lanes) {
	case 1:
		val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
		break;
	case 2:
		val |= PORT_LOGIC_LINK_WIDTH_2_LANES;
		break;
	case 4:
		val |= PORT_LOGIC_LINK_WIDTH_4_LANES;
		break;
	}
	dw_pcie_writel_rc(pp, val, PCIE_LINK_WIDTH_SPEED_CONTROL);

	/* setup RC BARs */
	dw_pcie_writel_rc(pp, 0x00000004, PCI_BASE_ADDRESS_0);
	dw_pcie_writel_rc(pp, 0x00000000, PCI_BASE_ADDRESS_1);

	/* setup interrupt pins */
	dw_pcie_readl_rc(pp, PCI_INTERRUPT_LINE, &val);
	val &= 0xffff00ff;
	val |= 0x00000100;
	dw_pcie_writel_rc(pp, val, PCI_INTERRUPT_LINE);

	/* setup bus numbers */
	dw_pcie_readl_rc(pp, PCI_PRIMARY_BUS, &val);
	val &= 0xff000000;
	val |= 0x00010100;
	dw_pcie_writel_rc(pp, val, PCI_PRIMARY_BUS);

	/* setup memory base, memory limit */
	membase = ((u32)pp->mem_base & 0xfff00000) >> 16;
	memlimit = (pp->mem_size + (u32)pp->mem_base) & 0xfff00000;
	val = memlimit | membase;
	dw_pcie_writel_rc(pp, val, PCI_MEMORY_BASE);

	/* setup command register */
	dw_pcie_readl_rc(pp, PCI_COMMAND, &val);
	val &= 0xffff0000;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
	dw_pcie_writel_rc(pp, val, PCI_COMMAND);
}

MODULE_AUTHOR("Jingoo Han <jg1.han@samsung.com>");
MODULE_DESCRIPTION("Designware PCIe host controller driver");
MODULE_LICENSE("GPL v2");
